#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/ioctl.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <trace/events/power.h>
#include <linux/oem/control_center.h>
#include <linux/oem/houston.h>
#include <linux/oem/aigov.h>

#ifdef CONFIG_OPCHAIN
#include <../coretech/uxcore/opchain_helper.h>
#include <../coretech/uxcore/opchain_define.h>
#endif

/* time measurement */
#define CC_TIME_START(start) { \
	if (cc_time_measure) \
		start = ktime_get(); \
}
#define CC_TIME_END(start, end, t, tmax) { \
	if (cc_time_measure) { \
		end = ktime_get(); \
		t = ktime_to_us(ktime_sub(end, begin)); \
		if (t > tmax) \
			tmax = t; \
		cc_logv("%s: cost: %lldus, max: %lldus\n", __func__, t, tmax); \
	}\
}
static bool cc_time_measure = true;
module_param_named(time_measure, cc_time_measure, bool, 0644);

/* boost enable options */
static bool cc_cpu_boost_enable = true;
module_param_named(cpu_boost_enable, cc_cpu_boost_enable, bool, 0644);

bool cc_ddr_boost_enable = true;
module_param_named(ddr_boost_enable, cc_ddr_boost_enable, bool, 0644);

//bool cc_ddr_lower_bound_enable = false;
//module_param_named(ddr_lower_bound_enable, cc_ddr_lower_bound_enable, bool, 0644);
//
//bool cc_ddr_set_enable = false;
//module_param_named(ddr_set_enable, cc_ddr_set_enable, bool, 0644);
//
///* FIXME
// * this is for voting, should not named as lock_
// */
//bool cc_ddr_lock_enable = true;
//module_param_named(ddr_lock_enable, cc_ddr_lock_enable, bool, 0644);

/* record */
static struct cc_record {
	spinlock_t lock;
	/* priority list */
	struct list_head phead[CC_PRIO_MAX];
} cc_record[CC_CTL_CATEGORY_MAX];

/*
 * verbose output
 * lv: 0 -> verbose
 * lv: 1 -> info
 * lv: 2 -> wraning
 * lv: 3 -> error
 */
static int cc_log_lv = 1;
module_param_named(log_lv, cc_log_lv, int, 0644);

/* ddr lock api */
extern void aop_lock_ddr_freq(int lv);

/* boost ts information */
static struct cc_boost_ts cbt[CC_BOOST_TS_SIZE];
static int boost_ts_idx = 0;
static DEFINE_SPINLOCK(boost_ts_lock);

/* calling with lock held */
static int boost_ts_get_idx(void) {
	int idx = boost_ts_idx++;
	return idx % CC_BOOST_TS_SIZE;
}

static void cc_boost_ts_update(struct cc_command* cc)
{
	u64 ts_us = ktime_to_us(ktime_get());
	int idx = 0;
	bool reset = cc->type == CC_CTL_TYPE_RESET || cc->type == CC_CTL_TYPE_RESET_NONBLOCK;

	if (!cc)
		return;

	if (cc->category != CC_CTL_CATEGORY_CLUS_1_FREQ)
		return;

	cc_logv("[%s] boost from %u group %u category %u type %u period %u min %llu max %llu\n",
		reset? "Exit": "Enter",
		cc->bind_leader? cc->leader: cc->pid,
		cc->group, cc->category, cc->type, cc->period_us, cc->params[0], cc->params[1]);

	spin_lock(&boost_ts_lock);

	idx = boost_ts_get_idx();
	cbt[idx].pid = cc->bind_leader? cc->leader: cc->pid;
	cbt[idx].type = reset? 0: 1;
	cbt[idx].ts_us = ts_us;
	cbt[idx].min = cc->params[0];
	cbt[idx].max = cc->params[1];
	spin_unlock(&boost_ts_lock);
}

void cc_boost_ts_collect(struct cc_boost_ts* source)
{
	spin_lock(&boost_ts_lock);
	memcpy(source, cbt, sizeof(struct cc_boost_ts) * CC_BOOST_TS_SIZE);
	memset(cbt, 0, sizeof(struct cc_boost_ts) * CC_BOOST_TS_SIZE);
	boost_ts_idx = 0;
	spin_unlock(&boost_ts_lock);
}

/* cpufreq boost qos */
enum cc_cpufreq_boost_lv {
	CC_CPUFREQ_BOOST_LV_0 = 0,
	CC_CPUFREQ_BOOST_LV_1,
	CC_CPUFREQ_BOOST_LV_2,
	CC_CPUFREQ_BOOST_LV_3,
	CC_CPUFREQ_BOOST_LV_4,

	CC_CPUFREQ_BOOST_LV_MAX
};

/* boost timestamp */

/* debug */
static bool old_version = false;
module_param_named(old_version, old_version, bool, 0644);

/* async work */
#define CC_ASYNC_RQ_MAX (64)
static struct cc_async_rq {
	struct cc_command cc;
	struct list_head node;
	struct work_struct work;
	int idx;
} cc_async_rq[CC_ASYNC_RQ_MAX];

static struct task_struct *cc_worker_task;
static struct list_head cc_request_list;
static struct list_head cc_pending_list;
static DEFINE_SPINLOCK(cc_async_lock);
static struct workqueue_struct *cc_wq;

extern void clk_get_ddr_freq(u64* val);
extern bool cc_is_nonblock(struct cc_command* cc);

static inline void cc_remove_nonblock(struct cc_command* cc)
{
	if (cc->type >= CC_CTL_TYPE_ONESHOT_NONBLOCK)
		cc->type -= CC_CTL_TYPE_ONESHOT_NONBLOCK;
}

static void __adjust_cpufreq(
	struct cpufreq_policy *pol, u32 min, u32 max, bool reset)
{
	u32 req_freq = pol->req_freq;
	u32 orig_req_freq = pol->req_freq;
	u32 next_freq = pol->req_freq;
	u32 cpu;

	spin_lock(&pol->cc_lock);

	/* quick check */
	if (pol->cc_max == max && pol->cc_min == min && !reset) {
		spin_unlock(&pol->cc_lock);
		goto out;
	}

	/* cc max/min always inside current pol->max/min */
	pol->cc_max = (pol->max >= max)? max: pol->max;
	pol->cc_min = (pol->min <= min)? min: pol->min;
	if (reset)
		req_freq = pol->req_freq;
	else
		req_freq = clamp_val(req_freq, pol->cc_min, pol->cc_max);

	spin_unlock(&pol->cc_lock);

	/* not update while current governor is not schedutil */
	if (unlikely(!pol->cc_enable))
		goto out;

	/* trigger frequency change */
	if (pol->fast_switch_enabled) {
		next_freq = cpufreq_driver_fast_switch(pol, req_freq);
		if (!next_freq || (next_freq == pol->cur))
			goto out;

		/* update cpufreq stat */
		pol->cur = next_freq;
		for_each_cpu(cpu, pol->cpus)
			trace_cpu_frequency(next_freq, cpu);
		cpufreq_stats_record_transition(pol, next_freq);
	} else {
		cpufreq_driver_target(pol, req_freq, CPUFREQ_RELATION_H);
	}
out:
	cc_logv("cc_max: %u, cc_min: %u, target: %u, orig: %u, cur: %u, gov: %d\n",
		pol->cc_max, pol->cc_min, req_freq, orig_req_freq, next_freq, pol->cc_enable);
}

/* called with get_online_cpus() */
static inline int cc_get_online_cpu(int start, int end)
{
	int idx = -1;
	for (idx = start; idx <= end; ++idx)
		if (cpu_online(idx))
			break;
	return idx;
}

static inline int cc_get_cpu_idx(int cluster)
{
	switch (cluster) {
	case 0: return cc_get_online_cpu(0, 3);
	case 1: return cc_get_online_cpu(4, 6);
	case 2: return cc_get_online_cpu(7, 7);
	}
	return -1;
}

static int __cc_adjust_cpufreq(
	u32 clus, u32 min, u32 max, bool reset)
{
	struct cpufreq_policy *pol;
	int idx;
	int ret = 0;

	get_online_cpus();

	idx = cc_get_cpu_idx(clus);
	if (idx == -1) {
		cc_logw("can' get cpu idx, input cluster %u\n", clus);
		ret = -1;
		goto out;
	}

	pol = cpufreq_cpu_get(idx);
	if (!pol) {
		ret = -1;
		cc_logw("can't get cluster %d cpufreqp policy\n", idx);
		goto out;
	}

	__adjust_cpufreq(pol, min, max, reset);

	cpufreq_cpu_put(pol);
out:
	put_online_cpus();
	return ret;
}

static void cc_adjust_cpufreq(struct cc_command* cc)
{
	u32 clus, min, max;
	bool reset = false;

	if (!cc_cpu_boost_enable)
		return;

#ifdef CONFIG_AIGOV
	if (aigov_hooked()) {
		switch (cc->category) {
		case CC_CTL_CATEGORY_CLUS_0_FREQ: clus = 0; break;
		case CC_CTL_CATEGORY_CLUS_1_FREQ: clus = 1; break;
		case CC_CTL_CATEGORY_CLUS_2_FREQ: clus = 2; break;
		}
		if (cc->type == CC_CTL_TYPE_RESET ||
			cc->type == CC_CTL_TYPE_RESET_NONBLOCK) {
			aigov_set_cpufreq(cc_get_cpu_idx(clus), 0);
		} else {
			aigov_set_cpufreq(cc_get_cpu_idx(clus), cc->params[1]);
		}
		return;
	}
#endif

	if (cc_is_nonblock(cc))
		return;

	switch (cc->category) {
	case CC_CTL_CATEGORY_CLUS_0_FREQ: clus = 0; break;
	case CC_CTL_CATEGORY_CLUS_1_FREQ: clus = 1; break;
	case CC_CTL_CATEGORY_CLUS_2_FREQ: clus = 2; break;
	default:
		cc_logw("cpufreq query invalid, category %u\n", cc->category);
		return;
	}

	if (cc->type == CC_CTL_TYPE_RESET) {
		min = 0;
		max = UINT_MAX;
		reset = true;
	} else {
		/* ONESHOT/PERIOD */
		min = cc->params[0];
		max = cc->params[1];
		/* validate parameters */
		if (min > max) {
			cc_logw("cpufrq incorrect, min %u, max %u\n", min, max);
			return;
		}
	}

	cc->status = __cc_adjust_cpufreq(clus, min, max, reset);
}

static int __cc_adjust_cpufreq_boost(
	u32 clus, u32 level)
{
	struct cpufreq_policy *pol;
	u32 cur;
	int idx;
	int ret = 0;

	get_online_cpus();

	idx = cc_get_cpu_idx(clus);
	if (idx == -1) {
		cc_logw("can' get cpu idx, input cluster %u\n", clus);
		ret = -1;
		goto out;
	}

	pol = cpufreq_cpu_get(idx);
	if (!pol) {
		ret = -1;
		cc_logw("can't get clus %d cpufreqp policy\n", idx);
		goto out;
	}

	/* scale up if needed */
	cur = pol->cur;
	switch (level) {
	case CC_CPUFREQ_BOOST_LV_0:
		cur = 0; break; // not scale
	case CC_CPUFREQ_BOOST_LV_1:
		cur = cur + (cur >> 2); break; // scale 1.25
	case CC_CPUFREQ_BOOST_LV_2:
		cur = cur + (cur >> 1); break; // scale 1.5
	case CC_CPUFREQ_BOOST_LV_3:
		cur = cur + (cur >> 2) + (cur >> 1); break; // scale 1.75
	case CC_CPUFREQ_BOOST_LV_4:
		cur = cur + cur; break; // scale 2.0
	case CC_CPUFREQ_BOOST_LV_MAX:
		cur = pol->max; break; // jump to max
	default: break; // no change
	}

	/* FIXME */
	__adjust_cpufreq(pol, cur, INT_MAX, false);

	cpufreq_cpu_put(pol);
out:
	put_online_cpus();
	return ret;
}

static inline u64 cc_ddr_to_devfreq(u64 val)
{
	int i;
	u64 ddr_devfreq_avail_freq[] = { 0, 2597, 2929, 3879, 5161, 5931, 6881, 7980 };
	u64 ddr_aop_mapping_freq[] = { 0, 681, 768, 1017, 1353, 1555, 1804, 2092 };

	/* map to devfreq whlie config is enabled */
	//if (cc_ddr_set_enable || cc_ddr_lock_enable) {
	//	for (i = ARRAY_SIZE(ddr_devfreq_avail_freq) - 1; i >= 0; --i) {
	//		if (val >= ddr_aop_mapping_freq[i])
	//			return ddr_devfreq_avail_freq[i];
	//	}
	//}
	for (i = ARRAY_SIZE(ddr_devfreq_avail_freq) - 1; i >= 0; --i) {
		if (val >= ddr_aop_mapping_freq[i])
			return ddr_devfreq_avail_freq[i];
	}
	return val;
}

//u64 cc_cpu_find_ddr(int cpu)
//{
//	int i, len, idx = 0;
//	u64 ddr, curr;
//	struct cpufreq_policy *pol;
//	u64 *tmp_cpu, *tmp_ddr;
//	u64 *ddr_cluster0_options;
//	u64 *ddr_cluster1_options;
//
//	u64 ddr_cluster0_vote_options[5] = {
//		762, 1720, 2086, 2929, 3879
//	};
//	u64 ddr_cluster1_vote_options[9] = {
//		762, 1720, 2086, 2929, 3879, 5161, 5931, 6881, 7980
//	};
//	u64 ddr_cluster0_lock_options[5] = {
//		200, 451, 547, 768, 1017
//	};
//	u64 ddr_cluster1_lock_options[9] = {
//		200, 451, 547, 768, 1017, 1353, 1555, 1804, 2092
//	};
//	u64 cpu_cluster0_options[5] = {
//		300000, 768000, 1113600, 1478400, 1632000
//	};
//	u64 cpu_cluster1_options[9] = {
//		300000, 710400, 825600, 1056000, 1286400, 1612800, 1804800, 2649600, 3000000
//	};
//
//	if (cc_ddr_set_enable || cc_ddr_lock_enable) {
//		ddr_cluster0_options = ddr_cluster0_vote_options;
//		ddr_cluster1_options = ddr_cluster1_vote_options;
//	} else {
//		ddr_cluster0_options = ddr_cluster0_lock_options;
//		ddr_cluster1_options = ddr_cluster1_lock_options;
//	}
//
//	pol = cpufreq_cpu_get(cpu);
//	if (unlikely(!pol))
//		return 0;
//	idx = (cpu > 3) ? 1 : 0;
//	curr = pol->cur;
//	if (idx) {
//		tmp_cpu = cpu_cluster1_options;
//		tmp_ddr = ddr_cluster1_options;
//		len = ARRAY_SIZE(cpu_cluster1_options);
//	} else {
//		tmp_cpu = cpu_cluster0_options;
//		tmp_ddr = ddr_cluster0_options;
//		len = ARRAY_SIZE(cpu_cluster0_options);
//	}
//	for (i = len - 1; i >= 0; --i) {
//		if (curr > tmp_cpu[i]) {
//			ddr = tmp_ddr[min(i+1, len - 1)];
//			break;
//		}
//	}
//	cpufreq_cpu_put(pol);
//	return ddr;
//}

static void cc_adjust_cpufreq_boost(struct cc_command* cc)
{
	u32 clus, level;

	if (!cc_cpu_boost_enable)
		return;

	if (cc_is_nonblock(cc))
		return;

	switch (cc->category) {
	case CC_CTL_CATEGORY_CLUS_0_FREQ: clus = 0; break;
	case CC_CTL_CATEGORY_CLUS_1_FREQ: clus = 1; break;
	case CC_CTL_CATEGORY_CLUS_2_FREQ: clus = 2; break;
	default:
		cc_logw("cpufreq query invalid, category %u\n", cc->category);
		return;
	}

	if (cc->type == CC_CTL_TYPE_RESET) {
		level = 0;
	} else
		level = cc->params[0];

	cc->status = __cc_adjust_cpufreq_boost(clus, level);
}

static void cc_query_cpufreq(struct cc_command* cc)
{
	struct cpufreq_policy *pol;
	u32 clus;
	int idx;

	get_online_cpus();

	switch (cc->category) {
	case CC_CTL_CATEGORY_CLUS_0_FREQ_QUERY: clus = 0; break;
	case CC_CTL_CATEGORY_CLUS_1_FREQ_QUERY: clus = 1; break;
	case CC_CTL_CATEGORY_CLUS_2_FREQ_QUERY: clus = 2; break;
	default:
		cc_logw("cpufreq query invalid, category %u\n", cc->category);
		goto out;
	}

	idx = cc_get_cpu_idx(clus);
	if (idx == -1) {
		cc_logw("can' get cpu idx, input cluster %u\n", clus);
		goto out;
	}

	pol = cpufreq_cpu_get(idx);
	if (!pol) {
		cc_logw("can't get cluster %d cpufreqp policy\n", idx);
		goto out;
	}
	cc->response = pol->cur;

out:
	put_online_cpus();
}

#define CC_DDRFREQ_CHECK(name, target) \
	if (!strcmp(name, target)) { \
		cc_logi("mark device %s as ddrfreq related\n", name); \
		return true; \
	}

bool cc_is_ddrfreq_related(const char* name)
{
	if (!unlikely(name))
		return false;

	/* ddrfreq voting device */
	//CC_DDRFREQ_CHECK(name, "soc:qcom,gpubw");
	CC_DDRFREQ_CHECK(name, "soc:qcom,cpu-llcc-ddr-bw");
	CC_DDRFREQ_CHECK(name, "soc:qcom,cpu4-cpu-ddr-latfloor");
	CC_DDRFREQ_CHECK(name, "soc:qcom,cpu0-llcc-ddr-lat");
	CC_DDRFREQ_CHECK(name, "soc:qcom,cpu4-llcc-ddr-lat");
	//CC_DDRFREQ_CHECK(name, "aa00000.qcom,vidc:arm9_bus_ddr");
	//CC_DDRFREQ_CHECK(name, "aa00000.qcom,vidc:venus_bus_ddr");
	return false;
}

static inline u64 query_ddrfreq(void)
{
	u64 val;
	clk_get_ddr_freq(&val);
	val /= 1000000;
	/* process for easy deal with */
	if (val == 1018) val = 1017;
	else if (val == 1355) val = 1353;
	else if (val == 1805) val = 1804;
	else if (val == 2096) val = 2092;
	return val;
}

static void cc_query_ddrfreq(struct cc_command* cc)
{
	cc->response = query_ddrfreq();
}

atomic_t cc_expect_ddrfreq;
#define CC_DDR_RESET_VAL 0
//static void cc_adjust_ddr_freq(struct cc_command *cc)
//{
//	u64 val = cc->params[0];
//	u64 cur;
//
//	if (!cc_ddr_boost_enable)
//		return;
//
//	val = cc_ddr_to_devfreq(val);
//
//	if (cc_is_nonblock(cc))
//		return;
//
//	if (cc_ddr_lower_bound_enable) {
//		val = max(cc_cpu_find_ddr(0), val);
//		val = max(cc_cpu_find_ddr(4), val);
//	}
//
//	if (cc->type == CC_CTL_TYPE_RESET)
//		val = CC_DDR_RESET_VAL;
//
//	/* FIXME
//	 * check cur & val not guarantee ddrfreq is locked or not */
//	if (cc_ddr_set_enable || cc_ddr_lock_enable) {
//		atomic_set(&cc_expect_ddrfreq, val);
//	} else {
//		/* check if need update */
//		cur = query_ddrfreq();
//
//		if (cur != val)
//			aop_lock_ddr_freq(val);
//	}
//}

static void cc_adjust_ddr_voting_freq(struct cc_command *cc)
{
	u64 val = cc->params[0];

	if (!cc_ddr_boost_enable)
		return;

	if (cc_is_nonblock(cc))
		return;

	val = cc_ddr_to_devfreq(val);

	if (cc->type == CC_CTL_TYPE_RESET)
		val = CC_DDR_RESET_VAL;

	atomic_set(&cc_expect_ddrfreq, val);
}

static void cc_adjust_ddr_lock_freq(struct cc_command *cc)
{
	u64 val = cc->params[0];
	u64 cur;

	if (!cc_ddr_boost_enable)
		return;

	if (cc_is_nonblock(cc))
		return;

	if (cc->type == CC_CTL_TYPE_RESET)
		val = CC_DDR_RESET_VAL;

	/* check if need update */
	cur = query_ddrfreq();

	if (cur != val)
		aop_lock_ddr_freq(val);
}

static void cc_adjust_sched(struct cc_command *cc)
{
	struct task_struct *task = NULL;
	pid_t pid = cc->params[0];

	if (cc_is_nonblock(cc))
		return;

#ifdef CONFIG_OPCHAIN
	if (cc->type == CC_CTL_TYPE_RESET) {
		opc_set_boost(0);
		return;
	}

	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (task) {
		cc_logv("set task %s %d to prime core\n", task->comm, task->pid);
		task->etask_claim = UT_PERF_TOP;
		opc_set_boost(1);
	} else
		cc_logw("can't find task %d\n", pid);
	rcu_read_unlock();
#endif
}

void cc_process(struct cc_command* cc)
{
	cc_logv("pid: %u, group: %u, category: %u, type: %u, params: %llu %llu %llu %llu\n",
		cc->pid, cc->group, cc->category, cc->type, cc->params[0], cc->params[1], cc->params[2], cc->params[3]);

	switch (cc->category) {
	case CC_CTL_CATEGORY_CLUS_0_FREQ:
		cc_logv("cpufreq: type: %u, cluster: 0 target: %llu\n", cc->type, cc->params[0]);
		cc_adjust_cpufreq(cc);
		break;
	case CC_CTL_CATEGORY_CLUS_1_FREQ:
		cc_logv("cpufreq: type: %u, cluster: 1 target: %llu\n", cc->type, cc->params[0]);
		cc_adjust_cpufreq(cc);
		break;
	case CC_CTL_CATEGORY_CLUS_2_FREQ:
		cc_logv("cpufreq: type: %u, cluster: 2 target: %llu\n", cc->type, cc->params[0]);
		cc_adjust_cpufreq(cc);
		break;
	case CC_CTL_CATEGORY_CPU_FREQ_BOOST:
		cc_logv("cpufreq_boost: type: %u, cluster: %llu target: %llu\n", cc->type, cc->params[0], cc->params[1]);
		cc_adjust_cpufreq_boost(cc);
		break;
	case CC_CTL_CATEGORY_DDR_VOTING_FREQ:
		cc_logv("ddrfreq voting: type: %u, target: %llu\n", cc->type, cc->params[0]);
		cc_adjust_ddr_voting_freq(cc);
		break;
	case CC_CTL_CATEGORY_DDR_LOCK_FREQ:
		cc_logv("ddrfreq lock: type: %u, target: %llu\n", cc->type, cc->params[0]);
		cc_adjust_ddr_lock_freq(cc);
		break;
	case CC_CTL_CATEGORY_SCHED_PRIME_BOOST:
		cc_logv("sched prime boost: type: %u, param: %llu\n", cc->type, cc->params[0]);
		cc_adjust_sched(cc);
		break;
	case CC_CTL_CATEGORY_CLUS_0_FREQ_QUERY:
		cc_query_cpufreq(cc);
		cc_logv("cpufreq query: type: %u, cluster: 0, freq: %llu\n", cc->type, cc->response);
		break;
	case CC_CTL_CATEGORY_CLUS_1_FREQ_QUERY:
		cc_query_cpufreq(cc);
		cc_logv("cpufreq query: type: %u, cluster: 1, freq: %llu\n", cc->type, cc->response);
		break;
	case CC_CTL_CATEGORY_CLUS_2_FREQ_QUERY:
		cc_query_cpufreq(cc);
		cc_logv("cpufreq query: type: %u, cluster: 2, freq: %llu\n", cc->type, cc->response);
		break;
	case CC_CTL_CATEGORY_DDR_FREQ_QUERY:
		cc_query_ddrfreq(cc);
		cc_logv("ddrfreq query: type: %u, freq: %llu\n", cc->type, cc->response);
		break;
	default:
		cc_logw("category %d not support\n", cc->category);
		break;
	}
}

static inline void dump_cc(struct cc_command *cc, const char* func, const char* msg)
{
	cc_logv("%s: %s: pid: %d, period_us: %u, prio: %u, group: %u, category: %u, type: %u, [0]: %llu, [1]: %llu, [2]: %llu, [3]: %llu, response: %llu, bind_leader: %d, status: %d\n",
		func, msg,
		cc->pid, cc->period_us, cc->prio, cc->group, cc->category,
		cc->type, cc->params[0], cc->params[1], cc->params[2],
		cc->params[3], cc->response, cc->bind_leader, cc->status);
}

static inline struct cc_command* find_highest_cc_nolock(int category)
{
	struct cc_tsk_data* data = NULL;
	struct cc_command *cc = NULL;
	int prio;

	/* find the highest priority request to perform */
	for (prio = CC_PRIO_HIGH; !cc && prio < CC_PRIO_MAX; ++prio) {
		if (!list_empty(&cc_record[category].phead[prio])) {
			list_for_each_entry(data, &cc_record[category].phead[prio], node) {
				cc = &data->cc;
				break;
			}
		}
	}
	return cc;
}

/* find the highest priority request to perform */
static struct cc_command* find_highest_cc(int category)
{
	struct cc_command* cc;

	spin_lock(&cc_record[category].lock);
	cc = find_highest_cc_nolock(category);
	spin_unlock(&cc_record[category].lock);
	return cc;
}


static void cc_record_acq(int category, struct cc_command* cc)
{
	struct cc_command *high_cc = find_highest_cc(category);

	dump_cc(cc, __func__, "current request");
	if (high_cc) {
		dump_cc(high_cc, __func__, "highest request");
	} else {
		cc_logw("%s: can't find any request\n", __func__);
		return;
	}

	/*
	 * apply change
	 * if high_cc not equal to cc, it should be applied earlier
	 */
	if (high_cc == cc)
		cc_process(high_cc);
}

static void cc_record_rel(int category, struct cc_command *cc)
{
	struct cc_command* next_cc = find_highest_cc(category);
	bool is_nonblock = cc->type >= CC_CTL_TYPE_ONESHOT_NONBLOCK;

	/* update reset type */
	cc->type = is_nonblock? CC_CTL_TYPE_RESET_NONBLOCK: CC_CTL_TYPE_RESET;
	if (next_cc) {
		/* apply next since we detach the highest before */
		cc_logv("got pending request, re-apply\n");
		dump_cc(next_cc, __func__, "next request");
		cc_process(next_cc);
	} else {
		/* no request pending, reset finally */
		cc_logv("no pending request, release\n");
		dump_cc(cc, __func__, "reset request");
		cc_process(cc);
	}
}

static void cc_record_init(void)
{
	int i, j;

	/* init cc_record */
	for (i = 0; i < CC_CTL_CATEGORY_MAX; ++i) {
		/* assign acquire and release */
		spin_lock_init(&cc_record[i].lock);
		for (j = 0; j < CC_PRIO_MAX; ++j)
			INIT_LIST_HEAD(&cc_record[i].phead[j]);
	}
}

static void cc_tsk_acq(struct cc_tsk_data* data)
{
	struct cc_command *cc;
	u32 delay_us;
	u32 category;
	int prio;

	/* update boost ts */
	cc_boost_ts_update(cc);

	current->cc_enable = true;

	/* add into cc_record */
	/* TODO check category & prio value */
	category = data->cc.category;
	prio = data->cc.prio;
	cc = &data->cc;
	delay_us = cc->period_us;

	dump_cc(cc, __func__, "current request");

	/* if already inside list, detach first */
	spin_lock(&cc_record[category].lock);
	if (!list_empty(&data->node)) {
		/* cancel queued delayed work first */
		cancel_delayed_work(&data->dwork);
		list_del_init(&data->node);
		dump_cc(cc, __func__, "[detach]");
	}
	list_add(&data->node, &cc_record[category].phead[prio]);
	dump_cc(cc, __func__, "[attach]");
	spin_unlock(&cc_record[category].lock);

	/* trigger system control */
	cc_record_acq(category, cc);

	/* queue delay work for release */
	queue_delayed_work(cc_wq, &data->dwork, usecs_to_jiffies(delay_us));
}

static void cc_tsk_rel(struct cc_tsk_data* data)
{
	struct cc_command* cc = &data->cc;
	struct cc_command* high_cc;
	u32 category = cc->category;

	/* update boost ts */
	cc_boost_ts_update(cc);

	/* detach first */
	dump_cc(cc, __func__, "current request");

	spin_lock(&cc_record[category].lock);
	high_cc = find_highest_cc_nolock(category);
	/* detach first */
	if (!list_empty(&data->node)) {
		cancel_delayed_work(&data->dwork);
		list_del_init(&data->node);
		dump_cc(cc, __func__, "[detach]");
	} else {
		cc_logv("try to detach, but already detached\n");
	}

	if (cc != high_cc) {
		/* no need to worry, just detach and return */
		spin_unlock(&cc_record[category].lock);
		return;
	}
	spin_unlock(&cc_record[category].lock);

	/* trigger system control */
	cc_record_rel(category, cc);
}

static void cc_delay_rel(struct work_struct *work)
{
	struct cc_tsk_data* data = container_of(work, struct cc_tsk_data, dwork.work);
	struct cc_command* cc = &data->cc;

	/* delay work no need to use nonblock call */
	cc->type = CC_CTL_TYPE_RESET;
	cc_tsk_rel(data);
}

static struct cc_tsk_data* cc_init_ctd(void)
{
	struct cc_tsk_data *ctd = NULL;
	int i = 0;

	ctd = kzalloc(sizeof(struct cc_tsk_data) * CC_CTL_CATEGORY_MAX, GFP_KERNEL);
	if (!ctd)
		return NULL;

	for (i = 0; i < CC_CTL_CATEGORY_MAX; ++i) {
		/* init all category control */
		INIT_LIST_HEAD(&ctd[i].node);
		INIT_DELAYED_WORK(&ctd[i].dwork, cc_delay_rel);
	}
	return ctd;
}

static inline struct cc_command* get_tsk_cc(bool bind_leader, u32 category)
{
	struct task_struct* task = bind_leader? current->group_leader: current;

	/* FIXME may be race */
	/* init ctd */
	if (!task->ctd) {
		task->ctd = cc_init_ctd();
		if (!task->ctd) {
			cc_loge("task %s(%d) cc_tsk_data init failed\n", task->comm, task->pid);
			return NULL;
		}
		cc_logv("%s: pid: %s(%d) init ctd successful\n",
				__func__, task->comm, task->pid);
	}

	return &task->ctd[category].cc;
}

static inline struct cc_tsk_data* get_tsk_data(bool bind_leader, u32 category)
{
	struct task_struct* task = bind_leader? current->group_leader: current;
	return &task->ctd[category];
}

static inline int cc_tsk_copy(struct cc_command* cc, bool copy_to_user)
{
	u32 category = cc->category;
	/* TODO dynamic allocate later */
	struct cc_command* tskcc = get_tsk_cc(cc->bind_leader, category);

	if (!tskcc)
		return -1;

	if (copy_to_user)
		memcpy(cc, tskcc, sizeof(struct cc_command));
	else
		memcpy(tskcc, cc, sizeof(struct cc_command));

	return 0;
}

void cc_tsk_process(struct cc_command* cc)
{
	u32 type = cc->type;
	u32 category = cc->category;

	/* query can return first */
	if (category >= CC_CTL_CATEGORY_CLUS_0_FREQ_QUERY) {
		cc_process(cc);
		return;
	}

	/* copy cc */
	if (cc_tsk_copy(cc, false))
		return;

	if (type == CC_CTL_TYPE_RESET ||
		type == CC_CTL_TYPE_RESET_NONBLOCK)
		cc_tsk_rel(get_tsk_data(cc->bind_leader, category));
	else
		cc_tsk_acq(get_tsk_data(cc->bind_leader, category));

	/* copy back to userspace cc */
	cc_tsk_copy(cc, true);
}

/* for fork and exit, use void* to avoid include sched.h in control_center.h */
void cc_tsk_init(void* ptr)
{
	struct task_struct *task = (struct task_struct*) ptr;

	task->cc_enable = false;
	task->ctd = NULL;
}

void cc_tsk_free(void* ptr)
{
	struct task_struct *task = (struct task_struct*) ptr;
	struct cc_tsk_data *data = task->ctd;
	u32 category;

	if (!task->cc_enable)
		return;

	if (!task->ctd)
		return;

	task->cc_enable = false;

	/* TODO free and reset. If needed */
	/* detach all */
	for (category = 0; category < CC_CTL_CATEGORY_MAX; ++category) {
		bool need_free = false;
		cc_logv("%s: pid: %s(%d) free category %d\n",
			__func__, task->comm, task->pid, category);
		spin_lock(&cc_record[category].lock);
		if (!list_empty(&data[category].node)) {
			need_free = true;
			list_del_init(&data[category].node);
			dump_cc(&data[category].cc, __func__, "[detach]");
		}
		spin_unlock(&cc_record[category].lock);

		if (need_free) {
			cc_logv("%s: pid: %s(%d) free category %d, need update.\n",
				__func__, task->comm, task->pid, category);
			cancel_delayed_work_sync(&data[category].dwork);
			/* since we're going to free ctd, we need to force set type to blocked version */
			data[category].cc.type = CC_CTL_TYPE_RESET;

			cc_record_rel(category, &data[category].cc);
		}
	}
}

static int cc_ctl_show(struct seq_file *m, void *v)
{
	seq_printf(m, "control center version: %s\n", CC_CTL_VERSION);
	return 0;
}

static int cc_ctl_open(struct inode *ip, struct file *fp)
{
	cc_logv("opened by %s %d\n", current->comm, current->pid);
	return single_open(fp, cc_ctl_show, NULL);;
}

static int cc_ctl_close(struct inode *ip, struct file *fp)
{
	cc_logv("closed by %s %d\n", current->comm, current->pid);
	return 0;
}

static long cc_ctl_ioctl(struct file *file, unsigned int cmd, unsigned long __user arg)
{
	ktime_t begin, end;
	s64 t;
	static s64 tmax = 0;

	if (_IOC_TYPE(cmd) != CC_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > CC_IOC_MAX) return -ENOTTY;

	CC_TIME_START(begin);

	cc_logv("%s: cmd: %u, arg: %lu\n", __func__, CC_IOC_COMMAND, arg);
	switch (cmd) {
	case CC_IOC_COMMAND:
		{
			struct cc_command cc;
			if (copy_from_user(&cc, (struct cc_command *) arg, sizeof(struct cc_command)))
				goto err_out;

			if (old_version)
				cc_process(&cc);
			else
				cc_tsk_process(&cc);

			if (copy_to_user((struct cc_command *) arg, &cc, sizeof(struct cc_command)))
				goto err_out;
		}
	}

	CC_TIME_END(begin, end, t, tmax);
	return 0;

err_out:
	CC_TIME_END(begin, end, t, tmax);
	return 0;
}

static const struct file_operations cc_ctl_fops = {
	.owner = THIS_MODULE,
	.open = cc_ctl_open,
	.release = cc_ctl_close,
	.unlocked_ioctl = cc_ctl_ioctl,
	.compat_ioctl = cc_ctl_ioctl,

	.read = seq_read,
	.llseek = seq_lseek,
};

/* TODO try to simplify the register flow */
static dev_t cc_ctl_dev;
static struct class *driver_class;
static struct cdev cdev;
static int cc_cdev_init(void)
{
	int rc;
	struct device *class_dev;

	rc = alloc_chrdev_region(&cc_ctl_dev, 0, 1, CC_CTL_NODE);
	if (rc < 0) {
		cc_loge("alloc_chrdev_region failed %d\n", rc);
		return 0;
	}

	driver_class = class_create(THIS_MODULE, CC_CTL_NODE);
	if (IS_ERR(driver_class)) {
		rc = -ENOMEM;
		cc_loge("class_create failed %d\n", rc);
		goto exit_unreg_chrdev_region;
	}
	class_dev = device_create(driver_class, NULL, cc_ctl_dev, NULL, CC_CTL_NODE);
	if (IS_ERR(class_dev)) {
		cc_loge("class_device_create failed %d\n", rc);
		rc = -ENOMEM;
		goto exit_destroy_class;
	}
	cdev_init(&cdev, &cc_ctl_fops);
	cdev.owner = THIS_MODULE;
	rc = cdev_add(&cdev, MKDEV(MAJOR(cc_ctl_dev), 0), 1);
	if (rc < 0) {
		cc_loge("cdev_add failed %d\n", rc);
		goto exit_destroy_device;
	}
	return 0;
exit_destroy_device:
	device_destroy(driver_class, cc_ctl_dev);
exit_destroy_class:
	class_destroy(driver_class);
exit_unreg_chrdev_region:
	unregister_chrdev_region(cc_ctl_dev, 1);
	return 0;
}

static struct cc_async_rq* cc_get_rq(struct list_head* head)
{
	struct cc_async_rq *rq = NULL;

	spin_lock(&cc_async_lock);
	if (!list_empty(head)) {
		list_for_each_entry(rq, head, node) {
			list_del_init(&rq->node);
			break;
		}
	}
	spin_unlock(&cc_async_lock);

	return rq;
}

static void __cc_attach_rq(struct cc_async_rq *rq, struct list_head* head)
{
	spin_lock(&cc_async_lock);
	list_add(&rq->node, head);
	spin_unlock(&cc_async_lock);
}

static void cc_release_rq(struct cc_async_rq* rq, struct list_head* head)
{
	/* clean before release */
	memset(&rq->cc, 0, sizeof (struct cc_command));
	__cc_attach_rq(rq, head);
}

static void __cc_queue_rq(struct cc_async_rq* rq, struct list_head* head)
{
	__cc_attach_rq(rq, head);
}

static void cc_work(struct work_struct *work)
{
	/* time related */
	ktime_t begin, end;
	s64 t;
	static s64 tmax = 0;

	struct cc_async_rq* rq =
		container_of(work, struct cc_async_rq, work);

	CC_TIME_START(begin);

	/* main loop */
	cc_process(&rq->cc);

	cc_release_rq(rq, &cc_request_list);

	CC_TIME_END(begin, end, t, tmax);
}

static int cc_worker(void* arg)
{
	ktime_t begin, end;
	s64 t;
	static s64 tmax = 0;

	/* perform async system resousrce adjustment */
	while (!kthread_should_stop()) {
		struct cc_async_rq *rq;

		CC_TIME_START(begin);
redo:
		rq = cc_get_rq(&cc_pending_list);
		if (!rq) {
			goto finish;
		}
		/* main loop */
		cc_process(&rq->cc);

		cc_release_rq(rq, &cc_request_list);
		goto redo;

finish:
		CC_TIME_END(begin, end, t, tmax);

		/* sleep for next wake up */
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
	}
	return 0;
}

static void cc_queue_rq(struct cc_command *cc)
{
	struct cc_async_rq *rq = cc_get_rq(&cc_request_list);
	if (!rq) {
		cc_logw("rq not enough\n");
		return;
	}

	memcpy(&rq->cc, cc, sizeof(struct cc_command));

	if (likely(cc_wq)) {
		/* if support workqueue, using workqueue */
		queue_work(cc_wq, &rq->work);
	} else if (likely(cc_worker_task)) {
		/* if support worker, using worker */
		__cc_queue_rq(rq, &cc_pending_list);
		wake_up_process(cc_worker_task);
	} else {
		/* fall back to original version */
		cc_logw_ratelimited("cc command fall back\n");
		cc_process(&rq->cc);
		cc_release_rq(rq, &cc_request_list);
	}
}

bool cc_is_nonblock(struct cc_command* cc)
{
	bool nonblock = false;
	if (cc->type >= CC_CTL_TYPE_ONESHOT_NONBLOCK) {
		nonblock = true;
		cc->type -= CC_CTL_TYPE_ONESHOT_NONBLOCK;
		cc_queue_rq(cc);
	}
	return nonblock;
}

static void cc_worker_init(void)
{
	int i;

	/* init for request/ pending/ lock */
	INIT_LIST_HEAD(&cc_request_list);
	INIT_LIST_HEAD(&cc_pending_list);

	/* init requests */
	for (i = 0; i < CC_ASYNC_RQ_MAX; ++i) {
		INIT_LIST_HEAD(&cc_async_rq[i].node);
		INIT_WORK(&cc_async_rq[i].work, cc_work);
		cc_async_rq[i].idx = i;
		spin_lock(&cc_async_lock);
		list_add(&cc_async_rq[i].node, &cc_request_list);
		spin_unlock(&cc_async_lock);
	}

	cc_worker_task = kthread_run(cc_worker, NULL, "cc_worker");
	if (IS_ERR(cc_worker_task))
		cc_loge("cc_worker create failed\n");

	cc_wq = alloc_ordered_workqueue("cc_wq", 0);
	if (!cc_wq)
		cc_loge("alloc work queue fail\n");
}

static int cc_dump_list_show(char *buf, const struct kernel_param *kp)
{
	int cnt = 0;
	int size = 0;
	struct cc_async_rq *rq;

	spin_lock(&cc_async_lock);

	/* request list */
	size = 0;
	list_for_each_entry(rq, &cc_request_list, node) {
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "%d ", rq->idx);
		++size;
	}
	if (size)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\n", rq->idx);
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "request list: size: %d\n", size);

	/* pending list */
	size = 0;
	list_for_each_entry(rq, &cc_pending_list, node) {
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "%d ", rq->idx);
		++size;
	}
	if (size)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\n", rq->idx);
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "pending list: size: %d\n", size);

	spin_unlock(&cc_async_lock);

	return cnt;
}

static struct kernel_param_ops cc_dump_list_ops = {
	.get = cc_dump_list_show,
};
module_param_cb(dump_list, &cc_dump_list_ops, NULL, 0644);

static int cc_dump_status_show(char *buf, const struct kernel_param *kp)
{
	struct cpufreq_policy *pol;
	int cnt = 0;
	int i, idx;
	u64 val;

	/* dump cpufreq control status */
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "cpufreq:\n");
	for (i = 0; i < CLUSTER_NUM; ++i) {
		idx = cc_get_cpu_idx(i);
		if (idx == -1) {
			cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "cluster %d offline\n", i);
			continue;
		}
		pol = cpufreq_cpu_get(idx);
		if (!pol) {
			cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "cluster %d can't get policy\n", i);
			continue;
		}
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "cluster %d min %u max %u cur %u, cc_min %u cc_max %u\n",
			i, pol->min, pol->max, pol->cur, pol->cc_min, pol->cc_max);
		cpufreq_cpu_put(pol);
	}

	/* dump ddrfreq control status */
	val = query_ddrfreq();
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "ddrfreq: %llu\n", val);
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "expected ddrfreq: %lu\n", atomic_read(&cc_expect_ddrfreq));
	return cnt;
}

static struct kernel_param_ops cc_dump_status_ops = {
	.get = cc_dump_status_show,
};
module_param_cb(dump_status, &cc_dump_status_ops, NULL, 0644);

static int cc_dump_record_show(char *buf, const struct kernel_param *kp)
{
	struct cc_tsk_data* data;
	const char* tag;
	u32 prio;
	int cnt = 0;
	int i;

	for (i = 0; i < CC_CTL_CATEGORY_MAX; ++i) {
		/* ignore query part */
		if (i >= CC_CTL_CATEGORY_CLUS_0_FREQ_QUERY)
			break;

		spin_lock(&cc_record[i].lock);
		switch (i) {
		case CC_CTL_CATEGORY_CLUS_0_FREQ: tag = "cpufreq_0:"; break;
		case CC_CTL_CATEGORY_CLUS_1_FREQ: tag = "cpufreq_1:"; break;
		case CC_CTL_CATEGORY_CLUS_2_FREQ: tag = "cpufreq_2:"; break;
		case CC_CTL_CATEGORY_CPU_FREQ_BOOST: tag = "cpufreq_boost:"; break;
		case CC_CTL_CATEGORY_DDR_VOTING_FREQ:
			tag = "ddrfreq voting:";
			break;
		case CC_CTL_CATEGORY_DDR_LOCK_FREQ:
			tag = "ddrfreq lock:";
			break;
		case CC_CTL_CATEGORY_SCHED_PRIME_BOOST: tag = "sched_prime_boost:"; break;
		case CC_CTL_CATEGORY_CLUS_0_FREQ_QUERY: tag = "cpufreq_0_query:"; break;
		case CC_CTL_CATEGORY_CLUS_1_FREQ_QUERY: tag = "cpufreq_1_query:"; break;
		case CC_CTL_CATEGORY_CLUS_2_FREQ_QUERY: tag = "cpufreq_2_query:"; break;
		case CC_CTL_CATEGORY_DDR_FREQ_QUERY: tag = "ddrfreq_query:"; break;
		}
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "%s\n", tag);
		for (prio = 0; prio < CC_PRIO_MAX; ++prio) {
			cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "  p[%d]:\n", prio);
			list_for_each_entry(data, &cc_record[i].phead[prio], node) {
				struct cc_command* cc = &data->cc;
				cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
					"    pid: %d, period_us: %u, prio: %u, group: %u, category: %u"
					", type: %u, [0]: %llu, [1]: %llu, [2]: %llu, [3]: %llu"
					", response: %llu, status: %d\n",
					cc->pid, cc->period_us, cc->prio, cc->group, cc->category,
					cc->type, cc->params[0], cc->params[1], cc->params[2],
					cc->params[3], cc->response, cc->status);
			}
		}
		spin_unlock(&cc_record[i].lock);
	}
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "cnt: %d\n", cnt);
	return cnt;
}

static struct kernel_param_ops cc_dump_record_ops = {
	.get = cc_dump_record_show,
};
module_param_cb(dump_record, &cc_dump_record_ops, NULL, 0644);

static const struct file_operations cc_ctl_proc_fops = {
	.owner = THIS_MODULE,
	.open = cc_ctl_open,
	.release = cc_ctl_close,
	.unlocked_ioctl = cc_ctl_ioctl,
	.compat_ioctl = cc_ctl_ioctl,

	.read = seq_read,
	.llseek = seq_lseek,
};

static inline void cc_proc_init(void)
{
	proc_create(CC_CTL_NODE, S_IFREG | 0660, NULL, &cc_ctl_proc_fops);
}

static int cc_init(void)
{
	/* FIXME
	 * remove later, so far just for compatible
	 */
	cc_cdev_init(); // create /dev/cc_ctl

	cc_proc_init(); // create /proc/cc_ctl
	cc_record_init();
	cc_worker_init();
	cc_logi("control center inited\n");
	return 0;
}

pure_initcall(cc_init);
