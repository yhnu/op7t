#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/tick.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/freezer.h>
#include <linux/wait.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/ioctl.h>
#include <linux/oem/houston.h>
#include <linux/perf_event.h>
#include <linux/cdev.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/jiffies.h>

#ifdef CONFIG_AIGOV
#include <linux/oem/aigov.h>
#endif

#include "../drivers/gpu/msm/kgsl.h"
#include "../drivers/gpu/msm/kgsl_pwrctrl.h"

#include <oneplus/houston/houston_helper.h>

#include <linux/smp.h>

#ifdef CONFIG_OPCHAIN
#include <../coretech/uxcore/opchain_helper.h>
#endif

#ifdef CONFIG_CONTROL_CENTER
#include <linux/oem/control_center.h>
#endif

/* perf raw counter */
#define ARMV8_PMCR_MASK 0x3f
#define ARMV8_PMCR_E (1 << 0) /* Enable all counters */
#define ARMV8_PMCR_C (1 << 2) /* Cycle counter reset */
#define ARMV8_PMCR_LC (1 << 6) /* Cycle Counter 64bit overflow */

/* Need to align housotn.h HT_MONITOR_SIZE */
static const char *ht_monitor_case[HT_MONITOR_SIZE] = {
	"ts",
	"clus_0_min", "clus_0_cur", "clus_0_max", "clus_0_iso",
	"clus_1_min", "clus_1_cur", "clus_1_max", "clus_1_iso",
	"clus_2_min", "clus_2_cur", "clus_2_max", "clus_2_iso",
	"gpu_cur", "voltage_now", "current_now", "hw_instruction",
	"hw_cache_miss", "hw_cycle",
	"cpu-0-0-usr", "cpu-0-1-usr", "cpu-0-2-usr", "cpu-0-3-usr",
	"cpu-1-0-usr", "cpu-1-1-usr", "cpu-1-2-usr", "cpu-1-3-usr",
	"cpu-1-4-usr", "cpu-1-5-usr", "cpu-1-6-usr", "cpu-1-7-usr",
	"skin-therm", "skin-msm-therm",
	"util-0", "util-1", "util-2", "util-3", "util-4",
	"util-5", "util-6", "util-7",
	"process name", "layer name", "pid", "fps_align", "actualFps",
	"predictFps", "appSwapTime", "appSwapDuration",
	"appEnqueueDuration", "sfTotalDuration", "sfPresentTime",
	"Vsync", "missedLayer", "render_pid", "render_util",
	"nt_rtg", "rtg_util_sum"
};

/*
 * log output
 * lv == 0 -> verbose info warning error
 * lv == 1 -> info warning error
 * lv == 2 -> wraning error
 * lv >= 3 -> error
 */
static int ht_log_lv = 1;
module_param_named(log_lv, ht_log_lv, int, 0664);

/* ais */
static int ais_enable = 0;
module_param_named(ais_enable, ais_enable, int, 0664);

/* pmu */
static int perf_ready = -1;

/* perf notify */
static struct ai_parcel parcel;
static struct workqueue_struct *ht_perf_workq;

static DEFINE_SPINLOCK(ht_perf_lock);
static DECLARE_WAIT_QUEUE_HEAD(ht_perf_waitq);
static DECLARE_WAIT_QUEUE_HEAD(ht_poll_waitq);

/* render & rtg util*/
static pid_t RenPid = -1;
static int get_util(bool isRender, int *num);
/*
 * perf event list
 * A list chained with task which perf event created
 */
static DEFINE_SPINLOCK(ht_perf_event_lock);
static struct list_head ht_perf_event_head = LIST_HEAD_INIT(ht_perf_event_head);

/* RTG (related thread group) */
static DEFINE_SPINLOCK(ht_rtg_lock);
static struct list_head ht_rtg_head = LIST_HEAD_INIT(ht_rtg_head);

/*
 * tmp list for storing rtg tasks
 * when traverse rtg list, we need hold spin lock, but in this context
 * we can't collect perf data (might sleep), so we need to use another
 * list to store these tasks, and then collect perf data in safe context.
 */
static struct list_head ht_rtg_perf_head = LIST_HEAD_INIT(ht_rtg_perf_head);

/* report skin_temp to ais */
static unsigned int thermal_update_period_hz = 100;
module_param_named(thermal_update_period_hz, thermal_update_period_hz, uint, 0664);

/*
 * filter mechanism
 * base_util: rtg task util threshold
 * rtg_filter_cnt: rtg task called cnt threshold under 1 sec
 */
static unsigned int base_util = 100;
module_param_named(base_util, base_util, uint, 0664);
static unsigned int rtg_filter_cnt = 10;
module_param_named(rtg_filter_cnt, rtg_filter_cnt, uint, 0664);

/* sched */
extern unsigned long long task_sched_runtime(struct task_struct *p);

/* fps boost info */
static atomic_t boost_cnt = ATOMIC_INIT(0);

/* fps tag to align other dump report */
static atomic64_t fps_align_ns;

/* cpuload tracking */
/* TODO these info maybe useless to sufraceflinger, should remove later */
struct cpuload_info {
	int cnt;
	int cmin;
	int cmax;
	int sum;
	long long iowait_min;
	long long iowait_max;
	long long iowait_sum;
};
static long long ht_iowait[8] = {0};
static long long ht_delta_iowait[8] = {0};
static bool cpuload_query = false;
module_param_named(cpuload_query, cpuload_query, bool, 0664);

/* battery query, it takes time to query */
static bool bat_query = false;
module_param_named(bat_query, bat_query, bool, 0664);

static bool bat_sample_high_resolution = false;
module_param_named(bat_sample_high_resolution, bat_sample_high_resolution, bool, 0664);

/* force update battery current */
static unsigned long bat_update_period_us = 1000000; // 1 sec
module_param_named(bat_update_period_us, bat_update_period_us, ulong, 0664);

extern void bq27541_force_update_current(void);

/* ioctl retry count */
#define HT_IOCTL_RETRY_MAX 128
static int ht_ioctl_retry_count = 0;
module_param_named(ioctl_retry_count, ht_ioctl_retry_count, int, 0664);

/* brain status */
static bool ht_brain_active = true;
module_param_named(brain_active, ht_brain_active, bool, 0664);

/* fps boost switch */
static bool fps_boost_enable = true;
module_param_named(fps_boost_enable, fps_boost_enable, bool, 0664);

/* freq hispeed */
static bool cpufreq_hispeed_enable = false;
module_param_named(cpufreq_hispeed_enable, cpufreq_hispeed_enable, bool, 0664);

static unsigned int cpufreq_hispeed[HT_CLUSTERS] = { 1209600, 1612800, 1612800 };
module_param_array_named(cpufreq_hispeed, cpufreq_hispeed, uint, NULL, 0664);

static bool ddrfreq_hispeed_enable = true;
module_param_named(ddrfreq_hispeed_enable, ddrfreq_hispeed_enable, bool, 0664);

static unsigned int ddrfreq_hispeed = 1017;
module_param_named(ddrfreq_hispeed, ddrfreq_hispeed, uint, 0664);

/* choose boost freq to lock or lower bound */
static unsigned int fps_boost_type = 1;
module_param_named(fps_boost_type, fps_boost_type, uint, 0664);

/* filter out too close boost hint */
static unsigned long fps_boost_filter_us = 8000;
module_param_named(fps_boost_filter_us, fps_boost_filter_us, ulong, 0664);

/* houston monitor
 * data: sample data
 * layer: sample data for frame info
 * process: sample data for frame process info
 */
struct sample_data {
	u64 data[MAX_REPORT_PERIOD][HT_MONITOR_SIZE];
	char layer[MAX_REPORT_PERIOD][FPS_LAYER_LEN];
	char process[MAX_REPORT_PERIOD][FPS_PROCESS_NAME_LEN];
};

struct ht_monitor {
	struct power_supply *psy;
	struct thermal_zone_device* tzd[HT_MONITOR_SIZE];
	struct task_struct *thread;
	struct sample_data *buf;
} monitor = {
	.psy = NULL,
	.thread = NULL,
	.buf = NULL,
};

struct ht_util_pol {
	unsigned long *utils[HT_CPUS_PER_CLUS];
	unsigned long *hi_util;
};

/* monitor switch */
static unsigned int ht_enable = 0;

/* mask only allow within 64 events */
static unsigned long ht_all_mask = 0;

static unsigned long filter_mask = 0;
module_param_named(filter_mask, filter_mask, ulong, 0664);

static unsigned long disable_mask = 0;
module_param_named(disable_mask, disable_mask, ulong, 0664);

static unsigned int report_div[HT_MONITOR_SIZE];
module_param_array_named(div, report_div, uint, NULL, 0664);

/*
 * monitor configuration
 * sidx: current used idx (should be update only by monitor thread)
 * record_cnt: current recorded sample amount
 * cached_fps: to record current efps and fps info
 * cached_layer_name: to record layer name. (debug purpose)
 * ht_tzd_idx: thermal zone index
 * gpwe: saved kgsl ptr, to get gpu freq
 * sample_rate: sample rate in ms
 * ht_utils: saved util prt, update from sugov
 * keep_alive: monitor life cycle
 */
static int sidx;
static unsigned int record_cnt = 0;

static atomic_t cached_fps[2];
/*ignore pass layer name to aischeduler*/
//static char cached_layer_name[FPS_CACHE_LAYER_LEN] = {0};

static int ht_tzd_idx = HT_CPU_0;
static struct kgsl_pwrctrl *gpwr;
static unsigned int sample_rate = 3000;
static struct ht_util_pol ht_utils[HT_CLUSTERS];

static bool __read_mostly keep_alive = false;

static dev_t ht_ctl_dev;
static struct class *driver_class;
static struct cdev cdev;

/* helper */
static inline int cpu_to_clus(int cpu)
{
	switch (cpu) {
	case 0: case 1: case 2: case 3: return 0;
	case 4: case 5: case 6: return 1;
	case 7: return 2;
	}
	return 0;
}

static inline int clus_to_cpu(int clus)
{
	switch (clus) {
	case 0: return CLUS_0_IDX;
	case 1: return CLUS_1_IDX;
	case 2: return CLUS_2_IDX;
	}
	return CLUS_0_IDX;
}

static inline u64 ddr_find_target(u64 target) {
	int i;
	u64 ddr_options[11] = {
		200, 300, 451, 547, 681, 768, 1017, 1353, 1555, 1804, 2092
	};

	for (i = 10; i >= 0; --i) {
		if (target >= ddr_options[i]) {
			target = ddr_options[i];
			break;
		}
	}
	return target;
}

static inline void ht_query_ddrfreq(u64* val)
{
	clk_get_ddr_freq(val);

	*val /= 1000000;
	/* process for easy deal with */
	if (*val == 1018) *val = 1017;
	else if (*val == 1355) *val = 1353;
	else if (*val == 1805) *val = 1804;
	else if (*val == 2096) *val = 2092;
}

static inline int ht_next_sample_idx(void)
{
	++sidx;
	sidx %= MAX_REPORT_PERIOD;

	return sidx;
}

static inline void ht_set_all_mask(void)
{
	int i;

	for (i = 0; i < HT_MONITOR_SIZE; ++i)
		ht_all_mask |= (1L << i);
}

static inline bool ht_is_all_disabled(unsigned long mask)
{
	return ht_all_mask == mask;
}

static inline bool ht_is_all_filtered(unsigned long mask)
{
	return ht_all_mask == mask;
}

static inline int ht_mapping_tags(char *name)
{
	int i;

	for (i = 0; i < HT_MONITOR_SIZE; ++i)
		if (!strcmp(name, ht_monitor_case[i]))
			return i;

	return HT_MONITOR_SIZE;
}

static inline const char* ht_ioctl_str(unsigned int cmd)
{
	switch (cmd) {
	case HT_IOC_COLLECT: return "HT_IOC_COLLECT";
	case HT_IOC_SCHEDSTAT: return "HT_IOC_SCHEDSTAT";
	case HT_IOC_CPU_LOAD: return "HT_IOC_CPU_LOAD";
	}
	return "NONE";
}

static inline int ht_get_temp(int monitor_idx)
{
	int temp = 0;

	if (unlikely(!monitor.tzd[monitor_idx]))
		return 0;

	if (disable_mask & (1 << monitor_idx))
		return 0;

	if (thermal_zone_get_temp(monitor.tzd[monitor_idx], &temp)) {
		ht_logv("failed to read out thermal zone with idx %d\n", monitor.tzd[monitor_idx]->id);
		return 0;
	}

	return temp;
}

static inline void ht_update_battery(void)
{
	static u64 prev = 0;
	u64 cur = ktime_to_us(ktime_get());

	if (cur - prev >= bat_update_period_us) {
		if (bat_sample_high_resolution)
			bq27541_force_update_current();
		ht_logv("force update battery info\n");
		prev = cur;
	} else if (prev > cur) {
		prev = cur;
		ht_logv("fix update battery timestamp\n");
	}
}

static inline u64 ht_get_iowait_time(int cpu)
{
	u64 iowait, iowait_usecs = -1ULL;

	if (cpu_online(cpu))
		iowait_usecs = get_cpu_iowait_time_us(cpu, NULL);

	if (iowait_usecs == -1ULL)
		/* !NO_HZ or cpu offline so we can rely on cpustat.iowait */
		iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
	else
		iowait = iowait_usecs * NSEC_PER_USEC;

	return iowait;
}

/* offline included */
static inline int ht_iso_count(const cpumask_t *mask)
{
	cpumask_t count_mask = CPU_MASK_NONE;

	cpumask_complement(&count_mask, cpu_online_mask);
	cpumask_or(&count_mask, &count_mask, cpu_isolated_mask);
	cpumask_and(&count_mask, &count_mask, mask);

	return cpumask_weight(&count_mask);
}

/* sched switch update */
static inline void ht_sched_update(struct task_struct *task, bool in)
{
	u64 now = 0;

	spin_lock(&task->rtg_lock);
	asm volatile("isb;mrs %0, pmccntr_el0" : "=r"(now));
	if (in) {
		task->run_ts = task->end_ts = now;
	} else {
		task->acc_run_ts += now - task->run_ts;
		task->end_ts = now;
	}
	spin_unlock(&task->rtg_lock);
}

static inline u32 armv8pmu_pmcr_read(void)
{
	u64 val = 0;
	asm volatile("mrs %0, pmcr_el0" : "=r" (val));
	return (u32)val;
}

static inline void armv8pmu_pmcr_write(u32 val)
{
	val &= ARMV8_PMCR_MASK;
	isb();
	asm volatile("msr pmcr_el0, %0" : : "r" ((u64)val));
}

static void enable_cpu_counters(void* data)
{
	armv8pmu_pmcr_write(armv8pmu_pmcr_read() | ARMV8_PMCR_LC | ARMV8_PMCR_E | ARMV8_PMCR_C);
	ht_logi("CPU:%d enable counter\n", smp_processor_id());
}

static unsigned int ht_get_temp_delay(int idx)
{
	static unsigned long next[HT_MONITOR_SIZE] = {0};
	static unsigned int temps[HT_MONITOR_SIZE] = {0};

	/* only allow for reading sensor data */
	if (unlikely(idx < HT_CPU_0 || idx > HT_THERM_1))
		return 0;

	/* update */
	if (jiffies > next[idx] && jiffies - next[idx] > thermal_update_period_hz) {
		next[idx] = jiffies;
		temps[idx] = ht_get_temp(idx);
	}

	if (jiffies < next[idx]) {
		next[idx] = jiffies;
		temps[idx] = ht_get_temp(idx);
	}

	return temps[idx];
}

/*
 * boost cpufreq while no ais activated
 * boost_target[0] : pid
 * boost_target[1] : tid
 */
#ifdef CONFIG_CONTROL_CENTER
static int boost_target[FPS_TARGET_NUM] = {0};
#endif

void ht_rtg_init(struct task_struct *task)
{
	task->rtg_ts = 0;
	INIT_LIST_HEAD(&task->rtg_node);
	INIT_LIST_HEAD(&task->rtg_perf_node);
	spin_lock_init(&task->rtg_lock);
}

static int perf_ready_store(const char *buf, const struct kernel_param *kp)
{
	int val;
	LIST_HEAD(release_pending);
	struct task_struct* task, *next;

	if (sscanf(buf, "%d\n", &val) <= 0)
		return 0;

	rcu_read_lock();
	spin_lock(&ht_rtg_lock);

	/* clean up */
	if (perf_ready != val) {
		struct task_struct *task, *next;
		list_for_each_entry_safe(task, next, &ht_rtg_head, rtg_node) {
			ht_logv("release task %s(%d) from rtg list\n", task->comm, task->pid);
			list_del_init(&task->rtg_node);
			list_add(&task->perf_node, &release_pending);
			get_task_struct(task);
		}

		spin_lock(&ht_perf_event_lock);
		list_for_each_entry_safe(task, next, &ht_perf_event_head, ht_perf_event_node) {
			ht_logv("release task %s(%d) from perf list\n", task->comm, task->pid);
			list_del_init(&task->ht_perf_event_node);
			if (list_empty(&task->perf_node)) {
				list_add(&task->perf_node, &release_pending);
				get_task_struct(task);
			}
		}
		spin_unlock(&ht_perf_event_lock);
	}
	perf_ready = val;
	spin_unlock(&ht_rtg_lock);
	rcu_read_unlock();

	/* release perf event */
	list_for_each_entry_safe(task, next, &release_pending, perf_node) {
		ht_logv("release task %s(%d) from pending list\n", task->comm, task->pid);
		ht_perf_event_release(task);
		list_del_init(&task->perf_node);
		put_task_struct(task);
	}
	return 0;
}

static int perf_ready_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", perf_ready);
}

static struct kernel_param_ops perf_ready_ops = {
	.set = perf_ready_store,
	.get = perf_ready_show,
};
module_param_cb(perf_ready, &perf_ready_ops, NULL, 0664);

static inline void __ht_perf_event_enable(struct task_struct *task, int event_id)
{
	if (!(task->perf_activate & (1 << event_id))) {
		bool status = ht_perf_event_open(task->pid, event_id);
		if (status) {
			/* Track tasks which have created perf events */
			spin_lock(&ht_perf_event_lock);
			if (list_empty(&task->ht_perf_event_node))
				list_add_tail(&task->ht_perf_event_node, &ht_perf_event_head);
			spin_unlock(&ht_perf_event_lock);
		}
		ht_logv("perf event create %s. task %s %d id %d\n", status? "successful": "failed", task->comm, task->pid, event_id);
	}
}

static int ht_track_events[] = {
	//HT_PERF_COUNT_CPU_CYCLES,
	HT_PERF_COUNT_INSTRUCTIONS,
	//HT_PERF_COUNT_CACHE_MISSES_L1,
	HT_PERF_COUNT_CACHE_MISSES_L2,
	HT_PERF_COUNT_CACHE_MISSES_L3
};

static inline void ht_perf_event_enable(struct task_struct *task)
{
	int i = 0;
	for (i = 0; i < sizeof(ht_track_events)/ sizeof(int); ++i)
		__ht_perf_event_enable(task, ht_track_events[i]);
}

static inline void __ht_perf_event_read(struct task_struct *task, struct ai_thread_parcel* t, int event_id)
{
	u64 val;
	if (!(task->perf_activate & (1 << event_id)))
		return;

	/* read pmu counter value */
	val = ht_perf_read(task, event_id);

	/* sanity check */
	if (task->perf_counters[event_id] > val) {
		ht_logw("counter value warning: task %d %s, id: %d, old: %llu, cur: %llu\n",
			task->pid, task->comm, event_id, task->perf_counters[event_id], val);
	}
	task->perf_counters[event_id] = val;

	switch (event_id) {
	case HT_PERF_COUNT_CPU_CYCLES:
		t->cycle = val; break;
	case HT_PERF_COUNT_INSTRUCTIONS:
		t->inst = val; break;
	case HT_PERF_COUNT_CACHE_MISSES_L1:
		t->cache_miss_l1 = val; break;
	case HT_PERF_COUNT_CACHE_MISSES_L2:
		t->cache_miss_l2 = val; break;
	case HT_PERF_COUNT_CACHE_MISSES_L3:
		t->cache_miss_l3 = val; break;
	}
}

static inline void ht_perf_event_read(struct task_struct *task, struct ai_thread_parcel* t)
{
	int i = 0;
	for (i = 0; i < sizeof(ht_track_events)/ sizeof(int); ++i)
		__ht_perf_event_read(task, t, ht_track_events[i]);
}

static inline void ht_collect_parcel_data(
	struct task_struct *task,
	struct ai_parcel* parcel,
	int pidx,
	bool is_enqueue_task)
{
	unsigned int cur_cpufreq = 0;
	u64 delta = 0;

	parcel->thread_amount += 1;
	parcel->t[pidx].tid = task->pid;

	if (is_enqueue_task)
		ht_perf_event_read(task, &parcel->t[pidx]);

	switch (task->cpu) {
	case 0: case 1: case 2: case 3: cur_cpufreq = parcel->cpu_cur_freq_0; break;
	case 4: case 5: case 6: cur_cpufreq = parcel->cpu_cur_freq_1; break;
	case 7: cur_cpufreq = parcel->cpu_cur_freq_2; break;
	}

	/* overflow workaround */
	if (task->total_run_ts >= task->delta_ts &&
		task->total_run_ts - task->delta_ts < 0xffffffff00000000)
		delta = task->total_run_ts - task->delta_ts;

	if (is_enqueue_task) {
		parcel->t[pidx].exec_time_ns = cur_cpufreq? (delta * 1000000LL / cur_cpufreq): 0;
		parcel->t[pidx].cycle = task->total_run_ts;
	} else {
		/* compare enqueue ts to check if rtg task presented at previous frame */
		unsigned long long cur_schedstat = task_sched_runtime(task);
		parcel->t[pidx].exec_time_ns = (parcel->prev_queued_ts_us == task->prev_ts_us)?
			(cur_schedstat - task->prev_schedstat): 0;
		task->prev_ts_us = parcel->queued_ts_us;
		task->prev_schedstat = cur_schedstat;
	}
}

void ht_collect_perf_data(struct work_struct *work)
{
	struct task_struct *task = container_of(work, struct task_struct, perf_work);
	int pidx = 0;
	struct task_struct *rtg_task, *next;

	if (unlikely(!task))
		return;

	get_task_struct(task);

	/* step 1. update enqueue timestamp */
	parcel.prev_queued_ts_us = parcel.queued_ts_us;
	parcel.queued_ts_us = task->enqueue_ts;

	/* step 2. init need perf hw events */
	ht_perf_event_enable(task);

	/* step 3. collect perf data */
	if (!spin_trylock(&ht_perf_lock)) {
		put_task_struct(task);
		return;
	}

	parcel.pid = task->tgid;
	parcel.cpu = task->cpu;
	parcel.clus = cpu_to_clus(task->cpu);
	parcel.thread_amount = 0;

	/* RTG */
	rcu_read_lock();
	spin_lock(&ht_rtg_lock);
	list_for_each_entry_safe(rtg_task, next, &ht_rtg_head, rtg_node) {
		/* enqueue task mayibe also a RTG member, skip it! */
		if (rtg_task == task)
			continue;

		/*
		 * evict rotten rtg tasks
		 * 1. not enter longer than threshold
		 * 2. util not reach the target
		 * 3. enter frequency not higher than threshold
		 */
		if (parcel.queued_ts_us - rtg_task->rtg_ts >= 1000000 /* 1 sec */ ||
				rtg_task->ravg.demand_scaled < base_util ||
				rtg_task->rtg_peak < rtg_filter_cnt) {
			list_del_init(&rtg_task->rtg_node);
			continue;
		}

		/* create perf event */
		ht_logv(
				"rtg_task: comm: %s, pid: %d, tgid: %d, fcnt: %u, fpeak: %u, rtg_cnt: %lu, rtg_peak: %lu, ts: %lld\n",
				rtg_task->comm, rtg_task->pid, rtg_task->tgid,
				rtg_task->f_cnt, rtg_task->f_peak,
				rtg_task->rtg_cnt, rtg_task->rtg_peak,
				rtg_task->enqueue_ts);

		/* add to perf list head to create perf event */
		get_task_struct(rtg_task);
		list_add_tail(&rtg_task->rtg_perf_node, &ht_rtg_perf_head);
	}
	spin_unlock(&ht_rtg_lock);
	rcu_read_unlock();

	spin_unlock(&ht_perf_lock);

	/* collect enqueue task perf data */
	ht_collect_parcel_data(task, &parcel, pidx++, true);

	list_for_each_entry_safe(rtg_task, next, &ht_rtg_perf_head, rtg_perf_node) {
		/* RTG tasks no need to collect perf data */
		if (rtg_task != task && pidx < AI_THREAD_PARCEL_MAX)
			ht_collect_parcel_data(rtg_task, &parcel, pidx++, false);
		list_del_init(&rtg_task->rtg_perf_node);
		put_task_struct(rtg_task);
	}

	/* notify ai_scheduler that data collected */
	if (likely(ht_brain_active))
		wake_up(&ht_perf_waitq);
	put_task_struct(task);
}

static int ht_enable_store(const char *buf, const struct kernel_param *kp)
{
	unsigned int val;

	if (sscanf(buf, "%u\n", &val) <= 0)
		return -EINVAL;

	ht_enable = !!val;

	if (ht_enable) {
		if (!monitor.buf) {
			ht_logi("try to init sample buffer\n");
			monitor.buf = vzalloc(sizeof(struct sample_data));
			if (monitor.buf)
				ht_logi("sample buffer inited\n");
			else {
				ht_loge("can't init sample buffer, set enable state to 0\n");
				ht_enable = 0;
				return 0;
			}
		}

		ht_logi("wake up monitor thread\n");
		wake_up_process(monitor.thread);
	}

	return 0;
}

static int ht_enable_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", ht_enable);
}

static struct kernel_param_ops ht_enable_ops = {
	.set = ht_enable_store,
	.get = ht_enable_show,
};
module_param_cb(ht_enable, &ht_enable_ops, NULL, 0664);

static int sample_rate_store(const char *buf, const struct kernel_param *kp)
{
	unsigned int val;

	if (sscanf(buf, "%u\n", &val) <= 0)
		return -EINVAL;

	sample_rate = val < MIN_POLLING_VAL? MIN_POLLING_VAL: val;
	return 0;
}

static int sample_rate_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", sample_rate);
}

static struct kernel_param_ops sample_rate_ops = {
	.set = sample_rate_store,
	.get = sample_rate_show,
};
module_param_cb(sample_rate_ms, &sample_rate_ops, NULL, 0664);

#ifdef CONFIG_CONTROL_CENTER
static inline void do_cpufreq_boost_helper(
	unsigned int cpu,
	unsigned long val,
	unsigned int period_us,
	unsigned int *orig_freq,
	unsigned int *cur_freq)
{
	struct cpufreq_policy *pol;
	struct cc_command cc;
	unsigned int cur, orig;
	int idx = 0;
	int category = 0;

	pol = cpufreq_cpu_get(cpu);
	if (unlikely(!pol))
		return;

	idx = cpu_to_clus(cpu);

	orig = cur = pol->cur;

	switch (cpu) {
	case CLUS_0_IDX: category = CC_CTL_CATEGORY_CLUS_0_FREQ; break;
	case CLUS_1_IDX: category = CC_CTL_CATEGORY_CLUS_1_FREQ; break;
	case CLUS_2_IDX: category = CC_CTL_CATEGORY_CLUS_2_FREQ; break;
	}

	memset(&cc, 0, sizeof(struct cc_command));
	cc.pid = current->pid;
	cc.prio = CC_PRIO_HIGH;
	cc.period_us = period_us;
	cc.group = CC_CTL_GROUP_GRAPHIC;
	cc.category = category;
	cc.response = 0;
	cc.leader = current->tgid;
	cc.bind_leader = true;
	cc.status = 0;
	cc.type = CC_CTL_TYPE_PERIOD_NONBLOCK;

	if (val == BOOST_LV_0) {
		cc.type = CC_CTL_TYPE_RESET_NONBLOCK;
	} else {
		switch (val) {
		case BOOST_LV_1: cur = cur + (cur >> 2); break; // scale 1.25
		case BOOST_LV_2: cur = cur + (cur >> 1); break; // scale 1.5
		case BOOST_LV_3: cur = cur + (cur >> 2) + (cur >> 1); break; // scale 1.75
		case BOOST_LV_4: cur = cur + cur; break; // scale 2.0
		case BOOST_LV_MAX: cur = pol->max; break; // jump to max
		}
		if (cpufreq_hispeed_enable && cpufreq_hispeed[idx] > cur) {
			ht_logv("boost cpu hispeed from %u to %u\n", cur, cpufreq_hispeed[idx]);
			cur = cpufreq_hispeed[idx];
		}

		/* cur should not large then pol->max */
		if (cur > pol->max)
			cur = pol->max;
		cc.params[0] = cur;
		cc.params[1] = fps_boost_type? pol->max: cur;
	}
	cpufreq_cpu_put(pol);

	cc_tsk_process(&cc);

	/* update boosted freq info */
	*(orig_freq + idx) = orig;
	*(cur_freq + idx) = cur;
}

static void do_fps_boost(unsigned int val, unsigned int period_us)
{
	int ais_active = ais_enable;
	int i;
	int boost_cluster[HT_CLUSTERS] = {0};
	unsigned int cur[HT_CLUSTERS] = {0}, orig[HT_CLUSTERS] = {0};
	struct task_struct *t;
	u64 prev_ddr_target = 100;
	u64 ddr_target = 100; /* default value */
	//struct cc_command cc;

	if (!fps_boost_enable)
		return;

#ifdef CONFIG_AIGOV
	if (aigov_hooked()) {
		if (val > 0) {
			int cpu = -1;
			rcu_read_lock();
			t = find_task_by_vpid(boost_target[i]);
			if (t)
				cpu = t->cpu;
			rcu_read_unlock();

			aigov_inc_boost_hint(cpu);
			aigov_update_util(cpu_to_clus(cpu));
		} else {
			/* FIXME
			 * here we will reset, but aigov_hook in fps_boost case
			 * not queue into control, so there is no delay work to do
			 * reset, if fps_boost doesn't call reset, then boost hint
			 * will not change.
			 */
			aigov_reset_boost_hint(0);
			aigov_reset_boost_hint(4);
			aigov_reset_boost_hint(7);
			aigov_update_util(0);
			aigov_update_util(1);
			aigov_update_util(2);
		}
		return;
	}
#endif

	ht_logv("boost handler: %llu\n", val);

	if (val > 0) {
		/* ais version boost */
		if (ais_active) {
			/* get cpus which need to be boost */
			rcu_read_lock();
			spin_lock(&ht_rtg_lock);
			list_for_each_entry(t, &ht_rtg_head, rtg_node) {
				++boost_cluster[cpu_to_clus(t->cpu)];
				ht_logv("boost RTG target task %d %s on cpu %d\n", t->pid, t->comm, t->cpu);
			}
			spin_unlock(&ht_rtg_lock);
			spin_lock(&ht_perf_event_lock);
			list_for_each_entry(t, &ht_perf_event_head, ht_perf_event_node) {
				++boost_cluster[cpu_to_clus(t->cpu)];
				ht_logv("boost enqueue target task %d %s on cpu %d\n", t->pid, t->comm, t->cpu);
			}
			spin_unlock(&ht_perf_event_lock);
			rcu_read_unlock();
		} else {
			/* default version */
			for (i = 0; i < FPS_TARGET_NUM; ++i) {
				if (boost_target[i] <= 0)
					continue;
				rcu_read_lock();
				t = find_task_by_vpid(boost_target[i]);
				if (t) {
					++boost_cluster[cpu_to_clus(t->cpu)];
					ht_logv("boost default target task %d %s on cpu %d\n", t->pid, t->comm, t->cpu);
				}
				rcu_read_unlock();
			}
		}

		/* boost */
		for (i = 0; i < HT_CLUSTERS; ++i) {
			if (boost_cluster[i])
				do_cpufreq_boost_helper(clus_to_cpu(i), val, period_us, orig, cur);
		}
	} else {
		/* no need to boost, reset it */
		do_cpufreq_boost_helper(CLUS_0_IDX, val, period_us, orig, cur);
		do_cpufreq_boost_helper(CLUS_1_IDX, val, period_us, orig, cur);
		do_cpufreq_boost_helper(CLUS_2_IDX, val, period_us, orig, cur);
	}

	///* boost ddrfreq */
	//if (ais_active) {
	//	/* setup boost command */
	//	cc.pid = current->pid;
	//	cc.prio = CC_PRIO_HIGH;
	//	cc.period_us = period_us;
	//	cc.group = CC_CTL_GROUP_GRAPHIC;
	//	cc.category = CC_CTL_CATEGORY_DDR_FREQ;
	//	cc.response = 0;
	//	cc.leader = current->tgid;
	//	cc.bind_leader = true;
	//	cc.status = 0;
	//	cc.type = CC_CTL_TYPE_ONESHOT_NONBLOCK;

	//	if (val > 0) {
	//		clk_get_ddr_freq(&prev_ddr_target);
	//		ddr_target = prev_ddr_target;
	//		ddr_target /= 1000000;
	//		ddr_target *= 2;
	//		ddr_target = ddr_find_target(ddr_target);
	//		prev_ddr_target = ddr_find_target(prev_ddr_target/1000000);
	//		if (ddrfreq_hispeed_enable && ddrfreq_hispeed > ddr_target) {
	//			ht_logv("boost ddr hispeed from %u to %u\n", ddr_target, ddrfreq_hispeed);
	//			ddr_target = ddrfreq_hispeed;
	//		}
	//		cc.params[0] = ddr_target;
	//		cc_tsk_process(&cc);
	//	} else {
	//		cc.type = CC_CTL_TYPE_RESET_NONBLOCK;
	//		cc_tsk_process(&cc);
	//	}
	//}

	if (val > 0) {
		for (i = 0; i < HT_CLUSTERS; ++i) {
			if (boost_cluster[i]) {
				ht_logv("boost cluster %d from %lu to %lu, ddr from %llu to %llu. ais %d\n",
						i, orig[i], cur[i], prev_ddr_target, ddr_target, ais_active);
			}
		}
	}

	if (val > 0)
		atomic_inc(&boost_cnt);

}
#endif

static int ht_fps_boost_store(const char *buf, const struct kernel_param *kp)
{
#ifdef CONFIG_CONTROL_CENTER
	unsigned int vals[5] = {0};
	int ret;
	u64 now;

	if (!fps_boost_enable)
		return 0;

	ret = sscanf(buf, "%u,%u,%u,%u,%u\n", &vals[0], &vals[1], &vals[2], &vals[3], &vals[4]);
	if (ret != 5) {
		/* instead of return -EINVAL, just return 0 to keep fd connection keep working */
		ht_loge("boost params invalid. %s. IGNORED.\n", buf);
		return 0;
	}

	ht_logv("boost params: %u %u %u %u %u, from %s %d\n",
		vals[0], vals[1], vals[2], vals[3], vals[4], current->comm, current->pid);

	/* check under ais is working */
	if (perf_ready > 0) {
		/* ignore not tracking task */
		if (vals[1] != perf_ready) {
			ht_logv("%u not the current target(%d). IGNORED\n", vals[1], perf_ready);
			return 0;
		}

		/* ignore boost which is too close to frame enqueue ts */
		if (vals[3] > 0) {
			now = ktime_to_us(ktime_get());
			if (now - parcel.queued_ts_us < fps_boost_filter_us) {
				ht_logv("boost too close, IGNORED. diff %llu\n", now - parcel.queued_ts_us);
				return 0;
			}
		}
	}

	/* update boost target */
	boost_target[0] = vals[1];
	boost_target[1] = vals[2];

	if (vals[0] != FRAME_BOOST && vals[0] != CPU_BOOST) {
		ht_loge("boost type not support\n");
		return 0;
	}

	if (vals[4] == 0) {
		ht_logw("boost period is 0\n");
		return 0;
	}

	do_fps_boost(vals[3], vals[4] * 1000 /* us */);
#endif
	return 0;
}

static struct kernel_param_ops ht_fps_boost_ops = {
	.set = ht_fps_boost_store,
};
module_param_cb(fps_boost, &ht_fps_boost_ops, NULL, 0220);

void ht_register_kgsl_pwrctrl(void *pwr)
{
	gpwr = (struct kgsl_pwrctrl*) pwr;
	ht_logi("ht register kgsl pwrctrl\n");
}

void ht_register_cpu_util(unsigned int cpu, unsigned int first_cpu,
		unsigned long *util, unsigned long *hi_util)
{
	struct ht_util_pol *hus;

	switch (first_cpu) {
	case 0: case 1: case 2: case 3: hus = &ht_utils[0]; break;
	case 4: case 5: case 6: hus = &ht_utils[1]; break;
	case 7: hus = &ht_utils[2]; break;
	default:
		/* should not happen */
		ht_logw("wrnog first cpu utils idx %d\n", first_cpu);
		return;
	}

	if (cpu == CLUS_2_IDX)
		cpu = 0;
	else
		cpu %= HT_CPUS_PER_CLUS;
	hus->utils[cpu] = util;
	hus->hi_util = hi_util;
	ht_logv("ht register cpu %d util, first cpu %d\n", cpu, first_cpu);
}

/* monitor hw event */
void ht_update_hw_events(u64 inst, u64 miss, u64 cycle)
{
	unsigned int d_mask = disable_mask;
	static int cur_idx = 0;

	if (!ht_enable)
		return;

	if (ht_is_all_disabled(d_mask))
		return;

	if (!monitor.buf)
		return;

	if (cur_idx != sidx) {
		cur_idx = sidx;
		monitor.buf->data[cur_idx][HT_HW_INSTRUCTION] = 0;
		monitor.buf->data[cur_idx][HT_HW_CACHE_MISS] = 0;
		monitor.buf->data[cur_idx][HT_HW_CYCLE] = 0;
	}

	monitor.buf->data[cur_idx][HT_HW_INSTRUCTION] += inst;
	monitor.buf->data[cur_idx][HT_HW_CACHE_MISS] += miss;
	monitor.buf->data[cur_idx][HT_HW_CYCLE] += cycle;
}

void ht_register_thermal_zone_device(struct thermal_zone_device *tzd)
{
	int idx;

	/* tzd is guaranteed has value */
	ht_logi("tzd: %s id: %d\n", tzd->type, tzd->id);
	idx = ht_mapping_tags(tzd->type);
	if (idx > HT_THERM_1)
		return;
	if (ht_tzd_idx <= HT_THERM_1) {
		++ht_tzd_idx;
		monitor.tzd[idx] = tzd;
	}
}

void ht_register_power_supply(struct power_supply *psy)
{
	if (!psy)
		return;

	ht_logi("ht power supply list %s\n", psy->desc->name);
	if (!strcmp(psy->desc->name, "battery")) {
		monitor.psy = psy;
		ht_logi("ht power supply registed %s\n", psy->desc->name);
	}
}

static int ht_fps_data_sync_store(const char *buf, const struct kernel_param *kp)
{
	u64 fps_data[FPS_COLS] = {0};
	static long long fps_align = 0;
	char fps_layer_name[FPS_DATA_BUF_SIZE] = {0};
	char process_name[FPS_DATA_BUF_SIZE] = {0};
	static int cur_idx = 0;
	//size_t len;
	int i = 0, ret;

	if (strlen(buf) >= FPS_DATA_BUF_SIZE)
		return 0;

	memset(fps_layer_name, 0, FPS_DATA_BUF_SIZE);

	ret = sscanf(buf, "%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%lld,%s %s\n",
			&fps_data[0], &fps_data[1], &fps_data[2], &fps_data[3],
			&fps_data[4], &fps_data[5], &fps_data[6], &fps_data[7],
			&fps_data[8], &fps_align,
			process_name, fps_layer_name);

	if (ret != 12) {
		/* instead of return -EINVAL, just return 0 to keep fd connection keep working */
		ht_loge("fps data params invalid. %s. IGNORED.\n", buf);
		return 0;
	}

	ht_logv("fps data params: %llu %llu %llu %llu %llu %llu %llu %llu %llu %lld %s %s\n",
		fps_data[0], fps_data[1], fps_data[2], fps_data[3], fps_data[4],
		fps_data[5], fps_data[6], fps_data[7], fps_data[8], fps_align,
		process_name, fps_layer_name);

	/*
	 * cached_fps should be always updated
	 * rounding up
	 */
	atomic_set(&cached_fps[0], (fps_data[0] + 5)/ 10);
	atomic_set(&cached_fps[1], (fps_data[1] + 5)/ 10);

	atomic64_set(&fps_align_ns, fps_align);

	// ignore pass layer name to aischeduler
	/*
	memset(cached_layer_name, 0, FPS_CACHE_LAYER_LEN);
	len = strlen(fps_layer_name);
	if (len < FPS_CACHE_LAYER_LEN)
		memcpy(cached_layer_name, fps_layer_name, len);
	else {
		// first 32 bytes + last 31 bytes + \0
		memcpy(cached_layer_name + (FPS_CACHE_LAYER_LEN/2), fps_layer_name + len - (FPS_CACHE_LAYER_LEN/2) + 1, FPS_CACHE_LAYER_LEN/2);
		memcpy(cached_layer_name, fps_layer_name, (FPS_CACHE_LAYER_LEN/2));
	}
	ht_logv("cached: fps: %u, efps: %u, layer: %s\n", atomic_read(&cached_fps[0]), atomic_read(&cached_fps[1]), cached_layer_name);
	*/

	if (!monitor.buf)
		return 0;

	if (cur_idx != sidx) {
		/* new frame data */
		cur_idx = sidx;
		monitor.buf->layer[cur_idx][0] = '\0';
		monitor.buf->process[cur_idx][0] = '\0';
		monitor.buf->data[cur_idx][HT_FPS_1] = 0;
		monitor.buf->data[cur_idx][HT_FPS_2] = 0;
		monitor.buf->data[cur_idx][HT_FPS_3] = 0;
		monitor.buf->data[cur_idx][HT_FPS_4] = 0;
		monitor.buf->data[cur_idx][HT_FPS_5] = 0;
		monitor.buf->data[cur_idx][HT_FPS_6] = 0;
		monitor.buf->data[cur_idx][HT_FPS_7] = 0;
		monitor.buf->data[cur_idx][HT_FPS_8] = 0;
		monitor.buf->data[cur_idx][HT_FPS_MISS_LAYER] = 0;
	} else if (monitor.buf->layer[cur_idx]) {
		monitor.buf->data[cur_idx][HT_FPS_MISS_LAYER] += 1;
		return 0;
	}

	monitor.buf->data[cur_idx][HT_FPS_1] = fps_data[0]; //actualFps
	monitor.buf->data[cur_idx][HT_FPS_2] = fps_data[1]; //predictFps
	monitor.buf->data[cur_idx][HT_FPS_3] = fps_data[2]; //appSwapTime
	monitor.buf->data[cur_idx][HT_FPS_4] = fps_data[3]; //appSwapDuration
	monitor.buf->data[cur_idx][HT_FPS_5] = fps_data[4]; //appEnqueueDuration
	monitor.buf->data[cur_idx][HT_FPS_6] = fps_data[5]; //sfTotalDuration
	monitor.buf->data[cur_idx][HT_FPS_7] = fps_data[6]; //Vsync
	monitor.buf->data[cur_idx][HT_FPS_8] = fps_data[7]; //sfPresentTime
	monitor.buf->data[cur_idx][HT_FPS_PID] = fps_data[8]; //pid
	monitor.buf->data[cur_idx][HT_FPS_ALIGN]= fps_align; //fps align ts

	fps_layer_name[FPS_LAYER_LEN - 1] = '\0';
	process_name[FPS_PROCESS_NAME_LEN - 1] = '\0';

	i = 0;
	while (fps_layer_name[i]) {
		monitor.buf->layer[cur_idx][i] = fps_layer_name[i];
		++i;
	}

	i = 0;
	while (process_name[i]) {
		monitor.buf->process[cur_idx][i] = process_name[i];
		++i;
	}
	return 0;
}

static struct kernel_param_ops ht_fps_data_sync_ops = {
	.set = ht_fps_data_sync_store,
};
module_param_cb(fps_data_sync, &ht_fps_data_sync_ops, NULL, 0220);

/* poll for fps data synchronization */
static unsigned int ht_ctl_poll(struct file *fp, poll_table *wait)
{
	poll_wait(fp, &ht_poll_waitq, wait);
	return POLLIN;
}

static void ht_collect_system_data(struct ai_parcel *p)
{
	struct cpufreq_policy *pols[HT_CLUSTERS];
	union power_supply_propval prop = {0, };
	int i, ret;

	pols[0] = cpufreq_cpu_get(CLUS_0_IDX);
	pols[1] = cpufreq_cpu_get(CLUS_1_IDX);
	pols[2] = cpufreq_cpu_get(CLUS_2_IDX);

	p->fps_align_ns = atomic64_read(&fps_align_ns);

	p->fps = atomic_read(&cached_fps[0]);
	p->efps = atomic_read(&cached_fps[1]);
	//memcpy(p->layer, cached_layer_name, FPS_CACHE_LAYER_LEN);
	p->cpu_cur_freq_0 = pols[0]? pols[0]->cur: 0;
	p->cpu_cur_freq_1 = pols[1]? pols[1]->cur: 0;
	p->cpu_cur_freq_2 = pols[2]? pols[2]->cur: 0;
	p->cpu_orig_freq_0 = pols[0]? pols[0]->req_freq: 0;
	p->cpu_orig_freq_1 = pols[1]? pols[1]->req_freq: 0;
	p->cpu_orig_freq_2 = pols[2]? pols[2]->req_freq: 0;
	p->cpu_cc_min_freq_1 = pols[1] ? pols[1]->cc_min : 0;
	p->cpu_cc_max_freq_1 = pols[1] ? pols[1]->cc_max : 0;
	p->cpu_orig_min_freq_1 = pols[1] ? pols[1]->min : 0;
	p->cpu_orig_max_freq_1 = pols[1] ? pols[1]->max : 0;
	p->gpu_freq = gpwr? (int) kgsl_pwrctrl_active_freq(gpwr): 0;
	ht_query_ddrfreq(&p->ddr_freq);
	p->ddr_bw = 0; /* should be removed in the future */
	if (bat_query) {
		ht_update_battery();
		ret = power_supply_get_property(monitor.psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
		p->volt_now = ret >= 0? prop.intval: 0;
		ret = power_supply_get_property(monitor.psy, POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
		p->curr_now = ret >= 0? prop.intval: 0;
	}
	/* utils */
	p->utils[0] = ht_utils[0].utils[0]? (u64) *(ht_utils[0].utils[0]): 0;
	p->utils[1] = ht_utils[0].utils[1]? (u64) *(ht_utils[0].utils[1]): 0;
	p->utils[2] = ht_utils[0].utils[2]? (u64) *(ht_utils[0].utils[2]): 0;
	p->utils[3] = ht_utils[0].utils[3]? (u64) *(ht_utils[0].utils[3]): 0;
	p->utils[4] = ht_utils[1].utils[0]? (u64) *(ht_utils[1].utils[0]): 0;
	p->utils[5] = ht_utils[1].utils[1]? (u64) *(ht_utils[1].utils[1]): 0;
	p->utils[6] = ht_utils[1].utils[2]? (u64) *(ht_utils[1].utils[2]): 0;
	p->utils[7] = ht_utils[2].utils[0]? (u64) *(ht_utils[2].utils[0]): 0;

	for (i = 0; i < HT_CLUSTERS; ++i)
		if (pols[i]) cpufreq_cpu_put(pols[i]);

#ifdef CONFIG_CONTROL_CENTER
	/* collect boost ts information */
	cc_boost_ts_collect(p->cbt);
#endif

	p->boost_cnt = atomic_read(&boost_cnt);
	p->notify_start_ts_us = p->queued_ts_us;
	p->notify_end_ts_us = ktime_to_us(ktime_get());
	p->skin_temp = ht_get_temp_delay(HT_THERM_0);
}

static inline void ht_cpuload_helper(int clus, int cpus, struct cpuload_info *cli)
{
	int i, util;
	long long iowait;
	for (i = 0; i < cpus; ++i) {
		/* not ready */
		if (!ht_utils[clus].utils[i])
			return;

		util = (u32) *(ht_utils[clus].utils[i]);
		if (util >= cli->cmax)
			cli->cmax = util;
		if (util <= cli->cmin)
			cli->cmin = util;
		cli->sum += util;

		iowait = ht_delta_iowait[i];
		if (iowait >= cli->iowait_max)
			cli->iowait_max = iowait;
		if (iowait <= cli->iowait_min)
			cli->iowait_min = iowait;
		cli->iowait_sum += iowait;

		++(cli->cnt);
	}
}

static long ht_ctl_ioctl(struct file *file, unsigned int cmd, unsigned long __user arg)
{
	if (_IOC_TYPE(cmd) != HT_IOC_MAGIC) return 0;
	if (_IOC_NR(cmd) > HT_IOC_MAX) return 0;

	ht_logv("%s: cmd: %s %x, arg: %lu\n", __func__, ht_ioctl_str(cmd), cmd, arg);
	switch (cmd) {
	case HT_IOC_COLLECT:
	{
		DEFINE_WAIT(wait);
		prepare_to_wait(&ht_perf_waitq, &wait, TASK_INTERRUPTIBLE);
		schedule();
		finish_wait(&ht_perf_waitq, &wait);
		ht_collect_system_data(&parcel);
		ht_ioctl_retry_count = 0;
		ht_brain_active = true;
		if (copy_to_user((struct ai_parcel __user *) arg, &parcel, sizeof(parcel)))
			return 0;
		break;
	}
	case HT_IOC_SCHEDSTAT:
	{
		struct task_struct *task;
		u64 exec_ns = 0;
		u64 pid;
		if (copy_from_user(&pid, (u64 *) arg, sizeof(u64)))
			return 0;

		rcu_read_lock();
		task = find_task_by_vpid(pid);
		if (task) {
			get_task_struct(task);
			rcu_read_unlock();
			exec_ns = task_sched_runtime(task);
			put_task_struct(task);
			if (exec_ns == 0)
				ht_logw("schedstat is 0\n");
		} else {
			rcu_read_unlock();
			ht_logw("can't find task for schedstat\n");
		}
		if (copy_to_user((u64 *) arg, &exec_ns, sizeof(u64)))
			return 0;
		break;
	}
	case HT_IOC_CPU_LOAD:
	{
		struct task_struct *task;
		struct cpuload_info cli = {
			0, INT_MAX, INT_MIN, 0, LONG_MAX, LONG_MIN, 0
		};
		struct cpuload cl;
		int clus = 0;
		int cpu;
		int i;

		if (!cpuload_query) {
			ht_logv("cpuload query not enabled\n");
			return 0;
		}

		if (copy_from_user(&cl, (struct cpuload *) arg, sizeof(struct cpuload)))
			return 0;

		ht_logv("pid %u tid %u\n", cl.pid, cl.tid);
		rcu_read_lock();
		task = find_task_by_vpid(cl.pid);
		if (task)
			clus |= 1 << cpu_to_clus(task->cpu);
		task = find_task_by_vpid(cl.tid);
		if (task)
			clus |= 1 << cpu_to_clus(task->cpu);
		rcu_read_unlock();

		/* update iowait info */
		for_each_online_cpu(cpu) {
			long long iowait = ht_get_iowait_time(cpu);
			ht_delta_iowait[cpu] = iowait - ht_iowait[cpu];
			ht_iowait[cpu] = iowait;
		}

		for (i = 0; i < 3; ++i) {
			if (clus & (1 << i)) {
				switch (i) {
					case 0:
						ht_cpuload_helper(0, 4, &cli);
						break;
					case 1:
						ht_cpuload_helper(1, 3, &cli);
						break;
					case 2:
						ht_cpuload_helper(2, 1, &cli);
						break;
				}
			}
		}
		cl.min = cli.cmin;
		cl.max = cli.cmax;
		cl.sum = cli.sum;
		cl.avg = cli.cnt? cli.sum/cli.cnt: 0;
		cl.iowait_min = cli.iowait_min;
		cl.iowait_max = cli.iowait_max;
		cl.iowait_sum = cli.iowait_sum;
		cl.iowait_avg = cli.cnt? cli.iowait_sum/cli.cnt: 0;

		ht_logv("cpuload min %d max %d sum %d avg %d, iowait min %lld max %lld sum %lld avg %llu\n",
			cl.min, cl.max, cl.sum, cl.avg, cl.iowait_min, cl.iowait_max, cl.iowait_sum, cl.iowait_avg);
		if (copy_to_user((struct cpuload *) arg, &cl, sizeof(struct cpuload)))
			return 0;
		break;
	}
	default:
		++ht_ioctl_retry_count;
		if (ht_ioctl_retry_count >= HT_IOCTL_RETRY_MAX) {
			DEFINE_WAIT(wait);
			ht_logw("disable support from ai brain\n");
			/* block ai observer here */
			ht_brain_active = false;
			prepare_to_wait(&ht_perf_waitq, &wait, TASK_INTERRUPTIBLE);
			schedule();
			finish_wait(&ht_perf_waitq, &wait);
		}
	}
	return 0;
}

static const struct file_operations ht_ctl_fops = {
	.owner = THIS_MODULE,
	.poll = ht_ctl_poll,
	.unlocked_ioctl = ht_ctl_ioctl,
	.compat_ioctl = ht_ctl_ioctl,
};

static void ht_collect_data(void)
{
	union power_supply_propval prop = {0, };
	struct cpufreq_policy *pol;
	int idx;
	int ret;
	int num = 0;

	if (unlikely(!ht_enable))
		return;

	if (unlikely(!monitor.buf))
		return;

	if (ht_is_all_disabled(disable_mask))
		return;

	idx = ht_next_sample_idx();

	/* set timestamp */
	monitor.buf->data[idx][HT_TS] = ktime_to_ms(ktime_get());

	/* wake up fps data sync polling */
	wake_up_interruptible(&ht_poll_waitq);

	/* temperature part */
	monitor.buf->data[idx][HT_CPU_0] = ht_get_temp(HT_CPU_0);
	monitor.buf->data[idx][HT_CPU_1] = ht_get_temp(HT_CPU_1);
	monitor.buf->data[idx][HT_CPU_2] = ht_get_temp(HT_CPU_2);
	monitor.buf->data[idx][HT_CPU_3] = ht_get_temp(HT_CPU_3);
	monitor.buf->data[idx][HT_CPU_4_0] = ht_get_temp(HT_CPU_4_0);
	monitor.buf->data[idx][HT_CPU_4_1] = ht_get_temp(HT_CPU_4_1);
	monitor.buf->data[idx][HT_CPU_5_0] = ht_get_temp(HT_CPU_5_0);
	monitor.buf->data[idx][HT_CPU_5_1] = ht_get_temp(HT_CPU_5_1);
	monitor.buf->data[idx][HT_CPU_6_0] = ht_get_temp(HT_CPU_6_0);
	monitor.buf->data[idx][HT_CPU_6_1] = ht_get_temp(HT_CPU_6_1);
	monitor.buf->data[idx][HT_CPU_7_0] = ht_get_temp(HT_CPU_7_0);
	monitor.buf->data[idx][HT_CPU_7_1] = ht_get_temp(HT_CPU_7_1);
	monitor.buf->data[idx][HT_THERM_0] = ht_get_temp(HT_THERM_0);
	monitor.buf->data[idx][HT_THERM_1] = ht_get_temp(HT_THERM_1);

	/* cpu part */
	pol = cpufreq_cpu_get(CLUS_0_IDX);
	if (likely(pol)) {
		monitor.buf->data[idx][HT_CLUS_FREQ_0_MIN] = pol->min;
		monitor.buf->data[idx][HT_CLUS_FREQ_0_CUR] = pol->cur;
		monitor.buf->data[idx][HT_CLUS_FREQ_0_MAX] = pol->max;
		monitor.buf->data[idx][HT_ISO_0] = ht_iso_count(pol->related_cpus);
		cpufreq_cpu_put(pol);
	}
	pol = cpufreq_cpu_get(CLUS_1_IDX);
	if (likely(pol)) {
		monitor.buf->data[idx][HT_CLUS_FREQ_1_MIN] = pol->min;
		monitor.buf->data[idx][HT_CLUS_FREQ_1_CUR] = pol->cur;
		monitor.buf->data[idx][HT_CLUS_FREQ_1_MAX] = pol->max;
		monitor.buf->data[idx][HT_ISO_1] = ht_iso_count(pol->related_cpus);
		cpufreq_cpu_put(pol);
	}
	pol = cpufreq_cpu_get(CLUS_2_IDX);
	if (likely(pol)) {
		monitor.buf->data[idx][HT_CLUS_FREQ_2_MIN] = pol->min;
		monitor.buf->data[idx][HT_CLUS_FREQ_2_CUR] = pol->cur;
		monitor.buf->data[idx][HT_CLUS_FREQ_2_MAX] = pol->max;
		monitor.buf->data[idx][HT_ISO_2] = ht_iso_count(pol->related_cpus);
		cpufreq_cpu_put(pol);
	}
	pol = NULL;

	/* gpu part */
	monitor.buf->data[idx][HT_GPU_FREQ] = gpwr? (int) kgsl_pwrctrl_active_freq(gpwr): 0;

	/* util part */
	monitor.buf->data[idx][HT_UTIL_0] = ht_utils[0].utils[0]? (u64) *(ht_utils[0].utils[0]): 0;
	monitor.buf->data[idx][HT_UTIL_1] = ht_utils[0].utils[1]? (u64) *(ht_utils[0].utils[1]): 0;
	monitor.buf->data[idx][HT_UTIL_2] = ht_utils[0].utils[2]? (u64) *(ht_utils[0].utils[2]): 0;
	monitor.buf->data[idx][HT_UTIL_3] = ht_utils[0].utils[3]? (u64) *(ht_utils[0].utils[3]): 0;
	monitor.buf->data[idx][HT_UTIL_4] = ht_utils[1].utils[0]? (u64) *(ht_utils[1].utils[0]): 0;
	monitor.buf->data[idx][HT_UTIL_5] = ht_utils[1].utils[1]? (u64) *(ht_utils[1].utils[1]): 0;
	monitor.buf->data[idx][HT_UTIL_6] = ht_utils[1].utils[2]? (u64) *(ht_utils[1].utils[2]): 0;
	monitor.buf->data[idx][HT_UTIL_7] = ht_utils[2].utils[0]? (u64) *(ht_utils[2].utils[0]): 0;

	/* battery part */
	ht_update_battery();
	ret = power_supply_get_property(monitor.psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	monitor.buf->data[idx][HT_BAT_VOLT_NOW] = ret >= 0? prop.intval: 0;
	ret = power_supply_get_property(monitor.psy, POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
	monitor.buf->data[idx][HT_BAT_CURR_NOW] = ret >= 0? prop.intval: 0;

	/* render & rtg util part*/
	monitor.buf->data[idx][HT_RENDER_PID] = RenPid;
	monitor.buf->data[idx][HT_RENDER_UTIL] = get_util(true, &num);
	monitor.buf->data[idx][HT_RTG_UTIL_SUM] = get_util(false, &num);
	monitor.buf->data[idx][HT_NT_RTG] = num;

	++record_cnt;
	record_cnt = min(record_cnt, (unsigned int) MAX_REPORT_PERIOD);
}

/* main thread for monitor */
static int ht_monitor(void *arg)
{
	while (likely(keep_alive)) {
		if (!ht_enable) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
			continue;
		}

		if (likely(!pm_freezing))
			ht_collect_data();

		msleep(sample_rate);
	}
	return 0;
}

static int ht_registered_show(char* buf, const struct kernel_param *kp)
{
	int i, offset = 0;
	struct thermal_zone_device* tzd;

	if (ht_tzd_idx == 0)
		return 0;

	for (i = 0; i < ht_tzd_idx; ++i) {
		tzd = monitor.tzd[i];
		if (tzd)
			offset += snprintf(buf + offset, PAGE_SIZE - offset, "%s, id: %d\n", tzd->type, tzd->id);
		else if (i < HT_CPU_0)
			offset += snprintf(buf + offset, PAGE_SIZE - offset, "%s\n", ht_monitor_case[i]);
	}

	for (i = HT_UTIL_0; i < HT_MONITOR_SIZE; ++i)
		offset += snprintf(buf + offset, PAGE_SIZE - offset, "%s\n", ht_monitor_case[i]);

	return offset;
}

static struct kernel_param_ops ht_registed_ops = {
	.get = ht_registered_show,
};
module_param_cb(ht_registed, &ht_registed_ops, NULL, 0444);

static int ht_reset_store(const char *buf, const struct kernel_param *kp)
{
	int val;
	if (sscanf(buf, "%d\n", &val) != 1)
		return -EINVAL;

	if (val != 1)
		return 0;

	memset(monitor.buf, 0, sizeof(struct sample_data));
	record_cnt = 0;
	RenPid = -1;
	ht_logi("sample data reset\n");

	return 0;
}

static struct kernel_param_ops ht_reset_ops = {
	.set = ht_reset_store,
};
module_param_cb(reset, &ht_reset_ops, NULL, 0664);

static int ht_report_proc_show(struct seq_file *m, void *v)
{
	unsigned int i, cnt = 0, j;
	unsigned long f_mask = filter_mask, d_mask = disable_mask;
	unsigned int report_cnt = 0;

	if (!monitor.buf) {
		seq_printf(m, "sample buffer not inited\n");
		return 0;
	}

	report_cnt = record_cnt;

	/* cap max */
	report_cnt = min(report_cnt, (unsigned int) MAX_REPORT_PERIOD);

	if (!report_cnt) {
		seq_printf(m, "no data recorded\n");
		return 0;
	}

	if (ht_is_all_filtered(f_mask) || ht_is_all_disabled(d_mask)) {
		seq_printf(m, "all data be masked out\n");
		return 0;
	}

	/* mark */
	for (i = 0; i < ht_tzd_idx; ++i) {
		if (f_mask & (1 << i) || d_mask & (1 << i))
			continue;
		if (monitor.tzd[i])
			seq_printf(m, "%s,", monitor.tzd[i]->type);
		else if (i < HT_CPU_0)
			seq_printf(m, "%s,", ht_monitor_case[i]);
	}

	/* TODO should refine this part */
	for (i = HT_UTIL_0; i < HT_MONITOR_SIZE; ++i) {
		if (f_mask & (1 << i) || d_mask & (1 << i))
			continue;
		seq_printf(m, "%s,", ht_monitor_case[i]);
	}
	seq_printf(m, "\n");

	/* fill gap */
	i = sidx + MAX_REPORT_PERIOD - report_cnt + 1;

	/* value */
	for (; cnt < report_cnt; ++cnt, ++i) {
		i %= MAX_REPORT_PERIOD;

		for (j = 0; j < HT_MONITOR_SIZE; ++j) {
			if (f_mask & (1 << j) || d_mask & (1 << j))
				continue;
			if (j == HT_FPS_LAYER) {
				seq_printf(m, "%s,", monitor.buf->layer[i]);
				continue;
			}
			if (j == HT_FPS_PROCESS) {
				seq_printf(m, "%s,", monitor.buf->process[i]);
				continue;
			}
			seq_printf(m, "%llu,", monitor.buf->data[i][j]/ (report_div[j]? report_div[j]: 1));
		}
		seq_printf(m, "\n");
	}

	return 0;
}

static int ht_report_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ht_report_proc_show, NULL);
}

static const struct file_operations ht_report_proc_fops = {
	.open= ht_report_proc_open,
	.read= seq_read,
	.llseek= seq_lseek,
	.release= single_release,
};

static int fps_sync_init(void)
{
	int rc;
	struct device *class_dev;

	rc = alloc_chrdev_region(&ht_ctl_dev, 0, 1, HT_CTL_NODE);
	if (rc < 0) {
		ht_loge("alloc_chrdev_region failed %d\n", rc);
		return 0;
	}

	driver_class = class_create(THIS_MODULE, HT_CTL_NODE);
	if (IS_ERR(driver_class)) {
		rc = -ENOMEM;
		ht_loge("class_create failed %d\n", rc);
		goto exit_unreg_chrdev_region;
	}
	class_dev = device_create(driver_class, NULL, ht_ctl_dev, NULL, HT_CTL_NODE);
	if (IS_ERR(class_dev)) {
		ht_loge("class_device_create failed %d\n", rc);
		rc = -ENOMEM;
		goto exit_destroy_class;
	}
	cdev_init(&cdev, &ht_ctl_fops);
	cdev.owner = THIS_MODULE;
	rc = cdev_add(&cdev, MKDEV(MAJOR(ht_ctl_dev), 0), 1);
	if (rc < 0) {
		ht_loge("cdev_add failed %d\n", rc);
		goto exit_destroy_device;
	}
	ht_logi("fps data sync ready\n");
	return 0;
exit_destroy_device:
	device_destroy(driver_class, ht_ctl_dev);
exit_destroy_class:
	class_destroy(driver_class);
exit_unreg_chrdev_region:
	unregister_chrdev_region(ht_ctl_dev, 1);
	return 0;
}

void ht_update_enqueue_ts(struct task_struct *task)
{
	u64 now = 0;
	spin_lock(&task->rtg_lock);
	asm volatile("isb;mrs %0, PMCCNTR_EL0" : "=r"(now));

	if (task->run_ts == task->end_ts) {
		/* task not finished yet */
		task->acc_run_ts += now - task->run_ts;
	}
	task->delta_ts = task->total_run_ts;
	task->total_run_ts += task->acc_run_ts;
	task->acc_run_ts = 0;
	task->run_ts = task->end_ts = now;
	spin_unlock(&task->rtg_lock);
}

void ht_perf_notify(void)
{
	u64 time;

	if (perf_ready <= 0)
		return;

	if (perf_ready != current->tgid) {
		current->enqueue_ts = 0;
		ht_logv("notify: task %s %d ignored due to not target process\n", current->comm, current->pid);
		return;
	}

	/* if thread been tracked by perf, skip it */
	if (current->perf_regular_activate) {
		ht_logv("notify: task %s %d ignored due to perf already activated\n", current->comm, current->pid);
		return;
	}

	ht_logv("notify: task: comm: %s, pid: %d, tgid: %d, freq: %u, peak: %u, leader: %s %d %d, ts: %lld\n",
		current->comm, current->pid, current->tgid, current->f_cnt, current->f_peak,
		current->group_leader->comm, current->group_leader->pid, current->group_leader->tgid,
		current->enqueue_ts);

	RenPid = current->pid;

	/* fps info update */
	wake_up_interruptible(&ht_poll_waitq);

	/* get timestamp */
	time = ktime_to_us(ktime_get());
	ht_update_enqueue_ts(current);
	current->enqueue_ts = time;

	/* may need to handle overflow */
	if (time < current->f_ts ||
		time - current->f_ts >= 1000000 /* 1 sec */) {
		current->f_peak = current->f_cnt;
		current->f_cnt = 0;
		current->f_ts = time;
	}
	++current->f_cnt;

	if (likely(ht_perf_workq)) {
		queue_work(ht_perf_workq, &current->perf_work);
	}
}

void ht_perf_event_init(struct task_struct *task)
{
	int i;

	INIT_LIST_HEAD(&task->ht_perf_event_node);
	INIT_LIST_HEAD(&task->perf_node);
	INIT_WORK(&task->perf_work, ht_collect_perf_data);
	for (i = 0; i < HT_PERF_COUNT_MAX; ++i) {
		task->perf_events[i] = NULL;
		task->perf_counters[i] = 0;
	}
	task->perf_activate = 0;
	task->perf_regular_activate = 0;
	task->enqueue_ts = 0;
	task->run_ts = 0;
	task->end_ts = 0;
	task->acc_run_ts = 0;
	task->delta_ts = 0;
	task->total_run_ts = 0;
}

void ht_perf_event_release(struct task_struct *task)
{
	int i;

	mutex_lock(&task->perf_event_mutex);
	for (i = 0; i < HT_PERF_COUNT_MAX; ++i) {
		if (task->perf_events[i]) {
			perf_event_disable(task->perf_events[i]);
			perf_event_release_kernel(task->perf_events[i]);
			task->perf_events[i] = NULL;
			task->perf_activate &= ~(1 << i);
			ht_logv("perf event released. task %s %d id %d\n", task->comm, task->pid, i);
		}
	}
	mutex_unlock(&task->perf_event_mutex);

	spin_lock(&ht_perf_event_lock);
	if (!list_empty(&task->ht_perf_event_node))
		list_del_init(&task->ht_perf_event_node);
	spin_unlock(&ht_perf_event_lock);
}

void ht_sched_switch_update(struct task_struct *prev, struct task_struct *next)
{
	if (perf_ready <= 0)
		return;

	if (prev->tgid != perf_ready && next->tgid != perf_ready)
		return;

	if (!prev->enqueue_ts && !next->enqueue_ts)
		return;

	if (prev->enqueue_ts)
		ht_sched_update(prev, false);
	if (next->enqueue_ts)
		ht_sched_update(next, true);
}

/* perform LRU order */
void ht_rtg_list_add_tail(struct task_struct *task)
{
	u64 time;

	if (perf_ready <= 0)
		return;

	if (!task->pid)
		return;

	if (task->tgid != perf_ready)
		return;

	ht_logv("rtg task add list: %s(%d) util: %d, peak: %d\n",
		task->comm, task->pid, task->ravg.demand_scaled, task->rtg_peak);

	if (task->ravg.demand_scaled < base_util)
		return;

	time = ktime_to_us(ktime_get());

	/* may need to handle overflow */
	if (time < task->rtg_period_ts ||
		time - task->rtg_period_ts >= 1000000 /* 1 sec */) {
		task->rtg_peak = task->rtg_cnt;
		task->rtg_cnt = 0;
		task->rtg_period_ts = time;
	}
	++task->rtg_cnt;
	task->rtg_ts = time;

	/* filter out low frequent candidate */
	if (task->rtg_peak < rtg_filter_cnt)
		return;

	/* quick check with no lock */
	if (!list_empty(&task->rtg_node))
		return;

	/* real list update */
	spin_lock(&ht_rtg_lock);
	if (list_empty(&task->rtg_node))
		list_add_tail(&task->rtg_node, &ht_rtg_head);
	spin_unlock(&ht_rtg_lock);
}
EXPORT_SYMBOL(ht_rtg_list_add_tail);

void ht_rtg_list_del(struct task_struct *task)
{
	if (!task->pid)
		return;

	spin_lock(&ht_rtg_lock);
	if (!list_empty(&task->rtg_node)) {
		ht_logv("rtg task del list: %s(%d) util: %d, peak: %d\n",
			task->comm, task->pid, task->ravg.demand_scaled, task->rtg_peak);
		list_del_init(&task->rtg_node);
	}
	spin_unlock(&ht_rtg_lock);
}
EXPORT_SYMBOL(ht_rtg_list_del);

static int rtg_dump_show(char *buf, const struct kernel_param *kp)
{
	int cnt = 0;
	int size = 0;
	u64 time = ktime_to_us(ktime_get());
	struct task_struct *t;

	rcu_read_lock();
	spin_lock(&ht_rtg_lock);
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "RTG list: comm, pid, util, peak, cnt, delta ts, ts\n");
	list_for_each_entry(t, &ht_rtg_head, rtg_node) {
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "%s %d %lu %u %u %lld %lld\n",
				t->comm, t->pid, t->ravg.demand_scaled,
				t->rtg_peak, t->rtg_cnt,
				time - t->rtg_ts, t->rtg_ts);
		++size;
	}
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "Total: %d\n", size);
	spin_unlock(&ht_rtg_lock);
	size = 0;
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "RTG perf event created list: comm, pid\n");
	spin_lock(&ht_perf_event_lock);
	list_for_each_entry(t, &ht_perf_event_head, ht_perf_event_node) {
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "%s %d\n", t->comm, t->pid);
		++size;
	}
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "Total: %d\n", size);
	spin_unlock(&ht_perf_event_lock);
	rcu_read_unlock();
	return cnt;
}

static struct kernel_param_ops rtg_dump_ops = {
	.get = rtg_dump_show,
};
module_param_cb(rtg_dump, &rtg_dump_ops, NULL, 0444);


static int get_util(bool isRender, int *num)
{
	int util = 0;
	struct task_struct *t;

	rcu_read_lock();

	if (!isRender) {
		spin_lock(&ht_rtg_lock);
		list_for_each_entry(t, &ht_rtg_head, rtg_node) {
			util += t->ravg.demand_scaled;
			(*num)++;
			ht_logv("RTG: comm:%s pid:%d util:%lu\n",
					t->comm, t->pid, t->ravg.demand_scaled);
		}
		spin_unlock(&ht_rtg_lock);
	} else {
		spin_lock(&ht_perf_event_lock);
		list_for_each_entry(t, &ht_perf_event_head,
				ht_perf_event_node) {
			if (RenPid != t->pid)
				continue;
			util = t->ravg.demand_scaled;
			ht_logv("Render: comm:%s pid:%d util:%lu\n",
					t->comm, t->pid, t->ravg.demand_scaled);
			break;
		}
		spin_unlock(&ht_perf_event_lock);
	}

	rcu_read_unlock();

	return util;
}

static int ht_init(void)
{
	int i;

	/* init cached fps info */
	atomic_set(&cached_fps[0], 60);
	atomic_set(&cached_fps[1], 60);
	atomic64_set(&fps_align_ns, 0);

	for (i = 0; i < HT_MONITOR_SIZE; ++i)
		report_div[i] = (i >= HT_CPU_0 && i <= HT_THERM_1)? 100: 1;

	ht_set_all_mask();

	monitor.thread = kthread_create(ht_monitor, NULL, "ht_monitor");
	if (IS_ERR(monitor.thread)) {
		ht_loge("Can't create ht monitor thread\n");
		return -EAGAIN;
	}

	keep_alive = true;
	wake_up_process(monitor.thread);

	proc_create("ht_report", S_IFREG | 0444, NULL, &ht_report_proc_fops);

	fps_sync_init();
	ht_perf_workq = alloc_ordered_workqueue("ht_wq", 0);
	if (!ht_perf_workq) {
		/* TODO handle this */
		ht_loge("alloc work queue fail\n");
	}

	/* enable all counter */
	on_each_cpu(enable_cpu_counters, NULL, 1);
	ht_logi("Enable Access PMU Initialized\n");
	return 0;
}
pure_initcall(ht_init);
