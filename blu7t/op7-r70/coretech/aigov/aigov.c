#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/sched/cpufreq.h>

#include <../../kernel/sched/sched.h>

#include <linux/oem/aigov.h>

/*
 * log output
 * lv == 0 -> verbose info warning error
 * lv == 1 -> info warning error
 * lv == 2 -> wraning error
 * lv >= 3 -> error
 */
static int aigov_log_lv = 1;
module_param_named(log_lv, aigov_log_lv, int, 0664);

/* init flag */
static bool aigov_inited __read_mostly = 0;

/* feature enable flag */
static bool aigov_enable = false;
module_param_named(enable, aigov_enable, bool, 0664);

/* hook enable flag, to hook other module for info update */
static bool aigov_hook = false;
module_param_named(hook, aigov_hook, bool, 0664);

static bool _aigov_use_util = false;
module_param_named(use_util, _aigov_use_util, bool, 0664);

static bool aigov_can_update = false;
module_param_named(aigov_can_update, aigov_can_update, bool, 0664);

/* global resource */
static int g_cpufreq[3] = {0};
static int g_ddrfreq = 0;
static atomic_t g_boost_hint[3] = {ATOMIC_INIT(0)};

/* aigov weight */
static unsigned int aigov_weight = 50;

DEFINE_PER_CPU(struct aigov_stat, aig_stat);

/* helper */
static inline int cpu_to_clus(int cpu)
{
	if (cpu < 4)
		return 0;
	if (cpu < 7)
		return 1;
	return 2;
}

static inline int clus_to_first_cpu(int clus)
{
	if (clus == 0)
		return 0;
	if (clus == 1)
		return 4;
	return 7;
}

bool aigov_enabled(void)
{
	return aigov_enable;
}

bool aigov_hooked(void)
{
	return aigov_hook;
}

bool aigov_use_util(void)
{
	return _aigov_use_util;
}

void aigov_reset(void)
{
	int i;

	if (unlikely(!aigov_inited)) {
		aigov_loge("update before init\n");
		return;
	}

	for (i = 0; i < 3; ++i) {
		struct aigov_stat* ais = &per_cpu(aig_stat, clus_to_first_cpu(i));
		*ais->pred_cpufreq = 0;
		*ais->pred_ddrfreq = 0;
		atomic_set(ais->boost_hint, 0);
	}
	aigov_logv("reset\n");
}

void aigov_set_cpufreq(int cpu, int cpufreq)
{
	struct aigov_stat* ais = &per_cpu(aig_stat, cpu);

	if (likely(aigov_inited))
		*ais->pred_cpufreq = cpufreq;
	else
		aigov_loge("update before init\n");
}

void aigov_set_ddrfreq(int ddrfreq)
{
	struct aigov_stat* ais = &per_cpu(aig_stat, 0);

	if (likely(aigov_inited))
		*ais->pred_ddrfreq = ddrfreq;
	else
		aigov_loge("update before init\n");
}

void aigov_inc_boost_hint(int cpu)
{
	struct aigov_stat* ais = &per_cpu(aig_stat, cpu);

	if (likely(aigov_inited))
		atomic_inc(ais->boost_hint);
	else
		aigov_loge("update before init\n");
}

void aigov_reset_boost_hint(int cpu)
{
	struct aigov_stat* ais = &per_cpu(aig_stat, cpu);

	if (likely(aigov_inited))
		atomic_set(ais->boost_hint, 0);
	else
		aigov_loge("update before init\n");
}

int aigov_get_cpufreq(int cpu)
{
	struct aigov_stat* ais = &per_cpu(aig_stat, cpu);

	if (likely(aigov_inited))
		return *ais->pred_cpufreq;
	return 0;
}

int aigov_get_ddrfreq(void)
{
	struct aigov_stat* ais = &per_cpu(aig_stat, 0);

	if (likely(aigov_inited))
		return *ais->pred_ddrfreq;
	return 0;
}

int aigov_get_boost_hint(int cpu)
{
	struct aigov_stat* ais = &per_cpu(aig_stat, cpu);

	if (likely(aigov_inited))
		return (int) atomic_read(ais->boost_hint);
	return 0;
}

void aigov_dump(int cpu, unsigned long util, unsigned long aig_util, unsigned long extra_util)
{
	aigov_logv("cpu%d %d cpu%d %d cpu%d %d ddrfreq %d boost %d %d %d\n",
		0, aigov_get_cpufreq(0),
		4, aigov_get_cpufreq(4),
		7, aigov_get_cpufreq(7),
		aigov_get_ddrfreq(),
		aigov_get_boost_hint(0),
		aigov_get_boost_hint(4),
		aigov_get_boost_hint(7));

	aigov_logv("cpu%d: orig: %llu, aig: %llu: extra: %llu\n",
		cpu, util, aig_util, extra_util);
}

static int aigov_weight_store(const char *buf, const struct kernel_param *kp)
{
	unsigned int val;

	if (sscanf(buf, "%u\n", &val) <= 0)
		return -EINVAL;
	aigov_weight = clamp_val(val, 0, 100);
	return 0;
}

static int aigov_weight_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", aigov_weight);
}

static struct kernel_param_ops aigov_weight_ops = {
	.set = aigov_weight_store,
	.get = aigov_weight_show,
};
module_param_cb(aigov_weight, &aigov_weight_ops, NULL, 0644);

unsigned int aigov_get_weight(void)
{
	return aigov_weight;
}

/*
 * FIXME
 * from control center it only has cluster info
 * may not proper to call cpufreq_update_util withou cpu info
 */
void aigov_update_util(int clus)
{
	/* FIXME, pick first cpu as target */
	int flag = SCHED_CPUFREQ_WALT | SCHED_CPUFREQ_AIGOV;
	int cpu = clus_to_first_cpu(clus);

	if (!aigov_can_update)
		return;

	/*
	 * does this lock hold too frequently?
	 * we have 2 approaches for this.
	 * 1. leverage cpufreq_update_util to choose frequency with system util
	 * 2. direct call cpufreq_driver_fast_switch to change frequency
	 */
	raw_spin_lock(&cpu_rq(cpu)->lock);

	cpufreq_update_util(cpu_rq(cpu), flag);

	raw_spin_unlock(&cpu_rq(cpu)->lock);

	aigov_logv("clus %d update util\n", clus);
}

static int aigov_dump_show(char *buf, const struct kernel_param *kp)
{
	int cnt = 0;
	int cpu;

	for_each_possible_cpu(cpu) {
		struct aigov_stat* ais = &per_cpu(aig_stat, cpu);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
			"clus: %d, cpu: %d, p_cpufreq: %d, p_ddrfreq: %d\n",
		ais->clus, ais->cpu, *ais->pred_cpufreq, *ais->pred_ddrfreq);
	}

	return cnt;
}

static struct kernel_param_ops aigov_dump_ops = {
	.get = aigov_dump_show,
};
module_param_cb(aigov_dump, &aigov_dump_ops, NULL, 0444);

/* test trigger entry */
static int aigov_trigger_show(char *buf, const struct kernel_param *kp)
{
	aigov_update_util(0);
	aigov_update_util(1);
	aigov_update_util(2);

	return snprintf(buf, PAGE_SIZE, "triggered\n");
}

static struct kernel_param_ops aigov_trigger_ops = {
	.get = aigov_trigger_show,
};
module_param_cb(aigov_trigger, &aigov_trigger_ops, NULL, 0444);

static int aigov_init(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		struct aigov_stat* ais = &per_cpu(aig_stat, cpu);
		aigov_logi("init cpu %d\n", cpu);
		ais->cpu = cpu;
		ais->clus = cpu_to_clus(cpu);
		ais->pred_cpufreq = &g_cpufreq[ais->clus];
		ais->pred_ddrfreq = &g_ddrfreq;
		ais->boost_hint = &g_boost_hint[ais->clus];
	}

	aigov_logi("inited\n");

	aigov_inited = true;

	return 0;
}
pure_initcall(aigov_init);
