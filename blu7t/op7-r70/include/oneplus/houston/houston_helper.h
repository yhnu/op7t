#ifndef __INCLUDE_HOUSTON_HELPER__
#define __INCLUDE_HOUSTON_HELPER__

#include <linux/sched.h>
#include <linux/thermal.h>
#include <linux/power_supply.h>

enum ht_perf_id {
	HT_PERF_COUNT_CPU_CYCLES = 0,
	HT_PERF_COUNT_INSTRUCTIONS = 1,
	HT_PERF_COUNT_CACHE_MISSES_L1 = 2,
	HT_PERF_COUNT_CACHE_MISSES_L2 = 3,
	HT_PERF_COUNT_CACHE_MISSES_L3 = 4,
	HT_PERF_COUNT_MAX,
};

#ifdef CONFIG_HOUSTON
extern void ht_register_thermal_zone_device(struct thermal_zone_device *tz);
extern void ht_register_power_supply(struct power_supply* psy);
extern void ht_register_cpu_util(unsigned int cpu, unsigned int first_cpu, unsigned long *util, unsigned long *hi_util);
extern void ht_register_kgsl_pwrctrl(void *pwr);
extern void ht_update_hw_events(u64 inst, u64 miss, u64 cycle);
extern void ht_perf_notify(void);
extern void ht_perf_event_init(struct task_struct *tsk);
extern void ht_perf_event_release(struct task_struct *tsk);
extern void ht_collect_perf_data(struct work_struct *work);
extern void ht_rtg_init(struct task_struct *task);
extern void ht_rtg_list_add_tail(struct task_struct *task);
extern void ht_rtg_list_del(struct task_struct *task);
extern void ht_sched_switch_update(struct task_struct *prev, struct task_struct *next);
#else
static inline void ht_register_thermal_zone_device(struct thermal_zone_device *tz) {};
static inline void ht_register_power_supply(struct power_supply* psy) {};
static inline void ht_register_cpu_util(unsigned int cpu, unsigned int first_cpu, unsigned long *util, unsigned long *hi_util) {};
static inline void ht_register_kgsl_pwrctrl(void *pwr) {};
static inline void ht_update_hw_events(u64 inst, u64 miss, u64 cycle) {};
static inline void ht_perf_notify(void) {};
static inline void ht_perf_event_init(struct task_struct *tsk) {};
static inline void ht_perf_event_release(struct task_struct *tsk) {};
static inline void ht_collect_perf_data(struct work_struct *work) {};
static inline void ht_rtg_init(struct task_struct *task) {};
static inline void ht_rtg_list_add_tail(struct task_struct *task) {};
static inline void ht_rtg_list_del(struct task_struct *task) {};
static inline void ht_sched_switch_update(struct task_struct *prev, struct task_struct *next) {};
#endif
#endif // __INCLUDE_HOUSTON_HELPER__
