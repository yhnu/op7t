#ifndef __INCLUDE_AIGOV_HELPER__
#define __INCLUDE_AIGOV_HELPER__

#ifdef CONFIG_AIGOV
extern bool aigov_enabled(void);
extern bool aigov_hooked(void);
extern bool aigov_use_util(void);
extern void aigov_reset(void);
extern void aigov_update_util(int clus);
extern void aigov_set_cpufreq(int cpu, int cpufreq);
extern void aigov_set_ddrfreq(int ddrfreq);
extern void aigov_inc_boost_hint(int cpu);
extern void aigov_reset_boost_hint(int cpu);
extern int aigov_get_cpufreq(int cpu);
extern int aigov_get_ddrfreq(void);
extern int aigov_get_boost_hint(int cpu);
extern unsigned int aigov_get_weight(void);
extern void aigov_dump(int cpu, unsigned long util, unsigned long aig_util, unsigned long extra_util);
#else
static inline bool aigov_enabled(void) { return false; }
static inline bool aigov_hooked(void) { return false; }
static inline bool aigov_use_util(void) { return false; }
static inline void aigov_reset(void) {}
static inline void aigov_update_util(int clus) {}
static inline void aigov_set_cpufreq(int cpu, int cpufreq) {}
static inline void aigov_set_ddrfreq(int ddrfreq) {}
static inline void aigov_inc_boost_hint(int cpu) {}
static inline void aigov_reset_boost_hint(int cpu) {}
static inline int aigov_get_cpufreq(int cpu) { return 0; }
static inline int aigov_get_ddrfreq(void) { return 0; }
static inline int aigov_get_boost_hint(int cpu) { return 0; }
static inline unsigned int aigov_get_weight(void) { return 0; }
static inline void aigov_dump(int cpu, unsigned long util, unsigned long aig_util, unsigned long extra_util) {}
#endif

#endif // __INCLUDE_AIGOV_HELPER__
