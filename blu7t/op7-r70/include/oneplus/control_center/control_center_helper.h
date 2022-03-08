#ifndef __CONTROL_CENTER_HELPER_INC__
#define __CONTROL_CENTER_HELPER_INC__

#ifdef CONFIG_CONTROL_CENTER
/* define for boost ts information */
#define CC_BOOST_TS_SIZE (8)
struct cc_boost_ts {
	pid_t pid;
	u32 type;
	u64 ts_us;
	u64 min;
	u64 max;
};

#define CC_CTL_PARAM_SIZE 4
struct cc_command {
	pid_t pid;
	pid_t leader;
	u32 period_us;
	u32 prio;
	u32 group;
	u32 category;
	u32 type;
	u64 params[CC_CTL_PARAM_SIZE];
	u64 response;
	bool bind_leader;
	int status;
};

/* define for task embedded data */
struct cc_tsk_data {
	struct cc_command cc;
	struct list_head node;
	struct delayed_work dwork;
};

extern void cc_tsk_init(void* task);
extern void cc_tsk_free(void* task);

/* ddr related control */
extern atomic_t cc_expect_ddrfreq;
extern bool cc_ddr_boost_enable;
//extern bool cc_ddr_set_enable;
//extern bool cc_ddr_lower_bound_enable;
//extern bool cc_ddr_lock_enable;

extern u64 cc_cpu_find_ddr(int cpu);
extern bool cc_is_ddrfreq_related(const char* name);
#else
static inline void cc_tsk_init(void* task) {};
static inline void cc_tsk_free(void* task) {};

extern u64 cc_cpu_find_ddr(int cpu) { return 0; }
extern bool cc_is_ddrfreq_related(const char* name) { return false; }
#endif

#endif
