#ifndef _OPCHAIN_STRUCT_OFFSET_HELPER_INC_
#define _OPCHAIN_STRUCT_OFFSET_HELPER_INC_

/* define macro to extern function declaration */
#define gen_type_offset(type) \
	extern unsigned int opchain_get_##type##_offset(int m);

/* define macro to create offset impl and export get offset/value symbol */
#define gen_type_offset_impl(type) \
	unsigned int opchain_get_##type##_offset(int m) { \
		return opchain_##type##_offset[m]; } \
	EXPORT_SYMBOL(opchain_get_##type##_offset);

/* enum of struct task struct */
enum {
	TASK_OFFSET_WAKEE_FLIPS,
	TASK_OFFSET_CPUS_ALLOWED,
	TASK_OFFSET_PID,
	TASK_OFFSET_TGID,
	TASK_OFFSET_GROUP_LEADER,
	TASK_OFFSET_COMM,
	TASK_OFFSET_UTASK_TAG,
	TASK_OFFSET_UTASK_TAG_BASE,
	TASK_OFFSET_ETASK_CLAIM,
	TASK_OFFSET_CLAIM_CPU,
	TASK_OFFSET_UTASK_SLAVE,
	TASK_OFFSET_EXIT_STATE,

	__TASK_OFFSET_MAX
};
#define TASK_WAKEE_FLIPS_R(t) (*(unsigned long *)((char *)t + opchain_get_task_struct_offset(TASK_OFFSET_WAKEE_FLIPS)))
#define TASK_CPUS_ALLOWED_ADDR(t) (cpumask_t *)((char *)t + opchain_get_task_struct_offset(TASK_OFFSET_CPUS_ALLOWED))
#define TASK_PID_R(t) (*(pid_t *)((char *)t + opchain_get_task_struct_offset(TASK_OFFSET_PID)))
#define TASK_TGID_R(t) (*(pid_t *)((char *)t + opchain_get_task_struct_offset(TASK_OFFSET_TGID)))
#define TASK_GROUP_LEADER_R(t) (*(struct task_struct **)((char *)t + opchain_get_task_struct_offset(TASK_OFFSET_GROUP_LEADER)))
#define TASK_COMM_R(t) ((char *)t + opchain_get_task_struct_offset(TASK_OFFSET_COMM))
#define TASK_UTASK_TAG_R(t) (*(u64 *)((char *)t + opchain_get_task_struct_offset(TASK_OFFSET_UTASK_TAG)))
#define TASK_UTASK_TAG_BASE_R(t) (*(u64 *)((char *)t + opchain_get_task_struct_offset(TASK_OFFSET_UTASK_TAG_BASE)))
#define TASK_ETASK_CLAIM_R(t) (*(int *)((char *)t + opchain_get_task_struct_offset(TASK_OFFSET_ETASK_CLAIM)))
#define TASK_CLAIM_CPU_R(t) (*(int *)((char *)t + opchain_get_task_struct_offset(TASK_OFFSET_CLAIM_CPU)))
#define TASK_UTASK_SLAVE_R(t) (*(bool *)((char *)t + opchain_get_task_struct_offset(TASK_OFFSET_UTASK_SLAVE)))
#define TASK_EXIT_STATE_R(t) (*(bool *)((char *)t + opchain_get_task_struct_offset(TASK_OFFSET_EXIT_STATE)))
gen_type_offset(task_struct);

/* enum of struct rq */
enum {
	RQ_OFFSET_CLOCK,
#ifdef CONFIG_SMP
	RQ_OFFSET_CPU_CAPACITY_ORIG,
	RQ_OFFSET_CPU,
#endif
#ifdef CONFIG_SCHED_HMP
	RQ_OFFSET_WINDOW_START,
#endif

	__RQ_OFFSET_MAX
};
#define RQ_CLOCK_R(rq) (*(u64 *)((char *)rq + opchain_get_rq_offset(RQ_OFFSET_CLOCK)))
#ifdef CONFIG_SMP
#define RQ_CPU_CAPACITY_ORIG_R(rq) (*(unsigned long *)((char *)rq + opchain_get_rq_offset(RQ_OFFSET_CPU_CAPACITY_ORIG)))
#define RQ_CPU_R(rq) (*(int *)((char *)rq + opchain_get_rq_offset(RQ_OFFSET_CPU)))
#endif
#ifdef CONFIG_SCHED_HMP
#define RQ_WINDOW_START_R(rq) (*(u64 *)((char *)rq + opchain_get_rq_offset(RQ_OFFSET_WINDOW_START)))
#endif
gen_type_offset(rq);

#endif //_OPCHAIN_STRUCT_OFFSET_HELPER_INC_
