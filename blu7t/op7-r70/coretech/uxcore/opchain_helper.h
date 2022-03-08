#ifndef _LINUX_OPCHAIN_HELPER_H
#define _LINUX_OPCHAIN_HELPER_H
#include "opchain_define.h"

#ifdef CONFIG_OPCHAIN
extern struct opchain_cb uxcore_api;
extern void opc_binder_pass(size_t data_size, uint32_t *data, int send);
extern bool is_opc_task(struct task_struct *t, int type);
extern void opc_task_switch(bool enqueue, int cpu, struct task_struct *p, u64 clock);
extern int opc_get_claim_on_cpu(int cpu);
extern unsigned int opc_get_claims(void);
extern int opc_select_path(struct task_struct *cur, struct task_struct *t, int prev_cpu);
extern unsigned long opc_cpu_util(unsigned long util, int cpu, struct task_struct *t, int op_path);
extern bool opc_fps_check(int lvl);
extern void *opc_task_rq(void *t);
extern struct rq *opc_cpu_rq(int cpu);
extern unsigned int opc_task_load(struct task_struct *p);
extern int opc_cpu_active(int cpu);
extern int opc_cpu_isolated(int cpu);
bool opc_check_uxtop_cpu(int uxtop, int cpu);
bool opc_utask_slave(struct task_struct *t);
extern unsigned long __init opc_get_orig_capacity(int cpu);
extern void __exit opc_exit_module(void);
extern void opc_set_boost(unsigned int val);
#define UTASK_SLAVE(t) opc_utask_slave(t)

#else
#define UTASK_SLAVE(t) 0
static inline void opc_binder_pass(size_t data_size, uint32_t *data, int send) {}
static inline bool is_opc_task(struct task_struct *t, int type) { return 0; }
static inline void opc_task_switch(bool enqueue, int cpu, struct task_struct *p, u64 clock) {}
static inline int opc_get_claim_on_cpu(int cpu) { return 0; }
static inline unsigned int opc_get_claims(void) { return 0; }
static inline int opc_select_path(struct task_struct *cur, struct task_struct *t, int prev_cpu) { return OP_PATH_NORMAL; }
static inline unsigned long opc_cpu_util(unsigned long util, int cpu, struct task_struct *t, int op_path) { return util; }
static inline bool opc_fps_check(int lvl) { return false;}
static inline void opc_add_to_chain(struct task_struct *t) {}
static inline bool opc_check_uxtop_cpu(int uxtop, int cpu) { return true; }
static inline void opc_set_boost(unsigned int val) {};
#endif
#endif
