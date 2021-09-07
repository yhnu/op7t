#ifndef _LINUX_ADJ_CHAIN_H
#define _LINUX_ADJ_CHAIN_H

#include <linux/list.h>
#include <linux/sched.h>
#include <uapi/linux/oom.h>
#include <linux/sched/signal.h>

#define ADJ_CHAIN_MAX ((OOM_SCORE_ADJ_MAX) - (OOM_SCORE_ADJ_MIN))

// real adj to adj chain index
#define __adjc(adj) (adj + OOM_SCORE_ADJ_MAX)

// adj chain index to real adj
#define __adjr(adj) (adj - OOM_SCORE_ADJ_MAX)

// adj chain helper
#define __adj_tasks(adj) (adj_chain[adj].adj_chain_tasks)
#define adj_tasks(adj) (adj_chain[__adjc(adj)].adj_chain_tasks)
#define get_oom_score_adj(p) (p->signal->oom_score_adj)
#define get_task_struct_adj_chain_rcu(h) \
	list_entry_rcu(h, struct task_struct, adj_chain_tasks)
enum {
	AC_ATTACH,
	AC_DETACH,
	AC_UPDATE_ADJ,
	AC_PROM_LEADER
};

#ifdef CONFIG_ADJ_CHAIN
#define adj_chain_init_list(p) INIT_LIST_HEAD(&(p)->adj_chain_tasks)
#define adj_chain_empty(adj) list_empty(&__adj_tasks(adj))
extern struct list_head adj_chain[ADJ_CHAIN_MAX + 1];
extern int adj_chain_ready;
extern int adj_chain_hist_high;
extern void adj_chain_update_oom_score_adj(struct task_struct *p);
extern void adj_chain_attach(struct task_struct *p);
extern void adj_chain_detach(struct task_struct *p);
#else
#define adj_chain_init_list(p)
#define adj_chain_empty(adj) (1)
static inline void adj_chain_update_oom_score_adj(struct task_struct *p) { }
static inline void adj_chain_attach(struct task_struct *p) { }
static inline void adj_chain_detach(struct task_struct *p) { }
#endif // CONFIG_ADJ_CHAIN

#endif // _LINUX_ADJ_CHAIN_H
