#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/oem/adj_chain.h>

struct list_head adj_chain[ADJ_CHAIN_MAX + 1];
EXPORT_SYMBOL(adj_chain);

int adj_chain_ready = 0;
EXPORT_SYMBOL(adj_chain_ready);

int adj_chain_hist_high = 0;
EXPORT_SYMBOL(adj_chain_hist_high);

static void __adj_chain_detach(struct task_struct *p)
{
	p->adj_chain_status |= 1 << AC_DETACH;
	list_del_rcu(&p->adj_chain_tasks);
	p->adj_chain_status &= ~(1 << AC_DETACH);
}

static void __adj_chain_attach(struct task_struct *p)
{
	p->adj_chain_status |= 1 << AC_ATTACH;
	list_add_tail_rcu(&p->adj_chain_tasks, &adj_chain[__adjc(get_oom_score_adj(p))]);
	p->adj_chain_status &= ~(1 << AC_ATTACH);
	if (__adjc(get_oom_score_adj(p)) > adj_chain_hist_high)
		adj_chain_hist_high = __adjc(get_oom_score_adj(p));
}

void adj_chain_update_oom_score_adj(struct task_struct *p)
{
	if (likely(adj_chain_ready)) {
		/* sync with system task_list */
		write_lock_irq(&tasklist_lock);
		spin_lock(&current->sighand->siglock);
		if (!thread_group_leader(p)) {
			pr_warn("%s '%d' not update from group leader: %s '%d', leader: %s '%d'\n",
				current->comm, current->pid,
				p->comm, p->pid, p->group_leader->comm, p->group_leader->pid);
			p = p->group_leader;
		}
		if (unlikely(p->flags & PF_EXITING))
			goto done;
		p->adj_chain_status |= 1 << AC_UPDATE_ADJ;
		__adj_chain_detach(p);
		__adj_chain_attach(p);
		p->adj_chain_status &= ~(1 << AC_UPDATE_ADJ);
done:
		spin_unlock(&current->sighand->siglock);
		write_unlock_irq(&tasklist_lock);
	}
}
EXPORT_SYMBOL(adj_chain_update_oom_score_adj);

void adj_chain_attach(struct task_struct *p)
{
	if (likely(adj_chain_ready)) {
		__adj_chain_attach(p);
	}
}
EXPORT_SYMBOL(adj_chain_attach);

void adj_chain_detach(struct task_struct *p)
{
	if (likely(adj_chain_ready)) {
		__adj_chain_detach(p);
	}
}
EXPORT_SYMBOL(adj_chain_detach);

static int init_adj_chain(void)
{
	int i = 0;

	for (i = 0; i <= ADJ_CHAIN_MAX; ++i) {
		INIT_LIST_HEAD(&adj_chain[i]);
	}
	pr_info("adj_chain init completed\n");
	adj_chain_ready = 1;
	return 0;
}

pure_initcall(init_adj_chain);
MODULE_DESCRIPTION("Oneplus adj chain");
MODULE_LICENSE("GPL v2");
