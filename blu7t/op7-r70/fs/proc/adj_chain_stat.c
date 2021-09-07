#include <linux/init.h>
#include <linux/oom.h>
#include <linux/mm.h>
#include <linux/oem/adj_chain.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int adj_chain_proc_show(struct seq_file *m, void *v)
{
	struct task_struct *tsk, *p;
	int i = 0;
	int tasksize = 0;

	rcu_read_lock();
	read_lock_irq(&tasklist_lock);
	while (i <= adj_chain_hist_high) {
		if (!list_empty(&adj_chain[i])) {
			list_for_each_entry_rcu(tsk, &adj_chain[i],
					adj_chain_tasks) {
				tasksize = 0;
				p = find_lock_task_mm(tsk);
				if (!p)
					goto just_print;
				tasksize = get_mm_rss(p->mm);
				task_unlock(p);
just_print:
				seq_printf(m, "%d tsk: %s, %d, adj %d, size %d\n",
					__adjr(i), tsk->comm, tsk->pid,
					tsk->signal->oom_score_adj, tasksize);
			}
		}
		++i;
	}
	read_unlock_irq(&tasklist_lock);
	rcu_read_unlock();
	seq_printf(m, "adj chain hist high: %d\n", __adjr(adj_chain_hist_high));
	return 0;
}

static int adj_chain_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, adj_chain_proc_show, NULL);
}

static const struct file_operations adj_chain_proc_fops = {
	.open = adj_chain_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int adj_chain_verify_proc_show(struct seq_file *m, void *v)
{
	struct task_struct *tsk, *p;
	int tasksize = 0;

	rcu_read_lock();
	for_each_process(tsk) {
		if (tsk) {
			tasksize = 0;
			p = find_lock_task_mm(tsk);
			if (!p)
				goto just_print;
			tasksize = get_mm_rss(p->mm);
			task_unlock(p);
just_print:
			seq_printf(m, "tsk: %s, %d, adj %d, size %d\n",
			tsk->comm, tsk->pid, tsk->signal->oom_score_adj,
			tasksize);
		}
	}
	rcu_read_unlock();
	return 0;
}

static int adj_chain_verify_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, adj_chain_verify_proc_show, NULL);
}

static const struct file_operations adj_chain_verify_proc_fops = {
	.open = adj_chain_verify_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init proc_adj_chain_init(void)
{
	proc_create("adj_chain_stat", S_IFREG | 0400, NULL,
					&adj_chain_proc_fops);
	proc_create("adj_chain_verify", S_IFREG | 0400, NULL,
					&adj_chain_verify_proc_fops);
	return 0;
}
fs_initcall(proc_adj_chain_init);
