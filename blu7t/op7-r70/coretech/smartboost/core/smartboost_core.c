#include <linux/module.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/rwlock.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/rwlock_types.h>
#include <linux/mmzone.h>
#include <linux/mm.h>
#include <linux/rcupdate.h>
#include <linux/cred.h>
#include <linux/spinlock.h>
#include <linux/vmstat.h>
#include <linux/huge_mm.h>
#include <linux/mm_inline.h>
#include <linux/memcontrol.h>
#include <linux/page-flags.h>
#include <linux/ratelimit.h>
#include <linux/swap.h>
#include <linux/mmdebug.h>
#include <linux/rmap.h>
#include <linux/proc_fs.h>
#include "../../../fs/proc/internal.h"

static int sysctl_page_cache_reside_switch;
unsigned long inactive_nr, active_nr;
unsigned long priority_nr[3];
#define SMART_BOOST_PUTBACK_LRU 2
#define VMPRESSURE_COUNT 5
static atomic64_t vmpress[VMPRESSURE_COUNT];

static LIST_HEAD(hotcount_prio_list);
static DEFINE_RWLOCK(prio_list_lock);

struct hotcount_prio_node {
	unsigned int hotcount;
	uid_t uid;
	struct list_head list;
};

static unsigned int find_node_uid_prio(struct hotcount_prio_node **node,
		struct list_head **prio_pos,
		uid_t uid,
		unsigned int hotcount)
{
	struct hotcount_prio_node *pos;
	unsigned int ret_hotcount = 0;

	read_lock(&prio_list_lock);
	list_for_each_entry(pos, &hotcount_prio_list, list) {

		if (*node && *prio_pos)
			break;

		if (pos->uid == uid) {
			*node = pos;
			ret_hotcount = pos->hotcount;
		}
		if ((!(*prio_pos)) &&
			(pos->hotcount > hotcount))
			*prio_pos = &pos->list;
	}

	if (!(*prio_pos))
		*prio_pos = &hotcount_prio_list;

	read_unlock(&prio_list_lock);

	return ret_hotcount;
}

static void insert_prio_node(unsigned int new_hotcount, uid_t uid)
{
	struct hotcount_prio_node *node = NULL;
	struct list_head *prio_pos = NULL;
	unsigned int old_hotcount;

	old_hotcount = find_node_uid_prio(&node, &prio_pos, uid, new_hotcount);

	if (node) {
		if (old_hotcount == new_hotcount)
			return;

		write_lock(&prio_list_lock);

		if (&node->list == prio_pos) {
			node->hotcount = new_hotcount;
			goto unlock;
		}
		list_del(&node->list);
	} else {
		node = (struct hotcount_prio_node *)
			kmalloc(sizeof(struct hotcount_prio_node), GFP_KERNEL);
		if (!node) {
			pr_err("no memory to insert prio_node!\n");
			return;
		}
		node->uid = uid;
		write_lock(&prio_list_lock);
	}

	node->hotcount = new_hotcount;
	list_add_tail(&node->list, prio_pos);
unlock:
	write_unlock(&prio_list_lock);
}

static void delete_prio_node(uid_t uid)
{
	struct hotcount_prio_node *pos;
	int found = 0;

	read_lock(&prio_list_lock);
	list_for_each_entry(pos, &hotcount_prio_list, list)
		if (pos->uid == uid) {
			found = 1;
			break;
		}
	read_unlock(&prio_list_lock);

	if (found) {
		write_lock(&prio_list_lock);
		list_del(&pos->list);
		write_unlock(&prio_list_lock);
		kfree(pos);
	}
}

static void print_prio_chain(struct seq_file *m)
{
	struct hotcount_prio_node *pos;

	read_lock(&prio_list_lock);
	list_for_each_entry(pos, &hotcount_prio_list, list)
		seq_printf(m, "%d(%d)\t", pos->uid, pos->hotcount);
	read_unlock(&prio_list_lock);

	seq_putc(m, '\n');
}

#define UID_HASH_ORDER 5
#define uid_hashfn(nr)	hash_long((unsigned long)nr, UID_HASH_ORDER)

static struct uid_node **alloc_uid_hash_table(void)
{
	struct uid_node **hash_table;
	int size = (1 << UID_HASH_ORDER) * sizeof(struct uid_node *);

	if (size <= PAGE_SIZE)
		hash_table = kzalloc(size, GFP_ATOMIC);
	else
		hash_table = (struct uid_node **)__get_free_pages(
				GFP_ATOMIC | __GFP_ZERO, get_order(size));
	if (!hash_table)
		return NULL;
	return hash_table;
}

static struct uid_node *alloc_uid_node(uid_t uid)
{
	struct uid_node *uid_nd;

	uid_nd = kzalloc(sizeof(struct uid_node), GFP_ATOMIC);
	if (!uid_nd)
		return NULL;
	uid_nd->uid = uid;
	uid_nd->hot_count = 0; /* initialize a new UID's count */
	uid_nd->next = NULL;
	INIT_LIST_HEAD(&uid_nd->page_cache_list);
	return uid_nd;
}

static struct uid_node *insert_uid_node(struct uid_node **hash_table, uid_t uid)
{
	struct uid_node *puid;
	unsigned int index, sise = 1 << UID_HASH_ORDER;

	index = uid_hashfn((unsigned long)uid);
	if (index >= sise)
		return NULL;
	puid = alloc_uid_node(uid);

	if (!puid)
		return NULL;

	rcu_assign_pointer(puid->next, hash_table[index]);
	rcu_assign_pointer(hash_table[index], puid);
	return puid;
}

static struct uid_node *find_uid_node(uid_t uid, struct lruvec *lruvec)
{
	struct uid_node *uid_nd, *ret = NULL;
	unsigned int index;

	index = uid_hashfn((unsigned int)uid);

	if (lruvec->uid_hash == NULL)
		return NULL;
	if (index >= (1 << UID_HASH_ORDER))
		return NULL;
	for (uid_nd = rcu_dereference(lruvec->uid_hash[index]);
		uid_nd != NULL; uid_nd = rcu_dereference(uid_nd->next)) {
		if (uid_nd->uid == uid) {
			ret = uid_nd;
			break;
		}
	}
	return ret;
}

void free_hash_table(struct lruvec *lruvec)
{
	int i, table_num;
	struct uid_node *puid, **np;

	table_num = 1 << UID_HASH_ORDER;
	for (i = 0; i < table_num; i++) {
		np = &lruvec->uid_hash[i];
		while ((puid = rcu_dereference(*np)) != NULL) {
			rcu_assign_pointer(*np, rcu_dereference(puid->next));
			kfree_rcu(puid, rcu);
		}
	}
}

__always_inline
bool ctech_smb_update_uid_lru_size(struct page *page,
				struct lruvec *lruvec, enum lru_list lru)
{
	struct pglist_data *pgdata = lruvec_pgdat(lruvec);

	if (is_file_lru(lru) && PageUIDLRU(page)) {
		ClearPageUIDLRU(page);
		__mod_zone_page_state(
			&pgdata->node_zones[page_zonenum(page)],
			NR_ZONE_UID_LRU, -hpage_nr_pages(page));
		return true;
	} else
		return false;
}


static void _uid_lru_add_fn(struct page *page, struct lruvec *lruvec)
{
	struct uid_node *uid_nd;
	unsigned long flag;
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	uid_t uid = __task_cred(current)->user->uid.val;

	if (!pgdat)
		pgdat = page_pgdat(page);

	get_page(page);
	spin_lock_irqsave(&pgdat->lru_lock, flag);
	VM_BUG_ON_PAGE(PageAnon(page), page);
	VM_BUG_ON_PAGE(PageLRU(page), page);
	VM_BUG_ON_PAGE(PageUIDLRU(page), page);
	SetPageUIDLRU(page);
	SetPageLRU(page);
	uid_nd = find_uid_node(uid, lruvec);
	if (uid_nd == NULL) {
		if (lruvec->uid_hash == NULL)
			lruvec->uid_hash = alloc_uid_hash_table();
		uid_nd = insert_uid_node(lruvec->uid_hash, uid);
	}
	list_add(&page->lru, &uid_nd->page_cache_list);
	mod_zone_page_state(page_zone(page), NR_ZONE_UID_LRU,
					hpage_nr_pages(page));
	spin_unlock_irqrestore(&pgdat->lru_lock, flag);
	put_page(page);
}

static void __uid_lru_cache_add(struct page *page)
{
	struct pglist_data *pagepgdat = page_pgdat(page);
	struct lruvec *lruvec;

	lruvec = mem_cgroup_page_lruvec(page, pagepgdat);
	_uid_lru_add_fn(page, lruvec);
}

static unsigned long isolate_uid_lru_pages(struct page *page)
{
	int ret = -EBUSY;

	//VM_BUG_ON_PAGE(!page_count(page), page);
	WARN_RATELIMIT(PageTail(page), "trying to isolate tail page");

	if (PageLRU(page)) {
		struct zone *zone = page_zone(page);
		struct lruvec *lruvec;
		int lru = page_lru(page);

		if (unlikely(!get_page_unless_zero(page)))
			return ret;

		if (PageUnevictable(page))
			return ret;

		lruvec = mem_cgroup_page_lruvec(page, zone->zone_pgdat);
		ClearPageLRU(page);
		del_page_from_lru_list(page, lruvec, lru);
		ret = 0;
	}

	return ret;
}

static bool cache_is_low(void)
{
	unsigned long cache =
		global_node_page_state(NR_FILE_PAGES) - total_swapcache_pages();

	if (cache < get_max_minfree())
		return true;

	return false;
}

bool ctech_smb_uid_lru_add(struct page *page)
{
	if (!sysctl_page_cache_reside_switch)
		return false;

	if (cache_is_low())
		return false;

	if (!current->group_leader->hot_count)
		return false;

	VM_BUG_ON_PAGE(PageActive(page) && PageUnevictable(page), page);
	VM_BUG_ON_PAGE(PageLRU(page), page);
	__uid_lru_cache_add(page);

	return true;
}

static unsigned long
smb_isolate_pages_by_uid(struct list_head *page_list, uid_t uid)
{
	LIST_HEAD(putback_page_list);
	struct pglist_data *pgdat;
	struct mem_cgroup *memcg;
	struct uid_node *node;
	struct lruvec *lruvec;
	unsigned long nr_isolate = 0, nr_isolate_failed = 0;
	struct page *page;

	for_each_online_pgdat(pgdat) {
		memcg = mem_cgroup_iter(NULL, NULL, NULL);
		do {
			lruvec = mem_cgroup_lruvec(pgdat, memcg);
			if (!lruvec)
				goto next;
			spin_lock_irq(&pgdat->lru_lock);
			node = find_uid_node(uid, lruvec);
			if (!node) {
				spin_unlock_irq(&pgdat->lru_lock);
				goto next;
			}

			while (!list_empty(&node->page_cache_list)) {
				page = lru_to_page(&node->page_cache_list);
				VM_BUG_ON_PAGE(!PageUIDLRU(page), page);

				if (isolate_uid_lru_pages(page)) {
					list_move(&page->lru,
						&putback_page_list);
					nr_isolate_failed++;
					continue;
				}

				ClearPageActive(page);
				list_add(&page->lru, page_list);
				nr_isolate++;
				inc_node_page_state(page, NR_ISOLATED_ANON +
					page_is_file_cache(page));
			}

			list_splice_init(&putback_page_list,
					&node->page_cache_list);
			spin_unlock_irq(&pgdat->lru_lock);
next:
			memcg = mem_cgroup_iter(NULL, memcg, NULL);
		} while (memcg);
	}
	return nr_isolate;
}


static unsigned long uid_lru_size(void)
{
	return global_zone_page_state(NR_ZONE_UID_LRU);
}

static int suitable_reclaim_check(struct lruvec *lruvec)
{
	unsigned long active = global_zone_page_state(NR_ZONE_ACTIVE_FILE);
	unsigned long inactive = global_zone_page_state(NR_ZONE_INACTIVE_FILE);
	unsigned long total_uid_lru_nr = uid_lru_size();

	if ((active + inactive) > get_max_minfree())
		return ((active + inactive) << 3) < total_uid_lru_nr;
	else
		return SMART_BOOST_PUTBACK_LRU;
}

static bool suitable_isolate_in_direct_reclaim(int priority,
		bool enough_list_reclaimed)
{
	bool need_isolate = false;

	if (current_is_kswapd())
		need_isolate = false;

	if (priority <= 11 && !enough_list_reclaimed)
		need_isolate = true;

	return need_isolate;
}

unsigned long ctech_smb_isolate_list_or_putbcak(struct list_head *page_list,
	struct lruvec *lruvec, struct pglist_data *pgdat, int priority,
	bool enough_list_reclaimed)
{
	LIST_HEAD(putback_list);
	unsigned long nr_isolated = 0, nr_isolate_failed = 0;
	unsigned long uid_size = uid_lru_size();
	long nr_to_shrink = uid_size >> priority;
	int stat = suitable_reclaim_check(lruvec);
	struct hotcount_prio_node *pos;
	struct page *page;

	if (!sysctl_page_cache_reside_switch)
		return 0;

	if (!stat &&
		!suitable_isolate_in_direct_reclaim(priority,enough_list_reclaimed))
		return 0;

	if (uid_size <= 0)
		return 0;

	if (stat == SMART_BOOST_PUTBACK_LRU)
		nr_to_shrink = uid_size;

	read_lock(&prio_list_lock);
	spin_lock_irq(&pgdat->lru_lock);
	list_for_each_entry(pos, &hotcount_prio_list, list) {
		struct uid_node *tmp_uid_list = find_uid_node(pos->uid, lruvec);

		if (!nr_to_shrink)
			break;

		if (tmp_uid_list == NULL)
			continue;

		while (!list_empty(&tmp_uid_list->page_cache_list)) {
			page = lru_to_page(&tmp_uid_list->page_cache_list);
			VM_BUG_ON_PAGE(!PageUIDLRU(page), page);

			if (isolate_uid_lru_pages(page)) {
				list_move(&page->lru, &putback_list);
				nr_isolate_failed++;
				continue;
			}

			ClearPageActive(page);
			list_add(&page->lru, page_list);
			nr_to_shrink--;
			nr_isolated++;
			if (!nr_to_shrink)
				break;
		}

		list_splice_init(&putback_list, &tmp_uid_list->page_cache_list);
	}
	spin_unlock_irq(&pgdat->lru_lock);
	read_unlock(&prio_list_lock);

	if (stat == SMART_BOOST_PUTBACK_LRU)
		while (!list_empty(page_list)) {
			page = lru_to_page(page_list);
			list_del(&page->lru);
			putback_lru_page(page);
			nr_isolated--;
		}

	return nr_isolated;

}

static void uid_lru_info_show_print(struct seq_file *m, pg_data_t *pgdat)
{
	int i;
	struct uid_node **table;
	struct list_head *pos;
	unsigned long nr_pages;
	struct mem_cgroup *memcg;

	seq_puts(m, "vmpressure:\n0_20\t20_40\t40_60\t60_80\t80_100\n");
	for (i = 0; i < VMPRESSURE_COUNT; i++)
		seq_printf(m, "%lu\t", atomic64_read(&vmpress[i]));

	seq_printf(m, "\nNode %d\n", pgdat->node_id);
	seq_puts(m, "uid_lru_list priority:\n");
	print_prio_chain(m);
	seq_puts(m, "uid\thot_count\tpages\n");

	memcg = mem_cgroup_iter(NULL, NULL, NULL);
	do {
		struct lruvec *lruvec = mem_cgroup_lruvec(pgdat, memcg);

		if (!lruvec)
			goto next;

		table = lruvec->uid_hash;
		if (!table)
			goto next;

		for (i = 0; i < (1 << 5); i++) {
			struct uid_node *node = rcu_dereference(table[i]);

			if (!node)
				continue;

			do {
				nr_pages = 0;
				list_for_each(pos, &node->page_cache_list)
					nr_pages++;
				seq_printf(m, "%d\t%d\t%lu\n",
					node->uid,
					node->hot_count,
					nr_pages);
			} while ((node = rcu_dereference(node->next)) != NULL);
		}
next:
		memcg = mem_cgroup_iter(NULL, memcg, NULL);
	} while (memcg);
	seq_putc(m, '\n');
}
/*
 * Output information about zones in @pgdat.
 */
static int uid_lru_info_show(struct seq_file *m, void *arg)
{
	pg_data_t *pgdat = (pg_data_t *)arg;

	uid_lru_info_show_print(m, pgdat);

	return 0;
}
static void *uid_lru_info_start(struct seq_file *m, loff_t *pos)
{
	pg_data_t *pgdat;
	loff_t node = *pos;

	for (pgdat = first_online_pgdat();
	     pgdat && node;
	     pgdat = next_online_pgdat(pgdat))
		--node;

	return pgdat;
}

static void *uid_lru_info_next(struct seq_file *m, void *arg, loff_t *pos)
{
	pg_data_t *pgdat = (pg_data_t *)arg;

	(*pos)++;
	return next_online_pgdat(pgdat);
}

static void uid_lru_info_stop(struct seq_file *m, void *arg)
{
}

static const struct seq_operations uid_lru_info_op = {
	.start	= uid_lru_info_start,
	.next	= uid_lru_info_next,
	.stop	= uid_lru_info_stop,
	.show	= uid_lru_info_show,
};

static int uid_lru_info_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &uid_lru_info_op);
}

static const struct file_operations proc_uid_lru_info_file_operations = {
	.open		= uid_lru_info_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int smb_vmpressure_notifier(struct notifier_block *nb,
			unsigned long action, void *data)
{
	unsigned long pressure = action;

	if (pressure < 20)
		atomic64_inc(&vmpress[0]);
	else if (pressure < 40)
		atomic64_inc(&vmpress[1]);
	else if (pressure < 60)
		atomic64_inc(&vmpress[2]);
	else if (pressure < 80)
		atomic64_inc(&vmpress[3]);
	else
		atomic64_inc(&vmpress[4]);

	return 0;
}

static struct notifier_block smb_vmpressure_statistic = {
	.notifier_call = smb_vmpressure_notifier,
};

static void smart_boost_reclaim_by_uid(uid_t uid)
{
	LIST_HEAD(page_list);
	unsigned long nr_isolate = 0;
	unsigned long nr_reclaimed = 0;

	nr_isolate = smb_isolate_pages_by_uid(&page_list, uid);
	if (!nr_isolate)
		return;

	nr_reclaimed = coretech_reclaim_pagelist(&page_list, NULL, NULL);

	pr_err("clean uid(%d) pagecache:%d\n", uid, nr_reclaimed);
}

static ssize_t page_hot_count_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct task_struct *task;
	char buffer[PROC_NUMBUF];
	size_t len;
	int page_hot_count;

	task = get_proc_task(file_inode(file));

	if (!task)
		return -ESRCH;

	page_hot_count = task->hot_count;

	put_task_struct(task);

	len = snprintf(buffer, sizeof(buffer), "%d\n", page_hot_count);
	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t page_hot_count_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct task_struct *task;
	char buffer[PROC_NUMBUF];
	int page_hot_count;
	int err;
	uid_t uid;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count)) {
		err = -EFAULT;
		goto out;
	}

	err = kstrtoint(strstrip(buffer), 0, &page_hot_count);
	if (err)
		goto out;

	task = get_proc_task(file_inode(file));
	if (!task) {
		err = -ESRCH;
		goto out;
	}

	task->hot_count = page_hot_count;
	uid = __task_cred(task)->user->uid.val;

	if (!page_hot_count) {
		smart_boost_reclaim_by_uid(uid);
		delete_prio_node(uid);
	} else {
		insert_prio_node(page_hot_count, uid);
	}

	put_task_struct(task);

out:
	return err < 0 ? err : count;
}

const struct file_operations proc_page_hot_count_operations = {
	.read		= page_hot_count_read,
	.write		= page_hot_count_write,
};

static int __init smartboost_init(void)
{
	struct smb_cb_set set;

	vmpressure_notifier_register(&smb_vmpressure_statistic);

	proc_create("uid_lru_info", 0444, NULL,
				&proc_uid_lru_info_file_operations);

	set.smb_uid_lru_add_cb = ctech_smb_uid_lru_add;
	set.smb_isolate_list_or_putbcak_cb = ctech_smb_isolate_list_or_putbcak;
	set.smb_update_uid_lru_size_cb = ctech_smb_update_uid_lru_size;
	smb_register_cb_set(&set);

	return 0;
}

module_param_named(page_cache_reside_switch,
			sysctl_page_cache_reside_switch,
			uint, 0644);

module_init(smartboost_init)

