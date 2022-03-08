#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/mm.h>
#include <linux/module.h>
#include <linux/gfp.h>
#include <linux/kernel_stat.h>
#include <linux/pagemap.h>
#include <linux/init.h>
#include <linux/highmem.h>
#include <linux/vmpressure.h>
#include <linux/vmstat.h>
#include <linux/file.h>
#include <linux/writeback.h>
#include <linux/blkdev.h>
#include <linux/mm_inline.h>
#include <linux/backing-dev.h>
#include <linux/rmap.h>
#include <linux/topology.h>
#include <linux/cpu.h>
#include <linux/cpuset.h>
#include <linux/compaction.h>
#include <linux/notifier.h>
#include <linux/rwsem.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/memcontrol.h>
#include <linux/delayacct.h>
#include <linux/sysctl.h>
#include <linux/oom.h>
#include <linux/prefetch.h>
#include <linux/printk.h>
#include <linux/dax.h>

#include <asm/tlbflush.h>
#include <asm/div64.h>

#include <linux/swapops.h>
#include <linux/balloon_compaction.h>
#include <linux/swap.h>
#include <linux/sched/mm.h>
#include <linux/page-flags.h>
#include <linux/swapfile.h>
#include <linux/pagevec.h>
#include <linux/fs.h>
#include "memplus.h"

#define PF_NO_TAIL(page, enforce) ({					\
		VM_BUG_ON_PGFLAGS(enforce && PageTail(page), page);	\
		compound_head(page);})

#define RD_SIZE 128
#define DEBUG_TIME_INFO 0
#define FEAT_RECLAIM_LIMIT 0
struct reclaim_stat {
	unsigned nr_dirty;
	unsigned nr_unqueued_dirty;
	unsigned nr_congested;
	unsigned nr_writeback;
	unsigned nr_immediate;
	unsigned nr_activate;
	unsigned nr_ref_keep;
	unsigned nr_unmap_fail;
};

struct mp_reclaim_param {
	struct vm_area_struct *vma;
	/* Number of pages scanned */
	int nr_scanned;
#if FEAT_RECLAIM_LIMIT
	unsigned long start_jiffies;
#endif
	/* max pages to reclaim */
	int nr_to_reclaim;
	/* pages reclaimed */
	int nr_reclaimed;
	int type;
};

struct reclaim_data {
	pid_t pid;
	int prev_adj;
};

struct reclaim_info {
	struct reclaim_data rd[RD_SIZE];
	int i_idx;
	int o_idx;
	int count;
};

struct reclaim_info ri = { {{0}}, 0, 0, 0 };
struct reclaim_info si = { {{0}}, 0, 0, 0 };
static struct task_struct *reclaimd_tsk = NULL;
static struct task_struct *swapind_tsk = NULL;
static DEFINE_SPINLOCK(rd_lock);

/* -1 = system free to use swap, 0 = disable retention, swap not available, 1 = enable retention */
static int vm_memory_plus __read_mostly = 0;
static unsigned long memplus_add_to_swap = 0;
unsigned long coretech_reclaim_pagelist(struct list_head *page_list, struct vm_area_struct *vma, void *sc);
unsigned long swapout_to_zram(struct list_head *page_list, struct vm_area_struct *vma);
unsigned long swapout_to_disk(struct list_head *page_list, struct vm_area_struct *vma);

bool ctech_current_is_swapind(void) {
	return current == swapind_tsk;
}

bool ctech_memplus_check_isolate_page(struct page*page)
{
	return (memplus_enabled() && (!PageSwapCache(page) || PageWillneed(page)));
}

static bool is_fast_entry(swp_entry_t entry)
{
	struct swap_info_struct *p;
	bool ret = false;
	unsigned long offset, type;

	if (!entry.val)
		goto out;
	type = swp_type(entry);
	p = swap_info[type];
	if (!(p->flags & SWP_USED))
		goto out;
	offset = swp_offset(entry);
	if (offset >= p->max)
		goto out;

	spin_lock(&p->lock);
	if (p->flags & SWP_SYNCHRONOUS_IO)
		ret = true;
	spin_unlock(&p->lock);
out:
	return ret;
}
static bool enough_swap_size(unsigned long req_size, int swap_bdv)
{
	bool ret = false;
	unsigned int n = 0;
	struct swap_info_struct *sis, *next;
	unsigned int node = numa_node_id();

	if (swap_bdv > 1)
		return ret;

	spin_lock(&swap_lock);
	plist_for_each_entry_safe(sis, next, &swap_avail_heads[node], avail_lists[node]) {
		int fast_i = (sis->flags & SWP_SYNCHRONOUS_IO)? 1:0;

		if (fast_i == swap_bdv) {
			spin_lock(&sis->lock);
			if (sis->flags & SWP_WRITEOK) {
				n += sis->pages - sis->inuse_pages;
				if (n > req_size) {
					ret = true;
					spin_unlock(&sis->lock);
					goto unlock;
				}
			}
			spin_unlock(&sis->lock);
		}
	}

unlock:
	spin_unlock(&swap_lock);
	return ret;
}

__always_inline void ctech_memplus_move_swapcache_to_anon_lru(struct page *page)
{
	struct zone *zone = page_zone(page);
	unsigned long flag;

	if (memplus_enabled()) {
		spin_lock_irqsave(zone_lru_lock(zone), flag);
		if (PageLRU(page)) {
			struct lruvec *lruvec;
			enum lru_list lru, oldlru = page_lru(page);

			clear_bit(PG_swapcache, &(PF_NO_TAIL(page, 1))->flags);
			lru = page_lru(page);
			lruvec = mem_cgroup_page_lruvec(page, zone->zone_pgdat);
			list_move(&page->lru, &lruvec->lists[lru]);
			update_lru_size(lruvec, oldlru, page_zonenum(page), -hpage_nr_pages(page));
			update_lru_size(lruvec, lru, page_zonenum(page), hpage_nr_pages(page));
		} else
			clear_bit(PG_swapcache, &(PF_NO_TAIL(page, 1))->flags);
		spin_unlock_irqrestore(zone_lru_lock(zone), flag);
	}else
		clear_bit(PG_swapcache, &(PF_NO_TAIL(page, 1))->flags);
}

__always_inline void ctech_memplus_move_anon_to_swapcache_lru(struct page *page)
{
	struct zone *zone = page_zone(page);
	unsigned long flag;

	if (memplus_enabled()) {
		spin_lock_irqsave(zone_lru_lock(zone), flag);
		if (likely(!PageLRU(page)))
			set_bit(PG_swapcache, &(PF_NO_TAIL(page, 1))->flags);
		else {
			struct lruvec *lruvec;
			enum lru_list lru, oldlru = page_lru(page);

			set_bit(PG_swapcache, &(PF_NO_TAIL(page, 1))->flags);
			lru = page_lru(page);
			lruvec = mem_cgroup_page_lruvec(page, zone->zone_pgdat);
			list_move(&page->lru, &lruvec->lists[lru]);
			update_lru_size(lruvec, oldlru, page_zonenum(page), -hpage_nr_pages(page));
			update_lru_size(lruvec, lru, page_zonenum(page), hpage_nr_pages(page));
		}
		spin_unlock_irqrestore(zone_lru_lock(zone), flag);
	} else
		set_bit(PG_swapcache, &(PF_NO_TAIL(page, 1))->flags);
}
extern int do_swap_page(struct vm_fault *vmf);
static int memplus_swapin_walk_pmd_entry(pmd_t *pmd, unsigned long start,
	unsigned long end, struct mm_walk *walk)
{
	pte_t *orig_pte;
	struct vm_area_struct *vma = walk->private;
	unsigned long index;
	int ret = 0;

	if (pmd_none_or_trans_huge_or_clear_bad(pmd))
		return 0;

	for (index = start; index != end; index += PAGE_SIZE) {
		pte_t pte;
		swp_entry_t entry;
		struct page *page;
		spinlock_t *ptl;

		if (!list_empty(&vma->vm_mm->mmap_sem.wait_list))
			return -1;

		orig_pte = pte_offset_map_lock(vma->vm_mm, pmd, start, &ptl);
		pte = *(orig_pte + ((index - start) / PAGE_SIZE));
		pte_unmap_unlock(orig_pte, ptl);

		if (pte_present(pte) || pte_none(pte))
			continue;
		entry = pte_to_swp_entry(pte);
		if (unlikely(non_swap_entry(entry)))
			continue;

		if (is_fast_entry(entry)) {
			struct vm_fault fe = {
				.vma = vma,
				.pte = &pte,
				.orig_pte = pte,
				.address = index,
				.flags = FAULT_FLAG_ALLOW_RETRY | FAULT_FLAG_RETRY_NOWAIT,
				.pmd = pmd,
				.vma_flags = vma->vm_flags,
				.vma_page_prot = vma->vm_page_prot,
			};

			ret = do_swap_page(&fe);
			if (ret & VM_FAULT_ERROR) {
				printk(KERN_ERR "%s: do_swap_page ERROR\n", __func__);
				return -1;
			}
			continue;
		} else
			page = read_swap_cache_async(entry, GFP_HIGHUSER_MOVABLE,
								vma, index, false);
		if (page)
			put_page(page);
	}

	return 0;
}

static void enqueue_reclaim_data(pid_t nr, int prev_adj, struct reclaim_info *info)
{
	int idx;
	struct task_struct *waken_task;

	waken_task = (info == &ri? reclaimd_tsk : swapind_tsk);
	if (!waken_task)
		return;

	spin_lock(&rd_lock);
	if (info->count < RD_SIZE) {
		info->count++;
		idx = info->i_idx++ % RD_SIZE;
		info->rd[idx].pid = nr;
		info->rd[idx].prev_adj = prev_adj;
	}
	spin_unlock(&rd_lock);
	BUG_ON(info->count > RD_SIZE || info->count < 0);

	if (waken_task->state == TASK_INTERRUPTIBLE)
		wake_up_process(waken_task);
}

bool ctech_memplus_enabled(void)
{
	return vm_memory_plus > 0 && total_swap_pages;
}

__always_inline bool ctech__memplus_enabled(void)
{
	return vm_memory_plus > 0;
}

static void __memplus_state_check(int cur_adj, int prev_adj, struct task_struct* task)
{
	int uid = task_uid(task).val;

	if (!memplus_enabled())
		return;

	/* special case that SmartBoost toggle disables this feature */
	if (vm_memory_plus == 2)
		goto queue_swapin;

	if (task->signal->memplus_type == TYPE_SYS_IGNORE)
		return;

	if (task->signal->memplus_type == TYPE_WILL_NEED)
		goto queue_swapin;

	if (unlikely((prev_adj == -1) || (cur_adj == prev_adj)))
		return;

	if (uid < AID_APP) {
		//trace_printk("QUIT-reclaim %s (pid %d) (adj %d -> %d) (uid %d)\n", task->comm, task->pid, prev_adj, cur_adj, uid);
		return;
	}

	if (cur_adj >= 800 && time_after_eq(jiffies, task->signal->reclaim_timeout)) {
		spin_lock(&task->signal->reclaim_state_lock);
		/* reclaim should not kick-in within 2 secs */
		task->signal->reclaim_timeout = jiffies + 2*HZ;

		if (task->signal->swapin_should_readahead_m == RECLAIM_STANDBY) {
			task->signal->swapin_should_readahead_m = RECLAIM_QUEUE;
			//trace_printk("Q-reclaim %s (pid %d) (adj %d -> %d) (uid %d)\n", task->comm, task->pid, prev_adj, cur_adj, uid);
			enqueue_reclaim_data(task->pid, prev_adj, &ri);
		}
		spin_unlock(&task->signal->reclaim_state_lock);
	} else if (cur_adj == 0) {
queue_swapin:
		spin_lock(&task->signal->reclaim_state_lock);
		/* swapin kicked in, don't reclaim within 2 secs */
		task->signal->reclaim_timeout = jiffies + 2*HZ;

		if (task->signal->swapin_should_readahead_m == RECLAIM_QUEUE) {
			task->signal->swapin_should_readahead_m = RECLAIM_STANDBY;
		} else if (task->signal->swapin_should_readahead_m == RECLAIM_DONE) {
			task->signal->swapin_should_readahead_m = SWAPIN_QUEUE;
			//trace_printk("Q-swapin %s (pid %d) (adj %d -> %d) (uid %d)\n", task->comm, task->pid, prev_adj, cur_adj, uid);
			enqueue_reclaim_data(task->pid, prev_adj, &si);
		}
		spin_unlock(&task->signal->reclaim_state_lock);
	}
}

void ctech_memplus_state_check(bool legacy, int oom_adj, struct task_struct *task, int type, int update)
{
	int oldadj = task->signal->oom_score_adj;

	if (update) {
		if (type >= TYPE_END || type < 0)
			return;
		task->signal->memplus_type = type;

		if (type == TYPE_WILL_NEED)
			oom_adj = 0;
		else if ((type & MEMPLUS_TYPE_MASK) < TYPE_SYS_IGNORE)
			oom_adj = oldadj;
		else
			return;
	}

	if (!legacy && (oom_adj >= 800 || oom_adj == 0))
		__memplus_state_check(oom_adj, oldadj, task);
}

static bool dequeue_reclaim_data(struct reclaim_data *data, struct reclaim_info *info)
{
	int idx;
	bool has_data = false;

	spin_lock(&rd_lock);
	if (info->count > 0) {
		has_data = true;
		info->count--;
		idx = info->o_idx++ % RD_SIZE;
		*data = info->rd[idx];
	}
	spin_unlock(&rd_lock);
	BUG_ON(info->count > RD_SIZE || info->count < 0);

	return has_data;
}

static ssize_t swapin_anon(struct task_struct *task, int prev_adj)
{
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	struct mm_walk walk = {};
	int task_anon = 0, task_swap = 0, err = 0;
	//u64 time_ns = 0;
#if DEBUG_TIME_INFO
	struct timespec ts1, ts2;
	getnstimeofday(&ts1);
#endif

retry:
	/* TODO: do we need to use p = find_lock_task_mm(tsk); in case main thread got killed */
	mm = get_task_mm(task);
	if (!mm)
		goto out;

	/* system pid may reach its max value and this pid was reused by other process */
	if (unlikely(task->signal->swapin_should_readahead_m != SWAPIN_QUEUE)) {
		mmput(mm);
		return 0;
	}

	task_anon = get_mm_counter(mm, MM_ANONPAGES);
	task_swap = get_mm_counter(mm, MM_SWAPENTS);

	/* swapin only for large APP, flip 33000, bow 60000, eightpoll 16000 */
	if (task_swap <= 10000) {
		mmput(mm);
		//trace_printk("SMALL swapin: this task is too small\n");
		goto out;
	}

	walk.mm = mm;
	walk.pmd_entry = memplus_swapin_walk_pmd_entry;

	down_read(&mm->mmap_sem);

	for (vma = mm->mmap; vma; vma = vma->vm_next) {
		if (vma->memplus_flags)
			continue;

		if (is_vm_hugetlb_page(vma))
			continue;

		if (vma->vm_file)
			continue;

		/* if mlocked, don't reclaim it */
		if (vma->vm_flags & VM_LOCKED)
			continue;

		walk.private = vma;
		err = walk_page_range(vma->vm_start, vma->vm_end, &walk);
		if (err == -1)
			break;
		vma->memplus_flags = 1;
	}

	flush_tlb_mm(mm);
	up_read(&mm->mmap_sem);
	mmput(mm);
	if (err) {
		err = 0;
		//schedule();
		goto retry;
	}
out:
		/* TODO */
	lru_add_drain();	/* Push any new pages onto the LRU now */
#if DEBUG_TIME_INFO
	getnstimeofday(&ts2);
	ts2 = timespec_sub(ts2, ts1);
	time_ns = timespec_to_ns(&ts2);
#endif
	//trace_printk("%s (pid %d)(size %d-%d) (adj %d -> %d) consumed %llu ms %llu us\n", task->comm, task->pid, task_anon, task_swap, prev_adj, task->signal->oom_score_adj, (time_ns/1000000), (time_ns/1000)%1000);

	spin_lock(&task->signal->reclaim_state_lock);
	task->signal->swapin_should_readahead_m = RECLAIM_STANDBY;
	spin_unlock(&task->signal->reclaim_state_lock);

	return 0;
}

//TODO: blk_plug don't seem to work
static int swapind_fn(void *p)
{
	struct reclaim_data data;
	struct task_struct *tsk;

	set_freezable();
	for ( ; ; ) {
		while (!pm_freezing && dequeue_reclaim_data(&data, &si)) {
			rcu_read_lock();
			tsk = find_task_by_vpid(data.pid);

			/* KTHREAD is almost impossible to hit this */
			//if (tsk->flags & PF_KTHREAD) {
			//  rcu_read_unlock();
			//	continue;
			//}

			if (!tsk) {
				rcu_read_unlock();
				continue;
			}

			get_task_struct(tsk);
			rcu_read_unlock();

			swapin_anon(tsk, data.prev_adj);
			put_task_struct(tsk);
		}

		set_current_state(TASK_INTERRUPTIBLE);
		freezable_schedule();

		if (kthread_should_stop())
			break;
	}

	return 0;
}

static int memplus_reclaim_pte(pmd_t *pmd, unsigned long addr,
				unsigned long end, struct mm_walk *walk)
{
	struct mp_reclaim_param *rp = walk->private;
	struct vm_area_struct *vma = rp->vma;
	pte_t *pte, ptent;
	spinlock_t *ptl;
	struct page *page;
	LIST_HEAD(page_list);
	int isolated;
	int reclaimed = 0;
	int reclaim_type = rp->type;

	split_huge_pmd(vma, addr, pmd);
	if (pmd_trans_unstable(pmd) || !rp->nr_to_reclaim)
		return 0;
cont:
	isolated = 0;
	pte = pte_offset_map_lock(vma->vm_mm, pmd, addr, &ptl);
	for (; addr != end; pte++, addr += PAGE_SIZE) {
		ptent = *pte;
		if (!pte_present(ptent))
			continue;

		page = vm_normal_page(vma, addr, ptent);
		if (!page)
			continue;

		ClearPageWillneed(page);

		if ((reclaim_type == TYPE_NORMAL) && PageSwapCache(page))
			continue;

		/* About 11% of pages have more than 1 map_count
		 * only take care mapcount == 1 is good enough */
		if (page_mapcount(page) != 1)
			continue;

		if (isolate_lru_page(page))
			continue;

		if (PageAnon(page) && !PageSwapBacked(page)) {
			putback_lru_page(page);
			continue;
		}

		list_add(&page->lru, &page_list);
		inc_node_page_state(page, NR_ISOLATED_ANON +
				page_is_file_cache(page));
		isolated++;
		rp->nr_scanned++;

		if ((isolated >= SWAP_CLUSTER_MAX) || !rp->nr_to_reclaim)
			break;
	}
	pte_unmap_unlock(pte - 1, ptl);

	memplus_add_to_swap += isolated;

	if (reclaim_type == TYPE_NORMAL && !enough_swap_size(isolated, TYPE_NORMAL))
		reclaim_type = TYPE_FREQUENT;

	if (reclaim_type == TYPE_NORMAL)
		reclaimed = swapout_to_disk(&page_list, vma);
	else if (reclaim_type == TYPE_FREQUENT)
		reclaimed = swapout_to_zram(&page_list, vma);

	rp->nr_reclaimed += reclaimed;
	rp->nr_to_reclaim -= reclaimed;
	if (rp->nr_to_reclaim < 0)
		rp->nr_to_reclaim = 0;

#if FEAT_RECLAIM_LIMIT
	/* TODO: early quit */
	/* timeout (range from 10~20ms), emergency quit back to reclaim_anon() */
	/* statistics shows 90% of reclaim finish within 60ms, should be a good timeout value */
	/* statistics shows 80% of reclaim finish within 26ms, should be a good timeout value */
	/* statistics shows 77% of reclaim finish within 20ms, should be a good timeout value */
	/* statistics shows 68% of reclaim finish within 10ms, should be a good timeout value */
	//if (time_after_eq(jiffies, rp->start_jiffies + 2)) {
	//	rp->nr_to_reclaim = 0;
	//	return 1;
	//}

	/* this will make black screen shorter */
	//if (rp->nr_reclaimed > 2000) {
	//	rp->nr_to_reclaim = 0;
	//	return 1;
	//}
#endif

	if (rp->nr_to_reclaim && (addr != end))
		goto cont;

	/* TODO: is there other reschedule point we can add */
	cond_resched();

	return 0;
}

/* get_task_struct before using this function */
static ssize_t reclaim_anon(struct task_struct *task, int prev_adj)
{
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	struct mm_walk reclaim_walk = {};
	struct mp_reclaim_param rp;
	int task_anon = 0, task_swap = 0;
	int a_task_anon = 0, a_task_swap = 0;

	//u64 time_ns = 0;
#if DEBUG_TIME_INFO
	struct timespec ts1, ts2;
	getnstimeofday(&ts1);
#endif
#if FEAT_RECLAIM_LIMIT
	rp.start_jiffies = jiffies;
#endif

	spin_lock(&task->signal->reclaim_state_lock);

	/*TODO: additional handle for PF_EXITING do_exit()->exit_signal()*/
	if (task->signal->swapin_should_readahead_m != RECLAIM_QUEUE) {
		//trace_printk("EXIT reclaim: this task is either (reclaimed) or (adj 0 swapin)\n");
		spin_unlock(&task->signal->reclaim_state_lock);
		goto out;
	}
	task->signal->swapin_should_readahead_m = RECLAIM_DONE;
	spin_unlock(&task->signal->reclaim_state_lock);

	/* TODO: do we need to use p = find_lock_task_mm(tsk); in case main thread got killed */
	mm = get_task_mm(task);
	if (!mm)
		goto out;

	task_anon = get_mm_counter(mm, MM_ANONPAGES);
	task_swap = get_mm_counter(mm, MM_SWAPENTS);

	reclaim_walk.mm = mm;
	reclaim_walk.pmd_entry = memplus_reclaim_pte;

	rp.nr_to_reclaim = INT_MAX;
	rp.nr_reclaimed = 0;
	rp.nr_scanned = 0;

	/* if app is larger than 200MB, override its property to frequent */
	if (task_anon + task_swap > 51200) {
		rp.type = TYPE_FREQUENT;
	} else
		rp.type = task->signal->memplus_type;

	reclaim_walk.private = &rp;

	down_read(&mm->mmap_sem);
	for (vma = mm->mmap; vma; vma = vma->vm_next) {
		if (is_vm_hugetlb_page(vma))
			continue;

		if (vma->vm_file)
			continue;

		/* if mlocked, don't reclaim it */
		if (vma->vm_flags & VM_LOCKED)
			continue;

		if (task->signal->swapin_should_readahead_m != RECLAIM_DONE)
			break;

		rp.vma = vma;
		walk_page_range(vma->vm_start, vma->vm_end,
				&reclaim_walk);

		vma->memplus_flags = 0;
		if (!rp.nr_to_reclaim)
			break;
	}

	flush_tlb_mm(mm);
	up_read(&mm->mmap_sem);
	a_task_anon = get_mm_counter(mm, MM_ANONPAGES);
	a_task_swap = get_mm_counter(mm, MM_SWAPENTS);
	mmput(mm);
out:
#if DEBUG_TIME_INFO
	getnstimeofday(&ts2);
	ts2 = timespec_sub(ts2, ts1);
	time_ns = timespec_to_ns(&ts2);
#endif
	/* it's possible that rp data isn't initialized because mm don't exist */
	//trace_printk("%s (pid %d)(size %d-%d to %d-%d) (adj %d -> %d) reclaimed %d scan %d consumed %llu ms %llu us\n"
	//		, task->comm, task->pid, task_anon, task_swap, a_task_anon, a_task_swap
	//		, prev_adj, task->signal->oom_score_adj, rp.nr_reclaimed, rp.nr_scanned
	//		, (time_ns/1000000), (time_ns/1000)%1000);

	/* TODO : return proper value */
	return 0;
}

//TODO: should we mark reclaimd/swapind freezable?
static int reclaimd_fn(void *p)
{
	struct reclaim_data data;
	struct task_struct *tsk;

	set_freezable();
	for ( ; ; ) {
		while (!pm_freezing && dequeue_reclaim_data(&data, &ri)) {
			rcu_read_lock();
			tsk = find_task_by_vpid(data.pid);

			/* KTHREAD is almost impossible to hit this */
			//if (tsk->flags & PF_KTHREAD) {
			//  rcu_read_unlock();
			//	continue;
			//}

			if (!tsk) {
				rcu_read_unlock();
				continue;
			}

			get_task_struct(tsk);
			rcu_read_unlock();

			do {
				msleep(30);
			} while (swapind_tsk && (swapind_tsk->state == TASK_RUNNING));

			reclaim_anon(tsk, data.prev_adj);
			put_task_struct(tsk);
		}

		set_current_state(TASK_INTERRUPTIBLE);
		freezable_schedule();

		if (kthread_should_stop())
			break;
	}

	return 0;
}

void memplus_stop(void)
{
	if (reclaimd_tsk) {
		kthread_stop(reclaimd_tsk);
		reclaimd_tsk = NULL;
	}
	if (swapind_tsk) {
		kthread_stop(swapind_tsk);
		swapind_tsk = NULL;
	}
}

static int __init memplus_init(void)
{
	//TODO: priority tuning for reclaimd/swapind
	//struct sched_param param = { .sched_priority = MAX_USER_RT_PRIO -1 };
	//struct sched_param param = { .sched_priority = 1 };
	struct memplus_cb_set set;

	reclaimd_tsk = kthread_run(reclaimd_fn, 0, "reclaimd");
	if (IS_ERR(reclaimd_tsk)) {
		pr_err("Failed to start reclaimd\n");
		reclaimd_tsk = NULL;
	}

	swapind_tsk = kthread_run(swapind_fn, 0, "swapind");
	if (IS_ERR(swapind_tsk)) {
		pr_err("Failed to start swapind\n");
		swapind_tsk = NULL;
	} else {
		//if (sched_setscheduler_nocheck(swapind_tsk, SCHED_FIFO, &param)) {
		//	pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		//}
	}

	set.current_is_swapind_cb = ctech_current_is_swapind;
	set.memplus_check_isolate_page_cb = ctech_memplus_check_isolate_page;
	set.memplus_enabled_cb = ctech_memplus_enabled;
	set.memplus_move_anon_to_swapcache_lru_cb = ctech_memplus_move_anon_to_swapcache_lru;
	set.memplus_move_swapcache_to_anon_lru_cb = ctech_memplus_move_swapcache_to_anon_lru;
	set.memplus_state_check_cb = ctech_memplus_state_check;
	set.__memplus_enabled_cb = ctech__memplus_enabled;

	register_cb_set(&set);
	return 0;
}

unsigned long memplus_scan(void)
{
	struct pagevec pvec;
	unsigned nr_space = 0;
	pgoff_t index = 0, indices[PAGEVEC_SIZE];
	int i, j, iso_count = 0;
	struct address_space *spaces;
	struct swap_info_struct *sis, *next;
	unsigned int node = numa_node_id();
	unsigned int total_swapcache = total_swapcache_pages();
	LIST_HEAD(page_list);

	if (!total_swapcache)
		return 0;

	spin_lock(&swap_lock);
	plist_for_each_entry_safe(sis, next, &swap_avail_heads[node], avail_lists[node]) {
		nr_space = DIV_ROUND_UP(sis->max, SWAP_ADDRESS_SPACE_PAGES);
		spaces = rcu_dereference(swapper_spaces[sis->type]);
		if (!nr_space || !spaces)
			continue;
		for (j = 0; j < nr_space; j++) {
			index = 0;
			pagevec_init(&pvec, 0);
			while (pagevec_lookup_entries(&pvec, &spaces[j], index, (pgoff_t)PAGEVEC_SIZE, indices)) {
				for (i = 0; i < pagevec_count(&pvec); i++) {
					struct page *page = pvec.pages[i];

					index = indices[i];

					if (radix_tree_exceptional_entry(page)) {
						continue;
					}

					if (!PageSwapCache(page))
						continue;

					if (PageWriteback(page))
						continue;

					if (isolate_lru_page(page))
						continue;

					if (PageAnon(page) && !PageSwapBacked(page)) {
						putback_lru_page(page);
						continue;
					}

					ClearPageWillneed(page);
					list_add(&page->lru, &page_list);
					inc_node_page_state(page, NR_ISOLATED_ANON +
							page_is_file_cache(page));
					iso_count ++;
				}
				pagevec_remove_exceptionals(&pvec);
				pagevec_release(&pvec);
				index++;
			}
		}
	}
	spin_unlock(&swap_lock);
	return coretech_reclaim_pagelist(&page_list, NULL, NULL);
}

static int memory_plus_test_worstcase_store(const char *buf, const struct kernel_param *kp)
{
	unsigned int val;
	unsigned long freed = 0;

	if (sscanf(buf, "%u\n", &val) <= 0)
		return -EINVAL;

	if(val == 1)
		freed = memplus_scan();
	printk("memory_plus_test_worstcase_store: freed = %ld\n", freed);

	return 0;
}

static struct kernel_param_ops memory_plus_test_worstcase_ops = {
	.set = memory_plus_test_worstcase_store,
};

module_param_cb(memory_plus_test_worstcase, &memory_plus_test_worstcase_ops, NULL, 0200);

module_param_named(memory_plus_enabled, vm_memory_plus, uint, S_IRUGO | S_IWUSR);
module_param_named(memplus_add_to_swap, memplus_add_to_swap, ulong, S_IRUGO | S_IWUSR);

module_init(memplus_init)
