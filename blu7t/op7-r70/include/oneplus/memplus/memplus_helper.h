#ifndef _MEMORY_PLUS_HELPER_H
#define _MEMORY_PLUS_HELPER_H

#ifdef CONFIG_MEMPLUS
#include <linux/mm_types.h>

struct memplus_cb_set {
	bool (*memplus_enabled_cb)(void);
	bool (*__memplus_enabled_cb)(void);
	bool (*current_is_swapind_cb)(void);
	void (*memplus_move_swapcache_to_anon_lru_cb)(struct page *);
	void (*memplus_move_anon_to_swapcache_lru_cb)(struct page *);
	void (*memplus_state_check_cb)(bool, int,
		struct task_struct *, int, int);
	bool (*memplus_check_isolate_page_cb)(struct page *);
};
void register_cb_set(struct memplus_cb_set *set);
extern bool memplus_enabled(void);
extern bool __memplus_enabled(void);
extern bool current_is_swapind(void);
extern void memplus_move_swapcache_to_anon_lru(struct page *page);
extern void memplus_move_anon_to_swapcache_lru(struct page *page);
extern void memplus_state_check(bool legacy, int oom_adj,
	struct task_struct *task, int type, int update);
extern  bool memplus_check_isolate_page(struct page *page);

#define memplus_init_task_reclaim_stat(sig) \
do { \
	sig->swapin_should_readahead_m = 0; \
	sig->reclaim_state_lock = __SPIN_LOCK_UNLOCKED(reclaim_state_lock); \
	sig->memplus_type = 0; \
} while (0)
#define __memplus_clear_entry(entry) ((entry).val = 0)
#define __memplus_entry(bdev_flag) ((bdev_flag) & SWP_SYNCHRONOUS_IO)
#define __get_memplus_swp_flag(entry) ((entry).val & SWP_SYNCHRONOUS_IO)
#define __set_memplus_entry(entry, flag) (entry.val = flag)
#define memplus_set_private(page, type)	\
	set_page_private((page), __memplus_entry((type)?SWP_SYNCHRONOUS_IO:0))

#define memplus_set_willneed(page) \
do {	\
	if (current_is_swapind())	\
		SetPageWillneed(page);	\
} while (0)

#define MEMPLUS_PAGE_LRU \
	(__memplus_enabled()?LRU_INACTIVE_ANON_SWPCACHE:LRU_INACTIVE_ANON)

#define INIT_RECLAIM_STATE      .reclaim_timeout = 0,	\
		.swapin_should_readahead_m = 0, \
		.reclaim_state_lock =	\
		__SPIN_LOCK_UNLOCKED(reclaim_state_lock),	\
		.memplus_type = 0,

#define memplus_page_to_lru(lru, page) (lru = page_lru(page))
#else
static __always_inline
void memplus_move_swapcache_to_anon_lru(struct page *page)
{
	clear_bit(PG_swapcache, &(PF_NO_TAIL(page, 1))->flags);
}
static __always_inline
void memplus_move_anon_to_swapcache_lru(struct page *page)
{
	set_bit(PG_swapcache, &(PF_NO_TAIL(page, 1))->flags);
}
#define memplus_init_task_reclaim_stat(sig)
#define __memplus_clear_entry(entry)
#define __memplus_entry(bdev_flag)
#define __get_memplus_swp_flag(entry)
#define __set_memplus_entry(entry, flag)
#define memplus_set_private(page, type)
#define memplus_state_check(legacy, oom_adj, task, type, update)
static __always_inline bool memplus_enabled(void)
{
	return false;
}
static __always_inline bool __memplus_enabled(void)
{
	return false;
}
static __always_inline bool memplus_check_isolate_page(struct page *page)
{
	return false;
}

#define MEMPLUS_PAGE_LRU LRU_INACTIVE_ANON

#define INIT_RECLAIM_STATE

#define memplus_page_to_lru(lru, page)

#endif
#endif /* _MEMORY_PLUS_H */
