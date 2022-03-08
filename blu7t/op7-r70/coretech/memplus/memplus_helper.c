#include <oneplus/memplus/memplus_helper.h>
#include <linux/page-flags.h>

static struct memplus_cb_set cb_set;
#define PF_NO_TAIL(page, enforce) ({                                  \
		VM_BUG_ON_PGFLAGS(enforce && PageTail(page), page);     \
		compound_head(page); })

bool memplus_enabled(void)
{
	if (cb_set.memplus_enabled_cb)
		return cb_set.memplus_enabled_cb();
	return false;
}
bool __memplus_enabled(void)
{
	if (cb_set.__memplus_enabled_cb)
		return cb_set.__memplus_enabled_cb();
	return false;
}
bool current_is_swapind(void)
{
	if (cb_set.current_is_swapind_cb)
		return cb_set.current_is_swapind_cb();
	return false;
}

void memplus_move_swapcache_to_anon_lru(struct page *page)
{
	if (cb_set.memplus_move_swapcache_to_anon_lru_cb)
		cb_set.memplus_move_swapcache_to_anon_lru_cb(page);
	else
		clear_bit(PG_swapcache, &(PF_NO_TAIL(page, 1))->flags);
}
void memplus_move_anon_to_swapcache_lru(struct page *page)
{
	if (cb_set.memplus_move_anon_to_swapcache_lru_cb)
		cb_set.memplus_move_anon_to_swapcache_lru_cb(page);
	else
		set_bit(PG_swapcache, &(PF_NO_TAIL(page, 1))->flags);
}
void memplus_state_check(bool legacy, int oom_adj,
	struct task_struct *task, int type, int update)
{
	if (cb_set.memplus_state_check_cb)
		cb_set.memplus_state_check_cb(legacy,
			oom_adj, task, type, update);
}
bool memplus_check_isolate_page(struct page *page)
{
	if (cb_set.memplus_check_isolate_page_cb)
		return cb_set.memplus_check_isolate_page_cb(page);
	return false;
}

void register_cb_set(struct memplus_cb_set *set)
{
	cb_set = *set;
}

#undef PF_NO_TAIL
