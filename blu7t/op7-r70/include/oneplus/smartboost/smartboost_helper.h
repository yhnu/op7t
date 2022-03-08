#ifndef __SMART_BOOST_HELPER_H__
#define __SMART_BOOST_HELPER_H__

#include <linux/mmzone.h>

#ifdef CONFIG_SMART_BOOST
struct smb_cb_set {
	bool (*smb_uid_lru_add_cb)(struct page *page);

	unsigned long (*smb_isolate_list_or_putbcak_cb)
				(struct list_head *page_list,
				struct lruvec *lruvec,
				struct pglist_data *pgdat, int priority,
				bool enough_list_reclaimed);

	bool (*smb_update_uid_lru_size_cb)(struct page *page,
				struct lruvec *lruvec, enum lru_list lru);
};

extern unsigned long get_max_minfree(void);
extern unsigned long coretech_reclaim_pagelist(struct list_head *page_list,
		struct vm_area_struct *vma, void *sc);
void smb_register_cb_set(struct smb_cb_set *set);
extern const struct file_operations proc_page_hot_count_operations;
extern unsigned long smb_invalidate_mapping_pages(struct address_space *mapping,
		pgoff_t start, pgoff_t end);
extern bool smb_uid_lru_add(struct page *page);
extern unsigned long smb_isolate_list_or_putbcak(struct list_head *page_list,
	struct lruvec *lruvec, struct pglist_data *pgdat, int priority,
	bool enough_list_reclaimed);

extern bool smb_update_uid_lru_size(struct page *page,
				struct lruvec *lruvec, enum lru_list lru);

#define UID_LRU_SIZE global_zone_page_state(NR_ZONE_UID_LRU)
#define PG_UIDLRU(page) PageUIDLRU(page)
#define ZONE_UID_LRU_SIZE(z) zone_page_state((z), NR_ZONE_UID_LRU)

#define SMB_HOT_COUNT_INIT(condition, p) \
	do {	\
		if (!condition)	\
			p->hot_count = 0;	\
	} while (0)


#else  /* !CONFIG_SMART_BOOST */
#define SMB_HOT_COUNT_INIT
#define UID_LRU_SIZE 0
#define ZONE_UID_LRU_SIZE(z) 0
#define PG_UIDLRU(page) 0

static __always_inline
unsigned long smb_invalidate_mapping_pages(struct address_space *mapping,
		pgoff_t start, pgoff_t end)
{
	return invalidate_mapping_pages(mapping, start, end);
}
static __always_inline
bool smb_update_uid_lru_size(struct page *page,
				struct lruvec *lruvec, enum lru_list lru)
{
	return false;
}
static __always_inline bool smb_uid_lru_add(struct page *page)
{
	return false;
}

static __always_inline unsigned long
smb_isolate_list_or_putbcak(struct list_head *page_list,
	struct lruvec *lruvec, struct pglist_data *pgdat, int priority,
	bool enough_list_reclaimed)
{
	return 0;
}

#endif
#endif
