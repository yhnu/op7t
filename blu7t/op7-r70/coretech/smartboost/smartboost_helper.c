#include <linux/fs.h>
#include <oneplus/smartboost/smartboost_helper.h>

struct smb_cb_set smb_cbs;

bool smb_uid_lru_add(struct page *page)
{
	if (smb_cbs.smb_uid_lru_add_cb)
		return smb_cbs.smb_uid_lru_add_cb(page);
	else
		return false;
}

unsigned long smb_isolate_list_or_putbcak(struct list_head *page_list,
	struct lruvec *lruvec, struct pglist_data *pgdat, int priority,
	bool enough_list_reclaimed)
{
	if (smb_cbs.smb_isolate_list_or_putbcak_cb)
		return smb_cbs.smb_isolate_list_or_putbcak_cb(page_list,
			lruvec, pgdat, priority, enough_list_reclaimed);
	else
		return 0;
}

bool smb_update_uid_lru_size(struct page *page,
				struct lruvec *lruvec, enum lru_list lru)
{
	if (smb_cbs.smb_update_uid_lru_size_cb)
		return smb_cbs.smb_update_uid_lru_size_cb(page, lruvec, lru);
	else
		return false;
}

void smb_register_cb_set(struct smb_cb_set *set)
{
	smb_cbs = *set;
}

