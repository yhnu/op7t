/*
 * Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "defrag.h"


static struct zone *ctech_first_zone(void)
{
	return NODE_DATA(first_online_node)->node_zones;
}

static struct zone *ctech_next_zone(struct zone *zone)
{
	return next_zone(zone);
}

static unsigned long ctech_zone_end_pfn(const struct zone *zone)
{
	return zone_end_pfn(zone);
}

static int ctech_pfn_valid(unsigned long pfn)
{
	return pfn_valid(pfn);
}

static struct page *ctech_pfn_to_page(unsigned long pfn)
{
	return pfn_to_page(pfn);
}

static bool ctech_defrag_check_alloc_flag(unsigned int alloc_flag, int order)
{
	return !check_alloc_flag(alloc_flag, order);
}

static struct page *ctech_defrag_pool_alloc(struct zone *zone, unsigned long flags,
			int migratetype, int order)
{
	struct page *page = NULL;

	if (check_alloc_type(migratetype, order)) {
		spin_lock_irqsave(&zone->lock, flags);
		page = defrag___rmqueue(zone, order,
					MIGRATE_UNMOVABLE_DEFRAG_POOL);

		if (page) {
			spin_unlock(&zone->lock);
			__mod_zone_page_state(zone, NR_FREE_PAGES,
							-(1 << order));
			__mod_zone_page_state(zone, NR_FREE_DEFRAG_POOL,
							-(1 << order));
			local_irq_restore(flags);
		} else
			spin_unlock_irqrestore(&zone->lock, flags);
	}

	return page;
}

/* return pool size, if this allocation cannot use our pool */
static long ctech_calculate_reserved_pool(struct zone *z, int order, int alloc_flag)
{
	if (check_alloc_flag(alloc_flag, order))
		return zone_page_state(z, NR_FREE_DEFRAG_POOL);
	else
		return 0;
}

static void release_unused_area(int request)
{
	struct zone *zone;
	unsigned long start_pfn, pfn, end_pfn;
	unsigned long block_mt;
	unsigned long flags;
	struct page *page;
	int counter, pages_moved;

	if (request)
		request = request_reserved_block();

	/* for_each_zone(zone) { */
	for (zone = ctech_first_zone(); zone; zone = ctech_next_zone(zone)) {
		if (strstr(zone->name, "Movable") != NULL)
			continue;
		spin_lock_irqsave(&zone->lock, flags);
		start_pfn = zone->zone_start_pfn;
		end_pfn = ctech_zone_end_pfn(zone);
		start_pfn = roundup(start_pfn, pageblock_nr_pages);
		counter = 0;

		for (pfn = start_pfn; pfn < end_pfn;
					pfn += pageblock_nr_pages) {
			if (!ctech_pfn_valid(pfn))
				continue;
			page = ctech_pfn_to_page(pfn);
			block_mt = get_pageblock_migratetype(page);
			if (block_mt == MIGRATE_UNMOVABLE_DEFRAG_POOL) {
				if (++counter <= request)
					continue;
				else {
					set_pageblock_migratetype(page,
							MIGRATE_MOVABLE);
					pages_moved = move_freepages_block(zone,
							page, MIGRATE_MOVABLE, NULL);
					__mod_zone_page_state(zone,
						NR_FREE_DEFRAG_POOL,
							-pages_moved);
				}
			}
		}
		spin_unlock_irqrestore(&zone->lock, flags);
	}
}

static int __init defrag_pool_setup(void)
{
	struct zone *zone;
	unsigned long start_pfn, pfn, end_pfn;
	unsigned long block_mt;
	unsigned long flags;
	struct page *page;
	int pages_moved;
	int nr_pgblock = 0;
	int nr_movebcak = 0;

	/* for_each_zone(zone) { */
	for (zone = ctech_first_zone(); zone; zone = ctech_next_zone(zone)) {
		nr_pgblock = 0;
		if (strstr(zone->name, "Movable") != NULL)
			continue;
		spin_lock_irqsave(&zone->lock, flags);
		start_pfn = zone->zone_start_pfn;
		end_pfn = ctech_zone_end_pfn(zone);
		start_pfn = roundup(start_pfn, pageblock_nr_pages);

		for (pfn = start_pfn; pfn < end_pfn;
					pfn += pageblock_nr_pages) {
			if (!ctech_pfn_valid(pfn))
				continue;
			page = ctech_pfn_to_page(pfn);
			block_mt = get_pageblock_migratetype(page);
			if (block_mt == MIGRATE_MOVABLE) {
				pages_moved = move_freepages_block(zone,
							page, MIGRATE_UNMOVABLE_DEFRAG_POOL, NULL);

				if (pages_moved == pageblock_nr_pages) {
					nr_pgblock++;
					set_pageblock_migratetype(page, MIGRATE_UNMOVABLE_DEFRAG_POOL);
					__mod_zone_page_state(zone, NR_FREE_DEFRAG_POOL, pages_moved);
				} else {
					move_freepages_block(zone, page, block_mt, NULL);
					nr_movebcak++;
				}
				if (nr_pgblock >= request_reserved_block())
					break;
			}
		}
		spin_unlock_irqrestore(&zone->lock, flags);
		printk("anti-defragment: zone(%s) setup page blocks(%d), moveback blcoks(%d)\n", zone->name, nr_pgblock, nr_movebcak);
	}
	return 0;
}

static void release_migratetype(void)
{
	struct zone *zone;
	unsigned long start_pfn, pfn, end_pfn;
	unsigned long block_migratetype;
	unsigned long flags;
	struct page *page;
	int counter, pages_moved;

	for_each_zone(zone) {
		spin_lock_irqsave(&zone->lock, flags);
		start_pfn = zone->zone_start_pfn;
		end_pfn = zone_end_pfn(zone);
		start_pfn = roundup(start_pfn, pageblock_nr_pages);
		counter = 0;

		for (pfn = start_pfn; pfn < end_pfn;
					pfn += pageblock_nr_pages) {
			if (!pfn_valid(pfn))
				continue;
			page = pfn_to_page(pfn);
			block_migratetype = get_pageblock_migratetype(page);
			if (block_migratetype ==
					MIGRATE_UNMOVABLE_DEFRAG_POOL) {
				set_pageblock_migratetype(page,
						MIGRATE_MOVABLE);
				pages_moved = move_freepages_block(zone, page,
							MIGRATE_MOVABLE, NULL);
				__mod_zone_page_state(zone,
				NR_FREE_DEFRAG_POOL, -pages_moved);
			}
		}
		spin_unlock_irqrestore(&zone->lock, flags);
	}
}

unsigned int __read_mostly disable;

static int disable_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u", disable);
}

static int disable_store(const char *buf, const struct kernel_param *kp)
{
	unsigned int val;

	if (sscanf(buf, "%u\n", &val) <= 0)
		return -EINVAL;
	if (!strncmp(buf, "1", 1)) {
		disable = val;
		release_migratetype();
		defrag_unregister_cb_set();
	}
	return 0;
}

static const struct kernel_param_ops param_ops_disable = {
	.get = disable_show,
	.set = disable_store,
};
module_param_cb(disable, &param_ops_disable, NULL, 0644);

static inline int print_fp_statistics(char *buf)
{
	int order;
	int size = 0;

	size += sprintf(buf + size, "fp_usage\t\t");
	for (order = 0; order < MAX_ORDER; ++order)
		size += sprintf(buf + size,  "%6lu ", atomic64_read(&fp_order_usage[order]));
	size += sprintf(buf + size, "\n");
	size += sprintf(buf + size, "fp_fail \t\t");
	for (order = 0; order < MAX_ORDER; ++order)
		size += sprintf(buf + size, "%6lu ", atomic64_read(&fp_order_fail[order]));
	size += sprintf(buf + size, "\n");

	return size;
}

static int fp_fail_show(char *buf, const struct kernel_param *kp)
{
	int size = 0;
	size = print_fp_statistics(buf);
	return size;
}

static const struct kernel_param_ops param_ops_fp_fail = {
	.get = fp_fail_show,
};
module_param_cb(fp_fail, &param_ops_fp_fail, NULL, 0444);

static int __init defrag_pool_init(void)
{
	struct defrag_cb_set set;
	set.defrag_alloc_cb = ctech_defrag_pool_alloc;
	set.defrag_calc_cb = ctech_calculate_reserved_pool;
	set.defrag_check_alloc_flag_cb = ctech_defrag_check_alloc_flag;
	defrag_register_cb_set(&set);

	return 0;
}

static void __exit defrag_pool_exit(void)
{
	defrag_unregister_cb_set();
	release_unused_area(0);
}
early_initcall(defrag_pool_setup)

module_init(defrag_pool_init);
module_exit(defrag_pool_exit);

MODULE_DESCRIPTION("OnePlus Defragger");
MODULE_LICENSE("GPL");
