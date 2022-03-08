/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _LINUX_DEFRAG_HELPER_H
#define _LINUX_DEFRAG_HELPER_H
#include <linux/seq_file.h>
#include <linux/mm_types.h>
#include <linux/vmstat.h>
#include <linux/mmzone.h>


#ifdef CONFIG_DEFRAG
#define ALLOC_UNMOVE    0x800

#define DEFRAG_FREE_SIZE global_zone_page_state(NR_FREE_DEFRAG_POOL)
#define defrag_zone_free_size(z) zone_page_state(z, NR_FREE_DEFRAG_POOL)
#define show_defrag_free(m) show_val_kb(m, "DefragPoolFree: ", DEFRAG_FREE_SIZE)
#define show_real_freemem(m, free) \
	show_val_kb(m, "RealMemFree:    ", free - DEFRAG_FREE_SIZE)
#define IS_NOT_DEFRAG_POOL_EMPTY(area) \
	(!list_empty(&area->free_list[MIGRATE_UNMOVABLE_DEFRAG_POOL]))

extern atomic64_t fp_order_usage[MAX_ORDER];
extern atomic64_t fp_order_fail[MAX_ORDER];

#define ADD_ORDER_USAGE(o) atomic64_inc(&fp_order_usage[o])
#define ADD_ORDER_FAIL(o) atomic64_inc(&fp_order_fail[o])

struct defrag_cb_set {
	struct page *(*defrag_alloc_cb)(struct zone *zone,
		unsigned long flags, int migratetype, int order);
	long (*defrag_calc_cb)(struct zone *zone, int order,
			int alloc_flag);
	bool (*defrag_check_alloc_flag_cb)(unsigned int alloc_flags, int order);
};

#define defrag_migrate_to_alloc_flag(allocflag, migratetype) \
	do {	\
		if (migratetype == MIGRATE_UNMOVABLE)	\
			allocflag |= ALLOC_UNMOVE;	\
	} while (0)

extern struct page *defrag___rmqueue(struct zone *zone, unsigned int order,
			int migratetype);
extern void defrag_unregister_cb_set(void);
extern void defrag_register_cb_set(struct defrag_cb_set *cbs);

struct page *defrag_alloc(struct zone *zone, unsigned long flags,
			int migratetype, int order);
long defrag_calc(struct zone *zone, int order, int alloc_flag);
bool defrag_check_alloc_flag(unsigned int alloc_flags, int order);

#else  /* !CONFIG_DEFRAG */

#define DEFRAG_FREE_SIZE 0
#define defrag_zone_free_size(z) 0
#define show_defrag_free(m)
#define show_real_freemem(m, free)
#define ADD_ORDER_USAGE(o)
#define ADD_ORDER_FAIL(o)
#define defrag_migrate_to_alloc_flag(allocflag, migratetype)
#define IS_NOT_DEFRAG_POOL_EMPTY(area) false


static __always_inline struct page *defrag_alloc(struct zone *zone,
			unsigned long flags,
			int migratetype, int order)
{
	return NULL;
}

static __always_inline long defrag_calc(struct zone *zone,
			int order, int alloc_flag)
{
	return 0;
}
static inline bool defrag_check_alloc_flag(unsigned int alloc_flags, int order)
{
	return false;
}

#endif
#endif

