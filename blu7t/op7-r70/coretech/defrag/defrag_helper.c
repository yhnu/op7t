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

#include <linux/types.h>
#include <asm/pgtable.h>
#include <linux/mmzone.h>
#include <linux/page-isolation.h>
#include <linux/atomic.h>
#include <linux/moduleparam.h>
#include <linux/module.h>
#include <oneplus/defrag/defrag_helper.h>

struct defrag_cb_set defrag_cbs;
atomic64_t fp_order_usage[MAX_ORDER] = {ATOMIC64_INIT(0)};
atomic64_t fp_order_fail[MAX_ORDER] = {ATOMIC64_INIT(0)};

/* calling functions */
struct page *defrag_alloc(struct zone *zone, unsigned long flags,
			int migratetype, int order)
{
	if (defrag_cbs.defrag_alloc_cb)
		return defrag_cbs.defrag_alloc_cb(zone,
			 flags, migratetype, order);
	return NULL;
}

long defrag_calc(struct zone *zone, int order, int alloc_flag)
{
	if (likely(defrag_cbs.defrag_calc_cb))
		return defrag_cbs.defrag_calc_cb(zone, order, alloc_flag);
	else
		return defrag_zone_free_size(zone);
}

bool defrag_check_alloc_flag(unsigned int alloc_flags, int order)
{
	if (defrag_cbs.defrag_check_alloc_flag_cb)
		return defrag_cbs.defrag_check_alloc_flag_cb(alloc_flags,
						order);
	return false;
}

void defrag_register_cb_set(struct defrag_cb_set *cbs)
{
	defrag_cbs = *cbs;
}

void defrag_unregister_cb_set(void)
{
	defrag_cbs.defrag_alloc_cb = NULL;
	defrag_cbs.defrag_check_alloc_flag_cb = NULL;
	defrag_cbs.defrag_calc_cb = NULL;
}
