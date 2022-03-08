/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#include <linux/stddef.h>
#include <linux/cpufreq.h>
#include <linux/module.h>

#include "op_struct_offset_helper.h"

/* struct zone */
unsigned int _zone_offset[__ZONE_OFFSET_MAX] = {
	[ZONE_OFFSET_LOCK] = offsetof(struct zone, lock),
	[ZONE_OFFSET_ZSP] = offsetof(struct zone, zone_start_pfn)
};
gen_type_offset_impl(zone);

/* struct pglist_data */
unsigned int _pglist_data_offset[__PGLIST_DATA_OFFSET_MAX] = {
	[PGLIST_DATA_OFFSET_NODE_ZONES] = offsetof(struct pglist_data,
								node_zones)
};
gen_type_offset_impl(pglist_data);
