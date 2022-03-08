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

#ifndef _OP_STRUCT_OFFSET_HELPER_INC_
#define _OP_STRUCT_OFFSET_HELPER_INC_

/* define macro to extern function declaration */
#define gen_type_offset(type) \
	extern unsigned int get_##type##_offset(int m)

/* define macro to create offset impl and export get offset/value symbol */
#define gen_type_offset_impl(type) \
	unsigned int get_##type##_offset(int m) \
		{ return _##type##_offset[m]; } \
	EXPORT_SYMBOL(get_##type##_offset)

/* enum of struct zone */
enum {
	ZONE_OFFSET_LOCK,
	ZONE_OFFSET_ZSP,

	__ZONE_OFFSET_MAX
};
#define ZONE_LOCK_R(zone) ((spinlock_t *)((char *)zone \
			+ get_zone_offset(ZONE_OFFSET_LOCK)))
#define ZONE_ZSP_R(zone) (*(unsigned long *)((char *)zone \
			+ get_zone_offset(ZONE_OFFSET_ZSP)))
gen_type_offset(zone);

/* enum of struct pglist_data */
enum {
	PGLIST_DATA_OFFSET_NODE_ZONES,

	__PGLIST_DATA_OFFSET_MAX
};
#define PGLIST_DATA_NODE_ZONES_R(pglist_data) ((struct zone *) \
			((char *)pglist_data + \
			get_pglist_data_offset(PGLIST_DATA_OFFSET_NODE_ZONES)))
gen_type_offset(pglist_data);
#endif //_OP_STRUCT_OFFSET_HELPER_INC_
