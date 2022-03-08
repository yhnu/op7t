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

#ifndef _LINUX_OPCHAIN_DEFINE_H
#define _LINUX_OPCHAIN_DEFINE_H

#define UX_DEBUG			0
#define UTASK				0
#define UT_CLK_BASE			0x01
#define UT_ETASK			0x02
#define UT_LATEST_ONE			0x04
#define UT_PERF_TOP			0x08
#define UT_FORE				(UT_CLK_BASE | UT_ETASK)

#define OP_CLAIM_S			-1
#define OP_PATH_SLAVE			-4
#define OP_PATH_CLAIM			-3
#define OP_PATH_NORMAL			-2
#define OP_PATH_OCCUPIED		-1
#define MIN_POWER_CPU			0

#define ONESEC_NANO 1000000000

#if 1
/* for MSM8998, SDM845*/
#define FIRST_BIG_CORE			4
#define NUMS_CPU			8
#define CPU_VIRTUAL_PLUG_IN(i) (opc_cpu_active(i) && !opc_cpu_isolated(i))
#else
/* for MSM8996*/
#define FIRST_BIG_CORE			2
#define NUMS_CPU			4
#define CPU_VIRTUAL_PLUG_IN(i) (opc_cpu_active(i))
#endif

struct opchain_cb {
	unsigned int (*is_opc_task_t)(void *rq, void *t, int type);
	int (*opc_binder_pass_t)(void *rq, void* cur, unsigned int dsize, unsigned int *data, int send);
	void (*opc_task_switch_t)(unsigned int enqueue, int cpu, void *p, void *rq, unsigned long long clock);
	int (*opc_get_claim_on_cpu_t)(int cpu, void *rq);
	unsigned int (*opc_get_claims_t)(void **rqs);
	int (*opc_select_path_t)(void **rqs, void *w_rq, void *t_rq, void *cur, void *t, int prev_cpu);
	unsigned long (*opc_cpu_util_t)(unsigned long util, int cpu, void *t, void *rq, int op_path);
	void (*opc_add_to_chain_t)(void *rq, void *t);
	int (*opc_check_uxtop_cpu_t)(int uxtop, int cpu);
};
#endif
