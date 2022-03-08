/*
 * Copyright (c) 2015-2017, The OnePlus corporation. All rights reserved.
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

#include <linux/types.h>
#include <linux/module.h>
#include "../kernel/sched/sched.h"
#include "opchain_define.h"

// tedlin@ASTI 2019/06/12 add for CONFIG_HOUSTON
#include <oneplus/houston/houston_helper.h>

#define t_rq(t)		task_rq(t)
#define c_rq(cpu) 	cpu_rq(cpu)

struct opchain_cb uxcore_api = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
EXPORT_SYMBOL(uxcore_api);

unsigned int *opc_boost_tl;
EXPORT_SYMBOL(opc_boost_tl);

unsigned int *opc_boost;
EXPORT_SYMBOL(opc_boost);

void opc_set_boost(unsigned int val)
{
	if (opc_boost)
		*opc_boost = val;
}
EXPORT_SYMBOL(opc_set_boost);

bool is_opc_task(struct task_struct *t, int type)
{
	if (uxcore_api.is_opc_task_t)
		return uxcore_api.is_opc_task_t((void *)t_rq(t), (void *)t, type);
	return 0;
}
EXPORT_SYMBOL(is_opc_task);

void opc_binder_pass(size_t data_size, uint32_t *data, int send)
{
	if (uxcore_api.opc_binder_pass_t) {
		if (uxcore_api.opc_binder_pass_t((void *)t_rq(current), (void *)current, data_size, data, send))
// tedlin@ASTI 2019/06/12 add for CONFIG_HOUSTON
			ht_perf_notify();
	}
}
EXPORT_SYMBOL(opc_binder_pass);

void opc_task_switch(unsigned int enqueue, int cpu, struct task_struct *p, u64 clock) {
    if (uxcore_api.opc_task_switch_t)
	uxcore_api.opc_task_switch_t(enqueue, cpu, (void *)p, (void *)t_rq(p), clock);
}
EXPORT_SYMBOL(opc_task_switch);

int opc_get_claim_on_cpu(int cpu)
{
	if (uxcore_api.opc_get_claim_on_cpu_t)
		return uxcore_api.opc_get_claim_on_cpu_t(cpu, (void *)c_rq(cpu));
	return 0;
}
EXPORT_SYMBOL(opc_get_claim_on_cpu);

unsigned int opc_get_claims(void)
{
	void *rqs[NUMS_CPU];
	int idx;

	if (uxcore_api.opc_get_claims_t) {
		for (idx = 0; idx < NUMS_CPU; idx++) {
			rqs[idx] = (void *)c_rq(idx);
		}
		return uxcore_api.opc_get_claims_t(rqs);
	}
	return 0;
}
EXPORT_SYMBOL(opc_get_claims);

int opc_select_path(struct task_struct *cur, struct task_struct *t, int prev_cpu)
{
	void *rqs[NUMS_CPU];
	int idx;

	if (uxcore_api.opc_select_path_t) {
		for (idx = 0; idx < NUMS_CPU; idx++) {
			rqs[idx] = (void *)c_rq(idx);
		}
		return uxcore_api.opc_select_path_t(rqs, (void *)t_rq(cur), (void *)t_rq(t), (void *)cur, (void *)t, prev_cpu);
	}
	return OP_PATH_NORMAL;
}
EXPORT_SYMBOL(opc_select_path);

unsigned long opc_cpu_util(unsigned long util, int cpu, struct task_struct *t, int op_path)
{
	if (uxcore_api.opc_cpu_util_t)
		return uxcore_api.opc_cpu_util_t(util, cpu, (void *)t, (void *)c_rq(cpu), op_path);
	return util;
}
EXPORT_SYMBOL(opc_cpu_util);

void opc_add_to_chain(struct task_struct *t)
{
	if (uxcore_api.opc_add_to_chain_t)
		uxcore_api.opc_add_to_chain_t((void *)t_rq(t), (void *)t);
}
EXPORT_SYMBOL(opc_add_to_chain);

bool opc_check_uxtop_cpu(int uxtop, int cpu)
{
	if (uxcore_api.opc_check_uxtop_cpu_t)
		return uxcore_api.opc_check_uxtop_cpu_t(uxtop, cpu);
	return true;
}
EXPORT_SYMBOL(opc_check_uxtop_cpu);

unsigned long __init opc_get_orig_capacity(int cpu)
{
	return cpu_rq(cpu)->cpu_capacity_orig;
}
EXPORT_SYMBOL(opc_get_orig_capacity);

bool opc_utask_slave(struct task_struct *t)
{
	return t->utask_slave;
}
EXPORT_SYMBOL(opc_utask_slave);

void __exit opc_exit_module(void)
{
	uxcore_api.opc_binder_pass_t = NULL;
	uxcore_api.is_opc_task_t = NULL;
	uxcore_api.opc_task_switch_t = NULL;
	uxcore_api.opc_get_claim_on_cpu_t = NULL;
	uxcore_api.opc_get_claims_t = NULL;
	uxcore_api.opc_select_path_t = NULL;
	uxcore_api.opc_cpu_util_t = NULL;
	uxcore_api.opc_add_to_chain_t = NULL;
	uxcore_api.opc_check_uxtop_cpu_t = NULL;
	opc_boost_tl = NULL;
	opc_boost = NULL;
}
EXPORT_SYMBOL(opc_exit_module);
