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

#include <linux/stddef.h>
#include <linux/module.h>

#include "opchain_struct_offset_helper.h"
#include "../kernel/sched/sched.h"

/* struct task_struct */
unsigned int opchain_task_struct_offset[__TASK_OFFSET_MAX] = {
	[TASK_OFFSET_WAKEE_FLIPS] = offsetof(struct task_struct, wakee_flips),
	[TASK_OFFSET_CPUS_ALLOWED] = offsetof(struct task_struct, cpus_allowed),
	[TASK_OFFSET_PID] = offsetof(struct task_struct, pid),
	[TASK_OFFSET_TGID] = offsetof(struct task_struct, tgid),
	[TASK_OFFSET_GROUP_LEADER] = offsetof(struct task_struct, group_leader),
	[TASK_OFFSET_COMM] = offsetof(struct task_struct, comm),
	[TASK_OFFSET_UTASK_TAG] = offsetof(struct task_struct, utask_tag),
	[TASK_OFFSET_UTASK_TAG_BASE] = offsetof(struct task_struct, utask_tag_base),
	[TASK_OFFSET_ETASK_CLAIM] = offsetof(struct task_struct, etask_claim),
	[TASK_OFFSET_CLAIM_CPU] = offsetof(struct task_struct, claim_cpu),
	[TASK_OFFSET_UTASK_SLAVE] = offsetof(struct task_struct, utask_slave),
	[TASK_OFFSET_EXIT_STATE] = offsetof(struct task_struct, exit_state)
};
gen_type_offset_impl(task_struct);

/* struct rq */
unsigned int opchain_rq_offset[__RQ_OFFSET_MAX] = {
#ifdef CONFIG_SMP
	[RQ_OFFSET_CPU_CAPACITY_ORIG] = offsetof(struct rq, cpu_capacity_orig),
	[RQ_OFFSET_CPU] = offsetof(struct rq, cpu),
#endif
#ifdef CONFIG_SCHED_HMP
	[RQ_OFFSET_WINDOW_START] = offsetof(struct rq, window_start),
#endif
	[RQ_OFFSET_CLOCK] = offsetof(struct rq, clock)
};
gen_type_offset_impl(rq);
