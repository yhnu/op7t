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

#include <linux/module.h>
#include <linux/types.h>
#include <asm/page.h>

#include "../kernel/sched/sched.h"
#include "../opchain_define.h"
#include "opchain_core.h"
#include "../opchain_helper.h"

unsigned int __read_mostly boost;
unsigned int __read_mostly boost_tl;
unsigned int __read_mostly boost_sample_time = 1;
unsigned int __read_mostly chain_on = 1;
unsigned int __read_mostly latest_ms = 100;
unsigned int __read_mostly latest_threshold = 100000000;

#if UX_DEBUG
static int opchain_status_show(char *buf, const struct kernel_param *kp)
{
	return opchain_status_show_core(buf, kp);
}

static const struct kernel_param_ops param_ops_opchain_status = {
	.get = opchain_status_show,
};
module_param_cb(opchain_status, &param_ops_opchain_status, NULL, 0644);
#endif

static int latest_ms_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE,	"%u", latest_ms);
}

static int latest_ms_store(const char *buf, const struct kernel_param *kp)
{
	unsigned int val;

	if (sscanf(buf, "%u\n", &val) <= 0)
		return -EINVAL;
	latest_ms = val;
	latest_threshold = val * 1000000;
	return 0;
}

static const struct kernel_param_ops param_ops_latest_ms = {
	.get = latest_ms_show,
	.set = latest_ms_store,
};

module_param(boost, uint, 0644);
module_param(boost_sample_time, uint, 0644);
module_param(boost_tl, uint, 0644);
module_param(chain_on, uint, 0644);
module_param_cb(latest_ms, &param_ops_latest_ms, NULL, 0644);

static int __init opchain_init(void)
{
	ctech_opchain_init(&uxcore_api, opc_get_orig_capacity(MIN_POWER_CPU));

	return 0;
}

module_init(opchain_init);

static void __exit opchain_exit_module(void)
{
	opc_exit_module();
}

module_exit(opchain_exit_module);
