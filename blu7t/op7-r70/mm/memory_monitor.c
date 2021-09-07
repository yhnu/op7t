/*
 *Copyright (c)  2018 OnePlus Mobile Comm Corp., Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include "internal.h"
#include <linux/oem/memory_monitor.h>
#include <linux/oem/oneplus_healthinfo.h>

struct alloc_wait_para allocwait_para = {0, 0, 0, 0, 0, 0, 0, 0};

#ifdef CONFIG_ONEPLUS_HEALTHINFO
extern bool ohm_memmon_ctrl;
extern bool ohm_memmon_logon;
extern bool ohm_memmon_trig;
extern void ohm_action_trig(int type);
#else
static bool ohm_memmon_ctrl;
static bool ohm_memmon_logon;
static bool ohm_memmon_trig;
void ohm_action_trig(int type)
{
	return;
}
#endif

static int alloc_wait_h_ms = 50;
static int alloc_wait_l_ms = 10;
static int alloc_wait_log_ms = 1000;
static int alloc_wait_trig_ms = 10000;

void memory_alloc_monitor(gfp_t gfp_mask, unsigned int order, u64 wait_ms)
{
	int fg = 0;
	if (!ohm_memmon_ctrl)
		return;

	fg = current_is_fg();
	if (fg) {
		if (wait_ms >= alloc_wait_h_ms) {
			allocwait_para.fg_alloc_wait_h_cnt++;
		} else if (wait_ms >= alloc_wait_l_ms) {
			allocwait_para.fg_alloc_wait_l_cnt++;
		}
		if (allocwait_para.fg_alloc_wait_max_ms < wait_ms) {
			allocwait_para.fg_alloc_wait_max_ms = wait_ms;
			allocwait_para.fg_alloc_wait_max_order = order;
		}
	}

	if (wait_ms >= alloc_wait_h_ms) {
		allocwait_para.total_alloc_wait_h_cnt++;
		if (ohm_memmon_logon && (wait_ms >= alloc_wait_log_ms)) {
			ohm_debug("[alloc_wait / %s] long, order %d, wait %lld ms!\n",
					(fg ? "fg":"bg"), order, wait_ms);
			warn_alloc(gfp_mask, NULL, "page allocation stalls for %lld ms, order: %d",
					wait_ms, order);
		}
	if (ohm_memmon_trig && wait_ms >= alloc_wait_trig_ms) {
		/* Trig Uevent */
		ohm_action_trig(OHM_MEM_MON);
	}
	} else if (wait_ms >= alloc_wait_l_ms) {
		allocwait_para.total_alloc_wait_l_cnt++;
	}
	if (allocwait_para.total_alloc_wait_max_ms < wait_ms) {
		allocwait_para.total_alloc_wait_max_ms = wait_ms;
		allocwait_para.total_alloc_wait_max_order = order;
	}
}

module_param_named(alloc_wait_h_ms, alloc_wait_h_ms, int, S_IRUGO | S_IWUSR);
module_param_named(alloc_wait_l_ms, alloc_wait_l_ms, int, S_IRUGO | S_IWUSR);
module_param_named(alloc_wait_log_ms, alloc_wait_log_ms, int, S_IRUGO | S_IWUSR);
module_param_named(alloc_wait_trig_ms, alloc_wait_trig_ms, int, S_IRUGO | S_IWUSR);
