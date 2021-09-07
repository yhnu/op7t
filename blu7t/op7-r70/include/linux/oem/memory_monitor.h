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

#ifndef _MEMORY_MONITOR_H_
#define _MEMORY_MONITOR_H_

struct alloc_wait_para {
	u64 total_alloc_wait_h_cnt;
	u64 total_alloc_wait_l_cnt;
	u64 fg_alloc_wait_h_cnt;
	u64 fg_alloc_wait_l_cnt;
	u64 total_alloc_wait_max_ms;
	u64 total_alloc_wait_max_order;
	u64 fg_alloc_wait_max_ms;
	u64 fg_alloc_wait_max_order;
};

extern void memory_alloc_monitor(gfp_t gfp_mask, unsigned int order, u64 wait_ms);
extern struct alloc_wait_para allocwait_para;
#endif /*_MEMORY_MONITOR_H_*/
