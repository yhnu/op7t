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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "../opchain_define.h"
#include "opchain_proxy.h"
#include "opchain_struct_offset_helper.h"
#include <linux/atomic.h>

/* +1 for group leader*/
/* 6 fore * 3 histories*/
/* ********************************
   **rep 0***Render  ***Binder*****
   **rep 1***Leader  ***SF    *****
   **type ***UT_ETASK***UTASK *****
*/
#define UX_ENTRY_LEN        2
#define UX_TOTAL_ENTRIES    18
#define UX_GROUP_OTHER_ENTRIES	6
#define R_MAGIC_ID_0 ((unsigned int)0x50656E4F)
#define R_MAGIC_ID_1 ((unsigned int)0x3F73756C)
#define MAGIC_SIZE (sizeof(R_MAGIC_ID_0) + sizeof(R_MAGIC_ID_1) + 1)
#define CLAIMSTONS 16000000
#define UX_MIGRATE_LOAD_ADJ 20

#define UXTAG(t)   TASK_UTASK_TAG_R(t)
#define GUXTAG(t)  TASK_UTASK_TAG_R(TASK_GROUP_LEADER_R(t))
#define UXTIME(t)  TASK_UTASK_TAG_BASE_R(t)
#define GUXTIME(t) TASK_UTASK_TAG_BASE_R(TASK_GROUP_LEADER_R(t))
#define CHAIN_REP(pos, sub) (ux_chain.caches[pos].rep[sub])
#define CHAIN_TYPE(pos) (ux_chain.caches[pos].type)

#define opc_claim_bit_test(claim, cpu) (claim & ((1 << cpu) | (1 << (cpu + NUMS_CPU))))

static unsigned int binder_tag;
static unsigned long ux_realm_util;
static unsigned long ux_realm_claim_util;

static struct {
	atomic_t lru_pos;
	struct {
		/*pid*/
		int rep[UX_ENTRY_LEN];
		int type;
	} caches[UX_TOTAL_ENTRIES];
} ux_chain = {
	.lru_pos = ATOMIC_INIT(0),
	.caches = {{{ 0 }, 0 } }
};
static struct {
	atomic_t claim_counts;
	void *last_claimant;
	unsigned long long last_pass_time;
} claim_store[] = {
	[0 ... 31] = { ATOMIC_INIT(0), NULL, (unsigned long long)0 } /* max 32 cores supported, DONT extend this! */
};

static unsigned int ctech_opc_get_claims(void **rq);
static int ctech_opc_get_claim_on_cpu(int cpu, void *rq);

#if UX_DEBUG
#include <linux/sched.h>
#include <linux/module.h>
#include "opchain_helper.h"
int opchain_status_show_core(char *buf, const struct kernel_param *kp)
{
	unsigned int most_recent = atomic_read(&ux_chain.lru_pos), size = 0;
	char *buf_new;
	int iters = UX_TOTAL_ENTRIES;

	buf_new = buf;

	for (; iters > 0; iters--) {
		unsigned int pos = most_recent-- % UX_TOTAL_ENTRIES;
		if (!pos && !(CHAIN_REP(pos, 0) || CHAIN_REP(pos, 1)))
			break;
		if (CHAIN_TYPE(pos)) {
			printk(pr_fmt("%d, %d %d\n"), CHAIN_REP(pos, 0), CHAIN_REP(pos, 1), CHAIN_TYPE(pos));
			size = snprintf(buf_new, PAGE_SIZE - size, "%d, %d %d\n", CHAIN_REP(pos, 0), CHAIN_REP(pos, 1), CHAIN_TYPE(pos));
			buf_new += size;
			size = buf_new - buf;
		}
		else {
			printk(pr_fmt("%d, %d\n"), CHAIN_REP(pos, 0), CHAIN_REP(pos, 1));
			size = snprintf(buf_new, PAGE_SIZE - size, "%d, %d\n", CHAIN_REP(pos, 0), CHAIN_REP(pos, 1));
			buf_new += size;
			size = buf_new - buf;
		}
	}
	size += snprintf(buf_new, PAGE_SIZE - size,
			"tag %u", atomic_read(&ux_chain.lru_pos));
	buf_new += size;
	size = buf_new - buf;
	printk(pr_fmt("tag %u\n"), atomic_read(&ux_chain.lru_pos));
	size += snprintf(buf_new, PAGE_SIZE - size,
			"claims %x\n", opc_get_claims());
	buf_new += size;
	size = buf_new - buf;
	size += snprintf(buf_new, PAGE_SIZE - size,
			"claim_count %d %d %d %d\n", opc_get_claim_on_cpu(0), opc_get_claim_on_cpu(1), opc_get_claim_on_cpu(2), opc_get_claim_on_cpu(3));

	return size;
}
#endif
#if 0
static inline bool ctech_is_major_utask(int pid, u32 tag)
{
	return (CHAIN_REP((tag % UX_TOTAL_ENTRIES), 0) == pid);
}
#endif
static unsigned int ctech_is_opc_task(void *rq, void *t, int type)
{
	unsigned long long tag = UXTAG(t);
	unsigned long long avl_entries = 0;

	if (!tag || !chain_on)
		return false;

	/*100ms*/
	if (latest_threshold && (type & UT_CLK_BASE) &&
			(RQ_CLOCK_R(rq) - UXTIME(t) > latest_threshold))
		return false;

	if ((type & UT_ETASK) &&
			!(CHAIN_TYPE(tag % UX_TOTAL_ENTRIES) & UT_ETASK))
		return false;

	if (type & UT_LATEST_ONE)
		avl_entries = UX_GROUP_OTHER_ENTRIES;
	else
		avl_entries = UX_TOTAL_ENTRIES;
	tag = tag + avl_entries - atomic_read(&ux_chain.lru_pos);
	if (tag <= avl_entries) {
		return true;
	}
	else
		return false;
}

static inline void ctech_ux_clock_base_mark(void *rq, void *t)
{
	GUXTIME(t) = UXTIME(t) = RQ_CLOCK_R(rq);
}

static void ctech_ux_mark(void *rq, void *t, int ux_group)
{
	unsigned int cache_inpos, latest;

	latest = atomic_read(&ux_chain.lru_pos);
	cache_inpos = latest % UX_TOTAL_ENTRIES;

	if (!ctech_is_opc_task(rq, t, UT_LATEST_ONE)) {
		UXTAG(t) = atomic_inc_return(&ux_chain.lru_pos);
		GUXTAG(t) = UXTAG(t);
		cache_inpos = UXTAG(t) % UX_TOTAL_ENTRIES;
		if (!ux_group)
			binder_tag = UXTAG(t);
		if (CHAIN_REP(cache_inpos, 0) != TASK_PID_R(t)) {
			CHAIN_REP(cache_inpos, 0) = TASK_PID_R(t);
			CHAIN_TYPE(cache_inpos) = ux_group;
		}
		if (CHAIN_REP(cache_inpos, 1) != TASK_TGID_R(t))
			CHAIN_REP(cache_inpos, 1) = TASK_TGID_R(t);
#if UX_DEBUG
		printk(KERN_DEBUG pr_fmt("UX tag%llu, %d:%s, %d:%s %d %llu\n"), UXTAG(t), TASK_PID_R(t), TASK_COMM_R(t), TASK_TGID_R(t), TASK_COMM_R(TASK_GROUP_LEADER_R(t)), ux_group, RQ_CLOCK_R(rq));
#endif
	}
#if UX_DEBUG
	else {
		printk(KERN_DEBUG pr_fmt("UX tag%llu, only update time base %d:%s, %d:%s %llu\n"), UXTAG(t), TASK_PID_R(t), TASK_COMM_R(t), TASK_TGID_R(t), TASK_COMM_R(TASK_GROUP_LEADER_R(t)), RQ_CLOCK_R(rq));
	}
#endif
}

static void ctech_opc_add_to_chain(void *rq, void *t)
{
	ctech_ux_mark(rq, t, UT_ETASK);
	ctech_ux_clock_base_mark(rq, t);
}

static int ctech_opc_binder_parse(void *rq, void *cur,
		unsigned int dsize,unsigned int *data,
		int send)
{
	/* dont move forward to cache diverse histories as many as possible */
	if (chain_on && !TASK_EXIT_STATE_R(TASK_GROUP_LEADER_R(cur)) &&
			dsize > MAGIC_SIZE &&
			data[0] == R_MAGIC_ID_0 &&
			data[1] == R_MAGIC_ID_1) {
		if (send) {
			ctech_ux_mark(rq, cur, UT_ETASK);
			ctech_ux_clock_base_mark(rq, cur);
		}
		/*
		else {
			ctech_ux_mark(current, UTASK);
			ctech_ux_clock_base_mark(current, render_base);
		}
		*/
		return 1;
	}
	return 0;
}

static inline int atomic_dec_if_positive_ported(atomic_t *v)
{
	int c, old, dec;
	c = atomic_read(v);
	for (;;) {
		dec = c - 1;
		if (unlikely(dec < 0))
			break;
		old = atomic_cmpxchg((v), c, dec);
		if (likely(old == c))
			break;
		c = old;
	}
	return dec;
}

static inline unsigned int audit_claim_cpu_range(int cpu) {
	return (cpu >= 0 && cpu < ARRAY_SIZE(claim_store));
}

static void ctech_opc_task_switch(
	unsigned int enqueue, int cpu, void *p, void *rq,  unsigned long long clock)
{
	if (likely(p) && likely(audit_claim_cpu_range(cpu))) {
		if (enqueue) {
			if (ctech_is_opc_task(rq, p, UT_FORE)) {
				atomic_inc(&claim_store[cpu].claim_counts);
				claim_store[cpu].last_claimant = p;
				TASK_ETASK_CLAIM_R(p) = 1;
				if (TASK_CLAIM_CPU_R(p) != -1 && TASK_CLAIM_CPU_R(p) != cpu
						&& claim_store[TASK_CLAIM_CPU_R(p)].last_claimant == p) {
					claim_store[TASK_CLAIM_CPU_R(p)].last_pass_time = 0;
					TASK_CLAIM_CPU_R(p) = -1;
				}
			}
		} else {
			if (TASK_ETASK_CLAIM_R(p)) {
				/* remove claim */
				TASK_ETASK_CLAIM_R(p) = 0;
				if (!atomic_dec_if_positive_ported(&claim_store[cpu].claim_counts))
					TASK_CLAIM_CPU_R(p) = cpu;
				claim_store[cpu].last_pass_time = clock;
			}
		}
	}
}

/* return value:
 * > 0, return # of renders if any renders claim CPU
 * = 0, no render
 * < 0, if there's any render within a threshold (16ms) */
static int ctech_opc_get_claim_on_cpu(int cpu, void *rq)
{
	if (!chain_on)
		return 0;

	if (likely(audit_claim_cpu_range(cpu))) {
		int render_counts = atomic_read(&claim_store[cpu].claim_counts);
		u64 rq_time = RQ_CLOCK_R(rq);

		if (render_counts) {
			return render_counts;
		} else if (rq_time >= claim_store[cpu].last_pass_time &&
				   (rq_time - claim_store[cpu].last_pass_time) <= CLAIMSTONS) {
			return OP_CLAIM_S;
		}
	}
	/* zero weight owing to no foreground ux tasks */
	return 0;
}

static unsigned int ctech_opc_get_claims(void **rqs)
{
	unsigned int claims = 0;
	int idx, num_cpus, t_claim;

	if (chain_on) {
		for (idx = 0, num_cpus = NUMS_CPU; idx < num_cpus; idx++) {
			t_claim = ctech_opc_get_claim_on_cpu(idx, rqs[idx]);
			if (t_claim > 0)
				claims |= (1 << idx);
			else if (t_claim == OP_CLAIM_S)
				claims |= (1 << (idx + num_cpus));
		}
	}

	return claims;
}

static unsigned int ctech_opc_is_slave_task(void *rq, void *t)
{
	if (TASK_UTASK_SLAVE_R(t)) {
		if (RQ_CLOCK_R(rq) - UXTIME(t) < CLAIMSTONS)
			return true;
		else
			TASK_UTASK_SLAVE_R(t) = 0;
	}

	return false;
}

/* return value:
 * OP_PATH_NORMAL(-2) means normal case
 * OP_PATH_OCCUPIED(-1) means prev_cpu is occuping by other renders, ignore it
 *  0,1,2,3... means this task is render, keep running on previous CPU
 *  OP_PATH_CLAIM (-3) means there's render within 10ms, so this task should go somewhere else
 *      of course, this task would not meet any above conditions.
 *
 *  Assume dormant min_cpus is 2.
*/
static int ctech_opc_select_path(void **rqs, void *w_rq, void *t_rq, void *waker, void *t, int prev_cpu)
{
	unsigned int claims = ctech_opc_get_claims(rqs), last_cpu = NUMS_CPU - 1;
	unsigned int t_is_ux_top = ctech_is_opc_task(t_rq, t, UT_FORE);
	//int i;

	if (!chain_on)
		return OP_PATH_NORMAL;

	if (t_is_ux_top) {
		/*
		if (prev_cpu >= MIN_POWER_CPU && CPU_VIRTUAL_PLUG_IN(prev_cpu))
			return prev_cpu;
		for (i = NUMS_CPU - 1; i >= FIRST_BIG_CORE; i--)
			if (CPU_VIRTUAL_PLUG_IN(i) && cpumask_test_cpu(i, TASK_CPUS_ALLOWED_ADDR(t)) && !opc_claim_bit_test(claims, i) && opc_idle_get_state_idx(i) == -1)
				return i;
		*/
		return prev_cpu;
	} else if ((ctech_is_opc_task(w_rq, waker, UT_FORE) || ctech_opc_is_slave_task(w_rq, waker)) && TASK_GROUP_LEADER_R(t) == TASK_GROUP_LEADER_R(waker)){
		TASK_UTASK_SLAVE_R(t) = true;
		UXTIME(t) = UXTIME(waker);
		return OP_PATH_SLAVE;
	}

	if (claims & (1 << prev_cpu))
		return OP_PATH_OCCUPIED;

	if (claims & (1 << (prev_cpu + last_cpu + 1)))
		return OP_PATH_CLAIM;

	return OP_PATH_NORMAL;
}

unsigned long ctech_opc_cpu_util(unsigned long util, int cpu, void *t, void *rq, int prev_cpu)
{
	/*TODO: render real demand*/
	int get_claim = ctech_opc_get_claim_on_cpu(cpu, rq);

	if (!get_claim || TASK_UTASK_SLAVE_R(t))
		return util;

	if (get_claim <= 1 && prev_cpu != cpu)
		util += ux_realm_claim_util;
	else if (util < 1024)
		util += get_claim * ux_realm_util;

	return (util > 1024) ? 1024 : util;
}

static int ctech_opc_check_uxtop_cpu(int uxtop, int cpu)
{
	if (!uxtop || cpu >= FIRST_BIG_CORE)
		return true;
	return false;
}

void __init ctech_opchain_init(struct opchain_cb *cb, unsigned long util)
{
	cb->is_opc_task_t = ctech_is_opc_task;
	cb->opc_binder_pass_t = ctech_opc_binder_parse;
	cb->opc_task_switch_t = ctech_opc_task_switch;
	cb->opc_get_claim_on_cpu_t = ctech_opc_get_claim_on_cpu;
	cb->opc_get_claims_t = ctech_opc_get_claims;
	cb->opc_select_path_t = ctech_opc_select_path;
	cb->opc_cpu_util_t = ctech_opc_cpu_util;
	cb->opc_add_to_chain_t = ctech_opc_add_to_chain;
	cb->opc_check_uxtop_cpu_t = ctech_opc_check_uxtop_cpu;
	ux_realm_util = util;
	ux_realm_claim_util = ux_realm_util >> 1;
}
