/* Copyright (c) 2019 OnePlus. All rights reserved.
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

#include <slalib/op_sla.h>

static rwlock_t sla_lock;
static rwlock_t sla_game_lock;
static rwlock_t sla_game_rx_lock;

#define sla_write_lock()		  write_lock_bh(&sla_lock)
#define sla_write_unlock()		  write_unlock_bh(&sla_lock)

#define sla_game_write_lock()		  write_lock_bh(&sla_game_lock)
#define sla_game_write_unlock()		  write_unlock_bh(&sla_game_lock)

#define sla_game_rx_error_write_lock()	  write_lock_bh(&sla_game_rx_lock)
#define sla_game_rx_error_write_unlock()  write_unlock_bh(&sla_game_rx_lock)

static void init_rtt_queue_info(void)
{
	rtt_rear = 0;
	memset(rtt_queue, 0, sizeof(rtt_queue));
}

int get_app_type(int game_type)
{
	if (game_type >= GAME_BASE && game_type < (GAME_BASE + GAME_NUM))
		return GAME_TYPE;
	else
		return INIT_APP_TYPE;
}

static int op_sla_send_to_user(int msg_type, char *payload, int payload_len)
{
	int ret = -1;
	struct sk_buff *skbuff;
	struct nlmsghdr *nlh;

	if (!op_sla_pid) {
		if (op_sla_debug)
			pr_info("[op_sla] %s: op_sla_pid == 0!!\n", __func__);
		return ret;
	}

	//allocate new buffer cache
	skbuff = alloc_skb(NLMSG_SPACE(payload_len), GFP_ATOMIC);
	if (skbuff == NULL) {
		if (op_sla_debug)
			pr_info("[op_sla] %s: skbuff alloc_skb failed\n",
				__func__);
		return ret;
	}

	//fill in the data structure
	nlh = nlmsg_put(skbuff, 0, 0, msg_type, NLMSG_ALIGN(payload_len), 0);
	if (nlh == NULL) {
		if (op_sla_debug)
			pr_info("[op_sla] %s: nlmsg_put failaure\n", __func__);
		nlmsg_free(skbuff);
		return ret;
	}

	//compute nlmsg length
	nlh->nlmsg_len = NLMSG_HDRLEN + NLMSG_ALIGN(payload_len);

	if (payload != NULL)
		memcpy((char *)NLMSG_DATA(nlh), payload, payload_len);

	//set control field,sender's pid
#if (KERNEL_VERSION(3, 7, 0) > LINUX_VERSION_CODE)
	NETLINK_CB(skbuff).pid = 0;
#else
	NETLINK_CB(skbuff).portid = 0;
#endif
	NETLINK_CB(skbuff).dst_group = 0;

	//send data
	ret = netlink_unicast(op_sla_sock, skbuff, op_sla_pid, MSG_DONTWAIT);
	if (ret < 0) {
		if (op_sla_debug) {
			pr_info("[op_sla] %s: can not unicast skbuff,ret = %d\n",
				__func__, ret);
		}
		return 1;
	}
	return 0;
}

static int enable_op_sla_module(void)
{
	if (op_sla_debug) {
		pr_info("[op_sla] %s: wlan-if_up:%d cell-if_up:%d\n",
			__func__, op_sla_info[WLAN_INDEX].if_up,
			op_sla_info[CELLULAR_INDEX].if_up);
	}

	sla_write_lock();
	if (op_sla_info[WLAN_INDEX].if_up &&
	    op_sla_info[CELLULAR_INDEX].if_up) {
		op_sla_enable = 1;
		op_sla_send_to_user(SLA_ENABLED, NULL, 0);
	}
	sla_write_unlock();
	return 0;
}

static int *make_game_vals(struct nf_conn *ct, int game_rtt,
			   int game_type)
{
	game_data[0] = game_type;//game_type
	game_data[1] = game_rtt;//rtt
	game_data[2] = ct->op_game_time_interval;//op_game_time_interval
	game_data[3] = ct->op_game_lost_count;//op_game_lost_count
	game_data[4] = game_rtt_wan_detect_flag;
	return game_data;
}

static void rx_interval_error_estimator(int game_type, int time_error)
{
#ifdef CONFIG_SLA_ALGO
	op_rx_interval_error_estimator(game_type, time_error);
#endif
	if (op_sla_debug) {
		pr_info("[op_sla] %s: time_error:%d error_count:%d\n",
			__func__, time_error,
			op_sla_game_app_list.special_rx_error_count[game_type]);
	}
}

static void game_rtt_estimator(int game_type, int rtt, struct nf_conn *ct)
{
	int *game_data;

	game_data = make_game_vals(ct, rtt, game_type);
#ifdef CONFIG_SLA_ALGO
	op_game_rtt_estimator(game_data);
#endif
	if (op_sla_debug) {
		pr_info("[op_sla] %s: rtt:%d averagertt:%d\n",
			__func__, rtt,
			op_sla_game_app_list.rtt[game_type]);
	}
}

static void game_app_switch_network(struct nf_conn *ct, struct sk_buff *skb)
{
	int game_type = ct->op_app_type;
	int game_rtt = 0;
	int time_now = (int)(ktime_get_ns() / 1000000);
	int gamelostcount = ct->op_game_lost_count;
	int game_bp_info[4];
#ifdef CONFIG_SLA_ALGO
	int cell_quality_good = op_get_ct_cell_quality(game_type);
	int wlan_bad = op_get_wlan_quality();
#else
	int cell_quality_good = (op_sla_info[CELLULAR_INDEX].cur_score
				 >= CELL_SCORE_BAD) ? 1 : 0;
	int wlan_bad = 0;
#endif
	int game_switch_interval = time_now -
		op_sla_game_app_list.switch_time[game_type];

	if (!op_sla_enable)
		return;

	if (!game_start_state)
		return;

	game_rtt = op_sla_game_app_list.rtt[game_type];

	if (op_sla_debug) {
		pr_info("[op_sla] %s: cell_quality_good:%d wlan_bad:%d game_rtt:%d\n",
			__func__, cell_quality_good, wlan_bad, game_rtt);
		pr_info("[op_sla] %s: special_rx_error_count:%d game mark:%d time:%d\n",
			__func__,
			op_sla_game_app_list.special_rx_error_count[game_type],
			op_sla_game_app_list.mark[game_type],
			game_switch_interval);
		pr_info("[op_sla] %s: wlan valid:%d cell valid:%d\n", __func__,
			op_sla_info[WLAN_INDEX].netlink_valid,
			op_sla_info[CELLULAR_INDEX].netlink_valid);
		pr_info("[op_sla] %s: switch_count:%d repeat_switch_time:%d\n",
			__func__, op_sla_game_app_list.switch_count[game_type],
			op_sla_game_app_list.repeat_switch_time[game_type]);
		pr_info("[op_sla] %s: interval_for_switch_time:%d gamelostcount:%d\n",
			__func__,
			time_now -
			op_sla_game_app_list.repeat_switch_time[game_type],
			gamelostcount);
	}

#ifdef CONFIG_SLA_ALGO
	if (is_ping_pong(game_type, time_now))
		return;

	if (switch_to_cell(cell_quality_good,
			   game_rtt,
			   gamelostcount,
			   game_switch_interval,
			   game_type) || fw_set_game_mark == 1) {
#else
	if (fw_set_game_mark == 1) {
#endif
		fw_set_game_mark = -1;
		if (op_sla_debug) {
			pr_info("[op_sla] %s: game switch to cellular...\n",
				__func__);
		}
		init_rtt_queue_info();
#ifdef CONFIG_SLA_ALGO
		record_sla_game_cell_state(game_type,
					   game_switch_interval,
					   time_now);
#endif
		memset(game_bp_info, 0x0, sizeof(game_bp_info));
		game_bp_info[0] = game_type;
		game_bp_info[1] = CELLULAR_MARK;
		game_bp_info[2] = wlan_bad;
		game_bp_info[3] = cell_quality_good;
		op_sla_send_to_user(SLA_SWITCH_GAME_NETWORK,
				    (char *)game_bp_info,
				    sizeof(game_bp_info));
		return;
	}

#ifdef CONFIG_SLA_ALGO
	if (switch_to_wifi(wlan_bad,
			   game_rtt,
			   gamelostcount,
			   game_switch_interval,
			   game_type) || fw_set_game_mark == 0) {
#else
	if (fw_set_game_mark == 0) {
#endif
		fw_set_game_mark = -1;
		if (op_sla_debug) {
			pr_info("[op_sla] %s: game switch to wlan...\n",
				__func__);
		}
		init_rtt_queue_info();
#ifdef CONFIG_SLA_ALGO
		record_sla_game_wifi_state(game_type,
					   game_switch_interval,
					   time_now);
#endif
		memset(game_bp_info, 0x0, sizeof(game_bp_info));
		game_bp_info[0] = game_type;
		game_bp_info[1] = WLAN_MARK;
		game_bp_info[2] = wlan_bad;
		game_bp_info[3] = cell_quality_good;
		op_sla_send_to_user(SLA_SWITCH_GAME_NETWORK,
				    (char *)game_bp_info,
				    sizeof(game_bp_info));
		return;
	}
}

static void set_game_rtt_stream_up_info(struct nf_conn *ct, s64 now,
					u32 game_type)
{
	int game_rtt;
	int game_lost_count_threshold = 0;
	int game_interval;
	int game_lost_count;
	int game_time_interval;

#ifdef CONFIG_SLA_ALGO
	game_lost_count_threshold = get_lost_count_threshold(game_type);
#endif
	if (!ct->op_game_timestamp && !game_rtt_wan_detect_flag) {
		ct->op_game_timestamp = now;
		game_interval = (int)(now - ct->op_game_last_timestamp);
		ct->op_game_time_interval =
#ifdef CONFIG_SLA_ALGO
			get_game_interval(game_type,
					  game_interval);
#else
			game_interval;
#endif
		ct->op_game_last_timestamp = now;
		ct->op_game_lost_count = 0;
		if (ct->op_game_time_interval >= 10000)
			ct->op_game_timestamp = 0;
#ifdef CONFIG_SLA_ALGO
		if (check_wan_detect_flag(game_type))
			return;
#endif
	} else {
		ct->op_game_timestamp = now;
		ct->op_game_last_timestamp = now;
		ct->op_game_lost_count++;
		if (op_sla_debug) {
			pr_info("[op_sla] %s: lost game detect skb count:%d\n",
				__func__, ct->op_game_lost_count);
		}
		game_lost_count = ct->op_game_lost_count;
		game_time_interval = ct->op_game_time_interval;
#ifdef CONFIG_SLA_ALGO
		if (is_detect_game_lost(game_lost_count,
					game_lost_count_threshold,
					game_time_interval)) {
#else
		if (ct->op_game_lost_count >= game_lost_count_threshold) {
#endif
			game_rtt = MAX_GAME_RTT;
			if (op_sla_debug) {
				pr_info("[op_sla] %s: lost detect skb, game_type:%d\n",
					__func__, game_type);
				pr_info("[op_sla] %s: last game rtt:%d\n",
					__func__,
					op_sla_game_app_list.rtt[game_type]);
			}
			sla_game_write_lock();
			game_rtt_estimator(game_type, game_rtt, ct);
			sla_game_write_unlock();
			game_rtt_wan_detect_flag = 0;
		}
	}
}

static void detect_game_tx_stream(struct nf_conn *ct, struct sk_buff *skb,
				  enum ip_conntrack_info ctinfo)
{
	int game_type = ct->op_app_type;
	int time_now = (int)(ktime_get_ns() / 1000000);
#ifdef CONFIG_SLA_ALGO
	int specialrxthreshold = 0;
	int rtt_callback[3] = {0};
#endif
	int lastspecialrxtiming = 0;
	int datastallthreshold = 5000;
	int special_rx_pkt_last_timestamp =
		(int)ct->op_game_special_rx_pkt_timestamp;
	int rx_normal_time_record = (int)ct->op_game_rx_normal_time_record;
	int datastalltimer = time_now - rx_normal_time_record;

	if (op_sla_debug) {
		pr_info("[op_sla] %s: time_now:%d\n",
			__func__, time_now);
		pr_info("[op_sla] %s: op_game_special_rx_pkt_timestamp:%d\n",
			__func__, special_rx_pkt_last_timestamp);
		pr_info("[op_sla] %s: op_game_rx_normal_time_record:%d\n",
			__func__, rx_normal_time_record);
	}

#ifdef CONFIG_SLA_ALGO
	if (is_support_detect_game_tx(game_type,
				      special_rx_pkt_last_timestamp)) {
		get_rx_pkt_threshold(game_type,
				     time_now,
				     special_rx_pkt_last_timestamp,
				     rtt_callback);
		specialrxthreshold = rtt_callback[0];
		datastallthreshold = rtt_callback[1];
		lastspecialrxtiming = rtt_callback[2];

		if (data_stall_detect(lastspecialrxtiming,
				      specialrxthreshold,
				      datastalltimer,
				      datastallthreshold)) {
#else
	if (game_type) {
		if (datastalltimer >= datastallthreshold) {
#endif
			if (op_sla_debug) {
				pr_info("[op_sla] %s: lastspecialrxtiming:%d\n",
					__func__, lastspecialrxtiming);
				pr_info("[op_sla] %s: datastalltimer:%d\n",
					__func__, datastalltimer);
			}
			sla_game_rx_error_write_lock();
			rx_interval_error_estimator(game_type,
						    datastalltimer);
			sla_game_rx_error_write_unlock();
		}
	}
}

static void detect_game_rtt_stream(struct nf_conn *ct, struct sk_buff *skb,
				   enum ip_conntrack_info ctinfo)
{
	int same_count_max = 10;
	int up_max_count = 100;
	s64 time_now = ktime_get_ns() / 1000000;
	s64 time_interval = time_now - ct->op_game_timestamp;
	int game_type = ct->op_app_type;
	int skb_len = (int)skb->len;
	int game_category = 0;

#ifdef CONFIG_SLA_ALGO
	game_category = get_game_tx_category(game_type, skb_len);
#endif
	if (game_category == 1) {
		same_count_max = 6;
	} else if (game_category == 2) {
		ct->op_game_detect_status = GAME_RTT_DETECTED_STREAM;
		ct->op_game_up_count++;
	} else if (game_category == 3) {
		same_count_max = 3;
	} else if (game_category == 4) {
		return;
	}

	if (op_sla_debug) {
		pr_info("[op_sla] %s: game type:%d timestamp:%llu inter:%u\n",
			__func__, game_type, ct->op_game_timestamp,
			ct->op_game_time_interval);
		pr_info("[op_sla] %s: src port:%d ct state:%d up count:%d game_status:%d\n",
			__func__, ntohs(udp_hdr(skb)->source), XT_STATE_BIT(ctinfo),
			ct->op_game_up_count, ct->op_game_detect_status);
		pr_info("[op_sla] %s: skb len:%d, game_rtt_wan_detect_flag:%d\n",
			__func__, skb_len, game_rtt_wan_detect_flag);
	}

	if (ct->op_game_up_count == 0) {
		ct->op_game_up_count = 1;
		ct->op_game_same_count = 0;
		ct->op_game_lost_count = 0;
		ct->op_game_detect_status = GAME_RTT_DETECT_INITIAL;
		ct->op_game_skb_len = skb->len;
		ct->op_game_timestamp = time_now;
	} else if (ct->op_game_detect_status == GAME_RTT_DETECT_INITIAL) {
		ct->op_game_timestamp = time_now;
		if (skb->len > 150)
			return;
		if (ct->op_game_skb_len == skb->len) {
			if (time_interval < 300 || time_interval > 10000)
				return;
			ct->op_game_same_count++;
		} else {
			ct->op_game_skb_len = skb->len;
			ct->op_game_same_count = 0;
			ct->op_game_down_count = 0;
		}

		if (op_sla_debug) {
			pr_info("[op_sla] %s: interval_time:%llu up_count:%d\n",
				__func__, time_interval, ct->op_game_up_count);
			pr_info("[op_sla] %s: down count:%d same count:%d same_count_max:%d\n",
				__func__, ct->op_game_down_count, ct->op_game_same_count, same_count_max);
			pr_info("[op_sla] %s: ct->op_game_skb_len:%d\n",
				__func__, ct->op_game_skb_len);
		}

		if (ct->op_game_down_count >= same_count_max &&
		    ct->op_game_same_count >= same_count_max) {
#ifdef CONFIG_SLA_ALGO
			reset_sla_game_app_rtt(game_type);
#endif
			init_rtt_queue_info();
			ct->op_game_last_timestamp = time_now;
			ct->op_game_time_interval = time_interval;
			ct->op_game_detect_status = GAME_RTT_DETECTED_STREAM;
			ct->op_game_up_count++;
			return;
		}

		if (ct->op_game_up_count >= up_max_count) {
			ct->op_game_detect_status = GAME_SKB_COUNT_ENOUGH;
			if (op_sla_debug) {
				pr_info("[op_sla] %s: GAME_SKB_COUNT_ENOUGH!!\n",
					__func__);
			}
		}
		ct->op_game_up_count++;

	} else if (ct->op_game_detect_status == GAME_RTT_DETECTED_STREAM) {
#ifdef CONFIG_SLA_ALGO
		if (drop_pkt_check(game_type, skb_len))
			return;
#endif
		set_game_rtt_stream_up_info(ct, time_now, game_type);
	}
}

static int mark_game_app_skb(struct nf_conn *ct, struct sk_buff *skb,
			     enum ip_conntrack_info ctinfo)
{
	int game_type = ct->op_app_type;
	struct iphdr *iph = NULL;
	u32 ct_mark = ct->mark & GAME_UNSPEC_MASK;
	int ret = SLA_SKB_ACCEPT;

	if (!game_start_state &&
	    (ct_mark & GAME_UNSPEC_MARK))
		return SLA_SKB_ACCEPT;

	iph = ip_hdr(skb);
	if (iph && (iph->protocol == IPPROTO_UDP ||
		    iph->protocol == IPPROTO_TCP)) {
		ct_mark	= ct->mark & MARK_MASK;

		if (iph->protocol == IPPROTO_TCP &&
		    ((XT_STATE_BIT(ctinfo) &
		      XT_STATE_BIT(IP_CT_ESTABLISHED)) ||
		    (XT_STATE_BIT(ctinfo) & XT_STATE_BIT(IP_CT_RELATED)))) {
#ifdef CONFIG_SLA_ALGO
			if (is_support_game_mark(game_type)) {
#else
			if (game_type) {
#endif
				if (ct_mark == WLAN_MARK &&
				    op_sla_info[WLAN_INDEX].cur_score > 40) {
					return SLA_SKB_ACCEPT;
				} else if (ct_mark == CELLULAR_MARK) {
					skb->mark = CELLULAR_MARK;
					return SLA_SKB_MARKED;
				}
			}
		}

		skb->mark = op_sla_game_app_list.mark[game_type];

		if (ct_mark && skb->mark &&
		    ct_mark != skb->mark) {
			if (op_sla_debug) {
				pr_info("[op_sla] %s: reset ct proto:%u game type:%d\n",
					__func__, iph->protocol, game_type);
				pr_info("[op_sla] %s: ct mark:%x skb mark:%x\n",
					__func__, ct_mark, skb->mark);
			}
			if (!nf_ct_is_dying(ct) &&
			    nf_ct_is_confirmed(ct)) {
				nf_ct_kill(ct);
				return SLA_SKB_DROP;
			}
			skb->mark = ct_mark;
		}

		if (!ct_mark)
			ct->mark = (ct->mark & RTT_MASK) |
				   op_sla_game_app_list.mark[game_type];
		ret = SLA_SKB_MARKED;
	}

	return ret;
}

static bool is_game_app_skb(struct nf_conn *ct, struct sk_buff *skb,
			    enum ip_conntrack_info ctinfo)
{
	int game_type = ct->op_app_type;
	kuid_t uid;
	struct sock *sk = NULL;
	struct iphdr *iph = NULL;
	const struct file *filp = NULL;
	int app_type = get_app_type(game_type);
	int total = op_sla_game_app_list.count + GAME_BASE;

	if (app_type == INIT_APP_TYPE) {
		sk = skb_to_full_sk(skb);
		if (sk == NULL || sk->sk_socket == NULL)
			return false;

		filp = sk->sk_socket->file;
		if (filp == NULL)
			return false;

		iph = ip_hdr(skb);
		for (game_type = GAME_BASE; game_type < total; game_type++) {
			if (op_sla_game_app_list.uid[game_type]) {
				uid = make_kuid(&init_user_ns,
						op_sla_game_app_list.uid[game_type]);
				if (uid_eq(filp->f_cred->fsuid, uid)) {
					ct->op_app_type = game_type;

					if (!game_start_state &&
					    iph && IPPROTO_TCP ==
					    iph->protocol) {
						ct->mark = (ct->mark &
							    RTT_MASK) |
							    WLAN_MARK;
						ct->mark |= GAME_UNSPEC_MARK;
					} else {
						ct->mark = (ct->mark &
							    RTT_MASK) |
							    op_sla_game_app_list.mark[game_type];
					}
					return true;
				}
			}
		}
	} else if (app_type == GAME_TYPE) {
		return true;
	}
	return false;
}

static void detect_game_up_skb(struct sk_buff *skb)
{
	struct iphdr *iph = NULL;
	struct nf_conn *ct = NULL;
	enum ip_conntrack_info ctinfo;

	if (!op_sla_rtt_detect)
		return;

	ct = nf_ct_get(skb, &ctinfo);
	if (ct == NULL)
		return;

	if (!is_game_app_skb(ct, skb, ctinfo))
		return;

	//TCP and udp need to switch network
	iph = ip_hdr(skb);
	if (iph && iph->protocol == IPPROTO_UDP) {
		//only udp packet can active switch network to void updating
		//game with cell.
		sla_game_write_lock();
		game_app_switch_network(ct, skb);
		sla_game_write_unlock();

		detect_game_rtt_stream(ct, skb, ctinfo);
		detect_game_tx_stream(ct, skb, ctinfo);
	}
}

static void rtt_game_check(struct nf_conn *ct, struct sk_buff *skb)
{
	int time_now = (int)(ktime_get_ns() / 1000000);
	int game_rtt = 0;
	struct iphdr *iph = ip_hdr(skb);
	int game_type = ct->op_app_type;
	int game_detect_status = ct->op_game_detect_status;
	int game_timestamp = (int)ct->op_game_timestamp;
	int game_time_interval = (int)ct->op_game_time_interval;
	int skb_len = skb->len;
	int app_type = get_app_type(game_type);
#ifdef CONFIG_SLA_ALGO
	int cell_quality_good = op_get_ct_cell_quality(game_type);
#else
	int cell_quality_good = (op_sla_info[CELLULAR_INDEX].cur_score
				>= CELL_SCORE_BAD) ? 1 : 0;
#endif

	if (op_sla_debug) {
		if (iph && iph->protocol == IPPROTO_UDP &&
		    app_type == GAME_TYPE) {
			pr_info("[op_sla] %s: skb dev:%s game_status:%d\n",
				__func__, skb->dev->name,
				game_detect_status);
			pr_info("[op_sla] %s: game type:%d lost count:%d interval_time:%u\n",
				__func__, game_type, ct->op_game_lost_count,
				game_time_interval);
			pr_info("[op_sla] %s: game mark:%x protp:%d src_port:%d skb len:%d\n",
				__func__, op_sla_game_app_list.mark[game_type],
				iph->protocol, ntohs(udp_hdr(skb)->dest),
				skb_len);
			pr_info("[op_sla] %s: time_now:%d game_timeStamp:%d\n",
				__func__, time_now, game_timestamp);
		}
	}

#ifdef CONFIG_SLA_ALGO
	if (is_support_rtt_wan_detect(game_type)) {
#else
	if (game_type) {
#endif
		if (iph && iph->protocol == IPPROTO_UDP)
			game_rtt_wan_detect_flag = 0;
	}

	if (app_type == GAME_TYPE)
		ct->op_game_down_count++;

	if (!iph || iph->protocol != IPPROTO_UDP)
		return;

#ifdef CONFIG_SLA_ALGO
	if (is_need_check_game_rtt(game_detect_status,
				   game_timestamp,
				   skb_len)) {
		game_rtt = get_game_rtt(time_now, game_timestamp, game_type);
#else
	if (game_type) {
#endif
		if (game_rtt <= 0) {
			if (op_sla_debug) {
				pr_info("[op_sla] %s: invalid RTT:%dms\n",
					__func__,  game_rtt);
			}
			ct->op_game_timestamp = 0;
			return;
		}
		ct->op_game_timestamp = 0;
#ifdef CONFIG_SLA_ALGO
		if (is_skip_rx_rtt(game_type, game_time_interval))
			return;

		if (need_enable_sla(cell_quality_good)) {
#else
		if (cell_quality_good) {
#endif
			if (op_sla_debug) {
				pr_info("[op_sla] %s: send SLA_ENABLE\n",
					__func__);
			}
			op_sla_send_to_user(SLA_ENABLE, NULL, 0);
		}
		if (op_sla_debug) {
			pr_info("[op_sla] %s: game_rtt = %d\n", __func__,
				op_sla_game_app_list.rtt[game_type]);
		}
		ct->op_game_lost_count = 0;
		sla_game_write_lock();
		game_rtt_estimator(game_type, game_rtt, ct);
		sla_game_write_unlock();
	}
}

static void rx_interval_error_check(struct nf_conn *ct, struct sk_buff *skb)
{
	int time_now = (int)(ktime_get_ns() / 1000000);
	int rx_interval = 0;
	int special_rx_interval_error = 0;
	struct iphdr *iph = ip_hdr(skb);
	int game_type = ct->op_app_type;
	int skb_len = skb->len;
	int rx_pkt_timestamp = (int)ct->op_game_special_rx_pkt_timestamp;
	int app_type = get_app_type(game_type);
#ifdef CONFIG_SLA_ALGO
	int game_category = get_game_rx_category(game_type, skb_len);
	int cell_quality_good = op_get_ct_cell_quality(game_type);
#else
	int cell_quality_good = (op_sla_info[CELLULAR_INDEX].cur_score
				>= CELL_SCORE_BAD) ? 1 : 0;
#endif

	if (app_type == GAME_TYPE)
		ct->op_game_rx_normal_time_record = time_now;

	if (!iph || iph->protocol != IPPROTO_UDP)
		return;

#ifdef CONFIG_SLA_ALGO
	if (game_category == 1) {
#else
	if (game_type) {
#endif
		if (rx_pkt_timestamp) {
#ifdef CONFIG_SLA_ALGO
			special_rx_interval_error =
				get_rx_interval_error(game_category,
						      time_now,
						      rx_pkt_timestamp);
#endif
			sla_game_rx_error_write_lock();
			rx_interval_error_estimator(game_type,
						    special_rx_interval_error);
			sla_game_rx_error_write_unlock();
#ifdef CONFIG_SLA_ALGO
		} else {
			reset_sla_game_app_rx_error(game_type);
#endif
		}
		ct->op_game_special_rx_pkt_timestamp = time_now;
		if (op_sla_debug) {
			pr_info("[op_sla] %s: skb_len:%d rx_interval:%dms\n",
				__func__, skb_len, rx_interval);
			pr_info("[op_sla] %s: special_rx_interval_error:%dms\n",
				__func__, special_rx_interval_error);
		}
#ifdef CONFIG_SLA_ALGO
		if (need_enable_sla(cell_quality_good)) {
#else
		if (cell_quality_good) {
#endif
			if (op_sla_debug) {
				pr_info("[op_sla] %s: send SLA_ENABLE\n",
					__func__);
			}
			op_sla_send_to_user(SLA_ENABLE, NULL, 0);
		}
	}

#ifdef CONFIG_SLA_ALGO
	if (game_category == 2) {
#else
	if (game_type) {
#endif
		if (ct->op_game_special_rx_pkt_timestamp) {
#ifdef CONFIG_SLA_ALGO
			special_rx_interval_error =
				get_rx_interval_error(game_category,
						      time_now,
						      rx_pkt_timestamp);
		} else {
			reset_sla_game_app_rx_error(game_type);
#endif
		}
		ct->op_game_special_rx_pkt_timestamp = time_now;
		if (op_sla_debug) {
			pr_info("[op_sla] %s: rx_interval:%dms\n",
				__func__, rx_interval);
			pr_info("[op_sla] %s: special_rx_interval_error:%dms\n",
				__func__, special_rx_interval_error);
		}
#ifdef CONFIG_SLA_ALGO
		if (need_enable_sla(cell_quality_good)) {
#else
		if (cell_quality_good) {
#endif
			if (op_sla_debug) {
				pr_info("[op_sla] %s: send SLA_ENABLE\n",
					__func__);
			}
			op_sla_send_to_user(SLA_ENABLE, NULL, 0);
		}
	}
}

static unsigned int op_sla_rx_calc(void *priv,
				   struct sk_buff *skb,
				   const struct nf_hook_state *state)
{
	struct nf_conn *ct = NULL;
	enum ip_conntrack_info ctinfo;

	if (!op_sla_rtt_detect)
		return NF_ACCEPT;
	ct = nf_ct_get(skb, &ctinfo);
	if (ct == NULL)
		return NF_ACCEPT;
	rtt_game_check(ct, skb);
	rx_interval_error_check(ct, skb);
	return NF_ACCEPT;
}

static bool is_skb_pre_bound(struct sk_buff *skb)
{
	u32 pre_mark = skb->mark & 0x10000;

	if (pre_mark == 0x10000)
		return true;
	return false;
}

static int sla_skb_reroute(struct sk_buff *skb, struct nf_conn *ct,
			   const struct nf_hook_state *state)
{
	int err;

	err = ip_route_me_harder(state->net, skb, RTN_UNSPEC);
	if (err < 0)
		return NF_DROP_ERR(err);
	if (op_sla_debug)
		pr_info("[op_sla] %s: skb->mark=%x\n", __func__, skb->mark);
	return NF_ACCEPT;
}

static int handle_game_app_skb(struct nf_conn *ct, struct sk_buff *skb,
			       enum ip_conntrack_info ctinfo,
			       const struct nf_hook_state *state)
{
	int ret = SLA_SKB_CONTINUE;

	ret = mark_game_app_skb(ct, skb, ctinfo);
	if (ret == SLA_SKB_MARKED)
		return sla_skb_reroute(skb, ct, state);
	else if (ret == SLA_SKB_ACCEPT)
		return NF_ACCEPT;
	else if (ret == SLA_SKB_DROP)
		return NF_DROP;
	return NF_ACCEPT;
}

static int sla_mark_skb(struct sk_buff *skb, const struct nf_hook_state *state)
{
	struct nf_conn *ct = NULL;
	enum ip_conntrack_info ctinfo;
	int app_type;
	int game_type;

	//if wlan assistant has change network to cell, do not mark SKB
	if (op_sla_def_net)
		return NF_ACCEPT;

	ct = nf_ct_get(skb, &ctinfo);

	if (ct == NULL)
		return NF_ACCEPT;

	if (is_skb_pre_bound(skb))
		return NF_ACCEPT;

	game_type = ct->op_app_type;
	app_type = get_app_type(game_type);

	if (app_type == GAME_TYPE)
		return handle_game_app_skb(ct, skb, ctinfo, state);

	return NF_ACCEPT;
}

// op sla hook function, mark skb and rerout skb
static unsigned int op_sla(void *priv,
			   struct sk_buff *skb,
			   const struct nf_hook_state *state)
{
	int ret = NF_ACCEPT;
	struct nf_conn *ct = NULL;
	enum ip_conntrack_info ctinfo;

	ct = nf_ct_get(skb, &ctinfo);
	if (ct == NULL)
		return NF_ACCEPT;

	detect_game_up_skb(skb);

	if (op_sla_enable) {
		ret = sla_mark_skb(skb, state);
	}
	return ret;
}

static void init_game_online_info(void)
{
	int i = 0;
	int time_now = (int)ktime_get_ns() / 1000000;
	int total = op_sla_game_app_list.count + GAME_BASE;

	if (op_sla_debug)
		pr_info("[op_sla] %s\n", __func__);
	sla_game_write_lock();
	for (i = 0 + GAME_BASE; i < total; i++) {
#ifdef CONFIG_SLA_ALGO
		op_init_game_online_info(i, time_now);
#else
		op_sla_game_app_list.switch_time[i] = time_now;
#endif
	}
	sla_game_write_unlock();
}

static struct nf_hook_ops op_sla_ops[] __read_mostly = {
	{
		.hook		= op_sla,
		.pf		= NFPROTO_IPV4,
		.hooknum	= NF_INET_LOCAL_OUT,
		.priority	= NF_IP_PRI_CONNTRACK + 1,
	},
	{
		.hook		= op_sla_rx_calc,
		.pf		= NFPROTO_IPV4,
		.hooknum	= NF_INET_LOCAL_IN,
		.priority	= NF_IP_PRI_FILTER + 1,
	},
	{ }
};

static int op_sla_set_debug(struct nlmsghdr *nlh)
{
	op_sla_debug = *(u32 *)NLMSG_DATA(nlh);
	if (op_sla_debug)
		pr_info("[op_sla] %s: set debug = %d\n",
			__func__, op_sla_debug);
	return	0;
}

static int op_sla_set_game_mark(struct nlmsghdr *nlh)
{
	fw_set_game_mark = *(u32 *)NLMSG_DATA(nlh);
	if (op_sla_debug)
		pr_info("[op_sla] %s: game mark= %d\n",
			__func__, fw_set_game_mark);
	return  0;
}

static int op_sla_set_default_network(struct nlmsghdr *nlh)
{
	op_sla_def_net = *(u32 *)NLMSG_DATA(nlh);
	if (op_sla_debug)
		pr_info("[op_sla] %s: set default network = %d\n", __func__,
			op_sla_def_net);
	return 0;
}

static int disable_op_sla_module(void)
{
	if (op_sla_debug)
		pr_info("[op_sla] %s: op_sla_enable=%d\n",
			__func__, op_sla_enable);
	sla_write_lock();
	if (op_sla_enable) {
		op_sla_enable = 0;
		init_game_online_info();
		op_sla_send_to_user(SLA_DISABLED, NULL, 0);
	}
	sla_write_unlock();
	return 0;
}

static int op_sla_iface_up(struct nlmsghdr *nlh)
{
	sla_write_lock();
	if (op_sla_debug)
		pr_info("[op_sla] %s: enter type=%d\n",
			__func__, nlh->nlmsg_type);

	if (nlh->nlmsg_type == SLA_WIFI_UP) {
		op_sla_info[WLAN_INDEX].if_up = 1;
	} else if (nlh->nlmsg_type == SLA_CELLULAR_UP) {
		op_sla_info[CELLULAR_INDEX].if_up = 1;
	}
	sla_write_unlock();
	return 0;
}

static int op_sla_iface_down(struct nlmsghdr *nlh)
{
	int index = -1;

	if (op_sla_debug)
		pr_info("[op_sla] %s: type=%d\n", __func__, nlh->nlmsg_type);

	if (nlh->nlmsg_type == SLA_WIFI_DOWN) {
		index = WLAN_INDEX;
	} else if (nlh->nlmsg_type == SLA_CELLULAR_DOWN) {
		index = CELLULAR_INDEX;
	}

	if (index != -1) {
		sla_write_lock();
		memset(&op_sla_info[index], 0x0,
		       sizeof(struct op_dev_info));
		sla_write_unlock();
	}
	return 0;
}

static int op_sla_get_pid(struct sk_buff *skb, struct nlmsghdr *nlh)
{
	op_sla_pid = NETLINK_CB(skb).portid;
	if (op_sla_debug)
		pr_info("[op_sla] %s: op_sla_pid = %u\n", __func__, op_sla_pid);
	return 0;
}

static int op_sla_update_wlan_score(struct nlmsghdr *nlh)
{
	int *score = (int *)NLMSG_DATA(nlh);

	if (op_sla_debug)
		pr_info("[op_sla] %s: score=%d\n", __func__, *score);

	op_sla_info[WLAN_INDEX].cur_score = *score;

#ifdef CONFIG_SLA_ALGO
	update_wlan_score();
	if (need_enable_sla_for_wlan_score(sla_screen_on)) {
#else
	if (sla_screen_on) {
#endif
		if (op_sla_debug)
			pr_info("[op_sla] %s: send SLA_ENABLE\n",
				__func__);
		op_sla_send_to_user(SLA_ENABLE, NULL, 0);
	}
	return 0;
}

static int op_sla_set_game_app_uid(struct nlmsghdr *nlh)
{
	int i;
	u32 *info = (u32 *)NLMSG_DATA(nlh);
	int total;

	memset(&op_sla_game_app_list, 0x0, sizeof(struct op_game_app_info));
	init_rtt_queue_info();
	op_sla_game_app_list.count = info[0];
	total = op_sla_game_app_list.count + GAME_BASE;
	if (op_sla_game_app_list.count > 0 &&
	    op_sla_game_app_list.count <= GAME_NUM) {
		for (i = 0 + GAME_BASE; i < total; i++) {
			op_sla_game_app_list.uid[i] = info[i];
#ifdef CONFIG_SLA_ALGO
			set_sla_game_parameter(i);
#endif
			if (op_sla_debug)
				pr_info("[op_sla] %s: index=%d uid=%d\n",
					__func__,
					op_sla_game_app_list.game_type[i],
					op_sla_game_app_list.uid[i]);
		}
	}
	return 0;
}

static int op_sla_set_netlink_valid(struct nlmsghdr *nlh)
{
	u32 *info = (u32 *)NLMSG_DATA(nlh);

	op_sla_info[WLAN_INDEX].netlink_valid = info[0];
	op_sla_info[CELLULAR_INDEX].netlink_valid = info[1];
	if (op_sla_debug)
		pr_info("[op_sla] %s: wlan valid:%d cell valid:%d\n",
			__func__,
			op_sla_info[WLAN_INDEX].netlink_valid,
			op_sla_info[CELLULAR_INDEX].netlink_valid);
	return 0;
}

static int op_sla_set_game_rtt_detecting(struct nlmsghdr *nlh)
{
	op_sla_rtt_detect = (nlh->nlmsg_type == SLA_ENABLE_GAME_RTT) ? 1 : 0;
	if (op_sla_debug)
		pr_info("[op_sla] %s: set game rtt detect:%d\n", __func__,
			op_sla_rtt_detect);
	return 0;
}

static int op_sla_set_switch_state(struct nlmsghdr *nlh)
{
	u32 *switch_enable = (u32 *)NLMSG_DATA(nlh);

	sla_switch_enable = *switch_enable;
	if (op_sla_debug)
		pr_info("[op_sla] %s: sla switch:%d\n",
			__func__, sla_switch_enable);
	return 0;
}

static int op_sla_update_screen_state(struct nlmsghdr *nlh)
{
	u32 *screen_state = (u32 *)NLMSG_DATA(nlh);

	sla_screen_on =	(*screen_state);
	if (op_sla_debug) {
		pr_info("[op_sla] %s: update screen state = %d\n", __func__,
			sla_screen_on);
	}
	return	0;
}

static int op_sla_update_cell_score(struct nlmsghdr *nlh)
{
	int *score = (int *)NLMSG_DATA(nlh);

	op_sla_info[CELLULAR_INDEX].cur_score = *score;

	if (op_sla_debug) {
		pr_info("[op_sla] %s: update cell score:%d\n", __func__,
			op_sla_info[CELLULAR_INDEX].cur_score);
	}
	return	0;
}

static int op_sla_set_game_start_state(struct nlmsghdr *nlh)
{
	int *data = (int *)NLMSG_DATA(nlh);

	game_start_state = *data;
	if (op_sla_debug) {
		pr_info("[op_sla] %s: set game_start_state = %d\n", __func__,
			game_start_state);
	}
	return	0;
}

static int sla_netlink_rcv_msg(struct sk_buff *skb, struct nlmsghdr *nlh,
			       struct netlink_ext_ack *extack)
{
	int ret = 0;

	switch (nlh->nlmsg_type) {
	case SLA_ENABLE:
		ret = enable_op_sla_module();
		break;
	case SLA_DISABLE:
		ret = disable_op_sla_module();
		break;
	case SLA_WIFI_UP:
	case SLA_CELLULAR_UP:
		ret = op_sla_iface_up(nlh);
		break;
	case SLA_WIFI_DOWN:
	case SLA_CELLULAR_DOWN:
		ret = op_sla_iface_down(nlh);
		break;
	case SLA_NOTIFY_PID:
		ret = op_sla_get_pid(skb, nlh);
		break;
	case SLA_NOTIFY_WIFI_SCORE:
		ret = op_sla_update_wlan_score(nlh);
		break;
	case SLA_NOTIFY_GAME_UID:
		ret = op_sla_set_game_app_uid(nlh);
		break;
	case SLA_SET_NETWORK_VALID:
		ret = op_sla_set_netlink_valid(nlh);
		break;
	case SLA_ENABLE_GAME_RTT:
	case SLA_DISABLE_GAME_RTT:
		ret = op_sla_set_game_rtt_detecting(nlh);
		break;
	case SLA_NOTIFY_SWITCH_STATE:
		ret = op_sla_set_switch_state(nlh);
		break;
	case SLA_NOTIFY_SCREEN_STATE:
		ret = op_sla_update_screen_state(nlh);
		break;
	case SLA_NOTIFY_CELL_SCORE:
		ret = op_sla_update_cell_score(nlh);
		break;
	case SLA_SET_DEBUG:
		op_sla_set_debug(nlh);
		break;
	case SLA_SET_GAME_MARK:
		op_sla_set_game_mark(nlh);
		break;
	case SLA_NOTIFY_DEFAULT_NETWORK:
		op_sla_set_default_network(nlh);
		break;
	case SLA_NOTIFY_GAME_STATE:
		op_sla_set_game_start_state(nlh);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static void sla_netlink_rcv(struct sk_buff *skb)
{
	mutex_lock(&sla_netlink_mutex);
	netlink_rcv_skb(skb, &sla_netlink_rcv_msg);
	mutex_unlock(&sla_netlink_mutex);
}

static int op_sla_netlink_init(void)
{
	struct netlink_kernel_cfg cfg = {
		.input	= sla_netlink_rcv,
	};

	op_sla_sock = netlink_kernel_create(&init_net, NETLINK_OP_SLA, &cfg);
	return op_sla_sock == NULL ? -ENOMEM : 0;
}

static void op_sla_netlink_exit(void)
{
	netlink_kernel_release(op_sla_sock);
	op_sla_sock = NULL;
}

static void init_parameter(void)
{
	op_sla_rtt_detect = 1;
	op_sla_debug = 0;
	fw_set_game_mark = -1;
	op_sla_def_net = 0;    //WLAN->0 CELL->1
	game_start_state = 0;
	game_rtt_wan_detect_flag = 0;
	sla_switch_enable = 0;
	sla_screen_on = 1;
	rtt_rear = 0;
}

static int __init op_sla_init(void)
{
	int ret = 0;

	init_parameter();
	rwlock_init(&sla_lock);
	rwlock_init(&sla_game_lock);
	rwlock_init(&sla_game_rx_lock);

	ret = op_sla_netlink_init();
	if (ret < 0) {
		pr_info("[op_sla] %s: module can not init op sla netlink.\n",
			__func__);
	}

	ret |= nf_register_net_hooks(&init_net,
		op_sla_ops, ARRAY_SIZE(op_sla_ops));
	if (ret < 0) {
		pr_info("[op_sla] %s: module can not register netfilter ops.\n",
			__func__);
	}

	return ret;
}

static void __exit op_sla_deinit(void)
{
	op_sla_netlink_exit();
	nf_unregister_net_hooks(&init_net, op_sla_ops, ARRAY_SIZE(op_sla_ops));
}

module_init(op_sla_init);
module_exit(op_sla_deinit);
