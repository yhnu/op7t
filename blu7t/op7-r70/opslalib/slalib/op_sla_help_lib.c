/*
 * Copyright (c) 2018-2019, The OnePlus corporation. All rights reserved.
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

#include <slalib/op_sla_help_lib.h>

#define GAME_LINK_SWITCH_TIME (10 * 60 * 100) //10minutes
#define PINGPONG_AVOID_TIME (60 * 60 * 1000) //60minutes

#define WLAN_SCORE_BAD_NUM	10
#define WLAN_SCORE_GOOD 65
#define WLAN_SCORE_BAD  55

struct op_sla_params_info {
	int sla_rtt;
	int wzry_rtt;
	int cjzc_rtt;
	int pubg_rtt;
	int qqcar_rtt;
};
struct op_sla_params_info sla_params_info = {
	.sla_rtt = 200,
	.wzry_rtt = 200,
	.cjzc_rtt = 300,
	.pubg_rtt = 300,
	.qqcar_rtt = 300,
};
struct op_game_app_info op_sla_game_app_list;
struct op_dev_info op_sla_info[IFACE_NUM];
int rtt_record_num = MAX_RTT_RECORD_NUM;
int rtt_queue[MAX_RTT_RECORD_NUM];
int rtt_rear;
int game_rtt_wan_detect_flag;
int game_data[5];
int op_sla_enable;
int game_start_state;
int sla_switch_enable;
int wlan_score_bad_count;

enum {
	GAME_WZRY = 1,
	GAME_WZRY_2,
	GAME_CJZC,
	GAME_PUBG,
	GAME_PUBG_TW,
	GAME_MOBILE_LEGENDS,
	GAME_AOV,
	GAME_JZPAJ,
	GAME_JZPAJ_TW,
	GAME_QQ_CAR,
	GAME_QQ_CAR_TW,
	GAME_BRAWLSTARS,
	GAME_CLASHROYALE_H2,
	GAME_CLASHROYALE,
	GAME_DWRG_H2,
	GAME_DWRG,
	GAME_DWRG_TW,
	GAME_MRZH_H2,
	GAME_MRZH,
	GAME_MRZH_TW,
};

int abs(int num)
{
	return (num >= 0 ? num : ((-1) * num));
}

int is_ping_pong(int game_type, int time_now)
{
	if (op_sla_game_app_list.switch_count[game_type] > 1 &&
	    op_sla_game_app_list.repeat_switch_time[game_type] != 0 &&
	    (time_now - op_sla_game_app_list.repeat_switch_time[game_type])
	    < PINGPONG_AVOID_TIME)
		return 1;
        return 0;
}

int get_app_rtt_threshold(int game_type, int game_lost_count)
{
	int max_rtt = sla_params_info.sla_rtt;

	if (game_type == GAME_WZRY || game_type == GAME_WZRY_2) {
		max_rtt = sla_params_info.wzry_rtt;
		if (rtt_rear == 4 && game_lost_count == 0 &&
		    rtt_queue[0] == MAX_GAME_RTT &&
		    rtt_queue[1] == MAX_GAME_RTT &&
		    rtt_queue[2] == MAX_GAME_RTT &&
		    rtt_queue[3] == MAX_GAME_RTT) {
			op_sla_game_app_list.rtt[game_type] = 0;
		}
	} else if (game_type == GAME_CJZC) {
		max_rtt = sla_params_info.cjzc_rtt;
	} else if (game_type == GAME_PUBG || game_type == GAME_PUBG_TW) {
		max_rtt = sla_params_info.pubg_rtt;
	} else if (game_type == GAME_QQ_CAR_TW) {
		max_rtt = sla_params_info.qqcar_rtt;
	}
	return max_rtt;
}

void op_rx_interval_error_estimator(int game_type, int time_error)
{
	int dropnum = 0;
	int gamethreshold = 0;

	if (game_type == GAME_WZRY || game_type == GAME_WZRY_2) {
	        dropnum = 5;
	        gamethreshold = 300;
	} else if (game_type == GAME_QQ_CAR) {
	        dropnum = 2;
	        gamethreshold = 1000;
	}
	if (dropnum != 0 &&
	    op_sla_game_app_list.special_rx_count[game_type] >= dropnum &&
	    (gamethreshold != 0 && time_error >= gamethreshold))
		op_sla_game_app_list.special_rx_error_count[game_type]++;
	else if (dropnum != 0 &&
		 op_sla_game_app_list.special_rx_count[game_type] >= dropnum &&
		 op_sla_game_app_list.special_rx_error_count[game_type])
		op_sla_game_app_list.special_rx_error_count[game_type]--;

	op_sla_game_app_list.special_rx_count[game_type]++;
}

void rttQueueEnqueue(int data)
{
	if (rtt_rear == rtt_record_num)
		return;
	rtt_queue[rtt_rear] = data;
	rtt_rear++;
}

void rttQueueDequeue(void)
{
	int i;

	if (rtt_rear == 0)
		return;
	for (i = 0; i < rtt_rear - 1; i++)
		rtt_queue[i] = rtt_queue[i + 1];
	rtt_rear--;
}

int average_rtt_queue(void)
{
	int sum = 0;
	int i = 0;

	for (i = 0; i < rtt_rear; i++)
		sum += rtt_queue[i] * (i + 1) / 10;
	return sum;
}

void op_game_rtt_estimator(int *game_data)
{
	int game_type = game_data[0];
	int rtt = game_data[1];
	int op_game_time_interval = game_data[2];
	int op_game_lost_count = game_data[3];
	int game_rtt_wan_detect_flag = game_data[4];
	int averagertt = 0;
	int game_rtt_detect_lag = 0;

	op_sla_game_app_list.rtt_num[game_type]++;
	if (op_sla_game_app_list.rtt_num[game_type] <= (rtt_record_num >> 1))
		return;

	if (rtt_rear == rtt_record_num) {
		rttQueueDequeue();
		rttQueueEnqueue(rtt);
		averagertt = average_rtt_queue();
	} else {
		rttQueueEnqueue(rtt);
	}

	if (game_type == GAME_WZRY || game_type == GAME_WZRY_2) {
		game_rtt_detect_lag =
			(abs(op_game_time_interval - 5000) >= 50) ? 1 : 0;
		//if game rtt not regular and last game rtt
		//over 300 ms
		if (game_rtt_detect_lag &&
		    rtt_rear == MAX_RTT_RECORD_NUM &&
		    rtt == MAX_GAME_RTT) {
			averagertt = MAX_GAME_RTT;
		//if game rtt continue over 200ms and current
		//rtt bigger than before
		} else if (rtt_rear == MAX_RTT_RECORD_NUM &&
			   (rtt_queue[rtt_rear - 2] >=
			    sla_params_info.wzry_rtt &&
			    rtt_queue[rtt_rear - 1] >=
			    sla_params_info.wzry_rtt)) {
			averagertt = MAX_GAME_RTT;
			//ct->op_game_lost_count
		} else if (op_game_lost_count >= 1 &&
			   rtt == MAX_GAME_RTT) {
			averagertt = MAX_GAME_RTT;
		}
	} else if (game_type == GAME_PUBG ||
		   game_type == GAME_PUBG_TW ||
		   game_type == GAME_AOV ||
		   game_type == GAME_QQ_CAR_TW) {
		if (op_game_lost_count >= 1 &&
		    rtt == MAX_GAME_RTT &&
		    game_rtt_wan_detect_flag) {
			averagertt = MAX_GAME_RTT;
		}
	} else if (game_type == GAME_CJZC) {
		if (op_game_lost_count >= 3 &&
		    rtt == MAX_GAME_RTT) {
			averagertt = MAX_GAME_RTT;
		}
	}
	op_sla_game_app_list.rtt[game_type] = averagertt;
}

int op_get_ct_cell_quality(int game_type)
{
	int score_base = 0;

	if (op_sla_game_app_list.mark[game_type] == CELLULAR_MARK)
		score_base = 10;

	if (game_type == GAME_WZRY || game_type == GAME_WZRY_2) {
		return (op_sla_info[CELLULAR_INDEX].cur_score
			>= (CELL_SCORE_BAD - score_base)) ? 1 : 0;
	} else if (game_type == GAME_CJZC || game_type == GAME_PUBG ||
		   game_type == GAME_PUBG_TW || game_type == GAME_QQ_CAR ||
		   game_type == GAME_QQ_CAR_TW) {
		return (op_sla_info[CELLULAR_INDEX].cur_score
			>= (-110 - score_base)) ? 1 : 0;
	} else {
		return (op_sla_info[CELLULAR_INDEX].cur_score
			>= (CELL_SCORE_BAD - score_base)) ? 1 : 0;
	}
}

int switch_to_cell(int cell_quality_good,
		   int game_rtt,
		   int gamelostcount,
		   int game_switch_interval,
		   int game_type) {
	int max_rtt = get_app_rtt_threshold(game_type, gamelostcount);

	if ((cell_quality_good && op_sla_info[CELLULAR_INDEX].netlink_valid &&
	     ((game_rtt != 0 && game_rtt >= max_rtt) ||
	      op_sla_game_app_list.special_rx_error_count[game_type] >= 2) &&
	      op_sla_game_app_list.mark[game_type] == WLAN_MARK) &&
	     (!op_sla_game_app_list.switch_time[game_type] ||
	      game_switch_interval > 30000))
		return 1;
	return 0;
}
int switch_to_wifi(int wlan_bad,
		   int game_rtt,
		   int gamelostcount,
		   int game_switch_interval,
		   int game_type) {
	int max_rtt = get_app_rtt_threshold(game_type, gamelostcount);

	if ((!wlan_bad && op_sla_info[WLAN_INDEX].netlink_valid &&
	     ((game_rtt != 0 && game_rtt >= max_rtt) ||
	      op_sla_game_app_list.special_rx_error_count[game_type] >= 2) &&
	      op_sla_game_app_list.mark[game_type] == CELLULAR_MARK) &&
	     (!op_sla_game_app_list.switch_time[game_type] ||
	      game_switch_interval > 30000))
		return 1;
	return 0;
}

void reset_sla_game_app_rx_error(int game_type)
{
	op_sla_game_app_list.special_rx_error_count[game_type] = 0;
	op_sla_game_app_list.special_rx_count[game_type] = 0;
}

void reset_sla_game_app_rtt(int game_type)
{
	op_sla_game_app_list.rtt[game_type] = 0;
	op_sla_game_app_list.rtt_num[game_type] = 0;
}

void record_sla_game_cell_state(int game_type,
				int game_switch_interval,
				int time_now)
{
	reset_sla_game_app_rx_error(game_type);
	reset_sla_game_app_rtt(game_type);
	op_sla_game_app_list.switch_count[game_type]++;
	if (op_sla_game_app_list.switch_count[game_type] > 1 &&
	    game_switch_interval < GAME_LINK_SWITCH_TIME) {
		op_sla_game_app_list.repeat_switch_time[game_type] =
			time_now;
	}
	op_sla_game_app_list.switch_time[game_type] = time_now;
	op_sla_game_app_list.mark[game_type] = CELLULAR_MARK;
}

void record_sla_game_wifi_state(int game_type,
				int game_switch_interval,
				int time_now)
{
	reset_sla_game_app_rx_error(game_type);
	reset_sla_game_app_rtt(game_type);
	op_sla_game_app_list.switch_count[game_type]++;
	if (game_switch_interval < GAME_LINK_SWITCH_TIME) {
		op_sla_game_app_list.repeat_switch_time[game_type] =
			time_now;
	}
	op_sla_game_app_list.switch_time[game_type] = time_now;
	op_sla_game_app_list.mark[game_type] = WLAN_MARK;
}

int get_lost_count_threshold(int game_type)
{
	if (game_type == GAME_WZRY || game_type == GAME_WZRY_2) {
		if (op_sla_game_app_list.mark[game_type] == CELLULAR_MARK)
			return 2;
		else
			return 1;
	} else if (game_type == GAME_PUBG || game_type == GAME_PUBG_TW ||
		   game_type == GAME_AOV || game_type == GAME_QQ_CAR_TW) {
		return 1;
	} else {
		return 3;
	}
}

int get_game_interval(int game_type, int game_interval)
{
	if (game_type == GAME_PUBG || game_type == GAME_PUBG_TW ||
	    game_type == GAME_QQ_CAR_TW) {
		return 5000;
	} else if (game_type == GAME_AOV) {
		return 2000;
	} else if (game_type == GAME_CJZC) {
		return 1000;
	} else {
		return game_interval;
	}
}

int check_wan_detect_flag(int game_type)
{
	if ((game_type == GAME_PUBG || game_type == GAME_PUBG_TW ||
	     game_type == GAME_AOV || game_type == GAME_QQ_CAR_TW) &&
	     !game_rtt_wan_detect_flag) {
		game_rtt_wan_detect_flag = 1;
		return 1;
	}
	return 0;
}

int is_detect_game_lost(int game_lost_count,
			int game_lost_count_threshold,
			int game_time_interval)
{
	if (op_sla_enable &&
	    game_lost_count >= game_lost_count_threshold &&
	    (game_time_interval > 300 || game_rtt_wan_detect_flag))
		return 1;
	return 0;
}

int is_support_detect_game_tx(int game_type,
                              int special_rx_pkt_last_timestamp)
{
	if (game_type == GAME_QQ_CAR &&
	    special_rx_pkt_last_timestamp)
		return 1;
	return 0;
}

void get_rx_pkt_threshold(int game_type,
			  int time_now,
			  int special_rx_pkt_last_timestamp,
			  int *rtt_callback)
{
	if (game_type == GAME_QQ_CAR) {
		rtt_callback[0] = 10000 * 1.2;
		rtt_callback[1] = 10000 / 2;
		rtt_callback[2] = time_now - special_rx_pkt_last_timestamp;
	}
}

int data_stall_detect(int lastspecialrxtiming,
		      int specialrxthreshold,
		      int datastalltimer,
		      int datastallthreshold)
{
	if (op_sla_enable &&
	    lastspecialrxtiming >= specialrxthreshold &&
	    datastalltimer >= datastallthreshold)
		return 1;
	return 0;
}

int get_game_tx_category(int game_type, int skb_len)
{
	if (game_type == GAME_CJZC) {
		return 1;
	} else if ((game_type == GAME_PUBG || game_type == GAME_PUBG_TW ||
		    game_type == GAME_AOV || game_type == GAME_QQ_CAR_TW) &&
		    skb_len == 33) {
		return 2;
	} else if (game_type == GAME_WZRY || game_type == GAME_WZRY_2) {
		if (skb_len == 47)
			return 3;
		else
			return 4;
	}
	return 0;
}

int get_game_rx_category(int game_type, unsigned int skb_len)
{
	if (game_type == GAME_QQ_CAR && skb_len == 83)
		return 1;
	else if ((game_type == GAME_WZRY || game_type == GAME_WZRY_2) &&
		  skb_len == 100)
		return 2;
	return 0;
}

int drop_pkt_check(int game_type, int skb_len)
{
	if (skb_len > 150 || (game_type == GAME_CJZC && skb_len == 123))
		return 1;
	return 0;
}

int is_support_rtt_wan_detect(int game_type)
{
	if (game_type == GAME_PUBG || game_type == GAME_PUBG_TW ||
	   game_type == GAME_AOV || game_type == GAME_QQ_CAR_TW)
		return 1;
	return 0;
}

int get_rx_interval_error(int game_category,
			  int time_now,
			  int rx_pkt_timestamp)
{
	if (game_category == 1)
		return abs((time_now - rx_pkt_timestamp) - 10000);
	else
		return abs((time_now - rx_pkt_timestamp) - 2000);
}

int is_need_check_game_rtt(int game_detect_status,
			   int game_timestamp,
			   int skb_len)
{
	if (game_detect_status == GAME_RTT_DETECTED_STREAM &&
	    game_timestamp && skb_len <= 150)
		return 1;
	return 0;
}

int get_game_rtt(int time_now, int game_timestamp, int game_type)
{
	int game_rtt = (time_now - game_timestamp);
	if ((game_type == GAME_WZRY || game_type == GAME_WZRY_2) &&
	    (game_rtt > MAX_GAME_RTT))
			return MAX_GAME_RTT;
	return game_rtt;
}

int is_skip_rx_rtt(int game_type, int game_time_interval)
{
	if (game_type == GAME_WZRY || game_type == GAME_WZRY_2) {
		if (game_time_interval < 1000 &&
		    op_sla_game_app_list.mark[game_type] == CELLULAR_MARK)
			return 1;
	} else {
		if (game_time_interval < 200)
			return 1;
	}
	return 0;
}

int is_support_game_mark(int game_type)
{
	if (game_type == GAME_CJZC || game_type == GAME_WZRY ||
	    game_type == GAME_WZRY_2)
		return 1;
	return 0;
}

int need_enable_sla(int cell_quality_good)
{
	if (!op_sla_enable && game_start_state &&
	     sla_switch_enable && cell_quality_good)
		return 1;
	return 0;
}

int need_enable_sla_for_wlan_score(int sla_screen_on)
{
	if (op_sla_info[WLAN_INDEX].cur_score <= (WLAN_SCORE_BAD - 4) &&
	    !op_sla_info[CELLULAR_INDEX].if_up &&
	    (!op_sla_enable && game_start_state &&
	     sla_switch_enable && sla_screen_on))
		return 1;
	return 0;
}

void set_sla_game_parameter(int num)
{
	op_sla_game_app_list.switch_time[num] = 0;
	op_sla_game_app_list.switch_count[num] = 0;
	op_sla_game_app_list.repeat_switch_time[num] = 0;
	op_sla_game_app_list.game_type[num] = num;
	op_sla_game_app_list.mark[num] = WLAN_MARK;
}

void op_init_game_online_info(int num, int time_now)
{
	op_sla_game_app_list.mark[num] = WLAN_MARK;
	op_sla_game_app_list.switch_time[num] = time_now;
	op_sla_game_app_list.switch_count[num] = 0;
	op_sla_game_app_list.repeat_switch_time[num] = 0;
}

int op_get_wlan_quality(void)
{
	if (wlan_score_bad_count >= WLAN_SCORE_BAD_NUM)
		return 1;
	return 0;
}

void update_wlan_score(void)
{
	if (op_sla_info[WLAN_INDEX].cur_score <= (WLAN_SCORE_BAD - 5)) {
		wlan_score_bad_count +=	WLAN_SCORE_BAD_NUM;
	} else if (op_sla_info[WLAN_INDEX].cur_score <= (WLAN_SCORE_BAD - 2)) {
		wlan_score_bad_count += 2;
	} else if (op_sla_info[WLAN_INDEX].cur_score <= WLAN_SCORE_BAD) {
		wlan_score_bad_count++;
	} else if (op_sla_info[WLAN_INDEX].cur_score >= WLAN_SCORE_GOOD) {
		wlan_score_bad_count = 0;
	} else if (op_sla_info[WLAN_INDEX].cur_score
		   >= (WLAN_SCORE_GOOD - 2) && wlan_score_bad_count >= 2) {
		wlan_score_bad_count -= 2;
	} else if (op_sla_info[WLAN_INDEX].cur_score
		   >= (WLAN_SCORE_GOOD - 5) && wlan_score_bad_count) {
		wlan_score_bad_count--;
	}

	if (wlan_score_bad_count > (2 * WLAN_SCORE_BAD_NUM))
		wlan_score_bad_count = 2 * WLAN_SCORE_BAD_NUM;
}
