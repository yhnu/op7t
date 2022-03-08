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

#define MAX_RTT_RECORD_NUM	4
#define MAX_GAME_RTT	300

#define CELL_SCORE_BAD	-100

#define INIT_APP_TYPE	0
#define GAME_TYPE	1

#define GAME_BASE	1
#define GAME_NUM	64

#define MARK_MASK		0x0fff
#define RTT_MASK		0xf000
#define GAME_UNSPEC_MASK	0x8000

#define WLAN_INDEX	0
#define CELLULAR_INDEX	1

#define IFACE_NUM	2

struct op_game_app_info {
	int count;
	int game_type[GAME_NUM];
	int uid[GAME_NUM];
	int rtt[GAME_NUM];
	int special_rx_error_count[GAME_NUM];
	int special_rx_count[GAME_NUM];
	int mark[GAME_NUM];
	int switch_time[GAME_NUM];
	int switch_count[GAME_NUM];
	int repeat_switch_time[GAME_NUM];
	int rtt_num[GAME_NUM];
};

struct op_dev_info {
	int if_up;
	int cur_score;
	int netlink_valid;
};

/* NLMSG_MIN_TYPE is 0x10,so we start at 0x11 */
enum {
	SLA_NOTIFY_WIFI_SCORE = 0x11,
	SLA_NOTIFY_PID = 0x12,
	SLA_ENABLE = 0x13,
	SLA_DISABLE = 0x14,
	SLA_WIFI_UP = 0x15,
	SLA_CELLULAR_UP = 0x16,
	SLA_WIFI_DOWN = 0x17,
	SLA_CELLULAR_DOWN = 0x18,
	SLA_CELLULAR_NET_MODE = 0x19,
	SLA_NOTIFY_GAME_UID = 0x1A,
	SLA_NOTIFY_GAME_RTT = 0x1B,
	SLA_NOTIFY_WHITE_LIST_APP = 0x1C,
	SLA_ENABLED = 0x1D,
	SLA_DISABLED = 0x1E,
	SLA_ENABLE_GAME_RTT = 0x1F,
	SLA_DISABLE_GAME_RTT = 0x20,
	SLA_NOTIFY_SWITCH_STATE = 0x21,
	SLA_NOTIFY_SPEED_RTT = 0x22,
	SLA_SWITCH_GAME_NETWORK  = 0x23,
	SLA_NOTIFY_SCREEN_STATE = 0x24,
	SLA_NOTIFY_CELL_SCORE = 0x25,
	SLA_SHOW_DIALOG_NOW = 0x26,
	SLA_NOTIFY_SHOW_DIALOG = 0x27,
	SLA_SEND_APP_TRAFFIC = 0x28,
	SLA_SEND_GAME_APP_STATISTIC = 0x29,
	SLA_GET_SYN_RETRAN_INFO = 0x2A,
	SLA_GET_SPEED_UP_APP = 0x2B,
	SLA_SET_DEBUG = 0x2C,
	SLA_NOTIFY_DEFAULT_NETWORK = 0x2D,
	SLA_NOTIFY_PARAMS = 0x2E,
	SLA_NOTIFY_GAME_STATE = 0x2F,
	SLA_ENABLE_LINK_TURBO = 0x30,
	SLA_DISABLE_LINK_TURBO = 0x31,
	SLA_SET_GAME_MARK = 0x32,
	SLA_SET_NETWORK_VALID = 0x33,
};

enum {
	SLA_SKB_ACCEPT,
	SLA_SKB_CONTINUE,
	SLA_SKB_MARKED,
	SLA_SKB_REMARK,
	SLA_SKB_DROP,
};

enum {
	WLAN_MARK_BIT = 8, //WLAN mark value,mask 0x0fff
	WLAN_MARK = (1 << WLAN_MARK_BIT),

	CELLULAR_MARK_BIT = 9, //cellular mark value  mask 0x0fff
	CELLULAR_MARK = (1 << CELLULAR_MARK_BIT),

	GAME_UNSPEC_MARK_BIT = 15, //mark game skb when game not start
	GAME_UNSPEC_MARK = (1 << GAME_UNSPEC_MARK_BIT),
};

enum {
	GAME_RTT_DETECT_INITIAL = 0,
	GAME_SKB_COUNT_ENOUGH,
	GAME_RTT_DETECTED_STREAM,
};

#ifdef CONFIG_SLA
#ifndef CONFIG_SLA_ALGO
struct op_game_app_info op_sla_game_app_list;
struct op_dev_info op_sla_info[IFACE_NUM];
int rtt_record_num;
int rtt_queue[MAX_RTT_RECORD_NUM];
int rtt_rear;
int game_rtt_wan_detect_flag;
int game_data[5];
int op_sla_enable;
int game_start_state;
int sla_switch_enable;
#else
extern struct op_game_app_info op_sla_game_app_list;
extern struct op_dev_info op_sla_info[IFACE_NUM];
extern int rtt_record_num;
extern int rtt_queue[MAX_RTT_RECORD_NUM];
extern int rtt_rear;
extern int game_rtt_wan_detect_flag;
extern int game_data[5];
extern int op_sla_enable;
extern int game_start_state;
extern int sla_switch_enable;

int is_ping_pong(int game_type, int time_now);
void op_rx_interval_error_estimator(int game_type,
				    int time_error);
void op_game_rtt_estimator(int *game_data);
int op_get_ct_cell_quality(int game_type);
int switch_to_cell(int cell_quality_good,
		   int game_rtt,
		   int gamelostcount,
		   int game_switch_interval,
		   int game_type);
int switch_to_wifi(int wlan_bad,
		   int game_rtt,
		   int gamelostcount,
		   int game_switch_interval,
		   int game_type);
void reset_sla_game_app_rx_error(int game_type);
void reset_sla_game_app_rtt(int game_type);
void record_sla_game_cell_state(int game_type,
				int game_switch_interval,
				int time_now);
void record_sla_game_wifi_state(int game_type,
				int game_switch_interval,
				int time_now);
int get_lost_count_threshold(int game_type);
int get_game_interval(int game_type, int game_interval);
int check_wan_detect_flag(int game_type);
int is_detect_game_lost(int game_lost_count,
			int game_lost_count_threshold,
			int game_time_interval);
int is_support_detect_game_tx(int game_type,
			      int special_rx_pkt_last_timestamp);
void get_rx_pkt_threshold(int game_type,
			  int time_now,
			  int special_rx_pkt_last_timestamp,
			  int *rtt_callback);
int data_stall_detect(int lastspecialrxtiming,
		      int specialrxthreshold,
		      int datastalltimer,
		      int datastallthreshold);
int get_game_tx_category(int game_type, int skb_len);
int get_game_rx_category(int game_type, unsigned int skb_len);
int drop_pkt_check(int game_type, int skb_len);
int is_support_rtt_wan_detect(int game_type);
int get_rx_interval_error(int game_category,
			  int time_now,
			  int rx_pkt_timestamp);
int is_need_check_game_rtt(int game_detect_status,
			   int game_timestamp,
			   int skb_len);
int get_game_rtt(int time_now, int game_timestamp, int game_type);
int is_skip_rx_rtt(int game_type, int game_time_interval);
int is_support_game_mark(int game_type);
int need_enable_sla(int cell_quality_good);
int need_enable_sla_for_wlan_score(int sla_screen_on);
void set_sla_game_parameter(int num);
void op_init_game_online_info(int num, int time_now);
int op_get_wlan_quality(void);
void update_wlan_score(void);
#endif
#endif
