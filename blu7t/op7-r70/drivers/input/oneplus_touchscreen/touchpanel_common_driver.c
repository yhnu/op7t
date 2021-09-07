#include <asm/uaccess.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/task_work.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/of_gpio.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/project_info.h>
#include <linux/time.h>
#include <linux/pm_wakeup.h>


#ifndef TPD_USE_EINT
#include <linux/hrtimer.h>
#endif

#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

#ifdef CONFIG_DRM_MSM
#include <linux/msm_drm_notify.h>
#endif

#include "touchpanel_common.h"
#include "util_interface/touch_interfaces.h"

/*******Part0:LOG TAG Declear************************/
#define TPD_PRINT_POINT_NUM 150
#define TPD_DEVICE "touchpanel"
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)
#define TPD_DEBUG(a, arg...)\
    do{\
        if (LEVEL_DEBUG == tp_debug)\
            pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

#define TPD_DETAIL(a, arg...)\
    do{\
        if (LEVEL_BASIC != tp_debug)\
            pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

#define TPD_SPECIFIC_PRINT(count, a, arg...)\
    do{\
        if (count++ == TPD_PRINT_POINT_NUM || LEVEL_DEBUG == tp_debug) {\
            TPD_INFO(TPD_DEVICE ": " a, ##arg);\
            count = 0;\
        }\
    }while(0)

/*******Part1:Global variables Area********************/
unsigned int tp_debug = 0;
unsigned int tp_register_times = 0;
//unsigned int probe_time = 0;
struct touchpanel_data *g_tp = NULL;
int tp_1v8_power = 0;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static struct input_dev *ps_input_dev = NULL;
static int lcd_id = 0;
static int gesture_switch_value = 0;


/* add haptic audio tp mask */
struct shake_point record_point[10];
/* add haptic audio tp mask end */

int sigle_num = 0;
struct timeval tpstart, tpend;
int pointx[2] = {0, 0};
int pointy[2] = {0, 0};
#define ABS(a,b) ((a - b > 0) ? a - b : b - a)

uint8_t DouTap_enable = 0;				 // double tap
uint8_t UpVee_enable  = 0;				 // V
uint8_t LeftVee_enable = 0; 			 // >
uint8_t RightVee_enable = 0;			 // <
uint8_t Circle_enable = 0;				 // O
uint8_t DouSwip_enable = 0; 			 // ||
uint8_t Mgestrue_enable = 0;			 // M
uint8_t Wgestrue_enable = 0;			 // W
uint8_t Sgestrue_enable = 0;			 // S
uint8_t SingleTap_enable = 0;			 // single tap
uint8_t Enable_gesture = 0;

/*******Part2:declear Area********************************/
static void speedup_resume(struct work_struct *work);
void esd_handle_switch(struct esd_information *esd_info, bool flag);

#ifdef TPD_USE_EINT
static irqreturn_t tp_irq_thread_fn(int irq, void *dev_id);
#endif

#if defined(CONFIG_FB) || defined(CONFIG_DRM_MSM)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif

static void tp_touch_release(struct touchpanel_data *ts);
static void tp_btnkey_release(struct touchpanel_data *ts);
static void tp_fw_update_work(struct work_struct *work);
static int init_debug_info_proc(struct touchpanel_data *ts);
static void tp_work_func(struct touchpanel_data *ts);
__attribute__((weak)) int request_firmware_select(const struct firmware **firmware_p, const char *name, struct device *device) {return 1;}
__attribute__((weak)) int register_devinfo(char *name, struct manufacture_info *info) {return 1;}
__attribute__((weak)) int preconfig_power_control(struct touchpanel_data *ts) {return 0;}
__attribute__((weak)) int reconfig_power_control(struct touchpanel_data *ts) {return 0;}

static int __init get_cmdlinelcd_id(char *str)
{
	TPD_INFO("%s enter %s\n", __func__, str);
	if (str) {
		if (strncmp(str, "normal", 6) == 0) {
			lcd_id = 0;
		} else if (strncmp(str, "x_talk", 6) == 0) {
			lcd_id = 1;
		} else if (strncmp(str, "tx", 2) == 0) {
			lcd_id = 1;
		}
	}
	return 0;
}
__setup("panel_type=", get_cmdlinelcd_id);


int check_touchirq_triggerd(void)
{
	int value = -1;

	if (!g_tp) {
		return 0;
	}
	if (!g_tp->gesture_enable) {
		return 0;
	}
	value = gpio_get_value(g_tp->hw_res.irq_gpio);
	if ((value == 0) && (g_tp->irq_flags & IRQF_TRIGGER_LOW)) {
		return 1;
	}

	return 0;
}

/*******Part3:Function  Area********************************/
/**
 * operate_mode_switch - switch work mode based on current params
 * @ts: touchpanel_data struct using for common driver
 *
 * switch work mode based on current params(gesture_enable, limit_enable, glove_enable)
 * Do not care the result: Return void type
 */
void operate_mode_switch(struct touchpanel_data *ts)
{
    if (!ts->ts_ops->mode_switch) {
        TPD_INFO("not support ts_ops->mode_switch callback\n");
        return;
    }

    if (ts->is_suspended) {
        if (ts->game_switch_support)
            ts->ts_ops->mode_switch(ts->chip_data, MODE_GAME, false);

        if (ts->black_gesture_support) {
            if (ts->gesture_enable == 1) {
                ts->ts_ops->mode_switch(ts->chip_data, MODE_GESTURE, true);
                if (ts->mode_switch_type == SEQUENCE)
                    ts->ts_ops->mode_switch(ts->chip_data, MODE_NORMAL, true);
            } else {
                ts->ts_ops->mode_switch(ts->chip_data, MODE_GESTURE, false);
                if (ts->mode_switch_type == SEQUENCE)
                    ts->ts_ops->mode_switch(ts->chip_data, MODE_SLEEP, true);
            }
        } else
            ts->ts_ops->mode_switch(ts->chip_data, MODE_SLEEP, true);
    } else {
        if (ts->ear_sense_support) {
            ts->ts_ops->mode_switch(ts->chip_data, MODE_EARSENSE, ts->es_enable == 1);
            ts->ts_ops->mode_switch(ts->chip_data, MODE_PALM_REJECTION, ts->palm_enable);
        }

        if (ts->face_detect_support) {
            //ts->ts_ops->mode_switch(ts->chip_data, MODE_FACE_DETECT, ts->fd_enable == 1);
        }

        if (ts->mode_switch_type == SEQUENCE) {
            if (ts->black_gesture_support)
                ts->ts_ops->mode_switch(ts->chip_data, MODE_GESTURE, false);
        }
        if (ts->edge_limit_support)
            ts->ts_ops->mode_switch(ts->chip_data, MODE_EDGE, ts->limit_edge);

        if (ts->glove_mode_support)
            ts->ts_ops->mode_switch(ts->chip_data, MODE_GLOVE, ts->glove_enable);

		if (ts->charge_detect_support)
			ts->ts_ops->mode_switch(ts->chip_data, MODE_CHARGE, ts->charge_detect);

		if (ts->touch_hold_support)
			ts->ts_ops->mode_switch(ts->chip_data, MODE_TOUCH_HOLD, ts->touch_hold_enable);

        ts->ts_ops->mode_switch(ts->chip_data, MODE_NORMAL, true);
    }
}

static void tp_touch_down(struct touchpanel_data *ts, struct point_info points, int touch_report_num, int id)
{
    static int last_width_major;
    static int point_num = 0;

    if (ts->input_dev == NULL)
        return;

    input_report_key(ts->input_dev, BTN_TOUCH, 1);
    input_report_key(ts->input_dev, BTN_TOOL_FINGER, 1);
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
    if (ts->boot_mode == RECOVERY_BOOT)
#else
    if (ts->boot_mode == MSM_BOOT_MODE__RECOVERY)
#endif
    {
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, points.z);
    } else {
        if (touch_report_num == 1) {
            input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, points.width_major);
            last_width_major = points.width_major;
        } else if(!(touch_report_num & 0x7f) || touch_report_num == 30) {	//avoid same point info getevent cannot report
            //if touch_report_num == 127, every 127 points, change width_major
            //down and keep long time, auto repeat per 5 seconds, for weixing
            //report move event after down event, for weixing voice delay problem, 30 -> 300ms in order to avoid the intercept by shortcut
            if (last_width_major == points.width_major) {
                last_width_major = points.width_major + 1;
            } else {
                last_width_major = points.width_major;
            }
            input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, last_width_major);
        }
        if (ts->smart_gesture_support && (points.touch_major > 0x12) &&\
            (points.x > ts->touch_major_limit.width_range) && (points.x < ts->resolution_info.max_x - ts->touch_major_limit.width_range) &&\
            (points.y > ts->touch_major_limit.height_range) && (points.y < ts->resolution_info.max_y - ts->touch_major_limit.height_range)) {
            input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, points.touch_major);
        }
        if(!CHK_BIT(ts->irq_slot, (1<<id))) {
            TPD_DETAIL("first touch point id %d [%4d %4d %4d]\n", id, points.x, points.y, points.z);
        }
    }

    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, points.x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, points.y);

    TPD_SPECIFIC_PRINT(point_num, "Touchpanel id %d :Down[%4d %4d %4d]\n", id, points.x, points.y, points.z);

#ifndef TYPE_B_PROTOCOL
    input_mt_sync(ts->input_dev);
#endif
}

static void tp_touch_up(struct touchpanel_data *ts)
{
    if (ts->input_dev == NULL)
        return;

    input_report_key(ts->input_dev, BTN_TOUCH, 0);
    input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
    input_mt_sync(ts->input_dev);
#endif
}

static void tp_exception_handle(struct touchpanel_data *ts)
{
    if (!ts->ts_ops->reset) {
        TPD_INFO("not support ts->ts_ops->reset callback\n");
        return;
    }

    ts->ts_ops->reset(ts->chip_data);    // after reset, all registers set to default
    operate_mode_switch(ts);

    tp_btnkey_release(ts);
    tp_touch_release(ts);
}

static void tp_fw_auto_reset_handle(struct touchpanel_data *ts)
{
    TPD_INFO("%s\n", __func__);

    if(ts->ts_ops->write_ps_status) {
        ts->ts_ops->write_ps_status(ts->chip_data, ts->ps_status);

        if (!ts->ps_status) {
            if (ts->ts_ops->exit_esd_mode) {
                ts->ts_ops->exit_esd_mode(ts->chip_data);
            }
        }
    }

    operate_mode_switch(ts);

    tp_btnkey_release(ts);
    tp_touch_release(ts);
}

static void tp_geture_info_transform(struct gesture_info * gesture, struct resolution_info *resolution_info)
{
    gesture->Point_start.x = gesture->Point_start.x * resolution_info->LCD_WIDTH  / (resolution_info->max_x);
    gesture->Point_start.y = gesture->Point_start.y * resolution_info->LCD_HEIGHT / (resolution_info->max_y);
    gesture->Point_end.x   = gesture->Point_end.x   * resolution_info->LCD_WIDTH  / (resolution_info->max_x);
    gesture->Point_end.y   = gesture->Point_end.y   * resolution_info->LCD_HEIGHT / (resolution_info->max_y);
    gesture->Point_1st.x   = gesture->Point_1st.x   * resolution_info->LCD_WIDTH  / (resolution_info->max_x);
    gesture->Point_1st.y   = gesture->Point_1st.y   * resolution_info->LCD_HEIGHT / (resolution_info->max_y);
    gesture->Point_2nd.x   = gesture->Point_2nd.x   * resolution_info->LCD_WIDTH  / (resolution_info->max_x);
    gesture->Point_2nd.y   = gesture->Point_2nd.y   * resolution_info->LCD_HEIGHT / (resolution_info->max_y);
    gesture->Point_3rd.x   = gesture->Point_3rd.x   * resolution_info->LCD_WIDTH  / (resolution_info->max_x);
    gesture->Point_3rd.y   = gesture->Point_3rd.y   * resolution_info->LCD_HEIGHT / (resolution_info->max_y);
    gesture->Point_4th.x   = gesture->Point_4th.x   * resolution_info->LCD_WIDTH  / (resolution_info->max_x);
    gesture->Point_4th.y   = gesture->Point_4th.y   * resolution_info->LCD_HEIGHT / (resolution_info->max_y);
}

int sec_double_tap(struct gesture_info *gesture)
{
	uint32_t timeuse = 0;

	if (sigle_num == 0) {
		do_gettimeofday(&tpstart);
		pointx[0] = gesture->Point_start.x;
		pointy[0] = gesture->Point_start.y;
		sigle_num ++;
		TPD_DEBUG("first enter double tap\n");
	} else if (sigle_num == 1) {
		do_gettimeofday(&tpend);
		pointx[1] = gesture->Point_start.x;
		pointy[1] = gesture->Point_start.y;
		sigle_num = 0;
		timeuse = 1000000 * (tpend.tv_sec-tpstart.tv_sec)
				+ tpend.tv_usec-tpstart.tv_usec;
		TPD_DEBUG("timeuse = %d, distance[x] = %d, distance[y] = %d\n", timeuse, ABS(pointx[0], pointx[1]), ABS(pointy[0], pointy[1]));
		if((ABS(pointx[0], pointx[1]) < 150) && (ABS(pointy[0], pointy[1]) < 200) && (timeuse < 500000)){
			return 1;
		}else {
			TPD_DEBUG("not match double tap\n");
			do_gettimeofday(&tpstart);
			pointx[0] = gesture->Point_start.x;
			pointy[0] = gesture->Point_start.y;
			sigle_num = 1;
		}
	}
	return 0;

}

static void tp_gesture_handle(struct touchpanel_data *ts)
{
    struct gesture_info gesture_info_temp;

    if (!ts->ts_ops->get_gesture_info) {
        TPD_INFO("not support ts->ts_ops->get_gesture_info callback\n");
        return;
    }

    memset(&gesture_info_temp, 0, sizeof(struct gesture_info));
    ts->ts_ops->get_gesture_info(ts->chip_data, &gesture_info_temp);
    tp_geture_info_transform(&gesture_info_temp, &ts->resolution_info);
	if (DouTap_enable) {
		if(gesture_info_temp.gesture_type == SingleTap) {
			if (sec_double_tap(&gesture_info_temp) == 1)
			{
				gesture_info_temp.gesture_type  = DouTap;
			}
		}
	}
    TPD_INFO("detect %s gesture\n", gesture_info_temp.gesture_type == DouTap ? "double tap" :
            gesture_info_temp.gesture_type == UpVee ? "up vee" :
            gesture_info_temp.gesture_type == DownVee ? "down vee" :
            gesture_info_temp.gesture_type == LeftVee ? "(>)" :
            gesture_info_temp.gesture_type == RightVee ? "(<)" :
            gesture_info_temp.gesture_type == Circle ? "o" :
            gesture_info_temp.gesture_type == DouSwip ? "(||)" :
            gesture_info_temp.gesture_type == Left2RightSwip ? "(-->)" :
            gesture_info_temp.gesture_type == Right2LeftSwip ? "(<--)" :
            gesture_info_temp.gesture_type == Up2DownSwip ? "up to down |" :
            gesture_info_temp.gesture_type == Down2UpSwip ? "down to up |" :
            gesture_info_temp.gesture_type == Mgestrue ? "(M)" :
            gesture_info_temp.gesture_type == Sgestrue ? "(S)" :
            gesture_info_temp.gesture_type == SingleTap ? "(single tap)" :
            gesture_info_temp.gesture_type == Wgestrue ? "(W)" : "unknown");

	if ((gesture_info_temp.gesture_type == DouTap && DouTap_enable) ||
		(gesture_info_temp.gesture_type == UpVee && UpVee_enable) ||
		(gesture_info_temp.gesture_type ==  LeftVee&& LeftVee_enable) ||
		(gesture_info_temp.gesture_type == RightVee && RightVee_enable) ||
		(gesture_info_temp.gesture_type == Circle && Circle_enable) ||
		(gesture_info_temp.gesture_type == DouSwip && DouSwip_enable) ||
		(gesture_info_temp.gesture_type == Mgestrue && Mgestrue_enable) ||
		(gesture_info_temp.gesture_type == Sgestrue && Sgestrue_enable) ||
		(gesture_info_temp.gesture_type == SingleTap && SingleTap_enable) ||
		(gesture_info_temp.gesture_type == Wgestrue && Wgestrue_enable)) {
        memcpy(&ts->gesture, &gesture_info_temp, sizeof(struct gesture_info));
        input_report_key(ts->input_dev, KEY_F4, 1);
        input_sync(ts->input_dev);
        input_report_key(ts->input_dev, KEY_F4, 0);
        input_sync(ts->input_dev);
    }
}

void tp_touch_btnkey_release(void)
{
    struct touchpanel_data *ts = g_tp;

    if (!ts) {
        TPD_INFO("ts is NULL\n");
        return ;
    }

    tp_touch_release(ts);
    tp_btnkey_release(ts);
}

static void tp_touch_release(struct touchpanel_data *ts)
{
    int i = 0;

#ifdef TYPE_B_PROTOCOL
    for (i = 0; i < ts->max_num; i++) {
        input_mt_slot(ts->input_dev, i);
        input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
    }
    input_report_key(ts->input_dev, BTN_TOUCH, 0);
    input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
    input_sync(ts->input_dev);
#else
    input_report_key(ts->input_dev, BTN_TOUCH, 0);
    input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
    input_mt_sync(ts->input_dev);
    input_sync(ts->input_dev);
#endif
    TPD_INFO("release all touch point and key, clear tp touch down flag\n");
    ts->view_area_touched = 0; //realse all touch point,must clear this flag
    ts->touch_count = 0;
    ts->irq_slot = 0;
    ts->corner_delay_up = -1;
}

static bool edge_point_process(struct touchpanel_data *ts, struct point_info points)
{
    if (ts->limit_edge) {
        if (points.x > ts->edge_limit.left_x2 && points.x < ts->edge_limit.right_x2) {
            if (ts->edge_limit.in_which_area == AREA_EDGE)
                tp_touch_release(ts);
            ts->edge_limit.in_which_area = AREA_NORMAL;
        } else if ((points.x > ts->edge_limit.left_x1 && points.x < ts->edge_limit.left_x2) || (points.x >ts->edge_limit.right_x2 && points.x < ts->edge_limit.right_x1)) {//area2
            if (ts->edge_limit.in_which_area == AREA_EDGE) {
                ts->edge_limit.in_which_area = AREA_CRITICAL;
            }
        } else if (points.x < ts->edge_limit.left_x1 || points.x > ts->edge_limit.right_x1) {        //area 1
            if (ts->edge_limit.in_which_area == AREA_CRITICAL) {
                ts->edge_limit.in_which_area = AREA_EDGE;
                return true;
            }
            if (ts->edge_limit.in_which_area ==  AREA_NORMAL)
                return true;

            ts->edge_limit.in_which_area = AREA_EDGE;
        }
    }

    return false;
}

static bool corner_point_process(struct touchpanel_data *ts, struct corner_info *corner, struct point_info *points, int i)
{
    int j;
    if (ts->limit_corner) {
        if ((ts->limit_corner & (1 << CORNER_TOPLEFT)) && (points[i].x < ts->edge_limit.left_x3 && points[i].y < ts->edge_limit.left_y1)) {
            points[i].type  = AREA_CORNER;
            if (ts->edge_limit.in_which_area == AREA_NORMAL)
                return true;

            corner[CORNER_TOPLEFT].id = i;
            corner[CORNER_TOPLEFT].point = points[i];
            corner[CORNER_TOPLEFT].flag = true;

            ts->edge_limit.in_which_area = points[i].type;
        }
        if ((ts->limit_corner & (1 << CORNER_TOPRIGHT))  && (points[i].x < ts->edge_limit.left_x3 && points[i].y > ts->edge_limit.right_y1)) {
            points[i].type  = AREA_CORNER;
            if (ts->edge_limit.in_which_area == AREA_NORMAL)
                return true;

            corner[CORNER_TOPRIGHT].id = i;
            corner[CORNER_TOPRIGHT].point = points[i];
            corner[CORNER_TOPRIGHT].flag = true;

            ts->edge_limit.in_which_area = points[i].type;
        }
        if ((ts->limit_corner & (1 << CORNER_BOTTOMLEFT))  && (points[i].x > ts->edge_limit.right_x3 && points[i].y < ts->edge_limit.left_y1)) {
            points[i].type  = AREA_CORNER;
            if (ts->edge_limit.in_which_area == AREA_NORMAL)
                return true;

            corner[CORNER_BOTTOMLEFT].id = i;
            corner[CORNER_BOTTOMLEFT].point = points[i];
            corner[CORNER_BOTTOMLEFT].flag = true;

            ts->edge_limit.in_which_area = points[i].type;
        }
        if ((ts->limit_corner & (1 << CORNER_BOTTOMRIGHT))  && (points[i].x > ts->edge_limit.right_x3 && points[i].y > ts->edge_limit.right_y1)) {
            points[i].type  = AREA_CORNER;
            if (ts->edge_limit.in_which_area == AREA_NORMAL)
                return true;

            corner[CORNER_BOTTOMRIGHT].id = i;
            corner[CORNER_BOTTOMRIGHT].point = points[i];
            corner[CORNER_BOTTOMRIGHT].flag = true;

            ts->edge_limit.in_which_area = points[i].type;
        }

        if (points[i].type != AREA_CORNER) {
            if (ts->edge_limit.in_which_area == AREA_CORNER) {
                if(ts->corner_delay_up == 0) {
                        for (j = 0; j < 4; j++) {
                            if (corner[j].flag) {
#ifdef TYPE_B_PROTOCOL
                                input_mt_slot(ts->input_dev, corner[j].id);
                                input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
#endif
                            }
                        }
                        ts->corner_delay_up = -1;
                } else {
                        ts->corner_delay_up = 1;
                }
            }
            if(ts->corner_delay_up == -1) {
                points[i].type = AREA_NORMAL;
                ts->edge_limit.in_which_area = points[i].type;
            }
        }

    }

    return false;
}

static void tp_touch_handle(struct touchpanel_data *ts)
{
    int i = 0;
    uint8_t finger_num = 0, touch_near_edge = 0;
    int obj_attention = 0;
    struct point_info *points;
    struct corner_info corner[4];
    static struct point_info last_point = {.x = 0, .y = 0};
    static int touch_report_num = 0;
    struct msm_drm_notifier notifier_data;
    /* add haptic audio tp mask */
    int bank;
    int record_flag[10] = {0};
    /* add haptic audio tp mask end */

    if (!ts->ts_ops->get_touch_points) {
        TPD_INFO("not support ts->ts_ops->get_touch_points callback\n");
        return;
    }

	points= kzalloc(sizeof(struct point_info)*ts->max_num, GFP_KERNEL);
	if (!points) {
		TPD_INFO("points kzalloc failed\n");
		return;
	}
    memset(corner, 0, sizeof(corner));
	if (ts->reject_point) {		//sensor will reject point when call mode.
		if (ts->touch_count) {
			#ifdef TYPE_B_PROTOCOL
				for (i = 0; i < ts->max_num; i++) {
					input_mt_slot(ts->input_dev, i);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
				}
			#endif
				input_report_key(ts->input_dev, BTN_TOUCH, 0);
				input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
			#ifndef TYPE_B_PROTOCOL
				input_mt_sync(ts->input_dev);
			#endif
				input_sync(ts->input_dev);
		}
		kfree(points);
		return;
	}
    obj_attention = ts->ts_ops->get_touch_points(ts->chip_data, points, ts->max_num);
    if ((obj_attention & TOUCH_BIT_CHECK) != 0) {
        for (i = 0; i < ts->max_num; i++) {
            if (((obj_attention & TOUCH_BIT_CHECK) >> i) & 0x01 && (points[i].status == 0)) // buf[0] == 0 is wrong point, no process
                continue;
            if (((obj_attention & TOUCH_BIT_CHECK) >> i) & 0x01 && (points[i].status != 0)) {
                //Edge process before report abs
                if (ts->edge_limit_support) {
                    if (ts->corner_delay_up < 1 && corner_point_process(ts, corner, points, i))
                        continue;
                    if (edge_point_process(ts, points[i]))
                        continue;
                }
#ifdef TYPE_B_PROTOCOL
                input_mt_slot(ts->input_dev, i);
                input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
#endif
                touch_report_num++;
                tp_touch_down(ts, points[i], touch_report_num, i);
                /* add haptic audio tp mask */
                /*bank = points[i].status;*/
                bank = i;
                notifier_data.data = &bank;
                record_point[i].status = 1;
                record_point[i].x = points[i].x;
                record_point[i].y = points[i].y;
                record_flag[i] = 1;
                msm_drm_notifier_call_chain(11, &notifier_data);	//down;
                /* add haptic audio tp mask end */
                SET_BIT(ts->irq_slot, (1<<i));
                finger_num++;
                if (points[i].x > ts->resolution_info.max_x / 100 && points[i].x < ts->resolution_info.max_x * 99 / 100) {
                    ts->view_area_touched = finger_num;
                } else {
                    touch_near_edge++;
                }
                /*strore  the last point data*/
                memcpy(&last_point, &points[i], sizeof(struct point_info));
            }
#ifdef TYPE_B_PROTOCOL
            else {
                input_mt_slot(ts->input_dev, i);
                input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
                bank = i;
                notifier_data.data = &bank;
                record_point[i].status = 0;
                msm_drm_notifier_call_chain(10, &notifier_data);    //up;
                record_flag[i] = 0;
                /* add haptic audio tp mak */
            }
#endif
        }

        if(ts->corner_delay_up > -1) {
                TPD_DETAIL("corner_delay_up is %d\n", ts->corner_delay_up);
        }
        ts->corner_delay_up = ts->corner_delay_up > 0 ? ts->corner_delay_up - 1 : ts->corner_delay_up;
        if (touch_near_edge == finger_num) {        //means all the touchpoint is near the edge
            ts->view_area_touched = 0;
        }
        if(ts->ear_sense_support && ts->es_enable && (finger_num > ts->touch_count)) {
            ts->delta_state = TYPE_DELTA_BUSY;
            queue_work(ts->delta_read_wq, &ts->read_delta_work);
        }
    } else {
        finger_num = 0;
        touch_report_num = 0;
#ifdef TYPE_B_PROTOCOL
        for (i = 0; i < ts->max_num; i++) {
            input_mt_slot(ts->input_dev, i);
            input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
            /* add haptic audio tp mask */
            bank = i;
            notifier_data.data = &bank;
            record_point[i].status = 0;
            msm_drm_notifier_call_chain(0, &notifier_data);
            /* add haptic audio tp mask end */
        }
#endif
        tp_touch_up(ts);
        ts->view_area_touched = 0;
        ts->irq_slot = 0;
        ts->corner_delay_up = -1;
        TPD_DETAIL("all touch up,view_area_touched=%d finger_num=%d\n",ts->view_area_touched, finger_num);
        TPD_DETAIL("last point x:%d y:%d\n", last_point.x, last_point.y);
        if (ts->edge_limit_support)
            ts->edge_limit.in_which_area = AREA_NOTOUCH;
    }
    input_sync(ts->input_dev);
    ts->touch_count = finger_num;
	kfree(points);
}

static void tp_btnkey_release(struct touchpanel_data *ts)
{
    if (CHK_BIT(ts->vk_bitmap, BIT_MENU))
        input_report_key_reduce(ts->kpd_input_dev, KEY_MENU, 0);
    if (CHK_BIT(ts->vk_bitmap, BIT_HOME))
        input_report_key_reduce(ts->kpd_input_dev, KEY_HOMEPAGE, 0);
    if (CHK_BIT(ts->vk_bitmap, BIT_BACK))
        input_report_key_reduce(ts->kpd_input_dev, KEY_BACK, 0);
    input_sync(ts->kpd_input_dev);
}

static void tp_btnkey_handle(struct touchpanel_data *ts)
{
    u8 touch_state = 0;

    if (ts->vk_type != TYPE_AREA_SEPRATE) {
        TPD_DEBUG("TP vk_type not proper, checktouchpanel, button-type\n");

        return;
    }
    if (!ts->ts_ops->get_keycode) {
        TPD_INFO("not support ts->ts_ops->get_keycode callback\n");

        return;
    }
    touch_state = ts->ts_ops->get_keycode(ts->chip_data);

    if (CHK_BIT(ts->vk_bitmap, BIT_MENU))
        input_report_key_reduce(ts->kpd_input_dev, KEY_MENU, CHK_BIT(touch_state, BIT_MENU));
    if (CHK_BIT(ts->vk_bitmap, BIT_HOME))
        input_report_key_reduce(ts->kpd_input_dev, KEY_HOMEPAGE, CHK_BIT(touch_state, BIT_HOME));
    if (CHK_BIT(ts->vk_bitmap, BIT_BACK))
        input_report_key_reduce(ts->kpd_input_dev, KEY_BACK, CHK_BIT(touch_state, BIT_BACK));
    input_sync(ts->kpd_input_dev);
}

static void tp_config_handle(struct touchpanel_data *ts)
{
    int ret = 0;
    if (!ts->ts_ops->fw_handle) {
        TPD_INFO("not support ts->ts_ops->fw_handle callback\n");
        return;
    }

    ret = ts->ts_ops->fw_handle(ts->chip_data);
}

static void tp_datalogger_handle(struct touchpanel_data *ts)
{
    if (!ts->ts_ops->data_logger_get) {
        TPD_INFO("not support ts->ts_ops->data_logger_get callback\n");
        return;
    }

    ts->ts_ops->data_logger_get(ts->chip_data);
}

static void tp_face_detect_handle(struct touchpanel_data *ts)
{
    int ps_state = 0;

    if (!ts->ts_ops->get_face_state) {
        TPD_INFO("not support ts->ts_ops->get_face_state callback\n");
        return;
    }
	TPD_INFO("enter tp_face_detect_handle\n");
    ps_state = ts->ts_ops->get_face_state(ts->chip_data);
    TPD_DETAIL("ps state: %s\n", ps_state > 0 ? "near" : "far");

    input_event(ps_input_dev, EV_MSC, MSC_RAW, ps_state > 0);
	input_sync(ps_input_dev);
}

static void tp_async_work_callback(void)
{
    struct touchpanel_data *ts = g_tp;

    if (ts == NULL)
        return;

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
    if ((ts->boot_mode == META_BOOT || ts->boot_mode == FACTORY_BOOT))
#else
    if ((ts->boot_mode == MSM_BOOT_MODE__FACTORY || ts->boot_mode == MSM_BOOT_MODE__RF || ts->boot_mode == MSM_BOOT_MODE__WLAN))
#endif
    {
        TPD_INFO("%s: in ftm mode, no need to call back\n", __func__);
        return;
    }

    TPD_INFO("%s: async work\n", __func__);
    if (ts->use_resume_notify && ts->suspend_state == TP_RESUME_COMPLETE) {
        complete(&ts->resume_complete);
        return;
    }

    if (ts->in_test_process) {
        TPD_INFO("%s: In test process, do not switch mode\n", __func__);
        return;
    }

    queue_work(ts->async_workqueue, &ts->async_work);
}

static void tp_async_work_lock(struct work_struct *work)
{
    struct touchpanel_data *ts = container_of(work, struct touchpanel_data, async_work);
    mutex_lock(&ts->mutex);
    if (ts->ts_ops->async_work) {
        ts->ts_ops->async_work(ts->chip_data);
    }
    mutex_unlock(&ts->mutex);
}

static void tp_work_common_callback(void)
{
    struct touchpanel_data *ts;

    if (g_tp == NULL)
        return;
    ts = g_tp;
    tp_work_func(ts);
}

static void tp_work_func(struct touchpanel_data *ts)
{
    u8 cur_event = 0;

    if (!ts->ts_ops->trigger_reason) {
        TPD_INFO("not support ts_ops->trigger_reason callback\n");
        return;
    }
    /*
     *  trigger_reason:this callback determine which trigger reason should be
     *  The value returned has some policy!
     *  1.IRQ_EXCEPTION /IRQ_GESTURE /IRQ_IGNORE /IRQ_FW_CONFIG --->should be only reported  individually
     *  2.IRQ_TOUCH && IRQ_BTN_KEY --->should depends on real situation && set correspond bit on trigger_reason
     */
    cur_event = ts->ts_ops->trigger_reason(ts->chip_data, ts->gesture_enable, ts->is_suspended);
    if (CHK_BIT(cur_event, IRQ_TOUCH) || CHK_BIT(cur_event, IRQ_BTN_KEY) || CHK_BIT(cur_event, IRQ_DATA_LOGGER) || CHK_BIT(cur_event, IRQ_FACE_STATE)) {
        if (CHK_BIT(cur_event, IRQ_BTN_KEY)) {
            tp_btnkey_handle(ts);
        }
        if (CHK_BIT(cur_event, IRQ_TOUCH)) {
            tp_touch_handle(ts);
        }
        if (CHK_BIT(cur_event, IRQ_DATA_LOGGER)) {
            tp_datalogger_handle(ts);
        }
        if (CHK_BIT(cur_event, IRQ_FACE_STATE) && ts->fd_enable) {
            tp_face_detect_handle(ts);
		}
    } else if (CHK_BIT(cur_event, IRQ_GESTURE)) {
        tp_gesture_handle(ts);
    } else if (CHK_BIT(cur_event, IRQ_EXCEPTION)) {
        tp_exception_handle(ts);
    } else if (CHK_BIT(cur_event, IRQ_FW_CONFIG)) {
        tp_config_handle(ts);
    }  else if (CHK_BIT(cur_event, IRQ_FW_AUTO_RESET)) {
        tp_fw_auto_reset_handle(ts);
    } else {
        TPD_DEBUG("unknown irq trigger reason\n");
    }
}

static void tp_work_func_unlock(struct touchpanel_data *ts)
{
    if (ts->ts_ops->irq_handle_unlock) {
        ts->ts_ops->irq_handle_unlock(ts->chip_data);
    }
}

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
extern void primary_display_esd_check_enable(int enable);
#endif
void __attribute__((weak)) display_esd_check_enable_bytouchpanel(bool enable) {return;}

static void tp_fw_update_work(struct work_struct *work)
{
    const struct firmware *fw = NULL;
    int ret, fw_update_result = 0;
    int count_tmp = 0, retry = 5;
    char *p_node = NULL;
    char *fw_name_fae = NULL;
    char *postfix = "_FAE";
    uint8_t copy_len = 0;

    struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
            fw_update_work);

    if (!ts->ts_ops->fw_check || !ts->ts_ops->reset) {
        TPD_INFO("not support ts_ops->fw_check callback\n");
        complete(&ts->fw_complete);
        return;
    }

    TPD_INFO("%s: fw_name = %s\n", __func__, ts->panel_data.fw_name);

    mutex_lock(&ts->mutex);
    if (ts->int_mode == BANNABLE) {
        disable_irq_nosync(ts->irq);
    }
    ts->loading_fw = true;

    if (ts->esd_handle_support) {
        esd_handle_switch(&ts->esd_info, false);
    }

    display_esd_check_enable_bytouchpanel(0);
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
    primary_display_esd_check_enable(0); //avoid rst pulled to low while updating
#endif

    if (ts->ts_ops->fw_update) {
        do {
            if(ts->firmware_update_type == 0 || ts->firmware_update_type == 1) {
                if(ts->fw_update_app_support) {
                        fw_name_fae = kzalloc(MAX_FW_NAME_LENGTH, GFP_KERNEL);
                        if(fw_name_fae == NULL) {
                                TPD_INFO("fw_name_fae kzalloc error!\n");
                                goto EXIT;
                        }
                        p_node  = strstr(ts->panel_data.fw_name, ".");
                        copy_len = p_node - ts->panel_data.fw_name;
                        memcpy(fw_name_fae, ts->panel_data.fw_name, copy_len);
                        strlcat(fw_name_fae, postfix, MAX_FW_NAME_LENGTH);
                        strlcat(fw_name_fae, p_node, MAX_FW_NAME_LENGTH);
                        TPD_INFO("fw_name_fae is %s\n", fw_name_fae);
                        ret = request_firmware(&fw, fw_name_fae, ts->dev);
                        if (!ret)
                                break;
                } else {
                        ret = request_firmware(&fw, ts->panel_data.fw_name, ts->dev);
                        if (!ret)
                                break;
                }
            } else {
                ret = request_firmware_select(&fw, ts->panel_data.fw_name, ts->dev);
                if (!ret)
                        break;
            }
        } while((ret < 0) && (--retry > 0));

        TPD_INFO("retry times %d\n", 5 - retry);

        if (!ret || ts->is_noflash_ic) {
            do {
                count_tmp++;
                ret = ts->ts_ops->fw_update(ts->chip_data, fw, ts->force_update);
                fw_update_result = ret;
                if (ret == FW_NO_NEED_UPDATE) {
                    break;
                }

                if(!ts->is_noflash_ic) {        //noflash update fw in reset and do bootloader reset in get_chip_info
                        ret |= ts->ts_ops->reset(ts->chip_data);
                        ret |= ts->ts_ops->get_chip_info(ts->chip_data);
                }

                ret |= ts->ts_ops->fw_check(ts->chip_data, &ts->resolution_info, &ts->panel_data);
            } while((count_tmp < 2) && (ret != 0));

            if(fw != NULL) {
                release_firmware(fw);
            }
        } else {
            TPD_INFO("%s: fw_name request failed %s %d\n", __func__, ts->panel_data.fw_name, ret);
            goto EXIT;
        }
    }

    tp_touch_release(ts);
    tp_btnkey_release(ts);
    operate_mode_switch(ts);
    if (fw_update_result != FW_NO_NEED_UPDATE) {
        if (ts->spurious_fp_support && ts->ts_ops->finger_proctect_data_get) {
            ts->ts_ops->finger_proctect_data_get(ts->chip_data);
        }
        if (ts->ts_ops->data_logger_open) {
            ts->ts_ops->data_logger_open(ts->chip_data);
        }
    }

EXIT:
    ts->loading_fw = false;

    display_esd_check_enable_bytouchpanel(1);
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
    primary_display_esd_check_enable(1); //avoid rst pulled to low while updating
#endif

    if (ts->esd_handle_support) {
        esd_handle_switch(&ts->esd_info, true);
    }

    kfree(fw_name_fae);
    fw_name_fae = NULL;
    if (ts->int_mode == BANNABLE) {
        enable_irq(ts->irq);
    }
    mutex_unlock(&ts->mutex);

    ts->force_update = 0;

    complete(&ts->fw_complete); //notify to init.rc that fw update finished
    return;
}

#ifndef TPD_USE_EINT
static enum hrtimer_restart touchpanel_timer_func(struct hrtimer *timer)
{
    struct touchpanel_data *ts = container_of(timer, struct touchpanel_data, timer);

    mutex_lock(&ts->mutex);
    tp_work_func(ts);
    mutex_unlock(&ts->mutex);
    hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);

    return HRTIMER_NORESTART;
}
#else
static irqreturn_t tp_irq_thread_fn(int irq, void *dev_id)
{
    struct touchpanel_data *ts = (struct touchpanel_data *)dev_id;

    if (ts->int_mode == BANNABLE) {
		__pm_stay_awake(&ts->source);	//avoid system enter suspend lead to i2c error
        mutex_lock(&ts->mutex);
        tp_work_func(ts);
        mutex_unlock(&ts->mutex);
		__pm_relax(&ts->source);
    } else {
        tp_work_func_unlock(ts);
    }
    return IRQ_HANDLED;
}
#endif

/**
 * tp_gesture_enable_flag -   expose gesture control status for other module.
 * Return gesture_enable status.
 */
int tp_gesture_enable_flag(void)
{
    if (!g_tp || !g_tp->is_incell_panel)
        return LCD_POWER_OFF;

    return (g_tp->gesture_enable > 0)?LCD_POWER_ON: LCD_POWER_OFF;
}

/*
*Interface for lcd to control reset pin
*/
int tp_control_reset_gpio(bool enable)
{
    if (!g_tp) {
        return 0;
    }

    if (gpio_is_valid(g_tp->hw_res.reset_gpio)) {
        if (g_tp->ts_ops->reset_gpio_control) {
            g_tp->ts_ops->reset_gpio_control(g_tp->chip_data, enable);
        }
    }

    return 0;
}

/*
 * check_usb_state----expose to be called by charger int to get usb state
 * @usb_state : 1 if usb checked, otherwise is 0
*/
void switch_usb_state(int usb_state)
{
    if (!g_tp) {
        return;
    }

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
    if ((g_tp->boot_mode == META_BOOT || g_tp->boot_mode == FACTORY_BOOT))
#else
    if ((g_tp->boot_mode == MSM_BOOT_MODE__FACTORY || g_tp->boot_mode == MSM_BOOT_MODE__RF || g_tp->boot_mode == MSM_BOOT_MODE__WLAN))
#endif
    {
        TPD_INFO("Ftm mode, do not switch usb state\n");
        return;
    }

    if (g_tp->charger_pump_support && (g_tp->is_usb_checked != usb_state)) {
        g_tp->is_usb_checked = !!usb_state;
        TPD_INFO("%s: check usb state : %d, is_suspended: %d\n", __func__, usb_state, g_tp->is_suspended);
        if (!g_tp->is_suspended && (g_tp->suspend_state == TP_SPEEDUP_RESUME_COMPLETE)) {
            mutex_lock(&g_tp->mutex);
            g_tp->ts_ops->mode_switch(g_tp->chip_data, MODE_CHARGE, g_tp->is_usb_checked);
            mutex_unlock(&g_tp->mutex);
        }
    }
}
EXPORT_SYMBOL(switch_usb_state);

/*
 *    gesture_enable = 0 : disable gesture
 *    gesture_enable = 1 : enable gesture when ps is far away
 *    gesture_enable = 2 : disable gesture when ps is near
 */
static ssize_t proc_gesture_control_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int value = 0;
    char buf[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (count > 2)
        return count;
    if (!ts)
        return count;

    if (copy_from_user(buf, buffer, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }
	TPD_DEBUG("%s write argc1[0x%x],argc2[0x%x]\n",__func__,buf[0],buf[1]);
	UpVee_enable = (buf[0] & BIT0)?1:0;
	DouSwip_enable = (buf[0] & BIT1)?1:0;
	LeftVee_enable = (buf[0] & BIT3)?1:0;
	RightVee_enable = (buf[0] & BIT4)?1:0;
	Circle_enable = (buf[0] & BIT6)?1:0;
	DouTap_enable = (buf[0] & BIT7)?1:0;
	Sgestrue_enable = (buf[1] & BIT0)?1:0;
	Mgestrue_enable	= (buf[1] & BIT1)?1:0;
	Wgestrue_enable = (buf[1] & BIT2)?1:0;
	SingleTap_enable = (buf[1] & BIT3)?1:0;
	Enable_gesture = (buf[1] & BIT7)?1:0;

	if (UpVee_enable || DouSwip_enable || LeftVee_enable || RightVee_enable
		|| Circle_enable || DouTap_enable || Sgestrue_enable || Mgestrue_enable
		|| Wgestrue_enable || SingleTap_enable || Enable_gesture) {
		value = 1;
	} else {
		value = 0;
	}

    mutex_lock(&ts->mutex);
    if (ts->gesture_enable != value) {
        ts->gesture_enable = value;
		tp_1v8_power = ts->gesture_enable;
        TPD_INFO("%s: gesture_enable = %d, is_suspended = %d\n", __func__, ts->gesture_enable, ts->is_suspended);
        if (ts->is_incell_panel && (ts->suspend_state == TP_RESUME_EARLY_EVENT) && (ts->tp_resume_order == LCD_TP_RESUME)) {
            TPD_INFO("tp will resume, no need mode_switch in incell panel\n"); /*avoid i2c error or tp rst pulled down in lcd resume*/
        } else if (ts->is_suspended)
            operate_mode_switch(ts);
    }else {
        TPD_INFO("%s: do not do same operator :%d\n", __func__, value);
    }
    mutex_unlock(&ts->mutex);

    return count;
}

static ssize_t proc_gesture_control_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    TPD_DEBUG("gesture_enable is: %d\n", ts->gesture_enable);
    ret = sprintf(page, "%d\n", ts->gesture_enable);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

static ssize_t proc_coordinate_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[PAGESIZE] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    TPD_DEBUG("%s:gesture_type = %d\n", __func__, ts->gesture.gesture_type);
    ret = sprintf(page, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", ts->gesture.gesture_type,
            ts->gesture.Point_start.x, ts->gesture.Point_start.y, ts->gesture.Point_end.x, ts->gesture.Point_end.y,
            ts->gesture.Point_1st.x,   ts->gesture.Point_1st.y,   ts->gesture.Point_2nd.x, ts->gesture.Point_2nd.y,
            ts->gesture.Point_3rd.x,   ts->gesture.Point_3rd.y,   ts->gesture.Point_4th.x, ts->gesture.Point_4th.y,
            ts->gesture.clockwise);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

static const struct file_operations proc_gesture_control_fops = {
    .write = proc_gesture_control_write,
    .read  = proc_gesture_control_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static const struct file_operations proc_coordinate_fops = {
    .read  = proc_coordinate_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_ps_status_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int value = 0;
    char buf[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));
    if (count > 2)
        return count;
    if (!ts)
        return count;

    if (!ts->ts_ops->write_ps_status) {
        TPD_INFO("not support ts_ops->write_ps_status callback\n");
        return count;
    }

    if (copy_from_user(buf, buffer, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }
    sscanf(buf, "%d", &value);
    if (value > 2)
        return count;

    if (!ts->is_suspended && (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE)) {
        mutex_lock(&ts->mutex);
        ts->ps_status = value;
        ts->ts_ops->write_ps_status(ts->chip_data, value);
        mutex_unlock(&ts->mutex);
    }

    return count;
}

static ssize_t proc_ps_support_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts) {
        sprintf(page, "%d\n", -1);//no support
    } else if (!ts->ts_ops->write_ps_status) {
        sprintf(page, "%d\n", -1);
    } else {
        sprintf(page, "%d\n", 0);//support
    }
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}

static const struct file_operations proc_write_ps_status_fops = {
    .write = proc_ps_status_write,
    .read  = proc_ps_support_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

//proc/touchpanel/game_switch_enable
static ssize_t proc_game_switch_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int value = 0 ;
    char buf[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (count > 4) {
        TPD_INFO("%s:count > 4\n",__func__);
        return count;
    }

    if (!ts) {
        TPD_INFO("%s: ts is NULL\n",__func__);
        return count;
    }

    if (!ts->ts_ops->mode_switch) {
        TPD_INFO("%s:not support ts_ops->mode_switch callback\n",__func__);
        return count;
    }
    if (copy_from_user(buf, buffer, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }
    sscanf(buf, "%x", &value);
    ts->noise_level = value;

    TPD_INFO("%s: game_switch value=0x%x\n", __func__, value);
    if (!ts->is_suspended) {
        mutex_lock(&ts->mutex);
        ts->ts_ops->mode_switch(ts->chip_data, MODE_GAME, value > 0);
        mutex_unlock(&ts->mutex);
    } else {
        TPD_INFO("%s: game_switch_support is_suspended.\n", __func__);
    }

    return count;
}

static ssize_t proc_game_switch_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts) {
        sprintf(page, "%d\n", -1);//no support
    } else {
        sprintf(page, "%d\n", ts->noise_level);//support
    }
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}

static const struct file_operations proc_game_switch_fops = {
    .write = proc_game_switch_write,
    .read  = proc_game_switch_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_gesture_switch_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int value = 0 ;
    char buf[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (count > 4) {
        TPD_INFO("%s:count > 4\n",__func__);
        return count;
    }

    if (!ts) {
        TPD_INFO("%s: ts is NULL\n",__func__);
        return count;
    }

    if (copy_from_user(buf, buffer, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }
    sscanf(buf, "%d", &value);
	if (gesture_switch_value == value) {
		//TPD_INFO("gesture_switch_value is %d\n", value);
		return count;
	}
	gesture_switch_value = value;
	value = value - 1;		//cmd 2 is disable gesture,cmd 1 is open gesture
    ts->gesture_switch = value;

    TPD_DEBUG("%s: gesture_switch value= %d\n", __func__, value);
    if ((ts->is_suspended == 1) && (ts->gesture_enable ==1)) {
		__pm_stay_awake(&ts->source);	//avoid system enter suspend lead to i2c error
        mutex_lock(&ts->mutex);
        ts->ts_ops->mode_switch(ts->chip_data, MODE_GESTURE_SWITCH, ts->gesture_switch);
		mutex_unlock(&ts->mutex);
		__pm_relax(&ts->source);
    } else {
        TPD_INFO("%s: gesture mode switch must be suspend.\n", __func__);
    }

    return count;
}

static ssize_t proc_gesture_switch_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts) {
        sprintf(page, "%d\n", -1);//no support
    } else {
        sprintf(page, "%d\n", ts->gesture_switch);//support
    }
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}


static const struct file_operations proc_gesture_switch_fops = {
    .write = proc_gesture_switch_write,
    .read  = proc_gesture_switch_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_reject_point_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int value = 0 ;
    char buf[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (count > 4) {
        TPD_INFO("%s:count > 4\n",__func__);
        return count;
    }

    if (!ts) {
        TPD_INFO("%s: ts is NULL\n",__func__);
        return count;
    }

    if (copy_from_user(buf, buffer, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }
    sscanf(buf, "%d", &value);
    ts->reject_point = value;

    TPD_INFO("%s: ts->reject_poin = %d\n", __func__, value);

    return count;
}

static ssize_t proc_reject_point_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts) {
        sprintf(page, "%d\n", -1);//no support
    } else {
        sprintf(page, "%d\n", ts->reject_point);//support
    }
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}

static const struct file_operations proc_reject_point_fops = {
	.write = proc_reject_point_write,
	.read  = proc_reject_point_read,
	.open  = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t proc_limit_switch_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int value = 0 ;
    char buf[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (count > 4) {
        TPD_INFO("%s:count > 4\n",__func__);
        return count;
    }

    if (!ts) {
        TPD_INFO("%s: ts is NULL\n",__func__);
        return count;
    }

    if (copy_from_user(buf, buffer, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }
    sscanf(buf, "%d", &value);
    ts->limit_switch = value;

    TPD_DEBUG("%s: ts->limit_switch = %d\n", __func__, value);
	if (ts->is_suspended == 0) {
		mutex_lock(&ts->mutex);
		ts->ts_ops->mode_switch(ts->chip_data, MODE_LIMIT_SWITCH, ts->limit_switch);
		mutex_unlock(&ts->mutex);
	}
    return count;
}

static ssize_t proc_limit_switch_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts) {
        sprintf(page, "%d\n", -1);//no support
    } else {
        sprintf(page, "%d\n", ts->limit_switch);//support
    }
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}

static const struct file_operations proc_limit_switch_fops = {
	.write = proc_limit_switch_write,
	.read  = proc_limit_switch_read,
	.open  = simple_open,
	.owner = THIS_MODULE,
};

//proc/touchpanel/black_screen_test
static ssize_t proc_black_screen_test_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    int retry = 20;
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    TPD_INFO("%s %ld %lld\n", __func__, count, *ppos);

    if (!ts || !ts->gesture_test.flag)
        return 0;

    ts->gesture_test.message = kzalloc(256, GFP_KERNEL);
    if (!ts->gesture_test.message) {
        TPD_INFO("failed to alloc memory\n");
        return 0;
    }

    /* wait until tp is in sleep, then sleep 500ms to make sure tp is in gesture mode*/
    do {
        if (ts->is_suspended) {
            msleep(500);
            break;
        }
        msleep(200);
    } while(--retry);

    TPD_INFO("%s retry times %d\n", __func__, retry);
    if (retry == 0 && !ts->is_suspended) {
        sprintf(ts->gesture_test.message, "1 errors: not in sleep ");
        goto OUT;
    }

    mutex_lock(&ts->mutex);
    if (ts->ts_ops->black_screen_test) {
        ts->ts_ops->black_screen_test(ts->chip_data, ts->gesture_test.message);
    } else {
         TPD_INFO("black_screen_test not support\n");
         sprintf(ts->gesture_test.message, "1 errors:not support gesture test");
    }
    mutex_unlock(&ts->mutex);

OUT:
    ts->gesture_test.flag = false;
    ts->gesture_enable = ts->gesture_test.gesture_backup;

    ret = simple_read_from_buffer(user_buf, count, ppos, ts->gesture_test.message, strlen(ts->gesture_test.message));
    kfree(ts->gesture_test.message);
    return ret;
}

static ssize_t proc_black_screen_test_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int value = 0;
    char buf[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (count > 2 || !ts)
        return count;

    if (copy_from_user(buf, buffer, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }
    sscanf(buf, "%d", &value);
    TPD_INFO("%s %d\n", __func__, value);

    ts->gesture_test.gesture_backup = ts->gesture_enable;
    ts->gesture_enable = true;
    ts->gesture_test.flag = !!value;

    return count;
}

static const struct file_operations proc_black_screen_test_fops = {
    .owner = THIS_MODULE,
    .open  = simple_open,
    .read  = proc_black_screen_test_read,
    .write = proc_black_screen_test_write,
};

//proc/touchpanel/irq_depth
static ssize_t proc_get_irq_depth_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));
    struct irq_desc *desc = NULL;

    if(!ts) {
        return count;
    }

    desc = irq_to_desc(ts->irq);

    sprintf(page, "%d\n", desc->depth);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}

static ssize_t proc_irq_status_write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
    int value = 0;
    char buf[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (count > 2 || !ts)
        return count;

    if (copy_from_user(buf, user_buf, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }
    sscanf(buf, "%d", &value);
    TPD_INFO("%s %d, %s ts->irq=%d\n", __func__, value, value ? "enable" : "disable", ts->irq);

    if (value == 1) {
        enable_irq(ts->irq);
    } else {
        disable_irq_nosync(ts->irq);
    }

    return count;
}

static const struct file_operations proc_get_irq_depth_fops = {
    .owner = THIS_MODULE,
    .open  = simple_open,
    .read  = proc_get_irq_depth_read,
    .write = proc_irq_status_write,
};

static ssize_t proc_vendor_id_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if(!ts) {
        return count;
    }

    sprintf(page, "%d\n", ts->panel_data.tp_type);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}

static const struct file_operations vendor_id_proc_fops = {
    .owner = THIS_MODULE,
    .open  = simple_open,
    .read  = proc_vendor_id_read,
};

static ssize_t proc_glove_control_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int ret = 0 ;
    char buf[3] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;
    if (count > 2)
        return count;
    if (!ts->ts_ops->mode_switch) {
        TPD_INFO("not support ts_ops->mode_switch callback\n");
        return count;
    }
    if (copy_from_user(buf, buffer, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }
    sscanf(buf, "%d", &ret);
    TPD_INFO("%s:buf = %d, ret = %d\n", __func__, *buf, ret);
    if ((ret == 0) || (ret == 1)) {
        mutex_lock(&ts->mutex);
        ts->glove_enable = ret;
        ret = ts->ts_ops->mode_switch(ts->chip_data, MODE_GLOVE, ts->glove_enable);
        if (ret < 0) {
            TPD_INFO("%s, Touchpanel operate mode switch failed\n", __func__);
        }
        mutex_unlock(&ts->mutex);
    }
    switch(ret) {
        case 0:
            TPD_DEBUG("tp_glove_func will be disable\n");
            break;
        case 1:
            TPD_DEBUG("tp_glove_func will be enable\n");
            break;
        default:
            TPD_DEBUG("Please enter 0 or 1 to open or close the glove function\n");
    }

    return count;
}

static ssize_t proc_glove_control_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[PAGESIZE] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    TPD_INFO("glove mode enable is: %d\n", ts->glove_enable);
    ret = sprintf(page, "%d\n", ts->glove_enable);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

static const struct file_operations proc_glove_control_fops = {
    .write = proc_glove_control_write,
    .read =  proc_glove_control_read,
    .open = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct button_map *button_map;
    if (!g_tp)
        return sprintf(buf, "not support");

    button_map = &g_tp->button_map;
    return sprintf(buf,
            __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":%d:%d:%d:%d"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":%d:%d:%d:%d"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":%d:%d:%d:%d"
            "\n", button_map->coord_menu.x, button_map->coord_menu.y, button_map->width_x, button_map->height_y, \
            button_map->coord_home.x, button_map->coord_home.y, button_map->width_x, button_map->height_y, \
            button_map->coord_back.x, button_map->coord_back.y, button_map->width_x, button_map->height_y);
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys."TPD_DEVICE,
        .mode = S_IRUGO,
    },
    .show = &cap_vk_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static ssize_t proc_debug_control_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    uint8_t ret = 0;
    char page[PAGESIZE];

    TPD_INFO("%s: tp_debug = %d.\n", __func__, tp_debug);
    sprintf(page, "%d", tp_debug);
    ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

    return ret;
}

static ssize_t proc_debug_control_write(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
    int tmp = 0;
        char buffer[4] = {0};

        if (count > 2) {
                return count;
        }

        if (copy_from_user(buffer, buf, count)) {
                TPD_INFO("%s: read proc input error.\n", __func__);
                return count;
        }

        if (1 == sscanf(buffer, "%d", &tmp)) {
        tp_debug = tmp;
    } else {
        TPD_DEBUG("invalid content: '%s', length = %zd\n", buf, count);
    }

    return count;
}

static const struct file_operations proc_debug_control_ops =
{
    .write = proc_debug_control_write,
    .read  = proc_debug_control_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_limit_area_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    ssize_t ret = 0;
    char page[PAGESIZE];
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;
    TPD_DEBUG("limit_area is: %d\n", ts->edge_limit.limit_area);
    ret = sprintf(page, "limit_area = %d left_x1 = %d right_x1 = %d left_x2 = %d right_x2 = %d left_x3 = %d right_x3 = %d left_y1 = %d right_y1 = %d left_y2 = %d right_y2 = %d left_y3 = %d right_y3 = %d\n",
            ts->edge_limit.limit_area, ts->edge_limit.left_x1, ts->edge_limit.right_x1, ts->edge_limit.left_x2, ts->edge_limit.right_x2, ts->edge_limit.left_x3, ts->edge_limit.right_x3,
            ts->edge_limit.left_y1, ts->edge_limit.right_y1, ts->edge_limit.left_y2, ts->edge_limit.right_y2, ts->edge_limit.left_y3, ts->edge_limit.right_y3);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

static ssize_t proc_limit_area_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    char buf[8];
    int temp;
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    if (copy_from_user(buf, buffer, count)) {
        TPD_DEBUG("%s: read proc input error.\n", __func__);
        return count;
    }

    sscanf(buf, "%d", &temp);
    if (temp < 0 || temp > 10)
        return count;

    ts->edge_limit.limit_area = temp;
    ts->edge_limit.left_x1    = (ts->edge_limit.limit_area*1000)/100;
    ts->edge_limit.right_x1   = ts->resolution_info.LCD_WIDTH - ts->edge_limit.left_x1;
    ts->edge_limit.left_x2    = 2 * ts->edge_limit.left_x1;
    ts->edge_limit.right_x2   = ts->resolution_info.LCD_WIDTH - (2 * ts->edge_limit.left_x1);
    ts->edge_limit.left_x3    = 5 * ts->edge_limit.left_x1;
    ts->edge_limit.right_x3   = ts->resolution_info.LCD_WIDTH - (5 * ts->edge_limit.left_x1);

    TPD_INFO("limit_area = %d; left_x1 = %d; right_x1 = %d; left_x2 = %d; right_x2 = %d; left_x3 = %d; right_x3 = %d\n",
           ts->edge_limit.limit_area, ts->edge_limit.left_x1, ts->edge_limit.right_x1, ts->edge_limit.left_x2, ts->edge_limit.right_x2, ts->edge_limit.left_x3, ts->edge_limit.right_x3);

    ts->edge_limit.left_y1    = (ts->edge_limit.limit_area*1000)/100;
    ts->edge_limit.right_y1   = ts->resolution_info.LCD_HEIGHT - ts->edge_limit.left_y1;
    ts->edge_limit.left_y2    = 2 * ts->edge_limit.left_y1;
    ts->edge_limit.right_y2   = ts->resolution_info.LCD_HEIGHT - (2 * ts->edge_limit.left_y1);
    ts->edge_limit.left_y3    = 5 * ts->edge_limit.left_y1;
    ts->edge_limit.right_y3   = ts->resolution_info.LCD_HEIGHT - (5 * ts->edge_limit.left_y1);

    TPD_INFO("limit_area = %d; left_y1 = %d; right_y1 = %d; left_y2 = %d; right_y2 = %d; left_y3 = %d; right_y3 = %d\n",
            ts->edge_limit.limit_area, ts->edge_limit.left_y1, ts->edge_limit.right_y1, ts->edge_limit.left_y2, ts->edge_limit.right_y2, ts->edge_limit.left_y3, ts->edge_limit.right_y3);

    return count;
}

static const struct file_operations proc_limit_area_ops =
{
    .read  = proc_limit_area_read,
    .write = proc_limit_area_write,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_limit_control_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    ssize_t ret = 0;
    char page[PAGESIZE];
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    TPD_INFO("limit_enable is: 0x%x, ts->limit_edge = 0x%x, ts->limit_corner = 0x%x\n", ts->limit_enable, ts->limit_edge, ts->limit_corner);
    ret = sprintf(page, "%d\n", ts->limit_enable);

    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

static ssize_t proc_limit_control_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    char buf[8] = {0};
    int ret, temp;
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    if (count > 3)
        count = 3;
    if (copy_from_user(buf, buffer, count)) {
        TPD_DEBUG("%s: read proc input error.\n", __func__);
        return count;
    }

    sscanf(buf, "%x", &temp);
    if (temp > 0x1F) {
        TPD_INFO("%s: temp = 0x%x > 0x1F \n", __func__, temp);
        return count;
    }

    mutex_lock(&ts->mutex);
    ts->limit_enable = temp;
    ts->limit_edge = ts->limit_enable & 1;
    ts->limit_corner = ts->limit_enable >> 1;
    TPD_INFO("%s: limit_enable = 0x%x, ts->limit_edge = 0x%x, ts->limit_corner=0x%x\n", __func__, ts->limit_enable, ts->limit_edge, ts->limit_corner);

    if (ts->is_suspended == 0) {
        ret = ts->ts_ops->mode_switch(ts->chip_data, MODE_EDGE, ts->limit_edge);
        if (ret < 0) {
            TPD_INFO("%s, Touchpanel operate mode switch failed\n", __func__);
        }
    }
    mutex_unlock(&ts->mutex);

    return count;
}

static const struct file_operations proc_limit_control_ops =
{
    .read  = proc_limit_control_read,
    .write = proc_limit_control_write,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_fw_update_write(struct file *file, const char __user *page, size_t size, loff_t *lo)
{
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));
    int val = 0;
    int ret = 0;
    char buf[4] = {0};
    if (!ts)
        return size;
    if (size > 2)
        return size;

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
    if (ts->boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT)
#else
    if (ts->boot_mode == MSM_BOOT_MODE__CHARGE)
#endif
    {
        TPD_INFO("boot mode is MSM_BOOT_MODE__CHARGE,not need update tp firmware\n");
        return size;
    }

    if (copy_from_user(buf, page, size)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return size;
    }

    sscanf(buf, "%d", &val);
    ts->firmware_update_type = val;
    if (!ts->force_update && ts->firmware_update_type != 2)
        ts->force_update = !!val;

    schedule_work(&ts->fw_update_work);

    ret = wait_for_completion_killable_timeout(&ts->fw_complete, FW_UPDATE_COMPLETE_TIMEOUT);
    if (ret < 0) {
        TPD_INFO("kill signal interrupt\n");
    }

    TPD_INFO("fw update finished\n");
    return size;
}



static const struct file_operations proc_fw_update_ops =
{
    .write = proc_fw_update_write,
    .open = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_finger_protect_result_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    uint8_t ret = 0;
    char page[16] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));
    if(!ts)
        return 0;

    if(ts->spuri_fp_touch.fp_touch_st == FINGER_PROTECT_TOUCH_UP || ts->spuri_fp_touch.fp_touch_st == FINGER_PROTECT_TOUCH_DOWN)
        TPD_INFO("%s report_finger_protect = %d\n", __func__, ts->spuri_fp_touch.fp_touch_st);
    ret = sprintf(page, "%d\n", ts->spuri_fp_touch.fp_touch_st);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}


static const struct file_operations proc_finger_protect_result= {
    .read  =  proc_finger_protect_result_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_finger_protect_trigger_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int op = 0;
    char buf[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

        if (count > 2) {
                return count;
        }
        if (!ts) {
                return count;
        }

        if (copy_from_user(buf, buffer, count)) {
                TPD_INFO("%s: read proc input error.\n", __func__);
                return count;
        }

        if (1 == sscanf(buf, "%d", &op)) {
        if (op == 1) {
            ts->spuri_fp_touch.fp_trigger= true;
            ts->spuri_fp_touch.fp_touch_st = FINGER_PROTECT_NOTREADY;
            TPD_INFO("%s : %d\n",__func__,__LINE__);
            wake_up_interruptible(&waiter);
        }
    } else {
        TPD_INFO("invalid content: '%s', length = %zd\n", buffer, count);
        return -EINVAL;
    }

    return count;
}

static const struct file_operations proc_finger_protect_trigger= {
    .write = proc_finger_protect_trigger_write,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

void lcd_wakeup_finger_protect(bool wakeup)
{
    TPD_INFO("%s wakeup=%d\n",__func__,wakeup);
    if (g_tp != NULL) {
        if (g_tp->spuri_fp_touch.lcd_trigger_fp_check) {
            if(wakeup) {
                g_tp->spuri_fp_touch.lcd_resume_ok = true;
                wake_up_interruptible(&waiter);
            }
            else {
                mutex_lock(&g_tp->mutex);
                g_tp->spuri_fp_touch.lcd_resume_ok = false;
                mutex_unlock(&g_tp->mutex);
            }
        }
    }
}

static int finger_protect_handler(void *data)
{
    struct touchpanel_data *ts = (struct touchpanel_data *)data;
    if (!ts) {
        TPD_INFO("ts is null should nerver get here!\n");
        return 0;
    };
    if (!ts->ts_ops->spurious_fp_check) {
        TPD_INFO("not support spurious_fp_check call back\n");
        return 0;
    }

    do {
        if (ts->spuri_fp_touch.lcd_trigger_fp_check)
            wait_event_interruptible(waiter, ts->spuri_fp_touch.fp_trigger && ts->i2c_ready && ts->spuri_fp_touch.lcd_resume_ok);
        else
            wait_event_interruptible(waiter, ts->spuri_fp_touch.fp_trigger && ts->i2c_ready);
        ts->spuri_fp_touch.fp_trigger = false;
        ts->spuri_fp_touch.fp_touch_st = FINGER_PROTECT_NOTREADY;

        mutex_lock(&ts->mutex);
        if (g_tp->spuri_fp_touch.lcd_trigger_fp_check && !g_tp->spuri_fp_touch.lcd_resume_ok) {
            TPD_INFO("LCD is suspend, can not detect finger touch in incell panel\n");
            mutex_unlock(&ts->mutex);
            continue;
        }

        ts->spuri_fp_touch.fp_touch_st = ts->ts_ops->spurious_fp_check(ts->chip_data);
        if (ts->view_area_touched) {
            TPD_INFO("%s tp touch down,clear flag\n",__func__);
            ts->view_area_touched = 0;
        }
        operate_mode_switch(ts);
        mutex_unlock(&ts->mutex);
    } while (!kthread_should_stop());
    return 0;
}

//proc/touchpanel/register_info node use info:
//first choose register_add and lenght, example: echo 000e,1 > register_info
//second read: cat register_info
static ssize_t proc_register_info_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    int i = 0;
    ssize_t num_read_chars = 0;
    char page[256] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    if (ts->reg_info.reg_length < 1 || ts->reg_info.reg_length > 9) {
        TPD_INFO("ts->reg_info.reg_length error!\n");
        return count;
    }
    ts->reg_info.reg_result = kzalloc(ts->reg_info.reg_length * (sizeof(uint16_t)), GFP_KERNEL);
    if (!ts->reg_info.reg_result) {
        TPD_INFO("ts->reg_info.reg_result kzalloc error\n");
        return count;
    }

    if (ts->ts_ops->register_info_read) {
        mutex_lock(&ts->mutex);
        ts->ts_ops->register_info_read(ts->chip_data, ts->reg_info.reg_addr, ts->reg_info.reg_result, ts->reg_info.reg_length);
        mutex_unlock(&ts->mutex);
        for(i = 0; i < ts->reg_info.reg_length; i++) {
            num_read_chars += sprintf(&(page[num_read_chars]), "reg_addr(0x%x) = 0x%x\n", ts->reg_info.reg_addr, ts->reg_info.reg_result[i]);
        }
        ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    }

    kfree(ts->reg_info.reg_result);
    return ret;
}

//write info: echo 000e,1 > register_info
static ssize_t proc_register_info_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int addr = 0, length = 0;
    char buf[16] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (count > 7) {
        TPD_INFO("%s count = %ld\n", __func__, count);
        return count;
    }
    if (!ts) {
        TPD_INFO("ts not exist!\n");
        return count;
    }
    if (copy_from_user(buf, buffer, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }

    sscanf(buf, "%x,%d", &addr, &length);
    ts->reg_info.reg_addr = (uint16_t)addr;
    ts->reg_info.reg_length = (uint16_t)length;
    TPD_INFO("ts->reg_info.reg_addr = 0x%x, ts->reg_info.reg_lenght = %d\n", ts->reg_info.reg_addr, ts->reg_info.reg_length);

    return count;
}


static const struct file_operations proc_register_info_fops = {
    .owner = THIS_MODULE,
    .open  = simple_open,
    .read  = proc_register_info_read,
    .write = proc_register_info_write,
};

static ssize_t proc_incell_panel_info_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    uint8_t ret = 0;
    char page[32];
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    sprintf(page, "%d", ts->is_incell_panel);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

static const struct file_operations proc_incell_panel_fops = {
    .owner = THIS_MODULE,
    .open = simple_open,
    .read = proc_incell_panel_info_read,
};

/**
 * init_touchpanel_proc - Using for create proc interface
 * @ts: touchpanel_data struct using for common driver
 *
 * we need to set touchpanel_data struct as private_data to those file_inode
 * Returning zero(success) or negative errno(failed)
 */
static ssize_t sec_update_fw_store(struct device *dev, struct device_attribute *attr, const char *buffer, size_t size)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);
	int val;
	int ret = 0;

	if (!ts)
		return size;
	if (size > 2)
		return size;

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	if (ts->boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT)
#else
	if (ts->boot_mode == MSM_BOOT_MODE__CHARGE)
#endif
	{
		TPD_INFO("boot mode is MSM_BOOT_MODE__CHARGE,not need update tp firmware\n");
		return size;
	}

	ret = kstrtoint(buffer, 10, &val);
	if (ret != 0) {
		TPD_INFO("invalid content: '%s', length = %zd\n", buffer, size);
		return ret;
	}
	ts->firmware_update_type = val;
	if (!ts->force_update && ts->firmware_update_type != 2)
		ts->force_update = !!val;

	schedule_work(&ts->fw_update_work);

	ret = wait_for_completion_killable_timeout(&ts->fw_complete, FW_UPDATE_COMPLETE_TIMEOUT);
	if (ret < 0) {
		TPD_INFO("kill signal interrupt\n");
	}

	TPD_INFO("fw update finished\n");
	return size;

}
static ssize_t sec_update_fw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", ts->loading_fw);
}

static DEVICE_ATTR(tp_fw_update, 0644, sec_update_fw_show, sec_update_fw_store);

static int init_touchpanel_proc(struct touchpanel_data *ts)
{
    int ret = 0;
    struct proc_dir_entry *prEntry_tp = NULL;
    struct proc_dir_entry *prEntry_tmp = NULL;

    TPD_INFO("%s entry\n", __func__);

    //proc files-step1:/proc/devinfo/tp  (touchpanel device info)
    if(ts->fw_update_app_support) {
        register_devinfo("tp", &ts->panel_data.manufacture_info);
    }	

	if (device_create_file(&ts->client->dev, &dev_attr_tp_fw_update)) {
		TPD_INFO("driver_create_file failt\n");
		ret = -ENOMEM;
	}
    //proc files-step2:/proc/touchpanel
    prEntry_tp = proc_mkdir("touchpanel", NULL);
    if (prEntry_tp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create TP proc entry\n", __func__);
    }

    //proc files-step2-1:/proc/touchpanel/tp_debug_log_level (log control interface)
    prEntry_tmp = proc_create("tp_debug_log", 0644, prEntry_tp, &proc_debug_control_ops);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    //proc files-step2-2:/proc/touchpanel/tp_fw_update (FW update interface)
    prEntry_tmp = proc_create_data("tp_fw_update", 0644, prEntry_tp, &proc_fw_update_ops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    //proc files-step2-3:/proc/touchpanel/tp_fw_update (edge limit control interface)
    if (ts->edge_limit_support) {
        prEntry_tmp = proc_create_data("tp_limit_area", 0664, prEntry_tp, &proc_limit_area_ops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
        prEntry_tmp = proc_create_data("tp_limit_enable", 0664, prEntry_tp, &proc_limit_control_ops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
    }

    //proc files-step2-4:/proc/touchpanel/double_tap_enable (black gesture related interface)
    if (ts->black_gesture_support) {
        prEntry_tmp = proc_create_data("gesture_enable", 0666, prEntry_tp, &proc_gesture_control_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
        prEntry_tmp = proc_create_data("coordinate", 0444, prEntry_tp, &proc_coordinate_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
    }

    //proc files-step2-5:/proc/touchpanel/glove_mode_enable (Glove mode related interface)
    if (ts->glove_mode_support) {
        prEntry_tmp = proc_create_data("glove_mode_enable", 0666, prEntry_tp, &proc_glove_control_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
    }

    //proc files-step2-6:/proc/touchpanel/finger_protect_result
    if (ts->spurious_fp_support) {
        prEntry_tmp = proc_create_data("finger_protect_result", 0666, prEntry_tp, &proc_finger_protect_result, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
        prEntry_tmp = proc_create_data("finger_protect_trigger", 0666, prEntry_tp, &proc_finger_protect_trigger, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
    }

    //proc files-step2-7:/proc/touchpanel/register_info
    prEntry_tmp = proc_create_data("register_info", 0664, prEntry_tp, &proc_register_info_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    prEntry_tmp = proc_create_data("ps_status", 0666, prEntry_tp, &proc_write_ps_status_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }


    //proc files-step2-8:/proc/touchpanel/incell_panel
    if (ts->is_incell_panel) {
        prEntry_tmp = proc_create_data("incell_panel", 0664, prEntry_tp, &proc_incell_panel_fops, ts);
    }

    if (ts->gesture_test_support) {
        prEntry_tmp = proc_create_data("black_screen_test", 0666, prEntry_tp, &proc_black_screen_test_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
    }

    //proc file-step2-9:/proc/touchpanel/irq_depth
    prEntry_tmp = proc_create_data("irq_depth", 0666, prEntry_tp, &proc_get_irq_depth_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    //proc file-step2-10:/proc/touchpanel/vendor_id
    prEntry_tmp = proc_create_data("vendor_id", 0444, prEntry_tp, &vendor_id_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    //proc files-step2-3:/proc/touchpanel/game_switch_enable (edge limit control interface)
    if (ts->game_switch_support) {
        prEntry_tmp = proc_create_data("game_switch_enable", 0666, prEntry_tp, &proc_game_switch_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
    }

	prEntry_tmp = proc_create_data("gesture_switch", 0666, prEntry_tp, &proc_gesture_switch_fops, ts);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

	prEntry_tmp = proc_create_data("reject_point", 0666, prEntry_tp, &proc_reject_point_fops, ts);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

	prEntry_tmp = proc_create_data("tpedge_limit_enable", 0666, prEntry_tp, &proc_limit_switch_fops, ts);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

    ts->prEntry_tp = prEntry_tp;

    //create debug_info node
    init_debug_info_proc(ts);

    return ret;
}

//proc/touchpanel/debug_info/baseline
static int tp_baseline_debug_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct debug_info_proc_operations *debug_info_ops;

    if (!ts)
        return 0;
    debug_info_ops = (struct debug_info_proc_operations *)(ts->debug_info_ops);
    if (!debug_info_ops) {
        TPD_INFO("debug_info_ops==NULL");
        return 0;
    }
    if (!debug_info_ops->baseline_read && !debug_info_ops->baseline_blackscreen_read) {
        seq_printf(s, "Not support baseline proc node\n");
        return 0;
    }
    if ((ts->suspend_state != TP_SPEEDUP_RESUME_COMPLETE) && (1 != ts->gesture_enable)) {
        seq_printf(s, "Not in resume over or gesture state\n");
        return 0;
    }

    if (ts->int_mode == BANNABLE) {
        disable_irq_nosync(ts->irq);
    }
    mutex_lock(&ts->mutex);
    if (ts->is_suspended && ts->gesture_enable) {
        if (debug_info_ops->baseline_blackscreen_read) {
            debug_info_ops->baseline_blackscreen_read(s, ts->chip_data);
        }
    } else {
        if (debug_info_ops->baseline_read) {
            debug_info_ops->baseline_read(s, ts->chip_data);
        }
    }

    //step6: return to normal mode
    ts->ts_ops->reset(ts->chip_data);
    operate_mode_switch(ts);

    mutex_unlock(&ts->mutex);

    if (ts->int_mode == BANNABLE) {
        enable_irq(ts->irq);
    }
    return 0;

}

static int data_baseline_open(struct inode *inode, struct file *file)
{
    return single_open(file, tp_baseline_debug_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_baseline_data_proc_fops = {
    .owner = THIS_MODULE,
    .open = data_baseline_open,
    .read = seq_read,
    .release = single_release,
};

//proc/touchpanel/debug_info/delta
static int tp_delta_debug_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct debug_info_proc_operations *debug_info_ops;

    if (!ts)
        return 0;
    debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

    if (!debug_info_ops)
        return 0;
    if (!debug_info_ops->delta_read) {
        seq_printf(s, "Not support delta proc node\n");
        return 0;
    }
    if (ts->suspend_state != TP_SPEEDUP_RESUME_COMPLETE) {
        seq_printf(s, "Not in resume over state\n");
        return 0;
    }

    if (ts->int_mode == BANNABLE) {
        disable_irq_nosync(ts->irq);
    }
    mutex_lock(&ts->mutex);
    debug_info_ops->delta_read(s, ts->chip_data);
    mutex_unlock(&ts->mutex);
    if (ts->int_mode == BANNABLE) {
        enable_irq(ts->irq);
    }
    return 0;
}

static int data_delta_open(struct inode *inode, struct file *file)
{
    return single_open(file, tp_delta_debug_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_delta_data_proc_fops = {
    .owner = THIS_MODULE,
    .open  = data_delta_open,
    .read  = seq_read,
    .release = single_release,
};

//proc/touchpanel/debug_info/self_delta
static int tp_self_delta_debug_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct debug_info_proc_operations *debug_info_ops;

    if (!ts)
        return 0;
    debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

    if (!debug_info_ops)
        return 0;
    if (!debug_info_ops->self_delta_read) {
        seq_printf(s, "Not support self_delta proc node\n");
        return 0;
    }
    if (ts->suspend_state != TP_SPEEDUP_RESUME_COMPLETE) {
        seq_printf(s, "Not in resume over state\n");
        return 0;
    }

    if (ts->int_mode == BANNABLE) {
        disable_irq_nosync(ts->irq);
    }
    mutex_lock(&ts->mutex);
    debug_info_ops->self_delta_read(s, ts->chip_data);
    mutex_unlock(&ts->mutex);
    if (ts->int_mode == BANNABLE) {
        enable_irq(ts->irq);
    }
    return 0;
}

static int data_self_delta_open(struct inode *inode, struct file *file)
{
    return single_open(file, tp_self_delta_debug_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_self_delta_data_proc_fops = {
    .owner = THIS_MODULE,
    .open  = data_self_delta_open,
    .read  = seq_read,
    .release = single_release,
};

//proc/touchpanel/debug_info/self_raw
static int tp_self_raw_debug_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct debug_info_proc_operations *debug_info_ops;

    if (!ts)
        return 0;
    debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

    if (!debug_info_ops)
        return 0;
    if (!debug_info_ops->self_raw_read) {
        seq_printf(s, "Not support self_raw proc node\n");
        return 0;
    }
    if (ts->suspend_state != TP_SPEEDUP_RESUME_COMPLETE) {
        seq_printf(s, "Not in resume over state\n");
        return 0;
    }

    if (ts->int_mode == BANNABLE) {
        disable_irq_nosync(ts->irq);
    }
    mutex_lock(&ts->mutex);
    debug_info_ops->self_raw_read(s, ts->chip_data);
    mutex_unlock(&ts->mutex);
    if (ts->int_mode == BANNABLE) {
        enable_irq(ts->irq);
    }
    return 0;
}

static int data_self_raw_open(struct inode *inode, struct file *file)
{
    return single_open(file, tp_self_raw_debug_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_self_raw_data_proc_fops = {
    .owner = THIS_MODULE,
    .open  = data_self_raw_open,
    .read  = seq_read,
    .release = single_release,
};

//proc/touchpanel/debug_info/main_register
static int tp_main_register_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct debug_info_proc_operations *debug_info_ops;

    if (!ts)
        return 0;
    debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

    if (!debug_info_ops)
        return 0;
    if (!debug_info_ops->main_register_read) {
        seq_printf(s, "Not support main_register proc node\n");
        return 0;
    }
    if (ts->suspend_state != TP_SPEEDUP_RESUME_COMPLETE) {
        seq_printf(s, "Not in resume over state\n");
        return 0;
    }

    if (ts->int_mode == BANNABLE) {
        disable_irq_nosync(ts->irq);
    }
    mutex_lock(&ts->mutex);
    seq_printf(s, "es_enable:%d\n", ts->es_enable);
    seq_printf(s, "touch_count:%d\n", ts->touch_count);
    debug_info_ops->main_register_read(s, ts->chip_data);
    mutex_unlock(&ts->mutex);
    if (ts->int_mode == BANNABLE) {
        enable_irq(ts->irq);
    }
    return 0;
}

static int main_register_open(struct inode *inode, struct file *file)
{
    return single_open(file, tp_main_register_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_main_register_proc_fops = {
    .owner = THIS_MODULE,
    .open  = main_register_open,
    .read  = seq_read,
    .release = single_release,
};

//proc/touchpanel/debug_info/reserve
static int tp_reserve_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct debug_info_proc_operations *debug_info_ops;

    if (!ts)
        return 0;
    debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

    if (!debug_info_ops)
        return 0;
    if (!debug_info_ops->reserve_read) {
        seq_printf(s, "Not support main_register proc node\n");
        return 0;
    }
    if (ts->suspend_state != TP_SPEEDUP_RESUME_COMPLETE) {
        seq_printf(s, "Not in resume over state\n");
        return 0;
    }

    if (ts->int_mode == BANNABLE) {
        disable_irq_nosync(ts->irq);
    }
    mutex_lock(&ts->mutex);
    debug_info_ops->reserve_read(s, ts->chip_data);
    mutex_unlock(&ts->mutex);

    if (ts->int_mode == BANNABLE) {
        enable_irq(ts->irq);
    }
    return 0;
}

static int reserve_open(struct inode *inode, struct file *file)
{
    return single_open(file, tp_reserve_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_reserve_proc_fops = {
    .owner = THIS_MODULE,
    .open  = reserve_open,
    .read  = seq_read,
    .release = single_release,
};

//proc/touchpanel/debug_info/data_limit
static int tp_limit_data_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct debug_info_proc_operations *debug_info_ops;

    if (!ts)
        return 0;
    debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;
    if (!debug_info_ops)
        return 0;
    if (!debug_info_ops->limit_read) {
        seq_printf(s, "Not support limit_data proc node\n");
        return 0;
    }
    debug_info_ops->limit_read(s, ts);

    return 0;
}

static int limit_data_open(struct inode *inode, struct file *file)
{
    return single_open(file, tp_limit_data_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_limit_data_proc_fops = {
    .owner = THIS_MODULE,
    .open  = limit_data_open,
    .read  = seq_read,
    .release = single_release,
};

//proc/touchpanel/debug_info/abs_doze
static int tp_abs_doze_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct debug_info_proc_operations *debug_info_ops;

    if (!ts)
        return 0;
    debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

    if (!debug_info_ops)
        return 0;
    if (!debug_info_ops->abs_doze_read) {
        seq_printf(s, "Not support main_register proc node\n");
        return 0;
    }
    if (ts->suspend_state != TP_SPEEDUP_RESUME_COMPLETE) {
        seq_printf(s, "Not in resume over state\n");
        return 0;
    }

    if (ts->int_mode == BANNABLE) {
        disable_irq_nosync(ts->irq);
    }
    mutex_lock(&ts->mutex);
    debug_info_ops->abs_doze_read(s, ts->chip_data);
    mutex_unlock(&ts->mutex);

    if (ts->int_mode == BANNABLE) {
        enable_irq(ts->irq);
    }
    return 0;
}

static int abs_doze_open(struct inode *inode, struct file *file)
{
    return single_open(file, tp_abs_doze_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_abs_doze_proc_fops = {
    .owner = THIS_MODULE,
    .open  = abs_doze_open,
    .read  = seq_read,
    .release = single_release,
};

//write function of /proc/touchpanel/earsense/palm_control
static ssize_t proc_earsense_palm_control_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int value = 0;
    char buf[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (count > 2)
        return count;
    if (!ts)
        return count;

    if (copy_from_user(buf, buffer, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }
    sscanf(buf, "%d", &value);
    if (value > 2)
        return count;

    TPD_DEBUG("%s value: %d, palm_enable :%d\n", __func__, value, ts->palm_enable);
    if (value == ts->palm_enable)
        return count;

    mutex_lock(&ts->mutex);
    ts->palm_enable = value;
    if (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE) {
        ts->ts_ops->mode_switch(ts->chip_data, MODE_PALM_REJECTION, value);
    }
    mutex_unlock(&ts->mutex);

    return count;
}

//read function of /proc/touchpanel/earsense/palm_control
static ssize_t proc_earsense_palm_control_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    TPD_DEBUG("%s value: %d\n", __func__, ts->palm_enable);
    ret = sprintf(page, "%d\n", ts->palm_enable);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

// operation of /proc/touchpanel/earsense/palm_control
static const struct file_operations tp_earsense_palm_control_fops = {
    .write = proc_earsense_palm_control_write,
    .read  = proc_earsense_palm_control_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

// write function of /proc/touchpanel/earsense/es_enable
static ssize_t proc_earsense_es_enable_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int value = 0;
    bool state_changed = true;
    char buf[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (count > 2)
        return count;
    if (!ts)
        return count;

    if (copy_from_user(buf, buffer, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }
    sscanf(buf, "%d", &value);
    if (value > 2)
        return count;

    TPD_DEBUG("%s value: %d, es_enable :%d\n", __func__, value, ts->es_enable);
    if (value == ts->es_enable)
        return count;

    mutex_lock(&ts->mutex);
    if ((ts->es_enable != 1) && (value != 1))
        state_changed =false;
    ts->es_enable = value;
    if (!ts->es_enable) {
        memset(ts->earsense_delta, 0, 2 * ts->hw_res.EARSENSE_TX_NUM * ts->hw_res.EARSENSE_RX_NUM);
    }
    if (!ts->is_suspended && (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE) && state_changed) {
        ts->ts_ops->mode_switch(ts->chip_data, MODE_EARSENSE, ts->es_enable == 1);
    }
    mutex_unlock(&ts->mutex);

    return count;
}

// read function of /proc/touchpanel/earsense/es_enable
static ssize_t proc_earsense_es_enable_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    TPD_DEBUG("%s value: %d\n", __func__, ts->es_enable);
    ret = sprintf(page, "%d\n", ts->es_enable);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

// operation of /proc/touchpanel/earsense/es_enable
static const struct file_operations tp_earsense_es_enable_fops = {
    .write = proc_earsense_es_enable_write,
    .read  = proc_earsense_es_enable_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

// read function of /proc/touchpanel/earsense/es_touch_count
static ssize_t proc_earsense_touchcnt_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    TPD_DEBUG("%s value: %d\n", __func__, ts->touch_count);
    mutex_lock(&ts->mutex);
    ret = sprintf(page, "%d\n", ts->touch_count);
    mutex_unlock(&ts->mutex);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

// operation of /proc/touchpanel/earsense/es_touch_count
static const struct file_operations tp_earsense_es_touchcnt_fops = {
    .read  = proc_earsense_touchcnt_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

// read function of /proc/touchpanel/earsense/rawdata
static ssize_t proc_earsense_rawdata_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));
    int read_len = 2 * ts->hw_res.EARSENSE_TX_NUM * ts->hw_res.EARSENSE_RX_NUM;

    char *tmp_data = NULL;
    if (!ts)
        *ppos += 11;
    if (*ppos > 10)
        return 0;
    if (count != read_len) {
        TPD_INFO("%s, length:%d not match data_len:%d\n", __func__, (int)count, read_len);
        return 0;
    }

    TPD_DEBUG("%s is called\n", __func__);
    mutex_lock(&ts->mutex);
    if ((!ts->es_enable) || (ts->suspend_state != TP_SPEEDUP_RESUME_COMPLETE)) {
        *ppos += 11;
        mutex_unlock(&ts->mutex);
        return 0;
    }
    if (ts->delta_state == TYPE_DELTA_IDLE) {
        tmp_data = kzalloc(read_len ,GFP_KERNEL);
        ts->earsense_ops->rawdata_read(ts->chip_data, tmp_data, read_len);
        mutex_unlock(&ts->mutex);
        ret = copy_to_user(user_buf, tmp_data, read_len);
        if (ret)
            TPD_INFO("touch rawdata read fail\n");
        kfree(tmp_data);
        *ppos += 11;
    } else {
        mutex_unlock(&ts->mutex);
        msleep(3);
        *ppos += 1;
    }

    return read_len;
}

// operation of /proc/touchpanel/earsense/rawdata
static const struct file_operations tp_earsense_rawdata_fops = {
    .read  = proc_earsense_rawdata_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

// read function of /proc/touchpanel/earsense/delta
static ssize_t proc_earsense_delta_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));
    int read_len = 2 * ts->hw_res.EARSENSE_TX_NUM * ts->hw_res.EARSENSE_RX_NUM;

    if (!ts)
        return count;
    if (*ppos > 0)
        return 0;
    if (count != read_len) {
       TPD_INFO("%s, length:%d not match data_len:%d\n", __func__, (int)count, read_len);
        return 0;
    }

    TPD_DEBUG("%s is called\n", __func__);
    if ((!ts->es_enable) || (ts->suspend_state != TP_SPEEDUP_RESUME_COMPLETE)) {
        return 0;
    }

    mutex_lock(&ts->mutex_earsense);
    ret = copy_to_user(user_buf, ts->earsense_delta, read_len);
    mutex_unlock(&ts->mutex_earsense);
    if (ret)
        TPD_INFO("tp rawdata read fail\n");
    *ppos += read_len;
    return read_len;
}

// operation of /proc/touchpanel/earsense/delta
static const struct file_operations tp_earsense_delta_fops = {
    .read  = proc_earsense_delta_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

// read function of /proc/touchpanel/earsense/hover_selfdata
static ssize_t proc_earsense_selfdata_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));
    uint16_t data_len = 2*(ts->hw_res.TX_NUM + ts->hw_res.RX_NUM);

    char *tmp_data = NULL;
    if (!ts)
        *ppos += 11;
    if (*ppos > 10)
        return 0;
    if (count != data_len) {
        TPD_INFO("%s, length:%d not match data_len:%d\n", __func__, (int)count, data_len);
        return 0;
    }

    TPD_DEBUG("%s is called\n", __func__);
    mutex_lock(&ts->mutex);
    if ((!ts->es_enable) || (ts->suspend_state != TP_SPEEDUP_RESUME_COMPLETE)) {
        *ppos += 11;
        mutex_unlock(&ts->mutex);
        return 0;
    }
    if (ts->delta_state == TYPE_DELTA_IDLE) {
        tmp_data = kzalloc(data_len ,GFP_KERNEL);
        ts->earsense_ops->self_data_read(ts->chip_data, tmp_data, data_len);
        mutex_unlock(&ts->mutex);
        ret = copy_to_user(user_buf, tmp_data, data_len);
        if (ret)
            TPD_INFO("tp self delta read fail\n");
        kfree(tmp_data);
        *ppos += 11;
    } else {
        mutex_unlock(&ts->mutex);
        msleep(3);
        *ppos += 1;
    }

    return data_len;
}

// operation of /proc/touchpanel/earsense/hover_selfdata
static const struct file_operations tp_earsense_selfdata_fops = {
    .read  = proc_earsense_selfdata_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

// write function of /proc/touchpanel/earsense/es_enable
static ssize_t proc_fd_enable_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int value = 0;
    char buf[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (count > 2)
        return count;
    if (!ts)
        return count;

    if (copy_from_user(buf, buffer, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }
    sscanf(buf, "%d", &value);
    if (value > 2)
        return count;

    TPD_DEBUG("%s value: %d, es_enable :%d\n", __func__, value, ts->fd_enable);
    if (!value) {
        input_event(ps_input_dev, EV_MSC, MSC_RAW, 2);
		input_sync(ps_input_dev);
    }
    if (value == ts->fd_enable)
        return count;

    mutex_lock(&ts->mutex);
    ts->fd_enable = value;
    if (!ts->is_suspended && (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE)) {
        ts->ts_ops->mode_switch(ts->chip_data, MODE_FACE_DETECT, ts->fd_enable == 1);
		input_event(ps_input_dev, EV_MSC, MSC_RAW, 0);	//when open fd report default key for sensor.
		input_sync(ps_input_dev);
    }
    mutex_unlock(&ts->mutex);

    return count;
}

// read function of /proc/touchpanel/fd_enable
static ssize_t proc_fd_enable_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    TPD_DEBUG("%s value: %d\n", __func__, ts->fd_enable);
    ret = sprintf(page, "%d\n", ts->fd_enable);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

// operation of /proc/touchpanel/fd_enable
static const struct file_operations tp_fd_enable_fops = {
    .write = proc_fd_enable_write,
    .read  = proc_fd_enable_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

// read function of /proc/touchpanel/event_num
static ssize_t proc_event_num_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    const char *devname = NULL;
    struct input_handle *handle;
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    list_for_each_entry(handle, &ps_input_dev->h_list, d_node) {
        if (strncmp(handle->name, "event", 5) == 0) {
            devname = handle->name;
            break;
        }
    }

    ret = simple_read_from_buffer(user_buf, count, ppos, devname, strlen(devname));
    return ret;
}

// operation of /proc/touchpanel/event_num
static const struct file_operations tp_event_num_fops = {
    .read  = proc_event_num_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_fd_calibrate_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int value = 0;
    char buf[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (count > 2)
        return count;
    if (!ts)
        return count;

    if (copy_from_user(buf, buffer, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }
    sscanf(buf, "%d", &value);
    if (value > 2)
        return count;

    TPD_DEBUG("%s value: %d, fd_calibrate :%d\n", __func__, value, ts->fd_calibrate);

    mutex_lock(&ts->mutex);
    ts->fd_calibrate = value;
    if (ts->fd_enable) {
        ts->ts_ops->mode_switch(ts->chip_data, MODE_FACE_CALIBRATE, ts->fd_calibrate);
    }
    mutex_unlock(&ts->mutex);

    return count;
}

static ssize_t proc_fd_calibrate_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    TPD_DEBUG("%s value: %d\n", __func__, ts->fd_calibrate);
    ret = sprintf(page, "%d\n", ts->fd_calibrate);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

static const struct file_operations tp_fd_calibrate_fops = {
    .write = proc_fd_calibrate_write,
    .read  = proc_fd_calibrate_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_refresh_switch_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 2)
		return count;
	if (!ts)
		return count;

	if (copy_from_user(buf, buffer, count)) {
		TPD_INFO("%s: read proc input error.\n", __func__);
		return count;
	}
	sscanf(buf, "%d", &value);
	if (value > 2)
		return count;

	TPD_DEBUG("%s value: %d, lcd_refresh_rate_switch :%d\n", __func__, value, ts->lcd_refresh_rate);
	if (value == ts->lcd_refresh_rate)
		return count;

	mutex_lock(&ts->mutex);
	ts->lcd_refresh_rate = value;
	if (!ts->is_suspended && (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE)) {
		ts->ts_ops->mode_switch(ts->chip_data, MODE_REFRESH_SWITCH, ts->lcd_refresh_rate == 1);
	}
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t proc_refresh_switch_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts)
		return count;

	TPD_DEBUG("%s value: %d\n", __func__, ts->lcd_refresh_rate);
	ret = sprintf(page, "%d\n", ts->lcd_refresh_rate);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

	return ret;
}
// operation of /proc/touchpanel/lcd_refresh_rate_switch
static const struct file_operations tp_lcd_refresh_switch_fops = {
	.write = proc_refresh_switch_write,
    .read  = proc_refresh_switch_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_touch_hold_switch_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 2)
		return count;
	if (!ts)
		return count;

	if (copy_from_user(buf, buffer, count)) {
		TPD_INFO("%s: read proc input error.\n", __func__);
		return count;
	}
	sscanf(buf, "%d", &value);

	TPD_DEBUG("%s value: %d, touch_hold_enable :%d\n", __func__, value, ts->touch_hold_enable);
	if (value == 2) {
		ts->skip_reset_in_resume = true;
		return count;
	}

	ts->touch_hold_enable = value;

	if ((ts->is_suspended) && (!ts->gesture_enable)) { //suspend and close gesture cannot response touchhold
		return count;
	}

	mutex_lock(&ts->mutex);
	ts->ts_ops->mode_switch(ts->chip_data, MODE_TOUCH_HOLD, ts->touch_hold_enable);
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t proc_touch_hold_switch_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts)
		return count;

	TPD_DEBUG("%s value: %d\n", __func__, ts->touch_hold_enable);
	ret = sprintf(page, "%d\n", ts->touch_hold_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

	return ret;
}

// operation of /proc/touchpanel/touch_hold
static const struct file_operations tp_touch_hold_switch_fops = {
	.write = proc_touch_hold_switch_write,
	.read  = proc_touch_hold_switch_read,
	.open  = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t proc_touch_area_switch_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 2)
		return count;
	if (!ts)
		return count;

	if (copy_from_user(buf, buffer, count)) {
		TPD_INFO("%s: read proc input error.\n", __func__);
		return count;
	}
	sscanf(buf, "%d", &value);
	if (value > 1)
		return count;

	TPD_DEBUG("%s value: %d, touch_hold_enable :%d\n", __func__, value, ts->touch_area_switch);
	if (value == ts->touch_area_switch)
		return count;

	if ((ts->is_suspended) && (!ts->gesture_enable)) { //suspend and close gesture cannot response touchhold
		return count;
	}

	mutex_lock(&ts->mutex);
	ts->touch_area_switch = value;
	ts->ts_ops->mode_switch(ts->chip_data, MODE_TOUCH_AREA_SWITCH, ts->touch_area_switch == 1);
	mutex_unlock(&ts->mutex);
	return count;
}

static ssize_t proc_touch_area_switch_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts)
		return count;

	TPD_DEBUG("%s value: %d\n", __func__, ts->touch_area_switch);
	ret = sprintf(page, "%d\n", ts->touch_area_switch);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

	return ret;
}

// operation of /proc/touchpanel/touch_hold
static const struct file_operations tp_touch_area_switch_fops = {
	.write = proc_touch_area_switch_write,
	.read  = proc_touch_area_switch_read,
	.open  = simple_open,
	.owner = THIS_MODULE,
};
static ssize_t proc_fingerprint_int_test_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 2)
		return count;
	if (!ts)
		return count;

	if (copy_from_user(buf, buffer, count)) {
		TPD_INFO("%s: read proc input error.\n", __func__);
		return count;
	}
	sscanf(buf, "%d", &value);
	if (value > 1)
		return count;

	TPD_DEBUG("%s value: %d, fingerprint_int_test :%d\n", __func__, value, ts->fingerprint_int_test);
	if (value == ts->fingerprint_int_test)
		return count;

	mutex_lock(&ts->mutex);
	ts->fingerprint_int_test = value;
	ts->ts_ops->mode_switch(ts->chip_data, MODE_FINGERPRINT_TEST, ts->fingerprint_int_test == 1);
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t proc_fingerprint_int_test_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts)
		return count;

	TPD_DEBUG("%s value: %d\n", __func__, ts->fingerprint_int_test);
	ret = sprintf(page, "%d\n", ts->fingerprint_int_test);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

	return ret;
}

static const struct file_operations tp_fingerprint_int_test_fops = {
		.write = proc_fingerprint_int_test_write,
		.read  = proc_fingerprint_int_test_read,
		.open  = simple_open,
		.owner = THIS_MODULE,
};

static ssize_t proc_charge_detect_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 2)
		return count;
	if (!ts)
		return count;

	if (copy_from_user(buf, buffer, count)) {
		TPD_INFO("%s: read proc input error.\n", __func__);
		return count;
	}
	sscanf(buf, "%d", &value);

	TPD_DEBUG("%s value: %d, charge detect enable:%d\n", __func__, value, ts->charge_detect);
	ts->charge_detect = value;
	mutex_lock(&ts->mutex);
	if (ts->charge_detect_support && ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE) {
		ts->ts_ops->mode_switch(ts->chip_data, MODE_CHARGE, ts->charge_detect);
	}
	mutex_unlock(&ts->mutex);
	return count;
}

static ssize_t proc_charge_detect_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts)
		return count;

	TPD_DEBUG("%s value: %d\n", __func__, ts->charge_detect);
	ret = sprintf(page, "%d\n", ts->charge_detect);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

	return ret;
}

// operation of /proc/touchpanel/charge_detect
static const struct file_operations tp_charge_detect_fops = {
	.write = proc_charge_detect_write,
	.read  = proc_charge_detect_read,
	.open  = simple_open,
	.owner = THIS_MODULE,
};

//proc/touchpanel/debug_info/
static int init_debug_info_proc(struct touchpanel_data *ts)
{
    int ret = 0;
    struct proc_dir_entry *prEntry_debug_info = NULL;
    struct proc_dir_entry *prEntry_earsense = NULL;
    struct proc_dir_entry *prEntry_tmp = NULL;

    TPD_INFO("%s entry\n", __func__);

    //proc files-step1:/proc/touchpanel/debug_info
    prEntry_debug_info = proc_mkdir("debug_info", ts->prEntry_tp);
    if (prEntry_debug_info == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create debug_info proc entry\n", __func__);
    }

    // show limit data interface
    prEntry_tmp = proc_create_data("data_limit", 0666, prEntry_debug_info, &tp_limit_data_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    // show baseline data interface
    prEntry_tmp = proc_create_data("baseline", 0666, prEntry_debug_info, &tp_baseline_data_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    // show delta interface
    prEntry_tmp = proc_create_data("delta", 0666, prEntry_debug_info, &tp_delta_data_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    // show self delta interface
    prEntry_tmp = proc_create_data("self_delta", 0666, prEntry_debug_info, &tp_self_delta_data_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    // show self_raw interface
    prEntry_tmp = proc_create_data("self_raw", 0666, prEntry_debug_info, &tp_self_raw_data_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    // show main_register interface
    prEntry_tmp = proc_create_data("main_register", 0666, prEntry_debug_info, &tp_main_register_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    // show reserve interface
    prEntry_tmp = proc_create_data("reserve", 0666, prEntry_debug_info, &tp_reserve_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    // show abs_doze interface
    prEntry_tmp = proc_create_data("abs_doze", 0666, prEntry_debug_info, &tp_abs_doze_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    ts->prEntry_debug_tp = prEntry_debug_info;

    if (ts->ear_sense_support) {
        //proc files-step1:/proc/touchpanel/earsense
        prEntry_earsense = proc_mkdir("earsense", ts->prEntry_tp);
        if (prEntry_earsense == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create debug_info proc entry\n", __func__);
        }
        // show baseline for earsense
        prEntry_tmp = proc_create_data("rawdata", 0666, prEntry_earsense, &tp_earsense_rawdata_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
        // show delta for earsense
        prEntry_tmp = proc_create_data("delta", 0666, prEntry_earsense, &tp_earsense_delta_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
        // show self delta for earsense
        prEntry_tmp = proc_create_data("hover_selfdata", 0666, prEntry_earsense, &tp_earsense_selfdata_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
        // palm control for earsense
        prEntry_tmp = proc_create_data("palm_control", 0666, prEntry_earsense, &tp_earsense_palm_control_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
        // es_enable for earsense
        prEntry_tmp = proc_create_data("es_enable", 0666, prEntry_earsense, &tp_earsense_es_enable_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }

        // touch count for earsense
        prEntry_tmp = proc_create_data("es_touch_count", 0666, prEntry_earsense, &tp_earsense_es_touchcnt_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
    }

    if (ts->face_detect_support) {
        // proc for face detect
        prEntry_tmp = proc_create_data("fd_enable", 0666, ts->prEntry_tp, &tp_fd_enable_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }

        prEntry_tmp = proc_create_data("event_num", 0666, ts->prEntry_tp, &tp_event_num_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }

		prEntry_tmp = proc_create_data("fd_calibrate", 0666, ts->prEntry_tp, &tp_fd_calibrate_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
    }

	if (ts->lcd_refresh_rate_switch) {
		// proc for lcd_refresh_rate_switch
		prEntry_tmp = proc_create_data("lcd_refresh_rate_switch", 0666, ts->prEntry_tp, &tp_lcd_refresh_switch_fops, ts);
		if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
		}
	}

	if (ts->touch_hold_support) {
		// proc for touchhold switch
		prEntry_tmp = proc_create_data("touch_hold", 0666, ts->prEntry_tp, &tp_touch_hold_switch_fops, ts);
		if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
		}
		prEntry_tmp = proc_create_data("touch_area_switch", 0666, ts->prEntry_tp, &tp_touch_area_switch_fops, ts);
		if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
		}
	}

	// proc for lcd_refresh_rate_switch
	prEntry_tmp = proc_create_data("fingerprint_int_test", 0666, ts->prEntry_tp, &tp_fingerprint_int_test_fops, ts);
	if (prEntry_tmp == NULL) {
	ret = -ENOMEM;
	TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}
	// proc for charge detect
	prEntry_tmp = proc_create_data("charge_detect", 0666, ts->prEntry_tp, &tp_charge_detect_fops,ts);
	if(prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TPD_INFO("%s: Couldn鈥檛 create proc entey, %d\n", __func__, __LINE__);
	}
    return ret;
}

/**
 * init_input_device - Using for register input device
 * @ts: touchpanel_data struct using for common driver
 *
 * we should using this function setting input report capbility && register input device
 * Returning zero(success) or negative errno(failed)
 */
static int init_input_device(struct touchpanel_data *ts)
{
    int ret = 0;
    struct kobject *vk_properties_kobj;

    TPD_INFO("%s is called\n", __func__);
    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL) {
        ret = -ENOMEM;
        TPD_INFO("Failed to allocate input device\n");
        return ret;
    }

    ts->kpd_input_dev  = input_allocate_device();
    if (ts->kpd_input_dev == NULL) {
        ret = -ENOMEM;
        TPD_INFO("Failed to allocate key input device\n");
        return ret;
    }

    if (ts->face_detect_support) {
        ps_input_dev  = input_allocate_device();
        if (ps_input_dev == NULL) {
            ret = -ENOMEM;
            TPD_INFO("Failed to allocate ps input device\n");
            return ret;
        }

        ps_input_dev->name = TPD_DEVICE"_ps";
        set_bit(EV_MSC, ps_input_dev->evbit);
        set_bit(MSC_RAW, ps_input_dev->mscbit);
    }

    ts->input_dev->name = TPD_DEVICE;
    set_bit(EV_SYN, ts->input_dev->evbit);
    set_bit(EV_ABS, ts->input_dev->evbit);
    set_bit(EV_KEY, ts->input_dev->evbit);
    set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
    set_bit(ABS_MT_WIDTH_MAJOR, ts->input_dev->absbit);
    set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
    set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
    set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
    set_bit(BTN_TOUCH, ts->input_dev->keybit);
    if (ts->black_gesture_support) {
        set_bit(KEY_F4, ts->input_dev->keybit);
    }

    ts->kpd_input_dev->name = TPD_DEVICE"_kpd";
    set_bit(EV_KEY, ts->kpd_input_dev->evbit);
    set_bit(EV_SYN, ts->kpd_input_dev->evbit);

    switch(ts->vk_type) {
        case TYPE_PROPERTIES :
            {
                TPD_DEBUG("Type 1: using board_properties\n");
                vk_properties_kobj = kobject_create_and_add("board_properties", NULL);
                if (vk_properties_kobj)
                    ret = sysfs_create_group(vk_properties_kobj, &properties_attr_group);
                if (!vk_properties_kobj || ret)
                    TPD_DEBUG("failed to create board_properties\n");
                break;
            }
        case TYPE_AREA_SEPRATE:
            {
                TPD_DEBUG("Type 2:using same IC (button zone &&  touch zone are seprate)\n");
                if (CHK_BIT(ts->vk_bitmap, BIT_MENU))
                    set_bit(KEY_MENU, ts->kpd_input_dev->keybit);
                if (CHK_BIT(ts->vk_bitmap, BIT_HOME))
                    set_bit(KEY_HOMEPAGE, ts->kpd_input_dev->keybit);
                if (CHK_BIT(ts->vk_bitmap, BIT_BACK))
                    set_bit(KEY_BACK, ts->kpd_input_dev->keybit);
                break;
            }
        default :
            break;
    }

#ifdef TYPE_B_PROTOCOL
    input_mt_init_slots(ts->input_dev, ts->max_num, 0);
#endif
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->resolution_info.max_x, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->resolution_info.max_y, 0, 0);
    input_set_drvdata(ts->input_dev, ts);
    input_set_drvdata(ts->kpd_input_dev, ts);

    if (input_register_device(ts->input_dev)) {
        TPD_INFO("%s: Failed to register input device\n", __func__);
        input_free_device(ts->input_dev);
        return -1;
    }

    if (input_register_device(ts->kpd_input_dev)) {
        TPD_INFO("%s: Failed to register key input device\n", __func__);
        input_free_device(ts->kpd_input_dev);
        return -1;
    }

    if (ts->face_detect_support) {
        if (input_register_device(ps_input_dev)) {
            TPD_INFO("%s: Failed to register ps input device\n", __func__);
            input_free_device(ps_input_dev);
            return -1;
        }
    }

    return 0;
}

/**
 * init_parse_dts - parse dts, get resource defined in Dts
 * @dev: i2c_client->dev using to get device tree
 * @ts: touchpanel_data, using for common driver
 *
 * If there is any Resource needed by chip_data, we can add a call-back func in this function
 * Do not care the result : Returning void type
 */
static void init_parse_dts(struct device *dev, struct touchpanel_data *ts)
{
    int rc;
    struct device_node *np;
    int temp_array[8];
    int tx_rx_num[2];
    int val = 0;

    np = dev->of_node;

    ts->register_is_16bit       = of_property_read_bool(np, "register-is-16bit");
    ts->edge_limit_support      = of_property_read_bool(np, "edge_limit_support");
    ts->glove_mode_support      = of_property_read_bool(np, "glove_mode_support");
    ts->esd_handle_support      = of_property_read_bool(np, "esd_handle_support");
    ts->spurious_fp_support     = of_property_read_bool(np, "spurious_fingerprint_support");
    ts->charger_pump_support    = of_property_read_bool(np, "charger_pump_support");
    ts->black_gesture_support   = of_property_read_bool(np, "black_gesture_support");
    ts->gesture_test_support    = of_property_read_bool(np, "black_gesture_test_support");
    ts->fw_update_app_support   = of_property_read_bool(np, "fw_update_app_support");
    ts->game_switch_support     = of_property_read_bool(np, "game_switch_support");
    ts->ear_sense_support       = of_property_read_bool(np, "ear_sense_support");
    ts->smart_gesture_support   = of_property_read_bool(np, "smart_gesture_support");
    ts->is_noflash_ic           = of_property_read_bool(np, "noflash_support");
    ts->face_detect_support     = of_property_read_bool(np, "face_detect_support");
    ts->lcd_refresh_rate_switch = of_property_read_bool(np, "lcd_refresh_rate_switch");
    ts->touch_hold_support 		= of_property_read_bool(np, "touch_hold_support");
	ts->ctl_base_address		= of_property_read_bool(np, "ctrl_base_change");

	ts->charge_detect_support	= of_property_read_bool(np, "charge_detect_support");
	ts->module_id_support		= of_property_read_bool(np, "module_id_support");
    ts->spuri_fp_touch.lcd_trigger_fp_check = of_property_read_bool(np, "lcd_trigger_fp_check");
    ts->spuri_fp_touch.lcd_resume_ok = true;

	rc = of_property_read_string(np, "project-name", &ts->panel_data.project_name);
	if (rc < 0) {
		TPD_INFO("failed to get project name, firmware/limit name will be invalid\n");
	}
	rc = of_property_read_string(np, "chip-name", &ts->panel_data.chip_name);
	if (rc < 0) {
		TPD_INFO("failed to get chip name, firmware/limit name will be invalid\n");
	}
	rc = of_property_read_u32(np, "module_id" , &ts->panel_data.tp_type);
	if(rc < 0) {
		TPD_INFO("module id is not specified\n");
		ts->panel_data.tp_type = 0;
	}

    rc = of_property_read_u32(np, "vdd_2v8_volt", &ts->hw_res.vdd_volt);
    if (rc < 0) {
        ts->hw_res.vdd_volt = 0;
        TPD_INFO("vdd_2v8_volt not defined\n");
    }

    // irq gpio
    ts->hw_res.irq_gpio = of_get_named_gpio_flags(np, "irq-gpio", 0, &(ts->irq_flags));
    if (gpio_is_valid(ts->hw_res.irq_gpio)) {
        rc= gpio_request(ts->hw_res.irq_gpio, "tp_irq_gpio");
        if (rc) {
            TPD_INFO("unable to request gpio [%d]\n", ts->hw_res.irq_gpio);
        }
    } else {
        TPD_INFO("irq-gpio not specified in dts\n");
    }

	ts->irq = gpio_to_irq(ts->hw_res.irq_gpio);
	ts->client->irq = ts->irq;
    // reset gpio
    ts->hw_res.reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
    if (gpio_is_valid(ts->hw_res.reset_gpio)) {
        rc = gpio_request(ts->hw_res.reset_gpio, "reset-gpio");
        if (rc)
            TPD_INFO("unable to request gpio [%d]\n", ts->hw_res.reset_gpio);
    } else {
        TPD_INFO("ts->reset-gpio not specified\n");
    }

    TPD_INFO("%s : irq_gpio = %d, irq_flags = 0x%x ts_irq = %d, reset_gpio = %d\n",
            __func__, ts->hw_res.irq_gpio, ts->irq_flags, ts->irq, ts->hw_res.reset_gpio);

    // tp type gpio
    ts->hw_res.id1_gpio = of_get_named_gpio(np, "id1-gpio", 0);
    if (gpio_is_valid(ts->hw_res.id1_gpio)) {
        rc = gpio_request(ts->hw_res.id1_gpio, "TP_ID1");
        if (rc)
            TPD_INFO("unable to request gpio [%d]\n", ts->hw_res.id1_gpio);
    } else {
        TPD_INFO("id1_gpio not specified\n");
    }

    ts->hw_res.id2_gpio = of_get_named_gpio(np, "id2-gpio", 0);
    if (gpio_is_valid(ts->hw_res.id2_gpio)) {
        rc = gpio_request(ts->hw_res.id2_gpio, "TP_ID2");
        if (rc)
            TPD_INFO("unable to request gpio [%d]\n", ts->hw_res.id2_gpio);
    } else {
        TPD_INFO("id2_gpio not specified\n");
    }

    ts->hw_res.id3_gpio = of_get_named_gpio(np, "id3-gpio", 0);
    if (gpio_is_valid(ts->hw_res.id3_gpio)) {
        rc = gpio_request(ts->hw_res.id3_gpio, "TP_ID3");
        if (rc)
            TPD_INFO("unable to request gpio [%d]\n", ts->hw_res.id3_gpio);
    } else {
        TPD_INFO("id3_gpio not specified\n");
    }

    ts->hw_res.pinctrl= devm_pinctrl_get(dev);
    if (IS_ERR_OR_NULL(ts->hw_res.pinctrl)) {
       TPD_INFO("Getting pinctrl handle failed");
    } else {
        ts->hw_res.pin_set_high = pinctrl_lookup_state(ts->hw_res.pinctrl, "pin_set_high");
        if (IS_ERR_OR_NULL(ts->hw_res.pin_set_high)) {
            TPD_INFO("Failed to get the high state pinctrl handle \n");
        }
        ts->hw_res.pin_set_low = pinctrl_lookup_state(ts->hw_res.pinctrl, "pin_set_low");
        if (IS_ERR_OR_NULL(ts->hw_res.pin_set_low)) {
            TPD_INFO(" Failed to get the low state pinctrl handle\n");
        }
    }
    ts->hw_res.enable2v8_gpio = of_get_named_gpio(np, "enable2v8_gpio", 0);
    if (ts->hw_res.enable2v8_gpio < 0) {
        TPD_INFO("ts->hw_res.enable2v8_gpio not specified\n");
    } else {
        if (gpio_is_valid(ts->hw_res.enable2v8_gpio)) {
            rc = gpio_request(ts->hw_res.enable2v8_gpio, "vdd2v8-gpio");
            if (rc) {
                TPD_INFO("unable to request gpio [%d] %d\n", ts->hw_res.enable2v8_gpio, rc);
            }
        }
    }

    ts->hw_res.enable1v8_gpio = of_get_named_gpio(np, "enable1v8_gpio", 0);
    if (ts->hw_res.enable1v8_gpio < 0) {
        TPD_INFO("ts->hw_res.enable1v8_gpio not specified\n");
    } else {
        if (gpio_is_valid(ts->hw_res.enable1v8_gpio)) {
            rc = gpio_request(ts->hw_res.enable1v8_gpio, "vcc1v8-gpio");
            if (rc) {
                TPD_INFO("unable to request gpio [%d], %d\n", ts->hw_res.enable1v8_gpio, rc);
            }
        }
    }

    // interrupt mode
    ts->int_mode = BANNABLE;
    rc = of_property_read_u32(np, "touchpanel,int-mode", &val);
    if (rc) {
        TPD_INFO("int-mode not specified\n");
    } else {
        if (val < INTERRUPT_MODE_MAX) {
            ts->int_mode = val;
        }
    }

    // resolution info
    rc = of_property_read_u32(np, "touchpanel,max-num-support", &ts->max_num);
    if (rc) {
        TPD_INFO("ts->max_num not specified\n");
        ts->max_num = 10;
    }

    rc = of_property_read_u32_array(np, "touchpanel,tx-rx-num", tx_rx_num, 2);
    if (rc) {
        TPD_INFO("tx-rx-num not set\n");
        ts->hw_res.TX_NUM = 0;
        ts->hw_res.RX_NUM = 0;
    } else {
        ts->hw_res.TX_NUM = tx_rx_num[0];
        ts->hw_res.RX_NUM = tx_rx_num[1];
    }
    TPD_INFO("TX_NUM = %d, RX_NUM = %d \n", ts->hw_res.TX_NUM, ts->hw_res.RX_NUM);

    rc = of_property_read_u32_array(np, "earsense,tx-rx-num", tx_rx_num, 2);
    if (rc) {
        TPD_INFO("tx-rx-num not set\n");
        ts->hw_res.EARSENSE_TX_NUM = ts->hw_res.TX_NUM;
        ts->hw_res.EARSENSE_RX_NUM = ts->hw_res.RX_NUM / 2;
    } else {
        ts->hw_res.EARSENSE_TX_NUM = tx_rx_num[0];
        ts->hw_res.EARSENSE_RX_NUM = tx_rx_num[1];
    }
    TPD_INFO("EARSENSE_TX_NUM = %d, EARSENSE_RX_NUM = %d \n", ts->hw_res.EARSENSE_TX_NUM, ts->hw_res.EARSENSE_RX_NUM);

    rc = of_property_read_u32_array(np, "touchpanel,display-coords", temp_array, 2);
    if (rc) {
        TPD_INFO("Lcd size not set\n");
        ts->resolution_info.LCD_WIDTH = 0;
        ts->resolution_info.LCD_HEIGHT = 0;
    }else{
        ts->resolution_info.LCD_WIDTH = temp_array[0];
        ts->resolution_info.LCD_HEIGHT = temp_array[1];
    }

    rc = of_property_read_u32_array(np, "touchpanel,panel-coords", temp_array, 2);
    if (rc) {
        ts->resolution_info.max_x = 0;
        ts->resolution_info.max_y = 0;
    }else{
        ts->resolution_info.max_x = temp_array[0];
        ts->resolution_info.max_y = temp_array[1];
    }
    rc = of_property_read_u32_array(np, "touchpanel,touchmajor-limit", temp_array, 2);
    if (rc) {
        ts->touch_major_limit.width_range = 0;
        ts->touch_major_limit.height_range = 0;
    }else{
        ts->touch_major_limit.width_range = temp_array[0];
        ts->touch_major_limit.height_range = temp_array[1];
    }
    TPD_INFO("LCD_WIDTH = %d, LCD_HEIGHT = %d, max_x = %d, max_y = %d, limit_witdh = %d, limit_height = %d\n",
            ts->resolution_info.LCD_WIDTH, ts->resolution_info.LCD_HEIGHT, ts->resolution_info.max_x, ts->resolution_info.max_y,\
            ts->touch_major_limit.width_range, ts->touch_major_limit.height_range);

    // virturl key Related
    rc = of_property_read_u32_array(np, "touchpanel,button-type", temp_array, 2);
    if (rc < 0) {
        TPD_INFO("error:button-type should be setting in dts!");
    } else {
        ts->vk_type = temp_array[0];
        ts->vk_bitmap = temp_array[1] & 0xFF;
        if (ts->vk_type == TYPE_PROPERTIES) {
            rc = of_property_read_u32_array(np, "touchpanel,button-map", temp_array, 8);
            if (rc) {
                TPD_INFO("button-map not set\n");
            }else{
                ts->button_map.coord_menu.x = temp_array[0];
                ts->button_map.coord_menu.y = temp_array[1];
                ts->button_map.coord_home.x = temp_array[2];
                ts->button_map.coord_home.y = temp_array[3];
                ts->button_map.coord_back.x = temp_array[4];
                ts->button_map.coord_back.y = temp_array[5];
                ts->button_map.width_x = temp_array[6];
                ts->button_map.height_y = temp_array[7];
            }
        }
    }

    //touchkey take tx num and rx num
    rc = of_property_read_u32_array(np, "touchpanel.button-TRx", temp_array, 2);
    if(rc < 0) {
        TPD_INFO("error:button-TRx should be setting in dts!\n");
        ts->hw_res.key_TX = 0;
        ts->hw_res.key_RX = 0;
    } else {
        ts->hw_res.key_TX = temp_array[0];
        ts->hw_res.key_RX = temp_array[1];
        TPD_INFO("key_tx is %d, key_rx is %d\n", ts->hw_res.key_TX, ts->hw_res.key_RX);
    }

    //set incell panel parameter, for of_property_read_bool return 1 when success and return 0 when item is not exist
    rc = ts->is_incell_panel = of_property_read_bool(np, "incell_screen");
    if(rc > 0) {
        TPD_INFO("panel is incell!\n");
        ts->is_incell_panel = 1;
    } else {
        TPD_INFO("panel is oncell!\n");
        ts->is_incell_panel = 0;
    }

    // We can Add callback fuction here if necessary seprate some dts config for chip_data
}

int init_power_control(struct touchpanel_data *ts)
{
    int ret = 0;

    // 1.8v
    ts->hw_res.vcc_1v8 = regulator_get(ts->dev, "vcc_1v8");
    if (IS_ERR_OR_NULL(ts->hw_res.vcc_1v8)) {
        TPD_INFO("Regulator get failed vcc_1v8, ret = %d\n", ret);
    } else {
        if (regulator_count_voltages(ts->hw_res.vcc_1v8) > 0) {
            ret = regulator_set_voltage(ts->hw_res.vcc_1v8, 1800000, 1800000);
            if (ret) {
                dev_err(ts->dev, "Regulator set_vtg failed vcc_i2c rc = %d\n", ret);
                goto regulator_vcc_1v8_put;
            }

            ret = regulator_set_load(ts->hw_res.vcc_1v8, 200000);
            if (ret < 0) {
                dev_err(ts->dev, "Failed to set vcc_1v8 mode(rc:%d)\n", ret);
                goto regulator_vcc_1v8_put;
            }
        }
    }
    // vdd 2.8v
    ts->hw_res.vdd_2v8 = regulator_get(ts->dev, "vdd_2v8");
    if (IS_ERR_OR_NULL(ts->hw_res.vdd_2v8)) {
        TPD_INFO("Regulator vdd2v8 get failed, ret = %d\n", ret);
    } else {
        if (regulator_count_voltages(ts->hw_res.vdd_2v8) > 0) {
            TPD_INFO("set avdd voltage to %d uV\n", ts->hw_res.vdd_volt);
            if (ts->hw_res.vdd_volt) {
                ret = regulator_set_voltage(ts->hw_res.vdd_2v8, ts->hw_res.vdd_volt, ts->hw_res.vdd_volt);
            } else {
                ret = regulator_set_voltage(ts->hw_res.vdd_2v8, 3100000, 3100000);
            }
            if (ret) {
                dev_err(ts->dev, "Regulator set_vtg failed vdd rc = %d\n", ret);
                goto regulator_vdd_2v8_put;
            }

            ret = regulator_set_load(ts->hw_res.vdd_2v8, 200000);
            if (ret < 0) {
                dev_err(ts->dev, "Failed to set vdd_2v8 mode(rc:%d)\n", ret);
                goto regulator_vdd_2v8_put;
            }
        }
    }

    return 0;

regulator_vdd_2v8_put:
    regulator_put(ts->hw_res.vdd_2v8);
    ts->hw_res.vdd_2v8 = NULL;
regulator_vcc_1v8_put:
    if (!IS_ERR_OR_NULL(ts->hw_res.vcc_1v8)) {
        regulator_put(ts->hw_res.vcc_1v8);
        ts->hw_res.vcc_1v8 = NULL;
    }

    return ret;
}

int tp_powercontrol_1v8(struct hw_resource *hw_res, bool on)
{
    int ret = 0;

    if (on) {// 1v8 power on
        if (!IS_ERR_OR_NULL(hw_res->vcc_1v8)) {
            TPD_INFO("Enable the Regulator1v8.\n");
            ret = regulator_enable(hw_res->vcc_1v8);
            if (ret) {
                TPD_INFO("Regulator vcc_i2c enable failed ret = %d\n", ret);
                return ret;
            }
        }

        if (hw_res->enable1v8_gpio > 0) {
            TPD_INFO("Enable the 1v8_gpio\n");
            ret = gpio_direction_output(hw_res->enable1v8_gpio, 1);
            if (ret) {
                TPD_INFO("enable the enable1v8_gpio failed.\n");
                return ret;
            }
        }
    } else {// 1v8 power off
        if (!IS_ERR_OR_NULL(hw_res->vcc_1v8)) {
            ret = regulator_disable(hw_res->vcc_1v8);
            if (ret) {
                TPD_INFO("Regulator vcc_i2c enable failed rc = %d\n", ret);
                return ret;
            }
        }

        if (hw_res->enable1v8_gpio > 0) {
            TPD_INFO("disable the 1v8_gpio\n");
            ret = gpio_direction_output(hw_res->enable1v8_gpio, 0);
            if (ret) {
                TPD_INFO("disable the enable1v8_gpio failed.\n");
                return ret;
            }
        }
    }

    return 0;
}

int tp_powercontrol_2v8(struct hw_resource *hw_res, bool on)
{
    int ret = 0;

    if (on) {// 2v8 power on
        if (!IS_ERR_OR_NULL(hw_res->vdd_2v8)) {
            TPD_INFO("Enable the Regulator2v8.\n");
            ret = regulator_enable(hw_res->vdd_2v8);
            if (ret) {
                TPD_INFO("Regulator vdd enable failed ret = %d\n", ret);
                return ret;
            }
        }
        if (hw_res->enable2v8_gpio > 0) {
            TPD_INFO("Enable the 2v8_gpio, hw_res->enable2v8_gpio is %d\n", hw_res->enable2v8_gpio);
            ret = gpio_direction_output(hw_res->enable2v8_gpio, 1);
            if (ret) {
                TPD_INFO("enable the enable2v8_gpio failed.\n");
                return ret;
            }
        }
    } else {// 2v8 power off
        if (!IS_ERR_OR_NULL(hw_res->vdd_2v8)) {
			TPD_INFO("disable the vdd_2v8\n");
            ret = regulator_disable(hw_res->vdd_2v8);
            if (ret) {
                TPD_INFO("Regulator vdd disable failed rc = %d\n", ret);
                return ret;
            }
        }
        if (hw_res->enable2v8_gpio > 0) {
            TPD_INFO("disable the 2v8_gpio\n");
            ret = gpio_direction_output(hw_res->enable2v8_gpio, 0);
            if (ret) {
                TPD_INFO("disable the enable2v8_gpio failed.\n");
                return ret;
            }
        }
    }
    return ret;
}


static void esd_handle_func(struct work_struct *work)
{
    int ret = 0;
    struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
            esd_info.esd_check_work.work);

    if (ts->loading_fw) {
        TPD_INFO("FW is updating, stop esd handle!\n");
        return;
    }

    mutex_lock(&ts->esd_info.esd_lock);
    if (!ts->esd_info.esd_running_flag) {
        TPD_INFO("Esd protector has stopped!\n");
        goto ESD_END;
    }

    if (ts->is_suspended == 1) {
        TPD_INFO("Touch panel has suspended!\n");
        goto ESD_END;
    }

    if (!ts->ts_ops->esd_handle) {
        TPD_INFO("not support ts_ops->esd_handle callback\n");
        goto ESD_END;
    }

    ret = ts->ts_ops->esd_handle(ts->chip_data);
    if (ret == -1) {    //-1 means esd hanppened: handled in IC part, recovery the state here
         operate_mode_switch(ts);
    }

    if (ts->esd_info.esd_running_flag)
        queue_delayed_work(ts->esd_info.esd_workqueue, &ts->esd_info.esd_check_work, ts->esd_info.esd_work_time);
    else
        TPD_INFO("Esd protector suspended!");

ESD_END:
    mutex_unlock(&ts->esd_info.esd_lock);
    return;
}

/**
 * esd_handle_switch - open or close esd thread
 * @esd_info: touchpanel_data, using for common driver resource
 * @on: bool variable using for  indicating open or close esd check function.
 *     true:open;
 *     false:close;
 */
void esd_handle_switch(struct esd_information *esd_info, bool on)
{
    mutex_lock(&esd_info->esd_lock);

    if (on) {
        if (!esd_info->esd_running_flag) {
            esd_info->esd_running_flag = 1;

            TPD_INFO("Esd protector started, cycle: %d s\n", esd_info->esd_work_time/HZ);
            queue_delayed_work(esd_info->esd_workqueue, &esd_info->esd_check_work, esd_info->esd_work_time);
        }
    } else {
        if (esd_info->esd_running_flag) {
            esd_info->esd_running_flag = 0;

            TPD_INFO("Esd protector stoped!\n");
            cancel_delayed_work(&esd_info->esd_check_work);
        }
    }

    mutex_unlock(&esd_info->esd_lock);
}

int tp_register_irq_func(struct touchpanel_data *ts)
{
    int ret = 0;
	
#ifdef TPD_USE_EINT
    if (gpio_is_valid(ts->hw_res.irq_gpio)) {
        TPD_DEBUG("%s, irq_gpio is %d, ts->irq is %d\n", __func__, ts->hw_res.irq_gpio, ts->irq);
        ret = request_threaded_irq(ts->irq, NULL,
                tp_irq_thread_fn,
                ts->irq_flags | IRQF_ONESHOT,
                TPD_DEVICE, ts);
        if (ret < 0) {
            TPD_INFO("%s request_threaded_irq ret is %d\n", __func__, ret);
        }
    } else {
        TPD_INFO("%s:no valid irq\n", __func__);
    }
#else
    hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    ts->timer.function = touchpanel_timer_func;
    hrtimer_start(&ts->timer, ktime_set(3, 0), HRTIMER_MODE_REL);
#endif

    return ret;
}

//work schdule for reading&update delta
static void touch_read_delta(struct work_struct *work)
{
    struct touchpanel_data *ts = container_of(work, struct touchpanel_data, read_delta_work);

    mutex_lock(&ts->mutex_earsense);
    mutex_lock(&ts->mutex);
    ts->earsense_ops->delta_read(ts->chip_data, ts->earsense_delta, 2 * ts->hw_res.EARSENSE_TX_NUM * ts->hw_res.EARSENSE_RX_NUM);
    mutex_unlock(&ts->mutex);
    mutex_unlock(&ts->mutex_earsense);
    ts->delta_state=TYPE_DELTA_IDLE;
}

void tp_util_get_vendor(struct touchpanel_data *ts, struct panel_info *panel_data)
{

	panel_data->test_limit_name = kzalloc(MAX_LIMIT_DATA_LENGTH, GFP_KERNEL);
	if (panel_data->test_limit_name == NULL) {
		TPD_INFO("panel_data.test_limit_name kzalloc error\n");
	}

	if (ts->module_id_support) {
		if (lcd_id) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			"tp/FW_%s_%s_NEW.img", panel_data->project_name, panel_data->chip_name);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				"tp/LIMIT_%s_%s_NEW.img", panel_data->project_name, panel_data->chip_name);
			}
		} else {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			"tp/FW_%s_%s.img", panel_data->project_name, panel_data->chip_name);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				"tp/LIMIT_%s_%s.img", panel_data->project_name, panel_data->chip_name);
			}
		}
		ts->tx_change_order = lcd_id;
	} else {
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
		"tp/FW_%s_%s.img", panel_data->project_name, panel_data->chip_name);

		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
			"tp/LIMIT_%s_%s.img", panel_data->project_name, panel_data->chip_name);
		}
	}

	sprintf(panel_data->manufacture_info.version, "0x%x", panel_data->TP_FW);
	sprintf(panel_data->manufacture_info.manufacture, panel_data->chip_name);
	push_component_info(TOUCH_KEY, panel_data->manufacture_info.version, panel_data->manufacture_info.manufacture);
	push_component_info(TP, panel_data->manufacture_info.version, panel_data->manufacture_info.manufacture);

	TPD_INFO("%s fw:%s limit:%s\n", __func__, panel_data->fw_name, panel_data->test_limit_name);
}


void sec_ts_pinctrl_configure(struct hw_resource *hw_res, bool enable)
{
	int ret;

	if (enable) {
		if (hw_res->pinctrl) {
			ret = pinctrl_select_state(hw_res->pinctrl, hw_res->pin_set_high);
			if (ret)
				TPD_INFO("%s could not set active pinstate", __func__);
		}
	} else {
		if (hw_res->pinctrl) {
			ret = pinctrl_select_state(hw_res->pinctrl, hw_res->pin_set_low);
			if (ret)
				TPD_INFO("%s could not set suspend pinstate", __func__);
		}
	}
}


/**
 * register_common_touch_device - parse dts, get resource defined in Dts
 * @pdata: touchpanel_data, using for common driver
 *
 * entrance of common touch Driver
 * Returning zero(sucess) or negative errno(failed)
 */

int register_common_touch_device(struct touchpanel_data *pdata)
{
    struct touchpanel_data *ts = pdata;
    struct invoke_method *invoke;

    int ret = -1;

    TPD_INFO("%s  is called\n", __func__);

	//step : FTM process
	ts->boot_mode = get_boot_mode();
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
		if ((ts->boot_mode == META_BOOT || ts->boot_mode == FACTORY_BOOT))
#else
		if ((ts->boot_mode == MSM_BOOT_MODE__FACTORY || ts->boot_mode == MSM_BOOT_MODE__RF || ts->boot_mode == MSM_BOOT_MODE__WLAN || ts->boot_mode == MSM_BOOT_MODE__CHARGE))
#endif
		{
			TPD_INFO("%s: not int normal mode, return.\n", __func__);
			return -1;
		}
    //step1 : dts parse
    init_parse_dts(ts->dev, ts);

    //step2 : IIC interfaces init
    init_touch_interfaces(ts->dev, ts->register_is_16bit);

    //step3 : mutex init
    mutex_init(&ts->mutex);
    init_completion(&ts->pm_complete);
    init_completion(&ts->fw_complete);
    init_completion(&ts->resume_complete);

    ts->async_workqueue = create_singlethread_workqueue("tp_async");
    if (!ts->async_workqueue) {
        ret = -ENOMEM;
        return -1;
    }
    INIT_WORK(&ts->async_work, tp_async_work_lock);

    if (ts->has_callback) {
        invoke = (struct invoke_method *)pdata->chip_data;
        invoke->invoke_common = tp_work_common_callback;
        invoke->async_work = tp_async_work_callback;
    }
	sec_ts_pinctrl_configure(&ts->hw_res, true);
    //step4 : Power init && setting
    preconfig_power_control(ts);
    ret = init_power_control(ts);
    if (ret) {
        TPD_INFO("%s: tp power init failed.\n", __func__);
        return -1;
    }
    ret = reconfig_power_control(ts);
    if (ret) {
        TPD_INFO("%s: reconfig power failed.\n", __func__);
        return -1;
    }
    if (!ts->ts_ops->power_control) {
        ret = -EINVAL;
        TPD_INFO("tp power_control NULL!\n");
        goto power_control_failed;
    }
    ret = ts->ts_ops->power_control(ts->chip_data, true);
    if (ret) {
        TPD_INFO("%s: tp power init failed.\n", __func__);
        goto power_control_failed;
    }
    //step5 : I2C function check
    if (!ts->is_noflash_ic) {
        if (!i2c_check_functionality(ts->client->adapter, I2C_FUNC_I2C)) {
            TPD_INFO("%s: need I2C_FUNC_I2C\n", __func__);
            ret = -ENODEV;
            goto err_check_functionality_failed;
        }
    }

    //step6 : touch input dev init
    ret = init_input_device(ts);
    if (ret < 0) {
        ret = -EINVAL;
        TPD_INFO("tp_input_init failed!\n");
        goto err_check_functionality_failed;
    }

    if (ts->int_mode == UNBANNABLE) {
        ret = tp_register_irq_func(ts);
        if (ret < 0) {
            goto free_touch_panel_input;
        }
    }

    //step8 : Alloc fw_name/devinfo memory space
    ts->panel_data.fw_name = kzalloc(MAX_FW_NAME_LENGTH, GFP_KERNEL);
    if (ts->panel_data.fw_name == NULL) {
        ret = -ENOMEM;
        TPD_INFO("panel_data.fw_name kzalloc error\n");
        goto free_touch_panel_input;
    }

    ts->panel_data.manufacture_info.version = kzalloc(MAX_DEVICE_VERSION_LENGTH, GFP_KERNEL);
    if (ts->panel_data.manufacture_info.version == NULL) {
        ret = -ENOMEM;
        TPD_INFO("manufacture_info.version kzalloc error\n");
        goto manu_version_alloc_err;
    }

    ts->panel_data.manufacture_info.manufacture = kzalloc(MAX_DEVICE_MANU_LENGTH, GFP_KERNEL);
    if (ts->panel_data.manufacture_info.manufacture == NULL) {
        ret = -ENOMEM;
        TPD_INFO("panel_data.fw_name kzalloc error\n");
        goto manu_info_alloc_err;
    }

    //step8 : touchpanel vendor
    tp_util_get_vendor(ts, &ts->panel_data);
	if (ts->ts_ops->get_vendor) {
		ts->ts_ops->get_vendor(ts->chip_data, &ts->panel_data);
	}

    //step10:get chip info
    if (!ts->ts_ops->get_chip_info) {
        ret = -EINVAL;
        TPD_INFO("tp get_chip_info NULL!\n");
        goto err_check_functionality_failed;
    }
    ret = ts->ts_ops->get_chip_info(ts->chip_data);
    if (ret < 0) {
        ret = -EINVAL;
        TPD_INFO("tp get_chip_info failed!\n");
        goto err_check_functionality_failed;
    }

    //step11 : touchpanel Fw check
    if(!ts->is_noflash_ic) {            //noflash don't have firmware before fw update
        if (!ts->ts_ops->fw_check) {
                ret = -EINVAL;
                TPD_INFO("tp fw_check NULL!\n");
                goto manu_info_alloc_err;
        }
        ret = ts->ts_ops->fw_check(ts->chip_data, &ts->resolution_info, &ts->panel_data);
        if (ret == FW_ABNORMAL) {
                ts->force_update = 1;
                TPD_INFO("This FW need to be updated!\n");
        } else {
                ts->force_update = 0;
        }
    }

    //step12 : enable touch ic irq output ability
    if (!ts->ts_ops->mode_switch) {
        ret = -EINVAL;
        TPD_INFO("tp mode_switch NULL!\n");
        goto manu_info_alloc_err;
    }
    ret = ts->ts_ops->mode_switch(ts->chip_data, MODE_NORMAL, true);
    if (ret < 0) {
        ret = -EINVAL;
        TPD_INFO("%s:modem switch failed!\n", __func__);
        goto manu_info_alloc_err;
    }

	wakeup_source_init(&ts->source, "tp_syna");
	//step13 : irq request setting
    if (ts->int_mode == BANNABLE) {
        ret = tp_register_irq_func(ts);
        if (ret < 0) {
            goto manu_info_alloc_err;
        }
    }

    //step14 : suspend && resume fuction register
#if defined(CONFIG_DRM_MSM)
    ts->fb_notif.notifier_call = fb_notifier_callback;
    ret = msm_drm_register_client(&ts->fb_notif);
    if (ret) {
        TPD_INFO("Unable to register fb_notifier: %d\n", ret);
    }
#elif defined(CONFIG_FB)
    ts->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&ts->fb_notif);
    if (ret) {
        TPD_INFO("Unable to register fb_notifier: %d\n", ret);
    }
#endif/*CONFIG_FB*/

    //step15 : workqueue create(speedup_resume)
    ts->speedup_resume_wq = create_singlethread_workqueue("speedup_resume_wq");
    if (!ts->speedup_resume_wq) {
        ret = -ENOMEM;
        goto threaded_irq_free;
    }

    INIT_WORK(&ts->speed_up_work, speedup_resume);
    INIT_WORK(&ts->fw_update_work, tp_fw_update_work);

    //step 16 : short edge shield
    if (ts->edge_limit_support) {
        ts->limit_enable = 1;
        ts->limit_edge = ts->limit_enable & 1;
        ts->limit_corner = 0;
        ts->edge_limit.limit_area = 1;
        ts->edge_limit.in_which_area = AREA_NOTOUCH;

        ts->edge_limit.left_x1  = (ts->edge_limit.limit_area * 1000)/100;
        ts->edge_limit.right_x1 = ts->resolution_info.LCD_WIDTH - ts->edge_limit.left_x1;
        ts->edge_limit.left_x2  = 2 * ts->edge_limit.left_x1;
        ts->edge_limit.right_x2 = ts->resolution_info.LCD_WIDTH - (2 * ts->edge_limit.left_x1);
        ts->edge_limit.left_x3  = 5 * ts->edge_limit.left_x1;
        ts->edge_limit.right_x3 = ts->resolution_info.LCD_WIDTH - (5 * ts->edge_limit.left_x1);

        ts->edge_limit.left_y1  = (ts->edge_limit.limit_area * 1000)/100;
        ts->edge_limit.right_y1 = ts->resolution_info.LCD_HEIGHT - ts->edge_limit.left_y1;
        ts->edge_limit.left_y2  = 2 * ts->edge_limit.left_y1;
        ts->edge_limit.right_y2 = ts->resolution_info.LCD_HEIGHT - (2 * ts->edge_limit.left_y1);
        ts->edge_limit.left_y3  = 5 * ts->edge_limit.left_y1;
        ts->edge_limit.right_y3 = ts->resolution_info.LCD_HEIGHT - (5 * ts->edge_limit.left_y1);
    }

    //step 17:esd recover support
    if (ts->esd_handle_support) {
        ts->esd_info.esd_workqueue = create_singlethread_workqueue("esd_workthread");
        INIT_DELAYED_WORK(&ts->esd_info.esd_check_work, esd_handle_func);

        mutex_init(&ts->esd_info.esd_lock);

        ts->esd_info.esd_running_flag = 0;
        ts->esd_info.esd_work_time = 2 * HZ; // HZ: clock ticks in 1 second generated by system
        TPD_INFO("Clock ticks for an esd cycle: %d\n", ts->esd_info.esd_work_time);

        esd_handle_switch(&ts->esd_info, true);
    }

    //step 18:spurious_fingerprint support
    if (ts->spurious_fp_support) {
        ts->spuri_fp_touch.thread = kthread_run(finger_protect_handler, ts, "touchpanel_fp");
        if (IS_ERR(ts->spuri_fp_touch.thread)) {
            TPD_INFO("spurious fingerprint thread create failed\n");
        }
    }

    //step 19: charge pump support
    if (ts->charger_pump_support) {
        if (ts->ts_ops->get_usb_state) {
            ts->is_usb_checked = !!ts->ts_ops->get_usb_state();
        } else {
            ts->is_usb_checked = 0;
        }
        TPD_INFO("curent usb state is %d\n", ts->is_usb_checked);
        ret = ts->ts_ops->mode_switch(ts->chip_data, MODE_CHARGE, ts->is_usb_checked);
        if (ret < 0) {
            TPD_INFO("switch charge mode failed\n");
        }
    }

    // step 20: ear sense support
    if (ts->ear_sense_support) {
        mutex_init(&ts->mutex_earsense);    // init earsense operate mutex

        //malloc space for storing earsense delta
        ts->earsense_delta = kzalloc(2 * ts->hw_res.EARSENSE_TX_NUM * ts->hw_res.EARSENSE_RX_NUM, GFP_KERNEL);
        if (ts->earsense_delta == NULL) {
            ret = -ENOMEM;
            TPD_INFO("earsense_delta kzalloc error\n");
            goto threaded_irq_free;
        }

        //create work queue for read earsense delta
        ts->delta_read_wq = create_singlethread_workqueue("touch_delta_wq");
        if (!ts->delta_read_wq) {
            ret = -ENOMEM;
            goto earsense_alloc_free;
        }
        INIT_WORK(&ts->read_delta_work, touch_read_delta);
    }

    //step 21 : createproc proc files interface
    init_touchpanel_proc(ts);

    //step 22 : Other****
    ts->i2c_ready = true;
    ts->loading_fw = false;
    ts->is_suspended = 0;
    ts->suspend_state = TP_SPEEDUP_RESUME_COMPLETE;
    ts->gesture_enable = 0;
    ts->es_enable = 0;
    ts->fd_enable = 0;
    ts->palm_enable = 1;
    ts->touch_count = 0;
    ts->glove_enable = 0;
    ts->view_area_touched = 0;
    ts->tp_suspend_order = LCD_TP_SUSPEND;
    ts->tp_resume_order = TP_LCD_RESUME;
    ts->skip_suspend_operate = false;
    ts->skip_reset_in_resume = false;
    ts->irq_slot = 0;
	ts->touch_hold_enable = 0;
	ts->lcd_refresh_rate = 0;
	ts->reject_point = 0;
	ts->charge_detect = 0;
    ts->firmware_update_type = 0;
    ts->corner_delay_up = -1;
    if(ts->is_noflash_ic) {
        ts->irq = ts->s_client->irq;
    } else {
        ts->irq = ts->client->irq;
    }
    tp_register_times++;
    g_tp = ts;
    complete(&ts->pm_complete);
    TPD_INFO("Touch panel probe : normal end\n");
    return 0;

earsense_alloc_free:
    kfree(ts->earsense_delta);

threaded_irq_free:
    free_irq(ts->irq, ts);

manu_info_alloc_err:
    kfree(ts->panel_data.manufacture_info.version);

manu_version_alloc_err:
    kfree(ts->panel_data.fw_name);

free_touch_panel_input:
    input_unregister_device(ts->input_dev);
    input_unregister_device(ts->kpd_input_dev);
	input_unregister_device(ps_input_dev);
	

err_check_functionality_failed:
    ts->ts_ops->power_control(ts->chip_data, false);

power_control_failed:

    if (!IS_ERR_OR_NULL(ts->hw_res.vdd_2v8)) {
		regulator_disable(ts->hw_res.vdd_2v8);
        regulator_put(ts->hw_res.vdd_2v8);
        ts->hw_res.vdd_2v8 = NULL;
    }

    if (!IS_ERR_OR_NULL(ts->hw_res.vcc_1v8)) {
		regulator_disable(ts->hw_res.vcc_1v8);
        regulator_put(ts->hw_res.vcc_1v8);
        ts->hw_res.vcc_1v8 = NULL;
    }

    if (gpio_is_valid(ts->hw_res.enable2v8_gpio))
        gpio_free(ts->hw_res.enable2v8_gpio);

    if (gpio_is_valid(ts->hw_res.enable1v8_gpio))
        gpio_free(ts->hw_res.enable1v8_gpio);

    if (gpio_is_valid(ts->hw_res.irq_gpio)) {
        gpio_free(ts->hw_res.irq_gpio);
    }

    if (gpio_is_valid(ts->hw_res.reset_gpio)) {
        gpio_free(ts->hw_res.reset_gpio);
    }

    if (gpio_is_valid(ts->hw_res.id1_gpio)) {
        gpio_free(ts->hw_res.id1_gpio);
    }

    if (gpio_is_valid(ts->hw_res.id2_gpio)) {
        gpio_free(ts->hw_res.id2_gpio);
    }

    if (gpio_is_valid(ts->hw_res.id3_gpio)) {
        gpio_free(ts->hw_res.id3_gpio);
    }
	msleep(200);
	sec_ts_pinctrl_configure(&ts->hw_res, false);

    return ret;
}

/**
 * touchpanel_ts_suspend - touchpanel suspend function
 * @dev: i2c_client->dev using to get touchpanel_data resource
 *
 * suspend function bind to LCD on/off status
 * Returning zero(sucess) or negative errno(failed)
 */
static int tp_suspend(struct device *dev)
{
    int ret;
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s: start.\n", __func__);

    TPD_INFO("tp_suspend ts->spuri_fp_touch.fp_trigger =%d  ts->i2c_ready =%d  ts->spuri_fp_touch.lcd_resume_ok=%d \n",
              ts->spuri_fp_touch.fp_trigger , ts->i2c_ready , ts->spuri_fp_touch.lcd_resume_ok);
    ts->spuri_fp_touch.lcd_resume_ok = false;
    //step1:detect whether we need to do suspend
    if (ts->input_dev == NULL) {
        TPD_INFO("input_dev  registration is not complete\n");
        goto NO_NEED_SUSPEND;
    }
    if (ts->loading_fw) {
        TPD_INFO("FW is updating while suspending");
        goto NO_NEED_SUSPEND;
    }

#ifndef TPD_USE_EINT
        hrtimer_cancel(&ts->timer);
#endif

    /* release all complete first */
    if (ts->ts_ops->reinit_device) {
        ts->ts_ops->reinit_device(ts->chip_data);
    }

    //step2:get mutex && start process suspend flow
    mutex_lock(&ts->mutex);
    if (!ts->is_suspended) {
        ts->is_suspended = 1;
        ts->suspend_state = TP_SUSPEND_COMPLETE;
    } else {
        TPD_INFO("%s: do not suspend twice.\n", __func__);
        goto EXIT;
    }

    //step3:Release key && touch event before suspend
    tp_btnkey_release(ts);
    tp_touch_release(ts);

    //step4:cancel esd test
    if (ts->esd_handle_support) {
        esd_handle_switch(&ts->esd_info, false);
    }

    //step5:ear sense support
    if (ts->ear_sense_support) {
        ts->ts_ops->mode_switch(ts->chip_data, MODE_EARSENSE, false);
    }
    if (ts->face_detect_support) {
        ts->ts_ops->mode_switch(ts->chip_data, MODE_FACE_DETECT, false);
    }

    //step6:gesture mode status process
    if (ts->black_gesture_support) {
        if (ts->gesture_enable == 1) {
            ts->ts_ops->mode_switch(ts->chip_data, MODE_GESTURE, true);
            goto EXIT;
        }
    }

    //step7:skip suspend operate only when gesture_enable is 0
    if (ts->skip_suspend_operate && (!ts->gesture_enable)) {
        goto EXIT;
    }

    //step8:switch mode to sleep
    ret = ts->ts_ops->mode_switch(ts->chip_data, MODE_SLEEP, true);
    if (ret < 0) {
        TPD_INFO("%s, Touchpanel operate mode switch failed\n", __func__);
    }

	sec_ts_pinctrl_configure(&ts->hw_res, false);


EXIT:
    TPD_INFO("%s: end.\n", __func__);
    mutex_unlock(&ts->mutex);

NO_NEED_SUSPEND:
    complete(&ts->pm_complete);

    return 0;
}

/**
 * touchpanel_ts_suspend - touchpanel resume function
 * @dev: i2c_client->dev using to get touchpanel_data resource
 *
 * resume function bind to LCD on/off status, this fuction start thread to speedup screen on flow.
 * Do not care the result: Return void type
 */
static void tp_resume(struct device *dev)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s start.\n", __func__);

    if (!ts->is_suspended) {
        TPD_INFO("%s: do not resume twice.\n", __func__);
        goto NO_NEED_RESUME;
    }
    ts->is_suspended = 0;
    ts->suspend_state = TP_RESUME_COMPLETE;
    if (ts->loading_fw)
        goto NO_NEED_RESUME;

    //free irq at first
    free_irq(ts->irq, ts);

    if (ts->ts_ops->reinit_device) {
        ts->ts_ops->reinit_device(ts->chip_data);
    }
    if(ts->ts_ops->resume_prepare) {
        mutex_lock(&ts->mutex);
        ts->ts_ops->resume_prepare(ts->chip_data);
        mutex_unlock(&ts->mutex);
    }

    queue_work(ts->speedup_resume_wq, &ts->speed_up_work);
    return;

NO_NEED_RESUME:
    ts->suspend_state = TP_SPEEDUP_RESUME_COMPLETE;
    complete(&ts->pm_complete);
}

/**
 * speedup_resume - speedup resume thread process
 * @work: work struct using for this thread
 *
 * do actully resume function
 * Do not care the result: Return void type
 */
static void speedup_resume(struct work_struct *work)
{
    int timed_out = 0;
    struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
            speed_up_work);

    TPD_INFO("%s is called\n", __func__);

    //step1: get mutex for locking i2c acess flow
    mutex_lock(&ts->mutex);

    //step2:before Resume clear All of touch/key event Reset some flag to default satus
    if (ts->edge_limit_support)
        ts->edge_limit.in_which_area = AREA_NOTOUCH;
    tp_btnkey_release(ts);
    tp_touch_release(ts);

    if (ts->int_mode == UNBANNABLE) {
        tp_register_irq_func(ts);
    }


    //step3:Reset IC && switch work mode, ft8006 is reset by lcd, no more reset needed
	if (!ts->skip_reset_in_resume) {
		ts->ts_ops->reset(ts->chip_data);
		if (ts->gesture_enable) {
			if (g_tp->touchold_event) {
				TPD_INFO("touchhold up\n");
				g_tp->touchold_event = 0;
				gf_opticalfp_irq_handler(0);	//do reset will lost touchhold up event.
			}
		}
	}
	ts->skip_reset_in_resume = false;
	if (!ts->gesture_enable) {
		sec_ts_pinctrl_configure(&ts->hw_res, true);
	}

    //step4:If use resume notify, exit wait first
    if (ts->use_resume_notify) {
        reinit_completion(&ts->resume_complete);
        timed_out = wait_for_completion_timeout(&ts->resume_complete, 1*HZ);  //wait resume over for 1s
        if ((0 == timed_out) || (ts->resume_complete.done)) {
             TPD_INFO("resume state, timed_out:%d, done:%d\n", timed_out, ts->resume_complete.done);
        }
    }

    if (ts->ts_ops->specific_resume_operate) {
        ts->ts_ops->specific_resume_operate(ts->chip_data);
    }

   //step5: set default ps status to far
    if (ts->ts_ops->write_ps_status) {
        ts->ts_ops->write_ps_status(ts->chip_data, 0);
    }

    operate_mode_switch(ts);

    if (ts->esd_handle_support) {
        esd_handle_switch(&ts->esd_info, true);
    }

    //step6:Request irq again
    if (ts->int_mode == BANNABLE) {
        tp_register_irq_func(ts);
    }

    ts->suspend_state = TP_SPEEDUP_RESUME_COMPLETE;

    //step7:Unlock  && exit
    TPD_INFO("%s: end!\n", __func__);
    mutex_unlock(&ts->mutex);
    complete(&ts->pm_complete);
}

#if defined(CONFIG_FB) || defined(CONFIG_DRM_MSM)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    int *blank;
    int timed_out = -1;
    struct fb_event *evdata = data;
    struct touchpanel_data *ts = container_of(self, struct touchpanel_data, fb_notif);

    //to aviod some kernel bug (at fbmem.c some local veriable are not initialized)
#ifdef CONFIG_DRM_MSM
    if(event != MSM_DRM_EARLY_EVENT_BLANK && event != MSM_DRM_EVENT_BLANK)
#else
    if(event != FB_EARLY_EVENT_BLANK && event != FB_EVENT_BLANK)
#endif
        return 0;

    if (evdata && evdata->data && ts && ts->chip_data) {
        blank = evdata->data;
        TPD_INFO("%s: event = %ld, blank = %d\n", __func__, event, *blank);
        if (*blank == MSM_DRM_BLANK_POWERDOWN_CUST) { //suspend
            if (event == MSM_DRM_EARLY_EVENT_BLANK) {    //early event

                timed_out = wait_for_completion_timeout(&ts->pm_complete, 0.5*HZ);  //wait resume over for 0.5s
                if ((0 == timed_out) || (ts->pm_complete.done)) {
                    TPD_INFO("completion state, timed_out:%d, done:%d\n", timed_out, ts->pm_complete.done);
                }

                ts->suspend_state = TP_SUSPEND_EARLY_EVENT;      //set suspend_resume_state
                if (ts->esd_handle_support && ts->is_incell_panel && (ts->tp_suspend_order == LCD_TP_SUSPEND)) {
                    esd_handle_switch(&ts->esd_info, false);     //incell panel need cancel esd early
                }

                if (ts->tp_suspend_order == TP_LCD_SUSPEND) {
                    tp_suspend(ts->dev);
                } else if (ts->tp_suspend_order == LCD_TP_SUSPEND) {
					if (!ts->gesture_enable) {
						disable_irq_nosync(ts->irq);	//avoid iic error
					}
					tp_suspend(ts->dev);
                }
            } else if (event == MSM_DRM_EVENT_BLANK) {   //event

                if (ts->tp_suspend_order == TP_LCD_SUSPEND) {

                } else if (ts->tp_suspend_order == LCD_TP_SUSPEND) {
					tp_suspend(ts->dev);
                }
            }
        } else if (*blank == MSM_DRM_BLANK_UNBLANK_CUST) {//resume
            if (event == MSM_DRM_EARLY_EVENT_BLANK) {    //early event

                timed_out = wait_for_completion_timeout(&ts->pm_complete, 0.5*HZ);  //wait suspend over for 0.5s
                if ((0 == timed_out) || (ts->pm_complete.done)) {
                    TPD_INFO("completion state, timed_out:%d, done:%d\n", timed_out, ts->pm_complete.done);
                }

                ts->suspend_state = TP_RESUME_EARLY_EVENT;      //set suspend_resume_state

                if (ts->tp_resume_order == TP_LCD_RESUME) {
					tp_resume(ts->dev);
                } else if (ts->tp_resume_order == LCD_TP_RESUME) {
                    disable_irq_nosync(ts->irq);
                }
            } else if (event == MSM_DRM_EVENT_BLANK) {   //event

                if (ts->tp_resume_order == TP_LCD_RESUME) {

                } else if (ts->tp_resume_order == LCD_TP_RESUME) {
                    tp_resume(ts->dev);
                    enable_irq(ts->irq);
                }
            }
          }else if (*blank == MSM_DRM_DYNAMICFPS_60) {  //60-90HZ LCD refresh switch
			if (event == MSM_DRM_EARLY_EVENT_BLANK) {
				mutex_lock(&ts->mutex);
				if (!ts->is_suspended && (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE)) {
					ts->ts_ops->mode_switch(ts->chip_data, MODE_REFRESH_SWITCH, 0);
				}
				mutex_unlock(&ts->mutex);
			}
		} else if (*blank == MSM_DRM_DYNAMICFPS_90) {
			if (event == MSM_DRM_EARLY_EVENT_BLANK) {
				mutex_lock(&ts->mutex);
				if (!ts->is_suspended && (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE)) {
					ts->ts_ops->mode_switch(ts->chip_data, MODE_REFRESH_SWITCH, 1);
				}
				mutex_unlock(&ts->mutex);
			}	
		}
    }

    return 0;
}
#endif

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
void tp_i2c_suspend(struct touchpanel_data *ts)
{
    ts->i2c_ready = false;
    if (ts->black_gesture_support) {
        if (ts->gesture_enable == 1) {
            /*enable gpio wake system through interrupt*/
            enable_irq_wake(ts->irq);
            return;
        }
    }
    disable_irq_nosync(ts->irq);
}

void tp_i2c_resume(struct touchpanel_data *ts)
{
    if (ts->black_gesture_support) {
        if (ts->gesture_enable == 1) {
            /*disable gpio wake system through intterrupt*/
            disable_irq_wake(ts->irq);
            goto OUT;
        }
    }
    enable_irq(ts->irq);

OUT:
    ts->i2c_ready = true;
    if (ts->spurious_fp_support && ts->spuri_fp_touch.fp_trigger) {
        wake_up_interruptible(&waiter);
    }
}

#else
void tp_i2c_suspend(struct touchpanel_data *ts)
{
    ts->i2c_ready = false;
    if (ts->black_gesture_support) {
        if (ts->gesture_enable == 1) {
            /*enable gpio wake system through interrupt*/
            enable_irq_wake(ts->irq);
        }
    }
    disable_irq_nosync(ts->irq);
}

void tp_i2c_resume(struct touchpanel_data *ts)
{
    if (ts->black_gesture_support) {
        if (ts->gesture_enable == 1) {
            /*disable gpio wake system through intterrupt*/
            disable_irq_wake(ts->irq);
        }
    }
    enable_irq(ts->irq);
    ts->i2c_ready = true;
    if (ts->spurious_fp_support && ts->spuri_fp_touch.fp_trigger) {
        wake_up_interruptible(&waiter);
    }
}
#endif

struct touchpanel_data *common_touch_data_alloc(void)
{
    if (g_tp) {
        TPD_INFO("%s:common panel struct has alloc already!\n", __func__);
        return NULL;
    }
    return kzalloc(sizeof(struct touchpanel_data), GFP_KERNEL);
}

int common_touch_data_free(struct touchpanel_data *pdata)
{
    if (pdata) {
        kfree(pdata);
    }

    g_tp = NULL;
    return 0;
}

/**
 * input_report_key_reduce - Using for report virtual key
 * @work: work struct using for this thread
 *
 * before report virtual key, detect whether touch_area has been touched
 * Do not care the result: Return void type
 */
void input_report_key_reduce(struct input_dev *dev, unsigned int code, int value)
{
    if (value) {//report Key[down]
        if (g_tp) {
            if (g_tp->view_area_touched == 0) {
                input_report_key(dev, code, value);
            }
            else
                TPD_INFO("Sorry,tp is touch down,can not report touch key\n");
        }
    } else {
        input_report_key(dev, code, value);
    }
}

void clear_view_touchdown_flag(void)
{
    if (g_tp) {
        g_tp->view_area_touched = 0;
    }
}

