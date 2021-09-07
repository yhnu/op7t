/****************************************************************************************
** Copyright (C), 2013-2018, ONEPLUS Mobile Comm Corp., Ltd
** File: oneplus_motor.h
**
** Description:
**      Definitions for m1120 camera motor control layer.
**
****************************************************************************************/
#ifndef __ONEPLUS_MOTOR__H
#define __ONEPLUS_MOTOR__H

#include <linux/alarmtimer.h>
#include <linux/version.h>

#define MOTOR_TAG                  "[oneplus_motor] "
#define MOTOR_ERR(fmt, args...)    printk(KERN_ERR MOTOR_TAG" %s : "fmt,__FUNCTION__,##args)
#define MOTOR_LOG(fmt, args...)    printk(KERN_INFO MOTOR_TAG" %s : "fmt,__FUNCTION__,##args)

//camera state event to report
#define MOTOR_EVENT_TYPE			EV_KEY
#define MOTOR_EVENT_MANUAL_TO_DOWN  KEY_F14
#define MOTOR_EVENT_UP				KEY_F15
#define MOTOR_EVENT_UP_ABNORMAL		KEY_F16
#define MOTOR_EVENT_UP_NORMAL		KEY_F17
#define MOTOR_EVENT_DOWN			KEY_F18
#define MOTOR_EVENT_DOWN_ABNORMAL	KEY_F19
#define MOTOR_EVENT_DOWN_NORMAL		KEY_F20

//position hall data
#define HALL_DETECT_RANGE_HIGH		    (150)
#define HALL_DETECT_RANGE_LOW		    (-512)
#define MOTOR_RESET_TIMER		        (500)//500ms
#define MOTOR_STOP_STAITC_POS_VALUE		(10)
#define MOTOR_STOP_STAITC_NEG_VALUE		(-10)
#define MOTOR_STOP_RETARD_VALUE			(500)
//irq
#define MOTOR_IRQ_MONITOR_TIME		    (20)//20ms
#define MOTOR_IRQ_MONITOR_COUNT		    (8)

#define MOTOR_STOP_TIMEOUT              (12000) //80k

typedef enum hall_id {
	HALL_DOWN = 0,
	HALL_UP,
} hall_id;

enum dhall_detection_mode {
	DETECTION_MODE_POLLING = 0,
	DETECTION_MODE_INTERRUPT,
	DETECTION_MODE_INVALID,
};

enum motor_type {
	MOTOR_UNKNOWN = 0,
	MOTOR_FI5,
	MOTOR_FI6,
};

typedef enum motor_power {
	MOTOR_POWER_OFF = 0,
	MOTOR_POWER_ON,
} motor_power;

typedef enum motor_direction_t {
	MOTOR_DOWN = 0,
	MOTOR_UPWARD,
} motor_direction_t;

typedef enum camera_position_state_event {
	MANUAL_TO_DOWN_EVENT = 0,
	UPING_EVENT,
	UP_ABNORMA_EVENT,
	UP_NORMAL_EVENT,
	DOWNING_EVENT,
	DOWN_ABNORMAL_EVENT,
	DOWN_NORMAL_EVENT
} camera_position_state_event;

enum wakelock_id {
	MOTOR_RUN_LOCK = 0,
	HALL_DATA_LOCK,
	POSITION_DETECT_LOCK,
	MAX_LOCK
};

typedef enum motor_move_state {
	MOTOR_STOP = 0,//never move after boot
	MOTOR_UPWARD_ING,//motor is uping 
	MOTOR_DOWNWARD_ING,//motor down stop , may be abnormal
	MOTOR_UPWARD_STOP,//motor up stop , may be abnormal
	MOTOR_DOWNWARD_STOP//motor is downing
} motor_move_state;

typedef enum camera_position {
	PEAK_STATE,
	BOTTOM_STATE,
	MID_STATE,
} camera_position;

typedef enum motor_stall_mode {
	ENTER_DELTAD_RANGE_TWO_TIEMS  = 0,
	ENTER_DELTAD_AND_SHAKE,
	ONLY_SHAKE,
	UNKNOW_MODE,
} motor_stall_mode_t;

typedef enum motor_work_mode {
	MOTOR_MODE_FULL = 0,
	MOTOR_MODE_1_2,
	MOTOR_MODE_1_4,
	MOTOR_MODE_1_8,
	MOTOR_MODE_1_16,
	MOTOR_MODE_1_32
} motor_work_mode_t;

typedef enum motor_speed_t {
	MOTOR_SPEED0 = 0,     //high 2500pps  80khz 
	MOTOR_SPEED1,         //     2000pps  64khz
	MOTOR_SPEED2,         //     1600pps  51.2khz
	MOTOR_SPEED3,         //     1200pps  38.4khz
	MOTOR_SPEED4,         //     1000pps  32khz
	MOTOR_SPEED5,         //     800pps   25.6khz
	MOTOR_SPEED6,         //     700pps   22.4khz
	MOTOR_SPEED7,         //     600pps   19.2khz
	MOTOR_SPEED8,         //     518pps   16.6khz
	MOTOR_SPEED9,         //     400pps   12.8khz
	MOTOR_SPEED10,        //     300pps   9.6kkhz
	MOTOR_SPEED11,        //     200pps   6.4kkhz
	MOTOR_SPEED12,        //     100pps   3.2kkhz
	MOTOR_SPEED13,        //LOW  50pps	 1.6khz
	MOTOR_SPEED_SPECIAL   //2800pps
} motor_speed_t;

typedef enum hall_sensitive_t {
	HALL_10BIT_0_068mT = 1,
	HALL_10BIT_0_034mT
} hall_sensitive_t;

struct oneplus_hall_operations {
	const char*  name;
	bool (*is_power_on) (void);
	int  (*set_hall_enable_state) (bool enable);
	bool (*get_hall_enable_state) (void);
	int  (*get_data_real) (short* data);
	int  (*get_data_abs) (short* data);
	int  (*set_detection_mode) (u8 mode);
	int  (*enable_irq) (bool enable);
	int  (*clear_irq) (void);
	int  (*get_irq_state) (void);
	bool (*update_threshold) (int position, short lowthd, short highthd);
	void (*dump_regs) (u8* buf);
	int  (*set_reg) (int reg, int val);
	void  (*set_sensitive) (hall_sensitive_t sensitive);
};

struct oneplus_motor_operations {
	const char*  name;
	int (*set_power) (int mode);
	int (*set_direction) (int dir);
	int (*set_working_mode) (int mode);
	int (*get_all_config)(int* config , int count);
	int (*calculate_pwm_count) (int angle, int mode);
	int (*pwm_config) (int duty_ns, int period_ns);
	int (*pwm_enable) (void);
	int (*pwm_disable) (void);
	int (*get_motor_type) (void);
};

struct oneplus_motor_chip {
    //step motor  property 
	unsigned long                     pwm_duty;
	unsigned long                     pwm_period;
	motor_work_mode_t                 motor_work_mode;
	motor_speed_t                     motor_speed;
	motor_direction_t                 motor_direction;
	bool     	                      motor_enable;
	bool		                      motor_started;
	bool		                      motor_switch;

	//digital hall property
	short                             hall_down_data;
	short                             hall_up_data;
	int	                              hall_down_irq_count;
	int	                              hall_up_irq_count;
	short                             hall_down_irq_position;
	short                             hall_up_irq_position;
	hall_sensitive_t                  hall_sensitive;

	//calibrate property
	short	                          camera_up_slow_down_position_hall_down_data;
	short	                          camera_up_slow_down_position_hall_up_data;
	short	                          camera_down_slow_down_position_hall_down_data;
	short	                          camera_down_slow_down_position_hall_up_data;
    short                             bottom_position_hall_down_data;
	short                             bottom_position_hall_up_data;
    short                             peak_position_hall_down_data;
	short                             peak_position_hall_up_data;	
	int                               camera_up_step_count;
	int                               is_stall;
	int                               stall_steps;
	motor_stall_mode_t                stall_mode;

	//special test property
	bool		                      is_motor_test;
	bool                              force_move;
	bool                              is_speed_set;
	int                               is_factory_mode;//0:normal, 1:calibrate, 2:step test
	motor_speed_t                     test_speed;

	//logical control property
	atomic_t		                  in_suspend;
	int  		                      pwm_count;
	unsigned long                     whole_jonery_time;
	int				                  whole_jonery_length;
	int                               begin_stop_detect_percent;
	int				                  speed_up_distance;
	int				                  speed_down_distance;
	int				                  speed_up_pwm_count;
	int                               slow_down_speed;
	int                               deltad_range;
	//int				                  speed_down_pwm_count;
	bool                              camera_position_detect;
	int				                  position_detect_delay;
	struct timeval                    motor_start_time;
	bool                              save_hall_data_to_file;
	bool		 	                  hall_detect_switch;
	camera_position	                  position;
	motor_move_state                  move_state;
	bool			                  is_skip_pos_check;
	bool      		                  manual2auto_down_switch;
	bool                              is_free_fall;
	int                               free_fall_irq_times;
	int                               infrared_shut_down_state;
	unsigned int	                  free_fall_gpio;
	int		                          free_fall_irq;
	bool        	                  irq_monitor_started;
	bool        	                  is_irq_abnormal;
	bool        	                  is_t0_structure;
	bool        	                  is_mag_positive;
	bool			                  led_on;
	

	struct device*                    dev;
	struct pinctrl*                   pctrl;
	struct pinctrl_state*             free_fall_state;
	struct input_dev*                 input_dev;
	struct workqueue_struct*          manual2auto_wq;
	struct workqueue_struct*          motor_run_work_wq;
	struct delayed_work	              detect_work;
	struct delayed_work	              free_fall_irq_check_work;
	struct work_struct                motor_work;
	struct delayed_work               up_work;
	struct delayed_work			      down_work;
	struct hrtimer 					  stop_timer;
	bool		                      stop_timer_trigger;
	struct hrtimer 					  speed_up_timer;
	struct alarm    				  reset_timer;
	struct notifier_block 			  fb_notify;
	struct oneplus_hall_operations*   hall_up_ops;
	struct oneplus_hall_operations*   hall_down_ops;
	struct oneplus_motor_operations*  motor_ops;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	struct wake_lock        		   suspend_lock;
#else
	struct wakeup_source*	           suspend_ws;
#endif
	
};


/************************digital_hall and step_motor register ops function************************/
int oneplus_register_dhall(const char*  name, struct oneplus_hall_operations* ops);
int oneplus_register_motor(const char*  name, struct oneplus_motor_operations* ops);

int  oneplus_dhall_irq_handler(unsigned int id);


#endif
