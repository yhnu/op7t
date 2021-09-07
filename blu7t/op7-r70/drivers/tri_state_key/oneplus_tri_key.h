/************************************************************************************
** Copyright (C), 2013-2018, Oneplus Mobile Comm Corp., Ltd
** File: oneplus_tri_key.h
**
** Description:
**      Definitions for m1120 tri_state_key data process.
**
** Version: 1.0
**************************************************************************************/

#include <linux/alarmtimer.h>
#include <linux/version.h>

/*
#define MODE_MUTE 1
#define MODE_DO_NOT_DISTURB 2
#define MODE_NORMAL 3
*/
typedef enum debug_level {
	LEVEL_BASIC,
	LEVEL_DEBUG,
}tri_key_debug_level;

enum dhall_id {
	DHALL_0 = 0,
	DHALL_1,
};
// enum dhall_id {
// 	DHALL_DOWN = 0,
// 	DHALL_UP,
// };

enum dhall_detection_mode {
	DETECTION_MODE_POLLING = 0,
	DETECTION_MODE_INTERRUPT,
	DETECTION_MODE_INVALID,
};

enum motor_direction {
	MOTOR_DOWN = 0,
	MOTOR_UPWARD,
};

enum tri_key_position {
	UP_STATE,
	DOWN_STATE,
	MID_STATE,
};

extern unsigned int tristate_extcon_tab[];
extern unsigned int tri_key_debug;

typedef struct {
	short data0;
	short data1;
} dhall_data_t;

struct dhall_operations {
	int (*get_data) (short *data);
	int (*set_detection_mode) (u8 mode);
	int (*enable_irq) (bool enable);
	int (*clear_irq) (void);
	int (*get_irq_state) (void);
	bool (*update_threshold) (int position, short lowthd, short highthd);
	void (*dump_regs) (u8 *buf);
	int (*set_reg) (int reg, int val);
	bool (*is_power_on) (void);
	void (*set_sensitivity) (char *data);
}; 

 struct extcon_dev_data {

	struct work_struct dwork;
	struct extcon_dev *edev; //change 1
	struct device *dev;
	struct timer_list s_timer;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	struct delayed_work	up_work;
	struct delayed_work	down_work;
	struct dhall_operations *dhall_up_ops;
	struct dhall_operations *dhall_down_ops;
	struct mutex mtx;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	//struct wake_lock        suspend_lock;
#else
	//struct wakeup_source	*suspend_ws;
#endif
	const char *d_name;
	const char *m_name;
	int		position;
	int		last_position;
	int		interf;//interference
	short		state;
	short		dhall_data0;
	short		dhall_data1;
	short		dnHall_UpV;
	short		dnHall_MdV;
	short		dnHall_DnV;
	short		upHall_UpV;
	short		upHall_MdV;
	short		upHall_DnV;
	//short		dnHall_UpV_pre;
	//short		dnHall_MdV_pre;
	//short		dnHall_DnV_pre;
	//short		upHall_UpV_pre;
	//short		upHall_MdV_pre;
	//short		upHall_DnV_pre;
	int        	manual2auto_up_switch;
	int        	manual2auto_down_switch;
	int			irq;
	//bool        irq_monitor_started;
	//bool        is_irq_abnormal;
};

extern int oneplus_register_hall(const char *name, struct dhall_operations *ops);
//dhall control api
extern int oneplus_hall_get_data(unsigned int id);
extern int oneplus_hall_set_detection_mode(unsigned int id, u8 mode);
extern int oneplus_hall_enable_irq (unsigned int id, bool enable);
extern int oneplus_hall_clear_irq (unsigned int id);
extern int oneplus_hall_irq_handler(unsigned int id);
extern int oneplus_hall_get_irq_state(unsigned int id);
extern void oneplus_hall_dump_regs(unsigned int id, u8 *buf);
extern int oneplus_hall_set_reg(unsigned int id, int reg, int val);
extern bool oneplus_hall_update_threshold(unsigned int id, int position, short lowthd, short highthd);
extern bool oneplus_hall_is_power_on(void);
extern int aw8697_op_haptic_stop(void);
