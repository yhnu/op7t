/****************************************************************************************
** Copyright (C), 2013-2018, ONEPLUS Mobile Comm Corp., Ltd
** File: oneplus_motor.c
**
** Description:
**      Definitions for m1120 camera motor control layer.
**
****************************************************************************************/

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/hrtimer.h>
#include <linux/alarmtimer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/string.h>
#include <linux/syscalls.h>
//#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#ifdef CONFIG_DRM_MSM
#include <linux/msm_drm_notify.h>
#endif
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
#include <linux/wakelock.h>
#endif
#include "oneplus_motor.h"
#include "oneplus_motor_notifier.h"

/************************static global variable************************/
static struct oneplus_motor_chip*  g_the_chip = NULL;
static DEFINE_MUTEX(motor_running_mutex);
static DEFINE_MUTEX(position_detect_mutex);
static DEFINE_MUTEX(motor_start_mutex);

/************************fb_notifier************************/
static void oneplus_motor_notify_state(unsigned long val);
static int  fb_notifier_callback(struct notifier_block* nb, unsigned long event, void*  data);

/************************digital_hall control interface************************/
bool        oneplus_dhall_is_power_on(void);
static int  oneplus_hall_set_enable_state(unsigned int id, bool enable);
static bool oneplus_hall_get_enable_state(unsigned int id);
static int  oneplus_hall_get_abs_data(unsigned int id);
int         oneplus_hall_get_real_data(unsigned int id);
int         oneplus_dhall_set_detection_mode(unsigned int id, u8 mode);
static int  oneplus_hall_enable_irq (unsigned int id, bool enable);
static int  oneplus_hall_clear_irq (unsigned int id);
int         oneplus_dhall_get_irq_state(unsigned int id);
static int  oneplus_hall_update_threshold(unsigned int id, int position, short lowthd, short highthd);
void        oneplus_dhall_dump_regs(unsigned int id, u8* buf);
int         oneplus_dhall_set_reg(unsigned int id, int reg, int val);
int         oneplus_dhall_set_sensitive(unsigned int id, hall_sensitive_t sensitive);
	
/************************step_motor control interface************************/
static int  oneplus_motor_set_power (motor_power mode);
static int  oneplus_motor_set_direction (int dir);
static int  oneplus_motor_set_working_mode (int mode);
//static int  oneplus_motor_calculate_pwm_count(int L, int mode);
static int  oneplus_motor_pwm_config(int duty_ns, int period_ns);
static int  oneplus_motor_pwm_enable(void);
static int  oneplus_motor_pwm_disable(void);
int         oneplus_motor_get_all_config(int* config, int count);
int         oneplus_get_motor_type(void);
void        oneplus_set_md_mode_para(int motor_work_mode);

/************************combine digital_hall and step_motor for logical process************************/
static void  manual_to_auto_down_work(struct work_struct* work);
static void  oneplus_set_motor_speed(int speed);
static void  oneplus_change_motor_speed(int speed);
static void  oneplus_set_motor_direction(int direction);
static void  oneplus_set_motor_move_state(int move_state);
 
static bool  oneplus_motor_run_check(struct oneplus_motor_chip* chip);
static void  oneplus_motor_control(int on, int speed, int direction);
static void  oneplus_motor_start(void);
static void  oneplus_motor_stop(void);
static void  oneplus_motor_downward(void);
static void  motor_run_work(struct work_struct* work);
static void  camera_position_detect_work(struct work_struct* work);
static void  free_fall_irq_check_work_func(struct work_struct* work);
static int   write_hall_data_to_file(short* hall_up_data, short* hall_down_data, int buf_len, int direction, bool append);
static enum  hrtimer_restart motor_stop_timer_func(struct hrtimer* hrtimer);
static enum  hrtimer_restart motor_speed_up_timer_func(struct hrtimer* hrtimer);
static enum  alarmtimer_restart motor_reset_timer_func(struct alarm* alrm, ktime_t now);
static void  oneplus_motor_set_awake(struct oneplus_motor_chip* chip, int id, bool awake);
static void  report_position_state(struct oneplus_motor_chip* chip, camera_position_state_event state_event); 

static void  oneplus_motor_irq_monitor(struct oneplus_motor_chip* chip);
static irqreturn_t oneplus_free_fall_detect_handler(int irq, void*  dev_id);

/************************node operation by MotorManagerService or user************************/
//for normal use
static ssize_t  motor_direction_store(struct device* pdev, struct device_attribute* attr,
		                              const char* buf, size_t count);
static ssize_t  motor_direction_show(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t  motor_enable_store(struct device* pdev, struct device_attribute* attr, 
								   const char* buff, size_t count);
static ssize_t  step_count_store(struct device *pdev, struct device_attribute *attr,
			                    const char *buff, size_t count);
static ssize_t  step_count_show(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t  save_hall_data_store(struct device *pdev, struct device_attribute *attr,
		                            const char *buff, size_t count);
static ssize_t  save_hall_data_show(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t  hall_data_show(struct device* dev,struct device_attribute* attr, char* buf);
static ssize_t  motor_move_state_show(struct device* dev, struct device_attribute* attr, char* buf);

static ssize_t  hall_calibration_show(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t  hall_calibration_store(struct device* pdev, struct device_attribute* attr,
					                   const char* buff, size_t count);
static ssize_t  stall_show(struct device* dev, struct device_attribute* attr, char* buf);	
static ssize_t  stall_steps_show(struct device* dev, struct device_attribute* attr, char* buf);	
static ssize_t  motor_test_show(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t  motor_test_store(struct device* pdev, struct device_attribute* attr, 
 								 const char* buff, size_t count);
static ssize_t  hall_irq_count_show(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t  motor_mode_store(struct device* pdev, struct device_attribute* attr,
		                         const char* buff, size_t count);
static ssize_t  motor_mode_show(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t  motor_manual2auto_switch_show(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t  motor_manual2auto_switch_store(struct device* pdev, struct device_attribute* attr,
				 	                           const char* buff, size_t count);
static ssize_t  motor_sw_switch_store(struct device* pdev, struct device_attribute* attr,
				 					  const char* buff, size_t count);
static ssize_t  motor_sw_switch_show(struct device* dev, struct device_attribute* attr, char* buf);
//for special test
static ssize_t  motor_force_move_store(struct device* pdev, struct device_attribute* attr,
				                       const char* buff, size_t count);
///static ssize_t  motor_stop_time_store(struct device* pdev, struct device_attribute* attr,
//		                              const char* buff, size_t count);
//static ssize_t  motor_stop_time_show(struct device* dev, struct device_attribute* attr, char* buf);
//static ssize_t  motor_initial_time_store(struct device* pdev, struct device_attribute* attr,
//		                                 const char* buff, size_t count);
//static ssize_t  motor_initial_time_show(struct device* dev, struct device_attribute* attr, char* buf);
//static ssize_t  motor_initial_speed_store(struct device* pdev, struct device_attribute* attr,
//		                                       const char* buff, size_t count);
//static ssize_t  motor_initial_speed_show(struct device* dev, struct device_attribute* attr, char* buf);
//static ssize_t  motor_high_speed_store(struct device* pdev, struct device_attribute* attr,
//		                               const char* buff, size_t count);
//static ssize_t  motor_high_speed_show(struct device* dev, struct device_attribute* attr, char* buf);
//static ssize_t  motor_high_speed_time_store(struct device* pdev, struct device_attribute* attr,
//		                                    const char* buff, size_t count);
//static ssize_t  motor_high_speed_time_show(struct device* dev,struct device_attribute* attr, char* buf);
static ssize_t  motor_slow_down_speed_store(struct device* pdev, struct device_attribute* attr,
		                                    const char* buff, size_t count);
static ssize_t  motor_slow_down_speed_show(struct device* dev, struct device_attribute* attr, char* buf);
//not use 
// static ssize_t  motor_speed_change_switch_store(struct device* pdev, struct device_attribute* attr,
// 				 							    const char* buff, size_t count);
// static ssize_t  motor_speed_change_switch_show(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t  hall_detect_switch_store(struct device* pdev, struct device_attribute* attr,
				 						 const char* buff, size_t count);
static ssize_t  hall_detect_switch_show(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t  motor_all_config_show(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t  motor_position_show(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t  hall_all_reg_show(struct device* dev, struct device_attribute* attr, char* buf);
 //save for compatibility
static ssize_t  motor_change_speed_store(struct device* pdev, struct device_attribute* attr,
				 	                     const char* buff, size_t count);
static ssize_t  motor_speed_store(struct device* pdev, struct device_attribute* attr,
		                          const char* buff, size_t count);
static ssize_t  motor_speed_show(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t  hall_sensitive_store(struct device* pdev, struct device_attribute* attr,
									 const char* buff, size_t count);
static ssize_t  hall_sensitive_show(struct device* dev, struct device_attribute* attr, char* buf);


/************************init function************************/	
static int   oneplus_input_dev_init(struct oneplus_motor_chip* chip);
static int   oneplus_motor_chip_init(struct oneplus_motor_chip* chip);
static void  oneplus_motor_awake_init(struct oneplus_motor_chip* chip);
static void  oneplus_motor_free_fall_register(struct oneplus_motor_chip*  chip);
static void  oneplus_motor_reset_check(struct oneplus_motor_chip* chip);
static void  oneplus_parameter_init(struct oneplus_motor_chip* chip);
static int   motor_platform_probe(struct platform_device* pdev);
static int   motor_platform_remove(struct platform_device* pdev);
static int   motor_platform_suspend(struct platform_device* pdev, pm_message_t state);
static int   motor_platform_resume(struct platform_device* pdev);
static void  motor_platform_shutdown(struct platform_device* pdev);


/*********************************************************************
						 fb_notifier
**********************************************************************/
static void oneplus_motor_notify_state(unsigned long val)
{
    if (val < MOTOR_UP_EVENT || val > MOTOR_BLOCK_EVENT)
        return;

    motor_notifier_call_chain(val);
}

#ifdef CONFIG_DRM_MSM
static int fb_notifier_callback(struct notifier_block* nb, unsigned long event, void* data)
{
	int blank;
	struct msm_drm_notifier* evdata = data;

	if (g_the_chip == NULL) {
		return 0;
	}

	if (!evdata || (evdata->id != 0))
		return 0;

	if (event == MSM_DRM_EARLY_EVENT_BLANK) {
		blank =* (int* )(evdata->data);
		if (blank == MSM_DRM_BLANK_UNBLANK) {
			g_the_chip->led_on = true;
			MOTOR_LOG("led_on %d\n", g_the_chip->led_on);
		} else if (blank == MSM_DRM_BLANK_POWERDOWN) {
			g_the_chip->led_on = false;
			MOTOR_LOG("led_on %d \n", g_the_chip->led_on);
		} else {
			MOTOR_LOG("receives wrong data EARLY_BLANK:%d \n", blank);
		}
	}

	return 0;
}
#else
static int fb_notifier_callback(struct notifier_block* nb, unsigned long event, void* data)
{
	int blank;
	struct fb_event* evdata = data;

	if (g_the_chip == NULL) {
		return 0;
	}

	if (evdata && evdata->data) {
		if (event == FB_EVENT_BLANK) {
			blank =* (int* )evdata->data;
			if (blank == FB_BLANK_UNBLANK) {
				g_the_chip->led_on = true;
				MOTOR_LOG("led_on %d\n", g_the_chip->led_on);
			} else if (blank == FB_BLANK_POWERDOWN) {
				g_the_chip->led_on = false;
				MOTOR_LOG("led_on %d\n", g_the_chip->led_on);
			}
		}
	}
	return 0;
}
#endif /*CONFIG_DRM_MSM*/

/*********************************************************************
						 digital_hall control interface
**********************************************************************/
bool oneplus_dhall_is_power_on(void)
{
	if (g_the_chip == NULL || g_the_chip->hall_down_ops == NULL 
						|| g_the_chip->hall_down_ops->is_power_on == NULL
						|| g_the_chip->hall_up_ops == NULL 
						|| g_the_chip->hall_up_ops->is_power_on == NULL) {

		MOTOR_ERR("null pointer");

  		return false;
 	} else {
 	    if (g_the_chip->hall_down_ops->is_power_on() && g_the_chip->hall_up_ops->is_power_on())
 		    return true;
 		else
 		    return false;
	}

}

static int oneplus_hall_set_enable_state(unsigned int id, bool enable)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return -EINVAL;
	}

	switch (id) {
	case HALL_DOWN :
		if(g_the_chip->hall_down_ops == NULL ||  g_the_chip->hall_down_ops->set_hall_enable_state == NULL) {

			MOTOR_ERR("g_the_chip->hall_down_ops == NULL ||  g_the_chip->hall_down_ops->set_hall_enable_state == NULL");
			return -EINVAL;
		} else {
			return g_the_chip->hall_down_ops->set_hall_enable_state(enable);
		}
	case HALL_UP :
		if(g_the_chip->hall_up_ops == NULL ||  g_the_chip->hall_up_ops->set_hall_enable_state == NULL) {

			MOTOR_ERR("g_the_chip->hall_up_ops == NULL ||  g_the_chip->hall_up_ops->set_hall_enable_state == NULL");
			return -EINVAL;
		} else {
			return g_the_chip->hall_up_ops->set_hall_enable_state(enable);
		}
	default : 
		return -EINVAL;
	}

	return -EINVAL;
}

static bool oneplus_hall_get_enable_state(unsigned int id)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return -EINVAL;
	}

	switch (id) {
	case HALL_DOWN :
		if(g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->get_hall_enable_state == NULL) {

			MOTOR_ERR("g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->get_hall_enable_state == NULL");
			return false;
		} else {
			return g_the_chip->hall_down_ops->get_hall_enable_state();
		}
	case HALL_UP :
		if(g_the_chip->hall_up_ops == NULL ||  g_the_chip->hall_up_ops->get_hall_enable_state == NULL) {

			MOTOR_ERR("g_the_chip->hall_up_ops == NULL ||  g_the_chip->hall_up_ops->get_hall_enable_state == NULL");
			return false;
		} else {
			return g_the_chip->hall_up_ops->get_hall_enable_state();
		}
	default : 
		return false;
	}

	return false;
}

static int oneplus_hall_get_abs_data(unsigned int id)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return -EINVAL;
	}

	switch (id){
	case HALL_DOWN:		
		if (g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->get_data_abs == NULL) {
			MOTOR_ERR("g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->get_data_abs == NULL");
  			return -EINVAL;
 		} else {
 				return g_the_chip->hall_down_ops->get_data_abs(&g_the_chip->hall_down_data);
		}
	case HALL_UP:		
		if (g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->get_data_abs == NULL) {
			MOTOR_ERR("g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->get_data_abs == NULL");
  			return -EINVAL;
 		} else {
 				return g_the_chip->hall_up_ops->get_data_abs(&g_the_chip->hall_up_data);
		}
		default:
			MOTOR_ERR("id : %d is not correct \n", id);
			return -EINVAL;
	}

	return -EINVAL;
}

int oneplus_hall_get_real_data(unsigned int id)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return -EINVAL;
	}

	switch (id){
	case HALL_DOWN:		 
		if (g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->get_data_real == NULL) {

			MOTOR_ERR("g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->get_data_real == NULL \n");
  			return -EINVAL;
 		} else {
 			return g_the_chip->hall_down_ops->get_data_real(&g_the_chip->hall_down_data);
		}
	case HALL_UP:		
		if (g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->get_data_real == NULL) {

			MOTOR_ERR("g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->get_data_real == NULL \n");
  			return -EINVAL;
 		} else {
 			return g_the_chip->hall_up_ops->get_data_real(&g_the_chip->hall_up_data);
		}
		default:
			MOTOR_ERR("id : %d is not correct \n", id);
			return -EINVAL;	
	}

	return -EINVAL;
}

int oneplus_dhall_set_sensitive(unsigned int id, hall_sensitive_t sensitive)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return -EINVAL ;
	}

	switch (id){
	case HALL_DOWN:
		if (g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->set_sensitive == NULL) {

			MOTOR_ERR("g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->get_data_real == NULL \n");
			return -EINVAL;
		} else {
			g_the_chip->hall_down_ops->set_sensitive(sensitive);
			return 0;
		}
	case HALL_UP:
		if (g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->set_sensitive == NULL) {

			MOTOR_ERR("g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->get_data_real == NULL \n");
			return -EINVAL;
		} else {
			g_the_chip->hall_up_ops->set_sensitive(sensitive);
			return 0;
		}
		default:
			MOTOR_ERR("id : %d is not correct \n", id);
			return -EINVAL;
	}

	return 0;
}

int oneplus_dhall_set_detection_mode(unsigned int id, u8 mode)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return -EINVAL;
	}

	switch (id){
	case HALL_DOWN:		
		if (g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->set_detection_mode == NULL) {

			MOTOR_ERR("g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->set_detection_mode == NULL \n");
  			return -EINVAL;
 		} else {
 			return g_the_chip->hall_down_ops->set_detection_mode(mode);
		}
	case HALL_UP:		
		if (g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->set_detection_mode == NULL) {

			MOTOR_ERR("g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->set_detection_mode == NULL \n");
  			return -EINVAL;
 		} else {
 			return g_the_chip->hall_up_ops->set_detection_mode(mode);
		}
		default:
			MOTOR_ERR("id : %d is not correct \n", id);
			return -EINVAL;
	}

	return -EINVAL;
}


//should use in irq handle
int oneplus_hall_enable_irq (unsigned int id, bool enable)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return -EINVAL;
	}

	switch (id){
	case HALL_DOWN:		
		if (g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->enable_irq == NULL){
			MOTOR_ERR("g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->enable_irq == NULL \n");
  			return -EINVAL;
		} else {
			 return g_the_chip->hall_down_ops->enable_irq(enable);
		 } 
 			
	case HALL_UP:		
		if (g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->enable_irq == NULL) {
			MOTOR_ERR("g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->enable_irq == NULL \n");
  			return -EINVAL;
 		} else {
 			return g_the_chip->hall_up_ops->enable_irq(enable);
		}
	default:
		MOTOR_ERR("id : %d is not correct \n", id);	
		return -EINVAL;
	}

	return -EINVAL;
}

int oneplus_hall_clear_irq (unsigned int id)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return -EINVAL;
	}

	MOTOR_LOG("oneplus_hall_clear_irq \n");

	switch (id){
	case HALL_DOWN:		
		if (g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->enable_irq == NULL) {

			MOTOR_ERR("g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->enable_irq == NULL \n");
  			return -EINVAL;
 		} else {
 			return g_the_chip->hall_down_ops->clear_irq();
		}
	case HALL_UP:		
		if (g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->enable_irq == NULL) {

			MOTOR_ERR("g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->enable_irq == NULL \n");
  			return -EINVAL;
 		} else {
 			return g_the_chip->hall_up_ops->clear_irq();
		}
		default:
			MOTOR_ERR("id : %d is not correct \n", id);
			return -EINVAL;		
	}

	return -EINVAL;
}

int oneplus_dhall_get_irq_state(unsigned int id)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return -EINVAL;
	}

	switch (id){
	case HALL_DOWN:		
		if (g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->get_irq_state == NULL) {

			MOTOR_ERR("g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->get_irq_state == NULL \n");
  			return -EINVAL;
 		} else {
 			return g_the_chip->hall_down_ops->get_irq_state();
		}
	case HALL_UP:		
		if (g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->get_irq_state == NULL) {

			MOTOR_ERR("g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->get_irq_state == NULL \n");
  			return -EINVAL;
 		} else {
 			return g_the_chip->hall_up_ops->get_irq_state();
		}
		default:
			MOTOR_ERR("id : %d is not correct \n", id);
			return -EINVAL;
	}

	return -EINVAL;
}

static int oneplus_hall_update_threshold(unsigned int id, int position, short lowthd, short highthd)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return -EINVAL;
	}

	MOTOR_LOG("lowthd : %d, highthd : %d, is_mag_positive : %d", lowthd, highthd, g_the_chip->is_mag_positive);
	//mag is negative
	if (!g_the_chip->is_mag_positive) {
		if (highthd != 511 ) {
			lowthd = -highthd;
			highthd = 511;
		}
	}

	switch (id){
	case HALL_DOWN:		
		if (g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->update_threshold == NULL) {

			MOTOR_ERR("g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->update_threshold == NULL \n");
  			return -EINVAL;
 		} else {
 			return g_the_chip->hall_down_ops->update_threshold(position,lowthd,highthd);
		}
	case HALL_UP:		
		if (g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->update_threshold == NULL) {

			MOTOR_ERR("g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->update_threshold == NULL \n");
  			return -EINVAL;
 		} else {
 			return g_the_chip->hall_up_ops->update_threshold(position,lowthd,highthd);
		}
		default:
			MOTOR_ERR("id : %d is not correct \n", id);
			return -EINVAL;
	}

	return -EINVAL;
}

void oneplus_dhall_dump_regs(unsigned int id, u8* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return;
	}

	switch (id){
	case HALL_DOWN:		
		if (g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->dump_regs == NULL) {

			MOTOR_ERR("g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->dump_regs == NULL \n");
  			return;
 		} else {
 			g_the_chip->hall_down_ops->dump_regs(buf);
		}
		break;
	case HALL_UP:		
		if (g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->dump_regs == NULL) {

			MOTOR_ERR("g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->dump_regs == NULL \n");
  			return;
 		} else {
 			g_the_chip->hall_up_ops->dump_regs(buf);
		}
		break;
		default:
			MOTOR_ERR("id : %d is not correct \n", id);
			return;
		
	}

	return;
}

int oneplus_dhall_set_reg(unsigned int id, int reg, int val)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return -EINVAL;
	}

	switch (id){
	case HALL_DOWN:		
		if (g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->set_reg == NULL) {

			MOTOR_ERR("g_the_chip->hall_down_ops == NULL || g_the_chip->hall_down_ops->set_reg == NULL \n");
  			return -EINVAL;
 		} else {
 			return g_the_chip->hall_down_ops->set_reg(reg,val);
		}
	case HALL_UP:		
		if (g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->set_reg == NULL) {

			MOTOR_ERR("g_the_chip->hall_up_ops == NULL || g_the_chip->hall_up_ops->set_reg == NULL \n");
  			return -EINVAL;
 		} else {
 			return g_the_chip->hall_up_ops->set_reg(reg,val);
		}
		default:
			MOTOR_ERR("id : %d is not correct \n", id);
			return -EINVAL;
	}

	return -EINVAL;
}

/*********************************************************************
            			step_motor control interface
**********************************************************************/
static int oneplus_motor_set_power(motor_power mode)
{
	if (g_the_chip == NULL || g_the_chip->motor_ops == NULL
			   || g_the_chip->motor_ops->set_power == NULL) {
		
		MOTOR_ERR("g_the_chip == NULL || g_the_chip->motor_ops == NULL || set_power == NULL");
  		return -EINVAL;
 	} else {
		MOTOR_ERR("oneplus_motor_set_power\n");
 		return g_the_chip->motor_ops->set_power(mode);
	}
}

static int oneplus_motor_set_direction (int dir)
{
	if (g_the_chip == NULL || g_the_chip->motor_ops->set_direction == NULL) {

		MOTOR_ERR("g_the_chip == NULL || g_the_chip->motor_ops->set_direction == NULL");
  		return -EINVAL;
 	} else {
 		return g_the_chip->motor_ops->set_direction(dir);
	}
}

static int oneplus_motor_set_working_mode (int mode)
{
	if (g_the_chip == NULL || g_the_chip->motor_ops == NULL 
			|| g_the_chip->motor_ops->set_working_mode == NULL) {

		MOTOR_ERR("g_the_chip == NULL || g_the_chip->motor_ops == NULL || set_working_mode == NULL");
  		return -EINVAL;
 	} else {
 		return g_the_chip->motor_ops->set_working_mode(mode);
	}

}

// static int oneplus_motor_calculate_pwm_count(int L, int mode)
// {
// 	if (g_the_chip == NULL || g_the_chip->motor_ops == NULL 
// 		|| g_the_chip->motor_ops->set_working_mode == NULL) {
		
// 		MOTOR_ERR("g_the_chip == NULL || g_the_chip->motor_ops == NULL || set_working_mode == NULL");
//   		return 0;
//  	} else {
//  		return g_the_chip->motor_ops->calculate_pwm_count(L, mode);
// 	}

// }

static int oneplus_motor_pwm_config(int duty_ns, int period_ns)
{
	MOTOR_LOG("duty_ns : %d, period_ns : %d\n", duty_ns, period_ns);
	if (g_the_chip == NULL || g_the_chip->motor_ops == NULL
			|| g_the_chip->motor_ops->calculate_pwm_count == NULL) {

		MOTOR_ERR("g_the_chip == NULL || g_the_chip->motor_ops == NULL || calculate_pwm_count == NULL");
  		return -EINVAL;
 	} else {
 		return g_the_chip->motor_ops->pwm_config(duty_ns, period_ns);
	}

}

static int oneplus_motor_pwm_enable(void)
{
	if (g_the_chip == NULL || g_the_chip->motor_ops == NULL
			|| g_the_chip->motor_ops->pwm_enable == NULL) {
		
		MOTOR_ERR("g_the_chip == NULL || g_the_chip->motor_ops == NULL || pwm_enable == NULL");
  		return -EINVAL;
 	} else {
 		return g_the_chip->motor_ops->pwm_enable();
	}

}

static int oneplus_motor_pwm_disable(void)
{
	if (g_the_chip == NULL || g_the_chip->motor_ops == NULL
		 || g_the_chip->motor_ops->pwm_disable == NULL) {

		MOTOR_ERR("g_the_chip == NULL || g_the_chip->motor_ops == NULL || pwm_disable == NULL");
  		return -EINVAL;
 	} else {
 		return g_the_chip->motor_ops->pwm_disable();
	}

}

int oneplus_motor_get_all_config(int* config ,int count)
{
	if (g_the_chip == NULL || g_the_chip->motor_ops == NULL 
		|| g_the_chip->motor_ops->get_all_config == NULL) {
		
		MOTOR_ERR("g_the_chip == NULL || g_the_chip->motor_ops == NULL || get_all_config == NULL");
  		return -EINVAL;
 	} else {
 		return g_the_chip->motor_ops->get_all_config(config,count);
	}

}


int oneplus_get_motor_type(void)
{
	if (g_the_chip == NULL || g_the_chip->motor_ops == NULL	
			 || g_the_chip->motor_ops->get_motor_type == NULL) {

		MOTOR_ERR("g_the_chip == NULL || g_the_chip->motor_ops == NULL || get_motor_type == NULL");
  		return MOTOR_UNKNOWN;
 	} else {
 		return g_the_chip->motor_ops->get_motor_type();
	}

}

static void oneplus_set_motor_work_mode_para(int motor_work_mode)
{
	int mode = 0;
	static int mode_pre = -1;

	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip == NULL \n");
		return;
	}

	if ((motor_work_mode >= MOTOR_MODE_FULL) && (motor_work_mode <= MOTOR_MODE_1_32)) {
		if (mode_pre != motor_work_mode) {
			mode_pre = motor_work_mode;
			g_the_chip->motor_work_mode = motor_work_mode;
		} else {
			return;//working mode not change
		}
	}

	switch (g_the_chip->motor_work_mode) {
		case MOTOR_MODE_FULL:
			oneplus_motor_set_working_mode(MOTOR_MODE_FULL);
			mode = 1;
			break;

		case MOTOR_MODE_1_16:
			oneplus_motor_set_working_mode(MOTOR_MODE_1_16);
			mode = 16;
			break;
		case MOTOR_MODE_1_32:
			oneplus_motor_set_working_mode(MOTOR_MODE_1_32);
			mode = 32;
			break;
		default:
			oneplus_motor_set_working_mode(MOTOR_MODE_1_32);
			mode = 32;
			break;
	}
	//default 32
	//g_the_chip->pwm_count = oneplus_motor_calculate_pwm_count(g_the_chip->whole_jonery_length, mode);//8.2mm
	//g_the_chip->pwm_count = g_the_chip->pwm_count*  32 / 20;
	//																			0.5mm
	//g_the_chip->speed_up_pwm_count = oneplus_motor_calculate_pwm_count(g_the_chip->speed_up_distance, mode);
	//																			 2mm
	//g_the_chip->speed_down_pwm_count = oneplus_motor_calculate_pwm_count(g_the_chip->speed_down_L, mode);
	//MOTOR_LOG("pwm_count %d,whole_jonery_length %d, speed_up_pwm_count %d, mode %d\n",
	//		   g_the_chip->pwm_count, g_the_chip->whole_jonery_length,mode);
}

/*********************************************************************
            combine digital_hall and step_motor for logical process
**********************************************************************/
static void manual_to_auto_up_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
  	struct oneplus_motor_chip *chip = container_of(dwork, struct oneplus_motor_chip, up_work);

	oneplus_motor_set_awake(chip,HALL_DATA_LOCK,true);//dont sleep
	oneplus_motor_irq_monitor(chip);

	MOTOR_LOG("call");
	if (atomic_read(&chip->in_suspend)) {
		MOTOR_ERR("in_suspend delay 20 ms \n");
		queue_delayed_work(chip->manual2auto_wq, &chip->up_work, msecs_to_jiffies(20));
		return;
	}

	oneplus_hall_update_threshold(HALL_DOWN, MID_STATE, 511, 511);

	oneplus_hall_clear_irq(HALL_DOWN);

	if (!chip->is_irq_abnormal)
		oneplus_hall_enable_irq(HALL_DOWN, true);

	oneplus_motor_set_awake(chip, HALL_DATA_LOCK, false);
}

static void manual_to_auto_down_work(struct work_struct* work)
{
	struct delayed_work* dwork = to_delayed_work(work);
  	struct oneplus_motor_chip* chip = container_of(dwork, struct oneplus_motor_chip, down_work);

	oneplus_motor_set_awake(chip,HALL_DATA_LOCK,true);//dont sleep
	oneplus_motor_irq_monitor(chip);

	MOTOR_LOG("call \n");

	if (atomic_read(&chip->in_suspend)) {
		MOTOR_ERR("in_suspend delay 20 ms \n");
		queue_delayed_work(chip->manual2auto_wq,&chip->down_work,msecs_to_jiffies(20));
		return;
	}

	oneplus_hall_update_threshold(HALL_UP, MID_STATE, 511, 511);

	if (chip->manual2auto_down_switch) {
		if (!chip->motor_started) {
			report_position_state(chip, MANUAL_TO_DOWN_EVENT);
			oneplus_motor_downward();
		}
	}

	oneplus_hall_clear_irq(HALL_UP);

	if (!chip->is_irq_abnormal)
		oneplus_hall_enable_irq(HALL_UP, true);

	oneplus_motor_set_awake(chip, HALL_DATA_LOCK, false);

	return;
}

static void oneplus_set_motor_speed(int speed)
{
	long long period_ns = 0;
	unsigned long duty_ns = 0;

	

	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip == NULL \n");
		return;
	}

	g_the_chip->motor_speed = speed;

	MOTOR_LOG("call, speed : %d , chip->motor_speed : %d \n", speed, g_the_chip->motor_speed);
	
	switch (g_the_chip->motor_speed) {
	case MOTOR_SPEED0:
		period_ns = 12000;// 80KHZ
		break;

	case MOTOR_SPEED1:
		period_ns = 15500;// 64KHZ
		break;

	case MOTOR_SPEED2:
		period_ns = 19531;// 51.2HZ
		break;

	case MOTOR_SPEED3:
		period_ns = 26041;// 38.4KHZ
		break;

	case MOTOR_SPEED4:
		period_ns = 31250;// 32KHZ
		break;

	case MOTOR_SPEED5:
		period_ns = 39062;// 25.6KHZ
		break;

	case MOTOR_SPEED6:
		period_ns = 44642;// 22.4KHZ
		break;

	case MOTOR_SPEED7:
		period_ns = 52083;// 19.2KHZ
		break;

	case MOTOR_SPEED8:
		period_ns = 60240;// 16.6KHZ
		break;

	case MOTOR_SPEED9:
		period_ns = 78125;// 12.8KHZ
		break;

	case MOTOR_SPEED10:
		period_ns = 104166;// 9.6KHZ
		break;

	case MOTOR_SPEED11:
		period_ns = 156250;// 6.4KHZ
		break;

	case MOTOR_SPEED12:
		period_ns = 312500;// 3.2KHZ
		break;

	case MOTOR_SPEED13:
		period_ns = 625000;// 1.6KHZ
		break;
	case MOTOR_SPEED_SPECIAL:
		period_ns = 11161;//2800pps, only use in free fall
		break;

	default:
		period_ns = 12000;// 80KHZ.
		break;

	}
	
	duty_ns = (unsigned long)(period_ns/2);
	g_the_chip->pwm_duty = duty_ns;
	g_the_chip->pwm_period = period_ns;
	MOTOR_LOG("pwm_duty : %d, pwm_period : %d", g_the_chip->pwm_duty, g_the_chip->pwm_period);

	return;
}

static void oneplus_change_motor_speed(int speed)
{
	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip == NULL \n");
		return;
	}

	if (g_the_chip->motor_speed == speed) {
		MOTOR_LOG("speed not change");
		return;
	}

	MOTOR_LOG("call, speed : %d , chip->motor_speed : %d \n", speed, g_the_chip->motor_speed);

	oneplus_set_motor_speed(speed);

	mutex_lock(&motor_running_mutex);
	if (g_the_chip->motor_started) {
		oneplus_motor_pwm_disable();
		oneplus_motor_pwm_config(g_the_chip->pwm_duty, g_the_chip->pwm_period);
		oneplus_motor_pwm_enable();
	}
	mutex_unlock(&motor_running_mutex);

	return;
}

static void oneplus_set_motor_direction(int direction)
{
	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip == NULL \n");
		return;
	}

	if (direction >= 0)
		g_the_chip->motor_direction = !!direction;
}

static void oneplus_set_motor_move_state(int move_state)
{
	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip == NULL \n");
		return;
	}

	if ((move_state >= MOTOR_STOP) && (move_state <= MOTOR_DOWNWARD_STOP)) {
		g_the_chip->move_state = move_state;
	}
}

static bool oneplus_motor_run_check(struct oneplus_motor_chip* chip)
{
	//TO DO : remove temporary, for debug

	if (chip->is_skip_pos_check) {
		MOTOR_ERR("skip_pos_check \n");
		return true;
	}

	if ((chip->position == PEAK_STATE) && (chip->motor_direction == MOTOR_UPWARD)) {
		MOTOR_LOG("has been in up_state, return false\n");
		return false;
	} else if ((chip->position == BOTTOM_STATE) && (chip->motor_direction == MOTOR_DOWN)) {
		MOTOR_LOG("has been in down_state, return false\n");
		return false;
	}

	MOTOR_LOG("oneplus_motor_run_check ok\n");
	return true;
}

static void oneplus_motor_control(int on ,int speed ,int direction)
{
	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip == NULL \n");
		return;
	}

	MOTOR_LOG("on = %d, speed = %d, direction = %d, motor_switch : %d \n",
								on, speed, direction, g_the_chip->motor_switch);

	if (on) {
		
		if (g_the_chip->motor_switch == 0) {
			MOTOR_ERR("motor_switch == 0, would not set motor start \n");
		 	return;
		}
		if (!g_the_chip->motor_started) {
			MOTOR_LOG("set motor_enable = 1");
			g_the_chip->motor_enable = 1;

			//enable hall
			if (!g_the_chip->is_free_fall) {
				if (!oneplus_hall_get_enable_state(HALL_DOWN))
					oneplus_hall_set_enable_state(HALL_DOWN, true);

				if (!oneplus_hall_get_enable_state(HALL_UP))
					oneplus_hall_set_enable_state(HALL_UP, true);
			}


			oneplus_set_motor_speed(speed);
			oneplus_set_motor_direction(direction);
			if (oneplus_motor_run_check(g_the_chip)) {
				queue_work(g_the_chip->motor_run_work_wq, &g_the_chip->motor_work);
			}
		}
	} else {
		if (g_the_chip->motor_started) {
			MOTOR_LOG("set motor_enable = 0");
			g_the_chip->motor_enable = 0;
			queue_work(g_the_chip->motor_run_work_wq, &g_the_chip->motor_work);
		}
	}

	return;
}

static void oneplus_motor_start(void)
{
	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip ==NULL \n");
		return;
	}

	oneplus_motor_control(1, g_the_chip->motor_speed, g_the_chip->motor_direction);

	return;
}

static void oneplus_motor_stop(void)
{
	int on = 0;
	int speed = 0;
	int direction = 0;

	MOTOR_LOG("call \n");
	oneplus_motor_control(on, speed, direction);

	return;
}

static void oneplus_motor_downward(void)
{
	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip == NULL \n");
		return;
	}

	oneplus_motor_control(1, g_the_chip->motor_speed, MOTOR_DOWN);
}

static void motor_run_work(struct work_struct* work)
{
	struct oneplus_motor_chip* chip = container_of(work, struct oneplus_motor_chip, motor_work);
	unsigned long intsecond = 0;
	unsigned long nsecond = 0;
	long long value = 0;
	int err = 0;

	mutex_lock(&motor_running_mutex);
	MOTOR_LOG("%s, motor_enable : %d, motor_started : %d",
						 __func__, chip->motor_enable, chip->motor_started);

	MOTOR_LOG("is_factory_mode : %d \n", chip->is_factory_mode);

	if (chip->motor_enable && (chip->motor_started == 0)) {
		mutex_lock(&motor_start_mutex);

		MOTOR_LOG("start motor\n");

	    oneplus_motor_set_awake(chip,MOTOR_RUN_LOCK, true);
		oneplus_motor_set_power(MOTOR_POWER_ON);
		oneplus_motor_set_direction(chip->motor_direction);

		if (!g_the_chip->is_motor_test) {
			if (chip->motor_direction == MOTOR_UPWARD)
				oneplus_hall_update_threshold(HALL_DOWN, BOTTOM_STATE, 511, 511);
			else if (chip->motor_direction == MOTOR_DOWN)
				oneplus_hall_update_threshold(HALL_UP, BOTTOM_STATE, 511, 511);
		} else if (chip->motor_direction == MOTOR_UPWARD) {
			MOTOR_LOG("motor test, update hall down irq threshold");
			oneplus_hall_update_threshold(HALL_DOWN,BOTTOM_STATE,-512,chip->hall_down_irq_position);
		}

		if (chip->is_free_fall) {
			MOTOR_ERR("phone is falling, use special speed");
			oneplus_set_motor_speed(MOTOR_SPEED_SPECIAL);
		} else if (chip->is_factory_mode == 1){
			oneplus_set_motor_speed(MOTOR_SPEED0);//calibrate mode
		} else {
			oneplus_set_motor_speed(MOTOR_SPEED10);//normal mode and step test mode
		}
	
		err = oneplus_motor_pwm_config(chip->pwm_duty, chip->pwm_period);
		if (err < 0) {
			MOTOR_ERR("pwm_config failed, err : %d \n",err);
			chip->motor_started = 0;
			mutex_unlock(&motor_running_mutex);
			mutex_unlock(&motor_start_mutex);
			return;
		}

		err = oneplus_motor_pwm_enable();
		if (err < 0) {
			MOTOR_ERR("pwm_enable failed, err : %d \n",err);
			chip->motor_started = 0;
			mutex_unlock(&motor_running_mutex);
			mutex_unlock(&motor_start_mutex);
			return;
		} 

		if (chip->motor_direction == MOTOR_UPWARD) {
			if (chip->is_factory_mode == 0)
				report_position_state(chip, UPING_EVENT);

			oneplus_set_motor_move_state(MOTOR_UPWARD_ING);
		} else if (chip->motor_direction == MOTOR_DOWN) {
			if (chip->is_factory_mode == 0)
				report_position_state(chip, DOWNING_EVENT);

			oneplus_set_motor_move_state(MOTOR_DOWNWARD_ING);
		}
		chip->motor_started = 1;


		chip->camera_position_detect = true;

		do_gettimeofday(&chip->motor_start_time);
		MOTOR_LOG("motor_start_time tv_sec : %lu, tv_usec : %lu \n", chip->motor_start_time.tv_sec, chip->motor_start_time.tv_usec);
		
		//calculate when the motor should stop
		if (chip->is_factory_mode == 1) {
			if (chip->motor_direction == 1) {//camera up
				value = chip->camera_up_step_count * MOTOR_STOP_TIMEOUT;

				MOTOR_LOG("motor up, use step count, value : %d", value);

				nsecond = do_div(value, 1000000000);//value = value/1000000000 ,nsecond = value % 1000000000
				intsecond = (unsigned long) value;

				chip->whole_jonery_time = nsecond / 1000;//ns to ms
			} else {
				MOTOR_LOG("motor up, set timeout as 1s");
			    value = 1000000000;
			
			    nsecond = do_div(value, 1000000000);
			    intsecond = (unsigned long) value;
			}
		} else {
			if (chip->motor_direction == 1) {//camera up
				value = (chip->speed_up_pwm_count * chip->pwm_period)+
						 ((chip->camera_up_step_count - chip->speed_up_pwm_count) * MOTOR_STOP_TIMEOUT);

				MOTOR_LOG("motor up, use step count, value : %d", value);

				nsecond = do_div(value, 1000000000);//value = value/1000000000 ,nsecond = value % 1000000000
				intsecond = (unsigned long) value;

				chip->whole_jonery_time = nsecond / 1000;//ns to ms
			} else { //camera down
				value = (chip->speed_up_pwm_count * chip->pwm_period)+
						 ((chip->camera_up_step_count - chip->speed_up_pwm_count + 20 * 32) * MOTOR_STOP_TIMEOUT);

				MOTOR_LOG("motor up, use step count, value : %d", value);

				nsecond = do_div(value, 1000000000);//value = value/1000000000 ,nsecond = value % 1000000000
				intsecond = (unsigned long) value;

				chip->whole_jonery_time = nsecond / 1000;//ns to ms
			} 
		}

		//else {
			//MOTOR_LOG("motor up, set timeout as 1s");
			//value = 1000000000;
			
			//nsecond = do_div(value, 1000000000);
			//intsecond = (unsigned long) value;
		//}

		MOTOR_LOG("time value = %llu nsecond = %lu intsecond = %lu , whole_jonery_time : %lu \n", 
		                value, nsecond, intsecond, chip->whole_jonery_time);
		hrtimer_start(&chip->stop_timer, ktime_set(intsecond, nsecond), HRTIMER_MODE_REL);

		//calculate when the motor should speed up
		value = chip->speed_up_pwm_count * chip->pwm_period;
		nsecond = do_div(value, 1000000000);//value = value/1000000000 ,nsecond = value % 1000000000
		intsecond = (unsigned long) value;
		MOTOR_LOG("time value = %llu nsecond = %lu intsecond = %lu, chip->speed_up_pwm_count = %d,chip->pwm_period = %d \n", 
		                value, nsecond, intsecond, chip->speed_up_pwm_count, chip->pwm_period);
		hrtimer_start(&chip->speed_up_timer, ktime_set(intsecond, nsecond), HRTIMER_MODE_REL);
		mutex_unlock(&motor_start_mutex);
	} else if (!chip->motor_enable && (chip->motor_started == 1)) {
		MOTOR_LOG("stop motor \n");

		oneplus_motor_pwm_disable();
		oneplus_motor_set_power(MOTOR_POWER_OFF);

		if (!chip->stop_timer_trigger)
			hrtimer_cancel(&chip->stop_timer);
		else
			chip->stop_timer_trigger = 0;

		chip->motor_started = 0;

		if (chip->move_state == MOTOR_UPWARD_ING) {
			oneplus_set_motor_move_state(MOTOR_UPWARD_STOP);
			oneplus_hall_get_abs_data(HALL_UP);
			MOTOR_LOG("move_state == MOTOR_UPWARD_ING, hall_up_data : %d, hall_up_irq_position : %d"
							        ,chip->hall_up_data, chip->hall_up_irq_position);

			if (chip->hall_up_data > chip->hall_up_irq_position) {
				MOTOR_ERR("%s, position = PEAK_STATE", __func__);
				chip->position = PEAK_STATE;
				if (chip->is_factory_mode == 0)
					report_position_state(chip, UP_NORMAL_EVENT);

				MOTOR_ERR("POS_NORMAL, hall_up_irq_count %d\n",chip->hall_up_irq_count);
				chip->hall_up_irq_count = 0;
				oneplus_motor_notify_state(MOTOR_UP_EVENT);
				oneplus_hall_update_threshold(HALL_UP,PEAK_STATE,-512,chip->hall_up_irq_position);
			} else if (!chip->force_move) {
				chip->position = MID_STATE;
				if (chip->is_factory_mode == 0)
					report_position_state(chip, UP_ABNORMA_EVENT);

				MOTOR_ERR("POS_ABNORMAL %d %d\n",chip->hall_up_data ,chip->hall_up_irq_position);
				oneplus_motor_notify_state(MOTOR_BLOCK_EVENT);
			} else {
				MOTOR_ERR("POS_ABNORMAL %d %d, but it is force_move mode, would not report \n",
									    chip->hall_up_data ,chip->hall_up_irq_position);
			}

		} else if (chip->move_state == MOTOR_DOWNWARD_ING) {
			oneplus_set_motor_move_state(MOTOR_DOWNWARD_STOP);
			oneplus_hall_get_abs_data(HALL_DOWN);
			MOTOR_LOG("move_state == MOTOR_DOWNWARD_ING, hall_down_data : %d, hall_down_irq_position : %d"
								    ,chip->hall_down_data, chip->hall_down_irq_position);

			if (chip->hall_down_data > chip->hall_down_irq_position) {
				MOTOR_ERR("%s, position = BOTTOM_STATE", __func__);
				chip->position = BOTTOM_STATE;
				if (chip->is_factory_mode == 0)
					report_position_state(chip, DOWN_NORMAL_EVENT);

				MOTOR_ERR("POS_NORMAL, hall_down_irq_count %d\n",chip->hall_down_irq_count);
				chip->hall_down_irq_count = 0;
				oneplus_motor_notify_state(MOTOR_DOWN_EVENT);
				//oneplus_hall_update_threshold(HALL_DOWN,BOTTOM_STATE,-512,chip->hall_down_irq_position);
			} else if (!chip->force_move){
				chip->position = MID_STATE;
				if (chip->is_factory_mode == 0)
					report_position_state(chip, DOWN_ABNORMAL_EVENT);

				MOTOR_ERR("POS_ABNORMAL %d %d \n",chip->hall_down_data ,chip->hall_down_irq_position);
				oneplus_motor_notify_state(MOTOR_BLOCK_EVENT);
			} else {
				MOTOR_ERR("POS_ABNORMAL %d %d, but it is force_move mode, would not report \n",
										chip->hall_down_data ,chip->hall_down_irq_position);
			}
		}

		//set motor starting speed after motor stop
		oneplus_set_motor_speed(MOTOR_SPEED10);

		if (chip->save_hall_data_to_file)
			chip->save_hall_data_to_file = false;

		if (chip->force_move)
			chip->force_move = false;//reset to normal mode
		
		if (chip->is_free_fall)
			chip->is_free_fall = false;
		
		//chip->slow_down_speed = MOTOR_SPEED11;

		//disable hall when down finish for save power
		//remove temporary
		// if (g_the_chip->motor_direction == MOTOR_DOWN && chip->position == BOTTOM_STATE) {
		// 	if (oneplus_hall_get_enable_state(HALL_DOWN))
		// 		oneplus_hall_set_enable_state(HALL_DOWN, false);
	
		// 	if (oneplus_hall_get_enable_state(HALL_UP))
		// 		oneplus_hall_set_enable_state(HALL_UP, false);		 
		// }

		oneplus_motor_set_awake(chip,MOTOR_RUN_LOCK,false);

		chip->camera_position_detect = false;//stop camera position detect
	}

	mutex_unlock(&motor_running_mutex);
}

static void  free_fall_irq_check_work_func(struct work_struct* work)
{
	struct delayed_work* dwork = to_delayed_work(work);
	struct oneplus_motor_chip* chip = container_of(dwork, struct oneplus_motor_chip, free_fall_irq_check_work);

	if (chip == NULL) {
		MOTOR_ERR("free_fall_irq_check_work_func error, chip == NULL");
		return;
	}

	MOTOR_LOG("free_fall_irq_times : %d, infrared_shut_down_state : %d \n", chip->free_fall_irq_times, chip->infrared_shut_down_state);

	if (chip->free_fall_irq_times == 1) {
		g_the_chip->is_free_fall = true;
		disable_irq_nosync(g_the_chip->free_fall_irq);
		oneplus_motor_downward();
		enable_irq(g_the_chip->free_fall_irq);

	} else if (chip->free_fall_irq_times >= 2) {
		MOTOR_LOG("infrared notify event, free_fall_irq_times : %d \n", chip->free_fall_irq_times);
		chip->infrared_shut_down_state = 1;

	} else {
		MOTOR_LOG("unknow event, free_fall_irq_times : %d \n", chip->free_fall_irq_times);
	}

	chip->free_fall_irq_times = 0;

	return;
}
static void  camera_position_detect_work(struct work_struct* work)
{
  	int            hall_delta_pre = 0;
  	int            hall_delta = 0;
  	int            delta_d = 0;
	int            hall_down_data_pre = 0;
	int            hall_up_data_pre = 0;
  	int            speed_down = 0;
  	int            should_stop_count = 0;
	int            up_stop_times = 0;
	int            down_stop_times = 0;
	int            abnormal_judge_time = 400000;//400ms default
	int            data_count = 0;
	int            deltad_range_low = -5;
	int            deltad_range_high = 5;
	unsigned long  enter_deltad_first_time = 0;
	bool           mag_noise = false;
	bool           append_to_line = false;
	short          hall_up_data[64] = {0};
	short          hall_down_data[64] = {0};
	struct         delayed_work* dwork = to_delayed_work(work);
  	struct         oneplus_motor_chip* chip = container_of(dwork, struct oneplus_motor_chip, detect_work);
	unsigned long  distance_time = 0;
	struct timeval current_time;
	

	if (chip == NULL) {
		MOTOR_ERR("camera_position_detect_work error, chip == NULL");
		return;
	}

	mutex_lock(&position_detect_mutex);
	mutex_lock(&motor_start_mutex);
	oneplus_motor_set_awake(chip,POSITION_DETECT_LOCK,true);//should not be sleep during detecting
	
	if (chip->move_state == MOTOR_UPWARD_ING)
		chip->stall_steps = chip->camera_up_step_count / 32;
	else
		chip->stall_steps = 0;

	deltad_range_low = 0 - chip->deltad_range;
	deltad_range_high = chip->deltad_range;
	chip->is_stall = 0;
	chip->stall_mode = UNKNOW_MODE;
	up_stop_times = 2;
	down_stop_times = 1;
	abnormal_judge_time = chip->whole_jonery_time * chip->begin_stop_detect_percent / 100;
	MOTOR_LOG("motor_diretion : %d , move_state : %d motor_started : %d, abnormal_judge_time : %d\n",
	            chip->motor_direction, chip->move_state, chip->motor_started, abnormal_judge_time);
	MOTOR_LOG("camera_down_slow_down_position_hall_down_data : %d, camera_down_slow_down_position_hall_up_data : %d\n", 
		       chip->camera_down_slow_down_position_hall_down_data, chip->camera_down_slow_down_position_hall_up_data);
	MOTOR_LOG("deltad_range_low : %d, deltad_range_high : %d \n", deltad_range_low, deltad_range_high);
	MOTOR_LOG("stall_steps : %d, begin_stop_detect_percent : %d \n", chip->stall_steps, chip->begin_stop_detect_percent);

	oneplus_change_motor_speed(MOTOR_SPEED0);

    while (chip->camera_position_detect) {
		hall_down_data_pre = chip->hall_down_data;
		hall_up_data_pre = chip->hall_up_data;
    	oneplus_hall_get_abs_data(HALL_DOWN);
    	oneplus_hall_get_abs_data(HALL_UP);
		hall_up_data[data_count] = chip->hall_down_data;
		hall_down_data[data_count] = chip->hall_up_data;
		data_count++;

		//may be mag noise or speed is very slow
		if (chip->save_hall_data_to_file && data_count >= 60) {
			MOTOR_LOG("may be mag noise!!!, data_count : %d", data_count);
			mag_noise = true;

			write_hall_data_to_file(hall_up_data, hall_down_data, data_count, chip->motor_direction, false);

			data_count = 0;
			memset(hall_up_data, 0, sizeof(hall_up_data));
			memset(hall_down_data, 0, sizeof(hall_down_data));
		} else if (data_count >= 60) {
			MOTOR_LOG("may be mag noise!!!, data_count : %d", data_count);
			data_count = 0;
		}
	
    	hall_delta = chip->hall_down_data - chip->hall_up_data;
    	delta_d = hall_delta - hall_delta_pre;
    	hall_delta_pre = hall_delta;

    	MOTOR_LOG("dhall_down data : %d, dhall_up data : %d , hall_delta : %d, delta_d : %d \n",
    			   chip->hall_down_data, chip->hall_up_data, hall_delta, delta_d);

		//if force_move mode, then we don't use brake mechanism mode, use time out mode
		if (chip->force_move) {
			mdelay(chip->position_detect_delay);	
			continue;
		}

		//if free fall, then we don't use brake mechanism mode, use time out mode
		if (chip->is_free_fall) {
			MOTOR_ERR("phone is falling");
			mdelay(chip->position_detect_delay);	
			continue;
		}

		//camera down
    	if ((chip->hall_down_data - chip->hall_up_data) > (chip->camera_down_slow_down_position_hall_down_data - chip->camera_down_slow_down_position_hall_up_data)) {
    		if ((chip->move_state == MOTOR_DOWNWARD_ING) && (speed_down == 0)) {
    		   	speed_down = 1;
				oneplus_change_motor_speed(chip->slow_down_speed);
    		}
    	 } 

	    do_gettimeofday(&current_time);
		distance_time = (current_time.tv_sec - chip->motor_start_time.tv_sec) * 1000000 + 
		                (current_time.tv_usec - chip->motor_start_time.tv_usec);
		MOTOR_LOG("distance_time : %d, current_time.tv_sec : %lu, current_time.tv_usec : %lu", 
		           distance_time, (unsigned long)current_time.tv_sec, (unsigned long)current_time.tv_usec);

    	//stop motor algo
		MOTOR_LOG("should_stop_count : %d, is_stall : %d, up_stop_times : %d, down_stop_times : %d", 
		           should_stop_count, chip->is_stall, up_stop_times, down_stop_times);
				
    	if ((distance_time >= abnormal_judge_time) && (delta_d >= deltad_range_low) && (delta_d <= deltad_range_high)) {
				should_stop_count ++;
			
			if (enter_deltad_first_time == 0) {
				enter_deltad_first_time = distance_time;
			}
			MOTOR_LOG("enter deltad_range, enter_deltad_first_time :%lu", enter_deltad_first_time);
			
    	}

		//when calibrate mode, motor down stop by hall, else stop by steps
		if ((should_stop_count >= down_stop_times) && (chip->move_state == MOTOR_DOWNWARD_ING) 
												   && (chip->is_factory_mode == 1)) {
			oneplus_motor_stop();

    		break; 
    	} else if ((should_stop_count >= up_stop_times) && (chip->move_state == MOTOR_UPWARD_ING)) {
			chip->stall_mode = ENTER_DELTAD_RANGE_TWO_TIEMS;
			chip->is_stall = 1;
			chip->stall_steps = enter_deltad_first_time / 384;//(enter_deltad_first_time / 12) / 32;
			MOTOR_LOG("should_stop_count >= up_stop_times, stall_steps : %d \n", chip->stall_steps);
		}

		//anti-shake when motor down
		MOTOR_LOG("hall_down_data_pre : %d, hall_down_data : %d \n", hall_down_data_pre, chip->hall_down_data);
		if ( (distance_time >= abnormal_judge_time) && (chip->move_state == MOTOR_DOWNWARD_ING) &&
			 (hall_down_data_pre > chip->hall_down_data) && (chip->is_factory_mode == 1)) {
			
			MOTOR_LOG("hall_down_data_pre > chip->hall_down_data, may be structure shake, stop motor");
			oneplus_motor_stop();
    		break; 
		}

		//anti-shake when motor up
		if ( (distance_time >= abnormal_judge_time) && (chip->move_state == MOTOR_UPWARD_ING) &&
			                                           (hall_up_data_pre > chip->hall_up_data)) {
			
			MOTOR_LOG("hall_up_data_pre > chip->hall_up_data");
		

			if (enter_deltad_first_time != 0) {
				chip->stall_steps = enter_deltad_first_time / 384;//(enter_deltad_first_time / 12) / 32;
				chip->stall_mode = ENTER_DELTAD_AND_SHAKE;
			} else {
				chip->stall_steps = distance_time / 384;//(distance_time / 12) / 32;
				chip->stall_mode = ONLY_SHAKE;
			}
			chip->is_stall = 1;
			MOTOR_LOG("chip->stall_steps : %d, distance_time : %lu, enter_deltad_first_time : %lu \n", 
			           chip->stall_steps, distance_time, enter_deltad_first_time);
    		break; 
		}

		mdelay(chip->position_detect_delay);//15ms
    }

	oneplus_motor_set_awake(chip,POSITION_DETECT_LOCK,false);

	MOTOR_LOG("chip->stall_mode : %d \n", chip->stall_mode);
	if (chip->save_hall_data_to_file && data_count > 0) {
		append_to_line = mag_noise ? true : false;
		write_hall_data_to_file(hall_up_data, hall_down_data, data_count, chip->motor_direction, append_to_line);
	}

	mutex_unlock(&position_detect_mutex);
	mutex_unlock(&motor_start_mutex);

	return;
}

static int write_hall_data_to_file(short* hall_up_data, short* hall_down_data, int buf_len, int direction, bool append)
{
	int  fd = -1;
	int  i = 0;
	int  data_num = 0;
	char hall_data_bufs[1024] = {0};// 16 * 64
	char hall_data_buf[16] = {0};

	if (hall_up_data == NULL || hall_down_data == NULL || buf_len < 1) {
		MOTOR_ERR("write_hall_data_to_file failed, hall_up_data == NULL || hall_down_data == NULL || buf_len < 1");
		return -EINVAL;
	}

	fd = sys_open("/sdcard/hall_data.csv",  O_WRONLY | O_CREAT | O_APPEND, 0);
	if (fd < 0) {
		MOTOR_ERR("open log file /sdcard/hall_data.csv failed, fd : %d \n", fd);
		return -1;
	}
	
	data_num = buf_len > 60 ? 60 : buf_len;
	if (!append) {
		if (direction) 
			sys_write(fd, "1\n", 2);//up
		else 
			sys_write(fd, "0\n", 2);//down
	}

	for (i = 0; i < data_num; i++) {
		memset(hall_data_buf, 0, sizeof(hall_data_buf));
		sprintf(hall_data_buf, "%d,", hall_up_data[i]);
		strcat(hall_data_bufs, hall_data_buf);
	}
	strcat(hall_data_bufs, "\n");
	sys_write(fd, hall_data_bufs, strlen(hall_data_bufs));
	memset(hall_data_bufs, 0 , sizeof(hall_data_bufs));

	for (i = 0; i < data_num; i++) {
		memset(hall_data_buf, 0, sizeof(hall_data_buf));
		sprintf(hall_data_buf, "%d,", hall_down_data[i]);
		strcat(hall_data_bufs, hall_data_buf);
	}
	strcat(hall_data_bufs, "\n");
	sys_write(fd, hall_data_bufs, strlen(hall_data_bufs));
	sys_write(fd, "\n", 1);

	sys_close(fd);
	return 0;
}

static enum hrtimer_restart motor_stop_timer_func(struct hrtimer* hrtimer)
{
	MOTOR_LOG("force_move : %d \n", g_the_chip->force_move);

	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip == NULL \n");
		return HRTIMER_NORESTART;
	}

	if (g_the_chip->force_move){
		MOTOR_LOG("force_move is enable, would not stop motor");
		return HRTIMER_NORESTART;
	}
	g_the_chip->stop_timer_trigger = 1;

	oneplus_motor_stop();
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart motor_speed_up_timer_func(struct hrtimer* hrtimer)
{
	MOTOR_LOG("call \n");

	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip == NULL \n");
		return HRTIMER_NORESTART;
	}

	mod_delayed_work(system_highpri_wq, &g_the_chip->detect_work, 0);

	return HRTIMER_NORESTART;
}

static enum alarmtimer_restart motor_reset_timer_func(struct alarm* alrm, ktime_t now)
{
	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip == NULL \n");
		return ALARMTIMER_NORESTART;
	}

	MOTOR_LOG("hall_down_irq_count %d hall_up_irq_count %d \n",g_the_chip->hall_down_irq_count,g_the_chip->hall_up_irq_count);

	if ((g_the_chip->hall_down_irq_count >= MOTOR_IRQ_MONITOR_COUNT) ||
		(g_the_chip->hall_up_irq_count >= MOTOR_IRQ_MONITOR_COUNT)) {
		MOTOR_ERR("irq abnormal,reset \n");
		g_the_chip->is_irq_abnormal = true;
		g_the_chip->motor_switch = 0;
		g_the_chip->hall_down_irq_count = 0;
		g_the_chip->hall_up_irq_count = 0;
	}
	g_the_chip->irq_monitor_started = false;

	return ALARMTIMER_NORESTART;
}

static void oneplus_motor_set_awake(struct oneplus_motor_chip* chip, int id ,bool awake)
{
	static int awake_count = 0;
	static int wakelock_holder = 0;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	if (id >= MAX_LOCK)
		return;

	if (awake && !awake_count) {
		if (!(wakelock_holder & (1 << id))) {//this holder have not this lock now
			wakelock_holder |= (1 << id);
			awake_count ++;
			wake_lock(&chip->suspend_lock);
			MOTOR_LOG("awake \n");
		}
	} else if (!awake && (awake_count == 1)) {//only one lock hold
		if (wakelock_holder & (1 << id)) {
			wakelock_holder &= ~(1 << id);
			awake_count = 0;
			wake_unlock(&chip->suspend_lock);
			MOTOR_LOG("relax \n");
		}
	} else if (!awake) {
		if (awake_count != 0) {
			if (wakelock_holder & (1 << id)) {
				awake_count --;
				wakelock_holder &= ~(1 << id);
			}
		}
	} else {
		if (!(wakelock_holder & (1 << id))) {//this holder have not this lock now
			awake_count ++;
			wakelock_holder |= (1 << id);
		}
	}
	//MOTOR_LOG("awake_count %d wakelock_holder %d\n",awake_count,wakelock_holder);
#else
	if (!chip->suspend_ws)
		return;

	if (id >= MAX_LOCK)
		return;

	if (awake && !awake_count) {
		if (!(wakelock_holder & (1 << id))) {//this holder have not this lock now
			wakelock_holder |= (1 << id);
			awake_count ++;
			__pm_stay_awake(chip->suspend_ws);//not alow system suspend
			
			MOTOR_LOG("awake \n");
		}
	} else if (!awake && (awake_count == 1)) {//only one lock hold
		if (wakelock_holder & (1 << id)) {
			wakelock_holder &= ~(1 << id);
			awake_count = 0;
			__pm_relax(chip->suspend_ws);
			MOTOR_LOG("relax \n");
		}
	} else if (!awake) {
		if (awake_count != 0) {
			if (wakelock_holder & (1 << id)) {
				awake_count --;
				wakelock_holder &= ~(1 << id);
			}
		}
	} else {
		if (!(wakelock_holder & (1 << id))) {//this holder have not this lock now
			awake_count ++;
			wakelock_holder |= (1 << id);
		}
	}
	MOTOR_LOG("awake_count %d wakelock_holder %d \n", awake_count, wakelock_holder);
#endif
}

static void report_position_state(struct oneplus_motor_chip* chip, camera_position_state_event state_event)
{
	if (chip == NULL) {
		MOTOR_ERR("chip == NULL \n");
		return;
	}

	MOTOR_LOG("call, state_event : %d", state_event);
	
	switch (state_event) {
	case MANUAL_TO_DOWN_EVENT:
		input_report_key(chip->input_dev, MOTOR_EVENT_MANUAL_TO_DOWN, 1);
		input_sync(chip->input_dev);
		input_report_key(chip->input_dev, MOTOR_EVENT_MANUAL_TO_DOWN, 0);
		input_sync(chip->input_dev);
		break;
	case UPING_EVENT:
		input_report_key(chip->input_dev, MOTOR_EVENT_UP, 1);
		input_sync(chip->input_dev);
		input_report_key(chip->input_dev, MOTOR_EVENT_UP, 0);
		input_sync(chip->input_dev);
		break;
	case UP_ABNORMA_EVENT:
		input_report_key(chip->input_dev, MOTOR_EVENT_UP_ABNORMAL, 1);
		input_sync(chip->input_dev);
		input_report_key(chip->input_dev, MOTOR_EVENT_UP_ABNORMAL, 0);
		input_sync(chip->input_dev);
		break;
	case UP_NORMAL_EVENT:
		input_report_key(chip->input_dev, MOTOR_EVENT_UP_NORMAL, 1);
		input_sync(chip->input_dev);
		input_report_key(chip->input_dev, MOTOR_EVENT_UP_NORMAL, 0);
		input_sync(chip->input_dev);
		break;
	case DOWNING_EVENT:
		input_report_key(chip->input_dev, MOTOR_EVENT_DOWN, 1);
		input_sync(chip->input_dev);
		input_report_key(chip->input_dev, MOTOR_EVENT_DOWN, 0);
		input_sync(chip->input_dev);
		break;
	case DOWN_ABNORMAL_EVENT:
		input_report_key(chip->input_dev, MOTOR_EVENT_DOWN_ABNORMAL, 1);
		input_sync(chip->input_dev);
		input_report_key(chip->input_dev, MOTOR_EVENT_DOWN_ABNORMAL, 0);
		input_sync(chip->input_dev);
		break;
	case DOWN_NORMAL_EVENT:
		input_report_key(chip->input_dev, MOTOR_EVENT_DOWN_NORMAL, 1);
		input_sync(chip->input_dev);
		input_report_key(chip->input_dev, MOTOR_EVENT_DOWN_NORMAL, 0);
		input_sync(chip->input_dev);
		break;
	default:
		MOTOR_ERR("state_event (%d) is invalid, would not report \n", state_event);
		break;
	}

	return;
}


//note:work in irq context
int oneplus_dhall_irq_handler(unsigned int id)
{
	MOTOR_LOG("call, id : %d \n", id);
	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip == NULL \n");
		return -EINVAL;
	}

	if (id == HALL_UP) {//push camera
		g_the_chip->hall_up_irq_count ++;
		queue_delayed_work(g_the_chip->manual2auto_wq, &g_the_chip->down_work, 0);
	} else if (id == HALL_DOWN){
		g_the_chip->hall_down_irq_count ++;
		queue_delayed_work(g_the_chip->manual2auto_wq, &g_the_chip->up_work, 0);
	}

	return 0;
}

static void oneplus_motor_irq_monitor(struct oneplus_motor_chip* chip)
{
	if (!chip->irq_monitor_started) {
		MOTOR_LOG("start \n");
		chip->irq_monitor_started = true;
		alarm_start_relative(&chip->reset_timer,
					ktime_set(MOTOR_IRQ_MONITOR_TIME / 1000, (MOTOR_IRQ_MONITOR_TIME % 1000)*  1000000));
	}
}

static irqreturn_t oneplus_free_fall_detect_handler(int irq, void* dev_id)
{
	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip ==NULL \n");
		return -EINVAL;
	}

	g_the_chip->free_fall_irq_times++;


	if (g_the_chip->free_fall_irq_times == 1)
		mod_delayed_work(system_highpri_wq, &g_the_chip->free_fall_irq_check_work, msecs_to_jiffies(1));

	// g_the_chip->is_free_fall = true;
	// disable_irq_nosync(g_the_chip->free_fall_irq);
	// oneplus_motor_downward();
	// enable_irq(g_the_chip->free_fall_irq);

	return IRQ_HANDLED;
}

/*********************************************************************
            node operation by MotorManagerService and user
**********************************************************************/
static ssize_t motor_direction_store(struct device* pdev, struct device_attribute* attr,
			   const char* buf, size_t count)
{
	unsigned long direction = 0;
	int err = 0;

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	err = sscanf(buf, "%lu", &direction);
	MOTOR_LOG("direction : %d, motor_started : %d", direction,g_the_chip->motor_started);
	if (g_the_chip->motor_started) {
		MOTOR_ERR("g_the_chip->motor_started != 0\n");
		return count;
	}

	if (err == 1) {
		oneplus_set_motor_direction(direction);
	}

	return count;;
}

static ssize_t motor_direction_show(struct device* dev,
				  struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->motor_direction);
}

static ssize_t motor_enable_store(struct device* pdev, struct device_attribute* attr,
					const char* buff, size_t count)
{
	unsigned long enable = 0;

	if (sscanf(buff, "%lu", &enable) == 1) {
		MOTOR_ERR("motor_enable_store enable : %d\n", enable);
		if (enable) {
			MOTOR_ERR("oneplus_motor_start \n");
			oneplus_motor_start();
		} else {
			MOTOR_ERR("oneplus_motor_stop \n");
			oneplus_motor_stop();
		}
	}

	return count;
}

static ssize_t step_count_store(struct device *pdev, struct device_attribute *attr,
		                        const char *buff, size_t count)
{
	unsigned long step_count = 0;

	MOTOR_LOG("call");
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}


	if (sscanf(buff, "%lu", &step_count) == 1) {
		g_the_chip->camera_up_step_count = step_count * 32;

		MOTOR_LOG("would set step_count, step_count : %d, g_the_chip->step_count : %d \n", 
							             step_count, g_the_chip->camera_up_step_count);
	} else {
		MOTOR_LOG("would not set step_count, step_count : %d", step_count);
	}

	return count;
}

static ssize_t step_count_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int  step_count = 0;

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}
	
	step_count = g_the_chip->camera_up_step_count / 32;
	MOTOR_LOG("step_count : %d, camera_up_step_count / 32 : %d \n", step_count , g_the_chip->camera_up_step_count / 32 );
	return sprintf(buf, "%d\n",step_count);
}

static ssize_t save_hall_data_store(struct device *pdev, struct device_attribute *attr,
		                            const char *buff, size_t count)
{
	unsigned long save_hall_data_to_file = 0;

	MOTOR_LOG("call");
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}


	if (sscanf(buff, "%lu", &save_hall_data_to_file) == 1) {
		g_the_chip->save_hall_data_to_file = save_hall_data_to_file > 0 ? true : false;

		MOTOR_LOG("would set save_hall_data_to_file, save_hall_data_to_file : %d, g_the_chip->save_hall_data_to_file : %d \n", 
							 save_hall_data_to_file, g_the_chip->save_hall_data_to_file);
	} else {
		MOTOR_LOG("would not set step_count, save_hall_data_to_file : %d", save_hall_data_to_file);
	}

	return count;
}

static ssize_t save_hall_data_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->save_hall_data_to_file);
}

static ssize_t hall_data_show(struct device* dev,struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	oneplus_hall_get_abs_data(HALL_DOWN);
	oneplus_hall_get_abs_data(HALL_UP);

	MOTOR_LOG("hall_down_data  %d hall_up_data %d \n", g_the_chip->hall_down_data, g_the_chip->hall_up_data);

	return sprintf(buf, "%d,%d\n", g_the_chip->hall_down_data, g_the_chip->hall_up_data);
}

static ssize_t motor_move_state_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", -1);
	}

	return sprintf(buf, "%d\n", g_the_chip->move_state);
}

static ssize_t hall_calibration_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	int step_count = 0;

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n%d\n",-1,-1);
	}

	step_count = g_the_chip->camera_up_step_count / 32;
	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
	                    g_the_chip->hall_down_irq_position, 
						g_the_chip->hall_up_irq_position,
	                    g_the_chip->camera_down_slow_down_position_hall_down_data, 
						g_the_chip->camera_down_slow_down_position_hall_up_data,
	                    g_the_chip->camera_up_slow_down_position_hall_down_data, 
						g_the_chip->camera_up_slow_down_position_hall_up_data,
						g_the_chip->bottom_position_hall_down_data,
						g_the_chip->bottom_position_hall_up_data,
						g_the_chip->peak_position_hall_down_data,
						g_the_chip->peak_position_hall_up_data,
						step_count,
						g_the_chip->hall_sensitive);
}

static ssize_t hall_calibration_store(struct device* pdev, struct device_attribute* attr,
						              const char* buff, size_t count)
{
	int data[12] = {0};

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	if (sscanf(buff, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
	                 &data[0], &data[1], &data[2], &data[3], &data[4], &data[5],
					 &data[6], &data[7], &data[8], &data[9], &data[10], &data[11]) == 12) {

		if (data[0] >= 0)
			g_the_chip->hall_down_irq_position = data[0];
		if (data[1] >= 0)
			g_the_chip->hall_up_irq_position = data[1];
		if (data[2] >= 0)
		    g_the_chip->camera_down_slow_down_position_hall_down_data = data[2];
		if (data[3] >= 0)
		    g_the_chip->camera_down_slow_down_position_hall_up_data = data[3];
		if (data[4] >= 0)
		    g_the_chip->camera_up_slow_down_position_hall_down_data = data[4];
		if (data[5] >= 0)
		    g_the_chip->camera_up_slow_down_position_hall_up_data = data[5];
		if (data[6] >= 0)
			g_the_chip->bottom_position_hall_down_data = data[6];
		if (data[7] >= 0)
			g_the_chip->bottom_position_hall_up_data = data[7];
		if (data[8] >= 0)
			g_the_chip->peak_position_hall_down_data = data[8];
		if (data[9] >= 0)
			g_the_chip->peak_position_hall_up_data = data[9];
		if (data[10] >= 0)
			g_the_chip->camera_up_step_count = data[10] * 32;
		if (data[11] == 1 || data[11] == 2)
			g_the_chip->hall_sensitive = data[11];

		MOTOR_LOG("hall calibrate date : %d %d %d %d %d %d %d %d %d %d %d %d\n",
		                data[0], data[1], data[2], data[3], data[4], data[5],
						data[6], data[7], data[8], data[9], data[10], data[11]);

		if (g_the_chip->bottom_position_hall_down_data < 330)
			g_the_chip->deltad_range = 5;
		else 
			g_the_chip->deltad_range = 10;

		MOTOR_LOG("deltad_range : %d \n", g_the_chip->deltad_range);

		oneplus_dhall_set_sensitive(HALL_DOWN, g_the_chip->hall_sensitive);
		oneplus_dhall_set_sensitive(HALL_UP, g_the_chip->hall_sensitive);

		//oneplus_hall_update_threshold(HALL_DOWN, BOTTOM_STATE, HALL_DETECT_RANGE_LOW, g_the_chip->hall_down_irq_position);
	} else {
		MOTOR_ERR("fail\n");
	}
	return count;

}

static ssize_t stall_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n",0);;
	}

	MOTOR_LOG("is_stall %d\n",g_the_chip->is_stall);

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->is_stall);
}

static ssize_t stall_steps_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n",0);;
	}

	MOTOR_LOG("stall_steps %d\n",g_the_chip->stall_steps);

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->stall_steps);
}

static ssize_t stall_mode_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n",0);;
	}

	MOTOR_LOG("stall_mode %d\n",g_the_chip->stall_mode);

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->stall_mode);
}

static ssize_t  motor_test_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n",0);;
	}

	MOTOR_LOG("is_motor_test %d\n",g_the_chip->is_motor_test);

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->is_motor_test);
}

static ssize_t motor_test_store(struct device* pdev, struct device_attribute* attr, const char* buff, size_t count)
{
	int test = 0;

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	if (sscanf(buff, "%d", &test) == 1) {
		g_the_chip->is_motor_test = test;
	}
	MOTOR_LOG("test %d ,is_motor_test %d\n", test, g_the_chip->is_motor_test);

	return count;
}

static ssize_t hall_irq_count_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n%d\n",-1,-1);
	}

	return sprintf(buf, "%d,%d\n",g_the_chip->hall_down_irq_count, g_the_chip->hall_up_irq_count);
}

static ssize_t motor_mode_store(struct device* pdev, struct device_attribute* attr,
			   const char* buff, size_t count)
{
	int mdmode = 0;

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	if (g_the_chip->motor_started)
		return count;

	if (sscanf(buff, "%d", &mdmode) == 1) {
		MOTOR_LOG("mdmode = %d \n", mdmode);
		oneplus_set_motor_work_mode_para(mdmode);
	}

	return count;
}

static ssize_t  motor_mode_show(struct device* dev,
				  struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->motor_work_mode);
}

static ssize_t motor_manual2auto_switch_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	int manual2auto_switch = 0;

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n",0);
	}

	if (g_the_chip->manual2auto_down_switch == 1)
		manual2auto_switch = 1;

	return sprintf(buf, "%d\n",manual2auto_switch);
}

static ssize_t motor_manual2auto_switch_store(struct device* pdev, struct device_attribute* attr,
						const char* buff, size_t count)
{
	int data[1] = {0};

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL\n");
		return count;
	}

	if (sscanf(buff, "%d", &data[0]) == 1) {
		g_the_chip->manual2auto_down_switch = data[0];
		MOTOR_LOG("data : %d \n", data[0]);
		if (data[0] == 1) {
		    if (g_the_chip->position == PEAK_STATE) {
		        oneplus_hall_update_threshold(HALL_UP,MID_STATE, HALL_DETECT_RANGE_LOW, g_the_chip->hall_up_irq_position);
		    } else if (g_the_chip->position == BOTTOM_STATE) {
		        oneplus_hall_update_threshold(HALL_DOWN,MID_STATE, HALL_DETECT_RANGE_LOW, g_the_chip->hall_down_irq_position);
		    }
			g_the_chip->is_skip_pos_check = 0;
		} else {
			g_the_chip->is_skip_pos_check = 1;
		}
	} else {
		MOTOR_ERR("failed \n");
	}

	return count;
}

static ssize_t motor_sw_switch_store(struct device* pdev, struct device_attribute* attr,
						             const char* buff, size_t count)
{
	unsigned long sw_switch = 0;

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	if (sscanf(buff, "%lu", &sw_switch) == 1) {
		g_the_chip->motor_switch = sw_switch;
	}

	return count;
}

static ssize_t  motor_sw_switch_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->motor_switch);
}

static ssize_t motor_force_move_store(struct device* pdev, struct device_attribute* attr,
					                  const char* buff, size_t count)
{
	unsigned long enable = 0;

	MOTOR_LOG("call, enable : %d");

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	if (sscanf(buff, "%lu", &enable) == 1) {
		if (enable) {
			g_the_chip->force_move = true;
			oneplus_motor_start();
		} else {
			oneplus_motor_stop();
		}
	}

	return count;
}

static ssize_t hall_detect_switch_store(struct device* pdev, struct device_attribute* attr,
						                const char* buff, size_t count)
{
	unsigned long sw_switch = 0;

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	if (sscanf(buff, "%lu", &sw_switch) == 1) {
		if (sw_switch)
			g_the_chip->hall_detect_switch = true;
		else
			g_the_chip->hall_detect_switch = false;
	}

	return count;
}

static ssize_t  hall_detect_switch_show(struct device* dev,
				  struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->hall_detect_switch);
}

static ssize_t  motor_all_config_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	int config[6] = {0};

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	oneplus_motor_get_all_config(config, 6);

	return snprintf(buf, PAGE_SIZE, "config {sleep : %d step : %d m0 : %d m1 : %d vref : %d dir : %d}\n",
			                         config[0], config[1], config[2],config[3], config[4], config[5]);
}

static ssize_t  motor_position_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	int config[6] = {0};

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n",0);;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->position);
}

static ssize_t hall_all_reg_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	u8 _buf[1024] = {0};

	oneplus_dhall_dump_regs(HALL_DOWN, _buf);

	return sprintf(buf, "%s\n", _buf);
}

static ssize_t motor_change_speed_store(struct device* pdev, struct device_attribute* attr,
										const char* buff, size_t count)
{
	unsigned long speed = 0;

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	if (sscanf(buff, "%lu", &speed) == 1) {
		oneplus_change_motor_speed(speed);
	}

	return count;
}

static ssize_t motor_speed_store(struct device* pdev, struct device_attribute* attr,
			                     const char* buff, size_t count)
{

	unsigned long speed = 0;

	MOTOR_LOG("call");

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	if (g_the_chip->motor_started)
		return count;

	if (sscanf(buff, "%lu", &speed) == 1) {
		if (speed >=0 && speed <=14) {
			g_the_chip->is_speed_set = true;
			g_the_chip->test_speed = speed;
			MOTOR_LOG("would set speed, test_speed : %d", g_the_chip->test_speed);
		} else {
			MOTOR_LOG("speed (%d) parameter is invalid, would not set speed", speed);
		}
	}

	return count;
}

static ssize_t motor_speed_show(struct device* dev,struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->motor_speed);
}

//slow down speed
static ssize_t motor_slow_down_speed_store(struct device* pdev, struct device_attribute* attr,
			                               const char* buff, size_t count)
{

	unsigned long slow_down_speed = 0;

	MOTOR_LOG("call");

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	if (sscanf(buff, "%lu", &slow_down_speed) == 1) {
		if (slow_down_speed >=0 && slow_down_speed <=14) {
			g_the_chip->slow_down_speed = slow_down_speed;
			MOTOR_LOG("would set slow_down_speed, slow_down_speed : %d", g_the_chip->slow_down_speed);
		} else {
			MOTOR_LOG("slow_down_speed (%d) parameter is invalid, would not set slow_down_speed", slow_down_speed);
		}
	}

	return count;
}

static ssize_t motor_slow_down_speed_show(struct device* dev,struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->slow_down_speed);
}

//deltad range
static ssize_t deltad_range_store(struct device* pdev, struct device_attribute* attr,
			                               const char* buff, size_t count)
{

	unsigned long deltad_range = 0;

	MOTOR_LOG("call");

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	if (sscanf(buff, "%lu", &deltad_range) == 1) {
		g_the_chip->deltad_range = deltad_range;
		MOTOR_LOG("would set deltad_range, deltad_range : %d", g_the_chip->deltad_range);
	} 

	return count;
}

static ssize_t deltad_range_show(struct device* dev,struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->deltad_range);
}

static ssize_t begin_stop_detect_percent_store(struct device* pdev, struct device_attribute* attr,
			                               const char* buff, size_t count)
{

	unsigned long begin_stop_detect_percent = 0;

	MOTOR_LOG("call");

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	if (sscanf(buff, "%lu", &begin_stop_detect_percent) == 1) {
		g_the_chip->begin_stop_detect_percent = begin_stop_detect_percent;
		MOTOR_LOG("would set begin_stop_detect_percent, begin_stop_detect_percent : %d",
				 g_the_chip->begin_stop_detect_percent);
	} 
	MOTOR_LOG("begin_stop_detect_percent : %d", begin_stop_detect_percent);

	return count;
}

static ssize_t begin_stop_detect_percent_show(struct device* dev,struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->begin_stop_detect_percent);
}

static ssize_t factory_mode_store(struct device* pdev, struct device_attribute* attr,
			                               const char* buff, size_t count)
{

	unsigned long is_factory_mode = 0;

	MOTOR_LOG("call");

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	if (sscanf(buff, "%lu", &is_factory_mode) == 1) {
		g_the_chip->is_factory_mode = is_factory_mode;
		MOTOR_LOG("would set is_factory_mode, is_factory_mode : %d", g_the_chip->is_factory_mode);
	} 
	MOTOR_LOG("is_factory_mode : %d", is_factory_mode);

	return count;
}

static ssize_t factory_mode_show(struct device* dev,struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->is_factory_mode);
}

static ssize_t free_fall_irq_times_store(struct device* pdev, struct device_attribute* attr,
			                               const char* buff, size_t count)
{

	unsigned long free_fall_irq_times = 0;

	MOTOR_LOG("call");

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	if (sscanf(buff, "%lu", &free_fall_irq_times) == 1) {
		g_the_chip->free_fall_irq_times = free_fall_irq_times;
		MOTOR_LOG("would set free_fall_irq_times, free_fall_irq_times : %d", g_the_chip->free_fall_irq_times);
	} 
	MOTOR_LOG("free_fall_irq_times : %d", free_fall_irq_times);

	return count;
}

static ssize_t free_fall_irq_times_show(struct device* dev,struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->free_fall_irq_times);
}

static ssize_t infrared_shut_down_state_store(struct device* pdev, struct device_attribute* attr,
			                               const char* buff, size_t count)
{

	unsigned long infrared_shut_down_state = 0;

	MOTOR_LOG("call");

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	if (sscanf(buff, "%lu", &infrared_shut_down_state) == 1) {
		g_the_chip->infrared_shut_down_state = infrared_shut_down_state;
		MOTOR_LOG("would set infrared_shut_down_state, free_fall_irq_times : %d", g_the_chip->infrared_shut_down_state);
	} 
	MOTOR_LOG("infrared_shut_down_state : %d", infrared_shut_down_state);

	return count;
}

static ssize_t infrared_shut_down_state_show(struct device* dev,struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->infrared_shut_down_state);
}

static ssize_t hall_sensitive_store(struct device* pdev, struct device_attribute* attr,
			                               const char* buff, size_t count)
{

	unsigned long hall_sensitive = 0;

	MOTOR_LOG("call");

	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return count;
	}

	if (sscanf(buff, "%lu", &hall_sensitive) == 1) {
		g_the_chip->hall_sensitive = hall_sensitive;

		MOTOR_LOG("would set hall_sensitive, hall_sensitive : %d", g_the_chip->hall_sensitive);

		oneplus_dhall_set_sensitive(HALL_DOWN, g_the_chip->hall_sensitive);
		oneplus_dhall_set_sensitive(HALL_UP, g_the_chip->hall_sensitive);
	}

	MOTOR_LOG("hall_sensitive : %d", hall_sensitive);

	return count;
}

static ssize_t hall_sensitive_show(struct device* dev,struct device_attribute* attr, char* buf)
{
	if (g_the_chip == NULL) {
		MOTOR_ERR("g_the_chip == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->hall_sensitive);
}

static DEVICE_ATTR(direction, S_IRUGO | S_IWUSR, motor_direction_show, motor_direction_store);
static DEVICE_ATTR(speed, S_IRUGO | S_IWUSR, motor_speed_show, motor_speed_store);
static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, motor_mode_show, motor_mode_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, NULL, motor_enable_store);
static DEVICE_ATTR(step_count,   S_IRUGO | S_IWUSR, step_count_show, step_count_store);
static DEVICE_ATTR(save_hall_data, S_IRUGO | S_IWUSR, save_hall_data_show, save_hall_data_store);
static DEVICE_ATTR(change_speed,   S_IRUGO | S_IWUSR,NULL, motor_change_speed_store);
static DEVICE_ATTR(move_state,    S_IRUGO | S_IWUSR, motor_move_state_show,NULL);
static DEVICE_ATTR(config,   S_IRUGO | S_IWUSR, motor_all_config_show,NULL);
static DEVICE_ATTR(sw_switch,    S_IRUGO | S_IWUSR, motor_sw_switch_show,motor_sw_switch_store);
//static DEVICE_ATTR(speed_change_switch,    S_IRUGO | S_IWUSR, motor_speed_change_switch_show,motor_speed_change_switch_store);
static DEVICE_ATTR(position,   S_IRUGO | S_IWUSR, motor_position_show,NULL);
static DEVICE_ATTR(manual2auto_switch,   S_IRUGO | S_IWUSR, motor_manual2auto_switch_show,motor_manual2auto_switch_store);
static DEVICE_ATTR(motor_test,   S_IRUGO | S_IWUSR, motor_test_show,motor_test_store);
static DEVICE_ATTR(hall_data,   S_IRUGO | S_IWUSR,hall_data_show,NULL);
static DEVICE_ATTR(hall_reg,   S_IRUGO | S_IWUSR,hall_all_reg_show,NULL);
static DEVICE_ATTR(hall_irq_count,   S_IRUGO | S_IWUSR,hall_irq_count_show,NULL);
static DEVICE_ATTR(hall_calibration,   S_IRUGO | S_IWUSR,hall_calibration_show,hall_calibration_store);
static DEVICE_ATTR(stall,   S_IRUGO | S_IWUSR, stall_show, NULL);
static DEVICE_ATTR(stall_steps,   S_IRUGO | S_IWUSR, stall_steps_show, NULL);
static DEVICE_ATTR(stall_mode, S_IRUGO | S_IWUSR, stall_mode_show, NULL);
static DEVICE_ATTR(hall_detect_switch,   S_IRUGO | S_IWUSR,hall_detect_switch_show,hall_detect_switch_store);
static DEVICE_ATTR(force_move, 	S_IRUGO | S_IWUSR, NULL, motor_force_move_store);
//static DEVICE_ATTR(stop_time, S_IRUGO | S_IWUSR, motor_stop_time_show, motor_stop_time_store);
//static DEVICE_ATTR(initial_time, S_IRUGO | S_IWUSR, motor_initial_time_show, motor_initial_time_store);
//static DEVICE_ATTR(initial_speed, S_IRUGO | S_IWUSR, motor_initial_speed_show, motor_initial_speed_time_store);
//static DEVICE_ATTR(high_speed, S_IRUGO | S_IWUSR, motor_high_speed_show, motor_high_speed_store);
//static DEVICE_ATTR(high_speed_time, S_IRUGO | S_IWUSR, motor_high_speed_time_show, motor_high_speed_time_store);
static DEVICE_ATTR(slow_down_speed, S_IRUGO | S_IWUSR, motor_slow_down_speed_show, motor_slow_down_speed_store);
static DEVICE_ATTR(deltad_range, S_IRUGO | S_IWUSR, deltad_range_show, deltad_range_store);
static DEVICE_ATTR(begin_stop_detect_percent, S_IRUGO | S_IWUSR, begin_stop_detect_percent_show, begin_stop_detect_percent_store);
static DEVICE_ATTR(factory_mode, S_IRUGO | S_IWUSR, factory_mode_show, factory_mode_store);
static DEVICE_ATTR(free_fall_irq_times, S_IRUGO | S_IWUSR, free_fall_irq_times_show, free_fall_irq_times_store);
static DEVICE_ATTR(infrared_shut_down_state, S_IRUGO | S_IWUSR, infrared_shut_down_state_show, infrared_shut_down_state_store);
static DEVICE_ATTR(hall_sensitive, S_IRUGO | S_IWUSR, hall_sensitive_show, hall_sensitive_store);


static struct attribute*  __attributes[] = {
	&dev_attr_direction.attr,
	&dev_attr_speed.attr,
	&dev_attr_mode.attr,
	&dev_attr_enable.attr,
	&dev_attr_step_count.attr,
	&dev_attr_save_hall_data.attr,
	&dev_attr_change_speed.attr,
	&dev_attr_move_state.attr,
	&dev_attr_config.attr,
	&dev_attr_sw_switch.attr,
	&dev_attr_position.attr,
	&dev_attr_manual2auto_switch.attr,
	&dev_attr_motor_test.attr,
	&dev_attr_hall_data.attr,
	&dev_attr_hall_reg.attr,
	&dev_attr_hall_irq_count.attr,
	&dev_attr_hall_calibration.attr,
	&dev_attr_stall.attr,
	&dev_attr_stall_steps.attr,
	&dev_attr_stall_mode.attr,
	&dev_attr_hall_detect_switch.attr,
	&dev_attr_force_move.attr,

	// &dev_attr_stop_time.attr,
	// &dev_attr_initial_time.attr,
	// &dev_attr_initial_speed.attr,
	// &dev_attr_high_speed.attr,
	// &dev_attr_high_speed_time.attr,
	&dev_attr_slow_down_speed.attr,
	&dev_attr_deltad_range.attr,
	&dev_attr_begin_stop_detect_percent.attr,
	&dev_attr_factory_mode.attr,
	&dev_attr_free_fall_irq_times.attr,
	&dev_attr_infrared_shut_down_state.attr,
	&dev_attr_hall_sensitive.attr,
	NULL
};

static struct attribute_group __attribute_group = {
	.attrs = __attributes
};

/*********************************************************************
            digital_hall and step_motor register ops function
**********************************************************************/
int oneplus_register_dhall(const char* name, struct oneplus_hall_operations* ops)
{
	if(name == NULL || ops == NULL) {
		MOTOR_ERR("name == NULL || ops == NULL, would not register digital hall \n");
		return -EINVAL;
	}

	if (g_the_chip == NULL) {
		struct oneplus_motor_chip* chip = kzalloc(sizeof(struct oneplus_motor_chip), GFP_KERNEL);
		if (!chip) {
			MOTOR_ERR("kzalloc err \n");
			return -ENOMEM;
		}
		oneplus_motor_chip_init(chip);
		g_the_chip = chip;
		
	}

	MOTOR_LOG("name : %s\n", name);

	if (strcmp(name, "m1120_down") == 0) {
		if (g_the_chip->hall_down_ops == NULL) {
			g_the_chip->hall_down_ops = ops;
		} else {
			MOTOR_ERR("hall_down_ops has been registered \n");
			return -EINVAL;
		}
	}

	if (strcmp(name, "m1120_up") == 0) {
		if (g_the_chip->hall_up_ops == NULL) {
			g_the_chip->hall_up_ops = ops;
		} else {
			MOTOR_ERR("hall_up_ops has been registered \n");
			return -EINVAL;
		}
	}

	return 0;
}

int oneplus_register_motor(const char*  name, struct oneplus_motor_operations* ops)
{
	if(name == NULL || ops == NULL) {
		MOTOR_ERR("name == NULL || ops == NULL, would not register step motor \n");
		return -EINVAL;
	}

	MOTOR_LOG("motor name : %s \n", name);
	if (g_the_chip == NULL) {
		struct oneplus_motor_chip* chip = kzalloc(sizeof(struct oneplus_motor_chip), GFP_KERNEL);
		if (!chip) {
			MOTOR_ERR("kzalloc err \n");
			return -ENOMEM;
		}
		oneplus_motor_chip_init(chip);
		g_the_chip = chip;
	}

	if (g_the_chip->motor_ops == NULL) {
		g_the_chip->motor_ops = ops;
	} else {
		MOTOR_ERR("motor_ops has been registered \n");
		return -EINVAL;
	}

	return 0;
}

/*********************************************************************
						 init function
**********************************************************************/
static int oneplus_input_dev_init(struct oneplus_motor_chip* chip)
{
	struct input_dev* dev;
	int err;

	dev = input_allocate_device();
	if (dev == NULL) {
		MOTOR_ERR("input_allocate_device failed, dev == NULL \n");
		return -ENOMEM;
	}

	dev->name = "motor";
	dev->id.bustype = BUS_I2C;

	set_bit(MOTOR_EVENT_TYPE, dev->evbit);
	set_bit(MOTOR_EVENT_MANUAL_TO_DOWN, dev->keybit);
	set_bit(MOTOR_EVENT_UP, dev->keybit);
	set_bit(MOTOR_EVENT_UP_ABNORMAL, dev->keybit);
	set_bit(MOTOR_EVENT_UP_NORMAL, dev->keybit);
	set_bit(MOTOR_EVENT_DOWN, dev->keybit);
	set_bit(MOTOR_EVENT_DOWN_ABNORMAL, dev->keybit);
	set_bit(MOTOR_EVENT_DOWN_NORMAL, dev->keybit);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
 
		MOTOR_ERR("input_register_device failed, err : %d \n", err);
		return err;
	}

	chip->input_dev = dev;

	return 0;
}

static int oneplus_motor_chip_init(struct oneplus_motor_chip* chip)
{
	MOTOR_LOG("call \n");

	if (chip == NULL) {
		MOTOR_ERR("chip == NULL \n");
		return -EINVAL;
	}

	//step motor  property 
	chip->pwm_duty = 0;
	chip->pwm_period = 0;
	chip->motor_work_mode = MOTOR_MODE_1_32;
	chip->motor_speed = MOTOR_SPEED0;
	chip->motor_direction = MOTOR_UPWARD;
	chip->motor_enable = false;
	chip->pwm_count = 0;
	chip->motor_started = false;
	chip->motor_switch = true;

	//digital hall property
	chip->hall_down_data = 0;
	chip->hall_up_data = 0;
	chip->hall_down_irq_count = 0;
	chip->hall_up_irq_count = 0;
	chip->hall_sensitive = HALL_10BIT_0_034mT;

	//calibrate property
	chip->hall_down_irq_position = HALL_DETECT_RANGE_HIGH;
	chip->hall_up_irq_position = HALL_DETECT_RANGE_HIGH;
	chip->camera_up_slow_down_position_hall_down_data = 0;
	chip->camera_up_slow_down_position_hall_up_data =  MOTOR_STOP_RETARD_VALUE;
	chip->camera_down_slow_down_position_hall_down_data =  MOTOR_STOP_RETARD_VALUE;
	chip->camera_down_slow_down_position_hall_up_data = 0;
	chip->is_stall = 0;
	chip->stall_steps = 0;
	chip->stall_mode = UNKNOW_MODE;

	//special test property
	chip->is_motor_test = false;
	chip->force_move = false;
	chip->is_speed_set = false;
	chip->is_factory_mode = 0;
	chip->test_speed = MOTOR_SPEED0;

	//logical control property
	atomic_set(&chip->in_suspend, 0);
	chip->whole_jonery_time = 500000;//500ms default

	chip->whole_jonery_length = 82;//8.2mm, unuse
	chip->begin_stop_detect_percent = 95;//95% default
	chip->speed_up_distance = 5;//0.5mm
	chip->speed_down_distance = 3;//0.3mm
	chip->speed_up_pwm_count = 30 * 32;//75 step default
	chip->camera_up_step_count = 1380 * 32;//1380 step default
	chip->slow_down_speed = MOTOR_SPEED0;
	chip->deltad_range = 5;//5 default
	//chip->speed_down_pwm_count = 0;
	chip->position_detect_delay = 15;//15ms
	chip->camera_position_detect = false;
	//chip->motor_start_time = 0;
	chip->save_hall_data_to_file = false;
	chip->hall_detect_switch = true;
	chip->position = BOTTOM_STATE;
	chip->move_state = MOTOR_STOP;
	chip->is_skip_pos_check = false;
	chip->manual2auto_down_switch = true;
	chip->is_free_fall = false;
	chip->free_fall_irq_times = 0;
	chip->infrared_shut_down_state = 1;
	chip->free_fall_gpio = 0;
	chip->free_fall_irq = 0;
	chip->irq_monitor_started = false;
	chip->is_irq_abnormal  = false;
	chip->is_t0_structure = false;
	chip->is_mag_positive = true;
	chip->led_on = false;
	
	chip->dev = NULL;
	chip->pctrl = NULL;
	chip->free_fall_state = NULL;
	chip->input_dev = NULL;
	chip->manual2auto_wq = NULL;
	chip->motor_run_work_wq = NULL;
	chip->hall_up_ops = NULL;
	chip->hall_down_ops = NULL;
	chip->motor_ops = NULL;

	return 0;
}

static void oneplus_motor_awake_init(struct oneplus_motor_chip* chip)
{
	if (chip == NULL) {
		MOTOR_ERR("chip == NULL \n");
		return;
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	wake_lock_init(&chip->suspend_lock, WAKE_LOCK_SUSPEND, "motor_wakelock");
#else
	chip->suspend_ws = wakeup_source_register("motor_wakelock");
#endif
}


static void oneplus_motor_free_fall_register(struct oneplus_motor_chip*  chip)
{
	struct device_node* np = NULL;
	int err = 0;

	np = chip->dev->of_node;

	chip->pctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR(chip->pctrl)) {
		MOTOR_ERR("failed to get pinctrl \n");
		return;
	};

	chip->free_fall_state = pinctrl_lookup_state(chip->pctrl, "free_fall_input");
	if (IS_ERR(chip->free_fall_state)) {
		err = PTR_ERR(chip->free_fall_state);
		MOTOR_ERR("pinctrl_lookup_state failed, err : %d \n", err);
		return;
	};

	pinctrl_select_state(chip->pctrl,chip->free_fall_state);
	chip->free_fall_gpio = of_get_named_gpio(np, "motor,irq-gpio", 0);

	if (!gpio_is_valid(chip->free_fall_gpio)) {
		MOTOR_LOG("qcom,hall-power-gpio gpio not specified \n");
	} else {
		err = gpio_request(chip->free_fall_gpio, "motor-irq-gpio");
		if (err)
			MOTOR_LOG("request free_fall_gpio gpio failed, err : %d \n",err);

		err = gpio_direction_input(chip->free_fall_gpio);
		msleep(50);
		chip->free_fall_irq = gpio_to_irq(chip->free_fall_gpio);

		if (request_irq(chip->free_fall_irq, &oneplus_free_fall_detect_handler, IRQ_TYPE_EDGE_RISING, "free_fall", NULL)) {
			MOTOR_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return;
		}
		irq_set_irq_wake(chip->free_fall_irq, 1);
	}

	MOTOR_LOG("gpio %d irq:%d \n",chip->free_fall_gpio, chip->free_fall_irq);
}

static void oneplus_motor_reset_check(struct oneplus_motor_chip* chip)
{
    oneplus_hall_get_abs_data(HALL_DOWN);
	
    MOTOR_LOG("hall0 data %d  hall_down_irq_position %d \n", chip->hall_down_data, chip->hall_down_irq_position);
	
	if (chip->hall_down_data < 0)
		chip->hall_down_data = 0 - chip->hall_down_data;

    if (chip->hall_down_data < chip->hall_down_irq_position) {
        MOTOR_LOG("reset motor \n");
	    chip->position = PEAK_STATE;
        oneplus_motor_downward();
    }

}

static void oneplus_parameter_init(struct oneplus_motor_chip* chip)
{
	u32 id = 0;
	struct device_node* np = NULL;
	int err = -1;

	if (chip == NULL || chip->dev == NULL) {
		MOTOR_ERR("oneplus_parameter_init failed, chip == NULL || chip->dev == NULL");
		return;
	}

	np = chip->dev->of_node;
	err = of_property_read_u32(np, "structure,id", &id);
	if (err && (err != -EINVAL)) {
		MOTOR_ERR("get structure,id err : %d", err);
	} else {
		MOTOR_LOG("read structure,id success, id : %d", id);
		if (id == 1) {
			MOTOR_LOG("is T0 structure");
			chip->is_t0_structure = true;
		}
	}

	oneplus_hall_update_threshold(HALL_DOWN, BOTTOM_STATE, 511, 511);
	oneplus_hall_update_threshold(HALL_UP, BOTTOM_STATE, 511, 511);
	oneplus_dhall_set_detection_mode(HALL_DOWN, DETECTION_MODE_INTERRUPT);
	oneplus_dhall_set_detection_mode(HALL_UP, DETECTION_MODE_INTERRUPT);

	oneplus_set_motor_work_mode_para(MOTOR_MODE_1_32);

	oneplus_set_motor_direction(MOTOR_UPWARD);

	oneplus_motor_awake_init(chip);


	oneplus_hall_get_real_data(HALL_DOWN);
	oneplus_hall_get_real_data(HALL_UP);
	//save hall data
	MOTOR_LOG("hall_down_data : %d, HALL_UP : %d", chip->hall_down_data, chip->hall_up_data);
	if (chip->hall_down_data < 0) {
		MOTOR_LOG("mag is negative");
		chip->is_mag_positive = false;
	}

}

static int motor_platform_probe(struct platform_device* pdev)
{
	struct oneplus_motor_chip* chip = NULL;
	struct proc_dir_entry* dir = NULL;
	int err = 0;


	MOTOR_LOG("call \n");

	if (g_the_chip == NULL) {
		chip = kzalloc(sizeof(struct oneplus_motor_chip), GFP_KERNEL);
		if (!chip) {
			MOTOR_ERR("kzalloc err \n");
			return -ENOMEM;
		}
		g_the_chip = chip;
	} else {
		chip = g_the_chip;
	}

	chip->dev = &pdev->dev;

	if (chip->hall_up_ops == NULL || chip->hall_down_ops == NULL) {
		MOTOR_ERR("no digital hall available \n");
		//goto fail;
	}

	if (chip->motor_ops == NULL) {
		MOTOR_ERR("no motor driver available \n");
		//goto fail;
	}
	//create
	err = sysfs_create_group(&pdev->dev.kobj, &__attribute_group);
	if(err) {
		MOTOR_ERR("sysfs_create_group failed, err : %d \n", err);
		goto sysfs_create_fail;
	}

	err = oneplus_input_dev_init(chip);
	if (err < 0) {
		MOTOR_ERR("oneplus_input_dev_init failed, err : %d \n", err);
		//goto input_fail;
	}

	chip->motor_run_work_wq = create_singlethread_workqueue("motor_run_work_wq");
	if (chip->motor_run_work_wq == NULL) {
  		MOTOR_ERR("create_singlethread_workqueue failed, motor_run_work_wq == NULL \n");
		//goto input_fail;
	}

	chip->manual2auto_wq = create_singlethread_workqueue("manual2auto_wq");
	if (!chip->manual2auto_wq) {
	    MOTOR_ERR("create_singlethread_workqueue failed, manual2auto_wq == NULL \n");
		//goto input_fail;
	}

    oneplus_motor_free_fall_register(chip);

	oneplus_parameter_init(chip);

	hrtimer_init(&chip->stop_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->stop_timer.function = motor_stop_timer_func;

	hrtimer_init(&chip->speed_up_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->speed_up_timer.function = motor_speed_up_timer_func;

	//INIT_WORK(&chip->motor_work, motor_run_work);
	//INIT_WORK(&chip->manual_position_work,manual_position_detect_work);//zhe


	alarm_init(&chip->reset_timer, ALARM_BOOTTIME, motor_reset_timer_func); 

	INIT_DELAYED_WORK(&chip->detect_work, camera_position_detect_work);
	INIT_DELAYED_WORK(&chip->up_work, manual_to_auto_up_work);
	INIT_DELAYED_WORK(&chip->down_work, manual_to_auto_down_work);
	INIT_DELAYED_WORK(&chip->free_fall_irq_check_work, free_fall_irq_check_work_func);
	INIT_WORK(&chip->motor_work, motor_run_work);

	chip->fb_notify.notifier_call = fb_notifier_callback;
	#ifdef CONFIG_DRM_MSM
	msm_drm_register_client(&chip->fb_notify);
	#else
	fb_register_client(&chip->fb_notify);
	#endif

	oneplus_motor_reset_check(g_the_chip);

	MOTOR_LOG("success. \n");
	return 0;

//input_fail:
sysfs_create_fail:
//fail:
	kfree(chip);
	g_the_chip = NULL;
	MOTOR_LOG("fail \n");
	return -EINVAL;
}

static int motor_platform_remove(struct platform_device* pdev)
{
	if (g_the_chip) {
		sysfs_remove_group(&pdev->dev.kobj, &__attribute_group);
		input_unregister_device(g_the_chip->input_dev);
        input_free_device(g_the_chip->input_dev);
		kfree(g_the_chip);
		g_the_chip = NULL;
	}
	return 0;
}

static int motor_platform_suspend(struct platform_device* pdev, pm_message_t state)
{
	if (g_the_chip) {
		atomic_set(&g_the_chip->in_suspend, 1);
	}
	return 0;
}

static int motor_platform_resume(struct platform_device* pdev)
{
	if (g_the_chip) {
		atomic_set(&g_the_chip->in_suspend, 0);
	}
	return 0;
}
static void motor_platform_shutdown(struct platform_device* pdev)
{
	MOTOR_LOG("call \n");
	if (g_the_chip) {
		//when phone is power off, check if camera is outside 
		oneplus_motor_reset_check(g_the_chip);
	}
	return;
}


static const struct of_device_id of_motor_match[] = {
	{ .compatible = "oneplus-motor"},
	{},
};
MODULE_DEVICE_TABLE(of, of_motor_match);

static struct platform_driver motor_platform_driver = {
	.probe		= motor_platform_probe,
	.remove		= motor_platform_remove,
	.suspend	= motor_platform_suspend,
	.resume		= motor_platform_resume,
	.shutdown   = motor_platform_shutdown,
	.driver		= {
		.name	= "oneplus_motor",
		.of_match_table = of_motor_match,
	},
};

static int __init motor_platform_init(void)
{
	MOTOR_LOG("call \n");

	platform_driver_register(&motor_platform_driver);
	return 0;
}

late_initcall(motor_platform_init);
MODULE_DESCRIPTION("camera motor platform driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("quentin.lin@oneplus.com");
