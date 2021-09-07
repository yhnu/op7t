/************************************************************************************
** Copyright (C), 2013-2018, Oneplus Mobile Comm Corp., Ltd
** File: oneplus_tri_key.c
**
** Description:
**      Definitions for m1120 tri_state_key data process.
**
** Version: 1.0
**************************************************************************************/

#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/hrtimer.h>
#include <linux/alarmtimer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/extcon.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/time.h>


#include <linux/string.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
#include <linux/wakelock.h>
#endif
#include "oneplus_tri_key.h"
#include "../extcon/extcon.h"

#define TRI_KEY_TAG                  "[tri_state_key] "
#define TRI_KEY_ERR(fmt, args...)    printk(KERN_ERR TRI_KEY_TAG" %s : "fmt, __FUNCTION__, ##args)
#define TRI_KEY_LOG(fmt, args...)    printk(KERN_INFO TRI_KEY_TAG" %s : "fmt, __FUNCTION__, ##args)
#define TRI_KEY_DEBUG(fmt, args...)\
	do{\
		if (LEVEL_DEBUG == tri_key_debug)\
			printk(KERN_INFO TRI_KEY_TAG " %s: " fmt, __FUNCTION__, ##args);\
	}while(0)

enum {
	MODE_UNKNOWN,
	MODE_MUTE,
	MODE_DO_NOT_DISTURB,
	MODE_NORMAL,
	MODE_MAX_NUM
	} tri_mode;


unsigned int tristate_extcon_tab[] = {
		MODE_MUTE,
		MODE_DO_NOT_DISTURB,
		MODE_NORMAL,
	};

static struct hrtimer tri_key_timer;
struct work_struct tri_key_timeout_work;

static struct extcon_dev_data *g_the_chip = NULL;
static int last_d0 = 0;
static int last_d1 = 0;
static int last_position = -1;
static int last_interf = -1;
static int interf_count;
static int time = 1;
unsigned int tri_key_debug = 0;
//static int up_buf[20] = {0};
//static int down_buf[20] = {0};

//static short tol0 = 10;
static short tol1 = 15;
static short tol2 = 22;
static short tol3 = 92;
static short tol4 = -91;
static short calib_UpValueSum = 0, calib_MdValueSum = 0, calib_DnValueSum = 0;
static short calib_UpValueMin = 0, calib_MdValueMin = 0, calib_DnValueMin = 0;
static short calib_dnHall_UM_distance = 0, calib_dnHall_MD_distance = 0;
static short calib_upHall_UM_distance = 0, calib_upHall_MD_distance = 0;
static short calib_upHall_UD_distance = 0, calib_dnHall_UD_distance = 0;




int oneplus_register_hall(const char *name, struct dhall_operations *ops)
{
	if (!name || !ops) {
		TRI_KEY_ERR("name is NULL or ops is NULL, would not register digital hall \n");
		return -EINVAL;
	}

	if (!g_the_chip) {
		struct extcon_dev_data *chip = kzalloc(sizeof(struct extcon_dev_data), GFP_KERNEL);
		if (!chip) {
			TRI_KEY_ERR("kzalloc err \n");
			return -ENOMEM;
		}
		g_the_chip = chip;
	}
	TRI_KEY_LOG("name : %s\n", name);
	if (strcmp(name, "hall_down") == 0) {
		TRI_KEY_LOG("name == hall_down");
		if (!g_the_chip->dhall_down_ops) {
			if (ops) {
				g_the_chip->dhall_down_ops = ops;
				g_the_chip->d_name = name;
			} else {
				TRI_KEY_ERR("dhall_down_ops NULL \n");
				return -EINVAL;
			}
		} else {
			TRI_KEY_ERR("dhall_down_ops has been register \n");
			return -EINVAL;
		}
	}
	if (strcmp(name, "hall_up") == 0) {
		TRI_KEY_LOG("name == hall_up");
		if (!g_the_chip->dhall_up_ops) {
			if (ops) {
				g_the_chip->dhall_up_ops = ops;
				g_the_chip->d_name = name;
			}  else {
				TRI_KEY_ERR("dhall_up_ops NULL \n");
				return -EINVAL;
			}
		} else {
			TRI_KEY_ERR("dhall_up_ops has been register \n");
			return -EINVAL;
		}
	}

	return 0;
}

int oneplus_hall_enable_irq (unsigned int id, bool enable)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops || !g_the_chip->dhall_down_ops->enable_irq) {
			return -EINVAL;
		} else {
			oneplus_hall_clear_irq(DHALL_0);
			oneplus_hall_clear_irq(DHALL_1);
			return g_the_chip->dhall_down_ops->enable_irq(enable);
		}
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops || !g_the_chip->dhall_up_ops->enable_irq) {
			return -EINVAL;
		} else {
			oneplus_hall_clear_irq(DHALL_0);
			oneplus_hall_clear_irq(DHALL_1);
			return g_the_chip->dhall_up_ops->enable_irq(enable);
		}
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}

	return -EINVAL;
}

int oneplus_hall_clear_irq (unsigned int id)
{
	if (!g_the_chip)
		return -EINVAL;

	TRI_KEY_DEBUG("dhall_clear_irq\n");
	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops || !g_the_chip->dhall_down_ops->enable_irq) {
			return -EINVAL;
		} else {
			return g_the_chip->dhall_down_ops->clear_irq();
		}
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops || !g_the_chip->dhall_up_ops->enable_irq) {
			return -EINVAL;
		} else {
			return g_the_chip->dhall_up_ops->clear_irq();
		}
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}

	return -EINVAL;
}

int oneplus_hall_get_data(unsigned int id)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops || !g_the_chip->dhall_down_ops->get_data) {
			return -EINVAL;
		} else {
				return g_the_chip->dhall_down_ops->get_data(&g_the_chip->dhall_data0);
		}
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops || !g_the_chip->dhall_up_ops->get_data) {
			return -EINVAL;
		} else {
				return g_the_chip->dhall_up_ops->get_data(&g_the_chip->dhall_data1);
		}
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}

}

bool oneplus_hall_update_threshold(unsigned int id, int position, short lowthd, short highthd)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops || !g_the_chip->dhall_down_ops->update_threshold) {
			return false;
		} else {
			return g_the_chip->dhall_down_ops->update_threshold(position, lowthd, highthd);
		}
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops || !g_the_chip->dhall_up_ops->update_threshold) {
			return false;
		} else {
			return g_the_chip->dhall_up_ops->update_threshold(position, lowthd, highthd);
		}
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}

}

int oneplus_hall_set_detection_mode(unsigned int id, u8 mode)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops || !g_the_chip->dhall_down_ops->set_detection_mode) {
			return -EINVAL;
		} else {
			return g_the_chip->dhall_down_ops->set_detection_mode(mode);
		}
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops || !g_the_chip->dhall_up_ops->set_detection_mode) {
			return -EINVAL;
		} else {
			return g_the_chip->dhall_up_ops->set_detection_mode(mode);
		}
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}

}

int oneplus_hall_get_irq_state(unsigned int id)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops || !g_the_chip->dhall_down_ops->get_irq_state) {
			return -EINVAL;
		} else {
			return g_the_chip->dhall_down_ops->get_irq_state();
		}
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops || !g_the_chip->dhall_up_ops->get_irq_state) {
			return -EINVAL;
		} else {
			return g_the_chip->dhall_up_ops->get_irq_state();
		}
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}

}



void oneplus_hall_dump_regs(unsigned int id, u8 *buf)
{
	if (!g_the_chip)
		return;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops || !g_the_chip->dhall_down_ops->dump_regs) {
			return;
		} else {
			g_the_chip->dhall_down_ops->dump_regs(buf);
		}
		break;
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops || !g_the_chip->dhall_up_ops->dump_regs) {
			return;
		} else {
			g_the_chip->dhall_up_ops->dump_regs(buf);
		}
		break;
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return;
	}
}

int oneplus_hall_set_reg(unsigned int id, int reg, int val)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops || !g_the_chip->dhall_down_ops->set_reg) {
			return -EINVAL;
		} else {
			return g_the_chip->dhall_down_ops->set_reg(reg, val);
		}
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops || !g_the_chip->dhall_up_ops->set_reg) {
			return -EINVAL;
		} else {
			return g_the_chip->dhall_up_ops->set_reg(reg, val);
		}
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}

}

bool oneplus_hall_is_power_on(void)
{
	if (!g_the_chip || !g_the_chip->dhall_down_ops || !g_the_chip->dhall_down_ops->is_power_on
					|| !g_the_chip->dhall_up_ops || !g_the_chip->dhall_up_ops->is_power_on) {
		return false;
	} else {
		if (g_the_chip->dhall_down_ops->is_power_on() || g_the_chip->dhall_up_ops->is_power_on())
			return true;
		else
			return false;
	}

}
static void reboot_get_position(struct extcon_dev_data *chip)
{
	short delta;
	short up_data1;
	short down_data1;
	if (chip->dhall_data1 < 0 || chip->dhall_data0 < 0) {
		up_data1 = -chip->dhall_data1;
		down_data1 = -chip->dhall_data0;
		delta = up_data1 - down_data1;
	} else
		delta = chip->dhall_data1 - chip->dhall_data0;
	if (delta > 30)
		chip->position = UP_STATE;
	else if (-delta > 30)
		chip->position = DOWN_STATE;
	else
		chip->position = MID_STATE;
	last_position = chip->position;
}

static int interf_get_position(struct extcon_dev_data *chip)
{
	short delta0;
	short delta1;
	delta0 = chip->dhall_data0 - last_d0;
	delta1 = chip->dhall_data1 - last_d1;
	TRI_KEY_LOG("tri_key: delta0 is %d ,delta1 is %d,last_postion is %d\n",
			delta0, delta1, last_position);
	if ((delta1 > calib_upHall_UM_distance - tol1 &&
			delta1 < calib_upHall_UM_distance + tol1) &&
			(delta0 > calib_dnHall_UM_distance - tol1 &&
			delta0 < calib_dnHall_UM_distance + tol1)) {
		if (last_position == MID_STATE)
			return UP_STATE;
			}
	if ((delta1 > calib_upHall_UD_distance - tol1 &&
		delta1 < calib_upHall_UD_distance + tol1) &&
		(delta0 > calib_dnHall_UD_distance - tol1 &&
		delta0 < calib_dnHall_UD_distance + tol1))
		return UP_STATE;
	if ((delta1 > -calib_upHall_MD_distance - tol1 &&
		delta1 < -calib_upHall_MD_distance + tol1) &&
		(delta0 > -calib_dnHall_MD_distance - tol1 &&
		delta0 < -calib_dnHall_MD_distance + tol1)) {
		if (last_position == MID_STATE)
			return DOWN_STATE;
			}
	if ((delta1 > -calib_upHall_UD_distance - tol1 &&
		delta1 < -calib_upHall_UD_distance + tol1) &&
		(delta0 > -calib_dnHall_UD_distance - tol1 &&
		delta0 < -calib_dnHall_UD_distance + tol1))
		return DOWN_STATE;
	if ((delta1 > -calib_upHall_UM_distance - tol1 &&
		delta1 < -calib_upHall_UM_distance + tol1) &&
		(delta0 > -calib_dnHall_UM_distance - tol1 &&
		delta0 < -calib_dnHall_UM_distance + tol1)) {
		if (last_position == UP_STATE)
			return MID_STATE;
			}
	if ((delta1 > calib_upHall_MD_distance - tol1 &&
		delta1 < calib_upHall_MD_distance + tol1) &&
		(delta0 > calib_dnHall_MD_distance - tol1 &&
		delta0 < calib_dnHall_MD_distance + tol1)) {
		if (last_position == DOWN_STATE)
			return MID_STATE;
			}
	return -EINVAL;

}

static int get_position(struct extcon_dev_data *chip)
{
	short diff;
	diff = chip->dhall_data1 - chip->dhall_data0;
	if (chip->dhall_data0 > 0) {
		//if (diff > calib_UpValueMin - tol1 && diff < calib_UpValueMin + tol2)
		if (diff > calib_UpValueMin - tol1 && diff < tol3)
			chip->position = UP_STATE;
		if (calib_MdValueMin < 0) {
			if (diff > calib_MdValueMin - tol1 && diff < calib_MdValueMin + tol1)
				chip->position = MID_STATE;
			}
		if (calib_MdValueMin > 0 || calib_MdValueMin == 0) {
			if (diff > calib_MdValueMin - tol1 && diff < calib_MdValueMin + tol1)
				chip->position = MID_STATE;
			}
		//if (diff > calib_DnValueMin - tol2 && diff < calib_DnValueMin + tol1)
		if (diff > tol4 && diff < calib_DnValueMin + tol1)
			chip->position = DOWN_STATE;
	} else {
		//if (diff > calib_UpValueMin - tol2 && diff < calib_UpValueMin + tol1)
		if (diff > -tol3 && diff < calib_UpValueMin + tol1)
			chip->position = UP_STATE;
		if (calib_MdValueMin < 0) {
			if (diff > calib_MdValueMin - tol1 && diff < calib_MdValueMin + tol1)
				chip->position = MID_STATE;
			}
		if (calib_MdValueMin > 0 || calib_MdValueMin == 0) {
			if (diff > calib_MdValueMin - tol1 && diff < calib_MdValueMin + tol1)
				chip->position = MID_STATE;
			}
		//if (diff > calib_DnValueMin - tol1 && diff < calib_DnValueMin + tol2)
		if (diff > calib_DnValueMin - tol1 && diff < -tol4)
			chip->position = DOWN_STATE;
	}
	return 0;
}

static int judge_interference(struct extcon_dev_data *chip)
{
	short delta;
	short sum;
	delta = chip->dhall_data1 - chip->dhall_data0;
	TRI_KEY_LOG("tri_key:delta is %d\n", delta);
	sum = chip->dhall_data0 + chip->dhall_data1;
	TRI_KEY_LOG("tri_key:sum is %d\n", sum);
	if (chip->dhall_data1 > 0) {//the hall data is positive number
		//if (delta > calib_UpValueMin - tol1 && delta < calib_UpValueMin + tol2) {
		if (delta > calib_UpValueMin - tol1 && delta < tol3) {
			TRI_KEY_LOG("tri_key:calib_Min:%d,calib_Sum:%d\n",
				calib_UpValueMin, calib_UpValueSum);
			if (sum < calib_UpValueSum - tol1 || sum > calib_UpValueSum + tol2) {
				chip->interf = 1;
				chip->state = 1;
			} else {
				chip->interf = 0;
				chip->state = 1;
			}
			return 0;
		}
		if (calib_MdValueMin < 0) {
			if (delta > calib_MdValueMin - tol1 && delta < calib_MdValueMin + tol1) {
				TRI_KEY_LOG("tri_key:calibMin:%d,calib_Sum:%d\n",
					calib_MdValueMin, calib_MdValueSum);

				if (sum > calib_MdValueSum + tol2 || sum < calib_MdValueSum - tol1) {
					chip->interf = 1;
					chip->state = 2;
				} else {
					chip->interf = 0;
					chip->state = 2;
				}
				return 0;
			}
		}
		if (calib_MdValueMin > 0 || calib_MdValueMin == 0) {
			if (delta > calib_MdValueMin - tol1 && delta < calib_MdValueMin + tol1) {
				TRI_KEY_LOG("tri_key:calib_Min:%d,calib_Sum:%d\n",
					calib_MdValueMin, calib_MdValueSum);

				if (sum > calib_MdValueSum + tol2 || sum < calib_MdValueSum - tol1) {
					chip->interf = 1;
					chip->state = 2;
				} else {
					chip->interf = 0;
					chip->state = 2;
					}
				return 0;
				}
			}
		//if (delta > calib_DnValueMin - tol2 && delta < calib_DnValueMin + tol1) {
		if (delta > tol4 && delta < calib_DnValueMin + tol1) {
			TRI_KEY_LOG("tri_key:calib_Min:%d,calib_Sum:%d\n",
				calib_DnValueMin, calib_DnValueSum);

			if (sum < calib_DnValueSum - tol1 || sum > calib_DnValueSum + tol2) {
				chip->interf = 1;
				chip->state = 3;
			} else {
				chip->interf = 0;
				chip->state = 3;
			}
			return 0;
		}
		chip->interf = 1;
		chip->state = 0;
	} else {//the hall data is negative number
		//if (delta > calib_UpValueMin - tol2 && delta < calib_UpValueMin + tol1) {
		if (delta > -tol3 && delta < calib_UpValueMin + tol1) {
			TRI_KEY_LOG("tri_key:calib_Min:%d,calib_Sum:%d\n",
				calib_UpValueMin, calib_UpValueSum);

			if (sum < calib_UpValueSum - tol2 || sum > calib_UpValueSum + tol1) {
				chip->interf = 1;
				chip->state = 1;
			} else {
				chip->interf = 0;
				chip->state = 1;
			}
			return 0;
		}
		if (calib_MdValueMin < 0) {
			if (delta > calib_MdValueMin - tol1 && delta < calib_MdValueMin + tol1) {
				TRI_KEY_LOG("tri_key:calib_Min:%d,calib_Sum:%d\n",
					calib_MdValueMin, calib_MdValueSum);

				if (sum > calib_MdValueSum + tol1 || sum < calib_MdValueSum - tol2) {
					chip->interf = 1;
					chip->state = 2;
				} else {
					chip->interf = 0;
					chip->state = 2;
				}
				return 0;
			}
		}
		if (calib_MdValueMin > 0 || calib_MdValueMin == 0) {
			if (delta > calib_MdValueMin - tol1 && delta < calib_MdValueMin + tol1) {
				TRI_KEY_LOG("tri_key:calib_Min:%d,calib_Sum:%d\n",
					calib_MdValueMin, calib_MdValueSum);

				if (sum > calib_MdValueSum + tol1 ||
					sum < calib_MdValueSum - tol2) {
					chip->interf = 1;
					chip->state = 2;
				} else {
					chip->interf = 0;
					chip->state = 2;
					}
				return 0;
				}
			}
		//if (delta > calib_DnValueMin - tol1 && delta < calib_DnValueMin + tol2) {
		if (delta > calib_DnValueMin - tol1 && delta < -tol4) {
			TRI_KEY_LOG("tri_key:calib_Min:%d,calib_Sum:%d\n",
				calib_DnValueMin, calib_DnValueSum);

			if (sum < calib_DnValueSum - tol2 || sum > calib_DnValueSum + tol1) {
				chip->interf = 1;
				chip->state = 3;
			} else {
				chip->interf = 0;
				chip->state = 3;
			}
			return 0;
		}
		chip->interf = 1;
		chip->state = 0;
	}
	return -EINVAL;

}


static int oneplus_get_data(struct extcon_dev_data *chip)
{
	int res = 0;
	mutex_lock(&chip->mtx);
	res = oneplus_hall_get_data(DHALL_0);
	if (res < 0)
		TRI_KEY_LOG("tri_key:get DHALL_0 data failed,res =%d\n", res);
	res = oneplus_hall_get_data(DHALL_1);
	if (res < 0) {
		TRI_KEY_LOG("tri_key:get DHALL_1 data failed,res =%d\n", res);
	}
	mutex_unlock(&chip->mtx);
	return res;
}

static int reupdata_threshold(struct extcon_dev_data *chip)
{
	int res = 0;
	int tolen = 22;
	switch (chip->position) {
	case UP_STATE:
		if (chip->dhall_data0 < 0 || chip->dhall_data1 < 0) {
			res = oneplus_hall_update_threshold(DHALL_1, UP_STATE,
			chip->dhall_data1-tolen, chip->dhall_data1+tolen);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold up:low:%d,high: %d\n",
			chip->dhall_data1-tolen, chip->dhall_data1+tolen);
		} else {
			res = oneplus_hall_update_threshold(DHALL_1, UP_STATE,
			chip->dhall_data1-tolen, chip->dhall_data1+tolen);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold up:low:%d,high: %d\n",
			chip->dhall_data1-tolen, chip->dhall_data1+tolen);
		}
		oneplus_hall_clear_irq(DHALL_1);
		res = oneplus_hall_update_threshold(DHALL_0, UP_STATE, -500, 500);
		if (res < 0) {
			TRI_KEY_LOG("updata_threshold fail:%d\n", res);
			goto fail;
		}
		break;
	case MID_STATE:
		if (chip->dhall_data0 < 0 || chip->dhall_data1 < 0) {
			res = oneplus_hall_update_threshold(DHALL_1, MID_STATE,
			chip->dhall_data1-tolen, chip->dhall_data1+tolen);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold up:low:%d,high:%d\n",
			chip->dhall_data1-tolen, chip->dhall_data1+tolen);
		} else {
			res = oneplus_hall_update_threshold(DHALL_1, MID_STATE,
			chip->dhall_data1-tolen, chip->dhall_data1+tolen);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold up:low:%d,high:%d\n",
			chip->dhall_data1-tolen, chip->dhall_data1+tolen);
		}
		oneplus_hall_clear_irq(DHALL_1);
		if (chip->dhall_data0 < 0 || chip->dhall_data1 < 0) {
			res = oneplus_hall_update_threshold(DHALL_0, MID_STATE,
			chip->dhall_data0-tolen, chip->dhall_data0+tolen);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold down:low:%d,high:%d\n",
			chip->dhall_data0-tolen, chip->dhall_data0+tolen);
		} else {
			res = oneplus_hall_update_threshold(DHALL_0, MID_STATE,
			chip->dhall_data0-tolen, chip->dhall_data0+tolen);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold down:low:%d,high:%d\n",
			chip->dhall_data0-tolen, chip->dhall_data0+tolen);
		}
		oneplus_hall_clear_irq(DHALL_0);
		break;
	case DOWN_STATE:
		res = oneplus_hall_update_threshold(DHALL_1, DOWN_STATE, -500, 500);
		if (res < 0) {
			TRI_KEY_LOG("updata_threshold fail:%d\n", res);
			goto fail;
		}
		if (chip->dhall_data0 < 0 || chip->dhall_data1 < 0) {
			res = oneplus_hall_update_threshold(DHALL_0, DOWN_STATE,
			chip->dhall_data0-tolen, chip->dhall_data0+tolen);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold down:low:%d,high:%d\n",
			chip->dhall_data0-tolen, chip->dhall_data0+tolen);
		} else {
			res = oneplus_hall_update_threshold(DHALL_0, DOWN_STATE,
			chip->dhall_data0-tolen, chip->dhall_data0+tolen);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold down:low:%d,high:%d\n",
			chip->dhall_data0-tolen, chip->dhall_data0+tolen);
		}
		oneplus_hall_clear_irq(DHALL_0);
		break;
		}
fail:
	last_d0 = chip->dhall_data0;
	last_d1 = chip->dhall_data1;
	last_interf = chip->interf;
	TRI_KEY_LOG("tri_key:last_d0 is %d ,last_d1 is %d\n", last_d0, last_d1);
	oneplus_hall_clear_irq(DHALL_0);
	oneplus_hall_clear_irq(DHALL_1);
	return res;
}

static void report_key_value(struct extcon_dev_data *chip)
{
	if (chip->position == DOWN_STATE) {
		extcon_set_state_sync(chip->edev, 1, 0);
		extcon_set_state_sync(chip->edev, 2, 1);
		extcon_set_state_sync(chip->edev, 3, 1);
		chip->state = 3;
		TRI_KEY_LOG("tri_key: report down key successful!\n");
	}
	if (chip->position == UP_STATE) { //near up hall
		extcon_set_state_sync(chip->edev, 1, 1);
		extcon_set_state_sync(chip->edev, 2, 1);
		extcon_set_state_sync(chip->edev, 3, 0);
		chip->state = 1;
		aw8697_op_haptic_stop();
		TRI_KEY_LOG("tri_key: report up key successful!\n");
	}
	if (chip->position == MID_STATE) {
		extcon_set_state_sync(chip->edev, 1, 1);
		extcon_set_state_sync(chip->edev, 2, 0);
		extcon_set_state_sync(chip->edev, 3, 1);
		chip->state = 2;
		aw8697_op_haptic_stop();
		TRI_KEY_LOG("tri_key: report mid key successful!\n");
	} else
		TRI_KEY_LOG("no report\n");
}

static int report_calibration_location(struct extcon_dev_data *chip)
{
	oneplus_get_data(chip);
	get_position(chip);
	reupdata_threshold(chip);
	if (chip->position == last_position) {
		TRI_KEY_LOG("no report\n");
		goto err;
	} else
		report_key_value(chip);
	return 0;
err:
	return -EINVAL;
}

//note:work in irq context
int oneplus_hall_irq_handler(unsigned int id)
{
	TRI_KEY_LOG("%d tri_key:call :%s\n", id, __func__);
	if (!g_the_chip) {
		TRI_KEY_LOG("g_the_chip null\n ");
		return -EINVAL;
	} else {
		schedule_work(&g_the_chip->dwork);
	}
	return IRQ_HANDLED;
}


static void tri_key_dev_work(struct work_struct *work)
{
	struct extcon_dev_data *chip = container_of(work,
			struct extcon_dev_data, dwork);
	int res = 0;
	int position = -1;
	int diff0 = 0;
	int diff1 = 0;
	int count = 0;
	int dhall0_sum = 0;
	int dhall1_sum = 0;
	int aver0 = 0;
	int aver1 = 0;
	ktime_t starttime, endtime;
	u64 usecs64;
	int usecs;

	starttime = ktime_get();
//	msleep(50);
//get data
	res = oneplus_get_data(chip);
	if (res < 0) {
		TRI_KEY_LOG("tri_key:get hall data failed!\n");
		goto fail;
	}
	TRI_KEY_LOG("tri_key:data1 is %d, data0 is %d\n",
				chip->dhall_data1, chip->dhall_data0);

//judge interference
	res = judge_interference(chip);
	TRI_KEY_LOG("tri_key:chip->interf is %d ,chip->state is %d\n",
					chip->interf, chip->state);
	if (!last_interf && chip->interf) {
		msleep(200);
		oneplus_get_data(chip);
		TRI_KEY_LOG("tri_key:data1 is %d, data0 is %d\n",
					chip->dhall_data1, chip->dhall_data0);

		judge_interference(chip);
	}
//get position
	if (!chip->interf) {
		hrtimer_cancel(&tri_key_timer);
		time = 1;
		if (!last_interf) {
			interf_count = 0;
			get_position(chip);
		TRI_KEY_LOG("tri_key:the position is %d\n", chip->position);
		} else {
			msleep(50);
			oneplus_get_data(chip);
			judge_interference(chip);
			if (chip->interf)
				goto FINAL;
			else
				get_position(chip);
			}
		}
	else {
		hrtimer_cancel(&tri_key_timer);
		TRI_KEY_LOG("tri_key:time0 is %d\n", time);
		hrtimer_start(&tri_key_timer, ktime_set(time, 0),
			HRTIMER_MODE_REL);
		while (count < 4) {
			msleep(35);
			oneplus_hall_get_data(DHALL_0);
			oneplus_hall_get_data(DHALL_1);
			dhall0_sum += chip->dhall_data0;
			dhall1_sum += chip->dhall_data1;
			count++;
		}
		aver0 = dhall0_sum / 4;
		aver1 = dhall1_sum / 4;
		if (!last_interf) {//from no interference to constant interference
			diff0 = aver0 - chip->dhall_data0;
			diff1 = aver1 - chip->dhall_data1;
			TRI_KEY_LOG("tri_key:diff0 is %d,diff1 is %d\n",
				diff0, diff1);
			if ((diff0 > -10 && diff0 < 10) && (diff1 > -10 && diff1 < 10)) {
				chip->position = last_position;
				goto UPDATA_HTRES;
			} else {//inconstant interference
				last_interf = chip->interf;
				goto FINAL;
			}
		}
		diff0 = aver0 - chip->dhall_data0;
		diff1 = aver1 - chip->dhall_data1;
		TRI_KEY_LOG("tri_key:diff0 is %d,diff1 is %d\n",
			diff0, diff1);

//inconstantly interference
		if ((diff0 < -10 || diff0 > 10) &&
				(diff1 < -10 || diff1 > 10)) {
			interf_count++;
			if (interf_count == 15) {
				TRI_KEY_LOG("tri_key:count = 15,msleep 5s\n");
				msleep(5000);
				interf_count = 0;
				goto FINAL;
			}
			TRI_KEY_LOG("tri_key:inconstantlt interference\n");
			//last_interf = chip->interf;
			reupdata_threshold(chip);
			goto FINAL;
		}

		chip->dhall_data0 = aver0;
		chip->dhall_data1 = aver1;
		position = interf_get_position(chip);
		if (position == -22) {
			TRI_KEY_LOG("tri_key:get position failed\n");
			}
		else
			chip->position = position;
	}
	TRI_KEY_LOG("tri_key:t_diff0 is %d,t_diff1 is %d\n",
	chip->dhall_data0 - last_d0, chip->dhall_data1 - last_d1);
//updata threshold
UPDATA_HTRES:
	res = reupdata_threshold(chip);
	if (res < 0) {
		TRI_KEY_LOG("tri_key:updata_threshold failed!\n");
		goto fail;
		}
	mutex_lock(&chip->mtx);
// report key value
	if (chip->position == last_position)
		goto FINAL;
	else {
		report_key_value(chip);
		last_position = chip->position;
		endtime = ktime_get();
		usecs64 = ktime_to_ns(ktime_sub(endtime, starttime));
		do_div(usecs64, NSEC_PER_USEC);
		usecs = usecs64;
		if (usecs == 0)
			usecs = 1;
		TRI_KEY_LOG("report key after %ld.%03ld msecs\n",
			usecs / USEC_PER_MSEC, usecs % USEC_PER_MSEC);
	}
fail:
	if (res < 0)
		TRI_KEY_LOG("tri_key:dev_work failed,res =%d\n", res);
FINAL:
	oneplus_hall_enable_irq(DHALL_0, 1);
	oneplus_hall_enable_irq(DHALL_1, 1);
	TRI_KEY_LOG("%s achieve\n", __func__);
	mutex_unlock(&chip->mtx);
}

static ssize_t dhall_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//int fb = -1;
	//short hall_up_data[128] = {0};
	//short hall_down_data[128] = {0};
	//char hall_data_bufs[512] = {0};
	//char hall_data_buf[16] = {0};
	//int data_count = 0;
	//int i = 0;
	//int up_sum = 0;
	//int down_sum = 0;
	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	oneplus_hall_get_data(DHALL_0);
	oneplus_hall_get_data(DHALL_1);
//save hall data
/*	hall_up_data[data_count] = g_the_chip->dhall_data0;
	hall_down_data[data_count] = g_the_chip->dhall_data1;
	data_count++;
	fd = sys_open("/sdcard/trikey_hall_data.csv",  O_WRONLY | O_CREAT | O_APPEND, 0);
	if (fd < 0) {
		MOTOR_ERR("open log file /sdcard/hall_data.csv failed.\n");
	}
	if (fd >= 0) {
//		if (g_the_chip->state)
//			sys_write(fd, "1\n", 2);
//		else //interference
//			sys_write(fd, "0\n", 2);
		for (i = 0; i < data_count; i++) {
			memset(hall_data_buf, 0, sizeof(hall_data_buf));
			sprintf(hall_data_buf, "%d, ", hall_up_data[i]);
			strcat(hall_data_bufs, hall_data_buf);
		}
		strcat(hall_data_bufs, "\n");
		sys_write(fd, hall_data_bufs, strlen(hall_data_bufs));
		memset(hall_data_bufs, 0 , sizeof(hall_data_bufs));

		for (i = 0; i < data_count; i++) {
			memset(hall_data_buf, 0, sizeof(hall_data_buf));
			sprintf(hall_data_buf, "%d,", hall_down_data[i]);
			strcat(hall_data_bufs, hall_data_buf);
		}
		strcat(hall_data_bufs, "\n");
		sys_write(fd, hall_data_bufs, strlen(hall_data_bufs));
		sys_write(fd, "\n", 1);
		sys_close(fd);
	}
*/

	return snprintf(buf, PAGE_SIZE, "%d, %d\n",
		g_the_chip->dhall_data0, g_the_chip->dhall_data1);
}


static ssize_t tri_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	//int position =-1;
	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}
	oneplus_hall_get_data(DHALL_0);
	oneplus_hall_get_data(DHALL_1);
//	judge_interference(g_the_chip);
	//position = get_position(g_the_chip);

	return snprintf(buf, PAGE_SIZE, "%d\n", g_the_chip->state);
}

static enum hrtimer_restart tri_key_status_timeout(struct hrtimer *timer)
{
	schedule_work(&tri_key_timeout_work);
	return HRTIMER_NORESTART;
}

static void tri_key_timeout_work_func(struct work_struct *work)
{
	oneplus_get_data(g_the_chip);
	judge_interference(g_the_chip);
	if (g_the_chip->interf) {
		time = time * 2;
		TRI_KEY_LOG("tri_key:time1 is %d\n", time);
		if (time > 2)
			time = 2;
		}
	else {
		get_position(g_the_chip);
		if (g_the_chip->position == last_position)
			return;
		reupdata_threshold(g_the_chip);
		report_key_value(g_the_chip);
		last_position = g_the_chip->position;
		time = 1;
		}
	return;
}


static short Sum(short value0, short value1)
{
    short sum = 0;
	sum = value0 + value1;
	return sum;
}
static short Minus(short value0, short value1)
{
	short minus = 0;
	minus = value0 - value1;
	return minus;
}

void initialCalibValue(short calib_dnHall_UpV, short calib_dnHall_MdV,
			short calib_dnHall_DnV, short calib_upHall_UpV,
			short calib_upHall_MdV, short calib_upHall_DnV)
{
	calib_UpValueSum = Sum(calib_dnHall_UpV,calib_upHall_UpV);
	calib_MdValueSum = Sum(calib_dnHall_MdV,calib_upHall_MdV);
	calib_DnValueSum = Sum(calib_dnHall_DnV,calib_upHall_DnV);
	calib_UpValueMin = Minus(calib_upHall_UpV,calib_dnHall_UpV);
	calib_MdValueMin = Minus(calib_upHall_MdV,calib_dnHall_MdV);
	calib_DnValueMin = Minus(calib_upHall_DnV,calib_dnHall_DnV);
	calib_upHall_UM_distance = Minus(calib_upHall_UpV, calib_upHall_MdV);
	calib_upHall_MD_distance = Minus(calib_upHall_MdV, calib_upHall_DnV);
	calib_dnHall_UM_distance = Minus(calib_dnHall_UpV, calib_dnHall_MdV);
	calib_dnHall_MD_distance = Minus(calib_dnHall_MdV, calib_dnHall_DnV);
	calib_upHall_UD_distance = Minus(calib_upHall_UpV, calib_upHall_DnV);
	calib_dnHall_UD_distance = Minus(calib_dnHall_UpV, calib_dnHall_DnV);
}


static ssize_t hall_data_calib_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n%d\n",-1,-1);
	}
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d\n",
		g_the_chip->dnHall_UpV, g_the_chip->upHall_UpV,
		g_the_chip->dnHall_MdV, g_the_chip->upHall_MdV,
		g_the_chip->dnHall_DnV, g_the_chip->upHall_DnV);
}

static ssize_t hall_data_calib_store(struct device *pdev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int data[6] = {0};
	char temp[35] = {0};
	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		return count;
	}
	strlcpy(temp, buf, sizeof(temp));
	TRI_KEY_LOG("temp is %s:\n", temp);
	if (sscanf(temp, "%d,%d,%d,%d,%d,%d", &data[0], &data[1], &data[2],
		&data[3], &data[4], &data[5]) == 6) {
		g_the_chip->dnHall_UpV = data[0];
		g_the_chip->upHall_UpV = data[1];
		g_the_chip->dnHall_MdV = data[2];
		g_the_chip->upHall_MdV = data[3];
		g_the_chip->dnHall_DnV = data[4];
		g_the_chip->upHall_DnV = data[5];
		TRI_KEY_ERR("data[%d %d %d %d %d %d]\n", data[0], data[1],
				data[2], data[3], data[4], data[5]);
	} else {
		TRI_KEY_ERR("fail\n");
	}
	initialCalibValue(g_the_chip->dnHall_UpV, g_the_chip->dnHall_MdV,
			g_the_chip->dnHall_DnV, g_the_chip->upHall_UpV,
			g_the_chip->upHall_MdV, g_the_chip->upHall_DnV);
	report_calibration_location(g_the_chip);
	return count;
}
static ssize_t hall_dump_regs_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 _buf[1024] = {0};

	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		return 0;
	}

	oneplus_hall_dump_regs(1, _buf);
	return sprintf(buf, "%s\n %s\n", _buf);
}
static ssize_t hall_debug_info_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp = 0;
	char buffer[4] = {0};

	if (count > 2)
		return count;
	strlcpy(buffer, buf, sizeof(buffer));
	if (1 == sscanf(buffer, "%d", &tmp)) {
		tri_key_debug = tmp;
	} else {
		TRI_KEY_DEBUG("invalid content: '%s', length = %zd\n", buf, count);
	}

	return count;
}
static ssize_t hall_debug_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", tri_key_debug);
}

static ssize_t hall_set_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", tol1);
}

static ssize_t hall_set_value_store(struct device *pdev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp = 0;
	char buffer[4] = {0};

	strlcpy(buffer, buf, sizeof(buffer));
	TRI_KEY_LOG("buffer is %s\n", buffer);
	if (sscanf(buffer, "%d", &tmp) == 1) {
		tol1 = tmp;
		TRI_KEY_LOG("tol1 is %d\n", tol1);
	}
	else
		TRI_KEY_ERR("invalid content: %s, length = %zd\n", buf, count);

	return count;
}

static DEVICE_ATTR(hall_data, S_IRUGO | S_IWUSR, dhall_data_show, NULL);
static DEVICE_ATTR(tri_state, S_IRUGO | S_IWUSR, tri_state_show, NULL);
static DEVICE_ATTR(hall_data_calib, 0644,
		hall_data_calib_show, hall_data_calib_store);
static DEVICE_ATTR(hall_dump_regs, 0644, hall_dump_regs_show, NULL);
static DEVICE_ATTR(hall_debug_info, 0644,hall_debug_info_show, hall_debug_info_store);
static DEVICE_ATTR(hall_set_value, 0644, hall_set_value_show, hall_set_value_store);

static struct attribute *tri_key_attributes[] = {
	&dev_attr_tri_state.attr,
	&dev_attr_hall_data.attr,
	&dev_attr_hall_data_calib.attr,
	&dev_attr_hall_dump_regs.attr,
	&dev_attr_hall_debug_info.attr,
	&dev_attr_hall_set_value.attr,
	NULL
};


static struct attribute_group tri_key_attribute_group = {
	.attrs = tri_key_attributes
};

static int tri_key_platform_probe(struct platform_device *pdev)
{
	struct extcon_dev_data *chip = NULL;
	int err = 0;
	int res = 0;
	//int hall_value_min = 0;

	TRI_KEY_LOG("call %s\n", __func__);

	if (!g_the_chip) {
		chip = kzalloc(sizeof(struct extcon_dev_data), GFP_KERNEL);
		if (!chip) {
			TRI_KEY_ERR("kzalloc err\n");
			return -ENOMEM;
		}
		g_the_chip = chip;
	} else {
		chip = g_the_chip;
	}
	mutex_init(&chip->mtx);
	chip->dev = &pdev->dev;
	err = sysfs_create_group(&pdev->dev.kobj, &tri_key_attribute_group);
	if (err) {
		TRI_KEY_ERR("tri_key:sysfs_create_group was failed(%d)\n", err);
		goto sysfs_create_fail;
	}

	if (0 && (!chip->dhall_up_ops || !chip->dhall_down_ops)) {
		TRI_KEY_ERR("no dhall available\n");
		goto fail;
	}
// extcon registration
	chip->edev = devm_extcon_dev_allocate(chip->dev, tristate_extcon_tab);
	chip->edev->name = "tri_state_key";
	err = devm_extcon_dev_register(chip->dev, chip->edev);

	if (err < 0) {
		TRI_KEY_ERR("%s register extcon dev failed\n", __func__);
		goto err_extcon_dev_register;
	}

	INIT_WORK(&chip->dwork, tri_key_dev_work);
	hrtimer_init(&tri_key_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tri_key_timer.function = tri_key_status_timeout;
	INIT_WORK(&tri_key_timeout_work, tri_key_timeout_work_func);
//get data when reboot
	res = oneplus_get_data(chip);
	if (res < 0) {
		TRI_KEY_LOG("tri_key:get hall data failed!\n");
		goto fail;
	}
	TRI_KEY_LOG("tri_key:data1 is %d, data0 is %d\n",
				chip->dhall_data1, chip->dhall_data0);

//get position when reboot
	reboot_get_position(chip);
//set threshold when reboot;
	err = reupdata_threshold(chip);
	if (err < 1) {
		TRI_KEY_ERR("%s reupdata_threshold failed\n", __func__);
		goto fail;
	}
//report key value
	report_key_value(chip);
	last_position = chip->position;
	err = oneplus_hall_set_detection_mode(DHALL_0, DETECTION_MODE_INTERRUPT);
	TRI_KEY_LOG("tri_key:set 0 detection mode\n");
	if (err < 0) {
		TRI_KEY_ERR("%s set HALL0 detection mode failed %d\n",
			__func__, err);
		goto fail;
	}
	err = oneplus_hall_set_detection_mode(DHALL_1, DETECTION_MODE_INTERRUPT);
	TRI_KEY_LOG("tri_key:set 1 detection mode\n");
	if (err < 0) {
		TRI_KEY_ERR("%s set HALL1 detection mode failed %d\n", __func__, err);
		goto fail;
	}
	TRI_KEY_LOG("%s probe success.\n", __func__);
	return 0;

fail:
	kfree(chip);
	g_the_chip = NULL;
	TRI_KEY_LOG("fail\n");
	return -EINVAL;
sysfs_create_fail:
	sysfs_remove_group(&pdev->dev.kobj, &tri_key_attribute_group);

err_extcon_dev_register:
	devm_extcon_dev_unregister(chip->dev, chip->edev);
	TRI_KEY_LOG("fail\n");
	return -EINVAL;

}

static int tri_key_platform_remove(struct platform_device *pdev)
{
	if (g_the_chip) {
		cancel_work_sync(&g_the_chip->dwork);
		extcon_dev_unregister(g_the_chip->edev);
		kfree(g_the_chip);
		g_the_chip = NULL;
	}
	return 0;
}

static const struct of_device_id tristate_dev_of_match[] = {
	{ .compatible = "oneplus,hall_tri_state_key"},
	{},
};
MODULE_DEVICE_TABLE(of, of_motor_match);

static struct platform_driver tri_key_platform_driver = {
	.probe		= tri_key_platform_probe,
	.remove		= tri_key_platform_remove,
	.driver		= {
		.name	= "tri-state-key",
		.of_match_table = tristate_dev_of_match,
	},
};

static int __init tri_key_platform_init(void)
{
	int res = 0;
	TRI_KEY_LOG("call : %s\n", __func__);
	res = platform_driver_register(&tri_key_platform_driver);
	if (res < 0)
		TRI_KEY_LOG("%s failed\n", __func__);
	return res;
}

module_init(tri_key_platform_init);

static void __exit tri_key_platform_exit(void)
{
	platform_driver_unregister(&tri_key_platform_driver);
}
module_exit(tri_key_platform_exit);
MODULE_DESCRIPTION("oem tri_state_key driver");
MODULE_LICENSE("GPL v2");

