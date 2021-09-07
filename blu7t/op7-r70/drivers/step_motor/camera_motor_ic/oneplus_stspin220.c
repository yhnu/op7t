/************************************************************************************
** Copyright (C), 2013-2018, ONEPLUS Mobile Comm Corp., Ltd
** File: oneplus_stspin220.c
**
** Description:
**      Definitions for m1120 motor driver ic stspin220.
**
**************************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/qpnp/pwm.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/qpnp/pin.h>
#include <soc/oneplus/oneplus_project.h>

#include "oneplus_stspin220.h"
#include "../oneplus_motor.h"

static struct oneplus_sts_chip *g_the_chip = NULL;
static void stspin220_check_motor_type(struct oneplus_sts_chip *chip);
static int stspin220_set_working_mode (int mode);

static void stspin220_parse_dts(struct oneplus_sts_chip * chip)
{
	struct device_node *np = chip->dev->of_node;
	int rc = 0;

	chip->pctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pctrl)) {
		MOTOR_ERR("failed to get pinctrl\n");
	};

	chip->pwm_dev = of_pwm_get(np, NULL);

	if (IS_ERR_OR_NULL(chip->pwm_dev)) {
		MOTOR_LOG("pwm_dev not specified \n");
	} else {
		MOTOR_LOG("request pwm seccess\n");
	}

	chip->boost_gpio = of_get_named_gpio(np, "qcom,boost-gpio", 0);
	if (!gpio_is_valid(chip->boost_gpio)) {
		MOTOR_LOG("md-boost-gpio gpio not specified\n");
	} else {
		rc = gpio_request(chip->boost_gpio, "boost-gpio");
		if (rc)
			MOTOR_LOG("request md-boost-gpio gpio failed, rc=%d\n",rc);
	}

	chip->step_gpio = of_get_named_gpio(np, "qcom,step-gpio", 0);
	if (!gpio_is_valid(chip->step_gpio)) {
		MOTOR_LOG("md-step-gpio gpio not specified\n");
	} else {
		rc = gpio_request(chip->step_gpio, "step-gpio");
		if (rc)
			MOTOR_LOG("request md-step-gpio failed, rc=%d\n",rc);
		//else
		//	gpio_set_value(chip->step_gpio, 0);
	}

	chip->sleep_gpio = of_get_named_gpio(np, "qcom,sleep-gpio", 0);
	if (!gpio_is_valid(chip->sleep_gpio)) {
		MOTOR_LOG("md-sleep-gpio gpio not specified\n");
	} else {
		rc = gpio_request(chip->sleep_gpio, "sleep-gpio");
		if (rc)
			MOTOR_LOG("request md-sleep-gpio gpio failed, rc=%d\n",rc);
		else
			gpio_set_value(chip->sleep_gpio, 0);
	}

    chip->enable_gpio = of_get_named_gpio(np, "qcom,enable-gpio", 0);
	if (!gpio_is_valid(chip->enable_gpio)) {
		MOTOR_LOG("md-enable-gpio gpio not specified\n");
	} else {
		rc = gpio_request(chip->enable_gpio, "enable-gpio");
		if (rc)
			MOTOR_LOG("request md-enavle-gpio gpio failed, rc=%d\n",rc);
		else
			gpio_set_value(chip->enable_gpio, 0);
	}


	chip->sleep1_gpio = of_get_named_gpio(np, "qcom,sleep1-gpio", 0);
	if (!gpio_is_valid(chip->sleep1_gpio)) {
		MOTOR_LOG("md-sleep-gpio1 gpio not specified\n");
	} else {
		rc = gpio_request(chip->sleep1_gpio, "sleep-gpio1");
		if (rc)
			MOTOR_LOG("request md-sleep-gpio1 gpio failed, rc=%d\n",rc);
		else
			gpio_set_value(chip->sleep1_gpio, 0);
	}

	chip->dir_gpio = of_get_named_gpio(np, "qcom,dir-gpio", 0);
	if (!gpio_is_valid(chip->dir_gpio)) {
		MOTOR_LOG("md-dir-gpio gpio not specified\n");
	} else {
		rc = gpio_request(chip->dir_gpio, "dir-gpio");
		if (rc)
			MOTOR_LOG("request md-dir-gpio gpio failed, rc=%d\n",rc);
		else
			gpio_set_value(chip->dir_gpio, 0);
	}

	chip->dir_switch_gpio = of_get_named_gpio(np, "qcom,dir_switch-gpio", 0);
	if (!gpio_is_valid(chip->dir_switch_gpio)) {
		MOTOR_LOG("dir_switch_gpio not specified\n");
	} else {
		rc = gpio_request(chip->dir_switch_gpio, "dir_switch-gpio");
		if (rc)
			MOTOR_LOG("request md-m0-gpio failed, rc=%d\n",rc);
	}

	MOTOR_LOG("%d %d %d %d %d %d %d\n",chip->boost_gpio,chip->sleep_gpio,chip->sleep1_gpio,
						chip->dir_gpio,chip->step_gpio,chip->dir_switch_gpio,chip->enable_gpio);
}

static int stspin220_init_dir_switch(struct oneplus_sts_chip * chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	};

	chip->dir_switch_state = pinctrl_lookup_state(chip->pctrl, "dir_switch_gpio_input_high");
	if (IS_ERR_OR_NULL(chip->dir_switch_state)) {
		ret = PTR_ERR(chip->dir_switch_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	};

	pinctrl_select_state(chip->pctrl,chip->dir_switch_state);

	msleep(10);

	chip->dir_switch = gpio_get_value(chip->dir_switch_gpio);

	chip->dir_switch_state = pinctrl_lookup_state(chip->pctrl, "dir_switch_gpio_input_low");
	if (IS_ERR_OR_NULL(chip->dir_switch_state)) {
		ret = PTR_ERR(chip->dir_switch_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	};
	pinctrl_select_state(chip->pctrl,chip->dir_switch_state);

	return 0;

}

static int stspin220_init_pwm_config(struct oneplus_sts_chip * chip , int is_pwm_mode)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	};

	if (is_pwm_mode)
		chip->pwm_state = pinctrl_lookup_state(chip->pctrl, "pwm_config");
	else
		chip->pwm_state = pinctrl_lookup_state(chip->pctrl, "pwm_config_as_gpio"); 
	if (IS_ERR_OR_NULL(chip->pwm_state)) {
		ret = PTR_ERR(chip->pwm_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	};

	pinctrl_select_state(chip->pctrl,chip->pwm_state);

	return 0;

}

static int stspin220_init_boost(struct oneplus_sts_chip * chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	};

	chip->boost_state = pinctrl_lookup_state(chip->pctrl, "boost");
	if (IS_ERR_OR_NULL(chip->boost_state)) {
		ret = PTR_ERR(chip->boost_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	};

	pinctrl_select_state(chip->pctrl,chip->boost_state);

	return 0;

}

static int stspin220_init_sleep_gpio(struct oneplus_sts_chip * chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	};

	chip->sleep_state = pinctrl_lookup_state(chip->pctrl, "sleep_gpio");
	if (IS_ERR_OR_NULL(chip->sleep_state)) {
		ret = PTR_ERR(chip->sleep_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	};

	pinctrl_select_state(chip->pctrl,chip->sleep_state);

	return 0;

}

static int stspin220_init_dir_gpio(struct oneplus_sts_chip * chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	};

	chip->dir_state = pinctrl_lookup_state(chip->pctrl, "dir_gpio");
	if (IS_ERR_OR_NULL(chip->dir_state)) {
		ret = PTR_ERR(chip->dir_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	};

	pinctrl_select_state(chip->pctrl,chip->dir_state);

	return 0;

}

static int stspin220_init_enable_gpio(struct oneplus_sts_chip * chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	};

	chip->enable_state = pinctrl_lookup_state(chip->pctrl, "enable_gpio");
	if (IS_ERR_OR_NULL(chip->enable_state)) {
		ret = PTR_ERR(chip->enable_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	};

	pinctrl_select_state(chip->pctrl,chip->enable_state);

	return 0;

}

static int stspin220_hardware_init(struct oneplus_sts_chip * chip)
{
	int ret = 0;

	//init boost
	ret = stspin220_init_boost(chip);
	if (ret < 0){
		MOTOR_ERR("stspin220_init_boost %d \n",ret);
		return -EINVAL;;
	}
	gpio_set_value(chip->boost_gpio, 0);


	//config pwm for step-gpio
	ret = stspin220_init_pwm_config(chip,0);
	if (ret < 0){
		MOTOR_ERR("stspin220_init_pwm_config %d \n",ret);
		return -EINVAL;
	}

	//config sleep_gpio
	ret = stspin220_init_sleep_gpio(chip);
	if (ret < 0){
		MOTOR_ERR("stspin220_init_sleep_gpio %d \n",ret);
		return -EINVAL;;
	}

	//config dir_gpio
	ret = stspin220_init_dir_gpio(chip);
	if (ret < 0){
		MOTOR_ERR("stspin220_init_dir_gpio %d \n",ret);
		return -EINVAL;;
	}

	//config dir_switch _gpio
	ret = stspin220_init_dir_switch(chip);
	if (ret < 0){
		MOTOR_ERR("drv8834_init_dir_switch %d \n",ret);
		return -EINVAL;;
	}

	//config enable_gpio _gpio
	ret = stspin220_init_enable_gpio(chip);
	if (ret < 0){
		MOTOR_ERR("stspin220_init_enable_gpio %d \n",ret);
		return -EINVAL;;
	}

	stspin220_check_motor_type(chip);

	return 0;

}

static int stspin220_set_power(int mode)
{
	static bool first_init = true;

	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip null \n ");
		return -EINVAL;
	}

	MOTOR_ERR("stspin220_set_power call mode: %d power on:%d\n",mode,MOTOR_POWER_ON);

	if (mode == MOTOR_POWER_ON) {
		stspin220_set_working_mode(MOTOR_MODE_1_32);
		stspin220_init_pwm_config(g_the_chip,1);
		if (first_init) {
			first_init = false;
			gpio_set_value(g_the_chip->boost_gpio, 1);//boost should be always on
			msleep(10);
		}
		gpio_set_value(g_the_chip->enable_gpio, 1);
	} else {
		gpio_set_value(g_the_chip->sleep_gpio, 0);
		gpio_set_value(g_the_chip->sleep1_gpio, 0);
		gpio_set_value(g_the_chip->enable_gpio, 0);
	}
	return 0;
}

static int stspin220_set_direction(int dir)
{
	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip null \n ");
		return -EINVAL;
	}

	if (g_the_chip->motor_type == MOTOR_FI5) { // fi 5 motor
		if (g_the_chip->dir_switch) {
			if (dir == MOTOR_UPWARD) {
				gpio_set_value(g_the_chip->dir_gpio, 1);
			} else {
				gpio_set_value(g_the_chip->dir_gpio, 0);
			}
		} else {
			if (is_project(ONEPLUS_17107)) {
				if (get_PCB_Version() >= HW_VERSION__14) {
					if (dir == MOTOR_UPWARD) {
						gpio_set_value(g_the_chip->dir_gpio, 1);
					} else {
						gpio_set_value(g_the_chip->dir_gpio, 0);
					}
				} else {
					if (dir == MOTOR_UPWARD) {
						gpio_set_value(g_the_chip->dir_gpio, 0);
					} else {
						gpio_set_value(g_the_chip->dir_gpio, 1);
					}
				}
			} else {
					if (get_PCB_Version() >= HW_VERSION__11) {
					if (dir == MOTOR_UPWARD) {
						gpio_set_value(g_the_chip->dir_gpio, 1);
					} else {
						gpio_set_value(g_the_chip->dir_gpio, 0);
					}
				} else {
					if (dir == MOTOR_UPWARD) {
						gpio_set_value(g_the_chip->dir_gpio, 0);
					} else {
						gpio_set_value(g_the_chip->dir_gpio, 1);
					}
				}
			}
		}
	} else { //fi 6 motor
		if (dir == MOTOR_UPWARD) {
			gpio_set_value(g_the_chip->dir_gpio, 1);
		} else {
			gpio_set_value(g_the_chip->dir_gpio, 0);
		}
	}
	return 0;
}

static int stspin220_set_working_mode (int mode)
{
	int ret = 0;

	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip null \n ");
		return -EINVAL;
	}

	gpio_set_value(g_the_chip->sleep_gpio, 0);
	gpio_set_value(g_the_chip->sleep1_gpio, 0);

	switch (mode) {
	case MOTOR_MODE_FULL:
		MOTOR_ERR("MOTOR_MODE_FULL call \n");
		stspin220_init_pwm_config(g_the_chip,0);
		gpio_set_value(g_the_chip->dir_gpio,0);
		break;
	case MOTOR_MODE_1_16:
		MOTOR_ERR("MOTOR_MODE_1_16 call \n");
		stspin220_init_pwm_config(g_the_chip,0);
		gpio_set_value(g_the_chip->dir_gpio,0);
		break;
	case MOTOR_MODE_1_32:
		MOTOR_ERR("MOTOR_MODE_1_32 call \n");
		stspin220_init_pwm_config(g_the_chip,0);
		gpio_set_value(g_the_chip->dir_gpio,0);
		break;
	default:
		MOTOR_ERR("MOTOR_MODE_default call \n");
		stspin220_init_pwm_config(g_the_chip,0);
		gpio_set_value(g_the_chip->dir_gpio,0);
		break;
	}

	gpio_set_value(g_the_chip->sleep_gpio, 1);
	gpio_set_value(g_the_chip->sleep1_gpio, 1);

	MOTOR_ERR("config change %d %d %d\n",
		gpio_get_value(g_the_chip->sleep_gpio),
		gpio_get_value(g_the_chip->sleep1_gpio),
		gpio_get_value(g_the_chip->dir_gpio));
	return 0;
}


static int stspin220_calculate_pwm_count(int L, int mode)
{
	int pwm_count = 0;
	int mdmode = 0;

	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip null \n");
		return 0;
	}
	switch (mode) {
	case MOTOR_MODE_FULL:
		mdmode = 1;
		break;
	case MOTOR_MODE_1_16:
		mdmode = 16;
		break;
	case MOTOR_MODE_1_32:
		mdmode = 32;
		break;
	default:
		mdmode = 32;
		break;
	}
	//PWM = (L*18.06*20*32 )/2.4
	pwm_count = (L * RATIO_B * RATIO_C *RATIO_D) / (RATIO_A * 100);
	//pwm_count = (L * RATIO_B_FI_6 * RATIO_C *RATIO_D) / (RATIO_A * 100);
	return pwm_count;
}

static int stspin220_pwm_config(int duty_ns, int period_ns)
{
	int pwm_count = 0;
	int mdmode = 0;

	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip null \n");
		return -EINVAL;
	}

	MOTOR_LOG("duty_ns %d period_ns %d \n",duty_ns,period_ns);

	return pwm_config(g_the_chip->pwm_dev, duty_ns, period_ns);
}

static int stspin220_pwm_enable(void)
{
	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip null \n");
		return -EINVAL;
	}

	return pwm_enable(g_the_chip->pwm_dev);
}

static int stspin220_pwm_disable(void)
{
	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip null \n");
		return -EINVAL;
	}

	pwm_disable(g_the_chip->pwm_dev);
	return 0;
}

static int stspin220_get_motor_type(void)
{
	if (g_the_chip == NULL) {
		MOTOR_LOG("g_the_chip null \n");
		return MOTOR_UNKNOWN;
	}

	return g_the_chip->motor_type;
}

static void stspin220_check_motor_type(struct oneplus_sts_chip *chip)
{
    if (get_PCB_Version() == HW_VERSION__12) { // fi 6 motor
        chip->motor_type = MOTOR_FI6;
    } else { //fi 5 motor
        chip->motor_type = MOTOR_FI5;
    }

    MOTOR_LOG("motor_type %d \n",chip->motor_type);
}

static int  stspin220_get_all_config(int* config ,int count)
{
	if (g_the_chip == NULL || count > 6) {
		MOTOR_LOG("g_the_chip null \n");
		return -EINVAL;
	}

	config[0] = gpio_get_value(g_the_chip->sleep_gpio);
	config[1] = gpio_get_value(g_the_chip->step_gpio);
	config[2] = gpio_get_value(g_the_chip->sleep1_gpio);
	config[3] = gpio_get_value(g_the_chip->enable_gpio);
	config[4] = gpio_get_value(g_the_chip->dir_gpio);
	config[5] = g_the_chip->motor_type;

	MOTOR_ERR("config change %d %d %d %d %d %d\n",config[0],config[1],config[2],
		config[3],config[4],config[5]);

	return 0;
}


struct oneplus_motor_operations  stspin220_ops = {
	.set_power = stspin220_set_power,
	.set_direction = stspin220_set_direction,
	.set_working_mode = stspin220_set_working_mode,
	.get_all_config = stspin220_get_all_config,
	.calculate_pwm_count = stspin220_calculate_pwm_count,
	.pwm_config = stspin220_pwm_config,
	.pwm_enable = stspin220_pwm_enable,
	.pwm_disable = stspin220_pwm_disable,
	.get_motor_type = stspin220_get_motor_type,
};

static int stspin220_platform_probe(struct platform_device *pdev)
{
	struct oneplus_sts_chip *chip = NULL;
	int ret = 0;

	MOTOR_LOG("call\n");

	chip = devm_kzalloc(&pdev->dev,sizeof(struct oneplus_sts_chip), GFP_KERNEL);
	if (!chip) {
		MOTOR_ERR("kernel memory alocation was failed");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;

	stspin220_parse_dts(chip);

	ret = stspin220_hardware_init(chip);
	if (ret < 0){
		MOTOR_ERR("stspin220_hardware_init %d \n",ret);
		return -EINVAL;;
	}

	oneplus_register_motor("stspin220",&stspin220_ops);

	g_the_chip = chip;

	MOTOR_LOG("success \n");
	return 0;
}

static int stspin220_platform_remove(struct platform_device *pdev)
{
	if (g_the_chip) {
		gpio_free(g_the_chip->boost_gpio);
		gpio_free(g_the_chip->dir_gpio);
		gpio_free(g_the_chip->enable_gpio);
		gpio_free(g_the_chip->sleep_gpio);
		gpio_free(g_the_chip->sleep1_gpio);
		kfree(g_the_chip);
		g_the_chip = NULL;
	}
	return 0;
}

static const struct of_device_id of_motor_drv_match[] = {
	{ .compatible = "motor_drv-220"},
	{},
};
MODULE_DEVICE_TABLE(of, of_motor_match);

static struct platform_driver motor_drv_driver = {
	.probe		= stspin220_platform_probe,
	.remove		= stspin220_platform_remove,
	.driver		= {
		.name	= "stspin220",
		.of_match_table = of_motor_drv_match,
	},
};

static int __init stspin220_init(void)
{
	MOTOR_LOG("call\n");
	platform_driver_register(&motor_drv_driver);
	return 0;
}

static void __exit stspin220_exit(void)
{
	MOTOR_LOG("call\n");
}

module_init(stspin220_init);
module_exit(stspin220_exit);
MODULE_DESCRIPTION("camera motor driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("mofei@oneplus.com");

