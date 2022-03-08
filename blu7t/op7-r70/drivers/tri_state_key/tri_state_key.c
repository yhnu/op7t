/*
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

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
#include "../extcon/extcon.h"
//#include <linux/project_info.h>

#define DRV_NAME	"tri_state_key"
#define KEY_LOG(fmt, args...)	printk(KERN_INFO DRV_NAME" %s : "fmt, __FUNCTION__, ##args)
/*
 *
 *					KEY1(GPIO1)	KEY2(GPIO92)
 *	pin1 connect to pin4	0	            1         | MUTE
 *	pin2 connect to pin5	1	            1         | Do Not Disturb
 *	pin4 connect to pin3	1	            0         | Normal
 */
enum  {
	MODE_UNKNOWN,
	MODE_MUTE,
	MODE_DO_NOT_DISTURB,
	MODE_NORMAL,
	MODE_MAX_NUM
	} tri_mode_t;

static const unsigned int tristate_extcon_tab[] = {
	MODE_MUTE,
	MODE_DO_NOT_DISTURB,
	MODE_NORMAL,
	EXTCON_NONE,
};

struct extcon_dev_data {
	int irq_key3;
	int irq_key2;
	int irq_key1;
	int key1_gpio;
	int key2_gpio;
	int key3_gpio;

	struct regulator *vdd_io;

	struct work_struct work;
	struct extcon_dev *edev; //change 1
	struct device *dev;

	struct timer_list s_timer;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;

};

static struct extcon_dev_data *extcon_data;
static DEFINE_MUTEX(sem);
static int set_gpio_by_pinctrl(void)
{
	return pinctrl_select_state(extcon_data->key_pinctrl,
		extcon_data->set_state);
}
/*op add to fix GCE-7551 begin*/
extern int aw8697_op_haptic_stop(void);
/*op add to fix GCE-7551 end*/

static void extcon_dev_work(struct work_struct *work)
{
	int key[3] = {0, 0, 0};
	int hw_version = 0;
	/*op add to fix ISTRACKING-34823 begin*/
	static int pre_key0, pre_key1, pre_key2;
	/*op add to fix ISTRACKING-34823 end*/
	/*hw 13 use special tri state key no use key2*/
	//hw_version=get_hw_version();
	KEY_LOG("hw_version=%d\n", hw_version);
	if (hw_version == 13) {
		key[0] = gpio_get_value(extcon_data->key1_gpio);
		key[2] = gpio_get_value(extcon_data->key3_gpio);

		KEY_LOG("key[0]=%d,key[1]=%d,key[2]=%d\n",
			 key[0], key[1], key[2]);
		if (key[0] == 1 && key[2] == 1) {
			extcon_set_state_sync(extcon_data->edev, 1, 1);
			extcon_set_state_sync(extcon_data->edev, 2, 0);
			extcon_set_state_sync(extcon_data->edev, 3, 1);
		} else if (key[0] == 0 && key[2] == 1) {
			extcon_set_state_sync(extcon_data->edev, 1, 0);
			extcon_set_state_sync(extcon_data->edev, 2, 1);
			extcon_set_state_sync(extcon_data->edev, 3, 1);
		} else if (key[0] == 1 && key[2] == 0) {
			extcon_set_state_sync(extcon_data->edev, 1, 1);
			extcon_set_state_sync(extcon_data->edev, 2, 1);
			extcon_set_state_sync(extcon_data->edev, 3, 0);
		}
	} else {
		key[0] = gpio_get_value(extcon_data->key1_gpio);
		key[1] = gpio_get_value(extcon_data->key2_gpio);
		key[2] = gpio_get_value(extcon_data->key3_gpio);
		KEY_LOG("key[0]=%d,key[1]=%d,key[2]=%d\n",
			key[0], key[1], key[2]);
		/*op add to fix ISTRACKING-34823 begin*/
		if (!key[0] || !key[1] || !key[2]) {
			if (pre_key0 == key[0] && pre_key1 == key[1]
					&& pre_key2 == key[2]) {
				pre_key0 = key[0];
				pre_key1 = key[1];
				pre_key2 = key[2];
				return;
			}
		}
		/*op add to fix ISTRACKING-34823 end*/
		/*op add to fix GCE-7551 begin*/
		if (key[0] && key[1] && key[2])
			return;
		if (!key[0] && !key[1] && !key[2])
			return;
		if (!key[0] && !key[1] && key[2])
			return;
		if (!key[0] && key[1] && !key[2])
			return;
		if (key[0] && !key[1] && !key[2])
			return;
		/*op add to fix GCE-7551 end*/
		extcon_set_state_sync(
				extcon_data->edev, 1, key[0]);
		extcon_set_state_sync(
				extcon_data->edev, 2, key[1]);
		extcon_set_state_sync(
				extcon_data->edev, 3, key[2]);
		/*op add to fix GCE-7551 begin*/
		if (!key[2] ||  !key[1])
			aw8697_op_haptic_stop();
		/*op add to fix GCE-7551 end*/
		/*op add to fix ISTRACKING-34823 begin*/
		if (!key[0] || !key[1] || !key[2]) {
			pre_key0 = key[0];
			pre_key1 = key[1];
			pre_key2 = key[2];
		}
		/*op add to fix ISTRACKING-34823 end*/
	}
}


static irqreturn_t extcon_dev_interrupt(int irq, void *_dev)
{
	schedule_work(&extcon_data->work);
	return IRQ_HANDLED;
}

static void timer_handle(unsigned long arg)
{
	schedule_work(&extcon_data->work);
}

#ifdef CONFIG_OF
static int extcon_dev_get_devtree_pdata(struct device *dev)
{
	struct device_node *node;

	node = dev->of_node;
	if (!node)
		return -EINVAL;

	extcon_data->key3_gpio =
	of_get_named_gpio(node, "tristate,gpio_key3", 0);
	if ((!gpio_is_valid(extcon_data->key3_gpio)))
		return -EINVAL;
	KEY_LOG("extcon_data->key3_gpio=%d\n", extcon_data->key3_gpio);

	extcon_data->key2_gpio =
		of_get_named_gpio(node, "tristate,gpio_key2", 0);
	if ((!gpio_is_valid(extcon_data->key2_gpio)))
		return -EINVAL;
	KEY_LOG("extcon_data->key2_gpio=%d\n", extcon_data->key2_gpio);

	extcon_data->key1_gpio =
		of_get_named_gpio(node, "tristate,gpio_key1", 0);
	if ((!gpio_is_valid(extcon_data->key1_gpio)))
		return -EINVAL;
	KEY_LOG("extcon_data->key1_gpio=%d\n", extcon_data->key1_gpio);

	return 0;
}
#else
static inline int
extcon_dev_get_devtree_pdata(struct device *dev)
{
	KEY_LOG("inline function\n");
	return 0;
}
#endif

static int tristate_dev_probe(struct platform_device *pdev)
{
	struct device *dev;
	int ret = 0;
	KEY_LOG("SYSY\n");
	dev = &pdev->dev;

	extcon_data = kzalloc(sizeof(struct extcon_dev_data), GFP_KERNEL);
	if (!extcon_data)
		return -ENOMEM;

	extcon_data->dev = dev;


	extcon_data->key_pinctrl = devm_pinctrl_get(extcon_data->dev);

	if (IS_ERR_OR_NULL(extcon_data->key_pinctrl)) {
		KEY_LOG("Failed to get pinctrl\n");
		goto err_extcon_dev_register;
	}
	extcon_data->set_state = pinctrl_lookup_state(extcon_data->key_pinctrl,
		"pmx_tri_state_key_active");
	if (IS_ERR_OR_NULL(extcon_data->set_state)) {
		KEY_LOG("Failed to lookup_state\n");
		goto err_extcon_dev_register;
	}

	set_gpio_by_pinctrl();

	ret = extcon_dev_get_devtree_pdata(dev);
	if (ret) {
		KEY_LOG("parse device tree fail!!!\n");
		goto err_extcon_dev_register;
	}


	/* extcon registration */
	extcon_data->edev =
	devm_extcon_dev_allocate(extcon_data->dev, tristate_extcon_tab);
	extcon_data->edev->name = DRV_NAME;

	ret = devm_extcon_dev_register(extcon_data->dev, extcon_data->edev);
	if (ret < 0)
		goto err_extcon_dev_register;

	//config irq gpio and request irq
	ret = gpio_request(extcon_data->key1_gpio, "tristate_key1");
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(extcon_data->key1_gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	extcon_data->irq_key1 = gpio_to_irq(extcon_data->key1_gpio);
	if (extcon_data->irq_key1 < 0) {
		ret = extcon_data->irq_key1;
		goto err_detect_irq_num_failed;
	}

	ret = request_irq(extcon_data->irq_key1, extcon_dev_interrupt,
		IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
		"tristate_key1", extcon_data);
	if (ret < 0)
		goto err_request_irq;

	ret = gpio_request(extcon_data->key2_gpio,
		"tristate_key2");
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(extcon_data->key2_gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	extcon_data->irq_key2 = gpio_to_irq(extcon_data->key2_gpio);
	if (extcon_data->irq_key2 < 0) {
		ret = extcon_data->irq_key2;
		goto err_detect_irq_num_failed;
	}

	ret = request_irq(extcon_data->irq_key2, extcon_dev_interrupt,
		IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
		"tristate_key2", extcon_data);
	if (ret < 0)
		goto err_request_irq;

	ret = gpio_request(extcon_data->key3_gpio,
		"tristate_key3");
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(extcon_data->key3_gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	extcon_data->irq_key3 = gpio_to_irq(extcon_data->key3_gpio);
	if (extcon_data->irq_key3 < 0) {
		ret = extcon_data->irq_key3;
		goto err_detect_irq_num_failed;
	}

	ret = request_irq(extcon_data->irq_key3, extcon_dev_interrupt,
		IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
		"tristate_key3", extcon_data);
	if (ret < 0)
		goto err_request_irq;

	INIT_WORK(&extcon_data->work, extcon_dev_work);

	init_timer(&extcon_data->s_timer);
	extcon_data->s_timer.function = &timer_handle;
	extcon_data->s_timer.expires = jiffies + 5*HZ;

	add_timer(&extcon_data->s_timer);

	enable_irq_wake(extcon_data->irq_key1);
	enable_irq_wake(extcon_data->irq_key2);
	enable_irq_wake(extcon_data->irq_key3);

	return 0;

err_request_gpio:
	devm_extcon_dev_unregister(extcon_data->dev, extcon_data->edev);
err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(extcon_data->key2_gpio);
	gpio_free(extcon_data->key1_gpio);
	gpio_free(extcon_data->key3_gpio);
err_extcon_dev_register:
	kfree(extcon_data);

	return ret;
}

static int tristate_dev_remove(struct platform_device *pdev)
{
	cancel_work_sync(&extcon_data->work);
	gpio_free(extcon_data->key1_gpio);
	gpio_free(extcon_data->key2_gpio);
	gpio_free(extcon_data->key3_gpio);
	extcon_dev_unregister(extcon_data->edev);
	kfree(extcon_data);

	return 0;
}
#ifdef CONFIG_OF
static const struct of_device_id tristate_dev_of_match[] = {
	{ .compatible = "oneplus, tri-state-key", },
	{ },
};
MODULE_DEVICE_TABLE(of, tristate_dev_of_match);
#endif

static struct platform_driver tristate_dev_driver = {
	.probe		= tristate_dev_probe,
	.remove		= tristate_dev_remove,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = tristate_dev_of_match,
	},
};
static int __init oem_tristate_init(void)
{
	return platform_driver_register(&tristate_dev_driver);
}
module_init(oem_tristate_init);

static void __exit oem_tristate_exit(void)
{
	platform_driver_unregister(&tristate_dev_driver);
}
module_exit(oem_tristate_exit);
MODULE_DESCRIPTION("oem tri_state_key driver");
MODULE_LICENSE("GPL v2");

