/****************************************************************************************
** Copyright (C), 2013-2019, ONEPLUS Mobile Comm Corp., Ltd
** File: infrared_power_control.c
**
** Description:
**      Definitions to control infrared led power.
**
****************************************************************************************/
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/syscalls.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include "infrared_power_control.h"

static struct oneplus_infrared_state *g_infrared_state;


static ssize_t infrared_power_enable_store(struct device* pdev, struct device_attribute* attr,
			                               const char* buff, size_t count)
{
	unsigned long enable = 0;
	int res = 0;

	INFRARED_LOG("call");

	if (g_infrared_state == NULL) {
		INFRARED_ERR("g_infrared_state == NULL \n");
		return count;
	}

	if (sscanf(buff, "%lu", &enable) == 1) {
		g_infrared_state->infrared_power_enable = enable;
		INFRARED_LOG("would set infrared_power_enable : %d", g_infrared_state->infrared_power_enable);
	}

  if (enable > 0) {
    res = regulator_enable(g_infrared_state->vdd);
	  //g_infrared_state->infrared_shutdown_state = 0;
    INFRARED_LOG("enable, res : %d \n", res);
  } else {
    res = regulator_disable(g_infrared_state->vdd);
    //g_infrared_state->infrared_shutdown_state = 1;
    INFRARED_LOG("disable, res : %d \n", res);
  }

  if (regulator_is_enabled(g_infrared_state->vdd) > 0) {
    g_infrared_state->infrared_shutdown_state = 0;
    g_infrared_state->infrared_power_enable = 1;
  } else {
    g_infrared_state->infrared_shutdown_state = 1;
    g_infrared_state->infrared_power_enable = 0;
  }

	INFRARED_LOG("infrared_power_enable : %d, enable : %d", g_infrared_state->infrared_power_enable, enable);

	return count;
}

static ssize_t infrared_power_enable_show(struct device* dev,struct device_attribute* attr, char* buf)
{
	if (g_infrared_state == NULL) {
		INFRARED_ERR("g_infrared_state == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_infrared_state->infrared_power_enable);
}


static ssize_t infrared_shut_down_state_store(struct device* pdev, struct device_attribute* attr,
			                               const char* buff, size_t count)
{
  unsigned long infrared_shut_down_state = 0;

	INFRARED_LOG("call");

	if (g_infrared_state == NULL) {
		INFRARED_ERR("g_infrared_state == NULL \n");
		return count;
	}

	if (sscanf(buff, "%lu", &infrared_shut_down_state) == 1) {
		g_infrared_state->infrared_shutdown_state = infrared_shut_down_state;
		INFRARED_LOG("would set infrared_shut_down_state, free_fall_irq_times : %d", g_infrared_state->infrared_shutdown_state);
	}
	INFRARED_LOG("infrared_shut_down_state : %d", infrared_shut_down_state);

	return count;
}

static ssize_t infrared_shut_down_state_show(struct device* dev,struct device_attribute* attr, char* buf)
{
	if (g_infrared_state == NULL) {
		INFRARED_ERR("g_infrared_state == NULL \n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_infrared_state->infrared_shutdown_state);
}

static ssize_t infrared_shut_down_state2_store (struct device *pdev, struct device_attribute *attr,
				 const char *buff, size_t count)
{
  unsigned long infrared_shut_down_state = 0;

  INFRARED_LOG ("call");

  if (g_infrared_state == NULL) {
    INFRARED_ERR ("g_infrared_state == NULL \n");
    return count;
  }

  if (sscanf (buff, "%lu", &infrared_shut_down_state) == 1) {
    g_infrared_state->infrared_shutdown_state2 = infrared_shut_down_state;
    INFRARED_LOG ("would set infrared_shut_down_state2 : %d", g_infrared_state->infrared_shutdown_state2);
  }
  INFRARED_LOG ("infrared_shut_down_state : %d", infrared_shut_down_state);

  return count;
}

static ssize_t infrared_shut_down_state2_show (struct device *dev,
				    struct device_attribute *attr, char *buf)
{
  if (g_infrared_state == NULL) {
    INFRARED_ERR ("g_infrared_state == NULL \n");
    return snprintf (buf, PAGE_SIZE, "%d\n", 0);
  }

  return snprintf (buf, PAGE_SIZE, "%d\n", g_infrared_state->infrared_shutdown_state2);
}



static DEVICE_ATTR(infrared_power_enable, S_IRUGO | S_IWUSR, infrared_power_enable_show, infrared_power_enable_store);
static DEVICE_ATTR(infrared_shut_down_state, S_IRUGO | S_IWUSR, infrared_shut_down_state_show, infrared_shut_down_state_store);
static DEVICE_ATTR (infrared_shut_down_state2, S_IRUGO | S_IWUSR, infrared_shut_down_state2_show, infrared_shut_down_state2_store);
static struct attribute*  __attributes[] = {
	&dev_attr_infrared_power_enable.attr,
	&dev_attr_infrared_shut_down_state.attr,
  &dev_attr_infrared_shut_down_state2.attr,
	NULL
};

static struct attribute_group __attribute_group = {
	.attrs = __attributes
};


static void oneplus_parameter_init(struct oneplus_infrared_state* state)
{
	INFRARED_LOG("call \n");

	if (g_infrared_state == NULL) {
		INFRARED_ERR("g_infrared_state == NULL \n");
		return;
	}

	g_infrared_state->infrared_power_enable = 0;
	g_infrared_state->infrared_shutdown_state = 0;
  g_infrared_state->infrared_shutdown_state2 = 1;
  g_infrared_state->irq_times = 0;

	return;
}

static void infrared_irq_check_work_func (struct work_struct *work)
{
  struct delayed_work *dwork = to_delayed_work (work);
  struct oneplus_infrared_state *state = container_of(dwork, struct oneplus_infrared_state, infrared_irq_check_work);

  if (state == NULL) {
    INFRARED_ERR ("infrared_irq_check_work_func error, state == NULL");
    return;
  }

  INFRARED_LOG ("irq_times : %d, infrared_shutdown_state2 : %d \n", state->irq_times, state->infrared_shutdown_state2);

  if (state->irq_times == 1) {
    disable_irq_nosync (state->infrared_irq);
    enable_irq (state->infrared_irq);
  } else if (state->irq_times >= 2) {
    INFRARED_LOG ("infrared notify event, irq_times : %d \n", state->irq_times);
    state->infrared_shutdown_state2 = 1;
  } else {
    INFRARED_LOG ("unknow event, irq_times : %d \n", state->irq_times);
  }

  state->irq_times = 0;

  return;
}

static irqreturn_t oneplus_infrared_detect_handler (int irq, void *dev_id)
{
  if (g_infrared_state == NULL) {
    INFRARED_ERR ("g_infrared_state == NULL");
    return -EINVAL;
  }

  g_infrared_state->irq_times++;

  if (g_infrared_state->irq_times == 1)
      mod_delayed_work (system_highpri_wq, &g_infrared_state->infrared_irq_check_work, msecs_to_jiffies (1));

  return IRQ_HANDLED;
}


static void oneplus_infrared_interrupt_register (struct oneplus_infrared_state *state)
{
  struct device_node *np = NULL;
  int err = 0;

  np = state->dev->of_node;

  state->pctrl = devm_pinctrl_get (state->dev);
  if (IS_ERR (state->pctrl)) {
    INFRARED_ERR ("failed to get pinctrl \n");
    return;
  };

  state->shutdown_state = pinctrl_lookup_state (state->pctrl, "infrared_input");\

  if (IS_ERR (state->shutdown_state)) {
    err = PTR_ERR (state->shutdown_state);
    INFRARED_ERR ("pinctrl_lookup_state failed, err : %d \n", err);
    return;
  };

  pinctrl_select_state (state->pctrl, state->shutdown_state);
  state->infrared_gpio = of_get_named_gpio (np, "infrared,irq-gpio", 0);

  if (!gpio_is_valid (state->infrared_gpio)) {
    INFRARED_LOG ("gpio not specified \n");
  } else {
    err = gpio_request (state->infrared_gpio, "infrared-irq-gpio");
    if (err)
      INFRARED_LOG ("request infrared_gpio gpio failed, err : %d \n", err);

    err = gpio_direction_input (state->infrared_gpio);
    msleep (50);
    state->infrared_irq = gpio_to_irq (state->infrared_gpio);

    if (request_irq(state->infrared_irq, &oneplus_infrared_detect_handler, IRQ_TYPE_EDGE_RISING, "infrared", NULL)) {
      INFRARED_ERR ("IRQ LINE NOT AVAILABLE!!\n");
      return;
    }
    irq_set_irq_wake (state->infrared_irq, 1);
  }

  INFRARED_LOG ("gpio %d irq:%d \n", state->infrared_gpio, state->infrared_irq);
}


static int infrared_platform_probe(struct platform_device* pdev)
{
	struct oneplus_infrared_state* p_infrared_state = NULL;
	int err = 0;

	INFRARED_LOG("call \n");

	if (g_infrared_state == NULL) {
		p_infrared_state = kzalloc(sizeof(struct oneplus_infrared_state), GFP_KERNEL);
		if (!p_infrared_state) {
			INFRARED_ERR("kzalloc err \n");
			return -ENOMEM;
		}
		g_infrared_state = p_infrared_state;
	} else {
		p_infrared_state = g_infrared_state;
	}

	p_infrared_state->dev = &pdev->dev;

	//get vdd power control
	p_infrared_state->vdd = devm_regulator_get(p_infrared_state->dev, "vdd");
        if (IS_ERR(p_infrared_state->vdd)) {
                err = PTR_ERR(p_infrared_state->vdd);
                INFRARED_ERR("regulator get vdd failed, err : %d \n", err);
                goto power_get_fail;
        } else {
		if (regulator_count_voltages(p_infrared_state->vdd) > 0) {
			err = regulator_set_voltage(p_infrared_state->vdd, 2960000, 2960000);
			if (err) {
				INFRARED_ERR("regulator set vdd voltage failed, err : %d \n", err);
			}
			err = regulator_set_load(p_infrared_state->vdd, 20000);
			if (err) {
				INFRARED_ERR("regulator set vdd load failed, err : %d \n", err);
			}
		}
	}

	//create node
	err = sysfs_create_group(&pdev->dev.kobj, &__attribute_group);
	if(err) {
		INFRARED_ERR("sysfs_create_group failed, err : %d \n", err);
		goto sysfs_create_fail;
	}

	oneplus_parameter_init(p_infrared_state);

  oneplus_infrared_interrupt_register(p_infrared_state);

  INIT_DELAYED_WORK(&p_infrared_state->infrared_irq_check_work, infrared_irq_check_work_func);

	INFRARED_LOG("success. \n");

	return 0;

power_get_fail:
sysfs_create_fail:
	kfree(p_infrared_state);
	g_infrared_state = NULL;
	INFRARED_LOG("fail \n");
	return -EINVAL;
}

static int infrared_platform_remove(struct platform_device* pdev)
{
	INFRARED_LOG("call \n");

	if (g_infrared_state) {
		sysfs_remove_group(&pdev->dev.kobj, &__attribute_group);
		kfree(g_infrared_state);
		g_infrared_state = NULL;
	}

	return 0;
}

static int infrared_platform_suspend(struct platform_device* pdev, pm_message_t state)
{
	INFRARED_LOG("call \n");

	return 0;
}

static int infrared_platform_resume(struct platform_device* pdev)
{
	//INFRARED_LOG("call \n");

	return 0;
}

static void infrared_platform_shutdown(struct platform_device* pdev)
{
	INFRARED_LOG("call \n");

	return;
}

static const struct of_device_id of_infrared_match[] = {
	{ .compatible = "oneplus-infrared"},
	{},
};

MODULE_DEVICE_TABLE(of, of_infrared_match);

static struct platform_driver infrared_platform_driver = {
	.probe		= infrared_platform_probe,
	.remove		= infrared_platform_remove,
	.suspend	= infrared_platform_suspend,
	.resume		= infrared_platform_resume,
	.shutdown   = infrared_platform_shutdown,
	.driver		= {
		.name	= "oneplus_infrared",
		.of_match_table = of_infrared_match,
	},
};

static int __init infrared_power_control_platform_init(void)
{
	INFRARED_LOG("call \n");

	platform_driver_register(&infrared_platform_driver);
	return 0;
}

late_initcall(infrared_power_control_platform_init);


MODULE_DESCRIPTION("infrared proximity led power control driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("quentin.lin");
