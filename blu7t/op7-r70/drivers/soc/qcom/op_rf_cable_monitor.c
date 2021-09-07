/*For OEM project monitor RF cable connection status,
 * and config different RF configuration
 */

#include <linux/export.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/sys_soc.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/project_info.h>
#include <linux/soc/qcom/smem.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <soc/qcom/subsystem_restart.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/op_rf_cable_monitor.h>

#define RF_CABLE_STATE_ACTIVE   "oem_rf_cable_active"

#define SDX5X_RF_CABLE_UEVENT
#ifdef SDX5X_RF_CABLE_UEVENT
#include <linux/platform_device.h>
#include <linux/kdev_t.h>

static struct class *sdx5x_rf_class;
static struct device *sdx5x_rf_device;
static int esoc_ssr_reason_feature_enable = 0;
#endif

static struct project_info *project_info_desc;

struct cable_data {
	int irq_0;
	int irq_1;
	int cable_gpio_0;
	int cable_gpio_1;
	int support_timer;
	struct delayed_work work;
	struct workqueue_struct *wqueue;
	struct device *dev;
	struct wakeup_source wl;
	atomic_t running;
	int rf_v2;
	int rf_v3;
	int rf_v3_pre;
	spinlock_t lock;
	int enable;
	int is_rf_factory_mode;
	int gpio_state;
	struct pinctrl *gpio_pinctrl;
	struct pinctrl_state *gpio_pinctrl_active;
	struct pinctrl_state *gpio_pinctrl_suspend;
#ifdef SDX5X_RF_CABLE_UEVENT
	bool connected;
#endif
};
static struct cable_data *rf_cable_data;

static char *cmdline_find_option(char *str)
{
    return strnstr(saved_command_line, str, strlen(saved_command_line));
}

int modify_rf_cable_smem_info(uint32 status)
{
    size_t size;

    project_info_desc = qcom_smem_get(QCOM_SMEM_HOST_ANY,SMEM_PROJECT_INFO, &size);

    if (IS_ERR_OR_NULL(project_info_desc))
        pr_err("%s: get project_info failure\n", __func__);
    else {
        project_info_desc->rf_v3 = status;
        pr_err("%s: rf_cable: %d\n",
            __func__, project_info_desc->rf_v3);
    }
    return 0;
}

int modify_rf_v2_info(uint32 status)
{
    size_t size;

    project_info_desc = qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_PROJECT_INFO, &size);

    if (IS_ERR_OR_NULL(project_info_desc))
        pr_err("%s: get project_info failure\n", __func__);
    else {
        project_info_desc->rf_v2 = status;
        pr_err("%s: rf_cable: %d\n",
                __func__, project_info_desc->rf_v3);
    }
    return 0;
}

static void irq_cable_enable(int enable)
{
	unsigned long flags;

	if (!rf_cable_data->support_timer) {

		spin_lock_irqsave(&rf_cable_data->lock, flags);
		if (enable) {
			enable_irq(rf_cable_data->irq_0);
			enable_irq(rf_cable_data->irq_1);
		} else {
			disable_irq_nosync(rf_cable_data->irq_0);
			disable_irq_nosync(rf_cable_data->irq_1);
		}
		spin_unlock_irqrestore(&rf_cable_data->lock, flags);
	}
}

static void cable_connect_state(int enable)
{
	char *connected[2]    = { "CABLE_STATE=CONNECTED", NULL };
	char *disconnected[2] = { "CABLE_STATE=DISCONNECTED", NULL };

	if (esoc_ssr_reason_feature_enable == 1) {
		if (enable) {
			kobject_uevent_env(&sdx5x_rf_device->kobj,
					KOBJ_CHANGE, connected);
			rf_cable_data->connected = true;
			pr_err("%s: sent uevent %s\n", __func__,
					connected[0]);
		} else {
			kobject_uevent_env(&sdx5x_rf_device->kobj,
					KOBJ_CHANGE, disconnected);
			pr_err("%s: sent uevent %s\n", __func__,
					disconnected[0]);
			rf_cable_data->connected = false;
		}
	}

}

static void rf_cable_work(struct work_struct *work)
{
	int current_gpio_state = 0;

	irq_cable_enable(0);
	pr_err("%s rf_v3_pre=%d, rf_v3=%d gpio0=%d gpio1=%d,\n",
			__func__, rf_cable_data->rf_v3_pre,
			rf_cable_data->rf_v3,
			gpio_get_value(rf_cable_data->cable_gpio_0),
			gpio_get_value(rf_cable_data->cable_gpio_1));
	current_gpio_state = gpio_get_value(rf_cable_data->cable_gpio_0)*10 +
		gpio_get_value(rf_cable_data->cable_gpio_1);

	if (rf_cable_data->gpio_state != current_gpio_state) {
		pr_err("%s gpio_state=%d, current_gpio_state=%d ignore\n",
		__func__, rf_cable_data->gpio_state, current_gpio_state);
		goto out;
	}

	rf_cable_data->rf_v3 =
		gpio_get_value(rf_cable_data->cable_gpio_0) ||
		gpio_get_value(rf_cable_data->cable_gpio_1);

	if (rf_cable_data->rf_v3 != rf_cable_data->rf_v3_pre) {
		modify_rf_cable_smem_info(rf_cable_data->rf_v3);
		op_restart_modem();
#ifdef SDX5X_RF_CABLE_UEVENT
		cable_connect_state(rf_cable_data->rf_v3);
#endif
	}
	rf_cable_data->rf_v3_pre =
		gpio_get_value(rf_cable_data->cable_gpio_0) ||
		gpio_get_value(rf_cable_data->cable_gpio_1);

out:
	irq_cable_enable(1);

	if (rf_cable_data->support_timer) {
		queue_delayed_work(rf_cable_data->wqueue, &rf_cable_data->work,
				msecs_to_jiffies(2*HZ));
	}
}

irqreturn_t cable_interrupt(int irq, void *_dev)
{

    rf_cable_data->gpio_state = gpio_get_value(rf_cable_data->cable_gpio_0)*10 +
                                gpio_get_value(rf_cable_data->cable_gpio_1);
    __pm_wakeup_event(&rf_cable_data->wl,
        msecs_to_jiffies(CABLE_WAKELOCK_HOLD_TIME));
    queue_delayed_work(rf_cable_data->wqueue,
        &rf_cable_data->work, msecs_to_jiffies(500));
    return IRQ_HANDLED;
}

static ssize_t rf_factory_mode_proc_read_func(struct file *file,
        char __user *user_buf, size_t count, loff_t *ppos)
{
    char page[PAGESIZE];
    int len;

    len = scnprintf(page, sizeof(page), "%d\n", rf_cable_data->is_rf_factory_mode);

    return simple_read_from_buffer(user_buf,
            count, ppos, page, len);
}

static ssize_t rf_factory_mode_proc_write_func(struct file *file,
        const char __user *buffer, size_t count, loff_t *ppos)
{

	int enable = 0;
	char buf[10] = {0};
	int ret = 0;

    if (copy_from_user(buf, buffer, count))  {
        pr_err("%s: read proc input error.\n", __func__);
        return count;
    }

    ret = kstrtoint(buf, 0, &enable);
    if (ret < 0)
        return ret;

    pr_err("%s: input : %d\n", enable, __func__);
    irq_cable_enable(0);
    rf_cable_data->is_rf_factory_mode = enable;
    if (!rf_cable_data->is_rf_factory_mode) {
        modify_rf_v2_info(0);
        pr_err("%s: Modem restart due to RF Factory Mode exit\n", enable, __func__);
        op_restart_modem();
    } else {
        modify_rf_v2_info(2);
        pr_err("%s: Modem restart due to RF Factory Mode entry\n", enable, __func__);
        op_restart_modem();
    }
    irq_cable_enable(1);

    return count;
}

static const struct file_operations rf_factory_mode_proc_fops = {
    .write = rf_factory_mode_proc_write_func,
    .read =  rf_factory_mode_proc_read_func,
    .open = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t rf_cable_proc_read_func(struct file *file,
    char __user *user_buf, size_t count, loff_t *ppos)
{
    char page[PAGESIZE];
    int len;

    len = scnprintf(page, sizeof(page), "%d\n", rf_cable_data->enable);

    return simple_read_from_buffer(user_buf,
        count, ppos, page, len);
}

static ssize_t rf_cable_proc_write_func(struct file *file,
    const char __user *buffer, size_t count, loff_t *ppos)
{
    int enable = 0;
    char buf[10];
    int ret;

    if (copy_from_user(buf, buffer, count))  {
        pr_err("%s: read proc input error.\n", __func__);
        return count;
    }

    ret = kstrtoint(buf, 0, &enable);
    if (ret < 0)
        return ret;

    irq_cable_enable(0);

    if (enable != rf_cable_data->enable) {
        rf_cable_data->enable = enable;
        if (!rf_cable_data->enable) {

            rf_cable_data->rf_v3 = 1;

            modify_rf_cable_smem_info(1);
            if (!rf_cable_data->rf_v3_pre)
                op_restart_modem();
            rf_cable_data->rf_v3_pre = 1;
        } else {

            rf_cable_data->rf_v3 = gpio_get_value(rf_cable_data->cable_gpio_0) ||
                gpio_get_value(rf_cable_data->cable_gpio_1);

            modify_rf_cable_smem_info(rf_cable_data->rf_v3);
            if (rf_cable_data->rf_v3 != rf_cable_data->rf_v3_pre)
                op_restart_modem();
            rf_cable_data->rf_v3_pre =
            gpio_get_value(rf_cable_data->cable_gpio_0) ||
            gpio_get_value(rf_cable_data->cable_gpio_1);
        }
    }
    irq_cable_enable(1);

    return count;
}

static const struct file_operations rf_enable_proc_fops = {
    .write = rf_cable_proc_write_func,
    .read =  rf_cable_proc_read_func,
    .open = simple_open,
    .owner = THIS_MODULE,
};

int create_rf_cable_procfs(void)
{
    int ret = 0;

    if (!proc_create("rf_cable_config",
        0644, NULL, &rf_enable_proc_fops)) {
        pr_err("%s: proc_create enable fail!\n", __func__);
        ret = -1;
    }
    rf_cable_data->enable = 1;

    if (!proc_create("rf_factory_mode",
                0644, NULL, &rf_factory_mode_proc_fops)) {
        pr_err("%s: proc_create re_factory_mode fail!\n", __func__);
        ret = -1;
    }
    rf_cable_data->is_rf_factory_mode = 0;
    return ret;
}

static int op_rf_request_named_gpio(const char *label, int *gpio)
{
    struct device *dev = rf_cable_data->dev;
    struct device_node *np = dev->of_node;
    int rc = of_get_named_gpio(np, label, 0);

    if (rc < 0) {
        dev_err(dev, "failed to get '%s'\n", label);
        *gpio = rc;
        return rc;
    }
    *gpio = rc;
    rc = devm_gpio_request(dev, *gpio, label);
    if (rc) {
        dev_err(dev, "failed to request gpio %d\n", *gpio);
        return rc;
    }
    dev_info(dev, "%s - gpio: %d\n", label, *gpio);
    return 0;
}
static int rf_cable_gpio_pinctrl_init(struct platform_device *pdev)
{
	int retval;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	rf_cable_data->gpio_pinctrl = devm_pinctrl_get(&(pdev->dev));
	if (IS_ERR_OR_NULL(rf_cable_data->gpio_pinctrl)) {
		retval = PTR_ERR(rf_cable_data->gpio_pinctrl);
		dev_dbg(&pdev->dev, "Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	rf_cable_data->gpio_pinctrl_active
		= pinctrl_lookup_state(rf_cable_data->gpio_pinctrl,
				RF_CABLE_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(rf_cable_data->gpio_pinctrl_active)) {
		retval = PTR_ERR(rf_cable_data->gpio_pinctrl_active);
		dev_err(&pdev->dev,
				"Can not lookup %s pinstate %d\n",
				RF_CABLE_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	if (rf_cable_data->gpio_pinctrl) {

		retval = pinctrl_select_state(rf_cable_data->gpio_pinctrl,
				rf_cable_data->gpio_pinctrl_active);
		if (retval < 0) {
			dev_err(&pdev->dev,
					"failed to select pin to active state");
		}
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(rf_cable_data->gpio_pinctrl);
err_pinctrl_get:
	rf_cable_data->gpio_pinctrl = NULL;
	return retval;
}

#ifdef SDX5X_RF_CABLE_UEVENT
static ssize_t state_show(struct device *pdev, struct device_attribute *attr,
		char *buf)
{
	char *state = "DISCONNECTED";

	if (rf_cable_data->connected == true)
		state = "CONNECTED";
	return snprintf(buf, sizeof(state), "%s\n", state);
}
static ssize_t state_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t size)
{
	int val;
	int ret = 0;

	ret = kstrtoint(buffer, 10, &val);
	if (ret != 0) {
		pr_err("%s: invalid content: '%s', length = %zd\n", __func__,
				buffer, size);
		return ret;
	}

	cable_connect_state(val);
	return size;

}
static DEVICE_ATTR(state, 0644, state_show, state_store);

static struct device_attribute *sdx5x_rf_attributes[] = {
	&dev_attr_state,
	NULL
};
#endif

static int op_rf_cable_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device *dev = &pdev->dev;
	int cable_state = 0;
#ifdef SDX5X_RF_CABLE_UEVENT
	struct device_attribute **attrs;
	struct device_attribute *attr;
	int err;
#endif

	rf_cable_data = kzalloc(sizeof(struct cable_data), GFP_KERNEL);
	if (!rf_cable_data)
		goto exit;

	rf_cable_data->dev = dev;
	dev_set_drvdata(dev, rf_cable_data);

#ifdef SDX5X_RF_CABLE_UEVENT
	esoc_ssr_reason_feature_enable = of_property_read_bool(pdev->dev.of_node,
						"oem,sdx5x_uevent_feature_enable");
	if (esoc_ssr_reason_feature_enable == 1) {
		pr_err("SDX5X RF Cable init\n");
		sdx5x_rf_class = class_create(THIS_MODULE, "sdx5x_rf_cable");
		if (IS_ERR(sdx5x_rf_class)) {
			pr_err("%s: class_create fail - %d!\n", __func__,
					PTR_ERR(sdx5x_rf_class));
			return PTR_ERR(sdx5x_rf_class);
		}

		sdx5x_rf_device = device_create(sdx5x_rf_class, rf_cable_data->dev,
				MKDEV(0, 0), NULL, "rf_cable");
		if (IS_ERR(sdx5x_rf_device)) {
			pr_err("%s: sdx5x_rf_device fail - %d!\n", __func__,
					PTR_ERR(sdx5x_rf_device));
			return PTR_ERR(sdx5x_rf_device);
		}

		attrs = sdx5x_rf_attributes;
		while ((attr = *attrs++)) {
			err = device_create_file(sdx5x_rf_device, attr);
			if (err) {
				device_destroy(sdx5x_rf_device->class,
						sdx5x_rf_device->devt);
				return err;
			}
		}
	}
#endif

	if (cmdline_find_option("ftm_mode")) {
		pr_err("%s: ftm_mode FOUND! use 1 always\n", __func__);
		modify_rf_cable_smem_info(1);
		cable_connect_state(1);
	} else {

		rf_cable_gpio_pinctrl_init(pdev);

		//request gpio 0 .gpio 1.
		rc = op_rf_request_named_gpio("rf,cable-gpio-0",
		&rf_cable_data->cable_gpio_0);
		if (rc) {
			pr_err("%s: op_rf_request_named_gpio gpio-0 fail\n",
			__func__);
			goto exit_gpio;
		}
		rc = op_rf_request_named_gpio("rf,cable-gpio-1",
		&rf_cable_data->cable_gpio_1);
		if (rc) {
			pr_err("%s: op_rf_request_named_gpio gpio-1 fail\n",
			__func__);
			goto exit_gpio;
		}
		rc = of_property_read_u32(pdev->dev.of_node, "rf,cable-support-timer",
						&rf_cable_data->support_timer);
		if (rc) {
			pr_err("%s: op_rf_request_named_gpio gpio-1 fail\n",
			__func__);
			goto exit_gpio;
		}

		//creat workqueue.
		rf_cable_data->wqueue = create_singlethread_workqueue(
								"op_rf_cable_wqueue");
		INIT_DELAYED_WORK(&rf_cable_data->work, rf_cable_work);

		//set input  and gpio to irq.
		gpio_direction_input(rf_cable_data->cable_gpio_0);
		gpio_direction_input(rf_cable_data->cable_gpio_1);

		if (rf_cable_data->support_timer)
			queue_delayed_work(rf_cable_data->wqueue, &rf_cable_data->work, msecs_to_jiffies(HZ));
		else {
			rf_cable_data->irq_0 = gpio_to_irq(rf_cable_data->cable_gpio_0);
			if (rf_cable_data->irq_0 < 0) {
				pr_err("Unable to get irq number for GPIO %d, error %d\n",
				rf_cable_data->cable_gpio_0, rf_cable_data->irq_0);
				rc = rf_cable_data->irq_0;
				goto exit_gpio;
			}

			rf_cable_data->irq_1 = gpio_to_irq(rf_cable_data->cable_gpio_1);
			if (rf_cable_data->irq_1 < 0) {
				pr_err("Unable to get irq number for GPIO %d, error %d\n",
				rf_cable_data->cable_gpio_1, rf_cable_data->irq_1);
				rc = rf_cable_data->irq_1;
				goto exit_gpio;
			}

			//request _irq0
			rc = request_irq(rf_cable_data->irq_0, cable_interrupt,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"op_rf_cable", rf_cable_data);
			if (rc) {
				pr_err("could not request irq %d\n", rf_cable_data->irq_0);
				goto exit_gpio;
			}
			pr_err("requested irq %d\n", rf_cable_data->irq_0);
			enable_irq_wake(rf_cable_data->irq_0);

			//request _irq1
			rc = request_irq(rf_cable_data->irq_1, cable_interrupt,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"op_rf_cable", rf_cable_data);
			if (rc) {
				pr_err("could not request irq %d\n", rf_cable_data->irq_1);
				goto exit_gpio;
			}

			pr_err("requested irq %d\n", rf_cable_data->irq_1);
			enable_irq_wake(rf_cable_data->irq_1);

		}
		wakeup_source_init(&rf_cable_data->wl,
		"rf_cable_wake_lock");
		spin_lock_init(&rf_cable_data->lock);
		cable_state = gpio_get_value(rf_cable_data->cable_gpio_0) ||
					gpio_get_value(rf_cable_data->cable_gpio_1);
		atomic_set(&rf_cable_data->running, cable_state);

		modify_rf_cable_smem_info(cable_state);
		cable_connect_state(cable_state);

		pr_err("%s gpio0=%d gpio1=%d,\n", __func__, cable_state);
		create_rf_cable_procfs();
	}

	pr_err("%s: probe ok!\n", __func__);
	return 0;

exit_gpio:
    kfree(rf_cable_data);
exit:
    pr_err("%s: probe Fail!\n", __func__);

    return rc;
}

static const struct of_device_id rf_of_match[] = {
    { .compatible = "oem,rf_cable", },
    {}
};
MODULE_DEVICE_TABLE(of, rf_of_match);

static struct platform_driver op_rf_cable_driver = {
    .driver = {
        .name       = "op_rf_cable",
        .owner      = THIS_MODULE,
        .of_match_table = rf_of_match,
    },
    .probe = op_rf_cable_probe,
};

static int __init op_rf_cable_init(void)
{
    int ret;

    ret = platform_driver_register(&op_rf_cable_driver);
    if (ret)
        pr_err("rf_cable_driver register failed: %d\n", ret);

    return ret;
}

MODULE_LICENSE("GPL v2");
late_initcall(op_rf_cable_init);

