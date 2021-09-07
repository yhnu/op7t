
/*
 * drm_sysfs.c - Modifications to drm_sysfs_class.c to support
 *               extra sysfs attribute from DRM. Normal drm_sysfs_class
 *               does not allow adding attributes.
 *
 * Copyright (c) 2004 Jon Smirl <jonsmirl@gmail.com>
 * Copyright (c) 2003-2004 Greg Kroah-Hartman <greg@kroah.com>
 * Copyright (c) 2003-2004 IBM Corp.
 *
 * This file is released under the GPLv2
 *
 */

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/gfp.h>
#include <linux/err.h>
#include <linux/export.h>

#include <drm/drm_sysfs.h>
#include <drm/drmP.h>
#include "drm_internal.h"
#include <linux/list.h>
#include <linux/of.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/init.h>
#include <drm/drm_mipi_dsi.h>
#include <linux/input.h>
#include <linux/proc_fs.h>

#define to_drm_minor(d) dev_get_drvdata(d)
#define to_drm_connector(d) dev_get_drvdata(d)

#define DSI_PANEL_SAMSUNG_S6E3HC2 0
#define DSI_PANEL_SAMSUNG_S6E3FC2X01 1
#define DSI_PANEL_SAMSUNG_SOFEF03F_M 2
extern char gamma_para[2][413];
extern char dsi_panel_name;
/**
 * DOC: overview
 *
 * DRM provides very little additional support to drivers for sysfs
 * interactions, beyond just all the standard stuff. Drivers who want to expose
 * additional sysfs properties and property groups can attach them at either
 * &drm_device.dev or &drm_connector.kdev.
 *
 * Registration is automatically handled when calling drm_dev_register(), or
 * drm_connector_register() in case of hot-plugged connectors. Unregistration is
 * also automatically handled by drm_dev_unregister() and
 * drm_connector_unregister().
 */

static struct device_type drm_sysfs_device_minor = {
	.name = "drm_minor"
};

static struct input_dev *dc_mode_input_dev;
static struct proc_dir_entry *prEntry_dc;
static int    dc_mode_report_enable;

struct class *drm_class;

static char *drm_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "dri/%s", dev_name(dev));
}

static CLASS_ATTR_STRING(version, S_IRUGO, "drm 1.1.0 20060810");

static ssize_t dc_mode_event_num_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	const char *devname = NULL;
	struct input_handle *handle;

	if (!dc_mode_input_dev)
		return count;

	list_for_each_entry(handle, &(dc_mode_input_dev->h_list), d_node) {
		if (strncmp(handle->name, "event", 5) == 0) {
			devname = handle->name;
			break;
		}
	}
	ret = simple_read_from_buffer(user_buf, count, ppos, devname, strlen(devname));
	return ret;
}

static const struct file_operations dc_mode_event_num_fops = {
	.read  = dc_mode_event_num_read,
	.open  = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t dc_mode_report_enable_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char page[4];

	pr_info("the dc_mode_report_enable is: %d\n", dc_mode_report_enable);
	ret = snprintf(page, 4, "%d\n", dc_mode_report_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;

}

static ssize_t dc_mode_report_enable_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[8] = {0};

	if (count > 2)
		count = 2;

	if (copy_from_user(buf, buffer, count)) {
		pr_err("%s: read proc input error.\n", __func__);
		return count;
	}

	if ('0' == buf[0])
		dc_mode_report_enable = 0;
	else if ('1' == buf[0])
		dc_mode_report_enable = 1;


	return count;
}

static const struct file_operations dc_mode_report_enable_fops = {
	.read  = dc_mode_report_enable_read,
	.write = dc_mode_report_enable_write,
	.open  = simple_open,
	.owner = THIS_MODULE,
};

/**
 * drm_sysfs_init - initialize sysfs helpers
 *
 * This is used to create the DRM class, which is the implicit parent of any
 * other top-level DRM sysfs objects.
 *
 * You must call drm_sysfs_destroy() to release the allocated resources.
 *
 * Return: 0 on success, negative error code on failure.
 */
int drm_sysfs_init(void)
{
	int err;
	struct proc_dir_entry *prEntry_tmp  = NULL;

	drm_class = class_create(THIS_MODULE, "drm");
	if (IS_ERR(drm_class))
		return PTR_ERR(drm_class);

	err = class_create_file(drm_class, &class_attr_version.attr);
	if (err) {
		class_destroy(drm_class);
		drm_class = NULL;
		return err;
	}
	drm_class->devnode = drm_devnode;

	prEntry_dc = proc_mkdir("dc_for_sensor", NULL);
	if (prEntry_dc == NULL) {
		pr_err("Couldn't create dc_for_sensor directory\n");
		return 0;
	}

	//create dc_mode_event_num
	prEntry_tmp = proc_create("dc_mode_event_num", 0664,
							prEntry_dc, &dc_mode_event_num_fops);
	if (prEntry_tmp == NULL) {
		pr_err("Couldn't create dc_mode_event_num_fops\n");
		return 0;
	}

	//create dc_mode_report_enable
	prEntry_tmp = proc_create("dc_mode_report_enable", 0666,
							prEntry_dc, &dc_mode_report_enable_fops);
	if (prEntry_tmp == NULL) {
		pr_err("Couldn't create dc_mode_report_enable_fops\n");
		return 0;
	}

	//create input event
	dc_mode_input_dev  = input_allocate_device();
	if (dc_mode_input_dev == NULL) {
		pr_err("Failed to allocate dc mode input device\n");
		return 0;
	}

	dc_mode_input_dev->name = "oneplus,dc_mode";

	set_bit(EV_MSC,  dc_mode_input_dev->evbit);
	set_bit(MSC_RAW, dc_mode_input_dev->mscbit);

	if (input_register_device(dc_mode_input_dev)) {
		pr_err("%s: Failed to register dc mode input device\n", __func__);
		input_free_device(dc_mode_input_dev);
		return 0;
	}

	return 0;
}

/**
 * drm_sysfs_destroy - destroys DRM class
 *
 * Destroy the DRM device class.
 */
void drm_sysfs_destroy(void)
{
	input_unregister_device(dc_mode_input_dev);
	input_free_device(dc_mode_input_dev);

	if (IS_ERR_OR_NULL(drm_class))
		return;
	class_remove_file(drm_class, &class_attr_version.attr);
	class_destroy(drm_class);
	drm_class = NULL;
}

/*
 * Connector properties
 */
static ssize_t status_store(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(device);
	struct drm_device *dev = connector->dev;
	enum drm_connector_force old_force;
	int ret;

	ret = mutex_lock_interruptible(&dev->mode_config.mutex);
	if (ret)
		return ret;

	old_force = connector->force;

	if (sysfs_streq(buf, "detect"))
		connector->force = 0;
	else if (sysfs_streq(buf, "on"))
		connector->force = DRM_FORCE_ON;
	else if (sysfs_streq(buf, "on-digital"))
		connector->force = DRM_FORCE_ON_DIGITAL;
	else if (sysfs_streq(buf, "off"))
		connector->force = DRM_FORCE_OFF;
	else
		ret = -EINVAL;

	if (old_force != connector->force || !connector->force) {
		DRM_DEBUG_KMS("[CONNECTOR:%d:%s] force updated from %d to %d or reprobing\n",
			      connector->base.id,
			      connector->name,
			      old_force, connector->force);

		connector->funcs->fill_modes(connector,
					     dev->mode_config.max_width,
					     dev->mode_config.max_height);
	}

	mutex_unlock(&dev->mode_config.mutex);

	return ret ? ret : count;
}

static ssize_t status_show(struct device *device,
			   struct device_attribute *attr,
			   char *buf)
{
	struct drm_connector *connector = to_drm_connector(device);
	enum drm_connector_status status;

	status = READ_ONCE(connector->status);

	return snprintf(buf, PAGE_SIZE, "%s\n",
			drm_get_connector_status_name(status));
}

static ssize_t dpms_show(struct device *device,
			   struct device_attribute *attr,
			   char *buf)
{
	struct drm_connector *connector = to_drm_connector(device);
	int dpms;

	dpms = READ_ONCE(connector->dpms);

	return snprintf(buf, PAGE_SIZE, "%s\n",
			drm_get_dpms_name(dpms));
}

static ssize_t enabled_show(struct device *device,
			    struct device_attribute *attr,
			   char *buf)
{
	struct drm_connector *connector = to_drm_connector(device);
	bool enabled;

	enabled = READ_ONCE(connector->encoder);

	return snprintf(buf, PAGE_SIZE, enabled ? "enabled\n" : "disabled\n");
}

static ssize_t edid_show(struct file *filp, struct kobject *kobj,
			 struct bin_attribute *attr, char *buf, loff_t off,
			 size_t count)
{
	struct device *connector_dev = kobj_to_dev(kobj);
	struct drm_connector *connector = to_drm_connector(connector_dev);
	unsigned char *edid;
	size_t size;
	ssize_t ret = 0;

	mutex_lock(&connector->dev->mode_config.mutex);
	if (!connector->edid_blob_ptr)
		goto unlock;

	edid = connector->edid_blob_ptr->data;
	size = connector->edid_blob_ptr->length;
	if (!edid)
		goto unlock;

	if (off >= size)
		goto unlock;

	if (off + count > size)
		count = size - off;
	memcpy(buf, edid + off, count);

	ret = count;
unlock:
	mutex_unlock(&connector->dev->mode_config.mutex);

	return ret;
}

static ssize_t modes_show(struct device *device,
			   struct device_attribute *attr,
			   char *buf)
{
	struct drm_connector *connector = to_drm_connector(device);
	struct drm_display_mode *mode;
	int written = 0;

	mutex_lock(&connector->dev->mode_config.mutex);
	list_for_each_entry(mode, &connector->modes, head) {
		written += snprintf(buf + written, PAGE_SIZE - written, "%s\n",
				    mode->name);
	}
	mutex_unlock(&connector->dev->mode_config.mutex);

	return written;
}
static ssize_t acl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int acl_mode = 0;

	acl_mode = dsi_display_get_acl_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "acl mode = %d\n"
											"0--acl mode(off)\n"
											"1--acl mode(5)\n"
											"2--acl mode(10)\n"
											"3--acl mode(15)\n",
											acl_mode);
	return ret;
}

static ssize_t acl_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int acl_mode = 0;

	ret = kstrtoint(buf, 10, &acl_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_acl_mode(connector, acl_mode);
	if (ret)
		pr_err("set acl mode(%d) fail\n", acl_mode);

	return count;
}
static ssize_t hbm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int hbm_mode = 0;

	hbm_mode = dsi_display_get_hbm_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "hbm mode = %d\n"
											"0--hbm mode(off)\n"
											"1--hbm mode(XX)\n"
											"2--hbm mode(XX)\n"
											"3--hbm mode(XX)\n"
											"4--hbm mode(XX)\n"
											"5--hbm mode(670)\n",
											hbm_mode);
	return ret;
}

static ssize_t hbm_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int hbm_mode = 0;

	ret = kstrtoint(buf, 10, &hbm_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_hbm_mode(connector, hbm_mode);
	if (ret)
		pr_err("set hbm mode(%d) fail\n", hbm_mode);

	return count;
}

static ssize_t hbm_brightness_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int hbm_brightness = 0;

	hbm_brightness = dsi_display_get_hbm_brightness(connector);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", hbm_brightness);
	return ret;
}

static ssize_t hbm_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int hbm_brightness = 0;

	ret = kstrtoint(buf, 10, &hbm_brightness);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}
	ret = dsi_display_set_hbm_brightness(connector, hbm_brightness);
	if (ret)
		pr_err("set hbm brightness (%d) failed\n", hbm_brightness);
	return count;
}

static ssize_t op_friginer_print_hbm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int op_hbm_mode = 0;

	op_hbm_mode = dsi_display_get_fp_hbm_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "OP_FP mode = %d\n"
											"0--finger-hbm mode(off)\n"
											"1--finger-hbm mode(600)\n",
											op_hbm_mode);
	return ret;
}

static ssize_t op_friginer_print_hbm_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int op_hbm_mode = 0;

	ret = kstrtoint(buf, 10, &op_hbm_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_fp_hbm_mode(connector, op_hbm_mode);
	if (ret)
		pr_err("set hbm mode(%d) fail\n", op_hbm_mode);

	return count;
}

static ssize_t aod_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int aod_mode = 0;

	aod_mode = dsi_display_get_aod_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", aod_mode);
	return ret;
}

static ssize_t aod_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int aod_mode = 0;

	ret = kstrtoint(buf, 10, &aod_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}
	ret = dsi_display_set_aod_mode(connector, aod_mode);
	if (ret)
		pr_err("set AOD mode(%d) fail\n", aod_mode);
	return count;
}

static ssize_t aod_disable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int aod_disable = 0;

	aod_disable = dsi_display_get_aod_disable(connector);

	ret = scnprintf(buf, PAGE_SIZE, "AOD disable = %d\n"
											"0--AOD enable\n"
											"1--AOD disable\n",
											aod_disable);
	return ret;
}

static ssize_t aod_disable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int aod_disable = 0;

	ret = kstrtoint(buf, 10, &aod_disable);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_aod_disable(connector, aod_disable);
	if (ret)
		pr_err("set AOD disable(%d) fail\n", aod_disable);

	return count;
}

static ssize_t DCI_P3_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int dci_p3_mode = 0;

	dci_p3_mode = dsi_display_get_dci_p3_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "dci-p3 mode = %d\n"
											"0--dci-p3 mode Off\n"
											"1--dci-p3 mode On\n",
											dci_p3_mode);
	return ret;
}

static ssize_t DCI_P3_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int dci_p3_mode = 0;

	ret = kstrtoint(buf, 10, &dci_p3_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_dci_p3_mode(connector, dci_p3_mode);
	if (ret) {
		pr_err("set dci-p3 mode(%d) fail\n", dci_p3_mode);
	}
	return count;
}

static ssize_t night_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int night_mode = 0;

	night_mode = dsi_display_get_night_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "night mode = %d\n"
											"0--night mode Off\n"
											"1--night mode On\n",
											night_mode);
	return ret;
}

static ssize_t night_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int night_mode = 0;

	ret = kstrtoint(buf, 10, &night_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_night_mode(connector, night_mode);
	if (ret) {
		pr_err("set night mode(%d) fail\n", night_mode);
	}
	return count;
}

static ssize_t native_display_p3_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_p3_mode = 0;

	native_display_p3_mode = dsi_display_get_native_display_p3_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "native display p3 mode = %d\n"
											"0--native display p3 mode Off\n"
											"1--native display p3 mode On\n",
											native_display_p3_mode);
	return ret;
}

static ssize_t native_display_p3_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_p3_mode = 0;

	ret = kstrtoint(buf, 10, &native_display_p3_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_native_display_p3_mode(connector, native_display_p3_mode);
	if (ret) {
		pr_err("set native_display_p3  mode(%d) fail\n", native_display_p3_mode);
	}
	return count;
}
static ssize_t native_display_wide_color_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_wide_color_mode = 0;

	native_display_wide_color_mode = dsi_display_get_native_display_wide_color_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "native display wide color mode = %d\n"
											"0--native display wide color mode Off\n"
											"1--native display wide color mode On\n",
											native_display_wide_color_mode);
	return ret;
}

static ssize_t native_display_loading_effect_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_loading_effect_mode = 0;

	ret = kstrtoint(buf, 10, &native_display_loading_effect_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_native_loading_effect_mode(connector, native_display_loading_effect_mode);
	if (ret) {
		pr_err("set loading effect  mode(%d) fail\n", native_display_loading_effect_mode);
	}
	return count;
}

static ssize_t native_display_loading_effect_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_loading_effect_mode = 0;

	native_display_loading_effect_mode = dsi_display_get_native_display_loading_effect_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "native display loading effect mode = %d\n"
											"0--native display loading effect mode Off\n"
											"1--native display loading effect mode On\n",
											native_display_loading_effect_mode);
	return ret;
}

static ssize_t native_display_customer_p3_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_customer_p3_mode = 0;

	ret = kstrtoint(buf, 10, &native_display_customer_p3_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_customer_p3_mode(connector, native_display_customer_p3_mode);
	if (ret) {
		pr_err("set customer p3  mode(%d) fail\n", native_display_customer_p3_mode);
	}
	return count;
}

static ssize_t native_display_customer_p3_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_customer_p3_mode = 0;

	native_display_customer_p3_mode = dsi_display_get_customer_p3_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "native display customer p3 mode = %d\n"
											"0--native display customer p3 mode Off\n"
											"1--native display customer p3 mode On\n",
											native_display_customer_p3_mode);
	return ret;
}
static ssize_t native_display_customer_srgb_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_customer_srgb_mode = 0;

	ret = kstrtoint(buf, 10, &native_display_customer_srgb_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_customer_srgb_mode(connector, native_display_customer_srgb_mode);
	if (ret) {
		pr_err("set customer srgb  mode(%d) fail\n", native_display_customer_srgb_mode);
	}
	return count;
}

static ssize_t native_display_customer_srgb_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_customer_srgb_mode = 0;

	native_display_customer_srgb_mode = dsi_display_get_customer_srgb_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "native display customer srgb mode = %d\n"
											"0--native display customer srgb mode Off\n"
											"1--native display customer srgb mode On\n",
											native_display_customer_srgb_mode);
	return ret;
}


static ssize_t native_display_wide_color_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_wide_color_mode = 0;

	ret = kstrtoint(buf, 10, &native_display_wide_color_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_native_display_wide_color_mode(connector, native_display_wide_color_mode);
	if (ret) {
		pr_err("set native_display_p3  mode(%d) fail\n", native_display_wide_color_mode);
	}
	return count;
}

static ssize_t native_display_srgb_color_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_srgb_color_mode = 0;

	native_display_srgb_color_mode = dsi_display_get_native_display_srgb_color_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "native display srgb color mode = %d\n"
											"0--native display srgb color mode Off\n"
											"1--native display srgb color mode On\n",
											native_display_srgb_color_mode);
	return ret;
}

static ssize_t native_display_srgb_color_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_srgb_color_mode = 0;

	ret = kstrtoint(buf, 10, &native_display_srgb_color_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_native_display_srgb_color_mode(connector, native_display_srgb_color_mode);
	if (ret) {
		pr_err("set native_display_srgb  mode(%d) fail\n", native_display_srgb_color_mode);
	}
	return count;
}

static ssize_t gamma_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int gamma_test_flag = 0;
	int panel_stage_info = 0;
	int pvt_mp_panel_flag = 0;

	if (dsi_panel_name == DSI_PANEL_SAMSUNG_S6E3HC2) {
		if ((gamma_para[0][18] == 0xFF) && (gamma_para[0][19] == 0xFF) && (gamma_para[0][20] == 0xFF)) {
			gamma_test_flag = 0;
		}
		else {
			gamma_test_flag = 1;
		}

		panel_stage_info = dsi_display_get_stage_info(connector);
		if ((0x07 == panel_stage_info) || (0x10 == panel_stage_info) || (0x11 == panel_stage_info)) {
			pvt_mp_panel_flag = 1;
		}
		else {
			pvt_mp_panel_flag = 0;
		}

		ret = scnprintf(buf, PAGE_SIZE, "%d\n", (gamma_test_flag << 1) + pvt_mp_panel_flag);
		return ret;
	}
	else {
		ret = scnprintf(buf, PAGE_SIZE, "%d\n", 3);
		pr_err("It is not S6E3HC2 panel!\n");
		return ret;
	}
}

static ssize_t panel_serial_number_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int panel_year = 0;
	int panel_mon = 0;
	int panel_day = 0;
	int panel_hour = 0;
	int panel_min = 0;
	int panel_sec = 0;
	int panel_code_info = 0;
	int panel_stage_info = 0;
	int panel_production_info = 0;
	char * production_string_info = NULL;
	char * stage_string_info = NULL;
	int ret = 0;

	dsi_display_get_serial_number(connector);

	panel_year = dsi_display_get_serial_number_year(connector);
	panel_mon = dsi_display_get_serial_number_mon(connector);
	panel_day = dsi_display_get_serial_number_day(connector);
	panel_hour = dsi_display_get_serial_number_hour(connector);
	panel_min = dsi_display_get_serial_number_min(connector);
	panel_sec = dsi_display_get_serial_number_sec(connector);
	panel_code_info = dsi_display_get_code_info(connector);
	panel_stage_info = dsi_display_get_stage_info(connector);
	panel_production_info = dsi_display_get_production_info(connector);

	if (dsi_panel_name == DSI_PANEL_SAMSUNG_S6E3HC2) {
		if (panel_code_info == 0xED) {
			if (panel_stage_info == 0x02)
				stage_string_info = "STAGE: EVT2";
			else if (panel_stage_info == 0x03)
				stage_string_info = "STAGE: EVT2(NEW_DIMMING_SET)";
			else if (panel_stage_info == 0x99)
				stage_string_info = "STAGE: EVT2(113MHZ_OSC)";
			else if (panel_stage_info == 0x04)
				stage_string_info = "STAGE: DVT1";
			else if (panel_stage_info == 0x05)
				stage_string_info = "STAGE: DVT2";
			else if (panel_stage_info == 0x06)
				stage_string_info = "STAGE: DVT3";
			else if (panel_stage_info == 0x07)
				stage_string_info = "STAGE: PVT(112MHZ_OSC)";
			else if (panel_stage_info == 0x10)
				stage_string_info = "STAGE: PVT(113MHZ_OSC)";
			else if (panel_stage_info == 0x11)
				stage_string_info = "STAGE: PVT(113MHZ_OSC+X_TALK_IMPROVEMENT)";
			else
				stage_string_info = "STAGE: UNKNOWN";

			if (panel_production_info == 0x0C)
				production_string_info = "TPIC: LSI\nCOVER: JNTC\nOTP_GAMMA: 90HZ";
			else if (panel_production_info == 0x0E)
				production_string_info = "TPIC: LSI\nCOVER: LENS\nOTP_GAMMA: 90HZ";
			else if (panel_production_info == 0x1C)
				production_string_info = "TPIC: STM\nCOVER: JNTC\nOTP_GAMMA: 90HZ";
			else if (panel_production_info == 0x6C)
				production_string_info = "TPIC: LSI\nCOVER: JNTC\nOTP_GAMMA: 60HZ";
			else if (panel_production_info == 0x6E)
				production_string_info = "TPIC: LSI\nCOVER: LENS\nOTP_GAMMA: 60HZ";
			else if (panel_production_info == 0x1E)
				production_string_info = "TPIC: STM\nCOVER: LENS\nOTP_GAMMA: 90HZ";
			else if (panel_production_info == 0x0D)
				production_string_info = "TPIC: LSI\nID3: 0x0D\nOTP_GAMMA: 90HZ";
			else
				production_string_info = "TPIC: UNKNOWN\nCOVER: UNKNOWN\nOTP_GAMMA: UNKNOWN";

			ret = scnprintf(buf, PAGE_SIZE, "%04d/%02d/%02d %02d:%02d:%02d\n%s\n%s\nID: %02X %02X %02X\n",
					panel_year, panel_mon, panel_day, panel_hour, panel_min, panel_sec,
					stage_string_info, production_string_info, panel_code_info,
						panel_stage_info, panel_production_info);
		}

		if (panel_code_info == 0xEE) {
			if (panel_stage_info == 0x12)
				stage_string_info = "STAGE: T0/EVT1";
			else if (panel_stage_info == 0x13)
				stage_string_info = "STAGE: EVT2";
			else if (panel_stage_info == 0x14)
				stage_string_info = "STAGE: EVT2";
			else if (panel_stage_info == 0x15)
				stage_string_info = "STAGE: EVT3";
			else if (panel_stage_info == 0x16)
				stage_string_info = "STAGE: DVT";
			else if (panel_stage_info == 0x17)
				stage_string_info = "STAGE: DVT";
			else if (panel_stage_info == 0x19)
				stage_string_info = "STAGE: PVT";
			else
				stage_string_info = "STAGE: UNKNOWN";

			ret = scnprintf(buf, PAGE_SIZE, "%04d/%02d/%02d %02d:%02d:%02d\n%s\nID: %02X %02X %02X\n",
					panel_year, panel_mon, panel_day, panel_hour, panel_min, panel_sec,
					stage_string_info, production_string_info, panel_code_info,
						panel_stage_info, panel_production_info);
		}

	} else if (dsi_panel_name == DSI_PANEL_SAMSUNG_SOFEF03F_M) {
		if (panel_stage_info == 0x01)
			stage_string_info = "STAGE: T0";
		else if (panel_stage_info == 0x21)
			stage_string_info = "STAGE: EVT1";
		else if (panel_stage_info == 0x22)
			stage_string_info = "STAGE: EVT2";
		else if (panel_stage_info == 0x24)
			stage_string_info = "STAGE: DVT1-1";
		else if (panel_stage_info == 0x26)
			stage_string_info = "STAGE: DVT1-2";
		else if (panel_stage_info == 0x25)
			stage_string_info = "STAGE: DVT2";
		else if (panel_stage_info == 0x28)
			stage_string_info = "STAGE: DVT3";
		else if (panel_stage_info == 0x27)
			stage_string_info = "STAGE: PVT/MP";

		ret = scnprintf(buf, PAGE_SIZE, "%04d/%02d/%02d %02d:%02d:%02d\n%s\nID: %02X %02X %02X\n",
				panel_year, panel_mon, panel_day, panel_hour, panel_min, panel_sec, stage_string_info,
					panel_code_info, panel_stage_info, panel_production_info);
	}
	else {
		ret = scnprintf(buf, PAGE_SIZE, "%04d/%02d/%02d %02d:%02d:%02d\n",
				panel_year, panel_mon, panel_day, panel_hour, panel_min, panel_sec);
	}

	pr_err("panel year = %d, mon = %d, day = %d, hour = %d, min = %d\n",
		panel_year, panel_mon, panel_day, panel_hour, panel_min);

	return ret;
}

static ssize_t panel_serial_number_AT_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	int ret = 0;
	uint64_t serial_number = 0;

	ret = scnprintf(buf, PAGE_SIZE, "%llu\n",dsi_display_get_serial_number_id(serial_number));

	return ret;
}

static ssize_t dsi_on_command_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;

	ret = dsi_display_get_dsi_on_command(connector, buf);

	return ret;
}

static ssize_t dsi_on_command_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;

	ret = dsi_display_update_dsi_on_command(connector, buf, count);
	if (ret)
		pr_err("Failed to update dsi on command, ret=%d\n", ret);

	return count;
}

static ssize_t dsi_panel_command_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;

	ret = dsi_display_get_dsi_panel_command(connector, buf);

	return ret;
}

static ssize_t dsi_panel_command_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;

	ret = dsi_display_update_dsi_panel_command(connector, buf, count);
	if (ret)
		pr_err("Failed to update dsi panel command, ret=%d\n", ret);

	return count;
}

static ssize_t dsi_seed_command_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;

	ret = dsi_display_get_dsi_seed_command(connector, buf);

	return ret;
}

static ssize_t dsi_seed_command_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;

	ret = dsi_display_update_dsi_seed_command(connector, buf, count);
	if (ret)
		pr_err("Failed to update dsi seed command, ret=%d\n", ret);

	return count;
}

int current_freq = 0;
static ssize_t dynamic_dsitiming_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = scnprintf(buf, PAGE_SIZE, "current_freq = %d\n",
											current_freq);
	return ret;
}

static ssize_t dynamic_dsitiming_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int freq_value = 0;

	ret = kstrtoint(buf, 10, &freq_value);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

    current_freq = freq_value;

    pr_err("freq setting=%d\n", current_freq);

	if (ret) {
		pr_err("set dsi freq (%d) fail\n", current_freq);
	}
	return count;
}

extern u32 mode_fps;
static ssize_t dynamic_fps_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", mode_fps);

	return ret;
}

static ssize_t panel_mismatch_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int wrong_panel = 0;

	dsi_display_panel_mismatch_check(connector);

	wrong_panel = dsi_display_panel_mismatch(connector);
	ret = scnprintf(buf, PAGE_SIZE, "panel mismatch = %d\n"
										    "0--(panel match)\n"
											"1--(panel mismatch)\n",
											wrong_panel);
	return ret;
}

int oneplus_panel_alpha =0;
int oneplus_force_screenfp = 0;
int op_dimlayer_bl_enable = 0;
int op_dp_enable = 0;
int op_dither_enable = 0;
extern int oneplus_get_panel_brightness_to_alpha(void);

static ssize_t oneplus_display_get_dim_alpha(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", oneplus_get_panel_brightness_to_alpha());
}

static ssize_t oneplus_display_set_dim_alpha(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
	sscanf(buf, "%d", &oneplus_panel_alpha);
	return count;
}

static ssize_t oneplus_display_get_forcescreenfp(struct device *dev,
                                struct device_attribute *attr, char *buf)
{

	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	oneplus_force_screenfp = dsi_display_get_fp_hbm_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "OP_FP mode = %d\n"
											"0--finger-hbm mode(off)\n"
											"1--finger-hbm mode(600)\n",
											oneplus_force_screenfp);
	return sprintf(buf, "%d\n", oneplus_force_screenfp);
	
}

static ssize_t oneplus_display_set_forcescreenfp(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
	//sscanf(buf, "%x", &oneplus_force_screenfp);
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	ret = kstrtoint(buf, 10, &oneplus_force_screenfp);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_fp_hbm_mode(connector, oneplus_force_screenfp);
	if (ret)
		pr_err("set hbm mode(%d) fail\n", oneplus_force_screenfp);
	return count;
}


static ssize_t op_display_get_dimlayer_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", op_dimlayer_bl_enable);
}

static ssize_t op_display_set_dimlayer_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	sscanf(buf, "%d", &op_dimlayer_bl_enable);

	pr_err("op_dimlayer_bl_enable : %d\n", op_dimlayer_bl_enable);

	if (dc_mode_report_enable) {
		input_event(dc_mode_input_dev, EV_MSC, MSC_RAW, op_dimlayer_bl_enable);
		input_sync(dc_mode_input_dev);
	}

	return count;
}

static ssize_t op_display_get_dither_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", op_dither_enable);
}

static ssize_t op_display_set_dither_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	sscanf(buf, "%d", &op_dither_enable);

	return count;
}

static ssize_t op_display_get_dp_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", op_dp_enable);
}

static ssize_t op_display_set_dp_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	sscanf(buf, "%d", &op_dp_enable);

	return count;
}

extern  ssize_t oneplus_display_notify_fp_press(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);

extern  ssize_t oneplus_display_notify_dim(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);

extern  ssize_t oneplus_display_notify_aod_hid(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);

static DEVICE_ATTR_RW(status);
static DEVICE_ATTR_RO(enabled);
static DEVICE_ATTR_RO(dpms);
static DEVICE_ATTR_RO(modes);
static DEVICE_ATTR_RW(acl);
static DEVICE_ATTR_RW(hbm);
static DEVICE_ATTR_RW(hbm_brightness);
static DEVICE_ATTR_RW(op_friginer_print_hbm);
static DEVICE_ATTR_RW(aod);
static DEVICE_ATTR_RW(aod_disable);
static DEVICE_ATTR_RW(DCI_P3);
static DEVICE_ATTR_RW(night_mode);
static DEVICE_ATTR_RW(native_display_p3_mode);
static DEVICE_ATTR_RW(native_display_wide_color_mode);
static DEVICE_ATTR_RW(native_display_loading_effect_mode);
static DEVICE_ATTR_RW(native_display_srgb_color_mode);
static DEVICE_ATTR_RW(native_display_customer_p3_mode);
static DEVICE_ATTR_RW(native_display_customer_srgb_mode);
static DEVICE_ATTR_RO(gamma_test);
static DEVICE_ATTR_RO(panel_serial_number);
static DEVICE_ATTR_RO(panel_serial_number_AT);
static DEVICE_ATTR_RW(dsi_on_command);
static DEVICE_ATTR_RW(dsi_panel_command);
static DEVICE_ATTR_RW(dsi_seed_command);
static DEVICE_ATTR_RW(dynamic_dsitiming);
static DEVICE_ATTR_RO(panel_mismatch);
static DEVICE_ATTR_RO(dynamic_fps);
static DEVICE_ATTR(dim_alpha, S_IRUGO|S_IWUSR, oneplus_display_get_dim_alpha, oneplus_display_set_dim_alpha);
static DEVICE_ATTR(force_screenfp, S_IRUGO|S_IWUSR, oneplus_display_get_forcescreenfp, oneplus_display_set_forcescreenfp);
static DEVICE_ATTR(notify_fppress, S_IRUGO|S_IWUSR, NULL, oneplus_display_notify_fp_press);
static DEVICE_ATTR(notify_dim, S_IRUGO|S_IWUSR, NULL, oneplus_display_notify_dim);
static DEVICE_ATTR(notify_aod, S_IRUGO|S_IWUSR, NULL, oneplus_display_notify_aod_hid);
static DEVICE_ATTR(dimlayer_bl_en, S_IRUGO|S_IWUSR, op_display_get_dimlayer_enable, op_display_set_dimlayer_enable);
static DEVICE_ATTR(dp_en, S_IRUGO|S_IWUSR, op_display_get_dp_enable, op_display_set_dp_enable);
static DEVICE_ATTR(dither_en, S_IRUGO|S_IWUSR, op_display_get_dither_enable, op_display_set_dither_enable);

static struct attribute *connector_dev_attrs[] = {
	&dev_attr_status.attr,
	&dev_attr_enabled.attr,
	&dev_attr_dpms.attr,
	&dev_attr_modes.attr,
	&dev_attr_acl.attr,
	&dev_attr_hbm.attr,
	&dev_attr_hbm_brightness.attr,
	&dev_attr_op_friginer_print_hbm.attr,
	&dev_attr_aod.attr,
	&dev_attr_aod_disable.attr,
	&dev_attr_DCI_P3.attr,
	&dev_attr_night_mode.attr,
	&dev_attr_native_display_p3_mode.attr,
	&dev_attr_native_display_wide_color_mode.attr,
	&dev_attr_native_display_loading_effect_mode.attr,
	&dev_attr_native_display_srgb_color_mode.attr,
	&dev_attr_native_display_customer_p3_mode.attr,
	&dev_attr_native_display_customer_srgb_mode.attr,
	&dev_attr_gamma_test.attr,
	&dev_attr_panel_serial_number.attr,
	&dev_attr_panel_serial_number_AT.attr,
	&dev_attr_dsi_on_command.attr,
	&dev_attr_dsi_panel_command.attr,
	&dev_attr_dsi_seed_command.attr,
	&dev_attr_dynamic_dsitiming.attr,
	&dev_attr_panel_mismatch.attr,
	&dev_attr_force_screenfp.attr,
	&dev_attr_dim_alpha.attr,
	&dev_attr_dynamic_fps.attr,
	&dev_attr_notify_fppress.attr,
	&dev_attr_notify_dim.attr,
	&dev_attr_notify_aod.attr,
	&dev_attr_dimlayer_bl_en.attr,
	&dev_attr_dp_en.attr,
	&dev_attr_dither_en.attr,
	NULL
};

static struct bin_attribute edid_attr = {
	.attr.name = "edid",
	.attr.mode = 0444,
	.size = 0,
	.read = edid_show,
};

static struct bin_attribute *connector_bin_attrs[] = {
	&edid_attr,
	NULL
};

static const struct attribute_group connector_dev_group = {
	.attrs = connector_dev_attrs,
	.bin_attrs = connector_bin_attrs,
};

static const struct attribute_group *connector_dev_groups[] = {
	&connector_dev_group,
	NULL
};

int drm_sysfs_connector_add(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;

	if (connector->kdev)
		return 0;

	connector->kdev =
		device_create_with_groups(drm_class, dev->primary->kdev, 0,
					  connector, connector_dev_groups,
					  "card%d-%s", dev->primary->index,
					  connector->name);
	DRM_DEBUG("adding \"%s\" to sysfs\n",
		  connector->name);

	if (IS_ERR(connector->kdev)) {
		DRM_ERROR("failed to register connector device: %ld\n", PTR_ERR(connector->kdev));
		return PTR_ERR(connector->kdev);
	}

	/* Let userspace know we have a new connector */
	drm_sysfs_hotplug_event(dev);

	return 0;
}

void drm_sysfs_connector_remove(struct drm_connector *connector)
{
	if (!connector->kdev)
		return;
	DRM_DEBUG("removing \"%s\" from sysfs\n",
		  connector->name);

	device_unregister(connector->kdev);
	connector->kdev = NULL;
}

/**
 * drm_sysfs_hotplug_event - generate a DRM uevent
 * @dev: DRM device
 *
 * Send a uevent for the DRM device specified by @dev.  Currently we only
 * set HOTPLUG=1 in the uevent environment, but this could be expanded to
 * deal with other types of events.
 */
void drm_sysfs_hotplug_event(struct drm_device *dev)
{
	char *event_string = "HOTPLUG=1";
	char *envp[] = { event_string, NULL };

	DRM_DEBUG("generating hotplug event\n");

	kobject_uevent_env(&dev->primary->kdev->kobj, KOBJ_CHANGE, envp);
}
EXPORT_SYMBOL(drm_sysfs_hotplug_event);

static void drm_sysfs_release(struct device *dev)
{
	kfree(dev);
}

struct device *drm_sysfs_minor_alloc(struct drm_minor *minor)
{
	const char *minor_str;
	struct device *kdev;
	int r;

	if (minor->type == DRM_MINOR_CONTROL)
		minor_str = "controlD%d";
	else if (minor->type == DRM_MINOR_RENDER)
		minor_str = "renderD%d";
	else
		minor_str = "card%d";

	kdev = kzalloc(sizeof(*kdev), GFP_KERNEL);
	if (!kdev)
		return ERR_PTR(-ENOMEM);

	device_initialize(kdev);
	kdev->devt = MKDEV(DRM_MAJOR, minor->index);
	kdev->class = drm_class;
	kdev->type = &drm_sysfs_device_minor;
	kdev->parent = minor->dev->dev;
	kdev->release = drm_sysfs_release;
	dev_set_drvdata(kdev, minor);

	r = dev_set_name(kdev, minor_str, minor->index);
	if (r < 0)
		goto err_free;

	return kdev;

err_free:
	put_device(kdev);
	return ERR_PTR(r);
}

/**
 * drm_class_device_register - register new device with the DRM sysfs class
 * @dev: device to register
 *
 * Registers a new &struct device within the DRM sysfs class. Essentially only
 * used by ttm to have a place for its global settings. Drivers should never use
 * this.
 */
int drm_class_device_register(struct device *dev)
{
	if (!drm_class || IS_ERR(drm_class))
		return -ENOENT;

	dev->class = drm_class;
	return device_register(dev);
}
EXPORT_SYMBOL_GPL(drm_class_device_register);

/**
 * drm_class_device_unregister - unregister device with the DRM sysfs class
 * @dev: device to unregister
 *
 * Unregisters a &struct device from the DRM sysfs class. Essentially only used
 * by ttm to have a place for its global settings. Drivers should never use
 * this.
 */
void drm_class_device_unregister(struct device *dev)
{
	return device_unregister(dev);
}
EXPORT_SYMBOL_GPL(drm_class_device_unregister);
