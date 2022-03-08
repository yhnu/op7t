#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include "synaptics_touch_panel_remote.h"

#define CHAR_DEVICE_NAME "rmi"
#define DEVICE_CLASS_NAME "rmidev"
#define DEV_NUMBER 1
#define REG_ADDR_LIMIT 0xFFFF

static ssize_t rmidev_sysfs_data_show(struct file *data_file,
        struct kobject *kobj, struct bin_attribute *attributes,
        char *buf, loff_t pos, size_t count);

static ssize_t rmidev_sysfs_data_store(struct file *data_file,
        struct kobject *kobj, struct bin_attribute *attributes,
        char *buf, loff_t pos, size_t count);

static ssize_t rmidev_sysfs_open_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_release_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_address_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_length_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_attn_state_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static int remote_rmi4_i2c_read(unsigned short addr, unsigned char *data, unsigned short length);
static int remote_rmi4_i2c_write(unsigned short addr, unsigned char *data, unsigned short length);
static int remote_rmi4_i2c_enable(bool enable);
static int remote_rmi4_get_irq_gpio(void);
static int remote_rmit_set_page(unsigned int address);
static int remote_rmit_put_page(void);

static struct input_dev *remote_rmi4_get_input(void);
static struct i2c_client *remote_rmi4_get_i2c_client(void);
static void remote_rmi4_delay_work(struct work_struct *work);
static struct remotepanel_data *remote_free_panel_data(struct remotepanel_data *pdata);

#define MASK_8BIT 0xFF ;
#define SYN_I2C_RETRY_TIMES 3;
#define BUFFER_SIZE 252
struct rmidev_handle {
    dev_t dev_no;
    unsigned short address;
    unsigned int length;
    struct device dev;
    struct kobject *sysfs_dir;
    void *data;
};

struct rmidev_data {
    int ref_count;
    struct cdev main_dev;
    struct class *device_class;
    struct mutex file_mutex;
    struct rmidev_handle *rmi_dev;
    struct remotepanel_data *pdata;
};

static struct bin_attribute attr_data = {
    .attr = {
        .name = "data",
        .mode = (S_IRUSR | S_IWUSR),
    },
    .size = 0,
    .read = rmidev_sysfs_data_show,
    .write = rmidev_sysfs_data_store,
};

static struct device_attribute attrs[] = {
    __ATTR(open, S_IRUSR | S_IWUSR,
            NULL,
            rmidev_sysfs_open_store),
    __ATTR(release, S_IRUSR | S_IWUSR,
            NULL,
            rmidev_sysfs_release_store),
    __ATTR(address, S_IRUSR | S_IWUSR,
            NULL,
            rmidev_sysfs_address_store),
    __ATTR(length, S_IRUSR | S_IWUSR,
            NULL,
            rmidev_sysfs_length_store),
    __ATTR(attn_state, S_IRUSR | S_IWUSR,
            rmidev_sysfs_attn_state_show,
            NULL),
};

static int rmidev_major_num;

static struct class *rmidev_device_class;

static struct rmidev_handle *rmidev;

static struct device *device_ptr;
static struct delayed_work delay_work;


static ssize_t rmidev_sysfs_data_show(struct file *data_file,
        struct kobject *kobj, struct bin_attribute *attributes,
        char *buf, loff_t pos, size_t count)
{
    int retval;
    unsigned int data_length = rmidev->length;

    if (data_length > (REG_ADDR_LIMIT - rmidev->address))
        data_length = REG_ADDR_LIMIT - rmidev->address;

    if (count < data_length) {
        dev_err(device_ptr,
                "%s: Not enough space (%zd bytes) in buffer\n",
                __func__, count);
        return -EINVAL;
    }

    if (data_length) {
        retval = remote_rmi4_i2c_read(
                rmidev->address,
                (unsigned char *)buf,
                data_length);
        if (retval < 0) {
            dev_err(device_ptr,
                    "%s: Failed to read data\n",
                    __func__);
            return retval;
        }
    } else {
        return -EINVAL;
    }

    return data_length;
}

static ssize_t rmidev_sysfs_data_store(struct file *data_file,
        struct kobject *kobj, struct bin_attribute *attributes,
        char *buf, loff_t pos, size_t count)
{
    int retval;
    unsigned int data_length = rmidev->length;

    if (data_length > (REG_ADDR_LIMIT - rmidev->address))
        data_length = REG_ADDR_LIMIT - rmidev->address;

    if (data_length) {
        retval = remote_rmi4_i2c_write(
                rmidev->address,
                (unsigned char *)buf,
                data_length);
        if (retval < 0) {
            dev_err(device_ptr,
                    "%s: Failed to write data\n",
                    __func__);
            return retval;
        }
    } else {
        return -EINVAL;
    }

    return count;
}

static ssize_t rmidev_sysfs_open_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int input;

    if (sscanf(buf, "%u", &input) != 1)
        return -EINVAL;

    if (input != 1)
        return -EINVAL;

    remote_rmi4_i2c_enable(false);
    dev_dbg(device_ptr,
            "%s: Attention interrupt disabled\n",
            __func__);

    return count;
}

static ssize_t rmidev_sysfs_release_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int input;

    if (sscanf(buf, "%u", &input) != 1)
        return -EINVAL;

    if (input != 1)
        return -EINVAL;

    remote_rmi4_i2c_enable(true);
    dev_dbg(device_ptr,
            "%s: Attention interrupt enabled\n",
            __func__);

    return count;
}

static ssize_t rmidev_sysfs_address_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int input;

    if (sscanf(buf, "%u", &input) != 1)
        return -EINVAL;

    if (input > REG_ADDR_LIMIT)
        return -EINVAL;

    rmidev->address = (unsigned short)input;

    return count;
}

static ssize_t rmidev_sysfs_length_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int input;

    if (sscanf(buf, "%u", &input) != 1)
        return -EINVAL;

    if (input > REG_ADDR_LIMIT)
        return -EINVAL;

    rmidev->length = input;

    return count;
}

static ssize_t rmidev_sysfs_attn_state_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int attn_state;

    attn_state = gpio_get_value(remote_rmi4_get_irq_gpio());

    return snprintf(buf, PAGE_SIZE, "%d\n", attn_state);
}

static int remote_rmi4_get_irq_gpio(void)
{
    struct rmidev_data *dev_data = (struct rmidev_data *)rmidev->data;
    return dev_data->pdata->irq_gpio;
}

static struct input_dev *remote_rmi4_get_input(void)
{
    struct rmidev_data *dev_data = (struct rmidev_data *)rmidev->data;
    return dev_data->pdata->input_dev;
}

static struct i2c_client* remote_rmi4_get_i2c_client(void)
{
    struct rmidev_data *dev_data = (struct rmidev_data *)rmidev->data;
    return dev_data->pdata->client;
}

static int remote_rmit_set_page(unsigned int address) {
    struct i2c_client* i2c_client = remote_rmi4_get_i2c_client();
    unsigned char retry;
    //unsigned char buf[2];
	unsigned char *buf = kzalloc(2, GFP_KERNEL | GFP_DMA);
    struct i2c_msg msg[] = {
        {
            .addr = i2c_client->addr,
            .flags = 0,
            .len = 2,
            .buf = buf,
        }
    };
    buf[0] = 0xff;
    buf[1] = ((address >> 8) & 0xFF);

    for (retry = 0; retry < 2; retry++) {
        if (i2c_transfer(i2c_client->adapter, msg, 1) == 1) {
            break;
        }
        msleep(20);
    }

    if (retry == 2) {
	kfree(buf);
        return -EIO;
    }
	kfree(buf);
    return 0;
}

static int remote_rmit_put_page(void)
{
    struct i2c_client* i2c_client = remote_rmi4_get_i2c_client();
    unsigned char retry;
   // unsigned char buf[2];
	unsigned char *buf = kzalloc(2, GFP_KERNEL | GFP_DMA);
    struct i2c_msg msg[] = {
        {
            .addr = i2c_client->addr,
            .flags = 0,
            .len = 2,
            .buf = buf,
        }
    };
    buf[0] = 0xff;
    buf[1] = 0x00;

    for (retry = 0; retry < 2; retry++) {
        if (i2c_transfer(i2c_client->adapter, msg, 1) == 1) {
            break;
        }
        msleep(20);
    }

    if (retry == 2) {
	kfree(buf);
        return -EIO;
    }
	kfree(buf);
    return 0;
}

int remote_rmi4_i2c_read(unsigned short addr, unsigned char *data, unsigned short length)
{
    int retval;
    unsigned char retry;
    //unsigned char buf;
	unsigned char *buf = kzalloc(1, GFP_KERNEL | GFP_DMA);
    struct i2c_client* i2c_client = remote_rmi4_get_i2c_client();
    struct i2c_msg msg[] = {
        {
            .addr = i2c_client->addr,
            .flags = 0,
            .len = 1,
            .buf = buf,
        },
        {
            .addr = i2c_client->addr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = data,
        },
    };

    buf[0] = addr & 0xff;

    retval = remote_rmit_set_page(addr);
    if (retval < 0)
        goto exit;

    for (retry = 0; retry < 2; retry++) {
        if (i2c_transfer(i2c_client->adapter, msg, 2) == 2) {
            retval = length;
            break;
        }
        msleep(20);
    }

    if (retry == 2) {
        retval = -EIO;
    }

exit:
    remote_rmit_put_page();
	kfree(buf);
    return retval;
}

int remote_rmi4_i2c_write(unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
   // unsigned char buf[length + 1];
	unsigned char *buf = kzalloc((length + 1) * sizeof(char), GFP_KERNEL | GFP_DMA);
	struct i2c_client *i2c_client = remote_rmi4_get_i2c_client();
	struct i2c_msg msg[] = {
		{
		.addr = i2c_client->addr,
		.flags = 0,
		.len = length + 1,
		.buf = buf,
		}
	};

	retval = remote_rmit_set_page(addr);
	if (retval < 0)
		goto exit;

	buf[0] = addr & 0xff;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < 2; retry++) {
		if (i2c_transfer(i2c_client->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		msleep(20);
	}
	msleep(20);
	if (retry == 2)
		retval = -EIO;

exit:
	remote_rmit_put_page();
	kfree(buf);
	return retval;
}

int remote_rmi4_i2c_enable(bool enable)
{
    struct rmidev_data *dev_data = (struct rmidev_data *)rmidev->data;

    if (enable) {
        *(dev_data->pdata->enable_remote) = 0;
    }else{
        *(dev_data->pdata->enable_remote) = 1;
    }
    return 0 ;
}


/*
 * rmidev_llseek - used to set up register address
 *
 * @filp: file structure for seek
 * @off: offset
 *   if whence == SEEK_SET,
 *     high 16 bits: page address
 *     low 16 bits: register address
 *   if whence == SEEK_CUR,
 *     offset from current position
 *   if whence == SEEK_END,
 *     offset from end position (0xFFFF)
 * @whence: SEEK_SET, SEEK_CUR, or SEEK_END
 */
static loff_t rmidev_llseek(struct file *filp, loff_t off, int whence)
{
    loff_t newpos;
    struct rmidev_data *dev_data = filp->private_data;

    if (IS_ERR(dev_data)) {
        pr_err("%s: Pointer of char device data is invalid", __func__);
        return -EBADF;
    }


    mutex_lock(&(dev_data->file_mutex));

    switch (whence) {
        case SEEK_SET:
            newpos = off;
            break;
        case SEEK_CUR:
            newpos = filp->f_pos + off;
            break;
        case SEEK_END:
            newpos = REG_ADDR_LIMIT + off;
            break;
        default:
            newpos = -EINVAL;
            goto clean_up;
    }

    if (newpos < 0 || newpos > REG_ADDR_LIMIT) {
        dev_err(device_ptr,
                "%s: New position 0x%04x is invalid\n",
                __func__, (unsigned int)newpos);
        newpos = -EINVAL;
        goto clean_up;
    }

    filp->f_pos = newpos;

clean_up:
    mutex_unlock(&(dev_data->file_mutex));

    return newpos;
}

/*
 * rmidev_read: - use to read data from rmi device
 *
 * @filp: file structure for read
 * @buf: user space buffer pointer
 * @count: number of bytes to read
 * @f_pos: offset (starting register address)
 */
static ssize_t rmidev_read(struct file *filp, char __user *buf,
        size_t count, loff_t *f_pos)
{
    ssize_t retval;
//unsigned char tmpbuf[count + 1];
	unsigned char *tmpbuf;
    struct rmidev_data *dev_data = filp->private_data;

	tmpbuf = kzalloc((count + 1) * sizeof(char), GFP_KERNEL);
    if (IS_ERR(dev_data)) {
        pr_err("%s: Pointer of char device data is invalid", __func__);
	kfree(tmpbuf);
        return -EBADF;
    }

    if (count == 0) {
	kfree(tmpbuf);
        return 0;
	}

    if (count > (REG_ADDR_LIMIT - *f_pos))
        count = REG_ADDR_LIMIT - *f_pos;

    mutex_lock(dev_data->pdata->pmutex);
    mutex_lock(&(dev_data->file_mutex));

    retval = remote_rmi4_i2c_read(
            *f_pos,
            tmpbuf,
            count);
    if (retval < 0)
        goto clean_up;

    if (copy_to_user(buf, tmpbuf, count))
        retval = -EFAULT;
    else
        *f_pos += retval;

clean_up:
    mutex_unlock(&(dev_data->file_mutex));
    mutex_unlock(dev_data->pdata->pmutex);

	kfree(tmpbuf);
    return retval;
}

/*
 * rmidev_write: - used to write data to rmi device
 *
 * @filep: file structure for write
 * @buf: user space buffer pointer
 * @count: number of bytes to write
 * @f_pos: offset (starting register address)
 */
static ssize_t rmidev_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *f_pos)
{
    ssize_t retval;
//    unsigned char tmpbuf[count + 1];
	unsigned char *tmpbuf;
    struct rmidev_data *dev_data = filp->private_data;

	tmpbuf = kzalloc((count + 1) * sizeof(char), GFP_KERNEL);
    if (IS_ERR(dev_data)) {
        pr_err("%s: Pointer of char device data is invalid", __func__);
	kfree(tmpbuf);
        return -EBADF;
    }

    if (count == 0) {
	kfree(tmpbuf);
        return 0;
	}

    if (count > (REG_ADDR_LIMIT - *f_pos))
        count = REG_ADDR_LIMIT - *f_pos;

    if (copy_from_user(tmpbuf, buf, count)) {
	kfree(tmpbuf);
        return -EFAULT;
	}
    mutex_lock(dev_data->pdata->pmutex);
    mutex_lock(&(dev_data->file_mutex));

    retval = remote_rmi4_i2c_write(
            *f_pos,
            tmpbuf,
            count);
    if (retval >= 0)
        *f_pos += retval;


    mutex_unlock(&(dev_data->file_mutex));
    mutex_unlock(dev_data->pdata->pmutex);
	kfree(tmpbuf);

    return retval;
}

static int rmidev_create_attr(bool create) {
    int retval = 0;
    unsigned char attr_count;
    struct input_dev *input_dev = remote_rmi4_get_input();

    if (!create)
        goto err_sysfs_attrs ;

    if (rmidev->sysfs_dir)
        return 0 ;

    if (!input_dev)
        return -1;
    /*
       retval = gpio_export(remote_rmi4_get_irq_gpio(), false);
       if (retval < 0) {
       dev_err(device_ptr,
       "%s: Failed to export attention gpio\n",
       __func__);
       } else {
       retval = gpio_export_link(&(input_dev->dev),
       "attn", remote_rmi4_get_irq_gpio());
       if (retval < 0) {
       dev_err(device_ptr,
       "%s Failed to create gpio symlink\n",
       __func__);
       }
       }
     */
    rmidev->sysfs_dir = kobject_create_and_add("rmidev",
            &input_dev->dev.kobj);
    if (!rmidev->sysfs_dir) {
        dev_err(device_ptr,
                "%s: Failed to create sysfs directory\n", __func__);
        return -1;
    }

    retval = sysfs_create_bin_file(rmidev->sysfs_dir,
            &attr_data);
    if (retval < 0) {
        dev_err(device_ptr,
                "%s: Failed to create sysfs bin file\n",
                __func__);
        goto err_sysfs_bin;
    }

    for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
        retval = sysfs_create_file(rmidev->sysfs_dir, &attrs[attr_count].attr);
        if (retval < 0) {
            dev_err(device_ptr,
                    "%s: Failed to create sysfs attributes\n", __func__);
            retval = -ENODEV;
            goto err_sysfs_attrs;
        }
    }

    return 0 ;

err_sysfs_attrs:
    if (!rmidev->sysfs_dir)
        return 0 ;
    for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
        sysfs_remove_file(rmidev->sysfs_dir, &attrs[attr_count].attr);
    }

    sysfs_remove_bin_file(rmidev->sysfs_dir, &attr_data);

err_sysfs_bin:
    kobject_put(rmidev->sysfs_dir);
    rmidev->sysfs_dir = NULL;

    return retval;
}

/*
 * rmidev_open: enable access to rmi device
 * @inp: inode struture
 * @filp: file structure
 */
static int rmidev_open(struct inode *inp, struct file *filp)
{
    int retval = 0;
    struct rmidev_data *dev_data =
        container_of(inp->i_cdev, struct rmidev_data, main_dev);

    rmidev_create_attr(true);

    filp->private_data = dev_data;

    mutex_lock(&(dev_data->file_mutex));
    *(dev_data->pdata->enable_remote) = 1;
    //remote_rmi4_i2c_enable(false);
    dev_dbg(device_ptr,
            "%s: Attention interrupt disabled\n", __func__);
    disable_irq_nosync(dev_data->pdata->irq);

    if (dev_data->ref_count < 1)
        dev_data->ref_count++;
    else
        retval = -EACCES;

    mutex_unlock(&(dev_data->file_mutex));

    return retval;
}

/*
 * rmidev_release: - release access to rmi device
 * @inp: inode structure
 * @filp: file structure
 */
static int rmidev_release(struct inode *inp, struct file *filp)
{
    struct rmidev_data *dev_data =
        container_of(inp->i_cdev, struct rmidev_data, main_dev);

    rmidev_create_attr(false);

    mutex_lock(&(dev_data->file_mutex));

    dev_data->ref_count--;
    if (dev_data->ref_count < 0)
        dev_data->ref_count = 0;

    remote_rmi4_i2c_enable(true);
    dev_dbg(device_ptr,
            "%s: Attention interrupt enabled\n", __func__);
    enable_irq(dev_data->pdata->irq);
    mutex_unlock(&(dev_data->file_mutex));

    return 0;
}

static const struct file_operations rmidev_fops = {
    .owner = THIS_MODULE,
    .llseek = rmidev_llseek,
    .read = rmidev_read,
    .write = rmidev_write,
    .open = rmidev_open,
    .release = rmidev_release,
};

static void rmidev_device_cleanup(struct rmidev_data *dev_data)
{
    dev_t devno;

    if (dev_data) {
        devno = dev_data->main_dev.dev;

        if (dev_data->device_class)
            device_destroy(dev_data->device_class, devno);

        cdev_del(&dev_data->main_dev);
        unregister_chrdev_region(devno, 1);
        remote_free_panel_data(dev_data->pdata);

        dev_dbg(device_ptr,
                "%s: rmidev device removed\n",  __func__);
    }

    return;
}

static char *rmi_char_devnode(struct device *dev, umode_t *mode)
{
    if (!mode)
        return NULL;

    *mode = (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);

    return kasprintf(GFP_KERNEL, "rmi/%s", dev_name(dev));
}

static int rmidev_create_device_class(void)
{
    rmidev_device_class = class_create(THIS_MODULE, DEVICE_CLASS_NAME);

    if (IS_ERR(rmidev_device_class)) {
        pr_err("%s: Failed to create /dev/%s\n",
                __func__, CHAR_DEVICE_NAME);
        return -ENODEV;
    }

    rmidev_device_class->devnode = rmi_char_devnode;

    return 0;
}

static void remote_rmi4_delay_work(struct work_struct *work) {
    rmidev_create_attr(true) ;
}

struct remotepanel_data *remote_alloc_panel_data(void)
{
    if (rmidev)
    {
        pr_err("%s:remote panel data has alloc already null\n", __func__);
        return NULL;
    }

    return kzalloc(sizeof(struct remotepanel_data), GFP_KERNEL);
}

static struct remotepanel_data *remote_free_panel_data(struct remotepanel_data *pdata)
{
    if (pdata)
        kfree(pdata);
    pdata = NULL;
    return NULL;
}

//int rmidev_init_device(void)
int register_remote_device(struct remotepanel_data *pdata)
{
    int retval;
    dev_t dev_no;
    struct rmidev_data *dev_data = NULL;


    if (pdata == NULL)
    {
        pr_err("%s:pdata is null\n", __func__);
        return -1;
    }
    if (rmidev)
    {
        pr_err("%s:remote device has register already null\n", __func__);
        return -1;
    }
    rmidev = kzalloc(sizeof(*rmidev), GFP_KERNEL);
    if (!rmidev) {
        retval = -ENOMEM;
        goto err_rmidev;
    }

    retval = rmidev_create_device_class();
    if (retval < 0) {
        goto err_device_class;
    }

    if (rmidev_major_num) {
        dev_no = MKDEV(rmidev_major_num, DEV_NUMBER);
        retval = register_chrdev_region(dev_no, 1, CHAR_DEVICE_NAME);
        if (retval < 0) {
            goto err_device_region;
        }
    } else {
        retval = alloc_chrdev_region(&dev_no, 0, 1, CHAR_DEVICE_NAME);
        if (retval < 0) {
            goto err_device_region;
        }

        rmidev_major_num = MAJOR(dev_no);
    }

    dev_data = kzalloc(sizeof(*dev_data), GFP_KERNEL);
    if (!dev_data) {
        retval = -ENOMEM;
        goto err_dev_data;
    }

    dev_data->pdata = pdata;

    mutex_init(&dev_data->file_mutex);
    dev_data->rmi_dev = rmidev;
    rmidev->data = dev_data;

    cdev_init(&dev_data->main_dev, &rmidev_fops);

    retval = cdev_add(&dev_data->main_dev, dev_no, 1);
    if (retval < 0) {
        goto err_char_device;
    }

    dev_set_name(&rmidev->dev, "rmidev%d", MINOR(dev_no));
    dev_data->device_class = rmidev_device_class;

    device_ptr = device_create(dev_data->device_class, NULL, dev_no,
            NULL, CHAR_DEVICE_NAME"%d", MINOR(dev_no));
    if (IS_ERR(device_ptr)) {
        dev_err(device_ptr,
                "%s: Failed to create rmi char device\n", __func__);
        retval = -ENODEV;
        goto err_char_device;
    }

    INIT_DELAYED_WORK(&delay_work, remote_rmi4_delay_work);
    schedule_delayed_work(&delay_work, msecs_to_jiffies(8*1000));

    return 0;

err_char_device:
    remote_free_panel_data(dev_data->pdata);
    rmidev_device_cleanup(dev_data);
    kfree(dev_data);

err_dev_data:
    unregister_chrdev_region(dev_no, 1);

err_device_region:
    class_destroy(rmidev_device_class);

err_device_class:
    kfree(rmidev);
    rmidev = NULL;
err_rmidev:
    return retval;
}

//void rmidev_remove_device(void)
void unregister_remote_device(void)
{
    struct rmidev_data *dev_data;

    if (!rmidev)
        return;

    dev_data = rmidev->data;
    if (dev_data) {
        rmidev_device_cleanup(dev_data);
        kfree(dev_data);
    }

    unregister_chrdev_region(rmidev->dev_no, 1);

    class_destroy(rmidev_device_class);

    kfree(rmidev);

    return;
}

/*
   static int __init rmidev_module_init(void)
   {
   rmidev_init_device();

   return 0;
   }

   static void __exit rmidev_module_exit(void)
   {
   rmidev_remove_device();

   return;
   }

   module_init(rmidev_module_init);
   module_exit(rmidev_module_exit);

   MODULE_AUTHOR("Synaptics, Inc.");
   MODULE_DESCRIPTION("Synaptics DSX RMI Dev Module");
   MODULE_LICENSE("GPL v2");
 */

