/*
 * The original Work has been changed by NXP Semiconductors.
 * Copyright 2013-2019 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*
 * Copyright 2017 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/oneplus/boot_mode.h>

#include "../nfc/nfc.h"

#include "ese.h"
#include "sn1xx.h"
#include "pn8xt.h"

/*Compile time function calls based on the platform selection*/
#define platform_func(prefix, postfix) prefix##postfix
#define func(prefix, postfix) platform_func(prefix, postfix)

#define MAX_BUFFER_SIZE func(NFC_PLATFORM, _max_buffer_size)


static ssize_t ese_dev_read(struct file *filp, char __user *ubuf,
                size_t len, loff_t *offset)
{
    ssize_t ret = -EFAULT;
    struct ese_dev *ese_dev = filp->private_data;
    // char rx_buf[MAX_BUFFER_SIZE];
    unsigned char *rx_buf = NULL;
    mutex_lock(&ese_dev->mutex);
    if (len > MAX_BUFFER_SIZE) {
        len = MAX_BUFFER_SIZE;
    }

    rx_buf = ese_dev->kbuf;
    if (!rx_buf) {
        pr_info("%s: device doesn't exist anymore.\n", __func__);
        ret = -ENODEV;
        return ret;
    }
    memset(rx_buf, 0, len);
    ret = spi_read(ese_dev->spi, rx_buf, len);
    if (0 > ret) {
        pr_err("%s failed to read from SPI\n", __func__);
        mutex_unlock(&ese_dev->mutex);
        return -EIO;
    }
    if (copy_to_user(ubuf, rx_buf, len)) {
        pr_err("%s failed to copy from user\n", __func__);
        mutex_unlock(&ese_dev->mutex);
        return -EFAULT;
    }
    mutex_unlock(&ese_dev->mutex);
    return ret;
}

static ssize_t ese_dev_write(struct file *filp, const char __user *ubuf,
                size_t len, loff_t *offset)
{
    ssize_t ret = -EFAULT;
    struct ese_dev *ese_dev = filp->private_data;
    // char tx_buf[MAX_BUFFER_SIZE];
    char *tx_buf = NULL;

    mutex_lock(&ese_dev->write_mutex);
    if (len > MAX_BUFFER_SIZE)
        len = MAX_BUFFER_SIZE;

#if 1
    tx_buf = memdup_user(ubuf, len);
    if (IS_ERR(tx_buf)) {
        pr_info("%s: memdup_user failed\n", __func__);
        ret = PTR_ERR(tx_buf);
        return ret;
    }
#else
    memset(tx_buf, 0, sizeof(tx_buf));
    if (copy_from_user(tx_buf, ubuf, len)) {
        pr_err("%s: failed to copy from user\n", __func__);
        mutex_unlock(&ese_dev->write_mutex);
        return -EFAULT;
    }
#endif
    ret = spi_write(ese_dev->spi, tx_buf, len);
    if (ret < 0) {
        pr_err("%s: failed to write to SPI\n", __func__);
        mutex_unlock(&ese_dev->write_mutex);
        return -EIO;
    }
    kfree(tx_buf);
    mutex_unlock(&ese_dev->write_mutex);
    return len;
}

static long ese_dev_ioctl(struct file *filep, unsigned int cmd,
        unsigned long arg)
{
    long ret = 0;
    struct ese_dev *ese_dev = filep->private_data;
    ret = func(NFC_PLATFORM, _ese_dev_ioctl)(ese_dev, cmd, arg);
    if (ret != 0)
        pr_err("%s: ioctl: cmd = %u, arg = %lu\n", __func__, cmd, arg);
    return ret;
}

static int ese_dev_open(struct inode *inode, struct file *filp)
{
    struct ese_dev *ese_dev = container_of(filp->private_data,
                struct ese_dev, device);
    mutex_lock(&ese_dev->mutex);
    /* Find the NFC parent device if it exists. */
    if (ese_dev->nfcc_data == NULL) {
        struct device *nfc_dev = bus_find_device_by_name(
                    &i2c_bus_type,
                    NULL,
                    ese_dev->nfcc_name);
        if (!nfc_dev) {
            pr_err("%s: cannot find NFC controller '%s'\n",
                __func__, ese_dev->nfcc_name);
            mutex_unlock(&ese_dev->mutex);
            return -ENODEV;
        }
        ese_dev->nfcc_data = dev_get_drvdata(nfc_dev);
        if (!ese_dev->nfcc_data) {
            pr_err("%s: cannot find NFC controller device data\n",
                __func__);
            put_device(nfc_dev);
            mutex_unlock(&ese_dev->mutex);
            return -ENODEV;
        }
        pr_info("%s: NFC controller found\n", __func__);
        ese_dev->nfcc_device = nfc_dev;
    }
    mutex_unlock(&ese_dev->mutex);
    func(NFC_PLATFORM, _ese_dev_open)(ese_dev);
    filp->private_data = ese_dev;
    pr_info("%s: major,minor: %d,%d\n",
            __func__, imajor(inode), iminor(inode));
    return 0;
}

static int ese_dev_release(struct inode *inode, struct file *filp)
{
    struct ese_dev *ese_dev = filp->private_data;
    func(NFC_PLATFORM, _ese_dev_release)(ese_dev);
    pr_info("%s: released\n", __func__);
    return 0;
}

/* possible fops on the ese device */
static const struct file_operations ese_dev_fops = {
        .owner = THIS_MODULE,
        .read = ese_dev_read,
        .write = ese_dev_write,
        .open = ese_dev_open,
        .release = ese_dev_release,
        .unlocked_ioctl = ese_dev_ioctl,
};

static int ese_probe(struct spi_device *spi)
{
    int ret;
    unsigned int rst_gpio;
    unsigned int max_speed_hz;
    struct ese_dev *ese_dev;
    struct device_node *np = dev_of_node(&spi->dev);

    if (get_second_board_absent() == 1) {
        pr_err("%s second board absent, don't probe p73",__func__);
        return -EINVAL;
    }

    pr_debug("%s: called\n", __func__);
    if (!np) {
        pr_err("%s: device tree data missing\n", __func__);
        return -EINVAL;
    }
    ese_dev = kzalloc(sizeof(*ese_dev), GFP_KERNEL);
    if (ese_dev == NULL) {
        pr_err("%s: No memory\n", __func__);
        return -ENOMEM;
    }

    ese_dev->kbuflen = MAX_BUFFER_SIZE;
    ese_dev->kbuf = kzalloc(MAX_BUFFER_SIZE, GFP_KERNEL);
    if (!ese_dev->kbuf) {
        pr_err("failed to allocate memory for ese_dev->kbuf");
        ret = -ENOMEM;
        goto err_free_dev;
    }

    ese_dev->spi = spi;
    ese_dev->device.minor = MISC_DYNAMIC_MINOR;
    ese_dev->device.name = "p73";
    ese_dev->device.fops = &ese_dev_fops;
    ese_dev->device.parent = &spi->dev;
    spi->bits_per_word = 8;
    spi->mode = SPI_MODE_0;
    ret = of_property_read_u32(np, "spi-max-frequency",
                    &max_speed_hz);
    if (ret < 0) {
        pr_err("%s: There's no spi-max-frequency property\n",
                __func__);
        goto err;
    }
    spi->max_speed_hz = max_speed_hz;
    pr_info("%s: device tree set SPI clock Frequency %u\n",
        __func__, spi->max_speed_hz);
    ret = spi_setup(spi);
    if (ret < 0) {
        pr_err("%s: failed to do spi_setup()\n", __func__);
        goto err;
    }
    rst_gpio = of_get_named_gpio(np, "nxp,p61-rst", 0);
    if (gpio_is_valid(rst_gpio)) {
        ret = gpio_request(rst_gpio, "p61 reset");
        if (ret) {
            pr_err("%s: unable to request rst gpio [%d]\n",
                    __func__, rst_gpio);
            goto err;
        }
        /*soft reset gpio is set to default high*/
        ret = gpio_direction_output(rst_gpio, 1);
        if (ret) {
            pr_err("%s: cannot set direction for rst gpio [%d]\n",
                    __func__, rst_gpio);
            goto err;
        }
    } else {
        /* rst gpio not required for latest platform*/
        pr_info("%s: rst gpio not provided\n", __func__);
    }

    ret = of_property_read_string(np, "nxp,nfcc", &ese_dev->nfcc_name);
    if (ret < 0) {
        pr_err("%s: nxp,nfcc invalid or missing in device tree (%d)\n",
            __func__, ret);
        goto err;
    }
    pr_info("%s: device tree set '%s' as eSE power controller\n",
        __func__, ese_dev->nfcc_name);

    ret = misc_register(&ese_dev->device);
    if (ret) {
        pr_err("%s: misc_register failed\n", __func__);
        goto err;
    }
    pr_info("%s: eSE is configured\n", __func__);
    spi_set_drvdata(spi, ese_dev);

    mutex_init(&ese_dev->mutex);
    mutex_init(&ese_dev->write_mutex);
    return 0;
err:
    kfree(ese_dev->kbuf);
err_free_dev:
    kfree(ese_dev);
    return ret;
}


static int ese_remove(struct spi_device *spi)
{
    struct ese_dev *ese_dev = spi_get_drvdata(spi);
    int ret = 0;
    if (!ese_dev) {
        pr_err("%s: device doesn't exist anymore\n", __func__);
        return -ENODEV;
    }
    /* If we have a NFC device, release it. */
    if (ese_dev->nfcc_device) {
        put_device(ese_dev->nfcc_device);
        ese_dev->nfcc_data = NULL;
        ese_dev->nfcc_device = NULL;
    }
    misc_deregister(&ese_dev->device);
    mutex_destroy(&ese_dev->mutex);
    mutex_destroy(&ese_dev->write_mutex);
    kfree(ese_dev);
    return ret;
}

static struct of_device_id ese_match_table[] = {
    { .compatible = "nxp,p61", },
    { }
};
MODULE_DEVICE_TABLE(of, ese_match_table);
static struct spi_driver ese_driver = {
        .driver = {
                .name = "p61",
                .bus = &spi_bus_type,
                .owner = THIS_MODULE,
                .of_match_table = ese_match_table,
        },
        .probe =  ese_probe,
        .remove = ese_remove,
};

/*
 * module load/unload record keeping
 */

static int __init ese_dev_init(void)
{
    pr_info("Loading NXP ESE driver\n");
    return spi_register_driver(&ese_driver);
}
module_init(ese_dev_init);

static void __exit ese_dev_exit(void)
{
    pr_info("Unloading NXP ESE driver\n");
    spi_unregister_driver(&ese_driver);
}

module_exit(ese_dev_exit);
MODULE_DESCRIPTION("NXP ESE SPI driver");
MODULE_AUTHOR("Google Inc");
MODULE_LICENSE("GPL");