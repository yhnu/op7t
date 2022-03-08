/*
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

#include "../nfc/nfc.h"
#include "../nfc/sn1xx.h"

#include "ese.h"
#include "sn1xx.h"

extern long sn1xx_nfc_ese_ioctl(struct nfc_dev *nfc_dev, unsigned int cmd, unsigned long arg);

long sn1xx_ese_dev_ioctl(struct ese_dev *ese_dev, unsigned int cmd, unsigned long arg)
{
    long ret = 0;
    mutex_lock(&ese_dev->mutex);
    switch (cmd) {
        case SN1XX_ESE_PERFORM_COLD_RESET:
            ret = sn1xx_nfc_ese_ioctl(ese_dev->nfcc_data, cmd, arg);
        break;
        default:
            ret = -ENOIOCTLCMD;
            pr_err("%s: bad ioctl: cmd = %u, arg = %lu\n", __func__, cmd, arg);
    }
    mutex_unlock(&ese_dev->mutex);
    return ret;
}

void sn1xx_ese_dev_open(struct ese_dev *ese_dev)
{
    nfc_ese_acquire(ese_dev->nfcc_data);
    pr_info("%s: acquired\n", __func__);
}

void sn1xx_ese_dev_release(struct ese_dev *ese_dev)
{
    nfc_ese_release(ese_dev->nfcc_data);
    pr_info("%s: released\n", __func__);
}