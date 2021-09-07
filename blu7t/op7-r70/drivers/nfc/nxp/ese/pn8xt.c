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
#include "../nfc/pn8xt.h"

#include "ese.h"
#include "pn8xt.h"

extern long pn8xt_nfc_ese_ioctl(struct nfc_dev *nfc_dev, unsigned int cmd, unsigned long arg);

long pn8xt_ese_dev_ioctl(struct ese_dev *ese_dev, unsigned int cmd, unsigned long arg)
{
    long ret = 0;
    pr_debug("%s :enter cmd = %u, arg = %ld\n", __func__, cmd, arg);

    switch (cmd) {
        case PN8XT_ESE_SET_SPI_PWR:
            ret = pn8xt_nfc_ese_ioctl(ese_dev->nfcc_data, PN8XT_SET_SPI_PWR, arg);
            break;
        case PN8XT_ESE_GET_SPI_STATUS:
            ret = pn8xt_nfc_ese_ioctl(ese_dev->nfcc_data, PN8XT_GET_PWR_STATUS, arg);
            break;
        case PN8XT_ESE_GET_ACCESS:
            ret = pn8xt_nfc_ese_ioctl(ese_dev->nfcc_data, PN8XT_GET_ESE_ACCESS, arg);
            break;
        case PN8XT_ESE_SET_PWR_SCM:
            ret = pn8xt_nfc_ese_ioctl(ese_dev->nfcc_data, PN8XT_SET_POWER_SCM, arg);
            break;
        case PN8XT_ESE_SET_DN_STATUS:
            ret = pn8xt_nfc_ese_ioctl(ese_dev->nfcc_data, PN8XT_SET_DN_STATUS, arg);
            break;
        case PN8XT_ESE_INHIBIT_PWR_CNTRL:
            ret = pn8xt_nfc_ese_ioctl(ese_dev->nfcc_data, PN8XT_SECURE_TIMER_SESSION, arg);
            break;
        default:
            ret = -ENOIOCTLCMD;
            pr_err("%s: bad ioctl: cmd = %u, arg = %lu\n", __func__, cmd, arg);
    }
    pr_debug("%s :exit cmd = %u, arg = %ld\n", __func__, cmd, arg);
    return ret;
}

void pn8xt_ese_dev_open(struct ese_dev *ese_dev)
{
    pr_info("%s: called\n", __func__);
}

void pn8xt_ese_dev_release(struct ese_dev *ese_dev)
{
    pr_info("%s: called\n", __func__);
}