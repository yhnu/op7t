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
#ifndef _NXP_ESE_H_
#define _NXP_ESE_H_
#define NXP_ESE_MAGIC 0xEA

/* Device specific structure */
struct ese_dev {
    struct spi_device   *spi;
    struct device       *nfcc_device;
    struct nfc_dev      *nfcc_data;
    struct mutex        mutex;
    struct mutex        write_mutex; /* write mutex */
    struct miscdevice   device;
    const char          *nfcc_name;
    size_t kbuflen;
    u8 *kbuf;
};

#endif //_NXP_ESE_H_