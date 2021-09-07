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
#ifndef _NXP_ESE_SN1XX_H_
#define _NXP_ESE_SN1XX_H_

#define sn1xx_max_buffer_size 780U

#define SN1XX_ESE_PERFORM_COLD_RESET  _IOW(NXP_ESE_MAGIC, 0x0C, long)

long sn1xx_ese_dev_ioctl(struct ese_dev *ese_dev, unsigned int cmd, unsigned long arg);
void sn1xx_ese_dev_open(struct ese_dev *ese_dev);
void sn1xx_ese_dev_release(struct ese_dev *ese_dev);
#endif //_NXP_ESE_SN1XX_H_