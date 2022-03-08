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
#ifndef _NXP_ESE_PN8XT_H_
#define _NXP_ESE_PN8XT_H_

#define pn8xt_max_buffer_size 258U

/* SPI Request NFCC to enable pn8xt ese power,
 * Only for SPI
 * level 1 = Enable power
 * level 0 = Disable power
 */
#define PN8XT_ESE_SET_SPI_PWR           _IOW(NXP_ESE_MAGIC, 0x04, unsigned int)
/* SPI or DWP can call this ioctl to get the current
 * power state of P61
*/
#define PN8XT_ESE_GET_SPI_STATUS        _IOR(NXP_ESE_MAGIC, 0x05, unsigned int)
#define PN8XT_ESE_GET_ACCESS            _IOW(NXP_ESE_MAGIC, 0x07, unsigned int)
#define PN8XT_ESE_SET_PWR_SCM           _IOW(NXP_ESE_MAGIC, 0x08, unsigned int)
#define PN8XT_ESE_SET_DN_STATUS         _IOW(NXP_ESE_MAGIC, 0x09, unsigned int)
#define PN8XT_ESE_INHIBIT_PWR_CNTRL     _IOW(NXP_ESE_MAGIC, 0x0A, unsigned int)

long pn8xt_ese_dev_ioctl(struct ese_dev *ese_dev, unsigned int cmd, unsigned long arg);
void pn8xt_ese_dev_open(struct ese_dev *ese_dev);
void pn8xt_ese_dev_release(struct ese_dev *ese_dev);
#endif //_NXP_ESE_PN8XT_H_