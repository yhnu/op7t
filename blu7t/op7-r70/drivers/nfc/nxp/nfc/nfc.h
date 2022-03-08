/*
 * The original Work has been changed by NXP Semiconductors.
 * Copyright 2013-2019 NXP
 *
 * Copyright (c) 2015-2017, The Linux Foundation. All rights reserved.
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
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef _NXP_NFC_H_
#define _NXP_NFC_H_
#define NXP_NFC_MAGIC 0xE9

/* Device specific structure */
struct nfc_dev    {
    wait_queue_head_t   read_wq;
    struct mutex        read_mutex;
    struct mutex        ese_status_mutex;
    struct i2c_client   *client;
    struct miscdevice   nfc_device;
    /* NFC GPIO variables */
    unsigned int        irq_gpio;
    unsigned int        ven_gpio;
    unsigned int        firm_gpio;
    unsigned int        ese_pwr_gpio;
    /* NFC_IRQ state */
    bool                irq_enabled;
    spinlock_t          irq_enabled_lock;
    unsigned int        count_irq;
    /* read buffer*/
    size_t kbuflen;
    u8 *kbuf;
    /* NFC additional parameters for old platforms */
    void *pdata_op;
};

void nfc_disable_irq(struct nfc_dev *nfc_dev);
void nfc_enable_irq(struct nfc_dev *nfc_dev);
void nfc_ese_acquire(struct nfc_dev *nfc_dev);
void nfc_ese_release(struct nfc_dev *nfc_dev);

struct hw_type_info {
    /*
     * Response of get_version_cmd will be stored in data
     * byte structure :
     * byte 0-1     : Header
     * byte 2       : Status
     * byte 3       : Hardware Version
     * byte 4       : ROM code
     * byte 5       : 0x00 constant
     * byte 6-7     : Protected data version
     * byte 8-9     : Trim data version
     * byte 10-11   : FW version
     * byte 12-13   : CRC
     * */
    char data[20];
    int len;
};

enum nfcc_chip_variant {
    NFCC_NQ_210         = 0x48, /**< NFCC NQ210 */
    NFCC_NQ_220         = 0x58, /**< NFCC NQ220 */
    NFCC_NQ_310         = 0x40, /**< NFCC NQ310 */
    NFCC_NQ_310_V2          = 0x41, /**< NFCC NQ310 */
    NFCC_NQ_330         = 0x51, /**< NFCC NQ330 */
    NFCC_PN66T          = 0x18, /**< NFCC PN66T */
    NFCC_NOT_SUPPORTED          = 0xFF  /**< NFCC is not supported */
};
#endif //_NXP_NFC_H_