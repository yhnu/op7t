/*
 * aw8697.c   aw8697 haptic module
 *
 * Version: v1.3.3
 *
 * Copyright (c) 2018 AWINIC Technology CO., LTD
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include "aw8697.h"
#include "aw8697_reg.h"
#include "aw8697_config.h"
#include <linux/oneplus/boot_mode.h>
#include <linux/pm_qos.h>

#include <linux/msm_drm_notify.h>
#include <linux/fb.h>
#include <linux/vmalloc.h>


/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW8697_I2C_NAME "aw8697_haptic"
#define AW8697_HAPTIC_NAME "aw8697_haptic"

#define AW8697_VERSION "v1.3.3"

#define AWINIC_RAM_UPDATE_DELAY

#define AW_I2C_RETRIES 10
#define AW_I2C_RETRY_DELAY 2
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 2
#define AW8697_MAX_DSP_START_TRY_COUNT    10
#define AWINIC_READ_BIN_FLEXBALLY

#define AW8697_MAX_FIRMWARE_LOAD_CNT 20
#define OP_AW_DEBUG
#define OP_OCS_CALIBRATION_T_LENGTH 3500000

#define PM_QOS_VALUE_VB 200
struct pm_qos_request pm_qos_req_vb;

#define SCORE_MODE
#ifdef SCORE_MODE
#define FIRST_SCOREMODE
//#define SECOND_SCOREMODE
#else
#define AISCAN_CTRL
#endif

#define TRUST_LEVEL 5
#define ABS(x)		((x) < 0 ? (-x) : (x))

/* add haptic audio tp mask */
extern struct shake_point record_point[10];
/* add haptic audio tp mask end */
/******************************************************
 *
 * variable
 *
 ******************************************************/
#define AW8697_RTP_NAME_MAX        64
static char aw8697_ram_name[5][30] = {
{"aw8697_haptic_166.bin"},
{"aw8697_haptic_168.bin"},
{"aw8697_haptic_170.bin"},
{"aw8697_haptic_172.bin"},
{"aw8697_haptic_174.bin"},
};
static char aw8697_rtp_name[][AW8697_RTP_NAME_MAX] = {
	{"aw8697_rtp.bin"},
	{"ring_alacrity.bin"},
	{"ring_amenity.bin"},
	{"ring_blues.bin"},
	{"ring_brisk.bin"},
	{"ring_cloud.bin"},
	{"ring_cyclotron.bin"},
	{"ring_distinct.bin"},
	{"ring_dynamic.bin"},
	{"ring_echo.bin"},
	{"ring_expect.bin"},
	{"ring_fanatical.bin"},
	{"ring_funky.bin"},
	{"ring_game.bin"},
	{"ring_guitar.bin"},
	{"ring_harping.bin"},
	{"ring_highlight.bin"},
	{"ring_idyl.bin"},
	{"ring_innocence.bin"},
	{"ring_journey.bin"},
	{"ring_joyous.bin"},
	{"ring_marimba.bin"},
	{"ring_old_telephone.bin"},
	{"ring_oneplus_tune.bin"},
	{"ring_oneplus_tune_rhythm.bin"},
	{"ring_optimistic.bin"},
	{"ring_piano.bin"},
	{"ring_pyxis.bin"},
	{"ring_T-Jingle.bin"},
	{"ring_McLarn.bin"},
	{"notif_allusion.bin"},
	{"notif_amiable.bin"},
	{"notif_blare.bin"},
	{"notif_blissful.bin"},
	{"notif_cheerful.bin"},
	{"notif_comely.bin"},
	{"notif_cozy.bin"},
	{"notif_ding.bin"},
	{"notif_effervesce.bin"},
	{"notif_elegant.bin"},
	{"notif_free.bin"},
	{"notif_hallucination.bin"},
	{"notif_inbound.bin"},
	{"notif_incidence.bin"},
	{"notif_light.bin"},
	{"notif_meet.bin"},
	{"notif_naivety.bin"},
	{"notif_quickly.bin"},
	{"notif_rhythm.bin"},
	{"notif_silent.bin"},
	{"notif_spirit.bin"},
	{"notif_surprise.bin"},
	{"notif_twinkle.bin"},
	{"notif_Ardour.bin"},
	{"notif_Chic.bin"},
	{"alarm_clock1.bin"},
	{"alarm_clock2.bin"},
	{"alarm_clock3.bin"},
	{"alarm_clock4.bin"},
	{"alarm_clock5.bin"},
	{"alarm_breeze.bin"},
	{"alarm_dream.bin"},
	{"alarm_fluttering.bin"},
	{"alarm_flyer.bin"},
	{"alarm_interesting.bin"},
	{"alarm_leisurely.bin"},
	{"alarm_memory.bin"},
	{"alarm_relieved.bin"},
	{"alarm_ripple.bin"},
	{"alarm_spring.bin"},
	{"alarm_stars.bin"},
	{"alarm_surging.bin"},
	{"alarm_tactfully.bin"},
	{"alarm_the_wind.bin"},
	{"alarm_walking_in_the_rain.bin"},
	{"shuntai24k_rtp.bin"},
	{"wentai24k_rtp.bin"},
};

struct aw8697_container *aw8697_rtp;
struct aw8697 *g_aw8697;
static int aw8697_haptic_get_f0(struct aw8697 *aw8697);

/******************************************************
 *
 * functions
 *
 ******************************************************/
static void aw8697_interrupt_clear(struct aw8697 *aw8697);
static int aw8697_haptic_trig_enable_config(struct aw8697 *aw8697);

#define get_abs(a) ((a)>=0? (a) : (-(a)))
 /******************************************************
 *
 * aw8697 i2c write/read
 *
 ******************************************************/
static int aw8697_i2c_write(struct aw8697 *aw8697,
         unsigned char reg_addr, unsigned char reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;

    while(cnt < AW_I2C_RETRIES) {
        ret = i2c_smbus_write_byte_data(aw8697->i2c, reg_addr, reg_data);
        if(ret < 0) {
            pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            break;
        }
        cnt ++;
        msleep(AW_I2C_RETRY_DELAY);
    }
    return ret;
}

static int aw8697_i2c_read(struct aw8697 *aw8697,
        unsigned char reg_addr, unsigned char *reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;

    while(cnt < AW_I2C_RETRIES) {
        ret = i2c_smbus_read_byte_data(aw8697->i2c, reg_addr);
        if(ret < 0) {
            pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            *reg_data = ret;
            break;
        }
        cnt ++;
        msleep(AW_I2C_RETRY_DELAY);
    }

    return ret;
}

static int aw8697_i2c_write_bits(struct aw8697 *aw8697,
         unsigned char reg_addr, unsigned int mask, unsigned char reg_data)
{
    unsigned char reg_val = 0;

    aw8697_i2c_read(aw8697, reg_addr, &reg_val);
    reg_val &= mask;
    reg_val |= reg_data;
    aw8697_i2c_write(aw8697, reg_addr, reg_val);

    return 0;
}

static int aw8697_i2c_writes(struct aw8697 *aw8697,
        unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
    int ret = -1;
    unsigned char *data;

    data = kmalloc(len+1, GFP_KERNEL);
    if (data == NULL) {
        pr_err("%s: can not allocate memory\n", __func__);
        return  -ENOMEM;
    }

    data[0] = reg_addr;
    memcpy(&data[1], buf, len);

    ret = i2c_master_send(aw8697->i2c, data, len+1);
    if (ret < 0) {
        pr_err("%s: i2c master send error\n", __func__);
    }

    kfree(data);

    return ret;
}

/*****************************************************
 *
 * ram update
 *
 *****************************************************/
static void aw8697_rtp_loaded(const struct firmware *cont, void *context)
{
    struct aw8697 *aw8697 = context;
    pr_info("%s enter\n", __func__);

    if (!cont) {
        pr_err("%s: failed to read %s\n", __func__, aw8697_rtp_name[aw8697->rtp_file_num]);
        release_firmware(cont);
        return;
    }

    pr_info("%s: loaded %s - size: %zu\n", __func__, aw8697_rtp_name[aw8697->rtp_file_num],
                    cont ? cont->size : 0);

    /* aw8697 rtp update */
    mutex_lock(&aw8697->rtp_lock);
    aw8697_rtp = vmalloc(cont->size+sizeof(int));
    if (!aw8697_rtp) {
        release_firmware(cont);
        mutex_unlock(&aw8697->rtp_lock);
        pr_err("%s: Error allocating memory\n", __func__);
        return;
    }
    aw8697_rtp->len = cont->size;
    pr_info("%s: rtp size = %d\n", __func__, aw8697_rtp->len);
    memcpy(aw8697_rtp->data, cont->data, cont->size);
    release_firmware(cont);
    mutex_unlock(&aw8697->rtp_lock);

    aw8697->rtp_init = 1;
    pr_info("%s: rtp update complete\n", __func__);
}

static int aw8697_rtp_update(struct aw8697 *aw8697)
{
    pr_info("%s enter\n", __func__);

    return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
                aw8697_rtp_name[aw8697->rtp_file_num], aw8697->dev, GFP_KERNEL,
                aw8697, aw8697_rtp_loaded);
}


 static void aw8697_container_update(struct aw8697 *aw8697,
        struct aw8697_container *aw8697_cont)
{
    int i = 0;
    unsigned int shift = 0;

    pr_info("%s enter\n", __func__);

    mutex_lock(&aw8697->lock);

    aw8697->ram.baseaddr_shift = 2;
    aw8697->ram.ram_shift = 4;

    /* RAMINIT Enable */
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_EN);

    /* base addr */
    shift = aw8697->ram.baseaddr_shift;
    aw8697->ram.base_addr = (unsigned int)((aw8697_cont->data[0+shift]<<8) |
            (aw8697_cont->data[1+shift]));
    pr_info("%s: base_addr=0x%4x\n", __func__, aw8697->ram.base_addr);

    aw8697_i2c_write(aw8697, AW8697_REG_BASE_ADDRH, aw8697_cont->data[0+shift]);
    aw8697_i2c_write(aw8697, AW8697_REG_BASE_ADDRL, aw8697_cont->data[1+shift]);

    aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AEH,
                    (unsigned char)((aw8697->ram.base_addr>>1)>>8)); /*1/2 FIFO*/
    aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AEL,
                    (unsigned char)((aw8697->ram.base_addr>>1)&0x00FF));/*1/2 FIFO*/
    aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AFH,
                    (unsigned char)((aw8697->ram.base_addr-(aw8697->ram.base_addr>>2))>>8));
    aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AFL,
                    (unsigned char)((aw8697->ram.base_addr-(aw8697->ram.base_addr>>2))&0x00FF));

    /* ram */
    shift = aw8697->ram.baseaddr_shift;
    aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRH, aw8697_cont->data[0+shift]);
    aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRL, aw8697_cont->data[1+shift]);
    shift = aw8697->ram.ram_shift;
    for(i=shift; i<aw8697_cont->len; i++) {
        aw8697_i2c_write(aw8697, AW8697_REG_RAMDATA, aw8697_cont->data[i]);
    }

    /* RAMINIT Disable */
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_OFF);

    mutex_unlock(&aw8697->lock);

    pr_info("%s exit\n", __func__);
}


static void aw8697_ram_loaded(const struct firmware *cont, void *context)
{
    struct aw8697 *aw8697 = context;
    struct aw8697_container *aw8697_fw;
    int i = 0;
    unsigned short check_sum = 0;
    #ifdef AWINIC_READ_BIN_FLEXBALLY
    static unsigned char load_count =0;
    int ram_timer_val = 1000;
    load_count++;
    #endif

    pr_info("%s enter\n", __func__);

    if (!cont) {
        pr_err("%s: failed to read %s\n", __func__, aw8697_ram_name[aw8697->ram_bin_index]);
        release_firmware(cont);
    #ifdef AWINIC_READ_BIN_FLEXBALLY
    if(load_count <=20)
    {
       schedule_delayed_work(&aw8697->ram_work, msecs_to_jiffies(ram_timer_val));
       pr_info("%s:start hrtimer:load_count%d\n", __func__, load_count);
    }
    #endif
        return;
    }

    pr_info("%s: loaded %s - size: %zu\n", __func__, aw8697_ram_name[aw8697->ram_bin_index],
                    cont ? cont->size : 0);
    /* check sum */
    for(i=2; i<cont->size; i++) {
        check_sum += cont->data[i];
    }
    if(check_sum != (unsigned short)((cont->data[0]<<8)|(cont->data[1]))) {
        pr_err("%s: check sum err: check_sum=0x%04x\n", __func__, check_sum);
        return;
    } else {
        pr_info("%s: check sum pass : 0x%04x\n", __func__, check_sum);
        aw8697->ram.check_sum = check_sum;
    }

    /* aw8697 ram update */
    aw8697_fw = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
    if (!aw8697_fw) {
        release_firmware(cont);
        pr_err("%s: Error allocating memory\n", __func__);
        return;
    }
    aw8697_fw->len = cont->size;
    memcpy(aw8697_fw->data, cont->data, cont->size);
    release_firmware(cont);

    aw8697_container_update(aw8697, aw8697_fw);

    aw8697->ram.len = aw8697_fw->len;

    kfree(aw8697_fw);

    aw8697->ram_init = 1;
    pr_info("%s: fw update complete\n", __func__);
    aw8697_rtp_update(aw8697);
}

static int aw8697_ram_update(struct aw8697 *aw8697)
{
    aw8697->ram_init = 0;
    aw8697->rtp_init = 0;

    pr_info("%s:aw8697->haptic_real_f0:%d\n", __func__,aw8697->haptic_real_f0);
    if (aw8697->haptic_real_f0 <167)
        aw8697->ram_bin_index = 0;
    else if(aw8697->haptic_real_f0 <169)
        aw8697->ram_bin_index = 1;
    else if(aw8697->haptic_real_f0 <171)
        aw8697->ram_bin_index = 2;
    else if(aw8697->haptic_real_f0 <173)
        aw8697->ram_bin_index = 3;
    else
        aw8697->ram_bin_index = 4;
    pr_info("%s:haptic bin name  %s \n", __func__,aw8697_ram_name[aw8697->ram_bin_index]);
    return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
                aw8697_ram_name[aw8697->ram_bin_index], aw8697->dev, GFP_KERNEL,
                aw8697, aw8697_ram_loaded);
}

#ifdef AWINIC_RAM_UPDATE_DELAY
static void aw8697_ram_work_routine(struct work_struct *work)
{
    struct aw8697 *aw8697 = container_of(work, struct aw8697, ram_work.work);

    pr_info("%s enter\n", __func__);
    aw8697_ram_update(aw8697);
}
#endif

static int aw8697_ram_init(struct aw8697 *aw8697)
{

#ifdef AWINIC_RAM_UPDATE_DELAY
    int ram_timer_val = 1000;
    aw8697->haptic_real_f0 = AW8697_HAPTIC_F0_PRE/10;

    INIT_DELAYED_WORK(&aw8697->ram_work, aw8697_ram_work_routine);
    schedule_delayed_work(&aw8697->ram_work, msecs_to_jiffies(ram_timer_val));
#else
    aw8697_ram_update(aw8697);
#endif
    return 0;
}

/*****************************************************
 *
 * haptic control
 *
 *****************************************************/
static int aw8697_haptic_softreset(struct aw8697 *aw8697)
{
    pr_debug("%s enter\n", __func__);

    aw8697_i2c_write(aw8697, AW8697_REG_ID, 0xAA);
    msleep(1);
    return 0;
}

static int aw8697_haptic_active(struct aw8697 *aw8697)
{
    pr_debug("%s enter\n", __func__);

    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_WORK_MODE_MASK, AW8697_BIT_SYSCTRL_ACTIVE);
    aw8697_interrupt_clear(aw8697);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
            AW8697_BIT_SYSINTM_UVLO_MASK, AW8697_BIT_SYSINTM_UVLO_EN);
    return 0;
}

static int aw8697_haptic_play_mode(struct aw8697 *aw8697, unsigned char play_mode)
{
    pr_debug("%s enter\n", __func__);

    switch(play_mode) {
        case AW8697_HAPTIC_STANDBY_MODE:
            aw8697->play_mode = AW8697_HAPTIC_STANDBY_MODE;
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
                    AW8697_BIT_SYSINTM_UVLO_MASK, AW8697_BIT_SYSINTM_UVLO_OFF);
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                    AW8697_BIT_SYSCTRL_WORK_MODE_MASK, AW8697_BIT_SYSCTRL_STANDBY);
            break;
        case AW8697_HAPTIC_RAM_MODE:
            aw8697->play_mode = AW8697_HAPTIC_RAM_MODE;
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                    AW8697_BIT_SYSCTRL_PLAY_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                    AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
            if(aw8697->auto_boost) {
                aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
                        AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
            }
            aw8697_haptic_active(aw8697);
            if(aw8697->auto_boost) {
                aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
                        AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RAM_ENABLE);
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                        AW8697_BIT_SYSCTRL_BST_MODE_MASK&AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
                        AW8697_BIT_SYSCTRL_BST_MODE_BOOST|AW8697_BIT_SYSCTRL_STANDBY);
                aw8697_haptic_active(aw8697);
            } else {
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                    AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
            }
            mdelay(2);
            break;
        case AW8697_HAPTIC_RAM_LOOP_MODE:
            aw8697->play_mode = AW8697_HAPTIC_RAM_LOOP_MODE;
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                    AW8697_BIT_SYSCTRL_PLAY_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                    AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
            if(aw8697->auto_boost) {
                aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
                        AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
            }
            aw8697_haptic_active(aw8697);
            break;
        case AW8697_HAPTIC_RTP_MODE:
            aw8697->play_mode = AW8697_HAPTIC_RTP_MODE;
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                    AW8697_BIT_SYSCTRL_PLAY_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_RTP);
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                    AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
            if(aw8697->auto_boost) {
                aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
                        AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RTP_DISABLE);
            }
            aw8697_haptic_active(aw8697);
            if(aw8697->auto_boost) {
                aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
                        AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RTP_ENABLE);
                aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                        AW8697_BIT_SYSCTRL_BST_MODE_MASK&AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
                        AW8697_BIT_SYSCTRL_BST_MODE_BOOST|AW8697_BIT_SYSCTRL_STANDBY);
                aw8697_haptic_active(aw8697);
            } else {
                aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                        AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
            }
            msleep(3);
            break;
        case AW8697_HAPTIC_TRIG_MODE:
            aw8697->play_mode = AW8697_HAPTIC_TRIG_MODE;
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                    AW8697_BIT_SYSCTRL_PLAY_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                    AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
            if(aw8697->auto_boost) {
                aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
                        AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
            }
            aw8697_haptic_active(aw8697);
            if(aw8697->auto_boost) {
                aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
                        AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RAM_ENABLE);
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                        AW8697_BIT_SYSCTRL_BST_MODE_MASK&AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
                        AW8697_BIT_SYSCTRL_BST_MODE_BOOST|AW8697_BIT_SYSCTRL_STANDBY);
                aw8697_haptic_active(aw8697);
            } else {
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                    AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
            }
            msleep(3);
            break;
        case AW8697_HAPTIC_CONT_MODE:
            aw8697->play_mode = AW8697_HAPTIC_CONT_MODE;
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                    AW8697_BIT_SYSCTRL_PLAY_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_CONT);
            aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                    AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
            if(aw8697->auto_boost) {
                aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
                        AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
            }
            aw8697_haptic_active(aw8697);
            break;
        default:
            dev_err(aw8697->dev, "%s: play mode %d err",
                    __func__, play_mode);
            break;
    }
    return 0;
}
/*OP add for juge rtp on begin*/
static int aw8697_haptic_juge_RTP_is_going_on(struct aw8697 *aw8697)
{
    unsigned char reg_val = 0;
    unsigned char rtp_state = 0;
    static unsigned char pre_reg_val;

    aw8697_i2c_read(aw8697, AW8697_REG_SYSCTRL, &reg_val);
    if((reg_val&AW8697_BIT_SYSCTRL_PLAY_MODE_RTP)&&(!(reg_val&AW8697_BIT_SYSCTRL_STANDBY)))
         rtp_state = 1;/*is going on*/
    if (pre_reg_val != reg_val)
        pr_debug("%sAW8697_REG_SYSCTRL 0x04==%02x rtp_state=%d\n", __func__,reg_val,rtp_state);
    pre_reg_val = reg_val;
    if (aw8697->rtp_routine_on) {
        pr_debug("%s:rtp_routine_on\n", __func__);
        rtp_state = 1;/*is going on*/
    }
    aw8697->rtp_on = (bool)rtp_state;
    return rtp_state;
}
/*OP add for juge rtp on begin*/

static int aw8697_haptic_play_go(struct aw8697 *aw8697, bool flag)
{
    pr_debug("%s enter\n", __func__);
    if (!flag) {
       do_gettimeofday(&aw8697->current_time);
        aw8697->interval_us = (aw8697->current_time.tv_sec-aw8697->pre_enter_time.tv_sec) * 1000000
          + (aw8697->current_time.tv_usec-aw8697->pre_enter_time.tv_usec);
       if (aw8697->interval_us < 2000) {
           pr_info("aw8697->interval_us t=%ld\n",aw8697->interval_us);
           mdelay(2);
       }
    }
    if (flag == true) {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_GO,
            AW8697_BIT_GO_MASK, AW8697_BIT_GO_ENABLE);
        do_gettimeofday(&aw8697->pre_enter_time);
    } else {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_GO,
            AW8697_BIT_GO_MASK, AW8697_BIT_GO_DISABLE);
    }
    return 0;
}

static int aw8697_haptic_stop_delay(struct aw8697 *aw8697)
{
    unsigned char reg_val = 0;
    unsigned int cnt = 60;

    while(cnt--) {
        aw8697_i2c_read(aw8697, AW8697_REG_GLB_STATE, &reg_val);
        if((reg_val&0x0f) == 0x00) {
            return 0;
        }
        mdelay(2);
        pr_info("%s wait for standby, reg glb_state=0x%02x\n",
            __func__, reg_val);
    }
    pr_err("%s do not enter standby automatically\n", __func__);

    return 0;
}

int aw8697_op_haptic_stop(void)
{
    pr_debug("%s enter\n", __func__);
    if (g_aw8697 == NULL)
       return 0;
    mutex_lock(&g_aw8697->lock);
    if (g_aw8697) {
       aw8697_haptic_play_go(g_aw8697, false);
       aw8697_haptic_stop_delay(g_aw8697);
       aw8697_haptic_play_mode(g_aw8697, AW8697_HAPTIC_STANDBY_MODE);
    }
    mutex_unlock(&g_aw8697->lock);

    return 0;
}

static int aw8697_haptic_stop(struct aw8697 *aw8697)
{
    pr_debug("%s enter\n", __func__);
#ifdef OP_AW_DEBUG
		do_gettimeofday(&aw8697->t_stop);
#endif

    aw8697_haptic_play_go(aw8697, false);
    aw8697_haptic_stop_delay(aw8697);
    aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_STANDBY_MODE);
    return 0;
}

static int aw8697_haptic_start(struct aw8697 *aw8697)
{
    pr_debug("%s enter\n", __func__);
    aw8697_haptic_play_go(aw8697, true);
#ifdef OP_AW_DEBUG
    do_gettimeofday(&aw8697->t_start);
    aw8697->game_microsecond = (aw8697->t_start.tv_sec-aw8697->t_stop.tv_sec) * 1000000 +
        (aw8697->t_start.tv_usec-aw8697->t_stop.tv_usec);
    pr_debug("%s: haptic_audio_delay=%dus\n", __func__, aw8697->game_microsecond);
#endif
    return 0;
}

static int aw8697_haptic_set_wav_seq(struct aw8697 *aw8697,
        unsigned char wav, unsigned char seq)
{
    aw8697_i2c_write(aw8697, AW8697_REG_WAVSEQ1+wav,
            seq);
    return 0;
}

static int aw8697_haptic_set_wav_loop(struct aw8697 *aw8697,
        unsigned char wav, unsigned char loop)
{
    unsigned char tmp = 0;

    if(wav%2) {
        tmp = loop<<0;
        aw8697_i2c_write_bits(aw8697, AW8697_REG_WAVLOOP1+(wav/2),
                AW8697_BIT_WAVLOOP_SEQNP1_MASK, tmp);
    } else {
        tmp = loop<<4;
        aw8697_i2c_write_bits(aw8697, AW8697_REG_WAVLOOP1+(wav/2),
                AW8697_BIT_WAVLOOP_SEQN_MASK, tmp);
    }

    return 0;
}

static int aw8697_haptic_set_repeat_wav_seq(struct aw8697 *aw8697, unsigned char seq)
{
    aw8697_haptic_set_wav_seq(aw8697, 0x00, seq);
    aw8697_haptic_set_wav_loop(aw8697, 0x00, AW8697_BIT_WAVLOOP_INIFINITELY);

    return 0;
}


static int aw8697_haptic_set_bst_vol(struct aw8697 *aw8697, unsigned char bst_vol)
{
    if(bst_vol & 0xe0) {
        bst_vol = 0x1f;
    }
    aw8697_i2c_write_bits(aw8697, AW8697_REG_BSTDBG4,
            AW8697_BIT_BSTDBG4_BSTVOL_MASK, (bst_vol<<1));
    return 0;
}

static int aw8697_haptic_set_bst_peak_cur(struct aw8697 *aw8697, unsigned char peak_cur)
{
    peak_cur &= AW8697_BSTCFG_PEAKCUR_LIMIT;
    aw8697_i2c_write_bits(aw8697, AW8697_REG_BSTCFG,
            AW8697_BIT_BSTCFG_PEAKCUR_MASK, peak_cur);
    return 0;
}

static unsigned char aw8697_haptic_set_level(struct aw8697 *aw8697, int gain)
{
    int val = 80;

    val = aw8697->level * gain / 3;
    if (val > 255)
        val = 255;

    return val;
}

static int aw8697_haptic_set_gain(struct aw8697 *aw8697, unsigned char gain)
{
    aw8697_i2c_write(aw8697, AW8697_REG_DATDBG, aw8697_haptic_set_level(aw8697, gain));
    return 0;
}

static int aw8697_haptic_set_pwm(struct aw8697 *aw8697, unsigned char mode)
{
    switch(mode) {
        case AW8697_PWM_48K:
            aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMDBG,
                AW8697_BIT_PWMDBG_PWM_MODE_MASK, AW8697_BIT_PWMDBG_PWM_48K);
            break;
        case AW8697_PWM_24K:
            aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMDBG,
                AW8697_BIT_PWMDBG_PWM_MODE_MASK, AW8697_BIT_PWMDBG_PWM_24K);
            break;
        case AW8697_PWM_12K:
            aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMDBG,
                AW8697_BIT_PWMDBG_PWM_MODE_MASK, AW8697_BIT_PWMDBG_PWM_12K);
            break;
        default:
            break;
    }
    return 0;
}

static int aw8697_haptic_play_wav_seq(struct aw8697 *aw8697, unsigned char flag)
{
    pr_debug("%s enter\n", __func__);

    if(flag) {
        aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RAM_MODE);
        aw8697_haptic_start(aw8697);
    }
    return 0;
}

static int aw8697_haptic_play_repeat_seq(struct aw8697 *aw8697, unsigned char flag)
{
    pr_debug("%s enter\n", __func__);
    if(flag) {
        aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RAM_LOOP_MODE);
        aw8697_haptic_start(aw8697);
    }

    return 0;
}

static int aw8697_haptic_swicth_motorprotect_config(struct aw8697 *aw8697, unsigned char addr, unsigned char val)
{
    pr_debug("%s enter\n", __func__);
    if(addr == 1) {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
                AW8697_BIT_DETCTRL_PROTECT_MASK, AW8697_BIT_DETCTRL_PROTECT_SHUTDOWN);
        aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMPRC,
                AW8697_BIT_PWMPRC_PRC_MASK, AW8697_BIT_PWMPRC_PRC_ENABLE);
        aw8697_i2c_write_bits(aw8697, AW8697_REG_PRLVL,
                AW8697_BIT_PRLVL_PR_MASK, AW8697_BIT_PRLVL_PR_ENABLE);
    } else if (addr == 0) {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
                AW8697_BIT_DETCTRL_PROTECT_MASK,  AW8697_BIT_DETCTRL_PROTECT_NO_ACTION);
        aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMPRC,
                AW8697_BIT_PWMPRC_PRC_MASK, AW8697_BIT_PWMPRC_PRC_DISABLE);
        aw8697_i2c_write_bits(aw8697, AW8697_REG_PRLVL,
                AW8697_BIT_PRLVL_PR_MASK, AW8697_BIT_PRLVL_PR_DISABLE);
    } else if (addr == 0x2d){
        aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMPRC,
                AW8697_BIT_PWMPRC_PRCTIME_MASK, val);
    }else if (addr == 0x3e){
        aw8697_i2c_write_bits(aw8697, AW8697_REG_PRLVL,
                AW8697_BIT_PRLVL_PRLVL_MASK, val);
    }else if (addr == 0x3f){
        aw8697_i2c_write_bits(aw8697, AW8697_REG_PRTIME,
                AW8697_BIT_PRTIME_PRTIME_MASK, val);
    } else{
        /*nothing to do;*/
    }
	 return 0;
}

/*****************************************************
 *
 * os calibration
 *
 *****************************************************/
static int aw8697_haptic_os_calibration(struct aw8697 *aw8697)
{
    unsigned int cont = 2000;
    unsigned char reg_val = 0;
    pr_debug("%s enter\n", __func__);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_EN);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
            AW8697_BIT_DETCTRL_DIAG_GO_MASK, AW8697_BIT_DETCTRL_DIAG_GO_ENABLE);
    while(1){
        aw8697_i2c_read(aw8697, AW8697_REG_DETCTRL, &reg_val);
        if((reg_val & 0x01) == 0 || cont ==0)
            break;
         cont--;
    }
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_OFF);
    return 0;
}
/*****************************************************
 *
 * trig config
 *
 *****************************************************/
static int aw8697_haptic_trig_param_init(struct aw8697 *aw8697)
{
    pr_debug("%s enter\n", __func__);

    aw8697->trig[0].enable = AW8697_TRG1_ENABLE;
    aw8697->trig[0].default_level = AW8697_TRG1_DEFAULT_LEVEL;
    aw8697->trig[0].dual_edge = AW8697_TRG1_DUAL_EDGE;
    aw8697->trig[0].frist_seq = AW8697_TRG1_FIRST_EDGE_SEQ;
    aw8697->trig[0].second_seq = AW8697_TRG1_SECOND_EDGE_SEQ;

    aw8697->trig[1].enable = AW8697_TRG2_ENABLE;
    aw8697->trig[1].default_level = AW8697_TRG2_DEFAULT_LEVEL;
    aw8697->trig[1].dual_edge = AW8697_TRG2_DUAL_EDGE;
    aw8697->trig[1].frist_seq = AW8697_TRG2_FIRST_EDGE_SEQ;
    aw8697->trig[1].second_seq = AW8697_TRG2_SECOND_EDGE_SEQ;

    aw8697->trig[2].enable = AW8697_TRG3_ENABLE;
    aw8697->trig[2].default_level = AW8697_TRG3_DEFAULT_LEVEL;
    aw8697->trig[2].dual_edge = AW8697_TRG3_DUAL_EDGE;
    aw8697->trig[2].frist_seq = AW8697_TRG3_FIRST_EDGE_SEQ;
    aw8697->trig[2].second_seq = AW8697_TRG3_SECOND_EDGE_SEQ;

    return 0;
}
static int aw8697_haptic_trig_param_config(struct aw8697 *aw8697)
{
    pr_debug("%s enter\n", __func__);

    if(aw8697->trig[0].default_level) {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
                AW8697_BIT_TRGCFG1_TRG1_POLAR_MASK, AW8697_BIT_TRGCFG1_TRG1_POLAR_NEG);
    } else {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
                AW8697_BIT_TRGCFG1_TRG1_POLAR_MASK, AW8697_BIT_TRGCFG1_TRG1_POLAR_POS);
    }
    if(aw8697->trig[1].default_level) {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
                AW8697_BIT_TRGCFG1_TRG2_POLAR_MASK, AW8697_BIT_TRGCFG1_TRG2_POLAR_NEG);
    } else {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
                AW8697_BIT_TRGCFG1_TRG2_POLAR_MASK, AW8697_BIT_TRGCFG1_TRG2_POLAR_POS);
    }
    if(aw8697->trig[2].default_level) {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
                AW8697_BIT_TRGCFG1_TRG3_POLAR_MASK, AW8697_BIT_TRGCFG1_TRG3_POLAR_NEG);
    } else {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
                AW8697_BIT_TRGCFG1_TRG3_POLAR_MASK, AW8697_BIT_TRGCFG1_TRG3_POLAR_POS);
    }

    if(aw8697->trig[0].dual_edge) {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
                AW8697_BIT_TRGCFG1_TRG1_EDGE_MASK, AW8697_BIT_TRGCFG1_TRG1_EDGE_POS_NEG);
    } else {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
                AW8697_BIT_TRGCFG1_TRG1_EDGE_MASK, AW8697_BIT_TRGCFG1_TRG1_EDGE_POS);
    }
    if(aw8697->trig[1].dual_edge) {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
                AW8697_BIT_TRGCFG1_TRG2_EDGE_MASK, AW8697_BIT_TRGCFG1_TRG2_EDGE_POS_NEG);
    } else {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
                AW8697_BIT_TRGCFG1_TRG2_EDGE_MASK, AW8697_BIT_TRGCFG1_TRG2_EDGE_POS);
    }
    if(aw8697->trig[2].dual_edge) {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
                AW8697_BIT_TRGCFG1_TRG3_EDGE_MASK, AW8697_BIT_TRGCFG1_TRG3_EDGE_POS_NEG);
    } else {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
                AW8697_BIT_TRGCFG1_TRG3_EDGE_MASK, AW8697_BIT_TRGCFG1_TRG3_EDGE_POS);
    }

    if(aw8697->trig[0].frist_seq) {
        aw8697_i2c_write(aw8697, AW8697_REG_TRG1_WAV_P, aw8697->trig[0].frist_seq);
    }
    if(aw8697->trig[0].second_seq && aw8697->trig[0].dual_edge) {
        aw8697_i2c_write(aw8697, AW8697_REG_TRG1_WAV_N, aw8697->trig[0].second_seq);
    }
    if(aw8697->trig[1].frist_seq) {
        aw8697_i2c_write(aw8697, AW8697_REG_TRG2_WAV_P, aw8697->trig[1].frist_seq);
    }
    if(aw8697->trig[1].second_seq && aw8697->trig[1].dual_edge) {
        aw8697_i2c_write(aw8697, AW8697_REG_TRG2_WAV_N, aw8697->trig[1].second_seq);
    }
    if(aw8697->trig[2].frist_seq) {
        aw8697_i2c_write(aw8697, AW8697_REG_TRG3_WAV_P, aw8697->trig[1].frist_seq);
    }
    if(aw8697->trig[2].second_seq && aw8697->trig[2].dual_edge) {
        aw8697_i2c_write(aw8697, AW8697_REG_TRG3_WAV_N, aw8697->trig[1].second_seq);
    }

    return 0;
}

static int aw8697_haptic_trig_enable_config(struct aw8697 *aw8697)
{
    pr_debug("%s enter\n", __func__);
        aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG2,
            AW8697_BIT_TRGCFG2_TRG1_ENABLE_MASK, aw8697->trig[0].enable);
        aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG2,
            AW8697_BIT_TRGCFG2_TRG2_ENABLE_MASK, aw8697->trig[1].enable);
        aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG2,
            AW8697_BIT_TRGCFG2_TRG3_ENABLE_MASK, aw8697->trig[2].enable);
    return 0;
}
static int aw8697_haptic_auto_boost_config(struct aw8697 *aw8697, unsigned char flag)
{
    aw8697->auto_boost = flag;
    if(flag) {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
                AW8697_BIT_BST_AUTO_BST_AUTOSW_MASK, AW8697_BIT_BST_AUTO_BST_AUTOMATIC_BOOST);
    } else {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
                AW8697_BIT_BST_AUTO_BST_AUTOSW_MASK, AW8697_BIT_BST_AUTO_BST_MANUAL_BOOST);
    }
    return 0;
}

/*****************************************************
 *
 * vbat mode
 *
 *****************************************************/
static int aw8697_haptic_cont_vbat_mode(struct aw8697 *aw8697, unsigned char flag)
{
    if(flag == AW8697_HAPTIC_CONT_VBAT_HW_COMP_MODE) {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_ADCTEST,
                AW8697_BIT_ADCTEST_VBAT_MODE_MASK, AW8697_BIT_ADCTEST_VBAT_HW_COMP);
    } else {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_ADCTEST,
                AW8697_BIT_ADCTEST_VBAT_MODE_MASK, AW8697_BIT_ADCTEST_VBAT_SW_COMP);
    }
    return 0;
}

static int aw8697_haptic_get_vbat(struct aw8697 *aw8697)
{
    unsigned char reg_val = 0;
    unsigned int cont = 2000;

    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_EN);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
            AW8697_BIT_DETCTRL_VBAT_GO_MASK, AW8697_BIT_DETCTRL_VABT_GO_ENABLE);

    while(1){
        aw8697_i2c_read(aw8697, AW8697_REG_DETCTRL, &reg_val);
        if((reg_val & 0x02) == 0 || cont == 0)
             break;
        cont--;
    }

    aw8697_i2c_read(aw8697, AW8697_REG_VBATDET, &reg_val);
    aw8697->vbat = 6100 * reg_val / 256;
    if(aw8697->vbat > AW8697_VBAT_MAX) {
        aw8697->vbat = AW8697_VBAT_MAX;
        pr_debug("%s vbat max limit = %dmV\n", __func__, aw8697->vbat);
    }
    if(aw8697->vbat < AW8697_VBAT_MIN) {
        aw8697->vbat = AW8697_VBAT_MIN;
        pr_debug("%s vbat min limit = %dmV\n", __func__, aw8697->vbat);
    }

    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_OFF);

    return 0;
}

static int aw8697_haptic_ram_vbat_comp(struct aw8697 *aw8697, bool flag)
{
    int temp_gain = 0;

    if(flag) {
        if(aw8697->ram_vbat_comp == AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE) {
            aw8697_haptic_get_vbat(aw8697);
            temp_gain = aw8697->gain * AW8697_VBAT_REFER / aw8697->vbat;
            if(temp_gain > (128*AW8697_VBAT_REFER/AW8697_VBAT_MIN)) {
                temp_gain = 128*AW8697_VBAT_REFER/AW8697_VBAT_MIN;
                pr_debug("%s gain limit=%d\n", __func__, temp_gain);
            }
            aw8697_haptic_set_gain(aw8697, temp_gain);
        } else {
            aw8697_haptic_set_gain(aw8697, aw8697->gain);
        }
    } else {
        aw8697_haptic_set_gain(aw8697, aw8697->gain);
    }

    return 0;
}

/*****************************************************
 *
 * f0
 *
 *****************************************************/
static int aw8697_haptic_set_f0_preset(struct aw8697 *aw8697)
{
    unsigned int f0_reg = 0;

    pr_debug("%s enter\n", __func__);

    f0_reg = 1000000000/(aw8697->f0_pre*AW8697_HAPTIC_F0_COEFF);
    aw8697_i2c_write(aw8697, AW8697_REG_F_PRE_H, (unsigned char)((f0_reg>>8)&0xff));
    aw8697_i2c_write(aw8697, AW8697_REG_F_PRE_L, (unsigned char)((f0_reg>>0)&0xff));

    return 0;
}

static int haptic_read_f0_count_go(struct aw8697 *aw8697)
{
    int ret = 0;
    unsigned char reg_val = 0;
    unsigned int f0_reg = 0;
    unsigned long f0_tmp = 0;

    ret = aw8697_i2c_read(aw8697, AW8697_REG_F_LRA_CONT_H, &reg_val);
    f0_reg = (reg_val<<8);
    ret = aw8697_i2c_read(aw8697, AW8697_REG_F_LRA_CONT_L, &reg_val);
    f0_reg |= (reg_val<<0);
    if (!f0_reg) {
       pr_info("%s not get f0 because f0_reg value is 0!\n",__func__);
       return 0;
     }
     f0_tmp = 1000000000/(f0_reg*AW8697_HAPTIC_F0_COEFF);
     aw8697->f0 = (unsigned int)f0_tmp;
     aw8697->f0 += 30;
     return 0;
}

static int haptic_read_f0_count_default(struct aw8697 *aw8697)
{
    int ret = 0;
    unsigned char reg_val = 0;
    unsigned int f0_reg = 0;
    unsigned long f0_tmp = 0;

    ret = aw8697_i2c_read(aw8697, AW8697_REG_F_LRA_F0_H, &reg_val);
    f0_reg = (reg_val<<8);
    ret = aw8697_i2c_read(aw8697, AW8697_REG_F_LRA_F0_L, &reg_val);
    f0_reg |= (reg_val<<0);
    if (!f0_reg) {
       pr_info("%s not get f0 because f0_reg vaule is 0\n", __func__);
       return 0;
    }
    f0_tmp = 1000000000/(f0_reg*AW8697_HAPTIC_F0_COEFF);
    aw8697->f0 = (unsigned int)f0_tmp;
    return 0;
}

static int aw8697_haptic_read_f0(struct aw8697 *aw8697)
{

    pr_debug("%s enter\n", __func__);
    if (aw8697->count_go) {
       haptic_read_f0_count_go(aw8697);
    } else {
        haptic_read_f0_count_default(aw8697);
    }
    pr_info("%s f0=%d\n", __func__, aw8697->f0);

    return 0;
}

static int aw8697_haptic_read_cont_f0(struct aw8697 *aw8697)
{
    int ret = 0;
    unsigned char reg_val = 0;
    unsigned int f0_reg = 0;
    unsigned long f0_tmp = 0;

    pr_debug("%s enter\n", __func__);

    ret = aw8697_i2c_read(aw8697, AW8697_REG_F_LRA_CONT_H, &reg_val);
    f0_reg = (reg_val<<8);
    ret = aw8697_i2c_read(aw8697, AW8697_REG_F_LRA_CONT_L, &reg_val);
    f0_reg |= (reg_val<<0);
    if (!f0_reg) {
        pr_info("%s not get f0 because f0_reg vaule is 0\n", __func__);
        return 0;
    }
    f0_tmp = 1000000000/(f0_reg*AW8697_HAPTIC_F0_COEFF);
    aw8697->cont_f0 = (unsigned int)f0_tmp;
    pr_info("%s f0=%d\n", __func__, aw8697->cont_f0);

    return 0;
}

static int aw8697_haptic_read_beme(struct aw8697 *aw8697)
{
    int ret = 0;
    unsigned char reg_val = 0;

    ret = aw8697_i2c_read(aw8697, AW8697_REG_WAIT_VOL_MP, &reg_val);
    aw8697->max_pos_beme = (reg_val<<0);
    ret = aw8697_i2c_read(aw8697, AW8697_REG_WAIT_VOL_MN, &reg_val);
    aw8697->max_neg_beme = (reg_val<<0);

    pr_info("%s max_pos_beme=%d\n", __func__, aw8697->max_pos_beme);
    pr_info("%s max_neg_beme=%d\n", __func__, aw8697->max_neg_beme);

    return 0;
}

static int aw8697_haptic_read_bemf(struct aw8697 *aw8697)
{
    int ret = 0;
    unsigned char reg_val = 0;
    unsigned int bemf = 0;

    ret = aw8697_i2c_read(aw8697, AW8697_REG_BEMF_VOL_H, &reg_val);
    bemf |= (reg_val<<8);
    ret = aw8697_i2c_read(aw8697, AW8697_REG_BEMF_VOL_L, &reg_val);
    bemf |= (reg_val<<0);

    pr_info("%s bemf=%d\n", __func__, bemf);

    return 0;
}



/*****************************************************
 *
 * rtp
 *
 *****************************************************/
static void aw8697_haptic_set_rtp_aei(struct aw8697 *aw8697, bool flag)
{
    if(flag) {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
                AW8697_BIT_SYSINTM_FF_AE_MASK, AW8697_BIT_SYSINTM_FF_AE_EN);
    } else {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
                AW8697_BIT_SYSINTM_FF_AE_MASK, AW8697_BIT_SYSINTM_FF_AE_OFF);
    }
}

static unsigned char aw8697_haptic_rtp_get_fifo_afi(struct aw8697 *aw8697)
{
    unsigned char ret = 0;
    unsigned char reg_val = 0;

    if(aw8697->osc_cali_flag==1){
        aw8697_i2c_read(aw8697, AW8697_REG_SYSST, &reg_val);
        reg_val &= AW8697_BIT_SYSST_FF_AFS;
        ret = reg_val>>3;
    }else{
    aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
    reg_val &= AW8697_BIT_SYSINT_FF_AFI;
    ret = reg_val>>3;
    }

    return ret;
}

/*****************************************************
 *
 * rtp
 *
 *****************************************************/
static int aw8697_haptic_rtp_init(struct aw8697 *aw8697)
{
    unsigned int buf_len = 0;

    pr_info("%s enter\n", __func__);
    pm_qos_add_request(&pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY, PM_QOS_VALUE_VB);
    aw8697->rtp_cnt = 0;

    mutex_lock(&aw8697->rtp_lock);
    while((!aw8697_haptic_rtp_get_fifo_afi(aw8697)) &&
            (aw8697->play_mode == AW8697_HAPTIC_RTP_MODE)) {
        pr_info("%s rtp cnt = %d\n", __func__, aw8697->rtp_cnt);
        if((aw8697_rtp->len-aw8697->rtp_cnt) < (aw8697->ram.base_addr>>2)) {
            buf_len = aw8697_rtp->len-aw8697->rtp_cnt;
        } else {
            buf_len = (aw8697->ram.base_addr>>2);
        }
        aw8697_i2c_writes(aw8697, AW8697_REG_RTP_DATA,
                &aw8697_rtp->data[aw8697->rtp_cnt], buf_len);
        aw8697->rtp_cnt += buf_len;
        if(aw8697->rtp_cnt == aw8697_rtp->len || !buf_len) {
            pr_info("%s: rtp update complete,buf_len:%d\n", __func__, buf_len);
            aw8697->rtp_cnt = 0;
            mutex_unlock(&aw8697->rtp_lock);
            pm_qos_remove_request(&pm_qos_req_vb);
            return 0;
        }
    }
    mutex_unlock(&aw8697->rtp_lock);

    if(aw8697->play_mode == AW8697_HAPTIC_RTP_MODE) {
        aw8697_haptic_set_rtp_aei(aw8697, true);
    }

    pr_info("%s exit\n", __func__);
    pm_qos_remove_request(&pm_qos_req_vb);
    return 0;
}
static int aw8697_clock_OSC_trim_calibration(unsigned long int theory_time, unsigned long int real_time)
{
	unsigned int real_code = 0;
	unsigned int LRA_TRIM_CODE = 0;
	unsigned int DFT_LRA_TRIM_CODE = 0;
	unsigned int Not_need_cali_threshold = 10;/*0.1 percent not need calibrate*/

	if(theory_time == real_time ) {
		pr_info("aw_osctheory_time == real_time:%ld  theory_time = %ld not need to cali\n",real_time,theory_time);
		return 0;
	} else if(theory_time < real_time ){
		if((real_time - theory_time) > (theory_time/50 ))
		{
			pr_info("aw_osc(real_time - theory_time) > (theory_time/50 ) not to cali\n");
			return DFT_LRA_TRIM_CODE;
		}

		if ((real_time - theory_time) < (Not_need_cali_threshold*theory_time/10000))
		{
			pr_info("aw_oscmicrosecond:%ld  theory_time = %ld not need to cali\n",real_time,theory_time);
			return DFT_LRA_TRIM_CODE;
		}

		real_code = ((real_time - theory_time)* 4000) / theory_time;
		real_code = ((real_code%10 < 5)? 0 : 1) + real_code/10;
		real_code = 32 + real_code;
	} else if(theory_time > real_time){
		if(( theory_time - real_time) > (theory_time/50 ))
		{
			pr_info("aw_osc(( theory_time - real_time) > (theory_time/50 ))  not to cali \n");
			return DFT_LRA_TRIM_CODE;
		}
		if ((theory_time - real_time) < (Not_need_cali_threshold*theory_time/10000))
		{
			pr_info("aw_oscmicrosecond:%ld  theory_time = %ld not need to cali\n",real_time,theory_time);
			return DFT_LRA_TRIM_CODE;
		}
		real_code = ((theory_time - real_time)* 4000) / theory_time ;
		real_code = ((real_code%10 < 5)? 0 : 1) + real_code/10;
		real_code = 32 - real_code;
	}
	if (real_code>31)
		LRA_TRIM_CODE = real_code -32;
	else
		LRA_TRIM_CODE = real_code +32;
	pr_info("aw_oscmicrosecond:%ld  theory_time = %ld real_code =0X%02X LRA_TRIM_CODE 0X%02X\n",real_time,theory_time,real_code,LRA_TRIM_CODE);

	return LRA_TRIM_CODE;
}

static int aw8697_rtp_trim_lra_calibration(struct aw8697 *aw8697)
{
    unsigned char reg_val = 0;
    unsigned int fre_val = 0;
    unsigned int theory_time = 0;
    unsigned int lra_rtim_code = 0;

    aw8697_i2c_read(aw8697, AW8697_REG_PWMDBG, &reg_val);
    fre_val = (reg_val & 0x006f )>> 5;

    if(fre_val == 3)
      theory_time = (aw8697->rtp_len / 12000) * 1000000; /*12K */
    if(fre_val == 2)
      theory_time = (aw8697->rtp_len / 24000) * 1000000; /*24K */
    if(fre_val == 1 || fre_val == 0)
      theory_time = (aw8697->rtp_len / 48000) * 1000000; /*48K */

    printk("microsecond:%ld  theory_time = %d\n",aw8697->microsecond,theory_time);

    lra_rtim_code = aw8697_clock_OSC_trim_calibration(theory_time,aw8697->microsecond);
    if(lra_rtim_code > 0 ) {
        aw8697->lra_calib_data = lra_rtim_code;
        aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, (char)lra_rtim_code);
    }
    return 0;
}
static unsigned char aw8697_haptic_osc_read_int(struct aw8697 *aw8697)
{
    unsigned char reg_val = 0;
    aw8697_i2c_read(aw8697, AW8697_REG_DBGSTAT, &reg_val);
    return reg_val;
}
static int aw8697_rtp_osc_calibration(struct aw8697 *aw8697)
{
    const struct firmware *rtp_file;
    int ret = -1;
    unsigned int buf_len = 0;
    unsigned char osc_int_state = 0;
    aw8697->rtp_cnt = 0;
    aw8697->timeval_flags = 1;
    aw8697->osc_cali_flag =1;

    pr_info("%s enter\n", __func__);
    /* fw loaded */
    ret = request_firmware(&rtp_file,
            aw8697_rtp_name[/*aw8697->rtp_file_num*/ 0],
            aw8697->dev);
    if(ret < 0)
    {
        pr_err("%s: failed to read %s\n", __func__,
                aw8697_rtp_name[/*aw8697->rtp_file_num*/ 0]);
        return ret;
    }
    /*op add stop,for irq interrupt during calibrate*/
    aw8697_haptic_stop(aw8697);
    aw8697->rtp_init = 0;
    mutex_lock(&aw8697->rtp_lock);
    vfree(aw8697_rtp);
    aw8697_rtp = vmalloc(rtp_file->size+sizeof(int));
    if (!aw8697_rtp) {
        release_firmware(rtp_file);
        mutex_unlock(&aw8697->rtp_lock);
        pr_err("%s: error allocating memory\n", __func__);
        return -1;
    }
    aw8697_rtp->len = rtp_file->size;
    aw8697->rtp_len = rtp_file->size;
    pr_info("%s: rtp file [%s] size = %d\n", __func__,
            aw8697_rtp_name[/*aw8697->rtp_file_num*/ 0], aw8697_rtp->len);
    memcpy(aw8697_rtp->data, rtp_file->data, rtp_file->size);
    release_firmware(rtp_file);
    mutex_unlock(&aw8697->rtp_lock);

    /* gain */
    aw8697_haptic_ram_vbat_comp(aw8697, false);

    /* rtp mode config */
    aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RTP_MODE);

    aw8697_i2c_write_bits(aw8697, AW8697_REG_DBGCTRL,
            AW8697_BIT_DBGCTRL_INT_MODE_MASK, AW8697_BIT_DBGCTRL_INT_MODE_EDGE);
    disable_irq(gpio_to_irq(aw8697->irq_gpio));
    /* haptic start */
    aw8697_haptic_start(aw8697);
    pm_qos_add_request(&pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY, PM_QOS_VALUE_VB);
    while(1) {
        if(!aw8697_haptic_rtp_get_fifo_afi(aw8697)) {
            pr_info("%s !aw8697_haptic_rtp_get_fifo_afi done aw8697->rtp_cnt= %d \n", __func__,aw8697->rtp_cnt);
            mutex_lock(&aw8697->rtp_lock);
            if((aw8697_rtp->len-aw8697->rtp_cnt) < (aw8697->ram.base_addr>>2))
                buf_len = aw8697_rtp->len-aw8697->rtp_cnt;
            else
                buf_len = (aw8697->ram.base_addr>>2);
            if (aw8697->rtp_cnt != aw8697_rtp->len) {
                if(aw8697->timeval_flags ==1) {
                    do_gettimeofday(&aw8697->start);
                    aw8697->timeval_flags = 0;
                }
                aw8697_i2c_writes(aw8697, AW8697_REG_RTP_DATA,&aw8697_rtp->data[aw8697->rtp_cnt], buf_len);
                aw8697->rtp_cnt += buf_len;
              }
             mutex_unlock(&aw8697->rtp_lock);
        }
        osc_int_state = aw8697_haptic_osc_read_int(aw8697);
        if(osc_int_state&AW8697_BIT_SYSINT_DONEI) {
            do_gettimeofday(&aw8697->end);
            pr_info("%s vincent playback done aw8697->rtp_cnt= %d \n", __func__,aw8697->rtp_cnt);
            break;
        }

        do_gettimeofday(&aw8697->end);
        aw8697->microsecond = (aw8697->end.tv_sec - aw8697->start.tv_sec)*1000000 +
        (aw8697->end.tv_usec - aw8697->start.tv_usec);
        if (aw8697->microsecond > OP_OCS_CALIBRATION_T_LENGTH) {
            pr_info("%s vincent time out aw8697->rtp_cnt %d osc_int_state %02x\n", __func__,aw8697->rtp_cnt, osc_int_state);
            break;
          }
    }
    pm_qos_remove_request(&pm_qos_req_vb);
    enable_irq(gpio_to_irq(aw8697->irq_gpio));

    aw8697->osc_cali_flag =0;
    aw8697->microsecond = (aw8697->end.tv_sec - aw8697->start.tv_sec)*1000000 +
        (aw8697->end.tv_usec - aw8697->start.tv_usec);
    /*calibration osc*/
    pr_info("%s 2018_microsecond:%ld \n",__func__,aw8697->microsecond);
    pr_info("%s exit\n", __func__);
    return 0;
}

static int aw8697_haptic_osc_set_default_trim(struct aw8697 *aw8697)
{
    pr_info("%s enter\n", __func__);
    aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA,AW8697_TRIM_DEFAULT);
    return 0;
}

static void aw8697_op_clean_status(struct aw8697 *aw8697)
{
	aw8697->audio_ready = false;
	aw8697->haptic_ready = false;
	aw8697->pre_haptic_number = 0;
	aw8697->rtp_routine_on = 0;
	aw8697->rtp_on = 0;
	pr_info("%s enter\n", __FUNCTION__);
}

static void aw8697_rtp_work_routine(struct work_struct *work)
{
    const struct firmware *rtp_file;
    int ret = -1;
    struct aw8697 *aw8697 = container_of(work, struct aw8697, rtp_work);

    pr_info("%s enter\n", __func__);
    if (aw8697->rtp_file_num == 0) {
        pr_info("rtp_file_num return\n");
        return;
    }
    aw8697->rtp_routine_on = 1;
    /* fw loaded */
    ret = request_firmware(&rtp_file,
            aw8697_rtp_name[aw8697->rtp_file_num],
            aw8697->dev);
    if(ret < 0)
    {
        pr_err("%s: failed to read %s\n", __func__,
                aw8697_rtp_name[aw8697->rtp_file_num]);
        aw8697->rtp_routine_on = 0;
        return;
    }
    aw8697->rtp_init = 0;
    mutex_lock(&aw8697->rtp_lock);
    vfree(aw8697_rtp);
    aw8697_rtp = vmalloc(rtp_file->size+sizeof(int));
    if (!aw8697_rtp) {
        release_firmware(rtp_file);
        pr_err("%s: error allocating memory\n", __func__);
        aw8697_op_clean_status(aw8697);
        aw8697->rtp_routine_on = 0;
        mutex_unlock(&aw8697->rtp_lock);
        return;
    }
    aw8697_rtp->len = rtp_file->size;
    pr_info("%s: rtp file [%s] size = %d\n", __func__,
            aw8697_rtp_name[aw8697->rtp_file_num], aw8697_rtp->len);
    memcpy(aw8697_rtp->data, rtp_file->data, rtp_file->size);
    mutex_unlock(&aw8697->rtp_lock);
    release_firmware(rtp_file);

    mutex_lock(&aw8697->lock);
    aw8697->rtp_init = 1;
    /* gain */
    aw8697_haptic_ram_vbat_comp(aw8697, false);

    /* rtp mode config */
    aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RTP_MODE);

    aw8697_i2c_write_bits(aw8697, AW8697_REG_DBGCTRL,
            AW8697_BIT_DBGCTRL_INT_MODE_MASK, AW8697_BIT_DBGCTRL_INT_MODE_EDGE);
    /* haptic start */
    aw8697_haptic_start(aw8697);
    aw8697_haptic_rtp_init(aw8697);
    mutex_unlock(&aw8697->lock);
    aw8697->rtp_routine_on = 0;
}

/*****************************************************
 *
 * haptic - audio
 *
 *****************************************************/
static int aw8697_haptic_audio_tp_list_match(struct shake_point *pt_info, struct haptic_audio_trust_zone *p_tmp)
{
	if ((pt_info->x+AW8697_HAPTIC_AI_X_JITTER*p_tmp->w/AW8697_HAPTIC_AI_X_DFT_W > p_tmp->x) &&
		(pt_info->x < p_tmp->x+p_tmp->w+AW8697_HAPTIC_AI_X_JITTER*p_tmp->w/AW8697_HAPTIC_AI_X_DFT_W) &&
		(pt_info->y+AW8697_HAPTIC_AI_Y_JITTER*p_tmp->h/AW8697_HAPTIC_AI_Y_DFT_H> p_tmp->y) &&
		(pt_info->y < p_tmp->y+p_tmp->h+AW8697_HAPTIC_AI_Y_JITTER*p_tmp->h/AW8697_HAPTIC_AI_Y_DFT_H)) {

#ifdef SECOND_SCOREMODE
		//p_tmp->level = (p_tmp->level < TRUST_LEVEL) ? (++p_tmp->level) : p_tmp->level;
		if(p_tmp->level >= TRUST_LEVEL){
			return 1;
		}else{
			return 2;
		}
#else
		return 1;
#endif
	} else {
		return 0;
	}
}

static int aw8697_haptic_audio_tz_list_match(struct haptic_audio_trust_zone *p_tmp, struct trust_zone_info *p_tmp_new)
{
	int match = 0;
	int deta_x = 0;
	int deta_y = 0;
	int deta_x_w = 0;
	int deta_y_h = 0;
	unsigned int h = (p_tmp->h > p_tmp_new->h) ? p_tmp_new->h : p_tmp->h;
	unsigned int w = (p_tmp->w > p_tmp_new->w) ? p_tmp_new->w : p_tmp->w;

	deta_x = p_tmp->x - p_tmp_new->x;
	deta_y = p_tmp->y - p_tmp_new->y;
	deta_x_w = (p_tmp->x+p_tmp->w)-(p_tmp_new->x+p_tmp_new->w);
	deta_y_h = (p_tmp->y+p_tmp->h)-(p_tmp_new->y+p_tmp_new->h);

	if((ABS(deta_x)<AW8697_HAPTIC_AI_X_JITTER*w*2/AW8697_HAPTIC_AI_X_DFT_W) &&
		(ABS(deta_y)<AW8697_HAPTIC_AI_X_JITTER*h*2/AW8697_HAPTIC_AI_Y_DFT_H) &&
		(ABS(deta_y_h)<AW8697_HAPTIC_AI_X_JITTER*h*2/AW8697_HAPTIC_AI_Y_DFT_H) &&
		(ABS(deta_x_w)<AW8697_HAPTIC_AI_X_JITTER*w*2/AW8697_HAPTIC_AI_X_DFT_W)){
		match = 1;
	}else{
		match = 0;
	}
	pr_debug("match = %d,(%d,%d)(%d,%d),(%d,%d)(%d,%d),deta_x = %d,deta_y = %d,deta_x_w = %d,deta_y_h = %d\n",
	match,p_tmp->x,p_tmp->y,p_tmp->x+p_tmp->w,p_tmp->y+p_tmp->h,p_tmp_new->x,
	p_tmp_new->y,p_tmp_new->x+p_tmp_new->w,p_tmp_new->y+p_tmp_new->h,deta_x,deta_y,deta_x_w,deta_y_h);
	return match;
}

static int aw8697_haptic_audio_tz_list_show(struct haptic_audio *haptic_audio)
{
	struct haptic_audio_trust_zone *p_tmp = NULL;
	unsigned int i = 0;

	list_for_each_entry(p_tmp, &haptic_audio->list, list) {
		pr_debug("%s: tz[%02d]: [%01d, %04d, %04d, %04d, %04d]\n",
			__func__, i, p_tmp->level, p_tmp->x, p_tmp->y,
			p_tmp->w, p_tmp->h);
		i++;

	}
	return 0;
}

static int aw8697_haptic_audio_tz_score_list_clear(struct haptic_audio *haptic_audio)
{
	struct haptic_audio_trust_zone *p_tmp = NULL;
	struct haptic_audio_trust_zone *p_tmp_bak = NULL;

	list_for_each_entry_safe(p_tmp, p_tmp_bak, &(haptic_audio->score_list), list) {
		list_del(&p_tmp->list);
		kfree(p_tmp);
	}

	return 0;
}

static int aw8697_haptic_audio_tz_list_insert(
	struct haptic_audio *haptic_audio, struct trust_zone_info *tz_info)
{
	struct haptic_audio_trust_zone *p_new = NULL;

	pr_debug("%s: enter\n", __func__);

	p_new = (struct haptic_audio_trust_zone *)kzalloc(
		sizeof(struct haptic_audio_trust_zone), GFP_KERNEL);
	if (p_new == NULL ) {
		pr_err("%s: kzalloc memory fail\n", __func__);
		return -1;
	}

	/* update new list info */
	p_new->level = tz_info->level;
	p_new->cnt = haptic_audio->tz_cnt_thr * 2;
	p_new->x = tz_info->x;
	p_new->y = tz_info->y;
	p_new->w = tz_info->w;
	p_new->h = tz_info->h;
	p_new->dirty = 0;

	INIT_LIST_HEAD(&(p_new->list));

#ifdef SECOND_SCOREMODE
	list_add(&(p_new->list), &(haptic_audio->list));
	haptic_audio->tz_num = haptic_audio->tz_num +1;
#else
	if(haptic_audio->tz_init){
		list_add(&(p_new->list), &(haptic_audio->score_list));
		haptic_audio->tz_num += 1;
		pr_debug("%s scorelist_num = %d\n", __func__, haptic_audio->tz_num);
	}else
		list_add(&(p_new->list), &(haptic_audio->list));
#endif

	return 0;
}

static int aw8697_haptic_audio_ctr_list_insert(
	struct haptic_audio *haptic_audio, struct haptic_ctr *haptic_ctr)
{
	struct haptic_ctr *p_new = NULL;

	p_new = (struct haptic_ctr *)kzalloc(
		sizeof(struct haptic_ctr), GFP_KERNEL);
	if (p_new == NULL ) {
		pr_err("%s: kzalloc memory fail\n", __func__);
		return -1;
	}
	/* update new list info */
	p_new->cnt = haptic_ctr->cnt;
	p_new->cmd = haptic_ctr->cmd;
	p_new->play = haptic_ctr->play;
	p_new->wavseq = haptic_ctr->wavseq;
	p_new->loop = haptic_ctr->loop;
	p_new->gain = haptic_ctr->gain;

	INIT_LIST_HEAD(&(p_new->list));
	list_add(&(p_new->list), &(haptic_audio->ctr_list));

	return 0;
}


static int aw8697_haptic_audio_ctr_list_clear(struct haptic_audio *haptic_audio)
{
	struct haptic_ctr *p_ctr = NULL;
	struct haptic_ctr *p_ctr_bak = NULL;

	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
		list_del(&p_ctr->list);
		kfree(p_ctr);
	}

	return 0;
}

static int aw8697_fb_notifier_callback_tp(struct notifier_block *self, unsigned long event, void *data)
{
	struct aw8697 *aw8697 = container_of(self, struct aw8697, fb_notif);
	struct tp *tp = &(aw8697->haptic_audio.tp);
	int i = 0;
	int *blank;
	struct fb_event *evdata = data;
	struct haptic_audio *haptic_audio = NULL;
	struct haptic_audio_trust_zone *p_tmp = NULL;

	haptic_audio = &(aw8697->haptic_audio);

	blank = evdata->data;
	pr_debug("%s: tp event = %ld, blank = %d\n", __func__, event, *blank);
	i= *blank;
#if 0
	if (event == 11) {
		pr_debug("%s: tp down\n", __func__);
			pr_debug("%s: tp record_point_down[%d].status = %d\n", __func__, i, record_point[i].status);
			if((AW8697_HAPTIC_TP_ST_PRESS == record_point[i].status) &&
				(record_point[i].status != tp->id[i].pt_info.status)) {
				tp->id[i].pt_info.status = record_point[i].status;
				tp->id[i].tp_flag = AW8697_HAPTIC_TP_PRESS;
				tp->id[i].press_flag = AW8697_HAPTIC_TP_PRESS;
				tp->id[i].release_flag = AW8697_HAPTIC_TP_NULL;
				tp->id[i].no_play_cnt = 0;
				do_gettimeofday(&tp->id[i].t_press);
				pr_info("%s: tp_press_release: status=%d, flag=%d", __func__, tp->id[i].pt_info.status, tp->id[i].tp_flag);
			}
	}
	if (event == 10) {
		pr_debug("%s: tp up\n", __func__);
			pr_debug("%s: tp record_point_up[%d].status = %d\n", __func__, i, record_point[i].status);
			if((AW8697_HAPTIC_TP_ST_RELEASE == record_point[i].status) &&
				(record_point[i].status != tp->id[i].pt_info.status)) {
				tp->id[i].pt_info.status = record_point[i].status;
				tp->id[i].tp_flag = AW8697_HAPTIC_TP_RELEASE;
				tp->id[i].release_flag = AW8697_HAPTIC_TP_RELEASE;
				do_gettimeofday(&tp->id[i].t_release);
				pr_info("%s: tp_press_release: status=%d, flag=%d", __func__, tp->id[i].pt_info.status, tp->id[i].tp_flag);
			}
	}
#endif
	if (event == 0) {
		list_for_each_entry(p_tmp, &(haptic_audio->list), list) {
			if ((tp->id[i].pt_info.x+AW8697_HAPTIC_AI_X_JITTER*p_tmp->w/AW8697_HAPTIC_AI_X_DFT_W > p_tmp->x) &&
				(tp->id[i].pt_info.x < p_tmp->x+p_tmp->w+AW8697_HAPTIC_AI_X_JITTER*p_tmp->w/AW8697_HAPTIC_AI_X_DFT_W) &&
				(tp->id[i].pt_info.y+AW8697_HAPTIC_AI_Y_JITTER*p_tmp->h/AW8697_HAPTIC_AI_Y_DFT_H > p_tmp->y) &&
				(tp->id[i].pt_info.y < p_tmp->y+p_tmp->h+AW8697_HAPTIC_AI_Y_JITTER*p_tmp->h/AW8697_HAPTIC_AI_Y_DFT_H)) {
				pr_debug("%s: tp input point[%d, %04d, %04d] up is in the ai trust zone[%d, %04d, %04d, %04d, %04d]\n",
					__func__, tp->id[i].pt_info.id, tp->id[i].pt_info.x, tp->id[i].pt_info.y,
					p_tmp->level, p_tmp->x, p_tmp->y, p_tmp->w, p_tmp->h);
				if(tp->virtual_id == i) {
					tp->id[AW8697_HAPTIC_TP_ID_MAX].pt_info.id = tp->id[i].pt_info.id;
					tp->id[AW8697_HAPTIC_TP_ID_MAX].pt_info.x = tp->id[i].pt_info.x;
					tp->id[AW8697_HAPTIC_TP_ID_MAX].pt_info.y = tp->id[i].pt_info.y;
					tp->id[AW8697_HAPTIC_TP_ID_MAX].pt_info.status = AW8697_HAPTIC_TP_ST_RELEASE;
					tp->id[AW8697_HAPTIC_TP_ID_MAX].tp_flag = AW8697_HAPTIC_TP_RELEASE;
					tp->id[AW8697_HAPTIC_TP_ID_MAX].release_flag = AW8697_HAPTIC_TP_RELEASE;
					do_gettimeofday(&tp->id[AW8697_HAPTIC_TP_ID_MAX].t_release);
				}
				break;
			}
		}
		tp->id[i].pt_info.status = AW8697_HAPTIC_TP_ST_RELEASE;
		tp->id[i].tp_flag = AW8697_HAPTIC_TP_RELEASE;
		tp->id[i].release_flag = AW8697_HAPTIC_TP_RELEASE;
		do_gettimeofday(&tp->id[i].t_release);

		pr_debug("%s: tp input point[%d, %04d, %04d] up to [%d, %04d, %04d] \n",
			__func__, i, tp->id[i].pt_info.x, tp->id[i].pt_info.y,
			i, tp->id[i].pt_info.x, tp->id[i].pt_info.y);
	}

	return 0;
}

#ifdef OP_AW_DEBUG
static int aw8697_haptic_audio_init(struct aw8697 *aw8697)
{
	unsigned int i = 0;

	pr_debug("%s enter\n", __func__);

	aw8697_haptic_set_wav_seq(aw8697, 0x01, 0x00);

	for (i=0; i<AW8697_HAPTIC_TP_ID_MAX+1; i++) {
		//aw8697->haptic_audio.tp.id[i].tp_flag = AW8697_HAPTIC_TP_NULL;
		//aw8697->haptic_audio.tp.id[i].press_flag = AW8697_HAPTIC_TP_NULL;
		//aw8697->haptic_audio.tp.id[i].release_flag = AW8697_HAPTIC_TP_NULL;
		//aw8697->haptic_audio.tp.id[i].play_flag = AW8697_HAPTIC_TP_PLAY_NULL;
		aw8697->haptic_audio.tp.id[i].tp_ai_match_flag = 1;
		aw8697->haptic_audio.tp.id[i].no_play_cnt = 0;
	}
	//aw8697->haptic_audio.tp.play_flag = AW8697_HAPTIC_TP_PLAY_NULL;
	//aw8697->haptic_audio.tp.press_flag = AW8697_HAPTIC_TP_NULL;
	//aw8697->haptic_audio.tp.tp_ai_match_flag = 0;
	//aw8697->haptic_audio.tp.tp_ai_check_flag = 0;
	aw8697->haptic_audio.tp.hap_match_without_tz_cnt = 0;
	aw8697->haptic_audio.uevent_report_flag = 0;
	aw8697->haptic_audio.hap_cnt_outside_tz = 0;
	return 0;
}

static int aw8697_haptic_audio_off(struct aw8697 *aw8697)
{
	pr_debug("%s enter\n", __func__);

	mutex_lock(&aw8697->lock);
	aw8697_haptic_set_gain(aw8697, 0x80);
	aw8697_haptic_stop(aw8697);
	aw8697_haptic_audio_ctr_list_clear(&aw8697->haptic_audio);
	mutex_unlock(&aw8697->lock);

	return 0;
}

static int aw8697_haptic_audio_stop(struct aw8697 *aw8697)
{
    pr_debug("%s enter\n", __func__);
#ifdef OP_AW_DEBUG
		do_gettimeofday(&aw8697->t_stop);
#endif

    aw8697_haptic_play_go(aw8697, false);
    aw8697_haptic_stop_delay(aw8697);
    aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_STANDBY_MODE);

    return 0;
}



static int aw8697_haptic_audio_play_cfg(struct aw8697 *aw8697)
{
	pr_debug("%s enter\n", __func__);

	aw8697->play_mode = AW8697_HAPTIC_RAM_MODE;
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_PLAY_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
			AW8697_BIT_SYSINTM_UVLO_MASK, AW8697_BIT_SYSINTM_UVLO_OFF);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_WORK_MODE_MASK, AW8697_BIT_SYSCTRL_ACTIVE);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BOOST);

	return 0;
}
#endif
static enum hrtimer_restart aw8697_haptic_audio_timer_func(struct hrtimer *timer)
{
    struct aw8697 *aw8697 = container_of(timer, struct aw8697, haptic_audio.timer);

    pr_debug("%s enter\n", __func__);
    schedule_work(&aw8697->haptic_audio.work);

    hrtimer_start(&aw8697->haptic_audio.timer,
            ktime_set(aw8697->haptic_audio.timer_val/1000000,
                    (aw8697->haptic_audio.timer_val%1000000)*1000),
            HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

static void aw8697_haptic_audio_work_routine(struct work_struct *work)
{
	struct aw8697 *aw8697 = container_of(work, struct aw8697, haptic_audio.work);
	struct tp *tp = &(aw8697->haptic_audio.tp);
	struct haptic_audio *haptic_audio = NULL;
	struct haptic_audio_trust_zone *p_tmp = NULL;
	struct haptic_audio_trust_zone *p_tmp_r = NULL;
	struct haptic_audio_trust_zone *p_tmp_l = NULL;
	struct haptic_ctr *p_ctr = NULL;
	struct haptic_ctr *p_ctr_bak = NULL;
	struct timeval tmp_time;
	unsigned int press_delay = 0;
	unsigned int release_delay = 0;
	unsigned int tp_touch_play_flag = 0;
	unsigned int i = 0;
	unsigned int touch_vibrator_id = 0;
	unsigned int ctr_list_flag = 0;
	unsigned int ctr_list_input_cnt = 0;
	unsigned int ctr_list_output_cnt = 0;
	unsigned int ctr_list_diff_cnt = 0;
	unsigned int ctr_list_del_cnt = 0;
	unsigned int tp_ai_match_id = 0;
	unsigned int x_thr = aw8697->haptic_audio.tp_size.x>>1;

	/*OP add for juge rtp on begin*/
	int rtp_is_going_on = 0;
	int match_re = 0, dirty = 0;

	pr_debug("%s enter\n", __func__);
	/*OP add for juge rtp on end*/

	haptic_audio = &(aw8697->haptic_audio);

	mutex_lock(&aw8697->haptic_audio.lock);
	memset(&aw8697->haptic_audio.ctr, 0,
		sizeof(struct haptic_ctr));
	ctr_list_flag = 0;
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
		ctr_list_flag = 1;
		break;
	}
	if(ctr_list_flag == 0) {
		pr_debug("%s: ctr list empty\n", __func__);
	}
	if(ctr_list_flag == 1) {
		list_for_each_entry_safe(p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
			ctr_list_input_cnt =  p_ctr->cnt;
			break;
		}
		list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
			ctr_list_output_cnt =  p_ctr->cnt;
			break;
		}
		if(ctr_list_input_cnt > ctr_list_output_cnt) {
			ctr_list_diff_cnt = ctr_list_input_cnt - ctr_list_output_cnt;
		}
		if(ctr_list_input_cnt < ctr_list_output_cnt) {
			ctr_list_diff_cnt = 32 + ctr_list_input_cnt - ctr_list_output_cnt;
		}
		if(ctr_list_diff_cnt > 2) {
			list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
				if((p_ctr->play == 0) &&
					(AW8697_HAPTIC_CMD_ENABLE == (AW8697_HAPTIC_CMD_HAPTIC & p_ctr->cmd))) {
					list_del(&p_ctr->list);
					kfree(p_ctr);
					ctr_list_del_cnt ++;
				}
				if(ctr_list_del_cnt == ctr_list_diff_cnt) {
					break;
				}
			}
		}

	}

	/* get the last data from list */
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
		aw8697->haptic_audio.ctr.cnt = p_ctr->cnt;
		aw8697->haptic_audio.ctr.cmd = p_ctr->cmd;
		aw8697->haptic_audio.ctr.play = p_ctr->play;
		aw8697->haptic_audio.ctr.wavseq = p_ctr->wavseq;
		aw8697->haptic_audio.ctr.loop = p_ctr->loop;
		aw8697->haptic_audio.ctr.gain = p_ctr->gain;
		list_del(&p_ctr->list);
		kfree(p_ctr);
		break;
	}
	if(aw8697->haptic_audio.ctr.play) {
		pr_debug("%s: cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d\n",
			__func__,
			aw8697->haptic_audio.ctr.cnt,
			aw8697->haptic_audio.ctr.cmd,
			aw8697->haptic_audio.ctr.play,
			aw8697->haptic_audio.ctr.wavseq,
			aw8697->haptic_audio.ctr.loop,
			aw8697->haptic_audio.ctr.gain);
	}

	/* rtp mode jump */
	rtp_is_going_on = aw8697_haptic_juge_RTP_is_going_on(aw8697);
	if (rtp_is_going_on) {
		mutex_unlock(&aw8697->haptic_audio.lock);
		return;
	}
	/* haptic play with tp adjust info */
	if(AW8697_HAPTIC_CMD_TP ==
		(aw8697->haptic_audio.ctr.cmd & AW8697_HAPTIC_CMD_SYS)) {
		do_gettimeofday(&tmp_time);
		for(i=0; i<AW8697_HAPTIC_TP_ID_MAX+1; i++) {
			/* get tp press delay and tp release delay */
			press_delay = 0;
			release_delay = 0;
			if(AW8697_HAPTIC_TP_PRESS == tp->id[i].press_flag) {
				press_delay = (tmp_time.tv_sec-tp->id[i].t_press.tv_sec)*1000000 +
					(tmp_time.tv_usec-tp->id[i].t_press.tv_usec);
			}
			if(AW8697_HAPTIC_TP_RELEASE == tp->id[i].release_flag) {
				release_delay = (tmp_time.tv_sec-tp->id[i].t_release.tv_sec)*1000000 +
					(tmp_time.tv_usec-tp->id[i].t_release.tv_usec);
			}
			if(tp->id[i].press_flag || tp->id[i].release_flag) {
				pr_debug("%s: id[%d]: press_flag=%d, press_delay=%dus, release_flag=%d, release_delaly=%dus\n",
					__func__, i, tp->id[i].press_flag, press_delay, tp->id[i].release_flag, release_delay);
			}
			/* adjust tp play with tp press delay and tp release delay */
			if(AW8697_HAPTIC_PLAY_ENABLE == aw8697->haptic_audio.ctr.play) {
				if(AW8697_HAPTIC_TP_PRESS == tp->id[i].press_flag) {
					if(AW8697_HAPTIC_TP_PLAY_NULL == tp->id[i].play_flag) {
						if(press_delay > tp->press_delay_max) {
							/* no play time > tp-play delay max time */
							tp->id[i].play_flag = AW8697_HAPTIC_TP_PLAY_NOMORE;
							tp->id[i].press_flag = AW8697_HAPTIC_TP_NULL;
							tp->id[i].release_flag = AW8697_HAPTIC_TP_NULL;
							if(i == AW8697_HAPTIC_TP_ID_MAX) {
								tp->id[i].pt_info.status = AW8697_HAPTIC_TP_ST_RELEASE;
								tp->id[i].pt_info.touch_flag = AW8697_HAPTIC_TP_TOUCH_INVAIL;
								tp->id[i].tp_flag = AW8697_HAPTIC_TP_NULL;
								tp->id[i].press_flag = AW8697_HAPTIC_TP_NULL;
								tp->id[i].release_flag = AW8697_HAPTIC_TP_NULL;
								tp->id[i].no_play_cnt = 0;
								//tp->id[i].press_no_vibrate_flag = 0;
							}
							if(tp->id[i].press_no_vibrate_flag == 0)
								tp->id[i].press_no_vibrate_flag = 2;
						} else {
							if(press_delay > tp->press_delay_min) {
								/* tp-play delay min time < no play time < tp-play delay max time */
								tp->id[i].play_flag = AW8697_HAPTIC_TP_PLAY_ENABLE;
								tp->id[i].press_no_vibrate_flag = 1;
							} else {
								/* no play time < tp-play delay min time */
								tp->id[i].play_flag = AW8697_HAPTIC_TP_PLAY_NOMORE;
								if(tp->id[i].press_no_vibrate_flag == 0)
									tp->id[i].press_no_vibrate_flag = 2;
							}
						}
					} else if(AW8697_HAPTIC_TP_PLAY_ENABLE == tp->id[i].play_flag) {
					}
					pr_debug("%s: %d: id[%d]:play_flag=%d, press_flag=%d, release_flag=%d, press_no_vibrate_flag=%d, no_play_cnt=%d\n",
						__func__, __LINE__, i, tp->id[i].play_flag, tp->id[i].press_flag, tp->id[i].release_flag,
						tp->id[i].press_no_vibrate_flag, tp->id[i].no_play_cnt);
				}
				if(AW8697_HAPTIC_TP_RELEASE == tp->id[i].release_flag) {
					if(release_delay > tp->release_delay_max) {
						/* tp release time > 3-continue-time play time*/
						if(AW8697_HAPTIC_TP_PLAY_ENABLE == tp->id[i].play_flag) {
							tp->id[i].play_flag = AW8697_HAPTIC_TP_PLAY_NOMORE;
							tp->id[i].press_flag = AW8697_HAPTIC_TP_NULL;
							tp->id[i].release_flag = AW8697_HAPTIC_TP_NULL;
							if(i == AW8697_HAPTIC_TP_ID_MAX) {
								tp->id[i].pt_info.status = AW8697_HAPTIC_TP_ST_RELEASE;
								tp->id[i].pt_info.touch_flag = AW8697_HAPTIC_TP_TOUCH_INVAIL;
								tp->id[i].tp_flag = AW8697_HAPTIC_TP_NULL;
								tp->id[i].press_flag = AW8697_HAPTIC_TP_NULL;
								tp->id[i].release_flag = AW8697_HAPTIC_TP_NULL;
								tp->id[i].no_play_cnt = 0;
								//tp->id[i].press_no_vibrate_flag = 0;
							}
							if(tp->id[i].press_no_vibrate_flag == 0)
								tp->id[i].press_no_vibrate_flag = 2;
						} else {
						}
					}
					pr_debug("%s: %d: id[%d]:play_flag=%d, press_flag=%d, release_flag=%d, press_no_vibrate_flag=%d, no_play_cnt=%d\n",
						__func__, __LINE__, i, tp->id[i].play_flag, tp->id[i].press_flag, tp->id[i].release_flag,
						tp->id[i].press_no_vibrate_flag, tp->id[i].no_play_cnt);
				}
				tp->id[i].no_play_cnt = 0;
			} else if(AW8697_HAPTIC_PLAY_NULL == aw8697->haptic_audio.ctr.play) {
				if(AW8697_HAPTIC_TP_PRESS == tp->id[i].press_flag) {
					if(AW8697_HAPTIC_TP_PLAY_ENABLE == tp->id[i].play_flag) {
						tp->id[i].no_play_cnt ++;
						if(tp->id[i].no_play_cnt > tp->no_play_cnt_max) {
							/* no play cnt > play interal time */
							tp->id[i].no_play_cnt = tp->no_play_cnt_max;
							tp->id[i].play_flag = AW8697_HAPTIC_TP_PLAY_NULL;
							tp->id[i].press_flag = AW8697_HAPTIC_TP_NULL;
							tp->id[i].release_flag = AW8697_HAPTIC_TP_NULL;
							if(i == AW8697_HAPTIC_TP_ID_MAX) {
								tp->id[i].pt_info.status = AW8697_HAPTIC_TP_ST_RELEASE;
								tp->id[i].pt_info.touch_flag = AW8697_HAPTIC_TP_TOUCH_INVAIL;
								tp->id[i].tp_flag = AW8697_HAPTIC_TP_NULL;
								tp->id[i].press_flag = AW8697_HAPTIC_TP_NULL;
								tp->id[i].release_flag = AW8697_HAPTIC_TP_NULL;
								//tp->id[i].press_no_vibrate_flag = 0;
							}
							if(tp->id[i].press_no_vibrate_flag == 0)
								tp->id[i].press_no_vibrate_flag = 2;
						} else {
						}
					} else {
						tp->id[i].play_flag = AW8697_HAPTIC_TP_PLAY_NULL;
					}
					pr_debug("%s: %d: id[%d]:play_flag=%d, press_flag=%d, release_flag=%d, press_no_vibrate_flag=%d, no_play_cnt=%d\n",
						__func__, __LINE__, i, tp->id[i].play_flag, tp->id[i].press_flag, tp->id[i].release_flag,
						tp->id[i].press_no_vibrate_flag, tp->id[i].no_play_cnt);
				}
				if(AW8697_HAPTIC_TP_RELEASE == tp->id[i].release_flag) {
					/* tp release time > 3-continue-time play time*/
					if(release_delay > tp->release_delay_max) {
						tp->id[i].play_flag = AW8697_HAPTIC_TP_PLAY_NULL;
						tp->id[i].press_flag = AW8697_HAPTIC_TP_NULL;
						tp->id[i].release_flag = AW8697_HAPTIC_TP_NULL;
						tp->id[i].tp_ai_match_flag = 1;
						if(tp->id[i].press_no_vibrate_flag == 0)
							tp->id[i].press_no_vibrate_flag = 2;
						tp->tp_ai_check_flag = 0;
					} else {
					}
					pr_debug("%s: %d: id[%d]:play_flag=%d, press_flag=%d, release_flag=%d, press_no_vibrate_flag=%d, no_play_cnt=%d\n",
						__func__, __LINE__, i, tp->id[i].play_flag, tp->id[i].press_flag, tp->id[i].release_flag,
						tp->id[i].press_no_vibrate_flag, tp->id[i].no_play_cnt);
				}
			} else {
			}
		}

		/* adjust tp play enable */
		tp->play_flag = AW8697_HAPTIC_TP_PLAY_NULL;
		for(i=0; i<AW8697_HAPTIC_TP_ID_MAX+1; i++) {
			if(AW8697_HAPTIC_TP_PLAY_ENABLE == tp->id[i].play_flag) {
				tp_touch_play_flag = AW8697_HAPTIC_TP_PLAY_ENABLE;
				list_for_each_entry(p_tmp, &(haptic_audio->list), list) {
					match_re = aw8697_haptic_audio_tp_list_match(&(tp->id[i].pt_info), p_tmp);
					if (match_re == 1) {
						tp->play_flag = AW8697_HAPTIC_TP_PLAY_ENABLE;
						touch_vibrator_id = i;
						pr_debug("%s: tp input point[%d, %04d, %04d] is in the ai trust zone[%d, %04d, %04d, %04d, %04d]\n",
							__func__, i, tp->id[i].pt_info.x, tp->id[i].pt_info.y,
							p_tmp->level, p_tmp->x, p_tmp->y, p_tmp->w, p_tmp->h);
						break;
					} else if (match_re == 2) {
						if(!dirty)
							p_tmp->dirty = 1;
						dirty = 1;
						break;
					}
				}
			}
#ifdef SECOND_SCOREMODE
			if(tp->play_flag == AW8697_HAPTIC_TP_PLAY_ENABLE && dirty == 1) {
					break;
			}
#else
			if(tp->play_flag == AW8697_HAPTIC_TP_PLAY_ENABLE) {
					break;
			}
#endif
		}
		if(tp->play_flag) {
			pr_debug("%s: tp.play_flag with tp =%d, touch_vibrator_id=%d\n", __func__, tp->play_flag, touch_vibrator_id);
		}

		/* haptic play cnt outside trust zone over limit, restart ai scan */
		tp->tp_ai_match_flag = 1;
		for(i=0; i<AW8697_HAPTIC_TP_ID_MAX; i++) {
			pr_debug("%s: [%d] tp_ai_match_flag=%d\n", __func__, i, tp->id[i].tp_ai_match_flag);
			if(tp->id[i].tp_ai_match_flag == 0) {
				tp->tp_ai_match_flag = 0;
				tp_ai_match_id = i;
				break;
			}

		}
		pr_debug("%s: tp_ai_match_flag=%d, tp_ai_check_flag=%d, tz_high_num=%d, tp_touch_play_flag=%d, tp->last_play_flag=%d, hap_cnt_outside_tz=%d\n",
                       __func__, tp->tp_ai_match_flag, tp->tp_ai_check_flag,
                       haptic_audio->tz_high_num, tp_touch_play_flag, tp->last_play_flag, haptic_audio->hap_cnt_outside_tz);

		/* restart ai scan when tz_high_num=2 */
		if ((tp->tp_ai_match_flag == 0) &&
			(tp->tp_ai_check_flag == 1)) {
			if ((AW8697_HAPTIC_TP_PLAY_ENABLE == tp_touch_play_flag) &&
				(AW8697_HAPTIC_TP_PLAY_NULL == tp->play_flag)) {
#ifdef FIRST_SCOREMODE
				list_for_each_entry(p_tmp, &(haptic_audio->list), list) {
					if ((p_tmp->level >= TRUST_LEVEL) && (p_tmp->x < x_thr)) {
						p_tmp_r = p_tmp;
					} else if ((p_tmp->level >= TRUST_LEVEL) && (p_tmp->x >= x_thr)) {
						p_tmp_l = p_tmp;
					}
				}

				for(i=0; i<AW8697_HAPTIC_TP_ID_MAX+1; i++){
					if (tp->id[i].play_flag) {
						list_for_each_entry(p_tmp, &(haptic_audio->score_list), list) {
							if (aw8697_haptic_audio_tp_list_match(&(tp->id[i].pt_info), p_tmp))
							{
								p_tmp->level = (p_tmp->level < TRUST_LEVEL) ? (++p_tmp->level) : TRUST_LEVEL;
							}
						}
					}
				}

				list_for_each_entry(p_tmp, &(haptic_audio->score_list), list) {
					if (p_tmp->level >= TRUST_LEVEL) {
						if(p_tmp->x < x_thr) {
							if (p_tmp_r) {
								list_del(&p_tmp_r->list);
								kfree(p_tmp_r);
								haptic_audio->tz_num -= 1;
								pr_debug("%s scorelist_num = %d\n", __func__, haptic_audio->tz_num);
							}
						}else{
							if (p_tmp_l) {
								list_del(&p_tmp_l->list);
								kfree(p_tmp_l);
								haptic_audio->tz_num -= 1;
								pr_debug("%s scorelist_num = %d\n", __func__, haptic_audio->tz_num);
							}
						}
						list_del(&p_tmp->list);
						list_add(&p_tmp->list,&(haptic_audio->list));
						pr_debug("p_tmp->level >= TRUST_LEVEL\n");
						break;
					}
				}
#endif
				tp->tp_ai_check_flag = 0;
				tp->id[tp_ai_match_id].tp_ai_match_flag = 1;
			} else {
				if (tp->id[touch_vibrator_id].press_no_vibrate_flag == 2) {
					tp->tp_ai_check_flag = 0;
				}
			}
		}

		if(AW8697_HAPTIC_PLAY_ENABLE == aw8697->haptic_audio.ctr.play) {
			tp->last_play_flag = tp->play_flag;
		}
		/* clear all id tp flag */
		for(i=0; i<AW8697_HAPTIC_TP_ID_MAX+1; i++) {
			tp->id[i].tp_flag = AW8697_HAPTIC_TP_NULL;
		}
	} else {
		for(i=0; i<AW8697_HAPTIC_TP_ID_MAX+1; i++) {
			tp->id[i].tp_flag = AW8697_HAPTIC_TP_NULL;
		}
	}
	mutex_unlock(&aw8697->haptic_audio.lock);

	/* aw8697 haptic play control */
	if(AW8697_HAPTIC_CMD_ENABLE ==
		(AW8697_HAPTIC_CMD_HAPTIC & aw8697->haptic_audio.ctr.cmd)) {
		if(AW8697_HAPTIC_PLAY_ENABLE == aw8697->haptic_audio.ctr.play) {
			pr_info("%s: haptic_audio_play_start\n", __func__);
			if(AW8697_HAPTIC_CMD_TP == (AW8697_HAPTIC_CMD_SYS & aw8697->haptic_audio.ctr.cmd) &&
				(AW8697_HAPTIC_TP_PLAY_ENABLE != tp->play_flag)) {
				pr_info("%s: cancel haptic with tp event\n", __func__);
			} else {
				pr_info("%s: normal haptic with tp event\n", __func__);
				mutex_lock(&aw8697->lock);
				aw8697_haptic_audio_stop(aw8697);
				aw8697_haptic_audio_play_cfg(aw8697);

				aw8697_haptic_set_wav_seq(aw8697, 0x00,
						aw8697->haptic_audio.ctr.wavseq);

				aw8697_haptic_set_wav_loop(aw8697, 0x00,
						aw8697->haptic_audio.ctr.loop);

				aw8697_haptic_set_gain(aw8697,
						aw8697->haptic_audio.ctr.gain);

				aw8697_haptic_start(aw8697);
				mutex_unlock(&aw8697->lock);
			}
		} else if(AW8697_HAPTIC_PLAY_STOP == aw8697->haptic_audio.ctr.play) {
			mutex_lock(&aw8697->lock);
			aw8697_haptic_audio_stop(aw8697);
			mutex_unlock(&aw8697->lock);
		} else if(AW8697_HAPTIC_PLAY_GAIN == aw8697->haptic_audio.ctr.play) {
			mutex_lock(&aw8697->lock);
			aw8697_haptic_set_gain(aw8697,
				aw8697->haptic_audio.ctr.gain);
			mutex_unlock(&aw8697->lock);
		}
	}
}


/*****************************************************
 *
 * haptic cont
 *
 *****************************************************/
static int aw8697_haptic_cont(struct aw8697 *aw8697)
{
    pr_info("%s enter\n", __func__);

    /* work mode */
    aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_CONT_MODE);
    /* preset f0 */
    aw8697->f0_pre = aw8697->f0;
    aw8697_haptic_set_f0_preset(aw8697);
    /* lpf */
    aw8697_i2c_write_bits(aw8697, AW8697_REG_DATCTRL,
            AW8697_BIT_DATCTRL_FC_MASK, AW8697_BIT_DATCTRL_FC_1000HZ);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_DATCTRL,
            AW8697_BIT_DATCTRL_LPF_ENABLE_MASK, AW8697_BIT_DATCTRL_LPF_ENABLE);
    /* cont config */
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
            AW8697_BIT_CONT_CTRL_ZC_DETEC_MASK, AW8697_BIT_CONT_CTRL_ZC_DETEC_ENABLE);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
            AW8697_BIT_CONT_CTRL_WAIT_PERIOD_MASK, AW8697_BIT_CONT_CTRL_WAIT_1PERIOD);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
            AW8697_BIT_CONT_CTRL_MODE_MASK, AW8697_BIT_CONT_CTRL_BY_GO_SIGNAL);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
            AW8697_BIT_CONT_CTRL_EN_CLOSE_MASK, AW8697_CONT_PLAYBACK_MODE);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
            AW8697_BIT_CONT_CTRL_F0_DETECT_MASK, AW8697_BIT_CONT_CTRL_F0_DETECT_DISABLE);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
            AW8697_BIT_CONT_CTRL_O2C_MASK, AW8697_BIT_CONT_CTRL_O2C_DISABLE);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
            AW8697_BIT_CONT_CTRL_AUTO_BRK_MASK, AW8697_BIT_CONT_CTRL_AUTO_BRK_ENABLE);
    /* TD time */
    aw8697_i2c_write(aw8697, AW8697_REG_TD_H, (unsigned char)(aw8697->cont_td>>8));
    aw8697_i2c_write(aw8697, AW8697_REG_TD_L, (unsigned char)(aw8697->cont_td>>0));
    aw8697_i2c_write(aw8697, AW8697_REG_TSET, 0x12);
    /* zero cross */
    aw8697_i2c_write(aw8697, AW8697_REG_ZC_THRSH_H, (unsigned char)(aw8697->cont_zc_thr>>8));
    aw8697_i2c_write(aw8697, AW8697_REG_ZC_THRSH_L, (unsigned char)(aw8697->cont_zc_thr>>0));
    /* bemf */
    aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHH_H, 0x10);
    aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHH_L, 0x08);
    aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHL_H, 0x03);
    aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHL_L, 0xf8);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_BEMF_NUM,
            AW8697_BIT_BEMF_NUM_BRK_MASK, aw8697->cont_num_brk);
    aw8697_i2c_write(aw8697, AW8697_REG_TIME_NZC, 0x23);    /* 35*171us=5.985ms*/
    /* f0 driver level */
    aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL, aw8697->cont_drv_lvl);
    aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL_OV, aw8697->cont_drv_lvl_ov);
    /* cont play go */
    aw8697_haptic_play_go(aw8697, true);

    return 0;
}

static int haptic_get_f0_defalut(struct aw8697 *aw8697)
{
    int ret = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;
    unsigned int t_f0_trace_ms = 0;
    unsigned int f0_cali_cnt = 50;
    unsigned char f0_pre_num = 0;
    unsigned char f0_wait_num = 0;
    unsigned char f0_repeat_num = 0;
    unsigned char f0_trace_num = 0;
    unsigned int t_f0_ms = 0;
    pr_info("%s enter\n", __func__);

    aw8697->f0 = aw8697->f0_pre;
    /* f0 calibrate work mode */
    aw8697_haptic_stop(aw8697);
    aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_CONT_MODE);

    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
        AW8697_BIT_CONT_CTRL_EN_CLOSE_MASK, AW8697_BIT_CONT_CTRL_OPEN_PLAYBACK);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
        AW8697_BIT_CONT_CTRL_F0_DETECT_MASK, AW8697_BIT_CONT_CTRL_F0_DETECT_ENABLE);

    /* LPF */
    aw8697_i2c_write_bits(aw8697, AW8697_REG_DATCTRL,
        AW8697_BIT_DATCTRL_FC_MASK, AW8697_BIT_DATCTRL_FC_1000HZ);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_DATCTRL,
        AW8697_BIT_DATCTRL_LPF_ENABLE_MASK, AW8697_BIT_DATCTRL_LPF_ENABLE);

    /* LRA OSC Source */
    if(aw8697->f0_cali_flag == AW8697_HAPTIC_CALI_F0) {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
            AW8697_BIT_ANACTRL_LRA_SRC_MASK, AW8697_BIT_ANACTRL_LRA_SRC_REG);
    } else {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
            AW8697_BIT_ANACTRL_LRA_SRC_MASK, AW8697_BIT_ANACTRL_LRA_SRC_EFUSE);
    }

    /* preset f0 */
    aw8697_haptic_set_f0_preset(aw8697);

    /* beme config */
    aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHH_H, 0x10);
    aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHH_L, 0x08);
    aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHL_H, 0x03);
    aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHL_L, 0xf8);

    /* f0 driver level */
    aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL, aw8697->cont_drv_lvl);

    /* f0 trace parameter */
    f0_pre_num = 0x05;
    f0_wait_num = 0x03;
    f0_repeat_num = 0x02;
    f0_trace_num = 0x0f;
    aw8697_i2c_write(aw8697, AW8697_REG_NUM_F0_1, (f0_pre_num<<4)|(f0_wait_num<<0));
    aw8697_i2c_write(aw8697, AW8697_REG_NUM_F0_2, (f0_repeat_num<<0));
    aw8697_i2c_write(aw8697, AW8697_REG_NUM_F0_3, (f0_trace_num<<0));

    /* clear aw8697 interrupt */
    ret = aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);

    /* play go and start f0 calibration */
    aw8697_haptic_play_go(aw8697, true);

    /* f0 trace time */
    t_f0_ms = 1000*10/aw8697->f0_pre;
    t_f0_trace_ms = t_f0_ms * (f0_pre_num + f0_wait_num + (f0_trace_num+f0_wait_num)*(f0_repeat_num-1));
    msleep(t_f0_trace_ms);

    for(i=0; i<f0_cali_cnt; i++) {
        ret = aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
        /* f0 calibrate done */
        if(reg_val & 0x01) {
            aw8697_haptic_read_f0(aw8697);
            aw8697_haptic_read_beme(aw8697);
            break;
        }
        msleep(10);
        pr_info("%s f0 cali sleep 10ms\n", __func__);
    }

    if(i == f0_cali_cnt) {
        ret = -1;
    } else {
        ret = 0;
    }

    /* restore default config */
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
            AW8697_BIT_CONT_CTRL_EN_CLOSE_MASK, AW8697_CONT_PLAYBACK_MODE);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
            AW8697_BIT_CONT_CTRL_F0_DETECT_MASK, AW8697_BIT_CONT_CTRL_F0_DETECT_DISABLE);
    return ret;
}

static int haptic_get_f0_count_go(struct aw8697 *aw8697)
{
    int ret = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;
    unsigned int t_f0_trace_ms = 0;
    unsigned int f0_cali_cnt = 50;

    pr_info("%s enter\n", __func__);
    aw8697->f0 = aw8697->f0_pre;

    /* f0 calibrate work mode */
    aw8697_haptic_stop(aw8697);
    aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_CONT_MODE);

    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
        AW8697_BIT_CONT_CTRL_MODE_MASK, AW8697_BIT_CONT_CTRL_BY_DRV_TIME);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
        AW8697_BIT_CONT_CTRL_EN_CLOSE_MASK, AW8697_BIT_CONT_CTRL_OPEN_PLAYBACK);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
        AW8697_BIT_CONT_CTRL_EN_CLOSE_MASK, AW8697_BIT_CONT_CTRL_CLOSE_PLAYBACK);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
        AW8697_BIT_CONT_CTRL_F0_DETECT_MASK, AW8697_BIT_CONT_CTRL_F0_DETECT_DISABLE);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
        AW8697_BIT_CONT_CTRL_AUTO_BRK_MASK, AW8697_BIT_CONT_CTRL_AUTO_BRK_DISABLE);

    /* LRA OSC Source */
    if(aw8697->f0_cali_flag == AW8697_HAPTIC_CALI_F0) {
       aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
            AW8697_BIT_ANACTRL_LRA_SRC_MASK, AW8697_BIT_ANACTRL_LRA_SRC_REG);
    } else {
        aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
            AW8697_BIT_ANACTRL_LRA_SRC_MASK, AW8697_BIT_ANACTRL_LRA_SRC_EFUSE);
    }

    /* f0 driver level */
    aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL, 0x61);
    aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL_OV, 0x61);

    /* TD time */
    aw8697_i2c_write(aw8697, AW8697_REG_TD_H, 0x00);
    aw8697_i2c_write(aw8697, AW8697_REG_TD_L, 0xe5);
    aw8697_i2c_write(aw8697, AW8697_REG_TSET, 0x48);

    /* drive time  */
    aw8697_i2c_write(aw8697, AW8697_REG_DRV_TIME, 0xFE);

    /* preset f0 */
    aw8697_haptic_set_f0_preset(aw8697);

    /* clear aw8697 interrupt */
    ret = aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);

    /* play go and start f0 calibration */
    aw8697_haptic_play_go(aw8697, true);

    /* f0 trace time */
    t_f0_trace_ms = 0xfe * 684 / 1000;
    msleep(t_f0_trace_ms);

    for(i=0; i<f0_cali_cnt; i++) {
        ret = aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
        /* f0 calibrate done */
       if(reg_val & 0x01) {
            aw8697_haptic_read_f0(aw8697);
            aw8697_haptic_read_bemf(aw8697);
            break;
        }
        msleep(10);
        pr_info("%s f0 cali sleep 10ms\n", __func__);
    }

    if(i == f0_cali_cnt) {
        ret = -1;
    } else {
        ret = 0;
    }

    /* restore default config */
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
        AW8697_BIT_CONT_CTRL_EN_CLOSE_MASK, AW8697_CONT_PLAYBACK_MODE);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
        AW8697_BIT_CONT_CTRL_F0_DETECT_MASK, AW8697_BIT_CONT_CTRL_F0_DETECT_DISABLE);
    return ret;
}

/*****************************************************
 *
 * haptic f0 cali
 *
 *****************************************************/
static int aw8697_haptic_get_f0(struct aw8697 *aw8697)
{
    int ret;

    if (aw8697->count_go)
          ret = haptic_get_f0_count_go(aw8697);
    else
          ret = haptic_get_f0_defalut(aw8697);
    return ret;
}

static int aw8697_haptic_f0_calibration(struct aw8697 *aw8697)
{
    int ret = 0;
    unsigned char reg_val = 0;
    unsigned int f0_limit = 0;
    char f0_cali_lra = 0;
    int f0_cali_step = 0;
    int f0_dft_step = 0;

    pr_info("%s enter\n", __func__);

    aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;

    if(aw8697_haptic_get_f0(aw8697)) {
        pr_err("%s get f0 error, user defafult f0\n", __func__);
    } else {
        /* max and min limit */
        f0_limit = aw8697->f0;
        if(aw8697->f0*100 < AW8697_HAPTIC_F0_PRE*(100-AW8697_HAPTIC_F0_CALI_PERCEN)) {
            f0_limit = AW8697_HAPTIC_F0_PRE*(100-AW8697_HAPTIC_F0_CALI_PERCEN)/100;
        }
        if(aw8697->f0*100 > AW8697_HAPTIC_F0_PRE*(100+AW8697_HAPTIC_F0_CALI_PERCEN)) {
            f0_limit = AW8697_HAPTIC_F0_PRE*(100+AW8697_HAPTIC_F0_CALI_PERCEN)/100;
        }
        /* calculate cali step */
        f0_cali_step = 10000*((int)f0_limit-(int)aw8697->f0_pre)/((int)f0_limit*25);
        pr_debug("%s f0_cali_step=%d\n", __func__, f0_cali_step);

        /* get default cali step */
        aw8697_i2c_read(aw8697, AW8697_REG_TRIM_LRA, &reg_val);
        if(reg_val & 0x20) {
            f0_dft_step = reg_val - 0x40;
        } else {
            f0_dft_step = reg_val;
        }
        pr_debug("%s f0_dft_step=%d\n", __func__, f0_dft_step);

        /* get new cali step */
        f0_cali_step += f0_dft_step;
        pr_debug("%s f0_cali_step=%d\n", __func__, f0_cali_step);

        if(f0_cali_step > 31) {
            f0_cali_step = 31;
        } else if(f0_cali_step < -32) {
            f0_cali_step = -32;
        }
        f0_cali_lra = (char)f0_cali_step;
        pr_debug("%s f0_cali_lra=%d\n", __func__, f0_cali_lra);

        /* get cali step complement code*/
        if(f0_cali_lra < 0) {
            f0_cali_lra += 0x40;
        }
        pr_debug("%s reg f0_cali_lra=%d\n", __func__, f0_cali_lra);

        /* update cali step */
        aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, (char)f0_cali_lra);
        aw8697_i2c_read(aw8697, AW8697_REG_TRIM_LRA, &reg_val);
        pr_info("%s final trim_lra=0x%02x\n", __func__, reg_val);
    }

    if(aw8697_haptic_get_f0(aw8697)) {
        pr_err("%s get f0 error, user defafult f0\n", __func__);
    }
    /* restore default work mode */
    aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_STANDBY_MODE);
    aw8697->play_mode = AW8697_HAPTIC_RAM_MODE;
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_PLAY_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);
    aw8697_haptic_stop(aw8697);

    return ret;
}

/*****************************************************
 *
 * haptic fops
 *
 *****************************************************/
static int aw8697_file_open(struct inode *inode, struct file *file)
{
    if (!try_module_get(THIS_MODULE))
        return -ENODEV;

    file->private_data = (void*)g_aw8697;

    return 0;
}

static int aw8697_file_release(struct inode *inode, struct file *file)
{
    file->private_data = (void*)NULL;

    module_put(THIS_MODULE);

    return 0;
}

static long aw8697_file_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct aw8697 *aw8697 = (struct aw8697 *)file->private_data;

    int ret = 0;

    dev_info(aw8697->dev, "%s: cmd=0x%x, arg=0x%lx\n",
              __func__, cmd, arg);

    mutex_lock(&aw8697->lock);

    if(_IOC_TYPE(cmd) != AW8697_HAPTIC_IOCTL_MAGIC) {
        dev_err(aw8697->dev, "%s: cmd magic err\n",
                __func__);
        return -EINVAL;
    }

    switch (cmd) {
        default:
            dev_err(aw8697->dev, "%s, unknown cmd\n", __func__);
            break;
    }

    mutex_unlock(&aw8697->lock);

    return ret;
}

static ssize_t aw8697_file_read(struct file* filp, char* buff, size_t len, loff_t* offset)
{
    struct aw8697 *aw8697 = (struct aw8697 *)filp->private_data;
    int ret = 0;
    int i = 0;
    unsigned char reg_val = 0;
    unsigned char *pbuff = NULL;

    mutex_lock(&aw8697->lock);

    dev_info(aw8697->dev, "%s: len=%zu\n", __func__, len);

    switch(aw8697->fileops.cmd)
    {
        case AW8697_HAPTIC_CMD_READ_REG:
            pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
            if(pbuff != NULL) {
                for(i=0; i<len; i++) {
                    aw8697_i2c_read(aw8697, aw8697->fileops.reg+i, &reg_val);
                    pbuff[i] = reg_val;
                }
                for(i=0; i<len; i++) {
                    dev_info(aw8697->dev, "%s: pbuff[%d]=0x%02x\n",
                            __func__, i, pbuff[i]);
                }
                ret = copy_to_user(buff, pbuff, len);
                if(ret) {
                    dev_err(aw8697->dev, "%s: copy to user fail\n", __func__);
                }
                kfree(pbuff);
            } else {
                dev_err(aw8697->dev, "%s: alloc memory fail\n", __func__);
            }
            break;
        default:
            dev_err(aw8697->dev, "%s, unknown cmd %d \n", __func__, aw8697->fileops.cmd);
            break;
    }

    mutex_unlock(&aw8697->lock);


    return len;
}

static ssize_t aw8697_file_write(struct file* filp, const char* buff, size_t len, loff_t* off)
{
    struct aw8697 *aw8697 = (struct aw8697 *)filp->private_data;
    int i = 0;
    int ret = 0;
    unsigned char *pbuff = NULL;

    pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
    if(pbuff == NULL) {
        dev_err(aw8697->dev, "%s: alloc memory fail\n", __func__);
        return len;
    }
    ret = copy_from_user(pbuff, buff, len);
    if(ret) {
        kfree(pbuff);
        dev_err(aw8697->dev, "%s: copy from user fail\n", __func__);
        return len;
    }

    for(i=0; i<len; i++) {
        dev_info(aw8697->dev, "%s: pbuff[%d]=0x%02x\n",
                __func__, i, pbuff[i]);
    }

    mutex_lock(&aw8697->lock);

    aw8697->fileops.cmd = pbuff[0];

    switch(aw8697->fileops.cmd)
    {
        case AW8697_HAPTIC_CMD_READ_REG:
            if(len == 2) {
                aw8697->fileops.reg = pbuff[1];
            } else {
                dev_err(aw8697->dev, "%s: read cmd len %zu err\n", __func__, len);
            }
            break;
        case AW8697_HAPTIC_CMD_WRITE_REG:
            if(len > 2) {
                for(i=0; i<len-2; i++) {
                    dev_info(aw8697->dev, "%s: write reg0x%02x=0x%02x\n",
                            __func__, pbuff[1]+i, pbuff[i+2]);
                    aw8697_i2c_write(aw8697, pbuff[1]+i, pbuff[2+i]);
                }
            } else {
                dev_err(aw8697->dev, "%s: write cmd len %zu err\n", __func__, len);
            }
            break;
        default:
            dev_err(aw8697->dev, "%s, unknown cmd %d \n", __func__, aw8697->fileops.cmd);
            break;
    }

    mutex_unlock(&aw8697->lock);

    if(pbuff != NULL) {
        kfree(pbuff);
    }
    return len;
}

static struct file_operations fops =
{
    .owner = THIS_MODULE,
    .read = aw8697_file_read,
    .write = aw8697_file_write,
    .unlocked_ioctl = aw8697_file_unlocked_ioctl,
    .open = aw8697_file_open,
    .release = aw8697_file_release,
};

static struct miscdevice aw8697_haptic_misc =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = AW8697_HAPTIC_NAME,
    .fops = &fops,
};

static bool check_factory_mode(void)
{
	int boot_mode = get_boot_mode();

	if (boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN
			|| boot_mode == MSM_BOOT_MODE__FACTORY)
		return true;
	else
		return false;
}

static int aw8697_haptic_init(struct aw8697 *aw8697)
{
    int ret = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;
    struct tp *tp = &(aw8697->haptic_audio.tp);

    pr_info("%s enter\n", __func__);

    ret = misc_register(&aw8697_haptic_misc);
    if(ret) {
        dev_err(aw8697->dev,  "%s: misc fail: %d\n", __func__, ret);
        return ret;
    }

    /* haptic audio */
    aw8697->haptic_audio.delay_val = 1;
    aw8697->haptic_audio.timer_val = 21318;
    INIT_LIST_HEAD(&(aw8697->haptic_audio.ctr_list));

    hrtimer_init(&aw8697->haptic_audio.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    aw8697->haptic_audio.timer.function = aw8697_haptic_audio_timer_func;
    INIT_WORK(&aw8697->haptic_audio.work, aw8697_haptic_audio_work_routine);

    mutex_init(&aw8697->haptic_audio.lock);

    aw8697->haptic_audio.tp.press_delay_min = 1;
    aw8697->haptic_audio.tp.press_delay_max = 500000;
    aw8697->haptic_audio.tp.release_delay_max = 600000;
    aw8697->haptic_audio.tp.no_play_cnt_max = 230;
    for(i=0; i<AW8697_HAPTIC_TP_ID_MAX+1; i++) {
        tp->id[i].pt_info.status = 0;
    }

    INIT_LIST_HEAD(&(aw8697->haptic_audio.list));
	INIT_LIST_HEAD(&(aw8697->haptic_audio.score_list));
    aw8697->haptic_audio.tp_size.x = 3120;
    aw8697->haptic_audio.tp_size.y = 1440;
    aw8697->haptic_audio.tz_cnt_thr = 3;
    aw8697->haptic_audio.tz_cnt_max = 10;
    aw8697->haptic_audio.hap_cnt_max_outside_tz = 5;

    aw8697_op_clean_status(aw8697);

    /* haptic init */
    mutex_lock(&aw8697->lock);
    /*For haptic test 90mA in AT begin*/
    aw8697->activate_mode = AW8697_HAPTIC_ACTIVATE_RAM_MODE;
    aw8697_haptic_set_wav_loop(aw8697, aw8697->loop[0x00], 0xff);
    aw8697_haptic_set_bst_vol(aw8697, 0x11);
    aw8697->level = 3;
    if (check_factory_mode())
       aw8697_haptic_set_gain(aw8697, 0xcf);
    else
       aw8697_haptic_set_gain(aw8697, 0x80);
    aw8697_haptic_set_repeat_wav_seq(aw8697, 10);
    /*For haptic test 90mA in AT end*/
    ret = aw8697_i2c_read(aw8697, AW8697_REG_WAVSEQ1, &reg_val);
    aw8697->index = reg_val & 0x7F;
    ret = aw8697_i2c_read(aw8697, AW8697_REG_DATDBG, &reg_val);
    aw8697->gain = reg_val & 0xFF;
    ret = aw8697_i2c_read(aw8697, AW8697_REG_BSTDBG4, &reg_val);
    aw8697->vmax = (reg_val>>1)&0x1F;
    for(i=0; i<AW8697_SEQUENCER_SIZE; i++) {
        ret = aw8697_i2c_read(aw8697, AW8697_REG_WAVSEQ1+i, &reg_val);
        aw8697->seq[i] = reg_val;
    }

    aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_STANDBY_MODE);

    aw8697_haptic_set_pwm(aw8697, AW8697_PWM_24K);

    aw8697_i2c_write(aw8697, AW8697_REG_BSTDBG1, 0x30);
    aw8697_i2c_write(aw8697, AW8697_REG_BSTDBG2, 0xeb);
    aw8697_i2c_write(aw8697, AW8697_REG_BSTDBG3, 0xd4);
    aw8697_i2c_write(aw8697, AW8697_REG_TSET, 0x12);
    aw8697_i2c_write(aw8697, AW8697_REG_R_SPARE, 0x68);

    aw8697_i2c_write_bits(aw8697, AW8697_REG_ANADBG,
            AW8697_BIT_ANADBG_IOC_MASK, AW8697_BIT_ANADBG_IOC_4P65A);
    /*OP over current limmit set to max begin*/
    aw8697_i2c_write_bits(aw8697, AW8697_REG_ANADBG,
            AW8697_BIT_ANADBG_IOC_OCDT_MASK, AW8697_BIT_ANADBG_IOC_OCDT_150NS);
    /*OP over current limmit set to max end*/
    aw8697_haptic_set_bst_peak_cur(aw8697, AW8697_DEFAULT_PEAKCUR);
    aw8697_haptic_swicth_motorprotect_config(aw8697, 0x00, 0x00);
    aw8697_haptic_auto_boost_config(aw8697, false);

    aw8697_haptic_trig_param_init(aw8697);
    aw8697_haptic_trig_param_config(aw8697);

    aw8697_haptic_os_calibration(aw8697);
    if (aw8697->count_go) {
        aw8697_haptic_cont_vbat_mode(aw8697,
            AW8697_HAPTIC_CONT_VBAT_SW_COMP_MODE);
    } else {
        aw8697_haptic_cont_vbat_mode(aw8697,
            AW8697_HAPTIC_CONT_VBAT_HW_COMP_MODE);
    }
    aw8697->ram_vbat_comp = AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE;
    aw8697_haptic_set_wav_seq(aw8697, 0x01, 0x00);
    mutex_unlock(&aw8697->lock);

    /* f0 calibration */
    mutex_lock(&aw8697->lock);
    aw8697->f0_pre = AW8697_HAPTIC_F0_PRE;
    aw8697->cont_drv_lvl = AW8697_HAPTIC_CONT_DRV_LVL;
    aw8697->cont_drv_lvl_ov = AW8697_HAPTIC_CONT_DRV_LVL_OV;
    aw8697->cont_td = AW8697_HAPTIC_CONT_TD;
    aw8697->cont_zc_thr = AW8697_HAPTIC_CONT_ZC_THR;
    aw8697->cont_num_brk = AW8697_HAPTIC_CONT_NUM_BRK;
    mutex_unlock(&aw8697->lock);

    return ret;
}



/*****************************************************
 *
 * vibrator
 *
 *****************************************************/
#ifdef TIMED_OUTPUT
static int aw8697_vibrator_get_time(struct timed_output_dev *dev)
{
    struct aw8697 *aw8697 = container_of(dev, struct aw8697, to_dev);

    if (hrtimer_active(&aw8697->timer)) {
        ktime_t r = hrtimer_get_remaining(&aw8697->timer);
        return ktime_to_ms(r);
    }

    return 0;
}

static void aw8697_vibrator_enable( struct timed_output_dev *dev, int value)
{
    struct aw8697 *aw8697 = container_of(dev, struct aw8697, to_dev);
    unsigned char reg_val = 0;

    mutex_lock(&aw8697->lock);

    pr_debug("%s enter\n", __func__);
    /*RTP mode do not allow other vibrate begign*/
    if(aw8697->play_mode == AW8697_HAPTIC_RTP_MODE)
    {
         aw8697_i2c_read(aw8697, AW8697_REG_GLB_STATE, &reg_val);
         if((reg_val&0x0f) != 0x00) {
             mutex_unlock(&aw8697->lock);
             pr_info("%s RTP on && not stop,return\n", __func__);
             return;
         }
    }
    /*RTP mode do not allow other vibrate end*/

    aw8697_haptic_stop(aw8697);

    if (value > 0) {
        aw8697_haptic_ram_vbat_comp(aw8697, false);
        aw8697_haptic_play_wav_seq(aw8697, value);
    }

    mutex_unlock(&aw8697->lock);

    pr_debug("%s exit\n", __func__);
}

#else
static enum led_brightness aw8697_haptic_brightness_get(struct led_classdev *cdev)
{
    struct aw8697 *aw8697 =
        container_of(cdev, struct aw8697, cdev);

    return aw8697->amplitude;
}

static void aw8697_haptic_brightness_set(struct led_classdev *cdev,
                enum led_brightness level)
{
    struct aw8697 *aw8697 =
        container_of(cdev, struct aw8697, cdev);
    int rtp_is_going_on = 0;
    static int val_pre;

    /*OP add for juge rtp on begin*/
    rtp_is_going_on = aw8697_haptic_juge_RTP_is_going_on(aw8697);
    if (rtp_is_going_on)
       return;
    if (!aw8697->ram_init)
       return;
    if (aw8697->amplitude != val_pre)
        pr_info("%s enter,level:%d\n", __func__, level);
    /*OP add for juge rtp on end*/
    aw8697->amplitude = level;
    val_pre = aw8697->amplitude;
    mutex_lock(&aw8697->lock);
    aw8697_haptic_stop(aw8697);
    if (aw8697->amplitude > 0) {
        /*aw8697_haptic_ram_vbat_comp(aw8697, false);*/
        aw8697_haptic_play_wav_seq(aw8697, aw8697->amplitude);
    }
    mutex_unlock(&aw8697->lock);
}
#endif

static ssize_t aw8697_state_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

    return snprintf(buf, PAGE_SIZE, "%d\n", aw8697->state);
}

static ssize_t aw8697_state_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    return count;
}

static ssize_t aw8697_duration_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ktime_t time_rem;
    s64 time_ms = 0;

    if (hrtimer_active(&aw8697->timer)) {
        time_rem = hrtimer_get_remaining(&aw8697->timer);
        time_ms = ktime_to_ms(time_rem);
    }

    return snprintf(buf, PAGE_SIZE, "%lld\n", time_ms);
}

static ssize_t aw8697_duration_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;
    static unsigned int val_pre;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;
    if (val_pre != val)
       pr_info("%s:val:%d\n", __func__, val);
    val_pre = val;
    /* setting 0 on duration is NOP for now */
    if (val <= 0)
        return count;

    aw8697->duration = val;

    return count;
}

static ssize_t aw8697_activate_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

    /* For now nothing to show */
    return snprintf(buf, PAGE_SIZE, "%d\n", aw8697->state);
}

static ssize_t aw8697_activate_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;
    int rtp_is_going_on = 0;
    static unsigned int  val_pre;

    /*OP add for juge rtp on begin*/
    rtp_is_going_on = aw8697_haptic_juge_RTP_is_going_on(aw8697);
    if (rtp_is_going_on)
       return count;
    if (!aw8697->ram_init)
       return count;
    /*OP add for juge rtp on end*/
    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    if (val != 0 && val != 1)
        return count;
    if (val_pre != val)
       pr_info("%s: value=%d\n", __FUNCTION__, val);
    val_pre = val;
    mutex_lock(&aw8697->lock);
/*for type Android OS's vibrator 20181225 begin*/
/*set mode as RAM*/
	aw8697->activate_mode = AW8697_HAPTIC_ACTIVATE_RAM_MODE;
/*set ram_vbat_comp*/
	/*aw8697->ram_vbat_comp = AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE;*/
/*set wave number,index*/
    if (check_factory_mode() && aw8697->count_go)
        aw8697->index = 19;/* index 19sine 170hz*/
    else
        aw8697->index = 10;/*sine 170hz*/
    aw8697_haptic_set_repeat_wav_seq(aw8697, aw8697->index);
/*for type Android OS's vibrator 20181225  end*/
    if (val == 0)
       mdelay(10);
    hrtimer_cancel(&aw8697->timer);

    aw8697->state = val;
    mutex_unlock(&aw8697->lock);
    schedule_work(&aw8697->vibrator_work);

    return count;
}

static ssize_t aw8697_activate_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

    return snprintf(buf, PAGE_SIZE, "activate_mode=%d\n", aw8697->activate_mode);
}

static ssize_t aw8697_activate_mode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    if (!aw8697->ram_init)
       return count;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;
    pr_info("%s:val:%d\n", __func__, val);
    mutex_lock(&aw8697->lock);
    aw8697->activate_mode = val;
    mutex_unlock(&aw8697->lock);
    return count;
}

static ssize_t aw8697_index_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned char reg_val = 0;
    aw8697_i2c_read(aw8697, AW8697_REG_WAVSEQ1, &reg_val);
    aw8697->index = reg_val;

    return snprintf(buf, PAGE_SIZE, "%d\n", aw8697->index);
}

static ssize_t aw8697_index_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;
    static unsigned int val_pre;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;
    if (val_pre != val)
       pr_info("%s: value=%d\n", __FUNCTION__, val);
    val_pre = val;
    mutex_lock(&aw8697->lock);
    aw8697->index = val;
    aw8697_haptic_set_repeat_wav_seq(aw8697, aw8697->index);
    mutex_unlock(&aw8697->lock);
    return count;
}

static ssize_t aw8697_vmax_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

    return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw8697->vmax);
}

static ssize_t aw8697_vmax_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;
    static unsigned int val_pre;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;
    if (!aw8697->ram_init)
       return count;
    if (val_pre != val)
        pr_info("%s: value=%d\n", __FUNCTION__, val);
    val_pre = val;
    mutex_lock(&aw8697->lock);
    aw8697->vmax = val;
    aw8697_haptic_set_bst_vol(aw8697, aw8697->vmax);
    mutex_unlock(&aw8697->lock);
    return count;
}

static ssize_t aw8697_gain_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

    return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw8697->gain);
}

static ssize_t aw8697_gain_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;
    static unsigned int val_pre;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;
    if (!aw8697->ram_init)
       return count;
    if (val_pre != val)
        pr_info("%s: value=%d\n", __FUNCTION__, val);
    val_pre = val;
    mutex_lock(&aw8697->lock);
    aw8697->gain = val;
    aw8697_haptic_set_gain(aw8697, aw8697->gain);
    mutex_unlock(&aw8697->lock);
    return count;
}

static ssize_t aw8697_level_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

    return snprintf(buf, PAGE_SIZE, "%d\n", aw8697->level);
}

static ssize_t aw8697_level_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    if (val < 0 || val > 10)
        val = 3;

    pr_info("%s: value=%d\n", __FUNCTION__, val);
    mutex_lock(&aw8697->lock);
    aw8697->level = val;
    aw8697_haptic_set_gain(aw8697, aw8697->gain);
    mutex_unlock(&aw8697->lock);
    return count;
}

static ssize_t aw8697_seq_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    size_t count = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;

    for(i=0; i<AW8697_SEQUENCER_SIZE; i++) {
        aw8697_i2c_read(aw8697, AW8697_REG_WAVSEQ1+i, &reg_val);
        count += snprintf(buf+count, PAGE_SIZE-count,
                "seq%d: 0x%02x\n", i+1, reg_val);
        aw8697->seq[i] |= reg_val;
    }
    return count;
}

static ssize_t aw8697_seq_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int databuf[2] = {0, 0};

    if (!aw8697->ram_init)
       return count;

    if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        pr_debug("%s: seq%d=0x%x\n", __FUNCTION__, databuf[0], databuf[1]);
        mutex_lock(&aw8697->lock);
        aw8697->seq[databuf[0]] = (unsigned char)databuf[1];
        aw8697_haptic_set_wav_seq(aw8697, (unsigned char)databuf[0],
                aw8697->seq[databuf[0]]);
        mutex_unlock(&aw8697->lock);
    }
    return count;
}

static ssize_t aw8697_loop_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    size_t count = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;

    for(i=0; i<AW8697_SEQUENCER_LOOP_SIZE; i++) {
        aw8697_i2c_read(aw8697, AW8697_REG_WAVLOOP1+i, &reg_val);
        aw8697->loop[i*2+0] = (reg_val>>4)&0x0F;
        aw8697->loop[i*2+1] = (reg_val>>0)&0x0F;

        count += snprintf(buf+count, PAGE_SIZE-count,
                "seq%d loop: 0x%02x\n", i*2+1, aw8697->loop[i*2+0]);
        count += snprintf(buf+count, PAGE_SIZE-count,
                "seq%d loop: 0x%02x\n", i*2+2, aw8697->loop[i*2+1]);
    }
    return count;
}

static ssize_t aw8697_loop_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int databuf[2] = {0, 0};

    if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        pr_debug("%s: seq%d loop=0x%x\n", __FUNCTION__, databuf[0], databuf[1]);
        mutex_lock(&aw8697->lock);
        aw8697->loop[databuf[0]] = (unsigned char)databuf[1];
        aw8697_haptic_set_wav_loop(aw8697, (unsigned char)databuf[0],
                aw8697->loop[databuf[0]]);
        mutex_unlock(&aw8697->lock);
    }

    return count;
}

static ssize_t aw8697_reg_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;
    for(i = 0; i < AW8697_REG_MAX; i ++) {
        if(!(aw8697_reg_access[i]&REG_RD_ACCESS))
           continue;
        aw8697_i2c_read(aw8697, i, &reg_val);
        len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x \n", i, reg_val);
    }
    return len;
}

static ssize_t aw8697_reg_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int databuf[2] = {0, 0};

    if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        aw8697_i2c_write(aw8697, (unsigned char)databuf[0], (unsigned char)databuf[1]);
    }

    return count;
}

static ssize_t aw8697_rtp_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "rtp play: %d\n", aw8697->rtp_cnt);

    return len;
}
#define AUDIO_READY_STATUS  1024
#define FACTORY_MODE_NORMAL_RTP_NUMBER  76
#define FACTORY_MODE_HIGH_TEMP_RTP_NUMBER  75

static ssize_t aw8697_rtp_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;
    int rtp_is_going_on = 0;

    if (!aw8697->ram_init)
       return count;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0) {
        pr_info("%s: kstrtouint fail\n", __FUNCTION__);
        return rc;
    }
    pr_info("%s: rtp[%d]\n", __FUNCTION__,val);

    /*OP add for juge rtp on begin*/
    rtp_is_going_on = aw8697_haptic_juge_RTP_is_going_on(aw8697);
    if (rtp_is_going_on && (val == AUDIO_READY_STATUS))
    {
        pr_info("%s: seem audio status rtp[%d]\n", __FUNCTION__,val);
        return count;
    }
    /*OP add for juge rtp on end*/

    if (!aw8697->haptic_ready && (val == AUDIO_READY_STATUS)) {
        pr_info("%s: invalid audio ready\n", __FUNCTION__);
        return count;
    }

    if (val != FACTORY_MODE_NORMAL_RTP_NUMBER
        && val != FACTORY_MODE_HIGH_TEMP_RTP_NUMBER
        && val != 0
        && !aw8697->ignore_sync) {
        if (val == AUDIO_READY_STATUS)
           aw8697->audio_ready = true;
        else
           aw8697->haptic_ready = true;

       pr_info("%s:audio[%d]and haptic[%d] ready\n", __FUNCTION__,
                        aw8697->audio_ready, aw8697->haptic_ready);
       if (aw8697->haptic_ready && !aw8697->audio_ready) {
           aw8697->pre_haptic_number = val;
       }
       if (!aw8697->audio_ready || !aw8697->haptic_ready) {
           return count;
       }
    }
    if (val == AUDIO_READY_STATUS && aw8697->pre_haptic_number) {
        pr_info("pre_haptic_number:%d\n",aw8697->pre_haptic_number);
       val = aw8697->pre_haptic_number;
    }
    if (!val) {
      aw8697_op_clean_status(aw8697);
    }
    aw8697->audio_ready = false;
    aw8697->haptic_ready = false;
    aw8697->pre_haptic_number = 0;
    mutex_lock(&aw8697->lock);
    aw8697_haptic_stop(aw8697);
    aw8697_haptic_set_rtp_aei(aw8697, false);
    aw8697_interrupt_clear(aw8697);
    if(val < (sizeof(aw8697_rtp_name)/AW8697_RTP_NAME_MAX)) {
        aw8697->rtp_file_num = val;
        if(val) {
            schedule_work(&aw8697->rtp_work);
        }
    } else {
        pr_err("%s: rtp_file_num 0x%02x over max value \n", __func__, aw8697->rtp_file_num);
    }
    mutex_unlock(&aw8697->lock);

    return count;
}

static ssize_t aw8697_ram_update_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    ssize_t len = 0;

    len += snprintf(buf+len, PAGE_SIZE-len, "sram update mode\n");
    return len;
}

static ssize_t aw8697_ram_update_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    if(val) {
        aw8697_ram_update(aw8697);
    }
    return count;
}

static ssize_t aw8697_f0_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;

    mutex_lock(&aw8697->lock);
    aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
    aw8697_haptic_get_f0(aw8697);
    aw8697->haptic_real_f0 = aw8697->f0/10;
    mutex_unlock(&aw8697->lock);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", aw8697->f0/10);
    return len;
}

static ssize_t aw8697_f0_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;
    aw8697->haptic_real_f0 = val;
    pr_info("aw8697->haptic_real_f0:%d\n", aw8697->haptic_real_f0);
    return count;
}

static ssize_t aw8697_cali_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;
    mutex_lock(&aw8697->lock);
    aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
    aw8697_haptic_get_f0(aw8697);
    mutex_unlock(&aw8697->lock);
    len += snprintf(buf+len, PAGE_SIZE-len, "aw8697 cali f0 = %d\n", aw8697->f0);
    return len;
}

static ssize_t aw8697_cali_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;
    pr_info("%s:val:%d\n", __func__, val);
    if(val) {
        mutex_lock(&aw8697->lock);
        aw8697_haptic_f0_calibration(aw8697);
        mutex_unlock(&aw8697->lock);
    }
    return count;
}

static ssize_t aw8697_ignore_sync_tore(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
	static unsigned int val_pre;
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val_pre != val)
		pr_info("%s:val:%d\n", __func__, val);
	val_pre = val;
	aw8697->ignore_sync = val;
	return count;
}

static ssize_t aw8697_ignore_sync_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);

	return snprintf(buf, PAGE_SIZE, "%lld\n", aw8697->ignore_sync);
}

static ssize_t aw8697_cont_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;
    aw8697_haptic_read_cont_f0(aw8697);
    len += snprintf(buf+len, PAGE_SIZE-len, "aw8697 cont f0 = %d\n", aw8697->cont_f0);
    return len;
}

static ssize_t aw8697_cont_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;
    pr_info("%s:val:%d\n", __func__, val);
    mutex_lock(&aw8697->lock);
    aw8697_haptic_stop(aw8697);
    if(val) {
        aw8697_haptic_cont(aw8697);
    }
    mutex_unlock(&aw8697->lock);
    return count;
}


static ssize_t aw8697_cont_td_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "aw8697 cont delay time = 0x%04x\n", aw8697->cont_td);
    return len;
}

static ssize_t aw8697_cont_td_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int databuf[1] = {0};
    if(1 == sscanf(buf, "%x", &databuf[0])) {
        aw8697->cont_td = databuf[0];
        aw8697_i2c_write(aw8697, AW8697_REG_TD_H, (unsigned char)(databuf[0]>>8));
        aw8697_i2c_write(aw8697, AW8697_REG_TD_L, (unsigned char)(databuf[0]>>0));
    }
    return count;
}

static ssize_t aw8697_cont_drv_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "aw8697 cont drv level = %d\n", aw8697->cont_drv_lvl);
    len += snprintf(buf+len, PAGE_SIZE-len, "aw8697 cont drv level overdrive= %d\n", aw8697->cont_drv_lvl_ov);
    return len;
}

static ssize_t aw8697_cont_drv_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int databuf[2] = {0, 0};
    if(2 == sscanf(buf, "%d %d", &databuf[0], &databuf[1])) {
        aw8697->cont_drv_lvl = databuf[0];
        aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL, aw8697->cont_drv_lvl);
        aw8697->cont_drv_lvl_ov = databuf[1];
        aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL_OV, aw8697->cont_drv_lvl_ov);
    }
    return count;
}

static ssize_t aw8697_cont_num_brk_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "aw8697 cont break num = %d\n", aw8697->cont_num_brk);
    return len;
}

static ssize_t aw8697_cont_num_brk_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int databuf[1] = {0};
    if(1 == sscanf(buf, "%d", &databuf[0])) {
        aw8697->cont_num_brk = databuf[0];
        if(aw8697->cont_num_brk > 7) {
            aw8697->cont_num_brk = 7;
        }
        aw8697_i2c_write_bits(aw8697, AW8697_REG_BEMF_NUM,
                AW8697_BIT_BEMF_NUM_BRK_MASK, aw8697->cont_num_brk);
    }
    return count;
}

static ssize_t aw8697_cont_zc_thr_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "aw8697 cont zero cross thr = 0x%04x\n", aw8697->cont_zc_thr);
    return len;
}

static ssize_t aw8697_cont_zc_thr_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int databuf[1] = {0};
    if(1 == sscanf(buf, "%x", &databuf[0])) {
        aw8697->cont_zc_thr = databuf[0];
        aw8697_i2c_write(aw8697, AW8697_REG_ZC_THRSH_H, (unsigned char)(databuf[0]>>8));
        aw8697_i2c_write(aw8697, AW8697_REG_ZC_THRSH_L, (unsigned char)(databuf[0]>>0));
    }
    return count;
}

static ssize_t aw8697_vbat_monitor_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;

    mutex_lock(&aw8697->lock);
    aw8697_haptic_stop(aw8697);
    aw8697_haptic_get_vbat(aw8697);
    len += snprintf(buf+len, PAGE_SIZE-len, "vbat=%dmV\n", aw8697->vbat);
    mutex_unlock(&aw8697->lock);

    return len;
}

static ssize_t aw8697_vbat_monitor_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    return count;
}

static ssize_t aw8697_lra_resistance_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;
    unsigned char reg_val = 0;

    mutex_lock(&aw8697->lock);
    aw8697_haptic_stop(aw8697);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_EN);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);


    aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
            AW8697_BIT_ANACTRL_HD_PD_MASK, AW8697_BIT_ANACTRL_HD_HZ_EN);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_D2SCFG,
            AW8697_BIT_D2SCFG_CLK_ADC_MASK, AW8697_BIT_D2SCFG_CLK_ASC_1P5MHZ);

    aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
            AW8697_BIT_DETCTRL_RL_OS_MASK, AW8697_BIT_DETCTRL_RL_DETECT);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
            AW8697_BIT_DETCTRL_DIAG_GO_MASK, AW8697_BIT_DETCTRL_DIAG_GO_ENABLE);
    msleep(3);
    aw8697_i2c_read(aw8697, AW8697_REG_RLDET, &reg_val);
    aw8697->lra = 298 * reg_val;
    len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", aw8697->lra/100);


    aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
            AW8697_BIT_ANACTRL_HD_PD_MASK, AW8697_BIT_ANACTRL_HD_PD_EN);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_D2SCFG,
            AW8697_BIT_D2SCFG_CLK_ADC_MASK, AW8697_BIT_D2SCFG_CLK_ASC_6MHZ);

    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_OFF);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
    mutex_unlock(&aw8697->lock);

    return len;
}

static ssize_t aw8697_lra_resistance_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    return count;
}

static ssize_t aw8697_auto_boost_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "auto_boost=%d\n", aw8697->auto_boost);
    return len;
}


static ssize_t aw8697_auto_boost_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;
    pr_info("%s:val:%d\n", __func__, val);
    mutex_lock(&aw8697->lock);
    aw8697_haptic_stop(aw8697);
    aw8697_haptic_auto_boost_config(aw8697, val);
    mutex_unlock(&aw8697->lock);

    return count;
}

static ssize_t aw8697_prctmode_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;
    unsigned char reg_val = 0;

    aw8697_i2c_read(aw8697, AW8697_REG_RLDET, &reg_val);

    len += snprintf(buf+len, PAGE_SIZE-len, "prctmode=%d\n", reg_val&0x20);
    return len;
}


static ssize_t aw8697_prctmode_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    #ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
    #else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
    #endif
    unsigned int databuf[2] = {0, 0};
    unsigned int addr=0;
    unsigned int val=0;
    if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        addr = databuf[0];
        val=databuf[1];
        mutex_lock(&aw8697->lock);
        aw8697_haptic_swicth_motorprotect_config(aw8697, addr, val);
        mutex_unlock(&aw8697->lock);
   }
    return count;
}
static ssize_t aw8697_trig_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;
    unsigned char i = 0;
    for(i=0; i<AW8697_TRIG_NUM; i++) {
        len += snprintf(buf+len, PAGE_SIZE-len,
            "trig%d: enable=%d, default_level=%d, dual_edge=%d, frist_seq=%d, second_seq=%d\n",
            i+1, aw8697->trig[i].enable, aw8697->trig[i].default_level, aw8697->trig[i].dual_edge,
            aw8697->trig[i].frist_seq, aw8697->trig[i].second_seq);
    }

    return len;
}

static ssize_t aw8697_trig_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    #ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
    #else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
    #endif
    unsigned int databuf[6] = {0};
    if(sscanf(buf, "%d %d %d %d %d %d",
            &databuf[0], &databuf[1], &databuf[2], &databuf[3], &databuf[4], &databuf[5])) {
        pr_info("%s: %d, %d, %d, %d, %d, %d\n", __func__,
            databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5]);
        if(databuf[0] > 3) {
            databuf[0] = 3;
        }
        if(databuf[0] > 0) {
            databuf[0] -= 1;
        }
        aw8697->trig[databuf[0]].enable = databuf[1];
        aw8697->trig[databuf[0]].default_level = databuf[2];
        aw8697->trig[databuf[0]].dual_edge = databuf[3];
        aw8697->trig[databuf[0]].frist_seq = databuf[4];
        aw8697->trig[databuf[0]].second_seq = databuf[5];
        mutex_lock(&aw8697->lock);
        aw8697_haptic_trig_param_config(aw8697);
        aw8697_haptic_trig_enable_config(aw8697);
        mutex_unlock(&aw8697->lock);
   }
    return count;
}

static ssize_t aw8697_ram_vbat_comp_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;

    len += snprintf(buf+len, PAGE_SIZE-len, "ram_vbat_comp=%d\n", aw8697->ram_vbat_comp);

    return len;
}


static ssize_t aw8697_ram_vbat_comp_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    mutex_lock(&aw8697->lock);
    if(val) {
        aw8697->ram_vbat_comp = AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE;
    } else {
        aw8697->ram_vbat_comp = AW8697_HAPTIC_RAM_VBAT_COMP_DISABLE;
    }
    mutex_unlock(&aw8697->lock);
    return count;
}

static ssize_t aw8697_haptic_osc_data_show(struct device *dev, struct device_attribute *attr,
char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", aw8697->lra_calib_data);

	return len;
}


static ssize_t aw8697_osc_data_store(struct device *dev, struct device_attribute *attr,
const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	pr_info("%s: vqal:\n", __func__, val);
	if (rc < 0)
	return rc;
	mutex_lock(&aw8697->lock);
	pr_info("%s: val:%d\n", __func__, val);
	if(val)
		aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, (char)val);
	mutex_unlock(&aw8697->lock);
	return count;
}

static ssize_t aw8697_osc_cali_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", aw8697->microsecond);
	return len;
}

static ssize_t aw8697_osc_cali_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int val = 0;

    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    mutex_lock(&aw8697->lock);
    if(val == 3){
        aw8697_haptic_osc_set_default_trim(aw8697);
        aw8697_rtp_osc_calibration(aw8697);
        aw8697_rtp_trim_lra_calibration(aw8697);
    }
   if(val == 1)
      aw8697_rtp_osc_calibration(aw8697);

    mutex_unlock(&aw8697->lock);
    return count;
}

static ssize_t aw8697_haptic_audio_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", aw8697->haptic_audio.ctr.cnt);
    return len;
}

static ssize_t aw8697_haptic_audio_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int databuf[6] = {0};
    struct haptic_ctr *hap_ctr = NULL;

    if (aw8697->rtp_on)
       return count;
    if (!aw8697->ram_init)
       return count;

    if (!aw8697->ram_init)
       return count;
    if(6 == sscanf(buf, "%d %d %d %d %d %d", &databuf[0], &databuf[1], &databuf[2],
            &databuf[3], &databuf[4], &databuf[5])) {
	if (databuf[2]) {
		pr_debug("%s: cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d\n",
		        __func__, databuf[0], databuf[1], databuf[2], databuf[3],
		        databuf[4], databuf[5]);
	}

	hap_ctr = (struct haptic_ctr *)kzalloc(
		sizeof(struct haptic_ctr), GFP_KERNEL);
	if (hap_ctr== NULL ) {
		pr_err("%s: kzalloc memory fail\n", __func__);
		return count;
	}
        mutex_lock(&aw8697->haptic_audio.lock);
        hap_ctr->cnt = (unsigned char)databuf[0];
        hap_ctr->cmd = (unsigned char)databuf[1];
        hap_ctr->play = (unsigned char)databuf[2];
        hap_ctr->wavseq = (unsigned char)databuf[3];
        hap_ctr->loop = (unsigned char)databuf[4];
        hap_ctr->gain = (unsigned char)databuf[5];
        aw8697_haptic_audio_ctr_list_insert(&aw8697->haptic_audio, hap_ctr);

        if(hap_ctr->cmd == 0xff) {
            pr_debug("%s: haptic_audio stop\n", __func__);
            if(hrtimer_active(&aw8697->haptic_audio.timer)) {
                pr_debug("%s: cancel haptic_audio_timer\n", __func__);
                hrtimer_cancel(&aw8697->haptic_audio.timer);
                aw8697->haptic_audio.ctr.cnt = 0;
                aw8697_haptic_audio_off(aw8697);
            }
        } else {
            if(hrtimer_active(&aw8697->haptic_audio.timer)) {
            } else {
                pr_debug("%s: start haptic_audio_timer\n", __func__);
                aw8697_haptic_audio_init(aw8697);
                hrtimer_start(&aw8697->haptic_audio.timer,
                        ktime_set(aw8697->haptic_audio.delay_val/1000000,
                                (aw8697->haptic_audio.delay_val%1000000)*1000),
                        HRTIMER_MODE_REL);
            }
        }
        mutex_unlock(&aw8697->haptic_audio.lock);
	kfree(hap_ctr);
    }
    return count;
}


static ssize_t aw8697_haptic_audio_time_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "haptic_audio.delay_val=%dus\n", aw8697->haptic_audio.delay_val);
    len += snprintf(buf+len, PAGE_SIZE-len, "haptic_audio.timer_val=%dus\n", aw8697->haptic_audio.timer_val);
    return len;
}

static ssize_t aw8697_haptic_audio_time_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int databuf[2] = {0};

    if (aw8697->rtp_on)
       return count;
    if (!aw8697->ram_init)
       return count;

    if(2 == sscanf(buf, "%d %d", &databuf[0], &databuf[1])) {
        aw8697->haptic_audio.delay_val = databuf[0];
        aw8697->haptic_audio.timer_val = databuf[1];
    }
    return count;
}

static ssize_t aw8697_haptic_audio_tp_time_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	struct tp *tp = &(aw8697->haptic_audio.tp);
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "tp->press_delay_min=%dus\n", tp->press_delay_min);
	len += snprintf(buf+len, PAGE_SIZE-len, "tp->press_delay_max=%dus\n", tp->press_delay_max);
	len += snprintf(buf+len, PAGE_SIZE-len, "tp->release_delay_max=%dus\n", tp->release_delay_max);
	len += snprintf(buf+len, PAGE_SIZE-len, "tp->no_play_cnt_max=%d\n", tp->no_play_cnt_max);
	return len;
}

static ssize_t aw8697_haptic_audio_tp_time_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[4] = {0};
	struct tp *tp = &(aw8697->haptic_audio.tp);

	if (aw8697->rtp_on)
		return count;
	if (!aw8697->ram_init)
		return count;

	if(4 == sscanf(buf, "%d %d %d %d", &databuf[0], &databuf[1], &databuf[2], &databuf[3])) {
		tp->press_delay_min = databuf[0];
		tp->press_delay_max = databuf[1];
		tp->release_delay_max = databuf[2];
		tp->no_play_cnt_max= databuf[3];
		pr_info("tp->press_delay_min=%dus\n", tp->press_delay_min);
		pr_info("tp->press_delay_max=%dus\n", tp->press_delay_max);
		pr_info("tp->release_delay_max=%dus\n", tp->release_delay_max);
		pr_info("tp->no_play_cnt_max=%dus\n", tp->no_play_cnt_max);
	}
	return count;
}


static ssize_t aw8697_haptic_audio_tp_input_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	//struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	//struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	//struct led_classdev *cdev = dev_get_drvdata(dev);
	//struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	return len;
}

static ssize_t aw8697_haptic_audio_tp_input_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	struct tp_input_info tp_input;
	struct tp *tp = &(aw8697->haptic_audio.tp);
	struct haptic_audio *haptic_audio = NULL;
	struct haptic_audio_trust_zone *p_tmp = NULL;

	if (aw8697->rtp_on)
		return count;
	if (!aw8697->ram_init)
		return count;

	haptic_audio = &(aw8697->haptic_audio);

	if (count != sizeof(struct tp_input_info)) {
		pr_err("%s: unmatch buf len, node_len=%d, struct len=%d\n",
			__func__, count, sizeof(struct tp_input_info));
		return count;
	}

	memcpy(&tp_input, buf, sizeof(struct tp_input_info));
	pr_debug("%s: id[%d]: status=%d, x=%d, y=%d\n",
		__func__, tp_input.id, tp_input.status, tp_input.x, tp_input.y);

	mutex_lock(&aw8697->haptic_audio.lock);
	if((AW8697_HAPTIC_TP_ST_PRESS == tp_input.status) &&
		(tp_input.status != tp->id[tp_input.id].pt_info.status)) {
		tp->tp_ai_check_flag = 1;
		tp->id[tp_input.id].tp_ai_match_flag = 0;
		list_for_each_entry(p_tmp, &(haptic_audio->list), list) {
			if ((tp_input.x+AW8697_HAPTIC_AI_X_JITTER*p_tmp->w/AW8697_HAPTIC_AI_X_DFT_W > p_tmp->x) &&
				(tp_input.x < p_tmp->x+p_tmp->w+AW8697_HAPTIC_AI_X_JITTER*p_tmp->w/AW8697_HAPTIC_AI_X_DFT_W) &&
				(tp_input.y+AW8697_HAPTIC_AI_Y_JITTER*p_tmp->h/AW8697_HAPTIC_AI_Y_DFT_H > p_tmp->y) &&
				(tp_input.y < p_tmp->y+p_tmp->h+AW8697_HAPTIC_AI_Y_JITTER*p_tmp->h/AW8697_HAPTIC_AI_Y_DFT_H)) {
				pr_debug("%s: tp input point[%d, %04d, %04d] is in the ai trust zone[%d, %04d, %04d, %04d, %04d]\n",
					__func__, tp_input.id, tp_input.x, tp_input.y,
					p_tmp->level, p_tmp->x, p_tmp->y, p_tmp->w, p_tmp->h);
				tp->id[tp_input.id].tp_ai_match_flag = 1;
				do_gettimeofday(&tp->id[AW8697_HAPTIC_TP_ID_MAX].t_press);
				tp->virtual_id = tp_input.id;
				tp->id[AW8697_HAPTIC_TP_ID_MAX].pt_info.id = tp_input.id;
				tp->id[AW8697_HAPTIC_TP_ID_MAX].pt_info.x = tp_input.x;
				tp->id[AW8697_HAPTIC_TP_ID_MAX].pt_info.y = tp_input.y;
				tp->id[AW8697_HAPTIC_TP_ID_MAX].pt_info.status = tp_input.status;
				tp->id[AW8697_HAPTIC_TP_ID_MAX].pt_info.touch_flag = AW8697_HAPTIC_TP_TOUCH_VAIL;
				tp->id[AW8697_HAPTIC_TP_ID_MAX].pt_info.touch_outside_tz_flag = AW8697_HAPTIC_TP_TOUCH_VAIL;
				tp->id[AW8697_HAPTIC_TP_ID_MAX].tp_flag = AW8697_HAPTIC_TP_PRESS;
				tp->id[AW8697_HAPTIC_TP_ID_MAX].press_flag = AW8697_HAPTIC_TP_PRESS;
				tp->id[AW8697_HAPTIC_TP_ID_MAX].release_flag = AW8697_HAPTIC_TP_NULL;
				tp->id[AW8697_HAPTIC_TP_ID_MAX].no_play_cnt = 0;
				tp->id[AW8697_HAPTIC_TP_ID_MAX].press_no_vibrate_flag = 0;
				break;
			}
		}
		if (tp->id[tp_input.id].tp_ai_match_flag == 0) {
			pr_debug("%s: tp input point[%d, %04d, %04d] is out the trust zone\n",
				__func__, tp_input.id, tp_input.x, tp_input.y);
		}
		tp->id[tp_input.id].pt_info.id = tp_input.id;
		tp->id[tp_input.id].pt_info.x = tp_input.x;
		tp->id[tp_input.id].pt_info.y = tp_input.y;
		tp->id[tp_input.id].pt_info.status = tp_input.status;
		tp->id[tp_input.id].pt_info.touch_flag = AW8697_HAPTIC_TP_TOUCH_VAIL;
		tp->id[tp_input.id].pt_info.touch_outside_tz_flag = AW8697_HAPTIC_TP_TOUCH_VAIL;
		tp->id[tp_input.id].tp_flag = AW8697_HAPTIC_TP_PRESS;
		tp->id[tp_input.id].press_flag = AW8697_HAPTIC_TP_PRESS;
		tp->id[tp_input.id].release_flag = AW8697_HAPTIC_TP_NULL;
		tp->id[tp_input.id].no_play_cnt = 0;
		tp->id[tp_input.id].press_no_vibrate_flag = 0;
		do_gettimeofday(&tp->id[tp_input.id].t_press);
		pr_debug("%s: tp_press_release: status=%d, flag=%d",
			__func__, tp->id[tp_input.id].pt_info.status, tp->id[tp_input.id].tp_flag);
	}

	if((AW8697_HAPTIC_TP_ST_RELEASE == tp_input.status) &&
		(tp_input.status != tp->id[tp_input.id].pt_info.status)) {
		list_for_each_entry(p_tmp, &(haptic_audio->list), list) {
			if ((tp->id[tp_input.id].pt_info.x+AW8697_HAPTIC_AI_X_JITTER*p_tmp->w/AW8697_HAPTIC_AI_X_DFT_W > p_tmp->x) &&
				(tp->id[tp_input.id].pt_info.x < p_tmp->x+p_tmp->w+AW8697_HAPTIC_AI_X_JITTER*p_tmp->w/AW8697_HAPTIC_AI_X_DFT_W) &&
				(tp->id[tp_input.id].pt_info.y+AW8697_HAPTIC_AI_Y_JITTER*p_tmp->h/AW8697_HAPTIC_AI_Y_DFT_H > p_tmp->y) &&
				(tp->id[tp_input.id].pt_info.y < p_tmp->y+p_tmp->h+AW8697_HAPTIC_AI_Y_JITTER*p_tmp->h/AW8697_HAPTIC_AI_Y_DFT_H)) {
				pr_debug("%s: tp input point[%d, %04d, %04d] up is in the ai trust zone[%d, %04d, %04d, %04d, %04d]\n",
					__func__, tp->id[tp_input.id].pt_info.id, tp->id[tp_input.id].pt_info.x, tp->id[tp_input.id].pt_info.y,
					p_tmp->level, p_tmp->x, p_tmp->y, p_tmp->w, p_tmp->h);
				if(tp->virtual_id == tp_input.id) {
					tp->id[AW8697_HAPTIC_TP_ID_MAX].pt_info.id = tp->id[tp_input.id].pt_info.id;
					tp->id[AW8697_HAPTIC_TP_ID_MAX].pt_info.x = tp->id[tp_input.id].pt_info.x;
					tp->id[AW8697_HAPTIC_TP_ID_MAX].pt_info.y = tp->id[tp_input.id].pt_info.y;
					tp->id[AW8697_HAPTIC_TP_ID_MAX].pt_info.status = tp_input.status;
					tp->id[AW8697_HAPTIC_TP_ID_MAX].tp_flag = AW8697_HAPTIC_TP_RELEASE;
					tp->id[AW8697_HAPTIC_TP_ID_MAX].release_flag = AW8697_HAPTIC_TP_RELEASE;
					do_gettimeofday(&tp->id[AW8697_HAPTIC_TP_ID_MAX].t_release);
				}
				break;
			}
		}
		tp->id[tp_input.id].pt_info.status = tp_input.status;
		tp->id[tp_input.id].tp_flag = AW8697_HAPTIC_TP_RELEASE;
		tp->id[tp_input.id].release_flag = AW8697_HAPTIC_TP_RELEASE;
		do_gettimeofday(&tp->id[tp_input.id].t_release);
		pr_debug("%s: tp input point[%d, %04d, %04d] up to [%d, %04d, %04d] \n",
			__func__, tp_input.id, tp->id[tp_input.id].pt_info.x, tp->id[tp_input.id].pt_info.y,
			tp_input.id, tp_input.x, tp_input.y);
		pr_debug("%s: tp_press_release: status=%d, flag=%d",
			__func__, tp->id[tp_input.id].pt_info.status, tp->id[tp_input.id].tp_flag);
	}
	mutex_unlock(&aw8697->haptic_audio.lock);

	return count;
}

static ssize_t aw8697_haptic_audio_ai_input_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	struct haptic_audio *haptic_audio = NULL;
	struct haptic_audio_trust_zone *p_tmp = NULL;
	char *p_ai_data = NULL;
	ssize_t len = 0;
	int i = 0;
	uint8_t ai_tz_num = 0;

	mutex_lock(&aw8697->haptic_audio.lock);
	haptic_audio = &(aw8697->haptic_audio);
	ai_tz_num = 0;

	list_for_each_entry(p_tmp, &(haptic_audio->list), list) {
		ai_tz_num ++;
	}
	pr_debug("%s: ai_tz_num=%d\n", __func__, ai_tz_num);

	p_ai_data = (char *) kzalloc(sizeof(uint8_t) + ai_tz_num * sizeof(struct ai_trust_zone), GFP_KERNEL);
	if (!p_ai_data) {
		pr_err("%s: p_ai_data error allocating memory\n", __func__);
		mutex_unlock(&aw8697->haptic_audio.lock);
		return len;
	}

	memcpy(&p_ai_data[len], &ai_tz_num, sizeof(ai_tz_num));
	len += sizeof(ai_tz_num);

	list_for_each_entry(p_tmp, &(haptic_audio->list), list) {
		haptic_audio->output_tz_info[i].level = p_tmp->level;
		haptic_audio->output_tz_info[i].x = p_tmp->x;
		haptic_audio->output_tz_info[i].y = p_tmp->y;
		haptic_audio->output_tz_info[i].w = p_tmp->w;
		haptic_audio->output_tz_info[i].h = p_tmp->h;
		memcpy(&p_ai_data[len], &haptic_audio->output_tz_info[i], sizeof(struct trust_zone_info));
		len += sizeof(struct trust_zone_info);
		pr_debug("%s: trust zone [%d]: level=%d, x=%d, y=%d, w=%d, h=%d\n",
			__func__, i, haptic_audio->output_tz_info[i].level,
			haptic_audio->output_tz_info[i].x, haptic_audio->output_tz_info[i].y,
			haptic_audio->output_tz_info[i].w, haptic_audio->output_tz_info[i].h);
		i ++;
	}
	memcpy(buf, p_ai_data, len);

	kfree(p_ai_data);
	mutex_unlock(&aw8697->haptic_audio.lock);

	return len;
}

static ssize_t aw8697_haptic_audio_ai_input_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	int ret = -1;
	unsigned int i = 0;
	int match = 0;
	struct ai_trust_zone *ai_tz = NULL;
	struct trust_zone_info *ai_tz_info = NULL;
	struct haptic_audio *haptic_audio = NULL;
	struct haptic_audio_trust_zone *p_tmp = NULL;

    if (aw8697->rtp_on)
       return count;
    if (!aw8697->ram_init)
       return count;

	haptic_audio = &(aw8697->haptic_audio);

	if (count != sizeof(struct ai_trust_zone)) {
		pr_err("%s: unmatch buf len, node_len=%d, struct len=%d\n",
			__func__, count, sizeof(struct ai_trust_zone));
		return count;
	}

	mutex_lock(&aw8697->haptic_audio.lock);
	ai_tz = kzalloc(sizeof(struct ai_trust_zone), GFP_KERNEL);
	if (!ai_tz) {
		pr_err("%s: ai_tz error allocating memory\n", __func__);
		mutex_unlock(&aw8697->haptic_audio.lock);
		return count;
	}
	memcpy(ai_tz, buf, sizeof(struct ai_trust_zone));
	pr_debug("%s: ai_tz num=%d\n", __func__, ai_tz->num);

	ai_tz_info = kzalloc(ai_tz->num * sizeof(struct trust_zone_info), GFP_KERNEL);
	if (!ai_tz_info) {
		pr_err("%s: ai_tz_info error allocating memory\n", __func__);
		kfree(ai_tz);
		mutex_unlock(&aw8697->haptic_audio.lock);
		return count;
	}
	ret = copy_from_user(ai_tz_info, ai_tz->tz_info, ai_tz->num * sizeof(struct trust_zone_info));
	if (ret) {
		kfree(ai_tz_info);
		kfree(ai_tz);
		dev_err(aw8697->dev, "%s: copy from user fail\n", __func__);
		mutex_unlock(&aw8697->haptic_audio.lock);
		return count;
	}

	aw8697->haptic_audio.tz_high_num = 2;
	aw8697->haptic_audio.hap_cnt_outside_tz = 0;
#ifndef SCORE_MODE
	/* clear unused list link */
	aw8697_haptic_audio_tz_list_clear(&aw8697->haptic_audio);
#endif

	if(haptic_audio->tz_num >= 5) {
		haptic_audio->tz_num = 0;
		aw8697_haptic_audio_tz_score_list_clear(haptic_audio);
	}

	for (i=0; i<ai_tz->num; i++) {
		pr_debug("%s: trust zone [%d]: level=%d, x=%d, y=%d, w=%d, h=%d\n",
			__func__, i, ai_tz_info[i].level,
			ai_tz_info[i].x, ai_tz_info[i].y,
			ai_tz_info[i].w, ai_tz_info[i].h);
		match = 0;

		if(haptic_audio->tz_init)
			ai_tz_info[i].level = 0;
		else
			ai_tz_info[i].level = TRUST_LEVEL;

		if(haptic_audio->tz_init){
			list_for_each_entry(p_tmp, &haptic_audio->list, list) {
				if(aw8697_haptic_audio_tz_list_match(p_tmp,&ai_tz_info[i])){
					match = 1;
					break;
				}
			}
			if(match)
				continue;
		}

		if(haptic_audio->tz_init){
			list_for_each_entry(p_tmp, &haptic_audio->score_list, list) {
				if(aw8697_haptic_audio_tz_list_match(p_tmp,&ai_tz_info[i])){
					match = 1;
					break;
				}
			}
		}
		if(match != 1)
			aw8697_haptic_audio_tz_list_insert(haptic_audio, &ai_tz_info[i]);
	}
	haptic_audio->tz_init = 1;
	aw8697_haptic_audio_tz_list_show(haptic_audio);

	kfree(ai_tz_info);
	kfree(ai_tz);

	mutex_unlock(&aw8697->haptic_audio.lock);
	return count;
}

static ssize_t aw8697_haptic_audio_tp_size_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	struct haptic_audio_tp_size *tp_size = NULL;
	ssize_t len = 0;

	tp_size = &(aw8697->haptic_audio.tp_size);

	len += snprintf(buf+len, PAGE_SIZE-len, "tp_size: x=%04d, y=%04d\n", tp_size->x, tp_size->y);

	return len;
}

static ssize_t aw8697_haptic_audio_tp_size_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	struct haptic_audio_tp_size *tp_size = NULL;

    if (aw8697->rtp_on)
       return count;
    if (!aw8697->ram_init)
       return count;

	tp_size = &(aw8697->haptic_audio.tp_size);

	if (count != sizeof(struct haptic_audio_tp_size)) {
		pr_err("%s: unmatch buf len, node_len=%d, struct len=%d\n",
			__func__, count, sizeof(struct haptic_audio_tp_size));
		return count;
	}

	memcpy(tp_size, buf, sizeof(struct haptic_audio_tp_size));
	pr_info("%s: tp_size: x=%d, y=%d\n",
		__func__, tp_size->x, tp_size->y);


	return count;
}

static ssize_t aw8697_haptic_audio_tz_cnt_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "tz_cnt_thr=%d\n", aw8697->haptic_audio.tz_cnt_thr);
	len += snprintf(buf+len, PAGE_SIZE-len, "tz_cnt_max=%d\n", aw8697->haptic_audio.tz_cnt_max);
	return len;
}

static ssize_t aw8697_haptic_audio_tz_cnt_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int databuf[2] = {0};

    if (aw8697->rtp_on)
       return count;
    if (!aw8697->ram_init)
       return count;

    if(2 == sscanf(buf, "%d %d", &databuf[0], &databuf[1])) {
        aw8697->haptic_audio.tz_cnt_thr = databuf[0];
        aw8697->haptic_audio.tz_cnt_max = databuf[1];
    }
    return count;
}


static ssize_t aw8697_haptic_audio_hap_cnt_max_outside_tz_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "hap_cnt_max_outside_tz=%d\n", aw8697->haptic_audio.hap_cnt_max_outside_tz);
	return len;
}

static ssize_t aw8697_haptic_audio_hap_cnt_max_outside_tz_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned int databuf[1] = {0};

    if (aw8697->rtp_on)
       return count;
    if (!aw8697->ram_init)
       return count;

    if(1 == sscanf(buf, "%d", &databuf[0])) {
        aw8697->haptic_audio.hap_cnt_max_outside_tz = databuf[0];
    }
    return count;
}

static int aw8697_i2c_reads(struct aw8697 *aw8697,
		unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
	int ret = -1;

	ret = i2c_smbus_write_byte(aw8697->i2c, reg_addr);
	if (ret) {
		pr_err("%s: couldn't send request, ret=%d\n",
			__func__, ret);
		return ret;
	}
	ret = i2c_master_recv(aw8697->i2c, buf, len);
	if (ret != len) {
		pr_err("%s: couldn't read registers, return %d bytes\n",
			__func__,  ret);
		return ret;
	}
	return ret;
}

static ssize_t aw8697_haptic_ram_test_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	struct aw8697_container *aw8697_ramtest;
	int i, j= 0;
	unsigned int val = 0;
	int rc = 0;
	unsigned int start_addr;
	unsigned int tmp_len,retries;
	char *pbuf = NULL;

	pr_info("%s enter\n", __func__);
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	start_addr = 0;
	aw8697->ram_test_flag_0 = 0;
	aw8697->ram_test_flag_1 = 0;
	aw8697->ram_test_result = 0;
	tmp_len = 1024;	/*1K*/
	retries= 8;	/*tmp_len * retries  =8*1024*/
	aw8697_ramtest = kzalloc(tmp_len*sizeof(char) +sizeof(int), GFP_KERNEL);
	if (!aw8697_ramtest) {
		pr_err("%s: error allocating memory\n", __func__);
		return count ;
	}
	pbuf = kzalloc(tmp_len*sizeof(char), GFP_KERNEL);
	if (!pbuf) {
		pr_err("%s: Error allocating memory\n", __func__);
		return count;
	}
	mutex_lock(&aw8697->lock);
	disable_irq(gpio_to_irq(aw8697->irq_gpio));
	aw8697_ramtest->len = tmp_len;
	if (val == 1){
			/* RAMINIT Enable */
			aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_EN);
		for (j = 0;j < retries;j++){
			/*test 1	start*/
			memset(aw8697_ramtest->data, 0xff, aw8697_ramtest->len);
			memset(pbuf, 0x00, aw8697_ramtest->len);
			/* write ram 1 test */
			aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRH, start_addr >> 8);
			aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRL, start_addr & 0x00FF);

			aw8697_i2c_writes(aw8697, AW8697_REG_RAMDATA,
							aw8697_ramtest->data, aw8697_ramtest->len);
			/* read ram 1 test */
			aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRH, start_addr>>8);
			aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRL, start_addr&0x00FF);
			aw8697_i2c_reads(aw8697, AW8697_REG_RAMDATA, pbuf, aw8697_ramtest->len);
				for(i = 0;i < aw8697_ramtest->len;i++) {
					if(pbuf[i] != 0xff )
						aw8697->ram_test_flag_1++;
				}
			/*test 1  end*/
			/*test 0 start*/
			memset(aw8697_ramtest->data, 0x00, aw8697_ramtest->len);
			memset(pbuf, 0xff, aw8697_ramtest->len);
			/* write ram 0 test */
			aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRH, start_addr>>8);
			aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRL, start_addr&0x00FF);
			aw8697_i2c_writes(aw8697, AW8697_REG_RAMDATA,
							aw8697_ramtest->data, aw8697_ramtest->len);
			/* read ram 0 test */
			aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRH, start_addr>>8);
			aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRL, start_addr&0x00FF);

			aw8697_i2c_reads(aw8697, AW8697_REG_RAMDATA, pbuf, aw8697_ramtest->len);

				for (i = 0;i < aw8697_ramtest->len;i++) {
					if (pbuf[i] != 0)
						aw8697->ram_test_flag_0++;
				}
			/*test 0 end*/
			start_addr += tmp_len;
		}
		/* RAMINIT Disable */
		 aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_OFF);
	}
	kfree(aw8697_ramtest);
	kfree(pbuf);
	pbuf = NULL;
	aw8697->ram_test_result = !aw8697->ram_test_flag_0 && !aw8697->ram_test_flag_1;
	aw8697_ram_update(aw8697);
	enable_irq(gpio_to_irq(aw8697->irq_gpio));
	mutex_unlock(&aw8697->lock);
	pr_info("ram_test_flag_0:%d,ram_test_flag_1:%d\n",
			aw8697->ram_test_flag_0, aw8697->ram_test_flag_1);
	pr_info("%s exit\n", __func__);
	return count;
}

static ssize_t aw8697_haptic_ram_test_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
					"%d\n", aw8697->ram_test_result);
	return len;
}

static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, aw8697_state_show, aw8697_state_store);
static DEVICE_ATTR(duration, S_IWUSR | S_IRUGO, aw8697_duration_show, aw8697_duration_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IRUGO, aw8697_activate_show, aw8697_activate_store);
static DEVICE_ATTR(activate_mode, S_IWUSR | S_IRUGO, aw8697_activate_mode_show, aw8697_activate_mode_store);
static DEVICE_ATTR(index, S_IWUSR | S_IRUGO, aw8697_index_show, aw8697_index_store);
static DEVICE_ATTR(vmax, S_IWUSR | S_IRUGO, aw8697_vmax_show, aw8697_vmax_store);
static DEVICE_ATTR(gain, S_IWUSR | S_IRUGO, aw8697_gain_show, aw8697_gain_store);
static DEVICE_ATTR(level, S_IWUSR | S_IRUGO, aw8697_level_show, aw8697_level_store);
static DEVICE_ATTR(seq, S_IWUSR | S_IRUGO, aw8697_seq_show, aw8697_seq_store);
static DEVICE_ATTR(loop, S_IWUSR | S_IRUGO, aw8697_loop_show, aw8697_loop_store);
static DEVICE_ATTR(register, S_IWUSR | S_IRUGO, aw8697_reg_show, aw8697_reg_store);
static DEVICE_ATTR(rtp, S_IWUSR | S_IRUGO, aw8697_rtp_show, aw8697_rtp_store);
static DEVICE_ATTR(ram_update, S_IWUSR | S_IRUGO, aw8697_ram_update_show, aw8697_ram_update_store);
static DEVICE_ATTR(rf_hz, S_IWUSR | S_IRUGO, aw8697_f0_show, aw8697_f0_store);
static DEVICE_ATTR(cali, S_IWUSR | S_IRUGO, aw8697_cali_show, aw8697_cali_store);
static DEVICE_ATTR(ignore_store, S_IWUSR | S_IRUGO, aw8697_ignore_sync_show, aw8697_ignore_sync_tore);
static DEVICE_ATTR(cont, S_IWUSR | S_IRUGO, aw8697_cont_show, aw8697_cont_store);
static DEVICE_ATTR(cont_td, S_IWUSR | S_IRUGO, aw8697_cont_td_show, aw8697_cont_td_store);
static DEVICE_ATTR(cont_drv, S_IWUSR | S_IRUGO, aw8697_cont_drv_show, aw8697_cont_drv_store);
static DEVICE_ATTR(cont_num_brk, S_IWUSR | S_IRUGO, aw8697_cont_num_brk_show, aw8697_cont_num_brk_store);
static DEVICE_ATTR(cont_zc_thr, S_IWUSR | S_IRUGO, aw8697_cont_zc_thr_show, aw8697_cont_zc_thr_store);
static DEVICE_ATTR(vbat_monitor, S_IWUSR | S_IRUGO, aw8697_vbat_monitor_show, aw8697_vbat_monitor_store);
static DEVICE_ATTR(lra_resistance, S_IWUSR | S_IRUGO, aw8697_lra_resistance_show, aw8697_lra_resistance_store);
static DEVICE_ATTR(auto_boost, S_IWUSR | S_IRUGO, aw8697_auto_boost_show, aw8697_auto_boost_store);
static DEVICE_ATTR(prctmode, S_IWUSR | S_IRUGO, aw8697_prctmode_show, aw8697_prctmode_store);
static DEVICE_ATTR(trig, S_IWUSR | S_IRUGO, aw8697_trig_show, aw8697_trig_store);
static DEVICE_ATTR(ram_vbat_comp, S_IWUSR | S_IRUGO, aw8697_ram_vbat_comp_show, aw8697_ram_vbat_comp_store);
static DEVICE_ATTR(osc_cali, S_IWUSR | S_IRUGO, aw8697_osc_cali_show, aw8697_osc_cali_store);
static DEVICE_ATTR(haptic_audio, S_IWUSR | S_IRUGO, aw8697_haptic_audio_show, aw8697_haptic_audio_store);
static DEVICE_ATTR(haptic_audio_time, S_IWUSR | S_IRUGO, aw8697_haptic_audio_time_show, aw8697_haptic_audio_time_store);
static DEVICE_ATTR(haptic_audio_tp_time, S_IWUSR | S_IRUGO, aw8697_haptic_audio_tp_time_show, aw8697_haptic_audio_tp_time_store);
static DEVICE_ATTR(haptic_audio_tp_input, S_IWUSR | S_IRUGO, aw8697_haptic_audio_tp_input_show, aw8697_haptic_audio_tp_input_store);
static DEVICE_ATTR(haptic_audio_ai_input, S_IWUSR | S_IRUGO, aw8697_haptic_audio_ai_input_show, aw8697_haptic_audio_ai_input_store);
static DEVICE_ATTR(haptic_audio_tp_size, S_IWUSR | S_IRUGO, aw8697_haptic_audio_tp_size_show, aw8697_haptic_audio_tp_size_store);
static DEVICE_ATTR(haptic_audio_tz_cnt, S_IWUSR | S_IRUGO, aw8697_haptic_audio_tz_cnt_show, aw8697_haptic_audio_tz_cnt_store);
static DEVICE_ATTR(haptic_audio_hap_cnt_max_outside_tz, S_IWUSR | S_IRUGO, aw8697_haptic_audio_hap_cnt_max_outside_tz_show, aw8697_haptic_audio_hap_cnt_max_outside_tz_store);
static DEVICE_ATTR(haptic_osc_data, S_IWUSR | S_IRUGO, aw8697_haptic_osc_data_show, aw8697_osc_data_store);
static DEVICE_ATTR(ram_test, S_IWUSR | S_IRUGO, aw8697_haptic_ram_test_show, aw8697_haptic_ram_test_store);

static struct attribute *aw8697_vibrator_attributes[] = {
    &dev_attr_state.attr,
    &dev_attr_duration.attr,
    &dev_attr_activate.attr,
    &dev_attr_activate_mode.attr,
    &dev_attr_index.attr,
    &dev_attr_vmax.attr,
    &dev_attr_gain.attr,
    &dev_attr_level.attr,
    &dev_attr_seq.attr,
    &dev_attr_loop.attr,
    &dev_attr_register.attr,
    &dev_attr_rtp.attr,
    &dev_attr_ram_update.attr,
    &dev_attr_rf_hz.attr,
    &dev_attr_cali.attr,
    &dev_attr_ignore_store.attr,
    &dev_attr_cont.attr,
    &dev_attr_cont_td.attr,
    &dev_attr_cont_drv.attr,
    &dev_attr_cont_num_brk.attr,
    &dev_attr_cont_zc_thr.attr,
    &dev_attr_vbat_monitor.attr,
    &dev_attr_lra_resistance.attr,
    &dev_attr_auto_boost.attr,
    &dev_attr_prctmode.attr,
    &dev_attr_trig.attr,
    &dev_attr_ram_vbat_comp.attr,
    &dev_attr_osc_cali.attr,
    &dev_attr_haptic_audio.attr,
    &dev_attr_haptic_audio_time.attr,
    &dev_attr_haptic_audio_tp_time.attr,
    &dev_attr_haptic_audio_tp_input.attr,
    &dev_attr_haptic_audio_ai_input.attr,
    &dev_attr_haptic_audio_tp_size.attr,
    &dev_attr_haptic_audio_tz_cnt.attr,
    &dev_attr_haptic_audio_hap_cnt_max_outside_tz.attr,
    &dev_attr_haptic_osc_data.attr,
    &dev_attr_ram_test.attr,
    NULL
};

static struct attribute_group aw8697_vibrator_attribute_group = {
    .attrs = aw8697_vibrator_attributes
};

static enum hrtimer_restart aw8697_vibrator_timer_func(struct hrtimer *timer)
{
    struct aw8697 *aw8697 = container_of(timer, struct aw8697, timer);

    pr_debug("%s enter\n", __func__);
    aw8697->state = 0;
    schedule_work(&aw8697->vibrator_work);
    return HRTIMER_NORESTART;
}

static void aw8697_vibrator_work_routine(struct work_struct *work)
{
    struct aw8697 *aw8697 = container_of(work, struct aw8697, vibrator_work);
    unsigned val = 0;
    pr_debug("%s enter\n", __func__);

    mutex_lock(&aw8697->lock);
    if (aw8697->state) {
       if (aw8697->pm_awake == false) {
           __pm_stay_awake(&aw8697->vibrator_on);
           aw8697->pm_awake = true;
           pr_info("aw8697->pm_awake:%d\n", aw8697->pm_awake);
       }
    } else {
        if (aw8697->pm_awake) {
            __pm_relax(&aw8697->vibrator_on);
            aw8697->pm_awake = false;
            pr_info("aw8697->pm_awake:%d\n", aw8697->pm_awake);
        }
    }
    aw8697_haptic_stop(aw8697);

    if (aw8697->state) {
        if (aw8697->activate_mode == AW8697_HAPTIC_ACTIVATE_RAM_MODE) {
            aw8697_haptic_ram_vbat_comp(aw8697, true);
            aw8697_haptic_play_repeat_seq(aw8697, true);
        } else if(aw8697->activate_mode == AW8697_HAPTIC_ACTIVATE_CONT_MODE) {
            aw8697_haptic_cont(aw8697);
        } else {
        }
        val = aw8697->duration;
        /* run ms timer */
        hrtimer_start(&aw8697->timer,
                  ktime_set(val / 1000, (val % 1000) * 1000000),
                  HRTIMER_MODE_REL);
    }
    mutex_unlock(&aw8697->lock);
}

static int aw8697_vibrator_init(struct aw8697 *aw8697)
{
    int ret = 0;

    pr_info("%s enter\n", __func__);

#ifdef TIMED_OUTPUT
    aw8697->to_dev.name = "vibrator";
    aw8697->to_dev.get_time = aw8697_vibrator_get_time;
    aw8697->to_dev.enable = aw8697_vibrator_enable;

    ret = timed_output_dev_register(&(aw8697->to_dev));
    if ( ret < 0){
        dev_err(aw8697->dev, "%s: fail to create timed output dev\n",
                __func__);
        return ret;
    }
    ret = sysfs_create_group(&aw8697->to_dev.dev->kobj, &aw8697_vibrator_attribute_group);
    if (ret < 0) {
        dev_err(aw8697->dev, "%s error creating sysfs attr files\n", __func__);
        return ret;
    }
#else
    aw8697->cdev.name = "vibrator";
    aw8697->cdev.brightness_get = aw8697_haptic_brightness_get;
    aw8697->cdev.brightness_set = aw8697_haptic_brightness_set;

    ret = devm_led_classdev_register(&aw8697->i2c->dev, &aw8697->cdev);
    if (ret < 0){
        dev_err(aw8697->dev, "%s: fail to create led dev\n",
                __func__);
        return ret;
    }
    ret = sysfs_create_group(&aw8697->cdev.dev->kobj, &aw8697_vibrator_attribute_group);
    if (ret < 0) {
        dev_err(aw8697->dev, "%s error creating sysfs attr files\n", __func__);
        return ret;
     }
#endif
    wakeup_source_init(&aw8697->vibrator_on, "aw_stay_awake");
    hrtimer_init(&aw8697->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    aw8697->timer.function = aw8697_vibrator_timer_func;
    INIT_WORK(&aw8697->vibrator_work, aw8697_vibrator_work_routine);

    INIT_WORK(&aw8697->rtp_work, aw8697_rtp_work_routine);

    mutex_init(&aw8697->lock);
    mutex_init(&aw8697->rtp_lock);

    return 0;
}




/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw8697_interrupt_clear(struct aw8697 *aw8697)
{
    unsigned char reg_val = 0;
    pr_debug("%s enter\n", __func__);
    aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
    pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);
}

static void aw8697_interrupt_setup(struct aw8697 *aw8697)
{
    unsigned char reg_val = 0;

    pr_info("%s enter\n", __func__);

    aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
    pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);

    /* edge int mode */
    aw8697_i2c_write_bits(aw8697, AW8697_REG_DBGCTRL,
            AW8697_BIT_DBGCTRL_INT_MODE_MASK, AW8697_BIT_DBGCTRL_INT_MODE_EDGE);

    /* int enable */
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
            AW8697_BIT_SYSINTM_BSTERR_MASK, AW8697_BIT_SYSINTM_BSTERR_OFF);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
            AW8697_BIT_SYSINTM_OV_MASK, AW8697_BIT_SYSINTM_OV_EN);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
            AW8697_BIT_SYSINTM_UVLO_MASK, AW8697_BIT_SYSINTM_UVLO_EN);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
            AW8697_BIT_SYSINTM_OCD_MASK, AW8697_BIT_SYSINTM_OCD_EN);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
            AW8697_BIT_SYSINTM_OT_MASK, AW8697_BIT_SYSINTM_OT_EN);
}

static irqreturn_t aw8697_irq(int irq, void *data)
{
    struct aw8697 *aw8697 = data;
    unsigned char reg_val = 0;
    unsigned char dbg_val = 0;
    unsigned int buf_len = 0;

    pr_debug("%s enter\n", __func__);

    aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
    pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);
    aw8697_i2c_read(aw8697, AW8697_REG_DBGSTAT, &dbg_val);
    pr_debug("%s: reg DBGSTAT=0x%x\n", __func__, dbg_val);

    if(reg_val & AW8697_BIT_SYSINT_OVI) {
		aw8697_op_clean_status(aw8697);
        pr_err("%s chip ov int error\n", __func__);
    }
    if(reg_val & AW8697_BIT_SYSINT_UVLI) {
		aw8697_op_clean_status(aw8697);
        pr_err("%s chip uvlo int error\n", __func__);
    }
    if(reg_val & AW8697_BIT_SYSINT_OCDI) {
		aw8697_op_clean_status(aw8697);
        pr_err("%s chip over current int error\n", __func__);
    }
    if(reg_val & AW8697_BIT_SYSINT_OTI) {
		aw8697_op_clean_status(aw8697);
        pr_err("%s chip over temperature int error\n", __func__);
    }
    if(reg_val & AW8697_BIT_SYSINT_DONEI) {
		aw8697_op_clean_status(aw8697);
        pr_info("%s chip playback done\n", __func__);
    }

    if(reg_val & AW8697_BIT_SYSINT_FF_AEI) {
        pr_debug("%s: aw8697 rtp fifo almost empty int\n", __func__);
        if(aw8697->rtp_init) {
            while((!aw8697_haptic_rtp_get_fifo_afi(aw8697)) &&
                    (aw8697->play_mode == AW8697_HAPTIC_RTP_MODE)) {
                mutex_lock(&aw8697->rtp_lock);
                pr_debug("%s: aw8697 rtp mode fifo update, cnt=%d\n",
                        __func__, aw8697->rtp_cnt);
                if (!aw8697_rtp) {
                   pr_info("%s:aw8697_rtp is null break\n",
                        __func__);
                   mutex_unlock(&aw8697->rtp_lock);
                   break;
                }
                if((aw8697_rtp->len-aw8697->rtp_cnt) < (aw8697->ram.base_addr>>2)) {
                    buf_len = aw8697_rtp->len-aw8697->rtp_cnt;
                } else {
                    buf_len = (aw8697->ram.base_addr>>2);
                }
                aw8697_i2c_writes(aw8697, AW8697_REG_RTP_DATA,
                        &aw8697_rtp->data[aw8697->rtp_cnt], buf_len);
                aw8697->rtp_cnt += buf_len;
                if(aw8697->rtp_cnt == aw8697_rtp->len) {
                    aw8697_op_clean_status(aw8697);
                    pr_info("%s: rtp update complete\n", __func__);
                    aw8697_haptic_set_rtp_aei(aw8697, false);
                    aw8697->rtp_cnt = 0;
                    aw8697->rtp_init = 0;
                    mutex_unlock(&aw8697->rtp_lock);
                    break;
                }
                mutex_unlock(&aw8697->rtp_lock);
            }
        } else {
            pr_err("%s: aw8697 rtp init = %d, init error\n", __func__, aw8697->rtp_init);
        }
    }

    if(reg_val & AW8697_BIT_SYSINT_FF_AFI) {
        pr_debug("%s: aw8697 rtp mode fifo full empty\n", __func__);
    }

    if(aw8697->play_mode != AW8697_HAPTIC_RTP_MODE) {
        aw8697_haptic_set_rtp_aei(aw8697, false);
    }

    aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
    pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);
    aw8697_i2c_read(aw8697, AW8697_REG_SYSST, &reg_val);
    pr_debug("%s: reg SYSST=0x%x\n", __func__, reg_val);

    pr_debug("%s exit\n", __func__);

    return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw8697_parse_dt(struct device *dev, struct aw8697 *aw8697,
        struct device_node *np) {
    aw8697->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
    if (aw8697->reset_gpio < 0) {
        dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n", __func__);
        return -1;
    } else {
        dev_info(dev, "%s: reset gpio provided ok\n", __func__);
    }
    aw8697->irq_gpio =  of_get_named_gpio(np, "irq-gpio", 0);
    if (aw8697->irq_gpio < 0) {
        dev_err(dev, "%s: no irq gpio provided.\n", __func__);
    } else {
        dev_info(dev, "%s: irq gpio provided ok.\n", __func__);
    }
    aw8697->count_go = of_property_read_bool(np,
                                              "op,count_go");
    pr_info("aw8697->count_go:%d\n", aw8697->count_go);
    return 0;
}

static int aw8697_hw_reset(struct aw8697 *aw8697)
{
    pr_info("%s enter\n", __func__);

    if (aw8697 && gpio_is_valid(aw8697->reset_gpio)) {
        gpio_set_value_cansleep(aw8697->reset_gpio, 0);
        msleep(1);
        gpio_set_value_cansleep(aw8697->reset_gpio, 1);
        msleep(1);
    } else {
        if (aw8697)
           dev_err(aw8697->dev, "%s:  failed\n", __func__);
    }
    return 0;
}


/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw8697_read_chipid(struct aw8697 *aw8697)
{
    int ret = -1;
    unsigned char cnt = 0;
    unsigned char reg = 0;

    while(cnt < AW_READ_CHIPID_RETRIES) {
        /* hardware reset */
        aw8697_hw_reset(aw8697);

        ret = aw8697_i2c_read(aw8697, AW8697_REG_ID, &reg);
        if (ret < 0) {
            dev_err(aw8697->dev, "%s: failed to read register AW8697_REG_ID: %d\n", __func__, ret);
        }
        switch (reg) {
        case AW8697_CHIPID:
            pr_info("%s aw8697 detected\n", __func__);
            aw8697->chipid = AW8697_CHIPID;
            aw8697_haptic_softreset(aw8697);
            return 0;
        default:
            pr_info("%s unsupported device revision (0x%x)\n", __func__, reg );
            break;
        }
        cnt ++;

        msleep(AW_READ_CHIPID_RETRY_DELAY);
    }

    return -EINVAL;
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw8697_i2c_reg_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct aw8697 *aw8697 = dev_get_drvdata(dev);

    unsigned int databuf[2] = {0, 0};

    if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        aw8697_i2c_write(aw8697, (unsigned char)databuf[0], (unsigned char)databuf[1]);
    }

    return count;
}

static ssize_t aw8697_i2c_reg_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
    struct aw8697 *aw8697 = dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;
    for(i = 0; i < AW8697_REG_MAX; i ++) {
        if(!(aw8697_reg_access[i]&REG_RD_ACCESS))
           continue;
        aw8697_i2c_read(aw8697, i, &reg_val);
        len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x \n", i, reg_val);
    }
    return len;
}
static ssize_t aw8697_i2c_ram_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct aw8697 *aw8697 = dev_get_drvdata(dev);

    unsigned int databuf[1] = {0};

    if(1 == sscanf(buf, "%x", &databuf[0])) {
        if(1 == databuf[0]) {
            aw8697_ram_update(aw8697);
        }
    }

    return count;
}

static ssize_t aw8697_i2c_ram_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
    struct aw8697 *aw8697 = dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned int i = 0;
    unsigned char reg_val = 0;

    mutex_lock(&aw8697->lock);
    aw8697_haptic_stop(aw8697);
    mutex_unlock(&aw8697->lock);
    /* RAMINIT Enable */
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_EN);

    aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRH, (unsigned char)(aw8697->ram.base_addr>>8));
    aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRL, (unsigned char)(aw8697->ram.base_addr&0x00ff));
    len += snprintf(buf+len, PAGE_SIZE-len, "aw8697_haptic_ram:\n");
    for(i=0; i<aw8697->ram.len; i++) {
        aw8697_i2c_read(aw8697, AW8697_REG_RAMDATA, &reg_val);
        len += snprintf(buf+len, PAGE_SIZE-len, "0x%02x,", reg_val);
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
    /* RAMINIT Disable */
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_OFF);

    return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw8697_i2c_reg_show, aw8697_i2c_reg_store);
static DEVICE_ATTR(ram, S_IWUSR | S_IRUGO, aw8697_i2c_ram_show, aw8697_i2c_ram_store);

static struct attribute *aw8697_attributes[] = {
    &dev_attr_reg.attr,
    &dev_attr_ram.attr,
    NULL
};

static struct attribute_group aw8697_attribute_group = {
    .attrs = aw8697_attributes
};


/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw8697_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    struct aw8697 *aw8697;
    struct device_node *np = i2c->dev.of_node;
    int irq_flags = 0;
    int ret = -1;

    pr_info("%s enter\n", __func__);

    if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
        dev_err(&i2c->dev, "check_functionality failed\n");
        return -EIO;
    }

    aw8697 = devm_kzalloc(&i2c->dev, sizeof(struct aw8697), GFP_KERNEL);
    if (aw8697 == NULL)
        return -ENOMEM;

    aw8697->dev = &i2c->dev;
    aw8697->i2c = i2c;

    i2c_set_clientdata(i2c, aw8697);

    /* aw8697 rst & int */
    if (np) {
        ret = aw8697_parse_dt(&i2c->dev, aw8697, np);
        if (ret) {
            dev_err(&i2c->dev, "%s: failed to parse device tree node\n", __func__);
            goto err_parse_dt;
        }
    } else {
        aw8697->reset_gpio = -1;
        aw8697->irq_gpio = -1;
    }

    if (gpio_is_valid(aw8697->reset_gpio)) {
        ret = devm_gpio_request_one(&i2c->dev, aw8697->reset_gpio,
            GPIOF_OUT_INIT_LOW, "aw8697_rst");
        if (ret){
            dev_err(&i2c->dev, "%s: rst request failed\n", __func__);
            goto err_reset_gpio_request;
        }
    }

    if (gpio_is_valid(aw8697->irq_gpio)) {
        ret = devm_gpio_request_one(&i2c->dev, aw8697->irq_gpio,
            GPIOF_DIR_IN, "aw8697_int");
        if (ret){
            dev_err(&i2c->dev, "%s: int request failed\n", __func__);
            goto err_irq_gpio_request;
        }
    }

    /* aw8697 chip id */
    ret = aw8697_read_chipid(aw8697);
    if (ret < 0) {
        dev_err(&i2c->dev, "%s: aw8697_read_chipid failed ret=%d\n", __func__, ret);
        goto err_id;
    }

    /* aw8697 irq */
    if (gpio_is_valid(aw8697->irq_gpio) &&
        !(aw8697->flags & AW8697_FLAG_SKIP_INTERRUPTS)) {
        /* register irq handler */
        aw8697_interrupt_setup(aw8697);
        irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
        ret = devm_request_threaded_irq(&i2c->dev,
                    gpio_to_irq(aw8697->irq_gpio),
                    NULL, aw8697_irq, irq_flags,
                    "aw8697", aw8697);
        if (ret != 0) {
            dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
                    __func__, gpio_to_irq(aw8697->irq_gpio), ret);
            goto err_irq;
        }
    } else {
        dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
        /* disable feature support if gpio was invalid */
        aw8697->flags |= AW8697_FLAG_SKIP_INTERRUPTS;
    }

    dev_set_drvdata(&i2c->dev, aw8697);

    ret = sysfs_create_group(&i2c->dev.kobj, &aw8697_attribute_group);
    if (ret < 0) {
        dev_info(&i2c->dev, "%s error creating sysfs attr files\n", __func__);
        goto err_sysfs;
    }

    g_aw8697 = aw8697;

    aw8697_vibrator_init(aw8697);

    aw8697_haptic_init(aw8697);

    aw8697_ram_init(aw8697);

/* add haptic audio tp mask */
    aw8697->fb_notif.notifier_call = aw8697_fb_notifier_callback_tp;
    ret = msm_drm_register_client(&aw8697->fb_notif);
    if (ret)
       pr_info("Unable to register fb_notifier: %d\n", ret);
/* add haptic audio tp mask end */
    pr_info("%s probe completed successfully!\n", __func__);

    return 0;

err_sysfs:
    devm_free_irq(&i2c->dev, gpio_to_irq(aw8697->irq_gpio), aw8697);
err_irq:
err_id:
    if (gpio_is_valid(aw8697->irq_gpio))
        devm_gpio_free(&i2c->dev, aw8697->irq_gpio);
err_irq_gpio_request:
    if (gpio_is_valid(aw8697->reset_gpio))
        devm_gpio_free(&i2c->dev, aw8697->reset_gpio);
err_reset_gpio_request:
err_parse_dt:
    devm_kfree(&i2c->dev, aw8697);
    aw8697 = NULL;
    return ret;
}

static int aw8697_i2c_remove(struct i2c_client *i2c)
{
    struct aw8697 *aw8697 = i2c_get_clientdata(i2c);

    pr_info("%s enter\n", __func__);

    sysfs_remove_group(&i2c->dev.kobj, &aw8697_attribute_group);

    devm_free_irq(&i2c->dev, gpio_to_irq(aw8697->irq_gpio), aw8697);
    if (gpio_is_valid(aw8697->irq_gpio))
        devm_gpio_free(&i2c->dev, aw8697->irq_gpio);
    if (gpio_is_valid(aw8697->reset_gpio))
        devm_gpio_free(&i2c->dev, aw8697->reset_gpio);
    devm_kfree(&i2c->dev, aw8697);
    aw8697 = NULL;

    return 0;
}

static int __maybe_unused aw8697_suspend(struct device *dev)
{
	int ret = 0;

	 struct aw8697 *aw8697 = dev_get_drvdata(dev);
	 mutex_lock(&aw8697->lock);
	 aw8697_haptic_stop(aw8697);
	 mutex_unlock(&aw8697->lock);

	return ret;
}

static int __maybe_unused aw8697_resume(struct device *dev)
{

	int ret = 0;

	return ret;
}


static SIMPLE_DEV_PM_OPS(aw8697_pm_ops, aw8697_suspend, aw8697_resume);
static const struct i2c_device_id aw8697_i2c_id[] = {
	{ AW8697_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw8697_i2c_id);

static struct of_device_id aw8697_dt_match[] = {
	{ .compatible = "awinic,aw8697_haptic" },
	{ },
};

static struct i2c_driver aw8697_i2c_driver = {
	.driver = {
		.name = AW8697_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw8697_dt_match),
		.pm = &aw8697_pm_ops,
	},
	.probe = aw8697_i2c_probe,
	.remove = aw8697_i2c_remove,
	.id_table = aw8697_i2c_id,
};


static int __init aw8697_i2c_init(void)
{
    int ret = 0;

    pr_info("aw8697 driver version %s\n", AW8697_VERSION);

    ret = i2c_add_driver(&aw8697_i2c_driver);
    if(ret){
        pr_err("fail to add aw8697 device into i2c\n");
        return ret;
    }

    return 0;
}
module_init(aw8697_i2c_init);


static void __exit aw8697_i2c_exit(void)
{
    i2c_del_driver(&aw8697_i2c_driver);
}
module_exit(aw8697_i2c_exit);


MODULE_DESCRIPTION("AW8697 Haptic Driver");
MODULE_LICENSE("GPL v2");
