#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/task_work.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/machine.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

#include "sec_drivers_s6sy761.h"

extern int tp_register_times;
extern struct touchpanel_data *g_tp;
/****************** Start of Log Tag Declear and level define*******************************/
#define TPD_DEVICE "sec-s6sy761"
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)
#define TPD_DEBUG(a, arg...)\
    do{\
        if (LEVEL_DEBUG == tp_debug)\
            pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

#define TPD_DETAIL(a, arg...)\
    do{\
        if (LEVEL_BASIC != tp_debug)\
            pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

#define TPD_DEBUG_NTAG(a, arg...)\
    do{\
        if (tp_debug)\
            printk(a, ##arg);\
    }while(0)
/******************** End of Log Tag Declear and level define*********************************/

/*************************** start of function delcare****************************************/
void sec_mdelay(unsigned int ms);
static int sec_reset(void *chip_data);
static int sec_power_control(void *chip_data, bool enable);
static int sec_get_verify_result(struct chip_data_s6sy761 *chip_info);
static void sec_ts_read_info_work(struct work_struct *work);


/**************************** end of function delcare*****************************************/


/****** Start of other functions that work for touchpanel_operations callbacks***********/
static int sec_enable_black_gesture(struct chip_data_s6sy761 *chip_info, bool enable)
{
    int ret = -1;
    int i = 0;

    TPD_INFO("%s, enable = %d\n", __func__, enable);

    if (enable) {
        for (i = 0; i < 20; i++)
        {
            touch_i2c_write_word(chip_info->client, SEC_CMD_WAKEUP_GESTURE_MODE, 0xFFFF);
            touch_i2c_write_byte(chip_info->client, SEC_CMD_SET_POWER_MODE, 0x01);
            sec_mdelay(10);
            ret = touch_i2c_read_byte(chip_info->client, SEC_CMD_SET_POWER_MODE);
            if (0x01 == ret)
                break;
        }
	} else {
		for (i = 0; i < 20; i++)
		{
			touch_i2c_write_word(chip_info->client, SEC_CMD_WAKEUP_GESTURE_MODE, 0x0000);
			touch_i2c_write_byte(chip_info->client, SEC_CMD_SET_POWER_MODE, 0x00);
			sec_mdelay(10);
			ret = touch_i2c_read_byte(chip_info->client, SEC_CMD_SET_POWER_MODE);
			if (0x00 == ret)
			break;
		}
        return 0;
    }

    if (i >= 5) {
        ret = -1;
        TPD_INFO("%s: enter black gesture failed\n", __func__);
    } else {
        TPD_INFO("%s: %d times enter black gesture success\n", __func__, i);
    }
    return ret;
}

static int sec_enable_edge_limit(struct chip_data_s6sy761 *chip_info, bool enable)
{
    int ret = -1;

    if (enable) {
        ret = touch_i2c_write_word(chip_info->client, SEC_CMD_GRIP_SWITCH, 0x2000);
    } else {
        ret = touch_i2c_write_word(chip_info->client, SEC_CMD_GRIP_SWITCH, 0x0000);
    }

    TPD_INFO("%s: state: %d %s!\n", __func__, enable, ret < 0 ? "failed" : "success");
    return ret;
}

static int sec_enable_charge_mode(struct chip_data_s6sy761 *chip_info, bool enable)
{
    int ret = -1;

    if (enable) {
        ret = touch_i2c_write_byte(chip_info->client, SET_CMD_SET_CHARGER_MODE, 0x02);
    } else {
        ret = touch_i2c_write_byte(chip_info->client, SET_CMD_SET_CHARGER_MODE, 0x01);
    }

    TPD_INFO("%s: state: %d %s!\n", __func__, enable, ret < 0 ? "failed" : "success");
    return ret;
}

static int sec_enable_earsense_mode(struct chip_data_s6sy761 *chip_info, bool enable)
{
    int ret = -1;

    if (enable) {
        ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_HOVER_DETECT, 1);
        ret |= touch_i2c_write_byte(chip_info->client, SEC_CMD_MUTU_RAW_TYPE, TYPE_DATA_DELTA);
    } else {
        ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_HOVER_DETECT, 0);
        ret |= touch_i2c_write_byte(chip_info->client, SEC_CMD_MUTU_RAW_TYPE, TYPE_SIGNAL_DATA);
    }

    TPD_INFO("%s: state: %d %s!\n", __func__, enable, ret < 0 ? "failed" : "success");
    return ret;
}

static int sec_enable_face_mode(struct chip_data_s6sy761 *chip_info, bool enable)
{
    int ret = -1;

    if (enable) {
        ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_HOVER_DETECT, 1);
    } else {
        ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_HOVER_DETECT, 0);
    }

    TPD_INFO("%s: state: %d %s!\n", __func__, enable, ret < 0 ? "failed" : "success");
    return ret;
}

static int sec_face_reduce_mode(struct chip_data_s6sy761 *chip_info, bool enable)
{
	int ret = -1;

	if (enable) {
		ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_TOUCHHOLD_CALIBRATE, 1);
	}

	TPD_INFO("%s: state: %d %s!\n", __func__, enable, ret < 0 ? "failed" : "success");
	return ret;
}

static int sec_enable_palm_reject(struct chip_data_s6sy761 *chip_info, bool enable)
{
    int ret = -1;

    if (enable) {
        ret = touch_i2c_write_word(chip_info->client, SEC_CMD_PALM_SWITCH, 0x0061);
    } else {
        ret = touch_i2c_write_word(chip_info->client, SEC_CMD_PALM_SWITCH, 0x0041);
    }

    TPD_INFO("%s: state: %d %s!\n", __func__, enable, ret < 0 ? "failed" : "success");
    return ret;
}

static int sec_enable_game_mode(struct chip_data_s6sy761 *chip_info, bool enable)
{
	int ret = -1;

	if (enable)
		ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_GAME_FAST_SLIDE, 1);
	else
		ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_GAME_FAST_SLIDE, 0);

	TPD_INFO("%s: state: %d %s!\n", __func__, enable, ret < 0 ? "failed" : "success");

	return ret;
}

static int sec_refresh_switch_mode(struct chip_data_s6sy761 *chip_info, bool enable)
{
	int ret = -1;

	if (enable) {
		ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_REFRESH_RATE_SWITCH, 0x5A);
	} else {
		ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_REFRESH_RATE_SWITCH, 0x3C);
	}
    TPD_INFO("%s: refresh_switch: %s %s!\n", __func__, enable == 0 ? "60HZ":"90HZ", ret < 0 ? "failed" : "success");
    return ret;
}

static int sec_touchhold_switch_mode(struct chip_data_s6sy761 *chip_info, bool enable)
{
	int ret = -1;
	int i = 0;

	if (enable == 1) {
		for(i = 0; i < 10; i++) {
			ret = touch_i2c_read_byte(chip_info->client, SEC_CMD_TOUCHHOLD_SWITCH);
			ret |= 0x01;
			ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_TOUCHHOLD_SWITCH, ret);
			sec_mdelay(10);
			ret = touch_i2c_read_byte(chip_info->client, SEC_CMD_TOUCHHOLD_SWITCH);
			if (ret == 1)
				break;
		}
	} else if (enable == 0){
		ret = touch_i2c_read_byte(chip_info->client, SEC_CMD_TOUCHHOLD_SWITCH);
		ret &= 0xFE;
		ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_TOUCHHOLD_SWITCH, ret);
	}
    TPD_INFO("%s: touchhold_enable: %d %s!\n", __func__, enable, ret < 0 ? "failed" : "success");
    return ret;
}

static int sec_toucharea_switch_mode(struct chip_data_s6sy761 *chip_info, bool enable)
{
	int ret = -1;

	if (enable) {
		ret = touch_i2c_read_byte(chip_info->client, SEC_CMD_TOUCHHOLD_SWITCH);
		ret |= 0x02;
		ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_TOUCHHOLD_SWITCH, ret);
		TPD_INFO("%s:cmd = 0x%x, touch area switch qualcom %s\n", __func__, ret, ret < 0 ? "failed" : "success");
	} else {
		ret = touch_i2c_read_byte(chip_info->client, SEC_CMD_TOUCHHOLD_SWITCH);
		ret &= 0xFD;
		ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_TOUCHHOLD_SWITCH, ret);
		TPD_INFO("%s:cmd = 0x%x, touch area switch goodix %s\n", __func__, ret, ret < 0 ? "failed" : "success");
	}

    return ret;
}
static int sec_limit_switch_mode(struct chip_data_s6sy761 *chip_info, bool enable)
{
	int ret = -1;
	unsigned char buf[4] = {0};
	unsigned char cmd[3] = {0};

		if (g_tp->limit_switch == 1) {		//LANDSPACE
			ret = touch_i2c_write_byte(chip_info->client, 0x4B, 0);	//close wet mode
			cmd[0] = 0x01;
			ret = touch_i2c_write_block(chip_info->client, SEC_CMD_GRIPMODE_SWITCH, 3, cmd);	//change mode
			buf[1] = 0x64;
			buf[3] = 0x64;
			ret = touch_i2c_write_block(chip_info->client, SEC_CMD_LANDSPACE_CORNER, 4, buf);
		}  else if (g_tp->limit_switch == 3) {
			ret = touch_i2c_write_byte(chip_info->client, 0x4B, 0);
			cmd[0] = 0x02;
			ret = touch_i2c_write_block(chip_info->client, SEC_CMD_GRIPMODE_SWITCH, 3, cmd);
			buf[1] = 0x64;
			buf[3] = 0x64;
			ret = touch_i2c_write_block(chip_info->client, SEC_CMD_LANDSPACE_CORNER, 4, buf);
		} else {	//portrait
			ret = touch_i2c_write_byte(chip_info->client, 0x4B, 1);	//open wet mode
			cmd[0] = 0;
			ret = touch_i2c_write_block(chip_info->client, SEC_CMD_GRIPMODE_SWITCH, 3, cmd);
			buf[1] = 0x64;
			buf[3] = 0x64;
			ret = touch_i2c_write_block(chip_info->client, SEC_CMD_PORTRAIT_CORNER, 4, buf);
	}
    return ret;

}

static int sec_gesture_switch_mode(struct chip_data_s6sy761 *chip_info, bool enable)
{
	int ret = -1;

    if (enable) {
        ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_DISABLE_GESTURE_MODE, 1); //disable gesture
    } else {
        ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_DISABLE_GESTURE_MODE, 0); //enable gesture
    }

    TPD_INFO("%s: gesture_switch: %s %s!\n", __func__, enable == 0 ? "enable":"disable", ret < 0 ? "failed" : "success");
    return ret;
}

void sec_mdelay(unsigned int ms)
{
    if (ms < 20)
        usleep_range(ms * 1000, ms * 1000);
    else
        msleep(ms);
}

int sec_wait_for_ready(struct chip_data_s6sy761 *chip_info, unsigned int ack)
{
    int rc = -1;
    int retry = 0, retry_cnt = 100;
    int8_t status = -1;
    u8 tBuff[SEC_EVENT_BUFF_SIZE] = {0,};

    while (touch_i2c_read_block(chip_info->client, SEC_READ_ONE_EVENT, SEC_EVENT_BUFF_SIZE, tBuff) > 0) {
        status = (tBuff[0] >> 2) & 0xF;
        if ((status == TYPE_STATUS_EVENT_INFO) || (status == TYPE_STATUS_EVENT_VENDOR_INFO)) {
            if (tBuff[1] == ack) {
                rc = 0;
                break;
            }
        }

        if (retry++ > retry_cnt) {
            TPD_INFO("%s: Time Over, event_buf: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X \n", \
                        __func__, tBuff[0], tBuff[1], tBuff[2], tBuff[3], tBuff[4], tBuff[5], tBuff[6], tBuff[7]);
			status = touch_i2c_read_byte(chip_info->client, SEC_READ_BOOT_STATUS);
			if (status == SEC_STATUS_BOOT_MODE) {
				TPD_INFO("%s: firmware in bootloader mode,boot failed\n", __func__);
			}
            break;
        }
        sec_mdelay(20);
    }

    return rc;
}

static int sec_enter_fw_mode(struct chip_data_s6sy761 *chip_info)
{
    int ret = -1;
    u8 device_id[3] = {0};
    u8 fw_update_mode_passwd[] = {0x55, 0xAC};

    ret = touch_i2c_write_block(chip_info->client, SEC_CMD_ENTER_FW_MODE, sizeof(fw_update_mode_passwd), fw_update_mode_passwd);
    sec_mdelay(20);
    if (ret < 0) {
        TPD_INFO("%s: write cmd to enter fw mode failed\n", __func__);
        return -1;
    }

    //need soft reset or hard reset
    if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
        gpio_direction_output(chip_info->hw_res->reset_gpio, false);
        sec_mdelay(10);
        gpio_direction_output(chip_info->hw_res->reset_gpio, true);
    } else {
        ret = touch_i2c_write_block(chip_info->client, SEC_CMD_SOFT_RESET, 0, NULL);
        if (ret < 0) {
            TPD_INFO("%s: write soft reset failed\n", __func__);
            return -1;
        }
    }
    sec_mdelay(100);

    ret = touch_i2c_read_byte(chip_info->client, SEC_READ_BOOT_STATUS);     //after reset, check bootloader again
    if (ret < 0) {
        TPD_INFO("%s: read boot status failed\n", __func__);
        return -1;
    }
    if (ret != SEC_STATUS_BOOT_MODE) {
        TPD_INFO("%s: read boot status, but no in boot mode(%d)\n", __func__, ret);
        return -1;
    }

    sec_mdelay(10);

    ret = touch_i2c_read_block(chip_info->client, SEC_READ_ID, 3, device_id);
    if (ret < 0) {
        TPD_INFO("%s: read 3 byte device id failed\n", __func__);
        return -1;
    }

    chip_info->boot_ver[0] = device_id[0];
    chip_info->boot_ver[1] = device_id[1];
    chip_info->boot_ver[2] = device_id[2];
    chip_info->flash_page_size = SEC_FW_BLK_DEFAULT_SIZE;
    if ((device_id[1] == 0x37) && (device_id[2] == 0x61))
        chip_info->flash_page_size = 512;

    return 0;
}

static u8 sec_checksum(u8 *data, int offset, int size)
{
    int i;
    u8 checksum = 0;

    for (i = 0; i < size; i++)
        checksum += data[i + offset];

    return checksum;
}

static int sec_flash_page_erase(struct chip_data_s6sy761 *chip_info, u32 page_idx, u32 page_num)
{
    int ret = -1;
    u8 tCmd[6] = {0};

    tCmd[0] = SEC_CMD_FLASH_ERASE;
    tCmd[1] = (u8)((page_idx >> 8) & 0xFF);
    tCmd[2] = (u8)((page_idx >> 0) & 0xFF);
    tCmd[3] = (u8)((page_num >> 8) & 0xFF);
    tCmd[4] = (u8)((page_num >> 0) & 0xFF);
    tCmd[5] = sec_checksum(tCmd, 1, 4);

    ret = touch_i2c_write(chip_info->client, tCmd, 6);

    return ret;
}

static int sec_flash_page_write(struct chip_data_s6sy761 *chip_info, u32 page_idx, u8 *page_data)
{
    int ret;
    u8 tCmd[1 + 2 + SEC_FW_BLK_SIZE_MAX + 1];
    int flash_page_size = (int)chip_info->flash_page_size;

    tCmd[0] = SEC_CMD_FLASH_WRITE;
    tCmd[1] = (u8)((page_idx >> 8) & 0xFF);
    tCmd[2] = (u8)((page_idx >> 0) & 0xFF);

    memcpy(&tCmd[3], page_data, flash_page_size);
    tCmd[1 + 2 + flash_page_size] = sec_checksum(tCmd, 1, 2 + flash_page_size);

    ret = touch_i2c_write(chip_info->client, tCmd, 1 + 2 + flash_page_size + 1);
    return ret;
}

static bool sec_limited_flash_page_write(struct chip_data_s6sy761 *chip_info, u32 page_idx, u8 *page_data)
{
    int ret = -1;
    u8 *tCmd = NULL;
    u8 copy_data[3 + SEC_FW_BLK_SIZE_MAX];
    int copy_left = (int)chip_info->flash_page_size + 3;
    int copy_size = 0;
    int copy_max = I2C_BURSTMAX - 1;
    int flash_page_size = (int)chip_info->flash_page_size;

    copy_data[0] = (u8)((page_idx >> 8) & 0xFF);    /* addH */
    copy_data[1] = (u8)((page_idx >> 0) & 0xFF);    /* addL */

    memcpy(&copy_data[2], page_data, flash_page_size);    /* DATA */
    copy_data[2 + flash_page_size] = sec_checksum(copy_data, 0, 2 + flash_page_size);    /* CS */

    while (copy_left > 0) {
        int copy_cur = (copy_left > copy_max) ? copy_max : copy_left;

        tCmd = kzalloc(copy_cur + 1, GFP_KERNEL);
        if (!tCmd)
            goto err_write;

        if (copy_size == 0)
            tCmd[0] = SEC_CMD_FLASH_WRITE;
        else
            tCmd[0] = SEC_CMD_FLASH_PADDING;

        memcpy(&tCmd[1], &copy_data[copy_size], copy_cur);

        ret = touch_i2c_write(chip_info->client, tCmd, 1 + copy_cur);
        if (ret < 0) {
            ret = touch_i2c_write(chip_info->client, tCmd, 1 + copy_cur);
            if (ret < 0) {
                TPD_INFO("%s: failed, ret:%d\n", __func__, ret);
            }
        }

        copy_size += copy_cur;
        copy_left -= copy_cur;
        kfree(tCmd);
    }
    return ret;

err_write:
    TPD_INFO("%s: failed to alloc.\n", __func__);
    return -ENOMEM;

}

static int sec_flash_write(struct chip_data_s6sy761 *chip_info, u32 mem_addr, u8 *mem_data, u32 mem_size)
{
    int ret = -1;
    u32 page_idx = 0, size_copy = 0, flash_page_size = 0;
    u32 page_idx_start = 0, page_idx_end = 0, page_num = 0;
    u8 page_buf[SEC_FW_BLK_SIZE_MAX] = {0};

    if (mem_size == 0)
        return 0;

    flash_page_size = chip_info->flash_page_size;
    page_idx_start = mem_addr / flash_page_size;
    page_idx_end = (mem_addr + mem_size - 1) / flash_page_size;
    page_num = page_idx_end - page_idx_start + 1;

    ret = sec_flash_page_erase(chip_info, page_idx_start, page_num);
    if (ret < 0) {
        TPD_INFO("%s: fw erase failed, mem_addr= %08X, pagenum = %d\n", __func__, mem_addr, page_num);
        return -EIO;
    }

    sec_mdelay(page_num + 10);

    size_copy = mem_size % flash_page_size;
    if (size_copy == 0)
        size_copy = flash_page_size;

    memset(page_buf, 0, flash_page_size);

    for (page_idx = page_num - 1;; page_idx--) {
        memcpy(page_buf, mem_data + (page_idx * flash_page_size), size_copy);
        if (chip_info->boot_ver[0] == 0xB2) {
            ret = sec_flash_page_write(chip_info, (page_idx + page_idx_start), page_buf);
            if (ret < 0) {
                sec_mdelay(50);
                ret = sec_flash_page_write(chip_info, (page_idx + page_idx_start), page_buf);                
                if (ret < 0) {
                    TPD_INFO("%s: fw write failed, page_idx = %u\n", __func__, page_idx);
                    goto err;
                }
            }
        } else {
            ret = sec_limited_flash_page_write(chip_info, (page_idx + page_idx_start), page_buf);
            if (ret < 0) {
                sec_mdelay(50);
                ret = sec_limited_flash_page_write(chip_info, (page_idx + page_idx_start), page_buf);
                if (ret < 0) {
                    TPD_INFO("%s: fw write failed, page_idx = %u\n", __func__, page_idx);
                    goto err;
                }
            }

        }

        size_copy = flash_page_size;
        sec_mdelay(5);

        if (page_idx == 0) /* end condition (page_idx >= 0)   page_idx type unsinged int */
            break;
    }

    return mem_size;
err:
    return -EIO;
}

static int sec_block_read(struct chip_data_s6sy761 *chip_info, u32 mem_addr, int mem_size, u8 *buf)
{
    int ret;
    u8 cmd[5];
    u8 *data;

    if (mem_size >= 64 * 1024) {
        TPD_INFO("%s: mem size over 64K\n", __func__);
        return -EIO;
    }

    cmd[0] = (u8)SEC_CMD_FLASH_READ_ADDR;
    cmd[1] = (u8)((mem_addr >> 24) & 0xff);
    cmd[2] = (u8)((mem_addr >> 16) & 0xff);
    cmd[3] = (u8)((mem_addr >> 8) & 0xff);
    cmd[4] = (u8)((mem_addr >> 0) & 0xff);

    ret = touch_i2c_write(chip_info->client, cmd, 5);
    if (ret < 0) {
        TPD_INFO("%s: send command failed, %02X\n", __func__, cmd[0]);
        return -EIO;
    }

    udelay(10);
    cmd[0] = (u8)SEC_CMD_FLASH_READ_SIZE;
    cmd[1] = (u8)((mem_size >> 8) & 0xff);
    cmd[2] = (u8)((mem_size >> 0) & 0xff);

    ret = touch_i2c_write(chip_info->client, cmd, 3);
    if (ret < 0) {
        TPD_INFO("%s: send command failed, %02X\n", __func__, cmd[0]);
        return -EIO;
    }

    udelay(10);
    cmd[0] = (u8)SEC_CMD_FLASH_READ_DATA;
    data = buf;

    ret = touch_i2c_read(chip_info->client, cmd, 1, data, mem_size);
    if (ret < 0) {
        TPD_INFO("%s: memory read failed\n", __func__);
        return -EIO;
    }

    return 0;
}

static int sec_memory_read(struct chip_data_s6sy761 *chip_info, u32 mem_addr, u8 *mem_data, u32 mem_size)
{
    int ret;
    int retry = 3;
    int read_size = 0;
    int unit_size;
    int max_size = I2C_BURSTMAX;
    int read_left = (int)mem_size;
    u8 *tmp_data;

    tmp_data = kmalloc(max_size, GFP_KERNEL);
    if (!tmp_data) {
        TPD_INFO("%s: failed to kmalloc\n", __func__);
        return -ENOMEM;
    }

    while (read_left > 0) {
        unit_size = (read_left > max_size) ? max_size : read_left;
        retry = 3;
        do {
            ret = sec_block_read(chip_info, mem_addr, unit_size, tmp_data);
            if (retry-- == 0) {
                TPD_INFO("%s: fw read fail mem_addr=%08X, unit_size=%d\n", __func__, mem_addr, unit_size);
                kfree(tmp_data);
                return -1;
            }

            memcpy(mem_data + read_size, tmp_data, unit_size);
        } while (ret < 0);

        mem_addr += unit_size;
        read_size += unit_size;
        read_left -= unit_size;
    }

    kfree(tmp_data);

    return read_size;
}

static int sec_chunk_update(struct chip_data_s6sy761 *chip_info, u32 addr, u32 size, u8 *data)
{
    int ii = 0, ret = 0;
    u8 *mem_rb = NULL;
    u32 write_size = 0;
    u32 fw_size = size;

    write_size = sec_flash_write(chip_info, addr, data, fw_size);
    if (write_size != fw_size) {
        TPD_INFO("%s: fw write failed\n", __func__);
        ret = -1;
        goto err_write_fail;
    }

    mem_rb = vzalloc(fw_size);
    if (!mem_rb) {
        TPD_INFO("%s: vzalloc failed\n", __func__);
        ret = -1;
        goto err_write_fail;
    }

    if (sec_memory_read(chip_info, addr, mem_rb, fw_size) >= 0) {
        for (ii = 0; ii < fw_size; ii++) {
            if (data[ii] != mem_rb[ii])
                break;
        }

        if (fw_size != ii) {
            TPD_INFO("%s: fw verify fail at data[%d](%d, %d)\n", __func__, ii, data[ii], mem_rb[ii]);
            ret = -1;
            goto out;
        }
    } else {
        ret = -1;
        goto out;
    }

    TPD_INFO("%s: verify done(%d)\n", __func__, ret);

out:
    vfree(mem_rb);
err_write_fail:
    sec_mdelay(10);

    return ret;
}

int sec_read_calibration_report(struct chip_data_s6sy761 *chip_info)
{
    int ret;
    u8 buf[5] = { 0 };

    buf[0] = SEC_CMD_READ_CALIBRATION_REPORT;

    ret = touch_i2c_read(chip_info->client, &buf[0], 1, &buf[1], 4);
    if (ret < 0) {
        TPD_INFO("%s: failed to read, ret = %d\n", __func__, ret);
        return ret;
    }

    TPD_INFO("%s: count:%d, pass count:%d, fail count:%d, status:0x%X\n",
                __func__, buf[1], buf[2], buf[3], buf[4]);

    return buf[4];
}

int sec_execute_force_calibration(struct chip_data_s6sy761 *chip_info)
{
    int rc = -1;

    if (touch_i2c_write_block(chip_info->client, SEC_CMD_FACTORY_PANELCALIBRATION, 0, NULL) < 0) {
        TPD_INFO("%s: Write Cal commend failed!\n", __func__);
        return rc;
    }

    sec_mdelay(1000);
    rc = sec_wait_for_ready(chip_info, SEC_VENDOR_ACK_OFFSET_CAL_DONE);

    return rc;
}

static void handleFourCornerPoint(struct Coordinate *point, int n)
{
    int i = 0;
    struct Coordinate left_most = point[0], right_most = point[0], top_most = point[0], down_most = point[0];

    if (n < 4)
        return;

    for (i = 0; i < n; i++) {
        if (right_most.x < point[i].x) {   //xmax
            right_most = point[i];
        }
        if (left_most.x > point[i].x) {   //xmin
            left_most = point[i];
        }
        if (down_most.y < point[i].y) {   //ymax
            down_most = point[i];
        }
        if (top_most.y > point[i].y) {   //ymin
            top_most = point[i];
        }
    }
    point[0] = top_most;
    point[1] = left_most;
    point[2] = down_most;
    point[3] = right_most;
}
/****** End of other functions that work for touchpanel_operations callbacks*************/

/********* Start of implementation of touchpanel_operations callbacks********************/
static int sec_reset(void *chip_data)
{
    int ret = -1;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;

    TPD_INFO("%s is called\n", __func__);
    if (chip_info->is_power_down) { //power off state, no need reset
        return 0;
    }

    disable_irq_nosync(chip_info->client->irq);

    if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {    //rsted by rst pin
        TPD_INFO("reset by pull down rst pin");
        gpio_direction_output(chip_info->hw_res->reset_gpio, false);
        sec_mdelay(5);
        gpio_direction_output(chip_info->hw_res->reset_gpio, true);
    } else {    //otherwise by soft reset
        touch_i2c_write_block(chip_info->client, SEC_CMD_SOFT_RESET, 0, NULL);
    }

    sec_mdelay(RESET_TO_NORMAL_TIME);
    sec_wait_for_ready(chip_info, SEC_ACK_BOOT_COMPLETE);
    ret = touch_i2c_write_block(chip_info->client, SEC_CMD_SENSE_ON, 0, NULL);
    TPD_INFO("%s: write sense on %s\n", __func__, (ret < 0) ? "failed" : "success");

    enable_irq(chip_info->client->irq);

    return 0;
}

static int sec_get_vendor(void *chip_data, struct panel_info *panel_data)
{
    int len = 0;
    char manu_temp[MAX_DEVICE_MANU_LENGTH] = "SEC_";
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;

    len = strlen(panel_data->fw_name);
    if ((len > 3) && (panel_data->fw_name[len-3] == 'i') && \
        (panel_data->fw_name[len-2] == 'm') && (panel_data->fw_name[len-1] == 'g')) {
        panel_data->fw_name[len-3] = 'b';
        panel_data->fw_name[len-2] = 'i';
        panel_data->fw_name[len-1] = 'n';
    }
    chip_info->tp_type = panel_data->tp_type;
    strlcat(manu_temp, panel_data->manufacture_info.manufacture, MAX_DEVICE_MANU_LENGTH);
    strncpy(panel_data->manufacture_info.manufacture, manu_temp, MAX_DEVICE_MANU_LENGTH);
    TPD_INFO("chip_info->tp_type = %d, panel_data->fw_name = %s\n", chip_info->tp_type, panel_data->fw_name);

    return 0;
}

static int sec_get_chip_info(void *chip_data)
{
    return 0;
}

static int sec_power_control(void *chip_data, bool enable)
{
    int ret = 0;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;

    TPD_INFO("%s enable :%d\n", __func__, enable);
    if (true == enable) {
		tp_powercontrol_1v8(chip_info->hw_res, true);
		tp_powercontrol_2v8(chip_info->hw_res, true);
		if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
			TPD_INFO("Set the reset_gpio \n");
			gpio_direction_output(chip_info->hw_res->reset_gpio, 1);
        }
        msleep(RESET_TO_NORMAL_TIME);
        sec_wait_for_ready(chip_info, SEC_ACK_BOOT_COMPLETE);
        ret = touch_i2c_write_block(chip_info->client, SEC_CMD_SENSE_ON, 0, NULL);
        TPD_INFO("%s: write sense on %s\n", __func__, (ret < 0) ? "failed" : "success");
        chip_info->is_power_down = false;
    } else {
        tp_powercontrol_2v8(chip_info->hw_res, false);
		msleep(5);
		tp_powercontrol_1v8(chip_info->hw_res, false);
		if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
			TPD_INFO("Set the reset_gpio \n");
			gpio_direction_output(chip_info->hw_res->reset_gpio, 0);
		}

        chip_info->is_power_down = true;
    }

    return ret;
}

static fw_check_state sec_fw_check(void *chip_data, struct resolution_info *resolution_info, struct panel_info *panel_data)
{
    int ret = 0;
    unsigned char data[5] = { 0 };
    bool valid_fw_integrity = false;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;

    ret = touch_i2c_read_byte(chip_info->client, SEC_READ_FIRMWARE_INTEGRITY);  //judge whether fw is right
    if (ret < 0) {
        TPD_INFO("%s: failed to do integrity check (%d)\n", __func__, ret);
    } else {
        if (ret & 0x80) {
            valid_fw_integrity = true;
        } else {
            valid_fw_integrity = false;
            TPD_INFO("invalid firmware integrity (%d)\n", ret);
        }
    }

    ret = touch_i2c_read_byte(chip_info->client, SEC_READ_BOOT_STATUS);
    if (ret < 0) {
        TPD_INFO("%s: failed to read boot status\n", __func__);
    } else {
        ret = touch_i2c_read_block(chip_info->client, SEC_READ_TS_STATUS, 4, &data[1]);
        if (ret < 0) {
            TPD_INFO("%s: failed to read touch status\n", __func__);
        }
    }
    if ((((data[0] == SEC_STATUS_APP_MODE) && (data[2] == TOUCH_SYSTEM_MODE_FLASH)) || (ret < 0)) && (valid_fw_integrity == false)) {
        TPD_INFO("%s: fw id abnormal, need update\n", __func__);
        return FW_ABNORMAL;
    }

    memset(data, 0, 5);
    touch_i2c_read_block(chip_info->client, SEC_READ_IMG_VERSION, 4, data);
    panel_data->TP_FW = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    if (panel_data->manufacture_info.version)
        sprintf(panel_data->manufacture_info.version, "0x%x", panel_data->TP_FW);

    // ret = touch_i2c_write_block(chip_info->client, SEC_CMD_SENSE_ON, 0, NULL);
    // TPD_INFO("%s: write sense on %s\n", (ret < 0) ? "failed" : "success");
    return FW_NORMAL;
}

static fw_update_state sec_fw_update(void *chip_data, const struct firmware *fw, bool force)
{
    int i = 0, ret = 0;
    u8 buf[4] = {0};
    u8 *fd = NULL;
    uint8_t cal_status = 0;
    sec_fw_chunk *fw_ch = NULL;
    sec_fw_header *fw_hd = NULL;
    uint32_t fw_version_in_bin = 0, fw_version_in_ic = 0;
    uint32_t config_version_in_bin = 0, config_version_in_ic = 0;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;

    if (!chip_info) {
        TPD_INFO("Chip info is NULL\n");
        return 0;
    }

    TPD_INFO("%s is called, force update:%d\n", __func__, force);

    fd = (u8 *)(fw->data);
    fw_hd = (sec_fw_header *)(fw->data);
    buf[3] = (fw_hd->img_ver >> 24) & 0xff;
    buf[2] = (fw_hd->img_ver >> 16) & 0xff;
    buf[1] = (fw_hd->img_ver >> 8) & 0xff;
    buf[0] = (fw_hd->img_ver >> 0) & 0xff;
    fw_version_in_bin = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    memset(buf, 0, 4);
    touch_i2c_read_block(chip_info->client, SEC_READ_IMG_VERSION, 4, buf);
    fw_version_in_ic = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    TPD_INFO("img version in bin is 0x%04x, img version in ic is 0x%04x\n", fw_version_in_bin, fw_version_in_ic);

    buf[3] = (fw_hd->para_ver >> 24) & 0xff;
    buf[2] = (fw_hd->para_ver >> 16) & 0xff;
    buf[1] = (fw_hd->para_ver >> 8) & 0xff;
    buf[0] = (fw_hd->para_ver >> 0) & 0xff;
    config_version_in_bin = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    memset(buf, 0, 4);
    touch_i2c_read_block(chip_info->client, SEC_READ_CONFIG_VERSION, 4, buf);
    config_version_in_ic = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    TPD_INFO("config version in bin is 0x%04x, config version in ic is 0x%04x\n", config_version_in_bin, config_version_in_ic);

    ret = touch_i2c_read_byte(chip_info->client, SEC_READ_BOOT_STATUS);
    if (ret == SEC_STATUS_BOOT_MODE) {
        force = 1;
        TPD_INFO("%s: still in bootloader mode, will do force update\n", __func__);
    }

    if (!force) {
        if (fw_version_in_bin == fw_version_in_ic) {
            return FW_NO_NEED_UPDATE;
        }
    }

    if (sec_enter_fw_mode(chip_info)) {
        TPD_INFO("%s: enter fw mode failed\n", __func__);
        return FW_UPDATE_ERROR;
    }

    if (fw_hd->signature != SEC_FW_HEADER_SIGN) {
        TPD_INFO("%s: firmware header error(0x%08x)\n", __func__, fw_hd->signature);
        return FW_UPDATE_ERROR;
    }

    fd += sizeof(sec_fw_header);
    for (i = 0; i < fw_hd->num_chunk; i++) {
        fw_ch = (sec_fw_chunk *)fd;
        TPD_INFO("update %d chunk(addr: 0x%08x, size: 0x%08x)\n", i, fw_ch->addr, fw_ch->size);
        if (fw_ch->signature != SEC_FW_CHUNK_SIGN) {
            TPD_INFO("%s: firmware chunk error(0x%08x)\n", __func__, fw_ch->signature);
            return FW_UPDATE_ERROR;
        }
        fd += sizeof(sec_fw_chunk);
        ret = sec_chunk_update(chip_info, fw_ch->addr, fw_ch->size, fd);
        if (ret < 0) {
            TPD_INFO("update chunk failed\n");
            return FW_UPDATE_ERROR;
        }
        fd += fw_ch->size;
    }

    sec_reset(chip_info);
    cal_status = sec_read_calibration_report(chip_info);    //read out calibration result
    if ((cal_status == 0) || (cal_status == 0xFF) || ((config_version_in_ic != config_version_in_bin) && (config_version_in_ic != 0xFFFFFFFF))) {
        TPD_INFO("start calibration.\n");
        ret = sec_execute_force_calibration(chip_info);
        if (ret < 0) {
            TPD_INFO("calibration failed once, try again.\n");
            ret = sec_execute_force_calibration(chip_info);
        }
        TPD_INFO("calibration %s\n", (ret < 0) ? "failed" : "success");
    }
    TPD_INFO("%s: update success\n", __func__);
    return FW_UPDATE_SUCCESS;
}

static u8 sec_trigger_reason(void *chip_data, int gesture_enable, int is_suspended)
{
    int ret = 0;
    int event_id = 0;
    u8 left_event_cnt = 0;
    struct sec_event_status *p_event_status = NULL;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;
	struct fp_underscreen_info tp_info;

    memset(chip_info->first_event, 0, SEC_EVENT_BUFF_SIZE);
    ret = touch_i2c_read_block(chip_info->client, SEC_READ_ONE_EVENT, SEC_EVENT_BUFF_SIZE, chip_info->first_event);
    if (ret < 0) {
        TPD_DETAIL("%s: read one event failed\n", __func__);
        return IRQ_IGNORE;
    }

    TPD_DEBUG("first event: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", \
        chip_info->first_event[0], chip_info->first_event[1], chip_info->first_event[2], chip_info->first_event[3],\
        chip_info->first_event[4], chip_info->first_event[5], chip_info->first_event[6], chip_info->first_event[7]);

    if (chip_info->first_event[0] == 0) {
        TPD_DETAIL("%s: event buffer is empty\n", __func__);
        return IRQ_IGNORE;
    }

    left_event_cnt = chip_info->first_event[7] & 0x3F;
    if ((left_event_cnt > MAX_EVENT_COUNT - 1) || (left_event_cnt == 0xFF)) {
        TPD_INFO("%s: event buffer overflow, do clear the buffer\n", __func__);
        ret = touch_i2c_write_block(chip_info->client, SEC_CMD_CLEAR_EVENT_STACK, 0, NULL);
        if (ret < 0) {
            TPD_INFO("%s: clear event buffer failed\n", __func__);
        }
        return IRQ_IGNORE;
    }

    event_id = chip_info->first_event[0] & 0x3;
    if (event_id == SEC_STATUS_EVENT) {
        /* watchdog reset -> send SENSEON command */
        p_event_status = (struct sec_event_status *)chip_info->first_event;
        if ((p_event_status->stype == TYPE_STATUS_EVENT_INFO) &&
            (p_event_status->status_id == SEC_ACK_BOOT_COMPLETE) && (p_event_status->status_data_1 == 0x20)) {

            ret = touch_i2c_write_block(chip_info->client, SEC_CMD_SENSE_ON, 0, NULL);
            if (ret < 0) {
                TPD_INFO("%s: write sense on failed\n", __func__);
            }
            return IRQ_FW_AUTO_RESET;
        }

        /* event queue full-> all finger release */
        if ((p_event_status->stype == TYPE_STATUS_EVENT_ERR) && (p_event_status->status_id == SEC_ERR_EVENT_QUEUE_FULL)) {
            TPD_INFO("%s: IC Event Queue is full\n", __func__);
            tp_touch_btnkey_release();
        }

        if ((p_event_status->stype == TYPE_STATUS_EVENT_ERR) && (p_event_status->status_id == SEC_ERR_EVENT_ESD)) {
            TPD_INFO("%s: ESD detected. run reset\n", __func__);
            return IRQ_EXCEPTION;
        }

        if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) && (p_event_status->status_id == SEC_STATUS_EARDETECTED)) {
			chip_info->proximity_status = p_event_status->status_data_1;
			TPD_INFO("%s: face detect status %d\n",__func__, chip_info->proximity_status);
			return IRQ_FACE_STATE;
        }

		if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) && (p_event_status->status_id == SEC_STATUS_TOUCHHOLD)) {
			if (p_event_status->status_data_1 == 1) {
				tp_info.touch_state = 1;
				tp_info.x = 720;
				tp_info.y = 2728;
				tp_info.area_rate = 90;
				g_tp->touchold_event = 1;
 				gf_opticalfp_irq_handler(1);
			} else if (p_event_status->status_data_1 == 0) {
				tp_info.touch_state = 0;
				tp_info.x = 0;
				tp_info.y = 0;
				tp_info.area_rate = 0;
				g_tp->touchold_event = 0;
				gf_opticalfp_irq_handler(0);
			}
			TPD_INFO("%s: touch_hold status %d\n",__func__, p_event_status->status_data_1);
			return IRQ_IGNORE;
		}

        if ((p_event_status->stype == TYPE_STATUS_EVENT_INFO) && (p_event_status->status_id == SEC_TS_ACK_WET_MODE)) {
			chip_info->wet_mode = p_event_status->status_data_1;
            TPD_INFO("%s: water wet mode %d\n",__func__, chip_info->wet_mode);
            return IRQ_IGNORE;
        }
		if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) && (p_event_status->status_id == SEC_TS_VENDOR_ACK_NOISE_STATUS_NOTI)) {
			chip_info->touch_noise_status = !!p_event_status->status_data_1;
            TPD_INFO("%s: TSP NOISE MODE %s[%d]\n",
						__func__,chip_info->touch_noise_status == 0 ? "OFF" : "ON",
						p_event_status->status_data_1);
            return IRQ_IGNORE;
        }
    } else if (event_id == SEC_COORDINATE_EVENT) {
        return IRQ_TOUCH;
    } else if (event_id == SEC_GESTURE_EVENT) {
        return IRQ_GESTURE;
    }

    return IRQ_IGNORE;
}

static int sec_get_touch_points(void *chip_data, struct point_info *points, int max_num)
{
    int i = 0;
    int t_id = 0;
    int ret = -1;
    int left_event = 0;
    struct sec_event_coordinate *p_event_coord = NULL;
    uint32_t obj_attention = 0;
    u8 *event_buff;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;

	event_buff = kzalloc(MAX_EVENT_COUNT*SEC_EVENT_BUFF_SIZE * (sizeof(uint8_t)), GFP_KERNEL);
	if (!event_buff) {
		TPD_INFO("event_buff kzalloc failed\n");
		return -ENOMEM;
	}
    p_event_coord = (struct sec_event_coordinate *)chip_info->first_event;
    t_id = (p_event_coord->tid - 1);
    if ((t_id < max_num) && ((p_event_coord->tchsta == SEC_COORDINATE_ACTION_PRESS) || (p_event_coord->tchsta == SEC_COORDINATE_ACTION_MOVE))) {
        points[t_id].x = (p_event_coord->x_11_4 << 4) | (p_event_coord->x_3_0);
        points[t_id].y = (p_event_coord->y_11_4 << 4) | (p_event_coord->y_3_0);
        points[t_id].z = p_event_coord->z & 0x3F;
        points[t_id].width_major = p_event_coord->major;
        points[t_id].touch_major = p_event_coord->major;
        points[t_id].status = 1;

        if (points[t_id].z <= 0) {
            points[t_id].z = 1;
        }
        obj_attention = obj_attention | (1 << t_id);    //set touch bit
    }

    left_event = chip_info->first_event[7] & 0x3F;
    if (left_event == 0) {
	kfree(event_buff);
        return obj_attention;
    } else if (left_event > max_num - 1) {
        TPD_INFO("%s: read left event beyond max touch points\n", __func__);
        left_event = max_num - 1;
    }
    ret = touch_i2c_read_block(chip_info->client, SEC_READ_ALL_EVENT, sizeof(u8) * (SEC_EVENT_BUFF_SIZE) * (left_event), &event_buff[0]);
    if (ret < 0) {
        TPD_INFO("%s: i2c read all event failed\n", __func__);
	kfree(event_buff);
        return obj_attention;
    }

    for (i = 0; i < left_event; i++) {
        p_event_coord = (struct sec_event_coordinate *)&event_buff[i*SEC_EVENT_BUFF_SIZE];
        t_id = (p_event_coord->tid - 1);
        if ((t_id < max_num) && ((p_event_coord->tchsta == SEC_COORDINATE_ACTION_PRESS) || (p_event_coord->tchsta == SEC_COORDINATE_ACTION_MOVE))) {
            points[t_id].x = (p_event_coord->x_11_4 << 4) | (p_event_coord->x_3_0);
            points[t_id].y = (p_event_coord->y_11_4 << 4) | (p_event_coord->y_3_0);
            points[t_id].z = p_event_coord->z & 0x3F;
            points[t_id].width_major = p_event_coord->major;
            points[t_id].touch_major = p_event_coord->major;
            points[t_id].status = 1;

            if (points[t_id].z <= 0) {
                points[t_id].z = 1;
            }
            obj_attention = obj_attention | (1 << t_id);    //set touch bit
        }
    }
	kfree(event_buff);

    return obj_attention;
}

static int sec_get_gesture_info(void *chip_data, struct gesture_info * gesture)
{
    int i = 0, ret = -1;
    uint8_t coord[18] = {0};
    struct Coordinate limitPoint[4];
    struct sec_gesture_status *p_event_gesture = NULL;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;
    
    p_event_gesture = (struct sec_gesture_status *)chip_info->first_event;
    if (p_event_gesture->coordLen > 18) {
        p_event_gesture->coordLen = 18;
    }

    ret = touch_i2c_read_block(chip_info->client, SEC_READ_GESTURE_EVENT, p_event_gesture->coordLen, coord);
    if (ret < 0) {
        TPD_INFO("%s: read gesture data failed\n", __func__);
    }

    if (LEVEL_BASIC != tp_debug) {
        TPD_INFO("gesture points:");
        for (i = 0; i < p_event_gesture->coordLen/3; i++) {
            printk("(%d, %d) ",(coord[3*i] << 4) | ((coord[3*i+2] >> 0) & 0x0F), (coord[3*i+1] << 4) | ((coord[3*i+2] >> 4) & 0x0F));
        }
    }

    switch (p_event_gesture->gestureId)     //judge gesture type
    {
        case GESTURE_RIGHT:
            gesture->gesture_type  = Left2RightSwip;
            gesture->Point_start.x = (coord[0] << 4) | ((coord[2] >> 4) & 0x0F);
            gesture->Point_start.y = (coord[1] << 4) | ((coord[2] >> 0) & 0x0F);
            gesture->Point_end.x   = (coord[3] << 4) | ((coord[5] >> 4) & 0x0F);
            gesture->Point_end.y   = (coord[4] << 4) | ((coord[5] >> 0) & 0x0F);
            break;

        case GESTURE_LEFT:
            gesture->gesture_type  = Right2LeftSwip;
            gesture->Point_start.x = (coord[0] << 4) | ((coord[2] >> 4) & 0x0F);
            gesture->Point_start.y = (coord[1] << 4) | ((coord[2] >> 0) & 0x0F);
            gesture->Point_end.x   = (coord[3] << 4) | ((coord[5] >> 4) & 0x0F);
            gesture->Point_end.y   = (coord[4] << 4) | ((coord[5] >> 0) & 0x0F);
            break;

        case GESTURE_DOWN:
            gesture->gesture_type  = Up2DownSwip;
            gesture->Point_start.x = (coord[0] << 4) | ((coord[2] >> 4) & 0x0F);
            gesture->Point_start.y = (coord[1] << 4) | ((coord[2] >> 0) & 0x0F);
            gesture->Point_end.x   = (coord[3] << 4) | ((coord[5] >> 4) & 0x0F);
            gesture->Point_end.y   = (coord[4] << 4) | ((coord[5] >> 0) & 0x0F);
            break;

        case GESTURE_UP:
            gesture->gesture_type  = Down2UpSwip;
            gesture->Point_start.x = (coord[0] << 4) | ((coord[2] >> 4) & 0x0F);
            gesture->Point_start.y = (coord[1] << 4) | ((coord[2] >> 0) & 0x0F);
            gesture->Point_end.x   = (coord[3] << 4) | ((coord[5] >> 4) & 0x0F);
            gesture->Point_end.y   = (coord[4] << 4) | ((coord[5] >> 0) & 0x0F);
            break;

        case GESTURE_DOUBLECLICK:
            gesture->gesture_type  = DouTap;
            gesture->Point_start.x = (coord[0] << 4) | ((coord[2] >> 4) & 0x0F);
            gesture->Point_start.y = (coord[1] << 4) | ((coord[2] >> 0) & 0x0F);
            gesture->Point_end     = gesture->Point_start;
            break;

        case GESTURE_UP_V:
            gesture->gesture_type  = UpVee;
            gesture->Point_start.x = (coord[0] << 4) | ((coord[2] >> 4) & 0x0F);
            gesture->Point_start.y = (coord[1] << 4) | ((coord[2] >> 0) & 0x0F);
            gesture->Point_end.x   = (coord[6] << 4) | ((coord[8] >> 4) & 0x0F);
            gesture->Point_end.y   = (coord[7] << 4) | ((coord[8] >> 0) & 0x0F);
            gesture->Point_1st.x   = (coord[3] << 4) | ((coord[5] >> 4) & 0x0F);
            gesture->Point_1st.y   = (coord[4] << 4) | ((coord[5] >> 0) & 0x0F);
            break;

        case GESTURE_DOWN_V:
            gesture->gesture_type  = DownVee;
            gesture->Point_start.x = (coord[0] << 4) | ((coord[2] >> 4) & 0x0F);
            gesture->Point_start.y = (coord[1] << 4) | ((coord[2] >> 0) & 0x0F);
            gesture->Point_end.x   = (coord[6] << 4) | ((coord[8] >> 4) & 0x0F);
            gesture->Point_end.y   = (coord[7] << 4) | ((coord[8] >> 0) & 0x0F);
            gesture->Point_1st.x   = (coord[3] << 4) | ((coord[5] >> 4) & 0x0F);
            gesture->Point_1st.y   = (coord[4] << 4) | ((coord[5] >> 0) & 0x0F);
            break;

        case GESTURE_LEFT_V:
            gesture->gesture_type = LeftVee;
            gesture->Point_start.x = (coord[0] << 4) | ((coord[2] >> 4) & 0x0F);
            gesture->Point_start.y = (coord[1] << 4) | ((coord[2] >> 0) & 0x0F);
            gesture->Point_end.x   = (coord[6] << 4) | ((coord[8] >> 4) & 0x0F);
            gesture->Point_end.y   = (coord[7] << 4) | ((coord[8] >> 0) & 0x0F);
            gesture->Point_1st.x   = (coord[3] << 4) | ((coord[5] >> 4) & 0x0F);
            gesture->Point_1st.y   = (coord[4] << 4) | ((coord[5] >> 0) & 0x0F);
            break;

        case GESTURE_RIGHT_V:
            gesture->gesture_type  = RightVee;
            gesture->Point_start.x = (coord[0] << 4) | ((coord[2] >> 4) & 0x0F);
            gesture->Point_start.y = (coord[1] << 4) | ((coord[2] >> 0) & 0x0F);
            gesture->Point_end.x   = (coord[6] << 4) | ((coord[8] >> 4) & 0x0F);
            gesture->Point_end.y   = (coord[7] << 4) | ((coord[8] >> 0) & 0x0F);
            gesture->Point_1st.x   = (coord[3] << 4) | ((coord[5] >> 4) & 0x0F);
            gesture->Point_1st.y   = (coord[4] << 4) | ((coord[5] >> 0) & 0x0F);
            break;

        case GESTURE_O:
            gesture->gesture_type = Circle;
            gesture->clockwise = (p_event_gesture->data == 0) ? 1 : 0;
            gesture->Point_start.x = (coord[0] << 4) | ((coord[2] >> 4) & 0x0F);
            gesture->Point_start.y = (coord[1] << 4) | ((coord[2] >> 0) & 0x0F);
            limitPoint[0].x   = (coord[3] << 4) | ((coord[5] >> 4) & 0x0F);    //ymin
            limitPoint[0].y   = (coord[4] << 4) | ((coord[5] >> 0) & 0x0F);
            limitPoint[1].x   = (coord[6] << 4) | ((coord[8] >> 4) & 0x0F);    //xmin
            limitPoint[1].y   = (coord[7] << 4) | ((coord[8] >> 0) & 0x0F);
            limitPoint[2].x   = (coord[9] << 4) | ((coord[11] >> 4) & 0x0F);   //ymax
            limitPoint[2].y   = (coord[10] << 4) | ((coord[11] >> 0) & 0x0F);
            limitPoint[3].x   = (coord[12] << 4) | ((coord[14] >> 4) & 0x0F);  //xmax
            limitPoint[3].y   = (coord[13] << 4) | ((coord[14] >> 0) & 0x0F);
            gesture->Point_end.x   = (coord[15] << 4) | ((coord[17] >> 4) & 0x0F);
            gesture->Point_end.y   = (coord[16] << 4) | ((coord[17] >> 0) & 0x0F);
            handleFourCornerPoint(&limitPoint[0], 4);
            gesture->Point_1st = limitPoint[0]; //ymin
            gesture->Point_2nd = limitPoint[1]; //xmin
            gesture->Point_3rd = limitPoint[2]; //ymax
            gesture->Point_4th = limitPoint[3]; //xmax
            break;

        case GESTURE_DOUBLE_LINE:
            gesture->gesture_type  = DouSwip;
            gesture->Point_start.x = (coord[0] << 4) | ((coord[2] >> 4) & 0x0F);
            gesture->Point_start.y = (coord[1] << 4) | ((coord[2] >> 0) & 0x0F);
            gesture->Point_end.x   = (coord[3] << 4) | ((coord[5] >> 4) & 0x0F);
            gesture->Point_end.y   = (coord[4] << 4) | ((coord[5] >> 0) & 0x0F);
            gesture->Point_1st.x   = (coord[6] << 4) | ((coord[8] >> 4) & 0x0F);
            gesture->Point_1st.y   = (coord[7] << 4) | ((coord[8] >> 0) & 0x0F);
            gesture->Point_2nd.x   = (coord[9] << 4) | ((coord[11] >> 4) & 0x0F);
            gesture->Point_2nd.y   = (coord[10] << 4) | ((coord[11] >> 0) & 0x0F);
            break;

        case GESTURE_M:
            gesture->gesture_type  = Mgestrue;
            gesture->Point_start.x = (coord[0] << 4) | ((coord[2] >> 4) & 0x0F);
            gesture->Point_start.y = (coord[1] << 4) | ((coord[2] >> 0) & 0x0F);
            gesture->Point_1st.x   = (coord[3] << 4) | ((coord[5] >> 4) & 0x0F);
            gesture->Point_1st.y   = (coord[4] << 4) | ((coord[5] >> 0) & 0x0F);
            gesture->Point_2nd.x   = (coord[6] << 4) | ((coord[8] >> 4) & 0x0F);
            gesture->Point_2nd.y   = (coord[7] << 4) | ((coord[8] >> 0) & 0x0F);
            gesture->Point_3rd.x   = (coord[9] << 4) | ((coord[11] >> 4) & 0x0F);
            gesture->Point_3rd.y   = (coord[10] << 4) | ((coord[11] >> 0) & 0x0F);
            gesture->Point_end.x   = (coord[12] << 4) | ((coord[14] >> 4) & 0x0F);
            gesture->Point_end.y   = (coord[13] << 4) | ((coord[14] >> 0) & 0x0F);
            break;

        case GESTURE_W:
            gesture->gesture_type  = Wgestrue;
            gesture->Point_start.x = (coord[0] << 4) | ((coord[2] >> 4) & 0x0F);
            gesture->Point_start.y = (coord[1] << 4) | ((coord[2] >> 0) & 0x0F);
            gesture->Point_1st.x   = (coord[3] << 4) | ((coord[5] >> 4) & 0x0F);
            gesture->Point_1st.y   = (coord[4] << 4) | ((coord[5] >> 0) & 0x0F);
            gesture->Point_2nd.x   = (coord[6] << 4) | ((coord[8] >> 4) & 0x0F);
            gesture->Point_2nd.y   = (coord[7] << 4) | ((coord[8] >> 0) & 0x0F);
            gesture->Point_3rd.x   = (coord[9] << 4) | ((coord[11] >> 4) & 0x0F);
            gesture->Point_3rd.y   = (coord[10] << 4) | ((coord[11] >> 0) & 0x0F);
            gesture->Point_end.x   = (coord[12] << 4) | ((coord[14] >> 4) & 0x0F);
            gesture->Point_end.y   = (coord[13] << 4) | ((coord[14] >> 0) & 0x0F);
            break;

        case GESTURE_SINGLE_TAP:
            gesture->gesture_type  = SingleTap;
            gesture->Point_start.x = (coord[0] << 4) | ((coord[2] >> 4) & 0x0F);
            gesture->Point_start.y = (coord[1] << 4) | ((coord[2] >> 0) & 0x0F);
            break;

        case GESTURE_S:
            gesture->gesture_type  = Sgestrue;
            gesture->Point_start.x = (coord[0] << 4) | ((coord[2] >> 4) & 0x0F);
            gesture->Point_start.y = (coord[1] << 4) | ((coord[2] >> 0) & 0x0F);
            gesture->Point_1st.x   = (coord[3] << 4) | ((coord[5] >> 4) & 0x0F);
            gesture->Point_1st.y   = (coord[4] << 4) | ((coord[5] >> 0) & 0x0F);
            gesture->Point_2nd.x   = (coord[6] << 4) | ((coord[8] >> 4) & 0x0F);
            gesture->Point_2nd.y   = (coord[7] << 4) | ((coord[8] >> 0) & 0x0F);
            gesture->Point_end.x   = (coord[9] << 4) | ((coord[11] >> 4) & 0x0F);
            gesture->Point_end.y   = (coord[10] << 4) | ((coord[11] >> 0) & 0x0F);
            break;


        default:
            gesture->gesture_type = UnkownGesture;
            break;
    }

    TPD_INFO("%s, gesture_id: 0x%x, gesture_type: %d, clockwise: %d, points: (%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)\n", \
                __func__, p_event_gesture->gestureId, gesture->gesture_type, gesture->clockwise, \
                gesture->Point_start.x, gesture->Point_start.y, \
                gesture->Point_end.x, gesture->Point_end.y, \
                gesture->Point_1st.x, gesture->Point_1st.y, \
                gesture->Point_2nd.x, gesture->Point_2nd.y, \
                gesture->Point_3rd.x, gesture->Point_3rd.y, \
                gesture->Point_4th.x, gesture->Point_4th.y);

    return 0;
}

static int sec_mode_switch(void *chip_data, work_mode mode, bool flag)
{
    int ret = -1;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;

    if (chip_info->is_power_down) {
        sec_power_control(chip_info, true);
    }

    switch(mode) {
        case MODE_NORMAL:
            ret = 0;
            break;

        case MODE_SLEEP:
            ret = sec_power_control(chip_info, false);
            if (ret < 0) {
                TPD_INFO("%s: power down failed\n", __func__);
            }
            break;

        case MODE_GESTURE:
            ret = sec_enable_black_gesture(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: sec enable gesture failed.\n", __func__);
                return ret;
            }
            break;

        case MODE_EDGE:
            ret = sec_enable_edge_limit(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: sec enable edg limit failed.\n", __func__);
                return ret;
            }
            break;

        case MODE_CHARGE:
            ret = sec_enable_charge_mode(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: enable charge mode : %d failed\n", __func__, flag);
            }
            break;

        case MODE_EARSENSE:
            ret = sec_enable_earsense_mode(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: enable earsense mode : %d failed\n", __func__, flag);
            }
            break;

		case MODE_FACE_DETECT:
			ret = sec_enable_face_mode(chip_info, flag);
			if (ret < 0) {
				TPD_INFO("%s: enable face detect mode : %d failed\n", __func__, flag);
			}
			break;

		case MODE_FACE_CALIBRATE:
			ret = sec_face_reduce_mode(chip_info, flag);
			if (ret < 0) {
				TPD_INFO("%s: enable face reduce mode : %d failed\n", __func__, flag);
			}
			break;
        case MODE_PALM_REJECTION:
            ret = sec_enable_palm_reject(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: enable palm rejection: %d failed\n", __func__, flag);
            }
            break;

        case MODE_GAME:
            ret = sec_enable_game_mode(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: enable game mode: %d failed\n", __func__, flag);
            }
            break;

		case MODE_REFRESH_SWITCH:
			ret = sec_refresh_switch_mode(chip_info, flag);
			if (ret < 0) {
				TPD_INFO("%s: swhitch refresh rate mode: %d failed\n", __func__, flag);
			}
			break;

		case MODE_TOUCH_HOLD:
			ret = sec_touchhold_switch_mode(chip_info, flag);
			if (ret < 0) {
				TPD_INFO("%s: open touchhold mode: %d failed\n", __func__, flag);
			}
			break;

		case MODE_TOUCH_AREA_SWITCH:
			ret = sec_toucharea_switch_mode(chip_info, flag);
			if (ret < 0) {
				TPD_INFO("%s: switch touchhold area: %d failed\n", __func__, flag);
			}
			break;

		case MODE_LIMIT_SWITCH:
			ret = sec_limit_switch_mode(chip_info, flag);
			if (ret < 0) {
				TPD_INFO("%s: limit switch: %d failed\n", __func__, flag);
			}
			break;

		case MODE_GESTURE_SWITCH:
			ret = sec_gesture_switch_mode(chip_info, flag);
			if (ret < 0) {
				TPD_INFO("%s: switch gestrue mode: %d failed\n", __func__, flag);
			}
			break;

        default:
            TPD_INFO("%s: Wrong mode.\n", __func__);
    }

    return ret;
}

static int sec_get_face_detect(void * chip_data)
{
    int state = -1;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;

	if (chip_info->proximity_status == 0x2E) {
		state = 0;		//far
	} else if (chip_info->proximity_status == 0) {
		state = 1;		//near
	}
    return state;
}


#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
extern unsigned int upmu_get_rgs_chrdet(void);
static int sec_get_usb_state(void)
{
    return upmu_get_rgs_chrdet();
}
#else
static int sec_get_usb_state(void)
{
    return 0;
}
#endif

static struct touchpanel_operations sec_ops = {
    .get_vendor                 = sec_get_vendor,
    .get_chip_info              = sec_get_chip_info,
    .reset                      = sec_reset,
    .power_control              = sec_power_control,
    .fw_check                   = sec_fw_check,
    .fw_update                  = sec_fw_update,
    .trigger_reason             = sec_trigger_reason,
    .get_touch_points           = sec_get_touch_points,
    .get_gesture_info           = sec_get_gesture_info,
    .mode_switch                = sec_mode_switch,
    .get_usb_state              = sec_get_usb_state,
    .get_face_state				= sec_get_face_detect,
};
/********* End of implementation of touchpanel_operations callbacks**********************/


/**************** Start of implementation of debug_info proc callbacks************************/
int sec_fix_tmode(struct chip_data_s6sy761 *chip_info, u8 mode, u8 state)
{
    int ret = -1;
    u8 tBuff[2] = { mode, state };

    ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_STATEMANAGE_ON, STATE_MANAGE_OFF);
    sec_mdelay(20);
    ret |= touch_i2c_write_block(chip_info->client, SEC_CMD_CHG_SYSMODE, sizeof(tBuff), tBuff);
    sec_mdelay(20);

    return ret;
}

int sec_release_tmode(struct chip_data_s6sy761 *chip_info)
{
    int ret = -1;

    ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_STATEMANAGE_ON, STATE_MANAGE_ON);
    sec_mdelay(20);

    return ret;
}

static int sec_read_self(struct chip_data_s6sy761 *chip_info, u8 type, char *data, int len)
{
    int ret = 0;
    unsigned int data_len = (chip_info->hw_res->TX_NUM + chip_info->hw_res->RX_NUM) * 2;

    if (len != data_len) {
        return -1;
    }

    ret = sec_fix_tmode(chip_info, TOUCH_SYSTEM_MODE_TOUCH, TOUCH_MODE_STATE_TOUCH);
    if (ret < 0) {
        TPD_INFO("%s: fix touch mode failed\n", __func__);
        goto err_out;
    }

    ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_SELF_RAW_TYPE, type);
    if (ret < 0) {
        TPD_INFO("%s: Set self type failed\n", __func__);
        goto err_out;
    }

    sec_mdelay(50);
    ret = touch_i2c_read_block(chip_info->client, SEC_READ_TOUCH_SELF_RAWDATA, data_len, data);
    if (ret < 0) {
        TPD_INFO("%s: read self failed!\n", __func__);
    }

    /* release data monitory (unprepare AFE data memory) */
    ret |= touch_i2c_write_byte(chip_info->client, SEC_CMD_SELF_RAW_TYPE, TYPE_INVALID_DATA);
    if (ret < 0) {
        TPD_INFO("%s: Set self type failed\n", __func__);
    }

err_out:
    ret |= sec_release_tmode(chip_info);

    return ret;
}

static int sec_read_mutual(struct chip_data_s6sy761 *chip_info, u8 type, char *data, int len)
{
    int ret = 0;
    u8 buf[2] = {0};
    unsigned int data_len = (chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM) * 2;

    if ((len > data_len) || (len % chip_info->hw_res->TX_NUM != 0)) {
        return -1;
    }

    ret = sec_fix_tmode(chip_info, TOUCH_SYSTEM_MODE_TOUCH, TOUCH_MODE_STATE_TOUCH);
    if (ret < 0) {
        TPD_INFO("%s: fix touch mode failed\n", __func__);
        goto err_out;
    }

    ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_MUTU_RAW_TYPE, type);
    if (ret < 0) {
        TPD_INFO("%s: Set mutual type failed\n", __func__);
        goto err_out;
    }

    sec_mdelay(20);
    buf[0] = (u8)((len >> 8) & 0xFF);
    buf[1] = (u8)(len & 0xFF);
    touch_i2c_write_block(chip_info->client, SEC_CMD_TOUCH_RAWDATA_SETLEN, 2, buf);
    ret = touch_i2c_read_block(chip_info->client, SEC_READ_TOUCH_RAWDATA, len, data);
    if (ret < 0) {
        TPD_INFO("%s: read mutual failed!\n", __func__);
    }

    /* release data monitory (unprepare AFE data memory) */
    ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_MUTU_RAW_TYPE, TYPE_INVALID_DATA);
    if (ret < 0) {
        TPD_INFO("%s: Set mutual type failed\n", __func__);
    }

err_out:
    ret |= sec_release_tmode(chip_info);

    return ret;
}

static void sec_delta_read(struct seq_file *s, void *chip_data)
{
    u8 *data = NULL;
    int16_t x = 0, y = 0, z = 0, temp_delta = 0, ret = 0;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;
    int readbytes = chip_info->hw_res->TX_NUM * (chip_info->hw_res->RX_NUM) * 2;

    data = kmalloc(readbytes, GFP_KERNEL);
    if (!data) {
        return;
    }

    memset(data, 0, readbytes);
    ret = sec_read_mutual(chip_info, TYPE_SIGNAL_DATA, data, readbytes);
    if (ret < 0) {
        seq_printf(s, "read delta failed\n");
        goto kfree_out;
    }

    for (x = 0; x < chip_info->hw_res->TX_NUM; x++) {
        seq_printf(s, "\n[%2d]", x);
        for (y = chip_info->hw_res->RX_NUM - 1; y >= 0; y--) {
            z = chip_info->hw_res->RX_NUM * x + y;
            temp_delta = ((data[z * 2] << 8) | data[z * 2 + 1]);
            seq_printf(s, "%4d, ", temp_delta);
        }
    }
    seq_printf(s, "\n");

kfree_out:
    kfree(data);
    return;
}

static void sec_baseline_read(struct seq_file *s, void *chip_data)
{
    u8 *data = NULL;
    int16_t x = 0, y = 0, z = 0, temp_delta = 0, ret = -1;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;
    int readbytes = (chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM) * 2;

    data = kmalloc(readbytes, GFP_KERNEL);
    if (!data) {
        return;
    }

    //read decoded data
    memset(data, 0, readbytes);
    ret = sec_read_mutual(chip_info, TYPE_DECODED_DATA, data, readbytes);
    if (ret < 0) {
        seq_printf(s, "read rawdata failed\n");
        goto kfree_out;
    }

    seq_printf(s, "DECODED DATA:");
    for (x = 0; x < chip_info->hw_res->TX_NUM; x++) {
        seq_printf(s, "\n[%2d]", x);
        for (y = chip_info->hw_res->RX_NUM - 1; y >= 0; y--) {
            z = chip_info->hw_res->RX_NUM * x + y;
            temp_delta = (data[z * 2] << 8) | data[z * 2 + 1];
            seq_printf(s, "%4d, ", temp_delta);
        }
    }
    seq_printf(s, "\n");

    //read ambient data
    memset(data, 0, readbytes);
    ret = sec_read_mutual(chip_info, TYPE_AMBIENT_DATA, data, readbytes);
    if (ret < 0) {
        seq_printf(s, "read rawdata failed\n");
        goto kfree_out;
    }

    seq_printf(s, "AMBIENT DATA:");
    for (x = 0; x < chip_info->hw_res->TX_NUM; x++) {
        seq_printf(s, "\n[%2d]", x);
        for (y = chip_info->hw_res->RX_NUM - 1; y >= 0; y--) {
            z = chip_info->hw_res->RX_NUM * x + y;
            temp_delta = (data[z * 2] << 8) | data[z * 2 + 1];
            seq_printf(s, "%4d, ", temp_delta);
        }
    }
    seq_printf(s, "\n");

kfree_out:
    kfree(data);
    return;
}

static void sec_self_delta_read(struct seq_file *s, void *chip_data)
{
    u8 *data = NULL;
    int16_t x = 0, rx_offset = 0, temp_delta = 0, ret = -1;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;
    int readbytes = (chip_info->hw_res->TX_NUM + chip_info->hw_res->RX_NUM) * 2;

    data = kmalloc(readbytes, GFP_KERNEL);
    if (!data) {
        return;
    }

    memset(data, 0, readbytes);
    ret = sec_read_self(chip_info, TYPE_SIGNAL_DATA, data, readbytes);
    if (ret < 0) {
        seq_printf(s, "read self delta failed\n");
        goto kfree_out;
    }

    seq_printf(s, "TX:\n");
    for (x = 0; x < chip_info->hw_res->TX_NUM; x++) {
        temp_delta = (data[x * 2] << 8) | data[x * 2 + 1];
        seq_printf(s, "%4d, ", temp_delta);
    }

    seq_printf(s, "\nRX:\n");
    rx_offset = chip_info->hw_res->TX_NUM * 2;
    for (x = 0; x < chip_info->hw_res->RX_NUM; x++) {
        temp_delta = (data[x * 2 + rx_offset] << 8) | data[x * 2 + 1 + rx_offset];
        seq_printf(s, "%4d, ", temp_delta);
    }
    seq_printf(s, "\n");

kfree_out:
    kfree(data);
    return;
}

static void sec_self_raw_read(struct seq_file *s, void *chip_data)
{
    u8 *data = NULL;
    int16_t x = 0, rx_offset = 0, temp_delta = 0, ret = -1;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;
    int readbytes = (chip_info->hw_res->TX_NUM + chip_info->hw_res->RX_NUM) * 2;

    data = kmalloc(readbytes, GFP_KERNEL);
    if (!data) {
        return;
    }

    memset(data, 0, readbytes);
    ret = sec_read_self(chip_info, TYPE_RAW_DATA, data, readbytes);
    if (ret < 0) {
        seq_printf(s, "read self rawdata failed\n");
        goto kfree_out;
    }

    seq_printf(s, "TX:\n");
    for (x = 0; x < chip_info->hw_res->TX_NUM; x++) {
        temp_delta = (data[x * 2] << 8) | data[x * 2 + 1];
        seq_printf(s, "%4d, ", temp_delta);
    }

    seq_printf(s, "\nRX:\n");
    rx_offset = chip_info->hw_res->TX_NUM * 2;
    for (x = 0; x < chip_info->hw_res->RX_NUM; x++) {
        temp_delta = (data[x * 2 + rx_offset] << 8) | data[x * 2 + 1 + rx_offset];
        seq_printf(s, "%4d, ", temp_delta);
    }
    seq_printf(s, "\n");

kfree_out:
    kfree(data);
    return;
}

static void sec_main_register_read(struct seq_file *s, void *chip_data)
{
    u8 buf[4] = {0};
    int state = -1;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;

    state = sec_read_calibration_report(chip_info);
    seq_printf(s, "calibration status: 0x%02x\n", state);

    state = sec_get_verify_result(chip_info);
    seq_printf(s, "calibration result: 0x%02x\n", state);

    state = touch_i2c_read_byte(chip_info->client, SET_CMD_SET_CHARGER_MODE);
    seq_printf(s, "charger state: 0x%02x\n", state);

    memset(buf, 0, 4);
    touch_i2c_read_block(chip_info->client, SEC_READ_ID, 3, buf);
    seq_printf(s, "boot state: 0x%02x\n", buf[0]);

    state = touch_i2c_read_byte(chip_info->client, 0x30);
    seq_printf(s, "touch function: 0x%02x(proximity/wetmode/palm/stylus/glove/cover/hover/touch)\n", state);

    state = touch_i2c_read_byte(chip_info->client, 0x3B);
    seq_printf(s, "wetmode state: 0x%02x\n", state);

    state = touch_i2c_read_word(chip_info->client, SEC_CMD_WAKEUP_GESTURE_MODE);
    seq_printf(s, "gesture mode: 0x%04x\n", state);

    memset(buf, 0, 4);
    touch_i2c_read_block(chip_info->client, SEC_READ_IMG_VERSION, 4, buf);
    seq_printf(s, "fw img version: 0x%08x\n", (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3]);

    memset(buf, 0, 4);
    touch_i2c_read_block(chip_info->client, SEC_READ_CONFIG_VERSION, 4, buf);
    seq_printf(s, "config version: 0x%08x\n", (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3]);

    state = touch_i2c_read_byte(chip_info->client, SEC_CMD_STATEMANAGE_ON);
    seq_printf(s, "auto change enabled: 0x%02x\n", state);

    state = touch_i2c_read_byte(chip_info->client, SEC_CMD_HOVER_DETECT);
    seq_printf(s, "earsense state: 0x%02x\n", state);

    state = touch_i2c_read_word(chip_info->client, SEC_CMD_PALM_SWITCH);
    seq_printf(s, "palm state: 0x%04x(0x0041-off/0x0061-on)\n", state);

    state = touch_i2c_read_word(chip_info->client, SEC_CMD_GRIP_SWITCH);
    seq_printf(s, "grip state: 0x%04x(0x0000-off/0x2000-on)\n", state);

    memset(buf, 0, 4);
    touch_i2c_read_block(chip_info->client, SEC_READ_TS_STATUS, 4, buf);
    seq_printf(s, "power mode: 0x%02x[normal-0x02/lpwg-0x05], 0x%02x[indle-0x00/active-0x02]\n", buf[1], buf[3]);
    return;
}

static void sec_reserve_read(struct seq_file *s, void *chip_data)
{
    static int int_state = 1;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;

    if (int_state) {
        int_state = 0;
        touch_i2c_write_byte(chip_info->client, SEC_CMD_INTERRUPT_SWITCH, 1);   //disable interrupt
    } else {
        int_state = 1;
        touch_i2c_write_block(chip_info->client, SEC_CMD_CLEAR_EVENT_STACK, 0, NULL);   //clear event buffer
        touch_i2c_write_byte(chip_info->client, SEC_CMD_INTERRUPT_SWITCH, 0);   //enable interrupt
    }
}

static struct debug_info_proc_operations debug_info_proc_ops = {
    .limit_read         = sec_limit_read,
    .delta_read         = sec_delta_read,
    .self_delta_read    = sec_self_delta_read,
    .baseline_read      = sec_baseline_read,
    .self_raw_read      = sec_self_raw_read,
    .main_register_read = sec_main_register_read,
    .reserve_read       = sec_reserve_read,
};

static void sec_earsese_rawdata_read(void *chip_data, char *rawdata, int read_len)
{
    int ret = 0;
    u8 buf[2] = {0};
    int i = 0, j = 0;
    int8_t tmp_byte[2] = {0};
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;
    uint8_t len_y = chip_info->hw_res->EARSENSE_RX_NUM;
	uint8_t len_x = chip_info->hw_res->EARSENSE_TX_NUM;

	if ((!chip_info) || (!rawdata))
        return;

    buf[0] = (u8)((read_len >> 8) & 0xFF);
    buf[1] = (u8)(read_len & 0xFF);
    touch_i2c_write_byte(chip_info->client, SEC_CMD_MUTU_RAW_TYPE, TYPE_DATA_RAWDATA);
    touch_i2c_write_block(chip_info->client, SEC_CMD_TOUCH_RAWDATA_SETLEN, 2, buf);
    ret = touch_i2c_read_block(chip_info->client, SEC_CMD_TOUCH_RAWDATA_READ, read_len, rawdata);
    if (ret < 0) {
        TPD_INFO("read rawdata failed\n");
        return;
    }
	TPD_INFO("sec_earsese_rawdata_read lenx = %d, leny =%d\n",len_x,len_y);
	for(i = 0; i < read_len; i++ ) {
		TPD_INFO("rawdata = %d\n",rawdata[i]);
	}
    for (i = 0; i < len_y; i++) {
        for (j = 0; j < len_x/2; j++) {
            tmp_byte[0] = rawdata[2*(len_x*i+j)];
            tmp_byte[1] = rawdata[2*(len_x*i+j)+1];
            rawdata[2*(len_x*i+j)] = rawdata[2*(len_x*i+len_x-1-j)+1];
            rawdata[2*(len_x*i+j)+1] = rawdata[2*(len_x*i+len_x-1-j)];
            rawdata[2*(len_x*i+len_x-1-j)] = tmp_byte[1];
            rawdata[2*(len_x*i+len_x-1-j)+1] = tmp_byte[0];
        }
    }
    if (len_x%2) {
        j = len_x/2;
        for (i = 0; i < len_y; i++) {
            tmp_byte[0] = rawdata[2*(len_x*i+j)];
            rawdata[2*(len_x*i+j)] = rawdata[2*(len_x*i+j)+1];
            rawdata[2*(len_x*i+j)+1] = tmp_byte[0];
        }
    }
    return;
}

static void sec_earsese_delta_read(void *chip_data, char *earsense_delta, int read_len)
{
    int ret = 0, hover_status = 0, data_type = 0;
    u8 buf[2] = {0};
    int i = 0, j = 0;
    int8_t tmp_byte[2] = {0};
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;
    uint8_t len_x = chip_info->hw_res->EARSENSE_TX_NUM;
    uint8_t len_y = chip_info->hw_res->EARSENSE_RX_NUM;

    if (!chip_info)
        return ;

    if (!earsense_delta) {
        TPD_INFO("earsense_delta is NULL\n");
        return;
    }

    buf[0] = (u8)((read_len >> 8) & 0xFF);
    buf[1] = (u8)(read_len & 0xFF);
    hover_status = touch_i2c_read_byte(chip_info->client, SEC_CMD_HOVER_DETECT); //read hover state
    data_type = touch_i2c_read_byte(chip_info->client, SEC_CMD_MUTU_RAW_TYPE); //read current data type
    if (hover_status && (data_type != TYPE_DATA_DELTA)) {
        touch_i2c_write_byte(chip_info->client, SEC_CMD_MUTU_RAW_TYPE, TYPE_DATA_DELTA);
        sec_mdelay(20);
    } else if (!hover_status && (data_type != TYPE_SIGNAL_DATA)){
        touch_i2c_write_byte(chip_info->client, SEC_CMD_MUTU_RAW_TYPE, TYPE_SIGNAL_DATA);
        sec_mdelay(20);
    }

    touch_i2c_write_block(chip_info->client, SEC_CMD_TOUCH_RAWDATA_SETLEN, 2, buf);
    ret = touch_i2c_read_block(chip_info->client, SEC_CMD_TOUCH_DELTA_READ, read_len, earsense_delta);
    if (ret < 0) {
        TPD_INFO("read delta failed\n");
        return;
    }
	for(i = 0; i < read_len; i++ ) {
		TPD_INFO("earsense_delta = %d\n",earsense_delta[i]);
	}
    for (i = 0; i < len_y; i++) {
        for (j = 0; j < len_x/2; j++) {
            tmp_byte[0] = earsense_delta[2*(len_x*i+j)];
            tmp_byte[1] = earsense_delta[2*(len_x*i+j)+1];
            earsense_delta[2*(len_x*i+j)] = earsense_delta[2*(len_x*i+len_x-1-j)+1];
            earsense_delta[2*(len_x*i+j)+1] = earsense_delta[2*(len_x*i+len_x-1-j)];
            earsense_delta[2*(len_x*i+len_x-1-j)] = tmp_byte[1];
            earsense_delta[2*(len_x*i+len_x-1-j)+1] = tmp_byte[0];
        }
    }
    if (len_x%2) {
        j = len_x/2;
        for (i = 0; i < len_y; i++) {
            tmp_byte[0] = earsense_delta[2*(len_x*i+j)];
            earsense_delta[2*(len_x*i+j)] = earsense_delta[2*(len_x*i+j)+1];
            earsense_delta[2*(len_x*i+j)+1] = tmp_byte[0];
        }
    }
    return;
}

static void sec_earsese_selfdata_read( void *chip_data, char *self_data, int read_len)
{
    int i = 0, ret = 0;
    int8_t tmp = 0;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;

    if ((!chip_info) || (!self_data))
        return ;

    ret = touch_i2c_read_block(chip_info->client, SEC_CMD_TOUCH_SELFDATA_READ, read_len, self_data);
    if (ret < 0) {
        TPD_INFO("read selfdata failed\n");
        return;
    }
	for(i = 0; i < read_len; i++ ) {
		TPD_INFO("self_data = %d\n",self_data[i]);
	}
    for (i = 0; i < chip_info->hw_res->TX_NUM + chip_info->hw_res->RX_NUM; i++) {
        tmp = self_data[2*i];
        self_data[2*i] = self_data[2*i + 1];
        self_data[2*i + 1] = tmp;
    }
    return;
}


static struct earsense_proc_operations earsense_proc_ops = {
    .rawdata_read = sec_earsese_rawdata_read,
    .delta_read   = sec_earsese_delta_read,
    .self_data_read = sec_earsese_selfdata_read,
};
/***************** End of implementation of debug_info proc callbacks*************************/
static void sec_swap(u8 *a, u8 *b)
{
    u8 temp = *a;
    *a = *b;
    *b = temp;
}

static void rearrange_sft_result(u8 *data, int length)
{
    int i = 0;

    for(i = 0; i < length; i += 4) {
        sec_swap(&data[i], &data[i + 3]);
        sec_swap(&data[i + 1], &data[i + 2]);
    }
}

static int sec_execute_selftest(struct seq_file *s, struct chip_data_s6sy761 *chip_info, struct sec_testdata *sec_testdata)
{
    int rc = -1;
    u8 tpara[2] = {0x25, 0x40};
    u8 *rBuff = NULL;
    int i;
    int result_size = SEC_SELFTEST_REPORT_SIZE + sec_testdata->TX_NUM * sec_testdata->RX_NUM * 2;

    /* save selftest result in flash */
    tpara[0] = 0x21;

    rBuff = kzalloc(result_size, GFP_KERNEL);
    if (!rBuff) {
        seq_printf(s, "kzalloc space failed\n");
        return -ENOMEM;
    }

    //send self test cmd
    rc = touch_i2c_write_block(chip_info->client, SEC_CMD_SELFTEST, 2, tpara);
    if (rc < 0) {
        TPD_INFO("%s: Send selftest cmd failed!\n", __func__);
        seq_printf(s, "Send selftest cmd failed!\n");
        goto ERR_EXIT;
    }

    sec_mdelay(350);

    rc = sec_wait_for_ready(chip_info, SEC_VENDOR_ACK_SELF_TEST_DONE);
    if (rc < 0) {
        TPD_INFO("%s: Selftest execution time out!\n", __func__);
        seq_printf(s, "Selftest execution time out!\n");
        goto ERR_EXIT;
    }

    rc = touch_i2c_read_block(chip_info->client, SEC_READ_SELFTEST_RESULT, result_size, rBuff);
    if (rc < 0) {
        TPD_INFO("%s: read selftest relest failed\n", __func__);
        seq_printf(s, "read selftest relest failed\n");
        goto ERR_EXIT;
    }
    rearrange_sft_result(rBuff, result_size);

    TPD_INFO("sec_ts : \n");
    for (i = 0; i < 80; i += 4) {
        if (i / 4 == 0) TPD_DEBUG_NTAG("SIG");
        else if (i / 4 == 1) TPD_DEBUG_NTAG("VER");
        else if (i / 4 == 2) TPD_DEBUG_NTAG("SIZ");
        else if (i / 4 == 3) TPD_DEBUG_NTAG("CRC");
        else if (i / 4 == 4) TPD_DEBUG_NTAG("RES");
        else if (i / 4 == 5) TPD_DEBUG_NTAG("COU");
        else if (i / 4 == 6) TPD_DEBUG_NTAG("PAS");
        else if (i / 4 == 7) TPD_DEBUG_NTAG("FAI");
        else if (i / 4 == 8) TPD_DEBUG_NTAG("CHA");
        else if (i / 4 == 9) TPD_DEBUG_NTAG("AMB");
        else if (i / 4 == 10) TPD_DEBUG_NTAG("RXS");
        else if (i / 4 == 11) TPD_DEBUG_NTAG("TXS");
        else if (i / 4 == 12) TPD_DEBUG_NTAG("RXO");
        else if (i / 4 == 13) TPD_DEBUG_NTAG("TXO");
        else if (i / 4 == 14) TPD_DEBUG_NTAG("RXG");
        else if (i / 4 == 15) TPD_DEBUG_NTAG("TXG");
        else if (i / 4 == 16) TPD_DEBUG_NTAG("RXR");
        else if (i / 4 == 17) TPD_DEBUG_NTAG("TXT");
        else if (i / 4 == 18) TPD_DEBUG_NTAG("RXT");
        else if (i / 4 == 19) TPD_DEBUG_NTAG("TXR");

        TPD_DEBUG_NTAG(": %2X, %2X, %2X, %2X \n", rBuff[i], rBuff[i + 1], rBuff[i + 2], rBuff[i + 3]);

        if (i / 4 == 4) {
            /* RX, RX open check. */
            if ((rBuff[i + 3] & 0x30) != 0) {
                rc = 0;
                seq_printf(s, "RX, RX open check failed\n");
            }
            /* TX, RX GND(VDD) short check. */
            else if ((rBuff[i + 3] & 0xC0) != 0) {
                rc = 0;
                seq_printf(s, "TX, RX GND(VDD) short check failed\n");
            }
            /* RX-RX, TX-TX short check. */
            else if ((rBuff[i + 2] & 0x03) != 0) {
                rc = 0;
                seq_printf(s, "RX-RX, TX-TX short check failed\n");
            }
            /* TX-RX short check. */
            else if ((rBuff[i + 2] & 0x04) != 0) {
                rc = 0;
                seq_printf(s, "TX-RX short check failed\n");
            }
            else
                rc = 1;
        }

    }

ERR_EXIT:
    kfree(rBuff);
    return rc;
}

int sec_execute_p2ptest(struct seq_file *s, struct chip_data_s6sy761 *chip_info, struct sec_testdata *sec_testdata)
{
    int rc;
    u8 tpara[2] = {0x0F, 0x11};

    TPD_INFO("%s: P2P test start!\n", __func__);
    rc = touch_i2c_write_block(chip_info->client, SEC_CMD_SET_P2PTEST_MODE, 2, tpara);
    if (rc < 0) {
        seq_printf(s, "%s: Send P2Ptest Mode cmd failed!\n", __func__);
        goto err_exit;
    }

    sec_mdelay(15);

    tpara[0] = 0x00;
    tpara[1] = 0x64;
    rc = touch_i2c_write_block(chip_info->client, SEC_CMD_P2PTEST, 2, tpara);
    if (rc < 0) {
        seq_printf(s, "%s: Send P2Ptest cmd failed!\n", __func__);
        goto err_exit;
    }

    sec_mdelay(1000);

    rc = sec_wait_for_ready(chip_info, SEC_VENDOR_ACK_P2P_TEST_DONE);
    if (rc < 0) {
        seq_printf(s, "%s: P2Ptest execution time out!\n", __func__);
        goto err_exit;
    }

    TPD_INFO("%s: P2P test done!\n", __func__);

err_exit:
    return rc;
}

static void store_to_file(int fd, char* format, ...)
{
    va_list args;
    char buf[64] = {0};

    va_start(args, format);
    vsnprintf(buf, 64, format, args);
    va_end(args);

    if(fd >= 0) {
        sys_write(fd, buf, strlen(buf));
    }
}

static uint32_t search_for_item(const struct firmware *fw, int item_cnt, uint8_t item_index)
{
    int i = 0;
    uint32_t item_offset = 0;
    struct sec_test_item_header *item_header = NULL;
    uint32_t *p_item_offset = (uint32_t *)(fw->data + 16);

    for (i = 0; i < item_cnt; i++) {
        item_header = (struct sec_test_item_header *)(fw->data + p_item_offset[i]);
        if (item_header->item_bit == item_index) {      //check the matched item offset
            item_offset = p_item_offset[i];
        }
    }

    return item_offset;
}

static void sec_auto_test(struct seq_file *s, void *chip_data, struct sec_testdata *sec_testdata)
{
    u8 type = 0;
    uint32_t err_cnt = 0, item_offset = 0;
    uint8_t *pRead = NULL;
    int eint_status = 0, eint_count = 0, read_gpio_num = 0;
    int16_t nodeData = 0, Buff16 = 0, Buff16_2 = 0, nodeGap = 0;
    int i = 0, j = 0, item_cnt = 0, iArrayIndex = 0, iArrayIndex_2 = 0, ret = -1;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;
    const uint32_t readbytes = sec_testdata->TX_NUM * sec_testdata->RX_NUM * 2;
    const uint32_t readselfbytes = (sec_testdata->TX_NUM + sec_testdata->RX_NUM) * 2;
    struct sec_test_item_header *item_header = NULL;
    int32_t *p_mutual_p = NULL, *p_mutual_n = NULL, *p_mutualGap_p = NULL, *p_mutualGap_n = NULL;
    int32_t *p_tx_offset_p = NULL, *p_tx_offset_n = NULL, *p_rx_offset_p = NULL, *p_rx_offset_n = NULL;
    int32_t *p_p2p_p = NULL, *p_p2p_n = NULL;


    //interrupt pin short test
    read_gpio_num = 10;
    touch_i2c_write_byte(chip_info->client, SEC_CMD_INTERRUPT_SWITCH, 1);   //disable interrupt
    touch_i2c_write_block(chip_info->client, SEC_CMD_CLEAR_EVENT_STACK, 0, NULL);   //clear event buffer
    while (read_gpio_num--) {
        sec_mdelay(5);
        eint_status = gpio_get_value(sec_testdata->irq_gpio);
        if (eint_status == 1) {
            eint_count--;
        } else {
            eint_count++;
        }
    }
    TPD_INFO("TP EINT PIN direct short test! eint_count = %d\n", eint_count);
    if (eint_count == 10) {
        TPD_INFO("error :  TP EINT PIN direct short!\n");
		err_cnt++;
        seq_printf(s, "Step 0:eint_status is low, TP EINT direct short\n");
		goto ERR_INT;

		return;
    }
    touch_i2c_write_byte(chip_info->client, SEC_CMD_INTERRUPT_SWITCH, 0);   //enable interrupt

    for (i = 0; i < 8*sizeof(sec_testdata->test_item); i++) {
        if ((sec_testdata->test_item >> i) & 0x01 ) {
            item_cnt++;
        }
    }

    pRead = kzalloc(readbytes, GFP_KERNEL);
    if (!pRead) {
        TPD_INFO("kzalloc space failed\n");
        seq_printf(s, "kzalloc space failed\n");
        return;
    }

    /* execute selftest */
    ret = sec_execute_selftest(s, chip_info, sec_testdata);
    if (ret <= 0) {
        err_cnt++;
        TPD_INFO("%s: execute selftest failed\n", __func__);
        seq_printf(s, "execute selftest failed\n");
        if (ret < 0)
            goto ERR_OUT;
    }

    //test item mutual_raw offset_data_sec
    if (sec_testdata->test_item & (1 << TYPE_MUTUAL_RAW_OFFSET_DATA_SDC)) {
        TPD_INFO("do test item mutual_raw offset_data_sec\n");

        item_offset = search_for_item(sec_testdata->fw, item_cnt, TYPE_MUTUAL_RAW_OFFSET_DATA_SDC);
        if (item_offset == 0) {
            err_cnt++;
            TPD_INFO("search for item limit offset failed\n");
            seq_printf(s, "search for item limit offset failed\n");
            goto ERR_OUT;
        }
        item_header = (struct sec_test_item_header *)(sec_testdata->fw->data + item_offset);
        if (item_header->item_magic != 0x4F50504F) {
            err_cnt++;
            TPD_INFO("test item: %d magic number(%4x) is wrong\n", TYPE_MUTUAL_RAW_OFFSET_DATA_SDC, item_header->item_magic);
            seq_printf(s, "test item: %d magic number(%4x) is wrong\n", TYPE_MUTUAL_RAW_OFFSET_DATA_SDC, item_header->item_magic);
            goto ERR_OUT;
        }
        if (item_header->item_limit_type == LIMIT_TYPE_EACH_NODE_DATA) {
            p_mutual_p = (int32_t *)(sec_testdata->fw->data + item_header->top_limit_offset);
            p_mutualGap_p = (int32_t *)(sec_testdata->fw->data + item_header->top_limit_offset + 4*sec_testdata->TX_NUM*sec_testdata->RX_NUM);
            p_mutual_n = (int32_t *)(sec_testdata->fw->data + item_header->floor_limit_offset);
            p_mutualGap_n = (int32_t *)(sec_testdata->fw->data + item_header->floor_limit_offset + 4*sec_testdata->TX_NUM*sec_testdata->RX_NUM);
        } else {
            err_cnt++;
            TPD_INFO("item: %d has invalid limit type(%d)\n", TYPE_MUTUAL_RAW_OFFSET_DATA_SDC, item_header->item_limit_type);
            seq_printf(s, "item: %d has invalid limit type(%d)\n", TYPE_MUTUAL_RAW_OFFSET_DATA_SDC, item_header->item_limit_type);
            goto ERR_OUT;
        }
        if (item_header->para_num == 0) {
        } else {
            err_cnt++;
            TPD_INFO("item: %d has %d parameter\n", TYPE_MUTUAL_RAW_OFFSET_DATA_SDC, item_header->para_num);
            seq_printf(s, "item: %d has %d parameter\n", TYPE_MUTUAL_RAW_OFFSET_DATA_SDC, item_header->para_num);
            goto ERR_OUT;
        }

        /* set reference data */
        type = TYPE_OFFSET_DATA_SDC;
        ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_MUTU_RAW_TYPE, type);
        if (ret < 0) {
            err_cnt++;
            TPD_INFO("%s: set OFFSET_DATA_SEC type failed\n", __func__);
            seq_printf(s, "set OFFSET_DATA_SEC type failed\n");
            goto ERR_OUT;
        }

        sec_mdelay(50);
        /* read reference data */
        ret = touch_i2c_read_block(chip_info->client, SEC_READ_TOUCH_RAWDATA, readbytes, pRead);
        if (ret < 0) {
            err_cnt++;
            TPD_INFO("%s: read mutual rawdata failed!\n", __func__);
            seq_printf(s, "read mutual rawdata failed\n");
            goto ERR_OUT;
        }
        /* store and judge reference data */
        store_to_file(sec_testdata->fd, "TYPE_MUTUAL_RAW_OFFSET_DATA_SDC:\n");
        for (j = 0; j <  sec_testdata->RX_NUM; j++) {
            for (i = 0; i <  sec_testdata->TX_NUM; i++) {
                iArrayIndex = i * sec_testdata->RX_NUM + j;
                nodeData = (pRead[iArrayIndex*2] << 8) | pRead[iArrayIndex*2+1];
                if (sec_testdata->fd >= 0) {
                    store_to_file(sec_testdata->fd, "%d, ", nodeData);
                }
                if ((nodeData < p_mutual_n[iArrayIndex]) || (nodeData > p_mutual_p[iArrayIndex])) {
                    TPD_INFO(" mutual offset_data failed at data[%d][%d] = %d [%d,%d]\n", i, j, nodeData, p_mutual_n[iArrayIndex], p_mutual_p[iArrayIndex]);
                    if (!err_cnt) {
                        seq_printf(s, "Step 1: mutual offset_data failed at data[%d][%d] = %d [%d,%d]\n", i, j, nodeData, p_mutual_n[iArrayIndex], p_mutual_p[iArrayIndex]);
                    }
                    err_cnt++;
                }
            }
            if (sec_testdata->fd >= 0) {
                store_to_file(sec_testdata->fd, "\n");
            }
        }

        /* CM Offseet gap (left-right)*/
        for (j = 0; j < sec_testdata->RX_NUM; j++) {
            for (i = 0; i < sec_testdata->TX_NUM - 1; i++) {
                iArrayIndex = i * sec_testdata->RX_NUM + j;
                iArrayIndex_2 = (i + 1) * sec_testdata->RX_NUM + j;
                Buff16 = (pRead[iArrayIndex * 2] << 8) | pRead[iArrayIndex * 2 + 1];
                Buff16_2 = (pRead[iArrayIndex_2 * 2] << 8) | pRead[iArrayIndex_2 * 2 + 1];
                if (Buff16 > Buff16_2) {
                     nodeGap = 100 - (Buff16_2 * 100 / Buff16);
                } else {
                    nodeGap = 100 - (Buff16 * 100 / Buff16_2);
                }
                if ((nodeGap > p_mutualGap_p[iArrayIndex]) || (nodeGap < p_mutualGap_n[iArrayIndex])) {
                    TPD_INFO("mutual node[%d, %d]=%d and node[%d, %d]=%d gap beyond [%d, %d]\n", i, j, Buff16, i+1, j, Buff16_2, p_mutualGap_n[iArrayIndex], p_mutualGap_p[iArrayIndex]);
                    if (!err_cnt) {
                        seq_printf(s, "Step 2-1:mutual node[%d, %d]=%d and node[%d, %d]=%d gap beyond [%d, %d]\n", i, j, Buff16, i+1, j, Buff16_2, p_mutualGap_n[iArrayIndex], p_mutualGap_p[iArrayIndex]);
                    }
                    err_cnt++;
                }
            }
        }
        /* CM Offseet gap (up-down)*/
        for (j = 0; j < sec_testdata->RX_NUM - 1; j++) {
            for (i = 0; i < sec_testdata->TX_NUM; i++) {
                iArrayIndex = i * sec_testdata->RX_NUM + j;
                iArrayIndex_2 = i * sec_testdata->RX_NUM + j + 1;
                Buff16 = (pRead[iArrayIndex * 2] << 8) | pRead[iArrayIndex * 2 + 1];
                Buff16_2 = (pRead[iArrayIndex_2 * 2] << 8) | pRead[iArrayIndex_2 * 2 + 1];
                if (Buff16 > Buff16_2) {
                     nodeGap = 100 - (Buff16_2 * 100 / Buff16);
                } else {
                    nodeGap = 100 - (Buff16 * 100 / Buff16_2);
                }
                if ((nodeGap > p_mutualGap_p[iArrayIndex]) || (nodeGap < p_mutualGap_n[iArrayIndex])) {
                    TPD_INFO("mutual node[%d, %d]=%d and node[%d, %d]=%d gap beyond [%d, %d]\n", j, i, Buff16, j+1, i, Buff16_2, p_mutualGap_n[iArrayIndex], p_mutualGap_p[iArrayIndex]);
                    if (!err_cnt) {
                        seq_printf(s, "Step 2-2:mutual node[%d, %d]=%d and node[%d, %d]=%d gap beyond [%d, %d]\n", j, i, Buff16, j+1, i, Buff16_2, p_mutualGap_n[iArrayIndex], p_mutualGap_p[iArrayIndex]);
                    }
                    err_cnt++;
                }
            }
        }
    }

    //test item self_raw offset_data_sec
    if (sec_testdata->test_item & (1 << TYPE_SELF_RAW_OFFSET_DATA_SDC)) {
        TPD_INFO("do test item self_raw offset_data_sec\n");

        item_offset = search_for_item(sec_testdata->fw, item_cnt, TYPE_SELF_RAW_OFFSET_DATA_SDC);
        if (item_offset == 0) {
            err_cnt++;
            TPD_INFO("search for item limit offset failed\n");
            seq_printf(s, "search for item limit offset failed\n");
            goto ERR_OUT;
        }
        item_header = (struct sec_test_item_header *)(sec_testdata->fw->data + item_offset);
        if (item_header->item_magic != 0x4F50504F) {
            err_cnt++;
            TPD_INFO("test item: %d magic number(%4x) is wrong\n", TYPE_SELF_RAW_OFFSET_DATA_SDC, item_header->item_magic);
            seq_printf(s, "test item: %d magic number(%4x) is wrong\n", TYPE_SELF_RAW_OFFSET_DATA_SDC, item_header->item_magic);
            goto ERR_OUT;
        }
        if (item_header->item_limit_type == LIMIT_TYPE_EACH_NODE_DATA) {
            p_tx_offset_p = (int32_t *)(sec_testdata->fw->data + item_header->top_limit_offset);
            p_tx_offset_n = (int32_t *)(sec_testdata->fw->data + item_header->top_limit_offset + 4*sec_testdata->TX_NUM);
            p_rx_offset_p = (int32_t *)(sec_testdata->fw->data + item_header->top_limit_offset + 2*4*sec_testdata->TX_NUM);
            p_rx_offset_n = (int32_t *)(sec_testdata->fw->data + item_header->top_limit_offset + 2*4*sec_testdata->TX_NUM + 4*sec_testdata->RX_NUM);
        } else {
            err_cnt++;
            TPD_INFO("item: %d has invalid limit type(%d)\n", TYPE_SELF_RAW_OFFSET_DATA_SDC, item_header->item_limit_type);
            seq_printf(s, "item: %d has invalid limit type(%d)\n", TYPE_SELF_RAW_OFFSET_DATA_SDC, item_header->item_limit_type);
            goto ERR_OUT;
        }
        if (item_header->para_num == 0) {
        } else {
            err_cnt++;
            TPD_INFO("item: %d has %d parameter\n", TYPE_MUTUAL_RAW_OFFSET_DATA_SDC, item_header->para_num);
            seq_printf(s, "item: %d has %d parameter\n", TYPE_MUTUAL_RAW_OFFSET_DATA_SDC, item_header->para_num);
            goto ERR_OUT;
        }

        /* check self offset data */
        type = TYPE_OFFSET_DATA_SDC;
        ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_SELF_RAW_TYPE, type);
        if (ret < 0) {
            err_cnt++;
            TPD_INFO("%s: set self rawdata type failed\n", __func__);
            seq_printf(s, "set self rawdata type failed\n");
            goto ERR_OUT;
        }

        sec_mdelay(100);
        /* read raw data */
        memset(pRead, 0, readbytes); //clear buffer
        ret = touch_i2c_read_block(chip_info->client, SEC_READ_TOUCH_SELF_RAWDATA, readselfbytes, pRead);
        if (ret < 0) {
            err_cnt++;
            TPD_INFO("%s: read self rawdata failed!\n", __func__);
            seq_printf(s, "read self rawdata failed\n");
            goto ERR_OUT;
        }

        /* store and judge self minimum frame data */
        store_to_file(sec_testdata->fd, "TYPE_SELF_TX_OFFSET_DATA:\n");
        /* check long channel of self data  */
        for (i = 0; i < sec_testdata->TX_NUM; i++) {
            nodeData = (pRead[i*2] << 8) | pRead[i*2+1];
            if (sec_testdata->fd >= 0) {
                store_to_file(sec_testdata->fd, "%d, ", nodeData);
            }
            if ((nodeData < p_tx_offset_n[i]) || (nodeData > p_tx_offset_p[i])) {
                TPD_INFO("self_offset_tx_data failed at data[%d] = %d [%d,%d]\n", i, nodeData, p_tx_offset_n[i], p_tx_offset_p[i]);
                if (!err_cnt) {
                    seq_printf(s, "Step 3-1:self_offset_tx_data failed at data[%d] = %d [%d,%d]\n", i, nodeData, p_tx_offset_n[i], p_tx_offset_p[i]);
                }
                err_cnt++;
            }
        }
        store_to_file(sec_testdata->fd, "\n");

        store_to_file(sec_testdata->fd, "TYPE_SELF_RX_OFFSET_DATA:\n");
        /* check short channel of self data */
        for (i = 0; i < sec_testdata->RX_NUM; i++) {
            nodeData = (pRead[sec_testdata->TX_NUM*2 + i*2] << 8) | pRead[sec_testdata->TX_NUM*2 + i*2 +1];  
            if (sec_testdata->fd >= 0) {
                store_to_file(sec_testdata->fd, "%d, ", nodeData);
            }
            if ((nodeData < p_rx_offset_n[i]) || (nodeData > p_rx_offset_p[i])) {
                TPD_INFO("self_offset_rx_data failed at data[%d] = %d [%d,%d]\n", i, nodeData, p_rx_offset_n[i], p_rx_offset_p[i]);
                if (!err_cnt) {
                    seq_printf(s, "Step 3-2:self_offset_rx_data failed at data[%d] = %d [%d,%d]\n", i, nodeData, p_rx_offset_n[i], p_rx_offset_p[i]);
                }
                err_cnt++;
            }
        }
        store_to_file(sec_testdata->fd, "\n");
    }

    ret = sec_execute_p2ptest(s, chip_info, sec_testdata);
    if (ret < 0) {
        err_cnt++;
        TPD_INFO("%s: p2ptest failed\n", __func__);
        seq_printf(s, "p2ptest failed\n");
        goto ERR_OUT;
    }

    //test item mutual raw noise
    if (sec_testdata->test_item & (1 << TYPE_MUTU_RAW_NOI_P2P)) {
        TPD_INFO("do test item raw noise p2p\n");

        item_offset = search_for_item(sec_testdata->fw, item_cnt, TYPE_MUTU_RAW_NOI_P2P);
        if (item_offset == 0) {
            err_cnt++;
            TPD_INFO("search for item limit offset failed\n");
            seq_printf(s, "search for item limit offset failed\n");
            goto ERR_OUT;
        }
        item_header = (struct sec_test_item_header *)(sec_testdata->fw->data + item_offset);
        if (item_header->item_magic != 0x4F50504F) {
            err_cnt++;
            TPD_INFO("test item: %d magic number(%4x) is wrong\n", TYPE_MUTU_RAW_NOI_P2P, item_header->item_magic);
            seq_printf(s, "test item: %d magic number(%4x) is wrong\n", TYPE_MUTU_RAW_NOI_P2P, item_header->item_magic);
            goto ERR_OUT;
        }
        if (item_header->item_limit_type == LIMIT_TYPE_EACH_NODE_DATA) {
            p_p2p_p = (int32_t *)(sec_testdata->fw->data + item_header->top_limit_offset);
            p_p2p_n = (int32_t *)(sec_testdata->fw->data + item_header->floor_limit_offset);
        } else {
            err_cnt++;
            TPD_INFO("item: %d has invalid limit type(%d)\n", TYPE_MUTU_RAW_NOI_P2P, item_header->item_limit_type);
            seq_printf(s, "item: %d has invalid limit type(%d)\n", TYPE_MUTU_RAW_NOI_P2P, item_header->item_limit_type);
            goto ERR_OUT;
        }
        if (item_header->para_num == 0) {
        } else {
            err_cnt++;
            TPD_INFO("item: %d has %d parameter\n", TYPE_MUTU_RAW_NOI_P2P, item_header->para_num);
            seq_printf(s, "item: %d has %d parameter\n", TYPE_MUTU_RAW_NOI_P2P, item_header->para_num);
            goto ERR_OUT;
        }

        /* read minimum value frame from p2p test result */
        type = TYPE_NOI_P2P_MIN;
        ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_MUTU_RAW_TYPE, type);
        if (ret < 0) {
            err_cnt++;
            TPD_INFO("%s: set rawdata type failed\n", __func__);
            seq_printf(s, "set rawdata type failed\n");
            goto ERR_OUT;
        }

        sec_mdelay(100);
        /* read raw data */
        memset(pRead, 0, readbytes); //clear buffer
        ret = touch_i2c_read_block(chip_info->client, SEC_READ_TOUCH_RAWDATA, readbytes, pRead);
        if (ret < 0) {
            err_cnt++;
            TPD_INFO("%s: read rawdata failed!\n", __func__);
            seq_printf(s, "read rawdata failed\n");
            goto ERR_OUT;
        }

        store_to_file(sec_testdata->fd, "TYPE_P2P_MIN_DATA:\n");
        /* check minimum value */
        for (j = 0; j < sec_testdata->RX_NUM; j++) {
            for (i = 0; i < sec_testdata->TX_NUM; i++) {
                iArrayIndex = i * sec_testdata->RX_NUM + j;
                nodeData = (pRead[iArrayIndex*2] << 8) | pRead[iArrayIndex*2+1];
                if (sec_testdata->fd >= 0) {
                    store_to_file(sec_testdata->fd, "%d, ", nodeData);
                }
                if ((nodeData < p_p2p_n[iArrayIndex])) {
                    TPD_INFO("p2p_min_data failed at data[%d][%d] = %d [%d]\n", i, j, nodeData, p_p2p_n[iArrayIndex]);
                    if (!err_cnt) {
                        seq_printf(s, "Step 4-1:p2p_min_data failed at data[%d][%d] = %d [%d]\n", i, j, nodeData, p_p2p_n[iArrayIndex]);
                    }
                    err_cnt++;
                }
            }
            if (sec_testdata->fd >= 0) {
                store_to_file(sec_testdata->fd, "\n");
            }
        }

        /* read maximum value frame from p2p test result */
        type = TYPE_NOI_P2P_MAX;
        ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_MUTU_RAW_TYPE, type);
        if (ret < 0) {
            err_cnt++;
            TPD_INFO("%s: set rawdata type failed\n", __func__);
            seq_printf(s, "set rawdata type failed\n");
            goto ERR_OUT;
        }

        sec_mdelay(100);
        /* read raw data */
        memset(pRead, 0, readbytes); //clear buffer
        ret = touch_i2c_read_block(chip_info->client, SEC_READ_TOUCH_RAWDATA, readbytes, pRead);
        if (ret < 0) {
            err_cnt++;
            TPD_INFO("%s: read rawdata failed!\n", __func__);
            seq_printf(s, "read rawdata failed\n");
            goto ERR_OUT;
        }

        store_to_file(sec_testdata->fd, "TYPE_P2P_MAX_DATA:\n");
        /* check maximum value */
        for (j = 0; j < sec_testdata->RX_NUM; j++) {
            for (i = 0; i < sec_testdata->TX_NUM; i++) {
                iArrayIndex = i * sec_testdata->RX_NUM + j;
                nodeData = (pRead[iArrayIndex*2] << 8) | pRead[iArrayIndex*2+1];
                if (sec_testdata->fd >= 0) {
                    store_to_file(sec_testdata->fd, "%d, ", nodeData);
                }
                if ((nodeData > p_p2p_p[iArrayIndex])) {
                    TPD_INFO("p2p_max_data failed at data[%d][%d] = %d [%d]\n", i, j, nodeData, p_p2p_p[iArrayIndex]);
                    if (!err_cnt) {
                        seq_printf(s, "Step 4-2:p2p_max_data failed at data[%d][%d] = %d [%d]\n", i, j, nodeData, p_p2p_p[iArrayIndex]);
                    }
                    err_cnt++;
                }
            }
            if (sec_testdata->fd >= 0) {
                store_to_file(sec_testdata->fd, "\n");
            }
        }
    }

ERR_OUT:
    if (pRead) {
        kfree(pRead);
        pRead = NULL;
    }

ERR_INT:
    seq_printf(s, "FW:0x%llx\n", sec_testdata->TP_FW);
    seq_printf(s, "%d error(s). %s\n", err_cnt, err_cnt?"":"All test passed.");
    TPD_INFO(" TP auto test %d error(s). %s\n", err_cnt, err_cnt?"":"All test passed.");
}

static int sec_get_verify_result(struct chip_data_s6sy761 *chip_info)
{
    int ret = -1;

    sec_fix_tmode(chip_info, TOUCH_SYSTEM_MODE_TOUCH, TOUCH_MODE_STATE_TOUCH);
    touch_i2c_write_block(chip_info->client, 0xA7, 0, NULL);    //set to verify calibration
    sec_mdelay(100);
    //get calibration result, 0x0F:FAIL(bit[0]:data condition FAIL, bit[1]:RX gap FAIL, bit[2]:TX gap FAIL, bit[3]:TX/RX peak FAIL)
    ret = touch_i2c_read_byte(chip_info->client, 0xA8);
    sec_release_tmode(chip_info);

    return ret;
}


int execute_selftest(struct chip_data_s6sy761 *chip_info, bool save_result)
{
	u8 pStr[50] = {0};
	u8 pTmp[20];
	int rc = 0;
	u8 tpara[2] = {0x25, 0x40};
	u8 *rBuff;
	int i;
	int result_size = SEC_SELFTEST_REPORT_SIZE + chip_info->hw_res->TX_NUM* chip_info->hw_res->RX_NUM * 2;

	/* save selftest result in flash */
	if (save_result)
		tpara[0] = 0x25;
	else
		tpara[0] = 0xA5;

	rBuff = kzalloc(result_size, GFP_KERNEL);
	if (!rBuff)
		return -ENOMEM;

	TPD_INFO("%s: Self test start!\n", __func__);
	rc = touch_i2c_write_block(chip_info->client, SEC_CMD_SELFTEST, 2, tpara);
	if (rc < 0) {
		TPD_INFO("%s: Send selftest cmd failed!\n", __func__);
		goto err_exit;
	}

	sec_mdelay(350);

	rc = sec_wait_for_ready(chip_info, SEC_VENDOR_ACK_SELF_TEST_DONE);
	if (rc < 0) {
		TPD_INFO("%s: Selftest execution time out!\n", __func__);
		goto err_exit;
	}

	TPD_INFO("%s: Self test done!\n", __func__);

	rc = touch_i2c_read_block(chip_info->client, SEC_READ_SELFTEST_RESULT, result_size, rBuff);
	if (rc < 0) {
		TPD_INFO("%s: Selftest execution time out!\n", __func__);
		goto err_exit;
	}

	rearrange_sft_result(rBuff, result_size);

	for (i = 0; i < 80; i += 4) {
		if (i / 4 == 0)
			strncat(pStr, "SIG ", 5);
		else if (i / 4 == 1)
			strncat(pStr, "VER ", 5);
		else if (i / 4 == 2)
			strncat(pStr, "SIZ ", 5);
		else if (i / 4 == 3)
			strncat(pStr, "CRC ", 5);
		else if (i / 4 == 4)
			strncat(pStr, "RES ", 5);
		else if (i / 4 == 5)
			strncat(pStr, "COU ", 5);
		else if (i / 4 == 6)
			strncat(pStr, "PAS ", 5);
		else if (i / 4 == 7)
			strncat(pStr, "FAI ", 5);
		else if (i / 4 == 8)
			strncat(pStr, "CHA ", 5);
		else if (i / 4 == 9)
			strncat(pStr, "AMB ", 5);
		else if (i / 4 == 10)
			strncat(pStr, "RXS ", 5);
		else if (i / 4 == 11)
			strncat(pStr, "TXS ", 5);
		else if (i / 4 == 12)
			strncat(pStr, "RXO ", 5);
		else if (i / 4 == 13)
			strncat(pStr, "TXO ", 5);
		else if (i / 4 == 14)
			strncat(pStr, "RXG ", 5);
		else if (i / 4 == 15)
			strncat(pStr, "TXG ", 5);
		else if (i / 4 == 16)
			strncat(pStr, "RXR ", 5);
		else if (i / 4 == 17)
			strncat(pStr, "TXT ", 5);
		else if (i / 4 == 18)
			strncat(pStr, "RXT ", 5);
		else if (i / 4 == 19)
			strncat(pStr, "TXR ", 5);

		snprintf(pTmp, sizeof(pTmp), "%2X, %2X, %2X, %2X",
			rBuff[i], rBuff[i + 1], rBuff[i + 2], rBuff[i + 3]);
		strncat(pStr, pTmp, strnlen(pTmp, sizeof(pTmp)));

		if (i / 4 == 4) {
			if ((rBuff[i + 3] & 0x30) != 0)// RX, RX open check.
				rc = 0;
			else
				rc = 1;

		}
		if (i % 8 == 4) {
			TPD_INFO("%s\n", pStr);
			memset(pStr, 0x00, sizeof(pStr));
		} else {
			strncat(pStr, "  ", 3);
		}
	}

err_exit:
	kfree(rBuff);
	return rc;
}


static void sec_ts_print_channel(struct chip_data_s6sy761 *chip_info)
{
	unsigned char *pStr = NULL;
	unsigned char pTmp[16] = { 0 };
	int i = 0, j = 0, k = 0;

	if (!chip_info->hw_res->TX_NUM)
		return;

	pStr = vzalloc(7 * (chip_info->hw_res->TX_NUM + 1));
	if (!pStr)
		return;

	memset(pStr, 0x0, 7 * (chip_info->hw_res->TX_NUM + 1));
	snprintf(pTmp, sizeof(pTmp), " TX");
	strncat(pStr, pTmp, 7 * chip_info->hw_res->TX_NUM);

	for (k = 0; k < chip_info->hw_res->TX_NUM; k++) {
		snprintf(pTmp, sizeof(pTmp), "    %02d", k);
		strncat(pStr, pTmp, 7 * chip_info->hw_res->TX_NUM);
	}
	TPD_INFO("%s\n", pStr);

	memset(pStr, 0x0, 7 * (chip_info->hw_res->TX_NUM + 1));
	snprintf(pTmp, sizeof(pTmp), " +");
	strncat(pStr, pTmp, 7 * chip_info->hw_res->TX_NUM);

	for (k = 0; k < chip_info->hw_res->TX_NUM; k++) {
		snprintf(pTmp, sizeof(pTmp), "------");
		strncat(pStr, pTmp, 7 * chip_info->hw_res->TX_NUM);
	}
	TPD_INFO("%s\n", pStr);

	memset(pStr, 0x0, 7 * (chip_info->hw_res->TX_NUM + 1));
	snprintf(pTmp, sizeof(pTmp), " | ");
	strncat(pStr, pTmp, 7 * chip_info->hw_res->TX_NUM);

	for (i = 0; i < (chip_info->hw_res->TX_NUM + chip_info->hw_res->RX_NUM) * 2; i += 2) {
		if (j == chip_info->hw_res->TX_NUM) {
			TPD_INFO("%s\n", pStr);
			TPD_INFO("\n");
			memset(pStr, 0x0, 7 * (chip_info->hw_res->TX_NUM + 1));
			snprintf(pTmp, sizeof(pTmp), " RX");
			strncat(pStr, pTmp, 7 *chip_info->hw_res->TX_NUM);

			for (k = 0; k < chip_info->hw_res->TX_NUM; k++) {
				snprintf(pTmp, sizeof(pTmp), "    %02d", k);
				strncat(pStr, pTmp, 7 * chip_info->hw_res->TX_NUM);
			}

			TPD_INFO("%s\n", pStr);

			memset(pStr, 0x0, 7 * (chip_info->hw_res->TX_NUM + 1));
			snprintf(pTmp, sizeof(pTmp), " +");
			strncat(pStr, pTmp, 7 * chip_info->hw_res->TX_NUM);

			for (k = 0; k < chip_info->hw_res->TX_NUM; k++) {
				snprintf(pTmp, sizeof(pTmp), "------");
				strncat(pStr, pTmp, 7 * chip_info->hw_res->TX_NUM);
			}
			TPD_INFO("%s\n", pStr);

			memset(pStr, 0x0, 7 * (chip_info->hw_res->TX_NUM + 1));
			snprintf(pTmp, sizeof(pTmp), " | ");
			strncat(pStr, pTmp, 7 * chip_info->hw_res->TX_NUM);
		} else if (j && !(j % chip_info->hw_res->TX_NUM)) {
			TPD_INFO("%s\n", pStr);
			memset(pStr, 0x0, 7 * (chip_info->hw_res->TX_NUM + 1));
			snprintf(pTmp, sizeof(pTmp), " | ");
			strncat(pStr, pTmp, 7 * chip_info->hw_res->TX_NUM);
		}

		snprintf(pTmp, sizeof(pTmp), " %5d", chip_info->pFrame[j]);
		strncat(pStr, pTmp, 7 * chip_info->hw_res->TX_NUM);

		j++;
	}
	TPD_INFO("%s\n", pStr);
	vfree(pStr);
}

static int sec_ts_read_channel(struct chip_data_s6sy761 *chip_info, u8 type,
				short *min, short *max, bool save_result)
{
	unsigned char *pRead = NULL;
	u8 mode = TYPE_INVALID_DATA;
	int ret = 0;
	int ii = 0;
	int jj = 0;
	unsigned int data_length = (chip_info->hw_res->TX_NUM + chip_info->hw_res->RX_NUM) * 2;
	u8 w_data;

	TPD_INFO("%s: type %d\n", __func__, type);

	pRead = kzalloc(data_length, GFP_KERNEL);
	if (!pRead)
		return -ENOMEM;

	/* set OPCODE and data type */
	w_data = type;

	ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_SELF_RAW_TYPE, type);
	if (ret < 0) {
		TPD_DEBUG("%s: Set rawdata type failed\n", __func__);
		goto out_read_channel;
	}

	sec_mdelay(50);

	if (type == TYPE_OFFSET_DATA_SDC) {
		/* execute selftest for real cap offset data,
		 * because real cap data is not memory data in normal touch.
		 */
		char para = 0;// 0 for return touch mode
		disable_irq(chip_info->client->irq);
		ret = execute_selftest(chip_info, save_result);
		if (ret < 0) {
			TPD_DEBUG("%s: execute_selftest failed!\n", __func__);
			enable_irq(chip_info->client->irq);
			goto err_read_data;
		}
		ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_SET_POWER_MODE, para);
		if (ret < 0) {
			TPD_DEBUG("%s: set rawdata type failed!\n", __func__);
			enable_irq(chip_info->client->irq);
			goto err_read_data;
		}
		enable_irq(chip_info->client->irq);
		/* end */
	}
	/* read data */
	ret =  touch_i2c_read_block(chip_info->client, SEC_READ_TOUCH_SELF_RAWDATA, data_length, pRead);
	if (ret < 0) {
		TPD_DEBUG("%s: read rawdata failed!\n", __func__);
		goto err_read_data;
	}

	/* clear all pFrame data */
	memset(chip_info->pFrame, 0x00, data_length);

	for (ii = 0; ii < data_length; ii += 2) {
		chip_info->pFrame[jj] = ((pRead[ii] << 8) | pRead[ii + 1]);

		if (ii == 0)
			*min = *max =chip_info->pFrame[jj];

		if(chip_info->pFrame[jj] < *min ) *min = chip_info->pFrame[jj];
		if(chip_info->pFrame[jj] > *max ) *max = chip_info->pFrame[jj];
		jj++;
	}

	sec_ts_print_channel(chip_info);

err_read_data:
	/* release data monitory (unprepare AFE data memory) */
	ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_SELF_RAW_TYPE, mode);
	if (ret < 0)
		TPD_DEBUG("%s: Set rawdata type failed\n", __func__);

out_read_channel:
	kfree(pRead);

	return ret;
}

static void sec_ts_print_frame(struct chip_data_s6sy761 *chip_info, short *min, short *max)
{
	int i = 0;
	int j = 0;
	unsigned char *pStr = NULL;
	unsigned char pTmp[16] = { 0 };
	int lsize = 7 * (chip_info->hw_res->TX_NUM + 1);

	TPD_INFO("%s\n", __func__);

	pStr = kzalloc(lsize * (sizeof(int)), GFP_KERNEL);
	if (pStr == NULL)
		return;

	memset(pStr, 0x0, lsize);
	snprintf(pTmp, sizeof(pTmp), "      TX");
	strncat(pStr, pTmp, lsize);

	for (i = 0; i < chip_info->hw_res->TX_NUM; i++) {
		snprintf(pTmp, sizeof(pTmp), " %02d ", i);
		strncat(pStr, pTmp, lsize);
	}
	if (chip_info->print_num == 1) {
		TPD_INFO("%s\n", pStr);
	} else {
		TPD_DEBUG("%s\n", pStr);
	}
	memset(pStr, 0x0, lsize);
	snprintf(pTmp, sizeof(pTmp), " +");
	strncat(pStr, pTmp, lsize);

	for (i = 0; i < chip_info->hw_res->TX_NUM; i++) {
		snprintf(pTmp, sizeof(pTmp), "----");
		strncat(pStr, pTmp, lsize);
	}
	if (chip_info->print_num == 1) {
		TPD_INFO("%s\n", pStr);
	} else {
		TPD_DEBUG("%s\n", pStr);
	}
	for (i = 0; i < chip_info->hw_res->RX_NUM; i++) {
		memset(pStr, 0x0, lsize);
		snprintf(pTmp, sizeof(pTmp), "Rx%02d | ", i);
		strncat(pStr, pTmp, lsize);

		for (j = 0; j < chip_info->hw_res->TX_NUM; j++) {
			snprintf(pTmp, sizeof(pTmp), " %3d", chip_info->pFrame[(j * chip_info->hw_res->RX_NUM) + i]);

			if (chip_info->pFrame[(j * chip_info->hw_res->RX_NUM) + i] < *min)
				*min = chip_info->pFrame[(j * chip_info->hw_res->RX_NUM) + i];

			if (chip_info->pFrame[(j * chip_info->hw_res->RX_NUM) + i] > *max)
				*max = chip_info->pFrame[(j * chip_info->hw_res->RX_NUM) + i];

			strncat(pStr, pTmp, lsize);
		}
		if (chip_info->print_num == 1) {
			TPD_INFO("%s\n", pStr);
		} else {
			TPD_DEBUG("%s\n", pStr);
		}
	}
	kfree(pStr);
}

static int sec_ts_read_frame(struct chip_data_s6sy761 *chip_info, u8 type, short *min,
		short *max, bool save_result)
{
	unsigned int readbytes = 0xFF;
	unsigned char *pRead = NULL;
	u8 mode = TYPE_INVALID_DATA;
	int ret = 0;
	int i = 0;
	int j = 0;
	short *temp = NULL;

	TPD_INFO("%s: type %d\n", __func__, type);

	/* set data length, allocation buffer memory */
	readbytes = chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM * 2;

	pRead = kzalloc(readbytes, GFP_KERNEL);
	if (!pRead)
		return -ENOMEM;

	/* set OPCODE and data type */
	ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_MUTU_RAW_TYPE, type);
	if (ret < 0) {
		TPD_INFO("%s: Set rawdata type failed\n", __func__);
		goto ErrorExit;
	}

	sec_mdelay(50);

	if (type == TYPE_OFFSET_DATA_SDC) {
		/* excute selftest for real cap offset data, because real cap data is not memory data in normal touch. */
		char para = 0;//0 for return touch mode

		disable_irq(chip_info->client->irq);

		ret = execute_selftest(chip_info, save_result);
		if (ret < 0) {
			TPD_DEBUG("%s: execute_selftest failed\n", __func__);
			enable_irq(chip_info->client->irq);
			goto ErrorRelease;
		}

		ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_SET_POWER_MODE, para);
		if (ret < 0) {
			TPD_DEBUG( "%s: Set power mode failed\n", __func__);
			enable_irq(chip_info->client->irq);
			goto ErrorRelease;
		}

		enable_irq(chip_info->client->irq);
	}

	/* read data */
	ret =  touch_i2c_read_block(chip_info->client, SEC_READ_TOUCH_RAWDATA, readbytes, pRead);
	if (ret < 0) {
		TPD_DEBUG("%s: read rawdata failed!\n", __func__);
		goto ErrorRelease;
	}

	memset(chip_info->pFrame, 0x00, readbytes);

	for (i = 0; i < readbytes; i += 2)
		chip_info->pFrame[i / 2] = pRead[i + 1] + (pRead[i] << 8);

	*min = *max = chip_info->pFrame[0];

#ifdef DEBUG_MSG
	TPD_INFO("%s: 02X%02X%02X readbytes=%d\n", __func__,
			pRead[0], pRead[1], pRead[2], readbytes);
#endif
	sec_ts_print_frame(chip_info, min, max);

	temp = kzalloc(readbytes, GFP_KERNEL);
	if (!temp)
		goto ErrorRelease;

	memcpy(temp, chip_info->pFrame, readbytes);
	memset(chip_info->pFrame, 0x00, readbytes);

	for (i = 0; i < chip_info->hw_res->TX_NUM; i++) {
		for (j = 0; j < chip_info->hw_res->RX_NUM; j++)
			chip_info->pFrame[(j * chip_info->hw_res->TX_NUM) + i] = temp[(i * chip_info->hw_res->RX_NUM) + j];
	}

	kfree(temp);

ErrorRelease:
	/* release data monitory (unprepare AFE data memory) */
	ret = touch_i2c_write_byte(chip_info->client, SEC_CMD_MUTU_RAW_TYPE, mode);
	if (ret < 0)
		TPD_DEBUG("%s: Set rawdata type failed\n", __func__);

ErrorExit:
	kfree(pRead);

	return ret;
}

/*
 * sec_ts_run_rawdata_all : read all raw data
 *
 * when you want to read full raw data (full_read : true)
 * "mutual/self 3, 5, 29, 1, 19" data will be saved in log
 *
 * otherwise, (full_read : false, especially on boot time)
 * only "mutual 3, 5, 29" data will be saved in log
 */
void sec_ts_run_rawdata_all(void *chip_data, bool full_read)
{
	short min = 0, max = 0;
	int ret = 0, i = 0, read_num = 0;
	u8 test_type[5] = {TYPE_AMBIENT_DATA, TYPE_DECODED_DATA,
		TYPE_SIGNAL_DATA, TYPE_OFFSET_DATA_SEC, TYPE_OFFSET_DATA_SDC};
	
	struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;
	chip_info->pFrame = kzalloc(chip_info->hw_res->TX_NUM* chip_info->hw_res->RX_NUM* 2, GFP_KERNEL);
	if (!chip_info->pFrame) {
		TPD_INFO("%s: chip_info->pFrame kzalloc fail\n", __func__);
		return;
	}
	TPD_INFO("%s: start (noise:%d, wet:%d)##\n",
			__func__, chip_info->touch_noise_status, chip_info->wet_mode);

	ret = sec_fix_tmode(chip_info, TOUCH_SYSTEM_MODE_TOUCH, TOUCH_MODE_STATE_TOUCH);
	if (ret < 0) {
		TPD_INFO("%s: failed to fix tmode\n",__func__);
		goto out;
	}

	if (full_read) {
		read_num = 5;
	} else {
		read_num = 3;
		test_type[read_num - 1] = TYPE_OFFSET_DATA_SEC;
	}

	for (i = 0; i < read_num; i++) {
		chip_info->print_num = i;
		ret = sec_ts_read_frame(chip_info, test_type[i], &min, &max, false);
		if (ret < 0) {
			TPD_INFO("%s: mutual %d : error ## ret:%d\n",
					__func__, test_type[i], ret);
			goto out;
		} else {
			TPD_INFO("%s: mutual %d : Max/Min %d,%d ##\n",
					__func__, test_type[i], max, min);
		}
		sec_mdelay(20);

		if (full_read) {
			ret = sec_ts_read_channel(chip_info, test_type[i], &min, &max, false);
			if (ret < 0) {
				TPD_INFO("%s: self %d : error ## ret:%d\n",
						__func__, test_type[i], ret);
				goto out;
			} else {
				TPD_INFO("%s: self %d : Max/Min %d,%d ##\n",
						__func__, test_type[i], max, min);
			}
			sec_mdelay(20);
		}
	}

	sec_release_tmode(chip_info);

out:
	TPD_INFO("%s: done (noise:%d, wet:%d)##\n",
			__func__, chip_info->touch_noise_status, chip_info->wet_mode);

	tp_touch_btnkey_release();

}


static void sec_ts_read_info_work(struct work_struct *work)
{
	struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
			work_read_info.work);

	TPD_INFO("%s\n", __func__);
	mutex_lock(&ts->mutex);
	sec_ts_run_rawdata_all(ts->chip_data, false);
	mutex_unlock(&ts->mutex);
}

static void sec_calibrate(struct seq_file *s, void *chip_data)
{
    int ret = -1;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;

    ret = sec_execute_force_calibration(chip_info);
    if (ret < 0) {
		TPD_INFO("%s calibration failed\n", __func__);
        seq_printf(s, "1 error, calibration failed\n");
    } else {
		TPD_INFO("%s calibration successed\n", __func__);
        seq_printf(s, "0 error, calibration successed\n");
    }

    return;
}

static void sec_verify_calibration(struct seq_file *s, void *chip_data)
{
    int ret = -1;
    struct chip_data_s6sy761 *chip_info = (struct chip_data_s6sy761 *)chip_data;

    ret = sec_get_verify_result(chip_info);
    if (ret != 0) {
		TPD_INFO("%s verify calibration failed\n", __func__);
        seq_printf(s, "1 error, verify calibration result failed(0x%02x)\n", ret);
    } else {
		TPD_INFO("%s verify calibration successed\n", __func__);
        seq_printf(s, "0 error, verify calibration result successed\n");
    }

    return;
}

static void sec_calibration_data(struct seq_file *s, void *chip_data)
{
	schedule_delayed_work(&g_tp->work_read_info, msecs_to_jiffies(10));
}

static struct sec_proc_operations sec_proc_ops = {
    .auto_test          = sec_auto_test,
    .calibrate          = sec_calibrate,
    .verify_calibration = sec_verify_calibration,
    .calibration_data   = sec_calibration_data,
};

/*********** Start of I2C Driver and Implementation of it's callbacks*************************/
static int sec_tp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct chip_data_s6sy761 *chip_info = NULL;
    struct touchpanel_data *ts = NULL;
    int ret = -1;

    TPD_INFO("%s  is called\n", __func__);
	if (tp_register_times > 0) {
		TPD_INFO("TP driver have success loaded %d times, exit\n", tp_register_times);
		return -1;
	}
    /* 1. alloc chip info */
    chip_info = kzalloc(sizeof(struct chip_data_s6sy761), GFP_KERNEL);
    if (chip_info == NULL) {
        TPD_INFO("chip info kzalloc error\n");
        ret = -ENOMEM;
        return ret;
    }
    memset(chip_info, 0, sizeof(*chip_info));

    /* 2. Alloc common ts */
    ts = common_touch_data_alloc();
    if (ts == NULL) {
        TPD_INFO("ts kzalloc error\n");
        goto ts_malloc_failed;
    }
    memset(ts, 0, sizeof(*ts));

    /* 3. bind client and dev for easy operate */
    chip_info->client = client;
    ts->debug_info_ops = &debug_info_proc_ops;
    ts->client = client;
    ts->irq = client->irq;
    i2c_set_clientdata(client, ts);
    ts->dev = &client->dev;
    ts->chip_data = chip_info;
    chip_info->hw_res = &ts->hw_res;
    /* 4. file_operations callbacks binding */
    ts->ts_ops = &sec_ops;
    ts->earsense_ops = &earsense_proc_ops;

    /* 5. register common touch device*/
    ret = register_common_touch_device(ts);
    if (ret < 0) {
        goto err_register_driver;
    }
    //ts->tp_resume_order = LCD_TP_RESUME;
	INIT_DELAYED_WORK(&ts->work_read_info, sec_ts_read_info_work);
    /* 6. create debug interface*/
    sec_raw_device_init(ts);
    sec_create_proc(ts, &sec_proc_ops);
	schedule_delayed_work(&ts->work_read_info, msecs_to_jiffies(50));
    TPD_INFO("%s, probe normal end\n", __func__);
    return 0;

err_register_driver:
    common_touch_data_free(ts);
    ts = NULL;	

ts_malloc_failed:
    kfree(chip_info);
    chip_info = NULL;
    ret = -1;

    TPD_INFO("%s, probe error\n", __func__);
    return ret;
}

static int sec_tp_remove(struct i2c_client *client)
{
    struct touchpanel_data *ts = i2c_get_clientdata(client);

    TPD_INFO("%s is called\n", __func__);

	cancel_delayed_work_sync(&ts->work_read_info);
	flush_delayed_work(&ts->work_read_info);
    kfree(ts);

    return 0;
}

static int sec_i2c_suspend(struct device *dev)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s: is called\n", __func__);
    tp_i2c_suspend(ts);

    return 0;
}

static int sec_i2c_resume(struct device *dev)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s is called\n", __func__);
    tp_i2c_resume(ts);

    return 0;
}

static void sec_tp_shutdown(struct i2c_client *client)
{
    struct touchpanel_data *ts = i2c_get_clientdata(client);
	int ret = 0;

    TPD_INFO("%s is called\n", __func__);
	if (!ts->ts_ops->power_control) {
		ret = -EINVAL;
		TPD_INFO("tp power_control NULL!\n");
		return;
    }
    ret = ts->ts_ops->power_control(ts->chip_data, false);
}


static const struct i2c_device_id tp_id[] =
{
    {TPD_DEVICE, 0},
    {},
};

static struct of_device_id tp_match_table[] =
{
    { .compatible = TPD_DEVICE, },
    { },
};

static const struct dev_pm_ops tp_pm_ops = {
#ifdef CONFIG_FB
    .suspend = sec_i2c_suspend,
    .resume = sec_i2c_resume,
#endif
};

static struct i2c_driver tp_i2c_driver =
{
    .probe = sec_tp_probe,
    .remove = sec_tp_remove,
    .shutdown = sec_tp_shutdown,
    .id_table = tp_id,
    .driver = {
        .name = TPD_DEVICE,
        .owner = THIS_MODULE,
        .of_match_table = tp_match_table,
        .pm = &tp_pm_ops,
    },
};
/******************* End of I2C Driver and It's dev_pm_ops***********************/

/***********************Start of module init and exit****************************/
static int __init tp_driver_init(void)
{
    TPD_INFO("%s is called\n", __func__);
    if (i2c_add_driver(&tp_i2c_driver)!= 0) {
        TPD_INFO("unable to add i2c driver.\n");
        return -1;
    }
    return 0;
}

static void __exit tp_driver_exit(void)
{
    i2c_del_driver(&tp_i2c_driver);
}

module_init(tp_driver_init);
module_exit(tp_driver_exit);
/***********************End of module init and exit*******************************/

MODULE_AUTHOR("Samsung Driver");
MODULE_DESCRIPTION("Samsung Electronics TouchScreen driver");
MODULE_LICENSE("GPL v2");
