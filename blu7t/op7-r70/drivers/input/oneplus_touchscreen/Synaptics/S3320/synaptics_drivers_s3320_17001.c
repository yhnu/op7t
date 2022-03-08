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

#include "synaptics_s3320.h"

static struct chip_data_s3320 *g_chip_info = NULL;

/*******Part0:LOG TAG Declear********************/
#define TPD_DEVICE "synaptics-s3320"
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

/*******Part1: Function Declearation*******/
static int synaptics_power_control(void *chip_data, bool enable);
static int synaptics_get_chip_info(void *chip_data);
static int synaptics_mode_switch(void *chip_data, work_mode mode, bool flag);
static int checkCMD(struct chip_data_s3320 *chip_info);

/*******Part2:Call Back Function implement*******/
static int synaptics_get_touch_points(void *chip_data, struct point_info *points, int max_num)
{
    int ret, i, obj_attention;
    unsigned char fingers_to_process = max_num;
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;
    char buf[8*max_num];

    memset(buf, 0, sizeof(buf));
    obj_attention = touch_i2c_read_word(chip_info->client, chip_info->reg_info.F12_2D_DATA15);
    for (i = 9;  ; i--) {
        if ((obj_attention & 0x03FF) >> i  || i == 0)
            break;
        else
            fingers_to_process--;
    }

    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F12_2D_DATA_BASE, 8*fingers_to_process, buf);
    if (ret < 0) {
        TPD_INFO("touch i2c read block failed\n");
        return -1;
    }
    for (i = 0; i < fingers_to_process; i++) {
        points[i].x = ((buf[i*8 + 2] & 0x0f) << 8) | (buf[i*8 + 1] & 0xff);
        points[i].y = ((buf[i*8 + 4] & 0x0f) << 8) | (buf[i*8 + 3] & 0xff);
        points[i].z = buf[i*8 + 5];
        points[i].width_major = ((buf[i*8 + 6] & 0x0f) + (buf[i*8 + 7] & 0x0f)) / 2;
        points[i].status = buf[i*8];
    }

    return obj_attention;
}

static int synaptics_get_vendor(void *chip_data, struct panel_info *panel_data)
{
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

    chip_info->tp_type = panel_data->tp_type;
    chip_info->p_tp_fw = &panel_data->TP_FW;
    TPD_INFO("chip_info->tp_type = %d, panel_data->test_limit_name = %s, panel_data->fw_name = %s\n", chip_info->tp_type, panel_data->test_limit_name, panel_data->fw_name);
    return 0;
}

static int synaptics_read_F54_base_reg(struct chip_data_s3320 *chip_info)
{
    uint8_t buf[4] = {0};
    int ret = 0;

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x01);    // page 1
    if (ret < 0) {
        TPD_INFO("%s: failed for page select\n", __func__);
        return -1;
    }
    ret = touch_i2c_read_block(chip_info->client, 0xE9, 4, &(buf[0x0]));
    chip_info->reg_info.F54_ANALOG_QUERY_BASE = buf[0];
    chip_info->reg_info.F54_ANALOG_COMMAND_BASE = buf[1];
    chip_info->reg_info.F54_ANALOG_CONTROL_BASE = buf[2];
    chip_info->reg_info.F54_ANALOG_DATA_BASE = buf[3];
    TPD_INFO("F54_QUERY_BASE = %x \n\
            F54_CMD_BASE    = %x \n\
            F54_CTRL_BASE   = %x \n\
            F54_DATA_BASE   = %x \n",
            chip_info->reg_info.F54_ANALOG_QUERY_BASE, chip_info->reg_info.F54_ANALOG_COMMAND_BASE,
            chip_info->reg_info.F54_ANALOG_CONTROL_BASE, chip_info->reg_info.F54_ANALOG_DATA_BASE);

    return ret;
}

static int synaptics_get_chip_info(void *chip_data)
{
    uint8_t buf[4] = {0};
    int ret;
    struct chip_data_s3320    *chip_info = (struct chip_data_s3320 *)chip_data;
    struct synaptics_register *reg_info = &chip_info->reg_info;

    memset(buf, 0, sizeof(buf));
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x0);   // page 0
    if (ret < 0) {
        TPD_INFO("%s: failed for page select\n", __func__);
        return -1;
    }
    ret = touch_i2c_read_block(chip_info->client, 0xDD, 4, &(buf[0x0]));
    if (ret < 0) {
        TPD_INFO("failed for page select!\n");
        return -1;
    }

    reg_info->F12_2D_QUERY_BASE = buf[0];
    reg_info->F12_2D_CMD_BASE = buf[1];
    reg_info->F12_2D_CTRL_BASE = buf[2];
    reg_info->F12_2D_DATA_BASE = buf[3];

    TPD_INFO("F12_2D_QUERY_BASE = 0x%x \n\
            F12_2D_CMD_BASE    = 0x%x \n\
            F12_2D_CTRL_BASE   = 0x%x \n\
            F12_2D_DATA_BASE   = 0x%x \n",
            reg_info->F12_2D_QUERY_BASE, reg_info->F12_2D_CMD_BASE,
            reg_info->F12_2D_CTRL_BASE,  reg_info->F12_2D_DATA_BASE);

    ret = touch_i2c_read_block(chip_info->client, 0xE3, 4, &(buf[0x0]));
    reg_info->F01_RMI_QUERY_BASE = buf[0];
    reg_info->F01_RMI_CMD_BASE = buf[1];
    reg_info->F01_RMI_CTRL_BASE = buf[2];
    reg_info->F01_RMI_DATA_BASE = buf[3];
    TPD_INFO("F01_RMI_QUERY_BASE = 0x%x \n\
            F01_RMI_CMD_BASE    = 0x%x \n\
            F01_RMI_CTRL_BASE   = 0x%x \n\
            F01_RMI_DATA_BASE   = 0x%x \n",
            reg_info->F01_RMI_QUERY_BASE, reg_info->F01_RMI_CMD_BASE,
            reg_info->F01_RMI_CTRL_BASE,  reg_info->F01_RMI_DATA_BASE);

    ret = touch_i2c_read_block(chip_info->client, 0xE9, 4, &(buf[0x0]));
    reg_info->F34_FLASH_QUERY_BASE = buf[0];
    reg_info->F34_FLASH_CMD_BASE = buf[1];
    reg_info->F34_FLASH_CTRL_BASE = buf[2];
    reg_info->F34_FLASH_DATA_BASE = buf[3];
    TPD_INFO("F34_FLASH_QUERY_BASE   = 0x%x \n\
            F34_FLASH_CMD_BASE      = 0x%x \n\
            F34_FLASH_CTRL_BASE     = 0x%x \n\
            F34_FLASH_DATA_BASE     = 0x%x \n",
            reg_info->F34_FLASH_QUERY_BASE, reg_info->F34_FLASH_CMD_BASE,
            reg_info->F34_FLASH_CTRL_BASE,  reg_info->F34_FLASH_DATA_BASE);

    reg_info->F01_RMI_QUERY11 = reg_info->F01_RMI_QUERY_BASE + 0x0b;    // product id
    reg_info->F01_RMI_CTRL00 = reg_info->F01_RMI_CTRL_BASE;
    reg_info->F01_RMI_CTRL01 = reg_info->F01_RMI_CTRL_BASE + 1;
    reg_info->F01_RMI_CTRL02 = reg_info->F01_RMI_CTRL_BASE + 2;
    reg_info->F01_RMI_CMD00  = reg_info->F01_RMI_CMD_BASE;
    reg_info->F01_RMI_DATA01 = reg_info->F01_RMI_DATA_BASE + 1;

    reg_info->F12_2D_CTRL08 = reg_info->F12_2D_CTRL_BASE;               // max XY Coordinate
    reg_info->F12_2D_CTRL23 = reg_info->F12_2D_CTRL_BASE + 9;           //glove enable
    reg_info->F12_2D_CTRL32 = reg_info->F12_2D_CTRL_BASE + 0x10;        //moisture enable
    reg_info->F12_2D_DATA04 = reg_info->F12_2D_DATA_BASE + 1;           //gesture type
    reg_info->F12_2D_DATA15 = reg_info->F12_2D_DATA_BASE + 3;           //object attention
    reg_info->F12_2D_CMD00  = reg_info->F12_2D_CMD_BASE;
    reg_info->F12_2D_CTRL20 = reg_info->F12_2D_CTRL_BASE + 0x07;        //motion suppression
    reg_info->F12_2D_CTRL27 = reg_info->F12_2D_CTRL_BASE + 0x0c;        // wakeup Gesture enable

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x4);
    if (ret < 0) {
        TPD_INFO("%s: failed for page select\n", __func__);
        return -1;
    }
    ret = touch_i2c_read_block(chip_info->client, 0xE9, 4, &(buf[0x0]));
    reg_info->F51_CUSTOM_QUERY_BASE = buf[0];
    reg_info->F51_CUSTOM_CMD_BASE   = buf[1];
    reg_info->F51_CUSTOM_CTRL_BASE  = buf[2];
    reg_info->F51_CUSTOM_DATA_BASE  = buf[3];
    TPD_INFO("F51_CUSTOM_QUERY_BASE  = 0x%x \n\
            F51_CUSTOM_CMD_BASE     = 0x%x \n\
            F51_CUSTOM_CTRL_BASE    = 0x%x \n\
            F51_CUSTOM_DATA_BASE    = 0x%x \n",
            reg_info->F51_CUSTOM_QUERY_BASE, reg_info->F51_CUSTOM_CMD_BASE,
            reg_info->F51_CUSTOM_CTRL_BASE,  reg_info->F51_CUSTOM_DATA_BASE);

    reg_info->F51_CUSTOM_CTRL00 = reg_info->F51_CUSTOM_CTRL_BASE;       //no use
    reg_info->F51_CUSTOM_DATA   = reg_info->F51_CUSTOM_DATA_BASE;       //wakeup gesture data
    reg_info->F51_CUSTOM_CTRL50 = reg_info->F51_CUSTOM_CTRL_BASE + 0x08;//edge function register, realy use F51_CUSTOM_CTRL30 register

    synaptics_read_F54_base_reg(chip_info);

    reg_info->F55_SENSOR_CTRL01 = 0x01;      //rx  number
    reg_info->F55_SENSOR_CTRL02 = 0x02;      //tx  number
    // select page 0
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x00);

    return ret;
}

/**
 * synaptics_get_fw_id -   get device fw id.
 * @chip_info: struct include i2c resource.
 * Return fw version result.
 */
static uint32_t synaptics_get_fw_id(struct chip_data_s3320 *chip_info)
{
    uint8_t buf[4];
    uint32_t current_firmware = 0;

    touch_i2c_write_byte(chip_info->client, 0xff, 0x0);
    touch_i2c_read_block(chip_info->client, chip_info->reg_info.F34_FLASH_CTRL_BASE, 4, buf);
    current_firmware = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    TPD_INFO("CURRENT_FIRMWARE_ID = 0x%x\n", current_firmware);

    return current_firmware;
}

static fw_check_state synaptics_fw_check(void *chip_data, struct resolution_info *resolution_info, struct panel_info *panel_data)
{
    uint32_t bootloader_mode;
    int max_y_ic = 0;
    int max_x_ic = 0;
    uint8_t buf[4];
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

    touch_i2c_write_byte(chip_info->client, 0xff, 0x00);

    touch_i2c_read_block(chip_info->client, chip_info->reg_info.F12_2D_CTRL08, 4, buf);
    max_x_ic = ((buf[1] << 8) & 0xffff) | (buf[0] & 0xffff);
    max_y_ic = ((buf[3] << 8) & 0xffff) | (buf[2] & 0xffff);
    TPD_INFO("max_x = %d, max_y = %d, max_x_ic = %d, max_y_ic = %d\n", resolution_info->max_x, resolution_info->max_y, max_x_ic, max_y_ic);
    if ((resolution_info->max_x == 0) ||(resolution_info->max_y == 0)) {
        resolution_info->max_x = max_x_ic;
        resolution_info->max_y = max_y_ic;
    }

    bootloader_mode = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F01_RMI_DATA_BASE);
    bootloader_mode = bootloader_mode & 0xff;
    bootloader_mode = bootloader_mode & 0x40;
    TPD_INFO("%s, bootloader_mode = 0x%x\n", __func__, bootloader_mode);

    if ((max_x_ic == 0) || (max_y_ic == 0) || (bootloader_mode == 0x40)) {
        TPD_INFO("Something terrible wrong, Trying Update the Firmware again\n");
        return FW_ABNORMAL;
    }

    //fw check normal need update TP_FW  && device info
    panel_data->TP_FW = synaptics_get_fw_id(chip_info);
    if (panel_data->manufacture_info.version)
        sprintf(panel_data->manufacture_info.version, "0x%x", panel_data->TP_FW);

    return FW_NORMAL;
}

/**
 * synaptics_enable_interrupt -   Device interrupt ability control.
 * @chip_info: struct include i2c resource.
 * @enable: disable or enable control purpose.
 * Return  0: succeed, -1: failed.
 */
static int synaptics_enable_interrupt(struct chip_data_s3320 *chip_info, bool enable)
{
    int ret;
    uint8_t abs_status_int;

    TPD_INFO("%s enter, enable = %d.\n", __func__, enable);
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x0);
    if (ret < 0) {
        TPD_INFO("%s: select page failed ret = %d\n", __func__, ret);
        return -1;
    }
    if (enable) {
        abs_status_int = 0x7f;
        /*clear interrupt bits for previous touch*/
        ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F01_RMI_DATA_BASE + 1);
        if (ret < 0) {
            TPD_INFO("%s :clear interrupt bits failed\n", __func__);
            return -1;
        }
    } else {
        abs_status_int = 0x0;
    }

    ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F01_RMI_CTRL00 + 1, abs_status_int);
    if (ret < 0) {
        TPD_INFO("%s: enable or disable abs interrupt failed, abs_int = %d\n", __func__, abs_status_int);
        return -1;
    }

    return 0;
}

static u8 synaptics_trigger_reason(void *chip_data, int gesture_enable, int is_suspended)
{
    int ret = 0;
    uint8_t device_status = 0;
    uint8_t interrupt_status = 0;
    u8 result_event = 0;
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

#ifdef CONFIG_SYNAPTIC_RED
    if (chip_info->enable_remote)
        return IRQ_IGNORE;
#endif
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x0);   // page 0
    ret = touch_i2c_read_word(chip_info->client, chip_info->reg_info.F01_RMI_DATA_BASE);
    if (ret < 0) {
        TPD_INFO("%s, i2c read error, ret = %d\n", __func__, ret);
        return IRQ_EXCEPTION;
    }
    device_status = ret & 0xff;
    interrupt_status = (ret & 0x7f00) >> 8;

    if (device_status) {
        TPD_INFO("%s, IRQ_EXCEPTION ,interrupt_status = 0x%x, device_status = 0x%x\n", __func__, interrupt_status, device_status);
        return IRQ_EXCEPTION;
    }
    if (interrupt_status & 0x04) {
        if ((gesture_enable == 1) && is_suspended) {
            return IRQ_GESTURE;
        }
        SET_BIT(result_event, IRQ_TOUCH);
    }
    if (interrupt_status & 0x10) {
        SET_BIT(result_event, IRQ_BTN_KEY);
    }

    return  result_event;
}

static int synaptics_resetgpio_set(struct hw_resource *hw_res, bool on)
{
    if (gpio_is_valid(hw_res->reset_gpio)) {
        TPD_INFO("Set the reset_gpio \n");
        gpio_direction_output(hw_res->reset_gpio, on);
    }

    return 0;
}

static int synaptics_reset_for_prepare(void *chip_data)
{
    int ret = -1;
    int i2c_error_number = 0;
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

    TPD_INFO("%s.\n", __func__);
    synaptics_resetgpio_set(chip_info->hw_res, true); // reset gpio
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x00);
    while ((ret < 0) && (i2c_error_number < I2C_ERROR_MAX_TIME)) {
        ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x00);
        if (ret < 0) {
            TPD_INFO("%s error, ret = %d i2c_error_number=%d\n", __func__, ret,i2c_error_number);
        }
        i2c_error_number++;
    }
    if (i2c_error_number == I2C_ERROR_MAX_TIME) {
        TPD_INFO("i2c_error_number=%d , pull down tp 3V ,then pull up\n",i2c_error_number);
        ret = tp_powercontrol_2v8(chip_info->hw_res, false);
        msleep(30);
        ret = tp_powercontrol_2v8(chip_info->hw_res, true);
        msleep(80);

        ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x00);
        if (ret < 0) {
            TPD_INFO("%s error, ret = %d\n", __func__, ret);
        }
    }

    ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F01_RMI_CMD_BASE, 0x01);   // reset register
    if (ret < 0) {
        TPD_INFO("%s error, ret = %d\n", __func__, ret);
    }

    return ret;
}

/*
 * return success: 0 ; fail : negative
 */
static int synaptics_reset(void *chip_data)
{
    int ret = -1;
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

    TPD_INFO("%s.\n", __func__);
    clear_view_touchdown_flag(); //clear touch download flag
    synaptics_resetgpio_set(chip_info->hw_res, true); // reset gpio
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x00);
    ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F01_RMI_CMD_BASE, 0x01);   // reset register
    if (ret < 0) {
        TPD_INFO("%s error, ret = %d\n", __func__, ret);
    }
    msleep(RESET_TO_NORMAL_TIME);

    return ret;
}

static int synaptics_configuration_init(struct chip_data_s3320 *chip_info, bool config)
{
    int ret = 0;

    TPD_INFO("%s, configuration init = %d\n", __func__, config);
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x0); // page 0
    if (ret < 0) {
        TPD_INFO("init_panel failed for page select\n");
        return -1;
    }

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F01_RMI_CTRL00);
    if (ret < 0) {
        TPD_INFO("init_panel failed for page select\n");
        return -1;
    }

    //device control: normal operation
    if (config) {//configed  && out of sleep mode
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F01_RMI_CTRL00, (ret & 0xf8) | 0x80);
        if (ret < 0) {
            TPD_INFO("%s failed for mode select\n", __func__);
            return -1;
        }
    } else {//sleep mode
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F01_RMI_CTRL00, (ret & 0xf8) | 0x81);
        if (ret < 0) {
            TPD_INFO("%s failed for mode select\n", __func__);
            return -1;
        }
    }

    return ret;
}

static int synaptics_glove_mode_enable(struct chip_data_s3320 *chip_info, bool enable)
{
    int ret = 0;

    TPD_INFO("%s, enable = %d\n", __func__, enable);
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x00);
    if (ret < 0) {
        TPD_DEBUG("touch_i2c_write_byte failed for mode select\n");
        return ret;
    }

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F12_2D_CTRL23);
    if (enable) {
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F12_2D_CTRL23, ret | 0x20);
    } else {
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F12_2D_CTRL23, ret & 0xdf);
    }

    return ret;
}

static int synaptics_enable_black_gesture(struct chip_data_s3320 *chip_info, bool enable)
{
    int ret;
    unsigned char report_gesture_ctrl_buf[3];
    unsigned char wakeup_gesture_enable_buf;

    TPD_INFO("%s, enable = %d\n", __func__, enable);
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x0);
    if (ret < 0) {
        TPD_INFO("%s: select page failed ret = %d\n", __func__, ret);
        return -1;
    }
    touch_i2c_read_block(chip_info->client, chip_info->reg_info.F12_2D_CTRL20, 3, &(report_gesture_ctrl_buf[0x0]));
    if (enable) {
        report_gesture_ctrl_buf[2] |= 0x02 ;
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F01_RMI_CTRL02, 0x3);         //set doze interval to 30ms
    } else  {
        report_gesture_ctrl_buf[2] &= 0xfd ;
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F01_RMI_CTRL02, 0x1);         //set doze interval to 10ms
    }

    touch_i2c_write_block(chip_info->client, chip_info->reg_info.F12_2D_CTRL20, 3, &(report_gesture_ctrl_buf[0x0]));
    wakeup_gesture_enable_buf = 0xef;   // all kinds of gesture except triangle
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F12_2D_CTRL27, wakeup_gesture_enable_buf);

    return 0;
}

static int synaptics_enable_edge_limit(struct chip_data_s3320 *chip_info, bool enable)
{
    int ret;

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x04);
    if (ret < 0) {
        TPD_INFO("%s: select page failed ret = %d\n", __func__, ret);
        return -1;
    }

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F51_CUSTOM_CTRL50);
    if (enable) {
        ret |= 0x01;
    } else  {
        ret &= 0xfe;
    }
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F51_CUSTOM_CTRL50, ret);

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F51_CUSTOM_CTRL50);
    TPD_INFO("%s %s, write back value = 0x%x\n", enable ? "enable" : "disable", __func__, ret);

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x00);
    if (ret < 0) {
        TPD_INFO("%s: failed for page select\n", __func__);
        return -1;
    }
    return ret;
}

static int synaptics_mode_switch(void *chip_data, work_mode mode, bool flag)
{
    int ret = -1;
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

    switch(mode) {
        case MODE_NORMAL:
            ret = synaptics_configuration_init(chip_info, true);
            if (ret < 0) {
                TPD_INFO("%s: synaptics configuration init failed.\n", __func__);
                return ret;
            }
            ret = synaptics_enable_interrupt(chip_info, true);
            if (ret < 0) {
                TPD_INFO("%s: synaptics enable interrupt failed.\n", __func__);
                return ret;
            }

            break;

        case MODE_SLEEP:
            ret = synaptics_enable_interrupt(chip_info, false);
            if (ret < 0) {
                TPD_INFO("%s: synaptics enable interrupt failed.\n", __func__);
                return ret;
            }

            /*device control: sleep mode*/
            ret = synaptics_configuration_init(chip_info, false) ;
            if (ret < 0) {
                TPD_INFO("%s: synaptics configuration init failed.\n", __func__);
                return ret;
            }

            break;

        case MODE_GESTURE:
            ret = synaptics_enable_black_gesture(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: synaptics enable gesture failed.\n", __func__);
                return ret;
            }

            break;

        case MODE_GLOVE:
            ret = synaptics_glove_mode_enable(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: synaptics enable glove mode failed.\n", __func__);
                return ret;
            }

            break;

        case MODE_EDGE:
            ret = synaptics_enable_edge_limit(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: synaptics enable edg limit failed.\n", __func__);
                return ret;
            }

            break;

        default:
            TPD_INFO("%s: Wrong mode.\n", __func__);
    }

    return ret;
}

static int synaptics_get_gesture_info(void *chip_data, struct gesture_info * gesture)
{
    int ret = 0, i, gesture_sign, regswipe;
    uint8_t gesture_buffer[10];
    uint8_t coordinate_buf[25] = {0};
    uint16_t trspoint = 0;
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x00);
    if (ret < 0) {
        TPD_INFO("failed to transfer the data, ret = %d\n", ret);
        return -1;
    }

    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F12_2D_DATA04, 5, &(gesture_buffer[0]));
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x4);
    regswipe = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F51_CUSTOM_DATA + 0x18);

    TPD_DEBUG("gesture_buffer[0] = 0x%x, regswipe = 0x%x, gesture_buffer[1] = 0x%x, gesture_buffer[4] = 0x%x\n",
            gesture_buffer[0], regswipe, gesture_buffer[1], gesture_buffer[4]);

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x00);
    gesture_sign = gesture_buffer[0];

    //detect the gesture mode
    switch (gesture_sign)
    {
        case DTAP_DETECT:
            gesture->gesture_type = DouTap;
            break;
        case SWIPE_DETECT:
            gesture->gesture_type = (regswipe == 0x41) ? Left2RightSwip   :
                (regswipe == 0x42) ? Right2LeftSwip   :
                (regswipe == 0x44) ? Up2DownSwip      :
                (regswipe == 0x48) ? Down2UpSwip      :
                (regswipe == 0x80) ? DouSwip          :
                UnkownGesture;
            break;
        case CIRCLE_DETECT:
            gesture->gesture_type = Circle;
            break;
        case VEE_DETECT:
            gesture->gesture_type = (gesture_buffer[2] == 0x01) ? DownVee  :
                (gesture_buffer[2] == 0x02) ? UpVee    :
                (gesture_buffer[2] == 0x04) ? RightVee :
                (gesture_buffer[2] == 0x08) ? LeftVee  :
                UnkownGesture;
            break;
        case UNICODE_DETECT:
            gesture->gesture_type = (gesture_buffer[2] == 0x77 && gesture_buffer[3] == 0x00) ? Wgestrue :
                (gesture_buffer[2] == 0x6d && gesture_buffer[3] == 0x00) ? Mgestrue :
                UnkownGesture;
            break;
        default:
            gesture->gesture_type = UnkownGesture;
    }
    TPD_DETAIL("%s, gesture_sign = 0x%x, gesture_type = %d\n", __func__, gesture_sign, gesture->gesture_type);

    if (gesture->gesture_type != UnkownGesture) {
        ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x4);
        ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F51_CUSTOM_DATA,      8, &(coordinate_buf[0]));
        ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F51_CUSTOM_DATA + 8,  8, &(coordinate_buf[8]));
        ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F51_CUSTOM_DATA + 16, 8, &(coordinate_buf[16]));
        ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F51_CUSTOM_DATA + 24, 1, &(coordinate_buf[24]));
        ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x0);
        for (i = 0; i < 23; i += 2) {
            trspoint = coordinate_buf[i]|coordinate_buf[i + 1] << 8;
            TPD_DEBUG("synaptics TP read coordinate_point[%d] = %d\n", i, trspoint);
        }

        TPD_DEBUG("synaptics TP coordinate_buf = 0x%x\n", coordinate_buf[24]);
        gesture->Point_start.x = (coordinate_buf[0] | (coordinate_buf[1] << 8));
        gesture->Point_start.y = (coordinate_buf[2] | (coordinate_buf[3] << 8));
        gesture->Point_end.x   = (coordinate_buf[4] | (coordinate_buf[5] << 8));
        gesture->Point_end.y   = (coordinate_buf[6] | (coordinate_buf[7] << 8));
        gesture->Point_1st.x   = (coordinate_buf[8] | (coordinate_buf[9] << 8));
        gesture->Point_1st.y   = (coordinate_buf[10] | (coordinate_buf[11] << 8));
        gesture->Point_2nd.x   = (coordinate_buf[12] | (coordinate_buf[13] << 8));
        gesture->Point_2nd.y   = (coordinate_buf[14] | (coordinate_buf[15] << 8));
        gesture->Point_3rd.x   = (coordinate_buf[16] | (coordinate_buf[17] << 8));
        gesture->Point_3rd.y   = (coordinate_buf[18] | (coordinate_buf[19] << 8));
        gesture->Point_4th.x   = (coordinate_buf[20] | (coordinate_buf[21] << 8));
        gesture->Point_4th.y   = (coordinate_buf[22] | (coordinate_buf[23] << 8));
        gesture->clockwise     = (coordinate_buf[24] & 0x10) ? 1 :
            (coordinate_buf[24] & 0x20) ? 0 : 2; // 1--clockwise, 0--anticlockwise, not circle, report 2
    }

    return 0;
}

static int synaptics_power_control(void *chip_data, bool enable)
{
    int ret = 0;
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

    if (true == enable) {
        ret = tp_powercontrol_2v8(chip_info->hw_res, true);
        if (ret)
            return -1;
        ret = tp_powercontrol_1v8(chip_info->hw_res, true);
        if (ret)
            return -1;
        synaptics_resetgpio_set(chip_info->hw_res, true);
        msleep(RESET_TO_NORMAL_TIME);
    } else {
        ret = tp_powercontrol_1v8(chip_info->hw_res, false);
        if (ret)
            return -1;
        ret = tp_powercontrol_2v8(chip_info->hw_res, false);
        if (ret)
            return -1;
    }

    return ret;
}

static int checkCMD(struct chip_data_s3320 *chip_info)
{
    int ret;
    int flag_err = 0;

    do {
        msleep(30); //wait 10ms
        ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE);
        flag_err++;
    }while((ret > 0x00) && (flag_err < 30));
    if (ret > 0x00)
        TPD_INFO("checkCMD error ret is %x flag_err is %d\n", ret, flag_err);
    return ret;
}

static bool checkCMD_for_short_test(struct chip_data_s3320 *chip_info)
{
    int ret;
    int flag_err = 0;

    do {
        msleep(30); //wait 10ms
        ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE);
        if ((ret & 0x02) || (ret & 0x04)) {
            break;
        }
        flag_err++;
    } while ((flag_err < 30));
    if ((ret & 0x02) || (ret & 0x04)) {
        return true;
    } else {
        TPD_INFO("checkCMD_for_short_test ret is %x flag_err is %d\n", ret, flag_err);
        return false;
    }
}

static int checkCMD_for_finger(struct chip_data_s3320 *chip_info)
{
    int ret;
    int flag_err = 0;

    if (chip_info->p_spuri_fp_touch == NULL) {
        TPD_INFO("checkCMD_for_finger() chip_info->p_spuri_fp_touch == NULL\n");
        return -1;
    }

    do {
        msleep(10); //wait 10ms
        ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE);
        flag_err++;
        if (chip_info->p_spuri_fp_touch->lcd_trigger_fp_check && !chip_info->p_spuri_fp_touch->lcd_resume_ok) {
            TPD_INFO("checkCMD_for_finger() lcd_resume_ok=%d ret=%d flag_err=%d line=%d\n",chip_info->p_spuri_fp_touch->lcd_resume_ok,ret,flag_err,__LINE__);
            return -1;
        }
    }while((ret > 0x00) && (flag_err < SPURIOUS_FP_BASE_DATA_RETRY));
    TPD_INFO("checkCMD error ret is %x flag_err is %d\n", ret, flag_err);

    if (10 == flag_err) {
        TPD_INFO("checkCMD_for_finger error,reset  then msleep(80)\n");
        if (chip_info->p_spuri_fp_touch->lcd_trigger_fp_check && !chip_info->p_spuri_fp_touch->lcd_resume_ok) {
            TPD_INFO("checkCMD_for_finger() lcd_resume_ok=%d ret=%d flag_err=%d line=%d\n",chip_info->p_spuri_fp_touch->lcd_resume_ok,ret,flag_err,__LINE__);
            return -1;
        }
        synaptics_reset(chip_info);
        msleep(80);
        synaptics_mode_switch(chip_info, MODE_NORMAL, true);
        return -1;
    } else {
        TPD_INFO("checkCMD_for_finger ok\n");
        return 0;
    }
}

static void synaptics_auto_test(struct seq_file *s, void *chip_data, struct syna_testdata *syna_testdata)
{
    uint8_t x, y;
    uint8_t i;
    uint8_t buffer_rx[syna_testdata->RX_NUM];
    uint8_t buffer_tx[syna_testdata->TX_NUM];
    uint8_t tmp_arg1 = 0, tmp_arg2 = 0;
    uint8_t buffer[9];
    uint16_t baseline_data = 0;
    uint16_t *baseline_data_test;
    uint16_t count = 0;
    int ret = 0;
    int error_count = 0;
    int enable_cbc = 1;
    int eint_status, eint_count = 0, read_gpio_num = 0;
    uint8_t data_buf[256];
    struct test_header *ph = NULL;
    uint16_t z = 0;
    uint8_t *raw_data = NULL;
    uint16_t doze_data;

    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;
    struct i2c_client *client = chip_info->client;
    struct synaptics_register *reg_info = &chip_info->reg_info;

    raw_data = kzalloc(syna_testdata->TX_NUM * syna_testdata->RX_NUM * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!raw_data) {
        error_count++;
        snprintf(data_buf, sizeof(data_buf), "raw_data kzalloc error\n");
        TPD_INFO("%s", data_buf);
        goto END;
    }

    //open file to save test data
    ph = (struct test_header *)(syna_testdata->fw->data);

    TPD_INFO("%s, step 0: begin to check INT-GND short item\n", __func__);
    ret = synaptics_enable_interrupt(chip_info, false);

    eint_count = 0;
    read_gpio_num = 10;
    while(read_gpio_num--) {
        msleep(5);
        eint_status = gpio_get_value(syna_testdata->irq_gpio);
        if (eint_status == 1)
            eint_count--;
        else
            eint_count++;
        TPD_INFO("%s eint_count = %d  eint_status = %d\n", __func__, eint_count, eint_status);
    }
    TPD_INFO("TP EINT PIN direct short! eint_count = %d\n", eint_count);
    if (eint_count == 10) {
        TPD_INFO("error :  TP EINT PIN direct short!\n");
        error_count++;
        snprintf(data_buf, sizeof(data_buf), "step 0: begin to check INT-GND short item,"
                            "eint_status is low, TP EINT direct stort\n");
        sys_write(syna_testdata->fd, data_buf, strlen(data_buf));
        TPD_INFO("%s", data_buf);
        goto END;
    }

    synaptics_read_F54_base_reg(chip_info);

    //step 1: Close CBC and test RT20
    TPD_INFO("\n step 1:Close CBC and test RT20\n");
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_DATA_BASE, 0x14);//select report type 0x14
    if (ret < 0) {
        error_count++;
        snprintf(data_buf, sizeof(data_buf), "step 1: Close CBC and test RT20 error,"
                            "[line:%d]touch_i2c_write_byte failed \n", __LINE__);
        TPD_INFO("%s", data_buf);
        goto END;
    }

    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_CONTROL_BASE + 7, 0x01);    //Disable Noise Mitigation, F54_ANALOG_CTRL20
    TPD_DEBUG("forbid Forbid NoiseMitigation oK\n");

    ret = touch_i2c_read_byte(client, reg_info->F54_ANALOG_CONTROL_BASE + 40);        //Read Original CBC settings from F54_ANALOG_CTRL88
    tmp_arg1 = ret&0xff;

    //Close CBC
    TPD_DEBUG("ret = %x, tmp_arg1 = %x, tmp_arg2 = %x\n", ret, tmp_arg1, (tmp_arg1 & 0xdf));
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_CONTROL_BASE + 40, (tmp_arg1 & 0xdf));//Set CBC, F54_ANALOG_CTRL88, close CBC
    ret = touch_i2c_write_word(client, reg_info->F54_ANALOG_COMMAND_BASE, 0x04);//force update
    checkCMD(chip_info);
    TPD_DEBUG("Test disable cbc\n");
    baseline_data_test = (uint16_t *)(syna_testdata->fw->data + ph->array_limit_offset);

    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
    checkCMD(chip_info);
    TPD_DEBUG("Force Cal oK\n");
    ret = touch_i2c_write_word(client, reg_info->F54_ANALOG_DATA_BASE + 1, 0x00);//set fifo 00
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0x01);//get report
    checkCMD(chip_info);

    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE+3, syna_testdata->TX_NUM*syna_testdata->RX_NUM*2, raw_data);     //read data
    for (x = 0; x < syna_testdata->TX_NUM; x++) {
        TPD_DEBUG_NTAG("[%d]: ", x);
        for (y = 0; y < syna_testdata->RX_NUM; y++) {
            z = syna_testdata->RX_NUM * x + y;
            tmp_arg1 = raw_data[z*2] & 0xff;
            tmp_arg2 = raw_data[z*2 + 1] & 0xff;
            baseline_data = (tmp_arg2 << 8) | tmp_arg1;

            if (syna_testdata->fd >= 0) {
                snprintf(data_buf, sizeof(data_buf), "%d, ", baseline_data);
                sys_write(syna_testdata->fd, data_buf, strlen(data_buf));
            }

            if ((x < (syna_testdata->TX_NUM - 1))
             && (y < (syna_testdata->RX_NUM - 2))) {
                TPD_DEBUG_NTAG("%d, ", baseline_data);

                if ((baseline_data < *(baseline_data_test + count*2))
                 || (baseline_data > *(baseline_data_test + count*2 + 1))) {
                    snprintf(data_buf, sizeof(data_buf), "step 1: Close CBC and test RT20 error,"
                                        "baseline_data[%d][%d] = %d[%d, %d]\n",
                                        x, y, baseline_data,
                                        *(baseline_data_test + count*2),
                                        *(baseline_data_test + count*2 + 1));
                    TPD_INFO("%s", data_buf);
                    error_count++;
                    if (tp_debug != 2) {
                        goto END;
                    }
                }
            } else if ((x == (syna_testdata->TX_NUM - 1))
                   && (y > (syna_testdata->RX_NUM - 3))) {
                TPD_DEBUG_NTAG("%d, ", baseline_data);

                if (((baseline_data) < *(baseline_data_test + count*2))
                 || ((baseline_data) > *(baseline_data_test + count*2 + 1))) {
                    snprintf(data_buf, sizeof(data_buf), "step 1: Close CBC and test RT20 error,"
                                        "touch key baseline_data[%d][%d] = %d[%d, %d]\n",
                                        x, y, baseline_data,
                                        *(baseline_data_test + count*2),
                                        *(baseline_data_test + count*2 + 1));
                    TPD_INFO("%s", data_buf);
                    error_count++;
                    if (tp_debug != 2) {
                        goto END;
                    }
                }
            }
            count++;
        }
        if (syna_testdata->fd >= 0) {
                sys_write(syna_testdata->fd, "\n", 1);
        }
        TPD_DEBUG_NTAG("\n");
    }
    if (syna_testdata->fd >= 0) {
            sys_write(syna_testdata->fd, "\n", 1);
    }

    //step 2: Open CBC and test RT3
    TPD_INFO("\n step 2:Open CBC and test RT3\n");
    do{
        ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_DATA_BASE, 0x03);//select report type 0x03
        if (ret < 0) {
            snprintf(data_buf, sizeof(data_buf), "step 2:Open CBC and test RT3 error, "
                                "[line:%d]touch_i2c_write_byte failed \n", __LINE__);
            TPD_INFO("%s", data_buf);
            error_count++;
            goto END;
        }
        ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_CONTROL_BASE + 7, 0x01);//Disable Noise Mitigation, F54_ANALOG_CTRL20
        TPD_DEBUG("forbid Forbid NoiseMitigation oK\n");

        ret = touch_i2c_read_byte(client, reg_info->F54_ANALOG_CONTROL_BASE + 40);        //Read Original CBC settings from F54_ANALOG_CTRL88
        tmp_arg1 = ret&0xff;

        if (enable_cbc) {
            TPD_DEBUG("ret = %x, tmp_arg1 = %x, tmp_arg2 = %x\n", ret, tmp_arg1, (tmp_arg1 | 0x20));
            ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_CONTROL_BASE + 40, (tmp_arg1 | 0x20));//Set CBC, F54_ANALOG_CTRL88
            ret = touch_i2c_write_word(client, reg_info->F54_ANALOG_COMMAND_BASE, 0x04);//force update
            checkCMD(chip_info);
            TPD_DEBUG("Test enable cbc\n");
            baseline_data_test = (uint16_t *)(syna_testdata->fw->data + ph->array_limitcbc_offset);
        } else {
            TPD_DEBUG("ret = %x, tmp_arg1 = %x, tmp_arg2 = %x\n", ret, tmp_arg1, (tmp_arg1 & 0xdf));
            ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_CONTROL_BASE + 40, (tmp_arg1 & 0xdf));//Set CBC, F54_ANALOG_CTRL88
            ret = touch_i2c_write_word(client, reg_info->F54_ANALOG_COMMAND_BASE, 0x04);//force update
            checkCMD(chip_info);
            TPD_DEBUG("Test disable cbc\n");
            baseline_data_test = (uint16_t *)(syna_testdata->fw->data + ph->array_limit_offset);
        }

        ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
        checkCMD(chip_info);
        TPD_DEBUG("Force Cal oK\n");
        ret = touch_i2c_write_word(client, reg_info->F54_ANALOG_DATA_BASE + 1, 0x00);//set fifo 00
        ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0x01);//get report
        checkCMD(chip_info);

        TPD_INFO("RX_NUM is %d, TX_NUM is %d\n", syna_testdata->RX_NUM, syna_testdata->TX_NUM);
        ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE+3, syna_testdata->TX_NUM*syna_testdata->RX_NUM*2, raw_data);     //read data
        count = 0;
        for (x = 0; x < syna_testdata->TX_NUM; x++) {
            TPD_DEBUG_NTAG("[%d]: ", x);
            for (y = 0; y < syna_testdata->RX_NUM; y++) {
                z = syna_testdata->RX_NUM * x + y;
                tmp_arg1 = raw_data[z*2] & 0xff;
                tmp_arg2 = raw_data[z*2 + 1] & 0xff;
                baseline_data = (tmp_arg2 << 8) | tmp_arg1;

                if (syna_testdata->fd >= 0) {
                    snprintf(data_buf, sizeof(data_buf), "%d, ", baseline_data);
                    sys_write(syna_testdata->fd, data_buf, strlen(data_buf));
                }
                TPD_DEBUG("baseline_data is %d\n", baseline_data);
                if ((x < (syna_testdata->TX_NUM - 1)) && (y < (syna_testdata->RX_NUM - 2))) {
                    TPD_DEBUG_NTAG("%d, ", baseline_data);
                    if ((baseline_data < *(baseline_data_test + count*2))
                        || (baseline_data > *(baseline_data_test + count*2 + 1))) {
                        snprintf(data_buf, sizeof(data_buf), "step 2: Open CBC and test RT3 error,"
                                            "baseline_data[%d][%d] = %d[%d, %d]\n",
                                            x, y, baseline_data,
                                            *(baseline_data_test + count*2),
                                            *(baseline_data_test + count*2 + 1));
                        TPD_INFO("%s", data_buf);
                        error_count++;
                        if (tp_debug != 2) {
                            goto END;
                        }
                    }
                } else if ((x == (syna_testdata->TX_NUM - 1)) && (y > (syna_testdata->RX_NUM - 3))) {
                    TPD_DEBUG_NTAG("%d, ", baseline_data);
                    if (((baseline_data) < *(baseline_data_test + count*2))
                        || ((baseline_data) > *(baseline_data_test + count*2 + 1))) {
                        snprintf(data_buf, sizeof(data_buf), "step 2: Open CBC and test RT3 error,"
                                            "touchkey baseline_data[%d][%d] = %d[%d, %d]\n",
                                            x, y, baseline_data,
                                            *(baseline_data_test + count*2),
                                            *(baseline_data_test + count*2 + 1));
                        TPD_INFO("%s", data_buf);
                        error_count++;
                        if (tp_debug != 2) {
                            goto END;
                        }
                    }
                }
                count++;
            }
            if (syna_testdata->fd >= 0) {
                sys_write(syna_testdata->fd, "\n", 1);
            }
            TPD_DEBUG_NTAG("\n");
        }
        if (syna_testdata->fd >= 0) {
            sys_write(syna_testdata->fd, "\n", 1);
        }
        enable_cbc++;
    }while(enable_cbc < 2);

    //step 3 :check tx-to-tx and tx-to-vdd
    TPD_INFO("step 3:check TRx-TRx & TRx-Vdd short\n");
    ret = touch_i2c_write_byte(client, reg_info->F01_RMI_CMD_BASE, 0x01);//software reset TP
    msleep(100);
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_DATA_BASE, 0x1A);//select report type 26
    ret = touch_i2c_write_word(client, reg_info->F54_ANALOG_DATA_BASE + 1, 0x0);//set fifo 00
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0x01);//get report
    //msleep(100);
    checkCMD(chip_info);
    ret = touch_i2c_read_block(client, reg_info->F54_ANALOG_DATA_BASE + 3, 4, buffer);//for s3320 only need to check 32 Rx channels

    //guomingqiang@phone.bsp, 2016-06-27, add for tp test step2
    touch_i2c_write_byte(client, 0xff, 0x3);
    touch_i2c_read_block(client, reg_info->F55_SENSOR_CTRL01, syna_testdata->RX_NUM, buffer_rx);
    touch_i2c_read_block(client, reg_info->F55_SENSOR_CTRL02, syna_testdata->TX_NUM, buffer_tx);

    TPD_DEBUG("RX_NUM : ");
    for (i = 0; i< syna_testdata->RX_NUM; i++)
        printk("%d, ", buffer_rx[i]);
    printk("\n");

    TPD_DEBUG("TX_NUM : ");
    for (i = 0; i< syna_testdata->TX_NUM; i++)
        printk("%d, ", buffer_tx[i]);
    printk("\n");

    for (x = 0; x < 4; x++) {
        if (buffer[x] != 0) {
            error_count++;
            snprintf(data_buf, sizeof(data_buf), "step 3: check tx-to-tx and tx-to-vdd error,"
                                "x = %d, data = %d\n", x, buffer[x]);
            TPD_INFO("%s", data_buf);
            if (syna_testdata->fd >= 0) {
                sys_write(syna_testdata->fd, data_buf, strlen(data_buf));
            }
            goto END;
        }
    }

    synaptics_reset(chip_info);
    msleep(50);

    //Step 4 : Check the broken line with RT133
    TPD_INFO("Step 4 : Check the broken line\n" );
    ret = touch_i2c_write_byte(client, 0xff, 0x01);    // page 1
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_DATA_BASE, 0x85);//select report type 0x85
    ret = touch_i2c_write_word(client, reg_info->F54_ANALOG_DATA_BASE + 1, 0x00);//set fifo 00
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0x01);//get report
    checkCMD_for_short_test(chip_info);

    for (y = 0; y < syna_testdata->RX_NUM; y++) {
        ret = touch_i2c_read_byte(client, reg_info->F54_ANALOG_DATA_BASE + 3);
        tmp_arg1 = ret & 0xff;
        ret = touch_i2c_read_byte(client, reg_info->F54_ANALOG_DATA_BASE + 3);
        tmp_arg2 = ret & 0xff;
        baseline_data = (tmp_arg2 << 8) | tmp_arg1;
        TPD_DEBUG("Step 4 data[%d] is %d\n", y, baseline_data);
        if (baseline_data > 100) {
            error_count++;
            snprintf(data_buf, sizeof(data_buf), "step 4 :Check the broken line with RT133 error,"
                                "error_line is y =%d, data[%d] = %d\n", y, y, baseline_data);
            TPD_INFO("%s", data_buf);
            goto END;
        }
    }

    msleep(20);
    TPD_INFO("Step 4 after read ret is %d\n", ret);

    //step5: check doze mode
    TPD_INFO("step5: check doze mode\n" );
    synaptics_reset(chip_info);
    ret = touch_i2c_write_byte(client, 0xff, 0x1);
    ret |= touch_i2c_write_byte(client, 0x67, 0x01); //enter doze mode
    msleep(100);

    ret |= touch_i2c_write_byte(client, reg_info->F54_ANALOG_DATA_BASE, 0x25);//select report type 37
    ret |= touch_i2c_write_word(client, reg_info->F54_ANALOG_COMMAND_BASE, 0x04); // force update
    checkCMD_for_short_test(chip_info);
    ret |= touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
    checkCMD_for_short_test(chip_info);
    ret |= touch_i2c_write_word(client, reg_info->F54_ANALOG_DATA_BASE + 1, 0x0);//set fifo 0
    ret |= touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0x01);//get report
    checkCMD_for_short_test(chip_info);

    touch_i2c_read_block(client, reg_info->F54_ANALOG_DATA_BASE + 3, (syna_testdata->TX_NUM + syna_testdata->RX_NUM) * 2, raw_data);
    if (syna_testdata->fd >= 0) {
        sys_write(syna_testdata->fd, "\nstep5: check doze mode\n", 1);
    }

    for (x = 0; x < syna_testdata->RX_NUM; x++) {
        doze_data = (raw_data[x * 2 + 1] << 8) | raw_data[x * 2];
        TPD_DEBUG("Doze %d: %d\n", x, doze_data);
        if (syna_testdata->fd >= 0) {
            snprintf(data_buf, sizeof(data_buf), "%d, ", doze_data);
            sys_write(syna_testdata->fd, data_buf, strlen(data_buf));
        }

        if ((doze_data > LIMIT_DOZE) || (doze_data < LIMIT_DOZE_LOW)) {
            error_count++;
            snprintf(data_buf, sizeof(data_buf), "step5: check abs doze error,"
                                "%d: %d not in( %d %d)\n", x, doze_data, LIMIT_DOZE, LIMIT_DOZE_LOW);
            TPD_INFO("%s", data_buf);
            goto END;
        }
    }

END:
    if (raw_data)
        kfree(raw_data);
    seq_printf(s, "imageid = 0x%llx, deviceid = 0x%llx\n", syna_testdata->TP_FW, syna_testdata->TP_FW);
    if (error_count) {
        seq_printf(s, data_buf);
    }
    seq_printf(s, "%d error(s). %s\n", error_count, error_count?"":"All test passed.");
    TPD_INFO(" TP auto test %d error(s). %s\n", error_count, error_count?"":"All test passed.");
    TPD_INFO("\n\nstep5 reset and open irq complete\n");
}

static void synaptics_baseline_read(struct seq_file *s, void *chip_data)
{
    int ret = 0;
    int x = 0, y = 0, z = 0;
    uint16_t baseline_data = 0;
    uint8_t tmp_arg1 = 0;
    uint8_t *raw_data = NULL;
    int enable_cbc = 0;//enable cbc flag for baseline limit test
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

    if (!chip_info)
        return ;
    raw_data = kzalloc(chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!raw_data) {
        TPD_INFO("raw_data kzalloc error\n");
        return;
    }
    synaptics_read_F54_base_reg(chip_info);
    do {
        if (enable_cbc) {
            ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE, 0x03);//select report type 0x03
        } else {
            ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE, 0x14);//select report type 0x14
        }

        if (ret < 0) {
            TPD_INFO("read_baseline: touch_i2c_write_byte failed \n");
            seq_printf(s, "what the hell4, ");
            goto END;
        }
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_CONTROL_BASE + 7, 0x01); //Disable Noise Mitigation, F54_ANALOG_CTRL20
        TPD_DEBUG("forbid Forbid NoiseMitigation oK\n");

        ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_CONTROL_BASE + 40); //Read Original CBC settings from F54_ANALOG_CTRL88
        tmp_arg1 = ret & 0xff;

        if (enable_cbc) {
            seq_printf(s, "\nWith CBC:\n");
            TPD_DEBUG("ret = %x, tmp_arg1 = %x, tmp_arg2 = %x\n", ret, tmp_arg1, (tmp_arg1 | 0x20));
            ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_CONTROL_BASE + 40, (tmp_arg1 | 0x20));//Set CBC, F54_ANALOG_CTRL88
            ret = touch_i2c_write_word(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE, 0x04);// force update
            checkCMD(chip_info);
            TPD_DEBUG("Test enable cbc\n");
        } else {
            seq_printf(s, "\nWithout CBC:\n");
            TPD_DEBUG("ret = %x, tmp_arg1 = %x, tmp_arg2 = %x\n", ret, tmp_arg1, (tmp_arg1 & 0xdf));
            ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_CONTROL_BASE + 40, (tmp_arg1 & 0xdf));//Set CBC, F54_ANALOG_CTRL88
            ret = touch_i2c_write_word(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE, 0x04);// force update
            checkCMD(chip_info);
            TPD_DEBUG("Test disable cbc\n");
        }

        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
        checkCMD(chip_info);
        TPD_DEBUG("Force Cal oK\n");
        ret = touch_i2c_write_word(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE + 1, 0x00);//set fifo 00
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE, 0x01);//get report
        checkCMD(chip_info);
        ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE+3, chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM * 2, raw_data);     //read data

        for (x = 0; x < chip_info->hw_res->TX_NUM; x++) {
            seq_printf(s, "[%2d] ", x);
            for (y = 0; y < chip_info->hw_res->RX_NUM; y++) {
                z = chip_info->hw_res->RX_NUM * x + y;
                baseline_data = (raw_data[z * 2 + 1] << 8) | raw_data[z * 2];
                seq_printf(s, "%4d, ", baseline_data);
            }
            seq_printf(s, "\n");
        }
        enable_cbc++;
    }while(enable_cbc < 2);

    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE, 0);

END:
    kfree(raw_data);
    return ;
}

static void synaptics_delta_read(struct seq_file *s, void *chip_data)
{
    int ret = 0, x = 0, y = 0, z = 0;
    int16_t temp_delta = 0;
    uint8_t *raw_data = NULL;
    int i = 0;
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

    if (!chip_info)
        return ;

    raw_data = kzalloc(chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!raw_data) {
            TPD_INFO("raw_data kzalloc error\n");
            return;
    }

    /*disable irq when read data from IC*/
    synaptics_read_F54_base_reg(chip_info);

    ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE, 0x02);//select report type 0x02
    ret = touch_i2c_write_word(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE + 1, 0x00);//set fifo 00
    ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE, 0x01);//get report

    for (i = 0; i < 2; i++) {
        ret = checkCMD(chip_info);
        if (ret == 0) {
            break;
        }
    }
    if (ret > 0) {
        touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE, 0);
        seq_printf(s, "checkCMD error, can not read delta data\n");
        TPD_INFO("checkCMD error, can not read delta data\n");
        goto OUT;
    }

    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE+3, chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM * 2, raw_data);     //read data
    for (x = 0; x < chip_info->hw_res->TX_NUM; x++) {
        seq_printf(s, "\n[%2d]", x);
        for (y = 0; y < chip_info->hw_res->RX_NUM; y++) {
            z = chip_info->hw_res->RX_NUM * x + y;
            temp_delta = (raw_data[z * 2 + 1] << 8) | raw_data[z * 2];
            seq_printf(s, "%4d, ", temp_delta);
        }
    }
    seq_printf(s, "\n");

OUT:
    touch_i2c_write_byte(chip_info->client, 0xff, 0x00);    // page 0
    msleep(30);
    kfree(raw_data);
}

static void synaptics_main_register_read(struct seq_file *s, void *chip_data)
{
    int ret = 0;
    uint8_t buf[8];
    int i = 0;
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

    if (!chip_info)
        return ;
    /*disable irq when read data from IC*/
    seq_printf(s, "====================================================\n");
    seq_printf(s, "tp fw = 0x%x\n", *(chip_info->p_tp_fw));

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0);

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F01_RMI_DATA_BASE - 1);
    seq_printf(s, "F34_FLASH_DATA03 Flash Status: 0x%x\n", ret);

    ret = touch_i2c_read_word(chip_info->client, chip_info->reg_info.F01_RMI_DATA_BASE);
    if (ret < 0) {
        TPD_INFO("%s, i2c read error, ret = %d\n", __func__, ret);
    }
    seq_printf(s, "F01_RMI_DATA00 Device Status: 0x%x\n", ret & 0xff);
    seq_printf(s, "F01_RMI_DATA01.00 Interrupt Status: 0x%x\n", (ret & 0x7f00) >> 8);

    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F12_2D_DATA_BASE, 8, buf);
    seq_printf(s, "F12_2D_DATA01(00)/00 Object Type and Status 0: \n");
    for (i = 0; i < 8; i++)
        seq_printf(s, "0x%x\n", buf[i]);

    ret = touch_i2c_read_word(chip_info->client, chip_info->reg_info.F01_RMI_CTRL00);
    seq_printf(s, "F01_RMI_CTRL00 Device Control: 0x%x\n", ret);

    synaptics_read_F54_base_reg(chip_info);

    ret = touch_i2c_read_word(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE + 0x04);
    seq_printf(s, "Interference Metric: 0x%x\n", ret);

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE + 0x08);
    seq_printf(s, "Current Noise State: 0x%x\n", ret);

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE + 0x0b);
    seq_printf(s, "Sense Frequency Selection: 0x%x\n", ret);

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x0);
    if (ret < 0) {
        TPD_INFO("%s: failed for page select\n", __func__);
        seq_printf(s, "%s: failed for page select\n", __func__);
        return;
    }

    msleep(10);
}

//Reserved node
static void synaptics_reserve_read(struct seq_file *s, void *chip_data)
{
    int ret = 0;
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

    if (!chip_info)
        return ;

    //1.get firmware doze mode info
    TPD_INFO("1.get firmware doze mode info\n");
    seq_printf(s, "1.get firmware doze mode info\n");
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x04);    // page 4
    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F51_CUSTOM_DATA_BASE + 0x19);
    TPD_INFO("TP doze mode status is %d\n", ret);
    seq_printf(s, "TP doze mode status is %d\n", ret);

    msleep(10);

    /*disable irq when read data from IC*/
    synaptics_read_F54_base_reg(chip_info);

    msleep(10);
}

static void synaptics_abs_doze_read(struct seq_file *s, void *chip_data)
{
    int ret = 0, x = 0;
    int16_t doze_data = 0;
    uint8_t *raw_data = NULL;
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

    if (!chip_info)
        return ;

    raw_data = kzalloc((chip_info->hw_res->TX_NUM + chip_info->hw_res->RX_NUM) * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!raw_data) {
            TPD_INFO("raw_data kzalloc error\n");
            return;
    }

    synaptics_reset(chip_info);
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x1);
    ret |= touch_i2c_write_byte(chip_info->client, 0x67, 0x01); //enter doze mode
    msleep(100);

    ret |= touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE, 0x25);//select report type 37
    ret |= touch_i2c_write_word(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE, 0x04); // force update
    checkCMD_for_short_test(chip_info);
    ret |= touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
    checkCMD_for_short_test(chip_info);
    ret |= touch_i2c_write_word(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE + 1, 0x0);//set fifo 0
    ret |= touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE, 0x01);//get report
    checkCMD_for_short_test(chip_info);

    touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE + 3, (chip_info->hw_res->TX_NUM + chip_info->hw_res->RX_NUM) * 2, raw_data);
    TPD_INFO("abs doze(limite-%d): ",LIMIT_DOZE);
    seq_printf(s, "abs doze(limite-%d): ",LIMIT_DOZE);
    for (x = 0; x < chip_info->hw_res->RX_NUM; x++) {
        doze_data = (raw_data[x * 2 + 1] << 8) | raw_data[x * 2];
        printk("%4d, ", doze_data);
        seq_printf(s, "%4d, ", doze_data);
    }
    TPD_INFO("\n");
    seq_printf(s, "\n");

    touch_i2c_write_byte(chip_info->client, 0xff, 0x00);    // page 0
    msleep(30);
}

static int checkFlashState(struct chip_data_s3320 *chip_info)
{
    int ret ;
    int count = 0;

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.SynaF34_FlashControl + 1);
    while ((ret != 0x80) && (count < 8)) {
        msleep(3); //wait 3ms
        ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.SynaF34_FlashControl + 1);
        count++;
    }

    if (count == 8)
        return 1;
    else
        return 0;
}

/**
 * re_scan_PDT -   rescan && init F34  base addr.
 * @chip_info: struct include i2c resource.
 * Return NULL.
 */
static void re_scan_PDT(struct chip_data_s3320 *chip_info)
{
    uint8_t buf[8];

    touch_i2c_read_block(chip_info->client, 0xE9, 6, buf);
    chip_info->reg_info.F34_FLASH_DATA_BASE = buf[3];
    chip_info->reg_info.F34_FLASH_QUERY_BASE = buf[0];
    touch_i2c_read_block(chip_info->client, 0xE3, 6, buf);
    chip_info->reg_info.F01_RMI_DATA_BASE = buf[3];
    chip_info->reg_info.F01_RMI_CMD_BASE = buf[1];
    touch_i2c_read_block(chip_info->client, 0xDD, 6, buf);

    chip_info->reg_info.SynaF34Reflash_BlockNum    = chip_info->reg_info.F34_FLASH_DATA_BASE;
    chip_info->reg_info.SynaF34Reflash_BlockData   = chip_info->reg_info.F34_FLASH_DATA_BASE + 1;
    chip_info->reg_info.SynaF34ReflashQuery_BootID = chip_info->reg_info.F34_FLASH_QUERY_BASE;
    chip_info->reg_info.SynaF34ReflashQuery_FlashPropertyQuery = chip_info->reg_info.F34_FLASH_QUERY_BASE + 1;
    chip_info->reg_info.SynaF34ReflashQuery_FirmwareBlockSize  = chip_info->reg_info.F34_FLASH_QUERY_BASE + 2;
    chip_info->reg_info.SynaF34ReflashQuery_FirmwareBlockCount = chip_info->reg_info.F34_FLASH_QUERY_BASE + 3;
    chip_info->reg_info.SynaF34ReflashQuery_ConfigBlockSize    = chip_info->reg_info.F34_FLASH_QUERY_BASE + 3;
    chip_info->reg_info.SynaF34ReflashQuery_ConfigBlockCount   = chip_info->reg_info.F34_FLASH_QUERY_BASE + 3;

    touch_i2c_read_block(chip_info->client, chip_info->reg_info.SynaF34ReflashQuery_FirmwareBlockSize, 2, buf);
    TPD_DEBUG("SynaFirmwareBlockSize is %d\n", (buf[0] | (buf[1] << 8)));
    chip_info->reg_info.SynaF34_FlashControl = chip_info->reg_info.F34_FLASH_DATA_BASE + 2;
}

static fw_update_state synaptics_fw_update(void *chip_data, const struct firmware *fw, bool force)
{
    int ret, j;
    uint8_t buf[8];
    uint8_t bootloder_id[10];
    uint16_t block, firmware, configuration;
    uint32_t CURRENT_FIRMWARE_ID = 0, FIRMWARE_ID = 0;
    const uint8_t *Config_Data = NULL;
    const uint8_t *Firmware_Data = NULL;
    struct image_header_data header;
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

    if (!chip_info) {
        TPD_INFO("Chip info is NULL\n");
        return 0;
    }

    TPD_INFO("%s is called\n", __func__);

    //step 1:fill Fw related header, get all data.
    synaptics_parse_header(&header, fw->data);
    if ((header.firmware_size + header.config_size + 0x100) > (uint32_t)fw->size) {
        TPD_DEBUG("firmware_size + config_size + 0x100 > data_len data_len = %d \n", (uint32_t)fw->size);
        return FW_NO_NEED_UPDATE;
    }
    Firmware_Data = fw->data + 0x100;
    Config_Data = Firmware_Data + header.firmware_size;

    //step 2:Get FW version from IC && determine whether we need get into update flow.
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x0);
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F34_FLASH_CTRL_BASE, 4, buf);
    CURRENT_FIRMWARE_ID = (buf[0] << 24)|(buf[1] << 16)|(buf[2] << 8)|buf[3];
    FIRMWARE_ID = (Config_Data[0] << 24)|(Config_Data[1] << 16)|(Config_Data[2] << 8)|Config_Data[3];
    TPD_INFO("CURRENT TP FIRMWARE ID is 0x%x, FIRMWARE IMAGE ID is 0x%x\n", CURRENT_FIRMWARE_ID, FIRMWARE_ID);

    if (!force) {
        if (CURRENT_FIRMWARE_ID == FIRMWARE_ID) {
            return FW_NO_NEED_UPDATE;
        }
    }

    //step 3:init needed params
    re_scan_PDT(chip_info);
    block = 16;
    TPD_DEBUG("block is %d \n", block);
    firmware = header.firmware_size / 16;
    TPD_DEBUG("firmware is %d \n", firmware);
    configuration = header.config_size / 16;
    TPD_DEBUG("configuration is %d \n", configuration);

    //step 3:Get into program mode
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.SynaF34ReflashQuery_BootID, 8, &(bootloder_id[0]));
    TPD_DEBUG("bootloader id is %x \n", (bootloder_id[1] << 8)|bootloder_id[0]);
    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
    TPD_DEBUG("Write bootloader id SynaF34_FlashControl is 0x00%x ret is %d\n", chip_info->reg_info.SynaF34_FlashControl, ret);

    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.SynaF34_FlashControl, 0x0F);
    msleep(10);
    TPD_DEBUG("attn step 4\n");
    ret = checkFlashState(chip_info);
    if (ret > 0) {
        TPD_INFO("Get in prog:The status(Image) of flashstate is %x\n", ret);
        return FW_UPDATE_ERROR;
    }
    ret = touch_i2c_read_byte(chip_info->client, 0x04);
    TPD_DEBUG("The status(device state) is %x\n", ret);
    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F01_RMI_CTRL_BASE);
    TPD_DEBUG("The status(control f01_RMI_CTRL_DATA) is %x\n", ret);
    ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F01_RMI_CTRL_BASE, ret&0x04);

    /********************get into prog end************/
    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
    TPD_DEBUG("ret is %d\n", ret);
    re_scan_PDT(chip_info);
    touch_i2c_read_block(chip_info->client, chip_info->reg_info.SynaF34ReflashQuery_BootID, 2, buf);
    touch_i2c_write_block(chip_info->client, chip_info->reg_info.SynaF34Reflash_BlockData, 2, buf);
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.SynaF34_FlashControl, 0x03);
    msleep(2000);
    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.SynaF34_FlashControl);
    TPD_DEBUG("going to flash firmware area synaF34_FlashControl %d\n", ret);

    //step 4:flash firmware zone
    TPD_INFO("update-----------------firmware ------------------update!\n");
    TPD_DEBUG("cnt %d\n", firmware);
    for (j = 0; j < firmware; j++) {
        buf[0] = j & 0x00ff;
        buf[1] = (j & 0xff00) >> 8;
        touch_i2c_write_block(chip_info->client, chip_info->reg_info.SynaF34Reflash_BlockNum, 2, buf);
        touch_i2c_write_block(chip_info->client, chip_info->reg_info.SynaF34Reflash_BlockData, 16, &Firmware_Data[j*16]);
        touch_i2c_write_byte(chip_info->client, chip_info->reg_info.SynaF34_FlashControl, 0x02);
        ret = checkFlashState(chip_info);
        if (ret > 0) {
            TPD_INFO("Firmware:The status(Image) of flash data3 is %x, time = %d\n", ret, j);
            return FW_UPDATE_ERROR;
        }
    }

    //step 5:flash configure data
    //TPD_INFO("going to flash configuration area\n");
    //TPD_INFO("header.firmware_size is 0x%x\n", header.firmware_size);
    //TPD_INFO("bootloader_size is 0x%x\n", bootloader_size);
    TPD_INFO("update-----------------configuration ------------------update!\n");
    for (j = 0; j < configuration; j++) {
        //a)write SynaF34Reflash_BlockNum to access
        buf[0] = j & 0x00ff;
        buf[1] = (j & 0xff00) >> 8;
        touch_i2c_write_block(chip_info->client, chip_info->reg_info.SynaF34Reflash_BlockNum, 2, buf);
        //b) write data
        touch_i2c_write_block(chip_info->client, chip_info->reg_info.SynaF34Reflash_BlockData, 16, &Config_Data[j*16]);
        //c) issue write
        touch_i2c_write_byte(chip_info->client, chip_info->reg_info.SynaF34_FlashControl, 0x06);
        //d) wait attn
        ret = checkFlashState(chip_info);
        if (ret > 0) {
            TPD_INFO("Configuration:The status(Image) of flash data3 is %x, time = %d\n", ret, j);
            return FW_UPDATE_ERROR;
        }
    }

    TPD_INFO("Firmware && configuration flash over\n");

    return FW_UPDATE_SUCCESS;
}

static fp_touch_state synaptics_spurious_fp_check(void *chip_data)
{
    int x = 0, y = 0, z = 0, err_count = 0;
    int ret = 0, TX_NUM = 0, RX_NUM = 0;
    int16_t temp_data = 0, delta_data = 0;
    uint8_t raw_data[1800] = {0};
    fp_touch_state fp_touch_state = FINGER_PROTECT_TOUCH_UP;

    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;
    TPD_INFO(" synaptics_spurious_fp_check  start\n");

    if(TX_NUM*RX_NUM*(sizeof(int16_t)) > 1800){
        TPD_INFO("%s,TX_NUM*RX_NUM*(sizeof(int16_t)>1800,There is not enough space\n", __func__);
        return FINGER_PROTECT_NOTREADY ;
    }

    if (!chip_info->spuri_fp_data) {
        TPD_INFO("chip_info->spuri_fp_data kzalloc error\n");
        return fp_touch_state;
    }
    TX_NUM = chip_info->hw_res->TX_NUM;
    RX_NUM = chip_info->hw_res->RX_NUM;

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x0);
    if (ret < 0) {
        TPD_INFO("%s,I2C transfer error\n", __func__);
        return fp_touch_state;
    }

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F01_RMI_CTRL00);
    ret = (ret & 0xF8) | 0x80;
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F01_RMI_CTRL00, ret);   //exit sleep
    msleep(1);
    touch_i2c_write_byte(chip_info->client, 0xff, 0x1);
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE, 0x7c);//select report type 124
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE+1, 0x00);//set LSB
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE+2, 0x00);//set MSB
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE, 0x01);//get report

    ret = checkCMD_for_finger(chip_info);
    if (ret < 0) {
        return FINGER_PROTECT_TOUCH_DOWN;
    }

    fp_touch_state = FINGER_PROTECT_TOUCH_UP;
    touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE+3, RX_NUM*SPURIOUS_FP_TX_NUM*2, raw_data); //read baseline data
    for (x = 0; x < SPURIOUS_FP_TX_NUM - 2; x++) {
        TPD_DEBUG_NTAG("[%2d]: ", x);
        for (y = 1; y < (RX_NUM - 3); y++) {
            z = RX_NUM * x + y;
            temp_data = (raw_data[z * 2 + 1] << 8) | raw_data[z * 2];
            delta_data = temp_data - chip_info->spuri_fp_data[z];
            TPD_DEBUG_NTAG("%4d, ", delta_data);
            if ((delta_data + SPURIOUS_FP_LIMIT) < 0) {
                if (!tp_debug)
                    TPD_INFO("delta_data too large, delta_data = %d TX[%d] RX[%d]\n", delta_data, x, y);
                err_count++;
            }
        }
        TPD_DEBUG_NTAG("\n");
        if(err_count > 2) {
            fp_touch_state = FINGER_PROTECT_TOUCH_DOWN;
            err_count = 0;
            if (!tp_debug) {
                TPD_INFO("finger protect trigger!report_finger_protect = %d\n", fp_touch_state);
                break;
            }
        }
    }

    TPD_INFO("%s:%d chip_info->reg_info.F54_ANALOG_COMMAND_BASE=0x%x set 0, \n",__func__,__LINE__,chip_info->reg_info.F54_ANALOG_COMMAND_BASE); //add for Prevent TP failure
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE,0);

    touch_i2c_write_byte(chip_info->client, 0xff, 0x0);
    if (ret < 0) {
        TPD_INFO("%s,I2C transfer error,line=%d\n", __func__,__LINE__);
    }

    TPD_INFO("finger protect trigger fp_touch_state= %d\n", fp_touch_state);

    return fp_touch_state;
}

static u8 synaptics_get_keycode(void *chip_data)
{
    int ret = 0;
    u8 bitmap_result = 0;
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

    touch_i2c_write_byte(chip_info->client, 0xff, 0x02);
    ret = touch_i2c_read_byte(chip_info->client, 0x00);
    TPD_INFO("touch key int_key code = %d\n",ret);

    if (ret & 0x01)
        SET_BIT(bitmap_result, BIT_MENU);
    if (ret & 0x02)
        SET_BIT(bitmap_result, BIT_BACK);

    touch_i2c_write_byte(chip_info->client, 0xff, 0x00);
    return bitmap_result;
}

/**
 * synaptics_resume_prepare -   release inter pins before lcd resume flow(only gesture mode will hold those pins).
 * @chip_info: struct include i2c resource.
 * Return NULL.
 */
static void synaptics_resume_prepare(void *chip_data)
{
    synaptics_reset_for_prepare(chip_data);
    msleep(10);
}

static void synaptics_finger_proctect_data_get(void * chip_data)
{
    int ret = 0, x = 0, y = 0, z = 0;
    uint8_t *raw_data = NULL;
    static uint8_t retry_time = 3;
    struct chip_data_s3320 *chip_info = (struct chip_data_s3320 *)chip_data;

    int TX_NUM = chip_info->hw_res->TX_NUM;
    int RX_NUM = chip_info->hw_res->RX_NUM;

    if(TX_NUM*RX_NUM*(sizeof(int16_t)) > 1800){
        TPD_INFO("%s,TX_NUM*RX_NUM*(sizeof(int16_t)>1800,There is not enough space\n", __func__);
        return;
    }

    raw_data = kzalloc(TX_NUM * SPURIOUS_FP_RX_NUM * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!raw_data) {
            TPD_INFO("raw_data kzalloc error\n");
            return;
    }

    chip_info->spuri_fp_data = kzalloc(TX_NUM*RX_NUM*(sizeof(int16_t)), GFP_KERNEL);
    if (!chip_info->spuri_fp_data) {
        TPD_INFO("chip_info->spuri_fp_data kzalloc error\n");
        ret = -ENOMEM;
    }

RE_TRY:
    TPD_INFO("%s retry_time=%d line=%d\n",__func__,retry_time,__LINE__);
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x1);
    if (ret < 0) {
        TPD_INFO("%s,I2C transfer error\n", __func__);
        return;
    }
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE, 0X02); //forcecal
    ret = checkCMD_for_finger(chip_info);
    if (ret < 0) {
        if (retry_time) {
            TPD_INFO("checkCMD_for_finger error line=%d\n",__LINE__);
            retry_time--;
            goto RE_TRY;
        }
    }

    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE, 0x7c);//select report type 0x02
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE+1, 0x00);//set LSB
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE+2, 0x00);//set MSB
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE, 0x01);//get report

    ret = checkCMD_for_finger(chip_info);
    if (ret < 0) {
        if (retry_time) {
            TPD_INFO("checkCMD_for_finger error line=%d\n",__LINE__);
            retry_time--;
            goto RE_TRY;
        }
    }

    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE+3, RX_NUM*SPURIOUS_FP_TX_NUM*2, raw_data);     //read data
    if (ret < 0) {
        if (retry_time) {
            TPD_INFO("%s touch_i2c_read_block error\n",__func__);
            retry_time--;
            goto RE_TRY;
        }
    }

    for (x = 0; x < SPURIOUS_FP_TX_NUM; x++) {
        printk("[%2d]: ", x);
        for (y = 0; y < RX_NUM; y++) {
            z = RX_NUM * x + y;
            chip_info->spuri_fp_data[z] = (raw_data[z*2 + 1] << 8) | raw_data[z *2];
            printk("%5d,",chip_info->spuri_fp_data[z]);
        }
        printk("\n");
    }

    TPD_INFO("%s F54_ANALOG_COMMAND_BASE=0x%x set 0, \n",__func__,chip_info->reg_info.F54_ANALOG_COMMAND_BASE); //add for Prevent TP failure
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE,0);

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x0);   // page 0
    if (ret < 0) {
        TPD_INFO("%s,I2C transfer error\n", __func__);
    }

    kfree(raw_data);
}

static struct touchpanel_operations synaptics_ops = {
    .get_vendor       = synaptics_get_vendor,
    .get_chip_info    = synaptics_get_chip_info,
    .reset            = synaptics_reset,
    .power_control    = synaptics_power_control,
    .fw_check         = synaptics_fw_check,
    .fw_update        = synaptics_fw_update,
    .trigger_reason   = synaptics_trigger_reason,
    .get_touch_points = synaptics_get_touch_points,
    .get_gesture_info = synaptics_get_gesture_info,
    .mode_switch      = synaptics_mode_switch,
    .get_keycode      = synaptics_get_keycode,
    .resume_prepare   = synaptics_resume_prepare,
    .spurious_fp_check= synaptics_spurious_fp_check,
    .finger_proctect_data_get = synaptics_finger_proctect_data_get,
};

static struct synaptics_proc_operations synaptics_proc_ops = {
    .auto_test     = synaptics_auto_test,
};

static struct debug_info_proc_operations debug_info_proc_ops = {
    .limit_read    = synaptics_limit_read,
    .delta_read    = synaptics_delta_read,
    .baseline_read = synaptics_baseline_read,
    .main_register_read = synaptics_main_register_read,
    .reserve_read = synaptics_reserve_read,
    .abs_doze_read = synaptics_abs_doze_read,
};

static int synaptics_tp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
#ifdef CONFIG_SYNAPTIC_RED
    struct remotepanel_data *premote_data = NULL;
#endif

    struct chip_data_s3320 *chip_info;
    struct touchpanel_data *ts = NULL;
    int ret = -1;

    TPD_INFO("%s  is called\n", __func__);
    //step1:Alloc chip_info
    chip_info = kzalloc(sizeof(struct chip_data_s3320), GFP_KERNEL);
    if (chip_info == NULL) {
        TPD_INFO("chip info kzalloc error\n");
        ret = -ENOMEM;
        return ret;
    }
    memset(chip_info, 0, sizeof(*chip_info));
    g_chip_info = chip_info;

    //step2:Alloc common ts
    ts = common_touch_data_alloc();
    if (ts == NULL) {
        TPD_INFO("ts kzalloc error\n");
        goto ts_malloc_failed;
    }
    memset(ts, 0, sizeof(*ts));

    //step3:binding client && dev for easy operate
    chip_info->client = client;
    chip_info->p_spuri_fp_touch = &(ts->spuri_fp_touch);
    chip_info->syna_ops = &synaptics_proc_ops;
    ts->debug_info_ops = &debug_info_proc_ops;
    ts->client = client;
    ts->irq = client->irq;
    i2c_set_clientdata(client, ts);
    ts->dev = &client->dev;
    ts->chip_data = chip_info;
    chip_info->hw_res = &ts->hw_res;

    //step4:file_operations callback binding
    ts->ts_ops = &synaptics_ops;

    //step5:register common touch
    ret = register_common_touch_device(ts);
    if (ret < 0) {
        goto err_register_driver;
    }

    //step6: collect data for supurious_fp_touch
    if (ts->spurious_fp_support) {
        mutex_lock(&ts->mutex);
        synaptics_finger_proctect_data_get(chip_info);
        mutex_unlock(&ts->mutex);
    }

    //step7:create synaptics related proc files
    synaptics_create_proc(ts, chip_info->syna_ops);

    //step8:Chip Related function
#ifdef CONFIG_SYNAPTIC_RED
    premote_data = remote_alloc_panel_data();
    chip_info->premote_data = premote_data;
    if (premote_data) {
        premote_data->client        = client;
        premote_data->input_dev     = ts->input_dev;
        premote_data->pmutex        = &ts->mutex;
        premote_data->irq_gpio      = ts->hw_res.irq_gpio;
        premote_data->irq           = client->irq;
        premote_data->enable_remote = &(chip_info->enable_remote);
        register_remote_device(premote_data);
    }
#endif

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

static int synaptics_tp_remove(struct i2c_client *client)
{
    struct touchpanel_data *ts = i2c_get_clientdata(client);

    TPD_INFO("%s is called\n", __func__);
#ifdef CONFIG_SYNAPTIC_RED
    unregister_remote_device();
#endif
    kfree(ts);

    return 0;
}

static int synaptics_i2c_suspend(struct device *dev)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s: is called\n", __func__);
    tp_i2c_suspend(ts);

    return 0;
}

static int synaptics_i2c_resume(struct device *dev)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s is called\n", __func__);
    tp_i2c_resume(ts);

    return 0;
}

static const struct i2c_device_id tp_id[] = {
    { TPD_DEVICE, 0 },
    { }
};

static struct of_device_id tp_match_table[] = {
    { .compatible = TPD_DEVICE,},
    { .compatible = "synaptics-s3320",},
    { }
};

static const struct dev_pm_ops tp_pm_ops = {
#ifdef CONFIG_FB
    .suspend = synaptics_i2c_suspend,
    .resume = synaptics_i2c_resume,
#endif
};

static struct i2c_driver tp_i2c_driver = {
    .probe      = synaptics_tp_probe,
    .remove     = synaptics_tp_remove,
    .id_table   = tp_id,
    .driver     = {
        .name   = TPD_DEVICE,
        .of_match_table =  tp_match_table,
        .pm = &tp_pm_ops,
    },
};

static int __init tp_driver_init(void)
{
    TPD_INFO("%s is called\n", __func__);

    if (!tp_judge_ic_match(TPD_DEVICE))
        return -1;

    if (i2c_add_driver(&tp_i2c_driver)!= 0) {
        TPD_INFO("unable to add i2c driver.\n");
        return -1;
    }
    return 0;
}

/* should never be called */
static void __exit tp_driver_exit(void)
{
    i2c_del_driver(&tp_i2c_driver);
    return;
}

module_init(tp_driver_init);
module_exit(tp_driver_exit);

MODULE_DESCRIPTION("Touchscreen Driver");
MODULE_LICENSE("GPL");
