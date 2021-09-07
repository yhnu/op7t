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

#include "synaptics_s4322.h"

static struct chip_data_s4322 *g_chip_info = NULL;

#define OP17001TRULY_TD4322_1080P_CMD_PANEL 29

/*******Part0:LOG TAG Declear********************/
#define TPD_DEVICE "synaptics-s4322"
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
static int checkCMD(struct chip_data_s4322 *chip_info);

/*******Part2:Call Back Function implement*******/
static int synaptics_get_touch_points(void *chip_data, struct point_info *points, int max_num)
{
    int ret, i, obj_attention;
    unsigned char fingers_to_process = max_num;
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;
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
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;

    chip_info->tp_type = panel_data->tp_type;
    chip_info->p_tp_fw = &panel_data->TP_FW;
    TPD_INFO("chip_info->tp_type = %d, panel_data->test_limit_name = %s, panel_data->fw_name = %s\n", chip_info->tp_type, panel_data->test_limit_name, panel_data->fw_name);
    return 0;
}

static int synaptics_read_F54_base_reg(struct chip_data_s4322 *chip_info)
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
    struct chip_data_s4322    *chip_info = (struct chip_data_s4322 *)chip_data;
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
    reg_info->F12_2D_CTRL23 = reg_info->F12_2D_CTRL_BASE + 8;           //glove enable

    reg_info->F12_2D_DATA04 = reg_info->F12_2D_DATA_BASE + 1;           //gesture type
    reg_info->F12_2D_DATA15 = reg_info->F12_2D_DATA_BASE + 2;           //object attention
    reg_info->F12_2D_CMD00  = reg_info->F12_2D_CMD_BASE;
    reg_info->F12_2D_CTRL20 = reg_info->F12_2D_CTRL_BASE + 0x06;        //motion suppression
    reg_info->F12_2D_CTRL27 = reg_info->F12_2D_CTRL_BASE + 0x0b;        // wakeup Gesture enable

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
    reg_info->F51_CUSTOM_GRIP_CTRL = reg_info->F51_CUSTOM_CTRL_BASE + 0x1D;//edge function register,the enable bit of Grip suppression
    reg_info->F51_CUSTOM_CTRL48 = reg_info->F51_CUSTOM_CTRL_BASE + 0x2B;

    synaptics_read_F54_base_reg(chip_info);
    reg_info->F54_ANALOG_CTRL91 = reg_info->F54_ANALOG_CONTROL_BASE + 0x28;
    reg_info->F54_ANALOG_CTRL99 = reg_info->F54_ANALOG_CONTROL_BASE + 0x2d;
    reg_info->F54_ANALOG_CTRL182 = reg_info->F54_ANALOG_CONTROL_BASE + 0x32;
    reg_info->F54_ANALOG_CTRL225 = reg_info->F54_ANALOG_CONTROL_BASE + 0x37;

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x3);
    if (ret < 0) {
        TPD_INFO("%s: failed for page select\n", __func__);
        return -1;
    }
    ret = touch_i2c_read_block(chip_info->client, 0xE9, 4, &(buf[0x0]));
    reg_info->F55_SENSOR_QUERY_BASE = buf[0];
    reg_info->F55_SENSOR_CMD_BASE   = buf[1];
    reg_info->F55_SENSOR_CTRL_BASE  = buf[2];
    reg_info->F55_SENSOR_DATA_BASE  = buf[3];
    TPD_INFO("F55_SENSOR_QUERY_BASE  = 0x%x \n\
            F55_SENSOR_CMD_BASE     = 0x%x \n\
            F55_SENSOR_CTRL_BASE    = 0x%x \n\
            F55_SENSOR_DATA_BASE    = 0x%x \n\
            edge function register: F51_CUSTOM_GRIP_CTRL = 0x%x ",
            reg_info->F55_SENSOR_QUERY_BASE, reg_info->F55_SENSOR_CMD_BASE,
            reg_info->F55_SENSOR_CTRL_BASE,  reg_info->F55_SENSOR_DATA_BASE,
            reg_info->F51_CUSTOM_GRIP_CTRL);

    reg_info->F55_SENSOR_CTRL43 = reg_info->F55_SENSOR_CTRL_BASE + 5;
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
static uint32_t synaptics_get_fw_id(struct chip_data_s4322 *chip_info)
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
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;

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
    TPD_INFO("chip_info->reg_info.F01_RMI_DATA_BASE =  0x%x , bootloader_mode = 0x%x\n", chip_info->reg_info.F01_RMI_DATA_BASE, bootloader_mode);
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
static int synaptics_enable_interrupt(struct chip_data_s4322 *chip_info, bool enable)
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
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;

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
    } else {
        TPD_DEBUG("%s, interrupt_status = 0x%x, device_status = 0x%x\n", __func__, interrupt_status, device_status);
    }
    if (interrupt_status & 0x04) {
        if ((gesture_enable == 1) && is_suspended) {
            return IRQ_GESTURE;
        }
        SET_BIT(result_event, IRQ_TOUCH);
    }
    if (interrupt_status & 0x20) {
        SET_BIT(result_event, IRQ_BTN_KEY);
    }
    if (interrupt_status & 0x02) {
        SET_BIT(result_event, IRQ_FW_AUTO_RESET);
        TPD_INFO("%s, tp fw auto reset detect, interrupt_status = 0x%x, device_status = 0x%x\n", __func__, interrupt_status, device_status);
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

static void synaptics_exit_esd_mode(void *chip_data)
{
    int ret = 0;
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x4);
    if (ret < 0) {
        TPD_INFO("%s: failed for page select\n", __func__);
        return ;
    }

    ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F51_CUSTOM_CMD_BASE, 0x2);
    if (ret < 0) {
        TPD_INFO("exit esd mode error\n");
        return;
    }
    TPD_INFO("exit esd mode ok\n");

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x0);
    if (ret < 0) {
        TPD_INFO("%s: failed for page select\n", __func__);
    }
}

/*
 * return success: 0 ; fail : negative
 */
static int synaptics_reset(void *chip_data)
{
    int ret = -1;
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;

    TPD_INFO("%s.\n", __func__);
    clear_view_touchdown_flag(); //clear touch download flag
    synaptics_resetgpio_set(chip_info->hw_res, true); // reset gpio
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x00);
    ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F01_RMI_CMD_BASE, 0x01);   // reset register
    if (ret < 0) {
        TPD_INFO("%s error, ret = %d\n", __func__, ret);
    }
    msleep(RESET_TO_NORMAL_TIME);

    //exit esd mode
    synaptics_exit_esd_mode(chip_data);

    return ret;
}

static int synaptics_configuration_init(struct chip_data_s4322 *chip_info, bool config)
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

static int synaptics_glove_mode_enable(struct chip_data_s4322 *chip_info, bool enable)
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

static int synaptics_enable_black_gesture(struct chip_data_s4322 *chip_info, bool enable)
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

    if (enable) {
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F01_RMI_CTRL02, 0x3);         //set doze interval to 30ms
    } else  {
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F01_RMI_CTRL02, 0x1);         //set doze interval to 10ms
    }

    touch_i2c_read_block(chip_info->client, chip_info->reg_info.F12_2D_CTRL20, 3, &(report_gesture_ctrl_buf[0x0]));
    if (report_gesture_ctrl_buf[2] & 0x02) {
        TPD_INFO("Black gesture already open,can not write black gesture regiseter in suspend\n");
        return 0;
    }
    if (enable) {
        report_gesture_ctrl_buf[2] |= 0x02 ;
    } else  {
        report_gesture_ctrl_buf[2] &= 0xfd ;
    }

    touch_i2c_write_block(chip_info->client, chip_info->reg_info.F12_2D_CTRL20, 3, &(report_gesture_ctrl_buf[0x0]));
    wakeup_gesture_enable_buf = 0xef;   // all kinds of gesture except triangle
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F12_2D_CTRL27, wakeup_gesture_enable_buf);

    return 0;
}

static int synaptics_enable_edge_limit(struct chip_data_s4322 *chip_info, bool enable)
{
    int ret;

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x04);
    if (ret < 0) {
        TPD_INFO("%s: select page failed ret = %d\n", __func__, ret);
        return -1;
    }

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F51_CUSTOM_GRIP_CTRL);
    if (enable) {
        ret |= 0x01;
    } else  {
        ret &= 0xfe;
    }
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F51_CUSTOM_GRIP_CTRL, ret);

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F51_CUSTOM_GRIP_CTRL);
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
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;

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
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;

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
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;

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

static int checkCMD(struct chip_data_s4322 *chip_info)
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

static int checkCMD_for_finger(struct chip_data_s4322 *chip_info)
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

    if (SPURIOUS_FP_BASE_DATA_RETRY == flag_err) {
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

static short find_median(short* pdata, int num)
{
    int i,j;
    short temp;
    short *value;
    short median;

    value = (short *)kzalloc( num * sizeof(short), GFP_KERNEL);

    for (i = 0; i < num; i++)
        *(value+i) = *(pdata+i);

    //sorting
    for ( i = 1; i <= num-1; i++)
    {
        for ( j = 1; j <= num-i; j++)
        {
            if (*(value + j - 1) <= *(value + j))
            {
               temp = *(value + j - 1);
               *(value + j - 1)= *(value + j);
               *(value + j) = temp;
            }
            else
                continue ;
        }
    }

    //calculation of median
    if ( num % 2 == 0)
        median = (*(value + (num/2 - 1)) + *(value + (num / 2))) / 2;
    else
        median = *(value + (num/2));

    if(value)
        kfree(value);

    return median;
}

static int tddi_ratio_calculation(signed short *p_image,  struct f55_control_43 *p_ctrl_43, int tx, int rx)
{
    int retval = 0;
    int i, j;
    int tx_num = tx;
    int rx_num = rx;
    unsigned char left_size = p_ctrl_43->afe_l_mux_size;
    unsigned char right_size = p_ctrl_43->afe_r_mux_size;
    signed short *p_data_16;
    signed short *p_left_median = NULL;
    signed short *p_right_median = NULL;
    signed short *p_left_column_buf = NULL;
    signed short *p_right_column_buf = NULL;
    signed int temp;

    if (!p_image) {
        TPD_INFO("%s: Fail. p_image is null\n", __func__);
        retval = -EINVAL;
        goto exit;
    }

    // allocate the buffer for the median value in left/right half
    p_right_median = (signed short *) kzalloc(rx_num * sizeof(short), GFP_KERNEL);
    if (!p_right_median) {
        TPD_INFO("%s: Failed to alloc mem for p_right_median\n", __func__);
        retval = -ENOMEM;
        goto exit;
    }

    p_left_median = (signed short *) kzalloc(rx_num * sizeof(short), GFP_KERNEL);
    if (!p_left_median) {
        TPD_INFO("%s: Failed to alloc mem for p_left_median\n", __func__);
        retval = -ENOMEM;
        goto exit;
    }

    p_right_column_buf = (signed short *) kzalloc(right_size * rx_num * sizeof(short), GFP_KERNEL);
    if (!p_right_column_buf ) {
        TPD_INFO("%s: Failed to alloc mem for p_right_column_buf\n", __func__);
        retval = -ENOMEM;
        goto exit;
    }

    p_left_column_buf = (signed short *) kzalloc(left_size * rx_num * sizeof(short), GFP_KERNEL);
    if (!p_left_column_buf ) {
        TPD_INFO("%s: Failed to alloc mem for p_left_column_buf\n", __func__);
        retval = -ENOMEM;
        goto exit;
    }

    // divide the input image into left/right parts
    if (p_ctrl_43->swap_sensor_side) {

        // first row is left side data
        p_data_16 = p_image;
        for (i = 0; i < rx_num; i++) {
            for (j = 0; j < left_size; j++) {
                p_left_column_buf[i * left_size + j] = p_data_16[j * rx_num + i];
            }
        }
        // right side data
        p_data_16 = p_image + left_size * rx_num;
        for (i = 0; i < rx_num; i++) {
            for (j = 0; j < right_size; j++) {
                p_right_column_buf[i * right_size + j] = p_data_16[j * rx_num + i];
            }
        }
    }
    else {

        // first row is right side data
        p_data_16 = p_image;
        for (i = 0; i < rx_num; i++) {
            for (j = 0; j < right_size; j++) {
                p_right_column_buf[i * right_size + j] = p_data_16[j * rx_num + i];
            }
        }
        // left side data
        p_data_16 = p_image + right_size * rx_num;
        for (i = 0; i < rx_num; i++) {
            for (j = 0; j < left_size; j++) {
                p_left_column_buf[i * left_size + j] = p_data_16[j * rx_num + i];
            }
        }
    }

    // find the median in every column
    for (i = 0; i < rx_num; i++) {
        p_left_median[i] = find_median(p_left_column_buf + i * left_size, left_size);
        p_right_median[i] = find_median(p_right_column_buf + i * right_size, right_size);
    }

    // walk through the image of all data
    // and calculate the ratio by using the median
    for (i = 0; i < tx_num; i++) {
        for (j = 0; j < rx_num; j++) {

            // calcueate the ratio
            if (p_ctrl_43->swap_sensor_side) {
                // first row is left side
                if (i < left_size) {
                    temp = (signed int) p_image[i * rx_num + j];
                    temp = temp * 100 / p_left_median[j];
                } else {
                    temp = (signed int) p_image[i * rx_num + j];
                    temp = temp * 100 / p_right_median[j];
                }
            }
            else {
                // first row is right side
                if (i < right_size) {
                    temp = (signed int) p_image[i * rx_num + j];
                    temp = temp * 100 / p_right_median[j];
                } else {
                    temp = (signed int) p_image[i * rx_num + j];
                    temp = temp * 100 / p_left_median[j];
                }
            }

            // replace the original data with the calculated ratio
            p_image[i * rx_num + j] = temp;
        }
    }

exit:
    kfree(p_right_median);
    kfree(p_left_median);
    kfree(p_right_column_buf);
    kfree(p_left_column_buf);
    return retval;
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

static int synaptics_int_pin_test(struct seq_file *s, void *chip_data, struct syna_testdata *syna_testdata)
{
    int eint_status, eint_count = 0, read_gpio_num = 10;

    TPD_INFO("%s, step 0: begin to check INT-GND short item\n", __func__);
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
        seq_printf(s, "TP EINT direct stort\n");
        store_to_file(syna_testdata->fd, "eint_status is low, TP EINT direct stort, \n");
        eint_count = 0;
        return TEST_FAIL;
    }

    return TEST_PASS;
}

static int synaptics_tddi_RT92_test(struct seq_file *s, void *chip_data, struct syna_testdata *syna_testdata)
{
    int ret = 0;
    uint8_t *raw_data = NULL;
    uint16_t *baseline_data_test = NULL;
    uint16_t baseline_data;
    uint8_t x, y;
    uint16_t z;
    uint8_t tmp_arg1;
    uint8_t tmp_arg2;
    int error_count = 0;
    int count = 0;
    int tx_2d_num = syna_testdata->TX_NUM - 1;

    struct test_header *ph = (struct test_header *)(syna_testdata->fw->data);
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;
    struct synaptics_register *reg_info = &chip_info->reg_info;
    struct i2c_client *client = chip_info->client;

    store_to_file(syna_testdata->fd, "RT92:\n");

    TPD_INFO("\n step 1:Close CBC and test RT92\n");

    synaptics_read_F54_base_reg(chip_info);

    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_DATA_BASE, 92);//select report type 92
    if (ret < 0) {
        TPD_INFO("[line:%d]read_baseline: touch_i2c_write_byte failed \n",__LINE__);
        seq_printf(s, "[line:%d]read_baseline: touch_i2c_write_byte failed \n",__LINE__);
        store_to_file(syna_testdata->fd, "I2C read error\n");
        return TEST_FAIL;
    }

    baseline_data_test = (uint16_t *)(syna_testdata->fw->data + ph->array_limit_offset);
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
    checkCMD(chip_info);
    TPD_DEBUG("Force Cal oK\n");
    ret = touch_i2c_write_word(client, reg_info->F54_ANALOG_DATA_BASE + 1, 0x00);//set fifo 00
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0x01);//get report
    checkCMD(chip_info);

    raw_data = kzalloc(syna_testdata->TX_NUM * syna_testdata->RX_NUM * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!raw_data) {
        TPD_INFO("raw_data kzalloc error\n");
        seq_printf(s, "RT92: failed to alloc memory\n");
        store_to_file(syna_testdata->fd, "failed to alloc memory\n");
        return TEST_FAIL;
    }

    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE+3,
                            syna_testdata->TX_NUM*syna_testdata->RX_NUM*2, raw_data);     //read data
    for (x = 0; x < syna_testdata->TX_NUM; x++) {
        TPD_DEBUG_NTAG("[%d]: ", x);
        for (y = 0; y < syna_testdata->RX_NUM; y++) {
            z = syna_testdata->RX_NUM * x + y;
            tmp_arg1 = raw_data[z*2] & 0xff;
            tmp_arg2 = raw_data[z*2 + 1] & 0xff;
            baseline_data = (tmp_arg2 << 8) | tmp_arg1;

            if (syna_testdata->fd >= 0) {
                store_to_file(syna_testdata->fd, "%d, ", baseline_data);
            }

            if ((x < tx_2d_num) && (y < syna_testdata->RX_NUM)) {
                TPD_DEBUG_NTAG("%d, ", baseline_data);
                if ((baseline_data < *(baseline_data_test + count*2))
                 || (baseline_data > *(baseline_data_test + count*2 + 1))) {
                    seq_printf(s, "step 1:test RT92 error,"
                                  "baseline_data[%d][%d] = %d[%d, %d]\n",
                                  x, y, baseline_data,
                                  *(baseline_data_test + count*2),
                                  *(baseline_data_test + count*2 + 1));
                    TPD_INFO("step 1:test RT92 error,"
                             "baseline_data[%d][%d] = %d[%d, %d]\n",
                             x, y, baseline_data,
                             *(baseline_data_test + count*2),
                             *(baseline_data_test + count*2 + 1));
                    error_count++;
                    if (tp_debug != 2) {
                        goto END;
                    }
                }
            } else if ((x == tx_2d_num) && (y < 2)) {
                TPD_DEBUG_NTAG("%d, ", baseline_data);

                if (((baseline_data) < *(baseline_data_test + count*2))
                 || ((baseline_data) > *(baseline_data_test + count*2 + 1))) {
                    seq_printf(s, "step 1:test RT92 error,"
                                  "touch key baseline_data[%d][%d] = %d[%d, %d]\n",
                                  x, y, baseline_data,
                                  *(baseline_data_test + count*2),
                                  *(baseline_data_test + count*2 + 1));
                    TPD_INFO("step 1:test RT92 error,"
                             "touch key baseline_data[%d][%d] = %d[%d, %d]\n",
                             x, y, baseline_data,
                             *(baseline_data_test + count*2),
                             *(baseline_data_test + count*2 + 1));
                    error_count++;
                    if (tp_debug != 2) {
                        goto END;
                    }
                }
            }
            count++;
        }

        TPD_DEBUG_NTAG("\n");
        if (syna_testdata->fd >= 0) {
            store_to_file(syna_testdata->fd, "\n");
        }
    }
    if (syna_testdata->fd >= 0) {
        store_to_file(syna_testdata->fd, "\n");
    }

END:
    if (raw_data) {
        kfree(raw_data);
    }

    if (error_count) {
        return TEST_FAIL;
    }
    return TEST_PASS;
}
static int synaptics_tddi_RT92_doze_test(struct seq_file *s, void *chip_data, struct syna_testdata *syna_testdata)
{
    int ret = 0;
    uint8_t *raw_data = NULL;
    uint16_t *baseline_data_test = NULL;
    uint16_t baseline_data;
    uint8_t x, y;
    uint16_t z;
    uint8_t tmp_arg1;
    uint8_t tmp_arg2;
    int tx_2d_num = syna_testdata->TX_NUM - 1;
    int error_count = 0;
    struct test_header *ph = (struct test_header *)(syna_testdata->fw->data);
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;
    struct synaptics_register *reg_info = &chip_info->reg_info;
    struct i2c_client *client = chip_info->client;

    store_to_file(syna_testdata->fd, "RT92 doze:\n");

    TPD_INFO("\n step 5:RT92 doze test\n");
    TPD_INFO("Force Doze Mode:\r\n");
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x01);
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_CTRL225, 0x01);//Force Doze
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, COMMAND_FORCE_UPDATE);//force update
    checkCMD(chip_info);

    synaptics_read_F54_base_reg(chip_info);

    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_DATA_BASE, 92);//select report type 92
    if (ret < 0) {
        TPD_INFO("[line:%d]read_baseline: touch_i2c_write_byte failed \n",__LINE__);
        seq_printf(s, "[line:%d]read_baseline: touch_i2c_write_byte failed \n",__LINE__);
        store_to_file(syna_testdata->fd, "I2C read error\n");
        return TEST_FAIL;
    }

    baseline_data_test = (uint16_t *)(syna_testdata->fw->data + ph->array_limit_offset);
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
    checkCMD(chip_info);
    TPD_DEBUG("Force Cal oK\n");
    ret = touch_i2c_write_word(client, reg_info->F54_ANALOG_DATA_BASE + 1, 0x00);//set fifo 00
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0x01);//get report
    checkCMD(chip_info);

    raw_data = kzalloc(syna_testdata->TX_NUM * syna_testdata->RX_NUM * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!raw_data) {
        TPD_INFO("raw_data kzalloc error\n");
        seq_printf(s, "RT92: failed to alloc memory\n");
        store_to_file(syna_testdata->fd, "failed to alloc memory\n");
        return TEST_FAIL;
    }

    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE+3,
                            syna_testdata->TX_NUM*syna_testdata->RX_NUM*2, raw_data);     //read data
    for (x = 0; x < syna_testdata->TX_NUM; x++) {
        TPD_DEBUG_NTAG("[%d]: ", x);
        for (y = 0; y < syna_testdata->RX_NUM; y++) {
            z = syna_testdata->RX_NUM * x + y;
            tmp_arg1 = raw_data[z*2] & 0xff;
            tmp_arg2 = raw_data[z*2 + 1] & 0xff;
            baseline_data = (tmp_arg2 << 8) | tmp_arg1;

            if (syna_testdata->fd >= 0) {
                store_to_file(syna_testdata->fd, "%d, ", baseline_data);
            }

            if (((x < 2) && (y < syna_testdata->RX_NUM ))
               || ((x == tx_2d_num ) && (y == 0))) {
                TPD_DEBUG_NTAG("%d, ", baseline_data);
                if ((baseline_data < LIMIT_DOZE_LOW) || (baseline_data > LIMIT_DOZE_HIGH)){
                    seq_printf(s, "step 5:RT92 doze test error,"
                                  "baseline_data[%d][%d] = %d[%d, %d]\n",
                                  x, y, baseline_data, LIMIT_DOZE_LOW, LIMIT_DOZE_HIGH);
                    TPD_INFO("step 5:RT92 doze test error,"
                             "baseline_data[%d][%d] = %d[%d, %d]\n",
                             x, y, baseline_data, LIMIT_DOZE_LOW, LIMIT_DOZE_HIGH);
                    error_count++;
                    if (tp_debug != 2) {
                        goto END;
                    }
                }
            }
        }

        TPD_DEBUG_NTAG("\n");
        if (syna_testdata->fd >= 0) {
            store_to_file(syna_testdata->fd, "\n");
        }
    }
    if (syna_testdata->fd >= 0) {
        store_to_file(syna_testdata->fd, "\n");
    }

END:
    if (raw_data) {
        kfree(raw_data);
    }

    if (error_count) {
        return TEST_FAIL;
    }
    return TEST_PASS;
}

static int synaptics_tddi_open_test(struct seq_file *s, void *chip_data, struct syna_testdata *syna_testdata)
{
    int ret = 0;
    int x, y, z;
    uint8_t buffer[9];
    uint8_t *raw_data = NULL;
    uint8_t *raw_data_2 = NULL;
    int tx_2d_num = syna_testdata->TX_NUM - 1;
    uint16_t *baseline_data_test = NULL;
    uint16_t baseline_data;
    signed short *raw_data_delta_image = NULL;
    uint8_t tmp_arg1, tmp_arg2;
    int error_count = 0;

    struct f54_control_91  original_f54_ctrl91;
    struct f54_control_99  original_f54_ctrl99;
    struct f54_control_182 original_f54_ctrl182;
    struct f55_control_43  ctrl_43;

    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;
    struct i2c_client *client = chip_info->client;
    struct synaptics_register *reg_info = &chip_info->reg_info;
    struct test_header *ph = (struct test_header *)(syna_testdata->fw->data);

    TPD_INFO("\n step 2 open test \n");

    store_to_file(syna_testdata->fd, "Open Test:\n");
    synaptics_read_F54_base_reg(chip_info);

    /* keep the original reference high/low capacitance */
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL91, sizeof(original_f54_ctrl91.data), original_f54_ctrl91.data);
    if (ret < 0) {
        TPD_INFO("step 2 open test: Failed to read original data from f54_ctrl91\n");
        store_to_file(syna_testdata->fd, "Failed to read Ctrl91");
        return TEST_FAIL;
    }
    /* keep the original integration and reset duration */
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL99, sizeof(original_f54_ctrl99.data), original_f54_ctrl99.data);
    if (ret < 0) {
        TPD_INFO("step 2 open test: Failed to read original data from f54_ctrl99\n");
        store_to_file(syna_testdata->fd, "Failed to read Ctrl99");
        return TEST_FAIL;
    }
    /* keep the original timing control */
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL182, sizeof(original_f54_ctrl182.data), original_f54_ctrl182.data);
    if (ret < 0) {
        TPD_INFO("step 2 open test: Failed to read original data from f54_ctrl182\n");
        store_to_file(syna_testdata->fd, "Failed to read Ctrl182");
        return TEST_FAIL;
    }

    /* 1 Wide refcap hi/ lo and feedback, Write 0x0F to F54_ANALOG_CTRL91 */
    buffer[0] = 0x0f;
    buffer[1] = 0x0f;
    buffer[2] = 0x0f;
    buffer[3] = original_f54_ctrl91.reference_receiver_feedback_capacitance;
    buffer[4] = original_f54_ctrl91.gain_ctrl;
    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL91, 5, buffer);
    if (ret < 0) {
        TPD_INFO("step 2 open test write F54_ANALOG_CTRL91 error\n");
        seq_printf(s, "step 2 open test write F54_ANALOG_CTRL91 error\n");
        store_to_file(syna_testdata->fd, "Failed to write Ctrl191");
        return TEST_FAIL;
    }

    /* 2 Increase RST_DUR to 1.53us, Write 0x5c to F54_ANALOG_CTRL99 */
    buffer[0] = original_f54_ctrl99.integration_duration_lsb;
    buffer[1] = original_f54_ctrl99.integration_duration_msb;
    buffer[2] = 0x5c;
    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL99, 3, buffer);
    if (ret < 0) {
        TPD_INFO("step 2 open test write F54_ANALOG_CTRL99 error\n");
        seq_printf(s, "step 2 open test write F54_ANALOG_CTRL99 error\n");
        store_to_file(syna_testdata->fd, "Failed to write Ctrl199");
        return TEST_FAIL;
    }

    /* 3 Write 0x02 to F54_ANALOG_CTRL182 (00)/00 and (00)/02 */
    buffer[0] = ELEC_OPEN_TEST_TX_ON_COUNT;                 //cbc_timing_ctrl_tx_lsb
    buffer[1] = (ELEC_OPEN_TEST_TX_ON_COUNT >> 8) & 0xff;   //cbc_timing_ctrl_tx_msb
    buffer[2] = ELEC_OPEN_TEST_RX_ON_COUNT;                 //cbc_timing_ctrl_rx_lsb
    buffer[3] = (ELEC_OPEN_TEST_RX_ON_COUNT >> 8) & 0xff;   //cbc_timing_ctrl_rx_msb
    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL182, 4, buffer);
    if (ret < 0) {
        TPD_INFO("step 2 open test write F54_ANALOG_CTRL182 error\n");
        seq_printf(s, "step 2 open test write F54_ANALOG_CTRL182 error\n");
        store_to_file(syna_testdata->fd, "Failed to write Ctrl82");
        return TEST_FAIL;
    }

    /* 4 Change the INT_DUR as ELEC_OPEN_INT_DUR_ONE */
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL99, 3, buffer);
    buffer[0] = ELEC_OPEN_INT_DUR_ONE;                      //integration_duration_lsb
    buffer[1] = (ELEC_OPEN_INT_DUR_ONE >> 8) & 0xff;        //integration_duration_msb
    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL99, 3, buffer);
    if (ret < 0) {
        TPD_INFO("step 2 open test Change the INT_DUR as ELEC_OPEN_INT_DUR_ONE error\n");
        seq_printf(s, "step 2 open Change the INT_DUR as ELEC_OPEN_INT_DUR_ONE error\n");
        store_to_file(syna_testdata->fd, "Failed to change OPEN INT\n");
        return TEST_FAIL;
    }

    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, COMMAND_FORCE_UPDATE);//force update
    checkCMD(chip_info);

    /* 5 Capture raw capacitance (rt92) image 1 */
    /* Run Report Type 92 */
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_DATA_BASE, 92);//select report type 92
    if (ret < 0) {
        TPD_INFO("step 2 open test: Capture raw capacitance (rt92) image 1 error\n");
        seq_printf(s, "step 2 open test: Capture raw capacitance (rt92) image 1 error\n");
        store_to_file(syna_testdata->fd, "Failed to capture raw rt92\n");
        return TEST_FAIL;
    }

    raw_data = kzalloc(syna_testdata->TX_NUM * syna_testdata->RX_NUM * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!raw_data) {
        TPD_INFO("failed to alloc memory\n");
        seq_printf(s, "failed to alloc memory");
        store_to_file(syna_testdata->fd, "no more memory\n");
        error_count++;
        goto RESTORE_CONFIG;
    }

    baseline_data_test = (uint16_t *)(syna_testdata->fw->data + ph->array_limit_offset);
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
    ret = touch_i2c_write_word(client, reg_info->F54_ANALOG_DATA_BASE + 1, 0x00);//set fifo 00
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0x01);//get report
    checkCMD(chip_info);
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE+3,
                            syna_testdata->TX_NUM*syna_testdata->RX_NUM*2, raw_data);     //read data

    /* 6 Change the INT_DUR into ELEC_OPEN_INT_DUR_TWO */
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL99, 3, buffer);
    buffer[0] = ELEC_OPEN_INT_DUR_TWO;                      //integration_duration_lsb
    buffer[1] = (ELEC_OPEN_INT_DUR_TWO >> 8) & 0xff;        //integration_duration_msb
    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL99, 3, buffer);
    if (ret < 0) {
        TPD_INFO("step 2 open test: Change the INT_DUR into ELEC_OPEN_INT_DUR_TWO  error\n");
        seq_printf(s, "step 2 open test: Change the INT_DUR into ELEC_OPEN_INT_DUR_TWO  error\n");
        store_to_file(syna_testdata->fd, "failed to Change Int\n");
        error_count++;
        goto RESTORE_CONFIG;
    }

    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, COMMAND_FORCE_UPDATE);//force update
    checkCMD(chip_info);

    /* 7 Capture raw capacitance (rt92) image 2 */
    /* Run Report Type 92 */
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_DATA_BASE, 92);//select report type 92
    if (ret < 0) {
        TPD_INFO("step 2 open test: Capture raw capacitance (rt92) image 2 error\n");
        seq_printf(s, "step 2 open test: Capture raw capacitance (rt92) image 2 error\n");
        error_count++;
        goto RESTORE_CONFIG;
    }

    raw_data_2 = kzalloc(syna_testdata->TX_NUM * syna_testdata->RX_NUM * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!raw_data_2) {
        TPD_INFO("failed to alloc memory\n");
        seq_printf(s, "failed to alloc memory");
        store_to_file(syna_testdata->fd, "No more memory\n");
        error_count++;
        goto RESTORE_CONFIG;
    }

    raw_data_delta_image = kzalloc(syna_testdata->TX_NUM * syna_testdata->RX_NUM * sizeof(signed short), GFP_KERNEL);
    if (!raw_data_delta_image) {
        TPD_INFO("raw_data_delta_image kzalloc error\n");
        seq_printf(s, "failed to alloc memory");
        store_to_file(syna_testdata->fd, "No more memory\n");
        error_count++;
        goto RESTORE_CONFIG;
    }

    baseline_data_test = (uint16_t *)(syna_testdata->fw->data + ph->array_limit_offset);
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
    ret = touch_i2c_write_word(client, reg_info->F54_ANALOG_DATA_BASE + 1, 0x00);//set fifo 00
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0x01);//get report
    checkCMD(chip_info);
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE + 3, syna_testdata->TX_NUM*syna_testdata->RX_NUM*2, raw_data_2);     //read data

    /* 8 generate the delta image, which is equeal to image2 - image1 */
    /* unit is femtofarad (fF) */
    TPD_DEBUG_NTAG("----------- delta data ---------- \n ");
    for (x = 0; x < syna_testdata->TX_NUM; x++) {
        TPD_DEBUG_NTAG("[%2d] ", x);
        for (y = 0; y < syna_testdata->RX_NUM; y++) {
            z = syna_testdata->RX_NUM * x + y;
            tmp_arg1 = raw_data[z*2] & 0xff;
            tmp_arg2 = raw_data[z*2 + 1] & 0xff;
            baseline_data = (tmp_arg2 << 8) | tmp_arg1;

            tmp_arg1 = raw_data_2[z*2] & 0xff;
            tmp_arg2 = raw_data_2[z*2 + 1] & 0xff;
            raw_data_delta_image[z] = ((tmp_arg2 << 8) | tmp_arg1) - baseline_data;
            TPD_DEBUG_NTAG("%4d ", raw_data_delta_image[z]);
        }
        TPD_DEBUG_NTAG("\n");
    }

    /* 9 restore the original configuration */
    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL91, sizeof(original_f54_ctrl91.data), original_f54_ctrl91.data);
    if (ret < 0) {
        TPD_INFO("%s: Failed to read original data from f54_ctrl91\n",__func__);
    }

    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL99, sizeof(original_f54_ctrl99.data), original_f54_ctrl99.data);
    if (ret < 0) {
        TPD_INFO("%s: Failed to read original data from f54_ctrl99\n",__func__);
    }

    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL182, sizeof(original_f54_ctrl182.data), original_f54_ctrl182.data);
    if (ret < 0) {
        TPD_INFO("%s: Failed to read original data from f54_ctrl182\n",__func__);
    }

    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, COMMAND_FORCE_UPDATE);//force update
    checkCMD(chip_info);

    store_to_file(syna_testdata->fd, "Open Test Limit 1:  > %d ok\n", ELEC_OPEN_TEST_LIMIT_ONE);

    /* 10 phase 1, data verification */
    /* the delta value should be lower than the test limit */
    for (x = 0; x < syna_testdata->TX_NUM; x++) {
        for (y = 0; y < syna_testdata->RX_NUM; y++) {
            store_to_file(syna_testdata->fd, "%d, ", raw_data_delta_image[x * syna_testdata->RX_NUM + y]);
            if (x < tx_2d_num && raw_data_delta_image[x * syna_testdata->RX_NUM + y] < ELEC_OPEN_TEST_LIMIT_ONE) {
                TPD_INFO("step 2 open test: (%2d,%2d) = %4d < ELEC_OPEN_TEST_LIMIT_ONE(%d) fail\n",
                        x, y, raw_data_delta_image[x * syna_testdata->RX_NUM + y], ELEC_OPEN_TEST_LIMIT_ONE);
                if (!error_count) {
                    seq_printf(s, "step 2 open test Open One:(%2d,%2d) = %4d Limit:%d\n",
                        x, y, raw_data_delta_image[x * syna_testdata->RX_NUM + y], ELEC_OPEN_TEST_LIMIT_ONE);
                }
                error_count++;
                if (tp_debug != 2) {
                    goto RESTORE_CONFIG;
                }
            }
        }
        store_to_file(syna_testdata->fd, "\n");
    }

    /* 11 phase 2, data calculation and verification */
    /* the calculated ratio should be lower than the test limit */
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x03);    // page 3
    if (ret < 0) {
        TPD_INFO("%s: failed for page select\n", __func__);
        return TEST_FAIL;
    }
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F55_SENSOR_CTRL43, sizeof(ctrl_43), ctrl_43.data);     //read data
    synaptics_read_F54_base_reg(chip_info);
    tddi_ratio_calculation(raw_data_delta_image, &ctrl_43, syna_testdata->TX_NUM, syna_testdata->RX_NUM);

    store_to_file(syna_testdata->fd, "Open Test Limit 2:  > %d ok\n", ELEC_OPEN_TEST_LIMIT_TWO);

    for (x = 0; x < syna_testdata->TX_NUM; x++) {
        for (y = 0; y < syna_testdata->RX_NUM; y++) {
            store_to_file(syna_testdata->fd, "%d, ", raw_data_delta_image[x * syna_testdata->RX_NUM + y]);
            if (x < tx_2d_num && raw_data_delta_image[x * syna_testdata->RX_NUM + y] < ELEC_OPEN_TEST_LIMIT_TWO) {
                TPD_INFO("step 2 open test: (%2d,%2d) = %4d < ELEC_OPEN_TEST_LIMIT_TWO(%d) fail\n",
                        x, y, raw_data_delta_image[x * syna_testdata->RX_NUM + y], ELEC_OPEN_TEST_LIMIT_TWO);
                if (!error_count) {
                    seq_printf(s, "step 2 open test Open Two:(%2d,%2d)=%4d Limit:%d\n",
                        x, y, raw_data_delta_image[x * syna_testdata->RX_NUM + y], ELEC_OPEN_TEST_LIMIT_TWO);
                }
                error_count++;
                if (tp_debug != 2) {
                    goto RESTORE_CONFIG;
                }
            }
        }
        store_to_file(syna_testdata->fd, "\n");
    }

    store_to_file(syna_testdata->fd, "\n");

RESTORE_CONFIG:
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x00);
    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL91, sizeof(original_f54_ctrl91.data), original_f54_ctrl91.data);
    if (ret < 0) {
        TPD_INFO("%s: Failed to read original data from f54_ctrl91\n",__func__);
    }

    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL99, sizeof(original_f54_ctrl99.data), original_f54_ctrl99.data);
    if (ret < 0) {
        TPD_INFO("%s: Failed to read original data from f54_ctrl99\n",__func__);
    }

    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL182, sizeof(original_f54_ctrl182.data), original_f54_ctrl182.data);
    if (ret < 0) {
        TPD_INFO("%s: Failed to read original data from f54_ctrl182\n",__func__);
    }


    if (raw_data)
        kfree(raw_data);
    if (raw_data_2)
        kfree(raw_data_2);
    if (raw_data_delta_image)
        kfree(raw_data_delta_image);

    if (error_count) {
        return TEST_FAIL;
    }
    return TEST_PASS;
}

static int synaptics_tddi_short_test(struct seq_file *s, void *chip_data, struct syna_testdata *syna_testdata)
{
    int ret = 0;
    int x, y, z;
    uint8_t buffer[9];
    uint8_t tmp_arg1, tmp_arg2;
    int tx_2d_num = syna_testdata->TX_NUM - 1;
    unsigned int buffer_size = tx_2d_num * syna_testdata->RX_NUM * 2;
    uint8_t *tddi_rt95_raw_data = NULL;
    signed short *tddi_rt95_part_one = NULL;
    signed short *tddi_rt95_part_two = NULL;
    int error_count = 0;

    struct f54_control_91  original_f54_ctrl91;
    struct f54_control_99  original_f54_ctrl99;
    struct f54_control_182 original_f54_ctrl182;
    struct f55_control_43  ctrl_43;

    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;
    struct i2c_client *client = chip_info->client;
    struct synaptics_register *reg_info = &chip_info->reg_info;

    store_to_file(syna_testdata->fd, "Short Test:\n");

    synaptics_read_F54_base_reg(chip_info);

    /* keep the original reference high/low capacitance */
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL91, sizeof(original_f54_ctrl91.data), original_f54_ctrl91.data);
    if (ret < 0) {
        TPD_INFO("step 2 open test: Failed to read original data from f54_ctrl91\n");
        store_to_file(syna_testdata->fd, "Failed to read Ctrl91");
        return TEST_FAIL;
    }
    /* keep the original integration and reset duration */
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL99, sizeof(original_f54_ctrl99.data), original_f54_ctrl99.data);
    if (ret < 0) {
        TPD_INFO("step 2 open test: Failed to read original data from f54_ctrl99\n");
        store_to_file(syna_testdata->fd, "Failed to read Ctrl99");
        return TEST_FAIL;
    }
    /* keep the original timing control */
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL182, sizeof(original_f54_ctrl182.data), original_f54_ctrl182.data);
    if (ret < 0) {
        TPD_INFO("step 2 open test: Failed to read original data from f54_ctrl182\n");
        store_to_file(syna_testdata->fd, "Failed to read Ctrl182");
        return TEST_FAIL;
    }

    tddi_rt95_raw_data = kzalloc(2 * tx_2d_num * syna_testdata->RX_NUM * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!tddi_rt95_raw_data) {
        TPD_INFO("tddi_rt95_raw_data kzalloc error\n");
        error_count++;
        goto END;
    }

    tddi_rt95_part_one = kzalloc(tx_2d_num * syna_testdata->RX_NUM * sizeof(signed short), GFP_KERNEL);
    if (!tddi_rt95_part_one) {
        TPD_INFO("tddi_rt95_part_one kzalloc error\n");
        error_count++;
        goto END;
    }

    tddi_rt95_part_two = kzalloc(tx_2d_num * syna_testdata->RX_NUM * sizeof(signed short), GFP_KERNEL);
    if (!tddi_rt95_part_two) {
        TPD_INFO("tddi_rt95_part_two kzalloc error\n");
        error_count++;
        goto END;
    }

    buffer[0] = EXTEND_EE_SHORT_INT_DUR;
    buffer[1] = (EXTEND_EE_SHORT_INT_DUR >> 8) & 0xff;
    buffer[2] = EXTEND_EE_SHORT_RESET_DUR;
    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL99, 3, buffer);
    if (ret < 0) {
        TPD_INFO("step 3 short test Modify the Reset duration, Integration duration error\n");
        seq_printf(s, "step 3 short test Modify the Reset duration, Integration duration error\n");
        error_count++;
        goto RESTORE_CONFIG;
    }

    buffer[0] = EXTEND_EE_SHORT_TX_ON_COUNT;
    buffer[1] = (EXTEND_EE_SHORT_TX_ON_COUNT >> 8) & 0xff;
    buffer[2] = EXTEND_EE_SHORT_RX_ON_COUNT;
    buffer[3] = (EXTEND_EE_SHORT_RX_ON_COUNT >> 8) & 0xff;
    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL182, 4, buffer);
    if (ret < 0) {
        TPD_INFO("step 3 short test Modify CBC Timing Control error\n");
        seq_printf(s, "step 3 short test Modify CBC Timing Control error\n");
        error_count++;
        goto RESTORE_CONFIG;
    }

    /* 2 Force update */
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, COMMAND_FORCE_UPDATE);//force update
    checkCMD(chip_info);
    TPD_INFO("short test force update %d\n", __LINE__);

    /* 3 get report type 95 */
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_DATA_BASE, 95);//select report type 95
    if (ret < 0) {
        TPD_INFO("[line:%d]read_baseline: touch_i2c_write_byte failed \n",__LINE__);
        seq_printf(s, "[line:%d]read_baseline: touch_i2c_write_byte failed \n",__LINE__);
        error_count++;
        goto RESTORE_CONFIG;
    }

    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
    ret = touch_i2c_write_word(client, reg_info->F54_ANALOG_DATA_BASE + 1, 0x00);//set fifo 00
    ret = touch_i2c_write_byte(client, reg_info->F54_ANALOG_COMMAND_BASE, 0x01);//get report
    checkCMD(chip_info);

    TPD_INFO("short test get report %d\n", __LINE__);
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE + 3,
                                2 * tx_2d_num*syna_testdata->RX_NUM * 2, tddi_rt95_raw_data);     //read data

    /* 4 use the upper half as part 1 image */
    /* the data should be lower than TEST_LIMIT_PART1 ( fail, if > TEST_LIMIT_PART1 ) */
    TPD_DEBUG_NTAG("----------- step 3 short test : rt95 part 1 value ---------- \n ");
    for (x = 0; x < tx_2d_num; x++) {
        TPD_DEBUG_NTAG("[%2d] ", x);
        for (y = 0; y < syna_testdata->RX_NUM; y++) {
            z = syna_testdata->RX_NUM * x + y;
            tmp_arg1 = tddi_rt95_raw_data[z*2] & 0xff;
            tmp_arg2 = tddi_rt95_raw_data[z*2 + 1] & 0xff;
            tddi_rt95_part_one[z] = ((tmp_arg2 << 8) | tmp_arg1);
            TPD_DEBUG_NTAG("%4d ", tddi_rt95_part_one[z]);
        }
        TPD_DEBUG_NTAG("\n");
    }

    store_to_file(syna_testdata->fd, "Short Test Limit 1: < %d ok\n", EXTEND_EE_SHORT_TEST_LIMIT_PART1);
    for (x = 0; x < tx_2d_num; x++) {
        for (y = 0; y < syna_testdata->RX_NUM; y++) {
            store_to_file(syna_testdata->fd, "%d, ", tddi_rt95_part_one[x * syna_testdata->RX_NUM + y]);
            if (tddi_rt95_part_one[x*syna_testdata->RX_NUM + y] > EXTEND_EE_SHORT_TEST_LIMIT_PART1) {
                TPD_INFO("step 3 short test : (%2d,%2d) = %4d > EXTEND_EE_SHORT_TEST_LIMIT_PART1(%d) fail\n",
                        x, y, tddi_rt95_part_one[x * syna_testdata->RX_NUM + y], EXTEND_EE_SHORT_TEST_LIMIT_PART1);
                if (!error_count) {
                    seq_printf(s, "step 3 short test Short One:(%2d,%2d)=%4d Limit:%d\n",
                        x, y, tddi_rt95_part_two[x * syna_testdata->RX_NUM + y], EXTEND_EE_SHORT_TEST_LIMIT_PART1);
                }
                error_count++;
                if (tp_debug != 2) {
                    goto RESTORE_CONFIG;
                }
            }
        }
        store_to_file(syna_testdata->fd, "\n");
    }

    /* 5 use the lower half as part 2 image */
    /* and perform the calculation */
    /* the calculated data should be over than TEST_LIMIT_PART2 ( fail, if < TEST_LIMIT_PART2 ) */
    TPD_DEBUG_NTAG("----------- step 3 short test : rt95 part 2 value ---------- \n ");
    for (x = 0; x < tx_2d_num; x++) {
        TPD_DEBUG_NTAG("[%2d] ", x);
        for (y = 0; y < syna_testdata->RX_NUM; y++) {
            z = syna_testdata->RX_NUM * x + y;
            tmp_arg1 = tddi_rt95_raw_data[buffer_size + z*2] & 0xff;
            tmp_arg2 = tddi_rt95_raw_data[buffer_size + z*2 + 1] & 0xff;
            tddi_rt95_part_two[syna_testdata->RX_NUM * x + y] = ((tmp_arg2 << 8) | tmp_arg1);
            TPD_DEBUG_NTAG("%4d ", tddi_rt95_part_two[z]);
        }
        TPD_DEBUG_NTAG("\n");
    }

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x03);    // page 3
    if (ret < 0) {
        TPD_INFO("%s: failed for page select\n", __func__);
        error_count++;
        goto RESTORE_CONFIG;
    }
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F55_SENSOR_CTRL43, sizeof(ctrl_43), ctrl_43.data); //read data
    synaptics_read_F54_base_reg(chip_info);
    tddi_ratio_calculation(tddi_rt95_part_two, &ctrl_43, tx_2d_num, syna_testdata->RX_NUM);

    store_to_file(syna_testdata->fd, "Short Test Limit 2: > %d ok\n", EXTEND_EE_SHORT_TEST_LIMIT_PART2);

    for (x = 0; x < tx_2d_num; x++) {
        for (y = 0; y < syna_testdata->RX_NUM; y++) {
            store_to_file(syna_testdata->fd, "%d, ", tddi_rt95_part_two[x * syna_testdata->RX_NUM + y]);
            if (tddi_rt95_part_two[x * syna_testdata->RX_NUM + y] < EXTEND_EE_SHORT_TEST_LIMIT_PART2) {
                TPD_INFO("step 3 short test : (%2d,%2d) = %4d < EXTEND_EE_SHORT_TEST_LIMIT_PART2(%d) fail\n",
                        x, y, tddi_rt95_part_two[x * syna_testdata->RX_NUM + y], EXTEND_EE_SHORT_TEST_LIMIT_PART2);
                if (!error_count) {
                    seq_printf(s, "step 3 short test Short Two:(%2d,%2d)=%4d Limit:%d\n",
                        x, y, tddi_rt95_part_two[x * syna_testdata->RX_NUM + y], EXTEND_EE_SHORT_TEST_LIMIT_PART2);
                }
                error_count++;
                if (tp_debug != 2) {
                    goto RESTORE_CONFIG;
                }
            }
        }
        store_to_file(syna_testdata->fd, "\n");
    }

RESTORE_CONFIG:
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x01);

    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL91, sizeof(original_f54_ctrl91.data), original_f54_ctrl91.data);
    if (ret < 0) {
        TPD_INFO("%s: Failed to read original data from f54_ctrl91\n",__func__);
    }

    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL99, sizeof(original_f54_ctrl99.data), original_f54_ctrl99.data);
    if (ret < 0) {
        TPD_INFO("%s: Failed to read original data from f54_ctrl99\n",__func__);
    }

    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL182, sizeof(original_f54_ctrl182.data), original_f54_ctrl182.data);
    if (ret < 0) {
        TPD_INFO("%s: Failed to read original data from f54_ctrl182\n",__func__);
    }


END:

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x00);

    if (tddi_rt95_raw_data)
        kfree(tddi_rt95_raw_data);
    if (tddi_rt95_part_one)
        kfree(tddi_rt95_part_one);
    if (tddi_rt95_part_two)
        kfree(tddi_rt95_part_two);

    if (error_count) {
        return TEST_FAIL;
    }

    return TEST_PASS;
}

static void synaptics_auto_test(struct seq_file *s, void *chip_data, struct syna_testdata *syna_testdata)
{
    int error_count = 0;
    int ret = 0;
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;

    ret = synaptics_enable_interrupt(chip_info, false);

    error_count += synaptics_int_pin_test(s, chip_data, syna_testdata);
    error_count += synaptics_tddi_RT92_test(s, chip_data, syna_testdata);
    synaptics_tddi_open_test(s, chip_data, syna_testdata);
    synaptics_tddi_short_test(s, chip_data, syna_testdata);
    error_count += synaptics_tddi_RT92_doze_test(s, chip_data, syna_testdata);

    seq_printf(s, "imageid = 0x%llx, deviceid = 0x%llx\n", syna_testdata->TP_FW, syna_testdata->TP_FW);
    seq_printf(s, "%d error(s). %s\n", error_count, error_count?"":"All test passed.");
    TPD_INFO(" TP auto test %d error(s). %s\n", error_count, error_count?"":"All test passed.");
}

static void synaptics_baseline_read(struct seq_file *s, void *chip_data)
{
    int ret = 0;
    int x = 0, y = 0, z = 0;
    int i = 0;
    uint16_t baseline_data = 0;
    uint8_t *raw_data = NULL;
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;

    if (!chip_info)
        return ;

    raw_data = kzalloc(chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!raw_data) {
        TPD_INFO("raw_data kzalloc error\n");
        return;
    }

    synaptics_read_F54_base_reg(chip_info);

    for (i = 0; i < 2; i++) {
        if (i == 0) {
            seq_printf(s, "\nRT92 data:\n");
        } else if (i == 1) {
            seq_printf(s, "\nRT92 doze data:\n");
            TPD_INFO("Force Doze Mode:\r\n");
            ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x01);
            ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_CTRL225, 0x01);//Force Doze
            ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE, COMMAND_FORCE_UPDATE);//force update
            checkCMD(chip_info);
        }

        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE, 92);//select report type 92
        if (ret < 0) {
            TPD_INFO("[line:%d]read_baseline: touch_i2c_write_byte failed \n",__LINE__);
            seq_printf(s, "[line:%d]read_baseline: touch_i2c_write_byte failed \n",__LINE__);
            goto END;
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
    }

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
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;

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
        seq_printf(s, "checkCMD error, can not read delta data\n");
        TPD_INFO("checkCMD error, can not read delta data\n");
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE, 0x00);//select report type 0x00
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F54_ANALOG_COMMAND_BASE, 0x01);//get report 1
        seq_printf(s, "write F54_ANALOG_DATA_BASE=0, F54_ANALOG_COMMAND_BASE=1,recovery tp touch\n");
        TPD_INFO("write F54_ANALOG_DATA_BASE=0, F54_ANALOG_COMMAND_BASE=1, recovery tp touch\n");
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
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;

    if (!chip_info)
        return ;
    TPD_INFO("%s start\n", __func__);
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

    seq_printf(s, "====================================================\n");
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x4);
    if (ret < 0) {
        TPD_INFO("%s: failed for page select\n", __func__);
        seq_printf(s, "%s: failed for page select\n", __func__);
        return;
    }
    ret = touch_i2c_read_byte(chip_info->client, 0x19);
    seq_printf(s, "F51_CUSTOM_DATA95 Metal Detected Interrupt: 0x%x, bit1(in esd mode)\n", ret);

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F51_CUSTOM_GRIP_CTRL);
    seq_printf(s, "edge control function register(0x%x) = 0x%x, bit0(Enable Grip Suppression)\n",
               chip_info->reg_info.F51_CUSTOM_GRIP_CTRL, ret);

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F51_CUSTOM_CTRL48);
    seq_printf(s, "ps status register(0x%x) = 0x%x, bit2(ps_status,1:near 0:far) bit1(Force enter ESD mode) bit0(ESD mode enable)\n\n",
               chip_info->reg_info.F51_CUSTOM_CTRL48, ret);

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
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;

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


static int checkFlashState(struct chip_data_s4322 *chip_info)
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
static void re_scan_PDT(struct chip_data_s4322 *chip_info)
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
    uint16_t block, firmware, configuration, disp_configuration;
    uint32_t CURRENT_FIRMWARE_ID = 0, FIRMWARE_ID = 0;
    const uint8_t *Config_Data = NULL;
    const uint8_t *Disp_Config_Data = NULL;
    const uint8_t *Firmware_Data = NULL;
    struct image_header_data header;
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;

    if (!chip_info) {
        TPD_INFO("Chip info is NULL\n");
        return 0;
    }

    TPD_INFO("%s is called\n", __func__);

    //step 1:fill Fw related header, get all data.
    synaptics_parse_header(&header, fw->data);
    if ((header.firmware_size + header.config_size + 0x100 + header.disp_config_size) > (uint32_t)fw->size) {
        TPD_INFO("firmware_size + config_size + 0x100 > data_len data_len = %d \n", (uint32_t)fw->size);
        return FW_NO_NEED_UPDATE;
    }
    Firmware_Data = fw->data + 0x100 + header.bootloader_size;
    Config_Data = Firmware_Data + header.firmware_size;
    Disp_Config_Data = fw->data + header.disp_config_offset;

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
    block = 256;
    TPD_DEBUG("block is %d \n", block);
    firmware = header.firmware_size / block;
    TPD_DEBUG("firmware is %d \n", firmware);
    configuration = header.config_size / block;
    TPD_DEBUG("configuration is %d \n", configuration);
    disp_configuration = header.disp_config_size / block;
    TPD_DEBUG("disp_configuration is %d \n", disp_configuration);

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
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.SynaF34_FlashControl, 0x22); //erase flash except display config  ,0x03 erase all flash
    msleep(2000);
    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.SynaF34_FlashControl);
    TPD_DEBUG("going to flash firmware area synaF34_FlashControl %d\n", ret);

    //step 4:flash firmware zone
    TPD_INFO("update-----------------firmware ------------------update!\n");
    TPD_DEBUG("cnt %d\n", firmware);

    //a)write SynaF34Reflash_BlockNum to access
    buf[0] = 0 & 0x00ff;
    buf[1] = (0 & 0xff00) >> 8;
    touch_i2c_write_block(chip_info->client, chip_info->reg_info.SynaF34Reflash_BlockNum, 2, buf);

    for (j = 0; j < firmware; j++) {

        touch_i2c_write_block(chip_info->client, chip_info->reg_info.SynaF34Reflash_BlockData, block, &Firmware_Data[j*block]);
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
    //a)write SynaF34Reflash_BlockNum to access
    buf[0] = 0 & 0x00ff;
    buf[1] = (0 & 0xff00) >> 8;
    touch_i2c_write_block(chip_info->client, chip_info->reg_info.SynaF34Reflash_BlockNum, 2, buf);

    for (j = 0; j < configuration; j++) {

        //b) write data
        touch_i2c_write_block(chip_info->client, chip_info->reg_info.SynaF34Reflash_BlockData, block, &Config_Data[j*block]);
        //c) issue write
        touch_i2c_write_byte(chip_info->client, chip_info->reg_info.SynaF34_FlashControl, 0x06);
        //d) wait attn
        ret = checkFlashState(chip_info);
        if (ret > 0) {
            TPD_INFO("Configuration:The status(Image) of flash data3 is %x, time = %d\n", ret, j);
            return FW_UPDATE_ERROR;
        }
    }

#ifdef UPDATE_DISPLAY_CONFIG
    TPD_INFO("update-----------------display_configuration ------------------update!\n");
    //a)write SynaF34Reflash_BlockNum to access
    buf[0] = 0;
    buf[1] = 0x60;//high 3 bit configuration area select
    TPD_INFO("---------------- buf[0]=0x%x  buf[1]=0x%x \n",buf[0],buf[1]);
    touch_i2c_write_block(chip_info->client, chip_info->reg_info.SynaF34Reflash_BlockNum, 2, buf);

    TPD_INFO("---------------- Disp_Config_Data ---------------------- \n");
    for(j = 0; j < 16 ; j++)
        printk("0x%x  ", Disp_Config_Data[j]);
    TPD_INFO("-------------------------------------- \n");

    for (j = 0; j < disp_configuration; j++) {
        //b) write data
        touch_i2c_write_block(chip_info->client, chip_info->reg_info.SynaF34Reflash_BlockData, block, &Disp_Config_Data[j*block]);
        //c) issue write
        touch_i2c_write_byte(chip_info->client, chip_info->reg_info.SynaF34_FlashControl, 0x06);
        //d) wait attn
        ret = checkFlashState(chip_info);
        if (ret > 0) {
            TPD_INFO("Disp_Configuration:The status(Image) of flash data3 is %x, time = %d\n", ret, j);
            return FW_UPDATE_ERROR;
        }
    }
    TPD_INFO("Disp_Configuration:The status(Image) of flash data3 is %x, time = %d\n", ret, j);
#endif

    TPD_INFO("Firmware && configuration flash over\n");

    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.SynaF34_FlashControl, 0x00);
    msleep(100);

    synaptics_reset(chip_info);
    msleep(200);

    return FW_UPDATE_SUCCESS;
}

static fp_touch_state synaptics_spurious_fp_check(void *chip_data)
{
    int x = 0, y = 0, z = 0, err_count = 0;
    int ret = 0, TX_NUM = 0, RX_NUM = 0;
    int16_t temp_data = 0, delta_data = 0;
    uint8_t raw_data[1800] = {0};
    fp_touch_state fp_touch_state = FINGER_PROTECT_TOUCH_UP;

    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;
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
    touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE + 3, TX_NUM * SPURIOUS_FP_RX_NUM * 2, raw_data); //read baseline data
    for (x = 1; x < TX_NUM - 2; x++) {
        TPD_DEBUG_NTAG("[%2d]: ", x);
        for (y = 1; y < SPURIOUS_FP_RX_NUM; y++) {
            z = SPURIOUS_FP_RX_NUM * x + y;
            temp_data = (raw_data[z * 2 + 1] << 8) | raw_data[z * 2];
            delta_data = temp_data - chip_info->spuri_fp_data[z];
            TPD_DEBUG_NTAG("%4d, ", delta_data);
            if (delta_data > SPURIOUS_FP_LIMIT) {
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
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;

    touch_i2c_write_byte(chip_info->client, 0xff, 0x03);
    ret = touch_i2c_read_byte(chip_info->client, 0x00);
    TPD_INFO("touch key int_key code = %d\n",ret);

    if (ret & 0x01)
        SET_BIT(bitmap_result, BIT_MENU);
    if (ret & 0x02)
        SET_BIT(bitmap_result, BIT_BACK);

    touch_i2c_write_byte(chip_info->client, 0xff, 0x00);
    return bitmap_result;
}

static void synaptics_finger_proctect_data_get(void * chip_data)
{
    int ret = 0, x = 0, y = 0, z = 0;
    uint8_t *raw_data = NULL;
    static uint8_t retry_time = 3;
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;

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

    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.F54_ANALOG_DATA_BASE + 3, TX_NUM * SPURIOUS_FP_RX_NUM * 2, raw_data);     //read data
    if (ret < 0) {
        if (retry_time) {
            TPD_INFO("%s touch_i2c_read_block error\n",__func__);
            retry_time--;
            goto RE_TRY;
        }
    }

    for (x = 0; x < TX_NUM; x++) {
        printk("[%2d]: ", x);
        for (y = 0; y < SPURIOUS_FP_RX_NUM; y++) {
            z = SPURIOUS_FP_RX_NUM * x + y;
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

static void synaptics_write_ps_status(void *chip_data, int ps_status)
{
    int ret = 0;
    int tmp = 0;
    struct chip_data_s4322 *chip_info = (struct chip_data_s4322 *)chip_data;
    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x4);
    if (ret < 0) {
        TPD_INFO("%s: failed for page select\n", __func__);
        return ;
    }

    tmp = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.F51_CUSTOM_CTRL48);
    if (ps_status == 1) {
        tmp |= 0x04;    //bit2:1 ps near
    } else {
        tmp &= 0xfB;    //bit2:0 ps far
    }
    ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.F51_CUSTOM_CTRL48, tmp);
    if (ret < 0) {
        TPD_INFO("write ps_status error\n");
        return;
    }
    TPD_INFO("chip_info->reg_info.F51_CUSTOM_CTRL48=0x%x  write ps_status value 0x%x ok, ps %s \n",
              chip_info->reg_info.F51_CUSTOM_CTRL48, tmp, ps_status ? "near" : "far");

    ret = touch_i2c_write_byte(chip_info->client, 0xff, 0x0);
    if (ret < 0) {
        TPD_INFO("%s: failed for page select\n", __func__);
    }
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
    .spurious_fp_check= synaptics_spurious_fp_check,
    .finger_proctect_data_get = synaptics_finger_proctect_data_get,
    .exit_esd_mode = synaptics_exit_esd_mode,
    .write_ps_status = synaptics_write_ps_status,
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
};

static int synaptics_tp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
#ifdef CONFIG_SYNAPTIC_RED
    struct remotepanel_data *premote_data = NULL;
#endif

    struct chip_data_s4322 *chip_info;
    struct touchpanel_data *ts = NULL;
    int ret = -1;

    TPD_INFO("%s  is called\n", __func__);
    TPD_INFO("i2c address = 0x%x ,modify i2c address = 0x20\n", client->addr);
    client->addr = 0x20;

    //step1:Alloc chip_info
    chip_info = kzalloc(sizeof(struct chip_data_s4322), GFP_KERNEL);
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
    ts->tp_resume_order = LCD_TP_RESUME;
    ts->skip_suspend_operate = true;

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
    { .compatible = "synaptics-s4322",},
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
