/***********************************************************
* Description : OnePlus touchpanel driver
* 
* File		  : synaptics_s3320.h
*
* Function	  : third party interface
* 
* Source	  : provide by synaptics
*
* Version	  : V1.0
*
***********************************************************/
#ifndef SYNAPTICS_H_S3320
#define SYNAPTICS_H_S3320

/*********PART1:Head files**********************/
#include <linux/i2c.h>
#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

#include "../synaptics_common.h"
#ifdef CONFIG_SYNAPTIC_RED
#include "../synaptics_touch_panel_remote.h"
#endif

/*********PART2:Define Area**********************/
#define ENABLE_UNICODE  0x40
#define ENABLE_VEE      0x20
#define ENABLE_CIRCLE   0x08
#define ENABLE_SWIPE    0x02
#define ENABLE_DTAP     0x01

#define UNICODE_DETECT  0x0b
#define VEE_DETECT      0x0a
#define CIRCLE_DETECT   0x08
#define SWIPE_DETECT    0x07
#define DTAP_DETECT     0x03

#define RESET_TO_NORMAL_TIME 80    /*Sleep time after reset*/

#define SPURIOUS_FP_LIMIT 60
#define SPURIOUS_FP_RX_NUM 9
#define SPURIOUS_FP_TX_NUM 9
#define SPURIOUS_FP_BASE_DATA_RETRY 10

#define I2C_ERROR_MAX_TIME 5

//Self Capacitance key test limite
#define MENU_LOW_LIMITE  1630
#define MENU_HIGH_LIMITE 3803

#define BACK_LOW_LIMITE 3016
#define BACK_HIGH_LIMITE 7039

#define LIMIT_DOZE  3900
#define LIMIT_DOZE_LOW  200

#define COMMAND_FORCE_UPDATE 4

/*********PART3:Struct Area**********************/
struct synaptics_register
{
    uint8_t F01_RMI_QUERY_BASE;
    uint8_t F01_RMI_CMD_BASE;
    uint8_t F01_RMI_CTRL_BASE;
    uint8_t F01_RMI_DATA_BASE;

    uint8_t F01_RMI_QUERY11;
    uint8_t F01_RMI_DATA01;
    uint8_t F01_RMI_CMD00;
    uint8_t F01_RMI_CTRL00;
    uint8_t F01_RMI_CTRL01;
    uint8_t F01_RMI_CTRL02;

    uint8_t F12_2D_QUERY_BASE;
    uint8_t F12_2D_CMD_BASE;
    uint8_t F12_2D_CTRL_BASE;
    uint8_t F12_2D_DATA_BASE;

    uint8_t F12_2D_CTRL08;
    uint8_t F12_2D_CTRL20;
    uint8_t F12_2D_CTRL23;
    uint8_t F12_2D_CTRL27;
    uint8_t F12_2D_CTRL32;
    uint8_t F12_2D_DATA04;
    uint8_t F12_2D_DATA15;
    uint8_t F12_2D_DATA38;
    uint8_t F12_2D_DATA39;
    uint8_t F12_2D_CMD00;

    uint8_t F34_FLASH_QUERY_BASE;
    uint8_t F34_FLASH_CMD_BASE;
    uint8_t F34_FLASH_CTRL_BASE;
    uint8_t F34_FLASH_DATA_BASE;

    uint8_t SynaF34_FlashControl;
    uint8_t SynaF34Reflash_BlockNum;
    uint8_t SynaF34Reflash_BlockData;
    uint8_t SynaF34ReflashQuery_BootID;
    uint8_t SynaF34ReflashQuery_FlashPropertyQuery;
    uint8_t SynaF34ReflashQuery_FirmwareBlockSize;
    uint8_t SynaF34ReflashQuery_FirmwareBlockCount;
    uint8_t SynaF34ReflashQuery_ConfigBlockSize;
    uint8_t SynaF34ReflashQuery_ConfigBlockCount;

    uint8_t F51_CUSTOM_QUERY_BASE;
    uint8_t F51_CUSTOM_CMD_BASE;
    uint8_t F51_CUSTOM_CTRL_BASE;
    uint8_t F51_CUSTOM_DATA_BASE;

    uint8_t F51_CUSTOM_CTRL00;
    uint8_t F51_CUSTOM_CTRL31;
    uint8_t F51_CUSTOM_CTRL42;
    uint8_t F51_CUSTOM_CTRL50;
    uint8_t F51_CUSTOM_DATA;

    uint8_t F54_ANALOG_QUERY_BASE;
    uint8_t F54_ANALOG_COMMAND_BASE;
    uint8_t F54_ANALOG_CONTROL_BASE;
    uint8_t F54_ANALOG_DATA_BASE;
    uint8_t F54_ANALOG_CTRL123;

    uint8_t F55_SENSOR_CTRL01;
    uint8_t F55_SENSOR_CTRL02;
};

struct chip_data_s3320 {
    uint32_t *p_tp_fw;
    tp_dev tp_type;
    struct i2c_client *client;
    struct synaptics_proc_operations *syna_ops; /*synaptics func provide to synaptics common driver*/
#ifdef CONFIG_SYNAPTIC_RED
    int    enable_remote;                       /*Redremote connect state*/
    struct remotepanel_data *premote_data;
#endif/*CONFIG_SYNAPTIC_RED*/
    struct synaptics_register     reg_info;
    struct hw_resource          *hw_res;
    int16_t *spuri_fp_data;
    struct spurious_fp_touch *p_spuri_fp_touch;
};

#endif
