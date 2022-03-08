/***********************************************************
* Description : OnePlus touchpanel driver
* 
* File		  : synaptics_common.h
*
* Function	  : third party interface
* 
* Source	  : provide by synaptics
*
* Version	  : V1.0
*
***********************************************************/
#ifndef SYNAPTICS_H
#define SYNAPTICS_H
#define CONFIG_SYNAPTIC_RED

/*********PART1:Head files**********************/
#include <linux/firmware.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/time.h>

#include "../touchpanel_common.h"
#include "synaptics_firmware_v2.h"

/*********PART2:Define Area**********************/
#define SYNAPTICS_RMI4_PRODUCT_ID_SIZE 10
#define SYNAPTICS_RMI4_PRODUCT_INFO_SIZE 2

#define DiagonalUpperLimit  1100
#define DiagonalLowerLimit  900

#define MAX_RESERVE_SIZE 4
#define MAX_LIMIT_NAME_SIZE 16

#define Limit_MagicNum1     0x494D494C
#define Limit_MagicNum2     0x474D4954
#define Limit_MagicNum2_V2  0x32562D54


/*********PART3:Struct Area**********************/
struct image_header {
    /* 0x00 - 0x0f */
    unsigned char checksum[4];
    unsigned char reserved_04;
    unsigned char reserved_05;
    unsigned char options_firmware_id:1;
    unsigned char options_contain_bootloader:1;
    /* only available in s4322 , reserved in other, begin*/
    unsigned char options_guest_code:1;
    unsigned char options_tddi:1;
    unsigned char options_reserved:4;
    /* only available in s4322 , reserved in other ,  end*/
    unsigned char bootloader_version;
    unsigned char firmware_size[4];
    unsigned char config_size[4];
    /* 0x10 - 0x1f */
    unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE];
    unsigned char package_id[2];
    unsigned char package_id_revision[2];
    unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
    /* 0x20 - 0x2f */
    /* only available in s4322 , reserved in other, begin*/
    unsigned char bootloader_addr[4];
    unsigned char bootloader_size[4];
    unsigned char ui_addr[4];
    unsigned char ui_size[4];
    /* only available in s4322 , reserved in other ,  end*/
    /* 0x30 - 0x3f */
    unsigned char ds_id[16];
    /* 0x40 - 0x4f */
    /* only available in s4322 , reserved in other, begin*/
    union {
        struct {
            unsigned char dsp_cfg_addr[4];
            unsigned char dsp_cfg_size[4];
            unsigned char reserved_48_4f[8];
          };
    };
    /* only available in s4322 , reserved in other ,  end*/
    /* 0x50 - 0x53 */
    unsigned char firmware_id[4];
};

struct image_header_data {
    bool contains_firmware_id;
    unsigned int firmware_id;
    unsigned int checksum;
    unsigned int firmware_size;
    unsigned int config_size;
    /* only available in s4322 , reserved in other, begin*/
    unsigned int disp_config_offset;
    unsigned int disp_config_size;
    unsigned int bootloader_offset;
    unsigned int bootloader_size;
    /* only available in s4322 , reserved in other ,  end*/
    unsigned char bootloader_version;
    unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
    unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
};

struct limit_block {
    char name[MAX_LIMIT_NAME_SIZE];
    int mode;
    int reserve[MAX_RESERVE_SIZE]; /*16*/
    int size;
    int16_t data;
};

struct limit_info {
    unsigned int magic1;
    unsigned int magic2;
    unsigned int count;
};

struct test_header {
    unsigned int magic1;
    unsigned int magic2;
    unsigned int withCBC;
    unsigned int array_limit_offset;
    unsigned int array_limit_size;
    unsigned int array_limitcbc_offset;
    unsigned int array_limitcbc_size;
};

struct syna_testdata{
    int TX_NUM;
    int RX_NUM;
    int fd;
    int irq_gpio;
    int key_TX;
    int key_RX;
    uint64_t  TP_FW;
    const struct firmware *fw;
};

//import from "android/bootable/bootloader/lk/platform/msm_shared/include/msm_panel.h"
enum {
    OP16037JDI_R63452_1080P_CMD_PANEL = 13,
    OP16037SAMSUNG_S6E3FA5_1080P_CMD_PANEL = 14,
    UNKNOWN_PANEL
};

struct synaptics_proc_operations {
    void (*auto_test)    (struct seq_file *s, void *chip_data, struct syna_testdata *syna_testdata);
};

void synaptics_limit_read(struct seq_file *s, struct touchpanel_data *ts);
int  synaptics_create_proc(struct touchpanel_data *ts, struct synaptics_proc_operations *syna_ops);
void synaptics_parse_header(struct image_header_data *header, const unsigned char *fw_image);
int synaptics_parse_header_v2(struct image_info *image_info, const unsigned char *fw_image);
int synaptics_get_limit_data(char *type, const unsigned char *fw_image);

#endif
