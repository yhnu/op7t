/***********************************************************
* Description : OnePlus touchpanel driver
* 
* File		  : synaptics_firmware_v2.h
*
* Function	  : third party interface
* 
* Source	  : provide by synaptics
*
* Version	  : V1.0
*
***********************************************************/
#ifndef SYNAPTICS_FIRMWARE_V2_H
#define SYNAPTICS_FIRMWARE_V2_H

/*********PART1:Head files**********************/
#include <linux/firmware.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/time.h>

#define IMAGE_FILE_MAGIC_VALUE 0x4818472b
#define FLASH_AREA_MAGIC_VALUE 0x7c05e516

#define BOOT_CONFIG_ID "BOOT_CONFIG"
#define APP_CODE_ID "APP_CODE"
#define APP_CONFIG_ID "APP_CONFIG"
#define DISP_CONFIG_ID "DISPLAY"

struct app_config_header {
    unsigned short magic_value[4];
    unsigned char checksum[4];
    unsigned char length[2];
    unsigned char build_id[4];
    unsigned char customer_config_id[16];
};

struct area_descriptor {
    unsigned char magic_value[4];
    unsigned char id_string[16];
    unsigned char flags[4];
    unsigned char flash_addr_words[4];
    unsigned char length[4];
    unsigned char checksum[4];
};

struct block_data_v2 {
    const unsigned char *data;
    unsigned int size;
    unsigned int flash_addr;
};

struct image_info {
    struct block_data_v2 boot_config;
    struct block_data_v2 app_firmware;
    struct block_data_v2 app_config;
    struct block_data_v2 disp_config;
};

struct image_header_v2 {
    unsigned char magic_value[4];
    unsigned char num_of_areas[4];
};

struct boot_config {
    union {
        unsigned char i2c_address;
        struct {
            unsigned char cpha:1;
            unsigned char cpol:1;
            unsigned char word0_b2__7:6;
        } __packed;
    };
    unsigned char attn_polarity:1;
    unsigned char attn_drive:2;
    unsigned char attn_pullup:1;
    unsigned char word0_b12__14:3;
    unsigned char used:1;
    unsigned short customer_part_id;
    unsigned short boot_timeout;
    unsigned short continue_on_reset:1;
    unsigned short word3_b1__15:15;
} __packed;

static inline unsigned int le2_to_uint(const unsigned char *src)
{
    return (unsigned int)src[0] +
            (unsigned int)src[1] * 0x100;
}

static inline unsigned int le4_to_uint(const unsigned char *src)
{
    return (unsigned int)src[0] +
            (unsigned int)src[1] * 0x100 +
            (unsigned int)src[2] * 0x10000 +
            (unsigned int)src[3] * 0x1000000;
}

#endif
