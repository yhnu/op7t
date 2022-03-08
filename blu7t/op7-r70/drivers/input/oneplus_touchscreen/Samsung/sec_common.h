/***********************************************************
* Description : OnePlus touchpanel driver
* 
* File		  : sec_common.h 
*
* Function	  : third party interface
* 
* Source	  : provide by LSI
*
* Version	  : V1.0
*
***********************************************************/
#ifndef SEC_H
#define SEC_H

/*********PART1:Head files**********************/
#include <linux/firmware.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/time.h>

#include "../touchpanel_common.h"

#define I2C_BURSTMAX                        (64)
/*********PART2:Define Area**********************/
struct sec_testdata{
    int TX_NUM;
    int RX_NUM;
    int fd;
    int irq_gpio;
    uint64_t  TP_FW;
    const struct firmware *fw;
    uint64_t test_item;
};

struct sec_test_header {
    uint32_t magic1;
    uint32_t magic2;
    uint64_t test_item;
};

enum {
    LIMIT_TYPE_NO_DATA          = 0x00,     //means no limit data
    LIMIT_TYPE_CERTAIN_DATA     = 0x01,     //means all nodes limit data is a certain data
    LIMIT_TYPE_EACH_NODE_DATA   = 0x02,     //means all nodes have it's own limit
    LIMIT_TYPE_INVALID_DATA     = 0xFF,     //means wrong limit data type
};

//test item
enum {
    TYPE_ERROR                              = 0x00,
    TYPE_MUTUAL_RAW_OFFSET_DATA_SDC         = 0x01,
    TYPE_MUTUAL_RAW_DATA                    = 0x02,
    TYPE_SELF_RAW_OFFSET_DATA_SDC           = 0x03,
    TYPE_MUTU_RAW_NOI_P2P                   = 0x04,
    TYPE_MAX                                = 0xFF,
};

struct sec_test_item_header {
    uint32_t    item_magic;
    uint32_t    item_size;
    uint16_t    item_bit;
    uint16_t    item_limit_type;
    uint32_t    top_limit_offset;
    uint32_t    floor_limit_offset;
    uint32_t    para_num;
};

/*********PART3:Struct Area**********************/
struct sec_proc_operations {
    void (*auto_test)    (struct seq_file *s, void *chip_data, struct sec_testdata *sec_testdata);
    void (*calibrate)    (struct seq_file *s, void *chip_data);
    void (*verify_calibration)    (struct seq_file *s, void *chip_data);
	void (*calibration_data)      (struct seq_file *s, void *chip_data);
};

/*********PART4:function declare*****************/
int sec_create_proc(struct touchpanel_data *ts, struct sec_proc_operations *sec_ops);
void sec_flash_proc_init(struct touchpanel_data *ts, const char *name);
void sec_limit_read(struct seq_file *s, struct touchpanel_data *ts);
void sec_raw_device_init(struct touchpanel_data *ts);

#endif
