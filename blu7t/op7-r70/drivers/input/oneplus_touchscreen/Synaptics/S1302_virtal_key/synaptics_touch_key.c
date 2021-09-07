#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>

#include <linux/regulator/consumer.h>
#include <linux/firmware.h>

#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/machine.h>

#include <linux/kthread.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/task_work.h>

#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/time.h>

#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

#include <linux/input/mt.h>

/*------------------------------------------------Global Define--------------------------------------------*/
#define PAGESIZE 512
#define VKNUMBER 3
#define TP_TEST_ENABLE 1

#undef LCD_TRIGGER_KEY_FORCE_CAL
#define SUPPORT_TP_SLEEP_MODE
#define TYPE_B_PROTOCOL      //Multi-finger operation
#define TP_FW_NAME_MAX_LEN 128

/******************for Red function*****************/
#undef CONFIG_SYNAPTIC_RED

#define TPD_DEVICE "synaptics-s1302"
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)
#define TPD_DEBUG(a, arg...)\
    do{\
        if (tp_debug)\
        pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

/*---------------------------------------------Global Variable----------------------------------------------*/
static int baseline_ret = 0;
static unsigned int is_suspend = 0;
static int16_t delta_baseline[30][30];

static unsigned int tp_debug = 0;
static int force_update = 0;
static int key_reverse = 0;
static struct synaptics_ts_data *ts_g = NULL;
int test_err = 0;

/*-----------------------------------------Global Registers----------------------------------------------*/
static unsigned short SynaF34DataBase;
static unsigned short SynaF34QueryBase;
static unsigned short SynaF01DataBase;
static unsigned short SynaF01CommandBase;

static unsigned short SynaF34Reflash_BlockNum;
static unsigned short SynaF34Reflash_BlockData;
static unsigned short SynaF34ReflashQuery_BootID;
static unsigned short SynaF34ReflashQuery_FlashPropertyQuery;
static unsigned short SynaF34ReflashQuery_FirmwareBlockSize;
static unsigned short SynaF34ReflashQuery_FirmwareBlockCount;
static unsigned short SynaF34ReflashQuery_ConfigBlockSize;
static unsigned short SynaF34ReflashQuery_ConfigBlockCount;

static unsigned short SynaFirmwareBlockSize;
static unsigned short SynaF34_FlashControl;

static int F01_RMI_QUERY_BASE;
static int F01_RMI_CMD_BASE;

static int F01_RMI_DATA_BASE;

static int F11_2D_QUERY_BASE;
static int F11_2D_CMD_BASE;
static int F11_2D_CTRL_BASE;
static int F11_2D_DATA_BASE;

static int F34_FLASH_QUERY_BASE;
static int F34_FLASH_CMD_BASE;
static int F34_FLASH_CTRL_BASE;
static int F34_FLASH_DATA_BASE;

static int F51_CUSTOM_QUERY_BASE;
static int F51_CUSTOM_CMD_BASE;
static int F51_CUSTOM_CTRL_BASE;
static int F51_CUSTOM_DATA_BASE;

static int F01_RMI_CMD00;
static int F01_RMI_CTRL00;
static int F01_RMI_CTRL01;

#if TP_TEST_ENABLE
static int synaptics_read_register_map_page1(struct synaptics_ts_data *ts);
static int F54_ANALOG_QUERY_BASE;//0x73
static int F54_ANALOG_COMMAND_BASE;//0x72
static int F54_ANALOG_CONTROL_BASE;//0x0d
static int F54_ANALOG_DATA_BASE;//0x00
#endif

int KEY_FW;
static struct manufacture_info touch_key_info;
extern void input_report_key_reduce(struct input_dev *dev, unsigned int code, int value);
/*------------------------------------------Fuction Declare----------------------------------------------*/
static int synaptics_ts_resume(struct device *dev);
static int synaptics_ts_suspend(struct device *dev);
static int synaptics_ts_remove(struct i2c_client *client);
static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int synapitcs_ts_update(struct i2c_client *client, const uint8_t *data, uint32_t data_len, bool force);

static int synaptics_rmi4_i2c_read_block(struct i2c_client* client,
        unsigned char addr, unsigned short length, unsigned char *data);

static int synaptics_rmi4_i2c_write_block(struct i2c_client* client,
        unsigned char addr, unsigned short length, unsigned char const *data);

static int synaptics_rmi4_i2c_read_byte(struct i2c_client* client,
        unsigned char addr);

static int synaptics_rmi4_i2c_write_byte(struct i2c_client* client,
        unsigned char addr, unsigned char data);

static int synaptics_rmi4_i2c_read_word(struct i2c_client* client,
        unsigned char addr);

static int synaptics_init_panel(struct synaptics_ts_data *ts);
static void checkCMD(void);

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif
static int synaptics_soft_reset(struct synaptics_ts_data *ts);
static void synaptics_hard_reset(struct synaptics_ts_data *ts);

/*-------------------------------Using Struct----------------------------------*/
static const struct i2c_device_id synaptics_ts_id[] = {
    { TPD_DEVICE, 0 },
    { }
};

static struct of_device_id synaptics_match_table[] = {
    { .compatible = TPD_DEVICE, },
    { },
};

static const struct dev_pm_ops synaptic_pm_ops = {
#ifdef CONFIG_PM
    .suspend = NULL,
    .resume = NULL,
#else
    .suspend = NULL,
    .resume = NULL,
#endif
};

static struct workqueue_struct *speedup_resume_wq = NULL;
static void speedup_synaptics_resume(struct work_struct *work);

static struct i2c_driver tc_i2c_driver = {
    .probe        = synaptics_ts_probe,
    .remove        = synaptics_ts_remove,
    .id_table    = synaptics_ts_id,
    .driver = {
        //        .owner  = THIS_MODULE,
        .name    = TPD_DEVICE,
        .of_match_table = synaptics_match_table,
        .pm = &synaptic_pm_ops,
    },
};

struct synaptics_ts_data {
    struct i2c_client *client;
    struct mutex mutex;
    int irq;
    int irq_gpio;
    int reset_gpio;
    int en3v_gpio;
    int enable_remote;
    int boot_mode;
    int pre_btn_state;
    int is_suspended;
    uint32_t irq_flags;
    struct work_struct  work;
    struct work_struct speed_up_work;
#ifdef LCD_TRIGGER_KEY_FORCE_CAL
    struct work_struct judge_key_status_work ;
#endif
    struct input_dev *input_dev;
#if defined(CONFIG_FB)
    struct notifier_block fb_notif;
#endif

    /******power*******/
    struct regulator *vdd_2v8;
    struct regulator *vcc_i2c_1v8;

    /*pinctrl******/
    struct device                        *dev;
    struct pinctrl                         *pinctrl;
    struct pinctrl_state                 *pinctrl_state_active;
    struct pinctrl_state                 *pinctrl_state_suspend;

    /*******for FW update*******/
    bool suspended;
    bool loading_fw;
    char fw_name[TP_FW_NAME_MAX_LEN];
    char test_limit_name[TP_FW_NAME_MAX_LEN];
    char fw_id[12];
    char manu_name[12];
};

#define SYNAPTICS_RMI4_PRODUCT_ID_SIZE 10
#define SYNAPTICS_RMI4_PRODUCT_INFO_SIZE 2
struct image_header {
    /* 0x00 - 0x0f */
    unsigned char checksum[4];
    unsigned char reserved_04;
    unsigned char reserved_05;
    unsigned char options_firmware_id:1;
    unsigned char options_contain_bootloader:1;
    unsigned char options_reserved:6;
    unsigned char bootloader_version;
    unsigned char firmware_size[4];
    unsigned char config_size[4];
    /* 0x10 - 0x1f */
    unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE];
    unsigned char package_id[2];
    unsigned char package_id_revision[2];
    unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
    /* 0x20 - 0x2f */
    unsigned char reserved_20_2f[16];
    /* 0x30 - 0x3f */
    unsigned char ds_id[16];
    /* 0x40 - 0x4f */
    unsigned char ds_info[10];
    unsigned char reserved_4a_4f[6];
    /* 0x50 - 0x53 */
    unsigned char firmware_id[4];
};

struct image_header_data {
    bool contains_firmware_id;
    unsigned int firmware_id;
    unsigned int checksum;
    unsigned int firmware_size;
    unsigned int config_size;
    unsigned char bootloader_version;
    unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
    unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
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

static unsigned int extract_uint_le(const unsigned char *ptr)
{
    return (unsigned int)ptr[0] +
        (unsigned int)ptr[1] * 0x100 +
        (unsigned int)ptr[2] * 0x10000 +
        (unsigned int)ptr[3] * 0x1000000;
}

static int tc_hw_pwron(struct synaptics_ts_data *ts)
{
    int rc = 0;

    //enable the 2v8 power
    if (!IS_ERR(ts->vdd_2v8)) {
        rc = regulator_enable(ts->vdd_2v8);
        if (rc) {
            dev_err(&ts->client->dev,
                    "Regulator vdd enable failed rc = %d\n", rc);
        }
    }

    if (ts->en3v_gpio > 0) {
        TPD_INFO("synaptics:enable the en3v_gpio\n");
        gpio_direction_output(ts->en3v_gpio, 1);
    }

    if (!IS_ERR(ts->vcc_i2c_1v8)) {
        rc = regulator_enable(ts->vcc_i2c_1v8);
        if (rc) {
            dev_err(&ts->client->dev, "Regulator vcc_i2c enable failed rc = %d\n", rc);
        }
    }

    if (ts->reset_gpio > 0) {
        TPD_DEBUG("synaptics:enable the reset_gpio\n");
        gpio_direction_output(ts->reset_gpio, 1);
    }
    msleep(80); //mingqiang.guo@phone.bsp, 2016/7/7 modify synaptics tp vender need delay 80ms
    return rc;
}

static int tc_hw_pwroff(struct synaptics_ts_data *ts)
{
    int rc = 0;
    if (ts->reset_gpio > 0) {
        TPD_INFO("synaptics:disable the reset_gpio\n");
        gpio_direction_output(ts->reset_gpio, 0);
    }

    if (!IS_ERR(ts->vcc_i2c_1v8)) {
        rc = regulator_disable(ts->vcc_i2c_1v8);
        if (rc) {
            dev_err(&ts->client->dev, "Regulator vcc_i2c enable failed rc = %d\n", rc);
            return rc;
        }
    }

    if (!IS_ERR(ts->vdd_2v8)) {
        rc = regulator_disable(ts->vdd_2v8);
        if (rc) {
            dev_err(&ts->client->dev, "Regulator vdd disable failed rc = %d\n", rc);
            return rc;
        }
    }

    if (ts->en3v_gpio > 0) {
        TPD_INFO("synaptics:disable  the en3v_gpio\n");
        gpio_direction_output(ts->en3v_gpio, 0);
    }
    return rc;
}

static int tc_power(struct synaptics_ts_data *ts, unsigned int on)
{
    int ret;
    if (on)
        ret = tc_hw_pwron(ts);
    else
        ret = tc_hw_pwroff(ts);

    return ret;
}

static int synaptics_read_register_map(struct synaptics_ts_data *ts)
{
    uint8_t buf[4];
    int ret;
    memset(buf, 0, sizeof(buf));
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
    if (ret < 0) {
        TPD_INFO("synaptics_read_register_map: failed for page select\n");
        return -1;
    }
    ret = synaptics_rmi4_i2c_read_block(ts->client, 0xDD, 4, &(buf[0x0]));
    if (ret < 0) {
        TPD_INFO("failed for page select!\n");
        return -1;
    }

    F11_2D_QUERY_BASE = buf[0];
    F11_2D_CMD_BASE = buf[1];
    F11_2D_CTRL_BASE = buf[2];
    F11_2D_DATA_BASE = buf[3];

    TPD_INFO("F11_2D_QUERY_BASE = %x \n \
            F11_2D_CMD_BASE  = %x \n\
            F11_2D_CTRL_BASE    = %x \n\
            F11_2D_DATA_BASE    = %x \n\
            ", F11_2D_QUERY_BASE, F11_2D_CMD_BASE, F11_2D_CTRL_BASE, F11_2D_DATA_BASE);


    ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE3, 4, &(buf[0x0]));
    F01_RMI_QUERY_BASE = buf[0];
    F01_RMI_CMD_BASE = buf[1];
    F01_RMI_CTRL00 = buf[2];
    F01_RMI_DATA_BASE = buf[3];
    TPD_DEBUG("F01_RMI_QUERY_BASE = %x \n\
            F01_RMI_CMD_BASE  = %x \n\
            F01_RMI_CTRL00    = %x \n\
            F01_RMI_DATA_BASE    = %x \n\
            ", F01_RMI_QUERY_BASE, F01_RMI_CMD_BASE, F01_RMI_CTRL00, F01_RMI_DATA_BASE);

    ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0]));
    F34_FLASH_QUERY_BASE = buf[0];
    F34_FLASH_CMD_BASE = buf[1];
    F34_FLASH_CTRL_BASE = buf[2];
    F34_FLASH_DATA_BASE = buf[3];
    TPD_DEBUG("F34_FLASH_QUERY_BASE = %x \n\
            F34_FLASH_CMD_BASE    = %x \n\
            F34_FLASH_CTRL_BASE    = %x \n\
            F34_FLASH_DATA_BASE    = %x \n\
            ", F34_FLASH_QUERY_BASE, F34_FLASH_CMD_BASE, F34_FLASH_CTRL_BASE, F34_FLASH_DATA_BASE);

    F01_RMI_CMD00 = F01_RMI_CMD_BASE;
    F01_RMI_CTRL01 = F01_RMI_CTRL00 + 1;

    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x4);
    if (ret < 0) {
        TPD_DEBUG("synaptics_read_register_map: failed for page select\n");
        return -1;
    }
    ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0]));
    F51_CUSTOM_QUERY_BASE = buf[0];
    F51_CUSTOM_CMD_BASE = buf[1];
    F51_CUSTOM_CTRL_BASE = buf[2];
    F51_CUSTOM_DATA_BASE = buf[3];

    TPD_DEBUG("F51_CUSTOM_QUERY_BASE = %x \n\
            F51_CUSTOM_CMD_BASE  = %x \n\
            F51_CUSTOM_CTRL_BASE    = %x \n\
            F51_CUSTOM_DATA_BASE    = %x \n\
            ", F51_CUSTOM_QUERY_BASE, F51_CUSTOM_CMD_BASE, F51_CUSTOM_CTRL_BASE, F51_CUSTOM_DATA_BASE);

#if TP_TEST_ENABLE
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x01);
    if (ret < 0) {
        TPD_INFO("synaptics_read_register_map: failed for page select\n");
        return -1;
    }
    ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0]));
    F54_ANALOG_QUERY_BASE = buf[0];
    F54_ANALOG_COMMAND_BASE = buf[1];
    F54_ANALOG_CONTROL_BASE = buf[2];
    F54_ANALOG_DATA_BASE = buf[3];
    TPD_DEBUG("F54_QUERY_BASE = %x \n\
            F54_CMD_BASE  = %x \n\
            F54_CTRL_BASE    = %x \n\
            F54_DATA_BASE    = %x \n\
            ", F54_ANALOG_QUERY_BASE, F54_ANALOG_COMMAND_BASE, F54_ANALOG_CONTROL_BASE, F54_ANALOG_DATA_BASE);
#endif
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
    return 0;
}

static int synaptics_read_product_id(struct synaptics_ts_data *ts, char *id)
{
    uint8_t buf[5] = {"\n"};
    int ret ;

    memset(buf, 0, sizeof(buf));
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
    if (ret < 0) {
        TPD_INFO("synaptics_read_product_id: failed for page select\n");
        return -1;
    }
    ret = synaptics_rmi4_i2c_read_block(ts->client, 0x83, 5, &(buf[0x0]));
    if (ret < 0) {
        TPD_INFO("synaptics_read_product_id error %s\n", &buf[0]);
        return -1;
    }
    memcpy(id, &buf[0], sizeof(buf));
    TPD_INFO("product id is %s\n", &buf[0]);
    return 0;
}

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
    int ret;

    TPD_DEBUG("%s is called!\n", __func__);
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
    if (ret < 0) {
        TPD_INFO("init_panel failed for page select\n");
        return -1;
    }
    /*device control: normal operation, configur = 1*/
    //chenggang.li @BSP change 0x80 to 0x84, bit2:1 nosleep  bit2:0 sleep
    ret = synaptics_rmi4_i2c_write_byte(ts->client, F01_RMI_CTRL00, 0x80);
    if (ret < 0) {
        msleep(150);
        ret = synaptics_rmi4_i2c_write_byte(ts->client, F01_RMI_CTRL00, 0x80);
        if (ret < 0) {
            TPD_INFO("%s failed for mode select\n", __func__);
        }
    }

    return ret;
}

static int synaptics_enable_interrupt(struct synaptics_ts_data *ts, int enable)
{
    int ret = 0;
    uint8_t abs_status_int;

    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
    if (ret < 0) {
        TPD_DEBUG("%s: select page failed ret = %d\n", __func__,
                ret);
        return -1;
    }
    if (enable) {
        abs_status_int = 0x7f;
        /*clear interrupt bits for previous touch*/
        ret = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA_BASE + 1);
        if (ret < 0) {
            TPD_DEBUG("%s :clear interrupt bits failed\n", __func__);
            return -1;
        }
    } else {
        abs_status_int = 0x0;
    }
    ret = synaptics_rmi4_i2c_write_byte(ts->client, F01_RMI_CTRL01, abs_status_int);
    if (ret < 0) {
        TPD_DEBUG("%s: enable or disable abs \
                interrupt failed, abs_int = %d\n", __func__, abs_status_int);
        return -1;
    }
    return 0;
}

static void int_state(struct synaptics_ts_data *ts)
{
    int ret = -1;
    TPD_INFO("%s %d\n", __func__, __LINE__);
    ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);
    if (ret) {
        TPD_INFO("%s:cannot reset touch panel \n", __func__);
        return;
    }
    msleep(170);

    synaptics_init_panel(ts);
    if (ret < 0) {
        TPD_INFO("%s:to wakeup failed\n", __func__);
    }
    ret = synaptics_enable_interrupt(ts, 1);
    if (ret) {
        TPD_DEBUG("%s:cannot  enable interrupt \n", __func__);
        return;
    }
}

//Added for larger than 32 length read!
static int synaptics_rmi4_i2c_read_block(struct i2c_client* client,
        unsigned char addr, unsigned short length, unsigned char *data)
{
    int retval;
    unsigned char retry;
    unsigned char buf;
    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = &buf,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = data,
        },
    };
    buf = addr & 0xFF;
    for (retry = 0; retry < 2; retry++) {
        if (i2c_transfer(client->adapter, msg, 2) == 2) {
            retval = length;
            break;
        }
        msleep(20);
    }
    if (retry == 2) {
        dev_err(&client->dev,
                "%s: I2C read over retry limit\n",
                __func__);
        //rst_flag_counter = 1;//reset tp
        retval = -5;
    } else {
        //rst_flag_counter = 0;
    }
    return retval;
}

static int synaptics_rmi4_i2c_write_block(struct i2c_client* client,
        unsigned char addr, unsigned short length, unsigned char const *data)
{
    int retval;
    unsigned char retry;
    unsigned char buf[length + 1];
    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = length + 1,
            .buf = buf,
        }
    };

    buf[0] = addr & 0xff;
    memcpy(&buf[1], &data[0], length);

    for (retry = 0; retry < 2; retry++) {
        if (i2c_transfer(client->adapter, msg, 1) == 1) {
            retval = length;
            break;
        }
        msleep(20);
    }
    if (retry == 2) {
        //rst_flag_counter = 1;//rest tp
        retval = -EIO;
    } else {
        //rst_flag_counter = 0;
    }
    return retval;
}

static int synaptics_rmi4_i2c_read_byte(struct i2c_client* client,
        unsigned char addr)
{
    int retval = 0;
    unsigned char buf[2] = {0};
    retval = synaptics_rmi4_i2c_read_block(client, addr, 1, buf);
    if (retval >= 0)
        retval = buf[0]&0xff;
    return retval;
}

static int synaptics_rmi4_i2c_write_byte(struct i2c_client* client,
        unsigned char addr, unsigned char data)
{
    int retval;
    unsigned char data_send = data;
    retval = synaptics_rmi4_i2c_write_block(client, addr, 1, &data_send);
    return retval;
}

static int synaptics_rmi4_i2c_read_word(struct i2c_client* client,
        unsigned char addr)
{
    int retval;
    unsigned char buf[2] = {0};
    retval = synaptics_rmi4_i2c_read_block(client, addr, 2, buf);
    if (retval >= 0)
        retval = buf[1] << 8 | buf[0];
    return retval;
}

static int synaptics_rmi4_i2c_write_word(struct i2c_client* client,
        unsigned char addr, unsigned short data)
{
    int retval;
    unsigned char buf[2] = {data & 0xff, (data >> 8) & 0xff};
    retval = synaptics_rmi4_i2c_write_block(client, addr, 2, buf);
    if (retval >= 0)
        retval = buf[1] << 8 | buf[0];
    return retval;
}

//mingqiang.guo@phone.bsp 2016-1-7 add for key press all the time, can not touch up, need force cal
#ifdef LCD_TRIGGER_KEY_FORCE_CAL
static int key_press_all_the_time = 0;
void judge_key_press_all_the_time(void)
{
    if (ts_g == NULL)
        return;

    if (key_press_all_the_time != 0)
        key_press_all_the_time++;

    if (key_press_all_the_time == 40)
    {
        key_press_all_the_time = 0;
        schedule_work(&(ts_g->judge_key_status_work));
    }
}

static void judge_key_status_work_func(struct work_struct *work)
{
    mutex_lock(&ts_g->mutex);
    TPD_INFO("key_press_all_the_time = %d, detecte  key press all the time when touch lcd, force Cal touch key \n", key_press_all_the_time);
    synaptics_rmi4_i2c_write_byte(ts_g->client, 0xff, 0x01);
    synaptics_rmi4_i2c_write_word(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
    checkCMD();
    synaptics_rmi4_i2c_write_byte(ts_g->client, F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
    checkCMD();
    synaptics_rmi4_i2c_write_byte(ts_g->client, 0xff, 0x00);

    input_report_key(ts_g->input_dev, KEY_MENU, 0);
    input_sync(ts_g->input_dev);
    input_report_key(ts_g->input_dev, KEY_BACK, 0);
    input_sync(ts_g->input_dev);
    mutex_unlock(&ts_g->mutex);
}
#endif
//end

static char log_count = 0;
static void int_key(struct synaptics_ts_data *ts)
{
    int ret;
    int button_key;

    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x02);
    if (ret < 0) {
        TPD_INFO("%s: Failed to change page 2!!\n", __func__);
        return;
    }

    button_key = synaptics_rmi4_i2c_read_byte(ts->client, 0x00);
    if (6 == (++log_count % 12))
        TPD_INFO("%s button_key : %d   pre_btn_state:%d\n", __func__, button_key, ts->pre_btn_state);
    else
        TPD_INFO("%s button_key : %d   pre_btn_state:%d\n", __func__, button_key, ts->pre_btn_state);

    //mingqiang.guo add for two key down, can not report long press, resolve for  electrostatic experiment
    switch(button_key&0x03)
    {
        case 0: //up
            if (ts->pre_btn_state == 1)
            {
                input_report_key_reduce(ts->input_dev, KEY_MENU, 0);
                input_sync(ts->input_dev);
            }
            else if (ts->pre_btn_state == 2)
            {
                input_report_key_reduce(ts->input_dev, KEY_BACK, 0);
                input_sync(ts->input_dev);
            }
            ts->pre_btn_state = 0;
#ifdef LCD_TRIGGER_KEY_FORCE_CAL
            key_press_all_the_time = 0;
#endif
            break;

        case 1: //down
            ts->pre_btn_state = 0x01;
            input_report_key_reduce(ts->input_dev, KEY_MENU, 1);
            input_sync(ts->input_dev);
#ifdef LCD_TRIGGER_KEY_FORCE_CAL
            key_press_all_the_time = 1;
#endif
            break;
        case 2: //down
            ts->pre_btn_state = 0x02;
            input_report_key_reduce(ts->input_dev, KEY_BACK, 1);
            input_sync(ts->input_dev);
#ifdef LCD_TRIGGER_KEY_FORCE_CAL
            key_press_all_the_time = 1;
#endif
            break;

        case 3: //force cal
            TPD_INFO("key code = %d, key press all the time, force Cal touch key\n", button_key & 0x03);
            synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x01);
            synaptics_rmi4_i2c_write_word(ts->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
            checkCMD();
            synaptics_rmi4_i2c_write_byte(ts->client, F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
            checkCMD();
            synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);

            input_report_key(ts->input_dev, KEY_MENU, 0);
            input_sync(ts->input_dev);
            input_report_key(ts->input_dev, KEY_BACK, 0);
            input_sync(ts->input_dev);
#ifdef LCD_TRIGGER_KEY_FORCE_CAL
            key_press_all_the_time = 0;
#endif
            break;
    }

    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
    if (ret < 0) {
        TPD_INFO("%s: Failed to change page 2!!\n", __func__);
        return;
    }
    return;
}

static void synaptics_ts_report(struct synaptics_ts_data *ts)
{
    int ret;
    uint8_t status = 0;
    uint8_t inte = 0;

    if (ts->enable_remote) {
        goto END;
    }
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
    ret = synaptics_rmi4_i2c_read_word(ts->client, 0x13);

    if (ret < 0) {
        TPD_DEBUG("Synaptic:ret = %d\n", ret);
        synaptics_hard_reset(ts);
        goto END;
    }
    status = ret & 0xff;
    inte = (ret & 0x7f00) >> 8;
    if (status) {
        int_state(ts);
    }
    if (inte & 0x10) {
        int_key(ts);
    }
END:
    return;
}

static irqreturn_t synaptics_irq_thread_fn(int irq, void *dev_id)
{
    struct synaptics_ts_data *ts = (struct synaptics_ts_data *)dev_id;
    //TPD_INFO("%s is call \n", __func__);
    mutex_lock(&ts->mutex);
    synaptics_ts_report(ts);
    mutex_unlock(&ts->mutex);
    return IRQ_HANDLED;
}

static int synaptics_input_init(struct synaptics_ts_data *ts)
{
    int ret = 0;

    TPD_DEBUG("%s is called\n", __func__);
    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL) {
        ret = -ENOMEM;
        TPD_INFO("synaptics_ts_probe: Failed to allocate input device\n");
        return ret;
    }
    ts->input_dev->name = TPD_DEVICE;;
    ts->input_dev->dev.parent = &ts->client->dev;
    set_bit(EV_SYN, ts->input_dev->evbit);
    set_bit(EV_KEY, ts->input_dev->evbit);
    set_bit(KEY_BACK, ts->input_dev->keybit);
    set_bit(KEY_MENU, ts->input_dev->keybit);
    //set_bit(KEY_APPSELECT, ts->input_dev->keybit); //set this KEY_MENU will report KEY_APPSELECT
    set_bit(KEY_HOMEPAGE, ts->input_dev->keybit);
    input_set_drvdata(ts->input_dev, ts);

    if (input_register_device(ts->input_dev)) {
        TPD_INFO("%s: Failed to register input device\n", __func__);
        input_unregister_device(ts->input_dev);
        input_free_device(ts->input_dev);
        return -1;
    }

    return 0;
}

/*********************FW Update Func******************************************/
static int synatpitcs_fw_update(struct device *dev, bool force)
{
    struct synaptics_ts_data *ts = dev_get_drvdata(dev);
    const struct firmware *fw = NULL;
    int ret;
    char fw_id_temp[12];
    uint8_t buf[4];
    uint32_t CURRENT_FIRMWARE_ID = 0 ;

    TPD_DEBUG("%s is called\n", __func__);

    if (!ts->client) {
        TPD_INFO("i2c client point is NULL\n");
        return 0;
    }
    ret = request_firmware(&fw, ts->fw_name, dev);
    if (ret < 0) {
        TPD_INFO("Request firmware failed - %s (%d)\n",
                ts->fw_name, ret);
        return ret;
    }
    ret = synapitcs_ts_update(ts->client, fw->data, fw->size, force);
    if (ret < 0) {
        TPD_INFO("FW update not success try again\n");
        ret = synapitcs_ts_update(ts->client, fw->data, fw->size, force);
        if (ret < 0) {
            TPD_INFO("FW update failed twice, quit updating process!\n");
            return ret;
        }
    }
    release_firmware(fw);

    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
    ret = synaptics_rmi4_i2c_read_block(ts->client, F34_FLASH_CTRL_BASE, 4, buf);
    CURRENT_FIRMWARE_ID = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    sprintf(fw_id_temp, "0x%x", CURRENT_FIRMWARE_ID);
    strcpy(ts->fw_id, fw_id_temp);
    synaptics_init_panel(ts);
    synaptics_enable_interrupt(ts, 1);
    input_report_key(ts->input_dev, BTN_TOUCH, 0);
    input_mt_sync(ts->input_dev);
    input_sync(ts->input_dev);

    return 0;
}

static int synaptics_s1302_fw_show(struct seq_file *seq, void *offset)
{
    struct synaptics_ts_data *ts = ts_g;
    seq_printf(seq, "\
            synaptics ic type is %s\n\
            firmware name is %s\n\
            firmware version is %s\n\
            is update firmware state: %s\n", \
            ts->manu_name, ts->fw_name, ts->fw_id, \
            (ts->loading_fw)?("loading..."):("no update"));
    return 0;
}

static ssize_t synaptics_s1302_fw_write(struct file *file, const char __user *page, size_t t, loff_t *lo)
{
    int val = 0;
    TPD_INFO("start update ******* page :%s  ts_g == NULL? %s\n", page, (ts_g == NULL)? "yes":"no");
    if (NULL == ts_g)
        return -EINVAL;

    TPD_INFO("start update ******* fw_name:%s\n", ts_g->fw_name);
    if (t > 2)
        return -EINVAL;

    sscanf(page, "%d", &val);

    if (!val)
        val = force_update;

    disable_irq_nosync(ts_g->irq);
    mutex_lock(&ts_g->mutex);
    ts_g->loading_fw = true;
    synatpitcs_fw_update(ts_g->dev, val);
    ts_g->loading_fw = false;
    mutex_unlock(&ts_g->mutex);
    enable_irq(ts_g->irq);
    force_update = 0;

    return t;
}

static int synaptics_s1302_fw_open(struct inode *inode, struct file *file)
{
    return single_open(file, synaptics_s1302_fw_show, inode->i_private);
}

const struct file_operations proc_firmware_update =
{
    .owner        = THIS_MODULE,
    .open        = synaptics_s1302_fw_open,
    .read        = seq_read,
    .write        = synaptics_s1302_fw_write,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static ssize_t synaptics_s1302_key_reverse_write(struct file *file, const char __user *page, size_t t, loff_t *lo)
{
    int ret = 0;
    char buf[10];

    if (t > 2)
        return t;
    if (copy_from_user(buf, page, t)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return t;
    }

    sscanf(buf, "%d", &ret);
    TPD_INFO("%s key_reverse:%d\n", __func__, ret);
    if ((ret == 0) || (ret == 1))
    {
        key_reverse = ret;
    }

    return t;
}

static int synaptics_s1302_key_reverse_show(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "s1302 menu key in %s\n", key_reverse?("right"):("left"));
    return 0 ;
}

static int synaptics_s1302_key_reverse_open(struct inode *inode, struct file *file)
{
    return single_open(file, synaptics_s1302_key_reverse_show, inode->i_private);
}

const struct file_operations proc_reverse_key =
{
    .owner        = THIS_MODULE,
    .open        = synaptics_s1302_key_reverse_open,
    .read        = seq_read,
    .write      = synaptics_s1302_key_reverse_write,
    .llseek     = seq_lseek,
    .release    = single_release,
};
static int page, address, block;
static int synaptics_s1302_radd_show(struct seq_file *seq, void *offset)
{
    int ret;
    char buffer[256];
    int i;

    struct synaptics_ts_data *ts = ts_g;
    TPD_INFO("%s page = 0x%x, address = 0x%x, block = 0x%x\n", __func__, page, address, block);
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, page);
    ret = synaptics_rmi4_i2c_read_block(ts->client, address, block, buffer);
    for (i = 0; i < block; i++)
    {
        TPD_INFO("buffer[%d] = 0x%x\n", i, buffer[i]);
    }
    seq_printf(seq, "page:0x%x; address:0x%x; buff[%d]:[0]0x%x, [1]0x%x, [2]0x%x, [3]0x%x\n", \
            page, address, block, buffer[0], buffer[1], buffer[2], buffer[3]);

    return 0;
}

static ssize_t synaptics_s1302_radd_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int buf[128];
    int ret, i;
    struct synaptics_ts_data *ts = ts_g;
    int temp_block, wbyte;
    char reg[30];

    ret = sscanf(buffer, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x", \
            &buf[0], &buf[1], &buf[2], &buf[3], &buf[4], &buf[5], &buf[6], &buf[7], &buf[8], &buf[9], \
            &buf[10], &buf[11], &buf[12], &buf[13], &buf[14], &buf[15], &buf[16], &buf[17]);
    for (i = 0;i < ret;i++)
    {
        TPD_INFO("buf[i] = 0x%x, ", buf[i]);
    }
    TPD_INFO("\n");
    page = buf[0];
    address = buf[1];
    temp_block = buf[2];
    wbyte = buf[3];
    if (0xFF == temp_block)//the  mark is to write register else read register
    {
        for (i = 0; i < wbyte; i++)
        {
            reg[i] = (char)buf[4 + i];
        }
        ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, page);
        ret = synaptics_rmi4_i2c_write_block(ts->client, (char)address, wbyte, reg);
        TPD_INFO("%s write page = 0x%x, address = 0x%x\n", __func__, page, address);
        for (i = 0;i < wbyte;i++)
        {
            TPD_INFO("reg = 0x%x\n", reg[i]);
        }
    }
    else
        block = temp_block;

    return count;
}

static int synaptics_s1302_radd_open(struct inode *inode, struct file *file)
{
    return single_open(file, synaptics_s1302_radd_show, inode->i_private);
}
const struct file_operations proc_radd =
{
    .owner      = THIS_MODULE,
    .open       = synaptics_s1302_radd_open,
    .read       = seq_read,
    .write      = synaptics_s1302_radd_write,
    .llseek     = seq_lseek,
    .release    = single_release,
};
static ssize_t synaptics_s1302_reset_write (struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
    int ret, write_flag;
    struct synaptics_ts_data *ts = ts_g;

    if (ts->loading_fw) {
        TPD_INFO("%s FW is updating break!!\n", __func__);
        return count;
    }

    ret = sscanf(buffer, "%x", &write_flag);
    TPD_INFO("%s write %d\n", __func__, write_flag);
    if (1 == write_flag)
    {
        ret = synaptics_soft_reset(ts);
    }
    else if (2 == write_flag)
    {
        synaptics_hard_reset(ts);
    }
    return count;
}

static int synaptics_s1302_reset_show(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "%d\n", test_err);
    return 0 ;
}

static int synaptics_s1302_reset_open(struct inode *inode, struct file *file)
{
    return single_open(file, synaptics_s1302_reset_show, inode->i_private);
}

const struct file_operations proc_reset =
{
    .owner      = THIS_MODULE,
    .open       = synaptics_s1302_reset_open,
    .read       = seq_read,
    .write      = synaptics_s1302_reset_write,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static ssize_t synaptics_s1302_debug_write (struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
    int ret, write_flag;

    ret = sscanf(buffer, "%x", &write_flag);
    TPD_INFO("%s write %d\n", __func__, write_flag);
    tp_debug = write_flag?1:0;
    return count;
}
static int synaptics_s1302_debug_show(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "s1302 debug log is %s!\n", tp_debug?"on":"off");
    return 0 ;
}

static int synaptics_s1302_debug_open(struct inode *inode, struct file *file)
{
    return single_open(file, synaptics_s1302_debug_show, inode->i_private);
}

const struct file_operations proc_debug =
{
    .owner      = THIS_MODULE,
    .open       = synaptics_s1302_debug_open,
    .read       = seq_read,
    .write      = synaptics_s1302_debug_write,
    .llseek     = seq_lseek,
    .release    = single_release,
};

//guomingqiang@phone.bsp 2015-12-17 add for touch key debug node
static ssize_t synaptics_rmi4_baseline_show_s1302(struct device *dev, char *buf, bool savefile)
{
    int ret = 0;
    int x, y;
    int16_t baseline_data = 0;
    uint8_t tmp_l = 0, tmp_h = 0;
    ssize_t num_read_chars = 0;
    uint16_t count = 0;
    int error_count = 0;
    int16_t *baseline_data_test;
    int fd = -1;
    struct timespec   now_time;
    struct rtc_time   rtc_now_time;
    uint8_t  data_buf[64];
    mm_segment_t old_fs;
    const struct firmware *fw = NULL;
    struct test_header *ph = NULL;

    if (!ts_g) {
        num_read_chars += sprintf(&(buf[num_read_chars]), "ts_g is null\n");
        return num_read_chars;
    }

    ret = request_firmware(&fw, ts_g->test_limit_name, dev);
    if (ret < 0) {
        TPD_INFO("Request firmware failed - %s (%d)\n", ts_g->test_limit_name, ret);
        error_count++;
        num_read_chars += sprintf(&(buf[num_read_chars]), "imageid = 0x%x, deviceid = 0x%x\n", KEY_FW, KEY_FW);
        num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, "Request firmware failed");
        return num_read_chars;
    }
    ph = (struct test_header *)(fw->data);

    mutex_lock(&ts_g->mutex);
    disable_irq(ts_g->irq);//disable_irq_nosync(ts_g->client->irq);

    synaptics_read_register_map_page1(ts_g);
    TPD_INFO("synaptics_rmi4_baseline_show_s1302  step 1:select report type 0x03\n");

    if (savefile) {
        getnstimeofday(&now_time);
        rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
        sprintf(data_buf, "/sdcard/tp_key_testlimit_%02d%02d%02d-%02d%02d%02d.csv",
                (rtc_now_time.tm_year + 1900) % 100, rtc_now_time.tm_mon + 1, rtc_now_time.tm_mday,
                rtc_now_time.tm_hour + 8 > 23 ? (rtc_now_time.tm_hour + 8-24) : rtc_now_time.tm_hour + 8, rtc_now_time.tm_min, rtc_now_time.tm_sec);

        old_fs = get_fs();
        set_fs(KERNEL_DS);

        fd = sys_open(data_buf, O_WRONLY | O_CREAT | O_TRUNC, 0);
        if (fd < 0) {
            TPD_INFO("Open log file '%s' failed.\n", data_buf);
            set_fs(old_fs);
        }
    }

    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, F54_ANALOG_DATA_BASE, 0x03);//select report type 0x03
    if (ret < 0) {
        TPD_INFO("read_baseline: i2c_smbus_write_byte_data_s1302 failed \n");
        goto END;
    }

    //baseline_data_test = (uint16_t *)(fw->data + ph->array_limit_offset);
    //for (z = 0; z < 10; z++)
    //{
    //    TPD_DEBUG("baseline_data_test1 = %d \t baseline_data_test2 = %d\n", *(baseline_data_test + z * 2), *(baseline_data_test + z * 2 + 1));
    //}
    baseline_data_test = (uint16_t *)(fw->data + ph->array_limit_offset);

    ret = synaptics_rmi4_i2c_write_word(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update

    checkCMD();
    TPD_DEBUG("forbid Forbid NoiseMitigation oK\n");
    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
    checkCMD();
    TPD_DEBUG("Force Cal oK\n");
    ret = synaptics_rmi4_i2c_write_word(ts_g->client, F54_ANALOG_DATA_BASE + 1, 0x00);//set fifo 00
    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
    checkCMD();

    count = 0;
    for (x = 0; x < 2; x++)
    {
        for (y = 0; y < 5; y++)
        {
            ret = synaptics_rmi4_i2c_read_byte(ts_g->client, F54_ANALOG_DATA_BASE + 3);
            tmp_l = ret&0xff;
            ret = synaptics_rmi4_i2c_read_byte(ts_g->client, F54_ANALOG_DATA_BASE + 3);
            tmp_h = ret&0xff;
            baseline_data = (tmp_h << 8)|tmp_l;
            if (fd >= 0) {
                sprintf(data_buf, "%d, ", baseline_data);
                sys_write(fd, data_buf, strlen(data_buf));
            }
            //TPD_DEBUG("baseline_data is %d\n", baseline_data);
            //TPD_DEBUG("x = %d  y = %d\n", x, y);
            if ((x == 0) && ((y == 3) || (y == 4)))
            {
                TPD_DEBUG("count = %d\n", count);
                if ((baseline_data < *(baseline_data_test + count*2)) || (baseline_data > *(baseline_data_test + count*2 + 1)))
                {
                    //TPD_INFO("TPD error baseline_data[%d][%d] = %d[%d, %d]\n", x, y, baseline_data, *(baseline_data_test + count*2),    *(baseline_data_test + count*2 + 1));
                    num_read_chars += sprintf(&(buf[num_read_chars]), "conut = %d TPD error baseline_data[%d][%d] = %d[%d, %d]\n", count, x, y, baseline_data, *(baseline_data_test + count*2), *(baseline_data_test + count*2 + 1));
                    error_count++;
                    goto END;
                }
                count++;
            }
            else{
                count++;
                continue;
            }
        }
        if (fd >= 0) {
            sys_write(fd, "\n", 1);
        }
        TPD_INFO("\n");
    }

    ret = synaptics_rmi4_i2c_write_word(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x04);
    checkCMD();

END:
    if (fd >= 0) {
        sys_close(fd);
        set_fs(old_fs);
    }
    release_firmware(fw);
    num_read_chars += sprintf(&(buf[num_read_chars]), "imageid = 0x%x, deviceid = 0x%x\n", KEY_FW, KEY_FW);
    num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, error_count?"":"All test passed.");
    //TPD_DEBUG("buf = %s\n", buf);
    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, F54_ANALOG_COMMAND_BASE, 0X02);
    msleep(60);
    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, 0xff, 0x00);
    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, F01_RMI_CMD00, 0x01);
    msleep(150);
    synaptics_init_panel(ts_g);
    synaptics_enable_interrupt(ts_g, 1);
    enable_irq(ts_g->irq);
    TPD_DEBUG("\n\nstep5 reset and open irq complete\n");
    mutex_unlock(&ts_g->mutex);

    return num_read_chars;
}

static int tp_reg_address;
static ssize_t tp_reg_operate_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[PAGESIZE];

    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, 0xff, tp_reg_address >> 8);
    if (ret < 0)
    {
        TPD_DEBUG("%s error %d\n", __func__, __LINE__);
        return -1;
    }

    ret  = synaptics_rmi4_i2c_read_byte(ts_g->client, tp_reg_address & 0x00ff);

    TPD_INFO("read reg = 0x%x is 0x%x\n", tp_reg_address, ret);
    ret = sprintf(page, "read reg = 0x%x is 0x%x\n", tp_reg_address, ret);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

static ssize_t tp_reg_operate_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int ret = 0 ;
    char buf[50];

    int data;

    if (copy_from_user(buf, buffer, count)) {
        TPD_DEBUG("%s: read proc input error.\n", __func__);
        return count;
    }

    sscanf(buf, "0x%x 0x%x ", &tp_reg_address, &data);
    TPD_INFO("%s write  reg = 0x%x data = 0x%x \n", __func__, tp_reg_address, data);

    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, 0xff, tp_reg_address >> 8);
    if (ret < 0)
    {
        TPD_DEBUG("%s error %d\n", __func__, __LINE__);
        return -1;
    }

    data = synaptics_rmi4_i2c_read_byte(ts_g->client, tp_reg_address & 0x00ff);

    TPD_INFO("%s  read data %d\n", __func__, data);

    return count;
}
static ssize_t tp_baseline_test_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    char page[PAGESIZE];

    if (!ts_g)
        return baseline_ret;
    if (baseline_ret == 0) //proc node will auto read two time, but only need tp test one time
    {
        count = synaptics_rmi4_baseline_show_s1302(ts_g->dev, page, 1);
        baseline_ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    }
    else
    {
        baseline_ret = 0;
    }

    return baseline_ret;
}

static ssize_t tp_delta_show(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
    int ret = 0;
    int x, y;
    char * kernel_buf;
    ssize_t num_read_chars = 0;
    uint8_t tmp_l = 0, tmp_h = 0;
    uint16_t count = 0;

    if (!ts_g)
        return 0;
    kernel_buf = kmalloc(4096, GFP_KERNEL);
    if (kernel_buf == NULL)
    {
        TPD_INFO("kmalloc error!\n");
        return 0;
    }
    memset(delta_baseline, 0, sizeof(delta_baseline));

    disable_irq(ts_g->irq);//disable_irq_nosync(ts_g->client->irq);

    synaptics_read_register_map_page1(ts_g);

    //TPD_DEBUG("\nstep 2:report type2 delta image\n");
    memset(delta_baseline, 0, sizeof(delta_baseline));
    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, F54_ANALOG_DATA_BASE, 0x02);//select report type 0x02
    ret = synaptics_rmi4_i2c_write_word(ts_g->client, F54_ANALOG_DATA_BASE + 1, 0x00);//set fifo 00
    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, F54_ANALOG_COMMAND_BASE, 0X01);//get report

    checkCMD();

    count = 0;
    for (x = 0; x < 2; x++) {
        //TPD_INFO("\n[%d]", x);
        //num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "\n[%3d]", x);
        for (y = 0; y < 5; y++) {
            ret = synaptics_rmi4_i2c_read_byte(ts_g->client, F54_ANALOG_DATA_BASE + 3);
            tmp_l = ret & 0xff;
            ret = synaptics_rmi4_i2c_read_byte(ts_g->client, F54_ANALOG_DATA_BASE + 3);
            tmp_h = ret & 0xff;
            delta_baseline[x][y] = (tmp_h << 8) | tmp_l;
            //TPD_INFO("%3d, ", delta_baseline[x][y]);
            if (x == 0 && y == 3)
                num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "menu_key: %d\n", delta_baseline[x][y]);
            if (x == 0 && y == 4)
                num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "back_key: %d\n", delta_baseline[x][y]);
        }
    }
    msleep(60);
    //ret = synaptics_soft_reset(ts_g);
    //delay_qt_ms(60);
    synaptics_enable_interrupt(ts_g, 1);

    enable_irq(ts_g->irq);

    TPD_INFO("num_read_chars = %zd, count = %zd\n", num_read_chars, size);
    num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "%s", "\r\n");
    ret = simple_read_from_buffer(buf, size, ppos, kernel_buf, strlen(kernel_buf));
    kfree(kernel_buf);
    return ret;
}

static ssize_t tp_baseline_show(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
    int ret = 0;
    int x, y;
    char * kernel_buf;
    ssize_t num_read_chars = 0;
    uint8_t tmp_old = 0;
    uint8_t tmp_l = 0, tmp_h = 0;
    uint16_t count = 0;
    int16_t baseline_data[2][5];

    if (is_suspend == 1)
        return count;
    if (!ts_g)
        return count;

    kernel_buf = kmalloc(4096, GFP_KERNEL);
    if (kernel_buf == NULL)
    {
        TPD_INFO("kmalloc error!\n");
        return 0;
    }
    memset(delta_baseline, 0, sizeof(delta_baseline));
    disable_irq(ts_g->irq);
    synaptics_read_register_map_page1(ts_g);
    mutex_lock(&ts_g->mutex);

    TPD_DEBUG("\n wanghao test start\n");
    TPD_DEBUG("\n step 1:select report type 0x03 baseline\n");

    //step 1:check raw capacitance.
    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, F54_ANALOG_DATA_BASE, 0x03);//select report type 0x03
    if (ret < 0) {
        TPD_DEBUG("read_baseline: synaptics_rmi4_i2c_write_byte failed \n");
        //return sprintf(buf, "i2c err!");
    }

    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, F54_ANALOG_CONTROL_BASE + 81, 0x01);
    ret = synaptics_rmi4_i2c_read_byte(ts_g->client, F54_ANALOG_CONTROL_BASE + 86);
    tmp_old = ret & 0xff;
    TPD_DEBUG("ret = %x, tmp_old = %x, tmp_new = %x\n", ret, tmp_old, (tmp_old & 0xef));
    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, F54_ANALOG_CONTROL_BASE + 86, (tmp_old & 0xef));
    ret = synaptics_rmi4_i2c_write_word(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x04);

    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, F54_ANALOG_CONTROL_BASE + 29, 0x01);// Forbid NoiseMitigation
    ret = synaptics_rmi4_i2c_write_word(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
    checkCMD();
    TPD_DEBUG("Forbid NoiseMitigation oK\n");\
    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
    checkCMD();
    TPD_DEBUG("Force Cal oK\n");
    ret = synaptics_rmi4_i2c_write_word(ts_g->client, F54_ANALOG_DATA_BASE + 1, 0x00);//set fifo 00
    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
    checkCMD();
    count = 0;

    for (x = 0;x < 2; x++)
    {
        TPD_DEBUG("\n[%2d]", x);
        num_read_chars += sprintf(kernel_buf + num_read_chars, "\n[%2d]", x);

        for (y = 0; y < 5; y++)
        {
            ret = synaptics_rmi4_i2c_read_byte(ts_g->client, F54_ANALOG_DATA_BASE + 3);
            tmp_l = ret & 0xff;
            ret = synaptics_rmi4_i2c_read_byte(ts_g->client, F54_ANALOG_DATA_BASE + 3);
            tmp_h = ret & 0xff;
            baseline_data[x][y] = (tmp_h << 8) | tmp_l;

            TPD_DEBUG("%d, ", baseline_data[x][y]);
            num_read_chars += sprintf(kernel_buf + num_read_chars, "%5d", baseline_data[x][y]);
        }
    }

    TPD_DEBUG("\n report type2 delta image \n");

    ret = synaptics_rmi4_i2c_write_byte(ts_g->client, F54_ANALOG_COMMAND_BASE, 0X02);
    msleep(60);
    ret = synaptics_soft_reset(ts_g);
    msleep(60);
    synaptics_enable_interrupt(ts_g, 1);
    mutex_unlock(&ts_g->mutex);
    enable_irq(ts_g->irq);
    TPD_DEBUG("\nreport delta image end\n");
    TPD_INFO("num_read_chars = %zd, size = %zd\n", num_read_chars, size);
    num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "%s", "\r\n");
    ret = simple_read_from_buffer(buf, size, ppos, kernel_buf, strlen(kernel_buf));
    kfree(kernel_buf);

    return ret;
}

static const struct file_operations tp_reg_operate_proc_fops = {
    .write = tp_reg_operate_write_func,
    .read = tp_reg_operate_read_func,
    .open = simple_open,
    .owner = THIS_MODULE,
};
static const struct file_operations tp_baseline_test_proc_fops =
{
    .read = tp_baseline_test_read_func,
    .owner = THIS_MODULE,
};

static const struct file_operations tp_delta_data_proc_fops =
{
    .read = tp_delta_show,
    .owner = THIS_MODULE,
};

static const struct file_operations tp_baseline_image_proc_fops =
{
    .read = tp_baseline_show,
    .owner = THIS_MODULE,
};
//guomingqiang@phone.bsp 2015-12-17 add for touch key debug node  end

static int synaptics_s1302_proc(void)
{
    struct proc_dir_entry *proc_entry = 0;

    struct proc_dir_entry *procdir = proc_mkdir("touchkey", NULL);
    //for firmware version
    proc_entry = proc_create_data("fw_update", 0444, procdir, &proc_firmware_update, NULL);
    proc_entry = proc_create_data("key_rep", 0666, procdir, &proc_reverse_key, NULL);
    proc_entry = proc_create_data("radd", 0666, procdir, &proc_radd, NULL);
    proc_entry = proc_create_data("reset", 0666, procdir, &proc_reset, NULL);
    proc_entry = proc_create_data("tp_debug", 0666, procdir, &proc_debug, NULL);

    //guomingqiang@phone.bsp 2015-12-17 add for touch key debug node
    proc_entry = proc_create_data("baseline_test", 0666, procdir, &tp_baseline_test_proc_fops, NULL);
    proc_entry = proc_create_data("tp_delta_data", 0644, procdir, &tp_delta_data_proc_fops, NULL);
    proc_entry = proc_create_data("tp_baseline_image", 0644, procdir, &tp_baseline_image_proc_fops, NULL);
    //guomingqiang@phone.bsp 2015-12-17 add for touch key debug node end

    TPD_INFO("create nodes is successe!\n");

    return 0;
}
/******************************end****************************/

/****************************S3203*****update**********************************/
#define SYNAPTICS_RMI4_PRODUCT_ID_SIZE 10
#define SYNAPTICS_RMI4_PRODUCT_INFO_SIZE 2

static void re_scan_PDT(struct i2c_client *client)
{
    uint8_t buf[8];
    i2c_smbus_read_i2c_block_data(client, 0xE9, 6,  buf);
    SynaF34DataBase = buf[3];
    SynaF34QueryBase = buf[0];
    i2c_smbus_read_i2c_block_data(client, 0xE3, 6,  buf);
    SynaF01DataBase = buf[3];
    SynaF01CommandBase = buf[1];
    i2c_smbus_read_i2c_block_data(client, 0xDD, 6,  buf);

    SynaF34Reflash_BlockNum = SynaF34DataBase;
    SynaF34Reflash_BlockData = SynaF34DataBase + 2;
    SynaF34ReflashQuery_BootID = SynaF34QueryBase;
    SynaF34ReflashQuery_FlashPropertyQuery = SynaF34QueryBase + 2;
    SynaF34ReflashQuery_FirmwareBlockSize = SynaF34QueryBase + 3;
    SynaF34ReflashQuery_FirmwareBlockCount = SynaF34QueryBase + 5;
    SynaF34ReflashQuery_ConfigBlockSize = SynaF34QueryBase + 7;
    SynaF34ReflashQuery_ConfigBlockCount = SynaF34QueryBase + 9;
    i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_FirmwareBlockSize, 2, buf);
    SynaFirmwareBlockSize = buf[0] | (buf[1] << 8);
    TPD_DEBUG("SynaFirmwareBlockSize s1302 is %d\n", SynaFirmwareBlockSize);
    SynaF34_FlashControl = SynaF34DataBase + 0x12;
}

static void parse_header(struct image_header_data *header, const unsigned char *fw_image)
{
    struct image_header *data = (struct image_header *)fw_image;

    header->checksum = extract_uint_le(data->checksum);
    TPD_DEBUG(" debug checksume is 0x%x\n", header->checksum);
    header->bootloader_version = data->bootloader_version;
    TPD_DEBUG(" debug bootloader_version is %d\n", header->bootloader_version);

    header->firmware_size = extract_uint_le(data->firmware_size);
    TPD_DEBUG(" debug firmware_size is 0x%x\n", header->firmware_size);

    header->config_size = extract_uint_le(data->config_size);
    TPD_DEBUG(" debug header->config_size is 0x%x\n", header->config_size);

    memcpy(header->product_id, data->product_id, sizeof(data->product_id));
    header->product_id[sizeof(data->product_id)] = 0;

    memcpy(header->product_info, data->product_info,
            sizeof(data->product_info));

    header->contains_firmware_id = data->options_firmware_id;
    TPD_DEBUG(" debug header->contains_firmware_id is %x\n", header->contains_firmware_id);
    if (header->contains_firmware_id)
        header->firmware_id = extract_uint_le(data->firmware_id);

    return;
}

#ifdef  TP_TEST_ENABLE
static int synaptics_read_register_map_page1(struct synaptics_ts_data *ts)
{
    unsigned char buf[4];
    int ret;
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);
    if (ret < 0) {
        TPD_DEBUG("synaptics_rmi4_i2c_write_byte failed for page select\n");
        return -1;
    }
    ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0]));
    F54_ANALOG_QUERY_BASE = buf[0];
    TPD_DEBUG("F54_ANALOG_QUERY_BASE = 0x%x\n", F54_ANALOG_QUERY_BASE);
    F54_ANALOG_COMMAND_BASE = buf[1];
    TPD_DEBUG("F54_ANALOG_COMMAND_BASE = 0x%x\n", F54_ANALOG_COMMAND_BASE);
    F54_ANALOG_CONTROL_BASE = buf[2];
    TPD_DEBUG("F54_ANALOG_CONTROL_BASE = 0x%x\n", F54_ANALOG_CONTROL_BASE);
    F54_ANALOG_DATA_BASE = buf[3];
    TPD_DEBUG("F54_ANALOG_DATA_BASE = 0x%x\n", F54_ANALOG_DATA_BASE);

    return 0;
}

static void checkCMD(void)
{
    int ret;
    int flag_err = 0;

    TPD_DEBUG("<kernel> enter checkCMD!\n");
    do {
        msleep(30); //wait 10ms
        ret = synaptics_rmi4_i2c_read_byte(ts_g->client, F54_ANALOG_COMMAND_BASE);
        flag_err++;
        TPD_INFO("try read touch ic %d time \n", flag_err);
    }while((ret > 0x00) && (flag_err < 30));

    if (ret > 0x00)
        TPD_INFO("checkCMD error ret is %x flag_err is %d\n", ret, flag_err);
}
#endif

static int checkFlashState(struct i2c_client *client)
{
    int ret ;
    int count = 0;
    ret = synaptics_rmi4_i2c_read_byte(client, SynaF34_FlashControl);
    while ((ret != 0x80) && (count < 8)) {
        msleep(3); //wait 3ms
        ret = synaptics_rmi4_i2c_read_byte(client, SynaF34_FlashControl);
        count++;
    }
    if (count == 8)
        return 1;
    else
        return 0;
}

static int synaptics_fw_check(struct synaptics_ts_data *ts)
{
    int ret;
    uint8_t buf[5];
    uint32_t bootloader_mode;

    if (!ts) {
        TPD_INFO("%s ts is NULL\n", __func__);
        return -1;
    }

    ret = synaptics_enable_interrupt(ts, 0);
    if (ret < 0) {
        TPD_DEBUG(" synaptics_ts_probe: disable interrupt failed\n");
    }

    /*read product id */
    ret = synaptics_read_product_id(ts, &buf[0]);
    if (ret) {
        TPD_INFO("failed to read product info \n");
        return -1;
    }

    bootloader_mode = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA_BASE);
    bootloader_mode = bootloader_mode & 0xff;
    bootloader_mode = bootloader_mode & 0x40;
    TPD_DEBUG("afte fw update, program memory self-check bootloader_mode = 0x%x\n", bootloader_mode);

    if (bootloader_mode == 0x40) {
        TPD_INFO("Something terrible wrong \n Trying Update the Firmware again\n");
        return -1;
    }
    return 0;
}

static int synapitcs_ts_update(struct i2c_client *client, const uint8_t *data, uint32_t data_len, bool force)
{
    int ret, j;
    uint8_t buf[8];
    uint8_t bootloder_id[10];
    uint16_t block, firmware, configuration;
    uint32_t CURRENT_FIRMWARE_ID = 0, FIRMWARE_ID = 0;
    const uint8_t *Config_Data = NULL;
    const uint8_t *Firmware_Data = NULL;
    struct image_header_data header;
    struct synaptics_ts_data *ts = dev_get_drvdata(&client->dev);

    TPD_DEBUG("%s is called\n", __func__);
    if (!client)
        return -1;

    parse_header(&header, data);
    if ((header.firmware_size + header.config_size + 0x100) > data_len) {
        TPD_DEBUG("firmware_size + config_size + 0x100 > data_len data_len = %d \n", data_len);
        return -1;
    }

    Firmware_Data = data + 0x100;
    Config_Data = Firmware_Data + header.firmware_size;
    ret = synaptics_rmi4_i2c_write_byte(client, 0xff, 0x0);

    ret = synaptics_rmi4_i2c_read_block(client, F34_FLASH_CTRL_BASE, 4, buf);
    CURRENT_FIRMWARE_ID = (buf[0] << 24)|(buf[1] << 16)|(buf[2] << 8) | buf[3];
    FIRMWARE_ID = (Config_Data[0] << 24)|(Config_Data[1] << 16)|(Config_Data[2] << 8) | Config_Data[3];
    TPD_INFO("CURRENT_FIRMWARE_ID is %x-----------, FIRMWARE_ID is %x-----------\n", CURRENT_FIRMWARE_ID, FIRMWARE_ID);
    //TPD_INFO("synaptics force is %d\n", force);
    if (!force) {
        if (CURRENT_FIRMWARE_ID == FIRMWARE_ID) {
            return 0;
        }
    }
    re_scan_PDT(client);
    block = 16;
    TPD_DEBUG("block is %d \n", block);
    firmware = (header.firmware_size)/16;
    TPD_DEBUG("firmware is %d \n", firmware);
    configuration = (header.config_size)/16;
    TPD_DEBUG("configuration is %d \n", configuration);


    ret = i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_BootID, 8, &(bootloder_id[0]));
    TPD_DEBUG("bootloader id is %x \n", (bootloder_id[1] << 8)|bootloder_id[0]);
    ret = i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
    TPD_DEBUG("Write bootloader id SynaF34_FlashControl is 0x00%x ret is %d\n", SynaF34_FlashControl, ret);

    synaptics_rmi4_i2c_write_byte(client, SynaF34_FlashControl, 0x0F);
    msleep(10);
    TPD_DEBUG("attn step 4 SynaF34_FlashControl:0x%x\n", SynaF34_FlashControl);
    ret = checkFlashState(client);
    if (ret > 0) {
        TPD_INFO("Get in prog:The status(Image) of flashstate is %x\n", ret);
        return -1;
    }
    ret = i2c_smbus_read_byte_data(client, 0x04);
    TPD_DEBUG("The status(device state) is %x\n", ret);
    ret = i2c_smbus_read_byte_data(client, F01_RMI_CTRL00);
    TPD_DEBUG("The status(control f01_RMI_CTRL_DATA) is %x\n", ret);
    ret = i2c_smbus_write_byte_data(client, F01_RMI_CTRL00, ret|0x04);
    /********************get into prog end************/
    ret = i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
    TPD_DEBUG("ret is %d\n", ret);
    re_scan_PDT(client);
    i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_BootID, 2, buf);
    i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, buf);
    i2c_smbus_write_byte_data(client, SynaF34_FlashControl, 0x03);
    msleep(1500);
    ret = i2c_smbus_read_byte_data(client, SynaF34_FlashControl);
    TPD_DEBUG("going to flash firmware area synaF34_FlashControl %d\n", ret);

    TPD_INFO("update-----------------firmware ------------------update!\n");
    TPD_DEBUG("cnt %d\n", firmware);
    for (j = 0; j < firmware; j++) {
        buf[0] = j & 0x00ff;
        buf[1] = (j & 0xff00) >> 8;
        synaptics_rmi4_i2c_write_block(client, SynaF34Reflash_BlockNum, 2, buf);
        synaptics_rmi4_i2c_write_block(client, SynaF34Reflash_BlockData, 16, &Firmware_Data[j*16]);
        synaptics_rmi4_i2c_write_byte(client, SynaF34_FlashControl, 0x02);
        ret = checkFlashState(client);
        if (ret > 0) {
            TPD_INFO("Firmware:The status(Image) of flash data3 is %x, time = %d\n", ret, j);
            return -1;
        }
    }
    //step 7 configure data
    //TPD_INFO("going to flash configuration area\n");
    //TPD_INFO("header.firmware_size is 0x%x\n", header.firmware_size);
    //TPD_INFO("bootloader_size is 0x%x\n", bootloader_size);
    TPD_INFO("update-----------------configuration ------------------update!\n");
    for (j = 0; j < configuration; j++) {
        //a)write SynaF34Reflash_BlockNum to access
        buf[0] = j & 0x00ff;
        buf[1] = (j & 0xff00) >> 8;
        synaptics_rmi4_i2c_write_block(client, SynaF34Reflash_BlockNum, 2, buf);
        //b) write data
        synaptics_rmi4_i2c_write_block(client, SynaF34Reflash_BlockData, 16, &Config_Data[j*16]);
        //c) issue write
        synaptics_rmi4_i2c_write_byte(client, SynaF34_FlashControl, 0x06);
        //d) wait attn
        ret = checkFlashState(client);
        if (ret > 0) {
            TPD_INFO("Configuration:The status(Image) of flash data3 is %x, time = %d\n", ret, j);
            return -1;
        }
    }

    //step 1 issue reset
    synaptics_rmi4_i2c_write_byte(client, SynaF01CommandBase, 0x01);
    //step2 wait ATTN
    //delay_qt_ms(1000);
    mdelay(1500);
    synaptics_read_register_map(ts);
    //FW flash check!
    ret = synaptics_fw_check(ts);
    if (ret < 0) {
        TPD_INFO("Firmware self check failed\n");
        return -1;
    }
    TPD_INFO("Firmware self check Ok\n");

    return 0;
}

static int synaptics_soft_reset(struct synaptics_ts_data *ts)
{
    int ret;

    ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);
    if (ret < 0) {
        TPD_INFO("%s error ret = %d\n", __func__, ret);
    }
    TPD_INFO("%s !!!\n", __func__);
    return ret;
}

static void synaptics_hard_reset(struct synaptics_ts_data *ts)
{
    if (ts->reset_gpio > 0)
    {
        gpio_set_value(ts->reset_gpio, 0);
        msleep(5);
        gpio_set_value(ts->reset_gpio, 1);
        msleep(100);
        TPD_INFO("%s !!!\n", __func__);
    }

}
static int synaptics_parse_dts(struct device *dev, struct synaptics_ts_data *ts)
{
    int rc;
    int retval;
    struct device_node *np;

    np = dev->of_node;
    ts->irq_gpio = of_get_named_gpio_flags(np, "synaptics,irq-gpio", 0, &(ts->irq_flags));
    if (ts->irq_gpio < 0) {
        TPD_DEBUG("ts->irq_gpio not specified\n");
    }

    ts->reset_gpio = of_get_named_gpio(np, "synaptics,reset-gpio", 0);
    if (ts->reset_gpio < 0) {
        TPD_DEBUG("ts->reset-gpio  not specified\n");
    }
    ts->en3v_gpio = of_get_named_gpio(np, "synaptics,en3v_gpio", 0);
    if (ts->en3v_gpio < 0) {
        TPD_DEBUG("ts->en3v_gpio not specified\n");
    }

    /***********power regulator_get****************/

    ts->vdd_2v8 = regulator_get(&ts->client->dev, "vdd_2v8");
    if (IS_ERR(ts->vdd_2v8)) {
        rc = PTR_ERR(ts->vdd_2v8);
        TPD_DEBUG("Regulator get failed vdd rc = %d\n", rc);
    }

    ts->vcc_i2c_1v8 = regulator_get(&ts->client->dev, "vcc_i2c_1v8");
    if (IS_ERR(ts->vcc_i2c_1v8)) {
        rc = PTR_ERR(ts->vcc_i2c_1v8);
        TPD_DEBUG("Regulator get failed vcc_i2c rc = %d\n", rc);
    }

    if (ts->reset_gpio > 0) {
        if (gpio_is_valid(ts->reset_gpio)) {
            rc = gpio_request(ts->reset_gpio, "tp-s1302-reset");
            if (rc) {
                TPD_INFO("unable to request gpio [%d]\n", ts->reset_gpio);
            }
        }
    }

    if (ts->reset_gpio > 0) {
        if (gpio_is_valid(ts->reset_gpio)) {
            rc = gpio_request(ts->reset_gpio, "s1302_reset");
            if (rc)
                TPD_INFO("unable to request gpio [%d]\n", ts->reset_gpio);
            retval = gpio_direction_input(ts->reset_gpio);
        }
    }
    if (ts->en3v_gpio > 0) {
        if (gpio_is_valid(ts->en3v_gpio)) {
            rc = gpio_request(ts->en3v_gpio, "s1302_en3v");
            if (rc)
                TPD_INFO("unable to request gpio [%d]\n", ts->en3v_gpio);
            retval = gpio_direction_output(ts->en3v_gpio, 1);
        }
    }
    return rc;
}
static int synaptics_dsx_pinctrl_init(struct synaptics_ts_data *ts)
{
    int retval;

    /* Get pinctrl if target uses pinctrl */
    ts->pinctrl = devm_pinctrl_get((ts->dev));
    if (IS_ERR_OR_NULL(ts->pinctrl)) {
        retval = PTR_ERR(ts->pinctrl);
        TPD_INFO("%s %d error!\n", __func__, __LINE__);
        goto err_pinctrl_get;
    }

    ts->pinctrl_state_active
        = pinctrl_lookup_state(ts->pinctrl, "pmx_tk_active");
    if (IS_ERR_OR_NULL(ts->pinctrl_state_active)) {
        retval = PTR_ERR(ts->pinctrl_state_active);
        TPD_INFO("%s %d error!\n", __func__, __LINE__);
        goto err_pinctrl_lookup;
    }

    ts->pinctrl_state_suspend
        = pinctrl_lookup_state(ts->pinctrl, "pmx_tk_suspend");
    if (IS_ERR_OR_NULL(ts->pinctrl_state_suspend)) {
        retval = PTR_ERR(ts->pinctrl_state_suspend);
        TPD_INFO("%s %d !\n", __func__, __LINE__);
        goto err_pinctrl_lookup;
    }
    return 0;

err_pinctrl_lookup:
    devm_pinctrl_put(ts->pinctrl);
err_pinctrl_get:
    ts->pinctrl = NULL;
    return retval;
}

static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
#ifdef CONFIG_SYNAPTIC_RED
    struct remotepanel_data *premote_data = NULL;
#endif
    struct synaptics_ts_data *ts = NULL;
    int ret = -1;
    uint8_t buf[4];
    uint32_t CURRENT_FIRMWARE_ID = 0;
    uint32_t bootloader_mode;

    TPD_INFO("%s  is called\n", __func__);

    ts = kzalloc(sizeof(struct synaptics_ts_data), GFP_KERNEL);
    if (ts == NULL) {
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }

    ts->boot_mode = MSM_BOOT_MODE__NORMAL ;//get_boot_mode();

    if ((ts->boot_mode == MSM_BOOT_MODE__FACTORY || ts->boot_mode == MSM_BOOT_MODE__RF || ts->boot_mode == MSM_BOOT_MODE__WLAN))
    {
        TPD_INFO("FTM regulator_disable is called\n");
        kfree(ts); 
        return 0;
    }

    ts->client = client;
    i2c_set_clientdata(client, ts);
    ts->dev = &client->dev;
    ts->loading_fw = false;
    ts_g = ts;

    ret = synaptics_dsx_pinctrl_init(ts);
    if (!ret && ts->pinctrl) {
        pinctrl_select_state(ts->pinctrl, ts->pinctrl_state_active);
    }

    synaptics_parse_dts(&client->dev, ts);
    /***power_init*****/
    ret = tc_power(ts, 1);
    if (ret < 0)
        TPD_INFO("regulator_enable is called\n");

    mutex_init(&ts->mutex);
    synaptics_s1302_proc();
    ts->is_suspended = 0;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        TPD_INFO("%s: need I2C_FUNC_I2C\n", __func__);
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }
    ret = synaptics_read_product_id(ts, ts->manu_name);
    if (ret < 0) {
        test_err = 1;
        synaptics_hard_reset(ts);
        ret = synaptics_read_product_id(ts, ts->manu_name);
        if (ret < 0) {
            TPD_INFO("synaptics is no exist!\n");
            goto err_check_functionality_failed;
        }
    }

    speedup_resume_wq = create_singlethread_workqueue("speedup_resume_wq");
    if (!speedup_resume_wq) {
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }
    INIT_WORK(&ts->speed_up_work, speedup_synaptics_resume);
#ifdef LCD_TRIGGER_KEY_FORCE_CAL
    INIT_WORK(&ts->judge_key_status_work, judge_key_status_work_func);
#endif
    synaptics_read_register_map(ts);
    synaptics_rmi4_i2c_read_block(ts->client, F34_FLASH_CTRL_BASE, 4, buf);
    CURRENT_FIRMWARE_ID = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    TPD_INFO("CURRENT_FIRMWARE_ID = 0x%x\n", CURRENT_FIRMWARE_ID);
    sprintf(ts->fw_id, "0x%x", CURRENT_FIRMWARE_ID);

    memset(ts->fw_name, 0, TP_FW_NAME_MAX_LEN);
    strcpy(ts->fw_name, "tp/15103/15103_Firmware_touchkey.img");
    memset(ts->test_limit_name, 0, TP_FW_NAME_MAX_LEN);
    strcpy(ts->test_limit_name, "tp/15103/15103_key_Limit_Samsung.img");
    TPD_INFO("touch_key fw : fw_name = %s  limit_name = %s \n", ts->fw_name, ts->test_limit_name);
    touch_key_info.version = ts->fw_id;
    touch_key_info.manufacture = "synatpitcs";
    register_device_proc("touch_key", touch_key_info.version, touch_key_info.manufacture);

    //push_component_info(TOUCH_KEY, ts->fw_id, ts->manu_name);

    bootloader_mode = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA_BASE);
    bootloader_mode = bootloader_mode&0x40;
    TPD_INFO("before fw update, bootloader_mode = 0x%x\n", bootloader_mode);
    if (0x40 == bootloader_mode) {
        force_update = 1;
        TPD_INFO("This FW need to be updated!\n");
    } else {
        force_update = 0;
    }

    ret = synaptics_input_init(ts);
    if (ret < 0) {
        TPD_INFO("synaptics_input_init failed!\n");
    }
#if defined(CONFIG_FB)
    ts->suspended = 0;
    ts->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&ts->fb_notif);
    if (ret)
        TPD_INFO("Unable to register fb_notifier: %d\n", ret);
#endif
    if (gpio_is_valid(ts->irq_gpio)) {
        /* configure touchscreen irq gpio */
        ret = gpio_request(ts->irq_gpio, "s1302_int");
        if (ret) {
            TPD_INFO("unable to request gpio [%d]\n", ts->irq_gpio);
        }
        ret = gpio_direction_input(ts->irq_gpio);
        msleep(50);
        ts->irq = gpio_to_irq(ts->irq_gpio);
    }
    TPD_INFO("synaptic:ts->irq is %d\n", ts->irq);
    ret = request_threaded_irq(ts->irq, NULL,
            synaptics_irq_thread_fn,
            ts->irq_flags | IRQF_ONESHOT,
            TPD_DEVICE, ts);
    if (ret < 0)
        TPD_INFO("%s request_threaded_irq ret is %d\n", __func__, ret);

    //   ret = synaptics_soft_reset(ts);
    ret = synaptics_init_panel(ts); // switch to normal work mode, can not go to sleep when power on
    if (ret < 0) {
        TPD_INFO("synaptics_init_panel failed\n");
    }
    ts->loading_fw = false;
    ret = synaptics_enable_interrupt(ts, 1);
    if (ret < 0)
        TPD_INFO("%s enable interrupt error ret = %d\n", __func__, ret);

#ifdef CONFIG_SYNAPTIC_RED
    premote_data = remote_alloc_panel_data_s1302();
    if (premote_data) {
        premote_data->client        = client;
        premote_data->input_dev     = ts->input_dev;
        premote_data->pmutex        = &ts->mutex;
        premote_data->irq_gpio      = ts->irq_gpio;
        premote_data->irq           = client->irq;
        premote_data->enable_remote = &(ts->enable_remote);
        register_remote_device_s1302(premote_data);
    }
#endif

    TPD_DEBUG("synaptics_ts_probe s1302: normal end\n");
    return 0;

err_check_functionality_failed:
err_alloc_data_failed:
    kfree(ts);
    ts = NULL;
    ts_g = NULL;
    TPD_INFO("touchkey, s1302 probe error: not normal end ret = %d\n", ret);

    return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
    struct synaptics_ts_data *ts = i2c_get_clientdata(client);

    TPD_INFO("touchkey, s1302 %s is called\n", __func__);
#ifdef CONFIG_SYNAPTIC_RED
    unregister_remote_device_s1302();
#endif

#if defined(CONFIG_FB)
    if (fb_unregister_client(&ts->fb_notif))
        dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#endif
    input_unregister_device(ts->input_dev);
    input_free_device(ts->input_dev);
    kfree(ts);

    return 0;
}

static int synaptics_ts_suspend(struct device *dev)
{
    int ret;
    struct synaptics_ts_data *ts = dev_get_drvdata(dev);
    TPD_INFO("%s: is called\n", __func__);

    if (ts->input_dev == NULL) {
        ret = -ENOMEM;
        TPD_INFO("input_dev  registration is not complete\n");
        return -1;
    }

    input_report_key(ts->input_dev, KEY_MENU, 0);
    input_sync(ts->input_dev);
    input_report_key(ts->input_dev, KEY_BACK, 0);
    input_sync(ts->input_dev);

    if (ts->loading_fw) {
        TPD_INFO("FW is updating while suspending");
        return -1;
    }
    ts->is_suspended = 1;
    disable_irq_nosync(ts->irq);
    ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x81);
    if (ret < 0) {
        TPD_INFO("%s to sleep failed\n", __func__);
        return -1;
    }
    TPD_DEBUG("%s:normal end\n", __func__);

    return 0;
}

static int synaptics_ts_resume(struct device *dev)
{
    struct synaptics_ts_data *ts = dev_get_drvdata(dev);
    TPD_INFO("%s is called\n", __func__);

    if (ts->loading_fw)
        return 0;
    queue_work(speedup_resume_wq, &ts_g->speed_up_work);

    return 0;
}

static void speedup_synaptics_resume(struct work_struct *work)
{
    int ret;
    uint32_t bootloader_mode;
    struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, speed_up_work);
    TPD_INFO("%s \n", __func__);

    if (ts->input_dev == NULL) {
        ret = -ENOMEM;
        TPD_INFO("input_dev  registration is not complete\n");
        goto ERR_RESUME;
    }

    ts->is_suspended = 0;
    mutex_lock(&ts->mutex);

    ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
    if (ret < 0) {
        TPD_INFO("%s: failed for page select try again later\n", __func__);
        msleep(20);
        ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
        if (ret < 0) {
            TPD_INFO("%s: failed for page select try again later\n", __func__);
        }
    }
    ret = synaptics_soft_reset(ts);
    msleep(50);

    //guomingqiang@phone.bsp 2016-4-15 add for check touch key firmware mode
    bootloader_mode = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA_BASE);
    bootloader_mode = bootloader_mode & 0xff;
    bootloader_mode = bootloader_mode & 0x40;

    if (bootloader_mode == 0x40)
    {
        TPD_INFO(" %s Something terrible wrong, need update the Firmware again\n", __func__);
#if 0  //only for test
        force_update = 1;
        ts_g->loading_fw = true;
        synatpitcs_fw_update(ts_g->dev, force_update);
        ts_g->loading_fw = false;
        force_update = 0;
#endif
    }
    else
    {
        TPD_INFO("%s firmware self-check bootloader_mode = 0x%x is ok\n", __func__, bootloader_mode);
    }

    ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x80);
    if (ret < 0) {
        TPD_INFO("%s:to wakeup failed\n", __func__);
        goto ERR_RESUME;
    }
    enable_irq(ts->irq);
    TPD_INFO("%s:normal end!\n", __func__);
ERR_RESUME:
    mutex_unlock(&ts->mutex);
    return ;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;

    struct synaptics_ts_data *ts = container_of(self, struct synaptics_ts_data, fb_notif);

    if (FB_EVENT_BLANK != event)
        return 0;
    if ((evdata) && (evdata->data) && (ts) && (ts->client) && (event == FB_EVENT_BLANK)) {
        blank = evdata->data;
        if (*blank == FB_BLANK_UNBLANK) {
            TPD_DEBUG("%s going TP resume\n", __func__);
            if (ts->suspended == 1) {
                synaptics_ts_resume(&ts->client->dev);
                ts->suspended = 0;
            }

        } else if (*blank == FB_BLANK_POWERDOWN) {
            TPD_DEBUG("%s : going TP suspend\n", __func__);
            if (ts->suspended == 0) {
                ts->suspended = 1;
                synaptics_ts_suspend(&ts->client->dev);
            }
        }
    }
    return 0;
}
#endif

static int __init tc_driver_init(void)
{
    if (i2c_add_driver(&tc_i2c_driver)!= 0) {
        TPD_INFO("unable to add i2c driver.\n");
        return -1;
    }
    return 0;
}

/* should never be called */
static void __exit tc_driver_exit(void)
{
    i2c_del_driver(&tc_i2c_driver);
    return;
}

module_init(tc_driver_init);
module_exit(tc_driver_exit);

MODULE_DESCRIPTION("Synaptics S1302 Touchscreen Driver");
MODULE_LICENSE("GPL");
