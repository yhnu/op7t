 /***********************************************************
 * Description : OnePlus touchpanel driver
 * 
 * File        : touchpanel_common.h      
 *
 * Function    : touchpanel public interface  
 * 
 * Version     : V1.0 
 *
 ***********************************************************/
#ifndef _TOUCHPANEL_COMMON_H_
#define _TOUCHPANEL_COMMON_H_

/*********PART1:Head files**********************/
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/oneplus/boot_mode.h>
#include <linux/workqueue.h>


#include "util_interface/touch_interfaces.h"
#include "tp_devices.h"

#define EFTM (250)
#define FW_UPDATE_COMPLETE_TIMEOUT  msecs_to_jiffies(40*1000)

/*********PART2:Define Area**********************/
#define TPD_USE_EINT
#define TYPE_B_PROTOCOL

#define PAGESIZE 512
#define MAX_GESTURE_COORD 6

#define UnkownGesture       0
#define DouTap              1   // double tap
#define UpVee               2   // V
#define DownVee             3   // ^
#define LeftVee             4   // >
#define RightVee            5   // <
#define Circle              6   // O
#define DouSwip             7   // ||
#define Left2RightSwip      8   // -->
#define Right2LeftSwip      9   // <--
#define Up2DownSwip         10  // |v
#define Down2UpSwip         11  // |^
#define Mgestrue            12  // M
#define Wgestrue            13  // W
#define SingleTap           15  // single tap
#define Sgestrue            14  // S

#define BIT0 (0x1 << 0)
#define BIT1 (0x1 << 1)
#define BIT2 (0x1 << 2)
#define BIT3 (0x1 << 3)
#define BIT4 (0x1 << 4)
#define BIT5 (0x1 << 5)
#define BIT6 (0x1 << 6)
#define BIT7 (0x1 << 7)

/* bit operation */
#define SET_BIT(data, flag) ((data) |= (flag))
#define CLR_BIT(data, flag) ((data) &= ~(flag))
#define CHK_BIT(data, flag) ((data) & (flag))
#define VK_TAB {KEY_MENU, KEY_HOMEPAGE, KEY_BACK, KEY_SEARCH}

#define TOUCH_BIT_CHECK           0x3FF  //max support 10 point report.using for detect non-valid points
#define MAX_FW_NAME_LENGTH        60
#define MAX_EXTRA_NAME_LENGTH     60
#define MAX_LIMIT_DATA_LENGTH 	  60


#define MAX_DEVICE_VERSION_LENGTH 16
#define MAX_DEVICE_MANU_LENGTH    16

#define SYNAPTICS_PREFIX    "SY_"
#define GOODIX_PREFIX       "GT_"
#define FOCAL_PREFIX        "FT_"

#define FW_UPDATE_DELAY        msecs_to_jiffies(2*1000)

/*********PART3:Struct Area**********************/
typedef enum {
    TYPE_DELTA_IDLE,   /*means not in reading delta*/
    TYPE_DELTA_BUSY,   /*reading delta data*/
}delta_state;

typedef enum {
    TYPE_PROPERTIES = 1,   /*using board_properties*/
    TYPE_AREA_SEPRATE,     /*using same IC (button zone &&  touch zone are seprate)*/
    TYPE_DIFF_IC,          /*using diffrent IC (button zone &&  touch zone are seprate)*/
    TYPE_NO_NEED,          /*No need of virtual key process*/
}vk_type;

typedef enum {
    AREA_NOTOUCH,
    AREA_EDGE,
    AREA_CRITICAL,
    AREA_NORMAL,
    AREA_CORNER,
}touch_area;

typedef enum {
    CORNER_TOPLEFT,      /*When Phone Face you in portrait top left corner*/
    CORNER_TOPRIGHT,     /*When Phone Face you in portrait top right corner*/
    CORNER_BOTTOMLEFT,   /*When Phone Face you in portrait bottom left corner*/
    CORNER_BOTTOMRIGHT,  /*When Phone Face you in portrait 7bottom right corner*/
}corner_type;

typedef enum {
    MODE_NORMAL,
    MODE_SLEEP,
    MODE_EDGE,
    MODE_GESTURE,
    MODE_GLOVE,
    MODE_CHARGE,
    MODE_GAME,
    MODE_EARSENSE,
    MODE_PALM_REJECTION,
    MODE_FACE_DETECT,
    MODE_FACE_CALIBRATE,
    MODE_REFRESH_SWITCH,
    MODE_TOUCH_HOLD,
    MODE_TOUCH_AREA_SWITCH,
    MODE_LIMIT_SWITCH,
    MODE_GESTURE_SWITCH,
    MODE_FINGERPRINT_TEST,
}work_mode;

typedef enum {
    FW_NORMAL,     /*fw might update, depend on the fw id*/
    FW_ABNORMAL,   /*fw abnormal, need update*/
}fw_check_state;

typedef enum {
    FW_UPDATE_SUCCESS,
    FW_NO_NEED_UPDATE,
    FW_UPDATE_ERROR,
    FW_UPDATE_FATAL,
}fw_update_state;

typedef enum {
    TP_SUSPEND_EARLY_EVENT,
    TP_SUSPEND_COMPLETE,
    TP_RESUME_EARLY_EVENT,
    TP_RESUME_COMPLETE,
    TP_SPEEDUP_RESUME_COMPLETE,
}suspend_resume_state;

typedef enum IRQ_TRIGGER_REASON {
    IRQ_TOUCH       = 0x01,
    IRQ_GESTURE     = 0x02,
    IRQ_BTN_KEY     = 0x04,
    IRQ_EXCEPTION   = 0x08,
    IRQ_FW_CONFIG   = 0x10,
    IRQ_DATA_LOGGER = 0x20,
    IRQ_FW_AUTO_RESET = 0x40,
    IRQ_FACE_STATE = 0x80,
    IRQ_IGNORE      = 0x00,
}irq_reason;

typedef enum vk_bitmap{
    BIT_reserve    = 0x08,
    BIT_BACK       = 0x04,
    BIT_HOME       = 0x02,
    BIT_MENU       = 0x01,
}vk_bitmap;

typedef enum finger_protect_status {
    FINGER_PROTECT_TOUCH_UP,
    FINGER_PROTECT_TOUCH_DOWN,
    FINGER_PROTECT_NOTREADY,
}fp_touch_state;

typedef enum debug_level {
    LEVEL_BASIC,    /*printk basic tp debug info*/
    LEVEL_DEBUG,    /*printk all tp debug info*/
    LEVEL_DETAIL,   /*printk tp detail log for stress test*/
}tp_debug_level;

typedef enum resume_order {
    TP_LCD_RESUME,
    LCD_TP_RESUME,
}tp_resume_order;

typedef enum suspend_order {
    TP_LCD_SUSPEND,
    LCD_TP_SUSPEND,
}tp_suspend_order;

typedef enum lcd_power {
    LCD_POWER_OFF,
    LCD_POWER_ON,
}lcd_power_status;

struct Coordinate {
    int x;
    int y;
};

typedef enum interrupt_mode {
    BANNABLE,
    UNBANNABLE,
    INTERRUPT_MODE_MAX,
}tp_interrupt_mode;

typedef enum switch_mode_type {
    SEQUENCE,
    SINGLE,
}tp_switch_mode;

struct gesture_info {
    uint32_t gesture_type;
    uint32_t clockwise;
    struct Coordinate Point_start;
    struct Coordinate Point_end;
    struct Coordinate Point_1st;
    struct Coordinate Point_2nd;
    struct Coordinate Point_3rd;
    struct Coordinate Point_4th;
};

struct point_info {
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint8_t  width_major;
    uint8_t  touch_major;
    uint8_t  status;
    touch_area type;
};

/* add haptic audio tp mask */
struct shake_point {
    uint16_t x;
    uint16_t y;
    uint8_t  status;
};

struct corner_info {
    uint8_t id;
    bool flag;
    struct point_info point;
};

struct manufacture_info {
    char *version;
	char *manufacture;
};

struct panel_info {
    char    *fw_name;                               /*FW name*/
    char    *test_limit_name;                       /*test limit name*/
    char    *extra;                                 /*for some ic, may need other information*/
   	const char  *chip_name;                         /*chip name the panel is controlled by*/
	const char  *project_name;						/*project_name*/
    uint32_t TP_FW;                                 /*FW Version Read from IC*/
    tp_dev  tp_type;
    struct manufacture_info manufacture_info;       /*touchpanel device info*/
};

struct hw_resource {
    //gpio
    int id1_gpio;
    int id2_gpio;
    int id3_gpio;

    int irq_gpio;                                   /*irq GPIO num*/
    int reset_gpio;                                 /*Reset GPIO*/

    int enable2v8_gpio;                             /*vdd_2v8 enable GPIO*/
    int enable1v8_gpio;                             /*vcc_1v8 enable GPIO*/

    //TX&&RX Num
    int TX_NUM;
    int RX_NUM;
    int key_TX;                                     /*the tx num occupied by touchkey*/
    int key_RX;                                     /*the rx num occupied by touchkey*/
    int EARSENSE_TX_NUM;                            /*for earsense function data reading*/
    int EARSENSE_RX_NUM;                            /*for earsense function data reading*/

    //power
    struct regulator *vdd_2v8;                      /*power 2v8*/
    struct regulator *vcc_1v8;                      /*power 1v8*/
    uint32_t vdd_volt;                              /*avdd specific volt*/

    //pinctrl
    struct pinctrl          *pinctrl;
    struct pinctrl_state    *pin_set_high;
    struct pinctrl_state    *pin_set_low;
};

struct edge_limit {
    int limit_area;
    int left_x1;
    int right_x1;
    int left_x2;
    int right_x2;
    int left_x3;
    int right_x3;
    int left_y1;
    int right_y1;
    int left_y2;
    int right_y2;
    int left_y3;
    int right_y3;
    touch_area in_which_area;
};

struct touch_major_limit {
    int width_range;
    int height_range;
};

struct button_map {
    int width_x;                                        /*width of each key area */
    int height_y;                                       /*height of each key area*/
    struct Coordinate coord_menu;                       /*Menu centre coordinates*/
    struct Coordinate coord_home;                       /*Home centre coordinates*/
    struct Coordinate coord_back;                       /*Back centre coordinates*/
};

struct resolution_info {
    uint32_t max_x;                                     /*touchpanel width */
    uint32_t max_y;                                     /*touchpanel height*/
    uint32_t LCD_WIDTH;                                 /*LCD WIDTH        */
    uint32_t LCD_HEIGHT;                                /*LCD HEIGHT       */
};

struct esd_information {
    bool esd_running_flag;
    int  esd_work_time;
    struct mutex             esd_lock;
    struct workqueue_struct *esd_workqueue;
    struct delayed_work      esd_check_work;
};

struct spurious_fp_touch {
    bool fp_trigger;                                    /*thread only turn into runnning state by fingerprint kick proc/touchpanel/finger_protect_trigger*/
    bool lcd_resume_ok;
    bool lcd_trigger_fp_check;
    fp_touch_state fp_touch_st;                         /*provide result to fingerprint of touch status data*/
    struct task_struct *thread;                         /*tread use for fingerprint susprious touch check*/
};

struct register_info {
    uint8_t reg_length;
    uint16_t reg_addr;
    uint8_t * reg_result;
};

struct black_gesture_test {
    bool gesture_backup;                               /*store the gesture enable flag */
    bool flag;                                         /* indicate do black gesture test or not*/
    char *message;                                 /* failure information if gesture test failed */
};

struct debug_info_proc_operations;
struct earsense_proc_operations;
struct touchpanel_data {
    bool register_is_16bit;                             /*register is 16bit*/
    bool glove_mode_support;                            /*glove_mode support feature*/
    bool black_gesture_support;                         /*black_gesture support feature*/
    bool charger_pump_support;                          /*charger_pump support feature*/
    bool edge_limit_support;                            /*edge_limit support feature*/
    bool esd_handle_support;                            /*esd handle support feature*/
    bool spurious_fp_support;                           /*avoid fingerprint spurious trrigger feature*/
    bool gesture_test_support;                          /*indicate test black gesture or not*/
    bool game_switch_support;                           /*indicate game switch support or not*/
    bool ear_sense_support;                             /*touch porximity function*/
    bool smart_gesture_support;                         /*feature used to controltouch_major report*/
    bool face_detect_support;                           /*touch porximity function*/
    bool lcd_refresh_rate_switch;                       /*switch lcd refresh rate 60-90hz*/
	bool touch_hold_support;							/*touchhold function for fingerprint*/
	bool ctl_base_address;								/*change only for 18865*/
	bool tx_change_order;								/*add for dvt change tx order */

	bool charge_detect;
	bool charge_detect_support;
	bool module_id_support;								/*update firmware by lcd module id*/
    bool i2c_ready;                                     /*i2c resume status*/
    bool is_usb_checked;                                /*state of charger or usb*/
    bool loading_fw;                                    /*touchpanel FW updating*/
    bool is_incell_panel;                               /*touchpanel is incell*/
    bool is_noflash_ic;                                 /*noflash ic*/
    bool has_callback;                                  /*whether have callback method to invoke common*/
    bool use_resume_notify;                             /*notify speed resume process*/
    bool fw_update_app_support;                         /*bspFwUpdate is used*/
    bool in_test_process;                               /*flag whether in test process*/
    u8   vk_bitmap ;                                     /*every bit declear one state of key "reserve(keycode)|home(keycode)|menu(keycode)|back(keycode)"*/
    vk_type  vk_type;                                   /*virtual_key type*/
    delta_state delta_state;

    uint32_t irq_flags;                                 /*irq setting flag*/
    int irq;                                            /*irq num*/

    int gesture_enable;                                 /*control state of black gesture*/
    int palm_enable;
    int es_enable;
    int fd_enable;
	int fd_calibrate;
    int lcd_refresh_rate;
	int	touch_hold_enable;
	int touch_area_switch;
    int touch_count;
    int glove_enable;                                   /*control state of glove gesture*/
    int limit_enable;                                   /*control state of limit ebale */
    int limit_edge;                                     /*control state of limit edge*/
    int limit_corner;                                /*control state of limit corner*/
    int is_suspended;                                   /*suspend/resume flow exec flag*/
    int corner_delay_up;                                /*corner mode flag*/
    suspend_resume_state suspend_state;                 /*detail suspend/resume state*/

    int boot_mode;                                      /*boot up mode */
    int view_area_touched;                              /*view area touched flag*/
    int force_update;                                   /*force update flag*/
    int max_num;                                        /*max muti-touch num supportted*/
    int irq_slot;                                       /*debug use, for print all finger's first touch log*/
    int firmware_update_type;                           /*firmware_update_type: 0=check firmware version 1=force update; 2=for FAE debug*/

    tp_resume_order tp_resume_order;
    tp_suspend_order tp_suspend_order;
    tp_interrupt_mode int_mode;                         /*whether interrupt and be disabled*/
    tp_switch_mode mode_switch_type;                    /*used for switch mode*/
    bool skip_reset_in_resume;                          /*some incell ic is reset by lcd reset*/
    bool skip_suspend_operate;                          /*LCD and TP is in one chip,lcd power off in suspend at first,
                                                         can not operate i2c when tp suspend*/
    bool ps_status;                                     /*save ps status, ps near = 1, ps far = 0*/
    int noise_level;                                     /*save ps status, ps near = 1, ps far = 0*/
	bool gesture_switch;								/*gesture mode close or open gesture*/
	bool reject_point;									/*reject point for sensor*/
	bool fingerprint_int_test;							/*fingerprint int pin test*/
	u8 limit_switch;									/*0 is phone up 1 is crosswise*/
	u8 touchold_event;									/*0 is touchhold down 1 is up*/

#if defined(TPD_USE_EINT)
    struct hrtimer         timer;                       /*using polling instead of IRQ*/
#endif
#if defined(CONFIG_FB)
    struct notifier_block fb_notif;                     /*register to control suspend/resume*/
#endif

    struct mutex           mutex;                       /*mutex for lock i2c related flow*/
    struct mutex           mutex_earsense;
    struct completion      pm_complete;                 /*completion for control suspend and resume flow*/
    struct completion      fw_complete;                 /*completion for control fw update*/
    struct completion      resume_complete;                 /*completion for control fw update*/
    struct panel_info      panel_data;                  /*GPIO control(id && pinctrl && tp_type)*/
    struct hw_resource     hw_res;                      /*hw resourc information*/
    struct edge_limit      edge_limit;                  /*edge limit*/
    struct button_map      button_map;                  /*virtual_key button area*/
    struct resolution_info resolution_info;             /*resolution of touchpanel && LCD*/
    struct gesture_info    gesture;                     /*gesture related info*/
    struct touch_major_limit touch_major_limit;         /*used for control touch major reporting area*/

    struct work_struct     speed_up_work;               /*using for speedup resume*/
    struct workqueue_struct *speedup_resume_wq;         /*using for touchpanel speedup resume wq*/

    struct work_struct     read_delta_work;               /*using for read delta*/
    struct workqueue_struct *delta_read_wq;

    struct work_struct     async_work;
    struct workqueue_struct *async_workqueue;
    struct work_struct     fw_update_work;             /*using for fw update*/
	struct delayed_work 	work_read_info;				/*using for print more rawdata when probe*/
	struct wakeup_source	source;


    struct esd_information  esd_info;
    struct spurious_fp_touch spuri_fp_touch;           /*spurious_finger_support*/

    struct device         *dev;                         /*used for i2c->dev*/
    struct i2c_client     *client;
    struct spi_device     *s_client;
    struct input_dev      *input_dev;
    struct input_dev      *kpd_input_dev;

    struct touchpanel_operations *ts_ops;          /*call_back function*/
    struct proc_dir_entry *prEntry_tp;                  /*struct proc_dir_entry of "/proc/touchpanel"*/
    struct proc_dir_entry *prEntry_debug_tp;                  /*struct proc_dir_entry of "/proc/touchpanel/debug_info"*/
    struct debug_info_proc_operations  *debug_info_ops;                /*debug info data*/
    struct earsense_proc_operations *earsense_ops;
    struct register_info reg_info;                      /*debug node for register length*/
    struct black_gesture_test gesture_test;             /*gesture test struct*/

    void                  *chip_data;                   /*Chip Related data*/
    void                  *private_data;                /*Reserved Private data*/
    char                  *earsense_delta;
};

struct touchpanel_operations {
    int  (*get_chip_info)        (void *chip_data);                                           /*return 0:success;other:failed*/
    int  (*mode_switch)          (void *chip_data, work_mode mode, bool flag);                /*return 0:success;other:failed*/
    int  (*get_touch_points)     (void *chip_data, struct point_info *points, int max_num);   /*return point bit-map*/
    int  (*get_gesture_info)     (void *chip_data, struct gesture_info * gesture);            /*return 0:success;other:failed*/
    int  (*get_vendor)           (void *chip_data, struct panel_info  *panel_data);           /*distingush which panel we use, (TRULY/OFLIM/BIEL/TPK)*/
    int  (*reset)                (void *chip_data);                                           /*Reset Touchpanel*/
    int  (*reinit_device)        (void *chip_data);
    fw_check_state  (*fw_check)  (void *chip_data, struct resolution_info *resolution_info,
                                                  struct panel_info *panel_data);             /*return < 0 :failed; 0 sucess*/
    fw_update_state (*fw_update) (void *chip_data, const struct firmware *fw, bool force);    /*return 0 normal; return -1:update failed;*/
    int  (*power_control)        (void *chip_data, bool enable);                              /*return 0:success;other:abnormal, need to jump out*/
    int  (*reset_gpio_control)   (void *chip_data, bool enable);                              /*used for reset gpio*/
    u8   (*trigger_reason)       (void *chip_data, int gesture_enable, int is_suspended);     /*clear innterrupt reg && detect irq trigger reason*/
    u8   (*get_keycode)          (void *chip_data);                                           /*get touch-key code*/
    int  (*esd_handle)           (void *chip_data);
    int  (*fw_handle)            (void *chip_data);                                           /*return 0 normal; return -1:update failed;*/
    void (*resume_prepare)       (void *chip_data);                                           /*using for operation before resume flow,
                                                                                                eg:incell 3320 need to disable gesture to release inter pins for lcd resume*/
    fp_touch_state (*spurious_fp_check) (void *chip_data);                                    /*spurious fingerprint check*/
    void (*finger_proctect_data_get)    (void *chip_data);                                    /*finger protect data get*/
    void (*data_logger_open)     (void *chip_data);                                           /*get data logger open status in probe or after fwupdate*/
    void (*data_logger_get)      (void *chip_data);                                           /*data logger get*/
    void (*exit_esd_mode)        (void *chip_data);                                          /*add for s4322 exit esd mode*/
    void (*register_info_read)(void * chip_data, uint16_t register_addr, uint8_t *result, uint8_t length);   /*add for read registers*/
    void (*write_ps_status)      (void *chip_data, int ps_status);                            /*when detect iron plate, if ps is near ,enter iron plate mode;if ps is far, can not enter; exit esd mode when ps is far*/
    void (*specific_resume_operate) (void *chip_data);                                        /*some ic need specific opearation in resuming*/
    int  (*get_usb_state)        (void);                                                      /*get current usb state*/
    void (*black_screen_test)    (void *chip_data, char *msg);                                /*message of black gesture test*/
    int  (*irq_handle_unlock)     (void *chip_info);                                          /*irq handler without mutex*/
    int  (*async_work)           (void *chip_info);                                           /*async work*/
    int  (*get_face_state)       (void *chip_info);                                          /*get face detect state*/
};

struct debug_info_proc_operations {
    void (*limit_read)          (struct seq_file *s, struct touchpanel_data *ts);
    void (*delta_read)          (struct seq_file *s, void *chip_data);
    void (*self_delta_read)     (struct seq_file *s, void *chip_data);
    void (*self_raw_read)       (struct seq_file *s, void *chip_data);
    void (*baseline_read)       (struct seq_file *s, void *chip_data);
    void (*baseline_blackscreen_read) (struct seq_file *s, void *chip_data);
    void (*main_register_read)  (struct seq_file *s, void *chip_data);
    void (*reserve_read)        (struct seq_file *s, void *chip_data);
    void (*abs_doze_read)       (struct seq_file *s, void *chip_data);
    void (*RT251)               (struct seq_file *s, void *chip_data);
    void (*RT76)                (struct seq_file *s, void *chip_data);
    void (*RT254)               (struct seq_file *s, void *chip_data);
    void (*DRT)                 (struct seq_file *s, void *chip_data);
};

struct invoke_method {
    void (*invoke_common)(void);
    void (*async_work)(void);
};

struct earsense_proc_operations {
    void (*rawdata_read)        (void *chip_data, char *earsense_baseline, int read_length);
    void (*delta_read)          (void *chip_data, char *earsense_delta, int read_length);
    void (*self_data_read)      (void *chip_data, char *earsense_self_data, int read_length);
};

/*********PART3:function or variables for other files**********************/
extern unsigned int tp_debug ;                                                            /*using for print debug log*/

struct touchpanel_data *common_touch_data_alloc(void);

int  common_touch_data_free(struct touchpanel_data *pdata);
int  register_common_touch_device(struct touchpanel_data *pdata);

void tp_i2c_suspend(struct touchpanel_data *ts);
void tp_i2c_resume (struct touchpanel_data *ts);

int  tp_powercontrol_1v8(struct hw_resource *hw_res, bool on);
int  tp_powercontrol_2v8(struct hw_resource *hw_res, bool on);

void operate_mode_switch  (struct touchpanel_data *ts);
void input_report_key_reduce(struct input_dev *dev, unsigned int code, int value);
void esd_handle_switch(struct esd_information *esd_info, bool on);
void clear_view_touchdown_flag(void);
void tp_touch_btnkey_release(void);
void tp_util_get_vendor(struct touchpanel_data *ts, struct panel_info *panel_data);
extern bool tp_judge_ic_match(char * tp_ic_name);
/* add haptic audio tp mask */
extern int msm_drm_notifier_call_chain(unsigned long val, void *v);
extern int gf_opticalfp_irq_handler(int event);

#endif

