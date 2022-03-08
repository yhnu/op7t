#ifndef _AW8697_H_
#define _AW8697_H_

/*********************************************************
 *
 * kernel version
 *
 ********************************************************/
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 4, 1)
#define TIMED_OUTPUT
#endif

/*********************************************************
 *
 * aw8697.h
 *
 ********************************************************/
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#ifdef TIMED_OUTPUT
#include <../../../drivers/staging/android/timed_output.h>
#else
#include <linux/leds.h>
#endif

/*********************************************************
 *
 * marco
 *
 ********************************************************/
#define MAX_I2C_BUFFER_SIZE                 65536

#define AW8697_REG_MAX                      0xff

#define AW8697_SEQUENCER_SIZE               8
#define AW8697_SEQUENCER_LOOP_SIZE          4

#define AW8697_RTP_I2C_SINGLE_MAX_NUM       512

#define HAPTIC_MAX_TIMEOUT                  10000

#define AW8697_VBAT_REFER                   4200
#define AW8697_VBAT_MIN                     3000
#define AW8697_VBAT_MAX                     4500
/* motor config */
#define LRA_0619

#ifdef LRA_0619
#define AW8697_HAPTIC_F0_PRE                1700    /* 170Hz*/
#define AW8697_HAPTIC_F0_CALI_PERCEN        7       /* -7%~7%*/
#define AW8697_HAPTIC_CONT_DRV_LVL          97  /*op modify for count mode*/   /* value*6.1/256*/
#define AW8697_HAPTIC_CONT_DRV_LVL_OV       97 /*op modify for count mode*/    /* value*6.1/256*/
#define AW8697_HAPTIC_CONT_TD               0x009a
#define AW8697_HAPTIC_CONT_ZC_THR           0x0ff1
#define AW8697_HAPTIC_CONT_NUM_BRK          3
#endif

#ifdef LRA_0832
#define AW8697_HAPTIC_F0_PRE                2350    /* 170Hz*/
#define AW8697_HAPTIC_F0_CALI_PERCEN        7       /* -7%~7%*/
#define AW8697_HAPTIC_CONT_DRV_LVL          125     /* 125*6.1/256=2.98v*/
#define AW8697_HAPTIC_CONT_DRV_LVL_OV       155     /*155*6.1/256=3.69v*/
#define AW8697_HAPTIC_CONT_TD               0x006c
#define AW8697_HAPTIC_CONT_ZC_THR           0x0ff1
#define AW8697_HAPTIC_CONT_NUM_BRK          3
#endif


#define AW8697_HAPTIC_F0_COEFF              260     /*2.604167*/


/* trig config */
#define AW8697_TRIG_NUM                     3
#define AW8697_TRG1_ENABLE                  1
#define AW8697_TRG2_ENABLE                  1
#define AW8697_TRG3_ENABLE                  1
/*
 * trig default high level
 * ___________           _________________
 *           |           |
 *           |           |
 *           |___________|
 *        first edge
 *                   second edge
 *
 *
 * trig default low level
 *            ___________
 *           |           |
 *           |           |
 * __________|           |_________________
 *        first edge
 *                   second edge
 */
#define AW8697_TRG1_DEFAULT_LEVEL           1       // 1: high level; 0: low level
#define AW8697_TRG2_DEFAULT_LEVEL           1       // 1: high level; 0: low level
#define AW8697_TRG3_DEFAULT_LEVEL           1       // 1: high level; 0: low level

#define AW8697_TRG1_DUAL_EDGE               1       // 1: dual edge; 0: first edge
#define AW8697_TRG2_DUAL_EDGE               1       // 1: dual edge; 0: first edge
#define AW8697_TRG3_DUAL_EDGE               1       // 1: dual edge; 0: first edge

#define AW8697_TRG1_FIRST_EDGE_SEQ          1       // trig1: first edge waveform seq
#define AW8697_TRG1_SECOND_EDGE_SEQ         2       // trig1: second edge waveform seq
#define AW8697_TRG2_FIRST_EDGE_SEQ          1       // trig2: first edge waveform seq
#define AW8697_TRG2_SECOND_EDGE_SEQ         2       // trig2: second edge waveform seq
#define AW8697_TRG3_FIRST_EDGE_SEQ          1       // trig3: first edge waveform seq
#define AW8697_TRG3_SECOND_EDGE_SEQ         2       // trig3: second edge waveform seq


#if AW8697_TRG1_ENABLE
#define AW8697_TRG1_DEFAULT_ENABLE          AW8697_BIT_TRGCFG2_TRG1_ENABLE
#else
#define AW8697_TRG1_DEFAULT_ENABLE          AW8697_BIT_TRGCFG2_TRG1_DISABLE
#endif

#if AW8697_TRG2_ENABLE
#define AW8697_TRG2_DEFAULT_ENABLE          AW8697_BIT_TRGCFG2_TRG2_ENABLE
#else
#define AW8697_TRG2_DEFAULT_ENABLE          AW8697_BIT_TRGCFG2_TRG2_DISABLE
#endif

#if AW8697_TRG3_ENABLE
#define AW8697_TRG3_DEFAULT_ENABLE          AW8697_BIT_TRGCFG2_TRG3_ENABLE
#else
#define AW8697_TRG3_DEFAULT_ENABLE          AW8697_BIT_TRGCFG2_TRG3_DISABLE
#endif

#if AW8697_TRG1_DEFAULT_LEVEL
#define AW8697_TRG1_DEFAULT_POLAR           AW8697_BIT_TRGCFG1_TRG1_POLAR_POS
#else
#define AW8697_TRG1_DEFAULT_POLAR           AW8697_BIT_TRGCFG1_TRG1_POLAR_NEG
#endif

#if AW8697_TRG2_DEFAULT_LEVEL
#define AW8697_TRG2_DEFAULT_POLAR           AW8697_BIT_TRGCFG1_TRG2_POLAR_POS
#else
#define AW8697_TRG2_DEFAULT_POLAR           AW8697_BIT_TRGCFG1_TRG2_POLAR_NEG
#endif

#if AW8697_TRG3_DEFAULT_LEVEL
#define AW8697_TRG3_DEFAULT_POLAR           AW8697_BIT_TRGCFG1_TRG3_POLAR_POS
#else
#define AW8697_TRG3_DEFAULT_POLAR           AW8697_BIT_TRGCFG1_TRG3_POLAR_NEG
#endif

#if AW8697_TRG1_DUAL_EDGE
#define AW8697_TRG1_DEFAULT_EDGE            AW8697_BIT_TRGCFG1_TRG1_EDGE_POS_NEG
#else
#define AW8697_TRG1_DEFAULT_EDGE            AW8697_BIT_TRGCFG1_TRG1_EDGE_POS
#endif

#if AW8697_TRG2_DUAL_EDGE
#define AW8697_TRG2_DEFAULT_EDGE            AW8697_BIT_TRGCFG1_TRG2_EDGE_POS_NEG
#else
#define AW8697_TRG2_DEFAULT_EDGE            AW8697_BIT_TRGCFG1_TRG2_EDGE_POS
#endif

#if AW8697_TRG3_DUAL_EDGE
#define AW8697_TRG3_DEFAULT_EDGE            AW8697_BIT_TRGCFG1_TRG3_EDGE_POS_NEG
#else
#define AW8697_TRG3_DEFAULT_EDGE            AW8697_BIT_TRGCFG1_TRG3_EDGE_POS
#endif
enum aw8697_flags {
    AW8697_FLAG_NONR = 0,
    AW8697_FLAG_SKIP_INTERRUPTS = 1,
};

enum aw8697_haptic_read_write {
    AW8697_HAPTIC_CMD_READ_REG = 0,
    AW8697_HAPTIC_CMD_WRITE_REG = 1,
};


enum aw8697_haptic_work_mode {
    AW8697_HAPTIC_STANDBY_MODE = 0,
    AW8697_HAPTIC_RAM_MODE = 1,
    AW8697_HAPTIC_RTP_MODE = 2,
    AW8697_HAPTIC_TRIG_MODE = 3,
    AW8697_HAPTIC_CONT_MODE = 4,
    AW8697_HAPTIC_RAM_LOOP_MODE = 5,
};

enum aw8697_haptic_bst_mode {
    AW8697_HAPTIC_BYPASS_MODE = 0,
    AW8697_HAPTIC_BOOST_MODE = 1,
};

enum aw8697_haptic_activate_mode {
  AW8697_HAPTIC_ACTIVATE_RAM_MODE = 0,
  AW8697_HAPTIC_ACTIVATE_CONT_MODE = 1,
};


enum aw8697_haptic_cont_vbat_comp_mode {
    AW8697_HAPTIC_CONT_VBAT_SW_COMP_MODE = 0,
    AW8697_HAPTIC_CONT_VBAT_HW_COMP_MODE = 1,
};

enum aw8697_haptic_ram_vbat_comp_mode {
    AW8697_HAPTIC_RAM_VBAT_COMP_DISABLE = 0,
    AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE = 1,
};

enum aw8697_haptic_f0_flag {
    AW8697_HAPTIC_LRA_F0 = 0,
    AW8697_HAPTIC_CALI_F0 = 1,
};

enum aw8697_haptic_pwm_mode {
    AW8697_PWM_48K = 0,
    AW8697_PWM_24K = 1,
    AW8697_PWM_12K = 2,
};


enum aw8697_haptic_play {
    AW8697_HAPTIC_PLAY_NULL = 0,
    AW8697_HAPTIC_PLAY_ENABLE = 1,
    AW8697_HAPTIC_PLAY_STOP = 2,
    AW8697_HAPTIC_PLAY_GAIN = 8,
};

enum aw8697_haptic_cmd {
    AW8697_HAPTIC_CMD_NULL = 0,
    AW8697_HAPTIC_CMD_ENABLE = 1,
    AW8697_HAPTIC_CMD_HAPTIC = 0x0f,
    AW8697_HAPTIC_CMD_TP = 0x10,
    AW8697_HAPTIC_CMD_SYS = 0xf0,
    AW8697_HAPTIC_CMD_STOP = 255,
};

enum aw8697_haptic_tp_flag {
    AW8697_HAPTIC_TP_NULL = 0,
    AW8697_HAPTIC_TP_PRESS = 1,
    AW8697_HAPTIC_TP_PRESS_HOLD = 2,
    AW8697_HAPTIC_TP_RELEASE = 3,
    AW8697_HAPTIC_TP_RELEASE_HOLD = 4,
};

enum aw8697_haptic_tp_staus {
    AW8697_HAPTIC_TP_ST_RELEASE = 0,
    AW8697_HAPTIC_TP_ST_PRESS = 1,
};

enum aw8697_haptic_tp_play_flag {
    AW8697_HAPTIC_TP_PLAY_NULL = 0,
    AW8697_HAPTIC_TP_PLAY_ENABLE = 1,
    AW8697_HAPTIC_TP_PLAY_NOMORE= 2,
};


enum aw8697_haptic_tp_touch_flag {
    AW8697_HAPTIC_TP_TOUCH_INVAIL = 0,
    AW8697_HAPTIC_TP_TOUCH_VAIL = 1,
};

#define AW8697_HAPTIC_TP_ID_MAX     10


#define AW8697_HAPTIC_AI_X_JITTER	20
#define AW8697_HAPTIC_AI_Y_JITTER	20
#define AW8697_HAPTIC_AI_X_DFT_W	200
#define AW8697_HAPTIC_AI_Y_DFT_H	200


enum aw8697_haptic_tz_level {
    AW8697_HAPTIC_TZ_LEVEL_LOW = 0,
    AW8697_HAPTIC_TZ_LEVEL_HIGH = 1,
};

/*********************************************************
 *
 * struct
 *
 ********************************************************/
struct tp_input_info {
    uint8_t  id;
    uint8_t  status;
    uint16_t x;
    uint16_t y;
};

struct trust_zone_info {
    uint8_t  level;
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
};

struct ai_trust_zone {
    uint8_t  num;
    struct trust_zone_info *tz_info;
};


struct haptic_audio_trust_zone {
    uint8_t  level;//tz score
    uint8_t  cnt;
    uint8_t  dirty;
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    struct list_head list;
};

struct haptic_audio_tp_size {
    uint16_t x;
    uint16_t y;
};

struct shake_point {
    uint8_t  id;
    uint16_t x;
    uint16_t y;
    uint8_t  status;
    uint8_t  touch_flag;
    uint8_t  touch_outside_tz_flag;
};


struct fileops {
    unsigned char cmd;
    unsigned char reg;
    unsigned char ram_addrh;
    unsigned char ram_addrl;
};

struct ram {
    unsigned int len;
    unsigned int check_sum;
    unsigned int base_addr;
    unsigned char version;
    unsigned char ram_shift;
    unsigned char baseaddr_shift;
};

struct haptic_ctr{
    unsigned char cnt;
    unsigned char cmd;
    unsigned char play;
    unsigned char wavseq;
    unsigned char loop;
    unsigned char gain;
    struct list_head list;
};

struct tp_id{
    struct shake_point pt_info;
    unsigned char tp_flag;
    unsigned char press_flag;
    unsigned char release_flag;
    struct timeval t_press;
    struct timeval t_release;
    unsigned char play_flag;
    unsigned int no_play_cnt;
    unsigned char tp_ai_match_flag;
    unsigned char press_no_vibrate_flag;
    unsigned char release_no_vibrate_flag;
};

struct tp{
    struct tp_id id[AW8697_HAPTIC_TP_ID_MAX+1];
    unsigned char id_index;
    unsigned char virtual_id;
    unsigned int press_delay_min;
    unsigned int press_delay_max;
    unsigned int release_delay_max;
    unsigned char play_flag;
    unsigned char last_play_flag;
    unsigned char press_flag;
    unsigned char tp_ai_match_flag;
    unsigned char tp_ai_check_flag;
    unsigned char hap_match_without_tz_cnt;
    unsigned int no_play_cnt_max;
};

struct haptic_audio{
    struct mutex lock;
    struct hrtimer timer;
    struct work_struct work;
    int delay_val;
    int timer_val;
    struct haptic_ctr ctr;
    struct list_head ctr_list;
    struct tp tp;
    struct list_head list;
    struct list_head score_list;
    struct haptic_audio_tp_size tp_size;
    struct trust_zone_info output_tz_info[10];
    int tz_num;
    int tz_high_num;
    int tz_cnt_thr;
    int tz_cnt_max;
    int tz_init;
    unsigned int uevent_report_flag;
    unsigned int hap_cnt_outside_tz;
    unsigned int hap_cnt_max_outside_tz;
};

struct trig{
    unsigned char enable;
    unsigned char default_level;
    unsigned char dual_edge;
    unsigned char frist_seq;
    unsigned char second_seq;
};

struct aw8697 {
    struct regmap *regmap;
    struct i2c_client *i2c;
    struct device *dev;
    struct input_dev *input;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pinctrl_state_active;
    struct mutex lock;
    struct hrtimer timer;
    struct work_struct vibrator_work;
    struct work_struct rtp_work;
    struct delayed_work ram_work;
    struct timeval current_time;
    struct timeval pre_enter_time;
    struct wakeup_source vibrator_on;
#ifdef TIMED_OUTPUT
    struct timed_output_dev to_dev;
#else
    struct led_classdev cdev;
#endif
    struct fileops fileops;
    struct ram ram;
    bool pm_awake;
    bool haptic_ready;
    bool audio_ready;
    bool ignore_sync;
    int pre_haptic_number;
    bool rtp_on;
    struct timeval start,end;
    unsigned int timeval_flags;
    unsigned int osc_cali_flag;
    unsigned long int microsecond;
    unsigned int sys_frequency;
    unsigned int rtp_len;
    unsigned int lra_calib_data;

    int reset_gpio;
    int irq_gpio;

    unsigned char hwen_flag;
    unsigned char flags;
    unsigned char chipid;

    unsigned char play_mode;

    unsigned char activate_mode;
    unsigned char auto_boost;

    int state;
    int duration;
    int amplitude;
    int index;
    int vmax;
    int gain;
    int level;

    unsigned char seq[AW8697_SEQUENCER_SIZE];
    unsigned char loop[AW8697_SEQUENCER_SIZE];

    unsigned int rtp_cnt;
    unsigned int rtp_file_num;

    unsigned char rtp_init;
    unsigned char ram_init;
    unsigned char rtp_routine_on;

    unsigned int f0;
    unsigned int f0_pre;
    unsigned int cont_f0;
    unsigned int cont_td;
    unsigned int cont_zc_thr;
    unsigned char cont_drv_lvl;
    unsigned char cont_drv_lvl_ov;
    unsigned char cont_num_brk;
    unsigned char max_pos_beme;
    unsigned char max_neg_beme;
    unsigned char f0_cali_flag;

    unsigned char ram_vbat_comp;
    unsigned int vbat;
    unsigned int lra;
    unsigned int ram_bin_index;
    unsigned int haptic_real_f0;
    unsigned int ram_test_flag_0;
    unsigned int ram_test_flag_1;
    unsigned int ram_test_result;
    bool count_go;

    struct trig trig[AW8697_TRIG_NUM];
    struct haptic_audio haptic_audio;
    struct mutex rtp_lock;
    struct timeval t_stop;
    struct timeval t_start;
    unsigned int game_microsecond;
    unsigned int interval_us;
    struct notifier_block fb_notif;/*register to control tp report*/
};

struct aw8697_container{
    int len;
    unsigned char data[];
};


/*********************************************************
 *
 * ioctl
 *
 ********************************************************/
struct aw8697_seq_loop {
    unsigned char loop[AW8697_SEQUENCER_SIZE];
};

struct aw8697_que_seq {
    unsigned char index[AW8697_SEQUENCER_SIZE];
};


#define AW8697_HAPTIC_IOCTL_MAGIC         'h'

#define AW8697_HAPTIC_SET_QUE_SEQ         _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 1, struct aw8697_que_seq*)
#define AW8697_HAPTIC_SET_SEQ_LOOP        _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 2, struct aw8697_seq_loop*)
#define AW8697_HAPTIC_PLAY_QUE_SEQ        _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 3, unsigned int)
#define AW8697_HAPTIC_SET_BST_VOL         _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 4, unsigned int)
#define AW8697_HAPTIC_SET_BST_PEAK_CUR    _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 5, unsigned int)
#define AW8697_HAPTIC_SET_GAIN            _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 6, unsigned int)
#define AW8697_HAPTIC_PLAY_REPEAT_SEQ     _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 7, unsigned int)
#endif

