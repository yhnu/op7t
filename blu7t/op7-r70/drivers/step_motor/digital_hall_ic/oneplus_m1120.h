/****************************************************************************************
** Copyright (C), 2013-2018, ONEPLUS 
** File        : oneplus_m1120.c
**
** Description : 
**      Definitions for m1120 digital hall_up and hall_down sensor 
**
****************************************************************************************/
#ifndef __MXM1120_H__
#define __MXM1120_H__

#include <linux/ioctl.h>
#include <linux/mutex.h>

 /*********************************************************************
                            register map
 **********************************************************************/
#define M1120_REG_PERSINT                   (0x00)
#define M1120_VAL_PERSINT_COUNT(n)          (n<<4)
#define M1120_VAL_PERSINT_INTCLR            (0x01)
#define M1120_REG_INTSRS                    (0x01)
#define M1120_VAL_INTSRS_INT_ON             (0x80)
#define M1120_DETECTION_MODE_INTERRUPT      M1120_VAL_INTSRS_INT_ON
#define M1120_VAL_INTSRS_INT_OFF            (0x00)
#define M1120_DETECTION_MODE_POLLING        M1120_VAL_INTSRS_INT_OFF
#define M1120_VAL_INTSRS_INTTYPE_BESIDE     (0x00)
#define M1120_VAL_INTSRS_INTTYPE_WITHIN     (0x10)
#define M1120_VAL_INTSRS_SRS_10BIT_0_068mT  (0x00)
#define M1120_VAL_INTSRS_SRS_10BIT_0_034mT  (0x01)
#define M1120_VAL_INTSRS_SRS_10BIT_0_017mT  (0x02)
#define M1120_VAL_INTSRS_SRS_10BIT_0_009mT  (0x03)
#define M1120_VAL_INTSRS_SRS_10BIT_0_004mT  (0x04)
#define M1120_VAL_INTSRS_SRS_8BIT_0_272mT   (0x00)
#define M1120_VAL_INTSRS_SRS_8BIT_0_136mT   (0x01)
#define M1120_VAL_INTSRS_SRS_8BIT_0_068mT   (0x02)
#define M1120_VAL_INTSRS_SRS_8BIT_0_036mT   (0x03)
#define M1120_VAL_INTSRS_SRS_8BIT_0_016mT   (0x04)

#define M1120_REG_LTHL                      (0x02)
/* [7:0]   LTHL        : low byte of low threshold value */

#define M1120_REG_LTHH                      (0x03)
/* [7:6]   LTHH        : high 2bits of low threshold value with sign */

#define M1120_REG_HTHL                      (0x04)
/* [7:0]   HTHL        : low byte of high threshold value */

#define M1120_REG_HTHH                      (0x05)
/* [7:6]   HTHH        : high 2bits of high threshold value with sign */

#define M1120_REG_I2CDIS                    (0x06)
#define M1120_VAL_I2CDISABLE                (0x37)
/* [7:0]   I2CDIS      : disable i2c */

#define M1120_REG_SRST                      (0x07)
#define M1120_VAL_SRST_RESET                (0x01)
/* [0]     SRST    = 1 : soft reset */

#define M1120_REG_OPF                       (0x08)
#define M1120_VAL_OPF_FREQ_20HZ             (0x00)
#define M1120_VAL_OPF_FREQ_10HZ             (0x10)
#define M1120_VAL_OPF_FREQ_6_7HZ            (0x20)
#define M1120_VAL_OPF_FREQ_5HZ              (0x30)
#define M1120_VAL_OPF_FREQ_80HZ             (0x40)
#define M1120_VAL_OPF_FREQ_40HZ             (0x50)
#define M1120_VAL_OPF_FREQ_26_7HZ           (0x60)
#define M1120_VAL_OPF_EFRD_ON               (0x08)
#define M1120_VAL_OPF_BIT_8                 (0x02)
#define M1120_VAL_OPF_BIT_10                (0x00)
#define M1120_VAL_OPF_HSSON_ON              (0x01)
#define M1120_VAL_OPF_HSSON_OFF             (0x00)

#define M1120_REG_DID                       (0x09)
#define M1120_VAL_DID                       (0x9C)
/* [7:0]   DID         : Device ID */

#define M1120_REG_INFO                      (0x0A)
/*  [7:0]   INFO        : Information about IC */

#define M1120_REG_ASA                       (0x0B)
/* [7:0]   ASA         : Hall Sensor sensitivity adjustment */

#define M1120_REG_ST1                       (0x10)
#define M1120_VAL_ST1_DRDY                  (0x01)
/* [4] INTM            : status of interrupt mode
   [1] BITM            : status of resolution
   [0] DRDY            : status of data ready */

#define M1120_REG_HSL                       (0x11)
/* [7:0]   HSL         : low byte of hall sensor measurement data */

#define M1120_REG_HSH                       (0x12)
/* [7:6]   HSL         : high 2bits of hall sensor measurement data with sign */



 /*********************************************************************
    define specific parameters to use with step motor in front camera
 **********************************************************************/
#define M1120_DRIVER_NAME_UP                "m1120_up"
#define M1120_DRIVER_NAME_DOWN              "m1120_down"

#define M1120_DETECTION_MODE                M1120_DETECTION_MODE_INTERRUPT
#define M1120_INTERRUPT_TYPE                M1120_VAL_INTSRS_INTTYPE_WITHIN
#define M1120_SENSITIVITY_TYPE              M1120_VAL_INTSRS_SRS_10BIT_0_034mT
#define M1120_PERSISTENCE_COUNT             (0x40)
#define M1120_OPERATION_FREQUENCY           M1120_VAL_OPF_FREQ_80HZ
#define M1120_OPERATION_RESOLUTION          M1120_VAL_OPF_BIT_10

#define M1120_DELAY_MAX                     (200)   // ms
#define M1120_DELAY_MIN                     (20)    // ms

#define M1120_I2C_BUF_SIZE                  (17)
#define M1120_REG_NUM                       (15)

//MagnaChip Hall Sensor power supply VDD 2.7V~3.6V, VIO 1.65~VDD
#define M1120_VDD_MIN_UV                    2700000
#define M1120_VDD_MAX_UV                    3600000
#define M1120_VIO_MIN_UV                    1650000
#define M1120_VIO_MAX_UV                    3600000

 /*********************************************************************
                               define struct data type 
 **********************************************************************/
enum {
    OPERATION_MODE_POWERDOWN,
    OPERATION_MODE_MEASUREMENT,
    OPERATION_MODE_FUSEROMACCESS
};

typedef union {
    struct {
        unsigned char persint;
        unsigned char intsrs;
        unsigned char lthl;
        unsigned char lthh;
        unsigned char hthl;
        unsigned char hthh;
        unsigned char i2cdis;
        unsigned char srst;
        unsigned char opf;
        unsigned char did;
        unsigned char info;
        unsigned char asa;
        unsigned char st1;
        unsigned char hsl;
        unsigned char hsh;
    } map;
    unsigned char array[M1120_REG_NUM];
} m1120_reg_t;

typedef struct {
    struct i2c_client*  client;
    m1120_reg_t         reg;
    bool                irq_enabled;
    int                 power_vdd;
    int                 irq;
    int                 irq_gpio;
    struct regulator*   vdd;
    bool                power_enabled;
    atomic_t            device_enable;
} dhall_data_t;

#endif // __MXM1120_H__

