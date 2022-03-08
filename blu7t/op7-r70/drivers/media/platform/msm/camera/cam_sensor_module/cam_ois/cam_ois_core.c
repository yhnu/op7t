/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/firmware.h>
#include <cam_sensor_cmn_header.h>
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"

#include <linux/string.h>
#include <linux/time.h>
#include "ois_fw/Ois.h"

//18821 master semco
#include "ois_fw/LC898124EP3_Code_0_1_2_2_0_0.h"		// Gyro=LSM6DSM,	SPI Maste	SO2820_T1
#include "ois_fw/LC898124EP3_Code_0_1_3_2_1_0.h"		// Gyro=LSM6DSM,	SPI Slave	SO3600_T1

//18821 master ofilm
#include "ois_fw/LC898124EP3_Code_0_2_6_2_0_0.h"		// Gyro=LSM6DSM,	SPI Maste	SO2820_T1
#include "ois_fw/LC898124EP3_Code_0_2_6_2_0_1.h"		// Gyro=LSM6DSM,	SPI Slave	SO3600_T1

//18821 slave semco
#include "ois_fw/LC898124EP3_Code_0_1_2_3_0_0.h"		// Gyro=BMI160,		SPI Maste	SO2820_T1
#include "ois_fw/LC898124EP3_Code_0_1_3_3_1_0.h"		// Gyro=BMI160,		SPI Slave	SO3600_T1

//18821 slave ofilm
#include "ois_fw/LC898124EP3_Code_0_2_7_2_1_0.h"		// Gyro=BMI160,		SPI Maste	SO2820_T1
#include "ois_fw/LC898124EP3_Code_0_2_7_2_1_1.h"		// Gyro=BMI160,		SPI Slave	SO3600_T1

//18857 semco
#include "ois_fw/LC898124EP3_Code_0_1_0_2_2_0.h"		// Gyro=LSM6DSM,	SO2821
#include "ois_fw/LC898124EP3_Code_0_1_0_2_2_1.h"		// Gyro=LSM6DSM,	FRA, SO2821

//18821 Servo on master semco
#include "ois_fw/LC898124EP3_Servo_On_Code_0_1_2_2_0_0.h"		// Gyro=LSM6DSM,	SPI Maste	SO2820_T1
#include "ois_fw/LC898124EP3_Servo_On_Code_0_1_3_2_1_0.h"		// Gyro=LSM6DSM,	SPI Slave	SO3600_T1

//18821 Servo on master ofilm
#include "ois_fw/LC898124EP3_Servo_On_Code_0_2_6_2_0_0.h"		// Gyro=LSM6DSM,	SPI Maste	SO2820_T1

//18821 Servo on slave semco
#include "ois_fw/LC898124EP3_Servo_On_Code_0_1_2_3_0_0.h"		// Gyro=BMI160,		SPI Maste	SO2820_T1
#include "ois_fw/LC898124EP3_Servo_On_Code_0_1_3_3_1_0.h"		// Gyro=BMI160,		SPI Slave	SO3600_T1

//18821 Servo on slave ofilm
#include "ois_fw/LC898124EP3_Servo_On_Code_0_2_7_2_1_0.h"		// Gyro=BMI160,		SPI Maste	SO2820_T1

//18857 Servo on semco
#include "ois_fw/LC898124EP3_Servo_On_Code_0_1_0_2_2_0.h"		// Gyro=LSM6DSM,	SO2821
#include "ois_fw/LC898124EP3_Servo_On_Code_0_1_0_2_2_1.h"		// Gyro=LSM6DSM,	FRA, SO2821


//19801 Master semco
#include "ois_fw/LC898124EP3_Code_1_1_2_2_0_0.h"        // Gyro back
#include "ois_fw/LC898124EP3_Code_2_1_2_2_0_0.h"        // Gyro front

//19801 Slave semco
#include "ois_fw/LC898124EP3_Code_1_1_3_2_1_0.h"        // Gyro back
#include "ois_fw/LC898124EP3_Code_2_1_3_2_1_0.h"        // Gyro front

//19801 Master ofilm
#include "ois_fw/LC898124EP3_Code_1_2_6_2_0_0.h"                 // Gyro=LSM6DSM,        SPI Maste            M12337        Gyro back
#include "ois_fw/LC898124EP3_Code_2_2_6_2_0_0.h"                 // Gyro=LSM6DSM,        SPI Maste            M12337        Gyro front

//19801 Slave ofilm
#include "ois_fw/LC898124EP3_Code_1_2_7_2_1_0.h"                 // Gyro=LSM6DSM,        SPI Slave            M10235        Gyro back
#include "ois_fw/LC898124EP3_Code_2_2_7_2_1_0.h"                 // Gyro=LSM6DSM,        SPI Slave            M10235        Gyro front

//18865 semco
#include "ois_fw/LC898124EP3_Code_1_1_0_2_2_0.h"		// Gyro=LSM6DSM,	SO2823

//18865 ofilm
#include "ois_fw/LC898124EP3_Code_1_2_1_2_2_0.h"		 // Gyro=LSM6DSM,	                      M12337

#define MAX_DATA_NUM 64
#define MASTER_CCI_ADDR (0x7C >> 1)
#define SLAVE_CCI_ADDR (0x74 >> 1)

#define BURST_LENGTH_PM (12*5)
#define BURST_LENGTH_DM (10*6)
#define BURST_LENGTH BURST_LENGTH_PM

static bool imx586_ois_initialized = false;
static bool s5k3m5_ois_initialized = false;
static bool imx586_ois_ready = false;
static bool s5k3m5_ois_ready = false;
static struct cam_sensor_i2c_reg_array *i2c_write_setting_gl = NULL;
extern struct cam_ois_ctrl_t *ctrl_wide;
extern struct cam_ois_ctrl_t *ctrl_tele;
enum cci_i2c_master_t imx586_cci_master = MASTER_MAX;

typedef struct {
	INT32				SiSampleNum ;			// Measure Sample Number
	INT32				SiSampleMax ;			// Measure Sample Number Max

	struct {
		INT32			SiMax1 ;				// Max Measure Result
		INT32			SiMin1 ;				// Min Measure Result
		UINT32	UiAmp1 ;				// Amplitude Measure Result
		INT64		LLiIntegral1 ;			// Integration Measure Result
		INT64		LLiAbsInteg1 ;			// Absolute Integration Measure Result
		INT32			PiMeasureRam1 ;			// Measure Delay RAM Address
	} MeasureFilterA ;

	struct {
		INT32			SiMax2 ;				// Max Measure Result
		INT32			SiMin2 ;				// Min Measure Result
		UINT32	UiAmp2 ;				// Amplitude Measure Result
		INT64		LLiIntegral2 ;			// Integration Measure Result
		INT64		LLiAbsInteg2 ;			// Absolute Integration Measure Result
		INT32			PiMeasureRam2 ;			// Measure Delay RAM Address
	} MeasureFilterB ;
} MeasureFunction_Type ;

//**************************
//	define
//**************************
#define 	ONE_MSEC_COUNT	18			// 18.0288kHz * 18 ¨P 1ms

//#define 	HALL_ADJ		0
#define 	LOOPGAIN		1
#define 	THROUGH			2
#define 	NOISE			3
#define		OSCCHK			4

#define		CNT050MS		 676
#define		CNT100MS		1352
#define		CNT200MS		2703

#define         MODEL_0                 1
#define         MODEL_0_SERVO           2
#define         MODEL_1                 3
#define         MODEL_2                 4

//18857
const DOWNLOAD_TBL DTbl[] = {
 {0x0002, MODEL_0       , LC898124EP3_PM_0_1_0_2_2_0, LC898124EP3_PMSize_0_1_0_2_2_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_0_2_2_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_0_2_2_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_0_2_2_0), LC898124EP3_DM_0_1_0_2_2_0, LC898124EP3_DMA_ByteSize_0_1_0_2_2_0 , LC898124EP3_DMB_ByteSize_0_1_0_2_2_0 },
 {0x0082, MODEL_0       , LC898124EP3_PM_0_1_0_2_2_1, LC898124EP3_PMSize_0_1_0_2_2_1, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_0_2_2_1 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_0_2_2_1 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_0_2_2_1), LC898124EP3_DM_0_1_0_2_2_1, LC898124EP3_DMA_ByteSize_0_1_0_2_2_1 , LC898124EP3_DMB_ByteSize_0_1_0_2_2_1 },
 {0x0002, MODEL_0_SERVO , LC898124EP3_SERVO_ON_PM_0_1_0_2_2_0, LC898124EP3_SERVO_ON_PMSize_0_1_0_2_2_0, (UINT32)((UINT32)LC898124EP3_SERVO_ON_PMCheckSum_0_1_0_2_2_0 + (UINT32)LC898124EP3_SERVO_ON_DMA_CheckSum_0_1_2_2_0_0 + (UINT32)LC898124EP3_SERVO_ON_DMB_CheckSum_0_1_0_2_2_0), LC898124EP3_SERVO_ON_DM_0_1_0_2_2_0, LC898124EP3_SERVO_ON_DMA_ByteSize_0_1_0_2_2_0 , LC898124EP3_SERVO_ON_DMB_ByteSize_0_1_0_2_2_0 },
 {0x0002, MODEL_1       , LC898124EP3_PM_1_1_0_2_2_0, LC898124EP3_PMSize_1_1_0_2_2_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_1_1_0_2_2_0 + (UINT32)LC898124EP3_DMA_CheckSum_1_1_0_2_2_0 + (UINT32)LC898124EP3_DMB_CheckSum_1_1_0_2_2_0), LC898124EP3_DM_1_1_0_2_2_0, LC898124EP3_DMA_ByteSize_1_1_0_2_2_0 , LC898124EP3_DMB_ByteSize_1_1_0_2_2_0 },
 {0x0102, MODEL_1       , LC898124EP3_PM_1_2_1_2_2_0, LC898124EP3_PMSize_1_2_1_2_2_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_1_2_1_2_2_0 + (UINT32)LC898124EP3_DMA_CheckSum_1_2_1_2_2_0 + (UINT32)LC898124EP3_DMB_CheckSum_1_2_1_2_2_0), LC898124EP3_DM_1_2_1_2_2_0, LC898124EP3_DMA_ByteSize_1_2_1_2_2_0 , LC898124EP3_DMB_ByteSize_1_2_1_2_2_0 },
 {0xFFFF, MODEL_0       , (void*)0, 0, 0, (void*)0 ,0 ,0 }
};

//18821 master
const DOWNLOAD_TBL DTbl_M[] = {
 {0x0202, MODEL_0       , LC898124EP3_PM_0_1_2_2_0_0, LC898124EP3_PMSize_0_1_2_2_0_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_2_2_0_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_2_2_0_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_2_2_0_0), LC898124EP3_DM_0_1_2_2_0_0, LC898124EP3_DMA_ByteSize_0_1_2_2_0_0 , LC898124EP3_DMB_ByteSize_0_1_2_2_0_0 },
 {0x0203, MODEL_0       , LC898124EP3_PM_0_1_2_3_0_0, LC898124EP3_PMSize_0_1_2_3_0_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_2_3_0_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_2_3_0_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_2_3_0_0), LC898124EP3_DM_0_1_2_3_0_0, LC898124EP3_DMA_ByteSize_0_1_2_3_0_0 , LC898124EP3_DMB_ByteSize_0_1_2_3_0_0 },
 {0x0602, MODEL_0       , LC898124EP3_PM_0_2_6_2_0_0, LC898124EP3_PMSize_0_2_6_2_0_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_2_6_2_0_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_2_6_2_0_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_2_6_2_0_0), LC898124EP3_DM_0_2_6_2_0_0, LC898124EP3_DMA_ByteSize_0_2_6_2_0_0 , LC898124EP3_DMB_ByteSize_0_2_6_2_0_0 },
 {0x0202, MODEL_0_SERVO , LC898124EP3_SERVO_ON_PM_0_1_2_2_0_0, LC898124EP3_SERVO_ON_PMSize_0_1_2_2_0_0, (UINT32)((UINT32)LC898124EP3_SERVO_ON_PMCheckSum_0_1_2_2_0_0 + (UINT32)LC898124EP3_SERVO_ON_DMA_CheckSum_0_1_2_2_0_0 + (UINT32)LC898124EP3_SERVO_ON_DMB_CheckSum_0_1_2_2_0_0), LC898124EP3_SERVO_ON_DM_0_1_2_2_0_0, LC898124EP3_SERVO_ON_DMA_ByteSize_0_1_2_2_0_0 , LC898124EP3_SERVO_ON_DMB_ByteSize_0_1_2_2_0_0 },
 {0x0602, MODEL_0_SERVO , LC898124EP3_SERVO_ON_PM_0_2_6_2_0_0, LC898124EP3_SERVO_ON_PMSize_0_2_6_2_0_0, (UINT32)((UINT32)LC898124EP3_SERVO_ON_PMCheckSum_0_2_6_2_0_0 + (UINT32)LC898124EP3_SERVO_ON_DMA_CheckSum_0_2_6_2_0_0 + (UINT32)LC898124EP3_SERVO_ON_DMB_CheckSum_0_2_6_2_0_0), LC898124EP3_SERVO_ON_DM_0_2_6_2_0_0, LC898124EP3_SERVO_ON_DMA_ByteSize_0_2_6_2_0_0 , LC898124EP3_SERVO_ON_DMB_ByteSize_0_2_6_2_0_0 },
 {0x0202, MODEL_1       ,LC898124EP3_PM_1_1_2_2_0_0, LC898124EP3_PMSize_1_1_2_2_0_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_1_1_2_2_0_0 + (UINT32)LC898124EP3_DMA_CheckSum_1_1_2_2_0_0 + (UINT32)LC898124EP3_DMB_CheckSum_1_1_2_2_0_0), LC898124EP3_DM_1_1_2_2_0_0, LC898124EP3_DMA_ByteSize_1_1_2_2_0_0 , LC898124EP3_DMB_ByteSize_1_1_2_2_0_0 },
 {0x0202, MODEL_2       ,LC898124EP3_PM_2_1_2_2_0_0, LC898124EP3_PMSize_2_1_2_2_0_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_2_1_2_2_0_0 + (UINT32)LC898124EP3_DMA_CheckSum_2_1_2_2_0_0 + (UINT32)LC898124EP3_DMB_CheckSum_2_1_2_2_0_0), LC898124EP3_DM_2_1_2_2_0_0, LC898124EP3_DMA_ByteSize_2_1_2_2_0_0 , LC898124EP3_DMB_ByteSize_2_1_2_2_0_0 },
 {0xFFFF, 1, (void*)0, 0, 0, (void*)0 ,0 ,0 }
};

//18821 slave
const DOWNLOAD_TBL DTbl_S[] = {
 {0x0302, MODEL_0       , LC898124EP3_PM_0_1_3_2_1_0, LC898124EP3_PMSize_0_1_3_2_1_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_3_2_1_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_3_2_1_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_3_2_1_0), LC898124EP3_DM_0_1_3_2_1_0, LC898124EP3_DMA_ByteSize_0_1_3_2_1_0 , LC898124EP3_DMB_ByteSize_0_1_3_2_1_0 },
 {0x0303, MODEL_0       , LC898124EP3_PM_0_1_3_3_1_0, LC898124EP3_PMSize_0_1_3_3_1_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_3_3_1_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_3_3_1_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_3_3_1_0), LC898124EP3_DM_0_1_3_3_1_0, LC898124EP3_DMA_ByteSize_0_1_3_3_1_0 , LC898124EP3_DMB_ByteSize_0_1_3_3_1_0 },
 {0x0702, MODEL_0       , LC898124EP3_PM_0_2_7_2_1_0, LC898124EP3_PMSize_0_2_7_2_1_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_2_7_2_1_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_2_7_2_1_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_2_7_2_1_0), LC898124EP3_DM_0_2_7_2_1_0, LC898124EP3_DMA_ByteSize_0_2_7_2_1_0 , LC898124EP3_DMB_ByteSize_0_2_7_2_1_0 },
 {0x0302, MODEL_0_SERVO , LC898124EP3_SERVO_ON_PM_0_1_3_2_1_0, LC898124EP3_SERVO_ON_PMSize_0_1_3_2_1_0, (UINT32)((UINT32)LC898124EP3_SERVO_ON_PMCheckSum_0_1_3_2_1_0 + (UINT32)LC898124EP3_SERVO_ON_DMA_CheckSum_0_1_3_2_1_0 + (UINT32)LC898124EP3_SERVO_ON_DMB_CheckSum_0_1_3_2_1_0), LC898124EP3_SERVO_ON_DM_0_1_3_2_1_0, LC898124EP3_SERVO_ON_DMA_ByteSize_0_1_3_2_1_0 , LC898124EP3_SERVO_ON_DMB_ByteSize_0_1_3_2_1_0 },
 {0x0702, MODEL_0_SERVO , LC898124EP3_SERVO_ON_PM_0_2_7_2_1_0, LC898124EP3_SERVO_ON_PMSize_0_2_7_2_1_0, (UINT32)((UINT32)LC898124EP3_SERVO_ON_PMCheckSum_0_2_7_2_1_0 + (UINT32)LC898124EP3_SERVO_ON_DMA_CheckSum_0_2_7_2_1_0 + (UINT32)LC898124EP3_SERVO_ON_DMB_CheckSum_0_2_7_2_1_0), LC898124EP3_SERVO_ON_DM_0_2_7_2_1_0, LC898124EP3_SERVO_ON_DMA_ByteSize_0_2_7_2_1_0 , LC898124EP3_SERVO_ON_DMB_ByteSize_0_2_7_2_1_0 },
 {0x0302, MODEL_1       , LC898124EP3_PM_1_1_3_2_1_0, LC898124EP3_PMSize_1_1_3_2_1_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_1_1_3_2_1_0 + (UINT32)LC898124EP3_DMA_CheckSum_1_1_3_2_1_0 + (UINT32)LC898124EP3_DMB_CheckSum_1_1_3_2_1_0), LC898124EP3_DM_1_1_3_2_1_0, LC898124EP3_DMA_ByteSize_1_1_3_2_1_0 , LC898124EP3_DMB_ByteSize_1_1_3_2_1_0 },
 {0x0302, MODEL_2       , LC898124EP3_PM_2_1_3_2_1_0, LC898124EP3_PMSize_2_1_3_2_1_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_2_1_3_2_1_0 + (UINT32)LC898124EP3_DMA_CheckSum_2_1_3_2_1_0 + (UINT32)LC898124EP3_DMB_CheckSum_2_1_3_2_1_0), LC898124EP3_DM_2_1_3_2_1_0, LC898124EP3_DMA_ByteSize_2_1_3_2_1_0 , LC898124EP3_DMB_ByteSize_2_1_3_2_1_0 },
 {0xFFFF, 1, (void*)0, 0, 0, (void*)0 ,0 ,0 }
};

static int RamWrite32A(struct cam_ois_ctrl_t *o_ctrl,
    UINT32 addr, UINT32 data)
{
    int32_t rc = 0;
    int retry = 3;
    int i;
    struct cam_sensor_i2c_reg_array i2c_write_setting = {
        .reg_addr = addr,
        .reg_data = data,
        .delay = 0x00,
        .data_mask = 0x00,
    };
    struct cam_sensor_i2c_reg_setting i2c_write = {
        .reg_setting = &i2c_write_setting,
        .size = 1,
        .addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
        .data_type = CAMERA_SENSOR_I2C_TYPE_DWORD,
        .delay = 0x00,
    };

    if (o_ctrl == NULL) {
        CAM_ERR(CAM_OIS, "Invalid Args");
        return -EINVAL;
    }

    for(i = 0; i < retry; i++)
    {
        rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_write);
        if (rc < 0) {
            CAM_ERR(CAM_OIS, "write 0x%04x failed, retry:%d", addr, i+1);
        } else {
            return rc;
        }
    }
    return rc;
}

static int RamRead32A(struct cam_ois_ctrl_t *o_ctrl,
    UINT32 addr, UINT32* data)
{
    int32_t rc = 0;
    int retry = 3;
    int i;
    if (o_ctrl == NULL) {
        CAM_ERR(CAM_OIS, "Invalid Args");
        return -EINVAL;
    }
    for(i = 0; i < retry; i++)
    {
        rc = camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)addr, (uint32_t *)data,
            CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD);
        if (rc < 0) {
            CAM_ERR(CAM_OIS, "read 0x%04x failed, retry:%d", addr, i+1);
        } else {
            return rc;
        }
    }
    return rc;
}

static void CntWrt(struct cam_ois_ctrl_t *o_ctrl,
    UINT8 *data, UINT16 size)
{
    int32_t rc = 0;
    int i = 0;
    int reg_data_cnt = size - 1;
    int continue_cnt = 0;
    int retry = 3;
    struct cam_sensor_i2c_reg_setting i2c_write;

    if (o_ctrl == NULL) {
        CAM_ERR(CAM_OIS, "Invalid Args");
        return;
    }

    if (i2c_write_setting_gl == NULL) {
        i2c_write_setting_gl = (struct cam_sensor_i2c_reg_array *)kzalloc(
            sizeof(struct cam_sensor_i2c_reg_array)*MAX_DATA_NUM, GFP_KERNEL);
        if(!i2c_write_setting_gl) {
            CAM_ERR(CAM_OIS, "Alloc i2c_write_setting_gl failed");
            return;
        }
    }

    for(i = 0; i< reg_data_cnt; i++) {
        if (i == 0) {
            i2c_write_setting_gl[continue_cnt].reg_addr = data[0];
            i2c_write_setting_gl[continue_cnt].reg_data = data[1];
            i2c_write_setting_gl[continue_cnt].delay = 0x00;
            i2c_write_setting_gl[continue_cnt].data_mask = 0x00;
        } else {
            i2c_write_setting_gl[continue_cnt].reg_data = data[i+1];
            i2c_write_setting_gl[continue_cnt].delay = 0x00;
            i2c_write_setting_gl[continue_cnt].data_mask = 0x00;
        }
        continue_cnt++;
    }
    i2c_write.reg_setting = i2c_write_setting_gl;
    i2c_write.size = continue_cnt;
    i2c_write.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    i2c_write.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    i2c_write.delay = 0x00;

    for(i = 0; i < retry; i++)
    {
        rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
            &i2c_write, 1);
        if (rc < 0) {
            CAM_ERR(CAM_OIS, "Continue write failed, rc:%d, retry:%d", rc, i+1);
        } else {
            break;
        }
    }
}

static void DMIOWrite32(struct cam_ois_ctrl_t *o_ctrl, UINT32 IOadrs, UINT32 IOdata )
{
#if 1
    UINT8 data[10];
    data[0] = 0xC0;		// Pmem address set
    data[1] = 0x00;		// Command High
    data[2] = (UINT8)(IOadrs >>24);		// IOadres
    data[3] = (UINT8)(IOadrs >>16);		// Command High
    data[4] = (UINT8)(IOadrs >> 8);		// Command High
    data[5] = (UINT8)(IOadrs >> 0);		// Command High
    data[6] = (UINT8)(IOdata >>24);		// IOadres
    data[7] = (UINT8)(IOdata >>16);		// Command High
    data[8] = (UINT8)(IOdata >> 8);		// Command High
    data[9] = (UINT8)(IOdata >> 0);		// Command High
    CntWrt(o_ctrl, data, 10 ); 	// I2C 1Byte address.
#else
    RamWrite32A( CMD_IO_ADR_ACCESS, IOadrs ) ;
    RamWrite32A( CMD_IO_DAT_ACCESS, IOdata ) ;
#endif
};

//********************************************************************************
// Function Name 	: MonitorInfo
// Retun Value		: NON
// Argment Value	: NON
// Explanation		:
// History			: Second edition
//********************************************************************************
void MonitorInfo( DSPVER* Dspcode )
{
    CAM_INFO(CAM_OIS, "Vendor : %02x \n", Dspcode->Vendor);
    CAM_INFO(CAM_OIS, "User : %02x \n", Dspcode->User);
    CAM_INFO(CAM_OIS, "Model : %02x \n", Dspcode->Model);
    CAM_INFO(CAM_OIS, "Version : %02x \n", Dspcode->Version);

    if(Dspcode->SpiMode == SPI_MST)
        CAM_INFO(CAM_OIS, "spi mode : Master\n");
    if(Dspcode->SpiMode == SPI_SLV)
        CAM_INFO(CAM_OIS, "spi mode : Slave\n");
    if(Dspcode->SpiMode == SPI_SNGL)
        CAM_INFO(CAM_OIS, "spi mode : only master\n");

    if(Dspcode->ActType == ACT_SO2820) {
        CAM_INFO(CAM_OIS, "actuator type : SO2820\n");
    } else if(Dspcode->ActType == ACT_SO3600) {
        CAM_INFO(CAM_OIS, "actuator type : SO3600\n");
    } else {
        CAM_INFO(CAM_OIS, "actuator type : SOXXXX\n");
    }

    if(Dspcode->GyroType == GYRO_ICM20690)
        CAM_INFO(CAM_OIS, "gyro type : INVEN ICM20690 \n");
    if(Dspcode->GyroType == GYRO_LSM6DSM)
        CAM_INFO(CAM_OIS, "gyro type : ST LSM6DSM \n");

}

//********************************************************************************
// Function Name 	: GetInfomationBeforeDownlaod
// Retun Value		: True(0) / Fail(1)
// Argment Value	: NON
// Explanation		: <Pmem Memory> Write Data
// History			: First edition
//********************************************************************************
UINT8 GetInfomationBeforeDownload( DSPVER* Info, const UINT8* DataDM,  UINT32 LengthDM )
{
	UINT32 i;
	Info->ActType = 0;
	Info->GyroType = 0;

	for( i=0; i < LengthDM; i+=6 )
	{
		if ( (DataDM[0+i] == 0xA0) && (DataDM[1+i] == 0x00) )
		{
			Info->Vendor = DataDM[2+i];
			Info->User = DataDM[3+i];
			Info->Model = DataDM[4+i];
			Info->Version = DataDM[5+i];
			if ( (DataDM[6+i] == 0xA0) && (DataDM[7+i] == 0x04) )
			{
				Info->SpiMode = DataDM[8+i];
				Info->ActType = DataDM[10+i];
				Info->GyroType = DataDM[11+i];
			}
			MonitorInfo( Info );
			return (0);
		}
	}
	return(1);
}

//********************************************************************************
// Function Name 	: DownloadToEP3
// Retun Value		: NON
// Argment Value	: PMlength: 5byte unit, DMlength : 1Byte unit
// Explanation		: <Pmem Memory> Write Data
// History			: First edition
//********************************************************************************
unsigned char DownloadToEP3(struct cam_ois_ctrl_t *o_ctrl, const UINT8* DataPM, UINT32 LengthPM, UINT32 Parity, const UINT8* DataDM, UINT32 LengthDMA , UINT32 LengthDMB )
{
    UINT32 i, j;
    UINT8 data[MAX_DATA_NUM];		// work fifo buffer max size 64 byte
    UINT8 Remainder;
    UINT32 UlReadVal, UlCnt;
    UINT32 ReadVerifyPM = 0, ReadVerifyDMA = 0, ReadVerifyDMB = 0;	// Checksum
    UINT32 VerifySUM = 0;

//*******************************************************************************//
//*   pre-check ROM code version 												*//
//*******************************************************************************//
    RamRead32A(o_ctrl, CMD_ROMVER , &UlReadVal );
    if( UlReadVal == OLD_VER )	return( 3 );		/* ROM code version error */

//--------------------------------------------------------------------------------
// 0. Start up to boot exection
//--------------------------------------------------------------------------------
    RamWrite32A(o_ctrl, CMD_IO_ADR_ACCESS , ROMINFO );
    RamRead32A(o_ctrl, CMD_IO_DAT_ACCESS, &UlReadVal );
    switch ( (UINT8)UlReadVal ){
        case 0x0A:	/* Normal Rom program execution */
            break;

        case 0x01:	/* Normal Ram program execution */
            DMIOWrite32(o_ctrl, SYSDSP_REMAP, 0x00001000 ); 	// CORE_RST
            msleep(6) ;							// Boot 6msec
            break;

//	case 0x0B:
//	case 0x08:
        default:
            return( 1 );
    }
//--------------------------------------------------------------------------------
// 1. Download Program
//--------------------------------------------------------------------------------
    data[0] = 0x30;		// Pmem address set
    data[1] = 0x00;		// Command High
    data[2] = 0x10;		// Command High
    data[3] = 0x00;		// Command High
    data[4] = 0x00;		// Command High
    CntWrt(o_ctrl, data, 5 ); 	// I2C 1Byte address.

    // program start
    data[0] = 0x40;		// Pmem address set
    Remainder = ( (LengthPM*5) / BURST_LENGTH_PM );
    for(i=0 ; i< Remainder ; i++)
    {
        UlCnt = 1;
        for(j=0 ; j < BURST_LENGTH_PM; j++)	data[UlCnt++] = *DataPM++;

        CntWrt(o_ctrl, data, BURST_LENGTH_PM+1 );  // I2Caddresss 1Byte.
    }
    Remainder = ( (LengthPM*5) % BURST_LENGTH_PM);
    if (Remainder != 0 )
    {
        UlCnt = 1;
        for(j=0 ; j < Remainder; j++)	data[UlCnt++] = *DataPM++;
        CntWrt(o_ctrl, data, UlCnt );  // I2C 1Byte address.
    }
    // Chercksum start
    data[0] = 0xF0;											// Pmem address set
    data[1] = 0x0A;											// Command High
    data[2] = (unsigned char)(( LengthPM & 0xFF00) >> 8 );	// Size High
    data[3] = (unsigned char)(( LengthPM & 0x00FF) >> 0 );	// Size Low
    CntWrt(o_ctrl, data, 4 ); 	// I2C 2Byte addresss.

//--------------------------------------------------------------------------------
// 2. Download Table Data
//--------------------------------------------------------------------------------
    RamWrite32A(o_ctrl, DmCheck_CheckSumDMA, 0 );		// DMA Parity Clear
    RamWrite32A(o_ctrl, DmCheck_CheckSumDMB, 0 );		// DMB Parity Clear

    /***** DMA Data Send *****/
    Remainder = ( (LengthDMA*6/4) / BURST_LENGTH_DM );
    for(i=0 ; i< Remainder ; i++)
    {
        CntWrt(o_ctrl, (UINT8*)DataDM, BURST_LENGTH_DM );  // I2Caddresss 1Byte.
        DataDM += BURST_LENGTH_DM;
    }
    Remainder = ( (LengthDMA*6/4) % BURST_LENGTH_DM );
    if (Remainder != 0 )
    {
        CntWrt(o_ctrl, (UINT8*)DataDM, (UINT8)Remainder );  // I2Caddresss 1Byte.
    }
    DataDM += Remainder;

    /***** DMB Data Send *****/
    Remainder = ( (LengthDMB*6/4) / BURST_LENGTH_DM );
    for( i=0 ; i< Remainder ; i++)
    {
        CntWrt(o_ctrl, (UINT8*)DataDM, BURST_LENGTH_DM );  // I2Caddresss 1Byte.
        DataDM += BURST_LENGTH_DM;
    }
    Remainder = ( (LengthDMB*6/4) % BURST_LENGTH_DM );
    if (Remainder != 0 )
    {
        CntWrt(o_ctrl, (UINT8*)DataDM, (UINT8)Remainder );  // I2Caddresss 1Byte.
    }

//--------------------------------------------------------------------------------
// 3. Verify
//--------------------------------------------------------------------------------
    RamRead32A(o_ctrl, PmCheck_CheckSum, &ReadVerifyPM );
    RamRead32A(o_ctrl, DmCheck_CheckSumDMA, &ReadVerifyDMA );
    RamRead32A(o_ctrl, DmCheck_CheckSumDMB, &ReadVerifyDMB );
    VerifySUM = ReadVerifyPM + ReadVerifyDMA + ReadVerifyDMB;
    if(VerifySUM == Parity){
        CAM_ERR(CAM_OIS, "verify success. ReadVerifyPM=0x%x, ReadVerifyDMA=0x%x, ReadVerifyDMB=0x%x, VerifySUM=0x%x, Parity=0x%x",
            ReadVerifyPM, ReadVerifyDMA, ReadVerifyDMB, VerifySUM, Parity);
    } else {
        CAM_ERR(CAM_OIS, "verify fail. ReadVerifyPM=0x%x, ReadVerifyDMA=0x%x, ReadVerifyDMB=0x%x, VerifySUM=0x%x, Parity=0x%x",
            ReadVerifyPM, ReadVerifyDMA, ReadVerifyDMB, VerifySUM, Parity);
        return( 2 );
    }
    return(0);
}

unsigned char SelectDownload(struct cam_ois_ctrl_t *o_ctrl, UINT8 GyroSelect, UINT8 ActSelect, UINT8 MasterSlave, UINT8 FWType)
{
	DSPVER Dspcode;
	DOWNLOAD_TBL *ptr;
	CAM_INFO(CAM_OIS, "ois_name:%s, GyroSelect:0x%x, ActSelect:0x%x, MasterSlave:0x%x, FWType:%d\n", o_ctrl->ois_name, GyroSelect, ActSelect, MasterSlave, FWType);

   if(o_ctrl->ois_gyro_id==3){
    ptr = ( DOWNLOAD_TBL *)DTbl;
   }else{
      if ( MasterSlave == 0x00 ) {
            ptr = ( DOWNLOAD_TBL *)DTbl_M;
      } else {
            ptr = ( DOWNLOAD_TBL *)DTbl_S;
      }
   }

	while (ptr->Cmd != 0xFFFF ){
		if( (ptr->Cmd == ( ((UINT16)ActSelect<<8) + GyroSelect)) && (ptr->FWType == FWType) ) break;
		ptr++ ;
	}
	if (ptr->Cmd == 0xFFFF)	return(0xF0);

	if( GetInfomationBeforeDownload( &Dspcode, ptr->DataDM, ( ptr->LengthDMA +  ptr->LengthDMB ) ) != 0 ){
		return(0xF1);
	}

	if( (ActSelect != Dspcode.ActType) || ((GyroSelect&0x7f) != Dspcode.GyroType) ) return(0xF2);

	return( DownloadToEP3(o_ctrl, ptr->DataPM, ptr->LengthPM, ptr->Parity, ptr->DataDM, ptr->LengthDMA , ptr->LengthDMB ) );
}

//********************************************************************************
// Function Name 	: SetGyroAccelCoef
// Retun Value		: non
// Argment Value	:
// Explanation		: Set Gyro Coef and Accel Coef
// History			: First edition
//********************************************************************************
void SetGyroAccelCoef(struct cam_ois_ctrl_t *o_ctrl, UINT8 SelectAct ){
	CAM_INFO(CAM_OIS, "SetGyroAccelCoef SelectAct: %d", SelectAct);
	switch(SelectAct) {
        case ACT_SO2820 :
                    if (MODEL_1 == o_ctrl->ois_fw_flag)
                    {
                        CAM_INFO(CAM_OIS, "SetGyroAccelCoef ACT_SO2820 MODEL_1: %d", MODEL_1);
                        RamWrite32A(o_ctrl, GCNV_XX, (UINT32) 0x80000001);
                        RamWrite32A(o_ctrl, GCNV_XY, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, GCNV_YY, (UINT32) 0x7FFFFFFF);
                        RamWrite32A(o_ctrl, GCNV_YX, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, GCNV_ZP, (UINT32) 0x7FFFFFFF);

                        RamWrite32A(o_ctrl, ACNV_XX, (UINT32) 0x7FFFFFFF);
                        RamWrite32A(o_ctrl, ACNV_XY, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, ACNV_YY, (UINT32) 0x7FFFFFFF);
                        RamWrite32A(o_ctrl, ACNV_YX, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, ACNV_ZP, (UINT32) 0x80000001);

                        break;
                    }
                    if (MODEL_2 == o_ctrl->ois_fw_flag)
                    {
                        CAM_INFO(CAM_OIS, "SetGyroAccelCoef ACT_SO2820 MODEL_2: %d", MODEL_1);
                        RamWrite32A(o_ctrl, GCNV_XX, (UINT32) 0x80000001);
                        RamWrite32A(o_ctrl, GCNV_XY, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, GCNV_YY, (UINT32) 0x80000001);
                        RamWrite32A(o_ctrl, GCNV_YX, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, GCNV_ZP, (UINT32) 0x80000001);

                        RamWrite32A(o_ctrl, ACNV_XX, (UINT32) 0x7FFFFFFF);
                        RamWrite32A(o_ctrl, ACNV_XY, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, ACNV_YY, (UINT32) 0x7FFFFFFF);
                        RamWrite32A(o_ctrl, ACNV_YX, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, ACNV_ZP, (UINT32) 0x7FFFFFFF);
                        break;
                    }
                    if(1 == o_ctrl->ois_gyro_id)//18821 rear
                    {
                        CAM_INFO(CAM_OIS, "SetGyroAccelCoef : gyro %d 821 rear\n", o_ctrl->ois_gyro_id);
                        RamWrite32A(o_ctrl, GCNV_XX , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, GCNV_XY , (UINT32)0x7FFFFFFF );
                        RamWrite32A(o_ctrl, GCNV_YY , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, GCNV_YX , (UINT32)0x7FFFFFFF );
                        RamWrite32A(o_ctrl, GCNV_ZP , (UINT32)0x7FFFFFFF );

                        RamWrite32A(o_ctrl, ACNV_XX , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, ACNV_XY , (UINT32)0x7FFFFFFF );
                        RamWrite32A(o_ctrl, ACNV_YY , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, ACNV_YX , (UINT32)0x7FFFFFFF );
                        RamWrite32A(o_ctrl, ACNV_ZP , (UINT32)0x7FFFFFFF );
                    }
                    if(2 == o_ctrl->ois_gyro_id)//18827 rear
                    {
                        CAM_INFO(CAM_OIS, "SetGyroAccelCoef : gyro %d 827 rear\n", o_ctrl->ois_gyro_id);
                        RamWrite32A(o_ctrl, GCNV_XX , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, GCNV_XY , (UINT32)0x7FFFFFFF );
                        RamWrite32A(o_ctrl, GCNV_YY , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, GCNV_YX , (UINT32)0x80000001 );
                        RamWrite32A(o_ctrl, GCNV_ZP , (UINT32)0x80000001 );

                        RamWrite32A(o_ctrl, ACNV_XX , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, ACNV_XY , (UINT32)0x7FFFFFFF );
                        RamWrite32A(o_ctrl, ACNV_YY , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, ACNV_YX , (UINT32)0x80000001 );
                        RamWrite32A(o_ctrl, ACNV_ZP , (UINT32)0x80000001 );
                    }
                    break;
        case ACT_SO3600:
                    if (MODEL_1 == o_ctrl->ois_fw_flag)
                    {
                        CAM_INFO(CAM_OIS, "SetGyroAccelCoef ACT_SO3600 MODEL_1: %d", MODEL_1);
                        RamWrite32A(o_ctrl, GCNV_XX, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, GCNV_XY, (UINT32) 0x80000001);
                        RamWrite32A(o_ctrl, GCNV_YY, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, GCNV_YX, (UINT32) 0x7FFFFFFF);
                        RamWrite32A(o_ctrl, GCNV_ZP, (UINT32) 0x7FFFFFFF);

                        RamWrite32A(o_ctrl, ACNV_XX, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, ACNV_XY, (UINT32) 0x7FFFFFFF);
                        RamWrite32A(o_ctrl, ACNV_YY, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, ACNV_YX, (UINT32) 0x7FFFFFFF);
                        RamWrite32A(o_ctrl, ACNV_ZP, (UINT32) 0x80000001);
                        break;
                    }
                    if (MODEL_2 == o_ctrl->ois_fw_flag)
                    {
                        CAM_INFO(CAM_OIS, "SetGyroAccelCoef ACT_SO3600 MODEL_2: %d", MODEL_2);
                        RamWrite32A(o_ctrl, GCNV_XX, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, GCNV_XY, (UINT32) 0x7FFFFFFF);
                        RamWrite32A(o_ctrl, GCNV_YY, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, GCNV_YX, (UINT32) 0x7FFFFFFF);
                        RamWrite32A(o_ctrl, GCNV_ZP, (UINT32) 0x80000001);

                        RamWrite32A(o_ctrl, ACNV_XX, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, ACNV_XY, (UINT32) 0x7FFFFFFF);
                        RamWrite32A(o_ctrl, ACNV_YY, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl, ACNV_YX, (UINT32) 0x7FFFFFFF);
                        RamWrite32A(o_ctrl, ACNV_ZP, (UINT32) 0x7FFFFFFF);

                        break;
                    }
                    if(1 == o_ctrl->ois_gyro_id)//18821 rear
                    {
                        CAM_INFO(CAM_OIS, "SetGyroAccelCoef tele: gyro %d 821 rear\n", o_ctrl->ois_gyro_id);
                        RamWrite32A(o_ctrl, GCNV_XX , (UINT32)0x80000001 );
                        RamWrite32A(o_ctrl, GCNV_XY , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, GCNV_YY , (UINT32)0x80000001 );
                        RamWrite32A(o_ctrl, GCNV_YX , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, GCNV_ZP , (UINT32)0x7FFFFFFF );

                        RamWrite32A(o_ctrl, ACNV_XX , (UINT32)0x7FFFFFFF );
                        RamWrite32A(o_ctrl, ACNV_XY , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, ACNV_YY , (UINT32)0x80000001 );
                        RamWrite32A(o_ctrl, ACNV_YX , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, ACNV_ZP , (UINT32)0x7FFFFFFF );
                    }
                    if(2 == o_ctrl->ois_gyro_id)//18827 rear
                    {
                        CAM_INFO(CAM_OIS, "SetGyroAccelCoef tele: gyro %d 827 rear\n", o_ctrl->ois_gyro_id);
                        RamWrite32A(o_ctrl, GCNV_XX , (UINT32)0x7FFFFFFF );
                        RamWrite32A(o_ctrl, GCNV_XY , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, GCNV_YY , (UINT32)0x80000001 );
                        RamWrite32A(o_ctrl, GCNV_YX , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, GCNV_ZP , (UINT32)0x7FFFFFFF );

                        RamWrite32A(o_ctrl, ACNV_XX , (UINT32)0x80000001 );
                        RamWrite32A(o_ctrl, ACNV_XY , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, ACNV_YY , (UINT32)0x80000001 );
                        RamWrite32A(o_ctrl, ACNV_YX , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, ACNV_ZP , (UINT32)0x7FFFFFFF );
                    }
                    break;
        case ACT_SO2821:
                    if (3 == o_ctrl->ois_gyro_id) //semco 18857 rear
                    {
                        switch (o_ctrl->ois_fw_flag) {
                            case MODEL_0: //18857
                                CAM_INFO(CAM_OIS, "SetGyroAccelCoef 857main : gyro %d 857 rear\n", o_ctrl->ois_gyro_id);
                                RamWrite32A(o_ctrl, GCNV_XX, (UINT32) 0x00000000);
                                RamWrite32A(o_ctrl, GCNV_XY, (UINT32) 0x7FFFFFFF);
                                RamWrite32A(o_ctrl, GCNV_YY, (UINT32) 0x00000000);
                                RamWrite32A(o_ctrl, GCNV_YX, (UINT32) 0x7FFFFFFF);
                                RamWrite32A(o_ctrl, GCNV_ZP, (UINT32) 0x7FFFFFFF);

                                RamWrite32A(o_ctrl, ACNV_XX, (UINT32) 0x00000000);
                                RamWrite32A(o_ctrl, ACNV_XY, (UINT32) 0x7FFFFFFF);
                                RamWrite32A(o_ctrl, ACNV_YY, (UINT32) 0x00000000);
                                RamWrite32A(o_ctrl, ACNV_YX, (UINT32) 0x7FFFFFFF);
                                RamWrite32A(o_ctrl, ACNV_ZP, (UINT32) 0x7FFFFFFF);
                                break;
                            case MODEL_1: //18865/18863
                                CAM_INFO(CAM_OIS, "SetGyroAccelCoef 865main : gyro %d 865 rear\n", o_ctrl->ois_gyro_id);
                                RamWrite32A(o_ctrl, GCNV_XX, (UINT32) 0x00000000);
                                RamWrite32A(o_ctrl, GCNV_XY, (UINT32) 0x7FFFFFFF);
                                RamWrite32A(o_ctrl, GCNV_YY, (UINT32) 0x00000000);
                                RamWrite32A(o_ctrl, GCNV_YX, (UINT32) 0x7FFFFFFF);
                                RamWrite32A(o_ctrl, GCNV_ZP, (UINT32) 0x7FFFFFFF);

                                RamWrite32A(o_ctrl, ACNV_XX, (UINT32) 0x00000000);
                                RamWrite32A(o_ctrl, ACNV_XY, (UINT32) 0x7FFFFFFF);
                                RamWrite32A(o_ctrl, ACNV_YY, (UINT32) 0x00000000);
                                RamWrite32A(o_ctrl, ACNV_YX, (UINT32) 0x80000001);
                                RamWrite32A(o_ctrl, ACNV_ZP, (UINT32) 0x80000001);
                        }
                    }
                    break;
        case ACT_M12337: //ofilm master 199865/19863
                    if (3 == o_ctrl->ois_gyro_id)
                    {
                        if (MODEL_1 == o_ctrl->ois_fw_flag) //master 18865 only
                        {
                            CAM_INFO(CAM_OIS, "SetGyroAccelCoef ACT_M12337 MODEL_1: %d", MODEL_1);
                            //todo: get from onsemi for ofilm model 1, currently copied from ofilm 18821
                            RamWrite32A(o_ctrl, GCNV_XX , (UINT32)0x00000000 );
                            RamWrite32A(o_ctrl, GCNV_XY , (UINT32)0x7FFFFFFF );
                            RamWrite32A(o_ctrl, GCNV_YY , (UINT32)0x00000000 );
                            RamWrite32A(o_ctrl, GCNV_YX , (UINT32)0x80000001 );
                            RamWrite32A(o_ctrl, GCNV_ZP , (UINT32)0x7FFFFFFF );

                            RamWrite32A(o_ctrl, ACNV_XX , (UINT32)0x00000000 );
                            RamWrite32A(o_ctrl, ACNV_XY , (UINT32)0x7FFFFFFF );
                            RamWrite32A(o_ctrl, ACNV_YY , (UINT32)0x00000000 );
                            RamWrite32A(o_ctrl, ACNV_YX , (UINT32)0x7FFFFFFF );
                            RamWrite32A(o_ctrl, ACNV_ZP , (UINT32)0x7FFFFFFF );
                        }
                    }
                    break;
        case ACT_M12337_A1 : //ofilm master 19801/19861
        case ACT_M10235_A1 : //ofilm slave 19801/19861
                    if (MODEL_1 == o_ctrl->ois_fw_flag) //master & slave 19801/19861
                    {
                        CAM_INFO(CAM_OIS, "SetGyroAccelCoef ACT_M12337_A1/ACT_M10235_A1 MODEL_1: %d", MODEL_1);
                        RamWrite32A(o_ctrl,GCNV_XX, (UINT32) 0x80000001);
                        RamWrite32A(o_ctrl,GCNV_XY, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl,GCNV_YY, (UINT32) 0x80000001);
                        RamWrite32A(o_ctrl,GCNV_YX, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl,GCNV_ZP, (UINT32) 0x7FFFFFFF);

                        RamWrite32A(o_ctrl,ACNV_XX, (UINT32) 0x80000001);
                        RamWrite32A(o_ctrl,ACNV_XY, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl,ACNV_YY, (UINT32) 0x80000001);
                        RamWrite32A(o_ctrl,ACNV_YX, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl,ACNV_ZP, (UINT32) 0x7FFFFFFF);
                        break;
                    }
                    if (MODEL_2 == o_ctrl->ois_fw_flag) //master & slave 19801/19861
                    {
                        CAM_INFO(CAM_OIS, "SetGyroAccelCoef ACT_M12337_A1/ACT_M10235_A1 MODEL_2: %d", MODEL_2);
                        RamWrite32A(o_ctrl,GCNV_XX, (UINT32) 0x80000001);
                        RamWrite32A(o_ctrl,GCNV_XY, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl,GCNV_YY, (UINT32) 0x7FFFFFFF);
                        RamWrite32A(o_ctrl,GCNV_YX, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl,GCNV_ZP, (UINT32) 0x7FFFFFFF);

                        RamWrite32A(o_ctrl,ACNV_XX, (UINT32) 0x80000001);
                        RamWrite32A(o_ctrl,ACNV_XY, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl,ACNV_YY, (UINT32) 0x80000001);
                        RamWrite32A(o_ctrl,ACNV_YX, (UINT32) 0x00000000);
                        RamWrite32A(o_ctrl,ACNV_ZP, (UINT32) 0x7FFFFFFF);
                        break;
                    }
                    if(1 == o_ctrl->ois_gyro_id)//18821 rear ofilm
                    {
                        CAM_INFO(CAM_OIS, "SetGyroAccelCoef : gyro %d 821 rear\n", o_ctrl->ois_gyro_id);
                        RamWrite32A(o_ctrl, GCNV_XX , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, GCNV_XY , (UINT32)0x7FFFFFFF );
                        RamWrite32A(o_ctrl, GCNV_YY , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, GCNV_YX , (UINT32)0x80000001 );
                        RamWrite32A(o_ctrl, GCNV_ZP , (UINT32)0x7FFFFFFF );

                        RamWrite32A(o_ctrl, ACNV_XX , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, ACNV_XY , (UINT32)0x7FFFFFFF );
                        RamWrite32A(o_ctrl, ACNV_YY , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, ACNV_YX , (UINT32)0x7FFFFFFF );
                        RamWrite32A(o_ctrl, ACNV_ZP , (UINT32)0x7FFFFFFF );
                    }
                    if(2 == o_ctrl->ois_gyro_id)//18827 rear
                    {
                        CAM_INFO(CAM_OIS, "SetGyroAccelCoef : gyro %d 827 rear\n", o_ctrl->ois_gyro_id);
                        RamWrite32A(o_ctrl, GCNV_XX , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, GCNV_XY , (UINT32)0x7FFFFFFF );
                        RamWrite32A(o_ctrl, GCNV_YY , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, GCNV_YX , (UINT32)0x7FFFFFFF );
                        RamWrite32A(o_ctrl, GCNV_ZP , (UINT32)0x80000001 );

                        RamWrite32A(o_ctrl, ACNV_XX , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, ACNV_XY , (UINT32)0x7FFFFFFF );
                        RamWrite32A(o_ctrl, ACNV_YY , (UINT32)0x00000000 );
                        RamWrite32A(o_ctrl, ACNV_YX , (UINT32)0x7FFFFFFF );
                        RamWrite32A(o_ctrl, ACNV_ZP , (UINT32)0x80000001 );
                    }
                    break;
            default:
                    CAM_INFO(CAM_OIS, "SetGyroAccelCoef : default");
        }

}

//********************************************************************************
// Function Name 	: MesFil
// Retun Value		: NON
// Argment Value	: Measure Filter Mode
// Explanation		: Measure Filter Setting Function
// History			: First edition
//********************************************************************************
void MesFil(struct cam_ois_ctrl_t *o_ctrl, UINT8	UcMesMod )		// 20.019kHz
{
	UINT32	UlMeasFilaA , UlMeasFilaB , UlMeasFilaC ;
	UINT32	UlMeasFilbA , UlMeasFilbB , UlMeasFilbC ;

         UlMeasFilaA	=	0x00000000 ;
	     UlMeasFilaB	=	0x00000000 ;
	     UlMeasFilaC	=	0x00000000 ;
	     UlMeasFilbA	=	0x00000000 ;
	     UlMeasFilbB	=	0x00000000 ;
	     UlMeasFilbC	=	0x00000000 ;


	if( !UcMesMod ) {								// Hall Bias&Offset Adjust

		UlMeasFilaA	=	0x0342AD4D ;	// LPF 150Hz
		UlMeasFilaB	=	0x0342AD4D ;
		UlMeasFilaC	=	0x797AA565 ;
		UlMeasFilbA	=	0x7FFFFFFF ;	// Through
		UlMeasFilbB	=	0x00000000 ;
		UlMeasFilbC	=	0x00000000 ;

	} else if( UcMesMod == LOOPGAIN ) {				// Loop Gain Adjust

		UlMeasFilaA	=	0x12FEA055 ;	// LPF1000Hz
		UlMeasFilaB	=	0x12FEA055 ;
		UlMeasFilaC	=	0x5A02BF55 ;
		UlMeasFilbA	=	0x7F559791 ;	// HPF30Hz
		UlMeasFilbB	=	0x80AA686F ;
		UlMeasFilbC	=	0x7EAB2F23 ;

	} else if( UcMesMod == THROUGH ) {				// for Through

		UlMeasFilaA	=	0x7FFFFFFF ;	// Through
		UlMeasFilaB	=	0x00000000 ;
		UlMeasFilaC	=	0x00000000 ;
		UlMeasFilbA	=	0x7FFFFFFF ;	// Through
		UlMeasFilbB	=	0x00000000 ;
		UlMeasFilbC	=	0x00000000 ;

	} else if( UcMesMod == NOISE ) {				// SINE WAVE TEST for NOISE

		UlMeasFilaA	=	0x0342AD4D ;	// LPF150Hz
		UlMeasFilaB	=	0x0342AD4D ;
		UlMeasFilaC	=	0x797AA565 ;
		UlMeasFilbA	=	0x0342AD4D ;	// LPF150Hz
		UlMeasFilbB	=	0x0342AD4D ;
		UlMeasFilbC	=	0x797AA565 ;

	} else if(UcMesMod == OSCCHK) {
		UlMeasFilaA	=	0x065BE349 ;	// LPF300Hz
		UlMeasFilaB	=	0x065BE349 ;
		UlMeasFilaC	=	0x7348396D ;
		UlMeasFilbA	=	0x065BE349 ;	// LPF300Hz
		UlMeasFilbB	=	0x065BE349 ;
		UlMeasFilbC	=	0x7348396D ;
	}

	RamWrite32A (o_ctrl, MeasureFilterA_Coeff_a1	, UlMeasFilaA ) ;
	RamWrite32A (o_ctrl, MeasureFilterA_Coeff_b1	, UlMeasFilaB ) ;
	RamWrite32A (o_ctrl, MeasureFilterA_Coeff_c1	, UlMeasFilaC ) ;

	RamWrite32A (o_ctrl, MeasureFilterA_Coeff_a2	, UlMeasFilbA ) ;
	RamWrite32A (o_ctrl, MeasureFilterA_Coeff_b2	, UlMeasFilbB ) ;
	RamWrite32A (o_ctrl, MeasureFilterA_Coeff_c2	, UlMeasFilbC ) ;

	RamWrite32A (o_ctrl, MeasureFilterB_Coeff_a1	, UlMeasFilaA ) ;
	RamWrite32A (o_ctrl, MeasureFilterB_Coeff_b1	, UlMeasFilaB ) ;
	RamWrite32A (o_ctrl, MeasureFilterB_Coeff_c1	, UlMeasFilaC ) ;

	RamWrite32A (o_ctrl, MeasureFilterB_Coeff_a2	, UlMeasFilbA ) ;
	RamWrite32A (o_ctrl, MeasureFilterB_Coeff_b2	, UlMeasFilbB ) ;
	RamWrite32A (o_ctrl, MeasureFilterB_Coeff_c2	, UlMeasFilbC ) ;
}

//********************************************************************************
// Function Name 	: ClrMesFil
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Clear Measure Filter Function
// History			: First edition
//********************************************************************************
void ClrMesFil(struct cam_ois_ctrl_t *o_ctrl)
{
	RamWrite32A (o_ctrl, MeasureFilterA_Delay_z11	, 0 ) ;
	RamWrite32A (o_ctrl, MeasureFilterA_Delay_z12	, 0 ) ;

	RamWrite32A (o_ctrl, MeasureFilterA_Delay_z21	, 0 ) ;
	RamWrite32A (o_ctrl, MeasureFilterA_Delay_z22	, 0 ) ;

	RamWrite32A (o_ctrl, MeasureFilterB_Delay_z11	, 0 ) ;
	RamWrite32A (o_ctrl, MeasureFilterB_Delay_z12	, 0 ) ;

	RamWrite32A (o_ctrl, MeasureFilterB_Delay_z21	, 0 ) ;
	RamWrite32A (o_ctrl, MeasureFilterB_Delay_z22	, 0 ) ;
}


//********************************************************************************
// Function Name 	: SetWaitTime
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Set Timer wait Function
// History			: First edition
//********************************************************************************
void SetWaitTime(struct cam_ois_ctrl_t *o_ctrl, UINT16 UsWaitTime )
{
	RamWrite32A(o_ctrl, WaitTimerData_UiWaitCounter	, 0 ) ;
	RamWrite32A(o_ctrl, WaitTimerData_UiTargetCount	, (UINT32)(ONE_MSEC_COUNT * UsWaitTime)) ;
}

//********************************************************************************
// Function Name 	: SetTransDataAdr
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Trans Address for Data Function
// History			: First edition
//********************************************************************************
void SetTransDataAdr(struct cam_ois_ctrl_t *o_ctrl, UINT16 UsLowAddress , UINT32 UlLowAdrBeforeTrans )
{
	UnDwdVal	StTrsVal ;

	if( UlLowAdrBeforeTrans < 0x00009000 ){
		StTrsVal.UlDwdVal = UlLowAdrBeforeTrans ;
	}else{
		StTrsVal.StDwdVal.UsHigVal = (UINT16)(( UlLowAdrBeforeTrans & 0x0000F000 ) >> 8 ) ;
		StTrsVal.StDwdVal.UsLowVal = (UINT16)( UlLowAdrBeforeTrans & 0x00000FFF ) ;
	}
//TRACE(" TRANS  ADR = %04xh , DAT = %08xh \n",UsLowAddress , StTrsVal.UlDwdVal ) ;
	RamWrite32A(o_ctrl, UsLowAddress,	StTrsVal.UlDwdVal );

}

//********************************************************************************
// Function Name 	: MemoryClear
// Retun Value		: NON
// Argment Value	: Top pointer , Size
// Explanation		: Memory Clear Function
// History			: First edition
//********************************************************************************
void MemoryClear(struct cam_ois_ctrl_t *o_ctrl, UINT16 UsSourceAddress, UINT16 UsClearSize )
{
	UINT16	UsLoopIndex ;

	for ( UsLoopIndex = 0 ; UsLoopIndex < UsClearSize ; UsLoopIndex += 4 ) {
		RamWrite32A(o_ctrl, UsSourceAddress + UsLoopIndex, 0x00000000 ) ;				// 4Byte
	}
}

//********************************************************************************
// Function Name 	: MeasureStart
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure start setting Function
// History			: First edition
//********************************************************************************
void MeasureStart(struct cam_ois_ctrl_t *o_ctrl, INT32 SlMeasureParameterNum,
    UINT32 SlMeasureParameterA , UINT32 SlMeasureParameterB )
{
	MemoryClear(o_ctrl, StMeasFunc_SiSampleNum , sizeof( MeasureFunction_Type ) ) ;
	RamWrite32A(o_ctrl, StMeasFunc_MFA_SiMax1	 , 0x80000000 ) ;					// Set Min
	RamWrite32A(o_ctrl, StMeasFunc_MFB_SiMax2	 , 0x80000000 ) ;					// Set Min
	RamWrite32A(o_ctrl, StMeasFunc_MFA_SiMin1	 , 0x7FFFFFFF ) ;					// Set Max
	RamWrite32A(o_ctrl, StMeasFunc_MFB_SiMin2	 , 0x7FFFFFFF ) ;					// Set Max

	SetTransDataAdr(o_ctrl, StMeasFunc_MFA_PiMeasureRam1	 , SlMeasureParameterA ) ;		// Set Measure Filter A Ram Address
	SetTransDataAdr(o_ctrl, StMeasFunc_MFB_PiMeasureRam2	 , SlMeasureParameterB ) ;		// Set Measure Filter B Ram Address
	RamWrite32A(o_ctrl, StMeasFunc_SiSampleNum	 , 0 ) ;													// Clear Measure Counter
	ClrMesFil(o_ctrl) ;						// Clear Delay Ram
//	SetWaitTime(50) ;
	SetWaitTime(o_ctrl, 1) ;
	RamWrite32A(o_ctrl, StMeasFunc_SiSampleMax	 , SlMeasureParameterNum ) ;						// Set Measure Max Number

}

//********************************************************************************
// Function Name 	: MeasureWait
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Wait complete of Measure Function
// History			: First edition
//********************************************************************************
void MeasureWait(struct cam_ois_ctrl_t *o_ctrl)
{
	UINT32			SlWaitTimerSt ;

	SlWaitTimerSt = 1 ;
	while( SlWaitTimerSt ){
		RamRead32A(o_ctrl, StMeasFunc_SiSampleMax , &SlWaitTimerSt ) ;
	}
}

//********************************************************************************
// Function Name 	: SetGyroOffset
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: set the gyro offset data. before do this before Remapmain.
// History			: First edition
//********************************************************************************
void SetGyroOffset(struct cam_ois_ctrl_t *o_ctrl, UINT16 GyroOffsetX, UINT16 GyroOffsetY, UINT16 GyroOffsetZ )
{
	RamWrite32A(o_ctrl, GYRO_RAM_GXOFFZ , (( GyroOffsetX << 16 ) & 0xFFFF0000 ) ) ;		// X axis Gyro offset
	RamWrite32A(o_ctrl, GYRO_RAM_GYOFFZ , (( GyroOffsetY << 16 ) & 0xFFFF0000 ) ) ;		// Y axis Gyro offset
	RamWrite32A(o_ctrl, GYRO_RAM_GZOFFZ , (( GyroOffsetZ << 16 ) & 0xFFFF0000 ) ) ;		// Y axis Gyro offset
}

//********************************************************************************
// Function Name 	:GyroOffsetMeasureStart
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: start the gyro offset adjustment
// History			: First edition
//********************************************************************************
#define 	GYROF_NUM		2048			// 2048times
void GyroOffsetMeasureStart(struct cam_ois_ctrl_t *o_ctrl)
{
	MesFil(o_ctrl, THROUGH ) ;								// Set Measure Filter
	MeasureStart(o_ctrl, GYROF_NUM , GYRO_RAM_GX_ADIDAT , GYRO_RAM_GY_ADIDAT ) ;	// Start measure
}

//********************************************************************************
// Function Name 	:GyroOffsetMeasureStartZ
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: start the gyro offset adjustment
// History			: First edition
//********************************************************************************
#define 	GYROF_NUM		2048			// 2048times
void GyroOffsetMeasureStartZ(struct cam_ois_ctrl_t *o_ctrl)
{
	MesFil(o_ctrl, THROUGH ) ;								// Set Measure Filter
	MeasureStart(o_ctrl, GYROF_NUM , GYRO_RAM_GZ_ADIDAT , GYRO_RAM_GZ_ADIDAT ) ;	// Start measure
}

//********************************************************************************
// Function Name 	: GetGyroOffset
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: get the gyro offset adjustment result
// History			: First edition
//********************************************************************************
UINT8 GetGyroOffset(struct cam_ois_ctrl_t *o_ctrl, UINT16* GyroOffsetX, UINT16* GyroOffsetY, INT16 GYROF_UPPER, INT16 GYROF_LOWER   )
{
	UnllnVal		StMeasValueA , StMeasValueB ;
	INT32			SlMeasureAveValueA , SlMeasureAveValueB ;
	INT32			SlMeasureMaxValue , SlMeasureMinValue ;
	UINT32 UlReadVal, UlCnt=0;
	UINT8 ans=0;

	// Wait complete of measurement
	do{
		if( UlCnt++ > 100 ){
			/* timeout error */
			*GyroOffsetX = 0;
			*GyroOffsetY = 0;
			CAM_ERR(CAM_OIS,
				"UlCnt up to 100+ return 3.");//byron
			return( 3 );
		}
		RamRead32A(o_ctrl, StMeasFunc_SiSampleMax , &UlReadVal ) ;
	}while ( UlReadVal != 0 );

	RamRead32A(o_ctrl, StMeasFunc_MFA_SiMax1 , ( UINT32 * )&SlMeasureMaxValue ) ;	// Max value
	RamRead32A(o_ctrl, StMeasFunc_MFA_SiMin1 , ( UINT32 * )&SlMeasureMinValue ) ;	// Min value
	if (SlMeasureMaxValue == SlMeasureMinValue )
	{
		CAM_ERR(CAM_OIS,
				"SlMeasureMaxValue == SlMeasureMinValue return 3.");//byron2
		return( 3 );
	}

	RamRead32A(o_ctrl, StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
	RamRead32A(o_ctrl, StMeasFunc_MFA_LLiIntegral1 + 4		, &StMeasValueA.StUllnVal.UlHigVal ) ;
	RamRead32A(o_ctrl, StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
	RamRead32A(o_ctrl, StMeasFunc_MFB_LLiIntegral2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;

	SlMeasureAveValueA = (INT32)( (INT64)StMeasValueA.UllnValue / GYROF_NUM ) ;
	SlMeasureAveValueB = (INT32)( (INT64)StMeasValueB.UllnValue / GYROF_NUM ) ;

	SlMeasureAveValueA = ( SlMeasureAveValueA >> 16 ) & 0x0000FFFF ;
	SlMeasureAveValueB = ( SlMeasureAveValueB >> 16 ) & 0x0000FFFF ;

	*GyroOffsetX = ( UINT16 )( SlMeasureAveValueA & 0x0000FFFF );		//Measure Result Store
	*GyroOffsetY = ( UINT16 )( SlMeasureAveValueB & 0x0000FFFF );		//Measure Result Store

	if(( (INT16)(*GyroOffsetX) > GYROF_UPPER ) || ( (INT16)(*GyroOffsetX) < GYROF_LOWER )){
		ans |= 1;
	}
	if(( (INT16)(*GyroOffsetY) > GYROF_UPPER ) || ( (INT16)(*GyroOffsetY) < GYROF_LOWER )){
		ans |= 2;
	}
    CAM_ERR(CAM_OIS,
				"func finish return.");//byron3
	return( ans );
}

#ifdef ENABLE_OIS_DELAY_POWER_DOWN
static int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl);

int ois_power_down_thread(void *arg)
{
    int rc = 0;
    int i;
    struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;
    struct cam_ois_soc_private *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
    struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	if (!o_ctrl || !soc_private || !power_info) {
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK, soc_private %pK, power_info %pK", o_ctrl, soc_private, power_info);
		return -EINVAL;
	}

    mutex_lock(&(o_ctrl->ois_power_down_mutex));
    o_ctrl->ois_power_down_thread_state = CAM_OIS_POWER_DOWN_THREAD_RUNNING;
    mutex_unlock(&(o_ctrl->ois_power_down_mutex));

    for (i = 0; i < (OIS_POWER_DOWN_DELAY/50); i++) {
        msleep(50);// sleep 50ms every time, and sleep OIS_POWER_DOWN_DELAY/50 times.

        mutex_lock(&(o_ctrl->ois_power_down_mutex));
        if (o_ctrl->ois_power_down_thread_exit) {
            mutex_unlock(&(o_ctrl->ois_power_down_mutex));
            break;
        }
        mutex_unlock(&(o_ctrl->ois_power_down_mutex));
    }

    mutex_lock(&(o_ctrl->ois_power_down_mutex));
    if ((!o_ctrl->ois_power_down_thread_exit) && (o_ctrl->ois_power_state == CAM_OIS_POWER_ON)) {
		rc = cam_ois_power_down(o_ctrl);
		if (!rc){
			kfree(power_info->power_setting);
			kfree(power_info->power_down_setting);
			power_info->power_setting = NULL;
			power_info->power_down_setting = NULL;
			power_info->power_down_setting_size = 0;
			power_info->power_setting_size = 0;
			CAM_ERR(CAM_OIS, "cam_ois_power_down successfully");
		} else {
			CAM_ERR(CAM_OIS, "cam_ois_power_down failed");
		}
		o_ctrl->ois_power_state = CAM_OIS_POWER_OFF;
    } else {
		CAM_ERR(CAM_OIS, "No need to do power down, ois_power_down_thread_exit %d, ois_power_state %d", o_ctrl->ois_power_down_thread_exit, o_ctrl->ois_power_state);
    }
    o_ctrl->ois_power_down_thread_state = CAM_OIS_POWER_DOWN_THREAD_STOPPED;
    mutex_unlock(&(o_ctrl->ois_power_down_mutex));

    return rc;
}
#endif

int32_t cam_ois_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 1;
	power_info->power_setting =
		(struct cam_sensor_power_setting *)
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VAF;
	power_info->power_setting[0].seq_val = CAM_VAF;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 2;

	power_info->power_down_setting_size = 1;
	power_info->power_down_setting =
		(struct cam_sensor_power_setting *)
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VAF;
	power_info->power_down_setting[0].config_val = 0;

	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}


/**
 * cam_ois_get_dev_handle - get device handle
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_get_dev_handle(struct cam_ois_ctrl_t *o_ctrl,
	void *arg)
{
	struct cam_sensor_acquire_dev    ois_acq_dev;
	struct cam_create_dev_hdl        bridge_params;
	struct cam_control              *cmd = (struct cam_control *)arg;

	if (o_ctrl->bridge_intf.device_hdl != -1) {
		CAM_ERR(CAM_OIS, "Device is already acquired");
		return -EFAULT;
	}
	if (copy_from_user(&ois_acq_dev, u64_to_user_ptr(cmd->handle),
		sizeof(ois_acq_dev)))
		return -EFAULT;

	bridge_params.session_hdl = ois_acq_dev.session_handle;
	bridge_params.ops = &o_ctrl->bridge_intf.ops;
	bridge_params.v4l2_sub_dev_flag = 0;
	bridge_params.media_entity_flag = 0;
	bridge_params.priv = o_ctrl;

	ois_acq_dev.device_handle =
		cam_create_device_hdl(&bridge_params);
	o_ctrl->bridge_intf.device_hdl = ois_acq_dev.device_handle;
	o_ctrl->bridge_intf.session_hdl = ois_acq_dev.session_handle;

	CAM_DBG(CAM_OIS, "Device Handle: %d", ois_acq_dev.device_handle);
	if (copy_to_user(u64_to_user_ptr(cmd->handle), &ois_acq_dev,
		sizeof(struct cam_sensor_acquire_dev))) {
		CAM_ERR(CAM_OIS, "ACQUIRE_DEV: copy to user failed");
		return -EFAULT;
	}
	return 0;
}

static int cam_ois_power_up(struct cam_ois_ctrl_t *o_ctrl)
{
	int                             rc = 0;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t  *power_info;

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_OIS,
			"Using default power settings");
		rc = cam_ois_construct_default_power_setting(power_info);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Construct default ois power setting failed.");
			return rc;
		}
	}

    ctrl_wide = NULL;
    ctrl_tele = NULL;
	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed in ois power up rc %d", rc);
		return rc;
	}

	rc = camera_io_init(&o_ctrl->io_master_info);
	if (rc)
		CAM_ERR(CAM_OIS, "cci_init failed: rc: %d", rc);
	else
		CAM_INFO(CAM_OIS,"camera_io_init");

	return rc;
}

/**
 * cam_ois_power_down - power down OIS device
 * @o_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
static int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t                         rc = 0;
	struct cam_sensor_power_ctrl_t  *power_info;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &o_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_OIS, "failed: power_info %pK", power_info);
		return -EINVAL;
	}

	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "power down the core is failed:%d", rc);
		return rc;
	}

	camera_io_release(&o_ctrl->io_master_info);
	CAM_INFO(CAM_OIS, "cam_io_release");

	if (strstr(o_ctrl->ois_name, "imx586")) {
		imx586_ois_initialized = false;
		imx586_ois_ready = false;
	} else if (strstr(o_ctrl->ois_name, "s5k3m5")) {
		s5k3m5_ois_initialized = false;
		s5k3m5_ois_ready = false;
	}

	return rc;
}

static int cam_ois_apply_settings(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t i, size;

	if (o_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_OIS, " Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_RANDOM) {
			rc = camera_io_dev_write(&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings));
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed in Applying i2c wrt settings");
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
			size = i2c_list->i2c_settings.size;
			for (i = 0; i < size; i++) {
				rc = camera_io_dev_poll(
				&(o_ctrl->io_master_info),
				i2c_list->i2c_settings.reg_setting[i].reg_addr,
				i2c_list->i2c_settings.reg_setting[i].reg_data,
				i2c_list->i2c_settings.reg_setting[i].data_mask,
				i2c_list->i2c_settings.addr_type,
				i2c_list->i2c_settings.data_type,
				i2c_list->i2c_settings.reg_setting[i].delay);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"i2c poll apply setting Fail");
					return rc;
				}
			}
		}
	}

	return rc;
}

static int cam_ois_slaveInfo_pkt_parser(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t *cmd_buf)
{
	int32_t rc = 0;
	struct cam_cmd_ois_info *ois_info;

	if (!o_ctrl || !cmd_buf) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_info = (struct cam_cmd_ois_info *)cmd_buf;
	if (o_ctrl->io_master_info.master_type == CCI_MASTER) {
		o_ctrl->io_master_info.cci_client->i2c_freq_mode =
			ois_info->i2c_freq_mode;
		o_ctrl->io_master_info.cci_client->sid =
			ois_info->slave_addr >> 1;
		o_ctrl->ois_fw_flag = ois_info->ois_fw_flag;
		o_ctrl->is_ois_calib = ois_info->is_ois_calib;
		memcpy(o_ctrl->ois_name, ois_info->ois_name, 32);
		o_ctrl->io_master_info.cci_client->retries = 3;
		o_ctrl->io_master_info.cci_client->id_map = 0;
		memcpy(&(o_ctrl->opcode), &(ois_info->opcode),
			sizeof(struct cam_ois_opcode));
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x Freq Mode: %d",
			ois_info->slave_addr, ois_info->i2c_freq_mode);
	} else if (o_ctrl->io_master_info.master_type == I2C_MASTER) {
		o_ctrl->io_master_info.client->addr = ois_info->slave_addr;
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x", ois_info->slave_addr);
	} else {
		CAM_ERR(CAM_OIS, "Invalid Master type : %d",
			o_ctrl->io_master_info.master_type);
		rc = -EINVAL;
	}

	return rc;
}
static int cam_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
    UINT32 UlReadValX, UlReadValY;
    //UINT16 OffsetX, OffsetY;
    UINT32 spi_type;
    UINT32 UlReadVal;
    unsigned char rc = 0;
    //unsigned char gyroresult = 0;
    struct timespec mStartTime, mEndTime, diff;
    UINT64 mSpendTime = 0;
    enum cci_i2c_master_t entry_cci_master;
    if (!o_ctrl) {
        CAM_ERR(CAM_OIS, "Invalid Args");
        return -EINVAL;
    }
    entry_cci_master = o_ctrl->io_master_info.cci_client->cci_i2c_master;
    getnstimeofday(&mStartTime);

    //Master_1:imx586 Master_0:s5k3m5
    CAM_INFO(CAM_OIS, "sid:0x%02x, Master:%d, ois_gyro_id:%d cci_master_id:%d",
        o_ctrl->io_master_info.cci_client->sid,
        o_ctrl->io_master_info.cci_client->cci_i2c_master,
        o_ctrl->ois_gyro_id,o_ctrl->cci_master_id);
    //identify if hotdogb else fallback
    imx586_cci_master = (enum cci_i2c_master_t)o_ctrl->cci_master_id;
    if (imx586_cci_master == o_ctrl->io_master_info.cci_client->cci_i2c_master &&
        false == imx586_ois_initialized && strstr(o_ctrl->ois_name, "imx586"))
    {
        if (o_ctrl->ois_gyro_id == 3) {
            if (strcmp(o_ctrl->ois_name, "ofilm_imx586_lc898124ep3_ois") == 0) {
                rc = SelectDownload(o_ctrl, 0x02, 0x01, 0x00, o_ctrl->ois_fw_flag);
            } else {
                rc = SelectDownload(o_ctrl, 0x02, 0x00, 0x00, o_ctrl->ois_fw_flag);
            }
        } else {
            if (strcmp(o_ctrl->ois_name, "ofilm_imx586_lc898124ep3_ois") == 0) {
                rc = SelectDownload(o_ctrl, 0x02, 0x06, 0x00, o_ctrl->ois_fw_flag);
            } else {
                rc = SelectDownload(o_ctrl, 0x02, 0x02, 0x00, o_ctrl->ois_fw_flag);
            }
        }

        if (0 == rc) {
            imx586_ois_initialized = true;
            RamRead32A( o_ctrl,(SiVerNum + 4), &UlReadVal );
            SetGyroAccelCoef( o_ctrl,(UINT8)(UlReadVal>>8) );
            CAM_INFO(CAM_OIS, "SetGyroAccelCoef : %02x\n", (UINT8)(UlReadVal>>8));
            //GyroOffsetMeasureStart(o_ctrl);// +
            //msleep(120);
            //gyroresult = GetGyroOffset(o_ctrl, &OffsetX, &OffsetY, 15, 0);
            //CAM_INFO(CAM_OIS, "Gyro_offset_X:0x%x, Gyro_offset_Y:0x%x, Gyroresult:0x%x", OffsetX, OffsetY,gyroresult);
            //GyroOffsetMeasureStartZ(o_ctrl);
            //msleep(120);
            //gyroresult = GetGyroOffset(o_ctrl, &OffsetX, &OffsetY, 15, 0);
            //CAM_INFO(CAM_OIS, "Gyro_offset_Z:0x%x, Gyro_offset_Z:0x%x, Gyroresult:0x%x", OffsetX, OffsetY,gyroresult);// -

            //remap master
            RamWrite32A(o_ctrl, 0xF000, 0x00000000 );
            //msleep(120);

            //SPI-Master ( Act1 )  Check gyro signal
            RamRead32A(o_ctrl, 0x061C, & UlReadValX );
            RamRead32A(o_ctrl, 0x0620, & UlReadValY );
            CAM_INFO(CAM_OIS, "Gyro_X:0x%x, Gyro_Y:0x%x", UlReadValX, UlReadValY);

            spi_type = 0;
            RamRead32A(o_ctrl, 0xf112, & spi_type );
            CAM_INFO(CAM_OIS, "spi_type:0x%x", spi_type);

            //GyroOffsetMeasureStart(o_ctrl);
            //msleep(10);
            //GetGyroOffset(o_ctrl, &OffsetX, &OffsetY, 15, 0);
            //CAM_INFO(CAM_OIS, "Gyro_offset_X:0x%x, Gyro_offset_Y:0x%x", OffsetX, OffsetY);

            //GyroOffsetMeasureStartZ(o_ctrl);
            //msleep(10);
            //GetGyroOffset(o_ctrl, &OffsetX, &OffsetY, 15, 0);
            //CAM_INFO(CAM_OIS, "Gyro_offset_Z:0x%x, Gyro_offset_Z:0x%x", OffsetX, OffsetY);

            //SPI-Master ( Act1 )  Check gyro gain
            RamRead32A(o_ctrl, 0x82b8, & UlReadValX );
            RamRead32A(o_ctrl, 0x8318, & UlReadValY );
            CAM_INFO(CAM_OIS, "Gyro_gain_X:0x%x, Gyro_gain_Y:0x%x", UlReadValX, UlReadValY);

            //SPI-Master ( Act1 )  start gyro signal transfer. ( from Master to slave. )
            RamWrite32A(o_ctrl, 0x8970, 0x00000001 );
            //msleep(5);
            RamWrite32A(o_ctrl, 0xf111, 0x00000001 );
            //msleep(5);
        } else {
            switch (rc) {
                case 0x01:
                    CAM_ERR(CAM_OIS, "H/W error");
                    break;
                case 0x02:
                    CAM_ERR(CAM_OIS, "Table Data & Program download verify error");
                    break;
                case 0xF0:
                    CAM_ERR(CAM_OIS, "Download code select error");
                    break;
                case 0xF1:
                    CAM_ERR(CAM_OIS, "Download code information read error");
                    break;
                case 0xF2:
                    CAM_ERR(CAM_OIS, "Download code information disagreement");
                    break;
                case 0xF3:
                    CAM_ERR(CAM_OIS, "Download code version error");
                    break;
                default:
                    CAM_ERR(CAM_OIS, "Unkown error code");
                    break;
            }
        }
    }
        else if (MASTER_0 == o_ctrl->io_master_info.cci_client->cci_i2c_master &&
        false == s5k3m5_ois_initialized && strstr(o_ctrl->ois_name, "s5k3m5"))
    {
        o_ctrl->io_master_info.cci_client->cci_i2c_master = MASTER_0;
        o_ctrl->io_master_info.cci_client->sid = SLAVE_CCI_ADDR;
        if (strcmp(o_ctrl->ois_name, "ofilm_s5k3m5_lc898124ep3_ois") == 0 ){
            rc = SelectDownload(o_ctrl, 0x02, 0x07, 0x01, o_ctrl->ois_fw_flag);
        }else {
            rc = SelectDownload(o_ctrl, 0x02, 0x03, 0x01, o_ctrl->ois_fw_flag);
        }
        if (0 == rc) {
            RamRead32A( o_ctrl,(SiVerNum + 4), &UlReadVal );
            //SetGyroAccelCoef(o_ctrl, 0x02);
            SetGyroAccelCoef( o_ctrl,(UINT8)(UlReadVal>>8) );
            CAM_INFO(CAM_OIS, "SetGyroAccelCoef : %02x\n", (UINT8)(UlReadVal>>8));
            //remap slave
            RamWrite32A(o_ctrl, 0xF000, 0x00000000 );
            //msleep(120);
            //SPI-Master ( Act1 )  Check gyro signal
            RamRead32A(o_ctrl, 0x061C, & UlReadValX );
            RamRead32A(o_ctrl, 0x0620, & UlReadValY );
            CAM_INFO(CAM_OIS, "Gyro_X:0x%x, Gyro_Y:0x%x", UlReadValX, UlReadValY);

            spi_type = 0;
            RamRead32A(o_ctrl, 0xf112, & spi_type );
            CAM_INFO(CAM_OIS, "spi_type:0x%x", spi_type);

            //SPI-Master ( Act1 )  Check gyro gain
            RamRead32A(o_ctrl, 0x82b8, & UlReadValX );
            RamRead32A(o_ctrl, 0x8318, & UlReadValY );
            CAM_INFO(CAM_OIS, "Gyro_gain_X:0x%x, Gyro_gain_Y:0x%x", UlReadValX, UlReadValY);
            s5k3m5_ois_initialized = true;
        } else {
            switch (rc) {
                case 0x01:
                    CAM_ERR(CAM_OIS, "Slave H/W error");
                    break;
                case 0x02:
                    CAM_ERR(CAM_OIS, "Slave Table Data & Program download verify error");
                    break;
                case 0xF0:
                    CAM_ERR(CAM_OIS, "Slave Download code select error");
                    break;
                case 0xF1:
                    CAM_ERR(CAM_OIS, "Slave Download code information read error");
                    break;
                case 0xF2:
                    CAM_ERR(CAM_OIS, "Slave Download code information disagreement");
                    break;
                case 0xF3:
                    CAM_ERR(CAM_OIS, "Slave Download code version error");
                    break;
                default:
                    CAM_ERR(CAM_OIS, "Slave Unkown error code");
                    break;
            }

        }
        if (false == imx586_ois_initialized) {
            o_ctrl->io_master_info.cci_client->cci_i2c_master = imx586_cci_master;
            o_ctrl->io_master_info.cci_client->sid = MASTER_CCI_ADDR;
            if (strcmp(o_ctrl->ois_name, "ofilm_imx586_lc898124ep3_ois") == 0 ){
                rc = SelectDownload(o_ctrl, 0x02, 0x06, 0x00, o_ctrl->ois_fw_flag);
            } else {
                rc = SelectDownload(o_ctrl, 0x02, 0x02, 0x00, o_ctrl->ois_fw_flag);
            }
            if (0 == rc) {
	            //remap master
	            RamWrite32A(o_ctrl, 0xF000, 0x00000000 );
	            msleep(120);
	            //SPI-Master ( Act1 )  start gyro signal transfer. ( from Master to slave. )
	            RamWrite32A(o_ctrl, 0x8970, 0x00000001 );
	            msleep(5);
	            RamWrite32A(o_ctrl, 0xf111, 0x00000001 );
	            o_ctrl->io_master_info.cci_client->cci_i2c_master = MASTER_0;
	            o_ctrl->io_master_info.cci_client->sid = SLAVE_CCI_ADDR;
	            RamRead32A(o_ctrl, 0x061C, & UlReadValX );
	            RamRead32A(o_ctrl, 0x0620, & UlReadValY );
	            CAM_INFO(CAM_OIS, "Slave Gyro_X:0x%x, Gyro_Y:0x%x", UlReadValX, UlReadValY);
	            spi_type = 0;
	            RamRead32A(o_ctrl, 0xf112, & spi_type );
	            CAM_INFO(CAM_OIS, "spi_type:0x%x", spi_type);
	            //imx586_ois_initialized = true;
	        } else {
	            switch (rc) {
	                case 0x01:
	                    CAM_ERR(CAM_OIS, "H/W error");
	                    break;
	                case 0x02:
	                    CAM_ERR(CAM_OIS, "Table Data & Program download verify error");
	                    break;
	                case 0xF0:
	                    CAM_ERR(CAM_OIS, "Download code select error");
	                    break;
	                case 0xF1:
	                    CAM_ERR(CAM_OIS, "Download code information read error");
	                    break;
	                case 0xF2:
	                    CAM_ERR(CAM_OIS, "Download code information disagreement");
	                    break;
	                case 0xF3:
	                    CAM_ERR(CAM_OIS, "Download code version error");
	                    break;
	                default:
	                    CAM_ERR(CAM_OIS, "Unkown error code");
	                    break;
	            }
	        }
	    }
    }
    getnstimeofday(&mEndTime);
    diff = timespec_sub(mEndTime, mStartTime);
    mSpendTime = (timespec_to_ns(&diff))/1000000;
    o_ctrl->io_master_info.cci_client->cci_i2c_master = entry_cci_master;
    CAM_INFO(CAM_OIS, "cam_ois_fw_download rc=%d, (Spend: %d ms)", rc, mSpendTime);

    return 0;
}

/**
 * cam_ois_pkt_parse - Parse csl packet
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_pkt_parse(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int32_t                         rc = 0;
	int32_t                         i = 0;
	uint32_t                        total_cmd_buf_in_bytes = 0;
	struct common_header           *cmm_hdr = NULL;
	uintptr_t                       generic_ptr;
	struct cam_control             *ioctl_ctrl = NULL;
	struct cam_config_dev_cmd       dev_config;
	struct i2c_settings_array      *i2c_reg_settings = NULL;
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	uintptr_t                       generic_pkt_addr;
	size_t                          pkt_len;
	size_t                          remaining_len_of_buff = 0;
	struct cam_packet              *csl_packet = NULL;
	size_t                          len_of_buff = 0;
	uint32_t                       *offset = NULL, *cmd_buf;
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t  *power_info = &soc_private->power_info;
    uint32_t temp, retry_cnt;

	ioctl_ctrl = (struct cam_control *)arg;
	if (copy_from_user(&dev_config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(dev_config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(dev_config.packet_handle,
		&generic_pkt_addr, &pkt_len);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"error in converting command Handle Error: %d", rc);
		return rc;
	}

	remaining_len_of_buff = pkt_len;
	if ((sizeof(struct cam_packet) > pkt_len) ||
		((size_t)dev_config.offset >= pkt_len -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_OIS,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), pkt_len);
		rc = -EINVAL;
		goto rel_pkt;
	}

	remaining_len_of_buff -= dev_config.offset;
	csl_packet = (struct cam_packet *)
		(generic_pkt_addr + (uint32_t)dev_config.offset);

	if (((size_t)(csl_packet->header.size) > remaining_len_of_buff)) {
		CAM_ERR(CAM_OIS,
			"Inval pkt_header_size: %zu, len:of_buff: %zu",
			csl_packet->header.size, remaining_len_of_buff);
		rc = -EINVAL;
		goto rel_pkt;
	}

	remaining_len_of_buff -= sizeof(struct cam_packet);

	if ((sizeof(struct cam_cmd_buf_desc) > remaining_len_of_buff) ||
		(csl_packet->num_cmd_buf * sizeof(struct cam_cmd_buf_desc) >
			remaining_len_of_buff)) {
		CAM_ERR(CAM_OIS, "InVal len: %zu", remaining_len_of_buff);
		rc = -EINVAL;
		goto rel_pkt;
	}

	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_OIS_PACKET_OPCODE_INIT:
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);

		/* Loop through multiple command buffers */
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			total_cmd_buf_in_bytes = cmd_desc[i].length;
			if (!total_cmd_buf_in_bytes)
				continue;

			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "Failed to get cpu buf : 0x%x",
					cmd_desc[i].mem_handle);
				goto rel_pkt;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_OIS, "invalid cmd buf");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmm_hdr = (struct common_header *)cmd_buf;

			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				rc = cam_ois_slaveInfo_pkt_parser(
					o_ctrl, cmd_buf);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"Failed in parsing slave info");
					goto rel_cmd_buf;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_OIS,
					"Received power settings buffer");
#ifdef ENABLE_OIS_DELAY_POWER_DOWN
				mutex_lock(&(o_ctrl->ois_power_down_mutex));
				if (o_ctrl->ois_power_state == CAM_OIS_POWER_OFF){
					rc = cam_sensor_update_power_settings(
					    cmd_buf,
					    total_cmd_buf_in_bytes,
					    power_info);
					if (!rc){
					    CAM_ERR(CAM_OIS, "cam_sensor_update_power_settings successfully");
					} else {
					    CAM_ERR(CAM_OIS, "cam_sensor_update_power_settings failed");
					    goto rel_cmd_buf;
					}
				} else {
				    CAM_ERR(CAM_OIS, "OIS already power on, no need to update power setting");
				}
				mutex_unlock(&(o_ctrl->ois_power_down_mutex));
#else
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					power_info);
#endif
				if (rc) {
					CAM_ERR(CAM_OIS,
					"Failed: parse power settings");
					goto rel_cmd_buf;
				}
				break;
			default:
			if (o_ctrl->i2c_init_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS,
				"Received init settings");
				i2c_reg_settings =
					&(o_ctrl->i2c_init_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"init parsing failed: %d", rc);
					goto rel_cmd_buf;
				}
			} else if ((o_ctrl->is_ois_calib != 0) &&
				(o_ctrl->i2c_calib_data.is_settings_valid ==
				0)) {
				CAM_DBG(CAM_OIS,
					"Received calib settings");
				i2c_reg_settings = &(o_ctrl->i2c_calib_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"Calib parsing failed: %d", rc);
					goto rel_cmd_buf;
				}
			}
			break;
			}
			if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
				CAM_WARN(CAM_OIS, "Failed to put cpu buf: 0x%x",
					cmd_desc[i].mem_handle);
		}

		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
#ifdef ENABLE_OIS_DELAY_POWER_DOWN
			mutex_lock(&(o_ctrl->ois_power_down_mutex));
			o_ctrl->ois_power_down_thread_exit = true;
			if (o_ctrl->ois_power_state == CAM_OIS_POWER_OFF){
				rc = cam_ois_power_up(o_ctrl);
				if (!rc){
					o_ctrl->ois_power_state = CAM_OIS_POWER_ON;
					CAM_ERR(CAM_OIS, "cam_ois_power_up successfully");
				} else {
					CAM_ERR(CAM_OIS, "cam_ois_power_up failed");
					goto rel_pkt;
				}
			} else {
				CAM_ERR(CAM_OIS, "OIS already power on, no need to power on again");
			}
			mutex_unlock(&(o_ctrl->ois_power_down_mutex));
#else
			rc = cam_ois_power_up(o_ctrl);
#endif
			if (rc) {
				CAM_ERR(CAM_OIS, " OIS Power up failed");
				goto rel_pkt;
			}
			o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		}

		if (o_ctrl->ois_fw_flag) {
			rc = cam_ois_fw_download(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, "Failed OIS FW Download");
				goto pwr_dwn;
			}
		}

		rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply Init settings");
			goto pwr_dwn;
		}

		if (o_ctrl->is_ois_calib) {
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_calib_data);
			if (rc) {
				CAM_ERR(CAM_OIS, "Cannot apply calib data");
				goto pwr_dwn;
			}
		}

		rc = delete_request(&o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Init data: rc: %d", rc);
			rc = 0;
		}
		rc = delete_request(&o_ctrl->i2c_calib_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Calibration data: rc: %d", rc);
			rc = 0;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_OIS_CONTROL:
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to control OIS: %d",
				o_ctrl->cam_ois_state);
			goto rel_pkt;
		}
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_mode_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			goto rel_pkt;
		}
        imx586_cci_master = (enum cci_i2c_master_t)o_ctrl->cci_master_id;
        if (imx586_cci_master == o_ctrl->io_master_info.cci_client->cci_i2c_master && !imx586_ois_ready) {
            retry_cnt = 10;
            do {
                RamRead32A(o_ctrl, 0xF100, &temp);
                CAM_ERR(CAM_OIS, "read imx586 0xF100 = 0x%x", temp);
                if (temp == 0)
                {
                    imx586_ois_ready = true;
                    break;
                }
                retry_cnt--;
                msleep(10);
            } while(retry_cnt);
        }
        if (MASTER_0 == o_ctrl->io_master_info.cci_client->cci_i2c_master && !s5k3m5_ois_ready) {
            retry_cnt = 10;
            do {
                RamRead32A(o_ctrl, 0xF100, &temp);
                CAM_ERR(CAM_OIS, "read s5k3m5 0xF100 = 0x%x", temp);
                if (temp == 0)
                {
                    s5k3m5_ois_ready = true;
                    break;
                }
                retry_cnt--;
                msleep(10);
            } while(retry_cnt);
        }

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply mode settings");
			goto rel_pkt;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Fail deleting Mode data: rc: %d", rc);
			goto rel_pkt;
		}
		break;
	default:
		CAM_ERR(CAM_OIS, "Invalid Opcode: %d",
			(csl_packet->header.op_code & 0xFFFFFF));
		rc = -EINVAL;
		goto rel_pkt;
	}

	if (!rc)
		goto rel_pkt;

rel_cmd_buf:
	if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
		CAM_WARN(CAM_OIS, "Failed to put cpu buf: 0x%x",
			cmd_desc[i].mem_handle);
pwr_dwn:
	cam_ois_power_down(o_ctrl);
rel_pkt:
	if (cam_mem_put_cpu_buf(dev_config.packet_handle))
		CAM_WARN(CAM_OIS, "Fail in put buffer: 0x%x",
			dev_config.packet_handle);

	return rc;
}

void cam_ois_shutdown(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
#ifndef ENABLE_OIS_DELAY_POWER_DOWN
	struct cam_ois_soc_private *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;
#endif

	if (o_ctrl->cam_ois_state == CAM_OIS_INIT)
		return;

	if (o_ctrl->cam_ois_state >= CAM_OIS_CONFIG) {
#ifdef ENABLE_OIS_DELAY_POWER_DOWN
		mutex_lock(&(o_ctrl->ois_power_down_mutex));
		if (o_ctrl->ois_power_state == CAM_OIS_POWER_ON && o_ctrl->ois_power_down_thread_state == CAM_OIS_POWER_DOWN_THREAD_STOPPED) {
			o_ctrl->ois_power_down_thread_exit = false;
			kthread_run(ois_power_down_thread, o_ctrl, "ois_power_down_thread");
			CAM_ERR(CAM_OIS, "ois_power_down_thread created");
		} else {
			CAM_ERR(CAM_OIS, "no need to create ois_power_down_thread, ois_power_state %d, ois_power_down_thread_state %d", o_ctrl->ois_power_state, o_ctrl->ois_power_down_thread_state);
		}
		mutex_unlock(&(o_ctrl->ois_power_down_mutex));
#else
		rc = cam_ois_power_down(o_ctrl);
#endif
		if (rc < 0)
			CAM_ERR(CAM_OIS, "OIS Power down failed");
		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
	}

	if (o_ctrl->cam_ois_state >= CAM_OIS_ACQUIRE) {
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
	}

	if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_mode_data);

	if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_calib_data);

	if (o_ctrl->i2c_init_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_init_data);

#ifndef ENABLE_OIS_DELAY_POWER_DOWN
	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;
#endif

	o_ctrl->cam_ois_state = CAM_OIS_INIT;
}

/**
 * cam_ois_driver_cmd - Handle ois cmds
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
int cam_ois_driver_cmd(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int                              rc = 0;
	struct cam_ois_query_cap_t       ois_cap = {0};
	struct cam_control              *cmd = (struct cam_control *)arg;
#ifndef ENABLE_OIS_DELAY_POWER_DOWN
	struct cam_ois_soc_private      *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;
#endif

	uint32_t testAddr  = 61458;
	uint32_t testData  = 0;

	if (!o_ctrl || !cmd) {
		CAM_ERR(CAM_OIS, "Invalid arguments");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_OIS, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

    if(MASTER_1 == o_ctrl->io_master_info.cci_client->cci_i2c_master && ctrl_wide == NULL)
    {
        ctrl_wide = o_ctrl; //record wide device info
    }
    else if(MASTER_0 == o_ctrl->io_master_info.cci_client->cci_i2c_master && ctrl_tele == NULL)
    {
        ctrl_tele = o_ctrl; //record tele device info
    }
#ifndef ENABLE_OIS_DELAY_POWER_DOWN
	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
#endif

	mutex_lock(&(o_ctrl->ois_mutex));
	switch (cmd->op_code) {
	case CAM_QUERY_CAP:
		ois_cap.slot_info = o_ctrl->soc_info.index;

		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&ois_cap,
			sizeof(struct cam_ois_query_cap_t))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		CAM_DBG(CAM_OIS, "ois_cap: ID: %d", ois_cap.slot_info);
		break;
	case CAM_ACQUIRE_DEV:
		rc = cam_ois_get_dev_handle(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed to acquire dev");
			goto release_mutex;
		}

		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
		break;
	case CAM_START_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for start : %d",
			o_ctrl->cam_ois_state);
			goto release_mutex;
		}
		o_ctrl->cam_ois_state = CAM_OIS_START;
		break;
	case CAM_CONFIG_DEV:
		CAM_INFO(CAM_OIS, "CAM_CONFIG_DEV for %s", o_ctrl->ois_name);
		rc = cam_ois_pkt_parse(o_ctrl, arg);

		camera_io_dev_read((&o_ctrl->io_master_info),
		testAddr, &testData, 2, 4);
		CAM_INFO(CAM_OIS, "OISAddr 0x%x, value read %d for %s",
		testAddr, testData, o_ctrl->ois_name);

		if (rc) {
			CAM_ERR(CAM_OIS, "Failed in ois pkt Parsing");
			goto release_mutex;
		}
		break;
	case CAM_RELEASE_DEV:
		if (o_ctrl->cam_ois_state == CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Cant release ois: in start state");
			goto release_mutex;
		}

		if (o_ctrl->cam_ois_state == CAM_OIS_CONFIG) {
#ifdef ENABLE_OIS_DELAY_POWER_DOWN
			mutex_lock(&(o_ctrl->ois_power_down_mutex));
			if (o_ctrl->ois_power_state == CAM_OIS_POWER_ON && o_ctrl->ois_power_down_thread_state == CAM_OIS_POWER_DOWN_THREAD_STOPPED) {
				o_ctrl->ois_power_down_thread_exit = false;
				kthread_run(ois_power_down_thread, o_ctrl, "ois_power_down_thread");
				CAM_ERR(CAM_OIS, "ois_power_down_thread created");
			} else {
				CAM_ERR(CAM_OIS, "no need to create ois_power_down_thread, ois_power_state %d, ois_power_down_thread_state %d", o_ctrl->ois_power_state, o_ctrl->ois_power_down_thread_state);
			}
			mutex_unlock(&(o_ctrl->ois_power_down_mutex));
#else
			rc = cam_ois_power_down(o_ctrl);
#endif
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "OIS Power down failed");
				goto release_mutex;
			}
		}

		if (o_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_OIS, "link hdl: %d device hdl: %d",
				o_ctrl->bridge_intf.device_hdl,
				o_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
		o_ctrl->cam_ois_state = CAM_OIS_INIT;

#ifndef ENABLE_OIS_DELAY_POWER_DOWN
		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_down_setting_size = 0;
		power_info->power_setting_size = 0;
#endif

		if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_mode_data);

		if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_calib_data);

		if (o_ctrl->i2c_init_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_init_data);

		break;
	case CAM_STOP_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for stop : %d",
			o_ctrl->cam_ois_state);
		}
		o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		break;
	default:
		CAM_ERR(CAM_OIS, "invalid opcode");
		goto release_mutex;
	}
release_mutex:
	mutex_unlock(&(o_ctrl->ois_mutex));
	return rc;
}
