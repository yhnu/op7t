//********************************************************************************
//
//		<< LC898124EP3 Evaluation Soft>>
//		Program Name	: OisLC898124EP1.h
// 		Explanation		: LC898124 Global Declaration & ProtType Declaration
//		Design			: K.abe
//		History			: First edition
//********************************************************************************
/************************************************/
/*	Command										*/
/************************************************/

//#define	ACT_SEMCO			0x01	// SEMCO
//#define	ACT_SEMCO			0x00	// SEMCO
#define		ACT_SO2820			0x02	// SEMCO SO2820(Wide)
#define		ACT_SO3600			0x03	// SEMCO SO3600(Tele)
#define		ACT_M12337_A1		0x06	// JAHWA M12337(Wide)
#define		ACT_M10235_A1		0x07	// JAHWA M10235(Tele)

#define		CMD_IO_ADR_ACCESS				0xC000				// IO Write Access
#define		CMD_IO_DAT_ACCESS				0xD000				// IO Read Access
#define		CMD_REMAP						0xF001				// Remap
#define		CMD_REBOOT						0xF003				// Reboot
#define		CMD_ROMVER						0xF00F				// Rom code version read
#define		OLD_VER							0x20161121
#define		CUR_VER							0x20170410
#define		CMD_RETURN_TO_CENTER			0xF010				// Center Servo ON/OFF choose axis
#define		BOTH_SRV_OFF					0x00000000			// Both   Servo OFF
#define		XAXS_SRV_ON						0x00000001			// X axis Servo ON
#define		YAXS_SRV_ON						0x00000002			// Y axis Servo ON
#define		BOTH_SRV_ON						0x00000003			// Both   Servo ON
#define		ZAXS_SRV_OFF					0x00000004			// Z axis Servo OFF
#define		ZAXS_SRV_ON						0x00000005			// Z axis Servo ON
#define		CMD_PAN_TILT					0xF011				// Pan Tilt Enable/Disable
#define		PAN_TILT_OFF					0x00000000			// Pan/Tilt OFF
#define		PAN_TILT_ON						0x00000001			// Pan/Tilt ON
#define		CMD_OIS_ENABLE					0xF012				// Ois Enable/Disable
#define		OIS_DISABLE						0x00000000			// OIS Disable
#define		OIS_ENABLE						0x00000001			// OIS Enable
#define		OIS_ENA_NCL						0x00000002			// OIS Enable ( none Delay clear )
#define		OIS_ENA_DOF						0x00000004			// OIS Enable ( Drift offset exec )
#define		CMD_MOVE_STILL_MODE				0xF013				// Select mode
#define		MOVIE_MODE						0x00000000			// Movie mode
#define		STILL_MODE						0x00000001			// Still mode
#define		MOVIE_MODE1						0x00000002			// Movie Preview mode 1
#define		STILL_MODE1						0x00000003			// Still Preview mode 1
#define		MOVIE_MODE2						0x00000004			// Movie Preview mode 2
#define		STILL_MODE2						0x00000005			// Still Preview mode 2
#define		MOVIE_MODE3						0x00000006			// Movie Preview mode 3
#define		STILL_MODE3						0x00000007			// Still Preview mode 3
#define		CMD_CHASE_CONFIRMATION			0xF015				// Hall Chase confirmation
#define		CMD_GYRO_SIG_CONFIRMATION		0xF016				// Gyro Signal confirmation
#define		CMD_FLASH_LOAD					0xF017				// Flash Load
#define		HALL_CALB_FLG					0x00008000
#define		HALL_CALB_BIT					0x00FF00FF
#define		GYRO_GAIN_FLG					0x00004000
#define		ANGL_CORR_FLG					0x00002000
#define		FOCL_GAIN_FLG					0x00001000
#define		ZSRV_CAL_FLG					0x00000800			// ZeroServo calibration data
#define		HLLN_CALB_FLG					0x00000400			// Hall linear calibration
#define		MIXI_CALB_FLG					0x00000200			// Mixing calibration
#define		CROS_TALK_FLG					0x00000200			//!< Cross talk calibration
#define		CMD_LASER_LINEAR_DATA			0xF019
#define		CMD_AF_POSITION					0xF01A				// AF Position
#define		CMD_SSC_ENABLE					0xF01C				//!< Select mode
#define		SSC_DISABLE						0x00000000			//!< Ssc Disable
#define		SSC_ENABLE						0x00000001			//!< Ssc Enable
#define		CMD_GYRO_RD_ACCS				0xF01D				// Gyro Read Acess
#define		CMD_GYRO_WR_ACCS				0xF01E				// Gyro Write Acess

#define		CMD_READ_STATUS					0xF100				// Status Read
#define		READ_STATUS_INI					0x01000000

#define		CMD_ZSRV_MODE					0xF01F
#define		ZSRV_DISABLE					0x00000000
#define		ZSRV_ENABLE						0x00000001
#define		ZSRV_HOLD						0x00000003

#define		OIS_POS_FLG			0x00000100			// OIS position by AF measurement

//==============================================================================
//E2PROM
//==============================================================================
#define		EEPROM_ONSEMI_LDO					0x00
#define		EEPROM_ONSEMI_CP1					0x01
#define		EEPROM_ONSEMI_CP2					0x02
#define		EEPROM_ONSEMI_CP3					0x03
#define		EEPROM_ONSEMI_OSCS					0x04
#define		EEPROM_ONSEMI_DRVGAINAF				0x05
#define		EEPROM_ONSEMI_DRVOFSTAF				0x06
#define		EEPROM_ONSEMI_DRVOFSTAFM			0x07
#define		EEPROM_ONSEMI_DRVGAINX				0x08
#define		EEPROM_ONSEMI_DRVOFSTX				0x09
#define		EEPROM_ONSEMI_DRVOFSTXM				0x0A
#define		EEPROM_ONSEMI_DRVGAINY				0x0B
#define		EEPROM_ONSEMI_DRVOFSTY				0x0C
#define		EEPROM_ONSEMI_DRVOFSTYM				0x0D
#define		EEPROM_ONSEMI_MARK					0x0E
#define		EEPROM_ONSEMI_CHECKSUM				0x0F		/* target area 0x00~0x0E */

#define		EEPROM_ONSEMI_IDSEL					0x10
#define		EEPROM_DrvZdir						0x11
#define		EEPROM_AfUcoef_LSB					0x12
#define		EEPROM_AfUcoef_MSB					0x13
#define		EEPROM_AfDcoef_LSB					0x14
#define		EEPROM_AfDcoef_MSB					0x15
#define		EEPROM_AfFrq_LSB					0x16
#define		EEPROM_AfFrq_MSB					0x17

#define		EEPROM_Calibration_Status_LSB		0x18
#define		EEPROM_Calibration_Status_MSB		0x19
#define		EEPROM_Calibration_HallMaxX_LSB		0x1A
#define		EEPROM_Calibration_HallMaxX_MSB		0x1B
#define		EEPROM_Calibration_HallMinX_LSB		0x1C
#define		EEPROM_Calibration_HallMinX_MSB		0x1D
#define		EEPROM_Calibration_HallMaxY_LSB		0x1E
#define		EEPROM_Calibration_HallMaxY_MSB		0x1F

#define		EEPROM_Calibration_HallMinY_LSB		0x20
#define		EEPROM_Calibration_HallMinY_MSB		0x21
#define		EEPROM_Calibration_HallBiasX		0x22
#define		EEPROM_Calibration_HallOffsetX		0x23
#define		EEPROM_Calibration_HallBiasY		0x24
#define		EEPROM_Calibration_HallOffsetY		0x25
#define		EEPROM_Calibration_LoopGainX_LSB	0x26
#define		EEPROM_Calibration_LoopGainX_MSB	0x27

#define		EEPROM_Calibration_LoopGainY_LSB	0x28
#define		EEPROM_Calibration_LoopGainY_MSB	0x29
#define		EEPROM_Calibration_LensOffsetX_LSB	0x2A
#define		EEPROM_Calibration_LensOffsetX_MSB	0x2B
#define		EEPROM_Calibration_LensOffsetY_LSB	0x2C
#define		EEPROM_Calibration_LensOffsetY_MSB	0x2D

#define		EEPROM_Calibration_GyroGainX_0Byte	0x2E
#define		EEPROM_Calibration_GyroGainX_1Byte	0x2F
#define		EEPROM_Calibration_GyroGainX_2Byte	0x30
#define		EEPROM_Calibration_GyroGainX_3Byte	0x31
#define		EEPROM_Calibration_GyroGainY_0Byte	0x32
#define		EEPROM_Calibration_GyroGainY_1Byte	0x33
#define		EEPROM_Calibration_GyroGainY_2Byte	0x34
#define		EEPROM_Calibration_GyroGainY_3Byte	0x35

#if 0
#define		EEPROM_Calibration_ZSX_offset_0Byte		0x36
#define		EEPROM_Calibration_ZSX_offset_1Byte		0x37
#define		EEPROM_Calibration_ZSX_shift_0Byte		0x38
#define		EEPROM_Calibration_ZSX_shift_1Byte		0x39
#define		EEPROM_Calibration_ZSX_gcore_0Byte		0x3A
#define		EEPROM_Calibration_ZSX_gcore_1Byte		0x3B
#define		EEPROM_Calibration_ZSX_gaina_0Byte		0x3C
#define		EEPROM_Calibration_ZSX_gaina_1Byte		0x3D

#define		EEPROM_Calibration_ZSY_offset_0Byte		0x3E
#define		EEPROM_Calibration_ZSY_offset_1Byte		0x3F
#define		EEPROM_Calibration_ZSY_shift_0Byte		0x40
#define		EEPROM_Calibration_ZSY_shift_1Byte		0x41
#define		EEPROM_Calibration_ZSY_gcore_0Byte		0x42
#define		EEPROM_Calibration_ZSY_gcore_1Byte		0x43
#define		EEPROM_Calibration_ZSY_gaina_0Byte		0x44
#define		EEPROM_Calibration_ZSY_gaina_1Byte		0x45

#define		EEPROM_Calibration_ZSZ_offset_0Byte		0x46
#define		EEPROM_Calibration_ZSZ_offset_1Byte		0x47
#endif

#define		EEPROM_POSITION_1X_LSB				0x48
#define		EEPROM_POSITION_1X_MSB				0x49
#define		EEPROM_POSITION_1Y_LSB				0x4A
#define		EEPROM_POSITION_1Y_MSB				0x4B
#define		EEPROM_POSITION_2X_LSB				0x4C
#define		EEPROM_POSITION_2X_MSB				0x4D
#define		EEPROM_POSITION_2Y_LSB				0x4E
#define		EEPROM_POSITION_2Y_MSB				0x4F

#define		EEPROM_POSITION_3X_LSB				0x50
#define		EEPROM_POSITION_3X_MSB				0x51
#define		EEPROM_POSITION_3Y_LSB				0x52
#define		EEPROM_POSITION_3Y_MSB				0x53
#define		EEPROM_POSITION_4X_LSB				0x54
#define		EEPROM_POSITION_4X_MSB				0x55
#define		EEPROM_POSITION_4Y_LSB				0x56
#define		EEPROM_POSITION_4Y_MSB				0x57

#define		EEPROM_POSITION_5X_LSB				0x58
#define		EEPROM_POSITION_5X_MSB				0x59
#define		EEPROM_POSITION_5Y_LSB				0x5A
#define		EEPROM_POSITION_5Y_MSB				0x5B
#define		EEPROM_POSITION_6X_LSB				0x5C
#define		EEPROM_POSITION_6X_MSB				0x5D
#define		EEPROM_POSITION_6Y_LSB				0x5E
#define		EEPROM_POSITION_6Y_MSB				0x5F

#define		EEPROM_POSITION_7X_LSB				0x60
#define		EEPROM_POSITION_7X_MSB				0x61
#define		EEPROM_POSITION_7Y_LSB				0x62
#define		EEPROM_POSITION_7Y_MSB				0x63
#define		EEPROM_STEPX_0Byte					0x64
#define		EEPROM_STEPX_1Byte					0x65
#define		EEPROM_STEPY_0Byte					0x66
#define		EEPROM_STEPY_1Byte					0x67

#define		EEPROM_CROSSTALK_XX_LSB				0x68
#define		EEPROM_CROSSTALK_XX_MSB				0x69
#define		EEPROM_CROSSTALK_XY_LSB				0x6A
#define		EEPROM_CROSSTALK_XY_MSB				0x6B
#define		EEPROM_CROSSTALK_YY_LSB				0x6C
#define		EEPROM_CROSSTALK_YY_MSB				0x6D
#define		EEPROM_CROSSTALK_YX_LSB				0x6E
#define		EEPROM_CROSSTALK_YX_MSB				0x6F

#define		EEPROM_CROSSTALK_XSHIFT				0x70
#define		EEPROM_CROSSTALK_YSHIFT				0x71

#define		EEPROM_Optical_LensOffsetX_LSB		0x72
#define		EEPROM_Optical_LensOffsetX_MSB		0x73
#define		EEPROM_Optical_LensOffsetY_LSB		0x74
#define		EEPROM_Optical_LensOffsetY_MSB		0x75

////////////////////////////////////////////////////

//#define		EEPROM_Calibration_Reserve_78		0x78
//#define		EEPROM_Calibration_Reserve_79		0x79
//#define		EEPROM_Calibration_Reserve_7A		0x7A
//#define		EEPROM_Calibration_Reserve_7B		0x7B
//#define		EEPROM_Calibration_Reserve_7C		0x7C

#define		EEPROM_CheckSum						0x7D	/* target area 0x10~0x7c */
#define		EEPROM_CheckCode1					0x7E
#define		EEPROM_CheckCode2					0x7F

//==============================================================================
//DMA
//==============================================================================

#define		DmCheck_CheckSumDMA				0x0100
#define		DmCheck_CheckSumDMB				0x0104

#define 	PmCheck							0x0108
#define 		PmCheck_PmemAddr				(0x0000 + PmCheck)						// 0x0108
#define 		PmCheck_Size					(0x0004 + PmCheck_PmemAddr)				// 0x010C
#define 		PmCheck_CheckSum				(0x0004 + PmCheck_Size)					// 0x0110
#define 		PmCheck_EndFlag					(0x0004 + PmCheck_CheckSum)				// 0x0114

#define		HallFilterD_HXDAZ1				0x0130	//0x0080
#define		HallFilterD_HYDAZ1				0x0180	//0x00D0

#define		HALL_RAM_X_COMMON				0x01C0	//0x0110
#define			HALL_RAM_HXOFF					(0x0000 + HALL_RAM_X_COMMON)		// 0x01C0
#define			HALL_RAM_HXOFF1					(0x0004 + HALL_RAM_X_COMMON)		// 0x01C4
#define			HALL_RAM_HXOUT0					(0x0008 + HALL_RAM_X_COMMON)		// 0x01C8
#define			HALL_RAM_HXOUT1					(0x000C + HALL_RAM_X_COMMON)		// 0x01CC
#define			HALL_RAM_HXOUT2					(0x0010 + HALL_RAM_X_COMMON)		// 0x01D0
#define			HALL_RAM_SINDX0					(0x0014 + HALL_RAM_X_COMMON)		// 0x01D4
#define			HALL_RAM_HXLOP					(0x0018 + HALL_RAM_X_COMMON)		// 0x01D8
#define			HALL_RAM_SINDX1					(0x001C + HALL_RAM_X_COMMON)		// 0x01DC
#define			HALL_RAM_HALL_X_OUT				(0x0020 + HALL_RAM_X_COMMON)		// 0x01E0
#define		HALL_RAM_HALL_SwitchX			0x0210	//0x015c

#define		HALL_RAM_Y_COMMON				0x0214	//0x0160
#define			HALL_RAM_HYOFF					(0x0000 + HALL_RAM_Y_COMMON)		// 0x0214
#define			HALL_RAM_HYOFF1					(0x0004 + HALL_RAM_Y_COMMON)		// 0x0218
#define			HALL_RAM_HYOUT0					(0x0008 + HALL_RAM_Y_COMMON)		// 0x021C
#define			HALL_RAM_HYOUT1					(0x000C + HALL_RAM_Y_COMMON)		// 0x0220
#define			HALL_RAM_HYOUT2					(0x0010 + HALL_RAM_Y_COMMON)		// 0x0224
#define			HALL_RAM_SINDY0					(0x0014 + HALL_RAM_Y_COMMON)		// 0x0228
#define			HALL_RAM_HYLOP					(0x0018 + HALL_RAM_Y_COMMON)		// 0x022C
#define			HALL_RAM_SINDY1					(0x001C + HALL_RAM_Y_COMMON)		// 0x0230
#define			HALL_RAM_HALL_Y_OUT				(0x0020 + HALL_RAM_Y_COMMON)		// 0x0234
#define		HALL_RAM_HALL_SwitchY			0x0264	//0x01AC


#define		HALL_RAM_COMMON					0x0268	//0x01B0
//  HallFilterDelay.h HALL_RAM_COMMON_t
#define			HALL_RAM_HXIDAT					(0x0000 + HALL_RAM_COMMON)			// 0x0268
#define			HALL_RAM_HYIDAT					(0x0004 + HALL_RAM_COMMON)			// 0x026C
#define			HALL_RAM_GYROX_OUT				(0x0008 + HALL_RAM_COMMON)			// 0x0270
#define			HALL_RAM_GYROY_OUT				(0x000C + HALL_RAM_COMMON)			// 0x0274

#define		GyroFilterDelayX_delay2			0x278
#define		GyroFilterDelayX_delay3			0x288	//0x1D0
#define			GyroFilterDelayX_GXH1Z1			(0x0000 + GyroFilterDelayX_delay3)	// 0x0288
#define			GyroFilterDelayX_GXH1Z2			(0x0004 + GyroFilterDelayX_delay3)	// 0x028C
#define			GyroFilterDelayX_GXK1Z1			(0x0008 + GyroFilterDelayX_delay3)	// 0x0290
#define			GyroFilterDelayX_GXK1Z2			(0x000C + GyroFilterDelayX_delay3)	// 0x0294
#define		GyroFilterDelayX_delay4			0x298

#define		GyroFilterDelayY_delay2			0x2A0
#define		GyroFilterDelayY_delay3			0x2B0	//0x1F8
#define			GyroFilterDelayY_GYH1Z1			(0x0000 + GyroFilterDelayY_delay3)	// 0x02B0
#define			GyroFilterDelayY_GYH1Z2			(0x0004 + GyroFilterDelayY_delay3)	// 0x02B4
#define			GyroFilterDelayY_GYK1Z1			(0x0008 + GyroFilterDelayY_delay3)	// 0x02B8
#define			GyroFilterDelayY_GYK1Z2			(0x000C + GyroFilterDelayY_delay3)	// 0x02BC
#define		GyroFilterDelayY_delay4			0x2C0

#define		GYRO_RAM_X						0x02C8	//0x0210
// GyroFilterDelay.h GYRO_RAM_t
#define			GYRO_RAM_GYROX_OFFSET			(0x0000 + GYRO_RAM_X)				// 0x02C8
#define			GYRO_RAM_GX2X4XF_IN				(0x0004 + GYRO_RAM_GYROX_OFFSET)	// 0x02CC
#define			GYRO_RAM_GX2X4XF_OUT			(0x0004 + GYRO_RAM_GX2X4XF_IN)		// 0x02D0
#define			GYRO_RAM_GXFAST					(0x0004 + GYRO_RAM_GX2X4XF_OUT)		// 0x02D4
#define			GYRO_RAM_GXSLOW					(0x0004 + GYRO_RAM_GXFAST)			// 0x02D8
#define			GYRO_RAM_GYROX_G1OUT			(0x0004 + GYRO_RAM_GXSLOW)			// 0x02DC
#define			GYRO_RAM_GYROX_G2OUT			(0x0004 + GYRO_RAM_GYROX_G1OUT)		// 0x02E0
#define			GYRO_RAM_GYROX_G3OUT			(0x0004 + GYRO_RAM_GYROX_G2OUT)		// 0x02E4
#define			GYRO_RAM_GYROX_OUT				(0x0004 + GYRO_RAM_GYROX_G3OUT)		// 0x02E8
#define		GYRO_RAM_Y						0x02EC	//0x0234
// GyroFilterDelay.h GYRO_RAM_t
#define			GYRO_RAM_GYROY_OFFSET			(0x0000 + GYRO_RAM_Y)				// 0x02EC
#define			GYRO_RAM_GY2X4XF_IN				(0x0004 + GYRO_RAM_GYROY_OFFSET)	// 0x02F0
#define			GYRO_RAM_GY2X4XF_OUT			(0x0004 + GYRO_RAM_GY2X4XF_IN)		// 0x02F4
#define			GYRO_RAM_GYFAST					(0x0004 + GYRO_RAM_GY2X4XF_OUT)		// 0x02F8
#define			GYRO_RAM_GYSLOW					(0x0004 + GYRO_RAM_GYFAST)			// 0x02FC
#define			GYRO_RAM_GYROY_G1OUT			(0x0004 + GYRO_RAM_GYSLOW)			// 0x0300
#define			GYRO_RAM_GYROY_G2OUT			(0x0004 + GYRO_RAM_GYROY_G1OUT)		// 0x0304
#define			GYRO_RAM_GYROY_G3OUT			(0x0004 + GYRO_RAM_GYROY_G2OUT)		// 0x0308
#define			GYRO_RAM_GYROY_OUT				(0x0004 + GYRO_RAM_GYROY_G3OUT)		// 0x030C

#define		GYRO_RAM_COMMON					0x0310	//0x0258
// GyroFilterDelay.h GYRO_RAM_COMMON_t
#define			GYRO_RAM_GX_ADIDAT				(0x0000 + GYRO_RAM_COMMON)			// 0x0310
#define			GYRO_RAM_GY_ADIDAT				(0x0004 + GYRO_RAM_GX_ADIDAT)		// 0x0314
#define			GYRO_RAM_GZ_ADIDAT				(0x0004 + GYRO_RAM_GY_ADIDAT)		// 0x0318
#define			GYRO_RAM_SINDX					(0x0004 + GYRO_RAM_GZ_ADIDAT)		// 0x031C
#define			GYRO_RAM_SINDY					(0x0004 + GYRO_RAM_SINDX)			// 0x0320
#define			GYRO_RAM_GXLENSZ				(0x0004 + GYRO_RAM_SINDY)			// 0x0324
#define			GYRO_RAM_GYLENSZ				(0x0004 + GYRO_RAM_GXLENSZ)			// 0x0328
#define			GYRO_RAM_GZLENSZ				(0x0004 + GYRO_RAM_GYLENSZ)			// 0x032C
#define			GYRO_RAM_GXOX_OUT				(0x0004 + GYRO_RAM_GZLENSZ)			// 0x0330
#define			GYRO_RAM_GYOX_OUT				(0x0004 + GYRO_RAM_GXOX_OUT)		// 0x0334
#define			GYRO_RAM_GXOFFZ					(0x0004 + GYRO_RAM_GYOX_OUT)		// 0x0338
#define			GYRO_RAM_GYOFFZ					(0x0004 + GYRO_RAM_GXOFFZ)			// 0x033C
#define			GYRO_RAM_GZOFFZ					(0x0004 + GYRO_RAM_GYOFFZ)			// 0x0340
#define			GYRO_RAM_LIMITX					(0x0004 + GYRO_RAM_GZOFFZ)			// 0x0344
#define			GYRO_RAM_LIMITY					(0x0004 + GYRO_RAM_LIMITX)			// 0x0348
#define			GYRO_RAM_LIMITZ					(0x0004 + GYRO_RAM_LIMITY)			// 0x034C
#define			GXFILIN							(0x0004 + GYRO_RAM_LIMITZ)			// 0x0350
#define			GYFILIN							(0x0004 + GXFILIN)					// 0x0354
#define			GZFILIN							(0x0004 + GYFILIN)					// 0x0358
#define			GYRO_RAM_GYRO_Switch			(0x0004 + GZFILIN)					// 0x035C
#define			GYRO_RAM_GYRO_AF_Switch			(0x0001 + GYRO_RAM_GYRO_Switch)		// 0x035D

//#ifdef	ZERO_SERVO
#define		ZeroServoRAM_X					0x03A8
#define 		ZeroServoRAM_X_OFFSET			(0x0000 + ZeroServoRAM_X)			// 0x03A8
#define 		ZeroServoRAM_X_IN				(0x0004 + ZeroServoRAM_X_OFFSET)	// 0x03AC
#define 		ZeroServoRAM_X_LPFIN			(0x0004 + ZeroServoRAM_X_IN	)		// 0x03B0
#define 		ZeroServoRAM_X_LPFOUT			(0x0004 + ZeroServoRAM_X_LPFIN)		// 0x03B4
#define 		ZeroServoRAM_X_ANGOUT			(0x0004 + ZeroServoRAM_X_LPFOUT)	// 0x03B8
#define 		ZeroServoRAM_X_OUT				(0x0004 + ZeroServoRAM_X_ANGOUT)	// 0x03BC
#define		ZeroServoRAM_Y					0x03C0
#define 		ZeroServoRAM_Y_OFFSET			(0x0000 + ZeroServoRAM_Y)			// 0x03C0
#define 		ZeroServoRAM_Y_IN				(0x0004 + ZeroServoRAM_Y_OFFSET)	// 0x03C4
#define 		ZeroServoRAM_Y_LPFIN			(0x0004 + ZeroServoRAM_Y_IN	)		// 0x03C8
#define 		ZeroServoRAM_Y_LPFOUT			(0x0004 + ZeroServoRAM_Y_LPFIN)		// 0x03CC
#define 		ZeroServoRAM_Y_ANGOUT			(0x0004 + ZeroServoRAM_Y_LPFOUT)	// 0x03D0
#define 		ZeroServoRAM_Y_OUT				(0x0004 + ZeroServoRAM_Y_ANGOUT)	// 0x03D4
#define		ZeroServoRAM_Z					0x03D8
#define 		ZeroServoRAM_Z_OFFSET			(0x0000 + ZeroServoRAM_Z)			// 0x03D8
#define 		ZeroServoRAM_Z_IN				(0x0004 + ZeroServoRAM_Z_OFFSET)	// 0x03DC
#define 		ZeroServoRAM_Z_LPFIN			(0x0004 + ZeroServoRAM_Z_IN	)		// 0x03E0
#define 		ZeroServoRAM_Z_LPFOUT			(0x0004 + ZeroServoRAM_Z_LPFIN)		// 0x03E4
#define 		ZeroServoRAM_Z_ANGOUT			(0x0004 + ZeroServoRAM_Z_LPFOUT)	// 0x03E8
#define 		ZeroServoRAM_Z_OUT				(0x0004 + ZeroServoRAM_Z_ANGOUT)	// 0x03EC
//#endif	//ZERO_SERVO

#define		StMeasureFunc					0x0400	//0x02B0
#define			StMeasFunc_SiSampleNum			(0x0000 + StMeasureFunc		)			// 0x0400
#define			StMeasFunc_SiSampleMax			(0x0004 + StMeasFunc_SiSampleNum)		// 0x0404

#define		StMeasureFunc_MFA				0x0408	//0x02B8
#define			StMeasFunc_MFA_SiMax1			(0x0000 + StMeasureFunc_MFA		)		// 0x0408
#define			StMeasFunc_MFA_SiMin1			(0x0004 + StMeasFunc_MFA_SiMax1	)		// 0x040C
#define			StMeasFunc_MFA_UiAmp1			(0x0004 + StMeasFunc_MFA_SiMin1	)		// 0x0410
#define			StMeasFunc_MFA_UiDUMMY1			(0x0004 + StMeasFunc_MFA_UiAmp1	)		// 0x0414
#define			StMeasFunc_MFA_LLiIntegral1		(0x0004 + StMeasFunc_MFA_UiDUMMY1)		// 0x0418	// 8Byte
#define			StMeasFunc_MFA_LLiAbsInteg1		(0x0008 + StMeasFunc_MFA_LLiIntegral1)	// 0x0420	// 8Byte
#define			StMeasFunc_MFA_PiMeasureRam1	(0x0008 + StMeasFunc_MFA_LLiAbsInteg1)	// 0x0428
#define			StMeasFunc_MFA_UiDUMMY2			(0x0004 + StMeasFunc_MFA_PiMeasureRam1)	// 0x042C

#define		StMeasureFunc_MFB				0x0430	//0x02E0
#define			StMeasFunc_MFB_SiMax2			(0x0000 + StMeasureFunc_MFB			)	// 0x0430
#define			StMeasFunc_MFB_SiMin2			(0x0004 + StMeasFunc_MFB_SiMax2		)	// 0x0434
#define			StMeasFunc_MFB_UiAmp2			(0x0004 + StMeasFunc_MFB_SiMin2		)	// 0x0438
#define			StMeasFunc_MFB_UiDUMMY1			(0x0004 + StMeasFunc_MFB_UiAmp2		)	// 0x043C
#define			StMeasFunc_MFB_LLiIntegral2		(0x0004 + StMeasFunc_MFB_UiDUMMY1	)	// 0x0440	// 8Byte
#define			StMeasFunc_MFB_LLiAbsInteg2		(0x0008 + StMeasFunc_MFB_LLiIntegral2)	// 0x0448	// 8Byte
#define			StMeasFunc_MFB_PiMeasureRam2	(0x0008 + StMeasFunc_MFB_LLiAbsInteg2)	// 0x0450

#define		MeasureFilterA_Delay			0x0458	//0x0308
// MeasureFilter.h	MeasureFilter_Delay_Type
#define			MeasureFilterA_Delay_z11		(0x0000 + MeasureFilterA_Delay)			// 0x0458
#define			MeasureFilterA_Delay_z12		(0x0004 + MeasureFilterA_Delay_z11)		// 0x045C
#define			MeasureFilterA_Delay_z21		(0x0004 + MeasureFilterA_Delay_z12)		// 0x0460
#define			MeasureFilterA_Delay_z22		(0x0004 + MeasureFilterA_Delay_z21)		// 0x0464

#define		MeasureFilterB_Delay			0x0468	//0x0318
// MeasureFilter.h	MeasureFilter_Delay_Type
#define			MeasureFilterB_Delay_z11		(0x0000 + MeasureFilterB_Delay)			// 0x0468
#define			MeasureFilterB_Delay_z12		(0x0004 + MeasureFilterB_Delay_z11)		// 0x046C
#define			MeasureFilterB_Delay_z21		(0x0004 + MeasureFilterB_Delay_z12)		// 0x0470
#define			MeasureFilterB_Delay_z22		(0x0004 + MeasureFilterB_Delay_z21)		// 0x0474

#define		SinWaveC						0x0478	//0x0328
#define			SinWaveC_Pt						(0x0000 + SinWaveC)						// 0x0478
#define			SinWaveC_Regsiter				(0x0004 + SinWaveC_Pt)					// 0x047C
#define			SinWaveC_SignFlag				(0x0004 + SinWaveC_Regsiter)			// 0x0480

#define		SinWave							0x0484	//0x0334
// SinGenerator.h SinWave_t
#define			SinWave_Offset					(0x0000 + SinWave)						// 0x0484
#define			SinWave_Phase					(0x0004 + SinWave_Offset)				// 0x0488
#define			SinWave_Gain					(0x0004 + SinWave_Phase)				// 0x048C
#define			SinWave_Output					(0x0004 + SinWave_Gain)					// 0x0490
#define			SinWave_OutAddr					(0x0004 + SinWave_Output)				// 0x0494
#define		CosWave							0x0498	//0x0348
// SinGenerator.h SinWave_t
#define			CosWave_Offset					(0x0000 + CosWave)						// 0x0498
#define			CosWave_Phase					(0x0004 + CosWave_Offset)				// 0x049C
#define			CosWave_Gain					(0x0004 + CosWave_Phase)				// 0x04A0
#define			CosWave_Output					(0x0004 + CosWave_Gain)					// 0x04A4
#define			CosWave_OutAddr					(0x0004 + CosWave_Output)				// 0x04A8

#define		WaitTimerData					0x04AC	//0x035C
// CommonLibrary.h  WaitTimer_Type
#define			WaitTimerData_UiWaitCounter		(0x0000 + WaitTimerData	)				// 0x04AC
#define			WaitTimerData_UiTargetCount		(0x0004 + WaitTimerData_UiWaitCounter)	// 0x04B0
#define			WaitTimerData_UiBaseCount		(0x0004 + WaitTimerData_UiTargetCount)	// 0x04B4

#define		PanTilt_DMA_ScTpdSts			0x04CC	//0x037C



#define		StPosition						0x054C
// StPosition[8]
#define			StPosition_0					(0x0000 + StPosition )					// 0x054C
#define			StPosition_1					(0x0004 + StPosition )					// 0x0550
#define			StPosition_2					(0x0008 + StPosition )					// 0x0554
#define			StPosition_3					(0x000C + StPosition )					// 0x0558
#define			StPosition_4					(0x0010 + StPosition )					// 0x055C
#define			StPosition_5					(0x0014 + StPosition )					// 0x0560
#define			StPosition_6					(0x0018 + StPosition )					// 0x0564
#define			SiStepXY						(0x001C + StPosition )					// 0x0568

#define		Optical_Offset					0x056C
#define			Optical_Offset_X				(0x0000 + Optical_Offset )				// 0x056C
#define			Optical_Offset_Y				(0x0004 + Optical_Offset )				// 0x0570

#define		GYRO_RAM_Z						0x0618	//0x0618
#define			GYRO_RAM_GYROZ_OFFSET			(0x0000 + GYRO_RAM_Z)					// 0x0618

#define		GyroTemp						0x063C

#define		AcclFilDly_X					0x0640
#define		AcclFilDly_Y					0x0670
#define		AcclFilDly_Z					0x06A0

#define		AcclRAM_X						0x06D0
#define			ACCLRAM_X_AC_ADIDAT			0x0000 + AcclRAM_X
#define			ACCLRAM_X_AC_OFFSET			0x0004 + AcclRAM_X

#define		AcclRAM_Y						0x06FC
#define			ACCLRAM_Y_AC_ADIDAT			0x0000 + AcclRAM_Y
#define			ACCLRAM_Y_AC_OFFSET			0x0004 + AcclRAM_Y

#define		AcclRAM_Z						0x0728
#define			ACCLRAM_Z_AC_ADIDAT			0x0000 + AcclRAM_Z
#define			ACCLRAM_Z_AC_OFFSET			0x0004 + AcclRAM_Z


#define		FRA_DMA							(0x278)
#define			FRA_DMA_Control				(0x04 + FRA_DMA	)
//#define			FRA_DMA_DeciCount			(0x0C + FRA_DMA	)
#define			FRA_DMA_DeciShift			(0x10 + FRA_DMA	)
#define			FRA_DMA_InputData			(0x18 + FRA_DMA	)
#define			FRA_DMA_OutputData			(0x1C + FRA_DMA	)

#define			FRA_DMA_Gain				(0x70 + FRA_DMA	)
#define			FRA_DMA_Phase				(0x74 + FRA_DMA	)

#define		HALL_FRA_X_COMMON				0x0574
#define			HALL_FRA_XSININ					0x0000 + HALL_FRA_X_COMMON
#define			HALL_FRA_XHOUTB					0x0004 + HALL_FRA_XSININ
#define			HALL_FRA_XHOUTA					0x0004 + HALL_FRA_XHOUTB

#define		HALL_FRA_Y_COMMON				0x0580
#define			HALL_FRA_YSININ					0x0000 + HALL_FRA_Y_COMMON
#define			HALL_FRA_YHOUTB					0x0004 + HALL_FRA_YSININ
#define			HALL_FRA_YHOUTA					0x0004 + HALL_FRA_YHOUTB



//==============================================================================
//DMB
//==============================================================================
#define		SiVerNum						0x8000
#define		SPI_MST				0x00
#define		SPI_SLV				0x01
#define		SPI_SNGL			0x02

#define		ACT_SO2821			0x00	// SEMCO SO2821
#define		ACT_M12337			0x01	// JAHWA M12337(Wide Standalone)

#define		ACT_45DEG			0xff	// dummy

#define		GYRO_ICM20690		0x00
#define		GYRO_LSM6DSM		0x02

#define		StCalibrationData				0x8010
// Calibration.h  CalibrationData_Type
#define			StCaliData_UsCalibrationStatus	(0x0000 + StCalibrationData)
#define			StCaliData_SiHallMax_Before_X	(0x0004 + StCaliData_UsCalibrationStatus)
#define			StCaliData_SiHallMin_Before_X	(0x0004 + StCaliData_SiHallMax_Before_X)
#define			StCaliData_SiHallMax_After_X	(0x0004 + StCaliData_SiHallMin_Before_X)
#define			StCaliData_SiHallMin_After_X	(0x0004 + StCaliData_SiHallMax_After_X)
#define			StCaliData_SiHallMax_Before_Y	(0x0004 + StCaliData_SiHallMin_After_X)
#define			StCaliData_SiHallMin_Before_Y	(0x0004 + StCaliData_SiHallMax_Before_Y)
#define			StCaliData_SiHallMax_After_Y	(0x0004 + StCaliData_SiHallMin_Before_Y)
#define			StCaliData_SiHallMin_After_Y	(0x0004 + StCaliData_SiHallMax_After_Y)
#define			StCaliData_UiHallBias_X			(0x0004 + StCaliData_SiHallMin_After_Y)
#define			StCaliData_UiHallOffset_X		(0x0004 + StCaliData_UiHallBias_X)
#define			StCaliData_UiHallBias_Y			(0x0004 + StCaliData_UiHallOffset_X)
#define			StCaliData_UiHallOffset_Y		(0x0004 + StCaliData_UiHallBias_Y)
#define			StCaliData_SiLoopGain_X			(0x0004 + StCaliData_UiHallOffset_Y)
#define			StCaliData_SiLoopGain_Y			(0x0004 + StCaliData_SiLoopGain_X)
#define			StCaliData_SiLensCen_Offset_X	(0x0004 + StCaliData_SiLoopGain_Y)
#define			StCaliData_SiLensCen_Offset_Y	(0x0004 + StCaliData_SiLensCen_Offset_X)
#define			StCaliData_SiOtpCen_Offset_X	(0x0004 + StCaliData_SiLensCen_Offset_Y)
#define			StCaliData_SiOtpCen_Offset_Y	(0x0004 + StCaliData_SiOtpCen_Offset_X)
#define			StCaliData_SiGyroOffset_X		(0x0004 + StCaliData_SiOtpCen_Offset_Y)
#define			StCaliData_SiGyroOffset_Y		(0x0004 + StCaliData_SiGyroOffset_X)
#define			StCaliData_SiGyroGain_X			(0x0004 + StCaliData_SiGyroOffset_Y)
#define			StCaliData_SiGyroGain_Y			(0x0004 + StCaliData_SiGyroGain_X)
#define			StCaliData_UiHallBias_AF		(0x0004 + StCaliData_SiGyroGain_Y)
#define			StCaliData_UiHallOffset_AF		(0x0004 + StCaliData_UiHallBias_AF)
#define			StCaliData_SiLoopGain_AF		(0x0004 + StCaliData_UiHallOffset_AF)
#define			StCaliData_SiAD_Offset_AF		(0x0004 + StCaliData_SiLoopGain_AF)
#define			StCaliData_SiMagnification_AF	(0x0004 + StCaliData_SiAD_Offset_AF)
#define			StCaliData_SiHallMax_Before_AF	(0x0004 + StCaliData_SiMagnification_AF)
#define			StCaliData_SiHallMin_Before_AF	(0x0004 + StCaliData_SiHallMax_Before_AF)
#define			StCaliData_SiHallMax_After_AF	(0x0004 + StCaliData_SiHallMin_Before_AF)
#define			StCaliData_SiHallMin_After_AF	(0x0004 + StCaliData_SiHallMax_After_AF)

#define		HallFilterCoeffX				0x8090
// HallFilterCoeff.h  DM_HFC_t
#define			HallFilterCoeffX_HXIGAIN		(0x0000 + HallFilterCoeffX)
#define			HallFilterCoeffX_GYROXOUTGAIN	(0x0004 + HallFilterCoeffX_HXIGAIN)
#define			HallFilterCoeffX_HXOFFGAIN		(0x0004 + HallFilterCoeffX_GYROXOUTGAIN)

#define			HallFilterCoeffX_hxiab			(0x0004 + HallFilterCoeffX_HXOFFGAIN)
#define			HallFilterCoeffX_hxiac			(0x0004 + HallFilterCoeffX_hxiab)
#define			HallFilterCoeffX_hxiaa			(0x0004 + HallFilterCoeffX_hxiac)
#define			HallFilterCoeffX_hxibb			(0x0004 + HallFilterCoeffX_hxiaa)
#define			HallFilterCoeffX_hxibc			(0x0004 + HallFilterCoeffX_hxibb)
#define			HallFilterCoeffX_hxiba			(0x0004 + HallFilterCoeffX_hxibc)
#define			HallFilterCoeffX_hxdab			(0x0004 + HallFilterCoeffX_hxiba)
#define			HallFilterCoeffX_hxdac			(0x0004 + HallFilterCoeffX_hxdab)
#define			HallFilterCoeffX_hxdaa			(0x0004 + HallFilterCoeffX_hxdac)
#define			HallFilterCoeffX_hxdbb			(0x0004 + HallFilterCoeffX_hxdaa)
#define			HallFilterCoeffX_hxdbc			(0x0004 + HallFilterCoeffX_hxdbb)
#define			HallFilterCoeffX_hxdba			(0x0004 + HallFilterCoeffX_hxdbc)
#define			HallFilterCoeffX_hxdcc			(0x0004 + HallFilterCoeffX_hxdba)
#define			HallFilterCoeffX_hxdcb			(0x0004 + HallFilterCoeffX_hxdcc)
#define			HallFilterCoeffX_hxdca			(0x0004 + HallFilterCoeffX_hxdcb)
#define			HallFilterCoeffX_hxpgain0		(0x0004 + HallFilterCoeffX_hxdca)
#define			HallFilterCoeffX_hxigain0		(0x0004 + HallFilterCoeffX_hxpgain0)
#define			HallFilterCoeffX_hxdgain0		(0x0004 + HallFilterCoeffX_hxigain0)
#define			HallFilterCoeffX_hxpgain1		(0x0004 + HallFilterCoeffX_hxdgain0)
#define			HallFilterCoeffX_hxigain1		(0x0004 + HallFilterCoeffX_hxpgain1)
#define			HallFilterCoeffX_hxdgain1		(0x0004 + HallFilterCoeffX_hxigain1)
#define			HallFilterCoeffX_hxgain0		(0x0004 + HallFilterCoeffX_hxdgain1)
#define			HallFilterCoeffX_hxgain1		(0x0004 + HallFilterCoeffX_hxgain0)

#define			HallFilterCoeffX_hxsb			(0x0004 + HallFilterCoeffX_hxgain1)
#define			HallFilterCoeffX_hxsc			(0x0004 + HallFilterCoeffX_hxsb)
#define			HallFilterCoeffX_hxsa			(0x0004 + HallFilterCoeffX_hxsc)

#define			HallFilterCoeffX_hxob			(0x0004 + HallFilterCoeffX_hxsa)
#define			HallFilterCoeffX_hxoc			(0x0004 + HallFilterCoeffX_hxob)
#define			HallFilterCoeffX_hxod			(0x0004 + HallFilterCoeffX_hxoc)
#define			HallFilterCoeffX_hxoe			(0x0004 + HallFilterCoeffX_hxod)
#define			HallFilterCoeffX_hxoa			(0x0004 + HallFilterCoeffX_hxoe)
#define			HallFilterCoeffX_hxpb			(0x0004 + HallFilterCoeffX_hxoa)
#define			HallFilterCoeffX_hxpc			(0x0004 + HallFilterCoeffX_hxpb)
#define			HallFilterCoeffX_hxpd			(0x0004 + HallFilterCoeffX_hxpc)
#define			HallFilterCoeffX_hxpe			(0x0004 + HallFilterCoeffX_hxpd)
#define			HallFilterCoeffX_hxpa			(0x0004 + HallFilterCoeffX_hxpe)

#define		HallFilterCoeffY				0x812c
// HallFilterCoeff.h  DM_HFC_t
#define			HallFilterCoeffY_HYIGAIN		(0x0000 + HallFilterCoeffY)
#define			HallFilterCoeffY_GYROYOUTGAIN	(0x0004 + HallFilterCoeffY_HYIGAIN)
#define			HallFilterCoeffY_HYOFFGAIN		(0x0004 + HallFilterCoeffY_GYROYOUTGAIN)

#define			HallFilterCoeffY_hyiab			(0x0004 + HallFilterCoeffY_HYOFFGAIN)
#define			HallFilterCoeffY_hyiac			(0x0004 + HallFilterCoeffY_hyiab)
#define			HallFilterCoeffY_hyiaa			(0x0004 + HallFilterCoeffY_hyiac)
#define			HallFilterCoeffY_hyibb			(0x0004 + HallFilterCoeffY_hyiaa)
#define			HallFilterCoeffY_hyibc			(0x0004 + HallFilterCoeffY_hyibb)
#define			HallFilterCoeffY_hyiba			(0x0004 + HallFilterCoeffY_hyibc)
#define			HallFilterCoeffY_hydab			(0x0004 + HallFilterCoeffY_hyiba)
#define			HallFilterCoeffY_hydac			(0x0004 + HallFilterCoeffY_hydab)
#define			HallFilterCoeffY_hydaa			(0x0004 + HallFilterCoeffY_hydac)
#define			HallFilterCoeffY_hydbb			(0x0004 + HallFilterCoeffY_hydaa)
#define			HallFilterCoeffY_hydbc			(0x0004 + HallFilterCoeffY_hydbb)
#define			HallFilterCoeffY_hydba			(0x0004 + HallFilterCoeffY_hydbc)
#define			HallFilterCoeffY_hydcc			(0x0004 + HallFilterCoeffY_hydba)
#define			HallFilterCoeffY_hydcb			(0x0004 + HallFilterCoeffY_hydcc)
#define			HallFilterCoeffY_hydca			(0x0004 + HallFilterCoeffY_hydcb)
#define			HallFilterCoeffY_hypgain0		(0x0004 + HallFilterCoeffY_hydca)
#define			HallFilterCoeffY_hyigain0		(0x0004 + HallFilterCoeffY_hypgain0)
#define			HallFilterCoeffY_hydgain0		(0x0004 + HallFilterCoeffY_hyigain0)
#define			HallFilterCoeffY_hypgain1		(0x0004 + HallFilterCoeffY_hydgain0)
#define			HallFilterCoeffY_hyigain1		(0x0004 + HallFilterCoeffY_hypgain1)
#define			HallFilterCoeffY_hydgain1		(0x0004 + HallFilterCoeffY_hyigain1)
#define			HallFilterCoeffY_hygain0		(0x0004 + HallFilterCoeffY_hydgain1)
#define			HallFilterCoeffY_hygain1		(0x0004 + HallFilterCoeffY_hygain0)
#define			HallFilterCoeffY_hysb			(0x0004 + HallFilterCoeffY_hygain1)
#define			HallFilterCoeffY_hysc			(0x0004 + HallFilterCoeffY_hysb)
#define			HallFilterCoeffY_hysa			(0x0004 + HallFilterCoeffY_hysc)
#define			HallFilterCoeffY_hyob			(0x0004 + HallFilterCoeffY_hysa)
#define			HallFilterCoeffY_hyoc			(0x0004 + HallFilterCoeffY_hyob)
#define			HallFilterCoeffY_hyod			(0x0004 + HallFilterCoeffY_hyoc)
#define			HallFilterCoeffY_hyoe			(0x0004 + HallFilterCoeffY_hyod)
#define			HallFilterCoeffY_hyoa			(0x0004 + HallFilterCoeffY_hyoe)
#define			HallFilterCoeffY_hypb			(0x0004 + HallFilterCoeffY_hyoa)
#define			HallFilterCoeffY_hypc			(0x0004 + HallFilterCoeffY_hypb)
#define			HallFilterCoeffY_hypd			(0x0004 + HallFilterCoeffY_hypc)
#define			HallFilterCoeffY_hype			(0x0004 + HallFilterCoeffY_hypd)
#define			HallFilterCoeffY_hypa			(0x0004 + HallFilterCoeffY_hype)

#define		HallFilterLimitX				0x81c8
#define		HallFilterLimitY				0x81e0
#define		HallFilterShiftX				0x81f8
#define		HallFilterShiftY				0x81fe

#define		HF_MIXING						0x8214
#define			HF_hx45x						(0x0000 + HF_MIXING	)		//0x008005E4 : HallMixingCoeff.hx45x
#define			HF_hx45y						(0x0004 + HF_MIXING	)		//0x008005E8 : HallMixingCoeff.hx45y
#define			HF_hy45y						(0x0008 + HF_MIXING	)		//0x008005EC : HallMixingCoeff.hy45y
#define			HF_hy45x						(0x000C + HF_MIXING	)		//0x008005F0 : HallMixingCoeff.hy45x
#define			HF_ShiftX						(0x0010 + HF_MIXING )

#define		HAL_LN_CORRECT					0x8228
#define			HAL_LN_COEFAX					(0x0000 + HAL_LN_CORRECT)	// HallLinearCorrAX.zone_coef[6]
#define			HAL_LN_COEFBX					(0x000C + HAL_LN_COEFAX	)	// HallLinearCorrBX.zone_coef[6]
#define			HAL_LN_ZONEX					(0x000C + HAL_LN_COEFBX	)	// HallLinearZoneX.zone_area[5]
#define			HAL_LN_COEFAY					(0x000A + HAL_LN_ZONEX	)	// HallLinearCorrAY.zone_coef[6]
#define			HAL_LN_COEFBY					(0x000C + HAL_LN_COEFAY	)	// HallLinearCorrBY.zone_coef[6]
#define			HAL_LN_ZONEY					(0x000C + HAL_LN_COEFBY	)	// HallLinearZoneY.zone_area[5]

#define		GyroFilterTableX				0x8270
// GyroFilterCoeff.h  DM_GFC_t
#define			GyroFilterTableX_gx45x			(0x0000 + GyroFilterTableX)
#define			GyroFilterTableX_gx45y			(0x0004 + GyroFilterTableX_gx45x)
#define			GyroFilterTableX_gxgyro			(0x0004 + GyroFilterTableX_gx45y)
#define			GyroFilterTableX_gxsengen		(0x0004 + GyroFilterTableX_gxgyro)
#define			GyroFilterTableX_gxl1b			(0x0004 + GyroFilterTableX_gxsengen)
#define			GyroFilterTableX_gxl1c			(0x0004 + GyroFilterTableX_gxl1b)
#define			GyroFilterTableX_gxl1a			(0x0004 + GyroFilterTableX_gxl1c)
#define			GyroFilterTableX_gxl2b			(0x0004 + GyroFilterTableX_gxl1a)
#define			GyroFilterTableX_gxl2c			(0x0004 + GyroFilterTableX_gxl2b)
#define			GyroFilterTableX_gxl2a			(0x0004 + GyroFilterTableX_gxl2c)
#define			GyroFilterTableX_gxigain		(0x0004 + GyroFilterTableX_gxl2a)
#define			GyroFilterTableX_gxh1b			(0x0004 + GyroFilterTableX_gxigain)
#define			GyroFilterTableX_gxh1c			(0x0004 + GyroFilterTableX_gxh1b)
#define			GyroFilterTableX_gxh1a			(0x0004 + GyroFilterTableX_gxh1c)
#define			GyroFilterTableX_gxk1b			(0x0004 + GyroFilterTableX_gxh1a)
#define			GyroFilterTableX_gxk1c			(0x0004 + GyroFilterTableX_gxk1b)
#define			GyroFilterTableX_gxk1a			(0x0004 + GyroFilterTableX_gxk1c)
#define			GyroFilterTableX_gxgain			(0x0004 + GyroFilterTableX_gxk1a)
#define			GyroFilterTableX_gxzoom			(0x0004 + GyroFilterTableX_gxgain)
#define			GyroFilterTableX_gxlenz			(0x0004 + GyroFilterTableX_gxzoom)
#define			GyroFilterTableX_gxt2b			(0x0004 + GyroFilterTableX_gxlenz)
#define			GyroFilterTableX_gxt2c			(0x0004 + GyroFilterTableX_gxt2b)
#define			GyroFilterTableX_gxt2a			(0x0004 + GyroFilterTableX_gxt2c)
#define			GyroFilterTableX_afzoom			(0x0004 + GyroFilterTableX_gxt2a)

#define		GyroFilterTableY				0x82D0
// GyroFilterCoeff.h  DM_GFC_t
#define			GyroFilterTableY_gy45y			(0x0000 + GyroFilterTableY)
#define			GyroFilterTableY_gy45x			(0x0004 + GyroFilterTableY_gy45y)
#define			GyroFilterTableY_gygyro			(0x0004 + GyroFilterTableY_gy45x)
#define			GyroFilterTableY_gysengen		(0x0004 + GyroFilterTableY_gygyro)
#define			GyroFilterTableY_gyl1b			(0x0004 + GyroFilterTableY_gysengen)
#define			GyroFilterTableY_gyl1c			(0x0004 + GyroFilterTableY_gyl1b)
#define			GyroFilterTableY_gyl1a			(0x0004 + GyroFilterTableY_gyl1c)
#define			GyroFilterTableY_gyl2b			(0x0004 + GyroFilterTableY_gyl1a)
#define			GyroFilterTableY_gyl2c			(0x0004 + GyroFilterTableY_gyl2b)
#define			GyroFilterTableY_gyl2a			(0x0004 + GyroFilterTableY_gyl2c)
#define			GyroFilterTableY_gyigain		(0x0004 + GyroFilterTableY_gyl2a)
#define			GyroFilterTableY_gyh1b			(0x0004 + GyroFilterTableY_gyigain)
#define			GyroFilterTableY_gyh1c			(0x0004 + GyroFilterTableY_gyh1b)
#define			GyroFilterTableY_gyh1a			(0x0004 + GyroFilterTableY_gyh1c)
#define			GyroFilterTableY_gyk1b			(0x0004 + GyroFilterTableY_gyh1a)
#define			GyroFilterTableY_gyk1c			(0x0004 + GyroFilterTableY_gyk1b)
#define			GyroFilterTableY_gyk1a			(0x0004 + GyroFilterTableY_gyk1c)
#define			GyroFilterTableY_gygain			(0x0004 + GyroFilterTableY_gyk1a)
#define			GyroFilterTableY_gyzoom			(0x0004 + GyroFilterTableY_gygain)
#define			GyroFilterTableY_gylenz			(0x0004 + GyroFilterTableY_gyzoom)
#define			GyroFilterTableY_gyt2b			(0x0004 + GyroFilterTableY_gylenz)
#define			GyroFilterTableY_gyt2c			(0x0004 + GyroFilterTableY_gyt2b)
#define			GyroFilterTableY_gyt2a			(0x0004 + GyroFilterTableY_gyt2c)
#define			GyroFilterTableY_afzoom			(0x0004 + GyroFilterTableY_gyt2a)

#define			Gyro_Limiter_X				0x8330
#define			Gyro_Limiter_Y				0x8334
#define			Gyro_ShiftX_RG				0x8338
#define			Gyro_ShiftY_RG				0x833C

#define		ZeroServoFilterTableX			0x8384
#define			ZeroServoFilterTableX_coeff1_0	(0x0000 + ZeroServoFilterTableX)			// 0x8384
#define			ZeroServoFilterTableX_coeff1_1	(0x0004 + ZeroServoFilterTableX_coeff1_0)	// 0x8388
#define			ZeroServoFilterTableX_coeff1_2	(0x0004 + ZeroServoFilterTableX_coeff1_1)	// 0x838C
#define			ZeroServoFilterTableX_g45main	(0x0004 + ZeroServoFilterTableX_coeff1_2)	// 0x8390
#define			ZeroServoFilterTableX_g45sub	(0x0004 + ZeroServoFilterTableX_g45main)	// 0x8394
#define			ZeroServoFilterTableX_gcora		(0x0004 + ZeroServoFilterTableX_g45sub)		// 0x8398
#define			ZeroServoFilterTableX_gaina		(0x0004 + ZeroServoFilterTableX_gcora)		// 0x839C
#define			ZeroServoFilterTableX_shift		(0x0004 + ZeroServoFilterTableX_gaina)		// 0x83A0

#define		ZeroServoFilterTableY			0x83A4
#define			ZeroServoFilterTableY_coeff1_0	(0x0000 + ZeroServoFilterTableY)			// 0x83A4
#define			ZeroServoFilterTableY_coeff1_1	(0x0004 + ZeroServoFilterTableY_coeff1_0)	// 0x83A8
#define			ZeroServoFilterTableY_coeff1_2	(0x0004 + ZeroServoFilterTableY_coeff1_1)	// 0x83AC
#define			ZeroServoFilterTableY_g45main	(0x0004 + ZeroServoFilterTableY_coeff1_2)	// 0x83B0
#define			ZeroServoFilterTableY_g45sub	(0x0004 + ZeroServoFilterTableY_g45main)	// 0x83B4
#define			ZeroServoFilterTableY_gcora		(0x0004 + ZeroServoFilterTableY_g45sub)		// 0x83B8
#define			ZeroServoFilterTableY_gaina		(0x0004 + ZeroServoFilterTableY_gcora)		// 0x83BC
#define			ZeroServoFilterTableY_shift		(0x0004 + ZeroServoFilterTableY_gaina)		// 0x83C0

#define		MeasureFilterA_Coeff			0x83C4	//0x8380
// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterA_Coeff_b1			(0x0000 + MeasureFilterA_Coeff)
#define			MeasureFilterA_Coeff_c1			(0x0004 + MeasureFilterA_Coeff_b1)
#define			MeasureFilterA_Coeff_a1			(0x0004 + MeasureFilterA_Coeff_c1)
#define			MeasureFilterA_Coeff_b2			(0x0004 + MeasureFilterA_Coeff_a1)
#define			MeasureFilterA_Coeff_c2			(0x0004 + MeasureFilterA_Coeff_b2)
#define			MeasureFilterA_Coeff_a2			(0x0004 + MeasureFilterA_Coeff_c2)

#define		MeasureFilterB_Coeff			0x83DC	//0x8398
// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterB_Coeff_b1			(0x0000 + MeasureFilterB_Coeff)
#define			MeasureFilterB_Coeff_c1			(0x0004 + MeasureFilterB_Coeff_b1)
#define			MeasureFilterB_Coeff_a1			(0x0004 + MeasureFilterB_Coeff_c1)
#define			MeasureFilterB_Coeff_b2			(0x0004 + MeasureFilterB_Coeff_a1)
#define			MeasureFilterB_Coeff_c2			(0x0004 + MeasureFilterB_Coeff_b2)
#define			MeasureFilterB_Coeff_a2			(0x0004 + MeasureFilterB_Coeff_c2)

#define		OLAF_DMB_FT						0x8510

#define		OLAF_COEF						0x854C
#define			OLAF_COEF_FSTVAL0				(0x0000 + OLAF_COEF)
#define			OLAF_COEF_FSTVAL1				(0x0004 + OLAF_COEF)
#define			OLAF_COEF_FSTVAL2				(0x0008 + OLAF_COEF)


#define		CommandDecodeTable				0x85AC
// Command.cpp  CommandTable in Rom
#define			CommandDecodeTable_08			(0x0020 + CommandDecodeTable)

#define		GCNV_XX							0x86A4
#define		GCNV_XY							0x86A8
#define		GCNV_YY							0x86AC
#define		GCNV_YX							0x86B0
#define		GCNV_ZP							0x86C8
#define		ACNV_XX							0x86B4
#define		ACNV_XY							0x86B8
#define		ACNV_YY							0x867C
#define		ACNV_YX							0x8680
#define		ACNV_ZP							0x8684

#define		Accl45Filter					0x8780
#define			Accl45Filter_XAdir				(0x0000 + Accl45Filter )
#define			Accl45Filter_XAmain				(0x0004 + Accl45Filter )
#define			Accl45Filter_XAsub				(0x0008 + Accl45Filter )
#define			Accl45Filter_YAdir				(0x000C + Accl45Filter )
#define			Accl45Filter_YAmain				(0x0010 + Accl45Filter )
#define			Accl45Filter_YAsub				(0x0014 + Accl45Filter )

#define		ZS_LMT							0x87A4
#define			ZS_LMT_limitx					(0x0000 + ZS_LMT )
#define			ZS_LMT_limity					(0x0004 + ZS_LMT )


#define			FRA_DMB_C0 					0x8908		//FRA_DMB.C0
#define			FRA_DMB_S0 					0x890C		//FRA_DMB.S0
#define			FRA_DMB_CN 					0x8910		//FRA_DMB.CN
#define			FRA_DMB_SN 					0x8914		//FRA_DMB.SN


//==============================================================================
//IO
//==============================================================================
// System Control配置アドレス
#define 	PERICLKON						0xD00000
#define 	SYSDSP_DSPDIV					0xD00014
#define 	IOPLEV							0xD00020
#define 	IOPDIR							0xD00024
#define 	IOPUDON							0xD00028
#define 	IOPUD							0xD0002C
#define 	SYSDSP_SOFTRES					0xD0006C
#define 	SYSDSP_STBOTH					0xD00078
#define 	SYSDSP_DACI						0xD00088
#define 	SYSDSP_OPGSEL					0xD0008C
#define 	OSCRSEL							0xD00090
#define 	OSCCURSEL						0xD00094
#define 	FRQTRM							0xD00098
#define 	SYSDSP_REMAP					0xD000AC
#define 	OSCCNT							0xD000D4
#define 	SYSDSP_CVER						0xD00100
#define 	IOPLEVR							0xD00104
#define 	OSCCKCNT						0xD00108

#define 	ADDA_FSCNT						0xD01004
#define 	ADDA_FSCTRL						0xD01008
#define 	ADDA_ADDAINT					0xD0100C
#define 	ADDA_ADE						0xD01010
#define 	ADDA_ADAV						0xD01014
#define 	ADDA_ADORDER					0xD01018
#define 	ADDA_EXTEND						0xD0101C
#define 	ADDA_AD0O						0xD01020
#define 	ADDA_AD1O						0xD01024
#define 	ADDA_AD2O						0xD01028
#define 	ADDA_AD3O						0xD0102C

#define 	ADDA_DASELW						0xD01040
#define 	ADDA_DASU						0xD01044
#define 	ADDA_DAHD						0xD01048
#define 	ADDA_DASWAP						0xD0104C
#define 	ADDA_DASEL						0xD01050
#define	HLXO								0x00000001			// D/A Converter Channel Select HLXO
#define	HLYO								0x00000002			// D/A Converter Channel Select HLYO
#define	HLXBO								0x00000004			// D/A Converter Channel Select HLXBO
#define	HLYBO								0x00000008			// D/A Converter Channel Select HLYBO
#define	HLAFO								0x00000010			// D/A Converter Channel Select HLAFO
#define	HLAFBO								0x00000020			// D/A Converter Channel Select HLAFBO

#define 	ADDA_DAO						0xD01054

// PWM I/F配置アドレス
#define 	OISDRVFC1						0xD02100
#define 	OISDRVFC4						0xD0210C
#define 	OISDRVFC5						0xD02110
#define 	OISDRVFC6						0xD02114
#define 	OISDRVFC7						0xD02118
#define 	OISDRVFC8						0xD0211C
#define 	OISDRVFC9						0xD02120

#define 	DRVCH1SEL						0xD02128
#define 	DRVCH2SEL						0xD0212C

#define 	OISGAINAM						0xD02190
#define 	OISOFSTAM						0xD02194
#define 	OISGAINBM						0xD02198
#define 	OISOFSTBM						0xD0219C

#define 	AFDRVFC1						0xD02200
#define 	AFDRVFC4						0xD0220C
#define 	AFDRVFC5						0xD02210
#define 	AFDRVFC6						0xD02214
#define 	AFDRVFC7						0xD02218

#define 	DRVCH3SEL						0xD02220

#define 	AFGAINM							0xD02290
#define 	AFSOFSTM						0xD02294

//Periphral
#define 	ROMINFO							0xE0500C
#define 	SADR							0xE05030

// E2PROM 配置アドレス
#define 	E2P_RDAT						0xE07000
#define 	E2P_ADR							0xE07008
#define 	E2P_ASCNT						0xE0700C
#define 	E2P_CMD							0xE07010
#define 	E2P_WPB							0xE07014
#define 	E2P_INT							0xE07018

#define 	E2P_WDAT00						0xE07040
#define 	E2P_WDAT01						0xE07044
#define 	E2P_WDAT02						0xE07048
#define 	E2P_WDAT03						0xE0704C
#define 	E2P_WDAT04						0xE07050
#define 	E2P_WDAT05						0xE07054
#define 	E2P_WDAT06						0xE07058
#define 	E2P_WDAT07						0xE0705C
#define 	E2P_WDAT08						0xE07060
#define 	E2P_WDAT09						0xE07064
#define 	E2P_WDAT10						0xE07068
#define 	E2P_WDAT11						0xE0706C
#define 	E2P_WDAT12						0xE07070
#define 	E2P_WDAT13						0xE07074
#define 	E2P_WDAT14						0xE07078
#define 	E2P_WDAT15						0xE0707C
#define 	E2P_DFG							0xE07080

#define 	E2P_RSTB						0xE074CC
#define 	E2P_UNLK_CODE1					0xE07554
#define 	E2P_CLKON						0xE07664
#define 	E2P_UNLK_CODE2					0xE07AA8
#define 	E2P_UNLK_CODE3					0xE07CCC


