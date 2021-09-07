//********************************************************************************
//		<< LC898124 Evaluation Soft>>
//		Program Name	: Ois.h
// 		Explanation		: LC898124 Global Declaration & ProtType Declaration
//		Design			: Y.Yamada
//		History			: First edition	
//********************************************************************************
#include "OisLc898124EP3.h"

//****************************************************
//	MODE SELECTORS (Compile Switches)
//****************************************************
#define		__OIS_UIOIS_GYRO_USE__
//#define		UP_SIDE_GYRO

//#define		__OIS_BIG_ENDIAN__

//#define		EEPROM_FULL_ERASE		// E2Prom full erase

#define	SELECT_MODEL		0	// --- select model ---//
									// 0 : OnePlus 7

#define	SELECT_VENDOR		0x81	// --- select vender ---//
									// 0bit : SEMCO
									// 7bit : OnePlus

#define	MASTER_SLAVE		1	// --- select spi i/f mode ---//
								// 0 : only master
								// 1 : there are master&slave (for dual )

#define	EP3_ES				2	// --- LC898124EP3 ES ---//
								// 0 : none
								// 1 : ES1
								// 2 : ES2

#if (SELECT_VENDOR == 0x01)					// SEMCO
 #define	FW_VER				0x01		//ATMEL Version
 #define	SUB_VER				0x02		//ATMEL SUB Version
#else //(SELECT_VENDOR == 0x03)				// ALL
 #define	FW_VER				0x00		//ATMEL Version for ALL
 #define	SUB_VER				0x00		//ATMEL SUB Version
#endif

								
//#define		ZERO_SERVO
#define		ACCEL_SERVO
#define		SEL_SHIFT_COR
#define		LIN_CRSTLK

#define		USE_FRA			// uncomment if use FRA function

//****************************************************
//	TYPE
//****************************************************
#define		INT8	int8_t//char
#define		INT16	int16_t//short
#define		INT32	int32_t//long
#define		INT64	int64_t//long long
#define		UINT8	uint8_t//unsigned char
#define		UINT16	uint16_t//unsigned short
#define		UINT32	uint32_t//unsigned long
#define		UINT64	uint64_t//unsigned long long

//****************************************************
//	Defines
//****************************************************
/**************** Filter sampling **************/
#define	FS_FREQ			18038.84615F	// 45[MHz]/4/624

#define		SLT_OFFSET_HORIZONTAL_SO2821	(-2218L)
#define		SLT_OFFSET_VERTICAL_SO2821		(2218L)

// Command Status
#define		EXE_END		0x00000002L		// Execute End (Adjust OK)
#define		EXE_ERROR	0x00000003L		// Adjust NG : Execution Failure 
#define		EXE_HXADJ	0x00000006L		// Adjust NG : X Hall NG (Gain or Offset)
#define		EXE_HYADJ	0x0000000AL		// Adjust NG : Y Hall NG (Gain or Offset)
#define		EXE_LXADJ	0x00000012L		// Adjust NG : X Loop NG (Gain)
#define		EXE_LYADJ	0x00000022L		// Adjust NG : Y Loop NG (Gain)
#define		EXE_GXADJ	0x00000042L		// Adjust NG : X Gyro NG (offset)
#define		EXE_GYADJ	0x00000082L		// Adjust NG : Y Gyro NG (offset)

#ifdef	SEL_SHIFT_COR
#define		EXE_GZADJ	0x00400002L		// Adjust NG : Z Gyro NG (offset)
#endif	//SEL_SHIFT_COR

#define		EXE_HXMVER	0x06		// X Err
#define		EXE_HYMVER	0x0A		// Y Err
// Gyro Examination of Acceptance
#define		EXE_GXABOVE	0x06		// X Above
#define		EXE_GXBELOW	0x0A		// X Below
#define		EXE_GYABOVE	0x12		// Y Above
#define		EXE_GYBELOW	0x22		// Y Below

// Common Define
#define	SUCCESS			0x00		// Success
#define	FAILURE			0x01		// Failure

#ifndef ON
 #define	ON				0x01		// ON
 #define	OFF				0x00		// OFF
#endif

#define	X_DIR			0x00		// X Direction
#define	Y_DIR			0x01		// Y Direction
#define Z_DIR			0x02		// Z Direction

// mode
#define		GEA_MINMAX_MODE		0x00		// min, max mode
#define		GEA_MEAN_MODE		0x01		// mean mode

#define		BOTH_ON			0x00
#define		XONLY_ON		0x01
#define		YONLY_ON		0x02
#define		BOTH_OFF		0x03

#ifdef	SEL_SHIFT_COR
//#define		ZEROG_MRGN_Z	(204 << 16)			// Zero G tolerance for Z
//#define		ZEROG_MRGN_XY	(204 << 16)			// Zero G tolerance for XY
//#define		ACCL_SENS		2048
#define		ZEROG_MRGN_Z	(409 << 16)			// Zero G tolerance for Z
#define		ZEROG_MRGN_XY	(409 << 16)			// Zero G tolerance for XY
#define		ACCL_SENS		4096
#define		ACCL_SENS_M		-4096
#endif	//SEL_SHIFT_COR

#define CHECK_SUM_NUM	109		// 0x6D

//****************************************************
//	Command
//****************************************************
#define		OIS_POS_FLG			0x00000100			// OIS position by AF measurement

//==============================================================================
// Calibration Data Memory Map
//==============================================================================
// Calibration Status
#define	CALIBRATION_STATUS		(  0 )
// Hall amplitude Calibration X
#define	HALL_MAX_BEFORE_X		(  1 )
#define	HALL_MIN_BEFORE_X		(  2 )
#define	HALL_MAX_AFTER_X		(  3 )
#define	HALL_MIN_AFTER_X		(  4 )
// Hall amplitude Calibration Y
#define	HALL_MAX_BEFORE_Y		(  5 )
#define	HALL_MIN_BEFORE_Y		(  6 )
#define	HALL_MAX_AFTER_Y		(  7 )
#define	HALL_MIN_AFTER_Y		(  8 )
// Hall Bias/Offset
#define	HALL_BIAS_DAC_X			(  9 )
#define	HALL_OFFSET_DAC_X		( 10 )
#define	HALL_BIAS_DAC_Y			( 11 )
#define	HALL_OFFSET_DAC_Y		( 12 )
// Loop Gain Calibration X
#define	LOOP_GAIN_X				( 13 )
// Loop Gain Calibration Y
#define	LOOP_GAIN_Y				( 14 )
// Lens Center Calibration
#define	MECHA_CENTER_X			( 15 )
#define	MECHA_CENTER_Y			( 16 )
// Optical Center Calibration
#define	OPT_CENTER_X			( 17 )
#define	OPT_CENTER_Y			( 18 )
// Gyro Offset Calibration
#define	GYRO_OFFSET_X			( 19 )
#define	GYRO_OFFSET_Y			( 20 )
// Gyro Gain Calibration
#define	GYRO_GAIN_X				( 21 )
#define	GYRO_GAIN_Y				( 22 )
// AF calibration
#define	OIS_POS_BY_AF_X1		( 23 )
#define	OIS_POS_BY_AF_X2		( 24 )
#define	OIS_POS_BY_AF_X3		( 25 )
#define	OIS_POS_BY_AF_X4		( 26 )
#define	OIS_POS_BY_AF_X5		( 27 )
#define	OIS_POS_BY_AF_X6		( 28 )
#define	OIS_POS_BY_AF_X7		( 29 )
#define	OIS_POS_BY_AF_X8		( 30 )
#define	OIS_POS_BY_AF_X9		( 31 )
// Gyro mixing correction
#define MIXING_HX45X			( 32 )
#define MIXING_HX45Y			( 33 )
#define MIXING_HY45Y			( 34 )
#define MIXING_HY45X			( 35 )
#define MIXING_HXSX				( 36 )
#define MIXING_HYSX				( 36 )
// Gyro angle correction
#define MIXING_GX45X			( 37 )
#define MIXING_GX45Y			( 38 )
#define MIXING_GY45Y			( 39 )
#define MIXING_GY45X			( 40 )
// Liniearity correction
#define LN_POS1					( 41 )
#define LN_POS2					( 42 )
#define LN_POS3					( 43 )
#define LN_POS4					( 44 )
#define LN_POS5					( 45 )
#define LN_POS6					( 46 )
#define LN_POS7					( 47 )
#define LN_STEP					( 48 )
// Factory Gyro Gain Calibration
#define	GYRO_FCTRY_OFST_X		( 49 )
#define	GYRO_FCTRY_OFST_Y		( 50 )
// Gyro Offset Calibration
#define	GYRO_OFFSET_Z			( 51 )
// Accl offset
//#define	ACCL_OFFSET_X			( 52 )
//#define	ACCL_OFFSET_Y			( 53 )
#define	DEFAULT_GAIN_X			( 52 )
#define	DEFAULT_GAIN_Y			( 53 )
#define	ACCL_OFFSET_Z			( 54 )
#define	OIS_POS_BY_AF_Y1		( 55 )
#define	OIS_POS_BY_AF_Y2		( 56 )
#define	OIS_POS_BY_AF_Y3		( 57 )
#define	OIS_POS_BY_AF_Y4		( 58 )
#define	OIS_POS_BY_AF_Y5		( 59 )
#define	OIS_POS_BY_AF_Y6		( 60 )
#define	OIS_POS_BY_AF_Y7		( 61 )
#define	OIS_POS_BY_AF_Y8		( 62 )
#define	OIS_POS_BY_AF_Y9		( 63 )


//****************************************************
//	Generic memory 
//****************************************************
#ifdef __OIS_BIG_ENDIAN__
// Big endian
// Word Data Union
union	WRDVAL{
	INT16	SsWrdVal ;
	UINT16	UsWrdVal ;
	UINT8	UcWrkVal[ 2 ] ;
	signed char		ScWrkVal[ 2 ] ;
	struct {
		UINT8	UcHigVal ;
		UINT8	UcLowVal ;
	} StWrdVal ;
} ;


union	DWDVAL {
	UINT32	UlDwdVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsHigVal ;
		UINT16	UsLowVal ;
	} StDwdVal ;
	struct {
		UINT8	UcRamVa3 ;
		UINT8	UcRamVa2 ;
		UINT8	UcRamVa1 ;
		UINT8	UcRamVa0 ;
	} StCdwVal ;
} ;

union	ULLNVAL {
	UINT64	UllnValue ;
	UINT32	UlnValue[ 2 ] ;
	struct {
		UINT32	UlHigVal ;
		UINT32	UlLowVal ;
	} StUllnVal ;
} ;


// Float Data Union
union	FLTVAL {
	float			SfFltVal ;
	UINT32	UlLngVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsHigVal ;
		UINT16	UsLowVal ;
	} StFltVal ;
} ;

#else	// BIG_ENDDIAN
// Little endian
// Word Data Union
union	WRDVAL{
	INT16	SsWrdVal ;
	UINT16	UsWrdVal ;
	UINT8	UcWrkVal[ 2 ] ;
	signed char		ScWrkVal[ 2 ] ;
	struct {
		UINT8	UcLowVal ;
		UINT8	UcHigVal ;
	} StWrdVal ;
} ;

union	DWDVAL {
	UINT32	UlDwdVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsLowVal ;
		UINT16	UsHigVal ;
	} StDwdVal ;
	struct {
		UINT8	UcRamVa0 ;
		UINT8	UcRamVa1 ;
		UINT8	UcRamVa2 ;
		UINT8	UcRamVa3 ;
	} StCdwVal ;
} ;

union	ULLNVAL {
	UINT64	UllnValue ;
	UINT32	UlnValue[ 2 ] ;
	struct {
		UINT32	UlLowVal ;
		UINT32	UlHigVal ;
	} StUllnVal ;
} ;

// Float Data Union
union	FLTVAL {
	float			SfFltVal ;
	UINT32	UlLngVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsLowVal ;
		UINT16	UsHigVal ;
	} StFltVal ;
} ;
#endif	// __OIS_BIG_ENDIAN__

typedef union WRDVAL	UnWrdVal ;
typedef union DWDVAL	UnDwdVal;
typedef union ULLNVAL	UnllnVal;
typedef union FLTVAL	UnFltVal ;


typedef struct STMESRAM {
	INT32	SlMeasureMaxValue ;
	INT32	SlMeasureMinValue ;
	INT32	SlMeasureAmpValue ;
	INT32	SlMeasureAveValue ;
} stMesRam ;									// Struct Measure Ram

typedef struct {
	UINT8	DrvDir;
	UINT16	Rrmd1ToMacro;
	UINT16	Rrmd1ToInfini;
	UINT16	Freq;
} AF_PARA;

typedef struct {
	UINT32 BiasInit;
	UINT32 OffsetInit;
	UINT32 OffsetMargin;
	UINT32 TargetRange;
	UINT32 TargetMax;
	UINT32 TargetMin;
	UINT32 SinNum;
	UINT32 SinFreq;
	UINT32 SinGain;
	UINT32 DecrementStep;
	UINT32 ActMaxDrive_X;
	UINT32 ActMaxDrive_Y;
	UINT32 ActMinDrive_X;
	UINT32 ActMinDrive_Y;
	UINT16 MagneticOffset_X;
	UINT16 MagneticOffset_Y;
	UINT16 HallMax_X;
	UINT16 HallMax_Y;
	UINT16 HallMin_X;
	UINT16 HallMin_Y;
} ADJ_HALL;

typedef struct {
	UINT32 Hxgain;
	UINT32 Hygain;
	UINT32 NoiseNum;
	UINT32 NoiseFreq;
	UINT32 NoiseGain;
	UINT32 Gap;
	UINT32 XJudgeHigh;
	UINT32 XJudgeLow;
	UINT32 YJudgeHigh;
	UINT32 YJudgeLow;
} ADJ_LOPGAN;
	
typedef struct {
	UINT8 Vendor;
	UINT8 User;
	UINT8 Model;
	UINT8 Version;
	UINT8 SpiMode;
	UINT8 Reserve1;
	UINT8 ActType;
	UINT8 GyroType;
} DSPVER;

typedef struct {
	UINT16	Cmd ;
	UINT8	FWType ;
	const UINT8* DataPM;
	UINT32 LengthPM;
	UINT32 Parity;
	const UINT8* DataDM;
	UINT32 LengthDMA;
	UINT32 LengthDMB;
}DOWNLOAD_TBL ;

//****************************************************
//	Structure of calibration data for GUI
//****************************************************
typedef struct STADJPAR {
	struct {
		UINT32	UlAdjPhs ;				// Hall Adjust Phase

		UINT16	UsHlxCna ;				// Hall Center Value after Hall Adjust
		UINT16	UsHlxMax ;				// Hall Max Value
		UINT16	UsHlxMxa ;				// Hall Max Value after Hall Adjust
		UINT16	UsHlxMin ;				// Hall Min Value
		UINT16	UsHlxMna ;				// Hall Min Value after Hall Adjust
		UINT16	UsHlxGan ;				// Hall Gain Value
		UINT16	UsHlxOff ;				// Hall Offset Value
		UINT16	UsAdxOff ;				// Hall A/D Offset Value
		UINT16	UsHlxCen ;				// Hall Center Value

		UINT16	UsHlyCna ;				// Hall Center Value after Hall Adjust
		UINT16	UsHlyMax ;				// Hall Max Value
		UINT16	UsHlyMxa ;				// Hall Max Value after Hall Adjust
		UINT16	UsHlyMin ;				// Hall Min Value
		UINT16	UsHlyMna ;				// Hall Min Value after Hall Adjust
		UINT16	UsHlyGan ;				// Hall Gain Value
		UINT16	UsHlyOff ;				// Hall Offset Value
		UINT16	UsAdyOff ;				// Hall A/D Offset Value
		UINT16	UsHlyCen ;				// Hall Center Value
	} StHalAdj ;

	struct {
		UINT32	UlLxgVal ;				// Loop Gain X
		UINT32	UlLygVal ;				// Loop Gain Y
	} StLopGan ;

	struct {
		UINT16	UsGxoVal ;				// Gyro A/D Offset X
		UINT16	UsGyoVal ;				// Gyro A/D Offset Y
		UINT16	UsGxoSts ;				// Gyro Offset X Status
		UINT16	UsGyoSts ;				// Gyro Offset Y Status
	} StGvcOff ;
	
	UINT8		UcOscVal ;				// OSC value

} stAdjPar ;

#ifdef	ZERO_SERVO
typedef struct STZEROSERVO {
	INT32				SlOffset ;					// 
	INT32				SlG45m ;					// 
	INT32				SlG45s ;					// 
	INT32				SlGcora ;					// 
	INT32				SlGaina ;					// 
	INT32				SlShift ;					// 
} stZeroServo ;
#endif	//ZERO_SERVO

#ifdef	SEL_SHIFT_COR
typedef struct STPOSOFF {
	struct {
		INT32	Pos[6][3];
	} StPos;
	UINT32		UlAclOfSt ;				//!< accel offset status

} stPosOff ;

typedef struct STACLVAL {
	struct {
		INT32	SlOffsetX ;
		INT32	SlOffsetY ;
		INT32	SlOffsetZ ;
	} StAccel ;

	INT32	SlInvMatrix[9] ;

} stAclVal ;
#endif	//SEL_SHIFT_COR

struct tagMlMixingValue
{
	double	radianX;
	double	radianY;

	double	hx45x;
	double	hy45x;
	double	hy45y;
	double	hx45y;

	UINT8	hxsx;
	UINT8	hysx;

	INT32	hx45xL;		//! for Fixed point
	INT32	hy45xL;		//! for Fixed point
	INT32	hy45yL;		//! for Fixed point
	INT32	hx45yL;		//! for Fixed point

	double XonXmove[7];
	double YonXmove[7];
	double XonYmove[7];
	double YonYmove[7];
};
typedef	struct tagMlMixingValue		mlMixingValue;

struct tagMlLinearityValue
{
	INT32	measurecount;	//! input parameter
	UINT32	*dacX;			//! input parameter
	UINT32	*dacY;			//! input parameter

	double	*positionX;
	double	*positionY;
	UINT16	*thresholdX;
	UINT16	*thresholdY;

	UINT32	*coefAXL;		//! for Fixed point
	UINT32	*coefBXL;		//! for Fixed point
	UINT32	*coefAYL;		//! for Fixed point
	UINT32	*coefBYL;		//! for Fixed point
};
typedef	struct tagMlLinearityValue		mlLinearityValue;

struct ACT_MOV_t {
	int	startcode ;
	int	endcode ;
	int	step ;
} ;

typedef struct ACT_MOV_t	Act_Mov_t ;



//	for SetSinWavePara
#define		SINEWAVE		0
#define		XHALWAVE		1
#define		YHALWAVE		2
#define		ZHALWAVE		3
#define		XACTTEST		10
#define		YACTTEST		11
#define		CIRCWAVE		255


//****************************************************
//	Debug
//****************************************************
#ifdef DEBUG
#include <AT91SAM7S.h>
#include <us.h>
 #define TRACE_INIT(x)			dbgu_init(x)
 #define TRACE(fmt, ...)		dbgu_printf(fmt, ## __VA_ARGS__)
 #define TRACE_DUMP(x,y)		dbg_Dump(x,y)
 #define TRACE_USB(fmt, ...)	dbg_UsbData(fmt, ## __VA_ARGS__)
#else
 #define TRACE_INIT(x)
 #define TRACE(...)
 #define TRACE_DUMP(x,y)
 #define TRACE_USB(...)	
#endif

