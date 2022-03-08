//********************************************************************************
//		<< LC898124 Evaluation Soft >>
//	    Program Name	: OisCmd.c
//		Design			: Y.Shigoeka
//		History			: First edition						
//********************************************************************************
//**************************
//	Include Header File		
//**************************
#include	<stdlib.h>	/* use for abs() */
#include	<math.h>	/* use for sqrt() */

#include	"Ois.h"
#include	"OisAPI.h"
#include	"OisLc898124EP3.h"

#if USE_BOSCH
#include 	"Ois_BOSCH.h"
#include	"bmi2_defs.h"
#include	"bmi260.h"
#include	"bmi2.h"
#endif

//****************************************************
//	MODE SELECTORS (Compile Switches)
//****************************************************
//#define		NEUTRAL_CENTER			// Upper Position Current 0mA Measurement
//#define		NEUTRAL_CENTER_FINE		// Optimize natural center current

#define		HALL_ADJ_SERVO_ON		// 

#define	HALLADJ_FULLCURRENT

#define	TNE_PTP_NO_SIN

//****************************************************
//	LC898124 calibration parameters 
//****************************************************
 #if ((SELECT_VENDOR&0x01) == 0x01)				// SEMCO
	#include 	"LC898124EP3_Calibration_SO2821.h"
 #endif
//****************************************************
//	CUSTOMER NECESSARY CREATING LIST
//****************************************************
/* for I2C communication */ 
extern	void RamWrite32A(int addr, int data);
extern 	void RamRead32A( unsigned short addr, void * data );
extern void	WitTim( unsigned short	UsWitTim );
extern void		ClearLaser( void ) ;
extern short	GetLaser( void ) ;
extern void	OisDis( void );

#if USE_BOSCH
extern int8_t bmi2_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
extern int8_t bmi2_i2c_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len);
extern void bmi2_delay_milli_sec(uint32_t period);
#endif

//****************************************************
//	extern  Function LIST
//****************************************************
extern void	MesFil( UINT8 ) ;									// Measure Filter Setting
extern void	MeasureStart( INT32 , UINT32 , UINT32 ) ;				// Measure Start Function
extern void	MeasureStart2( INT32, INT32, INT32, UINT16 );
extern void	MeasureWait( void ) ;								// Measure Wait
extern void	SetTransDataAdr( UINT16 UsLowAddress , UINT32 UlLowAdrBeforeTrans );

extern void	SetSinWavGenInt( void );

extern	stAdjPar	StAdjPar ;				// Execute Command Parameter
extern	UINT16		UsGzoVal ;				// Gyro A/D Offset X

#ifdef	SEL_SHIFT_COR
extern	stPosOff	StPosOff ;				//!< Execute Command Parameter
extern	stAclVal	StAclVal ;				//!< Execute Command Parameter
#endif	//SEL_SHIFT_COR

extern void DMIOWrite32( UINT32 IOadrs, UINT32 IOdata );

#ifdef	ZERO_SERVO

extern	stZeroServo		StZeroServoX;
extern	stZeroServo		StZeroServoY;
extern	stZeroServo		StZeroServoZ;

INT32					GYROZ_OFFSET;

typedef struct STZEROSERVOMES{
	INT32				SlHallP ;					// 
	INT32				SlAcclP ;					// 
} stZeroServoMes ;

stZeroServoMes			StZeroServoMesX;
stZeroServoMes			StZeroServoMesY;

INT32					UlPostureSt;

#endif	//ZERO_SERVO

//****************************************************
//	Local Function LIST
//****************************************************
#if ((SELECT_VENDOR & 0x80 ) != 0x80)
UINT32	TnePtp ( UINT8	UcDirSel, UINT8	UcBfrAft, ADJ_HALL* p, UINT8 UcSrvSwitch );
UINT32	TneCen( UINT8 UcTneAxs, ADJ_HALL* ptr, UINT8 UcSrvSwitch  );
void	TneOff( UnDwdVal, UINT8 ) ;							// Hall Offset Tuning
void	TneBia( UnDwdVal, UINT8, UINT16, UINT8 ) ;			// Hall Bias Tuning
UINT32	LopGan( UINT8 UcDirSel, ADJ_LOPGAN* ptr );
#endif //((SELECT_VENDOR & 0x80 ) != 0x80)
void	TneHvc( void );
UINT32	TneGvc( void );
void	TneFin( ADJ_LOPGAN* ptr );

//****************************************************
//	Parameter LIST
//****************************************************
//#define 	HALL_ADJ		0
//#define 	LOOPGAIN		1
#define 	THROUGH			2
#define 	NOISE			3
#define		OSCCHK			4
// Measure Mode

#define		PTP_BEFORE		0
#define		PTP_AFTER		1

#define 	TNE 			80								// Waiting Time For Movement
#define 	OFFSET_DIV		2								// Divide Difference For Offset Step
#define 	TIME_OUT		20								// Time Out Count

#define		BIAS_HLMT		(0xBF000000 )
#define		BIAS_LLMT		(0x20000000 )

//#ifdef	ZERO_SERVO
/************** posture check ************/
#define		SENSITIVITY		4096						// LSB/g
#define		PSENS_MARG		(SENSITIVITY / 4)			// 1/4g
#define		POSTURETH_P		(SENSITIVITY - PSENS_MARG)	// LSB/g
#define		POSTURETH_M		(-POSTURETH_P)				// LSB/g
/************** posture check ************/
//#endif	//ZERO_SERVO




#define		HAll_SCAILING




//****************************************************
//	LOCAL RAM LIST
//****************************************************
INT16		SsNvcX = 1 ;									// NVC move direction X
INT16		SsNvcY = 1 ;									// NVC move direction Y

UINT8	BeforeControl;

Act_Mov_t	StActMov = {
	0x19998000,
	0xE6668000,
	0xFCCD0000
} ;

//UINT32	UlHall_ActMov[ 16 ] ;

//********************************************************************************
// Function Name 	: DacControl
// Retun Value		: Firmware version
// Argment Value	: NON
// Explanation		: Dac Control Function
// History			: First edition 						
//********************************************************************************
void	DacControl( UINT32 UiChannel, UINT32 PuiData )
{
	DMIOWrite32( ADDA_DASEL, UiChannel );	
	DMIOWrite32( ADDA_DAO, PuiData );	
}

#ifndef __OIS_UIOIS_GYRO_USE__
//********************************************************************************
// Function Name 	: GetGyroOffset
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: start the gyro offset adjustment and get result
// History			: First edition 						
//********************************************************************************
#define 	GYROF_NUM		2048			// 2048times
#define 	GYROF_UPPER		0x06D6			// 
#define 	GYROF_LOWER		0xF92A			// 
UINT32	TneGvc( void )
{
	UINT32	UlRsltSts;
	INT32			SlMeasureParameterA , SlMeasureParameterB ;
	INT32			SlMeasureParameterNum ;
	UnllnVal		StMeasValueA , StMeasValueB ;
	INT32			SlMeasureAveValueA , SlMeasureAveValueB ;
	
	
	//平均値測定
	
	MesFil( THROUGH ) ;					// Set Measure Filter

	SlMeasureParameterNum	=	GYROF_NUM ;					// Measurement times
	SlMeasureParameterA		=	GYRO_RAM_GX_ADIDAT ;		// Set Measure RAM Address
	SlMeasureParameterB		=	GYRO_RAM_GY_ADIDAT ;		// Set Measure RAM Address
	
	MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure
	
	MeasureWait() ;					// Wait complete of measurement
	
TRACE("Read Adr = %04x, %04xh \n",StMeasFunc_MFA_LLiIntegral1 + 4 , StMeasFunc_MFA_LLiIntegral1) ;
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4		, &StMeasValueA.StUllnVal.UlHigVal ) ;
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;
	
TRACE("GX_OFT = %08x, %08xh \n",(unsigned int)StMeasValueA.StUllnVal.UlHigVal,(unsigned int)StMeasValueA.StUllnVal.UlLowVal) ;
TRACE("GY_OFT = %08x, %08xh \n",(unsigned int)StMeasValueB.StUllnVal.UlHigVal,(unsigned int)StMeasValueB.StUllnVal.UlLowVal) ;
	SlMeasureAveValueA = (INT32)( (INT64)StMeasValueA.UllnValue / SlMeasureParameterNum ) ;
	SlMeasureAveValueB = (INT32)( (INT64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;
TRACE("GX_AVEOFT = %08xh \n",(unsigned int)SlMeasureAveValueA) ;
TRACE("GY_AVEOFT = %08xh \n",(unsigned int)SlMeasureAveValueB) ;
	
	SlMeasureAveValueA = ( SlMeasureAveValueA >> 16 ) & 0x0000FFFF ;
	SlMeasureAveValueB = ( SlMeasureAveValueB >> 16 ) & 0x0000FFFF ;
	// EP1では反転処理しない。
//	SlMeasureAveValueA = 0x00010000 - SlMeasureAveValueA ;
//	SlMeasureAveValueB = 0x00010000 - SlMeasureAveValueB ;
	
	UlRsltSts = EXE_END ;
	StAdjPar.StGvcOff.UsGxoVal = ( UINT16 )( SlMeasureAveValueA & 0x0000FFFF );		//Measure Result Store
	if(( (INT16)StAdjPar.StGvcOff.UsGxoVal > (INT16)GYROF_UPPER ) || ( (INT16)StAdjPar.StGvcOff.UsGxoVal < (INT16)GYROF_LOWER )){
		UlRsltSts |= EXE_GXADJ ;
	}
	RamWrite32A( GYRO_RAM_GXOFFZ , (( SlMeasureAveValueA << 16 ) & 0xFFFF0000 ) ) ;		// X axis Gyro offset
	
	StAdjPar.StGvcOff.UsGyoVal = ( UINT16 )( SlMeasureAveValueB & 0x0000FFFF );		//Measure Result Store
	if(( (INT16)StAdjPar.StGvcOff.UsGyoVal > (INT16)GYROF_UPPER ) || ( (INT16)StAdjPar.StGvcOff.UsGyoVal < (INT16)GYROF_LOWER )){
		UlRsltSts |= EXE_GYADJ ;
	}
	RamWrite32A( GYRO_RAM_GYOFFZ , (( SlMeasureAveValueB << 16 ) & 0xFFFF0000 ) ) ;		// Y axis Gyro offset
	
TRACE("GX_AVEOFT_RV = %08xh \n",(unsigned int)SlMeasureAveValueA) ;
TRACE("GY_AVEOFT_RV = %08xh \n",(unsigned int)SlMeasureAveValueB) ;
	
	RamWrite32A( GYRO_RAM_GYROX_OFFSET , 0x00000000 ) ;			// X axis Drift Gyro offset
	RamWrite32A( GYRO_RAM_GYROY_OFFSET , 0x00000000 ) ;			// Y axis Drift Gyro offset
	RamWrite32A( GyroFilterDelayX_GXH1Z2 , 0x00000000 ) ;		// X axis H1Z2 Clear
	RamWrite32A( GyroFilterDelayY_GYH1Z2 , 0x00000000 ) ;		// Y axis H1Z2 Clear
	
	return( UlRsltSts );
	
		
}
#endif


#if ((SELECT_VENDOR & 0x80 ) != 0x80)
//********************************************************************************
// Function Name 	: HallAdj
// Retun Value		: Hall Tuning SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: Hall System Auto Adjustment Function
// History			: First edition 						
//********************************************************************************
UINT32 HallAdj( ADJ_HALL* Ptr, ADJ_LOPGAN *LopgainPtr )
{
	UINT32	UlHlxSts, UlHlySts, UlReadVal;
	
	RtnCen( BOTH_OFF ) ;		// Both OFF
	WitTim( TNE ) ;
#ifdef	HALL_ADJ_SERVO_ON
	RamWrite32A( HALL_RAM_HXOFF,  0x00000000 ) ;		// X Offset Clr
	RamWrite32A( HALL_RAM_HYOFF,  0x00000000 ) ;		// Y Offset Clr
	RamWrite32A( HallFilterCoeffX_hxgain0 , LopgainPtr->Hxgain ) ;
	RamWrite32A( HallFilterCoeffY_hygain0 , LopgainPtr->Hygain ) ;
#endif
	DacControl( HLXBO , Ptr->BiasInit ) ;
	RamWrite32A( StCaliData_UiHallBias_X , Ptr->BiasInit ) ;
	DacControl( HLYBO , Ptr->BiasInit ) ;
	RamWrite32A( StCaliData_UiHallBias_Y , Ptr->BiasInit ) ;
	DacControl( HLXO, Ptr->OffsetInit ) ;
	RamWrite32A( StCaliData_UiHallOffset_X , Ptr->OffsetInit ) ;
	DacControl( HLYO, Ptr->OffsetInit ) ;
	RamWrite32A( StCaliData_UiHallOffset_Y , Ptr->OffsetInit ) ;
	
	BeforeControl=1;
	UlHlySts = TneCen( Y_DIR, Ptr, OFF ) ;
	StAdjPar.StHalAdj.UsHlyCna	-= Ptr->MagneticOffset_Y ;
	StAdjPar.StHalAdj.UsHlyCna	= ( UINT16 )( ( INT16 )StAdjPar.StHalAdj.UsHlyCna - ( INT16 )Ptr->MagneticOffset_Y ) ;
	RamWrite32A( HALL_RAM_HYOFF, ( ( ( UINT32 )StAdjPar.StHalAdj.UsHlyCna << 16 ) & 0xFFFF0000 ) ) ;

	WitTim( TNE ) ;

	BeforeControl=1;
	UlHlxSts = TneCen( X_DIR, Ptr, OFF ) ;
	StAdjPar.StHalAdj.UsHlxCna	= ( UINT16 )( ( INT16 )StAdjPar.StHalAdj.UsHlxCna - ( INT16 )Ptr->MagneticOffset_X ) ;
	RamWrite32A( HALL_RAM_HXOFF, ( ( ( UINT32 )StAdjPar.StHalAdj.UsHlxCna << 16 ) & 0xFFFF0000 ) ) ;

	if( (UlHlxSts != EXE_HXADJ) && (UlHlySts != EXE_HYADJ) ){
#ifdef	HALL_ADJ_SERVO_ON
		RtnCen( BOTH_ON ) ;		// X/Y Servo ON
#endif
		WitTim( TNE ) ;

		UlHlySts = TneCen( Y_DIR, Ptr, ON ) ;
		StAdjPar.StHalAdj.UsHlyCna	= ( UINT16 )( ( INT16 )StAdjPar.StHalAdj.UsHlyCna - ( INT16 )Ptr->MagneticOffset_Y ) ;
		RamWrite32A( HALL_RAM_HYOFF, ( ( ( UINT32 )StAdjPar.StHalAdj.UsHlyCna << 16 ) & 0xFFFF0000 ) ) ;

		WitTim( TNE ) ;
		UlHlxSts = TneCen( X_DIR, Ptr, ON ) ;
		StAdjPar.StHalAdj.UsHlxCna	= ( UINT16 )( ( INT16 )StAdjPar.StHalAdj.UsHlxCna - ( INT16 )Ptr->MagneticOffset_X ) ;
		RamWrite32A( HALL_RAM_HXOFF, ( ( ( UINT32 )StAdjPar.StHalAdj.UsHlxCna << 16 ) & 0xFFFF0000 ) ) ;
#ifdef	HALL_ADJ_SERVO_ON
		RtnCen( BOTH_OFF ) ;		// Both OFF
#endif
		WitTim( TNE ) ;
	}	
	
	RamRead32A( StCaliData_UiHallOffset_X , &UlReadVal ) ;
	StAdjPar.StHalAdj.UsHlxOff = (UINT16)( UlReadVal >> 16 ) ;
		
	RamRead32A( StCaliData_UiHallBias_X , &UlReadVal ) ;
	StAdjPar.StHalAdj.UsHlxGan = (UINT16)( UlReadVal >> 16 ) ;
		
	RamRead32A( StCaliData_UiHallOffset_Y , &UlReadVal ) ;
	StAdjPar.StHalAdj.UsHlyOff = (UINT16)( UlReadVal >> 16 ) ;
		
	RamRead32A( StCaliData_UiHallBias_Y , &UlReadVal ) ;
	StAdjPar.StHalAdj.UsHlyGan = (UINT16)( UlReadVal >> 16 ) ;


#ifdef HAll_SCAILING	//  start : Calculation  Hall Min/max Cut - added 2018/11/16
	UINT16	UsMax_hall_cut_X, UsMin_hall_cut_X ;
	UINT16	UsMax_hall_cut_Y, UsMin_hall_cut_Y ;
	INT16	SsTmp ;

	UsMax_hall_cut_X	= Ptr->HallMax_X ;
	UsMin_hall_cut_X	= Ptr->HallMin_X ;

	UsMax_hall_cut_Y	= Ptr->HallMax_Y ;
	UsMin_hall_cut_Y	= Ptr->HallMin_Y ;

	// Xch hall Cut
	SsTmp						= ( INT16 )StAdjPar.StHalAdj.UsHlxMxa ;
	TRACE( "Hall X Max = %d\n", SsTmp ) ;
	SsTmp						= ( INT16 )( ( INT32 )SsTmp * ( 100 - UsMax_hall_cut_X * 2 ) / 100 ) ;
	TRACE( "Hall X Max Cut = %d\n", SsTmp ) ;
	StAdjPar.StHalAdj.UsHlxMxa	= ( UINT16 )SsTmp ;
	RamWrite32A( StCaliData_SiHallMax_After_X, ( ( ( UINT32 )StAdjPar.StHalAdj.UsHlxMxa << 16 ) & 0xFFFF0000 ) ) ;

	SsTmp						= ( INT16 )StAdjPar.StHalAdj.UsHlxMna ;
	TRACE( "Hall X Min = %d\n", SsTmp ) ;
	SsTmp						= ( INT16 )( ( INT32 )SsTmp * ( 100 - UsMin_hall_cut_X * 2 ) / 100 ) ;
	TRACE( "Hall X Min Cut = %d\n", SsTmp ) ;
	StAdjPar.StHalAdj.UsHlxMna	= ( UINT16 )SsTmp ;
	RamWrite32A( StCaliData_SiHallMin_After_X, ( ( ( UINT32 )StAdjPar.StHalAdj.UsHlxMna << 16 ) & 0xFFFF0000 ) ) ;

	// Ych hall Cut
	SsTmp						= ( INT16 )StAdjPar.StHalAdj.UsHlyMxa ;
	TRACE( "Hall Y Max = %d\n", SsTmp ) ;
	SsTmp						= ( INT16 )( ( INT32 )SsTmp * ( 100 - UsMax_hall_cut_Y * 2 ) / 100 ) ;
	TRACE( "Hall Y Max Cut = %d\n", SsTmp ) ;
	StAdjPar.StHalAdj.UsHlyMxa	= ( UINT16 )SsTmp ;
	RamWrite32A( StCaliData_SiHallMax_After_Y, ( ( ( UINT32 )StAdjPar.StHalAdj.UsHlyMxa << 16 ) & 0xFFFF0000 ) ) ;
	
	SsTmp						= ( INT16 )StAdjPar.StHalAdj.UsHlyMna ;
	TRACE( "Hall Y Min = %d\n", SsTmp ) ;
	SsTmp						= ( INT16 )( ( INT32 )SsTmp * ( 100 - UsMin_hall_cut_Y * 2 ) / 100 ) ;
	TRACE( "Hall Y Min Cut = %d\n", SsTmp ) ;
	StAdjPar.StHalAdj.UsHlyMna	= ( UINT16 )SsTmp ;
	RamWrite32A( StCaliData_SiHallMin_After_Y, ( ( ( UINT32 )StAdjPar.StHalAdj.UsHlyMna << 16 ) & 0xFFFF0000 ) ) ;
#endif	//	end : Calculation  Hall Min/max Cut - added 2018/11/16


#ifdef	HALL_ADJ_SERVO_ON
	RtnCen( BOTH_OFF ) ;		// Both OFF
	WitTim( TNE ) ;
#endif	
	return ( UlHlySts | UlHlxSts );
}


//********************************************************************************
// Function Name 	: TneRun
// Retun Value		: Hall Tuning SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: Hall System Auto Adjustment Function
// History			: First edition 						
//********************************************************************************
UINT32	TneRun( void )
{
	UINT32	UlFinSts, UlReadVal;
	ADJ_HALL* HallPtr;
	ADJ_LOPGAN* LopgainPtr;
	DSPVER Info;

	// Check the status
	RamWrite32A( CMD_IO_ADR_ACCESS , ROMINFO );
	RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	if( UlReadVal != 0x01)	return( EXE_ERROR );

	// Select parameter
	if( GetInfomationAfterDownload( &Info ) != 0) return( EXE_ERROR );

	else if( Info.ActType == ACT_SO2821 ) {
		HallPtr = 		(ADJ_HALL*)&SO2821_HallCalParameter;
		LopgainPtr = (ADJ_LOPGAN* )&SO2821_LoopGainParameter;
	}else{
		return( EXE_ERROR );
	}

	/* Hall Adjustment */
	UlFinSts = HallAdj( HallPtr, LopgainPtr );
//	if( Info.ActType == ACT_SEMCO ) {
//		if( ((UlFinSts & EXE_HXADJ) == EXE_HXADJ) || ((UlFinSts & EXE_HYADJ) == EXE_HYADJ) ){
//			HallPtr = (ADJ_HALL*)&SO_HallCalParameter_F;
//			UlFinSts = HallAdj( HallPtr );
//		}
//	}else{
		if( ((UlFinSts & EXE_HXADJ) == EXE_HXADJ) || ((UlFinSts & EXE_HYADJ) == EXE_HYADJ) ) return ( UlFinSts );
//	}

	/* Hall Offser (neutral center)*/
#ifdef	NEUTRAL_CENTER
	TneHvc();
#endif

#ifdef	NEUTRAL_CENTER_FINE
	TneFin( LopgainPtr );
#endif

//20180906 Komori
	StAdjPar.StHalAdj.UsAdxOff = StAdjPar.StHalAdj.UsHlxCna  ;
	StAdjPar.StHalAdj.UsAdyOff = StAdjPar.StHalAdj.UsHlyCna  ;
TRACE("    Xadof = %04xh \n", StAdjPar.StHalAdj.UsAdxOff ) ;
TRACE("    Yadof = %04xh \n", StAdjPar.StHalAdj.UsAdyOff ) ;

	RamWrite32A( HALL_RAM_HXOFF,  (UINT32)((StAdjPar.StHalAdj.UsAdxOff << 16 ) & 0xFFFF0000 )) ;
	RamWrite32A( HALL_RAM_HYOFF,  (UINT32)((StAdjPar.StHalAdj.UsAdyOff << 16 ) & 0xFFFF0000 )) ;
//20180906 Komori
	
	/* Loop gain Adjustment */
	RamWrite32A( HallFilterCoeffX_hxgain0 , LopgainPtr->Hxgain ) ;
	RamWrite32A( HallFilterCoeffY_hygain0 , LopgainPtr->Hygain ) ;
	RtnCen( BOTH_ON ) ;		// Y ON / X ON
	WitTim( TNE ) ;
	UlFinSts |= LopGan( X_DIR, LopgainPtr ) ;	// X Loop Gain Adjust
	UlFinSts |= LopGan( Y_DIR, LopgainPtr ) ;	// Y Loop Gain Adjust

	/* Gyro DC offset Adjustment */
#ifdef __OIS_UIOIS_GYRO_USE__
#else
	UlFinSts |= TneGvc() ;
#endif
	StAdjPar.StHalAdj.UlAdjPhs = UlFinSts ;
	return( UlFinSts ) ;
}


//********************************************************************************
// Function Name 	: TnePtp
// Retun Value		: Hall Top & Bottom Gaps
// Argment Value	: X,Y Direction, Adjust Before After Parameter
// Explanation		: Measuring Hall Paek To Peak
// History			: First edition 						
//********************************************************************************
UINT32	TnePtp ( UINT8	UcDirSel, UINT8	UcBfrAft, ADJ_HALL* p, UINT8 UcSrvSwitch )
{
	UnDwdVal		StTneVal ;
	INT32			SlMeasureParameterA , SlMeasureParameterB ;
	INT32			SlMeasureParameterNum ;
	INT32			SlMeasureMaxValue , SlMeasureMinValue ;
	INT32			sl_act_min_drv, sl_act_max_drv ;
	UINT16			UsSinAdr ;

TRACE("TnePtp\n ") ;
#ifdef	HALLADJ_FULLCURRENT
	if( UcSrvSwitch != ON ) {
		DMIOWrite32( OISDRVFC1 , 0x00000003 );
	}
#endif	//HALLADJ_FULLCURRENT

	if( UcDirSel == X_DIR ) {								// X axis
		SlMeasureParameterA		=	HALL_RAM_HXIDAT ;		// Set Measure RAM Address
		SlMeasureParameterB		=	HALL_RAM_HYIDAT ;		// Set Measure RAM Address
		if( UcSrvSwitch != ON ) {
			UsSinAdr = HALL_RAM_SINDX1;
		} else {
			UsSinAdr = HALL_RAM_HXOFF1;
		}
		sl_act_max_drv			=	p->ActMaxDrive_X ;
		sl_act_min_drv			=	p->ActMinDrive_X ;
	} else if( UcDirSel == Y_DIR ) {						// Y axis
		SlMeasureParameterA		=	HALL_RAM_HYIDAT ;		// Set Measure RAM Address
		SlMeasureParameterB		=	HALL_RAM_HXIDAT ;		// Set Measure RAM Address
		if( UcSrvSwitch != ON ) {
			UsSinAdr = HALL_RAM_SINDY1;
		} else {
			UsSinAdr = HALL_RAM_HYOFF1;
		}
		sl_act_max_drv			=	p->ActMaxDrive_Y ;
		sl_act_min_drv			=	p->ActMinDrive_Y ;
	}
	
	MesFil( THROUGH ) ;					// Filter setting for measurement

	SlMeasureParameterNum	=	2000 ;

	MeasureStart2( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB, 50 ) ;		// Start measure
	
	if( UcSrvSwitch != ON ) {
		RamWrite32A( UsSinAdr, 0x7FFFFFFF ) ;
	} else {
		RamWrite32A( UsSinAdr, sl_act_min_drv ) ;
	}

	MeasureWait() ;						// Wait complete of measurement
	
	RamRead32A( StMeasFunc_MFA_SiMin1 , ( UINT32 * )&SlMeasureMinValue ) ;	// Min value
	
	SlMeasureParameterNum	=	2000 ;
	
	MeasureStart2( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB, 50 ) ;		// Start measure
	
	if( UcSrvSwitch != ON ) {
		RamWrite32A( UsSinAdr, 0x80000000 ) ;
	} else {
		RamWrite32A( UsSinAdr, sl_act_max_drv ) ;
	}
	
	MeasureWait() ;						// Wait complete of measurement

	RamRead32A( StMeasFunc_MFA_SiMax1 , ( UINT32 * )&SlMeasureMaxValue ) ;	// Max value

	StTneVal.StDwdVal.UsHigVal = (UINT16)((SlMeasureMaxValue >> 16) & 0x0000FFFF );
	StTneVal.StDwdVal.UsLowVal = (UINT16)((SlMeasureMinValue >> 16) & 0x0000FFFF );

	RamWrite32A( UsSinAdr, 0 ) ;
	
#ifdef	HALLADJ_FULLCURRENT
	if( UcSrvSwitch != ON ) {
		DMIOWrite32( OISDRVFC1 , 0x00000000 );
	}
#endif	//HALLADJ_FULLCURRENT

TRACE("\nPTP topbtm H = %04xh , L = %04xh , AXIS = %02x \n", StTneVal.StDwdVal.UsHigVal,StTneVal.StDwdVal.UsLowVal ,UcDirSel ) ;
	
	if( UcBfrAft == 0 ) {
		if( UcDirSel == X_DIR ) {
			StAdjPar.StHalAdj.UsHlxCen	= ( ( INT16 )StTneVal.StDwdVal.UsHigVal + ( INT16 )StTneVal.StDwdVal.UsLowVal ) / 2 ;
			StAdjPar.StHalAdj.UsHlxMax	= StTneVal.StDwdVal.UsHigVal ;
			StAdjPar.StHalAdj.UsHlxMin	= StTneVal.StDwdVal.UsLowVal ;
		} else if( UcDirSel == Y_DIR ){
			StAdjPar.StHalAdj.UsHlyCen	= ( ( INT16 )StTneVal.StDwdVal.UsHigVal + ( INT16 )StTneVal.StDwdVal.UsLowVal ) / 2 ;
			StAdjPar.StHalAdj.UsHlyMax	= StTneVal.StDwdVal.UsHigVal ;
			StAdjPar.StHalAdj.UsHlyMin	= StTneVal.StDwdVal.UsLowVal ;
		}
	} else {
		if( UcDirSel == X_DIR ){
			StAdjPar.StHalAdj.UsHlxCna	= ( ( INT16 )StTneVal.StDwdVal.UsHigVal + ( INT16 )StTneVal.StDwdVal.UsLowVal ) / 2 ;
			StAdjPar.StHalAdj.UsHlxMxa	= StTneVal.StDwdVal.UsHigVal ;
			StAdjPar.StHalAdj.UsHlxMna	= StTneVal.StDwdVal.UsLowVal ;
		} else if( UcDirSel == Y_DIR ){
			StAdjPar.StHalAdj.UsHlyCna	= ( ( INT16 )StTneVal.StDwdVal.UsHigVal + ( INT16 )StTneVal.StDwdVal.UsLowVal ) / 2 ;
			StAdjPar.StHalAdj.UsHlyMxa	= StTneVal.StDwdVal.UsHigVal ;
			StAdjPar.StHalAdj.UsHlyMna	= StTneVal.StDwdVal.UsLowVal ;
		}
	}

TRACE("		ADJ(%d) MAX = %04x, MIN = %04x, CNT = %04x, ", UcDirSel, StTneVal.StDwdVal.UsHigVal, StTneVal.StDwdVal.UsLowVal, ( ( signed int )StTneVal.StDwdVal.UsHigVal + ( signed int )StTneVal.StDwdVal.UsLowVal ) / 2 ) ;
	StTneVal.StDwdVal.UsHigVal	= 0x7fff - StTneVal.StDwdVal.UsHigVal ;		// Maximum Gap = Maximum - Hall Peak Top
	StTneVal.StDwdVal.UsLowVal	= StTneVal.StDwdVal.UsLowVal - 0x8000 ; 	// Minimum Gap = Hall Peak Bottom - Minimum

TRACE("	GapH = %04x, GapL = %04x\n", StTneVal.StDwdVal.UsHigVal, StTneVal.StDwdVal.UsLowVal ) ;
TRACE("		Raw MAX = %08x, MIN = %08x\n", (unsigned int)SlMeasureMaxValue , (unsigned int)SlMeasureMinValue ) ;
	
	return( StTneVal.UlDwdVal ) ;
}

//********************************************************************************
// Function Name 	: TneCen
// Retun Value		: Hall Center Tuning Result
// Argment Value	: X,Y Direction, Hall Top & Bottom Gaps
// Explanation		: Hall Center Tuning Function
// History			: First edition 						
//********************************************************************************
UINT32	TneCen( UINT8 UcTneAxs, ADJ_HALL* ptr, UINT8 UcSrvSwtich )
{
	UnDwdVal		StTneVal ;
	UINT8 	UcTmeOut =1, UcTofRst= FAILURE ;
	UINT16	UsBiasVal ;
	UINT32	UlTneRst = FAILURE, UlBiasVal , UlValNow ;
	UINT16	UsValBef,UsValNow ;
	UINT32	UlBiaBef,UlBiaNow ;
	
	if( BeforeControl != 0 ) {
		StTneVal.UlDwdVal	= TnePtp( UcTneAxs , PTP_BEFORE, ptr, UcSrvSwtich ) ;
	} else {
		StTneVal.UlDwdVal	= TnePtp( UcTneAxs , PTP_AFTER, ptr, UcSrvSwtich ) ;
	}
	BeforeControl=0;

	TneOff( StTneVal, UcTneAxs ) ;
	UcTofRst	= SUCCESS ;				/* 暫定でOKにする */

	while ( UlTneRst && (UINT32)UcTmeOut )
	{
		if( UcTofRst == FAILURE ) {
TRACE(" UcTofRst == FAILURE\n" ) ;
			TneOff( StTneVal, UcTneAxs ) ;
			StTneVal.UlDwdVal = TnePtp( UcTneAxs, PTP_AFTER, ptr, UcSrvSwtich ) ;
		} else {
TRACE(" else\n" ) ;
			if( UcTneAxs == X_DIR ) {
				RamRead32A( StCaliData_UiHallBias_X , &UlBiaBef ) ;		
			} else if( UcTneAxs == Y_DIR ) {
				RamRead32A( StCaliData_UiHallBias_Y , &UlBiaBef ) ;		
			}
			TneBia( StTneVal, UcTneAxs, ptr->TargetRange, UcSrvSwtich ) ;
			if( UcTneAxs == X_DIR ) {
				RamRead32A( StCaliData_UiHallBias_X , &UlBiaNow ) ;		
			} else if( UcTneAxs == Y_DIR ) {
				RamRead32A( StCaliData_UiHallBias_Y , &UlBiaNow ) ;		
			}
			if((( UlBiaBef == BIAS_HLMT ) && ( UlBiaNow == BIAS_HLMT ))
			|| (( UlBiaBef == BIAS_LLMT ) && ( UlBiaNow == BIAS_LLMT ))){
				UcTmeOut += 10;
TRACE("	No = %04d (bias count up)\n", UcTmeOut ) ;
			}
			StTneVal.UlDwdVal	= TnePtp( UcTneAxs , PTP_AFTER, ptr, UcSrvSwtich ) ;

			UcTofRst	= FAILURE ;
//			if( UcTneAxs == X_DIR ) {
//				RamRead32A( StCaliData_UiHallBias_X , &UlBiasVal ) ;
//			}else if( UcTneAxs == Y_DIR ){
//				RamRead32A( StCaliData_UiHallBias_Y , &UlBiasVal ) ;
//			}
//			if(UlBiasVal == 0x00000000){
//				UcTmeOut = TIME_OUT;
//			}
		}

		if( (StTneVal.StDwdVal.UsHigVal > ptr->OffsetMargin ) && (StTneVal.StDwdVal.UsLowVal > ptr->OffsetMargin ) )	/* position check */
		{
			UcTofRst	= SUCCESS ;
TRACE("  TofR = SUCC\n" ) ;
			UsValBef = UsValNow = 0x0000 ;
		}else if( (StTneVal.StDwdVal.UsHigVal <= ptr->OffsetMargin ) && (StTneVal.StDwdVal.UsLowVal <= ptr->OffsetMargin ) ){
			UcTofRst	= SUCCESS ;
			UlTneRst	= (UINT32)FAILURE ;
		}else{
			UcTofRst	= FAILURE ;
TRACE("  TofR = FAIL\n" ) ;
			
			UsValBef = UsValNow ;

			if( UcTneAxs == X_DIR  ) {
				RamRead32A( StCaliData_UiHallOffset_X , &UlValNow ) ;
				UsValNow = (UINT16)( UlValNow >> 16 ) ;
			}else if( UcTneAxs == Y_DIR ){
				RamRead32A( StCaliData_UiHallOffset_Y , &UlValNow ) ;
				UsValNow = (UINT16)( UlValNow >> 16 ) ;
			}
			if( ((( UsValBef & 0xFF00 ) == 0x1000 ) && ( UsValNow & 0xFF00 ) == 0x1000 )
			 || ((( UsValBef & 0xFF00 ) == 0xEF00 ) && ( UsValNow & 0xFF00 ) == 0xEF00 ) )
			{
				UcTmeOut += 10;
TRACE("	No = %04d (offset count up)\n", UcTmeOut ) ;
				if( UcTneAxs == X_DIR ) {
					RamRead32A( StCaliData_UiHallBias_X , &UlBiasVal ) ;
					UsBiasVal = (UINT16)( UlBiasVal >> 16 ) ;
				}else if( UcTneAxs == Y_DIR ){
					RamRead32A( StCaliData_UiHallBias_Y , &UlBiasVal ) ;
					UsBiasVal = (UINT16)( UlBiasVal >> 16 ) ;
				}
				
				if( UsBiasVal > ptr->DecrementStep )
				{
					UsBiasVal -= ptr->DecrementStep ;
				}
				
				if( UcTneAxs == X_DIR ) {
					UlBiasVal = ( UINT32 )( UsBiasVal << 16 ) ;
					DacControl( HLXBO , UlBiasVal ) ;
					RamWrite32A( StCaliData_UiHallBias_X , UlBiasVal ) ;
				}else if( UcTneAxs == Y_DIR ){
					UlBiasVal = ( UINT32 )( UsBiasVal << 16 ) ;
					DacControl( HLYBO , UlBiasVal ) ;
					RamWrite32A( StCaliData_UiHallBias_Y , UlBiasVal ) ;
				}
			}

		}
		
		if((( (UINT16)0xFFFF - ( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal )) < ptr->TargetMax )
		&& (( (UINT16)0xFFFF - ( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal )) > ptr->TargetMin ) ) {
			if(UcTofRst	== SUCCESS)
			{
				UlTneRst	= (UINT32)SUCCESS ;
				break ;
			}
		}
		UlTneRst	= (UINT32)FAILURE ;
		UcTmeOut++ ;
TRACE("  Tne = FAIL\n" ) ;

TRACE("	No = %04d", UcTmeOut ) ;
		if ( UcTmeOut >= TIME_OUT ) {
			UcTmeOut	= 0 ;
		}		 																							// Set Time Out Count
	}

	SetSinWavGenInt() ;		// 
	
	if( UlTneRst == (UINT32)FAILURE ) {
		if( UcTneAxs == X_DIR ) {
			UlTneRst					= EXE_HXADJ ;
			StAdjPar.StHalAdj.UsHlxGan	= 0xFFFF ;
			StAdjPar.StHalAdj.UsHlxOff	= 0xFFFF ;
		}else if( UcTneAxs == Y_DIR ) {
			UlTneRst					= EXE_HYADJ ;
			StAdjPar.StHalAdj.UsHlyGan	= 0xFFFF ;
			StAdjPar.StHalAdj.UsHlyOff	= 0xFFFF ;
		}
	} else {
		UlTneRst	= EXE_END ;
	}

	return( UlTneRst ) ;
}



//********************************************************************************
// Function Name 	: TneBia
// Retun Value		: Hall Top & Bottom Gaps
// Argment Value	: Hall Top & Bottom Gaps , X,Y Direction
// Explanation		: Hall Bias Tuning Function
// History			: First edition 						
//********************************************************************************
void TneBia( UnDwdVal StTneVal, UINT8 UcTneAxs, UINT16 UsHalAdjRange, UINT8 UcSrvSwitch  )
{
	UINT32			UlSetBia, UlOldBias ;
	float			SfAmp ;

TRACE("TneBia\n " ) ;
	if( UcTneAxs == X_DIR ) {
		RamRead32A( StCaliData_UiHallBias_X , &UlSetBia ) ;
	} else if( UcTneAxs == Y_DIR ) {
		RamRead32A( StCaliData_UiHallBias_Y , &UlSetBia ) ;
	}

TRACE("		UlSetBia = %08x\n ", (unsigned int)UlSetBia ) ;
	if( UlSetBia == 0x00000000 )	UlSetBia = 0x01000000 ;
	UlSetBia = (( UlSetBia >> 16 ) & (UINT32)0x0000FF00 ) ;
	UlOldBias	= UlSetBia ;
	UlSetBia *= (UINT32)UsHalAdjRange ;
	if(( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal ) == 0xFFFF ){
		UlSetBia = BIAS_HLMT ;
	}else{
		UlSetBia /= (UINT32)( 0xFFFF - ( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal ) ) ;
		if( UcSrvSwitch != OFF ) {
			SfAmp	= ( ( float )UlSetBia / ( float )UlOldBias - 1.0f ) ;
			SfAmp	= ( SfAmp / 2.0f ) + 1.0f ;
			UlSetBia	= ( UINT32 )( ( float )UlOldBias * SfAmp ) ;
		}
		if( UlSetBia > (UINT32)0x0000FFFF )		UlSetBia = 0x0000FFFF ;
		UlSetBia = ( UlSetBia << 16 ) ;
		if( UlSetBia > BIAS_HLMT )		UlSetBia = BIAS_HLMT ;
		if( UlSetBia < BIAS_LLMT )		UlSetBia = BIAS_LLMT ;
	}

	if( UcTneAxs == X_DIR ) {
		DacControl( HLXBO , UlSetBia ) ;
TRACE("		HLXBO = %08x\n ",  (unsigned int)UlSetBia ) ;
		RamWrite32A( StCaliData_UiHallBias_X , UlSetBia) ;
	} else if( UcTneAxs == Y_DIR ){
		DacControl( HLYBO , UlSetBia ) ;
TRACE("		HLYBO = %08x\n ",  (unsigned int)UlSetBia ) ;
		RamWrite32A( StCaliData_UiHallBias_Y , UlSetBia) ;
	}
TRACE("		( AXIS = %02x , BIAS = %08xh ) , \n", UcTneAxs , (unsigned int)UlSetBia ) ;
}


//********************************************************************************
// Function Name 	: TneOff
// Retun Value		: Hall Top & Bottom Gaps
// Argment Value	: Hall Top & Bottom Gaps , X,Y Direction
// Explanation		: Hall Offset Tuning Function
// History			: First edition 						
//********************************************************************************
void TneOff( UnDwdVal StTneVal, UINT8 UcTneAxs )
{
	UINT32	UlSetOff ;
	UINT32	UlSetVal ;
	
TRACE("TneOff\n ") ;
	if( UcTneAxs == X_DIR ) {
		RamRead32A( StCaliData_UiHallOffset_X , &UlSetOff ) ;
	} else if( UcTneAxs == Y_DIR ){
		RamRead32A( StCaliData_UiHallOffset_Y , &UlSetOff ) ;
	}
	UlSetOff 	= ( UlSetOff >> 16 ) ;

	if ( StTneVal.StDwdVal.UsHigVal > StTneVal.StDwdVal.UsLowVal ) {
		UlSetVal	= ( UINT32 )(( StTneVal.StDwdVal.UsHigVal - StTneVal.StDwdVal.UsLowVal ) / OFFSET_DIV ) ;	// Calculating Value For Increase Step
		UlSetOff	+= UlSetVal ;	// Calculating Value For Increase Step
		if( UlSetOff > 0x0000FFFF )		UlSetOff = 0x0000FFFF ;
	} else {
		UlSetVal	= ( UINT32 )(( StTneVal.StDwdVal.UsLowVal - StTneVal.StDwdVal.UsHigVal ) / OFFSET_DIV ) ;	// Calculating Value For Decrease Step
		if( UlSetOff < UlSetVal ){
			UlSetOff	= 0x00000000 ;
		}else{
			UlSetOff	-= UlSetVal ;	// Calculating Value For Decrease Step
		}
	}

TRACE("		UlSetOff = %08x\n ",  (unsigned int)UlSetOff ) ;
	if( UlSetOff > ( INT32 )0x0000EFFF ) {
		UlSetOff	= 0x0000EFFF ;
	} else if( UlSetOff < ( INT32 )0x00001000 ) {
		UlSetOff	= 0x00001000 ;
	}

	UlSetOff = ( UlSetOff << 16 ) ;
	
	if( UcTneAxs == X_DIR ) {
		DacControl( HLXO, UlSetOff ) ;
TRACE("		HLXO = %08x\n ",  (unsigned int)UlSetOff ) ;
		RamWrite32A( StCaliData_UiHallOffset_X , UlSetOff ) ;
	} else if( UcTneAxs == Y_DIR ){
		DacControl( HLYO, UlSetOff ) ;
TRACE("		HLYO = %08x\n ",  (unsigned int)UlSetOff ) ;
		RamWrite32A( StCaliData_UiHallOffset_Y , UlSetOff ) ;
	}
TRACE("		( AXIS = %02x , OFST = %08xh ) , \n", UcTneAxs , (unsigned int)UlSetOff ) ;

}


//********************************************************************************
// Function Name 	: LopGan
// Retun Value		: Execute Result
// Argment Value	: X,Y Direction
// Explanation		: Loop Gain Adjust Function
// History			: First edition 						
//********************************************************************************
UINT32	LopGan( UINT8 UcDirSel, ADJ_LOPGAN* ptr )
{
#if 1
	UINT32			UlReturnState ;
	
	if( UcDirSel == X_DIR ) {							// X axis
		RamWrite32A( HallFilterCoeffX_hxgain0 , ptr->Hxgain ) ;
		StAdjPar.StLopGan.UlLxgVal = ptr->Hxgain ;
		UlReturnState = EXE_END ;
	} else if( UcDirSel == Y_DIR ){						// Y axis
		RamWrite32A( HallFilterCoeffY_hygain0 , ptr->Hygain ) ;
		StAdjPar.StLopGan.UlLygVal = ptr->Hygain ;
		UlReturnState = EXE_END ;
	}
	
	return( UlReturnState ) ;
#else
	UnllnVal		StMeasValueA , StMeasValueB ;
	INT32			SlMeasureParameterA , SlMeasureParameterB ;
	UINT64	UllCalculateVal ;
	UINT32	UlReturnState ;
	UINT16	UsSinAdr ;
	
	if( UcDirSel == X_DIR ) {		// X axis
TRACE("LopGain X_DIR \n") ;
		SlMeasureParameterA		=	HALL_RAM_HXOUT1 ;		// Set Measure RAM Address
		SlMeasureParameterB		=	HALL_RAM_HXLOP ;		// Set Measure RAM Address
		UsSinAdr = HALL_RAM_SINDX0;
	} else if( UcDirSel == Y_DIR ){						// Y axis
TRACE("LopGain Y_DIR \n") ;
		SlMeasureParameterA		=	HALL_RAM_HYOUT1 ;		// Set Measure RAM Address
		SlMeasureParameterB		=	HALL_RAM_HYLOP ;		// Set Measure RAM Address
		UsSinAdr = HALL_RAM_SINDY0;
	}
	
	SetSinWavGenInt();
	RamWrite32A( SinWave_Offset		,	ptr->NoiseFreq ) ;								// Freq Setting
	RamWrite32A( SinWave_Gain		,	ptr->NoiseGain ) ;								// Set Sine Wave Gain					
	RamWrite32A( SinWaveC_Regsiter	,	0x00000001 ) ;								// Sine Wave Start

TRACE("LopGain NoiseFreq = %08xh \n", (unsigned int)ptr->NoiseFreq ) ;
TRACE("LopGain NoiseGain = %08xh \n", (unsigned int)ptr->NoiseGain ) ;

	SetTransDataAdr( SinWave_OutAddr	,	( UINT32 )UsSinAdr ) ;	// Set Sine Wave Input RAM
	
	MesFil( 1/*LOOPGAIN*/ ) ;					// Filter setting for measurement
	MeasureStart( ptr->NoiseNum , SlMeasureParameterA , SlMeasureParameterB ) ;			// Start measure
	MeasureWait() ;						// Wait complete of measurement

	SetSinWavGenInt();		// Sine wave stop
	
	SetTransDataAdr( SinWave_OutAddr	,	(UINT32)0x00000000 ) ;	// Set Sine Wave Input RAM
	RamWrite32A( UsSinAdr		,	0x00000000 ) ;				// DelayRam Clear
	
	RamRead32A( StMeasFunc_MFA_LLiAbsInteg1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
	RamRead32A( StMeasFunc_MFA_LLiAbsInteg1 + 4 	, &StMeasValueA.StUllnVal.UlHigVal ) ;
	RamRead32A( StMeasFunc_MFB_LLiAbsInteg2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
	RamRead32A( StMeasFunc_MFB_LLiAbsInteg2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;
	
TRACE("LopGain StMeasValueA.UllnValue = %08xh \n", (unsigned int)StMeasValueA.StUllnVal.UlHigVal ) ;
TRACE("LopGain StMeasValueB.UllnValue = %08xh \n", (unsigned int)StMeasValueB.StUllnVal.UlHigVal ) ;
	
	if( UcDirSel == X_DIR ) {		// X axis
		UllCalculateVal = ( StMeasValueB.UllnValue * 1000 / StMeasValueA.UllnValue ) * ptr->Hxgain / ptr->Gap ;
		if( UllCalculateVal > (UINT64)0x000000007FFFFFFF )		UllCalculateVal = (UINT64)0x000000007FFFFFFF ;
		StAdjPar.StLopGan.UlLxgVal = (UINT32)UllCalculateVal ;
		RamWrite32A( HallFilterCoeffX_hxgain0 , StAdjPar.StLopGan.UlLxgVal ) ;
TRACE("LopGain UlLxgVal = %08xh \n", (unsigned int)StAdjPar.StLopGan.UlLxgVal ) ;
		if( (UllCalculateVal > ptr->XJudgeHigh) || ( UllCalculateVal < ptr->XJudgeLow ) ){
			UlReturnState = EXE_LXADJ ;
		}else{
			UlReturnState = EXE_END ;
		}
		
	}else if( UcDirSel == Y_DIR ){							// Y axis
		UllCalculateVal = ( StMeasValueB.UllnValue * 1000 / StMeasValueA.UllnValue ) * ptr->Hygain / ptr->Gap ;
		if( UllCalculateVal > (UINT64)0x000000007FFFFFFF )		UllCalculateVal = (UINT64)0x000000007FFFFFFF ;
		StAdjPar.StLopGan.UlLygVal = (UINT32)UllCalculateVal ;
		RamWrite32A( HallFilterCoeffY_hygain0 , StAdjPar.StLopGan.UlLygVal ) ;
TRACE("LopGain UlLygVal = %08xh \n", (unsigned int)StAdjPar.StLopGan.UlLygVal ) ;
		if( (UllCalculateVal > ptr->YJudgeHigh) || ( UllCalculateVal < ptr->YJudgeLow ) ){
			UlReturnState = EXE_LYADJ ;
		}else{
			UlReturnState = EXE_END ;
		}
	}

TRACE("LopGain UlReturnState = %08xh \n", (unsigned int)UlReturnState ) ;
	return( UlReturnState ) ;
#endif
}
#endif //((SELECT_VENDOR & 0x80 ) != 0x80)

//********************************************************************************
// Function Name 	: SetSinWavePara
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Sine wave Test Function
// History			: First edition 						
//********************************************************************************
/* Servo Sampling Clock		=	18.0288kHz						*/
/* Freq						=	SinFreq*80000000h/Fs			*/
/* 05 00 XX MM 				XX:Freq MM:Sin or Circle */
const UINT32	CucFreqVal[ 17 ]	= {
		0xFFFFFFFF,				//  0:  Stop
		0x0001D14A,				//  1: 1Hz
		0x0003A294,				//  2: 2Hz
		0x000573DE,				//  3: 3Hz	
		0x00074528,				//  4: 4Hz
		0x00091672,				//  5: 5Hz
		0x000AE7BC,				//  6: 6Hz
		0x000CB906,				//  7: 7Hz
		0x000E8A50,				//  8: 8Hz
		0x00105B9A,				//  9: 9Hz
		0x00122CE4,				//  A: 10Hz
		0x0013FE2E,				//  B: 11Hz
		0x0015CF78,				//  C: 12Hz
		0x0017A0C2,				//  D: 13Hz
		0x0019720C,				//  E: 14Hz
		0x001B4356,				//  F: 15Hz
		0x001D14A0				// 10: 16Hz
	} ;
	
void	SetSinWavePara( UINT8 UcTableVal ,  UINT8 UcMethodVal )
{
	UINT32	UlFreqDat ;

	if(UcTableVal > 0x10 )	UcTableVal = 0x10 ;			/* Limit */
	UlFreqDat = CucFreqVal[ UcTableVal ] ;	
	
	if( UcMethodVal == 255/*CIRCWAVE*/) {
		RamWrite32A( SinWave_Phase	,	0x60000000 ) ;		// 正弦波の位相量
		RamWrite32A( CosWave_Phase 	,	0x00000000 );		// 正弦波の位相量
	}else{
		RamWrite32A( SinWave_Phase	,	0x60000000 ) ;		// 正弦波の位相量
		RamWrite32A( CosWave_Phase 	,	0x60000000 );		// 正弦波の位相量
	}

	if( UlFreqDat == 0xFFFFFFFF )			/* Sine波中止 */
	{
		RamWrite32A( SinWave_Offset		,	0x00000000 ) ;									// 発生周波数のオフセットを設定
		RamWrite32A( SinWave_Phase		,	0x60000000 ) ;									// 正弦波の位相量

		RamWrite32A( CosWave_Offset		,	0x00000000 );									// 発生周波数のオフセットを設定
		RamWrite32A( CosWave_Phase 		,	0x60000000 );									// 正弦波の位相量

		RamWrite32A( SinWaveC_Regsiter	,	0x00000000 ) ;									// Sine Wave Stop
		SetTransDataAdr( SinWave_OutAddr	,	0x00000000 ) ;		// 出力先アドレス
		SetTransDataAdr( CosWave_OutAddr	,	0x00000000 );		// 出力先アドレス
		RamWrite32A( HALL_RAM_HXOFF1		,	0x00000000 ) ;				// DelayRam Clear
		RamWrite32A( HALL_RAM_HYOFF1		,	0x00000000 ) ;				// DelayRam Clear
	}else{
		RamWrite32A( SinWave_Offset		,	UlFreqDat ) ;									// 発生周波数のオフセットを設定
		RamWrite32A( CosWave_Offset		,	UlFreqDat );									// 発生周波数のオフセットを設定

		RamWrite32A( SinWaveC_Regsiter	,	0x00000001 ) ;									// Sine Wave Start
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_RAM_HXOFF1 ) ;		// 出力先アドレス
		SetTransDataAdr( CosWave_OutAddr	,	(UINT32)HALL_RAM_HYOFF1 ) ;		// 出力先アドレス

	}
}

//********************************************************************************
// Function Name 	: TneHvc
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes the Hall VC offset
// History			: First edition 				
//********************************************************************************
#define		ADOFF_ROUGH_NUM		64
void	TneHvc( void )
{
	UnllnVal		StMeasValueA , StMeasValueB ;
	INT32			SlMeasureAveValueA , SlMeasureAveValueB ;
	
	RamWrite32A( HALL_RAM_HXOFF,  0x00000000 ) ;		// X Offset Clr
	RamWrite32A( HALL_RAM_HYOFF,  0x00000000 ) ;		// Y Offset Clr
	
	RtnCen( BOTH_OFF ) ;		// Both OFF
	WitTim( 500 ) ;
	
	MesFil( THROUGH ) ;					// Set Measure Filter
	MeasureStart( ADOFF_ROUGH_NUM , HALL_RAM_HXIDAT , HALL_RAM_HYIDAT ) ;					// Start measure
	MeasureWait() ;					// Wait complete of measurement

	RamRead32A( StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4 	, &StMeasValueA.StUllnVal.UlHigVal ) ;
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;

	SlMeasureAveValueA = (INT32)((( (INT64)StMeasValueA.UllnValue * 100 ) / ADOFF_ROUGH_NUM ) / 100 ) ;
	SlMeasureAveValueB = (INT32)((( (INT64)StMeasValueB.UllnValue * 100 ) / ADOFF_ROUGH_NUM ) / 100 ) ;

	StAdjPar.StHalAdj.UsHlxCna = ( UINT16 )(( SlMeasureAveValueA >> 16 ) & 0x0000FFFF );		//Measure Result Store
	StAdjPar.StHalAdj.UsHlxCen = StAdjPar.StHalAdj.UsHlxCna;									//Measure Result Store

	StAdjPar.StHalAdj.UsHlyCna = ( UINT16 )(( SlMeasureAveValueB >> 16 ) & 0x0000FFFF );		//Measure Result Store
	StAdjPar.StHalAdj.UsHlyCen = StAdjPar.StHalAdj.UsHlyCna;											//Measure Result Store

	StAdjPar.StHalAdj.UsAdxOff = StAdjPar.StHalAdj.UsHlxCna  ;
	StAdjPar.StHalAdj.UsAdyOff = StAdjPar.StHalAdj.UsHlyCna  ;
TRACE("    Xadof = %04xh \n", StAdjPar.StHalAdj.UsAdxOff ) ;
TRACE("    Yadof = %04xh \n", StAdjPar.StHalAdj.UsAdyOff ) ;

	RamWrite32A( HALL_RAM_HXOFF,  (UINT32)((StAdjPar.StHalAdj.UsAdxOff << 16 ) & 0xFFFF0000 )) ;
	RamWrite32A( HALL_RAM_HYOFF,  (UINT32)((StAdjPar.StHalAdj.UsAdyOff << 16 ) & 0xFFFF0000 )) ;

}

//********************************************************************************
// Function Name 	: TneFin
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes the Hall VC offset current optimize
// History			: First edition 				
//********************************************************************************
#define		ADOFF_FINE_NUM		2000	
void	TneFin( ADJ_LOPGAN* ptr )
{
	UINT32	UlReadVal ;
	UINT16	UsAdxOff, UsAdyOff ;
	INT32			SlMeasureAveValueA , SlMeasureAveValueB ;
	UnllnVal		StMeasValueA , StMeasValueB ;
	UINT32	UlMinimumValueA, UlMinimumValueB ;
	UINT16	UsAdxMin, UsAdyMin ;
	UINT8	UcFin ;
	
	// Loop gain set for servo
	RamWrite32A( HallFilterCoeffX_hxgain0 , ptr->Hxgain ) ;
	RamWrite32A( HallFilterCoeffY_hygain0 , ptr->Hygain ) ;
	
	// Get natural center offset
	RamRead32A( HALL_RAM_HXOFF,  &UlReadVal ) ;
	UsAdxOff = UsAdxMin = (UINT16)( UlReadVal >> 16 ) ;

	RamRead32A( HALL_RAM_HYOFF,  &UlReadVal ) ;
	UsAdyOff = UsAdyMin = (UINT16)( UlReadVal >> 16 ) ;
//TRACE("*****************************************************\n" );
//TRACE("TneFin: Before Adx=%04X, Ady=%04X\n", UsAdxOff, UsAdyOff );

	// Servo ON
	RtnCen( BOTH_ON ) ;
	WitTim( TNE ) ;

	MesFil( THROUGH ) ;					// Filter setting for measurement
	MeasureStart( ADOFF_FINE_NUM , HALL_RAM_HALL_X_OUT , HALL_RAM_HALL_Y_OUT ) ;					// Start measure
	MeasureWait() ;						// Wait complete of measurement

	RamRead32A( StMeasFunc_MFA_LLiIntegral1 	, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4 , &StMeasValueA.StUllnVal.UlHigVal ) ;
	SlMeasureAveValueA = (INT32)((( (INT64)StMeasValueA.UllnValue * 100 ) / ADOFF_FINE_NUM ) / 100 ) ;

	RamRead32A( StMeasFunc_MFB_LLiIntegral2 	, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;
	SlMeasureAveValueB = (INT32)((( (INT64)StMeasValueB.UllnValue * 100 ) / ADOFF_FINE_NUM ) / 100 ) ;

	UlMinimumValueA = abs(SlMeasureAveValueA) ;
	UlMinimumValueB = abs(SlMeasureAveValueB) ;
	UcFin = 0x11 ;

	while( UcFin ) {
		if( UcFin & 0x01 ) {
			if( UlMinimumValueA >= abs(SlMeasureAveValueA) ) {
				UlMinimumValueA = abs(SlMeasureAveValueA) ;
				UsAdxMin = UsAdxOff ;
				// 収束を早めるために、出力値に比例させる
				if( SlMeasureAveValueA > 0 )
					UsAdxOff = (INT16)UsAdxOff + (SlMeasureAveValueA >> 17) + 1 ;
				else
					UsAdxOff = (INT16)UsAdxOff + (SlMeasureAveValueA >> 17) - 1 ;

				RamWrite32A( HALL_RAM_HXOFF,  (UINT32)((UsAdxOff << 16 ) & 0xFFFF0000 )) ;
			} else {
//TRACE("X fine\n");
				UcFin &= 0xFE ;
			}
		}

		if( UcFin & 0x10 ) {
			if( UlMinimumValueB >= abs(SlMeasureAveValueB) ) {
				UlMinimumValueB = abs(SlMeasureAveValueB) ;
				UsAdyMin = UsAdyOff ;
				// 収束を早めるために、出力値に比例させる
				if( SlMeasureAveValueB > 0 )
					UsAdyOff = (INT16)UsAdyOff + (SlMeasureAveValueB >> 17) + 1 ;
				else
					UsAdyOff = (INT16)UsAdyOff + (SlMeasureAveValueB >> 17) - 1 ;

				RamWrite32A( HALL_RAM_HYOFF,  (UINT32)((UsAdyOff << 16 ) & 0xFFFF0000 )) ;
			} else {
//TRACE("Y fine\n");
				UcFin &= 0xEF ;
			}
		}
		
		MeasureStart( ADOFF_FINE_NUM , HALL_RAM_HALL_X_OUT , HALL_RAM_HALL_Y_OUT ) ;					// Start measure
		MeasureWait() ;						// Wait complete of measurement

		RamRead32A( StMeasFunc_MFA_LLiIntegral1 	, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
		RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4 , &StMeasValueA.StUllnVal.UlHigVal ) ;
		SlMeasureAveValueA = (INT32)((( (INT64)StMeasValueA.UllnValue * 100 ) / ADOFF_FINE_NUM ) / 100 ) ;

		RamRead32A( StMeasFunc_MFB_LLiIntegral2 	, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;
		SlMeasureAveValueB = (INT32)((( (INT64)StMeasValueB.UllnValue * 100 ) / ADOFF_FINE_NUM ) / 100 ) ;
//TRACE("-->Adx %04X, Ady %04X\n", UsAdxOff, UsAdyOff );
	}	// while
//TRACE("TneFin: After Adx=%04X, Ady=%04X\n", UsAdxMin, UsAdyMin );
	StAdjPar.StHalAdj.UsHlxCna = UsAdxMin;								//Measure Result Store
	StAdjPar.StHalAdj.UsHlxCen = StAdjPar.StHalAdj.UsHlxCna;			//Measure Result Store

	StAdjPar.StHalAdj.UsHlyCna = UsAdyMin;								//Measure Result Store
	StAdjPar.StHalAdj.UsHlyCen = StAdjPar.StHalAdj.UsHlyCna;			//Measure Result Store

	StAdjPar.StHalAdj.UsAdxOff = StAdjPar.StHalAdj.UsHlxCna  ;
	StAdjPar.StHalAdj.UsAdyOff = StAdjPar.StHalAdj.UsHlyCna  ;

	// Servo OFF
	RtnCen( BOTH_OFF ) ;		// Both OFF


TRACE("    XadofFin = %04xh \n", StAdjPar.StHalAdj.UsAdxOff ) ;
TRACE("    YadofFin = %04xh \n", StAdjPar.StHalAdj.UsAdyOff ) ;
	RamWrite32A( HALL_RAM_HXOFF,  (UINT32)((StAdjPar.StHalAdj.UsAdxOff << 16 ) & 0xFFFF0000 )) ;
	RamWrite32A( HALL_RAM_HYOFF,  (UINT32)((StAdjPar.StHalAdj.UsAdyOff << 16 ) & 0xFFFF0000 )) ;

}

/***************************************/
#define		SLT_OFFSET		(0x1000)
#define		LENS_MARGIN		(0x0800)
#define		PIXEL_SIZE		(1.12f)							// pixel size 1.12um
#define		SPEC_RANGE		(120.0f)						// spec need movable range 130um
#define		SPEC_PIXEL		(PIXEL_SIZE / SPEC_RANGE)		// spec need movable range pixel
/***************************************/
//********************************************************************************
// Function Name 	: IniNvc
// Retun Value		: NON
// Argment Value	: direction
// Explanation		: Set each direction sign function
//********************************************************************************
void	IniNvc( INT16 SsX, INT16 SsY )
{
	SsNvcX = SsX ;
	SsNvcY = SsY ;
}

//********************************************************************************
// Function Name 	: TneSltPos
// Retun Value		: NON
// Argment Value	: Position number(1, 2, 3, 4, 5, 6, 7, 0:reset)
// Explanation		: Move measurement position function
//********************************************************************************
void	TneSltPos( UINT8 UcPos )
{
	INT16 SsOff = 0x0000 ;

	UcPos &= 0x07 ;
	
	if ( UcPos ) {
		SsOff = SLT_OFFSET * (UcPos - 4);
	}

//TRACE("X = %04X, Y = %04X \n", SsOff, SsOff );

	RamWrite32A( HALL_RAM_HXOFF1,  (INT32)((SsOff * SsNvcX) << 16) ) ;
	RamWrite32A( HALL_RAM_HYOFF1,  (INT32)((SsOff * SsNvcY) << 16) ) ;

}

//********************************************************************************
// Function Name 	: TneVrtPos
// Retun Value		: NON
// Argment Value	: Position number(1, 2, 3, 4, 5, 6, 7, 0:reset)
// Explanation		: Move measurement position function
//********************************************************************************
void	TneVrtPos( UINT8 UcPos )
{
	INT16 SsOff = 0x0000 ;

	UcPos &= 0x07 ;
	
	if ( UcPos ) {
		SsOff = SLT_OFFSET * (UcPos - 4);
	}

//TRACE("X = %04X, Y = %04X \n", SsOff, SsOff );

	RamWrite32A( HALL_RAM_HXOFF1,  (INT32)0 ) ;
	RamWrite32A( HALL_RAM_HYOFF1,  (INT32)((SsOff * SsNvcY) << 16) ) ;
}

//********************************************************************************
// Function Name 	: TneHrzPos
// Retun Value		: NON
// Argment Value	: Position number(1, 2, 3, 4, 5, 6, 7, 0:reset)
// Explanation		: Move measurement position function
//********************************************************************************
void	TneHrzPos( UINT8 UcPos )
{
	INT16 SsOff = 0x0000 ;

	UcPos &= 0x07 ;
	
	if ( UcPos ) {
		SsOff = SLT_OFFSET * (UcPos - 4);
	}

//TRACE("X = %04X, Y = %04X \n", SsOff, SsOff );

	RamWrite32A( HALL_RAM_HXOFF1,  (INT32)((SsOff * SsNvcX) << 16) ) ;
	RamWrite32A( HALL_RAM_HYOFF1,  (INT32)0 ) ;
}




//********************************************************************************
// Function Name 	: TneADO
// Retun Value		: 0x0000:PASS, 0x0001:X MAX OVER, 0x0002:Y MAX OVER, 0x0003:X MIN OVER, 0x0004:Y MIN OVER, FFFF:Verify error
//					: 0x0100:X MAX RANGE ERROR, 0x0200:Y MAX RANGE ERROR, 0x0300:X MIN RANGE ERROR, 0x0400:Y MIN ERROR
// Argment Value	: 
// Explanation		: calculation margin Function
// History			: First edition 						
//********************************************************************************
UINT16	TneADO( )
{
	UINT16	UsSts = 0 ;
#if 0	
	INT32	iRetVal;
	INT32 limit ;
	INT32 gxgain ;
	INT32 gygain ;
	INT16 gout_x_marginp ;
	INT16 gout_x_marginm ;
	INT16 gout_y_marginp ;
	INT16 gout_y_marginm ;

	INT16 x_max ;
	INT16 x_min ;
	INT16 x_off ;
	INT16 y_max ;
	INT16 y_min ;
	INT16 y_off ;
	INT16 x_max_after ;
	INT16 x_min_after ;
	INT16 y_max_after ;
	INT16 y_min_after ;
	INT16 gout_x ;
	INT16 gout_y ;

	//
	// Flash update procedure
	//
	// Read calibration sector to buffer
	iRetVal = Calibration_VerifyUpdate_PreRead();
	if(iRetVal != 0) return(iRetVal);

	// Read calibration data
	RdHallCalData();

	x_max = (INT16)StAdjPar.StHalAdj.UsHlxMxa ;
	x_min = (INT16)StAdjPar.StHalAdj.UsHlxMna ;
	x_off = (INT16)StAdjPar.StHalAdj.UsAdxOff ;
	y_max = (INT16)StAdjPar.StHalAdj.UsHlyMxa ;
	y_min = (INT16)StAdjPar.StHalAdj.UsHlyMna ;
	y_off = (INT16)StAdjPar.StHalAdj.UsAdyOff ;

	RamRead32A( GF_LimitX_HLIMT,	&limit ) ;
	RamRead32A( StCaliData_SiGyroGain_X,	&gxgain ) ;
	RamRead32A( StCaliData_SiGyroGain_Y,	&gygain ) ;

	x_max_after = (x_max - x_off) ;
	if (x_off < 0)
	{
	    if ((0x7FFF - abs(x_max)) < abs(x_off)) x_max_after = 0x7FFF ;
	}

	x_min_after = (x_min - x_off) ;
	if (x_off > 0)
	{
	    if ((0x7FFF - abs(x_min)) < abs(x_off)) x_min_after = 0x8001 ;
	}

	y_max_after = (y_max - y_off) ;
	if (y_off < 0)
	{
	    if ((0x7FFF - abs(y_max)) < abs(y_off)) y_max_after = 0x7FFF ;
	}

	y_min_after = (y_min - y_off);
	if (y_off > 0)
	{
	    if ((0x7FFF - abs(y_min)) < abs(y_off)) y_min_after = 0x8001 ;
	}

	gout_x = (INT16)((INT32)(((float)gxgain / 0x7FFFFFFF) * limit * 4) >> 16);
	gout_y = (INT16)((INT32)(((float)gygain / 0x7FFFFFFF) * limit * 4) >> 16);

//TRACE( "ADOFF X\t=\t0x%04X\r\n", x_off ) ;
//TRACE( "ADOFF Y\t=\t0x%04X\r\n", y_off ) ;
//TRACE( "MAX GOUT X\t=\t0x%04X\r\n", gout_x ) ;
//TRACE( "MIN GOUT X\t=\t0x%04X\r\n", (gout_x * -1) ) ;
//TRACE( "MAX GOUT Y\t=\t0x%04X\r\n", gout_y) ;
//TRACE( "MIN GOUT Y\t=\t0x%04X\r\n", (gout_y * -1) ) ;

	gout_x_marginp = (INT16)(gout_x + LENS_MARGIN);			// MARGIN X+
	gout_x_marginm = (INT16)((gout_x + LENS_MARGIN) * -1);	// MARGIN X-
	gout_y_marginp = (INT16)(gout_y + LENS_MARGIN);			// MARGIN Y+
	gout_y_marginm = (INT16)((gout_y + LENS_MARGIN) * -1);	// MARGIN Y-

//TRACE( "MAX GOUT with margin X\t=\t0x%04X\r\n", gout_x_marginp ) ;
//TRACE( "MIN GOUT with margin X\t=\t0x%04X\r\n", gout_x_marginm ) ;
//TRACE( "MAX GOUT with margin Y\t=\t0x%04X\r\n", gout_y_marginp ) ;
//TRACE( "MIN GOUT with margin Y\t=\t0x%04X\r\n", gout_y_marginm ) ;

//TRACE( "MAX AFTER X\t=\t0x%04X\r\n", x_max_after ) ;
//TRACE( "MIN AFTER X\t=\t0x%04X\r\n", x_min_after ) ;
//TRACE( "MAX AFTER Y\t=\t0x%04X\r\n", y_max_after ) ;
//TRACE( "MIN AFTER Y\t=\t0x%04X\r\n", y_min_after ) ;

	// マージンがまったくないものは不良とする
	if (x_max_after < gout_x) {
		UsSts = 1 ;
	}
	else if (y_max_after < gout_y) {
		UsSts = 2 ;
	}
	else if (x_min_after > (gout_x * -1)) {
		UsSts = 3 ;
	}
	else if (y_min_after > (gout_y * -1)) {
		UsSts = 4 ;
	}
	else {
		// マージンオーバーであれば、ADOFFSETを更新する
		if (x_max_after < gout_x_marginp) {
			x_off -= (gout_x_marginp - x_max_after);
//TRACE( "UPDATE ADOFF X\t=\t0x%04X\r\n", x_off ) ;
		}
		if (x_min_after > gout_x_marginm) {
			x_off += abs(x_min_after - gout_x_marginm);
//TRACE( "UPDATE ADOFF X\t=\t0x%04X\r\n", x_off ) ;
		}
		if (y_max_after < gout_y_marginp) {
			y_off -= (gout_y_marginp - y_max_after);
//TRACE( "UPDATE ADOFF Y\t=\t0x%04X\r\n", y_off ) ;
		}
		if (y_min_after > gout_y_marginm) {
			y_off += abs(y_min_after - gout_y_marginm);
//TRACE( "UPDATE ADOFF X\t=\t0x%04X\r\n", y_off ) ;
		}
		
		if ( (StAdjPar.StHalAdj.UsAdxOff != (UINT16)x_off) || (StAdjPar.StHalAdj.UsAdyOff != (UINT16)y_off) ) {
			StAdjPar.StHalAdj.UsAdxOff = x_off ;
			StAdjPar.StHalAdj.UsAdyOff = y_off ;

			RamWrite32A( StCaliData_SiLensCen_Offset_X ,	(UINT32)(StAdjPar.StHalAdj.UsAdxOff << 16) ) ;
			RamWrite32A( StCaliData_SiLensCen_Offset_Y ,	(UINT32)(StAdjPar.StHalAdj.UsAdyOff << 16) ) ;

			_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsAdxOff << 16),	LENS_CENTER_VALUE_X	) ;
			_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsAdyOff << 16),	LENS_CENTER_VALUE_Y	) ;
			iRetVal = Calibration_VerifyUpdate();
		}
	}

	// *******************************
	// effective range check
	// *******************************
	if (UsSts == 0) {
		UINT16 UsReadVal ;
		float flDistanceX, flDistanceY ;
		float flDistanceAD = SLT_OFFSET * 6 ;

		// effective range check
		_GET_UINT16( UsReadVal,	DISTANCE_X	) ;
		flDistanceX = (float)UsReadVal / 10.0f ;
//TRACE("DISTANCE (X, Y) pixel = (%04X", UsReadVal );

		_GET_UINT16( UsReadVal,	DISTANCE_Y	) ;
		flDistanceY = (float)UsReadVal / 10.0f ;
//TRACE(", %04X)\r\n", UsReadVal );

//TRACE("DISTANCE (X, Y) pixel = (%x, %x)\r\n", (int)(flDistanceX * 10.0), (int)(flDistanceY * 10.0) );
//TRACE("X MAX um = %d\r\n", (int)((x_max_after * (flDistanceX / flDistanceAD)) * PIXEL_SIZE) ) ;
//TRACE("Y MAX um = %d\r\n", (int)((y_max_after * (flDistanceY / flDistanceAD)) * PIXEL_SIZE) ) ;
//TRACE("X MIN um = %d\r\n", (int)((abs(x_min_after) * (flDistanceX / flDistanceAD)) * PIXEL_SIZE) ) ;
//TRACE("Y MIN um = %d\r\n", (int)((abs(y_min_after) * (flDistanceY / flDistanceAD)) * PIXEL_SIZE) ) ;
//TRACE("SPEC PIXEL = %d\r\n", (int)SPEC_PIXEL ) ;

		if ( (x_max_after * (flDistanceX / flDistanceAD)) < SPEC_PIXEL ) {
//TRACE("X MAX < 85um\r\n");
			// error
			UsSts |= 0x0100 ;
		}
		else if ( (y_max_after * (flDistanceY / flDistanceAD)) < SPEC_PIXEL ) {
//TRACE("Y MAX < 85um\r\n");
			// error
			UsSts |= 0x0200 ;
		}
		else if ( (abs(x_min_after) * (flDistanceX / flDistanceAD)) < SPEC_PIXEL ) {
//TRACE("X MIN < 85um\r\n");
			// error
			UsSts |= 0x0300 ;
		}
		else if ( (abs(y_min_after) * (flDistanceY / flDistanceAD)) < SPEC_PIXEL ) {
//TRACE("Y MAX < 85um\r\n");
			// error
			UsSts |= 0x0400 ;
		}
	}
#endif	
	return( UsSts ) ;

}

#if 0
//********************************************************************************
// Function Name 	: FrqDet
// Retun Value		: 0:PASS, 1:OIS X NG, 2:OIS Y NG, 4:CLAF NG
// Argment Value	: NON
// Explanation		: Module Check 
// History			: First edition 						
//********************************************************************************
// Threshold of osciration amplitude
#define ULTHDVAL	0x01000000								// Threshold of the hale value
UINT8 FrqDet( void )
{
	INT32 SlMeasureParameterA , SlMeasureParameterB ;
	INT32 SlMeasureParameterNum ;
	UINT32 UlXasP_P , UlYasP_P ;
	UINT8 UcRtnVal;

	UcRtnVal = 0;

	//Measurement Setup
	MesFil( OSCCHK ) ;													// Set Measure Filter

	SlMeasureParameterNum	=	1000 ;									// 1000times( 50ms )
	SlMeasureParameterA		=	(UINT32)HALL_RAM_HXOUT0 ;		// Set Measure RAM Address
	SlMeasureParameterB		=	(UINT32)HALL_RAM_HYOUT0 ;		// Set Measure RAM Address

	// impulse Set
//	RamWrite32A( HALL_RAM_HXOFF1 , STEP1 ) ;							// X manual 
//	RamWrite32A( HALL_RAM_HYOFF1 , STEP1 ) ;							// Y manual

//	RamWrite32A( HALL_RAM_HXOFF1 , STEP2 ) ;							// X manual 
//	RamWrite32A( HALL_RAM_HYOFF1 , STEP2 ) ;							// Y manual
	WitTim( 300 ) ;

	// Start measure
	MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;	
	MeasureWait() ;														// Wait complete of measurement
	RamRead32A( StMeasFunc_MFA_UiAmp1, &UlXasP_P ) ;					// X Axis Peak to Peak
	RamRead32A( StMeasFunc_MFB_UiAmp2, &UlYasP_P ) ;					// Y Axis Peak to Peak
//TRACE("UlXasP_P = %X\r\n", (unsigned int)UlXasP_P ) ;
//TRACE("UlYasP_P = %X\r\n", (unsigned int)UlYasP_P ) ;

	WitTim( 50 ) ;

	// Osc Value Check X
	if(  UlXasP_P > ULTHDVAL ){
		UcRtnVal = 1;
	}
	// Osc Value Check Y
	if(  UlYasP_P > ULTHDVAL ){
		UcRtnVal |= 2;
	}


	return(UcRtnVal);													// Retun Status value
}
#endif

#ifdef	ZERO_SERVO
//********************************************************************************
// Function Name 	: TneZeroServo
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes Zero Servo Parameter
// History			: First edition
//********************************************************************************
#define 	ZERO_SERVO_NUM		4096				// 4096times
UINT32	TneZeroServo( UINT8 ucposture , float DegreeGap )
{
	UINT32			UlRsltSts;
	INT32			SlMeasureParameterA , SlMeasureParameterB ;
	INT32			SlMeasureParameterNum ;
	UnllnVal		StMeasValueA , StMeasValueB ;
	INT32			SlMeasureAveValueA , SlMeasureAveValueB ;
	double			dTemp;
	UINT8			i;
	UINT32			UlRdVal;
	double			OffsetAngle;
	
	UlRsltSts = EXE_END ;
	if( ucposture < 0x80 ){
		RamRead32A( CMD_RETURN_TO_CENTER , &UlRdVal );
		if( UlRdVal != 0x00000000 ){
			RtnCen( BOTH_OFF ) ;
		}

		RamRead32A( CMD_ZSRV_MODE , &UlRdVal );
		if( ( UlRdVal & 0x00000001) == ZSRV_ENABLE ){
			RamWrite32A( CMD_ZSRV_MODE , ZSRV_DISABLE );
		}
		
		//平均値測定
			MesFil( THROUGH ) ;					// Set Measure Filter

			SlMeasureParameterNum	=	ZERO_SERVO_NUM ;					// Measurement times

		if( ucposture != 0 && ucposture != 2 ){
			UlRsltSts = EXE_ERROR;
TRACE(" Result = %08x\n",(int)UlRsltSts ) ;
			return( UlRsltSts );
		}
			
		for( i=ucposture ; i< (ucposture+2) ; i++ )
		{
			switch( i ){
			case 0:				/* +Y */
				SlMeasureParameterA		=	ZeroServoRAM_X_IN ;			// Set Measure RAM Address
				SlMeasureParameterB		=	GYRO_RAM_GZ_ADIDAT ;		// Set Measure RAM Address
				break;
			case 1:
				SlMeasureParameterA		=	HALL_RAM_HYOUT2 ;			// Set Measure RAM Address
				SlMeasureParameterB		=	ZeroServoRAM_Y_IN ;			// Set Measure RAM Address
				break;
			case 2:				/* +X */
				SlMeasureParameterA		=	ZeroServoRAM_Y_IN ;			// Set Measure RAM Address
				SlMeasureParameterB		=	ZeroServoRAM_Z_IN ;			// Set Measure RAM Address
				break;
			case 3:
				SlMeasureParameterA		=	HALL_RAM_HXOUT2 ;			// Set Measure RAM Address
				SlMeasureParameterB		=	ZeroServoRAM_X_IN ;			// Set Measure RAM Address
				break;
			}

			MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure

			MeasureWait() ;					// Wait complete of measurement

			RamRead32A( StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;
			RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4		, &StMeasValueA.StUllnVal.UlHigVal ) ;
			RamRead32A( StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;
			RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;

			SlMeasureAveValueA = (INT32)( (INT64)StMeasValueA.UllnValue / SlMeasureParameterNum ) ;
			SlMeasureAveValueB = (INT32)( (INT64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;
			
			switch( i ){
			case 0:
				StZeroServoX.SlOffset = SlMeasureAveValueA ;		// i case 0 : ZeroServoRAM_X_IN
				GYROZ_OFFSET = SlMeasureAveValueB ;
				break;
			case 2:
				StZeroServoY.SlOffset = SlMeasureAveValueA ;		// i case 2 : ZeroServoRAM_Y_IN
				StZeroServoZ.SlOffset = SlMeasureAveValueB ;
				break;
			default :
				break;
			}
		}

TRACE("VAL(H,A) pos = \t%08xh\t%08xh\t%d \n",(int)SlMeasureAveValueA, (int)SlMeasureAveValueB, ucposture ) ;
			
			
			switch( ucposture ){
			case 0:		/* +Y */
				if( SlMeasureAveValueB < (int)(POSTURETH_P<<16) ){
					UlRsltSts = EXE_ERROR ;
					StZeroServoX.SlOffset = 0 ;
				}else{
					StZeroServoMesY.SlHallP = SlMeasureAveValueA ;	// i case 1 : HALL_RAM_HYOUT2
					StZeroServoMesY.SlAcclP = SlMeasureAveValueB ;	// i case 1 : ZeroServoRAM_Y_IN
					UlPostureSt |= 0x00000001;
				}
				break;
			case 2:		/* +X */
				if( SlMeasureAveValueB < (int)(POSTURETH_P<<16) ){
					UlRsltSts = EXE_ERROR ;
					StZeroServoY.SlOffset = 0 ;
				}else{
					StZeroServoMesX.SlHallP = SlMeasureAveValueA ;	// i case 3 : HALL_RAM_HXOUT2
					StZeroServoMesX.SlAcclP = SlMeasureAveValueB ;	// i case 3 : ZeroServoRAM_X_IN
					UlPostureSt |= 0x00000004;
				}
				break;
			}
	}else{
		switch(ucposture){
		case 0x80:	/* 計算 */
			
			if( UlPostureSt == 0x05L ){
				// ( Xhp ) / ( Xap)
				StZeroServoMesX.SlAcclP -= StZeroServoX.SlOffset;
				dTemp = (double)2147483647.0  / (double)( StZeroServoMesX.SlAcclP );
				for( StZeroServoX.SlShift = 0 ; StZeroServoX.SlShift <= 5;  StZeroServoX.SlShift++){
					if( fabs(dTemp) <= 1 ){
						break;
					}
					dTemp = dTemp / (double)2 ;
				}
				StZeroServoX.SlGcora = (INT32)(dTemp * (double)2147483647.0);
				StZeroServoX.SlShift = StZeroServoX.SlShift << 8;

				// ( Yhp ) / ( Yap )
				StZeroServoMesY.SlAcclP -= StZeroServoY.SlOffset;
				dTemp = (double)2147483647.0 / (double)( StZeroServoMesY.SlAcclP );
				for( StZeroServoY.SlShift = 0 ; StZeroServoY.SlShift <= 5;  StZeroServoY.SlShift++){
					if( fabs(dTemp) <= 1 ){
						break;
					}
					dTemp = dTemp / 2 ;
				}
				StZeroServoY.SlGcora = (INT32)(dTemp * (double)2147483647.0);
				StZeroServoY.SlShift = StZeroServoY.SlShift << 8;
				
				OffsetAngle = (double)( DegreeGap ) * 3.141592653589793238 / 180.0f ;



				
				StZeroServoX.SlGaina = (INT32)( (float)StZeroServoMesX.SlHallP / cos( OffsetAngle ) ) ;
				StZeroServoY.SlGaina = (INT32)( (float)StZeroServoMesY.SlHallP / cos( OffsetAngle ) ) ;
				
TRACE("   X    ,   Y    ,   angle = %f \n", DegreeGap ) ;
TRACE("%08xh,%08xh(Offset)\n",(int)StZeroServoX.SlOffset, (int)StZeroServoY.SlOffset) ;
TRACE("%08xh,%08xh(Gcora)\n",(int)StZeroServoX.SlGcora, (int)StZeroServoY.SlGcora) ;
TRACE("%08xh,%08xh(Gaina)\n",(int)StZeroServoX.SlGaina, (int)StZeroServoY.SlGaina) ;
TRACE("%08xh,%08xh(Shift)\n",(int)StZeroServoX.SlShift, (int)StZeroServoY.SlShift) ;
TRACE("%08xh,%08xh(AZ , GZ)\n",(int)StZeroServoZ.SlOffset, (int)GYROZ_OFFSET) ;
				RamWrite32A( ZeroServoRAM_X_OFFSET 			, StZeroServoX.SlOffset );
				RamWrite32A( ZeroServoFilterTableX_gcora	, StZeroServoX.SlGcora );
				RamWrite32A( ZeroServoFilterTableX_gaina 	, StZeroServoX.SlGaina );
				RamWrite32A( ZeroServoFilterTableX_shift 	, StZeroServoX.SlShift );

				RamWrite32A( ZeroServoRAM_Y_OFFSET 			, StZeroServoY.SlOffset );
				RamWrite32A( ZeroServoFilterTableY_gcora	, StZeroServoY.SlGcora );
				RamWrite32A( ZeroServoFilterTableY_gaina 	, StZeroServoY.SlGaina );
				RamWrite32A( ZeroServoFilterTableY_shift 	, StZeroServoY.SlShift );

				RamWrite32A( ZeroServoRAM_Z_OFFSET 			, StZeroServoZ.SlOffset );
				RamWrite32A( GYRO_RAM_GZOFFZ 				, GYROZ_OFFSET );
				
			}else{
				UlRsltSts = EXE_ERROR ;
			}
			break;
		case 0xFF:	/* RAM clear */
			UlPostureSt = 0L;
			break;
		}
	}

TRACE(" Result = %08x\n",(int)UlRsltSts ) ;
	return( UlRsltSts );
}

//********************************************************************************
// Function Name 	: ZeroServoLmt
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Zero Servo Limit Parameter
// History			: First edition
//********************************************************************************
UINT8	ZeroServoLmt( UINT8 UCMODE )
{
	UINT32			UlRsltSts;
#if (EP3_ES == 2)
	double			dTemp1;
#else
	double			dTemp1, dTemp2;
	UINT32			UlFilCoef;
	UINT16			Usshift , UsShiftx , UsShifty;
	UINT32			Ulcoef , Ulgainx , Ulgainy;
#endif
	
TRACE(" ZSRV LMT ( %d )%\n" , UCMODE ) ;
	UlRsltSts = EXE_END ;
	if(( UCMODE > 0x64 ) || ( UCMODE < 0x0A )){
TRACE(" 	error \n" ) ;
		return( EXE_ERROR ) ;
	}

#if (EP3_ES == 2)
	dTemp1 = (double)UCMODE /(double)100.0 * (double)2147483647;
	
	RamWrite32A( ZS_LMT_limitx 	, (INT32)dTemp1 );
	RamWrite32A( ZS_LMT_limity 	, (INT32)dTemp1 );
	
#else // ES1
	RamRead32A( ZeroServoFilterTableX_gcora 	, &StZeroServoX.SlGcora );
	RamRead32A( ZeroServoFilterTableX_shift 	, &StZeroServoX.SlShift );
	RamRead32A( ZeroServoFilterTableY_gcora		, &StZeroServoY.SlGcora );
	RamRead32A( ZeroServoFilterTableY_shift 	, &StZeroServoY.SlShift );
	RamRead32A( ZeroServoFilterTableX_coeff1_0 	, &UlFilCoef );
	
TRACE("		before X (gain , shift , coeff) = ( %08x , %08x , %08x )\n",(int)StZeroServoX.SlGcora ,(int)StZeroServoX.SlShift ,(int)UlFilCoef   ) ;
TRACE("		before Y (gain , shift , coeff) = ( %08x , %08x , %08x )\n",(int)StZeroServoY.SlGcora ,(int)StZeroServoY.SlShift ,(int)UlFilCoef   ) ;
	dTemp1 = (double)100.0 / (double)UCMODE;
	dTemp2 = (double)UCMODE / (double)100.0;
	
	for( Usshift = 0 ; Usshift <= 4;  Usshift++){
		if( fabs(dTemp1) <= 1 ){
			break;
		}
		dTemp1 = dTemp1 / (double)2 ;
	}
	
	Ulgainx = (INT32)(dTemp1 * (double)StZeroServoX.SlGcora);
	Ulgainy = (INT32)(dTemp1 * (double)StZeroServoY.SlGcora);
	UsShiftx = (UINT16)StZeroServoX.SlShift + (UINT16)(Usshift<<8);
	UsShifty = (UINT16)StZeroServoY.SlShift + (UINT16)(Usshift<<8);
	Ulcoef = (INT32)(dTemp2 * (double)UlFilCoef);
TRACE("		after  X (gain , shift , coeff) = ( %08x , %08x , %08x )\n",(int)Ulgainx ,(int)UsShiftx ,(int)Ulcoef   ) ;
TRACE("		after  Y (gain , shift , coeff) = ( %08x , %08x , %08x )\n",(int)Ulgainy ,(int)UsShifty ,(int)Ulcoef   ) ;
	
	RamWrite32A( ZeroServoFilterTableX_gcora 	, Ulgainx );
	RamWrite32A( ZeroServoFilterTableX_shift 	, (UINT32)UsShiftx );
	RamWrite32A( ZeroServoFilterTableY_gcora	, Ulgainy );
	RamWrite32A( ZeroServoFilterTableY_shift 	, (UINT32)UsShifty );
	RamWrite32A( ZeroServoFilterTableX_coeff1_0 , Ulcoef );
	RamWrite32A( ZeroServoFilterTableX_coeff1_2 , Ulcoef );
	RamWrite32A( ZeroServoFilterTableY_coeff1_0 , Ulcoef );
	RamWrite32A( ZeroServoFilterTableY_coeff1_2 , Ulcoef );
#endif

TRACE(" Result = %08x\n",(int)UlRsltSts ) ;
	return( UlRsltSts );
}
#endif	//ZERO_SERVO

#ifdef	SEL_SHIFT_COR
//********************************************************************************
// Function Name 	: MemClr
// Retun Value		: void
// Argment Value	: Clear Target Pointer, Clear Byte Number
// Explanation		: Memory Clear Function
// History			: First edition 						
//********************************************************************************
void	MemClr( UINT8	*NcTgtPtr, UINT16	UsClrSiz )
{
	UINT16	UsClrIdx ;

	for ( UsClrIdx = 0 ; UsClrIdx < UsClrSiz ; UsClrIdx++ )
	{
		*NcTgtPtr	= 0 ;
		NcTgtPtr++ ;
	}
}

//********************************************************************************
// Function Name 	: TneAvc
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes the Accel VC offset for All
// History			: First edition
//********************************************************************************
#define 	ACCLOF_NUM		4096				// 4096times
UINT32	TneAvc( UINT8 ucposture )
{
	UINT32			UlRsltSts;
	INT32			SlMeasureParameterA , SlMeasureParameterB ;
	INT32			SlMeasureParameterNum ;
	UnllnVal		StMeasValueA , StMeasValueB ;
	INT32			SlMeasureAveValueA , SlMeasureAveValueB ;
	INT32			SlMeasureRetValueX , SlMeasureRetValueY , SlMeasureRetValueZ;
	UINT8			j , k;
//	UINT8			i , j , k;
//	INT32			mtrx[9] , imtrx[9];
//	float			detA;
	INT32			SlDiff[3] ;

	UlRsltSts = EXE_END ;
	if( ucposture < 0x7f ){
		//平均値測定
		MesFil( THROUGH ) ;					// Set Measure Filter

		SlMeasureParameterNum	=	ACCLOF_NUM ;					// Measurement times
		/******* 1st *******/
		SlMeasureParameterA		=	ZeroServoRAM_X_IN ;			// Set Measure RAM Address
		SlMeasureParameterB		=	ZeroServoRAM_Y_IN ;			// Set Measure RAM Address

		MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure

		MeasureWait() ;					// Wait complete of measurement

		RamRead32A( StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;
		RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4		, &StMeasValueA.StUllnVal.UlHigVal ) ;
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;

		SlMeasureAveValueA = (INT32)( (INT64)StMeasValueA.UllnValue / SlMeasureParameterNum ) ;
		SlMeasureAveValueB = (INT32)( (INT64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;

		SlMeasureRetValueX = SlMeasureAveValueA ;
		SlMeasureRetValueY = SlMeasureAveValueB ;

		/******* 2nd *******/
		SlMeasureParameterA		=	ZeroServoRAM_Z_IN ;			// Set Measure RAM Address
		SlMeasureParameterB		=	ZeroServoRAM_Z_IN ;			// Set Measure RAM Address

		MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure

		MeasureWait() ;					// Wait complete of measurement

		RamRead32A( StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;
		RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4		, &StMeasValueA.StUllnVal.UlHigVal ) ;
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;

		SlMeasureAveValueA = (INT32)( (INT64)StMeasValueA.UllnValue / SlMeasureParameterNum ) ;
		SlMeasureAveValueB = (INT32)( (INT64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;

		SlMeasureRetValueZ = SlMeasureAveValueA ;
		


TRACE("VAL(X,Y,Z) pos = \t%08xh\t%08xh\t%08xh\t%d \n", (unsigned int)SlMeasureRetValueX, (unsigned int)SlMeasureRetValueY, (unsigned int)SlMeasureRetValueZ, ucposture ) ;
		if(( SlMeasureRetValueZ < (INT32)(POSTURETH_P<<16)) && (ucposture == 0x10)){
				UlRsltSts = EXE_ERROR ;
TRACE(" POS14 [ERROR] \t%08xh < %08xh\n", (unsigned int)(SlMeasureRetValueZ), (unsigned int)(POSTURETH_P<<16) ) ;
		}else if(( SlMeasureRetValueZ > (INT32)(POSTURETH_M<<16)) && (ucposture == 0x11)){
				UlRsltSts = EXE_ERROR ;
TRACE(" POS14 [ERROR] \t%08xh > %08xh\n", (unsigned int)(SlMeasureRetValueZ), (unsigned int)(POSTURETH_M<<16) ) ;
		}else{
TRACE("DEBUG = \t%08xh\t \n", abs( (INT32)(ACCL_SENS << 16) - abs(SlMeasureRetValueZ) ) ) ;
			if( abs(SlMeasureRetValueX) > ZEROG_MRGN_XY )									UlRsltSts |= EXE_GXADJ ;
			if( abs(SlMeasureRetValueY) > ZEROG_MRGN_XY )									UlRsltSts |= EXE_GYADJ ;
			if( abs( (INT32)(ACCL_SENS << 16) - abs(SlMeasureRetValueZ)) > ZEROG_MRGN_Z )	UlRsltSts |= EXE_GZADJ ;
			if( UlRsltSts == EXE_END ){
//				StPosOff.StPos.Pos[4][0] = SlMeasureRetValueX;
//				StPosOff.StPos.Pos[4][1] = SlMeasureRetValueY;
//				StPosOff.StPos.Pos[4][2] = SlMeasureRetValueZ;
				StPosOff.UlAclOfSt |= 0x0000003F;
TRACE("POS14(X,Y,Z) st = \t%08xh\t%08xh\t%08xh\t%08xh \n", (unsigned int)StPosOff.StPos.Pos[4][0], (unsigned int)StPosOff.StPos.Pos[4][1], (unsigned int)StPosOff.StPos.Pos[4][2], (unsigned int)StPosOff.UlAclOfSt ) ;

				SlDiff[0] = SlMeasureRetValueX - (INT32)0;
				SlDiff[1] = SlMeasureRetValueY - (INT32)0;
				if(ucposture == 0x10){
					SlDiff[2] = SlMeasureRetValueZ - (INT32)(ACCL_SENS << 16);
				}else{
					SlDiff[2] = SlMeasureRetValueZ - (INT32)(ACCL_SENS_M << 16);
				}
				StPosOff.StPos.Pos[4][0] = SlDiff[0];
				StPosOff.StPos.Pos[4][1] = SlDiff[1];
				StPosOff.StPos.Pos[4][2] = SlDiff[2];
			}
		}
	}else{
		switch(ucposture){
		case 0x80:	/* 計算 */

			if(StPosOff.UlAclOfSt == 0x3fL ){
				/*X offset*/
				StAclVal.StAccel.SlOffsetX = StPosOff.StPos.Pos[4][0] ;
				/*Y offset*/
				StAclVal.StAccel.SlOffsetY = StPosOff.StPos.Pos[4][1] ;
				/*Z offset*/
				StAclVal.StAccel.SlOffsetZ = StPosOff.StPos.Pos[4][2] ;
#ifdef DEBUG
TRACE("ACLOFST(X,Y,Z) = \t%08xh\t%08xh\t%08xh \n", (unsigned int)StAclVal.StAccel.SlOffsetX, (unsigned int)StAclVal.StAccel.SlOffsetY, (unsigned int)StAclVal.StAccel.SlOffsetZ   ) ;
#endif //DEBUG
				
				RamWrite32A( ZeroServoRAM_X_OFFSET , StAclVal.StAccel.SlOffsetX ) ;	// X axis Accel offset
				RamWrite32A( ZeroServoRAM_Y_OFFSET , StAclVal.StAccel.SlOffsetY ) ;	// Y axis Accel offset
				RamWrite32A( ZeroServoRAM_Z_OFFSET , StAclVal.StAccel.SlOffsetZ ) ;	// Z axis Accel offset
				
				StAclVal.StAccel.SlOffsetX = ( StAclVal.StAccel.SlOffsetX >> 16 ) & 0x0000FFFF;
				StAclVal.StAccel.SlOffsetY = ( StAclVal.StAccel.SlOffsetY >> 16 ) & 0x0000FFFF;
				StAclVal.StAccel.SlOffsetZ = ( StAclVal.StAccel.SlOffsetZ >> 16 ) & 0x0000FFFF;
				
				for( j=0 ; j < 6 ; j++ ){
					k = 4 * j;
					RamWrite32A( AcclFilDly_X + k , 0x00000000 ) ;			// X axis Accl LPF Clear
					RamWrite32A( AcclFilDly_Y + k , 0x00000000 ) ;			// Y axis Accl LPF Clear
					RamWrite32A( AcclFilDly_Z + k , 0x00000000 ) ;			// Z axis Accl LPF Clear
				}

			}else{
				UlRsltSts = EXE_ERROR ;
			}
			break;
		case 0xFF:	/* RAM clear */
			MemClr( ( UINT8 * )&StPosOff, sizeof( stPosOff ) ) ;	// Adjust Parameter Clear
			MemClr( ( UINT8 * )&StAclVal, sizeof( stAclVal ) ) ;	// Adjust Parameter Clear
//			StPosOff.UlAclOfSt = 0L;
			break;
		}
	}

TRACE(" Result = %08x\n", (unsigned int)UlRsltSts ) ;
	return( UlRsltSts );


}
#endif	//SEL_SHIFT_COR



//********************************************************************************
// Function Name 	: Actuator_Moving
// Retun Value		: Status
// Argment Value	: X or Y axis, Actuator Moving Parameter Pointer, Hall Data Pointer
// Explanation		: Actuator Moving Test Function
// History			: First edition 		
//********************************************************************************
UINT8	Actuator_Moving( UINT8 uc_axis, Act_Mov_t *pt_parameter, int *ul_readval )
{
	UINT8		uc_status	= SUCCESS ;
	UnllnVal	StMeasValueA, StMeasValueB ;
	int			targetcode ;

	// Initialize
	uc_status	= RtnCen( BOTH_ON ) ;															// X,Y Servo On
	
	if( uc_status != SUCCESS ) {
		return( uc_status ) ;
	}
	
	MesFil( NOISE ) ;																			// Measure Filter Setting

	targetcode	= pt_parameter->startcode ;														// Set Initial Position

	while( targetcode > pt_parameter->endcode ) {
		// Move to measuring porision
		if( !uc_axis ) {																		// X Axis
			RamWrite32A( HALL_RAM_GYROX_OUT, targetcode ) ;
		} else {																				// Y Axis
			RamWrite32A( HALL_RAM_GYROY_OUT, targetcode ) ;
		}
		
		// Measure position
		WitTim( 100 ) ;																			// Wait 100ms
		
		MeasureStart( 1024, HALL_RAM_HXIDAT, HALL_RAM_HYIDAT ) ;								// Measurement Starting
		
		MeasureWait() ;																			// Wait for finishing measure

		RamRead32A( StMeasFunc_MFA_LLiIntegral1, &StMeasValueA.StUllnVal.UlLowVal ) ;			// X axis
		RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4, &StMeasValueA.StUllnVal.UlHigVal ) ;
		RamRead32A( StMeasFunc_MFB_LLiIntegral2, &StMeasValueB.StUllnVal.UlLowVal ) ;			// Y axis
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4, &StMeasValueB.StUllnVal.UlHigVal ) ;

		if( !uc_axis ) {																		// X Axis
			*ul_readval	= ( INT32 )( ( INT64 )StMeasValueA.UllnValue / 1024 ) ;
		} else {																				// Y Axis
			*ul_readval	= ( INT32 )( ( INT64 )StMeasValueB.UllnValue / 1024 ) ;
		}
		
//		if( ( *( ul_readval - 1 ) - *ul_readval ) < abs( pt_parameter->step / 2 ) ) {
//			uc_status	= FAILURE ;
//		}
		
		targetcode	+= pt_parameter->step ;
		ul_readval++ ;
	}
	
	// Move to center position
	RamWrite32A( HALL_RAM_GYROX_OUT, 0 ) ;
	RamWrite32A( HALL_RAM_GYROY_OUT, 0 ) ;
	return( uc_status ) ;
}


//+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
//+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
//+=  ここから追加 by K.Otake                                                                     +=
//+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
//+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
//********************************************************************************
// Function Name 	: Laser_LinearityCorrection
// Retun Value		: Status
// Argment Value	: None
// Explanation		: Linearity correction by laser displacement meter Function
// History			: First edition 		
//********************************************************************************
#define	RATED_STROKE_RANGE	80L
UINT8	Laser_LinearityCorrection( void )
{
	UINT16	us_peak_to_peak_x, us_rated_peak_to_peak_x, us_peak_to_peak_y, us_rated_peak_to_peak_y ;
	INT32	sl_step_x, sl_position_x[ 7 ], sl_step_y, sl_position_y[ 7 ] ;
	INT16	ss_laser_x[ 7 ], ss_laser_y[ 7 ] ;
	UINT8	i, us_status	= SUCCESS ;
	INT32	sl_gyro_switch, sl_calib_status ;
	
	// Initialize
	if( ( short )StAdjPar.StHalAdj.UsHlxMxa < ( short )StAdjPar.StHalAdj.UsHlxMna ) {
		us_status	= FAILURE ;
		return( us_status ) ;
	}
	
	if( ( short )StAdjPar.StHalAdj.UsHlyMxa < ( short )StAdjPar.StHalAdj.UsHlyMna ) {
		us_status	= FAILURE ;
		return( us_status ) ;
	}
	
	// Linearity correction disable
	RamRead32A( GYRO_RAM_GYRO_Switch, &sl_gyro_switch ) ;
	sl_gyro_switch	&= 0xFFFFFFF7 ;
	RamWrite32A( GYRO_RAM_GYRO_Switch, sl_gyro_switch ) ;
	
	// Servo ON
	RamWrite32A( CMD_RETURN_TO_CENTER, BOTH_SRV_ON ) ;
	WitTim( 100 ) ;

	// Calculate the moving position
	us_peak_to_peak_x		= ( unsigned short )( ( long )StAdjPar.StHalAdj.UsHlxMxa - ( long )StAdjPar.StHalAdj.UsHlxMna ) ;
	us_rated_peak_to_peak_x	= ( unsigned short )( ( long )us_peak_to_peak_x * RATED_STROKE_RANGE / 100L ) ;
	sl_step_x				= us_rated_peak_to_peak_x / 6 ;

	us_peak_to_peak_y		= ( unsigned short )( ( long )StAdjPar.StHalAdj.UsHlyMxa - ( long )StAdjPar.StHalAdj.UsHlyMna ) ;
	us_rated_peak_to_peak_y	= ( unsigned short )( ( long )us_peak_to_peak_y * RATED_STROKE_RANGE / 100L ) ;
	sl_step_y				= us_rated_peak_to_peak_y / 6 ;
	for( i = 0 ; i < 7 ; i++ ) {
		sl_position_x[ i ]	= ( long )( 0 + ( sl_step_x * ( i - 3 ) ) ) << 16 ;
		sl_position_y[ i ]	= ( long )( 0 + ( sl_step_y * ( i - 3 ) ) ) << 16 ;
	}
	
	// Move the position x to initial position
	RamWrite32A( HALL_RAM_GYROX_OUT, ( unsigned long )sl_position_x[ 0 ] ) ;
	WitTim( 100 ) ;
	
	// Please set laser displacement meter to 0um
	ClearLaser() ;
	
	// Please take laser displacement data while the actuator move
	// Please take X axis laser data
	for( i = 0 ; i < 7 ; i++ ) {
		RamWrite32A( HALL_RAM_GYROX_OUT, ( unsigned long )sl_position_x[ i ] ) ;
		WitTim( 100 ) ;
		ss_laser_x[ i ]	= GetLaser() ;
	}
	
	RamWrite32A( HALL_RAM_GYROX_OUT, 0 ) ;										// Return to center position
	WitTim( 100 ) ;
	
	// Move the position y to initial position
	RamWrite32A( HALL_RAM_GYROY_OUT, ( unsigned long )sl_position_y[ 0 ] ) ;
	WitTim( 100 ) ;
	
	// Please set laser displacement meter to 0um
	ClearLaser() ;
	
	// Please take Y axis laser data
	for( i = 0 ; i < 7 ; i++ ) {
		RamWrite32A( HALL_RAM_GYROY_OUT, ( unsigned long )sl_position_y[ i ] ) ;
		WitTim( 100 ) ;
		ss_laser_y[ i ]	= GetLaser() ;
	}
	
	RamWrite32A( HALL_RAM_GYROY_OUT, 0 ) ;										// Return to center position
	
#if 0
	// Test
	ss_laser_x[ 0 ]	= 0 ;
	ss_laser_x[ 1 ]	= 60 ;
	ss_laser_x[ 2 ]	= 123 ;
	ss_laser_x[ 3 ]	= 191 ;
	ss_laser_x[ 4 ]	= 262 ;
	ss_laser_x[ 5 ]	= 341 ;
	ss_laser_x[ 6 ]	= 427 ;
	sl_step_x		= 4003 ;
	
	ss_laser_y[ 0 ]	= 0 ;
	ss_laser_y[ 1 ]	= 51 ;
	ss_laser_y[ 2 ]	= 91 ;
	ss_laser_y[ 3 ]	= 146 ;
	ss_laser_y[ 4 ]	= 201 ;
	ss_laser_y[ 5 ]	= 267 ;
	ss_laser_y[ 6 ]	= 330 ;
	sl_step_y		= 3482 ;
#endif

	// Send laser displacement data to DSP
	RamWrite32A( StPosition_0, ( unsigned long )( ( long )( ( ss_laser_y[ 0 ] * 10 ) << 16 ) | ( ss_laser_x[ 0 ] * 10 ) ) ) ;
	RamWrite32A( StPosition_1, ( unsigned long )( ( long )( ( ss_laser_y[ 1 ] * 10 ) << 16 ) | ( ss_laser_x[ 1 ] * 10 ) ) ) ;
	RamWrite32A( StPosition_2, ( unsigned long )( ( long )( ( ss_laser_y[ 2 ] * 10 ) << 16 ) | ( ss_laser_x[ 2 ] * 10 ) ) ) ;
	RamWrite32A( StPosition_3, ( unsigned long )( ( long )( ( ss_laser_y[ 3 ] * 10 ) << 16 ) | ( ss_laser_x[ 3 ] * 10 ) ) ) ;
	RamWrite32A( StPosition_4, ( unsigned long )( ( long )( ( ss_laser_y[ 4 ] * 10 ) << 16 ) | ( ss_laser_x[ 4 ] * 10 ) ) ) ;
	RamWrite32A( StPosition_5, ( unsigned long )( ( long )( ( ss_laser_y[ 5 ] * 10 ) << 16 ) | ( ss_laser_x[ 5 ] * 10 ) ) ) ;
	RamWrite32A( StPosition_6, ( unsigned long )( ( long )( ( ss_laser_y[ 6 ] * 10 ) << 16 ) | ( ss_laser_x[ 6 ] * 10 ) ) ) ;
	RamWrite32A( SiStepXY, ( unsigned long )( sl_step_y << 16 | sl_step_x ) ) ;
	
	// Linearity correction enable
	RamRead32A( GYRO_RAM_GYRO_Switch, &sl_gyro_switch ) ;
	sl_gyro_switch	|= 0x00000008 ;
	RamWrite32A( GYRO_RAM_GYRO_Switch, sl_gyro_switch ) ;

	// Linearity Data enable
	RamRead32A( StCaliData_UsCalibrationStatus, &sl_calib_status ) ;
	sl_calib_status	&= 0xFFFFFBFF ;
	RamWrite32A( StCaliData_UsCalibrationStatus, sl_calib_status ) ;
	
	// Calculate the coefficient for linearity correction
	RamWrite32A( CMD_LASER_LINEAR_DATA, 0 ) ;
	
	return( us_status ) ;
}



//********************************************************************************
// Function Name 	: CircleTest
// Retun Value		: Status
// Argment Value	: Frequency, XX% Mechanical Stroke
// Explanation		: Sine Wave Test Function
// History			: First edition 		
//********************************************************************************
#define	TRACKING_THRESHOLD	0x08000000
UINT8	CircleTest( UINT8 uc_freq, UINT8 us_amp )
{
	UINT16	us_amplitude_x, us_amplitude_y ;
	UINT32	ul_gain_x, ul_gain_y ;
	UINT8	uc_status = SUCCESS ;
	INT32	sl_sample_num, sl_ach_max, sl_ach_min, sl_bch_max, sl_bch_min ;

	// Initialize
	uc_status	= RtnCen( BOTH_ON ) ;
	
	if( uc_status != SUCCESS ) {
		return( uc_status ) ;
	}
	
	OisDis() ;

	// Sine wave amplitude setting
	SetSinWavGenInt() ;
	
	if( StAdjPar.StHalAdj.UsHlxMxa > 32767 ) {
		sl_ach_max	= ( INT32 )0xFFFF0000 | StAdjPar.StHalAdj.UsHlxMxa ;
	} else {
		sl_ach_max	= ( INT32 )StAdjPar.StHalAdj.UsHlxMxa ;
	}
	
	if( StAdjPar.StHalAdj.UsHlxMna > 32767 ) {
		sl_ach_min	= ( INT32 )0xFFFF0000 | StAdjPar.StHalAdj.UsHlxMna ;
	} else {
		sl_ach_min	= ( INT32 )StAdjPar.StHalAdj.UsHlxMna ;
	}
	
	us_amplitude_x	= ( UINT16 )( ( sl_ach_max - sl_ach_min ) / 2 ) ;

	if( StAdjPar.StHalAdj.UsHlyMxa > 32767 ) {
		sl_ach_max	= ( INT32 )0xFFFF0000 | StAdjPar.StHalAdj.UsHlyMxa ;
	} else {
		sl_ach_max	= ( INT32 )StAdjPar.StHalAdj.UsHlyMxa ;
	}
	
	if( StAdjPar.StHalAdj.UsHlyMna > 32767 ) {
		sl_ach_min	= ( INT32 )0xFFFF0000 | StAdjPar.StHalAdj.UsHlyMna ;
	} else {
		sl_ach_min	= ( INT32 )StAdjPar.StHalAdj.UsHlyMna ;
	}
	
	us_amplitude_y	= ( UINT16 )( ( sl_ach_max - sl_ach_min ) / 2 ) ;
	
	ul_gain_x	= ( UINT32 )( ( UINT32 )us_amplitude_x * us_amp / 100 ) << 16 ;
	ul_gain_y	= ( UINT32 )( ( UINT32 )us_amplitude_y * us_amp / 100 ) << 16 ;

	// Sine wave generation
	SetSinWavePara( uc_freq, CIRCWAVE ) ;
	
	RamWrite32A( SinWave_Gain, ul_gain_x ) ;									// Sine wave gain
	RamWrite32A( CosWave_Gain, ul_gain_y ) ;									// Cosine wave gain
	
	// Measurement initialize
	MesFil( NOISE ) ;
	
	sl_sample_num	= ( INT32 )( FS_FREQ / ( float )uc_freq ) ;
	
	// Start peak to peak measurement
	MeasureStart( sl_sample_num, HALL_RAM_HXOUT0, HALL_RAM_HYOUT0 ) ;
	
	MeasureWait() ;
	
	// Check tracking
	RamRead32A( StMeasFunc_MFA_SiMax1, &sl_ach_max ) ;
	RamRead32A( StMeasFunc_MFA_SiMin1, &sl_ach_min ) ;
	RamRead32A( StMeasFunc_MFB_SiMax2, &sl_bch_max ) ;
	RamRead32A( StMeasFunc_MFB_SiMin2, &sl_bch_min ) ;
	
	if( ( ( sl_ach_max - sl_ach_min ) > TRACKING_THRESHOLD ) || ( ( sl_bch_max - sl_bch_min ) > TRACKING_THRESHOLD ) ) {
		uc_status	= FAILURE ;
	}
	
	// Step sine wave
	SetSinWavePara( 0, CIRCWAVE ) ;
	
	return( uc_status ) ;
}



//********************************************************************************
// Function Name 	: OscillationDetection
// Retun Value		: Status
// Argment Value	: XX% X axis Mechanical Stroke, XX% Y axis Mechanical Stroke
// Explanation		: Sine Wave Test Function
// History			: First edition 		
//********************************************************************************
#define	OSCILLATION_THRESHOLD	0x01000000
UINT8	OscillationDetection( INT8 sc_offset_x, INT8 sc_offset_y, UINT16 us_wait_time )
{
	UINT16	us_amplitude_x, us_amplitude_y ;
	INT32	sl_offset_x, sl_offset_y ;
	UINT8	uc_status = SUCCESS ;
	INT32	sl_ach_max, sl_ach_min, sl_bch_max, sl_bch_min ;

	// Initialize
	uc_status	= RtnCen( BOTH_ON ) ;
	
	if( uc_status != SUCCESS ) {
		return( uc_status ) ;
	}
	
	OisDis() ;

	// Add Offset
	if( StAdjPar.StHalAdj.UsHlxMxa > 32767 ) {
		sl_ach_max	= ( INT32 )0xFFFF0000 | StAdjPar.StHalAdj.UsHlxMxa ;
	} else {
		sl_ach_max	= ( INT32 )StAdjPar.StHalAdj.UsHlxMxa ;
	}
	
	if( StAdjPar.StHalAdj.UsHlxMna > 32767 ) {
		sl_ach_min	= ( INT32 )0xFFFF0000 | StAdjPar.StHalAdj.UsHlxMna ;
	} else {
		sl_ach_min	= ( INT32 )StAdjPar.StHalAdj.UsHlxMna ;
	}
	
	us_amplitude_x	= ( UINT16 )( ( sl_ach_max - sl_ach_min ) / 2 ) ;

	if( StAdjPar.StHalAdj.UsHlyMxa > 32767 ) {
		sl_ach_max	= ( INT32 )0xFFFF0000 | StAdjPar.StHalAdj.UsHlyMxa ;
	} else {
		sl_ach_max	= ( INT32 )StAdjPar.StHalAdj.UsHlyMxa ;
	}
	
	if( StAdjPar.StHalAdj.UsHlyMna > 32767 ) {
		sl_ach_min	= ( INT32 )0xFFFF0000 | StAdjPar.StHalAdj.UsHlyMna ;
	} else {
		sl_ach_min	= ( INT32 )StAdjPar.StHalAdj.UsHlyMna ;
	}
	
	us_amplitude_y	= ( UINT16 )( ( sl_ach_max - sl_ach_min ) / 2 ) ;
	
	sl_offset_x	= ( UINT32 )( ( UINT32 )us_amplitude_x * sc_offset_x / 100 ) << 16 ;
	sl_offset_y	= ( UINT32 )( ( UINT32 )us_amplitude_y * sc_offset_y / 100 ) << 16 ;
	TRACE( "Offset X = %x\n", sl_offset_x ) ;
	TRACE( "Offset Y = %x\n", sl_offset_y ) ;

	RamWrite32A( HALL_RAM_GYROX_OUT, ( UINT32 )sl_offset_x ) ;
	RamWrite32A( HALL_RAM_GYROY_OUT, ( UINT32 )sl_offset_y ) ;

	WitTim( 100 ) ;

	RamWrite32A( HALL_RAM_GYROX_OUT, 0 ) ;
	RamWrite32A( HALL_RAM_GYROY_OUT, 0 ) ;

	WitTim( us_wait_time ) ;

	// Measurement initialize
	MesFil( NOISE ) ;
	
	// Start peak to peak measurement
	MeasureStart( 1024, HALL_RAM_HXIDAT, HALL_RAM_HYIDAT ) ;
	
	MeasureWait() ;
	
	// Check Oscillation
	RamRead32A( StMeasFunc_MFA_SiMax1, &sl_ach_max ) ;
	RamRead32A( StMeasFunc_MFA_SiMin1, &sl_ach_min ) ;
	RamRead32A( StMeasFunc_MFB_SiMax2, &sl_bch_max ) ;
	RamRead32A( StMeasFunc_MFB_SiMin2, &sl_bch_min ) ;
	TRACE( "sl_ach_max = %x\n", sl_ach_max ) ;
	TRACE( "sl_ach_min = %x\n", sl_ach_min ) ;
	TRACE( "sl_bch_max = %x\n", sl_bch_max ) ;
	TRACE( "sl_bch_min = %x\n", sl_bch_min ) ;
	
	if( ( ( sl_ach_max - sl_ach_min ) > OSCILLATION_THRESHOLD ) || ( ( sl_bch_max - sl_bch_min ) > OSCILLATION_THRESHOLD ) ) {
		uc_status	= FAILURE ;
	}
	
	return( uc_status ) ;
}



//********************************************************************************
// Function Name 	: OIS_PosMes_by_AF
// Retun Value		: Status
// Argment Value	: None
// Explanation		: OIS Position Measurement by AF Function
// History			: First edition 		
//********************************************************************************
#define	OIS_POSITION_THRESHOLD	0x15000000
UINT16	Us_Af_Position[ 9 ]	= { 0x0000, 0x0080, 0x0100, 0x0180, 0x0200, 0x0280, 0x0300, 0x0380, 0x03FF } ;
UINT32	UlBufDat[ 64 ] ;
INT32	Sl_Hall_X_by_AF[ 9 ] ;
INT32	Sl_Hall_Y_by_AF[ 9 ] ;
INT32	Sl_Hall_X_Slope_by_AF[ 8 ] ;
INT32	Sl_Hall_Y_Slope_by_AF[ 8 ] ;

UINT8	OIS_PosMes_by_AF( void )
{
	UINT8		uc_status, i ;
	INT32		sl_hall_x[ 9 ], sl_hall_y[ 9 ], sl_default_x, sl_default_y ;
	UnllnVal	StMeasValueA, StMeasValueB ;

	// Initialize
	uc_status	= RtnCen( BOTH_OFF ) ;														// X,Y Servo Off
	
	if( uc_status != SUCCESS ) {
		return( uc_status ) ;
	}
	
	OisDis() ;																				// OIS disable

	MesFil( NOISE ) ;																		// Measure filter setting

	// Measure default position
	WitTim( 100 ) ;																		// Wait 100ms
	
	MeasureStart( 1024, HALL_RAM_HXIDAT, HALL_RAM_HYIDAT ) ;							// Measurement Starting
	
	MeasureWait() ;																		// Wait for finishing measure

	RamRead32A( StMeasFunc_MFA_LLiIntegral1, &StMeasValueA.StUllnVal.UlLowVal ) ;		// X axis
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4, &StMeasValueA.StUllnVal.UlHigVal ) ;
	RamRead32A( StMeasFunc_MFB_LLiIntegral2, &StMeasValueB.StUllnVal.UlLowVal ) ;		// Y axis
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4, &StMeasValueB.StUllnVal.UlHigVal ) ;

	sl_default_x	= ( INT32 )( ( INT64 )StMeasValueA.UllnValue / 1024 ) ;
	sl_default_y	= ( INT32 )( ( INT64 )StMeasValueB.UllnValue / 1024 ) ;
	
	// OIS position measurement while moving AF
	for( i = 0 ; i < 9 ; i++ ) {
		// Please insert AF moving code here

		WitTim( 100 ) ;																		// Wait 100ms
		
		MeasureStart( 1024, HALL_RAM_HXIDAT, HALL_RAM_HYIDAT ) ;							// Measurement Starting
		
		MeasureWait() ;																		// Wait for finishing measure

		RamRead32A( StMeasFunc_MFA_LLiIntegral1, &StMeasValueA.StUllnVal.UlLowVal ) ;		// X axis
		RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4, &StMeasValueA.StUllnVal.UlHigVal ) ;
		RamRead32A( StMeasFunc_MFB_LLiIntegral2, &StMeasValueB.StUllnVal.UlLowVal ) ;		// Y axis
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4, &StMeasValueB.StUllnVal.UlHigVal ) ;

		sl_hall_x[ i ]	= ( INT32 )( ( INT64 )StMeasValueA.UllnValue / 1024 ) - sl_default_x ;
		sl_hall_y[ i ]	= ( INT32 )( ( INT64 )StMeasValueB.UllnValue / 1024 ) - sl_default_y ;
	}
	
	for( i = 1 ; i < 9 ; i++ ) {
		if( abs( sl_hall_x[ i ] - sl_hall_x[ i - 1 ] ) > OIS_POSITION_THRESHOLD ) {
			return( FAILURE ) ;
		}

		if( abs( sl_hall_y[ i ] - sl_hall_y[ i - 1 ] ) > OIS_POSITION_THRESHOLD ) {
			return( FAILURE ) ;
		}
	}
	
	// Please store the measurement data in external EEPROM
	
	return( uc_status ) ;
}



//********************************************************************************
// Function Name 	: Read_OIS_PosData_by_AF
// Retun Value		: None
// Argment Value	: None
// Explanation		: Read OIS Position Data by AF Function
// History			: First edition 		
//********************************************************************************
void	Read_OIS_PosData_by_AF( void )
{
	UINT8		i ;

	// Please insert the function what reads OIS shift cal data from external EEPROM

//------------------------------------------------------------------------------------------------
// Read Calibration data
//------------------------------------------------------------------------------------------------
	// Here is Inputing the dummy data
	// But, Actually, Please input OIS shift cal data
	Sl_Hall_X_by_AF[ 0 ]	= UlBufDat[ OIS_POS_BY_AF_X1 ] ;
	Sl_Hall_X_by_AF[ 1 ]	= UlBufDat[ OIS_POS_BY_AF_X2 ] ;
	Sl_Hall_X_by_AF[ 2 ]	= UlBufDat[ OIS_POS_BY_AF_X3 ] ;
	Sl_Hall_X_by_AF[ 3 ]	= UlBufDat[ OIS_POS_BY_AF_X4 ] ;
	Sl_Hall_X_by_AF[ 4 ]	= UlBufDat[ OIS_POS_BY_AF_X5 ] ;
	Sl_Hall_X_by_AF[ 5 ]	= UlBufDat[ OIS_POS_BY_AF_X6 ] ;
	Sl_Hall_X_by_AF[ 6 ]	= UlBufDat[ OIS_POS_BY_AF_X7 ] ;
	Sl_Hall_X_by_AF[ 7 ]	= UlBufDat[ OIS_POS_BY_AF_X8 ] ;
	Sl_Hall_X_by_AF[ 8 ]	= UlBufDat[ OIS_POS_BY_AF_X9 ] ;

	Sl_Hall_Y_by_AF[ 0 ]	= UlBufDat[ OIS_POS_BY_AF_Y1 ] ;
	Sl_Hall_Y_by_AF[ 1 ]	= UlBufDat[ OIS_POS_BY_AF_Y2 ] ;
	Sl_Hall_Y_by_AF[ 2 ]	= UlBufDat[ OIS_POS_BY_AF_Y3 ] ;
	Sl_Hall_Y_by_AF[ 3 ]	= UlBufDat[ OIS_POS_BY_AF_Y4 ] ;
	Sl_Hall_Y_by_AF[ 4 ]	= UlBufDat[ OIS_POS_BY_AF_Y5 ] ;
	Sl_Hall_Y_by_AF[ 5 ]	= UlBufDat[ OIS_POS_BY_AF_Y6 ] ;
	Sl_Hall_Y_by_AF[ 6 ]	= UlBufDat[ OIS_POS_BY_AF_Y7 ] ;
	Sl_Hall_Y_by_AF[ 7 ]	= UlBufDat[ OIS_POS_BY_AF_Y8 ] ;
	Sl_Hall_Y_by_AF[ 8 ]	= UlBufDat[ OIS_POS_BY_AF_Y9 ] ;

//------------------------------------------------------------------------------------------------
// Calculate the slope
//------------------------------------------------------------------------------------------------
	for( i = 0 ; i < 8 ; i++ ) {
		Sl_Hall_X_Slope_by_AF[ i ] = ( INT32 )( ( Sl_Hall_X_by_AF[ i + 1 ] - Sl_Hall_X_by_AF[ i ] ) / ( Us_Af_Position[ i + 1 ] - Us_Af_Position[ i ] ) ) ;
		Sl_Hall_Y_Slope_by_AF[ i ] = ( INT32 )( ( Sl_Hall_Y_by_AF[ i + 1 ] - Sl_Hall_Y_by_AF[ i ] ) / ( Us_Af_Position[ i + 1 ] - Us_Af_Position[ i ] ) ) ;
	}

	return ;
}



//********************************************************************************
// Function Name 	: OIS_Pos_Correction_by_AF
// Retun Value		: None
// Argment Value	: AF Code
// Explanation		: OIS Position Correction by AF Function
// History			: First edition 		
//********************************************************************************
void	OIS_Pos_Correction_by_AF( UINT16	us_af_code )
{
	UINT8	i ;
	INT32	sl_offset_x, sl_offset_y ;

	for( i = 1 ; i < 9 ; i++ ) {
		if( us_af_code < Us_Af_Position[ i ] + 1 ) {
			sl_offset_x	= ( INT32 )( Sl_Hall_X_Slope_by_AF[ i - 1 ] * ( us_af_code - Us_Af_Position[ i - 1 ] ) + ( Sl_Hall_X_by_AF[ i - 1 ] - Sl_Hall_X_by_AF[ 0 ] ) ) ;
			sl_offset_y	= ( INT32 )( Sl_Hall_Y_Slope_by_AF[ i - 1 ] * ( us_af_code - Us_Af_Position[ i - 1 ] ) + ( Sl_Hall_Y_by_AF[ i - 1 ] - Sl_Hall_Y_by_AF[ 0 ] ) ) ;
			break ;
		}
	}
	
	RamWrite32A( HALL_RAM_HXOFF1, sl_offset_x ) ;
	RamWrite32A( HALL_RAM_HYOFF1, sl_offset_y ) ;
}


#if ((SELECT_VENDOR & 0x80 ) != 0x80)
//********************************************************************************
// Function Name 	: CalcSetMizxAndLinearityData
// Retun Value		: 0:OK, 1:NG
// Argment Value	: UcMode	0:disable	1:enable
//					: mlLinearityValue *linval
// Explanation		: Flash write linearity correction data function
// History			: First edition
//********************************************************************************
#define N 7  /* データ数 */

UINT8	CalcSetMizxAndLinearityData( mlLinearityValue *linval ,  mlMixingValue *mixval )
{
	int i;
 	double Xa = 0, Ya = 0;
  	double Xsum_xy = 0, Xsum_x = 0, Xsum_y = 0, Xsum_x2 = 0;
  	double Ysum_xy = 0, Ysum_x = 0, Ysum_y = 0, Ysum_x2 = 0;
	UINT8		ans;
	
	// **************************************************
	// 最小2乗法
	// **************************************************
#ifdef __MIX_LIB_METHOD__		// TESTAPP library method
	for (i=0; i<N; i++) {
		Xsum_xy += i * mixval->XonXmove[i];
		Xsum_x  += i;
		Xsum_y  += mixval->XonXmove[i];
		
		Ysum_xy += i * mixval->YonXmove[i];
		Ysum_y  += mixval->YonXmove[i];
	}
 	Xa = ((N * Ysum_xy) - (Xsum_x * Ysum_y)) / ((N * Xsum_xy) - (Xsum_x * Xsum_y));

  	Xsum_xy = Xsum_x = Xsum_y = Xsum_x2 = 0;
  	Ysum_xy = Ysum_x = Ysum_y = Ysum_x2 = 0;

	for (i=0; i<N; i++) {
		Xsum_xy += i * mixval->XonYmove[i];
		Xsum_x  += i;
		Xsum_y  += mixval->XonYmove[i];

		Ysum_xy += i * mixval->YonYmove[i];
		Ysum_y  += mixval->YonYmove[i];
	}
 	Ya = ((N * Xsum_xy) - (Xsum_x * Xsum_y)) / ((N * Ysum_xy) - (Xsum_x * Ysum_y));
#else
	for (i=0; i<N; i++) {
		Xsum_xy += mixval->XonXmove[i] * mixval->YonXmove[i];
		Xsum_x  += mixval->XonXmove[i];
		Xsum_y  += mixval->YonXmove[i];
		Xsum_x2 += pow(mixval->XonXmove[i], 2);

		Ysum_xy += mixval->YonYmove[i] * mixval->XonYmove[i];
		Ysum_x  += mixval->YonYmove[i];
		Ysum_y  += mixval->XonYmove[i];
		Ysum_x2 += pow(mixval->YonYmove[i], 2);
	}
 	Xa = ((N * Xsum_xy) - (Xsum_x * Xsum_y)) / ((N * Xsum_x2) - pow(Xsum_x, 2));
 	Ya = ((N * Ysum_xy) - (Ysum_x * Ysum_y)) / ((N * Ysum_x2) - pow(Ysum_x, 2));
#endif

TRACE("Xa = %f\n", Xa);
TRACE("Ya = %f\n", Ya);

	// **************************************************
	// MIXING係数計算
	// **************************************************
TRACE("degreeX  = %f\n", -atan(Xa)*180/3.14159265358979323846 );
TRACE("degreeY  = %f\n", +atan(Ya)*180/3.14159265358979323846 );
#if (SLT_XY_SWAP ==1)
	mixval->radianX = +atan(Ya);
	mixval->radianY = -atan(Xa);
#else
	mixval->radianX = -atan(Xa);
	mixval->radianY = +atan(Ya);
#endif
//TRACE("radianX  = %f\n", mixval->radianX);
//TRACE("radianY  = %f\n", mixval->radianY);

	mixval->hx45x = +(cos(mixval->radianY) / cos(mixval->radianX - mixval->radianY));
    mixval->hx45y = +(sin(mixval->radianY) / cos(mixval->radianX - mixval->radianY));
//    mixval->hy45y = -(cos(mixval->radianX) / cos(mixval->radianX - mixval->radianY));
//    mixval->hy45x = -(sin(mixval->radianX) / cos(mixval->radianX - mixval->radianY));
    mixval->hy45y = +(cos(mixval->radianX) / cos(mixval->radianX - mixval->radianY));
    mixval->hy45x = -(sin(mixval->radianX) / cos(mixval->radianX - mixval->radianY));

	mixval->hxsx = (unsigned char)abs(mixval->hx45x);                                     // >1ならばシフト数として設定
	mixval->hysx = (unsigned char)abs(mixval->hy45y);                                     // >1ならばシフト数として設定

    mixval->hx45x = mixval->hx45x / pow(2, (double)mixval->hxsx);        // シフトを加味して再計算
    mixval->hy45y = mixval->hy45y / pow(2, (double)mixval->hysx);        // シフトを加味して再計算

TRACE("hx45x  = %f\n", mixval->hx45x);
TRACE("hx45y  = %f\n", mixval->hx45y);
TRACE("hy45y  = %f\n", mixval->hy45y);
TRACE("hy45x  = %f\n", mixval->hy45x);
TRACE("hxsx  = %02X\n", mixval->hxsx);
TRACE("hysx  = %02X\n", mixval->hysx);

	mixval->hx45xL = (long)(mixval->hx45x * 0x7FFFFFFF);
	mixval->hx45yL = (long)(mixval->hx45y * 0x7FFFFFFF);
	mixval->hy45yL = (long)(mixval->hy45y * 0x7FFFFFFF);
	mixval->hy45xL = (long)(mixval->hy45x * 0x7FFFFFFF);

TRACE("hx45xL  = %08X\n", mixval->hx45xL);
TRACE("hx45yL  = %08X\n", mixval->hx45yL);
TRACE("hy45yL  = %08X\n", mixval->hy45yL);
TRACE("hy45xL  = %08X\n", mixval->hy45xL);

	// **************************************************
	// RAMにセットする
	// **************************************************
	RamWrite32A( HF_hx45x, mixval->hx45xL ) ; 
	RamWrite32A( HF_hx45y, mixval->hx45yL ) ; 
	RamWrite32A( HF_hy45y, mixval->hy45yL ) ; 
	RamWrite32A( HF_hy45x, mixval->hy45xL ) ; 
	RamWrite32A( HF_ShiftX, ( (UINT32) mixval->hxsx << 0) | ((UINT32)mixval->hysx << 8) ) ;

	ans	= WrHallLnData( 1, linval ) ;
	
	if( ans ) {
		return( ans ) ;														// CheckSum OK
	}
	
	ans	= WrMixCalData( 1, mixval ) ;
	
	return( ans ) ;															// CheckSum OK
}
#endif

#if USE_BOSCH
//********************************************************************************
// Function Name 	: Read_BMI260_CHIP_ID
// Retun Value		: None
// Argment Value	: None
// Explanation		: Read CHIP_ID from BOSCH BMI260
// History			: First edition
//********************************************************************************
UINT8 Read_BMI260_CHIP_ID( void )
{
	unsigned char ReadData;

	struct bmi2_dev dev = {
	        .dev_id = BMI2_I2C_PRIM_ADDR,
	        .intf = BMI2_I2C_INTERFACE,
	        .read = bmi2_i2c_read,
	        .write = bmi2_i2c_write,
	        .delay_ms = bmi2_delay_milli_sec,
	        .read_write_len = 2,
	        .config_file_ptr = NULL
	};

	bmi2_get_regs( CHIP_ID_260, &ReadData, 1, &dev );

	return ReadData;
}

//********************************************************************************
// Function Name 	: Configuring_BMI260
// Retun Value		: None
// Argment Value	: None
// Explanation		: Initialize BOSCH BMI260
// History			: First edition
//********************************************************************************
UINT8 Configuring_BMI260( void )
{
	unsigned char WriteData;
	unsigned char ReadData;
	char          Count;
	unsigned char Status = FAILURE;									// 0x01=NG  0x00=OK

	struct bmi2_dev dev = {
	        .dev_id = BMI2_I2C_PRIM_ADDR,
	        .intf = BMI2_I2C_INTERFACE,
	        .read = bmi2_i2c_read,
	        .write = bmi2_i2c_write,
	        .delay_ms = bmi2_delay_milli_sec,
	        .read_write_len = 2,
	        .config_file_ptr = NULL
	};

	// PWR_CTRL(0x7D) to set bit ACC_EN and GYR_EN to neable them
	Count = 0;
	do {
		WriteData = 0x0E;
		bmi2_set_regs( PWR_CTRL_260, &WriteData, 1, &dev );			// write to BMI260
		bmi2_get_regs( PWR_CTRL_260, &ReadData, 1, &dev );			// read from BMI260

		if( WriteData == ReadData ) {
			Status = SUCCESS;
			break;
		}
	} while( Count++ < 20 );

	if( Status == FAILURE ) return Status;

	// Please set as necessary.
	// Setting accelerometer interface
	WriteData = 0xA8;										// ACC_CONF(0x40) to set accelerometer interface ODR bandwidth
	bmi2_set_regs( ACC_CONF_260, &WriteData, 1, &dev );
	WriteData = 0x00;										// ACC_RANGE(0x41) to set accelerometer interface range
	bmi2_set_regs( ACC_RANGE_260, &WriteData, 1, &dev );
	WriteData = 0x04;										// GYR_RANGE(0x43) to select OIS RANGE
	bmi2_set_regs( GYR_RANGE_260, &WriteData, 1, &dev );
	WriteData = 0xE9;										// GYR_CONF(0x42)
	bmi2_set_regs( GYR_CONF_260, &WriteData, 1, &dev );
	WriteData = 0x02;										// PWR_CONF(0x7C)
	bmi2_set_regs( PWR_CONF_260, &WriteData, 1, &dev );
	WriteData = 0x10;										// IF_CONF(0x6B)
	bmi2_set_regs( IF_CONF_260, &WriteData, 1, &dev );

	return Status;
}

//********************************************************************************
// Function Name 	: PerformingInitialization_BMI260
// Retun Value		: None
// Argment Value	: None
// Explanation		: Performing Initialization sequence BOSCH BMI260
// History			: First edition
//********************************************************************************
UINT8 PerformingInitialization_BMI260( void )
{
	int8_t rslt;

	struct bmi2_dev dev = {
	        .dev_id = BMI2_I2C_PRIM_ADDR,
	        .intf = BMI2_I2C_INTERFACE,
	        .read = bmi2_i2c_read,
	        .write = bmi2_i2c_write,
	        .delay_ms = bmi2_delay_milli_sec,
//	        .read_write_len = 2,
	        .read_write_len = 60,
	        .config_file_ptr = NULL
	};

	rslt = bmi2_init(&dev);
	if (rslt != BMI2_OK) {								// BMI2_OK(0x00)
		rslt = FAILURE;									// 0x01=NG  0x00=OK
	}

	return rslt;
}

//********************************************************************************
// Function Name 	: Read_InternalStatus_BMI260
// Retun Value		: None
// Argment Value	: None
// Explanation		: Checking the correct initialization status BOSCH BMI260
// History			: First edition
//********************************************************************************
UINT8 Read_InternalStatus_BMI260( void )
{
	unsigned char ReadData;

	struct bmi2_dev dev = {
	        .dev_id = BMI2_I2C_PRIM_ADDR,
	        .intf = BMI2_I2C_INTERFACE,
	        .read = bmi2_i2c_read,
	        .write = bmi2_i2c_write,
	        .delay_ms = bmi2_delay_milli_sec,
	        .read_write_len = 2,
	        .config_file_ptr = NULL
	};

	bmi2_get_regs( INTERNAL_STATUS_260, &ReadData, 1, &dev );

	return ReadData;
}
#endif


//+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
//+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
//+=  ここまで by K.Otake                                                                         +=
//+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
//+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=

