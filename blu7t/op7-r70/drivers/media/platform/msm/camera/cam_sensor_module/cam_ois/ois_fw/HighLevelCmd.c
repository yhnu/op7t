//********************************************************************************
//		<< LC898124 Evaluation Soft >>
//	    Program Name	: OisCmd.c
//		Design			: Y.Shigoeka
//		History			: First edition						
//********************************************************************************
//**************************
//	Include Header File		
//**************************
#include	"Ois.h"
#include	<stdlib.h>
#include	<math.h>

//****************************************************
//	CUSTOMER NECESSARY CREATING LIST
//****************************************************
/* for I2C communication */ 
extern	void RamWrite32A(int addr, int data);
extern 	void RamRead32A( unsigned short addr, void * data );
extern void	WitTim( unsigned short	UsWitTim );
extern UINT8 GetInfomationAfterDownload( DSPVER* Info );
extern void DMIOWrite32( UINT32 IOadrs, UINT32 IOdata );
extern void BurstReadE2Prom( UINT8 startaddress, UINT8* val, UINT8 cnt );
extern void ReadE2Prom( unsigned char address, unsigned char * val );
//**************************
//	STRICT
//**************************
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
//	Local Function LIST
//**************************
void	MesFil( UINT8 ) ;									// Measure Filter Setting
void	ClrMesFil( void );
void	MeasureStart( INT32 , UINT32 , UINT32 ) ;				// Measure Start Function
void	MeasureWait( void ) ;								// Measure Wait
void	MemoryClear( UINT16 , UINT16 ) ;					// Memory Cloear
void	SetWaitTime( UINT16 ) ; 							// Set Wait Timer
void	SetTransDataAdr( UINT16 UsLowAddress , UINT32 UlLowAdrBeforeTrans );

void	OisEnaNCL( void );
void	OisEnaDrCl( void );
void	OisEnaDrNcl( void );
void	OisDis( void );
void	SetRec( void );
void	SetStill( void );

UINT8	MesRam( INT32 , INT32 , INT32 , stMesRam* , stMesRam* );

UINT8	RdStatus( UINT8 UcStBitChk );
void	MeasureStart2( INT32 SlMeasureParameterNum , INT32 SlMeasureParameterA , INT32 SlMeasureParameterB , UINT16 UsTime );
void	MesFil2( UINT16	UsMesFreq )	;
void	SetLinearityParameter( void );
void	SetGyroCoef( UINT8 );
void	SetAccelCoef( UINT8 );

extern	stAclVal	StAclVal ;				//!< Execute Command Parameter

//**************************
//	define					
//**************************
#define 	ONE_MSEC_COUNT	18			// 18.0288kHz * 18 ≒ 1ms

//#define 	HALL_ADJ		0
#define 	LOOPGAIN		1
#define 	THROUGH			2
#define 	NOISE			3
#define		OSCCHK			4

#define		CNT050MS		 676
#define		CNT100MS		1352
#define		CNT200MS		2703

//********************************************************************************
// Function Name 	: SetTregAf
// Retun Value		: 
// Argment Value	: Min:000h Max:7FFh (11bit) in the case of Bi-direction
// Argment Value	: Min:000h Max:3FFh (10bit) in the case of Uni-direction
// Explanation		: 
// History			: First edition 						2014.06.19 T.Tokoro
//********************************************************************************
void	SetTregAf( UINT16 UsTregAf )
{
	UINT8	UcStRd = 1;
	UINT32	UlStCnt = 0;
	

	RamWrite32A( CMD_AF_POSITION,	UsTregAf| 0x00010000 ) ;		// bit 16 : FST mode
	while( UcStRd && ( UlStCnt++ < CNT050MS) ) {
		UcStRd = RdStatus(1);
	}
TRACE("SetTregAf( status , cnt ) = %02x , %08x\n", UcStRd , (int)UlStCnt ) ;
}

//********************************************************************************
// Function Name 	: PreparationForPowerOff
// Retun Value		: 
// Argment Value	: 
// Explanation		: 
// History			: First edition 						
//********************************************************************************
void PreparationForPowerOff( void )
{
	UINT32 UlReadVa;
	DSPVER Dspcode;
		
	/* SPI communication pending */
	RamRead32A ( (GYRO_RAM_GYRO_AF_Switch & 0xFFFC), &UlReadVa );
	RamWrite32A( (GYRO_RAM_GYRO_AF_Switch & 0xFFFC), (UlReadVa|0x00008000) );

	RamWrite32A( CMD_IO_ADR_ACCESS , ROMINFO );
	RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVa );
	if ( UlReadVa == 0x01 ){
		/* Normal Ram program execution */
		GetInfomationAfterDownload( &Dspcode );
		switch ( Dspcode.GyroType ){
		case GYRO_LSM6DSM:
			/* Gyro SPI disable command set for ST by secondary SPI*/
			RamWrite32A( CMD_GYRO_WR_ACCS, 0x70000000 );
		 	break;
		} 
	}
}

//********************************************************************************
// Function Name 	: MesFil
// Retun Value		: NON
// Argment Value	: Measure Filter Mode
// Explanation		: Measure Filter Setting Function
// History			: First edition 		
//********************************************************************************
void	MesFil( UINT8	UcMesMod )		// 20.019kHz
{
	UINT32	UlMeasFilaA , UlMeasFilaB , UlMeasFilaC ;
	UINT32	UlMeasFilbA , UlMeasFilbB , UlMeasFilbC ;

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
	
	RamWrite32A ( MeasureFilterA_Coeff_a1	, UlMeasFilaA ) ;
	RamWrite32A ( MeasureFilterA_Coeff_b1	, UlMeasFilaB ) ;
	RamWrite32A ( MeasureFilterA_Coeff_c1	, UlMeasFilaC ) ;

	RamWrite32A ( MeasureFilterA_Coeff_a2	, UlMeasFilbA ) ;
	RamWrite32A ( MeasureFilterA_Coeff_b2	, UlMeasFilbB ) ;
	RamWrite32A ( MeasureFilterA_Coeff_c2	, UlMeasFilbC ) ;

	RamWrite32A ( MeasureFilterB_Coeff_a1	, UlMeasFilaA ) ;
	RamWrite32A ( MeasureFilterB_Coeff_b1	, UlMeasFilaB ) ;
	RamWrite32A ( MeasureFilterB_Coeff_c1	, UlMeasFilaC ) ;

	RamWrite32A ( MeasureFilterB_Coeff_a2	, UlMeasFilbA ) ;
	RamWrite32A ( MeasureFilterB_Coeff_b2	, UlMeasFilbB ) ;
	RamWrite32A ( MeasureFilterB_Coeff_c2	, UlMeasFilbC ) ;
}

//********************************************************************************
// Function Name 	: ClrMesFil
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Clear Measure Filter Function
// History			: First edition 						
//********************************************************************************
void	ClrMesFil( void )
{
	RamWrite32A ( MeasureFilterA_Delay_z11	, 0 ) ;
	RamWrite32A ( MeasureFilterA_Delay_z12	, 0 ) ;

	RamWrite32A ( MeasureFilterA_Delay_z21	, 0 ) ;
	RamWrite32A ( MeasureFilterA_Delay_z22	, 0 ) ;

	RamWrite32A ( MeasureFilterB_Delay_z11	, 0 ) ;
	RamWrite32A ( MeasureFilterB_Delay_z12	, 0 ) ;

	RamWrite32A ( MeasureFilterB_Delay_z21	, 0 ) ;
	RamWrite32A ( MeasureFilterB_Delay_z22	, 0 ) ;
}


//********************************************************************************
// Function Name 	: SetWaitTime
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Set Timer wait Function
// History			: First edition 						
//********************************************************************************
void	SetWaitTime( UINT16 UsWaitTime )
{
	RamWrite32A( WaitTimerData_UiWaitCounter	, 0 ) ;
	RamWrite32A( WaitTimerData_UiTargetCount	, (UINT32)(ONE_MSEC_COUNT * UsWaitTime)) ;
}

//********************************************************************************
// Function Name 	: SetTransDataAdr
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Trans Address for Data Function
// History			: First edition 						
//********************************************************************************
void	SetTransDataAdr( UINT16 UsLowAddress , UINT32 UlLowAdrBeforeTrans )
{
	UnDwdVal	StTrsVal ;
	
	if( UlLowAdrBeforeTrans < 0x00009000 ){
		StTrsVal.UlDwdVal = UlLowAdrBeforeTrans ;
	}else{
		StTrsVal.StDwdVal.UsHigVal = (UINT16)(( UlLowAdrBeforeTrans & 0x0000F000 ) >> 8 ) ;
		StTrsVal.StDwdVal.UsLowVal = (UINT16)( UlLowAdrBeforeTrans & 0x00000FFF ) ;
	}
//TRACE(" TRANS  ADR = %04xh , DAT = %08xh \n",UsLowAddress , StTrsVal.UlDwdVal ) ;
	RamWrite32A( UsLowAddress	,	StTrsVal.UlDwdVal );
	
}

//********************************************************************************
// Function Name 	: MemoryClear
// Retun Value		: NON
// Argment Value	: Top pointer , Size
// Explanation		: Memory Clear Function
// History			: First edition 						
//********************************************************************************
void	MemoryClear( UINT16 UsSourceAddress, UINT16 UsClearSize )
{
	UINT16	UsLoopIndex ;

	for ( UsLoopIndex = 0 ; UsLoopIndex < UsClearSize ; UsLoopIndex += 4 ) {
		RamWrite32A( UsSourceAddress + UsLoopIndex	, 	0x00000000 ) ;				// 4Byte
//TRACE("MEM CLR ADR = %04xh \n",UsSourceAddress + UsLoopIndex) ;
	}
}

//********************************************************************************
// Function Name 	: MeasureStart
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure start setting Function
// History			: First edition 						
//********************************************************************************
void	MeasureStart( INT32 SlMeasureParameterNum , UINT32 SlMeasureParameterA , UINT32 SlMeasureParameterB )
{
	MemoryClear( StMeasFunc_SiSampleNum , sizeof( MeasureFunction_Type ) ) ;
	RamWrite32A( StMeasFunc_MFA_SiMax1	 , 0x80000000 ) ;					// Set Min 
	RamWrite32A( StMeasFunc_MFB_SiMax2	 , 0x80000000 ) ;					// Set Min 
	RamWrite32A( StMeasFunc_MFA_SiMin1	 , 0x7FFFFFFF ) ;					// Set Max 
	RamWrite32A( StMeasFunc_MFB_SiMin2	 , 0x7FFFFFFF ) ;					// Set Max 
	
	SetTransDataAdr( StMeasFunc_MFA_PiMeasureRam1	 , SlMeasureParameterA ) ;		// Set Measure Filter A Ram Address
	SetTransDataAdr( StMeasFunc_MFB_PiMeasureRam2	 , SlMeasureParameterB ) ;		// Set Measure Filter B Ram Address
	RamWrite32A( StMeasFunc_SiSampleNum	 , 0 ) ;													// Clear Measure Counter 
	ClrMesFil() ;						// Clear Delay Ram
//	SetWaitTime(50) ;
	SetWaitTime(1) ;
	RamWrite32A( StMeasFunc_SiSampleMax	 , SlMeasureParameterNum ) ;						// Set Measure Max Number

}
	
//********************************************************************************
// Function Name 	: MeasureWait
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Wait complete of Measure Function
// History			: First edition 						
//********************************************************************************
void	MeasureWait( void )
{
	UINT32			SlWaitTimerSt ;
	
	SlWaitTimerSt = 1 ;
	while( SlWaitTimerSt ){
		RamRead32A( StMeasFunc_SiSampleMax , &SlWaitTimerSt ) ;
	}
}

//********************************************************************************
// Function Name 	: SetGyroOffset
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: set the gyro offset data. before do this before Remapmain.
// History			: First edition 						
//********************************************************************************
void	SetGyroOffset( UINT16 GyroOffsetX, UINT16 GyroOffsetY, UINT16 GyroOffsetZ )
{
	RamWrite32A( GYRO_RAM_GXOFFZ , (( GyroOffsetX << 16 ) & 0xFFFF0000 ) ) ;		// X axis Gyro offset
	RamWrite32A( GYRO_RAM_GYOFFZ , (( GyroOffsetY << 16 ) & 0xFFFF0000 ) ) ;		// Y axis Gyro offset
	RamWrite32A( GYRO_RAM_GZOFFZ , (( GyroOffsetZ << 16 ) & 0xFFFF0000 ) ) ;		// Z axis Gyro offset
}

//********************************************************************************
// Function Name 	:GyroOffsetMeasureStart
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: start the gyro offset adjustment
// History			: First edition 						
//********************************************************************************
#define 	GYROF_NUM		2048			// 2048times
void GyroOffsetMeasureStart( void )	
{
	MesFil( THROUGH ) ;								// Set Measure Filter
	MeasureStart( GYROF_NUM , GYRO_RAM_GX_ADIDAT , GYRO_RAM_GY_ADIDAT ) ;	// Start measure
}

//********************************************************************************
// Function Name 	:GyroOffsetMeasureStartZ
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: start the gyro offset adjustment
// History			: First edition 						
//********************************************************************************
#define 	GYROF_NUM		2048			// 2048times
void GyroOffsetMeasureStartZ( void )	
{
	MesFil( THROUGH ) ;								// Set Measure Filter
	MeasureStart( GYROF_NUM , GYRO_RAM_GZ_ADIDAT , GYRO_RAM_GZ_ADIDAT ) ;	// Start measure
}

//********************************************************************************
// Function Name 	: GetGyroOffset
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: get the gyro offset adjustment result
// History			: First edition 						
//********************************************************************************
UINT8 GetGyroOffset( UINT16* GyroOffsetX, UINT16* GyroOffsetY, INT16 GYROF_UPPER, INT16 GYROF_LOWER   )
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
			return( 4 );
		}
		RamRead32A( StMeasFunc_SiSampleMax , &UlReadVal ) ;
	}while ( UlReadVal != 0 );
	
	RamRead32A( StMeasFunc_MFA_SiMax1 , ( UINT32 * )&SlMeasureMaxValue ) ;	// Max value
	RamRead32A( StMeasFunc_MFA_SiMin1 , ( UINT32 * )&SlMeasureMinValue ) ;	// Min value
	if (SlMeasureMaxValue == SlMeasureMinValue )
	{
		return( 8 ); 
	}
	
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4		, &StMeasValueA.StUllnVal.UlHigVal ) ;
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;
	
	SlMeasureAveValueA = (INT32)( (INT64)StMeasValueA.UllnValue / GYROF_NUM ) ;
	SlMeasureAveValueB = (INT32)( (INT64)StMeasValueB.UllnValue / GYROF_NUM ) ;
	
	SlMeasureAveValueA = ( SlMeasureAveValueA >> 16 ) & 0x0000FFFF ;
	SlMeasureAveValueB = ( SlMeasureAveValueB >> 16 ) & 0x0000FFFF ;
	// EP1では反転処理しない。
//	SlMeasureAveValueA = 0x00010000 - SlMeasureAveValueA ;
//	SlMeasureAveValueB = 0x00010000 - SlMeasureAveValueB ;
	
	*GyroOffsetX = ( UINT16 )( SlMeasureAveValueA & 0x0000FFFF );		//Measure Result Store
	*GyroOffsetY = ( UINT16 )( SlMeasureAveValueB & 0x0000FFFF );		//Measure Result Store

	if(( (INT16)(*GyroOffsetX) > GYROF_UPPER ) || ( (INT16)(*GyroOffsetX) < GYROF_LOWER )){
		ans |= 1; 
	}
	if(( (INT16)(*GyroOffsetY) > GYROF_UPPER ) || ( (INT16)(*GyroOffsetY) < GYROF_LOWER )){
		ans |= 2; 
	}

	return( ans ); 
}

#ifdef	SEL_SHIFT_COR
//********************************************************************************
// Function Name 	: SetAcclOffset
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: set the accl offset data. before do this before Remapmain.
// History			: First edition 						
//********************************************************************************
void	SetAcclOffset( UINT16 AcclOffsetX, UINT16 AcclOffsetY, UINT16 AcclOffsetZ )
{
	RamWrite32A( ZeroServoRAM_X_OFFSET , ( ( AcclOffsetX << 16 ) & 0xFFFF0000 ) ) ;		// X axis Accl offset
	RamWrite32A( ZeroServoRAM_Y_OFFSET , ( ( AcclOffsetY << 16 ) & 0xFFFF0000 ) ) ;		// Y axis Accl offset
	RamWrite32A( ZeroServoRAM_Z_OFFSET , ( ( AcclOffsetZ << 16 ) & 0xFFFF0000 ) ) ;		// Z axis Accl offset
}

//********************************************************************************
// Function Name 	: GetAcclOffset
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: get the accl offset adjustment result
// History			: First edition 						
//********************************************************************************
void	GetAcclOffset( UINT16* AcclOffsetX, UINT16* AcclOffsetY, UINT16* AcclOffsetZ )
{
	*AcclOffsetX = ( UINT16 )( StAclVal.StAccel.SlOffsetX & 0x0000FFFF );
	*AcclOffsetY = ( UINT16 )( StAclVal.StAccel.SlOffsetY & 0x0000FFFF );
	*AcclOffsetZ = ( UINT16 )( StAclVal.StAccel.SlOffsetZ & 0x0000FFFF );	
}
#endif	//SEL_SHIFT_COR

//********************************************************************************
// Function Name 	: RtnCen
// Retun Value		: Command Status
// Argment Value	: Command Parameter
// Explanation		: Return to center Command Function
// History			: First edition 						
//********************************************************************************
UINT8	RtnCen( UINT8	UcCmdPar )
{
	UINT8	UcSndDat = 1 ;
	UINT32	UlStCnt = 0;
	
	if( !UcCmdPar ){								// X,Y centering
		RamWrite32A( CMD_RETURN_TO_CENTER , BOTH_SRV_ON ) ;
	}else if( UcCmdPar == XONLY_ON ){				// only X centering
		RamWrite32A( CMD_RETURN_TO_CENTER , XAXS_SRV_ON ) ;
	}else if( UcCmdPar == YONLY_ON ){				// only Y centering
		RamWrite32A( CMD_RETURN_TO_CENTER , YAXS_SRV_ON ) ;
	}else{											// Both off
		RamWrite32A( CMD_RETURN_TO_CENTER , BOTH_SRV_OFF ) ;
	}
	
	while( UcSndDat && (UlStCnt++ < CNT200MS )) {
		UcSndDat = RdStatus(1);
	}
TRACE("RtnCen( cmd , status , cnt ) = %02x , %02x , %08x \n", UcCmdPar , UcSndDat , (int)UlStCnt ) ;
	return( UcSndDat );
}



//********************************************************************************
// Function Name 	: ZsvCnt
// Retun Value		: Command Status
// Argment Value	: Command Parameter
// Explanation		: Zero servo control Command Function
// History			: First edition
//********************************************************************************
UINT8	ZscCnt( UINT8	UcCmdPar )
{
	UINT8	UcSndDat = 1 ;
	UINT32	UlStCnt = 0;

TRACE("ZscCnt(%02x) = ", UcCmdPar ) ;
	if( !UcCmdPar ){								// Zero servo off
		RamWrite32A( CMD_ZSRV_MODE , ZSRV_DISABLE ) ;
	}else if( UcCmdPar == 1 ){						// Zero servo on
		RamWrite32A( CMD_ZSRV_MODE , ZSRV_ENABLE ) ;
	}else if( UcCmdPar == 3 ){						// Zero servo hold
		RamWrite32A( CMD_ZSRV_MODE , ZSRV_HOLD ) ;
	}else{
		return( 2 );
	}
	
	while( UcSndDat && (UlStCnt++ < CNT050MS )) {
		UcSndDat = RdStatus(1);
	}
TRACE("ZscCnt( cmd , status , cnt ) = %02x , %02x , %08x \n", UcCmdPar , UcSndDat , (int)UlStCnt ) ;
	
	return( UcSndDat );
}

//********************************************************************************
// Function Name 	: OisEna
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function
// History			: First edition 						
//********************************************************************************
void	OisEna( void )
{
	UINT8	UcStRd = 1;
	UINT32	UlStCnt = 0;
	UINT32	gyro_gain_x, gyro_gain_y ;
	
	RamRead32A( GyroFilterTableX_gxzoom, &gyro_gain_x ) ;
	RamRead32A( GyroFilterTableY_gyzoom, &gyro_gain_y ) ;
	RamWrite32A( GyroFilterTableX_gxzoom, gyro_gain_y ) ;
	RamWrite32A( GyroFilterTableY_gyzoom, gyro_gain_x ) ;
	RamWrite32A( CMD_OIS_ENABLE , OIS_ENABLE ) ;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = RdStatus(1);
	}
TRACE(" OisEna( Status , cnt ) = %02x , %08x \n", UcStRd , (int)UlStCnt ) ;
}

//********************************************************************************
// Function Name 	: OisEnaNCL
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function w/o delay clear
// History			: First edition 						
//********************************************************************************
void	OisEnaNCL( void )
{
	UINT8	UcStRd = 1;
	UINT32	UlStCnt = 0;
	
	RamWrite32A( CMD_OIS_ENABLE , OIS_ENA_NCL | OIS_ENABLE ) ;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = RdStatus(1);
	}
TRACE(" OisEnaNCL( Status , cnt ) = %02x %08x \n", UcStRd , (int)UlStCnt ) ;
}

//********************************************************************************
// Function Name 	: OisEnaDrCl
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function w/o delay clear
// History			: First edition 						
//********************************************************************************
void	OisEnaDrCl( void )
{
	UINT8	UcStRd = 1;
	UINT32	UlStCnt = 0;
	
	RamWrite32A( CMD_OIS_ENABLE , OIS_ENA_DOF | OIS_ENABLE ) ;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = RdStatus(1);
	}
TRACE(" OisEnaDrCl( Status , cnt ) = %02x , %08x \n", UcStRd , (int)UlStCnt ) ;
}

//********************************************************************************
// Function Name 	: OisEnaDrNcl
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function w/o delay clear
// History			: First edition 						
//********************************************************************************
void	OisEnaDrNcl( void )
{
	UINT8	UcStRd = 1;
	UINT32	UlStCnt = 0;
	
	RamWrite32A( CMD_OIS_ENABLE , OIS_ENA_DOF | OIS_ENA_NCL | OIS_ENABLE ) ;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = RdStatus(1);
	}
TRACE(" OisEnaDrCl( Status , cnt ) = %02x , %08x \n", UcStRd , (int)UlStCnt ) ;
}
//********************************************************************************
// Function Name 	: OisDis
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Disable Control Function
// History			: First edition 						
//********************************************************************************
void	OisDis( void )
{
	UINT8	UcStRd = 1;
	UINT32	UlStCnt = 0;
	
	RamWrite32A( CMD_OIS_ENABLE , OIS_DISABLE ) ;
	while( UcStRd && ( UlStCnt++ < CNT050MS)) {
		UcStRd = RdStatus(1);
	}
TRACE(" OisDis( Status , cnt ) = %02x , %08x \n", UcStRd , (int)UlStCnt ) ;
}

//********************************************************************************
// Function Name 	: SscEna
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Ssc Enable Control Function
// History			: First edition
//********************************************************************************
void	SscEna( void )
{
	UINT8	UcStRd = 1;
	UINT32	UlStCnt = 0;

	RamWrite32A( CMD_SSC_ENABLE , SSC_ENABLE ) ;
	while( UcStRd && ( UlStCnt++ < CNT050MS)) {
		UcStRd = RdStatus(1);
	}
TRACE(" SscEna( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: SscDis
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Ssc Disable Control Function
// History			: First edition
//********************************************************************************
void	SscDis( void )
{
	UINT8	UcStRd = 1;
	UINT32	UlStCnt = 0;

	RamWrite32A( CMD_SSC_ENABLE , SSC_DISABLE ) ;
	while( UcStRd && ( UlStCnt++ < CNT050MS)) {
		UcStRd = RdStatus(1);
	}
TRACE(" SscDis( Status) = %02x\n", UcStRd ) ;
}


//********************************************************************************
// Function Name 	: SetRec
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Rec Mode Enable Function
// History			: First edition 						
//********************************************************************************
void	SetRec( void )
{
	UINT8	UcStRd = 1;
	UINT32	UlStCnt = 0;
	
	RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE ) ;
	while( UcStRd && ( UlStCnt++ < CNT050MS)) {
		UcStRd = RdStatus(1);
	}
TRACE(" SetRec( Status , cnt ) = %02x , %08x \n", UcStRd , (int)UlStCnt ) ;
}


//********************************************************************************
// Function Name 	: SetStill
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set Still Mode Enable Function
// History			: First edition 						
//********************************************************************************
void	SetStill( void )
{
	UINT8	UcStRd = 1;
	UINT32	UlStCnt = 0;
	
	RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE ) ;
	while( UcStRd && ( UlStCnt++ < CNT050MS)) {
		UcStRd = RdStatus(1);
	}
TRACE(" SetStill( Status , cnt ) = %02x , %08x \n", UcStRd , (int)UlStCnt ) ;
}

//********************************************************************************
// Function Name 	: SetRecPreview
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Rec Preview Mode Enable Function
// History			: First edition 						
//********************************************************************************
void	SetRecPreview( UINT8 mode )
{
	UINT8	UcStRd = 1;
	UINT32	UlStCnt = 0;
	
	switch( mode ){
	case 0:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE ) ;
		break;
	case 1:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE1 ) ;
		break;
	case 2:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE2 ) ;
		break;
	case 3:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE3 ) ;
		break;
	}
	while( UcStRd && ( UlStCnt++ < CNT050MS)) {
		UcStRd = RdStatus(1);
	}
TRACE(" SetRec( %02x )( status , cnt ) = %02x , %08x , \n", mode , UcStRd ,(int)UlStCnt ) ;
}


//********************************************************************************
// Function Name 	: SetStillPreview
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set Still Preview Mode Enable Function
// History			: First edition 						
//********************************************************************************
void	SetStillPreview( unsigned char mode )
{
	UINT8	UcStRd = 1;
	UINT32	UlStCnt = 0;
	
	switch( mode ){
	case 0:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE ) ;
		break;
	case 1:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE1 ) ;
		break;
	case 2:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE2 ) ;
		break;
	case 3:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE3 ) ;
		break;
	}
	while( UcStRd && ( UlStCnt++ < CNT050MS)) {
		UcStRd = RdStatus(1);
	}
TRACE(" SetStill( %02x )( status , cnt ) = %02x , %08x \n", mode , UcStRd , (int)UlStCnt ) ;
}

//********************************************************************************
// Function Name 	: SetPanTiltMode
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Pan-Tilt Enable/Disable
// History			: First edition 						
//********************************************************************************
void	SetPanTiltMode( UINT8 UcPnTmod )
{
	UINT8	UcStRd = 1;
	UINT32	UlStCnt = 0;
	
	switch ( UcPnTmod ) {
		case OFF :
			RamWrite32A( CMD_PAN_TILT ,	PAN_TILT_OFF ) ;
//TRACE(" PanTilt OFF\n");
			break ;
		case ON :
			RamWrite32A( CMD_PAN_TILT ,	PAN_TILT_ON ) ;
//TRACE(" PanTilt ON\n");
			break ;
	}
	while( UcStRd && ( UlStCnt++ < CNT050MS)) {
		UcStRd = RdStatus(1);
	}
TRACE(" PanTilt( Status , mode , cnt ) = %02x , %02x , %08x \n", UcStRd , UcPnTmod , (int)UlStCnt ) ;

}

//********************************************************************************
// Function Name 	: AfStbyRls
// Retun Value		: Command Status
// Argment Value	: Command Parameter
// Explanation		: Af Standby release Command Function
// History			: First edition
//********************************************************************************
UINT8	AfStbyRls( void )
{

TRACE("AfStbyRls \n" ) ;
	DMIOWrite32( SYSDSP_STBOTH, 0x00000C00 );	
	
	return( SUCCESS );
}

//********************************************************************************
// Function Name 	: RdStatus
// Retun Value		: 0:success 1:FAILURE
// Argment Value	: bit check  0:ALL  1:bit24
// Explanation		: High level status check Function
// History			: First edition 						
//********************************************************************************
UINT8	RdStatus( UINT8 UcStBitChk )
{
	UINT32	UlReadVal ;
	
	RamRead32A( CMD_READ_STATUS , &UlReadVal );
//TRACE(" (Rd St) = %08x\n", (unsigned INT16)UlReadVal ) ;
	if( UcStBitChk ){
		UlReadVal &= READ_STATUS_INI ;
	}
	if( !UlReadVal ){
		return( SUCCESS );
	}else{
		return( FAILURE );
	}
}


//********************************************************************************
// Function Name 	: SetSinWavGenInt
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Sine wave generator initial Function
// History			: First edition 						
//********************************************************************************
void	SetSinWavGenInt( void )
{
	
	RamWrite32A( SinWave_Offset		,	0x00000000 ) ;		// 発生周波数のオフセットを設定
	RamWrite32A( SinWave_Phase		,	0x60000000 ) ;		// 正弦波の位相量
	RamWrite32A( SinWave_Gain		,	0x00000000 ) ;		// 発生周波数のアッテネータ(初期値は0[dB])
//	RamWrite32A( SinWave_Gain		,	0x7FFFFFFF ) ;		// 発生周波数のアッテネータ(初期値はCut)
//	SetTransDataAdr( SinWave_OutAddr	,	(UINT32)SinWave_Output ) ;		// 初期値の出力先アドレスは、自分のメンバ

	RamWrite32A( CosWave_Offset		,	0x00000000 );		// 発生周波数のオフセットを設定
	RamWrite32A( CosWave_Phase 		,	0x00000000 );		// 正弦波の位相量
	RamWrite32A( CosWave_Gain 		,	0x00000000 );		// 発生周波数のアッテネータ(初期値はCut)
//	RamWrite32A( CosWave_Gain 		,	0x7FFFFFFF );		// 発生周波数のアッテネータ(初期値は0[dB])
//	SetTransDataAdr( CosWave_OutAddr	,	(UINT32)CosWave_Output );		// 初期値の出力先アドレスは、自分のメンバ
	
	RamWrite32A( SinWaveC_Regsiter	,	0x00000000 ) ;								// Sine Wave Stop
	
}


//********************************************************************************
// Function Name 	: TstActMov
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Hall Examination of Acceptance
// History			: First edition 						
//********************************************************************************
// #define		ACT_CHK_LVL		0x33320000		// 0.4
 #define		ACT_CHK_FRQ		0x00074528		// 4Hz
 #define		ACT_CHK_NUM		4507			// 18.0288/0.004 
 #define		ACT_THR			0x000003E8		// 20dB 10*100
 #define		ACT_MARGIN		0.75f			// 
 
UINT8	TstActMov( UINT8 UcDirSel )
{
	UINT8	UcRsltSts = 0;
	INT32	SlMeasureParameterNum ;
	INT32	SlMeasureParameterA , SlMeasureParameterB ;
	UnllnVal	StMeasValueA  , StMeasValueB ;
	float		SfLimit , Sfzoom , Sflenz , Sfshift ;
	UINT32		UlLimit , Ulzoom , Ullenz , Ulshift , UlActChkLvl ;
	UINT8		i;
	UINT32		UlReturnVal;

	if( UcDirSel == X_DIR ) {								// X axis
		RamRead32A( Gyro_Limiter_X 			, ( UINT32 * )&UlLimit ) ;	// 
		RamRead32A( GyroFilterTableX_gxzoom , ( UINT32 * )&Ulzoom ) ;	// 
		RamRead32A( GyroFilterTableX_gxlenz , ( UINT32 * )&Ullenz ) ;	// 
		RamRead32A( Gyro_ShiftX_RG 			, ( UINT32 * )&Ulshift ) ;	// 
	}else{
		RamRead32A( Gyro_Limiter_Y 			, ( UINT32 * )&UlLimit ) ;	// 
		RamRead32A( GyroFilterTableY_gyzoom , ( UINT32 * )&Ulzoom ) ;	// 
		RamRead32A( GyroFilterTableY_gylenz , ( UINT32 * )&Ullenz ) ;	// 
		RamRead32A( Gyro_ShiftY_RG 			, ( UINT32 * )&Ulshift ) ;	// 
	}

TRACE(" DIR = %d, lmt = %08x, zom = %08x , lnz = %08x ,sft = %08x \n", UcDirSel, (unsigned int)UlLimit , (unsigned int)Ulzoom , (unsigned int)Ullenz , (unsigned int)Ulshift  ) ;

	SfLimit = (float)UlLimit / (float)0x7FFFFFFF;
	if( Ulzoom == 0){
		Sfzoom = 0;
	}else{
		Sfzoom = (float)abs(Ulzoom) / (float)0x7FFFFFFF;
	}
	if( Ullenz == 0){
		Sflenz = 0;
	}else{
		Sflenz = (float)Ullenz / (float)0x7FFFFFFF;
	}
	Ulshift = ( Ulshift & 0x0000FF00) >> 8 ;	// 2X4XB
	Sfshift = 1;
	for( i = 0 ; i < Ulshift ; i++ ){
		Sfshift *= 2;
	}
	UlActChkLvl = (UINT32)( (float)0x7FFFFFFF * SfLimit * Sfzoom * Sflenz * Sfshift * ACT_MARGIN );
//TRACE(" lvl = %08x \n", (unsigned int)UlActChkLvl  ) ;

	SlMeasureParameterNum	=	ACT_CHK_NUM ;

	if( UcDirSel == X_DIR ) {								// X axis
		SlMeasureParameterA		=	HALL_RAM_HXOUT0 ;			// Set Measure RAM Address
		SlMeasureParameterB		=	HallFilterD_HXDAZ1 ;		// Set Measure RAM Address
	} else if( UcDirSel == Y_DIR ) {						// Y axis
		SlMeasureParameterA		=	HALL_RAM_HYOUT0 ;			// Set Measure RAM Address
		SlMeasureParameterB		=	HallFilterD_HYDAZ1 ;		// Set Measure RAM Address
	}
	SetSinWavGenInt();
	
	RamWrite32A( SinWave_Offset		,	ACT_CHK_FRQ ) ;				// Freq Setting = Freq * 80000000h / Fs	: 5Hz
	RamWrite32A( SinWave_Gain		,	UlActChkLvl ) ;				// Set Sine Wave Gain
	RamWrite32A( SinWaveC_Regsiter	,	0x00000001 ) ;				// Sine Wave Start
	if( UcDirSel == X_DIR ) {
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_RAM_HXOFF1 ) ;	// Set Sine Wave Input RAM
	}else if( UcDirSel == Y_DIR ){
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_RAM_HYOFF1 ) ;	// Set Sine Wave Input RAM
	}
	MesFil( NOISE ) ;					// 測定用フィルターを設定する。

	MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure
	
	MeasureWait() ;						// Wait complete of measurement
	
	RamWrite32A( SinWaveC_Regsiter	,	0x00000000 ) ;								// Sine Wave Stop
	
	if( UcDirSel == X_DIR ) {
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)0x00000000 ) ;	// Set Sine Wave Input RAM
		RamWrite32A( HALL_RAM_HXOFF1		,	0x00000000 ) ;				// DelayRam Clear
	}else if( UcDirSel == Y_DIR ){
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)0x00000000 ) ;	// Set Sine Wave Input RAM
		RamWrite32A( HALL_RAM_HYOFF1		,	0x00000000 ) ;				// DelayRam Clear
	}
	RamRead32A( StMeasFunc_MFA_LLiAbsInteg1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	
	RamRead32A( StMeasFunc_MFA_LLiAbsInteg1 + 4 	, &StMeasValueA.StUllnVal.UlHigVal ) ;
	RamRead32A( StMeasFunc_MFB_LLiAbsInteg2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	
	RamRead32A( StMeasFunc_MFB_LLiAbsInteg2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;


	UlReturnVal = (INT32)((INT64)StMeasValueA.UllnValue * 100 / (INT64)StMeasValueB.UllnValue  ) ;


TRACE(" Ret = %d \n", (unsigned int)UlReturnVal ) ;

	
	UcRsltSts = EXE_END ;
	if( UlReturnVal < ACT_THR ){
		if ( !UcDirSel ) {					// AXIS X
			UcRsltSts = EXE_HXMVER ;
		}else{								// AXIS Y
			UcRsltSts = EXE_HYMVER ;
		}
	}

	return( UcRsltSts ) ;

}
//********************************************************************************
// Function Name 	: RunHea
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Hall Examination of Acceptance
// History			: First edition 						
//********************************************************************************
UINT8	RunHea( void )
{
	UINT8 	UcRst ;
	UcRst = EXE_END ;
	UcRst |= TstActMov( X_DIR) ;
	UcRst |= TstActMov( Y_DIR) ;
	
//TRACE("UcRst = %02x\n", UcRst ) ;
	return( UcRst ) ;
}


#if ((SELECT_VENDOR & 0x80 ) != 0x80)
//********************************************************************************
// Function Name 	: RunGea
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Gyro Examination of Acceptance
// History			: First edition 						
//********************************************************************************
#define 	GEA_NUM		512			// 512times
// #define		GEA_DIF_HIG		0x0062			// 2021_32.8lsb/°/s    max 3.0°/s-p-p
 #define		GEA_DIF_HIG		0x0057			// 2030_87.5lsb/°/s    max 1.0°/s-p-p
 #define		GEA_DIF_LOW		0x0001				// Gyro Examination of Acceptance
 
UINT8	RunGea( void )
{
	UnllnVal	StMeasValueA , StMeasValueB ;
	INT32		SlMeasureParameterA , SlMeasureParameterB ;
	UINT8 		UcRst, UcCnt, UcXLowCnt, UcYLowCnt, UcXHigCnt, UcYHigCnt ;
	UINT16		UsGxoVal[10], UsGyoVal[10], UsDif;
	INT32		SlMeasureParameterNum , SlMeasureAveValueA , SlMeasureAveValueB ;

	
	UcRst = EXE_END ;
	UcXLowCnt = UcYLowCnt = UcXHigCnt = UcYHigCnt = 0 ;
	
	MesFil( THROUGH ) ;				// 測定用フィルターを設定する。
	
	for( UcCnt = 0 ; UcCnt < 10 ; UcCnt++ )
	{
		//平均値測定
	
		MesFil( THROUGH ) ;					// Set Measure Filter

		SlMeasureParameterNum	=	GEA_NUM ;					// Measurement times
		SlMeasureParameterA		=	GYRO_RAM_GX_ADIDAT ;		// Set Measure RAM Address
		SlMeasureParameterB		=	GYRO_RAM_GY_ADIDAT ;		// Set Measure RAM Address
		
		MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure
	
		MeasureWait() ;					// Wait complete of measurement
	
//TRACE("Read Adr = %04x, %04xh \n",StMeasFunc_MFA_LLiIntegral1 + 4 , StMeasFunc_MFA_LLiIntegral1) ;
		RamRead32A( StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
		RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4		, &StMeasValueA.StUllnVal.UlHigVal ) ;
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;
	
//TRACE("GX_OFT = %08x, %08xh \n",(unsigned int)StMeasValueA.StUllnVal.UlHigVal,(unsigned int)StMeasValueA.StUllnVal.UlLowVal) ;
//TRACE("GY_OFT = %08x, %08xh \n",(unsigned int)StMeasValueB.StUllnVal.UlHigVal,(unsigned int)StMeasValueB.StUllnVal.UlLowVal) ;
		SlMeasureAveValueA = (INT32)( (INT64)StMeasValueA.UllnValue / SlMeasureParameterNum ) ;
		SlMeasureAveValueB = (INT32)( (INT64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;
//TRACE("GX_AVEOFT = %08xh \n",(unsigned int)SlMeasureAveValueA) ;
//TRACE("GY_AVEOFT = %08xh \n",(unsigned int)SlMeasureAveValueB) ;
		// X
		UsGxoVal[UcCnt] = (UINT16)( SlMeasureAveValueA >> 16 );	// 平均値測定
		
		// Y
		UsGyoVal[UcCnt] = (UINT16)( SlMeasureAveValueB >> 16 );	// 平均値測定
		
//TRACE("UcCnt = %02x, UsGxoVal[UcCnt] = %04x\n", UcCnt, UsGxoVal[UcCnt] ) ;
//TRACE("UcCnt = %02x, UsGyoVal[UcCnt] = %04x\n", UcCnt, UsGyoVal[UcCnt] ) ;
		
		
		if( UcCnt > 0 )
		{
			if ( (INT16)UsGxoVal[0] > (INT16)UsGxoVal[UcCnt] ) {
				UsDif = (UINT16)((INT16)UsGxoVal[0] - (INT16)UsGxoVal[UcCnt]) ;
			} else {
				UsDif = (UINT16)((INT16)UsGxoVal[UcCnt] - (INT16)UsGxoVal[0]) ;
			}
			
			if( UsDif > GEA_DIF_HIG ) {
				//UcRst = UcRst | EXE_GXABOVE ;
				UcXHigCnt ++ ;
			}
			if( UsDif < GEA_DIF_LOW ) {
				//UcRst = UcRst | EXE_GXBELOW ;
				UcXLowCnt ++ ;
			}
//TRACE("CNT = %02x  ,  X diff = %04x ", UcCnt , UsDif ) ;
			
			if ( (INT16)UsGyoVal[0] > (INT16)UsGyoVal[UcCnt] ) {
				UsDif = (UINT16)((INT16)UsGyoVal[0] - (INT16)UsGyoVal[UcCnt]) ;
			} else {
				UsDif = (UINT16)((INT16)UsGyoVal[UcCnt] - (INT16)UsGyoVal[0]) ;
			}
			
			if( UsDif > GEA_DIF_HIG ) {
				//UcRst = UcRst | EXE_GYABOVE ;
				UcYHigCnt ++ ;
			}
			if( UsDif < GEA_DIF_LOW ) {
				//UcRst = UcRst | EXE_GYBELOW ;
				UcYLowCnt ++ ;
			}
//TRACE("  Y diff = %04x \n", UsDif ) ;
		}
	}
	
	if( UcXHigCnt >= 1 ) {
		UcRst = UcRst | EXE_GXABOVE ;
	}
	if( UcXLowCnt > 8 ) {
		UcRst = UcRst | EXE_GXBELOW ;
	}
	
	if( UcYHigCnt >= 1 ) {
		UcRst = UcRst | EXE_GYABOVE ;
	}
	if( UcYLowCnt > 8 ) {
		UcRst = UcRst | EXE_GYBELOW ;
	}
	
//TRACE("UcRst = %02x\n", UcRst ) ;
	
	return( UcRst ) ;
}
#endif //((SELECT_VENDOR & 0x80 ) != 0x80)


//********************************************************************************
// Function Name 	: GetGyroWhoAmI
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Get Gyro Who Am I
// History			: First edition 						
//********************************************************************************
void GetGyroWhoAmI( UINT8 * UcWho )
{
	UINT32	UlVal ;
	
	RamWrite32A( 0xF01D , 0x75000000 ) ;
	WitTim( 5 ) ;
	
	RamRead32A( 0xF01D , &UlVal ) ;
//TRACE("%08x \n", UlVal );
	
	*UcWho = (UINT8)( UlVal >> 24 ) ;
}

//********************************************************************************
// Function Name 	: GetGyroID
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Get Gyro ID
// History			: First edition 						
//********************************************************************************
void GetGyroID( UINT8 * UcGid )
{
	UINT32	UlCnt;
	UINT32	UlVal;
	
	for( UlCnt = 0; UlCnt < 8; UlCnt++ ){
		RamWrite32A( 0xF01E, 0x6D000000 ) ;
		WitTim( 5 ) ;
		
		RamWrite32A( 0xF01E, ( 0x6E000000 | ( UlCnt << 16 ) ) ) ;
		WitTim( 5 ) ;
		
		RamWrite32A( 0xF01D, 0x6F000000 ) ;
		WitTim( 5 ) ;
		
		RamRead32A( 0xF01D, &UlVal ) ;
//TRACE("%08x \n", UlVal );
		
		WitTim( 5 ) ;
		UcGid[UlCnt] = (UINT8)( UlVal >> 24 ) ;
	}
}

//********************************************************************************
// Function Name 	: GyroSleep
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Gyro Sleep Control
// History			: First edition 						
//********************************************************************************
void GyroSleep( UINT8 UcCtrl )
{
	UINT8	UcReg;
	UINT32	UlVal;
	
	RamWrite32A( 0xF01D, 0x6B000000 ) ;
	WitTim( 5 ) ;
	
	RamRead32A( 0xF01D, &UlVal ) ;
	WitTim( 5 ) ;
	
	UcReg = (UINT8)( UlVal >> 24 ) ;
	
	if( UcCtrl == ON ){
		UcReg = UcReg | 0x40;	// Bit6(SLEEP) ON
	}
	else if( UcCtrl == OFF ){
		UcReg = UcReg & 0xBF;	// Bit6(SLEEP) OFF
	}
	
	RamWrite32A( 0xF01E, ( 0x6B000000 | ( UcReg << 16 ) ) ) ;
}

//********************************************************************************
// Function Name 	: MesRam
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure 
// History			: First edition 						2015.07.06 
//********************************************************************************
UINT8	 MesRam( INT32 SlMeasureParameterA, INT32 SlMeasureParameterB, INT32 SlMeasureParameterNum, stMesRam* pStMesRamA, stMesRam* pStMesRamB )
{
	UnllnVal	StMeasValueA , StMeasValueB ;
	
	MesFil( THROUGH ) ;							// Set Measure Filter
	
	MeasureStart( SlMeasureParameterNum,  SlMeasureParameterA, SlMeasureParameterB	) ;		// Start measure
	
	MeasureWait() ;								// Wait complete of measurement
	
	// A : X axis
	RamRead32A( StMeasFunc_MFA_SiMax1 , &(pStMesRamA->SlMeasureMaxValue) ) ;			// Max value
	RamRead32A( StMeasFunc_MFA_SiMin1 , &(pStMesRamA->SlMeasureMinValue) ) ;			// Min value
	RamRead32A( StMeasFunc_MFA_UiAmp1 , &(pStMesRamA->SlMeasureAmpValue) ) ;			// Amp value
	RamRead32A( StMeasFunc_MFA_LLiIntegral1,	 &(StMeasValueA.StUllnVal.UlLowVal) ) ;	// Integration Low
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4, &(StMeasValueA.StUllnVal.UlHigVal) ) ;	// Integration Hig
	pStMesRamA->SlMeasureAveValue = 
				(INT32)( (INT64)StMeasValueA.UllnValue / SlMeasureParameterNum ) ;	// Ave value
	
	// B : Y axis
	RamRead32A( StMeasFunc_MFB_SiMax2 , &(pStMesRamB->SlMeasureMaxValue) ) ;			// Max value
	RamRead32A( StMeasFunc_MFB_SiMin2 , &(pStMesRamB->SlMeasureMinValue) ) ;			// Min value
	RamRead32A( StMeasFunc_MFB_UiAmp2 , &(pStMesRamB->SlMeasureAmpValue) ) ;			// Amp value
	RamRead32A( StMeasFunc_MFB_LLiIntegral2,	 &(StMeasValueB.StUllnVal.UlLowVal) ) ;	// Integration Low
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4, &(StMeasValueB.StUllnVal.UlHigVal) ) ;	// Integration Hig
	pStMesRamB->SlMeasureAveValue = 
				(INT32)( (INT64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;	// Ave value
	
	return( 0 );
}


#if ((SELECT_VENDOR & 0x80 ) != 0x80)
//********************************************************************************
// Function Name 	: RunGea2
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Gyro Examination of Acceptance
// History			: First edition 						
//********************************************************************************
#define 	GEA_NUM2		2048			// 2048times
// level of judgement
#define		GEA_MAX_LVL		0x0A41			// 2030_87.5lsb/°/s    max 30°/s-p-p
#define		GEA_MIN_LVL		0x1482			// 2030_87.5lsb/°/s    min 60°/s-p-p
UINT8	RunGea2( UINT8 UcMode )
{
	INT32	SlMeasureParameterA , SlMeasureParameterB ;
	UINT8 	UcRst ;
	UINT16	UsGyrXval , UsGyrYval;
	INT32	SlMeasureParameterNum ;
	UINT8	UcStRd;
	UINT32	UlSwRd , UlGyroConfig ; //, UlGyCnt;
	
	stMesRam 		StMesRamA ;
	stMesRam 		StMesRamB ;
	
	UcRst = EXE_END ;
	
	OisDis() ;
	
	RamRead32A( GYRO_RAM_GYRO_Switch , &UlSwRd);
	RamWrite32A( GYRO_RAM_GYRO_Switch , UlSwRd & 0xFFFFFFFC ) ;
	
	RamWrite32A( CMD_GYRO_RD_ACCS , 0x1B000000 );
	UcStRd = 1;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
	RamRead32A( CMD_GYRO_RD_ACCS , &UlGyroConfig );		/* FS_SEL backup */
//TRACE("GYCONFIG = %08x \n",(UINT32)UlGyroConfig ) ;

////////// ////////// ////////// ////////// //////////

	RamWrite32A( CMD_GYRO_WR_ACCS , 0x1B180000 );		/* FS_SEL=3固定 & Disable Self Test */
	UcStRd = 1;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
	
	SlMeasureParameterNum	=	GEA_NUM2 ;					// Measurement times
	SlMeasureParameterA		=	GYRO_RAM_GX_ADIDAT ;		// Set Measure RAM Address
	SlMeasureParameterB		=	GYRO_RAM_GY_ADIDAT ;		// Set Measure RAM Address
	
	MesRam( SlMeasureParameterA, SlMeasureParameterB, SlMeasureParameterNum, &StMesRamA, &StMesRamB );
	
	if( UcMode == GEA_MINMAX_MODE ){	// min, max mode
		
//TRACE("GX [max] = %08x, [min] = %08xh \n", (unsigned int)StMesRamA.SlMeasureMaxValue, (unsigned int)StMesRamA.SlMeasureMinValue) ;
//TRACE("GY [max] = %08x, [min] = %08xh \n", (unsigned int)StMesRamB.SlMeasureMaxValue, (unsigned int)StMesRamB.SlMeasureMinValue) ;

//TRACE("ABS_GX [max] = %08x, [min] = %08xh \n", (unsigned int)abs(StMesRamA.SlMeasureMaxValue), (unsigned int)abs(StMesRamA.SlMeasureMinValue)) ;
//TRACE("ABS_GY [max] = %08x, [min] = %08xh \n", (unsigned int)abs(StMesRamB.SlMeasureMaxValue), (unsigned int)abs(StMesRamB.SlMeasureMinValue)) ;
		
		// X
		if( abs(StMesRamA.SlMeasureMaxValue) >= abs(StMesRamA.SlMeasureMinValue) ) {
			UsGyrXval = (UINT16)( abs(StMesRamA.SlMeasureMaxValue) >> 16 );		// max value
		}
		else{
			UsGyrXval = (UINT16)( abs(StMesRamA.SlMeasureMinValue) >> 16 );		// max value
		}
		
		// Y
		if( abs(StMesRamB.SlMeasureMaxValue) >= abs(StMesRamB.SlMeasureMinValue) ) {
			UsGyrYval = (UINT16)( abs(StMesRamB.SlMeasureMaxValue) >> 16 );		// max value
		}
		else{
			UsGyrYval = (UINT16)( abs(StMesRamB.SlMeasureMinValue) >> 16 );		// max value
		}
		
	}
	else{								// mean mode
		
//TRACE("GX [ave] = %08xh \n", (UINT32)StMesRamA.SlMeasureAveValue) ;
//TRACE("GY [ave] = %08xh \n", (UINT32)StMesRamB.SlMeasureAveValue) ;
		
//TRACE("ABS_GX [ave] = %08xh \n", (UINT32)abs(StMesRamA.SlMeasureAveValue)) ;
//TRACE("ABS_GY [ave] = %08xh \n", (UINT32)abs(StMesRamB.SlMeasureAveValue)) ;
		
		// X
		UsGyrXval = (UINT16)( abs(StMesRamA.SlMeasureAveValue) >> 16 );		// ave value
		
		// Y
		UsGyrYval = (UINT16)( abs(StMesRamB.SlMeasureAveValue) >> 16 );		// ave value
		
	}
		
//TRACE("UsGyrXval = %04x\n", UsGyrXval ) ;
//TRACE("UsGyrYval = %04x\n", UsGyrYval ) ;
		
	if( UsGyrXval > GEA_MAX_LVL ) {
		UcRst = UcRst | EXE_GXABOVE ;
	}
	if( UsGyrYval > GEA_MAX_LVL ) {
		UcRst = UcRst | EXE_GYABOVE ;
	}
	
	if( StMesRamA.SlMeasureMinValue == StMesRamA.SlMeasureMaxValue ){
		UcRst = UcRst | EXE_GXABOVE ;
	}
	if( StMesRamB.SlMeasureMinValue == StMesRamB.SlMeasureMaxValue ){
		UcRst = UcRst | EXE_GYABOVE ;
	}

////////// ////////// ////////// ////////// //////////

	RamWrite32A( CMD_GYRO_WR_ACCS , 0x1BD80000 );		/* FS_SEL=3固定 & Enable Self Test */
	UcStRd = 1;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
	
	WitTim( 50 ) ;					/* 50ms*/
	
	SlMeasureParameterNum	=	GEA_NUM2 ;					// Measurement times
	SlMeasureParameterA		=	GYRO_RAM_GX_ADIDAT ;		// Set Measure RAM Address
	SlMeasureParameterB		=	GYRO_RAM_GY_ADIDAT ;		// Set Measure RAM Address

	MesRam( SlMeasureParameterA, SlMeasureParameterB, SlMeasureParameterNum, &StMesRamA, &StMesRamB );

	if( UcMode == GEA_MINMAX_MODE ){	// min, max mode

//TRACE("GX [max] = %08x, [min] = %08xh \n", (UINT32)StMesRamA.SlMeasureMaxValue, (UINT32)StMesRamA.SlMeasureMinValue) ;
//TRACE("GY [max] = %08x, [min] = %08xh \n", (UINT32)StMesRamB.SlMeasureMaxValue, (UINT32)StMesRamB.SlMeasureMinValue) ;

		// X
		UsGyrXval = (UINT16)( StMesRamA.SlMeasureMinValue >> 16 );		// min value
		
		// Y
		UsGyrYval = (UINT16)( StMesRamB.SlMeasureMinValue >> 16 );		// min value
	
	}
	else{								// mean mode
//TRACE("GX [ave] = %08xh \n", (UINT32)StMesRamA.SlMeasureAveValue) ;
//TRACE("GY [ave] = %08xh \n", (UINT32)StMesRamB.SlMeasureAveValue) ;
		
		// X
		UsGyrXval = (UINT16)( StMesRamA.SlMeasureAveValue >> 16 );		// ave value
		
		// Y
		UsGyrYval = (UINT16)( StMesRamB.SlMeasureAveValue >> 16 );		// ave value
		
	}
	
//TRACE("UsGyrXval = %04x\n", UsGyrXval ) ;
//TRACE("UsGyrYval = %04x\n", UsGyrYval ) ;
	
	if( UsGyrXval < GEA_MIN_LVL ) {
		UcRst = UcRst | EXE_GXBELOW ;
	}
	if( UsGyrYval < GEA_MIN_LVL ) {
		UcRst = UcRst | EXE_GYBELOW ;
	}
	
	if( StMesRamA.SlMeasureMinValue == StMesRamA.SlMeasureMaxValue ){
		UcRst = UcRst | EXE_GXBELOW ;
	}
	if( StMesRamB.SlMeasureMinValue == StMesRamB.SlMeasureMaxValue ){
		UcRst = UcRst | EXE_GYBELOW ;
	}
	
	RamWrite32A( CMD_GYRO_WR_ACCS , 0x1B000000 | ( UlGyroConfig >> 8));		/* 元の設定値に戻す */
	UcStRd = 1;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
	
//TRACE("GYCONFIG = %08x \n",(UINT32)(0x1B000000 | ( UlGyroConfig >> 8)) ) ;
	
	RamWrite32A( GYRO_RAM_GYRO_Switch , UlSwRd | 0x00000001 ) ;
//TRACE("UcRst = %02x\n", UcRst ) ;
	
	return( UcRst ) ;
}
#endif //((SELECT_VENDOR & 0x80 ) != 0x80)


//********************************************************************************
// Function Name 	: SetAngleCorrection
// Retun Value		: True/Fail
// Argment Value	: 
// Explanation		: Angle Correction
// History			: First edition
//********************************************************************************
/*  bit7  	HX GYR			Hall X  と同方向のGyro信号がGX?               0:GX  1:GY  */
/*  bit6  	HX GYR pol		Hall X+ と同方向のGyro信号がX+とG+で同方向?   0:NEG 1:POS */
/*  bit5  	HY GYR pol		Hall Y+ と同方向のGyro信号がY+とG+で同方向?   0:NEG 1:POS */
/*  bit4  	GZ pol			基本極性に対してGyroZ信号が同方向?            0:NEG 1:POS */
/*  bit3  	HX ACL			Hall X  と同方向のAccl信号がAX?               0:AX  1:AY  */
/*  bit2  	HX ACL pol		Hall X+ と同方向のAccl信号がX+とA+で同方向?   0:NEG 1:POS */
/*  bit1  	HY ACL pol		Hall Y+ と同方向のAccl信号がY+とA+で同方向?   0:NEG 1:POS */
/*  bit0  	AZ pol			基本極性に対してAcclZ信号が同方向?            0:NEG 1:POS */
                      //   top0°btm0°//
const UINT8 PACT0Tbl[] = { 0xFF, 0xFF };	/* Dummy table */
const UINT8 PACT1Tbl[] = { 0x20, 0xDF };	
const UINT8 PACT2Tbl[] = { 0x26, 0xD9 };	/* ACT_45DEG */


UINT8 SetAngleCorrection( float DegreeGap, UINT8 SelectAct, UINT8 Arrangement )
{
	double OffsetAngle = 0.0f;
	INT32 Slgx45x = 0, Slgx45y = 0;
	INT32 Slgy45y = 0, Slgy45x = 0;
	
//	INT32 Slgx45m = 0, Slgx45s = 0;
//	INT32 Slgy45m = 0, Slgy45s = 0;
	UINT8	UcCnvF = 0;

	if( ( DegreeGap > 180.0f) || ( DegreeGap < -180.0f ) ) return ( 1 );
	if( Arrangement >= 2 ) return ( 1 );

/************************************************************************/
/*      	Gyro angle correction										*/
/************************************************************************/
	switch(SelectAct) {
		case ACT_45DEG :
			OffsetAngle = (double)( 45.0f + DegreeGap ) * 3.141592653589793238 / 180.0f ;
			UcCnvF = PACT2Tbl[ Arrangement ];
			break;
		case ACT_SO2820 :
		case ACT_SO3600 :
			OffsetAngle = (double)( DegreeGap ) * 3.141592653589793238 / 180.0f ;
			UcCnvF = PACT1Tbl[ Arrangement ];
			break;
		default :
			break;
	}
	
	SetGyroCoef( UcCnvF );
	SetAccelCoef( UcCnvF );

	//***********************************************//
	// Gyro & Accel rotation correction
	// Zero Servo angle correction
	//***********************************************//
	Slgx45x = (INT32)( cos( OffsetAngle )*2147483647.0);
	Slgx45y = (INT32)(-sin( OffsetAngle )*2147483647.0);
	Slgy45y = (INT32)( cos( OffsetAngle )*2147483647.0);
	Slgy45x = (INT32)( sin( OffsetAngle )*2147483647.0);
	
	RamWrite32A( GyroFilterTableX_gx45x , 			(UINT32)Slgx45x );
	RamWrite32A( GyroFilterTableX_gx45y , 			(UINT32)Slgx45y );
	RamWrite32A( GyroFilterTableY_gy45y , 			(UINT32)Slgy45y );
	RamWrite32A( GyroFilterTableY_gy45x , 			(UINT32)Slgy45x );
#ifdef ACCEL_SERVO
	RamWrite32A( Accl45Filter_XAmain , 				(UINT32)Slgx45x );
	RamWrite32A( Accl45Filter_XAsub  , 				(UINT32)Slgx45y );
	RamWrite32A( Accl45Filter_YAmain , 				(UINT32)Slgy45y );
	RamWrite32A( Accl45Filter_YAsub  , 				(UINT32)Slgy45x );
#endif
	
#ifdef ZERO_SERVO
	RamWrite32A( ZeroServoFilterTableX_g45main 	, 	(UINT32)Slgx45x );
	RamWrite32A( ZeroServoFilterTableX_g45sub 	, 	(UINT32)Slgx45y );
	RamWrite32A( ZeroServoFilterTableY_g45main 	, 	(UINT32)Slgy45y );
	RamWrite32A( ZeroServoFilterTableY_g45sub 	, 	(UINT32)Slgy45x );
#endif
	
	return ( 0 );
}

void	SetGyroCoef( UINT8 UcCnvF )
{
	INT32 Slgxx = 0, Slgxy = 0;
	INT32 Slgyy = 0, Slgyx = 0;
	INT32 Slgzp = 0;
	/************************************************/
	/*  signal convet								*/
	/************************************************/
	switch( UcCnvF & 0xE0 ){
		/* HX <== GX , HY <== GY */
	case 0x00:
		Slgxx = 0x7FFFFFFF ;	Slgxy = 0x00000000 ;	Slgyy = 0x7FFFFFFF ;	Slgyx = 0x00000000 ;	break;	//HX<==GX(NEG), HY<==GY(NEG)
	case 0x20:
		Slgxx = 0x7FFFFFFF ;	Slgxy = 0x00000000 ;	Slgyy = 0x80000001 ;	Slgyx = 0x00000000 ;	break;	//HX<==GX(NEG), HY<==GY(POS)
	case 0x40:
		Slgxx = 0x80000001 ;	Slgxy = 0x00000000 ;	Slgyy = 0x7FFFFFFF ;	Slgyx = 0x00000000 ;	break;	//HX<==GX(POS), HY<==GY(NEG)
	case 0x60:
		Slgxx = 0x80000001 ;	Slgxy = 0x00000000 ;	Slgyy = 0x80000001 ;	Slgyx = 0x00000000 ;	break;	//HX<==GX(POS), HY<==GY(POS)
		/* HX <== GY , HY <== GX */
	case 0x80:
		Slgxx = 0x00000000 ;	Slgxy = 0x7FFFFFFF ;	Slgyy = 0x00000000 ;	Slgyx = 0x7FFFFFFF ;	break;	//HX<==GY(NEG), HY<==GX(NEG)
	case 0xA0:
		Slgxx = 0x00000000 ;	Slgxy = 0x7FFFFFFF ;	Slgyy = 0x00000000 ;	Slgyx = 0x80000001 ;	break;	//HX<==GY(NEG), HY<==GX(POS)
	case 0xC0:
		Slgxx = 0x00000000 ;	Slgxy = 0x80000001 ;	Slgyy = 0x00000000 ;	Slgyx = 0x7FFFFFFF ;	break;	//HX<==GY(POS), HY<==GX(NEG)
	case 0xE0:
		Slgxx = 0x00000000 ;	Slgxy = 0x80000001 ;	Slgyy = 0x00000000 ;	Slgyx = 0x80000001 ;	break;	//HX<==GY(NEG), HY<==GX(NEG)
	}
	switch( UcCnvF & 0x10 ){
	case 0x00:
		Slgzp = 0x7FFFFFFF ;	break;																			//GZ(POS)
	case 0x10:
		Slgzp = 0x80000001 ;	break;																			//GZ(NEG)
	}
	RamWrite32A( GCNV_XX , (UINT32)Slgxx );
	RamWrite32A( GCNV_XY , (UINT32)Slgxy );
	RamWrite32A( GCNV_YY , (UINT32)Slgyy );
	RamWrite32A( GCNV_YX , (UINT32)Slgyx );
	RamWrite32A( GCNV_ZP , (UINT32)Slgzp );
}

void	SetAccelCoef( UINT8 UcCnvF )
{
	INT32 Slaxx = 0, Slaxy = 0;
	INT32 Slayy = 0, Slayx = 0;
	INT32 Slazp = 0;
	
	switch( UcCnvF & 0x0E ){
		/* HX <== AX , HY <== AY */
	case 0x00:
		Slaxx = 0x7FFFFFFF ;	Slaxy = 0x00000000 ;	Slayy = 0x7FFFFFFF ;	Slayx = 0x00000000 ;	break;	//HX<==AX(NEG), HY<==AY(NEG)
	case 0x02:
		Slaxx = 0x7FFFFFFF ;	Slaxy = 0x00000000 ;	Slayy = 0x80000001 ;	Slayx = 0x00000000 ;	break;	//HX<==AX(NEG), HY<==AY(POS)
	case 0x04:
		Slaxx = 0x80000001 ;	Slaxy = 0x00000000 ;	Slayy = 0x7FFFFFFF ;	Slayx = 0x00000000 ;	break;	//HX<==AX(POS), HY<==AY(NEG)
	case 0x06:
		Slaxx = 0x80000001 ;	Slaxy = 0x00000000 ;	Slayy = 0x80000001 ;	Slayx = 0x00000000 ;	break;	//HX<==AX(POS), HY<==AY(POS)
		/* HX <== AY , HY <== AX */
	case 0x08:
		Slaxx = 0x00000000 ;	Slaxy = 0x7FFFFFFF ;	Slayy = 0x00000000 ;	Slayx = 0x7FFFFFFF ;	break;	//HX<==AY(NEG), HY<==AX(NEG)
	case 0x0A:
		Slaxx = 0x00000000 ;	Slaxy = 0x7FFFFFFF ;	Slayy = 0x00000000 ;	Slayx = 0x80000001 ;	break;	//HX<==AY(NEG), HY<==AX(POS)
	case 0x0C:
		Slaxx = 0x00000000 ;	Slaxy = 0x80000001 ;	Slayy = 0x00000000 ;	Slayx = 0x7FFFFFFF ;	break;	//HX<==AY(POS), HY<==AX(NEG)
	case 0x0E:
		Slaxx = 0x00000000 ;	Slaxy = 0x80000001 ;	Slayy = 0x00000000 ;	Slayx = 0x80000001 ;	break;	//HX<==AY(NEG), HY<==AX(NEG)
	}
	switch( UcCnvF & 0x01 ){
	case 0x00:
		Slazp = 0x7FFFFFFF ;	break;																			//AZ(POS)
	case 0x01:
		Slazp = 0x80000001 ;	break;																			//AZ(NEG)
	}
	RamWrite32A( ACNV_XX , (UINT32)Slaxx );
	RamWrite32A( ACNV_XY , (UINT32)Slaxy );
	RamWrite32A( ACNV_YY , (UINT32)Slayy );
	RamWrite32A( ACNV_YX , (UINT32)Slayx );
	RamWrite32A( ACNV_ZP , (UINT32)Slazp );
}

//********************************************************************************
// Function Name 	: SetGyroAccelCoef
// Retun Value		: non
// Argment Value	: 
// Explanation		: Set Gyro Coef and Accel Coef
// History			: First edition
//********************************************************************************
void SetGyroAccelCoef( UINT8 SelectAct ){
	switch(SelectAct) {
#ifndef UP_SIDE_GYRO
		case ACT_SO2820 :
			RamWrite32A( GCNV_XX , (UINT32)0x00000000 );
			RamWrite32A( GCNV_XY , (UINT32)0x7FFFFFFF );
			RamWrite32A( GCNV_YY , (UINT32)0x00000000 );
			RamWrite32A( GCNV_YX , (UINT32)0x7FFFFFFF );
			RamWrite32A( GCNV_ZP , (UINT32)0x7FFFFFFF );
			
			RamWrite32A( ACNV_XX , (UINT32)0x00000000 );
			RamWrite32A( ACNV_XY , (UINT32)0x7FFFFFFF );
			RamWrite32A( ACNV_YY , (UINT32)0x00000000 );
			RamWrite32A( ACNV_YX , (UINT32)0x7FFFFFFF );
			RamWrite32A( ACNV_ZP , (UINT32)0x7FFFFFFF );
			break;
		case ACT_SO3600 :
			RamWrite32A( GCNV_XX , (UINT32)0x80000001 );
			RamWrite32A( GCNV_XY , (UINT32)0x00000000 );
			RamWrite32A( GCNV_YY , (UINT32)0x80000001 );
			RamWrite32A( GCNV_YX , (UINT32)0x00000000 );
			RamWrite32A( GCNV_ZP , (UINT32)0x7FFFFFFF );

			RamWrite32A( ACNV_XX , (UINT32)0x7FFFFFFF );
			RamWrite32A( ACNV_XY , (UINT32)0x00000000 );
			RamWrite32A( ACNV_YY , (UINT32)0x80000001 );
			RamWrite32A( ACNV_YX , (UINT32)0x00000000 );
			RamWrite32A( ACNV_ZP , (UINT32)0x7FFFFFFF );
		break;
#else
 #if USE_BOSCH
		case ACT_SO2820 :
			RamWrite32A( GCNV_XX , (UINT32)0x00000000 );
			RamWrite32A( GCNV_XY , (UINT32)0x80000001 );
			RamWrite32A( GCNV_YY , (UINT32)0x00000000 );
			RamWrite32A( GCNV_YX , (UINT32)0x7FFFFFFF );
			RamWrite32A( GCNV_ZP , (UINT32)0x80000001 );
			
			RamWrite32A( ACNV_XX , (UINT32)0x00000000 );
			RamWrite32A( ACNV_XY , (UINT32)0x7FFFFFFF );
			RamWrite32A( ACNV_YY , (UINT32)0x00000000 );
			RamWrite32A( ACNV_YX , (UINT32)0x7FFFFFFF );
			RamWrite32A( ACNV_ZP , (UINT32)0x7FFFFFFF );
			break;
		case ACT_SO3600 :
			RamWrite32A( GCNV_XX , (UINT32)0x80000001 );
			RamWrite32A( GCNV_XY , (UINT32)0x00000000 );
			RamWrite32A( GCNV_YY , (UINT32)0x7FFFFFFF );
			RamWrite32A( GCNV_YX , (UINT32)0x00000000 );
			RamWrite32A( GCNV_ZP , (UINT32)0x80000001 );

			RamWrite32A( ACNV_XX , (UINT32)0x80000001 );
			RamWrite32A( ACNV_XY , (UINT32)0x00000000 );
			RamWrite32A( ACNV_YY , (UINT32)0x80000001 );
			RamWrite32A( ACNV_YX , (UINT32)0x00000000 );
			RamWrite32A( ACNV_ZP , (UINT32)0x7FFFFFFF );
		break;
  #else
		case ACT_SO2820 :
			RamWrite32A( GCNV_XX , (UINT32)0x00000000 );
			RamWrite32A( GCNV_XY , (UINT32)0x7FFFFFFF );
			RamWrite32A( GCNV_YY , (UINT32)0x00000000 );
			RamWrite32A( GCNV_YX , (UINT32)0x80000001 );
			RamWrite32A( GCNV_ZP , (UINT32)0x80000001 );
			
			RamWrite32A( ACNV_XX , (UINT32)0x7FFFFFFF );
			RamWrite32A( ACNV_XY , (UINT32)0x00000000 );
			RamWrite32A( ACNV_YY , (UINT32)0x7FFFFFFF );
			RamWrite32A( ACNV_YX , (UINT32)0x00000000 );
			RamWrite32A( ACNV_ZP , (UINT32)0x7FFFFFFF );
			break;
		case ACT_SO3600 :
			RamWrite32A( GCNV_XX , (UINT32)0x80000001 );
			RamWrite32A( GCNV_XY , (UINT32)0x00000000 );
			RamWrite32A( GCNV_YY , (UINT32)0x80000001 );
			RamWrite32A( GCNV_YX , (UINT32)0x00000000 );
			RamWrite32A( GCNV_ZP , (UINT32)0x80000001 );

			RamWrite32A( ACNV_XX , (UINT32)0x00000000 );
			RamWrite32A( ACNV_XY , (UINT32)0x7FFFFFFF );
			RamWrite32A( ACNV_YY , (UINT32)0x00000000 );
			RamWrite32A( ACNV_YX , (UINT32)0x80000001 );
			RamWrite32A( ACNV_ZP , (UINT32)0x7FFFFFFF );
		break;
  #endif	// USE_BOSCH
#endif		// UP_SIDE_GYRO
	}
}

//********************************************************************************
// Function Name 	: MeasGain
// Retun Value		: Hall amp & Sine amp
// Argment Value	: X,Y Direction, Freq
// Explanation		: Measuring Hall Paek To Peak
// History			: First edition 						
//********************************************************************************
#define	FS4TIME	(UINT32)0x000119B3		// 18028.8 * 4
#define	FRQOFST	(UINT32)0x0001D14A		// 80000000h / 18028.8

UINT32	MeasGain ( UINT16	UcDirSel, UINT16	UsMeasFreq , UINT32 UlMesAmp )
{
	INT32			SlMeasureParameterA , SlMeasureParameterB ;
	INT32			SlMeasureParameterNum , SlSineWaveOffset;
	UnllnVal		StMeasValueA  , StMeasValueB ;
	UINT32	UlReturnVal;

	StMeasValueA.UllnValue = 0;
	StMeasValueB.UllnValue = 0;
	SlMeasureParameterNum	=	(INT32)( FS4TIME / (UINT32)UsMeasFreq) * 2;	// 
	
	if( UcDirSel == X_DIR ) {								// X axis
		SlMeasureParameterA		=	HALL_RAM_HXOUT0 ;		// Set Measure RAM Address
		SlMeasureParameterB		=	HallFilterD_HXDAZ1 ;	// Set Measure RAM Address
	} else if( UcDirSel == Y_DIR ) {						// Y axis
		SlMeasureParameterA		=	HALL_RAM_HYOUT0 ;		// Set Measure RAM Address
		SlMeasureParameterB		=	HallFilterD_HYDAZ1 ;	// Set Measure RAM Address
	}
	SetSinWavGenInt();
	
	SlSineWaveOffset = (INT32)( FRQOFST * (UINT32)UsMeasFreq );
	RamWrite32A( SinWave_Offset		,	SlSineWaveOffset ) ;		// Freq Setting = Freq * 80000000h / Fs	

	RamWrite32A( SinWave_Gain		,	UlMesAmp ) ;			// Set Sine Wave Gain

	RamWrite32A( SinWaveC_Regsiter	,	0x00000001 ) ;				// Sine Wave Start
	if( UcDirSel == X_DIR ) {
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_RAM_HXOFF1 ) ;	// Set Sine Wave Input RAM
	}else if( UcDirSel == Y_DIR ){
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_RAM_HYOFF1 ) ;	// Set Sine Wave Input RAM
	}
	
	MesFil2( UsMeasFreq ) ;					// Filter setting for measurement

	MeasureStart2( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB , 8000/UsMeasFreq ) ;			// Start measure
	
	MeasureWait() ;						// Wait complete of measurement
	
	RamWrite32A( SinWaveC_Regsiter	,	0x00000000 ) ;								// Sine Wave Stop
	
	if( UcDirSel == X_DIR ) {
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)0x00000000 ) ;	// Set Sine Wave Input RAM
		RamWrite32A( HALL_RAM_HXOFF1		,	0x00000000 ) ;				// DelayRam Clear
	}else if( UcDirSel == Y_DIR ){
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)0x00000000 ) ;	// Set Sine Wave Input RAM
		RamWrite32A( HALL_RAM_HYOFF1		,	0x00000000 ) ;				// DelayRam Clear
	}

	RamRead32A( StMeasFunc_MFA_LLiAbsInteg1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	
	RamRead32A( StMeasFunc_MFA_LLiAbsInteg1 + 4 	, &StMeasValueA.StUllnVal.UlHigVal ) ;
	RamRead32A( StMeasFunc_MFB_LLiAbsInteg2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	
	RamRead32A( StMeasFunc_MFB_LLiAbsInteg2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;

	
	UlReturnVal = (INT32)((INT64)StMeasValueA.UllnValue * 100 / (INT64)StMeasValueB.UllnValue  ) ;


	return( UlReturnVal ) ;
}
//********************************************************************************
// Function Name 	: MesFil2
// Retun Value		: NON
// Argment Value	: Measure Filter Mode
// Explanation		: Measure Filter Setting Function
// History			: First edition 		
//********************************************************************************
#define	DivOffset	5741.65f		/* 18028.8/3.14 */

void	MesFil2( UINT16	UsMesFreq )		
{
	UINT32	UlMeasFilA1 , UlMeasFilB1 , UlMeasFilC1 , UlTempval ;
	UINT32	UlMeasFilA2 , UlMeasFilC2 ;
		
	UlTempval = (UINT32)(2147483647 * (float)UsMesFreq / ((float)UsMesFreq + DivOffset ));
	UlMeasFilA1	=	0x7fffffff - UlTempval;
	UlMeasFilB1	=	~UlMeasFilA1 + 0x00000001;	
	UlMeasFilC1	=	0x7FFFFFFF - ( UlTempval << 1 ) ;

	UlMeasFilA2	=	UlTempval ;	
	UlMeasFilC2	=	UlMeasFilC1 ;

	
	RamWrite32A ( MeasureFilterA_Coeff_a1	, UlMeasFilA1 ) ;
	RamWrite32A ( MeasureFilterA_Coeff_b1	, UlMeasFilB1 ) ;
	RamWrite32A ( MeasureFilterA_Coeff_c1	, UlMeasFilC1 ) ;

	RamWrite32A ( MeasureFilterA_Coeff_a2	, UlMeasFilA2 ) ;
	RamWrite32A ( MeasureFilterA_Coeff_b2	, UlMeasFilA2 ) ;
	RamWrite32A ( MeasureFilterA_Coeff_c2	, UlMeasFilC2 ) ;

	RamWrite32A ( MeasureFilterB_Coeff_a1	, UlMeasFilA1 ) ;
	RamWrite32A ( MeasureFilterB_Coeff_b1	, UlMeasFilB1 ) ;
	RamWrite32A ( MeasureFilterB_Coeff_c1	, UlMeasFilC1 ) ;

	RamWrite32A ( MeasureFilterB_Coeff_a2	, UlMeasFilA2 ) ;
	RamWrite32A ( MeasureFilterB_Coeff_b2	, UlMeasFilA2 ) ;
	RamWrite32A ( MeasureFilterB_Coeff_c2	, UlMeasFilC2 ) ;
}

//********************************************************************************
// Function Name 	: MeasureStart2
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure start setting Function
// History			: First edition 						
//********************************************************************************
void	MeasureStart2( INT32 SlMeasureParameterNum , INT32 SlMeasureParameterA , INT32 SlMeasureParameterB , UINT16 UsTime )
{
	MemoryClear( StMeasFunc_SiSampleNum , sizeof( MeasureFunction_Type ) ) ;
	RamWrite32A( StMeasFunc_MFA_SiMax1	 , 0x80000001 ) ;					// Set Min 
	RamWrite32A( StMeasFunc_MFB_SiMax2	 , 0x80000001 ) ;					// Set Min 
	RamWrite32A( StMeasFunc_MFA_SiMin1	 , 0x7FFFFFFF ) ;					// Set Max 
	RamWrite32A( StMeasFunc_MFB_SiMin2	 , 0x7FFFFFFF ) ;					// Set Max 
	
	SetTransDataAdr( StMeasFunc_MFA_PiMeasureRam1	 , ( UINT32 )SlMeasureParameterA ) ;		// Set Measure Filter A Ram Address
	SetTransDataAdr( StMeasFunc_MFB_PiMeasureRam2	 , ( UINT32 )SlMeasureParameterB ) ;		// Set Measure Filter B Ram Address
	RamWrite32A( StMeasFunc_SiSampleNum	 , 0 ) ;													// Clear Measure Counter 
	ClrMesFil() ;						// Clear Delay Ram
	RamWrite32A( StMeasFunc_SiSampleMax	 , SlMeasureParameterNum ) ;						// Set Measure Max Number
	SetWaitTime(UsTime) ;

}


//********************************************************************************
// Function Name 	: LinearityCalculation
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure start setting Function
// History			: First edition 						
//********************************************************************************
void	LinearityCalculation( void )
{
	UINT8  tempL[32];
//	UINT8  tempC[10];
	UINT16 tblX[7],tblY[7];
	INT16  stpx,stpy;
//	UINT16 tblC[5];
	UINT8	i,n,cnt;
	UINT16  adr;
	INT32	dacx[7],dacy[7];
	INT16	pixx[7],pixy[7];
	INT16	cfax[6],cfbx[6],cfzx[5];
	INT16	cfay[6],cfby[6],cfzy[5];
	float	cffax[6];
	float	cffay[6];
	
	ReadE2Prom( EEPROM_Calibration_Status_MSB, &cnt );
TRACE("E2prom Read 0x19 = %02x   &  %02x  \n", cnt , (UINT8)(HLLN_CALB_FLG>>8) );
	if( cnt & (UINT8)(HLLN_CALB_FLG>>8) ){
		return;
	}
	
	BurstReadE2Prom( EEPROM_POSITION_1X_LSB  , tempL, 32 );
//	BurstReadE2Prom( EEPROM_CROSSTALK_XX_LSB , tempC, 10 );
	
TRACE("\n E2prom POSITION Read \n" );
TRACE(" %02xh %02xh %02xh %02xh %02xh %02xh %02xh %02xh \n",tempL[0],tempL[1],tempL[2],tempL[3],tempL[4],tempL[5],tempL[6],tempL[7]) ;
TRACE(" %02xh %02xh %02xh %02xh %02xh %02xh %02xh %02xh \n",tempL[8],tempL[9],tempL[10],tempL[11],tempL[12],tempL[13],tempL[14],tempL[15]) ;
TRACE(" %02xh %02xh %02xh %02xh %02xh %02xh %02xh %02xh \n",tempL[16],tempL[17],tempL[18],tempL[19],tempL[20],tempL[21],tempL[22],tempL[23]) ;
TRACE(" %02xh %02xh %02xh %02xh %02xh %02xh %02xh %02xh \n",tempL[24],tempL[25],tempL[26],tempL[27],tempL[28],tempL[29],tempL[30],tempL[31]) ;
TRACE("\n E2prom CROSS TALK Read \n" );
//TRACE(" %02xh %02xh %02xh %02xh %02xh %02xh %02xh %02xh \n",tempC[0],tempC[1],tempC[2],tempC[3],tempC[4],tempC[5],tempC[6],tempC[7]) ;
//TRACE(" %02xh %02xh \n",tempC[8],tempC[9]) ;

	/****** Linearity ******/
	for( i=0 , n=0 ; n<7 ; n++ ){
		tblX[n] = tempL[i] + (tempL[i+1]<<8);
		i += 2;
		tblY[n] = tempL[i] + (tempL[i+1]<<8);
		i += 2;
	}
	stpx = (INT16)(tempL[i] + (tempL[i+1]<<8));
	i += 2;
	stpy = (INT16)(tempL[i] + (tempL[i+1]<<8));
TRACE("\n POSITION TBL \n" );
TRACE("[x] %04xh %04xh %04xh %04xh %04xh %04xh %04xh \n",tblX[0],tblX[1],tblX[2],tblX[3],tblX[4],tblX[5],tblX[6]) ;
TRACE("[y] %04xh %04xh %04xh %04xh %04xh %04xh %04xh \n",tblY[0],tblY[1],tblY[2],tblY[3],tblY[4],tblY[5],tblY[6]) ;
TRACE("[s] %04xh %04xh \n",stpx,stpy) ;
	
	for( i=0 ; i<7 ; i++ ){
		dacx[i] = (( i - 3 ) * stpx)<<4;	/* 2^16/4096 = 2^4 */
		dacy[i] = (( i - 3 ) * stpy)<<4;	/* 2^16/4096 = 2^4 */
		pixx[i] = tblX[i] - tblX[3];
		pixy[i] = tblY[i] - tblY[3];
	}
	for( i=0 ; i<6 ; i++ ){
		if(i == 3){
//			cfax[i] = (INT16)((float)pixx[i+1] / (float)dacx[i+1] * 524287.0f);
//			cfay[i] = (INT16)((float)pixy[i+1] / (float)dacy[i+1] * 524287.0f);
			cffax[i] = ((float)pixx[i+1] / (float)dacx[i+1] * 524287.0f);
			cffay[i] = ((float)pixy[i+1] / (float)dacy[i+1] * 524287.0f);
			cfax[i] = (INT16)cffax[i];
			cfay[i] = (INT16)cffay[i];
			cfbx[i] = (INT16)0;
			cfby[i] = (INT16)0;
		}else{
//			cfax[i] = (INT16)(( dacx[i] - dacx[i+1] ) / ( pixx[i] - pixx[i+1] ));
//			cfay[i] = (INT16)(( dacy[i] - dacy[i+1] ) / ( pixy[i] - pixy[i+1] ));
			cffax[i] = (float)( dacx[i] - dacx[i+1] ) / (float)( pixx[i] - pixx[i+1] );
			cffay[i] = (float)( dacy[i] - dacy[i+1] ) / (float)( pixy[i] - pixy[i+1] );
			cfax[i] = (INT16)cffax[i];
			cfay[i] = (INT16)cffay[i];
			if(i == 2){
				cfbx[i] = (INT16)0;
				cfby[i] = (INT16)0;
			}else{
//				cfbx[i] = (INT16)( dacx[i] - (INT32)cfax[i] * (INT32)pixx[i] );
//				cfby[i] = (INT16)( dacy[i] - (INT32)cfay[i] * (INT32)pixy[i] );
				cfbx[i] = (INT16)( (float)dacx[i] - cffax[i] * (float)pixx[i] );
				cfby[i] = (INT16)( (float)dacy[i] - cffay[i] * (float)pixy[i] );
			}
		}
		if(i<5){
			cfzx[i] = pixx[i+1];
			cfzy[i] = pixy[i+1];
		}
	}
TRACE("\n CALCULATE \n" );
TRACE(" [%08xh , %04xh] [%08xh , %04xh]\n",(int)dacx[0],pixx[0],(int)dacy[0],pixy[0]) ;
TRACE(" [%08xh , %04xh] [%08xh , %04xh]\n",(int)dacx[1],pixx[1],(int)dacy[1],pixy[1]) ;
TRACE(" [%08xh , %04xh] [%08xh , %04xh]\n",(int)dacx[2],pixx[2],(int)dacy[2],pixy[2]) ;
TRACE(" [%08xh , %04xh] [%08xh , %04xh]\n",(int)dacx[3],pixx[3],(int)dacy[3],pixy[3]) ;
TRACE(" [%08xh , %04xh] [%08xh , %04xh]\n",(int)dacx[4],pixx[4],(int)dacy[4],pixy[4]) ;
TRACE(" [%08xh , %04xh] [%08xh , %04xh]\n",(int)dacx[5],pixx[5],(int)dacy[5],pixy[5]) ;
TRACE(" [%08xh , %04xh] [%08xh , %04xh]\n\n",(int)dacx[6],pixx[6],(int)dacy[6],pixy[6]) ;
TRACE(" [%04xh , %04xh , %04xh] [%04xh , %04xh , %04xh]\n",cfzx[0],cfax[0],cfbx[0],cfzy[0],cfay[0],cfby[0]) ;
TRACE(" [%04xh , %04xh , %04xh] [%04xh , %04xh , %04xh]\n",cfzx[1],cfax[1],cfbx[1],cfzy[1],cfay[1],cfby[1]) ;
TRACE(" [%04xh , %04xh , %04xh] [%04xh , %04xh , %04xh]\n",cfzx[2],cfax[2],cfbx[2],cfzy[2],cfay[2],cfby[2]) ;
TRACE(" [%04xh , %04xh , %04xh] [%04xh , %04xh , %04xh]\n",cfzx[3],cfax[3],cfbx[3],cfzy[3],cfay[3],cfby[3]) ;
TRACE(" [%04xh , %04xh , %04xh] [%04xh , %04xh , %04xh]\n",cfzx[4],cfax[4],cfbx[4],cfzy[4],cfay[4],cfby[4]) ;
TRACE(" [----- , %04xh , %04xh] [----- , %04xh , %04xh]\n"        ,cfax[5],cfbx[5]        ,cfay[5],cfby[5]) ;
	adr = HAL_LN_CORRECT;
	RamWrite32A( adr , (UINT32)( (UINT16)cfax[0] + ((UINT16)cfax[1]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfax[2] + ((UINT16)cfax[3]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfax[4] + ((UINT16)cfax[5]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfbx[0] + ((UINT16)cfbx[1]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfbx[2] + ((UINT16)cfbx[3]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfbx[4] + ((UINT16)cfbx[5]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfzx[0] + ((UINT16)cfzx[1]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfzx[2] + ((UINT16)cfzx[3]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfzx[4] + ((UINT16)cfay[0]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfay[1] + ((UINT16)cfay[2]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfay[3] + ((UINT16)cfay[4]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfay[5] + ((UINT16)cfby[0]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfby[1] + ((UINT16)cfby[2]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfby[3] + ((UINT16)cfby[4]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfby[5] + ((UINT16)cfzy[0]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfzy[1] + ((UINT16)cfzy[2]<<16 )));		adr += 0x0004;
	RamWrite32A( adr , (UINT32)( (UINT16)cfzy[3] + ((UINT16)cfzy[4]<<16 )));

	SetLinearityParameter();
	
//	/****** Cross talk ******/
//	for( i=0 , n=0 ; n<5 ; n++ ){
//		tblC[n] = tempC[i] + (tempC[i+1]<<8);
//		i += 2;
//	}
//TRACE("[XX] %04xh [XY]%04xh [SFT]%02xh\n",tblC[0],tblC[1],(unsigned char)(tblC[4]>>0)) ;
//TRACE("[YY] %04xh [YX]%04xh [SFT]%02xh\n",tblC[2],tblC[3],(unsigned char)(tblC[4]>>8)) ;
//	RamWrite32A( HF_hx45x , (UINT32)(tblC[0]<<16));
//	RamWrite32A( HF_hx45y , (UINT32)(tblC[1]<<16));
//	RamWrite32A( HF_hy45y , (UINT32)(tblC[2]<<16));
//	RamWrite32A( HF_hy45x , (UINT32)(tblC[3]<<16));
//	RamWrite32A( HF_ShiftX , (UINT32)(tblC[4]));
	
}	

void	SetLinearityParameter( void )
{
	UINT32 UlGyroSw;
	
	
	RamRead32A( GYRO_RAM_GYRO_Switch , &UlGyroSw );
	UlGyroSw	|=	0x00000008;
	RamWrite32A( GYRO_RAM_GYRO_Switch , UlGyroSw );
	
}

//********************************************************************************
// Function Name 	: CrosstalkCalculation
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure start setting Function
// History			: First edition
//********************************************************************************
void	CrosstalkCalculation( void )
{
	UINT16 tblC[5];
	UINT8  tempC[10];
	UINT8	i,n,cnt;
	
	ReadE2Prom( EEPROM_Calibration_Status_MSB, &cnt );
TRACE("E2prom Read 0x19 = %02x   &  %02x  \n", cnt , (UINT8)(MIXI_CALB_FLG>>8) );
	if( cnt & (UINT8)(MIXI_CALB_FLG>>8) ){
		return;
	}
	
	BurstReadE2Prom( EEPROM_CROSSTALK_XX_LSB , tempC, 10 );
	
TRACE("\n E2prom CROSS TALK Read \n" );
TRACE(" %02xh %02xh %02xh %02xh %02xh %02xh %02xh %02xh \n",tempC[0],tempC[1],tempC[2],tempC[3],tempC[4],tempC[5],tempC[6],tempC[7]) ;
TRACE(" %02xh %02xh \n",tempC[8],tempC[9]) ;
	
	/****** Cross talk ******/
	for( i=0 , n=0 ; n<5 ; n++ ){
		tblC[n] = tempC[i] + (tempC[i+1]<<8);
		i += 2;
	}
TRACE("[XX] %04xh [XY]%04xh [SFT]%02xh\n",tblC[0],tblC[1],(unsigned char)(tblC[4]>>0)) ;
TRACE("[YY] %04xh [YX]%04xh [SFT]%02xh\n",tblC[2],tblC[3],(unsigned char)(tblC[4]>>8)) ;
	RamWrite32A( HF_hx45x , (UINT32)(tblC[0]<<16));
	RamWrite32A( HF_hx45y , (UINT32)(tblC[1]<<16));
	RamWrite32A( HF_hy45y , (UINT32)(tblC[2]<<16));
	RamWrite32A( HF_hy45x , (UINT32)(tblC[3]<<16));
	RamWrite32A( HF_ShiftX , (UINT32)(tblC[4]));

}

//********************************************************************************
// Function Name 	: SetOpticalOffset
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure start setting Function
// History			: First edition
//********************************************************************************
void	SetOpticalOffset( void )
{
	UINT16 tblC[2];
	UINT8  tempC[4];
	UINT8	i,n,cnt;
	
	ReadE2Prom( EEPROM_Calibration_Status_MSB, &cnt );
TRACE("E2prom Read 0x19 = %02x   &  %02x  \n", cnt , (UINT8)(HALL_CALB_FLG>>8) );
	if( cnt & (UINT8)(HALL_CALB_FLG>>8) ){
		return;
	}
	
	BurstReadE2Prom( EEPROM_Optical_LensOffsetX_LSB , tempC, 4 );
	
TRACE("\n EEPROM_Optical_LensOffset Read \n" );
TRACE(" %02xh %02xh %02xh %02xh \n",tempC[0],tempC[1],tempC[2],tempC[3]) ;
	
	/****** Cross talk ******/
	for( i=0 , n=0 ; n<2 ; n++ ){
		tblC[n] = tempC[i] + (tempC[i+1]<<8);
		i += 2;
	}
	
	if( tblC[ 0 ] == 0xFFFF ) {
		tblC[ 0 ]	= 0 ;
	}
	
	if( tblC[ 1 ] == 0xFFFF ) {
		tblC[ 1 ]	= 0 ;
	}
	
TRACE("[XX] %04xh [XY]%04xh \n", tblC[0], tblC[1]) ;
	RamWrite32A( Optical_Offset_X , (UINT32)(tblC[0]<<16));
	RamWrite32A( Optical_Offset_Y , (UINT32)(tblC[1]<<16));

}

