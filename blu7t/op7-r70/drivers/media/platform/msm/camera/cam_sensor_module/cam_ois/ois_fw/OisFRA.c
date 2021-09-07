/**
 * @brief		FRA measurement command for LC898123 F40
 *
 * @author		Copyright (C) 2016, ON Semiconductor, all right reserved.
 *
 * @file		OisFRA.c
 * @date		svn:$Date:: 2016-06-17 16:42:32 +0900#$
 * @version	svn:$Revision: 54 $
 * @attention
 **/

//**************************
//	Include Header File
//**************************
#define		__OISFRA__

#include	<math.h>
#include	"Ois.h"
#include	"OisAPI.h"
#include	"OisFRA.h"


//#define ACT_THROUGH_CLOSE		// for ball type
//#define CLOSED_RESPONSE			// for openloop measurement

//****************************************************
//	CUSTOMER NECESSARY CREATING LIST
//****************************************************
/* for I2C communication */
extern	void RamWrite32A( INT32, INT32 );
extern 	void RamRead32A( UINT16, void * );
/* for Wait timer [Need to adjust for your system] */
extern void	WitTim( UINT16 );

//**************************
//	External Function Prototype
//**************************
extern void	SetSineWave(   UINT8 , UINT8 );
extern void	SetSinWavGenInt( void );
extern void	SetTransDataAdr( UINT16, UINT32  ) ;
extern void	MeasureWait( void ) ;
extern void	ClrMesFil( void ) ;
extern void	SetWaitTime( UINT16 ) ;

#ifdef ACT_THROUGH_CLOSE
UINT_32 BackupParameter[30];
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/* function name    : SetThroughParameter 		                                         */
/* input parameter  :                                                                    */
/* output parameter :                                                                    */
/* comment          : DFTの係数発生    			                                         */
/*                                                                            2018.01.18 */
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
void SetThroughParameter(UINT_8	UcDirSel )
{
	if( UcDirSel == X_DIR ) {
		BackupParameter[29] = X_DIR;
		RamRead32A( HallFilterCoeffX_hxdgain0, &BackupParameter[0]);
		RamRead32A( HallFilterCoeffX_hxpgain0, &BackupParameter[1]);
		RamRead32A( HallFilterCoeffX_hxpgain1, &BackupParameter[2]);
		RamRead32A( HallFilterCoeffX_hxigain0, &BackupParameter[3]);
		RamRead32A( HallFilterCoeffX_hxgain0, &BackupParameter[4]);	
		RamRead32A( HallFilterShiftX, &BackupParameter[5]);	
		RamRead32A( (HallFilterShiftX+4), &BackupParameter[6]);	
		RamRead32A( HallFilterCoeffX_hxsa, &BackupParameter[7]);
		RamRead32A( HallFilterCoeffX_hxsb, &BackupParameter[8]);
		RamRead32A( HallFilterCoeffX_hxsc, &BackupParameter[9]);
		RamRead32A( HallFilterCoeffX_hxoa, &BackupParameter[10]);
		RamRead32A( HallFilterCoeffX_hxob, &BackupParameter[11]);
		RamRead32A( HallFilterCoeffX_hxoc, &BackupParameter[12]);
		RamRead32A( HallFilterCoeffX_hxod, &BackupParameter[13]);
		RamRead32A( HallFilterCoeffX_hxoe, &BackupParameter[14]);
		RamRead32A( HallFilterCoeffX_hxpa, &BackupParameter[15]);
		RamRead32A( HallFilterCoeffX_hxpb, &BackupParameter[16]);
		RamRead32A( HallFilterCoeffX_hxpc, &BackupParameter[17]);
		RamRead32A( HallFilterCoeffX_hxpd, &BackupParameter[18]);
		RamRead32A( HallFilterCoeffX_hxpe, &BackupParameter[19]);

		RamWrite32A( HallFilterCoeffX_hxdgain0, 0x00000000);	//RAMW32	80EC	00000000
		RamWrite32A( HallFilterCoeffX_hxpgain0, 0x7fffffff);	//RAMW32	80D8	7fffffff
		RamWrite32A( HallFilterCoeffX_hxpgain1, 0x7fffffff);	//RAMW32	80E4	7fffffff
		RamWrite32A( HallFilterCoeffX_hxigain0, 0x00000000);	//RAMW32	80E8	00000000
		RamWrite32A( HallFilterCoeffX_hxgain0, 0x7fffffff);		//RAMW32	80F0	7fffffff
		RamWrite32A( HallFilterShiftX, 0x00000000);				//RAMW32	81F8	00000000
		RamWrite32A( (HallFilterShiftX+4), 0x00000000);			//RAMW32	81FC	00000000
		RamWrite32A( HallFilterCoeffX_hxsa, 0x7fffffff);		//RAMW32	8100	7fffffff
		RamWrite32A( HallFilterCoeffX_hxsb, 0x00000000);		//RAMW32	80F8	00000000
		RamWrite32A( HallFilterCoeffX_hxsc, 0x00000000);		//RAMW32	80FC	00000000
		RamWrite32A( HallFilterCoeffX_hxoa, 0x7fffffff);		//RAMW32	8114	7fffffff
		RamWrite32A( HallFilterCoeffX_hxob, 0x00000000);		//RAMW32	8104	00000000
		RamWrite32A( HallFilterCoeffX_hxoc, 0x00000000);		//RAMW32	8108	00000000
		RamWrite32A( HallFilterCoeffX_hxod, 0x00000000);		//RAMW32	810C	00000000
		RamWrite32A( HallFilterCoeffX_hxoe, 0x00000000);		//RAMW32	8110	00000000
		RamWrite32A( HallFilterCoeffX_hxpa, 0x7fffffff);		//RAMW32	8128	7fffffff
		RamWrite32A( HallFilterCoeffX_hxpb, 0x00000000);		//RAMW32	8118	00000000
		RamWrite32A( HallFilterCoeffX_hxpc, 0x00000000);		//RAMW32	811C	00000000
		RamWrite32A( HallFilterCoeffX_hxpd, 0x00000000);		//RAMW32	8120	00000000
		RamWrite32A( HallFilterCoeffX_hxpe, 0x00000000);		//RAMW32	8124	00000000
	}else if( UcDirSel == Y_DIR ){
		BackupParameter[29] = Y_DIR;
		RamRead32A( HallFilterCoeffY_hydgain0, &BackupParameter[0]);
		RamRead32A( HallFilterCoeffY_hypgain0, &BackupParameter[1]);
		RamRead32A( HallFilterCoeffY_hypgain1, &BackupParameter[2]);
		RamRead32A( HallFilterCoeffY_hyigain0, &BackupParameter[3]);
		RamRead32A( HallFilterCoeffY_hygain0, &BackupParameter[4]);
		RamRead32A( HallFilterShiftY, &BackupParameter[5]);	
		RamRead32A( HallFilterCoeffY_hysa, &BackupParameter[6]);
		RamRead32A( HallFilterCoeffY_hysb, &BackupParameter[7]);
		RamRead32A( HallFilterCoeffY_hysc, &BackupParameter[8]);
		RamRead32A( HallFilterCoeffY_hyoa, &BackupParameter[9]);
		RamRead32A( HallFilterCoeffY_hyob, &BackupParameter[10]);
		RamRead32A( HallFilterCoeffY_hyoc, &BackupParameter[11]);
		RamRead32A( HallFilterCoeffY_hyod, &BackupParameter[12]);
		RamRead32A( HallFilterCoeffY_hyoe, &BackupParameter[13]);
		RamRead32A( HallFilterCoeffY_hypa, &BackupParameter[14]);
		RamRead32A( HallFilterCoeffY_hypb, &BackupParameter[15]);
		RamRead32A( HallFilterCoeffY_hypc, &BackupParameter[16]);
		RamRead32A( HallFilterCoeffY_hypd, &BackupParameter[17]);
		RamRead32A( HallFilterCoeffY_hype, &BackupParameter[18]);
		
		RamWrite32A( HallFilterCoeffY_hydgain0, 0x00000000);	//RAMW32	8188	00000000
		RamWrite32A( HallFilterCoeffY_hypgain0, 0x7fffffff);	//RAMW32	8174	7fffffff
		RamWrite32A( HallFilterCoeffY_hypgain1, 0x7fffffff);	//RAMW32	8180	7fffffff
		RamWrite32A( HallFilterCoeffY_hyigain0, 0x00000000);	//RAMW32	8184	00000000
		RamWrite32A( HallFilterCoeffY_hygain0, 0x7fffffff);		//RAMW32	818C	7fffffff
		RamWrite32A( HallFilterShiftY, 0x00000000);				//RAMW32	8200	00000000
		RamWrite32A( HallFilterCoeffY_hysa, 0x7fffffff);		//RAMW32	819C	7fffffff
		RamWrite32A( HallFilterCoeffY_hysb, 0x00000000);		//RAMW32	8194	00000000
		RamWrite32A( HallFilterCoeffY_hysc, 0x00000000);		//RAMW32	8198	00000000
		RamWrite32A( HallFilterCoeffY_hyoa, 0x7fffffff);		//RAMW32	81B0	7fffffff
		RamWrite32A( HallFilterCoeffY_hyob, 0x00000000);		//RAMW32	81A0	00000000
		RamWrite32A( HallFilterCoeffY_hyoc, 0x00000000);		//RAMW32	81A4	00000000
		RamWrite32A( HallFilterCoeffY_hyod, 0x00000000);		//RAMW32	81A8	00000000
		RamWrite32A( HallFilterCoeffY_hyoe, 0x00000000);		//RAMW32	81AC	00000000
		RamWrite32A( HallFilterCoeffY_hypa, 0x7fffffff);		//RAMW32	81C4	7fffffff
		RamWrite32A( HallFilterCoeffY_hypb, 0x00000000);		//RAMW32	81B4	00000000
		RamWrite32A( HallFilterCoeffY_hypc, 0x00000000);		//RAMW32	81B8	00000000
		RamWrite32A( HallFilterCoeffY_hypd, 0x00000000);		//RAMW32	81BC	00000000
		RamWrite32A( HallFilterCoeffY_hype, 0x00000000);		//RAMW32	81C0	00000000
	}else if( UcDirSel == Z_DIR ){
		}
	}

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/* function name    : ResetThroughParameter 		                                     */
/* input parameter  :                                                                    */
/* output parameter :                                                                    */
/* comment          : DFTの係数発生    			                                         */
/*                                                                            2018.01.18 */
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
void ResetThroughParameter(void)
{
	if( BackupParameter[29] == X_DIR ) {
		RamWrite32A( HallFilterCoeffX_hxdgain0, BackupParameter[0]);
		RamWrite32A( HallFilterCoeffX_hxpgain0, BackupParameter[1]);
		RamWrite32A( HallFilterCoeffX_hxpgain1, BackupParameter[2]);
		RamWrite32A( HallFilterCoeffX_hxigain0, BackupParameter[3]);
		RamWrite32A( HallFilterCoeffX_hxgain0, BackupParameter[4]);	
		RamWrite32A( HallFilterShiftX, BackupParameter[5]);	
		RamWrite32A( (HallFilterShiftX+4), BackupParameter[6]);	
		RamWrite32A( HallFilterCoeffX_hxsa, BackupParameter[7]);
		RamWrite32A( HallFilterCoeffX_hxsb, BackupParameter[8]);
		RamWrite32A( HallFilterCoeffX_hxsc, BackupParameter[9]);
		RamWrite32A( HallFilterCoeffX_hxoa, BackupParameter[10]);
		RamWrite32A( HallFilterCoeffX_hxob, BackupParameter[11]);
		RamWrite32A( HallFilterCoeffX_hxoc, BackupParameter[12]);
		RamWrite32A( HallFilterCoeffX_hxod, BackupParameter[13]);
		RamWrite32A( HallFilterCoeffX_hxoe, BackupParameter[14]);
		RamWrite32A( HallFilterCoeffX_hxpa, BackupParameter[15]);
		RamWrite32A( HallFilterCoeffX_hxpb, BackupParameter[16]);
		RamWrite32A( HallFilterCoeffX_hxpc, BackupParameter[17]);
		RamWrite32A( HallFilterCoeffX_hxpd, BackupParameter[18]);
		RamWrite32A( HallFilterCoeffX_hxpe, BackupParameter[19]);
	}else if( BackupParameter[29] == Y_DIR ){
		RamWrite32A( HallFilterCoeffY_hydgain0, BackupParameter[0]);
		RamWrite32A( HallFilterCoeffY_hypgain0, BackupParameter[1]);
		RamWrite32A( HallFilterCoeffY_hypgain1, BackupParameter[2]);
		RamWrite32A( HallFilterCoeffY_hyigain0, BackupParameter[3]);
		RamWrite32A( HallFilterCoeffY_hygain0, BackupParameter[4]);
		RamWrite32A( HallFilterShiftY, BackupParameter[5]);	
		RamWrite32A( HallFilterCoeffY_hysa, BackupParameter[6]);
		RamWrite32A( HallFilterCoeffY_hysb, BackupParameter[7]);
		RamWrite32A( HallFilterCoeffY_hysc, BackupParameter[8]);
		RamWrite32A( HallFilterCoeffY_hyoa, BackupParameter[9]);
		RamWrite32A( HallFilterCoeffY_hyob, BackupParameter[10]);
		RamWrite32A( HallFilterCoeffY_hyoc, BackupParameter[11]);
		RamWrite32A( HallFilterCoeffY_hyod, BackupParameter[12]);
		RamWrite32A( HallFilterCoeffY_hyoe, BackupParameter[13]);
		RamWrite32A( HallFilterCoeffY_hypa, BackupParameter[14]);
		RamWrite32A( HallFilterCoeffY_hypb, BackupParameter[15]);
		RamWrite32A( HallFilterCoeffY_hypc, BackupParameter[16]);
		RamWrite32A( HallFilterCoeffY_hypd, BackupParameter[17]);
		RamWrite32A( HallFilterCoeffY_hype, BackupParameter[18]);
	}else if( BackupParameter[29] == Z_DIR ){
	}
}
#endif
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/* function name    : CoeffGenerate  		                                             */
/* input parameter  :                                                                    */
/* output parameter :                                                                    */
/* comment          : DFTの係数発生    			                                         */
/*                                                                            2018.01.18 */
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#define	Q31 	( 0x7FFFFFFF )
#define	Q23 	( 0x007FFFFF )
#define	Q21 	( 0x001FFFFF )
#define	PAI 	( 3.14159265358979323846 )
#define N 		( 2048 )
int     nDivision;

void CoeffGenerate( double fc )
{
	double  df, fs; 
	int     point, C0, S0, CN, SN;
	double  theta;			// theta = 2*Pi*f/Fs

	if 		( fc > 40 ){ nDivision = 0; fs = (FS_FREQ    ); }
	else if ( fc > 20 ){ nDivision = 1; fs = (FS_FREQ / 2); }
	else if ( fc > 10 ){ nDivision = 2; fs = (FS_FREQ / 4); }
	else if ( fc >  5 ){ nDivision = 3; fs = (FS_FREQ / 8); }
	else 			   { nDivision = 4; fs = (FS_FREQ /16); }

	//***** 取得した周波数テーブルから判定ポイントと判定thetaの算出 *****
	df = fs / (double)N;									// FFTの1ポイント当たりの周波数
	point = (int)(fc / df + 0.5);							// 判定ポイントの算出
	theta = 2.0 * PAI * (double)point * df / fs;			// 判定ポイントでの位相の算出

	C0 = (int)((double)Q31 * cos(theta) + 0.5);
	S0 = (int)((double)Q31 * sin(theta) + 0.5);
	CN = (int)((double)Q31 * cos(((double)N - 1.0) * theta) + 0.5);
	SN = (int)((double)Q31 * sin(((double)N - 1.0) * theta) + 0.5);

	RamWrite32A( FRA_DMA_DeciShift, nDivision );	
	RamWrite32A( FRA_DMB_C0, C0 ) ;
	RamWrite32A( FRA_DMB_S0, S0 ) ;
	RamWrite32A( FRA_DMB_CN, CN ) ;
	RamWrite32A( FRA_DMB_SN, SN ) ;

TRACE("0x%08X, 0x%08X, 0x%08X, 0x%08X,\n", C0, S0, CN, SN);
}

//********************************************************************************
// Function Name 	: Freq_Convert
// Retun Value		: Phase Step Value
// Argment Value	: Frequency
// Explanation		: Convert Frequency
// History			: First edition
//********************************************************************************
UINT32	Freq_Convert( float SfFreq )
{
	UINT32	UlPhsStep;
	
	UlPhsStep	= ( UINT32 )( ( SfFreq * ( float )0x100000000 / FS_FREQ + 0.5F ) / 2.0F ) ;

	return( UlPhsStep ) ;
}


//********************************************************************************
// Function Name 	: MesStart_FRA_Single
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure start setting Function
// History			: First edition
//********************************************************************************
void	MesStart_FRA_Single( UINT8	UcDirSel )
{
	float	SfTmp ;
	INT32	GainQ23, PhaseQ21 ;
	UINT32	UlReadVal ;	


	SetSinWavGenInt() ;
	// Change Frequency 
	RamWrite32A( SinWave_Offset,	Freq_Convert( StFRAParam.StHostCom.SfFrqCom.SfFltVal ) ) ;		// Freq Setting = Freq * 80000000h / Fs	: 10Hz

	SfTmp	= StFRAParam.StHostCom.SfAmpCom.SfFltVal / 1400.0F ;									// AVDD 2800mV / 2 = 1400mV
	RamWrite32A( SinWave_Gain,		( UINT32 )( ( float )0x7FFFFFFF * SfTmp ) ) ;					// Set Sine Wave Gain

	if ( StFRAParam.StHostCom.UcAvgCycl == 10) 	{  		// Actuator Through
#ifdef ACT_THROUGH_CLOSE
		SetThroughParameter( UcDirSel );

		if( UcDirSel == X_DIR ) {
			SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_FRA_XSININ ) ;								// Set Sine Wave Input RAM
			// Set parameter and input/output address
			RamWrite32A( FRA_DMA_InputData, HALL_FRA_XHOUTA ) ;
			RamWrite32A( FRA_DMA_OutputData, HALL_FRA_XHOUTB ) ;
		}else if( UcDirSel == Y_DIR ){
			SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_FRA_YSININ ) ;								// Set Sine Wave Input RAM
			// Set parameter and input/output address
			RamWrite32A( FRA_DMA_InputData, HALL_FRA_YHOUTA ) ;
			RamWrite32A( FRA_DMA_OutputData, HALL_FRA_YHOUTB ) ;
		}else if( UcDirSel == Z_DIR ){
		}
#else
		RtnCen( BOTH_OFF ) ;
		if( UcDirSel == X_DIR ) {
			SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_RAM_SINDX1 ) ;								// Set Sine Wave Input RAM
			// Set parameter and input/output address
			RamWrite32A( FRA_DMA_InputData, HALL_RAM_SINDX1 ) ;
			RamWrite32A( FRA_DMA_OutputData, HALL_RAM_HXIDAT ) ;
		}else if( UcDirSel == Y_DIR ){
			SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_RAM_SINDY1 ) ;								// Set Sine Wave Input RAM
			// Set parameter and input/output address
			RamWrite32A( FRA_DMA_InputData, HALL_RAM_SINDY1 ) ;
			RamWrite32A( FRA_DMA_OutputData, HALL_RAM_HXIDAT ) ;
		}
 #ifdef SEL_CLOSED_AF
		else if( UcDirSel == Z_DIR ){
			SetTransDataAdr( SinWave_OutAddr	,	(UINT32)CLAF_RAMA_AFOUT ) ;								// Set Sine Wave Input RAM
			// Set parameter and input/output address
			RamWrite32A( FRA_DMA_InputData, CLAF_RAMA_AFOUT ) ;
			RamWrite32A( FRA_DMA_OutputData, CLAF_RAMA_AFADIN ) ;
	}
 #endif
#endif
		} else {
		if( UcDirSel == X_DIR ) {
			SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_FRA_XSININ ) ;								// Set Sine Wave Input RAM
			// Set parameter and input/output address
#ifdef CLOSED_RESPONSE
			RamWrite32A( FRA_DMA_InputData, HALL_FRA_XSININ ) ;
#else
			RamWrite32A( FRA_DMA_InputData, HALL_FRA_XHOUTA ) ;
#endif
			RamWrite32A( FRA_DMA_OutputData, HALL_FRA_XHOUTB ) ;
		}else if( UcDirSel == Y_DIR ){
			SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_FRA_YSININ ) ;								// Set Sine Wave Input RAM
			// Set parameter and input/output address
#ifdef CLOSED_RESPONSE
			RamWrite32A( FRA_DMA_InputData, HALL_FRA_YSININ  ) ;
#else
			RamWrite32A( FRA_DMA_InputData, HALL_FRA_YHOUTA ) ;
#endif			
			RamWrite32A( FRA_DMA_OutputData, HALL_FRA_YHOUTB ) ;
		}
#ifdef SEL_CLOSED_AF
		else if( UcDirSel == Z_DIR ){
			SetTransDataAdr( SinWave_OutAddr	,	(UINT32)CLAF_RAMA_AFSINE ) ;								// Set Sine Wave Input RAM
			// Set parameter and input/output address
#ifdef CLOSED_RESPONSE
			RamWrite32A( FRA_DMA_InputData, CLAF_RAMA_AFSINE  ) ;
#else
			RamWrite32A( FRA_DMA_InputData, CLAF_DELAY_AFDZ0 ) ;
#endif			
			RamWrite32A( FRA_DMA_OutputData, CLAF_RAMA_AFDEV ) ;
		}
#endif // SEL_CLOSED_AF
	}
	RamWrite32A( SinWaveC_Regsiter	,	0x00000001 ) ;												// Sine Wave Start


	CoeffGenerate( StFRAParam.StHostCom.SfFrqCom.SfFltVal );
//	WitTim(10);
	// Start to measure
	RamWrite32A( FRA_DMA_Control,1 ) ;

	if (nDivision == 0)	WitTim(100);
	if (nDivision == 1)	WitTim(200);
	if (nDivision == 2)	WitTim(400);
	if (nDivision == 3)	WitTim(800);
	if (nDivision == 4)	WitTim(1600);
	do{
		WitTim(10);
		RamRead32A( FRA_DMA_Control	, &UlReadVal ) ;	
	}while (UlReadVal == 1);
	// Read answer
	RamRead32A( FRA_DMA_Gain	, &GainQ23 ) ;		// Gain
	RamRead32A( FRA_DMA_Phase	, &PhaseQ21 ) ;		// Phase
	StFRAParam.StMesRslt.SfGainAvg = (float)GainQ23 / Q23; //0x007FFFFF;
	StFRAParam.StMesRslt.SfPhaseAvg = (float)PhaseQ21 / Q21; //0x001FFFFF;	

	TRACE("Phase %f deg : Gain %f dB\n", StFRAParam.StMesRslt.SfPhaseAvg, StFRAParam.StMesRslt.SfGainAvg );

}



//********************************************************************************
// Function Name 	: MesStart_FRA_Continue
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Continue Measurement Function
// History			: First edition
//********************************************************************************
void	MesStart_FRA_Continue( void )
{
	INT32	GainQ23, PhaseQ21 ;
	UINT32	UlReadVal ;	
	
	// Change Frequency 
	RamWrite32A( SinWave_Offset,	Freq_Convert( StFRAParam.StHostCom.SfFrqCom.SfFltVal ) ) ;
	// Set parameter
	CoeffGenerate( StFRAParam.StHostCom.SfFrqCom.SfFltVal );
//	WitTim(10)
	// Start to measure
	RamWrite32A( FRA_DMA_Control,1 ) ;									// Integral Value Clear
	if (nDivision == 0)	WitTim(100);
	if (nDivision == 1)	WitTim(200);
	if (nDivision == 2)	WitTim(400);
	if (nDivision == 3)	WitTim(800);
	if (nDivision == 4)	WitTim(1600);
	do{
		WitTim(10);	
		RamRead32A( FRA_DMA_Control	, &UlReadVal ) ;	
	}while (UlReadVal == 1);
	// Read answer
	RamRead32A( FRA_DMA_Gain	, &GainQ23 ) ;		// Gain
	RamRead32A( FRA_DMA_Phase	, &PhaseQ21 ) ;		// Phase
	StFRAParam.StMesRslt.SfGainAvg = (float)GainQ23 / Q23;
	StFRAParam.StMesRslt.SfPhaseAvg = (float)PhaseQ21 / Q21;	

	TRACE("Phase %f deg : Gain %f dB\n", StFRAParam.StMesRslt.SfPhaseAvg, StFRAParam.StMesRslt.SfGainAvg );
}



//********************************************************************************
// Function Name 	: MesEnd_FRA_Sweep
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Stop Measurement Function
// History			: First edition
//********************************************************************************
void	MesEnd_FRA_Sweep( void )
{
	// Stop Sine Wave
	RamWrite32A( SinWaveC_Regsiter,		0x00000000 ) ;					// Sine Wave Stop
	SetTransDataAdr( SinWave_OutAddr,	( UINT32 )0x00000000 ) ;		// Set Sine Wave Input RAM

	if ( StFRAParam.StHostCom.UcAvgCycl == 10) 	{  		// Actuator Through
#ifdef ACT_THROUGH_CLOSE
		ResetThroughParameter( );
#else
		RtnCen( BOTH_ON ) ;
#endif
	}
	RamWrite32A( HALL_RAM_SINDX0,	0x00000000 ) ;		// DelayRam Clear
	RamWrite32A( HALL_RAM_SINDY0,	0x00000000 ) ;		// DelayRam Clear
	RamWrite32A( HALL_FRA_XSININ,	0x00000000 ) ;		// DelayRam Clear
	RamWrite32A( HALL_FRA_YSININ,	0x00000000 ) ;		// DelayRam Clear

}
