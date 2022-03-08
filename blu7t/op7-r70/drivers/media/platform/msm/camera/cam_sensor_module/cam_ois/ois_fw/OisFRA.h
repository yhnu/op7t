/**
 * @brief		FRA measurement header for LC898123 F40
 * 				API List for customers
 *
 * @author		Copyright (C) 2016, ON Semiconductor, all right reserved.
 *
 * @file		OisAPI.h
 * @date		svn:$Date:: 2016-04-28 14:30:21 +0900#$
 * @version		svn:$Revision: 43 $
 * @attention
 **/
#ifndef OISFRA_H_
#define OISFRA_H_

//****************************************************
//	extern selector for API
//****************************************************
#ifdef	__OISFRA__
	#define	__OIS_FRA_HEADER__
#else
	#define	__OIS_FRA_HEADER__		extern
#endif

typedef struct STFRA_PARAM {
	struct {
		UnFltVal		SfFrqCom ;
		UnFltVal		SfAmpCom ;
		unsigned char	UcAvgCycl ;
	} StHostCom ;

	float				SfGain[ 10 ] ;
	float				SfPhase[ 10 ] ;

	struct {
		float			SfGainAvg ;
		float			SfPhaseAvg ;
	} StMesRslt ;
} StFRAParam_t ;

__OIS_FRA_HEADER__	StFRAParam_t	StFRAParam ;
/*
typedef struct STFRA_MES {
	UINT_64	UllCumulAdd1 ;
	UINT_64	UllCumulAdd2 ;
	UINT_16	UsFsCount ;
} StFRAMes_t ;

__OIS_FRA_HEADER__	StFRAMes_t		StFRAMes ;

typedef struct  {
	INT_32	a1 ;
	INT_32	b1 ;
	INT_32	c1 ;
	INT_32	a2 ;
	INT_32	b2 ;
	INT_32	c2 ;
} StMesFCoeff_t ;

*/


#endif	// OISFRA_H_
