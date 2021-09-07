//********************************************************************************
//		<< LC898124 Evaluation Soft >>
//********************************************************************************

//**************************
//	Include Header File		
//**************************
#include "Ois.h"

#include "LC898124EP3_Code_0_1_2_2_0_0.h"		// Gyro=LSM6DSM,	SPI Maste	SO2820
#include "LC898124EP3_Code_0_1_3_2_1_0.h"		// Gyro=LSM6DSM,	SPI Slave	SO3600

#include "LC898124EP3_Code_0_1_2_3_0_0.h"		// Gyro=BMI160,		SPI Maste	SO2820
#include "LC898124EP3_Code_0_1_3_3_1_0.h"		// Gyro=BMI160,		SPI Slave	SO3600

#include "LC898124EP3_Code_0_1_2_2_0_1.h"		// Gyro=LSM6DSM,	FRA, SPI Maste	SO2820
#include "LC898124EP3_Code_0_1_3_2_1_1.h"		// Gyro=LSM6DSM,	FRA, SPI Slave	SO3600

#include "LC898124EP3_Code_0_1_2_3_0_1.h"		// Gyro=BMI160,		FRA, SPI Maste	SO2820
#include "LC898124EP3_Code_0_1_3_3_1_1.h"		// Gyro=BMI160,		FRA, SPI Slave	SO3600

//****************************************************
//	CUSTOMER NECESSARY CREATING LIST
//****************************************************
/* for I2C communication */ 
extern	void RamWrite32A( unsigned short, int );
extern 	void RamRead32A( unsigned short, void * );
/* for I2C Multi Translation : Burst Mode*/
extern 	void CntWrt( void *, unsigned short) ;
extern void CntRd( unsigned int addr, void *	PcSetDat, unsigned short	UsDatNum )  ;
/* for Wait timer [Need to adjust for your system] */ 
extern void	WitTim( unsigned short	UsWitTim );

//****************************************************
//	LOCAL RAM LIST
//****************************************************
#define BURST_LENGTH_PM ( 12*5 ) 	// 60 必ず5の倍数で設定すること。最大64Byteまで
#define BURST_LENGTH_DM ( 10*6 ) 	// 60 必ず6の倍数で設定すること。最大64Byteまで
#define BURST_LENGTH BURST_LENGTH_PM 	

//********************************************************************************
// Function Name 	: DMIOWrite32
// Retun Value		: None
// Argment Value	: IOadrs, IOdata
// Explanation		: Read From code version Command
// History			: First edition 						
//********************************************************************************
void DMIOWrite32( UINT32 IOadrs, UINT32 IOdata )
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
	CntWrt( data, 10 ); 	// I2C 1Byte address.
#else
	RamWrite32A( CMD_IO_ADR_ACCESS, IOadrs ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, IOdata ) ;
#endif
};

//********************************************************************************
// Function Name 	: DownloadToEP3
// Retun Value		: NON
// Argment Value	: PMlength: 5byte unit, DMlength : 1Byte unit 
// Explanation		: <Pmem Memory> Write Data
// History			: First edition
//********************************************************************************
unsigned char DownloadToEP3( const UINT8* DataPM, UINT32 LengthPM, UINT32 Parity, const UINT8* DataDM, UINT32 LengthDMA , UINT32 LengthDMB ) 
{
	UINT32 i, j;
	UINT8 data[64];		// work fifo buffer max size 64 byte
	UINT8 Remainder;	// 余り
	UINT32 UlReadVal, UlCnt;
	UINT32 ReadVerifyPM, ReadVerifyDMA, ReadVerifyDMB;	// Checksum
	
//*******************************************************************************//
//*   pre-check ROM code version 												*//
//*******************************************************************************//
	RamRead32A( CMD_ROMVER , &UlReadVal );
	if( UlReadVal == OLD_VER )	return( 3 );		/* ROM code version error */
	
//--------------------------------------------------------------------------------
// 0. Start up to boot exection 
//--------------------------------------------------------------------------------
	RamWrite32A( CMD_IO_ADR_ACCESS , ROMINFO );
	RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	switch ( (UINT8)UlReadVal ){
	case 0x0A:	/* Normal Rom program execution */
		break;
	
	case 0x01:	/* Normal Ram program execution */
		/* 再Donloadのためには RomRebootしなければならない。AutoDownloadさせるためにCORE_RSTで実行させる*/
		DMIOWrite32( SYSDSP_REMAP, 0x00001000 ); 	// CORE_RST
		WitTim( 6 ) ;								// Bootプログラムを回すのに6msec必要。
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
	CntWrt( data, 5 ); 	// I2C 1Byte address.
	// program start
	data[0] = 0x40;		// Pmem address set
	Remainder = ( (LengthPM*5) / BURST_LENGTH_PM ); 
	for(i=0 ; i< Remainder ; i++)
	{
		UlCnt = 1;
		for(j=0 ; j < BURST_LENGTH_PM; j++)	data[UlCnt++] = *DataPM++;
		
		CntWrt( data, BURST_LENGTH_PM+1 );  // I2Caddresss 1Byte.
	}
	Remainder = ( (LengthPM*5) % BURST_LENGTH_PM); 
	if (Remainder != 0 )
	{
		UlCnt = 1;
		for(j=0 ; j < Remainder; j++)	data[UlCnt++] = *DataPM++;
//TRACE("Remainder %d \n", (UINT8)Remainder );
		CntWrt( data, UlCnt );  // I2C 1Byte address.
	}
	// Chercksum start
	data[0] = 0xF0;											// Pmem address set
	data[1] = 0x0A;											// Command High
	data[2] = (unsigned char)(( LengthPM & 0xFF00) >> 8 );	// Size High
	data[3] = (unsigned char)(( LengthPM & 0x00FF) >> 0 );	// Size Low
	CntWrt( data, 4 ); 	// I2C 2Byte addresss.
	
//--------------------------------------------------------------------------------
// 2. Download Table Data
//--------------------------------------------------------------------------------
//TRACE("DM Start \n" );	
	RamWrite32A( DmCheck_CheckSumDMA, 0 );		// DMA Parity Clear
	RamWrite32A( DmCheck_CheckSumDMB, 0 );		// DMB Parity Clear

	/***** DMA Data Send *****/
	Remainder = ( (LengthDMA*6/4) / BURST_LENGTH_DM ); 
	for(i=0 ; i< Remainder ; i++)
	{
		CntWrt( (UINT8*)DataDM, BURST_LENGTH_DM );  // I2Caddresss 1Byte.
		DataDM += BURST_LENGTH_DM;
	}
	Remainder = ( (LengthDMA*6/4) % BURST_LENGTH_DM ); 
	if (Remainder != 0 )
	{
		CntWrt( (UINT8*)DataDM, (UINT8)Remainder );  // I2Caddresss 1Byte.
	}
	DataDM += Remainder;
	
	/***** DMB Data Send *****/
	Remainder = ( (LengthDMB*6/4) / BURST_LENGTH_DM ); 
	for( i=0 ; i< Remainder ; i++)	/* 続きから */
	{
		CntWrt( (UINT8*)DataDM, BURST_LENGTH_DM );  // I2Caddresss 1Byte.
		DataDM += BURST_LENGTH_DM;
	}
	Remainder = ( (LengthDMB*6/4) % BURST_LENGTH_DM ); 
	if (Remainder != 0 )
	{
		CntWrt( (UINT8*)DataDM, (UINT8)Remainder );  // I2Caddresss 1Byte.
	}
	
//--------------------------------------------------------------------------------
// 3. Verify
//--------------------------------------------------------------------------------
	RamRead32A( PmCheck_CheckSum, &ReadVerifyPM );
	RamRead32A( DmCheck_CheckSumDMA, &ReadVerifyDMA );
	RamRead32A( DmCheck_CheckSumDMB, &ReadVerifyDMB );

	
	if( (ReadVerifyPM + ReadVerifyDMA + ReadVerifyDMB) != Parity  ){
TRACE("error! %08x %08x %08x \n", (unsigned int)ReadVerifyPM, (unsigned int)ReadVerifyDMA, (unsigned int)ReadVerifyDMB );	
		return( 2 );
	}
	return(0);
}


//********************************************************************************
// Function Name 	: ReMapMain
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: <Pmem Memory> Write Data
// History			: First edition
//********************************************************************************
void RemapMain( void )
{
	RamWrite32A( 0xF000, 0x00000000 ) ;
}

//********************************************************************************
// Function Name 	: MonitorInfo
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: 
// History			: Second edition
//********************************************************************************
void MonitorInfo( DSPVER* Dspcode )
{
TRACE("Vendor : %02x \n", Dspcode->Vendor );
TRACE("User : %02x \n", Dspcode->User );
TRACE("Model : %02x \n", Dspcode->Model );
TRACE("Version : %02x \n", Dspcode->Version );


if(Dspcode->SpiMode == SPI_MST )
TRACE("spi mode : Master\n");	
if(Dspcode->SpiMode == SPI_SLV )
TRACE("spi mode : Slave\n");	
if(Dspcode->SpiMode == SPI_SNGL )
TRACE("spi mode : only master\n");	

//if(Dspcode->ActType == ACT_SEMCO	)
//TRACE("actuator type : SOXXXX\n");
if(Dspcode->ActType == ACT_SO2820) {
	TRACE("actuator type : SO2820\n");
} else if(Dspcode->ActType == ACT_SO3600) {
	TRACE("actuator type : SO3600\n");
} else {
	TRACE("actuator type : SOXXXX\n");
}

if(Dspcode->GyroType == GYRO_ICM20690 )
TRACE("gyro type : INVEN ICM20690 \n");	
if(Dspcode->GyroType == GYRO_LSM6DSM )
TRACE("gyro type : ST LSM6DSM \n")	;

}


//********************************************************************************
// Function Name 	: GetInfomationAfterDownload
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: <Pmem Memory> Write Data
// History			: First edition
//********************************************************************************
UINT8 GetInfomationAfterDownload( DSPVER* Info )
{
	UINT32 Data;
	UINT32 UlReadVal;

	RamWrite32A( CMD_IO_ADR_ACCESS , ROMINFO );
	RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	if( (UINT8)UlReadVal != 0x01 ) return( 1 );

	RamRead32A( (SiVerNum + 0), &Data );
	Info->Vendor 	= (UINT8)(Data >> 24 );
	Info->User 		= (UINT8)(Data >> 16 );
	Info->Model 		= (UINT8)(Data >> 8 );
	Info->Version 	= (UINT8)(Data >> 0 );
	RamRead32A( (SiVerNum + 4), &Data );
	Info->SpiMode =  (UINT8)(Data >> 24 );
	Info->ActType =  (UINT8)(Data >> 8 );
	Info->GyroType = (UINT8)(Data >> 0 );

//	 MonitorInfo( Info );
	return( 0 );
}

//********************************************************************************
// Function Name 	: GetInfomationBeforeDownlaod
// Retun Value		: True(0) / Fail(1)
// Argment Value	: NON
// Explanation		: <Pmem Memory> Write Data
// History			: First edition
//********************************************************************************
UINT8 GetInfomationBeforeDownlaod( DSPVER* Info, const UINT8* DataDM,  UINT32 LengthDM )
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
// Function Name 	: SelectDownload
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: 
// History			: Second edition
//********************************************************************************
extern unsigned char I2cSlvAddrWr;

const DOWNLOAD_TBL DTbl_M[] = {
 {0x0202, LC898124EP3_PM_0_1_2_2_0_0, LC898124EP3_PMSize_0_1_2_2_0_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_2_2_0_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_2_2_0_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_2_2_0_0), LC898124EP3_DM_0_1_2_2_0_0, LC898124EP3_DMA_ByteSize_0_1_2_2_0_0 , LC898124EP3_DMB_ByteSize_0_1_2_2_0_0 },
 {0x0282, LC898124EP3_PM_0_1_2_2_0_1, LC898124EP3_PMSize_0_1_2_2_0_1, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_2_2_0_1 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_2_2_0_1 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_2_2_0_1), LC898124EP3_DM_0_1_2_2_0_1, LC898124EP3_DMA_ByteSize_0_1_2_2_0_1 , LC898124EP3_DMB_ByteSize_0_1_2_2_0_1 },
 {0x0203, LC898124EP3_PM_0_1_2_3_0_0, LC898124EP3_PMSize_0_1_2_3_0_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_2_3_0_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_2_3_0_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_2_3_0_0), LC898124EP3_DM_0_1_2_3_0_0, LC898124EP3_DMA_ByteSize_0_1_2_3_0_0 , LC898124EP3_DMB_ByteSize_0_1_2_3_0_0 },
 {0x0283, LC898124EP3_PM_0_1_2_3_0_1, LC898124EP3_PMSize_0_1_2_3_0_1, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_2_3_0_1 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_2_3_0_1 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_2_3_0_1), LC898124EP3_DM_0_1_2_3_0_1, LC898124EP3_DMA_ByteSize_0_1_2_3_0_1 , LC898124EP3_DMB_ByteSize_0_1_2_3_0_1 },
 {0x0602, LC898124EP3_PM_0_1_2_2_0_0, LC898124EP3_PMSize_0_1_2_2_0_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_2_2_0_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_2_2_0_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_2_2_0_0), LC898124EP3_DM_0_1_2_2_0_0, LC898124EP3_DMA_ByteSize_0_1_2_2_0_0 , LC898124EP3_DMB_ByteSize_0_1_2_2_0_0 },
 {0xFFFF, (void*)0, 0, 0, (void*)0 ,0 ,0 }
};

const DOWNLOAD_TBL DTbl_S[] = {
 {0x0302, LC898124EP3_PM_0_1_3_2_1_0, LC898124EP3_PMSize_0_1_3_2_1_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_3_2_1_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_3_2_1_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_3_2_1_0), LC898124EP3_DM_0_1_3_2_1_0, LC898124EP3_DMA_ByteSize_0_1_3_2_1_0 , LC898124EP3_DMB_ByteSize_0_1_3_2_1_0 },
 {0x0382, LC898124EP3_PM_0_1_3_2_1_1, LC898124EP3_PMSize_0_1_3_2_1_1, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_3_2_1_1 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_3_2_1_1 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_3_2_1_1), LC898124EP3_DM_0_1_3_2_1_1, LC898124EP3_DMA_ByteSize_0_1_3_2_1_1 , LC898124EP3_DMB_ByteSize_0_1_3_2_1_1 },
 {0x0303, LC898124EP3_PM_0_1_3_3_1_0, LC898124EP3_PMSize_0_1_3_3_1_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_3_3_1_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_3_3_1_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_3_3_1_0), LC898124EP3_DM_0_1_3_3_1_0, LC898124EP3_DMA_ByteSize_0_1_3_3_1_0 , LC898124EP3_DMB_ByteSize_0_1_3_3_1_0 },
 {0x0383, LC898124EP3_PM_0_1_3_3_1_1, LC898124EP3_PMSize_0_1_3_3_1_1, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_3_3_1_1 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_3_3_1_1 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_3_3_1_1), LC898124EP3_DM_0_1_3_3_1_1, LC898124EP3_DMA_ByteSize_0_1_3_3_1_1 , LC898124EP3_DMB_ByteSize_0_1_3_3_1_1 },
 {0xFFFF, (void*)0, 0, 0, (void*)0 ,0 ,0 }		//20180910 komori
};

unsigned char SelectDownload( UINT8 GyroSelect, UINT8 ActSelect, UINT8 MasterSlave )
{
	DSPVER Dspcode;
	DOWNLOAD_TBL *ptr;

	if ( MasterSlave == 0x00 ) {
		ptr = ( DOWNLOAD_TBL *)DTbl_M;
	} else {
		ptr = ( DOWNLOAD_TBL *)DTbl_S;
	}

	/* どのCodeをDownloadするのかTableから検索 */
	while (ptr->Cmd != 0xFFFF ){
		if( ptr->Cmd == ( ((UINT16)ActSelect<<8) + GyroSelect) ) break;
		ptr++ ;
	}
	if (ptr->Cmd == 0xFFFF)	return(0xF0);

	/* Downloadする前CodeのInformation情報確認 */
	if( GetInfomationBeforeDownlaod( &Dspcode, ptr->DataDM, ( ptr->LengthDMA +  ptr->LengthDMB ) ) != 0 ){
		return(0xF1);
	}

	/* Downloadする前のCodeと、要求しているActuator/Gyro情報が一致しているか確認 */
	if( (ActSelect != Dspcode.ActType) || ((GyroSelect&0x7f) != Dspcode.GyroType) ) return(0xF2);

	// 高速化対応Download
TRACE("DataPM( %08x ), LengthPM( %08x ) , Parity( %08x ), DataDM( %08x ) , LengthDMA( %08x ) , LengthDMB( %08x ) \n"
	, (int)ptr->DataPM , (int)ptr->LengthPM , (int)ptr->Parity , (int)ptr->DataDM , (int)ptr->LengthDMA , (int)ptr->LengthDMB );
	return( DownloadToEP3( ptr->DataPM, ptr->LengthPM, ptr->Parity, ptr->DataDM, ptr->LengthDMA , ptr->LengthDMB ) ); 
}

