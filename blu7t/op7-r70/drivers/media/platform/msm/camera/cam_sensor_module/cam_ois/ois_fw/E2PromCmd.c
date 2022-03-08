//********************************************************************************
//		<< LC898124 Evaluation Soft >>
//********************************************************************************

//**************************
//	Include Header File		
//**************************
#include	"Ois.h"
#include	"OisAPI.h"

//****************************************************
//	LC898124 calibration parameters 
//****************************************************
#if ((SELECT_VENDOR&0x01) == 0x01)				// SEMCO
extern const ADJ_HALL SO2820_HallCalParameter;
extern const ADJ_LOPGAN SO2820_LoopGainParameter;
extern AF_PARA SO2820_OpenAfParameter;

extern const ADJ_HALL SO3600_HallCalParameter;
extern const ADJ_LOPGAN SO3600_LoopGainParameter;
extern AF_PARA SO3600_OpenAfParameter;
#endif

//****************************************************
//	CUSTOMER NECESSARY CREATING LIST
//****************************************************
/* for I2C communication */ 
extern	void RamWrite32A( int, int );
extern 	void RamRead32A( unsigned short, void * );
/* for Wait timer [Need to adjust for your system] */ 
extern void	WitTim( unsigned short	UsWitTim );

//****************************************************
//	EXTERN LIST
//****************************************************
extern void DMIOWrite32( UINT32 IOadrs, UINT32 IOdata );

  extern const AF_PARA SO_OpenAfParameter;
//****************************************************
//	LOCAL RAM LIST
//****************************************************
stAdjPar	StAdjPar ;				// temporary buffer for caribration data 
UINT16		UsGzoVal ;				// Gyro A/D Offset X

#ifdef	SEL_SHIFT_COR
stPosOff	StPosOff ;				//!< Execute Command Parameter
stAclVal	StAclVal ;				//!< Execute Command Parameter
#endif	//SEL_SHIFT_COR

#ifdef	ZERO_SERVO
stZeroServo			StZeroServoX;
stZeroServo			StZeroServoY;
stZeroServo			StZeroServoZ;
#endif	//ZERO_SERVO

//****************************************************
//	Parameter E2Prom defines
//****************************************************
#define HALLCROSSXX		0x7FFFFFFF
#define HALLCROSSXY		0x00000000
#define HALLCROSSYY		0x7FFFFFFF
#define HALLCROSSYX		0x00000000
#define HALLCROSSXSHIFT	0x00
#define HALLCROSSYSHIFT	0x00


//****************************************************
//	DEFINE LIST
//****************************************************
#define E2P_ONSEMI_AREA_SIZE	(8*2)
#define E2P_USER_AREA_SIZE		(8*13)

#define LSB 0
#define MSB 8 


#define CHECK_SUM_ADR	0x7D

//********************************************************************************
// Function Name 	: UnlockCodeSet
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: <Flash Memory> Unlock Code Set
// History			: First edition
//********************************************************************************
unsigned char UnlockCodeSet( void )
{
	unsigned long UlReadVal;

	DMIOWrite32( E2P_UNLK_CODE1, 0xAAAAAAAA );	// UNLK_CODE1(E0_7554h) = AAAA_AAAAh
	DMIOWrite32( E2P_UNLK_CODE2, 0x55555555 );	// UNLK_CODE2(E0_7AA8h) = 5555_5555h
	DMIOWrite32( E2P_RSTB, 		 0x00000001 );	// RSTB_FLA_WR(E0_74CCh[0])=1
	DMIOWrite32( E2P_CLKON, 	 0x00000010 );	// FLA_WR_ON(E0_7664h[4])=1
	DMIOWrite32( E2P_UNLK_CODE3, 0x0000ACD5 );	// Additional Unllock Code Set

	RamWrite32A( CMD_IO_ADR_ACCESS , E2P_WPB	) ;
	RamRead32A(  CMD_IO_DAT_ACCESS , &UlReadVal ) ;
	if ( (UlReadVal & 0x00000002) != 2 ) return(1);

	return(0);	

}

//********************************************************************************
// Function Name 	: UnlockCodeClear
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: <Flash Memory> Clear Unlock Code
// History			: First edition
//********************************************************************************
unsigned char UnlockCodeClear(void)
{
	unsigned long UlReadVal;

	RamWrite32A( CMD_IO_ADR_ACCESS, E2P_WPB );				// UNLOCK_CLR(E0_7014h[4])=1
	RamWrite32A( CMD_IO_DAT_ACCESS, 0x00000010 );
	RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	if( (UlReadVal & 0x00000080) != 0 )	return (3);						
	
	return(0);	
}

#if ((SELECT_VENDOR & 0x80 ) != 0x80)
//********************************************************************************
// Function Name 	: WriteE2Prom
// Retun Value		: error
// Argment Value	: NON
// Explanation		: Write data to E2Prom
// History			: First edition 						
//********************************************************************************
unsigned char WriteE2Prom( unsigned char address, unsigned char data )
{
	UINT32 UlReadVal, UlCnt;
	unsigned char ans;
		
	// Flash write€”õ
	ans = UnlockCodeSet();
	if ( ans != 0 ) return (ans);							// Unlock Code Set

	DMIOWrite32( E2P_ADR, address );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 			// FLG CLR
	
	switch ( (address & 0x0F) ){
		case 0 : 	DMIOWrite32( E2P_WDAT00, data );	break;
		case 1 : 	DMIOWrite32( E2P_WDAT01, data );	break;
		case 2 : 	DMIOWrite32( E2P_WDAT02, data );	break;
		case 3 : 	DMIOWrite32( E2P_WDAT03, data );	break;
		case 4 : 	DMIOWrite32( E2P_WDAT04, data );	break;
		case 5 : 	DMIOWrite32( E2P_WDAT05, data );	break;
		case 6 : 	DMIOWrite32( E2P_WDAT06, data );	break;
		case 7 : 	DMIOWrite32( E2P_WDAT07, data );	break;
		case 8 : 	DMIOWrite32( E2P_WDAT08, data );	break;
		case 9 : 	DMIOWrite32( E2P_WDAT09, data );	break;
		case 10 : 	DMIOWrite32( E2P_WDAT10, data );	break;
		case 11 : 	DMIOWrite32( E2P_WDAT11, data );	break;
		case 12 : 	DMIOWrite32( E2P_WDAT12, data );	break;
		case 13 : 	DMIOWrite32( E2P_WDAT13, data );	break;
		case 14 : 	DMIOWrite32( E2P_WDAT14, data );	break;
		case 15 : 	DMIOWrite32( E2P_WDAT15, data );	break;
	}	
	
	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){	ans = 2;	break;	} ;	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );
	
	UnlockCodeClear();							// Unlock Code Clear
	
	return(ans);

} 
#endif //((SELECT_VENDOR & 0x80 ) != 0x80)


//********************************************************************************
// Function Name 	: ReadE2Prom
// Retun Value		: data
// Argment Value	: NON
// Explanation		: Read data from E2Prom
// History			: First edition 						
//********************************************************************************
void ReadE2Prom( unsigned char address, unsigned char * val )
{
	UINT32 UlReadVal;

	DMIOWrite32( E2P_ADR, address );	// Start Address
	DMIOWrite32( E2P_ASCNT, 0 );		// Count Number
	DMIOWrite32( E2P_CMD, 1 ); 			// Re-Program
	// Read Exe
	RamWrite32A( CMD_IO_ADR_ACCESS, E2P_RDAT );
	RamRead32A ( CMD_IO_DAT_ACCESS, &UlReadVal );			// Read Access
	
	*val = (unsigned char)UlReadVal;
}

//********************************************************************************
// Function Name 	: BurstReadE2Prom
// Retun Value		: data
// Argment Value	: NON
// Explanation		: Read data from E2Prom
// History			: First edition 						
//********************************************************************************
void BurstReadE2Prom( unsigned char address, unsigned char * val, unsigned char cnt )
{
	UINT32 UlReadVal;
	unsigned char i;

	DMIOWrite32( E2P_ADR, address );	// Start Address
	DMIOWrite32( E2P_ASCNT, (cnt -1) );		// Count Number
	DMIOWrite32( E2P_CMD, 1 ); 			// Re-Program
	// Read Exe
	RamWrite32A( CMD_IO_ADR_ACCESS, E2P_RDAT );
	for(i=0; i<cnt; i++ ){
		RamRead32A ( CMD_IO_DAT_ACCESS, &UlReadVal );			// Read Access
		val[i] = (unsigned char)UlReadVal;
	}
}


//********************************************************************************
// Function Name 	: E2PromVerificationONSEMI
// Retun Value		: data
// Argment Value	: NON
// Explanation		: Read data from E2Prom
// History			: First edition 						
//********************************************************************************
UINT8 E2PromVerificationONSEMI( void )
{
	UINT32 Verify;
	UINT8 cnt;
	UINT8 temp[16];

	// Create Checksum
	Verify = 0;
	BurstReadE2Prom( EEPROM_ONSEMI_LDO , temp, E2P_ONSEMI_AREA_SIZE );
	for( cnt= 0 ; cnt <(E2P_ONSEMI_AREA_SIZE -1); cnt++ ){
		Verify += temp[cnt];
	}
	if ( (UINT8)Verify != temp[ (E2P_ONSEMI_AREA_SIZE -1) ])	return( FAILURE );

	return( SUCCESS	 );
}

//********************************************************************************
// Function Name 	: E2PromVerificationONSEMI
// Retun Value		: data
// Argment Value	: NON
// Explanation		: Read data from E2Prom
// History			: First edition 						
//********************************************************************************
UINT8 E2PromVerification( void )
{
	UINT32 Verify;
	UINT8 cnt;
	UINT8 temp[ E2P_USER_AREA_SIZE ];

	// Create Checksum
	Verify = 0;
	BurstReadE2Prom( EEPROM_ONSEMI_IDSEL , temp, E2P_USER_AREA_SIZE );
	for( cnt= 0; cnt < (E2P_USER_AREA_SIZE -1); cnt++ ){
		Verify += temp[cnt];
	}
	if ( (UINT8)Verify != temp[ (E2P_USER_AREA_SIZE-1) ])	return( FAILURE );
	return( SUCCESS	 );
}

//********************************************************************************
// Function Name 	: WrI2cSlaveAddr
// Retun Value		: error
// Argment Value	: NON
// Explanation		: Write data to E2Prom
// History			: First edition 						
//********************************************************************************
extern unsigned char I2cSlvAddrWr;

UINT8 WrI2cSlaveAddr( unsigned char Addr )
{
	UINT32 UlReadVal, UlCnt;
	UINT8 ans;

	ans = UnlockCodeSet();
	if ( ans != 0 ) return ( 1 );							// Unlock Code Set

	DMIOWrite32( E2P_ADR, 0x10 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR

	if( Addr == 0x74 ) {
		DMIOWrite32( E2P_WDAT00, 0x7F );		// IDSEL(Slave Addr = 0x74)
	} else {
		DMIOWrite32( E2P_WDAT00, 0xFF );		// IDSEL(Slave Addr = 0x7C)
	}

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 2 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

	return( 0 );
}


//********************************************************************************
// Function Name 	: WrHallCalData
// Retun Value		: error
// Argment Value	: NON
// Explanation		: Write data to E2Prom
// History			: First edition 						
//********************************************************************************
UINT8	WrHallCalData( void )
{
	UINT32 UlReadVal, UlCnt;
	UINT8 ans, data[CHECK_SUM_NUM], cnt, UcReadVal[2];
	UINT32 ReadVerify, Parity;

	// Read the Status & Update
	BurstReadE2Prom( EEPROM_Calibration_Status_LSB, UcReadVal, 2 );
	StAdjPar.StHalAdj.UlAdjPhs |= ( (((UINT32)UcReadVal[1]<<8) +UcReadVal[0]) & (~( HALL_CALB_FLG | HALL_CALB_BIT )) ); 
	
	// Flash write€”õ
	ans = UnlockCodeSet();
	if ( ans != 0 ) return ( 1 );							// Unlock Code Set
//------------------------------------------------------------------------------------------------
// Page 1 (0x10-0x1F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x10 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR

	if ( I2cSlvAddrWr == 0x7C ) {
		DMIOWrite32( E2P_WDAT00, 0xF7 );									// IDSEL
	} else {
		DMIOWrite32( E2P_WDAT00, 0x77 );									// IDSEL
	}

	DMIOWrite32( E2P_WDAT08, (UINT8)((StAdjPar.StHalAdj.UlAdjPhs)>>LSB ) ); // Calibration Status
	DMIOWrite32( E2P_WDAT09, (UINT8)((StAdjPar.StHalAdj.UlAdjPhs)>>MSB ) );
	DMIOWrite32( E2P_WDAT10, (UINT8)((StAdjPar.StHalAdj.UsHlxMxa)>>LSB ) ); // OIS Hall X Max After
	DMIOWrite32( E2P_WDAT11, (UINT8)((StAdjPar.StHalAdj.UsHlxMxa)>>MSB ) );
	DMIOWrite32( E2P_WDAT12, (UINT8)((StAdjPar.StHalAdj.UsHlxMna)>>LSB ) ); // OIS Hall X Min After	
	DMIOWrite32( E2P_WDAT13, (UINT8)((StAdjPar.StHalAdj.UsHlxMna)>>MSB ) );
	DMIOWrite32( E2P_WDAT14, (UINT8)((StAdjPar.StHalAdj.UsHlyMxa)>>LSB ) ); // OIS Hall Y Max After
	DMIOWrite32( E2P_WDAT15, (UINT8)((StAdjPar.StHalAdj.UsHlyMxa)>>MSB ) );

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 2 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );
//------------------------------------------------------------------------------------------------
// Page 2 (0x20-0x2F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x20 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
	
	DMIOWrite32( E2P_WDAT00, (UINT8)((StAdjPar.StHalAdj.UsHlyMna)>>LSB ) ); // OIS Hall Y Min After
	DMIOWrite32( E2P_WDAT01, (UINT8)((StAdjPar.StHalAdj.UsHlyMna)>>MSB ) );
	DMIOWrite32( E2P_WDAT02, (UINT8)((StAdjPar.StHalAdj.UsHlxGan)>>8 ) );		// OIS Hall Bias X
	DMIOWrite32( E2P_WDAT03, (UINT8)((StAdjPar.StHalAdj.UsHlxOff)>>8 ) );		// OIS Hall Offset X
	DMIOWrite32( E2P_WDAT04, (UINT8)((StAdjPar.StHalAdj.UsHlyGan)>>8 ) );		// OIS Hall Bias Y
	DMIOWrite32( E2P_WDAT05, (UINT8)((StAdjPar.StHalAdj.UsHlyOff)>>8 ) );		// OIS Hall Offset Y
	DMIOWrite32( E2P_WDAT06, (UINT8)((StAdjPar.StLopGan.UlLxgVal)>>16  ) );	// OIS Hall Loop Gain X
	DMIOWrite32( E2P_WDAT07, (UINT8)((StAdjPar.StLopGan.UlLxgVal)>>24  ) );	// OIS Hall Loop Gain X
	DMIOWrite32( E2P_WDAT08, (UINT8)((StAdjPar.StLopGan.UlLygVal)>>16  ) );   // OIS Hall Loop Gain Y
	DMIOWrite32( E2P_WDAT09, (UINT8)((StAdjPar.StLopGan.UlLygVal)>>24  ) );
	DMIOWrite32( E2P_WDAT10, (UINT8)((StAdjPar.StHalAdj.UsAdxOff)>>LSB ) );   // OIS Mecha center X
	DMIOWrite32( E2P_WDAT11, (UINT8)((StAdjPar.StHalAdj.UsAdxOff)>>MSB ) );
	DMIOWrite32( E2P_WDAT12, (UINT8)((StAdjPar.StHalAdj.UsAdyOff)>>LSB ) ); 	// OIS Mecha center Y
	DMIOWrite32( E2P_WDAT13, (UINT8)((StAdjPar.StHalAdj.UsAdyOff)>>MSB ) );
	DMIOWrite32( E2P_WDAT14, 0xFF );	//GyroFilterTableX_gxzoom
	DMIOWrite32( E2P_WDAT15, 0xFF );

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 3 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );
//------------------------------------------------------------------------------------------------
// Page 3 (0x30-0x32) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x30 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
	
	DMIOWrite32( E2P_WDAT00, 0xFF );	// GyroFilterTableX_gxzoom
	DMIOWrite32( E2P_WDAT01, 0x3F );
	DMIOWrite32( E2P_WDAT02, 0xFF );	//	GyroFilterTableY_gyzoom
	DMIOWrite32( E2P_WDAT03, 0xFF );
	DMIOWrite32( E2P_WDAT04, 0xFF );
	DMIOWrite32( E2P_WDAT05, 0x3F );	
	
	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 4 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );
//------------------------------------------------------------------------------------------------
// Page 5 (0x50-0x5F) 
//------------------------------------------------------------------------------------------------
#if 0
	DMIOWrite32( E2P_ADR, 0x50 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
	
	DMIOWrite32( E2P_WDAT06, 0 );	// OIS gyro offset X
	DMIOWrite32( E2P_WDAT07, (UINT8)(HALLCROSSXX>>16) );	// CrossXX lower Byte
	DMIOWrite32( E2P_WDAT08, (UINT8)(HALLCROSSXX>>24) );	// CrossXX Higher Byte
	DMIOWrite32( E2P_WDAT09, (UINT8)(HALLCROSSXY>>16) );	// CrossXY lower Byte
	DMIOWrite32( E2P_WDAT10, (UINT8)(HALLCROSSXY>>24) );	// CrossXY Higher Byte
	DMIOWrite32( E2P_WDAT11, (UINT8)(HALLCROSSYY>>16) );	// CrossYY lower Byte
	DMIOWrite32( E2P_WDAT12, (UINT8)(HALLCROSSYY>>24) );	// CrossYY Higher Byte
	DMIOWrite32( E2P_WDAT13, (UINT8)(HALLCROSSYX>>16) );	// CrossYX lower Byte
	DMIOWrite32( E2P_WDAT14, (UINT8)(HALLCROSSYX>>24) );	// CrossYX Higher Byte
	DMIOWrite32( E2P_WDAT15, HALLCROSSXSHIFT );	// CrsXsft
	DMIOWrite32( E2P_WDAT10, HALLCROSSYSHIFT );	// CrsYsft

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 4 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );
#endif
//------------------------------------------------------------------------------------------------
// Page 6 (0x50-0x5F) 
//------------------------------------------------------------------------------------------------
#if 0
	DMIOWrite32( E2P_ADR, 0x60 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
	
	DMIOWrite32( E2P_WDAT00, CH2SEL );	// DrvY direction 
	DMIOWrite32( E2P_WDAT01, CH3SEL );	// DrvZ direction 
	DMIOWrite32( E2P_WDAT02, (UINT8)(AF_FST_FREQ)      );	// AfFreq
	DMIOWrite32( E2P_WDAT03, (UINT8)(AF_FST_FREQ>>8)   );	// AfFreq
	DMIOWrite32( E2P_WDAT04, (UINT8)(AF_FST_UCOEF>>16) );	// AfUcode
	DMIOWrite32( E2P_WDAT05, (UINT8)(AF_FST_UCOEF>>24) );	// AfUcode
	DMIOWrite32( E2P_WDAT06, (UINT8)(AF_FST_DCOEF>>16) );	// AfDcode
	DMIOWrite32( E2P_WDAT07, (UINT8)(AF_FST_DCOEF>>24) );	// AfDcode

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 4 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );
#endif
//------------------------------------------------------------------------------------------------
// CheckSum Creating 
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom( EEPROM_ONSEMI_IDSEL, data, CHECK_SUM_NUM );
	Parity = 0;
	for( cnt=0; cnt < CHECK_SUM_NUM; cnt++ ){
		Parity +=  data[cnt];
	}

//------------------------------------------------------------------------------------------------
// Page 7 (0x70-0x7F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x70 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 				// FLG CLR

	DMIOWrite32( E2P_WDAT13, (UINT8)(Parity) ); // CheckSum:0x7D

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 5 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

	UnlockCodeClear();							// Unlock Code Clear
//------------------------------------------------------------------------------------------------
// Checksum Verification
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom( EEPROM_ONSEMI_IDSEL, data, CHECK_SUM_NUM );
	ReadVerify = 0;
	for( cnt=0; cnt < CHECK_SUM_NUM; cnt++ ){
		ReadVerify +=  data[cnt];
	}
	ReadE2Prom( CHECK_SUM_ADR, &cnt );
	Parity = cnt;
	if( (UINT8)ReadVerify != (UINT8)Parity)	return( 6 );  
	
	return( 0 );

} 
//********************************************************************************
// Function Name 	: WrGyroGainData
// Retun Value		: 0:OK, 1:NG
// Argment Value	: NON
// Explanation		: Flash Write Hall Calibration Data Function
// History			: First edition 						
//********************************************************************************
UINT8	WrGyroGainData( void )
{
	UINT32 UlReadVal, UlCnt;
	UINT8 ans, data[CHECK_SUM_NUM], cnt, UcReadVal[2];
	UINT32 ReadVerify, Parity;
	UINT32	UlZoomX, UlZoomY;

	// Read the Gyro Gain value
	RamRead32A(  GyroFilterTableX_gxzoom , &UlZoomX ) ;
	RamRead32A(  GyroFilterTableY_gyzoom , &UlZoomY ) ;
	// Read the Status & Update
	BurstReadE2Prom( EEPROM_Calibration_Status_LSB, UcReadVal, 2 );
	UlReadVal = (((UINT32)UcReadVal[1]<<8) +UcReadVal[0]) & (~GYRO_GAIN_FLG) ; 
		
	// Flash write€”õ
	ans = UnlockCodeSet();
	if ( ans != 0 ) return ( 1 );							// Unlock Code Set

//------------------------------------------------------------------------------------------------
// Page 1 (0x10-0x1F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x10 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR

//	DMIOWrite32( E2P_WDAT00,  );	// IDSEL	
	DMIOWrite32( E2P_WDAT08, (UINT8)((UlReadVal)>>LSB ) ); // Calibration Status
	DMIOWrite32( E2P_WDAT09, (UINT8)((UlReadVal)>>MSB ) );

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 2 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );
//------------------------------------------------------------------------------------------------
// Page 2 (0x20-0x2F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x20 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
	
	DMIOWrite32( E2P_WDAT14, (UINT8)(UlZoomX>>0) );//GyroFilterTableX_gxzoom
	DMIOWrite32( E2P_WDAT15, (UINT8)(UlZoomX>>8) );

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 3 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

//------------------------------------------------------------------------------------------------
// Page 3 (0x30-0x3F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x30 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
	
	DMIOWrite32( E2P_WDAT00, (UINT8)(UlZoomX>>16) );
	DMIOWrite32( E2P_WDAT01, (UINT8)(UlZoomX>>24) );
	DMIOWrite32( E2P_WDAT02, (UINT8)(UlZoomY>>0) );	//	GyroFilterTableY_gyzoom
	DMIOWrite32( E2P_WDAT03, (UINT8)(UlZoomY>>8) );
	DMIOWrite32( E2P_WDAT04, (UINT8)(UlZoomY>>16) );
	DMIOWrite32( E2P_WDAT05, (UINT8)(UlZoomY>>24) );	

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 3 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

//------------------------------------------------------------------------------------------------
// CheckSum Creating 
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom( EEPROM_ONSEMI_IDSEL, data, CHECK_SUM_NUM );
	Parity = 0;
	for( cnt=0; cnt < CHECK_SUM_NUM; cnt++ ){
		Parity +=  data[cnt];
	}

//------------------------------------------------------------------------------------------------
// Page 7 (0x70-0x7F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x70 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 				// FLG CLR

	DMIOWrite32( E2P_WDAT13, (UINT8)(Parity) ); // CheckSum:0x7D

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 5 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

	UnlockCodeClear();							// Unlock Code Clear
//------------------------------------------------------------------------------------------------
// Checksum Verification
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom( EEPROM_ONSEMI_IDSEL, data, CHECK_SUM_NUM );
	ReadVerify = 0;
	for( cnt=0; cnt < CHECK_SUM_NUM; cnt++ ){
		ReadVerify +=  data[cnt];
	}
	ReadE2Prom( CHECK_SUM_ADR, &cnt );
	Parity = cnt;
		
	if( (UINT8)ReadVerify != (UINT8)Parity)	return( 6 );  
	
	return(ans);
}

#if 0
//********************************************************************************
// Function Name 	: WrGyroGainData_NV
// Retun Value		: 0:OK, 1:NG
// Argment Value	: UlReadValX: gyro gain X, UlReadValY: gyro gain Y
// Explanation		: Flash Write Hall Calibration Data Function
// History			: First edition 						
//********************************************************************************
UINT8	WrGyroGainData_NV( UINT32 UlReadValX , UINT32 UlReadValY )
{

}
#endif


#ifdef	ZERO_SERVO
//********************************************************************************
// Function Name 	: WrZeroServoData
// Retun Value		: SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: Flash Write Zero Servo data Function
// History			: First edition 									2016.9.5
//********************************************************************************
UINT8	WrZeroServoData( void )
{
	UINT32 UlReadVal, UlCnt;
	UINT8 ans, data[CHECK_SUM_NUM], cnt, UcReadVal[2];
	UINT32 ReadVerify, Parity;
	
	// Read the Status & Update
	BurstReadE2Prom( EEPROM_Calibration_Status_LSB, UcReadVal, 2 );
	UlReadVal = (((UINT32)UcReadVal[1]<<8) +UcReadVal[0]) & (~ZSRV_CAL_FLG) ; 
	
	// Flash write€”õ
	ans = UnlockCodeSet();
	if ( ans != 0 ) return ( 1 );							// Unlock Code Set
	
//------------------------------------------------------------------------------------------------
// Page 1 (0x10-0x1F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x10 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
	
//	DMIOWrite32( E2P_WDAT00, 0xF7 );										// IDSEL
	DMIOWrite32( E2P_WDAT08, (UINT8)((UlReadVal)>>LSB ) ); // Calibration Status
	DMIOWrite32( E2P_WDAT09, (UINT8)((UlReadVal)>>MSB ) );

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 2 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );
	
//------------------------------------------------------------------------------------------------
// Page 3 (0x30-0x3F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x30 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
	
//	DMIOWrite32( E2P_WDAT00, (UINT8)(  );
//	DMIOWrite32( E2P_WDAT01, (UINT8)(  );
//	DMIOWrite32( E2P_WDAT02, (UINT8)(  );
//	DMIOWrite32( E2P_WDAT03, (UINT8)(  );
//	DMIOWrite32( E2P_WDAT04, (UINT8)(  );
//	DMIOWrite32( E2P_WDAT05, (UINT8)(  );
	DMIOWrite32( E2P_WDAT06, (UINT8)( ((StZeroServoX.SlOffset)>>16)>>LSB ) );
	DMIOWrite32( E2P_WDAT07, (UINT8)( ((StZeroServoX.SlOffset)>>16)>>MSB ) );
	DMIOWrite32( E2P_WDAT08, (UINT8)( ((StZeroServoX.SlShift) >> 0)>>LSB ) );
	DMIOWrite32( E2P_WDAT09, (UINT8)( ((StZeroServoX.SlShift) >> 0)>>MSB ) );
	DMIOWrite32( E2P_WDAT10, (UINT8)( ((StZeroServoX.SlGcora) >>16)>>LSB ) );
	DMIOWrite32( E2P_WDAT11, (UINT8)( ((StZeroServoX.SlGcora) >>16)>>MSB ) );
	DMIOWrite32( E2P_WDAT12, (UINT8)( ((StZeroServoX.SlGaina) >>16)>>LSB ) );
	DMIOWrite32( E2P_WDAT13, (UINT8)( ((StZeroServoX.SlGaina) >>16)>>MSB ) );
	DMIOWrite32( E2P_WDAT14, (UINT8)( ((StZeroServoY.SlOffset)>>16)>>LSB ) );
	DMIOWrite32( E2P_WDAT15, (UINT8)( ((StZeroServoY.SlOffset)>>16)>>MSB ) );

	DMIOWrite32( E2P_CMD, 2 ); 		// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 4 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

//------------------------------------------------------------------------------------------------
// Page 4 (0x40-0x4F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x40 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
	
	DMIOWrite32( E2P_WDAT00, (UINT8)( ((StZeroServoY.SlShift) >> 0)>>LSB ) );
	DMIOWrite32( E2P_WDAT01, (UINT8)( ((StZeroServoY.SlShift) >> 0)>>MSB ) );
	DMIOWrite32( E2P_WDAT02, (UINT8)( ((StZeroServoY.SlGcora) >>16)>>LSB ) );
	DMIOWrite32( E2P_WDAT03, (UINT8)( ((StZeroServoY.SlGcora) >>16)>>MSB ) );
	DMIOWrite32( E2P_WDAT04, (UINT8)( ((StZeroServoY.SlGaina) >>16)>>LSB ) );
	DMIOWrite32( E2P_WDAT05, (UINT8)( ((StZeroServoY.SlGaina) >>16)>>MSB ) );
	DMIOWrite32( E2P_WDAT06, (UINT8)( ((StZeroServoZ.SlOffset)>>16)>>LSB ) );
	DMIOWrite32( E2P_WDAT07, (UINT8)( ((StZeroServoZ.SlOffset)>>16)>>MSB ) );
//	DMIOWrite32( E2P_WDAT08, (UINT8)(  );
//	DMIOWrite32( E2P_WDAT09, (UINT8)(  );
//	DMIOWrite32( E2P_WDAT10, (UINT8)(  );
//	DMIOWrite32( E2P_WDAT11, (UINT8)(  );
//	DMIOWrite32( E2P_WDAT12, (UINT8)(  );
//	DMIOWrite32( E2P_WDAT13, (UINT8)(  );
//	DMIOWrite32( E2P_WDAT14, (UINT8)(  );
//	DMIOWrite32( E2P_WDAT15, (UINT8)(  );

	DMIOWrite32( E2P_CMD, 2 ); 		// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 4 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );
	
//------------------------------------------------------------------------------------------------
// CheckSum Creating 
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom( EEPROM_ONSEMI_IDSEL, data, CHECK_SUM_NUM );
	Parity = 0;
	for( cnt=0; cnt < CHECK_SUM_NUM; cnt++ ){
		Parity +=  data[cnt];
	}

//------------------------------------------------------------------------------------------------
// Page 7 (0x70-0x7F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x70 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 				// FLG CLR

	DMIOWrite32( E2P_WDAT13, (UINT8)(Parity) ); // CheckSum:0x7D

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 5 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

	UnlockCodeClear();							// Unlock Code Clear

//------------------------------------------------------------------------------------------------
// Checksum Verification
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom( EEPROM_ONSEMI_IDSEL, data, CHECK_SUM_NUM );
	ReadVerify = 0;
	for( cnt=0; cnt < CHECK_SUM_NUM; cnt++ ){
		ReadVerify +=  data[cnt];
	}
	ReadE2Prom( CHECK_SUM_ADR, &cnt );
	Parity = cnt;
	if( (UINT8)ReadVerify != (UINT8)Parity)	return( 6 );  
	
	return( 0 );
}

#endif	//ZERO_SERVO

//********************************************************************************
// Function Name 	: WrAfParameter
// Retun Value		: 0:OK, 1:NG
// Argment Value	: SelectAct
// Explanation		: Flash Write Af paramter Function
// History			: First edition 						
//********************************************************************************
UINT8	WrAfParameter( UINT8 SelectAct )
{
	UINT32 UlReadVal, UlCnt;
	UINT8 ans, data[CHECK_SUM_NUM], cnt;
	UINT32 ReadVerify, Parity;
	AF_PARA* AfParaPtr;
	// Select parameter
	
	if( SelectAct == ACT_SO2820 ){
		AfParaPtr = (AF_PARA*)&SO2820_OpenAfParameter;
	}else if( SelectAct == ACT_SO3600 ){
		AfParaPtr = (AF_PARA*)&SO3600_OpenAfParameter;
	}else{
		return( 7 );	/* Error */
	}

	// Flash write€”õ
	ans = UnlockCodeSet();
	if ( ans != 0 ) return ( 1 );							// Unlock Code Set

//------------------------------------------------------------------------------------------------
// Page 1 (0x10-0x1F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x10 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR

//	DMIOWrite32( E2P_WDAT00,  );	// IDSEL	
	DMIOWrite32( E2P_WDAT01, (UINT8)( AfParaPtr->DrvDir )); 					// Af driver direction
	DMIOWrite32( E2P_WDAT02, (UINT8)((AfParaPtr->Rrmd1ToMacro)>>LSB ) ); 	// Af rrmd1 para to macro
	DMIOWrite32( E2P_WDAT03, (UINT8)((AfParaPtr->Rrmd1ToMacro)>>MSB ) );
	DMIOWrite32( E2P_WDAT04, (UINT8)((AfParaPtr->Rrmd1ToInfini)>>LSB ) ); 	// Af rrmd1 para to inf
	DMIOWrite32( E2P_WDAT05, (UINT8)((AfParaPtr->Rrmd1ToInfini)>>MSB ) );
	DMIOWrite32( E2P_WDAT06, (UINT8)((AfParaPtr->Freq)>>LSB ) ); 			// Af freq
	DMIOWrite32( E2P_WDAT07, (UINT8)((AfParaPtr->Freq)>>MSB ) );

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 2 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

//------------------------------------------------------------------------------------------------
// CheckSum Creating 
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom( EEPROM_ONSEMI_IDSEL, data, CHECK_SUM_NUM );
	Parity = 0;
	for( cnt=0; cnt < CHECK_SUM_NUM; cnt++ ){
		Parity +=  data[cnt];
	}

//------------------------------------------------------------------------------------------------
// Page 7 (0x70-0x7F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x70 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 				// FLG CLR

	DMIOWrite32( E2P_WDAT13, (UINT8)(Parity) ); // CheckSum:0x7D

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 5 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

	UnlockCodeClear();							// Unlock Code Clear
//------------------------------------------------------------------------------------------------
// Checksum Verification
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom( EEPROM_ONSEMI_IDSEL, data, CHECK_SUM_NUM );
	ReadVerify = 0;
	for( cnt=0; cnt < CHECK_SUM_NUM; cnt++ ){
		ReadVerify +=  data[cnt];
	}
	ReadE2Prom( CHECK_SUM_ADR, &cnt );
	Parity = cnt;
		
	if( (UINT8)ReadVerify != (UINT8)Parity)	return( 6 );  
	
	DMIOWrite32( DRVCH3SEL,				(UINT32)AfParaPtr->DrvDir ); 					// Af driver direction
	RamWrite32A( OLAF_COEF_FSTVAL0,		((UINT32)AfParaPtr->Rrmd1ToMacro)<<16  ); 		// Af rrmd1 para to macro
	RamWrite32A( OLAF_COEF_FSTVAL1,		((UINT32)AfParaPtr->Rrmd1ToInfini)<<16 ); 		// Af rrmd1 para to inf
	RamWrite32A( OLAF_COEF_FSTVAL2, 	(UINT32)AfParaPtr->Freq ); 						// Af freq
	RamWrite32A( OLAF_DMB_FT, 			(UINT32)AfParaPtr->Freq ); 						// Af freq
	
	return(ans);
}

#if ((SELECT_VENDOR & 0x80 ) != 0x80)
//********************************************************************************
// Function Name 	: WrHallLnData
// Retun Value		: SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: EEPROM Write Hall Linearity data Function
// History			: First edition 									2017.6.27
//********************************************************************************
UINT8	WrHallLnData(  UINT8 UcMode, mlLinearityValue *linval )
{
	UINT32 UlReadVal, UlCnt;
	UINT8 ans, data[CHECK_SUM_NUM], cnt, UcReadVal[2];
	UINT32 ReadVerify, Parity;
	double		*pPosX, *pPosY;
	UINT32	PosDifX, PosDifY;
	DSPVER Info;
	
	// Read the Status & Update
	BurstReadE2Prom( EEPROM_Calibration_Status_LSB, UcReadVal, 2 );
	if( UcMode ){
		UlReadVal = (((UINT32)UcReadVal[1]<<8) +UcReadVal[0]) & (~HLLN_CALB_FLG) ; 
	}else{
		UlReadVal = (((UINT32)UcReadVal[1]<<8) +UcReadVal[0]) | (HLLN_CALB_FLG) ; 
	}
	
	// Flash write€”õ
	ans = UnlockCodeSet();
	if ( ans != 0 ) return ( 1 );							// Unlock Code Set
	
//------------------------------------------------------------------------------------------------
// Page 1 (0x10-0x1F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x10 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
	
//	DMIOWrite32( E2P_WDAT00, 0xF7 );										// IDSEL
	DMIOWrite32( E2P_WDAT08, (UINT8)((UlReadVal)>>LSB ) ); // Calibration Status
	DMIOWrite32( E2P_WDAT09, (UINT8)((UlReadVal)>>MSB ) );

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 2 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );
	
	if( GetInfomationAfterDownload( &Info ) != 0) return( EXE_ERROR );
//------------------------------------------------------------------------------------------------
// Page 4 (0x40-0x4F) 
//------------------------------------------------------------------------------------------------
		DMIOWrite32( E2P_ADR, 0x40 );	// Start Address
		DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
		
	if( UcMode ){

		if( Info.ActType == ACT_SO2820 ){
		//(XYSWAP == 0)
			pPosX = linval->positionX;
			pPosY = linval->positionY;
		}else if( Info.ActType == ACT_SO3600 ){
		//(XYSWAP == 1)
			pPosY = linval->positionX;
			pPosX = linval->positionY;
		}else{
		//(XYSWAP == 1)
			pPosY = linval->positionX;
			pPosX = linval->positionY;
		}

	//	DMIOWrite32( E2P_WDAT00, (UINT8)(  ) );
	//	DMIOWrite32( E2P_WDAT01, (UINT8)(  ) );
	//	DMIOWrite32( E2P_WDAT02, (UINT8)(  ) );
	//	DMIOWrite32( E2P_WDAT03, (UINT8)(  ) );
	//	DMIOWrite32( E2P_WDAT04, (UINT8)(  ) );
	//	DMIOWrite32( E2P_WDAT05, (UINT8)(  ) );
	//	DMIOWrite32( E2P_WDAT06, (UINT8)(  ) );
	//	DMIOWrite32( E2P_WDAT07, (UINT8)(  ) );
		DMIOWrite32( E2P_WDAT08, (UINT8)( (UINT32)(*pPosX * 10)>>LSB ) );				// POS1 X
		DMIOWrite32( E2P_WDAT09, (UINT8)( (UINT32)(*pPosX * 10)>>MSB ) );	pPosX++;	
		DMIOWrite32( E2P_WDAT10, (UINT8)( (UINT32)(*pPosY * 10)>>LSB ) );				// POS1 Y
		DMIOWrite32( E2P_WDAT11, (UINT8)( (UINT32)(*pPosY * 10)>>MSB ) );	pPosY++;
		DMIOWrite32( E2P_WDAT12, (UINT8)( (UINT32)(*pPosX * 10)>>LSB ) );				// POS2 X
		DMIOWrite32( E2P_WDAT13, (UINT8)( (UINT32)(*pPosX * 10)>>MSB ) );	pPosX++;
		DMIOWrite32( E2P_WDAT14, (UINT8)( (UINT32)(*pPosY * 10)>>LSB ) );				// POS2 Y
		DMIOWrite32( E2P_WDAT15, (UINT8)( (UINT32)(*pPosY * 10)>>MSB ) );	pPosY++;
	}else{
		DMIOWrite32( E2P_WDAT08, (UINT8)0xFF );		// POS1 X
		DMIOWrite32( E2P_WDAT09, (UINT8)0xFF );		
		DMIOWrite32( E2P_WDAT10, (UINT8)0xFF );		// POS1 Y
		DMIOWrite32( E2P_WDAT11, (UINT8)0xFF );	
		DMIOWrite32( E2P_WDAT12, (UINT8)0xFF );		// POS2 X
		DMIOWrite32( E2P_WDAT13, (UINT8)0xFF );	
		DMIOWrite32( E2P_WDAT14, (UINT8)0xFF );		// POS2 Y
		DMIOWrite32( E2P_WDAT15, (UINT8)0xFF );	
	}

		DMIOWrite32( E2P_CMD, 2 ); 		// Re-Program
		WitTim( 20 ) ;
		UlCnt=0;
		do{
			if( UlCnt++ > 10 ){
				UnlockCodeClear();							// Unlock Code Clear
				return( 4 );	
			}	
			RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
			RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
		}while ( (UlReadVal & 0x00000080) != 0 );

//------------------------------------------------------------------------------------------------
// Page 5 (0x50-0x5F) 
//------------------------------------------------------------------------------------------------
			DMIOWrite32( E2P_ADR, 0x50 );	// Start Address
			DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
			
		if( UcMode ){
			DMIOWrite32( E2P_WDAT00, (UINT8)( (UINT32)(*pPosX * 10)>>LSB ) );				// POS3 X
			DMIOWrite32( E2P_WDAT01, (UINT8)( (UINT32)(*pPosX * 10)>>MSB ) );	pPosX++;
			DMIOWrite32( E2P_WDAT02, (UINT8)( (UINT32)(*pPosY * 10)>>LSB ) );				// POS3 Y
			DMIOWrite32( E2P_WDAT03, (UINT8)( (UINT32)(*pPosY * 10)>>MSB ) );	pPosY++;
			DMIOWrite32( E2P_WDAT04, (UINT8)( (UINT32)(*pPosX * 10)>>LSB ) );				// POS4 X
			DMIOWrite32( E2P_WDAT05, (UINT8)( (UINT32)(*pPosX * 10)>>MSB ) );	pPosX++;
			DMIOWrite32( E2P_WDAT06, (UINT8)( (UINT32)(*pPosY * 10)>>LSB ) );				// POS4 Y
			DMIOWrite32( E2P_WDAT07, (UINT8)( (UINT32)(*pPosY * 10)>>MSB ) );	pPosY++;
			DMIOWrite32( E2P_WDAT08, (UINT8)( (UINT32)(*pPosX * 10)>>LSB ) );				// POS5 X
			DMIOWrite32( E2P_WDAT09, (UINT8)( (UINT32)(*pPosX * 10)>>MSB ) );	pPosX++;
			DMIOWrite32( E2P_WDAT10, (UINT8)( (UINT32)(*pPosY * 10)>>LSB ) );				// POS5 Y
			DMIOWrite32( E2P_WDAT11, (UINT8)( (UINT32)(*pPosY * 10)>>MSB ) );	pPosY++;
			DMIOWrite32( E2P_WDAT12, (UINT8)( (UINT32)(*pPosX * 10)>>LSB ) );				// POS6 X
			DMIOWrite32( E2P_WDAT13, (UINT8)( (UINT32)(*pPosX * 10)>>MSB ) );	pPosX++;
			DMIOWrite32( E2P_WDAT14, (UINT8)( (UINT32)(*pPosY * 10)>>LSB ) );				// POS6 Y
			DMIOWrite32( E2P_WDAT15, (UINT8)( (UINT32)(*pPosY * 10)>>MSB ) );	pPosY++;
		}else{
			DMIOWrite32( E2P_WDAT00, (UINT8)0xFF );	// POS3 X
			DMIOWrite32( E2P_WDAT01, (UINT8)0xFF );
			DMIOWrite32( E2P_WDAT02, (UINT8)0xFF );	// POS3 Y
			DMIOWrite32( E2P_WDAT03, (UINT8)0xFF );
			DMIOWrite32( E2P_WDAT04, (UINT8)0xFF );	// POS4 X
			DMIOWrite32( E2P_WDAT05, (UINT8)0xFF );
			DMIOWrite32( E2P_WDAT06, (UINT8)0xFF );	// POS4 Y
			DMIOWrite32( E2P_WDAT07, (UINT8)0xFF );
			DMIOWrite32( E2P_WDAT08, (UINT8)0xFF );	// POS5 X
			DMIOWrite32( E2P_WDAT09, (UINT8)0xFF );
			DMIOWrite32( E2P_WDAT10, (UINT8)0xFF );	// POS5 Y
			DMIOWrite32( E2P_WDAT11, (UINT8)0xFF );
			DMIOWrite32( E2P_WDAT12, (UINT8)0xFF );	// POS6 X
			DMIOWrite32( E2P_WDAT13, (UINT8)0xFF );
			DMIOWrite32( E2P_WDAT14, (UINT8)0xFF );	// POS6 Y
			DMIOWrite32( E2P_WDAT15, (UINT8)0xFF );
		}
			DMIOWrite32( E2P_CMD, 2 ); 		// Re-Program
			WitTim( 20 ) ;
			UlCnt=0;
			do{
				if( UlCnt++ > 10 ){
					UnlockCodeClear();							// Unlock Code Clear
					return( 4 );	
				}	
				RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
				RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
			}while ( (UlReadVal & 0x00000080) != 0 );
			
//------------------------------------------------------------------------------------------------
// Page 6 (0x60-0x6F) 
//------------------------------------------------------------------------------------------------
		DMIOWrite32( E2P_ADR, 0x60 );	// Start Address
		DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
		
		if( UcMode ){
			DMIOWrite32( E2P_WDAT00, (UINT8)( (UINT32)(*pPosX * 10)>>LSB ) );				// POS7 X
			DMIOWrite32( E2P_WDAT01, (UINT8)( (UINT32)(*pPosX * 10)>>MSB ) );
			DMIOWrite32( E2P_WDAT02, (UINT8)( (UINT32)(*pPosY * 10)>>LSB ) );				// POS7 Y
			DMIOWrite32( E2P_WDAT03, (UINT8)( (UINT32)(*pPosY * 10)>>MSB ) );

			if( Info.ActType == ACT_SO2820 ){
			//(XYSWAP == 0)
				PosDifX = (linval->dacX[1] - linval->dacX[0]); 
				PosDifY = (linval->dacY[1] - linval->dacY[0]); 
			}else if( Info.ActType == ACT_SO3600 ){
			//(XYSWAP == 1)
				PosDifY = (linval->dacX[1] - linval->dacX[0]); 
				PosDifX = (linval->dacY[1] - linval->dacY[0]); 
			}else{
			//(XYSWAP == 1)
				PosDifY = (linval->dacX[1] - linval->dacX[0]); 
				PosDifX = (linval->dacY[1] - linval->dacY[0]); 
			}

			DMIOWrite32( E2P_WDAT04, (UINT8)( (UINT16)(PosDifX>>16)>>LSB ) );				// STEP X
			DMIOWrite32( E2P_WDAT05, (UINT8)( (UINT16)(PosDifX>>16)>>MSB ) );
			DMIOWrite32( E2P_WDAT06, (UINT8)( (UINT16)(PosDifY>>16)>>LSB ) );				// STEP Y
			DMIOWrite32( E2P_WDAT07, (UINT8)( (UINT16)(PosDifY>>16)>>MSB ) );
		}else{
			DMIOWrite32( E2P_WDAT00, (UINT8)0xFF );				// POS7 X
			DMIOWrite32( E2P_WDAT01, (UINT8)0xFF );
			DMIOWrite32( E2P_WDAT02, (UINT8)0xFF );				// POS7 Y
			DMIOWrite32( E2P_WDAT03, (UINT8)0xFF );
			DMIOWrite32( E2P_WDAT04, (UINT8)0xFF );				// STEP X
			DMIOWrite32( E2P_WDAT05, (UINT8)0xFF );
			DMIOWrite32( E2P_WDAT06, (UINT8)0xFF );				// STEP Y
			DMIOWrite32( E2P_WDAT07, (UINT8)0xFF );
		}
	//	DMIOWrite32( E2P_WDAT08, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT09, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT10, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT11, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT12, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT13, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT14, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT15, (UINT8)( ) );

		DMIOWrite32( E2P_CMD, 2 ); 		// Re-Program
		WitTim( 20 ) ;
		UlCnt=0;
		do{
			if( UlCnt++ > 10 ){
				UnlockCodeClear();							// Unlock Code Clear
				return( 4 );	
			}	
			RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
			RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
		}while ( (UlReadVal & 0x00000080) != 0 );
		
//------------------------------------------------------------------------------------------------
// CheckSum Creating 
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom( EEPROM_ONSEMI_IDSEL, data, CHECK_SUM_NUM );
	Parity = 0;
	for( cnt=0; cnt < CHECK_SUM_NUM; cnt++ ){
		Parity +=  data[cnt];
	}

//------------------------------------------------------------------------------------------------
// Page 7 (0x70-0x7F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x70 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 				// FLG CLR

	DMIOWrite32( E2P_WDAT13, (UINT8)(Parity) ); // CheckSum:0x7D

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 5 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

	UnlockCodeClear();							// Unlock Code Clear

//------------------------------------------------------------------------------------------------
// Checksum Verification
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom( EEPROM_ONSEMI_IDSEL, data, CHECK_SUM_NUM );
	ReadVerify = 0;
	for( cnt=0; cnt < CHECK_SUM_NUM; cnt++ ){
		ReadVerify +=  data[cnt];
	}
	ReadE2Prom( CHECK_SUM_ADR, &cnt );
	Parity = cnt;
	if( (UINT8)ReadVerify != (UINT8)Parity)	return( 6 );  
	
	return( 0 );
}

//********************************************************************************
// Function Name 	: WrMixCalData
// Retun Value		: SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: EEPROM Writecross talk data Function
// History			: First edition 									2017.6.27
//********************************************************************************
UINT8	WrMixCalData(  UINT8 UcMode, mlMixingValue *mixval )
{
	UINT32 UlReadVal, UlCnt;
	UINT8 ans, data[CHECK_SUM_NUM], cnt, UcReadVal[2];
	UINT32 ReadVerify, Parity;
	DSPVER Info;
	
	// Read the Status & Update
	BurstReadE2Prom( EEPROM_Calibration_Status_LSB, UcReadVal, 2 );
	if( UcMode ){
		UlReadVal = (((UINT32)UcReadVal[1]<<8) +UcReadVal[0]) & (~MIXI_CALB_FLG) ; 
	}else{
		UlReadVal = (((UINT32)UcReadVal[1]<<8) +UcReadVal[0]) | (MIXI_CALB_FLG) ; 
	}
	
	// Flash write€”õ
	ans = UnlockCodeSet();
	if ( ans != 0 ) return ( 1 );							// Unlock Code Set
	
//------------------------------------------------------------------------------------------------
// Page 1 (0x10-0x1F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x10 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
	
//	DMIOWrite32( E2P_WDAT00, 0xF7 );										// IDSEL
	DMIOWrite32( E2P_WDAT08, (UINT8)((UlReadVal)>>LSB ) ); // Calibration Status
	DMIOWrite32( E2P_WDAT09, (UINT8)((UlReadVal)>>MSB ) );

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 2 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );
	
	if( GetInfomationAfterDownload( &Info ) != 0) return( EXE_ERROR );

//------------------------------------------------------------------------------------------------
// Page 6 (0x60-0x6F) 
//------------------------------------------------------------------------------------------------
		DMIOWrite32( E2P_ADR, 0x60 );	// Start Address
		DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
		
	//	DMIOWrite32( E2P_WDAT00, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT01, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT02, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT03, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT04, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT05, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT06, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT07, (UINT8)( ) );
	
	if( GetInfomationAfterDownload( &Info ) != 0) return( EXE_ERROR );
	
	if( Info.ActType == ACT_SO2820 ){
		
		mixval->hx45yL = (-1)*mixval->hx45yL;
		mixval->hy45xL = (-1)*mixval->hy45xL;
	
		if(mixval->hy45yL<0){			/* for MeasurementLibrary 1.X */
			mixval->hy45yL = (-1)*mixval->hy45yL;
			mixval->hx45yL = (-1)*mixval->hx45yL;
		}
	}else if( Info.ActType == ACT_SO3600 ){
		
		mixval->hx45yL = (-1)*mixval->hx45yL;
		mixval->hy45xL = (-1)*mixval->hy45xL;
	
		if(mixval->hy45yL<0){			/* for MeasurementLibrary 1.X */
			mixval->hy45yL = (-1)*mixval->hy45yL;
			mixval->hx45yL = (-1)*mixval->hx45yL;
		}
	} 
//*****************************************************//
//	if( XYSTPDIR == 0x10 || XYSTPDIR == 0x01  ){
//		mixval->hx45yL = (-1)*mixval->hx45yL;
//		mixval->hy45xL = (-1)*mixval->hy45xL;
//	}
//****************************************************//
	
	if( UcMode ){
		if( Info.ActType == ACT_SO2820 ){
			/* XYSWAP == 0 */
			DMIOWrite32( E2P_WDAT08, (UINT8)(( mixval->hx45xL>>16)>>LSB ) );
			DMIOWrite32( E2P_WDAT09, (UINT8)(( mixval->hx45xL>>16)>>MSB ) );
			DMIOWrite32( E2P_WDAT10, (UINT8)(( mixval->hx45yL>>16)>>LSB ) );
			DMIOWrite32( E2P_WDAT11, (UINT8)(( mixval->hx45yL>>16)>>MSB ) );
			DMIOWrite32( E2P_WDAT12, (UINT8)(( mixval->hy45yL>>16)>>LSB ) );
			DMIOWrite32( E2P_WDAT13, (UINT8)(( mixval->hy45yL>>16)>>MSB ) );
			DMIOWrite32( E2P_WDAT14, (UINT8)(( mixval->hy45xL>>16)>>LSB ) );
			DMIOWrite32( E2P_WDAT15, (UINT8)(( mixval->hy45xL>>16)>>MSB ) );
		}else if( Info.ActType == ACT_SO3600 ){
			/* XYSWAP == 1 */
			DMIOWrite32( E2P_WDAT08, (UINT8)(( mixval->hy45yL>>16)>>LSB ) );
			DMIOWrite32( E2P_WDAT09, (UINT8)(( mixval->hy45yL>>16)>>MSB ) );
			DMIOWrite32( E2P_WDAT10, (UINT8)(( mixval->hy45xL>>16)>>LSB ) );
			DMIOWrite32( E2P_WDAT11, (UINT8)(( mixval->hy45xL>>16)>>MSB ) );
			DMIOWrite32( E2P_WDAT12, (UINT8)(( mixval->hx45xL>>16)>>LSB ) );
			DMIOWrite32( E2P_WDAT13, (UINT8)(( mixval->hx45xL>>16)>>MSB ) );
			DMIOWrite32( E2P_WDAT14, (UINT8)(( mixval->hx45yL>>16)>>LSB ) );
			DMIOWrite32( E2P_WDAT15, (UINT8)(( mixval->hx45yL>>16)>>MSB ) );
		}else{
			/* XYSWAP == 1 */
			DMIOWrite32( E2P_WDAT08, (UINT8)(( mixval->hy45yL>>16)>>LSB ) );
			DMIOWrite32( E2P_WDAT09, (UINT8)(( mixval->hy45yL>>16)>>MSB ) );
			DMIOWrite32( E2P_WDAT10, (UINT8)(( mixval->hy45xL>>16)>>LSB ) );
			DMIOWrite32( E2P_WDAT11, (UINT8)(( mixval->hy45xL>>16)>>MSB ) );
			DMIOWrite32( E2P_WDAT12, (UINT8)(( mixval->hx45xL>>16)>>LSB ) );
			DMIOWrite32( E2P_WDAT13, (UINT8)(( mixval->hx45xL>>16)>>MSB ) );
			DMIOWrite32( E2P_WDAT14, (UINT8)(( mixval->hx45yL>>16)>>LSB ) );
			DMIOWrite32( E2P_WDAT15, (UINT8)(( mixval->hx45yL>>16)>>MSB ) );
		}
	}else{
		DMIOWrite32( E2P_WDAT08, (UINT8)0xFF );
		DMIOWrite32( E2P_WDAT09, (UINT8)0xFF );
		DMIOWrite32( E2P_WDAT10, (UINT8)0xFF );
		DMIOWrite32( E2P_WDAT11, (UINT8)0xFF );
		DMIOWrite32( E2P_WDAT12, (UINT8)0xFF );
		DMIOWrite32( E2P_WDAT13, (UINT8)0xFF );
		DMIOWrite32( E2P_WDAT14, (UINT8)0xFF );
		DMIOWrite32( E2P_WDAT15, (UINT8)0xFF );
	}

		DMIOWrite32( E2P_CMD, 2 ); 		// Re-Program
		WitTim( 20 ) ;
		UlCnt=0;
		do{
			if( UlCnt++ > 10 ){
				UnlockCodeClear();							// Unlock Code Clear
				return( 4 );	
			}	
			RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
			RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
		}while ( (UlReadVal & 0x00000080) != 0 );
		
//------------------------------------------------------------------------------------------------
// Page 7 (0x70-0x7F) 
//------------------------------------------------------------------------------------------------
		DMIOWrite32( E2P_ADR, 0x70 );	// Start Address
		DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR

	if( UcMode ){
		if( Info.ActType == ACT_SO2820 ){
			/* XYSWAP == 0 */
			DMIOWrite32( E2P_WDAT00, (UINT8)( mixval->hxsx ) );
			DMIOWrite32( E2P_WDAT01, (UINT8)( mixval->hysx ) );
		}else if( Info.ActType == ACT_SO3600 ){
			/* XYSWAP == 0 */
			DMIOWrite32( E2P_WDAT00, (UINT8)( mixval->hxsx ) );
			DMIOWrite32( E2P_WDAT01, (UINT8)( mixval->hysx ) );
		}else{
			/* XYSWAP == 1 */
			DMIOWrite32( E2P_WDAT00, (UINT8)( mixval->hysx ) );
			DMIOWrite32( E2P_WDAT01, (UINT8)( mixval->hxsx ) );
		}
	}else{
		DMIOWrite32( E2P_WDAT00, (UINT8)0xFF );
		DMIOWrite32( E2P_WDAT01, (UINT8)0xFF );
	}
	//	DMIOWrite32( E2P_WDAT02, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT03, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT04, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT05, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT06, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT07, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT08, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT09, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT10, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT11, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT12, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT13, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT14, (UINT8)( ) );
	//	DMIOWrite32( E2P_WDAT15, (UINT8)( ) );

		DMIOWrite32( E2P_CMD, 2 ); 		// Re-Program
		WitTim( 20 ) ;
		UlCnt=0;
		do{
			if( UlCnt++ > 10 ){
				UnlockCodeClear();							// Unlock Code Clear
				return( 4 );	
			}	
			RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
			RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
		}while ( (UlReadVal & 0x00000080) != 0 );
		
//	}
	
//------------------------------------------------------------------------------------------------
// CheckSum Creating 
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom( EEPROM_ONSEMI_IDSEL, data, CHECK_SUM_NUM );
	Parity = 0;
	for( cnt=0; cnt < CHECK_SUM_NUM; cnt++ ){
		Parity +=  data[cnt];
	}

//------------------------------------------------------------------------------------------------
// Page 7 (0x70-0x7F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x70 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 				// FLG CLR

	DMIOWrite32( E2P_WDAT13, (UINT8)(Parity) ); // CheckSum:0x7D

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
			return( 5 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

	UnlockCodeClear();							// Unlock Code Clear

//------------------------------------------------------------------------------------------------
// Checksum Verification
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom( EEPROM_ONSEMI_IDSEL, data, CHECK_SUM_NUM );
	ReadVerify = 0;
	for( cnt=0; cnt < CHECK_SUM_NUM; cnt++ ){
		ReadVerify +=  data[cnt];
	}
	ReadE2Prom( CHECK_SUM_ADR, &cnt );
	Parity = cnt;
	if( (UINT8)ReadVerify != (UINT8)Parity)	return( 6 );  
	
	return( 0 );
}
#endif //((SELECT_VENDOR & 0x80 ) != 0x80)

//********************************************************************************
// Function Name 	: WrOptOffsetData
// Retun Value		: error
// Argment Value	: NON
// Explanation		: Write opt offset data to E2Prom
// History			: First edition
//********************************************************************************
UINT8	WrOptOffsetData( void )
{
	UINT32 UlReadVal, UlCnt;
	UINT8 ans, data[CHECK_SUM_NUM], cnt;
	UINT32 ReadVerify, Parity;

	UINT32	UlHxoff;
	UINT32	UlHyoff;
	
	RamRead32A( ZeroServoRAM_X_OUT , &UlHxoff ) ;	// ZeroServoRAM_X_OUT // 0x03BC
	RamRead32A( ZeroServoRAM_Y_OUT , &UlHyoff ) ;	// ZeroServoRAM_Y_OUT // 0x03D4
//	UlHxoff = 0xFFFFFFFF;
//	UlHyoff = 0xFFFFFFFF;

TRACE("UlHxoff = %08X\n", UlHxoff);
TRACE("UlHyoff = %08X\n", UlHyoff);
	
	// Flash write€”õ
	ans = UnlockCodeSet();
	if ( ans != 0 ) {
		TRACE("Error 1 \n");
		return ( 1 );							// Unlock Code Set
	}

//------------------------------------------------------------------------------------------------
// Page 2 (0x72-0x75) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x70 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 		// FLG CLR
	
	DMIOWrite32( E2P_WDAT02, (UINT8)((UlHxoff)>>(16+LSB) ) );   // optical center X
	DMIOWrite32( E2P_WDAT03, (UINT8)((UlHxoff)>>(16+MSB) ) );
	DMIOWrite32( E2P_WDAT04, (UINT8)((UlHyoff)>>(16+LSB) ) ); 	// optical center Y
	DMIOWrite32( E2P_WDAT05, (UINT8)((UlHyoff)>>(16+MSB) ) );

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
TRACE("Error 2 \n");
			return( 3 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

//------------------------------------------------------------------------------------------------
// CheckSum Creating 
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom( EEPROM_ONSEMI_IDSEL, data, CHECK_SUM_NUM );
	Parity = 0;
	for( cnt=0; cnt < CHECK_SUM_NUM; cnt++ ){
		Parity +=  data[cnt];
	}

//------------------------------------------------------------------------------------------------
// Page 7 (0x70-0x7F) 
//------------------------------------------------------------------------------------------------
	DMIOWrite32( E2P_ADR, 0x70 );	// Start Address
	DMIOWrite32( E2P_DFG, 0 ); 				// FLG CLR

	DMIOWrite32( E2P_WDAT13, (UINT8)(Parity) ); // CheckSum:0x7D

	DMIOWrite32( E2P_CMD, 2 ); 			// Re-Program
	WitTim( 20 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ){
			UnlockCodeClear();							// Unlock Code Clear
TRACE("Error 3 \n");
			return( 5 );	
		}	
		RamWrite32A( CMD_IO_ADR_ACCESS , E2P_INT );
		RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

	UnlockCodeClear();							// Unlock Code Clear
//------------------------------------------------------------------------------------------------
// Checksum Verification
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom( EEPROM_ONSEMI_IDSEL, data, CHECK_SUM_NUM );
	ReadVerify = 0;
	for( cnt=0; cnt < CHECK_SUM_NUM; cnt++ ){
		ReadVerify +=  data[cnt];
	}
	ReadE2Prom( CHECK_SUM_ADR, &cnt );
	Parity = cnt;
	if( (UINT8)ReadVerify != (UINT8)Parity){
TRACE("Error 4 \n");
			return( 6 );  
	}
	
TRACE("Pass All \n");
	return( 0 );

} 
