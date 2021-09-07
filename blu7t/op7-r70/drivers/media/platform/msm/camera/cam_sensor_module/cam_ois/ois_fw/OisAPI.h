//********************************************************************************
//		<< LC898124EP3 Evaluation Soft>>
//		Program Name	: OisAPI
// 		Explanation		: API List for customers
//		History			: First edition						2016.03.15 K.abe
//********************************************************************************
//****************************************************
//	API FUNCTION LIST	
//****************************************************
/* Program Download & Start */
extern void RemapMain( void );							// make download code start. 
extern unsigned char SelectDownload( UINT8 GyroSelect, UINT8 ActSelect, UINT8 MasterSlave );	// download

/* Code information check such as version */
extern UINT8 GetInfomationAfterDownload( DSPVER* Info );	// get code information after download.

/* Power off for SPI interface */
extern void PreparationForPowerOff( void );				// SPI interface stop before power off

/* E2Prom Access */
extern UINT8 E2PromVerification( void );				// E2Prom verification for user area by using check sum. O:SUCCESS, 1:FAUILRE
extern UINT8 E2PromVerificationONSEMI( void );			// E2Prom verification for ONSEMI area by using check sum. O:SUCCESS, 1:FAUILRE
extern void BurstReadE2Prom( UINT8 address, UINT8 * val, UINT8 cnt );	// E2Prom reading continuously for every area.
extern void ReadE2Prom( unsigned char address, unsigned char * val );	// Read data from E2Prom
#if ((SELECT_VENDOR & 0x80 ) != 0x80)
extern UINT8 WriteE2Prom( UINT8 address, UINT8 data );	// E2Prom writing for user area 1byte.
#endif //((SELECT_VENDOR & 0x80 ) != 0x80)

/* Gyro Offset Adjustment */
extern void GyroOffsetMeasureStart( void );				// start the gyro offset adjustment
extern UINT8 GetGyroOffset( UINT16* GyroOffsetX, UINT16* GyroOffsetY, INT16 GYROF_UPPER, INT16 GYROF_LOWER   );	// get the gyro offset values.
extern void	SetGyroOffset( UINT16 GyroOffsetX, UINT16 GyroOffsetY, UINT16 GyroOffsetZ );	// set the gyro offset data. before do this before Remapmain.

/* Angle correction for 45deg*/
extern UINT8 SetAngleCorrection( float DegreeGap, UINT8 SelectAct, unsigned char Arrangement );

extern void	SetGyroAccelCoef( UINT8 );

/* AF position Control [mandatory] */
extern void		SetTregAf( UINT16 UsTregAf );			// Bi-direction :  Min:000h Max:7FFh (11bit) 

/* Status Read and OIS enable [mandatory] */
extern	UINT8	RdStatus( UINT8 UcStBitChk );			// Status Read whether initialization finish or not.
extern	void	OisEna( void );							// OIS Enable Function

/* Others [option] */
extern	UINT8	RtnCen( UINT8	UcCmdPar ) ;			// Return to Center Function. Hall servo on/off
extern	UINT8	ZscCnt( UINT8 ) ;						// Zero servo control 
extern	UINT8	AfStbyRls( void );						// Af Standby release Command Function
extern	void	SetPanTiltMode( UINT8 UcPnTmod );		// default ON.
extern  void	SetSinWavePara( UINT8 , UINT8 ) ;		// Circle movement 

extern	UINT8	RunHea( void ) ;						// Hall Examination of Acceptance
#if ((SELECT_VENDOR & 0x80 ) != 0x80)
extern	UINT8	RunGea( void ) ;						// Gyro Examination of Acceptance
#endif //((SELECT_VENDOR & 0x80 ) != 0x80)

extern void GyroOffsetMeasureStartZ( void );			// start the gyro Z offset adjustment
extern UINT32	TneZeroServo( UINT8 , float );			/* zero servo adjustment command */
extern UINT8	ZeroServoLmt( UINT8 UCMODE );
extern UINT32	TneAvc( UINT8 ucposture );				// Accel offset adjustment

/* Accl Offset Adjustment */
extern void	SetAcclOffset( UINT16  AcclOffsetX, UINT16  AcclOffsetY, UINT16  AcclOffsetZ );	// set the accl offset data. before do this before Remapmain.
extern void	GetAcclOffset( UINT16* AcclOffsetX, UINT16* AcclOffsetY, UINT16* AcclOffsetZ );	// get the accl offset values.

extern void	LinearityCalculation( void );				// Linearity correction
extern void	CrosstalkCalculation( void );				// Crosstalk correction
extern void	SetOpticalOffset( void );					// optical offset


//****************************************************
//	API FUNCTION LIST	for module maker
//****************************************************
/* Calibration Main [mandatory] */
extern	UINT32	TneRun( void );							// calibration for bi-direction AF

/* E2Prom Access */
extern UINT8	WrHallCalData( void );					// upload the calibration data except gyro gain to eeprom
extern UINT8	WrOptOffsetData( void );				// upload the opt offset data
extern UINT8	WrGyroGainData( void );					// upload the gyro gain to eeprom
extern UINT8	WrAfParameter( UINT8 );					// upload af parameter to eeprom
extern UINT8	WrZeroServoData( void );				// upload the servo servo parameter to eepom

//extern UINT8	WrLinCrsData( UINT8 writemode );		// upload linearity & cross talk parameter to eeprom
#if ((SELECT_VENDOR & 0x80 ) != 0x80)
extern UINT8	WrHallLnData(  UINT8 UcMode, mlLinearityValue *linval );	// upload linearity position to eeprom
extern UINT8	WrMixCalData(  UINT8 UcMode, mlMixingValue *mixval );		// upload cross talk to eeprom
#endif //((SELECT_VENDOR & 0x80 ) != 0x80)

/* NOTEST */
//extern UINT8 	NotestMain( void );						// for NOTEST (E2prom erase & data set)
//extern	void	TimPro( void );							// for NOTEST (OSC measurement function for 1msec interruption)


