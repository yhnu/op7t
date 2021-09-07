//********************************************************************************
//
//		<< BMI260 Evaluation Soft>>
//		Program Name	: Ois_BOSCH.h
// 		Explanation		: BMI260 Global Declaration & ProtType Declaration
//		Design			: K.abe
//		History			: First edition	
//********************************************************************************
#define BMI260_SlvAddrWr	0xD0		// I2C Slave Address
#define BMI260_SlvAddrRd	0xD1		// I2C Slave Address

/************************************************/
/*	Command Addrss								*/
/************************************************/
#define CHIP_ID_260			0x00
#define INTERNAL_STATUS_260	0x21
#define	ACC_CONF_260		0x40
#define ACC_RANGE_260		0x41
#define	GYR_CONF_260		0x42
#define GYR_RANGE_260		0x43
#define INIT_CTRL_260		0x59
#define INIT_DATA_260		0x5E
#define IF_CONF_260			0x6B
#define PWR_CONF_260		0x7C
#define PWR_CTRL_260		0x7D

/************************************************/
/*	Data										*/
/************************************************/
#define AUX_EN_260			0
#define GYR_EN_260			1
#define ACC_EN_260			2
#define	TEMP_EN_260			3

