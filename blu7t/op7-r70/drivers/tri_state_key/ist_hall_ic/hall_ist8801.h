#ifndef __IST8801_H__
#define __IST8801_H__

#include <linux/ioctl.h>
#include <linux/mutex.h>

/* ********************************************************* */
/* feature of ic revision */
/* ********************************************************* */
#define IST8801_REV_0_2						(0x02)
#define IST8801_REV_1_0						(0x10)
#define IST8801_REV							IST8801_REV_1_0
#define IST8801_DRIVER_VERSION				"Ver1.00-190222"
/* ********************************************************* */


/*
SAD1 SAD0 == 00  0001100 R/W   (7bits)0x0C	(8bits)0x18
SAD1 SAD0 == 01  0001101 R/W   (7bits)0x0D	(8bits)0x1A
SAD1 SAD0 == 10  0001110 R/W   (7bits)0x0E	(8bits)0x1C
SAD1 SAD0 == 11  0001111 R/W   (7bits)0x0F	(8bits)0x1E
*/
/* ********************************************************* */

/* ********************************************************* */
/* register map */
/* ********************************************************* */
#define IST8801_REG_PERSINT					(0x00)
#define IST8801_VAL_PERSINT_COUNT(n)				(n<<4)

	/*
	*[7:4]	 PERS		 : interrupt persistence count
	*[0]	 INTCLR  = 1 : interrupt clear
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_INTSRS					(0x01)
#define IST8801_VAL_INTSRS_INT_ON				(0x80)
#define IST8801_DETECTION_MODE_INTERRUPT			IST8801_VAL_INTSRS_INT_ON
#define IST8801_VAL_INTSRS_INT_OFF				(0x00)
#define IST8801_DETECTION_MODE_POLLING				IST8801_VAL_INTSRS_INT_OFF
#define IST8801_VAL_INTSRS_INTTYPE_BESIDE			(0x00)
#define IST8801_VAL_INTSRS_INTTYPE_WITHIN			(0x10)
#define IST8801_VAL_INTSRS_ZGAIN				(0x00)
#define IST8801_VAL_INTSRS_ZGAIN_DIV_2				(0x01)
#define IST8801_VAL_INTSRS_ZGAIN_DIV_4				(0x02)
#define IST8801_VAL_INTSRS_ZGAIN_DIV_8				(0x03)
#define IST8801_VAL_INTSRS_ZGAIN_DIV_16				(0x04)
	/*
	*[7]	 INTON	 = 0 : disable interrupt
	*[7]	 INTON	 = 1 : enable interrupt
	*[4]	 INT_TYP  = 0 : generate interrupt when raw data is beside range of threshold
	*[4]	 INT_TYP  = 1 : generate interrupt when raw data is within range of threshold
	*[2:0]	 SRS		 : Sensitivity range and resolution : default 010
	*000	 : GAIN= ZGAIN (Register 0x54)
	*001	 : GAIN= ZGAIN / 2
	*010	 : GAIN= ZGAIN / 4
	*011	 : GAIN= ZGAIN / 8
	*100	 : GAIN= ZGAIN / 16
	*Others	 : GAIN= ZGAIN / 4
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_LTHL					(0x02)
	/*
	*[7:0]	 LTHL		 : low byte of low threshold value
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_LTHH					(0x03)
	/*
	*[7:0]	 LTHH		 : high byte of low threshold value with sign
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_HTHL					(0x04)
	/*
	*[7:0]	 HTHL		 : low byte of high threshold value
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_HTHH					(0x05)
	/*
	*[7:0]	 HTHH		 : high bye of high threshold value with sign
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_I2CDIS					(0x06)
#define IST8801_VAL_I2CDISABLE					(0x37)
	/*
	*[7:0]	 I2CDIS	 : disable i2c
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_SRST					(0x07)
#define IST8801_VAL_SRST_RESET					(0x01)
	/*
	*[0]	 SRST	 = 1 : soft reset
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_OPF						(0x08)
#define IST8801_VAL_OPF_STANDBY					(0x00)
#define IST8801_VAL_OPF_FREQ_10HZ				(0x10)
#define IST8801_VAL_OPF_FREQ_6_7HZ				(0x20)
#define IST8801_VAL_OPF_FREQ_5HZ				(0x30)
#define IST8801_VAL_OPF_FREQ_80HZ				(0x40)
#define IST8801_VAL_OPF_FREQ_40HZ				(0x50)
#define IST8801_VAL_OPF_FREQ_26_7HZ				(0x60)
#define IST8801_VAL_OPF_FREQ_20HZ				(0x70)
#define IST8801_VAL_OPF_SINGLE_MODE				(0x80)
#define IST8801_VAL_OPF_FREQ_100HZ				(0x90)
#define IST8801_VAL_OPF_FREQ_50HZ				(0xA0)
#define IST8801_VAL_OPF_FREQ_1HZ				(0xB0)
#define IST8801_VAL_OPF_FREQ_200HZ				(0xC0)
#define IST8801_VAL_OPF_FREQ_250HZ				(0xD0)
#define IST8801_VAL_OPF_FREQ_320HZ				(0xE0)
#define IST8801_VAL_OPF_FREQ_500HZ				(0xF0)
	/*
	[7:4]	OPF			: operation frequency
	*00	 : Standby mode
	*10	 : 10	 (Hz)
	*20	 : 6.7	 (Hz)
	*30	 : 5	 (Hz)
	*40  : 80	 (Hz)
	*50	 : 40	 (Hz)
	*60	 : 26.7  (Hz)
	*70	 : 20	 (Hz)
	*80	 : Single mode
	*90	 : 100	 (Hz)
	*A0	 : 50	 (Hz)
	*B0	 : 1	 (Hz)
	*C0	 : 200	 (Hz)
	*D0	 : 250	 (Hz)
	*E0	 : 320	 (Hz)
	*F0	 : 500	 (Hz)
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_DID						(0x09)
#define IST8801_VAL_DID						(0x81)
	/*
	*[7:0]	 DID		 : Device ID by OTP
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_ST1						(0x10)
#define IST8801_VAL_ST1_DRDY					(0x01)
	/*
	*[4] INTM			 : status of interrupt mode
	*[2] DORZ			 : 0 = no data overrun, 1 = data is overrun
	*[0] DRDY			 : status of data ready
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_HSL						(0x11)
	/*
	*[7:0]	 HSL		 : low byte of hall sensor measurement data
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_HSH						(0x12)
	/*
	*[7:0]	 HSL		 : high byte of hall sensor measurement data with sign
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_TDATAL					(0x13)
	/*
	*[7:6]	 HSL		 : Temperature Data, Low Byte
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_TDATAH					(0x14)
	/*
	*[7:6]	 HSL		 : Temperature Data, High Byte
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_USR_ODR_L					(0x1A)
	/*
	*[7:6]	 HSL		 : More User ODR Mode, Low Byte
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_USR_ODR_H					(0x1B)
	/*
	*[7:6]	 HSL		 : More User ODR Mode, High Byte
	*/
/* --------------------------------------------------------- */
#define IST8801_REG_ACTION					(0x20)
	/*
	*[1]	 ACTR		 : Action Register
	*/
/* ********************************************************* */

/* --------------------------------------------------------- */
#define IST8801_REG_CNTL2                                      (0x0D)
#define GAIN_1_TIME						(0x0 << 5)
#define GAIN_2_TIME						(0x1 << 5) //for 40mT
#define GAIN_4_TIME						(0x2 << 5) //for 20mT
#define GAIN_8_TIME						(0x3 << 5) //for 10mT
#define ADC_RES_15_BIT						(0x1 << 1)
#define ADC_RES_14_BIT						(0x2 << 1)
#define ADC_RES_13_BIT						(0x3 << 1)
#define ADC_RES_12_BIT						(0x4 << 1)
#define ADC_RES_11_BIT						(0x5 << 1)
#define ADC_RES_10_BIT						(0x6 << 1)
#define ADC_RES_9_BIT						(0x7 << 1)
#define ADC_RES_8_BIT						(0x8 << 1)
#define ADC_RES_16_BIT						(0x9 << 1) //other
        /*
	 *[6:5]  GAIN           : 1 times/2 times/4 times/8 times
	 *[4:1]  ADC_RS         : 15/14/13/12/11/10/9/8 bits
        */
/* ********************************************************* */
#define IST8801_REG_IFCNTL 					(0x40)
/* ********************************************************* */
#define IST8801_REG_GAINCNTL					(0x54)
/* ********************************************************* */
#define IST8801_REG_TSTCNTL 					(0x76)
/* ********************************************************* */
#define IST8801_REG_OSRCNTL 					(0x6c)
/* ********************************************************* */
#define IST8801_REG_INFO					(0x87)
/* ********************************************************* */
/* data type for driver */
/* ********************************************************* */

enum {
	OPERATION_MODE_POWERDOWN,
	OPERATION_MODE_MEASUREMENT,
	OPERATION_MODE_FUSEROMACCESS,
	OPERATION_MODE_SUSPEND
};

struct hall_srs {
	char name[12];
	uint8_t value;
	bool bias;
	uint32_t ratio;
};

/* ********************************************************* */
/* Need to set parameter for ist8801 driver */
/* ********************************************************* */
#define CURRENT_LOAD_UA					(100000)//100ma
#define IST8801_INTERRUPT_TYPE             		IST8801_VAL_INTSRS_INTTYPE_BESIDE
//#define IST8801_INTERRUPT_TYPE             		IST8801_VAL_INTSRS_INTTYPE_WITHIN
#define IST8801_PERSISTENCE_COUNT			IST8801_VAL_PERSINT_COUNT(15)
#define IST8801_SENSITIVITY_TYPE			0
#define IST8801_DETECTION_MODE 				IST8801_DETECTION_MODE_INTERRUPT
#define IST8801_REG_NUM					(16)
#define FREQUENCY                                       IST8801_VAL_OPF_FREQ_80HZ
//#define DYNAMIC_GAIN_ADC_BIT  				(GAIN_2_TIME | ADC_RES_12_BIT)
#define DYNAMIC_GAIN_ADC_BIT                            (GAIN_4_TIME | ADC_RES_10_BIT)
#define ENABLE_FILTER					0
#define FILTER_MODE					0 // 0 for light , 1 for weight
#define DISABLE_TEMP_CONPEN				0
/* ********************************************************* */

typedef union {
	struct {
		unsigned char persint;
		unsigned char intsrs;
		unsigned char lthl;
		unsigned char lthh;
		unsigned char hthl;
		unsigned char hthh;
		unsigned char i2cdis;
		unsigned char srst;
		unsigned char opf;
		unsigned char did;
		unsigned char info;
		unsigned char asa;
		unsigned char st1;
		unsigned char hsl;
		unsigned char hsh;
		unsigned char range;
	} map;
	unsigned char array[IST8801_REG_NUM];
} ist8801_reg_t;

typedef struct {
	struct i2c_client	*client;
	struct pinctrl *pctrl;
	struct pinctrl_state *power_state;
	struct pinctrl_state *irq_state;
	ist8801_reg_t		reg;
	bool			irq_enabled;
	unsigned int		id;
	int			calibrated_data;
	int			irq_source;
	short			value_30degree;
	short			value_70degree;
	short			thrhigh;
	short			thrlow;
	bool			last_state;
	struct delayed_work	work;

	struct regulator *	power_1v8;
	struct regulator *	power_2v8;
	int			irq_gpio;
	int			irq;
	unsigned int		power_gpio;
	bool			is_power_on;
	bool enable_hidden;
	unsigned int bias_ratio;
	uint8_t origin_gain;
	uint8_t origin_osr;
	uint8_t origin_info;
} ist8801_data_t;
/* ********************************************************* */

#endif /* __IST8801_H__ */
