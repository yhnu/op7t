/***********************************************************
* Description : OnePlus touchpanel driver
* 
* File		  : touch_interfaces.h
*
* Function	  : touchpanel public interface
* 
* Version	  : V1.0
*
***********************************************************/
#include <linux/spi/spi.h>
#ifndef TOUCH_INTERFACES_H
#define TOUCH_INTERFACES_H

#define MAX_I2C_RETRY_TIME 2

//---SPI READ/WRITE---
#define SPI_WRITE_MASK(a)	(a | 0x80)
#define SPI_READ_MASK(a)	(a & 0x7F)
#define DUMMY_BYTES (1)
#define SPI_TANSFER_LEN		512

typedef enum {
	SPIWRITE = 1,
	SPIREAD  = 2
} SPI_RW;

int touch_i2c_read_byte(struct i2c_client* client, u16 addr);
int touch_i2c_write_byte(struct i2c_client* client, u16 addr, unsigned char data);

int touch_i2c_read_word(struct i2c_client* client, u16 addr);
int touch_i2c_write_word(struct i2c_client* client, u16 addr, unsigned short data);

int touch_i2c_read_block(struct i2c_client* client, u16 addr, unsigned short length, unsigned char *data);
int touch_i2c_write_block(struct i2c_client* client, u16 addr, unsigned short length, unsigned char const *data);

int touch_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
int touch_i2c_write(struct i2c_client *client, char *writebuf, int writelen);

int init_touch_interfaces(struct device *dev, bool flag_register_16bit);
int touch_i2c_continue_read(struct i2c_client* client, unsigned short length, unsigned char *data);
int touch_i2c_continue_write(struct i2c_client* client, unsigned short length, unsigned char *data);
int32_t spi_read_write(struct spi_device *client, uint8_t *buf, size_t len, uint8_t *rbuf, SPI_RW rw);
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len);
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len);


#endif

