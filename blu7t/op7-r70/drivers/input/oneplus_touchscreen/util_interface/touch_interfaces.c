#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>

#include "touch_interfaces.h"

#define TPD_DEVICE "touch_interface"
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)
#define FIX_I2C_LENGTH 256
static bool register_is_16bit = 0;

/**
 * touch_i2c_continue_read - Using for "read sequence bytes" through IIC
 * @client: Handle to slave device
 * @length: data size we want to read
 * @data: data read from IIC
 *
 * Actully, This function call i2c_transfer for IIC transfer,
 * Returning transfer length(transfer success) or most likely negative errno(transfer error)
 */
int touch_i2c_continue_read(struct i2c_client* client, unsigned short length, unsigned char *data)
{
    int retval;
    unsigned char retry;
    struct i2c_msg msg;

    msg.addr = client->addr;
    msg.flags = I2C_M_RD;
    msg.len = length;
    msg.buf = data;

    for (retry = 0; retry < MAX_I2C_RETRY_TIME; retry++) {
        if (i2c_transfer(client->adapter, &msg, 1) == 1) {
            retval = length;
            break;
        }
        msleep(20);
    }
    if (retry == MAX_I2C_RETRY_TIME) {
        dev_err(&client->dev, "%s: I2C read over retry limit\n", __func__);
        retval = -EIO;
    }
    return retval;

}

/**
 * touch_i2c_read_block - Using for "read word" through IIC
 * @client: Handle to slave device
 * @addr: addr to write
 * @length: data size we want to send
 * @data: data we want to send
 *
 * Actully, This function call i2c_transfer for IIC transfer,
 * Returning transfer length(transfer success) or most likely negative errno(transfer error)
 */
int touch_i2c_read_block(struct i2c_client* client, u16 addr, unsigned short length, unsigned char *data)
{
	int retval;
	unsigned char retry;
	static unsigned char *buffer = NULL;
	static unsigned int read_buf_size = 0;
	static unsigned char *read_buf = NULL;
	struct i2c_msg msg[2];

	buffer = kzalloc(2, GFP_KERNEL | GFP_DMA);
	if (length > FIX_I2C_LENGTH) {
		if (read_buf_size < length) {
			if (read_buf) {
				kfree(read_buf);
				TPD_INFO("read block_1,free onec\n");
			}
			read_buf =kzalloc(length, GFP_KERNEL);
			if(!read_buf) {
				read_buf_size = 0;
				TPD_INFO("read block kzaloc faied\n");
				return -ENOMEM;
			}
			read_buf_size = length;
		} else {
			memset(read_buf, 0 ,length);
		}
	} else {
		if (read_buf_size > FIX_I2C_LENGTH) {
			kfree(read_buf);
			read_buf = kzalloc(FIX_I2C_LENGTH,
				GFP_KERNEL | GFP_DMA);
			if(!read_buf) {
				read_buf_size = 0;
				TPD_INFO("read block kzaloc faied\n");
				return -ENOMEM;
			}
			read_buf_size = FIX_I2C_LENGTH;
		} else {
			if(!read_buf) {
				read_buf = kzalloc(FIX_I2C_LENGTH,
						GFP_KERNEL | GFP_DMA);
				if (!read_buf) {
					read_buf_size = 0;
					TPD_INFO("read block kzaloc faied\n");
					return -ENOMEM;
				}
				read_buf_size = FIX_I2C_LENGTH;
			} else {
				memset(read_buf, 0, length);
			}
		}
	}
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = buffer;

	if (!register_is_16bit) {  // if register is 8bit
		msg[0].len = 1;
		msg[0].buf[0] = addr & 0xff;
	} else {
		msg[0].len = 2;
		msg[0].buf[0] = addr >> 8 & 0xff;
		msg[0].buf[1] = addr * 0xff;
	}

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = length;
	msg[1].buf = read_buf;

	for (retry = 0; retry < MAX_I2C_RETRY_TIME; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		msleep(20);
	}
	if (retry == MAX_I2C_RETRY_TIME) {
		dev_err(&client->dev, "%s: I2C read over retry limit\n", __func__);
		retval = -EIO;
	}
	memcpy(data, read_buf,length);
	kfree(buffer);
	return retval;
}

/**
 * touch_i2c_continue_write - Using for "write sequence bytes" through IIC
 * @client: Handle to slave device
 * @length: data size we want to write
 * @data: data write to IIC
 *
 * Actully, This function call i2c_transfer for IIC transfer,
 * Returning transfer length(transfer success) or most likely negative errno(transfer error)
 */
int touch_i2c_continue_write(struct i2c_client* client, unsigned short length, unsigned char *data)
{
    int retval;
    unsigned char retry;
    struct i2c_msg msg;

    msg.addr = client->addr;
    msg.flags = 0;
    msg.buf = data;
    msg.len = length;

    for (retry = 0; retry < MAX_I2C_RETRY_TIME; retry++) {
        if (i2c_transfer(client->adapter, &msg, 1) == 1) {
            retval = length;
            break;
        }
        msleep(20);
    }
    if (retry == MAX_I2C_RETRY_TIME) {
        dev_err(&client->dev, "%s: I2C write over retry limit\n", __func__);
        retval = -EIO;
    }
    return retval;
}

/**
 * touch_i2c_write_block - Using for "read word" through IIC
 * @client: Handle to slave device
 * @addr: addr to write
 * @length: data size we want to send
 * @data: data we want to send
 *
 * Actully, This function call i2c_transfer for IIC transfer,
 * Returning transfer length(transfer success) or most likely negative errno(transfer error)
 */
int touch_i2c_write_block(struct i2c_client* client, u16 addr, unsigned short length, unsigned char const *data)
{
    int retval;
    unsigned char retry;
    //unsigned char buffer[length + 2];
    unsigned char *buffer;
    struct i2c_msg msg[1];
    buffer = (unsigned char *)kzalloc(length + 2,GFP_KERNEL);
    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].buf = buffer;

    if (!register_is_16bit)  // if register is 8bit
    {
        msg[0].len = length + 1;
        msg[0].buf[0] = addr & 0xff;

        memcpy(&buffer[1], &data[0], length);
    }
    else
    {
        msg[0].len = length + 2;
        msg[0].buf[0] = (addr >> 8) & 0xff;
        msg[0].buf[1] = addr & 0xff;

        memcpy(&buffer[2], &data[0], length);
    }

    for (retry = 0; retry < MAX_I2C_RETRY_TIME; retry++) {
        if (i2c_transfer(client->adapter, msg, 1) == 1) {
            retval = length;
            break;
        }
        msleep(20);
    }
    if (retry == MAX_I2C_RETRY_TIME) {
        dev_err(&client->dev, "%s: I2C write over retry limit\n", __func__);
        retval = -EIO;
    }
    kfree(buffer);
    return retval;
}

/**
 * touch_i2c_read_byte - Using for "read word" through IIC
 * @client: Handle to slave device
 * @addr: addr to read
 *
 * Actully, This function call touch_i2c_read_block for IIC transfer,
 * Returning zero(transfer success) or most likely negative errno(transfer error)
 */
int touch_i2c_read_byte(struct i2c_client* client, unsigned short addr)
{
    int retval = 0;
    unsigned char buf[2] = {0};

    if (!client)    {
        dump_stack();
        return -1;
    }
    retval = touch_i2c_read_block(client, addr, 1, buf);
    if (retval >= 0)
        retval = buf[0] & 0xff;

    return retval;
}


/**
 * touch_i2c_write_byte - Using for "read word" through IIC
 * @client: Handle to slave device
 * @addr: addr to write
 * @data: data we want to send
 *
 * Actully, This function call touch_i2c_write_block for IIC transfer,
 * Returning zero(transfer success) or most likely negative errno(transfer error)
 */
int touch_i2c_write_byte(struct i2c_client* client, unsigned short addr, unsigned char data)
{
    int retval;
    int length_trans = 1;
    unsigned char data_send = data;

    if (!client)    {
        dump_stack();
        return -EINVAL;
    }
    retval = touch_i2c_write_block(client, addr, length_trans, &data_send);
    if (retval == length_trans)
        retval = 0;

    return retval;
}

/**
 * touch_i2c_read_word - Using for "read word" through IIC
 * @client: Handle to slave device
 * @addr: addr to write
 * @data: data we want to read
 *
 * Actully, This func call touch_i2c_Read_block for IIC transfer,
 * Returning negative errno else a 16-bit unsigned "word" received from the device.
 */
int touch_i2c_read_word(struct i2c_client* client, unsigned short addr)
{
    int retval;
    unsigned char buf[2] = {0};

    if (!client)    {
        dump_stack();
        return -EINVAL;
    }
    retval = touch_i2c_read_block(client, addr, 2, buf);
    if (retval >= 0)
        retval = buf[1] << 8 | buf[0];

    return retval;
}

/**
 * touch_i2c_write_word - Using for "read word" through IIC
 * @client: Handle to slave device
 * @addr: addr to write
 * @data: data we want to send
 *
 * Actully, This function call touch_i2c_write_block for IIC transfer,
 * Returning zero(transfer success) or most likely negative errno(transfer error)
 */
int touch_i2c_write_word(struct i2c_client* client, unsigned short addr, unsigned short data)
{
    int retval;
    int length_trans = 2;
    unsigned char buf[2] = {data & 0xff, (data >> 8) & 0xff};

    if (!client)    {
        dump_stack();
        return -EINVAL;
    }

    retval = touch_i2c_write_block(client, addr, length_trans, buf);
    if (retval == length_trans)
        retval = 0;

    return retval;
}

/**
 * touch_i2c_read - Using for "read data from ic after writing or not" through IIC
 * @client: Handle to slave device
 * @writebuf: buf to write
 * @writelen: data size we want to send
 * @readbuf:  buf we want save data
 * @readlen:  data size we want to receive
 *
 * Actully, This function call i2c_transfer for IIC transfer,
 * Returning transfer msg length(transfer success) or most likely negative errno(transfer EIO error)
 */
int touch_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
    int retval = 0;
    int retry = 0;

    if (client == NULL) {
        TPD_INFO("%s: i2c_client == NULL!\n", __func__);
        return -1;
    }

    if (readlen > 0) {
        if (writelen > 0) {
            struct i2c_msg msgs[] =
            {
                {
                    .addr = client->addr,
                    .flags = 0,
                    .len = writelen,
                    .buf = writebuf,
                },
                {
                    .addr = client->addr,
                    .flags = I2C_M_RD,
                    .len = readlen,
                    .buf = readbuf,
                },
            };

            for (retry = 0; retry < MAX_I2C_RETRY_TIME; retry++) {
                if (i2c_transfer(client->adapter, msgs, 2) == 2) {
                    retval = 2;
                    break;
                }
                msleep(20);
            }
        } else {
            struct i2c_msg msgs[] =
            {
                {
                    .addr = client->addr,
                    .flags = I2C_M_RD,
                    .len = readlen,
                    .buf = readbuf,
                },
            };

            for (retry = 0; retry < MAX_I2C_RETRY_TIME; retry++) {
                if (i2c_transfer(client->adapter, msgs, 1) == 1) {
                    retval = 1;
                    break;
                }
                msleep(20);
            }
        }

        if (retry == MAX_I2C_RETRY_TIME) {
            TPD_INFO("%s: i2c_transfer(read) over retry limit\n", __func__);
            retval = -EIO;
        }
    }

    return retval;
}

/**
 * touch_i2c_write - Using for "write data to ic" through IIC
 * @client: Handle to slave device
 * @writebuf: buf data wo want to send
 * @writelen: data size we want to send
 *
 * Actully, This function call i2c_transfer for IIC transfer,
 * Returning transfer msg length(transfer success) or most likely negative errno(transfer EIO error)
 */
int touch_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
    int retval = 0;
    int retry = 0;

    if (client == NULL) {
        TPD_INFO("%s: i2c_client == NULL!", __func__);
        return -1;
    }

    if (writelen > 0) {
        struct i2c_msg msgs[] =
        {
            {
                .addr = client->addr,
                .flags = 0,
                .len = writelen,
                .buf = writebuf,
            },
        };

        for (retry = 0; retry < MAX_I2C_RETRY_TIME; retry++) {
            if (i2c_transfer(client->adapter, msgs, 1) == 1) {
                retval = 1;
                break;
            }
            msleep(20);
        }
        if (retry == MAX_I2C_RETRY_TIME) {
            TPD_INFO("%s: i2c_transfer(write) over retry limit\n", __func__);
            retval = -EIO;
        }
    }

    return retval;
}


/**
 * init_touch_interfaces - Using for Register IIC interface
 * @dev: i2c_client->dev using to alloc memory for dma transfer
 * @flag_register_16bit: bool param to detect whether this device using 16bit IIC address or 8bit address
 *
 * Actully, This function don't have many operation, we just detect device address length && alloc DMA memory for MTK platform
 * Returning zero(sucess) or -ENOMEM(memory alloc failed)
 */
int init_touch_interfaces(struct device *dev, bool flag_register_16bit)
{
    register_is_16bit = flag_register_16bit;

    return 0;
}

/*******************************************************
Description:
	Novatek touchscreen spi read/write core function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
int32_t spi_read_write(struct spi_device *client, uint8_t *buf, size_t len , uint8_t *rbuf, SPI_RW rw)
{
	struct spi_message m;
	struct spi_transfer t = {
		.len    = len,
	};

	switch (rw) {
		case SPIREAD:
			t.tx_buf = &buf[0];
			t.rx_buf = rbuf;
			t.len    = (len + DUMMY_BYTES);
			break;

		case SPIWRITE:
			t.tx_buf = buf;
			break;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(client, &m);
}

/*******************************************************
Description:
	Novatek touchscreen spi read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;
        uint8_t rbuf[SPI_TANSFER_LEN+1] = {0};

	buf[0] = SPI_READ_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, rbuf, SPIREAD);
		if (ret == 0) break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		TPD_INFO("read error, ret=%d\n", ret);
		ret = -EIO;
	} else {
		memcpy((buf+1), (rbuf+2), (len-1));
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen spi write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	buf[0] = SPI_WRITE_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NULL, SPIWRITE);
		if (ret == 0)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		TPD_INFO("error, ret=%d\n", ret);
		ret = -EIO;
	}

	return ret;
}

