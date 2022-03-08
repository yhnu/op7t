/****************************************************************************************
** Copyright (C), 2013-2018, ONEPLUS 
** File        : oneplus_m1120_up.c
**
** Description : 
**      Definitions for m1120 digital hall_up sensor, i2c address is 0x0C  
**
****************************************************************************************/

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
//#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/slab.h>
//#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "oneplus_m1120.h"
#include "../oneplus_motor.h"

/************************begin of declaration************************/

/************************static global variable************************/
static DEFINE_MUTEX(m1120_up_i2c_mutex);
static DEFINE_MUTEX(m1120_up_device_enable_mutex);
static dhall_data_t *g_hall_data;

/************************function about i2c************************/
static int   m1120_up_i2c_read_block(dhall_data_t* dhall_data, u8 addr, u8 *data, u8 len);
static int   m1120_up_i2c_write_block(dhall_data_t* dhall_data, u8 addr, u8 *data, u8 len);
static void  m1120_up_short_to_2byte(dhall_data_t* dhall_data, short x, u8 *hbyte, u8 *lbyte);
static short m1120_up_2byte_to_short(dhall_data_t* dhall_data, u8 hbyte, u8 lbyte);

/************************internal interrupt function************************/
static irqreturn_t   m1120_up_irq_handler(int irq, void *dev_id);
static int   m1120_up_clear_interrupt(dhall_data_t* dhall_data);

/************************internal power control function************************/
static int   m1120_up_power_init(dhall_data_t *data);
static int   m1120_up_power_ctl(dhall_data_t* dhall_data, bool on);
static int   m1120_up_set_operation_mode(dhall_data_t* dhall_data, int mode);
//static int   m1120_up_set_power(struct device *dev, bool on);
static bool  m1120_up_get_enable(dhall_data_t* dhall_data);
static void  m1120_up_set_enable(dhall_data_t* dhall_data, bool enable);

/************interface implement for abstract control level************/
static bool  m1120_up_is_power_on(void);
static int   m1120_up_parse_dt(dhall_data_t* pdata);
static int   m1120_up_set_enable_state(bool enable);
static bool  m1120_up_get_enable_state(void);
static int   m1120_up_get_real_data(short *data);
static int   m1120_up_get_abs_data( short *data);
static int   m1120_up_set_detection_mode(u8 mode);
static int   m1120_up_enable_irq(bool enable);
static int   m1120_up_clear_irq(void);
static int   m1120_up_get_irq_state(void);
static bool  m1120_up_update_threshold(int position, short lowthd, short highthd);
static void  m1120_up_dump_reg(u8* buf);
static int   m1120_up_set_reg(int reg, int val);

/************************init function************************/
static int   m1120_up_init_device(dhall_data_t* hall_data);
static int   m1120_up_reset_device(dhall_data_t* dhall_data);
static int   m1120_up_power_init(dhall_data_t* dhall_data);

/************************end of declaration************************/


 /*********************************************************************
                        function about i2c
 **********************************************************************/
static int m1120_up_i2c_read_block(dhall_data_t* dhall_data, u8 addr, u8 *data, u8 len)
{
	u8 reg_addr = addr;
	int err = 0;
	struct i2c_client *client = NULL;
	struct i2c_msg msgs[2]={{0},{0}};
    dhall_data_t* p_hall_data = dhall_data;

    if (dhall_data == NULL) {
		MOTOR_ERR("m1120_up_i2c_read_block, dhall_data == NULL \n");
		return -EINVAL;
    }
    client = dhall_data->client;

	if (!client) {
		MOTOR_ERR("client null\n");
		return -EINVAL;
	} else if (len >= M1120_I2C_BUF_SIZE) {
		MOTOR_ERR(" length %d exceeds %d\n", len, M1120_I2C_BUF_SIZE);
		return -EINVAL;
	}

	mutex_lock(&m1120_up_i2c_mutex); 

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
 	msgs[0].len =1;
 	msgs[0].buf = &reg_addr;

 	msgs[1].addr = client->addr;
 	msgs[1].flags = I2C_M_RD;
 	msgs[1].len =len;
 	msgs[1].buf = data;

 	err = i2c_transfer(client->adapter, msgs, (sizeof(msgs) / sizeof(msgs[0])));

 	if (err < 0) {
 		MOTOR_ERR("i2c_transfer error: (%d %p %d) %d\n",addr, data, len, err);
 		err = -EIO;
 	} else  {
 		err = 0;
 	}
 	mutex_unlock(&m1120_up_i2c_mutex);

 	return err;

}

static int m1120_up_i2c_write_block(dhall_data_t* dhall_data, u8 addr, u8 *data, u8 len)
{
	int err = 0;
	int idx = 0;
	int num = 0;
	char buf[M1120_I2C_BUF_SIZE] ={0};
	struct i2c_client *client = NULL;
    dhall_data_t* p_hall_data = dhall_data;

    if (p_hall_data == NULL) {
		MOTOR_ERR("m1120_up_i2c_write_block failed, p_hall_data == NULL \n");
		return -EINVAL;
    }
    client = p_hall_data->client;

	if (!client) {
		MOTOR_ERR("client null \n");
		return -EINVAL;
	} else if (len >= M1120_I2C_BUF_SIZE) {
		MOTOR_ERR(" length %d exceeds %d \n", len, M1120_I2C_BUF_SIZE);
		return -EINVAL;
	}

	mutex_lock(&m1120_up_i2c_mutex);

	buf[num++] = addr;
	for (idx = 0; idx < len; idx++) {
		buf[num++] = data[idx];
	}
    
	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		MOTOR_ERR("send command error!! %d\n",err);
	}

	//store reg written
	if (len == 1) {
		switch(addr){
		case M1120_REG_PERSINT:
			p_hall_data->reg.map.persint = data[0];
			break;
		case M1120_REG_INTSRS:
			p_hall_data->reg.map.intsrs = data[0];
			break;
		case M1120_REG_LTHL:
			p_hall_data->reg.map.lthl = data[0];
			break;
		case M1120_REG_LTHH:
			p_hall_data->reg.map.lthh = data[0];
			break;
		case M1120_REG_HTHL:
			p_hall_data->reg.map.hthl = data[0];
			break;
		case M1120_REG_HTHH:
			p_hall_data->reg.map.hthh = data[0];
			break;
		case M1120_REG_I2CDIS:
			p_hall_data->reg.map.i2cdis = data[0];
			break;
		case M1120_REG_SRST:
			p_hall_data->reg.map.srst = data[0];
			break;
		case M1120_REG_OPF:
			p_hall_data->reg.map.opf = data[0];
			break;
		}
	}
	mutex_unlock(&m1120_up_i2c_mutex);
	return err;
}

static void m1120_up_short_to_2byte(dhall_data_t* dhall_data, short x, u8 *hbyte, u8 *lbyte)
{
    dhall_data_t* p_hall_data = dhall_data;

    if (p_hall_data == NULL) {
		MOTOR_ERR("m1120_up_short_to_2byte failed, p_hall_data == NULL \n");
		return ;
    }

	if ((p_hall_data->reg.map.opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
		//8 bit
		if (x < -128) {
			x = -128;
		} else if(x > 127) {
			x = 127;
		}

		if (x >= 0) {
			*lbyte = x & 0x7F;
		} else {
			*lbyte = ( (0x80 - (x*(-1))) & 0x7F ) | 0x80;
		}
		*hbyte = 0x00;
	} else {
		//10 bit 
		if (x < -512) {
			x = -512;
		} else if (x > 511) {
			x = 511;
		}

		if (x >=0 ) {
			*lbyte = x & 0xFF;
			*hbyte = (((x & 0x100) >> 8) & 0x01) << 6;
		} else {
			*lbyte = (0x0200 - (x*(-1))) & 0xFF;
			*hbyte = ((((0x0200 - (x*(-1))) & 0x100) >> 8) << 6) | 0x80;
		}
	}
}

static short m1120_up_2byte_to_short(dhall_data_t* dhall_data, u8 hbyte, u8 lbyte)
{
	short x = 0;
    dhall_data_t* p_hall_data = dhall_data;

    if (p_hall_data == NULL) {
		MOTOR_ERR("m1120_up_2byte_to_short failed, p_hall_data == NULL \n");
		return -EINVAL;
    }

	if( (p_hall_data->reg.map.opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
		//8 bit 
		x = lbyte & 0x7F;
		if (lbyte & 0x80) {
			x -= 0x80;
		}
	} else {
		//10 bit
		x = (((hbyte & 0x40) >> 6) << 8 ) | lbyte;
		if (hbyte & 0x80) {
			x -= 0x200;
		}
	}

	return x;
}

 /*********************************************************************
                        internal interrupt function
 **********************************************************************/
static irqreturn_t m1120_up_irq_handler(int irq, void *dev_id)
{
	MOTOR_LOG("call \n");

	if (g_hall_data == NULL) {
		MOTOR_ERR("m1120_up_irq_handler failed, g_hall_data == NULL \n");
		return -EINVAL;
	}

	disable_irq_nosync(g_hall_data->irq);
	oneplus_dhall_irq_handler(HALL_UP);

	return IRQ_HANDLED;
}

static int m1120_up_clear_interrupt(dhall_data_t* dhall_data)
{
    int err = -1;
    u8 data = 0x00;
    dhall_data_t* p_hall_data = dhall_data;

    if (p_hall_data == NULL) {
		MOTOR_ERR("p_hall_data == NULL \n");
		return -EINVAL;
	}

    data = p_hall_data->reg.map.persint | 0x01;
    err =  m1120_up_i2c_write_block(p_hall_data, M1120_REG_PERSINT, &data,1);

    return err;
}

 /*********************************************************************
                        internal power control function
 **********************************************************************/
static int m1120_up_power_ctl(dhall_data_t* dhall_data, bool on)
{
    int err = -1;
    dhall_data_t* p_hall_data = dhall_data;

    if (p_hall_data == NULL ) {
        MOTOR_ERR("m1120_up_set_enable failed, p_hall_data == NULL \n");
        return -EINVAL;
    }

     //we don't need to set vio power since S4A will always-on
    if (!on && p_hall_data->power_enabled) {
        err = regulator_disable(p_hall_data->vdd);
        if (err) {
            MOTOR_ERR("regulator vdd disable failed err=%d \n", err);
            return err;
        }

        p_hall_data->power_enabled = on;
    } else if (on && !p_hall_data->power_enabled) {
        err = regulator_enable(p_hall_data->vdd);
        if (err) {
            MOTOR_ERR("regulator vdd enable failed err=%d\n", err);
            return err;
        }
        msleep(10); // wait 10ms
        p_hall_data->power_enabled = on;
    } else {
        MOTOR_LOG("power on=%d. enabled=%d\n", on, p_hall_data->power_enabled);
    }

    return err;
}

// static int m1120_up_set_power(struct device *dev, bool on)
// {
//     m1120_up_power_ctl(g_hall_data, on);

//     return 0;
// }


static int m1120_up_set_operation_mode(dhall_data_t* dhall_data, int mode)
{
    dhall_data_t* p_hall_data = dhall_data;
    u8 opf = p_hall_data->reg.map.opf;
    int err = -1;

    if (p_hall_data == NULL ) {
        MOTOR_ERR("m1120_up_set_enable failed, p_hall_data == NULL \n");
        return -EINVAL;
    }

    switch (mode) {
    case OPERATION_MODE_POWERDOWN:
        opf &= (0xFF - M1120_VAL_OPF_HSSON_ON);
        err = m1120_up_i2c_write_block(p_hall_data, M1120_REG_OPF, &opf, 1);
        MOTOR_LOG("operation mode chnage to OPERATION_MODE_POWERDOWN");
        break;

    case OPERATION_MODE_MEASUREMENT:
        opf &= (0xFF - M1120_VAL_OPF_EFRD_ON);
        opf |= M1120_VAL_OPF_HSSON_ON;
        err = m1120_up_i2c_write_block(p_hall_data, M1120_REG_OPF, &opf, 1);
        MOTOR_LOG("operation mode chnage to OPERATION_MODE_MEASUREMENT");
        break;

    case OPERATION_MODE_FUSEROMACCESS:
        opf |= M1120_VAL_OPF_EFRD_ON;
        opf |= M1120_VAL_OPF_HSSON_ON;
        err = m1120_up_i2c_write_block(p_hall_data, M1120_REG_OPF, &opf, 1);
        MOTOR_LOG("operation mode chnage to OPERATION_MODE_FUSEROMACCESS");
        break;

    default :
        MOTOR_ERR("unknow operation mode, mode : %d", mode);
    }

    MOTOR_LOG("opf = ox%x \n", opf);

    return err;
}

static bool m1120_up_get_enable(dhall_data_t* dhall_data)
{
    dhall_data_t* p_hall_data = dhall_data;
    int enable = -1;

    if (p_hall_data == NULL ) {
        MOTOR_ERR("m1120_up_set_enable failed, p_hall_data == NULL \n");
        return false;
    }
    mutex_lock(&m1120_up_device_enable_mutex);

    enable = atomic_read(&p_hall_data->device_enable);

    mutex_unlock(&m1120_up_device_enable_mutex);

    return enable > 0 ? true : false;
}

//需要看看這個怎麼搞
static void m1120_up_set_enable(dhall_data_t* dhall_data, bool enable)
{
    dhall_data_t* p_hall_data = dhall_data;
    int p_enable = enable ? 1 : 0;

    if (p_hall_data == NULL || p_hall_data->client == NULL) {
        MOTOR_ERR("m1120_up_set_enable failed, p_hall_data == NULL || p_hall_data->client == NULL \n");
        return;
    }
    MOTOR_LOG("1 \n ");
    mutex_lock(&m1120_up_device_enable_mutex);// add mutex for what ??
    MOTOR_LOG("m1120_up enable : %d\n", p_enable);
    if (enable) {                  
        //enable if state will be changed
        if (!atomic_cmpxchg(&p_hall_data->device_enable, 0, 1)) {
            m1120_up_set_operation_mode(p_hall_data, OPERATION_MODE_MEASUREMENT);
        }
    } else {                        
        //disable if state will be changed
        if (atomic_cmpxchg(&p_hall_data->device_enable, 1, 0)) {
            m1120_up_set_operation_mode(p_hall_data, OPERATION_MODE_POWERDOWN);
        }
    }
    atomic_set(&p_hall_data->device_enable, p_enable);
    MOTOR_LOG("end \n ");
    mutex_unlock(&m1120_up_device_enable_mutex);
}

 /*********************************************************************
                interface implement for onplus_motor.c
 **********************************************************************/
static bool m1120_up_get_enable_state(void)
{
    if (g_hall_data == NULL) {
		MOTOR_LOG("m1120_up_get_enable_state failed, g_hall_data == NULL \n");
		return false;
	}

    return m1120_up_get_enable(g_hall_data) ;
}

static int m1120_up_set_enable_state(bool enable)
{
    if (g_hall_data == NULL) {
		MOTOR_LOG("m1120_up_set_enable_state failed, g_hall_data == NULL \n");
		return false;
	}

    MOTOR_LOG("set m1120_up enable : %d", enable ? 1 : 0);
    m1120_up_set_enable(g_hall_data, enable);

    return 0;
}

static bool m1120_up_is_power_on(void)
{
    if (g_hall_data == NULL) {
		MOTOR_LOG("get m1120_up_is_power_on state failed, g_hall_data == NULL \n");
		return false;
	}

    return g_hall_data->power_enabled > 0 ? true : false;
}
static int m1120_up_get_real_data(short* data)
{ 
    int err = 0;
	u8 buf[3] = {0};
	short value = 0;

    if(!g_hall_data) {
        MOTOR_ERR("g_hall_data == NULL");
        return -1;
    }

	//read data
	err = m1120_up_i2c_read_block(g_hall_data, M1120_REG_ST1, buf, sizeof(buf));
	if (err < 0) {
		MOTOR_ERR("m1120_up get data fail, err : %d \n",err);
		return err;
	}

	//collect data
	if (buf[0] & 0x01) {
		value = m1120_up_2byte_to_short(g_hall_data, buf[2], buf[1]);
	} else {
		MOTOR_ERR("m1120: st1(0x%02X) is not DRDY.\n", buf[0]);
		return -1;
	}

	*data = value;

    MOTOR_LOG("value : %d", value);
	return 0;
}

static int m1120_up_get_abs_data(short* data)
{ 
    short value = 0;
    int err = -1;

    err = m1120_up_get_real_data(&value);
    if (err < 0) {
        MOTOR_ERR("m1120_up get abs data filed, err : %d \n", err);
        return err;
    }

    value = abs(value);
	*data = value;

    MOTOR_LOG("value : %d", value);

	return 0;
}

static int m1120_up_set_detection_mode(u8 mode)
{
	u8 data = 0;
	int err = 0;

    if(g_hall_data == NULL) {
        MOTOR_ERR("g_hall_data == NULL");
        return -EINVAL;
    }

	MOTOR_LOG("m1120 detection mode : %s, irq_enabled : %d\n", (mode == 0)? "POLLING":"INTERRUPT", g_hall_data->irq_enabled);
	if(mode & DETECTION_MODE_INTERRUPT) {
		if (!g_hall_data->irq_enabled) {
			data = g_hall_data->reg.map.intsrs | M1120_DETECTION_MODE_INTERRUPT;
			err = m1120_up_i2c_write_block(g_hall_data, M1120_REG_INTSRS, &data, 1);
			if (err < 0) {
				MOTOR_ERR("config interupt fail, err : %d \n",err);
				return err;
			}
			err = m1120_up_clear_interrupt(g_hall_data);
			if (err < 0) {
				MOTOR_ERR("clear interupt fail, err : %d \n",err);
				return err;
			}

			//requst irq 
            if (request_irq(g_hall_data->irq, &m1120_up_irq_handler, IRQ_TYPE_LEVEL_LOW, "m1120_up", (void *)g_hall_data->client)) {
				MOTOR_ERR("IRQ LINE NOT AVAILABLE!! \n");
				return -EINVAL;
			}
			irq_set_irq_wake(g_hall_data->irq, 1);

			g_hall_data->irq_enabled = 1;
		}
	} else {
		if (g_hall_data->irq_enabled) {
			data = g_hall_data->reg.map.intsrs & (0xFF - M1120_DETECTION_MODE_INTERRUPT);

			err = m1120_up_i2c_write_block(g_hall_data, M1120_REG_INTSRS, &data, 1);
			if (err < 0) {
				MOTOR_ERR("config interupt fail, err : %d \n",err);
				return err;
			}

			disable_irq(g_hall_data->irq);
			free_irq(g_hall_data->irq, NULL);

			g_hall_data->irq_enabled = 0;
		}
	}

    return 0;    
}
static int m1120_up_enable_irq(bool enable)
{
    if(g_hall_data == NULL) {
        MOTOR_ERR("g_hall_data == NULL");
        return -EINVAL;
    }

	if (enable) {
		enable_irq(g_hall_data->irq);
	} else {
		disable_irq_nosync(g_hall_data->irq);
	}

    return 0;
}

static int m1120_up_clear_irq(void)
{
    if(g_hall_data == NULL) {
        MOTOR_ERR("g_hall_data == NULL");
        return -EINVAL;
    }

	m1120_up_clear_interrupt(g_hall_data);
    return 0;
}

static int m1120_up_get_irq_state(void)
{
    if(g_hall_data == NULL) {
        MOTOR_ERR("g_hall_data == NULL");
        return -EINVAL;
    }

    return ((g_hall_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) ? 1 : 0);   
}

static bool m1120_up_update_threshold(int position, short lowthd, short highthd)
{
    
    u8 lthh, lthl, hthh, hthl;
	int err = 0;

	if (g_hall_data == NULL) {
		MOTOR_LOG("g_hall_data == NULL \n");
		return -EINVAL;
	}
	MOTOR_LOG("m1120_up ,low: %d, high :%d, intsrs : %d \n",lowthd, highthd, g_hall_data->reg.map.intsrs);

	err = m1120_up_clear_interrupt(g_hall_data);

	if (g_hall_data->reg.map.intsrs & M1120_VAL_INTSRS_INTTYPE_WITHIN) {

		m1120_up_short_to_2byte(g_hall_data, highthd, &hthh, &hthl);
		m1120_up_short_to_2byte(g_hall_data, lowthd, &lthh, &lthl);
		err |= m1120_up_i2c_write_block(g_hall_data, M1120_REG_HTHH,&hthh, 1);
		err |= m1120_up_i2c_write_block(g_hall_data, M1120_REG_HTHL,&hthl, 1);
		err |= m1120_up_i2c_write_block(g_hall_data, M1120_REG_LTHH,&lthh, 1);
		err |= m1120_up_i2c_write_block(g_hall_data, M1120_REG_LTHL,&lthl, 1);
	}

	if (err < 0) {
		MOTOR_ERR("update threshold fail, err : %d\n",err);
		return false;
	}

    return true;
}

static void m1120_up_dump_reg(u8* buf)
{
    int i, err;
	u8 val;
	u8 buffer[512] = {0};
	u8 _buf[20] = {0};

	if (g_hall_data == NULL) {
		MOTOR_LOG("g_hall_data == NULL \n");
		return ;
	}

	for (i = 0; i <= 0x12; i++) {
		memset(_buf, 0, sizeof(_buf));

		err = m1120_up_i2c_read_block(g_hall_data, i, &val, 1);
		if (err < 0) {
			sprintf(buf, "read reg error!\n");
			return;
		}

		sprintf(_buf,  "reg 0x%x:0x%x\n", i, val);
		strcat(buffer, _buf);
	}
	sprintf(buf, "%s\n", buffer);
	MOTOR_LOG("%s \n",buf);

    return ;  
}

static int m1120_up_set_reg(int reg, int val)
{
    u8 data = (u8)val;

    MOTOR_LOG("reg : %d, val : %d", reg, val);
    if(g_hall_data == NULL) {
        MOTOR_ERR("g_hall_data == NULL");
        return -EINVAL;
    }

	m1120_up_i2c_write_block(g_hall_data, (u8)reg, &data,1);

    return 0;  
}

static void m1120_up_set_sensitive(hall_sensitive_t sensitive)
{
    int err = 0;
    u8  data = 0x00;

    MOTOR_LOG("sensitive %d \n", sensitive);

    switch (sensitive) {
    case HALL_10BIT_0_068mT:
        data = M1120_DETECTION_MODE | M1120_VAL_INTSRS_SRS_10BIT_0_068mT;
        break;
    case HALL_10BIT_0_034mT:
        data = M1120_DETECTION_MODE | M1120_VAL_INTSRS_SRS_10BIT_0_034mT;
        break;
    default:
        data = M1120_DETECTION_MODE | M1120_VAL_INTSRS_SRS_10BIT_0_034mT;
    }

    if (data & M1120_DETECTION_MODE_INTERRUPT) {
        data |= M1120_INTERRUPT_TYPE;
    }

    err = m1120_up_i2c_write_block(g_hall_data, M1120_REG_INTSRS, &data, 1);
    if (err < 0) {
        MOTOR_ERR("m1120_up_i2c_write_block error, data : %d \n", data);
    }

    return;
}

struct oneplus_hall_operations m1120_up_ops = {
    .name = "m1120_up",
    .is_power_on = m1120_up_is_power_on,
    .set_hall_enable_state = m1120_up_set_enable_state,
    .get_hall_enable_state = m1120_up_get_enable_state,
    .get_data_real = m1120_up_get_real_data,
	.get_data_abs  = m1120_up_get_abs_data,
    .set_detection_mode = m1120_up_set_detection_mode,
	.enable_irq = m1120_up_enable_irq,
	.clear_irq = m1120_up_clear_irq,
	.get_irq_state = m1120_up_get_irq_state,
	.update_threshold = m1120_up_update_threshold,
	.dump_regs = m1120_up_dump_reg,
	.set_reg = m1120_up_set_reg,
    .set_sensitive = m1120_up_set_sensitive
};

 /*********************************************************************
                        init function
 **********************************************************************/
static int m1120_up_init_device(dhall_data_t* hall_data)
{
    int err = -1;
    dhall_data_t* p_hall_data = hall_data;

    if (p_hall_data == NULL || p_hall_data->client == NULL) {
        MOTOR_ERR("m1120_up_init_device failed, p_hall_data == NULL || p_hall_data->client == NULL");
    }

    //err = m1120_up_set_power(p_hall_data, true);
    //mutex_lock(&m1120_up_device_enable_mutex);

    //parameter init 
    atomic_set(&p_hall_data->device_enable, 0);
    p_hall_data->irq_enabled = 0;

    //reset registers
    err = m1120_up_reset_device(p_hall_data);
    if (err < 0) {
        MOTOR_ERR("m1120_up_init_device failed, err : %d \n", err);
        return err;
    }

    //mutex_unlock(&m1120_up_device_enable_mutex);

    MOTOR_LOG("initializing device was success! \n");
    
    return 0;
}

static int m1120_up_reset_device(dhall_data_t* dhall_data)
{
    int err = 0;
    u8  id = 0xFF, data = 0x00;
    dhall_data_t* p_hall_data = dhall_data;

    if (p_hall_data == NULL || p_hall_data->client == NULL) {
        MOTOR_ERR("m1120_up_reset_device failed, p_hall_data == NULL || p_hall_data->client == NULL");
        return -EINVAL;
    }

    // sw reset
    data = M1120_VAL_SRST_RESET;
    err = m1120_up_i2c_write_block(p_hall_data, M1120_REG_SRST, &data,1);
    if (err < 0) {
        MOTOR_ERR( "sw-reset failed, err : %d \n", err);
        return err;
    }
    msleep(5);
    MOTOR_LOG("wait 5ms after vdd power up \n");

    // check device id
    err = m1120_up_i2c_read_block(p_hall_data, M1120_REG_DID, &id, 1);
    MOTOR_LOG("1 \n");
    if (err < 0)
        return err;
    if (id != M1120_VAL_DID) {
        MOTOR_ERR("current device id(0x%02X) is not M1120 device id(0x%02X) \n", id, M1120_VAL_DID);
        return -ENXIO;
    }

    //init persint
    MOTOR_LOG("2 \n ");
    data = M1120_PERSISTENCE_COUNT;
    err = m1120_up_i2c_write_block(p_hall_data, M1120_REG_PERSINT, &data,1);
    if (err <0) {
        MOTOR_ERR("cm1120_up_i2c_write_block error, data : %d \n", data);
        return err;
    }
    //init intsrs
    MOTOR_LOG("3 \n ");
    data = M1120_DETECTION_MODE | M1120_SENSITIVITY_TYPE;
    if (data & M1120_DETECTION_MODE_INTERRUPT) {
        data |= M1120_INTERRUPT_TYPE;
    }
    err = m1120_up_i2c_write_block(p_hall_data, M1120_REG_INTSRS, &data, 1);
    if (err < 0) {
        MOTOR_ERR("cm1120_up_i2c_write_block error, data : %d \n", data);
        return err;
    }
    // init  operation mode
    MOTOR_LOG("4 \n ");
    data = M1120_OPERATION_FREQUENCY | M1120_OPERATION_RESOLUTION;
    err = m1120_up_i2c_write_block(p_hall_data, M1120_REG_OPF, &data, 1);
    if (err < 0) {
        MOTOR_ERR("cm1120_up_i2c_write_block error, data : %d \n", data);
        return err;
    }

    //set power on mode
    //这边若不行的话,重新搞的函数
    m1120_up_set_enable(p_hall_data, true);//TO DO : set disable when boot finish???
    MOTOR_LOG("5 \n ");
    return err;
}

static int m1120_up_power_init(dhall_data_t* dhall_data)
{
    int err = -1;
    dhall_data_t* p_hall_data = dhall_data;

    if (p_hall_data == NULL || p_hall_data->client == NULL) {
        MOTOR_ERR("m1120_up_power_init failed, p_hall_data == NULL || p_hall_data->client == NULL");
        return -EINVAL;
    }

    //we don't need to set vio power since S4A will always-on
    p_hall_data->vdd = regulator_get(&p_hall_data->client->dev, "vdd");
    if (IS_ERR(p_hall_data->vdd)) {
        err = PTR_ERR(p_hall_data->vdd);
        MOTOR_ERR("regulator get vdd failed, err : %d \n", err);
        return err;
    }

    if (regulator_count_voltages(p_hall_data->vdd) > 0) {
        err = regulator_set_voltage(p_hall_data->vdd, M1120_VDD_MIN_UV, M1120_VDD_MAX_UV);
        if (err) {
            MOTOR_ERR("regulator set vdd failed, err : %d \n", err);
            goto reg_vdd_put;
        }
    }

    return 0;

reg_vdd_put:
    regulator_put(p_hall_data->vdd);
    return err;
}

static int m1120_up_parse_dt(dhall_data_t* dhall_data)
{
    struct device_node *np = NULL;
    u32 temp_val;
    int err;
    dhall_data_t* p_hall_data = dhall_data;

    MOTOR_LOG("call");

    if (p_hall_data == NULL || p_hall_data->client == NULL) {
             MOTOR_ERR("m1120_up_parse_dt failed, p_hall_data == NULL || p_hall_data->client == NULL");
            // MOTOR_ERR("m1120_up_parse_dt failed, p_hall_data == NULL || p_hall_data->client == NULL || "
            //           "p_hall_data->client->dev == NULL || p_hall_data->client->dev->of_node == NULL");

            return -EINVAL;
    }

    np = p_hall_data->client->dev.of_node;

    //gpio irq request 
    p_hall_data->irq_gpio =  of_get_named_gpio(np, "dhall,irq-gpio", 0);
    MOTOR_LOG("irq_gpio : %d", p_hall_data->irq_gpio);
    if (gpio_is_valid(p_hall_data->irq_gpio)) {
        err = gpio_request(p_hall_data->irq_gpio, "m1120_up_irq");
        if (err) {
            MOTOR_ERR("unable to request gpio : %d", p_hall_data->irq_gpio);
        } else {
            err = gpio_direction_input(dhall_data->irq_gpio);
            msleep(50);
            p_hall_data->irq = gpio_to_irq(dhall_data->irq_gpio);
            MOTOR_LOG("gpio_to_irq success, irq : %d", p_hall_data->irq);
        }
    }

    return 0;
}
static int m1120_up_i2c_drv_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    dhall_data_t* p_hall_data;
    int           err = 0;

    MOTOR_LOG("call");

    //allocation memory for g_hall_data
    p_hall_data = kzalloc(sizeof(dhall_data_t), GFP_KERNEL);
    if (!p_hall_data) {
        MOTOR_ERR("kernel memory alocation was failed \n");
        err = -ENOMEM;
        goto error_0;
    }

    //config i2c client
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        MOTOR_ERR("i2c_check_functionality was failed \n");
        err = -ENODEV;
        goto error_1;
    }
    i2c_set_clientdata(client, p_hall_data);
    p_hall_data->client = client;
    g_hall_data = p_hall_data;//save in global val 

    if (client->dev.of_node) {
        MOTOR_LOG("use client->dev.of_node \n");
        err = m1120_up_parse_dt(p_hall_data);
        if (err) {
            MOTOR_LOG("failed to parse device tree, go to error_1 \n");
            err = -EINVAL;
            goto error_1;
        }
    } else {
        MOTOR_ERR("client->dev.of_node == NULL \n");
         goto error_0;
    }

    //power init and set power on for device
    p_hall_data->power_enabled = false;
    err = m1120_up_power_init(p_hall_data);
    if (err) {
        MOTOR_ERR("failed to get sensor regulators \n");
        err = -EINVAL;
        goto error_1;
    }
    err = m1120_up_power_ctl(p_hall_data, true);
    if (err) {
        MOTOR_ERR("failed to enable sensor power \n");
        err = -EINVAL;
        goto error_1;
    }

    //reset and init device
    err = m1120_up_init_device(p_hall_data);
    if (err) {
        MOTOR_ERR("m1120_up_init_device failed,  err : %d \n", err);
        goto error_1;
    }
    MOTOR_LOG( "%s found", id->name);
    MOTOR_LOG("%s initialized", M1120_DRIVER_NAME_UP);

    //register ops to abstract level
    oneplus_register_dhall("m1120_up",&m1120_up_ops);

    MOTOR_LOG("i2c addr : %d \n", client->addr);
    MOTOR_LOG("%s was probed \n", M1120_DRIVER_NAME_UP);

    return 0;

error_1:
    kfree(p_hall_data);

error_0:
       g_hall_data = NULL;
    return err;
}

static int m1120_up_i2c_drv_remove(struct i2c_client *client)
{
    dhall_data_t* p_hall_data = i2c_get_clientdata(client);

    if (p_hall_data == NULL) {
        MOTOR_ERR("m1120_up_i2c_drv_remove failed, p_hall_data == NULL \n");
        return -EINVAL;
    }

    m1120_up_set_enable(p_hall_data, false);
    kfree(p_hall_data);

    return 0;
}

static const struct i2c_device_id m1120_up_i2c_drv_id_table[] = {
    {"m1120_up", 0 },
    { }
};

static const struct of_device_id m1120_up_of_match[] = {
    { .compatible = "magnachip,mxm1120,up", },
    { },
};

static struct i2c_driver m1120_up_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = M1120_DRIVER_NAME_UP,
        .of_match_table = m1120_up_of_match,
    },
    .probe    = m1120_up_i2c_drv_probe,
    .remove  = m1120_up_i2c_drv_remove,
    .id_table   = m1120_up_i2c_drv_id_table,
    //.suspend  = m1120_up_i2c_drv_suspend,
    //.resume      = m1120_up_i2c_drv_resume,
};

static int __init m1120_up_driver_init_up(void)
{
    int err = 0;
    
    err = i2c_add_driver(&m1120_up_driver);
    MOTOR_LOG("err : %d \n", err);
    return err;
}
module_init(m1120_up_driver_init_up);

static void __exit m1120_up_driver_exit_up(void)
{
    MOTOR_LOG("call \n");
    i2c_del_driver(&m1120_up_driver);
}
module_exit(m1120_up_driver_exit_up);

MODULE_DESCRIPTION("M1120 hallswitch driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("quentin.lin@oneplus.com");

