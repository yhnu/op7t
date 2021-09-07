#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/time.h>
#include "op_dash_adapter.h"
/* add for dash adapter update */
#include <linux/power/oem_external_fg.h>

static struct op_adapter_chip *the_chip;

#define DEFALUT_TX_VALUE 0xFF
static void dash_uart_gpio_set_value(struct op_adapter_chip *chip,
	unsigned long pin, bool value)
{
	if (chip->tx_invalid_val != value) {
		gpio_set_value(pin, value);
		chip->tx_invalid_val = value;
	}
}

#define update_cycle 998
#define  LOG_COUNT 30
int rx_time[LOG_COUNT];
int tx_time[LOG_COUNT];
void print_oem(void)
{
	int i;

	for (i = 0; i < LOG_COUNT - 1; i++)
		pr_info("rx=%d, tx=%d\n", rx_time[i], tx_time[i]);
}

static int dash_uart_gpio_get_value(unsigned long pin)
{
	return gpio_get_value(pin);
}

void oem_delay(struct op_adapter_chip *chip, cycles_t begin_time)
{
	cycles_t time, curl_time;

	if (chip->timer_delay == 52) {
		do {
			curl_time = get_cycles();
			time = curl_time - begin_time;
		} while (time < update_cycle);
	} else if (chip->timer_delay == 78) {
		do {
			curl_time = get_cycles();
			time = curl_time - begin_time;
		} while (time < update_cycle * 3 / 2);
	} else if (chip->timer_delay == 25) {
		do {
			curl_time = get_cycles();
			time = curl_time - begin_time;
		} while (time < update_cycle / 2);
	} else if (chip->timer_delay == 2) {
		do {
			curl_time = get_cycles();
			time = curl_time - begin_time;
		} while (time < 30);
	} else {
		udelay(chip->timer_delay);
	}
}

static void dash_uart_tx_bit(
	struct op_adapter_chip *chip, unsigned char tx_data)
{
	static unsigned char tx_bit = BIT_START;

	switch (tx_bit) {
	case BIT_START:
		chip->tx_byte_over = false;
		dash_uart_gpio_set_value(
			chip, chip->uart_tx_gpio, 0);
		tx_bit = BIT_0;
		break;
	case BIT_0:
	case BIT_1:
	case BIT_2:
	case BIT_3:
	case BIT_4:
	case BIT_5:
	case BIT_6:
	case BIT_7:
		if (tx_data & (1 << tx_bit))
			dash_uart_gpio_set_value(
			chip, chip->uart_tx_gpio, 1);
		else
			dash_uart_gpio_set_value(
			chip, chip->uart_tx_gpio, 0);
			tx_bit++;
		break;
	case BIT_STOP:
	case BIT_IDLE:
		dash_uart_gpio_set_value(chip, chip->uart_tx_gpio, 1);
		tx_bit = BIT_START;
		chip->tx_byte_over = true;
		break;
	default:
		break;
	}
}

static int dash_uart_rx_bit(struct op_adapter_chip *chip)
{
	static unsigned char rx_bit = BIT_IDLE, rx_val;

	switch (rx_bit) {
	case BIT_IDLE:
		chip->rx_byte_over = false;
		if (!dash_uart_gpio_get_value(chip->uart_rx_gpio)) {
			rx_bit = BIT_0;
			chip->timer_delay = 78; /* 1.5 cycle */
		} else {
			chip->timer_delay = 2;	/* 0.02 cycle */
		}
		break;
	case BIT_0:
	case BIT_1:
	case BIT_2:
	case BIT_3:
	case BIT_4:
	case BIT_5:
	case BIT_6:
	case BIT_7:
		chip->timer_delay = 52; /* 1 cycle */
		if (dash_uart_gpio_get_value(chip->uart_rx_gpio))
			rx_val |= (1 << rx_bit);
		else
			rx_val &= ~(1 << rx_bit);
		rx_bit++;
		break;
	case BIT_STOP:
		rx_bit = BIT_IDLE;
		chip->rx_byte_over = true;
		break;
	default:
		break;
	}

	return rx_val;
}

static void dash_uart_tx_byte(
	struct op_adapter_chip *chip, unsigned char tx_data)
{
	cycles_t time, curl_time, begin_time;
	int i = 0;

	chip->timer_delay = 52;
	while (1) {
		begin_time = get_cycles();
		mb();/*need const tx*/
		dash_uart_tx_bit(chip, tx_data);
		curl_time = get_cycles();
		time = curl_time - begin_time;
		tx_time[i++] = time;

		if (i > LOG_COUNT - 1)
			i = 0;

		oem_delay(chip, begin_time);
		if (chip->tx_byte_over) {
			chip->timer_delay = 25;
			break;
		}
	}
}

static unsigned char dash_uart_rx_byte(
	struct op_adapter_chip *chip, unsigned int cmd)
{
	unsigned char rx_val = 0;
	unsigned int count = 0;
	unsigned int max_count = 0;
	cycles_t time, curl_time, begin_time;
	int i = 0;

	if (cmd == Read_Addr_Line_Cmd)
		max_count = Read_Addr_Line_Cmd_Count;
	else if (cmd == Write_Addr_Line_Cmd)
		max_count = Write_Addr_Line_Cmd_Count;
	else if (cmd == Erase_Addr_Line_Cmd)
		max_count = Erase_Addr_Line_Cmd_Count;
	else if (cmd == Read_All_Cmd)
		max_count = Read_All_Cmd_Count;
	else if (cmd == Erase_All_Cmd)
		max_count = Erase_All_Cmd_Count;
	else if (cmd == Boot_Over_Cmd)
		max_count = Boot_Over_Cmd_Count;
	else
		max_count = Other_Cmd_count;

	chip->rx_timeout = false;
	chip->timer_delay = 25;
	while (1) {
		begin_time = get_cycles();
		mb();/*need a const rx*/
		rx_val = dash_uart_rx_bit(chip);
		curl_time = get_cycles();
		time = curl_time - begin_time;
		rx_time[i++] = time;

		if (i > LOG_COUNT - 1)
			i = 0;

		oem_delay(chip, begin_time);
		if (chip->rx_byte_over)
			return rx_val;
		if (count > max_count) {
			chip->rx_timeout = true;
			return 0x00;
		}
		count++;
	}
}

static void dash_uart_irq_fiq_enable(bool enable)
{
	if (enable) {
		preempt_enable();
		local_fiq_enable();
		local_irq_enable();
	} else {
		local_irq_disable();
		local_fiq_disable();
		preempt_disable();
	}
}

static int dash_uart_write_some_addr(
	struct op_adapter_chip *chip, u8 *fw_buf, int length)
{
	unsigned int write_addr = 0, i = 0, fw_count = 0;
	unsigned char rx_val = 0;

	while (1) {
		/* tx: 0xF5 */
		dash_uart_irq_fiq_enable(false);
		dash_uart_tx_byte(chip, (Write_Addr_Line_Cmd >> 8) & 0xff);
		/* tx: 0x02 */
		dash_uart_tx_byte(chip, Write_Addr_Line_Cmd & 0xff);
		/* count:16 bytes */
		dash_uart_tx_byte(chip, 16);

		/* addr: 2 byte */
		if (write_addr == 0)
			write_addr =
			(fw_buf[fw_count + 1] << 8)
			| fw_buf[fw_count];
		dash_uart_tx_byte(chip, (write_addr >> 8) & 0xff);
		dash_uart_tx_byte(chip, write_addr & 0xff);

		if (!(write_addr % 0x20))
			fw_count += 2;

		/* data: 16 bytes */
		for (i = 0; i < 16; i++) {
			dash_uart_tx_byte(chip, fw_buf[fw_count]);
			fw_count++;

			if (i == 15)
				rx_val = dash_uart_rx_byte(
					chip, Write_Addr_Line_Cmd);
		}

		dash_uart_irq_fiq_enable(true);
		write_addr += 16;
		if (rx_val != UART_ACK || chip->rx_timeout) {
			pr_err("%s err,write_addr:0x%x,chip->rx_timeout:%d,rx_val=%d\n",
				__func__, write_addr,
				chip->rx_timeout, rx_val);
			return -EINVAL;
		}
		if (fw_count >= length)
			return 0;
	}
}

#define STM8S_ADAPTER_FIRST_ADDR	0x8C00
#define STM8S_ADAPTER_LAST_ADDR		0x9FEF
#define HALF_ONE_LINE			16

static bool dash_uart_read_addr_line_and_check(
	struct op_adapter_chip *chip, unsigned int addr)
{
	unsigned char fw_check_buf[20] = {0x00};
	int i = 0;
	static int fw_line;
	bool check_result = false;
	int addr_check_err = 0;

	if (addr == STM8S_ADAPTER_FIRST_ADDR)
		fw_line = 0;

	/* Tx_Read_Addr_Line */
	/* tx:0xF5 */
	dash_uart_irq_fiq_enable(false);
	dash_uart_tx_byte(chip, (Read_Addr_Line_Cmd >> 8) & 0xff);
	/* tx:0x01 */
	dash_uart_tx_byte(chip, Read_Addr_Line_Cmd & 0xff);
	/* tx:0x9F */
	dash_uart_tx_byte(chip, (addr >> 8) & 0xff);
	/* tx:0xF0 */
	dash_uart_tx_byte(chip, addr & 0xff);

	fw_check_buf[0] = dash_uart_rx_byte(chip, Read_Addr_Line_Cmd);
	if (chip->rx_timeout)
		goto read_addr_line_err;

	fw_check_buf[1] = dash_uart_rx_byte(chip, Read_Addr_Line_Cmd);
	if (chip->rx_timeout)
		goto read_addr_line_err;

	if (addr != ((fw_check_buf[0] << 8) | fw_check_buf[1])) {
		addr_check_err = 1;
		goto read_addr_line_err;
	}

	for (i = 0; i < 16; i++) {
		fw_check_buf[i + 2] =
			dash_uart_rx_byte(chip, Read_Addr_Line_Cmd);
		if (chip->rx_timeout)
			goto  read_addr_line_err;
	}

	if (!(addr % 0x20)) {
		if (addr == ((adapter_stm8s_firmware_data[fw_line * 34 + 1]
			<< 8)
			| (adapter_stm8s_firmware_data[fw_line * 34]))) {
			for (i = 0; i < 16; i++) {
				if (fw_check_buf[i + 2]
					!= adapter_stm8s_firmware_data[
					fw_line * 34 + 2 + i])
					goto read_addr_line_err;
			}
		}
	} else {
		if ((addr - 16) ==
			((adapter_stm8s_firmware_data[fw_line * 34 + 1] << 8)
			| (adapter_stm8s_firmware_data[fw_line * 34]))) {
			for (i = 0; i < 16; i++) {
				if (fw_check_buf[i + 2]
					!= adapter_stm8s_firmware_data[
					fw_line * 34
					+ 2 + HALF_ONE_LINE + i])
					goto read_addr_line_err;
			}
		}
		fw_line++;
	}
	check_result = true;

read_addr_line_err:
	dash_uart_irq_fiq_enable(true);
	if (addr_check_err) {
		pr_err("%s addr:0x%x,buf[0]:0x%x,buf[1]:0x%x\n",
			__func__, addr, fw_check_buf[0], fw_check_buf[1]);
	}
	if (!check_result) {
		pr_err("%s fw_check err,addr:0x%x,check_buf[%d]:0x%x != fw_data[%d]:0x%x\n",
			__func__, addr, i + 2, fw_check_buf[i + 2],
			(fw_line * 34 + 2 + i),
			adapter_stm8s_firmware_data[fw_line * 34 + 2 + i]);
		for (i = 0; i < 16; i++)
			pr_err("fw_check_buf[%d]=0x%x\n",
			i+2, fw_check_buf[i + 2]);
	}
	return check_result;
}

static int dash_uart_read_front_addr_and_check(struct op_adapter_chip *chip)
{
	unsigned int read_addr = STM8S_ADAPTER_FIRST_ADDR;
	bool result = false;

	while (read_addr < STM8S_ADAPTER_LAST_ADDR) {
		result = dash_uart_read_addr_line_and_check(chip, read_addr);
		read_addr = read_addr + 16;
		if ((!result) || chip->rx_timeout) {
			pr_err("%s result:%d,chip->rx_timeout:%d\n",
				__func__, result, chip->rx_timeout);
			return -EINVAL;
		}
	}
	return 0;
}

static bool
dash_adapter_update_handle(
	struct op_adapter_chip *chip,
	unsigned long tx_pin, unsigned long rx_pin)
{
	unsigned char rx_val = 0;
	int rx_last_line_count = 0;
	unsigned char rx_last_line[18] = {0x0};
	int rc = 0;

	pr_err("%s v1 begin\n", __func__);
	chip->uart_tx_gpio = tx_pin;
	chip->uart_rx_gpio = rx_pin;
	chip->adapter_update_ing = true;
	chip->tx_invalid_val = DEFALUT_TX_VALUE;
	chip->rx_timeout = false;

	/* step1: Tx_Erase_Addr_Line */
	/* tx:0xF5 */
	dash_uart_irq_fiq_enable(false);
	dash_uart_tx_byte(chip, (Erase_Addr_Line_Cmd >> 8) & 0xff);
	dash_uart_irq_fiq_enable(true);

	/* tx:0x03 */
	dash_uart_irq_fiq_enable(false);
	dash_uart_tx_byte(chip, Erase_Addr_Line_Cmd & 0xff);
	dash_uart_irq_fiq_enable(true);

	/* tx:0x9F */
	dash_uart_irq_fiq_enable(false);
	dash_uart_tx_byte(chip, (Last_Line_Addr >> 8) & 0xff);
	dash_uart_irq_fiq_enable(true);

	/* tx:0xF0 */
	dash_uart_irq_fiq_enable(false);
	dash_uart_tx_byte(chip, Last_Line_Addr & 0xff);
	rx_val = dash_uart_rx_byte(chip, Erase_Addr_Line_Cmd);
	dash_uart_irq_fiq_enable(true);
	if (rx_val != UART_ACK || chip->rx_timeout) {
		pr_err("%s Tx_Erase_Addr_Line err,chip->rx_timeout:%d, rx_val:0x%x\n",
			__func__, chip->rx_timeout, rx_val);
		goto update_err;
	}
	/* Step2: Tx_Read_Addr_Line */
	/* tx:0xF5 */
	dash_uart_irq_fiq_enable(false);
	dash_uart_tx_byte(chip, (Read_Addr_Line_Cmd >> 8) & 0xff);
	dash_uart_irq_fiq_enable(true);

	/* tx:0x01 */
	dash_uart_irq_fiq_enable(false);
	dash_uart_tx_byte(chip, Read_Addr_Line_Cmd & 0xff);
	dash_uart_irq_fiq_enable(true);

	/* tx:0x9F */
	dash_uart_irq_fiq_enable(false);
	dash_uart_tx_byte(chip, (Last_Line_Addr >> 8) & 0xff);
	dash_uart_irq_fiq_enable(true);

	/* tx:0xF0 */
	dash_uart_irq_fiq_enable(false);
	dash_uart_tx_byte(chip, Last_Line_Addr & 0xff);
	for (rx_last_line_count = 0;
		rx_last_line_count < 18; rx_last_line_count++) {
		rx_last_line[rx_last_line_count]
			= dash_uart_rx_byte(chip, Read_Addr_Line_Cmd);
		if (chip->rx_timeout)
			break;
	}

	dash_uart_irq_fiq_enable(true);
	if ((rx_last_line[FW_EXIST_LOW] == 0x55 &&
			rx_last_line[FW_EXIST_HIGH] == 0x34)
			|| chip->rx_timeout) {
		pr_err("%s Tx_Read_Addr_Line err,chip->rx_timeout:%d\n",
			__func__, chip->rx_timeout);
		goto update_err;
	}

	/* Step3: Tx_Erase_All */
	/* tx:0xF5 */
	dash_uart_irq_fiq_enable(false);
	dash_uart_tx_byte(chip, (Erase_All_Cmd >> 8) & 0xff);
	dash_uart_irq_fiq_enable(true);

	/* tx:0x05 */
	dash_uart_irq_fiq_enable(false);
	dash_uart_tx_byte(chip, Erase_All_Cmd & 0xff);
	rx_val = dash_uart_rx_byte(chip, Erase_All_Cmd);
	dash_uart_irq_fiq_enable(true);
	if (rx_val != UART_ACK || chip->rx_timeout) {
		pr_err("%s Tx_Erase_All err,chip->rx_timeout:%d\n",
			__func__, chip->rx_timeout);
		goto update_err;
	}

	/* Step4: Tx_Write_Addr_Line */
	rc = dash_uart_write_some_addr(chip, &adapter_stm8s_firmware_data[0],
		(sizeof(adapter_stm8s_firmware_data) - 34));
	if (rc) {
		pr_err("%s Tx_Write_Addr_Line err\n", __func__);
		goto update_err;
	}

	/* Step5: Tx_Read_All */
	rc = dash_uart_read_front_addr_and_check(chip);
	if (rc) {
		pr_err("%s Tx_Read_All err\n", __func__);
		goto update_err;
	}

	/* Step6: write the last line */
	rc = dash_uart_write_some_addr(chip,
			&adapter_stm8s_firmware_data[
			sizeof(adapter_stm8s_firmware_data) - 34],
			34);
	if (rc) {
		pr_err("%s write the last line err\n", __func__);
		goto update_err;
	}

	/* Step7: Tx_Boot_Over */
	/* tx:0xF5 */
	dash_uart_irq_fiq_enable(false);
	dash_uart_tx_byte(chip, (Boot_Over_Cmd >> 8) & 0xff);
	dash_uart_irq_fiq_enable(true);

	/* tx:0x06 */
	dash_uart_irq_fiq_enable(false);
	dash_uart_tx_byte(chip, Boot_Over_Cmd & 0xff);
	rx_val = dash_uart_rx_byte(chip, Boot_Over_Cmd);
	dash_uart_irq_fiq_enable(true);
	if (rx_val != UART_ACK || chip->rx_timeout) {
		pr_err("%s Tx_Boot_Over err,chip->rx_timeout:%d\n",
			__func__, chip->rx_timeout);
		goto update_err;
	}
	chip->rx_timeout = false;
	chip->adapter_update_ing = false;
	pr_err("%s success\n", __func__);
	return true;

update_err:
	chip->rx_timeout = false;
	chip->adapter_update_ing = false;
	print_oem();
	pr_err("%s err\n", __func__);
	return false;
}

bool dash_adapter_update_is_tx_gpio(unsigned int gpio_num)
{
	if (!the_chip)
		return false;
	if (the_chip->adapter_update_ing && gpio_num == the_chip->uart_tx_gpio)
		return true;
	else
		return false;
}

bool dash_adapter_update_is_rx_gpio(unsigned int gpio_num)
{
	if (!the_chip)
		return false;

	if (the_chip->adapter_update_ing && gpio_num == the_chip->uart_rx_gpio)
		return true;
	else
		return false;
}


struct op_adapter_operations op_adapter_ops = {
		.adapter_update = dash_adapter_update_handle,
};

static int __init dash_adapter_init(void)
{
	struct op_adapter_chip *chip = NULL;

	chip = kzalloc(sizeof(struct op_adapter_chip), GFP_KERNEL);
	if (!chip)
		return -EINVAL;

	chip->timer_delay = 0;
	chip->tx_byte_over = false;
	chip->rx_byte_over = false;
	chip->rx_timeout = false;
	chip->uart_tx_gpio = 0;
	chip->uart_rx_gpio = 0;
	chip->adapter_update_ing = false;
	chip->adapter_firmware_data = adapter_stm8s_firmware_data;
	chip->adapter_fw_data_count = sizeof(adapter_stm8s_firmware_data);
	chip->vops = &op_adapter_ops;
	op_adapter_init(chip);
	the_chip = chip;

	pr_info("%s success\n", __func__);
	return 0;
}

static void __init dash_adapter_exit(void)
{

}

module_init(dash_adapter_init);
module_exit(dash_adapter_exit);
