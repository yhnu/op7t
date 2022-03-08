#include "../touchpanel_common.h"
#include "synaptics_common.h"
#include <linux/crc32.h>

/*******Part0:LOG TAG Declear********************/

#define TPD_DEVICE "synaptics_common"
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)
#define TPD_DEBUG(a, arg...)\
	do{\
		if (LEVEL_DEBUG == tp_debug)\
		pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
	}while(0)

#define TPD_DETAIL(a, arg...)\
	do{\
		if (LEVEL_BASIC != tp_debug)\
		pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
	}while(0)


/*******Part1:Call Back Function implement*******/

unsigned int extract_uint_le(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] +
		(unsigned int)ptr[1] * 0x100 +
		(unsigned int)ptr[2] * 0x10000 +
		(unsigned int)ptr[3] * 0x1000000;
}

/*******************Limit File With "limit_block" Format*************************************/
int synaptics_get_limit_data(char *type, const unsigned char *fw_image)
{
	int i = 0;
	unsigned int offset, count;
	struct limit_info *limit_info;
	struct limit_block *limit_block;

	limit_info = (struct limit_info *)fw_image;
	count = limit_info->count;

	offset = sizeof(struct limit_info);
	for (i = 0; i < count; i++) {
		limit_block = (struct limit_block *)(fw_image + offset);
		pr_info("name: %s, size: %d, offset %d\n", limit_block->name, limit_block->size, offset);
		if (strncmp(limit_block->name, type, MAX_LIMIT_NAME_SIZE) == 0) {
			break;
		}

		offset += (sizeof(struct limit_block) - 4 + 2*limit_block->size); /*minus 4, because byte align*/
	}

	if (i == count) {
		return 0;
	}

	return offset;
}

/*************************************TCM Firmware Parse Funtion**************************************/
int synaptics_parse_header_v2(struct image_info *image_info, const unsigned char *fw_image)
{
	struct image_header_v2 *header;
	unsigned int magic_value;
	unsigned int number_of_areas;
	unsigned int i = 0;
	unsigned int addr;
	unsigned int length;
	unsigned int checksum;
	unsigned int flash_addr;
	const unsigned char *content;
	struct area_descriptor *descriptor;
	int offset = sizeof(struct image_header_v2);

	header = (struct image_header_v2 *)fw_image;
	magic_value = le4_to_uint(header->magic_value);

	if (magic_value != IMAGE_FILE_MAGIC_VALUE) {
		pr_err("invalid magic number %d\n", magic_value);
		return -EINVAL;
	}

	number_of_areas = le4_to_uint(header->num_of_areas);

	for (i = 0; i < number_of_areas; i++) {
		addr = le4_to_uint(fw_image + offset);
		descriptor = (struct area_descriptor *)(fw_image + addr);
		offset += 4;

		magic_value =  le4_to_uint(descriptor->magic_value);
		if (magic_value != FLASH_AREA_MAGIC_VALUE)
			continue;

		length = le4_to_uint(descriptor->length);
		content = (unsigned char *)descriptor + sizeof(*descriptor);
		flash_addr = le4_to_uint(descriptor->flash_addr_words) * 2;
		checksum = le4_to_uint(descriptor->checksum);

		if (0 == strncmp((char *)descriptor->id_string,
					BOOT_CONFIG_ID,
					strlen(BOOT_CONFIG_ID))) {
			if (checksum != (crc32(~0, content, length) ^ ~0)) {
				pr_err("Boot config checksum error\n");
				return -EINVAL;
			}
			image_info->boot_config.size = length;
			image_info->boot_config.data = content;
			image_info->boot_config.flash_addr = flash_addr;
			pr_info("Boot config size = %d, address = 0x%08x\n", length, flash_addr);
		} else if (0 == strncmp((char *)descriptor->id_string,
					APP_CODE_ID,
					strlen(APP_CODE_ID))) {
			if (checksum != (crc32(~0, content, length) ^ ~0)) {
				pr_err("Application firmware checksum error\n");
				return -EINVAL;
			}
			image_info->app_firmware.size = length;
			image_info->app_firmware.data = content;
			image_info->app_firmware.flash_addr = flash_addr;
			pr_info("Application firmware size = %d address = 0x%08x\n", length, flash_addr);
		} else if (0 == strncmp((char *)descriptor->id_string,
					APP_CONFIG_ID,
					strlen(APP_CONFIG_ID))) {
			if (checksum != (crc32(~0, content, length) ^ ~0)) {
				pr_err("Application config checksum error\n");
				return -EINVAL;
			}
			image_info->app_config.size = length;
			image_info->app_config.data = content;
			image_info->app_config.flash_addr = flash_addr;
			pr_info("Application config size = %d address = 0x%08x\n",length, flash_addr);
		} else if (0 == strncmp((char *)descriptor->id_string,
					DISP_CONFIG_ID,
					strlen(DISP_CONFIG_ID))) {
			if (checksum != (crc32(~0, content, length) ^ ~0)) {
				pr_err("Display config checksum error\n");
				return -EINVAL;
			}
			image_info->disp_config.size = length;
			image_info->disp_config.data = content;
			image_info->disp_config.flash_addr = flash_addr;
			pr_info("Display config size = %d address = 0x%08x\n", length, flash_addr);
		}
	}
	return 0;
}

/**********************************RMI Firmware Parse Funtion*****************************************/
void synaptics_parse_header(struct image_header_data *header, const unsigned char *fw_image)
{
	struct image_header *data = (struct image_header *)fw_image;

	header->checksum = extract_uint_le(data->checksum);
	TPD_DEBUG(" checksume is %x", header->checksum);

	header->bootloader_version = data->bootloader_version;
	TPD_DEBUG(" bootloader_version is %d\n", header->bootloader_version);

	header->firmware_size = extract_uint_le(data->firmware_size);
	TPD_DEBUG(" firmware_size is %x\n", header->firmware_size);

	header->config_size = extract_uint_le(data->config_size);
	TPD_DEBUG(" header->config_size is %x\n", header->config_size);

	/* only available in s4322 , reserved in other, begin*/
	header->bootloader_offset = extract_uint_le(data->bootloader_addr );
	header->bootloader_size = extract_uint_le(data->bootloader_size);
	TPD_DEBUG(" header->bootloader_offset is %x\n", header->bootloader_offset);
	TPD_DEBUG(" header->bootloader_size is %x\n", header->bootloader_size);

	header->disp_config_offset = extract_uint_le(data->dsp_cfg_addr);
	header->disp_config_size = extract_uint_le(data->dsp_cfg_size);
	TPD_DEBUG(" header->disp_config_offset is %x\n", header->disp_config_offset);
	TPD_DEBUG(" header->disp_config_size is %x\n", header->disp_config_size);
	/* only available in s4322 , reserved in other ,  end*/

	memcpy(header->product_id, data->product_id, sizeof(data->product_id));
	header->product_id[sizeof(data->product_id)] = 0;

	memcpy(header->product_info, data->product_info, sizeof(data->product_info));

	header->contains_firmware_id = data->options_firmware_id;
	TPD_DEBUG(" header->contains_firmware_id is %x\n", header->contains_firmware_id);
	if (header->contains_firmware_id)
		header->firmware_id = extract_uint_le(data->firmware_id);

	return;
}

void synaptics_print_limit_v2(struct seq_file *s, struct touchpanel_data *ts, const struct firmware *fw)
{
	int i = 0, index = 0;
	int row, col;
	int rows, cols;
	unsigned int offset, count;
	int16_t *data_16;
	struct limit_info *limit_info;
	struct limit_block *limit_block;
	const char *data = fw->data;

	limit_info = (struct limit_info*)data;
	count = limit_info->count;

	offset = sizeof(struct limit_info);
	for (i = 0; i < count; i++) {
		limit_block = (struct limit_block *)(data + offset);
		pr_info("name: %s, size: %d, offset %d\n", limit_block->name, limit_block->size, offset);

		seq_printf(s, "%s\n", limit_block->name);

		data_16 = &limit_block->data;
		offset += (sizeof(struct limit_block) - 4 + 2*limit_block->size); /*minus 4, because byte align*/

		if ((ts->hw_res.TX_NUM*ts->hw_res.RX_NUM) != limit_block->size/2) {
			continue;
		}

		cols = ts->hw_res.TX_NUM;
		rows = ts->hw_res.RX_NUM;

		index = 0;
		for (row = 0; row < rows; row++){
			seq_printf(s, "[%02d]:", row);
			for(col = 0; col < cols; col++) {
				seq_printf(s, "%4d %4d,", *(data_16 + 2*index), *(data_16 + 2*index + 1));
				index++;
			}
			seq_printf(s, "\n");
		}

	}
	return;
}

void synaptics_print_limit_v1(struct seq_file *s, struct touchpanel_data *ts, const struct firmware *fw)
{
	uint16_t *prow = NULL;
	uint16_t *prowcbc = NULL;
	int i = 0;
	struct test_header *ph = NULL;

	ph = (struct test_header *)(fw->data);
	prow = (uint16_t *)(fw->data + ph->array_limit_offset);
	prowcbc = (uint16_t *)(fw->data + ph->array_limitcbc_offset);
	TPD_INFO("synaptics_test_limit_show:array_limit_offset = %x array_limitcbc_offset = %x \n",
			ph->array_limit_offset, ph->array_limitcbc_offset);
	TPD_DEBUG("test begin:\n");
	seq_printf(s, "Without cbc:");

	for (i = 0 ; i < (ph->array_limit_size / 2); i++) {
		if (i % (2 * ts->hw_res.RX_NUM) == 0)
			seq_printf(s, "\n[%2d] ", (i / ts->hw_res.RX_NUM) / 2);
		seq_printf(s, "%4d, ", prow[i]);
		TPD_DEBUG("%d, ", prow[i]);
	}
	if (ph->withCBC == 1) {
		seq_printf(s, "\nWith cbc:");
		for (i = 0 ; i < (ph->array_limitcbc_size / 2); i++) {
			if (i % (2 * ts->hw_res.RX_NUM) == 0)
				seq_printf(s, "\n[%2d] ", (i / ts->hw_res.RX_NUM) / 2);
			seq_printf(s, "%4d, ", prowcbc[i]);
			TPD_DEBUG("%d, ", prowcbc[i]);
		}
	}

	seq_printf(s, "\n");
	return;
}


void synaptics_limit_read(struct seq_file *s, struct touchpanel_data *ts)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	struct test_header *ph = NULL;

	ret = request_firmware(&fw, ts->panel_data.test_limit_name, ts->dev);
	if (ret < 0) {
		TPD_INFO("Request firmware failed - %s (%d)\n", ts->panel_data.test_limit_name, ret);
		seq_printf(s, "Request failed, Check the path %s\n",ts->panel_data.test_limit_name);
		return;
	}

	ph = (struct test_header *)(fw->data);

	if (ph->magic1 == Limit_MagicNum1 && ph->magic2 == Limit_MagicNum2_V2) {
		synaptics_print_limit_v2(s, ts, fw);
	} else {
		synaptics_print_limit_v1(s, ts, fw);
	}
	release_firmware(fw);
}

//proc/touchpanel/baseline_test
static int tp_auto_test_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct synaptics_proc_operations *syna_ops;
	struct timespec now_time;
	struct rtc_time rtc_now_time;
	const struct firmware *fw = NULL;
	mm_segment_t old_fs;
	uint8_t data_buf[64];
	int ret = 0;
	int fd = -1;

	struct syna_testdata syna_testdata =
	{
		.TX_NUM = 0,
		.RX_NUM = 0,
		.fd = -1,
		.irq_gpio = -1,
		.key_TX = 0,
		.key_RX = 0,
		.TP_FW = 0,
		.fw = NULL,
	};

	if (!ts)
		return 0;
	syna_ops = (struct synaptics_proc_operations *)ts->private_data;
	if (!syna_ops)
		return 0;
	if (!syna_ops->auto_test) {
		seq_printf(s, "Step 0-1:Not support auto-test proc node\n");
		return 0;
	}

	/*if resume not completed, do not do screen on test*/
	if (ts->suspend_state != TP_SPEEDUP_RESUME_COMPLETE) {
		seq_printf(s, "Step 0-2:Not in resume state\n");
		return 0;
	}

	//step1:disable_irq && get mutex locked
	if (ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}
	mutex_lock(&ts->mutex);

	//step2: create a file to store test data in /sdcard/Tp_Test
	getnstimeofday(&now_time);
	rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
	sprintf(data_buf, "/sdcard/tp_testlimit_%02d%02d%02d-%02d%02d%02d-utc.csv",
			(rtc_now_time.tm_year + 1900) % 100, rtc_now_time.tm_mon + 1, rtc_now_time.tm_mday,
			rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fd = sys_open(data_buf, O_WRONLY | O_CREAT | O_TRUNC, 0);
	if (fd < 0) {
		TPD_INFO("Open log file '%s' failed.\n", data_buf);
		seq_printf(s, "Step 0-3:Open log file '%s' failed.\n", data_buf);
		set_fs(old_fs);
		mutex_unlock(&ts->mutex);
		if (ts->int_mode == BANNABLE) {
			enable_irq(ts->irq);
		}
		return 0;
	}

	//step3:request test limit data from userspace
	ret = request_firmware(&fw, ts->panel_data.test_limit_name, ts->dev);
	if (ret < 0) {
		TPD_INFO("Request firmware failed - %s (%d)\n", ts->panel_data.test_limit_name, ret);
		seq_printf(s, "Step 0-4:No limit IMG\n");
		sys_close(fd);
		set_fs(old_fs);
		mutex_unlock(&ts->mutex);
		if (ts->int_mode == BANNABLE) {
			enable_irq(ts->irq);
		}
		return 0;
	}

	ts->in_test_process = true;

	//step4:init syna_testdata
	syna_testdata.fd = fd;
	syna_testdata.TX_NUM = ts->hw_res.TX_NUM;
	syna_testdata.RX_NUM = ts->hw_res.RX_NUM;
	syna_testdata.irq_gpio = ts->hw_res.irq_gpio;
	syna_testdata.key_TX = ts->hw_res.key_TX;
	syna_testdata.key_RX = ts->hw_res.key_RX;
	syna_testdata.TP_FW = ts->panel_data.TP_FW;
	syna_testdata.fw = fw;

	syna_ops->auto_test(s, ts->chip_data, &syna_testdata);

	//step5: close file && release test limit firmware
	if (fd >= 0) {
		sys_close(fd);
		set_fs(old_fs);
	}
	release_firmware(fw);

	//step6: return to normal mode
	ts->ts_ops->reset(ts->chip_data);
	operate_mode_switch(ts);

	//step7: unlock the mutex && enable irq trigger
	mutex_unlock(&ts->mutex);
	if (ts->int_mode == BANNABLE) {
		enable_irq(ts->irq);
	}

	ts->in_test_process = false;
	return 0;
}

static int baseline_autotest_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_auto_test_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_auto_test_proc_fops = {
	.owner = THIS_MODULE,
	.open  = baseline_autotest_open,
	.read  = seq_read,
	.release = single_release,
};

static int tp_RT251_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct debug_info_proc_operations *debug_info_ops;

	if (!ts)
		return 0;
	debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

	if (!debug_info_ops)
		return 0;
	if (!debug_info_ops->RT251) {
		seq_printf(s, "Not support RT251 proc node\n");
		return 0;
	}
	disable_irq_nosync(ts->client->irq);
	mutex_lock(&ts->mutex);
	debug_info_ops->RT251(s, ts->chip_data);
	mutex_unlock(&ts->mutex);
	enable_irq(ts->client->irq);

	return 0;
}

static int RT251_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_RT251_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_RT251_proc_fops = {
	.owner = THIS_MODULE,
	.open  = RT251_open,
	.read  = seq_read,
	.release = single_release,
};

static int tp_RT76_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct debug_info_proc_operations *debug_info_ops;

	if (!ts)
		return 0;
	debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

	if (!debug_info_ops)
		return 0;
	if (!debug_info_ops->RT76) {
		seq_printf(s, "Not support RT76 proc node\n");
		return 0;
	}
	disable_irq_nosync(ts->client->irq);
	mutex_lock(&ts->mutex);
	debug_info_ops->RT76(s, ts->chip_data);
	mutex_unlock(&ts->mutex);
	enable_irq(ts->client->irq);

	return 0;
}

static int RT76_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_RT76_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_RT76_proc_fops = {
	.owner = THIS_MODULE,
	.open  = RT76_open,
	.read  = seq_read,
	.release = single_release,
};

static int tp_DRT_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct debug_info_proc_operations *debug_info_ops;

	if (!ts)
		return 0;
	debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

	if (!debug_info_ops)
		return 0;
	if (!debug_info_ops->DRT) {
		seq_printf(s, "Not support RT76 proc node\n");
		return 0;
	}

	if (ts->is_suspended && (ts->gesture_enable != 1)) {
		seq_printf(s, "In suspend state, and gesture not enable\n");
		return 0;
	}
	if (ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}

	mutex_lock(&ts->mutex);
	debug_info_ops->DRT(s, ts->chip_data);
	mutex_unlock(&ts->mutex);

	if (ts->int_mode == BANNABLE) {
		enable_irq(ts->client->irq);
	}
	return 0;

}

static int DRT_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_DRT_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_DRT_proc_fops = {
	.owner = THIS_MODULE,
	.open  = DRT_open,
	.read  = seq_read,
	.release = single_release,
};

int synaptics_create_proc(struct touchpanel_data *ts, struct synaptics_proc_operations *syna_ops)
{
	int ret = 0;

	// touchpanel_auto_test interface
	struct proc_dir_entry *prEntry_tmp = NULL;
	ts->private_data = syna_ops;
	prEntry_tmp = proc_create_data("baseline_test", 0666, ts->prEntry_tp, &tp_auto_test_proc_fops, ts);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

	// show RT251 interface
	prEntry_tmp = proc_create_data("RT251", 0666, ts->prEntry_debug_tp, &tp_RT251_proc_fops, ts);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

	// show RT76 interface
	prEntry_tmp = proc_create_data("RT76", 0666, ts->prEntry_debug_tp, &tp_RT76_proc_fops, ts);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

	prEntry_tmp = proc_create_data("DRT", 0666, ts->prEntry_debug_tp, &tp_DRT_proc_fops, ts);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}
	return ret;
}
