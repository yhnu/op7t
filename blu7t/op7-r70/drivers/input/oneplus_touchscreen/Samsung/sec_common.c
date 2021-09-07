#include "sec_common.h"

/*******LOG TAG Declear*****************************/

#define TPD_DEVICE "sec_common"
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)
#define TPD_DEBUG(a, arg...)\
    do{\
        if (tp_debug)\
        pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

/*********** sec tool operate content***********************/
u8 lv1cmd;
static int lv1_readsize;

static ssize_t sec_ts_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_ts_regreadsize_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_ts_regread_show(struct device *dev, struct device_attribute *attr, char *buf);

static DEVICE_ATTR(sec_ts_reg, (S_IWUSR | S_IWGRP), NULL, sec_ts_reg_store);
static DEVICE_ATTR(sec_ts_regreadsize, (S_IWUSR | S_IWGRP), NULL, sec_ts_regreadsize_store);
static DEVICE_ATTR(sec_ts_regread, S_IRUGO, sec_ts_regread_show, NULL);

static struct attribute *cmd_attributes[] = {
    &dev_attr_sec_ts_reg.attr,
    &dev_attr_sec_ts_regreadsize.attr,
    &dev_attr_sec_ts_regread.attr,
    NULL,
};

static struct attribute_group cmd_attr_group = {
    .attrs = cmd_attributes,
};

static ssize_t sec_ts_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    if (size > 0) {
        mutex_lock(&ts->mutex);
        touch_i2c_write(ts->client, (u8 *)buf, size);
        mutex_unlock(&ts->mutex);
    }

    TPD_DEBUG("%s: 0x%x, 0x%x, size %d\n", __func__, buf[0], buf[1], (int)size);
    return size;
}

static ssize_t sec_ts_regread_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);
    int ret = -1;
    u8 *read_lv1_buff = NULL;
    int length = 0, remain = 0, offset = 0;

    disable_irq_nosync(ts->irq);
    mutex_lock(&ts->mutex);

    read_lv1_buff = kzalloc(lv1_readsize, GFP_KERNEL);
    if (!read_lv1_buff)
        goto malloc_err;

    remain = lv1_readsize;
    offset = 0;
    do {
        if (remain >= I2C_BURSTMAX)
            length = I2C_BURSTMAX;
        else
            length = remain;

        if (offset == 0)
            ret = touch_i2c_read_block(ts->client, lv1cmd, length, &read_lv1_buff[offset]);
        else
            ret = touch_i2c_read(ts->client, NULL, 0, &read_lv1_buff[offset], length);

        if (ret < 0) {
            TPD_INFO("%s: i2c read %x command, remain =%d\n", __func__, lv1cmd, remain);
            goto i2c_err;
        }

        remain -= length;
        offset += length;
    } while (remain > 0);

    TPD_DEBUG("%s: lv1_readsize = %d\n", __func__, lv1_readsize);
    memcpy(buf, read_lv1_buff, lv1_readsize);

i2c_err:
    kfree(read_lv1_buff);
malloc_err:
    mutex_unlock(&ts->mutex);
    enable_irq(ts->irq);

    return lv1_readsize;
}

static ssize_t sec_ts_regreadsize_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    mutex_lock(&ts->mutex);

    lv1cmd = buf[0];
    lv1_readsize = ((unsigned int)buf[4] << 24) |
            ((unsigned int)buf[3] << 16) | ((unsigned int) buf[2] << 8) | ((unsigned int)buf[1] << 0);

    mutex_unlock(&ts->mutex);

    return size;
}

//create sec debug interfaces
void sec_raw_device_init(struct touchpanel_data *ts)
{
    int ret = -1;
    struct class *sec_class = NULL;

    sec_class = class_create(THIS_MODULE, "sec");
    ret = IS_ERR_OR_NULL(sec_class);
    if (ret) {
        TPD_INFO("%s: fail to create class\n", __func__);
        return;
    }

    ts->dev = device_create(sec_class, NULL, 0, ts, "sec_ts");
    ret = IS_ERR(ts->dev);
    if (ret) {
        TPD_INFO("%s: fail - device_create\n", __func__);
        return;
    }

    ret = sysfs_create_group(&ts->dev->kobj, &cmd_attr_group);
    if (ret < 0) {
        TPD_INFO("%s: fail - sysfs_create_group\n", __func__);
        goto err_sysfs;
    }

    TPD_INFO("create debug interface success\n");
    return;
err_sysfs:
    TPD_INFO("%s: fail\n", __func__);
    return;
}


void sec_limit_read(struct seq_file *s, struct touchpanel_data *ts)
{
    int ret = 0, m = 0, i = 0, j = 0, item_cnt = 0, array_index = 0;
    const struct firmware *fw = NULL;
    struct sec_test_header *ph = NULL;
    struct sec_test_item_header *item_head = NULL;
    uint32_t *p_item_offset = NULL;
    int32_t *p_data32 = NULL;

    ret = request_firmware(&fw, ts->panel_data.test_limit_name, ts->dev);
    if (ret < 0) {
        TPD_INFO("Request firmware failed - %s (%d)\n", ts->panel_data.test_limit_name, ret);
        seq_printf(s, "Request failed, Check the path\n");
        return;
    }

    ph = (struct sec_test_header *)(fw->data);
    p_item_offset = (uint32_t *)(fw->data + 16);
    if ((ph->magic1 != 0x494D494C) || (ph->magic2 != 0x474D4954)) {
        TPD_INFO("limit image is not generated\n");
        seq_printf(s, "limit image is not generated\n");
        release_firmware(fw);
        return;
    }

    for (i = 0; i < 8*sizeof(ph->test_item); i++) {
        if ((ph->test_item >> i) & 0x01 ) {
            item_cnt++;
        }
    }
    if (!item_cnt) {
        TPD_INFO("limit image has no test item\n");
        seq_printf(s, "limit image has no test item\n");
    }

    for (m = 0; m < item_cnt; m++) {
        item_head = (struct sec_test_item_header *)(fw->data + p_item_offset[m]);
        if (item_head->item_magic != 0x4F50504F) {
            seq_printf(s, "item: %d limit data has some problem\n", item_head->item_bit);
            continue;
        }

        seq_printf(s, "item %d[size %d, limit type %d, para num %d] :\n", item_head->item_bit, item_head->item_size, item_head->item_limit_type, item_head->para_num);
        if (item_head->item_limit_type == LIMIT_TYPE_NO_DATA) {
            seq_printf(s, "no limit data\n");
        } else if (item_head->item_limit_type == LIMIT_TYPE_CERTAIN_DATA) {
            p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset);
            seq_printf(s, "top limit data: %d\n", *p_data32);
            p_data32 = (int32_t *)(fw->data + item_head->floor_limit_offset);
            seq_printf(s, "floor limit data: %d\n", *p_data32);
        } else if (item_head->item_limit_type == LIMIT_TYPE_EACH_NODE_DATA) {
            if (item_head->item_bit == TYPE_MUTUAL_RAW_OFFSET_DATA_SDC) {
                p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset);
                seq_printf(s, "mutual raw top data: \n");
                for (i = 0; i < ts->hw_res.TX_NUM; i++) {
                    seq_printf(s, "[%02d] ", i);
                    for (j = 0; j < ts->hw_res.RX_NUM; j++) {
                        array_index = i * ts->hw_res.RX_NUM + j;
                        seq_printf(s, "%4d, ", p_data32[array_index]);
                    }
                    seq_printf(s, "\n");
                }

                p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset + 4 * ts->hw_res.TX_NUM * ts->hw_res.RX_NUM);
                seq_printf(s, "mutual gap top data: \n");
                for (i = 0; i < ts->hw_res.TX_NUM; i++) {
                    seq_printf(s, "[%02d] ", i);
                    for (j = 0; j < ts->hw_res.RX_NUM; j++) {
                        array_index = i * ts->hw_res.RX_NUM + j;
                        seq_printf(s, "%4d, ", p_data32[array_index]);
                    }
                    seq_printf(s, "\n");
                }

                p_data32 = (int32_t *)(fw->data + item_head->floor_limit_offset);
                seq_printf(s, "mutual raw floor data: \n");
                for (i = 0; i < ts->hw_res.TX_NUM; i++) {
                    seq_printf(s, "[%02d] ", i);
                    for (j = 0; j < ts->hw_res.RX_NUM; j++) {
                        array_index = i * ts->hw_res.RX_NUM + j;
                        seq_printf(s, "%4d, ", p_data32[array_index]);
                    }
                    seq_printf(s, "\n");
                }

                p_data32 = (int32_t *)(fw->data + item_head->floor_limit_offset + 4 * ts->hw_res.TX_NUM * ts->hw_res.RX_NUM);
                seq_printf(s, "mutual gap floor data: \n");
                for (i = 0; i < ts->hw_res.TX_NUM; i++) {
                    seq_printf(s, "[%02d] ", i);
                    for (j = 0; j < ts->hw_res.RX_NUM; j++) {
                        array_index = i * ts->hw_res.RX_NUM + j;
                        seq_printf(s, "%4d, ", p_data32[array_index]);
                    }
                    seq_printf(s, "\n");
                }
            } else if(item_head->item_bit == TYPE_SELF_RAW_OFFSET_DATA_SDC){
                p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset);
                seq_printf(s, "tx top data: \n");
                for (i = 0; i < ts->hw_res.TX_NUM; i++) {
                    seq_printf(s, "%4d, ", p_data32[i]);
                }
                seq_printf(s, "\n");

                p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset + 4*ts->hw_res.TX_NUM);
                seq_printf(s, "tx floor data: \n");
                for (i = 0; i < ts->hw_res.TX_NUM; i++) {
                    seq_printf(s, "%4d, ", p_data32[i]);
                }
                seq_printf(s, "\n");

                p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset + 2*4*ts->hw_res.TX_NUM);
                seq_printf(s, "rx top data: \n");
                for (i = 0; i < ts->hw_res.RX_NUM; i++) {
                    seq_printf(s, "%4d, ", p_data32[i]);
                }
                seq_printf(s, "\n");

                p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset + 2*4*ts->hw_res.TX_NUM + 4*ts->hw_res.RX_NUM);
                seq_printf(s, "rx floor data: \n");
                for (i = 0; i < ts->hw_res.RX_NUM; i++) {
                    seq_printf(s, "%4d, ", p_data32[i]);
                }
                seq_printf(s, "\n");
            } else if (item_head->item_bit == TYPE_MUTU_RAW_NOI_P2P) {
                p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset);
                seq_printf(s, "noise top data: \n");
                for (i = 0; i < ts->hw_res.TX_NUM; i++) {
                    seq_printf(s, "[%02d] ", i);
                    for (j = 0; j < ts->hw_res.RX_NUM; j++) {
                        array_index = i * ts->hw_res.RX_NUM + j;
                        seq_printf(s, "%4d, ", p_data32[array_index]);
                    }
                    seq_printf(s, "\n");
                }

                p_data32 = (int32_t *)(fw->data + item_head->floor_limit_offset);
                seq_printf(s, "noise floor data: \n");
                for (i = 0; i < ts->hw_res.TX_NUM; i++) {
                    seq_printf(s, "[%02d] ", i);
                    for (j = 0; j < ts->hw_res.RX_NUM; j++) {
                        array_index = i * ts->hw_res.RX_NUM + j;
                        seq_printf(s, "%4d, ", p_data32[array_index]);
                    }
                    seq_printf(s, "\n");
                }
            }
        }

        p_data32 = (int32_t *)(fw->data + p_item_offset[m] + sizeof(struct sec_test_item_header));
        if (item_head->para_num) {
            seq_printf(s, "parameter:");
            for (j = 0; j < item_head->para_num; j++) {
                seq_printf(s, "%d, ", p_data32[j]);
            }
            seq_printf(s, "\n");
        }
        seq_printf(s, "\n");
    }

    release_firmware(fw);
}

/************ sec auto test content*************************/

static int tp_auto_test_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct sec_proc_operations *sec_ops;
    const struct firmware *fw = NULL;
    struct timespec now_time;
    struct rtc_time rtc_now_time;
    mm_segment_t old_fs;
    uint8_t data_buf[128];
    int fd = -1, ret = -1;
    struct sec_test_header *test_head = NULL;
    uint32_t *p_data32 = NULL;

    struct sec_testdata sec_testdata =
    {
        .TX_NUM = 0,
        .RX_NUM = 0,
        .fd = -1,
        .irq_gpio = -1,
        .TP_FW = 0,
        .fw = NULL,
        .test_item = 0,
    };

    if (!ts)
        return 0;
    sec_ops = (struct sec_proc_operations *)ts->private_data;
    if (!sec_ops)
        return 0;
    if (!sec_ops->auto_test) {
        seq_printf(s, "Not support auto-test proc node\n");
        return 0;
    }

    //step1:disable_irq && get mutex locked
    disable_irq_nosync(ts->irq);
    mutex_lock(&ts->mutex);

    if (ts->esd_handle_support) {
        esd_handle_switch(&ts->esd_info, false);
    }

    //step2: create a file to store test data in /sdcard/Tp_Test
    getnstimeofday(&now_time);
    rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
    snprintf(data_buf, 128, "/sdcard/TpTestReport/screenOn/tp_testlimit_%02d%02d%02d-%02d%02d%02d-utc.csv",
            (rtc_now_time.tm_year + 1900) % 100, rtc_now_time.tm_mon + 1, rtc_now_time.tm_mday,
            rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    sys_mkdir("/sdcard/TpTestReport", 0666);
    sys_mkdir("/sdcard/TpTestReport/screenOn", 0666);
    fd = sys_open(data_buf, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd < 0) {
        TPD_INFO("Open log file '%s' failed.\n", data_buf);
        set_fs(old_fs);
        mutex_unlock(&ts->mutex);
        enable_irq(ts->irq);
        return 0;
    }

    //step3:request test limit data from userspace
    ret = request_firmware(&fw, ts->panel_data.test_limit_name, ts->dev);
    if (ret < 0) {
        TPD_INFO("Request firmware failed - %s (%d)\n", ts->panel_data.test_limit_name, ret);
        seq_printf(s, "No limit IMG\n");
        sys_close(fd);
        set_fs(old_fs);
        mutex_unlock(&ts->mutex);
        enable_irq(ts->irq);
        return 0;
    }

    //step4: decode the limit image
    test_head = (struct sec_test_header *)fw->data;
    p_data32 = (uint32_t *)(fw->data + 16);
    if ((test_head->magic1 != 0x494D494C) || (test_head->magic2 != 0x474D4954)) {
        TPD_INFO("limit image is not generated\n");
        seq_printf(s, "limit image is not generated by\n");
        goto OUT;
    }
    TPD_INFO("current test item: %llx\n", test_head->test_item);

    //init syna_testdata
    sec_testdata.fd = fd;
    sec_testdata.TX_NUM = ts->hw_res.TX_NUM;
    sec_testdata.RX_NUM = ts->hw_res.RX_NUM;
    sec_testdata.irq_gpio = ts->hw_res.irq_gpio;
    sec_testdata.TP_FW = ts->panel_data.TP_FW;
    sec_testdata.fw = fw;
    sec_testdata.test_item = test_head->test_item;

    sec_ops->auto_test(s, ts->chip_data, &sec_testdata);

OUT:
    //step4: close file && release test limit firmware
    if (fd >= 0) {
        sys_close(fd);
        set_fs(old_fs);
    }
    release_firmware(fw);

    //step5: return to normal mode
    ts->ts_ops->reset(ts->chip_data);
    operate_mode_switch(ts);

    //step6: unlock the mutex && enable irq trigger
    mutex_unlock(&ts->mutex);
    enable_irq(ts->irq);

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

static int calibrate_fops_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct sec_proc_operations *sec_ops = (struct sec_proc_operations *)ts->private_data;

    if (!sec_ops->calibrate)
        return 0;

    disable_irq_nosync(ts->irq);
    mutex_lock(&ts->mutex);
    if (!ts->touch_count) {
        sec_ops->calibrate(s, ts->chip_data);
    } else {
        seq_printf(s, "1 error, release touch on the screen\n");
    }
    mutex_unlock(&ts->mutex);
    enable_irq(ts->irq);

    return 0;
}

static int proc_calibrate_fops_open(struct inode *inode, struct file *file)
{
    return single_open(file, calibrate_fops_read_func, PDE_DATA(inode));
}

static const struct file_operations proc_calibrate_fops = {
    .owner = THIS_MODULE,
    .open  = proc_calibrate_fops_open,
    .read  = seq_read,
    .release = single_release,
};

static int verify_fops_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct sec_proc_operations *sec_ops = (struct sec_proc_operations *)ts->private_data;

    if (!sec_ops->verify_calibration)
        return 0;

    disable_irq_nosync(ts->irq);
    mutex_lock(&ts->mutex);
    if (!ts->touch_count) {
        sec_ops->verify_calibration(s, ts->chip_data);
    } else {
        seq_printf(s, "1 error, skip verify when touch on screen\n");
    }
    mutex_unlock(&ts->mutex);
    enable_irq(ts->irq);

    return 0;
}

static int proc_verify_fops_open(struct inode *inode, struct file *file)
{
    return single_open(file, verify_fops_read_func, PDE_DATA(inode));
}

static const struct file_operations proc_verify_fops = {
    .owner = THIS_MODULE,
    .open  = proc_verify_fops_open,
    .read  = seq_read,
    .release = single_release,
};

static int calibrate_data_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct sec_proc_operations *sec_ops = (struct sec_proc_operations *)ts->private_data;

    if (!sec_ops->calibration_data)
        return 0;

	if (ts->is_suspended)
		return 0;
    disable_irq_nosync(ts->irq);
    mutex_lock(&ts->mutex);
    if (!ts->touch_count) {
        sec_ops->calibration_data(s, ts->chip_data);
    } else {
        seq_printf(s, "1 error, skip calibrate_data when touch on screen\n");
    }
    mutex_unlock(&ts->mutex);
    enable_irq(ts->irq);

    return 0;
}

static int proc_calibrate_data_fops_open(struct inode *inode, struct file *file)
{
    return single_open(file, calibrate_data_read_func, PDE_DATA(inode));
}

static const struct file_operations proc_calibrate_data_fops = {
    .owner = THIS_MODULE,
    .open  = proc_calibrate_data_fops_open,
    .read  = seq_read,
    .release = single_release,
};

//proc/touchpanel/baseline_test
int sec_create_proc(struct touchpanel_data *ts, struct sec_proc_operations *sec_ops)
{
    int ret = 0;

    // touchpanel_auto_test interface
    struct proc_dir_entry *prEntry_tmp = NULL;
    ts->private_data = sec_ops;
    prEntry_tmp = proc_create_data("baseline_test", 0666, ts->prEntry_tp, &tp_auto_test_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    prEntry_tmp = proc_create_data("calibration", 0666, ts->prEntry_tp, &proc_calibrate_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    prEntry_tmp = proc_create_data("calibration_verify", 0666, ts->prEntry_tp, &proc_verify_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

	prEntry_tmp = proc_create_data("calibrate_data", 0666, ts->prEntry_tp, &proc_calibrate_data_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }
    return ret;
}