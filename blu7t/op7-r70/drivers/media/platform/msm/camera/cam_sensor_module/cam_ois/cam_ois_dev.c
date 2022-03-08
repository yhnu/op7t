/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "cam_ois_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_ois_soc.h"
#include "cam_ois_core.h"
#include "cam_debug_util.h"
#include "linux/proc_fs.h"
#include "ois_fw/Ois.h"
#include "ois_fw/OisAPI.h"
#include <linux/kthread.h>
#include <linux/time.h>
#include "linux/slab.h"
#include <linux/delay.h>


#define OIS_DATA_LENGTH 32
#define OIS_DATA_SIZE 16
struct task_struct *test_task = NULL;
unsigned char *pOISdata_wide = NULL;
unsigned char *pOISdata_tele = NULL;

int iWriteIndex = 0;
struct mutex ois_mutex;
struct cam_ois_ctrl_t *ctrl_wide = NULL;
struct cam_ois_ctrl_t *ctrl_tele = NULL;
#define OIS_wide 0
#define OIS_tele 2
int ois_id = OIS_wide;



static int RamRead32A(struct cam_ois_ctrl_t *o_ctrl,
    UINT32 addr, UINT32* data)
{
    int32_t rc = 0;
    int retry = 3;
    int i;
    if (o_ctrl == NULL) {
        CAM_ERR(CAM_OIS, "Invalid Args dev");
        return -EINVAL;
    }
    //CAM_INFO(CAM_OIS, "o_ctrl valid, power state = %d, iomaster = %p dev" , o_ctrl->cam_ois_state,&(o_ctrl->io_master_info));
    for(i = 0; i < retry; i++)
    {
        rc = camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)addr, (uint32_t *)data,
            CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD);
        if (rc < 0) {
            CAM_ERR(CAM_OIS, "read 0x%04x failed, retry:%d", addr, i+1);
        } else {
            //CAM_INFO(CAM_OIS, "I2C read address = %p result = %d dev", addr, *data);
            return rc;
        }
    }
    //CAM_INFO(CAM_OIS, "I2C read address = %p result = %d dev", addr, *data);
    return rc;
}

int HallData(void *arg)
{
    UINT32 Raw;
    bool isAlive = true;
    UINT16 *pUint = NULL;
    UINT64 *pUint64 = NULL;
    struct timeval systemTime;
    iWriteIndex = 0;
    while (isAlive){
        if(kthread_should_stop()){
            isAlive = false;
            break;
        }
        //implement FIFO
        if (iWriteIndex == OIS_DATA_LENGTH){
            if(pOISdata_wide){
                memmove(pOISdata_wide, pOISdata_wide+OIS_DATA_SIZE, OIS_DATA_SIZE*(OIS_DATA_LENGTH-1));
            }
            if(pOISdata_tele){
                memmove(pOISdata_tele, pOISdata_tele+OIS_DATA_SIZE, OIS_DATA_SIZE*(OIS_DATA_LENGTH-1));
            }
           iWriteIndex--;
        }
        if (iWriteIndex < OIS_DATA_LENGTH){
            do_gettimeofday(&systemTime);
            if(ctrl_wide != NULL)
            {
                RamRead32A(ctrl_wide, 0xF01A , &Raw ); //0xF01A is hallx and hall y
                mutex_lock(&ois_mutex);
                pUint = (UINT16 *)(pOISdata_wide + iWriteIndex*OIS_DATA_SIZE);
                *pUint = Raw >> 16; //update hall x y
                pUint++;
                *pUint = Raw & 0xFFFF;
                pUint += 3;
                pUint64 = (UINT64 *)pUint;
                *pUint64 = (systemTime.tv_sec & 0xFFFFFFFF) * 1000000000 + (systemTime.tv_usec) * 1000; //update timestamp
                //CAM_ERR(CAM_OIS, "0xF01A = 0x%x, index=%d, size of uint %d", Raw,iWriteIndex, sizeof(unsigned int));
                mutex_unlock(&ois_mutex);
            }
            else{
                mutex_lock(&ois_mutex);
                memset (pOISdata_wide, 0, sizeof(unsigned char)*OIS_DATA_SIZE*OIS_DATA_LENGTH);
                mutex_unlock(&ois_mutex);
            }

            if(ctrl_tele!=   NULL)
            {
                RamRead32A(ctrl_tele, 0xF01A , &Raw ); //0xF01A is hallx and hall y
                mutex_lock(&ois_mutex);
                pUint = (UINT16 *)(pOISdata_tele + iWriteIndex*OIS_DATA_SIZE);
                *pUint = Raw >> 16; //update hall x y
                pUint++;
                *pUint = Raw & 0xFFFF;
                pUint += 3;
                pUint64 = (UINT64 *)pUint;
                *pUint64 = (systemTime.tv_sec & 0xFFFFFFFF) * 1000000000 + (systemTime.tv_usec) * 1000; //update timestamp
                //CAM_ERR(CAM_OIS, "0xF01A = 0x%x, index=%d, size of uint %d", Raw,iWriteIndex, sizeof(unsigned int));
                mutex_unlock(&ois_mutex);
            }
            else{
                mutex_lock(&ois_mutex);
                memset (pOISdata_tele, 0, sizeof(unsigned char)*OIS_DATA_SIZE*OIS_DATA_LENGTH);
                mutex_unlock(&ois_mutex);
            }
            iWriteIndex++;

        }
        //CAM_ERR(CAM_OIS, "0xF01A = 0x%x", Raw);
        usleep_range(3995, 4000);
    }
    do_exit(0);
    return 1;
}


static long cam_ois_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int                       rc     = 0;
	struct cam_ois_ctrl_t *o_ctrl = v4l2_get_subdevdata(sd);

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_ois_driver_cmd(o_ctrl, arg);
		break;
	default:
		rc = -ENOIOCTLCMD;
		break;
	}

	return rc;
}

static int cam_ois_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_ois_ctrl_t *o_ctrl =
		v4l2_get_subdevdata(sd);

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "o_ctrl ptr is NULL");
			return -EINVAL;
	}

	mutex_lock(&(o_ctrl->ois_mutex));
	cam_ois_shutdown(o_ctrl);
	mutex_unlock(&(o_ctrl->ois_mutex));

	return 0;
}

static int32_t cam_ois_update_i2c_info(struct cam_ois_ctrl_t *o_ctrl,
	struct cam_ois_i2c_info_t *i2c_info)
{
	struct cam_sensor_cci_client        *cci_client = NULL;

	if (o_ctrl->io_master_info.master_type == CCI_MASTER) {
		cci_client = o_ctrl->io_master_info.cci_client;
		if (!cci_client) {
			CAM_ERR(CAM_OIS, "failed: cci_client %pK",
				cci_client);
			return -EINVAL;
		}
		cci_client->cci_i2c_master = o_ctrl->cci_i2c_master;
		cci_client->sid = (i2c_info->slave_addr) >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->i2c_freq_mode = i2c_info->i2c_freq_mode;
	}

	return 0;
}

#ifdef CONFIG_COMPAT
static long cam_ois_init_subdev_do_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct cam_control cmd_data;
	int32_t rc = 0;

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_OIS,
			"Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_ois_subdev_ioctl(sd, cmd, &cmd_data);
		if (rc) {
			CAM_ERR(CAM_OIS,
				"Failed in ois suddev handling rc %d",
				rc);
			return rc;
		}
		break;
	default:
		CAM_ERR(CAM_OIS, "Invalid compat ioctl: %d", cmd);
		rc = -EINVAL;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_OIS,
				"Failed to copy from user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}
	return rc;
}
#endif

static const struct v4l2_subdev_internal_ops cam_ois_internal_ops = {
	.close = cam_ois_subdev_close,
};

static struct v4l2_subdev_core_ops cam_ois_subdev_core_ops = {
	.ioctl = cam_ois_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_ois_init_subdev_do_ioctl,
#endif
};

static struct v4l2_subdev_ops cam_ois_subdev_ops = {
	.core = &cam_ois_subdev_core_ops,
};

static int cam_ois_init_subdev_param(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;

	o_ctrl->v4l2_dev_str.internal_ops = &cam_ois_internal_ops;
	o_ctrl->v4l2_dev_str.ops = &cam_ois_subdev_ops;
	strlcpy(o_ctrl->device_name, CAM_OIS_NAME,
		sizeof(o_ctrl->device_name));
	o_ctrl->v4l2_dev_str.name = o_ctrl->device_name;
	o_ctrl->v4l2_dev_str.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	o_ctrl->v4l2_dev_str.ent_function = CAM_OIS_DEVICE_TYPE;
	o_ctrl->v4l2_dev_str.token = o_ctrl;

	rc = cam_register_subdev(&(o_ctrl->v4l2_dev_str));
	if (rc)
		CAM_ERR(CAM_OIS, "fail to create subdev");

	return rc;
}

static int cam_ois_i2c_driver_probe(struct i2c_client *client,
	 const struct i2c_device_id *id)
{
	int                          rc = 0;
	struct cam_ois_ctrl_t       *o_ctrl = NULL;
	struct cam_ois_soc_private  *soc_private = NULL;

	if (client == NULL || id == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args client: %pK id: %pK",
			client, id);
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CAM_ERR(CAM_OIS, "i2c_check_functionality failed");
		goto probe_failure;
	}

	o_ctrl = kzalloc(sizeof(*o_ctrl), GFP_KERNEL);
	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "kzalloc failed");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, o_ctrl);

	o_ctrl->soc_info.dev = &client->dev;
	o_ctrl->soc_info.dev_name = client->name;
	o_ctrl->ois_device_type = MSM_CAMERA_I2C_DEVICE;
	o_ctrl->io_master_info.master_type = I2C_MASTER;
	o_ctrl->io_master_info.client = client;

	soc_private = kzalloc(sizeof(struct cam_ois_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		rc = -ENOMEM;
		goto octrl_free;
	}

	o_ctrl->soc_info.soc_private = soc_private;
	rc = cam_ois_driver_soc_init(o_ctrl);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed: cam_sensor_parse_dt rc %d", rc);
		goto soc_free;
	}

	rc = cam_ois_init_subdev_param(o_ctrl);
	if (rc)
		goto soc_free;

	o_ctrl->cam_ois_state = CAM_OIS_INIT;

	return rc;

soc_free:
	kfree(soc_private);
octrl_free:
	kfree(o_ctrl);
probe_failure:
	return rc;
}

static int cam_ois_i2c_driver_remove(struct i2c_client *client)
{
	int                             i;
	struct cam_ois_ctrl_t          *o_ctrl = i2c_get_clientdata(client);
	struct cam_hw_soc_info         *soc_info;
	struct cam_ois_soc_private     *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "ois device is NULL");
		return -EINVAL;
	}

	CAM_INFO(CAM_OIS, "i2c driver remove invoked");
	soc_info = &o_ctrl->soc_info;

	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);

	mutex_lock(&(o_ctrl->ois_mutex));
	cam_ois_shutdown(o_ctrl);
	mutex_unlock(&(o_ctrl->ois_mutex));
	cam_unregister_subdev(&(o_ctrl->v4l2_dev_str));

	soc_private =
		(struct cam_ois_soc_private *)soc_info->soc_private;
	power_info = &soc_private->power_info;

	kfree(o_ctrl->soc_info.soc_private);
	v4l2_set_subdevdata(&o_ctrl->v4l2_dev_str.sd, NULL);
	kfree(o_ctrl);

	return 0;
}

static int32_t cam_ois_platform_driver_probe(
	struct platform_device *pdev)
{
	int32_t                         rc = 0;
	struct cam_ois_ctrl_t          *o_ctrl = NULL;
	struct cam_ois_soc_private     *soc_private = NULL;

	o_ctrl = kzalloc(sizeof(struct cam_ois_ctrl_t), GFP_KERNEL);
	if (!o_ctrl)
		return -ENOMEM;

	o_ctrl->soc_info.pdev = pdev;
	o_ctrl->pdev = pdev;
	o_ctrl->soc_info.dev = &pdev->dev;
	o_ctrl->soc_info.dev_name = pdev->name;

	o_ctrl->ois_device_type = MSM_CAMERA_PLATFORM_DEVICE;

	o_ctrl->io_master_info.master_type = CCI_MASTER;
	o_ctrl->io_master_info.cci_client = kzalloc(
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!o_ctrl->io_master_info.cci_client)
		goto free_o_ctrl;

	soc_private = kzalloc(sizeof(struct cam_ois_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		rc = -ENOMEM;
		goto free_cci_client;
	}
	o_ctrl->soc_info.soc_private = soc_private;
	soc_private->power_info.dev  = &pdev->dev;

	INIT_LIST_HEAD(&(o_ctrl->i2c_init_data.list_head));
	INIT_LIST_HEAD(&(o_ctrl->i2c_calib_data.list_head));
	INIT_LIST_HEAD(&(o_ctrl->i2c_mode_data.list_head));
	mutex_init(&(o_ctrl->ois_mutex));
#ifdef ENABLE_OIS_DELAY_POWER_DOWN
	o_ctrl->ois_power_down_thread_state = CAM_OIS_POWER_DOWN_THREAD_STOPPED;
	o_ctrl->ois_power_state = CAM_OIS_POWER_OFF;
	o_ctrl->ois_power_down_thread_exit = false;
	mutex_init(&(o_ctrl->ois_power_down_mutex));
#endif
	rc = cam_ois_driver_soc_init(o_ctrl);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed: soc init rc %d", rc);
		goto free_soc;
	}

	rc = cam_ois_init_subdev_param(o_ctrl);
	if (rc)
		goto free_soc;

	rc = cam_ois_update_i2c_info(o_ctrl, &soc_private->i2c_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed: to update i2c info rc %d", rc);
		goto unreg_subdev;
	}
	o_ctrl->bridge_intf.device_hdl = -1;

	platform_set_drvdata(pdev, o_ctrl);
	o_ctrl->cam_ois_state = CAM_OIS_INIT;

	return rc;
unreg_subdev:
	cam_unregister_subdev(&(o_ctrl->v4l2_dev_str));
free_soc:
	kfree(soc_private);
free_cci_client:
	kfree(o_ctrl->io_master_info.cci_client);
free_o_ctrl:
	kfree(o_ctrl);
	return rc;
}

static int cam_ois_platform_driver_remove(struct platform_device *pdev)
{
	int                             i;
	struct cam_ois_ctrl_t          *o_ctrl;
	struct cam_ois_soc_private     *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_hw_soc_info         *soc_info;

	o_ctrl = platform_get_drvdata(pdev);
	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "ois device is NULL");
		return -EINVAL;
	}

	CAM_INFO(CAM_OIS, "platform driver remove invoked");
	soc_info = &o_ctrl->soc_info;
	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);

	mutex_lock(&(o_ctrl->ois_mutex));
	cam_ois_shutdown(o_ctrl);
	mutex_unlock(&(o_ctrl->ois_mutex));
	cam_unregister_subdev(&(o_ctrl->v4l2_dev_str));

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	kfree(o_ctrl->soc_info.soc_private);
	kfree(o_ctrl->io_master_info.cci_client);
	platform_set_drvdata(pdev, NULL);
	v4l2_set_subdevdata(&o_ctrl->v4l2_dev_str.sd, NULL);
	kfree(o_ctrl);

	return 0;
}

static const struct of_device_id cam_ois_dt_match[] = {
	{ .compatible = "qcom,ois" },
	{ }
};


MODULE_DEVICE_TABLE(of, cam_ois_dt_match);

static struct platform_driver cam_ois_platform_driver = {
	.driver = {
		.name = "qcom,ois",
		.owner = THIS_MODULE,
		.of_match_table = cam_ois_dt_match,
	},
	.probe = cam_ois_platform_driver_probe,
	.remove = cam_ois_platform_driver_remove,
};
static const struct i2c_device_id cam_ois_i2c_id[] = {
	{ "msm_ois", (kernel_ulong_t)NULL},
	{ }
};

static struct i2c_driver cam_ois_i2c_driver = {
	.id_table = cam_ois_i2c_id,
	.probe  = cam_ois_i2c_driver_probe,
	.remove = cam_ois_i2c_driver_remove,
	.driver = {
		.name = "msm_ois",
	},
};

static ssize_t OISread(struct file *p_file,
	char __user *puser_buf, size_t count, loff_t *p_offset)
{

    unsigned char page[OIS_DATA_SIZE*OIS_DATA_LENGTH] = {0};
    int len = 0;
    if (puser_buf == NULL ) return 0;
    //CAM_INFO(CAM_OIS, "hall_x = %d, hall_y = %d, hall_x = 0x%x, hall_y = 0x%x", hall_x, hall_y, hall_x, hall_y);
    if (len > *p_offset) {
        len -= *p_offset;
    }
    else {
        len = 0;
    }
    mutex_lock(&ois_mutex);
    if (pOISdata_wide && ois_id == OIS_wide)
    {
        memcpy(page, pOISdata_wide, sizeof(unsigned char)*iWriteIndex*OIS_DATA_SIZE);
        memset (pOISdata_wide, 0, sizeof(unsigned char)*OIS_DATA_SIZE*OIS_DATA_LENGTH);
        memset (pOISdata_tele, 0, sizeof(unsigned char)*OIS_DATA_SIZE*OIS_DATA_LENGTH);
    }
    else if(pOISdata_tele && ois_id == OIS_tele)
    {
        memcpy(page, pOISdata_tele, sizeof(unsigned char)*iWriteIndex*OIS_DATA_SIZE);
        memset (pOISdata_wide, 0, sizeof(unsigned char)*OIS_DATA_SIZE*OIS_DATA_LENGTH);
        memset (pOISdata_tele, 0, sizeof(unsigned char)*OIS_DATA_SIZE*OIS_DATA_LENGTH);
    }
    if (copy_to_user(puser_buf, page, OIS_DATA_SIZE*iWriteIndex)) {
        CAM_ERR(CAM_OIS, "copy to user error");
        mutex_unlock(&ois_mutex);
        return -EFAULT;
    }
    iWriteIndex = 0;
    mutex_unlock(&ois_mutex);
    *p_offset += len < count ? len : count;

    return (len < count ? len : count);
}

static ssize_t OISwrite(struct file *p_file,
	const char __user *puser_buf,
	size_t count, loff_t *p_offset)
{
    unsigned char data = 0xff;
    if (puser_buf == NULL)
        CAM_INFO(CAM_OIS, "OISwrite buffer is NULL");
    else
    {
        copy_from_user(&data, puser_buf, sizeof(char));
        CAM_INFO(CAM_OIS, "OISwrite data = 0x%x", data);
    }

    if (data == 1)
    {
        CAM_INFO(CAM_OIS, "OIS thread create");
        if (test_task == NULL)
        {
            test_task = kthread_create(HallData, NULL,"OisThread");
            wake_up_process(test_task);
        }
    }
    else if (data == 0)
    {
        CAM_INFO(CAM_OIS, "OIS thread destory");
        kthread_stop(test_task);
        test_task = NULL;
    }
    else if (data == 2)    //need to mapping to camxeisnode
    {
        ois_id = OIS_wide;
    }
    else if (data == 3)   //need to mapping to camxeisnode
    {
        ois_id = OIS_tele;
    }
    return 0;
}


static const struct file_operations proc_file_fops = {
 .owner = THIS_MODULE,
 .read  = OISread,
 .write = OISwrite,
};

static struct cam_ois_registered_driver_t registered_driver = {
	0, 0};

static int __init cam_ois_driver_init(void)
{
	int rc = 0;
	struct proc_dir_entry *face_common_dir = NULL;
	struct proc_dir_entry *proc_file_entry = NULL;

	rc = platform_driver_register(&cam_ois_platform_driver);
	if (rc) {
		CAM_ERR(CAM_OIS, "platform_driver_register failed rc = %d",
			rc);
		return rc;
	}

	registered_driver.platform_driver = 1;

	rc = i2c_add_driver(&cam_ois_i2c_driver);
	if (rc) {
		CAM_ERR(CAM_OIS, "i2c_add_driver failed rc = %d", rc);
		return rc;
	}

	registered_driver.i2c_driver = 1;

	face_common_dir =  proc_mkdir("OIS", NULL);
	if(!face_common_dir) {
		CAM_ERR(CAM_OIS, "create dir fail CAM_ERROR API");
		//return FACE_ERROR_GENERAL;
	}

    proc_file_entry = proc_create("OISGyro", 0777, face_common_dir, &proc_file_fops);
    if(proc_file_entry == NULL)
        CAM_ERR(CAM_OIS, "Create fail");
    else
        CAM_INFO(CAM_OIS, "Create successs");

    if (pOISdata_wide == NULL){
        pOISdata_wide = kmalloc(sizeof(unsigned char)*OIS_DATA_LENGTH*OIS_DATA_SIZE, GFP_KERNEL);
        memset (pOISdata_wide, 0, sizeof(unsigned char)*OIS_DATA_SIZE*OIS_DATA_LENGTH);
    }
    if (pOISdata_tele == NULL){
        pOISdata_tele = kmalloc(sizeof(unsigned char)*OIS_DATA_LENGTH*OIS_DATA_SIZE, GFP_KERNEL);
        memset (pOISdata_tele, 0, sizeof(unsigned char)*OIS_DATA_SIZE*OIS_DATA_LENGTH);
    }

    mutex_init(&ois_mutex);

	return rc;
}

static void __exit cam_ois_driver_exit(void)
{
    if (pOISdata_wide){
        kfree(pOISdata_wide);
        pOISdata_wide = NULL;
    }
    if (pOISdata_tele){
        kfree(pOISdata_tele);
        pOISdata_tele = NULL;
    }
    mutex_destroy(&ois_mutex);
	if (registered_driver.platform_driver)
		platform_driver_unregister(&cam_ois_platform_driver);

	if (registered_driver.i2c_driver)
		i2c_del_driver(&cam_ois_i2c_driver);
}

module_init(cam_ois_driver_init);
module_exit(cam_ois_driver_exit);
MODULE_DESCRIPTION("CAM OIS driver");
MODULE_LICENSE("GPL v2");
