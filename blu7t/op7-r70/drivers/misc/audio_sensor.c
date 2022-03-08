
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/printk.h>
#include <linux/atomic.h>
#include <linux/poll.h>
#include <linux/ioctl.h>

struct notify_data {
	int state;
	int ack;
	atomic_t read_ready;
	struct mutex lock;
	wait_queue_head_t wait;
};

static struct notify_data audio_data = {
	.state = 0,
	.ack = -1,
	.read_ready = ATOMIC_INIT(0),
};

static struct notify_data sensor_data = {
	.state = 0,
	.ack = -1,
	.read_ready = ATOMIC_INIT(0),
};

#define IOCTL_AUDIO_SEND_ACK _IOW('S', 0, int)
#define IOCTL_SENSOR_RECEIVE_ACK _IOR('S', 1, int*)

static ssize_t audio_write(struct file *file,
		const char __user *buf, size_t len, loff_t *lof)
{
	int ret = 0;
	int state = 0;
	char kbuf[4];

	pr_debug("%s: enter\n", __func__);

	len = len > ARRAY_SIZE(kbuf) ?
			ARRAY_SIZE(kbuf) : len;
	ret = copy_from_user(kbuf, buf, len);
	if (ret) {
		pr_err("%s: can't copy %d size data\n", __func__, ret);
		return -EAGAIN;
	}

	if (sscanf(kbuf, "%d", &state))
		audio_data.state = state;

	pr_info("%s: audio_data.state is %#x\n", __func__, audio_data.state);
	atomic_set(&audio_data.read_ready, 1);
	wake_up(&audio_data.wait);

	return len;
}

static ssize_t audio_read(struct file *file,
		char __user *buf, size_t len, loff_t *lof)
{
	int ret = 0;

	pr_debug("%s: enter\n", __func__);

	if (!atomic_read(&audio_data.read_ready))
		return 0;

	len = len > sizeof(audio_data.state) ? sizeof(audio_data.state) : len;
	ret = copy_to_user(buf, (char *)&audio_data.state, len);
	if (ret) {
		pr_err("%s: can't copy %d size data\n", __func__, ret);
		return -EAGAIN;
	}

	atomic_set(&audio_data.read_ready, 0);

	return len;
}

static unsigned int audio_poll(struct file *file,
		struct poll_table_struct *table)
{
	unsigned int mask = 0;

	pr_debug("%s: enter\n", __func__);

	poll_wait(file, &audio_data.wait, table);

	if (atomic_read(&audio_data.read_ready))
		mask |= POLLIN | POLLRDNORM;

	pr_info("%s: exit mask is %#x\n", __func__, mask);
	return mask;
}


static int audio_close(struct inode *inode, struct file *file)
{
	pr_debug("%s: enter\n", __func__);

	return 0;
}

static int audio_open(struct inode *inode, struct file *file)
{
	pr_debug("%s: enter\n", __func__);

	file->private_data = (void *)&audio_data;

	return 0;
}

static ssize_t sensor_write(struct file *file,
		const char __user *buf, size_t len, loff_t *lof)
{
	int ret = 0;
	int state = 0;
	char kbuf[4];

	pr_debug("%s: enter\n", __func__);

	len = len > ARRAY_SIZE(kbuf) ?
			ARRAY_SIZE(kbuf) : len;
	ret = copy_from_user(kbuf, buf, len);
	if (ret) {
		pr_err("%s: can't copy %d size data\n", __func__, ret);
		return -EAGAIN;
	}

	if (sscanf(kbuf, "%d", &state))
		sensor_data.state = state;

	pr_info("%s: state is %#x\n", __func__, sensor_data.state);

	atomic_set(&sensor_data.read_ready, 1);
	wake_up(&sensor_data.wait);

	return len;
}

static ssize_t sensor_read(struct file *file,
		char __user *buf, size_t len, loff_t *lof)
{
	int ret = 0;

	pr_debug("%s: enter\n", __func__);

	if (!atomic_read(&sensor_data.read_ready))
		return -EAGAIN;

	len = len > sizeof(sensor_data.state) ? sizeof(sensor_data.state) : len;
	ret = copy_to_user(buf, (char *)&sensor_data.state, len);
	if (ret) {
		pr_err("%s: can't copy %d size data\n", __func__, ret);
		return -EAGAIN;
	}

	atomic_set(&sensor_data.read_ready, 0);

	return len;
}

unsigned int sensor_poll(struct file *file, struct poll_table_struct *table)
{
	unsigned int mask = 0;

	pr_debug("%s: enter\n", __func__);

	poll_wait(file, &sensor_data.wait, table);

	if (atomic_read(&sensor_data.read_ready))
		mask |= POLLIN | POLLRDNORM;

	pr_info("%s: exit mask is %#x\n", __func__, mask);
	return mask;
}

static long share_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	char *temp = NULL;
	int ret = 0;
	struct notify_data *data = file->private_data;

	pr_debug("%s: arg is %#x\n", __func__, arg);

	switch (cmd) {
	case IOCTL_AUDIO_SEND_ACK:
		data->ack = arg == 1 ? 1 : 0;  //1 success, 0 fail
		pr_info("%s: send ack %d\n", __func__, data->ack);
		break;

	case IOCTL_SENSOR_RECEIVE_ACK:
		temp = (char *)arg;
		ret = copy_to_user(temp, (char *)&data->ack, sizeof(data->ack));
		if (ret) {
			pr_err("%s: can't copy %d size data\n", __func__, ret);
			return -EAGAIN;
		}

		pr_debug("%s: receive ack %d\n", __func__, data->ack);
		data->ack = -1; //no response
		break;

	default:
		pr_info("%s: INVAILD cmd\n", __func__);
		break;
	}

	return 0;
}

#ifdef CONFIG_COMPAT
static long share_compat_ioctl(struct file *file,
					unsigned int cmd, unsigned long arg)
{
	return share_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int sensor_close(struct inode *inode, struct file *file)
{
	pr_debug("%s: enter\n", __func__);

	return 0;
}

static int sensor_open(struct inode *inode, struct file *file)
{
	pr_debug("%s: enter\n", __func__);

	file->private_data = (void *)&sensor_data;

	return 0;
}

static const struct file_operations audio_opt = {
	.owner = THIS_MODULE,
	.open = audio_open,
	.release = audio_close,
	.write = audio_write,
	.read = audio_read,
	.poll = audio_poll,
	.unlocked_ioctl = share_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = share_compat_ioctl,
#endif
};

static const struct file_operations sensor_opt = {
	.owner = THIS_MODULE,
	.open = sensor_open,
	.release = sensor_close,
	.write = sensor_write,
	.read = sensor_read,
	.poll = sensor_poll,
	.unlocked_ioctl = share_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = share_compat_ioctl,
#endif
};

static struct miscdevice audio_misc = {
	.fops = &audio_opt,
	.name = "audio_ultrasound",
	.minor = MISC_DYNAMIC_MINOR,
};

static struct miscdevice sensor_misc = {
	.fops = &sensor_opt,
	.name = "sensor_ultrasound",
	.minor = MISC_DYNAMIC_MINOR,
};



static int __init audio_sensor_init(void)
{
	int ret = 0;

	ret = misc_register(&audio_misc);
	if (ret) {
		pr_err("misc register audio_misc fail, error = %d!\n", ret);
		return ret;
	}

	ret = misc_register(&sensor_misc);
	if (ret) {
		pr_err("misc register audio_misc fail, error = %d!\n", ret);
		return ret;
	}

	mutex_init(&audio_data.lock);
	mutex_init(&sensor_data.lock);

	init_waitqueue_head(&audio_data.wait);
	init_waitqueue_head(&sensor_data.wait);

	pr_info("%s: success!!\n", __func__);

	return ret;
}

static void __exit audio_sensor_exit(void)
{
	misc_deregister(&audio_misc);
	misc_deregister(&sensor_misc);

	mutex_destroy(&audio_data.lock);
	mutex_destroy(&sensor_data.lock);
}

module_exit(audio_sensor_exit);
module_init(audio_sensor_init);
MODULE_LICENSE("GPL");
