/*
 * This program is used to reserve kernel log in kernel
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/types.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/buffer_head.h>

MODULE_LICENSE("Dual BSD/GPL");

#define LOG_RESOURCE "/dev/kmsg"
#define LOG_TARGET "/dev/block/by-name/reserve1" //reserve1 partition
#define LOG_SIZE 655360 //log limit size
#define WB_BLOCK_SIZE 4096

struct delayed_work wb_workq;

void wb_workqueue_handler(struct work_struct *work)
{
	struct file *log_res_filp = NULL;
	struct file *log_tar_filp = NULL;
	char *buf = NULL;
	int write_size = 0;
	unsigned int ret = 0;
	unsigned int stop_size = 0;

	log_res_filp = filp_open(LOG_RESOURCE, O_RDONLY, 0600);
	if (IS_ERR(log_res_filp)) {
		ret = PTR_ERR(log_res_filp);
		log_res_filp = NULL;
		/*
		 * pr_info("[Op_kernel_log] Open fail %s error number is %d\n",
		 * LOG_RESOURCE, ret);
		 */
		goto out;
	}

	log_tar_filp = filp_open(LOG_TARGET, O_RDWR, 0600);
	if (IS_ERR(log_tar_filp)) {
		ret = PTR_ERR(log_tar_filp);
		log_tar_filp = NULL;
		/*
		 * pr_info("[Op_kernel_log] Open fail %s error number is %d\n",
		 * LOG_TARGET, ret);
		 */
		goto out;
	}

	log_res_filp->f_pos = 0;
	log_tar_filp->f_pos = 0;

	buf = vmalloc(WB_BLOCK_SIZE);
	if (!buf)
		goto out;
	memset(buf, 0, WB_BLOCK_SIZE);

	pr_info("[Op_kernel_log] syncing...\n");

	while ((ret = vfs_read(log_res_filp, buf, WB_BLOCK_SIZE,
		&log_res_filp->f_pos)) > 0) {

		write_size = 0;

		if ((stop_size + ret) > LOG_SIZE) {
			pr_info("[Op_kernel_log] Record stop\n");
			break;
		}

		while (ret) {
			write_size = vfs_write
				(log_tar_filp, buf, ret, &log_tar_filp->f_pos);
			if (write_size <= 0) {
				pr_info("[Op_kernel_log] Write back log fail\n");
				goto out;
			}
			stop_size += write_size;
			ret -= write_size;
		}
		memset(buf, 0, WB_BLOCK_SIZE);
	}

out:
	if (buf != NULL)
		kvfree(buf);
	if (log_res_filp != NULL)
		filp_close(log_res_filp, NULL);
	if (log_tar_filp != NULL)
		filp_close(log_tar_filp, NULL);
	if (stop_size == 0)
		schedule_delayed_work(&wb_workq, HZ/10);
	else {
		cancel_delayed_work(&wb_workq);
		pr_info("[Op_kernel_log] Stop write back from work queue\n");
	}
}

static int kernel_log_wb_int(void)
{
	pr_info("[Op_kernel_log] Start init\n");
	INIT_DELAYED_WORK(&wb_workq, wb_workqueue_handler);
	schedule_delayed_work(&wb_workq, HZ/10);
	pr_info("[Op_kernel_log] End init\n");
	return 0;
}

pure_initcall(kernel_log_wb_int);
