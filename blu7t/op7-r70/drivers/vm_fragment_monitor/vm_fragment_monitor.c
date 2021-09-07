/********************************************************************************
** Copyright (C), 2014-2019, OnePlus Mobile Comm Corp., Ltd
** All rights reserved.
**
** File: vm_fragment_monitor.c
** Version Number: 1.0
** Description:
**   This driver is to get the max virtual memory fragment gap by pid
**   For detail, Please refer to [OSP-3622]
**
** ------------------------------------------------------------------------------
**  <Author>         <Date>        <Version>    <Description>
**  allen.lv@ASTI    2019-10-12    v1           add init version
**
*********************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/sched/task.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kernel_stat.h>
#include <linux/sched/mm.h>
#include <linux/mm.h>
#include <linux/slab.h>

#define GET_GAP_SIZE  _IOR('a', 1, unsigned int)

static dev_t devno;
static struct cdev *cdev;
static struct class *fragment_monitor_class;
static struct device *fragment_monitor_dev;


static long monitor_fragment_ioctl(struct file *file, unsigned int cmd, unsigned long args)
{
	unsigned long pid;
	struct task_struct *task;
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	unsigned long vm_fragment_gap_max = 0;
	void __user *parg = (void __user *)args;

	if (cmd != GET_GAP_SIZE)
		return -EFAULT;

	if (copy_from_user(&pid, parg, sizeof(unsigned long)))
		return -EFAULT;

	rcu_read_lock();
	task = find_task_by_vpid((int)pid);

	if (!task) {
		rcu_read_unlock();
		return -ESRCH;
	}

	get_task_struct(task);
	rcu_read_unlock();

	mm = get_task_mm(task);
	if (!mm) {
		put_task_struct(task);
		return -ENOMEM;
	}

	if (RB_EMPTY_ROOT(&mm->mm_rb))
	{
		mmput(mm);
		put_task_struct(task);
		return -ENOMEM;
	}

	vma = rb_entry(mm->mm_rb.rb_node, struct vm_area_struct, vm_rb);
	vm_fragment_gap_max = (vma->rb_subtree_gap >> 20);
	mmput(mm);
	put_task_struct(task);

	pr_info("monitor_fragment_ioctl : pid=%d, vm_fragment_gap_max=%d\n", (int)pid, (int)vm_fragment_gap_max);

	if (copy_to_user(parg, &vm_fragment_gap_max, sizeof(unsigned long)))
		return -EFAULT;

	return 0;
}

static const struct file_operations chrdev_fops = {
	.owner	= THIS_MODULE,
	.unlocked_ioctl = monitor_fragment_ioctl,
};

static int chrdev_register(void)
{
	int ret = 0;

	cdev = kzalloc(sizeof(struct cdev), GFP_KERNEL);
	if (!cdev)
		return -ENOMEM;

	ret = alloc_chrdev_region(&devno, 0, 1, "fragment_monitor");
	if (ret)
		goto err_alloc_chardev;

	cdev_init(cdev, &chrdev_fops);
	cdev->owner = THIS_MODULE;
	ret = cdev_add(cdev, devno, 1);
	if (ret)
		goto err_cdev_add;

	fragment_monitor_class = class_create(THIS_MODULE, "fragment_monitor");
	if (IS_ERR(fragment_monitor_class)) {
		ret = PTR_ERR(fragment_monitor_class);
		goto err_create_class;
	}
	fragment_monitor_dev = device_create(fragment_monitor_class, NULL, devno, NULL, "fragment_monitor");
	if (IS_ERR(fragment_monitor_dev))
		goto err_create_device;

	return 0;

err_create_device:
	class_destroy(fragment_monitor_class);
err_create_class:
	cdev_del(cdev);
err_cdev_add:
	unregister_chrdev_region(devno, 1);
err_alloc_chardev:
	kfree(cdev);
	return ret;
}

static int __init monitor_fragment_init(void)
{
	int ret = 0;
	ret = chrdev_register();
	return ret;
}

static void __exit monitor_fragment_exit(void)
{
	cdev_del(cdev);
	unregister_chrdev_region(devno, 1);
}


module_init(monitor_fragment_init);
module_exit(monitor_fragment_exit);
MODULE_LICENSE("GPL v2");

