/*
 *Copyright (c)  2018  OnePlus Mobile Comm Corp., Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include "internal.h"
#include "fg_uid.h"

#define FG_RW (S_IWUSR|S_IRUSR|S_IWGRP|S_IRGRP|S_IWOTH|S_IROTH)

struct fg_info fginfo = {
	.fg_num = 0,
	.fg_uids = -555,
};

static struct proc_dir_entry *fg_dir;

bool is_fg(int uid)
{
	bool ret = false;
	if (uid == fginfo.fg_uids)
	ret = true;
	return ret;
}

static int fg_uids_show(struct seq_file *m, void *v)
{
	seq_printf(m, "fg_uids: %d\n", fginfo.fg_uids);
	return 0;
}

static int fg_uids_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, fg_uids_show, inode);
}

static ssize_t fg_uids_write(struct file *file, const char __user *buf,
							size_t count, loff_t *ppos)
{
	char buffer[MAX_ARRAY_LENGTH];
	int err = 0;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user((void *)buffer, buf, count)) {
		err = -EFAULT;
		goto out;
	}

	fginfo.fg_uids = simple_strtol(buffer, NULL, 0);
	fginfo.fg_num = 1;
out:
	return err < 0 ? err : count;
}

static const struct file_operations proc_fg_uids_operations = {
	.open       = fg_uids_open,
	.read       = seq_read,
	.write      = fg_uids_write,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static void uids_proc_fs_init(struct proc_dir_entry *p_parent)
{
	struct proc_dir_entry *p_temp;

	if (!p_parent)
		goto out_p_temp;

	p_temp = proc_create(FS_FG_UIDS, FG_RW, p_parent, &proc_fg_uids_operations);
	if (!p_temp)
		goto out_p_temp;

out_p_temp:
	return ;
}

static int __init fg_uids_init(void)
{
	struct proc_dir_entry *p_parent;

	p_parent = proc_mkdir(FS_FG_INFO_PATH, fg_dir);
	if (!p_parent) {
		return -ENOMEM;
	}
	uids_proc_fs_init(p_parent);
	return 0;
}

static  void __exit fg_uids_exit(void)
{
	if (!fg_dir)
		return ;

	remove_proc_entry(FS_FG_UIDS, fg_dir);
}

module_init(fg_uids_init);
module_exit(fg_uids_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jared Wu <jared.wu@oneplus.com>");
MODULE_DESCRIPTION("optimize foreground process to promote performance");
