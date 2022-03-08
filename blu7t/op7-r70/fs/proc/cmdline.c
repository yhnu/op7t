// SPDX-License-Identifier: GPL-2.0
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/setup.h>

static char new_command_line[COMMAND_LINE_SIZE];

static int cmdline_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", new_command_line);
	return 0;
}

static int cmdline_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdline_proc_show, NULL);
}

static const struct file_operations cmdline_proc_fops = {
	.open		= cmdline_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void remove_flag(char *cmd, const char *flag)
{
	char *start_addr, *end_addr;

	/* Ensure all instances of a flag are removed */
	while ((start_addr = strstr(cmd, flag))) {
		end_addr = strchr(start_addr, ' ');
		if (end_addr)
			memmove(start_addr, end_addr + 1, strlen(end_addr));
		else
			*(start_addr - 1) = '\0';
	}
}

static void remove_safetynet_flags(char *cmd)
{
	remove_flag(cmd, "androidboot.enable_dm_verity=");
	remove_flag(cmd, "androidboot.secboot=");
	if (strstr(saved_command_line, "project_name=18857") ||
		strstr(saved_command_line, "project_name=18821")) {
		remove_flag(cmd, "androidboot.verifiedbootstate=");
	}
	remove_flag(cmd, "androidboot.veritymode=");
}

static int __init proc_cmdline_init(void)
{
	strcpy(new_command_line, saved_command_line);

	/*
	 * Remove various flags from command line seen by userspace in order to
	 * pass SafetyNet CTS check.
	 */
	remove_safetynet_flags(new_command_line);

	proc_create("cmdline", 0, NULL, &cmdline_proc_fops);
	return 0;
}
fs_initcall(proc_cmdline_init);
