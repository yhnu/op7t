/* Copyright (c) 2016 OnePlus. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/mm.h>
#include <asm/pgtable.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/of_address.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

struct bootloader_log_platform_data {
	unsigned long	mem_size;
	unsigned long	mem_address;
	unsigned int	mem_type;

};

struct bootloader_log_ram_zone_t {
	phys_addr_t paddr;
	size_t size;
	void *vaddr;
	char *buffer;

} bootloader_log_ram_zone;

static int bootloader_log_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", bootloader_log_ram_zone.buffer);
	return 0;
}

static int bootloader_log_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bootloader_log_proc_show, NULL);
}

static const struct file_operations bootloader_log_proc_fops = {
	.open		= bootloader_log_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int proc_bootloader_log_init(void)
{
	proc_create("bootloader_log", 0, NULL, &bootloader_log_proc_fops);
	return 0;
}


static int __init of_bootloader_log_platform_data(struct device_node *node,
	struct bootloader_log_platform_data *pdata)
{
	const u32 *addr;
	u64 size;
	struct device_node *pnode;

	memset(pdata, 0, sizeof(*pdata));

	pnode = of_parse_phandle(node, "linux,contiguous-region", 0);
	if (pnode) {
		addr = of_get_address(pnode, 0, &size, NULL);
		if (!addr) {
			pr_err("failed to parse the bootloader log memory address\n");
			of_node_put(pnode);
			return -EINVAL;
		}
		pdata->mem_address = of_read_ulong(addr, 2);
		pdata->mem_size = (unsigned long) size;
		of_node_put(pnode);
	} else {
		pr_err("mem reservation for bootloader log not present\n");
		return -EINVAL;
	}

	return 0;
}

static void *persistent_ram_vmap(phys_addr_t start, size_t size)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);

	prot = pgprot_noncached(PAGE_KERNEL);


	pages = kmalloc_array(page_count, sizeof(struct page *), GFP_KERNEL);
	if (!pages)
		return NULL;

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;

		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vmap(pages, page_count, VM_MAP, prot);
	kfree(pages);

	return vaddr;
}


static int persistent_ram_buffer_map(phys_addr_t start, size_t size,
		struct bootloader_log_ram_zone_t *blrz)
{
	blrz->paddr = start;
	blrz->size = size;

	blrz->vaddr = persistent_ram_vmap(start, size);

	if (!blrz->vaddr) {
		pr_err("%s: Failed to map 0x%llx pages at 0x%llx\n", __func__,
			(unsigned long long)size, (unsigned long long)start);
		return -ENOMEM;
	}

	blrz->buffer = blrz->vaddr + offset_in_page(start);
	return 0;
}


static int bootloader_log_probe(struct platform_device *pdev)
{
	struct bootloader_log_platform_data *pdata = pdev->dev.platform_data;
	struct bootloader_log_platform_data of_pdata;

	phys_addr_t paddr;
	int err = -EINVAL;

	pr_err("bootloader_log_probe\n");

	if (pdev->dev.of_node) {
		if (of_bootloader_log_platform_data(pdev->dev.of_node,
			&of_pdata)) {
			pr_err("Invalid bootloader log device tree data\n");
			goto fail_out;
		}
		pdata = &of_pdata;
	}

	if (!pdata->mem_size) {
		pr_err("memory size and record size must be non-zero\n");
		goto fail_out;
	}

	paddr = pdata->mem_address;


	err = persistent_ram_buffer_map(paddr, pdata->mem_size,
		&bootloader_log_ram_zone);
	if (err)
		goto fail_out;

	proc_bootloader_log_init();


	pr_info("bootloader_log!\n");

	return 0;

fail_out:
	pr_err("bootloader_log, fail_out!\n");
	return err;
}

static int __exit bootloader_log_remove(struct platform_device *pdev)
{
	return -EBUSY;
}

static const struct of_device_id bootloader_log_of_match[] = {
	{ .compatible = "bootloader_log", },
	{ },
};
MODULE_DEVICE_TABLE(of, bootloader_log_of_match);

static struct platform_driver bootloader_log_driver = {
	.probe		= bootloader_log_probe,
	.remove		= __exit_p(bootloader_log_remove),
	.driver		= {
		.name	= "bootloader_log",
		.owner	= THIS_MODULE,
		.of_match_table = bootloader_log_of_match,
	},
};


static int __init bootloader_log_init(void)
{
	return platform_driver_register(&bootloader_log_driver);
}
postcore_initcall(bootloader_log_init);

static void __exit bootloader_log_exit(void)
{
	platform_driver_unregister(&bootloader_log_driver);

}
module_exit(bootloader_log_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("software@oneplus.cn>");

