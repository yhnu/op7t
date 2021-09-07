/*
 * drivers/param_read_write/param_read_write.c
 *
 * hefaxi@filesystems,2015/04/30
 *
 * This program is used to read/write param partition in kernel
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/ctype.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/oem/param_rw.h>


#define PARAM_PARTITION "/dev/block/bootdevice/by-name/param"
#define READ_CHUNK_MAX_SIZE (1024)
#define WRITE_CHUNK_MAX_SIZE (1024)

typedef struct{
	phys_addr_t paddr;
	size_t size;
	void *vaddr;
	void *buffer;
	struct mutex mutex;
}param_ram_zone_t;

static DEFINE_MUTEX(param_lock);
static bool param_init_done = 0;
static param_ram_zone_t param_ram_zone;

static int write_param_partition(const char *buf, unsigned long count,
            loff_t offset)
{
	struct file *filp;
	mm_segment_t fs;
	int ret = 0;

	filp = filp_open(PARAM_PARTITION,O_RDWR|O_SYNC,0);
	if(IS_ERR(filp)) {
	    ret = PTR_ERR(filp);
		pr_err("open file %s failed.(%d)\n",PARAM_PARTITION,ret);
		return ret;
	}

	fs = get_fs();
	set_fs(get_ds());

	ret = filp->f_op->llseek(filp, offset, SEEK_SET);
	if(ret < 0){
		pr_err("%s: llseek failed.(%d)\n",__func__,ret);
		goto out;
	}
	//ret = filp->f_op->write(filp,(char __user *)buf,count,&filp->f_pos);
	ret = vfs_write(filp, (char __user *)buf, count, &filp->f_pos);

out:
	set_fs(fs);
	filp_close(filp,NULL);
	return ret;
}

int get_param_by_index_and_offset(uint32 sid_index,
            uint32 offset, void * buf, int length)
{
    int ret = length;
    uint32 file_offset;
	mutex_lock(&param_ram_zone.mutex);
	pr_info("%s[%d]  sid_index = %d offset = %d buf = %p length = %d\n",
			__func__, __LINE__,sid_index,offset,buf,length);

    file_offset = PARAM_SID_LENGTH*sid_index+ offset;

	if (buf && ((offset + length) <= PARAM_SID_LENGTH) &&
			(file_offset + length) <=  param_ram_zone.size)
		memcpy(buf,(param_ram_zone.buffer +file_offset), length);
	else{
		pr_info("%s:invaild argument, sid_index=%d offset=%d buf=%p length=%d\n",
				__func__, sid_index, offset, buf, length);
		ret = -EINVAL;
	}

	mutex_unlock(&param_ram_zone.mutex);
	return ret;
}
EXPORT_SYMBOL(get_param_by_index_and_offset);

int set_param_by_index_and_offset(uint32 sid_index,
        uint32 offset, void * buf, int length)
{
    int ret;
    uint32 file_offset;
	mutex_lock(&param_ram_zone.mutex);
	pr_info("%s[%d]sid_index = %d offset = %d buf = %p length = %d\n",
			__func__, __LINE__,sid_index,offset,buf,length);

    file_offset = PARAM_SID_LENGTH*sid_index + offset;

	if (buf && ((offset + length) <= PARAM_SID_LENGTH) &&
			(file_offset + length) <=  param_ram_zone.size)
		memcpy((param_ram_zone.buffer+file_offset),buf,length);
	else{
		pr_info("%s:invaild argument,sid_index=%d offset=%d buf=%p length=%d\n",
		      __func__,sid_index,offset,buf,length);
		ret = -EINVAL;
		goto out;
	}

    ret = write_param_partition((param_ram_zone.buffer+file_offset),
                    length,file_offset);
	if ( ret < 0){
		pr_info("Error write param partition.(%d)\n",ret);
	}
out:
	mutex_unlock(&param_ram_zone.mutex);
	return ret;
}
EXPORT_SYMBOL(set_param_by_index_and_offset);

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

	/* prot = pgprot_noncached(PAGE_KERNEL); */
	prot = pgprot_writecombine(PAGE_KERNEL);

	pages = kmalloc(sizeof(struct page *) * page_count, GFP_KERNEL);
	if (!pages) {
		pr_err("%s: Failed to allocate array for %u pages\n", __func__,
				page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;
		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vmap(pages, page_count, VM_MAP, prot);
	kfree(pages);
	return vaddr;
}

static int param_ram_buffer_map(phys_addr_t start, phys_addr_t size,
		param_ram_zone_t *prz)
{
	prz->paddr = start;
	prz->size = size;
	prz->vaddr = persistent_ram_vmap(start, size);

	if (!prz->vaddr) {
		pr_err("%s: Failed to map 0x%llx pages at 0x%llx\n", __func__,
				(unsigned long long)size, (unsigned long long)start);
		return -ENOMEM;
	}

	prz->buffer = prz->vaddr + offset_in_page(start);
	return 0;
}

static ssize_t param_read(struct file *file, char __user *buff,
			size_t count, loff_t *pos)
{
	void * temp_buffer;
	int chunk_sz;
	int copied;
	int left;
	int ret;

	if (mutex_lock_interruptible(&param_lock))
		return -ERESTARTSYS;

	chunk_sz = count < READ_CHUNK_MAX_SIZE ? count : READ_CHUNK_MAX_SIZE;
	temp_buffer = kzalloc(chunk_sz, GFP_KERNEL);

	if (temp_buffer == NULL)
		return -ENOMEM;

	left = count;
	copied = 0;

	while (left) {
		chunk_sz = (left <= READ_CHUNK_MAX_SIZE) ?
			left : READ_CHUNK_MAX_SIZE;
		ret = get_param_by_index_and_offset(*pos/PARAM_SID_LENGTH,
				*pos%PARAM_SID_LENGTH, temp_buffer, chunk_sz);

		if (ret < 0) {
			pr_err("get_param_by_index_and_offset fail %d\n", ret);
			goto out;
		}

		if (copy_to_user(buff + copied, temp_buffer, chunk_sz)) {
			ret =  -EFAULT;
			pr_info("copy_to_user failure\n");
			goto out;
		}

		*pos += chunk_sz;
		left -= chunk_sz;
		copied += chunk_sz;
	}

out:
	kfree(temp_buffer);
	mutex_unlock(&param_lock);
	return copied;
}

static ssize_t param_write(struct file *file, const char __user *buff,
		size_t count, loff_t *pos)
{

	void * temp_buffer;
	int chunk_sz;
	int written;
	int left;
	int ret;
	if (mutex_lock_interruptible(&param_lock))
		return -ERESTARTSYS;

	chunk_sz = count < WRITE_CHUNK_MAX_SIZE ? count : WRITE_CHUNK_MAX_SIZE;
	temp_buffer = kzalloc(chunk_sz, GFP_KERNEL);

	if (temp_buffer == NULL)
		return -ENOMEM;

	left = count;
	written = 0;

	while (left > 0) {
		chunk_sz = (left <= WRITE_CHUNK_MAX_SIZE) ?
					left : WRITE_CHUNK_MAX_SIZE;
		ret = copy_from_user(temp_buffer, buff + written, chunk_sz);
		if (ret < 0) {
			pr_info("copy_from_user failure %d\n", ret);
			goto out;
		}

		ret = set_param_by_index_and_offset(*pos / PARAM_SID_LENGTH,
				*pos % PARAM_SID_LENGTH, temp_buffer, chunk_sz);

		if (ret < 0) {
			pr_err("set_param_by_index_and_offset failure %d\n",
					ret);
			goto out;
		}

		*pos += chunk_sz;
		left -= chunk_sz;
		written += chunk_sz;
	}
out:
	kfree(temp_buffer);
	mutex_unlock(&param_lock);
	return written;
}

static const struct file_operations param_fops = {
	.owner          = THIS_MODULE,
	.read           = param_read,
	.write          = param_write,
	.llseek		= default_llseek,
};

struct miscdevice param_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "param",
	.fops = &param_fops,
};

static int __init param_core_init(void)
{

	if(param_ram_buffer_map((phys_addr_t)param_ram_zone.paddr,
	        param_ram_zone.size, (param_ram_zone_t *)&param_ram_zone)){
		pr_err("param_ram_buffer_map failred\n");
		return -1;
	}
	mutex_init(&param_ram_zone.mutex);

	param_init_done= 1;
	return 0;
}
pure_initcall(param_core_init);

static int __init param_device_init(void)
{
    int ret;
    ret = misc_register(&param_misc);
    if(ret){
        pr_err("misc_register failure %d\n",ret);
        return -1;
    }
    return ret;
}
device_initcall(param_device_init);

void init_param_mem_base_size(phys_addr_t base, unsigned long size)
{
	param_ram_zone.paddr = base;
	param_ram_zone.size = size;
}
EXPORT_SYMBOL(init_param_mem_base_size);

/*
*Add more function here
*
*/

int restart_08_count;
int add_restart_08_count(void)
{
	int ret;

	ret = get_param_by_index_and_offset(9, 0x15c,
		&restart_08_count, sizeof(restart_08_count));

	restart_08_count = restart_08_count + 1;

	ret = set_param_by_index_and_offset(9, 0x15c,
		&restart_08_count, sizeof(restart_08_count));

	if (ret < 0)
		pr_info("%s[%d]  failed!\n", __func__, __LINE__);

	return ret;
}
EXPORT_SYMBOL(add_restart_08_count);

static int param_get_restart_08_count(char *val, const struct kernel_param *kp)
{

	int cnt = 0;
	int ret;

	ret = get_param_by_index_and_offset(9, 0x15c,
		&restart_08_count, sizeof(restart_08_count));

	if (ret < 0)
		pr_info("%s[%d]  failed!\n", __func__, __LINE__);

	cnt = snprintf(val, 4, "%d", restart_08_count);

	return cnt;
}
module_param_call(restart_08_count, NULL, param_get_restart_08_count, &restart_08_count, 0644);

int restart_other_count=0;
int add_restart_other_count(void)
{
	int ret;

	ret = get_param_by_index_and_offset(9, 0x160,
		&restart_other_count, sizeof(restart_other_count));

	restart_other_count = restart_other_count + 1;

	ret = set_param_by_index_and_offset(9, 0x160,
		&restart_other_count, sizeof(restart_other_count));

	if (ret < 0)
		pr_info("%s[%d]  failed!\n", __func__, __LINE__);

	return ret;
}
EXPORT_SYMBOL(add_restart_other_count);
static int param_get_restart_other_count(char *val, const struct kernel_param *kp)
{

	int cnt = 0;
	int ret;

	ret = get_param_by_index_and_offset(9, 0x160,
		&restart_other_count, sizeof(restart_other_count));

	if (ret < 0)
		pr_info("%s[%d]  failed!\n", __func__, __LINE__);

	cnt = snprintf(val, 4, "%d", restart_other_count);

	return cnt;
}
module_param_call(restart_other_count, NULL, param_get_restart_other_count, &restart_other_count, 0644);
//end
