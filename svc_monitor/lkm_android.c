#include <asm/unistd.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>

#include <linux/module.h> /* Needed by all modules */
#include <linux/kernel.h> /* Needed for KERN_INFO */
#include <linux/init.h>	  /* Needed for the macros */


MODULE_LICENSE("Dual BSD/GPL");

typedef void (* TYPE_update_mapping_prot)(phys_addr_t phys, unsigned long virt, phys_addr_t size, pgprot_t prot);
//typedef asmlinkage long (* TYPE_openat)(const struct pt_regs *pt_regs);
typedef int (* TYPE_openat)(int dirfd, const char *pathname, int flags, mode_t mode);


static unsigned long start_rodata;
static unsigned long end_rodata;
static unsigned long init_begin;
static void ** sys_call_table_ptr;
#define section_size  (init_begin - start_rodata)


TYPE_update_mapping_prot update_mapping_prot;
static TYPE_openat old_openat;

static void disable_wirte_protection(void)
{
    update_mapping_prot(__pa_symbol(start_rodata), (unsigned long)start_rodata, section_size, PAGE_KERNEL);
    return ;
}
 
static void enable_wirte_protection(void)
{
    update_mapping_prot(__pa_symbol(start_rodata), (unsigned long)start_rodata, section_size, PAGE_KERNEL_RO);
    return ;
}

/*
static atomic_t ref_count = ATOMIC_INIT(0);
asmlinkage long my_stub_openat(const struct pt_regs *pt_regs)
{
        atomic_inc(&ref_count);
        long value = -1;
        char kfilename[80] = {0};
 
        int dfd = (int)pt_regs->regs[0];
        char __user *filename = (char*)pt_regs->regs[1];
        int flags = (int)pt_regs->regs[2];
        int mode = (int)pt_regs->regs[3];
 
        value = old_openat_func(pt_regs);
 
        copy_from_user(kfilename, filename, 80);
        printk("%s. process:[%d:%s] open file:%s.\n\t-----> open flags:0x%0x, open %s, fd:%d.\n", __FUNCTION__,
           current->tgid, current->group_leader->comm, kfilename, flags, value>=0?"sucess":"fail", value);
 
openat_return:
        atomic_dec(&ref_count);
        return value;
}
*/

bool inline isUserPid(void)
{
   const struct cred * m_cred = current_cred();
   if(m_cred->uid.val > 10000)
   {
      return true;
   }
   return false;
}

int new_openat(int dirfd, const char *pathname, int flags, mode_t mode)
{
	if(isUserPid())
    {
		char bufname[256] = {0};
		int pid = get_current()->pid;
		strncpy_from_user(bufname, pathname, 255);

		printk("myLog::openat64 pathname:[%s] current->pid:[%d]\n", bufname, pid);
    }
	return old_openat(dirfd, pathname, flags, mode);
}

static int syscall_hook_init(void)
{
	printk(KERN_ALERT "defined Macro USE_IMMEDIATE\n");
	printk(KERN_ALERT "hello world!\n");
	start_rodata = (unsigned long)kallsyms_lookup_name("__start_rodata");
	init_begin = (unsigned long)kallsyms_lookup_name("__init_begin");
	end_rodata = (unsigned long)kallsyms_lookup_name("__end_rodata");
	
	update_mapping_prot = (TYPE_update_mapping_prot)kallsyms_lookup_name("update_mapping_prot");

	sys_call_table_ptr = (void **)kallsyms_lookup_name("sys_call_table");
	printk("sys_call_table=%lx. update_mapping_prot:%lx, start_rodata:%lx, end_rodata:%lx init_begin:%lx.\n", sys_call_table_ptr, update_mapping_prot, start_rodata, end_rodata, init_begin);


	preempt_disable();
	disable_wirte_protection();
	old_openat = (TYPE_openat)sys_call_table_ptr[__NR_openat];
	sys_call_table_ptr[__NR_openat] = (TYPE_openat)new_openat;
	enable_wirte_protection();
	preempt_enable();
	return 0;
}

static void cleanup(void)
{
	preempt_disable();
	disable_wirte_protection();

	if(sys_call_table_ptr[__NR_openat] == new_openat)
	{
		sys_call_table_ptr[__NR_openat] = old_openat;
	}

	enable_wirte_protection();
	preempt_enable();

	return ;
}

static void syscall_hook_exit(void)
{
	cleanup();
	printk(KERN_ALERT "I am back.kernel in planet Linux!\n");
}

module_exit(syscall_hook_exit);
module_init(syscall_hook_init);

//https://tldp.org/LDP/lkmpg/2.6/html/x351.html
//https://bbs.pediy.com/thread-267004.htm