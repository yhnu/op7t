#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <asm/unistd.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>

unsigned long cr0;

/*
static inline void protect_memory(void)
{
    //write_cr0(cr0);
}

static inline void unprotect_memory(void)
{
    //write_cr0(cr0 & ~0x00010000);
}
*/


/*
* This is not a whole code, but only a snippet. 
* Some functions *is* missing.
*/

asmlinkage long (*orig_shutdown)(int, int);
unsigned long *sys_call_table;
void* hook_addr = 0x00;

hooking_syscall(void *hook_addr, uint16_t syscall_offset, unsigned long *sys_call_tabe)
{
	//unprotect_memory();
	sys_call_table[syscall_offset] = (unsigned long)hook_addr;
	//protect_memory();
}

unhooking_syscall(void *orig_addr, uint16_t syscall_offset)
{
	//unprotect_memory();
	sys_call_table[syscall_offset] = (unsigned long)hook_addr;
	//protect_memory();
}

asmlinkage int hooked_shutdown(int magic1, int magic2)
{
	printk("Hello from hook!");
	return orig_shutdown(magic1, magic2);
}

static int __init module_init(void)
{
	unsigned long *sys_call_table = kallsyms_lookup_name("sys_call_table"));
	//orig_shutdown = (void*)sys_call_table[__NR_shutdown];
	//hooking_syscall(hooked_shutdown, __NR_shutdown, sys_call_tabe);
}

static void __exit module_cleanup(void)
{
	//unhooking_syscall(orig_shutdown, __NE_shutdown, sys_call_table);
}

module_exit(module_cleanup);
module_init(module_init);