/**
 * @file		sys_call_hook
 * @brief		Linux Kernel Module Template
 *
 * @author		yhnu
 * @copyright	Copyright (c) 2016-2021 T. yhnu
 *
 * @par License
 *	Released under the MIT and GPL Licenses.
 *	- https://github.com/ngtkt0909/linux-kernel-module-template/blob/master/LICENSE-MIT
 *	- https://github.com/ngtkt0909/linux-kernel-module-template/blob/master/LICENSE-GPL
 */
#include <asm/page-def.h>  // PAGE_SHIFT
#include <asm/uaccess.h>   /* copy_from_user(), copy_to_user() */
#include <linux/cdev.h>    /* cdev, dev_init(), cdev_add(), cdev_del() */
#include <linux/device.h>  /* class_create(), class_destroy(), device_create(), device_destroy() */
#include <linux/err.h>
#include <linux/fs.h> /* file_operations, alloc_chrdev_region, unregister_chrdev_region */
#include <linux/hashtable.h>
#include <linux/mm.h>
#include <linux/module.h> /* MODULE_*, module_* */
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/slab.h> /* kmalloc(), kfree() */
#include <linux/syscalls.h>
#include <linux/types.h>   // u32 etc.
#include <linux/uaccess.h> /* copy_from_user(), copy_to_user() */
#include <linux/dirent.h>  /* dirp */
/*------------------------------------------------------------------------------
    Define Declaration
------------------------------------------------------------------------------*/

#ifndef MAX_PATH
#define MAX_PATH 256
#endif
#define MAX_STACK_TRACE_DEPTH 64

static int myuid = 10229;
module_param(myuid, int, S_IRUGO);


typedef void (*TYPE_update_mapping_prot)(phys_addr_t phys, unsigned long virt, phys_addr_t size, pgprot_t prot);
TYPE_update_mapping_prot update_mapping_prot;

static unsigned long start_rodata;
static unsigned long end_rodata;
static unsigned long init_begin;
static void **sys_call_table_ptr;
#define section_size (init_begin - start_rodata)

static void disable_wirte_protection(void) {
    update_mapping_prot(__pa_symbol(start_rodata), (unsigned long)start_rodata, section_size, PAGE_KERNEL);
    return;
}

static void enable_wirte_protection(void) {
    update_mapping_prot(__pa_symbol(start_rodata), (unsigned long)start_rodata, section_size, PAGE_KERNEL_RO);
    return;
}

bool inline isUserPid(void) {
    const struct cred *m_cred = current_cred();
    if (m_cred->uid.val > 10000) {
        return true;
    }
    return false;
}

#include "kr_common.h"
//#include "kr_netbind.h"
//#include "kr_netsend.h"
#include "kr_kill.h"
#include "kr_openat.h"
#include "kr_dir.h"

int syscall_hook_init(void) {
    printk(KERN_ALERT "defined Macro USE_IMMEDIATE\n");
    printk(KERN_ALERT "hello world!\n");
    start_rodata = (unsigned long)kallsyms_lookup_name("__start_rodata");
    init_begin   = (unsigned long)kallsyms_lookup_name("__init_begin");
    end_rodata   = (unsigned long)kallsyms_lookup_name("__end_rodata");

    update_mapping_prot = (TYPE_update_mapping_prot)kallsyms_lookup_name("update_mapping_prot");

    sys_call_table_ptr = (void **)kallsyms_lookup_name("sys_call_table");
    printk("sys_call_table=%lx. update_mapping_prot:%lx, start_rodata:%lx, end_rodata:%lx init_begin:%lx.\n", sys_call_table_ptr, update_mapping_prot, start_rodata, end_rodata, init_begin);

    preempt_disable();
    disable_wirte_protection();
    
    // KR_NETBIND_INIT();
    // KR_NETSEND_INIT();
    KR_OPENAT_INIT();
    KR_KILL_INIT();
    KR_GETDENTS_INIT();

    enable_wirte_protection();
    preempt_enable();
    return 0;
}

void syscall_hook_exit(void) {
    preempt_disable();
    disable_wirte_protection();

    // KR_NETBIND_CLEAN();
    // KR_NETSEND_CLEAN();
    KR_OPENAT_CLEAN();
    KR_KILL_CLEAN();
    KR_GETDENTS_CLEAN();

    enable_wirte_protection();
    preempt_enable();
    printk(KERN_ALERT "I am back.kernel in planet Linux!\n");
}

module_init(syscall_hook_init);
module_exit(syscall_hook_exit);
MODULE_AUTHOR("yhnu");
MODULE_LICENSE("Dual BSD/GPL");
// https://tldp.org/LDP/lkmpg/2.6/html/x351.html
// https://bbs.pediy.com/thread-267004.htm
