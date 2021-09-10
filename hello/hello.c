#include <linux/module.h>	/* Needed by all modules */
#include <linux/kernel.h>	/* Needed for KERN_INFO */
#include <linux/init.h>		/* Needed for the macros */

#define USE_IMMEDIATE

//MODULE_LICENSE("Dual BSD/GPL");

static int hello_init(void)
{
	printk(KERN_ALERT "defined Macro USE_IMMEDIATE\n");
	printk(KERN_ALERT "hello world!\n");
	return 0;
}

static void hello_exit(void)
{
	printk(KERN_ALERT "I am back.kernel in planet Linux!\n");
}


module_exit(hello_exit);
module_init(hello_init);