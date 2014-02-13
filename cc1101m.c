#undef  __KERNEL__
#define __KERNEL__

#undef  MODULE
#define MODULE


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>


static int __init cc1101_module_init(void);
static void __exit cc1101_module_exit(void);


int __init cc1101_module_init(void)
{
	printk(KERN_INFO "CC1101 module initialization\n");
	return 0;
}


void __exit cc1101_module_exit(void)
{
	printk(KERN_INFO "CC1101 module de initialization\n");
}


module_init( cc1101_module_init );
module_exit( cc1101_module_exit );

