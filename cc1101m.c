#undef  __KERNEL__
#define __KERNEL__

#undef  MODULE
#define MODULE


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>


static int __init cc1101_module_init(void);
static void __exit cc1101_module_exit(void);


#define GDO0_pin 22
#define GDO0_name "GDO0"


#define GDO2_pin 23
#define GDO2_name "GDO2"


static int gdo0_res = -EINVAL;
static int gdo0_irq = -EINVAL;
static int gdo0_irq_num = -EINVAL;
#define GDO0_IRQ_NAME "GDO0 interrupt handler"

static int gdo2_res = -EINVAL;
static int gdo2_irq = -EINVAL;
static int gdo2_irq_num = -EINVAL;
#define GDO2_IRQ_NAME "GDO2 interrupt handler"


static irqreturn_t gdo0_interrupt_handler(int i, void * p)
{
    int value = gpio_get_value(GDO0_pin);
    printk(KERN_INFO "GDO0 interrupt handler %d %d, value = %d\n", i, (int) p, value);
    return IRQ_HANDLED; // IRQ_NONE;
}


static irqreturn_t gdo2_interrupt_handler(int i, void * p)
{
    int value = gpio_get_value(GDO2_pin);
    printk(KERN_INFO "GDO2 interrupt handler %d %d, value = %d\n", i, (int) p, value);
    return IRQ_HANDLED; // IRQ_NONE;
}


int __init cc1101_module_init(void)
{
	printk(KERN_INFO "CC1101 module initialization\n");

    if (gpio_is_valid(GDO0_pin)) {
        gdo0_res = gpio_request_one(GDO0_pin, GPIOF_DIR_IN, GDO0_name);
        if (gdo0_res == 0) {
            printk(KERN_INFO "GDO0 pin %d allocated\n", GDO0_pin);
            gdo0_irq_num = gpio_to_irq(GDO0_pin);
            if (gdo0_irq_num >= 0) {
                printk(KERN_INFO "GDO0 irq %d\n", gdo0_irq_num);
                gdo0_irq = request_irq(gdo0_irq_num, gdo0_interrupt_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, GDO0_IRQ_NAME, NULL); 
                if (gdo0_irq == 0) {
                    printk(KERN_INFO "GDO0 interrupt request eq %d\n", gdo0_irq);
                } else {
                    printk(KERN_INFO "GDO0 failed to request irq. Reason: %d\n", gdo0_irq);
                }
            } else {
                printk(KERN_ERR "Cannot allocate IRQ for GDO0. Reason: %d\n", gdo0_irq_num);
            }
        } else {
            printk(KERN_NOTICE "Failed to acquire GDO0. Reason: %d\n", gdo0_res);
        }
    } else {
        printk(KERN_ERR "GDO0 pin %d is invalid\n", GDO0_pin);
    }

    if (gpio_is_valid(GDO2_pin)) {
        gdo2_res = gpio_request_one(GDO2_pin, GPIOF_DIR_IN, GDO2_name);
        if (gdo2_res == 0) {
            printk(KERN_INFO "GDO2 pin %d allocated\n", GDO2_pin);
            if ((gdo2_irq_num = gpio_to_irq(GDO2_pin)) >= 0) {
                printk(KERN_INFO "GDO2 irq %d\n", gdo2_irq_num);
                gdo2_irq = request_irq(gdo2_irq_num, gdo2_interrupt_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, GDO2_IRQ_NAME, NULL); 
                if (gdo2_irq == 0) {
                    printk(KERN_INFO "GDO0 interrupt request eq %d\n", gdo2_irq);
                } else {
                    printk(KERN_INFO "GDO0 failed to request irq. Reason: %d\n", gdo2_irq);
                }
            } else {
                printk(KERN_ERR "Cannot allocate IRQ for GDO2. Reason: %d\n", gdo2_irq_num);
            }
        } else {
            printk(KERN_NOTICE "Failed to acquire GDO2. Reason: %d\n", gdo2_res);
        }
    } else {
        printk(KERN_ERR "GDO2 pin %d is invalid\n", GDO2_pin);
    }

	return 0;
}


void __exit cc1101_module_exit(void)
{
	printk(KERN_INFO "CC1101 module de initialization\n");

    if (gdo0_irq_num >= 0 && gdo0_irq == 0) {
        printk(KERN_INFO "Releasing GDO0 irq %d\n", gdo0_irq_num);
        free_irq(gdo0_irq_num, NULL);
    }

    if (gdo2_irq_num >= 0 && gdo2_irq == 0) {
        printk(KERN_INFO "Releasing GDO2 irq %d\n", gdo2_irq_num);
        free_irq(gdo2_irq_num, NULL);
    }

    if (gdo0_res == 0) {
        printk(KERN_INFO "Deallocate GDO0 pin\n");
        gpio_free(GDO0_pin);
        gdo0_res = -1;
    }

    if (gdo2_res == 0) {
        printk(KERN_INFO "Deallocate GDO2 pin\n");
        gpio_free(GDO2_pin);
        gdo2_res = -1;
    }
}


module_init( cc1101_module_init );
module_exit( cc1101_module_exit );


MODULE_DESCRIPTION("CC1101 support module");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zolkko");

