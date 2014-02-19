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
#include <linux/spi/spi.h>


#define CC1101_MNAME    "cc1101m"
#define SPI_BUS_MASTER  0
#define SPI_CHIP_SELECT 0


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


static int cc1101_remove(struct spi_device * spi);
static int cc1101_probe(struct spi_device * spi);


static struct spi_board_info cc1101_hotplug_spi_device = {
    .modalias = CC1101_MNAME,
    .max_speed_hz = 50000,
    .bus_num = SPI_BUS_MASTER,
    .chip_select = SPI_CHIP_SELECT,
    .mode = SPI_MODE_0
};


static struct spi_device * cc1101_spi_device;


enum cc1101_chip_id {
    CC1101_CHIP_ID
};


static const struct spi_device_id cc1101_id[] = {
    {CC1101_MNAME, CC1101_CHIP_ID},
    {}
};
MODULE_DEVICE_TABLE(spi, cc1101_id);


static struct spi_driver cc1101_driver = {
    .driver = {
        .name = CC1101_MNAME,
        .owner = THIS_MODULE,
    },
    .probe = cc1101_probe,
    .remove = cc1101_remove,
    .id_table = cc1101_id
};


static irqreturn_t gdo0_interrupt_handler(int i, void * data)
{
    int value = gpio_get_value(GDO0_pin);
    printk(KERN_INFO "GDO0 interrupt handler %d %d, value = %d\n", i, (int) data, value);
    return IRQ_HANDLED; // IRQ_NONE;
}


static irqreturn_t gdo2_interrupt_handler(int i, void * data)
{
    int value = gpio_get_value(GDO2_pin);
    printk(KERN_INFO "GDO2 interrupt handler %d %d, value = %d\n", i, (int) data, value);
    return IRQ_HANDLED; // IRQ_NONE;
}


int cc1101_probe(struct spi_device * spi)
{
	printk(KERN_INFO "CC1101 driver probe\n");
    
    // setup GDO0
    if (!gpio_is_valid(GDO0_pin)) {
        printk(KERN_ERR "CC1101 GDO0 pin %d is invalid\n", GDO0_pin);
        goto cc1101_probe_fail;
    }

    gdo0_res = gpio_request_one(GDO0_pin, GPIOF_DIR_IN, GDO0_name);
    if (gdo0_res != 0) {
        printk(KERN_ERR "CC1101 could not acquire GDO0. Reason: %d\n", gdo0_res);
        goto cc1101_probe_fail;
    }
    
    gdo0_irq_num = gpio_to_irq(GDO0_pin);
    if (gdo0_irq_num < 0) {
        printk(KERN_ERR "CC1101 driver cannot allocate IRQ for GDO0. Reason: %d\n", gdo0_irq_num);
        goto cc1101_probe_free_gdo0;
    }

    gdo0_irq = request_irq(gdo0_irq_num, gdo0_interrupt_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, GDO0_IRQ_NAME, spi);
    if (gdo0_irq != 0) {
        printk(KERN_ERR "CC1101 GDO0 failed to request irq. Reason: %d\n", gdo0_irq);
        goto cc1101_probe_free_gdo0;
    }

    // setup GDO2
    if (!gpio_is_valid(GDO2_pin)) {
        printk(KERN_ERR "CC1101 GDO2 pin %d is invalid\n", GDO2_pin);
        goto cc1101_probe_free_gdo0_irq;
    }

    gdo2_res = gpio_request_one(GDO2_pin, GPIOF_DIR_IN, GDO2_name);
    if (gdo2_res != 0) {
        printk(KERN_ERR "CC1101 could not acquire GDO2. Reason: %d\n", gdo2_res);
        goto cc1101_probe_free_gdo0_irq;
    }

    gdo2_irq_num = gpio_to_irq(GDO2_pin);
    if (gdo2_irq_num < 0) {
        printk(KERN_ERR "CC1101 driver cannot allocate IRQ for GDO2. Reason: %d\n", gdo2_irq_num);
        goto cc1101_probe_free_gdo2;
    }

    gdo2_irq = request_irq(gdo2_irq_num, gdo2_interrupt_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, GDO2_IRQ_NAME, spi);
    if (gdo2_irq != 0) {
        printk(KERN_ERR "CC1101 GDO2 failed to request irq. Reason: %d\n", gdo2_irq);
        goto cc1101_probe_free_gdo0;
    }

    // TODO: initialize CC1101 IC using SPI interface

    return 0;

//cc1101_probe_free_gdo2_irq:
//    free_irq(gdo2_irq_num, spi);

cc1101_probe_free_gdo2:
    gpio_free(GDO2_pin);

cc1101_probe_free_gdo0_irq:
    free_irq(gdo0_irq_num, spi);

cc1101_probe_free_gdo0:
    gpio_free(GDO0_pin);

cc1101_probe_fail:
    return -EINVAL;
}


int cc1101_remove(struct spi_device * spi)
{
	printk(KERN_INFO "CC1101 module de-initialization\n");

    free_irq(gdo0_irq_num, spi);
    free_irq(gdo2_irq_num, spi);

    gpio_free(GDO0_pin);
    gpio_free(GDO2_pin);

    return 0;
}


static int __init cc1101_module_init(void)
{
    int status;
    struct spi_master *master;

    printk(KERN_INFO "CC1101M module initialization in progress\n");

    master = spi_busnum_to_master(SPI_BUS_MASTER);
    if (!master) {
        printk(KERN_ERR "Could not find master spi driver on bus %d\n", SPI_BUS_MASTER);
        return -EINVAL;
    }

    cc1101_spi_device = spi_new_device(master, &cc1101_hotplug_spi_device);
    if (!cc1101_spi_device) {
        printk(KERN_ERR "Could not add hotplug SPI device\n");
        return -EINVAL;
    }

    printk(KERN_INFO "Module CC1101M added hotplug spi device. About to register cc1101m spi driver\n");

    status = spi_register_driver(&cc1101_driver);
    if (status < 0) {
        printk(KERN_ERR "Failed to register cc1101m spi driver. Reason: %d\n", status);
        spi_unregister_device(cc1101_spi_device);
        cc1101_spi_device = NULL;
        return -EINVAL;
    }

    printk(KERN_INFO "CC1101 spi driver has been registered. Code: %d\n", status);

    return 0;
}


static void __exit cc1101_module_exit(void)
{
    printk(KERN_INFO "About to deinitialize CC1101M module\n");

    spi_unregister_driver(&cc1101_driver);
    spi_unregister_device(cc1101_spi_device);
    cc1101_spi_device = NULL;

    printk(KERN_INFO "CC1101M module has been deinitialized\n");
}


module_init( cc1101_module_init );
module_exit( cc1101_module_exit );


MODULE_DESCRIPTION("CC1101 support module");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zolkko");

