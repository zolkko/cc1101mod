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
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/mutex.h>


// TODO: pass SPI_BUS_MASTER, SPI_CHIP_SELECT, GDO0_pin and GDO2_pin as parameters to the module


#define CC1101_MNAME    "cc1101m"
#define SPI_BUS_MASTER  0
#define SPI_CHIP_SELECT 0


#define GDO0_pin  22
#define GDO0_name "GDO0"


#define GDO2_pin  23
#define GDO2_name "GDO2"


#define GDO0_IRQ_NAME "GDO0 interrupt handler"
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


struct cc1101_data {
    struct spi_device * spi;

    spinlock_t spi_lock;

    int gdo0_pin;
    int gdo2_pin;

    int gdo0_irq;
    int gdo2_irq;

    int gdo0_val;
    int gdo2_val;

    struct workqueue_struct * queue;
    struct work_struct * reset;
};


static irqreturn_t gdo0_interrupt_handler(int i, void * irq_data)
{
    struct cc1101_data * data = (struct cc1101_data *)irq_data;
    int value = gpio_get_value(data->gdo0_pin);

    if (value != data->gdo0_val) {
        data->gdo0_val = value;
        printk(KERN_INFO "CC1101 GDO0 interrupt handler value: %d\n", value);
        return IRQ_HANDLED;
    } else {
        return IRQ_NONE;
    }
}


static irqreturn_t gdo2_interrupt_handler(int i, void * irq_data)
{
    struct cc1101_data * data = (struct cc1101_data *)irq_data;
    int value = gpio_get_value(data->gdo2_pin);

    if (value != data->gdo2_val) {
        data->gdo2_val = value;
        printk(KERN_INFO "CC1101 GDO2 interrupt handler value: %d\n", value);
        return IRQ_HANDLED;
    } else {
        return IRQ_NONE;
    }
}


static void cc1101_reset_configure(void * dev_data)
{
    struct cc1101_data * data = (struct cc1101_data *)dev_data;

    printk(KERN_INFO "CC1101 module resets chipcon\n");
    // TODO: lock SPI module
    // TODO: reset CC1101
    // TODO: unlock SPI module
    printk(KERN_INFO "CC1101 successfully reset chipcon\n");
}


int cc1101_probe(struct spi_device * spi)
{
    int status;
    int irq_num;
    int result;
    unsigned long flags;
    struct cc1101_data * spi_data;

    result = 0;

    printk(KERN_INFO "CC1101 driver probe\n");

    // setup data
    spi_data = kzalloc(sizeof(*spi_data), GFP_KERNEL);
    if (IS_ERR(spi_data)) {
        printk(KERN_ERR "CC1101 module could not allocate memory\n");
        result = -ENOMEM;
        goto cc1101_probe_exit;
    }
    spi_data->spi = spi;
    spi_data->gdo0_pin = GDO0_pin;
    spi_data->gdo2_pin = GDO2_pin;

    spin_lock_init(spi_data->spi_lock);
    spin_lock_irqsave(&(spi_data->spi_lock), flags);

    // create work queue
    spi_data->queue = create_singlethread_workqueue("CC1101-queue");
    if (IS_ERR(spi_data->queue)) {
        printk(KERN_ERR "CC1101 module could not allocate memory for queue\n");
	result = -ENOMEM;
	goto cc1101_probe_free_spi_lock;
    }

    // setup reset work
    spi_data->reset = kzalloc(sizeof(*(spi_data->reset)), GFP_KERNEL);
    if (IS_ERR(spi_data->reset)) {
        printk(KERN_ERR "CC1101 module could not allocate memory for reset work\n");
	goto cc1101_probe_free_queue;
    }

    // setup GDO0
    if (!gpio_is_valid(spi_data->gdo0_pin)) {
        printk(KERN_ERR "CC1101 GDO0 pin %d is invalid\n", spi_data->gdo0_pin);
        result = -EINVAL;
        goto cc1101_probe_free_reset_work;
    }

    status = gpio_request_one(spi_data->gdo0_pin, GPIOF_DIR_IN, GDO0_name);
    if (!status) {
        printk(KERN_ERR "CC1101 could not acquire GDO0. Reason: %d\n", status);
        result = -EBUSY;
        goto cc1101_probe_free_reset_work;
    }

    spi_data->gdo0_val = gpio_get_value(spi_data->gdo0_pin);

    irq_num = gpio_to_irq(spi_data->gdo0_pin);
    if (irq_num < 0) {
        printk(KERN_ERR "CC1101 driver could not allocate IRQ for GDO0. Reason: %d\n", irq_num);
        result = -EINVAL;
        goto cc1101_probe_free_gdo0;
    }
    spi_data->gdo0_irq = irq_num;

    status = request_irq(spi_data->gdo0_irq, gdo0_interrupt_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, GDO0_IRQ_NAME, spi_data);
    if (status != 0) {
        printk(KERN_ERR "CC1101 GDO0 failed to request irq. Reason: %d\n", status);
        result = -EBUSY;
        goto cc1101_probe_free_gdo0;
    }

    // setup GDO2
    if (!gpio_is_valid(spi_data->gdo2_pin)) {
        printk(KERN_ERR "CC1101 GDO2 pin %d is invalid\n", spi_data->gdo2_pin);
        result = -EINVAL;
        goto cc1101_probe_free_gdo0_irq;
    }

    status = gpio_request_one(spi_data->gdo2_pin, GPIOF_DIR_IN, GDO2_name);
    if (status != 0) {
        printk(KERN_ERR "CC1101 could not acquire GDO2. Reason: %d\n", status);
        result = -EBUSY;
        goto cc1101_probe_free_gdo0_irq;
    }

    spi_data->gdo2_val = gpio_get_value(spi_data->gdo2_pin);

    irq_num = gpio_to_irq(spi_data->gdo2_pin);
    if (irq_num < 0) {
        printk(KERN_ERR "CC1101 driver could not allocate IRQ for GDO2. Reason: %d\n", irq_num);
        result = -EINVAL;
        goto cc1101_probe_free_gdo2;
    }
    spi_data->gdo2_irq = irq_num;

    status = request_irq(irq_num, gdo2_interrupt_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, GDO2_IRQ_NAME, spi_data);
    if (status != 0) {
        printk(KERN_ERR "CC1101 GDO2 failed to request irq. Reason: %d\n", status);
        result = -EBUSY;
        goto cc1101_probe_free_gdo2;
    }

    // setup the rest of the struct
    spi_set_drvdata(spi, spi_data);

    status = queue_work(spi_data->queue, spi_data->reset);
    if (!status) {
        printk(KERN_ERR "CC1101 could not schedule reset work\n");
	result = -EINVAL;
	goto cc1101_probe_free_gdo2_irq;
    }

    spin_unlock_irqsave(spi_data->spi_lock, flags);

    return 0;

cc1101_probe_free_gdo2_irq:
    free_irq(spi_data->gdo2_irq, spi_data);

cc1101_probe_free_gdo2:
    gpio_free(spi_data->gdo2_pin);

cc1101_probe_free_gdo0_irq:
    free_irq(spi_data->gdo0_irq, spi_data);

cc1101_probe_free_gdo0:
    gpio_free(spi_data->gdo2_pin);

cc1101_probe_free_reset_work:
    kfree(spi_data->reset);

cc1101_probe_free_queue:
    destroy_workqueue(spi_data->queue);

cc1101_probe_free_spi_lock:
    spin_unlock_irqsave(&(spi_data->spi_lock), flags);

cc1101_probe_free_data:
    kfree(spi_data);

cc1101_probe_exit:
    return result;
}


int cc1101_remove(struct spi_device * spi)
{
    struct cc1101_data  * data;

    printk(KERN_INFO "CC1101 module de-initialization\n");

    // TODO: i need to make sure that all transfer operations have finished before free the structure

    data = spi_get_drvdata(spi);

    flush_workqueue(data->queue);
    kfree(data->reset);
    destroy_workqueue(data->queue);

    free_irq(data->gdo0_irq, data);
    free_irq(data->gdo2_irq, data);

    gpio_free(data->gdo0_pin);
    gpio_free(data->gdo2_pin);

    spi_set_drvdata(spi, NULL);

    kfree(data);

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

