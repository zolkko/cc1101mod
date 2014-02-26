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
#include <linux/delay.h>

#include <linux/string.h>

#include "cc1101.h"
#include "cc1101m.h"


// TODO: pass SPI_BUS_MASTER, SPI_CHIP_SELECT, GDO0_pin and GDO2_pin as parameters to the module


#define CC1101_MNAME    "cc1101m"
#define SPI_BUS_MASTER  0
#define SPI_CHIP_SELECT 0
#define SPI_MAX_SPEED   50000000


#define SPI_SPEED_HZ      2000000
#define SPI_BITS_PER_WORD 8
#define SPI_DELAY_USECS   0


#define GDO0_pin  22
#define GDO0_name "GDO0"


#define GDO2_pin  23
#define GDO2_name "GDO2"

#define GDO1_pin  9
#define GDO1_name "GDO1"
#define SO_pin    GDO1_pin


#define GDO0_IRQ_NAME "GDO0 interrupt handler"
#define GDO2_IRQ_NAME "GDO2 interrupt handler"


#define CC1101_WORK_BUF_LEN 128

static int cc1101_remove(struct spi_device * spi);
static int cc1101_probe(struct spi_device * spi);


static struct spi_board_info cc1101_hotplug_spi_device = {
    .modalias = CC1101_MNAME,
    .max_speed_hz = SPI_MAX_SPEED,
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
    struct mutex buffer_mutex;

    int gdo0_pin;
    int gdo1_pin;
    int gdo2_pin;

    int gdo0_irq;
    int gdo2_irq;

    int gdo0_val;
    int gdo2_val;

    struct workqueue_struct * queue;

    u8 buf[CC1101_WORK_BUF_LEN];
};


struct cc1101_work {
    struct work_struct work;
    struct cc1101_data * data;
};


static irqreturn_t gdo0_interrupt_handler(int i, void * irq_data)
{
    struct cc1101_data * data = (struct cc1101_data *)irq_data;
    int value = gpio_get_value(data->gdo0_pin);

    printk(KERN_INFO "cc1101: gdo0 interrupt handler, value = %d\n", value);

    if (value != data->gdo0_val) {
        data->gdo0_val = value;
        printk(KERN_INFO "cc1101: gdo0 interrupt handler value: %d\n", value);
        return IRQ_HANDLED;
    } else {
        return IRQ_NONE;
    }
}


static irqreturn_t gdo2_interrupt_handler(int i, void * irq_data)
{
    struct cc1101_data * data = (struct cc1101_data *)irq_data;
    int value = gpio_get_value(data->gdo2_pin);

    printk(KERN_INFO "cc1101: gdo2 interrupt handler, value = %d\n", value);

    if (value != data->gdo2_val) {
        data->gdo2_val = value;
        printk(KERN_INFO "CC1101 GDO2 interrupt handler value: %d\n", value);
        return IRQ_HANDLED;
    } else {
        return IRQ_NONE;
    }
}


static inline bool cc1101_ready(struct cc1101_data * self)
{
    return gpio_get_value(self->gdo1_pin) == 0;
}


static inline int cc1101_burst_write(struct cc1101_data * self, u8 addr, const u8 * data, int len)
{
    int result;
    struct spi_message msg;
    struct spi_transfer xfer = {
        .len = len + 1,
        .tx_buf = self->buf,
        .rx_buf = self->buf,
        .speed_hz = SPI_SPEED_HZ,
        .bits_per_word = SPI_BITS_PER_WORD,
        .delay_usecs = SPI_DELAY_USECS
    };

    if (len + 1 > CC1101_WORK_BUF_LEN) {
        printk(KERN_ERR "cc1101: could not burst write. %d too much data to send\n", len + 1);
        return 0;
    }

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);

    mutex_lock(&self->buffer_mutex);
    self->buf[0] = addr | CC1101_BURST_BIT_bm;

    memcpy(&self->buf[1], data, len);

    result = spi_sync(self->spi, &msg);
    if (result)
        printk(KERN_ERR "cc1101: burst write failed for address 0x%hhx\n", addr);
    mutex_unlock(&self->buffer_mutex);

    return result;
}

static inline int cc1101_write(struct cc1101_data * self, u8 addr, u8 data)
{
    int result;
    struct spi_message msg;
    struct spi_transfer xfer = {
        .len = 2,
        .tx_buf = self->buf,
        .rx_buf = self->buf,
        .speed_hz = SPI_SPEED_HZ,
        .bits_per_word = SPI_BITS_PER_WORD,
        .delay_usecs = SPI_DELAY_USECS
    };

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);

    mutex_lock(&self->buffer_mutex);
    self->buf[0] = addr;
    self->buf[1] = data;

    result = spi_sync(self->spi, &msg);
    if (result)
        printk(KERN_ERR "cc1101: write failed for address 0x%hhx\n", addr);
    mutex_unlock(&self->buffer_mutex);

    return result;
}


static inline int cc1101_strobe(struct cc1101_data * self, u8 addr)
{
    int result;
    struct spi_message msg;
    struct spi_transfer xfer = {
        .len = 1,
        .tx_buf = self->buf,
        .rx_buf = self->buf,
        .speed_hz = SPI_SPEED_HZ,
        .bits_per_word = SPI_BITS_PER_WORD,
        .delay_usecs = SPI_DELAY_USECS
    };

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);

    mutex_lock(&self->buffer_mutex);
    self->buf[0] = addr;

    result = spi_sync(self->spi, &msg);
    if (result)
        printk(KERN_ERR "cc1101: strobe failed for address 0x%hhx\n", addr);
    mutex_unlock(&self->buffer_mutex);

    return result;
}


static void cc1101_reset_configure(struct work_struct * work_ptr)
{
    u8 i;
    u8 status;
    struct cc1101_work * wrk = container_of(work_ptr, struct cc1101_work, work);

    printk(KERN_INFO "cc1101: module resets chipcon\n");

    // send reset strobe
    cc1101_strobe(wrk->data, CCx_SRES);
    mutex_lock(&wrk->data->buffer_mutex);
    status = wrk->data->buf[0];
    mutex_unlock(&wrk->data->buffer_mutex);

    // wait for chip ready
    for (i = 0; i < 0xff && (status & CC1101_STATUS_CHIP_RDYn_bm); i++) {
        msleep(50);

        cc1101_strobe(wrk->data, CCx_SNOP);
        mutex_lock(&wrk->data->buffer_mutex);
        status = wrk->data->buf[0];
        mutex_unlock(&wrk->data->buffer_mutex);
    }

    if (status & CC1101_STATUS_CHIP_RDYn_bm) {
        printk(KERN_ERR "cc1101: chip is not ready. Status: 0x%x\n", status);
    }

    // configure cc1101 chip
    cc1101_burst_write(wrk->data, CCx_REG_BEGIN, cc1101_cfg, sizeof(cc1101_cfg));

    printk(KERN_INFO "cc1101: configuration has been written to SPI interface\n");

    kfree(wrk);

    printk(KERN_INFO "cc1101: successfully reset chipcon\n");
}


int cc1101_probe(struct spi_device * spi)
{
    int status;
    int irq_num;
    int result;
    unsigned long flags;
    struct cc1101_data * spi_data;
    struct cc1101_work * reset_work;

    result = 0;

    printk(KERN_INFO "cc1101: spi driver probe\n");

    // setup data
    spi_data = kzalloc(sizeof(*spi_data), GFP_KERNEL);
    if (IS_ERR(spi_data)) {
        printk(KERN_ERR "cc1101: module could not allocate memory\n");
        result = -ENOMEM;
        goto cc1101_probe_exit;
    }

    spi_data->spi = spi;
    spi_data->gdo0_pin = GDO0_pin;
    spi_data->gdo1_pin = GDO1_pin;
    spi_data->gdo2_pin = GDO2_pin;

    // create work queue
    spi_data->queue = create_singlethread_workqueue("CC1101-queue");
    if (IS_ERR(spi_data->queue)) {
        printk(KERN_ERR "CC1101 module could not allocate memory for queue\n");
        result = -ENOMEM;
        goto cc1101_probe_free_data;
    }

    // setup reset work
    reset_work = kzalloc(sizeof(struct cc1101_work), GFP_KERNEL);
    if (IS_ERR(reset_work)) {
        printk(KERN_ERR "CC1101 module could not allocate memory for reset work\n");
        goto cc1101_probe_free_queue;
    }
    INIT_WORK(&reset_work->work, cc1101_reset_configure);
    reset_work->data = spi_data;

    // initialize mutex
    mutex_init(&spi_data->buffer_mutex);

    // disable interrupts while setuping IRQ handlers
    spin_lock_init(&spi_data->spi_lock);
    spin_lock_irqsave(&(spi_data->spi_lock), flags);

    // setup GDO0
    if (!gpio_is_valid(spi_data->gdo0_pin)) {
        printk(KERN_ERR "CC1101 GDO0 pin %d is invalid\n", spi_data->gdo0_pin);
        result = -EINVAL;
        goto cc1101_probe_free_spin_lock;
    }

    status = gpio_request_one(spi_data->gdo0_pin, GPIOF_DIR_IN, GDO0_name);
    if (status) {
        printk(KERN_ERR "CC1101 could not acquire GDO0. Reason: %d\n", status);
        result = -EBUSY;
        goto cc1101_probe_free_spin_lock;
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
    if (status) {
        printk(KERN_ERR "CC1101 GDO0 failed to request irq. Reason: %d\n", status);
        result = -EBUSY;
        goto cc1101_probe_free_gdo0;
    }

    // setup GDO1
    if (!gpio_is_valid(spi_data->gdo1_pin)) {
        printk(KERN_ERR "cc1101: GDO1 pin %d is invalid\n", spi_data->gdo1_pin);
        result = -EINVAL;
        goto cc1101_probe_free_gdo0_irq;
    }

    status = gpio_request_one(spi_data->gdo1_pin, GPIOF_DIR_IN, GDO1_name);
    if (status) {
        printk(KERN_ERR "cc1101: could not acquire GDO1 pin. Status: %d\n", status);
        result = -EBUSY;
        goto cc1101_probe_free_gdo0_irq;
    }

    // setup GDO2
    if (!gpio_is_valid(spi_data->gdo2_pin)) {
        printk(KERN_ERR "CC1101 GDO2 pin %d is invalid\n", spi_data->gdo2_pin);
        result = -EINVAL;
        goto cc1101_probe_free_gdo1;
    }

    status = gpio_request_one(spi_data->gdo2_pin, GPIOF_DIR_IN, GDO2_name);
    if (status) {
        printk(KERN_ERR "CC1101 could not acquire GDO2. Reason: %d\n", status);
        result = -EBUSY;
        goto cc1101_probe_free_gdo1;
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
    if (status) {
        printk(KERN_ERR "CC1101 GDO2 failed to request irq. Reason: %d\n", status);
        result = -EBUSY;
        goto cc1101_probe_free_gdo2;
    }

    spi->max_speed_hz = SPI_MAX_SPEED;
    status = spi_setup(spi);
    if (status < 0) {
        printk(KERN_ERR "cc1101: cannot setup spi device. Status: %d\n", status);
        result = -EINVAL;
        goto cc1101_probe_free_gdo2;
    }

    // setup the rest of the struct
    spi_set_drvdata(spi, spi_data);
    spin_unlock_irqrestore(&spi_data->spi_lock, flags);

    printk(KERN_INFO "CC1101 schedules reset-config work\n");

    status = queue_work(spi_data->queue, &(reset_work->work));
    if (!status) {
        printk(KERN_ERR "CC1101 could not schedule reset work\n");
        result = -EINVAL;
        goto cc1101_probe_free_gdo2_irq;
    }

    printk(KERN_INFO "CC1101 spi driver finished probing\n");

    return 0;

cc1101_probe_free_gdo2_irq:
    free_irq(spi_data->gdo2_irq, spi_data);

cc1101_probe_free_gdo2:
    gpio_free(spi_data->gdo2_pin);

cc1101_probe_free_gdo1:
    gpio_free(spi_data->gdo1_pin);

cc1101_probe_free_gdo0_irq:
    free_irq(spi_data->gdo0_irq, spi_data);

cc1101_probe_free_gdo0:
    gpio_free(spi_data->gdo2_pin);

cc1101_probe_free_spin_lock:
    spin_unlock_irqrestore(&spi_data->spi_lock, flags);

// cc1101_probe_free_reset_work:
    kfree(reset_work);

cc1101_probe_free_queue:
    destroy_workqueue(spi_data->queue);

cc1101_probe_free_data:
    kfree(spi_data);

cc1101_probe_exit:
    return result;
}


int cc1101_remove(struct spi_device * spi)
{
    // TODO: there is a chance of memory leak because of reset_work
    struct cc1101_data  * data;
    unsigned long flags;

    printk(KERN_INFO "CC1101 driver removing in progress\n");

    data = spi_get_drvdata(spi);

    flush_workqueue(data->queue);
    destroy_workqueue(data->queue);

    spin_lock_irqsave(&data->spi_lock, flags);
    free_irq(data->gdo0_irq, data);
    free_irq(data->gdo2_irq, data);

    gpio_free(data->gdo0_pin);
    gpio_free(data->gdo1_pin);
    gpio_free(data->gdo2_pin);

    data->spi = NULL;
    spi_set_drvdata(spi, NULL);

    spin_unlock_irqrestore(&data->spi_lock, flags);
    kfree(data);

    printk(KERN_INFO "CC1101 driver has been removed\n");
    return 0;
}


static int __init cc1101_module_init(void)
{
    int status;
    struct spi_master *master;

    printk(KERN_INFO "CC1101 module initialization in progress\n");

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
    printk(KERN_INFO "CC1101 module deinitialization\n");

    spi_unregister_driver(&cc1101_driver);
    spi_unregister_device(cc1101_spi_device);
    cc1101_spi_device = NULL;

    printk(KERN_INFO "CC1101 module has been deinitialized\n");
}


module_init( cc1101_module_init );
module_exit( cc1101_module_exit );


MODULE_DESCRIPTION("CC1101 support module");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zolkko");

