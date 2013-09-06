
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <linux/makerbot/fast_gpio.h>
#include <linux/makerbot/fast_gpio_uapi.h>

#include <asm/uaccess.h>


/*
 * This supports access to fast gpio devices using normal userspace I/O calls.
 *
 * We are using the local/experimental group of Major numbers for this device.  
 * We allocate minor numbers dynamically using a bitmask.
 */
#define GPIO_MAJOR			    240	/* local assigned */
#define GPIO_MINOR		        1	/* ... up to 256 */

struct gpiodev_data {
	dev_t			devt;
	spinlock_t		spin_lock;
	struct fast_gpio_platform_data	*pdata;
	struct list_head	device_entry;

	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*buffer;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static int gpio_write(struct fast_gpio_platform_data *pdata, unsigned long arg) {

    gpio_set_value(pdata->pins[0].gpio, arg);   
}

static int gpio_read(struct fast_gpio_platform_data *pdata) {

    return gpio_get_value(pdata->pins[0].gpio);
}

static long gpio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	struct gpiodev_data	*gpio;
    struct fast_gpio_platform_data *pdata;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != FAST_GPIO_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	gpio = filp->private_data;
	spin_lock_irq(&gpio->spin_lock);
    if (gpio->pdata) {
        pdata = gpio->pdata;
    } else {
        pdata = NULL:
    }
	spin_unlock_irq(&gpio->spin_lock);

	if (pdata == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) 
	 *  - prevent concurrent GPIO_IOC_WR_* from morphing
	 *    data fields while GPIO_IOC_RD_* reads them;
	 */
	mutex_lock(&gpio->buf_lock);

	switch (cmd) {
	/* read requests */
	case FAST_GPIO_IOC_RD_VALUE:
		retval = gpio_read(pdata);
		break;
	case FAST_GPIO_IOC_WR_VALUE:
		retval = gpio_write(pdata, arg);
		break;

	default:
        retval = -EINVAL;
        break;
	}

	mutex_unlock(&gpio->buf_lock);
	return retval;
}


int fast_gpio_device_register(struct device *parent) {

    int status;

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);

    gpio->devt = MKDEV(GPIO_MAJOR, GPIO_MINOR);
    dev = device_create(gpio_class, &gpio_dev->dev, gpio->devt,
                gpio, devname);

    status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
    if (status == 0) {
        list_add(&gpio->device_entry, &device_list);
    }
    mutex_unlock(&device_list_lock);

    return status;
    
}


static int gpio_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    const struct fast_gpio_platform_data *pdata = dev_get_platdata(dev);
	struct gpiodev_data	*gpio;
	int			status;
	unsigned long		minor;
    const char *devname = (pdev->id >= 0) ? ("fpgio%s", pdev->id) : "fpgio;

	/* Allocate driver data */
	gpio = kzalloc(sizeof(*gpio), GFP_KERNEL);

	if (!gpio)
		return -ENOMEM;

	/* Initialize the driver data */
	gpio->pdata = pdata;
	spin_lock_init(&gpio->spin_lock);
	mutex_init(&gpio->buf_lock);

    status = fast_gpio_device_register(dev);

    if (status != 0) {
        goto fail1;
    }

    platform_set_drvdata(pdev, gpio);
    for (i = 0; i < pdata->npins; i++) {
        
        gpio_request(pdata->pins.gpio);
        gpio_set_direction(pdata->pins.direction);
        gpio_set_value(pdata->pins.gpio);
    }

	return status;

    fail1:
        kfree(gpio);

    return status;
}

static int gpio_remove(struct gpio_device *gpio_dev)
{
	struct gpio_data *gpio = get_drvdata(&gpio_dev->dev);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&gpio->spin_lock);
	gpio->gpio_dev = NULL;
	dev_set_drvdata(gpio_dev, NULL);
	spin_unlock_irq(&gpio->spin_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&gpio->device_entry);
	device_destroy(gpio_class, gpio->devt);
	clear_bit(MINOR(gpio->devt), minors);
	if (gpio->users == 0)
		kfree(gpio);
	mutex_unlock(&device_list_lock);

	return 0;
}

static int gpio_suspend(struct gpio_device *gpio_dev) {


    return 0;
}

static int gpio_probe(struct gpio_device *gpio_dev) {


    return 0;
}

static const struct file_operations gpio_fops = {
	.owner =	THIS_MODULE,
	.unlocked_ioctl = gpio_ioctl,
};

static struct platform_driver gpio_driver = {
	.driver = {
		.name =		"fast_gpio",
		.owner =	THIS_MODULE,
	},
	.probe =	gpio_probe,
	.remove =	gpio_remove,
    .suspend =  gpio_suspend,
    .resume =   gpio_resume,

};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/fgpio character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *gpio_class;

static int __init gpio_init(void)
{
	int status;

	/* Claim our 256 localy reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_GPIO_MINORS > 256);
	status = register_chrdev(GPIO_MAJOR, "fast_gpio", &gpio_fops);
	if (status < 0)
		return status;

	gpio_class = class_create(THIS_MODULE, "fast_gpio");
	if (IS_ERR(gpio_class)) {
		unregister_chrdev(GPIO_MAJOR, gpio_driver.driver.name);
		return PTR_ERR(gpio_class);
	}

    status = platform_driver_register(&gpio_driver);
	if (status < 0) {
		class_destroy(gpio_class);
		unregister_chrdev(GPIO_MAJOR, gpio_driver.driver.name);
	}
	return status;
}
module_init(gpio_init);

static void __exit gpio_exit(void)
{
	spi_unregister_driver(&gpio_driver);
	class_destroy(gpio_class);
	unregister_chrdev(GPIO_MAJOR, gpio_driver.driver.name);
}
module_exit(gpio_exit);


MODULE_AUTHOR("Makerbot Industries");
MODULE_DESCRIPTION("Fast GPIO toggle for userspace access");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

