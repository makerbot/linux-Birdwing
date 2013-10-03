
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
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
#define N_GPIO_MINORS			32	/* ... up to 256 */

static DECLARE_BITMAP(minors, N_GPIO_MINORS);

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

    printk(KERN_INFO "set gpio: %d value: %d\n", pdata->pins[0].gpio, (int)arg);
    gpio_set_value(pdata->pins[0].gpio, arg);   
    return 0;
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
        pdata = NULL;
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

    printk(KERN_INFO "switch cmd: %d", cmd);

	switch (cmd) {
	/* read requests */
 	case FAST_GPIO_IOC_RD_VALUE:
        printk(KERN_INFO "read\n");
		retval = gpio_read(pdata);
		break;
	case FAST_GPIO_IOC_WR_VALUE:
        printk(KERN_INFO "write\n");
		retval = gpio_write(pdata, arg);
		break;

	default:
        retval = -EINVAL;
        break;
	}

	mutex_unlock(&gpio->buf_lock);
	return retval;
}

/* The main reason to have this class is to make mdev/udev create the
 * /dev/fgpio character device nodes exposing our userspace API.
 * It also simplifies memory management.
 * this is inialized in the driver init()
 */

static struct class *gpio_class;


int fast_gpio_device_register(struct gpiodev_data *gpio_data, struct device *dev) {

    int status;
	unsigned long		minor;

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);

	minor = find_first_zero_bit(minors, N_GPIO_MINORS);
    gpio_data->devt = MKDEV(GPIO_MAJOR, minor);
    dev = device_create(gpio_class, dev, gpio_data->devt,
                gpio_data, "fgpio%lu", minor);

    status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
    if (status == 0) {
        list_add(&gpio_data->device_entry, &device_list);
		set_bit(minor, minors);
    }
    mutex_unlock(&device_list_lock);

    return status;
    
}


static int gpio_probe(struct platform_device *pdev)
{
    struct fast_gpio_platform_data *pdata = pdev->dev.platform_data;
	struct gpiodev_data	*gpio;
	int			status;
    int i;

	/* Allocate driver data */
	gpio = kzalloc(sizeof(*gpio), GFP_KERNEL);

	if (!gpio)
		return -ENOMEM;

	/* Initialize the driver data */
	gpio->pdata = pdata;
	spin_lock_init(&gpio->spin_lock);
	mutex_init(&gpio->buf_lock);

    status = fast_gpio_device_register(gpio, &pdev->dev);

    if (status != 0) {
        goto fail1;
    }

    platform_set_drvdata(pdev, gpio);
    for (i = 0; i < pdata->npins; i++) {
        
        gpio_request(pdata->pins[i].gpio, "gpio");
        if (pdata->pins[i].direction) {
            gpio_direction_output(pdata->pins[i].gpio, 0);
        } else {
            gpio_direction_input(pdata->pins[i].gpio);
        }
    }

	return status;

    fail1:
        kfree(gpio);

    return status;
}

static int gpio_remove(struct platform_device *pdev)
{
	struct gpiodev_data *gpio = platform_get_drvdata(pdev);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&gpio->spin_lock);
	gpio->pdata = NULL;
	platform_set_drvdata(pdev, NULL);
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

static unsigned bufsiz = 4;
static int gpio_open(struct inode *inode, struct file *filp)
{
	struct gpiodev_data	*gpio_data;
	int			status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(gpio_data, &device_list, device_entry) {
		if (gpio_data->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (!gpio_data->buffer) {
			gpio_data->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (!gpio_data->buffer) {
				printk("FASTGPIO open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			gpio_data->users++;
			filp->private_data = gpio_data;
			nonseekable_open(inode, filp);
		}
	} else
		pr_debug("fast gpio: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	return status;
}

static int gpio_release(struct inode *inode, struct file *filp)
{
	struct gpiodev_data	*gpio_data;
	int			status = 0;

	mutex_lock(&device_list_lock);
	gpio_data = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	gpio_data->users--;
	if (!gpio_data->users) {
		int		dofree;

		kfree(gpio_data->buffer);
		gpio_data->buffer = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&gpio_data->spin_lock);
		dofree = (gpio_data->pdata == NULL);
		spin_unlock_irq(&gpio_data->spin_lock);

		if (dofree)
			kfree(gpio_data);
	}
	mutex_unlock(&device_list_lock);

	return status;
}



static const struct file_operations gpio_fops = {
	.owner =	THIS_MODULE,
	.unlocked_ioctl = gpio_ioctl,
	.open =		gpio_open,
	.release =	gpio_release,
};

static struct platform_driver gpio_driver = {
	.driver = {
		.name =		"fast_gpio",
		.owner =	THIS_MODULE,
	},
	.probe =	gpio_probe,
	.remove =	gpio_remove,

};

/*-------------------------------------------------------------------------*/

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
	platform_driver_unregister(&gpio_driver);
	class_destroy(gpio_class);
	unregister_chrdev(GPIO_MAJOR, gpio_driver.driver.name);
}
module_exit(gpio_exit);


MODULE_AUTHOR("Makerbot Industries");
MODULE_DESCRIPTION("Fast GPIO toggle for userspace access");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

