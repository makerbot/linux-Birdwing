//	
//	Attempt to write a driver from scratch for Linux
//	This should be awesome.
//	MSS 14 Oct 2013
//
//	Copywrite blah blah blah

//Macros that allow functions to include the __init keywork.
//Memory will be cleared after the init stage.
#include <linux/init.h>
//Allows dynamically loaded modules
#include <linux/module.h>
//Printk and other kernel-based lib functions
#include <linux/kernel.h>
//Hmm...
#include <linux/version.h>
//File operations
#include <linux/fs.h>
//Many type defs, macros etc
#include <linux/types.h>
//Kernel devices macros (major, minor, etc)
#include <linux/kdev_t.h>
// generic driver model
#include <linux/device.h>
//allows device drivers to register with the platform
#include <linux/platform_device.h>
//character device drivers
#include <linux/cdev.h>

#include "my_driver.h"

static dev_t first;		//Global holds first device number found
static struct cdev c_dev;	//Global for the char device struct
static struct class *cl;		//global for device class

static int my_first_open(struct mf_device *dev){
	printk("My First open\n");
	return 0;
}

static int my_first_read(struct mf_device *dev){
	printk("My First read\n");
	return 0;
}

static void my_first_write(struct mf_device *dev){
	printk("My First write\n");
}

static void my_first_release(struct mf_device *dev){
	printk("My First release\n");
}

static int __init my_first_init(void){
	if(alloc_chrdev_region(&first, 0, 3, "MyFirstDevice")<0){
		return -1;
	}
	printk("My First Init passed\n");
	printk("Registered with <%d, %d>\n", MAJOR(first), MINOR(first));
	return 0;
}

static void __exit my_first_exit(void){
	unregister_chrdev_region(first, 3);
	printk("My First Exit passed\n");
}

module_init(my_first_init);
module_exit(my_first_exit);

//TODO could define my own operations
static const struct first_ops my_first_ops = {
	.owner 		= THIS_MODULE,
	.open		= my_first_open,
	.read		= my_first_read,
	.write		= my_first_write,
	.release	= my_first_release
};

static int my_first_probe(struct platform_device *pdev){
	int ret;
	ret = 0;
	printk("My First Probe\n");
	return ret;
}

static int my_first_remove(struct platform_device *pdev){
	int ret;
	ret = 0;
	printk("My First remove\n");
	return ret;
}

static struct platform_driver my_first_driver = {
	.probe 		= my_first_probe,
	.remove		= my_first_remove,
	.driver		= {
		.name 	= "my_first",
		.owner	= THIS_MODULE
	},
};

//TODO figure out where this goes, what it does.
//module_platform_driver(my_first_driver);

MODULE_AUTHOR("Matt Sterling");
MODULE_DESCRIPTION("My First Linux Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
