#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>

#include <asm/uaccess.h>
//Global variable
static dev_t first;		//global to hold 1st device number
static struct cdev c_dev;	//global to hold char device structure
static struct class *cl;	//global for device class
static char c;

//Character device file operations
static int open_file(struct inode *i, struct file *f){
	pr_info("D2: open\n");
	return 0;
}

static int close_file(struct inode *i, struct file *f){
	pr_info("D2: close\n");
	return 0;
}

static ssize_t read_file(struct file *f, char __user *buf, size_t len, loff_t *off){
	pr_info("D2: read\n");
	if(copy_to_user(buf, &c, 1) !=0)
		return -EFAULT;
	else
		if(*off == 0){
			*off+=1;
			return 1;
		} else {
			return 0;
		}
	return 0;
}

static ssize_t write_file(struct file *f, const char __user *buf, size_t len, loff_t *off){
	pr_info("D2: write\n");
	if(copy_from_user(&c, buf + len-1, 1) !=0)
		return -EFAULT;
	else
		return len;
}

//File operation structure
static struct file_operations driver2_fops={
	.owner 		= THIS_MODULE,
	.open		= open_file,
	.release	= close_file,
	.read		= read_file,
	.write		= write_file
};

//Constructor
static int __init driver_two_init(void){
	pr_info("D2: Register...");

	if(alloc_chrdev_region(&first, 0, 1, "DriverTwo")<0)
		return -1;

	if((cl = class_create(THIS_MODULE, "D2")) == NULL){
		unregister_chrdev_region(first, 1);
		return -1;
	}

	if(device_create(cl, NULL, first, NULL, "mynull") == NULL){
		class_destroy(cl);
		unregister_chrdev_region(first, 1);
		return -1;
	}
	
	cdev_init(&c_dev, &driver2_fops);
	if(cdev_add(&c_dev, first, 1) == -1){
		device_destroy(cl, first);
		class_destroy(cl);
		unregister_chrdev_region(first, 1);
		return -1;
	}
	pr_info("success\n");
	return 0;
}

//Destructor
static void __exit driver_two_exit(void){
	cdev_del(&c_dev);
	device_destroy(cl, first);
	class_destroy(cl);
	unregister_chrdev_region(first, 1);
	pr_info("D2: unregistered\n");
}

module_init(driver_two_init);
module_exit(driver_two_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matt Sterling");
MODULE_DESCRIPTION("2nd driver attempt, char driver");
