/*
*	Buzzer Driver for Birdwing platform
*
*	Copyright (c) 2013 Makerbot Industries LLC
*
*/


#include <linux/init.h>				//init functions
#include <linux/module.h>			//allows dynamically loadable modules
#include <linux/moduleparam.h>

#include <linux/kernel.h>			//printk and other kernel-based lib functions
#include <linux/slab.h>				//Memory allocation function
#include <linux/fs.h>				//file operations
#include <linux/errno.h>			//errors
#include <linux/version.h>			//linux version numbers
#include <linux/types.h>			//type defs, macros, etc
#include <linux/kdev_t.h>			//kernel device macros (major, minor, etc)
#include <linux/device.h>			//generic device driver model
#include <linux/platform_device.h>		//allows device drivers to register with the platform
#include <linux/cdev.h>				//character drivers
#include <linux/sched.h>			//scheduling and timing related things
#include <linux/mutex.h>			//mutexes

#include <linux/makerbot/buzzer.h>		//buzzer files

#include "buzzer.h"
#include "notes.h"				//Standard note frequencies and MIDI note numbers
//#include "seqeunces.h"			//Sequences from MusicForMakerbots


int buzzer_major = BUZZER_MAJOR;
int buzzer_minor = BUZZER_MINOR;
int buzzer_count = BUZZER_COUNT;

module_param(buzzer_major, int, S_IRUGO);
module_param(buzzer_minor, int, S_IRUGO);
module_param(buzzer_count, int, S_IRUGO);

struct buzzer_dev buzzer;


static int buzzer_open(struct inode *i, struct file *f){
	int ret;
	ret = 0;
	pr_info("Buzzer Open\n");
	return 0;
}

static int buzzer_read(struct file *f, char __user *buf, size_t len, loff_t *off){
	int ret;
	ret =0;
	pr_info("Buzzer Read\n");
	return ret;
}

static ssize_t buzzer_write(struct file *f, const char __user *buf, size_t len, loff_t *off){
	pr_info("Buzzer Write\n");
	return 0;
}

static int buzzer_release(struct inode *i, struct file *f){
	pr_info("Buzzer release\n");
	return 0;
}

unsigned int poll(struct file *f, struct poll_table_struct *pts){
	pr_info("Buzzer poll\n");
	return 0;
}	

//static void synth(u16 dur, u16 freq_1, u16 freq_2, u32 wave, u16 points){
	//Start Time
	//Current time
	//End time from duration

	//Freq log
	//Wave event duration
	//event frequency

	//period of freq_1
	//wave index
	//next event time

	//while(current_time<end_time){
	//	if(current_time>=next_event){
	//		wave_index = (wave_index+1)%points
	//		write out to pin based on wave position
	//		calculate next event
	//	}
	//	update current_time
	//}

	//write 0 to output pin

//}

//TODO parse sequence to synth
//TODO parse MIDI file to synth

static const struct file_operations buzzer_fops = {
	.owner = 	THIS_MODULE,
	.open = 	buzzer_open,	//called on first operation, may not need
	.read = 	buzzer_read,	//read from the device? possibly the currently loaded sequence
	.release = 	buzzer_release,	//called when all versions are done, may not need
	.write = 	buzzer_write,	//write data to the device
};


static void buzzer_setup_cdev(struct buzzer_dev *dev, int index){
	int ret;
	int devno = MKDEV(buzzer_major, buzzer_minor+index);

	cdev_init(&dev->cdev, &buzzer_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &buzzer_fops;
	ret = cdev_add(&dev->cdev, devno, 1);
	if(ret)
		pr_err("Buzzer: Error %d adding buzzer%d", ret, index);
}


static int buzzer_probe(struct platform_device *pdev){
	int ret;
	//struct buzzer_platform_data *pdata = pdev->dev.platform_data;

	ret = 0;
	pr_info("Buzzer Probe\n");

	pr_info("Start time %lu\n", jiffies);
	pr_info("HZ setting: %d\n", HZ);
	pr_info("Ktime: %lld\n", ktime_to_ns(ktime_get()));

	//allocate memory for the number of devices we have
	//buzzer = kmalloc(buzzer_count *sizeof(struct buzzer_dev), GFP_KERNEL);
	//if(!buzzer){
	//	ret = -ENOMEM;
	//	goto fail;	//Undo previously registered char dev
	//}
	//memset(buzzer, 0, buzzer_count*sizeof(struct buzzer_dev));

	//TODO make this a loop, only one device for now
		buzzer.index = 0;	//nothing to index
		buzzer.freq = 440;	//hz
		buzzer.duration = 1;	//not sure what this corresponds to, maybe make it msec
		mutex_init(&buzzer.buzzer_mutex);
		buzzer_setup_cdev(&buzzer, 0);	//possibly unroll this instead of having another func
		//platform data?

	//Friend devices? probably don't need this
	//dev = MKDEV(buzzer_major, buzzer_minor+buzzer_count);
	//dev += buzzer_p_init(dev);		//??
	//dev += buzzer_access_init(dev);	//??

	//platform_set_drvdata(pdev, buzzer);

	pr_info("End time: %lu\n", jiffies);
	pr_info("Ktime: %lld\n", ktime_to_ns(ktime_get()));

	return ret;
fail:
	//kfree(buzzer);
	return ret;

}

static int buzzer_remove(struct platform_device *pdev){
	int ret;

	ret = 0;
	pr_info("Buzzer Remove\n");
	return ret;
}


static struct platform_driver buzzer_driver = {
	.driver = {
		.name = "buzzer",
		.owner = THIS_MODULE,
	},
	.probe = buzzer_probe,
	.remove= buzzer_remove,
};

static void __exit buzzer_exit(void){

	//dev_t devno;
	//devno = MKDEV(buzzer_major, buzzer_minor);
	pr_info("Buzzer Exit\n");

	//platform_driver_unregister(&buzzer_driver);
	//if(buzzer_devices){
	//	cdev_del(&buzzer_devices[0].cdev);
	//	kfree(buzzer_devices);
	//}

	unregister_chrdev_region(buzzer.dev, buzzer_count);

}


static int __init buzzer_init(void){
	int ret;
	//dev_t dev = 0;
	ret = 0;
	pr_info("Buzzer Init\n");

	ret=alloc_chrdev_region(&buzzer.dev, buzzer_minor, buzzer_count, "buzzer");
	if(ret<0){
		pr_err("Could not allocate char dev region: %d\n", ret);
		return -1;
	}

	//if(buzzer_major){
	//	dev = MKDEV(buzzer_major, buzzer_minor);
	//	ret = register_chrdev_region(dev, buzzer_count, "buzzer");
	//} else {
	//	ret = alloc_chrdev_region(&dev, buzzer_minor, buzzer_count, "buzzer");
	//	buzzer_major = MAJOR(dev);
	//}
	//
	pr_info("Registered with <%d, %d>\n", MAJOR(buzzer.dev), MINOR(buzzer.dev));

	if(ret <0){
		pr_warn("Buzzer: can't get major %d\n", buzzer_major);
		return ret;
	}


	//ret = platform_driver_register(&buzzer_driver);
	//if(ret<0){
	//	pr_err("Buzzer: can't register platform driver %d\n",ret);
	//	goto fail;
	//}


	return ret;

//fail:
//	buzzer_exit();
//	return ret;
}

module_init(buzzer_init);
module_exit(buzzer_exit);

MODULE_AUTHOR("Matt Sterling");
MODULE_DESCRIPTION("Makerbot Buzzer Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.2");
