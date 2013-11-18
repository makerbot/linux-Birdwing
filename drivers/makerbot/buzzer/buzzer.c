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
#include <linux/gpio.h>				//gpio...how fast is this?

#include <asm/uaccess.h>			//copy to / copy from user
#include <asm/io.h>				//io

#include <linux/makerbot/buzzer.h>		//buzzer files

#include "buzzer.h"
#include "notes.h"				//Standard note frequencies and MIDI note numbers
//#include "seqeunces.h"			//Sequences from MusicForMakerbots


struct buzzer_dev buzzer;
static struct class *buzzer_class;

static int buzzer_open(struct inode *i, struct file *f){
	int ret;
	ret = 0;
	pr_info("Buzzer Open\n");
	return 0;
}

static ssize_t buzzer_read(struct file *f, char __user *buf, size_t len, loff_t *off){

	char c;
	pr_info("Buzzer Read\n");
	pr_info("Len: %d\n", len);
	pr_info("Offset: %lld\n", *off);
	if(*off > 0){
		pr_info("Freq: %d\n", buzzer.freq);
		return 0;
	}else{
		if(copy_to_user(buf, &c, 1) != 0){
			return -EFAULT;
		} else{
			(*off)++;
			return 1;
		}
	}
	return 0;
}

static ssize_t buzzer_write(struct file *f, const char __user *buf, size_t len, loff_t *off){
	int ret;
//	long long start, end;
	char c[len];	//create a buffer

	ret = 1;
	//start = ktime_to_ns(ktime_get());

	//if(copy_from_user(&buzzer.freq, buf+len-1, 1)!=0){
	if(copy_from_user(&c, buf, len)){
		ret = -EFAULT;
	} else {
		ret = len;
	}
	//for(ret=0; ret<len; ret++){
	//	pr_info("C[%d]: %d\n", ret, c[ret]);
	//	//TODO check for LF, NL, etc
	//	//Check convert to int
	//}
		//Store int to freq
	buzzer.freq = simple_strtoul(c, NULL, 0);
	synth(buzzer.freq, buzzer.duration);
	//pr_info("Convert: %d\n", simple_strtoul(c, NULL, 0));
	//end = ktime_to_ns(ktime_get());

	ret = len;
	//pr_info("Duration: %lld\n", (end-start));
	return ret;
}

static int buzzer_release(struct inode *i, struct file *f){
	pr_info("Buzzer release\n");
	return 0;
}


static void synth(int freq, int dur){
//static void synth(u16 dur, u16 freq_1, u16 freq_2, u32 wave, u16 points){ //Get to this later
	//Freq in Hz
	//Dur in msec
	//lock mutex?
	
	long long start_time, curr_time, end_time, ns_half_period, next_event; //ns_dur
	bool is_high;
	start_time = ktime_to_ns(ktime_get());			//get the current time
	curr_time = start_time;
	end_time = start_time + (dur*1000000);			//calc the end time
	//ns_dur = dur*1000000					//convert duration to nanoseconds
	ns_half_period = 1000000000 / (2*freq);			//get half the period in nanoseconds
		/*CAUTION KERNEL DOES NOT LIKE FLOATING POINT MATH*/
	next_event = curr_time+ns_half_period;
	is_high = 0;


	//Freq log
	//Wave event duration
	//event frequency

	//period of freq_1
	//wave index
	//ptr = (void __iomem*)GPIO_BASE;
	//ptr +=GPIO_OUT_OFFSET;

	pr_info("Synth! %dHz for %dms\n", freq, dur);

	while(curr_time < end_time){
		if(curr_time >= next_event){
			if(is_high){
				gpio_set_value(BUZZER_OUT, 0);
				is_high = 0;
			} else {
				gpio_set_value(BUZZER_OUT, 1);
				is_high = 1;
			}
			next_event += ns_half_period;
		}
		curr_time = ktime_to_ns(ktime_get());
	}

	gpio_set_value(BUZZER_OUT, 0);

	//while(current_time<end_time){
	//	if(current_time>=next_event){
	//		wave_index = (wave_index+1)%points
	//		write out to pin based on wave position
	//		calculate next event
	//	}
	//	update current_time
	//}

	//Unlock Mutex
}

//TODO parse sequence to synth
//TODO parse MIDI file to synth

static const struct file_operations buzzer_fops = {
	.owner = 	THIS_MODULE,
	.open = 	buzzer_open,	//called on first operation, may not need
	.read = 	buzzer_read,	//read from the device? possibly the currently loaded sequence
	.release = 	buzzer_release,	//called when all versions are done, may not need
	.write = 	buzzer_write,	//write data to the device
};

static int __init buzzer_create_node(void){
	if(!buzzer_class){
		buzzer_class = class_create(THIS_MODULE, "buzzer_class");
		if(!buzzer_class){
			pr_err("Buzzer class creation failed.\n");
			return -1;
		}
	}

	buzzer.device = device_create(buzzer_class, NULL, buzzer.devt, NULL, "buzzer_dv");
	if(!buzzer.device){
		pr_err("Buzzer device creation failed.\n");
		return -1;
	}

	return 0;
}

static int __init buzzer_destroy_node(void){
	if(buzzer.device)
		device_destroy(buzzer_class, buzzer.devt);
	class_destroy(buzzer_class);

	return 0;
}

static int buzzer_probe(struct platform_device *pdev){
	int ret;
	ret = 0;
	pr_info("Buzzer Probe\n");

	//allocate memory for the number of devices we have
	//buzzer = kmalloc(buzzer_count *sizeof(struct buzzer_dev), GFP_KERNEL);
	//if(!buzzer){
	//	ret = -ENOMEM;
	//	goto fail;	//Undo previously registered char dev
	//}
	//memset(buzzer, 0, buzzer_count*sizeof(struct buzzer_dev));
	return ret;
}

static int buzzer_remove(struct platform_device *pdev){
	int ret;
	ret = 0;
	pr_info("Buzzer Remove\n");
	return ret;
}

//Not sure where this would get called
static struct platform_driver buzzer_driver = {
	.driver = {
		.name = "buzzer",
		.owner = THIS_MODULE,
	},
	.probe = buzzer_probe,
	.remove= buzzer_remove,
};

static int __init buzzer_dev_config(void){

	int ret;

	pr_info("Buzzer Device Config\n");

	cdev_init(&buzzer.cdev, &buzzer_fops);
	buzzer.cdev.owner = THIS_MODULE;

	ret = alloc_chrdev_region(&buzzer.devt, 0, 1, "buzzer_cr");
	if (ret < 0){
		pr_err("Chrdev Allocation failed: %d\n", ret);
		return -1;
	}

	ret = cdev_add(&buzzer.cdev, buzzer.devt, 1);
	if(ret){
		pr_err("Cdev add failed: %d\n", ret);
		unregister_chrdev_region(buzzer.devt, 1);
		return -1;
	}
	pr_info("Registered with <%d, %d>\n", MAJOR(buzzer.devt), MINOR(buzzer.devt));

	buzzer_create_node();

	return 0;
}

static void buzzer_dev_cleanup(void){
	buzzer_destroy_node();

	cdev_del(&buzzer.cdev);
	unregister_chrdev_region(buzzer.devt, 1);
}

static int __init buzzer_init(void){

	int ret;
	mutex_init(&buzzer.mutex);

	ret = buzzer_dev_config();		//changed the name of this to avoid confusion
	if(ret !=0){
		pr_err("Buzzer Device Init failed\n");
		return -1;
	}

	//FIXME this never gets used, can probably take it out
	ret = platform_driver_register(&buzzer_driver);
	if(ret <0){
		pr_err("Buzzer Platform Driver registration failed \n");
		return -1;
		//TODO need to unregister stuff?
	}
	//TODO init low level stuff here

	//Assign defaults
	//TODO need set and get methods for this
	buzzer.index = 1;	//
	buzzer.freq = 440;	//Hz
	buzzer.duration = 500;	//msec? sure
	buzzer.polywave = 1;	//good enough

	return 0;
}

static void __exit buzzer_exit(void){
	platform_driver_unregister(&buzzer_driver);
	buzzer_dev_cleanup();
}
module_init(buzzer_init);
module_exit(buzzer_exit);

MODULE_AUTHOR("Matt Sterling");
MODULE_DESCRIPTION("Makerbot Buzzer Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.3");
