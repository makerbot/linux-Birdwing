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
#include "sequences.h"			//Sequences from MusicForMakerbots


struct buzzer_dev buzzer;
static struct class *buzzer_class;
//extern const uint32_t sequences[];

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

//Write one of the synth indexes or return -1 for invalid 
static ssize_t buzzer_write(struct file *f, const char __user *buf, size_t len, loff_t *off){
	int ret;
	char c[len];	//buffer for recieved characters
	int i;		//can probably remove this
	i = 0;
	ret = 0;

	if(copy_from_user(&c, buf, len)){
		ret = -EFAULT;
	} else {
		ret = len;
	}
	//TODO use write to index the sequences
	buzzer.index = simple_strtoul(c, NULL, 0);
	if(buzzer.index<SEQ_COUNT && buzzer.index >= 0){
		pr_info("Play Seq %d\n", buzzer.index);
		do{
			//TODO reduce the size of the sequence arrays (or pad) to make this a bitshift
			synth(sequences[buzzer.index][(i*6)],			//Duration
				sequences[buzzer.index][(i*6)+1],		//Freq
				sequences[buzzer.index][(i*6)+3],		//Wave
				sequences[buzzer.index][(i*6)+4]);		//Npts
			i++;
		}while(sequences[buzzer.index][i*6]);				//Always end with a row of zeros or this won't break

	} else {
		ret = -EINVAL;
	}

	return ret;
}

static int buzzer_release(struct inode *i, struct file *f){
	pr_info("Buzzer release\n");
	return 0;
}


static void synth(uint16_t dur, uint16_t freq, uint16_t wave, uint16_t npts){
//static void synth(u16 dur, u16 freq_1, u16 freq_2, u32 wave, u16 points){ //Get to this later
	//Dur in msec, freq in Hz, per = 1/freq, wave and pts are dimensionless
	//wave could be a complex binary value, but is 1 for all seqs currently
	//npts effectively sets the modulation rate of the wave. 
	//Assume Wave = 1
	// npts = 2 -> 50% duty cycle
	// npts = 4 -> 25% duty cycle

		/*CAUTION KERNEL DOES NOT LIKE FLOATING POINT MATH*/


	long long start_time, curr_time, end_time, next_event, ns_per;
	uint8_t mod_index;

	mutex_lock(&buzzer.mutex);				//Only one process can access the buzzer at a time

	start_time = ktime_to_us(ktime_get());			//get the current time
	curr_time = start_time;
	end_time = start_time + (dur*1000);			//calc the end time (in nanoseconds)
	ns_per = 1000000 / (npts*freq);				//Event period: when pin might change next (in ns)

	//Freq log: ln(f2/f1), only needed if there's a slide
	//event frequency: 1/(end-start), only needed if there's a slide
	mod_index = 0;
	next_event = curr_time+(ns_per);
	//
	//pr_info("%dHz (%lld ns) for %dmSec with w=%d modulated %lld nsec \n", freq, ns_per, dur, wave, ns_mod_rate);
	//FIXME make this usleep_range or something that doesn't hold the kernel
	while(curr_time < end_time){
		if(curr_time >= next_event){
			mod_index = (mod_index+1)%npts;
			gpio_set_value(BUZZER_OUT, (wave>>mod_index)&1);
			next_event = curr_time+(ns_per);
		}
		curr_time = ktime_to_us(ktime_get());
	}

	gpio_set_value(BUZZER_OUT, 0);

	mutex_unlock(&buzzer.mutex);
}

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
	buzzer.duration = 50;	//msec? sure
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
MODULE_VERSION("0.4");
