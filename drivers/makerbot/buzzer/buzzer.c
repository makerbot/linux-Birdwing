/*
*	Buzzer Driver for Birdwing platform
*
*	Copyright (c) 2013 Makerbot Industries LLC
*
*/


#include <linux/init.h>			//init functions
#include <linux/module.h>		//allows dynamically loadable modules
#include <linux/kernel.h>		//printk and other kernel-based lib functions
#include <linux/version.h>		//??
#include <linux/fs.h>			//file operations
#include <linux/types.h>		//type defs, macros, etc
#include <linux/kdev_t.h>		//kernel device macros (major, minor, etc)
#include <linux/device.h>		//generic device driver model
#include <linux/platform_device.h>	//allows device drivers to register with the platform
#include <linux/cdev.h>			//character drivers

#include "buzzer.h"			//custom defines etc
#include "notes.h"			//Standard note frequencies and MIDI note numbers
//#include "seqeunces.h"			//Sequences from MusicForMakerbots

struct buzzer_dev_t buzzer_dev;				//device
static struct cdev buzzer_cdev;				//global for char device struct
static struct *class_type;				//global to hold the class type


static int buzzer_open(struct buzzer_dev_t *dev){
	int ret;
	ret = 0;
	printk("Buzzer Open\n");
	return ret;
}

static int buzzer_read(struct buzzer_dev_t *dev){
	int ret;
	ret =0;
	printk("Buzzer Read\n");
	return ret;
}

static void buzzer_write(struct buzzer_dev_t *dev){
	printk("Buzzer Write\n");
}

static void buzzer_release(struct buzzer_dev_t *dev){
	printk("Buzzer release\n");
}

static void synth(u16 dur, u16 freq_1, u16 freq_2, u32 wave, u16 points){
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

}

//TODO parse sequence to synth
//TODO parse MIDI file to synth


static int buzzer_probe(struct platform_device *pdev){
	int ret;
	ret = 0;
	printk("Buzzer Probe\n");
	return ret;
}

static int buzzer_remove(struct platform_device *pdev){
	int ret;
	ret = 0;
	printk("Buzzer Remove\n");
	return ret;
}

static const struct file_operations buzzer_fops = {
	.owner = 	THIS_MODULE,
	.open = 	buzzer_open,
	.read = 	buzzer_read,
	.release = 	buzzer_release,
	//TODO anything else?
};

static struct platform_driver buzzer_driver = {
	.driver = {
		.name = "buzzer",
		.owner = THIS_MODULE,
	},
	.probe = buzzer_probe,
	.remove= buzzer_remove,
};

static int __init buzzer_init(void){
	int ret;
	printk("Buzzer Init\n");
}

static void __exit buzzer_exit(void){
	printk("Buzzer Exit\n");
	platform_driver_unregister(&buzzer_driver);
	class_destroy(buzzer_class);
	unregister_chrdev(MAJOR, driver_name);
}

module_init(buzzer_init);
module_exit(buzzer_exit);

MODULE_AUTHOR("Matt Sterling");
MODULE_DESCRIPTION("Makerbot Buzzer Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
