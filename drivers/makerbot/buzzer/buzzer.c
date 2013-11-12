/*
*	Buzzer Driver for Birdwing platform
*
*	Copyright (c) 2013 Makerbot Industries LLC
*
*/


#include <linux/init.h>				//init functions
#include <linux/module.h>			//allows dynamically loadable modules
#include <linux/kernel.h>			//printk and other kernel-based lib functions
#include <linux/version.h>			//linux version numbers
#include <linux/fs.h>				//file operations
#include <linux/types.h>			//type defs, macros, etc
#include <linux/kdev_t.h>			//kernel device macros (major, minor, etc)
#include <linux/device.h>			//generic device driver model
#include <linux/platform_device.h>		//allows device drivers to register with the platform
#include <linux/cdev.h>				//character drivers

//#include "buzzer.h"				//custom defines etc
#include "notes.h"				//Standard note frequencies and MIDI note numbers
//#include "seqeunces.h"			//Sequences from MusicForMakerbots

#define BUZZER_COUNT 1

static dev_t buzzer_number;				//device major number

static struct class *class_type;		//global to hold the class type

struct buzzer_dev{
	int sample_rate;			//time base that data is output to the buzzer (sample rate)
	int ctrl_rate;				//time base that control events happen (envelop, glide, etc)
	int seq_rate;				//time base that sequence events happen on (note rate)
	int seq_index;				//index for the sequence
	int seq_tones;				//Array for sequence tones
	int seq_notes;				//Array for the sequence notes
	struct semaphore buzzer_sem;		//Semaphore (might be mutex or spinlock)
	struct cdev buzzer_cdev;		//char device structure
};


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

unsigned int poll(struct buzzer_dev_t *dev){
	printk("Buzzer poll\n");
	return 0;
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
	.open = 	buzzer_open,	//called on first operation, may not need
	.read = 	buzzer_read,	//read from the device? possibly the currently loaded sequence
	.release = 	buzzer_release,	//called when all versions are done, may not need
	.write = 	buzzer_write,	//write data to the device
	.poll = 	buzzer_poll, 	//check if the device is busy making music
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
	ret = 0;
	printk("Buzzer Init\n");


	ret = alloc_chrdev_region(&buzzer_number, 0, BUZZER_COUNT, "Buzzer");	//Try to allocate some space
	if(ret){
		printk("Failed to allocate region\n");
		return ret;
	printk("Registered with <%d, %d>\n", MAJOR(buzzer_number), MINOR(buzzer_number));
	//FIXME this should be in it's own structure probably
	cdev_init(buzzer_cdev, buzzer_fops);				//Init character driver
	
	//This needs to be at the end, goes live after this call returns
	ret = cdev_add(buzzer_cdev, first, 1);				//register 
	return ret;
}
module_init(buzzer_init);


static void __exit buzzer_exit(void){
	printk("Buzzer Exit\n");
	cdev_del(buzzer_cdev);
	//platform_driver_unregister(&buzzer_driver);
	//class_destroy(buzzer_class);
	unregister_chrdev(buzzer_number, BUZZER_COUNT);
}
module_exit(buzzer_exit);

MODULE_AUTHOR("Matt Sterling");
MODULE_DESCRIPTION("Makerbot Buzzer Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
