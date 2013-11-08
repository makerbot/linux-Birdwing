/*
*	Buzzer Driver for Birdwing platform
*
*	Copyright (c) 2013 Makerbot Industries LLC
*
*/


#include <linux/init.h>			//init functions
#include <linux/module.h>		//allows dynamically loadable modules
#include <linux/kernel.h>		//printk and other kernel-based lib functions
#include <linux/versioh.h>		//??
#include <linux/fs.h>			//file operations
#include <linux/types.h>		//type defs, macros, etc
#include <linux/kdev_t.h>		//kernel device macros (major, minor, etc)
#include <linux/device.h>		//generic device driver model
#include <linux/platform_device.h>	//allows device drivers to register with the platform
#include <linux/cdev.h>			//character drivers

#include "buzzer.h"			//custom defines etc
#include "notes.h"			//Standard note frequencies and MIDI note numbers
#include "seqeunces.h"			//Sequences from MusicForMakerbots

struct buzzer_dev_t{			//custom device type
	dev_t 		devt;
	spinlock_t	spin_lock;
	struct buzzer_platform_data	*pdev;
	struct mutex	buf_lock;
	u8		*buffer;
} buzzer_dev;				//device
//TODO character dev?
//TODO device class?

//open the buzzer device
static int buzzer_open(struct buzzer_platform_data *pdata){
	int ret;
	ret = 0;
	printk("Buzzer Open\n");
	return ret;
}

//read from the buzzer?
static int buzzer_read(struct buzzer_platform_data *pdata){
//what would we read from here? Sequence info?
	int ret;
	ret =0;
	printk("Buzzer Read\n");
	return ret;
}

static void buzzer_write(struct buzzer_platform_data *pdata){
	printk("Buzzer Write\n");
}

//TODO? write file?
//TODO? Release file

//TODO Synth
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
	platform_driver_ungregister(&buzzer_driver);
	class_destroy(buzzer_class);
	unregister_chrdev(MAJOR, driver_name);
}

module_init(buzzer_init);
module_exit(buzzer_exit);

MODULE_AUTHOR("Matt Sterling");
MODULE_DESCRIPTION("Makerbot Buzzer Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
