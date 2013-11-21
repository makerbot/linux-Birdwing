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
#include <linux/delay.h>			//delay / sleep
#include <linux/spinlock.h>			//spinlockit

#include <asm/uaccess.h>			//copy to / copy from user
#include <asm/io.h>				//io


#include <linux/makerbot/buzzer.h>		//buzzer files

#include "buzzer.h"
#include "notes.h"				//Standard note frequencies and MIDI note numbers
#include "sequences.h"				//Sequences from MusicForMakerbots


struct buzzer_dev buzzer;
static struct class *buzzer_class;

static int buzzer_open(struct inode *i, struct file *f){
	//nothing to do here...
	return 0;
}

static ssize_t buzzer_read(struct file *f, char __user *buf, size_t len, loff_t *off){
	//Don't really need this, nothing to read particularly, though could get last sequence
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
	//TODO don't let this get too big, maybe max out at 32
	char c[len+1];		//buffer for characters, needs to be len+1 to hold end NULL char
	int i;
	unsigned int index;
	mutex_lock(&buzzer.mutex);
	i = 0;
	ret = 0;

	if(copy_from_user(&c, buf, len))
		return -EFAULT;

	c[len]=0;				//NULL char termination

	if(!(kstrtouint(c, 0, &index))){	//0 = successful conversion
		if(index<SEQ_COUNT ){
			pr_info("Play Seq %d\n", index);
			do{
			//TODO reduce the size of the sequence arrays (or pad) to make this a bitshift
			//synth() doesn't return untill it has completed it's note
				synth(sequences[index][i],		//Duration
					sequences[index][i+1],		//Freq
					sequences[index][i+3],		//Wave
					sequences[index][i+4]);		//Npts
				i+=6;
			}while(sequences[index][i]);			//Always end with a row of zeros or this won't break

		} else {
			pr_info("Invalid sequence number. Values are 1-%d\n", SEQ_COUNT); 
			return -EINVAL; //may need to pick something else since kstrtouint could return this too
		}
	} else {		//else kstrtouint returned some error
		pr_info("Got: %s\n", c);
		if(ret == -EINVAL)
			pr_info("kstrtouint returned parse error\n");
		if(ret == -ERANGE)
			pr_info("kstrtouint returned overflow\n");
		//TODO parse the file
		//TODO file parse might be spun off in to a different function
	}

	mutex_unlock(&buzzer.mutex);
	return len;
}

static int buzzer_release(struct inode *i, struct file *f){
	//nothing to do here?
	return 0;
}


static void synth(uint16_t dur, uint16_t freq, uint16_t wave, uint16_t npts){
	//Dur in msec, freq in Hz, per = 1/freq, wave and pts are dimensionless
	//wave could be a complex binary value, but is 1 for all seqs currently
	//npts effectively sets the modulation rate of the wave. 
	//Assume Wave = 1
	// npts = 2 -> 50% duty cycle
	// npts = 4 -> 25% duty cycle

		/*CAUTION KERNEL DOES NOT LIKE FLOATING POINT MATH*/

//FIXME Currently uses spinlock, but spends a lot of time waiting for the next event time
//FIXME Would like to make this sleep for some time, but needs to have gauranteed wake up time
	long long start_time, curr_time, end_time, next_event, us_per;
	uint8_t mod_index;
	unsigned long flags;
	spin_lock_irqsave(&buzzer.spin_lock, flags);		// spinlock works, but it probably chews up system process
	start_time = ktime_to_us(ktime_get());			// get the current time
	curr_time = start_time;					// current time
//FIXME change dur to pass msec to remove this multiply
	end_time = start_time + (dur*1000);			// calc the end time (in usec)
	us_per = 1000000 / (npts*freq);				// Event period: when pin might change next (in us)

	//Freq log: ln(f2/f1), only needed if there's a slide
	//event frequency: 1/(end-start), only needed if there's a slide
	mod_index = 0;						// Used for modulation
	next_event = curr_time+(us_per);			// calculate the first
//FIXME make this usleep_range or something that doesn't hold the kernel
	while(curr_time < end_time){				// check if we should still run the loop
		if(curr_time >= next_event){			// check if the next event should happen
			mod_index = (mod_index+1)%npts;		// increment our mod index
			gpio_set_value(BUZZER_OUT, (wave>>mod_index)&1);	//set the output
			next_event = curr_time+(us_per);	// calc next event time

		}
		curr_time = ktime_to_us(ktime_get());
	}

	gpio_set_value(BUZZER_OUT, 0);				//make sure the pin is set low when we exit
	spin_unlock_irqrestore(&buzzer.spin_lock, flags);	//release the lock
}

//TODO parse MIDI file to synth

static const struct file_operations buzzer_fops = {
	.owner = 	THIS_MODULE,
	.open = 	buzzer_open,		//called on first operation, may not need
	.read = 	buzzer_read,		//read from the device? possibly the currently loaded sequence
	.release = 	buzzer_release,		//called when all versions are done, may not need
	.write = 	buzzer_write,		//write data to the device
	//TODO IOCTRL...maybe
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
	mutex_init(&buzzer.mutex);		//Mutex is used to lock the write operation
	spin_lock_init(&buzzer.spin_lock);	//spin lock is used to lock the synth
	ret = buzzer_dev_config();		//changed the name of this to avoid confusion
	if(ret !=0){
		pr_err("Buzzer Device Init failed\n");
		return -1;
	}


//TODO init low level stuff here

	//Assign defaults
	//TODO need set and get methods for this
//FIXME these don't get used, can probably remove and remove from struct
	buzzer.index = 1;	//
	buzzer.freq = 440;	//Hz
	buzzer.duration = 50;	//msec? sure
	buzzer.polywave = 1;	//good enough

	return 0;
}

static void __exit buzzer_exit(void){
	buzzer_dev_cleanup();
}
module_init(buzzer_init);
module_exit(buzzer_exit);

MODULE_AUTHOR("Matt Sterling");
MODULE_DESCRIPTION("Makerbot Buzzer Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.6");
