/*
*	Buzzer Driver for Birdwing platform
*
*	Copyright (c) 2013-2014 Makerbot Industries LLC
*
*	Last updated: 3 Aug 2014
*/

#include <linux/init.h>			//init functions
#include <linux/module.h>		//dynamically loadable modules

#include <linux/kernel.h>		//printk,etc
#include <linux/fs.h>			//file operations
#include <linux/errno.h>		//errors
#include <linux/types.h>		//type defs, macros, etc
#include <linux/cdev.h>			//character device
#include <linux/device.h>		//class, devices, generic device model
#include <linux/gpio.h>			//gpio manipulation
#include <linux/mutex.h>		//mutexalicious

#include <asm/uaccess.h>		//copy to /from user space

#include "buzzer.h"
#include "sequences.h"

struct buzzer_dev buzzer;		//our main man
static struct class *buzzer_class;	//class seen by the kernel


static void buzzer_play_seq(struct work_struct *work){

	pr_debug("Play Seq %d at %d\n", buzzer.song, buzzer.index);
	return;
}

static void buzzer_synth(struct work_struct *work){
	pr_debug("synthy synthy\n");
	return;
}

static int buzzer_open(struct inode *i, struct file *f){
	pr_debug("Buzzer open\n");
	return 0;
}

static ssize_t buzzer_write(struct file *f, const char __user *buf, size_t len, loff_t *off){
	int ret;

	char c[len+1];					//buffer for reading characters
	unsigned int song_num;				//which song do you want?

	mutex_lock(&buzzer.mutex);			//one at a time now boys
	ret = 0;

	if(copy_from_user(&c, buf, len))		//read from user space
		return -EFAULT;				//something went wrong
	c[len] = 0;					//NULL termination for the array
	if(!(kstrtouint(c,0,&song_num))){		//0 = successful conversion
		pr_debug("Buzzer write %d\n", song_num);
		if(song_num<SEQ_COUNT){
			buzzer.song = song_num;		//which tune would you like to hear
			buzzer.index = 0;		//reset the position in the song
			schedule_delayed_work(&buzzer.seq_work, msecs_to_jiffies(20)); //toss it on the que
		} else {
			pr_err("Invalid sequence number. Valid range is 1-%d\n", SEQ_COUNT);
			return -ENXIO;
		}
	} else {					//else kstrtouint threw some error
		pr_err("Read %s from user\n", c);
		if(ret == -EINVAL)
			pr_err("Integer parse error\n");
		if(ret == -ERANGE)
			pr_err("Overflow Error\n");
	}
	mutex_unlock(&buzzer.mutex);

	return len;
}

static int buzzer_release(struct inode *i, struct file *f){
	pr_debug("Buzzer release\n");
	return 0;
}

static long buzzer_ioctl(struct file *file, unsigned int cmd, unsigned long arg){

	switch(cmd){
	case 0:
		pr_debug("IOCTL: 0\n");
		return 0;
	case 1:
		pr_debug("IOCTL: 1\n");
		return 1;
	default:
		pr_debug("IOCTL: wrong\n");
		return -ENOTTY;
	}
	return 0;

}

static const struct file_operations buzzer_fops = {
	.owner = THIS_MODULE,
	.open = buzzer_open,
	.release = buzzer_release,
	.write = buzzer_write,
	.unlocked_ioctl= buzzer_ioctl,
};

static int __init buzzer_dev_config(void){
	int ret;
	pr_debug("Buzzer Device Config\n");

	cdev_init(&buzzer.cdev, &buzzer_fops);
	buzzer.cdev.owner = THIS_MODULE;

	//buzzer.dev = MKDEV(BUZZER_MAJOR,BUZZER_MINOR)
	//maybe this should be 
	//ret = register_chrdev_region(MKDEV(BUZZER_MAJOR, 0), 1, "buzzer_cdev");
	//ret = cdev_add(&buzzer.cdev, buzzer.dev, 1);
	ret = register_chrdev(BUZZER_MAJOR, "buzzer_cdev", &buzzer_fops);
	if(ret){
		pr_err("Char Dev Registration Failed: %d\n", ret);
		return -1;
	}
	pr_debug("Registered with <%d, %d>\n", MAJOR(buzzer.dev), MINOR(buzzer.dev));

	pr_debug("Create Buzzer Class\n");
	if(!buzzer_class){
		buzzer_class= class_create(THIS_MODULE, "buzzer_class");
		if(!buzzer_class){
			pr_err("Buzzer class creation failed\n");
			return -2;
		}
	}
	pr_debug("Create buzzer device\n");
	buzzer.device = device_create(buzzer_class, NULL, buzzer.dev, NULL, "buzzer");
	if(!buzzer.device){
		pr_err("Buzzer device creation failed\n");
		return -3;
	}

	pr_debug("Request buzzer GPIO\n");
	ret = gpio_request_one(BUZZER_OUT, GPIOF_OUT_INIT_LOW, "buzzer_gpio");
	if(ret){
		pr_err("GPIO Request failed: %d\n", ret);
		return -4;
	}

	return 0;
//TODO this needs a bunch of gotos for cleaning up if something fails

}


static int __init buzzer_init(void){
	int ret;
	pr_info("Buzzer init\n");

	//TODO init mutex, spinlock, workqueues
	mutex_init(&buzzer.mutex);
	INIT_DELAYED_WORK(&buzzer.seq_work, buzzer_play_seq);
	ret = buzzer_dev_config();
	if(ret!=0){
		pr_err("Buzzer Config Failed\n");
		return -1;
	}

	//TODO init struct values: index, freq, duration, etc

	return 0;
}

static void __exit buzzer_exit(void){
	pr_debug("Buzzer exit\n");

	pr_debug("Destroy Buzzer Device\n");
	if(buzzer.device)
		device_destroy(buzzer_class, buzzer.dev);
	pr_debug("Destroy Buzzer Class\n");
	class_destroy(buzzer_class);
	pr_debug("Delete cdev\n");
	cdev_del(&buzzer.cdev);
	pr_debug("Unregister region\n");
	unregister_chrdev_region(buzzer.dev, 1);
}

module_init(buzzer_init);
module_exit(buzzer_exit);

MODULE_AUTHOR("Matt Sterling");
MODULE_DESCRIPTION("Makerbot Buzzer Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.8");

