/*
*	Buzzer Driver for Birdwing platform
*
*	Copyright (c) 2013-2014 Makerbot Industries LLC
*
*	Last updated: 6 Aug 2014
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
#include <linux/kthread.h>		//worker threads
#include <linux/sched.h>		//scheduler
#include <linux/delay.h>		//sleep zzz
#include <linux/time.h>			//time functions

#include <asm/uaccess.h>		//copy to /from user space

#include "buzzer.h"
#include "sequences.h"

#define BUZZER_VERSION 32

struct buzzer_dev buzzer;		//our main man
static struct class *buzzer_class;	//class seen by the kernel

int toggle_pin(void *data){
	//unsigned int errno;
	struct timespec deadline;
	unsigned int* synth_p = (unsigned int*)data;
	unsigned int countdown=  synth_p[0];
	unsigned int dur = synth_p[1];
	unsigned long dur_ns = dur*1000;
	ktime_get_ts(&deadline);

	while(countdown){
		deadline.tv_nsec += dur_ns;				//calculate next value
		deadline.tv_sec += deadline.tv_nsec / NSEC_PER_SEC;	//account for wrapping
		deadline.tv_nsec %= NSEC_PER_SEC;			//reset this value
		gpio_set_value(BUZZER_OUT, buzzer.pin_state);
		countdown--;
		if(buzzer.pin_state)
			buzzer.pin_state = 0;
		else
			buzzer.pin_state = 1;
		//sleep for a few ns
		while(hrtimer_nanosleep(&deadline,NULL,HRTIMER_MODE_ABS,CLOCK_MONOTONIC)!=0)

		if(kthread_should_stop()){
			return 0;
		}
	}
	buzzer.synth_running = 0;
	return 0;
}

static void buzzer_play_seq(struct work_struct *work){
	struct buzzer_dev *b = container_of(
				container_of(work, struct delayed_work, work),
				struct buzzer_dev,
				seq_work);
	unsigned int synth_params[2];
	struct sched_param priority = { .sched_priority = MAX_RT_PRIO - 1 };

	pr_info("Play Seq %d at %d\n", b->song, b->index);

	//if there's still notes in the song, schedule the next one
	if(sequences[b->song][b->index+6])
		schedule_delayed_work(&b->seq_work, msecs_to_jiffies(sequences[b->song][b->index]));

	//update the synth params
	buzzer.duration = sequences[buzzer.song][buzzer.index];		//dur in ms
	buzzer.freq = sequences[buzzer.song][buzzer.index+1];		//freq in hz
	//buzzer.polywave = sequences[buzzer.song][buzzer.index+3];	//2 is end freq
	//buzzer.npts = sequences[buzzer.song][buzzer.index+4];
	buzzer.countdown = ((buzzer.freq*buzzer.duration)/1000)*2;
	buzzer.pin_state = 0;
	buzzer.event_dur_usec = ((unsigned long)1000000/(unsigned long)buzzer.freq)>>1;

	synth_params[0] = (unsigned int)buzzer.countdown;
	synth_params[1] = (unsigned int)buzzer.event_dur_usec;
//set up the worker thread
//Check if task is valid / still around
	//pr_info("Kworker: %d\n",task_nice(b->kworker_task));
	if(!buzzer.synth_running){
	b->kworker_task = kthread_run(toggle_pin,
					&synth_params,
					"synth");
	if(IS_ERR(b->kworker_task)){
		pr_err("Cannot create synth task\n");
		return;
	}
	buzzer.synth_running = 1;
	sched_setscheduler(b->kworker_task, SCHED_FIFO, &priority);
	} else {
		pr_info("nope synth is synthing\n");
	}
	buzzer.index+=6;

	return;
}

static int buzzer_open(struct inode *i, struct file *f){
	pr_info("Buzzer open\n");
	return 0;
}

static ssize_t buzzer_write(struct file *f, const char __user *buf, size_t len, loff_t *off){
	int ret;

	char c[len+1];					//buffer for reading characters
	unsigned int song_num;				//which song do you want?

	mutex_lock(&buzzer.buzzer_lock_mutex);			//one at a time now boys
	ret = 0;

	if(copy_from_user(&c, buf, len)){		//read from user space
		ret = -EFAULT;
		goto error;
	}

	c[len] = 0;					//NULL termination for the array
	if(!(kstrtouint(c,0,&song_num))){		//0 = successful conversion
		pr_info("Buzzer write %d\n", song_num);
		if(song_num<SEQ_COUNT){
			buzzer.song = song_num;		//which tune would you like to hear
			buzzer.index = 0;		//reset the position in the song
			buzzer.pin_state = 0;
			schedule_delayed_work(&buzzer.seq_work, msecs_to_jiffies(20)); //toss it on the que
		} else {
			pr_err("Invalid sequence number. Valid range is 1-%d\n", (SEQ_COUNT-1));
			ret= -EBADF;
			goto error;
		}
	} else {					//else kstrtouint threw some error
		pr_err("Read %s from user\n", c);
		if(ret == -EINVAL)
			pr_err("Integer parse error\n");
		if(ret == -ERANGE)
			pr_err("Overflow Error\n");
		goto error;
	}
	mutex_unlock(&buzzer.buzzer_lock_mutex);

	return len;

error:
	mutex_unlock(&buzzer.buzzer_lock_mutex);
	return ret;
}

static int buzzer_release(struct inode *i, struct file *f){
	pr_info("Buzzer release\n");
	return 0;
}

static long buzzer_ioctl(struct file *file, unsigned int cmd, unsigned long arg){


	switch(cmd){
	case 0:
		pr_info("IOCTL: 0\n");
		return 0;
	case 1:
		pr_info("IOCTL: 1\n");
		return 1;
	default:
		pr_info("IOCTL: wrong\n");
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
	pr_info("Buzzer Device Config\n");

	buzzer.dev = MKDEV(BUZZER_MAJOR,BUZZER_MINOR);
	ret = register_chrdev_region(buzzer.dev, 1, "buzzer_cdev");
	//ret = register_chrdev(BUZZER_MAJOR, "buzzer_cdev", &buzzer_fops);
	if(ret){
		pr_err("Char Dev Registration Failed: %d\n", ret);
		return ret;
	}
	pr_info("Registered with <%d, %d>\n", MAJOR(buzzer.dev), MINOR(buzzer.dev));

	cdev_init(&buzzer.cdev, &buzzer_fops);
	buzzer.cdev.owner = THIS_MODULE;

	ret = cdev_add(&buzzer.cdev, buzzer.dev, 1);
	if(ret){
		pr_err("Cdev Add Failed: %d\n", ret);
		return ret;
	}

	pr_info("Create Buzzer Class\n");
	if(!buzzer_class){
		buzzer_class= class_create(THIS_MODULE, "buzzer_class");
		if(!buzzer_class){
			pr_err("Buzzer class creation failed\n");
			return -2;
		}
	}

	pr_info("Create buzzer device\n");
	buzzer.device = device_create(buzzer_class, NULL, buzzer.dev, NULL, "buzzer");
	if(!buzzer.device){
		pr_err("Buzzer device creation failed\n");
		return -3;
	}

	pr_info("Request buzzer GPIO\n");
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
	pr_info("Buzzer v %d\n", BUZZER_VERSION);

	mutex_init(&buzzer.buzzer_lock_mutex);
	//spin_lock_init(&buzzer.spin_lock);
	INIT_DELAYED_WORK(&buzzer.seq_work, buzzer_play_seq);
	ret = buzzer_dev_config();
	if(ret!=0){
		pr_err("Buzzer Config Failed\n");
		return -1;
	}

	buzzer.index = 0;
	buzzer.freq = 440;
	buzzer.duration = 2000;
	buzzer.polywave = 1;
	buzzer.npts = 2;
	return 0;
}

static void __exit buzzer_exit(void){
	pr_info("Buzzer exit\n");

	pr_info("Free GPIO\n");
	gpio_free(BUZZER_OUT);
	pr_info("Destroy Buzzer Device\n");
	device_destroy(buzzer_class, buzzer.dev);
	pr_info("Destroy Buzzer Class\n");
	class_destroy(buzzer_class);
	pr_info("Delete cdev\n");
	cdev_del(&buzzer.cdev);
	pr_info("Unregister region\n");
	unregister_chrdev_region(buzzer.dev, 1);
}

module_init(buzzer_init);
module_exit(buzzer_exit);

MODULE_AUTHOR("Matt Sterling");
MODULE_DESCRIPTION("Makerbot Buzzer Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.8");

