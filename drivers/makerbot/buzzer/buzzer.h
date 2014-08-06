//	Header file for buzzer driver
//	MSS 30 Jul 2014

#ifndef _BUZZER_H_
#define _BUZZER_H_

#include <linux/makerbot/buzzer.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/sched.h>
//Defines
#ifndef BUZZER_MAJOR
#define BUZZER_MAJOR 14		//TODO change this to 14 (sound) later
#endif

#ifndef BUZZER_MINOR
#define BUZZER_MINOR 64		//64 should be unused, 1 is sequencer for sound major 
#endif

#ifndef BUZZER_COUNT
#define BUZZER_COUNT 1		//incase we ever support more than 1
#endif

#ifndef BUZZER_OUT
#define BUZZER_OUT 47 	//2, 15
#endif

#define GPIO_BASE 0x01E26000	//31612928
#define GPIO_OUT_OFFSET 0x40	//64
#define GPIO_OUT_BIT 0xF	//15
#define GPIO_ADDR 0x1E2604F	//Exact address for GPIO

//Functions
//static void synth(uint16_t, uint16_t, uint16_t, uint16_t);
//static void buzzer_play_seq(struct work_struct *);
//static void buzzer_synth_2(struct work_struct *);
//Structs
struct buzzer_dev{
	//Universal device/driver bits
	dev_t dev;				//device type, this is actually just a number
	//TODO add class here?
	struct cdev cdev;			//character driver
	struct mutex buzzer_lock_mutex;		//mutual exclusion semaphore
	//spinlock_t spin_lock;			//spinlock
	struct device *device;			//hmm, what is this?
	//struct buzzer_platform_data *pdata;	//platform data
	//Buzzer specific
	//synth params
	unsigned int freq;			//current frequency (pitch)
	unsigned int duration;			//duration of the note (time)
	unsigned int polywave;			//"polywave", this changes the timber of the note
	unsigned int npts;			//npts works with polywave to determine timbre
	//work structs / threads
	struct delayed_work seq_work;		//holds the scheduling work que
	struct delayed_work synth_work;		//synth work
	bool synth_running;
	struct kthread_worker kworker;
	struct task_struct *kworker_task;
	struct kthread_work synth;
	struct sched_param priority;
	//Song/sequence params
	unsigned int index;			//note number in the sequence
	unsigned int song;			//current song to play
	//array of notes (pitches)
	//array of tones (timbres)
};

//Sequence struct?

struct buzzer_ops{
	int	(*open)(struct buzzer_dev *dev);
	int	(*read)(struct buzzer_dev *dev);
	void	(*write)(struct buzzer_dev *dev);
	void	(*release)(struct buzzer_dev *dev);
	struct module *owner;
};

#endif
