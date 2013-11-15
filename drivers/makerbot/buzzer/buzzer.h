//	Header file for buzzer driver
//	MSS 8 Nov 2013

#ifndef _BUZZER_H_
#define _BUZZER_H_

#include <linux/makerbot/buzzer.h>

//Defines
#ifndef BUZZER_MAJOR
#define BUZZER_MAJOR 0u		//TODO change this to 14 (sound) later
#endif

#ifndef BUZZER_MINOR
#define BUZZER_MINOR 64u	//64 should be unused, 1 is sequencer for sound major 
#endif

#ifndef BUZZER_COUNT
#define BUZZER_COUNT 1u		//incase we ever support more than 1
#endif

//TODO Number of devices? Should only be 1

//Structs
struct buzzer_dev{
	struct cdev cdev;			//character driver
	struct mutex buzzer_mutex;		//mutual exclusion semaphore
	struct buzzer_platform_data *pdata;	//platform data
	unsigned int index;			//index to be used with frequency
	unsigned int freq;			//current frequency
	unsigned int duration;			//duration of the note
	//array of notes (pitches)
	//array of tones (timbres)
	//dev_t buzzer_devt			//need this?
};

//Sequence struct?

struct buzzer_ops{
	int	(*open)(struct buzzer_dev *dev);
	int	(*read)(struct buzzer_dev *dev);
	void	(*write)(struct buzzer_dev *dev);
	void	(*release)(struct buzzer_dev *dev);
	struct module *owner;
};

//Functions
//static int buzzer_open(struct);
//static int buzzer_read(struct);
//static void buzzer_write(struct);
//static void buzzer_close(struct);

//static void synth(u16, u16, u16, u32, u16);

#endif
