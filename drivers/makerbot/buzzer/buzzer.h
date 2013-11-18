//	Header file for buzzer driver
//	MSS 8 Nov 2013

#ifndef _BUZZER_H_
#define _BUZZER_H_

#include <linux/makerbot/buzzer.h>

//Defines
#ifndef BUZZER_MAJOR
#define BUZZER_MAJOR 0		//TODO change this to 14 (sound) later
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

//Structs
struct buzzer_dev{
	//Universal device/driver bits
	dev_t devt;				//device type
	struct cdev cdev;			//character driver
	struct mutex mutex;			//mutual exclusion semaphore
	struct device *device;			//hmm, what is this?
	struct buzzer_platform_data *pdata;	//platform data
	//Buzzer specific 
	unsigned int index;			//index to be used with frequency
	unsigned int freq;			//current frequency (pitch)
	unsigned int duration;			//duration of the note (time)
	unsigned int polywave;			//"polywave", this changes the timber of the note

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

//Functions
static void synth(int, int);
//static void synth(u16, u16, u16, u32, u16);

#endif
