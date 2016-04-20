//Header for synth part

#ifndef _SYNTH_H_
#define _SYNTH_H_

#include <linux/workqueue.h>
static void buzzer_play_seq(struct work_struct *work);
static void buzzer_synth(struct work_struct *work);

#endif
