/*
*
*	Auxilary synth/timing bits for buzzer
*
*	Copyright (c) 2014 Makerbot Industries LLC
*
*	Last Updated 3 Aug 2014
*/

#include <linux/workqueue.h>

#include "buzzer.h"
#include "synth.h"
//extern struct buzzer_dev buzzer;

static void buzzer_play_seq(struct work_struct *work){
	struct buzzer_dev *b;

	pr_debug("Play Seq %d at %d\n", b->song, b->index);

	return;
}

static void buzzer_synth(struct work_struct *work){

	pr_debug("Do something\n");

	return;
}
