#ifndef BUZZER_HH
#define BUZZER_HH

struct buzzer_platform_data{
	unsigned int sample_rate;	//rate which data is output to buzzer (sample rate)
	unsigned int ctrl_rate;		//rate which control events happen (env, glide, etc)
	unsigned int seq_rate;		//time base for the sequencer (PPQ probably)
	u16 output_pin;			//GPIO output pin
};

#endif
