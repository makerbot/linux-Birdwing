//	Header file for buzzer driver
//	MSS 8 Nov 2013

//Includes


//Structs

//Functions
static int buzzer_open(struct);
static int buzzer_read(struct);
static void buzzer_write(struct);
static void buzzer_close(struct);

static void synth(u16, u16, u16, u32, u16);
