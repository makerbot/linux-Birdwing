/*
*	Note frequency defines and MIDI note numbers
*
*/

//Note to Frequncy
#define C0	16.35
#define DB0	17.32
#define D0	18.35
#define EB0	19.45
#define F0	21.83
#define GB0	23.12
#define G0	24.50
#define AB0	25.96
#define A0	27.50
#define BB0	29.14

#define C1	32.70
#define DB1	34.65
#define D1	36.71
#define EB1	38.89
#define E1	41.20
#define F1	43.65
#define	GB1	46.25
#define G1	49.00
#define	AB1	51.91
#define A1	55.00
#define BB1	58.27
#define B1	61.74

#define C2	65.41
#define DB2	69.30
#define D2	73.42
#define EB2	77.78
#define F2	82.41
#define GB2	92.50
#define G2	98.00
#define AB2	103.83
#define A2	110.00
#define BB2	116.54
#define B2	123.47

#define C3	130.81
#define DB3	138.59
#define D3	146.83
#define EB3	155.56
#define E3	164.81
#define F3	174.61
#define GB3	185.00
#define G3	196.00
#define AB3	207.65
#define A3	220.00
#define BB3	233.08
#define B3	246.94

#define C4	261.63
#define DB4	277.18
#define D4	293.66
#define EB4	311.13
#define E4	329.63
#define F4	349.23
#define GB4	369.99
#define G4	392.00
#define AB4	415.30
#define A4	440.00
#define BB4	466.16
#define B4	493.88

#define C5	523.25
#define DB5	554.37
#define D5	587.33
#define EB5	622.25
#define E5	659.26
#define F5	698.46
#define GB5	739.99
#define G5	830.61
#define AB5	880.00
#define A5	932.33
#define BB5	932.33
#define B5	987.77

#define C6	1046.50
#define DB6	1108.73
#define D6	1174.66
#define EB6	1244.51
#define E6	1318.51
#define F6	1396.91
#define GB6	1479.98
#define G6	1567.98
#define AB6	1661.22
#define A6	1760.00
#define BB6	1864.66
#define B6	1975.53

#define C7	2093.00
#define DB7	2217.46
#define D7	2349.32
#define EB7	2489.02
#define E7	2637.02
#define F7	2793.83
#define GB7	2956.96
#define G7	3135.96
#define AB7	3322.44
#define A7	3520.00
#define BB7	3729.31
#define B7	3951.07

#define C8	4186.01

//Look up table for MIDI numbers
//TODO need to fill these in
int midi_notes[128] ={
	0,		//0,   C-1
	0,		//1,   C#-1
	0,		//2,   D-1
	0,		//3,   D#-1
	0,		//4,   E-1
	0,		//5,   F-1
	0,		//6,   F#-1
	0, 		//7,   G-1
	0, 		//8,   G#-1
	0,		//9,   A-1
	0,		//10,  A#-1
	0,		//11,  B-1
	0,		//12,  C0
	0,		//13,  C#0
	0,		//14,  D0
	0,		//15,  D#0
	0,		//16,  E0
	0,		//17,  F0
	0,		//18,  F#0
	0,		//19,  G0
	0,		//20,  G#0
	0,		//21,  A0
	0,		//22,  A#0
	0,		//23,  B0
	0,		//24,  C1
	0,		//25,  C#1
	0,		//26,  D1
	0,		//27,  D#1
	0, 		//28,  E1
	0,		//29,  F1
	0, 		//30,  F#1
	0, 		//31,  G1
	0,		//32,  G#1
	0,		//33,  A1
	0,		//34,  A#1
	0,		//35,  B1
	0,		//36,  C2
	0,		//37,  C#2
	0,		//38,  D2
	0, 		//39,  D#2
	0,		//40,  E2
	0,		//41,  F2
	0,		//42,  F#2
	0,		//43,  G2
	0,		//44,  G#2
	0,		//45,  A2
	0,		//46,  A#2
	0,		//47,  B2
	0,		//48,  C3
	0,		//49,  C#3
	0,		//50,  D3
	0,		//51,  D#3
	0,		//52,  E3
	0,		//53,  F3
	0, 		//54,  F#3
	0,		//55,  G3
	0,		//56,  G#3
	0,		//57,  A3
	0,		//58,  A#3
	0,		//59,  B3
	0,		//60,  C4
	0, 		//61,  C#4
	0,		//62,  D4
	0,		//63,  D#4
	0,		//64,  E4
	0,		//65,  F4
	0,		//66,  F#4
	0,		//67,  G4
	0,		//68,  G#4
	0,		//69,  A4
	0,		//70,  A#4
	0,		//71,  B4
	0,		//72,  C5
	0,		//73,  C#5
	0,		//74,  D5
	0,		//75,  D#5
	0, 		//76,  E5
	0,		//77,  F5
	0,		//78,  F#5
	0,		//79,  G5
	0,		//80,  G#5
	0,		//81,  A5
	0,		//82,  A#5
	0,		//83,  B5
	0,		//84,  C6
	0,		//85,  C#6
	0,		//86,  D6
	0,		//87,  D#6
	0,		//88,  E6
	0,		//89,  F6
	0,		//90,  F#6
	0,		//91,  G6
	0,		//92,  G#6
	0,		//93,  A6
	0,		//94,  A#6
	0,		//95,  B6
	0,		//96,  C7
	0,		//97,  C#7
	0,		//98,  D7
	0,		//99,  D#7
	0,		//100, E7
	0,		//101, F7
	0,		//102, F#7
	0,		//103, G7
	0,		//104, G#7
	0,		//105, A7
	0,		//106, A#7
	0,		//107, B7
	0,		//108, C8
	0,		//109, C#8
	0,		//110, D8
	0,		//111, D#8
	0,		//112, E8
	0,		//113, F8
	0,		//114, F#8
	0,		//115, G8
	0,		//116, G#8
	0,		//117, A8
	0,		//118, A#8
	0,		//119, B8
	0,		//120, C9
	0,		//121, C#9
	0,		//122, D9
	0,		//123, D#9
	0,		//124, E9
	0,		//125, F9
	0,		//126, F#9
	0		//127, G9
};
