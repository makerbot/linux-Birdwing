/*
 * ti_magadc.h - Header file for magadc driver 
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __TI_MAGADC_H__
#define __TI_MAGADC_H__

#include <linux/types.h>
#include <linux/ioctl.h>
#include <stdbool.h>

#define MAGADC_IOCTL 0xAE
#define MAGADC_SIGID	50
#define MAGADC_SIGINT   99

#define MAGADC_STORE_RW_ADDR            _IOR(MAGADC_IOCTL, 0, int)
#define MAGADC_STORE_RO_ADDR            _IO(MAGADC_IOCTL, 1)
#define MAGADC_READ_RW_ADDR             _IO(MAGADC_IOCTL, 2)
#define MAGADC_READ_RO_ADDR             _IO(MAGADC_IOCTL, 3)
#define MAGADC_COPY_RAW_DATA            _IO(MAGADC_IOCTL, 4)


/* This denote raw data for each track from ADC */
#define MAX_TRACK_DATA			(30000)
#define MAX_NO_OF_TRACKS		(3)
#define MAX_TRACK_DATA_BITS		(1500)
#define MAX_TRACK_DATA_CHARS		(250)
#define MAX_NO_OF_PEAKS			(1500 * 2)
#define MAX_CHARS_REPORT		(100)
#define MAX_INPUT_EVENTS		(300)
/* Size of total Raw data collected from ADC */
#define MAX_ADC_DATA_RAW		(MAX_TRACK_DATA * MAX_NO_OF_TRACKS)

#define MAGADC_TRACK1			(2)
#define MAGADC_TRACK2			(1)
#define MAGADC_TRACK3			(0)
#define MAGADC_SWIPETHRES_VAL		(2200)
#define MAGADC_DMA0_THRESHOLD		(31)
#define MAGADC_ZERO_VAL			(2048)
#define MAGADC_TRACK1_OFFSET		(32)
#define MAGADC_TRACK2_OFFSET		(48)
#define MAGADC_TRACK1_BITPERCHAR	(6)
#define MAGADC_TRACK2_BITPERCHAR	(4)
#define MAGADC_IOCTL_REGISTER_PID       1
#define MAGADC_IOCTL_COPY_DATA          2

enum timag_thresdata {
	MAGADC_SWIPE_THRESDATA1 = 0,
	MAGADC_SWIPE_THRESDATA2,
	MAGADC_SWIPE_THRESDATA3,
	MAGADC_SWIPE_THRESDATA4
};
/*
 * F2F decoding support.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

struct peak_data {
	unsigned short	value;
	int	index;
};
struct timag_decode_data {
	unsigned char	asciiData[MAX_NO_OF_TRACKS][MAX_TRACK_DATA_CHARS];
};
struct timag_track_data {
	unsigned int		num_samples;
	unsigned int		num_peaks;
	unsigned int		num_bits;
	unsigned int		num_chars;
	unsigned int		badchars;
	unsigned short		rawsample[MAX_TRACK_DATA *
					sizeof(unsigned int)];
	unsigned char		bitdata[MAX_TRACK_DATA_BITS];
	struct peak_data	peakdata[MAX_NO_OF_PEAKS *
		                         sizeof(struct peak_data)];
};

struct bit_data_prop {
	int bitperchar;
	int bitcount;
	unsigned char offset;
	bool paritycheck;
	char endchar;
};

struct timag_track {
	struct timag_track_data	tracks[MAX_NO_OF_TRACKS];
	bool			sentinel_pass[MAX_NO_OF_TRACKS];
	struct bit_data_prop	bitprop[MAX_NO_OF_TRACKS];
	bool			is_swiped;
	unsigned int		adc_data[MAX_ADC_DATA_RAW * sizeof(int)];
	unsigned int		total_samples;
	unsigned int		num_tracks;
};


#endif /* __TI_MAGADC_H__ */
