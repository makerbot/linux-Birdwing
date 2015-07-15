#ifndef __LINUX_FT6X06_TS_H__
#define __LINUX_FT6X06_TS_H__
#include <linux/i2c.h>
/* -- dirver configure -- */
#define CFG_MAX_TOUCH_POINTS	2

#define PRESS_MAX	0xFF
#define FT_PRESS		0x7F

#define FT6X06_NAME 	"ft6x06_ts"

#define FT_MAX_ID	0x0F
#define FT_TOUCH_STEP	6
#define FT_TOUCH_X_H_POS		3
#define FT_TOUCH_X_L_POS		4
#define FT_TOUCH_Y_H_POS		5
#define FT_TOUCH_Y_L_POS		6
#define FT_TOUCH_EVENT_POS		3
#define FT_TOUCH_ID_POS			5

// Number of events in the chip event stack
#define POINT_READ_BUF	(8)

#define FT6x06_REG_RESET         0x01
#define FT6x06_REG_ID            0x02
#define FT6x06_REG_DRIVE_CNT     0x06
#define FT6x06_REG_SENSE_CNT     0x07
#define FT6x06_REG_PINCFG0       0x08
#define FT6x06_REG_PINCFG1       0x09
#define FT6x06_REG_PINCFG2       0x0A
#define FT6x06_REG_PINCFG3       0x0B
#define FT6x06_REG_PINCFG4       0x0C
#define FT6x06_REG_PINCFG5       0x0D
#define FT6x06_REG_PINCFG6       0x0E
#define FT6x06_REG_PINCFG7       0x0F
#define FT6x06_REG_PINCFG8       0x10
#define FT6x06_REG_PINCFG9       0x11
#define FT6x06_REG_PINCFG10      0x12
#define FT6x06_REG_PINCFG11      0x13
#define FT6x06_REG_PINCFG12      0x14
#define FT6x06_REG_PINCFG13      0x15
#define FT6x06_REG_PINCFG14      0x16
#define FT6x06_REG_PINCFG15      0x17
#define FT6x06_REG_PINCFG16      0x18
#define FT6x06_REG_PINCFG17      0x19
#define FT6x06_REG_PINCFG18      0x1A
#define FT6x06_REG_PINCFG19      0x1B
#define FT6x06_REG_PINCFG20      0x1C
#define FT6x06_REG_ENABLE        0x23
#define FT6x06_REG_DISABLE       0x24
#define FT6x06_REG_WRITE_MODE    0x25
#define FT6x06_REG_READ_MODE     0x26
#define FT6x06_REG_POWERDOWN     0x27
#define FT6x06_REG_POWERSAVE     0x28
#define FT6x06_REG_IDLECYCLES    0x29
#define FT6x06_REG_SUBFRAMES     0x2A
#define FT6x06_REG_CLKDOMAIN     0x2B
#define FT6x06_REG_MINFINGERAREA 0x33
#define FT6x06_REG_MINFINGERLVL  0x34
#define FT6x06_REG_MINFINGERWT   0x35
#define FT6x06_REG_MAXFINGERAREA 0x36
#define FT6x06_REG_MAXSEGDEPTH   0x37
#define FT6x06_REG_DELTARANGE    0x38
#define FT6x06_REG_CGMETHOD      0x39
#define FT6x06_REG_INITFILTER    0x3A
#define FT6x06_REG_DELTAFILTER   0x3D
#define FT6x06_REG_AUTOCAL       0x3E
#define FT6x06_REG_1CLICKTIME    0x51
#define FT6x06_REG_2CLICKTIME    0x52
#define FT6x06_REG_CGTOLERANCE   0x53
#define FT6x06_REG_XTOLERANCE    0x54
#define FT6x06_REG_YTOLERANCE    0x55
#define FT6x06_REG_AVGFILTEN     0x56
#define FT6x06_REG_SPEEDSCALE    0x57
#define FT6x06_REG_PRESSWTSCALE  0x58
#define FT6x06_REG_TOLENABLE     0x59
#define FT6x06_REG_MAXMISSFRAME  0x5A
#define FT6x06_REG_MOVETOLERANCE 0x5B
#define FT6x06_REG_COORDREMAP    0x65
#define FT6x06_REG_XCOORDSCALE   0x66
#define FT6x06_REG_YCOORDSCALE   0x67
#define FT6x06_REG_XCOORDOFFSET  0x68
#define FT6x06_REG_YCOORDOFFSET  0x69
#define FT6x06_REG_EVENTSTATUS   0x79
#define FT6x06_REG_EVENTMASK     0x7A
#define FT6x06_REG_IRQMASK       0x7B
#define FT6x06_REG_FINGER0DATA   0x7C
#define FT6x06_REG_FINGER1DATA   0x7D
#define FT6x06_REG_FINGER2DATA   0x7E
#define FT6x06_REG_FINGER3DATA   0x7F
#define FT6x06_REG_EVENTSTACK    0x80
#define FT6x06_REG_EVSTACKCLR    0x81
#define FT6x06_REG_RESETINIT     0xA2
#define FT6x06_REG_SELFCAPCNT    0xAB
#define FT6x06_REG_SELFCAPWT     0xAC
#define FT6x06_REG_SELFCAPSCAN   0xAD
#define FT6x06_REG_SELFCAPCHEN   0xAE
#define FT6x06_REG_SELFCAPTHRESH 0xAF
#define FT6x06_REG_SELFCAPSTATUS 0xB9
#define FT6x06_REG_SELFCAPIRQ    0xBA
#define FT6x06_REG_SELFCAPEN     0xBC
#define FT6x06_REG_BOOSTERCTL    0xC1
#define FT6x06_REG_DRIVELVL      0xD5
#define FT6x06_REG_SAMPLEDELAY   0xD8
#define FT6x06_REG_SENSEFILT     0xD9

#define FT6x06_STATUS_LARGEOBJ   (1<<6)
#define FT6x06_STATUS_OVERFLOW   (1<<5)
#define FT6x06_STATUS_NOTEMPTY   (1<<4)
#define FT6x06_STATUS_FINGER3    (1<<3)
#define FT6x06_STATUS_FINGER2    (1<<2)
#define FT6x06_STATUS_FINGER1    (1<<1)
#define FT6x06_STATUS_FINGER0    (1<<0)

int ft6x06_i2c_Read(struct i2c_client *client, char *writebuf, int writelen,
                    char *readbuf, int readlen);
int ft6x06_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);

/* The platform data for the Focaltech ft5x0x touchscreen driver */
struct ft6x06_platform_data {
	unsigned int x_max;
	unsigned int y_max;
	unsigned long irqflags;
	unsigned int irq;
	unsigned int reset;
};

#endif
