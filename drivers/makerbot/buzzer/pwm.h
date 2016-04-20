/*
*
*	Constants for the AM1808 PWM drivers
*
*	Copyright (c) 2014 Makerbot Industries LLC
*
*	Last updated: 20Aug2014
*
*=======Function	| Pin	|
*	ECAP0_APWM0	| F3	| eCap / Aux PWM out 0
*	ECAP1_APWM1	| E4	| eCap / Aux PWM out 1
*	ECAP2_APWM2	| A4	| eCap / Aux PWM out 2
*	EPWM0A		| D19	| EHRPWM0 A Output
*	EPWM0B		| C17	| eHRPWM0 B Output
*	EPWM0TZ[0]	| A4	| eHRPWM0 Trip Zone Input
*	EPWMSYNCI	| C16	| eHRPWM0 Sync Input
*	EPWMSYNCO	| C18	| eHRPWM0 Sync Output
*	EPWM1A		| F18	| eHRPWM1 A Output
*	EPWM1B		| E19	| eHRPWM1 B Output
*	EPWM1TZ[0]	| D2	| eHRPWM1 Trip Zone Input
*
*=======Interrupt	| Number
*	EHRPWM0		| 63
*	EHRPWM0TZ	| 64
*	EHRPWM1		| 65
*	EHRPWM1TZ	| 66
*	ECAP0		| 69
*	ECAP1		| 70
*	ECAP2		| 71
*
*/

#ifndef __PWM_H
#define __PWM_H

#define EHRPWM_0_BASE	0x01F00000
#define HRPWM_0_BASE	0x01F01000
#define EHRPWM_1_BASE	0x01F02000
#define HRPWM_1_BASE	0x01F03000
#define ECAP_0_BASE	0x01F06000
#define ECAP_1_BASE	0x01F07000
#define ECAP_2_BASE	0x01F08000

//Generic offsets for the ECAP registers
#define TSCTR	0x00
#define CTRPHS	0x04
#define CAP1	0x08
#define CAP2	0x0C
#define CAP3	0x10
#define CAP4	0x14
#define ECCTL1	0x28
#define ECCTL2	0x2A
#define ECEINT 	0x2C
#define ECFLG	0x2E
#define ECCLR	0x30
#define ECFRC	0x32
#define REVID	0x5C

//Generic offsets for the eHRPWM Registers
#define TBCLT	0x00
#define TBSTS	0x02
#define TBPHSHR	0x04
#define TBPHS	0x06
#define TBCNT	0x08
#define TBPRD	0x0A
#define CMPCTL	0x0E
#define CMPAHR	0x10
#define CMPA	0x12
#define CMPB	0x14
#define AQCTLA	0x16
#define AQCTLB	0x18
#define AQSFRC	0x1A
#define AQCSFRC	0x1C
#define DBCTL	0x1E
#define DBRED	0x20
#define DBFED	0x22
#define TZSEL	0x24
#define TZCTL	0x28
#define TZEINT	0x2A
#define TZFLG	0x2C
#define TZCLR	0x2E
#define TZFRC	0x30
#define ETSEL	0x32
#define ETPS	0x34
#define ETFLG	0x36
#define ETCLR	0x38
#define ETFRC	0x3A
#define PCCTL	0x3C
#define HRCNFG	0x1040

//Time-Base Control Register (TBCTL) Offset and length
//No len = length of 1
//TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
#define CTRMODE		0
#define CTRMODE_LEN	2
#define PHSEN		2
#define PRDLD		3
#define SYNCOSEL	4
#define SYNCOSEL_LEN	2
#define SWFSYNC		6
#define HSPCLKDIV	7
#define HSPCLKDIV_LEN	2
#define	CLKDIV		10
#define CLKDIV_LEN	3
#define PHSDIR		13
#define FREE_SOFT	14
#define FREE_SOFT_LEN	2

//Time-based Status Register (TBSTS) Offset and Length
#define CTRDIR		0
#define SYNCI		1
#define CTRMAX		2

//Time-based Phase Register (TBPHS)
//16-bit phase value
#define TBPHS		0

//Time-based Counter Register (TBCNT)
//16-bit counter value
#define TBCNT		0

//Time-based Period Register (TBPRD)
//16-bit period value
#define TBPRD		0


//Counter-compare Control Register (CMPCTL)
#define LOADAMODE	0
#define LOADAMODE_LEN	2
#define LOADBMODE	2
#define LOADBMODE_LEN	2
#define SHDWAMODE	4
#define SHDWBMODE	6
#define SHDWAFULL	8
#define SHDWBFULL	9

//Counter-compare A Register (CMPA)
//16-bit value
#define CMPA		0

//Counter-compare B Register (CMPB)
//16-bit value
#define CMPB		0

//Action-Qualifer Output A Control Register (AQCTLA)
#define ZRO		0
#define ZRO_LEN		2
#define PRD		2
#define PRD_LEN		2
#define CAU		4
#define CAU_LEN		2
#define CAD		6
#define CAD_LEN		2
#define CBU		8
#define CBU_LEN		2
#define CBD		10
#define CBD_LEN		2

//Action-Qualifer Output B Control Register (AQCTLB)
#define ZRO		0
#define ZRO_LEN		2
#define	PRD		2
#define PRD_LEN		2
#define CAU		4
#define CAU_LEN		2
#define CAD		6
#define CAD_LEN		2
#define CBU		8
#define CBU_LEN		2
#define CBD		10
#define CBD_LEN		2

//Action-Qualifer Software Force Register (AQSFRC)
#define ACTSFA		0
#define ACTSFA_LEN	2
#define OTSFA		2
#define ACTSFB		3
#define ACTSFB_LEN	2
#define OTSFB		5
#define RLDCSF		6
#define RLDCSF_LEN	2

//Action-Qualifier Continuous Software Force Register (AQCSFRC)
#define CSFA		0
#define CSFA_LEN	2
#define CSFB		2
#define CSFB_LEN	2

//Dead-band Generator Control Register (DBCTL)
#define OUT_MODE	0
#define OUT_MODE_LEN	2
#define POLSEL		2
#define POLSEL_LEN	2
#define IN_MODE		4
#define IN_MODE_LEN	2

//Dead-band generator rising edge delay register (DBRED)
//10-bit value
#define DEL		0

//Dead-band generator falling edge delay register (DBFED)
//10-bit value
#define DEL		0

//PWM-Chopper Control Register (PCCTL)
#define CHPEN		0
#define OSHTWTH		1
#define OSHTWTH_LEN	4
#define	CHPFREQ		5
#define CHPFREQ_LEN	2
#define CHPDUTY		8
#define CHPDUTY_LEN	2

//Trip-Zone Submodule Select Register (TZSEL)
#define CBC1		0
#define CBC2		1
#define CBC3		2
#define CBC4		3
#define CBC5		4
#define CBC6		5
#define CBC7		6
#define CBC8		7
#define OSHT1		8
#define OSHT2		9
#define OSHT3		10
#define OSHT4		11
#define OSHT5		12
#define OSHT6		13
#define OSHT7		14
#define OSHT8		15

//Trip-zone Control Register (TZCTL)
#define TZA		0
#define TZA_LEN		2
#define TZB		2
#define TZB_LEN		2


//Trip-zone enable interrupt register (TZEINT)
#define CBC		1
#define OST		2

//Trip-zone Flag Register (TZFLG)
#define INT		0
#define CBC		1
#define OST		2

//Trip-zone Clear Register (TZCLR)
#define INT		0
#define CBC		1
#define OST		2

//Trip-zone Force Register (TZFRC)
#define CBC		1
#define OST		2

//Event-triggered Seelection Register (ETSEL)
#define INTSEL		0
#define INTSEL_LEN	3
#define INTEN		3

//Event-Trigger Prescale Register (ETPS)
#define INTPRD		0
#define INTPRD_LEN	2
#define INTCNT		2
#define INTCNT_LEN	2

//Event-trigger Flag Register (ETFLG)
#define INT		0

//Event-trigger Clear Register (ETCLR)
#define INT		0

//Event-trigger Force Register (ETFRC)
#define INT		0

//Time-base Phase High-resolution Register (TBPHSHR)
#define TBPHSH		8
#define TBPHSH_LEN	8

//Counter-compare A High-resolution Register (CMPAHR)
#define CMPAHR		8
#define CMPAHR_LEN	8

//HRPWM Configuration register (HRCNFG)
#define EDGMODE		0
#define EDGMODE_LEN	2
#define CTLMODE		2
#define HRLOAD		3

struct davinci_pwm{
	unsigned long 	iobase;		/* io base address */
	void __iomem	*membase;	/*ioremap cookie or NULL*/
	resource_size_t	mapbase;	/*resource base*/
	unsigned int	irq;		/*interrupt number*/
	unsigned long 	irqflags;	/*request_irq flags*/
	unsigned char	regshift;	/*register shift*/
	unsigned int	pwm_clk;	/*PWM clock rate (period)*/
	unsigned int 	pwm_count;	/*PWM count, current count value*/
	unsigned int	pwm_compare;	/*PWM compare value, sets duty cycle*/
	bool		pwm_count_up;	/*counting up?*/

}

#endif
//__raw_readl();
//__raw_writel();
//__iomem *membase

