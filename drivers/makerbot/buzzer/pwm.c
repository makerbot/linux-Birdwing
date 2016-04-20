/*
*
*
*
*
*/

#include "pwm.h"

static inline unsigned int pwm_read_reg(struct davinci_pwm *p, int offset){
	offset <<= p->regshift;
	WARN_ONCE(!p->membase, "unmapped read: PWM[%d]\n", offset);
	return (unsigned int)__raw_readl(p->membase+offset);
}

static inline void pwm_write_reg(struct davinci_pwm *p, int offset, int value){
	offset <<=p->regshift;
	WARN_ONCE(!p->membase, "unmapped write: PWM[%d]\n", offset);
	__raw_writel(value, p->membase+offset);
}
