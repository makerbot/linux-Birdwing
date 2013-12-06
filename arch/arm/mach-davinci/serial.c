/*
 * TI DaVinci serial driver
 *
 * Copyright (C) 2006 Texas Instruments.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <mach/serial.h>
#include <mach/cputype.h>

#define BAUD_RATE 14400

#define UART_MDR 0x34
#define UART_DLLO 0x20
#define UART_DLHI 0x24
#define UART_LCREG 0xC

#define UART_CONFIG 0x3
#define UART_OVR_SMPL 0x1
#define UART_STOP_BIT 0x4
#define UART_WORDL 0x3
#define UART_PARITY 0x8
#define UART_SET_PARITY 0x10
#define UART_STICK_PARITY 0x20
#define UART_BREAK_CTRL 0x40
#define UART_DLAB 0x80


static inline unsigned int serial_read_reg(struct plat_serial8250_port *up,
					   int offset)
{
	offset <<= up->regshift;

	WARN_ONCE(!up->membase, "unmapped read: uart[%d]\n", offset);

	return (unsigned int)__raw_readl(up->membase + offset);
}

static inline void serial_write_reg(struct plat_serial8250_port *p, int offset,
				    int value)
{
	offset <<= p->regshift;

	WARN_ONCE(!p->membase, "unmapped write: uart[%d]\n", offset);
pr_info("UART Write %d at %x\n", value, (unsigned int)(p->membase+offset));
	__raw_writel(value, p->membase + offset);
}

static void __init davinci_serial_reset(struct plat_serial8250_port *p)
{
	unsigned int pwremu = 0;

	unsigned int divisor = 0;

	serial_write_reg(p, UART_IER, 0);  /* disable all interrupts */

	/* reset both transmitter and receiver: bits 14,13 = UTRST, URRST */
	serial_write_reg(p, UART_DAVINCI_PWREMU, pwremu);
	mdelay(10);

	pwremu |= (0x3 << 13);
	pwremu |= 0x1;
	serial_write_reg(p, UART_DAVINCI_PWREMU, pwremu);

//	if(p->mapbase == 0x01D0D000){
//		pr_info("configuring UART2\n");
//		divisor = (p->uartclk)/(BAUD_RATE*16);
//
//		serial_write_reg(p, UART_MDR, 0);
//
//		serial_write_reg(p, UART_DLLO, (divisor & 0xFF));
//		serial_write_reg(p, UART_DLHI, ((divisor & 0xFF00)>>8));
//
//		serial_write_reg(p, UART_LCREG, UART_CONFIG & (UART_STOP_BIT | UART_WORDL |
//							UART_PARITY| UART_SET_PARITY |
//							UART_STICK_PARITY | UART_BREAK_CTRL |
//							UART_DLAB));
//	}

	if (cpu_is_davinci_dm646x())
		serial_write_reg(p, UART_DM646X_SCR,
				 UART_DM646X_SCR_TX_WATERMARK);
}

/* Enable UART clock and obtain its rate */
int __init davinci_serial_setup_clk(unsigned instance, unsigned int *rate)
{
	char name[16];
	struct clk *clk;
	struct davinci_soc_info *soc_info = &davinci_soc_info;
	struct device *dev = &soc_info->serial_dev->dev;


pr_err("==============>>UART CLK<<================\n");
pr_err("UART%d\n", instance);
	sprintf(name, "uart%d", instance);
	clk = clk_get(dev, name);
	if (IS_ERR(clk)) {
		pr_err("%s:%d: failed to get UART%d clock\n",
					__func__, __LINE__, instance);
		return PTR_ERR(clk);
	}

	clk_prepare_enable(clk);

	if (rate){
		*rate = clk_get_rate(clk);
pr_err("UART Rate %u\n", *rate);

	}

	return 0;
}

int __init davinci_serial_init(struct davinci_uart_config *info)
{
	int i, ret;
	struct davinci_soc_info *soc_info = &davinci_soc_info;
	struct device *dev = &soc_info->serial_dev->dev;
	struct plat_serial8250_port *p = dev->platform_data;



	/*
	 * Make sure the serial ports are muxed on at this point.
	 * You have to mux them off in device drivers later on if not needed.
	 */
pr_err("==============>>UART<<================\n");
	
	for (i = 0; p->flags; i++, p++) {
		if (!(info->enabled_uarts & (1 << i)))
			continue;

		ret = davinci_serial_setup_clk(i, &p->uartclk);
		if (ret)
			continue;

		if (!p->membase && p->mapbase) {
pr_err("UART Mapbase: %lu\n", (unsigned long) p->mapbase);
			p->membase = ioremap(p->mapbase, SZ_4K);
pr_err("UART Membase: %lu\n", (unsigned long)p->membase);
			if (p->membase)
				p->flags &= ~UPF_IOREMAP;
			else
				pr_err("uart regs ioremap failed\n");
		}
pr_err("UART Flags: %x\n", p->flags);

		if (p->membase && p->type != PORT_AR7)
			davinci_serial_reset(p);
	}

	return platform_device_register(soc_info->serial_dev);
}
