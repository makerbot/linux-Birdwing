/*
 * Copyright (C) 2008-2009 MontaVista Software Inc.
 * Copyright (C) 2008-2009 Texas Instruments Inc
 *
 * Based on the LCD driver for TI Avalanche processors written by
 * Ajay Singh and Shalom Hai.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option)any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/console.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/lcm.h>
#include <video/da8xx-fb.h>
#include <asm/div64.h>

/* this Driver has been modified to work with the solomon systech ssd2119 LCD module in LIDD mode */
#define DRIVER_NAME "da8xx_lcdc_lidd"

#define LCD_VERSION_1	1
#define LCD_VERSION_2	2

/* LCD Status Register */
#define LCD_END_OF_FRAME1		BIT(9)
#define LCD_END_OF_FRAME0		BIT(8)
#define LCD_PL_LOAD_DONE		BIT(6)
#define LCD_FIFO_UNDERFLOW		BIT(5)
#define LCD_SYNC_LOST			BIT(2)
#define LCD_FRAME_DONE			BIT(0)

/* LIDD_DMA_EN bit must be disabled while changing any of the LIDD CNTRL bits */
/* LIDD CNTRL */
#define LIDD_FRAME_DONE_INTERRUPT BIT(10)
#define LIDD_DMA_CS_SELECT        BIT(9)
#define LIDD_DMA_ENABLE           BIT(8)
#define LIDD_CS1_POLARITY         BIT(7)
#define LIDD_CS0_POLARITY         BIT(6)
#define LIDD_WS_DIR_POLARITY      BIT(5)
#define LIDD_RS_EN_POLARITY       BIT(4)
#define LIDD_RS_POLARITY          BIT(3)
#define LIDD_ASYN_8080_MODE         0x3

/* LIDD_CONF_REG  */
#define LIDD_WRITE_SETUP(x)     ((x) << 27)
#define LIDD_WRITE_DURATION(x)  ((x) << 21)
#define LIDD_WRITE_HOLD(x)      ((x) << 17)
#define LIDD_READ_SETUP(x)      ((x) << 12)
#define LIDD_READ_DURATION(x)   ((x) << 6)
#define LIDD_READ_HOLD(x)       ((x) << 2)
#define LIDD_OPERATION_GAP(x)   ((x) << 0)

/* LCD DMA Control Register */
#define LCD_DMA_BURST_SIZE(x)		((x) << 4)
#define LCD_DMA_BURST_1			0x0
#define LCD_DMA_BURST_2			0x1
#define LCD_DMA_BURST_4			0x2
#define LCD_DMA_BURST_8			0x3
#define LCD_DMA_BURST_16		0x4
#define LCD_V1_END_OF_FRAME_INT_ENA	BIT(2)
#define LCD_V2_END_OF_FRAME0_INT_ENA	BIT(8)
#define LCD_V2_END_OF_FRAME1_INT_ENA	BIT(9)
#define LCD_DUAL_FRAME_BUFFER_ENABLE	BIT(0)

/* unused? raster interrupts */
#define LCD_V1_PL_INT_ENA		BIT(4)
#define PALETTE_SIZE	256

/* LCD Control Register */
#define LCD_CLK_DIVISOR(x)		((x) << 8)
#define LCD_RASTER_MODE			0x01
#define LCD_LIDD_MODE           0x00

/* LCD Block */
#define  LCD_PID_REG				0x0
#define  LCD_CTRL_REG				0x4
#define  LCD_STAT_REG				0x8
#define  LCD_LIDD_CTRL_REG          0xC
#define  LCD_LIDD_CS0_CONF          0x10
#define  LCD_LIDD_CS1_CONF          0x14
#define  LCD_LIDD_CS0_ADDR          0x18
#define  LCD_LIDD_CS1_ADDR          0x1C
#define  LCD_LIDD_CS0_DATA          0x20
#define  LCD_LIDD_CS1_DATA          0x24

#define  LCD_RASTER_CTRL_REG			0x28
#define  LCD_RASTER_TIMING_0_REG		0x2C
#define  LCD_RASTER_TIMING_1_REG		0x30
#define  LCD_RASTER_TIMING_2_REG		0x34
#define  LCD_DMA_CTRL_REG			0x40
#define  LCD_DMA_FRM_BUF_BASE_ADDR_0_REG	0x44
#define  LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG	0x48
#define  LCD_DMA_FRM_BUF_BASE_ADDR_1_REG	0x4C
#define  LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG	0x50

#define LCD_NUM_BUFFERS	2


static void __iomem *da8xx_fb_reg_base;
static struct resource *lcdc_regs;
static unsigned int lcd_revision;
static irq_handler_t lcdc_irq_handler;
static wait_queue_head_t frame_done_wq;
static int frame_done_flag;

static inline unsigned int lcdc_read(unsigned int addr)
{
	return (unsigned int)__raw_readl(da8xx_fb_reg_base + (addr));
}

static inline void lcdc_write(unsigned int val, unsigned int addr)
{
	__raw_writel(val, da8xx_fb_reg_base + (addr));
}

struct da8xx_fb_par {
	resource_size_t p_palette_base;
	unsigned char *v_palette_base;
	dma_addr_t		vram_phys;
	unsigned long		vram_size;
	void			*vram_virt;
	unsigned int		dma_start;
	unsigned int		dma_end;
	struct clk *lcdc_clk;
	int irq;
	unsigned int palette_sz;
	unsigned int pxl_clk;
	int blank;
	wait_queue_head_t	vsync_wait;
	int			vsync_flag;
	int			vsync_timeout;
	spinlock_t		lock_for_chan_update;

	/*
	 * LCDC has 2 ping pong DMA channels, channel 0
	 * and channel 1.
	 */
	unsigned int		which_dma_channel_done;
#ifdef CONFIG_CPU_FREQ
	struct notifier_block	freq_transition;
	unsigned int		lcd_fck_rate;
#endif
	void (*panel_power_ctrl)(int);
	u32 pseudo_palette[16];
};

static struct fb_videomode known_lcd_panels[] = {
   [0] = {
		/* Solomon Systech SSD2119 Driver + UMSH-8252MD-T LCD Module */
		.name           = "SSD2119",
		.xres           = 320,
		.yres           = 240,
		.pixclock       = 7833600,
		.sync           = 0,
		.flag           = 0,
	},
};

/* Variable Screen Information */
static struct fb_var_screeninfo da8xx_fb_var = {
	.xoffset = 0,
	.yoffset = 0,
	.transp = {0, 0, 0},
	.nonstd = 0,
	.activate = 0,
	.height = -1,
	.width = -1,
	.accel_flags = 0,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED
};



static struct fb_fix_screeninfo da8xx_fb_fix = {
	.id = "DA8xx FB Drv",
	.type = FB_TYPE_PACKED_PIXELS,
	.type_aux = 0,
	.visual = FB_VISUAL_PSEUDOCOLOR,
	.xpanstep = 0,
	.ypanstep = 1,
	.ywrapstep = 0,
	.accel = FB_ACCEL_NONE
};

/* Enable the LCD Controller */
static inline void lcd_enable(void)
{
    u32 lidd_reg;

    // disable the DMA
    lidd_reg = lcdc_read(LCD_LIDD_CTRL_REG);
    lidd_reg |= LIDD_DMA_ENABLE;
    lcdc_write(lidd_reg, LCD_LIDD_CTRL_REG);        
    return;
}

/* Disable the LCD Controller */
static inline void lcd_disable(bool wait_for_frame_done)
{
  u32 lidd_reg;
  int ret;

  // disable the DMA
  lidd_reg = lcdc_read(LCD_LIDD_CTRL_REG);
  lidd_reg &= ~LIDD_DMA_ENABLE;
  lcdc_write(lidd_reg, LCD_LIDD_CTRL_REG);        

	if ((wait_for_frame_done == true) && (lcd_revision == LCD_VERSION_2)) {
		frame_done_flag = 0;
		ret = wait_event_interruptible_timeout(frame_done_wq,
				frame_done_flag != 0,
				msecs_to_jiffies(50));
		if (ret == 0)
			pr_err("LCD Controller timed out\n");
	}
    return;
}

static void lcd_blit(int load_mode, struct da8xx_fb_par *par)
{
	u32 start;
	u32 end;
	u32 reg_dma;

	/* init reg to clear PLM (loading mode) fields */
	//reg_ras = lcdc_read(LCD_RASTER_CTRL_REG);
	//reg_ras &= ~(3 << 20);

  pr_info("LCD_BLIT!!\n");
	reg_dma  = lcdc_read(LCD_DMA_CTRL_REG);

	if (load_mode == LOAD_DATA) {
		start    = par->dma_start;
		end      = par->dma_end;

		//reg_ras |= LCD_PALETTE_LOAD_MODE(DATA_ONLY);
        reg_dma |= LCD_V1_END_OF_FRAME_INT_ENA;
		reg_dma |= LCD_DUAL_FRAME_BUFFER_ENABLE;

		lcdc_write(start, LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
		lcdc_write(end, LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
		lcdc_write(start, LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
		lcdc_write(end, LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);
	} else if (load_mode == LOAD_PALETTE) {
		start    = par->p_palette_base;
		end      = start + par->palette_sz - 1;

		//reg_ras |= LCD_PALETTE_LOAD_MODE(PALETTE_ONLY);

        //reg_ras |= LCD_V1_PL_INT_ENA;

		lcdc_write(start, LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
		lcdc_write(end, LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
	}

	lcdc_write(reg_dma, LCD_DMA_CTRL_REG);
	//lcdc_write(reg_ras, LCD_RASTER_CTRL_REG);

	/*
	 * The DMA enable bit must be set after all other control fields are
	 * set.
	 */
	lcd_enable();
}

/* Configure the Burst Size and fifo threhold of DMA */
static int lcd_cfg_dma(int burst_size, int fifo_th)
{
	u32 reg;

	reg = lcdc_read(LCD_DMA_CTRL_REG) & 0x00000001;
	switch (burst_size) {
	case 1:
		reg |= LCD_DMA_BURST_SIZE(LCD_DMA_BURST_1);
		break;
	case 2:
		reg |= LCD_DMA_BURST_SIZE(LCD_DMA_BURST_2);
		break;
	case 4:
		reg |= LCD_DMA_BURST_SIZE(LCD_DMA_BURST_4);
		break;
	case 8:
		reg |= LCD_DMA_BURST_SIZE(LCD_DMA_BURST_8);
		break;
	case 16:
	default:
		reg |= LCD_DMA_BURST_SIZE(LCD_DMA_BURST_16);
		break;
	}

	reg |= (fifo_th << 8);

	lcdc_write(reg, LCD_DMA_CTRL_REG);

	return 0;
}

#define CNVT_TOHW(val, width) ((((val) << (width)) + 0x7FFF - (val)) >> 16)
static int fb_setcolreg(unsigned regno, unsigned red, unsigned green,
			      unsigned blue, unsigned transp,
			      struct fb_info *info)
{
	struct da8xx_fb_par *par = info->par;
	unsigned short *palette = (unsigned short *) par->v_palette_base;
	u_short pal;
	int update_hw = 0;

	if (regno > 255)
		return 1;

	if (info->fix.visual == FB_VISUAL_DIRECTCOLOR)
		return 1;

	if (info->var.bits_per_pixel > 16 && lcd_revision == LCD_VERSION_1)
		return -EINVAL;

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		red = CNVT_TOHW(red, info->var.red.length);
		green = CNVT_TOHW(green, info->var.green.length);
		blue = CNVT_TOHW(blue, info->var.blue.length);
		break;
	case FB_VISUAL_PSEUDOCOLOR:
		switch (info->var.bits_per_pixel) {
		case 4:
			if (regno > 15)
				return -EINVAL;

			if (info->var.grayscale) {
				pal = regno;
			} else {
				red >>= 4;
				green >>= 8;
				blue >>= 12;

				pal = red & 0x0f00;
				pal |= green & 0x00f0;
				pal |= blue & 0x000f;
			}
			if (regno == 0)
				pal |= 0x2000;
			palette[regno] = pal;
			break;

		case 8:
			red >>= 4;
			green >>= 8;
			blue >>= 12;

			pal = (red & 0x0f00);
			pal |= (green & 0x00f0);
			pal |= (blue & 0x000f);

			if (palette[regno] != pal) {
				update_hw = 1;
				palette[regno] = pal;
			}
			break;
		}
		break;
	}

	/* Truecolor has hardware independent palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v;

		if (regno > 15)
			return -EINVAL;

		v = (red << info->var.red.offset) |
			(green << info->var.green.offset) |
			(blue << info->var.blue.offset);

		switch (info->var.bits_per_pixel) {
		case 16:
			((u16 *) (info->pseudo_palette))[regno] = v;
			break;
		case 24:
		case 32:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		}
		if (palette[0] != 0x4000) {
			update_hw = 1;
			palette[0] = 0x4000;
		}
	}

	/* Update the palette in the h/w as needed. */
	if (update_hw)
        lcd_blit(LOAD_DATA, par);
    //	lcd_blit(LOAD_PALETTE, par);

	return 0;
}
#undef CNVT_TOHW

static void lcd_reset(struct da8xx_fb_par *par)
{
	/* Disable the Raster if previously Enabled */
	lcd_disable(false);

	/* DMA has to be disabled */
	lcdc_write(0, LCD_DMA_CTRL_REG);
}

static void lcd_calc_clk_divider(struct da8xx_fb_par *par)
{
	unsigned int lcd_clk, div;

	lcd_clk = clk_get_rate(par->lcdc_clk);
	div = lcd_clk / par->pxl_clk;

	/* Configure the LCD clock divisor. */
	lcdc_write(LCD_CLK_DIVISOR(div), LCD_CTRL_REG);
}

static void ssd2119_write_reg(u8 reg, u16 val){

    lcdc_write(reg, LCD_LIDD_CS0_ADDR);
    
    /* 16 Bit MODE */
    //lcdc_write(val, LCD_LIDD_CS0_DATA);

    /* 8 Bit Mode */
    lcdc_write((val >> 8) & 0xFF, LCD_LIDD_CS0_DATA);
    lcdc_write(val & 0xFF, LCD_LIDD_CS0_DATA);
}

static int ssd2119_module_init(void) {

    int device_code;

    // check device accress
    lcdc_write(0x0, LCD_LIDD_CS0_ADDR);
    mdelay(10);
    device_code = lcdc_read(LCD_LIDD_CS0_DATA);
    pr_info("Device Code: %d\n", device_code);
    if(device_code != 0x99){
        pr_err("Device Code Error!\n");
        //return -1;
    }

    
    // Power supply setting R03h
    //ssd2119_write_reg(3, 0x21);
    //ssd2119_write_reg(0x0C, 0x21);
    //ssd2119_write_reg(0x0D, 0x21);
    //ssd2119_write_reg(0x0E, 0x21);

    // device on but output off: R07h at 0021h
    ssd2119_write_reg(7, 0x21);

    // turn on the oscillator: R00h at 0001h
    ssd2119_write_reg(0, 0x01);

    // display output on: R07h at 0023h
    ssd2119_write_reg(7, 0x23);

    // exit sleep mode (R10h at 000h)
    ssd2119_write_reg(0x10, 0x00);

    // wait 30ms
    mdelay(30);

    // ??? set R07h at 0033h
    ssd2119_write_reg(7, 0x33);

#define LCD_MODE_65K 0x6000
#define LCD_DEN_MODE_INTERNAL 0x0800
#define LCD_WMODE_GENERIC 0x0400
#define LCD_NOSYNC_DMODE 0x0200
#define LCD_XY_DIR_INCREMENT 0x0030
    // entry mode setting (R11h)
    ssd2119_write_reg(0x11, LCD_MODE_65K + LCD_WMODE_GENERIC + LCD_DEN_MODE_INTERNAL + 
            LCD_NOSYNC_DMODE + LCD_XY_DIR_INCREMENT);

    // LCD driver AC setting (R02h)
    ssd2119_write_reg(2, 0x0);


    // Display ON
    
    return 0;
}

static int lcd_init(struct da8xx_fb_par *par, const struct lcd_ctrl_config *cfg,
		struct fb_videomode *panel)
{
	
	int ret = 0;

    /* disable raster controller */
    lcdc_write(0, LCD_RASTER_CTRL_REG);
    
    /* disable DMA */
	lcd_reset(par);

    /* setup LIDD control register: Asyn MPU 8080 mode, all signals active low, dma interrupt enable*/
    lcdc_write(LIDD_FRAME_DONE_INTERRUPT | LIDD_ASYN_8080_MODE, LCD_LIDD_CTRL_REG);

    /* clear status bits */
    lcdc_write(0x3FFF, LCD_STAT_REG);

    /* configure stobe timing */
    lcdc_write( LIDD_WRITE_SETUP(0x2) | LIDD_WRITE_DURATION(0x2) | LIDD_WRITE_HOLD(0x2) |  
                    LIDD_READ_SETUP(0x2) | LIDD_READ_DURATION(0x2) | LIDD_READ_HOLD(0x2) |  
                    LIDD_OPERATION_GAP(0),  LCD_LIDD_CS0_CONF);

	/* Calculate the divider */
	lcd_calc_clk_divider(par);

    ret = ssd2119_module_init();
	if (ret < 0)
		return ret;

	/* Configure the DMA burst size and fifo threshold. */
	ret = lcd_cfg_dma(cfg->dma_burst_sz, cfg->fifo_th);
	if (ret < 0)
		return ret;

	return 0;
}


/* IRQ handler for version 1 LCDC */
static irqreturn_t lcdc_irq_handler_rev01(int irq, void *arg)
{
	struct da8xx_fb_par *par = arg;
	u32 stat = lcdc_read(LCD_STAT_REG);
	u32 reg_ras;

	if ((stat & LCD_SYNC_LOST) && (stat & LCD_FIFO_UNDERFLOW)) {
		lcd_disable(false);
		lcdc_write(stat, LCD_STAT_REG);
		lcd_enable();
	} else if (stat & LCD_PL_LOAD_DONE) {
		/*
		 * Must disable raster before changing state of any control bit.
		 * And also must be disabled before clearing the PL loading
		 * interrupt via the following write to the status register. If
		 * this is done after then one gets multiple PL done interrupts.
		 */
    pr_info("PL Load Done interrupt\n");
		lcd_disable(false);

		lcdc_write(stat, LCD_STAT_REG);

		/* Disable PL completion inerrupt */
		reg_ras  = lcdc_read(LCD_RASTER_CTRL_REG);
		reg_ras &= ~LCD_V1_PL_INT_ENA;
		lcdc_write(reg_ras, LCD_RASTER_CTRL_REG);

		/* Setup and start data loading mode */
		lcd_blit(LOAD_DATA, par);
	} else {
		
        // disable the DMA
        lcd_disable(true);

        lcdc_write(stat, LCD_STAT_REG);
            
        // reset the LCD address
        ssd2119_write_reg(0x22, 0x0);

		if (stat & LCD_END_OF_FRAME0) {
			par->which_dma_channel_done = 0;
			lcdc_write(par->dma_start,
				   LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
			lcdc_write(par->dma_end,
				   LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
			par->vsync_flag = 1;
			wake_up_interruptible(&par->vsync_wait);
		}

		if (stat & LCD_END_OF_FRAME1) {
			par->which_dma_channel_done = 1;
			lcdc_write(par->dma_start,
				   LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
			lcdc_write(par->dma_end,
				   LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);
			par->vsync_flag = 1;
			wake_up_interruptible(&par->vsync_wait);
		}
        
        lcd_enable();
	}

	return IRQ_HANDLED;
}

static int fb_check_var(struct fb_var_screeninfo *var,
			struct fb_info *info)
{
	int err = 0;

	if (var->bits_per_pixel > 16 && lcd_revision == LCD_VERSION_1)
		return -EINVAL;

	switch (var->bits_per_pixel) {
	case 1:
	case 8:
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 0;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		var->nonstd = 0;
		break;
	case 4:
		var->red.offset = 0;
		var->red.length = 4;
		var->green.offset = 0;
		var->green.length = 4;
		var->blue.offset = 0;
		var->blue.length = 4;
		var->transp.offset = 0;
		var->transp.length = 0;
		var->nonstd = FB_NONSTD_REV_PIX_IN_B;
		break;
	case 16:		/* RGB 565 */
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;
		var->transp.offset = 0;
		var->transp.length = 0;
		var->nonstd = 0;
		break;
	case 24:
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->nonstd = 0;
		break;
	case 32:
		var->transp.offset = 24;
		var->transp.length = 8;
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->nonstd = 0;
		break;
	default:
		err = -EINVAL;
	}

	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->transp.msb_right = 0;
	return err;
}

#ifdef CONFIG_CPU_FREQ
static int lcd_da8xx_cpufreq_transition(struct notifier_block *nb,
				     unsigned long val, void *data)
{
	struct da8xx_fb_par *par;

	par = container_of(nb, struct da8xx_fb_par, freq_transition);
	if (val == CPUFREQ_POSTCHANGE) {
		if (par->lcd_fck_rate != clk_get_rate(par->lcdc_clk)) {
			par->lcd_fck_rate = clk_get_rate(par->lcdc_clk);
			lcd_disable(true);
			lcd_calc_clk_divider(par);
			if (par->blank == FB_BLANK_UNBLANK)
				lcd_enable();
		}
	}

	return 0;
}

static inline int lcd_da8xx_cpufreq_register(struct da8xx_fb_par *par)
{
	par->freq_transition.notifier_call = lcd_da8xx_cpufreq_transition;

	return cpufreq_register_notifier(&par->freq_transition,
					 CPUFREQ_TRANSITION_NOTIFIER);
}

static inline void lcd_da8xx_cpufreq_deregister(struct da8xx_fb_par *par)
{
	cpufreq_unregister_notifier(&par->freq_transition,
				    CPUFREQ_TRANSITION_NOTIFIER);
}
#endif

static int fb_remove(struct platform_device *dev)
{
	struct fb_info *info = dev_get_drvdata(&dev->dev);

	if (info) {
		struct da8xx_fb_par *par = info->par;

#ifdef CONFIG_CPU_FREQ
		lcd_da8xx_cpufreq_deregister(par);
#endif
		if (par->panel_power_ctrl)
			par->panel_power_ctrl(0);

		lcd_disable(true);
        
        lcdc_write(0, LCD_LIDD_CTRL_REG);

		/* disable DMA  */
		lcdc_write(0, LCD_DMA_CTRL_REG);

		unregister_framebuffer(info);
		fb_dealloc_cmap(&info->cmap);
		dma_free_coherent(NULL, PALETTE_SIZE, par->v_palette_base,
				  par->p_palette_base);
		dma_free_coherent(NULL, par->vram_size, par->vram_virt,
				  par->vram_phys);
		free_irq(par->irq, par);
		pm_runtime_put_sync(&dev->dev);
		pm_runtime_disable(&dev->dev);
		framebuffer_release(info);
		iounmap(da8xx_fb_reg_base);
		release_mem_region(lcdc_regs->start, resource_size(lcdc_regs));

	}
	return 0;
}


static int fb_wait_for_vsync(struct fb_info *info)
{
	struct da8xx_fb_par *par = info->par;
	int ret;

	/*
	 * Set flag to 0 and wait for isr to set to 1. It would seem there is a
	 * race condition here where the ISR could have occurred just before or
	 * just after this set. But since we are just coarsely waiting for
	 * a frame to complete then that's OK. i.e. if the frame completed
	 * just before this code executed then we have to wait another full
	 * frame time but there is no way to avoid such a situation. On the
	 * other hand if the frame completed just after then we don't need
	 * to wait long at all. Either way we are guaranteed to return to the
	 * user immediately after a frame completion which is all that is
	 * required.
	 */
	par->vsync_flag = 0;
	ret = wait_event_interruptible_timeout(par->vsync_wait,
					       par->vsync_flag != 0,
					       par->vsync_timeout);
	if (ret < 0)
		return ret;
	if (ret == 0)
		return -ETIMEDOUT;

	return 0;
}

static int fb_ioctl(struct fb_info *info, unsigned int cmd,
			  unsigned long arg)
{

	switch (cmd) {
	case FBIOGET_CONTRAST:
	case FBIOPUT_CONTRAST:
	case FBIGET_BRIGHTNESS:
	case FBIPUT_BRIGHTNESS:
	case FBIGET_COLOR:
	case FBIPUT_COLOR:
	case FBIPUT_HSYNC:
	case FBIPUT_VSYNC:
		return -ENOTTY;
	case FBIO_WAITFORVSYNC:
		return fb_wait_for_vsync(info);
	default:
		return -EINVAL;
	}
	return 0;
}

static int cfb_blank(int blank, struct fb_info *info)
{
	struct da8xx_fb_par *par = info->par;
	int ret = 0;

	if (par->blank == blank)
		return 0;

	par->blank = blank;
	switch (blank) {
	case FB_BLANK_UNBLANK:
		lcd_enable();

		if (par->panel_power_ctrl)
			par->panel_power_ctrl(1);
		break;
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		if (par->panel_power_ctrl)
			par->panel_power_ctrl(0);

		lcd_disable(true);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*
 * Set new x,y offsets in the virtual display for the visible area and switch
 * to the new mode.
 */
static int da8xx_pan_display(struct fb_var_screeninfo *var,
			     struct fb_info *fbi)
{
	int ret = 0;
	struct fb_var_screeninfo new_var;
	struct da8xx_fb_par         *par = fbi->par;
	struct fb_fix_screeninfo    *fix = &fbi->fix;
	unsigned int end;
	unsigned int start;
	unsigned long irq_flags;

	if (var->xoffset != fbi->var.xoffset ||
			var->yoffset != fbi->var.yoffset) {
		memcpy(&new_var, &fbi->var, sizeof(new_var));
		new_var.xoffset = var->xoffset;
		new_var.yoffset = var->yoffset;
		if (fb_check_var(&new_var, fbi))
			ret = -EINVAL;
		else {
			memcpy(&fbi->var, &new_var, sizeof(new_var));

			start	= fix->smem_start +
				new_var.yoffset * fix->line_length +
				new_var.xoffset * fbi->var.bits_per_pixel / 8;
			end	= start + fbi->var.yres * fix->line_length - 1;
			par->dma_start	= start;
			par->dma_end	= end;
			spin_lock_irqsave(&par->lock_for_chan_update,
					irq_flags);
			if (par->which_dma_channel_done == 0) {
				lcdc_write(par->dma_start,
					   LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
				lcdc_write(par->dma_end,
					   LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
			} else if (par->which_dma_channel_done == 1) {
				lcdc_write(par->dma_start,
					   LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
				lcdc_write(par->dma_end,
					   LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);
			}
			spin_unlock_irqrestore(&par->lock_for_chan_update,
					irq_flags);
		}
	}

	return ret;
}

static struct fb_ops da8xx_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = fb_check_var,
	.fb_setcolreg = fb_setcolreg,
	.fb_pan_display = da8xx_pan_display,
	.fb_ioctl = fb_ioctl,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_blank = cfb_blank,
};

/* Calculate and return pixel clock period in pico seconds */
static unsigned int da8xxfb_pixel_clk_period(struct da8xx_fb_par *par)
{
	unsigned int lcd_clk, div;
	unsigned int configured_pix_clk;
	unsigned long long pix_clk_period_picosec = 1000000000000ULL;

	lcd_clk = clk_get_rate(par->lcdc_clk);
	div = lcd_clk / par->pxl_clk;
	configured_pix_clk = (lcd_clk / div);

	do_div(pix_clk_period_picosec, configured_pix_clk);

	return pix_clk_period_picosec;
}

static int fb_probe(struct platform_device *device)
{
	struct da8xx_lcdc_platform_data *fb_pdata =
						device->dev.platform_data;
	struct lcd_ctrl_config *lcd_cfg;
	struct fb_videomode *lcdc_info;
	struct fb_info *da8xx_fb_info;
	struct clk *fb_clk = NULL;
	struct da8xx_fb_par *par;
	resource_size_t len;
	int ret, i;
	unsigned long ulcm;

  pr_info("probing the LCD LIDD module\n");

 	if (fb_pdata == NULL) {
		dev_err(&device->dev, "Can not get platform data\n");
		return -ENOENT;
	}

	lcdc_regs = platform_get_resource(device, IORESOURCE_MEM, 0);
	if (!lcdc_regs) {
		dev_err(&device->dev,
			"Can not get memory resource for LCD controller\n");
		return -ENOENT;
	}

	len = resource_size(lcdc_regs);

	lcdc_regs = request_mem_region(lcdc_regs->start, len, lcdc_regs->name);
	if (!lcdc_regs)
		return -EBUSY;

	da8xx_fb_reg_base = ioremap(lcdc_regs->start, len);
	if (!da8xx_fb_reg_base) {
		ret = -EBUSY;
		goto err_request_mem;
	}

	fb_clk = clk_get(&device->dev, "fck");
	if (IS_ERR(fb_clk)) {
		dev_err(&device->dev, "Can not get device clock\n");
		ret = -ENODEV;
		goto err_ioremap;
	}

	pm_runtime_enable(&device->dev);
	pm_runtime_get_sync(&device->dev);

	/* Determine LCD IP Version */
	switch (lcdc_read(LCD_PID_REG)) {
	case 0x4C100102:
		lcd_revision = LCD_VERSION_1;
		break;
	case 0x4F200800:
	case 0x4F201000:
		lcd_revision = LCD_VERSION_2;
		break;
	default:
		dev_warn(&device->dev, "Unknown PID Reg value 0x%x, "
				"defaulting to LCD revision 1\n",
				lcdc_read(LCD_PID_REG));
		lcd_revision = LCD_VERSION_1;
		break;
	}

	for (i = 0, lcdc_info = known_lcd_panels;
		i < ARRAY_SIZE(known_lcd_panels);
		i++, lcdc_info++) {
		if (strcmp(fb_pdata->type, lcdc_info->name) == 0)
			break;
	}

	if (i == ARRAY_SIZE(known_lcd_panels)) {
		dev_err(&device->dev, "GLCD: No valid panel found\n");
		ret = -ENODEV;
		goto err_pm_runtime_disable;
	} else
		dev_info(&device->dev, "GLCD: Found %s panel\n",
					fb_pdata->type);

	lcd_cfg = (struct lcd_ctrl_config *)fb_pdata->controller_data;

	da8xx_fb_info = framebuffer_alloc(sizeof(struct da8xx_fb_par),
					&device->dev);
	if (!da8xx_fb_info) {
		dev_dbg(&device->dev, "Memory allocation failed for fb_info\n");
		ret = -ENOMEM;
		goto err_pm_runtime_disable;
	}

	par = da8xx_fb_info->par;
	par->lcdc_clk = fb_clk;
#ifdef CONFIG_CPU_FREQ
	par->lcd_fck_rate = clk_get_rate(fb_clk);
#endif
	par->pxl_clk = lcdc_info->pixclock;
	if (fb_pdata->panel_power_ctrl) {
		par->panel_power_ctrl = fb_pdata->panel_power_ctrl;
		par->panel_power_ctrl(1);
	}

    // this is where we send module specific init commands
	if (lcd_init(par, lcd_cfg, lcdc_info) < 0) {
		dev_err(&device->dev, "lcd_init failed\n");
		ret = -EFAULT;
		goto err_release_fb;
	}

	/* allocate frame buffer */
	par->vram_size = lcdc_info->xres * lcdc_info->yres * lcd_cfg->bpp;
	ulcm = lcm((lcdc_info->xres * lcd_cfg->bpp)/8, PAGE_SIZE);
	par->vram_size = roundup(par->vram_size/8, ulcm);
	par->vram_size = par->vram_size * LCD_NUM_BUFFERS;

	par->vram_virt = dma_alloc_coherent(NULL,
					    par->vram_size,
					    (resource_size_t *) &par->vram_phys,
					    GFP_KERNEL | GFP_DMA);
	if (!par->vram_virt) {
		dev_err(&device->dev,
			"GLCD: kmalloc for frame buffer failed\n");
		ret = -EINVAL;
		goto err_release_fb;
	}

	da8xx_fb_info->screen_base = (char __iomem *) par->vram_virt;
	da8xx_fb_fix.smem_start    = par->vram_phys;
	da8xx_fb_fix.smem_len      = par->vram_size;
	da8xx_fb_fix.line_length   = (lcdc_info->xres * lcd_cfg->bpp) / 8;

	par->dma_start = par->vram_phys;
	par->dma_end   = par->dma_start + lcdc_info->yres *
		da8xx_fb_fix.line_length - 1;

	/* allocate palette buffer */
	par->v_palette_base = dma_alloc_coherent(NULL,
					       PALETTE_SIZE,
					       (resource_size_t *)
					       &par->p_palette_base,
					       GFP_KERNEL | GFP_DMA);
	if (!par->v_palette_base) {
		dev_err(&device->dev,
			"GLCD: kmalloc for palette buffer failed\n");
		ret = -EINVAL;
		goto err_release_fb_mem;
	}
	memset(par->v_palette_base, 0, PALETTE_SIZE);

	par->irq = platform_get_irq(device, 0);
	if (par->irq < 0) {
		ret = -ENOENT;
		goto err_release_pl_mem;
	}

	/* Initialize par */
	da8xx_fb_info->var.bits_per_pixel = lcd_cfg->bpp;

	da8xx_fb_var.xres = lcdc_info->xres;
	da8xx_fb_var.xres_virtual = lcdc_info->xres;

	da8xx_fb_var.yres         = lcdc_info->yres;
	da8xx_fb_var.yres_virtual = lcdc_info->yres * LCD_NUM_BUFFERS;

	da8xx_fb_var.grayscale =
	    lcd_cfg->panel_shade == MONOCHROME ? 1 : 0;
	da8xx_fb_var.bits_per_pixel = lcd_cfg->bpp;

	da8xx_fb_var.pixclock = da8xxfb_pixel_clk_period(par);

	/* Initialize fbinfo */
	da8xx_fb_info->flags = FBINFO_FLAG_DEFAULT;
	da8xx_fb_info->fix = da8xx_fb_fix;
	da8xx_fb_info->var = da8xx_fb_var;
	da8xx_fb_info->fbops = &da8xx_fb_ops;
	da8xx_fb_info->pseudo_palette = par->pseudo_palette;
	da8xx_fb_info->fix.visual = (da8xx_fb_info->var.bits_per_pixel <= 8) ?
				FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_TRUECOLOR;

	ret = fb_alloc_cmap(&da8xx_fb_info->cmap, PALETTE_SIZE, 0);
	if (ret)
		goto err_release_pl_mem;
	da8xx_fb_info->cmap.len = par->palette_sz;

	/* initialize var_screeninfo */
	da8xx_fb_var.activate = FB_ACTIVATE_FORCE;
	fb_set_var(da8xx_fb_info, &da8xx_fb_var);

	dev_set_drvdata(&device->dev, da8xx_fb_info);

	/* initialize the vsync wait queue */
	init_waitqueue_head(&par->vsync_wait);
	par->vsync_timeout = HZ / 5;
	par->which_dma_channel_done = -1;
	spin_lock_init(&par->lock_for_chan_update);

	/* Register the Frame Buffer  */
	if (register_framebuffer(da8xx_fb_info) < 0) {
		dev_err(&device->dev,
			"GLCD: Frame Buffer Registration Failed!\n");
		ret = -EINVAL;
		goto err_dealloc_cmap;
	}

#ifdef CONFIG_CPU_FREQ
	ret = lcd_da8xx_cpufreq_register(par);
	if (ret) {
		dev_err(&device->dev, "failed to register cpufreq\n");
		goto err_cpu_freq;
	}
#endif

    lcdc_irq_handler = lcdc_irq_handler_rev01;

	ret = request_irq(par->irq, lcdc_irq_handler, 0,
			DRIVER_NAME, par);
	if (ret)
		goto irq_freq;
	return 0;

irq_freq:
#ifdef CONFIG_CPU_FREQ
	lcd_da8xx_cpufreq_deregister(par);
err_cpu_freq:
#endif
	unregister_framebuffer(da8xx_fb_info);

err_dealloc_cmap:
	fb_dealloc_cmap(&da8xx_fb_info->cmap);

err_release_pl_mem:
	dma_free_coherent(NULL, PALETTE_SIZE, par->v_palette_base,
			  par->p_palette_base);

err_release_fb_mem:
	dma_free_coherent(NULL, par->vram_size, par->vram_virt, par->vram_phys);

err_release_fb:
	framebuffer_release(da8xx_fb_info);

err_pm_runtime_disable:
	pm_runtime_put_sync(&device->dev);
	pm_runtime_disable(&device->dev);

err_ioremap:
	iounmap(da8xx_fb_reg_base);

err_request_mem:
	release_mem_region(lcdc_regs->start, len);

	return ret;
}

#ifdef CONFIG_PM
struct lcdc_context {
	u32 clk_enable;
	u32 ctrl;
	u32 dma_ctrl;
	u32 int_enable_set;
	u32 dma_frm_buf_base_addr_0;
	u32 dma_frm_buf_ceiling_addr_0;
	u32 dma_frm_buf_base_addr_1;
	u32 dma_frm_buf_ceiling_addr_1;
} reg_context;

static void lcd_context_save(void)
{
	reg_context.ctrl = lcdc_read(LCD_CTRL_REG);
	reg_context.dma_ctrl = lcdc_read(LCD_DMA_CTRL_REG);
	reg_context.dma_frm_buf_base_addr_0 =
		lcdc_read(LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
	reg_context.dma_frm_buf_ceiling_addr_0 =
		lcdc_read(LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
	reg_context.dma_frm_buf_base_addr_1 =
		lcdc_read(LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
	reg_context.dma_frm_buf_ceiling_addr_1 =
		lcdc_read(LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);
	return;
}

static void lcd_context_restore(void)
{
	lcdc_write(reg_context.ctrl, LCD_CTRL_REG);
	lcdc_write(reg_context.dma_ctrl, LCD_DMA_CTRL_REG);
	lcdc_write(reg_context.dma_frm_buf_base_addr_0,
			LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
	lcdc_write(reg_context.dma_frm_buf_ceiling_addr_0,
			LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
	lcdc_write(reg_context.dma_frm_buf_base_addr_1,
			LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
	lcdc_write(reg_context.dma_frm_buf_ceiling_addr_1,
			LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);
	return;
}

static int fb_suspend(struct platform_device *dev, pm_message_t state)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct da8xx_fb_par *par = info->par;

	console_lock();
	if (par->panel_power_ctrl)
		par->panel_power_ctrl(0);

	fb_set_suspend(info, 1);
	lcd_disable(true);
	lcd_context_save();
	pm_runtime_put_sync(&dev->dev);
	console_unlock();

	return 0;
}
static int fb_resume(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct da8xx_fb_par *par = info->par;

	console_lock();
	pm_runtime_get_sync(&dev->dev);
	lcd_context_restore();
	if (par->blank == FB_BLANK_UNBLANK) {
		lcd_enable();

		if (par->panel_power_ctrl)
			par->panel_power_ctrl(1);
	}

	fb_set_suspend(info, 0);
	console_unlock();

	return 0;
}
#else
#define fb_suspend NULL
#define fb_resume NULL
#endif

static struct platform_driver da8xx_fb_driver = {
	.probe = fb_probe,
	.remove = fb_remove,
	.suspend = fb_suspend,
	.resume = fb_resume,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init da8xx_fb_init(void)
{ 
  pr_info("register the fb lidd driver\n");
	return platform_driver_register(&da8xx_fb_driver);
}

static void __exit da8xx_fb_cleanup(void)
{
	platform_driver_unregister(&da8xx_fb_driver);
}

module_init(da8xx_fb_init);
module_exit(da8xx_fb_cleanup);

MODULE_DESCRIPTION("Framebuffer driver for TI da8xx/omap-l1xx");
MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");
