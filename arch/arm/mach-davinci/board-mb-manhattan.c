/*
 * Hawkboard.org based on TI's OMAP-L138 Platform
 *
 * Initial code: Syed Mohammed Khasim
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of
 * any kind, whether express or implied.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/gpio.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_device.h>
#include <linux/spi/spi_gpio.h>
#include <linux/spi/spi.h>
#include <linux/platform_data/mtd-davinci.h>
#include <linux/platform_data/mtd-davinci-aemif.h>
#include <linux/platform_data/spi-davinci.h>
#include <linux/platform_data/uio_pruss.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/cp_intc.h>
#include <mach/da8xx.h>
#include <mach/mux.h>
#include <mach/psc.h>

#define MANHATTAN_PHY_ID		NULL

#define DA850_USB1_VBUS_PIN		GPIO_TO_PIN(2, 9)
#define DA850_USB1_OC_PIN		GPIO_TO_PIN(2, 8)

static short stepper_pru_pins[] = {
    DA850_PRU1_R30_1,
    DA850_PRU1_R30_8,
    DA850_PRU1_R30_3,
    DA850_PRU1_R30_6,
    DA850_PRU1_R30_11,
    DA850_PRU1_R31_26,
    DA850_PRU1_R30_22,
    DA850_PRU1_R30_15,
    DA850_PRU1_R30_21,
    DA850_PRU1_R30_20,
    DA850_PRU1_R30_16,
    DA850_PRU1_R31_23,
    DA850_PRU1_R30_18,
    DA850_PRU1_R30_29,
    DA850_PRU1_R30_17,
    DA850_PRU1_R30_9,
    DA850_PRU1_R30_0,
    DA850_PRU1_R31_25,
   -1,
};

static short toolhead_spi_pins[] = {
	DA850_GPIO2_11, //DA850_SPI1_SOMI,
	DA850_GPIO2_13, //DA850_SPI1_CLK,
	DA850_GPIO2_10, //DA850_SPI1_SIMO,
	DA850_GPIO2_14, //DA850_SPI1_SCS_0,
	DA850_GPIO1_3, //DA850_SPI1_SCS_5,
    DA850_GPIO2_5,
    DA850_GPIO2_7,
    DA850_GPIO6_5,
    DA850_GPIO6_11,
    DA850_GPIO0_15,
    DA850_GPIO1_15,
    -1,
};

static struct davinci_spi_config toolhead_spi_cfg = {
	.io_type	= SPI_IO_TYPE_INTR,
	.c2tdelay	= 8,
	.t2cdelay	= 8,
};

static struct spi_gpio_platform_data spi1_pdata = {
	.miso		= GPIO_TO_PIN(2, 11), //2, 13),
	.mosi		= GPIO_TO_PIN(2, 10),
	.sck        = GPIO_TO_PIN(2, 13),
    .num_chipselect = 2,
};

static struct platform_device spi1_device = {
	.name		= "spi_gpio",
	.id		= 1,
	.dev.platform_data = &spi1_pdata,
};

static struct spi_board_info toolhead_spi_info[] = {
	{
		.modalias		= "spidev",
		.controller_data	= (void *)GPIO_TO_PIN(2,14),
		//.controller_data	= &toolhead_spi_cfg,
		.mode			= SPI_MODE_3,
		.max_speed_hz		= 30000000,
		.bus_num		= 1,
		.chip_select		= 0,
	},
	{
		.modalias		= "spidev",
		.controller_data	= (void *)GPIO_TO_PIN(1,3),
		.mode			= SPI_MODE_3,
		.max_speed_hz		= 30000000,
		.bus_num		= 1,
		.chip_select		= 1,
	},
};

#define DA850_12V_POWER_PIN  GPIO_TO_PIN(3,15)

static short mb_power_pins[] = {
  DA850_GPIO3_15,
  -1,
};

static void da850_12V_power_control(int val)
{
	/* 12V power rail */
	gpio_set_value(DA850_12V_POWER_PIN, val);
}

static int da850_power_init(void)
{
	int status;

	status = gpio_request(DA850_12V_POWER_PIN, "12V power\n");
	if (status < 0)
		return status;

	gpio_direction_output(DA850_12V_POWER_PIN, 0);

    da850_12V_power_control(0);

	return 0;
}

#define DA850_LCD_BL_PIN    GPIO_TO_PIN(6, 7)
#define DA850_LCD_RESET_PIN GPIO_TO_PIN(8, 15)

#ifdef CONFIG_FB_DA8XX
static short mb_lcd_spi_pins[] = {
  DA850_GPIO6_3,
  DA850_GPIO6_13,
  DA850_GPIO6_7,
  DA850_GPIO6_8,
  -1,
};

static struct da8xx_spi_pin_data lcd_spi_gpio_data = {
    .sck = GPIO_TO_PIN(6, 13),
    .sdi = GPIO_TO_PIN(6, 3),
    .cs = GPIO_TO_PIN(6, 8),
};
#endif

static short mb_lcd_power_pins[] = {
    DA850_GPIO6_9,
    DA850_GPIO8_15,
    -1,
};

static void da850_panel_power_ctrl(int val)
{
	/* lcd backlight */
	gpio_set_value(DA850_LCD_BL_PIN, val);

    /* lcd_reset */
    gpio_set_value(DA850_LCD_RESET_PIN, val);
}

static int da850_lcd_hw_init(void)
{
	int status;

	status = gpio_request(DA850_LCD_BL_PIN, "lcd bl\n");
	if (status < 0)
		return status;

	gpio_direction_output(DA850_LCD_BL_PIN, 0);

    /* pull the reset pin high */
	status = gpio_request(DA850_LCD_RESET_PIN, "lcd reset\n");
	if (status < 0)
		return status;

	gpio_direction_output(DA850_LCD_RESET_PIN, 0);

	/* Switch off panel power and backlight */
	da850_panel_power_ctrl(0);

	/* Switch on panel power and backlight */
	da850_panel_power_ctrl(1);


	return 0;
}


const short mb_manhattan_led_pins[] = {
    DA850_GPIO0_1,
    DA850_PRU1_R30_24,
    DA850_PRU1_R30_25,
    -1
};

static struct gpio_led gpio_leds[] = {
    {
        .name           = "Status LED",
        .gpio           = GPIO_TO_PIN(0,1),
        .default_trigger= "heartbeat",
    },
};

static struct gpio_led_platform_data gpio_led_info = {
    .leds       = gpio_leds,
    .num_leds   = ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
    .name   = "leds-gpio",
    .id = -1,
    .dev    = {
    .platform_data  = &gpio_led_info,
    },
};

static struct mtd_partition da850_evm_nandflash_partition[] = {
	{
		.name		= "u-boot env",
		.offset		= 0,
		.size		= SZ_1M,
		.mask_flags	= MTD_WRITEABLE,
	 },
	{
		.name		= "UBL",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_1M,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "u-boot",
		.offset		= 6 * SZ_1M,
		.size		= SZ_1M,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "kernel",
		.offset		= 0x1000000,
		.size		= SZ_4M,
		.mask_flags	= 0,
	},
	{
		.name		= "filesystem",
		.offset		= 0x1500000,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	},
};

static struct davinci_aemif_timing da850_evm_nandflash_timing = {
	.wsetup		= 24,
	.wstrobe	= 21,
	.whold		= 14,
	.rsetup		= 19,
	.rstrobe	= 50,
	.rhold		= 0,
	.ta		= 20,
};

static struct davinci_nand_pdata da850_evm_nandflash_data = {
	.parts		= da850_evm_nandflash_partition,
	.nr_parts	= ARRAY_SIZE(da850_evm_nandflash_partition),
	.ecc_mode	= NAND_ECC_SOFT_BCH,
	.ecc_bits	= 24,
	.bbt_options	= NAND_BBT_USE_FLASH,
	.timing		= &da850_evm_nandflash_timing,
};

static struct resource da850_evm_nandflash_resource[] = {
	{
		.start	= DA8XX_AEMIF_CS3_BASE,
		.end	= DA8XX_AEMIF_CS3_BASE + SZ_512K + 2 * SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= DA8XX_AEMIF_CTL_BASE,
		.end	= DA8XX_AEMIF_CTL_BASE + SZ_32K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device da850_evm_nandflash_device = {
	.name		= "davinci_nand",
	.id		= 1,
	.dev		= {
		.platform_data	= &da850_evm_nandflash_data,
	},
	.num_resources	= ARRAY_SIZE(da850_evm_nandflash_resource),
	.resource	= da850_evm_nandflash_resource,
};

static struct platform_device *da850_evm_devices[] = {
	&da850_evm_nandflash_device,
};

#define DA8XX_AEMIF_CE2CFG_OFFSET	0x10
#define DA8XX_AEMIF_ASIZE_16BIT		0x1

static short omapl138_hawk_mii_pins[] __initdata = {
	DA850_MII_TXEN, DA850_MII_TXCLK, DA850_MII_COL, DA850_MII_TXD_3,
	DA850_MII_TXD_2, DA850_MII_TXD_1, DA850_MII_TXD_0, DA850_MII_RXER,
	DA850_MII_CRS, DA850_MII_RXCLK, DA850_MII_RXDV, DA850_MII_RXD_3,
	DA850_MII_RXD_2, DA850_MII_RXD_1, DA850_MII_RXD_0, DA850_MDIO_CLK,
	DA850_MDIO_D,
	-1
};

static __init void omapl138_hawk_config_emac(void)
{
	void __iomem *cfgchip3 = DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP3_REG);
	int ret;
	u32 val;
	struct davinci_soc_info *soc_info = &davinci_soc_info;

	val = __raw_readl(cfgchip3);
	val &= ~BIT(8);
	ret = davinci_cfg_reg_list(omapl138_hawk_mii_pins);
	if (ret) {
		pr_warn("%s: CPGMAC/MII mux setup failed: %d\n", __func__, ret);
		return;
	}

	/* configure the CFGCHIP3 register for MII */
	__raw_writel(val, cfgchip3);
	pr_info("EMAC: MII PHY configured\n");

	soc_info->emac_pdata->phy_id = MANHATTAN_PHY_ID;

	ret = da8xx_register_emac();
	if (ret)
		pr_warn("%s: EMAC registration failed: %d\n", __func__, ret);
}

/*
 * The following EDMA channels/slots are not being used by drivers (for
 * example: Timer, GPIO, UART events etc) on da850/omap-l138 EVM/Hawkboard,
 * hence they are being reserved for codecs on the DSP side.
 */
static const s16 da850_dma0_rsv_chans[][2] = {
	/* (offset, number) */
	{ 8,  6},
	{24,  4},
	{30,  2},
	{-1, -1}
};

static const s16 da850_dma0_rsv_slots[][2] = {
	/* (offset, number) */
	{ 8,  6},
	{24,  4},
	{30, 50},
	{-1, -1}
};

static const s16 da850_dma1_rsv_chans[][2] = {
	/* (offset, number) */
	{ 0, 28},
	{30,  2},
	{-1, -1}
};

static const s16 da850_dma1_rsv_slots[][2] = {
	/* (offset, number) */
	{ 0, 28},
	{30, 90},
	{-1, -1}
};

static struct edma_rsv_info da850_edma_cc0_rsv = {
	.rsv_chans	= da850_dma0_rsv_chans,
	.rsv_slots	= da850_dma0_rsv_slots,
};

static struct edma_rsv_info da850_edma_cc1_rsv = {
	.rsv_chans	= da850_dma1_rsv_chans,
	.rsv_slots	= da850_dma1_rsv_slots,
};

static struct edma_rsv_info *da850_edma_rsv[2] = {
	&da850_edma_cc0_rsv,
	&da850_edma_cc1_rsv,
};


static irqreturn_t omapl138_hawk_usb_ocic_irq(int irq, void *dev_id);
static da8xx_ocic_handler_t hawk_usb_ocic_handler;

static const short da850_hawk_usb11_pins[] = {
	DA850_GPIO2_8, DA850_GPIO2_9,
	-1
};

static int hawk_usb_set_power(unsigned port, int on)
{
	gpio_set_value(DA850_USB1_VBUS_PIN, on);
	return 0;
}

static int hawk_usb_get_power(unsigned port)
{
	return gpio_get_value(DA850_USB1_VBUS_PIN);
}

static int hawk_usb_get_oci(unsigned port)
{
	return !gpio_get_value(DA850_USB1_OC_PIN);
}

static int hawk_usb_ocic_notify(da8xx_ocic_handler_t handler)
{
	int irq         = gpio_to_irq(DA850_USB1_OC_PIN);
	int error       = 0;

	if (handler != NULL) {
		hawk_usb_ocic_handler = handler;

		error = request_irq(irq, omapl138_hawk_usb_ocic_irq,
					IRQF_DISABLED | IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING,
					"OHCI over-current indicator", NULL);
		if (error)
			pr_err("%s: could not request IRQ to watch "
				"over-current indicator changes\n", __func__);
	} else {
		free_irq(irq, NULL);
	}
	return error;
}

static struct da8xx_ohci_root_hub omapl138_hawk_usb11_pdata = {
	.set_power      = hawk_usb_set_power,
	.get_power      = hawk_usb_get_power,
	.get_oci        = hawk_usb_get_oci,
	.ocic_notify    = hawk_usb_ocic_notify,
	/* TPS2087 switch @ 5V */
	.potpgt         = (3 + 1) / 2,  /* 3 ms max */
};

static irqreturn_t omapl138_hawk_usb_ocic_irq(int irq, void *dev_id)
{
	hawk_usb_ocic_handler(&omapl138_hawk_usb11_pdata, 1);
	return IRQ_HANDLED;
}

static __init void omapl138_hawk_usb_init(void)
{
	int ret;
	u32 cfgchip2;

	ret = davinci_cfg_reg_list(da850_hawk_usb11_pins);
	if (ret) {
		pr_warn("%s: USB 1.1 PinMux setup failed: %d\n", __func__, ret);
		return;
	}

	/* Setup the Ref. clock frequency for the HAWK at 24 MHz. */
	cfgchip2 = __raw_readl(DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP2_REG));
	cfgchip2 &= ~CFGCHIP2_REFFREQ;
	cfgchip2 |=  CFGCHIP2_REFFREQ_24MHZ;

    cfgchip2 &= ~CFGCHIP2_OTGMODE;
	cfgchip2 |=  CFGCHIP2_FORCE_DEVICE;
	cfgchip2 |=  CFGCHIP2_SESENDEN | CFGCHIP2_PHY_PLLON;

	__raw_writel(cfgchip2, DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP2_REG));
   
    ret = da8xx_register_usb20(1000, 3);
    if (ret)
        pr_warning("%s: USB 2.0 registration failed: %d\n",
               __func__, ret);

	ret = gpio_request_one(DA850_USB1_VBUS_PIN,
			GPIOF_DIR_OUT, "USB1 VBUS");
	if (ret < 0) {
		pr_err("%s: failed to request GPIO for USB 1.1 port "
			"power control: %d\n", __func__, ret);
		return;
	}

	ret = gpio_request_one(DA850_USB1_OC_PIN,
			GPIOF_DIR_IN, "USB1 OC");
	if (ret < 0) {
		pr_err("%s: failed to request GPIO for USB 1.1 port "
			"over-current indicator: %d\n", __func__, ret);
		goto usb11_setup_oc_fail;
	}

	ret = da8xx_register_usb11(&omapl138_hawk_usb11_pdata);
	if (ret) {
		pr_warn("%s: USB 1.1 registration failed: %d\n", __func__, ret);
		goto usb11_setup_fail;
	}

	return;

usb11_setup_fail:
	gpio_free(DA850_USB1_OC_PIN);
usb11_setup_oc_fail:
	gpio_free(DA850_USB1_VBUS_PIN);
}

static struct davinci_uart_config omapl138_hawk_uart_config __initdata = {
	.enabled_uarts = 0x7,
};

static __init void omapl138_hawk_init(void)
{
	int ret;
	u32 cfgchip3;

	davinci_serial_init(&omapl138_hawk_uart_config);

	omapl138_hawk_config_emac();

	ret = da850_register_edma(0);
	if (ret)
		pr_warn("%s: EDMA registration failed: %d\n", __func__, ret);

	omapl138_hawk_usb_init();

	ret = davinci_cfg_reg_list(mb_power_pins);
    if (ret)
        pr_warn("%s: power pin setup failed!: %d\n", __func__, ret);
    ret = da850_power_init();
    if (ret)
        pr_warn("%s: power pin init failed!: %d\n", __func__, ret);
    else
        pr_warn("power pin success!!!!!!!!!!!!!!!!: pin value: %d\n", gpio_get_value(DA850_12V_POWER_PIN));

    platform_add_devices(da850_evm_devices,
        ARRAY_SIZE(da850_evm_devices));
  
    /* Toolhead SPI */ 
	ret = davinci_cfg_reg_list(toolhead_spi_pins);
	if (ret)
		pr_warn("%s: Toolhead spi mux setup failed: %d\n", __func__, ret);

	ret = spi_register_board_info(toolhead_spi_info,
				      ARRAY_SIZE(toolhead_spi_info));
	if (ret)
		pr_warn("%s: spi info registration failed: %d\n", __func__,
			ret);

    platform_device_register(&spi1_device);
    //ret = da8xx_register_spi_bus(1, ARRAY_SIZE(toolhead_spi_info));
	if (ret)
		pr_warn("%s: SPI 1 registration failed: %d\n", __func__, ret);

	/* LCD  */
	ret = davinci_cfg_reg_list(da850_lcdcntl_pins);
	if (ret)
		pr_warn("%s: LCDC mux setup failed: %d\n", __func__, ret);

    ret = davinci_cfg_reg_list(mb_lcd_power_pins);
	if (ret)
		pr_warn("%s: LCD pins initialization failed: %d\n", __func__, ret);
	ret = da850_lcd_hw_init();
	if (ret)
		pr_warn("%s: LCD initialization failed: %d\n", __func__, ret);

#ifdef CONFIG_FB_DA8XX
	ret = davinci_cfg_reg_list(mb_lcd_spi_pins);
	if (ret)
		pr_warn("%s: LCDC spi mux setup failed: %d\n", __func__, ret);

	ssd2119_spi_pdata.panel_power_ctrl = da850_panel_power_ctrl,
    ssd2119_spi_pdata.spi = &lcd_spi_gpio_data;
	ret = da8xx_register_lcdc_spi(&ssd2119_spi_pdata);
#else
    ssd2119_pdata.panel_power_ctrl = da850_panel_power_ctrl,
    pr_info("LCD LIDD: %s \n", ssd2119_pdata.manu_name);
    ret = da8xx_register_lcdc_lidd(&ssd2119_pdata);
#endif //FB_DA8XX_LIDD
	if (ret)
		pr_warn("%s: LCDC registration failed: %d\n", __func__, ret);

	ret = da8xx_register_watchdog();
	if (ret)
		pr_warn("%s: watchdog registration failed: %d\n",
			__func__, ret);

    ret = davinci_cfg_reg_list(mb_manhattan_led_pins);
    if (ret)
      pr_warn("mb_manhattan_init: LED pinmux failed: %d\n", ret);

    platform_device_register(&leds_gpio);
    if (ret)
         pr_warn("da850_evm_init: led device initialization failed: %d\n", ret);

	/* Setup alternate events on the PRUs */
	cfgchip3 = __raw_readl(DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP3_REG));
	cfgchip3 |=  BIT(3);
	__raw_writel(cfgchip3, DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP3_REG));

    ret = davinci_cfg_reg_list(stepper_pru_pins);
	if (ret)
		pr_warn("%s: stepper pins initialization failed: %d\n", __func__, ret);

	/* Register PRUSS device */
	da8xx_register_uio_pruss();
    if (ret)
         pr_warn("pruss init failed %d\n", ret);

    /* read the pruss clock */

    davinci_psc_is_clk_active(0,13);

}

#ifdef CONFIG_SERIAL_8250_CONSOLE
static int __init omapl138_hawk_console_init(void)
{

	return add_preferred_console("ttyS", 1, "115200");
}
console_initcall(omapl138_hawk_console_init);
#endif

static void __init omapl138_hawk_map_io(void)
{
	da850_init();
}

MACHINE_START(DAVINCI_MANHATTAN, "Makerbot Controller Manhattan on DaVinci AM18xx")
	.atag_offset	= 0x100,
	.map_io		= omapl138_hawk_map_io,
	.init_irq	= cp_intc_init,
	.timer		= &davinci_timer,
	.init_machine	= omapl138_hawk_init,
	.init_late	= davinci_init_late,
	.dma_zone_size	= SZ_128M,
	.restart	= da8xx_restart,
MACHINE_END
