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
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/rotary_encoder.h>
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
#include <linux/etherdevice.h>
#include <linux/wl12xx.h>
#include <linux/wireless.h>
#include <linux/leds.h>
#include <linux/i2c-gpio.h>
#include <linux/workqueue.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/cp_intc.h>
#include <mach/da8xx.h>
#include <mach/mux.h>
#include <mach/psc.h>
#include <mach/serial.h>

#include <linux/makerbot/buzzer.h>

#define MANHATTAN_PHY_ID		NULL

#define DA850_USB1_VBUS_PIN		GPIO_TO_PIN(6, 12)
#define DA850_USB1_OC_PIN		GPIO_TO_PIN(6, 13)

//========================Rotary & UI Buttons===============================

#define GPIO_ROTARY_A GPIO_TO_PIN(5,9)
#define GPIO_ROTARY_B GPIO_TO_PIN(5,6)

static struct rotary_encoder_platform_data encoder_info = {
    .steps      = 30,
    .axis       = ABS_X,
    .relative_axis  = true,
    .rollover   = false,
    .gpio_a     = GPIO_ROTARY_A,
    .gpio_b     = GPIO_ROTARY_B,
    .inverted_a = 0,
    .inverted_b = 0,
    .half_period    = true,
};

static struct platform_device rotary_encoder = {
    .name       = "rotary-encoder",
    .id     = -1,
    .dev = {
        .platform_data = &encoder_info,
    }
};

#define OPTION_BUTTON   GPIO_TO_PIN(5, 12)
#define BACK_BUTTON     GPIO_TO_PIN(5, 0)
#define SELECT_BUTTON   GPIO_TO_PIN(5, 3)

static short button_pins[] = {
		DA850_GPIO5_0,	//button 1
		DA850_GPIO5_3,	//quad switch
		DA850_GPIO5_6,	//encoder 0
		DA850_GPIO5_9,	//Encoder 1
		DA850_GPIO5_12,	//button 0
		-1
};

static struct gpio_keys_button gpio_keys[] = {
        {
            .code = KEY_ENTER,
            .gpio = OPTION_BUTTON,
            .desc = "Option",
            .debounce_interval = 10,
            .type = EV_KEY,
            //.active_low = 1,
        },
        {
            .code = KEY_BACKSPACE,
            .gpio = BACK_BUTTON,
            .desc = "Back",
            .debounce_interval = 10,
            .type = EV_KEY,
            //.active_low = 1,
        },
        {
            .code = KEY_SPACE,
            .gpio = SELECT_BUTTON,
            .desc = "Select",
            .debounce_interval = 10,
            .type = EV_KEY,
            //.active_low = 1,
        },
};

struct gpio_keys_platform_data gpio_key_info = {
    .buttons    = gpio_keys,
    .nbuttons   = ARRAY_SIZE(gpio_keys),
};

struct platform_device keys_gpio = {
    .name   = "gpio-keys",
    .id = -1,
    .dev    = {
    .platform_data  = &gpio_key_info,
    },
};

//==================Wireless====================================

#define WLAN_EN GPIO_TO_PIN(5, 8)
#define WLAN_IRQ GPIO_TO_PIN(5, 14)

//Platform struct set up
struct wl12xx_platform_data mb_wireless_data = {
		.irq = -1,
		.board_ref_clock	= WL12XX_REFCLOCK_38,
		.platform_quirks	= WL12XX_PLATFORM_QUIRK_EDGE_IRQ,
};

//Pin mux
static const short mb_wireless_pins[] __initconst = {
	DA850_MMCSD0_DAT_0,
	DA850_MMCSD0_DAT_1,
	DA850_MMCSD0_DAT_2,
	DA850_MMCSD0_DAT_3,
	DA850_MMCSD0_CLK,
	DA850_MMCSD0_CMD,
	DA850_GPIO5_8,
	DA850_GPIO5_14,
	-1
};


static void wl12xx_set_power(int index, bool power_on)
{
	static bool power_state;

	pr_debug("Powering %s wl12xx", power_on ? "on" : "off");

	if (power_on == power_state)
		return;
	power_state = power_on;

	if (power_on) {
		/* Power up sequence required for wl127x devices */
		mdelay(70);
		gpio_set_value(WLAN_EN, 1);
		mdelay(70);
	} else {
		gpio_set_value(WLAN_EN, 0);
	}
}

static struct davinci_mmc_config mb_wireless_mmc_config = {
	.set_power	= wl12xx_set_power,
	.wires		= 4,
	.max_freq	= 25000000,
	.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_NONREMOVABLE |MMC_CAP_POWER_OFF_CARD,
	.version	= MMC_CTLR_VERSION_2,
};


static __init int da850_wl12xx_init(void)
{
	int ret;

	pr_debug("wl12xx: Start Pin Mux\n");
	ret = davinci_cfg_reg_list(mb_wireless_pins);
	if (ret) {
		pr_err("wl12xx/mmc mux setup failed: %d\n", ret);
		goto exit;
	}

	pr_debug("wl12xx: MMC Register\n");
	ret = da8xx_register_mmcsd0(&mb_wireless_mmc_config);
	if (ret) {
		pr_err("wl12xx/mmc registration failed: %d\n", ret);
		goto exit;
	}

	pr_debug("wl12xx: WLAN Enable GPIO\n");
	ret = gpio_request_one(WLAN_EN, GPIOF_OUT_INIT_LOW, "wl12xx_en");
	if (ret) {
		pr_err("Could not request wl12xx enable gpio: %d\n", ret);
		goto exit;
	}

	pr_debug("wl12xx: WLAN IRQ register\n");
	ret = gpio_request_one(WLAN_IRQ, GPIOF_IN, "wl12xx_irq");
	if (ret) {
		pr_err("Could not request wl12xx irq gpio: %d\n", ret);
		goto free_wlan_en;
	}

	mb_wireless_data.irq = gpio_to_irq(WLAN_IRQ);

	pr_debug("wl12xx: Set Platform data\n");
	ret = wl12xx_set_platform_data(&mb_wireless_data);
	if (ret) {
		pr_err("Could not set wl12xx data: %d\n", ret);
		goto free_wlan_irq;
	}

	return 0;

free_wlan_irq:
	gpio_free(WLAN_IRQ);

free_wlan_en:
	gpio_free(WLAN_EN);

exit:
	return ret;
}

//=============Motor Control (PRU)=============================================

//Updated for Rev C
static short stepper_pru_pins[] = {
    DA850_PRU0_R30_23,	//X step
    DA850_PRU0_R30_22,	//x dir
    DA850_PRU0_R30_24,	//x en
    DA850_PRU0_R30_25,	//x vref
    DA850_GPIO0_13,	//x load

    DA850_PRU0_R30_20,	//y step
    DA850_PRU0_R30_19,	//y dir
    DA850_PRU0_R30_17,	//y en
    DA850_PRU0_R30_16,	//y vref
    DA850_GPIO0_12,	//y load

    DA850_PRU0_R30_21,	//z step
    DA850_PRU0_R30_5,	//z dir
    DA850_PRU0_R30_4,	//z en
    DA850_PRU0_R30_3,	//z vref
    DA850_GPIO2_4,	//z load

   -1,
};


//====================Toolhead SPI=================================
//Updated for Rev C
static short toolhead_spi_pins[] = {
	DA850_SPI1_SOMI, 	//TH SOMI
	DA850_SPI1_CLK, 	//TH CLK
	DA850_SPI1_SIMO, 	//TH SIMO
	DA850_SPI1_SCS_0, 	//NOR SCS0
	DA850_GPIO1_3, 	    //TH SCS0
    DA850_PRU0_R31_10,	//TH EXP0
   	DA850_GPIO2_12,		//TH EXP1
   	DA850_GPIO6_5,		//TH0 5V on
    DA850_GPIO6_11,		//TH0 12V on
    DA850_GPIO2_0,		//TH LVDS Enable
    -1,
};

static struct davinci_spi_config toolhead_spi_cfg[] = {
	{
        .io_type	= SPI_IO_TYPE_POLL,
		.c2tdelay	= 8,
		.t2cdelay	= 8,
    },
};

static u8 spi1_chip_selects[2] = {0xFF, 19};
static struct spi_board_info toolhead_spi_info[] = {
	{
		.modalias		= "spidev",
		.controller_data	= &toolhead_spi_cfg,
		.mode			= SPI_MODE_3,
		.max_speed_hz		= 1600000,
		.bus_num		= 1,
		.chip_select		= 0,
	},
	{
		.modalias		= "spidev",
		.controller_data	= &toolhead_spi_cfg,
		.mode			= SPI_MODE_3,
		.max_speed_hz		= 1600000,
		.bus_num		= 1,
		.chip_select		= 1,
	},
};

//====================Chamber Heater===============================

#define CH_MISO     GPIO_TO_PIN(3, 4)
#define CH_MOSI     GPIO_TO_PIN(3, 2)
#define CH_SCK      GPIO_TO_PIN(0, 7)
#define CH_CS       GPIO_TO_PIN(2, 2)
#define CH_RSV0     GPIO_TO_PIN(5, 5)
#define CH_RSV1     GPIO_TO_PIN(5, 7)

static short chamber_heater_pins[] = {
	DA850_GPIO0_7,	//Chamber heater CLK
	DA850_GPIO2_2,	//Chamber heater CS
	DA850_GPIO3_4,	//Chamber heater MISO
	DA850_GPIO3_2,	//Chamber heater SOMI
	DA850_GPIO5_5,	//Chamber heater reserved 0
	DA850_GPIO5_7,	//Chamber heater reserved 1
	-1,
};

static struct spi_gpio_platform_data chamber_heater_pdata = {
        .miso                = CH_MISO,
        .mosi                = CH_MOSI,
        .sck                 = CH_SCK,
        .num_chipselect      = 1,
};

static struct platform_device chamber_heater_device = {
        .name              = "spi_gpio",
        .id                = 2,
        .dev.platform_data = &chamber_heater_pdata,
};

static struct spi_board_info chamber_heater_info[] ={
        {
                .modalias           = "spidev",
                .controller_data    = (void *)CH_CS,
                .mode               = SPI_MODE_0,     //Mode 0 for Gowanus Rev A
                .max_speed_hz       = 100000,         //100Khz, above this the ATtiny24 USI module starts getting bad data
                .bus_num            = 2,
                .chip_select        = 0,
        },
};

static __init int chamber_heater_init(void){

        int ret;

        pr_debug("Chamber heater pin mux\n");
        ret = davinci_cfg_reg_list(chamber_heater_pins);
        if(ret){
                pr_err("ERROR pin mux setup failed: %d\n", ret);
                goto exit;
        }

        pr_debug("Chamber heater SPIDEV register\n");
        ret = spi_register_board_info(chamber_heater_info, ARRAY_SIZE(chamber_heater_info));
        if (ret) {
                pr_err("ERROR SPIDEV registration failed %d\n", ret);
                goto exit;
        }

        pr_debug("Chamber heater platform register\n");
        ret = platform_device_register(&chamber_heater_device);
        if(ret){
                pr_warn("ERROR platform device registration failed %d\n", ret);
                goto exit;
        }

        return 0;
exit:
        return ret;

}


//====================Power monitor I2C===========================
static short power_monitor_i2c_pins[] = {
    DA850_GPIO0_5,        // POWER_SCL
    DA850_GPIO0_6,        // POWER_SDA
    -1,
};

static struct i2c_gpio_platform_data power_monitor_i2c_gpio_pdata = {
    .sda_pin                = GPIO_TO_PIN(0, 6),   // POWER_SDA
    .scl_pin                = GPIO_TO_PIN(0, 5),   // POWER_SCL
    //.sda_is_open_drain    = 1,
    //.scl_is_open_drain    = 1,
    //.udelay               = 2,
};

static struct i2c_board_info power_monitor_i2c_info[] = {
    {
        I2C_BOARD_INFO("power_monitor_12_i2c", 0x40),
        .platform_data = &power_monitor_i2c_gpio_pdata,
    },
};

static struct platform_device power_monitor_12v_gpio_i2c_device = {
    .name                 = "i2c-gpio",
    .id                   = 3,
    .dev                  =
    {
        .platform_data    = &power_monitor_i2c_gpio_pdata,
    },
};

static struct platform_device *power_monitor_devices[] __initdata = {
    &power_monitor_12v_gpio_i2c_device,
};


//====================12V Control=================================

#define DA850_12V_POWER_PIN  GPIO_TO_PIN(0,8)

static short mb_power_pins[] = {
	DA850_GPIO0_8,  	//12V Power
	DA850_GPIO0_6,  	//Power monitor SDA
	DA850_GPIO0_5,  	//Power monitor SCL
	-1,
};

static void da850_12V_power_control(int val)
{
	/* 12V power rail */
	pr_debug("12V Power: %d\n", val);
	gpio_set_value(DA850_12V_POWER_PIN, val);
}

static int da850_power_init(void)
{
	int status;

	status = gpio_request(DA850_12V_POWER_PIN, "12V power\n");
	if (status < 0)
		return status;

	gpio_direction_output(DA850_12V_POWER_PIN, 0);

	da850_12V_power_control(1);

	return 0;
}

//====================LCD Configuration=================================

#define	LCD_BACKLIGHT		GPIO_TO_PIN(8, 10)
#define LCD_RESET		GPIO_TO_PIN(6, 3)
#define LCD_DISPLAY_TYPE	GPIO_TO_PIN(4, 0)
#define LCD_CLK			GPIO_TO_PIN(6, 2)
#define LCD_CS			GPIO_TO_PIN(6, 1)
#define LCD_SIMO		GPIO_TO_PIN(6, 4)

static short mb_lcd_pins[] = {
	DA850_GPIO6_3,		//LCD Reset
	DA850_GPIO4_0,      	//LCD Detect Type
	DA850_GPIO8_10,		//LCD Backlight (GPIO)
	DA850_GPIO6_4,		//LCD SIMO
	DA850_GPIO6_2,		//LCD SCK
	DA850_GPIO6_1,		//LCD SCS
	-1,
};

static short interface_i2c_pins[] = {
	DA850_I2C0_SDA,
	DA850_I2C0_SCL,
	-1,
};

//Platform data set up
static struct da8xx_spi_pin_data lcd_spi_gpio_data = {
	.sck = LCD_CLK,
	.sdi = LCD_SIMO,
	.cs = LCD_CS,
};

struct da8xx_lcdc_spi_platform_data *lcd_pdata;

static struct davinci_i2c_platform_data mb_i2c0_pdata = {
	.bus_freq	= 400,	/* kHz */
	.bus_delay	= 0,	/* usec */
};

//TODO this should be broken out to a dimmable driver
static void mb_lcd_enable_backlight(struct work_struct *work){
	gpio_set_value(LCD_BACKLIGHT, 1);
}

DECLARE_DELAYED_WORK(backlight_work, &mb_lcd_enable_backlight);

static void da850_panel_power_ctrl(int val)
{
	/* lcd_reset */
	gpio_set_value(LCD_RESET, val);

	pr_debug("switching lcd power to : %d\n", val);

	/* lcd backlight */
	if(val){	//Need to wait before turning on the backlight
		schedule_delayed_work(&backlight_work, msecs_to_jiffies(1000)); //delay in jiffies
	}
	else{		//Else we can turn it off immediately
		gpio_set_value(LCD_BACKLIGHT, 0);
	}
}



static int da850_lcd_hw_init(void)
{
	int status;

	pr_debug("LCD: register backlight\n");
	status = gpio_request_one(LCD_BACKLIGHT, GPIOF_OUT_INIT_LOW, "lcd backlight");
	if(status < 0)
		return status;

	pr_debug("LCD register reset\n");
	status = gpio_request(LCD_RESET, "lcd reset\n");
	if (status < 0)
		return status;

	gpio_direction_output(LCD_RESET, 0);

	pr_debug("LCD register display type\n");
	status = gpio_request(LCD_DISPLAY_TYPE, "lcd type\n");
	if (status < 0)
		return status;

    // 0 level for type pin indicates AZ display
	if(gpio_get_value(LCD_DISPLAY_TYPE) == 0) {
		lcd_pdata = &az_hx8238_pdata;
	} else {
		lcd_pdata = &ssd2119_spi_pdata;
	}

	/* Switch off panel power and backlight */
	//TODO might not need/want this
	da850_panel_power_ctrl(0);

	return 0;
}

static __init int mb_lcd_init(void){

	int ret;

	//Configure LCD controler pins
	ret = davinci_cfg_reg_list(da850_lcdcntl_pins);	
	if (ret){
		pr_warn("%s: LCDC pin mux setup failed: %d\n", __func__, ret);
//		return ret;
	}

	//Configure LCD power and configuration (SPI) pins
	ret = davinci_cfg_reg_list(mb_lcd_pins);
	if (ret){
		pr_warn("%s: LCD pins initialization failed: %d\n", __func__, ret);
	//TODO unregister lcdcntl pins
//		return ret;
	}

	//Set up the LCD power and config pins
	ret = da850_lcd_hw_init();
	if (ret){
		pr_warn("%s: LCD initialization failed: %d\n", __func__, ret);
	//TODO unregister lcdcntl
	//TODO unregister lcd power 
//		return ret;
	}

	//Associate power control functions with platform data
	lcd_pdata->panel_power_ctrl = da850_panel_power_ctrl;

	//Associate set up SPI with 
	lcd_pdata->spi = &lcd_spi_gpio_data;
	ret = da8xx_register_lcdc_spi(lcd_pdata);
	if (ret){
		pr_warn("%s: LCDC registration failed: %d\n", __func__, ret);
	//TODO unregister lcdcntl
	//TODO unregister lcd power
//		return ret;
	}

	//Configure LCD power / config pins

	ret = davinci_cfg_reg_list(interface_i2c_pins);
	if (ret){
		pr_warn("%s: LCD i2c pins initialization failed: %d\n", __func__, ret);
	//TODO undo stuff
		return ret;
	}

	//Register the I2C bus?
	ret = da8xx_register_i2c(0, &mb_i2c0_pdata);
	if (ret){
		pr_warn("%s: LCD i2c driver initialization failed: %d\n", __func__, ret);
	//TODO undo thigns
//		return ret;
	}

	return ret;
}


//====================LED Indicator Configuration================================

const short mb_manhattan_led_pins[] = {
	DA850_GPIO6_14,		//LED Status
	DA850_PRU0_R30_14,	//PRU LED0
	DA850_PRU0_R30_13,	//PRU LED1
    -1
};


static struct gpio_led gpio_leds[] = {
	{
		.name           = "Kernel_Status",
		.gpio           = GPIO_TO_PIN(6,14),
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

//====================Buzzer==================================================
#define BUZZER_OUT GPIO_TO_PIN(2,15)

const short buzzer_pins[] = {
	DA850_GPIO2_15,		//GPIO
	-1
};

static __init int buzzer_init(void){
	int ret;
	ret = 0;

	pr_debug("buzzer: Pin Mux\n");
	ret = davinci_cfg_reg_list(buzzer_pins);
	if(ret){
		pr_err("ERROR: Buzzer pin mux failed: %d\n", ret);
		return ret;
	}

	pr_debug("buzzer: Output pin request\n");
	ret = gpio_request_one(BUZZER_OUT, GPIOF_OUT_INIT_LOW, "buzzer_out");
	if(ret){
		pr_err("ERROR: Could not request buzzer output gpio: %d\n", ret);
		goto exit;
	}

	//platform data
	pr_debug("buzzer: init finished\n");
	return ret;

exit:
	gpio_free(BUZZER_OUT);
	return ret;
}

//====================NAND Flash Configuration=================================

static const short nand_pins[] = {
	DA850_EMA_D_0, DA850_EMA_D_1, DA850_EMA_D_2, DA850_EMA_D_3,
	DA850_EMA_D_4, DA850_EMA_D_5, DA850_EMA_D_6, DA850_EMA_D_7,
	DA850_EMA_A_1, DA850_EMA_A_2, DA850_NEMA_CS_3, DA850_NEMA_CS_2,
	DA850_NEMA_WE, DA850_NEMA_OE, DA850_EMA_WAIT_0, DA850_EMA_WAIT_1,
	-1
};

static struct mtd_partition da850_evm_nandflash_partition[] = {
	{
		.name		= "root",
		.offset		= 0,
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
	.ta			= 20,
};

static struct davinci_nand_pdata da850_evm_nandflash_data = {
	.parts			= da850_evm_nandflash_partition,
	.nr_parts		= ARRAY_SIZE(da850_evm_nandflash_partition),
	.ecc_mode		= NAND_ECC_SOFT_BCH,
	.ecc_bits		= 24,
	.bbt_options	= NAND_BBT_USE_FLASH,
	.timing			= &da850_evm_nandflash_timing,
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
	.id			= 1,
	.dev		= {
		.platform_data	= &da850_evm_nandflash_data,
	},
	.num_resources	= ARRAY_SIZE(da850_evm_nandflash_resource),
	.resource		= da850_evm_nandflash_resource,
};

static struct platform_device *da850_evm_devices[] = {
	&da850_evm_nandflash_device,
};

//====================Ethernet Configuration=================================

static short mb_manhattan_mii_pins[] __initdata = {
	DA850_MII_TXEN,
	DA850_MII_TXCLK,
	DA850_MII_COL,
	DA850_MII_TXD_3,
	DA850_MII_TXD_2,
	DA850_MII_TXD_1,
	DA850_MII_TXD_0,
	DA850_MII_RXER,
	DA850_MII_CRS,
	DA850_MII_RXCLK,
	DA850_MII_RXDV,
	DA850_MII_RXD_3,
	DA850_MII_RXD_2,
	DA850_MII_RXD_1,
	DA850_MII_RXD_0,
	DA850_MDIO_CLK,
	DA850_MDIO_D,
	-1
};

static __init void mb_manhattan_config_emac(void)
{
	void __iomem *cfgchip3 = DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP3_REG);
	int ret;
	u32 val;
	struct davinci_soc_info *soc_info = &davinci_soc_info;
    char eth[17];
    int mac_byte_counter;
    char *eth_ptr;
    unsigned char mac_addr[6];
    static char *ptr __initdata = NULL;
    char mac_bytes[3] = { 0};
    long temp_long = 0;

	val = __raw_readl(cfgchip3);
	val &= ~BIT(8);
	ret = davinci_cfg_reg_list(mb_manhattan_mii_pins);
	if (ret) {
		pr_warn("%s: CPGMAC/MII mux setup failed: %d\n", __func__, ret);
		return;
	}

	/* configure the CFGCHIP3 register for MII */
	__raw_writel(val, cfgchip3);
	pr_debug("EMAC: MII PHY configured\n");

	soc_info->emac_pdata->phy_id = MANHATTAN_PHY_ID;

	// get mac address
	ptr = strstr(boot_command_line, "eth=");

	if (ptr) {
		memcpy(eth, ptr+4, 17*sizeof(char));
        eth_ptr = eth;
        for (mac_byte_counter = 0; mac_byte_counter <= 5; mac_byte_counter ++) {
            mac_bytes[0] = *eth_ptr;
            mac_bytes[1] = *(eth_ptr + 1);
            ret = kstrtol(mac_bytes, 16, &temp_long);
            if (ret) {
                pr_warn("Error parsing mac address: %d\n", ret);
            }else {
                mac_addr[mac_byte_counter] = (uint8_t)(temp_long);
            }
            //pr_warn( "mac_addr:%d %2x\n", mac_byte_counter, mac_addr[mac_byte_counter] );
            eth_ptr+=3; /* skip ":" in  eth*/
        }
    }
    if (is_valid_ether_addr(mac_addr)) {
        //pr_warn("valid ethernet addr received from init\n");
        memcpy(da8xx_emac_pdata.mac_addr, mac_addr, ETH_ALEN);
    }

	ret = da8xx_register_emac();
	if (ret)
		pr_warn("%s: EMAC registration failed: %d\n", __func__, ret);
}


//====================USB Configuration=================================

static irqreturn_t mb_manhattan_usb_ocic_irq(int irq, void *dev_id);
static da8xx_ocic_handler_t hawk_usb_ocic_handler;

static const short da850_hawk_usb11_pins[] = {
	DA850_GPIO6_13,		//USB1 Overcurrent
	DA850_GPIO6_12,		//USB1 Drive Bus
	-1
};
//Set the VBus pin
static int hawk_usb_set_power(unsigned port, int on)
{
	gpio_set_value(DA850_USB1_VBUS_PIN, on);
	return 0;
}
//Get state of VBus pin
static int hawk_usb_get_power(unsigned port)
{
	return gpio_get_value(DA850_USB1_VBUS_PIN);
}
//Get state of the Overcurrent Pin
static int hawk_usb_get_oci(unsigned port)
{
	return !gpio_get_value(DA850_USB1_OC_PIN);
}
//Overcurrent handler
static int hawk_usb_ocic_notify(da8xx_ocic_handler_t handler)
{
	int irq         = gpio_to_irq(DA850_USB1_OC_PIN);
	int error       = 0;

	if (handler != NULL) {
		hawk_usb_ocic_handler = handler;

		error = request_irq(irq, mb_manhattan_usb_ocic_irq,
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

static struct da8xx_ohci_root_hub mb_manhattan_usb11_pdata = {
	.set_power      = hawk_usb_set_power,
	.get_power      = hawk_usb_get_power,
	.get_oci        = hawk_usb_get_oci,
	.ocic_notify    = hawk_usb_ocic_notify,
	/* TPS2087 switch @ 5V */
	.potpgt         = (3 + 1) / 2,  /* 3 ms max */
};

static irqreturn_t mb_manhattan_usb_ocic_irq(int irq, void *dev_id)
{
	hawk_usb_ocic_handler(&mb_manhattan_usb11_pdata, 1);
	return IRQ_HANDLED;
}

static __init void mb_manhattan_usb_init(void)
{
	int ret;
	u32 cfgchip2;

	ret = davinci_cfg_reg_list(da850_hawk_usb11_pins);
	if (ret) {
		pr_warn("%s: USB 1.1 PinMux setup failed: %d\n", __func__, ret);
		return;
	}

	ret = gpio_request_one(DA850_USB1_VBUS_PIN, GPIOF_DIR_OUT, "USB1 VBUS");
	if (ret < 0) {
		pr_err("%s: failed to request GPIO for USB 1.1 port "
			"power control: %d\n", __func__, ret);
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
        pr_warning("%s: USB 2.0 registration failed: %d\n",__func__, ret);


	ret = gpio_request_one(DA850_USB1_OC_PIN,
			GPIOF_DIR_IN, "USB1 OC");
	if (ret < 0) {
		pr_err("%s: failed to request GPIO for USB 1.1 port "
			"over-current indicator: %d\n", __func__, ret);
		goto usb11_setup_oc_fail;
	}

	ret = da8xx_register_usb11(&mb_manhattan_usb11_pdata);
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

//====================UART Configuration=================================

static struct davinci_uart_config mb_manhattan_uart_config __initdata = {
	.enabled_uarts = 0x7,
};

static const short uart_pins[]  ={
	DA850_UART2_TXD,
	-1
};



static __init void mb_manhattan_init(void)
{
	int ret;
	u32 cfgchip3;

	/*UART*/
	ret = davinci_cfg_reg_list(uart_pins);
	if(ret)
		pr_warn("%s: UART 2 pin mux failed: %d\n", __func__, ret);

	davinci_serial_init(&mb_manhattan_uart_config);		//Configure the serial port interface

	/*Ethernet*/
	mb_manhattan_config_emac();							//Configure Ethernet

	/*DMA*/
	ret = da850_register_edma(0);						//Register Ethernet with kernel
	if (ret)
		pr_warn("%s: EDMA registration failed: %d\n", __func__, ret);

	/* LCD  */
	ret = mb_lcd_init();
	if(ret)
		pr_warn("Error: Could not register LCD %d\n", ret);

	/*Wireless*/
	ret = da850_wl12xx_init();							//Configure and register Wifi
	if (ret)
		pr_warn("%s: WL12xx initialization failed: %d\n",__func__, ret);

	/*USB*/
	mb_manhattan_usb_init();							//Init USB

	/*Power Control*/
	ret = davinci_cfg_reg_list(mb_power_pins);			//Register power pins (+12v on)
	if (ret)
        	pr_warn("%s: power pin setup failed!: %d\n", __func__, ret);

	ret = da850_power_init();							//Init power pins
	if (ret)
		pr_warn("%s: power pin init failed!: %d\n", __func__, ret);

	/*NAND Flash*/
    ret = davinci_cfg_reg_list(nand_pins);
	if (ret)
		pr_warn("%s: nand pin init failed!!!!!!!!!!!: %d\n", __func__, ret);

	platform_add_devices(da850_evm_devices, ARRAY_SIZE(da850_evm_devices));		//add NAND storage

	/*Toolhead SPI*/
	ret = davinci_cfg_reg_list(toolhead_spi_pins);								//Configure Toolhead Pins
	if (ret)
		pr_warn("%s: Toolhead spi mux setup failed: %d\n", __func__, ret);

	ret = spi_register_board_info(toolhead_spi_info, ARRAY_SIZE(toolhead_spi_info));	//Register the pins
	if (ret)
		pr_warn("%s: spi info registration failed: %d\n", __func__, ret);

    da8xx_spi_pdata[1].chip_sel = spi1_chip_selects;
    ret = da8xx_register_spi_bus(1,2);
	if (ret)
		pr_warn("%s: SPI 1 registration failed: %d\n", __func__, ret);

    /*Power monitor I2C*/
    ret = davinci_cfg_reg_list(power_monitor_i2c_pins);
    if (ret)
        pr_warn("%s: Power monitor I2C mux setup failed: %d\n", __func__, ret);

    ret = i2c_register_board_info(3, power_monitor_i2c_info, ARRAY_SIZE(power_monitor_i2c_info));
    if (ret)
        pr_warn("%s: i2c info registration failed: %d\n", __func__, ret);

    ret = platform_add_devices(power_monitor_devices, ARRAY_SIZE(power_monitor_devices));
    if (ret)
        pr_warn("%s: i2c platform add devices failed: %d\n", __func__, ret);

	//Chamber Heater SPI
    ret = chamber_heater_init();
    if(ret)
        pr_warn("Error: could not register chamber heater %d\n", ret);



	/*Watchdog*/
	ret = da8xx_register_watchdog();							//Register the watchdog
	if (ret)
		pr_warn("%s: watchdog registration failed: %d\n", __func__, ret);

	/*LEDs*/
	ret = davinci_cfg_reg_list(mb_manhattan_led_pins);			//Configure LED pins
	if (ret)
		pr_warn("mb_manhattan_init: LED pinmux failed: %d\n", ret);

  	ret = platform_device_register(&leds_gpio);						//Register LED pin
	if (ret)
		pr_warn("da850_evm_init: led device initialization failed: %d\n", ret);

	/*Buzzer*/
	buzzer_init();

	/* Setup alternate events on the PRUs */
	cfgchip3 = __raw_readl(DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP3_REG));		//Read from one of the base registers
	cfgchip3 |=  BIT(3);												//Change it
	__raw_writel(cfgchip3, DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP3_REG));		//Write it back

	/* UI Buttons */
	ret = davinci_cfg_reg_list(button_pins);							//Configure the UI pins
	if (ret)
		pr_warn("%s: button pins initialization failed: %d\n", __func__, ret);

	ret = platform_device_register(&keys_gpio);							//register UI pins
	if (ret)
		pr_warn("%s: gpio key pins device initialization failed!: %d\n", __func__, ret);

	/*Encoder */
	ret = platform_device_register(&rotary_encoder);					//Register the encoder
	if (ret)
		pr_warn("%s: rotary encoder device initialization failed!: %d\n", __func__, ret);

	/*PRUs */
	ret = davinci_cfg_reg_list(stepper_pru_pins);						//Configure PRU pins
	if (ret)
		pr_warn("%s: stepper pins initialization failed: %d\n", __func__, ret);

	 // Disable pull-ups on MS2/Sense inputs on the steppers (CP0 & CP16 in PUPD_ENA reg)
	// default pull up configuration: 0xC3FFFFFF
	__raw_writel(0xCFFEFFFE,  ioremap(DA8XX_PUPD_ENA, SZ_1K) );

	/* Register PRUSS device */
	da8xx_register_uio_pruss();
	if (ret)
		pr_warn("pruss init failed %d\n", ret);

	/* read the pruss clock */
	davinci_psc_is_clk_active(0,13);
}

#ifdef CONFIG_SERIAL_8250_CONSOLE
static int __init mb_manhattan_console_init(void)
{
	return add_preferred_console("ttyS", 1, "115200");
}
console_initcall(mb_manhattan_console_init);
#endif

static void __init mb_manhattan_map_io(void)
{
	da850_init();			//This handles a lot of the system level config (PLLs, etc)
}

MACHINE_START(DAVINCI_MANHATTAN, "Makerbot Controller Manhattan on DaVinci AM18xx")
	.atag_offset	= 0x100,
	.map_io		= mb_manhattan_map_io,
	.init_irq	= cp_intc_init,
	.timer		= &davinci_timer,
	.init_machine	= mb_manhattan_init,
	.init_late	= davinci_init_late,
	.dma_zone_size	= SZ_128M,
	.restart	= da8xx_restart,
MACHINE_END
