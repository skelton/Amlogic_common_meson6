/*
 * arch/arm/mach-meson6/board-meson6-ref.c
 *
 * Copyright (C) 2011-2012 Amlogic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/reboot.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <plat/platform.h>
#include <plat/plat_dev.h>
#include <plat/platform_data.h>

#include <linux/io.h>
#include <plat/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/device.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <plat/platform.h>
#include <plat/plat_dev.h>
#include <plat/platform_data.h>
#include <plat/lm.h>
#include <plat/regops.h>
#include <linux/io.h>
#include <plat/io.h>

#include <mach/map.h>
#include <mach/i2c_aml.h>
#include <mach/nand.h>
#include <mach/usbclock.h>
#include <mach/usbsetting.h>
#include <mach/pm.h>
#include <mach/gpio_data.h>
#include <mach/pinmux.h>

#include <linux/uart-aml.h>
#include <linux/i2c-aml.h>
#include "board-m6g12-M805.h"

#ifdef CONFIG_MMC_AML
#include <mach/mmc.h>
#endif

#ifdef CONFIG_CARDREADER
#include <mach/card_io.h>
#endif

#include <mach/gpio.h>
#ifdef CONFIG_EFUSE
#include <linux/efuse.h>
#endif


#ifdef CONFIG_AW_AXP
#include <linux/power_supply.h>
#include <linux/apm_bios.h>
#include <linux/apm-emulation.h>
#include <linux/regulator/machine.h>
#include <mach/irqs.h>
#include "../../../drivers/amlogic/power/axp_power/axp-gpio.h"
#include "../../../drivers/amlogic/power/axp_power/axp-mfd.h"
#endif


#ifdef CONFIG_SND_SOC_RT5631
#include <sound/rt5631.h>
#endif

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE
#include <media/amlogic/aml_camera.h>
#endif

#ifdef CONFIG_AM_WIFI
#include <plat/wifi_power.h>
#endif

#ifdef CONFIG_AML_HDMI_TX
#include <plat/hdmi_config.h>
#endif

//#define CONFIG_AMLOGIC_MICRON_NYNIX_NAND_4_CHIPS		

static struct resource meson_fb_resource[] = {
    [0] = {
        .start = OSD1_ADDR_START,
        .end   = OSD1_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = OSD2_ADDR_START,
        .end   = OSD2_ADDR_END,
        .flags = IORESOURCE_MEM,
    },

};

#if defined(CONFIG_AM_FB_EXT)
static struct resource meson_fb_ext_resource[] = {
    [0] = {
        .start = OSD3_ADDR_START,
        .end   = OSD3_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = OSD4_ADDR_START,
        .end   = OSD4_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};
#endif

static struct resource meson_codec_resource[] = {
    [0] = {
        .start = CODEC_ADDR_START,
        .end   = CODEC_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = STREAMBUF_ADDR_START,
        .end   = STREAMBUF_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

#ifdef CONFIG_POST_PROCESS_MANAGER
static struct resource ppmgr_resources[] = {
    [0] = {
        .start = PPMGR_ADDR_START,
        .end   = PPMGR_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device ppmgr_device = {
    .name       = "ppmgr",
    .id         = 0,
    .num_resources = ARRAY_SIZE(ppmgr_resources),
    .resource      = ppmgr_resources,
};
#endif

#if  defined(CONFIG_AM_TV_OUTPUT2)
static struct resource vout2_device_resources[] = {
    [0] = {
        .start = 0,
        .end   = 0,
        .flags = IORESOURCE_MEM,
    },
};
static struct platform_device vout2_device = {
    .name       = "mesonvout2",
    .id         = 0,
    .num_resources = ARRAY_SIZE(vout2_device_resources),
    .resource      = vout2_device_resources,
};
#endif
static  int __init setup_devices_resource(void)
{
    setup_fb_resource(meson_fb_resource, ARRAY_SIZE(meson_fb_resource));
#if defined(CONFIG_AM_FB_EXT)
    setup_fb_ext_resource(meson_fb_ext_resource, ARRAY_SIZE(meson_fb_ext_resource));
#endif    
#ifdef CONFIG_AM_STREAMING
    setup_codec_resource(meson_codec_resource, ARRAY_SIZE(meson_codec_resource));
#endif
    return 0;
}

/***********************************************************************
 * Sensors Section
 **********************************************************************/


#ifdef CONFIG_BOSCH_BMA222
struct gsen_platform_data{
	int	rotator[9];
	};

static struct gsen_platform_data gsen_pdata={
		.rotator={-1,0,0,0,-1,0,0,0,1},
	};
#endif
#if defined (CONFIG_AML_HDMI_TX)
static void m6ref_hdmi_5v_ctrl(unsigned int pwr)
{
    if(pwr){
        printk("HDMI 5V Power On--Refer to board\n");
    }
    else{
        printk("HDMI 5V Power Off--Refer to board\n");
    }
}

static struct hdmi_phy_set_data brd_phy_data[] = {
    {27, 0x16, 0x30},   // 480i/p 576i/p
    {74, 0x16, 0x40},   // 720p 1080i
    {148, 0x16, 0x40},  // 1080p
    {-1,   -1},         //end of phy setting
};

static struct hdmi_config_platform_data aml_hdmi_pdata = {
    .hdmi_5v_ctrl = m6ref_hdmi_5v_ctrl,
    .hdmi_3v3_ctrl = NULL,
    .hdmi_pll_vdd_ctrl = NULL,
    .phy_data = brd_phy_data,
};
#endif

#ifdef CONFIG_AW_AXP20
static struct i2c_board_info __initdata aml_i2c_bus_info_ao[];

extern void axp_power_off(void);

/* Reverse engineered partly from Platformx drivers */
enum axp_regls {

    vcc_ldo1,
    vcc_ldo2,
    vcc_ldo3,
    vcc_ldo4,
    vcc_ldo5,

    vcc_buck2,
    vcc_buck3,
    vcc_ldoio0,
};

/* The values of the various regulator constraints are obviously dependent
 * on exactly what is wired to each ldo.  Unfortunately this information is
 * not generally available.  More information has been requested from Xbow
 * but as of yet they haven't been forthcoming.
 *
 * Some of these are clearly Stargate 2 related (no way of plugging
 * in an lcd on the IM2 for example!).
 */

static struct regulator_consumer_supply ldo1_data[] = {
    {
        .supply = "VDD_RTC",
    },
};


static struct regulator_consumer_supply ldo2_data[] = {
    {
        .supply = "VDDIO_AO",
    },
};

static struct regulator_consumer_supply ldo3_data[] = {
    {
        .supply = "AVDD2.5V",
    },
};

static struct regulator_consumer_supply ldo4_data[] = {
    {
        .supply = "AVDD3.0V",
    },
};

static struct regulator_consumer_supply ldoio0_data[] = {
    {
        .supply = "POWER_MISC",
    },
};


static struct regulator_consumer_supply buck2_data[] = {
    {
        .supply = "DDR3_1.5V",
    },
};

static struct regulator_consumer_supply buck3_data[] = {
    {
        .supply = "VDD_AO",
    },
};

static struct battery_parameter g24_battery = {
    .pmu_twi_id = 2,		//AXP20_I2CBUS
    .pmu_irq_id = INT_WATCHDOG,
    .pmu_twi_addr = AXP20_ADDR,
    .pmu_battery_rdc = 91,
    .pmu_battery_cap = 3850,
    .pmu_battery_technology = POWER_SUPPLY_TECHNOLOGY_LiFe,
    .pmu_battery_name = "PTI PL336078",
    .pmu_init_chgvol = 4200000,			//set initial charing target voltage
    .pmu_init_chgend_rate = 10,		//set initial charing end current	rate
    .pmu_init_chg_enabled = 1,		//set initial charing enabled
    .pmu_init_adc_freq = 25,		//set initial adc frequency
    .pmu_init_adc_freqc = 100,		//set initial coulomb adc coufrequency
    .pmu_init_chg_pretime = 50,		//set initial pre-charging time
    .pmu_init_chg_csttime = 480,		//set initial pre-charging time
#ifdef CONFIG_SUPPORT_USB_BURNING
    .pmu_init_chgcur = 0,		//set initial charging current limite
    .pmu_suspend_chgcur = 0,	//set suspend charging current limite
    .pmu_resume_chgcur = 0,							//set resume charging current limite
    .pmu_shutdown_chgcur = 0,		//set shutdown charging current limite
    .pmu_usbcur_limit = 0,
    .pmu_usbcur = 500,
#else
    .pmu_init_chgcur = 500000,		//set initial charging current limite
    .pmu_suspend_chgcur = 1000000,	//set suspend charging current limite
    .pmu_resume_chgcur = 500000,							//set resume charging current limite
    .pmu_shutdown_chgcur = 1000000,		//set shutdown charging current limite
    .pmu_usbcur_limit = 0,
    .pmu_usbcur = 500,
#endif
    .pmu_usbvol_limit = 1,
    .pmu_usbvol = 4400,
    .pmu_pwroff_vol = 2600,
    .pmu_pwron_vol = 2600,
    .pmu_pekoff_time = 6000,
    .pmu_pekoff_en  = 1,
    .pmu_peklong_time = 1500,
    .pmu_pwrok_time   = 64,
    .pmu_pwrnoe_time = 2000,
    .pmu_intotp_en = 1,
    .pmu_pekon_time = 128,		//powerkey hold time for power on
    .pmu_charge_efficiency = 92,
    .pmu_bat_curve = {
        // ocv, charge, discharge
        {3132,      0,      0},
        {3273,      0,      0},
        {3414,      0,      0},
        {3555,      5,     13},
        {3625,     13,     20},
        {3660,     17,     23},
        {3696,     19,     27},
        {3731,     23,     32},
        {3766,     27,     46},
        {3801,     42,     56},
        {3836,     52,     62},
        {3872,     61,     68},
        {3942,     73,     77},
        {4012,     82,     86},
        {4083,     91,     94},
        {4153,    100,    100}
    }, 
};

static struct regulator_init_data axp_regl_init_data[] = {
    [vcc_ldo1] = {
        .constraints = { /* board default 1.25V */
            .name = "axp20_ldo1",
            .min_uV =  1300 * 1000,
            .max_uV =  1300 * 1000,
        },
        .num_consumer_supplies = ARRAY_SIZE(ldo1_data),
        .consumer_supplies = ldo1_data,
    },
    [vcc_ldo2] = {
        .constraints = { /* board default 3.0V */
            .name = "axp20_ldo2",
            .min_uV = 1800000,
            .max_uV = 3300000,
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .initial_state = PM_SUSPEND_STANDBY,
            .state_standby = {
                //.uV = ldo2_vol * 1000,
                .enabled = 1,
            }
        },
        .num_consumer_supplies = ARRAY_SIZE(ldo2_data),
        .consumer_supplies = ldo2_data,
    },
    [vcc_ldo3] = {
        .constraints = {/* default is 1.8V */
            .name = "axp20_ldo3",
            .min_uV =  700 * 1000,
            .max_uV =  3500* 1000,
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .initial_state = PM_SUSPEND_STANDBY,
            .state_standby = {
                //.uV = ldo3_vol * 1000,
                .enabled = 1,
            }
        },
        .num_consumer_supplies = ARRAY_SIZE(ldo3_data),
        .consumer_supplies = ldo3_data,
    },
    [vcc_ldo4] = {
        .constraints = {
            /* board default is 3.3V */
            .name = "axp20_ldo4",
            .min_uV = 1250000,
            .max_uV = 3300000,
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .initial_state = PM_SUSPEND_STANDBY,
            .state_standby = {
                //.uV = ldo4_vol * 1000,
                .enabled = 1,
            }
        },
        .num_consumer_supplies = ARRAY_SIZE(ldo4_data),
        .consumer_supplies = ldo4_data,
    },
    [vcc_buck2] = {
        .constraints = { /* default 1.24V */
            .name = "axp20_buck2",
            .min_uV = 700 * 1000,
            .max_uV = 2275 * 1000,
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .initial_state = PM_SUSPEND_STANDBY,
            .state_standby = {
                .uV = 1500 * 1000,  //axp_cfg.dcdc2_vol * 1000,
                .enabled = 1,
            }
        },
        .num_consumer_supplies = ARRAY_SIZE(buck2_data),
        .consumer_supplies = buck2_data,
    },
    [vcc_buck3] = {
        .constraints = { /* default 2.5V */
            .name = "axp20_buck3",
            .min_uV = 700 * 1000,
            .max_uV = 3500 * 1000,
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .initial_state = PM_SUSPEND_STANDBY,
            .state_standby = {
                .uV = 1100 * 1000,  //axp_cfg.dcdc3_vol * 1000,
                .enabled = 1,
            }
        },
        .num_consumer_supplies = ARRAY_SIZE(buck3_data),
        .consumer_supplies = buck3_data,
    },
    [vcc_ldoio0] = {
        .constraints = { /* default 2.5V */
            .name = "axp20_ldoio0",
            .min_uV = 1800 * 1000,
            .max_uV = 3300 * 1000,
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
        },
        .num_consumer_supplies = ARRAY_SIZE(ldoio0_data),
        .consumer_supplies = ldoio0_data,
    },
};

static struct axp_funcdev_info axp_regldevs[] = {
    {
        .name = "axp20-regulator",
        .id = AXP20_ID_LDO1,
        .platform_data = &axp_regl_init_data[vcc_ldo1],
    }, {
        .name = "axp20-regulator",
        .id = AXP20_ID_LDO2,
        .platform_data = &axp_regl_init_data[vcc_ldo2],
    }, {
        .name = "axp20-regulator",
        .id = AXP20_ID_LDO3,
        .platform_data = &axp_regl_init_data[vcc_ldo3],
    }, {
        .name = "axp20-regulator",
        .id = AXP20_ID_LDO4,
        .platform_data = &axp_regl_init_data[vcc_ldo4],
    }, {
        .name = "axp20-regulator",
        .id = AXP20_ID_BUCK2,
        .platform_data = &axp_regl_init_data[vcc_buck2],
    }, {
        .name = "axp20-regulator",
        .id = AXP20_ID_BUCK3,
        .platform_data = &axp_regl_init_data[vcc_buck3],
    }, {
        .name = "axp20-regulator",
        .id = AXP20_ID_LDOIO0,
        .platform_data = &axp_regl_init_data[vcc_ldoio0],
    },
};

static int g24_pmu_call_back(void *para)
{
    // add your call back implement here, more information please see file 'axp-mfd.h'    
}

static struct axp_supply_init_data axp_sply_init_data = {
	/*
	 * if you have board specific call functions, add them here
	 */
  //.led_control     =  led_control,
    .soft_limit_to99 =  0,
    .para            =  NULL,
    .pmu_call_back   =  g24_pmu_call_back,
    .board_battery   = &g24_battery,
};

static struct axp_funcdev_info axp_splydev[]={
    {
        .name = "axp20-supplyer",
            .id = AXP20_ID_SUPPLY,
      .platform_data = &axp_sply_init_data,
    },
};
#ifdef CONFIG_SUPPORT_USB_BURNING
static axp_gpio_cfg_t axp_init_gpio_cfg[] = {
		{
        .gpio = AXP_GPIO0,			//AXP202 GPIO0 ==> 3G VCC 
        .dir = AXP_GPIO_OUTPUT,
        .level = AXP_GPIO_HIGH,		//set AXP202 GPIO0 high
    },
    {
        .gpio = AXP_GPIO1,			//AXP202 GPIO1 ==> VCCX2 
        .dir = AXP_GPIO_OUTPUT,
        .level = AXP_GPIO_HIGH,		//set AXP202 GPIO1 high
    },
    {
        .gpio = AXP_GPIO2,			//AXP202 GPIO2 ==> HEMI2V_EN 
        .dir = AXP_GPIO_OUTPUT,
        .level = AXP_GPIO_LOW,		//set AXP202 GPIO2 low
    },
    {
        .gpio = AXP_GPIO3,			//AXP202 GPIO3 ==> VCCX3 
        .dir = AXP_GPIO_OUTPUT,
        .level = AXP_GPIO_HIGH,		//set AXP202 GPIO3 high
    },
    AXPGPIO_CFG_END_ITEM
};
#else
static axp_gpio_cfg_t axp_init_gpio_cfg[] = {
    {
        .gpio = AXP_GPIO1,			//AXP202 GPIO1 ==> VCCX2 
        .dir = AXP_GPIO_OUTPUT,
        .level = AXP_GPIO_LOW,		//set AXP202 GPIO1 low
    },
    {
        .gpio = AXP_GPIO2,			//AXP202 GPIO2 ==> HEMI2V_EN 
        .dir = AXP_GPIO_OUTPUT,
        .level = AXP_GPIO_LOW,		//set AXP202 GPIO2 low
    },
    AXPGPIO_CFG_END_ITEM
};
#endif

static struct axp_funcdev_info axp_gpiodev[]={
    {   .name = "axp20-gpio",
        .id = AXP20_ID_GPIO,
        .platform_data = axp_init_gpio_cfg,
    },
};

static struct axp_platform_data axp_pdata = {
    .num_regl_devs = ARRAY_SIZE(axp_regldevs),
    .num_sply_devs = ARRAY_SIZE(axp_splydev),
    .num_gpio_devs = ARRAY_SIZE(axp_gpiodev),
    .regl_devs = axp_regldevs,
    .sply_devs = axp_splydev,
    .gpio_devs = axp_gpiodev,
    .gpio_base = 0,
};

#endif

/***********************************************************************
 * Meson CS DCDC section
 **********************************************************************/
#ifdef CONFIG_MESON_CS_DCDC_REGULATOR
#include <mach/voltage.h>
#include <linux/regulator/meson_cs_dcdc_regulator.h>
static struct regulator_consumer_supply vcck_data[] = {
    {
        .supply = "vcck-armcore",
    },
};

static struct regulator_init_data vcck_init_data = {
    .constraints = { /* VCCK default 1.2V */
        .name = "vcck",
        .min_uV =  959000,
        .max_uV =  1302000,
        .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies = ARRAY_SIZE(vcck_data),
    .consumer_supplies = vcck_data,
};

static struct meson_cs_pdata_t vcck_pdata = {
    .meson_cs_init_data = &vcck_init_data,
    .voltage_step_table = {
        1302000, 1279000, 1256000, 1234000,
        1210000, 1188000, 1165000, 1143000,
        1120000, 1097000, 1074000, 1051000,
        1028000, 1005000,  983000,  959000,
    },
    .default_uV = 1302000,
};

static struct meson_opp vcck_opp_table[] = {
    {
        .freq   = 1500000,
        .min_uV = 1302000,
        .max_uV = 1302000,
    },
    {
        .freq   = 1320000,
        .min_uV = 1234000,
        .max_uV = 1234000,
    },
    {
        .freq   = 1200000,
        .min_uV = 1210000,
        .max_uV = 1210000,
    },
    {
        .freq   = 1080000,
        .min_uV = 1120000,
        .max_uV = 1120000,
    },
    {
        .freq   = 840000,
        .min_uV = 1028000,
        .max_uV = 1028000,
    },
    {
        .freq   = 600000,
        .min_uV = 1005000,
        .max_uV = 1005000,
    },
    {
        .freq   = 200000,
        .min_uV = 983000,
        .max_uV = 983000,
    }
};
static struct platform_device meson_cs_dcdc_regulator_device = {
    .name = "meson-cs-regulator",
    .dev = {
        .platform_data = &vcck_pdata,
    }
};
#endif

/***********************************************************************
 * Meson CPUFREQ section
 **********************************************************************/
#ifdef CONFIG_CPU_FREQ
#include <linux/cpufreq.h>
#include <plat/cpufreq.h>

#ifdef CONFIG_MESON_CS_DCDC_REGULATOR
static struct regulator *vcck;
static struct meson_cpufreq_config cpufreq_info;

static unsigned int vcck_cur_max_freq(void)
{
    return meson_vcck_cur_max_freq(vcck, vcck_opp_table, ARRAY_SIZE(vcck_opp_table));
    }

static int vcck_scale(unsigned int frequency)
{
    return meson_vcck_scale(vcck, vcck_opp_table, ARRAY_SIZE(vcck_opp_table),
                            frequency);
}

static int vcck_regulator_init(void)
{
    vcck = regulator_get(NULL, vcck_data[0].supply);
    if (WARN(IS_ERR(vcck), "Unable to obtain voltage regulator for vcck;"
                    " voltage scaling unsupported\n")) {
        return PTR_ERR(vcck);
    }

    return 0;
}


static struct meson_cpufreq_config cpufreq_info = {
    .freq_table = NULL,
    .init = vcck_regulator_init,
    .cur_volt_max_freq = vcck_cur_max_freq,
    .voltage_scale = vcck_scale,
};
#endif //CONFIG_MESON_CS_DCDC_REGULATOR


static struct platform_device meson_cpufreq_device = {
    .name   = "cpufreq-meson",
    .dev = {
#ifdef CONFIG_MESON_CS_DCDC_REGULATOR
        .platform_data = &cpufreq_info,
#else
        .platform_data = NULL,
#endif
    },
    .id = -1,
};
#endif //CONFIG_CPU_FREQ

#ifdef CONFIG_SARADC_AM
#include <linux/saradc.h>
static struct platform_device saradc_device = {
    .name = "saradc",
    .id = 0,
    .dev = {
        .platform_data = NULL,
    },
};
#endif

#ifdef CONFIG_ANDROID_TIMED_GPIO
#ifndef _LINUX_TIMED_GPIO_H
#define _LINUX_TIMED_GPIO_H
#define TIMED_GPIO_NAME "timed-gpio"
struct timed_gpio {
	const char *name;
	unsigned 	gpio;
	int		max_timeout;
	u8 		active_low;
};

struct timed_gpio_platform_data {
	int 		num_gpios;
	struct timed_gpio *gpios;
};
#endif

static struct timed_gpio amlogic_gpio_vibravor_gpios[] ={
	{
		.name	="vibrator",
		.gpio	= PAD_GPIOC_14,	//gpioc_14
		.max_timeout	= 15000,											//15s
		.active_low	= 1,
	},
};

static struct timed_gpio_platform_data amlogic_gpio_vibravor_data= {
	.num_gpios	= 1,
	.gpios		= amlogic_gpio_vibravor_gpios,
};

static struct platform_device amlogic_gpio_vibravor = {
	.name = TIMED_GPIO_NAME,
	.dev={
		.platform_data= &amlogic_gpio_vibravor_data,
    },
};
#endif

#if defined(CONFIG_ADC_KEYPADS_AM)||defined(CONFIG_ADC_KEYPADS_AM_MODULE)
#include <linux/input.h>
#include <linux/adc_keypad.h>

static struct adc_key adc_kp_key[] = {
//	{KEY_HOLD,				"hold", CHAN_4,700,0,PAD_GPIOA_1,0},
//    {KEY_MENU,                 "menu", CHAN_4, 9, 40},
    {KEY_VOLUMEDOWN,    "vol-", CHAN_4, 150, 40},
    {KEY_VOLUMEUP,          "vol+", CHAN_4, 275, 40},
    {KEY_HOME,                "home", CHAN_4, 508, 40},
 //   {KEY_BACK,                "back", CHAN_4, 513, 40},
};

static struct adc_kp_platform_data adc_kp_pdata = {
    .key = &adc_kp_key[0],
    .key_num = ARRAY_SIZE(adc_kp_key),
};

static struct platform_device adc_kp_device = {
    .name = "m1-adckp",
    .id = 0,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
        .platform_data = &adc_kp_pdata,
    }
};
#endif

#ifdef CONFIG_AML_HOLD_KEY
#include <linux/aml_holdkey.h>
struct hold_key_platform_data hold_key_data = {
		{
			.gpio_num=PAD_GPIOA_1,
			.swap=0,
		}
	};
static struct platform_device hold_key_device = {
    .name = "holdkey",
    .id = 0,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
        .platform_data = &hold_key_data,
    }
};
#endif

/***********************************************************************
 * Power Key Section
 **********************************************************************/


#if defined(CONFIG_KEY_INPUT_CUSTOM_AM) || defined(CONFIG_KEY_INPUT_CUSTOM_AM_MODULE)
#include <linux/input.h>
#include <linux/input/key_input.h>

static int _key_code_list[] = {KEY_POWER};

static inline int key_input_init_func(void)
{
    WRITE_AOBUS_REG(AO_RTC_ADDR0, (READ_AOBUS_REG(AO_RTC_ADDR0) &~(1<<11)));
    WRITE_AOBUS_REG(AO_RTC_ADDR1, (READ_AOBUS_REG(AO_RTC_ADDR1) &~(1<<3)));
    return 0;
}
static inline int key_scan(void* data)
{
    int *key_state_list = (int*)data;
    int ret = 0;
    key_state_list[0] = ((READ_AOBUS_REG(AO_RTC_ADDR1) >> 2) & 1) ? 0 : 1;
    return ret;
}

static  struct key_input_platform_data  key_input_pdata = {
    .scan_period = 20,
    .fuzz_time = 60,
    .key_code_list = &_key_code_list[0],
    .key_num = ARRAY_SIZE(_key_code_list),
    .scan_func = key_scan,
    .init_func = key_input_init_func,
    .config = 0,
};

static struct platform_device input_device_key = {
    .name = "meson-keyinput",
    .id = 0,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
        .platform_data = &key_input_pdata,
    }
};
#endif


#ifdef CONFIG_FOCALTECH_CAPACITIVE_TOUCHSCREEN_G12_ANDA_M805A
#include <linux/ft5x06_ts.h>
#define GPIO_FT_RST  PAD_GPIOC_3
#define GPIO_FT_IRQ  PAD_GPIOA_16
#define GPIO_FT_WAKEUP PAD_GPIOA_6
#define FT_IRQ	INT_GPIO_0

static void ts_power(int on)
{
    gpio_set_status(GPIO_FT_RST, gpio_status_out);
    gpio_out(GPIO_FT_RST, on);
}

static int ts_init_irq(void)
{
    gpio_set_status(GPIO_FT_IRQ, gpio_status_in);
    gpio_irq_set(GPIO_FT_IRQ, GPIO_IRQ(FT_IRQ-INT_GPIO_0, GPIO_IRQ_FALLING));
    return 0;
}

static struct ts_platform_data ts_pdata = {
	.irq = FT_IRQ,
	.init_irq = ts_init_irq,
	.power = ts_power,
	.screen_max_x=1024,
	.screen_max_y=768,
    .swap_xy = 0,
    .xpol = 0, 
    .ypol = 0,
};
#endif

/***********************************************************************
 * I2C Section
 **********************************************************************/

#if defined(CONFIG_I2C_AML) || defined(CONFIG_I2C_HW_AML)
static bool pinmux_dummy_share(bool select)
{
    return select;
}

static pinmux_item_t aml_i2c_a_pinmux_item[] = {
    {
        .reg = 5,
        //.clrmask = (3<<24)|(3<<30),
        .setmask = 3<<26
    },
    PINMUX_END_ITEM
};

static struct aml_i2c_platform aml_i2c_plat_a = {
    .wait_count             = 50000,
    .wait_ack_interval   = 5,
    .wait_read_interval  = 5,
    .wait_xfer_interval   = 5,
    .master_no          = AML_I2C_MASTER_A,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_300K,

    .master_pinmux      = {
        .chip_select    = pinmux_dummy_share,
        .pinmux         = &aml_i2c_a_pinmux_item[0]
    }
};

static pinmux_item_t aml_i2c_b_pinmux_item[]={
    {
        .reg = 5,
        //.clrmask = (3<<28)|(3<<26),
        .setmask = 3<<30
    },
    PINMUX_END_ITEM
};

static struct aml_i2c_platform aml_i2c_plat_b = {
    .wait_count         = 50000,
    .wait_ack_interval = 5,
    .wait_read_interval = 5,
    .wait_xfer_interval = 5,
    .master_no          = AML_I2C_MASTER_B,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_300K,

    .master_pinmux      = {
        .chip_select    = pinmux_dummy_share,
        .pinmux         = &aml_i2c_b_pinmux_item[0]
    }
};

static pinmux_item_t aml_i2c_ao_pinmux_item[] = {
    {
        .reg = AO,
        .clrmask  = (3<<1)|(3<<23),
        .setmask = 3<<5
    },
    PINMUX_END_ITEM
};

static struct aml_i2c_platform aml_i2c_plat_ao = {
    .wait_count         = 50000,
    .wait_ack_interval  = 5,
    .wait_read_interval = 5,
    .wait_xfer_interval = 5,
    .master_no          = AML_I2C_MASTER_AO,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_100K,

    .master_pinmux      = {
        .pinmux         = &aml_i2c_ao_pinmux_item[0]
    }
};

static struct resource aml_i2c_resource_a[] = {
    [0] = {
        .start = MESON_I2C_MASTER_A_START,
        .end   = MESON_I2C_MASTER_A_END,
        .flags = IORESOURCE_MEM,
    }
};

static struct resource aml_i2c_resource_b[] = {
    [0] = {
        .start = MESON_I2C_MASTER_B_START,
        .end   = MESON_I2C_MASTER_B_END,
        .flags = IORESOURCE_MEM,
    }
};

static struct resource aml_i2c_resource_ao[] = {
    [0]= {
        .start =    MESON_I2C_MASTER_AO_START,
        .end   =    MESON_I2C_MASTER_AO_END,
        .flags =    IORESOURCE_MEM,
    }
};

static struct platform_device aml_i2c_device_a = {
    .name         = "aml-i2c",
    .id       = 0,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource_a),
    .resource     = aml_i2c_resource_a,
    .dev = {
        .platform_data = &aml_i2c_plat_a,
    },
};

static struct platform_device aml_i2c_device_b = {
    .name         = "aml-i2c",
    .id       = 1,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource_b),
    .resource     = aml_i2c_resource_b,
    .dev = {
        .platform_data = &aml_i2c_plat_b,
    },
};

static struct platform_device aml_i2c_device_ao = {
    .name         = "aml-i2c",
    .id       = 2,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource_ao),
    .resource     = aml_i2c_resource_ao,
    .dev = {
        .platform_data = &aml_i2c_plat_ao,
    },
};

#if defined (CONFIG_AMLOGIC_VIDEOIN_MANAGER)
static struct resource vm_resources[] = {
    [0] = {
        .start = VM_ADDR_START,
        .end   = VM_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device vm_device =
{
    .name = "vm",
    .id = 0,
    .num_resources = ARRAY_SIZE(vm_resources),
    .resource      = vm_resources,
};
#endif /* AMLOGIC_VIDEOIN_MANAGER */

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_GT2005
static int gt2005_have_inited = 0;
#endif

#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_GT2005)
static pinmux_item_t gc0308_pins[] = {
    {
        .reg = PINMUX_REG(9),
        .setmask = 1 << 12
    },
    PINMUX_END_ITEM
};

static pinmux_set_t gc0308_pinmux_set = {
    .chip_select = NULL,
    .pinmux = &gc0308_pins[0]
};

static void gt2005_init(void)
{
    pinmux_set(&gc0308_pinmux_set);

    aml_set_reg32_bits(P_HHI_GEN_CLK_CNTL, 1, 8, 5); //select XTAL as camera clock

    // reset low
    printk( "amlogic camera driver: gt2005_v4l2_init. \n");
    gpio_out(PAD_GPIOZ_0, 0);

    // set camera power disanable
    gpio_out(PAD_GPIOE_11, 0);

    msleep(20);

    // reset high
    gpio_out(PAD_GPIOZ_0, 1);
    msleep(20);

    // set camera power enable
    gpio_out(PAD_GPIOE_11, 1);
    msleep(20);
}

static void gt2005_v4l2_init(void)
{
    gt2005_have_inited=1;
    gt2005_init();
}
static void gt2005_v4l2_uninit(void)
{
    gt2005_have_inited=0;
    gpio_out(PAD_GPIOE_11, 0);    // set camera power disable
    msleep(5);
        aml_set_reg32_bits(P_HHI_GEN_CLK_CNTL, 0, 8, 1); //close clock
}

static void gt2005_v4l2_disable(void)
{

}

static void gt2005_v4l2_early_suspend(void)
{

}

static void gt2005_v4l2_late_resume(void)
{
    pinmux_set(&gc0308_pinmux_set);

    aml_set_reg32_bits(P_HHI_GEN_CLK_CNTL, 1, 8, 5); //select XTAL as camera clock
    msleep(20);

    gpio_out(PAD_GPIOZ_0, 1);
    msleep(20);

    // set camera power enable
    gpio_out(PAD_GPIOE_11, 1);
    msleep(20);
}

struct aml_camera_i2c_fig_s custom_gt2005_script[] = {
    {0x0101,0x11},
    {0xffff,0xff}, 
};

static aml_plat_cam_data_t video_gt2005_data = {
    .name="video-gt2005",
    .video_nr=0,   //    1
    .device_init= gt2005_v4l2_init,
    .device_uninit=gt2005_v4l2_uninit,
    .early_suspend = gt2005_v4l2_early_suspend,
    .late_resume = gt2005_v4l2_late_resume,
    .device_disable=gt2005_v4l2_disable,
		.custom_init_script = custom_gt2005_script,
};
#endif

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_GC0307
static int gc0307_have_inited = 0;

static pinmux_item_t gc0307_pins[] = {
    {
        .reg = PINMUX_REG(9),
        .setmask = 1 << 12
    },
    PINMUX_END_ITEM
};

static pinmux_set_t gc0307_pinmux_set = {
    .chip_select = NULL,
    .pinmux = &gc0307_pins[0]
};

static void gc0307_init(void)
{
    gpio_out(PAD_GPIOE_10, 1);
    pinmux_set(&gc0307_pinmux_set);

    aml_set_reg32_bits(P_HHI_GEN_CLK_CNTL, 1, 8, 5); //select XTAL as camera clock

    // reset low
    printk( "amlogic camera driver: gc0307_v4l2_init. \n");
    gpio_out(PAD_GPIOZ_0, 0);

    // set camera power disanable
    gpio_out(PAD_GPIOE_11, 1);

    msleep(20);

    // reset high
    gpio_out(PAD_GPIOZ_0, 1);
    msleep(20);

    // set camera power enable
    gpio_out(PAD_GPIOE_11, 0);
    msleep(20);
}

static void gc0307_v4l2_init(void)
{
    gc0307_have_inited=1;
    gc0307_init();
}
static void gc0307_v4l2_uninit(void)
{
    gc0307_have_inited=0;
    gpio_out(PAD_GPIOE_11, 1);    // set camera power disable
    msleep(5);
        aml_set_reg32_bits(P_HHI_GEN_CLK_CNTL, 0, 8, 1); //close clock
}

static void gc0307_v4l2_disable(void)
{

}

static void gc0307_v4l2_early_suspend(void)
{

}

static void gc0307_v4l2_late_resume(void)
{
    pinmux_set(&gc0307_pinmux_set);

    aml_set_reg32_bits(P_HHI_GEN_CLK_CNTL, 1, 8, 5); //select XTAL as camera clock
    msleep(20);

    gpio_out(PAD_GPIOZ_0, 1);
    msleep(20);

    // set camera power enable
    gpio_out(PAD_GPIOE_11, 0);
    msleep(20);
}

static aml_plat_cam_data_t video_gc0307_data = {
    .name="video-gc0307",
#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_GC2015
    .video_nr=1,   //    1
#else
    .video_nr=0,   //    1
#endif
    .device_init= gc0307_v4l2_init,
    .device_uninit=gc0307_v4l2_uninit,
    .early_suspend = gc0307_v4l2_early_suspend,
    .late_resume = gc0307_v4l2_late_resume,
    .device_disable=gc0307_v4l2_disable,
    .custom_init_script = NULL,
};
#endif /* CONFIG_VIDEO_AMLOGIC_CAPTURE_GC0307 */

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_GC2015
static int gc2015_have_inited = 0;

static pinmux_item_t gc2015_pins[] = {
    {
        .reg = PINMUX_REG(9),
        .setmask = 1 << 12
    },
    PINMUX_END_ITEM
};

static pinmux_set_t gc2015_pinmux_set = {
    .chip_select = NULL,
    .pinmux = &gc2015_pins[0]
};

static void gc2015_init(void)
{
    gpio_out(PAD_GPIOE_11, 1);
    pinmux_set(&gc2015_pinmux_set);

    aml_set_reg32_bits(P_HHI_GEN_CLK_CNTL, 1, 8, 5); //select XTAL as camera clock

    // reset low
    printk( "amlogic camera driver: gc2015_v4l2_init. \n");
    gpio_out(PAD_GPIOZ_0, 0);

    // set camera power disanable
    gpio_out(PAD_GPIOE_10, 1);

    msleep(20);

    // reset high
    gpio_out(PAD_GPIOZ_0, 1);
    msleep(20);

    // set camera power enable
    gpio_out(PAD_GPIOE_10, 0);
    msleep(20);
}

static void gc2015_v4l2_init(void)
{
    gc2015_have_inited=1;
    gc2015_init();
}
static void gc2015_v4l2_uninit(void)
{
    gc2015_have_inited=0;
    gpio_out(PAD_GPIOE_10, 1);    // set camera power disable
    msleep(5);
        aml_set_reg32_bits(P_HHI_GEN_CLK_CNTL, 0, 8, 1); //close clock
}

static void gc2015_v4l2_disable(void)
{

}

static void gc2015_v4l2_early_suspend(void)
{

}

static void gc2015_v4l2_late_resume(void)
{
    pinmux_set(&gc2015_pinmux_set);

    aml_set_reg32_bits(P_HHI_GEN_CLK_CNTL, 1, 8, 5); //select XTAL as camera clock
    msleep(20);

    gpio_out(PAD_GPIOZ_0, 1);
    msleep(20);

    // set camera power enable
    gpio_out(PAD_GPIOE_10, 0);
    msleep(20);
}

struct aml_camera_i2c_fig_s custom_gc2015_script[] = {
    {0x29,0x03},
};

static aml_plat_cam_data_t video_gc2015_data = {
    .name="video-gc2015",
    .video_nr=0,   //    1
    .device_init= gc2015_v4l2_init,
    .device_uninit=gc2015_v4l2_uninit,
    .early_suspend = gc2015_v4l2_early_suspend,
    .late_resume = gc2015_v4l2_late_resume,
    .device_disable=gc2015_v4l2_disable,
    .custom_init_script = NULL,
};
#endif /* CONFIG_VIDEO_AMLOGIC_CAPTURE_GC2015 */

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_OV5640
static int ov5640_have_inited = 0;

static pinmux_item_t ov5640_pins[] = {
    {
        .reg = PINMUX_REG(9),
        .setmask = 1 << 12
    },
    PINMUX_END_ITEM
};

static pinmux_set_t ov5640_pinmux_set = {
    .chip_select = NULL,
    .pinmux = &ov5640_pins[0]
};

static void ov5640_init(void)
{
    pinmux_set(&ov5640_pinmux_set);

    aml_set_reg32_bits(P_HHI_GEN_CLK_CNTL, 1, 8, 5); //select XTAL as camera clock

    // reset low
    printk( "amlogic camera driver: ov5640_v4l2_init. \n");
    gpio_out(PAD_GPIOZ_0, 0);

    // set camera power disanable
    gpio_out(PAD_GPIOE_10, 1);

    msleep(20);

    // reset high
    gpio_out(PAD_GPIOZ_0, 1);
    msleep(20);

    // set camera power enable
    gpio_out(PAD_GPIOE_10, 0);
    msleep(20);
}

static void ov5640_v4l2_init(void)
{
    ov5640_have_inited=1;
    ov5640_init();
}
static void ov5640_v4l2_uninit(void)
{
    ov5640_have_inited=0;
    gpio_out(PAD_GPIOE_10, 0);    // set camera power disable
    msleep(5);
        aml_set_reg32_bits(P_HHI_GEN_CLK_CNTL, 0, 8, 1); //close clock
}

static void ov5640_v4l2_disable(void)
{

}

static void ov5640_v4l2_early_suspend(void)
{

}

static void ov5640_v4l2_late_resume(void)
{

}

static aml_plat_cam_data_t video_ov5640_data = {
    .name="video-ov5640",
    .video_nr=0,   //    1
    .device_init= ov5640_v4l2_init,
    .device_uninit=ov5640_v4l2_uninit,
    .early_suspend = ov5640_v4l2_early_suspend,
    .late_resume = ov5640_v4l2_late_resume,
    .device_disable=ov5640_v4l2_disable,
    .custom_init_script = NULL,
};
#endif /* CONFIG_VIDEO_AMLOGIC_CAPTURE_OV5640 */

static struct i2c_board_info __initdata aml_i2c_bus_info_a[] = {

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_GT2005
    {
        /*gt2005 i2c address is 0x78/0x79*/
        I2C_BOARD_INFO("gt2005_i2c",  0x78 >> 1 ),
        .platform_data = (void *)&video_gt2005_data
    },
#endif
#ifdef CONFIG_FOCALTECH_CAPACITIVE_TOUCHSCREEN_G12_ANDA_M805A
    {
        I2C_BOARD_INFO("ft5x06", 0x38),
        .platform_data = (void *)&ts_pdata,
    },
#endif
#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_GC2015
    {
        I2C_BOARD_INFO("gc2015_i2c", 0x60 >> 1),
        .platform_data = (void *)&video_gc2015_data,
    },
#endif
#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_GC0307
    {
        I2C_BOARD_INFO("gc0307_i2c", 0x42 >> 1),
        .platform_data = (void *)&video_gc0307_data,
    },
#endif
#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_OV5640
    {
        I2C_BOARD_INFO("ov5640_i2c", 0x78 >> 1),
        .platform_data = (void *)&video_ov5640_data,
    },
#endif
};

static struct i2c_board_info __initdata aml_i2c_bus_info_ao[] = {
#ifdef CONFIG_AW_AXP20
    {
        I2C_BOARD_INFO("axp20_mfd", AXP20_ADDR),
        .platform_data = &axp_pdata,
        .addr = AXP20_ADDR, //axp_cfg.pmu_twi_addr,
        .irq = INT_WATCHDOG,	//0 smp irq number base change to 32
    },
#endif

};


static struct i2c_board_info __initdata aml_i2c_bus_info_b[] = {
#ifdef CONFIG_BOSCH_BMA250
	{
		I2C_BOARD_INFO("bma250",  0x18),
		//.irq = INT_GPIO_1,
	},
#endif
#ifdef CONFIG_SND_SOC_RT5631
    {
        I2C_BOARD_INFO("rt5631", 0x1A),
        .platform_data = (void *)NULL,
    },
#endif
#ifdef CONFIG_BOSCH_BMA222
	{
		I2C_BOARD_INFO("bma222",  0x08),
		//.irq = INT_GPIO_1,
		.platform_data = (void *) &gsen_pdata,	
	},
#endif
#ifdef CONFIG_SENSORS_MMA7660
	{
		I2C_BOARD_INFO("mma7660", 0x4C),
		.irq = INT_GPIO_2,
	},
#endif
};

static int __init aml_i2c_init(void)
{
    i2c_register_board_info(0, aml_i2c_bus_info_a,
        ARRAY_SIZE(aml_i2c_bus_info_a));
    i2c_register_board_info(1, aml_i2c_bus_info_b,
        ARRAY_SIZE(aml_i2c_bus_info_b));
    i2c_register_board_info(2, aml_i2c_bus_info_ao,
        ARRAY_SIZE(aml_i2c_bus_info_ao));
    return 0;
}
#endif

/***********************************************************************
 * UART Section
 **********************************************************************/
static pinmux_item_t uart_pins[] = {
    {
        .reg = PINMUX_REG(AO),
        .setmask = 3 << 11
    },
    PINMUX_END_ITEM
};

static pinmux_set_t aml_uart_ao = {
    .chip_select = NULL,
    .pinmux = &uart_pins[0]
};

static struct aml_uart_platform  __initdata aml_uart_plat = {
    .uart_line[0]   = UART_AO,
    .uart_line[1]   = UART_A,
    .uart_line[2]   = UART_B,
    .uart_line[3]   = UART_C,
    .uart_line[4]   = UART_D,

    .pinmux_uart[0] = (void*)&aml_uart_ao,
    .pinmux_uart[1] = NULL,
    .pinmux_uart[2] = NULL,
    .pinmux_uart[3] = NULL,
    .pinmux_uart[4] = NULL
};

static struct platform_device aml_uart_device = {
    .name       = "mesonuart",
    .id     = -1,
    .num_resources  = 0,
    .resource   = NULL,
    .dev = {
        .platform_data = &aml_uart_plat,
    },
};

/***********************************************************************
 * Nand Section
 **********************************************************************/

#ifdef CONFIG_AM_NAND
static struct mtd_partition normal_partition_info[] = {
    {
        .name = "logo",
        .offset = 32*SZ_1M+40*SZ_1M,
        .size = 8*SZ_1M,
    },
    {
        .name = "aml_logo",
        .offset = 48*SZ_1M+40*SZ_1M,
        .size = 8*SZ_1M,
    },
    {
        .name = "recovery",
        .offset = 64*SZ_1M+40*SZ_1M,
        .size = 8*SZ_1M,
    },
    {
        .name = "boot",
        .offset = 96*SZ_1M+40*SZ_1M,
        .size = 8*SZ_1M,
    },
    {
        .name = "system",
        .offset = 128*SZ_1M+40*SZ_1M,
        .size = 1024*SZ_1M,
    },
    {
        .name = "cache",
        .offset = 1152*SZ_1M+40*SZ_1M,
        .size = 256*SZ_1M,
    },
    {
        .name = "userdata",
        .offset = 1408*SZ_1M+40*SZ_1M,
        .size = 1024*SZ_1M,
    },
    {
        .name = "NFTL_Part",
        .offset = MTDPART_OFS_APPEND,
        .size = MTDPART_SIZ_FULL,
    },
};

#if 1
static struct aml_nand_platform aml_nand_mid_platform[] = {
#ifndef CONFIG_AMLOGIC_SPI_NOR
    {
        .name = NAND_BOOT_NAME,
        .chip_enable_pad = AML_NAND_CE0,
        .ready_busy_pad = AML_NAND_CE0,
        .platform_nand_data = {
            .chip =  {
                .nr_chips = 1,
                .options = (NAND_TIMING_MODE5 | NAND_ECC_BCH60_1K_MODE),
            },
        },
        .ran_mode=1,
        .rbpin_detect=1,        
        .T_REA = 20,
        .T_RHOH = 15,
    },
#endif
    {
        .name = NAND_NORMAL_NAME,
        .chip_enable_pad = ((AML_NAND_CE0) | (AML_NAND_CE1 << 4)  | (AML_NAND_CE2 << 8) | (AML_NAND_CE3 << 12)),
        .ready_busy_pad = (AML_NAND_CE0) | (AML_NAND_CE1 << 4),// | (AML_NAND_CE1 << 8) | (AML_NAND_CE1 << 12)*/),
        .platform_nand_data = {
            .chip =  {
                .nr_chips = 4,
  		.nr_partitions = ARRAY_SIZE(normal_partition_info),
                .partitions = normal_partition_info,
		.options = (NAND_TIMING_MODE5 | NAND_ECC_BCH60_1K_MODE /*| NAND_TWO_PLANE_MODE*/),
            },
        },
        .ran_mode=1,       
        .rbpin_detect=1, 
        .T_REA = 20,
        .T_RHOH = 15,
		
    }
};
#else
static struct aml_nand_platform aml_nand_mid_platform[] = {
#ifndef CONFIG_AMLOGIC_SPI_NOR
    {
        .name = NAND_BOOT_NAME,
        .chip_enable_pad = AML_NAND_CE0,
    //    .ready_busy_pad = AML_NAND_CE0,
        .platform_nand_data = {
            .chip =  {
                .nr_chips = 1,
                .options = (NAND_TIMING_MODE5 | NAND_ECC_BCH60_1K_MODE),
            },
        },
        .ran_mode=1,
        .T_REA = 20,
        .T_RHOH = 15,
    },
#endif
    {
	#ifdef CONFIG_AMLOGIC_MICRON_NYNIX_NAND_4_CHIPS
        .name = NAND_NORMAL_NAME,
        .chip_enable_pad = ((AML_NAND_CE0) | (AML_NAND_CE1 << 4)  | (AML_NAND_CE2 << 8) | (AML_NAND_CE3 << 12)),
        .platform_nand_data = {
            .chip =  {
                .nr_chips = 4,
                .nr_partitions = ARRAY_SIZE(normal_partition_info),
                .partitions = normal_partition_info,
                .options = (NAND_TIMING_MODE5 | NAND_ECC_BCH60_1K_MODE/* | NAND_TWO_PLANE_MODE*/),
            },
        },
	#endif
	#ifdef CONFIG_AMLOGIC_MICRON_NYNIX_NAND_2_CHIPS
        .name = NAND_NORMAL_NAME,
        .chip_enable_pad = (AML_NAND_CE0) | (AML_NAND_CE1 << 4),// | (AML_NAND_CE2 << 8) | (AML_NAND_CE3 << 12)*/),
        .ready_busy_pad = (AML_NAND_CE0) | (AML_NAND_CE1 << 4),// | (AML_NAND_CE1 << 8) | (AML_NAND_CE1 << 12)*/),
        .platform_nand_data = {
            .chip =  {
                .nr_chips = 2,
                .nr_partitions = ARRAY_SIZE(normal_partition_info),
                .partitions = normal_partition_info,
				 .options = (NAND_TIMING_MODE5 | NAND_ECC_BCH60_1K_MODE /*| NAND_TWO_PLANE_MODE*/),
            },
		},
	#endif
		.ran_mode=1,
        .T_REA = 20,
        .T_RHOH = 15,
    }
};
#endif

static struct aml_nand_device aml_nand_mid_device = {
    .aml_nand_platform = aml_nand_mid_platform,
    .dev_num = ARRAY_SIZE(aml_nand_mid_platform),
};

static struct resource aml_nand_resources[] = {
    {
        .start = 0xc1108600,
        .end = 0xc1108624,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device aml_nand_device = {
    .name = "aml_nand",
    .id = 0,
    .num_resources = ARRAY_SIZE(aml_nand_resources),
    .resource = aml_nand_resources,
    .dev = {
        .platform_data = &aml_nand_mid_device,
    },
};
#endif

/***********************************************************************
 * WIFI  Section
 **********************************************************************/
/**
*GPIOX_0			-->WIFI_SD_D0
*GPIOX_1			-->WIFI_SD_D1
*GPIOX_2			-->WIFI_SD_D2
*GPIOX_3			-->WIFI_SD_D3

*GPIOX_8			-->WIFI_SD_CMD
*GPIOX_9			-->WIFI_SD_CLK

*WIFI_EN			-->GPIOC_8
*WIFI_WAKE		-->GPIOX_11
*32K_CLOCK_OUT	-->GPIOX_12 (CLK_OUT3)
*/
#ifdef  CONFIG_AM_WIFI
#define DBG_LINE_INFO()  printk(KERN_INFO "[%s] in\n",__func__)

/* WIFI ON Flag */
static int WIFI_ON;
/* BT ON Flag */
static int BT_ON;
static void wifi_gpio_init(void)
{
//set status
    //WIFI_EN WIFI_PWREN  WLAN_RST --->out	:0
	gpio_set_status(PAD_GPIOC_8,gpio_status_out);
	//WIFI_WAKE -->1GPIOX_11   in	:
    	gpio_set_status(PAD_GPIOX_15,gpio_status_in);
	//set pull-up
	aml_clr_reg32_mask(P_PAD_PULL_UP_REG4,0xf|1<<8|1<<9|1<<11|1<<12);
	aml_clr_reg32_mask(P_PAD_PULL_UP_REG2,1<<8);	
}

static void wifi_clock_enable(int is_on)
{
    //set clk 32k for wifi  
    //GPIOX_12 (CLK_OUT3)  //reg : 108b  sr_sl:22-25  div:13-19    enable:21
    DBG_LINE_INFO();

	gpio_set_status(PAD_GPIOX_12,gpio_status_out);			//set  GPIOX_12 out
	aml_set_reg32_mask(P_HHI_GEN_CLK_CNTL2,1<<22);//set clk source
	aml_clr_reg32_mask(P_HHI_GEN_CLK_CNTL2,0x3f<<13);//set div ==1
	aml_set_reg32_mask(P_HHI_GEN_CLK_CNTL2,1<<21);//set enable clk
	aml_set_reg32_mask(P_PERIPHS_PIN_MUX_3,0x1<<21);//set mode GPIOX_12-->CLK_OUT3
}

void extern_wifi_set_power(int is_on)
{
	
}
EXPORT_SYMBOL(extern_wifi_set_power);

void extern_wifi_set_enable(int is_on)
{
	DBG_LINE_INFO();
    gpio_set_status(PAD_GPIOC_8,gpio_status_out);//set wifi_en gpio mode out
    if(is_on){
		gpio_out(PAD_GPIOC_8,1);
		printk("WIFI  Enable! \n");
    		}
	else{
		gpio_out(PAD_GPIOC_8,0);
            	printk("WIFI  Disenable! \n");
	}
}
EXPORT_SYMBOL(extern_wifi_set_enable);

void extern_wifi_reset(int is_on)
{

}
EXPORT_SYMBOL(extern_wifi_reset);

void wifi_dev_init(void)
{
	DBG_LINE_INFO();
	wifi_clock_enable(1);
	udelay(200);
	wifi_gpio_init();
}
#endif
/***********************************************************************
 * Card Reader Section
 **********************************************************************/
#ifdef CONFIG_CARDREADER
static struct resource meson_card_resource[] = {
    [0] = {
        .start = 0x1200230,   //physical address
        .end   = 0x120024c,
        .flags = 0x200,
    }
};


static void sdio_extern_init(void)
{
    
   #if defined(CONFIG_BCM40181_HW_OOB) || defined(CONFIG_BCM40181_OOB_INTR_ONLY)/* Jone add */

   

    gpio_set_status(PAD_GPIOX_15,gpio_status_in);
    gpio_irq_set(PAD_GPIOX_15,GPIO_IRQ(4,GPIO_IRQ_HIGH));
    #endif
    extern_wifi_set_enable(1);
}

static struct aml_card_info meson_card_info[] = {
    [0] = {
        .name           = "sd_card",
        .work_mode      = CARD_HW_MODE,
        .io_pad_type        = SDHC_CARD_0_5,
        .card_ins_en_reg    = CARD_GPIO_ENABLE,
        .card_ins_en_mask   = PREG_IO_29_MASK,
        .card_ins_input_reg = CARD_GPIO_INPUT,
        .card_ins_input_mask    = PREG_IO_29_MASK,
        .card_power_en_reg  = CARD_GPIO_ENABLE,
        .card_power_en_mask = PREG_IO_31_MASK,
        .card_power_output_reg  = CARD_GPIO_OUTPUT,
        .card_power_output_mask = PREG_IO_31_MASK,
        .card_power_en_lev  = 0,
        .card_wp_en_reg     = 0,
        .card_wp_en_mask    = 0,
        .card_wp_input_reg  = 0,
        .card_wp_input_mask = 0,
        .card_extern_init   = 0,
    },
#if 1
    [1] = {
        .name           = "sdio_card",
        .work_mode      = CARD_HW_MODE,
        .io_pad_type        = SDHC_GPIOX_0_9,
        .card_ins_en_reg    = 0,
        .card_ins_en_mask   = 0,
        .card_ins_input_reg = 0,
        .card_ins_input_mask    = 0,
        .card_power_en_reg  = 0,
        .card_power_en_mask = 0,
        .card_power_output_reg  = 0,
        .card_power_output_mask = 0,
        .card_power_en_lev  = 0,
        .card_wp_en_reg     = 0,
        .card_wp_en_mask    = 0,
        .card_wp_input_reg  = 0,
        .card_wp_input_mask = 0,
        .card_extern_init   = sdio_extern_init,
    },
#endif
};

static struct aml_card_platform meson_card_platform = {
    .card_num   = ARRAY_SIZE(meson_card_info),
    .card_info  = meson_card_info,
};

static struct platform_device meson_card_device = {
    .name       = "AMLOGIC_CARD",
    .id     = -1,
    .num_resources  = ARRAY_SIZE(meson_card_resource),
    .resource   = meson_card_resource,
    .dev = {
        .platform_data = &meson_card_platform,
    },
};

/**
 *  Some Meson6 socket board has card detect issue.
 *  Force card detect success for socket board.
 */
static int meson_mmc_detect(void)
{
    return 0;
}
#endif // CONFIG_CARDREADER

/***********************************************************************
 * MMC SD Card  Section
 **********************************************************************/
#ifdef CONFIG_MMC_AML
struct platform_device;
struct mmc_host;
struct mmc_card;
struct mmc_ios;

//return 1: no inserted  0: inserted
static int aml_sdio_detect(struct aml_sd_host * host)
{
    aml_set_reg32_mask(P_PREG_PAD_GPIO5_EN_N,1<<29);//CARD_6 input mode
    if((aml_read_reg32(P_PREG_PAD_GPIO5_I)&(1<<29)) == 0)
        return 0;
    else{ //for socket card box
                return 0;
    }
    return 1; //no insert.
}

static void  cpu_sdio_pwr_prepare(unsigned port)
{
    switch(port)
    {
        case MESON_SDIO_PORT_A:
            aml_clr_reg32_mask(P_PREG_PAD_GPIO4_EN_N,0x30f);
            aml_clr_reg32_mask(P_PREG_PAD_GPIO4_O   ,0x30f);
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_8,0x3f);
            break;
        case MESON_SDIO_PORT_B:
            aml_clr_reg32_mask(P_PREG_PAD_GPIO5_EN_N,0x3f<<23);
            aml_clr_reg32_mask(P_PREG_PAD_GPIO5_O   ,0x3f<<23);
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_2,0x3f<<10);
            break;
        case MESON_SDIO_PORT_C:
            aml_clr_reg32_mask(P_PREG_PAD_GPIO3_EN_N,0xc0f);
            aml_clr_reg32_mask(P_PREG_PAD_GPIO3_O   ,0xc0f);
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_6,(0x3f<<24));
            break;
        case MESON_SDIO_PORT_XC_A:
            break;
        case MESON_SDIO_PORT_XC_B:
            break;
        case MESON_SDIO_PORT_XC_C:
            break;
    }
}

static int cpu_sdio_init(unsigned port)
{
    switch(port)
    {
        case MESON_SDIO_PORT_A:
                aml_set_reg32_mask(P_PERIPHS_PIN_MUX_8,0x3d<<0);
                aml_set_reg32_mask(P_PERIPHS_PIN_MUX_8,0x1<<1);
                break;
        case MESON_SDIO_PORT_B:
                aml_set_reg32_mask(P_PERIPHS_PIN_MUX_2,0x3d<<10);
                aml_set_reg32_mask(P_PERIPHS_PIN_MUX_2,0x1<<11);
                break;
        case MESON_SDIO_PORT_C://SDIOC GPIOB_2~GPIOB_7
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_2,(0x1f<<22));
            aml_set_reg32_mask(P_PERIPHS_PIN_MUX_6,(0x1f<<25));
            aml_set_reg32_mask(P_PERIPHS_PIN_MUX_6,(0x1<<24));
            break;
        case MESON_SDIO_PORT_XC_A:
            #if 0
            //sdxc controller can't work
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_8,(0x3f<<0));
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_3,(0x0f<<27));
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_7,((0x3f<<18)|(0x7<<25)));
            //aml_set_reg32_mask(P_PERIPHS_PIN_MUX_5,(0x1f<<10));//data 8 bit
            aml_set_reg32_mask(P_PERIPHS_PIN_MUX_5,(0x1b<<10));//data 4 bit
            #endif
            break;
        case MESON_SDIO_PORT_XC_B:
            //sdxc controller can't work
            //aml_set_reg32_mask(P_PERIPHS_PIN_MUX_2,(0xf<<4));
            break;
        case MESON_SDIO_PORT_XC_C:
            #if 0
            //sdxc controller can't work
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_6,(0x3f<<24));
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_2,((0x13<<22)|(0x3<<16)));
            aml_set_reg32_mask(P_PERIPHS_PIN_MUX_4,(0x1f<<26));
            printk(KERN_INFO "inand sdio xc-c init\n");
            #endif
            break;
        default:
            return -1;
    }
    return 0;
}

static void aml_sdio_pwr_prepare(unsigned port)
{
    /// @todo NOT FINISH
    ///do nothing here
    cpu_sdio_pwr_prepare(port);
}

static void aml_sdio_pwr_on(unsigned port)
{
    if((aml_read_reg32(P_PREG_PAD_GPIO5_O) & (1<<31)) != 0){
    aml_clr_reg32_mask(P_PREG_PAD_GPIO5_O,(1<<31));
    aml_clr_reg32_mask(P_PREG_PAD_GPIO5_EN_N,(1<<31));
    udelay(1000);
    }
    /// @todo NOT FINISH
}
static void aml_sdio_pwr_off(unsigned port)
{
    if((aml_read_reg32(P_PREG_PAD_GPIO5_O) & (1<<31)) == 0){
        aml_set_reg32_mask(P_PREG_PAD_GPIO5_O,(1<<31));
        aml_clr_reg32_mask(P_PREG_PAD_GPIO5_EN_N,(1<<31));//GPIOD13
        udelay(1000);
    }
    /// @todo NOT FINISH
}
static int aml_sdio_init(struct aml_sd_host * host)
{ //set pinumx ..
    aml_set_reg32_mask(P_PREG_PAD_GPIO5_EN_N,1<<29);//CARD_6
    cpu_sdio_init(host->sdio_port);
        host->clk = clk_get_sys("clk81",NULL);
        if(!IS_ERR(host->clk))
                host->clk_rate = clk_get_rate(host->clk);
        else
                host->clk_rate = 0;
    return 0;
}

static struct resource aml_mmc_resource[] = {
   [0] = {
        .start = 0x1200230,   //physical address
        .end   = 0x1200248,
        .flags = IORESOURCE_MEM, //0x200
    },
};

static u64 aml_mmc_device_dmamask = 0xffffffffUL;
static struct aml_mmc_platform_data aml_mmc_def_platdata = {
    .no_wprotect = 1,
    .no_detect = 0,
    .wprotect_invert = 0,
    .detect_invert = 0,
    .use_dma = 0,
    .gpio_detect=1,
    .gpio_wprotect=0,
    .ocr_avail = MMC_VDD_33_34,

    .sdio_port = MESON_SDIO_PORT_B,
    .max_width  = 4,
    .host_caps  = (MMC_CAP_4_BIT_DATA |
                            MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED | MMC_CAP_NEEDS_POLL),

    .f_min = 200000,
    .f_max = 50000000,
    .clock = 300000,

    .sdio_init = aml_sdio_init,
    .sdio_detect = aml_sdio_detect,
    .sdio_pwr_prepare = aml_sdio_pwr_prepare,
    .sdio_pwr_on = aml_sdio_pwr_on,
    .sdio_pwr_off = aml_sdio_pwr_off,
};

static struct platform_device aml_mmc_device = {
    .name       = "aml_sd_mmc",
    .id     = 0,
    .num_resources  = ARRAY_SIZE(aml_mmc_resource),
    .resource   = aml_mmc_resource,
    .dev        = {
        .dma_mask       =       &aml_mmc_device_dmamask,
        .coherent_dma_mask  = 0xffffffffUL,
        .platform_data      = &aml_mmc_def_platdata,
    },
};
#endif //CONFIG_MMC_AML


/***********************************************************************
 * IO Mapping
 **********************************************************************/
/*
#define IO_CBUS_BASE        0xf1100000  ///2M
#define IO_AXI_BUS_BASE     0xf1300000  ///1M
#define IO_PL310_BASE       0xf2200000  ///4k
#define IO_PERIPH_BASE      0xf2300000  ///4k
#define IO_APB_BUS_BASE     0xf3000000  ///8k
#define IO_DOS_BUS_BASE     0xf3010000  ///64k
#define IO_AOBUS_BASE       0xf3100000  ///1M
#define IO_USB_A_BASE       0xf3240000  ///256k
#define IO_USB_B_BASE       0xf32C0000  ///256k
#define IO_WIFI_BASE        0xf3300000  ///1M
#define IO_SATA_BASE        0xf3400000  ///64k
#define IO_ETH_BASE         0xf3410000  ///64k

#define IO_SPIMEM_BASE      0xf4000000  ///64M
#define IO_A9_APB_BASE      0xf8000000  ///256k
#define IO_DEMOD_APB_BASE   0xf8044000  ///112k
#define IO_MALI_APB_BASE    0xf8060000  ///128k
#define IO_APB2_BUS_BASE    0xf8000000
#define IO_AHB_BASE         0xf9000000  ///128k
#define IO_BOOTROM_BASE     0xf9040000  ///64k
#define IO_SECBUS_BASE      0xfa000000
#define IO_EFUSE_BASE       0xfa000000  ///4k
*/
static __initdata struct map_desc meson_io_desc[] = {
    {
        .virtual    = IO_CBUS_BASE,
        .pfn        = __phys_to_pfn(IO_CBUS_PHY_BASE),
        .length     = SZ_2M,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_AXI_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_AXI_BUS_PHY_BASE),
        .length     = SZ_1M,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_PL310_BASE,
        .pfn        = __phys_to_pfn(IO_PL310_PHY_BASE),
        .length     = SZ_4K,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_PERIPH_BASE,
        .pfn        = __phys_to_pfn(IO_PERIPH_PHY_BASE),
        .length     = SZ_1M,
        .type       = MT_DEVICE,
    } , {
           .virtual    = IO_APB_BUS_BASE,
           .pfn        = __phys_to_pfn(IO_APB_BUS_PHY_BASE),
           .length     = SZ_1M,
           .type       = MT_DEVICE,
       } , /*{

           .virtual    = IO_DOS_BUS_BASE,
           .pfn        = __phys_to_pfn(IO_DOS_BUS_PHY_BASE),
           .length     = SZ_64K,
           .type       = MT_DEVICE,
       } , */{
           .virtual    = IO_AOBUS_BASE,
        .pfn        = __phys_to_pfn(IO_AOBUS_PHY_BASE),
        .length     = SZ_1M,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_AHB_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_AHB_BUS_PHY_BASE),
        .length     = SZ_8M,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_SPIMEM_BASE,
        .pfn        = __phys_to_pfn(IO_SPIMEM_PHY_BASE),
        .length     = SZ_64M,
        .type       = MT_ROM,
    } , {
        .virtual    = IO_APB2_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_APB2_BUS_PHY_BASE),
        .length     = SZ_512K,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_AHB_BASE,
        .pfn        = __phys_to_pfn(IO_AHB_PHY_BASE),
        .length     = SZ_128K,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_BOOTROM_BASE,
        .pfn        = __phys_to_pfn(IO_BOOTROM_PHY_BASE),
        .length     = SZ_64K,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_SECBUS_BASE,
        .pfn        = __phys_to_pfn(IO_SECBUS_PHY_BASE),
        .length     = SZ_4K,
        .type       = MT_DEVICE,
    }, {
        .virtual    = IO_SECURE_BASE,
        .pfn        = __phys_to_pfn(IO_SECURE_PHY_BASE),
        .length     = SZ_16K,
        .type       = MT_DEVICE,
    }, {
        .virtual    = PAGE_ALIGN(__phys_to_virt(RESERVED_MEM_START)),
        .pfn        = __phys_to_pfn(RESERVED_MEM_START),
        .length     = RESERVED_MEM_END - RESERVED_MEM_START + 1,
        .type       = MT_MEMORY_NONCACHED,
    },
#ifdef CONFIG_MESON_SUSPEND
        {
        .virtual    = PAGE_ALIGN(__phys_to_virt(0x9ff00000)),
        .pfn        = __phys_to_pfn(0x9ff00000),
        .length     = SZ_1M,
        .type       = MT_MEMORY_NONCACHED,
        },
#endif
};

static void __init meson_map_io(void)
{
    iotable_init(meson_io_desc, ARRAY_SIZE(meson_io_desc));
}

static void __init meson_fixup(struct machine_desc *mach, struct tag *tag, char **cmdline, struct meminfo *m)
{
    struct membank *pbank;
    mach->video_start    = RESERVED_MEM_START;
    mach->video_end      = RESERVED_MEM_END;

    m->nr_banks = 0;
    pbank = &m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(PHYS_MEM_START);
    pbank->size  = SZ_64M & PAGE_MASK;
    m->nr_banks++;
    pbank = &m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(RESERVED_MEM_END + 1);
#ifdef CONFIG_MESON_SUSPEND
    pbank->size  = (PHYS_MEM_END-RESERVED_MEM_END-SZ_1M) & PAGE_MASK;
#else
    pbank->size  = (PHYS_MEM_END-RESERVED_MEM_END) & PAGE_MASK;
#endif
    m->nr_banks++;
}

/***********************************************************************
 *USB Setting section
 **********************************************************************/
static void set_usb_a_vbus_power(char is_power_on)
{
//M6 SKT: GPIOD_9,  OEN: 0x2012, OUT:0x2013 , Bit25

#define USB_A_POW_GPIO  PREG_EGPIO
#define USB_A_POW_GPIO_BIT  25
#define USB_A_POW_GPIO_BIT_ON   1
#define USB_A_POW_GPIO_BIT_OFF  0
    if(is_power_on){
        printk( "set usb port power on (board gpio %d)!\n",USB_A_POW_GPIO_BIT);

        //aml_set_reg32_bits(CBUS_REG_ADDR(0x2012),0,USB_A_POW_GPIO_BIT,1);//mode
       // aml_set_reg32_bits(CBUS_REG_ADDR(0x2013),1,USB_A_POW_GPIO_BIT,1);//out
        gpio_out(PAD_GPIOD_9, 1); 
        //set_gpio_mode(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT,GPIO_OUTPUT_MODE);
        //set_gpio_val(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT,USB_A_POW_GPIO_BIT_ON);
    }
    else    {
        printk("set usb port power off (board gpio %d)!\n",USB_A_POW_GPIO_BIT);
        //aml_set_reg32_bits(CBUS_REG_ADDR(0x2012),0,USB_A_POW_GPIO_BIT,1);//mode
        //aml_set_reg32_bits(CBUS_REG_ADDR(0x2013),0,USB_A_POW_GPIO_BIT,1);//out
        gpio_out(PAD_GPIOD_9, 0); 
    }
}

static  int __init setup_usb_devices(void)
{
    struct lm_device * usb_ld_a, *usb_ld_b;
    usb_ld_a = alloc_usb_lm_device(USB_PORT_IDX_A);
    usb_ld_b = alloc_usb_lm_device(USB_PORT_IDX_B);
    usb_ld_a->param.usb.set_vbus_power = set_usb_a_vbus_power;
    //usb_ld_a->param.usb.port_type = USB_PORT_TYPE_HOST;
    lm_device_register(usb_ld_a);
    lm_device_register(usb_ld_b);
    return 0;
}

/***********************************************************************
 *WiFi power section
 **********************************************************************/
/* built-in usb wifi power ctrl, usb dongle must register NULL to power_ctrl! 1:power on  0:power off */
#ifdef CONFIG_AM_WIFI
#ifdef CONFIG_AM_WIFI_USB
static void usb_wifi_power(int is_power)
{
    printk(KERN_INFO "usb_wifi_power %s\n", is_power ? "On" : "Off");
//    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_1,(1<<11));
//    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0,(1<<18));
//    CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_EN_N, (1<<8));
//    if (is_power)
//        CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<8));
//    else
//        SET_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<8));
	if (is_power)
    	gpio_out(PAD_GPIOC_8, 0); 
	else
		gpio_out(PAD_GPIOC_8, 1); 
    return 0;
}

static struct wifi_power_platform_data wifi_plat_data = {
    .usb_set_power = usb_wifi_power,
};
#elif defined(CONFIG_AM_WIFI_SD_MMC)&&defined(CONFIG_CARDREADER)
    wifi_plat_data = {
    
};
#endif

static struct platform_device wifi_power_device = {
    .name       = "wifi_power",
    .id     = -1,
    .dev = {
        .platform_data = &wifi_plat_data,
    },
};
#endif

/***********************************************************************/
#ifdef CONFIG_EFUSE
static bool efuse_data_verify(unsigned char *usid)
{  int len;

    len = strlen(usid);
    if((len > 8)&&(len<31) )
        return true;
    else
        return false;
}

static struct efuse_platform_data aml_efuse_plat = {
    .pos = 454,
    .count = 58,
    .data_verify = efuse_data_verify,
};

static struct platform_device aml_efuse_device = {
    .name   = "efuse",
    .id = -1,
    .dev = {
                .platform_data = &aml_efuse_plat,
           },
};
#endif

#if defined(CONFIG_AML_RTC)
static  struct platform_device aml_rtc_device = {
            .name            = "aml_rtc",
            .id               = -1,
    };
#endif

#if defined(CONFIG_TVIN_VDIN)
static struct resource vdin_resources[] = {
    [0] = {
        .start =  VDIN_ADDR_START,  //pbufAddr
        .end   = VDIN_ADDR_END,     //pbufAddr + size
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = VDIN_ADDR_START,
        .end   = VDIN_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [2] = {
        .start = INT_VDIN_VSYNC,
        .end   = INT_VDIN_VSYNC,
        .flags = IORESOURCE_IRQ,
    },
    [3] = {
        .start = INT_VDIN_VSYNC,
        .end   = INT_VDIN_VSYNC,
        .flags = IORESOURCE_IRQ,
    },
};

static struct platform_device vdin_device = {
    .name       = "vdin",
    .id         = -1,
    .num_resources = ARRAY_SIZE(vdin_resources),
    .resource      = vdin_resources,
};
#endif

#ifdef CONFIG_TVIN_BT656IN
//add pin mux info for bt656 input
#if 0
static struct resource bt656in_resources[] = {
    [0] = {
        .start =  VDIN_ADDR_START,      //pbufAddr
        .end   = VDIN_ADDR_END,             //pbufAddr + size
        .flags = IORESOURCE_MEM,
    },
    [1] = {     //bt656/camera/bt601 input resource pin mux setting
        .start =  0x3000,       //mask--mux gpioD 15 to bt656 clk;  mux gpioD 16:23 to be bt656 dt_in
        .end   = PERIPHS_PIN_MUX_5 + 0x3000,
        .flags = IORESOURCE_MEM,
    },

    [2] = {         //camera/bt601 input resource pin mux setting
        .start =  0x1c000,      //mask--mux gpioD 12 to bt601 FIQ; mux gpioD 13 to bt601HS; mux gpioD 14 to bt601 VS;
        .end   = PERIPHS_PIN_MUX_5 + 0x1c000,
        .flags = IORESOURCE_MEM,
    },

    [3] = {         //bt601 input resource pin mux setting
        .start =  0x800,        //mask--mux gpioD 24 to bt601 IDQ;;
        .end   = PERIPHS_PIN_MUX_5 + 0x800,
        .flags = IORESOURCE_MEM,
    },

};
#endif

static struct platform_device bt656in_device = {
    .name       = "amvdec_656in",
    .id         = -1,
//    .num_resources = ARRAY_SIZE(bt656in_resources),
//    .resource      = bt656in_resources,
};
#endif

//tmp fix by Elvis Yu
static void m6ref_set_vccx2(int power_on)
{
/*    if (power_on) {
        printk(KERN_INFO "%s() Power ON\n", __FUNCTION__);
        axp_gpio_set_io(1,1);	//AXP 202 GPIO1 VCCX2 
		axp_gpio_set_value(1, 0);	//set AXP 202 GPIO1 low
    }
    else {
        printk(KERN_INFO "%s() Power OFF\n", __FUNCTION__);
        axp_gpio_set_io(1,1);	//GPIO1 VCCX2 
		axp_gpio_set_value(1, 1);	////set AXP 202 GPIO1 high
    }
    */
}
#if defined(CONFIG_SUSPEND)
static struct meson_pm_config aml_pm_pdata = {
    .pctl_reg_base = (void *)IO_APB_BUS_BASE,
    .mmc_reg_base = (void *)APB_REG_ADDR(0x1000),
    .hiu_reg_base = (void *)CBUS_REG_ADDR(0x1000),
    .power_key = (1<<8),
    .ddr_clk = 0x00110820,
    .sleepcount = 128,
    .set_vccx2 = m6ref_set_vccx2,
    .core_voltage_adjust = 7,  //5,8
};

static struct platform_device aml_pm_device = {
    .name           = "pm-meson",
    .dev = {
        .platform_data  = &aml_pm_pdata,
    },
    .id             = -1,
};
#endif

/***********************************************************************
 * Audio section
 **********************************************************************/

static struct resource aml_m6_audio_resource[] = {
    [0] =   {
        .start      =   0,
        .end        =   0,
        .flags      =   IORESOURCE_MEM,
    },
};

static struct platform_device aml_audio = {
    .name           = "aml-audio",
    .id             = 0,
};

static struct platform_device aml_audio_dai = {
    .name           = "aml-dai",
    .id             = 0,
};

extern char* get_vout_mode_internal(void);

/* Check current mode, 0: panel; 1: !panel*/
int get_display_mode(void) {
	int ret = 0;
	if(strncmp("panel", get_vout_mode_internal(), 5)){
		ret = 1;
	}

	return ret;
}

#if defined(CONFIG_SND_SOC_RT5631)

static pinmux_item_t rt5631_pinmux[] = {
    /* I2S_MCLK I2S_BCLK I2S_LRCLK I2S_DIN I2S_DOUT */
    {
        .reg = PINMUX_REG(9),
        .setmask = (1 << 7) | (1 << 5) | (1 << 9) | (1 << 11) | (1 << 4),
        .clrmask = (7 << 19) | (7 << 1) | (1 << 10) | (1 << 6),
    },
    {
        .reg = PINMUX_REG(8),
        .clrmask = (0x7f << 24),
    },
    PINMUX_END_ITEM
};

static pinmux_set_t rt5631_pinmux_set = {
    .chip_select = NULL,
    .pinmux = &rt5631_pinmux[0],
};

static void rt5631_device_init(void)
{
    /* audio pinmux */
    pinmux_set(&rt5631_pinmux_set);

    /* GPIOA_19 PULL_UP_REG0 bit19 */
    aml_set_reg32_bits(P_PAD_PULL_UP_REG0, 1, 19, 1);
}

static void rt5631_device_deinit(void)
{
    pinmux_clr(&rt5631_pinmux_set);
}

static int rt5631_hp_detect(void)
{
#define HP_PLUG     1
#define HP_UNPLUG   0

    int val = HP_UNPLUG;
    uint32_t level = 0;

    if(get_display_mode() != 0) {   //if !panel, return HP_PLUG
        return HP_PLUG;
    }

    /* GPIOA_19 */
    aml_set_reg32_bits(P_PREG_PAD_GPIO0_EN_N, 1, 19, 1);    // mode
    level = aml_get_reg32_bits(P_PREG_PAD_GPIO0_I, 19, 1);  // value
    if (level == 1) {
        val = HP_PLUG;
    }

    return val;
}

static struct rt5631_platform_data rt5631_pdata = {
    .hp_detect      = rt5631_hp_detect,
    .device_init    = rt5631_device_init,
    .device_uninit  = rt5631_device_deinit,

    .spk_watt       = RT5631_SPK_1_0W,
    .spk_output     = RT5631_SPK_STEREO,
    .mic_input      = RT5631_MIC_DIFFERENTIAL,
};

static struct platform_device aml_rt5631_audio = {
    .name           = "aml_rt5631_audio",
    .id             = 0,
    .resource       = aml_m6_audio_resource,
    .num_resources  = ARRAY_SIZE(aml_m6_audio_resource),
    .dev = {
        .platform_data = &rt5631_pdata,
    },
};

#endif


static void power_off(void)
{
	kernel_restart("charging_reboot");
}


/***********************************************************************
 * Device Register Section
 **********************************************************************/
static struct platform_device  *platform_devs[] = {
#if defined(CONFIG_I2C_AML) || defined(CONFIG_I2C_HW_AML)
    &aml_i2c_device_a,
    &aml_i2c_device_b,
    &aml_i2c_device_ao,
#endif
    &aml_uart_device,
    &meson_device_fb,
#ifdef CONFIG_AM_FB_EXT
    &meson_device_fb_ext,
#endif    
    &meson_device_vout,
#ifdef CONFIG_AM_STREAMING
    &meson_device_codec,
#endif
#if defined(CONFIG_AM_NAND)
    &aml_nand_device,
#endif
#if defined(CONFIG_TVIN_VDIN)
    &vdin_device,
#endif
#if defined(CONFIG_TVIN_BT656IN)
    &bt656in_device,
#endif
#ifdef CONFIG_AMLOGIC_VIDEOIN_MANAGER
    &vm_device,
#endif
#if defined(CONFIG_CARDREADER)
    &meson_card_device,
#endif // CONFIG_CARDREADER
#if defined(CONFIG_MMC_AML)
    &aml_mmc_device,
#endif
#if defined(CONFIG_SUSPEND)
    &aml_pm_device,
#endif
#ifdef CONFIG_EFUSE
    &aml_efuse_device,
#endif
#if defined(CONFIG_AML_RTC)
    &aml_rtc_device,
#endif
#ifdef CONFIG_SARADC_AM
        &saradc_device,
#endif
#ifdef CONFIG_AML_HOLD_KEY
    &hold_key_device,
#endif
#if defined(CONFIG_ADC_KEYPADS_AM)||defined(CONFIG_ADC_KEYPADS_AM_MODULE)
    &adc_kp_device,
#endif
#if defined(CONFIG_KEY_INPUT_CUSTOM_AM) || defined(CONFIG_KEY_INPUT_CUSTOM_AM_MODULE)
    &input_device_key,
#endif
    &aml_audio,
    &aml_audio_dai,
#if defined(CONFIG_SND_SOC_RT5631)
    &aml_rt5631_audio,
#endif
#if defined(CONFIG_SND_SOC_WM8960)
    &aml_wm8960_audio,
#endif
#ifdef CONFIG_AM_WIFI
    &wifi_power_device,
#endif

#ifdef CONFIG_USB_ANDROID
    &android_usb_device,
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
    &usb_mass_storage_device,
#endif
#endif

#ifdef CONFIG_POST_PROCESS_MANAGER
	&ppmgr_device,
#endif
#if defined(CONFIG_AM_TV_OUTPUT2)
    &vout2_device,   
#endif
#ifdef CONFIG_MESON_CS_DCDC_REGULATOR
	&meson_cs_dcdc_regulator_device,
#endif
#ifdef CONFIG_CPU_FREQ
	&meson_cpufreq_device
#endif
};
#ifdef CONFIG_SENSORS_MMA7660
static void __init set_mma7660_regs(void)
{
    /* set input mode */
    gpio_set_status(PAD_GPIOA_15, gpio_status_in);
    gpio_irq_set(PAD_GPIOA_15, GPIO_IRQ(INT_GPIO_2-INT_GPIO_0, GPIO_IRQ_FALLING));
}
#endif

static __init void meson_init_machine(void)
{
    //meson_cache_init();
    //mmc_lp_suspend_init();
    setup_usb_devices();
    setup_devices_resource();
    wifi_dev_init();
#ifdef CONFIG_AML_HDMI_TX
    extern int setup_hdmi_dev_platdata(void* platform_data);
    setup_hdmi_dev_platdata(&aml_hdmi_pdata);
#endif
    platform_add_devices(platform_devs, ARRAY_SIZE(platform_devs));

#if defined(CONFIG_I2C_AML) || defined(CONFIG_I2C_HW_AML)
    aml_i2c_init();
#endif
#ifdef CONFIG_AM_WIFI_USB
    if(wifi_plat_data.usb_set_power)
        wifi_plat_data.usb_set_power(0);//power off built-in usb wifi
#endif

#ifdef CONFIG_SENSORS_MMA7660
	set_mma7660_regs();
#endif
#ifdef CONFIG_AM_LCD_OUTPUT
    m6g12M805_lcd_init();
#endif
	pm_power_off = power_off;
}
static __init void meson_init_early(void)
{///boot seq 1

}

MACHINE_START(MESON6_G12, "Amlogic Meson6 g12 customer platform")
    .boot_params    = BOOT_PARAMS_OFFSET,
    .map_io         = meson_map_io,///2
    .init_early     = meson_init_early,///3
    .init_irq       = meson_init_irq,///0
    .timer          = &meson_sys_timer,
    .init_machine   = meson_init_machine,
    .fixup          = meson_fixup,///1
MACHINE_END
