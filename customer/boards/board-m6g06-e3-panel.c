/*
 * arch/arm/mach-meson3/board-m6g06-panel.c
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
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <plat/platform.h>
#include <plat/plat_dev.h>
#include <plat/lm.h>
#include <mach/clock.h>
#include <mach/map.h>
#include <mach/gpio.h>
#include <mach/gpio_data.h>
#include <linux/delay.h>
#include <plat/regops.h>
#include <mach/reg_addr.h>

#include <linux/vout/lcdoutc.h>
#include <linux/aml_bl.h>
#include <mach/lcd_aml.h>

#include "board-m6g06-e3.h"

#ifdef CONFIG_AW_AXP
extern int axp_gpio_set_io(int gpio, int io_state);
extern int axp_gpio_get_io(int gpio, int *io_state);
extern int axp_gpio_set_value(int gpio, int value);
extern int axp_gpio_get_value(int gpio, int *value);
#endif

extern Lcd_Config_t m6g06_lcd_config;

// Define backlight control method
#define BL_CTL_GPIO 0
#define BL_CTL_PWM  1
#define BL_CTL      BL_CTL_GPIO

#if(BL_CTL==BL_CTL_PWM)
#define PWM_MAX         60000   //PWM_MAX <= 65535
#define	PWM_PRE_DIV		0		//pwm_freq = 24M / (pre_div + 1) / PWM_MAX	
#endif

#define BL_MAX_LEVEL    255
#define BL_MIN_LEVEL    0

static unsigned m6g06_bl_level = 0;

static void m6g06_lvds_ports_ctrl(Bool_t status)
{
	printk(KERN_INFO "%s: %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
    if (status) {
        aml_write_reg32(P_LVDS_PHY_CNTL3, aml_read_reg32(P_LVDS_PHY_CNTL3) | (1<<0));
        aml_write_reg32(P_LVDS_GEN_CNTL, aml_read_reg32(P_LVDS_GEN_CNTL) | (1 << 3)); // enable fifo
        aml_write_reg32(P_LVDS_PHY_CNTL4, aml_read_reg32(P_LVDS_PHY_CNTL4) | (0x2f<<0));  //enable LVDS phy port
    }else {
    	aml_write_reg32(P_LVDS_PHY_CNTL3, aml_read_reg32(P_LVDS_PHY_CNTL3) & ~(1<<0));
		aml_write_reg32(P_LVDS_PHY_CNTL5, aml_read_reg32(P_LVDS_PHY_CNTL5) & ~(1<<11));  //shutdown lvds phy

        aml_write_reg32(P_LVDS_PHY_CNTL4, aml_read_reg32(P_LVDS_PHY_CNTL4) & ~(0x7f<<0));  //disable LVDS phy port
        aml_write_reg32(P_LVDS_GEN_CNTL, aml_read_reg32(P_LVDS_GEN_CNTL) & ~(1 << 3)); // disable fifo		
    }
}

// Refer to H/W schematics
static void m6g06_backlight_power_ctrl(Bool_t status)
{
    printk(KERN_INFO "%s() Power %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
    if( status == ON ){
        msleep(30);
		m6g06_lvds_ports_ctrl(ON);
        aml_set_reg32_bits(P_LED_PWM_REG0, 1, 12, 2);
        msleep(300); // wait for PWM charge
		//BL_EN -> GPIOD_1: 1
#if (BL_CTL==BL_CTL_GPIO)	
		gpio_out(PAD_GPIOD_1, 1); 
#elif (BL_CTL==BL_CTL_PWM)		
		aml_set_reg32_bits(P_PWM_PWM_D, 0, 0, 16);  		//pwm low
		aml_set_reg32_bits(P_PWM_PWM_D, PWM_MAX, 16, 16);	//pwm high
		aml_write_reg32(P_PWM_MISC_REG_CD, (aml_read_reg32(P_PWM_MISC_REG_CD) & ~(0x7f<<16)) | ((1 << 23) | (PWM_PRE_DIV<<16) | (1<<1)));  //enable pwm clk & pwm output
		aml_write_reg32(P_PERIPHS_PIN_MUX_2, aml_read_reg32(P_PERIPHS_PIN_MUX_2) | (1<<3));  //enable pwm pinmux
#endif
    }
    else{
#if (BL_CTL==BL_CTL_GPIO)
        //BL_EN -> GPIOD_1: 0		
		gpio_out(PAD_GPIOD_1, 0); 
#elif (BL_CTL==BL_CTL_PWM)
		aml_write_reg32(P_PWM_MISC_REG_CD, aml_read_reg32(P_PWM_MISC_REG_CD) & ~((1 << 23) | (1<<1)));  //disable pwm clk & pwm output
#endif
		
        msleep(30); 
        m6g06_lvds_ports_ctrl(OFF);
		msleep(30);
    }
}
#define MIN_LEVEL  0x0f
#define MAX_LEVEL  0x02
#define BL_MID_LEVEL    128
#define BL_MAPPED_MID_LEVEL    102
static void m6g06_set_backlight_level(unsigned level)
{
    m6g06_bl_level = level;
    printk(KERN_DEBUG "%s: %d\n", __FUNCTION__, m6g06_bl_level);
 
    level = (level > BL_MAX_LEVEL ? BL_MAX_LEVEL : (level < BL_MIN_LEVEL ? BL_MIN_LEVEL : level));
 
    if (level > BL_MID_LEVEL) {
                level = ((level - BL_MID_LEVEL)*(BL_MAX_LEVEL-BL_MAPPED_MID_LEVEL))/(BL_MAX_LEVEL - BL_MID_LEVEL) + BL_MAPPED_MID_LEVEL; 
    } else {  
                level = (level*BL_MAPPED_MID_LEVEL)/BL_MID_LEVEL;
    }
 
#if (BL_CTL==BL_CTL_GPIO)
    if (level < 15){   //after mapping ,DIM brightness 20 is maped to 20*102/128 = 15
        level = 15;
    }
    else {
        level = MIN_LEVEL - ((level - 15) * (MIN_LEVEL - MAX_LEVEL)) / (255 - 15); 
    }
    aml_set_reg32_bits(P_LED_PWM_REG0, level, 0, 4);
#elif (BL_CTL==BL_CTL_PWM)
    level = level * PWM_MAX / BL_MAX_LEVEL ;
    aml_set_reg32_bits(P_PWM_PWM_D, (PWM_MAX - level), 0, 16);  //pwm low
    aml_set_reg32_bits(P_PWM_PWM_D, level, 16, 16);				//pwm high
#endif
}

static unsigned m6g06_get_backlight_level(void)
{
    printk(KERN_DEBUG "%s: %d\n", __FUNCTION__, m6g06_bl_level);
    return m6g06_bl_level;
}

static void m6g06_lcd_power_ctrl(Bool_t status)
{
    printk(KERN_INFO "%s() Power %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
    if (status) {
        //GPIOA27 -> LCD_PWR_EN#: 1  lcd 3.3v
		gpio_out(PAD_GPIOA_27, 1);        
        msleep(20);
	
        //GPIOC2 -> VCCx3_EN: 0
#ifdef CONFIG_AW_AXP
		axp_gpio_set_io(3,1);     
		axp_gpio_set_value(3, 0); 
#endif			
    }
    else {
        //GPIOC2 -> VCCx3_EN: 1 
#ifdef CONFIG_AW_AXP
		axp_gpio_set_io(3,0);		
#endif		
		msleep(20);
		
        //GPIOA27 -> LCD_PWR_EN#: 0  lcd 3.3v
        gpio_out(PAD_GPIOA_27, 0);
    }
}

static int m6g06_lcd_suspend(void *args)
{
    args = args;

    printk(KERN_INFO "LCD suspending...\n");
    m6g06_backlight_power_ctrl(OFF);
    m6g06_lcd_power_ctrl(OFF);
    return 0;
}

static int m6g06_lcd_resume(void *args)
{
    args = args;

    printk(KERN_INFO "LCD resuming...\n");
    m6g06_lcd_power_ctrl(ON);
    m6g06_backlight_power_ctrl(ON);
    return 0;
}

#define H_ACTIVE		1024
#define V_ACTIVE		600
#define H_PERIOD		1344
#define V_PERIOD		636
#define VIDEO_ON_PIXEL	80
#define VIDEO_ON_LINE	22

// Define LVDS physical PREM SWING VCM REF
static Lvds_Phy_Control_t lcd_lvds_phy_control =
{
    .lvds_prem_ctl = 0x0,       
    .lvds_swing_ctl = 0x4,      
    .lvds_vcm_ctl = 0x7,
    .lvds_ref_ctl = 0x15,
};

//Define LVDS data mapping, pn swap.
static Lvds_Config_t lcd_lvds_config=
{
    .lvds_repack=1,   //data mapping  //0:JEDIA mode, 1:VESA mode
    .pn_swap=0,       //0:normal, 1:swap    
};

Lcd_Config_t m6g06_lcd_config = {
    // Refer to LCD Spec
    .lcd_basic = {
        .h_active = H_ACTIVE,
        .v_active = V_ACTIVE,
        .h_period = H_PERIOD,
        .v_period = V_PERIOD,
    	.screen_ratio_width = 16,
     	.screen_ratio_height = 9,
        .lcd_type = LCD_DIGITAL_LVDS,    //LCD_DIGITAL_TTL  //LCD_DIGITAL_LVDS  //LCD_DIGITAL_MINILVDS
        .lcd_bits = 8,  //8  //6
    },

    .lcd_timing = {
        .pll_ctrl = 0x1023c,  //42.9M, 50.2Hz
        .div_ctrl = 0x18813,  
        .clk_ctrl = 0x1111, //[19:16]ss_ctrl, [12]pll_sel, [8]div_sel, [4]vclk_sel, [3:0]xd
        //.sync_duration_num = 502,
        //.sync_duration_den = 10,
    
		.video_on_pixel = VIDEO_ON_PIXEL,
		.video_on_line = VIDEO_ON_LINE,
		
		.sth1_hs_addr = 10,
		.sth1_he_addr = 20,
		.sth1_vs_addr = 0,
		.sth1_ve_addr = V_PERIOD - 1,
		.stv1_hs_addr = 10,
		.stv1_he_addr = 20,
		.stv1_vs_addr = 2,
		.stv1_ve_addr = 4,
		
		.pol_cntl_addr = (0x0 << LCD_CPH1_POL) |(0x1 << LCD_HS_POL) | (0x1 << LCD_VS_POL),
		.inv_cnt_addr = (0<<LCD_INV_EN) | (0<<LCD_INV_CNT),
		.tcon_misc_sel_addr = (1<<LCD_STV1_SEL) | (1<<LCD_STV2_SEL),
		.dual_port_cntl_addr = (1<<LCD_TTL_SEL) | (1<<LCD_ANALOG_SEL_CPH3) | (1<<LCD_ANALOG_3PHI_CLK_SEL) | (0<<LCD_RGB_SWP) | (0<<LCD_BIT_SWP),		
	}, 

    .lcd_effect = {
        .gamma_cntl_port = (1 << LCD_GAMMA_EN) | (0 << LCD_GAMMA_RVS_OUT) | (1 << LCD_GAMMA_VCOM_POL),
        .gamma_vcom_hswitch_addr = 0,
        .rgb_base_addr = 0xf0,
        .rgb_coeff_addr = 0x74a, 
    },

	.lvds_mlvds_config = {
        .lvds_config = &lcd_lvds_config,
		.lvds_phy_control = &lcd_lvds_phy_control,
    },
	
    .lcd_power_ctrl = {
        .cur_bl_level = 0,
        .power_ctrl = m6g06_lcd_power_ctrl,
        .backlight_ctrl = m6g06_backlight_power_ctrl,
        .get_bl_level = m6g06_get_backlight_level,
        .set_bl_level = m6g06_set_backlight_level,
        .lcd_suspend = m6g06_lcd_suspend,
        .lcd_resume = m6g06_lcd_resume,
    },
};

static void lcd_setup_gamma_table(Lcd_Config_t *pConf)
{
    int i;
	
	const unsigned short gamma_adjust[256] = {
        0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
        32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,
        64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,
        96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,
        128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,
        160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,
        192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,
        224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255
    };

    for (i=0; i<256; i++) {
        pConf->lcd_effect.GammaTableR[i] = gamma_adjust[i] << 2;
        pConf->lcd_effect.GammaTableG[i] = gamma_adjust[i] << 2;
        pConf->lcd_effect.GammaTableB[i] = gamma_adjust[i] << 2;
    }
}

static void lcd_video_adjust(Lcd_Config_t *pConf)
{
	int i;
	
	const signed short video_adjust[33] = { -999, -937, -875, -812, -750, -687, -625, -562, -500, -437, -375, -312, -250, -187, -125, -62, 0, 62, 125, 187, 250, 312, 375, 437, 500, 562, 625, 687, 750, 812, 875, 937, 1000};
	
	for (i=0; i<33; i++)
	{
		pConf->lcd_effect.brightness[i] = video_adjust[i];
		pConf->lcd_effect.contrast[i]   = video_adjust[i];
		pConf->lcd_effect.saturation[i] = video_adjust[i];
		pConf->lcd_effect.hue[i]        = video_adjust[i];
	}
}

static void lcd_sync_duration(Lcd_Config_t *pConf)
{
	unsigned m, n, od, div, xd;
	unsigned pre_div;
	unsigned sync_duration;
	
	m = ((pConf->lcd_timing.pll_ctrl) >> 0) & 0x1ff;
	n = ((pConf->lcd_timing.pll_ctrl) >> 9) & 0x1f;
	od = ((pConf->lcd_timing.pll_ctrl) >> 16) & 0x3;
	div = ((pConf->lcd_timing.div_ctrl) >> 4) & 0x7;
	xd = ((pConf->lcd_timing.clk_ctrl) >> 0) & 0xf;
	
	od = (od == 0) ? 1:((od == 1) ? 2:4);
	switch(pConf->lcd_basic.lcd_type)
	{
		case LCD_DIGITAL_TTL:
			pre_div = 1;
			break;
		case LCD_DIGITAL_LVDS:
			pre_div = 7;
			break;
		default:
			pre_div = 1;
			break;
	}
	
	sync_duration = m*24*100/(n*od*(div+1)*xd*pre_div);	
	sync_duration = ((sync_duration * 100000 / H_PERIOD) * 10) / V_PERIOD;
	sync_duration = (sync_duration + 5) / 10;	
	
	pConf->lcd_timing.sync_duration_num = sync_duration;
	pConf->lcd_timing.sync_duration_den = 10;
}

static struct aml_bl_platform_data m6g06_backlight_data =
{
    //.power_on_bl = power_on_backlight,
    //.power_off_bl = power_off_backlight,
    .get_bl_level = m6g06_get_backlight_level,
    .set_bl_level = m6g06_set_backlight_level,
    .max_brightness = 255,
    .dft_brightness = 10,
};

static struct platform_device m6g06_backlight_device = {
    .name = "aml-bl",
    .id = -1,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
        .platform_data = &m6g06_backlight_data,
    },
};

static struct aml_lcd_platform __initdata m6g06_lcd_data = {
    .lcd_conf = &m6g06_lcd_config,
};

static struct platform_device __initdata * m6g06_lcd_devices[] = {
    &meson_device_lcd,
//    &meson_device_vout,
    &m6g06_backlight_device,
};

int  m6g06_lcd_init(void)
{
    int err;
	lcd_sync_duration(&m6g06_lcd_config);
	lcd_setup_gamma_table(&m6g06_lcd_config);
	lcd_video_adjust(&m6g06_lcd_config);	
    meson_lcd_set_platdata(&m6g06_lcd_data, sizeof(struct aml_lcd_platform));
    err = platform_add_devices(m6g06_lcd_devices, ARRAY_SIZE(m6g06_lcd_devices));
    return err;
}


