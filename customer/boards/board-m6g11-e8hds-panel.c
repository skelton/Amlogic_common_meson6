/*
 * arch/arm/mach-meson3/board-m6g11-e8hds-panel.c
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

#include "board-m6g11-e8hds.h"

#ifdef CONFIG_AW_AXP
extern int axp_gpio_set_io(int gpio, int io_state);
extern int axp_gpio_get_io(int gpio, int *io_state);
extern int axp_gpio_set_value(int gpio, int value);
extern int axp_gpio_get_value(int gpio, int *value);
#endif

extern Lcd_Config_t m6g11_e8hds_lcd_config;

//*****************************************
// Define backlight control method
//*****************************************
#define BL_CTL_GPIO 	0
#define BL_CTL_PWM  	1
#define BL_CTL      	BL_CTL_GPIO

//backlight controlled level in driver, define the real backlight level
#if (BL_CTL==BL_CTL_GPIO)
#define	DIM_MAX			0x0
#define	DIM_MIN			0xd	
#elif (BL_CTL==BL_CTL_PWM)
#define	PWM_CNT			60000			//PWM_CNT <= 65535
#define	PWM_PRE_DIV		0				//pwm_freq = 24M / (pre_div + 1) / PWM_CNT
#define PWM_MAX         (PWM_CNT * 8 / 10)		
#define PWM_MIN         (PWM_CNT * 3 / 10)  //according to LED-driver-IC reqirement(or customer reqirement), 0 <= PWM_MIN < PWM_MAX
#endif

//backlight level in UI menu
#define BL_MAX_LEVEL    	255
#define BL_MIN_LEVEL    	20		//Keep this value the same as UI min-limit
#define DEFAULT_BL_LEVEL	128

static unsigned bl_level = DEFAULT_BL_LEVEL;
static Bool_t bl_state = ON;
//*****************************************

static void minilvds_ports_ctrl(Bool_t status)
{
	printk(KERN_INFO "%s: %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
	if (status) 
	{		
		WRITE_MPEG_REG(0x202c, READ_MPEG_REG(0x202c) | (0xe6<<12));  //set tcon pinmux		
		WRITE_MPEG_REG(0x2013, READ_MPEG_REG(0x2013) | (1<<0));
		WRITE_MPEG_REG(0x2012, READ_MPEG_REG(0x2012) & ~(1<<0));  //set sth high
		
        aml_write_reg32(P_LVDS_GEN_CNTL, aml_read_reg32(P_LVDS_GEN_CNTL) | (1 << 3)); // enable fifo
        aml_write_reg32(P_LVDS_PHY_CNTL4, aml_read_reg32(P_LVDS_PHY_CNTL4) | (0x7f<<0));  //enable miniLVDS phy port
	}
	else
	{	
		aml_write_reg32(P_LVDS_PHY_CNTL3, aml_read_reg32(P_LVDS_PHY_CNTL3) & ~(1<<0));
        aml_write_reg32(P_LVDS_PHY_CNTL5, aml_read_reg32(P_LVDS_PHY_CNTL5) & ~(1<<11));  //shutdown lvds phy
        aml_write_reg32(P_LVDS_PHY_CNTL4, aml_read_reg32(P_LVDS_PHY_CNTL4) & ~(0x7f<<0));  //disable LVDS phy port
        aml_write_reg32(P_LVDS_GEN_CNTL, aml_read_reg32(P_LVDS_GEN_CNTL) & ~(1 << 3)); // disable fifo		

		WRITE_MPEG_REG(0x202c, READ_MPEG_REG(0x202c) & ~(0xe6<<12));  //clear tcon pinmux			
		WRITE_MPEG_REG(0x2012, READ_MPEG_REG(0x2012) | (0xe6<<2));  //set tcon as GPIO input
		WRITE_MPEG_REG(0x2012, READ_MPEG_REG(0x2012) | (1<<0));		
	}
	bl_state = status;
}


static void backlight_power_ctrl(Bool_t status)
{
    printk(KERN_INFO "%s() Power %s\n", __FUNCTION__, (status ? "ON" : "OFF"));

    if( status == ON ){        
        aml_set_reg32_bits(P_LED_PWM_REG0, 1, 12, 2);
        msleep(20); 
		//BL_EN -> GPIOD_1: 1
#if (BL_CTL==BL_CTL_GPIO)	
		gpio_out(PAD_GPIOD_1, 1); 
#elif (BL_CTL==BL_CTL_PWM)
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
    }
	//bl_state = status;
}

static const int backlight_level[17] = {
	250,	// 0
	240,	// 1
	230,	// 2
	220,	// 3
	210,	// 4
	200,	// 5
	190,	// 6
	180,	// 7
	160,	// 8
	130,	// 9
	110,	// 10
	90,		// 11
	70,		// 12
	50,		// 13
	30,		// 14
	20,		// 15
	0		// off
};

static unsigned set_level;
static void set_backlight_level(unsigned level)
{    
	int i;
	level = (level > BL_MAX_LEVEL ? BL_MAX_LEVEL : (level < BL_MIN_LEVEL ? 0 : level));	
	set_level =  level;   
    //printk(KERN_INFO "%s: %d\n", __FUNCTION__, level);	

	if ((set_level == 0)&&(bl_level != 0))
	{
		backlight_power_ctrl(OFF);		
	}
	else if (set_level > 0)
	{	
#if (BL_CTL==BL_CTL_GPIO)
		for (i=0; i < 17; i++) {
			if (set_level >= backlight_level[i]) {
				break;
			}
		}
		i = (i>15) ? 15 : i;
		aml_set_reg32_bits(P_LED_PWM_REG0, i, 0, 4);		
#elif (BL_CTL==BL_CTL_PWM) 		
		set_level = (PWM_MAX - PWM_MIN) * (set_level - BL_MIN_LEVEL) / (BL_MAX_LEVEL - BL_MIN_LEVEL) + PWM_MIN;
		aml_set_reg32_bits(P_PWM_PWM_D, (PWM_CNT - set_level), 0, 16);  //pwm low
		aml_set_reg32_bits(P_PWM_PWM_D, set_level, 16, 16);				//pwm high	
#endif

		if ((level != 0)&&(bl_level == 0)&&(bl_state == ON))
		{
			//gpio_out(PAD_GPIOD_1, 1);
			backlight_power_ctrl(ON);
			printk("##############on bl\n");		
		}
	}
    bl_level = level;
}

static unsigned get_backlight_level(void)
{
    printk(KERN_DEBUG "%s: %d\n", __FUNCTION__, bl_level);
    return bl_level;
}
extern 		void fast_recovery(int on);
static void lcd_power_ctrl(Bool_t status)
{
    printk(KERN_INFO "%s() Power %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
    if (status) {
        //GPIOA27 -> LCD_PWR_EN#: 0  lcd 3.3v
		gpio_out(PAD_GPIOA_27, 0);        
        msleep(20);
	
        //GPIOC2 -> VCCx3_EN: 0
        //gpio_out(PAD_GPIOC_2, 0);
#ifdef CONFIG_AW_AXP
		axp_gpio_set_io(3,1);     
		axp_gpio_set_value(3, 0); 
#endif
		msleep(20);	

		//data ports enable
		minilvds_ports_ctrl(ON);
		msleep(200);
	}
    else {
		//msleep(250); 
		//data ports disable
        minilvds_ports_ctrl(OFF);
		msleep(20);		
        
		//GPIOC2 -> VCCx3_EN: 1        
        //gpio_out(PAD_GPIOC_2, 1);
#ifdef CONFIG_AW_AXP
		axp_gpio_set_io(3,0);		
#endif		
		msleep(20);
		
        //GPIOA27 -> LCD_PWR_EN#: 1  lcd 3.3v
        gpio_out(PAD_GPIOA_27, 1);
		//msleep(500);        
    }
}

static int lcd_suspend(void *args)
{
    args = args;	
    printk(KERN_INFO "LCD suspending...\n");
	backlight_power_ctrl(OFF);
	lcd_power_ctrl(OFF);
    return 0;
}

static int lcd_resume(void *args)
{
    args = args;
    printk(KERN_INFO "LCD resuming...\n");
	lcd_power_ctrl(ON);
	backlight_power_ctrl(ON);
    
    return 0;
}

#define H_ACTIVE		1280
#define V_ACTIVE		768
#define H_PERIOD		1440
#define V_PERIOD		810
#define VIDEO_ON_PIXEL	80
#define VIDEO_ON_LINE	32

//Define miniLVDS tcon channel
#define STH_CHANNEL 	0   //sth
#define STV_CHANNEL 	1   //stv
#define LD_CHANNEL 		6   //ld
#define CPV_CHANNEL 	5   //ckv
#define OE_CHANNEL 		2   //oev
#define EVEN_CHANNEL 	4   //3D PWM+
#define POL_CHANNEL 	7   //pol
#define PCLK_CHANNEL 	3   //3D PWM-

static Mlvds_Tcon_Config_t lcd_mlvds_tcon_config[8]=
{
	{STH_CHANNEL, 0, 1400+5, 1400+5+30, VIDEO_ON_LINE, VIDEO_ON_LINE+768-1, 0, 0, 0, 0},
    {STV_CHANNEL, 1, 1400+10-(1440/2), 1400+10-(1440/2)+1448, VIDEO_ON_LINE, VIDEO_ON_LINE, 1400+10-(1440/2), 1400+10-(1440/2)+1448, VIDEO_ON_LINE, VIDEO_ON_LINE},
    {LD_CHANNEL, 0, 1400+10, 1400+10+10, VIDEO_ON_LINE-1, VIDEO_ON_LINE+768, 1400+10+1448,1400+10+10+1448, VIDEO_ON_LINE-1, VIDEO_ON_LINE+768+1},
    {CPV_CHANNEL, 0, 700, 1400, VIDEO_ON_LINE-1, VIDEO_ON_LINE+768, 700+1448, 1400+1448, VIDEO_ON_LINE-1, VIDEO_ON_LINE+768},
    {OE_CHANNEL, 0, 1300, 1440, VIDEO_ON_LINE, VIDEO_ON_LINE+768, 1300+1448, 1440+1448, VIDEO_ON_LINE, VIDEO_ON_LINE+768},
    {EVEN_CHANNEL, 1, 0, 1448+1448-1, 0, (V_PERIOD/2)-1, 0, 0, 0, 0},
    {POL_CHANNEL, 0, 1400+10-(1440/2), 1400+10-(1440/2)+1440-1448, VIDEO_ON_LINE, VIDEO_ON_LINE+768, 1400+10-(1440/2)+1448, 1400+10-(1440/2)+1440-1448+1448, VIDEO_ON_LINE, VIDEO_ON_LINE+768},
	{PCLK_CHANNEL, 1, 0, 1448+1448-1, V_PERIOD/2, V_PERIOD-1, 0, 0, 0, 0}
};

// Define miniLVDS dual/singal gate, pair num, bit num etc.
static Mlvds_Config_t lcd_mlvds_config =
{
    .mlvds_insert_start = 0x45,
    .total_line_clk = 1448,  
    .test_dual_gate = 1,    
    .test_pair_num = 6,  
    .scan_function = 0,  
    .phase_select = 1,
    .TL080_phase =3,
};

// Define LVDS physical PREM SWING VCM REF
static Lvds_Phy_Control_t lcd_lvds_phy_control =
{
    .lvds_prem_ctl = 0x0,       
    .lvds_swing_ctl = 0x4,      
    .lvds_vcm_ctl = 0x7,
    .lvds_ref_ctl = 0x15,
};

Lcd_Config_t m6g11_e8hds_lcd_config = {
    // Refer to LCD Spec
    .lcd_basic = {
        .h_active = H_ACTIVE,
        .v_active = V_ACTIVE,
        .h_period = H_PERIOD,
        .v_period = V_PERIOD,
    	.screen_ratio_width = 16,
     	.screen_ratio_height = 9,
        .lcd_type = LCD_DIGITAL_MINILVDS,    //LCD_DIGITAL_TTL  //LCD_DIGITAL_LVDS  //LCD_DIGITAL_MINILVDS
        .lcd_bits = 6,  //8  //6
    },

    .lcd_timing = {
        .pll_ctrl = 0x1023a,  //42.9M, 50.2Hz
        .div_ctrl = 0x18813,  
        .clk_ctrl = 0x1111, //[19:16]ss_ctrl, [12]pll_sel, [8]div_sel, [4]vclk_sel, [3:0]xd
        //.sync_duration_num = 497,   //this parameter is auto calculated in display driver
        //.sync_duration_den = 10,
    
		.video_on_pixel = VIDEO_ON_PIXEL,
		.video_on_line = VIDEO_ON_LINE,		
		
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
        .mlvds_config = &lcd_mlvds_config,
		.mlvds_tcon_config = &lcd_mlvds_tcon_config[0],
		.lvds_phy_control = &lcd_lvds_phy_control,		    
    },
	
    .lcd_power_ctrl = {
        .cur_bl_level = 0,
        .power_ctrl = lcd_power_ctrl,
        .backlight_ctrl = backlight_power_ctrl,
        .get_bl_level = get_backlight_level,
        .set_bl_level = set_backlight_level,
        .lcd_suspend = lcd_suspend,
        .lcd_resume = lcd_resume,
    },
};

static void lcd_setup_gamma_table(Lcd_Config_t *pConf)
{
    int i;

	const unsigned short gamma_adjust_R[256] = {
	0,2,4,5,7,9,11,13,14,16,18,20,22,23,25,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,
	44,45,46,47,48,49,49,50,51,52,53,54,55,56,57,58,59,59,60,61,62,63,64,65,66,66,67,68,69,70,71,72,
	73,74,75,76,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,90,91,92,93,94,95,96,97,98,99,100,102,103,
	104,105,106,107,108,109,111,112,113,114,115,117,118,119,120,122,123,124,125,127,128,129,130,132,133,134,135,136,137,138,139,140,
	140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,165,166,167,168,169,170,
	171,172,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,
	202,203,204,205,206,207,207,208,209,210,211,211,212,213,214,215,215,216,217,218,219,219,220,221,222,223,223,224,225,226,227,227,
	228,229,230,230,231,232,232,233,234,234,235,236,236,237,238,238,239,240,241,242,243,244,245,246,248,249,250,251,252,253,254,255,
	};

	const unsigned short gamma_adjust_G[256] = {
	0,2,4,5,7,9,11,13,14,16,18,20,22,23,25,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,
	44,45,46,47,48,49,49,50,51,52,53,54,55,56,57,58,59,59,60,61,62,63,64,65,66,66,67,68,69,70,71,72,
	73,74,75,76,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,90,91,92,93,94,95,96,97,98,99,100,102,103,
	104,105,106,107,108,109,111,112,113,114,115,117,118,119,120,122,123,124,125,127,128,129,130,132,133,134,135,136,137,138,139,140,
	140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,165,166,167,168,169,170,
	171,172,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,
	202,203,204,205,206,207,207,208,209,210,211,211,212,213,214,215,215,216,217,218,219,219,220,221,222,223,223,224,225,226,227,227,
	228,229,230,230,231,232,232,233,234,234,235,236,236,237,238,238,239,240,241,242,243,244,245,246,248,249,250,251,252,253,254,255,
	};

	const unsigned short gamma_adjust_B[256] = {
	0,2,4,5,7,9,11,13,14,16,18,20,22,23,25,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,
	44,45,46,47,48,49,49,50,51,52,53,54,55,56,57,58,59,59,60,61,62,63,64,65,66,66,67,68,69,70,71,72,
	73,74,75,76,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,90,91,92,93,94,95,96,97,98,99,100,102,103,
	104,105,106,107,108,109,111,112,113,114,115,117,118,119,120,122,123,124,125,127,128,129,130,132,133,134,135,136,137,138,139,140,
	140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,165,166,167,168,169,170,
	171,172,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,
	202,203,204,205,206,207,207,208,209,210,211,211,212,213,214,215,215,216,217,218,219,219,220,221,222,223,223,224,225,226,227,227,
	228,229,230,230,231,232,232,233,234,234,235,236,236,237,238,238,239,240,241,242,243,244,245,246,248,249,250,251,252,253,254,255,
	};
	

    for (i=0; i<256; i++) {
        pConf->lcd_effect.GammaTableR[i] = gamma_adjust_R[i] << 2;
        pConf->lcd_effect.GammaTableG[i] = gamma_adjust_G[i] << 2;
        pConf->lcd_effect.GammaTableB[i] = gamma_adjust_B[i] << 2;
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
	unsigned m, n, od, div, xd, pre_div;
	unsigned h_period, v_period;
	unsigned sync_duration;
	
	h_period = (pConf->lcd_basic.h_period);
	v_period = (pConf->lcd_basic.v_period);
	m = ((pConf->lcd_timing.pll_ctrl) >> 0) & 0x1ff;
	n = ((pConf->lcd_timing.pll_ctrl) >> 9) & 0x1f;
	od = ((pConf->lcd_timing.pll_ctrl) >> 16) & 0x3;
	div = ((pConf->lcd_timing.div_ctrl) >> 4) & 0x7;	
	
	od = (od == 0) ? 1:((od == 1) ? 2:4);
	switch(pConf->lcd_basic.lcd_type)
	{
		case LCD_DIGITAL_TTL:
			xd = ((pConf->lcd_timing.clk_ctrl) >> 0) & 0xf;
			pre_div = 1;
			break;
		case LCD_DIGITAL_LVDS:
			xd = 1;
			pre_div = 7;
			break;
		case LCD_DIGITAL_MINILVDS:
			xd = 1;
			pre_div = 6;
			break;	
		default:
			pre_div = 1;
			break;
	}
	
	sync_duration = m*24*100/(n*od*(div+1)*xd*pre_div);	
	sync_duration = ((sync_duration * 100000 / h_period) * 10) / v_period;
	sync_duration = (sync_duration + 5) / 10;	
	
	printk(KERN_INFO "sync_duration=%d\n",sync_duration);
	pConf->lcd_timing.sync_duration_num = sync_duration;
	pConf->lcd_timing.sync_duration_den = 10;	
}

static struct aml_bl_platform_data m6g11_e8hds_backlight_data =
{
    //.power_on_bl = power_on_backlight,
    //.power_off_bl = power_off_backlight,
    .get_bl_level = get_backlight_level,
    .set_bl_level = set_backlight_level,
    .max_brightness = 255,
    .dft_brightness = 200,
};

static struct platform_device m6g11_e8hds_backlight_device = {
    .name = "aml-bl",
    .id = -1,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
        .platform_data = &m6g11_e8hds_backlight_data,
    },
};

static struct aml_lcd_platform __initdata m6g11_e8hds_lcd_data = {
    .lcd_conf = &m6g11_e8hds_lcd_config,
};

static struct platform_device __initdata * m6g11_e8hds_lcd_devices[] = {
    &meson_device_lcd,
//    &meson_device_vout,
    &m6g11_e8hds_backlight_device,
};

int  m6g11_e8hds_lcd_init(void)
{
    int err;	
	lcd_sync_duration(&m6g11_e8hds_lcd_config);
	lcd_setup_gamma_table(&m6g11_e8hds_lcd_config);
	lcd_video_adjust(&m6g11_e8hds_lcd_config);	
    meson_lcd_set_platdata(&m6g11_e8hds_lcd_data, sizeof(struct aml_lcd_platform));
    err = platform_add_devices(m6g11_e8hds_lcd_devices, ARRAY_SIZE(m6g11_e8hds_lcd_devices));
    return err;
}


