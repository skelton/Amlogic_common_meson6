/*
 * customer/boards/board-m3f04-panel.c
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
#include <linux/delay.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <plat/platform.h>
#include <plat/plat_dev.h>
#include <plat/lm.h>
#include <plat/regops.h>
#include <mach/am_regs.h>
#include <mach/reg_addr.h>
#include <mach/clock.h>
#include <mach/map.h>
#include <mach/gpio.h>
#include <mach/gpio_data.h>

#include <linux/vout/lcdoutc.h>
#include <linux/aml_bl.h>
#include <mach/lcd_aml.h>

#include "board-m3f04.h"

// Define backlight control method
#define BL_CTL_GPIO    0
#define BL_CTL_PWM    1
#define BL_CTL        BL_CTL_GPIO


#define PWM_MAX         60000   // set pwm_freq=24MHz/PWM_MAX (Base on XTAL frequence: 24MHz, 0<PWM_MAX<65535)
#define BL_MAX_LEVEL    255
#define BL_MIN_LEVEL    0

static unsigned m3ref_lcd_brightness = 0;

static void m3ref_lcd_power_ctrl(Bool_t status);

// Refer to H/W schematics
static void m3ref_lcd_backlight_power_ctrl(Bool_t status)
{
    printk(KERN_INFO "%s() Power %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
    m3ref_lcd_power_ctrl(status);

    if (status == ON) {
        aml_set_reg32_bits(P_LED_PWM_REG0, 1, 12, 2);
        msleep(300); // wait for PWM charge
        gpio_out(PAD_GPIOD_1, gpio_status_out);
        gpio_out_high(PAD_GPIOD_1);
    } else {
        //BL_EN -> GPIOD_1: 0
        gpio_out(PAD_GPIOD_1, gpio_status_out);
        gpio_out_low(PAD_GPIOD_1);
    }
}

static void m3ref_lcd_set_brightness(unsigned level)
{
    m3ref_lcd_brightness = level;
    printk(KERN_DEBUG "%s() brightness=%d\n", __FUNCTION__, m3ref_lcd_brightness);

    level = (level > BL_MAX_LEVEL ? BL_MAX_LEVEL
                                  : (level < BL_MIN_LEVEL ? BL_MIN_LEVEL : level));

#if (BL_CTL==BL_CTL_GPIO)
    level = level * 15 / BL_MAX_LEVEL;
    level = 15 - level;
    WRITE_CBUS_REG_BITS(LED_PWM_REG0, level, 0, 4);
#elif (BL_CTL==BL_CTL_PWM)
    level = level * PWM_MAX / BL_MAX_LEVEL ;
    WRITE_CBUS_REG_BITS(PWM_PWM_D, (PWM_MAX - level), 0, 16);  //pwm low
    WRITE_CBUS_REG_BITS(PWM_PWM_D, level, 16, 16);  //pwm high
#endif
}

static unsigned m3ref_lcd_get_brightness(void)
{
    printk(KERN_DEBUG "%s() brightness=%d\n", __FUNCTION__, m3ref_lcd_brightness);
    return m3ref_lcd_brightness;
}

static void m3ref_lcd_power_ctrl(Bool_t status)
{
    printk(KERN_INFO "%s() Power %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
    if (status) {
        // enable lvds fifo
        aml_set_reg32_mask(P_LVDS_GEN_CNTL, (1 << 3));
        // enable minilvds_data channel
        aml_set_reg32_mask(P_LVDS_PHY_CNTL4, 0x2f);
        //msleep(30);
    }
    else {
        // disable minilvds_data channel
        aml_clr_reg32_mask(P_LVDS_PHY_CNTL4, 0x7f);
        // disable lvds fifo
        aml_clr_reg32_mask(P_LVDS_GEN_CNTL, (1 << 3));
    }
}

static int m3ref_lcd_suspend(void *args)
{
    args = args;
    
    printk(KERN_INFO "LCD suspending...\n");
    return 0;
}

static int m3ref_lcd_resume(void *args)
{
    args = args;
    
    printk(KERN_INFO "LCD resuming...\n");
    return 0;
}

#define LCD_WIDTH       1024
#define LCD_HEIGHT      600
#define MAX_WIDTH       1344
#define MAX_HEIGHT      635
#define VIDEO_ON_PIXEL  80
#define VIDEO_ON_LINE   22

static Lvds_Phy_Control_t lcd_lvds_phy_control = {
    .lvds_prem_ctl = 0x1,
    .lvds_swing_ctl = 0x3,
    .lvds_vcm_ctl = 0x1,
    .lvds_ref_ctl = 0xf,
};

static Lvds_Config_t lcd_lvds_config = {
    .lvds_repack = 1,
    .pn_swap = 0,
    .bit_num = 1,
    .lvds_phy_control = &lcd_lvds_phy_control,
};

static Lcd_Config_t m3ref_lcd_config = {

    // Refer to LCD Spec
    .lcd_basic_info = {
        .resolution_w = LCD_WIDTH,
        .resolution_h = LCD_HEIGHT,
        .max_res_w = MAX_WIDTH,
        .max_res_h = MAX_HEIGHT,
        .screen_ratio_w = 16,
        .screen_ratio_h = 9,
        .lcd_type = LCD_LVDS,    //LCD_TTL //LCD_LVDS //LCD_MINILVDS
        .lcd_bits = 8,  //6
    },

    .lcd_clk_ctrl = {
        .pll_ctrl = 0x10232,
        .div_ctrl = 0x18813,
        .clk_ctrl = 0x1111,    //pll_sel,div_sel,vclk_sel,xd
        .sync_duration_num = 502,
        .sync_duration_den = 10,
    },

    .ttl_lvds_tcon = {
        .lvds_tcon = {
            .sth1_hs_addr = 10,
            .sth1_he_addr = 20,
            .sth1_vs_addr = 0,
            .sth1_ve_addr = MAX_HEIGHT - 1,
            .oeh_hs_addr = 0,
            .oeh_he_addr = 0,
            .oeh_vs_addr = 0,
            .oeh_ve_addr = 0,
            .vcom_hswitch_addr = 0,
            .vcom_vs_addr = 0,
            .vcom_ve_addr = 0,
            .cpv1_hs_addr = 0,
            .cpv1_he_addr = 0,
            .cpv1_vs_addr = 0,
            .cpv1_ve_addr = 0,
            .stv1_hs_addr = 10,
            .stv1_he_addr = 20,
            .stv1_vs_addr = 2,
            .stv1_ve_addr = 4,
            .oev1_hs_addr = 0,
            .oev1_he_addr = 0,
            .oev1_vs_addr = 0,
            .oev1_ve_addr = 0,
            
            .pol_cntl_addr = (0x0 << LCD_CPH1_POL) |(0x1 << LCD_HS_POL) | (0x1 << LCD_VS_POL),
            .inv_cnt_addr = (0<<LCD_INV_EN) | (0<<LCD_INV_CNT),
            .tcon_misc_sel_addr = (1<<LCD_STV1_SEL) | (1<<LCD_STV2_SEL),
            .dual_port_cntl_addr = (1<<LCD_TTL_SEL) | (1<<LCD_ANALOG_SEL_CPH3) | (1<<LCD_ANALOG_3PHI_CLK_SEL),

            .video_on_pixel = VIDEO_ON_PIXEL,
            .video_on_line = VIDEO_ON_LINE,
        },
    },

    .lcd_effect = {
        .gamma_cntl_port = (1 << LCD_GAMMA_EN) | (0 << LCD_GAMMA_RVS_OUT) | (1 << LCD_GAMMA_VCOM_POL),
        .gamma_vcom_hswitch_addr = 0,
        .rgb_base_addr = 0xf0,
        .rgb_coeff_addr = 0x74a,
        .dith_cntl_addr = 0x400,    //For 6bits, 0x600      //For 8bits, 0x400
        .brightness = { -999, -937, -875, -812, -750, -687, -625, -562, -500, -437, -375, -312, -250, -187, -125, -62, 0, 62, 125, 187, 250, 312, 375, 437, 500, 562, 625, 687, 750, 812, 875, 937, 1000},
        .contrast   = { -999, -937, -875, -812, -750, -687, -625, -562, -500, -437, -375, -312, -250, -187, -125, -62, 0, 62, 125, 187, 250, 312, 375, 437, 500, 562, 625, 687, 750, 812, 875, 937, 1000},
        .saturation = { -999, -937, -875, -812, -750, -687, -625, -562, -500, -437, -375, -312, -250, -187, -125, -62, 0, 62, 125, 187, 250, 312, 375, 437, 500, 562, 625, 687, 750, 812, 875, 937, 1000},
        .hue        = { -999, -937, -875, -812, -750, -687, -625, -562, -500, -437, -375, -312, -250, -187, -125, -62, 0, 62, 125, 187, 250, 312, 375, 437, 500, 562, 625, 687, 750, 812, 875, 937, 1000},
        .GammaTableR = {
            0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
            32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,
            64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,
            96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,
            128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,
            160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,
            192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,
            224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255
                       },
        .GammaTableG = {
            0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
            32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,
            64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,
            96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,
            128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,
            160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,
            192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,
            224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255
                       },
        .GammaTableB = {
            0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
            32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,
            64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,
            96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,
            128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,
            160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,
            192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,
            224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255
                       },
    },

    .lcd_power_ctrl = {
        .cur_bl_level = 0,
        .power_ctrl = m3ref_lcd_power_ctrl,
        .backlight_ctrl = m3ref_lcd_backlight_power_ctrl,
        .get_bl_level = m3ref_lcd_get_brightness,
        .set_bl_level = m3ref_lcd_set_brightness,
        .lcd_suspend = m3ref_lcd_suspend,
        .lcd_resume = m3ref_lcd_resume,
    },

    .lvds_mlvds_config = {
        .lvds_config = &lcd_lvds_config,
    },
};

static struct aml_bl_platform_data m3ref_backlight_data =
{
    //.power_on_bl = power_on_backlight,
    //.power_off_bl = power_off_backlight,
    .get_bl_level = m3ref_lcd_get_brightness,
    .set_bl_level = m3ref_lcd_set_brightness,
    .max_brightness = 255,
    .dft_brightness = 200,
};

static struct platform_device m3ref_backlight_device = {
    .name = "aml-bl",
    .id = -1,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
        .platform_data = &m3ref_backlight_data,
    },
};

static struct aml_lcd_platform __initdata m3ref_lcd_data = {
    .lcd_conf = &m3ref_lcd_config,
};

static struct platform_device __initdata * m3ref_lcd_devices[] = {
    &meson_device_lcd,
    &meson_device_vout,
    &m3ref_backlight_device,
};

int __init f04_lcd_init(void)
{
    int err;

    meson_lcd_set_platdata(&m3ref_lcd_data, sizeof(struct aml_lcd_platform));
    err = platform_add_devices(m3ref_lcd_devices, ARRAY_SIZE(m3ref_lcd_devices));
    return err;
}
