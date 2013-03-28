/*
 * customer/boards/board-m3f04.c
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
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <plat/platform.h>
#include <plat/plat_dev.h>
#include <plat/platform_data.h>
#include <plat/lm.h>
#include <plat/regops.h>
#include <mach/am_regs.h>
#include <mach/clock.h>
#include <mach/map.h>
#include <mach/i2c_aml.h>
#include <mach/nand.h>
#include <mach/usbclock.h>
#include <mach/usbsetting.h>
#include <mach/gpio.h>

#include "board-m3f04.h"

#ifdef CONFIG_MMC_AML
#include <mach/mmc.h>
#endif
#include <linux/i2c-aml.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif
#ifdef CONFIG_ANDROID_PMEM
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/android_pmem.h>
#endif

#ifdef CONFIG_SENSORS_MMC328X
#include <linux/mmc328x.h>
#endif
#ifdef CONFIG_SENSORS_MMA8452
#include <linux/mma8452.h>
#endif

//#ifdef CONFIG_AM_UART
#include <linux/uart-aml.h>
//#endif
#ifdef CONFIG_SUSPEND
#include <mach/pm.h>
#endif

#ifdef CONFIG_CARDREADER
#include <mach/card_io.h>
#include <mach/gpio.h>
#endif

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE
#include <media/amlogic/aml_camera.h>
#endif

#include "board-m3f04-pinmux.h"

#define DEBUG_GPIO_INTERFACE
#ifdef DEBUG_GPIO_INTERFACE
#include <mach/gpio.h>
#include <mach/gpio_data.h>
#endif

#ifdef CONFIG_SND_AML_M3
#include <sound/soc.h>
#include <sound/aml_platform.h>
#endif

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

#if defined(CONFIG_ADC_KEYPADS_AM)||defined(CONFIG_ADC_KEYPADS_AM_MODULE)
#include <linux/input.h>
#include <linux/adc_keypad.h>

static struct adc_key adc_kp_key[] = {
    {KEY_MENU,          "menu", CHAN_4, 0, 60},
    {KEY_VOLUMEDOWN,    "vol-", CHAN_4, 140, 60},
    {KEY_VOLUMEUP,      "vol+", CHAN_4, 266, 60},
    {KEY_BACK,          "exit", CHAN_4, 386, 60},
    {KEY_HOME,          "home", CHAN_4, 508, 60},
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


#if 0
static  typeof(devices_pins)  *cur_board_pinmux;
static struct aml_uart_platform  __initdata aml_uart_plat = {
    .uart_line[0]       =  UART_AO,
    .uart_line[1]       =   UART_A,
    .uart_line[2]       =   UART_B,
    .uart_line[3]       =   UART_C,
};

    
//#endif
static  struct aml_i2c_platform_a  __initdata aml_i2c_data_a={
       .udelay = 8,
       .timeout =10 ,
       .resource ={
            .start =    I2C_MASTER_A_START,
            .end   =    I2C_MASTER_A_END,
            .flags =    IORESOURCE_MEM,
       }
   } ;
static struct aml_i2c_platform_b   __initdata aml_i2c_data_b={
       .freq= 3,
       .wait=7,
       .resource ={
            .start =    I2C_MASTER_B_START,
            .end   =    I2C_MASTER_B_END,
            .flags =    IORESOURCE_MEM,
       }
   };
#endif

static void set_usb_a_vbus_power(char is_power_on)
{
#ifdef DEBUG_GPIO_INTERFACE

#define USB_A_POW_GPIO	PAD_GPIOD_9
#define USB_A_POW_GPIO_BIT_ON	true
#define USB_A_POW_GPIO_BIT_OFF	false	
	if(is_power_on){
		printk(KERN_INFO "set usb port power on (board gpio %d)!\n",USB_A_POW_GPIO);
		gpio_out(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT_ON);	
	}else{
		printk(KERN_INFO "set usb port power off (board gpio %d)!\n",USB_A_POW_GPIO);
		gpio_out(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT_OFF);
	}

#else
#define USB_A_PWR_GPIO_MODE P_PREG_PAD_GPIO2_EN_N
#define USB_A_PWR_GPIO	P_PREG_PAD_GPIO2_O
#define USB_A_PWR_GPIO_BIT	25
#define USB_A_PWR_GPIO_ON		1
#define USB_A_PWR_GPIO_OFF	0

	aml_set_reg32_bits(P_PERIPHS_PIN_MUX_7,0,16,1);
	aml_set_reg32_bits(P_PERIPHS_PIN_MUX_3,0,26,1);
	aml_set_reg32_bits(P_PERIPHS_PIN_MUX_0,0,29,1);

	aml_set_reg32_bits(USB_A_PWR_GPIO_MODE,0,USB_A_PWR_GPIO_BIT,1);

	if(is_power_on){
		printk(KERN_INFO "set usb port power on (board gpio %d)!\n",USB_A_PWR_GPIO_BIT);
        	aml_set_reg32_bits(USB_A_PWR_GPIO,USB_A_PWR_GPIO_ON,USB_A_PWR_GPIO_BIT,1);
	}else{
		printk(KERN_INFO "set usb port power off (board gpio %d)!\n",USB_A_PWR_GPIO_BIT);
        	aml_set_reg32_bits(USB_A_PWR_GPIO,USB_A_PWR_GPIO_OFF,USB_A_PWR_GPIO_BIT,1);
	}

	printk("%x %x\n",aml_read_reg32(USB_A_PWR_GPIO_MODE),aml_read_reg32(USB_A_PWR_GPIO));
#endif
}

static  int __init setup_usb_devices(void)
{
	struct lm_device * usb_ld_a;
	usb_ld_a = alloc_usb_lm_device(USB_PORT_IDX_A);
	usb_ld_a->param.usb.set_vbus_power = set_usb_a_vbus_power;
	lm_device_register(usb_ld_a);
#if defined(CONFIG_RTL8XXX_CU_USB_WIFI) || defined(CONFIG_RTL8XXX_CU_USB_WIFI_MODULE)
	struct lm_device * usb_ld_b;
	{ //set_usb_phy_id_mode(USB_PHY_PORT_B, USB_PHY_MODE_SW_HOST);
		int i;
		int time_dly = 50000;
		unsigned int reg = PREI_USB_PHY_B_REG4;
		CLEAR_CBUS_REG_MASK(reg, PREI_USB_PHY_MODE_MASK);
		i=0;
		while (i++ < time_dly) {};
		SET_CBUS_REG_MASK(reg, PREI_USB_PHY_MODE_SW_HOST);
		i=0;
		while (i++<time_dly){};
	}
	usb_ld_b = alloc_usb_lm_device(USB_PORT_IDX_B);
	lm_device_register(usb_ld_b);
#endif
	return 0;
}

#ifdef CONFIG_ITK_CAPACITIVE_TOUCHSCREEN
///#include <mach/gpio-old.h>
#include <linux/i2c/itk.h>
#define GPIO_ITK_PENIRQ ((GPIOA_bank_bit0_27(16)<<16) | GPIOA_bit_bit0_27(16))
#define GPIO_ITK_RST ((GPIOC_bank_bit0_15(3)<<16) | GPIOC_bit_bit0_15(3))
#define ITK_INT INT_GPIO_0
static int itk_init_irq(void)
{
#ifdef DEBUG_GPIO_INTERFACE
    gpio_set_status(PAD_GPIOA_16, gpio_status_in);
    gpio_irq_set(PAD_GPIOA_16, GPIO_IRQ(0,GPIO_IRQ_FALLING));
#else
	/* set input mode */
	gpio_direction_input(GPIO_ITK_PENIRQ);
	/* set gpio interrupt #0 source=GPIOD_24, and triggered by falling edge(=1) */
	gpio_enable_edge_int(gpio_to_idx(GPIO_ITK_PENIRQ), 1, ITK_INT-INT_GPIO_0);
#endif
	return 0;
}

static int itk_get_irq_level(void)
{
	return gpio_in_get(PAD_GPIOA_16);
}

static void touch_on(int flag)
{
	printk("enter %s flag=%d \n",__FUNCTION__,flag);
	if(flag)
		gpio_out_high(PAD_GPIOC_3);
	else
		gpio_out_low(PAD_GPIOC_3);
}

static struct itk_platform_data itk_pdata = {
	.init_irq = &itk_init_irq,
	.get_irq_level = &itk_get_irq_level,
	.touch_on =  touch_on,
	.tp_max_width = 3328,
	.tp_max_height = 2432,
	.lcd_max_width = 800,
	.lcd_max_height = 600,
	.xpol = 1,
	.ypol = 1,
};
#endif

#ifdef CONFIG_PIXCIR_CAPACITIVE_TOUCHSCREEN
#include <linux/i2c/pixcir_i2c_ts.h>
static struct pixcir_i2c_ts_platform_data pixcir_pdata = {
    .gpio_shutdown = PAD_GPIOC_3,
    .gpio_irq = PAD_GPIOA_16,
    .xmin = 0,
    .xmax = 1025,
    .ymin = 0,
    .ymax = 600,
    .swap_xy = 0,
    .xpol = 0,
    .ypol = 0,
    .point_id_available = 0,
};
#endif


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

static void  camera_power_on_init(void)
{
    //power on camera
	  udelay(1000);
    SET_CBUS_REG_MASK(HHI_ETH_CLK_CNTL,0x30f);// 24M XTAL
    SET_CBUS_REG_MASK(HHI_DEMOD_PLL_CNTL,0x232);// 24M XTAL
 	
}

#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_GC0308)
static int gc0308_init(void)
{
    unsigned pwm_cnt;
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_1, (1<<29));
    SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_2, (1<<2));

    pwm_cnt = get_ddr_pll_clk()/48000000 - 1;
    pwm_cnt &= 0xffff;
    WRITE_CBUS_REG(PWM_PWM_C, (pwm_cnt<<16) | pwm_cnt);
    SET_CBUS_REG_MASK(PWM_MISC_REG_CD, (1<<15)|(0<<8)|(1<<4)|(1<<0)); //select ddr pll for source, and clk divide
    
    camera_power_on_init();

    gpio_out(PAD_GPIOY_10, 0);
    msleep(20);

    gpio_out(PAD_GPIOY_10, 1);
    msleep(20);

    gpio_out(PAD_GPIOA_25, 0);
    msleep(20);
    return 0;
}
static void gc0308_v4l2_init(void)
{
    gc0308_init();
}
static void gc0308_v4l2_uninit(void)
{
    printk( "amlogic camera driver: gc0308_v4l2_uninit. \n");
    gpio_out(PAD_GPIOA_25, 1); // set camera power disable
}
static void gc0308_v4l2_early_suspend(void)
{
    gpio_out(PAD_GPIOA_25, 1); // set camera power disable
}

static void gc0308_v4l2_late_resume(void)
{
    gpio_out(PAD_GPIOA_25, 0); // set camera power enable
}

struct aml_camera_i2c_fig1_s custom_gc0308_script[] = {  
	{0x14,0x11}, // h_v
	{0x26,0x01},
	{0xff,0xff}, 
};

static aml_plat_cam_data_t video_gc0308_data = {
    .name = "video-gc0308",
    .video_nr = 1,
    .device_init = gc0308_v4l2_init,
    .device_uninit = gc0308_v4l2_uninit,
    .early_suspend = gc0308_v4l2_early_suspend,
    .custom_init_script = custom_gc0308_script,
    .late_resume = gc0308_v4l2_late_resume,
};
#endif //CONFIG_VIDEO_AMLOGIC_CAPTURE_GC0308


#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_GT2005)
static int gt2005_v4l2_init(void)
{
	  unsigned pwm_cnt;
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_1, (1<<29));
    SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_2, (1<<2));

    pwm_cnt = get_ddr_pll_clk()/48000000 - 1;
    pwm_cnt &= 0xffff;
    WRITE_CBUS_REG(PWM_PWM_C, (pwm_cnt<<16) | pwm_cnt);
    SET_CBUS_REG_MASK(PWM_MISC_REG_CD, (1<<15)|(0<<8)|(1<<4)|(1<<0)); //select ddr pll for source, and clk divide
    
    camera_power_on_init();

	  //camera_power_on_init();
		printk( "---------------------------------gt2005_init\n");
	  // reset low 
	  gpio_out(PAD_GPIOY_10, 0);
	  // set camera power disanable 
	  gpio_out(PAD_GPIOA_24, 0);
	
    msleep(20);
  
    // reset high
    gpio_out(PAD_GPIOY_10, 1);
    msleep(20);
    
    // set camera power enable
    gpio_out(PAD_GPIOA_24, 1);
}

static int gt2005_v4l2_uninit(void)
{
	printk( "amlogic camera driver: gt2005_v4l2_uninit. \n");
    // set camera power disable
	gpio_out(PAD_GPIOA_24, 0);

}

static void gt2005_v4l2_early_suspend(void)
{
    /// set camera VDD disable
	gpio_out(PAD_GPIOA_24, 0);

}

static void gt2005_v4l2_late_resume(void)
{
	gpio_out(PAD_GPIOA_24, 1);
}

static aml_plat_cam_data_t video_gt2005_data = {
	.name="video-gt2005",
	.video_nr=0,
	.device_init=gt2005_v4l2_init,
	.device_uninit=gt2005_v4l2_uninit,
	.early_suspend=gt2005_v4l2_early_suspend,
	.late_resume=gt2005_v4l2_late_resume,
};
#endif //CONFIG_VIDEO_AMLOGIC_CAPTURE_GT2005


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
#endif /* CONFIG_POST_PROCESS_MANAGER */

#if defined(CONFIG_I2C_AML) || defined(CONFIG_I2C_HW_AML)
static bool pinmux_dummy_share(bool select)
{
    return select;
}

static pinmux_item_t aml_i2c_0_pinmux_item[] = {
    {
        .reg = 5,
        //.clrmask = (3<<24)|(3<<30),
        .setmask = 3<<26
    },
    PINMUX_END_ITEM
};

static struct aml_i2c_platform aml_i2c_plat = {
    .wait_count         = 50000,
    .wait_ack_interval  = 5,
    .wait_read_interval	= 5,
    .wait_xfer_interval	= 5,
    .master_no          = AML_I2C_MASTER_A,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_300K,

    .master_pinmux      = {
        .chip_select    = pinmux_dummy_share,
        .pinmux         = &aml_i2c_0_pinmux_item[0]
    }
};

static pinmux_item_t aml_i2c_1_pinmux_item[]={
    {
        .reg = 5,
        //.clrmask = (3<<28)|(3<<26),
        .setmask = 3<<30
    },
    PINMUX_END_ITEM
};

static struct aml_i2c_platform aml_i2c_plat1 = {
    .wait_count         = 50000,
    .wait_ack_interval	= 5,
    .wait_read_interval	= 5,
    .wait_xfer_interval	= 5,
    .master_no          = AML_I2C_MASTER_B,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_300K,

    .master_pinmux      = {
        .chip_select    = pinmux_dummy_share,
        .pinmux         = &aml_i2c_1_pinmux_item[0]
    }
};

static pinmux_item_t aml_i2c_ao_pinmux_item[] = {
    {
        .reg		= AO,
        .clrmask	= 3<<1,
        .setmask	= 3<<5
    },
    PINMUX_END_ITEM
};

static struct aml_i2c_platform aml_i2c_plat2 = {
    .wait_count         = 50000,
    .wait_ack_interval  = 5,
    .wait_read_interval = 5,
    .wait_xfer_interval	= 5,
    .master_no          = AML_I2C_MASTER_AO,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_100K,

    .master_pinmux      = {
        .pinmux         = &aml_i2c_ao_pinmux_item[0]
    }
};

static struct resource aml_i2c_resource[] = {
    [0] = {
        .start = MESON_I2C_MASTER_A_START,
        .end   = MESON_I2C_MASTER_A_END,
        .flags = IORESOURCE_MEM,
	}
};

static struct resource aml_i2c_resource1[] = {
    [0] = {
        .start = MESON_I2C_MASTER_A_START,
        .end   = MESON_I2C_MASTER_A_END,
        .flags = IORESOURCE_MEM,
    }
};

static struct resource aml_i2c_resource2[] = {
	[0]= {
		.start =    MESON_I2C_MASTER_AO_START,
		.end   =    MESON_I2C_MASTER_AO_END,
		.flags =    IORESOURCE_MEM,
	}
};

static struct platform_device aml_i2c_device = {
    .name         = "aml-i2c",
    .id       = 0,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource),
    .resource     = aml_i2c_resource,
    .dev = {
        .platform_data = &aml_i2c_plat,
    },
};

static struct platform_device aml_i2c_device1 = {
    .name         = "aml-i2c",
    .id       = 1,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource1),
    .resource     = aml_i2c_resource1,
    .dev = {
        .platform_data = &aml_i2c_plat1,
    },
};

static struct platform_device aml_i2c_device2 = {
    .name         = "aml-i2c",
    .id       = 2,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource2),
    .resource     = aml_i2c_resource2,
    .dev = {
        .platform_data = &aml_i2c_plat2,
    },
};

static struct i2c_board_info __initdata aml_i2c_bus_info[] = {
#ifdef CONFIG_ITK_CAPACITIVE_TOUCHSCREEN
    {
        I2C_BOARD_INFO("itk", 0x41),
        .irq = ITK_INT,
        .platform_data = (void *)&itk_pdata,
    },
#endif
#ifdef CONFIG_PIXCIR_CAPACITIVE_TOUCHSCREEN
    {
        I2C_BOARD_INFO("pixcir168", 0x5c),
        .irq = INT_GPIO_0,
        .platform_data = (void *)&pixcir_pdata,
    },
#endif
#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_GC0308
	{
        /*gc0308 i2c address is 0x42/0x43*/
		I2C_BOARD_INFO("gc0308_i2c",  0x42 >> 1),
		.platform_data = (void *)&video_gc0308_data,
	},
#endif
#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_GT2005
	{
		I2C_BOARD_INFO("gt2005_i2c", 0x78 >> 1),
		.platform_data = (void *)&video_gt2005_data,
	},
#endif
};

static struct i2c_board_info __initdata aml_i2c_bus_info_1[] = {
#ifdef CONFIG_SENSORS_MMC328X
	{
		I2C_BOARD_INFO(MMC328X_I2C_NAME,  MMC328X_I2C_ADDR),
	},
#endif
#ifdef CONFIG_SENSORS_MMA8452
	{
		I2C_BOARD_INFO(MMA8452_I2C_NAME,  MMA8452_I2C_ADDR),
	},
#endif
};

static int __init aml_i2c_init(void)
{
	i2c_register_board_info(0, aml_i2c_bus_info,
		ARRAY_SIZE(aml_i2c_bus_info));
	i2c_register_board_info(1, aml_i2c_bus_info_1,
		ARRAY_SIZE(aml_i2c_bus_info_1));

	return 0;
}
#endif

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

static struct resource meson_codec_resource[] = {
    [0] = {
        .start = CODEC_ADDR_START,
        .end   = CODEC_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = STREAMBUF_ADDR_START,
	 .end = STREAMBUF_ADDR_END,
	 .flags = IORESOURCE_MEM,
    },
};

static  int __init setup_devices_resource(void)
{
     setup_fb_resource(meson_fb_resource, ARRAY_SIZE(meson_fb_resource));
     setup_codec_resource(meson_codec_resource,ARRAY_SIZE(meson_codec_resource));
     return 0;
}

#if 0
static  int __init setup_i2c_devices(void)
{
    //just a sample 
    meson_i2c1_set_platdata(&aml_i2c_data_a,sizeof(struct aml_i2c_platform_a ));
    meson_i2c2_set_platdata(&aml_i2c_data_b,sizeof(struct aml_i2c_platform_b ));    
    return 0;
}



static  int __init setup_uart_devices(void)
{
#if 0    
    static pinmux_set_t temp;
//#if defined(CONFIG_AM_UART)
    aml_uart_plat.public.pinmux_cfg.setup=NULL; //NULL repsent use defaut pinmux_set.
    aml_uart_plat.public.pinmux_cfg.clear=NULL;
    aml_uart_plat.public.clk_src=clk_get_sys("clk81", NULL);
    temp.pinmux=cur_board_pinmux[DEVICE_PIN_ITEM_UART];
    aml_uart_plat.public.pinmux_set.pinmux=cur_board_pinmux[DEVICE_PIN_ITEM_UART];
    aml_uart_plat.pinmux_uart[0]=&temp;
    meson_uart_set_platdata(&aml_uart_plat,sizeof(struct aml_uart_platform));
#endif   
    return 0;
//#endif    
}
#endif

#if defined(CONFIG_TVIN_BT656IN)
static void __init bt656in_pinmux_init(void)
{
    SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_8, 0xf<<6);
    SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_3, 1<<21);
}


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



static void __init device_pinmux_init(void)
{
#if 0 ///@todo Jerry Yu, Compile break , enable it later	
    clearall_pinmux();
    /*other deivce power on*/
    uart_set_pinmux(UART_PORT_AO, UART_AO_GPIO_AO0_AO1_STD);
    /*pinmux of eth*/
    eth_pinmux_init();
    set_audio_pinmux(AUDIO_IN_JTAG); // for MIC input
    set_audio_pinmux(AUDIO_OUT_TEST_N); //External AUDIO DAC
    set_audio_pinmux(SPDIF_OUT_GPIOA); //SPDIF GPIOA_6
#endif

	#if defined(CONFIG_TVIN_BT656IN)
    bt656in_pinmux_init();
	#endif
    
#if 1
    //set clk for wifi
    WRITE_CBUS_REG(HHI_GEN_CLK_CNTL,(READ_CBUS_REG(HHI_GEN_CLK_CNTL)&(~(0x7f<<0)))|((0<<0)|(1<<8)|(7<<9)) );
    CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_EN_N, (1<<15));
    SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_3, (1<<22));
#endif 
}

static void __init  device_clk_setting(void)
{
#if 0 ///@todo Jerry Yu, Compile break , enable it later	
    /*Configurate the ethernet clock*/
    eth_clk_set(ETH_CLKSRC_MISC_CLK, get_misc_pll_clk(), (50 * CLK_1M), 0);
#endif    
}

static void disable_unused_model(void)
{
#if 0 ///@todo Jerry Yu, Compile break , enable it later	
    CLK_GATE_OFF(VIDEO_IN);
    CLK_GATE_OFF(BT656_IN);
#endif    
}
static void __init meson_cache_init(void)
{
#ifdef CONFIG_CACHE_L2X0
    /*
     * Early BRESP, I/D prefetch enabled
     * Non-secure enabled
     * 128kb (16KB/way),
     * 8-way associativity,
     * evmon/parity/share disabled
     * Full Line of Zero enabled
         * Bits:  .111 .... .100 0010 0000 .... .... ...1
     */
    l2x0_init((void __iomem *)IO_PL310_BASE, 0x7c420001, 0xff800fff);
#endif
}

static void  __init backup_board_pinmux(void)
{//devices_pins in __initdata section ,it will be released .
#if 0
        cur_board_pinmux=kmemdup(devices_pins, sizeof(devices_pins), GFP_KERNEL);
        printk(KERN_INFO " cur_board_pinmux=%p",cur_board_pinmux[0]);
        printk(KERN_INFO " cur_board_pinmux=%p",&(cur_board_pinmux[0]));
	printk(KERN_INFO " cur_board_pinmux=%x",cur_board_pinmux[0][0]);
	printk(KERN_INFO " cur_board_pinmux=%x",cur_board_pinmux[0][0]->reg);
#endif    
}

static  pinmux_item_t   uart_pins[]={
		{
			.reg=PINMUX_REG(AO),
			.clrmask=3<<16,
			.setmask=3<<11
		},
		PINMUX_END_ITEM
	};
static  pinmux_set_t   aml_uart_ao={
	.chip_select=NULL,
	.pinmux=&uart_pins[0]
};
static struct aml_uart_platform  __initdata aml_uart_plat = {
    
    .uart_line[0]       =  UART_AO,
    .uart_line[1]       =   UART_A,
    .uart_line[2]       =   UART_B,
    .uart_line[3]       =   UART_C,

    
    .pinmux_uart[0] = (void*)&aml_uart_ao,
    .pinmux_uart[1] = NULL,
    .pinmux_uart[2] = NULL,
    .pinmux_uart[3] = NULL,
};

static struct platform_device aml_uart_device = {
    .name         = "mesonuart",
    .id       = -1,
    .num_resources    = 0,
    .resource     = NULL,
    .dev = {
        .platform_data = &aml_uart_plat,
    },
};

#if defined(CONFIG_AML_RTC)
static struct platform_device meson_rtc_device = {
	.name	= "aml_rtc",
	.id	= -1,
};
#endif /* CONFIG_AML_RTC */

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
        .size = 512*SZ_1M,
    },
    {
        .name = "cache",
        .offset = 640*SZ_1M+40*SZ_1M,
        .size = 128*SZ_1M,
    },
    {
        .name = "userdata",
        .offset = 768*SZ_1M+40*SZ_1M,
        .size = 512*SZ_1M,
    },
    {
        .name = "NFTL_Part",
        .offset = MTDPART_OFS_APPEND,
        .size = MTDPART_SIZ_FULL,
    },
};


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
        .T_REA = 20,
        .T_RHOH = 15,
    },
#endif
    {
        .name = NAND_NORMAL_NAME,
        .chip_enable_pad = (AML_NAND_CE0/* | (AML_NAND_CE1 << 4) | (AML_NAND_CE2 << 8) | (AML_NAND_CE3 << 12)*/),
        .ready_busy_pad = (AML_NAND_CE0 /*| (AML_NAND_CE0 << 4) | (AML_NAND_CE1 << 8) | (AML_NAND_CE1 << 12)*/),
        .platform_nand_data = {
            .chip =  {
                .nr_chips = 1,
                .nr_partitions = ARRAY_SIZE(normal_partition_info),
                .partitions = normal_partition_info,
                .options = (NAND_TIMING_MODE5 | NAND_ECC_BCH60_1K_MODE | NAND_TWO_PLANE_MODE),
            },
        },
        .T_REA = 20,
        .T_RHOH = 15,
    }
};

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

#ifdef CONFIG_RTL8XXX_CU_EXT_POWER_CTRL
/* usb wifi power 1:power on  0:power off */
void extern_usb_wifi_power(int is_power)
{
    printk(KERN_INFO "usb_wifi_power %s\n", is_power ? "On" : "Off");
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_1,(1<<11));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0,(1<<18));
    CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_EN_N, (1<<8));
    if (is_power)
        CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<8));
    else
        SET_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<8));
}
EXPORT_SYMBOL(extern_usb_wifi_power);
#endif


#ifdef CONFIG_CARDREADER
static struct resource amlogic_card_resource[] = {
    [0] = {
        .start = 0x1200230,   //physical address
        .end   = 0x120024c,
        .flags = 0x200,
    }
};
/* WIFI ON Flag */
static int WIFI_ON;
/* BT ON Flag */
static int BT_ON;
/* WL_BT_REG_ON control function */
static void reg_on_control(int is_on)
{
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_1,(1<<11)); //WIFI_RST GPIO mode
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0,(1<<18)); //WIFI_EN GPIO mode
	CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_EN_N, (1<<8));//GPIOC_8 ==WIFI_EN
    
    if(is_on){
		SET_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<8));
    }
	else{
        /* only pull donw reg_on pin when wifi and bt off */
        if((!WIFI_ON) && (!BT_ON)){
		    CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<8));
            printk("WIFI BT Power down\n");
        }
	}
}

void extern_wifi_power(int is_power)
{
    WIFI_ON = is_power;
    reg_on_control(is_power);
    /*
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_1,(1<<11));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0,(1<<18));
	CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_EN_N, (1<<8));
	if(is_power)
		SET_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<8));
	else
		CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<8));
		*/
}

EXPORT_SYMBOL(extern_wifi_power);

void sdio_extern_init(void)
{
    #if defined(CONFIG_BCM4329_HW_OOB) || defined(CONFIG_BCM4329_OOB_INTR_ONLY)/* Jone add */
    gpio_set_status(PAD_GPIOX_11,gpio_status_in);
    gpio_irq_set(PAD_GPIOX_11,GPIO_IRQ(4,GPIO_IRQ_RISING));
    #endif 
    extern_wifi_power(1);
}

static struct aml_card_info  amlogic_card_info[] = {
    [0] = {
        .name = "sd_card",
        .work_mode = CARD_HW_MODE,
        .io_pad_type = SDIO_B_CARD_0_5,
        .card_ins_en_reg = CARD_GPIO_ENABLE,
        .card_ins_en_mask = PREG_IO_29_MASK,
        .card_ins_input_reg = CARD_GPIO_INPUT,
        .card_ins_input_mask = PREG_IO_29_MASK,
        .card_power_en_reg = CARD_GPIO_ENABLE,
        .card_power_en_mask = PREG_IO_31_MASK,
        .card_power_output_reg = CARD_GPIO_OUTPUT,
        .card_power_output_mask = PREG_IO_31_MASK,
        .card_power_en_lev = 0,
        .card_wp_en_reg = 0,
        .card_wp_en_mask = 0,
        .card_wp_input_reg = 0,
        .card_wp_input_mask = 0,
        .card_extern_init = 0,
    },
    [1] = {
        .name = "sdio_card",
        .work_mode = CARD_HW_MODE,
        .io_pad_type = SDIO_A_GPIOX_0_3,
        .card_ins_en_reg = 0,
        .card_ins_en_mask = 0,
        .card_ins_input_reg = 0,
        .card_ins_input_mask = 0,
        .card_power_en_reg = EGPIO_GPIOC_ENABLE,
        .card_power_en_mask = PREG_IO_7_MASK,
        .card_power_output_reg = EGPIO_GPIOC_OUTPUT,
        .card_power_output_mask = PREG_IO_7_MASK,
        .card_power_en_lev = 1,
        .card_wp_en_reg = 0,
        .card_wp_en_mask = 0,
        .card_wp_input_reg = 0,
        .card_wp_input_mask = 0,
        .card_extern_init = sdio_extern_init,
    },
};

void extern_wifi_reset(int is_on)
{
    unsigned int val;
    
    /*output*/
	//set WIFI_RST 
    val = aml_read_reg32(amlogic_card_info[1].card_power_en_reg);
    val &= ~(amlogic_card_info[1].card_power_en_mask);
    aml_write_reg32(amlogic_card_info[1].card_power_en_reg, val);
        
    if(is_on){
        /*high*/
        val = aml_read_reg32(amlogic_card_info[1].card_power_output_reg);
        val |=(amlogic_card_info[1].card_power_output_mask);
        aml_write_reg32(amlogic_card_info[1].card_power_output_reg, val);
        printk("on val = %x\n", val);
    }
    else{
        /*low*/
        val = aml_read_reg32(amlogic_card_info[1].card_power_output_reg);
        val &=~(amlogic_card_info[1].card_power_output_mask);
        aml_write_reg32(amlogic_card_info[1].card_power_output_reg, val);
        printk("off val = %x\n", val);
    }

    printk("ouput %x, bit %d, level %x, bit %d\n",
            amlogic_card_info[1].card_power_en_reg,
            amlogic_card_info[1].card_power_en_mask,
            amlogic_card_info[1].card_power_output_reg,
            amlogic_card_info[1].card_power_output_mask);
    return;
}
EXPORT_SYMBOL(extern_wifi_reset);

static struct aml_card_platform amlogic_card_platform = {
    .card_num = ARRAY_SIZE(amlogic_card_info),
    .card_info = amlogic_card_info,
};

static struct platform_device amlogic_card_device = { 
    .name = "AMLOGIC_CARD", 
    .id    = -1,
    .num_resources = ARRAY_SIZE(amlogic_card_resource),
    .resource = amlogic_card_resource,
    .dev = {
        .platform_data = &amlogic_card_platform,
    },
};	
#endif //END CONFIG_CARDREADER	
#ifdef CONFIG_MMC_AML
struct platform_device;
struct mmc_host;
struct mmc_card;
struct mmc_ios;

//return 1: no inserted  0: inserted
static int aml_sdio_detect(struct aml_sd_host * host)
{
	SET_CBUS_REG_MASK(PREG_PAD_GPIO5_EN_N,1<<29);//CARD_6
	return READ_CBUS_REG(PREG_PAD_GPIO5_I)&(1<<29)?1:0;
}

static void  cpu_sdio_pwr_prepare(unsigned port)
{
    switch(port)
    {
        case M3_SDIO_PORT_A:
            aml_clr_reg32_mask(P_PREG_PAD_GPIO4_EN_N,0x30f);
            aml_clr_reg32_mask(P_PREG_PAD_GPIO4_O   ,0x30f);
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_8,0x3f);
            break;
        case M3_SDIO_PORT_B:
            aml_clr_reg32_mask(P_PREG_PAD_GPIO5_EN_N,0x3f<<23);
            aml_clr_reg32_mask(P_PREG_PAD_GPIO5_O   ,0x3f<<23);
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_2,0x3f<<10);
            break;
        case M3_SDIO_PORT_C:
            aml_clr_reg32_mask(P_PREG_PAD_GPIO3_EN_N,0xc0f);
            aml_clr_reg32_mask(P_PREG_PAD_GPIO3_O   ,0xc0f);
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_6,(0x3f<<24));
            break;
        case M3_SDIO_PORT_XC_A:
            break;
        case M3_SDIO_PORT_XC_B:
            break;
        case M3_SDIO_PORT_XC_C:
            break;
    }
}

static int cpu_sdio_init(unsigned port)
{
    switch(port)
    {
        case M3_SDIO_PORT_A:
        		aml_set_reg32_mask(P_PERIPHS_PIN_MUX_8,0x3f);\
        		break;
        case M3_SDIO_PORT_B:
        		aml_set_reg32_mask(P_PERIPHS_PIN_MUX_2,0x3f<<10);
        		break;
        case M3_SDIO_PORT_C://SDIOC GPIOB_2~GPIOB_7
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_2,(0x1f<<22));
            aml_set_reg32_mask(P_PERIPHS_PIN_MUX_6,(0x3f<<24));
            break;
        case M3_SDIO_PORT_XC_A:
            #if 0
            //sdxc controller can't work
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_8,(0x3f<<0));
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_3,(0x0f<<27));
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_7,((0x3f<<18)|(0x7<<25)));
            //aml_set_reg32_mask(P_PERIPHS_PIN_MUX_5,(0x1f<<10));//data 8 bit
            aml_set_reg32_mask(P_PERIPHS_PIN_MUX_5,(0x1b<<10));//data 4 bit
            #endif
            break;
        case M3_SDIO_PORT_XC_B:
            //sdxc controller can't work
            //aml_set_reg32_mask(P_PERIPHS_PIN_MUX_2,(0xf<<4));
            break;
        case M3_SDIO_PORT_XC_C:
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
	
	.sdio_port = M3_SDIO_PORT_B,
	.max_width	= 4,
	.host_caps	= (MMC_CAP_4_BIT_DATA |
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
	.name		= "aml_sd_mmc",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(aml_mmc_resource),
	.resource	= aml_mmc_resource,
	.dev		= {
		.dma_mask		=       &aml_mmc_device_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
		.platform_data		= &aml_mmc_def_platdata,
	},
};
#endif //CONFIG_MMC_AML

#if defined(CONFIG_AML_AUDIO_DSP)
static struct resource audiodsp_resources[] = {
    [0] = {
        .start = AUDIODSP_ADDR_START,
        .end   = AUDIODSP_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device audiodsp_device = {
    .name       = "audiodsp",
    .id         = 0,
    .num_resources = ARRAY_SIZE(audiodsp_resources),
    .resource      = audiodsp_resources,
};
#endif //CONFIG_AML_AUDIO_DSP

#if defined(CONFIG_SND_AML_M3)
static struct resource aml_m3_audio_resource[] = {
    [0] =   {
        .start  =   0,
        .end        =   0,
        .flags  =   IORESOURCE_MEM,
    },
};

extern char* get_vout_mode_internal(void);

/* Check current mode, 0: panel; 1: !panel*/
static int get_display_mode(void) {
	int ret = 0;
	if(strncmp("panel", get_vout_mode_internal(), 5))
		ret = 1;
	return ret;
}

static int aml_m3_is_hp_pluged(void)
{
	if(get_display_mode() != 0) //if !panel, return 1 to mute spk		
		return 1;
		
	return READ_CBUS_REG_BITS(PREG_PAD_GPIO0_I, 19, 1); //return 1: hp pluged, 0: hp unpluged.
}

static void mute_spk(int flag)
{
    printk("%s() %s speaker\n", __FUNCTION__, flag ? "Mute" : "Unmute");
    if(flag){
        gpio_out_low(PAD_GPIOC_4); // mute speak
    }else{
        gpio_out_high(PAD_GPIOC_4); // unmute speak
    }
}

static void unmute_spk_pre_start(void)
{
	printk("aml_audio_pre_start\n");
	gpio_out_high(PAD_GPIOC_4); // unmute speak
}

static void mute_spk_post_stop(void)
{
	printk("aml_audio_post_stop\n");
       gpio_out_low(PAD_GPIOC_4); // mute speak
}

static struct aml_audio_platform aml_audio_pdata = {
    .is_hp_pluged = &aml_m3_is_hp_pluged,
    .mute_spk = &mute_spk,
    .audio_pre_start = &unmute_spk_pre_start,
    .audio_post_stop = &mute_spk_post_stop,
};

/* machine platform device */
static struct platform_device aml_m3_audio = {
    .name               = "aml_m3_audio",
    .id             = -1,
    .resource       =   aml_m3_audio_resource,
    .num_resources  =   ARRAY_SIZE(aml_m3_audio_resource),
    .dev = {
        .platform_data = &aml_audio_pdata,
    },
};

/* dai platform device */
static struct platform_device aml_dai = {
    .name               = "aml-m3-dai",
    .id             = 0,
    .num_resources  =   0,
};

/* pcm audio platform device */
static struct platform_device aml_audio = {
    .name           = "aml-audio",
    .id             = -1,
    .num_resources  = 0,
};
#endif //CONFIG_SND_AML_M3

#if defined(CONFIG_SUSPEND)
typedef struct {
	char name[32];
	unsigned reg;
	unsigned bits;
	unsigned enable;
} pinmux_data_t;

#define MAX_PINMUX	1
static pinmux_data_t pinmux_data[MAX_PINMUX] = {
	{"HDMI", 	0, (1<<2)|(1<<1)|(1<<0), 						1},
};

static unsigned pinmux_backup[6];
static void save_pinmux(void)
{
	int i;
	for (i=0;i<6;i++)
		pinmux_backup[i] = aml_read_reg32(P_PERIPHS_PIN_MUX_0+i);
	for (i=0;i<MAX_PINMUX;i++){
		if (pinmux_data[i].enable){
			printk("%s %x\n", pinmux_data[i].name, pinmux_data[i].bits);	
			aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_0+pinmux_data[i].reg, pinmux_data[i].bits);
		}
	}
}

static void restore_pinmux(void)
{
	int i;
	for (i=0;i<6;i++)
		aml_write_reg32(P_PERIPHS_PIN_MUX_0+i,pinmux_backup[i]);
}

void f04_set_vccx2(int power_on)
{
    if (power_on) {
        //restore_pinmux();
        printk(KERN_INFO "%s() Power ON\n", __FUNCTION__);
        aml_clr_reg32_mask(P_PREG_PAD_GPIO0_EN_N,(1<<26));
        aml_clr_reg32_mask(P_PREG_PAD_GPIO0_O,(1<<26));
    }
    else {
        printk(KERN_INFO "%s() Power OFF\n", __FUNCTION__);
        aml_clr_reg32_mask(P_PREG_PAD_GPIO0_EN_N,(1<<26));
        aml_set_reg32_mask(P_PREG_PAD_GPIO0_O,(1<<26));
        //save_pinmux();
    }
}

static struct meson_pm_config aml_pm_pdata = {
    .pctl_reg_base = (void *)IO_APB_BUS_BASE,
    .mmc_reg_base = (void *)APB_REG_ADDR(0x1000),
    .hiu_reg_base = (void *)CBUS_REG_ADDR(0x1000),
    .power_key = (1<<8),
    .ddr_clk = 0x00110820,
    .sleepcount = 128,
    .set_vccx2 = f04_set_vccx2,
    .core_voltage_adjust = 7,  //5,8
};

static struct platform_device aml_pm_device = {
    .name           = "pm-meson",
    .dev = {
        .platform_data  = &aml_pm_pdata,
    },
    .id             = -1,
};
#endif //CONFIG_SUSPEND

static struct platform_device  *platform_devs[] = {
///    &meson_device_uart,
    &meson_device_fb,
    &aml_uart_device,
    &meson_device_codec,
#if defined(CONFIG_I2C_AML) || defined(CONFIG_I2C_HW_AML)
    &aml_i2c_device,
    &aml_i2c_device1,
    &aml_i2c_device2,
#endif
#if defined(CONFIG_SND_AML_M3)
    &aml_audio,
    &aml_dai,
    &aml_m3_audio,
#endif
#if defined(CONFIG_KEYPADS_AM)||defined(CONFIG_VIRTUAL_REMOTE)||defined(CONFIG_KEYPADS_AM_MODULE)
    &input_device,
#endif
#ifdef CONFIG_SARADC_AM
    &saradc_device,
#endif
#if defined(CONFIG_ADC_KEYPADS_AM)||defined(CONFIG_ADC_KEYPADS_AM_MODULE)
    &adc_kp_device,
#endif
#if defined(CONFIG_KEY_INPUT_CUSTOM_AM) || defined(CONFIG_KEY_INPUT_CUSTOM_AM_MODULE)
    &input_device_key,
#endif
#if defined(CONFIG_MMC_AML)
    &aml_mmc_device,
#endif
#if defined(CONFIG_CARDREADER)
    &amlogic_card_device,
#endif
#if defined(CONFIG_SUSPEND)
    &aml_pm_device,
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
#if defined(CONFIG_AML_AUDIO_DSP)
    &audiodsp_device,
#endif //CONFIG_AML_AUDIO_DSP
#ifdef CONFIG_AMLOGIC_VIDEOIN_MANAGER
    &vm_device,
#endif
#ifdef CONFIG_POST_PROCESS_MANAGER
    &ppmgr_device,
#endif
#if defined(CONFIG_AML_RTC)
    &meson_rtc_device,
#endif
};

static __init void meson_m3ref_init(void)
{
    backup_board_pinmux();
    meson_cache_init();
    setup_devices_resource();
    ///setup_i2c_devices();

    ///setup_uart_devices();
    device_clk_setting();
    device_pinmux_init();

#ifdef CONFIG_RTL8XXX_CU_EXT_POWER_CTRL
    extern_usb_wifi_power(0);
#endif

    ///platform_device_register(&aml_uart_device);
    platform_add_devices(platform_devs, ARRAY_SIZE(platform_devs));

    f04_lcd_init();
    f04_power_init();
    f04_set_vccx2(1);
    setup_usb_devices();

    /*
    printk(KERN_INFO"TEST GPIOC 4");
    gpio_out_high(PAD_GPIOC_4);
    gpio_out_low(PAD_GPIOC_4);
*/
#if defined(CONFIG_TOUCHSCREEN_ADS7846)
    ads7846_init_gpio();
    spi_register_board_info(spi_board_info_list, ARRAY_SIZE(spi_board_info_list));
#endif

#if defined(CONFIG_I2C_AML) || defined(CONFIG_I2C_HW_AML)
    aml_i2c_init();
#endif



    disable_unused_model();
}

/***********************************************************************
 * IO Mapping
 **********************************************************************/
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
        .length     = SZ_4K,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_APB_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_APB_BUS_PHY_BASE),
        .length     = SZ_512K,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_AOBUS_BASE,
        .pfn        = __phys_to_pfn(IO_AOBUS_PHY_BASE),
        .length     = SZ_1M,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_AHB_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_AHB_BUS_PHY_BASE),
        .length     = SZ_16M,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_APB2_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_APB2_BUS_PHY_BASE),
        .length     = SZ_512K,
        .type       = MT_DEVICE,
    }, {
        .virtual    = PAGE_ALIGN(__phys_to_virt(RESERVED_MEM_START)),
        .pfn        = __phys_to_pfn(RESERVED_MEM_START),
        .length     = RESERVED_MEM_END - RESERVED_MEM_START + 1,
        .type       = MT_DEVICE,
    },
#ifdef CONFIG_MESON_SUSPEND
	{
        .virtual    = PAGE_ALIGN(0xdff00000),
        .pfn        = __phys_to_pfn(0x1ff00000),
        .length     = SZ_1M,
        .type       = MT_MEMORY,
	},
#endif
};

static  void __init meson_map_io(void)
{
    iotable_init(meson_io_desc, ARRAY_SIZE(meson_io_desc));
}

static __init void m3_irq_init(void)
{
    meson_init_irq();
}

static __init void m3_fixup(struct machine_desc *mach, struct tag *tag, char **cmdline, struct meminfo *m)
{
    struct membank *pbank;
    m->nr_banks = 0;
    pbank = &m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(PHYS_MEM_START);
    pbank->size  = SZ_64M & PAGE_MASK;
///    pbank->node  = PHYS_TO_NID(PHYS_MEM_START);
    m->nr_banks++;
    pbank = &m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(RESERVED_MEM_END + 1);
#ifdef CONFIG_MESON_SUSPEND
    pbank->size  = (PHYS_MEM_END-RESERVED_MEM_END-SZ_1M) & PAGE_MASK;
#else
    pbank->size  = (PHYS_MEM_END-RESERVED_MEM_END) & PAGE_MASK;
#endif
   /// pbank->node  = PHYS_TO_NID(RESERVED_MEM_END + 1);
    m->nr_banks++;

}


MACHINE_START(MESON3_F04, "Amlogic Meson3 f04 customer platform")
    .boot_params    = BOOT_PARAMS_OFFSET,
    .map_io         = meson_map_io,
    .init_irq       = m3_irq_init,
    .timer          = &meson_sys_timer,
    .init_machine   = meson_m3ref_init,
    .fixup          = m3_fixup,
    .video_start    = RESERVED_MEM_START,
    .video_end      = RESERVED_MEM_END,
MACHINE_END

MACHINE_START(VMX25, "Amlogic Meson3 f04 customer platform (legacy)")
    .boot_params    = BOOT_PARAMS_OFFSET,
    .map_io         = meson_map_io,
    .init_irq       = m3_irq_init,
    .timer          = &meson_sys_timer,
    .init_machine   = meson_m3ref_init,
    .fixup          = m3_fixup,
    .video_start    = RESERVED_MEM_START,
    .video_end      = RESERVED_MEM_END,
MACHINE_END
