/*
 *
 * arch/arm/mach-meson6/usbclock.c
 *
 *  Copyright (C) 2011 AMLOGIC, INC.
 *
 *	by Victor Wan 2012.2.14 @Santa Clara, CA
 *
 * License terms: GNU General Public License (GPL) version 2
 * Platform machine definition.
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
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/delay.h>
#include <plat/platform.h>
#include <plat/lm.h>
#include <plat/regops.h>
#include <mach/hardware.h>
#include <mach/memory.h>
#include <mach/clock.h>
#include <mach/reg_addr.h>
#include <mach/usbclock.h>

/*
 * M chip USB clock setting
 */
 
/*
 * Clock source name index must sync with chip's spec
 * M1/M2/M3/M6 are different!
 * This is only for M6
 */
static const char * clock_src_name[] = {
    "XTAL input",
    "XTAL input divided by 2",
    "DDR PLL",
    "MPLL OUT0"
    "MPLL OUT1",
    "MPLL OUT2",
    "FCLK / 2",
    "FCLK / 3"
};
#if 1
static int init_count = 0;
int set_usb_phy_clk(struct lm_device * plmdev,int is_enable)
{

	int port_idx,i;
	usb_peri_reg_t * peri_a,* peri_b,*peri;
	usb_config_data_t config;
	usb_ctrl_data_t control;
	usb_dbg_uart_data_t uart;
	int clk_sel,clk_div,clk_src;
	int time_dly = 500; //usec
	
	if(!plmdev)
		return -1;

	port_idx = plmdev->param.usb.port_idx;
	if(port_idx < 0 || port_idx > 1)
		return -1;
	
	peri_a = (usb_peri_reg_t *)P_USB_ADDR0;
	peri_b = (usb_peri_reg_t *)P_USB_ADDR8;

	if(port_idx == USB_PORT_IDX_A)
		peri = peri_a;
	else
		peri = peri_b;
	
	if(is_enable){
		if(!init_count){
			init_count++;
			aml_set_reg32_bits(P_RESET1_REGISTER, 1, 2, 1);
			for(i = 0; i < 1000; i++)
				udelay(time_dly);

			clk_sel = plmdev->clock.sel;
			clk_div = plmdev->clock.div;
			clk_src = plmdev->clock.src;

			config.d32 = peri_a->config;
			config.b.clk_sel = clk_sel;	
			config.b.clk_div = clk_div; 
		  	config.b.clk_en = 1;
			peri_a->config = config.d32;

			config.d32 = peri_b->config;
			config.b.clk_sel = clk_sel;	
			config.b.clk_div = clk_div; 
		  	config.b.clk_en = 1;
			peri_b->config = config.d32;
			printk(KERN_NOTICE"USB (%d) use clock source: %s\n",port_idx,clock_src_name[clk_sel]);

			control.d32 = peri_b->ctrl;
			control.b.fsel = 2;	/* PHY default is 24M (5), change to 12M (2) */
			control.b.por = 1;
			peri_b->ctrl = control.d32;
			
			control.d32 = peri_a->ctrl;
			control.b.fsel = 2;	/* PHY default is 24M (5), change to 12M (2) */
			control.b.por = 1;
			peri_a->ctrl = control.d32;
			udelay(time_dly);
			control.b.por = 0;
			peri_a->ctrl = control.d32;
			udelay(time_dly);

			/* read back clock detected flag*/
			control.d32 = peri_a->ctrl;
			if(!control.b.clk_detected){
				printk(KERN_ERR"USB (%d) PHY Clock not detected!\n",0);
			}
			control.d32 = peri_b->ctrl;
			if(!control.b.clk_detected){
				printk(KERN_ERR"USB (%d) PHY Clock not detected!\n",1);
			}
		}
		uart.d32 = peri->dbg_uart;
		uart.b.set_iddq = 0;
		peri->dbg_uart = uart.d32;		
	}else{
		init_count--;
		uart.d32 = peri->dbg_uart;
		uart.b.set_iddq = 1;
		peri->dbg_uart = uart.d32;
	}
	dmb();
	
	return 0;
}
#else
static int reset_count = 0;
int set_usb_phy_clk(struct lm_device * plmdev,int is_enable)
{

	int port_idx;
	usb_peri_reg_t * peri;
	usb_config_data_t config;
	usb_ctrl_data_t control;
	int clk_sel,clk_div,clk_src;
	int time_dly = 500; //usec
	
	if(!plmdev)
		return -1;

	port_idx = plmdev->param.usb.port_idx;
	if(port_idx < 0 || port_idx > 1)
		return -1;

	peri = (usb_peri_reg_t *)plmdev->param.usb.phy_tune_reg;
	
	printk(KERN_NOTICE"USB (%d) peri reg base: %x\n",port_idx,(uint32_t)peri);
	if(is_enable){

		clk_sel = plmdev->clock.sel;
		clk_div = plmdev->clock.div;
		clk_src = plmdev->clock.src;

		config.d32 = peri->config;
		config.b.clk_sel = clk_sel;	
		config.b.clk_div = clk_div; 
	  	config.b.clk_en = 1;
		peri->config = config.d32;

		printk(KERN_NOTICE"USB (%d) use clock source: %s\n",port_idx,clock_src_name[clk_sel]);
		
		control.d32 = peri->ctrl;
		control.b.fsel = 2;	/* PHY default is 24M (5), change to 12M (2) */
		control.b.por = 1;
		peri->ctrl = control.d32;
		udelay(time_dly);
		control.b.por = 0;
		peri->ctrl = control.d32;
		udelay(time_dly);

		/* read back clock detected flag*/
		control.d32 = peri->ctrl;
		if(!control.b.clk_detected){
			printk(KERN_ERR"USB (%d) PHY Clock not detected!\n",port_idx);
			return -1;
		}
	}else{
		if(reset_count)
			reset_count--;
		config.d32 = peri->config;
		config.b.clk_en = 0;
		peri->config = config.d32;
		control.d32 = peri->ctrl;
		control.b.por = 1;
		peri->ctrl = control.d32;
	}
	dmb();
	
	return 0;
}
#endif

EXPORT_SYMBOL(set_usb_phy_clk);

