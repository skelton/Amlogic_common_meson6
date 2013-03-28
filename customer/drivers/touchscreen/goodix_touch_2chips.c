/* drivers/input/touchscreen/goodix_touch.c
 *
 * Copyright (C) 2010 - 2011 Goodix, Inc.
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
 */
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <asm/div64.h>
#include <asm/io.h>
#include <mach/irqs.h>
#include <linux/irq.h>
#include <linux/workqueue.h>>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/device.h>
#include <linux/syscalls.h>
#include <linux/linkage.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <mach/io.h>

#include "goodix_touch_2chips.h"
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/completion.h>
#include <asm/uaccess.h>
#include <linux/reboot.h>
#include <mach/gpio.h>

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
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <plat/platform.h>
#include <plat/plat_dev.h>
#include <plat/platform_data.h>
#include <linux/reboot.h>
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

#define TP_GOODIX_9760_GT8110   1   //copy 9760 tp

#ifdef TOUCH_WITH_LIGHT_KEY


#ifdef CONFIG_REGULATOR
#define LOD4_KEYLED_NAME    "axp20_hdmi"
static struct regulator     *vdd_keyled_osc = NULL; 
#include <linux/regulator/consumer.h>
#endif

int gpio_ioctl(unsigned int flag)
{
#if  defined(CONFIG_REGULATOR)
	if(flag)
		{
		#ifdef CONFIG_REGULATOR
			vdd_keyled_osc = regulator_get(NULL, LOD4_KEYLED_NAME);
			if (IS_ERR(vdd_keyled_osc)) {
				printk("Failed to obtain vdd_keyled_osc\n");
				vdd_keyled_osc = NULL;
			}	
			if (vdd_keyled_osc){
				regulator_set_voltage(vdd_keyled_osc, 3300000, 3300000);
				regulator_enable(vdd_keyled_osc);
			}
		#endif
		}else{
		#ifdef CONFIG_REGULATOR	
			if(vdd_keyled_osc){
			regulator_disable(vdd_keyled_osc);
			vdd_keyled_osc = NULL;
			}
		#endif		
		}

#endif 
	return 0;
}
static int g_pen_down = 0;
static int g_light_on = 0;
//add by hm for  close light_key  for light_key alway on  when system resume 
void light_key_resume_close()
{
	
#if defined(TOUCH_WITH_LIGHT_KEY)
#ifdef CONFIG_REGULATOR
	vdd_keyled_osc = regulator_get(NULL, LOD4_KEYLED_NAME);
	if (IS_ERR(vdd_keyled_osc)) {
		printk("Failed to obtain vdd_keyled_osc\n");
		vdd_keyled_osc = NULL;
	}	
	if (vdd_keyled_osc){
		regulator_set_voltage(vdd_keyled_osc, 0000000, 0000000);
		regulator_enable(vdd_keyled_osc);
	}			
	if(vdd_keyled_osc){
		regulator_disable(vdd_keyled_osc);
		vdd_keyled_osc = NULL;
	}
#endif
#endif
}
EXPORT_SYMBOL(light_key_resume_close);

//end add by hm
#endif

//Add by emdoor jim.kuang 2011-12-22
/**********************************************/
void goodix_irq_reset()
{
	gpio_out(GPIO_GOODIX_IRQ,0);
	msleep(20);
	gpio_set_status(GPIO_GOODIX_IRQ, gpio_status_in);
	return ;
}
/**********************************************/

/*******************************************************	
Description:
	Read data from the i2c slave device;
	This operation consisted of 2 i2c_msgs,the first msg used
	to write the operate address,the second msg used to read data.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:read data bfprobeuffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer
*********************************************************/
static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret=-1;
	int retries = 0;

	msgs[0].flags=!I2C_M_RD;
	msgs[0].addr=client->addr;
	msgs[0].len=1;
	msgs[0].buf=&buf[0];

	msgs[1].flags=I2C_M_RD;
	msgs[1].addr=client->addr;
	msgs[1].len=len-1;
	msgs[1].buf=&buf[1];
#if 0
	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,msgs, 2);
		//if(ret == 2)break;
		if(ret > 0)break;
		retries++;
	}
#else
	ret=i2c_transfer(client->adapter,msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

#endif
	return ret;
}

/*******************************************************	
Description:
	write data to the i2c slave device.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:write data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer.
*********************************************************/
static int i2c_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
	struct i2c_msg msg;
	int ret=-1;
	int retries = 0;

	msg.flags=!I2C_M_RD;
	msg.addr=client->addr;
	msg.len=len;
	msg.buf=data;		
#if 0
	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,&msg, 1);
		if(ret == 1)break;
		retries++;
	}
#else
	ret=i2c_transfer(client->adapter,&msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);
#endif
	return ret;
}

/*******************************************************
Description:
	Goodix touchscreen initialize function.

Parameter:
	ts:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
static int goodix_init_panel(struct goodix_ts_data *ts)
{
	int ret=-1;
	uint8_t rd_cfg_buf[7] = {0x66,};

#ifdef DRIVER_SEND_CFG				
	//for kedi 9.7(puts your config info here,if need send config info)
	/*
	uint8_t config_info[] = {
	0x65,0x00,(TOUCH_MAX_HEIGHT>>8),(TOUCH_MAX_HEIGHT&0xff),
	(TOUCH_MAX_WIDTH>>8),(TOUCH_MAX_WIDTH&0xff),MAX_FINGER_NUM,(0x2C | INT_TRIGGER),
	0x11,0x11,0x32,0x02,0x08,0x10,0x20,0x00,
	0x00,0x88,0x88,0x88,0x03,0x13,0x32,0x64,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
	0x08,0x09,0x0A,0x0B,0x0C,0xFF,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,
	0x17,0x18,0x19,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00
	};
	*/

//#if defined(TP_GOODIX_SIZE_7)
#if defined(TP_GOODIX_EM7127_BM_81070027023_01)//em7127
	uint8_t config_info[] = {
	0x65,0x00,0x03,0x20,
	0x01,0xE0,0x0A,0xAD,
	0x20,0x00,0x32,0x0F,0x82,0x08,0x14,0x00,
	0x00,0x1D,0x00,0x00,0x88,0x88,0x88,0x00,0x37,0x00,0x00,0x00,0x01,0x02,0x03,0x04,
	0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0xFF,0xFF,0x00,0x01,0x02,0x03,0x04,
	0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0xFF,0xFF,0xFF,0x00,0x00,0x3C,0x64,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	};

#elif defined(TP_GOODIX_EM73_HOTA_C113171A1)//em73
	uint8_t config_info[] = {		
	//0x65,0x02,0x04,0x00,
	0x65,0x02,0x03,0x20,
	//0x02,0x58,0x05,0x85,
	0x01,0xE0,0x05,0x85,
	0x01,0x00,0x00,0x0C,0x02,0x08,0x10,0x00,
	0x00,0x1A,0x00,0x00,0x48,0x48,0x48,0x33,0x37,0x00,0x00,0x0A,0x09,0x08,0x07,0x06,
	0x05,0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x0A,0x09,0x08,0x07,0x06,
	0x05,0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x3C,0x64,0x50,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20
	};

//#elif defined(TP_GOODIX_SIZE_8)//EM86
#elif defined(TP_GOODIX_EM86_DPT_N3371A)//EM86
	/*
	uint8_t config_info[] = {
	//0x65,0x00,0x25,0x80,
	0x65,0x00,0x03,0x20,
	//0x19,0x00,0x05,0x80,
	0x02,0x58,0x05,0x80,
	0x00,0x00,0x00,0x10,0x02,0x08,0x10,0x00,
	0x00,0x1A,0x00,0x00,0x88,0x88,0x88,0x00,0x37,0x00,0x00,0x00,0x01,0x02,0x03,0x04,
	0x05,0x06,0x07,0x08,0x09,0x0A,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x01,0x02,0x03,0x04,
	0x05,0x06,0x07,0x08,0x09,0x0A,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x3C,0x64,0x50,
	0x2D,0x64,0x9C,0xD2,0x00,0x00,0x00,0x00,0x12,0x16,0x3C,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20
	};
	*/

	//0902
	/*uint8_t config_info[] = {
	0x65,0x00,0x03,0x20,
	0x02,0x58,0x05,0xA0,
	0x01,0x00,0x00,0x10,0x02,0x10,0x10,0x00,
	0x00,0x20,0x00,0x00,0x88,0x10,0x10,0x22,0x37,0x00,0x00,0x00,0x01,0x02,0x03,0x04,
	0x05,0x06,0x07,0x08,0x09,0x0A,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x01,0x02,0x03,0x04,
	0x05,0x06,0x07,0x08,0x09,0x0A,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x3C,0x64,0x50,
	0x2D,0x64,0x9C,0xD2,0x00,0x00,0x00,0x00,0x12,0x16,0x3C,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20
	};*/
	
	//HD, low resist.
	/*uint8_t config_info[] = {
	0x65,0x00,0x03,0x20,
	0x02,0x58,0x05,0xA0,
	0x00,0x00,0x00,0x10,0x02,0x08,0x10,0x00,
	0x00,0x2A,0x00,0x00,0x88,0x88,0x88,0x34,0x37,0x00,0x00,0x00,0x01,0x02,0x03,0x04,
	0x05,0x06,0x07,0x08,0x09,0x0A,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x01,0x02,0x03,0x04,
	0x05,0x06,0x07,0x08,0x09,0x0A,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x3C,0x64,0x50,
	0x2D,0x64,0x9C,0xD2,0x00,0x00,0x00,0x00,0x12,0x16,0x3C,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20
	};*/
	
	//ND, low resist
	uint8_t config_info[] = {
	0x65,0x00,0x03,0x20,
	0x02,0x58,0x05,0xA1,
	0x00,0x00,0x0F,0x10,0x02,0x10,0x10,0x00,
	0x00,0x2C,0x00,0x00,0x88,0x10,0x10,0x11,0x37,0x00,0x00,0x00,0x01,0x02,0x03,0x04,
	0x05,0x06,0x07,0x08,0x09,0x0A,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x01,0x02,0x03,0x04,
	0x05,0x06,0x07,0x08,0x09,0x0A,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x3C,0x64,0x64,
	0x2D,0x64,0x9C,0xD2,0x00,0x00,0x00,0x00,0x12,0x19,0x2A,0x08,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20
	};

#elif defined(TP_GOODIX_EM8127_BM_81080028A23_00)//em8127
	/*
	uint8_t config_info[] = {
	0x65,0x00,0x04,0x00,
	0x02,0x58,0x0A,0x61,
	0x00,0x00,0x00,0x26,0x02,0x08,0x10,0x00,
	0x00,0x17,0x00,0x00,0x80,0x80,0x80,0x21,0x13,0x00,0x00,0x0A,0x09,0x08,0x07,0x06,
	0x05,0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x0A,0x09,0x08,0x07,0x06,
	0x05,0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x32,0x3C,0x50,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20
	};
	*/
	
	//0902,
	uint8_t config_info[] = {
	0x65,0x01,0x03,0x20,
	0x02,0x58,0x05,0xA1,
	0x01,0x00,0x0F,0x13,0x02,0x08,0x10,0x00,
	//0x01,0x00,0x00,0x26,0x05,0x10,0x10,0x00,//test
	0x00,0x30,0x00,0x00,0x88,0x10,0x10,0x00,0x3B,0x00,0x00,0x0C,0x0B,0x0A,0x09,0x08,
	0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0xFF,0x0C,0x0B,0x0A,0x09,0x08,
	0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0xFF,0x00,0x00,0x3C,0x64,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20
	};
#elif defined(TP_GOODIX_EM1125_BM_81100026023_02)
	
	//0907,improve sensitivity of some tp
	uint8_t config_info[] = {
    0x65,0x00,0x04,0x00,
    0x02,0x58,0x08,0x60,//8 fingers
    //0x02,0x58,0x0A,0x60,//10 fingers,with problem
    0x01,0x00,0x0F,0x26,0x02,0x10,0x10,0x00,
    0x00,0x26,0x00,0x00,0x10,0x10,0x10,0x12,0x37,0x00,0x00,0x0A,0x09,0x08,0x07,0x06,
    0x05,0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x0A,0x09,0x08,0x07,0x06,
    0x05,0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x46,0x78,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20
    };
#elif defined(TP_GOODIX_EM02_QTW_101015_01A_V1)//em102,1102
	uint8_t config_info[] = {
	//0x65,0x02,0x25,0x80,
	0x65,0x02,0x04,0x00,
	//0x19,0x00,0x05,0x6D,
	0x02,0x58,0x05,0x6D,
	0x21,0x00,0x0F,0x28,0x02,0x08,0x14,0x00,
	0x00,0x2C,0x00,0x00,0x88,0x10,0x10,0x00,0x37,0x00,0x00,0x0C,0x0B,0x0A,0x09,0x08,
	0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0xFF,0x0B,0x0A,0x09,0x08,0x07,
	0x06,0x05,0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x3C,0x5A,0x50,
	0xCC,0x8C,0x5C,0x1E,0x00,0x00,0x00,0x00,0x11,0x19,0x28,0x1B,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20
	};
#elif defined(TP_GOODIX_EM101_300_L3512A_A00)
/*	uint8_t config_info[] = {//shuitian--huiting
	0x65,0x03,0x04,0x00,
	0x03,0x00,0x05,0x6D,
	0x01,0x00,0x0F,0x28,0x02,0x08,0x10,0x00,
	0x00,0x2C,0x00,0x00,0x68,0x10,0x10,0x33,0x37,0x00,0x00,0x0D,0x0C,0x0B,0x0A,0x09,
	0x08,0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0x0D,0x0C,0x0B,0x0A,0x09,
	0x08,0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0x00,0x00,0x3C,0x5A,0x50,
	0x20,0x60,0x9F,0xDE,0x00,0x00,0x00,0x00,0x01,0x1C,0x1A,0x2A,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20
	};*/
	uint8_t config_info[] ={
	0x65,0x03,0x04,0x00,
	0x03,0x00,0x05,0x6D,
	0x01,0x00,0x0F,0x28,0x02,0x08,0x10,0x00,
	0x00,0x2B,0x00,0x00,0x68,0x10,0x10,0x33,0x37,0x00,0x00,0x0D,0x0C,0x0B,0x0A,0x09,0x08,0x07,
	0x06,0x05,0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0x0D,0x0C,0x0B,0x0A,0x09,0x08,0x07,0x06,0x05,
	0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0x00,0x00,0x28,0x59,0x50,0x20,0x60,0x9F,0xDE,0x00,0x00,
	0x00,0x00,0x01,0x19,0x2D,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x1B,0x01,0x02,0x02,0x01,0x01,
	0x02,0x02,0x00,0x00,0x00,0x00,0x20
	};	
#elif defined(TP_GOODIX_EM103_GT8105) 
	uint8_t config_info[] ={
	0x65,0x00,0x10,0x00,
	0x10,0x00,0x05,0x21,
	0x01,0x00,0x0F,0x28,0x02,0x10,0x10,0x00,
	0x00,0x1B,0x00,0x00,0x28,0x10,0x10,0x00,0x33,0x00,0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x06,
	0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0xFF,0xFF,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,
	0x09,0x0A,0x0B,0x0C,0x0D,0xFF,0xFF,0x00,0x00,0x46,0x6E,0x64,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x20
	};
#elif defined(TP_GOODIX_9760_GT8110)
	uint8_t config_info[] ={	
	0x65,0x02,0x04,0x00,
	0x03,0x00,0x0A,0x61,
	0x01,0x00,0x0F,0x20,0x02,0x10,0x10,0x00,
	0x00,0x20,0x00,0x00,0x88,0x10,0x10,0x11,0x37,0x00,0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x06,
	0x07,0x08,0x09,0x0A,0x0B,0xFF,0xFF,0xFF,0xFF,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,
	0x09,0x0A,0x0B,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x32,0x5A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x20
	};
#else
	uint8_t config_info[] = {};
#endif


	//WAKEUP GREEN MODE
	//disable_irq(client->irq);

	/*
	gpio_direction_output(INT_PORT, 0);
	msleep(5);
	s3c_gpio_cfgpin(INT_PORT, INT_CFG);
	*/
	goodix_irq_reset();
	//enable_irq(client->irq);

	ret=i2c_write_bytes(ts->client,config_info, (sizeof(config_info)/sizeof(config_info[0])));
	if (ret < 0)
	{
		printk("************** goodix write config info err!%s\n",__func__);
		return ret;
	}
		
#endif
	ret=i2c_read_bytes(ts->client, rd_cfg_buf, 7);
	//if(ret != 2)
	if(ret <= 0)
	{
		dev_info(&ts->client->dev, "Read resolution & max_touch_num failed, use default value!,ret = %d\n",ret);
		ts->abs_x_max = TOUCH_MAX_HEIGHT;
		ts->abs_y_max = TOUCH_MAX_WIDTH;
		ts->max_touch_num = MAX_FINGER_NUM;
		ts->int_trigger_type = INT_TRIGGER;
		return 0;
	}
	ts->abs_x_max = (rd_cfg_buf[1]<<8) + rd_cfg_buf[2];
	ts->abs_y_max = (rd_cfg_buf[3]<<8) + rd_cfg_buf[4];
	ts->max_touch_num = rd_cfg_buf[5];
	ts->int_trigger_type = rd_cfg_buf[6]&0x03;
	if((!ts->abs_x_max)||(!ts->abs_y_max)||(!ts->max_touch_num))
	{
		dev_info(&ts->client->dev, "Read invalid resolution & max_touch_num, use default value!\n");
		ts->abs_x_max = TOUCH_MAX_HEIGHT;
		ts->abs_y_max = TOUCH_MAX_WIDTH;
		ts->max_touch_num = MAX_FINGER_NUM;
	}

	dev_info(&ts->client->dev,"X_MAX = %d,Y_MAX = %d,MAX_TOUCH_NUM = %d\n",ts->abs_x_max,ts->abs_y_max,ts->max_touch_num);
	//wake up mode from green mode
	rd_cfg_buf[0] = 0x6e;
	rd_cfg_buf[1] = 0x00;
	i2c_read_bytes(ts->client, rd_cfg_buf, 2);
	if((rd_cfg_buf[1]&0x0f)==0x0f)
	{
		dev_info(&ts->client->dev, "Touchscreen works in INT wake up green mode!\n");
		ts->green_wake_mode = 1;
	}
	else
	{
		dev_info(&ts->client->dev, "Touchscreen works in IIC wake up green mode!\n");
		ts->green_wake_mode = 0;
	}

	msleep(10);
	return 0;

}

/*******************************************************
Description:
	Read goodix touchscreen version function.

Parameter:
	ts:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
static int  goodix_read_version(struct goodix_ts_data *ts, char **version)
{
	int ret = -1, count = 0;
	char *version_data;
	char *p;
	
	*version = (char *)vmalloc(18);
	version_data = *version;
	if(!version_data)
		return -ENOMEM;
	p = version_data;
	memset(version_data, 0, sizeof(version_data));
	version_data[0]=240;
	if(ts->green_wake_mode)			//WAKEUP GREEN MODE
	{
		disable_irq(ts->client->irq);

		/*
		gpio_direction_output(INT_PORT, 0);
		msleep(5);
		s3c_gpio_cfgpin(INT_PORT, INT_CFG);
		*/
		goodix_irq_reset();
		enable_irq(ts->client->irq);
	}
	ret=i2c_read_bytes(ts->client,version_data, 17);
	if (ret < 0) 
		return ret;
	version_data[17]='\0';
	
	if(*p == '\0')
		return 0; 	
	do 					
	{
		if((*p > 122) || (*p < 48 && *p != 32) || (*p >57 && *p  < 65) 
			||(*p > 90 && *p < 97 && *p  != '_'))		//check illeqal character
			count++;
	}while(*++p != '\0' );
	if(count > 2)
		return 0;
	else 
		return 1;	
}


/*******************************************************
Description:
	Goodix touchscreen work function.

Parameter:
	ts:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{	
	int ret=-1;
	int tmp = 0;
	uint8_t  point_data[(1-READ_COOR_ADDR)+1+2+5*MAX_FINGER_NUM+1]={ 0 };  //read address(1byte)+key index(1byte)+point mask(2bytes)+5bytes*MAX_FINGER_NUM+coor checksum(1byte)
	uint8_t  check_sum = 0;
	uint16_t  finger_current = 0;
	uint16_t  finger_bit = 0;
	unsigned int  count = 0, point_count = 0;
	unsigned int position = 0;	
	uint8_t track_id[MAX_FINGER_NUM] = {0};
	unsigned int input_x = 0;
	unsigned int input_y = 0;
	unsigned int input_w = 0;
	unsigned char index = 0;
	unsigned char touch_num = 0;
	unsigned int swap = 0;
	
	struct goodix_ts_data *ts = container_of(work, struct goodix_ts_data, work);


	if(g_enter_isp)return;
#if defined(INT_PORT)
COORDINATE_POLL:
	if((ts->int_trigger_type> 1)&& (gpio_in_get(INT_PORT) != (ts->int_trigger_type&0x01)))
	{
		goto NO_ACTION;
	}	
#endif	


	if( tmp > 9) {
		
		dev_info(&(ts->client->dev), "I2C transfer error,touchscreen stop working.\n");
		goto XFER_ERROR ;
	}
	
	if(ts->bad_data)	
		msleep(20);
	
	point_data[0] = READ_COOR_ADDR;		//read coor address
	ret=i2c_read_bytes(ts->client, point_data,  sizeof(point_data)/sizeof(point_data[0]));
	if(ret <= 0)	
	{
		dev_err(&(ts->client->dev),"I2C transfer error. Number:%d\n ", ret);
		ts->bad_data = 1;
		tmp ++;
		ts->retry++;
	#if defined(INT_PORT)
		if(ts->int_trigger_type> 1)
			goto COORDINATE_POLL;
		else
			goto XFER_ERROR;
	#elif defined(TP_GOODIX_USE_IRQ)//TODO
		goto XFER_ERROR;
	#endif
	}	
	ts->bad_data = 0; 
	finger_current =  (point_data[3 - READ_COOR_ADDR]<<8) + point_data[2 - READ_COOR_ADDR];
	
	if(finger_current)
	{	
		point_count = 0, finger_bit = finger_current;
		for(count = 0; (finger_bit != 0) && (count < ts->max_touch_num); count++)//cal how many point touch currntly
		{
			if(finger_bit & 0x01)
			{
				track_id[point_count] = count;
				point_count++;
			}
			finger_bit >>= 1;
		}
		touch_num = point_count;

		check_sum = point_data[2 - READ_COOR_ADDR] + point_data[3 - READ_COOR_ADDR]; 			//cal coor checksum
		count = 4 - READ_COOR_ADDR;
		for(point_count *= 5; point_count > 0; point_count--)
			check_sum += point_data[count++];
		check_sum += point_data[count];
		if(check_sum  != 0)			//checksum verify error
		{
			printk("coor checksum error!\n");
		#if defined(INT_PORT)
			if(ts->int_trigger_type> 1)
				goto COORDINATE_POLL;
			else	
				goto XFER_ERROR;
		#elif defined(TP_GOODIX_USE_IRQ)//TODO
			goto XFER_ERROR;
		#endif
		}
	}

	

	if(touch_num)
	{
		//printk("max num:%d, num:%d\n", touch_num, ts->max_touch_num);
		for(index=0; index<touch_num; index++)
		{
			position = 4 - READ_COOR_ADDR + 5*index;
			input_x = (unsigned int) (point_data[position]<<8) + (unsigned int)( point_data[position+1]);
			input_y = (unsigned int)(point_data[position+2]<<8) + (unsigned int) (point_data[position+3]);
			input_w =(unsigned int) (point_data[position+4]);		

			#if defined(TP_GOODIX_REVERSE_X)
			input_x = TOUCH_MAX_HEIGHT - input_x;
			#endif
			#if defined(TP_GOODIX_REVERSE_Y)
			input_y = TOUCH_MAX_WIDTH- input_y;
			#endif
//			printk("input_x:%d, input_y:%d\n", input_x, input_y);
#ifdef SWAP_XY			
			swap = input_y;
			input_y = input_x;
			input_x = swap;
#endif
//			printk("\n swap :input_x:%d, input_y:%d\n", input_x, input_y);
			//modify by emdoor linshixiong  20120426 to fix the bug when touch the top
			#if defined(TP_GOODIX_EM86_DPT_N3371A)
			if(input_y < 11)
			{
				input_y = 11;
			}
			#endif
			//printk("input_x=%d, input_y=%d; **** ts->abs_x_max=%d,ts->abs_y_max=%d\n ", input_x, input_y,  ts->abs_x_max, ts->abs_y_max);
			if((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))continue;
                        input_report_key(ts->input_dev, BTN_TOUCH, 1);
                        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
                        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
                        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
                        input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
                        //input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 1);
                        input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, track_id[index]);
                        input_mt_sync(ts->input_dev);
		}
	}
	else
	{
            input_report_key(ts->input_dev, BTN_TOUCH, 0);
            input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
            input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
            //input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
            input_mt_sync(ts->input_dev);
	}

	#ifdef HAVE_TOUCH_KEY
	//printk(KERN_INFO"#####HAVE KEY DOWN!0x%x\n",point_data[1]);
	for(count = 0; count < MAX_KEY_NUM; count++)
	{
		input_report_key(ts->input_dev, touch_key_array[count], !!(point_data[1]&(0x01<<count)));	

		#if defined(TOUCH_WITH_LIGHT_KEY)
		if(!!(point_data[1]&(0x01<<count)))
			g_pen_down = 1;
		#endif
	}

	#if defined(TOUCH_WITH_LIGHT_KEY)
	//printk("***** g_pen_down = %d\n",g_pen_down);
	if(g_pen_down == 1 && g_light_on ==0)
	{
		gpio_ioctl(1);
		g_light_on = 1;
		//printk("***** gpio_ioctl(1)\n");
	}
	else if(g_pen_down == 0 && g_light_on ==1)
	{
		gpio_ioctl(0);
		g_light_on = 0;
		//printk("***** gpio_ioctl(0)\n");
	}
	g_pen_down = 0;
	#endif
	
	#endif
	//#if defined(TOUCH_WITH_LIGHT_KEY)
	//g_onoff_light_key = 0;
	//#endif	
	input_sync(ts->input_dev);

#if defined(INT_PORT)// || defined(TP_GOODIX_USE_IRQ)
	if(ts->int_trigger_type> 1)
	{
		msleep(POLL_TIME);
		goto COORDINATE_POLL;
	}
#endif
	goto END_WORK_FUNC;

NO_ACTION:	

#ifdef HAVE_TOUCH_KEY
	//printk(KERN_INFO"=====HAVE KEY DOWN!0x%x\n",point_data[1]);
	for(count = 0; count < MAX_KEY_NUM; count++)
	{
		input_report_key(ts->input_dev, touch_key_array[count], !!(point_data[1]&(0x01<<count)));	
	}
	input_sync(ts->input_dev);	   
#endif
END_WORK_FUNC:
XFER_ERROR:
	if(ts->use_irq)
		enable_irq(ts->client->irq);

}

/*******************************************************
Description:
	Timer interrupt service routine.

Parameter:
	timer:	timer struct pointer.
	
return:
	Timer work mode. HRTIMER_NORESTART---not restart mode
*******************************************************/
static enum hrtimer_restart goodix_ts_timer_func(struct hrtimer *timer)
{
	struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);
	printk("[Tomy] enter %s\n", __func__);
	queue_work(goodix_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, (POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

/*******************************************************
Description:
	External interrupt service routine.

Parameter:
	irq:	interrupt number.
	dev_id: private data pointer.
	
return:
	irq execute status.
*******************************************************/

static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	static int boot_count = 0;
	//printk("************** %s\n",__func__);
	struct goodix_ts_data *ts = dev_id;
	//Add by tomy.huang, check power of TP before queue work, as power of TP is closed in LCD earlysuspend.
	#if defined(POWER_OFF_LCD_3V3_IN_EARLY_SUSPEND)
	if(!gpio_in_get(GPIO_GOODIX_IRQ){
		printk("******** power of TP is down, return\n");
		return IRQ_HANDLED;
	}
	#endif
	if(boot_count < 5)
	{
		boot_count++;
		printk("ingor this irq %d\n", boot_count);
	}else{
		disable_irq_nosync(ts->client->irq);
		queue_work(goodix_wq, &ts->work);
	}
	
	return IRQ_HANDLED;
}

/*******************************************************
Description:
	Goodix touchscreen power manage function.

Parameter:
	on:	power status.0---suspend;1---resume.
	
return:
	Executive outcomes.-1---i2c transfer error;0---succeed.
*******************************************************/
static int goodix_ts_power(struct goodix_ts_data * ts, int on)
{
	int ret = -1;
	unsigned char i2c_control_buf[2] = {80,  1};		//suspend cmd
	int retry = 0;

	//printk("************** %s\n",__func__);
	
	if(on != 0 && on !=1)
	{
		printk(KERN_DEBUG "%s: Cant't support this command.", goodix_ts_name);
		return -EINVAL;
	}
	
	if(ts != NULL && !ts->use_irq)
		return -2;
	
	if(on == 0)		//suspend
	{ 
		if(ts->green_wake_mode)
		{
		/*
			disable_irq(ts->client->irq);
			gpio_direction_output(INT_PORT, 0);
			msleep(5);
			s3c_gpio_cfgpin(INT_PORT, INT_CFG);
			enable_irq(ts->client->irq);
		*/
		disable_irq(ts->client->irq);
		goodix_irq_reset();
		enable_irq(ts->client->irq);
		}
		while(retry<5)
		{
			ret = i2c_write_bytes(ts->client, i2c_control_buf, 2);
			if(ret == 1)
			{
				printk(KERN_INFO"Send suspend cmd\n");
				break;
			}
			printk("Send cmd failed!\n");
			retry++;
			msleep(10);
		}
		if(ret > 0)
			ret = 0;
	}
	else if(on == 1)		//resume
	{
		printk(KERN_INFO"Int resume\n");
		
		/*
		gpio_direction_output(INT_PORT, 0); 
		msleep(20);
		if(ts->use_irq) 
			s3c_gpio_cfgpin(INT_PORT, INT_CFG);	//Set IO port as interrupt port	
		else 
			gpio_direction_input(INT_PORT);
		//msleep(260);
		*/
		goodix_irq_reset();

		ret = 0;
	}	 
	return ret;
}

/*******************************************************
Description:
	Goodix debug sysfs cat version function.

Parameter:
	standard sysfs show param.
	
return:
	Executive outcomes. 0---failed.
*******************************************************/
static ssize_t goodix_debug_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	char *version_info = NULL;
	struct goodix_ts_data *ts;
	
	ts = i2c_get_clientdata(i2c_connect_client);
	if(ts==NULL)
		return 0;
	
	ret = goodix_read_version(ts, &version_info);
	if(ret <= 0)
	{
		printk(KERN_INFO"Read version data failed!\n");
		vfree(version_info);
		return 0;
	}

	printk(KERN_INFO"Goodix TouchScreen Version:%s\n", (version_info+1));
	sprintf(buf,"Goodix TouchScreen Version:%s\n",(version_info+1));
	vfree(version_info);
	ret = strlen(buf);
	return ret;
}

/*******************************************************
Description:
	Goodix debug sysfs cat resolution function.

Parameter:
	standard sysfs show param.
	
return:
	Executive outcomes. 0---failed.
*******************************************************/
static ssize_t goodix_debug_resolution_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts;
	ts = i2c_get_clientdata(i2c_connect_client);
	dev_info(&ts->client->dev,"ABS_X_MAX = %d,ABS_Y_MAX = %d\n",ts->abs_x_max,ts->abs_y_max);
	sprintf(buf,"ABS_X_MAX = %d,ABS_Y_MAX = %d\n",ts->abs_x_max,ts->abs_y_max);

	return strlen(buf);
}
/*******************************************************
Description:
	Goodix debug sysfs cat version function.

Parameter:
	standard sysfs show param.
	
return:
	Executive outcomes. 0---failed.
*******************************************************/
static ssize_t goodix_debug_diffdata_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	//char diff_data[300];
	unsigned char diff_data[2241] = {00,};
	int ret = -1;
	char diff_data_cmd[2] = {80, 202};
	int i;
	int short_tmp;
	struct goodix_ts_data *ts;

	disable_irq(TS_INT);
	
	ts = i2c_get_clientdata(i2c_connect_client);
	//memset(diff_data, 0, sizeof(diff_data));
	if(ts->green_wake_mode)
	{
		//disable_irq(client->irq);

		/*
		gpio_direction_output(INT_PORT, 0);
		msleep(5);
		s3c_gpio_cfgpin(INT_PORT, INT_CFG);
		*/
		goodix_irq_reset();

	}
	ret = i2c_write_bytes(ts->client, diff_data_cmd, 2);
	if(ret != 1)
	{
		dev_info(&ts->client->dev, "Write diff data cmd failed!\n");
		enable_irq(TS_INT);
		return 0;
	}

	while(1)
	{
		if((gpio_in_get(GPIO_GOODIX_IRQ))==0)
			break;
	}
	
	ret = i2c_read_bytes(ts->client, diff_data, sizeof(diff_data));
	//if(ret != 2)
	if(ret <= 0)
	{
		dev_info(&ts->client->dev, "Read diff data failed!\n");
		enable_irq(TS_INT);
		return 0;
	}
	for(i=1; i<sizeof(diff_data); i+=2)
	{
		short_tmp = diff_data[i] + (diff_data[i+1]<<8);
		if(short_tmp&0x8000)
			short_tmp -= 65535;
		if(short_tmp == 512)continue;
		sprintf(buf+strlen(buf)," %d",short_tmp);
		//printk(" %d\n", short_tmp);
	}
	
	diff_data_cmd[1] = 0;
	ret = i2c_write_bytes(ts->client, diff_data_cmd, 2);
	if(ret != 1)
	{
		dev_info(&ts->client->dev, "Write diff data cmd failed!\n");
		enable_irq(TS_INT);
		return 0;
	}
	enable_irq(TS_INT);
	/*for (i=0; i<1024; i++)
	{
 		sprintf(buf+strlen(buf)," %d",i);
	}*/
	
	return strlen(buf);
}


/*******************************************************
Description:
	Goodix debug sysfs echo calibration function.

Parameter:
	standard sysfs store param.
	
return:
	Executive outcomes..
*******************************************************/
static ssize_t goodix_debug_calibration_store(struct device *dev,
			struct device_attribute *attr, const char *buf, ssize_t count)
{
	int ret = -1;
	char cal_cmd_buf[] = {110,1};
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(i2c_connect_client);
	dev_info(&ts->client->dev,"Begin calibration......\n");
	if((*buf == 10)||(*buf == 49))
	{
		if(ts->green_wake_mode)
		{
			disable_irq(ts->client->irq);

			/*
			gpio_direction_output(INT_PORT, 0);
			msleep(5);
			s3c_gpio_cfgpin(INT_PORT, INT_CFG);
			*/
			goodix_irq_reset();		
			enable_irq(ts->client->irq);
		}
		ret = i2c_write_bytes(ts->client,cal_cmd_buf,2);
		if(ret!=1)
		{
			dev_info(&ts->client->dev,"Calibration failed!\n");
			return count;
		}
		else
		{
			dev_info(&ts->client->dev,"Calibration succeed!\n");
		}
	}
	return count;
}


static DEVICE_ATTR(version, S_IRUGO, goodix_debug_version_show, NULL);
static DEVICE_ATTR(resolution, S_IRUGO, goodix_debug_resolution_show, NULL);
static DEVICE_ATTR(diffdata, S_IRUGO, goodix_debug_diffdata_show, NULL);
static DEVICE_ATTR(calibration, S_IWUSR , NULL, goodix_debug_calibration_store); 


/*******************************************************
Description:
	Goodix debug sysfs init function.

Parameter:
	none.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_debug_sysfs_init(void)
{
	int ret ;
	struct goodix_ts_data *ts;
	ts = i2c_get_clientdata(i2c_connect_client);

	goodix_debug_kobj = kobject_create_and_add("goodix_debug", NULL) ;
	if (goodix_debug_kobj == NULL) {
		printk(KERN_ERR "%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_version.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_calibration.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_calibration_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_diffdata.attr);
	if (ret) 
	{
		printk(KERN_ERR "%s: sysfs_create_diffdata_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_resolution.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_resolution_file failed\n", __func__);
		return ret;
	}
	dev_info(&ts->client->dev,"Goodix debug sysfs create success!\n");
	return 0 ;
}

static void goodix_debug_sysfs_deinit(void)
{
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_version.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_resolution.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_diffdata.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_calibration.attr);
	kobject_del(goodix_debug_kobj);
}


#if defined(TP_GOODIX_USE_IRQ)
static void initial_irq(void)
{
	printk(KERN_INFO "%s\n",__func__);
	gpio_set_status(GPIO_GOODIX_IRQ, gpio_status_in);
    gpio_irq_set(170, GPIO_IRQ(INT_GPIO_0, GPIO_IRQ_FALLING));
}
#endif

/*******************************************************
Description:
	Goodix touchscreen probe function.

Parameter:
	client:	i2c device struct.
	id:device id.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static void goodix_pwr_ctl(int state)
{
	if(state){
		gpio_out(GPIO_GOODIX_PWR,1);//Power on TP
		gpio_out(GPIO_GOODIX_RST,0);//Reset TP
		msleep(10);
		gpio_out(GPIO_GOODIX_RST,1);
		msleep(20);
	}else{
		gpio_out(GPIO_GOODIX_PWR,0);//Power off TP
	}
}
static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int retry=0;
	struct goodix_ts_data *ts;
	char *version_info = NULL;
	char test_data = 1;
	const char irq_table[4] = {IRQ_TYPE_EDGE_RISING,
							   IRQ_TYPE_EDGE_FALLING,
							   IRQ_TYPE_LEVEL_LOW,
							   IRQ_TYPE_LEVEL_HIGH};

	struct goodix_i2c_rmi_platform_data *pdata;
	dev_dbg(&client->dev,"Install touch driver.\n");

	goodix_pwr_ctl(1);
	#ifdef TOUCH_WITH_LIGHT_KEY
	gpio_ioctl(0);
	#endif
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		dev_err(&client->dev, "Must have I2C_FUNC_I2C.\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

#ifdef INT_PORT	
	client->irq=TS_INT;
	if (client->irq)
	{
		ret = gpio_request(INT_PORT, "TS_INT");
		if (ret < 0) 
		{
			dev_err(&client->dev, "Failed to request GPIO:%d, ERRNO:%d\n",(int)INT_PORT,ret);
			client->irq = 0;
			goto err_gpio_request;
		}
	}
#endif		

err_gpio_request:
    i2c_connect_client = client;
	for(retry=0;retry < 5; retry++)
	{
		//disable_irq(client->irq);
		/*
		gpio_direction_output(INT_PORT, 0);
		msleep(5);
		s3c_gpio_cfgpin(INT_PORT, INT_CFG);
		*/
		goodix_irq_reset();
		//enable_irq(client->irq);
		ret =i2c_write_bytes(client, &test_data, 1);
		if (ret > 0)
			break;
		printk("GOODiX i2c test failed!\n");
	}
	if(ret <= 0)
	{
		dev_err(&client->dev, "I2C communication ERROR!Goodix touchscreen driver become invalid\n");
		goto err_i2c_failed;
	}	
	
	INIT_WORK(&ts->work, goodix_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	
	for(retry=0; retry<3; retry++)
	{
		ret=goodix_init_panel(ts);
		msleep(2);
		if(ret != 0)
			continue;
		else
			break;
	}
	if(ret != 0) {
		ts->bad_data=1;
		goto err_init_godix_ts;
	}

//if there is no tp connected on mid,and we also load the goodix tp driver,system will die.
//TODO:i2c_write_bytes should return a correct value.simon,2012-02-22
#define GOODIX_TP_VERIFY_FW_VERSION		1
#if defined(GOODIX_TP_VERIFY_FW_VERSION)
	for(retry = 0;retry < 5;retry++)
	{
		ret = goodix_read_version(ts, &version_info);
		if(ret <= 0)
			printk(KERN_INFO"\n****** ERROR,goodix tp read version failed, \
				retry time = %d,%s\n",retry,__func__);
		else
			break;
		msleep(50);
	}
	if(ret <= 0)
	{
		vfree(version_info);
		printk(KERN_INFO"***** Goodix tp Read version data failed!,unregister it now!\n");
		goto err_input_dev_alloc_failed;
	}
	else
	{
		printk(KERN_INFO"***** Goodix TouchScreen Version:%s\n", (version_info+1));
	}
	vfree(version_info);
#endif
	
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_dbg(&client->dev,"Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE); 						// absolute coor (x,y)
#ifdef HAVE_TOUCH_KEY
	for(retry = 0; retry < MAX_KEY_NUM; retry++)
	{
		input_set_capability(ts->input_dev,EV_KEY,touch_key_array[retry]);	
	}
#endif

	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
//	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);

#ifdef GOODIX_MULTI_TOUCH
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 0, 0, 0);	
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif	

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = goodix_ts_name;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;	//screen firmware version
	
	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev,"Probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	ts->bad_data = 0;
	
	#ifdef TP_GOODIX_USE_IRQ
	initial_irq();
	client->irq = TS_INT;
	ret  = request_irq(TS_INT, goodix_ts_irq_handler ,IRQF_DISABLED, client->name, ts);
	if (ret != 0) {
		printk("\n\nrequest_irq failed,ERRNO:%d,%s\n\n",ret,__func__);
		goto works_in_polling_mode;
	}
	else 
	{	
		disable_irq(TS_INT);
		ts->use_irq = 1;
		dev_dbg(&client->dev,"Reques EIRQ %d succesd\n",TS_INT);
	}
	#endif
	
works_in_polling_mode:		
	if (!ts->use_irq) 
	{
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
	
	if(ts->use_irq)
		enable_irq(client->irq);
#if defined(INT_PORT)
	if(ts->use_irq) 		
		ts->power = goodix_ts_power;
//#endif	
#elif defined(TP_GOODIX_USE_PW_CTRL)
		ts->power = goodix_ts_power;
#endif	

#if !defined(GOODIX_TP_VERIFY_FW_VERSION)
	ret = goodix_read_version(ts, &version_info);
	if(ret <= 0)
	{
		printk(KERN_INFO"Read version data failed!\n");
	}
	else
	{
		printk(KERN_INFO"Goodix TouchScreen Version:%s\n", (version_info+1));
	}
	vfree(version_info);
#endif
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#ifdef CONFIG_TOUCHSCREEN_GOODIX_IAP
	goodix_proc_entry = create_proc_entry("goodix-update", 0666, NULL);
	if(goodix_proc_entry == NULL)
	{
		dev_info(&client->dev, "Couldn't create proc entry!\n");
		ret = -ENOMEM;
		goto err_create_proc_entry;
	}
	else
	{
		dev_info(&client->dev, "Create proc entry success!\n");
		goodix_proc_entry->write_proc = goodix_update_write;
		goodix_proc_entry->read_proc = goodix_update_read;
		//goodix_proc_entry->owner =THIS_MODULE;
	}
#endif
	goodix_debug_sysfs_init();
	dev_info(&client->dev,"Start %s in %s mode\n", 
		ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	dev_info(&client->dev, "Driver Modify Date:2011-07-04\n");
	return 0;

err_init_godix_ts:
	if(ts->use_irq)
	{
		ts->use_irq = 0;
		free_irq(client->irq,ts);
	#ifdef INT_PORT	
		gpio_direction_input(INT_PORT);
		gpio_free(INT_PORT);
	#endif	
	}
	else 
		hrtimer_cancel(&ts->timer);

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	i2c_set_clientdata(client, NULL);
err_i2c_failed:	
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
err_create_proc_entry:
	return ret;
}


/*******************************************************
Description:
	Goodix touchscreen driver release function.

Parameter:
	client:	i2c device struct.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
#ifdef CONFIG_TOUCHSCREEN_GOODIX_IAP
	remove_proc_entry("goodix-update", NULL);
#endif
	goodix_debug_sysfs_deinit();
	if (ts && ts->use_irq) 
	{
	#ifdef INT_PORT
		gpio_direction_input(INT_PORT);
		gpio_free(INT_PORT);
	#endif	
		free_irq(client->irq, ts);
	}	
	else if(ts)
		hrtimer_cancel(&ts->timer);
	
	dev_notice(&client->dev,"The driver is removing...\n");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int goodix_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	
	if (ts->power) {
		ret = ts->power(ts, 0);
		if (ret < 0)
			printk(KERN_ERR "goodix_ts_resume power off failed\n");
	}

	//goodix_pwr_ctl(0);//power off TP
#if defined(TOUCH_WITH_LIGHT_KEY)
	gpio_ioctl(0);
#endif

	return 0;
}

static int goodix_ts_resume(struct i2c_client *client)
{
	int ret;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	//goodix_pwr_ctl(1);//power on TP
	if (ts->power) {
		ret = ts->power(ts, 1);
		if (ret < 0)
			printk(KERN_ERR "goodix_ts_resume power on failed\n");
	}

	if (ts->use_irq)
		enable_irq(client->irq);
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

#if defined(TOUCH_WITH_LIGHT_KEY)
	//gpio_ioctl(5);//5 secs
#endif

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	ts = container_of(h, struct goodix_ts_data, early_suspend);
	goodix_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void goodix_ts_late_resume(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	ts = container_of(h, struct goodix_ts_data, early_suspend);
	goodix_ts_resume(ts->client);
}
#endif

//******************************Begin of firmware update surpport*******************************
#ifdef CONFIG_TOUCHSCREEN_GOODIX_IAP
/**
@brief CRC cal proc,include : Reflect,init_crc32_table,GenerateCRC32
@param global var oldcrc32
@return states
*/
static unsigned int Reflect(unsigned long int ref, char ch)
{
	unsigned int value=0;
	int i;
	for(i = 1; i < (ch + 1); i++)
	{
		if(ref & 1)
			value |= 1 << (ch - i);
		ref >>= 1;
	}
	return value;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  CRC Check Program INIT								                                           		   */
/*---------------------------------------------------------------------------------------------------------*/
static void init_crc32_table(void)
{
	unsigned int temp;
	unsigned int t1,t2;
	unsigned int flag;
	int i,j;
	for(i = 0; i <= 0xFF; i++)
	{
		temp=Reflect(i, 8);
		crc32_table[i]= temp<< 24;
		for (j = 0; j < 8; j++)
		{

			flag=crc32_table[i]&0x80000000;
			t1=(crc32_table[i] << 1);
			if(flag==0)
				t2=0;
			else
				t2=ulPolynomial;
			crc32_table[i] =t1^t2 ;

		}
		crc32_table[i] = Reflect(crc32_table[i], 32);
	}
}
/*---------------------------------------------------------------------------------------------------------*/
/*  CRC main Program									                                           		   */
/*---------------------------------------------------------------------------------------------------------*/
static void GenerateCRC32(unsigned char * buf, unsigned int len)
{
	unsigned int i;
	unsigned int t;

	for (i = 0; i != len; ++i)
	{
		t = (oldcrc32 ^ buf[i]) & 0xFF;
		oldcrc32 = ((oldcrc32 >> 8) & 0xFFFFFF) ^ crc32_table[t];
	}
}

static struct file * update_file_open(char * path, mm_segment_t * old_fs_p)
{
	struct file * filp = NULL;
	int errno = -1;
		
	filp = filp_open(path, O_RDONLY, 0644);
	
	if(!filp || IS_ERR(filp))
	{
		if(!filp)
			errno = -ENOENT;
		else 
			errno = PTR_ERR(filp);					
		printk(KERN_ERR "The update file for Guitar open error.\n");
		return NULL;
	}
	*old_fs_p = get_fs();
	set_fs(get_ds());

	filp->f_op->llseek(filp,0,0);
	return filp ;
}

static void update_file_close(struct file * filp, mm_segment_t old_fs)
{
	set_fs(old_fs);
	if(filp)
		filp_close(filp, NULL);
}
static int update_get_flen(char * path)
{
	struct file * file_ck = NULL;
	mm_segment_t old_fs;
	int length ;
	
	file_ck = update_file_open(path, &old_fs);
	if(file_ck == NULL)
		return 0;

	length = file_ck->f_op->llseek(file_ck, 0, SEEK_END);
	//printk("File length: %d\n", length);
	if(length < 0)
		length = 0;
	update_file_close(file_ck, old_fs);
	return length;	
}
static int update_file_check(char * path)
{
	unsigned char buffer[64] = { 0 } ;
	struct file * file_ck = NULL;
	mm_segment_t old_fs;
	int count, ret, length ;
	
	file_ck = update_file_open(path, &old_fs);
	
	if(path != NULL)
		printk("File Path:%s\n", path);
	
	if(file_ck == NULL)
		return -ERROR_NO_FILE;

	length = file_ck->f_op->llseek(file_ck, 0, SEEK_END);
#ifdef GUITAR_MESSAGE
	printk(KERN_INFO "gt801 update: File length: %d\n",length);
#endif	
	if(length <= 0 || (length%4) != 0)
	{
		update_file_close(file_ck, old_fs);
		return -ERROR_FILE_TYPE;
	}
	
	//set file point to the begining of the file
	file_ck->f_op->llseek(file_ck, 0, SEEK_SET);	
	oldcrc32 = 0xFFFFFFFF;
	init_crc32_table();
	while(length > 0)
	{
		ret = file_ck->f_op->read(file_ck, buffer, sizeof(buffer), &file_ck->f_pos);
		if(ret > 0)
		{
			for(count = 0; count < ret;  count++) 	
				GenerateCRC32(&buffer[count],1);			
		}
		else 
		{
			update_file_close(file_ck, old_fs);
			return -ERROR_FILE_READ;
		}
		length -= ret;
	}
	oldcrc32 = ~oldcrc32;
#ifdef GUITAR_MESSAGE	
	printk("CRC_Check: %u\n", oldcrc32);
#endif	
	update_file_close(file_ck, old_fs);
	return 1;	
}

unsigned char wait_slave_ready(struct goodix_ts_data *ts, unsigned short *timeout)
{
	unsigned char i2c_state_buf[2] = {ADDR_STA, UNKNOWN_ERROR};
	int ret;
	while(*timeout < MAX_TIMEOUT)
	{
		ret = i2c_read_bytes(ts->client, i2c_state_buf, 2);
		if(ret <= 0)
			return ERROR_I2C_TRANSFER;
		if(i2c_state_buf[1] & SLAVE_READY)
		{
			return i2c_state_buf[1];
			//return 1;
		}
		msleep(10);
		*timeout += 5;
	}
	return 0;
}

static int goodix_update_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	unsigned char cmd[220];
	int ret = -1;

	static unsigned char update_path[100];
	static unsigned short time_count = 0;
	static unsigned int file_len = 0;
	
	unsigned char i2c_control_buf[2] = {ADDR_CMD, 0};
	unsigned char i2c_states_buf[2] = {ADDR_STA, 0};
	unsigned char i2c_data_buf[PACK_SIZE+1+8] = {ADDR_DAT,};
	//unsigned char i2c_rd_buf[1+4+PACK_SIZE+4];
	unsigned char i2c_rd_buf[160];
	unsigned char retries = 0;
	unsigned int rd_len;
	unsigned char i = 0;
	static unsigned char update_need_config = 0;

	unsigned char checksum_error_times = 0;
#ifdef UPDATE_NEW_PROTOCOL
	unsigned int frame_checksum = 0;
	unsigned int frame_number = 0;
#else
	unsigned char send_crc = 0;
#endif

	struct file * file_data = NULL;
	mm_segment_t old_fs;
	struct goodix_ts_data *ts;
	
	ts = i2c_get_clientdata(i2c_connect_client);
	if(ts==NULL)
		return 0;
	
	if(copy_from_user(&cmd, buff, len))
	{
		return -EFAULT;
	}
	switch(cmd[0])
	{
		case STEP_SET_PATH:
			printk(KERN_INFO"Write cmd is:%d,cmd arg is:%s,write len is:%ld\n",cmd[0], &cmd[1], len);
			memset(update_path, 0, 100);
			strncpy(update_path, cmd+1, 100);
			if(update_path[0] == 0)
				return 0;
			else
				return 1;
		case STEP_CHECK_FILE:
			printk(KERN_INFO"Begin to firmware update ......\n");
			ret = update_file_check(update_path);
			if(ret <= 0)
			{
				printk(KERN_INFO"fialed to check update file!\n");
				return ret;
			}
			msleep(500);
			printk(KERN_INFO"Update check file success!\n");
			return 1;
		case STEP_WRITE_SYN:
			printk(KERN_INFO"STEP1:Write synchronization signal!\n");
			i2c_control_buf[1] = UPDATE_START;
			if(ts->green_wake_mode)
			{
				//disable_irq(client->irq);

				/*
				gpio_direction_output(INT_PORT, 0);
				msleep(5);
				s3c_gpio_cfgpin(INT_PORT, INT_CFG);
				*/

				goodix_irq_reset();

				
				//enable_irq(client->irq);
			}
			ret = i2c_write_bytes(ts->client, i2c_control_buf, 2);
			if(ret <= 0)
			{
				ret = ERROR_I2C_TRANSFER;
				return ret;
			}
			//the time include time(APROM -> LDROM) and time(LDROM init)
			msleep(1000);
			return 1;
		case STEP_WAIT_SYN:
			printk(KERN_INFO"STEP2:Wait synchronization signal!\n");
			while(retries < MAX_I2C_RETRIES)
			{
				i2c_states_buf[1] = UNKNOWN_ERROR;
				ret = i2c_read_bytes(ts->client, i2c_states_buf, 2);
				printk(KERN_INFO"The read byte is:%d\n", i2c_states_buf[1]);
				if(i2c_states_buf[1] & UPDATE_START)
				{
					if(i2c_states_buf[1] & NEW_UPDATE_START)
					{
					#ifdef UPDATE_NEW_PROTOCOL
						update_need_config = 1;
						return 2;
					#else
						return 1;
					#endif
					}
					break;
				}
				msleep(5);
				retries++;
				time_count += 10;
			}
			if((retries >= MAX_I2C_RETRIES) && (!(i2c_states_buf[1] & UPDATE_START)))
			{
				if(ret <= 0)
					return 0;
				else
					return -1;
			}
			return 1;
		case STEP_WRITE_LENGTH:
			printk(KERN_INFO"STEP3:Write total update file length!\n");
			file_len = update_get_flen(update_path);
			if(file_len <= 0)
			{
				printk(KERN_INFO"get update file length failed!\n");
				return -1;
			}
			file_len += 4;
			i2c_data_buf[1] = (file_len>>24) & 0xff;
			i2c_data_buf[2] = (file_len>>16) & 0xff;
			i2c_data_buf[3] = (file_len>>8) & 0xff;
			i2c_data_buf[4] = file_len & 0xff;
			file_len -= 4;
			ret = i2c_write_bytes(ts->client, i2c_data_buf, 5);
			if(ret <= 0)
			{
				ret = ERROR_I2C_TRANSFER;
				return 0;
			}
			return 1;
		case STEP_WAIT_READY:
			printk(KERN_INFO"STEP4:Wait slave ready!\n");
			ret = wait_slave_ready(ts, &time_count);
			if(ret == ERROR_I2C_TRANSFER)
				return 0;
			if(!ret)
			{
				return -1;
			}
			printk(KERN_INFO"Slave ready!\n");
			return 1;
		case STEP_WRITE_DATA:
#ifdef UPDATE_NEW_PROTOCOL
			printk(KERN_INFO"STEP5:Begin to send file data use NEW protocol!\n");
			file_data = update_file_open(update_path, &old_fs);
			if(file_data == NULL)
			{
				return -1;
			}
			frame_number = 0;
			while(file_len >= 0)
			{
				i2c_data_buf[0] = ADDR_DAT;
				rd_len = (file_len >= PACK_SIZE) ? PACK_SIZE : file_len;
				frame_checksum = 0;
				if(file_len)
				{
					ret = file_data->f_op->read(file_data, i2c_data_buf+1+4, rd_len, &file_data->f_pos);
					if(ret <= 0)
					{
						printk("[GOODiX_ISP_NEW]:Read File Data Failed!\n");
						return -1;
					}
					i2c_data_buf[1] = (frame_number>>24)&0xff;
					i2c_data_buf[2] = (frame_number>>16)&0xff;
					i2c_data_buf[3] = (frame_number>>8)&0xff;
					i2c_data_buf[4] = frame_number&0xff;
					frame_number++;
					frame_checksum = 0;
					for(i=0; i<rd_len; i++)
					{
						frame_checksum += i2c_data_buf[5+i];
					}
					frame_checksum = 0 - frame_checksum;
					i2c_data_buf[5+rd_len+0] = frame_checksum&0xff;
					i2c_data_buf[5+rd_len+1] = (frame_checksum>>8)&0xff;
					i2c_data_buf[5+rd_len+2] = (frame_checksum>>16)&0xff;
					i2c_data_buf[5+rd_len+3] = (frame_checksum>>24)&0xff;
				}
rewrite:
				printk(KERN_INFO"[GOODiX_ISP_NEW]:%d\n", file_len);				
				ret = i2c_write_bytes(ts->client, i2c_data_buf, 1+4+rd_len+4);
					//if(ret <= 0)
				if(ret != 1)
				{
					printk("[GOODiX_ISP_NEW]:Write File Data Failed!Return:%d\n", ret);
					return 0;
				}

				memset(i2c_rd_buf, 0x00, 1+4+rd_len+4);
				ret = i2c_read_bytes(ts->client, i2c_rd_buf, 1+4+rd_len+4);
				printk("*************************** Return:%d\n", ret);
				if(ret <= 0)
				{
					printk("[GOODiX_ISP_NEW]:Read File Data Failed!Return:%d\n", ret);
					return 0;
				}
				for(i=1; i<(1+4+rd_len+4); i++)						//check communication
				{
					if(i2c_rd_buf[i] != i2c_data_buf[i])
					{
						i = 0;
						break;
					}
				}
				if(!i)
				{
					i2c_control_buf[0] = ADDR_CMD;
					i2c_control_buf[1] = 0x03;
					i2c_write_bytes(ts->client, i2c_control_buf, 2);		//communication error
					printk("[GOODiX_ISP_NEW]:File Data Frame readback check Error!\n");
				}
				else
				{
					i2c_control_buf[1] = 0x04;													//let LDROM write flash
					i2c_write_bytes(ts->client, i2c_control_buf, 2);
				}
				
				//Wait for slave ready signal.and read the checksum
				ret = wait_slave_ready(ts, &time_count);
				if((ret & CHECKSUM_ERROR)||(!i))
				{
					if(i)
					{
						printk("[GOODiX_ISP_NEW]:File Data Frame checksum Error!\n");
					}
					checksum_error_times++;
					msleep(20);
					if(checksum_error_times > 20)				//max retry times.
						return 0;
					goto rewrite;
				}
				checksum_error_times = 0;
				if(ret & (FRAME_ERROR))
				{
					printk("[GOODiX_ISP_NEW]:File Data Frame Miss!\n");
					return 0;
				}
				if(ret == ERROR_I2C_TRANSFER)
					return 0;
				if(!ret)
				{
					return -1;
				}
				if(file_len < PACK_SIZE)
				{
					update_file_close(file_data, old_fs);
					break;
				}
				file_len -= rd_len;
			}//end of while((file_len >= 0))
			return 1;
#else
			printk(KERN_INFO"STEP5:Begin to send file data use OLD protocol!\n");
			file_data = update_file_open(update_path, &old_fs);
			if(file_data == NULL)	//file_data has been opened at the last time
			{
				return -1;
			}
			while((file_len >= 0) && (!send_crc))
			{
				printk(KERN_INFO"[GOODiX_ISP_OLD]:%d\n", file_len);
				i2c_data_buf[0] = ADDR_DAT;
				rd_len = (file_len >= PACK_SIZE) ? PACK_SIZE : file_len;
				if(file_len)
				{
					ret = file_data->f_op->read(file_data, i2c_data_buf+1, rd_len, &file_data->f_pos);
					if(ret <= 0)
					{
						return -1;
					}
				}
				if(file_len < PACK_SIZE)
				{
					send_crc = 1;
					update_file_close(file_data, old_fs);
					i2c_data_buf[file_len+1] = oldcrc32&0xff;
					i2c_data_buf[file_len+2] = (oldcrc32>>8)&0xff;
					i2c_data_buf[file_len+3] = (oldcrc32>>16)&0xff;
					i2c_data_buf[file_len+4] = (oldcrc32>>24)&0xff;
					ret = i2c_write_bytes(ts->client, i2c_data_buf, (file_len+1+4));
					//if(ret <= 0)
					if(ret != 1)
					{
						printk("[GOODiX_ISP_OLD]:Write File Data Failed!Return:%d\n", ret);
						return 0;
					}
					break;
				}
				else
				{
					ret = i2c_write_bytes(ts->client, i2c_data_buf, PACK_SIZE+1);
					//if(ret <= 0)
					if(ret != 1)
					{
						printk("[GOODiX_ISP_OLD]:Write File Data Failed!Return:%d\n", ret);
						return 0;
					}
				}
				file_len -= rd_len;
			
				//Wait for slave ready signal.
				ret = wait_slave_ready(ts, &time_count);
				if(ret == ERROR_I2C_TRANSFER)
					return 0;
				if(!ret)
				{
					return -1;
				}
				//Slave is ready.
			}//end of while((file_len >= 0) && (!send_crc))
			return 1;
#endif
		case STEP_READ_STATUS:
			printk(KERN_INFO"STEP6:Read update status!\n");
			while(time_count < MAX_TIMEOUT)
			{
				ret = i2c_read_bytes(ts->client, i2c_states_buf, 2);
				if(ret <= 0)
				{
					return 0;
				}
				if(i2c_states_buf[1] & SLAVE_READY)
				{
					if(!(i2c_states_buf[1] &0xf0))
					{
						printk(KERN_INFO"The firmware updating succeed!update state:0x%x\n",i2c_states_buf[1]);
						return 1;
					}
					else
					{
						printk(KERN_INFO"The firmware updating failed!update state:0x%x\n",i2c_states_buf[1]);
						return 0;

					}
				}
				msleep(1);
				time_count += 5;
			}
			return -1;
		case FUN_CLR_VAL:								//clear the static val
			time_count = 0;
			file_len = 0;
			update_need_config = 0;
			return 1;
		case FUN_CMD:							//functional command
			if(cmd[1] == CMD_DISABLE_TP)
			{
				printk(KERN_INFO"Disable TS int!\n");
				g_enter_isp = 1;
				if(ts->use_irq)
					disable_irq(TS_INT);
			}
			else if(cmd[1] == CMD_ENABLE_TP)
			{
				printk(KERN_INFO"Enable TS int!\n");
				g_enter_isp = 0;
				if(ts->use_irq)
					enable_irq(TS_INT);
			}
			else if(cmd[1] == CMD_READ_VER)
			{
				printk(KERN_INFO"Read version!\n");
				ts->read_mode = MODE_RD_VER;
			}
			else if(cmd[1] == CMD_READ_RAW)
			{
				printk(KERN_INFO"Read raw data!\n");
				ts->read_mode = MODE_RD_RAW;
				i2c_control_buf[1] = 201;
				ret = i2c_write_bytes(ts->client, i2c_control_buf, 2);			//read raw data cmd
				if(ret <= 0)
				{
					printk(KERN_INFO"Write read raw data cmd failed!\n");
					return 0;
				}
				msleep(200);
			}
			else if(cmd[1] == CMD_READ_DIF)
			{
				printk(KERN_INFO"Read diff data!\n");
				ts->read_mode = MODE_RD_DIF;
				i2c_control_buf[1] = 202;
				ret = i2c_write_bytes(ts->client, i2c_control_buf, 2);			//read diff data cmd
				if(ret <= 0)
				{
					printk(KERN_INFO"Write read raw data cmd failed!\n");
					return 0;
				}
				msleep(200);
			}
			else if(cmd[1] == CMD_READ_CFG)
			{
				printk(KERN_INFO"Read config info!\n");
				ts->read_mode = MODE_RD_CFG;
				rd_cfg_addr = cmd[2];
				rd_cfg_len = cmd[3];
			}
			else if(cmd[1] == CMD_SYS_REBOOT)
			{
				printk(KERN_INFO"System reboot!\n");
				#if !defined(CONFIG_MULTI_TP_SUPPORT)
				sys_sync();
				#endif
				msleep(200);
				kernel_restart(NULL);
			}
			return 1;
		case FUN_WRITE_CONFIG:
			
			printk(KERN_INFO"Begin write config info!Config length:%d\n",cmd[1]);
			for(i=3; i<cmd[1];i++)
			{
				//if((i-3)%5 == 0)printk("\n");
				printk("(%d):0x%x ", i-3, cmd[i]);
			}
			printk("\n");

			if((cmd[2]>83)&&(cmd[2]<240)&&cmd[1])
			{
				checksum_error_times = 0;
				if(ts->green_wake_mode)			//WAKEUP GREEN MODE
				{
					if(!update_need_config)
						disable_irq(ts->client->irq);

					/*
					gpio_direction_output(INT_PORT, 0);
					msleep(5);
					s3c_gpio_cfgpin(INT_PORT, INT_CFG);
					*/

					goodix_irq_reset();

					
					if(!update_need_config)
						enable_irq(ts->client->irq);
				}
reconfig:
				ret = i2c_write_bytes(ts->client, cmd+2, cmd[1]); 
				if(ret != 1)
				{
					printk("Write Config failed!return:%d\n",ret);
					return -1;
				}
				if(!update_need_config)return 1;
				
				i2c_rd_buf[0] = cmd[2];
				ret = i2c_read_bytes(ts->client, i2c_rd_buf, cmd[1]);
				if(ret <= 0)
				{
					printk("Read Config failed!return:%d\n",ret);
					return -1;
				}
				for(i=0; i<cmd[1]; i++)
				{
					if(i2c_rd_buf[i] != cmd[i+2])
					{
						printk("Config readback check failed!\n");
						i = 0;
						break;
					}
				}
				if(!i)
				{
					i2c_control_buf[0] = ADDR_CMD;
					i2c_control_buf[1] = 0x03;
					i2c_write_bytes(ts->client, i2c_control_buf, 2);		//communication error
					checksum_error_times++;
					msleep(20);
					if(checksum_error_times > 20)				//max retry times.
						return 0;
					goto reconfig;
				}
				else
				{
					i2c_control_buf[0] = ADDR_CMD;
					i2c_control_buf[1] = 0x04;					//let LDROM write flash
					i2c_write_bytes(ts->client, i2c_control_buf, 2);
					return 1;
				}
				
			}
			else
			{
				printk(KERN_INFO"Invalid config addr!\n");
				return -1;
			}
		default:
			return -ENOSYS;
	}
	return 0;
}

static int goodix_update_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
	int ret = -1;
	struct goodix_ts_data *ts;
	int len = 0;
	char *version_info = NULL;
	unsigned char read_data[1201] = {80, };

	ts = i2c_get_clientdata(i2c_connect_client);
	if(ts==NULL)
		return 0;

	if(ts->read_mode == MODE_RD_VER)		//read version data
	{
		ret = goodix_read_version(ts, &version_info);
		if(ret <= 0)
		{
			printk(KERN_INFO"Read version data failed!\n");
			vfree(version_info);
			return 0;
		}

		for(len=0;len<100;len++)
		{
			if(*(version_info + len) == '\0')
				break;
		}
		printk(KERN_INFO"GOODiX Touchscreen Version is:%s\n", (version_info+1));
		strncpy(page, version_info+1, len + 1);
		vfree(version_info);
		*eof = 1;
		return len+1;
	}
	else if((ts->read_mode == MODE_RD_RAW)||(ts->read_mode == MODE_RD_DIF))		//read raw data or diff
	{
		//printk(KERN_INFO"Read raw data\n");
		ret = i2c_read_bytes(ts->client, read_data, 1201);
		if(ret <= 0)
		{
			if(ts->read_mode == 2)
				printk(KERN_INFO"Read raw data failed!\n");
			if(ts->read_mode == 3)
				printk(KERN_INFO"Read diff data failed!\n");
			return 0;
		}
		memcpy(page, read_data+1, 1200);
		*eof = 1;
		*start = NULL;
		return 1200;
	}
	else if(ts->read_mode == MODE_RD_CFG)
	{
		if((rd_cfg_addr>83)&&(rd_cfg_addr<240))
		{
			read_data[0] = rd_cfg_addr;
			printk("read config addr is:%d\n", rd_cfg_addr);
		}
		else
		{
			read_data[0] = 101;
			printk("invalid read config addr,use default!\n");
		}
		if((rd_cfg_len<0)||(rd_cfg_len>156))
		{
			printk("invalid read config length,use default!\n");
			rd_cfg_len = 239 - read_data[0];
		}
		printk("read config length is:%d\n", rd_cfg_len);
		ret = i2c_read_bytes(ts->client, read_data, rd_cfg_len);
		if(ret <= 0)
		{
			printk(KERN_INFO"Read config info failed!\n");
			return 0;
		}
		memcpy(page, read_data+1, rd_cfg_len);
		return rd_cfg_len;
	}
	return len;
}
              
#endif
//******************************End of firmware update surpport*******************************
static const struct i2c_device_id goodix_ts_id[] = {
	{ GOODIX_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver goodix_ts_driver = {
	.probe		= goodix_ts_probe,
	.remove		= goodix_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= goodix_ts_suspend,
	.resume		= goodix_ts_resume,
#endif
	.id_table	= goodix_ts_id,
	.driver = {
		.name	= GOODIX_I2C_NAME,
		.owner = THIS_MODULE,
	},
};


#define I2C_BUS_ID 	0
#define I2C_ADD		0x55//ox1010101
/*
*	add i2c driver
*/
static int goodix_add_i2c_device(void)
{
	printk("********************** goodix_add_i2c_device\n");

	struct i2c_board_info board_info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int ret;

	//return i2c_add_driver(&sis_ts_driver);
	ret = i2c_add_driver(&goodix_ts_driver);
	if (ret != 0) {
		printk("can't add i2c driver,%s\n",__func__);
		return ret;
	}

	memset(&board_info, 0, sizeof(struct i2c_board_info));
	board_info.addr = I2C_ADD;
	strlcpy(board_info.type, GOODIX_I2C_NAME, I2C_NAME_SIZE);

	adapter = i2c_get_adapter(I2C_BUS_ID);
	if (!adapter) {
		printk("can't get i2c adapter %d\n", I2C_BUS_ID);
		goto err_driver;
	}
	client = i2c_new_device(adapter, &board_info);
	i2c_put_adapter(adapter);
	if (!client) {
		printk("can't add i2c device at 0x%x\n", (unsigned int)board_info.addr);
		goto err_driver;
	}

	return 0;

err_driver:
	i2c_del_driver(&goodix_ts_driver);
	return -ENODEV;
}
/*******************************************************	
Description:
	Driver Install function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static int __devinit goodix_ts_init(void)
{
	int ret;
	
	goodix_wq = create_workqueue("goodix_wq");		//create a work queue and worker thread
	if (!goodix_wq) {
		printk(KERN_ALERT "creat workqueue faiked\n");
		return -ENOMEM;
		
	}
	//ret=i2c_add_driver(&goodix_ts_driver);
	
	ret = goodix_add_i2c_device();
	return ret; 
}

/*******************************************************	
Description:
	Driver uninstall function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static void __exit goodix_ts_exit(void)
{
	printk(KERN_ALERT "Touchscreen driver of guitar exited.\n");
	i2c_del_driver(&goodix_ts_driver);
	if (goodix_wq)
		destroy_workqueue(goodix_wq);		//release our work queue
}

late_initcall(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("Goodix Touchscreen Driver");
MODULE_LICENSE("GPL");
