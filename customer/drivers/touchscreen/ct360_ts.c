/*
 * drivers/input/touchscreen/ct360_ts.c
 *
 * Copyright (C) 2012 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
//#include <mach/gpio.h>
//#include <mach/iomux.h>
//#include <mach/board.h>
#include <linux/platform_device.h>
#include <asm/mach/time.h>
#include <linux/ct360_ts.h>
#include "ct360_ch.h"

volatile int is_ct360 = 1;
#define CT360_DEBUG		0
#define TOUCH_NUMBER	10
#define TOUCH_REG_NUM	6 
#define ct360_TS_NAME	"ct360_ts"

#define CT360_WRITE_FIRMWARE 0



#if CT360_DEBUG
	#define	ct360printk(msg...)	printk(msg)
#else
	#define	ct360printk(msg...)
#endif
#if 1
	#define  boot_printk(msg...) printk(msg)
#else
	#define  boot_printk(msg...)
#endif


int giDownIdNumber[TOUCH_NUMBER] = {0};

#if 0
struct ct360_ts_data {
	u16		x_max;	
	u16		y_max;
	int 	irq;
	struct	i2c_client			*client;
    struct	input_dev			*input_dev;
	struct	workqueue_struct	*ct360_wq;
    struct	work_struct			work;
    struct	early_suspend		early_suspend;
};
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ct360_ts_early_suspend(struct early_suspend *h);
static void ct360_ts_early_resume(struct early_suspend *h);
#endif

extern char Binary_Data[]; 
/*read the ct360 register ,used i2c bus*/
static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret=-1;
	msgs[0].flags=!I2C_M_RD;
	msgs[0].addr=client->addr;
	msgs[0].len=1;
	msgs[0].buf=&buf[0];

	msgs[1].flags=I2C_M_RD;
	msgs[1].addr=client->addr;
	msgs[1].len=len;
	msgs[1].buf=&buf[0];
	
	ret=i2c_transfer(client->adapter,msgs,2);
	return ret;
}

static int ct360_read_regs(struct i2c_client *client, uint8_t *buf, unsigned len)
{
#if 0
    int liRet;
	
	liRet =i2c_read_bytes(client, buf, len);
	if(liRet < 0)
	{
		printk("ct360_ts_work_func:i2c_transfer fail =%d\n",liRet);
	}
	else
	{
		//nothing
	}
	
	return liRet;
#else

	int ret,count;
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = client->flags | I2C_M_RD;
	msg.len = len;
	msg.buf = (char *)buf;

	ret = i2c_transfer(adap, &msg, 1);
             if(ret < 0)
                {
                  is_ct360 = 0;
                  printk("is_ct360 ====  %d, go to focal\n\n",is_ct360);
                  return -1;
                }
	return (ret == 1) ? count : ret;
#endif
}
/* set the ct360 registe,used i2c bus*/
static int ct360_write_regs(struct i2c_client *client, uint8_t *buf, unsigned short len)
{
#if 0
    int liRet;
	
	liRet = ct360_write_regs(client, buf, len);
 	if (liRet < 0)
	{
	  printk("ct360_ts_work_func:i2c_transfer fail =%d\n",liRet);
    }
	else
	{
		//nothing
	}
	
	return liRet;
#else

	int ret,count;
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = (char *)buf;

	ret = i2c_transfer(adap, &msg, 1);
	return (ret == 1) ? count : ret;
#endif
}

static void ct360_ts_work_func(struct work_struct *work)
{	
	unsigned short x;
	unsigned short y;
	int liTmp,liRet;
	unsigned char buf[TOUCH_REG_NUM*TOUCH_NUMBER] = {0};
	int liPointStatus;
	int liPointNumber;
	int syn_flag = 0;
	
	//printk("enter ct360_ts_work_func\n");
	struct ct360_ts_data *ts = container_of(work,struct ct360_ts_data,work);
	
	liRet= ct360_read_regs(ts->client,buf,TOUCH_REG_NUM*TOUCH_NUMBER);//only one data  represent the current touch num
	if (liRet < 0) {
	  	printk("%s:i2c_transfer fail =%d\n", __FUNCTION__, liRet);
		enable_irq(ts->irq);
		return;
    }
	
    for (liTmp = 0; liTmp < TOUCH_REG_NUM*TOUCH_NUMBER; liTmp += TOUCH_REG_NUM)
    {
		x = ((buf[liTmp + 1] << 4) | (buf[liTmp + 3] >> 4));
		y = ((buf[liTmp + 2] << 4) | (buf[liTmp + 3] & 0x0F));

		x = ts->x_max - x;
		//y = ts->y_max - y;
		//printk("the x is %d\n",x);
		//printk("the y is %d\n",y);
		
		liPointStatus	= buf[liTmp + 0] & 0x07;
		//if(liPointStatus!=15)
		//printk("hj---->liPointStatus = %d\n",liPointStatus);
		liPointNumber	= buf[liTmp + 0] >> 3;
//		printk("the liPointStatus is %d\n",liPointStatus);
//		printk("the liPointNumber is %d\n",liPointNumber);


/*	
		x = ((buf[liTmp + 0] << 4) | (buf[liTmp + 2] >> 4));
		y = ((buf[liTmp + 1] << 4) | (buf[liTmp + 2] & 0x0F));
		printk("the x is %d\n",x);
		printk("the y is %d\n",y);
		
		liPointStatus	= buf[liTmp + 3] & 0x07;
		//if(liPointStatus!=15)
		//printk("hj---->liPointStatus = %d\n",liPointStatus);
		liPointNumber	= buf[liTmp + 3] >> 3;
//		printk("the liPointStatus is %d\n",liPointStatus);
//		printk("the liPointNumber is %d\n",liPointNumber);
*/
	
	
		if ((x != 4095) && ( liPointNumber != 0))
		{
			if((liPointStatus == 1) || (liPointStatus == 2))
			{
//				printk("point %d: down x=%d, y=%d\n", liPointNumber, x, y);
				input_mt_slot(ts->input_dev, (liPointNumber - 1));
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, (liPointNumber - 1));
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 1);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 5);
				giDownIdNumber[liPointNumber - 1] = 1;
				syn_flag = 1;
			}
			//else if(liPointStatus == 3 || liPointStatus == 15)
			else if(liPointStatus == 3) //|| liPointStatus == 15)
			{
//				printk("point %d: up\n", liPointNumber);
				input_mt_slot(ts->input_dev, (liPointNumber - 1));
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
				giDownIdNumber[liPointNumber - 1] = 0;
				syn_flag = 1;
			}
			
		}
		
	}
	if(syn_flag){
		input_sync(ts->input_dev);
	}
	enable_irq(ts->irq);
	return;
}

static irqreturn_t ct360_ts_irq_handler(int irq, void *dev_id)
{
	//printk("enter ct360_ts_irq_handler\n");
    struct ct360_ts_data *ts = dev_id;
	disable_irq_nosync(ts->irq);
	
	
    queue_work(ts->ct360_wq, &ts->work);
//	printk("hj---->modify timer after 30ms\n");
//	mod_timer(&ts->timer, jiffies + msecs_to_jiffies(30));

    return IRQ_HANDLED;
}

static void ct360_irq_timer(unsigned long _data)
{
	int liNumber;
	struct ct360_ts_data *ts = (struct ct360_ts_data *)_data;
	
	for(liNumber = 0;liNumber < TOUCH_NUMBER; liNumber++)
		{	
			if(giDownIdNumber[liNumber]==1)
			{
				//printk("point %d: up	   +ct360_ts_resume\n", liNumber);
				input_mt_slot(ts->input_dev, liNumber);
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
				input_sync(ts->input_dev);
			}
			
		}
}



char CT360_CTP_BootLoader(struct ct360_ts_data *ts)
{
	char value = 0;
	char I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int i = 0, j = 0;
	unsigned int Flash_Address = 0;
	unsigned char CheckSum[16];
	int poweroff = 0;
	//Step 00 : initBootLoader
	ts->client->addr = 0x7F;
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0xA5;
	ct360_write_regs(ts->client,I2C_Buf,2); // Write a “A5H” to CT360
	mdelay(20);

	// Read CT360 status
	ct360_write_regs(ts->client,I2C_Buf,1);	
	mdelay(1);
	ct360_read_regs(ts->client,&value,1);

	boot_printk("%s......0...\n",__FUNCTION__);
	// if return “AAH” then going next step
	if (value != 0xAA)
		return 0;
	boot_printk("%s......1...\n",__FUNCTION__);

/**********************************************************************/
	//Step 0 : force CT360 generate check sum for host to compare data.
	//VTL_Address = 0x00A4
	//Prepare get check sum from CT360
					I2C_Buf[0] = 0x00;
					I2C_Buf[1] = 0x99; //Generate check sum command
					I2C_Buf[2] = (char)(0x0044 >> 8); //define a flash address for CT360 to generate check sum
					I2C_Buf[3] = (char)(0x0044 & 0xFF); //
					I2C_Buf[4] = 0x08; //Define a data length for CT360 to generate check sum
					ct360_write_regs(ts->client,I2C_Buf, 5); //Write Genertate check sum command to CT360
					mdelay(2); //Delay 1mS
					
					I2C_Buf[0] = 0x00;
					ct360_write_regs(ts->client,I2C_Buf,1); 
					mdelay(1);
					ct360_read_regs(ts->client,I2C_Buf, 13); // Read check sum and flash data from CT360

	//Compare host check sum with CT360 check sum(I2C_Buf[5]  I2C_Buf[9] )
		if ((I2C_Buf[5] != 0x56) || (I2C_Buf[9] != 0x54))
		//	return 0;	


	//Step 0 : force CT360 generate check sum for host to compare data.
	//VTL_Address = 0x00A4
	//Prepare get check sum from CT360
	{				I2C_Buf[0] = 0x00;
					I2C_Buf[1] = 0x99; //Generate check sum command
					I2C_Buf[2] = (char)(0x00A4 >> 8); //define a flash address for CT360 to generate check sum
					I2C_Buf[3] = (char)(0x00A4 & 0xFF); //
					I2C_Buf[4] = 0x08; //Define a data length for CT360 to generate check sum
					ct360_write_regs(ts->client,I2C_Buf, 5); //Write Genertate check sum command to CT360
					mdelay(2); //Delay 1mS
					
					I2C_Buf[0] = 0x00;
					ct360_write_regs(ts->client,I2C_Buf,1); 
					mdelay(1);
					ct360_read_regs(ts->client,I2C_Buf, 13); // Read check sum and flash data from CT360

	//Compare host check sum with CT360 check sum(I2C_Buf[5]  I2C_Buf[9] )
		if (((I2C_Buf[5] != 0x56) || (I2C_Buf[9] != 0x54))&&(poweroff == 0))
			return 0;
	}
	
	///////////////////////////////////////////////////////
	boot_printk("%s......2..\n",__FUNCTION__);
	//Step 1 : initBootLoader
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0xA5;
	ct360_write_regs(ts->client,I2C_Buf,2); // Write a “A5H” to CT360
	mdelay(20);

	I2C_Buf[0] = 0x00;
	ct360_write_regs(ts->client,I2C_Buf,1); // Write a “A5H” to CT360
	mdelay(1);
	ct360_read_regs(ts->client,&value, 1);


	// if return “AAH” then going next step
	if (value != 0xAA)
	return 0;

	//////////////////////////////////////////////////////
	// erase info block
	// info block erase command
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x60;
	I2C_Buf[2] = 0x00;
	ct360_write_regs(ts->client, I2C_Buf, 3);
	mdelay(10);
 
	// Reset I2C offset address
	I2C_Buf[0] = 0x00;
	ct360_write_regs(ts->client,I2C_Buf,1);
	mdelay(1);
 
	// Read I2C bus status
	ct360_read_regs(ts->client, &value, 1);
	// if return “AAH” then going next step
	if (value != 0xAA) {
		printk("trim data erase error!!! \n");
		//return 0;
	}

	///////////////////////////////////////////////////////
	// write init data into info block
	Flash_Address = 0x00;

	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x61;
 
	for ( i = 0; i < 16; i++ ) {
		// Flash address
		// data length
		I2C_Buf[2] = (char)(Flash_Address >> 8);
		I2C_Buf[3] = (char)(Flash_Address & 0xFF);
		I2C_Buf[4] = 0x08;
		if ( Flash_Address == 0x0000 )
		I2C_Buf[6] = 0x17;
		else
		I2C_Buf[6] = 0x00;

		I2C_Buf[7] = 0x00;
		I2C_Buf[8] = 0x00;
		I2C_Buf[9] = 0x00;
		I2C_Buf[10] = 0x00;
		I2C_Buf[11] = 0x00;
		I2C_Buf[12] = 0x00;
		I2C_Buf[13] = 0x00;

		I2C_Buf[5] = (~(I2C_Buf[2]+I2C_Buf[3]+I2C_Buf[4]+I2C_Buf[6]+I2C_Buf[7]+I2C_Buf[8]+I2C_Buf[9]+I2C_Buf[10]+I2C_Buf[11]+I2C_Buf[12]+I2C_Buf[13]))+1;

		ct360_write_regs(ts->client, I2C_Buf, 14);
		mdelay(10);
 
		Flash_Address += 8;
	}



    //////////////////////////////////////////////////////
	boot_printk("%s......3...\n",__FUNCTION__);
	//Step 2 : erase flash section 0~7
	for(i = 0; i<8; i++)
	{
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x33; //Erase command
	I2C_Buf[2] = 0x00; //Flash section address
	ct360_write_regs(ts->client,I2C_Buf, 3); //Write “33H” and “Flash section” to CT360
	mdelay(80); //Delay 75mS

	I2C_Buf[0] = 0x00;
	ct360_write_regs(ts->client,I2C_Buf,1); // Write a “A5H” to CT360
	mdelay(1);
	
	ct360_read_regs(ts->client,&value, 1);
	// if CT360 return “AAH” then going next step
	if (value == 0xAA)
		break; //CT360 out of controlled
	}
	if ( i >= 8 ) return 0;

    ///////////////////////////////////////////////////////////////
	boot_printk("%s......4...\n",__FUNCTION__);	
	//for (Flash_Address=0; Flash_Address<0x3fff; Flash_Address+=8)
	boot_printk("%s: FW flashing \n",__FUNCTION__);
	for (j=0; j<256; j++)
	{
        Flash_Address = 128*j;
        for (i = 0; i < 16; i++ )
        {
        	//boot_printk("%s: flash: i=%d, j=%d\n",__FUNCTION__, i, j);
	        //Step 3 : write binary data to CT360
	        I2C_Buf[0] = 0x00;
			I2C_Buf[1] = 0x55; //Flash write command
			I2C_Buf[2] = (char)(Flash_Address >> 8); //Flash address [15:8]
			I2C_Buf[3] = (char)(Flash_Address & 0xFF); //Flash address [7:0]
			I2C_Buf[4] = 0x08; //How many prepare to write to CT360
#if 1
			if((j==1 && i==4)||(j==1 && i==5))
			{
			    I2C_Buf[6] = ~Binary_Data[Flash_Address + 0]; //Binary data 1
			    I2C_Buf[7] = ~Binary_Data[Flash_Address + 1]; //Binary data 2
			    I2C_Buf[8] = ~Binary_Data[Flash_Address + 2]; //Binary data 3
			    I2C_Buf[9] = ~Binary_Data[Flash_Address + 3]; //Binary data 4
			    I2C_Buf[10] = ~Binary_Data[Flash_Address + 4]; //Binary data 5
			    I2C_Buf[11] = ~Binary_Data[Flash_Address + 5]; //Binary data 6
			    I2C_Buf[12] = ~Binary_Data[Flash_Address + 6]; //Binary data 7
			    I2C_Buf[13] = ~Binary_Data[Flash_Address + 7]; //Binary data 8
			}
			else
			{
			    I2C_Buf[6] = Binary_Data[Flash_Address + 0]; //Binary data 1
			    I2C_Buf[7] = Binary_Data[Flash_Address + 1]; //Binary data 2
			    I2C_Buf[8] = Binary_Data[Flash_Address + 2]; //Binary data 3
			    I2C_Buf[9] = Binary_Data[Flash_Address + 3]; //Binary data 4
			    I2C_Buf[10] = Binary_Data[Flash_Address + 4]; //Binary data 5
			    I2C_Buf[11] = Binary_Data[Flash_Address + 5]; //Binary data 6
			    I2C_Buf[12] = Binary_Data[Flash_Address + 6]; //Binary data 7
			    I2C_Buf[13] = Binary_Data[Flash_Address + 7]; //Binary data 8
			}
#else	  
			I2C_Buf[6] = Binary_Data[Flash_Address + 0]; //Binary data 1
			I2C_Buf[7] = Binary_Data[Flash_Address + 1]; //Binary data 2
			I2C_Buf[8] = Binary_Data[Flash_Address + 2]; //Binary data 3
			I2C_Buf[9] = Binary_Data[Flash_Address + 3]; //Binary data 4
			I2C_Buf[10] = Binary_Data[Flash_Address + 4]; //Binary data 5
			I2C_Buf[11] = Binary_Data[Flash_Address + 5]; //Binary data 6
			I2C_Buf[12] = Binary_Data[Flash_Address + 6]; //Binary data 7
			I2C_Buf[13] = Binary_Data[Flash_Address + 7]; //Binary data 8
#endif
	// Calculate a check sum by Host controller.
	// Checksum = / (FLASH_ADRH+FLASH_ADRL+LENGTH+
	// Binary_Data1+Binary_Data2+Binary_Data3+Binary_Data4+
	// Binary_Data5+Binary_Data6+Binary_Data7+Binary_Data8) + 1
			CheckSum[i] = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] +
			I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
			I2C_Buf[13]) + 1;
			I2C_Buf[5] = CheckSum[i]; //Load check sum
			ct360_write_regs(ts->client,I2C_Buf, 14); //Host write I2C_Buf[0…12] to CT360.
			mdelay(5); //Delay 2mS
			
			Flash_Address += 8;
        }
			//I2C_Buf[0] = 0x00;
			//ct360_write_regs(ts->client,I2C_Buf,1); // Write a “A5H” to CT360
			//mdelay(2);
			//ct360_read_regs(ts->client,&value, 1);

	// if return “AAH” then going next step
			//if (value != 0xAA)
			//		return 0;
	
	    mdelay(20);

		Flash_Address = 128 * j;
	
	//Step 4 : force CT360 generate check sum for host to compare data.
	//Prepare get check sum from CT360
	//boot_printk("%s: FW verifying \n",__FUNCTION__);
	for ( i = 0; i < 16; i++ ) {									// 128/8 = 16 times for One Row program
	    I2C_Buf[0] = 0x00;
	    I2C_Buf[1] = 0x99; //Generate check sum command
	    I2C_Buf[2] = (char)(Flash_Address >> 8); //define a flash address for CT360 to generate check sum
	    I2C_Buf[3] = (char)(Flash_Address & 0xFF); //
	    I2C_Buf[4] = 0x08; //Define a data length for CT360 to generate check sum
	    ct360_write_regs(ts->client,I2C_Buf, 5); //Write Genertate check sum command to CT360
	    mdelay(5); //Delay 1mS
	    
	//				I2C_Buf[0] = 0x00;
	//				ct360_write_regs(ts->client,I2C_Buf,1); 
	//				mdelay(1);
	
	    ct360_read_regs(ts->client,I2C_Buf, 13); // Read check sum and flash dat	//Compare host check sum with CT360 check sum(I2C_Buf[4])
	    if (I2C_Buf[4] != CheckSum[i] )
	        return 0;
	        
        Flash_Address += 8;

	}
}
		boot_printk("%s.....7..Flash_Address=%4x.\n",__FUNCTION__,Flash_Address);
		ts->client->addr = 0x01;
		boot_printk("%s.....7...%d\n",__FUNCTION__,ts->client->addr);
					return 1; // Boot loader function is completed.

		
	} 



static int ct360_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ct360_ts_data *ts;
	struct ct360_platform_data	*pdata = client->dev.platform_data;
    int liRet = 0;
	int ret,i;
	char loader_buf[3] = {0xfF,0x3f,0xff};
	char boot_buf = 0;
	char boot_loader[2] = {0};
    ct360printk("%s \n",__FUNCTION__);
	
    if (!pdata)
	{
		dev_err(&client->dev, "empty platform_data\n");
		goto err_check_functionality_failed;
    }
	else
	{
		//nothing
	}
	
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
        printk(KERN_ERR "ct360_ts_probe: need I2C_FUNC_I2C\n");
        liRet = -ENODEV;
        goto err_check_functionality_failed;
    }
	else
	{
		//nothing
	}
	
    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL)
	{
        liRet = -ENOMEM;
        goto err_alloc_data_failed;
    }
/*
	if(pdata->hw_init)
		{
			liRet = pdata->hw_init();
			if(liRet!=0)
				goto err_input_dev_alloc_failed;
		}
*/	
	if(pdata->shutdown)
		{
		    printk("%s: ct36x reset\n", __func__);
			pdata->shutdown(1);
			mdelay(50);
			pdata->shutdown(0);
			mdelay(20);
			pdata->shutdown(1);
			mdelay(500);
		}

	ts->ct360_wq = create_singlethread_workqueue("ct360_wq");
			if (!ts->ct360_wq)
			{
				printk(KERN_ERR"%s: create workqueue failed\n", __func__);
				liRet = -ENOMEM;
				goto err_input_dev_alloc_failed;
			}
		
	INIT_WORK(&ts->work, ct360_ts_work_func);
		
	ts->client = client;
	i2c_set_clientdata(client, ts);
    

	#if 1
	//加40ms延时，否则读取出错。。
	mdelay(20);
	mdelay(20);


	printk("\n--%s-- Read FW Version command !!!\n",__FUNCTION__);
	ret=ct360_write_regs(client,loader_buf, 3);
	mdelay(10);
	//if(ret<0){
	//	is_ct360  = 0;
	//	printk("\n--%s--Set Register values error !!!\n",__FUNCTION__);
	//	//goto err_input_dev_alloc_failed;
	//	kfree(ts);
	//	return -1;
	//}


	//printk("%s...........%d\n",__FUNCTION__,boot_buf);
	printk("\n--%s-- Reset I2C address offset !!!\n",__FUNCTION__);
	ret = ct360_write_regs(client,boot_loader,1);
	mdelay(10);
	//if(ret < 0)
	//	printk("ct360_ts_probe:sdf  i2c_transfer fail =%d\n",ret);
	//else
	//	printk("%s.............ok\n",__FUNCTION__);	



	printk("\n--%s-- Read FW Version !!!\n",__FUNCTION__);
	ret = ct360_read_regs(client,&boot_buf,1);
	mdelay(10);
	printk("\n--%s-- FW Ver: 0x%x!!!\n",__FUNCTION__, boot_buf);
	//if(ret < 0)
	//	printk("ct360_ts_probe:i2c_transfer fail =%d\n",ret);
	//else
	//	printk("%s.............boot_buf=%d\n",__FUNCTION__,boot_buf);



	if(Binary_Data[32756]!=boot_buf)
	{
		printk("start Bootloader ...........boot_Buf=%x.....%d......%x..........TP \n\n",boot_buf,(Binary_Data[32756]-boot_buf),Binary_Data[32756]);
		ret = CT360_CTP_BootLoader(ts);
		if (ret == 1)
			printk("TP Bootloader success\n");
		else
			printk("TP Bootloader failed  ret=%d\n",ret);
		printk("stop Bootloader.................................TP \n\n");
	}
	else
	{
		printk("Don't need bootloader.skip it %x \n",Binary_Data[16372]);
	}

	ts->client->addr = 0x01;
	if(pdata->shutdown){
		pdata->shutdown(1);
		mdelay(5);
		pdata->shutdown(0);
		mdelay(20);
		pdata->shutdown(1);
		mdelay(30);
	}

	ts->client->addr = 0x01;
	#endif

/*
	setup_timer(&ts->timer, ct360_irq_timer, (unsigned long)ts);
		ts->timer.expires  = jiffies + 20;
		add_timer(&ts->timer);
*/
	
    /* allocate input device */
    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL)
	{
        liRet = -ENOMEM;
        printk(KERN_ERR "%s: Failed to allocate input device\n",__FUNCTION__);
        goto err_input_dev_alloc_failed;
    }
	else
	{
		//noghing
	}
	
	ts->input_dev->name			= ct360_TS_NAME;
	ts->input_dev->phys			= "I2C";

	ts->x_max	= pdata->x_max;
	ts->y_max	= pdata->y_max;

	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	__set_bit(EV_ABS, ts->input_dev->evbit);

	input_mt_init_slots(ts->input_dev, TOUCH_NUMBER);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->x_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->y_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0); //Finger Size
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, TOUCH_NUMBER, 0, 0); //Touch Size
	//input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0)
    liRet = input_register_device(ts->input_dev);
    if (liRet)
	{
        printk(KERN_ERR "%s: Unable to register %s input device\n", __FUNCTION__,ts->input_dev->name);
        goto err_input_register_device_failed;
    }
	else
	{
		//nothing
	}
	pdata->init_gpio();
		printk("xiehui debug ct360 init_gpio\n");
	ts->irq = client->irq;
	liRet = request_irq(ts->irq, ct360_ts_irq_handler,IRQF_DISABLED, client->name, ts);
	if (liRet<0)
	{
		printk("ct360 request_irq failed(err=%d, irq=%d)\n",liRet, ts->irq);
		goto err_input_register_device_failed;
	}
	
	
#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level		= EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend	= ct360_ts_early_suspend;
    ts->early_suspend.resume	= ct360_ts_early_resume;
    register_early_suspend(&ts->early_suspend);
#endif

    ct360printk( "%s: probe ok!!\n", __FUNCTION__);

    return 0;

err_input_register_device_failed:
    input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	//pdata->hw_exit();
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	
    return -ENODEV;
}

static int ct360_ts_remove(struct i2c_client *client)
{
    struct ct360_ts_data *ts = i2c_get_clientdata(client);
	
    unregister_early_suspend(&ts->early_suspend);
	free_irq(ts->irq, ts);
    input_unregister_device(ts->input_dev);
	
    if (ts->ct360_wq)
    {
        destroy_workqueue(ts->ct360_wq);
    }
	else
	{
		//nothing
	}

    kfree(ts);
    return 0;
}

static int ct360_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct ct360_ts_data *ts = i2c_get_clientdata(client);

    char buf[3]	= {0xff,0x8f,0xff};
	char buf1[2]={0x00,0xaf};
	int liRet;
	
    printk("ct360 TS Suspend\n");
	
    //disable_irq(ts->irq);
    free_irq(ts->irq,ts);
	cancel_work_sync(&ts->work);
	flush_work(&ts->work);

	
	liRet = ct360_write_regs(client,buf,3);
	if(liRet<0)
	{
		printk("ct360_ts supend fail!\n");
	}
	
	mdelay(1);
	liRet = ct360_write_regs(client,buf1,2);
	if(liRet<0)
	{
		printk("ct360_ts supend fail\n");
	}
	
	
    return 0;
}

static int ct360_ts_resume(struct i2c_client *client)
{
    struct ct360_ts_data *ts = i2c_get_clientdata(client);
	struct ct360_platform_data	*pdata = client->dev.platform_data;
	int liNumber = 0;
    int ret;
    printk("ct360 TS Resume\n");

	//if(pdata->hw_init)
	//{
	//	pdata->hw_init();
	//}
//for(liNumber = 0;liNumber < TOUCH_NUMBER; liNumber++)
	for(liNumber = 0;liNumber < 10; liNumber++)
	{	
	//	if(giDownIdNumber[liNumber]==1)
		{
			ct360printk("point %d: up      +ct360_ts_resume\n", liNumber);
			input_mt_slot(ts->input_dev, liNumber);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
			input_sync(ts->input_dev);
		}
		
	}

	if(pdata->shutdown)
	{
		pdata->shutdown(1);
		mdelay(5);
		pdata->shutdown(0);
		mdelay(20);
		pdata->shutdown(1);
		mdelay(5);
	}

#if 1
		ret = request_irq(ts->irq, ct360_ts_irq_handler, IRQF_DISABLED, client->name, ts);
		if (ret){
			printk("!!! ct360 request_irq failed\n");
		}
#endif
	msleep(50);

//	enable_irq(ts->irq);
	
	//for(liNumber = 0;liNumber < TOUCH_NUMBER; liNumber++)
	for(liNumber = 0;liNumber < 10; liNumber++)
	{	
	//	if(giDownIdNumber[liNumber]==1)
		{
			ct360printk("point %d: up      +ct360_ts_resume\n", liNumber);
			input_mt_slot(ts->input_dev, liNumber);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
			input_sync(ts->input_dev);
		}
		
	}
	
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ct360_ts_early_suspend(struct early_suspend *h)
{
    struct ct360_ts_data *ts;
    ts = container_of(h, struct ct360_ts_data, early_suspend);
    ct360_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void ct360_ts_early_resume(struct early_suspend *h)
{
    struct ct360_ts_data *ts;
    ts = container_of(h, struct ct360_ts_data, early_suspend);
    ct360_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id ct360_ts_id[] = {
    { ct360_TS_NAME, 0 },
    { }
};

static struct i2c_driver ct360_ts_driver = {
    .probe      = ct360_ts_probe,
    .remove     = ct360_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = ct360_ts_suspend,
    .resume     = ct360_ts_resume,
#endif
    .id_table   = ct360_ts_id,
    .driver		= {
		.name   = ct360_TS_NAME,
    },
};

static int __devinit ct360_ts_init(void)
{
    
      int ret;
      ct360printk("%s\n",__FUNCTION__);
      ret=i2c_add_driver(&ct360_ts_driver);
      printk("ret=%d\n",ret);
      return ret;
}

static void __exit ct360_ts_exit(void)
{
    ct360printk("%s\n",__FUNCTION__);
    i2c_del_driver(&ct360_ts_driver);
}

module_init(ct360_ts_init);
module_exit(ct360_ts_exit);

MODULE_DESCRIPTION("ct360 Touchscreen Driver");
MODULE_LICENSE("GPL");
