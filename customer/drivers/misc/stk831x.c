/*
 *  stk831x.c - Linux kernel modules for sensortek stk8311/stk8312/stk8313 accelerometer
 *
 *  Copyright (C) 2011~2012 Lex Hsieh / sensortek 
 * <lex_hsieh@sitronix.com.tw>  or <lex_hsieh@sensortek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */   
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h> 
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/pm_runtime.h>
#include   <linux/fs.h>   
#include  <asm/uaccess.h>
#define STK_PERMISSION_THREAD
#ifdef STK_PERMISSION_THREAD
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#endif 	//	#ifdef STK_PERMISSION_THREAD

#ifdef CONFIG_SENSORS_STK8313
	#include <linux/stk8313.h>
#elif defined CONFIG_SENSORS_STK8312
	#include <linux/stk8312.h>
#else
	#error "What's your stk accelerometer?"
#endif

#define STK_ACC_DRIVER_VERSION	"1.5.6"
/*choose polling or interrupt mode*/
#define STK_ACC_POLLING_MODE	1
#if (!STK_ACC_POLLING_MODE)
	#define ADDITIONAL_GPIO_CFG 1
	#define STK_INT_PIN	39
#endif

static struct i2c_client *this_client;

struct stk831x_data 
{
	struct input_dev *input_dev;
	struct work_struct stk_work;
	int irq;	
	int raw_data[3]; 
	atomic_t enabled;
	unsigned char delay;	
	struct mutex read_lock;
	bool first_enable;
#if STK_ACC_POLLING_MODE
	atomic_t run_thread;
#endif	//#if STK_ACC_POLLING_MODE
};

const static int STK831X_SAMPLE_TIME[6] = {2500, 5000, 10000, 20000, 40000, 80000};
static struct stk831x_data *stk831x_data_ptr;
static unsigned char event_since_en = 0;
static char cali_offset[3] = {0};
#if STK_ACC_POLLING_MODE
static struct completion acc_thread_completion;
static struct task_struct *acc_polling_tsk=NULL;
static int stk_acc_polling_function(void *arg);
#else
static struct workqueue_struct *stk_mems_work_queue = NULL;
#endif	//#if STK_ACC_POLLING_MODE

//#define STK_DEBUG_CALI
#define STK_SAMPLE_NO	10
//#define STORE_OFFSET_IN_FILE
#define STORE_OFFSET_IN_IC
#define STK_ACC_CALI_VER0			0x3D
#define STK_ACC_CALI_VER1			0x01
#define STK_ACC_CALI_FILE "/data/misc/stk_acc_cali.conf"
#define STK_ACC_CALI_FILE_SIZE 6

#define POSITIVE_Z_UP		0
#define NEGATIVE_Z_UP	1
#define POSITIVE_X_UP		2
#define NEGATIVE_X_UP	3
#define POSITIVE_Y_UP		4
#define NEGATIVE_Y_UP	5
static unsigned char stk831x_placement = POSITIVE_Z_UP;

static int stk_store_in_ic( struct stk831x_data *stk, char offset[], char otp_offset[], char FT_index, unsigned char stk831x_placement);
static int32_t stk_get_file_content(char * r_buf, int8_t buf_size);
static int stk_store_in_file(char offset[]);

static int STK_i2c_Rx(char *rxData, int length)
{
	if( i2c_master_send(this_client, rxData, 1) < 0)
		return -EIO;
	
	if (i2c_master_recv(this_client, rxData, length) < 0)
		return -EIO;

	return 0;
}

static int STK_i2c_Tx(char *txData, int length)
{
	unsigned char w_twice = 0;
	if(*txData >= 0x21 && *txData <= 0x3F)
		w_twice = 1;
	if(i2c_master_send(this_client, txData, length) < 0)
		return -EIO;
	if(w_twice)
	{
		if(i2c_master_send(this_client, txData, length) < 0)
			return -EIO;		
	}		
	return 0;	
}


static int STK831X_SetVD(struct stk831x_data *stk)
{
	int result;
	char buffer[2] = "";
	char reg24;
	unsigned char retry;
		
	for(retry=0;retry<5;retry++)
	{
		buffer[0] = 0x3D;
		buffer[1] = 0x70;
		result = STK_i2c_Tx(buffer, 2);
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			return result;
		}	

		buffer[0] = 0x3F;
		buffer[1] = 0x02;
		result = STK_i2c_Tx(buffer, 2);
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			return result;
		}	
			
		buffer[0] = 0x3E;
		result = STK_i2c_Rx(buffer, 2);	
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			return result;
		}		
		
		if(buffer[1]>>7 == 0)
			msleep(1);		
		else
			break;
	}
	reg24 = buffer[0];
	
	if(reg24 != 0)
	{
		buffer[0] = 0x24;
		buffer[1] = reg24;
		//printk(KERN_INFO "%s:write 0x%x to 0x24\n",  __func__, buffer[1]);
		result = STK_i2c_Tx(buffer, 2);
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			return result;
		}
	}	
	else
	{
		//printk(KERN_INFO "%s: reg24=0, do nothing\n", __func__);
		return 0;
	}
	
	buffer[0] = 0x24;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}				
	if(buffer[0] != reg24)
	{
		printk(KERN_ERR "%s: error, reg24=0x%x, read=0x%x\n", __func__, reg24, buffer[0]);
		return -1;
	}
	//printk(KERN_INFO "%s: successfully", __func__);
	return 0;
}


#ifdef CONFIG_SENSORS_STK8312
static int STK831x_ReadSensorData(struct stk831x_data *stk)
{	
	int result;
	char buffer[3] = "";
	memset(buffer, 0, 3);
	
	buffer[0] = STK831X_XOUT;
	result = STK_i2c_Rx(buffer, 3);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:i2c transfer error", __func__);
		return result;
	}	
	mutex_lock(&stk->read_lock);
	if (buffer[0] & 0x80)
		stk->raw_data[0] = buffer[0] - 256;
	else
		stk->raw_data[0] = buffer[0];
	if (buffer[1] & 0x80)
		stk->raw_data[1] = buffer[1] - 256;
	else
		stk->raw_data[1] = buffer[1];
	if (buffer[2] & 0x80)
		stk->raw_data[2] = buffer[2] - 256;
	else
		stk->raw_data[2] = buffer[2];
	mutex_unlock(&stk->read_lock);
	
	return 0;	
}
#elif defined CONFIG_SENSORS_STK8313
static int STK831x_ReadSensorData(struct stk831x_data *stk)
{	
	int result;
	char buffer[6] = "";
	memset(buffer, 0, 6);
	
	buffer[0] = STK831X_XOUT;
	result = STK_i2c_Rx(buffer, 6);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:i2c transfer error", __func__);
		return result;
	}			
	mutex_lock(&stk->read_lock);		
	if (buffer[0] & 0x80)
		stk->raw_data[0] = ((int)buffer[0]<<4) + (buffer[1]>>4) - 4096;
	else
		stk->raw_data[0] = ((int)buffer[0]<<4) + (buffer[1]>>4);
	if (buffer[2] & 0x80)
		stk->raw_data[1] = ((int)buffer[2]<<4) + (buffer[3]>>4) - 4096;
	else
		stk->raw_data[1] = ((int)buffer[2]<<4) + (buffer[3]>>4);
	if (buffer[4] & 0x80)
		stk->raw_data[2] = ((int)buffer[4]<<4) + (buffer[5]>>4) - 4096;
	else
		stk->raw_data[2] = ((int)buffer[4]<<4) + (buffer[5]>>4);
	mutex_unlock(&stk->read_lock);
	
	return 0;	
}
#endif

static int STK831x_ReportValue(struct stk831x_data *stk)
{
	if(event_since_en < 12)
	{
		event_since_en++;
		return 0;
	}
	
	mutex_lock(&stk->read_lock);
	//printk(KERN_INFO "%s:%4d,%4d,%4d\n", __func__, stk->raw_data[0], stk->raw_data[1], stk->raw_data[2]);	
	input_report_abs(stk->input_dev, ABS_X, stk->raw_data[0]);  
	input_report_abs(stk->input_dev, ABS_Y, stk->raw_data[1]);
	input_report_abs(stk->input_dev, ABS_Z, stk->raw_data[2]);
	mutex_unlock(&stk->read_lock);
	input_sync(stk->input_dev);
	return 0;
}

static int STK831x_SetOffset(char buf[])
{
	int result;
	char buffer[4] = "";
	
	buffer[0] = STK831X_OFSX;	
	buffer[1] = buf[0];
	buffer[2] = buf[1];
	buffer[3] = buf[2];
	result = STK_i2c_Tx(buffer, 4);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed", __func__);
		return result;
	}	
	return 0;
}

static int STK831x_GetOffset(char buf[])
{
	int result;
	char buffer[3] = "";
	
	buffer[0] = STK831X_OFSX;
	result = STK_i2c_Rx(buffer, 3);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}		
	buf[0] = buffer[0];
	buf[1] = buffer[1];
	buf[2] = buffer[2];
	return 0;
}

static int STK831x_SetEnable(struct stk831x_data *stk, char en)
{
	int result;
	char buffer[2] = "";
	int new_enabled = (en)?1:0; 
	char r_buf[STK_ACC_CALI_FILE_SIZE] = {0};
	char offset[3];	
	if(stk->first_enable)
	{
		stk->first_enable = false;		
		if ((stk_get_file_content(r_buf, STK_ACC_CALI_FILE_SIZE)) == 0)
		{
			if(r_buf[0] == STK_ACC_CALI_VER0 && r_buf[1] == STK_ACC_CALI_VER1)
			{
				offset[0] = r_buf[2];
				offset[1] = r_buf[3];
				offset[2] = r_buf[4];
				cali_offset[0] = offset[0];
				cali_offset[1] = offset[1];
				cali_offset[2] = offset[2];
				STK831x_SetOffset(offset);
				printk(KERN_INFO "%s: set offset:%d,%d,%d\n", __func__, offset[0], offset[1], offset[2]);
			}
			else
			{
				printk(KERN_ERR "%s: cali version number error! r_buf=0x%x,0x%x,0x%x,0x%x\n", 
					__func__, r_buf[0], r_buf[1], r_buf[2], r_buf[3]);						
				return -EINVAL;
			}
		}
		else
		{
			offset[0] = offset[1] = offset[2] = 0;
			stk_store_in_file(offset);
		}
	}
	
	if(new_enabled == atomic_read(&stk->enabled))
		return 0;
	printk(KERN_INFO "%s:%x\n", __func__, en);


	if(en)
	{
#if STK_ACC_POLLING_MODE
		atomic_set(&stk->run_thread, 1);
		if(acc_polling_tsk == NULL)
			acc_polling_tsk = kthread_run(stk_acc_polling_function, stk, "stk_acc_polling");		
#else
		enable_irq((unsigned int)stk->irq);	
#endif	//#if STK_ACC_POLLING_MODE	
	}
	else
	{
#if STK_ACC_POLLING_MODE
		atomic_set(&stk->run_thread, 0);
		wait_for_completion(&acc_thread_completion);
		acc_polling_tsk = NULL;
#else
		disable_irq((unsigned int)stk->irq);	
#endif	//#if STK_ACC_POLLING_MODE
	}
	
	buffer[0] = STK831X_MODE;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}			
	if(en)
	{
		buffer[1] = (buffer[0] & 0xF8) | 0x01;
		event_since_en = 0;
	}
	else
		buffer[1] = (buffer[0] & 0xF8);
	buffer[0] = STK831X_MODE;	
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed", __func__);
		return result;
	}
	if(en)
	{
		msleep(1);
		STK831X_SetVD(stk);
	}
	atomic_set(&stk->enabled, new_enabled);
	return 0;
}

static int STK831x_GetEnable(struct stk831x_data *stk, char* gState)
{

	*gState = atomic_read(&stk->enabled);
	return 0;
}

static int STK831x_SetDelay(struct stk831x_data *stk, uint32_t sdelay_ns)
{
	unsigned char sr_no;	
	int result;
	char buffer[2] = "";
	uint32_t sdelay_us = sdelay_ns / 1000;
	
	printk(KERN_INFO "%s:sdelay_us=%d\n", __func__, sdelay_us);
	for(sr_no=5;sr_no>0;sr_no--)
	{
		if(sdelay_us >= STK831X_SAMPLE_TIME[sr_no])	
			break;		
	}	
	if(stk->delay == sr_no)
		return 0;

	buffer[0] = STK831X_SR;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}			
	
	buffer[1] = (buffer[0] & 0xF8) | (sr_no & 0x07);
	buffer[0] = STK831X_SR;	
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed", __func__);
		return result;
	}	
	stk->delay = sr_no;
	
	return 0;	
}

static int STK831x_GetDelay(struct stk831x_data *stk, uint32_t* gdelay_ns)
{
	*gdelay_ns = (uint32_t) STK831X_SAMPLE_TIME[stk->delay] * 1000;
	return 0;	
}


static int STK831x_SetRange(char srange)
{
	int result;
	char buffer[2] = "";
	printk(KERN_INFO "%s:range=0x%x\n", __func__, srange);
	buffer[0] = STK831X_STH;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}	
	
	if(srange >= 3)
	{
		printk(KERN_ERR "%s:parameter out of range\n", __func__);
		return -1;
	}
	
	buffer[1] = (buffer[0] & 0x3F) | srange<<6;
	buffer[0] = STK831X_STH;	
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed", __func__);
		return result;
	}	
	return 0;	
}

static int STK831x_GetRange(char* grange)
{
	int result;
	char buffer = 0;
	
	buffer = STK831X_STH;
	result = STK_i2c_Rx(&buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}		
	*grange = buffer >> 6;
	return 0;
}

#ifdef STK_DEBUG_CALI		
static int STK831x_ReadOTP(char addr)
{
	int result, redo, i;
	char buffer[2] = "";
	char regR[20] = "";
	char piAddr;
	for( i=0; i<3; i++)
	{
		redo = 0;

		do {

			redo++;

			buffer[0] = 0x3D;
			buffer[1] = addr+i;
			result = STK_i2c_Tx(buffer, 2);
			if (result < 0) 
			{
				printk(KERN_ERR "%s:failed\n", __func__);
				return result;
			}
			
			buffer[0] = 0x3F;
			buffer[1] = 0x02;
			result = STK_i2c_Tx(buffer, 2);
			if (result < 0) 
			{
				printk(KERN_ERR "%s:failed\n", __func__);
				return result;
			}
			
			for(piAddr=0x3D; piAddr<0x40; piAddr++)
			{		
				buffer[0] = piAddr;
				result = STK_i2c_Rx(buffer, 1);	
				if (result < 0) 
				{
					printk(KERN_ERR "%s:failed\n", __func__);
					return result;
				}
				else
				{
					regR[piAddr-0x3D+i*3] = buffer[0];
					
				}
			}
		}
		while((regR[2+i*3]>>7) != 1 && redo < 10);

		if((regR[2+i*3]>>7) == 0 && redo == 10)
		{
			printk(KERN_ERR "%s:OTP repeat read 3 times! Failed!\n", __func__);
			return -1;
		}
	}
	printk(KERN_INFO "%s: OTP x%x=0x%x\n", __func__, addr, regR[1+0]);
	printk(KERN_INFO "%s: OTP x%x=0x%x\n", __func__, addr+1, regR[1+3]);
	printk(KERN_INFO "%s: OTP x%x=0x%x\n", __func__, addr+2, regR[1+6]);
	
	return 0;
}

static int STK831x_ReadAllOTP( void )
{
	int result, i;
	char buffer[2] = "";
	char mode; 
	
	buffer[0] = STK831X_MODE;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}
	mode = buffer[0];
	buffer[1] = (mode | 0x01);
	buffer[0] = STK831X_MODE;	
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}
	msleep(1);
	
	for(i=0;i<3;i++)
	{
		buffer[0] = 0x2A+i*4;
		result = STK_i2c_Rx(buffer, 1);	
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			return result;
		}
		printk(KERN_INFO "%s: REG 0x%x=0x%x\n", __func__, 0x2A+i*4, buffer[0]);
		buffer[0] = 0x2B+i*4;
		result = STK_i2c_Rx(buffer, 1);	
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			return result;
		}
		printk(KERN_INFO "%s: REG 0x%x=0x%x\n", __func__, 0x2B+i*4, buffer[0]);
	}
	STK831x_ReadOTP(0x30);
	STK831x_ReadOTP(0x40);
	STK831x_ReadOTP(0x43);
	STK831x_ReadOTP(0x46);
	STK831x_ReadOTP(0x50);
	STK831x_ReadOTP(0x53);
	STK831x_ReadOTP(0x56);
	return 0;
}
#endif

static int STK831x_ReadByteOTP(char rReg, char *value)
{
	int redo = 0;
	int result;
	char buffer[2] = "";
	*value = 0;
	
	buffer[0] = 0x3D;
	buffer[1] = rReg;
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}
	buffer[0] = 0x3F;
	buffer[1] = 0x02;
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}
	
	msleep(1);	
	do {
		buffer[0] = 0x3F;
		result = STK_i2c_Rx(buffer, 1);	
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			return result;
		}
		if(buffer[0]& 0x80)
		{
			break;
		}		
		msleep(1);
		redo++;
	}while(redo < 5);
		
	if(redo == 5)
	{
		printk(KERN_ERR "%s:OTP read repeat read 5 times! Failed!\n", __func__);
		return -1;
	}	
	buffer[0] = 0x3E;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}	
	*value = buffer[0];
#ifdef STK_DEBUG_CALI		
	printk(KERN_INFO "%s: read 0x%x=0x%x", __func__, rReg, *value);
#endif	
	return 0;
}

static int STK831x_WriteByteOTP(char wReg, char value)
{
	int redo = 0;
	int result;
	char buffer[2] = "";
	char read_back;
	
	redo = 0;
	do {
		redo++;
		
		buffer[0] = 0x3D;
		buffer[1] = wReg;
		result = STK_i2c_Tx(buffer, 2);
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			return result;
		}
		buffer[0] = 0x3E;
		buffer[1] = value;
		result = STK_i2c_Tx(buffer, 2);
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			return result;
		}				
		buffer[0] = 0x3F;
		buffer[1] = 0x01;
		result = STK_i2c_Tx(buffer, 2);
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			return result;
		}				
		msleep(1);
		
		buffer[0] = 0x3F;
		result = STK_i2c_Rx(buffer, 1);	
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			return result;
		}
		if(buffer[0]& 0x80)
		{
			result = STK831x_ReadByteOTP(wReg, &read_back);
			if(result < 0)
			{
				printk(KERN_ERR "%s: read back error, result=%d", __func__, result);
				return result;
			}
			
			if(read_back == value)
			{
#ifdef STK_DEBUG_CALI					
				printk(KERN_INFO "%s: write 0x%x=0x%x successfully\n", __func__, wReg, value);
#endif				
				break;
			}
			else
				printk(KERN_ERR "%s: read back mismatch, write 0x%x=0x%x, read 0x%x=0x%x, try again\n", __func__, wReg, value, wReg, read_back);
		}
		msleep(1);
	}while(redo < 5);
	
	if(redo == 5)
	{
		printk(KERN_ERR "%s:OTP write 0x%x repeat read 5 times! Failed!\n", __func__, wReg);
		return -1;
	}		
	return 0;
}

static int STK831x_WriteOffsetOTP(struct stk831x_data *stk, int FT, char offsetData[])
{
	char regR[6];
	char mode; 
	int result;
	char buffer[2] = "";
	
//Check FT1
	if(FT==1)
	{
		result = STK831x_ReadByteOTP(0x7F, &regR[0]);
		if(result < 0)
			return -1;
		
		if(regR[0]&0x10)
		{
			printk(KERN_ERR "%s: 0x7F=0x%x\n", __func__, regR[0]);
			return -2;
		}
	}
	else if (FT == 2)
	{
		result = STK831x_ReadByteOTP(0x7F, &regR[0]);
		if(result < 0)
		return -1;
		
		if(regR[0]&0x20)
		{
			printk(KERN_ERR "%s: 0x7F=0x%x\n", __func__, regR[0]);
			return -2;
		}		
	}
//Check End
	
	buffer[0] = STK831X_MODE;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}
	mode = buffer[0];
	buffer[1] = (mode | 0x01);
	buffer[0] = STK831X_MODE;	
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}
	msleep(1);
	
	result = STK831x_ReadByteOTP(0x30, &regR[0]);
	if(result < 0)
		return -1;
	result = STK831x_ReadByteOTP(0x31, &regR[1]);
	if(result < 0)
		return -1;
	result = STK831x_ReadByteOTP(0x32, &regR[2]);
	if(result < 0)
		return -1;
	
	if(FT == 1)
	{
		result = STK831x_WriteByteOTP(0x40, regR[0]);
		if(result < 0)
			return -1;	
		result = STK831x_WriteByteOTP(0x41, regR[1]);
		if(result < 0)
			return -1;	
		result = STK831x_WriteByteOTP(0x42, regR[2]);
		if(result < 0)
			return -1;	
	}
	else if (FT == 2)
	{
		result = STK831x_WriteByteOTP(0x50, regR[0]);
		if(result < 0)
			return -1;	
		result = STK831x_WriteByteOTP(0x51, regR[1]);
		if(result < 0)
			return -1;	
		result = STK831x_WriteByteOTP(0x52, regR[2]);
		if(result < 0)
			return -1;			
	}
#ifdef STK_DEBUG_CALI
	printk(KERN_INFO "%s:OTP step1 Success!\n", __func__);
#endif
	buffer[0] = 0x2A;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}
	else
	{
		regR[0] = buffer[0];
	}
	buffer[0] = 0x2B;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}
	else
	{
		regR[1] = buffer[0];
	}
	buffer[0] = 0x2E;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}
	else
	{
		regR[2] = buffer[0];
	}
	buffer[0] = 0x2F;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}
	else
	{
		regR[3] = buffer[0];
	}
	buffer[0] = 0x32;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}
	else
	{
		regR[4] = buffer[0];
	}
	buffer[0] = 0x33;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}
	else
	{
		regR[5] = buffer[0];
	}
	
	regR[1] = offsetData[0];
	regR[3] = offsetData[2];
	regR[5] = offsetData[1];
	if(FT==1)
	{
		result = STK831x_WriteByteOTP(0x43, regR[0]);
		if(result < 0)
			return -1;	
		result = STK831x_WriteByteOTP(0x44, regR[1]);
		if(result < 0)
			return -1;	
		result = STK831x_WriteByteOTP(0x45, regR[2]);
		if(result < 0)
			return -1;	
		result = STK831x_WriteByteOTP(0x46, regR[3]);
		if(result < 0)
			return -1;	
		result = STK831x_WriteByteOTP(0x47, regR[4]);
		if(result < 0)
			return -1;	
		result = STK831x_WriteByteOTP(0x48, regR[5]);
		if(result < 0)
			return -1;			
	}
	else if (FT == 2)
	{
		result = STK831x_WriteByteOTP(0x53, regR[0]);
		if(result < 0)
			return -1;	
		result = STK831x_WriteByteOTP(0x54, regR[1]);
		if(result < 0)
			return -1;	
		result = STK831x_WriteByteOTP(0x55, regR[2]);
		if(result < 0)
			return -1;	
		result = STK831x_WriteByteOTP(0x56, regR[3]);
		if(result < 0)
			return -1;	
		result = STK831x_WriteByteOTP(0x57, regR[4]);
		if(result < 0)
			return -1;	
		result = STK831x_WriteByteOTP(0x58, regR[5]);
		if(result < 0)
			return -1;			
	}
#ifdef STK_DEBUG_CALI	
	printk(KERN_INFO "%s:OTP step2 Success!\n", __func__);
#endif
	result = STK831x_ReadByteOTP(0x7F, &regR[0]);
	if(result < 0)
		return -1;	
	
	if(FT==1)
		regR[0] = regR[0]|0x10;
	else if(FT==2)
		regR[0] = regR[0]|0x20;

	result = STK831x_WriteByteOTP(0x7F, regR[0]);
	if(result < 0)
		return -1;	
#ifdef STK_DEBUG_CALI	
	printk(KERN_INFO "%s:OTP step3 Success!\n", __func__);
#endif	
	return 0;
}

static int STK831X_VerifyCali(struct stk831x_data *stk, unsigned char en_dis)
{
	unsigned char axis, state;	
	int acc_ave[3] = {0, 0, 0};
	const unsigned char verify_sample_no = 2, verify_diff = 2;	
	int result;
	char buffer[2] = "";
	int ret = 0;
	
	if(en_dis)
	{
		STK831x_SetDelay(stk, 10000000);
		buffer[0] = STK831X_MODE;
		result = STK_i2c_Rx(buffer, 1);	
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			return result;
		}			
		buffer[1] = (buffer[0] & 0xF8) | 0x01;
		buffer[0] = STK831X_MODE;	
		result = STK_i2c_Tx(buffer, 2);
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed", __func__);
			return result;
		}
		msleep(1);
		STK831X_SetVD(stk);			
		msleep(150);	
	}
	
	for(state=0;state<verify_sample_no;state++)
	{
		STK831x_ReadSensorData(stk);
		mutex_lock(&stk->read_lock);
		for(axis=0;axis<3;axis++)			
			acc_ave[axis] += stk->raw_data[axis];	
#ifdef STK_DEBUG_CALI				
		printk(KERN_INFO "%s: acc=%d,%d,%d\n", __func__, stk->raw_data[0], stk->raw_data[1], stk->raw_data[2]);	
#endif
		mutex_unlock(&stk->read_lock);
		msleep(10);		
	}		
	
	for(axis=0;axis<3;axis++)
		acc_ave[axis] /= verify_sample_no;
	
	switch(stk831x_placement)
	{
	case POSITIVE_X_UP:
		acc_ave[0] -= STK_LSB_1G;
		break;
	case NEGATIVE_X_UP:
		acc_ave[0] += STK_LSB_1G;		
		break;
	case POSITIVE_Y_UP:
		acc_ave[1] -= STK_LSB_1G;
		break;
	case NEGATIVE_Y_UP:
		acc_ave[1] += STK_LSB_1G;
		break;
	case POSITIVE_Z_UP:
		acc_ave[2] -= STK_LSB_1G;
		break;
	case NEGATIVE_Z_UP:
		acc_ave[2] += STK_LSB_1G;
		break;
	default:
		printk("%s: invalid stk831x_placement=%d", __func__, stk831x_placement);
		ret = -1;
		break;
	}	
	if(abs(acc_ave[0]) > verify_diff || abs(acc_ave[1]) > verify_diff || abs(acc_ave[2]) > verify_diff)
	{
		printk(KERN_INFO "%s:Check data x:%d, y:%d, z:%d\n", __func__,acc_ave[0],acc_ave[1],acc_ave[2]);		
		printk(KERN_ERR "%s:Check Fail, Calibration Fail\n", __func__);
		ret = -2;
	}	
#ifdef STK_DEBUG_CALI
	else
		printk(KERN_INFO "%s:Check data pass\n", __func__);
#endif	
	if(en_dis)
	{
		buffer[0] = STK831X_MODE;
		result = STK_i2c_Rx(buffer, 1);	
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			return result;
		}			
		buffer[1] = (buffer[0] & 0xF8);
		buffer[0] = STK831X_MODE;	
		result = STK_i2c_Tx(buffer, 2);
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed", __func__);
			return result;
		}		
	}	
	
	return ret;
}


static int STK831x_SetCali(struct stk831x_data *stk, char sstate)
{
	char org_enable;
	int acc_ave[3] = {0, 0, 0};
	int state, axis, i;
	char new_offset[3],  otp_offset[3] = {0};
	int result;
	char buffer[2] = "";
	char reg_offset[3] = {0};
	char store_location = sstate;

	//sstate=1, STORE_OFFSET_IN_FILE
	//sstate=2, STORE_OFFSET_IN_IC		
#ifdef STK_DEBUG_CALI		
	printk(KERN_INFO "%s:store_location=%d, stk831x_placement=%d\n", __func__, store_location, stk831x_placement);
#endif	
	if((store_location != 3 && store_location != 2 && store_location != 1) || (stk831x_placement < 0 || stk831x_placement > 5) )
	{
		printk(KERN_ERR "%s, erro invalid parameters\n", __func__);
		return -EINVAL;
	}	
	
	STK831x_GetEnable(stk, &org_enable);
	if(org_enable)
		STK831x_SetEnable(stk, 0);
	STK831x_SetDelay(stk, 10000000);
	STK831x_SetRange(1);				
	STK831x_SetOffset(reg_offset);
	buffer[0] = STK831X_MODE;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}			
	buffer[1] = (buffer[0] & 0xF8) | 0x01;
	buffer[0] = STK831X_MODE;	
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed", __func__);
		return result;
	}
	msleep(1);
	STK831X_SetVD(stk);

	if(store_location >= 2)
	{
		buffer[0] = 0x2B;	
		buffer[1] = 0x0;
		result = STK_i2c_Tx(buffer, 2);
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed", __func__);
			return result;
		}
		buffer[0] = 0x2F;	
		buffer[1] = 0x0;
		result = STK_i2c_Tx(buffer, 2);
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed", __func__);
			return result;
		}
		buffer[0] = 0x33;	
		buffer[1] = 0x0;
		result = STK_i2c_Tx(buffer, 2);
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed", __func__);
			return result;
		}
	}		
	
	msleep(150);				
	for(state=0;state<STK_SAMPLE_NO;state++)
	{
		STK831x_ReadSensorData(stk);
		mutex_lock(&stk->read_lock);
		for(axis=0;axis<3;axis++)			
			acc_ave[axis] += stk->raw_data[axis];	
#ifdef STK_DEBUG_CALI				
		printk(KERN_INFO "%s: acc=%d,%d,%d\n", __func__, stk->raw_data[0], stk->raw_data[1], stk->raw_data[2]);	
#endif		
		mutex_unlock(&stk->read_lock);
		msleep(10);		
	}		
	buffer[0] = STK831X_MODE;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}			
	buffer[1] = (buffer[0] & 0xF8);
	buffer[0] = STK831X_MODE;	
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed", __func__);
		return result;
	}	
	
	for(axis=0;axis<3;axis++)
		acc_ave[axis] /= STK_SAMPLE_NO;
	
	if(acc_ave[2]<=0)
		stk831x_placement = NEGATIVE_Z_UP;
	else
		stk831x_placement = POSITIVE_Z_UP;

	switch(stk831x_placement)
	{
	case POSITIVE_X_UP:
		acc_ave[0] -= STK_LSB_1G;
		break;
	case NEGATIVE_X_UP:
		acc_ave[0] += STK_LSB_1G;		
		break;
	case POSITIVE_Y_UP:
		acc_ave[1] -= STK_LSB_1G;
		break;
	case NEGATIVE_Y_UP:
		acc_ave[1] += STK_LSB_1G;
		break;
	case POSITIVE_Z_UP:
		acc_ave[2] -= STK_LSB_1G;
		break;
	case NEGATIVE_Z_UP:
		acc_ave[2] += STK_LSB_1G;
		break;
	default:
		printk("%s: invalid stk831x_placement=%d", __func__, stk831x_placement);
		return -1;
		break;
	}		
	for(axis=0;axis<3;axis++)
	{
		acc_ave[axis] = -acc_ave[axis];
		new_offset[axis] = (char)acc_ave[axis];
	}				
#ifdef STK_DEBUG_CALI	
	printk(KERN_INFO "%s: New offset:%d,%d,%d\n", __func__, new_offset[0], new_offset[1], new_offset[2]);	
#endif	
	if(store_location == 1)
	{
		cali_offset[0] = new_offset[0];
		cali_offset[1] = new_offset[1];
		cali_offset[2] = new_offset[2];
		STK831x_SetOffset(new_offset);
		msleep(1);
		STK831x_GetOffset(reg_offset);
		for(axis=0;axis<3;axis++)
		{
			if(new_offset[axis] != reg_offset[axis])		
			{
				printk(KERN_ERR "%s: set offset to register fail!, new_offset[%d]=%d,reg_offset[%d]=%d\n",
					__func__, axis,new_offset[axis], axis, reg_offset[axis]);
							
				return -2;
			}
		}
	
		result = STK831X_VerifyCali(stk, 1);
		if(result)
			printk(KERN_ERR "%s: calibration check fail\n", __func__);
		else
		{
			stk_store_in_file(new_offset);
			printk(KERN_INFO "%s successfully\n", __func__);	
		}
	}
	else if(store_location >= 2)
	{
		for(i=0; i<3; i++)
		{
			if( (new_offset[i]>>7)==0)
			{
				if(new_offset[i] >= 0x20 )
				{
					printk(KERN_ERR "%s: offset[%d]=0x%x is too large, limit to 0x1f", __func__, i, new_offset[i] );
					otp_offset[i] = 0x1f;
				}
				else 
					otp_offset[i] = new_offset[i];
			}	
			else
			{
				if(new_offset[i] <= 0xDF)
				{
					printk(KERN_ERR "%s: offset[%d]=0x%x is too large, limit to 0x20", __func__, i, new_offset[i]);				
					otp_offset[i] = 0x20;
				}
				else
					otp_offset[i] = new_offset[i] & 0x3f;
			}
		}

		printk(KERN_INFO "%s: OTP offset:0x%x,0x%x,0x%x\n", __func__, otp_offset[0], otp_offset[1], otp_offset[2]);
		if(store_location == 2)
			result = stk_store_in_ic( stk, new_offset, otp_offset, 1, stk831x_placement);
		else if(store_location == 3)
			result = stk_store_in_ic( stk, new_offset, otp_offset, 2, stk831x_placement);
		
		if(result == 0)
			printk(KERN_INFO "%s successfully\n", __func__);
		else
			printk(KERN_ERR "%s fail, result=%d\n", __func__, result);
	}
	stk->first_enable = false;		
	if(org_enable)
		STK831x_SetEnable(stk, 1);		

	return 0;
}

static int STK831x_Init(struct stk831x_data *stk, struct i2c_client *client)
{
	int result;
	char buffer[2] = "";

#ifdef CONFIG_SENSORS_STK8312
	printk(KERN_INFO "%s: Initialize stk8312\n", __func__);
#elif defined CONFIG_SENSORS_STK8313
	printk(KERN_INFO "%s: Initialize stk8313\n", __func__);
#endif		
	
	buffer[0] = STK831X_RESET;
	buffer[1] = 0x00;
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}		

	buffer[0] = STK831X_MODE;
	buffer[1] = 0xC0;
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}		
	
	buffer[0] = STK831X_SR;
	buffer[1] = 0x04;	
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}	
	stk->delay = 0x04;

#if (!STK_ACC_POLLING_MODE)
	buffer[0] = STK831X_INTSU;
	buffer[1] = 0x10;
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:interrupt init failed\n", __func__);
		return result;
	}	
#endif 

	buffer[0] = STK831X_STH;
#ifdef CONFIG_SENSORS_STK8312	
	buffer[1] = 0x42;
#elif defined CONFIG_SENSORS_STK8313
	buffer[1] = 0x82;
#endif	
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:set range failed\n", __func__);	
		return result;
	}	
	
//	atomic_set(&stk->enabled, 0);	
#if STK_ACC_POLLING_MODE
	atomic_set(&stk->run_thread, 0);
#endif	//#if STK_ACC_POLLING_MODE				
	event_since_en = 0;
	return 0;
}

static int stk_store_in_ic( struct stk831x_data *stk, char offset[], char otp_offset[], char FT_index, unsigned char stk831x_placement)
{
	int result;
	char buffer[2] = "";

	buffer[0] = STK831X_MODE;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}			
	buffer[1] = (buffer[0] & 0xF8) | 0x01;
	buffer[0] = STK831X_MODE;	
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}		

	msleep(1);
	STK831X_SetVD(stk);
	
	buffer[0] = 0x2B;	
	buffer[1] = offset[0];
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}
	buffer[0] = 0x2F;	
	buffer[1] = offset[2];
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}
	buffer[0] = 0x33;	
	buffer[1] = offset[1];
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}		

#ifdef STK_DEBUG_CALI	
	printk(KERN_INFO "%s:Check All OTP Data after write 0x2B 0x2F 0x33\n", __func__);
	STK831x_ReadAllOTP();
#endif	
	
	msleep(150);		
	result = STK831X_VerifyCali(stk, 0);
	if(result)
	{
		printk(KERN_ERR "%s: calibration check1 fail, FT_index=%d\n", __func__, FT_index);		
		return -1;
	}
#ifdef STK_DEBUG_CALI		
	printk(KERN_INFO "\n%s:Check All OTP Data before write OTP\n", __func__);
	STK831x_ReadAllOTP();

	//Write OTP	
	printk(KERN_INFO "\n%s:Write offset data to FT%d OTP\n", __func__, FT_index);
#endif	
	result = STK831x_WriteOffsetOTP(stk, FT_index, otp_offset);
	if(result == -2)
	{
		printk(KERN_INFO "%s: write OTP fail, OTP%d has been used\n", __func__, FT_index);
		return -2;
	}
	else if(result < 0)
	{
		printk(KERN_INFO "%s: write OTP%d fail\n", __func__, FT_index);
		return -1;
	}
	
	buffer[0] = STK831X_MODE;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}			
	buffer[1] = (buffer[0] & 0xF8);
	buffer[0] = STK831X_MODE;	
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed", __func__);
		return result;
	}	
	
	msleep(1);
	STK831x_Init(stk, this_client);
#ifdef STK_DEBUG_CALI		
	printk(KERN_INFO "\n%s:Check All OTP Data after write OTP and reset\n", __func__);
	STK831x_ReadAllOTP();
#endif
		
	result = STK831X_VerifyCali(stk, 1);
	if(result)
	{
		printk(KERN_ERR "%s: calibration check2 fail\n", __func__);
		return result;
	}
	return 0;
}

static int32_t stk_get_file_content(char * r_buf, int8_t buf_size)
{
	struct file  *cali_file;
	mm_segment_t fs;	
	ssize_t ret;
	
    cali_file = filp_open(STK_ACC_CALI_FILE, O_RDWR,0);
    if(IS_ERR(cali_file))
	{
        printk(KERN_ERR "%s: filp_open error, no offset file!\n", __func__);
        return -ENOENT;
	}
	else
	{
		fs = get_fs();
		set_fs(get_ds());
		ret = cali_file->f_op->read(cali_file,r_buf, STK_ACC_CALI_FILE_SIZE,&cali_file->f_pos);
		if(ret < 0)
		{
			printk(KERN_ERR "%s: read error, ret=%d\n", __func__, ret);
			filp_close(cali_file,NULL);
			return -EIO;
		}		
		set_fs(fs);
    }
	
    filp_close(cali_file,NULL);	
	return 0;	
}

#ifdef STK_PERMISSION_THREAD
SYSCALL_DEFINE3(fchmodat, int, dfd, const char __user *, filename, mode_t, mode);
static struct task_struct *STKPermissionThread = NULL;

static int stk_permission_thread(void *data)
{
	int ret = 0;
	int retry = 0;
	mm_segment_t fs = get_fs();
	set_fs(KERNEL_DS);	
	msleep(20000);
	do{
		msleep(5000);
		ret = sys_fchmodat(AT_FDCWD, "/data/misc/stk_acc_cali.conf" , 0666);
		//if(ret < 0)
		//	printk("fail to execute sys_fchmodat, ret = %d\n", ret);
		if(retry++ > 10)
			break;
	}while(ret == -ENOENT);
	set_fs(fs);
	printk(KERN_INFO "%s exit, retry=%d\n", __func__, retry);
	return 0;
}
#endif	/*	#ifdef STK_PERMISSION_THREAD	*/
static int stk_store_in_file(char offset[])
{
	struct file  *cali_file;
	char r_buf[STK_ACC_CALI_FILE_SIZE] = {0};
	char w_buf[STK_ACC_CALI_FILE_SIZE] = {0};	
	mm_segment_t fs;	
	ssize_t ret;
	int8_t i;
	
	w_buf[0] = STK_ACC_CALI_VER0;
	w_buf[1] = STK_ACC_CALI_VER1;
	w_buf[2] = offset[0];
	w_buf[3] = offset[1];
	w_buf[4] = offset[2];
	
    cali_file = filp_open(STK_ACC_CALI_FILE, O_CREAT | O_RDWR,0666);
	
    if(IS_ERR(cali_file))
	{
        printk(KERN_ERR "%s: filp_open error!\n", __func__);
        return -ENOENT;
	}
	else
	{
		fs = get_fs();
		set_fs(get_ds());
		
		ret = cali_file->f_op->write(cali_file,w_buf,STK_ACC_CALI_FILE_SIZE,&cali_file->f_pos);
		if(ret != STK_ACC_CALI_FILE_SIZE)
		{
			printk(KERN_ERR "%s: write error!\n", __func__);
			filp_close(cali_file,NULL);
			return -EIO;
		}
		cali_file->f_pos=0x00;
		ret = cali_file->f_op->read(cali_file,r_buf, STK_ACC_CALI_FILE_SIZE,&cali_file->f_pos);
		if(ret < 0)
		{
			printk(KERN_ERR "%s: read error!\n", __func__);
			filp_close(cali_file,NULL);
			return -EIO;
		}		
		set_fs(fs);
		
		//printk(KERN_INFO "%s: read ret=%d!", __func__, ret);
		for(i=0;i<STK_ACC_CALI_FILE_SIZE;i++)
		{
			if(r_buf[i] != w_buf[i])
			{
				printk(KERN_ERR "%s: read back error, r_buf[%x](0x%x) != w_buf[%x](0x%x)\n", 
					__func__, i, r_buf[i], i, w_buf[i]);				
				filp_close(cali_file,NULL);
				return -EIO;
			}
		}
    }
    filp_close(cali_file,NULL);	
	//printk(KERN_INFO "%s successfully\n", __func__);
	return 0;		
}

static int stk_open(struct inode *inode, struct file *file)
{
	int ret;
	ret = nonseekable_open(inode, file);
	if(ret < 0)
		return ret;
	file->private_data = stk831x_data_ptr;		
	return 0;
}

static int stk_release(struct inode *inode, struct file *file)
{
	return 0;
}

#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,36))	
static long stk_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#else	
static int stk_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	void __user *argp = (void __user *)arg;
	int retval = 0;
	char state = 0, restore_state = 0;
	char rwbuf[8] = "";
	uint32_t delay_ns;	
	char char3_buffer[3];
	int result;
	int int3_buffer[3];
	struct stk831x_data *stk = file->private_data;
	
/*	printk(KERN_INFO "%s: cmd = 0x%x\n", __func__, cmd);	*/

	if(cmd == STK_IOCTL_SET_DELAY || cmd == STK_IOCTL_SET_OFFSET || cmd == STK_IOCTL_SET_RANGE || cmd == STK_IOCTL_WRITE || cmd == STK_IOCTL_SET_CALI)
	{
		STK831x_GetEnable(stk, &restore_state);
		if(restore_state)
			STK831x_SetEnable(stk, 0);
	}
	
	switch (cmd) 
	{
	case STK_IOCTL_SET_OFFSET:	
		if(copy_from_user(&char3_buffer, argp, sizeof(char3_buffer)))
			return -EFAULT;							
		break;
	case STK_IOCTL_SET_DELAY:	
		if(copy_from_user(&delay_ns, argp, sizeof(uint32_t)))
			return -EFAULT;					
		break;
	case STK_IOCTL_WRITE:	
	case STK_IOCTL_READ:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
			return -EFAULT;			
		break;	
	case STK_IOCTL_SET_ENABLE:
	case STK_IOCTL_SET_RANGE:
	case STK_IOCTL_SET_CALI:
		if(copy_from_user(&state, argp, sizeof(char)))
			return -EFAULT;		
		break;
	default:
		break;
	}
	
	switch (cmd) 
	{
	case STK_IOCTL_WRITE:
		if (rwbuf[0] < 2)
			return -EINVAL;		
		result = STK_i2c_Tx(&rwbuf[1], rwbuf[0]);
		if (result < 0) 
			return result;	
		break;	
	case STK_IOCTL_SET_OFFSET:					
		STK831x_SetOffset(char3_buffer);
		break;
	case STK_IOCTL_SET_DELAY:	
		STK831x_SetDelay(stk, delay_ns);
		break;		
	case STK_IOCTL_READ:
		if (rwbuf[0] < 1)
			return -EINVAL;		
		result = STK_i2c_Rx(&rwbuf[1], rwbuf[0]);
		if (result < 0) 
			return result;	
		break;	
	case STK_IOCTL_GET_DELAY:
		STK831x_GetDelay(stk, &delay_ns);
		break;
	case STK_IOCTL_GET_OFFSET:
		STK831x_GetOffset(char3_buffer);
		break;
	case STK_IOCTL_GET_ACCELERATION:		
		STK831x_ReadSensorData(stk);
		mutex_lock(&stk->read_lock);
		int3_buffer[0] = stk->raw_data[0];
		int3_buffer[1] = stk->raw_data[1];
		int3_buffer[2] = stk->raw_data[2];			
		mutex_unlock(&stk->read_lock);	
		break;
	case STK_IOCTL_SET_ENABLE:
		STK831x_SetEnable(stk, state);
			break;
	case STK_IOCTL_GET_ENABLE:
		STK831x_GetEnable(stk, &state);
		break;
	case STK_IOCTL_SET_RANGE:
		STK831x_SetRange(state);
		break;
	case STK_IOCTL_GET_RANGE:
		STK831x_GetRange(&state);
		break;
	case STK_IOCTL_SET_CALI:
		STK831x_SetCali(stk, state);
		break;
	default:
		retval = -ENOTTY;
		break;
	}	

	if(cmd == STK_IOCTL_SET_DELAY || cmd == STK_IOCTL_SET_OFFSET || cmd == STK_IOCTL_SET_RANGE || cmd == STK_IOCTL_WRITE || cmd == STK_IOCTL_SET_CALI)
	{
		if(restore_state)
			STK831x_SetEnable(stk, restore_state);	
	}
	switch (cmd) 
	{
	case STK_IOCTL_GET_ACCELERATION:		
		if(copy_to_user(argp, &int3_buffer, sizeof(int3_buffer)))
			return -EFAULT;			
		break;	
	case STK_IOCTL_READ:
		if(copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			return -EFAULT;			
		break;	
	case STK_IOCTL_GET_DELAY:
		if(copy_to_user(argp, &delay_ns, sizeof(delay_ns)))
			return -EFAULT;			
		break;			
	case STK_IOCTL_GET_OFFSET:
		if(copy_to_user(argp, &char3_buffer, sizeof(char3_buffer)))
			return -EFAULT;		
		break;		
	case STK_IOCTL_GET_RANGE:		
	case STK_IOCTL_GET_ENABLE:
		if(copy_to_user(argp, &state, sizeof(char)))
			return -EFAULT;		
		break;
	default:
		break;
	}		

	return retval;
}


static struct file_operations stk_fops = {
	.owner = THIS_MODULE,
	.open = stk_open,
	.release = stk_release,
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,36))	
	.unlocked_ioctl = stk_ioctl,
#else	
	.ioctl = stk_ioctl,
#endif
};

static struct miscdevice stk_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "stk831x",
	.fops = &stk_fops,
};


#if STK_ACC_POLLING_MODE

static int stk_acc_polling_function(void *arg)
{
	struct stk831x_data *stk = arg;	
    uint32_t ndelay;
    init_completion(&acc_thread_completion);
	
    while (1)
    {
        STK831x_GetDelay(stk, &ndelay);
        STK831x_ReadSensorData(stk);
		STK831x_ReportValue(stk);
        if (atomic_read(&stk->run_thread) == 0)
            break;
        msleep(ndelay/1000000);
    };
    complete(&acc_thread_completion);
    return 0;
}

#else

static irqreturn_t stk_mems_irq_handler(int irq, void *data)
{
	struct stk831x_data *pData = data;
	disable_irq_nosync(pData->irq);
    queue_work(stk_mems_work_queue,&pData->stk_work);
	return IRQ_HANDLED;
}


static void stk_mems_wq_function(struct work_struct *work)
{
	struct stk831x_data *stk = container_of(work, struct stk831x_data, stk_work);				
	STK831x_ReadSensorData(stk);
	STK831x_ReportValue(stk);
	enable_irq(stk->irq);
}



static int stk831x_irq_setup(struct i2c_client *client, struct stk831x_data *stk_int)
{
	int error;
	int irq= -1;	
#if ADDITIONAL_GPIO_CFG 
	if (gpio_request(STK_INT_PIN, "EINT"))
	{
		printk(KERN_ERR "%s:gpio_request() failed\n",__func__);
		return -1;
	}
	gpio_direction_input(STK_INT_PIN);
	
	irq = gpio_to_irq(STK_INT_PIN);
	if ( irq < 0 )
	{
		printk(KERN_ERR "%s:gpio_to_irq() failed\n",__func__);		
		return -1;
	}
	client->irq = irq;
	stk_int->irq = irq;	
#endif //#if ADDITIONAL_GPIO_CFG 
	printk(KERN_INFO "%s: irq # = %d\n", __func__, irq);
	if(irq < 0)
		printk(KERN_ERR "%s: irq number was not specified!\n", __func__);
	error = request_irq(client->irq, stk_mems_irq_handler, IRQF_TRIGGER_RISING , "stk-mems", stk_int);
	if (error < 0) 
	{
		printk(KERN_ERR "%s: request_irq(%d) failed for (%d)\n", __func__, client->irq, error);
		return -1;
	}	
	disable_irq(irq);	
	return irq;	
}

#endif	//#if STK_ACC_POLLING_MODE

static ssize_t stk831x_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct stk831x_data *stk = i2c_get_clientdata(this_client);

	return sprintf(buf, "%d\n", atomic_read(&stk->enabled));
}

static ssize_t stk831x_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	struct stk831x_data *stk = i2c_get_clientdata(this_client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if ((data == 0)||(data==1)) 
	{
		STK831x_SetEnable(stk,data);
	}
	else
		printk(KERN_ERR "%s: invalud argument, data=%ld\n", __func__, data);
	return count;
}

static ssize_t stk831x_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct stk831x_data *stk = i2c_get_clientdata(this_client);	
	int ddata[3];
	STK831x_ReadSensorData(stk);
	mutex_lock(&stk->read_lock);
	ddata[0]= stk->raw_data[0];
	ddata[1]= stk->raw_data[1];
	ddata[2]= stk->raw_data[2];
	mutex_unlock(&stk->read_lock);
	return sprintf(buf, "%d %d %d\n", ddata[0], ddata[1], ddata[2]);
}

static ssize_t stk831x_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct stk831x_data *stk = i2c_get_clientdata(this_client);
	uint32_t gdelay_ns;
	
	STK831x_GetDelay(stk, &gdelay_ns);
	return sprintf(buf, "%d\n", gdelay_ns/1000000);
}

static ssize_t stk831x_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct stk831x_data *stk = i2c_get_clientdata(this_client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	STK831x_SetDelay(stk, data*1000000);	// ms to ns
	return count;
}

static ssize_t stk831x_cali_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;	
}

static ssize_t stk831x_cali_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct stk831x_data *stk = i2c_get_clientdata(this_client);
	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	
	STK831x_SetCali(stk, data);
	return count;
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, stk831x_enable_show, stk831x_enable_store);
static DEVICE_ATTR(value, S_IRUGO, stk831x_value_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, stk831x_delay_show, stk831x_delay_store);
static DEVICE_ATTR(cali, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, stk831x_cali_show, stk831x_cali_store);

static struct attribute *stk831x_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_value.attr,
	&dev_attr_delay.attr,
	&dev_attr_cali.attr,
	NULL
};

static struct attribute_group stk831x_attribute_group = {
	.attrs = stk831x_attributes,
};
static int stk831x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int error;
	struct stk831x_data *stk;


	printk(KERN_INFO "stk831x_probe: driver version:%s\n",STK_ACC_DRIVER_VERSION);	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		printk(KERN_ERR "%s:i2c_check_functionality error\n", __func__);
		error = -ENODEV;
		goto exit_i2c_check_functionality_error;
	}	
	
	stk = kzalloc(sizeof(struct stk831x_data),GFP_KERNEL);
	if (!stk) 
	{	
		printk(KERN_ERR "%s:memory allocation error\n", __func__);
		error = -ENOMEM;
		goto exit_kzalloc_error;
	}
	stk831x_data_ptr = stk;
	mutex_init(&stk->read_lock);

#if (!STK_ACC_POLLING_MODE)	
	stk_mems_work_queue = create_workqueue("stk_mems_wq");
	if(stk_mems_work_queue)
		INIT_WORK(&stk->stk_work, stk_mems_wq_function);
	else
	{
		printk(KERN_ERR "%s:create_workqueue error\n", __func__);
		error = -EPERM;
		goto exit_create_workqueue_error;
	}		

	error = stk831x_irq_setup(client, stk);
	if(!error)
	{
		goto exit_irq_setup_error;
	}
#endif	//#if STK_ACC_POLLING_MODE
	
	i2c_set_clientdata(client, stk);	
	this_client = client;
	stk->first_enable = true;
	error = STK831x_Init(stk, client);
	if (error) 
	{		
		printk(KERN_ERR "%s:stk831x initialization failed\n", __func__);	
		goto exit_stk_init_error;
	}
	
	stk->input_dev = input_allocate_device();
	if (!stk->input_dev) 
	{
		error = -ENOMEM;
		printk(KERN_ERR "%s:input_allocate_device failed\n", __func__);
		goto exit_input_dev_alloc_error;
	}
	
	stk->input_dev->name = ACC_IDEVICE_NAME;
	set_bit(EV_ABS, stk->input_dev->evbit);	
#ifdef CONFIG_SENSORS_STK8312
	input_set_abs_params(stk->input_dev, ABS_X, -128, 127, 0, 0);
	input_set_abs_params(stk->input_dev, ABS_Y, -128, 127, 0, 0);
	input_set_abs_params(stk->input_dev, ABS_Z, -128, 127, 0, 0);	
#elif defined CONFIG_SENSORS_STK8313
	input_set_abs_params(stk->input_dev, ABS_X, -512, 511, 0, 0);
	input_set_abs_params(stk->input_dev, ABS_Y, -512, 511, 0, 0);
	input_set_abs_params(stk->input_dev, ABS_Z, -512, 511, 0, 0);	
#endif

	cali_offset[0] = 0;
	cali_offset[1] = 0;
	cali_offset[2] = 0;
	atomic_set(&stk->enabled, 0);

	error = input_register_device(stk->input_dev);
	if (error) 
	{
		printk(KERN_ERR "%s:Unable to register input device: %s\n", __func__, stk->input_dev->name);					 
		goto exit_input_register_device_error;
	}
	
	error = misc_register(&stk_device);
	if (error) 
	{
		printk(KERN_ERR "%s: misc_register failed\n", __func__);
		goto exit_misc_device_register_error;
	}		
	error = sysfs_create_group(&stk->input_dev->dev.kobj, &stk831x_attribute_group);
	if (error) 
	{
		printk(KERN_ERR "%s: sysfs_create_group failed\n", __func__);
		goto exit_sysfs_create_group_error;
	}		
	printk(KERN_INFO "%s successfully\n", __func__);
	return 0;
exit_sysfs_create_group_error:
	sysfs_remove_group(&stk->input_dev->dev.kobj, &stk831x_attribute_group);
exit_misc_device_register_error:
	misc_deregister(&stk_device);
exit_input_register_device_error:	
	input_unregister_device(stk->input_dev);	
exit_input_dev_alloc_error:	
exit_stk_init_error:	
#if (!STK_ACC_POLLING_MODE)
	free_irq(client->irq, stk);
#if ADDITIONAL_GPIO_CFG 
exit_irq_setup_error:
	gpio_free( STK_INT_PIN );	
#endif 	//#if ADDITIONAL_GPIO_CFG 
	destroy_workqueue(stk_mems_work_queue);		
exit_create_workqueue_error:	
#endif 	//#if (!STK_ACC_POLLING_MODE)	
	mutex_destroy(&stk->read_lock);
	kfree(stk);	
	stk = NULL;	
exit_kzalloc_error:	
exit_i2c_check_functionality_error:	
	return error;
}

static int stk831x_remove(struct i2c_client *client)
{
	struct stk831x_data *stk = i2c_get_clientdata(client);

	sysfs_remove_group(&stk->input_dev->dev.kobj, &stk831x_attribute_group);
	misc_deregister(&stk_device);
	input_unregister_device(stk->input_dev);	
	cancel_work_sync(&stk->stk_work);	
	free_irq(client->irq, stk);
#if (!STK_ACC_POLLING_MODE)
#if ADDITIONAL_GPIO_CFG
	gpio_free( STK_INT_PIN );
#endif //#if ADDITIONAL_GPIO_CFG 		
	if (stk_mems_work_queue)
		destroy_workqueue(stk_mems_work_queue);	
#endif	//#if (!STK_ACC_POLLING_MODE)	
	mutex_destroy(&stk->read_lock);	
	kfree(stk);
	stk = NULL;		
	return 0;
}

static const struct i2c_device_id stk831x[] = {
	{ STK831X_I2C_NAME, 0 },
	{ }
};


/*
static int stk831x_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct stk831x_data *stk = i2c_get_clientdata(client);
	if(atomic_read(&stk->enabled))
		STK831x_SetEnable(stk, 0);
	return 0;
}

static int stk831x_resume(struct i2c_client *client)
{
	struct stk831x_data *stk = i2c_get_clientdata(client);
	if(atomic_read(&stk->enabled))	
		STK831x_SetEnable(stk, 1);
	return 0;
}
*/

#ifdef CONFIG_PM_SLEEP
static int stk831x_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct stk831x_data *stk = i2c_get_clientdata(client);
	if(atomic_read(&stk->enabled))
	{
		STK831x_SetEnable(stk, 0);
		atomic_set(&stk->enabled, 1);	
	}
	return 0;
}


static int stk831x_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct stk831x_data *stk = i2c_get_clientdata(client);
	int error = 0;
	error = STK831x_Init(stk, client);
	if (error) 
	{		
		printk(KERN_ERR "%s:stk831x resume initialization failed\n", __func__);	
	}
	STK831x_SetOffset(cali_offset);
	if(atomic_read(&stk->enabled))	
		STK831x_SetEnable(stk, 1);
	return 0;		
}
#endif /* CONFIG_PM_SLEEP */


#ifdef CONFIG_PM_RUNTIME
static int stk831x_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct stk831x_data *stk = i2c_get_clientdata(client);
	if(atomic_read(&stk->enabled))
	{
		STK831x_SetEnable(stk, 0);
		atomic_set(&stk->enabled, 1);			
	}
	return 0;
}


static int stk831x_runtime_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct stk831x_data *stk = i2c_get_clientdata(client);
	int error = 0;
	error = STK831x_Init(stk, client);
	if (error) 
	{		
		printk(KERN_ERR "%s:stk831x resume initialization failed\n", __func__);	
	}
	STK831x_SetOffset(cali_offset);
	if(atomic_read(&stk->enabled))	
		STK831x_SetEnable(stk, 1);
	return 0;		
}
#endif /* CONFIG_PM_RUNTIME */

static const struct dev_pm_ops stk831x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stk831x_suspend, stk831x_resume)
	SET_RUNTIME_PM_OPS(stk831x_runtime_suspend, stk831x_runtime_resume, NULL)
};



static struct i2c_driver stk831x_driver = {
	.probe = stk831x_probe,
	.remove = stk831x_remove,
	.id_table	= stk831x,
//	.suspend = stk831x_suspend,
//	.resume = stk831x_resume,
	.driver = {
		   .name = STK831X_I2C_NAME,
		   .pm = &stk831x_pm_ops,
	},
};

static int __init stk831x_init(void)
{
	int ret;
	ret = i2c_add_driver(&stk831x_driver);
#ifdef STK_PERMISSION_THREAD
	STKPermissionThread = kthread_run(stk_permission_thread,"stk","Permissionthread");
	if(IS_ERR(STKPermissionThread))
		STKPermissionThread = NULL;
#endif // STK_PERMISSION_THREAD		
	return ret; 
}

static void __exit stk831x_exit(void)
{
	i2c_del_driver(&stk831x_driver);
#ifdef STK_PERMISSION_THREAD
	if(STKPermissionThread)
		STKPermissionThread = NULL;
#endif // STK_PERMISSION_THREAD		
}

module_init(stk831x_init);
module_exit(stk831x_exit);

MODULE_AUTHOR("Lex Hsieh / Sensortek");
MODULE_DESCRIPTION("stk831x 3-Axis accelerometer driver");
MODULE_LICENSE("GPL");	
MODULE_VERSION(STK_ACC_DRIVER_VERSION);
