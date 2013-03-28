/* drivers/input/touchscreen/sis_i2c.c - I2C Touch panel driver for SiS 9200 family
 *
 * Copyright (C) 2011 SiS, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Date: 2012/09/10
 * Version:	Android_v2.03.00-A639-0910
 */

#include <linux/module.h>
#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include "linux/sis_i2c.h"
#include <linux/linkage.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <mach/am_regs.h>

#ifdef _STD_RW_IO
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#define DEVICE_NAME "sis_aegis_touch_device"
static int sis_char_devs_count = 1;        /* device count */
static int sis_char_major = 0;
static struct cdev sis_char_cdev;
static struct class *sis_char_class = NULL;
#endif

static resume_flag = 0;

/* Addresses to scan */
static const unsigned short normal_i2c[] = { SIS_SLAVE_ADDR, I2C_CLIENT_END };
static struct workqueue_struct *sis_wq;
struct sis_ts_data *ts_bak = 0;
struct sisTP_driver_data *TPInfo = NULL;
static void sis_tpinfo_clear(struct sisTP_driver_data *TPInfo, int max);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sis_ts_early_suspend(struct early_suspend *h);
static void sis_ts_late_resume(struct early_suspend *h);
#endif

#ifdef CONFIG_X86
//static const struct i2c_client_address_data addr_data;
/* Insmod parameters */
static int sis_ts_detect(struct i2c_client *client, struct i2c_board_info *info);
#endif

#ifdef _CHECK_CRC
static uint16_t cal_crc (char* cmd, int start, int end);
#endif

void PrintBuffer(int start, int length, char* buf)
{
	int i;
	for ( i = start; i < length; i++ )
	{
		printk("%02x ", buf[i]);
		if (i != 0 && i % 30 == 0)
			printk("\n");
	}
	printk("\n");	
}

int sis_command_for_write(struct i2c_client *client, int wlength, unsigned char *wdata)
{
    int ret = -1;
    struct i2c_msg msg[1];

    msg[0].addr = client->addr;
    msg[0].flags = 0; //Write
    msg[0].len = wlength;
    msg[0].buf = (unsigned char *)wdata;

    ret = i2c_transfer(client->adapter, msg, 1);

    return ret;
}

int sis_command_for_read(struct i2c_client *client, int rlength, unsigned char *rdata)
{
    int ret = -1;
    struct i2c_msg msg[1];

    msg[0].addr = client->addr;
    msg[0].flags = I2C_M_RD; //Read
    msg[0].len = rlength;
    msg[0].buf = rdata;

    ret = i2c_transfer(client->adapter, msg, 1);

    return ret;
}

int sis_sent_command_to_fw(struct i2c_client *client, int wlength, unsigned char *wdata, int rlength, unsigned char *rdata,\
							const unsigned char* func_name)
{
    int ret = -1;  

	ret = sis_command_for_write(client, wlength, wdata);
    if (ret < 0)
	{
		if (wdata[0] == 0x90)
		{
			printk(KERN_ERR "%s: CMD- 90 +%2x i2c_transfer write error - %d\n", func_name, wdata[2] ,ret);
		}
		else
		{
			printk(KERN_ERR "%s: CMD-%2x i2c_transfer write error - %d\n", func_name, wdata[0] ,ret);
		}
	}
	else 
	{
		msleep(3000);
		ret = sis_command_for_read(client, rlength, rdata);
		if (ret < 0)
		{
			printk(KERN_ERR "%s: CMD-%2x i2c_transfer write error - %d\n", func_name, wdata[0] ,ret);
		}
	} 	    
		
    return ret;
}

int sis_cul_unit(uint8_t report_id)
{
	int basic = 6;
	int area = 2;
	int pressure = 1;
	int ret = basic;
	
	if (report_id != ALL_IN_ONE_PACKAGE)
	{
		if (IS_AREA(report_id) && IS_TOUCH(report_id))
		{
			ret += area;
		}
		if (IS_PRESSURE(report_id))
		{
			ret += pressure;
		}
	}
	
	return ret;	
}
#ifdef OLD_FORMAT_AEGIS
int sis_ReadPacket(struct i2c_client *client, uint8_t cmd, uint8_t* buf)
{
	uint8_t tmpbuf[MAX_BYTE] = {0};
    int ret = -1;
    int bytecount = 0;
    int touchnum = 0;
//	uint8_t offset = 0;
//	bool ReadNext = false;
//	uint8_t ByteCount = 0;
//	uint8_t fingers = 0;

/* 
#ifndef _SMBUS_INTERFACE
    struct i2c_msg msg[2];
 
	msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = (char *)(&cmd);

    msg[0].addr = client->addr;
    msg[0].flags = I2C_M_RD;
    msg[0].len = MAX_BYTE;
    msg[0].buf = tmpbuf;
#endif
	ret = i2c_transfer(client->adapter, msg, 1);
*/ 

/*
		New i2c format 
	* buf[0] = Low 8 bits of byte count value
	* buf[1] = High 8 bits of byte counte value
	* buf[2] = Report ID
	* buf[touch num * 6 + 2 ] = Touch informations; 1 touch point has 6 bytes, it could be none if no touch 
	* buf[touch num * 6 + 3] = Touch numbers
	* 
	* One touch point information include 6 bytes, the order is 
	* 
	* 1. status = touch down or touch up
	* 2. id = finger id 
	* 3. x axis low 8 bits
	* 4. x axis high 8 bits
	* 5. y axis low 8 bits
	* 6. y axis high 8 bits
	* 
*/
	ret = sis_command_for_read(client, MAX_BYTE, tmpbuf);

#if _DEBUG_PACKAGE
	printk(KERN_INFO "chaoban test: Buf_Data [0~63] \n");
	PrintBuffer(0, 64, tmpbuf);	
#endif

	if(ret < 0 )
	{
		printk(KERN_ERR "sis_ReadPacket: i2c transfer error\n");
	}
	memcpy(&buf[0], &tmpbuf[0], 58);
	bytecount = buf[P_BYTECOUNT] & 0xff;
	touchnum = buf[TOUCH_NUM] & 0xff;
	if (bytecount == ((touchnum * 8) + 2 ))
	{
		if(touchnum > 7)
		{
			ret = sis_command_for_read(client, MAX_BYTE, tmpbuf);

#if _DEBUG_PACKAGE
			printk(KERN_INFO "chaoban test: Buf_Data [64-125] \n");
			PrintBuffer(0, 64, tmpbuf);
#endif	

			if(ret < 0 )
			{
				printk(KERN_ERR "sis_ReadPacket: i2c transfer error\n");
				return ret;
			}
			memcpy(&buf[58], &tmpbuf[0], 64);
			ret = touchnum;
			return ret;			
		}
		else
		{
			ret = touchnum;
			return ret;
		}
	} 
	else
	{
		ret = -1;
		return ret;
	}
		
}	
#else
int sis_ReadPacket(struct i2c_client *client, uint8_t cmd, uint8_t* buf)
{
	uint8_t tmpbuf[MAX_BYTE] = {0};	//MAX_BYTE = 64;
#ifdef _CHECK_CRC
	uint16_t buf_crc = 0;
	uint16_t package_crc = 0;
	int l_package_crc = 0;
	int crc_end = 0;
#endif
    int ret = -1;
    int touchnum = 0;
    int p_count = 0;
    int touc_formate_id = 0;
    int locate = 0;
    bool read_first = true;
    
/*
		New i2c format 
	* buf[0] = Low 8 bits of byte count value
	* buf[1] = High 8 bits of byte counte value
	* buf[2] = Report ID
	* buf[touch num * 6 + 2 ] = Touch informations; 1 touch point has 6 bytes, it could be none if no touch 
	* buf[touch num * 6 + 3] = Touch numbers
	* 
	* One touch point information include 6 bytes, the order is 
	* 
	* 1. status = touch down or touch up
	* 2. id = finger id 
	* 3. x axis low 8 bits
	* 4. x axis high 8 bits
	* 5. y axis low 8 bits
	* 6. y axis high 8 bits
	* 
*/
	do
	{
		if (locate >= PACKET_BUFFER_SIZE)
		{
			printk(KERN_ERR "sis_ReadPacket: Buf Overflow\n");
			return -1;
		}
		
		ret = sis_command_for_read(client, MAX_BYTE, tmpbuf);

#ifdef _DEBUG_PACKAGE
		printk(KERN_INFO "chaoban test: Buf_Data [0~63] \n");
		PrintBuffer(0, 64, tmpbuf);	
#endif			

		if(ret < 0 )
		{
			printk(KERN_ERR "sis_ReadPacket: i2c transfer error\n");
			return ret;
		}
		// error package length of receiving data
		else if (tmpbuf[P_BYTECOUNT] > MAX_BYTE)
		{
			printk(KERN_ERR "sis_ReadPacket: Error Bytecount\n");
			return -1;	
		}
		
		if (read_first)
		{
#ifdef _SUPPORT_BUTTON_TOUCH
			// access BUTTON TOUCH event and BUTTON NO TOUCH event
			if (tmpbuf[P_REPORT_ID] ==  BUTTON_FORMAT)
			{
				memcpy(&buf[0], &tmpbuf[0], 7);
				return touchnum; 	//touchnum is 0
			}
#endif 
			// access NO TOUCH event unless BUTTON NO TOUCH event
			if (tmpbuf[P_BYTECOUNT] == NO_TOUCH_BYTECOUNT)
			{
				return touchnum;	//touchnum is 0
			}
		}

		//skip parsing data when two devices are registered at the same slave address
		//parsing data when P_REPORT_ID && 0xf is TOUCH_FORMAT or P_REPORT_ID is ALL_IN_ONE_PACKAGE
		touc_formate_id = tmpbuf[P_REPORT_ID] && 0xf;
		if ((touc_formate_id != TOUCH_FORMAT) && (tmpbuf[P_REPORT_ID] != ALL_IN_ONE_PACKAGE))
		{
			printk(KERN_ERR "sis_ReadPacket: Error Report_ID\n");
			return -1;		
		}
		
		p_count = (int) tmpbuf[P_BYTECOUNT] - 1;	//start from 0
		if (tmpbuf[P_REPORT_ID] != ALL_IN_ONE_PACKAGE)
		{
			p_count -= BYTE_CRC;
			if (IS_SCANTIME(tmpbuf[P_REPORT_ID]))
			{
				p_count -= BYTE_SCANTIME;
			}
		}
		//else {}    						// For ALL_IN_ONE_PACKAGE
		
		if (read_first)
		{
			touchnum = tmpbuf[p_count]; 	
		}
		else
		{
			if (tmpbuf[p_count] != 0)
			{
				printk(KERN_ERR "sis_ReadPacket: get error package\n");
				return -1;
			}
		}

#ifdef _CHECK_CRC
		crc_end = p_count + (IS_SCANTIME(tmpbuf[P_REPORT_ID]) * 2);
		buf_crc = cal_crc(tmpbuf, 2, crc_end); //sub bytecount (2 byte)
		l_package_crc = p_count + 1 + (IS_SCANTIME(tmpbuf[P_REPORT_ID]) * 2);
		package_crc = ((tmpbuf[l_package_crc] & 0xff) | ((tmpbuf[l_package_crc + 1] & 0xff) << 8));
			
		if (buf_crc != package_crc)
		{
			printk(KERN_ERR "sis_ReadPacket: CRC Error\n");
			return -1;
		}
#endif	
		memcpy(&buf[locate], &tmpbuf[0], 64);	//Buf_Data [0~63] [64~128]
		locate += 64;
		read_first = false;
		
	}while(tmpbuf[P_REPORT_ID] != ALL_IN_ONE_PACKAGE  &&  tmpbuf[p_count] > 5);

	return touchnum;
}	
#endif

int check_gpio_interrupt(void)
{
    int ret = 0;
    //TODO
    //CHECK GPIO INTERRUPT STATUS BY YOUR PLATFORM SETTING.
    //ret = gpio_get_value(GPIO_IRQ);
    return ret;
}

void ts_report_key(struct i2c_client *client, uint8_t keybit_state)
{
	int i = 0;
	uint8_t diff_keybit_state= 0x0; //check keybit_state is difference with pre_keybit_state
	uint8_t key_value = 0x0; //button location for binary
	uint8_t  key_pressed = 0x0; //button is up or down
	struct sis_ts_data *ts = i2c_get_clientdata(client);

	if (!ts)
	{
		printk(KERN_ERR "%s error: Missing Platform Data!\n", __func__);
		return;
	}

	diff_keybit_state = TPInfo->pre_keybit_state ^ keybit_state;

	if (diff_keybit_state)
	{
		for (i = 0; i < BUTTON_KEY_COUNT; i++)
		{
		    if ((diff_keybit_state >> i) & 0x01)
			{
				key_value = diff_keybit_state & (0x01 << i);
				key_pressed = (keybit_state >> i) & 0x01;
				switch (key_value)
				{
					case MSK_COMP:
						input_report_key(ts->input_dev, KEY_COMPOSE, key_pressed);
						printk(KERN_ERR "%s : MSK_COMP %d \n", __func__ , key_pressed);
						break;
					case MSK_BACK:
						input_report_key(ts->input_dev, KEY_BACK, key_pressed);
						printk(KERN_ERR "%s : MSK_BACK %d \n", __func__ , key_pressed);
						break;
					case MSK_MENU:
						input_report_key(ts->input_dev, KEY_MENU, key_pressed);
						printk(KERN_ERR "%s : MSK_MENU %d \n", __func__ , key_pressed);
						break;
					case MSK_HOME:
						input_report_key(ts->input_dev, KEY_HOME, key_pressed);
						printk(KERN_ERR "%s : MSK_HOME %d \n", __func__ , key_pressed);
						break;
					case MSK_NOBTN:
						//Release the button if it touched.
					default:
						break;
				}
			}
		}
		TPInfo->pre_keybit_state = keybit_state;
	}
}

#ifdef OLD_FORMAT_AEGIS
static void sis_ts_work_func(struct work_struct *work)
{
	struct sis_ts_data *ts = container_of(work, struct sis_ts_data, work);
    int ret = -1;
	uint8_t buf[PACKET_BUFFER_SIZE] = {0};
	uint8_t i = 0, fingers = 0;
	uint8_t px = 0, py = 0, pstatus = 0;
#ifdef _ANDROID_4
	bool all_touch_up = true;
#endif

    /* I2C or SMBUS block data read */
    ret = sis_ReadPacket(ts->client, SIS_CMD_NORMAL, buf);
#if _DEBUG_PACKAGE_WORKFUNC
	printk(KERN_INFO "chaoban test: Buf_Data [0~63] \n");
	PrintBuffer(0, 64, buf);			
	if (ret > 7 )
	{
		printk(KERN_INFO "chaoban test: Buf_Data [64~125] \n");
		PrintBuffer(0, 64, buf);	
	}
#endif
	if (ret < 0) //Error fingers' number or Unknow bytecount
	{
	    printk(KERN_INFO "chaoban test: ret = -1\n");
		goto err_free_allocate;
	}

/*	
	else if ((ret == 2) && (TPInfo->id == buf[0])) // Redundant package
	{
		goto label_send_report;
	}
*/
	sis_tpinfo_clear(TPInfo, MAX_FINGERS);

	/* Parser and Get the sis9200 data */
	fingers = (buf[TOUCH_NUM]);
	TPInfo->fingers = fingers = (fingers > MAX_FINGERS ? 0 : fingers);
//	TPInfo->id = buf[ADDR_REPORT_ID];
/*
	if ((buf[FORMAT_MODE] & MSK_BUTTON_POINT) == BUTTON_TOUCH_SERIAL)
	{
		int temp_fingers = 0;
		if (fingers > 1)
		{
			 temp_fingers = 2; // when fingers is >= 2, BS is placed at the same position
		}
		else
		{
			 temp_fingers = fingers;
		}
		ts_report_key(ts->client, buf[BUTTON_STATE + temp_fingers * 5]);
										//buf[BUTTON_STATE + temp_fingers * 5]: BS location
	}
	else
	{
		if (TPInfo->pre_keybit_state)
	  	{
			ts_report_key(ts->client, 0x0);//clear for polling
	  	}
	}
*/

	for (i = 0; i < fingers; i++)
	{
        pstatus = 2 + (i * 8);    // Calc point status

/*
		if (((buf[FORMAT_MODE] & MSK_BUTTON_POINT) == BUTTON_TOUCH_SERIAL) && i > 1)
		{
			pstatus += 1; 					// for button event and above 3 points
		}
*/
	    px = pstatus + 2;                   // Calc point x_coord
	    py = px + 2;                        // Calc point y_coord

		if ((buf[pstatus]) == TOUCHUP)
		{
			TPInfo->pt[i].Pressure = 0;
		}
		else if ((buf[pstatus]) == TOUCHDOWN)
		{
			TPInfo->pt[i].Pressure = (buf[pstatus + 7 ]);
		}
		else
		{
			goto err_free_allocate;
		}
//		TPInfo->pt[i].touch = (buf[pstatus + 6 ]);
		TPInfo->pt[i].Width = (buf[pstatus + 6 ]);
		TPInfo->pt[i].id = (buf[pstatus + 1 ]);
		TPInfo->pt[i].x = ((buf[px] & 0xff) | ((buf[px + 1] & 0xff)<< 8));
        TPInfo->pt[i].y = ((buf[py] & 0xff) | ((buf[py + 1] & 0xff)<< 8));
        
	}

#if _DEBUG_REPORT
	for (i = 0; i < TPInfo->fingers; i++)
	{
		printk(KERN_INFO "chaoban test: x[%d] = %d, y[%d] = %d, pressure[%d] = %d\n", i, TPInfo->pt[i].x, i, TPInfo->pt[i].y, i, TPInfo->pt[i].Pressure);
	}
#endif

//label_send_report:
    /* Report co-ordinates to the multi-touch stack */
#ifdef _ANDROID_4	
		for(i = 0; ((i < TPInfo->fingers) && (i < MAX_FINGERS)); i++)
		{
			if(TPInfo->pt[i].Pressure)
			{
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, TPInfo->pt[i].Width);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, TPInfo->pt[i].Pressure);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, TPInfo->pt[i].x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, TPInfo->pt[i].y);
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, TPInfo->pt[i].id);     //Android 2.3
				input_mt_sync(ts->input_dev);
				all_touch_up = false;
			}
			
			if (i == (TPInfo->fingers -1) && all_touch_up == true)
			{
				input_mt_sync(ts->input_dev);
			}
		}

		if(TPInfo->fingers == 0)
		{
			input_mt_sync(ts->input_dev);
		}
#else
	i = 0;
		do
		{
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, TPInfo->pt[i].Pressure);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, TPInfo->pt[i].Width);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, TPInfo->pt[i].x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, TPInfo->pt[i].y);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, TPInfo->pt[i].id);		//Android 2.3
			input_mt_sync(ts->input_dev);
			i++;
		}
		while ((i < TPInfo->fingers) && (i < MAX_FINGERS));
#endif
	input_sync(ts->input_dev);

err_free_allocate:

    if (ts->use_irq)
    {
#ifdef _INT_MODE_1 //case 1 mode
	    //TODO: After interrupt status low, read i2c bus data by polling, until interrupt status is high
	    //ret = check_gpio_interrupt();	//interrupt pin is still LOW, read data until interrupt pin is released.
	    //if (!ret)
	    //{
	    //    hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);
	    //}
	    //else
		mdelay(20);
	    {
			if (TPInfo->pre_keybit_state)
			{
				printk("TPInfo->pre_keybit_state\n");
				ts_report_key(ts->client, 0x0);//clear for interrupt
			}
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
        	if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#else
        	if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#endif
        	{
				printk("enable_irq\n");
				enable_irq(ts->client->irq);
			}
	    }
#else // case 2 mode

#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
		if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#else
		if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#endif
		{
			enable_irq(ts->client->irq);
		}
#endif
	}

    return;
}
#else
static void sis_ts_work_func(struct work_struct *work)
{
	struct sis_ts_data *ts = container_of(work, struct sis_ts_data, work);
    int ret = -1;
    int point_unit;  
	uint8_t buf[PACKET_BUFFER_SIZE] = {0};
	uint8_t i = 0, fingers = 0;
	uint8_t px = 0, py = 0, pstatus = 0;
	uint8_t p_area = 0;
    uint8_t p_preasure = 0;
#ifdef _SUPPORT_BUTTON_TOUCH	
	int button_key;
	uint8_t button_buf[10] = {0};
#endif

#ifdef _ANDROID_4
	bool all_touch_up = true;
#endif

	mutex_lock(&ts->mutex_wq); 

    /* I2C or SMBUS block data read */
    ret = sis_ReadPacket(ts->client, SIS_CMD_NORMAL, buf);
    
#ifdef _DEBUG_PACKAGE_WORKFUNC
	printk(KERN_INFO "chaoban test: Buf_Data [0~63] \n");
	PrintBuffer(0, 64, buf);			
	if ((buf[P_REPORT_ID] != ALL_IN_ONE_PACKAGE) && (ret > 5))
	{
		printk(KERN_INFO "chaoban test: Buf_Data [64~125] \n");
		PrintBuffer(64, 128, buf);	
	}
#endif

// add 
#ifdef _SUPPORT_BUTTON_TOUCH
	 sis_ReadPacket(ts->client, SIS_CMD_NORMAL, button_buf);
#endif

	if (ret < 0) // Error Number
	{
	    printk(KERN_INFO "chaoban test: ret = -1\n");
		goto err_free_allocate;
	}
#ifdef _SUPPORT_BUTTON_TOUCH
	// access BUTTON TOUCH event and BUTTON NO TOUCH even
	else if (button_buf[P_REPORT_ID] == BUTTON_FORMAT)
	{
		//fingers = 0; //modify
		button_key = ((button_buf[BUTTON_STATE] & 0xff) | ((button_buf[BUTTON_STATE + 1] & 0xff)<< 8));		
		ts_report_key(ts->client, button_key);
		//goto err_free_allocate; //modify
	}
#endif
	// access NO TOUCH event unless BUTTON NO TOUCH event
	else if (ret == 0)
	{
		TPInfo->fingers = fingers = 0;
		goto label_send_report;  //need to report input_mt_sync()
	}
	
	sis_tpinfo_clear(TPInfo, MAX_FINGERS);
	
	/* Parser and Get the sis9200 data */
	point_unit = sis_cul_unit(buf[P_REPORT_ID]);
	fingers = ret;
	TPInfo->fingers = fingers = (fingers > MAX_FINGERS ? 0 : fingers);

	for (i = 0; i < fingers; i++) // fingers 10 =  0 ~ 9
	{
        if ((buf[P_REPORT_ID] != ALL_IN_ONE_PACKAGE) && (i >= 5))
        {
			pstatus = BYTE_BYTECOUNT + BYTE_ReportID + ((i - 5) * point_unit);    	// Calc point status
			pstatus += 64;
		}
		else 
		{
			pstatus = BYTE_BYTECOUNT + BYTE_ReportID + (i * point_unit);          	// Calc point status
		}

	    px = pstatus + 2;                   			// Calc point x_coord
	    py = px + 2;                        			// Calc point y_coord

		if ((buf[pstatus]) == TOUCHUP)
		{
			TPInfo->pt[i].Width = 0;
			TPInfo->pt[i].Height = 0;
			TPInfo->pt[i].Pressure = 0;
		}
		else if (buf[P_REPORT_ID] == ALL_IN_ONE_PACKAGE && (buf[pstatus]) == TOUCHDOWN)
		{
			TPInfo->pt[i].Width = 1;
			TPInfo->pt[i].Height = 1;
			TPInfo->pt[i].Pressure = 1;			
		}
		else if ((buf[pstatus]) == TOUCHDOWN)
		{	
			p_area = py + 2;
			p_preasure = py + 2 + (IS_AREA(buf[P_REPORT_ID]) * 2);

			//area		
			if (IS_AREA(buf[P_REPORT_ID]))
			{
				TPInfo->pt[i].Width = buf[p_area] & 0xff;
				TPInfo->pt[i].Height = buf[p_area + 1] & 0xff;
			}
			else 
			{
				TPInfo->pt[i].Width = 1;
				TPInfo->pt[i].Height = 1;
			}
			//preasure
			if (IS_PRESSURE(buf[P_REPORT_ID]))
				TPInfo->pt[i].Pressure = (buf[p_preasure]);
			else 
				TPInfo->pt[i].Pressure = 1;				
		}
		else
		{
			printk(KERN_ERR "sis_ts_work_func: Error Touch Status\n");
			goto err_free_allocate;
		}
		
		TPInfo->pt[i].id = (buf[pstatus + 1]);
		TPInfo->pt[i].x = ((buf[px] & 0xff) | ((buf[px + 1] & 0xff)<< 8));
        TPInfo->pt[i].y = ((buf[py] & 0xff) | ((buf[py + 1] & 0xff)<< 8));      
	}
		
#ifdef _DEBUG_REPORT
	for (i = 0; i < TPInfo->fingers; i++)
	{
		 printk(KERN_INFO "chaoban test: i = %d, id = %d, x = %d, y = %d, pstatus = %d, width = %d, height = %d, pressure = %d, \n", i, TPInfo->pt[i].id, TPInfo->pt[i].x, TPInfo->pt[i].y, buf[pstatus], TPInfo->pt[i].Width,  TPInfo->pt[i].Height, TPInfo->pt[i].Pressure);
	}
#endif

label_send_report:
/* Report co-ordinates to the multi-touch stack */
#ifdef _ANDROID_4	
	for(i = 0; ((i < TPInfo->fingers) && (i < MAX_FINGERS)); i++)
	{
		if(TPInfo->pt[i].Pressure)
		{
			TPInfo->pt[i].Width *= AREA_UNIT;	
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, TPInfo->pt[i].Width);
			TPInfo->pt[i].Height *= AREA_UNIT;
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MINOR, TPInfo->pt[i].Height);			
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, TPInfo->pt[i].Pressure);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, TPInfo->pt[i].x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, TPInfo->pt[i].y);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, TPInfo->pt[i].id);     //Android 2.3
			input_mt_sync(ts->input_dev);
			all_touch_up = false;
		}
		else
			all_touch_up = true;
		//printk("TPInfo->fingers = %d, i= %d, all_touch_up=%d, TPInfo->pt[i].Pressure=%d\n", TPInfo->fingers, i, all_touch_up, TPInfo->pt[i].Pressure);
		if (i == (TPInfo->fingers -1) && all_touch_up == true)
		{
			input_report_key(ts->input_dev, BTN_TOUCH, 0);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_mt_sync(ts->input_dev);
		}
	}

	if(TPInfo->fingers == 0)
	{	
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(ts->input_dev);
	}
#else
	i = 0;
	do
	{
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, TPInfo->pt[i].Pressure);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, TPInfo->pt[i].Width);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MINOR, TPInfo->pt[i].Height);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, TPInfo->pt[i].x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, TPInfo->pt[i].y);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, TPInfo->pt[i].id);		//Android 2.3
		input_mt_sync(ts->input_dev);
		i++;
	}
	while ((i < TPInfo->fingers) && (i < MAX_FINGERS));
#endif
	input_sync(ts->input_dev);

err_free_allocate:

    if (ts->use_irq)
    {
#ifdef _INT_MODE_1 //case 1 mode
	    //TODO: After interrupt status low, read i2c bus data by polling, until interrupt status is high
	    //ret = check_gpio_interrupt();	//interrupt pin is still LOW, read data until interrupt pin is released.
	    //if (!ret)
	    //{
	    //    hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);
	    //}
	    //else
		mdelay(20);
	    {
			if (TPInfo->pre_keybit_state)
			{
				printk("clear int\n");
				ts_report_key(ts->client, 0x0);	//clear for interrupt
			}
			
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
        	if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#else
        	if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#endif
        	{
				enable_irq(ts->client->irq);
			}
	    }
#else // case 2 mode

#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
		if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#else
		if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#endif
		{
			enable_irq(ts->client->irq);
		}
#endif
	}

	mutex_unlock(&ts->mutex_wq);
    return;
}
#endif

static void sis_tpinfo_clear(struct sisTP_driver_data *TPInfo, int max)
{
	int i = 0;
	for(i = 0; i < max; i++)
	{
		TPInfo->pt[i].id = -1;
		TPInfo->pt[i].x = 0;
		TPInfo->pt[i].y = 0;
		TPInfo->pt[i].Pressure = 0;
		TPInfo->pt[i].Width = 0;
	}
	TPInfo->id = 0x0;
	TPInfo->fingers = 0;
}

static enum hrtimer_restart sis_ts_timer_func(struct hrtimer *timer)
{
	struct sis_ts_data *ts = container_of(timer, struct sis_ts_data, timer);
	queue_work(sis_wq, &ts->work);
	if (!ts->use_irq)
	{	// For Polling mode
	    hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);
	}
	return HRTIMER_NORESTART;
}

static irqreturn_t sis_ts_irq_handler(int irq, void *dev_id)
{
	struct sis_ts_data *ts = dev_id;
	if(resume_flag == 1)
	{
		resume_flag = 0;
		mdelay(100);
		return IRQ_HANDLED;
	}
#if 0	
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
	if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_ENABLED)
#else
	if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_ENABLED)
#endif
#endif
	{
		disable_irq_nosync(ts->client->irq);
	}
	queue_work(sis_wq, &ts->work);
		
	return IRQ_HANDLED;
}

static int initial_irq(void)
{
	int ret = 0;
#ifdef _I2C_INT_ENABLE
	/* initialize gpio and interrupt pins */
	/* TODO */
	//ret = gpio_request(GPIO_IRQ, "GPIO_133");	// ex. GPIO_133 for interrupt mode
	if (ret < 0)
	{
		// Set Active Low. Please reference the file include/linux/interrupt.h
		printk(KERN_ERR "sis_ts_probe: Failed to gpio_request\n");
		printk(KERN_ERR "sis_ts_probe: Fail : gpio_request was called before this driver call\n");
	}	
	/* setting gpio direction here OR boardinfo file*/
	/* TODO */
#else
	ret = -1;
#endif
	return ret;
}

static uint16_t cal_crc (char* cmd, int start, int end)
{
	int i = 0;
	uint16_t crc = 0;
	for (i = start; i <= end ; i++)
	{
		crc = (crc<<8) ^ crc16tab[((crc>>8) ^ cmd[i] )&0x00FF];
	}
	return crc;
}

void write_crc (unsigned char *buf, int start, int end)
{
	uint16_t crc = 0;
	crc = cal_crc (buf, start , end);
	buf[end+1] = (crc >> 8)& 0xff;
	buf[end+2] = crc & 0xff;
}

#ifdef CONFIG_FW_SUPPORT_POWERMODE
bool sis_check_fw_ready(struct i2c_client *client)
{
  	bool retry = true;
  	int ret = 0;
  	int check_num = 10;
	unsigned char rdata[MAX_BYTE] = {0};
	unsigned char CheckI2C_Address[MAX_BYTE] = {0x88, 0x0e, 0x70, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6f, 0x93};

	do
	{
		ret = sis_sent_command_to_fw(client, SIXTEEN_BYTE, CheckI2C_Address, MAX_BYTE, rdata, __func__);

		if (ret > -1)	
		{
#if 0
			PrintBuffer(0, 16, rdata);
#endif
			ret = rdata[0];
			if (ret == 6 && rdata[4]== 0x01)
			{
				retry = false;
			}
		}
		if (retry == true) 
		{
			if (check_num != 0 )
			{
				printk(KERN_ERR "sis_check_ready: retry_count- %d\n", check_num);
				check_num--;
			}
			else
			{
				printk(KERN_ERR "sis_check_ready: I2C not ready\n");
			}
			msleep(50);
		}
	}while(retry);

	if (retry == false) 
	{
		return true;
	}
	return false;
}

uint8_t sis_check_fw_mode(struct i2c_client *client, uint8_t mode)
{
  	int ret;
	uint8_t tmpbuf[MAX_BYTE] = {0};
	uint8_t cmd[5] = {SIS_CMD_POWERMODE, 0x03, mode, 0, 0};
				   // command, bytecount, power mode, CRC,CRC
	write_crc(cmd, 2, 2);

#if 0
	printk(KERN_INFO "command:%02x\n", mode);
	PrintBuffer(0, 16, cmd);
#endif

	ret = sis_sent_command_to_fw(client, FIVE_BYTE, cmd, MAX_BYTE, tmpbuf, __func__);

	if (ret > -1)
	{
#if 0
		printk(KERN_INFO "get:\n");
		PrintBuffer(0, 16, tmpbuf);
#endif
		if (tmpbuf[0] == 0x03)
		{
			return tmpbuf[1];
		}
	}
	return -1;
}

void sis_fw_softreset(struct i2c_client *client)
{
	//softreset to get ITO base
	uint8_t tmpbuf[MAX_BYTE] = {0};
	uint8_t wcmd = SIS_CMD_SOFTRESET;
	int ret = 0;

	ret = sis_sent_command_to_fw(client, ONE_BYTE, &wcmd, MAX_BYTE, tmpbuf, __func__);

	if (ret > -1)
	{
#if 0
		printk(KERN_INFO "SOFTRESET: ");
		PrintBuffer(0, 16, tmpbuf);
#endif
		if(tmpbuf[0] == 0x04 && tmpbuf[1] == 0x0 && tmpbuf[2] == 0x80)
		{
			sis_check_fw_ready(client);
		}
		else
		{
			printk(KERN_ERR "sis_fw_softreset: SOFTRESET NACK %d\n", ret);
		}
	}
}
#endif //CONFIG_FW_SUPPORT_POWERMODE

void sis_sent_zero_command(struct i2c_client *client)
{
	/* skip the waiting time of recieve update FW command in bootloader */
	int ret = 0;
	int retry = 5;
	unsigned char read_cmd = SIS_CMD_NORMAL;
	unsigned char rdata[MAX_BYTE] = {0};
	do
	{	
		ret = sis_sent_command_to_fw(client, ONE_BYTE, &read_cmd, MAX_BYTE, rdata, __func__);
		if (ret < 0)
		{
			printk(KERN_ERR "sis_sent_zero_command: retry - %d\n", retry);
			retry--;
		}
	}
	while(ret < 0 && retry > 0);
}

#ifdef _STD_RW_IO
#define BUFFER_SIZE MAX_BYTE
ssize_t sis_cdev_write( struct file *file, const char __user *buf, size_t count, loff_t *f_pos )
{
	 int ret = 0;
	 char *kdata;
	 char cmd;
	 printk(KERN_INFO "sis_cdev_write.\n");
	 
	 if (ts_bak == 0)
    	return -13;
    	
    ret = access_ok(VERIFY_WRITE, buf, BUFFER_SIZE);
    if (!ret) {
        printk(KERN_ERR "cannot access user space memory\n");
        return -11;
    }

	 kdata = kmalloc(BUFFER_SIZE, GFP_KERNEL);
     if (kdata == 0)
    	return -12;
    	
     ret = copy_from_user(kdata, buf, BUFFER_SIZE);
     if (ret) {
        printk(KERN_ERR "copy_from_user fail\n");
        kfree(kdata);
        return -14;
     } 
#if 0
	PrintBuffer(0, count, kdata);
#endif

	cmd = kdata[6];

    printk(KERN_INFO "io cmd=%02x\n", cmd);

//Write & Read
    ret = sis_command_for_write(ts_bak->client, count, kdata);
    if (ret < 0) {
        printk(KERN_ERR "i2c_transfer write error %d\n", ret);
		kfree(kdata);
		return -21;
	}

    if ( copy_to_user((char*) buf, kdata, BUFFER_SIZE ) )
    {
        printk(KERN_ERR "copy_to_user fail\n" );
        ret = -19;
    }

    kfree( kdata );

	return ret;
}

//for get system time
ssize_t sis_cdev_read( struct file *file, char __user *buf, size_t count, loff_t *f_pos )
{
	 int ret = 0;
	 char *kdata;
	 char cmd;
	 int i;
	 printk(KERN_INFO "sis_cdev_read.\n");
	 
	 if (ts_bak == 0)
    	return -13;
    	
    ret = access_ok(VERIFY_WRITE, buf, BUFFER_SIZE);
    if (!ret) {
        printk(KERN_ERR "cannot access user space memory\n");
        return -11;
    }

	 kdata = kmalloc(BUFFER_SIZE, GFP_KERNEL);
     if (kdata == 0)
    	return -12;
    	
     ret = copy_from_user(kdata, buf, BUFFER_SIZE);
     if (ret) {
        printk(KERN_ERR "copy_from_user fail\n");
        kfree(kdata);
        return -14;
     }    
#if 0
    PrintBuffer(0, count, kdata);
#endif
	 cmd = kdata[6];
	 //for making sure AP communicates with SiS driver
    if(cmd == 0xa2)
    {
		kdata[0] = 5;
		kdata[1] = 0;
		kdata[3] = 'S';
		kdata[4] = 'i';
		kdata[5] = 'S';
		if ( copy_to_user((char*) buf, kdata, BUFFER_SIZE ) )
		{
			printk(KERN_ERR "copy_to_user fail\n" );
			kfree( kdata );
			return -19;
		}

		kfree( kdata );
		return 3;	
	}
//Write & Read
    ret = sis_command_for_read(ts_bak->client, MAX_BYTE, kdata);
    if (ret < 0) {
        printk(KERN_ERR "i2c_transfer read error %d\n", ret);
		kfree(kdata);
		return -21;
	}

	ret = kdata[0] | (kdata[1] << 8);

/*
    for ( i = 0; i < BUFFER_SIZE - 1; i++ ) {
	    kdata[i] = kdata[i+1];
    }
*/

    printk(KERN_INFO "%d\n", ret);

    for ( i = 0; i < ret && i < BUFFER_SIZE; i++ )
    {
        printk("%02x ", kdata[i]);
    }

    printk( "\n" );

    if ( copy_to_user((char*) buf, kdata, BUFFER_SIZE ) )
    {
        printk(KERN_ERR "copy_to_user fail\n" );
        ret = -19;
    }

    kfree( kdata );

	return ret;
}

#undef BUFFER_SIZE

int sis_cdev_open(struct inode *inode, struct file *filp)
{
	printk(KERN_INFO "sis_cdev_open.\n");
	if ( ts_bak == 0 )
    	return -13;

	msleep(200);
	if (ts_bak->use_irq)
	{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
        if ((ts_bak->desc->status & IRQ_DISABLED) == IRQ_STATUS_ENABLED)
#else
		if ((ts_bak->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_ENABLED)
#endif	
		{
			disable_irq(ts_bak->client->irq);
		}
		else
		{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
			printk(KERN_INFO "sis_cdev_open: IRQ_STATUS: %x\n",(ts_bak->desc->status & IRQ_DISABLED));
#else
			printk(KERN_INFO "sis_cdev_open: IRQ_STATUS: %x\n",(ts_bak->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED));
#endif	
		}
	}
	hrtimer_cancel(&ts_bak->timer);

	flush_workqueue(sis_wq); 	   // only flush sis_wq
    
    msleep(200);

	return 0; /* success */
}

int sis_cdev_release(struct inode *inode, struct file *filp)
{
	printk(KERN_INFO "sis_cdev_release.\n");
	 msleep(200);
    if (ts_bak == 0)
    	return -13;

	if (ts_bak->use_irq)
	{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
        if ((ts_bak->desc->status & IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#else
		if ((ts_bak->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#endif	
		{
			enable_irq(ts_bak->client->irq);
		}
	}
	else
		hrtimer_start(&ts_bak->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

    return 0;
}

static const struct file_operations sis_cdev_fops = {
	.owner	= THIS_MODULE,
	.read	= sis_cdev_read,
	.write	= sis_cdev_write,
	.open	= sis_cdev_open,
	.release= sis_cdev_release,
};

static int sis_setup_chardev(struct sis_ts_data *ts)
{
	
	dev_t dev = MKDEV(sis_char_major, 0);
	int alloc_ret = 0;
	int cdev_err = 0;
	int input_err = 0;
	struct device *class_dev = NULL;
	void *ptr_err;
	
	printk("sis_setup_chardev.\n");
	
	if (ts == NULL) 
	{
	  input_err = -ENOMEM;
	  goto error;
	} 
	 // dynamic allocate driver handle
	alloc_ret = alloc_chrdev_region(&dev, 0, sis_char_devs_count, DEVICE_NAME);
	if (alloc_ret)
		goto error;
		
	sis_char_major = MAJOR(dev);
	cdev_init(&sis_char_cdev, &sis_cdev_fops);
	sis_char_cdev.owner = THIS_MODULE;
	cdev_err = cdev_add(&sis_char_cdev, MKDEV(sis_char_major, 0), sis_char_devs_count);
	
	if (cdev_err) 
		goto error;
	
	printk(KERN_INFO "%s driver(major %d) installed.\n", DEVICE_NAME, sis_char_major);
	
	// register class
	sis_char_class = class_create(THIS_MODULE, DEVICE_NAME);
	if(IS_ERR(ptr_err = sis_char_class)) 
	{
		goto err2;
	}
	
	class_dev = device_create(sis_char_class, NULL, MKDEV(sis_char_major, 0), NULL, DEVICE_NAME);
	
	if(IS_ERR(ptr_err = class_dev)) 
	{
		goto err;
	}
	
	return 0;
error:
	if (cdev_err == 0)
		cdev_del(&sis_char_cdev);
	if (alloc_ret == 0)
		unregister_chrdev_region(MKDEV(sis_char_major, 0), sis_char_devs_count);
	if(input_err != 0)
	{
		printk("sis_ts_bak error!\n");
	}
err:
	device_destroy(sis_char_class, MKDEV(sis_char_major, 0));
err2:
	class_destroy(sis_char_class);
	return -1;
}
#endif

static bool sis_i2c_test(struct i2c_client * client)
{
	int ret, retry;
	uint8_t test_data[1] = { 0 };	//only write a data address.
	
	for(retry=0; retry < 5; retry++)
	{
		char kdata[2] = {00, 00};

	//Write & Read
		ret = sis_command_for_write(client, 1, kdata);

		if (ret >= 0)
			break;
		msleep(5);
	}
	
	return ret>=0 ? true : false;
}

static int sis_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct sis_ts_data *ts = NULL;
	struct sis_i2c_rmi_platform_data *pdata = NULL;

    printk(KERN_INFO "sis_ts_probe\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C functionality not supported\n");
		return -ENODEV;
	}
#if 1
        ret = sis_i2c_test(client);
        if(!ret){
                printk("!!! SIS TP is not exist !!!\n");
                ret = -ENODEV;
                goto err_alloc_data_failed;
        }
        printk("===== SIS TP test ok=======\n");
#endif

    TPInfo = kzalloc(sizeof(struct sisTP_driver_data), GFP_KERNEL);
    if (TPInfo == NULL) 
    {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) 
	{
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts_bak = ts;

	mutex_init(&ts->mutex_wq);
	
	//1. Init Work queue and necessary buffers
	INIT_WORK(&ts->work, sis_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
    pdata = client->dev.platform_data;

	if (pdata)
		ts->power = pdata->power;
	if (ts->power) 
	{
		ret = ts->power(1);
		if (ret < 0) 
		{
			printk(KERN_ERR "sis_ts_probe power on failed\n");
			goto err_power_failed;
		}
	}

	//2. Allocate input device
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) 
	{
		ret = -ENOMEM;
		printk(KERN_ERR "sis_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = "sis_touch";//"SiS9200-i2c-touchscreen";

#ifdef CONFIG_FW_SUPPORT_POWERMODE
		//sis_check_fw_ready(client);
#endif

#ifdef _SKIP_FW_WAITING_TIME
		sis_sent_zero_command(client);	// skip the waiting time of recieve update FW command in bootloader
#endif

	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
    set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
    set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
    set_bit(ABS_MT_TRACKING_ID, ts->input_dev->absbit);

#ifdef _ANDROID_4
    set_bit(ABS_MT_PRESSURE, ts->input_dev->absbit);
    set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
    set_bit(ABS_MT_TOUCH_MINOR, ts->input_dev->absbit);
    input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, PRESSURE_MAX, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, AREA_LENGTH_LONGER, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MINOR, 0, AREA_LENGTH_SHORT, 0, 0);   
#else
    set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
    set_bit(ABS_MT_WIDTH_MAJOR, ts->input_dev->absbit);
    set_bit(ABS_MT_WIDTH_MINOR, ts->input_dev->absbit);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESSURE_MAX, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, AREA_LENGTH_LONGER, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MINOR, 0, AREA_LENGTH_SHORT, 0, 0);
#endif

    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, SIS_MAX_X, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, SIS_MAX_Y, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 15, 0, 0);
	
    /* add for touch keys */
	//set_bit(KEY_COMPOSE, ts->input_dev->keybit);
	//set_bit(KEY_BACK, ts->input_dev->keybit);
	//set_bit(KEY_MENU, ts->input_dev->keybit);
	//set_bit(KEY_HOME, ts->input_dev->keybit);

	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);//cancle mouse icon

	//3. Register input device to core
	ret = input_register_device(ts->input_dev);

	if (ret) 
	{
		printk(KERN_ERR "sis_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	
	//4. irq or timer setup
	//ret = initial_irq();
	//if (ret < 0) 
	//{

	//}
	//else
	{
		gpio_set_status(pdata->gpio_irq, gpio_status_in);
        gpio_irq_set(170, GPIO_IRQ( (INT_GPIO_0 -INT_GPIO_0), GPIO_IRQ_LOW));
		ret = request_irq(client->irq, sis_ts_irq_handler, IRQF_DISABLED, client->name, ts);
		if (ret == 0) 
		{
		   ts->use_irq = 1;
		}
		else 
		{
			dev_err(&client->dev, "request_irq failed\n");
		}
	}

	ts->desc = irq_to_desc(ts_bak->client->irq);
	
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = sis_ts_timer_func;

	if (!ts->use_irq) 
	{
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = sis_ts_early_suspend;
	ts->early_suspend.resume = sis_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	printk(KERN_INFO "sis_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	
	if (ts->use_irq)
	{
#ifdef _INT_MODE_1
		printk(KERN_INFO "sis_ts_probe: interrupt case 1 mode\n");
#else
		printk(KERN_INFO "sis_ts_probe: interrupt case 2 mode\n");
#endif
	}
	
#ifdef _STD_RW_IO
	ret = sis_setup_chardev(ts);
	if(ret)
	{
		printk( KERN_INFO"sis_setup_chardev fail\n");
	}
#endif

	printk( KERN_INFO"sis SIS_SLAVE_ADDR: %d\n", SIS_SLAVE_ADDR);

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
	return ret;
}

static int sis_ts_remove(struct i2c_client *client)
{
	struct sis_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int sis_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;
	struct sis_ts_data *ts = i2c_get_clientdata(client);
#ifdef _SMBUS_INTERFACE
	uint8_t tmpbuf[MAX_BYTE] = {0};
#endif

#ifdef CONFIG_FW_SUPPORT_POWERMODE
	int retry = 5;
	uint8_t status = -1;
#endif

	TPInfo->pre_keybit_state = 0x0;

	if (ts->use_irq)
	{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
		if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_ENABLED)
#else
		if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_ENABLED)
#endif
		{
			disable_irq(client->irq);
		}
	}
	else
		hrtimer_cancel(&ts->timer);
	flush_workqueue(sis_wq); 	   		// only flush sis_wq
//	flush_scheduled_work(); 		   	// flush all of workqueue in kernel
//	ret = cancel_work_sync(&ts->work); 	// only cancel one work(sis_ts_work_func), 
										// but there maybe are others in workqueue.
/*
	// For cancel_work_sync()
	if (ret && ts->use_irq) //if work was pending disable-count is now 2
	{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
		if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#else
		if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#endif
		{
			enable_irq(client->irq);
		}
	}
*/

#ifdef CONFIG_FW_SUPPORT_POWERMODE
#ifdef _SMBUS_INTERFACE
		uint8_t cmd[5] = {SIS_CMD_POWERMODE, 0x03, WRITE_DEEPSLEEP_MODE, 0, 0};
						// command, bytecount, power mode, CRC,CRC
		write_crc(cmd, 2, 2);
#else
		sis_check_fw_mode(client, WRITE_DEEPSLEEP_MODE); // Change to Deepsleep Mode
		
		while (retry > 0 && status != DEEPSLEEP_MODE)
		{
			//msleep(50);
			status = sis_check_fw_mode(client, READ_POWERMODE) & MSK_POWERMODE;
			printk(KERN_INFO "sis_power_saving_status: %x\n", status);	
			
			if (status != DEEPSLEEP_MODE)
			{
				if (retry == 1)
					printk(KERN_ERR "sis_ts_suspend: change mode failed\n");
				else
					printk(KERN_ERR "sis_ts_suspend: change mode retry - %d\n", retry);
				retry--;
			}
		}
#endif
#endif

#if 0
	/* Turn off SiS Chip*/
	/* TODO */
	gpio_direction_output(TOUCH_RESET_PIN, 0);
	printk(KERN_INFO "[MSI TOUCH] SiS Touch Reset Low\n");
//	msleep(5);
	gpio_direction_output(TOUCH_POWER_PIN, 0);
	printk(KERN_INFO "[MSI TOUCH] SiS Touch Power off\n");
#endif

	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0)
			printk(KERN_ERR "sis_ts_suspend power off failed\n");
	}
	
	return 0;
}

static int sis_ts_resume(struct i2c_client *client)
{
	int ret = 0;
	struct sis_ts_data *ts = i2c_get_clientdata(client);
#ifdef _SMBUS_INTERFACE
	uint8_t tmpbuf[MAX_BYTE] = {0};
	uint8_t cmd[5] = {0};
	uint16_t crc = 0;
#endif

#ifdef CONFIG_FW_SUPPORT_POWERMODE
	int retry = 5;
	uint8_t status = -1;
#endif
	printk("sis_ts_resume\n");

	if (ts->power)
	{
		ret = ts->power(1);
		if (ret < 0)
			printk(KERN_ERR "sis_ts_resume power on failed\n");
	}

#if 0
	/* Turn on SiS Chip*/
	/* TODO */
	gpio_direction_output(TOUCH_POWER_PIN, 1);
	printk(KERN_INFO "[MSI TOUCH] SiS Touch Power on\n");
	msleep(5);
	gpio_direction_output(TOUCH_RESET_PIN, 1);
	printk(KERN_INFO "[MSI TOUCH] SiS Touch Reset HI\n");
	msleep(5);
	gpio_direction_output(TOUCH_RESET_PIN, 0);
	printk(KERN_INFO "[MSI TOUCH] SiS Touch Reset Low\n");
	msleep(5);
	gpio_direction_output(TOUCH_RESET_PIN, 1);
	printk(KERN_INFO "[MSI TOUCH] SiS Touch Reset HI\n");
#endif

#ifdef CONFIG_FW_SUPPORT_POWERMODE
	status = sis_check_fw_mode(client, READ_POWERMODE) & MSK_POWERMODE;
	if( status != ACTIVE_MODE )
	{
#ifdef _SMBUS_INTERFACE
		uint8_t cmd[5] = {SIS_CMD_POWERMODE, 0x03, WRITE_ACTIVE_MODE, 0, 0}
						// command, bytecount, power mode, CRC,CRC
		write_crc(cmd, 2, 2);
		ret = i2c_smbus_read_block_data(client, cmd, tmpbuf);
#else
		sis_check_fw_mode(client, WRITE_ACTIVE_MODE); //Change to Active Mode

		while (retry > 0 && status != ACTIVE_MODE)
		{
			//msleep(50);
			status = sis_check_fw_mode(client, READ_POWERMODE) & MSK_POWERMODE;			
			printk(KERN_INFO "sis_power_saving_status: %x\n", status);	
					
			if (status != ACTIVE_MODE)
			{
				if (retry == 1)
					printk(KERN_ERR "sis_ts_suspend: change mode failed\n");
				else
					printk(KERN_ERR "sis_ts_suspend: change mode retry - %d\n", retry);
				retry--;
			}
		}
#endif
		sis_fw_softreset(client);
	}
	else
	{
		printk(KERN_INFO "sis_ts_resume: Active mode\n");
	}
#endif
	
#ifdef _SKIP_FW_WAITING_TIME	
	sis_sent_zero_command(client);	// skip the waiting time of recieve update FW command in bootloader
#endif

	if (ts->use_irq)
	{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
		if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#else
		if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#endif
		{
			enable_irq(client->irq);
		}
	}
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	resume_flag = 1;
	printk("resume over\n");

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sis_ts_early_suspend(struct early_suspend *h)
{
	struct sis_ts_data *ts;
	TPInfo->pre_keybit_state = 0x0;
	ts = container_of(h, struct sis_ts_data, early_suspend);
	sis_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void sis_ts_late_resume(struct early_suspend *h)
{
	struct sis_ts_data *ts;
	ts = container_of(h, struct sis_ts_data, early_suspend);
	sis_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id sis_ts_id[] = {
	{ SIS_I2C_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sis_ts_id);

static struct i2c_driver sis_ts_driver = {
	.probe		= sis_ts_probe,
	.remove		= sis_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= sis_ts_suspend,
	.resume		= sis_ts_resume,
#endif
#ifdef CONFIG_X86
    .class      = I2C_CLASS_HWMON,
    .detect		= sis_ts_detect,
	.address_list	= normal_i2c,
#endif
	.id_table	= sis_ts_id,
	.driver = {
		.name	= SIS_I2C_NAME,
	},
};

static int __devinit sis_ts_init(void)
{
	printk( KERN_INFO "sis_ts_init\n" );
	sis_wq = create_singlethread_workqueue("sis_wq");

	if (!sis_wq)
		return -ENOMEM;
	return i2c_add_driver(&sis_ts_driver);
}

#ifdef CONFIG_X86
/* Return 0 if detection is successful, -ENODEV otherwise */
static int sis_ts_detect(struct i2c_client *client,
		       struct i2c_board_info *info)
{
	const char *type_name;
    printk(KERN_INFO "sis_ts_detect\n");
	type_name = "sis_i2c_ts";
	strlcpy(info->type, type_name, I2C_NAME_SIZE);
	return 0;
}
#endif

static void __exit sis_ts_exit(void)
{
#ifdef _STD_RW_IO
	dev_t dev;
#endif

	printk(KERN_INFO "sis_ts_exit\n");
	i2c_del_driver(&sis_ts_driver);
	if (sis_wq)
		destroy_workqueue(sis_wq);

#ifdef _STD_RW_IO
	dev = MKDEV(sis_char_major, 0);
	cdev_del(&sis_char_cdev);
	unregister_chrdev_region(dev, sis_char_devs_count);
	device_destroy(sis_char_class, MKDEV(sis_char_major, 0));
	class_destroy(sis_char_class);
#endif
}

module_init(sis_ts_init);
module_exit(sis_ts_exit);
MODULE_DESCRIPTION("SiS 9200 Family Touchscreen Driver");
MODULE_LICENSE("GPL");
