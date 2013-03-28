/* 
 * drivers/input/touchscreen/AW5306_ts.c
 *
 * FocalTech AW5306 TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 * VERSION      	DATE			AUTHOR
 *    1.0		  2010-01-05			WenFS
 *
 * note: only support mulititouch	Wenfs 2010-10-01
 */



#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <mach/gpio.h>
#include <linux/ft5x06_ts.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <asm/io.h>


//**************************************add by zyc 20130103
#include <mach/gpio_data.h>
//*************************************


//static int AW5306_write_reg(u8 addr, u8 para);
//static int AW5306_read_reg(u8 addr, u8 *pdata);
static struct i2c_client *this_client;
static int AW5306_printk_enable_flag=0;

#include "AW5306_Drv.h"
#include "AW5306_userpara.h"

#define AW5306_EVENT_MAX	5
#define PRESS_MAX 255
//#define CONFIG_TOUCH_PANEL_KEY
#define CONFIG_AW5306_MULTITOUCH

//****************************
#define TOUCH_KEY_SUPPORT
#define TOUCH_KEY_FOR_EVB13
#define TOUCH_KEY_NUMBER 5
//****************************


#define AW5306_NAME	"aw5306"//"synaptics_i2c_rmi"//"synaptics-rmi-ts"// 
#define I2C_CTPM_ADDRESS        (0x39)
#define CTP_NAME			AW5306_NAME
#define SCREEN_MAX_X 		800//1024
#define SCREEN_MAX_Y 		480//600

struct ts_event {
	//u8 id;
	//s16	x;
	//s16	y;
	//s16	z;
	//s16     w;
        int     x[5];
        int     y[5];
        int     pressure;
        int     touch_point;
        int     pre_point;
        int     touch_ID[5];
};

struct AW5306_ts_data {
	struct input_dev	*input_dev;
	struct ts_event	event;
	u8 event_num;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct early_suspend	early_suspend;
    struct timer_list touch_timer;
};

#define AW5306_dbg(fmt, args...)  { if(AW5306_printk_enable_flag) \
					printk("[AW5306]: " fmt, ## args); }

//#define AC_DETECT_IN_TOUCH_DRIVER
static int ac_detect_flag_current=0;
static int ac_detect_flag_old=3;
#ifdef AC_DETECT_IN_TOUCH_DRIVER
#if 0
static void Ac_Detect_In_Touch_Driver(void)
{
	unsigned char ver;
	int ac_detect_flag_temp=0;
	ac_detect_flag_temp=focaltechPdata2->Ac_is_connect();
	ac_detect_flag_current=ac_detect_flag_temp;
	if(ac_detect_flag_current!=ac_detect_flag_old)
		{
		if(1==focaltechPdata2->Ac_is_connect())
			AW5306_write_reg(0xb2,0x1);
		else
			AW5306_write_reg(0xb2,0x0);
		ac_detect_flag_old=ac_detect_flag_current;
		}
	if(1==AW5306_printk_enable_flag)
		{
		AW5306_read_reg(0xb2, &ver);
		printk("reg 0xb2=%d\n",ver);
		}
}
#endif
#endif

#if 0
static ssize_t AW5306_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct capts *ts = (struct capts *)dev_get_drvdata(dev);

    if (!strcmp(attr->attr.name, "AW5306PrintFlag")) {
        memcpy(buf, &AW5306_printk_enable_flag,sizeof(AW5306_printk_enable_flag));
        printk("buf[0]=%d, buf[1]=%d\n", buf[0], buf[1]);
        return sizeof(AW5306_printk_enable_flag);
    }
    return 0;
}

static ssize_t AW5306_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct capts *ts = (struct capts *)dev_get_drvdata(dev);

	if (!strcmp(attr->attr.name, "AW5306PrintFlag")) {
		printk("buf[0]=%d, buf[1]=%d\n", buf[0], buf[1]);
		if (buf[0] == '0') AW5306_printk_enable_flag = 0;
		if (buf[0] == '1') AW5306_printk_enable_flag = 1;
		if (buf[0] == '2') {
			AW5306_printk_enable_flag=2;
			if(focaltechPdata2->power){
				focaltechPdata2->power(0);
				msleep(50);
				focaltechPdata2->power(1);
				msleep(200);
			}
		}
		if (buf[0] == '3') {
			u8 data;
			AW5306_write_reg(0xa5, 0x03);
			printk("set reg[0xa5] = 0x03\n");
			msleep(20);
			AW5306_read_reg(0xa5, &data);
			printk("read back: reg[0xa5] = %d\n", data);
    }
  }
	return count;
}

static DEVICE_ATTR(AW5306PrintFlag, S_IRWXUGO, AW5306_read, AW5306_write);
static struct attribute *AW5306_attr[] = {
    &dev_attr_AW5306PrintFlag.attr,
    NULL
};


static struct attribute_group AW5306_attr_group = {
    .name = NULL,
    .attrs = AW5306_attr,
};


/***********************************************************************************************
Name	:	AW5306_i2c_rxdata 

Input	:	*rxdata
                     *length

Output	:	ret

function	:	

***********************************************************************************************/
static int AW5306_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

    //msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}
#endif

extern char AW5306_CLB();
extern void AW5306_CLB_GetCfg();
extern STRUCTCALI       AW_Cali;
extern AW5306_UCF   AWTPCfg;
extern STRUCTBASE		AW_Base;
extern short	Diff[NUM_TX][NUM_RX];
extern short	adbDiff[NUM_TX][NUM_RX];
static unsigned char suspend_flag=0; //0: sleep out; 1: sleep in
static short tp_idlecnt = 0;
static char tp_SlowMode = 0;

#ifdef TOUCH_KEY_SUPPORT
static int key_tp  = 0;
static int key_val = 0;
#endif

int I2C_WriteByte(u8 addr, u8 para)
{
	int ret;
	u8 buf[3];

	buf[0] = addr;
	buf[1] = para;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= buf,
		},
	};

	ret = i2c_transfer(this_client->adapter, msg, 1);

	return ret;
}

unsigned char I2C_ReadByte(u8 addr)
{
	int ret;
	u8 buf[2] = {0};

	buf[0] = addr;
	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= buf,
		},
	};

	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);

	return buf[0];
  
}

unsigned char I2C_ReadXByte( unsigned char *buf, unsigned char addr, unsigned short len)
{
	int ret,i;
	u8 rdbuf[512] = {0};

	rdbuf[0] = addr;
	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rdbuf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= rdbuf,
		},
	};

	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	for(i = 0; i < len; i++)
	{
		buf[i] = rdbuf[i];
	}

    return ret;
}

unsigned char I2C_WriteXByte( unsigned char *buf, unsigned char addr, unsigned short len)
{
	int ret,i;
	u8 wdbuf[512] = {0};

	wdbuf[0] = addr;
	for(i = 0; i < len; i++)
	{
		wdbuf[i+1] = buf[i];
	}
	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= len+1,
			.buf	= wdbuf,
		}
	};

	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 1);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	

    return ret;
}

void AW_Sleep(unsigned int msec)
{
	msleep(msec);
}
static ssize_t AW5306_get_Cali(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW5306_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW5306_write_reg(struct device* cd,struct device_attribute *attr, char *buf, size_t count);
static ssize_t AW5306_get_Base(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW5306_get_Diff(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW5306_get_adbBase(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW5306_get_adbDiff(struct device* cd,struct device_attribute *attr, char* buf);

static DEVICE_ATTR(cali,  S_IRUGO | S_IWUSR, AW5306_get_Cali,    NULL);
static DEVICE_ATTR(getreg,  S_IRUGO | S_IWUSR, AW5306_get_reg,    AW5306_write_reg);
static DEVICE_ATTR(base,  S_IRUGO | S_IWUSR, AW5306_get_Base,    NULL);
static DEVICE_ATTR(diff, S_IRUGO | S_IWUSR, AW5306_get_Diff,    NULL);
static DEVICE_ATTR(adbbase,  S_IRUGO | S_IWUSR, AW5306_get_adbBase,    NULL);
static DEVICE_ATTR(adbdiff, S_IRUGO | S_IWUSR, AW5306_get_adbDiff,    NULL);

static ssize_t AW5306_get_Cali(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char i,j;
	ssize_t len = 0;
	
	len += snprintf(buf+len, PAGE_SIZE-len,"*****AW5306 Calibrate data*****\n");
	len += snprintf(buf+len, PAGE_SIZE-len,"TXOFFSET:");
	
	for(i=0;i<10;i++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "0x%02X ", AW_Cali.TXOFFSET[i]);
	}
	
	len += snprintf(buf+len, PAGE_SIZE-len,  "\n");
	len += snprintf(buf+len, PAGE_SIZE-len,  "RXOFFSET:");

	for(i=0;i<6;i++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "0x%02X ", AW_Cali.RXOFFSET[i]);
	}

	len += snprintf(buf+len, PAGE_SIZE-len,  "\n");
	len += snprintf(buf+len, PAGE_SIZE-len,  "TXCAC:");

	for(i=0;i<20;i++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "0x%02X ", AW_Cali.TXCAC[i]);
	}

	len += snprintf(buf+len, PAGE_SIZE-len,  "\n");
	len += snprintf(buf+len, PAGE_SIZE-len,  "RXCAC:");

	for(i=0;i<12;i++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "0x%02X ", AW_Cali.RXCAC[i]);
	}

	len += snprintf(buf+len, PAGE_SIZE-len,  "\n");
	len += snprintf(buf+len, PAGE_SIZE-len,  "TXGAIN:");

	for(i=0;i<20;i++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "0x%02X ", AW_Cali.TXGAIN[i]);
	}

	for(i=0;i<AWTPCfg.TX_LOCAL;i++)
	{
		for(j=0;j<AWTPCfg.RX_LOCAL;j++)
		{
			len += snprintf(buf+len, PAGE_SIZE-len, "%4d ", AW_Cali.SOFTOFFSET[i][j]);
		}
		len += snprintf(buf+len, PAGE_SIZE-len,  "\n");
	}
	return len;
	
}

static ssize_t AW5306_get_adbBase(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char i,j;
	unsigned short base;
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "base: \n");
	for(i=0;i< AWTPCfg.TX_LOCAL;i++)
	{
		for(j=0;j<AWTPCfg.RX_LOCAL;j++)
		{
			AW5306_GetBase(&base,i,j);	
			len += snprintf(buf+len, PAGE_SIZE-len, "%4d, ",base+AW_Cali.SOFTOFFSET[i][j]);
		}
		len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	}
	
	return len;
}

static ssize_t AW5306_get_Base(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char i,j;
	ssize_t len = 0;

	*(buf+len) = AWTPCfg.TX_LOCAL;
	len++;
	*(buf+len) = AWTPCfg.RX_LOCAL;
	len++;
	
	for(i=0;i< AWTPCfg.TX_LOCAL;i++)
	{
		for(j=0;j<AWTPCfg.RX_LOCAL;j++)
		{
			*(buf+len) = (char)(((AW_Base.Base[i][j]+AW_Cali.SOFTOFFSET[i][j]) & 0xFF00)>>8);
			len++;
			*(buf+len) = (char)((AW_Base.Base[i][j]+AW_Cali.SOFTOFFSET[i][j]) & 0x00FF);
			len++;
		}
	}
	return len;

}

static ssize_t AW5306_get_adbDiff(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char i,j;
	short diff;
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "Diff: \n");
	for(i=0;i< AWTPCfg.TX_LOCAL;i++)
	{
		for(j=0;j<AWTPCfg.RX_LOCAL;j++)
		{
			AW5306_GetDiff(&diff,i,j);	
			len += snprintf(buf+len, PAGE_SIZE-len, "%4d, ",diff);
		}
		len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	}
	
	return len;
}

static ssize_t AW5306_get_Diff(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char i,j;
	ssize_t len = 0;

	*(buf+len) = AWTPCfg.TX_LOCAL;
	len++;
	*(buf+len) = AWTPCfg.RX_LOCAL;
	len++;
	
	for(i=0;i< AWTPCfg.TX_LOCAL;i++)
	{
		for(j=0;j<AWTPCfg.RX_LOCAL;j++)
		{
			*(buf+len) = (char)((adbDiff[i][j] & 0xFF00)>>8);
			len++;
			*(buf+len) = (char)(adbDiff[i][j] & 0x00FF);
			len++;
		}
	}
	return len;
}

static ssize_t AW5306_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
	u8 reg_val[1];
	ssize_t len = 0;
	u8 i;

	for(i=1;i<0x7F;i++)
	{
		reg_val[0] = I2C_ReadByte(i);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg%02X = 0x%02X, ", i,reg_val[0]);
	}

	return len;

}

static ssize_t AW5306_write_reg(struct device* cd,struct device_attribute *attr, char *buf, size_t count)
{
	int databuf[2];
	
	if(2 == sscanf(buf, "%d %d", &databuf[0], &databuf[1]))
	{ 
		I2C_WriteByte((u8)databuf[0],(u8)databuf[1]);
	}
	else
	{
		printk("invalid content: '%s', length = %d\n", buf, count);
	}
	return count; 
}

static int AW5306_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	//TS_DBG("%s", __func__);
	
	err = device_create_file(dev, &dev_attr_cali);
	err = device_create_file(dev, &dev_attr_getreg);
	err = device_create_file(dev, &dev_attr_base);
	err = device_create_file(dev, &dev_attr_diff);
	err = device_create_file(dev, &dev_attr_adbbase);
	err = device_create_file(dev, &dev_attr_adbdiff);
	return err;
}

static void AW5306_ts_release(void)
{
	struct AW5306_ts_data *data = i2c_get_clientdata(this_client);
#ifdef CONFIG_AW5306_MULTITOUCH	
	#ifdef TOUCH_KEY_SUPPORT
	  
	if(1 == key_tp){
	#ifdef TOUCH_KEY_FOR_EVB13
		if(key_val == 1){
			input_report_key(data->input_dev, KEY_MENU, 0);
			input_sync(data->input_dev);  
        }
        else if(key_val == 2){
			input_report_key(data->input_dev,KEY_BACK, 0);
			input_sync(data->input_dev);  
		//	printk("===KEY 2   upupupupupu===++=\n");     
        }
        else if(key_val == 3){
			input_report_key(data->input_dev,KEY_SEARCH, 0);
			input_sync(data->input_dev);  
			//printk("===KEY 3   upupupupupu===++=\n");     
        }
        else if(key_val == 4){
			input_report_key(data->input_dev, KEY_HOMEPAGE, 0);
			input_sync(data->input_dev);  
		//	printk("===KEY 4   upupupupupu===++=\n");     
        }
        else if(key_val == 5){
			input_report_key(data->input_dev, KEY_VOLUMEDOWN, 0);
			input_sync(data->input_dev);  
		//	printk("===KEY 5   upupupupupu===++=\n");     
        }
        else if(key_val == 6){
			input_report_key(data->input_dev, KEY_VOLUMEUP, 0);
			input_sync(data->input_dev);  
		//	printk("===KEY 6   upupupupupu===++=\n");     
        }
	#endif // endof-TOUCH_KEY_FOR_EVB13
	#ifdef TOUCH_KEY_FOR_ANGDA
	   input_report_key(data->input_dev, key_val, 0);
	#endif //endof-TOUCH_KEY_FOR_ANGDA
//		input_report_key(data->input_dev, key_val, 0);
		//printk("Release Key = %d\n",key_val);		
		//printk("Release Keyi+++++++++++++++++++++++++++++\n");		
	} else{
	    input_report_key(data->input_dev, BTN_TOUCH, 0);
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		//printk("AW5306 TOUCH RELEASE!!!\n");
	}
	#else //not-TOUCH_KEY_SUPPORT
	//input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	//printk("AW5306 TOUCH RELEASE!!!\n");
	#endif

#else  //not-CONFIG_AW5306_MULTITOUCH
	input_report_abs(data->input_dev, ABS_PRESSURE, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
#endif
	
	input_sync(data->input_dev);
	return;

}

static void AW5306_report_multitouch(void)
{
	struct AW5306_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;

#ifdef TOUCH_KEY_SUPPORT
	if(1 == key_tp){
		return;
	}
#endif
	input_report_key(data->input_dev, BTN_TOUCH, 1);
	switch(event->touch_point) {
	case 5:
		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID[4]);	
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->y[4]);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->x[4]);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
	//	printk("=++==x5 = %d,y5 = %d ====\n",event->x[4],event->y[4]);
	case 4:
		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID[3]);	
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->y[3]);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->x[3]);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
	//	printk("===x4 = %d,y4 = %d ====\n",event->x[3],event->y[3]);
	case 3:
		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID[2]);	
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->y[2]);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->x[2]);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
	//	printk("===x3 = %d,y3 = %d ====\n",event->x[2],event->y[2]);
	case 2:
		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID[1]);	
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->y[1]);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->x[1]);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
	//	printk("===x2 = %d,y2 = %d ====\n",event->x[1],event->y[1]);
	case 1:
		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID[0]);	
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->y[0]);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->x[0]);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
		//printk("===x1 = %d,y1 = %d ====\n",event->x[0],event->y[0]);
		break;
	default:
//		print_point_info("==touch_point default =\n");
		break;
	}
	
	input_sync(data->input_dev);
	dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
		event->x[0], event->y[0], event->x[1], event->y[1]);
	return;
}


static int AW5306_read_data(void)
{
	struct AW5306_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	 int Pevent;
    int i = 0;
	
	AW5306_TouchProcess();
	
	//memset(event, 0, sizeof(struct ts_event));
	event->touch_point = AW5306_GetPointNum();

	for(i=0;i<event->touch_point;i++)
	{
		AW5306_GetPoint(&event->x[i],&event->y[i],&event->touch_ID[i],&Pevent,i);
		//printk("key%d = %d,%d,%d \n",i,event->x[i],event->y[i],event->touch_ID[i] );
		//printk("SCREEN_MAX_X %d, SCREEN_MAX_Y %d revert_x %d, revert_y %d, exchang_xy %d \n", SCREEN_MAX_X, SCREEN_MAX_Y,revert_x_flag, revert_y_flag, exchange_x_y_flag);
	}
    	
	if (event->touch_point == 0) 
	{
		if(tp_idlecnt <= FAST_FRAME*5)
		{
			tp_idlecnt++;
		}
		if(tp_idlecnt > FAST_FRAME*5)
		{
			tp_SlowMode = 1;
		}
		
		if (event->pre_point != 0)
		{
			event->pre_point = 0;
			AW5306_ts_release();
		}
		
		return 1; 
	}
	else
	{
		tp_SlowMode = 0;
		tp_idlecnt = 0;
		event->pre_point = event->touch_point; 
	
		event->pressure = 200;
	
		dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
		event->x[0], event->y[0], event->x[1], event->y[1]);
		
		return 0;
	}
}


#ifndef CONFIG_AW5306_MULTITOUCH
static void AW5306_report_singletouch(void)
{
	struct AW5306_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	
	if (event->touch_point == 1) {
		input_report_abs(data->input_dev, ABS_X, event->x1);
		input_report_abs(data->input_dev, ABS_Y, event->y1);
		input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
	}
	input_report_key(data->input_dev, BTN_TOUCH, 1);
	input_sync(data->input_dev);
	dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
		event->x1, event->y1, event->x2, event->y2);

	return;
}
#endif

#ifdef TOUCH_KEY_SUPPORT
static void AW5306_report_touchkey(void)
{
	struct AW5306_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	//print_point_info("x=%d===Y=%d\n",event->x1,event->y1);

#ifdef TOUCH_KEY_FOR_ANGDA
	if((1==event->touch_point)&&(event->x1 > TOUCH_KEY_X_LIMIT)){
		key_tp = 1;
		if(event->x1 < 40){
			key_val = 1;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);  
			print_point_info("===KEY 1====\n");
		}else if(event->y1 < 90){
			key_val = 2;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);     
			print_point_info("===KEY 2 ====\n");
		}else{
			key_val = 3;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);     
			print_point_info("===KEY 3====\n");	
		}
	} else{
		key_tp = 0;
	}
#endif
#ifdef TOUCH_KEY_FOR_EVB13
	if((1==event->touch_point)&&((event->x[0] > 515)&&(event->x[0]<533))){
		
		
			//printk("===KEY touch ++++++++++====++=\n");     
		key_tp = 1;
		if(event->y[0] < 80&&event->y[0] > 30){
			key_val = 1;
			input_report_key(data->input_dev, KEY_MENU, 1);
			input_sync(data->input_dev);  
		//	printk("===KEY 1===++=\n");     
		}else if((event->y[0] < 230)&&(event->y[0]>170)){
			key_val = 2;
			input_report_key(data->input_dev, KEY_BACK, 1);
			input_sync(data->input_dev);     
		//	printk("===KEY 2 ====\n");
		}else if((event->y[0] < 345)&&(event->y[0]>300)){
			key_val = 3;
			input_report_key(data->input_dev, KEY_SEARCH, 1);
			input_sync(data->input_dev);     
		//	print_point_info("===KEY 3====\n");
		}else if ((event->y[0] < 475)&&(event->y[0]>445))	{
			key_val = 4;
			input_report_key(data->input_dev, KEY_HOMEPAGE, 1);
			input_sync(data->input_dev);     
			//printk("===KEY 4====\n");	
		}else if ((event->y[0] <620 )&&(event->y[0]>580))	{
			key_val = 5;
			input_report_key(data->input_dev, KEY_VOLUMEDOWN, 1);
			input_sync(data->input_dev);     
		//	print_point_info("===KEY 5====\n");	
		}else if ((event->y[0] < 755)&&(event->y[0]>710))	{
			key_val = 6;
			input_report_key(data->input_dev, KEY_VOLUMEUP, 1);
			input_sync(data->input_dev);     
		//	print_point_info("===KEY 6====\n");	
		}
	
}else{
		key_tp = 0;
	}
#endif

#ifdef TOUCH_KEY_LIGHT_SUPPORT
	//AW5306_lighting();
#endif
	return;
}
#endif
#if 0
#define enter_touch_screen_state() {\
	data->first_event = data->event[0];\
	data->offset = 0;\
	data->touch_count = 0;\
	input_report_key(data->input_dev, BTN_TOUCH, 1);\
	data->touch_state = TOUCH_SCREEN;\
}

#define enter_key_pre_state() {\
	data->key = key;\
	data->touch_count = 0;\
	data->touch_state = TOUCH_KEY_PRE;\
}
#endif
static void AW5306_report_value(void)
{

#ifdef TOUCH_KEY_SUPPORT
	AW5306_report_touchkey();
#endif
#ifdef CONFIG_AW5306_MULTITOUCH
	AW5306_report_multitouch();
#else	/* CONFIG_AW5306_MULTITOUCH*/
	AW5306_report_singletouch();
#endif	/* CONFIG_AW5306_MULTITOUCH*/

	return;
}	/*end AW5306_report_value*/
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void AW5306_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
	
	ret = AW5306_read_data();
	if (ret == 0) {
		AW5306_report_value();
	}
	//enable_irq(SW_INT_IRQNO_PIO);
	if (suspend_flag == 1)
	{
	AW5306_Sleep();
	}

}

void AW5306_tpd_polling(void)
 {
	struct AW5306_ts_data *AW5306_ts = i2c_get_clientdata(this_client);
 	if (suspend_flag != 1)
	{
 	if (!work_pending(&AW5306_ts->pen_event_work)) {
    queue_work(AW5306_ts->ts_workqueue, &AW5306_ts->pen_event_work);
    }
#ifdef AUTO_RUDUCEFRAME
	if(tp_SlowMode)
	{  	
		AW5306_ts->touch_timer.expires = jiffies + HZ/SLOW_FRAME;
	}
	else
	{
		AW5306_ts->touch_timer.expires = jiffies + HZ/FAST_FRAME;
	}
#else
	AW5306_ts->touch_timer.expires = jiffies + HZ/FAST_FRAME;
#endif
	add_timer(&AW5306_ts->touch_timer);
 }
   else
   {
	AW5306_Sleep();
	}
}
#ifdef CONFIG_HAS_EARLYSUSPEND
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void AW5306_ts_suspend(struct early_suspend *handler)
{
	struct AW5306_ts_data *AW5306_ts = i2c_get_clientdata(this_client);
	
	if(suspend_flag != 1)
	{
		//AW5306_Sleep();   
		printk("AW5306 SLEEP!!!");
		del_timer(&AW5306_ts->touch_timer);  
		suspend_flag = 1;
	}  
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void AW5306_ts_resume(struct early_suspend *handler)
{
	struct AW5306_ts_data *AW5306_ts = i2c_get_clientdata(this_client);
	
	if(suspend_flag != 0)
	{
		printk("AW5306 WAKE UP start!!!\n");
		AW_Sleep(20);
		//****************************
		AW5306_TP_Init();
		//****************************
		AW5306_TP_Reinit();
		
		
		
		tp_idlecnt = 0;
		tp_SlowMode = 0;
		suspend_flag = 0;
		printk("AW5306 WAKE UP finish!!!\n");
		AW5306_ts->touch_timer.expires = jiffies + HZ/FAST_FRAME;
		add_timer(&AW5306_ts->touch_timer);
	}
	//if (focaltechPdata2->key_led_ctrl)
	//	focaltechPdata2->key_led_ctrl(1);
}
#endif  //CONFIG_HAS_EARLYSUSPEND



/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int 
AW5306_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct AW5306_ts_data *AW5306_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char reg_value;
	int i;
	#if 1
        gpio_set_status(PAD_GPIOC_3, gpio_status_out);
	gpio_out(PAD_GPIOC_3, 1);
	msleep(20);
        gpio_set_status(PAD_GPIOC_3, gpio_status_out);
	gpio_out(PAD_GPIOC_3, 0);
	#else
	extern void ts_aw5306_power(int on);
	ts_aw5306_power(0);
	#endif
	msleep(20);


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}
	//client->irq =  client->dev.platform_data->irq;
	//client->irq =pdata->irq;
	printk("==kzalloc=\n");
	AW5306_ts = kzalloc(sizeof(*AW5306_ts), GFP_KERNEL);
	if (!AW5306_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	printk("==kzalloc success=\n");
	this_client = client;

	printk("client addr1 = %x", client->addr);
	client->addr =0x38;
	this_client->addr = client->addr;
	printk("client addr2 = %x", client->addr);

	reg_value = I2C_ReadByte(0x01);
	if(reg_value != 0xA8)
	{
		client->addr = 0x39;
		dev_err(&client->dev, "AW5306_ts_probe: CHIP ID NOT CORRECT\n");
		goto exit_check_functionality_failed;
	}
	
	i2c_set_clientdata(client, AW5306_ts);

	INIT_WORK(&AW5306_ts->pen_event_work, AW5306_ts_pen_irq_work);
	AW5306_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!AW5306_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
 
//	__gpio_as_irq_fall_edge(pdata->intr);		//
	printk("==enable Irq=\n");
    //if (pdata->init_irq) 
    //{
    //    pdata->init_irq();
    //}
	printk("==enable Irq success=\n");

	disable_irq_nosync(this_client->irq);
//	disable_irq(IRQ_EINT(6));

	printk("==input_allocate_device=\n");
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	AW5306_ts->input_dev = input_dev;

	set_bit(BTN_TOUCH, input_dev->keybit);

	set_bit(EV_ABS, input_dev->evbit);	
	set_bit(EV_KEY, input_dev->evbit);
	input_set_abs_params(input_dev,ABS_MT_TRACKING_ID, 0, 5, 0, 0);	
	
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);	
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);	
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);	
	
	input_set_abs_params(input_dev,ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);	
	input_set_abs_params(input_dev,ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);	
	input_set_abs_params(input_dev,ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

//***************************************
#ifdef TOUCH_KEY_SUPPORT
	input_set_capability(input_dev,EV_KEY,KEY_BACK);
	input_set_capability(input_dev,EV_KEY,KEY_MENU);
	input_set_capability(input_dev,EV_KEY,KEY_HOMEPAGE);
	input_set_capability(input_dev,EV_KEY,KEY_SEARCH);
	input_set_capability(input_dev,EV_KEY,KEY_VOLUMEDOWN);
	input_set_capability(input_dev,EV_KEY,KEY_VOLUMEUP);
	set_bit(KEY_MENU, input_dev->keybit);
	key_tp = 0;
	
	input_dev->evbit[0] = BIT_MASK(EV_KEY);
	for (i = 1; i < TOUCH_KEY_NUMBER; i++)
		set_bit(i, input_dev->keybit);
#endif

set_bit(EV_ABS, input_dev->evbit);
set_bit(EV_KEY, input_dev->evbit);
//***************************************	

	input_dev->name		= CTP_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"AW5306_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	printk("==register_early_suspend =\n");
	AW5306_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	AW5306_ts->early_suspend.suspend = AW5306_ts_suspend;
	AW5306_ts->early_suspend.resume	= AW5306_ts_resume;
	register_early_suspend(&AW5306_ts->early_suspend);
#endif



//	err = request_irq(IRQ_EINT(6), AW5306_ts_interrupt, IRQF_TRIGGER_FALLING, "AW5306_ts", AW5306_ts);
	if (err < 0) {
		dev_err(&client->dev, "AW5306_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
        AW5306_create_sysfs(client);
//wake the CTPM
//	__gpio_as_output(GPIO_AW5306_WAKE);		
//	__gpio_clear_pin(GPIO_AW5306_WAKE);		//set wake = 0,base on system
//	 msleep(100);
//	__gpio_set_pin(GPIO_AW5306_WAKE);			//set wake = 1,base on system
//	msleep(100);
//	AW5306_set_reg(0x88, 0x05); //5, 6,7,8
//	AW5306_set_reg(0x80, 30);
//	msleep(50);
//  enable_irq(this_client->irq);
   // enable_irq(IRQ_EINT(6));
  AW_Cali.FirstFlag = 1; 
  AW5306_TP_Init();  
        //err = sysfs_create_group(&client->dev.kobj, &AW5306_attr_group);
	AW5306_ts->touch_timer.function = AW5306_tpd_polling;
	AW5306_ts->touch_timer.data = 0;
	init_timer(&AW5306_ts->touch_timer);
	AW5306_ts->touch_timer.expires = jiffies + HZ*5;
	add_timer(&AW5306_ts->touch_timer);

	
	
	
	//if (focaltechPdata2->key_led_ctrl)
	//	focaltechPdata2->key_led_ctrl(1);
	printk("==probe over =\n");
    return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(client->irq, AW5306_ts);
//	free_irq(IRQ_EINT(6), AW5306_ts);
exit_irq_request_failed:
exit_platform_data_null:
	cancel_work_sync(&AW5306_ts->pen_event_work);
	destroy_workqueue(AW5306_ts->ts_workqueue);
exit_create_singlethread:
	printk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(AW5306_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int __devexit AW5306_ts_remove(struct i2c_client *client)
{
	printk("==AW5306_ts_remove=\n");
	struct AW5306_ts_data *AW5306_ts = i2c_get_clientdata(client);
	unregister_early_suspend(&AW5306_ts->early_suspend);
	free_irq(client->irq, AW5306_ts);
//	free_irq(IRQ_EINT(6), AW5306_ts);
	input_unregister_device(AW5306_ts->input_dev);
	kfree(AW5306_ts);
	cancel_work_sync(&AW5306_ts->pen_event_work);
	destroy_workqueue(AW5306_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id AW5306_ts_id[] = {
	{ CTP_NAME, 0 },
	{}
};


MODULE_DEVICE_TABLE(i2c, AW5306_ts_id);

static struct i2c_driver AW5306_ts_driver = {
	.probe		= AW5306_ts_probe,
	.remove		= __devexit_p(AW5306_ts_remove),
	.id_table	= AW5306_ts_id,
	.driver	= {
		.name	= CTP_NAME,
		.owner	= THIS_MODULE,
	},
};

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int __init AW5306_ts_init(void)
{
	int ret;
	printk("==AW5306_ts_init==\n");
	ret = i2c_add_driver(&AW5306_ts_driver);
	printk("ret=%d\n",ret);
	return ret;
//	return i2c_add_driver(&AW5306_ts_driver);
}

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void __exit AW5306_ts_exit(void)
{
	printk("==AW5306_ts_exit==\n");
	i2c_del_driver(&AW5306_ts_driver);
}

module_init(AW5306_ts_init);
module_exit(AW5306_ts_exit);

MODULE_AUTHOR("<whj@AWINIC.com>");
MODULE_DESCRIPTION("AWINIC AW5306 TouchScreen driver");
MODULE_LICENSE("GPL");

