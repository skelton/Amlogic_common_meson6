/*
 * drivers/input/touchscreen/gsl1680.c
 *
 * Copyright (c) 2012 Shanghai Basewin
 *	Guan Yuwei<guanyuwei@basewin.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */



#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <mach/gpio_data.h>
#include <linux/jiffies.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/pm_runtime.h>

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include "gsl1680_m708a1_hld.h"
#include <linux/input/mt.h>

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <mach/irqs.h>
//#include <mach/system.h>
#include <mach/hardware.h>

static u32 id_sign[MAX_CONTACTS+1] = {0};
static u32 id_sign_old[MAX_CONTACTS+1] = {0};
static u8 id_state_flag[MAX_CONTACTS+1] = {0};
static u8 id_state_old_flag[MAX_CONTACTS+1] = {0};
static u16 x_old[MAX_CONTACTS+1] = {0};
static u16 y_old[MAX_CONTACTS+1] = {0};
static u16 x_new = 0;
static u16 y_new = 0;

//#define GLL_DEBUG
//#define GSL_TIMER
static u32 gsl1680_debug_mask = 1;


#ifdef CONFIG_TOUCHSCREEN_ITO_KEYS
#if defined(CONFIG_TOUCHSCREEN_GSL1680_MARY)
#define BTN1_X_MIN 0
#define BTN1_X_MAX 119
#define BTN1_Y_MIN 805
#define BTN1_Y_MAX 880

#define BTN2_X_MIN 120
#define BTN2_X_MAX 239
#define BTN2_Y_MIN 805
#define BTN2_Y_MAX 880

#define BTN3_X_MIN 240
#define BTN3_X_MAX 359
#define BTN3_Y_MIN 805
#define BTN3_Y_MAX 880

#define BTN4_X_MIN 360
#define BTN4_X_MAX 480
#define BTN4_Y_MIN 805
#define BTN4_Y_MAX 880

#define KEY_BTN_1 KEY_HOME
#define KEY_BTN_2 KEY_MENU
#define KEY_BTN_3 KEY_BACK
#define KEY_BTN_4 KEY_SEARCH

#else
#define BTN1_X_MIN 0
#define BTN1_X_MAX 0
#define BTN1_Y_MIN 0
#define BTN1_Y_MAX 0

#define BTN2_X_MIN 0
#define BTN2_X_MAX 0
#define BTN2_Y_MIN 0
#define BTN2_Y_MAX 0

#define BTN3_X_MIN 0
#define BTN3_X_MAX 0
#define BTN3_Y_MIN 0
#define BTN3_Y_MAX 0

#define BTN4_X_MIN 0
#define BTN4_X_MAX 0
#define BTN4_Y_MIN 0
#define BTN4_Y_MAX 0

#define KEY_BTN_1 KEY_RESERVED
#define KEY_BTN_2 KEY_RESERVED
#define KEY_BTN_3 KEY_RESERVED
#define KEY_BTN_4 KEY_RESERVED
#endif
#endif


struct gsl_ts_data {
	u8 x_index;
	u8 y_index;
	u8 z_index;
	u8 id_index;
	u8 touch_index;
	u8 data_reg;
	u8 status_reg;
	u8 data_size;
	u8 touch_bytes;
	u8 update_data;
	u8 touch_meta_data;
	u8 finger_size;
};

static struct gsl_ts_data devices[] = {
	{
		.x_index = 6,
		.y_index = 4,
		.z_index = 5,
		.id_index = 7,
		.data_reg = GSL_DATA_REG,
		.status_reg = GSL_STATUS_REG,
		.update_data = 0x4,
		.touch_bytes = 4,
		.touch_meta_data = 4,
		.finger_size = 70,
	},
};

struct gsl_ts {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
	struct workqueue_struct *wq;
	struct gsl1680_platform_data *pdata;
	struct gsl_ts_data *dd;
	u8 *touch_data;
	u8 device_id;
	u8 prev_touches;
	bool is_suspended;
	bool int_pending;
	struct mutex sus_lock;
//	uint32_t gpio_irq;
	int irq;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
#ifdef GSL_TIMER
	struct timer_list gsl_timer;
#endif

};

static int gsl1680_chip_init(void)
{
    gpio_set_status(PAD_GPIOA_6, gpio_status_out);
    gpio_out(PAD_GPIOA_6, 0);
	mdelay(10);  // > 10us
	
    gpio_out(PAD_GPIOA_6, 1);
	mdelay(120); // > 100ms

    gpio_set_status(PAD_GPIOA_16, gpio_status_in);
    gpio_irq_set(PAD_GPIOA_16, GPIO_IRQ(INT_GPIO_0-INT_GPIO_0, GPIO_IRQ_RISING));
      	
	return 0;
}

static int gsl1680_ts_suspend(void)
{
    gpio_set_status(PAD_GPIOA_6, gpio_status_out);
    gpio_out(PAD_GPIOA_6, 0);
	mdelay(5); 	
	return 0;
}

static int gsl1680_ts_resume(void)
{
	gpio_set_status(PAD_GPIOA_6, gpio_status_out);
  	gpio_out(PAD_GPIOA_6, 1);
	mdelay(20); 
  	gpio_out(PAD_GPIOA_6, 0);
	mdelay(5); 
  	gpio_out(PAD_GPIOA_6, 1);
	mdelay(20); 	
	return 0;
}

static struct gsl1680_platform_data gsl1680_dragon_pdata = {
	.ts_name = "gsl1680",
	.dis_min_x = 0,
	.dis_max_x = 1024,
	.dis_min_y = 0,
	.dis_max_y = 768,
	.min_tid = 0,
	.max_tid = 255,
	.min_touch = 0,
	.max_touch = 255,
	.min_width = 0,
	.max_width = 255,
	.nfingers = 5,
	
	.chip_init = gsl1680_chip_init,
	.suspend = gsl1680_ts_suspend,
	.resume = gsl1680_ts_resume,
};



struct i2c_client * i2c_connect_client = NULL;

static ssize_t gsl1680_debug_enable_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "0x%x", gsl1680_debug_mask);
}

static ssize_t gsl1680_debug_enable_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	if (buf[0] >= '0' && buf[0] <= '9')
	{
		gsl1680_debug_mask = (buf[0] - '0');
	}
	else if (buf[0] >= 'a' && buf[0] <= 'f')
	{
		gsl1680_debug_mask = 0x0A + (buf[0] - 'a');
	}
	else
	{
		gsl1680_debug_mask = 0;
	}
	return count;
}

//static DEVICE_ATTR(debug_enable, 0666, gsl1680_debug_enable_show, gsl1680_debug_enable_store);

static inline u16 join_bytes(u8 a, u8 b)
{
    u16 ab = 0;
    ab = ab | a;
    ab = ab << 8 | b;
    return ab;
}

static u32 gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{
    struct i2c_msg xfer_msg[2];

    xfer_msg[0].addr = client->addr;
    xfer_msg[0].len = 1;
    xfer_msg[0].flags = client->flags & I2C_M_TEN;
    xfer_msg[0].buf = &reg;

    xfer_msg[1].addr = client->addr;
    xfer_msg[1].len = num;
    xfer_msg[1].flags |= I2C_M_RD;
    xfer_msg[1].buf = buf;

	if (reg < 0x80) {
		i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg));
		msleep(5);
	}

    return i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg)) == ARRAY_SIZE(xfer_msg) ? 0 : -EFAULT;
}

static u32 gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
    struct i2c_msg xfer_msg[1];

    buf[0] = reg;

    xfer_msg[0].addr = client->addr;
    xfer_msg[0].len = num + 1;
    xfer_msg[0].flags = client->flags & I2C_M_TEN;
    xfer_msg[0].buf = buf;

    return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}

static __inline__ void fw2buf(u8 *buf, const u32 *fw)
{
    u32 *u32_buf = (int *)buf;
    *u32_buf = *fw;
}

static void gsl_load_fw(struct i2c_client *client)
{
    u8 buf[DMA_TRANS_LEN*4 + 1] = {0};
    u8 send_flag = 1;
    u8 *cur = buf + 1;
    u32 source_line = 0;
    u32 source_len = ARRAY_SIZE(GSL1680_FW);

    printk("=============gsl_load_fw start==============\n");

    for (source_line = 0; source_line < source_len; source_line++) 
    {
        /* init page trans, set the page val */
        if (GSL_PAGE_REG == GSL1680_FW[source_line].offset)
	{
            		fw2buf(cur, &GSL1680_FW[source_line].val);
            		gsl_write_interface(client, GSL_PAGE_REG, buf, 4);
            		send_flag = 1;
		}
	 else 
        {
            if (1 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
                buf[0] = (u8)GSL1680_FW[source_line].offset;

            fw2buf(cur, &GSL1680_FW[source_line].val);
            cur += 4;

            if (0 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20)) 
	    {
                gsl_write_interface(client, buf[0], buf, cur - buf - 1);
                cur = buf + 1;
            }

            send_flag++;
        }
    }

    printk("=============gsl_load_fw end==============\n");

    printk("=============gsl_load_cfg start==============\n");

    source_len = ARRAY_SIZE(GSL1680_CFG);
		
    for (source_line = 0; source_line < source_len; source_line++)
    {
        /* init page trans, set the page val */
        if (GSL_PAGE_REG == GSL1680_CFG[source_line].offset)
	 {
            fw2buf(cur, &GSL1680_CFG[source_line].val);
            gsl_write_interface(client, GSL_PAGE_REG, buf, 4);
            send_flag = 1;
        }
	 else 
	 {
            if (1 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
                buf[0] = (u8)GSL1680_CFG[source_line].offset;

            fw2buf(cur, &GSL1680_CFG[source_line].val);
            cur += 4;

            if (0 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20)) 
	    {
                gsl_write_interface(client, buf[0], buf, cur - buf - 1);
                cur = buf + 1;
            }

            send_flag++;
        }
    }

    printk("=============gsl_load_cfg end==============\n");

}


static int gsl_ts_write(struct i2c_client *client, u8 addr, u8 *pdata, int datalen)
{
	int ret = 0;
	u8 tmp_buf[128];
	unsigned int bytelen = 0;
	if (datalen > 125)
	{
		printk("%s too big datalen = %d!\n", __func__, datalen);
		return -1;
	}
	
	tmp_buf[0] = addr;
	bytelen++;
	
	if (datalen != 0 && pdata != NULL)
	{
		memcpy(&tmp_buf[bytelen], pdata, datalen);
		bytelen += datalen;
	}
	
	ret = i2c_master_send(client, tmp_buf, bytelen);
	return ret;
}

static int gsl_ts_read(struct i2c_client *client, u8 addr, u8 *pdata, unsigned int datalen)
{
	int ret = 0;

	if (datalen > 126)
	{
		printk("%s too big datalen = %d!\n", __func__, datalen);
		return -1;
	}

	/* set data address */
	ret = gsl_ts_write(client, addr, NULL, 0);
	if (ret < 0)
	{
		printk("%s set data address fail!\n", __func__);
		return ret;
	}
	
	/* read data */
	return i2c_master_recv(client, pdata, datalen);
}

static void test_i2c(struct i2c_client *client)
{
	u8 read_buf = 0;
	u8 write_buf = 0x12;
	int ret;
	ret = gsl_ts_read( client, 0xf0, &read_buf, sizeof(read_buf) );
	if  (ret  < 0)  
	{
#ifdef GLL_DEBUG
		printk("I2C transfer error!\n");
#endif
	}
	else
	{
#ifdef GLL_DEBUG
		printk("I read reg 0xf0 is %x\n", read_buf);
#endif
	}
	msleep(10);

	ret = gsl_ts_write(client, 0xf0, &write_buf, sizeof(write_buf));
	if  (ret  < 0)  
	{
#ifdef GLL_DEBUG
		printk("I2C transfer error!\n");
#endif
	}
	else
	{
#ifdef GLL_DEBUG
		printk("I write reg 0xf0 0x12\n");
#endif
	}
	msleep(10);

	ret = gsl_ts_read( client, 0xf0, &read_buf, sizeof(read_buf) );
	if  (ret  <  0 )
	{
#ifdef GLL_DEBUG
		printk("I2C transfer error!\n");
#endif
	}
	else
	{
#ifdef GLL_DEBUG
		printk("I read reg 0xf0 is 0x%x\n", read_buf);
#endif
	}
	msleep(10);

}


static void startup_chip(struct i2c_client *client)
{
	u8 tmp = 0x00;
	gsl_ts_write(client, 0xe0, &tmp, 1);
	msleep(2);
}

static void reset_chip(struct i2c_client *client)
{
	u8 buf[4] = {0x00};
	u8 tmp = 0x88;
	gsl_ts_write(client, 0xe0, &tmp, sizeof(tmp));
	msleep(2);
	tmp = 0x04;
	gsl_ts_write(client, 0xe4, &tmp, sizeof(tmp));
	msleep(2);
	gsl_ts_write(client, 0xbc, &buf[0], sizeof(buf));
	msleep(2);
}

static void check_mem_data(struct i2c_client *client)
{
	u8 write_buf;
	u8 read_buf[4]  = {0};
	struct gsl_ts *ts = i2c_get_clientdata(client);
	
	write_buf = 0x00;
	gsl_ts_write(client, 0xf0, &write_buf, sizeof(write_buf));
	gsl_ts_read(client, 0x00, &read_buf[0], sizeof(read_buf));
	gsl_ts_read(client, 0x00, &read_buf[0], sizeof(read_buf));
#ifdef GLL_DEBUG
	printk("!!!!!!!!!!!page: %x offset: %x val: %x %x %x %x\n",
				0x0, 0x0, read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
#endif		

	if (read_buf[3] != 0x1 || read_buf[2] != 0 || read_buf[1] != 0 || read_buf[0] != 0)
	{
		msleep(30);
		write_buf = 0x00;
		gsl_ts_write(client, 0xf0, &write_buf, sizeof(write_buf));
		gsl_ts_read(client, 0x00, &read_buf[0], sizeof(read_buf));
		gsl_ts_read(client, 0x00, &read_buf[0], sizeof(read_buf));
	#ifdef GLL_DEBUG
		printk("!!!!!!!!!!!page: %x offset: %x val: %x %x %x %x\n",
				0x0, 0x0, read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
	#endif
		if (read_buf[3] != 0x1 || read_buf[2] != 0 || read_buf[1] != 0 || read_buf[0] != 0)
		{
			reset_chip(client);
			gsl_load_fw(client);
			startup_chip(client);
			reset_chip(client);			
			ts->pdata->suspend();
			msleep(100);
			ts->pdata->resume();
			msleep(50);			
			reset_chip(client);
			startup_chip(client);
		}
	}

}

static void report_data(struct gsl_ts *ts, u16 x, u16 y, u8 pressure, u8 id)
{
	swap(x, y);

	if(x>=SCREEN_MAX_X||y>=SCREEN_MAX_Y)
		return;

	input_mt_slot(ts->input, id);	
	
	input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 100);
	input_report_abs(ts->input, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input, ABS_MT_POSITION_Y, y);	
    #ifdef GLL_DEBUG
	printk("#####id=%d,x=%d,y=%d######\n",id,x,y);
    #endif
	input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 100);
//	input_mt_sync(ts->input);


}

#ifdef CONFIG_TOUCHSCREEN_ITO_KEYS
#define DEBOUNCE_STRENGTH	2
static unsigned int key_code_last = 0;
static unsigned int key_code_simple[DEBOUNCE_STRENGTH];

/* just detect if this key is real key, to debounce */
static void clear_simple(void)
{
	int i = 0;
	for (i = 0; i < DEBOUNCE_STRENGTH; i++)
	{
		key_code_simple[i] = 0;
	}
}

static int is_key_real(unsigned int keycode)
{
	int ret = 0;
	int i = 0;

	if (!keycode)
	{
		clear_simple();
		return 1;
	}
	
	for (i = 0; i < DEBOUNCE_STRENGTH; i++)
	{
		if (keycode != key_code_simple[i])
		{
			key_code_simple[i] = keycode;
			break;
		}
	}
	
	if (i < DEBOUNCE_STRENGTH)
		ret = 0;
	else
		ret = 1;

	return ret;
}

static int ts_report_key(struct gsl_ts *ts, int x, int y, int pressure)
{
	unsigned int key_code = 0;
	int ret = 0;

	if (start_x >= BTN1_X_MIN && start_x<= BTN1_X_MAX && start_y >= BTN1_Y_MIN && start_y <= BTN1_Y_MAX)
	{
	}
	else if(start_x >= BTN2_X_MIN && start_x <= BTN2_X_MAX && start_y >= BTN2_Y_MIN && start_y <= BTN2_Y_MAX)
	{
	}
	else if(start_x >= BTN3_X_MIN && start_x<= BTN3_X_MAX && start_y >= BTN3_Y_MIN && start_y <= BTN3_Y_MAX)
	{
	}
#ifdef KEY_BTN_4
	else if(start_x >= BTN4_X_MIN && start_x <= BTN4_X_MAX && start_y >= BTN4_Y_MIN && start_y <= BTN4_Y_MAX)
	{
	}
#endif
	else
	{
		ret = 0;
		return ret;
	}

	if (x >= BTN1_X_MIN && x <= BTN1_X_MAX && y >= BTN1_Y_MIN && y <= BTN1_Y_MAX)
	{
		key_code = KEY_BTN_1;
	}
	else if(x >= BTN2_X_MIN && x <= BTN2_X_MAX && y >= BTN2_Y_MIN && y <= BTN2_Y_MAX)
	{
		key_code = KEY_BTN_2;
	}
	else if(x >= BTN3_X_MIN && x <= BTN3_X_MAX && y >= BTN3_Y_MIN && y <= BTN3_Y_MAX)
	{
		key_code = KEY_BTN_3;
	}
#ifdef KEY_BTN_4
	else if(x >= BTN4_X_MIN && x <= BTN4_X_MAX && y >= BTN4_Y_MIN && y <= BTN4_Y_MAX)
	{
		key_code = KEY_BTN_4;
	}
#endif
	else if(x == 0 && y == 0 && pressure == 0 && key_code_last)
	{
		key_code = key_code_last;
	}
	else
	{
		key_code = 0;
	}

	if (pressure == 0)
	{
		if (key_code_last)
		{
			input_report_key(ts->input, key_code_last, 0);
			key_code_last = 0;
			clear_simple();
			ret = 1;
		}
	}
	else
	{
		if (!is_key_real(key_code))
		{
			ret = 0;
			return ret;
		}
		if (key_code)
		{
			if (key_code_last != key_code)
			{
				if (key_code_last)
				{
					input_report_key(ts->input, key_code_last, 0);
				}

				input_report_key(ts->input, key_code, !!pressure);
				key_code_last = key_code;
				ret = 2;
			}
		}
		else
		{
			if (key_code_last)
			{
				input_report_key(ts->input, key_code_last, 0);
				key_code_last = key_code;
				clear_simple();
				ret = 1;
			}
		}
	}

	return ret;
}
#endif

static void record_point(u16 x, u16 y , u8 id)
{
	u16 x_err =0;
	u16 y_err =0;

	id_sign[id]=id_sign[id]+1;
	
	if(id_sign[id]==1)
	{
		x_old[id]=x;
		y_old[id]=y;
	}

	x = (x_old[id] + x)/2;
	y = (y_old[id] + y)/2;
		
	if(x>x_old[id])
	{
		x_err=x -x_old[id];
	}
	else
	{
		x_err=x_old[id]-x;
	}

	if(y>y_old[id])
	{
		y_err=y -y_old[id];
	}
	else
	{
		y_err=y_old[id]-y;
	}


	if( (x_err > 3 && y_err > 1) || (x_err > 1 && y_err > 3) )
	{
		x_new = x;     x_old[id] = x;
		y_new = y;     y_old[id] = y;
	}
	else
	{
		if(x_err > 3)
		{
			x_new = x;     x_old[id] = x;
		}
		else
			x_new = x_old[id];
		if(y_err> 3)
		{
			y_new = y;     y_old[id] = y;
		}
		else
			y_new = y_old[id];
	}

	if(id_sign[id]==1)
	{
		x_new= x_old[id];
		y_new= y_old[id];
	}
	
}

static void process_gsl1680_data(struct gsl_ts *ts)
{
	u8 id, touches;
	u16 x, y;
	int i = 0;

	touches = ts->touch_data[ts->dd->touch_index];
	//printk("######%d#####\n",touches);

	for(i=1;i<=MAX_CONTACTS;i++)
	{
		if(touches == 0)
			id_sign[i] = 0;	
		id_sign_old[i] =id_sign[i];
		id_state_flag[i] = 0;
	}

	for(i=0;i<(touches>(ts->pdata->nfingers)?(ts->pdata->nfingers):touches);i++)
	{
		x = join_bytes( ( ts->touch_data[ts->dd->x_index  + 4 * i + 1] & 0xf),
				ts->touch_data[ts->dd->x_index + 4 * i]);
		y = join_bytes(ts->touch_data[ts->dd->y_index + 4 * i + 1],
				ts->touch_data[ts->dd->y_index + 4 * i ]);
		id = ts->touch_data[ts->dd->id_index + 4 * i] >> 4;

		if(id<=MAX_CONTACTS)
		{
			record_point(x, y , id);
			//report_data(ts, x, y, 100, id);		
			report_data(ts, x_new, y_new, 100, id);		
			id_state_flag[id] = 1;
		}
	}
	for(i=1;i<=MAX_CONTACTS;i++)
	{	
		if( (0 == touches) || ((0 != id_state_old_flag[i]) && (0 == id_state_flag[i])) )
		{
			input_mt_slot(ts->input, i);
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
    	#ifdef GLL_DEBUG
			printk("$$$$$id=%d,state up$$$$$$n",id);
    	#endif			
			id_sign[i]=0;
		}
	}
	for(i=1;i<=MAX_CONTACTS;i++)
	{
		id_state_old_flag[i] = id_state_flag[i];
	}

	input_sync(ts->input);
	ts->prev_touches = touches;
}


static void gsl_ts_xy_worker(struct work_struct *work)
{
	int rc;
	u8 read_buf[4] = {0};

	struct gsl_ts *ts = container_of(work, struct gsl_ts,
				 work);				 

//	mutex_lock(&ts->sus_lock);
	if (ts->is_suspended == true) {
		dev_dbg(&ts->client->dev, "TS is supended\n");
		ts->int_pending = true;
//		mutex_unlock(&ts->sus_lock);
		return;
	}
//	mutex_unlock(&ts->sus_lock);

	/* read data from DATA_REG */
	rc = gsl_ts_read(ts->client, 0x80, ts->touch_data, ts->dd->data_size);
	if (rc < 0) 
	{
		dev_err(&ts->client->dev, "read failed\n");
		goto schedule;
	}

	if (ts->touch_data[ts->dd->touch_index] == 0xff) {
		goto schedule;
	}

	rc = gsl_ts_read( ts->client, 0xbc, &read_buf[0], sizeof(read_buf));
	if (rc < 0) 
	{
		dev_err(&ts->client->dev, "read 0xbc failed\n");
		goto schedule;
	}
#ifdef GLL_DEBUG
	printk("xxxxxxxx reg %x : %x %x %x %x\n",
			0xbc, read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
#endif
	if (read_buf[3] == 0 && read_buf[2] == 0 && read_buf[1] == 0 && read_buf[0] == 0)
	{
		process_gsl1680_data(ts);
	}
	else if (read_buf[3] == 0x80 && read_buf[2] == 0 && read_buf[1] == 0 && read_buf[0] == 0)
	{
		check_mem_data(ts->client);
	}
	else
	{
		msleep(100);
		gsl_ts_read( ts->client, 0xbc, &read_buf[0], sizeof(read_buf));
	#ifdef GLL_DEBUG
		printk("//////// reg %x : %x %x %x %x\n",
				0xbc, read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
	#endif
		if (read_buf[3] == 0 && read_buf[2] == 0 && read_buf[1] == 0 && read_buf[0] == 0)
		{
			process_gsl1680_data(ts);
		}
		else if (read_buf[3] == 0x80 && read_buf[2] == 0 && read_buf[1] == 0 && read_buf[0] == 0)
		{
			check_mem_data(ts->client);	
		}
		else
		{
			reset_chip(ts->client);
			startup_chip(ts->client);
		}
	}
	
schedule:
	enable_irq(ts->irq);
		
}

static irqreturn_t gsl_ts_irq(int irq, void *dev_id)
{	
	struct gsl_ts *ts = dev_id;
	//int reg_val;	
#ifdef GLL_DEBUG
		printk("[GSL1680] Enter irq!\n");
#endif

    disable_irq_nosync(ts->irq);

	if (!work_pending(&ts->work)) 
	{
		queue_work(ts->wq, &ts->work);
	}
	
	return IRQ_HANDLED;

}

#ifdef GSL_TIMER
static void gsl_timer_handle(unsigned long data)
{
	struct gsl_ts *ts = (struct gsl_ts *)data;

#ifdef GLL_DEBUG	
	printk("----------------gsl_timer_handle-----------------\n");	
#endif

	disable_irq_nosync(ts->irq);	
	check_mem_data(ts->client);
	ts->gsl_timer.expires = jiffies + 3 * HZ;
	add_timer(&ts->gsl_timer);
	enable_irq(ts->irq);
	
}
#endif

static int gsl_ts_init_ts(struct i2c_client *client, struct gsl_ts *ts)
{
	struct input_dev *input_device;
	int rc = 0;
	
	printk("[GSL1680] Enter %s\n", __func__);

	
	ts->dd = &devices[ts->device_id];

	if (!ts->pdata->nfingers) {
		dev_err(&client->dev, "Touches information not specified\n");
		return -EINVAL;
	}

	if (ts->device_id == 0) {
		if (ts->pdata->nfingers > MAX_CONTACTS) {
			dev_err(&client->dev, "Touches >=1 & <= 10\n");
			return -EINVAL;
		}
		ts->dd->data_size = ts->pdata->nfingers * ts->dd->touch_bytes +
						ts->dd->touch_meta_data;
		ts->dd->touch_index = 0;
	}

	ts->touch_data = kzalloc(ts->dd->data_size, GFP_KERNEL);
	if (!ts->touch_data) {
		pr_err("%s: Unable to allocate memory\n", __func__);
		return -ENOMEM;
	}

	ts->prev_touches = 0;

	input_device = input_allocate_device();
	if (!input_device) {
		rc = -ENOMEM;
		goto error_alloc_dev;
	}

	ts->input = input_device;
	input_device->name = ts->pdata->ts_name;
	input_device->id.bustype = BUS_I2C;
	input_device->dev.parent = &client->dev;
	input_set_drvdata(input_device, ts);

	__set_bit(EV_ABS, input_device->evbit);
	__set_bit(EV_KEY, input_device->evbit);
	__set_bit(EV_REP, input_device->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_device->propbit);
	set_bit(ABS_MT_POSITION_X, input_device->absbit);
	set_bit(ABS_MT_POSITION_Y, input_device->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, input_device->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_device->absbit);

	input_mt_init_slots(input_device, (MAX_CONTACTS+1));

	input_set_abs_params(input_device,ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_device,ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_device,ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_device,ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

#if defined(CONFIG_TOUCHSCREEN_ITO_KEYS) || defined(CONFIG_TOUCHSCREEN_IO_KEYS)
	input_set_capability(input_device, EV_KEY, KEY_BTN_1);
	input_set_capability(input_device, EV_KEY, KEY_BTN_2);
	input_set_capability(input_device, EV_KEY, KEY_BTN_3);
#ifdef KEY_BTN_4
	input_set_capability(input_device, EV_KEY, KEY_BTN_4);
#endif
#endif

	client->irq = INT_GPIO_0,
	ts->irq = client->irq;

	ts->wq = create_singlethread_workqueue("kworkqueue_ts");
	if (!ts->wq) {
		dev_err(&client->dev, "Could not create workqueue\n");
		goto error_wq_create;
	}
	flush_workqueue(ts->wq);	

	INIT_WORK(&ts->work, gsl_ts_xy_worker);

	rc = input_register_device(input_device);
	if (rc)
		goto error_unreg_device;

	return 0;

error_unreg_device:
	destroy_workqueue(ts->wq);
error_wq_create:
	input_free_device(input_device);
error_alloc_dev:
	kfree(ts->touch_data);
	return rc;
}

static int gsl_ts_suspend(struct device *dev)
{
	struct gsl_ts *ts = dev_get_drvdata(dev);
	int rc = 0;

  	printk("I'am in gsl_ts_suspend() start\n");
	
#ifdef GSL_TIMER
	printk( "gsl_ts_suspend () : delete gsl_timer\n");

	del_timer(&ts->gsl_timer);
#endif
	disable_irq_nosync(ts->irq);	
		   
	reset_chip(ts->client);
	ts->is_suspended = true;	
	//rc = cancel_delayed_work_sync(&ts->work);

	if (ts->pdata->suspend) 
	{
		rc = ts->pdata->suspend();
		if (rc < 0) 
		{
			dev_err(dev, "unable to goto suspend\n");
			return rc;
		}
	}
	msleep(10);

	return 0;
}

static int gsl_ts_resume(struct device *dev)
{
	struct gsl_ts *ts = dev_get_drvdata(dev);
	int rc = 0;
	
  printk("I'am in gsl_ts_resume() start\n");
	ts->is_suspended = false;
	if (ts->pdata->resume) 
	{
		rc = ts->pdata->resume();
		if (rc < 0) 
		{
			dev_err(dev, "unable to resume\n");
			return rc;
		}
	}
	
	msleep(30);
	check_mem_data(ts->client);
	reset_chip(ts->client);
	startup_chip(ts->client);
	
#ifdef GSL_TIMER
	printk( "gsl_ts_resume () : add gsl_timer\n");

	init_timer(&ts->gsl_timer);
	ts->gsl_timer.expires = jiffies + 3 * HZ;
	ts->gsl_timer.function = &gsl_timer_handle;
	ts->gsl_timer.data = (unsigned long)ts;
	add_timer(&ts->gsl_timer);
#endif

	enable_irq(ts->irq);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gsl_ts_early_suspend(struct early_suspend *h)
{
	struct gsl_ts *ts = container_of(h, struct gsl_ts, early_suspend);
	printk("[GSL1680] Enter %s\n", __func__);
	gsl_ts_suspend(&ts->client->dev);
}

static void gsl_ts_late_resume(struct early_suspend *h)
{
	struct gsl_ts *ts = container_of(h, struct gsl_ts, early_suspend);
	printk("[GSL1680] Enter %s\n", __func__);
	gsl_ts_resume(&ts->client->dev);
}
#endif
/*
static struct dev_pm_ops gsl_ts_pm_ops = {
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= gsl_ts_suspend,
	.resume	= gsl_ts_resume,
#endif
};
*/

static int __devinit gsl_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct gsl_ts *ts;
	struct gsl1680_platform_data *pdata = &gsl1680_dragon_pdata;
	int rc;

	printk("[GSL1680] Enter %s\n", __func__);

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

    gsl1680_chip_init();    
 
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C functionality not supported\n");
		return -ENODEV;
	}
 
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	printk("==kzalloc success=\n");

	ts->client = client;
	ts->pdata = pdata;
	i2c_set_clientdata(client, ts);
	ts->device_id = id->driver_data;

	ts->is_suspended = false;
	ts->int_pending = false;
	mutex_init(&ts->sus_lock);
	
	rc = gsl_ts_init_ts(client, ts);
	if (rc < 0) {
		dev_err(&client->dev, "GSL1680-D0 init failed\n");
		goto error_mutex_destroy;
	}	
	   
	test_i2c(ts->client);
	reset_chip(ts->client);
	gsl_load_fw(ts->client);			
	startup_chip(ts->client);
	reset_chip(ts->client);
	ts->pdata->suspend();
	msleep(100);
	ts->pdata->resume();
	msleep(50);			
	reset_chip(ts->client);	
	startup_chip(ts->client);

	rc=  request_irq(client->irq, gsl_ts_irq, IRQF_DISABLED, client->name, ts);
	if (rc < 0) {
		printk( "gsl_probe: request irq failed\n");
		goto error_req_irq_fail;
	}

#ifdef GSL_TIMER
	printk( "gsl_ts_probe () : add gsl_timer\n");

	init_timer(&ts->gsl_timer);
	ts->gsl_timer.expires = jiffies + 3 * HZ;	//¶¨Ê±3  ÃëÖÓ
	ts->gsl_timer.function = &gsl_timer_handle;
	ts->gsl_timer.data = (unsigned long)ts;
	add_timer(&ts->gsl_timer);
#endif

	/* create debug attribute */
	//rc = device_create_file(&ts->input->dev, &dev_attr_debug_enable);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = gsl_ts_early_suspend;
	ts->early_suspend.resume = gsl_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	gsl1680_debug_mask = 0;
	printk("[GSL1680] End %s\n", __func__);

	return 0;

//exit_set_irq_mode:	
error_req_irq_fail:
    free_irq(ts->irq, ts);	

error_mutex_destroy:
	mutex_destroy(&ts->sus_lock);
	input_free_device(ts->input);
	kfree(ts);
	return rc;
}

static int __devexit gsl_ts_remove(struct i2c_client *client)
{
   
    struct gsl_ts *ts = i2c_get_clientdata(client);
    printk("==gsl_ts_remove=\n");

#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif

	device_init_wakeup(&client->dev, 0);
	cancel_delayed_work_sync(&ts->work);
	free_irq(ts->irq, ts);
	destroy_workqueue(ts->wq);
	input_unregister_device(ts->input);
	mutex_destroy(&ts->sus_lock);

	//device_remove_file(&ts->input->dev, &dev_attr_debug_enable);
	
	if (ts->pdata->power)
		ts->pdata->power(0);

	kfree(ts->touch_data);
	kfree(ts);

	return 0;
}

static const struct i2c_device_id gsl_ts_id[] = {
	{"gsl1680", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, gsl_ts_id);


static struct i2c_driver gsl_ts_driver = {
	.driver = {
		.name = "gsl1680",
		.owner = THIS_MODULE,
	},
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= gsl_ts_suspend,
	.resume	= gsl_ts_resume,
#endif
	.probe		= gsl_ts_probe,
	.remove		= __devexit_p(gsl_ts_remove),
	.id_table	= gsl_ts_id,
};

static int __init gsl_ts_init(void)
{
    int ret;
    printk("==gsl_ts_init==\n");
	ret = i2c_add_driver(&gsl_ts_driver);
	printk("ret=%d\n",ret);
	return ret;
}

module_init(gsl_ts_init);

static void __exit gsl_ts_exit(void)
{

    printk("==gsl_ts_exit==\n");
    i2c_del_driver(&gsl_ts_driver);
	return;
}
module_exit(gsl_ts_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GSL1680 touchscreen controller driver");
MODULE_AUTHOR("Guan Yuwei, guanyuwei@basewin.com");
MODULE_ALIAS("platform:gsl_ts");
