/*
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * Initial Code:
 *	Robbie Cao
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <../arch/arm/mach-omap2/mux.h>
#include "afa750.h"

#define DEBUG			0
#define MAX_FAILURE_COUNT	3

#define AFA750_MAX_DELAY 200
#define SENSOR_NAME "afa750"

#define LOW_G_INTERRUPT				REL_Z
#define HIGH_G_INTERRUPT 			REL_HWHEEL
#define SLOP_INTERRUPT 				REL_DIAL
#define DOUBLE_TAP_INTERRUPT 			REL_WHEEL
#define SINGLE_TAP_INTERRUPT 			REL_MISC
#define ORIENT_INTERRUPT 			ABS_PRESSURE
#define FLAT_INTERRUPT 				ABS_DISTANCE

#define ABSMIN				-100
#define ABSMAX				100	

#define AFA750_BUF_SIZE	6
#define AFA750_DELAY_PWRON	300	/* ms, >= 300 ms */
#define AFA750_DELAY_PWRDN	1	/* ms */
#define AFA750_DELAY_SETDETECTION	AFA750_DELAY_PWRON

#define AFA750_RETRY_COUNT	3



int offset[3] = {0};
struct afa750_data *afa750_data;
static struct i2c_client *this_client;

static int dbglevel = 0;




/*********************************   Calibration *******************************/
#if defined(CONFIG_CALI_POSITIVE) || defined(CONFIG_CALI_NEGATIVE)
#define AFA750_FLOAT_SCALING 100

extern void ConsAP_OneTouchCalibration(struct i2c_client *client, void* getCali); 

struct AFA750_CALI_INFO{
	int offset[3];
	int gain[3];
};

struct AFA750_CALI_INFO sensor_data2;

s16 AFA750_Cali_X(s16 Cnt_X){
 
	 if(sensor_data2.offset[0] == 0 || sensor_data2.gain[0] == 0) return Cnt_X;
	 
     Cnt_X -= sensor_data2.offset[0];
	 
	 if (Cnt_X < 0 )
		{
			Cnt_X = (Cnt_X*sensor_data2.gain[0]/AFA750_FLOAT_SCALING);
			if(Cnt_X < -32768)
			{
				Cnt_X = -32768;
			}
		}
	 
	 return Cnt_X;
 }
 
 s16 AFA750_Cali_Y(s16 Cnt_Y){
     
	 if(sensor_data2.offset[1] == 0 || sensor_data2.gain[1] == 0) return Cnt_Y;
	 
     Cnt_Y -= sensor_data2.offset[1];
	 
	 if (Cnt_Y < 0 )
		{
			Cnt_Y = (Cnt_Y*sensor_data2.gain[1]/AFA750_FLOAT_SCALING);
			if(Cnt_Y < -32768)
			{
				Cnt_Y = -32768;
			}
		}
		
	 return Cnt_Y;
 }
 
 s16 AFA750_Cali_Z(s16 Cnt_Z){
 
	 if(sensor_data2.offset[2] == 0 || sensor_data2.gain[2] == 0) return Cnt_Z;
	 
     Cnt_Z -= sensor_data2.offset[2];
	 
	 if (Cnt_Z < 0)
		{
			Cnt_Z = (Cnt_Z*sensor_data2.gain[2]/AFA750_FLOAT_SCALING);
			if(Cnt_Z < -32768)
			{
				Cnt_Z = -32768;
			}
		}
		
	 return Cnt_Z;
 }
#endif 
/**********************************************************************************/





static int afa750_i2c_rx_data(char *buf, int len)
{
	uint8_t i;
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
			.len	= len,
			.buf	= buf,
		}
	};

	for (i = 0; i < AFA750_RETRY_COUNT; i++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= AFA750_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, AFA750_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int afa750_i2c_tx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= len,
			.buf	= buf,
		}
	};
	
	for (i = 0; i < AFA750_RETRY_COUNT; i++) {
		if (i2c_transfer(this_client->adapter, msg, 1) > 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= AFA750_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, AFA750_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int afa750_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int afa750_release(struct inode *inode, struct file *file)
{
	return 0;
}

struct afa750acc {
	short	x;
	short	y;
	short	z;
} ;

struct afa750_data {
	struct i2c_client *afa750_client;
	atomic_t delay;
	atomic_t enable;
	unsigned char mode;
	struct input_dev *input;
	struct afa750acc value;
	struct mutex value_mutex;
	struct mutex enable_mutex;
	struct mutex mode_mutex;
	struct delayed_work work;
	struct work_struct irq_work;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int IRQ;
};

static  short afa750_charge(unsigned char Mdata, unsigned char Ldata)
{
	short data;
	data = Mdata << 8 | Ldata;
	
	return data;
}

static int move_x(int var, int offset)
{
	int i = var;
	int a = - offset;

	if( i & 0x8000)
	{
		i = i & 0x7fff;
		i = 0x7fff - i;
        
		if( i > a)
		{
		 	i -= a;	
			i = 0x7fff - i;
		    i = i | 0x8000;
		}
		else
		{
			i = a - i;	
		}
	}
	else
	{
		i += a;		
	}
	
	return i;
}

static int move_y(int var, int offset)
{
	int i = var;

	if( i & 0x8000)
	{
	 	i &= 0x7fff;	
		i = 0x7fff - i;
		i += offset;
		i = 0x7fff - i;
		i |= 0x8000;
	}
	else
	{
		if( i >= offset)		
		{
			i -= offset;	
		}
		else
		{
			i = offset - i;		
			i = 0x7fff - i;
			i |= 0x8000;
		}
	}

	return i;
}

static int move(int var, int offset)
{
	if(offset > 0)
		return	move_y(var, offset);		
	else if(offset < 0)
		return move_x(var, offset);

	return var;
}

static int small(int *var) 
{
	int i = *var;

	if( i & 0x8000)
	{
		i = i & 0x7fff;
		i = 0x7fff - i;
		i = i / 0x2a;
		i = 0x7fff - i;
		i = i | 0x8000;
	}
	else
	{
		i =  i / 0x2a;	
	}

	return i;
}

static int chang_small(int *var)
{
	int i = *var;

	if( i & 0x8000)
	{
		i = i & 0x7fff;
		i = 0x7fff - i;
		i = i / 0x2a;
	}
	else
	{
		i =  i / 0x2a;	
		i = 0x7fff - i;
		i = i | 0x8000;
	}

	return i;

}


static void afa750_work_func(struct work_struct *work)
{
	struct afa750_data *afa750 = container_of((struct delayed_work *)work,
			struct afa750_data, work);
	unsigned long delay = msecs_to_jiffies(atomic_read(&afa750->delay));
	unsigned char data[6] = {0};
	short vec[3] = {0};

	data[0] = 0x10;
	if (afa750_i2c_rx_data(data, 6) < 0) {
		return;
	}
	if (dbglevel == 1) {
		printk("raw data(%x, %x, %x, %x, %x, %x)\n", data[0], data[1], data[2], data[3], data[4], data[5]);
	}
	vec[0] = afa750_charge(data[1], data[0]);		
	vec[1] = afa750_charge(data[3], data[2]);		
	vec[2] = afa750_charge(data[5], data[4]);
	if (dbglevel == 2) {
		printk("1...xyz(%d, %d, %d)\n", vec[0], vec[1], vec[2]);
	}


#if defined(CONFIG_CALI_POSITIVE) || defined(CONFIG_CALI_NEGATIVE)
	vec[0] = AFA750_Cali_X(vec[0]);
	vec[1] = AFA750_Cali_Y(vec[1]);
	vec[2] = AFA750_Cali_Z(vec[2]);

	if (dbglevel == 3) {
		printk("Calibrated xyz(%d, %d, %d)\n", vec[0], vec[1], vec[2]);
	}
#endif
		
	input_report_abs(afa750->input, ABS_X, vec[0]);
	input_report_abs(afa750->input, ABS_Y, vec[1]);
	input_report_abs(afa750->input, ABS_Z, vec[2]);

	input_sync(afa750->input);
	if (dbglevel==1) {
		printk("xyz(%d, %d, %d)\n", vec[0], vec[1], vec[2]);
	}
	schedule_delayed_work(&afa750->work, delay);

}

/*
 *static int afa750_ioctl(struct inode *inode, struct file *file,
 *    unsigned int cmd, unsigned long arg)
 */
static int afa750_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)

{
	void __user *pa = (void __user *)arg;
	unsigned char data[16] = {0};
	int vec[3] = {0};

	int err = 0;

	switch (cmd) {

	case AFA750_IOC_PWRON:

		data[0] = 0x03;
		data[1] = 0x04;
		if (afa750_i2c_tx_data(data, 2) < 0) {
			return -EFAULT;
		}
		data[0] = 0x00;
		data[1] = 0x07;
		if (afa750_i2c_tx_data(data, 2) < 0) {
			return -EFAULT;
		}
		data[0] = 0x02;
		data[1] = 0x03;
		if (afa750_i2c_tx_data(data, 2) < 0) {
			return -EFAULT;
		}
		data[0] = 0x04;
		data[1] = 0x00;
		if (afa750_i2c_tx_data(data, 2) < 0) {
			return -EFAULT;
		}
        /*
		 *data[0] = 0x05;
		 *data[1] = 0x02;
		 *if (afa750_i2c_tx_data(data, 2) < 0) {
		 *    return -EFAULT;
		 *}
         */
        /*
		 *data[0] = 0x07;
		 *data[1] = 0x0a;
		 *if (afa750_i2c_tx_data(data, 2) < 0) {
		 *    return -EFAULT;
		 *}
         */
		data[0] = 0x07;
		data[1] = 0x07;
		if (afa750_i2c_tx_data(data, 2) < 0) {
			return -EFAULT;
		}
		data[0] = 0x08;
		data[1] = 0x00;
		if (afa750_i2c_tx_data(data, 2) < 0) {
			return -EFAULT;
		}
		data[0] = 0x09;
		data[1] = 0x01;
		if (afa750_i2c_tx_data(data, 2) < 0) {
			return -EFAULT;
		}
	#if DEBUG
		printk("AFA750 POWER ON SUCCEED\n");
	#endif
		/* wait PWRON done */
		msleep(AFA750_DELAY_PWRON);

		break;

	case AFA750_IOC_PWRDN:
		data[0] = 0x03;
		data[1] = 0x02;
		if (afa750_i2c_tx_data(data, 2) < 0) {
			return -EFAULT;
		}
		/* wait PWRDN done */
		msleep(AFA750_DELAY_PWRDN);
		break;

	case AFA750_IOC_READXYZ:
		data[0] = 0x10;
		if (afa750_i2c_rx_data(data, 6) < 0) {
			return -EFAULT;
		}
		#if 0
		vec[0] = afa750_charge(data[0], data[1]);		
		vec[1] = afa750_charge(data[2], data[3]);		
		vec[2] = afa750_charge(data[4], data[5]);
		#else
		vec[0] = afa750_charge(data[1], data[0]);		
		vec[1] = afa750_charge(data[3], data[2]);		
		vec[2] = afa750_charge(data[5], data[4]);
		#endif

		vec[0] = move(vec[0], offset[0]);
		vec[1] = move(vec[1], offset[1]);

		vec[0] = chang_small(&vec[0]);
		vec[1] = small(&vec[1]);
	/*#if DEBUG*/
		printk("[X - %04x] [Y - %04x] [Z - %04x]\n", 
			vec[0], vec[1], vec[2]);
	/*#endif*/
	
		if (copy_to_user(pa, vec, sizeof(vec))) {
			return -EFAULT;
		}
		break;
	case AFA750_IOC_READSTATUS:
		data[0] = 0x0f;
		if (afa750_i2c_rx_data(data, 6) < 0) {
			return -EFAULT;
		}
		vec[0] = afa750_charge(data[0], data[1]);		
		vec[1] = afa750_charge(data[2], data[3]);		
		vec[2] = afa750_charge(data[4], data[5]);

	#if DEBUG
		printk("[X - %04x] [Y - %04x] [z - %04x]\n", 
			vec[0], vec[1], vec[2]);
	#endif
		if (copy_to_user(pa, vec, sizeof(vec))) {
			return -EFAULT;
		}

		break;
#if 0			//modify by Rojam 2011-05-11 14:38
	case AFA750_IOC_SETDETECTION:
		data[0] = AFA750_REG_CTRL;
		if (copy_from_user(&(data[1]), pa, sizeof(unsigned char))) {
			return -EFAULT;
		}
		if (afa750_i2c_tx_data(data, 2) < 0) {
			return -EFAULT;
		}
		/* wait SETDETECTION done */
		msleep(AFA750_DELAY_SETDETECTION);
		break;
#endif	//modify by Rojam 2011-05-11
#if defined(CONFIG_CALI_POSITIVE) || defined(CONFIG_CALI_NEGATIVE)
	case AFA750_IOC_GET_CALI:
    {
	        struct AFA750_CALI_INFO sensor_data;
			pa = (void __user*)arg;
			if(pa == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			ConsAP_OneTouchCalibration(this_client, (void *)&sensor_data);

			if(copy_to_user(pa, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
    }
		break;

	case AFA750_IOC_SET_CALI:
	{
            pa = (void __user*)arg;
			if(pa == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			

			if(copy_from_user(&sensor_data2 , pa,  sizeof(sensor_data2)))
			{
			    printk("Frances SET CALI failed!!\n");
				err = -EFAULT;
				break;
			}
    }
		break;
#endif

	default:
		break;
	}

	return 0;
}


static ssize_t afa750_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "AFA750");
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(afa750, S_IRUGO, afa750_show, NULL);

static struct file_operations afa750_fops = {
	.owner		= THIS_MODULE,
	.open		= afa750_open,
	.release	= afa750_release,
	.unlocked_ioctl	= afa750_ioctl,
};

static struct miscdevice afa750_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "afa750",
	.fops = &afa750_fops,
};


static void afa750_poweron(void)
{
    unsigned char data[16] = {0};
    data[0] = 0x03;
    data[1] = 0x04;
    if (afa750_i2c_tx_data(data, 2) < 0) {
        printk("AFA750 POWER ON FAIL\n");
    }
    data[0] = 0x00;
    data[1] = 0x07;
    if (afa750_i2c_tx_data(data, 2) < 0) {
        printk("AFA750 POWER ON FAIL\n");
    }
    data[0] = 0x02;
    data[1] = 0x03;
    if (afa750_i2c_tx_data(data, 2) < 0) {
        printk("AFA750 POWER ON FAIL\n");
    }
    data[0] = 0x04;
    data[1] = 0x00;
    if (afa750_i2c_tx_data(data, 2) < 0) {
        printk("AFA750 POWER ON FAIL\n");
    }
    
    data[0] = 0x07;
    data[1] = 0x07;
    if (afa750_i2c_tx_data(data, 2) < 0) {
        printk("AFA750 POWER ON FAIL\n");
    }
    data[0] = 0x08;
    data[1] = 0x00;
    if (afa750_i2c_tx_data(data, 2) < 0) {
        printk("AFA750 POWER ON FAIL\n");
    }
    data[0] = 0x09;
    data[1] = 0x01;
    if (afa750_i2c_tx_data(data, 2) < 0) {
        printk("AFA750 POWER ON FAIL\n");
    }
    #if DEBUG
    printk("AFA750 POWER ON SUCCEED\n");
    #endif
    /* wait PWRON done */
    msleep(AFA750_DELAY_PWRON);
}

static void afa750_powerdown(void)
{
    unsigned char data[16] = {0};
	
    data[0] = 0x03;
    data[1] = 0x02;
    if (afa750_i2c_tx_data(data, 2) < 0) {
        printk("AFA750 POWER DOWN FAIL\n");
    }
    /* wait PWRDN done */
    msleep(AFA750_DELAY_PWRDN);
}

static ssize_t afa750_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct afa750_data *afa750 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&afa750->delay));

}

static ssize_t afa750_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct afa750_data *afa750 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	atomic_set(&afa750->delay, (unsigned int) data);

	return count;
}

static ssize_t afa750_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct afa750_data *afa750 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&afa750->enable));

}

static void afa750_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct afa750_data *afa750 = i2c_get_clientdata(client);
	int pre_enable = atomic_read(&afa750->enable);

	mutex_lock(&afa750->enable_mutex);
	if (enable) {
		if (pre_enable == 0) {
#if defined(CONFIG_CALI_POSITIVE) || defined(CONFIG_CALI_NEGATIVE)
        struct file *fp; 
        mm_segment_t old_fs = get_fs();
        set_fs(KERNEL_DS);
        
        fp = filp_open("/data/.afa750_cali", O_RDONLY, 0644);
        if (!IS_ERR(fp))
        {
          /* read the file here */
            if(dbglevel==5)
                printk("AFA750 : Calibration file exists\n");
            fp->f_op->read(fp,(char *)&sensor_data2, sizeof(sensor_data2), &fp->f_pos);

            if(dbglevel==5)
            {
                printk("offset (%d, %d, %d), gain (%d, %d, %d)\n", sensor_data2.offset[0], sensor_data2.offset[1], 
                sensor_data2.offset[2], sensor_data2.gain[0], sensor_data2.gain[1], sensor_data2.gain[2]);
            }
            filp_close(fp, 0);
        }
        else
        {
            if(dbglevel==5)
                printk("AFA750 : Calibration file does not exist\n");

            memset(&sensor_data2, 0, sizeof(sensor_data2));
			ConsAP_OneTouchCalibration(this_client, (void *)&sensor_data2);
            fp = filp_open("/data/.afa750_cali", O_WRONLY|O_CREAT, 0644);
            if (!IS_ERR(fp))
            {
                    if(dbglevel==5)
                {
                    printk("offset (%d, %d, %d), gain (%d, %d, %d)\n", sensor_data2.offset[0], sensor_data2.offset[1], 
                    sensor_data2.offset[2], sensor_data2.gain[0], sensor_data2.gain[1], sensor_data2.gain[2]);
                }


                fp->f_op->write(fp, (char *)&sensor_data2, sizeof(sensor_data2), &fp->f_pos);
                filp_close(fp, 0);
            }
            else
            {
                printk("AFA750: Failed to write calibration file /data/.afa750_cali");
            }
        }

        set_fs(old_fs);
#endif
			//afa750_poweron();
			schedule_delayed_work(&afa750->work,
				msecs_to_jiffies(atomic_read(&afa750->delay)));
			atomic_set(&afa750->enable, 1);
		}

	} else {
		if (pre_enable == 1) {
			//afa750_powerdown();
			cancel_delayed_work_sync(&afa750->work);
			atomic_set(&afa750->enable, 0);
		}
	}
	mutex_unlock(&afa750->enable_mutex);

}

static ssize_t afa750_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if ((data == 0) || (data == 1))
		afa750_set_enable(dev, data);

	return count;
}

static ssize_t afa750_debug_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	
	dbglevel = data;

	return count;
}

static ssize_t afa750_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", dbglevel);
}


static ssize_t afa750_read_write_reg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	return count;
}


static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		afa750_delay_show, afa750_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		afa750_enable_show, afa750_enable_store);
static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		afa750_debug_show, afa750_debug_store);
static DEVICE_ATTR(reg, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		NULL, afa750_read_write_reg_store);
		

static struct attribute *afa750_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_debug.attr,
	&dev_attr_reg.attr,
	NULL
};

static struct attribute_group afa750_attribute_group = {
	.attrs = afa750_attributes
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void afa750_early_suspend(struct early_suspend *h)
{
    struct afa750_data *data =
        container_of(h, struct afa750_data, early_suspend);

    mutex_lock(&data->enable_mutex);
	
	//afa750_powerdown();
    cancel_delayed_work_sync(&data->work);
	atomic_set(&data->enable, 0);
	
    mutex_unlock(&data->enable_mutex);
}


static void afa750_late_resume(struct early_suspend *h)
{
    struct afa750_data *data =
    container_of(h, struct afa750_data, early_suspend);

    mutex_lock(&data->enable_mutex);
    
	//afa750_poweron();
    schedule_delayed_work(&data->work,
        msecs_to_jiffies(atomic_read(&data->delay)));
	atomic_set(&data->enable, 1);
	
    mutex_unlock(&data->enable_mutex);
}
#endif


int afa750_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct afa750_data *data;
	struct input_dev *dev;
	unsigned char buf[16] = {0};

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}

	data = kzalloc(sizeof(struct afa750_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	data->afa750_client = client;
	this_client = client;
	
	buf[0] = 0x03;
	buf[1] = 0x04;
	if (afa750_i2c_tx_data(buf, 2) < 0) {
			goto kfree_exit;
	}

	buf[0] = 0x36;
	buf[1] = 0x01;
	if (afa750_i2c_tx_data(buf, 2) < 0) {
			goto kfree_exit;
	}
	
	mdelay(5);
	
	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);

	INIT_DELAYED_WORK(&data->work, afa750_work_func);
	atomic_set(&data->delay, AFA750_MAX_DELAY);
	atomic_set(&data->enable, 0);

	dev = input_allocate_device();
	if (!dev) {
		goto kfree_exit;
	}

	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_REL, LOW_G_INTERRUPT);
	input_set_capability(dev, EV_REL, HIGH_G_INTERRUPT);
	input_set_capability(dev, EV_REL, SLOP_INTERRUPT);
	input_set_capability(dev, EV_REL, DOUBLE_TAP_INTERRUPT);
	input_set_capability(dev, EV_REL, SINGLE_TAP_INTERRUPT);
	input_set_capability(dev, EV_ABS, ORIENT_INTERRUPT);
	input_set_capability(dev, EV_ABS, FLAT_INTERRUPT);
	input_set_abs_params(dev, ABS_X, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN, ABSMAX, 0, 0);

	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		goto free_input_device;
	}

	data->input = dev;
	err = misc_register(&afa750_device);                  //major 10
	if (err) {
		pr_err("%s: afa750_device register failed\n", __FUNCTION__);
		goto deregister_input_device;
	}
	
	err = device_create_file(&client->dev, &dev_attr_afa750);
	if (err) {
		pr_err("%s: device_create_file failed\n", __FUNCTION__);
		goto out_deregister;
	}
	
	err = sysfs_create_group(&data->input->dev.kobj,
			&afa750_attribute_group);
	if (err < 0)
		goto out_deregister;

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = afa750_early_suspend;
	data->early_suspend.resume = afa750_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);

	printk("===afa750 prob ok====\n");
	return 0;

out_deregister:
	misc_deregister(&afa750_device);

deregister_input_device:
	input_unregister_device(data->input);

free_input_device:
    input_free_device(dev);

kfree_exit:
	kfree(data);
exit:
	return err;
}

static int afa750_remove(struct i2c_client *client)
{
	struct afa750_data *data = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
    device_remove_file(&client->dev, &dev_attr_afa750);
	sysfs_remove_group(&data->input->dev.kobj, &afa750_attribute_group);
	input_unregister_device(data->input);
	kfree(data);

	return 0;
}

static const struct i2c_device_id afa750_id[] = {
	{ "afa750", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, afa750_id);
static struct i2c_driver afa750_driver = {
	.probe 		= afa750_probe,
	.remove 	= afa750_remove,
	.id_table	= afa750_id,
	.driver 	= {
		.owner	= THIS_MODULE,
		.name = "afa750",
	},
};

static int __init afa750_init(void)
{
	pr_info("afa750 driver: init\n");
	return i2c_add_driver(&afa750_driver);
}

static void __exit afa750_exit(void)
{
	pr_info("afa750 driver: exit\n");
	i2c_del_driver(&afa750_driver);
}

module_init(afa750_init);
module_exit(afa750_exit);

MODULE_AUTHOR("Robbie Cao<hjcao@memsic.com>");
MODULE_DESCRIPTION("MEMSIC AFA750 (DTOS) Accelerometer Sensor Driver");
MODULE_LICENSE("GPL");

