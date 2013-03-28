/*
 *  mma7660.c - Linux kernel modules for 3-Axis Orientation/Motion
 *  Detection Sensor 
 *
 *  Copyright (C) 2009-2010 Freescale Semiconductor Ltd.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/input-polldev.h>

static struct mutex sensor_lock;

/*
 * Defines
 */

#define DEBUG	0

#if DEBUG
#define assert(expr)\
        if(!(expr)) {\
        printk( "Assertion failed! %s,%d,%s,%s\n",\
        __FILE__,__LINE__,__func__,#expr);\
        }
#else
#define assert(expr) do{} while(0)
#endif

#define MMA7660_DRV_NAME	"mma7660"
#define MMA7660_XOUT			0x00
#define MMA7660_YOUT			0x01
#define MMA7660_ZOUT			0x02
#define MMA7660_TILT			0x03
#define MMA7660_SRST			0x04
#define MMA7660_SPCNT			0x05
#define MMA7660_INTSU			0x06
#define MMA7660_MODE			0x07
#define MMA7660_SR				0x08
#define MMA7660_PDET			0x09
#define MMA7660_PD				0x0A

#define MK_MMA7660_SR(FILT, AWSR, AMSR)\
	(FILT<<5 | AWSR<<3 | AMSR)

#define MK_MMA7660_MODE(IAH, IPP, SCPS, ASE, AWE, TON, MODE)\
	(IAH<<7 | IPP<<6 | SCPS<<5 | ASE<<4 | AWE<<3 | TON<<2 | MODE)

#define MK_MMA7660_INTSU(SHINTX, SHINTY, SHINTZ, GINT, ASINT, PDINT, PLINT, FBINT)\
	(SHINTX<<7 | SHINTY<<6 | SHINTZ<<5 | GINT<<4 | ASINT<<3 | PDINT<<2 | PLINT<<1 | FBINT)

#define MODE_CHANGE_DELAY_MS 100

#define MAX_POLL_INTERVAL		200	
#define DEFAULT_POLL_INTERVAL		50

#ifdef CONFIG_PM
static int mma7660_suspend(struct device *dev);
static int mma7660_resume(struct device *dev);
#endif

ssize_t	show_orientation(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t	show_axis_force(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t	show_xyz_force(struct device *dev, struct device_attribute *attr, char *buf);


static struct device *hwmon_dev;
static struct i2c_client *mma7660_i2c_client;
static struct input_polled_dev *mma7660_idev;
static u32 is_enabled;
static u32 poll_interval;		//ms
static u8 orientation;
static int dbg_level = 0;

static SENSOR_DEVICE_ATTR(all_axis_force, S_IRUGO, show_xyz_force, NULL, 0);
static SENSOR_DEVICE_ATTR(x_axis_force, S_IRUGO, show_axis_force, NULL, 0);
static SENSOR_DEVICE_ATTR(y_axis_force, S_IRUGO, show_axis_force, NULL, 1);
static SENSOR_DEVICE_ATTR(z_axis_force, S_IRUGO, show_axis_force, NULL, 2);
static SENSOR_DEVICE_ATTR(orientation, S_IRUGO, show_orientation, NULL, 0);

static struct attribute* mma7660_attrs[] = 
{
	&sensor_dev_attr_all_axis_force.dev_attr.attr,
	&sensor_dev_attr_x_axis_force.dev_attr.attr,
	&sensor_dev_attr_y_axis_force.dev_attr.attr,
	&sensor_dev_attr_z_axis_force.dev_attr.attr,
	&sensor_dev_attr_orientation.dev_attr.attr,
	NULL
};



static const struct attribute_group mma7660_group =
{
	.attrs = mma7660_attrs,
};

static void mma7660_read_xyz(int idx, s8 *pf)
{
	s32 result;
	int count=0;
	assert(mma7660_i2c_client);
	do
	{
		result=i2c_smbus_read_byte_data(mma7660_i2c_client, idx+MMA7660_XOUT);
		assert(result>=0);
		count++;
		if(count>5)
			return;
	}while(result&(1<<6)); //read again if alert
	*pf = (result&(1<<5)) ? (result|(~0x0000003f)) : (result&0x0000003f);
}

static void mma7660_read_tilt(u8* pt)
{
	u32 result;
	int count=0;
	assert(mma7660_i2c_client);
	do
	{	
		result = i2c_smbus_read_byte_data(mma7660_i2c_client, MMA7660_TILT);
		assert(result>0);
		count++;
		if(count>5)
			return;
	}while(result&(1<<6)); //read again if alert
	*pt = result & 0x000000ff;
}

ssize_t	show_orientation(struct device *dev, struct device_attribute *attr, char *buf)
{
	int result;

	switch((orientation>>2)&0x07)
	{
	case 1: 
		result = sprintf(buf, "Left\n");
		break;

	case 2:
		result = sprintf(buf, "Right\n");
		break;
	
	case 5:
		result = sprintf(buf, "Downward\n");
		break;

	case 6:
		result = sprintf(buf, "Upward\n");
		break;

	default:
		switch(orientation & 0x03)
		{
		case 1:
			result = sprintf(buf, "Front\n");
			break;

		case 2:
			result = sprintf(buf, "Back\n");
			break;

		default:
			result = sprintf(buf, "Unknown\n");
		}
	}
	return result;
}

ssize_t show_xyz_force(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i;
	s8 xyz[3]; 

	for(i=0; i<3; i++)
		mma7660_read_xyz(i, &xyz[i]);
	return sprintf(buf, "(%d,%d,%d)\n", xyz[0], xyz[1], xyz[2]);	
}

ssize_t	show_axis_force(struct device *dev, struct device_attribute *attr, char *buf)
{
	s8 force;
    	int n = ((struct sensor_device_attribute *)to_sensor_dev_attr(attr))->index;

	mma7660_read_xyz(n, &force);
	return sprintf(buf, "%d\n", force);	
}

ssize_t mma7660_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", dbg_level);
}

static ssize_t mma7660_debug_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	dbg_level = data;
	return count;
}

ssize_t mma7660_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", poll_interval);
}

static ssize_t mma7660_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long delay;
	int error;

	error = strict_strtoul(buf, 10, &delay);
	if (error)
		return error;
	mma7660_idev->poll_interval = poll_interval = (delay > MAX_POLL_INTERVAL) ? MAX_POLL_INTERVAL: delay;
	return count;
}

static ssize_t mma7660_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	mma7660_idev->poll_interval = poll_interval;
    return sprintf(buf, "%d\n", is_enabled);
}

static ssize_t mma7660_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;


	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (data == 1)
			mma7660_resume(dev);
	else if(data == 0)
			mma7660_suspend(dev);
	return count;
}

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		mma7660_delay_show, mma7660_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		mma7660_enable_show, mma7660_enable_store);
static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR|S_IWGRP,
		mma7660_debug_show, mma7660_debug_store);


static struct attribute *mma7660_attributes[] = {
    &dev_attr_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_debug.attr,
    NULL
};

static struct attribute_group mma7660_attribute_group = {
    .attrs = mma7660_attributes,
};


static void mma7660_worker(struct work_struct *work)
{
	u8 tilt = 0, new_orientation;

	mma7660_read_tilt(&tilt);
	new_orientation = tilt & 0x1f;
	if(orientation!=new_orientation)
		orientation = new_orientation;
}

DECLARE_WORK(mma7660_work, mma7660_worker);

// interrupt handler
static irqreturn_t mmx7660_irq_handler(int irq, void *dev_id)
{
	schedule_work(&mma7660_work);
	return IRQ_RETVAL(1);
}


/*
 * Initialization function
 */

static int mma7660_init_client(struct i2c_client *client)
{
	int result;

	mma7660_i2c_client = client;
	//plat_data = (struct mxc_mma7660_platform_data *)client->dev.platform_data;
	//assert(plat_data);



	// Enable Orientation Detection Logic
	result = i2c_smbus_write_byte_data(client, 
		MMA7660_MODE, MK_MMA7660_MODE(0, 0, 0, 0, 0, 0, 0)); //enter standby
    if(result < 0)
        return result;
	result = i2c_smbus_write_byte_data(client, 
		MMA7660_SR, MK_MMA7660_SR(7, 2, 2)); 

    if(result < 0)
        return result;
	//result = i2c_smbus_write_byte_data(client, 
	//	MMA7660_INTSU, MK_MMA7660_INTSU(0, 0, 0, 0, 1, 0, 1, 1)); 
	
	result = i2c_smbus_write_byte_data(client, 
	MMA7660_INTSU, MK_MMA7660_INTSU(1, 1, 1, 1, 1, 1, 1, 1)); 	
	
    if(result < 0)
        return result;

	result = i2c_smbus_write_byte_data(client, 
		MMA7660_SPCNT, 0xA0); 

    if(result < 0)
        return result;
	result = i2c_smbus_write_byte_data(client, 
		MMA7660_MODE, MK_MMA7660_MODE(0, 1, 0, 0, 0, 0, 1)); 

    if(result < 0)
        return result;

#if 0
	result = request_irq(client->irq, mmx7660_irq_handler,
		IRQF_TRIGGER_FALLING , MMA7660_DRV_NAME, NULL);

    if(result < 0)
        return result;
#endif
	mdelay(MODE_CHANGE_DELAY_MS);

	{
		u8 tilt=0;
		mma7660_read_tilt(&tilt);
		orientation = tilt&0x1f;
	}
	return result;
}

#define MAX_ABS_X	22
#define MAX_ABS_Y	22
#define MAX_ABS_Z	22
#define MAX_ABS_THROTTLE	128

#define INPUT_FUZZ	0//if "0" be better?
#define INPUT_FLAT	0


static void report_abs(void)
{
	int i;
	s8 xyz[3]; 
	s16 x, y, z;

	if(!is_enabled)
		return;

	mutex_lock(&sensor_lock);

	if(!is_enabled)
	{
		mutex_unlock(&sensor_lock);
		return;
	}

	for(i=0; i<3; i++)
		mma7660_read_xyz(i, &xyz[i]);
		
	mutex_unlock(&sensor_lock);
	/* convert signed 10bits to signed 16bits */
	#if defined(CONFIG_MACH_MESON3_REFF08_v3) || defined(CONFIG_MACH_MESON3_REFF08_EM79v2)
	x =-((short)(xyz[1] << 6) >> 6);
	y =-((short)(xyz[0] << 6) >> 6);
	z =-((short)(xyz[2] << 6) >> 6);
	#elif defined(CONFIG_MACH_MESON6_G33) || defined(CONFIG_MACH_MESON6_G24_86V2)
        x = (short)(xyz[0] << 6) >> 6;
        y = (short)(xyz[1] << 6) >> 6;
        z = (short)(xyz[2] << 6) >> 6;
	#else
	x = (short)(xyz[1] << 6) >> 6;
	y = -(short)(xyz[0] << 6) >> 6;
	z = (short)(xyz[2] << 6) >> 6;
	#endif


	input_report_abs(mma7660_idev->input, ABS_X, x);
	input_report_abs(mma7660_idev->input, ABS_Y, y);
	input_report_abs(mma7660_idev->input, ABS_Z, z);
	input_sync(mma7660_idev->input);
	
	if(dbg_level)
		printk(KERN_INFO"mma7660 sensor data (%d, %d, %d)\n", x, y, z);

}


static void mma7660_dev_poll(struct input_polled_dev *dev)
{
	report_abs();
} 
/////////////////////////end//////

/*
 * I2C init/probing/exit functions
 */

static int __devinit mma7660_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	struct input_dev *idev;

#if DEBUG
	printk("probing mma7660 \n");
#endif
	mutex_init(&sensor_lock);	
	result = i2c_check_functionality(adapter, 
		I2C_FUNC_SMBUS_BYTE|I2C_FUNC_SMBUS_BYTE_DATA);
	assert(result);

	/* Initialize the MC32X0 chip */
	result = mma7660_init_client(client);
    if(result < 0)
        return -1;

	result = sysfs_create_group(&client->dev.kobj, &mma7660_group);
	assert(result==0);

	hwmon_dev = hwmon_device_register(&client->dev);
	assert(!(IS_ERR(hwmon_dev)));

	dev_info(&client->dev, "build time %s %s\n", __DATE__, __TIME__);
  

	/*input poll device register */
	mma7660_idev = input_allocate_polled_device();
	if (!mma7660_idev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		result = -ENOMEM;
		return result;
	}

	mma7660_idev->poll = mma7660_dev_poll;
	mma7660_idev->poll_interval = poll_interval = DEFAULT_POLL_INTERVAL;
	idev = mma7660_idev->input;
	idev->name = MMA7660_DRV_NAME;
	idev->id.bustype = BUS_I2C;
	idev->evbit[0] = BIT_MASK(EV_ABS);

	//change the param by simon.wang,2012-04-09
	//to enhance the sensititity
	input_set_abs_params(idev, ABS_X, -MAX_ABS_X, MAX_ABS_X, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Y, -MAX_ABS_Y, MAX_ABS_Y, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Z, -MAX_ABS_Z, MAX_ABS_Z, INPUT_FUZZ, INPUT_FLAT);
	//input_set_abs_params(idev, ABS_THROTTLE, -MAX_ABS_THROTTLE, MAX_ABS_THROTTLE, INPUT_FUZZ, INPUT_FLAT);//if necessary?
#if DEBUG
	printk("***** Sensor MMA7660 param: max_abs_x = %d,max_abs_y = %d,max_abs_z = %d \
		INPUT_FUZZ = %d,INPUT_FLAT = %d\n",MAX_ABS_X,MAX_ABS_Y,MAX_ABS_Y,INPUT_FUZZ,INPUT_FLAT);
#endif
	result = input_register_polled_device(mma7660_idev);


	result = sysfs_create_group(&mma7660_idev->input->dev.kobj, &mma7660_attribute_group);
	assert(result==0);

	if (result) {
		dev_err(&client->dev, "register poll device failed!\n");
		return result;
	}

//////////////////////////////

	
	return result;
}

static int __devexit mma7660_remove(struct i2c_client *client)
{
	int result;

	mutex_lock(&sensor_lock);
	result = i2c_smbus_write_byte_data(client,MMA7660_MODE, MK_MMA7660_MODE(0, 0, 0, 0, 0, 0, 0));
	assert(result==0);

	mutex_unlock(&sensor_lock);
	//free_irq(plat_data->irq, NULL);
	free_irq(client->irq, NULL);
	sysfs_remove_group(&client->dev.kobj, &mma7660_group);
	hwmon_device_unregister(hwmon_dev);

	return result;
}

#ifdef CONFIG_PM
static int mma7660_suspend(struct device *dev)
{
	int result;
	
	if(!is_enabled)
		return 0;
	mutex_lock(&sensor_lock);
	result = i2c_smbus_write_byte_data(mma7660_i2c_client, 
		MMA7660_MODE, MK_MMA7660_MODE(0, 0, 0, 0, 0, 0, 0));
	assert(result==0);
	is_enabled = 0;
	mutex_unlock(&sensor_lock);
	return result;
}

static int mma7660_resume(struct device *dev)
{
	int result;
	if(is_enabled)
		return 0;
	mutex_lock(&sensor_lock);
	result = i2c_smbus_write_byte_data(mma7660_i2c_client, 
		MMA7660_MODE, MK_MMA7660_MODE(0, 1, 0, 0, 0, 0, 1));
	assert(result==0);
	is_enabled = 1;
	mutex_unlock(&sensor_lock);
	return result;
}
#else

#define mma7660_suspend		NULL
#define mma7660_resume		NULL

#endif


static const struct dev_pm_ops mma7660_dev_pm_ops = {
	.suspend = mma7660_suspend,
	.resume  = mma7660_resume,
};


static const struct i2c_device_id mma7660_id[] = {
	{ MMA7660_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mma7660_id);

static struct i2c_driver mma7660_driver = {
	.driver = {
		.name	= MMA7660_DRV_NAME,
		.owner	= THIS_MODULE,
	#ifdef CONFIG_PM   //add by jf.s, for  sensor resume can not use
		.pm   = &mma7660_dev_pm_ops,
	#endif
	},
	.probe	= mma7660_probe,
	.remove	= __devexit_p(mma7660_remove),
	.id_table = mma7660_id,
};

static int __init mma7660_init(void)
{
	return i2c_add_driver(&mma7660_driver);
}

static void __exit mma7660_exit(void)
{
	i2c_del_driver(&mma7660_driver);
}

MODULE_DESCRIPTION("MMA7660 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");
module_init(mma7660_init);
module_exit(mma7660_exit);
