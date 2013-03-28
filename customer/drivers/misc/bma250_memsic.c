/*  $Date: 2010/09/17 11:40:00 $
 *  $Revision: 0.9 $ 
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2010 Bosch Sensortec GmbH
 * All Rights Reserved
 */



/*! \file BMA250_driver.c
    \brief This file contains all function implementations for the BMA250 in linux
    
    Details.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm/uaccess.h>
#include <linux/unistd.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#ifdef BMA250_HAS_EARLYSUSPEND
#include<linux/earlysuspend.h>
#endif

#include <linux/poll.h>

#include <linux/bma250_driver.h>

#define BMA250_DEBUG

bma250_t * p_bma250;

int bma250_init(bma250_t *bma250)
   {
   int comres= C_Zero_U8X ;
   unsigned char data;
   
   p_bma250 = bma250;                                                                                                                                                      /* assign bma250 ptr */
   p_bma250->dev_addr = BMA250_I2C_ADDR;                                                   /* preset bma250 I2C_addr */
   comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_CHIP_ID__REG, &data, 1);     /* read Chip Id */
   
   p_bma250->chip_id = data ;                                          /* get bitslice */
   
   comres += p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_ML_VERSION__REG, &data, 1); /* read Version reg */
   p_bma250->ml_version = bma250_GET_BITSLICE(data, bma250_ML_VERSION);                            /* get ML Version */
   p_bma250->al_version = bma250_GET_BITSLICE(data, bma250_AL_VERSION);                            /* get AL Version */
   return comres;
   }


unsigned char bma250_set_mode(unsigned char Mode)
   {
   int comres=C_Zero_U8X ;
   unsigned char data1;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      if (Mode < C_Three_U8X)
         {
         comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_LOW_POWER__REG, &data1, C_One_U8X );
         switch (Mode)
            {
            case bma250_MODE_NORMAL:
               data1  = bma250_SET_BITSLICE(data1, bma250_EN_LOW_POWER, C_Zero_U8X);
               data1  = bma250_SET_BITSLICE(data1, bma250_EN_SUSPEND, C_Zero_U8X);
               break;
            case bma250_MODE_LOWPOWER:
               data1  = bma250_SET_BITSLICE(data1, bma250_EN_LOW_POWER, C_One_U8X);
               data1  = bma250_SET_BITSLICE(data1, bma250_EN_SUSPEND, C_Zero_U8X);
               break;
            case bma250_MODE_SUSPEND:
               data1  = bma250_SET_BITSLICE(data1, bma250_EN_LOW_POWER, C_Zero_U8X);
               data1  = bma250_SET_BITSLICE(data1, bma250_EN_SUSPEND, C_One_U8X);
               break;
            default:
               break;
            }
         comres += p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_LOW_POWER__REG, &data1, C_One_U8X);
         p_bma250->mode = Mode;
         }
      else
         {
         comres = E_OUT_OF_RANGE ;
         }
      }
   return comres;
   }

unsigned char bma250_set_range(unsigned char Range)
   {
   int comres=C_Zero_U8X ;
   unsigned char data1;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      if (Range < C_Four_U8X)
         {
         comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_RANGE_SEL_REG, &data1, C_One_U8X );
         switch (Range)
            {
            case C_Zero_U8X:
               data1  = bma250_SET_BITSLICE(data1, bma250_RANGE_SEL, C_Zero_U8X);
               break;
            case C_One_U8X:
               data1  = bma250_SET_BITSLICE(data1, bma250_RANGE_SEL, C_Five_U8X);
               break;
            case C_Two_U8X:
               data1  = bma250_SET_BITSLICE(data1, bma250_RANGE_SEL, C_Eight_U8X);
               break;
            case C_Three_U8X:
               data1  = bma250_SET_BITSLICE(data1, bma250_RANGE_SEL, C_Twelve_U8X);
               break;
            default:
               break;
            }
         comres += p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_RANGE_SEL_REG, &data1, C_One_U8X);
         }
      else
         {
         comres = E_OUT_OF_RANGE ;
         }
      }
   return comres;
   }

int bma250_set_bandwidth(unsigned char BW)
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   int Bandwidth = C_Zero_U8X ;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      if (BW < C_Eight_U8X)
         {
         switch (BW)
            {
            case C_Zero_U8X:
               Bandwidth = bma250_BW_7_81HZ;
               
               /*  7.81 Hz      64000 uS   */
               break;
            case C_One_U8X:
               Bandwidth = bma250_BW_15_63HZ;
               
               /*  15.63 Hz     32000 uS   */
               break;
            case C_Two_U8X:
               Bandwidth = bma250_BW_31_25HZ;
               
               /*  31.25 Hz     16000 uS   */
               break;
            case C_Three_U8X:
               Bandwidth = bma250_BW_62_50HZ;
               
               /*  62.50 Hz     8000 uS   */
               break;
            case C_Four_U8X:
               Bandwidth = bma250_BW_125HZ;
               
               /*  125 Hz       4000 uS   */
               break;
            case C_Five_U8X:
               Bandwidth = bma250_BW_250HZ;
               
               /*  250 Hz       2000 uS   */
               break;
            case C_Six_U8X:
               Bandwidth = bma250_BW_500HZ;
               
               /*  500 Hz       1000 uS   */
               break;
            case C_Seven_U8X:
               Bandwidth = bma250_BW_1000HZ;
               
               /*  1000 Hz      500 uS   */
               break;
            default:
               break;
            }
         comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_BANDWIDTH__REG, &data, C_One_U8X );
         data = bma250_SET_BITSLICE(data, bma250_BANDWIDTH, Bandwidth );
         comres += p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_BANDWIDTH__REG, &data, C_One_U8X );
         }
      else
         {
         comres = E_OUT_OF_RANGE ;
         }
      }
   return comres;
   }

int bma250_read_accel_xyz(bma250acc_t * acc)
   {
   int comres;
   unsigned char data[6];
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_ACC_X_LSB__REG, data,6);
      
      acc->x = bma250_GET_BITSLICE(data[0],bma250_ACC_X_LSB) | (bma250_GET_BITSLICE(data[1],bma250_ACC_X_MSB)<<bma250_ACC_X_LSB__LEN);

      acc->x = acc->x << (sizeof(short)*8-(bma250_ACC_X_LSB__LEN + bma250_ACC_X_MSB__LEN));
      acc->x = acc->x >> (sizeof(short)*8-(bma250_ACC_X_LSB__LEN + bma250_ACC_X_MSB__LEN));
      
      acc->y = bma250_GET_BITSLICE(data[2],bma250_ACC_Y_LSB) | (bma250_GET_BITSLICE(data[3],bma250_ACC_Y_MSB)<<bma250_ACC_Y_LSB__LEN);
      acc->y = acc->y << (sizeof(short)*8-(bma250_ACC_Y_LSB__LEN + bma250_ACC_Y_MSB__LEN));
      acc->y = acc->y >> (sizeof(short)*8-(bma250_ACC_Y_LSB__LEN + bma250_ACC_Y_MSB__LEN));
 
      acc->z = bma250_GET_BITSLICE(data[4],bma250_ACC_Z_LSB) | (bma250_GET_BITSLICE(data[5],bma250_ACC_Z_MSB)<<bma250_ACC_Z_LSB__LEN);
      acc->z = acc->z << (sizeof(short)*8-(bma250_ACC_Z_LSB__LEN + bma250_ACC_Z_MSB__LEN));
      acc->z = acc->z >> (sizeof(short)*8-(bma250_ACC_Z_LSB__LEN + bma250_ACC_Z_MSB__LEN));
      }
      //printk("x=%d,y=%d,z=%d\n",acc->x,acc->y,acc->z);
   return comres;
   }




int bma250_get_data_interrupt(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS2_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_DATA_INT_S);
      *intstatus = data;
      }
   return comres;
   }


#ifdef BMA250_HAS_EARLYSUSPEND
static void bma250_early_suspend(struct early_suspend *h);
static void bma250_late_resume(struct early_suspend *h);
#endif


/* i2c operation for bma250 API */
static char bma250_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len);
static char bma250_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len);
static void bma250_i2c_delay(unsigned int msec);

/* globe variant */
static struct i2c_client *bma250_client = NULL;
struct bma250_data {
	bma250_t			bma250;
	int IRQ;
	struct fasync_struct *async_queue;
#ifdef BMA250_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};


#ifdef BMA250_ENABLE_IRQ
static int bma250_interrupt_config(void);
					


static int bma250_interrupt_config()
{
#ifdef BMA250_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

	return 0;
} 


static irqreturn_t bma250_irq_handler(int irq, void *_id)
{
	struct bma250_data *data;	
    unsigned long flags;
	if(((bma250_t*)_id)->chip_id != 0x03)
	{
#ifdef BMA250_DEBUG
		printk(KERN_INFO "%s error\n",__FUNCTION__);
#endif
		return IRQ_HANDLED;
	}
	if(bma250_client == NULL)
		return IRQ_HANDLED;
    printk("bma250 irq handler\n");
   
    data = i2c_get_clientdata(bma250_client);
    if(data == NULL)
		return IRQ_HANDLED;
 	local_irq_save(flags);
    if(data->async_queue)
				kill_fasync(&data->async_queue,SIGIO, POLL_IN);
	local_irq_restore(flags);
	return IRQ_HANDLED;
}

#endif

/*	i2c delay routine for eeprom	*/
static inline void bma250_i2c_delay(unsigned int msec)
{
	mdelay(msec);
}

/*	i2c write routine for bma250	*/
static inline char bma250_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
#ifndef BMA250_SMBUS
	unsigned char buffer[2];
#endif
	if( bma250_client == NULL )	/*	No global client pointer?	*/
		return -1;

	while(len--)
	{
#ifdef BMA250_SMBUS
		dummy = i2c_smbus_write_byte_data(bma250_client, reg_addr, *data);
#else
		buffer[0] = reg_addr;
		buffer[1] = *data;
		dummy = i2c_master_send(bma250_client, (char*)buffer, 2);
#endif
		reg_addr++;
		data++;
		if(dummy < 0)
			return -1;
	}
	return 0;
}

/*	i2c read routine for bma250	*/
static inline char bma250_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len) 
{
	s32 dummy;
	if( bma250_client == NULL )	/*	No global client pointer?	*/
		return -1;

	while(len--)
	{        
#ifdef BMA250_SMBUS
		dummy = i2c_smbus_read_byte_data(bma250_client, reg_addr);
		if(dummy < 0)
			return -1;
		*data = dummy & 0x000000ff;
#else
		dummy = i2c_master_send(bma250_client, (char*)&reg_addr, 1);
		if(dummy < 0)
			return -1;
		dummy = i2c_master_recv(bma250_client, (char*)data, 1);
		if(dummy < 0)
			return -1;
#endif
		reg_addr++;
		data++;
	}
	return 0;
}


/*	open command for BMA250 device file	*/
static int bma250_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

/*	release command for BMA250 device file	*/
static int bma250_close(struct inode *inode, struct file *file)
{
#ifdef BMA250_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);	
#endif
	return 0;
}


/*	ioctl command for BMA250 device file	*/
static long bma250_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    void __user *pa = (void __user *)arg;
	int err = 0;
    bma250acc_t acc;
	int vec[3] = {0};

#ifdef BMA250_DEBUG
	//printk(KERN_INFO "%s,cmd = %d , BMA250_IOC_READXYZ = %d\n",__FUNCTION__,cmd,BMA250_IOC_READXYZ);	
#endif

	/* cmd mapping */

	switch(cmd)
	{
    case BMA250_IOC_READXYZ:
		bma250_read_accel_xyz(&acc);

	
		vec[0] = acc.x;		
		vec[1] = acc.y;		
		vec[2] = acc.z;		
//		printk("[X - %d] [Y - %d] [Z - %d]\n", 
//			vec[0], vec[1], vec[2]);			
//		printk("[acc.x - %d] [acc.y - %d] [acc.z - %d]\n", 
//			acc.x, acc.y, acc.z);					
        if (copy_to_user(pa, vec, sizeof(vec))) {
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to error\n");
#endif
			return -EFAULT;
		}
		break;
    case BMA250_IOC_READSTATUS:
		bma250_read_accel_xyz(&acc);
		
		vec[0] = acc.x;		
		vec[1] = acc.y;		
		vec[2] = acc.z;		

        if (copy_to_user(pa, vec, sizeof(vec))) {
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to error\n");
#endif
			return -EFAULT;
		}
		break;
    case BMA250_IOC_PWRDN:

		bma250_set_mode(bma250_MODE_SUSPEND);
        break;

    case BMA250_IOC_PWRON:
        
        bma250_set_mode(bma250_MODE_NORMAL);
		break;

	default:
		return 0;
	}
		return 0;	
}

static int bma250_fasync(int fd, struct file *file, int mode)
{
    struct bma250_data* data;
#ifdef BMA250_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);	
#endif
 	data=i2c_get_clientdata(bma250_client); 
	return fasync_helper(fd,file,mode,&data->async_queue);
	return 0;
}

static const struct file_operations bma250_fops = {
	.owner = THIS_MODULE,
//	.read = bma250_read,
//	.write = bma250_write,
//    .poll = bma250_poll,
	.open = bma250_open,
	.release = bma250_close,
	.unlocked_ioctl = bma250_ioctl,
//	.fasync = bma250_fasync,
};


static struct miscdevice bma_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma250",
	.fops = &bma250_fops,
};

static int bma250_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
#ifdef BMA250_DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	strlcpy(info->type, "bma250", I2C_NAME_SIZE);

	return 0;
}

static int bma250_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0;
	int tempvalue;
	struct bma250_data *data;
#ifdef BMA250_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}
	data = kzalloc(sizeof(struct bma250_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}		
	/* read chip id */
	tempvalue = 0;
#ifdef BMA250_SMBUS
	tempvalue = i2c_smbus_read_word_data(client, 0x00);
#else
	i2c_master_send(client, (char*)&tempvalue, 1);
	i2c_master_recv(client, (char*)&tempvalue, 1);
#endif



	if((tempvalue&0x00FF) == 0x0003)
	{
		printk(KERN_INFO "Bosch Sensortec Device detected!\n BMA250 registered I2C driver!\n");
		bma250_client = client;
	}
	else
	{
		printk(KERN_INFO "Bosch Sensortec Device not found, i2c error %d \n", tempvalue);
				
		bma250_client = NULL;
		err = -1;
		goto kfree_exit;
	}
	i2c_set_clientdata(bma250_client, data);
	
	err = misc_register(&bma_device);
	if (err) {
		printk(KERN_ERR "bma250 device register failed\n");
		goto kfree_exit;
	}
	printk(KERN_INFO "bma250 device create ok\n");

	/* bma250 sensor initial */
	data->bma250.bus_write = bma250_i2c_write;
	data->bma250.bus_read = bma250_i2c_read;
	data->bma250.delay_msec = bma250_i2c_delay;
	bma250_init(&data->bma250);

	bma250_set_bandwidth(4);		//bandwidth 125Hz
	bma250_set_range(0);			//range +/-2G

  

	/* register interrupt */
#ifdef	BMA250_ENABLE_IRQ

	err = bma250_interrupt_config();
	if (err < 0)
		goto exit_dereg;
    data->IRQ = client->irq;
	err = request_irq(data->IRQ, bma250_irq_handler, IRQF_TRIGGER_RISING, "bma250", &data->bma250);
	if (err)
	{
		printk(KERN_ERR "could not request irq\n");
		goto exit_dereg;
	}
	
#endif


#ifdef BMA250_HAS_EARLYSUSPEND
    data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    data->early_suspend.suspend = bma250_early_suspend;
    data->early_suspend.resume = bma250_late_resume;
    register_early_suspend(&data->early_suspend);
#endif



	return 0;

#ifdef BMA250_ENABLE_IRQ


exit_dereg:
    misc_deregister(&bma_device);
#endif
kfree_exit:
	kfree(data);
exit:
	return err;
}

#ifdef BMA250_HAS_EARLYSUSPEND
static void bma250_early_suspend(struct early_suspend *h)
{
#ifdef BMA250_DEBUG
    printk(KERN_INFO "%s\n",__FUNCTION__);
#endif	
    bma250_set_mode(bma250_MODE_SUSPEND);
}


static void bma250_late_resume(struct early_suspend *h)
{
#ifdef BMA250_DEBUG
    printk(KERN_INFO "%s\n",__FUNCTION__);
#endif		
    bma250_set_mode(bma250_MODE_NORMAL);
}
#endif
static int bma250_suspend(struct i2c_client *client,pm_message_t mesg)
{
#ifdef BMA250_DEBUG
    printk(KERN_INFO "%s\n",__FUNCTION__);
#endif	
    bma250_set_mode(bma250_MODE_SUSPEND);
    return 0;	
}
static int bma250_resume(struct i2c_client *client)
{
#ifdef BMA250_DEBUG
    printk(KERN_INFO "%s\n",__FUNCTION__);
#endif	
    bma250_set_mode(bma250_MODE_NORMAL);
    return 0;	
}

static int bma250_remove(struct i2c_client *client)
{
	struct bma250_data *data = i2c_get_clientdata(client);
#ifdef BMA250_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif	

#ifdef BMA250_HAS_EARLYSUSPEND
    unregister_early_suspend(&data->early_suspend);
#endif
	misc_deregister(&bma_device);
#ifdef BMA250_ENABLE_IRQ
	free_irq(data->IRQ, &data->bma250);
#endif
	
	kfree(data);
	bma250_client = NULL;
	return 0;
}


static unsigned short normal_i2c[] = { I2C_CLIENT_END};
I2C_CLIENT_INSMOD_1(bma250);

static const struct i2c_device_id bma250_id[] = {
	{ "bma250", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma250_id);

static struct i2c_driver bma250_driver = {	
	.driver = {
		.owner	= THIS_MODULE,	
		.name	= "bma250",
	},
	.class		= I2C_CLASS_HWMON,
	.id_table	= bma250_id,
//	.address_data	= &addr_data,
	.probe		= bma250_probe,
	.remove		= bma250_remove,
	.detect		= bma250_detect,
    .suspend    = bma250_suspend,
    .resume     = bma250_resume,
};

static int __init BMA250_init(void)
{
#ifdef BMA250_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif
	return i2c_add_driver(&bma250_driver);
}

static void __exit BMA250_exit(void)
{
	i2c_del_driver(&bma250_driver);
	printk(KERN_ERR "BMA250 exit\n");
}



MODULE_DESCRIPTION("BMA250 driver");


module_init(BMA250_init);
module_exit(BMA250_exit);

