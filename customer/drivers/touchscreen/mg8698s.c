/* Morgan capacivite multi-touch device driver.
 *
 * Copyright(c) 2010 MorganTouch Inc.
 *
 *************************************
 ***	 Driver For Morgan Touch 
 ***	 Digitizer or Dual Mode Devices
 ***	 Data	 :	 2011,12
 ***	 Author  :	 Randy Pan
 **************************************/


//#define DEBUG
#include <linux/device.h>
#include <linux/kernel.h>
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
#include <mach/gpio_data.h>

#include "mg8698s.h"
#include <linux/miscdevice.h>
// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/uaccess.h>
//for test thread
#include <linux/kthread.h>
//by Jay
#define IOCTL_ENDING    		0xD0
#define IOCTL_I2C_SLAVE			0xD1
#define IOCTL_READ_ID		  	0xD2
#define IOCTL_READ_VERSION  	0xD3
#define IOCTL_RESET  			0xD4
#define IOCTL_IAP_MODE			0xD5
#define IOCTL_CALIBRATION		0xD6
#define IOCTL_ACTION2			0xD7
#define IOCTL_DEFAULT			0x88
static int command_flag= 0;

#define MG_DRIVER_NAME 	"mg8698s"
#define BUF_SIZE 		30

// --------------------------------------------------------------
struct mg_data{
	u16 x, y, w, p, id;
	struct i2c_client *client;
	/* capacivite device*/
	struct input_dev *dev;
	/* digitizer */
	struct timer_list timer;
	struct input_dev *dig_dev;
	struct mutex lock;
	//+++++add for touch led issue
	struct task_struct *mg_touch_thread;
	//+++++by Jay 2011/09/17
	int irq;
	struct work_struct work;
	struct early_suspend early_suspend;
	struct workqueue_struct *mg_wq;
	int (*power)(int on);
	int intr_gpio;
	int fw_ver;
    struct miscdevice firmware;
	#define IAP_MODE_ENABLE		1	/* TS is in IAP mode already */
	int iap_mode;		/* Firmware update mode or 0 for normal */
	
	struct ctp_platform_data *pdata;
};


static inline void mg8698s_reset(struct mg_data *ts, s32 ms)
{
    if (ts->pdata->gpio_reset) {
        gpio_set_status(ts->pdata->gpio_reset, gpio_status_out);
        gpio_out(ts->pdata->gpio_reset, 0);
        msleep(ms);
        gpio_out(ts->pdata->gpio_reset, 1);
        msleep(50);
        printk("mg8698s reset\n");
    }
}

static inline void mg8698s_power(struct mg_data *ts, s32 on)
{
    if (ts->pdata->gpio_power) {
        gpio_set_status(ts->pdata->gpio_power, gpio_status_out);
        gpio_out(ts->pdata->gpio_power, !!on);
        printk("mg8698s power %s\n", on ? "on":"off");
    }
}
// --------------------------------------------------------------

//+++++++modify to early suspend
//#define CONFIG_HAS_EARLYSUSPEND
//+++++++by Jay 2011/08/03

#define BABBAGE_TOUCH_RESET		(3*32 + 4) /* GPIO4_4*/
//+++++++open debug message
//#define RANDY_DEBUG
//+++++++by Jay 2011/08/03
//static int home_flag = 0;
//static int menu_flag = 0;
//static int back_flag = 0;
/************ randy  add for return 816***********/
//u8 ver_buf[5]={0};
/************ randy  add for return 816***********/

static int ver_flag = 0;
static int  id_flag = 0;
static int CS_flag = 0;
#define COORD_INTERPRET(MSB_BYTE, LSB_BYTE) \
		(MSB_BYTE << 8 | LSB_BYTE)

/************ randy  add for return 816***********/
#define CAP_BUF_SIZE 				30
static int ack_flag = 0;
static u8 read_buf[CAP_BUF_SIZE]={0};
static u8 ver_buf[COMMAND_BIT]={0};
/************ randy  add for return 816***********/

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mg_early_suspend(struct early_suspend *h);
static void mg_late_resume(struct early_suspend *h);
#endif
static struct mg_data *private_ts;
static struct i2c_client *touch_i2c_client;
//++++add for touch led issue
struct completion mg_touch_event;
EXPORT_SYMBOL(mg_touch_event);
//++++by Jay 2011/09/17
unsigned int kbc_i2c_read_reg(unsigned int reg);
unsigned int kbc_i2c_write_reg(unsigned int reg,unsigned int value);

unsigned int kbc_i2c_read_reg(unsigned int reg) 
{
	return i2c_smbus_read_byte_data(touch_i2c_client, reg);
}

unsigned int kbc_i2c_write_reg(unsigned int reg,unsigned int value) 
{
	return i2c_smbus_write_byte_data(touch_i2c_client, reg, value);
}

/*****************  ac dc *******************/
static u_int8_t touch_list[3][5] =
{
	{ 0x81, 0x02, 0x00, 0x00, 0x00},
	{ 0x81, 0x02, 0x01, 0x00, 0x00},
	{ 0x97, 0x00, 0x00, 0x00, 0x00},
};
static int touch_ac_dc=1;
static int ac_sw = 1;
static int dc_sw = 0;
/*****************  ac dc *******************/

static u_int8_t nomal_mode[5]=
{0x97, 0x00, 0x00, 0x00, 0x00};

//++++++++add for touch calibration
static u_int8_t cal_ack[5]=
{0x00,0x00,0x00,0x00,0x00};
int calibration_flag=0;
int IsCalibration=0;


int mg_iap_open(struct inode *inode, struct file *filp)
{ 
	//int i;
	struct mg_data *dev;

	#ifdef RANDY_DEBUG	
	printk("[Driver]into mg_iap_open\n");
	#endif
	dev = kmalloc(sizeof(struct mg_data), GFP_KERNEL);
	if (dev == NULL) {
		return -ENOMEM;
	}

	filp->private_data = dev;

	return 0;
}

int mg_iap_release(struct inode *inode, struct file *filp)
{   
	struct mg_data *dev = filp->private_data; 
	
	printk("[Driver]into mg_iap_release\n");
	
	if (dev) {
		kfree(dev);
	}
	printk("[Driver]into mg_iap_release OVER\n");

	return 0;
}
ssize_t mg_iap_write(struct file *filp, const char *buff,    size_t count, loff_t *offp)
{  

    int ret;
    char *tmp;
#ifdef RANDY_DEBUG	
	printk("[Driver]into mg_iap_write\n");
	#endif

 	if (count > 128)
       	count = 128;

    tmp = kmalloc(count, GFP_KERNEL);
    
    if (tmp == NULL)
        return -ENOMEM;

    if (copy_from_user(tmp, buff, count)) {
        return -EFAULT;
    }
	#ifdef RANDY_DEBUG	
	int i = 0;
	printk("[Driver]Writing : ");
	for(i = 0; i < count; i++)
	{
		printk("%4x", tmp[i]);
	}
	printk("\n");
#endif
//	ret = i2c_smbus_write_i2c_block_data(private_ts->client, 0, count, tmp);
	ret = i2c_master_send(private_ts->client, tmp, count);
    if (!ret) 
		printk("[Driver] i2c_master_send fail, ret=%d \n", ret);
	
    kfree(tmp);
#ifdef RANDY_DEBUG	
	printk("[Driver]into mg_iap_write OVER\n");
#endif
    return ret;

}

ssize_t mg_iap_read(struct file *filp, const char *buff,    size_t count, loff_t *offp){    

    char *tmp;
    int ret;  
    
#ifdef RANDY_DEBUG	
	printk("[Driver]into mg_iap_read\n");
#endif
    if (count > 128)
        count = 128;
	/**********  randy delete for return 816 **************/
	if(command_flag == 1) {
		#ifdef RANDY_DEBUG	
		printk("<<Waiting>>");
		#endif
		return -1;
	} else {
		if(ack_flag == 1) {
			copy_to_user(buff, read_buf, COMMAND_BIT);
#ifdef RANDY_DEBUG	
			int i = 0;
			printk("[Driver]Reading : ");
			for(i = 0; i < COMMAND_BIT; i++) {
				printk("%4x", read_buf[i]);
			}
			printk("\n");
#endif
			ack_flag = 0;
			return 5;
		}
	}
	/**********  randy delete for return 816 **************/

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;

//	ret = i2c_smbus_read_i2c_block_data(private_ts->client, 0, count, tmp);
	ret = i2c_master_recv(private_ts->client, tmp, count);
    	if (ret >= 0)
        	copy_to_user(buff, tmp, count);
    
	#ifdef RANDY_DEBUG	
	int i = 0;
	printk("[Driver]Reading : ");
	for(i = 0; i < count; i++) {
		printk("%4x", tmp[i]);
	}
	printk("\n");
	#endif
    kfree(tmp);
#ifdef RANDY_DEBUG	
	printk("[Driver]into mg_iap_read OVER\n");
	#endif

    return ret;
}


static int Act_FWPoll = 0;
static int Rtn_FWPoll = 0;

int FWRtnDecode()
{
	if( (read_buf[0]==0xab)&&
		(read_buf[1]==0x01)&&
		(read_buf[2]==0x55)		
		){
			return 1;
		}
		
	if( (read_buf[0]==0xab)&&
		(read_buf[1]==0x01)&&
		(read_buf[2]==0xCC)		
		){
			return -1;
		}		

	return 0;
}

int FWPoll_1()
{
	Act_FWPoll = 1;
	command_flag = 1;
	Rtn_FWPoll = 0;

	return 1;
}

int FWPoll_2()
{
	int PollCnt = 0;
	
	while(Act_FWPoll)
	{
		//Delay
		msleep(10);
		
		//
		PollCnt++;
		
		if( PollCnt>1000 )
		{
			Act_FWPoll = 0;
			command_flag = 0;
			return 0;
		}
	}	
	
	if( Rtn_FWPoll==-1 )  
	{
		printk("FW update return fail\n");
		return 0;	
	}
	
	return 1;
}

int ReadFWCheckSum()
{
	int i, j;
	int ret = 0;
	int PkgCnt = 0;
	u_int8_t commandBuf[5];
		
	int DataLen = private_ts->pdata->data_len;
	unsigned char *p_data = private_ts->pdata->data;
	
	#ifdef RANDY_DEBUG    
	printk("================ Get Check Sum =================\n");	
	#endif
	//Reset 
	mg8698s_reset(private_ts, 50);

	//Action 1 CMD( I/O )
	#ifdef RANDY_DEBUG	
	printk("[Driver ] IOCTL_IAP_MODE\n");
	#endif

	FWPoll_1();
	
	#ifdef RANDY_DEBUG
	printk("IOCTL_IAP_MODE\n");	
	#endif
	
	ret = i2c_master_send(private_ts->client, 
						command_list[10], 5);	
	
	if(ret < 0)
	{
		printk("[Driver ]IOCTL_IAP_MODE error!!!!!\n");
		return -1;
	}	
	
	if( !FWPoll_2() )
		return -1;
		
	
	//Action 2 Command
	#ifdef RANDY_DEBUG	
	printk("[Driver ] Read Check Sum\n");
	#endif

//	FWPoll_1();
	CS_flag	 = 1;
	command_flag = 1;
	
	commandBuf[0] = command_list[14][0];
	commandBuf[1] = command_list[14][1];
	commandBuf[2] = command_list[14][2];
	commandBuf[3] = p_data[8];
	commandBuf[4] = p_data[9];
	
	ret = i2c_master_send(private_ts->client, 
						commandBuf, 5);	

	#ifdef RANDY_DEBUG						
	printk("[Driver] Cmd Check Sum : %4x,%4x,%4x,%4x,%4x\n", commandBuf[0], commandBuf[1],commandBuf[2],commandBuf[3],commandBuf[4]);
	#endif
	
	if(ret < 0)
	{
		printk("[Driver ]Get Check Sum error!!!!!\n");
		return -1;
	}	
	
	msleep(1000);
	
	ack_flag = 0;
	
	/**********  randy delete for return 816 **************/
	#ifdef RANDY_DEBUG						
	printk("[Driver] Read Check Sum : %4x,%4x,%4x,%4x,%4x\n", ver_buf[0], ver_buf[1],ver_buf[2],ver_buf[3],ver_buf[4]);
	#endif
//	if( !FWPoll_2() )
//		return -1;	
		
	return 0;
}

int UpdateFW()
{
	int i, j;
	int ret = 0;
	int PkgCnt = 0;
	
	int DataLen = private_ts->pdata->data_len;
	unsigned char *p_data = private_ts->pdata->data;
	
//	#ifdef RANDY_DEBUG    
	printk("================start update=================\n");	
//	#endif
	//Reset 
	mg8698s_reset(private_ts, 50);
	
	//Action 1 CMD( I/O )
	#ifdef RANDY_DEBUG	
	printk("[Driver ] IOCTL_IAP_MODE\n");
	#endif

	FWPoll_1();
	
	ret = i2c_master_send(private_ts->client, 
						command_list[10], 5);	
	
	if(ret < 0)
	{
		printk("[Driver ]IOCTL_IAP_MODE error!!!!!\n");
		return -1;
	}	
	
	if( !FWPoll_2() )
		return -1;	

	//Action 2 CMD( I/O )
	#ifdef RANDY_DEBUG	
	printk("[Driver ] IOCTL_ACTION2\n");
	#endif
		
	FWPoll_1();
	
	ret = i2c_master_send(private_ts->client, 
						command_list[12], 5);	

	if(ret < 0)
	{
		printk("[Driver ]IOCTL_ACTION2 error!!!!!\n");
		return -1;
	}	

	if( !FWPoll_2() )
		return -1;
	
	//Loop To Read File & Load to CHip
	#ifdef RANDY_DEBUG
	printk("================update firmware=================\n");		
	#endif
	
	p_data += 16;
	PkgCnt = (DataLen-16)/13;
	for( i=0; i<PkgCnt; i++ )
	{
		FWPoll_1();
		
		#ifdef RANDY_DEBUG
		for( j=0; j<13; j++ )
			printk("0x%.2x ", *(p_data+j));
		#endif
				
		ret = i2c_master_send(private_ts->client, 
							p_data, 13);			
		
		if(ret < 0)
		{
			printk("[Driver ]IOCTL_ACTION2 error!!!!!\n");
			return -1;
		}	

		if( !FWPoll_2() )
		{
			printk("[Driver ]IOCTL_ACTION2 error!!!!!\n");
			return -1;	
		}
					
		p_data += 13;
	}
	
//	#ifdef RANDY_DEBUG
	printk("================end update=================\n");	
//	#endif
	//Jump to App/Or Reset
	mg8698s_reset(private_ts, 50);

	return 0;
}

unsigned char BL_Ver_M = 0;
unsigned char BL_Ver_S = 0;

int UpdateFW_2()
{
	int i, j;
	int ret = 0;
	int PkgCnt = 0;
	int PkgIdx = 0;
	int FWBuf_Idx = 0;
	int bLoadEnd = 0;
	int BigIdx = 0;
	
	int DataLen = private_ts->pdata->data_len;
	unsigned char *p_data = private_ts->pdata->data;
	
	unsigned char FWBuffer[1031];
	
//	#ifdef RANDY_DEBUG    
	printk("================start update=================\n");	
//	#endif
	//Reset 
	mg8698s_reset(private_ts, 50);
	
	//Action 1 CMD( I/O )
	#ifdef RANDY_DEBUG	
	printk("[Driver ] IOCTL_IAP_MODE\n");
	#endif

	FWPoll_1();
	
	ret = i2c_master_send(private_ts->client, 
						command_list[10], 5);	
	
	if(ret < 0)
	{
		printk("[Driver ]IOCTL_IAP_MODE error!!!!!\n");
		return -1;
	}	
	
	if( !FWPoll_2() )
		return -1;	
		
	//Record Bootloader Version
	BL_Ver_M = read_buf[3];
	BL_Ver_S = read_buf[4];
	
	printk( "BL Version : %X.%X\n", BL_Ver_M, BL_Ver_S );

	//Action 2 CMD( I/O )
	#ifdef RANDY_DEBUG	
	printk("[Driver ] IOCTL_ACTION2\n");
	#endif
		
	FWPoll_1();
	
	ret = i2c_master_send(private_ts->client, 
						command_list[12], 5);	

	if(ret < 0)
	{
		printk("[Driver ]IOCTL_ACTION2 error!!!!!\n");
		return -1;
	}	

	if( !FWPoll_2() )
		return -1;
		
		
	
	//Loop To Read File & Load to CHip
	#ifdef RANDY_DEBUG
	printk("================update firmware=================\n");		
	#endif
	
if( (BL_Ver_M>3) || ((BL_Ver_M==3)&&(BL_Ver_S>=3)) )
{
	printk( "Start Mass Update Process\n" );
	
	//Prepeare Buffer Parameters
	p_data += 16;
	PkgCnt = (DataLen-16)/13;
	PkgIdx = 0;		//Index for 13 byte package in fw array
	FWBuf_Idx = 0;	//Index for 1k fw buffer( byte )
	bLoadEnd = 0;	//Boolean to flag if the loading process ended
	BigIdx = 0;
	
	if( PkgCnt<0 )
	{
		printk( "FW Data Size Error(%d)\n", PkgCnt );
		return -1;
	}
	
	printk( "Total Package Count in FW Array : %d\n", PkgCnt );
	
	//Loop to Load FW
	while( !bLoadEnd )
	{	
			FWPoll_1();
			
			//Prepare Data
			memset( FWBuffer, 0xff, sizeof(FWBuffer) );
			
			//Prepare Header
			FWBuffer[0] = 0xAD;
			FWBuffer[1] = 0x15;
			
			FWBuffer[2] = p_data[2];
			FWBuffer[3] = p_data[3];

			FWBuffer[4] = 0x04;
			FWBuffer[5] = 0x00;			
			
			FWBuffer[6] = 0x00;
			
			printk( "Data(%d), AddH=%x, AddL=%x\n", BigIdx, FWBuffer[2], FWBuffer[3]  );
			BigIdx++;
			
			FWBuf_Idx = 7;
			
			
			while(  (FWBuf_Idx<1031) )
			{
				if( PkgIdx<PkgCnt )
				{
					for( j=5; j<13; j++ )
					{
						FWBuffer[FWBuf_Idx] = p_data[j];
						FWBuf_Idx++;
					}	
					
					//Increase Small Package Index & Pointer					
					p_data += 13;
					PkgIdx++;						
				}
				else
				{
					break;
				}
			}
			
			if( PkgIdx>=PkgCnt )
				bLoadEnd = 1;
			
			//Send I2C Data
			ret = i2c_master_send(private_ts->client, 
								FWBuffer, 1031);			
			
			if(ret < 0)
			{
				printk("[Driver ]IOCTL_ACTION2 error!!!!!\n");
				return -1;
			}	

			//Wait Data Result
			if( !FWPoll_2() )
			{
				printk("[Driver ]IOCTL_ACTION2 error!!!!!\n");
				return -1;	
			}	

			if( bLoadEnd )
				break;
	}
	
	printk( "Total Package Count Processed : %d\n", PkgIdx );
}
else
{	
	printk( "Start 8-Bytes Update Process\n" );
	
	p_data += 16;
	PkgCnt = (DataLen-16)/13;
	for( i=0; i<PkgCnt; i++ )
	{
		FWPoll_1();
		
		#ifdef RANDY_DEBUG
		for( j=0; j<13; j++ )
			printk("0x%.2x ", *(p_data+j));
		#endif
				
		ret = i2c_master_send(private_ts->client, 
							p_data, 13);			
		
		if(ret < 0)
		{
			printk("[Driver ]IOCTL_ACTION2 error!!!!!\n");
			return -1;
		}	

		if( !FWPoll_2() )
		{
			printk("[Driver ]IOCTL_ACTION2 error!!!!!\n");
			return -1;	
		}
					
		p_data += 13;
	}
}
	
//	#ifdef RANDY_DEBUG
	printk("================end update=================\n");	
//	#endif
	//Jump to App/Or Reset
	mg8698s_reset(private_ts, 50);

	return 0;
}

int ReadVersion()
{
	u_int8_t ret = 0;
	
	#ifdef RANDY_DEBUG    
	printk("[Driver ] IOCTL_READ_VERSION\n");
	#endif
	
	memset( ver_buf, 0, sizeof(ver_buf) );
	
	#ifdef RANDY_DEBUG						
	printk("[Driver ]DISABLE !!!!!!!\n");
	#endif
	
	ret = i2c_master_send(private_ts->client, 
						command_list[2], COMMAND_BIT);
	if(ret < 0)
	{
		printk("[Driver ]DISABLE  Error !!!!!!!\n");
		return -1;
	}
	
	#ifdef RANDY_DEBUG						
	printk("[Driver ]ENABLE !!!!!!!\n");
	#endif
	
	ret = i2c_master_send(private_ts->client, 
						command_list[1], COMMAND_BIT);
	if(ret < 0)
	{
		printk("[Driver ]ENABLE  Error !!!!!!!\n");
		return -1;
	}
	mdelay(500);
	ver_flag = 1;
	command_flag = 1;

	ret = i2c_master_send(private_ts->client, 
						command_list[4], COMMAND_BIT);
	if(ret < 0)
	{
		printk("[Driver ]IOCTL_READ_VERSION error!!!!!\n");
		return -1;
	}
	ver_flag = 1;
	command_flag = 1;
	
	mdelay(300);
	mdelay(800);
	/**********  randy delete for return 816 **************/
	ack_flag = 0;
	/**********  randy delete for return 816 **************/
	#ifdef RANDY_DEBUG						
	printk("[Driver] Read Fw Version : %4x,%4x,%4x,%4x,%4x\n", ver_buf[0], ver_buf[1],ver_buf[2],ver_buf[3],ver_buf[4]);
	#endif
	return 0;
}

int ReadID()
{
	int ret = 0;
#ifdef RANDY_DEBUG
	printk("[Driver ] IOCTL_READ_ID\n");
#endif
	
	memset( ver_buf, 0, sizeof(ver_buf) );

	#ifdef RANDY_DEBUG						
	printk("[Driver ]DISABLE !!!!!!!\n");
	#endif
	
	ret = i2c_master_send(private_ts->client, 
						command_list[2], COMMAND_BIT);
	if(ret < 0)
	{
		printk("[Driver ]DISABLE  Error !!!!!!!\n");
		return -1;
	}
	#ifdef RANDY_DEBUG						
	printk("[Driver ]ENABLE !!!!!!!\n");
	#endif
	
	ret = i2c_master_send(private_ts->client, 
						command_list[1], COMMAND_BIT);
	if(ret < 0)
	{
		printk("[Driver ]ENABLE  Error !!!!!!!\n");
		return -1;
	}
	mdelay(500);
	ver_flag = 1;
	command_flag = 1;

	ret = i2c_master_send(private_ts->client, 
						command_list[3], COMMAND_BIT);
	if(ret < 0)
	{
		printk("[Driver ]IOCTL_READ_VERSION error!!!!!\n");
		return -1;
	}
	ver_flag = 1;
	command_flag = 1;
	
	mdelay(300);
	mdelay(800);
	/**********  randy delete for return 816 **************/
	ack_flag = 0;
	/**********  randy delete for return 816 **************/
	#ifdef RANDY_DEBUG						
	printk("[Driver] Read ID Version : %4x,%4x,%4x,%4x,%4x\n", ver_buf[0], ver_buf[1],ver_buf[2],ver_buf[3],ver_buf[4]);
	#endif
}



static int IC_ProjID = 0;
static int IC_IC_ID  = 0;

static int IC_FW_Ver_1 = 0;
static int IC_FW_Ver_2 = 0;
static int IC_FW_Ver_3 = 0;

static int UF_ProjID = 0;
static int UF_IC_ID  = 0;

static int UF_FW_Ver_1 = 0;
static int UF_FW_Ver_2 = 0;
static int UF_FW_Ver_3 = 0;

static unsigned char IC_CS_L = 0;
static unsigned char IC_CS_H = 0;

static unsigned char UF_CS_L = 0;
static unsigned char UF_CS_H = 0;


int CheckUpdate()
{	
	int DataLen = private_ts->pdata->data_len;
	unsigned char *p_data = private_ts->pdata->data;

	if ((DataLen==0) || (p_data == NULL))
	{
		printk("No firmware for update\n");
		return 0;
	}
	
	//Get Version & Platform Info. from Update File Package
	ReadVersion();  // need check return
	if( (ver_buf[0]==0xde)&&(ver_buf[1]==0xee) )
	{	
		//Get Version Info
		IC_FW_Ver_1 	= ver_buf[3];
		IC_FW_Ver_2 	= ver_buf[4];
		IC_FW_Ver_3 	= ver_buf[2];
		
		#ifdef RANDY_DEBUG
		printk( "IC Version = %x.%x(%x)\n", IC_FW_Ver_1, IC_FW_Ver_2, IC_FW_Ver_3 );		
		#endif
	}
	else
	{
		printk( "[Drv:FU] Can't Get FW Version\n" );
		return 1;		//Condition 1 : Can Not Read F/W Version from IC -- Update			
	}
		
	ReadID();
	if( (ver_buf[0]==0xde)&&(ver_buf[1]==0xdd) )
	{	
		//Get ID Info
		IC_IC_ID 	= ver_buf[2];
		IC_ProjID 	= ver_buf[4];
		
		#ifdef RANDY_DEBUG
		printk( "IC Project ID=%X, IC ID=%X\n", IC_ProjID, IC_IC_ID );		
		#endif
	}
	else 
	{
		printk( "[Drv:FU] Can't Get Project ID\n" );
		return 1;		//Condition 1 : Can Not Read F/W Version from IC -- Update			
	}
	
	//Read Check Sum
   ReadFWCheckSum();
   
   IC_CS_L = ver_buf[3];
   IC_CS_H = ver_buf[4];
   
   #ifdef RANDY_DEBUG
   printk( "IC CheckSum=%X,%X\n", IC_CS_L, IC_CS_H );		
   #endif
	
	//Get Version & Platform Info. from IC
   UF_ProjID = p_data[2];
   UF_IC_ID  = p_data[0];

   UF_FW_Ver_1 = p_data[4];
   UF_FW_Ver_2 = p_data[5];
   UF_FW_Ver_3 = p_data[3];

   UF_CS_L = p_data[6];
   UF_CS_H = p_data[7];      
   
   #ifdef RANDY_DEBUG
   printk( "UF Version = %x.%x(%x)\n", UF_FW_Ver_1, UF_FW_Ver_2, UF_FW_Ver_3 );
   printk( "UF Project ID=%X, IC ID=%X\n", UF_ProjID, UF_IC_ID );		   
   printk( "UF CheckSum=%X,%X\n", UF_CS_L, UF_CS_H );		
   #endif
   
 
   
   //Compare Info. between IC & Update File
   if( (IC_IC_ID!=UF_IC_ID)||(IC_ProjID!=UF_ProjID) )
   {
		printk( "[Drv:FU] IC/Project ID Not Match\n" );
		return 1;		//Condition 2 : Platform Info. different from IC & Update File Package -- Update
   }

   //Comparing of forst Layer
   if( UF_FW_Ver_1>IC_FW_Ver_1 )
   {
		printk( "[Drv:FU] Older version in IC\n" );
		return 1;		//Condition 3 : Version number of Update Package larger than Version Number of IC -- Update
   }
   else if( UF_FW_Ver_1==IC_FW_Ver_1 )
   {
		//Comparing of Second Layer
		if( UF_FW_Ver_2>IC_FW_Ver_2 )
		{
			printk( "[Drv:FU] Older version in IC\n" );
			return 1;		//Condition 3 : Version number of Update Package larger than Version Number of IC -- Update
		}		
		else if( UF_FW_Ver_2==IC_FW_Ver_2 )
		{
			//Comparing of Third Layer
			if( UF_FW_Ver_3>IC_FW_Ver_3 )
			{
				printk( "[Drv:FU] Older version in IC\n" );
				return 1;		//Condition 3 : Version number of Update Package larger than Version Number of IC -- Update
			}		
			else if( UF_FW_Ver_3==IC_FW_Ver_3 )
			{
				// Check Check Sum
				if( (UF_CS_L==IC_CS_L)&&(UF_CS_H==IC_CS_H) )
				{
					//Check Sum Successed
					printk( "Check Sum Successed\n" );
				}
				else
				{
					printk( "[Drv:FU] Check Sum Error\n" );
					return 1;
				}
				
			}
		}
   }  		
	
	return 0;
}

int mg_iap_ioctl(struct inode *inode, struct file *filp,    unsigned int cmd, unsigned long arg)
{
	u_int8_t ret = 0;
	command_flag = 1;

	#ifdef RANDY_DEBUG	
	printk("[Driver ]mg_iap_ioctl  Cmd =%x\n", cmd);
//		printk("[Driver ]into mg_iap_ioctl1\n");
	#endif

	switch (cmd) {        
		case IOCTL_DEFAULT:
			#ifdef RANDY_DEBUG	
			printk("[Driver ] IOCTL_DEFAULT  \n");
			#endif
			break;   		
		case IOCTL_I2C_SLAVE: 
			#ifdef RANDY_DEBUG	
			printk("[Driver ] IOCTL_I2C_SLAVE  \n");
			#endif		
			//gpio_set_value(BABBAGE_TOUCH_RESET,1);
			//printk("  the BABBAGE_TOUCH_RESET pin value is %d\n", gpio_get_value(BABBAGE_TOUCH_RESET));
			break;   

		case IOCTL_RESET:
			#ifdef RANDY_DEBUG	
			printk("[Driver ] IOCTL_RESET  \n");
				#endif			
		//gpio_set_value(BABBAGE_TOUCH_RESET,0);
		//printk("  the BABBAGE_TOUCH_RESET pin value is %d\n", gpio_get_value(BABBAGE_TOUCH_RESET));	
			break;
		case IOCTL_READ_ID:        
		#ifdef RANDY_DEBUG
			printk("[Driver ] IOCTL_READ_ID\n");
			#endif
			printk("[Driver ]DISABLE !!!!!!!\n");
			ret = i2c_master_send(private_ts->client, 
								command_list[2], COMMAND_BIT);
			if(ret < 0)
			{
				printk("[Driver ]DISABLE  Error !!!!!!!\n");
				return -1;
			}
			printk("[Driver ]ENABLE !!!!!!!\n");
			ret = i2c_master_send(private_ts->client, 
								command_list[1], COMMAND_BIT);
			if(ret < 0)
			{
				printk("[Driver ]ENABLE  Error !!!!!!!\n");
				return -1;
			}
			mdelay(500);
			ver_flag = 1;
			command_flag = 1;
		
			ret = i2c_master_send(private_ts->client, 
								command_list[3], COMMAND_BIT);
			if(ret < 0)
			{
				printk("[Driver ]IOCTL_READ_VERSION error!!!!!\n");
				return -1;
			}
			ver_flag = 1;
			command_flag = 1;
			
			mdelay(300);
			mdelay(800);
			/**********  randy delete for return 816 **************/
			ack_flag = 0;
			/**********  randy delete for return 816 **************/
			printk("[Driver] Read ID Version : %4x,%4x,%4x,%4x,%4x\n", ver_buf[0], ver_buf[1],ver_buf[2],ver_buf[3],ver_buf[4]);

			break;    
		case IOCTL_READ_VERSION:    
			#ifdef RANDY_DEBUG    
			printk("[Driver ] IOCTL_READ_VERSION\n");
			#endif
			printk("[Driver ]DISABLE !!!!!!!\n");
			ret = i2c_master_send(private_ts->client, 
								command_list[2], COMMAND_BIT);
			if(ret < 0)
			{
				printk("[Driver ]DISABLE  Error !!!!!!!\n");
				return -1;
			}
			printk("[Driver ]ENABLE !!!!!!!\n");
			ret = i2c_master_send(private_ts->client, 
								command_list[1], COMMAND_BIT);
			if(ret < 0)
			{
				printk("[Driver ]ENABLE  Error !!!!!!!\n");
				return -1;
			}
			mdelay(500);
			ver_flag = 1;
			command_flag = 1;
		
			ret = i2c_master_send(private_ts->client, 
								command_list[4], COMMAND_BIT);
			if(ret < 0)
			{
				printk("[Driver ]IOCTL_READ_VERSION error!!!!!\n");
				return -1;
			}
			ver_flag = 1;
			command_flag = 1;
			
			mdelay(300);
			mdelay(800);
			/**********  randy delete for return 816 **************/
			ack_flag = 0;
			/**********  randy delete for return 816 **************/
			printk("[Driver] Read Fw Version : %4x,%4x,%4x,%4x,%4x\n", ver_buf[0], ver_buf[1],ver_buf[2],ver_buf[3],ver_buf[4]);

			break;    
			
		case IOCTL_ENDING:        
			#ifdef RANDY_DEBUG	
			printk("[Driver ] IOCTL_ENDING\n");
			#endif
			

			ret = i2c_master_send(private_ts->client, 
								command_list[13], COMMAND_BIT);
			if(ret < 0)
			{
				printk("[Driver ]IOCTL_ENDING error!!!!!\n");
				return -1;
			}
		/******/
		command_flag = 0;
		/******/
			break;        
		case IOCTL_ACTION2:        
			#ifdef RANDY_DEBUG	
			printk("[Driver ] IOCTL_ACTION2\n");
			#endif

			ret = i2c_master_send(private_ts->client, 
								command_list[12], COMMAND_BIT);

			if(ret < 0)
			{
				printk("[Driver ]IOCTL_ACTION2 error!!!!!\n");
				return -1;
			}

			break;        
		case IOCTL_IAP_MODE:
			#ifdef RANDY_DEBUG	
			printk("[Driver ] IOCTL_IAP_MODE\n");
			#endif
			
			ret = i2c_smbus_write_i2c_block_data(private_ts->client,
									0, COMMAND_BIT, command_list[10]);
			if(ret < 0)
			{
				printk("[Driver ]IOCTL_IAP_MODE error!!!!!\n");
				return -1;
			}

			break;
		case IOCTL_CALIBRATION:
			command_flag = 0;
			//+++++++add for touch calibration
			printk("[Bohai]======ioctl_calibration!!\n");
			calibration_flag=1;
			//+++++++by Jay 2011/11/14
			#ifdef RANDY_DEBUG	
			printk("[Driver ] IOCTL_CALIBRATION1\n");
			#endif

			//disable_irq_nosync(private_ts->client->irq);
			printk("[Driver ]DISABLE !!!!!!!\n");
			ret = i2c_master_send(private_ts->client, 
								command_list[2], COMMAND_BIT);
			if(ret < 0)
			{
				printk("[Driver ]DISABLE  Error !!!!!!!\n");
				return -1;
			}
			printk("[Driver ]ENABLE !!!!!!!\n");
			ret = i2c_master_send(private_ts->client, 
								command_list[1], COMMAND_BIT);
			if(ret < 0)
			{
				printk("[Driver ]ENABLE  Error !!!!!!!\n");
				return -1;
			}
			//enable_irq(private_ts->client->irq);

			mdelay(2000);
			disable_irq_nosync(private_ts->client->irq);

			printk("[Driver ]Calibrate !!!!!!!\n");

			ret = i2c_smbus_write_i2c_block_data(private_ts->client,
						0, COMMAND_BIT, command_list[0]);
			if(ret < 0)
			{
				printk("[Driver ]Calibrate  Write error!!!!!\n");
				return -1;
			}
			
			mdelay(10);
			enable_irq(private_ts->client->irq);
			command_flag = 1;
			break;
		default:            
			#ifdef RANDY_DEBUG	
			printk("[Driver ] default  \n");
			#endif
			break;   
	}     
//	command_flag = 1;
	return 0;
}

 
struct file_operations mg_touch_fops = {    
        open:       mg_iap_open,    
        write:      mg_iap_write,    
        read:		mg_iap_read,    
        release:    mg_iap_release,    
        compat_ioctl:   mg_iap_ioctl,  
 };
 

static ssize_t mg_fw_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	int err;
 // printk("[Touch]:(%s) touch_FW_version buf[0]=%4x, buf[1]= %4x,  buf[2]=%4x, buf[3]= %4x, buf[4]=%4x\n",__FUNCTION__, ver_buf[0][0], ver_buf[0][1], ver_buf[0][2], ver_buf[0][3], ver_buf[0][4]);
	#ifdef RANDY_DEBUG    
	printk("[Driver ] IOCTL_READ_VERSION\n");
	#endif
	printk("[Driver ]DISABLE !!!!!!!\n");
	err = i2c_master_send(private_ts->client, 
						command_list[2], COMMAND_BIT);
	if(err < 0) {
		printk("[Driver ]DISABLE  Error !!!!!!!\n");
		return -1;
	}
	printk("[Driver ]ENABLE !!!!!!!\n");
	err = i2c_master_send(private_ts->client, 
						command_list[1], COMMAND_BIT);
	if(err < 0) {
		printk("[Driver ]ENABLE  Error !!!!!!!\n");
		return -1;
	}
	//mdelay(50);
	//+++add for touch issue
	mdelay(500);
	ver_flag = 1;
	command_flag = 1;

	//+++by Jay 2011/09/07
	err = i2c_master_send(private_ts->client, 
						command_list[4], COMMAND_BIT);
	if(err < 0) {
		printk("[Driver ]IOCTL_READ_VERSION error!!!!!\n");
		return -1;
	}
	ver_flag = 1;
	command_flag = 1;
	
	//++++++add for nomal mode
	mdelay(30);
	mdelay(100);
	/**********  randy delete for return 816 **************/
	ack_flag = 0;
	/**********  randy delete for return 816 **************/
	//+++++modify 4x => x
	return sprintf(buf,"%x,%x,%x,%x,%x\n",  ver_buf[0], ver_buf[1],ver_buf[2],ver_buf[3],ver_buf[4]);
	//+++++by Jay 2011/08/02
}/*
static ssize_t mg_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	
  printk("[Touch]:(%s) touch_ID_version = %s \n",__FUNCTION__, ver_buf[1]);
	
	return sprintf(buf, "DeviceID = %x, ProjectID = %x\n",ver_buf[1][3], ver_buf[1][4]);
}
*/
static DEVICE_ATTR(fw_version, 0666, mg_fw_show, NULL);
static ssize_t mg_touch_calibration(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%d",IsCalibration);
}
static DEVICE_ATTR(touch_cal, 0666, mg_touch_calibration, NULL);

//static DEVICE_ATTR(id_version, 0666, mg_id_show, NULL);

static struct attribute *mg_attributes[] = {
	&dev_attr_fw_version.attr,
	//++++++add for touch calibration
	&dev_attr_touch_cal.attr,
	//------by Jay 2011/11/14
//	&dev_attr_id_version.attr,
	NULL
};

static const struct attribute_group mg_attr_group = {
	.attrs = mg_attributes,
};

static struct i2c_driver mg_driver;

static irqreturn_t mg_irq(int irq, void *_mg)
{
	struct mg_data *mg = _mg;
	schedule_work(&mg->work);
	return IRQ_HANDLED;
}

static inline void mg_mtreport(struct mg_data *mg)
{
	//delete for angry birds
	u16 x,y;
	
	input_report_key(mg->dev, BTN_TOUCH, 1);
	input_report_abs(mg->dev, ABS_MT_TRACKING_ID, mg->id);
	//+++++by Jay 20110901
	input_report_abs(mg->dev, ABS_MT_TOUCH_MAJOR, mg->w);
	input_report_abs(mg->dev, ABS_MT_WIDTH_MAJOR, 0);

	if (mg->pdata->swap_xy) {
		x = (mg->pdata->xpol) ? (mg->pdata->ymax + mg->pdata->ymin - mg->y) : mg->y;
		y = (mg->pdata->ypol) ? (mg->pdata->xmax + mg->pdata->xmin - mg->x) : mg->x;
	} else {
		x = (mg->pdata->xpol) ? (mg->pdata->xmax + mg->pdata->xmin - mg->x) : mg->x;
		y = (mg->pdata->ypol) ? (mg->pdata->ymax + mg->pdata->ymin - mg->y) : mg->y;
	}
	//printk("mg(%d,%d) x:%d, y:%d\n", mg->x, mg->y, x, y);
	input_report_abs(mg->dev, ABS_MT_POSITION_X, x); 
	input_report_abs(mg->dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(mg->dev);
}

static void mg_i2c_work(struct work_struct *work)
{
	int i = 0;
	u_int8_t ret = 0;
	struct mg_data *mg = container_of(work, struct mg_data, work);

	/**********  randy delete for return 1223 **************/
	//	u8 read_buf[BUF_SIZE];	/* buffer for first point of multitouch */
	/**********  randy delete for return 1223 **************/
	//	u8 recv_buf[BUF_SIZE]; /* buffer for second point of multitouch */

	for(i = 0; i < BUF_SIZE; i ++)	
			read_buf[i] = 0;

	if(command_flag == 1) 
	{
		if( Act_FWPoll )
		{
			ret = i2c_master_recv(mg->client, read_buf, 5);
			if(ret < 0) {
				printk("Read error!!!!!\n");
				return;
			}		
		
			Rtn_FWPoll = FWRtnDecode();
			
			if( Rtn_FWPoll )
			{
				Act_FWPoll = 0;
				command_flag = 0;	

				#ifdef RANDY_DEBUG	
				printk("FW Update Return : " );
				for(i = 0; i < 5; i ++) 	
					printk("%4x ", read_buf[i]);
					
				printk("\n");
				#endif		
				
				return ;			
			}
		}
		else
		{
//			ret = i2c_master_recv(mg->client, read_buf, COMMAND_BIT);
			ret = i2c_master_recv(mg->client, read_buf, 5);
			if(ret < 0) {
				printk("Read error!!!!!\n");
				return;
			}		
		}
		
		//++++++add for touch calibration --------copy reg data to global var
		if(calibration_flag==1) {
			for(i=0;i<5;i++)
				cal_ack[i]=read_buf[i];
			for(i = 0; i < 5; i ++) 	
				printk("%4x", cal_ack[i]);
			printk("\n");

			if(cal_ack[4]==0x55)
				IsCalibration=1;
			else
				IsCalibration=0;
			//--------by Jay 2011/11/14
			printk("[Bohai]======touch calibration!!\n");
			calibration_flag=0;
		}
		//-------by Jay 2011/11/14
		#ifdef RANDY_DEBUG	
		printk("\nRecieve Command ACK : "); 
		for(i = 0; i < 5; i ++) 	
			printk("%4x", read_buf[i]);
		printk("\n");
		#endif
			//++++add for touch F/W
		if( ver_flag == 1 || id_flag == 1 || CS_flag==1 ) {
			printk("read version id cs\n");
			for(i = 0; i < 5; i ++)
				ver_buf[i] = read_buf[i];
						//++++++add for nomal mode
			mdelay(20);
			ret =i2c_smbus_write_i2c_block_data(mg->client, 0, COMMAND_BIT, nomal_mode);
			if(ret<0) {
				printk("Wistron==>show f/w!!\n");
				return;
			}
			//++++++add for nomal mode
			ver_flag = 0;
			id_flag = 0;
		}
		command_flag = 0;
		/**********  randy delete for return 816 **************/
		ack_flag = 1;
		/**********  randy delete for return 816 **************/
		return;
		//++++++by Jay 2011/08/01	
	}

	ret = i2c_smbus_read_i2c_block_data(mg->client, 0x0, BUF_SIZE, read_buf);
	if(ret < 0)
	{
		printk("Read error!!!!!\n");
		return;
	}

	#ifdef RANDY_DEBUG	
	printk("\nRead: "); 
	for(i = 0; i < BUF_SIZE; i ++) 	
		printk("%4x", read_buf[i]);
	#endif

	if(read_buf[MG_MODE] == 2){

	}
	else
	{
		if (read_buf[ACTUAL_TOUCH_POINTS] > 0) {
			if(read_buf[ACTUAL_TOUCH_POINTS] >= 1) {
				mg->x = COORD_INTERPRET(read_buf[MG_POS_X_HI], read_buf[MG_POS_X_LOW]);
				mg->y = COORD_INTERPRET(read_buf[MG_POS_Y_HI], read_buf[MG_POS_Y_LOW]);
				mg->w = read_buf[MG_STATUS];
				mg->id = read_buf[MG_CONTACT_ID];
		//		input_report_abs(mg->dev, ABS_MT_TRACKING_ID, 0);
				mg_mtreport(mg);
			}
			if(read_buf[ACTUAL_TOUCH_POINTS] >= 2) {
				mg->x =	COORD_INTERPRET(read_buf[MG_POS_X_HI2], read_buf[MG_POS_X_LOW2]);
				mg->y = COORD_INTERPRET(read_buf[MG_POS_Y_HI2], read_buf[MG_POS_Y_LOW2]);
				mg->w = read_buf[MG_STATUS2];
				mg->id = read_buf[MG_CONTACT_ID2];
		//		input_report_abs(mg->dev, ABS_MT_TRACKING_ID, 1);
				mg_mtreport(mg);
			}

			if(read_buf[ACTUAL_TOUCH_POINTS] >= 3) {
				mg->x =	COORD_INTERPRET(read_buf[MG_POS_X_HI3], read_buf[MG_POS_X_LOW3]);
				mg->y = COORD_INTERPRET(read_buf[MG_POS_Y_HI3], read_buf[MG_POS_Y_LOW3]);
				mg->w = read_buf[MG_CONTACT_IDS3] & 0x0F;
				mg->id = (read_buf[MG_CONTACT_IDS3]>>4)&0x0F;
		//		input_report_abs(mg->dev, ABS_MT_TRACKING_ID, 1);
				mg_mtreport(mg);
			}		

			if(read_buf[ACTUAL_TOUCH_POINTS] >= 4) {
				mg->x =	COORD_INTERPRET(read_buf[MG_POS_X_HI4], read_buf[MG_POS_X_LOW4]);
				mg->y = COORD_INTERPRET(read_buf[MG_POS_Y_HI4], read_buf[MG_POS_Y_LOW4]);
				mg->w = read_buf[MG_CONTACT_IDS4] & 0x0F;
				mg->id = (read_buf[MG_CONTACT_IDS4]>>4)&0x0F;
		//		input_report_abs(mg->dev, ABS_MT_TRACKING_ID, 1);
				mg_mtreport(mg);
			}				

			if(read_buf[ACTUAL_TOUCH_POINTS] >= 5) {
				mg->x =	COORD_INTERPRET(read_buf[MG_POS_X_HI5], read_buf[MG_POS_X_LOW5]);
				mg->y = COORD_INTERPRET(read_buf[MG_POS_Y_HI5], read_buf[MG_POS_Y_LOW5]);
				mg->w = read_buf[MG_CONTACT_IDS5] & 0x0F;
				mg->id = (read_buf[MG_CONTACT_IDS5]>>4)&0x0F;
		//		input_report_abs(mg->dev, ABS_MT_TRACKING_ID, 1);
				mg_mtreport(mg);
			}							
			
		} else {
			input_report_key(mg->dev, BTN_TOUCH, 0);
			input_report_abs(mg->dev, ABS_MT_TOUCH_MAJOR, 0);
			input_mt_sync(mg->dev);
		}
		input_sync(mg->dev);
	}
}


static int mg_probe(struct i2c_client *client, const struct i2c_device_id *ids)
{
	int i;
	int err = 0;
	struct mg_data *mg;
	struct input_dev *input_dev;

	/* allocate mg data */
	mg = kzalloc(sizeof(struct mg_data), GFP_KERNEL);
	if (!mg)
		return -ENOMEM;

	mg->client = client;
	mg->pdata = client->dev.platform_data;
	touch_i2c_client=client;

	dev_info(&mg->client->dev, "device probing\n");
	i2c_set_clientdata(client, mg);
	mutex_init(&mg->lock);

    mg->firmware.minor = MISC_DYNAMIC_MINOR;
    mg->firmware.name = "MG_Update";
    mg->firmware.fops = &mg_touch_fops;
    mg->firmware.mode = S_IRWXUGO; 
    if (misc_register(&mg->firmware) < 0)
        printk("[mg]misc_register failed!!");
    else
        printk("[mg]misc_register finished!!"); 
		
	/* allocate input device for capacitive */
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&mg->client->dev, "failed to allocate input device \n");
		goto exit_kfree;
	}

	input_dev->name = "mg-capacitive";
	input_dev->phys = "I2C";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x0EAF;
	input_dev->id.product = 0x1020;
	mg->dev = input_dev;
	
	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	
	set_bit(KEY_BACK, input_dev->keybit);
        set_bit(KEY_MENU, input_dev->keybit);
        set_bit(KEY_HOME, input_dev->keybit);
		
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 8, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 8, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 4, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 256, 0, 0);

	if (mg->pdata->swap_xy) {
		input_set_abs_params(input_dev, ABS_MT_POSITION_X,
						0, mg->pdata->ymax, 0, 0);  
		input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
						0, mg->pdata->xmax, 0, 0);  
	} else {
		input_set_abs_params(input_dev, ABS_MT_POSITION_X,
						0, mg->pdata->xmax, 0, 0);  
		input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
						0, mg->pdata->ymax, 0, 0);  
	}
						
	err = input_register_device(input_dev);
	if (err)
		goto exit_input;
		
	mg8698s_power(mg, 1);
	mg8698s_reset(mg, 50);

	if (mg->pdata->init_irq) {
		mg->pdata->init_irq();
	}

	err = sysfs_create_group(&client->dev.kobj, &mg_attr_group);
	if (err)
		printk("[Touch](%s)failed to create flash sysfs files\n",__FUNCTION__);

	//+++++add for touch led issue
	init_completion(&mg_touch_event);
	
	//+++++by Jay 2011/09/17
	INIT_WORK(&mg->work, mg_i2c_work);

	/* request IRQ resouce */
	if (client->irq < 0) {
		dev_err(&mg->client->dev,
			"No irq allocated in client resources!\n");
		goto exit_input;
	}

	mg->irq = client->irq;
	err = request_irq(mg->irq, mg_irq,IRQF_DISABLED, MG_DRIVER_NAME, mg);

#ifdef CONFIG_HAS_EARLYSUSPEND
	mg->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	mg->early_suspend.suspend = mg_early_suspend;
	mg->early_suspend.resume = mg_late_resume;
	register_early_suspend(&mg->early_suspend);
#endif

#ifdef RANDY_DEBUG	
	printk("\nregister_early_suspend  OK !!!!\n"); 
#endif
		
	private_ts = mg;

#ifndef CONFIG_MG8698S_DISABLE_UPDATE
	if( CheckUpdate() )
	{
//		UpdateFW();
		UpdateFW_2();
	}
#endif
	
	return 0;

exit_input:
	input_unregister_device(mg->dev);
exit_kfree:
	kfree(mg);
	return err;
}

static int __devexit mg_remove(struct i2c_client *client)
{
	struct mg_data *mg = i2c_get_clientdata(client);
	
	if (mg->mg_touch_thread) {
		complete(&mg_touch_event);
		kthread_stop(mg->mg_touch_thread);
	}
	
	free_irq(mg->irq, mg);
	input_unregister_device(mg->dev);
	del_timer_sync(&mg->timer);
	cancel_work_sync(&mg->work);
	kfree(mg);
	
	return 0;
}


static int mg_suspend(struct i2c_client *client, pm_message_t state)
{
	int ret;
	struct mg_data *ts = i2c_get_clientdata(touch_i2c_client);

	if (ts->irq)
		disable_irq(touch_i2c_client->irq);
		
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->irq) /* if work was pending disable-count is now 2 */
		enable_irq(touch_i2c_client->irq);
	
	return 0;
}



static int mg_resume(struct i2c_client *client)
{
	struct mg_data *ts = i2c_get_clientdata(touch_i2c_client);
	
	if (ts->irq)
		enable_irq(touch_i2c_client->irq);
		
	return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void mg_early_suspend(struct early_suspend *h)
{
#ifdef RANDY_DEBUG	
	printk("\nmg_early_suspend\n"); 
#endif
	struct mg_data *ts = i2c_get_clientdata(touch_i2c_client);
	mg_suspend(touch_i2c_client, PMSG_SUSPEND);
	mg8698s_power(ts, 0);	
}

static void mg_late_resume(struct early_suspend *h)
{
#ifdef RANDY_DEBUG	
	printk("\nmg_late_resume\n"); 
#endif
	struct mg_data *ts = i2c_get_clientdata(touch_i2c_client);
	mg8698s_power(ts, 1);
	mg_resume(touch_i2c_client);
	msleep(50);
}
#endif

static struct i2c_device_id mg_id_table[] = {
    /* the slave address is passed by i2c_boardinfo */
    {MG_DRIVER_NAME, },
    {/* end of list */}
};

static struct i2c_driver mg_driver = {
	.driver = {
		.name	 = MG_DRIVER_NAME,

	},
	.id_table 	= mg_id_table,
	.probe 		= mg_probe,
	.remove 	= mg_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend = mg_early_suspend,
	.resume	 = mg_late_resume,
#endif
};

static int __init mg_init(void)
{
	return i2c_add_driver(&mg_driver);
}

static void mg_exit(void)
{
	i2c_del_driver(&mg_driver);
}
module_init(mg_init);
module_exit(mg_exit);

MODULE_AUTHOR("Randy pan <panhuangduan@morgan-touch.com>");
