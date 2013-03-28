///FOR eDP Driver ANX9804

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h> 
#include <linux/i2c.h>
//#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <asm/uaccess.h>
#include <mach/pinmux.h>
#include <mach/gpio.h>
#include <linux/platform_device.h>
#ifdef CONFIG_SN7325
#include <linux/sn7325.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
static struct early_suspend early_suspend;
static int early_suspend_flag = 0;
#endif
#include "DP_TX_Reg.h"
//TTL in  eDP out

extern int aml_anx9804_init();

static void anx9804_early_suspend(struct early_suspend *h);
static void anx9804_late_resume(struct early_suspend *h);
	
static struct i2c_client *anx9804_70_client;
static struct i2c_client *anx9804_72_client;

static int twx_i2c_write(struct i2c_client *i2client,unsigned char *buff, unsigned len)
{
    int res = 0;
    struct i2c_msg msg[] = {
        {
        .addr = i2client->addr,
        .flags = 0,
        .len = len,
        .buf = buff,
        }
    };
    //printk( "tc103 addr %#x reg:%#x  value:%#x \n",twx_tc103_client->addr,buff[0],buff[1]);	
    res = i2c_transfer(i2client->adapter, msg, 1);
    if (res < 0) {
        pr_err("%s: i2c transfer failed\n", __FUNCTION__);
    }
    
    return res;
}

static int twx_i2c_read(struct i2c_client *i2client,unsigned char *buff, unsigned len)
{
    int res = 0;
    struct i2c_msg msgs[] = {
        {
            .addr = i2client->addr,
            .flags = 0,
            .len = 1,
            .buf = buff,
        },
        {
            .addr = i2client->addr,
            .flags = I2C_M_RD,
            .len = len,
            .buf = buff,
        }
    };
    res = i2c_transfer(i2client->adapter, msgs, 2);
    if (res < 0) {
        pr_err("%s: i2c transfer failed\n", __FUNCTION__);
    }

    return res;
}

static unsigned char DP_TX_Read_Reg(unsigned char addr, unsigned char reg)
{
   unsigned char buff[3];
   int ret;  
   buff[0] = reg;
   if(addr == 0x70)
   {
     ret=twx_i2c_read(anx9804_70_client, buff, 1);
   }
    else
   {
     ret=twx_i2c_read(anx9804_72_client, buff, 1);
    }
	if (ret<0)
		return -1;
	else
		return buff[0];
}

static int DP_TX_Write_Reg(unsigned char addr, unsigned char reg, unsigned char dat)
{
   unsigned char tData[3];
   int ret;
   
   tData[0]=reg;
   tData[1]=dat;
   if(addr == 0x70)
   {
     ret=twx_i2c_write(anx9804_70_client,tData, 2);
   }
    else
   {
     ret=twx_i2c_write(anx9804_72_client,tData, 2);
    }
	return ret;
}

static void ANX9804_Initialize_4lane(void)
{
	unsigned char c;
    unsigned int i;
	
	printk("enter ANX9804_Initialize_4lane...\n");	
	//HW reset
	//DP_TX_Write_Reg(0x72, DP_TX_RST_CTRL_REG, DP_TX_RST_HW_RST);
	msleep(10);
	DP_TX_Write_Reg(0x72, DP_TX_RST_CTRL_REG, 0x00);
	//Power on total and select DP mode
       DP_TX_Write_Reg(0x72, DP_POWERD_CTRL_REG, 0x10 );	//0x00 //for low power test
	
	//get chip ID. Make sure I2C is OK
	c=DP_TX_Read_Reg(0x72, DP_TX_DEV_IDH_REG);
	if (c==0x98)
		printk("Chip found\n");	

	//for clock detect
	for(i=0;i<50;i++)
	{
		c=DP_TX_Read_Reg(0x70, DP_TX_SYS_CTRL1_REG);
		DP_TX_Write_Reg(0x70, DP_TX_SYS_CTRL1_REG, c);
		c=DP_TX_Read_Reg(0x70, DP_TX_SYS_CTRL1_REG);
		if((c&DP_TX_SYS_CTRL1_DET_STA)!=0)
		{
			printk("clock is detected.\n");
			break;
		}

		msleep(10);
	}
       //check whether clock is stable
	for(i=0;i<50;i++)
	{
		c=DP_TX_Read_Reg(0x70, DP_TX_SYS_CTRL2_REG);
		DP_TX_Write_Reg(0x70, DP_TX_SYS_CTRL2_REG, c);
		c=DP_TX_Read_Reg(0x70, DP_TX_SYS_CTRL2_REG);
		if((c&DP_TX_SYS_CTRL2_CHA_STA)==0)
		{
			printk("clock is stable.\n");
			break;
		}
		msleep(10);
	}

	//VESA range, 8bits BPC, RGB 
	DP_TX_Write_Reg(0x72, DP_TX_VID_CTRL2_REG, 0x10);
	
	//ANX9804 chip analog setting
	DP_TX_Write_Reg(0x70, DP_TX_PLL_CTRL_REG, 0x07); 
	DP_TX_Write_Reg(0x72, DP_TX_PLL_FILTER_CTRL3, 0x19); 
	DP_TX_Write_Reg(0x72, DP_TX_PLL_CTRL3, 0xd9); 
	
	//DP_TX_Write_Reg(0x7a, 0x38, 0x10); 
	//DP_TX_Write_Reg(0x7a, 0x39, 0x20); 
	//DP_TX_Write_Reg(0x7a, 0x65, 0x00); 
	
	//Select AC mode
	DP_TX_Write_Reg(0x72, DP_TX_RST_CTRL2_REG, 0x40); 
	
	//DP_TX_Write_Reg(0x7a, 0x61, 0x10); 
	//DP_TX_Write_Reg(0x7a, 0x62, 0x10); 
	//DP_TX_Write_Reg(0x7a, 0x63, 0x10); 
	//DP_TX_Write_Reg(0x7a, 0x64, 0x10); 

	//ANX9804 chip analog setting
	DP_TX_Write_Reg(0x72, ANALOG_DEBUG_REG1, 0xf0);
	DP_TX_Write_Reg(0x72, ANALOG_DEBUG_REG3, 0x99);
	DP_TX_Write_Reg(0x72, DP_TX_PLL_FILTER_CTRL1, 0x7b);
	DP_TX_Write_Reg(0x70, DP_TX_LINK_DEBUG_REG, 0x30);
	DP_TX_Write_Reg(0x72, DP_TX_PLL_FILTER_CTRL, 0x06);

	//force HPD
	DP_TX_Write_Reg(0x70, DP_TX_SYS_CTRL3_REG, 0x30);
	//power on 4 lanes
	DP_TX_Write_Reg(0x70, 0xc8, 0x00);
	//lanes setting
	DP_TX_Write_Reg(0x70, 0xa3, 0x00);
	DP_TX_Write_Reg(0x70, 0xa4, 0x00);
	DP_TX_Write_Reg(0x70, 0xa5, 0x00);
	DP_TX_Write_Reg(0x70, 0xa6, 0x00);
	DP_TX_Write_Reg(0x70, 0xa7, 0x00);	//add for low power test
#if 0
	//step 1: read DPCD 0x00001, the correct value should be 0x0a, or 0x06
	DP_TX_Write_Reg(0x70,  0xE4,  0x80);

	//set read cmd and count, read 2 bytes data, get downstream max_bandwidth and max_lanes
	DP_TX_Write_Reg(0x70, 0xE5,  0x19);

	//set aux address19:0
	DP_TX_Write_Reg(0x70,  0xE6,  0x01);
	DP_TX_Write_Reg(0x70,  0xE7,  0x00);
	DP_TX_Write_Reg(0x70,  0xE8,  0x00);

	//Enable Aux
	DP_TX_Write_Reg(0x70,  0xE9, 0x01);

	//wait aux finished
	for(i=0; i<50; i++)
	{
	  c=DP_TX_Read_Reg(0x70,  0xE9);
	  if(c==0x00)
	  {
	    break;
	  }
	}

	//read data from buffer
	DP_TX_Write_Reg(  0x70,  0xF0,   &max_bandwidth);
	DP_TX_Write_Reg(  0x70,  0xF1,   &max_lanes);
	printk("max_bandwidth = %.2x, max_lanes = %.2x\n", (WORD)max_bandwidth, (WORD)max_lanes);
#endif

	//reset AUX CH
	DP_TX_Write_Reg(0x72,  DP_TX_RST_CTRL2_REG,  0x44);
	DP_TX_Write_Reg(0x72,  DP_TX_RST_CTRL2_REG,  0x40);

	//Select 2.7G
	DP_TX_Write_Reg(0x70, DP_TX_LINK_BW_SET_REG, 0x0a);
	//Select 4 lanes
	DP_TX_Write_Reg(0x70, DP_TX_LANE_COUNT_SET_REG, 0x04);
	
	//strart link traing
	//DP_TX_LINK_TRAINING_CTRL_EN is self clear. If link training is OK, it will self cleared.
	DP_TX_Write_Reg(0x70, DP_TX_LINK_TRAINING_CTRL_REG, DP_TX_LINK_TRAINING_CTRL_EN);
	msleep(5);
	c=DP_TX_Read_Reg(0x70, DP_TX_LINK_TRAINING_CTRL_REG);
	while((c&0x01)!=0)
	{
		printk("Waiting...\n");
		msleep(5);
		c=DP_TX_Read_Reg(0x70, DP_TX_LINK_TRAINING_CTRL_REG);
	}
	//DP_TX_Write_Reg(0x7a, 0x7c, 0x02);  	

       //BIST MODE: video format. In normal mode, don't need to config these reg from 0x12~0x21
/*	DP_TX_Write_Reg(0x72, 0x12, 0x2c);
	DP_TX_Write_Reg(0x72, 0x13, 0x06);
	DP_TX_Write_Reg(0x72, 0x14, 0x00);
	DP_TX_Write_Reg(0x72, 0x15, 0x06);
	DP_TX_Write_Reg(0x72, 0x16, 0x02);
	DP_TX_Write_Reg(0x72, 0x17, 0x04);
	DP_TX_Write_Reg(0x72, 0x18, 0x26);
	DP_TX_Write_Reg(0x72, 0x19, 0x50);
	DP_TX_Write_Reg(0x72, 0x1a, 0x04);
	DP_TX_Write_Reg(0x72, 0x1b, 0x00);
	DP_TX_Write_Reg(0x72, 0x1c, 0x04);
	DP_TX_Write_Reg(0x72, 0x1d, 0x18);
	DP_TX_Write_Reg(0x72, 0x1e, 0x00);
	DP_TX_Write_Reg(0x72, 0x1f, 0x10);
	DP_TX_Write_Reg(0x72, 0x20, 0x00);
	DP_TX_Write_Reg(0x72, 0x21, 0x28);
*/
	DP_TX_Write_Reg(0x72, 0x11, 0x03);
	
       //enable BIST. In normal mode, don't need to config this reg
//	DP_TX_Write_Reg(0x72, 0x0b, 0x08);	
	
	//enable video input, set SDR mode; 
	DP_TX_Write_Reg(0x72, 0x08, 0x81);	
	
    //force HPD and stream valid
	DP_TX_Write_Reg(0x70, 0x82, 0x33); 
	
	printk("anx9804 init finished.\n");
        
}

static void ANX9804_Initialize_2lane(void)
{
	unsigned char c;
    unsigned int i;
	
	printk("enter ANX9804_Initialize_2lane...\n");	
	//HW reset
	//DP_TX_Write_Reg(0x72, DP_TX_RST_CTRL_REG, DP_TX_RST_HW_RST);
	msleep(10);
	DP_TX_Write_Reg(0x72, DP_TX_RST_CTRL_REG, 0x00);
	//Power on total and select DP mode
       DP_TX_Write_Reg(0x72, DP_POWERD_CTRL_REG, 0x10 );	//0x00 //for low power test
	
	//get chip ID. Make sure I2C is OK
	c=DP_TX_Read_Reg(0x72, DP_TX_DEV_IDH_REG);
	if (c==0x98)
		printk("Chip found\n");	

	//for clock detect
	for(i=0;i<50;i++)
	{
		c=DP_TX_Read_Reg(0x70, DP_TX_SYS_CTRL1_REG);
		DP_TX_Write_Reg(0x70, DP_TX_SYS_CTRL1_REG, c);
		c=DP_TX_Read_Reg(0x70, DP_TX_SYS_CTRL1_REG);
		if((c&DP_TX_SYS_CTRL1_DET_STA)!=0)
		{
			printk("clock is detected.\n");
			break;
		}

		msleep(10);
	}
       //check whether clock is stable
	for(i=0;i<50;i++)
	{
		c=DP_TX_Read_Reg(0x70, DP_TX_SYS_CTRL2_REG);
		DP_TX_Write_Reg(0x70, DP_TX_SYS_CTRL2_REG, c);
		c=DP_TX_Read_Reg(0x70, DP_TX_SYS_CTRL2_REG);
		if((c&DP_TX_SYS_CTRL2_CHA_STA)==0)
		{
			printk("clock is stable.\n");
			break;
		}
		msleep(10);
	}

	//VESA range, 8bits BPC, RGB 
	DP_TX_Write_Reg(0x72, DP_TX_VID_CTRL2_REG, 0x10);
	
	//ANX9804 chip analog setting
	DP_TX_Write_Reg(0x70, DP_TX_PLL_CTRL_REG, 0x07); 
	DP_TX_Write_Reg(0x72, DP_TX_PLL_FILTER_CTRL3, 0x19); 
	DP_TX_Write_Reg(0x72, DP_TX_PLL_CTRL3, 0xd9); 
	
	//DP_TX_Write_Reg(0x7a, 0x38, 0x10); 
	//DP_TX_Write_Reg(0x7a, 0x39, 0x20); 
	//DP_TX_Write_Reg(0x7a, 0x65, 0x00); 
	
	//Select AC mode
	DP_TX_Write_Reg(0x72, DP_TX_RST_CTRL2_REG, 0x40); 
	
	//DP_TX_Write_Reg(0x7a, 0x61, 0x10); 
	//DP_TX_Write_Reg(0x7a, 0x62, 0x10); 
	//DP_TX_Write_Reg(0x7a, 0x63, 0x10); 
	//DP_TX_Write_Reg(0x7a, 0x64, 0x10); 

	//ANX9804 chip analog setting
	DP_TX_Write_Reg(0x72, ANALOG_DEBUG_REG1, 0xf0);
	DP_TX_Write_Reg(0x72, ANALOG_DEBUG_REG3, 0x99);
	DP_TX_Write_Reg(0x72, DP_TX_PLL_FILTER_CTRL1, 0x7b);
	DP_TX_Write_Reg(0x70, DP_TX_LINK_DEBUG_REG, 0x30);
	DP_TX_Write_Reg(0x72, DP_TX_PLL_FILTER_CTRL, 0x06);

	//force HPD
	DP_TX_Write_Reg(0x70, DP_TX_SYS_CTRL3_REG, 0x30);
	//power on 4 lanes
	DP_TX_Write_Reg(0x70, 0xc8, 0x0c);	//0x00	//for 2lanes
	//lanes setting
	DP_TX_Write_Reg(0x70, 0xa3, 0x00);
	DP_TX_Write_Reg(0x70, 0xa4, 0x00);
	DP_TX_Write_Reg(0x70, 0xa5, 0x00);
	DP_TX_Write_Reg(0x70, 0xa6, 0x00);
	DP_TX_Write_Reg(0x70, 0xa7, 0x00);	//add for low power test
#if 0
	//step 1: read DPCD 0x00001, the correct value should be 0x0a, or 0x06
	DP_TX_Write_Reg(0x70,  0xE4,  0x80);

	//set read cmd and count, read 2 bytes data, get downstream max_bandwidth and max_lanes
	DP_TX_Write_Reg(0x70, 0xE5,  0x19);

	//set aux address19:0
	DP_TX_Write_Reg(0x70,  0xE6,  0x01);
	DP_TX_Write_Reg(0x70,  0xE7,  0x00);
	DP_TX_Write_Reg(0x70,  0xE8,  0x00);

	//Enable Aux
	DP_TX_Write_Reg(0x70,  0xE9, 0x01);

	//wait aux finished
	for(i=0; i<50; i++)
	{
	  c=DP_TX_Read_Reg(0x70,  0xE9);
	  if(c==0x00)
	  {
	    break;
	  }
	}

	//read data from buffer
	DP_TX_Write_Reg(  0x70,  0xF0,   &max_bandwidth);
	DP_TX_Write_Reg(  0x70,  0xF1,   &max_lanes);
	printk("max_bandwidth = %.2x, max_lanes = %.2x\n", (WORD)max_bandwidth, (WORD)max_lanes);
#endif

	//reset AUX CH
	DP_TX_Write_Reg(0x72,  DP_TX_RST_CTRL2_REG,  0x44);
	DP_TX_Write_Reg(0x72,  DP_TX_RST_CTRL2_REG,  0x40);

	//Select 2.7G
	DP_TX_Write_Reg(0x70, DP_TX_LINK_BW_SET_REG, 0x0a);
	//Select 4 lanes
	DP_TX_Write_Reg(0x70, DP_TX_LANE_COUNT_SET_REG, 0x02);	//0x04 //for 2lanes
	
	//strart link traing
	//DP_TX_LINK_TRAINING_CTRL_EN is self clear. If link training is OK, it will self cleared.
	DP_TX_Write_Reg(0x70, DP_TX_LINK_TRAINING_CTRL_REG, DP_TX_LINK_TRAINING_CTRL_EN);
	msleep(5);
	c=DP_TX_Read_Reg(0x70, DP_TX_LINK_TRAINING_CTRL_REG);
	while((c&0x01)!=0)
	{
		printk("Waiting...\n");
		msleep(5);
		c=DP_TX_Read_Reg(0x70, DP_TX_LINK_TRAINING_CTRL_REG);
	}
	//DP_TX_Write_Reg(0x7a, 0x7c, 0x02);  	

       //BIST MODE: video format. In normal mode, don't need to config these reg from 0x12~0x21
/*	DP_TX_Write_Reg(0x72, 0x12, 0x2c);
	DP_TX_Write_Reg(0x72, 0x13, 0x06);
	DP_TX_Write_Reg(0x72, 0x14, 0x00);
	DP_TX_Write_Reg(0x72, 0x15, 0x06);
	DP_TX_Write_Reg(0x72, 0x16, 0x02);
	DP_TX_Write_Reg(0x72, 0x17, 0x04);
	DP_TX_Write_Reg(0x72, 0x18, 0x26);
	DP_TX_Write_Reg(0x72, 0x19, 0x50);
	DP_TX_Write_Reg(0x72, 0x1a, 0x04);
	DP_TX_Write_Reg(0x72, 0x1b, 0x00);
	DP_TX_Write_Reg(0x72, 0x1c, 0x04);
	DP_TX_Write_Reg(0x72, 0x1d, 0x18);
	DP_TX_Write_Reg(0x72, 0x1e, 0x00);
	DP_TX_Write_Reg(0x72, 0x1f, 0x10);
	DP_TX_Write_Reg(0x72, 0x20, 0x00);
	DP_TX_Write_Reg(0x72, 0x21, 0x28);
*/
	DP_TX_Write_Reg(0x72, 0x11, 0x03);
	
       //enable BIST. In normal mode, don't need to config this reg
//	DP_TX_Write_Reg(0x72, 0x0b, 0x08);	
	
	//enable video input, set SDR mode; 
	DP_TX_Write_Reg(0x72, 0x08, 0x81);	
	
    //force HPD and stream valid
	DP_TX_Write_Reg(0x70, 0x82, 0x33); 
	
	printk("anx9804 init finished.\n");
        
}

static void test_clk(void)
{
	unsigned char val;
	val = DP_TX_Read_Reg(0x70, 0x80);	//clk detect
	printk("clk detect: %2x\n", val);
	
	val = DP_TX_Read_Reg(0x70, 0x81);	//clk change
	printk("clk change: %2x\nwait for 100ms...\n", val);	
	
	DP_TX_Write_Reg(0x70, 0x81, 0x40);
	msleep(100);
	val = DP_TX_Read_Reg(0x70, 0x81);	//clk change
	printk("clk change: %2x\n\n", val);
}

static void test_anx9804(void)
{
	unsigned char val;
	printk("\nenter anx9804 test.\n");
	
	//test_clk();
	
	val = DP_TX_Read_Reg(0x72, 0x23);	//video status
	printk("video status: %2x\n\n", val);
	
	val = DP_TX_Read_Reg(0x72, 0x24);	//total lines low
	printk("total lines low: %2x\n", val);	
	val = DP_TX_Read_Reg(0x72, 0x25);	//total lines high
	printk("total lines high: %2x\n", val);	
	val = DP_TX_Read_Reg(0x72, 0x26);	//active lines low
	printk("active lines low: %2x\n", val);	
	val = DP_TX_Read_Reg(0x72, 0x27);	//active lines high
	printk("active lines high: %2x\n", val);	
	val = DP_TX_Read_Reg(0x72, 0x29);	//vertical sync width
	printk("vsync width: %2x\n", val);	
	val = DP_TX_Read_Reg(0x72, 0x2a);	//vertical back porch
	printk("vertical back porch: %2x\n\n", val);
	
	val = DP_TX_Read_Reg(0x72, 0x2b);	//total pixels low
	printk("total pixels low: %2x\n", val);	
	val = DP_TX_Read_Reg(0x72, 0x2c);	//total pixels high
	printk("total pixels high: %2x\n", val);
	val = DP_TX_Read_Reg(0x72, 0x2d);	//active pixels low
	printk("active pixels low: %2x\n", val);	
	val = DP_TX_Read_Reg(0x72, 0x2e);	//active pixels high
	printk("active pixels high: %2x\n", val);
	val = DP_TX_Read_Reg(0x72, 0x31);	//horizon sync width low
	printk("hsync width low: %2x\n", val);
	val = DP_TX_Read_Reg(0x72, 0x32);	//horizon sync width high
	printk("hsync width high: %2x\n", val);
	val = DP_TX_Read_Reg(0x72, 0x33);	//horizon back porch low
	printk("horizon back porch low: %2x\n", val);
	val = DP_TX_Read_Reg(0x72, 0x34);	//horizon back porch high
	printk("horizon back porch high: %2x\n\n", val);
}

int aml_anx9804_init()
{
	int res =0; 
	printk("enter anx9804 init.\n");
	if ((anx9804_70_client) && (anx9804_72_client))
	{	
		//ANX9804_Initialize_4lane();	
		//msleep(20);
		ANX9804_Initialize_2lane();  
		//test_anx9804();
		res = 0;
	}
	printk("exit anx9804 init.\n");
	return 0;
}
EXPORT_SYMBOL_GPL(aml_anx9804_init);

static int anx9804_70_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int res = 0;
    printk( "anx9804_70_init.i2c probe begin \n");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("%s: functionality check failed\n", __FUNCTION__);
        res = -ENODEV;
    }
    else
    {
     anx9804_70_client = client;
     printk( "anx9804_70_init.i2c probe client get \n");
    }	
    printk( "anx9804_70_init.i2c probe end \n");
    
    return res;
}

static int anx9804_70_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id anx9804_70_id[] = {
    { "anx9804_70", 0 },
    { }
};


static struct i2c_driver anx9804_70_i2c_driver = {
    .probe = anx9804_70_i2c_probe,
    .remove = anx9804_70_i2c_remove,
    .id_table = anx9804_70_id,
    .driver = {
    .owner	= THIS_MODULE,
    .name = "anx9804_70",
    },
};


static int anx9804_72_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int res = 0;
    printk( "anx9804_72_init.i2c probe begin \n");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("%s: functionality check failed\n", __FUNCTION__);
        res = -ENODEV;
    }
    else
    {
     anx9804_72_client = client;
     printk( "anx9804_72_init.i2c probe client get \n");
    }	
    printk( "anx9804_72_init.i2c probe end \n");
    
    return res;
}

static int anx9804_72_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id anx9804_72_id[] = {
    { "anx9804_72", 0 },
    { }
};

static struct i2c_driver anx9804_72_i2c_driver = {
    .probe = anx9804_72_i2c_probe,
    .remove = anx9804_72_i2c_remove,
    .id_table = anx9804_72_id,
    .driver = {
    .owner	= THIS_MODULE,
    .name = "anx9804_72",
    },
};

//****************************
static ssize_t anx9804_ctrl(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{
    unsigned int addr, reg, val, ret;	
	unsigned char buff[3];
	int n=1,i;
	if(buf[0] == 'w'){
		ret = sscanf(buf, "w %x %x %x", &addr, &reg, &val);			
		if (DP_TX_Write_Reg(addr, reg, val) < 0)
			printk("write anx9804 0x%x reg error: 0x%x = 0x%x.\n", addr, reg, val);
		else
			printk("write anx9804 0x%x reg successful: 0x%x = 0x%x.\n", addr, reg, val);
	}
	else{
		ret =  sscanf(buf, "r %x %x %d", &addr, &reg,&n);
		//printk("read %d camera register from 0x%x reg: 0x%x \n", n, addr, reg);
		for(i=0;i<n;i++)
		{			
			val = DP_TX_Read_Reg(addr, reg+i);
			printk("read 0x%x reg 0x%x = 0x%x.\n", addr, reg+i, val);		
						
		}
	}
	
	if (ret != 1 || ret !=2)
		return -EINVAL;
	
	return count;
	//return 0;
}

static ssize_t anx9804_ctrl_init(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{
    unsigned int ret;	

	if(buf[0] == '2'){
		ANX9804_Initialize_2lane();
		printk("anx9804 init for 2 lanes.\n");
	}
	else{
		ANX9804_Initialize_4lane();
		printk("anx9804 init for 4 lanes.\n");
	}	
	
	if (ret != 1 || ret !=2)
		return -EINVAL;
	
	return count;
	//return 0;
}

static struct class_attribute anx9804_ctrl_class_attrs[] = {
    __ATTR(reg,  S_IRUGO | S_IWUSR, NULL,    anx9804_ctrl), 
	__ATTR(init,  S_IRUGO | S_IWUSR, NULL,    anx9804_ctrl_init), 
    __ATTR_NULL
};

static struct class anx9804_ctrl_class = {
    .name = "anx9804",
    .class_attrs = anx9804_ctrl_class_attrs,
};
//****************************

static int anx9804_probe(struct platform_device *pdev){
    int res;
	
	printk("\n\neDP Driver Init.\n\n");

    if (anx9804_70_client)
    {
        res = 0;
    }
    else
    {
        res = i2c_add_driver(&anx9804_70_i2c_driver);
        if (res < 0) {
            printk("add anx9804_70_i2c_driver error\n");
        }
    }

    if (anx9804_72_client)
    {
        res = 0;
    }
    else
    {
        res = i2c_add_driver(&anx9804_72_i2c_driver);
        if (res < 0) {
            printk("add anx9804_72_i2c_driver error\n");
        }
    }    
	aml_anx9804_init();
#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN -1;
	early_suspend.suspend = anx9804_early_suspend;
	early_suspend.resume = anx9804_late_resume;
	early_suspend.param = pdev;
	register_early_suspend(&early_suspend);
#endif
	
	int ret;
	ret = class_register(&anx9804_ctrl_class);
	if(ret){
		printk(" class register anx9804_ctrl_class fail!\n");
	}
	
    return res;
}

static int anx9804_remove(struct platform_device *pdev)
{
  return 0;
}

static int anx9804_suspend(struct platform_device * pdev, pm_message_t mesg)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (early_suspend_flag)
		return 0;
#endif
	return 0;
}

static int anx9804_resume(struct platform_device * pdev)
{
	int res = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (early_suspend_flag)
		return 0;
#endif

	return res;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void anx9804_early_suspend(struct early_suspend *h)
{
	early_suspend_flag = 1;
	anx9804_suspend((struct platform_device *)h->param, PMSG_SUSPEND);
}

static void anx9804_late_resume(struct early_suspend *h)
{
	if (!early_suspend_flag)
		return;
	early_suspend_flag = 0;
	anx9804_resume((struct platform_device *)h->param);
}
#endif

static struct platform_driver anx9804_driver = {
	.probe = anx9804_probe,
  .remove = anx9804_remove,
  .suspend = anx9804_suspend,
  .resume = anx9804_resume,
	.driver = {
		.name = "anx9804",
	},
};

static int __init anx9804_init(void)
{
	int ret = 0;
    printk( "anx9804_init. \n");

    ret = platform_driver_register(&anx9804_driver);
    if (ret != 0) {
        printk(KERN_ERR "failed to register twx module, error %d\n", ret);
        return -ENODEV;
    }

    return ret;
}

static void __exit anx9804_exit(void)
{
    pr_info("anx9804: exit\n");
    platform_driver_unregister(&anx9804_driver);
}
	
	
//arch_initcall(anx9804_init);
module_init(anx9804_init);
module_exit(anx9804_exit);

MODULE_AUTHOR("BESTIDEAR");
MODULE_DESCRIPTION("TTL IN eDP OUT driver for ANX9804");
MODULE_LICENSE("GPL");


