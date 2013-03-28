///FOR MINILVDS Driver TWX_TC103

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
#if defined (CONFIG_MACH_MESON6_G11_N8) || defined (CONFIG_MACH_MESON6_G11_T883)
#include "t3data-lvds.h"
#else
#include "t3data.h"
#endif
//TTL in  LVDS out

#define BACKLIGHT_BASE_STEP 80
#define BACKLIGHT_MIN_VALUE 10

#define TWX_TC103_I2C_NAME "twx_tc103"
extern void T3_PWMBL_CTR(int vla, int state);
extern int tc103_64pin_init();

//input: 0 关掉背光 1：打开背光
extern void TC103_BLCTROL(int Vale);

static void twx_early_suspend(struct early_suspend *h);
static void twx_late_resume(struct early_suspend *h);
	
static struct i2c_client *twx_tc103_i2c_client;
static struct i2c_client *twx_tc103_bl_i2c_client;

static unsigned char gamaSelect = 0x03;
static unsigned char dlightMode = 2;
static unsigned bl_level = 255;
static unsigned enable3d= 0;

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

void T103_TCON_Control_LVDS_Power(unsigned on_off)
{
    unsigned char tData[2];
	
	tData[0] = 0x06;
	tData[1] = on_off; /*on_off: 0-power on; 1-power off*/
    twx_i2c_write(twx_tc103_i2c_client,tData,2);
    printk("%s: on_off = %d\n",__FUNCTION__,on_off);
}
void T103_TCON_Control_LVDS_Signals(unsigned on_off)
{
    unsigned char tData[2];

	tData[0] = 0x06;
	tData[1] = on_off; /*on_off: 0x02-turn on LVDS; 0x03-turn off LVDS*/
    twx_i2c_write(twx_tc103_i2c_client,tData,2);	
    printk("%s: on_off = %d\n",__FUNCTION__,on_off);
}

static u8 twx_recv(struct i2c_client *i2client,unsigned short addr)

{

   int res = 0;
 	u8 buf[2]= {addr >> 8, addr&0xff };
	u8 data = 0x69;
    struct i2c_msg msgs[] = {
        {
            .addr = i2client->addr,
            .flags = 0,
            .len = 2,
            .buf = buf,
        },
        {
            .addr = i2client->addr,
            .flags = I2C_M_RD,
            .len = 1,
            .buf = &data,
        }
    };
    res = i2c_transfer(i2client->adapter, msgs, 2);
	pr_err("i2c return=%d\n", res);
   if (res < 0) {
        pr_err("%s: i2c transfer failed\n", __FUNCTION__);
   }
   else
		res = data;

    return res;
}


/**
  for 
*/
void initial_dithering_table(struct i2c_client *i2client)
{
	unsigned int i,j;
	unsigned char tAddr, tData[2];
	
	for(i=0;i<0x0c;i++)
	{
		tData[0] =0x00;
                tData[1] = i;	
                twx_i2c_write(i2client,tData,2);				
		for(j=0;j<64;j++)
		{
			tData[0]=0xc0+j;
                        tData[1] = Dith_Table[i][j];	
                        twx_i2c_write(i2client,tData,2);							
		}	
	}	
        tData[0]=0x00;	
	tData[1] = 0x10;
        twx_i2c_write(i2client,tData,2);	        					
};

/*
  for dblight db light addr
*/
void initial_gamma_table(struct i2c_client *client,char vale)
{
	unsigned int i,j;
	unsigned char tAddr, tData[3];       
   
	printk( "enter initial_gamma_table  gamaSelect=%d,vale=%d\n",gamaSelect,vale);  
	if(vale)
	{
		
            if(gamaSelect != 0)
            {
              for(i=0;i<0x04;i++)
		{
			tData[0]=0x4f;
                        tData[1] = i;	
                        twx_i2c_write(client,tData,2);                        			
			for(j=0;j<64;j++)
			{
				tData[0]=0xc0+j;
                                tData[1] = DBL_BANK_GAMMA_TABLE_DBL[i][j];		
				twx_i2c_write(client,tData,2);    					
			}	
		}
               }
               gamaSelect =0;
	}
	else 
	{
	       if(gamaSelect != 1)
              {
               for(i=0;i<0x04;i++)
		{
			tData[0] = 0x4f;	
                        tData[1] = i;	
			twx_i2c_write(client,tData,2);			
			for(j=0;j<64;j++)
			{
				tData[0] =0xc0+j;
                                tData[1] = DBL_BANK_GAMMA_TABLE[i][j];		
				twx_i2c_write(client,tData,2);					
			}	
		}	
               }	
                gamaSelect = 1;
	}	
         			
	tData[1] = 0x04;
        tData[0] = 0x4f;
	twx_i2c_write(client,tData,2);	
        msleep(200);				
};


void IIC_Read_forT101(unsigned char addr, unsigned char reg, unsigned char *buff, int len)
{
   unsigned char tData[3];
   tData[0]=reg;
   tData[1]=buff[0];
   if(addr == 0x6e)
   {
     twx_i2c_read(twx_tc103_i2c_client,tData, 1);
   }
    else
   {
     twx_i2c_read(twx_tc103_bl_i2c_client,tData, 1);
    }
}

void IIC_Write_forT101(unsigned char addr, unsigned char reg, unsigned char *buff, int len)
{
   unsigned char tData[3];
   tData[0]=reg;
   tData[1]=buff[0];
   if(addr == 0x6e)
   {
     twx_i2c_write(twx_tc103_i2c_client,tData, 2);
   }
    else
   {
     twx_i2c_write(twx_tc103_bl_i2c_client,tData, 2);
    }
}
//input pin:  0:com0 1:com1 2:seg0 3:seg1 4:seg2 5:seg3 6:seg4 7:seg5
//	vale: 0:关闭 1:正相打开 2:反相打开
//特别说明：com0 没有反相打开。
//output:无
void COMSEGSet(int Pin,int vale)
{
	unsigned char tData[2];
	switch(Pin)
	{
		case 0:
			if(vale==0)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x98, tData, 1);
				tData[0] |= 0x08;
				IIC_Write_forT101(T103ADDR_TCON, 0x98, tData, 1);					
			}
			else if(vale==1)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x98, tData, 1);
				tData[0] &= 0xf7;
				IIC_Write_forT101(T103ADDR_TCON, 0x98, tData, 1);				
			}
			else if(vale==2)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x98, tData, 1);
				tData[0] &= 0xf7;
				IIC_Write_forT101(T103ADDR_TCON, 0x98, tData, 1);					
			}
			else
				;
			break;
		case 1:
			if(vale==0)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x97, tData, 1);
				tData[0] |= 0x08;
				IIC_Write_forT101(T103ADDR_TCON, 0x97, tData, 1);					
			}
			else if(vale==1)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x97, tData, 1);
				tData[0] &= 0xf7;
				IIC_Write_forT101(T103ADDR_TCON, 0x97, tData, 1);
				
				IIC_Read_forT101(T103ADDR_TCON, 0x2e, tData, 1);
				tData[0] &= 0xdf;
				IIC_Write_forT101(T103ADDR_TCON, 0x2e, tData, 1);				
			}
			else if(vale==2)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x97, tData, 1);
				tData[0] &= 0xf7;
				IIC_Write_forT101(T103ADDR_TCON, 0x97, tData, 1);
				
				IIC_Read_forT101(T103ADDR_TCON, 0x2e, tData, 1);
				tData[0] |= 0x20;
				IIC_Write_forT101(T103ADDR_TCON, 0x2e, tData, 1);										
			}
			else
				;
			break;
		case 2:
			if(vale==0)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x9A, tData, 1);
				tData[0] |= 0x08;
				IIC_Write_forT101(T103ADDR_TCON, 0x9A, tData, 1);					
			}
			else if(vale==1)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x9A, tData, 1);
				tData[0] &= 0xf7;
				IIC_Write_forT101(T103ADDR_TCON, 0x9A, tData, 1);
				
				IIC_Read_forT101(T103ADDR_TCON, 0x2F, tData, 1);
				tData[0] &= 0xf7;
				IIC_Write_forT101(T103ADDR_TCON, 0x2F, tData, 1);				
			}
			else if(vale==2)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x9A, tData, 1);
				tData[0] &= 0xf7;
				IIC_Write_forT101(T103ADDR_TCON, 0x9A, tData, 1);
				
				IIC_Read_forT101(T103ADDR_TCON, 0x2F, tData, 1);
				tData[0] |= 0x08;
				IIC_Write_forT101(T103ADDR_TCON, 0x2F, tData, 1);										
			}
			else
				;			
			break;
		case 3:
			if(vale==0)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x99, tData, 1);
				tData[0] |= 0x08;
				IIC_Write_forT101(T103ADDR_TCON, 0x99, tData, 1);					
			}
			else if(vale==1)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x99, tData, 1);
				tData[0] &= 0xf7;
				IIC_Write_forT101(T103ADDR_TCON, 0x99, tData, 1);
				
				IIC_Read_forT101(T103ADDR_TCON, 0x2e, tData, 1);
				tData[0] &= 0xbf;
				IIC_Write_forT101(T103ADDR_TCON, 0x2e, tData, 1);				
			}
			else if(vale==2)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x99, tData, 1);
				tData[0] &= 0xf7;
				IIC_Write_forT101(T103ADDR_TCON, 0x99, tData, 1);
				
				IIC_Read_forT101(T103ADDR_TCON, 0x2e, tData, 1);
				tData[0] |= 0x40;
				IIC_Write_forT101(T103ADDR_TCON, 0x2e, tData, 1);										
			}
			else
				;				
			break;
		case 4:
			if(vale==0)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x98, tData, 1);
				tData[0] |= 0x80;
				IIC_Write_forT101(T103ADDR_TCON, 0x98, tData, 1);					
			}
			else if(vale==1)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x98, tData, 1);
				tData[0] &= 0x7f;
				IIC_Write_forT101(T103ADDR_TCON, 0x98, tData, 1);
				
				IIC_Read_forT101(T103ADDR_TCON, 0x2f, tData, 1);
				tData[0] &= 0xfb;
				IIC_Write_forT101(T103ADDR_TCON, 0x2f, tData, 1);				
			}
			else if(vale==2)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x98, tData, 1);
				tData[0] &= 0x7f;
				IIC_Write_forT101(T103ADDR_TCON, 0x98, tData, 1);
				
				IIC_Read_forT101(T103ADDR_TCON, 0x2f, tData, 1);
				tData[0] |= 0x04;
				IIC_Write_forT101(T103ADDR_TCON, 0x2f, tData, 1);										
			}
			else
				;		
			break;
		case 5:
			if(vale==0)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x94, tData, 1);
				tData[0] |= 0x08;
				IIC_Write_forT101(T103ADDR_TCON, 0x94, tData, 1);					
			}
			else if(vale==1)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x94, tData, 1);
				tData[0] &= 0xf7;
				IIC_Write_forT101(T103ADDR_TCON, 0x94, tData, 1);
				
				IIC_Read_forT101(T103ADDR_TCON, 0x2f, tData, 1);
				tData[0] &= 0xbf;
				IIC_Write_forT101(T103ADDR_TCON, 0x2f, tData, 1);				
			}
			else if(vale==2)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x94, tData, 1);
				tData[0] &= 0xf7;
				IIC_Write_forT101(T103ADDR_TCON, 0x94, tData, 1);
				
				IIC_Read_forT101(T103ADDR_TCON, 0x2f, tData, 1);
				tData[0] |= 0x40;
				IIC_Write_forT101(T103ADDR_TCON, 0x2f, tData, 1);										
			}
			else
				;		
			break;
		case 6:
			if(vale==0)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x99, tData, 1);
				tData[0] |= 0x80;
				IIC_Write_forT101(T103ADDR_TCON, 0x99, tData, 1);					
			}
			else if(vale==1)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x99, tData, 1);
				tData[0] &= 0x7f;
				IIC_Write_forT101(T103ADDR_TCON, 0x99, tData, 1);
				
				IIC_Read_forT101(T103ADDR_TCON, 0x2e, tData, 1);
				tData[0] &= 0x7f;
				IIC_Write_forT101(T103ADDR_TCON, 0x2e, tData, 1);				
			}
			else if(vale==2)
			{
				IIC_Read_forT101(T103ADDR_TCON, 0x99, tData, 1);
				tData[0] &= 0x7f;
				IIC_Write_forT101(T103ADDR_TCON, 0x99, tData, 1);
				
				IIC_Read_forT101(T103ADDR_TCON, 0x2e, tData, 1);
				tData[0] |= 0x80;
				IIC_Write_forT101(T103ADDR_TCON, 0x2e, tData, 1);										
			}
			else
				;			
			break;
			
		default:break;	
	}
}



//input: 当mode = 0;为一组光珊模式，vale选择用哪一组seg信号；
//	   mode = 1;为多组光珊的竖屏模式，只有一种状态，所以vale作开关用；
//	   mode = 2;为多组光珊的横屏模式，vale值范围：1到5；5种状态可选;
//vale = 0,为关闭光珊，其它做别的选择光珊;
//output:无
void T2_Panel3DLense(int mode,int vale)
{
	unsigned char tData[2];
	if(mode==0)
	{
		if(vale==0)
		{
			tData[0] = 0x40;
			IIC_Write_forT101(T103ADDR_TCON, 0x2e, tData, 1);
			tData[0] = 0x00;
			IIC_Write_forT101(T103ADDR_TCON, 0x2f, tData, 1);
		}
		else
		{
		    tData[0] = 0x80;
			IIC_Write_forT101(T103ADDR_TCON, 0x2f, tData, 1);
			tData[0] = 0x60;
			IIC_Write_forT101(T103ADDR_TCON, 0x2e, tData, 1);
		}
               enable3d=vale;
               T3_PWMBL_CTR(bl_level, 1);
               
	}
}

//input: 0 关掉背光 1：打开背光
void TC103_BLCTROL(int Vale)
{
	unsigned char tData[3];
        tData[0]=0x40;
        tData[1]=0;
        twx_i2c_read(twx_tc103_bl_i2c_client,tData, 2);
	if(Vale==0)
	{
		tData[1] = tData[0]|0x02;	
	}
	else
	{
		tData[1] = tData[0]&0x0fd;		
	}
        tData[0]=0x40;		
        twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);
};

//交织
void TC103_Panel3DInter(int flag)
{
	unsigned char tAddr, tData[2];
	tData[1] = 0x0;
        tData[0]=0x1b;
	if(0 == flag)
	{
		tData[1] = 0x00;
	}
	else
	{		
		tData[1] = 0x03;
	}		
        twx_i2c_write(twx_tc103_i2c_client,tData,2);
}


//左右眼设置
void TC103_Panel3DSwitchLRPic(int flag)
{
	unsigned char tAddr, tData[3];
	tData[2] = 0x0;
        tData[0]=0x48;
	if(0 == flag)
	{
                tData[1] = 0x03;
	}
	else
	{		
		tData[1] = 0x02;
		
	}
        twx_i2c_write(twx_tc103_i2c_client,tData,2);
}


//动态背光
//input: 0:关 1: 艳丽 2： 柔和 3: 强省电
void TC103_DBL_CTR(int vla)
{
	unsigned char tData[2];
        unsigned int TC103_DBL_TBL[4]={0x00,0xc0,0x80,0x60};
	switch(vla)
	{
		case 0:			
#if 1   //关动态背光时，用T3的gamma	                
	                tData[0]=0x01;	
			tData[1] = 0x08;
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);
	                tData[0] = 0x4e;
			tData[1] = 0x00;
#else	//关动态背光时，用主IC的gamma
	                tData[0]=0x01;	
			tData[1] = 0x00;
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);
	                tData[0] = 0x4e;
			tData[1] = 0x01;
#endif			
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);		
			initial_gamma_table(twx_tc103_bl_i2c_client,0);	
                        tData[0] = 0x40;
	                tData[1]=0x01;
	                twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);
			break;			
		case 1:				
			tData[1] = 0x0E;
			tData[0] = 0x01;
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);	
			tData[1] = 0x00;
			tData[0] = 0x4e;	
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);	
				
			tData[1] = TC103_DBL_TBL[1]/256;
			tData[0] = 0x44;
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);		
			tData[1] = TC103_DBL_TBL[1]%256;
			tData[0] = 0x45;
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);	
			initial_gamma_table(twx_tc103_bl_i2c_client,1);	
                        tData[0] = 0x40;
			tData[1] = 0x00;
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);	
			break;
		case 2:				
			tData[1] = 0x0E;
			tData[0] = 0x01;
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);	
			tData[1] = 0x00;
			tData[0] = 0x4e;	
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);	
				
			tData[1] = TC103_DBL_TBL[2]/256;
			tData[0] = 0x44;
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);		
			tData[1] = TC103_DBL_TBL[2]%256;
			tData[0] = 0x45;
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);	
			initial_gamma_table(twx_tc103_bl_i2c_client,1);	
                        tData[0] = 0x40;
			tData[1] = 0x00;
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);	
			break;		
		case 3:					
			tData[1] = 0x0E;
			tData[0] = 0x01;
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);	
			tData[1] = 0x00;
			tData[0] = 0x4e;	
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);	
				
			tData[1] = TC103_DBL_TBL[3]/256;
			tData[0] = 0x44;
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);		
			tData[1] = TC103_DBL_TBL[3]%256;
			tData[0] = 0x45;
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);	
			initial_gamma_table(twx_tc103_bl_i2c_client,1);	
                        tData[0] = 0x40;
			tData[1] = 0x00;
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);	
			break;					
	}	
}


//input: 0--0xff 
 void T3_PWMBL_CTR(int vla, int state)
{
    //背光强度控制
    unsigned int TC103_PWMBL_TBL[16]={0x08,0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80,0x90,0xa0,0xb0,0xc0,0xd0,0xe8,0xff};
    unsigned char tData[2];
    int blvalue=vla;
	bl_level=vla; 
        if(enable3d==0) 
        {
           blvalue = vla-BACKLIGHT_BASE_STEP;
           if(blvalue < BACKLIGHT_MIN_VALUE) blvalue=BACKLIGHT_MIN_VALUE;
        }
        else
        {
          if(blvalue<BACKLIGHT_BASE_STEP) blvalue=BACKLIGHT_BASE_STEP;
        }

	if (state) {
		if(twx_tc103_bl_i2c_client)
		{
			tData[0]=0x42;
			tData[1] = 0x00;
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);
			tData[0]=0x43;	
			tData[1] = blvalue;//TC103_DBL_TBL[vla];
			twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);
			printk( "tc103 set backlight leve %d\n",blvalue);   
		}
   }
}
EXPORT_SYMBOL_GPL(T3_PWMBL_CTR);


//input: 三行子像素交织。
//	第一行 第二行 第三行
//type=0:RGB	GBR    BRG
//type=1:RGB    BRG    GBR
//type=2:GBR    RGB    BRG
//type=3:GBR    BRG    RGB
//type=4:BRG    RGB    GBR
//type=5:BRG    GBR    RGB
#define RGB 0x00
#define GBR 0x01
#define BRG 0x02
void T2_Panel3DInter3(int type)
{
	unsigned char tAddr[2], tData[2];
	
	IIC_Read_forT101(T103ADDR_TCON, 0x1b, tData, 1);
	tAddr[0] = tData[0];
	tData[0] = 0x00;
	
	switch(type)
	{
		case 0:	//RGB	GBR    BRG
			tAddr[0] &= 0x3f;
			tAddr[0] |= (RGB*64);
			IIC_Write_forT101(T103ADDR_TCON, 0x1b, tAddr, 1);
			tData[0] |= GBR;
			tData[0] |= (BRG*4);
			IIC_Write_forT101(T103ADDR_TCON, 0x1c, tData, 1);
			break;
		case 1:	//RGB    BRG    GBR
			tAddr[0] &= 0x3f;
			tAddr[0] |= (RGB*64);
			IIC_Write_forT101(T103ADDR_TCON, 0x1b, tAddr, 1);
			tData[0] |= BRG;
			tData[0] |= (GBR*4);
			IIC_Write_forT101(T103ADDR_TCON, 0x1c, tData, 1);
			break;	
		case 2: //GBR    RGB    BRG
			tAddr[0] &= 0x3f;
			tAddr[0] |= (GBR*64);
			IIC_Write_forT101(T103ADDR_TCON, 0x1b, tAddr, 1);
			tData[0] |= RGB;
			tData[0] |= (BRG*4);
			IIC_Write_forT101(T103ADDR_TCON, 0x1c, tData, 1);
			break;	
		case 3://GBR    BRG    RGB
			tAddr[0] &= 0x3f;
			tAddr[0] |= (GBR*64);
			IIC_Write_forT101(T103ADDR_TCON, 0x1b, tAddr, 1);
			tData[0] |= BRG;
			tData[0] |= (RGB*4);
			IIC_Write_forT101(T103ADDR_TCON, 0x1c, tData, 1);
			break;
		case 4: //BRG    RGB    GBR
			tAddr[0] &= 0x3f;
			tAddr[0] |= (BRG*64);
			IIC_Write_forT101(T103ADDR_TCON, 0x1b, tAddr, 1);
			tData[0] |= RGB;
			tData[0] |= (GBR*4);
			IIC_Write_forT101(T103ADDR_TCON, 0x1c, tData, 1);
			break;	
		case 5://BRG    GBR    RGB
			tAddr[0] &= 0x3f;
			tAddr[0] |= (BRG*64);
			IIC_Write_forT101(T103ADDR_TCON, 0x1b, tAddr, 1);
			tData[0] |= GBR;
			tData[0] |= (RGB*4);
			IIC_Write_forT101(T103ADDR_TCON, 0x1c, tData, 1);
			break;	
		default:break;	
	}	
}

//奇偶像素选择不同的RGB数据交织
//input: 
//奇像素  偶像素 都一样
//type=0   RGB 选择 RGB
//type=1   RGB 选择 RBG
//type=2   RGB 选择 GRB
//type=3   RGB 选择 GBR
//type=4   RGB 选择 BRG
//type=5   RGB 选择 BGR
void T2_Panel3DInter1(int ODD_type,int EVEN_type)
{
	unsigned char tAddr[2], tData[2];	
	
	IIC_Read_forT101(T103ADDR_TCON, 0x19, tData, 1);
	tAddr[0] = tData[0];
	tData[0] = 0x00;
	switch(ODD_type)
	{
		case 0://RGB
			tData[0] |= 0x02;	//B_odd select B
			IIC_Write_forT101(T103ADDR_TCON, 0x1a, tData, 1);
			tAddr[0] &= 0x0f;
			tAddr[0] |= 0x40;	//G_odd select G
			tAddr[0] |= 0x00;	//R_odd select R
			IIC_Write_forT101(T103ADDR_TCON, 0x19, tAddr, 1);				
		case 1://RBG
			tData[0] |= 0x01;	//B_odd select G
			IIC_Write_forT101(T103ADDR_TCON, 0x1a, tData, 1);
			tAddr[0] &= 0x0f;
			tAddr[0] |= 0x80;	//G_odd select B
			tAddr[0] |= 0x00;	//R_odd select R
			IIC_Write_forT101(T103ADDR_TCON, 0x19, tAddr, 1);		
		case 2://GRB
			tData[0] |= 0x02;	//B_odd select B
			IIC_Write_forT101(T103ADDR_TCON, 0x1a, tData, 1);
			tAddr[0] &= 0x0f;
			tAddr[0] |= 0x00;	//G_odd select R
			tAddr[0] |= 0x10;	//R_odd select G
			IIC_Write_forT101(T103ADDR_TCON, 0x19, tAddr, 1);		
		case 3://GBR
			tData[0] |= 0x00;	//B_odd select R
			IIC_Write_forT101(T103ADDR_TCON, 0x1a, tData, 1);
			tAddr[0] &= 0x0f;
			tAddr[0] |= 0x80;	//G_odd select B
			tAddr[0] |= 0x10;	//R_odd select G
			IIC_Write_forT101(T103ADDR_TCON, 0x19, tAddr, 1);		
		case 4://BRG
			tData[0] |= 0x01;	//B_odd select G
			IIC_Write_forT101(T103ADDR_TCON, 0x1a, tData, 1);
			tAddr[0] &= 0x0f;
			tAddr[0] |= 0x00;	//G_odd select R
			tAddr[0] |= 0x20;	//R_odd select B
			IIC_Write_forT101(T103ADDR_TCON, 0x19, tAddr, 1);		
		case 5://BGR
			tData[0] |= 0x00;	//B_odd select R
			IIC_Write_forT101(T103ADDR_TCON, 0x1a, tData, 1);
			tAddr[0] &= 0x0f;
			tAddr[0] |= 0x40;	//G_odd select G
			tAddr[0] |= 0x20;	//R_odd select B
			IIC_Write_forT101(T103ADDR_TCON, 0x19, tAddr, 1);		
			break;
		default:break;
	}
	IIC_Read_forT101(T103ADDR_TCON, 0x1a, tData, 1);	
	switch(EVEN_type)
	{
		case 0://RGB
			tData[0] |= 0x80;	//B_EVEN select B
			tData[0] |= 0x10;	//G_EVEN select G
			tData[0] |= 0x00;	//R_EVEN select R
			IIC_Write_forT101(T103ADDR_TCON, 0x1a, tData, 1);
				
		case 1://RBG
			tData[0] |= 0x40;	//B_EVEN select G
			tData[0] |= 0x20;	//G_EVEN select B
			tData[0] |= 0x00;	//R_EVEN select R
			IIC_Write_forT101(T103ADDR_TCON, 0x1a, tData, 1);		
		case 2://GRB
			tData[0] |= 0x80;	//B_EVEN select B
			tData[0] |= 0x00;	//G_EVEN select R
			tData[0] |= 0x04;	//R_EVEN select G
			IIC_Write_forT101(T103ADDR_TCON, 0x1a, tData, 1);			
		case 3://GBR
			tData[0] |= 0x00;	//B_EVEN select R
			tData[0] |= 0x20;	//G_EVEN select B
			tData[0] |= 0x04;	//R_EVEN select G
			IIC_Write_forT101(T103ADDR_TCON, 0x1a, tData, 1);		
		case 4://BRG
			tData[0] |= 0x40;	//B_EVEN select G
			tData[0] |= 0x00;	//G_EVEN select R
			tData[0] |= 0x08;	//R_EVEN select B
			IIC_Write_forT101(T103ADDR_TCON, 0x1a, tData, 1);		
		case 5://BGR
			tData[0] |= 0x00;	//B_EVEN select R
			tData[0] |= 0x10;	//G_EVEN select G
			tData[0] |= 0x08;	//R_EVEN select B
			IIC_Write_forT101(T103ADDR_TCON, 0x1a, tData, 1);		
			break;
		default:break;		
	}	
}

//子像素交织
//以四个像素为周期,输出的R0G0B0R1G1B1R2G2B2R3G3B3任意选择输入的R0G0B0R1G1B1R2G2B2R3G3B3
//input:mode=0 为有左右交织时子像素交织，其它为没有左右交织时的子像素交织；
//input:四个像素点的选择的参数
//input:四个像素点都一样，现以一个为参考：
//如果在左右交织时:
//  0：left r0 或 left g0 或 left b0 (r只能选r0,g只能选g0，b只能选b0,以下都一样)
//  1：left r1 或 left g1 或 left b1
//  2：right r0 或 right g0 或 right b0
//  3：right r1 或 right g1 或 right b1
//如果不是左右交织时:
//  0：r0 或 g0 或 b0
//  1：r1 或 g1 或 b1
//  2：r2 或 g2 或 b2
//  3：r3 或 g3 或 b3

//现以一个参数来说明:
// 在左右交织时：
//	ONE=0x1b= 0001 1011 表示第一个像素点的选择：bit0,bit1表示R的选择：11表示选择了right r1 ;
//						    bit2,bit3表示G的选择：10表示选择了right g0 ;
//						    bit4,bit5表示B的选择：01表示选择了left b1 ;
// 在没有左右交织时：
//	ONE=0x1b= 0001 1011 表示第一个像素点的选择：bit0,bit1表示R的选择：11表示选择了r3 ;
//						    bit2,bit3表示G的选择：10表示选择了g2 ;
//						    bit4,bit5表示B的选择：01表示选择了b1 ;


//输入参数设置示例：

//左右像素交织：
//3D左右交织时：mode = 1；ONE = 0x00000000; TWO = 0x00101010; Three = 0x00010101; Four = 0x00111111;
//				0x00		  0x2a		      0x15		 0x3f
//2D交织时：	mode = 0；ONE = 0x00000000; TWO = 0x00010101; Three = 0x00101010; Four = 0x00111111;
//				0x00		  0x15		      0x2a		 0x3f

//两像素的G互换:
//3D左右交织时：mode = 1；ONE = 0x00001000; TWO = 0x00100010; Three = 0x00011101; Four = 0x00110111;
//				0x08		  0x22                0x1d		 0x37
//2D交织时：	mode = 0；ONE = 0x00000100; TWO = 0x00010001; Three = 0x00101110; Four = 0x00111011;
//				0x04		  0x11                0x2e		 0x3b

//两像素的R互换:
//3D左右交织时：mode = 1；ONE = 0x00000010; TWO = 0x00101000; Three = 0x00010111; Four = 0x00111101;
//				0x02		  0x28                0x17		 0x3d
//2D交织时：	mode = 0；ONE = 0x00000001; TWO = 0x00010100; Three = 0x00101011; Four = 0x00111110;
//				0x01		  0x14                0x2b		 0x3E

//两像素的B互换:
//3D左右交织时：mode = 1；ONE = 0x00100000; TWO = 0x00001010; Three = 0x00110101; Four = 0x00011111;
//				0x20		  0x0a                0x35		 0x1f
//2D交织时：	mode = 0；ONE = 0x00010000; TWO = 0x00000101; Three = 0x00111010; Four = 0x00101111;
//				0x10		  0x05                0x3a		 0x2f

void T2_Panel3DInter2(int mode,int ONE,int TWO,int Three,int Four)
{
	unsigned char tAddr, tData[2];
	unsigned char tempData=0;
	
	if(mode==0)  //有左右交织时
	{
		//第一个像素的R选择
		tAddr = ONE&0x03;
		IIC_Read_forT101(T103ADDR_TCON, 0x15, tData, 1);
		tData[0] &= 0xf8;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //r0 select left r0
				break;
			case 1:
				tData[0] |=0x01; //r0 select left r1
				break;
			case 2:
				tData[0] |=0x03; //r0 select right r0				
				break;
			case 3:
				tData[0] |=0x04; //r0 select right r2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x15, tData, 1);
		
		//第一个像素的G选择
		tAddr = (ONE&0x0c)/4;
		IIC_Read_forT101(T103ADDR_TCON, 0x16, tData, 1);
		tData[0] &= 0x8f;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //g0 select left g0
				break;
			case 1:
				tData[0] |=0x10; //g0 select left g1
				break;
			case 2:
				tData[0] |=0x30; //g0 select right g0				
				break;
			case 3:
				tData[0] |=0x40; //g0 select right g2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x16, tData, 1);	
		
		//第一个像素的B选择
		tAddr = (ONE&0x30)/16;
		IIC_Read_forT101(T103ADDR_TCON, 0x18, tData, 1);
		tData[0] &= 0xfc;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //b0 select left b0
				break;
			case 1:
				tData[0] |=0x01; //b0 select left b1
				break;
			case 2:
				tData[0] |=0x03; //b0 select right b0				
				break;
			case 3:
				tData[0] |=0x04; //b0 select right b2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x18, tData, 1);
		
		//第二个像素的R选择
		tAddr = TWO&0x03;
		IIC_Read_forT101(T103ADDR_TCON, 0x15, tData, 1);
		tData[0] &= 0xc7;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //r1 select left r0
				break;
			case 1:
				tData[0] |=0x08; //r1 select left r1
				break;
			case 2:
				tData[0] |=0x18; //r1 select right r0				
				break;
			case 3:
				tData[0] |=0x20; //r1 select right r2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x15, tData, 1);
		
		//第二个像素的G选择
		tAddr = (TWO&0x0c)/4;
		IIC_Read_forT101(T103ADDR_TCON, 0x16, tData, 1);
		tempData = tData[0];
		IIC_Read_forT101(T103ADDR_TCON, 0x17, tData, 1);
		tData[0] &= 0xfc;
		tempData &= 0x7f;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //g1 select left g0
				tempData |= 0x00;
				break;
			case 1:
				tData[0] |=0x00; //g1 select left g1
				tempData |= 0x80;
				break;
			case 2:
				tData[0] |=0x01; //g1 select right g0	
				tempData |= 0x80;			
				break;
			case 3:
				tData[0] |=0x02; //g1 select right g2
				tempData |= 0x00;
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x17, tData, 1);
		tData[0] = tempData;
		IIC_Write_forT101(T103ADDR_TCON, 0x16, tData, 1);	
		
		//第二个像素的B选择
		tAddr = (TWO&0x30)/16;
		IIC_Read_forT101(T103ADDR_TCON, 0x18, tData, 1);
		tData[0] &= 0xc7;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //b1 select left b0
				break;
			case 1:
				tData[0] |=0x08; //b1 select left b1
				break;
			case 2:
				tData[0] |=0x18; //b1 select right b0				
				break;
			case 3:
				tData[0] |=0x20; //b1 select right b2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x18, tData, 1);
		
		//第三个像素的R选择
		tAddr = (Three&0x03);
		IIC_Read_forT101(T103ADDR_TCON, 0x15, tData, 1);
		tempData = tData[0];
		IIC_Read_forT101(T103ADDR_TCON, 0x16, tData, 1);
		tData[0] &= 0xfe;
		tempData &= 0x3f;
		switch(tAddr)
		{
			case 0:
				tData[0] |= 0x00; //r2 select left r0
				tempData |= 0x00;
				break;
			case 1:
				tData[0] |=0x00; //r2 select left r1
				tempData |= 0x40;
				break;
			case 2:
				tData[0] |=0x00; //r2 select right r0	
				tempData |= 0xc0;			
				break;
			case 3:
				tData[0] |=0x01; //r2 select right r2
				tempData |= 0x00;
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x16, tData, 1);
		tData[0] = tempData;
		IIC_Write_forT101(T103ADDR_TCON, 0x15, tData, 1);
		
		//第三个像素的G选择
		tAddr = (Three&0x0c)/4;
		IIC_Read_forT101(T103ADDR_TCON, 0x17, tData, 1);
		tData[0] &= 0xe3;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //g0 select left g0
				break;
			case 1:
				tData[0] |=0x04; //g2 select left g1
				break;
			case 2:
				tData[0] |=0x0c; //g2 select right g0				
				break;
			case 3:
				tData[0] |=0x10; //g2 select right g2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x17, tData, 1);	
		
		//第三个像素的B选择
		tAddr = (Three&0x30)/16;
		IIC_Read_forT101(T103ADDR_TCON, 0x18, tData, 1);
		tempData = tData[0];
		IIC_Read_forT101(T103ADDR_TCON, 0x19, tData, 1);
		tData[0] &= 0xfe;
		tempData &= 0x3f;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //b2 select left b0
				tempData |= 0x00;
				break;
			case 1:
				tData[0] |=0x00; //b2 select left b1
				tempData |= 0x40;
				break;
			case 2:
				tData[0] |=0x00; //b2 select right b0	
				tempData |= 0xc0;			
				break;
			case 3:
				tData[0] |=0x01; //b2 select right b2
				tempData |= 0x00;
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x19, tData, 1);
		tData[0] = tempData;
		IIC_Write_forT101(T103ADDR_TCON, 0x18, tData, 1);	
		
		//第四个像素的R选择
		tAddr = (Four&0x03);
		IIC_Read_forT101(T103ADDR_TCON, 0x16, tData, 1);
		tData[0] &= 0xf1;
		switch(tAddr)
		{
			case 0:
				tData[0] |= 0x00; //r3 select left r0
				tempData |= 0x00;
				break;
			case 1:
				tData[0] |=0x02; //r3 select left r1
				break;
			case 2:
				tData[0] |=0x06; //r3 select right r0				
				break;
			case 3:
				tData[0] |=0x08; //r3 select right r2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x16, tData, 1);
		
		//第四个像素的G选择
		tAddr = (Four&0x0c)/4;
		IIC_Read_forT101(T103ADDR_TCON, 0x17, tData, 1);
		tData[0] &= 0x1f;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //g3 select left g0
				break;
			case 1:
				tData[0] |=0x20; //g3 select left g1
				break;
			case 2:
				tData[0] |=0x60; //g3 select right g0				
				break;
			case 3:
				tData[0] |=0x80; //g3 select right g2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x17, tData, 1);	
		
		//第四个像素的B选择
		tAddr = (Four&0x30)/16;
		IIC_Read_forT101(T103ADDR_TCON, 0x19, tData, 1);
		tData[0] &= 0xf1;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //b3 select left b0
				break;
			case 1:
				tData[0] |=0x02; //b3 select left b1
				break;
			case 2:
				tData[0] |=0x06; //b3 select right b0				
				break;
			case 3:
				tData[0] |=0x08; //b3 select right b2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x19, tData, 1);								
	}
	else 	   //没有左右交织时
	{
		//第一个像素的R选择
		tAddr = ONE&0x03;
		IIC_Read_forT101(T103ADDR_TCON, 0x15, tData, 1);
		tData[0] &= 0xf8;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //r0 select left r0
				break;
			case 1:
				tData[0] |=0x01; //r0 select left r1
				break;
			case 2:
				tData[0] |=0x02; //r0 select right r0				
				break;
			case 3:
				tData[0] |=0x03; //r0 select right r2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x15, tData, 1);
		
		//第一个像素的G选择
		tAddr = (ONE&0x0c)/4;
		IIC_Read_forT101(T103ADDR_TCON, 0x16, tData, 1);
		tData[0] &= 0x8f;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //g0 select left g0
				break;
			case 1:
				tData[0] |=0x10; //g0 select left g1
				break;
			case 2:
				tData[0] |=0x20; //g0 select right g0				
				break;
			case 3:
				tData[0] |=0x30; //g0 select right g2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x16, tData, 1);	
		
		//第一个像素的B选择
		tAddr = (ONE&0x30)/16;
		IIC_Read_forT101(T103ADDR_TCON, 0x18, tData, 1);
		tData[0] &= 0xfc;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //b0 select left b0
				break;
			case 1:
				tData[0] |=0x01; //b0 select left b1
				break;
			case 2:
				tData[0] |=0x02; //b0 select right b0				
				break;
			case 3:
				tData[0] |=0x03; //b0 select right b2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x18, tData, 1);
		
		//第二个像素的R选择
		tAddr = TWO&0x03;
		IIC_Read_forT101(T103ADDR_TCON, 0x15, tData, 1);
		tData[0] &= 0xc7;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //r1 select left r0
				break;
			case 1:
				tData[0] |=0x08; //r1 select left r1
				break;
			case 2:
				tData[0] |=0x10; //r1 select right r0				
				break;
			case 3:
				tData[0] |=0x18; //r1 select right r2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x15, tData, 1);
		
		//第二个像素的G选择
		tAddr = (TWO&0x0c)/4;
		IIC_Read_forT101(T103ADDR_TCON, 0x16, tData, 1);
		tempData = tData[0];
		IIC_Read_forT101(T103ADDR_TCON, 0x17, tData, 1);
		tData[0] &= 0xfc;
		tempData &= 0x7f;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //g1 select left g0
				tempData |= 0x00;
				break;
			case 1:
				tData[0] |=0x00; //g1 select left g1
				tempData |= 0x80;
				break;
			case 2:
				tData[0] |=0x01; //g1 select right g0	
				tempData |= 0x00;			
				break;
			case 3:
				tData[0] |=0x01; //g1 select right g2
				tempData |= 0x80;
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x17, tData, 1);
		tData[0] = tempData;
		IIC_Write_forT101(T103ADDR_TCON, 0x16, tData, 1);	
		
		//第二个像素的B选择
		tAddr = (TWO&0x30)/16;
		IIC_Read_forT101(T103ADDR_TCON, 0x18, tData, 1);
		tData[0] &= 0xc7;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //b1 select left b0
				break;
			case 1:
				tData[0] |=0x08; //b1 select left b1
				break;
			case 2:
				tData[0] |=0x10; //b1 select right b0				
				break;
			case 3:
				tData[0] |=0x18; //b1 select right b2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x18, tData, 1);
		
		//第三个像素的R选择
		tAddr = (Three&0x03);
		IIC_Read_forT101(T103ADDR_TCON, 0x15, tData, 1);
		tempData = tData[0];
		IIC_Read_forT101(T103ADDR_TCON, 0x16, tData, 1);
		tData[0] &= 0xfe;
		tempData &= 0x3f;
		switch(tAddr)
		{
			case 0:
				tData[0] |= 0x00; //r2 select left r0
				tempData |= 0x00;
				break;
			case 1:
				tData[0] |=0x00; //r2 select left r1
				tempData |= 0x40;
				break;
			case 2:
				tData[0] |=0x00; //r2 select right r0	
				tempData |= 0x80;			
				break;
			case 3:
				tData[0] |=0x00; //r2 select right r2
				tempData |= 0xc0;
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x16, tData, 1);
		tData[0] = tempData;
		IIC_Write_forT101(T103ADDR_TCON, 0x15, tData, 1);
		
		//第三个像素的G选择
		tAddr = (Three&0x0c)/4;
		IIC_Read_forT101(T103ADDR_TCON, 0x17, tData, 1);
		tData[0] &= 0xe3;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //g0 select left g0
				break;
			case 1:
				tData[0] |=0x04; //g2 select left g1
				break;
			case 2:
				tData[0] |=0x08; //g2 select right g0				
				break;
			case 3:
				tData[0] |=0x0c; //g2 select right g2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x17, tData, 1);	
		
		//第三个像素的B选择
		tAddr = (Three&0x30)/16;
		IIC_Read_forT101(T103ADDR_TCON, 0x18, tData, 1);
		tempData = tData[0];
		IIC_Read_forT101(T103ADDR_TCON, 0x19, tData, 1);
		tData[0] &= 0xfe;
		tempData &= 0x3f;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //b2 select left b0
				tempData |= 0x00;
				break;
			case 1:
				tData[0] |=0x00; //b2 select left b1
				tempData |= 0x40;
				break;
			case 2:
				tData[0] |=0x00; //b2 select right b0	
				tempData |= 0x80;			
				break;
			case 3:
				tData[0] |=0x01; //b2 select right b2
				tempData |= 0xc0;
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x19, tData, 1);
		tData[0] = tempData;
		IIC_Write_forT101(T103ADDR_TCON, 0x18, tData, 1);	
		
		//第四个像素的R选择
		tAddr = (Four&0x03);
		IIC_Read_forT101(T103ADDR_TCON, 0x16, tData, 1);
		tData[0] &= 0xf1;
		switch(tAddr)
		{
			case 0:
				tData[0] |= 0x00; //r3 select left r0
				tempData |= 0x00;
				break;
			case 1:
				tData[0] |=0x02; //r3 select left r1
				break;
			case 2:
				tData[0] |=0x04; //r3 select right r0				
				break;
			case 3:
				tData[0] |=0x06; //r3 select right r2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x16, tData, 1);
		
		//第四个像素的G选择
		tAddr = (Four&0x0c)/4;
		IIC_Read_forT101(T103ADDR_TCON, 0x17, tData, 1);
		tData[0] &= 0x1f;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //g3 select left g0
				break;
			case 1:
				tData[0] |=0x20; //g3 select left g1
				break;
			case 2:
				tData[0] |=0x40; //g3 select right g0				
				break;
			case 3:
				tData[0] |=0x60; //g3 select right g2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x17, tData, 1);	
		
		//第四个像素的B选择
		tAddr = (Four&0x30)/16;
		IIC_Read_forT101(T103ADDR_TCON, 0x19, tData, 1);
		tData[0] &= 0xf1;
		switch(tAddr)
		{
			case 0:
				tData[0] |=0x00; //b3 select left b0
				break;
			case 1:
				tData[0] |=0x02; //b3 select left b1
				break;
			case 2:
				tData[0] |=0x04; //b3 select right b0				
				break;
			case 3:
				tData[0] |=0x06; //b3 select right b2
				break;
			default:break;	
		}
		IIC_Write_forT101(T103ADDR_TCON, 0x19, tData, 1);		
	}
}


//3D交织开关
//input: 0: 关闭3D交织 1:左右像素交织开 2:左右子像素交织 3：2D子像素交织
void T3_Panel3DInter_EN(int flag)
{
	unsigned char tAddr, tData[2];
	tData[1] = 0x0;
	if(0 == flag)
	{
		tData[0] = 0x00;
		IIC_Write_forT101(T103ADDR_TCON, 0x1b, tData, 1);

	}
	else if(1 == flag)
	{		
		tData[0] = 0x03;
		IIC_Write_forT101(T103ADDR_TCON, 0x1b, tData, 1);
	}	
	else	
	{
		tData[0] = 0x06;
		IIC_Write_forT101(T103ADDR_TCON, 0x1b, tData, 1);	
	}	
}

static void set_backlight_level(unsigned level)
{    
    int set_level;
    printk(KERN_INFO "%s: %d\n", __FUNCTION__, level);	
    if(level > 15) level = 15;
    
    bl_level = level; 
}

static unsigned get_backlight_level(void)
{
    printk(KERN_DEBUG "%s: %d\n", __FUNCTION__, bl_level);
    return bl_level;
}

//input: Frequency为TTL 的 CLK，Hz 为 想得到的光珊COM的频率(比如：50Hz，60Hz)；
//output:无
void SetComFrequency(int Frequency,int Hz)
{
	unsigned char tAddr, tData[2];
	int Fre;
	Fre = 512*Hz;
	Fre = Frequency/Fre;
	tData[0] = Fre/256;
	IIC_Write_forT101(T103ADDR_TCON, 0x31, tData, 1);
	tData[0] = Fre%256;
	IIC_Write_forT101(T103ADDR_TCON, 0x30, tData, 1);			
}


void T103_TCON_Initialize(struct i2c_client *client)
{
	unsigned char i, tData[3];
	
	for(i=0;i<MAX_TC103_LEN_TCON;i++)
	{
		tData[0] = TC103_INIT_TBL_TCON[i][0];
                tData[1] = TC103_INIT_TBL_TCON[i][1];
                twx_i2c_write(client,tData,2);			
	}
	printk( "twx_tc103_init.write data to tc103 \n");
        
}

static int dlight_recovery_speed = 0;
void T103_BL_Initialize(struct i2c_client *client)
{
        unsigned char i, tData[3];

        gamaSelect = 3;	
	for(i=0;i<MAX_TC103_LEN_DBL;i++)
	{
		tData[0] = TC103_INIT_TBL_DBL[i][0];

                tData[1] = TC103_INIT_TBL_DBL[i][1];
		twx_i2c_write(client,tData,2);	
                //msleep(5);		
	}

	//initial_gamma_table(client,0);  
#ifdef CONFIG_MACH_MESON6_G11_E8
     if (dlight_recovery_speed) {      
         msleep(50);
	     tData[0] = 0x47;
         tData[1] = 0x11;
	     twx_i2c_write(client,tData,2);	
		 printk("Tc103 write reg[0x47] 0x11\n");
	 }     
#endif	      
        TC103_DBL_CTR(dlightMode);
        T3_PWMBL_CTR(bl_level, 1);
        printk( "twx_tc103_init. backlight level:%d \n",bl_level);   
}

void T103_BL_Initialize_Test(struct i2c_client *client)
{
	unsigned char TC103_INIT_TBL_DBL[4][2]={{0x00,0x00},{0x01,0x0d},{0x02,0x1f},{0x40,0x02},};
        unsigned char i, tData[3];
        gamaSelect = 3;	
	for(i=0;i<4;i++)
	{
		tData[0] = TC103_INIT_TBL_DBL[i][0];
                tData[1] = TC103_INIT_TBL_DBL[i][1];
		twx_i2c_write(client,tData,2);			
	}
       tData[0]=0x00;
       tData[1]=0x01;
       twx_i2c_write(client,tData,2);
}


int tc103_64pin_init()
{
  int res =0;
  T103_BL_Initialize_Test(twx_tc103_bl_i2c_client);
  if (twx_tc103_i2c_client)
  {
      T103_TCON_Initialize(twx_tc103_i2c_client);
      res = 0;
  }

  if (twx_tc103_bl_i2c_client)
  {
      T103_BL_Initialize(twx_tc103_bl_i2c_client);
      res = 0;
  }
 if (twx_tc103_i2c_client)
  {
    initial_dithering_table(twx_tc103_i2c_client);
  }
  return 0;
}
EXPORT_SYMBOL_GPL(tc103_64pin_init);

static int twx_tc103_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int res = 0;
    printk( "twx_tc103_init.i2c probe begin \n");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("%s: functionality check failed\n", __FUNCTION__);
        res = -ENODEV;
    }
    else
    {
     twx_tc103_i2c_client = client;
     printk( "twx_tc103_init.i2c probe client get \n");
     //T103_TCON_Initialize(client);
    }	
    printk( "twx_tc103_init.i2c probe end \n");
    
    return res;
}

static int twx_tc103_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id twx_tc103_id[] = {
    { "twx_tc103", 0 },
    { }
};


static struct i2c_driver twx_tc103_i2c_driver = {
    .probe = twx_tc103_i2c_probe,
    .remove = twx_tc103_i2c_remove,
    .id_table = twx_tc103_id,
    .driver = {
    .owner	= THIS_MODULE,
    .name = "twx_tc103",
    },
};


static int twx_tc103_bl_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int res = 0;
    printk( "twx_tc103_init.i2c probe begin \n");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("%s: functionality check failed\n", __FUNCTION__);
        res = -ENODEV;
    }
    else
    {
     twx_tc103_bl_i2c_client = client;
     //T103_BL_Initialize(client);
     printk( "twx_tc103_init.i2c probe client get \n");
    }	
    printk( "twx_tc103_init.i2c probe end \n");
    
    return res;
}

static int twx_tc103_bl_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id twx_tc103_bl_id[] = {
    { "twx_tc103_bl", 0 },
    { }
};

static struct i2c_driver twx_tc103_bl_i2c_driver = {
    .probe = twx_tc103_bl_i2c_probe,
    .remove = twx_tc103_bl_i2c_remove,
    .id_table = twx_tc103_bl_id,
    .driver = {
    .owner	= THIS_MODULE,
    .name = "twx_tc103_bl",
    },
};


static int twx_tc103_probe(struct platform_device *pdev){
    int res;
	
	printk("\n\nMINI LVDS Driver Init.\n\n");

    if (twx_tc103_bl_i2c_client)
    {
        res = 0;
    }
    else
    {
        res = i2c_add_driver(&twx_tc103_bl_i2c_driver);
        if (res < 0) {
            printk("add twx_tc103 i2c driver error\n");
        }
    }

    if (twx_tc103_i2c_client)
    {
        res = 0;
    }
    else
    {
        res = i2c_add_driver(&twx_tc103_i2c_driver);
        if (res < 0) {
            printk("add twx_tc103 i2c driver error\n");
        }
    }    

#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN -1;
	early_suspend.suspend = twx_early_suspend;
	early_suspend.resume = twx_late_resume;
	early_suspend.param = pdev;
	register_early_suspend(&early_suspend);
#endif
	//T3_PWMBL_CTR(twx_tc103_bl_client,0x08);
    return res;
}

static int twx_tc103_remove(struct platform_device *pdev)
{
  return 0;
}

static void power_on_lcd(void)
{
    msleep(50);
}

static int twx_tc103_suspend(struct platform_device * pdev, pm_message_t mesg)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (early_suspend_flag)
		return 0;
#endif
	return 0;
}

static int twx_tc103_resume(struct platform_device * pdev)
{
	int res = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (early_suspend_flag)
		return 0;
#endif

	return res;
}

int twx_tc103_reinit(void)
{
	int res = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (early_suspend_flag)
		return 0;
#endif

  if (twx_tc103_i2c_client)
  {
	T103_TCON_Initialize(twx_tc103_i2c_client);
  }
	return res;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void twx_early_suspend(struct early_suspend *h)
{
	early_suspend_flag = 1;
	twx_tc103_suspend((struct platform_device *)h->param, PMSG_SUSPEND);
}

static void twx_late_resume(struct early_suspend *h)
{
	if (!early_suspend_flag)
		return;
	early_suspend_flag = 0;
	twx_tc103_resume((struct platform_device *)h->param);
}
#endif

static ssize_t control_3DInter(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{
    char flag_3d = buf[0] ;    
    if(flag_3d == '1') {
        flag_3d = 1;
        printk("\nLCD 3D Inter on.\n");
        TC103_Panel3DInter(1);
    }else{
        flag_3d = 0;
        TC103_Panel3DInter(0);
        printk("\nLCD 3D Inter off.\n");
    }

	return count;
}

static ssize_t control_3DLense(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{
    char flag_3d = buf[0] ;    
    if(flag_3d == '0') {
        flag_3d = 1;
        printk("\nLCD 3D Lense off.\n");
        T2_Panel3DLense(0,0);
    }else{
        flag_3d = 0;
        T2_Panel3DLense(0,1);
        printk("\nLCD 3D Lense on.\n");
    }

	return count;
}

static ssize_t control_LRSwitch(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{
    char flag_3d = buf[0] ;    
    if(flag_3d == '0') {
        TC103_Panel3DSwitchLRPic(0);
    }else{
        TC103_Panel3DSwitchLRPic(1);
    }

	return count;
}

static ssize_t control_DLightPower(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{
    char flag_3d = buf[0] ;    
    if(flag_3d == '0') {
        //input: 0 关掉背光 1：打开背光
         TC103_BLCTROL(0);
    }else{
        TC103_BLCTROL(1);
    }

	return count;
}

static ssize_t control_DLight(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{
    char flag_3d = buf[0] ;
        
    if(flag_3d == '0') {
       if(dlightMode != 0)
       {
         TC103_BLCTROL(0);
         TC103_DBL_CTR(0); 
         //msleep(500);
         //TC103_BLCTROL(1);   
        } 
       dlightMode=0;
    }else if(flag_3d=='1'){
        if(dlightMode == 0) TC103_BLCTROL(0);
        if(dlightMode != 1) TC103_DBL_CTR(1);  
        //if(dlightMode == 0) {msleep(500);TC103_BLCTROL(1);   }
       dlightMode=1;
    }else if(flag_3d=='2'){
       if(dlightMode == 0) TC103_BLCTROL(0);
       if(dlightMode != 2) TC103_DBL_CTR(2); 
       //if(dlightMode == 0) {msleep(500);TC103_BLCTROL(1);   }   
       dlightMode=2;
    }else if(flag_3d=='3'){
       if(dlightMode == 0) TC103_BLCTROL(0);
       if(dlightMode != 3) TC103_DBL_CTR(3); 
       //if(dlightMode == 0) {msleep(500);TC103_BLCTROL(1);   }  
       dlightMode=3;
    }
    printk("tc103 set dlight to mode:%d",dlightMode);
    return count;
}

static ssize_t control_DLRegWrite(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{
    unsigned char tData[2],ret;
    printk("reg:%x  value:%x \n",tData[0],tData[1]);
    ret=sscanf(buf,"%x %x",&tData[0],&tData[1]);   
    twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);
	return count;
}

static ssize_t control_DLRegRead(struct class *class, 
			struct class_attribute *attr,	const char *buf)
{
    unsigned char tData[2],ret;
    int i;
    printk("---------read dynamic light reg -----------");
    for( i=0;i<256;i++)
    {
      if(i%16 == 0) printk("\n");
      tData[0]=i; 
      twx_i2c_read(twx_tc103_bl_i2c_client,tData,1);
      printk("%.2x\t",tData[0]);      
    }
     printk("\n");
	return 0;
}

static ssize_t control_TCONRegWrite(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{
    unsigned char tData[2],ret;
    
    ret=sscanf(buf,"%x %x",&tData[0],&tData[1]);   
    twx_i2c_write(twx_tc103_i2c_client,tData,2);
	printk("tc103 write reg[0x%2x] val:0x%2x\n", tData[0], tData[1]);
	return count;
}

static ssize_t control_TCONRegRead(struct class *class, 
			struct class_attribute *attr,	const char *buf)
{
    unsigned char tData[2],ret;
    int i;
    printk("---------read tcon reg -----------");
    for(i=0;i<256;i++)
    {
      if(i%16 == 0) printk("\n");
      tData[0]=i; 
      twx_i2c_read(twx_tc103_i2c_client,tData,2);
      printk(" %x ",tData[0]);      
    }
     printk("\n");
	return 0;
}


static ssize_t control_Light(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{

    int value;
    sscanf(buf,"%x",&value);  
    if(value<256) 
      T3_PWMBL_CTR(value, 1);

	return count;
}

static ssize_t control_gamma_table(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{
    char flag_3d = buf[0] ;    
    if(flag_3d == '1') {
        flag_3d = 1;
	printk("\n set control_gamma_table dynamic table (gamaSelect=%d).\n",gamaSelect);
        initial_gamma_table(twx_tc103_bl_i2c_client,flag_3d);
	
    }else{
        flag_3d = 0;
	printk("\n set control_gamma_table static table (gamaSelect=%d).\n",gamaSelect);
        initial_gamma_table(twx_tc103_bl_i2c_client,flag_3d);   
    }

    return count;
}

static ssize_t control_init(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{

    tc103_64pin_init();

	return count;
}

static int is_hdmi_switch=0;
static ssize_t speed_up_dlight_recovery(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{
    char value;
    unsigned char tData[2];
    sscanf(buf,"%x",&value); 
    dlight_recovery_speed = value;
	is_hdmi_switch = value;
    if (dlight_recovery_speed == 0) { 
        tData[0] = 0x47;
        tData[1] = 0x99;
        twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);  
		printk("speed_up_dlight_recovery off\n");
    }
	return count;
}

void dlight_fast_recovery(int on)
{
    unsigned char tData[2];
	dlight_recovery_speed = on;
	if (dlight_recovery_speed == 0 && !is_hdmi_switch) { 
	     tData[0] = 0x47;
         tData[1] = 0x99;
	     twx_i2c_write(twx_tc103_bl_i2c_client,tData,2);	
	}
	printk("dlight fast recovery %s\n", dlight_recovery_speed ? "on" : "off");
}

static struct class_attribute enable3d_class_attrs[] = {
    __ATTR(lense,  S_IRUGO | S_IWUSR, NULL,    control_3DLense), 
    __ATTR(inter, S_IRUGO | S_IWUSR, NULL,    control_3DInter), 
    __ATTR(lr_switch, S_IRUGO | S_IWUSR, NULL,    control_LRSwitch), 
    __ATTR(dlight, S_IRUGO | S_IWUSR, NULL,    control_DLight), 
    __ATTR(dlight_power, S_IRUGO | S_IWUSR, NULL,    control_DLightPower), 
    __ATTR(light, S_IRUGO | S_IWUSR, NULL,    control_Light),
    __ATTR(dlreg, S_IRUGO | S_IWUSR, control_DLRegRead,    control_DLRegWrite),
    __ATTR(tcreg, S_IRUGO | S_IWUSR, control_TCONRegRead,    control_TCONRegWrite),
    __ATTR(tcinit, S_IRUGO | S_IWUSR, NULL,    control_init),
    __ATTR(gtable,  S_IRUGO | S_IWUSR, NULL,    control_gamma_table), 
	__ATTR(dlight_recovery,  S_IRUGO | S_IWUSR, NULL,    speed_up_dlight_recovery), 
    __ATTR_NULL
};

static struct class enable3d_class = {
    .name = "twx3d",
    .class_attrs = enable3d_class_attrs,
};

static struct class_attribute enable3d_class_attrs_pre[] = {
    __ATTR(enable-3d,  S_IRUGO | S_IWUSR, NULL,    control_3DLense),     
    __ATTR_NULL
};

static struct class enable3d_class_pre ={
      .name ="enable3d",
      .class_attrs=enable3d_class_attrs_pre,
};


static struct platform_driver twx_tc103_driver = {
	.probe = twx_tc103_probe,
  .remove = twx_tc103_remove,
  .suspend = twx_tc103_suspend,
  .resume = twx_tc103_resume,
	.driver = {
		.name = "twx",
	},
};

static int __init twx_tc103_init(void)
{
	int ret = 0;
    printk( "twx_tc103_init. \n");

    ret = platform_driver_register(&twx_tc103_driver);
    if (ret != 0) {
        printk(KERN_ERR "failed to register twx module, error %d\n", ret);
        return -ENODEV;
    }

    ret = class_register(&enable3d_class);
    if(ret){
		printk(" class register enable3d_class fail!\n");
	}

    ret = class_register(&enable3d_class_pre);
    if(ret){
        printk(" class register enable3d_class_pre fail!\n");
    }
    return ret;
}

static void __exit twx_tc103_exit(void)
{
    pr_info("twx_tc103: exit\n");
    platform_driver_unregister(&twx_tc103_driver);
}
	
	
//arch_initcall(twx_tc103_init);
module_init(twx_tc103_init);
module_exit(twx_tc103_exit);

MODULE_AUTHOR("BESTIDEAR");
MODULE_DESCRIPTION("TTL IN MINILVDS OUT driver for TWX_TC103");
MODULE_LICENSE("GPL");


