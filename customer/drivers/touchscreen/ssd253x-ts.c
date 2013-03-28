#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <asm/io.h>
#include <mach/gpio.h>
#include <linux/slab.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */

#define CONFIG_TOUCHSCREEN_SSL_DEBUG	
#undef CONFIG_TOUCHSCREEN_SSL_DEBUG

#define DEVICE_ID_REG                    2
#define VERSION_ID_REG                 3
#define AUTO_INIT_RST_REG          68
#define EVENT_STATUS                   121
#define EVENT_MSK_REG                 122
#define IRQ_MSK_REG                     123
#define FINGER01_REG                    124
#define EVENT_STACK                   	 128
#define EVENT_FIFO_SCLR               135
#define TIMESTAMP_REG                 136
#define SELFCAP_STATUS_REG         185		

//#define	ON_TOUCH_INT	INT_GPIO_0 //IRQ_EINT0    //GPIO :set the interrupt 

struct ChipSetting {
	char No;
	char Reg;
	char Data1;
	char Data2;
};

#include <linux/ssd253x-ts_TP.h>

void deviceReset(struct i2c_client *client);
void deviceResume(struct i2c_client *client);
void deviceSuspend(struct i2c_client *client);

void SSD253xdeviceInit(struct i2c_client *client); 


static int ssd253x_ts_suspend(struct i2c_client *client, pm_message_t mesg);
static int ssd253x_ts_resume(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ssd253x_ts_early_suspend(struct early_suspend *h);
static void ssd253x_ts_late_resume(struct early_suspend *h);
#endif /* CONFIG_HAS_EARLYSUSPEND */

static enum hrtimer_restart ssd253x_ts_timer(struct hrtimer *timer);
static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id);
static struct workqueue_struct *ssd253x_wq;

struct ssl_ts_priv {
	struct i2c_client *client;
	struct input_dev *input;
	struct hrtimer timer;
	struct work_struct  ssl_work;
#ifdef	CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif 

	int irq;
	int use_irq;
	int FingerNo;
	int FingerX[FINGERNO];
	int FingerY[FINGERNO];
	int FingerP[FINGERNO];

	int Resolution;
	int EventStatus;
	int FingerDetect;

	int sFingerX[FINGERNO];
	int sFingerY[FINGERNO];
	int pFingerX[FINGERNO];
	int pFingerY[FINGERNO];
};
// SSD2533 Setting
// Touch Panel Example
#ifdef CONFIG_MACH_MESON6_G24
struct ChipSetting ssd253xcfgTable[]={
{1,0x06,0x13,0x00},
{1,0x06,0x13,0x00},
{1,0x07,0x1d,0x00},
{2,0x08,0x00,0xee},
{2,0x09,0x00,0xef},
{2,0x0a,0x00,0xf0},
{2,0x0b,0x00,0xf1},
{2,0x0c,0x00,0xf2},
{2,0x0d,0x00,0xf3},
{2,0x0e,0x00,0xf4},
{2,0x0f,0x00,0xf5},
{2,0x10,0x00,0xf6},
{2,0x11,0x00,0xea},
{2,0x12,0x00,0xe9},
{2,0x13,0x00,0xe8},
{2,0x14,0x00,0xe7},
{2,0x15,0x00,0xe6},
{2,0x16,0x00,0xe5},
{2,0x17,0x00,0xe4},
{2,0x18,0x00,0xe3},
{2,0x19,0x00,0xe2},
{2,0x1a,0x00,0xe1},
{2,0x1b,0x00,0xe0},
{1,0x1f,0x01,0x00},
{1,0xd5,0x05,0x00},
{1,0xd8,0x02,0x00},
{1,0x2a,0x07,0x00},
{1,0x2c,0x01,0x00},
{1,0x2e,0x0b,0x00},
{1,0x2f,0x01,0x00},
{1,0x30,0x08,0x00},
{1,0x31,0x0b,0x00},
{1,0xd7,0x04,0x00},
{1,0xdb,0x01,0x00},
{2,0x33,0x00,0x00},
{2,0x34,0x00,0x40},
{2,0x35,0x00,0x00},
{2,0x36,0x00,0x20},
{1,0x37,0x03,0x00},
{1,0x38,0x00,0x00},
{1,0x39,0x02,0x00},
{1,0x53,0x20,0x00},
{2,0x54,0x00,0x60},
{2,0x55,0x00,0x60},
{1,0x59,0x02,0x00},
{1,0x5b,0x18,0x00},
{1,0x3d,0x00,0x00},
{1,0x40,0xf0,0x00},
{1,0x44,0x01,0x00},
{1,0x8a,0x0a,0x00},
{1,0x8b,0x00,0x00},
{1,0x65,0x01,0x00},
{2,0x66,0x8d,0x34},
{2,0x67,0xa1,0xa5},
{1,0x68,0x00,0x00},
{1,0x69,0x00,0x00},
{2,0x7a,0xff,0xff},
{2,0x7b,0xc0,0x0f},
{1,0x25,0x10,0x00},
};
//#endif
#else //ifdef CONFIG_MACH_MESON6_G24_G6 G6 and G6T can use
struct ChipSetting ssd253xcfgTable[]={
{1,0x06,0x11,0x00},
{1,0x06,0x11,0x00},
{1,0x07,0x17,0x00},
{2,0x08,0x00,0x20},
{2,0x09,0x00,0x81},
{2,0x0a,0x00,0x82},
{2,0x0b,0x00,0x83},
{2,0x0c,0x00,0x84},
{2,0x0d,0x00,0x85},
{2,0x0e,0x00,0x86},
{2,0x0f,0x00,0x87},
{2,0x10,0x00,0x88},
{2,0x11,0x00,0x89},
{2,0x12,0x00,0x8a},
{2,0x13,0x00,0x91},
{2,0x14,0x00,0x90},
{2,0x15,0x00,0x8f},
{2,0x16,0x00,0x8e},
{2,0x17,0x00,0x8d},
{2,0x18,0x00,0x8c},
{2,0x19,0x00,0xeb},
{1,0x1f,0x01,0x00},
{1,0xd5,0x05,0x00},
{1,0xd8,0x02,0x00},
{1,0x2a,0x07,0x00},
{1,0x2c,0x01,0x00},
{1,0x2e,0x0b,0x00},
{1,0x2f,0x01,0x00},
{1,0x30,0x02,0x00},
{1,0x31,0x05,0x00},
{1,0xd7,0x04,0x00},
{1,0xdb,0x02,0x00},
{2,0x33,0x00,0x00},
{2,0x34,0x00,0x5c},
{2,0x35,0x00,0x00},
{2,0x36,0x00,0x30},
{1,0x37,0x02,0x00},
{1,0x39,0x02,0x00},
{1,0x53,0x20,0x00},
{2,0x54,0x00,0x60},
{2,0x55,0x00,0x60},
{1,0x59,0x01,0x00},
{1,0x5b,0x20,0x00},
{1,0x3d,0x00,0x00},
{1,0x40,0xf0,0x00},
{1,0x44,0x01,0x00},
{1,0x8a,0x0a,0x00},
{1,0x8b,0x00,0x00},
{1,0x8c,0xa0,0x00},
{1,0x65,0x00,0x00},
{2,0x66,0xff,0xf0},
{2,0x67,0xff,0xf0},
{2,0x7a,0xff,0xff},
{2,0x7b,0xfe,0x0f},
{1,0x25,0x10,0x00},
};
#endif
// For SSD2533 Bug Version Only //
//#define	SSD2533FIXEDCODE
struct ChipSetting ssd253xcfgTable1[]={
{ 1, 0xA4, 0x00, 0x00},			//MCU prescaler default=01
{ 1, 0xD4, 0x08, 0x00},			//Dummy Code
{ 1, 0xD4, 0x08, 0x00},			//Set Osc frequency default=8, range 0 to F
};

struct ChipSetting Reset[]={
{ 0, 0x04, 0x00, 0x00},	// SSD2533
};

struct ChipSetting Resume[]={
{ 0, 0x04, 0x00, 0x00},	// SSD2533
{ 1, 0x25, 0x0A, 0x00}, // Set Operation Mode   //Set from int setting
};

struct ChipSetting Suspend[] ={
{ 1, 0x25, 0x00, 0x00}, // Set Operation Mode
{ 0, 0x05, 0x00, 0x00},	// SSD2533

};

int ssd253x_record,ssd253x_current,ssd253x_timer_flag;

int ReadRegister(struct i2c_client *client,uint8_t reg,int ByteNo)
{
	unsigned char buf[4];
	struct i2c_msg msg[2];
	int ret;

	memset(buf, 0xFF, sizeof(buf));
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = ByteNo;
	msg[1].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 2);

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	if(ret<0)	printk("		ReadRegister: i2c_transfer Error !\n");
	else		printk("		ReadRegister: i2c_transfer OK !\n");
	#endif

	if(ByteNo==1) return (int)((unsigned int)buf[0]<<0);
	if(ByteNo==2) return (int)((unsigned int)buf[1]<<0)|((unsigned int)buf[0]<<8);
	if(ByteNo==3) return (int)((unsigned int)buf[2]<<0)|((unsigned int)buf[1]<<8)|((unsigned int)buf[0]<<16);
	if(ByteNo==4) return (int)((unsigned int)buf[3]<<0)|((unsigned int)buf[2]<<8)|((unsigned int)buf[1]<<16)|(buf[0]<<24);
	return 0;
}

int WriteRegister(struct i2c_client *client,uint8_t Reg,unsigned char Data1,unsigned char Data2,int ByteNo)
{	
	struct i2c_msg msg;
	unsigned char buf[4];
	int ret;

	buf[0]=Reg;
	buf[1]=Data1;
	buf[2]=Data2;
	buf[3]=0;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = ByteNo+1;
	msg.buf = (char *)buf;
	ret = i2c_transfer(client->adapter, &msg, 1);

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	if(ret<0)	printk("		WriteRegister: i2c_master_send Error !\n");
	else		printk("		WriteRegister: i2c_master_send OK !\n");
	#endif

	return ret;
}

void SSD253xdeviceInit(struct i2c_client *client)
{	
	int i;
	for(i=0;i<sizeof(ssd253xcfgTable)/sizeof(ssd253xcfgTable[0]);i++)
	{
		WriteRegister(	client,ssd253xcfgTable[i].Reg,
				ssd253xcfgTable[i].Data1,ssd253xcfgTable[i].Data2,
				ssd253xcfgTable[i].No);
	}
	mdelay(10);
}

void deviceReset(struct i2c_client *client)
{	
	int i;
	for(i=0;i<sizeof(Reset)/sizeof(Reset[0]);i++)
	{
		WriteRegister(	client,Reset[i].Reg,
				Reset[i].Data1,Reset[i].Data2,
				Reset[i].No);
	}
	mdelay(100);
}

void deviceResume(struct i2c_client *client)    
{	
	#if 1
	struct ts_platform_data_ssd253x *pdata = client->dev.platform_data;
	//RST pin pull down
	gpio_out(pdata->reset_gpio_no, 0);
	mdelay(5);
	gpio_out(pdata->reset_gpio_no, 1);
	mdelay(2);

	deviceReset(client);
	SSD253xdeviceInit(client);
	#else
	int i;
	//int timeout=10;
	int status;
	for(i=0;i<sizeof(Resume)/sizeof(Resume[0]);i++)
	{
		WriteRegister(	client,Resume[i].Reg,
				Resume[i].Data1,Resume[i].Data2,
				Resume[i].No);
		mdelay(200);
	}
	/*

	do {
		status=ReadRegister(client,0x26,1);
		printk("		deviceResume: status : %d !\n",status);
		if(status==Resume[2].Data1) break;
		mdelay(1);
	}while(timeout--); // Check the status
	*/
	#endif
	
}

void deviceSuspend(struct i2c_client *client)
{
	#if 1
	struct ts_platform_data_ssd253x *pdata = client->dev.platform_data;
	//RST pin pull down
	gpio_out(pdata->reset_gpio_no, 0);
	mdelay(5);
	gpio_out(pdata->reset_gpio_no, 1);
	mdelay(2);

	#else
	int i;
	//int timeout=10;
	int status;
	
	/*
	WriteRegister(	client,Suspend[0].Reg,
			Suspend[0].Data1,Suspend[0].Data2,
			Suspend[0].No);
	do {
		status=ReadRegister(client,0x26,1);
		if(status==Suspend[0].Data1) break;
		mdelay(1);				
	}while(timeout--);
	*/
	
	for(i=0;i<sizeof(Suspend)/sizeof(Suspend[0]);i++)
	{
		WriteRegister(	client,Suspend[i].Reg,
				Suspend[i].Data1,Suspend[i].Data2,
				Suspend[i].No);
		mdelay(200);
	}
	#endif
}

#ifdef SSD253x_CUT_EDGE
static int ssd253x_ts_cut_edge(unsigned short pos,unsigned short x_y)
{
#if 0
	u8 cut_value = 10; //cut_value < 32
	if(pos == 0xfff)
	{
		return pos;
	}
	if(x_y) //xpos
	{
		if(pos < 16)
			pos = cut_value + pos*(48 - cut_value) / 16;
		else if(pos > (XPOS_MAX - 16) )
			pos = XPOS_MAX + 16 + (pos - (XPOS_MAX -16))*(48 - cut_value) / 16;
		else
			pos = pos + 32;

		pos = SSD253X_SCREEN_MAX_X * pos / (DRIVENO * 64);
		return pos;
	}
	else    //ypos
	{
		if(pos < 16)
			pos = cut_value + pos*(48 - cut_value) / 16;
		else if(pos > (YPOS_MAX - 16) )
			pos = YPOS_MAX + 16 + (pos - (YPOS_MAX -16))*(48 - cut_value) / 16;
		else
			pos = pos + 32;
		
		pos = SSD253X_SCREEN_MAX_Y* pos / (SENSENO * 64);
		return pos;		
	}
#else
	u8 cut_value = 7;
	if(pos == 0xfff)
	{
		return pos;
	}
	if(x_y) //xpos //xpos 64-->96
	{
		if (pos < 64)
		{
			pos = pos + cut_value + 8;
			pos = SSD253X_SCREEN_MAX_X * pos / (XPOS_MAX + cut_value * 2);
			return pos;
		}
		else
		{
			pos = pos + cut_value;
			pos = SSD253X_SCREEN_MAX_X * pos / (XPOS_MAX + cut_value * 2);
			return pos;
		}
	}
	else
	{
		if (pos < 64)
		{
			pos = pos + cut_value + 8;
			pos = SSD253X_SCREEN_MAX_X * pos / (XPOS_MAX + cut_value * 2);
			return pos;
		}
		else
		{
			pos = pos + cut_value;
			pos = SSD253X_SCREEN_MAX_Y * pos / (YPOS_MAX + cut_value * 2);
			return pos;
		}
	}

#endif
	
}
#endif

static void ssd253x_ts_work(struct work_struct *work)
{
	int i;
	unsigned short xpos=0, ypos=0, width=0;
	int FingerInfo;
	int EventStatus;
	int FingerX[FINGERNO];
	int FingerY[FINGERNO];
	int FingerP[FINGERNO];
	int clrFlag=0;
	int timer_status;
	#ifdef SSD253x_TOUCH_KEY
	u8 btn_status;
	static bool key[4] = { 0, 0, 0, 0 }; 
	#endif
	

	struct ssl_ts_priv *ssl_priv = container_of(work,struct ssl_ts_priv,ssl_work);
	
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_work!                  |\n");
	printk("+-----------------------------------------+\n");
	#endif

	if(!ssd253x_timer_flag)
	{
		timer_status = ReadRegister(ssl_priv->client,TIMESTAMP_REG,2);
		if(!ssd253x_record)                                      
		{
				ssd253x_record = timer_status/1000;   			
		}
		
		ssd253x_current = timer_status/1000;        

		if(ssd253x_current < ssd253x_record)
		{
			ssd253x_current += 0xffff/1000;
		}	
		
		if((ssd253x_current - ssd253x_record) > 10)
		{
		WriteRegister(ssl_priv->client,AUTO_INIT_RST_REG,0x00,0x00,1);
		ssd253x_record = 0;
		ssd253x_timer_flag = 1;
		}
	 }

	#ifdef SSD253x_TOUCH_KEY
		btn_status = ReadRegister(ssl_priv->client,SELFCAP_STATUS_REG, 1);
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		if(KeyInfo<0)	printk("		ssd253x_ts_work: i2c_transfer Error !\n");
		#endif
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("ssd253x_ts_work read 0xB9,KeyInfo is %x\n",KeyInfo);
		#endif
		if(btn_status & 0x0f){
				switch(btn_status & 0x0f){
				case 1:
				//key[0] = 1;	
				key[2] = 1;
				input_event(ssl_priv->input,EV_KEY, key_code[2], 1);
				break;
				case 2:
				key[1] = 1;
				input_event(ssl_priv->input,EV_KEY, key_code[1], 1);
				break;
				
				case 4:
				//key[2] = 1;
				key[0] = 1;
				input_event(ssl_priv->input,EV_KEY, key_code[0], 1);
				break;
				case 8:
				key[3] = 1;
				input_event(ssl_priv->input,EV_KEY, key_code[3], 1);
				break;
				default:
				break;
				}
				hrtimer_start(&ssl_priv->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
			goto work_touch;
			}
			for(i = 0; i < 4; i++)
			{
				if(key[i])
				{
					key[i] = 0;
					input_event(ssl_priv->input, EV_KEY, key_code[i], 0);
				}
			}
			
	work_touch:
	#endif

	EventStatus = ReadRegister(ssl_priv->client,EVENT_STATUS,2)>>4;
	ssl_priv->FingerDetect=0;
	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		if((EventStatus>>i)&0x1)
		{
			FingerInfo=ReadRegister(ssl_priv->client,FINGER01_REG+i,4);
			xpos = ((FingerInfo>>4)&0xF00)|((FingerInfo>>24)&0xFF);
			ypos = ((FingerInfo>>0)&0xF00)|((FingerInfo>>16)&0xFF);
			//width= ((FingerInfo>>4)&0x00F);	

			if(xpos!=0xFFF)
			{
				ssl_priv->FingerDetect++;

				#ifdef SSD253x_CUT_EDGE
				xpos = ssd253x_ts_cut_edge(xpos, 1);
				ypos = ssd253x_ts_cut_edge(ypos, 0);
				#endif
			}
			else 
			{
				// This part is to avoid asyn problem when the finger leaves
				//printk("		ssd253x_ts_work: Correct %x\n",EventStatus);
				EventStatus=EventStatus&~(1<<i);
				clrFlag=1;
			}
		}
		else
		{
			xpos=ypos=0xFFF;
			//width=0;
			clrFlag=1;
		}
		FingerX[i]=xpos;
		FingerY[i]=ypos;
		//FingerP[i]=width;
	}
	if(ssl_priv->use_irq==1) enable_irq(ssl_priv->irq);
	if(ssl_priv->use_irq==2)
	{
		if(ssl_priv->FingerDetect!=0) //enable_irq(ssl_priv->irq);
			hrtimer_start(&ssl_priv->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
	}
	if(clrFlag) WriteRegister(ssl_priv->client,EVENT_FIFO_SCLR,0x01,0x00,1);

	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		xpos=FingerX[i];
		ypos=FingerY[i];
		width=FingerP[i];

		if(xpos!=0xFFF)
		{
			#ifdef SSD253x_SIMULATED_KEY
			if(xpos > SSD253X_SCREEN_MAX_X && xpos>SKeys[0].left_x)
			{
					if(ypos >= SKeys[0].top_y && ypos <= SKeys[0].bottom_y)
					{
					input_event(ssl_priv->input,EV_KEY, key_code[0], 1);
					input_mt_sync(ssl_priv->input);
					goto end;
					}
					
					else if(ypos >= SKeys[1].top_y && ypos <= SKeys[1].bottom_y)
					{
					input_event(ssl_priv->input,EV_KEY, key_code[1], 1);
					input_mt_sync(ssl_priv->input);
					goto end;
					}

					else if(ypos >= SKeys[2].top_y && ypos <= SKeys[2].bottom_y)
					{
					input_event(ssl_priv->input,EV_KEY, key_code[2], 1);
					input_mt_sync(ssl_priv->input);
					goto end;
					}

					else if(ypos >= SKeys[3].top_y && ypos <= SKeys[3].bottom_y)
					{
					input_event(ssl_priv->input,EV_KEY, key_code[3], 1);
					input_mt_sync(ssl_priv->input);
					goto end;
					}

					else
					{
					goto report_key_release;
					}				
			}
			else
			{
			#endif
            input_report_key(ssl_priv->input, BTN_TOUCH,  1);
			input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, i);  
			input_report_abs(ssl_priv->input, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(ssl_priv->input, ABS_MT_POSITION_X, xpos);
			input_report_abs(ssl_priv->input, ABS_MT_POSITION_Y, ypos);
			input_report_abs(ssl_priv->input, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(ssl_priv->input);
			
			#ifdef SSD253x_SIMULATED_KEY
			}
			#endif
			
			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			if(i==0) printk("		ssd253x_ts_work: X = 0x%x , Y = 0x%x, W = 0x%x\n",xpos,ypos,width);
			#endif
		}
		else if(EventStatus == 0)
		{
			input_report_key(ssl_priv->input, BTN_TOUCH,  0);
			input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, i);
			input_report_abs(ssl_priv->input, ABS_MT_POSITION_X, ssl_priv->FingerX[i]);
			input_report_abs(ssl_priv->input, ABS_MT_POSITION_Y, ssl_priv->FingerY[i]);
			input_report_abs(ssl_priv->input, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ssl_priv->input, ABS_MT_WIDTH_MAJOR, 0);
			input_mt_sync(ssl_priv->input);
			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			if(i==0) printk("	release	ssd253x_ts_work: X = 0x%x , Y = 0x%x, W = 0x%x\n",xpos,ypos,width);
			#endif
		}
		ssl_priv->FingerX[i]=FingerX[i];
		ssl_priv->FingerY[i]=FingerY[i];
		ssl_priv->FingerP[i]=width;
	}
	#ifdef SSD253x_SIMULATED_KEY

	report_key_release:
	for(i = 0 ;i<sizeof(key_code)/sizeof(key_code[0]);i++)
	{
		input_event(ssl_priv->input,EV_KEY,key_code[i],0);
	}
	end:
	#endif
	ssl_priv->EventStatus=EventStatus;	
	input_sync(ssl_priv->input);
}

static bool ssd253x_i2c_test(struct i2c_client * client)
{
	int ret, retry;
	//uint8_t test_data[1] = { 0 };	//only write a data address.
	
	for(retry=0; retry < 5; retry++)
	{
		ret =WriteRegister(client,0x04,0x00,0x00,0);	//Test i2c.
		if (ret == 1)
			break;
		msleep(5);
	}
	
	return ret==1 ? true : false;
}

static int ssd253x_ts_probe(struct i2c_client *client,const struct i2c_device_id *idp)
{
	struct ssl_ts_priv *ssl_priv;
	struct input_dev *ssl_input;
	int error;
	int i;

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_probe!                 |\n");
	printk("+-----------------------------------------+\n");
	#endif

	struct ts_platform_data_ssd253x *pdata = client->dev.platform_data;
	
       //gpio_request(pdata->reset_gpio_no, "TS_RESET");
	gpio_out(pdata->reset_gpio_no,0);
	msleep(50);
	gpio_out(pdata->reset_gpio_no,1);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: need I2C_FUNC_I2C\n");
		#endif
		return -ENODEV;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: i2c Check OK!\n");
		printk("		ssd253x_ts_probe: i2c_client name : %s\n",client->name);
		#endif
	}

#if 1
        error = ssd253x_i2c_test(client);
        if(!error){
                printk("!!! ssd253x TP is not exist !!!\n");
                error = -ENODEV;
                goto err0;
        }
        printk("===== ssd253x TP test ok=======\n");
#endif
	ssl_priv = kzalloc(sizeof(*ssl_priv), GFP_KERNEL);
	if (!ssl_priv)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: kzalloc Error!\n");
		#endif
		error=-ENODEV;
		goto	err0;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: kzalloc OK!\n");
		#endif
	}
	dev_set_drvdata(&client->dev, ssl_priv);
	
        //gpio_direction_input(pdata->irq_gpio_no );
        //gpio_enable_edge_int(gpio_to_idx(pdata->irq_gpio_no), 1, pdata->irq_no- INT_GPIO_0);
         gpio_set_status(pdata->irq_gpio_no, gpio_status_in);
         gpio_irq_set(pdata->irq_gpio_no, GPIO_IRQ(pdata->irq_no-INT_GPIO_0, GPIO_IRQ_FALLING));

	ssl_input = input_allocate_device();
	if (!ssl_input)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_allocate_device Error\n");
		#endif
		error=-ENODEV;
		goto	err1;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_allocate_device OK\n");
		#endif
	}

	ssl_input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) | BIT_MASK(EV_SYN) ;
	ssl_input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) | BIT_MASK(BTN_2);
	ssl_input->name = client->name;
	ssl_input->id.bustype = BUS_I2C;
	ssl_input->id.vendor  = 0xABCD;//0x2878; // Modify for Vendor ID
	ssl_input->dev.parent = &client->dev;

	input_set_drvdata(ssl_input, ssl_priv);
	ssl_priv->client = client;
	ssl_priv->input = ssl_input;
	ssl_priv->use_irq = ENABLE_INT;
	ssl_priv->irq = pdata->irq_no;
	ssl_priv->FingerNo=FINGERNO;
	ssl_priv->Resolution=64;

	//deviceReset(client);
	
	WriteRegister(client,0x04,0x00,0x00,0);
	mdelay(200);
	
	printk("SSL Touchscreen I2C Address: 0x%02X\n",client->addr);
	ssl_input->id.product = ReadRegister(client, DEVICE_ID_REG,2);
	ssl_input->id.version = ReadRegister(client,VERSION_ID_REG,3);
	ssl_input->id.product = 0xBEEE;//ReadRegister(client, DEVICE_ID_REG,2);
	ssl_input->id.version = ReadRegister(client,VERSION_ID_REG,2);
	printk("SSL Touchscreen Device ID  : 0x%04X\n",ssl_input->id.product);
	printk("SSL Touchscreen Version ID : 0x%04X\n",ssl_input->id.version);

	SSD253xdeviceInit(client);
	mdelay(150);
	
	WriteRegister(client,EVENT_FIFO_SCLR,0x01,0x00,1); // clear Event FiFo
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("		ssd253X_ts_probe: %04XdeviceInit OK!\n",ssl_input->id.product);
	#endif

	//if(ssl_priv->input->id.product==0x2531)		ssl_priv->Resolution=32;
	//else if(ssl_priv->input->id.product==0x2533)	ssl_priv->Resolution=64;
	//else
	//{
	//	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	//	printk("		ssd253x_ts_probe: ssl_input->id.product Error\n");
	//	#endif
	//	error=-ENODEV;
	//	goto	err1;
	//}

	input_set_abs_params(ssl_input, ABS_MT_TRACKING_ID, 0,16, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_WIDTH_MAJOR, 0, 8, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_POSITION_X,  0,SSD253X_SCREEN_MAX_X + 1, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_POSITION_Y,  0,SSD253X_SCREEN_MAX_Y + 1, 0, 0);

	#ifdef SSD253x_TOUCH_KEY
	set_bit(KEY_MENU, ssl_input->keybit);
	set_bit(KEY_HOME, ssl_input->keybit);
	set_bit(KEY_BACK, ssl_input->keybit);
	set_bit(KEY_SEARCH, ssl_input->keybit);
	#endif

	

	INIT_WORK(&ssl_priv->ssl_work, ssd253x_ts_work);
	error = input_register_device(ssl_input);
	if(error)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_register_device input Error!\n");
		#endif
		error=-ENODEV;
		goto	err1;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_register_device input OK!\n");
		#endif
	}

	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2))
	{
		// Options for different interrupt system 
//		error = request_irq(ssl_priv->irq, ssd253x_ts_isr, IRQF_DISABLED|IRQF_TRIGGER_FALLING, client->name,ssl_priv);
//		error = request_irq(ssl_priv->irq, ssd253x_ts_isr, IRQF_TRIGGER_FALLING, client->name,ssl_priv);
		error = request_irq(ssl_priv->irq, ssd253x_ts_isr, IRQF_DISABLED , client->name,ssl_priv);
		if(error)
		{
			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			printk("		ssd253x_ts_probe: request_irq Error!\n");
			#endif
			error=-ENODEV;
			goto err2;
		}
		else
		{
			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			printk("		ssd253x_ts_probe: request_irq OK!\n");
			#endif
		}	
	}

	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2))
	{
		hrtimer_init(&ssl_priv->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ssl_priv->timer.function = ssd253x_ts_timer;
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: timer_init OK!\n");
		#endif
	}
#ifdef	CONFIG_HAS_EARLYSUSPEND
	ssl_priv->early_suspend.suspend = ssd253x_ts_early_suspend;
	ssl_priv->early_suspend.resume  = ssd253x_ts_late_resume;
	ssl_priv->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN+1;
	register_early_suspend(&ssl_priv->early_suspend);
#endif 
	return 0;

err2:	input_unregister_device(ssl_input);
err1:	input_free_device(ssl_input);
	kfree(ssl_priv);
err0:	dev_set_drvdata(&client->dev, NULL);
	return error;
}

static int ssd253x_ts_resume(struct i2c_client *client)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_resume!                |\n");
	printk("+-----------------------------------------+\n");
	#endif
	deviceResume(client);
	if(ssl_priv->use_irq) enable_irq(ssl_priv->irq);
	else hrtimer_start(&ssl_priv->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	return 0;
}

static int ssd253x_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_suspend!               |\n");
	printk("+-----------------------------------------+\n");
	#endif
	ssd253x_timer_flag = 0;
	ssd253x_record = 0;
	deviceSuspend(client);		
	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2)) hrtimer_cancel(&ssl_priv->timer);
	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2))disable_irq(ssl_priv->client->irq);
	return 0;
}

#ifdef	CONFIG_HAS_EARLYSUSPEND
static void ssd253x_ts_late_resume(struct early_suspend *h)
{
	struct ssl_ts_priv *ssl_priv = container_of(h, struct ssl_ts_priv, early_suspend);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_late_resume!           |\n");
	printk("+-----------------------------------------+\n");
	#endif
	ssd253x_ts_resume(ssl_priv->client);
}
static void ssd253x_ts_early_suspend(struct early_suspend *h)
{
	struct ssl_ts_priv *ssl_priv = container_of(h, struct ssl_ts_priv, early_suspend);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_early_suspend!         |\n");
	printk("+-----------------------------------------+\n");
	#endif
	ssd253x_ts_suspend(ssl_priv->client, PMSG_SUSPEND);
}
#endif

static int ssd253x_ts_remove(struct i2c_client *client)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);	
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_remove !               |\n");
	printk("+-----------------------------------------+\n");
	#endif	
	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2)) hrtimer_cancel(&ssl_priv->timer);
	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2)) free_irq(ssl_priv->irq, ssl_priv);
	input_unregister_device(ssl_priv->input);
	input_free_device(ssl_priv->input);
	kfree(ssl_priv);
	dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id)
{
	struct ssl_ts_priv *ssl_priv = dev_id;
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_isr!                   |\n");
	printk("+-----------------------------------------+\n");
	#endif	
	disable_irq_nosync(ssl_priv->irq);
	queue_work(ssd253x_wq, &ssl_priv->ssl_work);
	if(ssl_priv->use_irq == 2)
		enable_irq(ssl_priv->irq);
	return IRQ_HANDLED;
}

static enum hrtimer_restart ssd253x_ts_timer(struct hrtimer *timer)
{
	struct ssl_ts_priv *ssl_priv = container_of(timer, struct ssl_ts_priv, timer);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_timer!                 |\n");
	printk("+-----------------------------------------+\n");
	#endif
	queue_work(ssd253x_wq, &ssl_priv->ssl_work);
	if(ssl_priv->use_irq==0) hrtimer_start(&ssl_priv->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static const struct i2c_device_id ssd253x_ts_id[] = {
	{ "ssd253x-ts", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ssd253x_ts_id);

static struct i2c_driver ssd253x_ts_driver = {
	.driver = {
		.name = "ssd253x-ts",
	},
	.probe = ssd253x_ts_probe,
	.remove = ssd253x_ts_remove,
#ifndef	CONFIG_HAS_EARLYSUSPEND
	.suspend = ssd253x_ts_suspend,
	.resume = ssd253x_ts_resume,
#endif
	.id_table = ssd253x_ts_id,
};

static char banner[] __initdata = KERN_INFO "SSL Touchscreen driver, (c) 2011 Solomon Systech Ltd.\n";
static int __init ssd253x_ts_init(void)
{
	int ret;
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG	
	printk("+-----------------------------------------+\n");
	printk("|	SSL_ts_init!                      |\n");
	printk("+-----------------------------------------+\n");
	#endif
	printk(banner);	
	ssd253x_wq = create_singlethread_workqueue("ssd253x_wq");
	if (!ssd253x_wq)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_init: create_singlethread_workqueue Error!\n");
		#endif
		return -ENOMEM;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_init: create_singlethread_workqueue OK!\n");
		#endif
	}
	ret=i2c_add_driver(&ssd253x_ts_driver);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	if(ret) printk("		ssd253x_ts_init: i2c_add_driver Error! \n");
	else    printk("		ssd253x_ts_init: i2c_add_driver OK! \n");
	#endif
	return ret;
}

static void __exit ssd253x_ts_exit(void)
{
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_exit!                  |\n");
	printk("+-----------------------------------------+\n");
	#endif
	i2c_del_driver(&ssd253x_ts_driver);
	if (ssd253x_wq) destroy_workqueue(ssd253x_wq);
}

module_init(ssd253x_ts_init);
module_exit(ssd253x_ts_exit);

MODULE_AUTHOR("Solomon Systech Ltd - Design Technology, Icarus Choi");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ssd253x Touchscreen Driver 1.4");
