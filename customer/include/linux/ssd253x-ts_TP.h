#ifndef __LINUX_SSD253X_TS_H__
#define __LINUX_SSD253X_TS_H__

/**************************************************************
使用前注意通道数，驱动默认使用通道是sense
大于drive否则需要将使用到的DRIVENO与SENSENO调换
此情况包括0x66和0x67寄存器，但不必修改。
***************************************************************/

#define DRIVENO	18
#define SENSENO	24
#define ENABLE_INT		2	// 0->Polling, 1->Interupt, 2->Hybrid
#define EdgeDisable		1	// if Edge Disable, set it to 1, else reset to 0, OR  SSD2533 set 0
#define RunningAverageMode	2	//{0,8},{5,3},{6,2},{7,1}
#define RunningAverageDist	4	// Threshold Between two consecutive points
#define MicroTimeTInterupt	25000000// 100Hz - 10,000,000us

#ifdef CONFIG_MACH_MESON6_G24
#define FINGERNO		10
#else//if CONFIG_MACH_MESON6_G6
#define FINGERNO		5
#endif
#define KeyInfo    1

#define SSD253X_SCREEN_MAX_X    1024    //CONFIG_FB_OSD1_DEFAULT_WIDTH
#define SSD253X_SCREEN_MAX_Y    768     //CONFIG_FB_OSD1_DEFAULT_HEIGHT

#define SSD253x_TOUCH_KEY 
#undef SSD253x_TOUCH_KEY

struct ts_platform_data_ssd253x{
	int irq_no;
	u32 irq_gpio_no;
	u32 reset_gpio_no;
	u32 power_gpio_no;
};



#ifdef SSD253x_TOUCH_KEY
	static uint32_t key_code[4] = {KEY_MENU, KEY_HOMEPAGE, KEY_BACK, KEY_SEARCH };
#endif

/** Use Cutedge **/
#ifndef CONFIG_MACH_MESON6_G24
#define SSD253x_CUT_EDGE   
#endif
//#undef  SSD253x_CUT_EDGE
#ifdef SSD253x_CUT_EDGE
		#define XPOS_MAX (SENSENO -1) *64
		#define YPOS_MAX (DRIVENO -1) *64
#endif

/** Use Simulated key **/
#ifdef SSD253x_SIMULATED_KEY
typedef struct{
	
	unsigned long left_x;
	unsigned long top_y;		
	unsigned long right_x;	
	unsigned long bottom_y;	
}SKey_Info,*pSKey_Info;

SKey_Info SKeys[]={
{820,56,850,86},
{820,164,850,204},
{820,290,850,320},
{820,400,850,430},
};
	 static uint32_t key_code[4] = {KEY_SEARCH, KEY_HOME, KEY_MENU, KEY_BACK }; 
#endif
// SSD2533 Setting
// Touch Panel Example

#endif
