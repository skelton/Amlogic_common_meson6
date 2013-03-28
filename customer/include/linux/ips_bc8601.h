#ifndef __LINUX_LDWZIC_TS_H__
#define __LINUX_LDWZIC_TS_H__

#define LDWZIC_ADDR 0X01 /* OR 0X70, THE DATASHEET didn't say */

//#if defined (CONFIG_IPS_BD7248F_800x480_TOUCHSCREEN)
#ifdef CONFIG_IPS_BD7248F_800x480_TOUCHSCREEN
#define SCREEN_MAX_X    800
#define SCREEN_MAX_Y    480
#define TP_MIN_X    50
#define TP_MIN_Y    50
#define TP_MAX_X     2650  //2650
#define TP_MAX_Y       1450 //1550
#else
//#elif defined (CONFIG_IPS_BD7248F_1024x600_TOUCHSCREEN)
#define SCREEN_MAX_X    1280
#define SCREEN_MAX_Y    800
#define TP_MIN_X    0
#define TP_MIN_Y    0
#define TP_MAX_X     2700  //2650
#define TP_MAX_Y       1500 //1550
#endif

#define PRESS_MAX       255

#define LDWZIC_NAME	"ldwzic_ts"
/*
struct ldwzic_ts_platform_data{
	u16	intr;		// irq number	
};
*/
enum ldwzic_ts_regs {
	LDWZIC_REG_STATE	= 0x00,	/* BIT[1:0]->00: finger leave; 01: one point, (X1,Y1)valid; 10: onepoint, (X2,Y2)valid; 11: two point,(X1,Y1),(X2,Y2)valid read only*/	
	LDWZIC_REG_POINT1_X1	= 0x01,	/* X1[15:8]	read only	*/	
	LDWZIC_REG_POINT1_X0	= 0x02,	/* X1[7:0]	read only	*/	
	LDWZIC_REG_POINT1_Y1	= 0x03,	/* Y1[15:8]	read only	*/	
	LDWZIC_REG_POINT1_Y0	= 0x04,	/* Y1[7:0]	read only	*/	
	LDWZIC_REG_POINT2_X1	= 0x05,	/* X2[15:8]	read only	*/	
	LDWZIC_REG_POINT2_X0	= 0x06,	/* X2[7:0]	read only	*/	
	LDWZIC_REG_POINT2_Y1	= 0x07,	/* Y2[15:8]	read only	*/	
	LDWZIC_REG_POINT2_Y0	= 0x08,	/* Y2[7:0]	read only	*/	
	LDWZIC_REG_VERTICAL_PIXELS_X1	= 0x09,	/* vertical pixels X[15:8]	read only	*/	
	LDWZIC_REG_VERTICAL_PIXELS_X0	= 0x0A,	/* vertical pixels X[7:0]	read only	*/	
	LDWZIC_REG_VERTICAL_PIXELS_Y1	= 0x0B,	/* vertical pixels Y[15:8]	read only	*/	
	LDWZIC_REG_VERTICAL_PIXELS_Y0	= 0x0C,	/* vertical pixels Y[7:0]	read only	*/	
	LDWZIC_REG_CTRL_OPMODE	= 0x0D,	/* sensor operation Control register	R&W	*/	
};



//-sensor state reg description
#define REG_STATE_MASK          0X03
#define REG_STATE_FINGER_LEAVE  0X00
#define REG_STATE_X1_Y1_VALID   0X01
#define REG_STATE_X2_Y2_VALID   0X02
#define REG_STATE_BOTH_VALID    0X03

//sensor operation control reg description
#define REG_OPMODE_POFF    0x00 /* SENSRO OFF */
#define REG_OPMODE_PON     0x80 /* SENSOR ON , DEFAULT */

#define REG_OPMODE_FULL_RUN_1S_AFTER_NOTOUCH 0X00 /* 1s;  DEFAULT */
#define REG_OPMODE_FULL_RUN_2S_AFTER_NOTOUCH 0X08 /* 2s; */
#define REG_OPMODE_FULL_RUN_3S_AFTER_NOTOUCH 0X10 /* 2s; */
#define REG_OPMODE_FULL_RUN_5S_AFTER_NOTOUCH 0X18 /* 2s; */

#define REG_OPMODE_32MS_DSLEEP_5_ADC     0x00 /* 32MS Deep Sleep +5 ADC wakeup cycles*/
#define REG_OPMODE_64MS_DSLEEP_5_ADC     0x01 /* 32MS Deep Sleep +5 ADC wakeup cycles*/
#define REG_OPMODE_128MS_DSLEEP_5_ADC     0x02 /* 32MS Deep Sleep +5 ADC wakeup cycles  DEFAULT */
#define REG_OPMODE_256MS_DSLEEP_6_ADC     0x03 /* 32MS Deep Sleep +5 ADC wakeup cycles*/
#define REG_OPMODE_512MS_DSLEEP_8_ADC     0x04 /* 32MS Deep Sleep +5 ADC wakeup cycles*/
#define REG_OPMODE_1025MS_DSLEEP_10_ADC     0x05 /* 32MS Deep Sleep +5 ADC wakeup cycles*/
#define REG_OPMODE_2048MS_DSLEEP_20_ADC     0x06 /* 32MS Deep Sleep +5 ADC wakeup cycles*/
#define REG_OPMODE_4096MS_DSLEEP_40_ADC     0x07 /* 32MS Deep Sleep +5 ADC wakeup cycles*/

#define REG_TOUCH_USED_OPMODE (REG_OPMODE_PON | REG_OPMODE_FULL_RUN_1S_AFTER_NOTOUCH|REG_OPMODE_32MS_DSLEEP_5_ADC)

struct ips_platform_data{
	int irq;
	int (*init_irq)(void);
	void (*power)(int on);
	void (*reset)(void);
	int screen_max_x;
	int screen_max_y;
	u8 swap_xy :1;
	u8 xpol :1;
	u8 ypol :1;
};

#endif

