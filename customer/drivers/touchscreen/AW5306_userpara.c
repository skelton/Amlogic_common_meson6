#include "AW5306_Reg.h"
#include "AW5306_Drv.h"
#include <linux/string.h>
#include "AW5306_userpara.h"

#define	POS_PRECISION				64

extern AW5306_UCF	AWTPCfg;
extern STRUCTCALI	AW_Cali;
extern char AW5306_WorkMode;

extern void AW5306_CLB_WriteCfg(void);
extern int I2C_WriteByte(unsigned char addr, unsigned char data);
extern unsigned char I2C_ReadByte(unsigned char addr);
extern unsigned char I2C_ReadXByte( unsigned char *buf, unsigned char addr, unsigned short len);
extern unsigned char AW5306_RAWDATACHK(void);

const STRUCTCALI Default_Cali1 = 
{
	"AWINIC TP CALI",
	//{0x33,0x23,0x22,0x22,0x22,0x22,0x22,0x02,0x22,0x22},       //TXOFFSET
	{0x32,0x32,0x23,0x32,0x33,0x33,0x33,0x03,0x22,0x22},       //TXOFFSET
        //{0x9A,0xA9,0xAA,0xA9,0x9B,0x00},                             //RXOFFSET
        {0x35,0x44,0x55,0x54,0x34,0x00},                             //RXOFFSET
        //{0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c},//TXCAC
        {0x2C,0x2B,0x2B,0x2A,0x2A,0x2C,0x2C,0x2C,0x2C,0x2C,0x2D,0x2D,0x2D,0x2D,0x31,0x2C,0x2C,0x2C,0x2C,0x2C},//TXCAC
        //{0x3d,0x3c,0x3c,0x3c,0x3e,0x3a,0x3a,0x3e,0x3c,0x3b,0x3c,0x3c},//RXCAC
        {0x84,0x84,0x82,0x82,0x80,0x86,0x86,0x80,0x8C,0x82,0x84,0x84},//RXCAC
        //{0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x0e,0x2e,0x2e,0x0e,0x0e,0x0e,0x0e,0x0e},//TXGAIN
        {0x88,0x88,0x88,0x88,0x88,0x68,0x68,0x68,0x68,0x68,0x48,0x48,0x48,0x48,0x28,0x08,0x08,0x08,0x08,0x08},//TXGAIN
	{0,}
};

void AW5306_User_Cfg1(void)			//yikong
{
	unsigned char i;
		
	AWTPCfg.TX_LOCAL = 15;		//TX number of TP
	AWTPCfg.RX_LOCAL = 10;		//RX number of TP
	AWTPCfg.RX_INV_ORDER = 0;	//RX mapping in inverted order

	AWTPCfg.HAVE_KEY_LINE = 0;

	for(i = 0; i < 16; i++)
	{
		AWTPCfg.KeyLineValid[i] = 1;
	}
	
	AWTPCfg.K_X = ((AWTPCfg.MAPPING_MAX_X - 1)*256)/(AWTPCfg.RX_LOCAL*POS_PRECISION - 1);  //192
	AWTPCfg.K_Y = ((AWTPCfg.MAPPING_MAX_Y - 1)*256)/(AWTPCfg.TX_LOCAL*POS_PRECISION - 1);   //195 

	//auto calibration para
	AWTPCfg.GainClbDeltaMin = 550;	// Expected minimum delta for GAIN calibration
	AWTPCfg.GainClbDeltaMax = 650;	// Expected maximum delta for GAIN calibration
	AWTPCfg.OffsetClbExpectedMin = 8300;	// Expected minimum data for OFFSET calibration
	AWTPCfg.OffsetClbExpectedMax = 8500;	// Expected minimum data for OFFSET calibration
	AWTPCfg.RawDataDeviation = 300;	// Maximum deviation in a frame
	AWTPCfg.CacMultiCoef = 8;

	for(i=0;i<AWTPCfg.TX_LOCAL;i++)
	{
		I2C_WriteByte(SA_TX_INDEX0+i,4+i);			//TX REVERT
	}

	I2C_WriteByte(SA_TX_NUM,AWTPCfg.TX_LOCAL);
	I2C_WriteByte(SA_RX_NUM,AWTPCfg.RX_LOCAL);

	AWTPCfg.MULTI_SCANFREQ = 1;

	
    if(1 == AWTPCfg.MULTI_SCANFREQ)
	{
		I2C_WriteByte(SA_SCANFREQ1,5);		//3-5
		I2C_WriteByte(SA_SCANFREQ2,5);		//3-5
		I2C_WriteByte(SA_SCANFREQ3,5);		//3-5
	}
	else
	{
		I2C_WriteByte(SA_SCANFREQ1,10);		//3-5
	}
	//I2C_WriteByte(SA_TCLKDLY,1);
	I2C_WriteByte(SA_RX_START,0);
	I2C_WriteByte(SA_SCANTIM,4);		// set to 32 TX cycles mode

	I2C_WriteByte(SA_PAGE,1);
	I2C_WriteByte(SA_CHAMPCFG,0x2b);	//
	I2C_WriteByte(SA_OSCCFG1,0xab);		// 
	I2C_WriteByte(SA_PAGE,0);

	if (1 == AW_Cali.FirstFlag)
	{
		memcpy(&AW_Cali,&Default_Cali1,sizeof(STRUCTCALI));		//load default cali value
		AW_Cali.FirstFlag = 0;
	}
	AW5306_CLB_WriteCfg();
	
}

void AW5306_User_Init(void)
{
	unsigned char ret;

	ret = 0;
	AW5306_WorkMode = DeltaMode;	//DeltaMode: chip output delta data  RawDataMode: chip output rawdata
	AWTPCfg.MAPPING_MAX_X = 533;//480;	//   320 LCD DISPLAY SOLUTION X
	AWTPCfg.MAPPING_MAX_Y = 800;	//   460 LCD DISPLAY SOLUTION Y  480 + 480 /13

	AWTPCfg.FLYING_TH = 150;	//flying theshold
	AWTPCfg.MOVING_TH = 100;	//moving theshold
	AWTPCfg.MOVING_ACCELER = 50;	//moving ACC
	AWTPCfg.FIRST_CALI = 1;			//calibrate switch, 1 means calibration at first poweron 

	AWTPCfg.DEBUG_SWITCH = 0;		// lib printk switch 

	AWTPCfg.ESD_PROTECT = 0;

	AW5306_User_Cfg1();

	ret = AW5306_RAWDATACHK();
	
}
