/**************************************************************************
*  AW5306_Drv.c
* 
*  AW5306 Driver code version 1.0
* 
*  Create Date : 2012/06/25
* 
*  Modify Date : 
*
*  Create by   : wuhaijun
* 
**************************************************************************/
#include <linux/time.h>
#include <linux/string.h>
#include "AW5306_Reg.h"
#include "AW5306_Drv.h"
#include "AW5306_userpara.h"
#include <asm/uaccess.h>
#include <linux/file.h>
#include <linux/proc_fs.h>


#define BIGAREA_PEAK_NUM 			28		//big area point num threshold
#define WATER_NEG_TH				-55		//water detection threshold


#define	POS_PRECISION				64		//

#define THDIFF				2560

#define	POSX_MIN			0								//The min x coordinate supported by CTMP
#define	POSY_MIN			0                           	//The min y coordinate supported by CTMP

#define	PITCH_X_LEFT		63
#define	PITCH_X_RIGHT		63

#define	PITCH_Y_UP			63
#define	PITCH_Y_DOWN		63

#define USTHTOUCHSIZE				100
#define CALNOISE_TH             	16
#define MARGCOORNUM 				3		//calculate the coordinate ,3*3
#define	UCTHPREFACTOR				4		//for touch pressure, default value is 7, stand for 127

#define	DOWN_EVENT		    0
#define	UP_EVENT			1
#define	CONTACT_EVENT		2
#define	NO_EVENT			3


#define WAKEUPCOMPEANSATE_KEEPTIME  	40000

#define MIN_ESD_DIFF_VAL			 70

#define MIN_ESD_NEGTIVE 			 -50

#define ESD_FILTER_FRAMES			 10


#define	MIN_DELTA_X		2

#define	MIN_DELTA_Y		2

#define	MIN_DELTA_STEP	1

#define MIN_BLUR_VAL	50

#define SHORT_DISTANCE	65000
#define LONG_DISTANCE	105000

#define RAW_DATA_FILTER


#define BASE_VALUE		8400
#define BASE_TH			70
#define BASE_NORMALTH	50

extern char BaseProcess(void);

#define ABS(X)                  ((X > 0) ? (X) : (-X))

extern void AW_Sleep(unsigned int msec);
extern int I2C_WriteByte(unsigned char addr, unsigned char data);
extern unsigned char I2C_ReadByte(unsigned char addr);
extern unsigned char I2C_ReadXByte( unsigned char *buf, unsigned char addr, unsigned short len);
extern unsigned char I2C_WriteXByte( unsigned char *buf, unsigned char addr, unsigned short len);
extern void AW5306_SOFTOFFSET_CLB(void);


STRUCTBASE		AW_Base;
STRUCTPEAK		AW_Peak;
STRUCTPOINT		AW_LastPoint[MAX_POINT];		//ID ORDER POINT LIST
STRUCTPOINT		AW_LastPointInfo[MAX_POINT];		//ID ORDER POINT LIST not filter point
STRUCTPOINT		AW_LastPoint_Filter[MAX_POINT];
STRUCTFRAME		AW_Frame;
AW5306_UCF		AWTPCfg;
STRUCTCALI		AW_Cali;
char G_BlurEnable;

short PointDistanceX[MAX_POINT];
short PointDistanceY[MAX_POINT];


unsigned short pointDistance[MAX_POINT]={0,};
unsigned char flycnt[MAX_POINT] = {0,};


short	Diff[NUM_TX][NUM_RX];
short	adbDiff[NUM_TX][NUM_RX];
 
const unsigned char ucRoot[] = {
                        0,
                        1,
                        2,
                        3,
                        4,

                        0,1,
                        0,2,
                        0,3,
                        0,4,

                        1,0,
                        1,2,
                        1,3,
                        1,4,

                        2,0,
                        2,1,
                        2,3,
                        2,4,

                        3,0,
                        3,1,
                        3,2,
                        3,4,

                        4,0,
                        4,1,
                        4,2,
                        4,3,

                        0,1,2,
                        0,1,3,
                        0,1,4,

                        0,2,1,
                        0,2,3,
                        0,2,4,

                        0,3,1,
                        0,3,2,
                        0,3,4,

                        0,4,1,
                        0,4,2,
                        0,4,3,

                        1,0,2,
                        1,0,3,
                        1,0,4,

                        1,2,0,
                        1,2,3,
                        1,2,4,

                        1,3,0,
                        1,3,2,
                        1,3,4,

                        1,4,0,
                        1,4,2,
                        1,4,3,

                        2,0,1,
                        2,0,3,
                        2,0,4,

                        2,1,0,
                        2,1,3,
                        2,1,4,

                        2,3,0,
                        2,3,1,
                        2,3,4,

                        2,4,0,
                        2,4,1,
                        2,4,3,

                        3,0,1,
                        3,0,2,
                        3,0,4,

                        3,1,0,
                        3,1,2,
                        3,1,4,

                        3,2,0,
                        3,2,1,
                        3,2,4,

                        3,4,0,
                        3,4,1,
                        3,4,2,

                        4,0,1,
                        4,0,2,
                        4,0,3,

                        4,1,0,
                        4,1,2,
                        4,1,3,

                        4,2,0,
                        4,2,1,
                        4,2,3,

                        4,3,0,
                        4,3,1,
                        4,3,2
                       };


char AW5306_WorkMode = DeltaMode;//DeltaMode;//RawDataMode;

unsigned char 	TouchedCnter = 0;

unsigned char 	Calibration_Flag = 0;

unsigned char	ChargePlugIn = 0;

short	BIGAREA_PEAK_TH = 100;
short	THPEAK = 80;	//suspected touch threshold
short	THGROUP = 90;	// touch threshold
short	THPEAKCOL[NUM_TX] = {0,};
short	THPEAKROW[NUM_RX] = {0,};

static unsigned char PointLastCnt[MAX_POINT];

#define CALI_FILENAME	"data/tpcali"
//#define FLY_BROKEN_LINE_METHORD


unsigned short CalDiff(unsigned short a0, unsigned short a1)
{
	if(a0 > a1)
	{
		return a0-a1;
	}
	else
	{
		return a1-a0;
	}
}

static int nvram_read(char *filename, char *buf, ssize_t len, int offset)
{	
    struct file *fd;
    //ssize_t ret;
    int retLen = -1;
    
    mm_segment_t old_fs = get_fs();
    set_fs(KERNEL_DS);
    
    fd = filp_open(filename, O_RDONLY, 0);
    
    if(IS_ERR(fd)) {
        printk("[AW5306][nvram_read] : failed to open!!\n");
        return -1;
    }
    do{
        if ((fd->f_op == NULL) || (fd->f_op->read == NULL))
    		{
            printk("[AW5306][nvram_read] : file can not be read!!\n");
            break;
    		} 
    		
        if (fd->f_pos != offset) {
            if (fd->f_op->llseek) {
        		    if(fd->f_op->llseek(fd, offset, 0) != offset) {
						printk("[AW5306][nvram_read] : failed to seek!!\n");
					    break;
        		    }
        	  } else {
        		    fd->f_pos = offset;
        	  }
        }    		
        
    		retLen = fd->f_op->read(fd,
    									  buf,
    									  len,
    									  &fd->f_pos);			
    		
    }while(false);
    
    filp_close(fd, NULL);
    
    set_fs(old_fs);
    
    return retLen;
}

static int nvram_write(char *filename, char *buf, ssize_t len, int offset)
{	
    struct file *fd;
    //ssize_t ret;
    int retLen = -1;
        
    mm_segment_t old_fs = get_fs();
    set_fs(KERNEL_DS);
    
    fd = filp_open(filename, O_WRONLY|O_CREAT, 0666);
    
    if(IS_ERR(fd)) {
        printk("[AW5306][nvram_write] : failed to open!!\n");
        return -1;
    }
    do{
        if ((fd->f_op == NULL) || (fd->f_op->write == NULL))
    		{
            printk("[AW5306][nvram_write] : file can not be write!!\n");
            break;
    		} /* End of if */
    		
        if (fd->f_pos != offset) {
            if (fd->f_op->llseek) {
        	    if(fd->f_op->llseek(fd, offset, 0) != offset) {
				    printk("[AW5306][nvram_write] : failed to seek!!\n");
                    break;
                }
            } else {
                fd->f_pos = offset;
            }
        }       		
        
        retLen = fd->f_op->write(fd,
                                 buf,
                                 len,
                                 &fd->f_pos);			
    		
    }while(false);
    
    filp_close(fd, NULL);
    
    set_fs(old_fs);
    
    return retLen;
}



void TP_Calibration(void)
{
	int ret;
	char buf[600];

	if(AWTPCfg.FIRST_CALI == 1)
	{
		ret = nvram_read(CALI_FILENAME,&buf[0],sizeof(AW_Cali),0);

		if((ret == -1) || (ret < sizeof(AW_Cali)))
		{
			ret = AW5306_CLB();
			if(ret == 0)
			{
				AW5306_CLB_GetCfg();
				ret = nvram_write(CALI_FILENAME,(char *)&AW_Cali,sizeof(AW_Cali),0);
				if(ret == -1)
				{
					printk("AW5306 fail to write cali file! \n");
				}
			}
			else
			{
				AW5306_CLB_WriteCfg();
				printk("AW5306 CALI Error ! \n");
			}
		}
		else
		{
			memcpy(&AW_Cali,buf,sizeof(AW_Cali));
			AW5306_CLB_WriteCfg();
			printk("AW5306 read data = %s \n", AW_Cali.fileflag);
			printk("AW5306 read and Write Cali File Success! \n");
		}
	}
	else
	{
		ret = nvram_read(CALI_FILENAME,&buf[0],sizeof(AW_Cali),0);
		
		if((ret == -1) || (ret < sizeof(AW_Cali)))
		{
			AW5306_SOFTOFFSET_CLB();
			ret = nvram_write(CALI_FILENAME,(char *)&AW_Cali,sizeof(AW_Cali),0);
			if(ret == -1)
			{
				printk("AW5306 fail to write cali file! \n");
			}
		}
		else
		{
			memcpy(&AW_Cali,buf,sizeof(AW_Cali));
			AW5306_CLB_WriteCfg();
			printk("AW5306 read data = %s \n", AW_Cali.fileflag);
			printk("AW5306 read and Write Cali File Success! \n");
		}
	}
}


void Cali_Refresh(void)
{
	nvram_write(CALI_FILENAME,(char *)&AW_Cali,sizeof(AW_Cali),0);
}

void TP_Force_Calibration(void)
{
	int ret;
	char buf[100];

	
	ret = AW5306_CLB();
	if(ret == 0)
	{
		AW5306_CLB_GetCfg();
		ret = nvram_write(CALI_FILENAME,(char *)&AW_Cali,sizeof(AW_Cali),0);
		if(ret == -1)
		{
			printk("AW5306 fail to write cali file! \n");
		}
	}
	else
	{
		AW5306_CLB_WriteCfg();
		printk("AW5306 CALI Error ! \n");
	}
		
	
}


void InitRawDataMode(void)
{
	unsigned char i,j;

	I2C_WriteByte(SA_PAGE,0x00);
	I2C_WriteByte(SA_SCANMD,0x00);			//scan off
	I2C_WriteByte(SA_IER,0x00);				//frame int enable
	I2C_WriteByte(SA_CTRL,0x09);			//scan en

//	I2C_WriteByte(SA_PAGE,0x01);			//change page 1
//	I2C_WriteByte(SA_TRACECTRL1,0x11);		//
//	I2C_WriteByte(SA_POSNUMTH,0x01);		//pos num threshold

//	I2C_WriteByte(SA_PAGE,0x00);			//change page 0

	AW_Sleep(10);
	/*
	for(i=0;i<10;i++)
	{
		ret = I2C_ReadByte(SA_ISR);			//read int
		if((ret & 0x01) == 0x01)
		{
			break;
		}
		AW_Sleep(5);
	}
	*/
	if(AWTPCfg.MULTI_SCANFREQ)
	{
		I2C_WriteByte(SA_SCANMD,0x07|0x20);			//start scan
	}
	else
	{
		I2C_WriteByte(SA_SCANMD,0x07);			//RAWDATA MODE single mode
	}
	
	AW_Base.CompensateState = BASE_INITIAL;
	AW_Base.FrameCnt = 0;
	AW_Base.InitialFrameCnt = 0;
	
	for(j=0; j<AWTPCfg.RX_LOCAL; j++)
	{
		for(i=0; i<AWTPCfg.TX_LOCAL; i++)
		{
			AW_Base.Base[i][j] = 0;
 		}
 	}
}

void InitDeltaMode(void)
{
	short i,j,k;
	unsigned short captemp;
	unsigned char ret,data[504];
	
	I2C_WriteByte(SA_SCANMD,0x00);			//scan off
	I2C_WriteByte(SA_IER,0x00);				//frame int disable
	I2C_WriteByte(SA_CTRL,0x09); 			//scan en

	I2C_ReadByte(SA_ISR);
	I2C_WriteByte(SA_PAGE,0x01);			//change page 1
	I2C_WriteByte(SA_TRACECTRL1,0x03);		//base function enable  close base trace
	I2C_WriteByte(SA_BIGPOINTTH, 0x10);		//big area threshold
	I2C_WriteByte(SA_POSNUMTH,0x01);		//pos num threshold

	I2C_WriteByte(SA_PAGE,0x00);			//change page 0


	for(i=0;i<20;i++)
	{
		ret = I2C_ReadByte(SA_ISR);			//wait base fresh finish
		if((ret & 0x08) == 0x08)
		{
			break;
		}
		AW_Sleep(5);
	}

	I2C_WriteByte(SA_READSEL,1);
	I2C_WriteByte(SA_ADDRH,0);
	I2C_WriteByte(SA_ADDRL,0);
	I2C_ReadXByte(data,SA_RAWDATA,AWTPCfg.TX_LOCAL*AWTPCfg.RX_LOCAL*2);

	for(i=0;i<AWTPCfg.TX_LOCAL;i++)
	{
		for(j=0;j<AWTPCfg.RX_LOCAL;j++)
		{
			captemp = data[i*AWTPCfg.RX_LOCAL*2+j*2];
			captemp <<= 8;
			captemp += data[i*AWTPCfg.RX_LOCAL*2+j*2+1];

			if (AWTPCfg.RX_INV_ORDER == 1)
			{
				k = AWTPCfg.RX_LOCAL-1-j;
				AW_Base.Base[i][k] = captemp;
				AW_Base.ChipBase[i][k] = captemp;
			} 
			else 
			{
				AW_Base.Base[i][j] = captemp;
				AW_Base.ChipBase[i][j] = captemp;
			}
		}
	}
	I2C_WriteByte(SA_READSEL,0);
	
	if(AWTPCfg.MULTI_SCANFREQ)
	{
		I2C_WriteByte(SA_SCANMD,0x87|0x20);			//start scan
	}
	else
	{
		I2C_WriteByte(SA_SCANMD,0x87);	//RAW DATA MODE start scan
	}

	AW_Base.CompensateState = BASE_STABLE;


	#ifdef NEWBASE_PROCESS
	k = 0;
	for(i=0;i<AWTPCfg.TX_LOCAL;i++)
	{
		for(j=0;j<AWTPCfg.RX_LOCAL;j++)
		{
			captemp = AW_Base.Base[i][j] + AW_Cali.SOFTOFFSET[i][j];

			if(captemp < BASE_VALUE-BASE_TH)
			{
				AW_Base.Flag[i][j] = -1;
				k++;
			}
			if(captemp > BASE_VALUE+BASE_TH)
			{
				AW_Base.Flag[i][j] = 1;
				k++;
			}
		}
	}
	if(k > 0)
	{
		AW_Base.CompensateState = BASE_INITIAL;
		AW_Base.FrameCnt = 0;
	}
	#endif
}

void InitMonitorMode(void)
{
	unsigned char ret;
	unsigned char i;
	
	I2C_WriteByte(SA_SCANMD,0x00);			//scan off
	I2C_WriteByte(SA_IER,0x00);				//frame int enable
	I2C_WriteByte(SA_CTRL,0x0b); 			//scan en & monitor en 

	I2C_WriteByte(SA_PAGE,0x01);			//change page 1
	I2C_WriteByte(SA_TRACECTRL1,0x11);		//base function enable
	I2C_WriteByte(SA_BIGPOINTTH, 0x10);		//big area threshold
	I2C_WriteByte(SA_POSNUMTH,0x01);		//pos num threshold

	I2C_WriteByte(SA_PAGE,0x00);			//change page 0

	AW_Sleep(100);
	for(i=0;i<10;i++)
	{
		ret = I2C_ReadByte(SA_ISR);			//read int
		if((ret & 0x01) == 0x01)
		{
			break;
		}
		AW_Sleep(5);
	}
	if(AWTPCfg.MULTI_SCANFREQ)
	{
		I2C_WriteByte(SA_SCANMD,0x81|0x20);			//start scan
	}
	else
	{
		I2C_WriteByte(SA_SCANMD,0x81);	//DIFF MODE 
	}
}

char CollectRawData(void)
{
	short i,j,k;
	unsigned short captemp;
	unsigned char ret,data[504];

//	printk("collect RAW data\n");
	for(i=0;i<20;i++)
	{
		ret = I2C_ReadByte(SA_SCANMD);			//read int
		if((ret & 0x01) == 0x00)
		{
			break;
		}
		AW_Sleep(1);
	}

//	printk("AW5306 read data\n");
	if(AWTPCfg.MULTI_SCANFREQ)
	{
		I2C_WriteByte(SA_SCANMD,0x07|0x20); 		//start scan
	}
	else
	{
		I2C_WriteByte(SA_SCANMD,0x07);			//start scan
	}
	
	I2C_WriteByte(SA_ADDRH,0);
	I2C_WriteByte(SA_ADDRL,0);
	I2C_ReadXByte(data,SA_RAWDATA,AWTPCfg.TX_LOCAL*AWTPCfg.RX_LOCAL*2);

	for(i=0;i<AWTPCfg.TX_LOCAL;i++)
	{
		for(j=0;j<AWTPCfg.RX_LOCAL;j++)
		{
			captemp = data[i*AWTPCfg.RX_LOCAL*2+j*2];
			captemp <<= 8;
			captemp += data[i*AWTPCfg.RX_LOCAL*2+j*2+1];
			if (AWTPCfg.RX_INV_ORDER == 1)
			{
				k = AWTPCfg.RX_LOCAL-1-j;
				Diff[i][k] = captemp;
			} 
			else 
			{
				Diff[i][j] = captemp;
			}
		}
	}
	#if 0
	printk("AW5306 RAW DATA: \n");
	for(i=0;i<18;i++)
	{
		printk("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d \n",Diff[i][0],Diff[i][1],Diff[i][2],Diff[i][3],Diff[i][4],Diff[i][5],
												Diff[i][6],Diff[i][7],Diff[i][8],Diff[i][9]);
	}
	#endif
	
	return 1;
}


void CaculateDelta(void)
{
	unsigned char i,j;
	
	for(j=0; j<AWTPCfg.RX_LOCAL; j++)
	{
 		for(i=0; i<AWTPCfg.TX_LOCAL; i++)
		{
 			Diff[i][j] = Diff[i][j] - AW_Base.Base[i][j];
			adbDiff[i][j] = Diff[i][j];
  		}
 	}
}

char CollectDeltaData(void)
{
	unsigned char i,j,ret,readNum;
	short captemp;
	unsigned char tx,rx;
	unsigned char data[504];
	
	for(i=0;i<AWTPCfg.TX_LOCAL;i++)
	{
		for(j=0;j<AWTPCfg.RX_LOCAL;j++)
		{
			Diff[i][j] = 0;
		}
	}

//	printk("collect delta data\n");
	
	ret = I2C_ReadByte(SA_SCANMD);
	while((ret&0x01) != 0x00)
	{
		ret = I2C_ReadByte(SA_SCANMD);			//read int
	}
	
	readNum = I2C_ReadByte(SA_VLDNUM);

	if(AWTPCfg.MULTI_SCANFREQ)
	{
		I2C_WriteByte(SA_SCANMD,0x87|0x20);			//start scan
	}
	else
	{
		I2C_WriteByte(SA_SCANMD,0x87);			//start scan
	}
//	printk("AW5306 readNum = %d \n",readNum);
	if(readNum > 0)
	{
		if(readNum >160)
		{
			readNum = 160;
		}
		I2C_WriteByte(SA_ADDRH,0);
		I2C_WriteByte(SA_ADDRL,0);
		I2C_ReadXByte(data,SA_RAWDATA,readNum*3);
	
		for(i =0;i<readNum;i++)
		{
			tx = data[i*3] / AWTPCfg.RX_LOCAL;			//??? 总是大于12
			if (AWTPCfg.RX_INV_ORDER == 1)
			{
				rx = AWTPCfg.RX_LOCAL - (data[i*3] % AWTPCfg.RX_LOCAL) - 1;
			}
			else
			{
				rx = data[i*3] % AWTPCfg.RX_LOCAL;
			}
			captemp = data[i*3+1];
			captemp <<= 8;
			captemp += data[i*3+2];
			Diff[tx][rx] = captemp;
		}

		for(i=0;i<AWTPCfg.TX_LOCAL;i++)
		{
			for(j=0;j<AWTPCfg.RX_LOCAL;j++)
			{
				Diff[i][j] = Diff[i][j] + (short)AW_Base.ChipBase[i][j] - (short)AW_Base.Base[i][j];
				adbDiff[i][j] = Diff[i][j];
			}
		}
		return 1;
	}
	else
	{
		for(i=0;i<AWTPCfg.TX_LOCAL;i++)
		{
			for(j=0;j<AWTPCfg.RX_LOCAL;j++)
			{
				Diff[i][j] = (short)AW_Base.ChipBase[i][j] - (short)AW_Base.Base[i][j];
				adbDiff[i][j] = Diff[i][j];
			}
		}
		return 1;
	}
}

void BaseInit(void)
{
	unsigned char i,j;
	
	CollectRawData();
	for(i=0; i<AWTPCfg.TX_LOCAL; i++)
	{
		for(j=0; j<AWTPCfg.RX_LOCAL; j++)
		{
			AW_Base.Base[i][j] += Diff[i][j]/4;
		}
	}
	AW_Base.FrameCnt++;
	if(AW_Base.FrameCnt == 4)
	{
		AW_Base.FrameCnt = 0;
		AW_Base.CompensateState = BASE_FAST_TRACE;
	}
}

void BaseReInit(void)
{
	unsigned char i,j;
	
	AW_Base.CompensateState = BASE_INITIAL;
	AW_Base.FrameCnt = 0;
	
	for(i=0; i<AWTPCfg.TX_LOCAL; i++)
	{
		for(j=0; j<AWTPCfg.RX_LOCAL; j++)
		{
			AW_Base.Base[i][j] = 0;
		}
	}
}

void TP_Init(void)
{
	unsigned char i,j;
	//InitI2C();
	switch(AW5306_WorkMode)
	{
		case RawDataMode:
			InitRawDataMode();
			break;
		case DeltaMode:
			InitDeltaMode();
			break;

		case MonitorMode:
			InitMonitorMode();
			AW_Base.BaseFrozen = 0;
			break;
		default:
			break;
	}
	for(i=0;i<MAX_POINT;i++)
	{
		AW_LastPoint[i].X = 0x7FFF;
		AW_LastPoint[i].Y = 0x7FFF;
		AW_LastPoint[i].Event = NO_EVENT;
		AW_LastPoint[i].PointID = 0xFF;

		AW_LastPointInfo[i].X = 0x7FFF;
		AW_LastPointInfo[i].Y = 0x7FFF;
		AW_LastPointInfo[i].Event = NO_EVENT;
		AW_LastPointInfo[i].PointID = 0xFF;
		
		AW_Frame.RptPoint[i].X = 0x7FFF;
		AW_Frame.RptPoint[i].Y = 0x7FFF;
		AW_Frame.RptPoint[i].Event = NO_EVENT;
		AW_Frame.RptPoint[i].PointID= 0xFF;
	}
	for(i=0; i<AWTPCfg.TX_LOCAL; i++)
	{
		for(j=0; j<AWTPCfg.RX_LOCAL; j++)
		{
			//AW_Base.Flag[i][j] = 0;
			AW_Base.BaseCnt[i][j] = 0;
		}
	}
	AW_Frame.PointNum = 0;
	
}
unsigned long fileoffset = 0;
unsigned char write_Flag = 0;
char CollectTPData(void)
{
	switch(AW5306_WorkMode)
	{
		case RawDataMode:
			if(AW_Base.CompensateState == BASE_INITIAL)
			{
				BaseInit();
				return 0;
			}
			else
			{
				if(CollectRawData())
				{
					CaculateDelta();
				if(AWTPCfg.DEBUG_SWITCH == 1)
				{
					if(write_Flag == 1)
					{
						nvram_write("data/awdiff", (char *)&Diff[0][0], NUM_TX*NUM_RX*2, fileoffset);
						fileoffset = fileoffset+NUM_TX*NUM_RX*2;
					}	
				}
					return 1;
				}
				else
				{
					return 0;
				}
			}
			break;
		case DeltaMode:
		case MonitorMode:
			if(CollectDeltaData())
			{
				if(AWTPCfg.DEBUG_SWITCH == 1)
				{
					if(write_Flag == 1)
					{
						nvram_write("data/awdiff", (char *)&Diff[0][0], NUM_TX*NUM_RX*2, fileoffset);
						fileoffset = fileoffset+NUM_TX*NUM_RX*2;
					}	
				}
				return 1;
			}
			else
			{
				return 0;
			}
			break;
		default:
			return 0;
			break;
	}
}

void CollectBase(void)
{
	unsigned char i,j;
	unsigned short captemp;
	unsigned char data[504];
	unsigned char reg_data;
	
	// Switch to Base reading mode
	reg_data = I2C_ReadByte(SA_SCANMD);
	reg_data = reg_data & 0xFE;		// Disable RUN
	I2C_WriteByte(SA_SCANMD, reg_data);
	I2C_WriteByte(SA_READSEL,1);	// Switch to Base reading mode
	
	I2C_WriteByte(SA_ADDRH,0);
	I2C_WriteByte(SA_ADDRL,0);
	I2C_ReadXByte(data,SA_RAWDATA,AWTPCfg.TX_LOCAL*AWTPCfg.RX_LOCAL*2);
	reg_data = reg_data | 0x1;		// Enable RUN
	I2C_WriteByte(SA_SCANMD, reg_data);

	for(i=0;i<AWTPCfg.TX_LOCAL;i++)
	{
		for(j=0;j<AWTPCfg.RX_LOCAL;j++)
		{
			captemp = data[i*AWTPCfg.RX_LOCAL*2+j*2];
			captemp <<= 8;
			captemp += data[i*AWTPCfg.RX_LOCAL*2+j*2+1];
			if (AWTPCfg.RX_INV_ORDER == 1)
			{
				AW_Base.Base[i][AWTPCfg.RX_LOCAL-1-j] = captemp;
			} else {
				AW_Base.Base[i][j] = captemp;
			}
		}
	}

	I2C_WriteByte(SA_READSEL,0);	// Switch back to diff reading mode

}

void RawDataFilter(void)
{
	unsigned int i,j,k,GroupID,MaxGroupSum;
	signed short GroupSum [8];
	signed short DataTmp [4];
	unsigned short GroupCount[8];
	

	for(j=0; j<AWTPCfg.RX_LOCAL; j++)
	{
		for(i=0; i<8; i++)
		{
			if (i >= 3)
			{
				GroupSum[i] = 1000;
			}
			else
			{
				GroupSum[i] = 0;
			}
			GroupCount[i] = 0;
		}
		GroupSum[7] = 0;
		for(i=0; i<AWTPCfg.TX_LOCAL; i++)
		{
			if (Diff[i][j] > GroupSum[0])
			{
				GroupSum[2] = GroupSum[1];
				GroupSum[1] = GroupSum[0];
				GroupSum[0] = Diff[i][j];
			}
			else if (Diff[i][j] > GroupSum[1])
			{
				GroupSum[2] = GroupSum[1];
				GroupSum[1] = Diff[i][j];
			}
			else if (Diff[i][j] > GroupSum[2])
			{
				GroupSum[2] = Diff[i][j];
			}

			if (Diff[i][j] < GroupSum[4])
			{
				GroupSum[6] = GroupSum[5];
				GroupSum[5] = GroupSum[4];
				GroupSum[4] = Diff[i][j];
			}
			else if (Diff[i][j] < GroupSum[5])
			{
				GroupSum[6] = GroupSum[5];
				GroupSum[5] = Diff[i][j];
			}
			else if (Diff[i][j] < GroupSum[6])
			{
				GroupSum[6] = Diff[i][j];
			}
			if (Diff[i][j] > 0)
			{
				GroupSum[7] += Diff[i][j];
			}
		}
		GroupSum[7] -= GroupSum[2];
		GroupSum[7] -= GroupSum[1];
		GroupSum[7] -= GroupSum[0];
		if (GroupSum[4] > 0)
		{
			GroupSum[7] -= GroupSum[4];
		}
		if (GroupSum[5] > 0)
		{
			GroupSum[7] -= GroupSum[5];
		}
		if (GroupSum[6] > 0)
		{
			GroupSum[7] -= GroupSum[6];
		}
		GroupSum[7] = GroupSum[7]/(AWTPCfg.TX_LOCAL-6);
		THPEAKCOL[j] = GroupSum[7]*2/3;
	}

	for(j=0; j<AWTPCfg.TX_LOCAL; j++)
	{
		for(i=0; i<8; i++)
		{
			if (i >= 3)
			{
				GroupSum[i] = 1000;
			}
			else
			{
				GroupSum[i] = 0;
			}
			GroupCount[i] = 0;
		}
		GroupSum[7] = 0;
		for(i=0; i<AWTPCfg.RX_LOCAL; i++)
		{
			if (Diff[j][i] > GroupSum[0])
			{
				GroupSum[2] = GroupSum[1];
				GroupSum[1] = GroupSum[0];
				GroupSum[0] = Diff[j][i];
			}
			else if (Diff[j][i] > GroupSum[1])
			{
				GroupSum[2] = GroupSum[1];
				GroupSum[1] = Diff[j][i];
			}
			else if (Diff[j][i] > GroupSum[2])
			{
				GroupSum[2] = Diff[j][i];
			}

			if (Diff[j][i] < GroupSum[4])
			{
				GroupSum[6] = GroupSum[5];
				GroupSum[5] = GroupSum[4];
				GroupSum[4] = Diff[j][i];
			}
			else if (Diff[j][i] < GroupSum[5])
			{
				GroupSum[6] = GroupSum[5];
				GroupSum[5] = Diff[j][i];
			}
			else if (Diff[j][i] < GroupSum[6])
			{
				GroupSum[6] = Diff[j][i];
			}
			if (Diff[j][i] > 0)
			{
				GroupSum[7] += Diff[j][i];
			}
		}
		GroupSum[7] -= GroupSum[2];
		GroupSum[7] -= GroupSum[1];
		GroupSum[7] -= GroupSum[0];
		if (GroupSum[4] > 0)
		{
			GroupSum[7] -= GroupSum[4];
		}
		if (GroupSum[5] > 0)
		{
			GroupSum[7] -= GroupSum[5];
		}
		if (GroupSum[6] > 0)
		{
			GroupSum[7] -= GroupSum[6];
		}
		GroupSum[7] = GroupSum[7]/(AWTPCfg.RX_LOCAL-6);
		THPEAKROW[j] = GroupSum[7];
	}

}

void FreshBase(void)
{
	unsigned char i,j,ret;
	unsigned char buf[480];
	unsigned short wdata;

	wdata = 8000;

	ret = I2C_ReadByte(SA_SCANMD);//wait scan finish
	while((ret&0x01) != 0x00)
	{
		ret = I2C_ReadByte(SA_SCANMD);		
	}
	
	I2C_WriteByte(SA_READSEL,1);	// Switch to Base reading mode

	
	I2C_WriteByte(SA_ADDRH,0);
	I2C_WriteByte(SA_ADDRL,0);

	//printk("write:\n");
	
	for(i=0;i<AWTPCfg.TX_LOCAL;i++)
	{
		for(j=0;j<AWTPCfg.RX_LOCAL;j++)
		{
			if (AWTPCfg.RX_INV_ORDER == 1)
			{
				buf[(i*AWTPCfg.RX_LOCAL+j)*2] = (unsigned char) ((AW_Base.Base[i][AWTPCfg.RX_LOCAL-1-j]&0xFF00)>>8);
				buf[(i*AWTPCfg.RX_LOCAL+j)*2+1] = (unsigned char) (AW_Base.Base[i][AWTPCfg.RX_LOCAL-1-j]&0x00FF);
				
			}
			else
			{
				buf[(i*AWTPCfg.RX_LOCAL+j)*2] = (unsigned char) ((AW_Base.Base[i][j]&0xFF00)>>8);
				buf[(i*AWTPCfg.RX_LOCAL+j)*2+1] = (unsigned char) (AW_Base.Base[i][j]&0x00FF);
			}
			//printk("%d, ",(short)(buf[(i*AWTPCfg.RX_LOCAL+j)*2]<< 8)|buf[(i*AWTPCfg.RX_LOCAL+j)*2+1]);
			wdata++;
		}
		//printk("\n");
	}

	
	I2C_WriteXByte(buf,SA_RAWDATA,AWTPCfg.TX_LOCAL*AWTPCfg.RX_LOCAL*2);
	
#if 0

	ret =I2C_ReadXByte(buf,0,127);
	for(i=0;i<127;i++)
	{
		if(i%8 == 0)
		{
			printk("\n");
		}
		printk("%d, ",buf[i]);
		
	}

	I2C_WriteByte(SA_ADDRH,0);
	I2C_WriteByte(SA_ADDRL,0);
	I2C_ReadXByte(buf,SA_RAWDATA,AWTPCfg.TX_LOCAL*AWTPCfg.RX_LOCAL*2);
	printk("read:\n");
	for(i=0;i<AWTPCfg.TX_LOCAL;i++)
	{
		for(j=0;j<AWTPCfg.RX_LOCAL;j++)
		{
			printk("%d, ",(short)(buf[(i*AWTPCfg.RX_LOCAL+j)*2]<< 8)|buf[(i*AWTPCfg.RX_LOCAL+j)*2+1]);
		}
		printk("\n");
	}

	ret =I2C_ReadByte(SA_ADDRH);
	printk("%d, ",ret);
	ret =I2C_ReadByte(SA_ADDRL);
	printk("%d, ",ret);
#endif
	I2C_WriteByte(SA_READSEL,0);

	if(AWTPCfg.MULTI_SCANFREQ)
	{
		I2C_WriteByte(SA_SCANMD,0x87|0x20);			//start scan
	}
	else
	{
		I2C_WriteByte(SA_SCANMD,0x87);			//start scan
	}
}

void BlurParamSelect(void)
{
	unsigned long int distance;
	unsigned short usTemp;

	if(AW_Frame.PointNum == 2)
	{
        usTemp  = CalDiff(AW_Frame.PointInfo[1].X, AW_Frame.PointInfo[0].X);
		distance= usTemp * usTemp;
        usTemp  = CalDiff(AW_Frame.PointInfo[1].Y, AW_Frame.PointInfo[0].Y);
        distance+= usTemp * usTemp;

		if(distance > LONG_DISTANCE)
		{
			G_BlurEnable = 0;
		}
		else if(distance < SHORT_DISTANCE)
		{
			G_BlurEnable = 1;
		}
	}
	else
	{
		if((AW_Frame.PointNum == 1) && (G_BlurEnable == 1))
		{
			G_BlurEnable = 1;	
		}
		else
		{
			G_BlurEnable = 0;	
		}
	}	
}

void RawDataBlur(void)
{
	unsigned char i, j, k,h;

	typedef	struct {
		char blur_factor[3][3];
		unsigned char shift_num;
	}STRUCTBLURPARAM;

	//sharpen   parameter
	const	  char  BlurFactor[3][3] = {  
								{3, 8, 3},
								{8, 20, 8},
				 				{3, 8, 3},
							   };
	#define		SHIFT_BIT		5
	short RawDataTempBuf[3][NUM_RX], tempData;

	int sum;	

	if(G_BlurEnable == 0)
	{
	    return;
	}

	#if 1	
	tempData = 0;
	for(i = 0; i < AWTPCfg.TX_LOCAL; i++)
	{
		for(j = 0; j < AWTPCfg.RX_LOCAL; j++)
		{
			if(Diff[i][j] > tempData)tempData = Diff[i][j];
		}
	}

	if(tempData < 250)
	//if(tempData < 400)   //防止与悬空下的两点合并功能冲突
	{
		return;
	}
	#endif
	//step1. init raw data temp buffer
	for(k = 0; k < AWTPCfg.RX_LOCAL; k++)
	{
		RawDataTempBuf[0][k] = 0;
		RawDataTempBuf[1][k] = 0;
		RawDataTempBuf[2][k] = Diff[0][k];		
	}
	
	for(i = 0; i < AWTPCfg.TX_LOCAL; i++)
	{
		//step2. load Raw data to temp buffer 
#if 1
		for(k = 0; k < AWTPCfg.RX_LOCAL; k++)
		{
			RawDataTempBuf[0][k] = RawDataTempBuf[1][k];
			RawDataTempBuf[1][k] = RawDataTempBuf[2][k];
			if((i+1)>= AWTPCfg.TX_LOCAL)
			{
	            RawDataTempBuf[2][k] = 0;			
			}
			else
			{
			    RawDataTempBuf[2][k] = Diff[i+1][k];			
			}
		}
#endif  // 1
		//step3: Blur every RX data
		for(j = 0; j < AWTPCfg.RX_LOCAL; j++)
		{
			if(Diff[i][j] > MIN_BLUR_VAL)
			{
			    //step3.1: calcultion the sum
			    sum = 0;
			    for(k = 0; k < 3; k++)
			    {
					for(h = 0; h < 3; h++)
					{
						if(((j+h)>= 1) && ((j-1+h) < AWTPCfg.RX_LOCAL))
						{// check data is available or not
							tempData = RawDataTempBuf[k][j-1+h];
						}
						else
						{
							tempData = 0;	
						}
						if(tempData < 0)tempData = 0;
						sum +=  ((long int)tempData * BlurFactor[k][h]);	//[k][h]
					}
			    }		
			    // step3.2: calcultion the average, and replace the raw data
			    Diff[i][j] = (sum >>SHIFT_BIT);
			}
		}
	}
}

void WaterDefense(void)
{
	unsigned char i,j,row,col;
	short DeltaBase, DeltaDiff, MinDiff;
	unsigned char MinRow;
	unsigned char reg_data;

	MinRow = 0;
	
	if (AW5306_WorkMode != DeltaMode)		// only working in DeltaMode
	{
		return;
	}
	if ((AW_Peak.CurrentNegPointNum == 0) || (AW_Peak.CurrentPointNum > 0))
	{
		if (AW_Base.BaseFrozen == 1)
		{
			AW_Base.BaseFrozen = 0;

			// Enable Base Tracing
			I2C_WriteByte(SA_PAGE, 1);
			reg_data = I2C_ReadByte(SA_TRACECTRL1);
			reg_data = reg_data | 0x10;
			I2C_WriteByte(SA_TRACECTRL1, reg_data);
			I2C_WriteByte(SA_PAGE, 0);

		}
	}
	else
	{
		if (AW_Base.BaseFrozen == 0)
		{
			CollectBase();
		}
		AW_Base.BaseFrozen = 0;
		for (i = 0; i < AW_Peak.CurrentNegPointNum; i++)
		{
			row = AW_Peak.NegPeak[i][0]/2;
			col = AW_Peak.NegPeak[i][1]/2;
			MinDiff = 0xfff;
			if (row > AWTPCfg.TX_LOCAL/2)
			{
				for (j = row-1; j > 0; j--)	// search min Diff point
				{
					if (ABS(Diff[j][col]) < ABS(MinDiff))
					{
						MinDiff = Diff[j][col];
						MinRow = j;
						if (ABS(MinDiff) < 10)
						{
							break;
						}
					}
				}
			}
			else
			{
				for (j = row+1; j < AWTPCfg.TX_LOCAL; j++)	// search min Diff point
				{
					if (ABS(Diff[j][col]) < ABS(MinDiff))
					{
						MinDiff = Diff[j][col];
						MinRow = j;
						if (ABS(MinDiff) < 10)
						{
							break;
						}
					}

				}
			}
			DeltaDiff = Diff[row][col] - Diff[MinRow][col];
			DeltaBase = AW_Base.Base[row][col] - AW_Base.Base[MinRow][col];
			DeltaDiff = DeltaDiff + DeltaBase;
			if (ABS(DeltaDiff) > ABS(DeltaBase))
			{
				AW_Base.BaseFrozen = 1;
			}
		}

		// Disable Base Tracing
		if (AW_Base.BaseFrozen == 1)
		{
			I2C_WriteByte(SA_PAGE, 1);
			reg_data = I2C_ReadByte(SA_TRACECTRL1);
			reg_data = reg_data & 0xEF;
			I2C_WriteByte(SA_TRACECTRL1, reg_data);
			I2C_WriteByte(SA_PAGE, 0);
		}
		else
		{
			// Enable Base Tracing
			I2C_WriteByte(SA_PAGE, 1);
			reg_data = I2C_ReadByte(SA_TRACECTRL1);
			reg_data = reg_data | 0x10;
			I2C_WriteByte(SA_TRACECTRL1, reg_data);
			I2C_WriteByte(SA_PAGE, 0);
		}

	}
}

void SearchPeak(void)
{
	unsigned char i,j,p_k,P_N,p_i,i_temp,j_temp,NegCnt,i_mid,j_mid;
	short Data,Offset,PosBigAreaCnt,NegBigAreaCnt,t_left,t_right,t_up,t_down;
	short MinValue,Temp;
	unsigned char NegPeak[5][2];
	unsigned char peak[20][2];
	unsigned char PeakTemp[20][2];

	NegCnt = 0;
	P_N = 0;
	PosBigAreaCnt = 0;
	NegBigAreaCnt = 0;

	for(i=0;i<AWTPCfg.TX_LOCAL;i++)
	{
		for(j=0;j<AWTPCfg.RX_LOCAL;j++)
		{
			Data = Diff[i][j];
			if(Data > BIGAREA_PEAK_TH)
			{
				PosBigAreaCnt++;
			}
			if(Data < -BIGAREA_PEAK_TH)
			{
				NegBigAreaCnt++;
			}
			if(THPEAKCOL[j] > THPEAKROW[i])
			{
				Offset = THPEAKCOL[j];
			}
			else
			{
				Offset = THPEAKROW[i];
			}
			if(Data > THPEAK + Offset)		//POS PEAK
			{
				if(i == 0)				
				{
					t_left = 0;
				}
				else
				{
					t_left = Diff[i-1][j];
				}
				if(i == AWTPCfg.TX_LOCAL-1)
				{
					t_right = 0;
				}
				else
				{
					t_right = Diff[i+1][j];
				}
				if(j==0)
				{
					t_up = 0;
				}
				else
				{
					t_up = Diff[i][j-1];
				}
				if(j == AWTPCfg.RX_LOCAL-1)
				{
					t_down = 0;
				}
				else
				{
					t_down = Diff[i][j+1];
				}

				if(t_left < t_right)
				{
					t_left = t_right; 
				}
				if(t_up < t_down)
				{
					t_up = t_down;
				}
				if(t_left < t_up)
				{
					t_left = t_up;
				}

				if(Data >= t_left)			
				{
					i_temp = i*2;
					j_temp = j*2;

					if(P_N == 0)
					{
						peak[0][0] = i_temp;
						peak[0][1] = j_temp;
						P_N++;
					}
					else if(P_N < 20)
					{
					
						for(p_k = 0; p_k < P_N; p_k++)
						{
							#if 0
							if((CalDiff(i_temp,peak[p_k][0]) <= 2) && (CalDiff(j_temp,peak[p_k][1]) <= 2))
							{
								if(Diff[i][j] > Diff[peak[p_k][0]/2][peak[p_k][1]/2])			
								{
									peak[p_k][0] = i*2;
									peak[p_k][1] = j*2;
								}
								p_k = 0xFF;
								break;
							}
							
							else if((CalDiff(i_temp,peak[p_k][0]) < 6) && (CalDiff(j_temp,peak[p_k][1]) < 6))
							{
								i_mid = (i_temp+peak[p_k][0])/2;
								j_mid = (j_temp+peak[p_k][1])/2;
								for(p_i = 0;p_i < P_N; p_i++)
								{
									if(p_i == p_k)
									{
										continue;
									}
									if((CalDiff(i_mid,peak[p_i][0]) <= 2)&&(CalDiff(j_mid,peak[p_i][1]) <= 2))
									{
										i_mid = 0xFF;			//退出循环
										break;
									}
								}
								if(i_mid != 0xFF)			//中间的点跟其它PEAK点距离都大于2
								{
									t_left = ABS(Diff[i_mid/2][j_mid/2-1]);
									t_right = ABS(Diff[i_mid/2][j_mid/2+1]);
									t_up = ABS(Diff[i_mid/2-1][j_mid/2]);
									t_down = ABS(Diff[i_mid/2+1][j_mid/2]);
									t_left = (t_left+t_right+t_up+t_down)/4;
									if(t_left >= (Diff[peak[p_k][0]/2][peak[p_k][1]/2]*3 /4))
									{
										peak[p_k][0] = i_mid;
										peak[p_k][1] = j_mid;
										p_k = 0xFF;
										break;
									}
								}
								else					
								{

								}
							}
						#endif
						}
					
						if(p_k != 0xFF)			
						{
							peak[P_N][0] = i_temp;
							peak[P_N][1] = j_temp;
							P_N++;
						}
					}
				}
			}
			else if(Data < -THPEAK)		//Neg Peak
			{
				if(i == 0)				
				{
					t_left = 0;
				}
				else
				{
					t_left = Diff[i-1][j];
				}
				if(i == AWTPCfg.TX_LOCAL-1)
				{
					t_right = 0;
				}
				else
				{
					t_right = Diff[i+1][j];
				}
				if(j==0)
				{
					t_up = 0;
				}
				else
				{
					t_up = Diff[i][j-1];
				}
				if(j == AWTPCfg.RX_LOCAL-1)
				{
					t_down = 0;
				}
				else
				{
					t_down = Diff[i][j+1];
				}

				if(t_left > t_right)
				{
					t_left = t_right; 
				}
				if(t_up > t_down)
				{
					t_up = t_down;
				}
				if(t_left > t_up)
				{
					t_left = t_up;
				}

				if(Data < t_left)
				{
					i_temp = i*2;
					j_temp = j*2;
					if(NegCnt == 0)
					{
						NegPeak[NegCnt][0] = i_temp;
						NegPeak[NegCnt][1] = j_temp;
						NegCnt++;
					}
					else
					{
						if(NegCnt < 5)
						{
							for(p_k = 0; p_k < NegCnt; p_k++)	
							{
								if((CalDiff(i_temp,NegPeak[p_k][0]) <= 2) && (CalDiff(j_temp,NegPeak[p_k][1]) <= 2))
								{
									if(Diff[i][j] < Diff[NegPeak[p_k][0]/2][NegPeak[p_k][1]/2])
									{
										NegPeak[p_k][0] = i*2;
										NegPeak[p_k][1] = j*2;
									}
									p_k = 0xFF;
									break;
								}
							}
							if(p_k != 0xFF)	
							{
								NegPeak[NegCnt][0] = i*2;
								NegPeak[NegCnt][1] = j*2;
								NegCnt++;
							}
						}
					}
				}
			}
		}
	}

	if(P_N > MAX_POINT)
	{
		i_mid = 0;

		for(i=0; i< AW_Peak.CurrentPointNum; i++)
		{
			MinValue = 0x3FFF;
			j_temp = 0;
			for(j=0; j<P_N; j++)
			{
				Temp = CalDiff(AW_Peak.LastPeak[i][1], peak[j][1]) + CalDiff(AW_Peak.LastPeak[i][0], peak[j][0]);
				if(Temp < MinValue)
				{
					MinValue = Temp;
					j_temp = j;
				}
			}
			
			if(Diff[peak[j_temp][0]/2][peak[j_temp][1]/2] >= THGROUP)
			{
				PeakTemp[i_mid][0] = peak[j_temp][0];
				PeakTemp[i_mid][1] = peak[j_temp][1];
				peak[j_temp][0] = 0xFF;
				i_mid++;
			}
			else
			{
			
			}
		}
		
		for(i = i_mid; i < MAX_POINT; i++)
		{
			t_left = 0;
			j_mid = 0;
			for(j=0;j<P_N;j++)
			{
				if(peak[j][0] != 0xFF)
				{
					if(Diff[peak[j_temp][0]/2][peak[j_temp][1]/2] >= t_left)
					{
						t_left = Diff[peak[j_temp][0]/2][peak[j_temp][1]/2];
						i_temp = peak[j][0];
						j_temp = peak[j][1];
						j_mid = j;
					}
				}
			}

			PeakTemp[i][0] = i_temp;
			PeakTemp[i][1] = j_temp;
			peak[j_mid][0] = 0xFF;
		}
		
		for(i=0;i<5;i++)
		{
			peak[i][0] = PeakTemp[i][0];
			peak[i][1] = PeakTemp[i][1];
		}
		P_N = 5;
	}

	for(i = 0;i<5;i++)
	{
		AW_Peak.Peak[i][0] = 0xFF;
		AW_Peak.Peak[i][1] = 0xFF;
	}
	
	if(P_N > 0)
	{
		for(i =0; i<P_N; i++)
		{
			AW_Peak.Peak[i][0] = peak[i][0];
			AW_Peak.Peak[i][1] = peak[i][1];
			AW_Peak.LastPeak[i][0] = peak[i][0];
			AW_Peak.LastPeak[i][1] = peak[i][1];
		}
	}
	
	if(NegCnt > 0)
	{
		for(i=0;i<NegCnt;i++)
		{
			AW_Peak.NegPeak[i][0] = NegPeak[i][0];
			AW_Peak.NegPeak[i][1] = NegPeak[i][1];
		}
	}

	AW_Peak.CurrentNegPointNum = NegCnt;
	AW_Peak.CurrentPointNum = P_N;
	AW_Peak.LastPointNum = P_N;
	AW_Base.PosBigAreaTouchFlag = 0;
	AW_Base.NegBigAreaTouchFlag = 0;

	if(PosBigAreaCnt >= BIGAREA_PEAK_NUM)
	{
		AW_Base.PosBigAreaTouchFlag = 1;
	}
	if(NegBigAreaCnt >= BIGAREA_PEAK_NUM)
	{
		AW_Base.NegBigAreaTouchFlag = 1;
	}
#ifdef WATER_PROOF
	WaterDefense();
#endif
}

void ESDCheck(void)
{
	unsigned char i, tx,rx, cnt;//, result;
	unsigned char ucFiterCnt = 0;;

	if ((AW_Peak.CurrentPointNum == 0) ||
	    (AW_Frame.LastPointNum != 0 ))     //如果是画线，不再做ESD的判断
	{
		AW_Frame.FilterPointCnt = 0;
		return;
	}

	tx = (AW_Peak.Peak[0][0] >> 1);
	rx = (AW_Peak.Peak[0][1] >> 1);
	// == check current tx ==
	cnt = 0;
	for(i = 0; i < AWTPCfg.RX_LOCAL; i++)
	{
	    if(ABS(Diff[tx][i]) > MIN_ESD_DIFF_VAL)
        {
            cnt ++;
	    }
	}

	if (cnt > (AWTPCfg.RX_LOCAL - 4))
	{
	    ucFiterCnt =1;//+= ESD_FILTER_FRAMES * (cnt - (AWTPCfg.RX_LOCAL - 4));
	}

	cnt = 0;
	if(tx > 0)
	{
	    for(i = 0; i < AWTPCfg.RX_LOCAL; i++)
	    {
	        if(ABS(Diff[tx][i]) > ABS(Diff[tx - 1][i]))
			{
	            cnt ++;
			}
	    }
	}
	if (cnt > (AWTPCfg.RX_LOCAL - 4))
	{
		ucFiterCnt =1;//+= ESD_FILTER_FRAMES * (cnt - (AWTPCfg.RX_LOCAL - 4));
	}
        

	// == check down tx ==
	cnt = 0;
	if(tx < (AWTPCfg.TX_LOCAL - 1))
	{
	    for(i = 0; i < AWTPCfg.RX_LOCAL; i++)
		{
			if(ABS(Diff[tx][i]) > ABS(Diff[tx + 1][i]))
			{
				cnt ++;
			}
		}
    }
    if (cnt > (AWTPCfg.RX_LOCAL - 4))
    {
        ucFiterCnt =1;//+= ESD_FILTER_FRAMES * (cnt - (AWTPCfg.RX_LOCAL - 4));
    }

    if (rx > 0)
    {
        if (Diff[tx][rx - 1] < MIN_ESD_NEGTIVE)
        {
            ucFiterCnt =1;//+= ESD_FILTER_FRAMES;
        }
    }

    if ((rx + 1) < AWTPCfg.TX_LOCAL)
    {
        if (Diff[tx][rx + 1] < MIN_ESD_NEGTIVE)
        {
            ucFiterCnt =1;//+= ESD_FILTER_FRAMES;
        }
    }
    AW_Frame.FilterPointCnt = ucFiterCnt;
}


void ESDFilterProcess()
{
	//ESD PROCESS
	if(AW_Peak.CurrentPointNum > AW_Frame.LastPointNum)
	{
	//	printk("AW_Frame.FilterPointCnt = %d\n", TouchedCnter);
		if(TouchedCnter < AW_Frame.FilterPointCnt)
		{
			TouchedCnter++;
			AW_Peak.CurrentPointNum = AW_Frame.LastPointNum;
		}
	}
	else
	{
		TouchedCnter = 0;
	}
}

void  FilterMarginPoint(void)
{
	short  temp;
	unsigned short x,y;
	unsigned char i;
	unsigned short POSX_MAX;	//The max x coordinate supported by CTMP 
	unsigned short POSY_MAX;	//The max y coordinate supported by CTMP
	unsigned short pitch_Y_Down;
	
	POSX_MAX = AWTPCfg.RX_LOCAL*POS_PRECISION - 1;
	POSY_MAX = AWTPCfg.TX_LOCAL*POS_PRECISION - 1;

	if(AWTPCfg.HAVE_KEY_LINE == 1)
	{
		pitch_Y_Down = PITCH_Y_DOWN+POS_PRECISION;
	}
	else
	{
		pitch_Y_Down = PITCH_Y_DOWN;
	}
	for (i = 0; i < AW_Frame.PointNum; i++)
	{
		x = AW_Frame.PointInfo[i].X;
		y = AW_Frame.PointInfo[i].Y;
	

		if (x<POSX_MIN) x = POSX_MIN;
		if (y<POSY_MIN) y = POSY_MIN;

		if (x <= (PITCH_X_LEFT))
		{
			//temp = (x << 1) - PITCH_X_LEFT ;
			temp = PITCH_X_LEFT - x;
			temp = x - temp*temp/32;
		if (temp < 2)
			{
				temp = 0;
			}
			x = temp;
		}
		else if (x >= (POSX_MAX - PITCH_X_RIGHT ))
		{
			//temp = x + (x- (POSX_MAX - PITCH_X_RIGHT));
			temp = x- (POSX_MAX - PITCH_X_RIGHT);
			temp = x + temp*temp/32;
			if (temp > (POSX_MAX - 2))
			{
				temp = POSX_MAX;
			}
			x = temp;
		}


		if (y <= (PITCH_Y_UP))
		{
			temp = (y<<1) - PITCH_Y_UP;
			// temp = PITCH_Y_UP - y;
			// temp = y - temp*temp/32;
			if (temp < 2)
			{
				temp = 0;
			}
			y = temp;
		}
		else if (y >= (POSY_MAX  - pitch_Y_Down ))
		{
			temp = y + (y-(POSY_MAX - pitch_Y_Down));
			// temp = y-(POSY_MAX - pitch_Y_Down);
			// temp = y + temp*temp/32;
			if (temp > (POSY_MAX - 2))
			{
				temp = POSY_MAX;
			}
			y = temp;
		}
		
		AW_Frame.PointInfo[i].X = x;
		AW_Frame.PointInfo[i].Y = y;	
	}
}


void CalculateCoordinate(void)
{
	unsigned char k,calc,GroupNum,CalNoiseTH,i_mid,TouchArea;
	short i,j,i_temp,j_temp;
	short t_left,t_up;
	short startX,EndX,startY,EndY;
	short TouchDataDiff;
	int sum_X[5],sum_Y[5],sum_t[5],sum_tb[5];
	unsigned char row,col;
	short neigbour_i, neigbour_j;

	GroupNum = 0;
	TouchArea = 0;

	CalNoiseTH = CALNOISE_TH;

	for(k=0;k<MAX_POINT;k++)
	{
		AW_Frame.PointInfo[k].X = 0;
		AW_Frame.PointInfo[k].Y = 0;
	}

	if(AW_Peak.CurrentPointNum != 0)
	{
		for(k=0; k<AW_Peak.CurrentPointNum; k++)
		{
			sum_X[k] = 0;
			sum_Y[k] = 0;
			sum_t[k] = 0;
			sum_tb[k] = 0;
			
			calc = 1;

			if(AW_Peak.Peak[k][0] == 0xFF)		//point has remove
			{
				continue;
			}

			row = AW_Peak.Peak[k][0]/2;
			if(row < 1 || row >= (AWTPCfg.TX_LOCAL-1-AWTPCfg.HAVE_KEY_LINE))
			{
				calc = MARGCOORNUM /2;
			}
			startX = row-calc;
			EndX = row+calc;

				
			col =  AW_Peak.Peak[k][1]/2;
			if(col <1 || col >=(AWTPCfg.RX_LOCAL-1))
			{
				calc = MARGCOORNUM /2;
			}
			startY = col-calc;
			EndY = col+calc;

			if ((AWTPCfg.HAVE_KEY_LINE == 1) && (row >= AWTPCfg.TX_LOCAL-1))		// peak on key line
			{
				TouchDataDiff = Diff[row][col] - CalNoiseTH;
				sum_Y[k] = TouchDataDiff *row;
				sum_X[k] = TouchDataDiff *col;
				sum_t[k] = TouchDataDiff;
				sum_tb[k] = TouchDataDiff;
				continue;
			}
			
			for(i=startX; i <= EndX; i++)
			{
				for(j = startY; j <= EndY; j++)
				{
					if((i>=0) && (i<AWTPCfg.TX_LOCAL-AWTPCfg.HAVE_KEY_LINE) && (j>=0) && (j<AWTPCfg.RX_LOCAL))
					{
						TouchDataDiff = Diff[i][j] - CalNoiseTH;

						if(TouchDataDiff > 0)
						{
							if(Diff[i][j] >= USTHTOUCHSIZE)
							{
								TouchArea += 1;
							}
							
							#if 1
							i_temp = i*2;
							j_temp = j*2;
							if((CalDiff(i_temp, AW_Peak.Peak[k][0])< 3) && (CalDiff(j_temp, AW_Peak.Peak[k][1])< 3))		//内圈，直接i*Diff
							{						
								#if 1
								for(i_mid = 0; i_mid < AW_Peak.CurrentPointNum;i_mid++)
								{
									if(i_mid != k)
									{
										if((CalDiff(i_temp, AW_Peak.Peak[i_mid][0]) == 0) && (CalDiff(j_temp, AW_Peak.Peak[i_mid][1])== 0))
										{
											i_mid = 0xFE;
											break;
										}
										else if((CalDiff(i_temp, AW_Peak.Peak[i_mid][0]) == 2) && (CalDiff(j_temp, AW_Peak.Peak[i_mid][1]) == 2))
										{
											i_mid = 0xFF;
											break;
										}
										
									}
								}
								if(i_mid == 0xFF)
								{
									sum_Y[k] = sum_Y[k] + TouchDataDiff*i/3;
									sum_X[k] = sum_X[k] + TouchDataDiff*j/3;
									sum_t[k] = sum_t[k]+ TouchDataDiff/3;
									sum_tb[k] = sum_tb[k] + TouchDataDiff/3;
								}
								else if(i_mid == 0xFE)
								{
									
								}
								else 
								{
									sum_Y[k] = sum_Y[k] + TouchDataDiff *i;
									sum_X[k] = sum_X[k] + TouchDataDiff *j;
									sum_t[k] = sum_t[k]+ TouchDataDiff;
									sum_tb[k] = sum_tb[k] + TouchDataDiff;
								}
								#endif
							}
							else																					//外圈，需要判断此点与其它peak点之间的距离
							{
								#if 0
								for(i_mid = 0; i_mid < AW_Peak.CurrentPointNum;i_mid++)
								{
									if(i_mid != k)
									{
										if((CalDiff(i_temp, AW_Peak.Peak[i_mid][0]) < 3) && (CalDiff(j_temp, AW_Peak.Peak[i_mid][1]) < 3))		//有一个小于5，则退出 for 循环， 
										{
											i_mid = 0xFF;
											break;
										}
									}
								}
								
								if(i_mid == 0xFF)				//周围有其它peak点。diff值减半
								{
									sum_Y[k] = sum_Y[k] + TouchDataDiff*i/4;
									sum_X[k] = sum_X[k] + TouchDataDiff*i/4;
									sum_t[k] = sum_t[k]+ TouchDataDiff/4;
									sum_tb[k] = sum_tb[k] + TouchDataDiff/4;
								}
								else 
								{
									sum_Y[k] = sum_Y[k] + TouchDataDiff *i/2;
									sum_X[k] = sum_X[k] + TouchDataDiff *j/2;
									sum_t[k] = sum_t[k]+ TouchDataDiff/2;
									sum_tb[k] = sum_tb[k] + TouchDataDiff/2;
								}
								#endif
								
							}
							#else
							i_temp = i*2;
							j_temp = j*2;
							i_mid = 0;
							if((CalDiff(i_temp, AW_Peak.Peak[k][0])< 3) && (CalDiff(j_temp, AW_Peak.Peak[k][1])< 3))		//i*Diff
							{
								neigbour_i = (i_temp*2 - AW_Peak.Peak[k][0])/2;
								neigbour_j = (j_temp*2 - AW_Peak.Peak[k][1])/2;
								if ((neigbour_i >= 0) && (neigbour_i < AWTPCfg.TX_LOCAL-1-AWTPCfg.HAVE_KEY_LINE))
								{
									if (Diff[neigbour_i][j] > Diff[i][j])	// valley point
									{
										i_mid = 0xFF;
									}
								}
								if ((neigbour_j >= 0) && (neigbour_j < AWTPCfg.RX_LOCAL-1))
								{
									if (Diff[i][neigbour_j] > Diff[i][j])	// valley point
									{
										i_mid = 0xFF;
									}
								}								
							}
							else																					//peak
							{
								neigbour_i = (i_temp + AW_Peak.Peak[k][0])/4;
								neigbour_j = (j_temp + AW_Peak.Peak[k][1])/4;
								if ((Diff[neigbour_i][j] < Diff[i][j]) || (Diff[i][neigbour_j] < Diff[i][j]))		// outside valley point
								{
									i_mid = 0xFE;
								}
							}
								
							if(i_mid == 0xFF)
							{
								sum_Y[k] = sum_Y[k] + TouchDataDiff/2 *i;
								sum_X[k] = sum_X[k] + TouchDataDiff/2 *j;
								sum_t[k] = sum_t[k]+ TouchDataDiff/2;
								sum_tb[k] = sum_tb[k] + TouchDataDiff/2;
							}
							else if(i_mid == 0xFE)
							{
								// ignore this point,  nothing to do
							}
							else 
							{
								sum_Y[k] = sum_Y[k] + TouchDataDiff *i;
								sum_X[k] = sum_X[k] + TouchDataDiff *j;
								sum_t[k] = sum_t[k]+ TouchDataDiff;
								sum_tb[k] = sum_tb[k] + TouchDataDiff;
							}
							#endif
						}
					}
				}
			}
		}

		for(k=0;k<AW_Peak.CurrentPointNum;k++)
		{
			if(Diff[AW_Peak.Peak[k][0]/2][AW_Peak.Peak[k][1]/2] > THGROUP)
			{
				sum_X[k] = sum_X[k]*POS_PRECISION;
				t_left = (short)(sum_X[k]/sum_tb[k]) + POS_PRECISION/2;

				sum_Y[k] = sum_Y[k]*POS_PRECISION;
				t_up = (short)(sum_Y[k]/sum_t[k]) + POS_PRECISION/2;


				for(i=0;i<GroupNum;i++)
				{
					i_temp = CalDiff(t_left,AW_Frame.PointInfo[i].X);
					j_temp = CalDiff(t_up,AW_Frame.PointInfo[i].Y);
					if( i_temp< 64 && j_temp < 64)
					{
						if(i_temp >45 && j_temp > 45)
						{
							continue;
						}
						else
						{
							AW_Frame.PointInfo[i].X = (AW_Frame.PointInfo[i].X +t_left)/2;
							AW_Frame.PointInfo[i].Y = (AW_Frame.PointInfo[i].Y +t_up)/2;
							break;
						}
					}
				}
				if(i == GroupNum)		//no point distance less than one pitch
				{
					AW_Frame.PointInfo[GroupNum].X =  t_left;
					AW_Frame.PointInfo[GroupNum].Y = t_up;

					sum_t[k] = sum_t[k] >> UCTHPREFACTOR;

					if(sum_t[k] > 0x7F)
					{
						sum_t[k] = 0x7F;
					}
				//	if(TouchArea > 0x0F)
				//	{
				//		TouchArea = 0x0F;
				//	}
				//	AW_Frame.PointInfo[k].TouchArea = TouchArea;
				//	AW_Frame.PointInfo[k].TouchWeight = sum_t[k];
						
					GroupNum++;
				}
			}
		}
	}
	
	AW_Frame.PointNum = GroupNum;
	
	FilterMarginPoint();
}

/*----------------------------------------------------------------------------------
 Inputs:	
 	AW_Frame.PointInfo[i].X/Y:	Coordinate of each point in current frame, unused point should be 0x7fff;
	AW_LastFrame.PointInfo[i].X/Y:		Coordinate and ID of each point in current frame. It should be updated by AW_Frame at last

Outputs:	
	AW_Frame.PointInfo[i].PointID:	Allocated ID for each point
----------------------------------------------------------------------------------*/

void AllocatePointID(void)
{
	unsigned char  i,j,k;

	unsigned char IDUsed[MAX_POINT];
	unsigned int Delta, MinDelta;
	unsigned short Delta_tmp1, Delta_tmp2;
	unsigned char ucOffset, ucLength, k_max;
	unsigned char CurrentIDList[MAX_POINT];		// Current ID allocation plan
	unsigned char MinIndex;
	unsigned short Distance[MAX_POINT];
	

	for (i = 0; i < MAX_POINT; i++) 
	{
		AW_Frame.PointInfo[i].PointID = 0xff;	// It can be initilized in coordinate calculation process
		AW_Frame.PointInfo[i].Event = NO_EVENT;
		IDUsed[i] = 0;
		Distance[i] = 0xffff;
	}

	//allocate ID
	if (AW_Frame.PointNum > 0) 
	{
		if (AW_Frame.LastPointNum == 0) 			//no point at last frame
		{
			for (i = 0; i < AW_Frame.PointNum; i++) 
			{
				AW_Frame.PointInfo[i].PointID = i;
				IDUsed[i] = 1;
			}
		} 
		else 
		{
			if (AW_Frame.PointNum <= 3) 
			{
				if (AW_Frame.PointNum == 1) 
				{
				   ucOffset = 0; ucLength = 1; k_max = 5;
				}
				else if (AW_Frame.PointNum == 2) 
				{
				   ucOffset = 5; ucLength = 2; k_max = 20;
				}
				else if (AW_Frame.PointNum == 3) 
				{
				   ucOffset = 45; ucLength = 3; k_max = 60;
				}

				k = 0;
				MinDelta = 0xffffffff;
				MinIndex = 0xff;
				while(k < k_max) // 采用穷举法，寻找最小的sum(DeltaX^2 + DeltaY^2)的ID分配方案
				{  
					Delta = 0;
					memcpy(CurrentIDList, ucRoot + (ucLength*k) + ucOffset, ucLength);
			   
					for(i = 0; i < AW_Frame.PointNum;i ++) 
					{
						// Delta = DeltaX^2 + DeltaY^2
						if (AW_LastPointInfo[CurrentIDList[i]].X == 0x7FFF) // 如果上一帧不存在该ID（IDList[i]）则Delta = 0x3ff_ffff
						{  
							Delta += 0x3ffffff;	//(0x7fff*0x7fff>>4);
						} 
						else 
						{
							 
							Delta_tmp1 = CalDiff(AW_Frame.PointInfo[i].X,AW_LastPointInfo[CurrentIDList[i]].X);
							Delta_tmp2 = CalDiff(AW_Frame.PointInfo[i].Y,AW_LastPointInfo[CurrentIDList[i]].Y);
							
							Delta_tmp1 >>= 2;
							Delta_tmp2 >>= 2;
							
							Delta += (unsigned int)Delta_tmp1*Delta_tmp1;
							Delta += (unsigned int)Delta_tmp2*Delta_tmp2;
						}
					}
					
					if (MinDelta > Delta) 
					{
						MinDelta = Delta;
						MinIndex = k;
					//	printk("MinDelta %d, MinIndex %d\n",MinDelta,MinIndex);
						
					}
					k++;
				}

				if (MinIndex < k_max) 
				{
					memcpy(CurrentIDList, ucRoot + (ucLength*MinIndex) + ucOffset, ucLength);
					for (i = 0; i < AW_Frame.PointNum;i ++) 
					{
						AW_Frame.PointInfo[i].PointID = CurrentIDList[i];
						IDUsed[CurrentIDList[i]] = 1;
					}
				}
			} 
			else // CurrentPointNum > 3
			{	
				for(i=0;i<AW_Frame.PointNum;i++) 
				{
					for(j=0;j<MAX_POINT;j++) 
					{	// 寻找上一帧中与该点距离最近的点
						Delta_tmp1 = CalDiff(AW_Frame.PointInfo[i].X,AW_LastPointInfo[j].X)+\
								 CalDiff(AW_Frame.PointInfo[i].Y,AW_LastPointInfo[j].Y);
						if(Distance[i] > Delta_tmp1) 
						{
							AW_Frame.PointInfo[i].PointID = j;
							Distance[i] = Delta_tmp1;
						}
					}
					if (AW_Frame.PointInfo[i].PointID != 0xff) 
					{	
						IDUsed[AW_Frame.PointInfo[i].PointID] = 1;
					}
				}

				// 消除同一个ID被分配给多个点的情况
				for(i=0;i<AW_Frame.PointNum;i++) 
				{
					for(j=i+1;j<AW_Frame.PointNum;j++) 
					{
						if (AW_Frame.PointInfo[i].PointID == AW_Frame.PointInfo[j].PointID) 
						{
							if (Distance[i] > Distance[j]) 
							{
								AW_Frame.PointInfo[i].PointID = 0xff;	// 取消距离较远的点占据此ID的资格
							} 
							else 
							{
								AW_Frame.PointInfo[j].PointID = 0xff;
							}
						}
					}
				}
			}

			//为剩余未分配ID的peak点分配ID
			for(i=0;i<AW_Frame.PointNum;i++) 
			{
				if(AW_Frame.PointInfo[i].PointID == 0xff) 
				{
					for(j=0;j<MAX_POINT;j++) 
					{
						if(IDUsed[j]==0) 
						{
							IDUsed[j]=1;
							AW_Frame.PointInfo[i].PointID = j;
							break;
						}
					}
				}
			}
		}
	}
#if 0
	//allocate Event
	for(i=0;i<MAX_POINT;i++)
	{
		if(IDUsed[i]==1)		
		{
			if(AW_LastPoint[i].Event == UP_EVENT)
			{
				CurrentEvents[i]=NO_EVENT;
			}
			if(AW_LastPoint[i].Event == NO_EVENT)
			{
				CurrentEvents[i] = DOWN_EVENT;
			}
			else //if(AW_LastPoint[i].Event==DOWN_EVENT)  // DOWN or CONTACT EVENT
			{
				CurrentEvents[i] = CONTACT_EVENT;
			}
		}
		else
		{
			if((AW_LastPoint[i].Event == CONTACT_EVENT)||(AW_LastPoint[i].Event == DOWN_EVENT))
			{
				CurrentEvents[i]=UP_EVENT;
			}
			else
			{
				CurrentEvents[i]=NO_EVENT;
			}
		}
		
	}

	//allocate pointinfo event
	for(i=0;i<AW_Frame.PointNum;i++) 
	{
		for(j=0;j<MAX_POINT;j++)
		{
			if(AW_Frame.PointInfo[i].PointID == j)
			{
				AW_Frame.PointInfo[i].Event = CurrentEvents[j];
			}
		}
	}
#endif

}


#define HOLD_MAX 3
#define HOLD_TURN 1 


unsigned char LiftUpFilter(void)
{
	unsigned char i,PointNum,PointId;
	unsigned char IdUsed [MAX_POINT];
	
	unsigned short POSX_MAX;	//The max x coordinate supported by CTMP 
	unsigned short POSY_MAX;	//The max y coordinate supported by CTMP

	short DeltaX,DeltaY;
		
	unsigned char ret = 0;

	POSX_MAX = AWTPCfg.RX_LOCAL*POS_PRECISION - 1;
	POSY_MAX = AWTPCfg.TX_LOCAL*POS_PRECISION - 1;

	if(AW_Frame.PointNum == AW_Frame.LastPointNum)	//means no break line
	{
		for(i=0;i<MAX_POINT;i++)
		{
			AW_Frame.PointHoldCnt[i] = 0;
		}
		if((AW_Frame.FirstLiftUpFlag == 0) && (AW_Frame.LastPointNum != 0))
		{
			for(i=0;i<AW_Frame.PointNum;i++)
			{
				PointId = AW_Frame.PointInfo[i].PointID;
				PointDistanceX[PointId] = (short)AW_Frame.PointInfo[i].X - (short)AW_LastPointInfo[PointId].X;
				PointDistanceY[PointId] = (short)AW_Frame.PointInfo[i].Y - (short)AW_LastPointInfo[PointId].Y;

			//	printk("AW Get DIstance = %d,%d \n", PointDistanceX[PointId],PointDistanceY[PointId]);
			}
		}
	}
	else if(AW_Frame.PointNum < AW_Frame.LastPointNum)		//current point < last point  should check to avoid broke line
	{
		PointNum = AW_Frame.PointNum;

		for(i =0; i<MAX_POINT; i++)
		{
			IdUsed[i] = 0;
		}
		
		for (i = 0; i < PointNum; i++)
		{
			PointId = AW_Frame.PointInfo[i].PointID;
			IdUsed[PointId] = 1;
		}
		
		for(i=0;i<MAX_POINT;i++)
		{
			if ((IdUsed[i] == 0) && (AW_LastPointInfo[i].PointID < MAX_POINT))		// ID using: current no; pre yes
			{
				if((AW_LastPointInfo[i].X > (AWTPCfg.RX_LOCAL*POS_PRECISION - POS_PRECISION)) || 
				   (AW_LastPointInfo[i].Y > (AWTPCfg.TX_LOCAL*POS_PRECISION - POS_PRECISION)) ||
				   (AW_LastPointInfo[i].X < POS_PRECISION) ||
				   (AW_LastPointInfo[i].Y < POS_PRECISION) )	//don't hold on at margin
				{
					AW_Frame.PointHoldCnt[i] = HOLD_TURN;	
				}
				if (AW_Frame.PointHoldCnt[i] < HOLD_TURN)
				{
					AW_Frame.PointHoldCnt[i]++;
					if(AW_Frame.PointHoldCnt[i] == HOLD_TURN)
					{
						if((PointDistanceX[i] > 64) || (PointDistanceY[i] > 64))
						{
							DeltaX = (short)AW_LastPointInfo[i].X+PointDistanceX[i];
							DeltaY = (short)AW_LastPointInfo[i].Y+PointDistanceY[i];
						}
						else
						{
							DeltaX = (short)AW_LastPointInfo[i].X;
							DeltaY = (short)AW_LastPointInfo[i].Y;
						}
						
						if(DeltaX < 0)
						{
							DeltaX = 0;
						}
						if(DeltaX >= POSX_MAX)
						{
							DeltaX = POSX_MAX - 1;
						}

						
						if(DeltaY < 0)
						{
							DeltaY = 0;
						}
						if(DeltaY >= POSY_MAX)
						{
							DeltaY = POSY_MAX - 1;
						}
						AW_Frame.PointInfo[AW_Frame.PointNum].X = DeltaX;
						AW_Frame.PointInfo[AW_Frame.PointNum].Y = DeltaY;
					//	printk("AW use Distance = %d,%d \n", PointDistanceX[i],PointDistanceY[i]);
					}
					else
					{
						AW_Frame.PointInfo[AW_Frame.PointNum].X = AW_LastPointInfo[i].X;
						AW_Frame.PointInfo[AW_Frame.PointNum].Y = AW_LastPointInfo[i].Y;
					}					
					AW_Frame.PointInfo[AW_Frame.PointNum].PointID = AW_LastPointInfo[i].PointID;
					AW_Frame.PointNum ++;
					ret = 1;
					if(AWTPCfg.DEBUG_SWITCH == 1)
					{
						printk("BROKEN LINE TRIGER!!!!!! PointNum=%d, i=%d\n", AW_Frame.PointNum,i);
					}
				}
				else
				{
					if(AWTPCfg.DEBUG_SWITCH == 1)
					{
						printk("Point Miss Two Times %d\n",AW_Frame.PointHoldCnt[i]);
					}
				}

				
			}	
		}
	}

	return ret;
}


void flying_process(void)
{
	unsigned char i,pointID;
	unsigned short DeltaX,DeltaY,Distance = 0;
	

	for(i=0;i<MAX_POINT;i++)
	{
		if(0xFF == AW_LastPointInfo[i].PointID)
		{
			pointDistance[i] = 0;
			flycnt[i] = 2;
		}
	}
	
	for(i=0;i<AW_Frame.PointNum;i++)
	{
		pointID = AW_Frame.PointInfo[i].PointID;
		if(AW_LastPointInfo[pointID].X != 0x7FFF)
		{
			DeltaX = CalDiff(AW_Frame.PointInfo[i].X, AW_LastPointInfo[pointID].X);
			DeltaY = CalDiff(AW_Frame.PointInfo[i].Y, AW_LastPointInfo[pointID].Y);
		}
		else
		{
			DeltaX = 0;
			DeltaY = 0;
		}
		Distance = DeltaX + DeltaY;
	
		if(flycnt[pointID] > 0)
		{
			pointDistance[pointID] = Distance;
			flycnt[pointID]--;
			continue;
		}
		
		if( (pointDistance[pointID] > AWTPCfg.MOVING_TH) && (Distance > AWTPCfg.MOVING_TH))
		{
			pointDistance[pointID] = Distance;
			continue;
		}
		if( Distance > (pointDistance[pointID]+ AWTPCfg.MOVING_ACCELER))
		{
			if(pointDistance[pointID] > AWTPCfg.MOVING_ACCELER)
			{
				pointDistance[pointID] = Distance;
				continue;
			}
		}
		if(Distance < (pointDistance[pointID] << 4))
		{
			pointDistance[pointID] = Distance;
			continue;
		}

		if(Distance > AWTPCfg.FLYING_TH)			//flying
		{	
			//printk("FLYING_TH TRIGER!!!!!! num = %d,id = %d, \n",i,AW_Frame.RptPoint[i].PointID );
			if(i < 4)
			{
				AW_Frame.PointInfo[i].X = AW_Frame.PointInfo[i+1].X;
				AW_Frame.PointInfo[i].Y = AW_Frame.PointInfo[i+1].Y;
				AW_Frame.PointInfo[i].PointID = AW_Frame.PointInfo[i+1].PointID;
			}
			else	//last point
			{
				AW_Frame.PointInfo[i].X = 0x7FFF;
				AW_Frame.PointInfo[i].Y = 0x7FFF;
				AW_Frame.PointInfo[i].PointID = 0xFF;
			}
			AW_Frame.PointNum--;
			i--;
			pointDistance[pointID] = 0;
			continue;
		}
		pointDistance[pointID] = Distance;
		continue;
	}

}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// Filter for touch point
// Calculate the difference between two frames, the bigger difference the smaller attenuation, vice versa.
// Make sure when the difference is small the reported coordinate is stable,
// While the difference is big, the reported coordinate can catch up quickly.
//----------------------------------------------------------------------------------

void preFilterTouchPoint(void)
{
	unsigned short TouchDiff; 
	signed short TouchDelta;
	unsigned short temp,temp1;
	unsigned char i,j,k;

	unsigned short POSX_MAX;
	unsigned short POSY_MAX;
	unsigned char ret;

	POSX_MAX = AWTPCfg.RX_LOCAL*POS_PRECISION -1;
	POSY_MAX = AWTPCfg.TX_LOCAL*POS_PRECISION -1;

	//clear RptPoint
	for (i=0; i<MAX_POINT; i++)
	{
		AW_Frame.RptPoint[i].X = 0x7fff;
		AW_Frame.RptPoint[i].Y = 0x7fff;
		AW_Frame.RptPoint[i].Event = NO_EVENT;
		AW_Frame.RptPoint[i].PointID = 0xff;
	}
#if 0
	if(CurrentPointNum == 0)
	{
		if(LastPointNum > 0)		//need process UP_EVENT
		{
			for (i=0; i<MAX_POINT; i++)
			{
				AW_Frame.RptPoint[i].Event = AW_Frame.PointInfo[i].Event;
			}
			AW_Frame.FirstLiftUpFlag = 0;
		}
		return;
	}
#endif
	if (AW_Frame.LastPointNum == 0)
	{//new touch RptPoint = PointInfo
		for (i = 0; i < AW_Frame.PointNum; i++)			
		{
			AW_Frame.RptPoint[i].X = AW_Frame.PointInfo[i].X;
			AW_Frame.RptPoint[i].Y = AW_Frame.PointInfo[i].Y;
			AW_Frame.RptPoint[i].PointID = AW_Frame.PointInfo[i].PointID;
			AW_Frame.RptPoint[i].Event = AW_Frame.PointInfo[i].Event;
			AW_Frame.RptPoint[i].X <<=  4;
			AW_Frame.RptPoint[i].Y <<=  4;
		}
	}   
	else
	{
		j = 0;
		// reorder the PointInfo, make old ID point first
		for (i = 0; i < AW_Frame.PointNum; i++)
		{
			if (AW_LastPoint_Filter[AW_Frame.PointInfo[i].PointID].X != 0x7fff)		// This ID has exist in last frame
			{
				AW_Frame.RptPoint[j].X = AW_Frame.PointInfo[i].X;
				AW_Frame.RptPoint[j].Y = AW_Frame.PointInfo[i].Y;
				AW_Frame.RptPoint[j].PointID = AW_Frame.PointInfo[i].PointID;
				AW_Frame.RptPoint[j].Event = AW_Frame.PointInfo[i].Event;
				j++;
					
			}
		}
		for (i = 0; i < AW_Frame.PointNum; i++)
		{
			if (AW_LastPoint_Filter[AW_Frame.PointInfo[i].PointID].X == 0x7fff)		// No match ID found
			{
				AW_Frame.RptPoint[j].X = AW_Frame.PointInfo[i].X;
				AW_Frame.RptPoint[j].Y = AW_Frame.PointInfo[i].Y;
				AW_Frame.RptPoint[j].PointID = AW_Frame.PointInfo[i].PointID;
				AW_Frame.RptPoint[j].Event = AW_Frame.PointInfo[i].Event;
				j++;
			}
		}
		
		for (k = 0; k<AW_Frame.PointNum; k++)
		{
			AW_Frame.RptPoint[k].X <<=  4;
			AW_Frame.RptPoint[k].Y <<=  4;
			if (AW_LastPoint_Filter[AW_Frame.RptPoint[k].PointID].X != 0x7fff)
			{
				TouchDiff = CalDiff(AW_Frame.RptPoint[k].X, AW_LastPoint_Filter[AW_Frame.RptPoint[k].PointID].X)
					+ CalDiff(AW_Frame.RptPoint[k].Y, AW_LastPoint_Filter[AW_Frame.RptPoint[k].PointID].Y);
				//TouchDiff <<= 4;
		
				if (TouchDiff < (THDIFF >>3))		// x+y < 20
				{
					temp = 4;		// 16
				}
				else if (TouchDiff < (THDIFF >>2))		// x+y < 40
				{
        			temp = 3;		// 8
				}
				else if (TouchDiff < (THDIFF >>1))		// x+y < 80
				{
        			temp = 2;		// 4
				}
				else if (TouchDiff < THDIFF)		// x+Y < 160
				{
					temp = 1;   		// 2
				}
				else
				{
					temp = 0;			// 1
				}
				//temp = 0;
				if(AW_Frame.FirstLiftUpFlag == 1)
				{
					temp = 1;
				}
		
				TouchDelta = (signed short)AW_Frame.RptPoint[k].X - (signed short)AW_LastPoint_Filter[AW_Frame.RptPoint[k].PointID].X;
				if(((((POSX_MAX<<4) - AW_Frame.RptPoint[k].X) < (32<<4))&&(TouchDelta > 0)) || ((AW_Frame.RptPoint[k].X < (32<<4)) &&(TouchDelta < 0)))
				{
					temp1 = 1;
				}
				else
				{
					temp1 = temp;
				}
				
				TouchDelta -= TouchDelta / (1 << temp1);  // 确保算术右移
				
				if ((temp1 != 0) && (TouchDelta != 0))
				{
					if(AW_Frame.RptPoint[k].X > TouchDelta)
					{
						AW_Frame.RptPoint[k].X -= TouchDelta;
					}
					else
					{
						AW_Frame.RptPoint[k].X = 0;
					}
				}
				
				TouchDelta = (signed short)AW_Frame.RptPoint[k].Y - (signed short)AW_LastPoint_Filter[AW_Frame.RptPoint[k].PointID].Y;
				if(((((POSY_MAX<<4) - AW_Frame.RptPoint[k].Y) < (32<<4))&&(TouchDelta > 0)) || ((AW_Frame.RptPoint[k].Y < (32<<4)) &&(TouchDelta < 0)))
				{
					temp1 = 1;
				}
				else
				{
					temp1 = temp;
				}
				{
					TouchDelta -= TouchDelta / (1 << temp1);  // 确保算术右移
				}
				if ((temp1 != 0) && (TouchDelta != 0))
				{
					if(AW_Frame.RptPoint[k].Y > TouchDelta)
					{
						AW_Frame.RptPoint[k].Y -= TouchDelta;
					}
					else
					{
						AW_Frame.RptPoint[k].Y = 0;
					}
				}
			}
		}
	}

	// update AW_LastPoint_Filter
	for (k = 0; k<MAX_POINT; k++)
	{
		AW_LastPoint_Filter[k].X = 0x7FFF;
		AW_LastPoint_Filter[k].Y = 0x7FFF;
	}

	for (k = 0; k<AW_Frame.PointNum; k++)
	{
		AW_LastPoint_Filter[AW_Frame.RptPoint[k].PointID].X = AW_Frame.RptPoint[k].X;
		AW_LastPoint_Filter[AW_Frame.RptPoint[k].PointID].Y = AW_Frame.RptPoint[k].Y;

	}

	for (k = 0; k<AW_Frame.PointNum; k++)
	{
		AW_Frame.RptPoint[k].X = AW_Frame.RptPoint[k].X >> 4;
		AW_Frame.RptPoint[k].Y = AW_Frame.RptPoint[k].Y >> 4;
	}	
}

//----------------------------------------------------------------------------------
// Mapping coordinate to screen pixel
// x = x * AWTPCfg.MAPPING_MAX_X/(RX_NUM*POS_PRECISION)
// AWTPCfg.MAPPING_MAX_X is pixel in X， RX_NUM is collum number， POS_PRECISION is granularity between two collum。
// multiply 64 then divide 64 is to improve the calculation prescision.
//----------------------------------------------------------------------------------
unsigned short XMapping(unsigned short x)
{

	unsigned int pixel;

	pixel = (unsigned int)x * AWTPCfg.K_X;		// AWTPCfg.K_X为映射系数 = (AWTPCfg.MAPPING_MAX_X-1)*256/(RX_NUM*POS_PRECISION)
	pixel = pixel >> 8;
	if(pixel == 0)
	{
         pixel =1;
	}

	if(pixel >= AWTPCfg.MAPPING_MAX_X) 
	{
		pixel = (AWTPCfg.MAPPING_MAX_X - 1);
	}

	return (unsigned short) pixel;
}

unsigned short YMapping(unsigned short y)
{

	unsigned int pixel;

	pixel = (unsigned int)y * AWTPCfg.K_Y;		// AWTPCfg.K_Y为映射系数 = (AWTPCfg.MAPPING_MAX_Y-1)*256/(TX_NUM*POS_PRECISION)
	pixel = pixel >> 8;
	if(pixel == 0)
	{
         pixel =1;
	}

	if(pixel >= AWTPCfg.MAPPING_MAX_Y) 
	{
		pixel = (AWTPCfg.MAPPING_MAX_Y - 1);
	}

	return (unsigned short) pixel;
}

void PointStable(void)
{
	unsigned char i,pointID;
	short Temp;
	unsigned short DeltaX,DeltaY,Distance = 0;
	
	if(AW_Frame.PointNum == 0)
	{
		return;
	}
	
	//stable process
	for(i=0;i<AW_Frame.PointNum;i++)
	{
		pointID = AW_Frame.RptPoint[i].PointID;
		if(pointID != 0xFF && AW_LastPoint[pointID].X != 0x7FFF)	//last and current frame all have this point
		{
			DeltaX = CalDiff(AW_Frame.RptPoint[i].X, AW_LastPoint[pointID].X);
			DeltaY = CalDiff(AW_Frame.RptPoint[i].Y, AW_LastPoint[pointID].Y);
			
			Distance = DeltaX + DeltaY;
			if (Distance < (MIN_DELTA_X + MIN_DELTA_Y))
			{
				if(PointLastCnt[pointID] > 120)
				{
					Temp = AW_LastPoint[pointID].X;
					Temp += (DeltaX >> MIN_DELTA_STEP);

					AW_Frame.RptPoint[i].X = Temp;

					Temp = AW_LastPoint[pointID].Y;
					Temp += (DeltaY >> MIN_DELTA_STEP);

					AW_Frame.RptPoint[i].Y = Temp;

					PointLastCnt[pointID] = 0;
				}
				else
				{
					PointLastCnt[pointID]++;
					AW_Frame.RptPoint[i].X = AW_LastPoint[pointID].X;
					AW_Frame.RptPoint[i].Y = AW_LastPoint[pointID].Y; 
				}	
			}
			else
			{
				PointLastCnt[i] = 0;
			}
		}
	}
}


void PointOutput(void)
{
	unsigned char i,pointID;

#ifndef FLY_BROKEN_LINE_METHORD
	//clear last point
	for(i=0;i<MAX_POINT;i++)
	{
		AW_LastPoint[i].X = 0x7FFF;
		AW_LastPoint[i].Y = 0x7FFF;
		AW_LastPoint[i].Event = NO_EVENT;
		AW_LastPoint[i].PointID = 0xFF;

		AW_LastPointInfo[i].X = 0x7FFF;
		AW_LastPointInfo[i].Y = 0x7FFF;
		AW_LastPointInfo[i].Event = NO_EVENT;
		AW_LastPointInfo[i].PointID = 0xFF;
	}

	//copy current non-filter point to last pointinfo in ID ORDER
	for (i=0; i<AW_Frame.PointNum; i++)
	{
		pointID = AW_Frame.PointInfo[i].PointID;
		if(pointID != 0xFF)
		{
			AW_LastPointInfo[pointID].X = AW_Frame.PointInfo[i].X;
			AW_LastPointInfo[pointID].Y = AW_Frame.PointInfo[i].Y;
			AW_LastPointInfo[pointID].PointID = pointID;
		}
	}
	
	//copy current filter point to last point in ID ORDER
	for (i=0; i<AW_Frame.PointNum; i++)
	{
		pointID = AW_Frame.RptPoint[i].PointID;
		if(pointID != 0xFF)
		{
			AW_LastPoint[pointID].X = AW_Frame.RptPoint[i].X;
			AW_LastPoint[pointID].Y = AW_Frame.RptPoint[i].Y;
			AW_LastPoint[pointID].PointID = pointID;
		}
	}

	AW_Frame.LastPointNum = AW_Frame.PointNum;
	
#endif	

	for (i=0; i<AW_Frame.PointNum; i++)
	{
		AW_Frame.RptPoint[i].X = XMapping(AW_Frame.RptPoint[i].X);
		AW_Frame.RptPoint[i].Y = YMapping(AW_Frame.RptPoint[i].Y);
	}
}

void AW5306_Sleep(void)
{
	I2C_WriteByte(SA_PAGE,0x00);
	I2C_WriteByte(SA_SCANMD,0x00);			//scan off
	I2C_WriteByte(SA_CTRL,0x00);
}

void AW5306_TP_Init(void)
{
	AW5306_User_Init();
}

void AW5306_TP_Reinit(void)
{
	TP_Init();
}

char AW5306_TouchProcess(void)
{
	char BaseError = 1;
	char ret = 0;
	short buf[11];
	char i;

	if(Calibration_Flag == 0)
	{
		TP_Calibration();

		TP_Init();
		Calibration_Flag = 1;
		return 1;
	}
	
	if(CollectTPData() == 0)		//not collect data
	{
		return 1;
	}
	
#ifdef RAW_DATA_FILTER
	RawDataFilter();
#endif
	//RawDataBlur();
	SearchPeak();
	if(AWTPCfg.DEBUG_SWITCH == 1)
	{
		if(AW_Peak.CurrentPointNum >= 2)
		{
			write_Flag = 1;
		}
		else
		{
			write_Flag = 0;
		}
		if(write_Flag == 1)
		{
			nvram_write("data/awdiff", &AW_Peak.CurrentPointNum, 1, fileoffset);
			fileoffset += 1;
		}
	}	
	BaseError = BaseProcess();
	if(BaseError != 1)
	{
		if(1 == AWTPCfg.ESD_PROTECT)
		{
			ESDCheck();

			ESDFilterProcess();
		}
		
		CalculateCoordinate();
		if(AWTPCfg.DEBUG_SWITCH == 1)
		{
			if(write_Flag == 1)
			{	
				buf[0] = AW_Frame.PointNum;
				buf[1] = AW_Frame.PointInfo[0].X;
				buf[2] = AW_Frame.PointInfo[0].Y;
				buf[3] = AW_Frame.PointInfo[1].X;
				buf[4] = AW_Frame.PointInfo[1].Y;

				buf[5] = Diff[AW_Peak.Peak[0][0]/2][AW_Peak.Peak[0][1]/2];
				buf[6] = Diff[AW_Peak.Peak[1][0]/2][AW_Peak.Peak[1][1]/2];
				buf[7] = Diff[AW_Peak.Peak[2][0]/2][AW_Peak.Peak[2][1]/2];
				buf[8] = Diff[AW_Peak.Peak[3][0]/2][AW_Peak.Peak[3][1]/2];
				buf[9] = Diff[AW_Peak.Peak[4][0]/2][AW_Peak.Peak[4][1]/2];
				buf[10] = AW_Peak.Peak[1][1]/2;

			
				nvram_write("data/awdiff", (char *)&buf, 22, fileoffset);
				fileoffset += 22;
			}	
		}
		AllocatePointID();

		AW_Frame.FirstLiftUpFlag = LiftUpFilter();
			
		flying_process();

		preFilterTouchPoint();
		//BlurParamSelect();

		PointStable();
		PointOutput();
		
	}
	ret = 1;
	
	return ret;
}


void AW5306_ChargeMode(char mode)
{
	if((mode == 1) && (ChargePlugIn == 0))
	{
		ChargePlugIn = 1;
		BIGAREA_PEAK_TH = 120;
		THPEAK = 100;
		THGROUP = 110;
	}
	else if((mode == 0) && (ChargePlugIn == 1))
	{
		ChargePlugIn = 0;
		BIGAREA_PEAK_TH = 90;
		THPEAK = 70;
		THGROUP = 80;
	}
}

unsigned char AW5306_GetPointNum(void)
{
	return AW_Frame.PointNum;
}

unsigned char AW5306_GetPeakNum(void)
{
	return AW_Peak.CurrentPointNum;
}

char AW5306_GetPoint(int *x,int *y, int *id, int *event,char Index)
{
	unsigned short i;

	i = Index;
	
	*x = AW_Frame.RptPoint[i].X;
	*y = AW_Frame.RptPoint[i].Y;
	*id = AW_Frame.RptPoint[i].PointID;
	*event = AW_Frame.RptPoint[i].Event;

//	printk("AW %d  %d \n", XMapping(AW_Frame.PointInfo[i].X),YMapping(AW_Frame.PointInfo[i].Y));
	
	return 1;
}

void AW5306_GetBase(unsigned short *data, char x,char y)
{
	unsigned short i,j;
	
	i = x;
	j = y;
	
	*data = AW_Base.Base[i][j];
}

void AW5306_GetDiff(short *data, char x,char y)
{
	unsigned short i,j;
	
	i = x;
	j = y;
	
	*data = adbDiff[i][j];
}

char AW5306_GetPeak(unsigned char *x,unsigned char *y,unsigned char Index)
{
	unsigned char i;

	i = Index;
	*x = AW_Peak.Peak[i][0];
	*y = AW_Peak.Peak[i][1];
	return 1;
}

char AW5306_GetNegPeak(unsigned char *x,unsigned char *y,unsigned char Index)
{
	unsigned char i;

	i = Index;
	
	*x = AW_Peak.NegPeak[i][0];
	*y = AW_Peak.NegPeak[i][1];
	return 1;
}

char AW5306_GetCalcPoint(unsigned short *x,unsigned short *y,unsigned char Index)
{
	unsigned char i;

	i = Index;
	
	*x = AW_Frame.PointInfo[i].X;
	*y = AW_Frame.PointInfo[i].Y;
	return 1;
}
