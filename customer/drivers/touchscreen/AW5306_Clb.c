/**************************************************************************
*  AW5306_Clb.cpp
* 
*  AW5306 Driver code version 1.0
*  Touch Screen calibration
* 
*  Create Date : 2012/07/17
* 
*  Modify Date : 
*
*  Create by   : YaoWei
* 
**************************************************************************/
#include <linux/time.h>
#include <linux/string.h>
#include "AW5306_Reg.h"
#include "AW5306_Drv.h"
#include "AW5306_userpara.h"

//#define GAIN_CLB_SEPARATE

unsigned char OffsetInitialValue = 0xf;	// Initial OFFSET setting for CAC calibration
unsigned char GainInitialValue = 0xf;	// Initial GAIN setting for CAC calibration
unsigned char TXOffsetStart = 0x22;	// TXOFFSET Initial setting for OFFSET calibration
unsigned char RxOffsetClbEn = 1;
unsigned char TxOffsetClbEn = 0;
short DiffBuff[NUM_TX][NUM_RX];

unsigned char  ClbErrorCode;	// Calibration result output. 
				// 0: no error; 1: CAC error; 2: OFFSET error; 3: GAIN error

extern AW5306_UCF	AWTPCfg;
extern STRUCTCALI	AW_Cali;
unsigned char 	RX_START;

extern void AW_Sleep(unsigned int msec);

extern int I2C_WriteByte(unsigned char addr, unsigned char data);
extern unsigned char I2C_ReadByte(unsigned char addr);
extern unsigned char I2C_ReadXByte( unsigned char *buf, unsigned char addr, unsigned short len);


void AW5306_CLB_GETADC(void)
{
	unsigned int i;
	unsigned int j;
	unsigned char reg_buf[512];
	unsigned char regdat;
	unsigned short captemp;

	I2C_WriteByte(SA_SCANMD, 0x07);
	while(1)
	{
		regdat = I2C_ReadByte(SA_SCANMD);
		if ( (regdat&0x01) == 0x00 ) 
		{ break; }
		AW_Sleep(2);
	}

	
	I2C_WriteByte(SA_SCANMD, 0x07);			//need read two times
	while(1)
	{
		regdat = I2C_ReadByte(SA_SCANMD);
		if ( (regdat&0x01) == 0x00 ) 
		{ break; }
		AW_Sleep(2);
	}

	I2C_WriteByte(SA_ADDRH,  0x00);
	I2C_WriteByte(SA_ADDRL,  0x00);
	I2C_ReadXByte(reg_buf,SA_RAWDATA, AWTPCfg.TX_LOCAL*AWTPCfg.RX_LOCAL*2  );

	for (i=0; i<AWTPCfg.TX_LOCAL; i++)
	{
		for (j=0; j<AWTPCfg.RX_LOCAL; j++)
		{
			captemp = reg_buf[(i*AWTPCfg.RX_LOCAL +j)*2];
			captemp <<= 8;
			captemp += reg_buf[(i*AWTPCfg.RX_LOCAL +j)*2 + 1];
			DiffBuff[i][j] = captemp;
		}
	}
}

long AW5306_CLB_GETSUM(unsigned char row, unsigned char col)
{
	unsigned char i,j;
	long Sum;
	unsigned char CalTxNum;

	if (AWTPCfg.HAVE_KEY_LINE == 1)
	{
		CalTxNum = AWTPCfg.TX_LOCAL - 1;
	}
	else
	{
		CalTxNum = AWTPCfg.TX_LOCAL;
	}

	Sum = 0;
	if (row != 0xff)
	{
		for (i = 0; i < AWTPCfg.RX_LOCAL; i++)
		{
			Sum += DiffBuff[row][i];
		}
	}
	else if (col != 0xff)
	{
		for (i = 0; i < CalTxNum; i++)
		{
			Sum += DiffBuff[i][col];
		}
	}
	else
	{
		for (i = 0; i < CalTxNum; i++)
		{
			for (j = 0; j < AWTPCfg.RX_LOCAL; j++)
			{
				Sum += DiffBuff[i][j];
			}
		}
	}

	return Sum;
}

long AW5306_CLB_GETAVE(unsigned char row, unsigned char col)
{
	unsigned char i,j;
	long Sum;
	short Cnt;
	unsigned char CalTxNum;

	if (AWTPCfg.HAVE_KEY_LINE == 1)
	{
		CalTxNum = AWTPCfg.TX_LOCAL - 1;
	}
	else
	{
		CalTxNum = AWTPCfg.TX_LOCAL;
	}

	Sum = 0;
	Cnt = 0;
	if (row != 0xff)
	{
		for (i = 0; i < AWTPCfg.RX_LOCAL; i++)
		{
			if (!((AWTPCfg.KeyLineValid[i] == 0) && (row == AWTPCfg.TX_LOCAL-1) && (AWTPCfg.HAVE_KEY_LINE == 1)))
			{
				Sum += DiffBuff[row][i];
				Cnt++;
			}
		}
	}
	else if (col != 0xff)
	{
		
		for (i = 0; i < CalTxNum; i++)
		{
			Sum += DiffBuff[i][col];
			Cnt++;
		}
	}
	else
	{
		for (i = 0; i < CalTxNum; i++)
		{
			for (j = 0; j < AWTPCfg.RX_LOCAL; j++)
			{
				Sum += DiffBuff[i][j];
				Cnt++;
			}
		}
	}

	Sum = Sum / Cnt;
	return Sum;
}


void AW5306_CLB_GETDIFF(void)
{
	unsigned char i,j;
	unsigned short DataBuf1[NUM_TX][NUM_RX];
	unsigned short DataBuf2[NUM_TX][NUM_RX];
	// Drive TX with 4V
	I2C_WriteByte(SA_DRV_VLT, 8 );
	AW_Sleep(10);
	
	AW5306_CLB_GETADC();

	for (i=0; i < AWTPCfg.TX_LOCAL; i++)
	{
		for (j=0; j < AWTPCfg.RX_LOCAL; j++) {
			DataBuf1[i][j] = DiffBuff[i][j];
		}
	}

	// Drive TX with default level
	I2C_WriteByte(SA_DRV_VLT, 0x0 );
	AW5306_CLB_GETADC();
	for (i=0; i < AWTPCfg.TX_LOCAL; i++)
	{
		for (j=0; j < AWTPCfg.RX_LOCAL; j++) 
		{ 
			DataBuf2[i][j] = DiffBuff[i][j];
			DiffBuff[i][j] = DataBuf1[i][j] - DataBuf2[i][j];
		}
	}
}

void AW5306_TXDLY_CLB(void)
{
	long Sum;
	long MinSum [NUM_TX];
	unsigned char TclkDlyClb [NUM_TX];
	unsigned char i,j;

	for(i=0;i<11;i++)
	{
		I2C_WriteByte(SA_TXOFFSET0+i,0x22);
	}
	for(i=0;i<6;i++)
	{
		I2C_WriteByte(SA_RXOFFSET0+i,0x33);
	}
	for(i=0;i<NUM_TX;i++)
	{
		I2C_WriteByte(SA_TXCAC0+i,0x30);
	}
	for(i=0;i<NUM_RX;i++)
	{
		I2C_WriteByte(SA_RXCAC0+i,0x60);
	}
	for(i=0;i<NUM_TX;i++)
	{
		I2C_WriteByte(SA_TXADCGAIN0+i, 0x8);
	}
	
	for (i = 0; i< AWTPCfg.TX_LOCAL; i++)
	{
		MinSum[i] = 0x0FFFFFFF;
	}
	for (i = 0; i< 5; i++)
	{
		I2C_WriteByte(SA_TCLKDLY, i);
		AW5306_CLB_GETADC();
		for (j = 0; j < AWTPCfg.TX_LOCAL; j++)
		{
			Sum = AW5306_CLB_GETSUM(j, 0xff);
			if (Sum < MinSum[j])
			{
				MinSum[j] = Sum;
				TclkDlyClb[j] = i;
			}
		}
	}
	for (j = 0; j < AWTPCfg.TX_LOCAL; j++)
	{
		I2C_WriteByte(SA_TXADCGAIN0+j, TclkDlyClb[j]*32+0xf);
	}
	I2C_WriteByte(SA_TCLKDLY, 0);
}

void AW5306_CAC_CLB(void)
{
	unsigned char i, Cac, StdCac;
	unsigned short RxCac[NUM_RX];
	unsigned short TxCac[NUM_TX];
	unsigned char Col;
	long MaxSum, Sum;
	short AveScreen, AveCol, AveRow;
	// Set OFFSET to an fixed value

	StdCac = 0;
	
	for(i=0;i<11;i++)
	{
		I2C_WriteByte(SA_TXOFFSET0+i, OffsetInitialValue*16+OffsetInitialValue);
	}
	for(i=0;i<6;i++)
	{
		I2C_WriteByte(SA_RXOFFSET0+i,0);
	}

	for(i=0;i<NUM_TX;i++)
	{
		I2C_WriteByte(SA_TXCAC0+i,0);
	}

	// Find inflection point of ADC output to CAC configuration
	MaxSum = 0;
	for (Cac = 0; Cac < 250; Cac = Cac+4)		// tmp code, should be 0~252
	{
		for(i=0;i<NUM_RX;i++)
		{
			I2C_WriteByte(SA_RXCAC0+i, Cac);
		}
		AW5306_CLB_GETDIFF();
		Sum = AW5306_CLB_GETSUM(0xff, 0xff);
		if (Sum > MaxSum)
		{
			MaxSum = Sum;
			StdCac = Cac;
		}
		if (StdCac*(AWTPCfg.CacMultiCoef -2)/2 > 255)
		{
			break;
		}
	}

	// Setting CAC to 1.5x of inflection point
	for(i=0;i<NUM_TX;i++)
	{
		I2C_WriteByte(SA_TXCAC0+i,StdCac );
	}
	for(i=0;i<NUM_RX;i++)
	{
		I2C_WriteByte(SA_RXCAC0+i, StdCac*(AWTPCfg.CacMultiCoef -2)/2 );
	}
	
	// Balance the screen by tuning the RXCAC
	AW5306_CLB_GETDIFF();
	//Sum = AW5306_CLB_GETSUM(0xff, 0xff);
	AveScreen = AW5306_CLB_GETAVE(0xff,0xff);
	if (AveScreen == 0)
	{
		ClbErrorCode |= 0x1;
	}
	else {
		for (Col = 0; Col < AWTPCfg.RX_LOCAL; Col++)
		{
			//Sum = AW5306_CLB_GETSUM(0xff, Col);
			AveCol = AW5306_CLB_GETAVE(0xff,Col);
			RxCac[Col] = StdCac*(AWTPCfg.CacMultiCoef-2)/2 + (StdCac*AWTPCfg.CacMultiCoef/2)*(AveCol - AveScreen)/AveScreen;
			if (RxCac[Col] > 255)
			{
				I2C_WriteByte(SA_RXCAC0+Col+RX_START, 0xFF);
			}
			else
			{
				I2C_WriteByte(SA_RXCAC0+Col+RX_START, RxCac[Col]);
			}
		}	
	}

	// Balance the screen by tuning the TXCAC
	AW5306_CLB_GETDIFF();
	//Sum = AW5306_CLB_GETSUM(0xff, 0xff);
	AveScreen = AW5306_CLB_GETAVE(0xff,0xff);
	if (AveScreen == 0)
	{
		ClbErrorCode |= 0x1;
	}
	else {
		for (i = 0; i < AWTPCfg.TX_LOCAL; i++)
		{
			AveRow = AW5306_CLB_GETAVE(i,0xff);
			TxCac[i] = StdCac+(StdCac *(AveRow - AveScreen)/AveScreen);
			if(TxCac[i] > 255)
			{
				I2C_WriteByte(SA_TXCAC0+i, 0xFF);
			}
			else
			{
				I2C_WriteByte(SA_TXCAC0+i, TxCac[i]);
			}
		}	
	}

}

void AW5306_OFFSET_CLB(void)
{
	unsigned char i, offset;
	unsigned char flag [32];	// max(NUM_TX, NUM_RX) must less than 32
	unsigned char RxOffsetCfg [16];	// NUM_RX must less than 16
	unsigned char TxOffsetCfg [32];	// NUM_TX must less than 32
	short Ave;
	short MinAveDiff[32];

	// RXOFFSET calibration
	if(1 == RxOffsetClbEn)
	{
		for (i = 0; i < AWTPCfg.RX_LOCAL; i++)
		{
			flag[i] = 0;
			RxOffsetCfg[i] = 15;
			MinAveDiff[i] = 0xfff;
		}

		for(i=0;i<(NUM_TX+1)/2;i++)
		{
			I2C_WriteByte(SA_TXOFFSET0+i,TXOffsetStart);
		}
		for(offset = 0; offset < 16; offset++)
		{
			for(i=0;i<(NUM_RX+1)/2;i++)
			{
				I2C_WriteByte(SA_RXOFFSET0+i,offset*16 + offset);
			}
			AW5306_CLB_GETADC();
			for (i = 0; i < AWTPCfg.RX_LOCAL; i++)
			{
				//Sum = AW5306_CLB_GETSUM(0xff, i);	// get sum of col i
				Ave = AW5306_CLB_GETAVE(0xff,i);
				Ave = Ave - AWTPCfg.OffsetClbExpectedMin;
				
				if(ABS(Ave) < MinAveDiff[i])
				{
					MinAveDiff[i] = ABS(Ave);
					RxOffsetCfg[i] = offset;
					printk("OFFSET cali %d,MinAve = %d,RxOffset=%d\n", i,Ave,offset);
				}
			}

		}
		for(i=0;i<NUM_RX;i++)
		{
			flag[i] = 0;
		}
		for(i=0;i<AWTPCfg.RX_LOCAL;i++)
		{
			flag[i+RX_START] = RxOffsetCfg[i];
		}
		for (i = 0; i < (NUM_RX+1)/2; i++)
		{
			I2C_WriteByte(SA_RXOFFSET0+i, 16*flag[2*i+1]  + flag[2*i] );
			printk("OFFSET cali flag = %x\n", 16*flag[2*i+1]  + flag[2*i]);
		}
	}

	// TXOFFSET calibration
	if(1 == TxOffsetClbEn)
	{
		for (i = 0; i < AWTPCfg.TX_LOCAL; i++)
		{
			flag[i] = 0;
			MinAveDiff[i] = 0xFFF;
		}

		for(offset = 0; offset < 16; offset++)
		{
			for(i=0;i<(AWTPCfg.TX_LOCAL+1)/2;i++)
			{
				I2C_WriteByte(SA_TXOFFSET0+i, offset*16 + offset);
			}
			AW5306_CLB_GETADC();
			for (i = 0; i < AWTPCfg.TX_LOCAL; i++)
			{
				Ave = AW5306_CLB_GETAVE(i, 0xff);	// get sum of row i
				Ave = Ave - AWTPCfg.OffsetClbExpectedMin;
				if((Ave > 0) && (Ave < MinAveDiff[i]))
				{
					MinAveDiff[i] = Ave;
					TxOffsetCfg[i] = offset;
				}
			}
		}
		for (i = 0; i < (AWTPCfg.TX_LOCAL+1)/2; i++)
		{
			I2C_WriteByte(SA_TXOFFSET0+i, 16*TxOffsetCfg[2*i+1]  + TxOffsetCfg[2*i] );
		}
	}
	
	// Check the result of OFFSET calibration
	AW5306_CLB_GETADC();

	for (i = 0; i < AWTPCfg.TX_LOCAL-AWTPCfg.HAVE_KEY_LINE; i++)
	{
		//Sum = AW5306_CLB_GETSUM(i, 0xff);	// get sum of row i
		Ave = AW5306_CLB_GETAVE(i, 0xff);
		if ((Ave <= AWTPCfg.OffsetClbExpectedMin-500) || (Ave >= AWTPCfg.OffsetClbExpectedMax+500 ))
		{
			printk("OFFSET LINE CHECKING FAIL TX %d,aVe = %d, flag = %d\n", i,Ave,flag[i]);
			//ClbErrorCode |= 0x2;
		}
	}
	for (i = 0; i < AWTPCfg.RX_LOCAL; i++)
	{
		//Sum = AW5306_CLB_GETSUM(0xff, i);	// get sum of col i
		Ave = AW5306_CLB_GETAVE(0xff,i);
		if ((Ave <= AWTPCfg.OffsetClbExpectedMin-500) || (Ave >= AWTPCfg.OffsetClbExpectedMax+500 ))
		{
			printk("OFFSET LINE CHECKING FAIL RX %d,ave=%d, flag = %d\n", i,Ave,flag[i]);
			//ClbErrorCode |= 0x2;
		}
	}
}


#ifdef GAIN_CLB_SEPARATE
void AW5306_GAIN_CLB(void)
{
	unsigned char Gain[NUM_TX],Flag[NUM_TX],i;
	unsigned char reg_data;
	unsigned char reg_buf[NUM_TX];
	short AveScreen;

	for(i=0;i<AWTPCfg.TX_LOCAL;i++)
	{
		Gain[i] = GainInitialValue;
		Flag[i] = 0;
	}


	I2C_ReadXByte(reg_buf, SA_TXADCGAIN0, AWTPCfg.TX_LOCAL);

ReCaliGain:
	for (i = 0; i < AWTPCfg.TX_LOCAL; i++)
	{
		reg_data = reg_buf[i];
		reg_data = (reg_data & 0xe0) | Gain[i];
		I2C_WriteByte(SA_TXADCGAIN0+i, reg_data);
	}

	AW5306_CLB_GETDIFF();
	for(i=0;i<AWTPCfg.TX_LOCAL;i++)
	{
		AveScreen = AW5306_CLB_GETAVE(i,0xff);
		if ( AveScreen <= AWTPCfg.GainClbDeltaMin ) 
		{
			if(Gain[i] > 1)
			{
				Gain[i] = Gain[i]-1;
			}
			else
			{
				printk("Gain clb  Min Error Col %d", i);
				goto GainClbFail;
			}
		}
		else if  ( AveScreen >= AWTPCfg.GainClbDeltaMax)
		{
			if(Gain[i] < 0x1e)
			{
				Gain[i] = Gain[i]+1;
			}
			else
			{
				printk("Gain clb  MAX Error Col %d", i);
				goto GainClbFail;
			}
			
		} 
		else
		{
			Flag[i] = 1;
		}
	}

	for(i=0;i<AWTPCfg.TX_LOCAL;i++)
	{
		if(Flag[i] != 1)
		{
			goto ReCaliGain;
		}
	}

	return;


GainClbFail:	
	ClbErrorCode |= 0x4;
	
	return; 
}
#else
void AW5306_GAIN_CLB()
{
	unsigned char Gain,i;
	unsigned char reg_data;
	unsigned char reg_buf[NUM_TX];
	short AveScreen;

	Gain = GainInitialValue;
	
	while ((Gain < 0x1e) && (Gain > 1))
	{
		I2C_ReadXByte(reg_buf, SA_TXADCGAIN0, AWTPCfg.TX_LOCAL);
		for (i = 0; i < AWTPCfg.TX_LOCAL; i++)
		{
			reg_data = reg_buf[i];
			reg_data = (reg_data & 0xe0) | Gain;
			I2C_WriteByte(SA_TXADCGAIN0+i, reg_data);
		}
		AW5306_CLB_GETDIFF();

		//Sum = AW5306_CLB_GETSUM(0xff, 0xff);
		AveScreen = AW5306_CLB_GETAVE(0xff,0xff);
		if ( AveScreen <= AWTPCfg.GainClbDeltaMin ) 
		{
			// printf("cur gain = %d, delta = %d ", Gain, AveScreen);
			Gain--;
		}
		else if  ( AveScreen >= AWTPCfg.GainClbDeltaMax)
		{
			// printf("cur gain = %d, delta = %d", Gain, AveScreen);
			Gain++;
		} 
		else 
		{
			//get_cfg();
			break;
		}
	}

	if ((Gain < 1) || (Gain > 0x1e))
	{
	//	printk("Can't find an approprate value for gain!\n");
		ClbErrorCode |= 0x4;
	}
}

#endif

void AW5306_SOFTOFFSET_CLB(void)
{
	unsigned char i,j,k;

	AW5306_CLB_GETADC();

	for(i=0;i<AWTPCfg.TX_LOCAL;i++)
	{
		for(j=0;j<AWTPCfg.RX_LOCAL;j++)
		{
			if (AWTPCfg.RX_INV_ORDER == 1)
			{
				k = AWTPCfg.RX_LOCAL-1-j;
				AW_Cali.SOFTOFFSET[i][k] = 8400 -DiffBuff[i][j];
			} 
			else 
			{
				AW_Cali.SOFTOFFSET[i][j] = 8400 -DiffBuff[i][j];
			}
		}
	}
}

void AW5306_CLB_GetCfg(void)
{
	I2C_ReadXByte(&AW_Cali.TXOFFSET[0], SA_TXOFFSET0, (NUM_TX+1)/2);
	I2C_ReadXByte(&AW_Cali.RXOFFSET[0], SA_RXOFFSET0, (NUM_RX+1)/2);
	I2C_ReadXByte(&AW_Cali.TXCAC[0], SA_TXCAC0, NUM_TX);
	I2C_ReadXByte(&AW_Cali.RXCAC[0], SA_RXCAC0, NUM_RX);
	I2C_ReadXByte(&AW_Cali.TXGAIN[0], SA_TXADCGAIN0, NUM_TX);
}

void AW5306_CLB_WriteCfg(void)
{
	unsigned char i;

	for(i=0;i<10;i++)
	{
		I2C_WriteByte(SA_TXOFFSET0+i, AW_Cali.TXOFFSET[i]);
	}
	for(i=0;i<6;i++)
	{
		I2C_WriteByte(SA_RXOFFSET0+i, AW_Cali.RXOFFSET[i]);
	}

	for(i=0;i<20;i++)
	{
		I2C_WriteByte(SA_TXCAC0+i, AW_Cali.TXCAC[i]);
	}
	for(i=0;i<12;i++)
	{
		I2C_WriteByte(SA_RXCAC0+i, AW_Cali.RXCAC[i]);
	}
	for(i=0;i<20;i++)
	{
		I2C_WriteByte(SA_TXADCGAIN0+i, AW_Cali.TXGAIN[i]);
	}
}

char AW5306_CLB(void)
{
	ClbErrorCode = 0;

	// chip enable, single scan mode
	I2C_WriteByte(SA_CTRL,   0x09);
	I2C_WriteByte(SA_SCANMD, 0x06);

	RX_START = I2C_ReadByte(SA_RX_START);

	// --------------------------------------------------------------------
	//printk("// 	TXCLKDLY calibration\n");
	// --------------------------------------------------------------------
	AW5306_TXDLY_CLB();

	// --------------------------------------------------------------------
	//printk("// 	TX/RXCAC (Charge Amplier Convertor) calibration\n");
	// --------------------------------------------------------------------
	AW5306_CAC_CLB();
	if(ClbErrorCode != 0)
	{
		return ClbErrorCode;
	}
	
	// --------------------------------------------------------------------
	//printk("// 	TX/RX OFFSET calibration\n");
	// --------------------------------------------------------------------
	AW5306_OFFSET_CLB();
	if(ClbErrorCode != 0)
	{
		return ClbErrorCode;
	}
	
	// --------------------------------------------------------------------
	//printk("// 	TX/RX ADC GAIN calibration\n");
	// --------------------------------------------------------------------
	AW5306_GAIN_CLB();
	if(ClbErrorCode != 0)
	{
		return ClbErrorCode;
	}

	AW5306_OFFSET_CLB();
	if(ClbErrorCode != 0)
	{
		return ClbErrorCode;
	}
	
	AW5306_SOFTOFFSET_CLB();
	//AW5306_CLB_GetCfg();

	return ClbErrorCode;

	//printk("Calibration Error Code = %d\n", ClbErrorCode);

}

unsigned char AW5306_RAWDATACHK(void)
{
	unsigned char i,j;
	unsigned int min, max, temp;
	unsigned char res = 0;

	I2C_WriteByte(SA_CTRL,0x09);			//scan en
	AW5306_CLB_GETADC();
	for(i = 0;i<AWTPCfg.TX_LOCAL;i++)
	{
		for(j =0;j<AWTPCfg.RX_LOCAL;j++)
		{
			if (((i == AWTPCfg.TX_LOCAL-1) && (AWTPCfg.KeyLineValid[j] == 1)) || (i < AWTPCfg.TX_LOCAL-1))
			{
				if((DiffBuff[i][j]<AWTPCfg.OffsetClbExpectedMin-1000)||(DiffBuff[i][j]>AWTPCfg.OffsetClbExpectedMin+1000))
				{
					res = 0x01;
					if(AWTPCfg.DEBUG_SWITCH == 1)
					{
						printk("Raw Data Test Fail.\n");
					}
				}
			}
		}
	}

	AW5306_CLB_GETDIFF();
	for(j = 0;j<AWTPCfg.RX_LOCAL;j++)
	{
		min = 65536;
		max = 0;
		for(i =0;i<AWTPCfg.TX_LOCAL;i++ )
		{
			if (((i == AWTPCfg.TX_LOCAL-1) && (AWTPCfg.KeyLineValid[j] == 1)) || (i < AWTPCfg.TX_LOCAL-1))
			{
				if(DiffBuff[i][j] < 200)
				{
					if(AWTPCfg.DEBUG_SWITCH == 1)
					{
						printk("Error: RX %d Rawdata Devaition Test FAIL! (MinVal = %d, MaxVal = %d ) \n", j, min, max);
					}
					res |= 0x2;
					break;
				}
				if ( DiffBuff[i][j] > max ) max = DiffBuff[i][j];
				if ( DiffBuff[i][j] < min ) min = DiffBuff[i][j];
				temp = max - min;

				if ( temp > AWTPCfg.RawDataDeviation)
				{
					if(AWTPCfg.DEBUG_SWITCH == 1)
					{
						printk("Error: RX %d Rawdata Devaition Test FAIL! (MinVal = %d, MaxVal = %d ) \n", j, min, max);
					}
					res |= 0x2;
					break;
				}
			}

		}
	}
	//if (res == 0) printf("AW5x06 Raw Data Checking PASS!\n");
	return res;
}
