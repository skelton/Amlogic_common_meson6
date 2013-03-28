/**************************************************************************
*  AW5306_Base.c
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

#define MARGCOORNUM 				3		//calculate the coordinate ,3*3


#define RXREGULARITY_GOOD				1
#define RXREGULARITY_BAD				2

#define TRACE_TEMP_NONE 				0
#define TRACE_TEMP_INC					1
#define TRACE_TEMP_DEC					2

#define TRACE_TEMP_THRELD				10
#define TRACE_TEMP_INTEVAL				5

#define TRACE_PEAKSTABLE_OFFSET  		25

#define TRACE_LIMIT_POINT			1  
#define TRACE_STEP					3
#define TRACE_THRES 				TRACE_STEP	


#define BIG_TOUCH_STABLE_FRAME_NUM		40
#define BIG_TOUCH_FRAME_THRESHOLD   	(5*20)

#define	LONGPEAK_INVALID				0
#define LONGPEAK_POSSIBLE				1
#define LONGPEAK_VALID					2

#define TRACE_PEAKSTABLE_OFFSET  		25

#define TIME_LONGTOUCH      			4


#define TRACE_COMPENSATE_NONE	       	0
#define TRACE_COMPENSATE_NORMAL			1
#define TRACE_COMPENSATE_TEMPDEC		2
#define TRACE_COMPENSATE_TEMPINC		3
#define TRACE_COMPENSATE_HIGH			4
#define TRACE_COMPENSATE_NONORMAL     	5

#define BASE_FRESH_TH		20

#define CALNOISE_TH             	16

#define BASE_FRESH_CNT		10

#define BASE_VALUE		8400
#define BASE_TH			70
#define BASE_NORMALTH	50



extern void BaseReInit(void);
extern unsigned short CalDiff(unsigned short a0, unsigned short a1);
extern void FreshBase(void);

extern STRUCTBASE		AW_Base;
extern STRUCTPEAK		AW_Peak;
extern AW5306_UCF	AWTPCfg;
extern STRUCTCALI		AW_Cali;
extern short	Diff[NUM_TX][NUM_RX];
extern char AW5306_WorkMode;

extern short	BIGAREA_PEAK_TH;
extern short	THPEAK;	//suspected touch threshold
extern short	THGROUP;	// touch threshold


unsigned short WakeUpLastCnt,KeepCnt;
unsigned long RawDataSumBig,RawDataSumNor;

typedef	struct{
	unsigned char PeakX;				//the peak x coordinary
	unsigned char PeakY;				//the peak y cooordinary
	short PeakValue;					//the adc value of the peak 
	unsigned char LongTouchValid;		//the valid flag,  0: not long touch ;  1: possible long touch; 2: real long touch
	unsigned char ZeroSet;
}STRUCTPEAKSTATUS;

STRUCTPEAKSTATUS PosPeak[5];
STRUCTPEAKSTATUS NegPeak[5];


char  CheckRxRegularity(void)
{
    unsigned short RxRaw;
    unsigned char i,j;
    unsigned short RxRawMax;
    unsigned short RxRawMin;
	unsigned char RxStableFlag;

    RxStableFlag = RXREGULARITY_BAD;

    for(i = 0; i < AWTPCfg.RX_LOCAL; i++)
    {
        RxRawMax = 0;
        RxRawMin = 0x7fff;

        //check the diff value between points of the same rx and mean value
        for(j = 0; j < AWTPCfg.TX_LOCAL; j ++)
        {        
            RxRaw = AW_Base.Base[j][i] + Diff[j][i];

            if (RxRaw > RxRawMax)
            {
                RxRawMax = RxRaw;
            }

            if (RxRaw < RxRawMin)
            {
                RxRawMin = RxRaw;
            }
        }

        if ((RxRawMax - RxRawMin) > TRACE_PEAKSTABLE_OFFSET)
        {
            RxStableFlag = RXREGULARITY_BAD;
            break;
        }
    }

    if (i == (AWTPCfg.RX_LOCAL + 1))
    {
        RxStableFlag = RXREGULARITY_GOOD;
    }

    return RxStableFlag;
}


char   CheckTemperatureDrift(void)
{
    unsigned char i,j;
    
    unsigned char ucTxOrderStep = AWTPCfg.TX_LOCAL>>2;
    unsigned char ucRxOrderStep = AWTPCfg.RX_LOCAL>>2;

    unsigned char ucIncCnt = 0;
    unsigned char ucDecCnt = 0;
    unsigned char ucSampleCnt = 0;

    static unsigned char ucLastTraceTempState = TRACE_TEMP_NONE;
    
    //Check the typical points
    for(i = 0 ;i < AWTPCfg.TX_LOCAL; i += ucTxOrderStep )
    {
        for(j = 0;j < AWTPCfg.RX_LOCAL; j += ucRxOrderStep)
        {
            if (Diff[i][j] > TRACE_TEMP_THRELD)
            {
                ucIncCnt ++;
            }
            else if (Diff[i][j] < -TRACE_TEMP_THRELD)
            {
                ucDecCnt ++;
            }
            ucSampleCnt++;
        }
    }

    //if the number of the points with the increasing value is bigger than the number of the points with the decreasing value
    if (ucIncCnt > (ucSampleCnt>>1))
    {
	    AW_Base.TraceTempDecCnt = 0;
	    if (AW_Base.TraceTempIncCnt < TRACE_TEMP_INTEVAL)
	    {
	    	AW_Base.TraceTempIncCnt ++;
	    }
	    else
	    {
	        AW_Base.TraceTempIncCnt = 0;
	        ucLastTraceTempState = TRACE_TEMP_INC;
	    }
    }
    else if (ucDecCnt > (ucSampleCnt>>1))
    {
        AW_Base.TraceTempIncCnt = 0;
        if (AW_Base.TraceTempDecCnt < TRACE_TEMP_INTEVAL)
        {
            AW_Base.TraceTempDecCnt ++;
        }
        else
        {
            AW_Base.TraceTempDecCnt = 0;
            ucLastTraceTempState = TRACE_TEMP_DEC;
        }
    }
    else
    {
        AW_Base.TraceTempDecCnt = 0;
        AW_Base.TraceTempIncCnt = 0;
        ucLastTraceTempState = TRACE_TEMP_NONE;
    }

    return ucLastTraceTempState;
}

void PeakValidCheck(void)
{
    signed char i;
    signed char k;
    short temp;
    unsigned char X,Y;
	STRUCTPEAKSTATUS *pstructPeak;

    signed char m,n;
    signed short DiffTemp;
	//unsigned char ChkPointsNum = 5;

    unsigned char CurrentPointNum = AW_Peak.CurrentPointNum;
    unsigned char CurrentNegPointNum = AW_Peak.CurrentNegPointNum;
	//unsigned char RowMin, RowMax;

    
    if ((CurrentPointNum > 0) || (CurrentNegPointNum > 0))
    {
		for (i = 0; i < CurrentPointNum; i++)
		{
			PosPeak[i].PeakY = AW_Peak.Peak[i][0]>>1;
            PosPeak[i].PeakX = AW_Peak.Peak[i][1]>>1;
			PosPeak[i].ZeroSet = 0;
		#if 1	
			// Remove Pos peak caused by water
			temp = 0;
			for (m = PosPeak[i].PeakY-1; m <= PosPeak[i].PeakY+1; m++)
			{
				for (n = PosPeak[i].PeakX-1; n <= PosPeak[i].PeakX+1; n++)
				{
					if ((m >= 0) && (m < AWTPCfg.TX_LOCAL) && (n >= 0) && (n < AWTPCfg.RX_LOCAL) && (m+n<PosPeak[i].PeakX+PosPeak[i].PeakY+2))
					{
						temp += Diff[m][n];
						// if (Diff[m][n] < -80)
						// {
						// 	Diff[PosPeak[i].PeakY][PosPeak[i].PeakX] = 0;
						// }
					}
				}
			}
			if ((temp < THPEAK*2) && ((PosPeak[i].PeakY != AWTPCfg.TX_LOCAL-1) || (AWTPCfg.HAVE_KEY_LINE == 0)))
			{
				//Diff[PosPeak[i].PeakY][PosPeak[i].PeakX] = 0;
				PosPeak[i].ZeroSet = 1;
			}
		#endif
			
		}
		for (i = 0; i < CurrentNegPointNum; i++)
		{
			NegPeak[i].PeakY = AW_Peak.NegPeak[i][0]>>1;
            NegPeak[i].PeakX = AW_Peak.NegPeak[i][1]>>1;
		}
        AW_Base.PeakCheckFrameCnt ++;
		
        if (AW_Base.PeakCheckFrameCnt == 5)			// 5 cycles trigger one shot
        {
            AW_Base.PeakCheckFrameCnt  = 0;
            if ((AW_Base.PosPeakCnt == 0) && (AW_Base.NegPeakCnt == 0))
            {
            	AW_Base.LongStableCnt = 0;			// clear the long touch time count 
            	AW_Base.PosPeakCnt = CurrentPointNum;
            	AW_Base.NegPeakCnt = CurrentNegPointNum;

                for (i = 0; i < (CurrentPointNum + CurrentNegPointNum); i ++)
                {
                    if (i < CurrentPointNum)
                    {
                        pstructPeak = &PosPeak[i];

                        Y = AW_Peak.Peak[i][0]>>1;
                        X = AW_Peak.Peak[i][1]>>1;
                    }
                    else
                    {
                        pstructPeak = &NegPeak[i - CurrentPointNum];

                        Y = AW_Peak.NegPeak[i - CurrentPointNum][0]>>1;
                        X = AW_Peak.NegPeak[i - CurrentPointNum][1]>>1;
                    }
                
                    pstructPeak->PeakY = Y;
                    pstructPeak->PeakX = X;
	    		    pstructPeak->LongTouchValid = LONGPEAK_POSSIBLE;		// by default, long touch
                    pstructPeak->PeakValue = Diff[Y][X];
                    // initialize the accumulated noise value to be 0 for the start value
                }
            }
            else 
            {
                k = AW_Base.PosPeakCnt + AW_Base.NegPeakCnt;
                for (i = 0; i < (AW_Base.PosPeakCnt + AW_Base.NegPeakCnt); i++)		// check if it is long touch
                {
                    if (i < AW_Base.PosPeakCnt)
                    {
                        pstructPeak = &PosPeak[i];
                    }
                    else
                    {
                        pstructPeak = &NegPeak[i - AW_Base.PosPeakCnt];
                    }
                    
                    temp = Diff[pstructPeak->PeakY][pstructPeak->PeakX];

                    if (pstructPeak->LongTouchValid == LONGPEAK_INVALID)		// not long touch
                    {
        	            k--;
                    }
                    else
	        	    {
	        	    	//whether the peak is the real peak

						//col
	        	        n = 0;
                        for(m = 0; m < AWTPCfg.TX_LOCAL; m ++)	
                        {
                            //The same point 
                            if (m == pstructPeak->PeakY)
                            {
                                continue;
                            }
				DiffTemp = (Diff[pstructPeak->PeakY][pstructPeak->PeakX] + AW_Base.Base[pstructPeak->PeakY][pstructPeak->PeakX]\
					+AW_Cali.SOFTOFFSET[pstructPeak->PeakY][pstructPeak->PeakX])\
                                        -((Diff[m][pstructPeak->PeakX] + AW_Base.Base[m][pstructPeak->PeakX]\
                                        +AW_Cali.SOFTOFFSET[m][pstructPeak->PeakX]));

							if (i < AW_Base.PosPeakCnt)     
							{
							   if ((DiffTemp > ((short)THGROUP-40)) && (pstructPeak->ZeroSet == 0))
							   {
							       n++;
							   }
							}
							else
							{
								if (CurrentPointNum > 0)	// Only check negtive points during no positive points
								{
							       n++;
							   	}
							}
                        }

						if (n >= (AWTPCfg.TX_LOCAL>>1))		// Modified by YaoWei 2012.7.26
                        {
                            if (AW_Base.CompensateFlag != TRACE_COMPENSATE_NONORMAL)
                            {
                            	//printk("TRACE_COMPENSATE_NONORMAL!!!!!!!!\n");
                                pstructPeak->LongTouchValid = LONGPEAK_INVALID;
                            }
                        }
	        	    
						 //col check
						n = 0;                       
						for(m = 0; m < AWTPCfg.RX_LOCAL; m ++)	
                        {
                            //The same point 
                            if (m == pstructPeak->PeakX)
                            {
                                continue;
                            }
				DiffTemp = (Diff[pstructPeak->PeakY][pstructPeak->PeakX] + AW_Base.Base[pstructPeak->PeakY][pstructPeak->PeakX]\
					+AW_Cali.SOFTOFFSET[pstructPeak->PeakY][pstructPeak->PeakX])\
                                        -((Diff[pstructPeak->PeakY][m] + AW_Base.Base[pstructPeak->PeakY][m]\
                                        +AW_Cali.SOFTOFFSET[pstructPeak->PeakY][m]));

							if (i < AW_Base.PosPeakCnt)     
							{
							   if ((DiffTemp > ((short)THGROUP-40)) && (pstructPeak->ZeroSet == 0))
							   {
							       n++;
							   }
							}
							else
							{
								if (CurrentPointNum > 0)	// Only check negtive points during no positive points
								{
							       n++;
							   	}
							}
							
                        }
						if (n >= (AWTPCfg.RX_LOCAL>>1))		// Modified by YaoWei 2012.7.26
                        {
                            if (AW_Base.CompensateFlag != TRACE_COMPENSATE_NONORMAL)
                            {
                                pstructPeak->LongTouchValid = LONGPEAK_INVALID;
                            }
                        }
                        // if (the difference > TH_NOT_STABLE), then (this point is definitely not long-stable peak)
						pstructPeak->ZeroSet = 0;
                        temp = temp - pstructPeak->PeakValue;  
						temp = ABS(temp);
	        		      	
		        		if (temp > TRACE_PEAKSTABLE_OFFSET)		// difference > TH_NOT_STABLE
	            		{
	            			//printk("DIFF > TH_NOT_STABLE!!!!!!!!!!\n");
				            pstructPeak->LongTouchValid = LONGPEAK_INVALID;		// not longtouch
	            		}
                    }
                }

                if (k == 0)	// without any possible long touch for this whole sample process
                {
                	//printk("K == 0 \n");
                    AW_Base.PosPeakCnt = 0;
                    AW_Base.NegPeakCnt = 0;
                }
                else			// possible long touch in the future for this whole sample process
                {
                    if (AW_Base.LongStableCnt >= TIME_LONGTOUCH)
                    {
                        for (i = 0; i < (AW_Base.PosPeakCnt + AW_Base.NegPeakCnt); i++)
                        {
                            if (i < AW_Base.PosPeakCnt)
                            {
                                pstructPeak = &PosPeak[i];
                            }
                            else
                            {
                                pstructPeak = &NegPeak[i - AW_Base.PosPeakCnt];
                            }
                            
                            if (pstructPeak->LongTouchValid == LONGPEAK_POSSIBLE)		// still long touch possible
	            		    {
	    		                // only when (noise level < noise_threshold), this peak could be regardes as long-stable peak
			    				//if (pstructPeak->usPeakNoise < NOISE_LONGSTABLE * TIME_LONGTOUCH)
			    				{
						            pstructPeak->LongTouchValid = LONGPEAK_VALID;		// long touch
			    				}
		    			    }
                        }
                    }
                    else
                    {
                        AW_Base.LongStableCnt ++;
                    }
                }	
            }//FOR A AREA HANDLE
    	}
    }
    else
    {
        AW_Base.LongStableCnt = 0;
        AW_Base.PosPeakCnt = 0;
        AW_Base.NegPeakCnt = 0;
    }  
}

unsigned long SumRawDataProcess(void)
{
	unsigned char  i_temp,j_temp;
	unsigned short index;
	unsigned long int RawDataSum;
	unsigned short *pRawData;
	short *pTouchData;
	

	pRawData = &AW_Base.Base[0][0];
	pTouchData = &Diff[0][0];
	RawDataSum = 0;
       
	for (i_temp=0; i_temp<AWTPCfg.TX_LOCAL; i_temp++)		
	{
		index = i_temp*NUM_RX;
		
		for (j_temp=0; j_temp<AWTPCfg.RX_LOCAL; j_temp++)	
		{
		    RawDataSum += pRawData[index] + pTouchData[index];
		    index++;
		}	
	}
	return RawDataSum;
}


char BigAreaProcess(void)
{
	if (AW_Base.NegBigAreaTouchFlag == 1)
	{
		AW_Base.BigTouchFrame++;

		if(AW_Base.BigAreaChangeFlag == 0)
		{
			RawDataSumBig = SumRawDataProcess();
			if(AW_Base.BigTouchFrame >= BIG_TOUCH_STABLE_FRAME_NUM)
			{
				AW_Base.BigAreaChangeFlag = 1;
				
				if(RawDataSumBig < RawDataSumNor)
				{
				    AW5306_TP_Reinit();
					return 1;
				}
			}
		}
		
		if(AW_Base.BigTouchFrame > BIG_TOUCH_FRAME_THRESHOLD)
		{
			AW5306_TP_Reinit();
			AW_Base.BigAreaChangeFlag = 0;
			AW_Base.BigTouchFrame = 0;
			return 1;
		}
	}
	else
	{
		AW_Base.BigTouchFrame = 0;
		
		if( (AW_Base.BigAreaChangeFlag == 0) && (AW_Peak.CurrentPointNum == 0) )
	    {
	        RawDataSumNor = SumRawDataProcess();
	    }
	}
	return 0;
}


void  BaseCompensate(void)
{
	signed char TraceStep,i,j;
	unsigned char k,calc,index,FlagCompensate;

	if(AW_Base.CompensateFlag != TRACE_COMPENSATE_NONE)
	{
		TraceStep = TRACE_STEP;
		
		if(AW_Base.CompensateFlag == TRACE_COMPENSATE_HIGH)
		{
			TraceStep *= 4;
		}
		switch(AW_Base.CompensateFlag)
		{
			case TRACE_COMPENSATE_TEMPINC:
				for(i=0; i<AWTPCfg.TX_LOCAL; i++)
				{
					for(j=0; j<AWTPCfg.RX_LOCAL; j++)
					{
						AW_Base.Base[i][j] += TraceStep;
					}
				}
				break;
			case TRACE_COMPENSATE_TEMPDEC:
				for(i=0; i<AWTPCfg.TX_LOCAL; i++)
				{
					for(j=0; j<AWTPCfg.RX_LOCAL; j++)
					{
						AW_Base.Base[i][j] -= TraceStep;
					}
				}
				break;
			case TRACE_COMPENSATE_NONORMAL:
				//set flag
				for(k=0; k<AW_Peak.CurrentPointNum; k++)		//peak point don't compensate
				{
					calc = 2;

					index = AW_Peak.Peak[k][0]/2;
					if(index < 1 || index >= (AWTPCfg.TX_LOCAL-1))
					{
						calc = MARGCOORNUM /2;
					}
					index =  AW_Peak.Peak[k][1]/2;
					if(index <1 || index >=(AWTPCfg.RX_LOCAL-1))
					{
						calc = MARGCOORNUM /2;
					}
					
					FlagCompensate = 0;
					
					if(Diff[AW_Peak.Peak[k][0]/2][AW_Peak.Peak[k][1]/2] > THGROUP)
					{
						FlagCompensate = 1;
					}

					for(i=AW_Peak.Peak[k][0]/2-calc; i < AW_Peak.Peak[k][0]/2 +calc; i++)
					{
						for(j = AW_Peak.Peak[k][1]/2-calc; j < AW_Peak.Peak[k][1]/2+calc; j++)
						{
							if(i>=0 && i<AWTPCfg.TX_LOCAL && j>=0 && j<AWTPCfg.RX_LOCAL)
							{
								if(FlagCompensate==1)
								{
									AW_Base.Flag[i][j] = 1;
								}
							}
						}
					}
				}
				//base compesate
				for(i=0; i<AWTPCfg.TX_LOCAL; i++)
				{
					for(j=0; j<AWTPCfg.RX_LOCAL; j++)
					{
						if(AW_Base.Flag[i][j] != 1)
						{
							if(Diff[i][j] > TRACE_THRES)
							{
								AW_Base.Base[i][j] += TraceStep;
							}
							if(Diff[i][j] < -TRACE_THRES)
							{
								AW_Base.Base[i][j] -= TraceStep;
							}
						}
					}
				}

				//clear flag
				for(i=0; i<AWTPCfg.TX_LOCAL; i++)
				{
					for(j=0; j<AWTPCfg.RX_LOCAL; j++)
					{
						AW_Base.Flag[i][j] = 0;
					}
				}
				break;
			default:
				for(i=0; i<AWTPCfg.TX_LOCAL; i++)
				{
					for(j=0; j<AWTPCfg.RX_LOCAL; j++)
					{
						if(Diff[i][j] > TRACE_THRES)
						{
							if(AW_Base.BaseCnt[i][j] > BASE_FRESH_CNT)
							{
								AW_Base.BaseCnt[i][j] = 0;
								AW_Base.Base[i][j] += TraceStep;
							}
							else
							{
								AW_Base.BaseCnt[i][j]++;
							}
						}
						if(Diff[i][j] < -TRACE_THRES)
						{
							if(AW_Base.BaseCnt[i][j] < -BASE_FRESH_CNT)
							{
								AW_Base.BaseCnt[i][j] = 0;
								AW_Base.Base[i][j] -= TraceStep;
							}
							else
							{
								AW_Base.BaseCnt[i][j]--;
							}
						}
					}
				}
				break;
		}
	}	
}

// peak[i][0]: row;
// peak[i][1]: col;
void AWPeakCompensate(void)
{
    signed char i,j,PeakRow,PeakCol,Minidx;
    unsigned char ucPeakCnt;
	short RawData, Row,MinDiff;
	short DeltaBase, DeltaRaw;

	Minidx = 0;
	
    for(ucPeakCnt = 0; ucPeakCnt < AW_Peak.CurrentNegPointNum; ucPeakCnt++)
    {
		PeakRow = AW_Peak.NegPeak[ucPeakCnt][0] >> 1;
		PeakCol = AW_Peak.NegPeak[ucPeakCnt][1] >> 1;
		if (Diff[PeakRow][PeakCol] > -1*THPEAK)
		{
			continue;
		}
 

        MinDiff = 0xfff;
		Row = PeakRow;
		for (i = 0; i < 5; i++)
		{
			if (PeakRow < AWTPCfg.TX_LOCAL/2)
			{
				Row++;
			}
			else
			{
				Row--;
			}
			if (Diff[Row][PeakCol] > THPEAK)
			{
				MinDiff = ABS(Diff[Row][PeakCol]);
				Minidx = Row;
				break;
			}
			
			if (ABS(Diff[Row][PeakCol]) < MinDiff)
			{
				MinDiff = ABS(Diff[Row][PeakCol]);
				Minidx = Row;
				if (MinDiff < 20)
				{
					break;
				}
			}
		}
		if (Diff[Minidx][PeakCol] > THPEAK)
		{
			break;
		}
		j=0;
		RawData = AW_Base.Base[PeakRow][PeakCol]+Diff[PeakRow][PeakCol];
		for (i = -2; i < 3; i++)
		{
			Row = PeakRow+i;
			if (Row < 0) { Row = PeakRow+2-Row; }
			else if (Row >= AWTPCfg.TX_LOCAL) { Row = PeakRow - 3 - (Row - AWTPCfg.TX_LOCAL); }
			if ((Row > 0) && (Row < AWTPCfg.TX_LOCAL))
			{
				
				DeltaRaw = RawData - (AW_Base.Base[Row][PeakCol]+Diff[Row][PeakCol]);
				if (ABS(DeltaRaw) < THPEAK)
				{
					j++;
				}
			}
		}
		DeltaRaw = AW_Base.Base[PeakRow][PeakCol]+Diff[PeakRow][PeakCol] - (AW_Base.Base[Minidx][PeakCol]+Diff[Minidx][PeakCol]);
		DeltaBase = AW_Base.Base[PeakRow][PeakCol] - AW_Base.Base[Minidx][PeakCol];
		if (((ABS(DeltaBase) > ABS(DeltaRaw)) && (ABS(DeltaBase) > THPEAK))
			|| (j > 2))// || (DeltaBase > (ABS(Diff[PeakRow][PeakCol])*8/10)))			
		{
			if (AW_Base.NegPeakCompensateCnt[ucPeakCnt] < 5)
			{
				AW_Base.NegPeakCompensateCnt[ucPeakCnt]++;
			}
			else
			{
				AW_Base.NegPeakCompensateCnt[ucPeakCnt]=0;
				for(i = (PeakRow- 2); i <= (PeakRow + 2);i ++)
				{
					for(j = (PeakCol- 2);j <= (PeakCol + 2);j ++)
					{
						if ((i>=0) && (i<NUM_TX) && (j>=0) && (j<NUM_RX))
						{
							AW_Base.Base[i][j] += Diff[i][j];
							Diff[i][j] = 0;
						}
					}
				}
			}
        }
		else
		{
			AW_Base.NegPeakCompensateCnt[ucPeakCnt]=0;
		}
    }

    for(ucPeakCnt = 0; ucPeakCnt < AW_Peak.CurrentPointNum; ucPeakCnt++)
    {
		PeakRow = AW_Peak.Peak[ucPeakCnt][0] >> 1;
		PeakCol = AW_Peak.Peak[ucPeakCnt][1] >> 1;


        MinDiff = 0xfff;
		Row = PeakRow;
		RawData = AW_Base.Base[PeakRow][PeakCol]+Diff[PeakRow][PeakCol];
		j=0;
		for (i = -2; i < 3; i++)
		{
			Row = PeakRow+i;
			if ((Row > 0) && (Row < AWTPCfg.TX_LOCAL))
			{
				if (RawData - AW_Base.Base[Row][PeakCol] < THPEAK)
				{
					j++;
				}
				if (ABS(Diff[Row][PeakCol]) < MinDiff)
				{
					MinDiff = ABS(Diff[Row][PeakCol]);
					Minidx = Row;
					if (MinDiff < 20)
					{
						MinDiff = 0;
					}
				}
			}
		}
		DeltaRaw = AW_Base.Base[PeakRow][PeakCol]+Diff[PeakRow][PeakCol]-(AW_Base.Base[Minidx][PeakCol]+Diff[Minidx][PeakCol]);
		DeltaBase = AW_Base.Base[PeakRow][PeakCol] - AW_Base.Base[Minidx][PeakCol];
		if (((ABS(DeltaBase) > DeltaRaw) && (ABS(DeltaBase) > THPEAK))
			|| (j>2))// || (DeltaBase > (ABS(Diff[PeakRow][PeakCol])*8/10)))			
		{
			if (AW_Base.PosPeakCompensateCnt[ucPeakCnt] < 5)
			{
				AW_Base.PosPeakCompensateCnt[ucPeakCnt]++;
			}
			else
			{
				AW_Base.PosPeakCompensateCnt[ucPeakCnt]=0;			
				for(i = (PeakRow- 2); i <= (PeakRow + 2);i ++)
				{
					for(j = (PeakCol- 2);j <= (PeakCol + 2);j ++)
					{
						if ((i>=0) && (i<NUM_TX) && (j>=0) && (j<NUM_RX))
						{
							AW_Base.Base[i][j] += Diff[i][j];
							Diff[i][j] = 0;
						}
					}
				}
				//AW_Base.Base[PeakRow][PeakCol] += Diff[PeakRow][PeakCol];
				//Diff[PeakRow][PeakCol] = 0;
			}
        }
		else
		{
			AW_Base.PosPeakCompensateCnt[ucPeakCnt]=0;
		}
    }
	for(ucPeakCnt = AW_Peak.CurrentPointNum; ucPeakCnt < MAX_POINT; ucPeakCnt++)
    {
		AW_Base.PosPeakCompensateCnt[ucPeakCnt]=0;
	}
	for(ucPeakCnt = AW_Peak.CurrentNegPointNum; ucPeakCnt < MAX_POINT; ucPeakCnt++)
    {
		AW_Base.NegPeakCompensateCnt[ucPeakCnt]=0;
	}
}

void PeakCompensate(void)
{
    signed char i,j;
    unsigned char ucPeakCnt;
	STRUCTPEAKSTATUS *pstructPeak;

	//if(AW_Base.InitialFrameCnt >= 200)
	//{
		
	//}
	//else
	//{
	//	AW_Base.InitialFrameCnt++;
	//}

	//if((AW_Base.BigAreaTouchFlag == 1)&&(AW_Base.InitialFrameCnt < 180))
	//{
	//	AW5306_TP_Reinit();
	//}
    
    for(ucPeakCnt = 0; ucPeakCnt < (MAX_POINT << 1);ucPeakCnt++)
    {
        if (ucPeakCnt < MAX_POINT)
        {
            pstructPeak = &PosPeak[ucPeakCnt];
        }
        else
        {     
            pstructPeak = &NegPeak[ucPeakCnt - MAX_POINT];
         
        }
        
        if (pstructPeak->LongTouchValid == LONGPEAK_VALID)
        {
            pstructPeak->LongTouchValid = LONGPEAK_INVALID;
			//printk("PEAK COMPENSATE %d,%d,",pstructPeak->PeakY,pstructPeak->PeakX);
            
            for(i = (pstructPeak->PeakY- 2); i <= (pstructPeak->PeakY + 2);i ++)
            {
                for(j = (pstructPeak->PeakX- 2);j <= (pstructPeak->PeakX + 2);j ++)
                {
                    if ((i>=0) && (i<NUM_TX) && (j>=0) && (j<NUM_RX))
					{
						if(ucPeakCnt < MAX_POINT)
						{
							AW_Base.Base[i][j] += Diff[i][j];
							Diff[i][j] = 0;
						}
						else if((ucPeakCnt >= MAX_POINT) && (Diff[i][j] < 0))
						{
							AW_Base.Base[i][j] += Diff[i][j];
							Diff[i][j] = 0;
						}
					}
                }
            }
        }
    }
}

/*use for delta mode check if base need refresh*/
unsigned char BaseCheck(void)
{
	unsigned char i,j;
	short DiffCount = 0;

	for(i=0;i<AWTPCfg.TX_LOCAL;i++)
	{
		for(j=0;j<AWTPCfg.RX_LOCAL;j++)
		{
			if(CalDiff(AW_Base.Base[i][j],AW_Base.ChipBase[i][j]) > CALNOISE_TH)
			{
				DiffCount++;
			}
		}
	}

	if( DiffCount > BASE_FRESH_CNT )
	{
	//	printk("AW5306 Base ReFresh!\n");
		FreshBase();
		for(i=0;i<AWTPCfg.TX_LOCAL;i++)
		{
			for(j=0;j<AWTPCfg.RX_LOCAL;j++)
			{
				AW_Base.ChipBase[i][j] = AW_Base.Base[i][j];
			}
		}
	}
	return 0;
}

#ifdef NEWBASE_PROCESS

extern void Cali_Refresh(void);
void  NewBaseCompensate(void)
{
	signed char TraceStep,i,j;
	unsigned char k,calc,index,FlagCompensate;

	TraceStep = TRACE_STEP;

	if(AW_Base.CompensateFlag == TRACE_COMPENSATE_NORMAL)
	{
		for(i=0; i<AWTPCfg.TX_LOCAL; i++)
		{
			for(j=0; j<AWTPCfg.RX_LOCAL; j++)
			{
				if((Diff[i][j] > TRACE_THRES) && (AW_Base.Flag[i][j] == 0))
				{
					if(AW_Base.BaseCnt[i][j] > BASE_FRESH_CNT)
					{
						AW_Base.BaseCnt[i][j] = 0;
						AW_Base.Base[i][j] += TraceStep;
					}
					else
					{
						AW_Base.BaseCnt[i][j]++;
					}
				}
				if((Diff[i][j] < -TRACE_THRES) && (AW_Base.Flag[i][j] == 0))
				{
					if(AW_Base.BaseCnt[i][j] < -BASE_FRESH_CNT)
					{
						AW_Base.BaseCnt[i][j] = 0;
						AW_Base.Base[i][j] -= TraceStep;
					}
					else
					{
						AW_Base.BaseCnt[i][j]--;
					}
				}
			}
		}
	}	
}

void BaseNormalCheck()
{
	unsigned short i,j,k;
	unsigned short NormalizeData;

	if(AW_Base.CompensateState == BASE_INITIAL)
	{
		k = 0;
		for(i=0;i<AWTPCfg.TX_LOCAL;i++)
		{
			for(j=0;j<AWTPCfg.RX_LOCAL;j++)
			{
				NormalizeData = AW_Base.Base[i][j] + Diff[i][j] + AW_Cali.SOFTOFFSET[i][j];
				
				if(AW_Base.Flag[i][j] == 1)		//maybe finger touch at init,    check if rawdata < 8450
				{
					if((NormalizeData < BASE_VALUE+BASE_NORMALTH) && (NormalizeData > BASE_VALUE-BASE_NORMALTH))
					{
						AW_Base.Flag[i][j] = 0;
						AW_Base.Base[i][j] = AW_Base.Base[i][j]+Diff[i][j];
					}
					else if(NormalizeData < BASE_VALUE - BASE_TH)		//
					{
						AW_Base.Flag[i][j] = -1;
						k++;
					}
					else
					{
						k++;
					}
				}
				else if(AW_Base.Flag[i][j] == -1)	//maybe water at init,   check if rawdata > 8350
				{
					if((NormalizeData > BASE_VALUE-BASE_NORMALTH) && (NormalizeData < BASE_VALUE+BASE_NORMALTH))
					{
						AW_Base.Flag[i][j] = 0;
						AW_Base.Base[i][j] = AW_Base.Base[i][j]+Diff[i][j];
					}
					else if(NormalizeData > BASE_VALUE + BASE_TH)
					{
						AW_Base.Flag[i][j] = 1;
						k++;
					}
					else
					{
						k++;
					}
				}
				else if(AW_Base.Flag[i][j] == 0)
				{
					if(NormalizeData > BASE_VALUE + BASE_TH)
					{
						AW_Base.Flag[i][j] = 1;
						k++;
					}
					else if(NormalizeData < BASE_VALUE - BASE_TH)		//
					{
						AW_Base.Flag[i][j] = -1;
						k++;
					}
				}
			}
		}

		printk("%d point not stable \n", k);
		if(k == 0)
		{
			printk("Base stable! \n");
			AW_Base.CompensateState = BASE_STABLE;

			for(i=0;i<AWTPCfg.TX_LOCAL;i++)
			{
				for(j=0;j<AWTPCfg.RX_LOCAL;j++)
				{
					
				}
			}
			AW_Base.FrameCnt = 0;
		}
		else
		{
			AW_Base.FrameCnt++;
		}

		if(AW_Base.FrameCnt > FAST_FRAME*60*2)	// 2 min  need modify SOFTOFFSET
		{
			k = 0;
			for(i=0;i<AWTPCfg.TX_LOCAL;i++)
			{
				for(j=0;j<AWTPCfg.RX_LOCAL;j++)
				{
					 if((AW_Base.Flag[i][j] != 0)&&((Diff[i][j] > -16) && (Diff[i][j] < 16)))		// SOFTOFFSET error, need modify 
					 {
					 	AW_Cali.SOFTOFFSET[i][j] = 8400 - (AW_Base.Base[i][j] + Diff[i][j]);
						AW_Base.Flag[i][j] = 0;
					 }
					 else
					 {
					 	k++;
					 }
				}
			}
			if(k > 0)		//still have point that SOFTOFFSET not correct
			{
				
			}
			else
			{
				printk("refresh softoffset \n");
				Cali_Refresh();
			}
		}
	}
}

void NewPeakValidCheck(void)		//check if water at TP
{
	signed char i,j;
    signed char k;
    short temp;
    unsigned char X,Y;
	STRUCTPEAKSTATUS *pstructPeak;

    signed char m,n;
    signed short DiffTemp;
	//unsigned char ChkPointsNum = 5;

    unsigned char CurrentPointNum = AW_Peak.CurrentPointNum;
    unsigned char CurrentNegPointNum = AW_Peak.CurrentNegPointNum;

	for (i = 0; i < CurrentPointNum; i++)
	{
		Y = AW_Peak.Peak[i][0]>>1;
        X = AW_Peak.Peak[i][1]>>1;

		// Remove Pos peak caused by water
		temp = 0;
		for (m = Y-1; m <= Y+1; m++)
		{
			for (n = X-1; n <= X+1; n++)
			{
				if ((m >= 0) && (m < AWTPCfg.TX_LOCAL) && (n >= 0) && (n < AWTPCfg.RX_LOCAL))
				{
					temp += Diff[m][n];
				}
			}
		}

		n = 0;
		for(m = 0; m < AWTPCfg.TX_LOCAL; m ++)	
		{
			if (m == Y)
            {
                continue;
            }
			DiffTemp = (Diff[Y][X]+AW_Base.Base[Y][X]+AW_Cali.SOFTOFFSET[Y][X])\
                       -((Diff[m][X]+AW_Base.Base[m][X]+AW_Cali.SOFTOFFSET[m][X]));

			if (DiffTemp > THGROUP)
			{
				n++;
			}
		}
		
		if (((temp < THPEAK*2) && ((Y != AWTPCfg.TX_LOCAL-1) || (AWTPCfg.HAVE_KEY_LINE == 0)))
			|| (n < (AWTPCfg.TX_LOCAL>>1)))
		{
			if(i<4)		//remove peak
			{
				AW_Peak.Peak[i][0] = AW_Peak.Peak[i+1][0];
				AW_Peak.Peak[i][1] = AW_Peak.Peak[i+1][1];
			}
			else
			{
				AW_Peak.Peak[i][0] = 0xFF;
				AW_Peak.Peak[i][1] = 0xFF;
			}
			CurrentPointNum--;
			i--;
		}
	}
	AW_Peak.CurrentPointNum = CurrentPointNum;
	
}

char BaseProcess(void)
{
	unsigned char ret = 0;
	
	BaseNormalCheck();
	
	if((AW_Peak.CurrentPointNum == 0) && (AW_Peak.CurrentNegPointNum == 0))	
	{
		AW_Base.CompensateFlag = TRACE_COMPENSATE_NORMAL;
	}
	else
	{
		AW_Base.CompensateFlag = TRACE_COMPENSATE_NONE;
	}

	NewBaseCompensate();

	NewPeakValidCheck();

	if(AW5306_WorkMode != RawDataMode)
	{
		ret = BaseCheck();
	}
	return ret;
}
#else
char BaseProcess(void)
{
	unsigned char ret = 0;
	unsigned char TemperatureDrift;

	if (AW_Base.CompensateState == BASE_FAST_TRACE)
	{
		if ((AW_Peak.CurrentPointNum == 0) && (AW_Peak.CurrentNegPointNum == 0))
		{
			if (AW_Base.FrameCnt >= 2)
			{
				AW_Base.FrameCnt = 0;
				AW_Base.CompensateState = BASE_STABLE;
			}
			else
			{
				AW_Base.FrameCnt++;
			}
			AW_Base.CompensateFlag = TRACE_COMPENSATE_NORMAL;
		}
		else
		{
			AW_Base.FrameCnt = 0;
			AW_Base.CompensateFlag = TRACE_COMPENSATE_HIGH;
		}
		 
	}
	else if (AW_Base.CompensateState == BASE_STABLE)
	{		
		TemperatureDrift = CheckTemperatureDrift();

		if (TemperatureDrift == TRACE_TEMP_INC)
		{
			AW_Base.CompensateFlag = TRACE_COMPENSATE_TEMPINC; 
		}
		else if(TemperatureDrift == TRACE_TEMP_DEC)
		{
			AW_Base.CompensateFlag = TRACE_COMPENSATE_TEMPDEC; 
		}
		else if((AW_Peak.CurrentPointNum == 0) && (AW_Peak.CurrentNegPointNum == 0))
		{
			AW_Base.CompensateFlag = TRACE_COMPENSATE_NORMAL;
		}
		else
		{
		//	check the peak
			AW_Base.CompensateFlag = TRACE_COMPENSATE_NONE;  
		}
		PeakValidCheck();
				
	}

	BaseCompensate();
	PeakCompensate();
	ret = BigAreaProcess();
	//AWPeakCompensate();

	if(AW5306_WorkMode != RawDataMode)
	{
		if(ret == 0)
		{
			ret = BaseCheck();
		}
	}
	return ret;
}

#endif
