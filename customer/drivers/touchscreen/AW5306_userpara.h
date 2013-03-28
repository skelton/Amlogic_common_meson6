#ifndef AW5306_USERPARA_H

#define AW5306_USERPARA_H

typedef struct {
	unsigned char	TX_LOCAL;	//					15		//TX number of TP
	unsigned char	RX_LOCAL;	//					10		//RX number of TP
	unsigned char	RX_INV_ORDER;	// RX mapping in inverted order
//	unsigned char	BAD_ADAPTOR_TH;	// BAD ADAPTOR detection threshold
	unsigned char	HAVE_KEY_LINE;	// 0: no KEY line, 1: have key line on TX line TX_LOCAL-1
	unsigned char	KeyLineValid[16];


	unsigned short	MAPPING_MAX_X;	//   320
	unsigned short	MAPPING_MAX_Y;	//   460

	unsigned short	K_X;	//       ((MAPPING_MAX_X - 1)*256)/(RX_LOCAL*POS_PRECISION - 1)  //192
	unsigned short	K_Y;	//	   ((MAPPING_MAX_Y - 1)*256)/(TX_LOCAL*POS_PRECISION - 1)   //195 

	unsigned short  FLYING_TH;
	unsigned short 	MOVING_TH;
	unsigned short 	MOVING_ACCELER;
	unsigned char 	FIRST_CALI;
	unsigned char	DEBUG_SWITCH;
	unsigned char	MULTI_SCANFREQ;
	unsigned char	ESD_PROTECT;

	unsigned short	GainClbDeltaMin;	// Expected minimum delta for GAIN calibration
	unsigned short	GainClbDeltaMax;	// Expected maximum delta for GAIN calibration
	unsigned short	OffsetClbExpectedMin;	// Expected minimum data for OFFSET calibration
	unsigned short	OffsetClbExpectedMax;	// Expected minimum data for OFFSET calibration
	unsigned short	RawDataDeviation;	// Maximum deviation in a frame
	unsigned short	CacMultiCoef;

}AW5306_UCF;

void AW5306_User_Init(void);

#endif
