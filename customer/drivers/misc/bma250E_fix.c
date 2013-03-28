#include <linux/kernel.h>
#include "bmainit.h"

#define BLOCK_SIZE 26

#define printf(format, args...) \
	printk("<3>""BMA_INIT: %s(): " format, __func__, ## args)

/*
 * @brief This function is a CRC8 calculation.
 *
 * @detail Here gives more detail description of the function
 * @param p_MemAddr  	data buffer will calculate
 * @param BlockSize   	data length
 *
 * @return   CRC value.
 */
static int CalcCRC8(unsigned char *p_MemAddr, unsigned char BlockSize)
{
	unsigned char CrcReg = 0xFF; 
	unsigned char MemByte;
	unsigned char BitNo;	

	while (BlockSize)
	{
		MemByte = *p_MemAddr;
		for (BitNo = 0; BitNo < 8; BitNo++)
		{
			if ((CrcReg^MemByte) & 0x80)
			{
				CrcReg = (CrcReg << 1) ^ 0x11D; 
			}
			else
			{
				CrcReg <<= 1;
			}
			MemByte <<= 1;
		}
		BlockSize--;
		p_MemAddr++;
	}

	CrcReg = ~CrcReg;
	return CrcReg;
}

/*
 * @brief This function is the detection bma sensor implemention.
 *
 * @detail: This function try to detect bma sensor on the I2C bus. 
 * Here gives more detail description of the function
 * @param p_pCB  	bma control block pointer.
 *
 * @return   	0	found
 * @return	-1	not found
 */
int detect_bma_sensor(struct bma_callback * pCB)
{
	int i2c_id_ii = 0x10;
	int flag_sensor_found = 0;
	unsigned char chip_id;
	unsigned char buf = 0x00;

	s32 err = 0;

	if ((pCB == 0)
		||(pCB->i2c_read == 0)
		||(pCB->i2c_write == 0)
		||(pCB->msleep == 0)
		||(pCB->sensor_i2c_id == 0))
	{
		printf("Bad parameter.\n");	
		return -1;
	}

	err = pCB->i2c_read(pCB->sensor_i2c_id, 0x00, 1, &chip_id,1);
	if(err < 0)
		return -2;

	if ((chip_id &0xFF) == BMA_CHIP_ID)
	{
		flag_sensor_found = 1;
		i2c_id_ii = pCB->sensor_i2c_id;
	} 
	else
	{
		do{
			pCB->i2c_read(i2c_id_ii, 0x00, 1, &chip_id,1);
			if ((chip_id & 0xFC) == 0xF8)
			{
				flag_sensor_found = 1;
			} 
			else
			{
				i2c_id_ii++;
			}
		} while ((!flag_sensor_found)&&(i2c_id_ii <= 0x1F));
	}

	if (!flag_sensor_found)
	{
		printf("Sensor not found.\n");	
		return -1;		
	}

	pCB->i2c_write(i2c_id_ii,0x11,1, &buf, 1);		
	pCB->msleep(2);
	
	buf = 0xAA;
	pCB->i2c_write(i2c_id_ii,0x35,1, &buf, 1);		
	pCB->i2c_write(i2c_id_ii,0x35,1, &buf, 1);	
	
	pCB->i2c_read(i2c_id_ii, 0x05, 1, &buf,1);		
	buf = (buf & 0x1F) |0x80 ;
	pCB->i2c_write(i2c_id_ii, 0x05,1, &buf, 1);	

	buf = 0x0A;
	pCB->i2c_write(pCB->sensor_i2c_id,0x35,1, &buf, 1);

	return 0;
}

/*!
 * @brief This function is the backup and restore the critical register value logic implemention.
 *
 * @detail: This function try to load the bakcup file, If no bakcup before,
 * function will backup the register file and save it. 
 * Here gives more detail description of the function
 * @param p_pCB  	bma control block pointer.
 *
 * @return   	0	sucessful
 * @return	-1	failture
 */
int backup_or_restore_i2c( struct bma_callback * pCB)
{
	unsigned char regs[BLOCK_SIZE + 1] = {0};
	int ii;
	int write_to_nvm = 0;
	int read_from_nvm = 0;	
	unsigned char buf = 0x00;
	unsigned char flag_sensor_backedup = 0;
	unsigned char crc8;

	if ((pCB == 0)
		||(pCB->i2c_read == 0)
		||(pCB->i2c_write == 0)
		||(pCB->fs_read == 0)
		||(pCB->fs_write == 0)
		||(pCB->msleep == 0)
		||(pCB->sensor_i2c_id == 0))
	{
		printf("Bad parameter.\n");	
		return -1;
	}

	pCB->i2c_write(pCB->sensor_i2c_id,0x11,1, &buf, 1);		
	pCB->msleep(2);
	
	pCB->fs_read(BMA_REG_BACKUP_FLAG,&flag_sensor_backedup, 1);
	
	if (flag_sensor_backedup) 
	{ 
		read_from_nvm = pCB->fs_read(BMA_REG_BACKUP_FILE,regs,BLOCK_SIZE+1);		

		if (!read_from_nvm)
		{
			printf("cannot get saved data from NVM\n");					
			return -1;
		}
		
		#ifdef BMAINIT_DEBUG
		{
			char strbuf[4*(BLOCK_SIZE+1)] = {0};
			for (ii = 0; ii <= BLOCK_SIZE+1; ii++)
			{
				sprintf(strbuf + (3 * ii), "%02X ", regs[ii]);
			}
			printf("<bc> %s", strbuf);
		}
		#endif //BMAINIT_DEBUG

		crc8 = CalcCRC8(regs,BLOCK_SIZE);
		if (crc8 != regs[0x1A])
		{
			printf("Restore CRC check failed , crc8 = 0x%x regs[0x1A] = 0x%x\n",crc8,regs[0x1A]);
			return -1;
		}

		buf = 0xAA;
		pCB->i2c_write(pCB->sensor_i2c_id,0x35,1, &buf, 1); 	
		pCB->i2c_write(pCB->sensor_i2c_id,0x35,1, &buf, 1); 

		for ( ii = 0x00; ii <=0x1A; ii++)
		{
			pCB->i2c_write(pCB->sensor_i2c_id, ii, 1, &regs[ii],1);						
		}

		buf = 0x0A;
		pCB->i2c_write(pCB->sensor_i2c_id,0x35,1, &buf, 1);

		buf = 0x00;			
		for ( ii = 0x38; ii <=0x3A; ii++)
		{
			pCB->i2c_write(pCB->sensor_i2c_id, ii, 1, &buf, 1);
		}			
	}
	else
	{
		buf = 0xAA;
		pCB->i2c_write(pCB->sensor_i2c_id,0x35,1, &buf, 1); 	
		pCB->i2c_write(pCB->sensor_i2c_id,0x35,1, &buf, 1); 

		for ( ii = 0x00; ii <=0x1A; ii++)
		{
			pCB->i2c_read(pCB->sensor_i2c_id, ii, 1, &regs[ii],1);						
		}

		buf = 0x0A;
		pCB->i2c_write(pCB->sensor_i2c_id,0x35,1, &buf, 1);

		buf = 0x00;			
		for ( ii = 0x38; ii <=0x3A; ii++)
		{
			pCB->i2c_write(pCB->sensor_i2c_id, ii, 1, &buf, 1);
		}
		
		#ifdef BMAINIT_DEBUG
		{
			char strbuf[4*(BLOCK_SIZE+1)] = {0};
			for (ii = 0; ii <= BLOCK_SIZE+1; ii++)
			{
				sprintf(strbuf + (3 * ii), "%02X ", regs[ii]);
			}
			printf("<rc> %s", strbuf);
		}
		#endif //BMAINIT_DEBUG

		crc8 = CalcCRC8(regs,BLOCK_SIZE);
		if (crc8 == regs[0x1A])
		{
			write_to_nvm = (pCB->fs_write(BMA_REG_BACKUP_FILE,regs,BLOCK_SIZE+1) == BLOCK_SIZE+1);
			
			if (write_to_nvm)
			{
				flag_sensor_backedup = 1;
				pCB->fs_write(BMA_REG_BACKUP_FLAG,&flag_sensor_backedup, 1);					
			}
			else
			{
				printf("backup to NVM failed\n");
				return -1;
			}
		}
		else
		{
			printf("Backup CRC check failed , crc8 = 0x%x regs[0x1A] = 0x%x\n",crc8,regs[0x1A]);
			return -1;				
		}
	}

	return 0;
}
