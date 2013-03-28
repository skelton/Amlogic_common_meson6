/*! This software program is licensed subject to the GNU General Public License
 *      (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *  (C) Copyright 2012 Bosch Sensortec GmbH
 *  All Rights Reserved
 * 
 * @file    bmainit.h
 * @date    Sep. 3rd, 2012 initial
 *
 *
 * @brief
 *
 * This file is an definition of register backup and restore.
 *
 * @detail
 * This file contains the definition of register backup and restore.
 * Detail logic see the bmainit.c
 *
 */

#ifndef __BMAINIT__
#define __BMAINIT__

#define BMAINIT_DEBUG

#define BMA_CHIP_ID  0xF9

/*define 2 backup files to verify the regisers vale backup*/
#define BMA_REG_BACKUP_FILE "/data/reg_backedup"	/*backup file contain the register value*/
#define BMA_REG_BACKUP_FLAG "/data/flag_sensor_backedup"	/*backup flag to indicate whether the data backup*/


/*bma control block structure
 * i2c_read: 	read data from the I2C device.
 * i2c_write: 	write data to the I2C device.
 * fs_read:	read data from a file.
 * fs_write:	write data to a file.
 * msleep:	sleep function.
 * sensor_i2c_id:	device I2C address.
*/
struct bma_callback {
	int (*i2c_read)( unsigned char chip, unsigned int addr, int alen, unsigned char *buffer, int len);
	int (*i2c_write)(unsigned char chip, unsigned int addr, int alen, const unsigned char *buffer, int len);
	int (*fs_read)(const unsigned char *file_name, unsigned char * addr, int count);
	int (*fs_write)(const unsigned char *file_name, unsigned char * addr, int count);
	void (*msleep)(unsigned int count);
	unsigned char sensor_i2c_id;
};

int detect_bma_sensor(struct bma_callback * pCB);
int backup_or_restore_i2c( struct bma_callback * pCB);

#endif /*__BMAINIT__*/

