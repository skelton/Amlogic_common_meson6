/*
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * Initial Code:
 *	Robbie Cao
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

/*
 * Definitions for afa750 accelorometer sensor chip.
 */
#ifndef __AFA750_H__
#define __AFA750_H__

#include <linux/ioctl.h>

#define AFA750_I2C_NAME		"afa750"

/*
 * This address comes must match the part# on your target.
 * Address to the sensor part# support as following list:
 *   AFA750	- 0x1C
 * Please refer to sensor datasheet for detail.
 */
#define AFA750_I2C_ADDR		0x1c

/* AFA750 register address */
//#define AFA750_REG_CTRL		0x2a
//#define AFA750_REG_DATA		0x01
//#define AFA750_XYZ_DATA_CFG		0x0e
/* AFA750 control bit */
#define AFA750_CTRL_PWRON_1		0x20	/* acceleration samples 20ms */
#define AFA750_CTRL_PWRON_2		0x28	/* acceleration samples 80ms */
#define AFA750_CTRL_PWRON_3		0x30	/* acceleration samples 160ms */
#define AFA750_CTRL_PWRON_4		0x38	/* acceleration samples 640ms */
#define AFA750_CTRL_PWRDN		0x80	/* power donw */
#define AFA750_CTRL_MODE_2G		0x00
#define AFA750_CTRL_MODE_4G		0x01
#define AFA750_CTRL_MODE_8G		0x02
#define AFA750_CTRL_ACTIVE		0x01	/* ACTIVE */



/* Use 'm' as magic number */
#define AFA750_IOM			'm'

/* IOCTLs for AFA750 device */
#define AFA750_IOC_PWRON		_IO (AFA750_IOM, 0x00)
#define AFA750_IOC_PWRDN		_IO (AFA750_IOM, 0x01)
#define AFA750_IOC_READXYZ		_IOR(AFA750_IOM, 0x05, int[3])
#define AFA750_IOC_READSTATUS		_IOR(AFA750_IOM, 0x07, int[3])
#define AFA750_IOC_SETDETECTION	_IOW(AFA750_IOM, 0x08, unsigned char)

#define AFA750_IOC_SET_CALI    _IO (AFA750_IOM, 0x11) 
#define AFA750_IOC_GET_CALI    _IO (AFA750_IOM, 0x12)

struct afa750_data_t{
	struct input_dev *input;
	struct delayed_work work;
	atomic_t delay;
	atomic_t enable;
};

#endif /* __AFA750_H__ */


