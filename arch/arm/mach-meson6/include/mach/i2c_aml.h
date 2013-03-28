/*
 * arch/arm/mach-meson/include/mach/i2c-aml.h
 */

 /* just a sample. you can add private i2c platform data*/
#ifndef  _MACH_AML_I2C_H
#define _MACH_AML_I2C_H

#include <plat/platform_data.h>

struct aml_i2c_platform_a {
	plat_data_public_t  public ;
       struct resource  resource;
	/* local settings */
	int udelay;
       int reserve; 		
	int timeout;		
                 
};
struct aml_i2c_platform_b {
	plat_data_public_t  public ;
       struct resource resource;
	/* local settings */
	int freq;		
	int wait;		/* in jiffies */	
};
enum i2c_resource_index{
    I2C_RES_MASTER_A,
    I2C_RES_MASTER_B,
    I2C_RES_SLAVE,
};
enum i2c_dev_index{
    I2C_DEV_A,
    I2C_DEV_B,
    I2C_DEV_C,
    I2C_DEV_D,
};
#endif