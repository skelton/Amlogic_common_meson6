/*
 * Amlogic BOARD ENV SYSFS
 *
 * Copyright (C) 2010 Amlogic Corporation
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/*Board env config for f11ref 
 **********************************************************************************
 									WIFI
 **********************************************************************************
 * GPIOA_7	wifi_type		details
 * 	  L		0			BCM40181(without BT)/GB9662
 *	  H		1			BCM40183(with BT)/GB9663
 **********************************************************************************
 									LCD
 **********************************************************************************
 * GPIOA_8	,	panel_size					details  
 *	  L				0						   8.0inch
 *	  H				1						   8.1inch
 **********************************************************************************
  **********************************************************************************
 * GPIOA_9	,	panel_type					details  
 *	  L				0						   2D
 *	  H				1						   3D
 **********************************************************************************
 									REAR CAMERA SELECTION
 **********************************************************************************
 * GPIOA_5  	rear_camera_type		details
 *	  L						0			OV2665	
 *	  H						1			GT2005
 **********************************************************************************
 									EXTENDED SETTING
 **********************************************************************************
 * GPIOA_6  					reserved			details
 *	  L								0				(reserved)
 *	  H								1				(reserved)
 **********************************************************************************
 ***********************************************************************************
 * GPIOA_18		gsensor_type					details
 * 	  L			0								MMA8452
 *	  H			1								BMA222
 ***********************************************************************************
 */

 
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <mach/register.h>
#include <mach/gpio.h>
#include <mach/gpio_data.h>
#include <mach/pinmux.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <plat/io.h>

static struct platform_device *pdev;

/* Sysfs Files */
static ssize_t wifi_type_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int val=0;
	
	aml_clr_reg32_mask(P_PAD_PULL_UP_REG0,1<<7);
	gpio_set_status(PAD_GPIOA_7,gpio_status_in);
	
	val = gpio_in_get(PAD_GPIOA_7);
	return sprintf(buf, "%d\n", val);
}

static ssize_t panel_type_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int val=0;
	
	aml_clr_reg32_mask(P_PAD_PULL_UP_REG0,1<<9);
	gpio_set_status(PAD_GPIOA_9,gpio_status_in);
	
	val = gpio_in_get(PAD_GPIOA_9);	
	return sprintf(buf, "%d\n", val);
}

static ssize_t gsensor_type_show(struct device *dev, 
                   struct device_attribute *attr, char *buf)
{
	int val=0;

	aml_clr_reg32_mask(P_PAD_PULL_UP_REG0,1<<18);
	gpio_set_status(PAD_GPIOA_18,gpio_status_in);
	
	val = gpio_in_get(PAD_GPIOA_18);
    return sprintf(buf, "%d\n", val);
}

static struct class_attribute board_env_class_attrs[] = {
	__ATTR_RO(wifi_type),
	__ATTR_RO(panel_type),
	__ATTR_RO(gsensor_type),
	__ATTR_NULL
};

static struct class board_env_class = {    
	.name = "board_env",    
	.class_attrs = board_env_class_attrs,

};

/* Device model stuff */
static int board_env_probe(struct platform_device *dev)
{
	printk(KERN_INFO "board_env: device successfully initialized.\n");
	return 0;
}

static struct platform_driver board_env_driver = {
	.probe = board_env_probe,
	.driver	= {
		.name = "board_env",
		.owner = THIS_MODULE,
	},
};

/* Module stuff */
static int __init board_env_init(void)
{
	int ret;

	ret = platform_driver_register(&board_env_driver);
	if (ret)
		goto out;

	pdev = platform_device_register_simple("board_env", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		ret = PTR_ERR(pdev);
		goto out_driver;
	}

	ret = class_register(&board_env_class);
	if (ret)
		goto out_device;

	printk(KERN_INFO "board_env: driver successfully loaded.\n");
	return 0;

out_device:
	platform_device_unregister(pdev);
out_driver:
	platform_driver_unregister(&board_env_driver);
out:
	printk(KERN_WARNING "board_env: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static void __exit board_env_exit(void)
{
	class_unregister(&board_env_class);
	platform_device_unregister(pdev);
	platform_driver_unregister(&board_env_driver);	

	printk(KERN_INFO "board_env: driver unloaded.\n");
}

module_init(board_env_init);
module_exit(board_env_exit);

MODULE_DESCRIPTION("Amlogic BOARD ENV SYSFS");
MODULE_LICENSE("GPL");
