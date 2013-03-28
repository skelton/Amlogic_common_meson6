/*
 *
 * arch/arm/mach-meson/meson.c
 *
 *  Copyright (C) 2010 AMLOGIC, INC.
 *
 * License terms: GNU General Public License (GPL) version 2
 * Platform machine definition.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/device.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <plat/platform.h>
#include <plat/plat_dev.h>
#include <plat/platform_data.h>
#include <plat/lm.h>
#include <plat/regops.h>
#include <mach/map.h>
#include <mach/i2c_aml.h>
#include <mach/usbclock.h>
#include "common-data.h"

static __init void meson_init_machine(void)
{
	mesonplat_register_device("meson_uart","AO",NULL);
    //~ mesonplat_register_device("meson_uart","1",NULL);
}
static __init void meson_init_early(void)
{///boot seq 1
	mesonplat_register_device_early("meson_uart","AO",NULL);
	parse_early_param();
	/* Let earlyprintk output early console messages */
	early_platform_driver_probe("earlyprintk", 4, 0);
	
}


MACHINE_START(MESON6_SKT, "Meson 6 socket board")
    .boot_params    = BOOT_PARAMS_OFFSET,
    .map_io         = meson_map_io,///2
    .init_early		= meson_init_early,///3
    .init_irq       = meson_init_irq,///0
    .timer          = &meson_sys_timer,
    .init_machine   = meson_init_machine,
    .fixup          = meson_fixup,///1
MACHINE_END
