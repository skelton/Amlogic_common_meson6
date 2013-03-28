#ifndef _MESON_COMMON_DATA_H_
#define _MESON_COMMON_DATA_H_
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <plat/platform.h>
#include <plat/plat_dev.h>
#include <plat/platform_data.h>

#include <linux/io.h>
#include <plat/io.h>
void __init meson_map_io(void);
void __init meson_fixup(struct machine_desc *mach, struct tag *tag, char **cmdline, struct meminfo *m);


#endif
