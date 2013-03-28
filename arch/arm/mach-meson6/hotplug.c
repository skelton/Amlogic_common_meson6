/*
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <plat/regops.h>

#include <linux/module.h>

extern inline void meson_set_cpu_ctrl_reg(int value);

int platform_cpu_kill(unsigned int cpu)
{
	return 1;
}

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void platform_cpu_die(unsigned int cpu)
{
	//aml_write_reg32((IO_AHB_BASE + 0x1ff80),0);
	meson_set_cpu_ctrl_reg(0);
	flush_cache_all();
	dsb();
	dmb();

	for (;;) {
		/*
		 * Execute WFI
		 */
	    pr_debug("CPU%u: Enter WFI\n", cpu);
	        __asm__ __volatile__ ("wfi" : : : "memory");


		if (smp_processor_id() == cpu) {
			/*
			 * OK, proper wakeup, we're done
			 */
			if((aml_read_reg32(IO_AHB_BASE + 0x1ff80)&0x3) == ((1 << cpu) | 1))
			{
#ifdef CONFIG_MESON6_SMP_HOTPLUG
				/*
				 * Need invalidate data cache because of disable cpu0 fw
				 */
				extern  void v7_invalidate_dcache_all(void);
				v7_invalidate_dcache_all();
#endif
				break;
			}
		}
		pr_debug("CPU%u: spurious wakeup call\n", cpu);
	}
#ifdef CONFIG_MESON6_SMP_HOTPLUG
	/*
	 * Need invalidate data cache because of disable cpu0 fw
	 */
	extern  void v7_invalidate_dcache_all(void);
	v7_invalidate_dcache_all();
#endif
}

int platform_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	return cpu == 0 ? -EPERM : 0;
}

#ifdef CONFIG_MESON6_SMP_HOTPLUG
/*
 *disable_cpu_fw() & restore_cpu_fw() need call by cpu0 to disable&restore fw bit setting
 *Disable fw bit avoids spurious interrupt noisy which from cpu0 cache&TLB broadcast
 *to offline cpu1 .
 *
 */
static unsigned int ACTLR_FW=0;
static unsigned int hotplug_flag=0;
void disable_cpu_fw()
{
	__asm__ __volatile__ (
			"mrc p15, 0,%0, c1, c0, 1 \n"
			"ldr r1,=0xfffffffe \n"// FW bit
			"and r0,%0,r1 \n"
			"mcr p15, 0,r0, c1, c0, 1 \n"//Disable FW bit
			:"+r" (ACTLR_FW)
			:
			:"r0","r1","cc","memory");

	hotplug_flag=1;
}
EXPORT_SYMBOL(disable_cpu_fw);

void restore_cpu_fw()
{
	if(hotplug_flag)
	{
		__asm__ __volatile__ (
				"mcr p15, 0,%0, c1, c0, 1 \n"
				:
				:"r" (ACTLR_FW)
				:"cc","memory");
	}
}
EXPORT_SYMBOL(restore_cpu_fw);
#endif//#ifdef CONFIG_MESON6_SMP_HOTPLUG
