/*
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *  Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <plat/io.h>
#include <asm/smp_scu.h>
#include <asm/hardware/gic.h>
#include <asm/cacheflush.h>
#include <asm/mach-types.h>

extern void meson_secondary_startup(void);
extern inline void meson_set_cpu_ctrl_reg(int value);

/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen".
 */
volatile int pen_release = -1;

static DEFINE_SPINLOCK(boot_lock);

void __cpuinit platform_secondary_init(unsigned int cpu)
{

#if (defined CONFIG_ARM_GIC) && CONFIG_ARM_GIC
	/*
	 * if any interrupts are already enabled for the primary
	 * core (e.g. timer irq), then they will not have been enabled
	 * for us: do so
	 */
	gic_secondary_init(0);
#endif    

	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	pen_release = -1;
	smp_wmb();

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}


int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
    unsigned long timeout;
    /*
     * Set synchronisation state between this boot processor
     * and the secondary one
     */
    spin_lock(&boot_lock);
    pen_release = cpu;
    __cpuc_flush_dcache_area((void *) &pen_release, sizeof(pen_release));
    outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));

    /*
     * Write the address of secondary startup routine into the
     * AuxCoreBoot1 where ROM code will jump and start executing
     * on secondary core once out of WFE
     * A barrier is added to ensure that write buffer is drained
     */
    aml_write_reg32((uint32_t)(IO_AHB_BASE + 0x1ff84),
            (const uint32_t)virt_to_phys(meson_secondary_startup));


    //aml_write_reg32(IO_AHB_BASE + 0x1ff80, (1 << cpu) | 1);
    meson_set_cpu_ctrl_reg((1 << cpu) | 1);

    smp_wmb();

    /*
     * Send a 'sev' to wake the secondary core from WFE.
     * Drain the outstanding writes to memory
     */mb();
#ifndef CONFIG_MESON6_SMP_HOTPLUG
    dsb_sev();
#else
    gic_raise_softirq(cpumask_of(cpu), 0);
#endif
    timeout = jiffies + (10 * HZ);
    while (time_before(jiffies, timeout))
        {
            smp_rmb();
            if (pen_release == -1)
                break;

            udelay(10);
        }
    /*
     * now the secondary core is starting up let it run its
     * calibrations, then wait for it to finish
     */
    spin_unlock(&boot_lock);

    return pen_release != -1 ? -ENOSYS : 0;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system. The msm8x60
 * does not support the ARM SCU, so just set the possible cpu mask to
 * NR_CPUS.
 */
void __init smp_init_cpus(void)
{
	unsigned int i;

	for (i = 0; i < NR_CPUS; i++)
		set_cpu_possible(i, true);
#if (defined CONFIG_ARM_GIC) && CONFIG_ARM_GIC
        set_smp_cross_call(gic_raise_softirq);
#endif
}

void __init platform_smp_prepare_cpus(unsigned int max_cpus)
{
    int i;

    /*
     * Initialise the present map, which describes the set of CPUs
     * actually populated at the present time.
     */
    for (i = 0; i < max_cpus; i++)
        set_cpu_present(i, true);
    /*
     * Initialise the SCU and wake up the secondary core using
     * wakeup_secondary().
     */
    scu_enable((void __iomem *) IO_PERIPH_BASE);
    ///wakeup_secondary();

    ///    dsb_sev();


}
