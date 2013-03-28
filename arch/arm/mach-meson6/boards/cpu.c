#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
static int __init meson_cache_init(void)
{

    /*
     * Early BRESP, I/D prefetch enabled
     * Non-secure enabled
     * 128kb (16KB/way),
     * 8-way associativity,
     * evmon/parity/share disabled
     * Full Line of Zero enabled
         * Bits:  .111 .... .100 0010 0000 .... .... ...1
     */
    l2x0_init((void __iomem *)IO_PL310_BASE, 0x7c420001, 0xff800fff);
    return 0;

}
early_initcall(meson_cache_init);
#endif
