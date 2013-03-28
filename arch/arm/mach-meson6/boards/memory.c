#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include "common-data.h"
#include <asm/page.h>

#define PHYS_MEM_START      (0x80000000)
#define PHYS_MEM_SIZE       (256*SZ_1M)
#define PHYS_MEM_END        (PHYS_MEM_START + PHYS_MEM_SIZE -1 )

/******** Reserved memory setting ************************/
#define RESERVED_MEM_START  (0x80000000+64*SZ_1M)   /*start at the second 64M*/

/******** CODEC memory setting ************************/
//  Codec need 16M for 1080p decode
//  4M for sd decode;
#define ALIGN_MSK           ((SZ_1M)-1)
#define U_ALIGN(x)          ((x+ALIGN_MSK)&(~ALIGN_MSK))
#define D_ALIGN(x)          ((x)&(~ALIGN_MSK))

/******** AUDIODSP memory setting ************************/
#define AUDIODSP_ADDR_START U_ALIGN(RESERVED_MEM_START) /*audiodsp memstart*/
#define AUDIODSP_ADDR_END   (AUDIODSP_ADDR_START+SZ_1M-1)   /*audiodsp memend*/

/******** Frame buffer memory configuration ***********/
#define OSD_480_PIX         (640*480)
#define OSD_576_PIX         (768*576)
#define OSD_720_PIX         (1280*720)
#define OSD_1080_PIX        (1920*1080)
#define OSD_PANEL_PIX       (800*480)
#define B16BpP  (2)
#define B32BpP  (4)
#define DOUBLE_BUFFER   (2)

#define OSD1_MAX_MEM        U_ALIGN(OSD_1080_PIX*B32BpP*DOUBLE_BUFFER)
#define OSD2_MAX_MEM        U_ALIGN(32*32*B32BpP)

/******** Reserved memory configuration ***************/
#define OSD1_ADDR_START     U_ALIGN(AUDIODSP_ADDR_END )
#define OSD1_ADDR_END       (OSD1_ADDR_START+OSD1_MAX_MEM - 1)
#define OSD2_ADDR_START     U_ALIGN(OSD1_ADDR_END)
#define OSD2_ADDR_END       (OSD2_ADDR_START +OSD2_MAX_MEM -1)

#if defined(CONFIG_AM_VDEC_H264)
#define CODEC_MEM_SIZE      U_ALIGN(32*SZ_1M)
#else
#define CODEC_MEM_SIZE      U_ALIGN(16*SZ_1M)
#endif
#define CODEC_ADDR_START    U_ALIGN(OSD2_ADDR_END)
#define CODEC_ADDR_END      (CODEC_ADDR_START+CODEC_MEM_SIZE-1)

/********VDIN memory configuration ***************/
#define VDIN_ADDR_START     U_ALIGN(OSD2_ADDR_END)
#define VDIN_ADDR_END       (VDIN_ADDR_START +CODEC_MEM_SIZE -1)


#if defined(CONFIG_AM_DEINTERLACE_SD_ONLY)
#define DI_MEM_SIZE         (SZ_1M*3)
#else
#define DI_MEM_SIZE         (SZ_1M*15)
#endif
#define DI_ADDR_START       U_ALIGN(CODEC_ADDR_END)
#define DI_ADDR_END         (DI_ADDR_START+DI_MEM_SIZE-1)

#define STREAMBUF_MEM_SIZE          (SZ_1M*7)
#define STREAMBUF_ADDR_START        U_ALIGN(DI_ADDR_END)
#define STREAMBUF_ADDR_END      (STREAMBUF_ADDR_START+STREAMBUF_MEM_SIZE-1)

#define RESERVED_MEM_END    (STREAMBUF_ADDR_END)


/***********************************************************************
 * IO Mapping
 **********************************************************************/
/*
#define IO_CBUS_BASE        0xf1100000  ///2M
#define IO_AXI_BUS_BASE     0xf1300000  ///1M
#define IO_PL310_BASE       0xf2200000  ///4k
#define IO_PERIPH_BASE      0xf2300000  ///4k
#define IO_APB_BUS_BASE     0xf3000000  ///8k
#define IO_DOS_BUS_BASE     0xf3010000  ///64k
#define IO_AOBUS_BASE       0xf3100000  ///1M
#define IO_USB_A_BASE       0xf3240000  ///256k
#define IO_USB_B_BASE       0xf32C0000  ///256k
#define IO_WIFI_BASE        0xf3300000  ///1M
#define IO_SATA_BASE        0xf3400000  ///64k
#define IO_ETH_BASE         0xf3410000  ///64k

#define IO_SPIMEM_BASE      0xf4000000  ///64M
#define IO_A9_APB_BASE      0xf8000000  ///256k
#define IO_DEMOD_APB_BASE   0xf8044000  ///112k
#define IO_MALI_APB_BASE    0xf8060000  ///128k
#define IO_APB2_BUS_BASE    0xf8000000
#define IO_AHB_BASE         0xf9000000  ///128k
#define IO_BOOTROM_BASE     0xf9040000  ///64k
#define IO_SECBUS_BASE      0xfa000000
#define IO_EFUSE_BASE       0xfa000000  ///4k
*/
static __initdata struct map_desc meson_io_desc[] = {
    {
        .virtual    = IO_CBUS_BASE,
        .pfn        = __phys_to_pfn(IO_CBUS_PHY_BASE),
        .length     = SZ_2M,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_AXI_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_AXI_BUS_PHY_BASE),
        .length     = SZ_1M,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_PL310_BASE,
        .pfn        = __phys_to_pfn(IO_PL310_PHY_BASE),
        .length     = SZ_4K,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_PERIPH_BASE,
        .pfn        = __phys_to_pfn(IO_PERIPH_PHY_BASE),
        .length     = SZ_4K,
        .type       = MT_DEVICE,
    } , {
           .virtual    = IO_APB_BUS_BASE,
           .pfn        = __phys_to_pfn(IO_APB_BUS_PHY_BASE),
           .length     = SZ_8K,
           .type       = MT_DEVICE,
       } , {

           .virtual    = IO_DOS_BUS_BASE,
           .pfn        = __phys_to_pfn(IO_DOS_BUS_PHY_BASE),
           .length     = SZ_64K,
           .type       = MT_DEVICE,
       } , {
           .virtual    = IO_AOBUS_BASE,
        .pfn        = __phys_to_pfn(IO_AOBUS_PHY_BASE),
        .length     = SZ_1M,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_AHB_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_AHB_BUS_PHY_BASE),
        .length     = SZ_8M,
        .type       = MT_DEVICE,
    } , {
            .virtual    = IO_SPIMEM_BASE,
            .pfn        = __phys_to_pfn(IO_SPIMEM_PHY_BASE),
            .length     = SZ_64M,
            .type       = MT_ROM,
    } , {
        .virtual    = IO_APB2_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_APB2_BUS_PHY_BASE),
        .length     = SZ_512K,
        .type       = MT_DEVICE,
    } , {
		.virtual    = IO_AHB_BASE,
		.pfn        = __phys_to_pfn(IO_AHB_PHY_BASE),
		.length     = SZ_128K,
		.type       = MT_DEVICE,
    } , {
		.virtual    = IO_BOOTROM_BASE,
		.pfn        = __phys_to_pfn(IO_BOOTROM_PHY_BASE),
		.length     = SZ_64K,
		.type       = MT_ROM,
    } , {
    		.virtual    = IO_SECBUS_BASE,
    		.pfn        = __phys_to_pfn(IO_SECBUS_PHY_BASE),
    		.length     = SZ_4K,
    		.type       = MT_DEVICE,
    }, {
        .virtual    = PAGE_ALIGN(__phys_to_virt(RESERVED_MEM_START)),
        .pfn        = __phys_to_pfn(RESERVED_MEM_START),
        .length     = RESERVED_MEM_END - RESERVED_MEM_START + 1,
        .type       = MT_DEVICE,
    }
};

void __init meson_map_io(void)
{
    iotable_init(meson_io_desc, ARRAY_SIZE(meson_io_desc));
}

void __init meson_fixup(struct machine_desc *mach, struct tag *tag, char **cmdline, struct meminfo *m)
{
    mach->video_start    = RESERVED_MEM_START;
    mach->video_end      = RESERVED_MEM_END;

    struct membank *pbank;
    m->nr_banks = 0;
    pbank = &m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(PHYS_MEM_START);
    pbank->size  = SZ_64M & PAGE_MASK;
    m->nr_banks++;
    pbank = &m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(RESERVED_MEM_END + 1);
    pbank->size  = (PHYS_MEM_END - RESERVED_MEM_END) & PAGE_MASK;
    m->nr_banks++;
}
