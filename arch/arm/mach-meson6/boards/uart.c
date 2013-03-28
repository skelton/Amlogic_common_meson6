#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <plat/io.h>
#include "common-data.h"

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <mach/irqs.h>
#include <linux/io.h>
#include <plat/io.h>


/**
 * Meson 3 uart resource description
 */
#define UART_BASEADDRAO (P_AO_UART_WFIFO) //AO UART
#define UART_BASEADDR0  (CBUS_REG_ADDR(UART0_WFIFO))
#define UART_BASEADDR1  (CBUS_REG_ADDR(UART1_WFIFO))
#define UART_BASEADDR2  (CBUS_REG_ADDR(UART2_WFIFO))
#define UART_LEVEL_AO	64
#define UART_LEVEL_0	128
#define UART_LEVEL_1	64
#define UART_LEVEL_2	64
static const char  uart_clk_src[]="clk81";
#define UART_VAR_RESOURCE(id)					\
		{                						\
				[0]={                           \
					.start=UART_BASEADDR##id,   \
                    .end=(UART_BASEADDR##id)+20, \
					.flags=IORESOURCE_MEM       \
				},                              \
				[1]={                           \
					.start=INT_UART_##id,       \
					.flags=IORESOURCE_IRQ       \
				},                              \
				[2]={                           \
					.start=UART_LEVEL_##id,     \
					.end=1, 					\
					.name="feature"             \
				} ,                             \
				[3]={                           \
					.start=(resource_size_t)&uart_clk_src[0], \
					.name="clksrc"              \
				}                               \
			}                                    
static  pinmux_item_t   uart_pins[]={
		{
			.reg=PINMUX_REG(AO),
			.setmask=3<<11
		},
		PINMUX_END_ITEM
	};
static  pinmux_set_t   aml_uart_ao={
	.chip_select=NULL,
	.pinmux=&uart_pins[0]
};	

mesonplat_cpu_resource_early_initcall_begin(earlycon,meson_uart,meson_uart);
	module_object(AO,UART_VAR_RESOURCE(AO),&aml_uart_ao);
mesonplat_cpu_resource_early_initcall_end(earlycon,meson_uart,meson_uart);

mesonplat_cpu_resource_pure_initcall_begin(meson_uart,meson_uart);
	module_object(0,UART_VAR_RESOURCE(0),NULL);
	module_object(1,UART_VAR_RESOURCE(1),NULL);
	module_object(2,UART_VAR_RESOURCE(2),NULL);
mesonplat_cpu_resource_pure_initcall_end(meson_uart,meson_uart);
	

