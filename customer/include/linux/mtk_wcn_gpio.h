#ifndef _MTK_WCN_GPIO_H_
#include <linux/io.h>
#include <plat/io.h>
#include <mach/gpio_data.h>
#include <mach/pinmux.h>
#include <mach/gpio.h>

#define EXT_OSC_EN  0

#define MTK_PMU_PIN PAD_GPIOAO_3
#define MTK_RST_PIN PAD_GPIOC_7
#define MTK_LDO_PIN PAD_GPIOX_1
#define MTK_GPS_LNA PAD_GPIOE_9
#define MTK_GPS_SYNC PAD_GPIOC_4
#define MTK_RTC_PIN PAD_GPIOA_12
#define MTK_BGF_INT  PAD_GPIOX_10
#define MTK_ALL_INT PAD_GPIOA_11


#define mtk_oob_irq  INT_GPIO_4
#define mtk_bgf_irq  INT_GPIO_5
#define mtk_bgf_irq_no   5

#endif