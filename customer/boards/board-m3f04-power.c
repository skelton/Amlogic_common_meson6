/*
 * customer/boards/board-m3f04-power.c
 *
 * Copyright (C) 2011-2012 Amlogic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
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
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <plat/platform.h>
#include <plat/plat_dev.h>
#include <plat/platform_data.h>
#include <plat/lm.h>
#include <plat/regops.h>
#include <mach/am_regs.h>
#include <mach/clock.h>
#include <mach/map.h>
#include <mach/i2c_aml.h>
#include <mach/nand.h>
#include <mach/usbclock.h>
#include <mach/usbsetting.h>
#include <mach/gpio.h>
#include <mach/gpio_data.h>

#include "board-m3f04.h"

#ifdef CONFIG_SUSPEND
#include <mach/pm.h>
#endif

#include "board-m3f04-pinmux.h"

#include <linux/i2c-aml.h>
#include <linux/reboot.h>
#include <linux/saradc.h>
#include <linux/act8xxx.h>
#include <linux/bq27x00_battery.h>

static void m3ref_power_off(void)
{
    // AC in after power off press
    if (act8xxx_is_ac_online())
        kernel_restart("charging_reboot");

    // BL_PWM power off
    //set_gpio_val(GPIOD_bank_bit0_9(1), GPIOD_bit_bit0_9(1), 0);
    //set_gpio_mode(GPIOD_bank_bit0_9(1), GPIOD_bit_bit0_9(1), GPIO_OUTPUT_MODE);
    gpio_out(PAD_GPIOD_9, 0);

    // VCCx2 power down
    f04_set_vccx2(0);

    // Power hold down
    //set_gpio_val(GPIOAO_bank_bit0_11(6), GPIOAO_bit_bit0_11(6), 0);
    //set_gpio_mode(GPIOAO_bank_bit0_11(6), GPIOAO_bit_bit0_11(6), GPIO_OUTPUT_MODE);
    gpio_out(PAD_GPIOAO_11, 0);
}

/*
 *  nSTAT OUTPUT(GPIOA_21)  enable internal pullup
 *      High:       Full
 *      Low:        Charging
 */
static inline int battery_get_charge_status(void)
{
    int val;

    SET_CBUS_REG_MASK(PAD_PULL_UP_REG0, (1<<21));   //enable internal pullup
    //set_gpio_mode(GPIOA_bank_bit0_27(21), GPIOA_bit_bit0_27(21), GPIO_INPUT_MODE);
    //val = get_gpio_val(GPIOA_bank_bit0_27(21), GPIOA_bit_bit0_27(21));
    val = gpio_in_get(PAD_GPIOA_21);

    printk(KERN_DEBUG "%s() Charge status 0x%x\n", __FUNCTION__, val);
    return val;
}

/*
 *  Fast charge when CHG_CON(GPIOAO_11) is High.
 *  Slow charge when CHG_CON(GPIOAO_11) is Low.
 */
static int battery_set_charge_current(int level)
{
    //set_gpio_mode(GPIOAO_bank_bit0_11(11), GPIOAO_bit_bit0_11(11), GPIO_OUTPUT_MODE);
    //set_gpio_val(GPIOAO_bank_bit0_11(11), GPIOAO_bit_bit0_11(11), (level ? 1 : 0));
    gpio_out(PAD_GPIOAO_11, (level ? 1 : 0));
    return 0;
}

#ifdef CONFIG_PMU_ACT8942
static inline int battery_get_adc_value(void)
{
    return get_adc_sample(5);
}

/*
 *  When BAT_SEL(GPIOA_22) is High Vbat=Vadc*2
 */
static inline int battery_measure_voltage(void)
{
    int val;

    msleep(2);
    //set_gpio_mode(GPIOA_bank_bit0_27(22), GPIOA_bit_bit0_27(22), GPIO_OUTPUT_MODE);
    //set_gpio_val(GPIOA_bank_bit0_27(22), GPIOA_bit_bit0_27(22), 1);
    gpio_out(PAD_GPIOA_22, 1);
    val = battery_get_adc_value() * (2 * 2500000 / 1023);

    printk(KERN_DEBUG "%s() Measured voltage %dmV\n", __FUNCTION__, val);
    return val;
}

/*
 *  Get Vhigh when BAT_SEL(GPIOA_22) is High.
 *  Get Vlow when BAT_SEL(GPIOA_22) is Low.
 *  I = Vdiff / 0.02R
 *  Vdiff = Vhigh - Vlow
 */
static inline int battery_measure_current(void)
{
    int val, Vh, Vl, Vdiff;

    //set_gpio_mode(GPIOA_bank_bit0_27(22), GPIOA_bit_bit0_27(22), GPIO_OUTPUT_MODE);
    //set_gpio_val(GPIOA_bank_bit0_27(22), GPIOA_bit_bit0_27(22), 1);
    gpio_out(PAD_GPIOA_22, 1);
    msleep(2);

    Vl = get_adc_sample(5) * (2 * 2500000 / 1023);
    printk(KERN_DEBUG "%s() Vl is %dmV\n", __FUNCTION__, Vl);

    //set_gpio_mode(GPIOA_bank_bit0_27(22), GPIOA_bit_bit0_27(22), GPIO_OUTPUT_MODE);
    //set_gpio_val(GPIOA_bank_bit0_27(22), GPIOA_bit_bit0_27(22), 0);
    gpio_out(PAD_GPIOA_22, 1);
    msleep(2);

    Vh = get_adc_sample(5) * (2 * 2500000 / 1023);
    printk(KERN_DEBUG "%s() Vh is %dmV\n", __FUNCTION__, Vh);

    Vdiff = Vh - Vl;
    val = Vdiff * 50;
    printk(KERN_DEBUG "%s() Current is %dmA\n", __FUNCTION__, val);
    return val;
}

static inline int battery_measure_capacity(void)
{
    int val, tmp;

    tmp = battery_measure_voltage();
    if ((tmp > 4200000) || (battery_get_charge_status() == 0x1))
    	val = 100;
    else
    	val = (tmp - 3600000) / (600000 / 100);

    printk(KERN_DEBUG "%s() Capacity %d%%\n", __FUNCTION__, val);
    return val;
}

static int battery_value_table[37] = {
    0,  //0
    730,//0
    737,//4
    742,//10
    750,//15
    752,//16
    753,//18
    754,//20
    755,//23
    756,//26
    757,//29
    758,//32
    760,//35
    761,//37
    762,//40
    763,//43
    766,//46
    768,//49
    770,//51
    772,//54
    775,//57
    778,//60
    781,//63
    786,//66
    788,//68
    791,//71
    795,//74
    798,//77
    800,//80
    806,//83
    810,//85
    812,//88
    817,//91
    823,//95
    828,//97
    832,//100
    835 //100
};

static int battery_charge_value_table[37] = {
    0,  //0
    770,//0
    776,//4
    781,//10
    788,//15
    790,//16
    791,//18
    792,//20
    793,//23
    794,//26
    795,//29
    796,//32
    797,//35
    798,//37
    799,//40
    800,//43
    803,//46
    806,//49
    808,//51
    810,//54
    812,//57
    815,//60
    818,//63
    822,//66
    827,//68
    830,//71
    833,//74
    836,//77
    838,//80
    843,//83
    847,//85
    849,//88
    852,//91
    854,//95
    855,//97
    856,//100
    857 //100
};

static int battery_level_table[37] = {
    0,
    0,
    4,
    10,
    15,
    16,
    18,
    20,
    23,
    26,
    29,
    32,
    35,
    37,
    40,
    43,
    46,
    49,
    51,
    54,
    57,
    60,
    63,
    66,
    68,
    71,
    74,
    77,
    80,
    83,
    85,
    88,
    91,
    95,
    97,
    100,
    100
};

static inline int battery_value2percent(int adc_vaule,
                                        int *adc_table,
                                        int *per_table,
                                        int table_size)
{
    int i;

    for (i = 0; i < (table_size - 1); i++) {
        if ((adc_vaule >= adc_table[i]) && (adc_vaule < adc_table[i+1])) {
            break;
        }
    }
    return per_table[i];
}

static int act8942_measure_capacity_charging(void)
{
    return battery_value2percent(battery_get_adc_value(), 
                                 battery_charge_value_table,
                                 battery_level_table,
                                 ARRAY_SIZE(battery_charge_value_table));
}

static int act8942_measure_capacity_battery(void)
{
    return battery_value2percent(battery_get_adc_value(),
                                 battery_value_table,
                                 battery_level_table,
                                 ARRAY_SIZE(battery_value_table));
}

static struct act8942_operations act8942_pdata = {
    .is_ac_online = act8xxx_is_ac_online,
    //.is_usb_online = is_usb_online,
    .get_charge_status = battery_get_charge_status,
    .set_charge_current = battery_set_charge_current,
    .measure_voltage = battery_measure_voltage,
    .measure_current = battery_measure_current,
    .measure_capacity_charging = act8942_measure_capacity_charging,
    .measure_capacity_battery = act8942_measure_capacity_battery,
    .update_period = 2000,  //2S
    .asn = 5,               //Average Sample Number
    .rvp = 1,               //Reverse Voltage Protection: 1:enable; 0:disable
};
#endif // CONFIG_PMU_ACT8942

static struct platform_device aml_pmu_device = {
    .name = ACT8xxx_DEVICE_NAME,
    .id = -1,
};

#ifdef CONFIG_BQ27x00_BATTERY
static struct bq27x00_battery_pdata bq27x00_pdata = {
    .is_ac_online = act8xxx_is_ac_online,
    .get_charge_status = battery_get_charge_status,
    .set_charge = battery_set_charge_current,
    .chip = 1,
};
#endif // CONFIG_BQ27x00_BATTERY

static struct i2c_board_info __initdata aml_i2c_bus_info_2[] = {
#ifdef CONFIG_BQ27x00_BATTERY
    {
        I2C_BOARD_INFO("bq27500", 0x55),
        .platform_data = (void *)&bq27x00_pdata,
    },
#endif
    {
        I2C_BOARD_INFO(ACT8xxx_I2C_NAME, ACT8xxx_ADDR),
#ifdef CONFIG_PMU_ACT8942
        .platform_data = (void *)&act8942_pdata,
#endif
    },
};

static struct platform_device __initdata * m3ref_power_devices[] = {
    &aml_pmu_device,
};

int __init f04_power_init(void)
{
    int err;

    pm_power_off = m3ref_power_off;
    i2c_register_board_info(2, aml_i2c_bus_info_2, ARRAY_SIZE(aml_i2c_bus_info_2));
    err = platform_add_devices(m3ref_power_devices, ARRAY_SIZE(m3ref_power_devices));
    return err;
}
