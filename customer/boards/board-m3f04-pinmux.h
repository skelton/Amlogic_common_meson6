/*
 * customer/boards/board-m3f04-pinmux.h
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

/*
Linux PIN_ITEMS.C
*/
#include <mach/pinmux.h>

//used by pinmux_item.c
//one device may have more than one pinmux relative regs.this is the max number.
#if 0
#define  MAX_PIN_ITEM_NUM     13
enum  device_pinitem_index{
        DEVICE_PIN_ITEM_UART,
        MAX_DEVICE_NUMBER,
 };


#define  uart_pins  { \
			.reg=PINMUX_REG(AO),\
			.clrmask=3<<16,\
			.setmask=3<<11,\
		}
		
		
	
static  pinmux_item_t   __initdata devices_pins[5][MAX_PIN_ITEM_NUM]=
{
    [0]={uart_pins,PINMUX_END_ITEM},
    [1]={{},{}},
    [2]={{},{}},
    //add other devices here. according to the uart item.
};
#endif
