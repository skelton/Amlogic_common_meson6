/*
 *  arch/arm/mach-meson3/include/mach/map.h
 *
 *  Copyright (C) 2010 AMLOGIC, INC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
*   this file used to map common VAR in plat-meson to mach-mesonX 
*
 */
#ifndef _MACH_MAP_H
#define _MACH_MAP_H

#define P_CPU_0_IRQ_IN0_INTR_MASK P_SYS_CPU_0_IRQ_IN0_INTR_MASK
#define P_CPU_0_IRQ_IN1_INTR_MASK P_SYS_CPU_0_IRQ_IN1_INTR_MASK
#define P_CPU_0_IRQ_IN2_INTR_MASK P_SYS_CPU_0_IRQ_IN2_INTR_MASK
#define P_CPU_0_IRQ_IN3_INTR_MASK P_SYS_CPU_0_IRQ_IN3_INTR_MASK

#define P_CPU_0_IRQ_IN0_INTR_STAT_CLR P_SYS_CPU_0_IRQ_IN0_INTR_STAT_CLR
#define P_CPU_0_IRQ_IN1_INTR_STAT_CLR P_SYS_CPU_0_IRQ_IN1_INTR_STAT_CLR
#define P_CPU_0_IRQ_IN2_INTR_STAT_CLR P_SYS_CPU_0_IRQ_IN2_INTR_STAT_CLR
#define P_CPU_0_IRQ_IN3_INTR_STAT_CLR P_SYS_CPU_0_IRQ_IN3_INTR_STAT_CLR

#define P_CPU_0_IRQ_IN0_INTR_FIRQ_SEL P_SYS_CPU_0_IRQ_IN0_INTR_FIRQ_SEL
#define P_CPU_0_IRQ_IN1_INTR_FIRQ_SEL P_SYS_CPU_0_IRQ_IN1_INTR_FIRQ_SEL
#define P_CPU_0_IRQ_IN2_INTR_FIRQ_SEL P_SYS_CPU_0_IRQ_IN2_INTR_FIRQ_SEL
#define P_CPU_0_IRQ_IN3_INTR_FIRQ_SEL P_SYS_CPU_0_IRQ_IN3_INTR_FIRQ_SEL

#define CPU_0_IRQ_IN0_INTR_MASK SYS_CPU_0_IRQ_IN0_INTR_MASK
#define CPU_0_IRQ_IN1_INTR_MASK SYS_CPU_0_IRQ_IN1_INTR_MASK
#define CPU_0_IRQ_IN2_INTR_MASK SYS_CPU_0_IRQ_IN2_INTR_MASK
#define CPU_0_IRQ_IN3_INTR_MASK SYS_CPU_0_IRQ_IN3_INTR_MASK

#define CPU_0_IRQ_IN0_INTR_STAT_CLR SYS_CPU_0_IRQ_IN0_INTR_STAT_CLR
#define CPU_0_IRQ_IN1_INTR_STAT_CLR SYS_CPU_0_IRQ_IN1_INTR_STAT_CLR
#define CPU_0_IRQ_IN2_INTR_STAT_CLR SYS_CPU_0_IRQ_IN2_INTR_STAT_CLR
#define CPU_0_IRQ_IN3_INTR_STAT_CLR SYS_CPU_0_IRQ_IN3_INTR_STAT_CLR

#define CPU_0_IRQ_IN0_INTR_FIRQ_SEL SYS_CPU_0_IRQ_IN0_INTR_FIRQ_SEL
#define CPU_0_IRQ_IN1_INTR_FIRQ_SEL SYS_CPU_0_IRQ_IN1_INTR_FIRQ_SEL
#define CPU_0_IRQ_IN2_INTR_FIRQ_SEL SYS_CPU_0_IRQ_IN2_INTR_FIRQ_SEL
#define CPU_0_IRQ_IN3_INTR_FIRQ_SEL SYS_CPU_0_IRQ_IN3_INTR_FIRQ_SEL

#define SRAM_PHY_ADDR   IO_AHB_BUS_PHY_BASE

#define MESON_I2C_1_START  P_I2C_M_0_CONTROL_REG
#define MESON_I2C_1_END      (P_I2C_M_0_RDATA_REG1-1)
#define MESON_I2C_2_START  P_I2C_M_1_CONTROL_REG
#define MESON_I2C_2_END      (P_I2C_M_1_RDATA_REG1-1)
#define MESON_I2C_3_START         P_I2C_S_CONTROL_REG
#define MESON_I2C_3_SLAVE_END             (P_I2C_S_CNTL1_REG-1)


#endif 
