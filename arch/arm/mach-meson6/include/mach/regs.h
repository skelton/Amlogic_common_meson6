/*
 *
 *
 *  Copyright (C) 2010 AMLOGIC, INC.
 *
 * License terms: GNU General Public License (GPL) version 2
 * Basic register address definitions in physical memory and
 * some block defintions for core devices like the timer.
 * copy from linux kernel
 */

/*
 * Temp solution file for M6 define
*/
#ifndef __MACH_MESSON_FIRM_REGS_H
#define __MACH_MESSON_FIRM_REGS_H

#define IO_CBUS_BASE2  0xc1100000

#define NAND_CMD  ((0xc1108600-IO_CBUS_BASE2)>>2)
#define NAND_CFG  ((0xc1108604-IO_CBUS_BASE2)>>2)
#define NAND_DADR ((0xc1108608-IO_CBUS_BASE2)>>2)
#define NAND_IADR ((0xc110860c-IO_CBUS_BASE2)>>2)
#define NAND_BUF  ((0xc1108610-IO_CBUS_BASE2)>>2)
#define NAND_INFO ((0xc1108614-IO_CBUS_BASE2)>>2)
#define NAND_DC   ((0xc1108618-IO_CBUS_BASE2)>>2)
#define NAND_ADR  ((0xc110861c-IO_CBUS_BASE2)>>2)
#define NAND_DL   ((0xc1108620-IO_CBUS_BASE2)>>2)
#define NAND_DH   ((0xc1108624-IO_CBUS_BASE2)>>2)
#define NAND_CADR ((0xc1108628-IO_CBUS_BASE2)>>2)
#define NAND_SADR ((0xc110862c-IO_CBUS_BASE2)>>2)

#define P_NAND_CMD                                CBUS_REG_ADDR(NAND_CMD)
#define P_NAND_CFG                                CBUS_REG_ADDR(NAND_CFG)
#define P_NAND_DADR                               CBUS_REG_ADDR(NAND_DADR)
#define P_NAND_IADR                               CBUS_REG_ADDR(NAND_IADR)
#define P_NAND_BUF                                CBUS_REG_ADDR(NAND_BUF)
#define P_NAND_INFO                               CBUS_REG_ADDR(NAND_INFO)
#define P_NAND_DC                                 CBUS_REG_ADDR(NAND_DC)
#define P_NAND_ADR                                CBUS_REG_ADDR(NAND_ADR)
#define P_NAND_DL                                 CBUS_REG_ADDR(NAND_DL)
#define P_NAND_DH                                 CBUS_REG_ADDR(NAND_DH)
#define P_NAND_CADR                               CBUS_REG_ADDR(NAND_CADR)
#define P_NAND_SADR                               CBUS_REG_ADDR(NAND_SADR)


#endif
