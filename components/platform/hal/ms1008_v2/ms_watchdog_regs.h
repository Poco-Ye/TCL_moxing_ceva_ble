/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_watchdog_regs.h
 * @brief Header file of watchdog  module.
 * @author haijun.mai
 * @date   2022-01-04
 * @version 1.0
 * @Revision
 */
#ifndef MS_WATCHDOG_REGS_H_
#define MS_WATCHDOG_REGS_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	volatile uint32_t CR; //Control Register
	volatile uint32_t TORR; //Timeout Range Register
	volatile uint32_t CCVR; //Current Counter Value Register
	volatile uint32_t CRR; //Counter Restart Register
	volatile uint32_t STAT; //Interrupt Status Register
	volatile uint32_t EOI; //Interrupt Clear Register
	//  volatile uint32_t PROT_LEVEL;//WDT Protection level register
	// volatile uint32_t COMP_PARAM_5;//Component Parameters Register 5
	// volatile uint32_t COMP_PARAM_4;//Component Parameters Register 4
	//volatile uint32_t COMP_PARAM_3;//Component Parameters Register 3
	//volatile uint32_t COMP_PARAM_2;//Component Parameters Register 2
	//volatile uint32_t COMP_PARAM_1;//Component Parameters Register 1
	//volatile uint32_t COMP_VERSION;//Component Version Register
	//volatile uint32_t COMP_TYPE;//Component Type Register
} Watchdog_Type;

/*Control Register Bit Define*/
#define WATCHDOG_CR_WDT_EN_POS               							(0UL)
#define WATCHDOG_CR_WDT_EN_MASK      								(0x1UL << WATCHDOG_CR_WDT_EN_POS)
#define WATCHDOG_CR_WDT_EN										(WATCHDOG_CR_WDT_EN_MASK)

#define WATCHDOG_CR_RMOD_POS               							(1UL)
#define WATCHDOG_CR_RMOD_MASK      								(0x1UL << WATCHDOG_CR_RMOD_POS)
#define WATCHDOG_CR_RMOD										       (WATCHDOG_CR_RMOD_MASK)

#define WATCHDOG_CR_RPL_POS               						             (2UL)
#define WATCHDOG_CR_RPL_MASK      								      (0x7UL << WATCHDOG_CR_RPL_POS)
#define WATCHDOG_CR_RPL										      (WATCHDOG_CR_RPL_MASK)

#define WATCHDOG_CR_NO_NAME_POS               					      (5UL)
#define WATCHDOG_CR_NO_NAME_MASK      						      (0x1UL << WATCHDOG_CR_NO_NAME_POS)
#define WATCHDOG_CR_NO_NAME									      (WATCHDOG_CR_NO_NAME_MASK)

/*Timeout Range Register Bit Define*/
#define WATCHDOG_TORR_TOP_POS               					             (0UL)
#define WATCHDOG_TORR_TOP_MASK      						             (0xFUL << WATCHDOG_TORR_TOP_POS)
#define WATCHDOG_TORR_TOP								                    (WATCHDOG_TORR_TOP_MASK)

#define WATCHDOG_TORR_TOP_INIT_POS               					      (4UL)
#define WATCHDOG_TORR_TOP_INIT_MASK      						      (0xFUL << WATCHDOG_TORR_TOP_INIT_POS)
#define WATCHDOG_TORR_TOP_INIT								      (WATCHDOG_TORR_TOP_INIT_MASK)

/* Current Counter Value Register Bit Define*/
#define WATCHDOG_CR_WDT_CCVR_POS               					      (0UL)
#define WATCHDOG_CR_WDT_CCVR_MASK      						      (0xFFFFFFFFUL << WATCHDOG_CR_WDT_CCVR_POS)
#define WATCHDOG_CR_WDT_CCVR									      (WATCHDOG_CR_WDT_CCVR_MASK)

/*Counter Restart Register Bit Define*/
#define WATCHDOG_CRR_WDT_CRR_POS               				             (0UL)
#define WATCHDOG_CRR_WDT_CRR_MASK     						      (0xFFUL << WATCHDOG_CRR_WDT_CRR_POS)
#define WATCHDOG_CRR_WDT_CRR									      (WATCHDOG_CRR_WDT_CRR_MASK)

/*Interrupt Status Register Bit Define*/
#define WATCHDOG_STAT_WDT_STAT_POS               					      (0UL)
#define WATCHDOG_STAT_WDT_STAT_MASK      						      (0x1UL << WATCHDOG_STAT_WDT_STAT_POS)
#define WATCHDOG_STAT_WDT_STAT								      (WATCHDOG_STAT_WDT_STAT_MASK)

/*Interrupt Clear Register  Bit Define*/
#define WATCHDOG_EOI_WDT_EOI_POS               					      (0UL)
#define WATCHDOG_EOI_WDT_EOI_MASK      						      (0x1UL << WATCHDOG_EOI_WDT_EOI_POS)
#define WATCHDOG_EOI_WDT_EOI								             (WATCHDOG_EOI_WDT_EOI_MASK)

#ifdef __cplusplus
}
#endif

#endif
