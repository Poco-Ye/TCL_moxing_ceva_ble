/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_ir_regs.h
 * @brief Header file of IR  module.
 * @author haijun.mai
 * @date   2022-03-28
 * @version 1.0
 * @Revision
 */
#ifndef MS_IR1_REGS_H_
#define MS_IR1_REGS_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    volatile uint32_t IR1_TX_CTL; /*IR TX Control Register   0x00*/
    volatile uint32_t IR1_TX_LENGTH;/*IR TX Length Register 0x04*/
    volatile uint32_t IR1_FIFO_CFG;/*IR FIFO Configuration Register 0x08*/
    volatile uint32_t IR1_TX_CARRY_CFG;/*IR TX Carry Configuration Register 0x0c*/
    volatile uint32_t IR1_TX_LEVEL_CFG;/*IR TX Level Configuration Register 0x10*/
    volatile uint32_t IR1_PROTO_TX_DATA;/*IR Protocol mode TX Data 0x14*/
    volatile uint32_t IR1_PROTO_START_H;/*IR Protocol mode Start bit High Register 0x18*/
    volatile uint32_t IR1_PROTO_START_L;/*IR Protocol mode Start bit Low Register 0x1c*/
    volatile uint32_t IR1_PROTO_DATA_END_H;/*IR Protocol mode Data and End bit High Register 0x20*/
    volatile uint32_t IR1_PROTO_DATA_L;/*IR Protocol mode Data bit Low Register 0x24*/
    volatile uint32_t IR1_PROTO_END_L;/*IR Protocol mode End bit Low Register 0x28*/
    volatile uint32_t IR1_PROTO_REP_END_L;/*IR Protocol mode Repeat frame End bit Low Register 0x2c*/
    volatile uint32_t IR1_INT_MASK;/*IR Interrupt Mask Register 0x30*/
    volatile uint32_t	IR1_INT_STATUS;/*IR Interrupt Status Register 0x34*/
    volatile uint32_t	IR1_STATUS;/*IR Status Register 0x38*/
    volatile uint32_t	IR1_FLUSH;/*IR Flush Register 0x3c*/
} Ir1_Type;

/*IR TX Control Register bit difine  0x00*/
#define IR1_TX_CTL_TX_EN_POS               		               			(0UL)
#define IR1_TX_CTL_TX_EN_MASK      								(0x1UL << IR1_TX_CTL_TX_EN_POS)
#define IR1_TX_CTL_TX_EN										(IR1_TX_CTL_TX_EN_MASK)

#define IR1_TX_CTL_DMA_EN_POS               		               			(1UL)
#define IR1_TX_CTL_DMA_EN_MASK      							(0x1UL << IR1_TX_CTL_DMA_EN_POS)
#define IR1_TX_CTL_DMA_EN										 (IR1_TX_CTL_DMA_EN_MASK)

#define IR1_TX_CTL_INT_EN_POS               		               			(2UL)
#define IR1_TX_CTL_INT_EN_MASK     								(0x1UL << IR1_TX_CTL_INT_EN_POS)
#define IR1_TX_CTL_INT_EN										(IR1_TX_CTL_INT_EN_MASK)

#define IR1_TX_CTL_PROTO_MODE_POS               		               	(4UL)
#define IR1_TX_CTL_PROTO_MODE_MASK      						(0x3UL << IR1_TX_CTL_PROTO_MODE_POS)
#define IR1_TX_CTL_PROTO_MODE								       (IR1_TX_CTL_PROTO_MODE_MASK)

#define IR1_TX_CTL_RPOTO_REPEAT_EN_POS               		              (6UL)
#define IR1_TX_CTL_PROTO_REPEAT_EN_MASK      				       (0x1UL << IR1_TX_CTL_RPOTO_REPEAT_EN_POS)
#define IR1_TX_CTL_PROTO_REPEAT_EN						       (IR1_TX_CTL_PROTO_REPEAT_EN_MASK)

#define IR1_TX_CTL_FIFO_DATA_MODE_POS               		              (7UL)
#define IR1_TX_CTL_FIFO_DATA_MODE_MASK      				       (0x1UL << IR1_TX_CTL_FIFO_DATA_MODE_POS)
#define IR1_TX_CTL_FIFO_DATA_MODE								(IR1_TX_CTL_FIFO_DATA_MODE_MASK)

#define IR1_TX_CTL_FIFO_CARRY_COMP_MODE_POS               		(8UL)
#define IR1_TX_CTL_FIFO_CARRY_COMP_MODE_MASK      			(0x3UL << IR1_TX_CTL_FIFO_CARRY_COMP_MODE_POS)
#define IR1_TX_CTL_FIFO_CARRY_COMP_MODE						(IR1_TX_CTL_FIFO_CARRY_COMP_MODE_MASK)



/*IR TX Length Register bit define 0x04*/
#define IR1_TX_LENGTH_CFG_POS               		               			(0UL)
#define IR1_TX_LENGTH_CFG_MASK      							(0xFFFUL << IR1_TX_LENGTH_CFG_POS)
#define IR1_TX_LENGTH_CFG										(IR1_TX_LENGTH_CFG_MASK)

/*IR FIFO Configuration Register bit define 0x08*/
#define IR1_FIFO_CFG_FIFO_WATERMASK_POS               		             (0UL)
#define IR1_FIFO_CFG_FIFO_WATERMASK_MASK      			      (0xFFUL << IR1_FIFO_CFG_FIFO_WATERMASK_POS)
#define IR1_FIFO_CFG_FIFO_WATERMASK						      (IR1_FIFO_CFG_FIFO_WATERMASK_MASK)

#define IR1_TX_CARRY_CFG_CARRY_HIGH_CYCLE_POS               		(0UL)
#define IR1_TX_CARRY_CFG_CARRY_HIGH_CYCLE_MASK      			(0xFFFFUL << IR1_TX_CARRY_CFG_CARRY_HIGH_CYCLE_POS)
#define IR1_TX_CARRY_CFG_CARRY_HIGH_CYCLE					(IR1_TX_CARRY_CFG_CARRY_HIGH_CYCLE_MASK)


/*IR TX Carry Configuration Register bit define 0x0c*/
#define IR1_TX_CARRY_CFG_CARRY_LOW_CYCLE_POS               		(16UL)
#define IR1_TX_CARRY_CFG_CARRY_LOW_CYCLE_MASK      			(0xFFFFUL << IR1_TX_CARRY_CFG_CARRY_LOW_CYCLE_POS)
#define IR1_TX_CARRY_CFG_CARRY_LOW_CYCLE					(IR1_TX_CARRY_CFG_CARRY_LOW_CYCLE_MASK)


/*IR TX Level Configuration Register bit define 0x10*/
#define IR1_TX_LEVEL_CFG_LEVEL_CYCLE_POS               		              (0UL)
#define IR1_TX_LEVEL_CFG_LEVEL_CYCLE_MASK      					(0x7FFFFFUL << IR1_TX_LEVEL_CFG_LEVEL_CYCLE_POS)
#define IR1_TX_LEVEL_CFG_LEVEL_CYCLE							(IR1_TX_LEVEL_CFG_LEVEL_CYCLE_MASK)

#define IR1_TX_LEVEL_CFG_LEVEL_POL_SEL_POS               		       (31UL)
#define IR1_TX_LEVEL_CFG_LEVEL_POL_SEL_MASK      			       (0x1UL << IR1_TX_LEVEL_CFG_LEVEL_POL_SEL_POS)
#define IR1_TX_LEVEL_CFG_LEVEL_POL_SEL						(IR1_TX_LEVEL_CFG_LEVEL_POL_SEL_MASK)

/*IR Protocol mode TX Data bit define 0x14*/
#define IR1_PROTO_TX_DATA_CFG_POS               		               	(0UL)
#define IR1_PROTO_TX_DATA_CFG_MASK      						(0xFFFFFFFFUL << IR1_PROTO_TX_DATA_CFG_POS)
#define IR1_PROTO_TX_DATA_CFG									(IR1_PROTO_TX_DATA_CFG_MASK)

/*IR Protocol mode Start bit High Register bit difine 0x18*/
#define IR1_PROTO_START_H_CFG_POS               		               	(0UL)
#define IR1_PROTO_START_H_CFG_MASK      						(0xFFFFFUL << IR1_PROTO_START_H_CFG_POS)
#define IR1_PROTO_START_H_CFG									(IR1_PROTO_START_H_CFG_MASK)

/*IR Protocol mode Start bit Low Register bit define 0x1c*/
#define IR1_PROTO_START_L_CFG_POS               		               	(0UL)
#define IR1_PROTO_START_L_CFG_MASK      						(0xFFFFFUL << IR1_PROTO_START_L_CFG_POS)
#define IR1_PROTO_START_L_CFG									(IR1_PROTO_START_L_CFG_MASK)

/*IR Protocol mode Data and End bit High Register bit define 0x20*/
#define IR1_PROTO_DATA_END_H_CFG_POS               		              (0UL)
#define IR1_PROTO_DATA_END_H_CFG_MASK      					(0xFFFFUL << IR1_PROTO_DATA_END_H_CFG_POS)
#define IR1_PROTO_DATA_END_H_CFG								(IR1_PROTO_DATA_END_H_CFG_MASK)

/*IR Protocol mode Data bit Low Register 0x24*/
#define IR1_PROTO_DATA_L_PROTO_DATA0_L_POS               		       (0UL)
#define IR1_PROTO_DATA_L_PROTO_DATA0_L_MASK      				(0xFFFFUL << IR1_PROTO_DATA_L_PROTO_DATA0_L_POS)
#define IR1_PROTO_DATA_L_PROTO_DATA0_L						(IR1_PROTO_DATA_L_PROTO_DATA0_L_MASK)

#define IR1_PROTO_DATA_L_PROTO_DATA1_L_POS               		       (16UL)
#define IR1_PROTO_DATA_L_PROTO_DATA1_L_MASK      				(0xFFFFUL << IR1_PROTO_DATA_L_PROTO_DATA1_L_POS)
#define IR1_PROTO_DATA_L_PROTO_DATA1_L						(IR1_PROTO_DATA_L_PROTO_DATA1_L_MASK)

/*IR Protocol mode End bit Low Register bit define 0x28*/
#define IR1_PROTO_END_L_CFG_POS               		               		(0UL)
#define IR1_PROTO_END_L_CFG_MASK      							(0x7FFFFFUL << IR1_PROTO_END_L_CFG_POS)
#define IR1_PROTO_END_L_CFG									(IR1_PROTO_END_L_CFG_MASK)

/*IR Protocol mode Repeat frame End bit Low Register bit define 0x2c*/
#define IR1_PROTO_REP_END_L_CFG_POS               		               	(0UL)
#define IR1_PROTO_REP_END_L_CFG_MASK      						(0x7FFFFFUL << IR1_PROTO_REP_END_L_CFG_POS)
#define IR1_PROTO_REP_END_L_CFG								(IR1_PROTO_REP_END_L_CFG_MASK)


/*IR Interrupt Mask Register 0x30 bit define*/
#define IR1_INT_MASK_TX_DONE_POS               		               		(0UL)
#define IR1_INT_MASK_TX_DONE_MASK      							(0x1UL << IR1_INT_MASK_TX_DONE_POS)
#define IR1_INT_MASK_TX_DONE									(IR1_INT_MASK_TX_DONE_MASK)

#define IR1_INT_MASK_FIFO_WATERMASK_POS               		       (1UL)
#define IR1_INT_MASK_FIFO_WATERMASK_MASK      				(0x1UL << IR1_INT_MASK_FIFO_WATERMASK_POS)
#define IR1_INT_MASK_FIFO_WATERMASK							(IR1_INT_MASK_FIFO_WATERMASK_MASK)

#define IR1_INT_MASK_TX_FIFO_EMPTY_POS               		              (2UL)
#define IR1_INT_MASK_TX_FIFO_EMPTY_MASK      					(0x1UL << IR1_INT_MASK_TX_FIFO_EMPTY_POS)
#define IR1_INT_MASK_TX_FIFO_EMPTY							(IR1_INT_MASK_TX_FIFO_EMPTY_MASK)

#define IR1_INT_MASK_TX_FIFO_FULL_POS               		               (3UL)
#define IR1_INT_MASK_TX_FIFO_FULL_MASK      					(0x1UL << IR1_INT_MASK_TX_FIFO_FULL_POS)
#define IR1_INT_MASK_TX_FIFO_FULL								(IR1_INT_MASK_TX_FIFO_FULL_MASK)

#define IR1_INT_MASK_TX_FIFO_OVERFLOW_POS               		       (4UL)
#define IR1_INT_MASK_TX_FIFO_OVERFLOW_MASK      				(0x1UL << IR1_INT_MASK_TX_FIFO_OVERFLOW_POS)
#define IR1_INT_MASK_TX_FIFO_OVERFLOW						(IR1_INT_MASK_TX_FIFO_OVERFLOW_MASK)

#define IR1_INT_MASK_TX_RD_FAIL_POS               		                     (5UL)
#define IR1_INT_MASK_TX_RD_FAIL_MASK      				              (0x1UL << IR1_INT_MASK_TX_RD_FAIL_POS)
#define IR1_INT_MASK_TX_RD_FAIL							       (IR1_INT_MASK_TX_RD_FAIL_POS)

#define  IR1_INT_MASK_ALL_POS               		                                   (0UL)
#define  IR1_INT_MASK_ALL_MASK      				                            (0x3FUL << IR1_INT_MASK_ALL_POS)
#define  IR1_INT_MASK_ALL						                            (IR1_INT_MASK_ALL_MASK)


/*IR Interrupt Status Register bit define 0x34*/
#define IR1_INT_STATUS_TX_DONE_INT_POS               		              (0UL)
#define IR1_INT_STATUS_TX_DONE_INT_MASK      					(0x1UL << IR1_INT_STATUS_TX_DONE_INT_POS)
#define IR1_INT_STATUS_TX_DONE_INT							(IR1_INT_STATUS_TX_DONE_INT_MASK)

#define IR1_INT_STATUS_FIFO_WATERMASK_INT_POS               		(1UL)
#define IR1_INT_STATUS_FIFO_WATERMASK_INT_MASK      			(0x1UL << IR1_INT_STATUS_FIFO_WATERMASK_INT_POS)
#define IR1_INT_STATUS_FIFO_WATERMASK_INT					(IR1_INT_STATUS_FIFO_WATERMASK_INT_MASK)

#define IR1_INT_STATUS_TX_FIFO_EMPTY_INT_POS               	       (2UL)
#define IR1_INT_STATUS_TX_FIFO_EMPTY_INT_MASK      			(0x1UL << IR1_INT_STATUS_TX_FIFO_EMPTY_INT_POS)
#define IR1_INT_STATUS_TX_FIFO_EMPTY_INT						(IR1_INT_STATUS_TX_FIFO_EMPTY_INT_MASK)

#define IR1_INT_STATUS_TX_FIFO_FULL_INT_POS               		       (3UL)
#define IR1_INT_STATUS_TX_FIFO_FULL_INT_MASK      				(0x1UL << IR1_INT_STATUS_TX_FIFO_FULL_INT_POS)
#define IR1_INT_STATUS_TX_FIFO_FULL_INT						(IR1_INT_STATUS_TX_FIFO_FULL_INT_MASK)

#define IR1_INT_STATUS_TX_FIFO_OVERFLOW_INT_POS               	(4UL)
#define IR1_INT_STATUS_TX_FIFO_OVERFLOW_INT_MASK      		(0x1UL << IR1_INT_STATUS_TX_FIFO_OVERFLOW_INT_POS)
#define IR1_INT_STATUS_TX_FIFO_OVERFLOW_INT				       (IR1_INT_STATUS_TX_FIFO_OVERFLOW_INT_MASK)

#define IR1_INT_STATUS_TX_RD_FAIL_INT_POS               		       (5UL)
#define IR1_INT_STATUS_TX_RD_FAIL_INT_MASK      				(0x1UL << IR1_INT_STATUS_TX_RD_FAIL_INT_POS)
#define IR1_INT_STATUS_TX_RD_FAIL_INT							(IR1_INT_STATUS_TX_RD_FAIL_INT_MASK)

#define IR1_INT_STATUS_ALL_POS               		                             (0UL)
#define IR1_INT_STATUS_ALL_MASK      				                      (0x3FUL << IR1_INT_STATUS_ALL_POS)
#define IR1_INT_STATUS_ALL							                      (IR1_INT_STATUS_ALL_MASK)


/*IR Status Register bit define 0x38*/
#define IR1_STATUS_TX_DONE_STA_POS               		               	(0UL)
#define IR1_STATUS_TX_DONE_STA_MASK      						(0x1UL << IR1_STATUS_TX_DONE_STA_POS)
#define IR1_STATUS_TX_DONE_STA								(IR1_STATUS_TX_DONE_STA_MASK)

#define IR1_STATUS_FIFO_WATERMASK_STA_POS               		       (1UL)
#define IR1_STATUS_FIFO_WATERMASK_STA_MASK      				(0x1UL << IR1_STATUS_FIFO_WATERMASK_STA_POS)
#define IR1_STATUS_FIFO_WATERMASK_STA						(IR1_STATUS_FIFO_WATERMASK_STA_MASK)

#define IR1_STATUS_TX_FIFO_EMPTY_STA_POS               		       (2UL)
#define IR1_STATUS_TX_FIFO_EMPTY_STA_MASK      				(0x1UL << IR1_STATUS_TX_FIFO_EMPTY_STA_POS)
#define IR1_STATUS_TX_FIFO_EMPTY_STA						       (IR1_STATUS_TX_FIFO_EMPTY_STA_MASK)

#define IR1_STATUS_TX_FIFO_FULL_STA_POS               		              (3UL)
#define IR1_STATUS_TX_FIFO_FULL_STA_MASK      				       (0x1UL << IR1_STATUS_TX_FIFO_FULL_STA_POS)
#define IR1_STATUS_TX_FIFO_FULL_STA							(IR1_STATUS_TX_FIFO_FULL_STA_MASK)

#define IR1_STATUS_TX_FIFO_OVERFLOW_STA_POS               	      (4UL)
#define IR1_STATUS_TX_FIFO_OVERFLOW_STA_MASK      			(0x1UL << IR1_STATUS_TX_FIFO_OVERFLOW_STA_POS)
#define IR1_STATUS_TX_FIFO_OVERFLOW_STA					       (IR1_STATUS_TX_FIFO_OVERFLOW_STA_MASK)

#define IR1_STATUS_TX_RD_FAIL_STA_POS               		              (5UL)
#define IR1_STATUS_TX_RD_FAIL_STA_MASK      				       (0x1UL << IR1_STATUS_TX_RD_FAIL_STA_POS)
#define IR1_STATUS_TX_RD_FAIL_STA							       (IR1_STATUS_TX_RD_FAIL_STA_MASK)

#define IR1_STATUS_IR_BUSY_STA_POS               		                     (6UL)
#define IR1_STATUS_IR_BUSY_STA_MASK      				              (0x1UL << IR1_STATUS_IR_BUSY_STA_POS)
#define IR1_STATUS_IR_BUSY_STA							       (IR1_STATUS_IR_BUSY_STA_MASK)

#define IR1_STATUS_ALL_POS               		                                   (0UL)
#define IR1_STATUS_ALL_MASK      				                            (0x7FUL << IR1_STATUS_ALL_POS)
#define IR1_STATUS_ALL							                            (IR1_STATUS_ALL_MASK)


/*IR Flush Register 0x3c bit define*/
#define IR1_FLUSH_CFG_POS               		                                          (0UL)
#define IR1_FLUSH_CFG_MASK      				                                   (0x1UL << IR1_FLUSH_CFG_POS)
#define IR1_FLUSH_CFG							                            (IR1_FLUSH_CFG_MASK)













#ifdef __cplusplus
}
#endif

#endif

