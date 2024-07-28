/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_ir0_regs.h
 * @brief Header file of IR0  module.
 * @author haijun.mai
 * @date   2022-04-25
 * @version 1.0
 * @Revision
 */
#ifndef MS_IR0_REGS_H_
#define MS_IR0_REGS_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    volatile uint32_t IR0_CTL; /*IR Control Register   0x00*/
    volatile uint32_t IR0_TX_CTL;/*IR TX Control Register 0x04*/
    volatile uint32_t IR0_TX_CARRY_CFG;/*IR TX Carry Configuration Register 0x08*/
    volatile uint32_t IR0_TX_PREAMBLE_CFG;/*IR TX PREAMBLE Config Register 0x0c*/
    volatile uint32_t IR0_TX_DATA0_CFG;/*IR TX DATA0 CFG Register 0x10*/
    volatile uint32_t IR0_TX_DATA1_CFG;/*IR TX DATA1  CFG  Register 0x14*/
    volatile uint32_t IR0_TX_END_CFG;/*IR TX END CFG Register 0x18*/
    volatile uint32_t IR0_TX_REPEAT_CFG;/*IR_TX_REPEAT_CFG Register 0x1c*/
    volatile uint32_t IR0_TX_DATA;/*IR TX DATA  Register 0x20*/
    volatile uint32_t IR0_TX_STATUS;/*IR TX STATUS Register 0x24*/
} Ir0_Type;

/*IR Control Register bit difine  0x00*/
#define IR0_CTL_TX_EN_POS               		               	        	       (0UL)
#define IR0_CTL_TX_EN_MASK      								       (0x1UL << IR0_CTL_TX_EN_POS)
#define IR0_CTL_TX_EN										       (IR0_CTL_TX_EN_MASK)

/*IR TX Control Register bit difine 0x04*/
#define IR0_TX_CTL_TX_ABORT_EN_POS               		                     (0UL)
#define IR0_TX_CTL_TX_ABORT_EN_MASK      						(0x1UL << IR0_TX_CTL_TX_ABORT_EN_POS)
#define IR0_TX_CTL_TX_ABORT_EN								(IR0_TX_CTL_TX_ABORT_EN_MASK)

#define IR0_TX_CTL_TX_PREAMBLE_EN_POS               		              (1UL)
#define IR0_TX_CTL_TX_PREAMBLE_EN_MASK      					(0x1UL << IR0_TX_CTL_TX_PREAMBLE_EN_POS)
#define IR0_TX_CTL_TX_PREAMBLE_EN								(IR0_TX_CTL_TX_PREAMBLE_EN_MASK)

#define IR0_TX_CTL_TX_DATA_EN_POS               		                     (2UL)
#define IR0_TX_CTL_TX_DATA_EN_MASK      					       (0x1UL << IR0_TX_CTL_TX_DATA_EN_POS)
#define IR0_TX_CTL_TX_DATA_EN							               (IR0_TX_CTL_TX_DATA_EN_MASK)

#define IR0_TX_CTL_TX_END_EN_POS               		                             (3UL)
#define IR0_TX_CTL_TX_END_EN_MASK      					               (0x1UL << IR0_TX_CTL_TX_END_EN_POS)
#define IR0_TX_CTL_TX_END_EN							               (IR0_TX_CTL_TX_END_EN_MASK)

#define IR0_TX_CTL_TX_REPEAT_EN_POS               		                     (4UL)
#define IR0_TX_CTL_TX_REPEAT_EN_MASK      					       (0x1UL << IR0_TX_CTL_TX_REPEAT_EN_POS)
#define IR0_TX_CTL_TX_REPEAT_EN							       (IR0_TX_CTL_TX_REPEAT_EN_MASK)

#define IR0_TX_CTL_TX_PROTOCOL_POS               		                      (5UL)
#define IR0_TX_CTL_TX_PROTOCOL_MASK      					        (0x3UL << IR0_TX_CTL_TX_PROTOCOL_POS)
#define IR0_TX_CTL_TX_PROTOCOL							               (IR0_TX_CTL_TX_PROTOCOL_MASK)

#define IR0_TX_CTL_TX_DATA_NUM_POS               		                      (10UL)
#define IR0_TX_CTL_TX_DATA_NUM_MASK      					        (0x1FUL << IR0_TX_CTL_TX_DATA_NUM_POS)
#define IR0_TX_CTL_TX_DATA_NUM						               (IR0_TX_CTL_TX_DATA_NUM_MASK)

#define IR0_TX_CTL_TX_INT_EN_POS               		                             (15UL)
#define IR0_TX_CTL_TX_INT_EN_MASK      					               (0x1UL << IR0_TX_CTL_TX_INT_EN_POS)
#define IR0_TX_CTL_TX_INT_EN							               (IR0_TX_CTL_TX_INT_EN_MASK)

/*IR TX CARRY CFGR  egister bit define 0x08*/
#define IR0_TX_CARRY_CFG_LOW_CYCLE_POS               		             (0UL)
#define IR0_TX_CARRY_CFG_LOW_CYCLE_MASK      			             (0xFFFFUL << IR0_TX_CARRY_CFG_LOW_CYCLE_POS)
#define IR0_TX_CARRY_CFG_LOW_CYCLE						      (IR0_TX_CARRY_CFG_LOW_CYCLE_MASK)

#define IR0_TX_CARRY_CFG_HIGH_CYCLE_POS               		             (16UL)
#define IR0_TX_CARRY_CFG_HIGH_CYCLE_MASK      			             (0xFFFFUL << IR0_TX_CARRY_CFG_HIGH_CYCLE_POS)
#define IR0_TX_CARRY_CFG_HIGH_CYCLE						      (IR0_TX_CARRY_CFG_HIGH_CYCLE_MASK)



/*IR TX PREAMBLE CFG Register bit define 0x0c*/
#define IR0_TX_PREAMBLE_CFG_CARRY_CNT_POS               		      (0UL)
#define IR0_TX_PREAMBLE_CFG_CARRY_CNT_MASK      			      (0xFFFUL << IR0_TX_PREAMBLE_CFG_CARRY_CNT_POS)
#define IR0_TX_PREAMBLE_CFG_CARRY_CNT					      (IR0_TX_PREAMBLE_CFG_CARRY_CNT_MASK)

#define IR0_TX_PREAMBLE_CFG_IDLE_CNT_POS               		      (12UL)
#define IR0_TX_PREAMBLE_CFG_IDLE_CNT_MASK      			      (0xFFFFFUL << IR0_TX_PREAMBLE_CFG_IDLE_CNT_POS)
#define IR0_TX_PREAMBLE_CFG_IDLE_CNT					             (IR0_TX_PREAMBLE_CFG_IDLE_CNT_MASK)

/*IR TX Level Configuration Register bit define 0x10*/
#define IR0_TX_DATA0_CFG_CARRY_CNT_POS               		              (0UL)
#define IR0_TX_DATA0_CFG_CARRY_CNT_MASK      					(0xFFFUL << IR0_TX_DATA0_CFG_CARRY_CNT_POS)
#define IR0_TX_DATA0_CFG_CARRY_CNT							(IR0_TX_DATA0_CFG_CARRY_CNT_MASK)

#define IR0_TX_DATA0_CFG_IDLE_CNT_POS               		              (12UL)
#define IR0_TX_DATA0_CFG_IDLE_CNT_MASK      					(0xFFFFFUL << IR0_TX_DATA0_CFG_IDLE_CNT_POS)
#define IR0_TX_DATA0_CFG_IDLE_CNT							       (IR0_TX_DATA0_CFG_IDLE_CNT_MASK)


/*IR TX DATA1 CFG bit define 0x14*/
#define IR0_TX_DATA1_CFG_CARRY_CNT_POS               		              (0UL)
#define IR0_TX_DATA1_CFG_CARRY_CNT_MASK      					(0xFFFUL << IR0_TX_DATA1_CFG_CARRY_CNT_POS)
#define IR0_TX_DATA1_CFG_CARRY_CNT							(IR0_TX_DATA1_CFG_CARRY_CNT_MASK)

#define IR0_TX_DATA1_CFG_IDLE_CNT_POS               		              (12UL)
#define IR0_TX_DATA1_CFG_IDLE_CNT_MASK      					(0xFFFFFUL << IR0_TX_DATA1_CFG_IDLE_CNT_POS)
#define IR0_TX_DATA1_CFG_IDLE_CNT							       (IR0_TX_DATA1_CFG_IDLE_CNT_MASK)

/*IR TX END CFG bit difine 0x18*/
#define IR0_TX_END_CFG_CARRY_CNT_POS               		              (0UL)
#define IR0_TX_END_CFG_CARRY_CNT_MASK      					(0xFFFUL << IR0_TX_END_CFG_CARRY_CNT_POS)
#define IR0_TX_END_CFG_CARRY_CNT								(IR0_TX_END_CFG_CARRY_CNT_MASK)

#define IR0_TX_END_CFG_IDLE_CNT_POS               		                     (12UL)
#define IR0_TX_END_CFG_IDLE_CNT_MASK      					       (0xFFFFFUL << IR0_TX_END_CFG_IDLE_CNT_POS)
#define IR0_TX_END_CFG_IDLE_CNT								(IR0_TX_END_CFG_IDLE_CNT_MASK)

/*IR Protocol mode Start bit Low Register bit define 0x1c*/
#define IR0_TX_REPEAT_CFG_TX_REPEAT_CYCLE_POS               		(0UL)
#define IR0_TX_REPEAT_CFG_TX_REPEAT_CYCLE_MASK      			(0xFFFFFFFFUL << IR0_TX_REPEAT_CFG_TX_REPEAT_CYCLE_POS)
#define IR0_TX_REPEAT_CFG_TX_REPEAT_CYCLE					(IR0_TX_REPEAT_CFG_TX_REPEAT_CYCLE_MASK)


/*IR Protocol mode Data and End bit High Register bit define 0x20*/
#define IR0_TX_DATA_TX_DATA_POS               		                            (0UL)
#define IR0_TX_DATA_TX_DATA_MASK      					              (0xFFFFFFFFUL << IR0_TX_DATA_TX_DATA_POS)
#define IR0_TX_DATA_TX_DATA								       (IR0_TX_DATA_TX_DATA_MASK)

/*IR Protocol mode Data bit Low Register 0x24*/
#define IR0_TX_STATUS_TX_DONE_POS               		                     (0UL)
#define IR0_TX_STATUS_TX_DONE_MASK      		               		(0x1UL << IR0_TX_STATUS_TX_DONE_POS)
#define IR0_TX_STATUS_TX_DONE				                     		(IR0_TX_STATUS_TX_DONE_MASK)

#define IR0_TX_STATUS_TX_BUSY_POS               		                     (1UL)
#define IR0_TX_STATUS_TX_BUSY_MASK      		               		(0x1UL << IR0_TX_STATUS_TX_BUSY_POS)
#define IR0_TX_STATUS_TX_BUSY				                     		(IR0_TX_STATUS_TX_BUSY_MASK)

#define IR0_TX_STATUS_TX_FRAME_CNT_POS               		              (2UL)
#define IR0_TX_STATUS_TX_FRAME_CNT_MASK      		               	(0x7FUL << IR0_TX_STATUS_TX_FRAME_CNT_POS)
#define IR0_TX_STATUS_TX_FRAME_CNT				                     (IR0_TX_STATUS_TX_FRAME_CNT_MASK)





#ifdef __cplusplus
}
#endif

#endif
