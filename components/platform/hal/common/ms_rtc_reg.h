/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_rtc_reg.h
 * @brief
 * @author che.jiang
 * @date 2022-3-10
 * @version 1.0
 * @Revision
 */
#ifndef BLE_SOC_MS1008_RTC_REG_H_
#define BLE_SOC_MS1008_RTC_REG_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*Define  RTC counter width, value:8,...,32, default value:32*/
#define  RTC_CNT_WIDTH                                      (32UL)

/*Define  RTC prescaler counter width, value:2,...,32, default value:16*/
#define  RTC_PRESCLR_WIDTH                                  (16UL)

/** @addtogroup MS_REGISTER
 * @{
 */

/** @defgroup RTC Registers definition
 * @{
 */
typedef struct
{
    volatile uint32_t RTC_CCVR;
    volatile uint32_t RTC_CMR;
    volatile uint32_t RTC_CLR;
    volatile uint32_t RTC_CCR;
    volatile uint32_t RTC_STAT;
    volatile uint32_t RTC_RSTAT;
    volatile uint32_t RTC_EOI;
    volatile uint32_t RTC_COMP_VERSION;
    volatile uint32_t RTC_CPSR;
    volatile uint32_t RTC_CPCVR;
} RTC_Type;

/**
 * @}
 */

/*RTC Current counter value register*/
#define RTC_CURRENT_COUNTER_VALUE_POS						(0UL)
#define RTC_CURRENT_COUNTER_VALUE_MASK                      (0xFFFFFFFFUL)
#define RTC_CURRENT_COUNTER_VALUE                           RTC_CURRENT_COUNTER_VALUE_MASK

/*RTC Counter match register*/
#define RTC_COUNTER_MATCH_POS						        (0UL)

/*RTC Counter control register*/
#define RTC_IEN_POS                                         (0UL)
#define RTC_IEN_MASK                                        (0x1UL << RTC_IEN_POS)
#define RTC_IEN                                             RTC_IEN_MASK
#define RTC_MASK_POS                                        (1UL)
#define RTC_MASK_MASK                                       (0x1UL << RTC_MASK_POS)
#define RTC_MASK                                            RTC_MASK_MASK
#define RTC_EN_POS                                          (2UL)
#define RTC_EN_MASK                                         (0x1UL << RTC_EN_POS)
#define RTC_EN                                              RTC_EN_MASK
#define RTC_WEN_POS                                         (3UL)
#define RTC_WEN_MASK                                        (0x1UL << RTC_WEN_POS)
#define RTC_WEN                                             RTC_WEN_MASK
#define RTC_PSCLR_EN_POS                                    (4UL)
#define RTC_PSCLR_EN_MASK                                   (0x1UL << RTC_PSCLR_EN_POS)
#define RTC_PSCLR_EN                                        RTC_PSCLR_EN_MASK
#define RTC_PROT_LEVEL_POS                                  (5UL)
#define RTC_PROT_LEVEL_MASK                                 (0x7UL << RTC_PROT_LEVEL_POS)
#define RTC_PROT_LEVEL_EN                                   RTC_PROT_LEVEL_MASK
#define RTC_RSVD_CCR_POS                                    (8UL)
#define RTC_RSVD_CCR_MASK                                   (0xFFFFFFUL << RTC_RSVD_CCR_POS)
#define RTC_RSVD_CCR_EN                                     RTC_RSVD_CCR_MASK

/*RTC Interrupt status register*/
#define RTC_STAT_POS                                        (0UL)
#define RTC_STAT_MASK                                       (0x1UL << RTC_STAT_POS)
#define RTC_STAT                                            RTC_STAT_MASK
#define RTC_RSVD_RTC_STAT_POS                               (1UL)
#define RTC_RSVD_RTC_STAT_MASK                              (0x7FFFFFFFUL << RTC_RSVD_RTC_STAT_POS)
#define RTC_RSVD_RTC_STAT                                   RTC_RSVD_RTC_STAT_MASK

/*RTC Interrupt raw status register*/
#define RTC_RSTAT_POS                                       (0UL)
#define RTC_RSTAT_MASK                                      (0x1UL << RTC_RSTAT_POS)
#define RTC_RSTAT                                           RTC_RSTAT_MASK
#define RTC_RSVD_RTC_RSTAT_POS                              (1UL)
#define RTC_RSVD_RTC_RSTAT_MASK                             (0x7FFFFFFFUL << RTC_RSVD_RTC_RSTAT_POS)
#define RTC_RSVD_RTC_RSTAT                                  RTC_RSVD_RTC_RSTAT_MASK

/*RTC End of Interrupt register*/
#define RTC_RTC_EOI_POS                                     (0UL)
#define RTC_RTC_EOI_MASK                                    (0x1UL << RTC_RTC_EOI_POS)
#define RTC_RTC_EOI                                         RTC_RTC_EOI_MASK
#define RTC_RSVD_RTC_EOI_POS                                (1UL)
#define RTC_RSVD_RTC_EOI_MASK                               (0x7FFFFFFFUL << RTC_RSVD_RTC_EOI_POS)
#define RTC_RSVD_RTC_EOI                                    RTC_RSVD_RTC_EOI_MASK

/*RTC Counter prescaler register*/
#define RTC_COUNTER_PRESCALER_VALUE_POS                     (0UL)
#define RTC_COUNTER_PRESCALER_VALUE_MASK                    (0xFFFFUL << RTC_COUNTER_PRESCALER_VALUE_POS)
#define RTC_COUNTER_PRESCALER_VALUE                         RTC_COUNTER_PRESCALER_VALUE_MASK
#define RTC_RSVD_CPSR_POS                                   (16UL)
#define RTC_RSVD_CPSR_MASK                                  (0xFFFFUL << RTC_RSVD_CPSR_POS)
#define RTC_RSVD_CPSR                                       RTC_RSVD_CPSR_MASK

/*RTC Current prescaler counter value register*/
#define RTC_CURRENT_PRESCALER_COUNTER_VALUE_POS             (0UL)
#define RTC_CURRENT_PRESCALER_COUNTER_VALUE_MASK            (0xFFFFUL << RTC_CURRENT_PRESCALER_COUNTER_VALUE_POS)
#define RTC_CURRENT_PRESCALER_COUNTER_VALUE                 RTC_CURRENT_PRESCALER_COUNTER_VALUE_MASK
#define RTC_RSVD_CPCVR_POS                                  (16UL)
#define RTC_RSVD_CPCVR_MASK                                 (0xFFFFUL << RTC_RSVD_CPCVR_POS)
#define RTC_RSVD_CPCVR                                      RTC_RSVD_CPCVR_MASK

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* BLE_SOC_MS1008_RTC_REG_H_ */
