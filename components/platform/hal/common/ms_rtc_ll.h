/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_qspi_ll.h
 * @brief Header file of rtc  module.
 * @author che.jiang
 * @date   2022-3-11
 * @version 1.0
 * @Revision:
 */

#ifndef MS_RTC_LL_H_
#define MS_RTC_LL_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>


/**
 * @brief  Enable the rtc enable
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline void ms_rtc_ll_enable(RTC_Type *rtc)
{
    SET_BIT(rtc->RTC_CCR, RTC_EN);
}

/**
 * @brief  Disable the rtc
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline void ms_rtc_ll_disable(RTC_Type *rtc)
{
    CLEAR_BIT(rtc->RTC_CCR, RTC_EN);
}


/**
 * @brief  Enable the rtc interrupt
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline void ms_rtc_ll_int_enable(RTC_Type *rtc)
{
    SET_BIT(rtc->RTC_CCR, RTC_IEN);
}

/**
 * @brief  Disable the rtc interrupt
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline void ms_rtc_ll_int_disable(RTC_Type *rtc)
{
    CLEAR_BIT(rtc->RTC_CCR, RTC_IEN);
}

/**
 * @brief  Clear the rtc interrupt
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline void ms_rtc_ll_int_clear(RTC_Type *rtc)
{
    READ_BIT(rtc->RTC_EOI, RTC_RTC_EOI);
}

/**
 * @brief  Enable the rtc prescaler
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline void ms_rtc_ll_enable_prescaler(RTC_Type *rtc)
{
    SET_BIT(rtc->RTC_CCR, RTC_PSCLR_EN);
}

/**
 * @brief  Disable the rtc prescaler
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline void ms_rtc_ll_disable_prescaler(RTC_Type *rtc)
{
    CLEAR_BIT(rtc->RTC_CCR, RTC_PSCLR_EN);
}

/**
 * @brief  Mask the rtc interrupt generation
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline void ms_rtc_ll_int_mask(RTC_Type *rtc)
{
    SET_BIT(rtc->RTC_CCR, RTC_MASK);
}

/**
 * @brief   Unmask the rtc interrupt generation
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline void ms_rtc_ll_int_unmask(RTC_Type *rtc)
{
    CLEAR_BIT(rtc->RTC_CCR, RTC_MASK);
}

/**
 * @brief  Set the rtc counter prescaler value
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline void ms_rtc_ll_set_counter_prescaler(RTC_Type *rtc, uint32_t val)
{
    WRITE_REG(rtc->RTC_CPSR, (val&RTC_COUNTER_PRESCALER_VALUE));
}

/**
 * @brief  Get the rtc counter prescaler value
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline uint32_t ms_rtc_ll_get_counter_prescaler(RTC_Type *rtc)
{
    return (READ_REG(rtc->RTC_CPSR)&RTC_COUNTER_PRESCALER_VALUE);
}

/**
 * @brief  Set the rtc counter match
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline void ms_rtc_ll_set_counter_match(RTC_Type *rtc, uint32_t val)
{
    WRITE_REG(rtc->RTC_CMR, val);
}

/**
 * @brief  Set the rtc counter match
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline uint32_t ms_rtc_ll_get_counter_match(RTC_Type *rtc)
{
    return READ_REG(rtc->RTC_CMR);
}

/**
 * @brief  Set the rtc counter load value
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline void ms_rtc_ll_set_counter_load(RTC_Type *rtc, uint32_t val)
{
    WRITE_REG(rtc->RTC_CLR, val);
}

/**
 * @brief  Get the rtc counter load value
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline uint32_t ms_rtc_ll_get_counter_load(RTC_Type *rtc)
{
    return READ_REG(rtc->RTC_CLR);
}

/**
 * @brief  Get the rtc current counter  value
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline uint32_t ms_rtc_ll_get_current_counter(RTC_Type *rtc)
{
    return READ_REG(rtc->RTC_CCVR);
}

/**
 * @brief  Get the rtc current prescaler counter value
 * @param  RTC_Type_t *rtc: rtc module description
 * @retval none
 */
static inline uint32_t ms_rtc_ll_get_current_prescaler_counter(RTC_Type *rtc)
{
    return (READ_REG(rtc->RTC_CPCVR)&RTC_CURRENT_PRESCALER_COUNTER_VALUE);
}

#endif /* MS_RTC_LL_H_ */
