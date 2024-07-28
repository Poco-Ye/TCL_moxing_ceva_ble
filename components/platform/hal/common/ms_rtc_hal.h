/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  ms_rtc_hal.h
  * @brief head   file of rtc  module.
  * @author che.jiang
  * @date 2022年1月4日
  * @version 1.0
  * @Revision: 
  */
#ifndef BLE_SOC_MS1008_RTC_HAL_H_
#define BLE_SOC_MS1008_RTC_HAL_H_

#include "ms_rtc_ll.h"



/**
 * @brief  RTC enable the counter
 */
#define  ms_rtc_hal_enable(hrtc) ms_rtc_ll_enable(hrtc)


/**
 * @brief  RTC disable the counter
 */
#define  ms_rtc_hal_disable(hrtc) ms_rtc_ll_disable(hrtc)

/**
 * @brief  RTC enable the interrupt generation
 */
#define  ms_rtc_hal_int_enable(hrtc) ms_rtc_ll_int_enable(hrtc)

/**
 * @brief  RTC mask the interrupt generation
 */
#define  ms_rtc_hal_int_mask(hrtc) ms_rtc_ll_int_mask(hrtc)

/**
 * @brief  RTC mask the interrupt generation
 */
#define  ms_rtc_hal_int_unmask(hrtc) ms_rtc_ll_int_unmask(hrtc)

/**
 * @brief  RTC disable the interrupt generation
 */
#define  ms_rtc_hal_int_disable(hrtc) ms_rtc_ll_int_disable(hrtc)

/**
 * @brief  RTC clear the match interrupt
 */
#define  ms_rtc_hal_int_clear(hrtc) ms_rtc_ll_int_clear(hrtc)

/**
 * @brief  RTC enable the prescaler counter
 */
#define  ms_rtc_hal_enable_prescaler(hrtc) ms_rtc_ll_enable_prescaler(hrtc)

/**
 * @brief  RTC disable the prescaler counter
 */
#define  ms_rtc_hal_disable_prescaler(hrtc) ms_rtc_ll_disable_prescaler(hrtc)

/**
 * @brief  RTC set the prescaler value
 */
#define  ms_rtc_hal_set_counter_prescaler(hrtc,val) ms_rtc_ll_set_counter_prescaler(hrtc,val)

/**
 * @brief  RTC set the counter match value
 */
#define  ms_rtc_hal_set_counter_match(hrtc,val) ms_rtc_ll_set_counter_match(hrtc,val)

/**
 * @brief  Get UART Receiver Data Ready State
 */
#define  ms_rtc_hal_set_counter_load(hrtc,val) ms_rtc_ll_set_counter_load(hrtc,val)


/**
 * @brief  RTC Get the counter prescaler value
 */
#define  ms_rtc_hal_get_counter_prescaler(hrtc) ms_rtc_ll_get_counter_prescaler(hrtc)

/**
 * @brief  RTC set the counter match value
 */
#define  ms_rtc_hal_get_counter_match(hrtc) ms_rtc_ll_get_counter_match(hrtc)

/**
 * @brief  RTC Get the counter load value
 */
#define  ms_rtc_hal_get_counter_load(hrtc) ms_rtc_ll_get_counter_load(hrtc)

/**
 * @brief  RTC Get the current counter value
 */
#define  ms_rtc_hal_get_current_counter(hrtc) ms_rtc_ll_get_current_counter(hrtc)

/**
 * @brief  RTC Get the current prescaler counter value
 */
#define  ms_rtc_hal_get_current_prescaler_counter(hrtc) ms_rtc_ll_get_current_prescaler_counter(hrtc)

#endif /* BLE_SOC_MS1008_RTC_HAL_H_ */
