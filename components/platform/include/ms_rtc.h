/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_rtc.h
 * @brief
 * @author che.jiang
 * @date 2021-12-30
 * @version 1.0
 * @Revision
 */
#ifndef MS_RTC_H_
#define MS_RTC_H_

#include "ms_rtc_hal.h"
#include <stdbool.h>


/**
 * @brief  RTC Configuration Structure definition
 */
typedef struct __RtcHandle_Type
{
    RTC_Type *instance;                                /*!< pointer to RTC controller registers base address*/
    IRQn_Type irq;                                     /*!< RTC IRQ Number                    */
    void (*init_callback)();                           /*!< RTC Init  callback */
    void (*deinit_callback)();                         /*!< RTC Deinit  callback */
    void (*alarm_callback)();                          /*!< alarm time callback */
    uint32_t alarm_time;                               /*!< alarm time interval(second) */
} RtcHandle_Type;


/**
 * @brief  Initial the RTC Module
 * @param  RtcHandle_Type *hrtc: RTC Instance
 * @retval STATUS_SUCCESS or STATUS_FAILURE
 */
extern int32_t ms_rtc_init(RtcHandle_Type *hrtc);

/**
 * @brief  De-initial the RTC
 * @param  RtcHandle_Type *hrtc: RTC Instance
 * @retval STATUS_SUCCESS or STATUS_FAILURE
 */
extern int32_t ms_rtc_deinit(RtcHandle_Type *hrtc);


/**
 * @brief  Enable the RTC Counter
 * @param  RtcHandle_Type *hrtc: RTC Instance
 * @retval none
 */
extern void ms_rtc_enable(RtcHandle_Type *hrtc);

/**
 * @brief  Disable the RTC Counter
 * @param  RtcHandle_Type *hrtc: RTC Instance
 * @retval none
 */
extern void ms_rtc_disable(RtcHandle_Type *hrtc);

/**
 * @brief  Disable the RTC Counter
 * @param  RtcHandle_Type *hrtc: RTC Instance
 * @retval none
 */
extern void ms_rtc_enable_cpu_interrupt(RtcHandle_Type *hrtc);

/**
 * @brief  Disable the RTC Counter
 * @param  RtcHandle_Type *hrtc: RTC Instance
 * @retval none
 */
extern void ms_rtc_disable_cpu_interrupt(RtcHandle_Type *hrtc);


/**
 * @brief  Get RTC Time(second)
 * @param  RtcHandle_Type *hrtc: RTC Instance
 * @retval none
 */
extern uint32_t ms_rtc_get_time(RtcHandle_Type *hrtc);

/**
 * @brief  Set RTC Alarm Time(second)
 * @param  RtcHandle_Type *hrtc: RTC Instance
 * @retval none
 */
extern int32_t ms_rtc_set_alarm_time(RtcHandle_Type *hrtc, uint32_t alarm_time);


/**
 * @brief  Interrupt Request Handle
 * @param[in]  RtcHandle_Type *hrtc:  Pointer to a RTCHandle_Type structure that contains
 *                the configuration information for the specified RTC module.
 * @retval none
 */
void ms_rtc_irq_handler(RtcHandle_Type *hrtc);

#endif /* MS_RTC_H_ */
