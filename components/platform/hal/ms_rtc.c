/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_rtc.c
 * @brief c source  file of rtc  module.
 * @author che.jiang
 * @date   2022-03-17
 * @version 1.0
 * @Revision
 */

#include <ms1008.h>
#include "ms_clock_hal.h"
#include "ms_interrupt.h"
#include "ms_rtc.h"
#include <stddef.h>



int32_t ms_rtc_init(RtcHandle_Type *hrtc)
{
    CHECK_PTR_NULL_RET(hrtc, STATUS_ERROR);

    /*Enable power, clock and interrupt setting*/
    if (hrtc->init_callback)
    {
        hrtc->init_callback();
    }

    /*Config Enable RTC module*/
    ms_rtc_hal_int_enable(hrtc->instance);
    ms_rtc_hal_int_unmask(hrtc->instance);
    ms_rtc_hal_enable(hrtc->instance);
    ms_rtc_hal_enable_prescaler(hrtc->instance);


    /*Config RTC Time counter, 1HZ*/
    ms_rtc_hal_set_counter_prescaler(hrtc->instance, 32768);
    ms_rtc_hal_set_counter_match(hrtc->instance, hrtc->alarm_time);
    ms_rtc_hal_set_counter_load(hrtc->instance, 0);

    return STATUS_SUCCESS;
}

int32_t ms_rtc_deinit(RtcHandle_Type *hrtc)
{
    /*Config Disable RTC module*/
    ms_rtc_hal_int_mask(hrtc->instance);
    ms_rtc_hal_int_disable(hrtc->instance);
    ms_rtc_hal_disable(hrtc->instance);
    ms_rtc_hal_disable_prescaler(hrtc->instance);

    /*Disable power, clock and interrupt setting*/
    if (hrtc->deinit_callback)
    {
        hrtc->deinit_callback();
    }

    return STATUS_SUCCESS;
}

void ms_rtc_enable(RtcHandle_Type *hrtc)
{
    ms_rtc_hal_enable(hrtc->instance);
}

void ms_rtc_disable(RtcHandle_Type *hrtc)
{
    ms_rtc_hal_disable(hrtc->instance);
}

void ms_rtc_enable_cpu_interrupt(RtcHandle_Type *hrtc)
{
    INTERRUPT_ENABLE_IRQ(hrtc->irq);
}


void ms_rtc_disable_cpu_interrupt(RtcHandle_Type *hrtc)
{
    INTERRUPT_DISABLE_IRQ(hrtc->irq);
}


uint32_t ms_rtc_get_time(RtcHandle_Type *hrtc)
{
    uint32_t rtc_clk = ms_clock_hal_get_lf_clk_freq();
    if(0 == rtc_clk)
    {
        return 0;
    }

    return  (ms_rtc_hal_get_current_counter(hrtc->instance)*\
            ms_rtc_hal_get_counter_prescaler(hrtc->instance)/rtc_clk);
}

int32_t ms_rtc_set_time(RtcHandle_Type *hrtc)
{
    ms_rtc_hal_set_counter_load(hrtc->instance, ms_rtc_hal_get_current_counter(hrtc->instance));
}

int32_t ms_rtc_set_alarm_time(RtcHandle_Type *hrtc, uint32_t alarm_time)
{
    hrtc->alarm_time = alarm_time;

    ms_rtc_hal_set_counter_load(hrtc->instance, ms_rtc_hal_get_current_counter(hrtc->instance));
    ms_rtc_hal_set_counter_match(hrtc->instance, ms_rtc_hal_get_current_counter(hrtc->instance)+alarm_time);

    return STATUS_SUCCESS;
}

void ms_rtc_irq_handler(RtcHandle_Type *hrtc)
{
    ms_rtc_hal_int_disable(hrtc->instance);
    ms_rtc_hal_int_clear(hrtc->instance);

    if(hrtc->alarm_callback)
    {
        hrtc->alarm_callback();
    }

    ms_rtc_hal_int_enable(hrtc->instance);
}
