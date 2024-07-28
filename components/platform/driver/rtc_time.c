/*
 * rtc_time.c
 *
 *  Created on: 2021年12月24日
 *      Author: che.jiang
 */

#include <ms_clock_hal.h>
#include "ms_rtc.h"
#include "rtc_time.h"
#include <time.h>

typedef void (*rtc_time_cb_handler_t)();
rtc_time_cb_handler_t rtc_time_alarm_callback = NULL;

RtcHandle_Type rtc_time_handle;

RETENTION_RAM_DATA_SECTION rtc_time_t  g_rtc_time;

void rtc_time_init_calllback(struct __RtcHandle_Type *hrtc)
{
    /*config the uart clock*/
    MS_CLOCK_HAL_CLK_ENABLE_RTC();

    /*enable the interrupt*/
    //ms_rtc_enable_cpu_interrupt(hrtc);
}

void rtc_time_deinit_calllback(struct __RtcHandle_Type *hrtc)
{
    /*enable the interrupt*/
    //ms_rtc_disable_cpu_interrupt(hrtc);

    /*config the uart clock*/
    MS_CLOCK_HAL_CLK_DISABLE_RTC();
}


void rtc_time_alarm_calllback(struct __RtcHandle_Type *hrtc)
{
    ms_rtc_set_alarm_time(hrtc, hrtc->alarm_time);
    if(rtc_time_alarm_callback)
    {
        rtc_time_alarm_callback();
    }
}

void rtc_time_register_alarm_config(uint32_t alarm_time, void (*callback)())
{
    rtc_time_handle.alarm_time = alarm_time;
    ms_rtc_set_alarm_time(&rtc_time_handle, rtc_time_handle.alarm_time);
    rtc_time_alarm_callback = callback;
}

void rtc_time_init(void)
{
    rtc_time_handle.instance = RTC;
    rtc_time_handle.irq = RTC_IRQn;
    rtc_time_handle.init_callback = rtc_time_init_calllback;
    rtc_time_handle.deinit_callback = rtc_time_deinit_calllback;
    rtc_time_handle.alarm_callback = rtc_time_alarm_calllback;
    rtc_time_handle.alarm_time = 10;

    ms_rtc_init(&rtc_time_handle);

    /*enable the interrupt*/
    ms_rtc_enable_cpu_interrupt(&rtc_time_handle);
}

void rtc_time_deinit(void)
{
    ms_rtc_disable_cpu_interrupt(&rtc_time_handle);
    ms_rtc_deinit(&rtc_time_handle);
}

int32_t rtc_set_time(const rtc_time_t *time)
{
    CHECK_PTR_NULL_RET((void*)time, STATUS_ERROR);

    g_rtc_time.year = time->year;
    g_rtc_time.month = time->month;
    g_rtc_time.date = time->date;
    g_rtc_time.hr = time->hr;
    g_rtc_time.min = time->min;
    g_rtc_time.sec = time->sec;
    g_rtc_time.weekday = time->weekday;

    return STATUS_SUCCESS;
}

int32_t rtc_get_time(rtc_time_t *time)
{
    struct tm tm1;
    struct tm *p_tm1;
    time_t time1;

    CHECK_PTR_NULL_RET(time, STATUS_ERROR);

    time1 = ms_rtc_get_time(&rtc_time_handle);

    tm1.tm_year = g_rtc_time.year;
    tm1.tm_mon = g_rtc_time.month;
    tm1.tm_mday = g_rtc_time.date;
    tm1.tm_hour = g_rtc_time.hr;
    tm1.tm_min = g_rtc_time.min;
    tm1.tm_sec = g_rtc_time.sec;

    time1 += mktime(&tm1);

    p_tm1 = gmtime(&time1);

    time->year = p_tm1->tm_year;
    time->month = p_tm1->tm_mon;
    time->date = p_tm1->tm_mday;
    time->hr = p_tm1->tm_hour;
    time->min = p_tm1->tm_min;
    time->sec = p_tm1->tm_sec;
    time->weekday = p_tm1->tm_wday;

    return STATUS_SUCCESS;
}


void RTC_IRQHandler(void)
{
    ms_rtc_irq_handler(&rtc_time_handle);
}

