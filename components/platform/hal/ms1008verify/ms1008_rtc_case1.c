/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  rtc_case
 * @brief
 * @author che.jiang
 * @date 2022-1-11
 * @version 1.0
 * @Revision:
 */
#include <ms_clock_hal.h>
#include <string.h>
#include "ms_rtc_hal.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "log.h"



RTC_Type *rtc_instance = RTC;

#define MAX_PRESCALER_COUNTER (0xFFFFFF)
#define MAX_MATCH_COUNTER     (0xFFFFFFFF)

void test_rtc_counter_init_base()
{
    /*config the uart clock*/
    MS_LOGI( MS_DRIVER, "test_uart1_init_base\r\n" );

    /*config the uart clock*/
    MS_CLOCK_HAL_CLK_ENABLE_RTC();

    /*Config Enable RTC module*/
    ms_rtc_hal_int_enable(rtc_instance);
    ms_rtc_hal_int_unmask(rtc_instance);
    ms_rtc_hal_enable(rtc_instance);
    ms_rtc_hal_enable_prescaler(rtc_instance);


}


TEST_CASE("rtc","ms1008_rtc_prescaler_counter_max", "[Driver/RTC]")
{
    uint32_t cur_prescaler_counter = 0;
    uint32_t cur_counter = 0;
    MS_LOGI( MS_DRIVER, "ms1008_rtc_prescaler_counter_max\r\n" );

    test_rtc_counter_init_base();

    ms_rtc_hal_set_counter_prescaler(rtc_instance, MAX_PRESCALER_COUNTER);
    ms_rtc_hal_set_counter_match(rtc_instance, 5);
    ms_rtc_hal_set_counter_load(rtc_instance, 0);

    MS_LOGI( MS_DRIVER, "set_prescaler_counter_max=0x%x,get_prescaler_counter_max=0x%x\r\n",
            MAX_PRESCALER_COUNTER, ms_rtc_hal_get_counter_prescaler(rtc_instance));

    while(1)
    {
        cur_prescaler_counter = ms_rtc_hal_get_current_prescaler_counter(rtc_instance);
        cur_counter = ms_rtc_hal_get_current_counter(rtc_instance);
        if(0 == (cur_prescaler_counter%32768))
        {
            MS_LOGI( MS_DRIVER, "cur_prescaler_counter=%u, cur_counter=%u\r\n",
                    cur_prescaler_counter, cur_counter);
        }
        if(cur_counter == 1)
        {
            MS_LOGI( MS_DRIVER, "get max prescaler counter:cur_prescaler_counter=%u, cur_counter=%u\r\n",
                    cur_prescaler_counter, cur_counter);
            break;
        }
    }
}

TEST_CASE("rtc","ms1008_rtc_counter_max", "[Driver/RTC]")
{
    uint32_t cur_prescaler_counter = 0;
    uint32_t cur_counter = 0;
    MS_LOGI( MS_DRIVER, "ms1008_rtc_counter_max\r\n" );

    test_rtc_counter_init_base();
    ms_rtc_hal_disable_prescaler(rtc_instance);
    ms_rtc_hal_set_counter_prescaler(rtc_instance, 1);
    ms_rtc_hal_set_counter_match(rtc_instance, MAX_MATCH_COUNTER);
    ms_rtc_hal_set_counter_load(rtc_instance, 0);

    MS_LOGI( MS_DRIVER, "set_counter_max=0x%x,get_counter_max=0x%x\r\n",
            MAX_MATCH_COUNTER, ms_rtc_hal_get_counter_match(rtc_instance));

    while(1)
    {
        cur_prescaler_counter = ms_rtc_hal_get_current_prescaler_counter(rtc_instance);
        cur_counter = ms_rtc_hal_get_current_counter(rtc_instance);
        if(0 == (cur_counter%32768))
        {
            MS_LOGI( MS_DRIVER, "cur_prescaler_counter=%u, cur_counter=%u\r\n",
                    cur_prescaler_counter, cur_counter);
        }
        if(cur_counter == MAX_MATCH_COUNTER)
        {
            MS_LOGI( MS_DRIVER, "get max counter:cur_prescaler_counter=%u, cur_counter=%u\r\n",
                    cur_prescaler_counter, cur_counter);
            break;
        }
    }
}




