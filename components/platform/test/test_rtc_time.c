/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  test_rtc_time.c
 * @brief
 * @author che.jiang
 * @date 2022年1月14日
 * @version 1.0
 * @Revision:
 */
#include <string.h>
#include "ms_rtc.h"
#include "rtc_time.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "unity.h"
#include "unity_test_runner.h"

static uint32_t time_cnt = 0;
const rtc_time_t init_time =
{
    .sec = 0,
    .min = 0,
    .hr = 12,
    .weekday = 5,
    .date = 5,
    .month = 3,
    .year = 52,
};

void test_rtc_time_alarm_callback(void)
{
    rtc_time_t cur_time;
    rtc_get_time(&cur_time);
    MS_LOGI(MS_DRIVER, "\r\n rtc alarm time:%d-%d-%d %d:%d:%d\r\n",
            cur_time.year+1970, cur_time.month, cur_time.date,
            cur_time.hr, cur_time.min, cur_time.sec);
}

TEST_CASE("sys","test_rtc_time", "[Driver/rtc]")
{
    rtc_time_t cur_time;
    MS_LOGI(MS_DRIVER, "test_rtc_time\r\n");

    rtc_time_init();

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, rtc_set_time(&init_time));
    rtc_time_register_alarm_config(10, test_rtc_time_alarm_callback);

    for(uint32_t i = 0; i < 10; i++)
    {
        wait_nop(5000000);
        rtc_get_time(&cur_time);

        MS_LOGI(MS_DRIVER, "\r\n rtc get cur time:%d-%d-%d %d:%d:%d\r\n",
                cur_time.year+1970, cur_time.month, cur_time.date,
                cur_time.hr, cur_time.min, cur_time.sec);
    }

    rtc_time_deinit();

    MS_LOGI(MS_DRIVER, "test_rtc_time over\r\n");
}

