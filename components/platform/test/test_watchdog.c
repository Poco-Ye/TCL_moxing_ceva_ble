/*
 * test_watchdog.c
 *
 *  Created on: 2021年12月16日
 *      Author:haijun.mai
 */

#include "watchdog.h"
#include "uart.h"
#include <string.h>
#include "ms_uart.h"
#include "ms_clock_hal.h"
#include "ms_pinmux_hal.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"




TEST_CASE("sys","test watchdog", "[Driver/watchdog]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, watchdog_init());
    for(uint8_t i = 0;i<20;i++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, watchdog_feed());
          MS_LOGI(MS_DRIVER,"\r\nwatchdog feed %d\n",i);
          vTaskDelay(1500);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, watchdog_deinit());

}




TEST_CASE("sys","test watchdog close", "[Driver/watchdog close]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, watchdog_init());
    for(uint8_t i = 0;i<20;i++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, watchdog_feed());
          MS_LOGI(MS_DRIVER,"\r\nwatchdog feed %d\n",i);
          vTaskDelay(1500);
    }
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, watchdog_close());
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, watchdog_deinit());

}



