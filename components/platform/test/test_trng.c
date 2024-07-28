/*
 * test_trng.c
 *
 *  Created on: 2021年12月16日
 *      Author:haijun.mai
 */

#include "trng.h"
#include "uart.h"
#include <string.h>
#include "ms_uart.h"
#include "ms_clock_hal.h"
#include "ms_pinmux_hal.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "log.h"


TEST_CASE("sys","test_trng", "[Driver/trng]")
//void test_ms_flash_read_write(void)
{



    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, trng_init());
    for(uint8_t i = 0;i<20;i++)
    {
          MS_LOGI(MS_DRIVER,"\r\ndata[%d] = %x\n",i,trng_get_data());
           
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, trng_deinit());
  

  

}

