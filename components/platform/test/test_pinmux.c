/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  test_pinmux.c
  * @brief 
  * @author bingrui.chen
  * @date 2022年1月19日
  * @version 1.0
  * @Revision: 
  */



#include <ms_single_dmac.h>
#include <string.h>
#include "ms_uart.h"
#include "ms_clock_hal.h"
#include "ms_pinmux_hal.h"
#include "unity.h"
#include "uart.h"
#include "unity_test_runner.h"

TEST_CASE("gpio","test_pinmux_get_pinmux_handle", "[Driver/pinmux]")
//void test_pinmux_get_pinmux_handle(void)
{
    for(uint32_t i = 0; i < 27;i++)
    {
        TEST_ASSERT_NOT_EQUAL(NULL,ms_pinmux_hal_get_pinmux_handle(i));
    }

    TEST_ASSERT_EQUAL(NULL,ms_pinmux_hal_get_pinmux_handle(27));
    TEST_ASSERT_EQUAL(NULL,ms_pinmux_hal_get_pinmux_handle(28));
}

TEST_CASE("gpio","test_pinmux_set_pinmux", "[Driver/pinmux]")
//void test_pinmux_set_pinmux(void)
{
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,ms_pinmux_hal_set_pinmux(0,PIN00_GPIO_0));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,ms_pinmux_hal_set_pinmux(1,PIN01_GPIO_1));
    TEST_ASSERT_EQUAL_INT32(STATUS_ERROR,ms_pinmux_hal_set_pinmux(27,PIN00_GPIO_0));
#if   defined ( __MS1008_V2 )
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,ms_pinmux_hal_set_pinmux(8,PIN08_SPI0_TXD));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,ms_pinmux_hal_set_pinmux(9,PIN09_SPI0_RXD));
	//TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,ms_pinmux_hal_set_pinmux(8,PIN08_I2S0_MCLK));
    //TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,ms_pinmux_hal_set_pinmux(9,PIN09_I2S0_DO));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,ms_pinmux_hal_set_pinmux(22,PIN22_UART1_TXD));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,ms_pinmux_hal_set_pinmux(24,PIN24_UART0_RTS));
#endif
}


//void setUp(void)
//{
//
//}
//
//void tearDown(void)
//{
//
//}
//
//extern void system_clock_init(void);
//char hello_str[] = "Hello PINMUX\r\n";
//
//int main(void)
//{
//    system_clock_init();
//    ms_pinmux_hal_config_default();
//    ms_dmac_mgmt_inti();
//    uart0_int();
//    uart0_write(hello_str, strlen(hello_str));
//
//    uint32_t index = 0;
//
//    UNITY_BEGIN();
//    RUN_TEST(test_pinmux_get_pinmux_handle);
//    RUN_TEST(test_pinmux_set_pinmux);
//
//
//    return UNITY_END();
//}
