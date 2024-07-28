/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  test_dmac.c
 * @brief
 * @author bingrui.chen
 * @date 2022-1-11
 * @version 1.0
 * @Revision:
 */
#include <ms_clock_hal.h>
#include <ms_pinmux_hal.h>
#include <ms_single_dmac.h>
#include <string.h>
#include "ms_uart.h"
#include "unity.h"
#include "uart.h"
#include "unity_test_runner.h"
#include "log.h"



TEST_CASE("dmac","ms1008_dmac_case1_alloc", "[Driver/DMA]")
{
 
    MS_LOGI( MS_DRIVER, "ms1008_dmac_case1_alloc\r\n" );

    DmacHandle_Type *hdmac_test =  ms_dmac_mgmt_alloc();
    TEST_ASSERT_EQUAL_INT8(1, hdmac_test->state);

    ms_dmac_mgmt_free(hdmac_test);
    TEST_ASSERT_EQUAL_INT8(0, hdmac_test->state);

    DmacHandle_Type *hdmac_test1 =  ms_dmac_mgmt_alloc();
    TEST_ASSERT_NOT_EQUAL(NULL,hdmac_test1);

    DmacHandle_Type *hdmac_test2 =  ms_dmac_mgmt_alloc();
    TEST_ASSERT_NOT_EQUAL(NULL,hdmac_test2);

    DmacHandle_Type *hdmac_test3 =  ms_dmac_mgmt_alloc();
    TEST_ASSERT_NOT_EQUAL(NULL,hdmac_test3);

    DmacHandle_Type *hdmac_test4 =  ms_dmac_mgmt_alloc();
    TEST_ASSERT_NOT_EQUAL(NULL,hdmac_test4);

    DmacHandle_Type *hdmac_test5 =  ms_dmac_mgmt_alloc();
    TEST_ASSERT_NOT_EQUAL(NULL,hdmac_test5);

    DmacHandle_Type *hdmac_test6 =  ms_dmac_mgmt_alloc();
    TEST_ASSERT_EQUAL(NULL,hdmac_test6);

    ms_dmac_mgmt_free(hdmac_test1);
    ms_dmac_mgmt_free(hdmac_test2);
    ms_dmac_mgmt_free(hdmac_test3);
    ms_dmac_mgmt_free(hdmac_test4);
    ms_dmac_mgmt_free(hdmac_test5); 
}

TEST_CASE("dmac","ms1008_dmac_case2_mem2mem_no_interrupt_WIDTH_32BITS", "[Driver/DMA]")
{

    MS_LOGI( MS_DRIVER, "ms1008_dmac_case2_mem2mem_no_interrupt_WIDTH_32BITS\r\n" );

}
