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

DmacHandle_Type *hdmac;

uint32_t is_xfer_done = 0;

void test_xfer_tfr_callback(struct __DmacHandle_Type *hdma)
{
    is_xfer_done = 1;
}

void test_xfer_error_callback(struct __DmacHandle_Type *hdma)
{
    is_xfer_done = 2;
}

void test_dmac_init_mem2mem(uint32_t dst_tr_width, uint32_t src_tr_width, uint32_t dst_msize, uint32_t src_msize)
{
    hdmac = ms_dmac_mgmt_alloc();

    /*initial tx dmac*/
    hdmac->init.dst_tr_width = dst_tr_width;
    hdmac->init.dst_addr_mode = DMAC_ADDRESS_MODE_INC;
    hdmac->init.dst_peri_type = DMAC_MEM;
    hdmac->init.dst_msize = dst_msize;
    hdmac->init.src_peri_type = DMAC_MEM;
    hdmac->init.src_tr_width = src_tr_width;
    hdmac->init.src_addr_mode = DMAC_ADDRESS_MODE_INC;
    hdmac->init.src_msize = src_msize;
    hdmac->xfer_tfr_callback = test_xfer_tfr_callback;
    hdmac->xfer_error_callback = test_xfer_error_callback;
    ms_dmac_init(hdmac);
}

void test_dmac_deinit_mem2mem(void)
{
    ms_dmac_stop_chx_xfer(hdmac);
    ms_dmac_mgmt_free(hdmac);
}

/*
void setUp(void)
{

}

void tearDown(void)
{

}
*/
TEST_CASE("dma","test_dmac_alloc", "[Driver/DMA]")
//void test_dmac_alloc(void)
{
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

TEST_CASE("dma","test_dmac_mem2mem_no_interrupt_WIDTH_32BITS", "[Driver/DMA]")
//void test_dmac_mem2mem_no_interrupt_WIDTH_32BITS(void)
{
    int32_t src_buffer[128] = { 0 };
    int32_t dst_buffer[128] = { 0 };
    uint32_t count = 0;

    MS_LOGI( MS_DRIVER, "WIDTH_32BITS\r\n" );
    for (uint32_t i = 0; i < 128; i++)
    {
        src_buffer[i] = i;
        dst_buffer[i] = 0;
    }

    test_dmac_init_mem2mem(DMAC_XFER_WIDTH_32BITS, DMAC_XFER_WIDTH_32BITS, DMAC_MSIZE_1, DMAC_MSIZE_1);

    ms_dmac_start_chx_xfer(hdmac, (uint32_t) &src_buffer[0], (uint32_t) &dst_buffer[0], 128 * sizeof(uint32_t));

    while ((!is_xfer_done) && (count < 50000))
    {
        count++;
    }

    /*Clear any pending interrupts */
    ms_dmac_clear_all_int(hdmac);

    TEST_ASSERT_EQUAL_INT_ARRAY(src_buffer, dst_buffer, 128);

    test_dmac_deinit_mem2mem();
}

TEST_CASE("dma","test_dmac_mem2mem_interrupt_WIDTH_16BITS", "[Driver/DMA]")
//void test_dmac_mem2mem_interrupt_WIDTH_16BITS(void)
{
    int16_t src_buffer[128] = { 0 };
    int16_t dst_buffer[128] = { 0 };
    uint32_t count = 0;

    MS_LOGI( MS_DRIVER, "WIDTH_16BITS\r\n" );

    for (uint32_t i = 0; i < 128; i++)
    {
        src_buffer[i] = i;
        dst_buffer[i] = 0;
    }

    is_xfer_done = 0;


    test_dmac_init_mem2mem(DMAC_XFER_WIDTH_16BITS, DMAC_XFER_WIDTH_16BITS, DMAC_MSIZE_1, DMAC_MSIZE_1);

    ms_dmac_start_chx_xfer_it(hdmac, (uint32_t) &src_buffer[0], (uint32_t) &dst_buffer[0], 128 * sizeof(int16_t));

    while ((!is_xfer_done) && (count < 50000))
    {
        count++;
    }

    TEST_ASSERT_EQUAL_INT32(1, is_xfer_done);

    TEST_ASSERT_EQUAL_INT16_ARRAY(src_buffer, dst_buffer, 128);

    test_dmac_deinit_mem2mem();
}

TEST_CASE("dma","test_dmac_mem2mem_interrupt_WIDTH_8BITS", "[Driver/DMA]")
//void test_dmac_mem2mem_interrupt_WIDTH_8BITS(void)
{
    int8_t src_buffer[128] = { 0 };
    int8_t dst_buffer[128] = { 0 };
    uint32_t count = 0;

    MS_LOGI( MS_DRIVER, "WIDTH_8BITS\r\n" );
    for (uint32_t i = 0; i < 128; i++)
    {
        src_buffer[i] = i;
        dst_buffer[i] = 0;
    }

    is_xfer_done = 0;

    test_dmac_init_mem2mem(DMAC_XFER_WIDTH_8BITS, DMAC_XFER_WIDTH_8BITS, DMAC_MSIZE_1, DMAC_MSIZE_1);

    ms_dmac_start_chx_xfer_it(hdmac, (uint32_t) &src_buffer[0], (uint32_t) &dst_buffer[0], 128 * sizeof(int8_t));

    while ((!is_xfer_done) && (count < 50000))
    {
        count++;
    }

    TEST_ASSERT_EQUAL_INT32(1, is_xfer_done);

    TEST_ASSERT_EQUAL_INT8_ARRAY(src_buffer, dst_buffer, 128);

    test_dmac_deinit_mem2mem();
}

/*

extern void system_clock_init(void);

int main(void)
{
    system_clock_init();
    ms_pinmux_hal_config_default();
    ms_dmac_mgmt_inti();
    uart0_int();

    UNITY_BEGIN();
    RUN_TEST(test_dmac_alloc);
    RUN_TEST(test_dmac_mem2mem_no_interrupt_WIDTH_32BITS);
    RUN_TEST(test_dmac_mem2mem_interrupt_WIDTH_16BITS);
    RUN_TEST(test_dmac_mem2mem_interrupt_WIDTH_8BITS);
    return UNITY_END();
}
*/
