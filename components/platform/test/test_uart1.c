/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  test_uart1.c
 * @brief
 * @author bingrui.chen
 * @date 2022-1-11
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
#include "log.h"

UartHandle_Type uart1_handle;
uint8_t uart1_rx_buffer[256] = { 0 };

static uint8_t rx_done = 0;
static uint8_t tx_done = 0;

void uart1_init_calllback(struct __UartHandle_Type *huart)
{
    /*config the uart clock*/
    MS_CLOCK_HAL_CLK_ENABLE_UART1();
    ms_clock_hal_set_uart1_div(0);
    ms_clock_hal_peripheral_clk_div_toggle(UART1_DIV_TOG);

    /*config the uart pin mux*/
    //ms_pinmux_hal_set_pinmux(PAD22,PIN22_UART1_TXD);
    ms_pinmux_hal_set_pinmux(PAD6,PIN06_UART1_TXD);
    ms_pinmux_hal_set_pinmux(PAD7,PIN07_UART1_RXD);


    /*enable the interrupt*/
    ms_uart_enable_cpu_interrupt(huart);

    huart->hdmatx = ms_dmac_mgmt_alloc();
    huart->hdmatx->parent = huart;

    /*initial tx dmac*/
    huart->hdmatx->init.dst_tr_width = DMAC_XFER_WIDTH_8BITS;
    huart->hdmatx->init.dst_addr_mode = DMAC_ADDRESS_MODE_NOC;
    huart->hdmatx->init.dst_peri_type = DMAC_UART1_TX;
    huart->hdmatx->init.dst_msize = DMAC_MSIZE_1;
    huart->hdmatx->init.src_peri_type = DMAC_MEM;
    huart->hdmatx->init.src_tr_width = DMAC_XFER_WIDTH_8BITS;
    huart->hdmatx->init.src_addr_mode = DMAC_ADDRESS_MODE_INC;
    huart->hdmatx->init.src_msize = DMAC_MSIZE_1;
    ms_dmac_init(huart->hdmatx);

    huart->hdmarx = ms_dmac_mgmt_alloc();
    huart->hdmarx->parent = huart;

    /*initial rx dmac*/
    huart->hdmarx->init.dst_tr_width = DMAC_XFER_WIDTH_8BITS;
    huart->hdmarx->init.dst_addr_mode = DMAC_ADDRESS_MODE_INC;
    huart->hdmarx->init.dst_peri_type = DMAC_MEM;
    huart->hdmarx->init.dst_msize = DMAC_MSIZE_1;
    huart->hdmarx->init.src_peri_type = DMAC_UART1_RX;
    huart->hdmarx->init.src_tr_width = DMAC_XFER_WIDTH_8BITS;
    huart->hdmarx->init.src_addr_mode = DMAC_ADDRESS_MODE_NOC;
    huart->hdmarx->init.src_msize = DMAC_MSIZE_1;
    ms_dmac_init(huart->hdmarx);
}

void uart1_deinit_calllback(struct __UartHandle_Type *huart)
{
    /*enable the interrupt*/
    ms_uart_disable_cpu_interrupt(huart);

    /*config the uart clock*/
    MS_CLOCK_HAL_CLK_DISABLE_UART1();

    if (huart->hdmarx)
    {
        ms_dmac_stop_chx_xfer(huart->hdmarx);
        ms_dmac_mgmt_free(huart->hdmarx);
    }

    if (huart->hdmatx)
    {
        ms_dmac_stop_chx_xfer(huart->hdmatx);
        ms_dmac_mgmt_free(huart->hdmatx);
    }
}

void uart1_rx_calllback(struct __UartHandle_Type *huart)
{
    rx_done = 1;
}

void uart1_tx_calllback(struct __UartHandle_Type *huart)
{
    tx_done = 1;
}

UartCallback_Type uart1_callback = { .init_callback = uart1_init_calllback, .deinit_callback = uart1_deinit_calllback,
        .rx_cplt_callback = uart1_rx_calllback, .tx_cplt_callback = uart1_tx_calllback, };

void test_uart1_init(void)
{
    uart1_handle.instance = UART1;
    uart1_handle.init.baud_rate = 19200;
    uart1_handle.init.hw_flow_ctl = UART_HWCONTROL_NONE;
    uart1_handle.init.parity = UART_PARITY_NONE;
    uart1_handle.init.stop_bits = UART_STOPBITS_1;
    uart1_handle.init.word_length = UART_WORDLENGTH_8B;
    uart1_handle.init.hw_flow_ctl = UART_HWCONTROL_NONE;
    uart1_handle.init.fifo_enable = 1;
    uart1_handle.init.rx_fifo_level = UART_RX_FIFO_AVAILABLE_TRG_LVL_1_CHAR;
    uart1_handle.init.tx_fifo_level = UART_TX_FIFO_EMPTY_TRG_LVL_QUARTER;
    uart1_handle.hcallback = &uart1_callback;
    uart1_handle.irq = UART1_IRQn;
    ms_uart_init(&uart1_handle);
}

void test_uart1_deinit(void)
{
    ms_uart_deinit(&uart1_handle);
}

TEST_CASE("uart","test_uart1_loopback", "[Driver/Uart1]")
//void test_uart1_loopback(void)
{
    char tx_buffer[] = { "hello uart1\r\n" };
    char rx_buffer[128] = { 0 };
    char dst_buffer[128] = { 0 };

    uint32_t count = 0;

    test_uart1_init();
    ms_uart_hal_enable_loopback(uart1_handle.instance);
    ms_uart_start_rx_it(&uart1_handle, uart1_rx_buffer, 256);
    ms_uart_transmit(&uart1_handle, tx_buffer, sizeof(tx_buffer));

    char *ptr = dst_buffer;
    int32_t len = 0;

    while (1)
    {
        if ((len = ms_uart_receive(&uart1_handle, rx_buffer, 128)) > 0)
        {
            memcpy(ptr, rx_buffer, len);
            ptr += len;
            continue;
        }

        if (count++ > 10000)
        {
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT8_ARRAY(tx_buffer, dst_buffer, sizeof(tx_buffer));

    ms_uart_stop_rx_it(&uart1_handle);
    ms_uart_hal_disable_loopback(uart1_handle.instance);
    test_uart1_deinit();
}

TEST_CASE("uart","test_uart1_loopback_it", "[Driver/Uart1]")
//void test_uart1_loopback_it(void)
{
    char tx_buffer[32] = { 0 };
    char rx_buffer[128] = { 0 };

    for (uint32_t i = 0; i < 32; i++)
    {
        tx_buffer[i] = i;
        rx_buffer[i] = 0;
    }

    uint32_t count = 0;
    rx_done = 0;
    test_uart1_init();
    ms_uart_hal_enable_loopback(uart1_handle.instance);
    ms_uart_start_rx_dma(&uart1_handle, rx_buffer, 32);
    ms_uart_transmit_it(&uart1_handle, tx_buffer, 32);

    while ((!rx_done) && (count < 5000000))
    {
        count++;
    }

    TEST_ASSERT_EQUAL_INT8(1, rx_done);
    TEST_ASSERT_EQUAL_INT8_ARRAY(tx_buffer, rx_buffer, 32);

    ms_uart_hal_disable_loopback(uart1_handle.instance);
    test_uart1_deinit();
}

TEST_CASE("uart","test_uart1_loopback_rx_dma", "[Driver/Uart1]")
//void test_uart1_loopback_rx_dma(void)
{
    char tx_buffer[32] = { 0 };
    char rx_buffer[128] = { 0 };

    test_uart1_init();
    ms_uart_hal_enable_loopback(uart1_handle.instance);

    for(uint8_t i = 0;i < 3;i++)
    {
        for (uint32_t i = 0; i < 32; i++)
        {
            tx_buffer[i] = i;
            rx_buffer[i] = 0;
        }

        uint32_t count = 0;
        rx_done = 0;
        ms_uart_start_rx_dma(&uart1_handle, rx_buffer, 32);
        ms_uart_transmit(&uart1_handle, tx_buffer, 32);

        while ((!rx_done) && (count < 50000))
        {
            count++;
        }

        TEST_ASSERT_EQUAL_INT8(1, rx_done);

        TEST_ASSERT_EQUAL_INT8_ARRAY(tx_buffer, rx_buffer, 32);
    }
    ms_uart_hal_disable_loopback(uart1_handle.instance);
    test_uart1_deinit();
}


TEST_CASE("uart","test_uart1_loopback_tx_dma", "[Driver/Uart1]")
//void test_uart1_loopback_tx_dma(void)
{
    const char send_len = 32;
    char tx_buffer[32] = { 0 };
    char rx_buffer[128] = { 0 };
    char dst_buffer[128] = { 0 };
    for (uint32_t i = 0; i < 32; i++)
    {
        tx_buffer[i] = i;
        dst_buffer[i] = 0;
    }

    uint32_t count = 0;

    test_uart1_init();
    ms_uart_hal_enable_loopback(uart1_handle.instance);
    tx_done = 0;
    ms_uart_start_rx_it(&uart1_handle, uart1_rx_buffer, 256);
    ms_uart_transmit_dma(&uart1_handle, tx_buffer, 32);

    char *ptr = dst_buffer;
    int32_t len = 0;

    while (1)
    {
        if ((len = ms_uart_receive(&uart1_handle, rx_buffer, 128)) > 0)
        {
            memcpy(ptr, rx_buffer, len);
            ptr += len;
            continue;
        }

        if (count++ > 10000)
        {
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT8_ARRAY(tx_buffer, dst_buffer, send_len);

    ms_uart_stop_rx_it(&uart1_handle);
    ms_uart_hal_disable_loopback(uart1_handle.instance);
    test_uart1_deinit();
}

TEST_CASE("uart","test_uart1_loopback_rx_tx_dma", "[Driver/Uart1]")
//void test_uart1_loopback_rx_tx_dma(void)
{
    char tx_buffer[32] = { 0 };
    char rx_buffer[128] = { 0 };

    for (uint32_t i = 0; i < 32; i++)
    {
        tx_buffer[i] = i;
        rx_buffer[0] = 0;
    }

    uint32_t count = 0;
    rx_done = 0;
    test_uart1_init();
    ms_uart_hal_enable_loopback(uart1_handle.instance);
    ms_uart_start_rx_dma(&uart1_handle, rx_buffer, 32);
    ms_uart_transmit_dma(&uart1_handle, tx_buffer, 32);

    while ((!rx_done) && (count < 50000))
    {
        count++;
    }

    TEST_ASSERT_EQUAL_INT8(1, rx_done);

    TEST_ASSERT_EQUAL_INT8_ARRAY(tx_buffer, rx_buffer, 32);

    ms_uart_hal_disable_loopback(uart1_handle.instance);
    test_uart1_deinit();
}


TEST_CASE("uart","test_uart1_send_reveive", "[Driver/Uart1]")
{
    char tx_buffer[] = { "hello uart1\r\n" };
    char rx_buffer[128] = { 0 };
    char dst_buffer[128] = { 0 };

    test_uart1_init();
    ms_uart_start_rx_it(&uart1_handle, uart1_rx_buffer, 256);
    ms_uart_transmit(&uart1_handle, tx_buffer, sizeof(tx_buffer));

    char *ptr = dst_buffer;
    int32_t len = 0;
    int32_t rx_len = 0;

    while (1)
    {
        if ((len = ms_uart_receive(&uart1_handle, rx_buffer, 128)) > 0)
        {
            memcpy(ptr, rx_buffer, len);
            ptr += len;
            rx_len += len;
            continue;
        }

        if (rx_len > 100)
        {
            break;
        }
    }
    MS_LOGI(MS_DRIVER, "\r\nrx_len=%d!\n", rx_len);
    ms_uart_transmit(&uart1_handle, dst_buffer, rx_len);

    ms_uart_stop_rx_it(&uart1_handle);

    test_uart1_deinit();
}

int32_t test_uart1_set_baudrate(uint32_t baudrate)
{
    return ms_uart_set_baudrate(&uart1_handle, baudrate);
}

void UART1_IRQHandler(void)
{
    ms_uart_irq_handler(&uart1_handle);
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

//extern void system_clock_init(void);
//char hello_str[] = "Hello Uart1\r\n";
//
//int main(void)
//{
//    system_clock_init();
//    ms_pinmux_hal_config_default();
//    ms_dmac_mgmt_inti();
//    uart0_int();
//    uart0_write(hello_str, strlen(hello_str));
//
//    UNITY_BEGIN();
//    RUN_TEST(test_uart1_loopback);
//    RUN_TEST(test_uart1_loopback_rx_dma);
//    RUN_TEST(test_uart1_loopback_rx_dma);
//    RUN_TEST(test_uart1_loopback_tx_dma);
//    RUN_TEST(test_uart1_loopback_rx_tx_dma);
//    RUN_TEST(test_uart1_loopback_rx_tx_dma);
//    RUN_TEST(test_uart1_loopback_it);
//    RUN_TEST(test_uart1_loopback_rx_dma);
//    RUN_TEST(test_uart1_loopback_tx_dma);
//
//    return UNITY_END();
//}
