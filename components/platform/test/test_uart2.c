/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  test_uart2.c
 * @brief
 * @author bingrui.chen
 * @date 2022年1月17日
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

UartHandle_Type uart2_handle;
uint8_t uart2_rx_buffer[256] = { 0 };

void uart2_init_calllback_pclk(struct __UartHandle_Type *huart)
{
    /*config the uart clock*/
    MS_CLOCK_HAL_CLK_ENABLE_UART2_PCLK();
    ms_clock_hal_uart2_fclk_sel(UART2_CLK_SEL_PCLK);

    /*Enable Communication Clock*/
    MS_CLOCK_HAL_CLK_ENABLE_UART2_COMM_CLK();

    uint32_t clk_sel = READ_REG(SYS_CTRL->CLK_SEL);
    uint32_t en0 = READ_REG(SYS_CTRL->CLK_EN0);
    uint32_t en1 = READ_REG(SYS_CTRL->CLK_EN1);

    /*config the uart pin mux*/
    ms_pinmux_hal_set_pinmux(PAD10,PIN10_UART2_TXD);
    //ms_pinmux_hal_set_pinmux(PAD13,PIN13_UART2_TXD);
    ms_pinmux_hal_set_pinmux(PAD11,PIN11_UART2_RXD);


    /*enable the interrupt*/
    ms_uart_enable_cpu_interrupt(huart);
}

void uart2_deinit_calllback(struct __UartHandle_Type *huart)
{
    /*enable the interrupt*/
    ms_uart_disable_cpu_interrupt(huart);

    /*config the uart clock*/
    MS_CLOCK_HAL_CLK_DISABLE_UART2();
    MS_CLOCK_HAL_CLK_DISABLE_UART2_PCLK();

}

void uart2_rx_calllback(struct __UartHandle_Type *huart)
{

}

void uart2_tx_calllback(struct __UartHandle_Type *huart)
{

}

UartCallback_Type uart2_callback_pclk =
{
    .init_callback = uart2_init_calllback_pclk,
    .deinit_callback = uart2_deinit_calllback,
    .rx_cplt_callback = uart2_rx_calllback,
    .tx_cplt_callback = uart2_tx_calllback
};



void test_uart2_init_pclk(void)
{
    uart2_handle.instance = UART2;
    uart2_handle.init.baud_rate = 2000;
    uart2_handle.init.hw_flow_ctl = UART_HWCONTROL_NONE;
    uart2_handle.init.parity = UART_PARITY_NONE;
    uart2_handle.init.stop_bits = UART_STOPBITS_1;
    uart2_handle.init.word_length = UART_WORDLENGTH_8B;
    uart2_handle.init.fifo_enable = 1;
    uart2_handle.init.rx_fifo_level = UART_RX_FIFO_AVAILABLE_TRG_LVL_HALF;
    uart2_handle.init.tx_fifo_level = UART_TX_FIFO_EMPTY_TRG_LVL_QUARTER;
    uart2_handle.hcallback = &uart2_callback_pclk;
    uart2_handle.irq = UART2_IRQn;
    ms_uart_init(&uart2_handle);
}


void test_uart2_deinit(void)
{
    ms_uart_deinit(&uart2_handle);
}

TEST_CASE("uart","test_uart2_loopback", "[Driver/Uart2]")
//void test_uart2_loopback(void)
{
    char tx_buffer[] = { "hello uart2\r\n" };
    char rx_buffer[128] = { 0 };
    char dst_buffer[128] = { 0 };

    uint32_t count = 0;

    test_uart2_init_pclk();
    ms_uart_hal_enable_loopback(uart2_handle.instance);
    ms_uart_start_rx_it(&uart2_handle, uart2_rx_buffer, 256);
    ms_uart_transmit(&uart2_handle, tx_buffer, sizeof(tx_buffer));

    char *ptr = dst_buffer;
    int32_t len = 0;

    while (1)
    {
        if ((len = ms_uart_receive(&uart2_handle, rx_buffer, 128)) > 0)
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

    ms_uart_stop_rx_it(&uart2_handle);
    ms_uart_hal_disable_loopback(uart2_handle.instance);
    test_uart2_deinit();
}

TEST_CASE("uart","test_uart2_set_baudrate", "[Driver/Uart2]")
//void test_uart2_set_baudrate(void)
{
    test_uart2_init_pclk();
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,ms_uart_set_baudrate(&uart2_handle, 2000));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,ms_uart_set_baudrate(&uart2_handle, 1000));
    TEST_ASSERT_EQUAL_INT32(STATUS_ERROR,ms_uart_set_baudrate(&uart2_handle, 4000));
    TEST_ASSERT_EQUAL_INT32(STATUS_ERROR,ms_uart_set_baudrate(&uart2_handle, 0));
    test_uart2_deinit();
}

//int32_t test_uart2_set_baudrate(uint32_t baudrate)
//{
//    return ms_uart_set_baudrate(&uart2_handle, baudrate);
//}

void UART2_IRQHandler(void)
{
    ms_uart_irq_handler(&uart2_handle);
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
//char hello_str[] = "Hello Uart2\r\n";
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
//    RUN_TEST(test_uart2_set_baudrate);
//    RUN_TEST(test_uart2_loopback);
//
//
//    return UNITY_END();
//}

