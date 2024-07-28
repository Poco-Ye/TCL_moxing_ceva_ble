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


#define TEST_UART_TRANSMIT_CNT               (0x000000040U)
#define TEST_UART_TRANSMIT_SIZE              (0x000000100U)

Uart_Type *instance = UART1;
uint32_t recive_cnt = 0;
char test_recieve_buf[TEST_UART_TRANSMIT_SIZE] = {0};
char test_transmit_buf[TEST_UART_TRANSMIT_SIZE] = {"test_uart1_send\r\n"};

void test_uart1_init_base(uint8_t parity, uint8_t stopbit)
{
    /*config the uart clock*/
    MS_LOGI( MS_DRIVER, "test_uart1_init_base\r\n" );

    /*config the uart clock*/
    MS_CLOCK_HAL_CLK_ENABLE_UART1();
    ms_clock_hal_set_uart1_div(1);
    ms_clock_hal_peripheral_clk_div_toggle(UART1_DIV_TOG);

    /*config the uart pin mux*/
    ms_pinmux_hal_set_pinmux(PAD18,PIN18_UART1_TXD);
    ms_pinmux_hal_set_pinmux(PAD19,PIN19_UART1_RXD);

    /*Disable the peripheral */
    while (ms_uart_hal_is_busy(instance));

    /* Configure the UART Stop Bits */
    ms_uart_hal_set_stop_bits_num(instance, stopbit);

    /* Configure the UART Word Length, Parity and mode*/

    ms_uart_hal_set_parity(instance, parity);
    ms_uart_hal_set_word_length(instance, UART_WORDLENGTH_8B);

    /* Configure the UART HFC: Set CTSE and RTSE bits according to huart->Init.HwFlowCtl value */

    ms_uart_hal_set_auto_flow_ctrl(instance, AUTO_FLOW_CTRL_DISABLE);
    ms_uart_hal_set_rts(instance, FLOW_CTRL_RTS_DEASSERT);

    /* Set USART parity odd Configuration */

    /* Set USART BRR Configuration */
    ms_uart_hal_set_baudrate(instance, 19200);
}

void test_uart1_transmit_data()
{
#if 1
    /* Send data*/
    for (uint16_t i = 0; i < TEST_UART_TRANSMIT_CNT; i++)
    {
        while (!ms_uart_hal_is_txe(instance))
            ;
        ms_uart_hal_write_tx_data(instance, test_transmit_buf[i]);
    }

    recive_cnt = 0;
    /* Recieve data*/
    do
    {
        while (!ms_uart_hal_is_rxne(instance));

        test_recieve_buf[recive_cnt] = (uint8_t)ms_uart_hal_read_rx_data(instance);
        if(recive_cnt++ >= (TEST_UART_TRANSMIT_SIZE - 1))
        {
            break;
        }

    }while(1);

    /* Send data*/
    for (uint16_t i = 0; i < recive_cnt; i++)
    {
        while (!ms_uart_hal_is_txe(instance));
        ms_uart_hal_write_tx_data(instance, test_recieve_buf[i]);
    }

#endif
}


TEST_CASE("uart","ms1008_uart_parity_odd", "[Driver/UART1]")
{
    MS_LOGI( MS_DRIVER, "ms1008_uart_parity_odd\r\n" );

    test_uart1_init_base(UART_PARITY_ODD, UART_STOPBITS_1);

    test_uart1_transmit_data();

}

TEST_CASE("uart","ms1008_uart_parity_even", "[Driver/UART1]")
{

    MS_LOGI( MS_DRIVER, "ms1008_uart_parity_even\r\n" );

    test_uart1_init_base(UART_PARITY_EVEN, UART_STOPBITS_1);

    test_uart1_transmit_data();
}

TEST_CASE("uart","ms1008_uart_stopbit2", "[Driver/UART1]")
{
    MS_LOGI( MS_DRIVER, "ms1008_uart_stopbit2\r\n" );

    test_uart1_init_base(UART_PARITY_NONE, UART_STOPBITS_2);

    test_uart1_transmit_data();
}


