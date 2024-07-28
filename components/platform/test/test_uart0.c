/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  test_uart0.c
 * @brief
 * @author bingrui.chen
 * @date 2022年1月14日
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

extern void system_clock_init(void);
char hello_str[] = "Hello Uart0\r\n";
char rx_buffer[128] = { 0 };

static uint8_t uart0_rx_done = 0;
static uint8_t uart0_tx_done = 0;

void uart0_init_dmarx(DmacHandle_Type **phdmarx)
{
//    *phdmarx = NULL;
    DmacHandle_Type *hdmarx;
    hdmarx = ms_dmac_mgmt_alloc();

    *phdmarx = hdmarx;

    /*initial rx dmac*/
    hdmarx->init.dst_tr_width = DMAC_XFER_WIDTH_8BITS;
    hdmarx->init.dst_addr_mode = DMAC_ADDRESS_MODE_INC;
    hdmarx->init.dst_peri_type = DMAC_MEM;
    hdmarx->init.dst_msize = DMAC_MSIZE_1;
    hdmarx->init.src_peri_type = DMAC_UART0_RX;
    hdmarx->init.src_tr_width = DMAC_XFER_WIDTH_8BITS;
    hdmarx->init.src_addr_mode = DMAC_ADDRESS_MODE_NOC;
    hdmarx->init.src_msize = DMAC_MSIZE_1;
    ms_dmac_init(hdmarx);

}

#if 0
static void uart0_rx_calllback(struct __UartHandle_Type *huart)
{
    uart0_rx_done = 1;
}

static void uart0_tx_calllback(struct __UartHandle_Type *huart)
{
    uart0_tx_done = 1;
}
#endif

void test_uart0_rx(void)
{
    int32_t len = 0;
    while (1)
    {
        if ((len = uart0_read_noblock(rx_buffer, 128)) > 0)
        {
            uart0_write(rx_buffer, 16);
        }
    }
}

void test_uart0_rx_dma(void)
{
    int32_t len = 0;
    while (1)
    {
        uint32_t count = 0;
        uart0_start_rx_dma(rx_buffer, 16);

        while ((!uart0_rx_done) && (count < 50000))
        {
            count++;
        }

        if (uart0_rx_done)
        {
            uart0_write(rx_buffer, 16);
        }
    }
}

#if 0
int main(void)
{
    system_clock_init();
    ms_pinmux_hal_config_default();
    ms_dmac_mgmt_inti();
    uart0_int();
    uart0_write(hello_str, strlen(hello_str));

    test_uart0_rx_dma();

//    UNITY_BEGIN();
//    RUN_TEST(test_uart1_loopback);
//    RUN_TEST(test_uart1_loopback_rx_dma);
////    RUN_TEST(test_uart1_loopback_rx_dma);
////    RUN_TEST(test_uart1_loopback_rx_tx_dma);
//    return UNITY_END();
    while (1)
        ;
    return 0;
}

#endif
