/*
 * uart.c
 *
 *  Created on: 2021å¹?2?ˆ24??
 *      Author: bingrui.chen
 */

#include <ms_clock_hal.h>
#include <ms_pinmux_hal.h>
#include <ms_single_dmac.h>
#include "ms_uart.h"

/* Kernel includes. */
#include "FreeRTOS.h" /* Must come first. */
#include "semphr.h"   /* Semaphore related API prototypes. */

UartHandle_Type uart0_handle;

uint8_t uart0_rx_buffer[256] = { 0 };

static SemaphoreHandle_t uart0_read_lock = NULL;

__weak void uart0_init_dmarx(DmacHandle_Type **phdmarx)
{
    *phdmarx = NULL;
//    DmacHandle_Type *hdmarx;
//    hdmarx = ms_dmac_mgmt_alloc();
//
//    *phdmarx = hdmarx;
//
//    /*initial rx dmac*/
//    hdmarx->init.dst_tr_width = DMAC_XFER_WIDTH_8BITS;
//    hdmarx->init.dst_addr_mode =  DMAC_ADDRESS_MODE_INC;
//    hdmarx->init.dst_peri_type = DMAC_MEM ;
//    hdmarx->init.dst_msize = DMAC_MSIZE_1;
//    hdmarx->init.src_peri_type = DMAC_UART0_RX;
//    hdmarx->init.src_tr_width = DMAC_XFER_WIDTH_8BITS;
//    hdmarx->init.src_addr_mode = DMAC_ADDRESS_MODE_NOC;
//    hdmarx->init.src_msize = DMAC_MSIZE_1;
//    ms_dmac_init(hdmarx);

}

__weak void uart0_init_dmatx(DmacHandle_Type **phdmatx)
{
    //    *phdmatx = NULL;
    DmacHandle_Type *hdmatx;
    hdmatx = ms_dmac_mgmt_alloc();
    *phdmatx = hdmatx;

    hdmatx->init.dst_tr_width = DMAC_XFER_WIDTH_8BITS;
    hdmatx->init.dst_addr_mode = DMAC_ADDRESS_MODE_NOC;
    hdmatx->init.dst_peri_type = DMAC_UART0_TX;
    hdmatx->init.dst_msize = DMAC_MSIZE_1;
    hdmatx->init.src_peri_type = DMAC_MEM;
    hdmatx->init.src_tr_width = DMAC_XFER_WIDTH_8BITS;
    hdmatx->init.src_addr_mode = DMAC_ADDRESS_MODE_INC;
    hdmatx->init.src_msize = DMAC_MSIZE_1;
    ms_dmac_init(hdmatx);
}

void uart0_init_calllback(struct __UartHandle_Type *huart)
{
    /*config the uart clock*/
    MS_CLOCK_HAL_CLK_ENABLE_UART0();
    ms_clock_hal_set_uart0_div(0);
    ms_clock_hal_peripheral_clk_div_toggle(UART0_DIV_TOG);

    /*config the uart pin mux*/
    ms_pinmux_hal_set_pinmux(PAD14,PIN14_UART0_TXD);
    ms_pinmux_hal_set_pinmux(PAD15,PIN15_UART0_RXD);

    /*enable the interrupt*/
    ms_uart_enable_cpu_interrupt(huart);

    uart0_init_dmatx(&huart->hdmatx);
    if (huart->hdmatx)
    {
        huart->hdmatx->parent = huart;
    }

    uart0_init_dmarx(&huart->hdmarx);
    if (huart->hdmarx)
    {
        huart->hdmarx->parent = huart;
    }

    uart0_read_lock = xSemaphoreCreateBinary();
}

void uart0_deinit_calllback(struct __UartHandle_Type *huart)
{
    /*enable the interrupt*/
    ms_uart_disable_cpu_interrupt(huart);

    /*config the uart clock*/
    MS_CLOCK_HAL_CLK_DISABLE_UART0();

    if (huart->hdmarx)
    {
        ms_dmac_mgmt_free(huart->hdmarx);
    }

    if (huart->hdmatx)
    {
        ms_dmac_mgmt_free(huart->hdmatx);
    }

    taskENTER_CRITICAL();
    vSemaphoreDelete(uart0_read_lock);
    uart0_read_lock = NULL;
    taskEXIT_CRITICAL();
}

static void uart0_rx_calllback(struct __UartHandle_Type *huart)
{
    unsigned long task_woken = pdFALSE;

    if (uart0_read_lock != NULL) {
        xSemaphoreGiveFromISR(uart0_read_lock, &task_woken);
        portYIELD_FROM_ISR(task_woken);
    }
}

static void uart0_tx_calllback(struct __UartHandle_Type *huart)
{
    return;
}

UartCallback_Type uart0_callback =
{
    .init_callback = uart0_init_calllback,
    .deinit_callback = uart0_deinit_calllback,
    .rx_cplt_callback = uart0_rx_calllback,
    .tx_cplt_callback = uart0_tx_calllback
};

void uart0_int(void)
{
    uart0_handle.instance = UART0;
    uart0_handle.init.baud_rate = 115200;
    uart0_handle.init.hw_flow_ctl = UART_HWCONTROL_NONE;
    uart0_handle.init.parity = UART_PARITY_NONE;
    uart0_handle.init.stop_bits = UART_STOPBITS_1;
    uart0_handle.init.word_length = UART_WORDLENGTH_8B;
    uart0_handle.init.hw_flow_ctl = UART_HWCONTROL_NONE;
    uart0_handle.init.fifo_enable = 1;
    uart0_handle.init.rx_fifo_level = UART_RX_FIFO_AVAILABLE_TRG_LVL_HALF;
    uart0_handle.init.tx_fifo_level = UART_TX_FIFO_EMPTY_TRG_LVL_EMPTY;
    uart0_handle.hcallback = &uart0_callback;
    uart0_handle.irq = UART0_IRQn;
    ms_uart_init(&uart0_handle);

    ms_uart_start_rx_it(&uart0_handle, uart0_rx_buffer, 256);
}

int32_t uart0_write_dma(char *ptr, int32_t len)
{
    return ms_uart_transmit_dma(&uart0_handle, ptr, len);
}

int32_t uart0_write(char *ptr, int32_t len)
{
    return ms_uart_transmit(&uart0_handle, ptr, len);
}

int32_t uart0_read_noblock(char *ptr, int32_t len)
{
    return ms_uart_receive(&uart0_handle, ptr, len);
}

int32_t uart0_read(char *ptr, int32_t len)
{
    int count;

    if (ptr == NULL || len <= 0) {
        return -1;
    }

    if (uart0_read_lock != NULL) {
        xSemaphoreTake(uart0_read_lock, portMAX_DELAY);
    }

    taskENTER_CRITICAL();
    count = ms_uart_receive(&uart0_handle, ptr, len);
    taskEXIT_CRITICAL();
    return count;
}

int32_t uart0_start_rx_dma(char *ptr, int32_t len)
{
    return ms_uart_start_rx_dma(&uart0_handle, ptr, len);
}

bool uart0_is_rx_dma_done(void)
{
    return ms_dmac_is_chx_xfer_done(uart0_handle.hdmarx);
}

void uart0_clear_rx_dma_int(void)
{
    ms_dmac_clear_all_int(uart0_handle.hdmarx);
}

int32_t uart0_read_dma(char *ptr, int32_t len)
{
    return ms_uart_start_rx_dma(&uart0_handle, ptr, len);
}

int32_t uart0_set_baudrate(uint32_t baudrate)
{
    return ms_uart_set_baudrate(&uart0_handle, baudrate);
}

void UART0_IRQHandler(void)
{
    ms_uart_irq_handler(&uart0_handle);
}

