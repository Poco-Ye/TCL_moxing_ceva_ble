/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_uart.c
 * @brief
 * @author bingrui.chen
 * @date 2021-12-22
 * @version 1.0
 * @Revision
 */
#include "ms_uart.h"
#include <stddef.h>

/**
 * @brief This function Configures UART Controller Parameters.
 * @param  huart  Pointer to a UART_Handle_Type structure that contains
 *                the configuration information for the specified UART module.
 * @retval none
 */
static int32_t  ms_uart_set_config(UartHandle_Type *huart)
{
    CHECK_PTR_NULL_RET(huart,STATUS_ERROR);

    /* Configure the UART Stop Bits */
    ms_uart_hal_set_stop_bits_num(huart->instance, huart->init.stop_bits);

    /* Configure the UART Word Length, Parity and mode*/
    ms_uart_hal_set_parity(huart->instance, huart->init.parity);
    ms_uart_hal_set_word_length(huart->instance, huart->init.word_length);

    /* Configure the UART HFC: Set CTSE and RTSE bits according to huart->Init.HwFlowCtl value */
    switch (huart->init.hw_flow_ctl)
    {
    case UART_HWCONTROL_NONE:
        ms_uart_hal_set_auto_flow_ctrl(huart->instance, AUTO_FLOW_CTRL_DISABLE);
        ms_uart_hal_set_rts(huart->instance, FLOW_CTRL_RTS_DEASSERT);
        break;
    case UART_HWCONTROL_AUTO_CTS:
        ms_uart_hal_set_auto_flow_ctrl(huart->instance, AUTO_FLOW_CTRL_ENABLE);
        break;
    case UART_HWCONTROL_AUTO_RTS_CTS:
        ms_uart_hal_set_auto_flow_ctrl(huart->instance, AUTO_FLOW_CTRL_ENABLE);
        ms_uart_hal_set_rts(huart->instance, FLOW_CTRL_RTS_ASSERT);
        break;
    }

    /* Set USART BRR Configuration */
    return ms_uart_hal_set_baudrate(huart->instance, huart->init.baud_rate);
}

/**
 * @brief This function Initials UART Controller Base Parameters.
 * @param  huart  Pointer to a UART_Handle_Type structure that contains
 *                the configuration information for the specified UART module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
static int32_t ms_uart_base_init(UartHandle_Type *huart)
{
    CHECK_PTR_NULL_RET(huart, STATUS_ERROR);

    /*initial uart gpio pinmux, clock and interrupt setting*/
    if (huart->hcallback && huart->hcallback->init_callback)
    {
        huart->hcallback->init_callback(huart);
    }

    /*Disable the peripheral */
    while (ms_uart_hal_is_busy(huart->instance))
        ;

    /*Set the UART Communication parameters */

    return ms_uart_set_config(huart);
}

static void ms_uart_dma_tx_xfer_tfr_callback(DmacHandle_Type *hdma)
{
    CHECK_PTR_NULL(hdma);

    UartHandle_Type *huart = hdma->parent;
    if (huart->hcallback == NULL || huart->hcallback->tx_cplt_callback == NULL)
    {
        huart->error_code |= UART_ERROR_INVALID_CALLBACK;
        return;
    }

    huart->hcallback->tx_cplt_callback(huart);
}

static void ms_uart_dma_rx_xfer_tfr_callback(DmacHandle_Type *hdma)
{
    CHECK_PTR_NULL(hdma);

    UartHandle_Type *huart = hdma->parent;
    if (huart->hcallback == NULL || huart->hcallback->rx_cplt_callback == NULL)
    {
        huart->error_code |= UART_ERROR_INVALID_CALLBACK;
        return;
    }

    huart->hcallback->rx_cplt_callback(huart);
}

static void ms_uart_dma_tx_xfer_error_callback(DmacHandle_Type *hdma)
{
    CHECK_PTR_NULL(hdma);

    UartHandle_Type *huart = hdma->parent;
    if (huart->hcallback == NULL || huart->hcallback->tx_cplt_callback == NULL)
    {
        huart->error_code |= UART_ERROR_INVALID_CALLBACK;
        return;
    }

    huart->hcallback->tx_cplt_callback(huart);
}

static void ms_uart_dma_rx_xfer_error_callback(DmacHandle_Type *hdma)
{
    CHECK_PTR_NULL(hdma);

    UartHandle_Type *huart = hdma->parent;
    if (huart->hcallback == NULL || huart->hcallback->rx_cplt_callback == NULL)
    {
        huart->error_code |= UART_ERROR_INVALID_CALLBACK;
        return;
    }

    huart->hcallback->rx_cplt_callback(huart);
}

/**
 * @brief  This function handles UART Transmit fifo empty interrupt request.
 * @param  huart  Pointer to a UART_Handle_Type structure that contains
 *                the configuration information for the specified UART module.
 * @retval none
 */
static void __uart_tx_it(UartHandle_Type *huart)
{
    huart->error_code = UART_ERROR_NONE;
    if (huart->tx_buff_ptr == NULL)
    {
        return;
    }

    if (huart->tx_xfer_size == 0)
    {
        return;
    }

    ms_uart_hal_write_tx_data(huart->instance, (uint16_t ) huart->tx_buff_ptr[huart->tx_xfer_cnt++]);

    if (huart->tx_xfer_size != huart->tx_xfer_cnt)
    {
        return;
    }

    if (huart->hcallback == NULL || huart->hcallback->tx_cplt_callback == NULL)
    {
        huart->error_code |= UART_ERROR_INVALID_CALLBACK;
        return;
    }

    ms_uart_hal_it_disable_txe(huart->instance);

    huart->hcallback->tx_cplt_callback(huart);
}

extern int32_t uart0_write(char *ptr, int32_t len);

/**
 * @brief This function handles UART received data available interrupt request.
 * @param  huart  Pointer to a UART_Handle_Type structure that contains
 *                the configuration information for the specified UART module.
 * @retval none
 */

static void __uart_rx_it(UartHandle_Type *huart)
{
    huart->error_code = UART_ERROR_NONE;
    while (ms_uart_hal_is_rxne(huart->instance))
    {
        ringbuffer_push_byte(&huart->ringbuffer, (uint8_t) ms_uart_hal_read_rx_data(huart->instance));
        huart->rx_xfer_cnt++;
		//uart0_write("+", 1);
    }

    if (huart->hcallback == NULL || huart->hcallback->rx_cplt_callback == NULL)
    {
        huart->error_code |= UART_ERROR_INVALID_CALLBACK;
        return;
    }

    if (ringbuffer_get_count(&huart->ringbuffer))
    {
        huart->hcallback->rx_cplt_callback(huart);
    }
}

/**
 * @brief  This function handles UART line status interrupt request.
 * @param  huart  Pointer to a UART_Handle_Type structure that contains
 *                the configuration information for the specified UART module.
 * @retval none
 */
static void __uart_receive_line_status_it(UartHandle_Type *huart)
{
    huart->error_code = UART_ERROR_NONE;
    if (ms_uart_hal_is_oe(huart->instance))
    {
        huart->error_code |= UART_ERROR_ORE;
    }

    if (ms_uart_hal_is_fe(huart->instance))
    {
        huart->error_code |= UART_ERROR_FE;
    }

    if (ms_uart_hal_is_pe(huart->instance))
    {
        huart->error_code |= UART_ERROR_PE;
    }

    if (ms_uart_hal_is_bi(huart->instance))
    {
        huart->error_code |= UART_ERROR_BI;
    }
}


void ms_uart_enable_cpu_interrupt(UartHandle_Type *huart)
{
    INTERRUPT_ENABLE_IRQ(huart->irq);
}

void ms_uart_disable_cpu_interrupt(UartHandle_Type *huart)
{
    INTERRUPT_DISABLE_IRQ(huart->irq);
}

int32_t ms_uart_init(UartHandle_Type *huart)
{
    CHECK_PTR_NULL_RET(huart, STATUS_ERROR);

	if(ms_uart_base_init(huart) == STATUS_ERROR)
	{
        return STATUS_ERROR;
	}

    /*Enable the peripheral */

    if (huart->init.fifo_enable)
    {
        ms_uart_hal_fifo_or_DMA_set(huart->instance, huart->init.tx_fifo_level, huart->init.rx_fifo_level, huart->init.fifo_enable, 0);
    }
    else
    {
        ms_uart_hal_fifo_disable(huart->instance);
    }

	//ms_uart_hal_it_enable_line_status(huart->instance);
    return STATUS_SUCCESS;
}

int32_t ms_uart_set_tx_fifo_empty_lvl(UartHandle_Type *huart, uint32_t empty_lvl)
{
    CHECK_PTR_NULL_RET(huart, STATUS_ERROR);

    ms_uart_hal_tx_fifo_set_empty_lvl(huart->instance, empty_lvl);
    return STATUS_SUCCESS;
}

int32_t ms_uart_set_rx_fifo_full_lvl(UartHandle_Type *huart, uint32_t trig_lvl)
{
    CHECK_PTR_NULL_RET(huart, STATUS_ERROR);

    ms_uart_hal_rx_fifo_set_trigger_lvl(huart->instance, trig_lvl);
    return STATUS_SUCCESS;
}

int32_t ms_uart_set_baudrate(UartHandle_Type *huart, uint32_t baudrate)
{
    CHECK_PTR_NULL_RET(huart, STATUS_ERROR);

    huart->init.baud_rate = baudrate;
    return ms_uart_base_init(huart);
}

/**
 * @brief  Transmit an amount of data in non blocking mode
 * @param  huart  Pointer to a UART_Handle_Type structure that contains
 *                the configuration information for the specified UART module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_uart_deinit(UartHandle_Type *huart)
{
    CHECK_PTR_NULL_RET(huart, STATUS_ERROR);

    /*Disable the peripheral */

    /* DeInit the low level hardware */
    if (huart->hcallback && huart->hcallback->deinit_callback)
    {
        huart->hcallback->deinit_callback(huart);
    }

    return STATUS_SUCCESS;
}

/**
 * @brief  Transmit an amount of data in dma mode
 * @param  huart  Pointer to a UART_Handle_Type structure that contains
 *                the configuration information for the specified UART module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_uart_transmit_dma(UartHandle_Type *huart, uint8_t *p_data, uint16_t size)
{
    CHECK_PTR_NULL_RET(huart, STATUS_ERROR);
    CHECK_PTR_NULL_RET(huart->hdmatx, STATUS_ERROR);
    CHECK_PTR_NULL_RET(p_data, STATUS_ERROR);

    /*Wait for dmac channel to be idle*/
    while (ms_dmac_hal_is_enable_channel(huart->hdmatx->dma_instance, huart->hdmatx->channel_index))
        ;

    huart->tx_buff_ptr = p_data;
    huart->tx_xfer_size = size;
    huart->tx_xfer_cnt = size;
    huart->hdmatx->xfer_tfr_callback = ms_uart_dma_tx_xfer_tfr_callback;
    huart->hdmatx->xfer_error_callback = ms_uart_dma_tx_xfer_error_callback;
    uint32_t *tmp = (uint32_t*) &p_data;
    ms_dmac_start_chx_xfer_it(huart->hdmatx, *(uint32_t*) tmp, (uint32_t) &huart->instance->OFFSET_0.THR, size);

    return STATUS_SUCCESS;
}

int32_t ms_uart_transmit(UartHandle_Type *huart, uint8_t *p_data, uint16_t size)
{
    CHECK_PTR_NULL_RET(huart, STATUS_ERROR);
    CHECK_PTR_NULL_RET(p_data, STATUS_ERROR);

    for (uint16_t i = 0; i < size; i++)
    {
        while (!ms_uart_hal_is_txe(huart->instance))
            ;
        ms_uart_hal_write_tx_data(huart->instance, p_data[i]);
    }
    while (!ms_uart_hal_is_txe(huart->instance))
        ;

    return STATUS_SUCCESS;
}

int32_t ms_uart_transmit_it(UartHandle_Type *huart, uint8_t *p_data, uint16_t size)
{
    CHECK_PTR_NULL_RET(huart, STATUS_ERROR);
    CHECK_PTR_NULL_RET(p_data, STATUS_ERROR);

    huart->tx_buff_ptr = p_data;
    huart->tx_xfer_size = size;
    huart->tx_xfer_cnt = 0;

    ms_uart_hal_it_enable_txe(huart->instance);

    return STATUS_SUCCESS;
}

int32_t ms_uart_start_rx_dma(UartHandle_Type *huart, uint8_t *buffer_ptr, uint32_t size)
{
    CHECK_PTR_NULL_RET(huart, STATUS_ERROR);
    CHECK_PTR_NULL_RET(huart->hdmarx, STATUS_ERROR);
    CHECK_PTR_NULL_RET(buffer_ptr, STATUS_ERROR);

    ms_uart_hal_it_disable_rxne(huart->instance);
    ms_uart_hal_rx_fifo_reset(huart->instance,huart->init.tx_fifo_level,huart->init.rx_fifo_level,huart->init.fifo_enable,0);
    huart->rx_buff_ptr = buffer_ptr;
    huart->rx_xfer_size = size;
    huart->rx_xfer_cnt = 0;
    huart->hdmarx->xfer_tfr_callback = ms_uart_dma_rx_xfer_tfr_callback;
    huart->hdmarx->xfer_error_callback = ms_uart_dma_rx_xfer_error_callback;
    uint32_t *tmp = (uint32_t*) &buffer_ptr;
    ms_dmac_start_chx_xfer_it(huart->hdmarx, (uint32_t) &huart->instance->OFFSET_0.RBR, *(uint32_t*) tmp, size);

    return STATUS_SUCCESS;
}

int32_t ms_uart_stop_rx_it(UartHandle_Type *huart)
{
    CHECK_PTR_NULL_RET(huart, STATUS_ERROR);

    ms_uart_hal_it_disable_rxne(huart->instance);
    ms_uart_hal_rx_fifo_reset(huart->instance,huart->init.tx_fifo_level,huart->init.rx_fifo_level,huart->init.fifo_enable,0);

    return STATUS_SUCCESS;
}

int32_t ms_uart_start_rx_it(UartHandle_Type *huart, uint8_t *p_ringbuffer, uint32_t len)
{
    CHECK_PTR_NULL_RET(huart, STATUS_ERROR);
    CHECK_PTR_NULL_RET(p_ringbuffer, STATUS_ERROR);

    ms_uart_hal_it_disable_rxne(huart->instance);
    ms_uart_hal_rx_fifo_reset(huart->instance, huart->init.tx_fifo_level,  huart->init.tx_fifo_level,  huart->init.fifo_enable, 0);
    huart->rx_xfer_size = 0;
    huart->rx_xfer_cnt = 0;
    ringbuffer_initial(&huart->ringbuffer, p_ringbuffer, len);
    ms_uart_hal_it_enable_rxne(huart->instance);
    return STATUS_SUCCESS;
}

int32_t ms_uart_receive(UartHandle_Type *huart, uint8_t *p_buffer, int32_t len)
{
    CHECK_PTR_NULL_RET(huart, STATUS_ERROR);
    CHECK_PTR_NULL_RET(p_buffer, STATUS_ERROR);

    int32_t count = ringbuffer_get_count(&huart->ringbuffer);
    if (count == 0)
    {
        return 0;
    }

    count = len > count ? count : len;
    ringbuffer_pop_data(&huart->ringbuffer, p_buffer, count, 0, 0);
    return count;
}

void ms_uart_irq_handler(UartHandle_Type *huart)
{
    CHECK_PTR_NULL(huart);

    /*todo: disable uart interrupt*/
	ECLIC_DisableIRQ(huart->irq);

    /*proccess the interrupt source according the iid*/
    uint32_t int_id = ms_uart_hal_get_it_id(huart->instance);
    switch (int_id & 0x0F)
    {
    case UART_IT_ID_MODEM_STATUS:
        /* todo:Clear to send or data set ready or ring indicator or data carrier detect. Note that if
         * auto flow control mode is enabled, a change in CTS (that is, DCTS set) does not cause an interrupt.
         * */
        break;

    case UART_IT_ID_NO_INTERRUPT_PENDING:
        /*nothing to do*/
        break;

    case UART_IT_ID_THR_EMPTY:

        __uart_tx_it(huart);

        break;

    case UART_IT_ID_RECEIVED_DATA_AVAILABLE:

        __uart_rx_it(huart);
        break;

    case UART_IT_ID_RECEIVER_LINE_STATUS:
        /*Overrun/parity/ framing errors, break interrupt, or address received interrupt
         * Reading the line status register or the Receiver line status is also cleared when RX_FIFO is read
         * */
        __uart_receive_line_status_it(huart);
        break;

    case UART_IT_ID_BUSY_DETECT:
    {
        uint32_t status = ms_uart_hal_get_status(huart->instance);
    }
        break;

    case UART_IT_ID_CHARACTER_TIMEOUT:
        /* No characters in or out of the RCVR FIFO during the last 4 character times
         * and there is at least 1 character in it during this time.
         * */
        break;
    default:
        break;
    }

    ECLIC_EnableIRQ(huart->irq);
}
