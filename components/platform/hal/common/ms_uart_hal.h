/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  ms_hal_uart.h
 * @brief
 * @author bingrui.chen
 * @date 2022年1月4日
 * @version 1.0
 * @Revision:
 */
#ifndef BLE_SOC_XXX_MS_UART_HAL_H_
#define BLE_SOC_XXX_MS_UART_HAL_H_

#include "ms_uart_ll.h"

/**
 * @brief  Get UART Receiver Data Ready State
 */
#define  ms_uart_hal_is_rxne(huart) ms_uart_ll_is_rxne((huart))

/**
 * @brief  Get UART Transmitter Empty State
 */
#define  ms_uart_hal_is_txe(huart) ms_uart_ll_is_txe((huart))

/**
 * @brief  Get UART Overrun Error State
 */
#define  ms_uart_hal_is_oe(huart) ms_uart_ll_is_oe((huart))

/**
 * @brief  Get UART Parity Error State
 */
#define  ms_uart_hal_is_pe(huart) ms_uart_ll_is_pe((huart))

/**
 * @brief  Get UART Framing Error State
 */
#define  ms_uart_hal_is_fe(huart) ms_uart_ll_is_fe((huart))

/**
 * @brief  Get UART Break Interrupt State
 */
#define  ms_uart_hal_is_bi(huart) ms_uart_ll_is_bi((huart))

/**
 * @brief  Get UART Receiver FIFO Error State
 */
#define  ms_uart_hal_is_fifo_err(huart) ms_uart_ll_is_fifo_err((huart))

/**
 * @brief  Get UART Busy State
 */
#define  ms_uart_hal_is_busy(huart) ms_uart_ll_is_busy((huart))

/**
 * @brief  Get UART Transmit FIFO Not Full State
 */
#define  ms_uart_hal_is_tx_fifo_nf(huart) ms_uart_ll_is_tx_fifo_nf((huart))

/**
 * @brief Get UART Transmit FIFO Empty State
 */
#define  ms_uart_hal_is_tx_fifo_empty(huart) ms_uart_ll_is_tx_fifo_empty((huart))

/**
 * @brief  Get UART Receive FIFO Not Empty State
 */
#define  ms_uart_hal_is_rx_fifo_ne(huart) ms_uart_ll_is_rx_fifo_ne((huart))

/**
 * @brief  Get UART Received FIFO Full State
 */
#define  ms_uart_hal_is_rx_fifo_full(huart) ms_uart_ll_is_rx_fifo_full((huart))

/**
 * @brief  UART Reset
 */
#define  ms_uart_hal_reset(huart) ms_uart_ll_reset((huart))

/**
 * @brief  Enable Received Data Available Interrupt.
 */
#define  ms_uart_hal_it_enable_rxne(huart) ms_uart_ll_it_enable_rxne((huart))

/**
 * @brief  Disable Received Data Available Interrupt.
 */
#define  ms_uart_hal_it_disable_rxne(huart) ms_uart_ll_it_disable_rxne((huart))

/**
 * @brief  Enable Received Data Available Interrupt.
 */
#define  ms_uart_hal_enable_prog_tx_empty_it_mod(huart) ms_uart_ll_enable_prog_tx_empty_it_mod((huart))

/**
 * @brief  Disable Received Data Available Interrupt.
 */
#define  ms_uart_hal_disable_prog_tx_empty_it_mod(huart) ms_uart_ll_disable_prog_tx_empty_it_mod((huart))

/**
 * @brief  Enable Transmit Holding Register Empty Interrupt
 */
#define  ms_uart_hal_it_enable_txe(huart) ms_uart_ll_it_enable_txe((huart))

/**
 * @brief  Disable Transmit Holding Register Empty Interrupt
 */
#define  ms_uart_hal_it_disable_txe(huart) ms_uart_ll_it_disable_txe((huart))

/**
 * @brief  Enable Receiver Line Status Interrupt
 */
#define  ms_uart_hal_it_enable_line_status(huart) ms_uart_ll_it_enable_line_status((huart))

/**
 * @brief  Disable Receiver Line Status Interrupt
 */

#define  ms_uart_hal_it_disable_line_status(huart) ms_uart_ll_it_disable_line_status((huart))

/**
 * @brief  Get Interrupt ID
 */
#define  ms_uart_hal_get_it_id(huart) ms_uart_ll_get_it_id((huart))

/**
 * @brief  Get Uart Status
 */
#define  ms_uart_hal_get_status(huart) ms_uart_ll_get_status((huart))

/**
 * @brief  Read Data from Receive Buffer Register
 */
#define  ms_uart_hal_read_rx_data(huart) ms_uart_ll_read_rx_data((huart))

/**
 * @brief  Write data to Transmit Holding Register
 */
#define  ms_uart_hal_write_tx_data(huart,data) ms_uart_ll_write_tx_data((huart),(data))

/**
 * @brief  Enable the transmit (XMIT) and receive (RCVR) FIFOs
 */
#define  ms_uart_hal_fifo_enable(huart) ms_uart_ll_fifo_enable((huart))

/**
 * @brief  Disable the transmit (XMIT) and receive (RCVR) FIFOs
 */
#define  ms_uart_hal_fifo_disable(huart) ms_uart_ll_fifo_disable((huart))

/**
 * @brief  Enable the transmit (XMIT) and receive (RCVR) FIFOs
 */
#define  ms_uart_hal_fifo_is_enable(huart) ms_uart_ll_fifo_is_enable((huart))

/**
 * @brief  Reset the receive FIFO and the receive FIFO is empty
 */
#define  ms_uart_hal_rx_fifo_reset(huart,tx_empty_lvl,rx_trigger_lvl,fifo_enable,dma_mode) ms_uart_ll_rx_fifo_reset((huart),(tx_empty_lvl),(rx_trigger_lvl),(fifo_enable),(dma_mode))

/**
 * @brief  Reset the  transmit FIFO and the  transmit FIFO is empty
 */
#define  ms_uart_hal_tx_fifo_reset(huart) ms_uart_ll_tx_fifo_reset((huart))

/**
 * @brief  Get the number of data entries in the transmit FIFO
 */
#define  ms_uart_hal_tx_fifo_get_byte_num(huart) ms_uart_ll_tx_fifo_get_byte_num((huart))

/**
 * @brief  Get the number of data entries in the receive FIFO
 */
#define  ms_uart_hal_rx_fifo_get_byte_num(huart) ms_uart_ll_rx_fifo_get_byte_num((huart))

/**
 * @brief  Select the trigger level in the receiver FIFO at which the Received Data Available Interrupt will be generated, and
 *         select the empty threshold level at which the THRE Interrupts will be generated when the mode is active.
 */
#define  ms_uart_hal_fifo_or_DMA_set(huart,tx_empty_lvl,rx_trigger_lvl,fifo_enable,dma_mode) ms_uart_ll_fifo_or_DMA_set((huart),(tx_empty_lvl),(rx_trigger_lvl),(fifo_enable),(dma_mode))

/**
 * @brief  Select the empty threshold level at which the THRE Interrupts will be generated
 *         when the mode is active
 */
#define  ms_uart_hal_tx_fifo_set_empty_lvl(huart,tx_empty_lvl) ms_uart_ll_tx_fifo_set_empty_lvl((huart),(tx_empty_lvl))

/**
 * @brief  Select the trigger level in the receiver FIFO at which the Received Data Available
 *         Interrupt will be generated
 */
#define  ms_uart_hal_rx_fifo_set_trigger_lvl(huart,rx_trigger_lvl) ms_uart_ll_rx_fifo_set_trigger_lvl((huart),(rx_trigger_lvl))

/**
 * @brief  Enable Loopback mode
 */
#define  ms_uart_hal_enable_loopback(huart) ms_uart_ll_enable_loopback((huart))

/**
 * @brief  Disable Loopback mode
 */
#define  ms_uart_hal_disable_loopback(huart) ms_uart_ll_disable_loopback((huart))

/**
 * @brief  Set stop bits num
 */
#define  ms_uart_hal_set_stop_bits_num(huart,bits_num) ms_uart_ll_set_stop_bits_num((huart),(bits_num))

/**
 * @brief  Set parity
 */
#define  ms_uart_hal_set_parity(huart,parity) ms_uart_ll_set_parity((huart),(parity))

/**
 * @brief  Enable Auto Flow Control
 * @note  When Auto CTS is enabled (active), the uart transmitter is disabled whenever the cts_n
 *         input becomes inactive (high); this prevents overflowing the FIFO of the receiving UART.
 *         Auto CTS – becomes active when the following occurs:
 *             FIFOs are implemented
 *             AFCE (MCR[5] bit = 1)
 *             FIFOs are enabled through FIFO Control Register FCR[0] bit
 *             SIR mode is disabled (MCR[6] bit = 0)
 * @param[in]  auto_flow_ctrl This parameter can be one of the following values:
 *             AUTO_FLOW_CTRL_DISABLE
 *             AUTO_FLOW_CTRL_ENABLE
 * @retval None
 */
#define  ms_uart_hal_set_auto_flow_ctrl(huart,auto_flow_ctrl) ms_uart_ll_set_auto_flow_ctrl((huart),(auto_flow_ctrl))

/**
 * @brief  This is used to directly control the Request to Send (rts_n) output
 * @note  When Auto RTS Flow Control is not enabled (MCR[5] set to zero), the
 *        rts_n signal is set low by programming MCR[1] (RTS) to a high.
 *        In Auto Flow Control, AFCE_MODE == Enabled and active (MCR[5] set to one)
 *        and FIFO's enable(FCR[0] set to one), the rts_n output is controlled
 *        in the same way, but is also gated with the receiver FIFO threshold trigger
 *        (rts_n is inactive high when above the threshold)
 * @param[in]  auto_flow_ctrl This parameter can be one of the following values:
 *             FLOW_CTRL_RTS_DEASSERT
 *             FLOW_CTRL_RTS_ASSERT
 * @retval None
 */
#define  ms_uart_hal_set_rts(huart,rts) ms_uart_ll_set_rts((huart),(rts))

/**
 * @brief  Set word length
 */
#define  ms_uart_hal_set_word_length(huart,length) ms_uart_ll_set_word_length((huart),(length))

/**
 * @brief  Set baudrate
 */
#define  ms_uart_hal_set_baudrate(huart,baudrate) ms_uart_ll_set_baudrate((huart),(baudrate))

#endif /* BLE_SOC_XXX_MS_UART_HAL_H_ */
