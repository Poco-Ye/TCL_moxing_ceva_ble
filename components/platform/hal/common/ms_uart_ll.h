/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_uart_ll.h
 * @brief Header file of Uart  module.
 * @author bingrui.chen
 * @date   2021-12-21
 * @version 1.0
 * @Revision:
 */
#ifndef MS_LL_UART_H_
#define MS_LL_UART_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>

/** @addtogroup MS_LL_Driver
 * @{
 */

/** @defgroup UART_LL UART
 * @{
 */

/** @defgroup UART_Baudrate UART Baudrate definition
 * @{
 */
#define UART_BAUDRATE_110                       (110)
#define UART_BAUDRATE_300                       (300)
#define UART_BAUDRATE_600                       (600)
#define UART_BAUDRATE_1200                      (1200)
#define UART_BAUDRATE_2400                      (2400)
#define UART_BAUDRATE_4800                      (4800)
#define UART_BAUDRATE_9600                      (9600)
#define UART_BAUDRATE_14400                     (14400)
#define UART_BAUDRATE_19200                     (19200)
#define UART_BAUDRATE_38400                     (38400)
#define UART_BAUDRATE_57600                     (57600)
#define UART_BAUDRATE_115200                    (115200)
#define UART_BAUDRATE_230400                    (230400)
#define UART_BAUDRATE_460800                    (460800)
#define UART_BAUDRATE_921600                    (921600)
/** @}*/

/** @defgroup UART_Word_Length UART Word Length definition
 * @{
 */
#define UART_WORDLENGTH_5B                  0x00000000U
#define UART_WORDLENGTH_6B                  0x00000001U
#define UART_WORDLENGTH_7B                  0x00000002U
#define UART_WORDLENGTH_8B                  0x00000003U
#define UART_WORDLENGTH_9B                  0x00000004U
/** @}
 */

/** @defgroup UART_Stop_Bits_Number UART Stop Bits Number definition
 * @{
 */
#define UART_STOPBITS_1                     0x00000000U
#define UART_STOPBITS_2                     ((uint32_t)UART_LCR_STOP)
/**
 * @}
 */

/** @defgroup UART_Parity UART Parity definition
 * @{
 */
#define UART_PARITY_NONE                    0x00000000U
#define UART_PARITY_ODD                     0x00000001U
#define UART_PARITY_EVEN                    0x00000003U
/**
 * @}
 */

/** @defgroup UART_Hardware_Flow_Control UART Hardware Flow Control definition
 * @{
 */
#define AUTO_FLOW_CTRL_DISABLE                0x00000000U
#define AUTO_FLOW_CTRL_ENABLE                 0x00000001U

#define FLOW_CTRL_RTS_DEASSERT                0x00000000U
#define FLOW_CTRL_RTS_ASSERT                  0x00000001U
/**
 * @}
 */

/** @defgroup UART_Loopback_Enable UART Loopback Option definition
 * @{
 */
#define UART_LOOPBACK_DISABLE                  0x00000000U
#define UART_LOOPBACK_ENABLE                   ((uint32_t)UART_MCR_LoopBack)

/**
 ** @}
 */

/** @defgroup UART_RX_FIFO_AVAILABLE_TRG_LVL  UART Rx FIFO Available Interrupt trigger level definition
 * @{
 */
#define UART_RX_FIFO_AVAILABLE_TRG_LVL_1_CHAR               0x00000000U
#define UART_RX_FIFO_AVAILABLE_TRG_LVL_QUARTER              0x00000001U
#define UART_RX_FIFO_AVAILABLE_TRG_LVL_HALF                 0x00000002U
#define UART_RX_FIFO_AVAILABLE_TRG_LVL_FULL_2               0x00000003U

/**
 *@}
 */

/** @defgroup UART_TX_FIFO_EMPTY_TRG_LVL  UART TX FIFO Empty Trigger definition
 *
 ** @{
 */

#define UART_TX_FIFO_EMPTY_TRG_LVL_EMPTY                    0x00000000U
#define UART_TX_FIFO_EMPTY_TRG_LVL_2_CHAR                   0x00000001U
#define UART_TX_FIFO_EMPTY_TRG_LVL_QUARTER                  0x00000002U
#define UART_TX_FIFO_EMPTY_TRG_LVL_HALF                     0x00000003U

/**
 * @}
 */

/** @defgroup UART_Interrupt_ID UART Interrupt ID definition
 * @{
 */

#define UART_IT_ID_MODEM_STATUS                            0x00000000U
#define UART_IT_ID_NO_INTERRUPT_PENDING                    0x00000001U
#define UART_IT_ID_THR_EMPTY                               0x00000002U
#define UART_IT_ID_RECEIVED_DATA_AVAILABLE                 0x00000004U
#define UART_IT_ID_RECEIVER_LINE_STATUS                    0x00000006U
#define UART_IT_ID_BUSY_DETECT                             0x00000007U
#define UART_IT_ID_CHARACTER_TIMEOUT                       0x0000000CU

/**
 * @}
 */

/**
 * @brief  Get UART Receiver Data Ready State
 * @retval true or false
 */
static inline bool ms_uart_ll_is_rxne(Uart_Type *huart)
{
    return (READ_BIT(huart->LSR,UART_LSR_DR) == UART_LSR_DR);
}

/**
 * @brief  Get UART Transmitter Empty State
 * @retval true or false
 */
static inline bool ms_uart_ll_is_txe(Uart_Type *huart)
{
    return (READ_BIT(huart->LSR,UART_LSR_TEMT) == UART_LSR_TEMT);
}

/**
 * @brief  Get UART Overrun Error State
 * @retval true or false
 */
static inline bool ms_uart_ll_is_oe(Uart_Type *huart)
{
    return (READ_BIT(huart->LSR,UART_LSR_OE) == UART_LSR_OE);
}

/**
 * @brief  Get UART Parity Error State
 * @retval true or false
 */
static inline bool ms_uart_ll_is_pe(Uart_Type *huart)
{
    return (READ_BIT(huart->LSR,UART_LSR_PE) == UART_LSR_PE);
}

/**
 * @brief  Get UART Framing Error State
 * @retval true or false
 */
static inline bool ms_uart_ll_is_fe(Uart_Type *huart)
{
    return (READ_BIT(huart->LSR,UART_LSR_FE) == UART_LSR_FE);
}

/**
 * @brief  Get UART Break Interrupt State
 * @retval true or false
 */
static inline bool ms_uart_ll_is_bi(Uart_Type *huart)
{
    return (READ_BIT(huart->LSR,UART_LSR_BI) == UART_LSR_BI);
}

/**
 * @brief  Get UART Receiver FIFO Error State
 * @retval true or false
 */
static inline bool ms_uart_ll_is_fifo_err(Uart_Type *huart)
{
    return (READ_BIT(huart->LSR,UART_LSR_RFE) == UART_LSR_RFE);
}

/**
 * @brief  Get UART Busy State
 * @retval true or false
 */
static inline bool ms_uart_ll_is_busy(Uart_Type *huart)
{
    return (READ_BIT(huart->USR,UART_USR_BUSY) == UART_USR_BUSY);
}

/**
 * @brief  Get UART Transmit FIFO Not Full State
 * @retval true or false
 */
static inline bool ms_uart_ll_is_tx_fifo_nf(Uart_Type *huart)
{
    return (READ_BIT(huart->USR,UART_USR_TFNF) == UART_USR_TFNF);
}

/**
 * @brief  Get UART Transmit FIFO Empty State
 * @retval true or false
 */
static inline bool ms_uart_ll_is_tx_fifo_empty(Uart_Type *huart)
{
    return (READ_BIT(huart->USR,UART_USR_TFE) == UART_USR_TFE);
}

/**
 * @brief  Get UART Receive FIFO Not Empty State
 * @retval true or false
 */
static inline bool ms_uart_ll_is_rx_fifo_ne(Uart_Type *huart)
{
    return (READ_BIT(huart->USR,UART_USR_RFNE) == UART_USR_RFNE);
}

/**
 * @brief  Get UART Received FIFO Full State
 * @retval true or false
 */
static inline bool ms_uart_ll_is_rx_fifo_full(Uart_Type *huart)
{
    return (READ_BIT(huart->USR,UART_USR_RFF) == UART_USR_RFF);
}

/**
 * @brief  UART Reset
 * @retval None
 */
static inline void ms_uart_ll_reset(Uart_Type *huart)
{
    SET_BIT(huart->SRR, UART_SRR_UR);
}

/**
 * @brief  Enable Received Data Available Interrupt.
 * @retval None
 */
static inline void ms_uart_ll_it_enable_rxne(Uart_Type *huart)
{
    SET_BIT(huart->OFFSET_4.IER, UART_IER_ERBFI);
}

/**
 * @brief  Disable Received Data Available Interrupt.
 * @retval None
 */
static inline void ms_uart_ll_it_disable_rxne(Uart_Type *huart)
{
    CLEAR_BIT(huart->OFFSET_4.IER, UART_IER_ERBFI);
}

/**
 * @brief  Enable Received Data Available Interrupt.
 * @retval None
 */
static inline void ms_uart_ll_enable_prog_tx_empty_it_mod(Uart_Type *huart)
{
    SET_BIT(huart->OFFSET_4.IER, UART_IER_PTIME);
}

/**
 * @brief  Disable Received Data Available Interrupt.
 * @retval None
 */
static inline void ms_uart_ll_disable_prog_tx_empty_it_mod(Uart_Type *huart)
{
    CLEAR_BIT(huart->OFFSET_4.IER, UART_IER_PTIME);
}

/**
 * @brief  Enable Transmit Holding Register Empty Interrupt
 * @retval None
 */
static inline void ms_uart_ll_it_enable_txe(Uart_Type *huart)
{
    SET_BIT(huart->OFFSET_4.IER, UART_IER_ETBEI);
}

/**
 * @brief  Disable Transmit Holding Register Empty Interrupt
 * @retval None
 */
static inline void ms_uart_ll_it_disable_txe(Uart_Type *huart)
{
    CLEAR_BIT(huart->OFFSET_4.IER, UART_IER_ETBEI);
}

/**
 * @brief  Enable Receiver Line Status Interrupt
 * @retval None
 */
static inline void ms_uart_ll_it_enable_line_status(Uart_Type *huart)
{
    SET_BIT(huart->OFFSET_4.IER, UART_IER_ELSI);
}

/**
 * @brief  Disable Receiver Line Status Interrupt
 * @retval None
 */
static inline void ms_uart_ll_it_disable_line_status(Uart_Type *huart)
{
    CLEAR_BIT(huart->OFFSET_4.IER, UART_IER_ELSI);
}

/**
 * @brief  Get Interrupt ID
 * @retval None
 */
static inline uint32_t ms_uart_ll_get_it_id(Uart_Type *huart)
{
    return (READ_REG(huart->OFFSET_8.IIR) & UART_IIR_IID_MASK);
}

/**
 * @brief  Get Uart Status
 * @retval None
 */
static inline uint32_t ms_uart_ll_get_status(Uart_Type *huart)
{
    return READ_REG(huart->USR);
}

/**
 * @brief  Read Data from Receive Buffer Register
 * @retval None
 */
static inline uint16_t ms_uart_ll_read_rx_data(Uart_Type *huart)
{
    return (uint16_t) READ_REG(huart->OFFSET_0.RBR);
}

/**
 * @brief  Write data to Transmit Holding Register
 * @retval None
 */
static inline void ms_uart_ll_write_tx_data(Uart_Type *huart, uint16_t data)
{
    WRITE_REG(huart->OFFSET_0.THR, data);
}

/**
 * @brief  Enable the transmit (XMIT) and receive (RCVR) FIFOs
 * @retval None
 */
static inline void ms_uart_ll_fifo_enable(Uart_Type *huart)
{
    if (huart == NULL)
    {
        return;
    }

    uint32_t dma_mode = READ_REG(huart->SDMAM);
    uint32_t tx_fifo_trgg_lvl = READ_REG(huart->STET);
    uint32_t rx_fifo_trgg_lvl = READ_REG(huart->SRT);
    uint32_t fifo_enable = 1;
    uint32_t value = 0;
    value = (fifo_enable << UART_FCR_FIFOE_POS) | (dma_mode << UART_FCR_DMAM_POS)
            | (tx_fifo_trgg_lvl << UART_FCR_TET_POS) | (rx_fifo_trgg_lvl << UART_FCR_RT_POS);
    WRITE_REG(huart->OFFSET_8.FCR, value);
}

/**
 * @brief  Disable the transmit (XMIT) and receive (RCVR) FIFOs
 * @retval None
 */
static inline void ms_uart_ll_fifo_disable(Uart_Type *huart)
{
    if (huart == NULL)
    {
        return;
    }

    uint32_t fifo_enable = 0;
    uint32_t value = 0;
    value = (fifo_enable << UART_FCR_FIFOE_POS);
    WRITE_REG(huart->OFFSET_8.FCR, value);
}

/**
 * @brief  Enable the transmit (XMIT) and receive (RCVR) FIFOs
 * @retval None
 */
static inline bool ms_uart_ll_fifo_is_enable(Uart_Type *huart)
{
    return (READ_BIT(huart->OFFSET_8.IIR,UART_IIR_FIFOSE) == UART_IIR_FIFOSE);
}

/**
 * @brief  Reset the receive FIFO and the receive FIFO is empty
 * @retval None
 */
static inline void ms_uart_ll_rx_fifo_reset(Uart_Type *huart, uint32_t tx_fifo_trgg_lvl, uint32_t rx_fifo_trgg_lvl, uint32_t fifo_enable, uint32_t dma_mode)
{
    uint32_t value = 0;

    value = (fifo_enable << UART_FCR_FIFOE_POS) | (dma_mode << UART_FCR_DMAM_POS)
            | (tx_fifo_trgg_lvl << UART_FCR_TET_POS) | (rx_fifo_trgg_lvl << UART_FCR_RT_POS) | UART_FCR_RFIFOR;
    WRITE_REG(huart->OFFSET_8.FCR, value);
}

/**
 * @brief  Reset the  transmit FIFO and the  transmit FIFO is empty
 * @retval None
 */
static inline void ms_uart_ll_tx_fifo_reset(Uart_Type *huart)
{
    uint32_t dma_mode = READ_REG(huart->SDMAM);
    uint32_t tx_fifo_trgg_lvl = READ_REG(huart->STET);
    uint32_t rx_fifo_trgg_lvl = READ_REG(huart->SRT);
    uint32_t fifo_enable = READ_REG(huart->SFE);

    uint32_t value = 0;
    value = (fifo_enable << UART_FCR_FIFOE_POS) | (dma_mode << UART_FCR_DMAM_POS)
            | (tx_fifo_trgg_lvl << UART_FCR_TET_POS) | (rx_fifo_trgg_lvl << UART_FCR_RT_POS) | UART_FCR_XFIFOR;
    WRITE_REG(huart->OFFSET_8.FCR, value);
}

/**
 * @brief  Get the number of data entries in the transmit FIFO
 * @retval None
 */
static inline uint32_t ms_uart_ll_tx_fifo_get_byte_num(Uart_Type *huart)
{
    return READ_REG(huart->TFL);
}

/**
 * @brief  Get the number of data entries in the receive FIFO
 * @retval None
 */
static inline uint32_t ms_uart_ll_rx_fifo_get_byte_num(Uart_Type *huart)
{
    return READ_REG(huart->RFL);
}

/**
 * @brief  Select the trigger level in the receiver FIFO at which the Received Data Available Interrupt will be generated, and
 *         select the empty threshold level at which the THRE Interrupts will be generated when the mode is active. 
 *         Set FIFO enable bit,and set DMA MODE.
 * @retval None
 */
static inline void ms_uart_ll_fifo_or_DMA_set(Uart_Type *huart, uint32_t tx_empty_lvl, uint32_t rx_trigger_lvl, uint32_t fifo_enable, uint32_t dma_mode)
{
    uint32_t value = 0;

    value = (fifo_enable << UART_FCR_FIFOE_POS) | (dma_mode << UART_FCR_DMAM_POS) | (tx_empty_lvl << UART_FCR_TET_POS)
            | (rx_trigger_lvl << UART_FCR_RT_POS);
    WRITE_REG(huart->OFFSET_8.FCR, value);
}

/**
 * @brief  Select the empty threshold level at which the THRE Interrupts will be generated
 *         when the mode is active
 * @retval None
 */
static inline void ms_uart_ll_tx_fifo_set_empty_lvl(Uart_Type *huart, uint32_t tx_empty_lvl)
{
    uint32_t dma_mode = READ_REG(huart->SDMAM);
 //   uint32_t tx_fifo_trgg_lvl = READ_REG(huart->STET);   //unsed var
    uint32_t rx_fifo_trgg_lvl = READ_REG(huart->SRT);    
    uint32_t fifo_enable = READ_REG(huart->SFE);

    uint32_t value = 0;
    value = (fifo_enable << UART_FCR_FIFOE_POS) | (dma_mode << UART_FCR_DMAM_POS) | (tx_empty_lvl << UART_FCR_TET_POS)
            | (rx_fifo_trgg_lvl << UART_FCR_RT_POS);
    WRITE_REG(huart->OFFSET_8.FCR, value);
}

/**
 * @brief  Select the trigger level in the receiver FIFO at which the Received Data Available
 *         Interrupt will be generated
 * @retval None
 */
static inline void ms_uart_ll_rx_fifo_set_trigger_lvl(Uart_Type *huart, uint32_t rx_trigger_lvl)
{
    uint32_t dma_mode = READ_REG(huart->SDMAM);
    uint32_t tx_fifo_trgg_lvl = READ_REG(huart->STET);
   // uint32_t rx_fifo_trgg_lvl = READ_REG(huart->SRT);   //unsed var
    uint32_t fifo_enable = READ_REG(huart->SFE);

    uint32_t value = 0;
    value = (fifo_enable << UART_FCR_FIFOE_POS) | (dma_mode << UART_FCR_DMAM_POS)
            | (tx_fifo_trgg_lvl << UART_FCR_TET_POS) | (rx_trigger_lvl << UART_FCR_RT_POS);
    WRITE_REG(huart->OFFSET_8.FCR, value);
}

/**
 * @brief  Enable UART Loopback mode
 * @retval None
 */
static inline void ms_uart_ll_enable_loopback(Uart_Type *huart)
{
    SET_BIT(huart->MCR, UART_MCR_LoopBack);
}

/**
 * @brief  Disable UART Loopback mode
 * @retval None
 */
static inline void ms_uart_ll_disable_loopback(Uart_Type *huart)
{
    CLEAR_BIT(huart->MCR, UART_MCR_LoopBack);
}

/**
 * @brief  Set UART stop bits num
 * @retval None
 */
static inline void ms_uart_ll_set_stop_bits_num(Uart_Type *huart, uint32_t bits_num)
{
    MODIFY_REG(huart->LCR, UART_LCR_STOP, (bits_num << UART_LCR_STOP_POS) & UART_LCR_STOP_MASK);
}

/**
 * @brief  Set UART parity
 * @retval None
 */
static inline void ms_uart_ll_set_parity(Uart_Type *huart, uint32_t parity)
{
    MODIFY_REG(huart->LCR, UART_LCR_PEN | UART_LCR_EPS, (parity << UART_LCR_PEN_POS) & (UART_LCR_PEN | UART_LCR_EPS));
}

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
static inline void ms_uart_ll_set_auto_flow_ctrl(Uart_Type *huart, uint32_t auto_flow_ctrl)
{
    MODIFY_REG(huart->MCR, UART_MCR_AFCE, (auto_flow_ctrl << UART_MCR_AFCE_POS) & (UART_MCR_AFCE_MASK));
}

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
 *
 * @retval None
 */
static inline void ms_uart_ll_set_rts(Uart_Type *huart, uint32_t rts)
{
    MODIFY_REG(huart->MCR, UART_MCR_RTS, (rts << UART_MCR_RTS_POS) & (UART_MCR_RTS_MASK));
}

/**
 * @brief  set uart word length
 * @retval None
 */
static inline void ms_uart_ll_set_word_length(Uart_Type *huart, uint32_t length)
{
    if (length == UART_WORDLENGTH_9B)
    {
        MODIFY_REG(huart->LCR, UART_LCR_DLS, UART_WORDLENGTH_8B << UART_LCR_DLS_POS);
        SET_BIT(huart->LCR_EXT, UART_LCR_EXT_DLS_E);
    }
    else
    {
        MODIFY_REG(huart->LCR, UART_LCR_DLS, length << UART_LCR_DLS_POS);
        CLEAR_BIT(huart->LCR_EXT, UART_LCR_EXT_DLS_E);
    }
}

/**
 * @brief  Set UART baudrate
 * @retval None
 */
extern int32_t ms_uart_ll_set_baudrate(Uart_Type *huart, uint32_t baudrate);

/** @}*/

/** @}*/

#endif /* MS_UART_H_ */
