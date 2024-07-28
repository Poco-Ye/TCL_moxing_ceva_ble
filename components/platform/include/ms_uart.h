/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_uart.h
 * @brief
 * @author bingrui.chen
 * @date 2021-12-21
 * @version 1.0
 * @Revision
 */
#ifndef MS_UART_H_
#define MS_UART_H_

#include <ms_single_dmac.h>
#include <ms_uart_hal.h>
#include <ringbuffer.h>
#include "ms_interrupt.h"

#include <stdint.h>
#include <stdbool.h>

/** @defgroup UART_Error_Code UART Error Code
 * @{
 */
#define UART_ERROR_NONE              0x00000000U   /*!< No error            */
#define UART_ERROR_PE                0x00000001U   /*!< Parity error        */
#define UART_ERROR_FE                0x00000002U   /*!< Frame error         */
#define UART_ERROR_ORE               0x00000004U   /*!< Overrun error       */
#define UART_ERROR_BI                0x00000008U   /*!< Break interrupt error  */
#define UART_ERROR_INVALID_CALLBACK  0x00000020U   /*!< Invalid Callback error  */
/**
 * @}
 */

/** @defgroup UART_Hardware_Flow_Control UART Hardware Flow Control
 * @{
 */
#define UART_HWCONTROL_NONE                  0x00000000U
#define UART_HWCONTROL_AUTO_CTS              0x00000001U
#define UART_HWCONTROL_AUTO_RTS_CTS          0x00000002U
/**
 * @}
 */

/**
 * @brief UART Init Structure Definition
 */
typedef struct
{
    uint32_t baud_rate;         /*!< This member configures the UART communication baud rate.*/
    uint32_t word_length;       /*!< Specifies the number of data bits transmitted or received in a frame.*/
    uint32_t stop_bits;         /*!< Specifies the number of stop bits transmitted. */
    uint32_t parity;            /*!< Specifies the parity mode.*/
    uint32_t mode;              /*!< Specifies whether the Receive or Transmit mode is enabled or disabled. */
    uint8_t hw_flow_ctl;        /*!< Specifies whether the hardware flow control mode is enabled or disabled.*/
    uint8_t fifo_enable;        /*!< DMAC Channel Priority  @ref DMAC_CHANNEL_PRIOR*/
    uint8_t tx_fifo_level;      /*!< DMAC Channel Priority  @ref DMAC_CHANNEL_PRIOR*/
    uint8_t rx_fifo_level;      /*!< DMAC Channel Priority  @ref DMAC_CHANNEL_PRIOR*/
} UartInit_Type;

struct __UartHandle_Type;

/**
 * @brief  UART Callback Handle Structure Definition
 */
typedef struct
{
    void (*rx_cplt_callback)(struct __UartHandle_Type *huart);  /*!< UART RX complete callback handle  */
    void (*tx_cplt_callback)(struct __UartHandle_Type *huart);  /*!< UART TX complete callback handle  */
    void (*init_callback)(struct __UartHandle_Type *huart);     /*!< UART Initial callback handle  */
    void (*deinit_callback)(struct __UartHandle_Type *huart);   /*!< UART De-initial callback handle  */
} UartCallback_Type;

/**
 * @brief  UART Handle Structure Definition
 */
typedef struct __UartHandle_Type
{
    Uart_Type *instance;                /*!< UART registers base address        */
    UartInit_Type init;                 /*!< UART communication parameters      */
    uint8_t *tx_buff_ptr;               /*!< Pointer to UART Tx transfer Buffer */
    uint16_t tx_xfer_size;              /*!< UART Tx Transfer size              */
    uint16_t tx_xfer_cnt;               /*!< UART Tx Transfer Counter           */
    uint8_t *rx_buff_ptr;               /*!< Pointer to UART Rx transfer Buffer */
    uint16_t rx_xfer_size;              /*!< UART Rx Transfer size              */
    uint16_t rx_xfer_cnt;               /*!< UART Rx Transfer Counter           */
    uint32_t error_code;                /*!< UART Error code                    */
    IRQn_Type irq;                      /*!< UART IRQ Number                    */
    RingBuffer_Type ringbuffer;         /*!< UART RingBuffer                    */
    UartCallback_Type *hcallback;       /*!< Pointer to the UART Callback Handle*/
    DmacHandle_Type *hdmatx;            /*!< Pointer to the TX DMA              */
    DmacHandle_Type *hdmarx;            /*!< Pointer to the RX DMA              */
} UartHandle_Type;

/**
 * @brief  Initial UART Controller
 * @param[in]  UartHandle_Type *huart:  Pointer to a UART_Handle_Type structure that contains
 *                the configuration information for the specified UART module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_uart_init(UartHandle_Type *huart);

/**
 * @brief  De-initial UART Controller
 * @param[in]  UartHandle_Type *huart:  Pointer to a UART_Handle_Type structure that contains
 *                the configuration information for the specified UART module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_uart_deinit(UartHandle_Type *huart);

/**
 * @brief  Transmit an amount of data in dma mode
 * @param[in]  UartHandle_Type *huart:  Pointer to a UART_Handle_Type structure that contains
 *                the configuration information for the specified UART module.
 * @param[in] uint8_t *pData: Pointer to Transmit data buffer.
 * @param[in] int32_t len: The length of Transmit data
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_uart_transmit_dma(UartHandle_Type *huart, uint8_t *p_data, uint16_t size);

/**
 * @brief  Transmit an amount of data in non blocking mode
 * @param[in]  UartHandle_Type *huart:  Pointer to a UART_Handle_Type structure that contains
 *                the configuration information for the specified UART module.
 * @param[in] uint8_t *pData: Pointer to Transmit data buffer.
 * @param[in] int32_t len: The length of Transmit data
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_uart_transmit(UartHandle_Type *huart, uint8_t *pData, uint16_t Size);

/**
 * @brief  Transmit an amount of data in interrupt mode
 * @param[in]  UartHandle_Type *huart:  Pointer to a UART_Handle_Type structure that contains
 *                the configuration information for the specified UART module.
 * @param[in] uint8_t *pData: Pointer to Transmit data buffer.
 * @param[in] int32_t len: The length of Transmit data
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_uart_transmit_it(UartHandle_Type *huart, uint8_t *pData, uint16_t Size);

/**
 * @brief Start Interrupt Mode Reception
 * @param[in] UartHandle_Type *huart: Pointer to a UART_Handle_Type structure that contains
 *            the configuration information for the specified UART module.
 * @param[in] uint8_t *ringbuffer_ptr: Pointer to Received RingBuffer
 * @param[in] int32_t len: The RingBuffer length
 * @retval The Result of Start Received in Interrupt Mode
 */
int32_t ms_uart_start_rx_dma(UartHandle_Type *huart, uint8_t *buffer_ptr, uint32_t size);
/**
 * @brief Start Interrupt Mode Reception
 * @param[in] UartHandle_Type *huart: Pointer to a UART_Handle_Type structure that contains
 *            the configuration information for the specified UART module.
 * @param[in] uint8_t *ringbuffer_ptr: Pointer to Received RingBuffer
 * @param[in] int32_t len: The RingBuffer length
 * @retval The Result of Start Received in Interrupt Mode
 */
int32_t ms_uart_start_rx_it(UartHandle_Type *huart, uint8_t *ringbuffer_ptr, uint32_t size);

/**
 * @brief Stop Interrupt Mode Reception
 * @param[in] UartHandle_Type *huart: Pointer to a UART_Handle_Type structure that contains
 *            the configuration information for the specified UART module.
 * @retval The Result of Stop Received in Interrupt Mode
 */
int32_t ms_uart_stop_rx_it(UartHandle_Type *huart);

/**
 * @brief  Receive Uart Data by Polling Mode
 * @param[in] UartHandle_Type *huart: Pointer to a UART_Handle_Type structure that contains
 *            the configuration information for the specified UART module.
 * @param[in] uint8_t *pbuffer: Pointer to Received Buffer
 * @param[in] int32_t len: The buffer length
 * @retval The Received Data Length
 */
int32_t ms_uart_receive(UartHandle_Type *huart, uint8_t *pbuffer, int32_t size);

/**
 * @brief  Enable Cpu Interrupt
 * @param[in] UartHandle_Type *huart: Pointer to a UART_Handle_Type structure that contains
 *            the configuration information for the specified UART module.
 * @retval None
 */
void ms_uart_enable_cpu_interrupt(UartHandle_Type *huart);

/**
 * @brief  Disable Cpu Interrupt
 * @param[in] UartHandle_Type *huart: Pointer to a UART_Handle_Type structure that contains
 *            the configuration information for the specified UART module.
 * @retval None
 */
void ms_uart_disable_cpu_interrupt(UartHandle_Type *huart);

/**
 * @brief  This function handles UART interrupt request.
 * @param[in] UartHandle_Type *huart: Pointer to a UART_Handle_Type structure that contains
 *            the configuration information for the specified UART module.
 * @retval None
 */
void ms_uart_irq_handler(UartHandle_Type *huart);

/**
 * @brief  Set UART Baudrate
 * @param[in] UartHandle_Type *huart: Pointer to a UART_Handle_Type structure that contains
 *            the configuration information for the specified UART module.
 * @param[in] uint32_t baudrate: Pointer to Received Buffer
 * @retval The Received Data Length
 */
int32_t ms_uart_set_baudrate(UartHandle_Type *huart, uint32_t baudrate);

/**
 * @brief  Set UART Baudrate
 * @param[in] UartHandle_Type *huart: Pointer to a UART_Handle_Type structure that contains
 *            the configuration information for the specified UART module.
 * @param[in] uint32_t empty_lvl: @ref UART_TX_FIFO_EMPTY_TRG_LVL
 * @retval The Received Data Length
 */
int32_t ms_uart_set_tx_fifo_empty_lvl(UartHandle_Type *huart, uint32_t empty_lvl);

/**
 * @brief  Set UART RX FIFO Full Level
 * @param[in] UartHandle_Type *huart: Pointer to a UART_Handle_Type structure that contains
 *            the configuration information for the specified UART module.
 * @param[in] uint32_t trig_lvl: @ref UART_RX_FIFO_AVAILABLE_TRG_LVL
 * @retval The Received Data Length
 */
int32_t ms_uart_set_rx_fifo_full_lvl(UartHandle_Type *huart, uint32_t trig_lvl);

#endif /* MS_UART_H_ */
