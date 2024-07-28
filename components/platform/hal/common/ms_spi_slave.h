/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file  ms_spi_slave.h
 * @brief
 * @author haijun.mai
 * @date 2022-02-22
 * @version 1.0
 * @Revision
 */

#ifndef MS_SPI_SLAVE_H_
#define MS_SPI_SLAVE_H_

#include "ms1008.h"
#include "ms_spi_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


#define SPI_SLAVE_ERROR_INVALID_CALLBACK  0x00000001U   /*!< Invalid Callback error  */

#define SPI_SLAVE_STATUS_IDLE   0
#define SPI_SLAVE_STATUS_BUSY   1

typedef struct
{
    uint8_t status;
    uint8_t *tx_ptr;  
    uint16_t tx_len;
    uint16_t tx_total_len; 
	
    uint8_t *rx_ptr;  
    uint16_t rx_len; 
    uint16_t rx_total_len;

} SpiSlaveInterrupt_Type;

typedef struct
{
    uint8_t      frame_forma;  
    uint8_t        frame_size;
    uint8_t        cpol;
    uint8_t 	 cpha;
    uint8_t 	 mode_transfer;
    uint8_t       role;
   // uint8_t      data_frames_number;
  //  uint8_t     sck_div;
    uint8_t     transmit_fifo_threshold;
    uint8_t    rx_threshold_level;
    uint8_t    sste; 
    uint8_t    protocol_frame_format;
    uint8_t   sample_delay;
} SpiSlaveInit_Type;

struct __SpiSlaveHandle_Type;
/**
  * @brief  Timer callback handle Structure definition
  */
typedef struct
{
    void (* error_callback)                (struct __SpiSlaveHandle_Type *spi_slave);
    void (* init_callback)                   (struct __SpiSlaveHandle_Type *spi_slave);
    void (* deinit_callback)               (struct __SpiSlaveHandle_Type *spi_slave);
    void (* spi_reach_callback)    (struct __SpiSlaveHandle_Type *spi_slave);
}SpiSlaveCallback_Type;


typedef struct __SpiSlaveHandle_Type
{
    Spi_Type           *instance;
    SpiSlaveInit_Type                init;/*!< Timer communication parameters      */
    uint32_t                           error_code;         /*!< TIMER Error code*/      
    SpiSlaveInterrupt_Type         interrupt;
    IRQn_Type                      irq;
    SpiSlaveCallback_Type         *p_callback;
}SpiSlaveHandle_Type;


/**
 * @brief  Enable Cpu Interrupt
 * @param[in] SpiSlaveHandle_Type *spi : Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @retval None
 */
void ms_spi_slave_enable_cpu_interrupt(SpiSlaveHandle_Type *spi);

/**
 * @brief  Disable Cpu Interrupt
 * @param[in] SpiSlaveHandle_Type *spi : Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @retval None
 */
void ms_spi_slave_disable_cpu_interrupt(SpiSlaveHandle_Type *spi);


/**
 * @brief  Initial SPI Controller
 * @param[in]  SpiSlaveHandle_Type *spi:  Pointer to a SpiSlaveHandle_Type  structure that contains
 * @ the configuration information for the specified SPI module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_spi_slave_init(SpiSlaveHandle_Type *spi);


/**
 * @brief  Deinitial SPI Controller
 * @param[in]  SpiSlaveHandle_Type *spi:  Pointer to a SpiSlaveHandle_Type  structure that contains
 * @ the configuration information for the specified SPI module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_spi_slave_deinit(SpiSlaveHandle_Type *spi);



/**
 * @brief  Transmit an amount of data in Polling Mode
 * @param[in]  SpiSlaveHandle_Type *spi:  Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @param[in] const uint8_t *data: Pointer to Transmit data buffer.
 * @param[in] uint16_t size: The length of Transmit data
 * @param[in] uint32_t timeout: The timerout value
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_spi_slave_send(SpiSlaveHandle_Type *spi, const uint8_t *data, uint16_t size, uint32_t timeout);


/**
 * @brief  Receive Spi Data by Polling Mode
 * @param[in] SpiSlaveHandle_Type *spi: Pointer to a SpiHandle_Type structure that contains
 *            the configuration information for the specified SPI module.
 * @param[in] uint8_t *rx_data: Pointer to Received Buffer
 * @param[in] uint16_t size: The buffer length
 * @param[in] uint32_t timeout: The timer value
 * @retval The Received Data Length
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_spi_slave_receive(SpiSlaveHandle_Type *spi, uint8_t *rx_data, uint16_t size, uint32_t timeout);


/**
 * @brief  Transmit an amount of data in interrupt mode
 * @param[in]  SpiSlaveHandle_Type *spi:  Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @param[in] const uint8_t *data: Pointer to Transmit data buffer.
 * @param[in] uint16_t size: The length of Transmit data
 * @param[in] uint32_t timeout: The timerout value
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_spi_slave_int_send(SpiSlaveHandle_Type *spi, const uint8_t *data, uint16_t size, uint32_t timeout);



/**
 * @brief  Receive Spi Data in interrupt mode 
 * @param[in] SpiSlaveHandle_Type *spi: Pointer to a SpiHandle_Type structure that contains
 *            the configuration information for the specified SPI module.
 * @param[in] uint8_t *rx_data: Pointer to Received Buffer
 * @param[in] uint16_t size: The buffer length
 * @param[in] uint32_t timeout: The timer value
 * @retval The Received Data Length
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_spi_slave_int_receive(SpiSlaveHandle_Type *spi, uint8_t *rx_data, uint16_t size, uint32_t timeout);

/**
 * @brief  This function handles SPI interrupt request.
 * @param[in] SpiSlaveHandle_Type *spi: Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @retval None
 */
void ms_spi_slave_irq_handler(SpiSlaveHandle_Type *spi);


#ifdef __cplusplus
}
#endif

#endif /* MS_SPI_SLAVE*/


