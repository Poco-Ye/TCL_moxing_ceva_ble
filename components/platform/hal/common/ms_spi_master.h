/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file  ms_spi_master.h
 * @brief
 * @author haijun.mai
 * @date 2022-01-12
 * @version 1.0
 * @Revision
 */

#ifndef MS_SPI_MASTER_H_
#define MS_SPI_MASTER_H_

#include "ms1008.h"
#include "ms_spi_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


#define SPI_MASTER_ERROR_INVALID_CALLBACK  0x00000001U   /*!< Invalid Callback error  */

#define SPI_MASTER_STATUS_IDLE   0
#define SPI_MASTER_STATUS_BUSY   1

typedef struct
{
    uint8_t status;
    uint8_t *tx_ptr;  
    uint16_t tx_len;
    uint16_t tx_total_len; 
	
    uint8_t *rx_ptr;  
    uint16_t rx_len; 
    uint16_t rx_total_len;

} SpiMasterInterrupt_Type;

typedef struct
{
    uint8_t      frame_forma;  
    uint8_t        frame_size;
    uint8_t        cpol;
    uint8_t 	 cpha;
    uint8_t 	 mode_transfer;
    uint8_t       role;
    uint8_t      data_frames_number;
    uint8_t     sck_div;
    uint8_t     transmit_fifo_threshold;
    uint8_t    rx_threshold_level;
    uint8_t    sste; 
    uint8_t    protocol_frame_format;
    uint8_t   sample_delay;
} SpMasteriInit_Type;

struct __SpiHandle_Type;
/**
  * @brief  Timer callback handle Structure definition
  */
typedef struct
{
    void (* error_callback)                (struct __SpiHandle_Type *spi_master);
    void (* init_callback)                   (struct __SpiHandle_Type *spi_master);
    void (* deinit_callback)               (struct __SpiHandle_Type *spi_master);
    void (* spi_reach_callback)    (struct __SpiHandle_Type *spi_master);
}SpiMasterCallback_Type;


typedef struct __SpiHandle_Type
{
    Spi_Type           *instance;
    SpMasteriInit_Type                init;/*!< Timer communication parameters      */
    uint32_t                           error_code;         /*!< TIMER Error code*/      
    SpiMasterInterrupt_Type         interrupt;
    IRQn_Type                      irq;
    SpiMasterCallback_Type         *p_callback;
}SpiMasterHandle_Type;




/**
 * @brief  Enable Cpu Interrupt
 * @param[in] SpiMasterHandle_Type *spi : Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @retval None
 */
extern void ms_spi_master_enable_cpu_interrupt(SpiMasterHandle_Type *spi);


/**
 * @brief  Disable Cpu Interrupt
 * @param[in] SpiMasterHandle_Type *spi : Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @retval None
 */
extern void ms_spi_master_disable_cpu_interrupt(SpiMasterHandle_Type *spi);


/**
 * @brief  Initial SPI Controller
 * @param[in]  SpiMasterHandle_Type *spi:  Pointer to a SpiHandle_Type  structure that contains
 * @ the configuration information for the specified SPI module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
extern int32_t ms_spi_master_init(SpiMasterHandle_Type *spi);


/**
 * @brief  Deinitial SPI Controller
 * @param[in]  SpiMasterHandle_Type *spi:  Pointer to a SpiHandle_Type  structure that contains
 * @ the configuration information for the specified SPI module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
extern int32_t ms_spi_master_deinit(SpiMasterHandle_Type *spi);




/**
 * @brief  Transmit an amount of data in Polling Mode
 * @param[in]  SpiMasterHandle_Type *spi:  Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @param[in] const uint8_t *data: Pointer to Transmit data buffer.
 * @param[in] uint16_t size: The length of Transmit data
 * @param[in] uint32_t timeout: The timerout value
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
extern int32_t ms_spi_master_send(SpiMasterHandle_Type *spi, const uint8_t *data, uint16_t size, uint32_t timeout);


/**
 * @brief  Receive Spi Data by Polling Mode
 * @param[in] SpiMasterHandle_Type *spi: Pointer to a SpiHandle_Type structure that contains
 *            the configuration information for the specified SPI module.
 * @param[in] uint8_t *rx_data: Pointer to Received Buffer
 * @param[in] uint16_t size: The buffer length
 * @param[in] uint32_t timeout: The timer value
 * @retval The Received Data Length
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
extern int32_t ms_spi_master_receive(SpiMasterHandle_Type *spi, uint8_t *rx_data, uint16_t size, uint32_t timeout);


/**
 * @brief  Transmit an amount of data in interrupt mode
 * @param[in]  SpiMasterHandle_Type *spi:  Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @param[in] const uint8_t *data: Pointer to Transmit data buffer.
 * @param[in] uint16_t size: The length of Transmit data
 * @param[in] uint32_t timeout: The timerout value
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
extern int32_t ms_spi_master_int_send(SpiMasterHandle_Type *spi, const uint8_t *data, uint16_t size, uint32_t timeout);


/**
 * @brief  Receive Spi Data in interrupt mode 
 * @param[in] SpiMasterHandle_Type *spi: Pointer to a SpiHandle_Type structure that contains
 *            the configuration information for the specified SPI module.
 * @param[in] uint8_t *rx_data: Pointer to Received Buffer
 * @param[in] uint16_t size: The buffer length
 * @param[in] uint32_t timeout: The timer value
 * @retval The Received Data Length
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
extern int32_t ms_spi_master_int_receive(SpiMasterHandle_Type *spi, uint8_t *rx_data, uint16_t size, uint32_t timeout);


/**
 * @brief  This function handles SPI interrupt request.
 * @param[in] SpiMasterHandle_Type *spi: Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @retval None
 */
extern void ms_spi_master_irq_handler(SpiMasterHandle_Type *spi);

#ifdef __cplusplus
}
#endif

#endif /* MS_SPI_MASTER*/
