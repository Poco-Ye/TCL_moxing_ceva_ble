/**
 ****************************************************************************************
 *
 * @file spi.h
 *
 * @brief SPI Driver
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _SPI_H_
#define _SPI_H_

/**
 ****************************************************************************************
 * @defgroup SPI
 * @ingroup DRIVERS
 * @brief SPI driver
 *
 * @{
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "arch.h"

#if (PLF_SPI)

#include <stdint.h>              // standard integer definitions


/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

///Enable/disable SPI module
enum SPI_ENABLE
{
    SPI_DISABLED = 0,
    SPI_ENABLED = 1
};


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Initializes the SPI to default values.
 *****************************************************************************************
 */
void spi_init(void);

/**
 ****************************************************************************************
 * @brief Starts a data reception.
 *
 * As soon as the end of the data transfer or a buffer overflow is detected,
 * the callback function is called.
 *
 * @param[in,out]  bufptr        Pointer to the RX buffer
 * @param[in]      size          Size of the expected reception
 * @param[in]      rx_callback   Callback for end of reception
 *****************************************************************************************
 */
void spi_read(uint8_t *bufptr, uint32_t size, void (*rx_callback)(void));

/**
 ****************************************************************************************
 * @brief Starts a data transmission.
 *
 * As soon as the end of the data transfer is detected, the callback function is called.
 *
 * @param[in]  bufptr        Pointer to the TX buffer
 * @param[in]  size          Size of the transmission
 * @param[in]  tx_callback   Callback for end of reception
 *****************************************************************************************
 */
void spi_write(uint8_t *bufptr, uint32_t size, void (*tx_callback)(void));

/**
 ****************************************************************************************
 * @brief Enable / Disable SPI interrupts.
 *
 * @param[in]  mode        0:disable / 1:enable
 *****************************************************************************************
 */
void spi_interrupt_mode(uint8_t mode);

/**
 ****************************************************************************************
 * @brief Poll SPI on reception and transmission.
 *
 * This function is used to poll SPI for reception and transmission.
 * It is used when IRQ are not used to detect incoming bytes.
 *****************************************************************************************
 */
void spi_poll(void);

/**
 ****************************************************************************************
 * @brief SPI interrupt handler
 *****************************************************************************************
 */
void spi_isr(void);

#endif //PLF_SPI

/// @}
#endif /* _SPI_H_ */
