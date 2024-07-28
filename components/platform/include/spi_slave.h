/*
 * spi_slave.h
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#ifndef SPI_SLAVE_H_
#define SPI_SLAVE_H_

#include "ms1008.h"
#include "ms_spi_slave.h"


extern uint32_t spi_slave_init(void);
extern int32_t spi_slave_send(const uint8_t *data, uint16_t size, uint32_t timeout);
extern int32_t spi_slave_receive(uint8_t *rx_data, uint16_t size, uint32_t timeout);
extern int32_t spi_slave_interrupt_send(const uint8_t *data, uint16_t size, uint32_t timeout);
extern int32_t spi_slave_interrupt_receive( uint8_t *rx_data, uint16_t size, uint32_t timeout);
extern int32_t spi_slave_deinit(void);
extern int32_t spi_slave_cs_set_low();
extern int32_t spi_slave_cs_set_high();

#endif /* SPI_MASTERH_ */

