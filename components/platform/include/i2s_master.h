/*
 * spi_master.h
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#ifndef I2S_MASTER_H_
#define I2S_MASTER_H_

#include "ms1008.h"
#include "ms_spi_master.h"


extern int32_t i2s0_master_init(void);
extern int32_t i2s0_master_send( uint32_t* left_chan_data, uint32_t* right_chan_data, uint32_t len);
extern int32_t i2s0_master_receive(uint32_t *left_chan_data, uint32_t *right_chan_data, uint32_t len);
extern int32_t i2s0_master_int_send( uint32_t* left_chan_data, uint32_t* right_chan_data, uint32_t len);
extern int32_t i2s0_master_int_receive(uint32_t *left_chan_data, uint32_t *right_chan_data, uint32_t len);
extern int32_t i2s0_master_dma_send( uint32_t* send_data, uint32_t len);
extern int32_t i2s0_master_dma_receive(  uint32_t *receive_data, uint32_t len);
extern int32_t i2s0_master_deinit(void);


extern int32_t i2s1_master_init(void);
extern int32_t i2s1_master_send( uint32_t* left_chan_data, uint32_t* right_chan_data, uint32_t len);
extern int32_t i2s1_master_receive(uint32_t *left_chan_data, uint32_t *right_chan_data, uint32_t len);
extern int32_t i2s1_master_int_send( uint32_t* left_chan_data, uint32_t* right_chan_data, uint32_t len);
extern int32_t i2s1_master_int_receive(uint32_t *left_chan_data, uint32_t *right_chan_data, uint32_t len);
extern int32_t i2s1_master_dma_send( uint32_t* send_data, uint32_t len);
extern int32_t i2s1_master_dma_receive(  uint32_t *receive_data, uint32_t len);
extern int32_t i2s1_master_deinit(void);


#endif /* SPI_MASTERH_ */

