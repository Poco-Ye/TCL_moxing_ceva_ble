/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_watchdog_hal.h
 * @brief Header file of watchdog  module.
 * @author haijun.mai
 * @date   2022-01-05
 * @version 1.0
 * @Revision
 */

#ifndef MS_I2S_H_
#define MS_I2S_H_

#include <ms1008.h>
#include "ms_i2s_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <ms_single_dmac.h>
#ifdef __cplusplus
extern "C" {
#endif



#define I2S_MASTER     0
#define I2S_SLAVE     1

#define I2S_ERROR_INVALID_CALLBACK  0x00000001U   /*!< Invalid Callback error  */

#define I2S_MASTER_STATUS_IDLE   0
#define I2S_MASTER_STATUS_BUSY   1


typedef struct
{
    uint8_t status;
    uint32_t *tx_l_ptr;  
    uint32_t *tx_r_ptr;  
    uint16_t tx_len;
    uint16_t tx_total_len; 
	
    uint32_t *rx_l_ptr; 
    uint32_t *rx_r_ptr;  
    uint16_t rx_len; 
    uint16_t rx_total_len;

} I2sMasterInterrupt_Type;

typedef struct {
    
	uint32_t wss; /**/
	uint32_t sclkg;
	uint32_t i2s_div;
	uint8_t tx_data_valid_len;
       uint8_t rx_data_valid_len;
	uint8_t role; /**/
	uint8_t rx_fifo_trigger_level;
	uint8_t tx_fifo_trigger_level;
	uint8_t i2s_tx_block_en;
	uint8_t i2s_tx_channel_en;
	uint8_t i2s_rx_block_en;
	uint8_t i2s_rx_channel_en;
       uint8_t i2s_tx_dma_en;
	uint8_t i2s_rx_dma_en;
	
	
	
} I2sInit_Type;

struct __I2sHandle_Type;

/**
 * @brief  Timer callback handle Structure definition
 */
typedef struct {
	void (*error_callback)(struct __I2sHandle_Type *i2s);
	void (*init_callback)(struct __I2sHandle_Type *i2s);
	void (*deinit_callback)(struct __I2sHandle_Type *i2s);
	void (*i2s_reach_callback)(struct __I2sHandle_Type *i2s);
} I2sCallback_Type;

typedef struct __I2sHandle_Type {
	I2s_Type *instance;
	I2sDma_Type *dma_instance;
	I2sInit_Type init;/*!< i2s communication parameters      */
	uint32_t error_code; /*!< i2s Error code*/
	I2sMasterInterrupt_Type interrupt;
	IRQn_Type irq;
	I2sCallback_Type *p_callback;
	DmacHandle_Type *hdmatx;            /*!< Pointer to the TX DMA              */
       DmacHandle_Type *hdmarx;            /*!< Pointer to the RX DMA              */
} I2sHandle_Type;



extern void ms_i2s_enable_cpu_interrupt(I2sHandle_Type *i2s);

extern void ms_i2s_disable_cpu_interrupt(I2sHandle_Type *i2s);

extern int32_t ms_i2s_master_init(I2sHandle_Type *i2s);

extern int32_t ms_i2s_master_deinit(I2sHandle_Type *i2s);

extern void ms_i2s_master_send_data(I2sHandle_Type *i2s, uint32_t* left_chan_data, uint32_t* right_chan_data, uint32_t len);

extern uint32_t ms_i2s_master_receive_data(I2sHandle_Type *i2s,  uint32_t *left_chan_data, uint32_t *right_chan_data, uint32_t len);

extern int32_t ms_i2s_master_int_send_data(I2sHandle_Type *i2s, uint32_t* left_chan_data, uint32_t* right_chan_data, uint32_t len);

extern int32_t ms_i2s_master_int_receive_data(I2sHandle_Type *i2s,  uint32_t* left_chan_data, uint32_t* right_chan_data, uint32_t len);

extern int32_t ms_i2s_master_dma_send_data(I2sHandle_Type *i2s, uint32_t *p_data, uint16_t size);

extern int32_t ms_i2s_master_dma_receive_data(I2sHandle_Type *i2s, uint32_t *buffer_ptr, uint32_t size);

extern void ms_i2s_master_irq_handler(I2sHandle_Type *i2s);

#ifdef __cplusplus
}
#endif

#endif /* MS_WATCHDOG_H_ */

