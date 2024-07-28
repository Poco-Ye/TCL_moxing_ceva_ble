/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_i2s_hal.h
 * @brief Header file of watchdog  module.
 * @author haijun.mai
 * @date   2022-03-08
 * @version 1.0
 * @Revision
 */

#ifndef MS_I2S_HAL_H_
#define MS_I2S_HAL_H_

#include "ms_i2s_ll.h"

/**
 * @brief enable i2s controller
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_enable_hal(i2s)    ms_i2s_enable_ll(i2s)


/**
 * @brief disable i2s controller
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_disable_hal(i2s)    ms_i2s_disable_ll(i2s)


/**
 * @brief enable i2s tx block
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_rx_block_enable_hal(i2s)     ms_i2s_rx_block_enable_ll(i2s)


/**
 * @brief disable i2s tx block
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_rx_block_disable_hal(i2s)     ms_i2s_rx_block_disable_ll(i2s)


/**
 * @brief enable i2s rx block
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_tx_block_enable_hal(i2s)    ms_i2s_tx_block_enable_ll(i2s)


/**
 * @brief disable i2s rx block
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_tx_block_disable_hal(i2s)    ms_i2s_tx_block_disable_ll(i2s)


/**
 * @brief the clock signals sclk_en, ws_out, and sclk_gate appear on the interface
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_clock_enable_hal(i2s)    ms_i2s_clock_enable_ll(i2s)


/**
 * @brief the clock signals sclk_en, ws_out, and sclk_gate  on the interface are close
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_clock_disable_hal(i2s)    ms_i2s_clock_disable_ll(i2s)

/**
 * @brief program the number of sclk cycles for which the word select line (ws_out) stays in the left or right sample mode.
 * @param  I2s_Type *i2s: I2S regs  
 * @param   This parameter can be one of the following values:
 * @NO_CLOCK_GATING           
 * @CLOCK_CYCLES_16            
 * @CLOCK_CYCLES_24           
 * @CLOCK_CYCLES_32                                                               
 * @retval None
 */
#define  ms_i2s_cfg_wss_hal(i2s, wss)    ms_i2s_cfg_wss_ll(i2s, wss)



/**
 * @brief program the gating of sclk. The programmed gating value must be less than or equal to the largest configured/programmed audio resolution to prevent the truncating of RX/TX data
 * @param  I2s_Type *i2s: I2S regs   
 * @param   This parameter can be one of the following values:
 * @  SCLK_NO_CLOCK_GATING
 * @ SCLK_CLOCK_CYCLES_12
 * @ SCLK_CLOCK_CYCLES_16          
 * @ SCLK_CLOCK_CYCLES_20           
 * @ SCLK_CLOCK_CYCLES_24           
 * @retval None
 */
#define  ms_i2s_cfg_sclkg_hal(i2s,  sclkg)    ms_i2s_cfg_sclkg_ll(i2s,  sclkg)


/**
 * @brief reset rx block fifo 
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define  ms_i2s_rx_block_fifo_reset_hal(i2s)    ms_i2s_rx_block_fifo_reset_ll(i2s)



/**
 * @brief  reset tx block fifo
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define  ms_i2s_tx_block_fifo_reset_hal(i2s)    ms_i2s_tx_block_fifo_reset_ll(i2s)


/**
 * @brief  read data forom left channel
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define  ms_i2s_get_left_channel_data_hal(i2s)    ms_i2s_get_left_channel_data_ll(i2s)

/**
 * @brief  read data forom right channel
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define  ms_i2s_get_right_channel_data_hal(i2s)    ms_i2s_get_right_channel_data_ll(i2s)



/**
 * @brief  fill data to i2s left channel
 * @param  I2s_Type *i2s: I2S regs
 * @uint32_t left_channel_data
 * @retval None
 */
#define ms_i2s_fill_left_channel_data_hal(i2s, left_channel_data)    ms_i2s_fill_left_channel_data_ll(i2s, left_channel_data)


/**
 * @brief  fill data to i2s right channel
 * @param  I2s_Type *i2s: I2S regs
 * @ uint32_t right_channel_data
 * @retval None
 */
#define ms_i2s_fill_right_channel_data_hal(i2s, right_channel_data)    ms_i2s_fill_right_channel_data_ll(i2s, right_channel_data)


/**
 * @brief enable i2s tx channel
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define  ms_i2s_rx_channel_enable_hal(i2s)    ms_i2s_rx_channel_enable_ll(i2s)


/**
 * @brief disable i2s tx channel
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_rx_channel_disable_hal(i2s)    ms_i2s_rx_channel_disable_ll(i2s)


/**
 * @brief enable i2s tx channel
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_tx_channel_enable_hal(i2s)    ms_i2s_tx_channel_enable_ll(i2s)

/**
 * @brief disable i2s tx channel
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_tx_channel_disable_hal(i2s)    ms_i2s_tx_channel_disable_ll(i2s)


/**
 * @brief program the trigger level in the RX FIFO at which the Received Data Available interrupt and DMA request is generated.
 * @param  I2s_Type *i2s: I2S regs   
 * @param  uint8_t fifo_level 
 * @param   This parameter can be one of the following values: 0-0xf           
 * @retval None
 */
#define ms_i2s_cfg_rx_fifo_trigger_level_hal(i2s, fifo_level)     ms_i2s_cfg_rx_fifo_trigger_level_ll(i2s, fifo_level)


/**
 * @brief program the trigger level in the TX FIFO at which the Empty Threshold Reached Interrupt and DMA request is generated.
 * @param  I2s_Type *i2s: I2S regs   
 * @param  uint8_t fifo_level 
 * @param   This parameter can be one of the following values: 0-0xf           
 * @retval None
 */
#define  ms_i2s_cfg_tx_fifo_trigger_level_hal(i2s, fifo_level)    ms_i2s_cfg_tx_fifo_trigger_level_ll(i2s, fifo_level)



/**
 * @brief program the desired data resolution of the receiver and enables the LSB of the incoming left (orright) word to be placed in the LSB of the LRBRx (orRRBRx) register.
 * @param  I2s_Type *i2s: I2S regs   
 * @param  uint8_t wlen 
 * @param   This parameter can be one of the following values: 
 * @param  IGNORE_WORD_LENGTH
 * @param RESOLUTION_12_BIT
 * @param RESOLUTION_16_BIT
 * @param RESOLUTION_20_BIT
 * @param RESOLUTION_24_BIT
 * @param RESOLUTION_32_BIT
 * @retval None
 */
#define ms_i2s_cfg_rx_resolution_wlen_hal(i2s, wlen)    ms_i2s_cfg_rx_resolution_wlen_ll(i2s, wlen)



/**
 * @brief program the data resolution of thetransmitter and ensures the MSB of the data is transmitted first
 * @param  I2s_Type *i2s: I2S regs   
 * @param  uint8_t wlen 
 * @param   This parameter can be one of the following values: 
 * @param  IGNORE_WORD_LENGTH
 * @param RESOLUTION_12_BIT
 * @param RESOLUTION_16_BIT
 * @param RESOLUTION_20_BIT
 * @param RESOLUTION_24_BIT
 * @param RESOLUTION_32_BIT
 * @retval None
 */
#define ms_i2s_cfg_tx_resolution_wlen_hal(i2s, wlen)    ms_i2s_cfg_tx_resolution_wlen_ll(i2s, wlen)





/**
 * @brief get the status of i2sl
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval i2s interrupt status
 */
#define  ms_i2s_get_interrupt_status_hal(i2s)    ms_i2s_get_interrupt_status_ll(i2s)



/**
 * @brief get the mask of i2s
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval i2s interrupt mask
 */
#define  ms_i2s_get_interrupt_mask_hal(i2s)   ms_i2s_get_interrupt_mask_ll(i2s)


/**
 * @brief mask interrupt of i2s
 * @param  I2s_Type *i2s: I2S regs   
 * @ uint32_t interrupt_mask
 * @param This parameter can be one of the following values: 
 * @param I2S_IMRX_RXDAM
 * @param I2S_IMRX_RXFOM
 * @param I2S_IMRX_TXFEM
 * @param I2S_IMRX_TXFOM
 * @retval none
 */
#define ms_i2s_interrupt_mask_hal(i2s, interrupt_mask)    ms_i2s_interrupt_mask_ll(i2s, interrupt_mask)


/**
 * @brief unmask interrupt of i2s
 * @param  I2s_Type *i2s: I2S regs   
 * @ uint32_t interrupt_mask
 * @param This parameter can be one of the following values: 
 * @param I2S_IMRX_RXDAM
 * @param I2S_IMRX_RXFOM
 * @param I2S_IMRX_TXFEM
 * @param I2S_IMRX_TXFOM
 * @retval none
 */
#define ms_i2s_interrupt_unmask_hal(i2s, interrupt_mask)   ms_i2s_interrupt_unmask_ll(i2s, interrupt_mask)

/**
 * @brief clear the RX FIFO Data Overrun interrupt
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define  ms_i2s_clear_rx_fifo_data_overun_status_hal(i2s)    ms_i2s_clear_rx_fifo_data_overun_status_ll(i2s)


/**
 * @brief clear the TX FIFO Data Overrun interrupt
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_clear_tx_fifo_data_overun_status_hal(i2s)    ms_i2s_clear_tx_fifo_data_overun_status_ll(i2s)




/**
 * @brief reset i2s rx fifo
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define  ms_i2s_rx_fifo_reset_hal(i2s)    ms_i2s_rx_fifo_reset_ll(i2s)



/**
 * @brief reset i2s tx fifo
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define  ms_i2s_tx_fifo_reset_hal(i2s)    ms_i2s_tx_fifo_reset_ll(i2s)


/**
 * @brief i2s tx channel dma enable
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_dma_tx_channel_enable_hal(i2s)    ms_i2s_dma_tx_channel_enable_ll(i2s)


/**
 * @brief i2s tx channel dma disable
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_dma_tx_channel_disable_hal(i2s)    ms_i2s_dma_tx_channel_disable_ll(i2s)




/**
 * @brief i2s rx channel dma enable
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_dma_rx_channel_enable_hal(i2s)    ms_i2s_dma_rx_channel_enable_ll(i2s)


/**
 * @brief i2s rx channel dma disable
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
#define ms_i2s_dma_rx_channel_disable_hal(i2s)    ms_i2s_dma_rx_channel_disable_ll(i2s)


#endif/* MS_I2S_HAL_H_ */

