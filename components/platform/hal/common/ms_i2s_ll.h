/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_i2s_ll.h
 * @brief Header file of i2s  module.
 * @author haijun.mai
 * @date   2022-03-07
 * @version 1.0
 * @Revision
 */

#ifndef MS_I2S_LL_H_
#define MS_I2S_LL_H_

#include "ms1008.h"
#include "ms_i2s_regs.h"
#include <stdint.h>
#include <stdbool.h>
#include "log.h"

#define I2S_TRUE               1
#define I2S_FAULT               0

#define CLOCK_CYCLES_16             (0)
#define CLOCK_CYCLES_24             (1)
#define CLOCK_CYCLES_32             (2)


#define SCLK_NO_CLOCK_GATING            (0)
#define SCLK_CLOCK_CYCLES_12             (1)
#define SCLK_CLOCK_CYCLES_16             (2)
#define SCLK_CLOCK_CYCLES_20             (3)
#define SCLK_CLOCK_CYCLES_24             (4)


#define IGNORE_WORD_LENGTH    (0)
#define RESOLUTION_12_BIT        (1)
#define RESOLUTION_16_BIT        (2)
#define RESOLUTION_20_BIT        (3)
#define RESOLUTION_24_BIT        (4)
#define RESOLUTION_32_BIT        (5)

#define I2S_FIFO_DEPTH  (16)

/* I2S module parameters */
#define    I2S_WORDSIZE_DONT_CARE (0)
#define    I2S_WORDSIZE_12bit       (1)
#define    I2S_WORDSIZE_16bit       (2)
#define    I2S_WORDSIZE_20bit       (3)
#define    I2S_WORDSIZE_24bit       (4)
#define    I2S_WORDSIZE_32bit       (5)

#define I2S_FIFO_TRIGGERL_LEVEL_1   (0)
#define I2S_FIFO_TRIGGERL_LEVEL_2   (1)
#define I2S_FIFO_TRIGGERL_LEVEL_3   (2)
#define I2S_FIFO_TRIGGERL_LEVEL_4   (3)
#define I2S_FIFO_TRIGGERL_LEVEL_5   (4)
#define I2S_FIFO_TRIGGERL_LEVEL_6   (5)
#define I2S_FIFO_TRIGGERL_LEVEL_7   (6)
#define I2S_FIFO_TRIGGERL_LEVEL_8   (7)
#define I2S_FIFO_TRIGGERL_LEVEL_9   (8)
#define I2S_FIFO_TRIGGERL_LEVEL_10  (9)
#define I2S_FIFO_TRIGGERL_LEVEL_11  (10)
#define I2S_FIFO_TRIGGERL_LEVEL_12  (11)
#define I2S_FIFO_TRIGGERL_LEVEL_13  (12)
#define I2S_FIFO_TRIGGERL_LEVEL_14  (13)
#define I2S_FIFO_TRIGGERL_LEVEL_15  (14)
#define I2S_FIFO_TRIGGERL_LEVEL_16  (15)



/**
 * @brief enable i2s controller
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_enable_ll(I2s_Type *i2s)
{
	SET_BIT(i2s->IER, I2S_IER_IEN);
}

/**
 * @brief disable i2s controller
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_disable_ll(I2s_Type *i2s)
{
	CLEAR_BIT(i2s->IER, I2S_IER_IEN);
}

/**
 * @brief enable i2s tx block
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_rx_block_enable_ll(I2s_Type *i2s)
{
	SET_BIT(i2s->IRER, I2S_IRER_RXEN);
}

/**
 * @brief disable i2s tx block
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_rx_block_disable_ll(I2s_Type *i2s)
{
	CLEAR_BIT(i2s->IRER, I2S_IRER_RXEN);
}


/**
 * @brief enable i2s rx block
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_tx_block_enable_ll(I2s_Type *i2s)
{
	SET_BIT(i2s->ITER, I2S_ITER_TXEN);
}

/**
 * @brief disable i2s rx block
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_tx_block_disable_ll(I2s_Type *i2s)
{
	CLEAR_BIT(i2s->ITER, I2S_ITER_TXEN);
}

/**
 * @brief the clock signals sclk_en, ws_out, and sclk_gate appear on the interface
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_clock_enable_ll(I2s_Type *i2s)
{
	SET_BIT(i2s->CER, I2S_CER_CLKEN);
}

/**
 * @brief the clock signals sclk_en, ws_out, and sclk_gate  on the interface are close
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_clock_disable_ll(I2s_Type *i2s)
{
	CLEAR_BIT(i2s->CER, I2S_CER_CLKEN);
}



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
static inline void ms_i2s_cfg_wss_ll(I2s_Type *i2s,uint8_t wss)
{
	MODIFY_REG(i2s->CCR,I2S_CCR_WSS ,((wss<<I2S_CCR_WSS_POS)& I2S_CCR_WSS_MASK));
}


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
static inline void ms_i2s_cfg_sclkg_ll(I2s_Type *i2s,uint8_t sclkg)
{
	MODIFY_REG(i2s->CCR, I2S_CCR_SCLKG,((sclkg<<I2S_CCR_SCLKG_POS)&I2S_CCR_SCLKG_MASK));
}



/**
 * @brief reset rx block fifo 
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_rx_block_fifo_reset_ll(I2s_Type *i2s)
{
	SET_BIT(i2s->RXFFR, I2S_RXFFR_RXFFR);
}

/**
 * @brief  reset tx block fifo
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_tx_block_fifo_reset_ll(I2s_Type *i2s)
{
	SET_BIT(i2s->TXFFR, I2S_TXFFR_TXFFR);
}


/**
 * @brief  read data forom left channel
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline uint32_t ms_i2s_get_left_channel_data_ll(I2s_Type *i2s)
{
	return READ_REG(i2s->LRTR0.LRBR0);
}

/**
 * @brief  read data forom right channel
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline uint32_t ms_i2s_get_right_channel_data_ll(I2s_Type *i2s)
{
	return READ_REG(i2s->RRTR0.RRBR0);
}



/**
 * @brief  fill data to i2s left channel
 * @param  I2s_Type *i2s: I2S regs
 * @uint32_t left_channel_data
 * @retval None
 */
static inline void ms_i2s_fill_left_channel_data_ll(I2s_Type *i2s,uint32_t left_channel_data)
{
	WRITE_REG(i2s->LRTR0.LTHR0, left_channel_data);
}


/**
 * @brief  fill data to i2s right channel
 * @param  I2s_Type *i2s: I2S regs
 * @ uint32_t right_channel_data
 * @retval None
 */
static inline void ms_i2s_fill_right_channel_data_ll(I2s_Type *i2s,uint32_t right_channel_data)
{
	WRITE_REG(i2s->RRTR0.RTHR0, right_channel_data);
}



/**
 * @brief enable i2s tx channel
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_rx_channel_enable_ll(I2s_Type *i2s)
{
	SET_BIT(i2s->RER0, I2S_RERX_RXCHENX);
}

/**
 * @brief disable i2s tx channel
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_rx_channel_disable_ll(I2s_Type *i2s)
{
	CLEAR_BIT(i2s->RER0, I2S_RERX_RXCHENX);
}



/**
 * @brief enable i2s tx channel
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_tx_channel_enable_ll(I2s_Type *i2s)
{
	SET_BIT(i2s->TER0, I2S_TERX_TXCHENX);
}

/**
 * @brief disable i2s tx channel
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_tx_channel_disable_ll(I2s_Type *i2s)
{
	CLEAR_BIT(i2s->TER0, I2S_TERX_TXCHENX);
}

/**
 * @brief program the trigger level in the RX FIFO at which the Received Data Available interrupt and DMA request is generated.
 * @param  I2s_Type *i2s: I2S regs   
 * @param  uint8_t fifo_level 
 * @param   This parameter can be one of the following values: 0-0xf           
 * @retval None
 */
static inline void ms_i2s_cfg_rx_fifo_trigger_level_ll(I2s_Type *i2s,uint8_t fifo_level)
{
	MODIFY_REG(i2s->RFCR0, I2S_RFCRX_RXCHDT,((fifo_level<<I2S_RFCRX_RXCHDT_POS)&I2S_RFCRX_RXCHDT_MASK));
}

/**
 * @brief program the trigger level in the TX FIFO at which the Empty Threshold Reached Interrupt and DMA request is generated.
 * @param  I2s_Type *i2s: I2S regs   
 * @param  uint8_t fifo_level 
 * @param   This parameter can be one of the following values: 0-0xf           
 * @retval None
 */
static inline void ms_i2s_cfg_tx_fifo_trigger_level_ll(I2s_Type *i2s,uint8_t fifo_level)
{
	MODIFY_REG(i2s->TFCR0, I2S_TFCRX_TXCHET,((fifo_level<<I2S_TFCRX_TXCHET_POS)&I2S_TFCRX_TXCHET_MASK));
}

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
static inline void ms_i2s_cfg_rx_resolution_wlen_ll(I2s_Type *i2s,uint8_t wlen)
{
	MODIFY_REG(i2s->RCR0, I2S_RCRX_WLEN,((wlen<<I2S_RCRX_WLEN_POS)&I2S_RCRX_WLEN_POS));
}



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
static inline void ms_i2s_cfg_tx_resolution_wlen_ll(I2s_Type *i2s,uint8_t wlen)
{
	MODIFY_REG(i2s->TCR0, I2S_TCRX_WLEN,((wlen<<I2S_TCRX_WLEN_POS)&I2S_TCRX_WLEN_MASK));
}


/**
 * @brief get the status of i2s
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval i2s interrupt status
 */
static inline uint32_t ms_i2s_get_interrupt_status_ll(I2s_Type *i2s)
{
	return READ_REG(i2s->ISR0);
}

/**
 * @brief get the mask of i2s
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval i2s interrupt mask
 */
static inline uint32_t ms_i2s_get_interrupt_mask_ll(I2s_Type *i2s)
{
	return READ_REG(i2s->IMR0);
}



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
static inline void ms_i2s_interrupt_mask_ll(I2s_Type *i2s,uint32_t interrupt_mask)
{
	SET_BIT(i2s->IMR0, interrupt_mask);
}

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
static inline void ms_i2s_interrupt_unmask_ll(I2s_Type *i2s,uint32_t interrupt_mask)
{
	CLEAR_BIT(i2s->IMR0, interrupt_mask);
}


/**
 * @brief clear the RX FIFO Data Overrun interrupt
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_clear_rx_fifo_data_overun_status_ll(I2s_Type *i2s)
{
	SET_BIT(i2s->ROR0, I2S_RORX_RXCHO);
}


/**
 * @brief clear the TX FIFO Data Overrun interrupt
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_clear_tx_fifo_data_overun_status_ll(I2s_Type *i2s)
{
	SET_BIT(i2s->TOR0, I2S_RORX_RXCHO); 
}




/**
 * @brief reset i2s rx fifo
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_rx_fifo_reset_ll(I2s_Type *i2s)
{
	SET_BIT(i2s->RFF0, I2S_RFFX_RXCHFR);
}


/**
 * @brief reset i2s tx fifo
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_tx_fifo_reset_ll(I2s_Type *i2s)
{
	SET_BIT(i2s->TFF0, I2S_TFFX_TXCHFR);
}



/**
 * @brief i2s tx channel dma enable
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_dma_tx_channel_enable_ll(I2sDma_Type *i2s)
{
	SET_BIT(i2s->DMACR, (I2S_DMACR_DMAEN_TXBLOCK|I2S_DMACR_DMAEN_TXCH_0));
#if 0
	MS_LOGI(MS_DRIVER, "\r\n RXDMA(%x) = %x!\n",&i2s->RXDMA,i2s->RXDMA);
	MS_LOGI(MS_DRIVER, "\r\n TXDMA(%x) = %x!\n",&i2s->TXDMA,i2s->TXDMA);
       MS_LOGI(MS_DRIVER, "\r\n I2S_COMP_PARAM_2(%x) = %x!\n",&i2s->I2S_COMP_PARAM_2,i2s->I2S_COMP_PARAM_2);
	MS_LOGI(MS_DRIVER, "\r\n I2S_COMP_VERSION(%x) = %x!\n",&i2s->I2S_COMP_VERSION,i2s->I2S_COMP_VERSION);
	MS_LOGI(MS_DRIVER, "\r\n I2S_COMP_TYPE(%x) = %x!\n",&i2s->I2S_COMP_TYPE,i2s->I2S_COMP_TYPE);
	MS_LOGI(MS_DRIVER, "\r\n DMACR(%x) = %x!\n",&i2s->DMACR,i2s->DMACR);
	MS_LOGI(MS_DRIVER, "\r\n (I2S_DMACR_DMAEN_TXBLOCK|I2S_DMACR_DMAEN_TXCH_0)= %x!\n",(I2S_DMACR_DMAEN_TXBLOCK|I2S_DMACR_DMAEN_TXCH_0));
#endif
}

/**
 * @brief i2s tx channel dma disable
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_dma_tx_channel_disable_ll(I2sDma_Type *i2s)
{
	CLEAR_BIT(i2s->DMACR, (I2S_DMACR_DMAEN_TXBLOCK|I2S_DMACR_DMAEN_TXCH_0));
}



/**
 * @brief i2s rx channel dma enable
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_dma_rx_channel_enable_ll(I2sDma_Type *i2s)
{
	SET_BIT(i2s->DMACR,(I2S_DMACR_DMAEN_RXBLOCK|I2S_DMACR_DMAEN_RXCH_0) ); 
}

/**
 * @brief i2s rx channel dma disable
 * @param  I2s_Type *i2s: I2S regs                    
 * @retval None
 */
static inline void ms_i2s_dma_rx_channel_disable_ll(I2sDma_Type *i2s)
{
	CLEAR_BIT(i2s->DMACR, (I2S_DMACR_DMAEN_RXBLOCK|I2S_DMACR_DMAEN_RXCH_0));
}


#endif
