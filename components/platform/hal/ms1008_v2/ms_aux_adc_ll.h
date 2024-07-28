/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_aux_adc_ll.h
 * @brief Header file of watchdog  module.
 * @author haijun.mai
 * @date   2022-01-24
 * @version 1.0
 * @Revision
 */


#ifndef MS_AUX_ADC_LL_H_
#define MS_AUX_ADC_LL_H_

#include "ms1008.h"
#include "ms_aux_adc_regs.h"
#include <stdint.h>
#include <stdbool.h>
#include "log.h"

#define   MS_AUX_ADC_SAMPLE_MODE_SINGLE_ENDED   0
#define   MS_AUX_ADC_SAMPLE_MODE_DIFFRENTIAL       1

#define   MS_AUX_ADC_CALIBRATION_MODE_NORMAL                0
#define   MS_AUX_ADC_CALIBRATION_MODE_CALIBRATION       1

#define   MS_AUX_ADC_Q0                     0
#define   MS_AUX_ADC_Q1                     1
#define   MS_AUX_ADC_Q2                     2
#define   MS_AUX_ADC_Q3                     3
#define   MS_AUX_ADC_Q4                     4
#define   MS_AUX_ADC_Q5                     5
#define   MS_AUX_ADC_Q6                     6
#define   MS_AUX_ADC_Q7                     7


/**
 * @brief enable aux adc 
 * @param  AuxAdc_Type *aux_adc: aux_adc regs
 * @retval None
 */
static inline void  ms_aux_adc_enable_ll(AuxAdc_Type *aux_adc)
{
    SET_BIT(aux_adc->CTR,AUX_ADC_CTR_ADC_SMP_EN);
}

/**
 * @brief disable aux adc 
 * @param  AuxAdc_Type *aux_adc: aux_adc regs
 * @retval None
 */
static inline void  ms_aux_adc_disable_ll(AuxAdc_Type *aux_adc)
{
    CLEAR_BIT(aux_adc->CTR,AUX_ADC_CTR_ADC_SMP_EN);
}


/**
 * @brief  aux adc dma enable
 * @param  AuxAdc_Type *aux_adc: aux_adc regs
 * @retval None
 */
static inline void  ms_aux_adc_dma_enable_ll(AuxAdc_Type *aux_adc)
{
    SET_BIT(aux_adc->CTR,AUX_ADC_CTR_DMA_CTL);
}

/**
 * @brief  aux adc dma disable
 * @param  AuxAdc_Type *aux_adc: aux_adc regs
 * @retval None
 */
static inline void  ms_aux_adc_dma_disable_ll(AuxAdc_Type *aux_adc)
{
    CLEAR_BIT(aux_adc->CTR,AUX_ADC_CTR_DMA_CTL);
}


/**
 * @brief  aux adc fifo flush
 * @param  AuxAdc_Type *aux_adc: aux_adc regs
 * @retval None
 */
static inline void  ms_aux_adc_fifo_flush_ll(AuxAdc_Type *aux_adc)
{
    SET_BIT(aux_adc->CTR,AUX_ADC_CTR_FIFO_FLUSH);
}

/**
 * @brief  aux adc set sample mode
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t sample_mode
 * @param This parameter can be one of the following values:
 * @param MS_AUX_ADC_SAMPLE_MODE_SINGLE_ENDED                           
 * @param MS_AUX_ADC_SAMPLE_MODE_DIFFRENTIAL           
 * @retval None
 */
static inline void  ms_aux_adc_set_sample_mode_ll(AuxAdc_Type *aux_adc,uint8_t sample_mode)
{
    CLEAR_BIT(aux_adc->CTR,AUX_ADC_CTR_SDIF);
    SET_BIT(aux_adc->CTR,((sample_mode<<AUX_ADC_CTR_SDIF_POS)&AUX_ADC_CTR_SDIF_MASK));
}

/**
 * @brief  aux adc set calibration mode
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t calibration_mode
 * @param This parameter can be one of the following values:
 * @param MS_AUX_ADC_CALIBRATION_MODE_NORMAL                           
 * @param MS_AUX_ADC_CALIBRATION_MODE_CALIBRATION           
 * @retval None
 */
static inline void  ms_aux_adc_set_calibration_mode_ll(AuxAdc_Type *aux_adc,uint8_t calibration_mode)
{
    CLEAR_BIT(aux_adc->CTR,AUX_ADC_CTR_DISH);
    SET_BIT(aux_adc->CTR,((calibration_mode<<AUX_ADC_CTR_DISH_POS)&AUX_ADC_CTR_DISH_MASK));
}


/**
 * @brief  aux adc set calibration value
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t calibration_value        
 * @retval None
 */
static inline void  ms_aux_adc_set_calibration_value_ll(AuxAdc_Type *aux_adc,uint8_t calibration_value)
{
    CLEAR_BIT(aux_adc->CTR,AUX_ADC_CTR_CAL_OFFSET);
    SET_BIT(aux_adc->CTR,((calibration_value<<AUX_ADC_CTR_CAL_OFFSET_POS)&AUX_ADC_CTR_CAL_OFFSET_MASK));
}


/**
 * @brief  aux adc set the number of sample
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t sample_cnt        
 * @retval None
 */
static inline void  ms_aux_adc_set_sample_count_ll(AuxAdc_Type *aux_adc,uint8_t sample_cnt)
{
    CLEAR_BIT(aux_adc->SMPCNT,AUX_ADC_SMPCNT_SMP_CNT);
    SET_BIT(aux_adc->SMPCNT,((sample_cnt<<AUX_ADC_SMPCNT_SMP_CNT_POS)&AUX_ADC_SMPCNT_SMP_CNT_MASK));
}


/**
 * @brief  aux adc get the number of already sample
 * @param AuxAdc_Type *aux_adc: aux_adc regs   
 * @retval the number of already sample count
 */
static inline uint16_t  ms_aux_adc_get_already_sample_count_ll(AuxAdc_Type *aux_adc)
{
    return (READ_REG(aux_adc->SMPCNTR)&AUX_ADC_SMPCNT_SMP_CNT_R);
    
}

/**
 * @brief  aux adc set watermask
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t watermask_value        
 * @retval None
 */
static inline void  ms_aux_adc_set_watermask_ll(AuxAdc_Type *aux_adc,uint8_t watermask_value)
{
    CLEAR_BIT(aux_adc->WATERMARK,AUX_ADC_WATERMARK);
    SET_BIT(aux_adc->WATERMARK,(((watermask_value-1)<<AUX_ADC_WATERMARK_POS)&AUX_ADC_WATERMARK_MASK));
}


/**
 * @brief  aux adc set watermask
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t channel_queue_depth        
 * @retval None
 */
static inline void  ms_aux_adc_set_channel_queue_depth_ll(AuxAdc_Type *aux_adc,uint8_t channel_queue_depth)
{
    CLEAR_BIT(aux_adc->CHNQUEUE,AUX_ADC_CHNQUEUE_CHN_QUEUE_LEN);
    SET_BIT(aux_adc->CHNQUEUE,(((channel_queue_depth-1)<<AUX_ADC_CHNQUEUE_CHN_QUEUE_LEN_POS)&AUX_ADC_CHNQUEUE_CHN_QUEUE_LEN_MASK));
}

/**
 * @brief  aux adc set channel queue
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t idx   
 * @param This parameter can be one of the following values:
 * @param  MS_AUX_ADC_Q0
 * @param  MS_AUX_ADC_Q1
 * @param  MS_AUX_ADC_Q2
 * @param  MS_AUX_ADC_Q3
 * @param  MS_AUX_ADC_Q4
 * @param  MS_AUX_ADC_Q5
 * @param  MS_AUX_ADC_Q6
 * @param  MS_AUX_ADC_Q7
 * @param uint8_t channel : channel is 0~7   
 * @retval None
 */
extern void  ms_aux_adc_set_channel_queue_ll(AuxAdc_Type *aux_adc,uint8_t idx,uint8_t channel);


/**
 * @brief  aux adc mask interrupt
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t interrupt_src 
 * @param This parameter can be one of the following values:
 * @param AUX_ADC_IRQMASK_WATERMASK
 * @param AUX_ADC_IRQMASK_ALMOSTFULL
 * @param AUX_ADC_IRQMASK_FULL
 * @param AUX_ADC_IRQMASK_ALMOSTEMPTY
 * @param AUX_ADC_IRQMASK_EMPTY
 * @param AUX_ADC_IRQMASK_FIFO_ERR
 * @param AUX_ADC_IRQMASK_SMP_DONE
 * @retval None
 */
static inline void  ms_aux_adc_mask_interrupt_ll(AuxAdc_Type *aux_adc,uint8_t interrupt_src)
{
    CLEAR_BIT(aux_adc->IRQMASK,interrupt_src);
}



/**
 * @brief  aux adc unmask interrupt
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t interrupt_mask  
 * @param This parameter can be one of the following values:
 * @param AUX_ADC_IRQMASK_WATERMASK
 * @param AUX_ADC_IRQMASK_ALMOSTFULL
 * @param AUX_ADC_IRQMASK_FULL
 * @param AUX_ADC_IRQMASK_ALMOSTEMPTY
 * @param AUX_ADC_IRQMASK_EMPTY
 * @param AUX_ADC_IRQMASK_FIFO_ERR
 * @param AUX_ADC_IRQMASK_SMP_DONE
 * @retval None
 */
static inline void  ms_aux_adc_unmask_interrupt_ll(AuxAdc_Type *aux_adc,uint8_t interrupt_mask)
{
     SET_BIT(aux_adc->IRQMASK,interrupt_mask);
}

/**
 * @brief  aux adc get interrupt status
 * @param AuxAdc_Type *aux_adc: aux_adc regs   
 * @retval the fifo status
 */
static inline uint8_t  ms_aux_adc_get_interrupt_status_ll(AuxAdc_Type *aux_adc)
{
    return READ_REG(aux_adc->IRQSTATUS);
    
}


/**
 * @brief  aux adc clear interrupt status
 * @param AuxAdc_Type *aux_adc: aux_adc regs 
 * @param uint8_t interrupt_src  
 * @param This parameter can be one of the following values:
 * @param AUX_ADC_IRQSTATUS_WATERMASK
 * @param AUX_ADC_IRQSTATUS_ALMOSTFULL
 * @param AUX_ADC_IRQSTATUS_FULL
 * @param AUX_ADC_IRQMASK_FIFO_ERR
 * @param AUX_ADC_IRQMASK_SMP_DONE
 * @retval the fifo status
 */
static inline void  ms_aux_adc_clear_interrupt_status_ll(AuxAdc_Type *aux_adc,uint8_t interrupt_src)
{
     SET_BIT(aux_adc->IRQSTATUS,interrupt_src);
}


/**
 * @brief  aux adc get fifo status
 * @param AuxAdc_Type *aux_adc: aux_adc regs   
 * @retval the fifo status
 */
static inline uint8_t  ms_aux_adc_get_fifo_status_ll(AuxAdc_Type *aux_adc)
{
    return READ_REG(aux_adc->FIFOSTATUS);
    
}

/**
 * @brief  aux adc get fifo data
 * @param AuxAdc_Type *aux_adc: aux_adc regs   
 * @retval the fifo status
 */
static inline uint32_t  ms_aux_adc_get_fifo_data_ll(AuxAdc_Type *aux_adc)
{
    return READ_REG(aux_adc->FIFODATA);
    
}



#endif
