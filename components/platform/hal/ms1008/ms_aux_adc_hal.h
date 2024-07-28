/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_aux_adc_hal_ll.c
 * @brief c source  file of aux adc  module.
 * @author haijun.mai
 * @date   2022-01-24
 * @version 1.0
 * @Revision
 */
#ifndef MS_AUX_ADC_HAL_H_
#define MS_AUX_ADC_HAL_H_

#include "ms_aux_adc_ll.h"

/**
 * @brief enable aux adc 
 * @param  AuxAdc_Type *aux_adc: aux_adc regs
 * @retval None
 */
#define  ms_aux_adc_enable_hal(aux_adc)   ms_aux_adc_enable_ll(aux_adc)

/**
 * @brief disable aux adc 
 * @param  AuxAdc_Type *aux_adc: aux_adc regs
 * @retval None
 */
#define  ms_aux_adc_disable_hal(aux_adc)  ms_aux_adc_disable_ll(aux_adc)

/**
 * @brief  aux adc dma enable
 * @param  AuxAdc_Type *aux_adc: aux_adc regs
 * @retval None
 */
#define ms_aux_adc_dma_enable_hal(aux_adc) ms_aux_adc_dma_enable_ll(aux_adc)

/**
 * @brief  aux adc dma disable
 * @param  AuxAdc_Type *aux_adc: aux_adc regs
 * @retval None
 */
#define  ms_aux_adc_dma_disable_hal(aux_adc)  ms_aux_adc_dma_disable_ll(aux_adc)


/**
 * @brief  aux adc fifo flush
 * @param  AuxAdc_Type *aux_adc: aux_adc regs
 * @retval None
 */
#define  ms_aux_adc_fifo_flush_hal(aux_adc)   ms_aux_adc_fifo_flush_ll(aux_adc)


/**
 * @brief  aux adc set sample mode
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t sample_mode
 * @param This parameter can be one of the following values:
 * @param MS_AUX_ADC_SAMPLE_MODE_SINGLE_ENDED                           
 * @param MS_AUX_ADC_SAMPLE_MODE_DIFFRENTIAL           
 * @retval None
 */
#define ms_aux_adc_set_sample_mode_hal(aux_adc, sample_mode) ms_aux_adc_set_sample_mode_ll(aux_adc, sample_mode)

/**
 * @brief  aux adc set calibration mode
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t calibration_mode
 * @param This parameter can be one of the following values:
 * @param MS_AUX_ADC_CALIBRATION_MODE_NORMAL                           
 * @param MS_AUX_ADC_CALIBRATION_MODE_CALIBRATION           
 * @retval None
 */
#define ms_aux_adc_set_calibration_mode_hal(aux_adc, calibration_mode) ms_aux_adc_set_calibration_mode_ll(aux_adc, calibration_mode)

/**
 * @brief  aux adc set calibration value
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t calibration_value        
 * @retval None
 */
#define  ms_aux_adc_set_calibration_value_hal(aux_adc, calibration_value) ms_aux_adc_set_calibration_value_ll(aux_adc, calibration_value)


/**
 * @brief  aux adc set the number of sample
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t sample_cnt        
 * @retval None
 */
#define ms_aux_adc_set_sample_count_hal(aux_adc, sample_cnt)  ms_aux_adc_set_sample_count_ll(aux_adc, sample_cnt) 

/**
 * @brief  aux adc get the number of already sample
 * @param AuxAdc_Type *aux_adc: aux_adc regs   
 * @retval the number of already sample count
 */
#define  ms_aux_adc_get_already_sample_count_hal(aux_adc)  ms_aux_adc_get_already_sample_count_ll(aux_adc) 


/**
 * @brief  aux adc set watermask
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t watermask_value        
 * @retval None
 */
#define ms_aux_adc_set_watermask_hal(aux_adc, watermask_value)  ms_aux_adc_set_watermask_ll(aux_adc, watermask_value)


/**
 * @brief  aux adc set watermask
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t channel_queue_depth        
 * @retval None
 */
#define  ms_aux_adc_set_channel_queue_depth_hal(aux_adc, channel_queue_depth)  ms_aux_adc_set_channel_queue_depth_ll(aux_adc, channel_queue_depth)

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
#define ms_aux_adc_set_channel_queue_hal(aux_adc, idx, channel)  ms_aux_adc_set_channel_queue_ll(aux_adc, idx, channel)

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
#define   ms_aux_adc_mask_interrupt_hal(aux_adc, interrupt_src) ms_aux_adc_mask_interrupt_ll(aux_adc, interrupt_src)


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
#define   ms_aux_adc_unmask_interrupt_hal(aux_adc, interrupt_mask)  ms_aux_adc_unmask_interrupt_ll(aux_adc, interrupt_mask)

/**
 * @brief  aux adc get interrupt status
 * @param AuxAdc_Type *aux_adc: aux_adc regs   
 * @retval the fifo status
 */
#define  ms_aux_adc_get_interrupt_status_hal(aux_adc)  ms_aux_adc_get_interrupt_status_ll(aux_adc)

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
#define   ms_aux_adc_clear_interrupt_status_hal(aux_adc, interrupt_src) ms_aux_adc_clear_interrupt_status_ll(aux_adc, interrupt_src)

/**
 * @brief  aux adc get fifo status
 * @param AuxAdc_Type *aux_adc: aux_adc regs   
 * @retval the fifo status
 */
#define  ms_aux_adc_get_fifo_status_hal(aux_adc) ms_aux_adc_get_fifo_status_ll(aux_adc)

/**
 * @brief  aux adc get fifo data
 * @param AuxAdc_Type *aux_adc: aux_adc regs   
 * @retval the fifo status
 */
#define  ms_aux_adc_get_fifo_data_hal(aux_adc)  ms_aux_adc_get_fifo_data_ll(aux_adc)


#endif/* MS_TIMER_HAL_H_ */


