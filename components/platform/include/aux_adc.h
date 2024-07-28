/*
 * aux_adc.h
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#ifndef AUX_ADC_H_
#define AUX_ADC_H_

#include "ms1008.h"
#include "ms_aux_adc_hal.h"


extern uint32_t aux_adc_init(void);
extern uint32_t  aux_adc_disable();
extern uint32_t  aux_adc_enable();
extern uint8_t aux_adc_get_fifo_status();
extern uint32_t aux_adc_get_data(void);
extern uint32_t aux_adc_deinit(void);
extern  uint32_t  aux_adc_set_channel_queue_depth(uint32_t queue_depth);
extern uint32_t aux_adc_set_channel_queue(uint32_t channel_queue_idx,uint32_t channel_queue_channelx);
extern uint32_t aux_adc_set_sample_count(uint32_t sample_count);
extern  uint32_t  aux_adc_set_watermask(uint32_t watermask_value);
extern uint8_t aux_adc_get_interrupt_status();
extern  uint32_t aux_adc_clear_interrupt_status();
extern uint32_t aux_adc_unmask_sample_done_interrupt();
extern uint32_t aux_adc_clear_interrupt_flag();
extern uint32_t aux_adc_get_interrupt_flag();
extern uint32_t aux_adc_mask_cpu_interrupt();
extern uint32_t aux_adc_dma_receive_data( uint32_t *buffer_ptr, uint32_t size);
extern uint32_t  aux_adc_unmask_watermask_interrupt();
extern uint32_t aux_adc_top_init(void);
extern uint32_t aux_adc_top_get_data(void);
#endif /* AUX_ADC_H_ */
