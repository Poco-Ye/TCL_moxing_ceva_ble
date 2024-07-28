/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_aux_adc_hal.h
 * @brief Header file of aux adc  module.
 * @author haijun.mai
 * @date   2022-01-24
 * @version 1.0
 * @Revision
 */


#ifndef MS_AUX_ADC_H_
#define MS_AUX_ADC_H_

#include "ms1008.h"
#include "ms_aux_adc_ll.h"
#include <stdint.h>
#include <stdbool.h>
#include <ms_single_dmac.h>
#ifdef __cplusplus
extern "C" {
#endif





#define AUX_ADC_ERROR_INVALID_CALLBACK  0x00000001U   /*!< Invalid Callback error  */


typedef struct
{
    uint8_t sample_mode; 
    uint8_t calibration_mode;
    uint8_t calibration_value;
    uint8_t sample_cnt;
    uint8_t watermask_value;
    uint8_t channel_queue_depth;
    uint8_t channel_queue_idx;
    uint8_t channel_queue_channelx;
    uint8_t interrupt_src;
    uint8_t dma_en; 
} AuxadcInit_Type;

struct __AuxadcHandle_Type;
/**
  * @brief  Timer callback handle Structure definition
  */
typedef struct
{
    void (* error_callback)                (struct __AuxadcHandle_Type *aux_adc);
    void (* init_callback)                   (struct __AuxadcHandle_Type *aux_adc);
    void (* deinit_callback)               (struct __AuxadcHandle_Type *aux_adc);
    void (* aux_adc_reach_callback)    (struct __AuxadcHandle_Type *aux_adc);
}AuxadcCallback_Type;


typedef struct __AuxadcHandle_Type
{
    AuxAdc_Type           *instance;
    AuxadcInit_Type                init;/*!< auxadc communication parameters      */
    uint32_t                           error_code;         /*!< TIMER Error code*/      
    uint8_t interrupt_status;
    IRQn_Type                      irq;
    AuxadcCallback_Type         *p_callback;
    DmacHandle_Type *hdmarx;            /*!< Pointer to the RX DMA              */
}AuxadcHandle_Type;

extern void ms_aux_adc_enable_cpu_interrupt(AuxadcHandle_Type *aux_adc);
extern void ms_aux_adc_disable_cpu_interrupt(AuxadcHandle_Type *aux_adc);
extern int32_t ms_aux_adc_init(AuxadcHandle_Type *aux_adc);
extern void  ms_aux_adc_enable(AuxadcHandle_Type *aux_adc);
extern void  ms_aux_adc_disable(AuxadcHandle_Type *aux_adc);
extern void ms_aux_adc_dma_enable(AuxadcHandle_Type *aux_adc);
extern void ms_aux_adc_dma_disable(AuxadcHandle_Type *aux_adc);
extern void ms_aux_adc_fifo_flush(AuxadcHandle_Type *aux_adc);
extern void ms_aux_adc_set_sample_mode(AuxadcHandle_Type *aux_adc);
extern void ms_aux_adc_set_calibration_mode(AuxadcHandle_Type *aux_adc);
extern void ms_aux_adc_set_calibration_value(AuxadcHandle_Type *aux_adc);
extern void ms_aux_adc_set_sample_count(AuxadcHandle_Type *aux_adc);
extern uint16_t ms_aux_adc_get_already_sample_count(AuxadcHandle_Type *aux_adc);
extern  void  ms_aux_adc_set_watermask(AuxadcHandle_Type *aux_adc);
extern void  ms_aux_adc_set_channel_queue_depth(AuxadcHandle_Type *aux_adc);
extern  void  ms_aux_adc_set_channel_queue(AuxadcHandle_Type *aux_adc);
extern  void  ms_aux_adc_mask_interrupt(AuxadcHandle_Type *aux_adc);
extern  void  ms_aux_adc_unmask_interrupt(AuxadcHandle_Type *aux_adc);
extern uint8_t ms_aux_adc_get_interrupt_status(AuxadcHandle_Type *aux_adc);
extern  void  ms_aux_adc_clear_interrupt_status(AuxadcHandle_Type *aux_adc);
extern uint8_t ms_aux_adc_get_fifo_status(AuxadcHandle_Type *aux_adc);
extern uint32_t ms_aux_adc_get_fifo_data(AuxadcHandle_Type *aux_adc);
extern int32_t ms_aux_adc_deinit(AuxadcHandle_Type *aux_adc);
extern void ms_aux_adc_irq_handler(AuxadcHandle_Type *aux_adc);
extern int32_t ms_aux_adc_dma_receive_data(AuxadcHandle_Type *aux_adc, uint32_t *buffer_ptr, uint32_t size);
#ifdef __cplusplus
}
#endif

#endif /* MS_AUX_ADC_H_ */

