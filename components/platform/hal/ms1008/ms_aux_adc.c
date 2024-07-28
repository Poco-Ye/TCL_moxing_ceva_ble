/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_aux_adc_.c
 * @brief c source  file of timer  module.
 * @author haijun.mai
 * @date   2022-01-25
 * @version 1.0
 * @Revision
 */
#include "ms_aux_adc_hal.h"
#include "ms_aux_adc.h"
#include "ms_aux_adc_regs.h"
#include <ms_clock_hal.h>
#include "ms_sys_ctrl_regs.h"
#include "ms1008.h"
#include "ms_interrupt.h"
#include <stddef.h>
#include "log.h"
/**
 * @brief  Enable aux adc Interrupt 
 * @param  AuxadcHandle_Type *aux_adc: aux_adc Instance
 * @retval none
 */
void ms_aux_adc_enable_cpu_interrupt(AuxadcHandle_Type *aux_adc)
{
    INTERRUPT_ENABLE_IRQ(aux_adc->irq);
}

/**
 * @brief  Disable aux adc Interrupt 
 * @param  AuxadcHandle_Type *aux_adc: aux_adc Instance
 * @retval none
 */
void ms_aux_adc_disable_cpu_interrupt(AuxadcHandle_Type *aux_adc)
{
    INTERRUPT_DISABLE_IRQ(aux_adc->irq);
}



/**
 * init aux adc
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
int32_t ms_aux_adc_init(AuxadcHandle_Type *aux_adc)
{
    if(NULL == aux_adc)
    {
         return STATUS_ERROR;
    }
    /*initial uart gpio pinmux, clock and interrupt setting*/
    if (aux_adc->p_callback && aux_adc->p_callback->init_callback)
    {
        aux_adc->p_callback->init_callback(aux_adc);
    }

    ms_aux_adc_fifo_flush_hal(aux_adc->instance);
	
    ms_aux_adc_set_sample_mode_hal(aux_adc->instance,aux_adc->init.sample_mode);

    ms_aux_adc_set_calibration_mode_hal(aux_adc->instance,aux_adc->init.calibration_mode);

    ms_aux_adc_set_calibration_value_hal(aux_adc->instance,aux_adc->init.calibration_value);

    
    ms_aux_adc_set_sample_count_hal(aux_adc->instance, aux_adc->init.sample_cnt);

    ms_aux_adc_set_watermask_hal(aux_adc->instance, aux_adc->init.watermask_value);
   
    ms_aux_adc_set_channel_queue_depth_hal(aux_adc->instance,aux_adc->init.channel_queue_depth);

   


    MS_LOGI(MS_DRIVER, "CTR = %x\r\n",READ_REG(aux_adc->instance->CTR));
    MS_LOGI(MS_DRIVER, "CHNQUEUE = %x\r\n",READ_REG(aux_adc->instance->CHNQUEUE));
    MS_LOGI(MS_DRIVER, "FIFOSTATUS = %x\r\n",READ_REG(aux_adc->instance->FIFOSTATUS));
    MS_LOGI(MS_DRIVER, "FIFODATA = %x\r\n",READ_REG(aux_adc->instance->FIFODATA));
    MS_LOGI(MS_DRIVER, "IRQMASK = %x\r\n",READ_REG(aux_adc->instance->IRQMASK));
    MS_LOGI(MS_DRIVER, "SMPCNT = %x\n",READ_REG(aux_adc->instance->SMPCNT));
    MS_LOGI(MS_DRIVER, "WATERMARK = %x\r\n",READ_REG(aux_adc->instance->WATERMARK));
    
    return STATUS_SUCCESS;

}

/**
 * enable aux adc
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
void  ms_aux_adc_enable(AuxadcHandle_Type *aux_adc)
{
     ms_aux_adc_enable_hal(aux_adc->instance);
     MS_LOGI(MS_DRIVER, "adc enable CTR = %x\r\n",READ_REG(aux_adc->instance->CTR));
}

/**
 * disable aux adc
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
void  ms_aux_adc_disable(AuxadcHandle_Type *aux_adc)
{
     ms_aux_adc_disable_hal(aux_adc->instance);
      MS_LOGI(MS_DRIVER, "adc disable CTR = %x\r\n",READ_REG(aux_adc->instance->CTR));
}

/**
 *  aux adc dma enable
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
void ms_aux_adc_dma_enable(AuxadcHandle_Type *aux_adc)
{
    ms_aux_adc_dma_enable_hal(aux_adc->instance);
}

/**
 *  aux adc dma disable
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
void ms_aux_adc_dma_disable(AuxadcHandle_Type *aux_adc)
{
    ms_aux_adc_dma_disable_hal(aux_adc->instance);
}

/**
 *  aux adc fifo flush
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
void ms_aux_adc_fifo_flush(AuxadcHandle_Type *aux_adc)
{
    ms_aux_adc_fifo_flush_hal(aux_adc->instance);
}


/**
 *  aux adc set sample mode
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
void ms_aux_adc_set_sample_mode(AuxadcHandle_Type *aux_adc)
{
    ms_aux_adc_set_sample_mode_hal(aux_adc->instance, aux_adc->init.sample_mode);
}


/**
 *  aux adc set calibration mode
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
void ms_aux_adc_set_calibration_mode(AuxadcHandle_Type *aux_adc)
{
    ms_aux_adc_set_calibration_mode_ll(aux_adc->instance, aux_adc->init.calibration_mode);
}



/**
 *  aux adc set calibration value
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
void ms_aux_adc_set_calibration_value(AuxadcHandle_Type *aux_adc)
{
    ms_aux_adc_set_calibration_value_hal(aux_adc->instance, aux_adc->init.calibration_value);
}


/**
 *  aux adc set sample count
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
void ms_aux_adc_set_sample_count(AuxadcHandle_Type *aux_adc)
{
    ms_aux_adc_set_sample_count_hal(aux_adc->instance, aux_adc->init.sample_cnt);
    MS_LOGI(MS_DRIVER, "SMPCNT = %x\r\n",READ_REG(aux_adc->instance->SMPCNT));
    MS_LOGI(MS_DRIVER, "SMPCNTR = %x\r\n",READ_REG(aux_adc->instance->SMPCNTR));
}

/**
 *  aux adc get already sample count
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
uint16_t ms_aux_adc_get_already_sample_count(AuxadcHandle_Type *aux_adc)
{
    return ms_aux_adc_get_already_sample_count_hal(aux_adc->instance);
}


/**
 *  aux adc set set watermask 
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
 void  ms_aux_adc_set_watermask(AuxadcHandle_Type *aux_adc)
{
     MS_LOGI(MS_DRIVER, " WATERMARK = %x\r\n",aux_adc->init.watermask_value);
     ms_aux_adc_set_watermask_hal(aux_adc->instance, aux_adc->init.watermask_value);
     MS_LOGI(MS_DRIVER, "set_watermask WATERMARK = %x\r\n",READ_REG(aux_adc->instance->WATERMARK));
}


/**
 *  aux adc set set channel queue depth 
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
 void  ms_aux_adc_set_channel_queue_depth(AuxadcHandle_Type *aux_adc)
{
    ms_aux_adc_set_channel_queue_depth_hal(aux_adc->instance, aux_adc->init.channel_queue_depth);
    MS_LOGI(MS_DRIVER, "set_channel_queue_depth CHNQUEUE = %x    channel_queue_depth =  %d\r\n",READ_REG(aux_adc->instance->CHNQUEUE),aux_adc->init.channel_queue_depth);
}


/**
 *  aux adc set set channel queue  
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
 void  ms_aux_adc_set_channel_queue(AuxadcHandle_Type *aux_adc)
{
    ms_aux_adc_set_channel_queue_hal(aux_adc->instance, aux_adc->init.channel_queue_idx, aux_adc->init.channel_queue_channelx);
    MS_LOGI(MS_DRIVER, "set_channel_queue     queue_idx = %d  queue_channel =%d CHNQUEUE = %x\r\n", aux_adc->init.channel_queue_idx,aux_adc->init.channel_queue_channelx,READ_REG(aux_adc->instance->CHNQUEUE));
}


/**
 *  aux adc mask_interrupt  
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
 void  ms_aux_adc_mask_interrupt(AuxadcHandle_Type *aux_adc)
{
	ms_aux_adc_mask_interrupt_hal(aux_adc->instance, aux_adc->init.interrupt_src);
}


/**
 *  aux adc unmask_interrupt  
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
 void  ms_aux_adc_unmask_interrupt(AuxadcHandle_Type *aux_adc)
{
	ms_aux_adc_unmask_interrupt_hal(aux_adc->instance, aux_adc->init.interrupt_src);
}



/**
 *  aux adc get interrupt status
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
uint8_t ms_aux_adc_get_interrupt_status(AuxadcHandle_Type *aux_adc)
{
    // MS_LOGI(MS_DRIVER, "IRQSTATUS = %x\r\n",READ_REG(aux_adc->instance->IRQSTATUS));
    return ms_aux_adc_get_interrupt_status_hal(aux_adc->instance) ;
}


/**
 *  aux adc unmask_interrupt  
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
 void  ms_aux_adc_clear_interrupt_status(AuxadcHandle_Type *aux_adc)
{
	ms_aux_adc_clear_interrupt_status_hal(aux_adc->instance, aux_adc->init.interrupt_src);
}




/**
 *  aux adc get fifo status
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
uint8_t ms_aux_adc_get_fifo_status(AuxadcHandle_Type *aux_adc)
{
   // MS_LOGI(MS_DRIVER, "fifo_status = %x\r\n",READ_REG(aux_adc->instance->FIFOSTATUS));
    return ms_aux_adc_get_fifo_status_hal(aux_adc->instance);
}


/**
 *  aux adc get fifo data
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
uint32_t ms_aux_adc_get_fifo_data(AuxadcHandle_Type *aux_adc)
{
     //MS_LOGI(MS_DRIVER, "get FIFODATA = %x\r\n",READ_REG(aux_adc->instance->FIFODATA));
    return ms_aux_adc_get_fifo_data_hal(aux_adc->instance);
}




/**
 * deinit aux adc
 *
 * @param[in]  AuxadcHandle_Type *aux_adc
 */
int32_t ms_aux_adc_deinit(AuxadcHandle_Type *aux_adc)
{
     if(NULL == aux_adc)
    {
        return -1;
    }
	   /* DeInit the low level hardware */
    if (aux_adc->p_callback && aux_adc->p_callback->deinit_callback)
    {
        aux_adc->p_callback->deinit_callback(aux_adc);
    }
    return 0;

}






/**
 * @brief  This function handles aux_adc interrupt request.
 * @param  param  AuxadcHandle_Type *aux_adc:
 * @retval None
 */
void ms_aux_adc_irq_handler(AuxadcHandle_Type *aux_adc)
{
	
    if (aux_adc->p_callback == NULL)
    {
        aux_adc->error_code |= AUX_ADC_ERROR_INVALID_CALLBACK;
        return;
    }
    aux_adc->p_callback->aux_adc_reach_callback(aux_adc);
}


static void ms_aux_adc_dma_rx_xfer_tfr_callback(DmacHandle_Type *hdma)
{
    CHECK_PTR_NULL(hdma);

}
static void ms_aux_adc_dma_rx_xfer_error_callback(DmacHandle_Type *hdma)
{
    CHECK_PTR_NULL(hdma);

}
int32_t ms_aux_adc_dma_receive_data(AuxadcHandle_Type *aux_adc, uint32_t *buffer_ptr, uint32_t size)
{
    CHECK_PTR_NULL_RET(aux_adc, STATUS_ERROR);
    CHECK_PTR_NULL_RET(aux_adc->hdmarx, STATUS_ERROR);
    CHECK_PTR_NULL_RET(buffer_ptr, STATUS_ERROR);

 
    aux_adc->hdmarx->xfer_tfr_callback = ms_aux_adc_dma_rx_xfer_tfr_callback;
    aux_adc->hdmarx->xfer_error_callback = ms_aux_adc_dma_rx_xfer_error_callback;
    uint32_t *tmp = (uint32_t*) &buffer_ptr;
    ms_dmac_start_chx_xfer_it(aux_adc->hdmarx, (uint32_t) &aux_adc->instance->FIFODATA, *(uint32_t*) tmp, size);

    return STATUS_SUCCESS;
}





