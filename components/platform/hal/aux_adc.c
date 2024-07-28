/*
 * trng.c
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#include <ms_clock_hal.h>
#include "ms_aux_adc.h"
#include "aux_adc.h"
#include <stddef.h>
#include "log.h"
AuxadcHandle_Type aux_adc_handle;




void aux_adc_init_dma(DmacHandle_Type **phdmarx)
{
    //    *phdmarx = NULL;
    DmacHandle_Type *hdmarx;
    hdmarx = ms_dmac_mgmt_alloc();
    *phdmarx = hdmarx;

    hdmarx->init.dst_tr_width = DMAC_XFER_WIDTH_32BITS;
    hdmarx->init.dst_addr_mode = DMAC_ADDRESS_MODE_INC;
    hdmarx->init.dst_peri_type = DMAC_MEM;
    hdmarx->init.dst_msize = DMAC_MSIZE_1;
    hdmarx->init.src_peri_type = DMAC_AUX_ADC_RX;
    hdmarx->init.src_tr_width = DMAC_XFER_WIDTH_32BITS;
    hdmarx->init.src_addr_mode =DMAC_ADDRESS_MODE_NOC ;
    hdmarx->init.src_msize = DMAC_MSIZE_1;
    ms_dmac_init(hdmarx);
}



uint32_t  aux_adc_set_interrupt_flag()
{
    aux_adc_handle.interrupt_status = 1;
    return STATUS_SUCCESS;
}
uint32_t  aux_adc_clear_interrupt_flag()
{
    aux_adc_handle.interrupt_status = 0;
    return STATUS_SUCCESS;
}

uint32_t  aux_adc_get_interrupt_flag()
{
    return aux_adc_handle.interrupt_status;
}
void aux_adc_reach_calllback(AuxadcHandle_Type *aux_adc)
{
   //MS_LOGI(MS_DRIVER, "\r\naux_adc call back int !r\n");
   aux_adc_set_interrupt_flag(); 
   ms_aux_adc_clear_interrupt_status(aux_adc);
   MS_LOGI(MS_DRIVER, "\r\n int_flag = %x !\r\n",aux_adc_get_interrupt_flag());
}



void aux_adc_init_calllback(AuxadcHandle_Type *aux_adc)
{
    /*config the aux adc clock*/
	MS_CLOCK_HAL_CLK_ENABLE_AUX_ADC();

	ms_aux_adc_pwr_on();
    MS_LOGI(MS_DRIVER, "aux_adc_init_calllback!\r\n\r\n");

    /*enable the interrupt*/
    ms_aux_adc_enable_cpu_interrupt(aux_adc);
    aux_adc->init.interrupt_src = AUX_ADC_IRQMASK_ALL;
    ms_aux_adc_mask_interrupt(aux_adc);
	
    aux_adc->init.interrupt_src = AUX_ADC_IRQMASK_SMP_DONE;
    ms_aux_adc_unmask_interrupt(aux_adc);

     /*dma  init*/
    aux_adc_init_dma(&aux_adc->hdmarx);
    
    if (aux_adc->hdmarx)
    {
        aux_adc->hdmarx->parent = aux_adc;
    }
  
}

void aux_adc_deinit_calllback(AuxadcHandle_Type *aux_adc)
{
    /*disable the interrupt*/
    ms_aux_adc_disable_cpu_interrupt(aux_adc);
    aux_adc->init.interrupt_src = AUX_ADC_IRQMASK_ALL;
    ms_aux_adc_mask_interrupt(aux_adc);
   
  
    /*disable the aux adc clock*/
    MS_CLOCK_HAL_CLK_DISABLE_AUX_ADC();
    if (aux_adc->hdmarx)
    {
        ms_dmac_mgmt_free(aux_adc->hdmarx);
    }
}

AuxadcCallback_Type  aux_adc_handle_callback =
{
       .init_callback = aux_adc_init_calllback,
	.deinit_callback = aux_adc_deinit_calllback,
	.aux_adc_reach_callback = aux_adc_reach_calllback,
};

uint32_t aux_adc_init(void)
{
    aux_adc_handle.instance = AUX_ADC;
    aux_adc_handle.init.calibration_mode = MS_AUX_ADC_CALIBRATION_MODE_NORMAL;
    aux_adc_handle.init.sample_mode = MS_AUX_ADC_SAMPLE_MODE_SINGLE_ENDED;
    aux_adc_handle.init.watermask_value = 8;
   aux_adc_handle.init.interrupt_src = AUX_ADC_IRQMASK_SMP_DONE;
    aux_adc_handle.init.channel_queue_depth = 1;
    aux_adc_handle.init.channel_queue_channelx = 0;
    aux_adc_handle.init.channel_queue_idx = 0;
    aux_adc_handle.init.sample_cnt = 1;
    aux_adc_handle.init.dma_en = 1;
	
    aux_adc_handle.p_callback = &aux_adc_handle_callback;
    aux_adc_handle.irq = AUXADC_IRQn;

     MS_LOGI(MS_DRIVER, "aux_adc_init!\n");
	
    ms_aux_adc_init(&aux_adc_handle);
    return STATUS_SUCCESS;
}

uint32_t  aux_adc_enable()
{
    ms_aux_adc_enable(&aux_adc_handle);
    return STATUS_SUCCESS;
}

uint32_t  aux_adc_disable()
{
    ms_aux_adc_disable(&aux_adc_handle);
    return STATUS_SUCCESS;
}

void aux_adc_dma_enable()
{
    ms_aux_adc_dma_enable(&aux_adc_handle);
}

void aux_adc_dma_disable()
{
    ms_aux_adc_dma_disable(&aux_adc_handle);
}
	

uint32_t aux_adc_set_sample_count(uint32_t sample_count)
{
    aux_adc_handle.init.sample_cnt = sample_count;
    ms_aux_adc_set_sample_count(&aux_adc_handle);
    return STATUS_SUCCESS;
}

 uint32_t  aux_adc_set_watermask(uint32_t watermask_value)
 {
    aux_adc_handle.init.watermask_value = watermask_value;
    ms_aux_adc_set_watermask(&aux_adc_handle);
    return STATUS_SUCCESS;
 }


 uint32_t  aux_adc_set_channel_queue_depth(uint32_t queue_depth)
 {
    aux_adc_handle.init.channel_queue_depth = queue_depth;
    ms_aux_adc_set_channel_queue_depth(&aux_adc_handle);
    return STATUS_SUCCESS;
 }

uint32_t aux_adc_set_channel_queue(uint32_t channel_queue_idx,uint32_t channel_queue_channelx)
{
    aux_adc_handle.init.channel_queue_idx = channel_queue_idx;
    aux_adc_handle.init.channel_queue_channelx = channel_queue_channelx;
    ms_aux_adc_set_channel_queue(&aux_adc_handle);
    return STATUS_SUCCESS;
}


uint8_t aux_adc_get_fifo_status()
{
    return ms_aux_adc_get_fifo_status(&aux_adc_handle);
}


uint8_t aux_adc_get_interrupt_status()
{
    return ms_aux_adc_get_interrupt_status(&aux_adc_handle);
}

 uint32_t  aux_adc_clear_interrupt_status()
 {
     ms_aux_adc_clear_interrupt_status(&aux_adc_handle);
     return STATUS_SUCCESS;
 }


uint32_t aux_adc_get_data(void)
{
       return ms_aux_adc_get_fifo_data(&aux_adc_handle);
}


uint32_t aux_adc_deinit(void)
{
	ms_aux_adc_deinit(&aux_adc_handle);
	return STATUS_SUCCESS;
}

void AUXADC_IRQHandler(void)
{
    ms_aux_adc_irq_handler(&aux_adc_handle);
}

uint32_t  aux_adc_unmask_sample_done_interrupt()
{
    aux_adc_handle.init.interrupt_src = AUX_ADC_IRQMASK_SMP_DONE;
    ms_aux_adc_unmask_interrupt(&aux_adc_handle);
    return STATUS_SUCCESS;
}



uint32_t  aux_adc_mask_cpu_interrupt()
{
    aux_adc_handle.irq = AUXADC_IRQn;
    ms_aux_adc_disable_cpu_interrupt(&aux_adc_handle);
    return STATUS_SUCCESS;
}

uint32_t aux_adc_dma_receive_data( uint32_t *buffer_ptr, uint32_t size)
{
    ms_aux_adc_dma_receive_data(&aux_adc_handle, buffer_ptr,  size);
    return STATUS_SUCCESS;
}

uint32_t aux_adc_top_init(void)
{
    
     aux_adc_init();
     aux_adc_mask_cpu_interrupt();
     return STATUS_SUCCESS;
}


uint32_t aux_adc_top_get_data(void)
{
    uint32_t result = 0;
	
    aux_adc_set_channel_queue( MS_AUX_ADC_Q0,5&0x7);
    aux_adc_enable();
    
    while(AUX_ADC_IRQSTATUS_SMP_DONE != (aux_adc_get_interrupt_status()&AUX_ADC_IRQSTATUS_SMP_DONE))
    {
        ;
    }
    
    aux_adc_clear_interrupt_status();
    
    result = aux_adc_get_data();
    
    aux_adc_disable();
    
    return result;
}


