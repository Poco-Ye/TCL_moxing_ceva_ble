/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  ms_audio_adc.c
 * @brief
 * @author bingrui.chen
 * @date 2022-1-14
 * @version 1.0
 * @Revision:
 */

#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "unity.h"
#include "ms_audio_adc.h"
#include "sbc_encoder.h"
#include "log.h"

static AudioADCHandle_Type haudio_adc;

static AudioADCHandle_Type *phaudio_adc = NULL;

SbcEncoderContext adc_sbc;
uint8_t audio_buffer[AUDIO_BUFFER_MAX];
QueueHandle_t AudQueue;


static void ms_audio_adc_dma_xfer_error_callback(DmacHandle_Type *hdma)
{
    CHECK_PTR_NULL(hdma);
    
}

int32_t ms_audio_adc_start_dma_xfer(void)
{
    uint32_t src_addr = (uint32_t)(&(phaudio_adc->instance->RDATA));
    uint32_t dst_addr = (uint32_t)phaudio_adc->buffer;
    uint16_t count = phaudio_adc->data_len;

    CHECK_PTR_NULL_RET(phaudio_adc, STATUS_ERROR);

    /*initial interrupt*/
    ms_audio_adc_hal_enable_cpu_int();
    ms_audio_adc_hal_enable_int();

    return ms_dmac_start_chx_xfer_it(phaudio_adc->hdma, src_addr,dst_addr, count);
}

static void ms_audio_adc_dma_xfer_tft_callback(DmacHandle_Type *hdma)
{
    CHECK_PTR_NULL(hdma);
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;

    AudioADCHandle_Type *haudio_adc = hdma->parent;

    hdma->busy[hdma->channel_index] = false;

    if(xQueueSendFromISR(haudio_adc->queue, (void *)(haudio_adc->buffer), &xHigherPriorityTaskWoken) == pdPASS)
    {
        ms_audio_adc_start_dma_xfer();
        return;
    }
    xQueueReset(phaudio_adc->queue);

    MS_LOGI(MS_DRIVER, " queue err,queue reset! !\r\n");

    ms_audio_adc_start_dma_xfer();
}


void ms_audio_adc_default_config(AudioADCHandle_Type *haudio_adc)
{
    DmacHandle_Type *hdma;
    hdma = ms_dmac_mgmt_alloc();

    TEST_ASSERT_NOT_NULL(hdma);

    haudio_adc->hdma = hdma;
    haudio_adc->buffer = audio_buffer;
    haudio_adc->data_len = AUDIO_BUFFER_MAX;
    haudio_adc->int_mask = 0x1F;
    haudio_adc->water_level = AUDIO_FIFO_MAX / 2;
    haudio_adc->instance = ms_audio_adc_hal_get_instance();
    haudio_adc->queue = xQueueCreate(AUDIO_QUEUE_MAX, haudio_adc->data_len);
    haudio_adc->audio_callback = audio_callback_just_for_test;
    haudio_adc->sbc = &adc_sbc;
    haudio_adc->sbc_sample_rate = 16000;
    haudio_adc->sbc_num_channels = 1;
    haudio_adc->sbc_init = sbc_encoder_init;

    hdma->parent = haudio_adc;
    hdma->xfer_tfr_callback = ms_audio_adc_dma_xfer_tft_callback;
    hdma->xfer_error_callback = ms_audio_adc_dma_xfer_error_callback;

    hdma->init.dst_tr_width = DMAC_XFER_WIDTH_32BITS;
    hdma->init.dst_addr_mode = DMAC_ADDRESS_MODE_INC;
    hdma->init.dst_peri_type = DMAC_MEM;

    hdma->init.dst_msize = DMAC_MSIZE_32;
    hdma->init.src_peri_type = DMAC_AUDIO_ADC_RX;
    hdma->init.src_tr_width = DMAC_XFER_WIDTH_16BITS;
    hdma->init.src_addr_mode = DMAC_ADDRESS_MODE_NOC;
    hdma->init.src_msize = DMAC_MSIZE_32;
    hdma->busy[hdma->channel_index] = false;

}



int32_t ms_audio_adc_init(AudioADCHandle_Type *haudio_adc)
{
    CHECK_PTR_NULL_RET(haudio_adc, STATUS_ERROR);
    phaudio_adc = haudio_adc;

    if(haudio_adc->sbc_init)
    {
        haudio_adc->sbc_init(haudio_adc->sbc, haudio_adc->sbc_sample_rate, haudio_adc->sbc_num_channels);
    }
    /*initial the queue buffer*/
    /*todo: configure audio adc parameters*/
    ms_audio_adc_hal_set_water_level(haudio_adc->water_level);
	
    /*Attention: Interrupt enable also control the data switch.*/
    ms_audio_adc_hal_mask_int(haudio_adc->int_mask);
	
    /*initial clock*/
    ms_audio_adc_hal_enable_clk();
    ms_audio_adc_hal_enable_power();

    ms_dmac_init(haudio_adc->hdma);

    /*Allocation and Initial DMA*/
    ms_audio_adc_hal_enable_dma();

    return STATUS_SUCCESS;
}

void ms_audio_adc_deinit(AudioADCHandle_Type *haudio_adc)
{

}


int32_t ms_audio_adc_stop_dma(void)
{
    CHECK_PTR_NULL_RET(phaudio_adc, STATUS_ERROR);

    ms_dmac_stop_chx_xfer(phaudio_adc->hdma);

    ms_audio_adc_hal_disable_cpu_int();
    ms_audio_adc_hal_disable_int();

    return STATUS_SUCCESS;
}


void ms_audio_adc_clear_fifo(AudioADCHandle_Type *haudio_adc)
{
    ms_audio_adc_hal_active_flush_fifo();
    ms_audio_adc_hal_inactive_flush_fifo();
}


void ms_audio_adc_irq(AudioADCHandle_Type *haudio_adc)
{
    ms_audio_adc_hal_disable_int();
    if (ms_audio_adc_hal_fifo_is_full())
    {
        ms_audio_adc_clear_fifo(haudio_adc);
    }

    if (ms_audio_adc_hal_fifo_is_empty())
    {
        /*todo:nothing to do*/
    }

    if (ms_audio_adc_hal_fifo_is_water())
    {
        /*todo:read data from fifo*/
    }

    if (ms_audio_adc_hal_fifo_is_overflow())
    {
        ms_audio_adc_clear_fifo(haudio_adc);
    }

    if (ms_audio_adc_hal_fifo_is_underflow())
    {
        ms_audio_adc_clear_fifo(haudio_adc);
    }

    ms_audio_adc_hal_enable_int();
}


void AUDADC_IRQHandler(void)
{
    ms_audio_adc_irq(phaudio_adc);
}

void ms_audio_module_init(void)
{
    ms_audio_adc_default_config(&haudio_adc);
    ms_audio_adc_init(&haudio_adc);
}

uint8_t  audio_thread(void)
{
    uint16_t buff[AUDIO_BUFFER_MAX];

    //memset(buff, 0 , AUDIO_BUFFER_MAX);
    if(xQueueReceive(phaudio_adc->queue, (void *)buff ,0) == pdPASS)
    {
       if(phaudio_adc->audio_callback)
       {
           phaudio_adc->audio_callback(buff, phaudio_adc->data_len);
	}
	return pdPASS;
    }
    return pdFAIL;
}



void  audio_callback_just_for_test(void* audio_buff,  uint16_t len)
{
    uint8_t sbc_data[40];

    for(int i=0;i<AUDIO_BUFFER_MAX;i+=128)
    {
       sbc_encoder_frame_encode(phaudio_adc->sbc , (const uint16_t *)audio_buff, sbc_data); //just test
#ifdef BT_SUPPORT
       extern void gatvs_send_data_via_rx_ntf(uint16_t data_len, uint8_t *p_data);
       
	gatvs_send_data_via_rx_ntf(128, audio_buff);
#else
       static uint32_t count = 0;
       MS_LOGI(MS_DRIVER, "count=%d\r\n",count+=AUDIO_BUFFER_MAX);
#endif
       audio_buff+=32;
    }
}

void gatvs_audio_stop(void)
{
      ms_audio_adc_stop_dma();
      xQueueReset(phaudio_adc->queue);
     MS_LOGI(MS_DRIVER, "gatvs_audio_stop\r\n");
}

