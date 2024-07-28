/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  ms_audio_adc.h
 * @brief
 * @author bingrui.chen
 * @date 2022年1月14日
 * @version 1.0
 * @Revision:
 */
#ifndef MS_AUDIO_ADC_H_
#define MS_AUDIO_ADC_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "ms_single_dmac.h"
#include "ms_audio_adc_hal.h"
#include "ms_queue.h"
#include "sbc_encoder.h"

#define AUDIO_BUFFER_MAX (256)
#define AUDIO_QUEUE_MAX (8)
#define AUDIO_FIFO_MAX     0x00000010UL


/**
 * @brief  Audio ADC Callback ID Structure Definition
 */
typedef enum
{
    AUDIO_ADC_CB_ID_INIT,
    AUDIO_ADC_CB_ID_DEINIT,
    AUDIO_ADC_CB_ID_CPLT,
    AUDIO_ADC_CB_ID_BUF_FULL
} AudioAdcCbId_Type;


/**
 * @brief  Audio ADC Handle Structure Definition
 */
typedef struct __AudioADCHandle_Type
{
    uint8_t *buffer;
    uint16_t data_len;
    uint32_t water_level;
    uint32_t int_mask;

    AudioAdc_Type *instance;    /*!< Audio ADC registers base address*/
    QueueHandle_t queue;   /*!< Audio ADC RingBuffer    */
    DmacHandle_Type *hdma;          /*!< Pointer to the DMA  */
    SbcEncoderContext *sbc;
    uint32_t sbc_sample_rate;
    uint32_t sbc_num_channels;
    int32_t (*sbc_init)(SbcEncoderContext* sbc, int32_t sample_rate, int32_t num_channels);
    void  (*audio_callback)(void* audio_buff,  uint16_t len);
} AudioADCHandle_Type;

/**
 * @brief  Initial Audio ADC peripheral
 * @param[in]  haudio_adc:  Pointer to a AudioADCHandle_Type structure that contains
 *                the configuration information for the specified Audio ADC module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_audio_adc_init(AudioADCHandle_Type *haudio_adc);
/**
 * @brief  DeInitializes the Audio ADC peripheral
 * @param[in]  haudio_adc:  Pointer to a AudioADCHandle_Type structure that contains
 *                the configuration information for the specified Audio ADC module.
 * @retval none
 */
void ms_audio_adc_deinit(AudioADCHandle_Type *haudio_adc);

void ms_audio_adc_default_config(AudioADCHandle_Type *haudio_adc);
/**
 * @brief  Register a Audio ADC DMA Mode Transmission
 * @param[in]  haudio_adc:  Pointer to a AudioADCHandle_Type structure that contains
 *                the configuration information for the specified Audio ADC module.
 * @param[in]  pdata: Pointer to the DMA Buffer.
 * @param[in]  len:  the length of the DMA Buffer.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_audio_adc_register_dma_trans(AudioADCHandle_Type *haudio_adc, uint8_t *pdata,uint16_t len);

/**
 * @brief  Stop the Audio ADC DMA Mode Transmission
 * @param[in]  haudio_adc:  Pointer to a AudioADCHandle_Type structure that contains
 *                the configuration information for the specified Audio ADC module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_audio_adc_stop_dma(void);

/**
 * @brief  Clear Audio ADC FIFO
 * @param[in]  haudio_adc:  Pointer to a AudioADCHandle_Type structure that contains
 *                the configuration information for the specified Audio ADC module.
 * @retval none
 */
void ms_audio_adc_clear_fifo(AudioADCHandle_Type *haudio_adc);

void ms_audio_adc_default_config(AudioADCHandle_Type *haudio_adc);
int32_t ms_audio_adc_start_dma_xfer(void);
void  audio_callback_just_for_test(void* audio_buff,  uint16_t len);


#endif /* MS_AUDIO_ADC_H_ */
