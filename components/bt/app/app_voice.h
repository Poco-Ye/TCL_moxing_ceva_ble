//--------------------------------------------------------------------
// Copyright (c) 2021 by MooreSilicon.
// All rights reserved.
// MooreSilicon Confidential Proprietary.
//--------------------------------------------------------------------
// Project name: Bt_soc
//    File name: ms_pwm.c
//       Author: Ethan Sun 
//        Dates: 2021-11-22
//      Version: V1.0
//-------------------------------------------------------------------
//-------------------------------------------------------------------




#ifndef _APP_VOICE_H_
#define _APP_VOICE_H_



/*
* INCLUDE FILES
****************************************************************************************
*/

#include <stdio.h>
#include "co_list.h"

extern uint8_t daw_encode_data[40];



/*
 * MACRO DEFINES
 ****************************************************************************************
 */

#define CONFIG_MIC_RATE      (AUD_SMP_RATE_16k)                   //AUD_8k,AUD_12k,AUD_16k
#define CONFIG_MIC_GAIN      (G_21db)                   //G_15db,G_0db
#define COUNT_MIC_FRAME      (1)                         //8ms for 1 frame.
#define COUNT_VOICE_FILTER   (0)                        //filter counter
#define FRAME_SIZE_PCM       (256)
//#define FRAME_SIZE_ADPCM     (32)                       
#define FRAME_SIZE_SBC       (36)                       
//#define SIZE_MIC_FRAME       (36*COUNT_MIC_FRAME)        //fr ame has 64*uin32_t
#define SIZE_MIC_FRAME       (36*COUNT_MIC_FRAME)        //fr ame has 64*uin32_t
#define SIZE_MIC_BUFFER      (SIZE_MIC_FRAME)   
//#define SIZE_MIC_BUFFER      (512)            
#define SIZE_PCM_BUFFER      (72*COUNT_MIC_FRAME)       //PCM buffer, 1 frame has 128*uin16_t

#define MIC_WORK_DURITION    (7000)
#define MIC_SWITCH_DELAY     (1)


#define MIC_LDO_GPIO          PAD23
extern unsigned int  DMAC_ADC_CH_USED;


//GATT
#define SIZE_GATT_FRAME      (SIZE_MIC_BUFFER + 6)
#define SIZE_TATT_FRAME      (FRAME_SIZE_SBC)
#define COUNT_GATT_FRAME     (1)

#define  APP_VOIVE_ADC_INIT_STEP (1)
#if APP_VOIVE_ADC_INIT_STEP
#define MIC_INIT_DELAY    (200)
#define AUD_PLL_DELAY     (10)
#endif



/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */
extern uint16_t gVoiceStartFlag;

typedef enum {
    CH_HID = 8,
    CH_RCS = 9,
    CH_GATV = 10,
    CH_TATV = 11,
}data_channel;

typedef struct __audio_buff_t
{
    uint32_t adc_data[SIZE_MIC_BUFFER];
    bool isFilled;
} audio_buff_t;

typedef struct ms_voice_data_queue
{
    /// List element header
    co_list_hdr_t hdr;
    uint16_t audio_data_len;
    uint8_t audio_buf[SIZE_TATT_FRAME];
} ms_voice_data_queue_t;


typedef struct ms_api_app_msg{
    uint8_t operation;
    void    *p_param;
	
}sonata_api_app_msg_t;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
void app_voice_init();
void app_voice_config_mic(uint8_t gain, uint8_t micBia, uint8_t audClk, uint8_t mode);


uint8_t app_voice_data_handler(void *param);
uint8_t app_voice_raw_data_handler(void *param);
uint8_t app_voice_demo_data_handler(void *param);
uint8_t app_voice_start_record(data_channel channel);
uint8_t app_voice_stop_record(void);
void app_mic_ldo_enable(void);
void app_mic_ldo_disable(void);
uint16_t app_voice_send_data(void);
void app_voice_clear_stored_queue(void);
bool app_voice_data_need_store(void);
void app_voice_reset(void);
uint8_t *app_voice_get_codec_info();
bool app_voice_adc_dma_run();

#endif //_APP_VOICE_H_
