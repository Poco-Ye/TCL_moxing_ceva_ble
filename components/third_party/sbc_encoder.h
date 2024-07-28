/**
 *************************************************************************************
 * @file    sbc_encoder.h
 * @brief   An implementation of SBC encoder for bluetooth.
 *
 * @author  Aixing.Li
 * @version V1.0.0
 *
 * &copy; 2017 BEKEN Corporation Ltd. All rights reserved.
 *
 *************************************************************************************
 */
#ifndef _SBC_ENCODER_
#define _SBC_ENCODER_
#include "sbc_common.h"

/**
 * SBC encoder error code
 */
typedef enum _SBC_ENCODER_ERROR_CODE
{
    SBC_ENCODER_ERRORS = -128,
    SBC_ENCODER_ERROR_INVALID_SAMPLE_RATE,      /**< invalid sample rate   */
    SBC_ENCODER_ERROR_INVALID_CHANNLES,         /**< invalid channels      */
    SBC_ENCODER_ERROR_INVALID_IOCTRL_CMD,       /**< invalid ioctrl cmd    */
    SBC_ENCODER_ERROR_INVALID_IOCTRL_ARG,       /**< invalid ioctrl arg    */
    SBC_ENCODER_ERROR_BITPOOL_OUT_BOUNDS,       /**< bitpool out of bounds */

    SBC_ENCODER_ERROR_OK = 0,                   /**< no error              */

}SBC_ENCODER_ERROR_CODE;

typedef enum _SBC_ENCODER_IOCTRL_CMD
{
    SBC_ENCODER_IOCTRL_CMD_SET_ALLOCATION_METHOD,    /**< arg: 0:loundness, 1:SNR                       */
    SBC_ENCODER_IOCTRL_CMD_SET_BITPOOL,              /**< arg: 2-250                                    */
    SBC_ENCODER_IOCTRL_CMD_SET_BLOCK_MODE,           /**< arg: 0:4, 1:8, 2:12, 3:16                     */
    SBC_ENCODER_IOCTRL_CMD_SET_CHANNEL_MODE,         /**< arg: 0:MONO, 1:DUAL, 2:STEREO, 3:JOINT STEREO */
    SBC_ENCODER_IOCTRL_CMD_SET_SAMPLE_RATE_INDEX,    /**< arg: 0:16000, 1:32000, 2:44100, 3:48000       */
    SBC_ENCODER_IOCTRL_CMD_SET_SUBBAND_MODE,         /**< arg: 0:4, 1:8                                 */
    SBC_ENCODER_IOCTRL_CMD_SET_MSBC_ENCODE_MODE,     /**< arg: NULL                                     */

    SBC_ENCODER_IOCTRL_CMD_GET_ALLOCATION_METHOD,    /**< get allcation method                          */
    SBC_ENCODER_IOCTRL_CMD_GET_BITPOOL,              /**< get bitpool value                             */
    SBC_ENCODER_IOCTRL_CMD_GET_BLOCK_MODE,           /**< get block mode                                */
    SBC_ENCODER_IOCTRL_CMD_GET_CHANNEL_MODE,         /**< get channel mode                              */
    SBC_ENCODER_IOCTRL_CMD_GET_SAMPLE_RATE_INDEX,    /**< get sample rate index                         */
    SBC_ENCODER_IOCTRL_CMD_GET_SUBBAND_MODE,         /**< get sunband mode                              */

}SBC_ENCODER_IOCTRL_CMD;

/**
 * @brief SBC encoder context
 */
typedef struct _SbcEncoderContext
{
    SbcCommonContext frame;
    SbcFrameHeader   header;

    int8_t   num_channels;              /**< channels number    */
    uint8_t  pcm_length;                /**< PCM length         */
    uint16_t sample_rate;               /**< sample rate        */

    int32_t  sb_sample_f[2][16][8];     /**< subband sample     */
    uint8_t  stream[512];               /**< encoded buffer     */

    int32_t  position[2];
    int32_t  xfifo[2][160];

}SbcEncoderContext;

/**
 * @brief  SBC encoder initialzie
 * @param  sbc          SBC encoder context pointer
 * @param  sample_rate  sample rate
 * @param  num_channels number of channels
 * @return error code, @see SBC_ENCODER_ERROR_CODE
 */
int32_t sbc_encoder_init(SbcEncoderContext* sbc, int32_t sample_rate, int32_t num_channels);

/**
 * @brief  SBC encoder parameters config
 * @param  sbc SBC encoder context pointer
 * @param  cmd @see SBC_ENCODER_IOCTRL_CMD
 * @param  arg the argument or result adress for the cmd
 * @return error code, @see SBC_ENCODER_ERROR_CODE
 */
int32_t sbc_encoder_ioctrl(SbcEncoderContext* sbc, uint32_t cmd, uint32_t arg);

/**
 * @brief  SBC encoder encode one frame
 * @param  sbc SBC encoder context pointer
 * @param  pcm input PCM samples to be encoded,
 *         the number of input PCM samples MUST be sbc->pcm_length !!!
 * @return encoded buffer length by encoder if no error ocurs,
 *         else error code (always small than 0) will be return, @see SBC_ENCODER_ERROR_CODE
 *         the output encoded buffer refer to sbc->stream.
 */
int32_t sbc_encoder_frame_encode(SbcEncoderContext* sbc, const int16_t* pcm, uint8_t *out);


#endif

