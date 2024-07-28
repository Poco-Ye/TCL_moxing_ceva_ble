/**
 *************************************************************************************
 * @file    sbc_encoder.c
 * @brief   An implementation of SBC encoder for bluetooth.
 *
 * @author  Aixing.Li
 * @version V1.0.0
 *
 * &copy; 2017 BEKEN Corporation Ltd. All rights reserved.
 *
 *************************************************************************************
 */

#include <string.h>
#include "sbc_encoder.h"
//#include "app_sbc.h"
//#include "uart.h"
//#include "app_voice.h"
#include "ms_section.h"


static const int32_t SBC_ENCODER_PROTO_4[20] =
{
    SP4(0x02cb3e8c), SP4(0x22b63dc0), SP4(0x002329cc), SP4(0x053b7548),
    SP4(0x31eab940), SP4(0xec1f5e60), SP4(0xff3773a8), SP4(0x0061c5a7),
    SP4(0x07646680), SP4(0x3f239480), SP4(0xf89f23a8), SP4(0x007a4737),
    SP4(0x00b32807), SP4(0x083ddc80), SP4(0x4825e480), SP4(0x0191e578),
    SP4(0x00ff11ca), SP4(0x00fb7991), SP4(0x069fdc58), SP4(0x4b584000)
};

static const int32_t SBC_ENCODER_PROTO_8[40] =
{
    SP8(0x02e5cd20), SP8(0x22d0c200), SP8(0x006bfe27), SP8(0x07808930),
    SP8(0x3f1c8800), SP8(0xf8810d70), SP8(0x002cfdc6), SP8(0x055acf28),
    SP8(0x31f566c0), SP8(0xebfe57e0), SP8(0xff27c437), SP8(0x001485cc),
    SP8(0x041c6e58), SP8(0x2a7cfa80), SP8(0xe4c4a240), SP8(0xfe359e4c),
    SP8(0x0048b1f8), SP8(0x0686ce30), SP8(0x38eec5c0), SP8(0xf2a1b9f0),
    SP8(0xffe8904a), SP8(0x0095698a), SP8(0x0824a480), SP8(0x443b3c00),
    SP8(0xfd7badc8), SP8(0x00d3e2d9), SP8(0x00c183d2), SP8(0x084e1950),
    SP8(0x4810d800), SP8(0x017f43fe), SP8(0x01056dd8), SP8(0x00e9cb9f),
    SP8(0x07d7d090), SP8(0x4a708980), SP8(0x0488fae8), SP8(0x0113bd20),
    SP8(0x0107b1a8), SP8(0x069fb3c0), SP8(0x4b3db200), SP8(0x00763f48)
};

static const int32_t SBC_ENCODER_ANAMATRIX_4[4] =
{
    SA4(0x2d413cc0), SA4(0x3b20d780), SA4(0x40000000), SA4(0x187de2a0)
};

static const int32_t SBC_ENCODER_ANAMATRIX_8[8] =
{
    SA8(0x3b20d780), SA8(0x187de2a0), SA8(0x3ec52f80), SA8(0x3536cc40),
    SA8(0x238e7680), SA8(0x0c7c5c20), SA8(0x2d413cc0), SA8(0x40000000)
};

static inline uint32_t sbc_abs(int32_t x)
{
    int32_t y = x - (x < 0);

    return (uint32_t)(y ^= (y >> 31));
}

static inline void _sbc_encoder_analyze_four(const int32_t* in, int32_t* out)
{
    int32_t t0, t1, t2, t3, t4, t5, t7;
    int32_t s0, s1, s2, s3, s4;

    t0  = SBC_ENCODER_PROTO_4[0] * (in[8]  - in[32]);
    t0 += SBC_ENCODER_PROTO_4[1] * (in[16] - in[24]);
    t0  = SCALE4_STAGE1(t0);

    t1  = SBC_ENCODER_PROTO_4[2] * in[1];
    t1 += SBC_ENCODER_PROTO_4[3] * in[9];
    t1 += SBC_ENCODER_PROTO_4[4] * in[17];
    t1 += SBC_ENCODER_PROTO_4[5] * in[25];
    t1 += SBC_ENCODER_PROTO_4[6] * in[33];
    t1  = SCALE4_STAGE1(t1);

    t2  = SBC_ENCODER_PROTO_4[7]  * in[2];
    t2 += SBC_ENCODER_PROTO_4[8]  * in[10];
    t2 += SBC_ENCODER_PROTO_4[9]  * in[18];
    t2 += SBC_ENCODER_PROTO_4[10] * in[26];
    t2 += SBC_ENCODER_PROTO_4[11] * in[34];
    t2  = SCALE4_STAGE1(t2);

    t3  = SBC_ENCODER_PROTO_4[12] * in[3];
    t3 += SBC_ENCODER_PROTO_4[13] * in[11];
    t3 += SBC_ENCODER_PROTO_4[14] * in[19];
    t3 += SBC_ENCODER_PROTO_4[15] * in[27];
    t3 += SBC_ENCODER_PROTO_4[16] * in[35];
    t3  = SCALE4_STAGE1(t3);

    t4  = SBC_ENCODER_PROTO_4[17] * (in[4] + in[36]);
    t4 += SBC_ENCODER_PROTO_4[18] * (in[12] + in[28]);
    t4 += SBC_ENCODER_PROTO_4[19] * in[20];
    t4  = SCALE4_STAGE1(t4);

    t5  = SBC_ENCODER_PROTO_4[16] * in[5];
    t5 += SBC_ENCODER_PROTO_4[15] * in[13];
    t5 += SBC_ENCODER_PROTO_4[14] * in[21];
    t5 += SBC_ENCODER_PROTO_4[13] * in[29];
    t5 += SBC_ENCODER_PROTO_4[12] * in[37];
    t5  = SCALE4_STAGE1(t5);

    /* don't compute t[6]... this term always multiplies with cos(pi/2) = 0 */

    t7  = SBC_ENCODER_PROTO_4[6] * in[7];
    t7 += SBC_ENCODER_PROTO_4[5] * in[15];
    t7 += SBC_ENCODER_PROTO_4[4] * in[23];
    t7 += SBC_ENCODER_PROTO_4[3] * in[31];
    t7 += SBC_ENCODER_PROTO_4[2] * in[39];
    t7  = SCALE4_STAGE1(t7);

    s0 = SBC_ENCODER_ANAMATRIX_4[0] * (t0 + t4);
    s1 = SBC_ENCODER_ANAMATRIX_4[2] * t2;
    s2 = SBC_ENCODER_ANAMATRIX_4[1] * (t1 + t3) + SBC_ENCODER_ANAMATRIX_4[3] * t5;
    s3 = SBC_ENCODER_ANAMATRIX_4[3] * (t1 + t3) + SBC_ENCODER_ANAMATRIX_4[1] * (-t5 + t7);
    s4 = SBC_ENCODER_ANAMATRIX_4[3] * t7;

    *out++ = SCALE4_STAGE2( s0 + s1 + s2 + s4);
    *out++ = SCALE4_STAGE2(-s0 + s1 + s3);
    *out++ = SCALE4_STAGE2(-s0 + s1 - s3);
    *out++ = SCALE4_STAGE2( s0 + s1 - s2 - s4);
}

static inline void sbc_encoder_analyze_four(SbcEncoderContext* sbc)
{
    int32_t ch, blk;

    for (ch = 0; ch < sbc->num_channels; ch++)
    {
        for (blk = 0; blk < sbc->frame.blocks; blk++)
        {
            int32_t* x   = &sbc->xfifo[ch][sbc->position[ch]];
            int32_t* pcm = sbc->sb_sample_f[ch][blk];

            /* Input 4 Audio Samples */
            x[40] = x[0] = pcm[3];
            x[41] = x[1] = pcm[2];
            x[42] = x[2] = pcm[1];
            x[43] = x[3] = pcm[0];

            _sbc_encoder_analyze_four(x, pcm);

            sbc->position[ch] -= 4;

            if(sbc->position[ch] < 0)
            {
                sbc->position[ch] = 36;
            }
        }
    }
}

static inline void _sbc_encoder_analyze_eight(const int32_t *in, int32_t *out)
{
    int32_t t0, t1, t2, t3, t4, t5, t6, t7;
    int32_t s0, s1, s2, s3, s4, s5, s6, s7;

    t0  = SBC_ENCODER_PROTO_8[0] * (in[16] - in[64]);
    t0 += SBC_ENCODER_PROTO_8[1] * (in[32] - in[48]);
    t0 += SBC_ENCODER_PROTO_8[2] * in[4];
    t0 += SBC_ENCODER_PROTO_8[3] * in[20];
    t0 += SBC_ENCODER_PROTO_8[4] * in[36];
    t0 += SBC_ENCODER_PROTO_8[5] * in[52];
    t0  = SCALE8_STAGE1(t0);

    t1  = SBC_ENCODER_PROTO_8[6]  * in[2];
    t1 += SBC_ENCODER_PROTO_8[7]  * in[18];
    t1 += SBC_ENCODER_PROTO_8[8]  * in[34];
    t1 += SBC_ENCODER_PROTO_8[9]  * in[50];
    t1 += SBC_ENCODER_PROTO_8[10] * in[66];
    t1  = SCALE8_STAGE1(t1);

    t2  = SBC_ENCODER_PROTO_8[11] * in[1];
    t2 += SBC_ENCODER_PROTO_8[12] * in[17];
    t2 += SBC_ENCODER_PROTO_8[13] * in[33];
    t2 += SBC_ENCODER_PROTO_8[14] * in[49];
    t2 += SBC_ENCODER_PROTO_8[15] * in[65];
    t2 += SBC_ENCODER_PROTO_8[16] * in[3];
    t2 += SBC_ENCODER_PROTO_8[17] * in[19];
    t2 += SBC_ENCODER_PROTO_8[18] * in[35];
    t2 += SBC_ENCODER_PROTO_8[19] * in[51];
    t2 += SBC_ENCODER_PROTO_8[20] * in[67];
    t2  = SCALE8_STAGE1(t2);

    t3  =  SBC_ENCODER_PROTO_8[21] * in[5];
    t3 +=  SBC_ENCODER_PROTO_8[22] * in[21];
    t3 +=  SBC_ENCODER_PROTO_8[23] * in[37];
    t3 +=  SBC_ENCODER_PROTO_8[24] * in[53];
    t3 +=  SBC_ENCODER_PROTO_8[25] * in[69];
    t3 += -SBC_ENCODER_PROTO_8[15] * in[15];
    t3 += -SBC_ENCODER_PROTO_8[14] * in[31];
    t3 += -SBC_ENCODER_PROTO_8[13] * in[47];
    t3 += -SBC_ENCODER_PROTO_8[12] * in[63];
    t3 += -SBC_ENCODER_PROTO_8[11] * in[79];
    t3  =  SCALE8_STAGE1(t3);

    t4  =  SBC_ENCODER_PROTO_8[26] * in[6];
    t4 +=  SBC_ENCODER_PROTO_8[27] * in[22];
    t4 +=  SBC_ENCODER_PROTO_8[28] * in[38];
    t4 +=  SBC_ENCODER_PROTO_8[29] * in[54];
    t4 +=  SBC_ENCODER_PROTO_8[30] * in[70];
    t4 += -SBC_ENCODER_PROTO_8[10] * in[14];
    t4 += -SBC_ENCODER_PROTO_8[9]  * in[30];
    t4 += -SBC_ENCODER_PROTO_8[8]  * in[46];
    t4 += -SBC_ENCODER_PROTO_8[7]  * in[62];
    t4 += -SBC_ENCODER_PROTO_8[6]  * in[78];
    t4  =  SCALE8_STAGE1(t4);

    t5  =  SBC_ENCODER_PROTO_8[31] * in[7];
    t5 +=  SBC_ENCODER_PROTO_8[32] * in[23];
    t5 +=  SBC_ENCODER_PROTO_8[33] * in[39];
    t5 +=  SBC_ENCODER_PROTO_8[34] * in[55];
    t5 +=  SBC_ENCODER_PROTO_8[35] * in[71];
    t5 += -SBC_ENCODER_PROTO_8[20] * in[13];
    t5 += -SBC_ENCODER_PROTO_8[19] * in[29];
    t5 += -SBC_ENCODER_PROTO_8[18] * in[45];
    t5 += -SBC_ENCODER_PROTO_8[17] * in[61];
    t5 += -SBC_ENCODER_PROTO_8[16] * in[77];
    t5  =  SCALE8_STAGE1(t5);

    t6  =  SBC_ENCODER_PROTO_8[36] * (in[8]  + in[72]);
    t6 +=  SBC_ENCODER_PROTO_8[37] * (in[24] + in[56]);
    t6 +=  SBC_ENCODER_PROTO_8[38] * in[40];
    t6 += -SBC_ENCODER_PROTO_8[39] * in[12];
    t6 += -SBC_ENCODER_PROTO_8[5]  * in[28];
    t6 += -SBC_ENCODER_PROTO_8[4]  * in[44];
    t6 += -SBC_ENCODER_PROTO_8[3]  * in[60];
    t6 += -SBC_ENCODER_PROTO_8[2]  * in[76];
    t6  =  SCALE8_STAGE1(t6);

    t7  =  SBC_ENCODER_PROTO_8[35] * in[9];
    t7 +=  SBC_ENCODER_PROTO_8[34] * in[25];
    t7 +=  SBC_ENCODER_PROTO_8[33] * in[41];
    t7 +=  SBC_ENCODER_PROTO_8[32] * in[57];
    t7 +=  SBC_ENCODER_PROTO_8[31] * in[73];
    t7 += -SBC_ENCODER_PROTO_8[25] * in[11];
    t7 += -SBC_ENCODER_PROTO_8[24] * in[27];
    t7 += -SBC_ENCODER_PROTO_8[23] * in[43];
    t7 += -SBC_ENCODER_PROTO_8[22] * in[59];
    t7 += -SBC_ENCODER_PROTO_8[21] * in[75];
    t7  =  SCALE8_STAGE1(t7);

    s0 = SBC_ENCODER_ANAMATRIX_8[0] * t0 +
         SBC_ENCODER_ANAMATRIX_8[1] * t6;
    s1 = SBC_ENCODER_ANAMATRIX_8[7] * t1;
    s2 = SBC_ENCODER_ANAMATRIX_8[2] * t2 +
         SBC_ENCODER_ANAMATRIX_8[3] * t3 +
         SBC_ENCODER_ANAMATRIX_8[4] * t5 +
         SBC_ENCODER_ANAMATRIX_8[5] * t7;
    s3 = SBC_ENCODER_ANAMATRIX_8[6] * t4;
    s4 = SBC_ENCODER_ANAMATRIX_8[3] * t2 -
         SBC_ENCODER_ANAMATRIX_8[5] * t3 -
         SBC_ENCODER_ANAMATRIX_8[2] * t5 -
         SBC_ENCODER_ANAMATRIX_8[4] * t7;
    s5 = SBC_ENCODER_ANAMATRIX_8[4] * t2 -
         SBC_ENCODER_ANAMATRIX_8[2] * t3 +
         SBC_ENCODER_ANAMATRIX_8[5] * t5 +
         SBC_ENCODER_ANAMATRIX_8[3] * t7;
    s6 = SBC_ENCODER_ANAMATRIX_8[1] * t0 -
         SBC_ENCODER_ANAMATRIX_8[0] * t6;
    s7 = SBC_ENCODER_ANAMATRIX_8[5] * t2 -
         SBC_ENCODER_ANAMATRIX_8[4] * t3 +
         SBC_ENCODER_ANAMATRIX_8[3] * t5 -
         SBC_ENCODER_ANAMATRIX_8[2] * t7;

    *out++ = SCALE8_STAGE2( s0 + s1 + s2 + s3);
    *out++ = SCALE8_STAGE2( s1 - s3 + s4 + s6);
    *out++ = SCALE8_STAGE2( s1 - s3 + s5 - s6);
    *out++ = SCALE8_STAGE2(-s0 + s1 + s3 + s7);
    *out++ = SCALE8_STAGE2(-s0 + s1 + s3 - s7);
    *out++ = SCALE8_STAGE2( s1 - s3 - s5 - s6);
    *out++ = SCALE8_STAGE2( s1 - s3 - s4 + s6);
    *out++ = SCALE8_STAGE2( s0 + s1 - s2 + s3);
}

static inline void sbc_encoder_analyze_eight(SbcEncoderContext* sbc)
{
    int32_t ch, blk;

    for (ch = 0; ch < sbc->num_channels; ch++)
    {
        for (blk = 0; blk < sbc->frame.blocks; blk++)
        {
            int32_t *x   = &sbc->xfifo[ch][sbc->position[ch]];
            int32_t *pcm = sbc->sb_sample_f[ch][blk];

            /* Input 8 Audio Samples */
            x[80] = x[0] = pcm[7];
            x[81] = x[1] = pcm[6];
            x[82] = x[2] = pcm[5];
            x[83] = x[3] = pcm[4];
            x[84] = x[4] = pcm[3];
            x[85] = x[5] = pcm[2];
            x[86] = x[6] = pcm[1];
            x[87] = x[7] = pcm[0];

            _sbc_encoder_analyze_eight(x, pcm);

            sbc->position[ch] -= 8;

            if(sbc->position[ch] < 0)
            {
                sbc->position[ch] = 72;
            }
        }
    }
}

static void sbc_encoder_subband_analyze_filter(SbcEncoderContext* sbc)
{
    switch(sbc->frame.subbands)
    {
    case 4:
        sbc_encoder_analyze_four(sbc);
        break;

    case 8:
        sbc_encoder_analyze_eight(sbc);
        break;

    default:
        break;
    }
}

/* @function sbc_encoder_init
 * @brief 初始化SBC编码器
 * @param sbc
 * @param sample_rate -- 采样率， 16000/32000/44100/48000
 * @param num_channels -- 声道， 0：单声道  1：双声道
 * @return
 * @note
 */
int32_t sbc_encoder_init(SbcEncoderContext* sbc, int32_t sample_rate, int32_t num_channels)
{
    SbcCommonContext* frame  = &sbc->frame;
    SbcFrameHeader*   header = &sbc->header;

   // memset(sbc, 0, sizeof(SbcEncoderContext));

    //Check input arguments  
    switch(sample_rate)
    {
    case 16000:				//16k
        frame->sample_rate_index = 0;
        break;
    case 32000:
        frame->sample_rate_index = 1;
        break;
    case 44100:
        frame->sample_rate_index = 2;
        break;
    case 48000:
        frame->sample_rate_index = 3;
        break;
    default:
        return SBC_ENCODER_ERROR_INVALID_SAMPLE_RATE;
    }

    switch(num_channels)
    {
    case 1:
        frame->channel_mode = 0;		//单声道
        break;
    case 2:
        frame->channel_mode = 2;
        break;
    default:
        return SBC_ENCODER_ERROR_INVALID_CHANNLES;
    }
	// 压缩数据输出量： 38bytes -header(4 bytes) =34 bytes  分两帧少1bytes  bitpool 14->15
    frame->blocks             = 16;				                                       //block number 16
    frame->allocation_method  = SBC_ALLOCATION_METHOD_LOUDNESS;  //0 
    frame->subbands           = 8;		                                                     // subbands number 8
    frame->bitpool            = 14;		                                                     //比特池。范围是2-250。此值越大，编码产生的数据越长 15
 // ------------------------------------------------------------------------------------------
    header->syncword          = SBC_SYNCWORD;
    header->sample_rate_index = frame->sample_rate_index;
    header->block_mode        = SBC_BLOCKS_16;
    header->channel_mode      = frame->channel_mode;
    header->allocation_method = frame->allocation_method;
    header->subband_mode      = SBC_SUBBANDS_8;
    header->bitpool           = frame->bitpool;
    header->crc_check         = 0;

    sbc->num_channels = num_channels;
    sbc->sample_rate  = sample_rate;
    sbc->pcm_length   = frame->blocks * frame->subbands;
    sbc->position[0]  = sbc->position[1] = 9 * frame->subbands;

    return SBC_ENCODER_ERROR_OK;
}

/* @function sbc_encoder_ioctrl
 * @brief SBC编码器输入输出控制
 * @param sbc
 * @param cmd -- 命令
 * @param arg -- 参数
 * @return
 * @note
 */
int32_t sbc_encoder_ioctrl(SbcEncoderContext* sbc, uint32_t cmd, uint32_t arg)
{
    switch(cmd)
    {
    case SBC_ENCODER_IOCTRL_CMD_SET_ALLOCATION_METHOD:
        if(arg > 1)
        {
            return SBC_ENCODER_ERROR_INVALID_IOCTRL_ARG;
        }
        sbc->frame.allocation_method  = arg;
        sbc->header.allocation_method = arg;
        break;

    case SBC_ENCODER_IOCTRL_CMD_SET_BITPOOL:
        if((sbc->frame.channel_mode == SBC_CHANNEL_MODE_MONO || sbc->frame.channel_mode == SBC_CHANNEL_MODE_DUAL_CHANNEL) && arg > (uint32_t)sbc->frame.subbands << 4)
        {
            return SBC_ENCODER_ERROR_BITPOOL_OUT_BOUNDS;
        }
        if((sbc->frame.channel_mode == SBC_CHANNEL_MODE_STEREO || sbc->frame.channel_mode == SBC_CHANNEL_MODE_JOINT_STEREO) && arg > (uint32_t)sbc->frame.subbands << 5)
        {
            return SBC_ENCODER_ERROR_BITPOOL_OUT_BOUNDS;
        }
        sbc->frame.bitpool  = arg;
        sbc->header.bitpool = arg;
        break;

    case SBC_ENCODER_IOCTRL_CMD_SET_BLOCK_MODE:
        if(arg > SBC_BLOCKS_16 && arg != 15)
        {
            return SBC_ENCODER_ERROR_INVALID_IOCTRL_ARG;
        }
        if(arg == 15)
        {
            sbc->header.syncword   = MSBC_SYNCWORD;
            sbc->header.block_mode = 3;
            sbc->frame.blocks      = 15;
        }
        else
        {
            sbc->header.syncword   = SBC_SYNCWORD;
            sbc->header.block_mode = arg;
            sbc->frame.blocks      = (arg + 1) << 2;
        }
        sbc->pcm_length = sbc->frame.blocks * sbc->frame.subbands;
        break;

    case SBC_ENCODER_IOCTRL_CMD_SET_CHANNEL_MODE:
        if(arg > 3)
        {
            return SBC_ENCODER_ERROR_INVALID_IOCTRL_ARG;
        }
        sbc->frame.channel_mode  = arg;
        sbc->header.channel_mode = arg;
        sbc->num_channels        = arg == SBC_CHANNEL_MODE_MONO ? 1 : 2;
        break;

    case SBC_ENCODER_IOCTRL_CMD_SET_SAMPLE_RATE_INDEX:
        if(arg > 3)
        {
            return SBC_ENCODER_ERROR_INVALID_IOCTRL_ARG;
        }
        sbc->frame.sample_rate_index  = arg;
        sbc->header.sample_rate_index = arg;
        sbc->sample_rate              = sbc_common_sample_rate_get(arg);
        break;

    case SBC_ENCODER_IOCTRL_CMD_SET_SUBBAND_MODE:
        if(arg > 1)
        {
            return SBC_ENCODER_ERROR_INVALID_IOCTRL_ARG;
        }
        sbc->frame.subbands      = (arg + 1) << 2;
        sbc->header.subband_mode = arg;
        sbc->pcm_length          = sbc->frame.blocks * sbc->frame.subbands;
        break;

    case SBC_ENCODER_IOCTRL_CMD_SET_MSBC_ENCODE_MODE:
        sbc->sample_rate             = 16000;
        sbc->num_channels            = 1;
        sbc->pcm_length								=128;	//120;   80
        sbc->frame.blocks            = 10;		//15;
        sbc->frame.allocation_method = SBC_ALLOCATION_METHOD_LOUDNESS;
        sbc->frame.subbands          = 8;
        sbc->frame.bitpool           = 26;
        sbc->header.syncword         = MSBC_SYNCWORD;
        sbc->header.sample_rate_index= 0;
        sbc->header.block_mode         = SBC_BLOCKS_16;
        sbc->header.channel_mode     = SBC_CHANNEL_MODE_MONO;
        sbc->header.allocation_method= SBC_ALLOCATION_METHOD_LOUDNESS;
        sbc->header.subband_mode     = SBC_SUBBANDS_8;
        sbc->header.bitpool             = 26;
        break;

    case SBC_ENCODER_IOCTRL_CMD_GET_ALLOCATION_METHOD:
        *(uint32_t*)arg = sbc->header.allocation_method;
        break;
    case SBC_ENCODER_IOCTRL_CMD_GET_BITPOOL:
        *(uint32_t*)arg = sbc->header.bitpool;
        break;
    case SBC_ENCODER_IOCTRL_CMD_GET_BLOCK_MODE:
        *(uint32_t*)arg = sbc->header.block_mode;
        break;
    case SBC_ENCODER_IOCTRL_CMD_GET_CHANNEL_MODE:
        *(uint32_t*)arg = sbc->header.channel_mode;
        break;
    case SBC_ENCODER_IOCTRL_CMD_GET_SAMPLE_RATE_INDEX:
        *(uint32_t*)arg = sbc->header.sample_rate_index;
        break;
    case SBC_ENCODER_IOCTRL_CMD_GET_SUBBAND_MODE:
        *(uint32_t*)arg = sbc->header.subband_mode;
        break;

    default:
        return SBC_ENCODER_ERROR_INVALID_IOCTRL_CMD;
    }

    return SBC_ENCODER_ERROR_OK;
}

/* @function sbc_encoder_ioctrl
 * @brief SBC帧编码
 * @param sbc
 * @param pcm -- 
 * @return
 * @note
 */
 
int32_t sbc_encoder_frame_encode(SbcEncoderContext* sbc, const int16_t* pcm, uint8_t *out)
{
    int32_t  blk, ch, sb, bit;
    uint8_t* data = sbc->stream;   //最终数据出来的 是这个stream    实际从第五帧开始为有效帧数, 前三个字节(0 1 2 4)为 的配置参数 
	
	int8_t*  bits;
    int32_t* levels;

    int32_t  produced;
    int32_t  crc_bits;

    SbcCommonContext* frame = &sbc->frame;

    for(ch = 0; ch < sbc->num_channels; ch++)
    {
        int16_t* ch_pcm = (int16_t*)pcm + ch;    
        int32_t  step   = sbc->num_channels;

        for(blk = 0; blk < frame->blocks; blk++)		// blocks = 15
        {
            int32_t* sb_sample = (int32_t*)sbc->sb_sample_f[ch][blk];

            for(sb = 0; sb < frame->subbands; sb++)
            {
                *sb_sample++ = *ch_pcm;
                ch_pcm += step;
            }
        }
    }

    sbc_encoder_subband_analyze_filter(sbc);

	SbcFrameHeader *data_temp;
	data_temp = &sbc->header;

	*(uint32_t*)data = *(uint32_t*)data_temp;

    //*(uint32_t*)data = *(uint32_t*)(&sbc->header);  //

    produced = 32;
    crc_bits = 16;

    for(ch = 0; ch < sbc->num_channels; ch++)
    {
        uint8_t* sf = (uint8_t*)frame->scale_factor[ch];

        for(sb = 0; sb < frame->subbands; sb++)
        {
            uint32_t sf2 = 2;

            sf[sb] = 0;

            for(blk = 0; blk < frame->blocks; blk++)
            {
                while(sf2 < sbc_abs(sbc->sb_sample_f[ch][blk][sb]))
                {
                    sf[sb]++;
                    sf2 *= 2;
                }
            }
        }
    }

    if(frame->channel_mode == SBC_CHANNEL_MODE_JOINT_STEREO)
    {
        /* scalefactor and scale_factor in joint case */
        uint8_t  sfj[2];
        uint32_t sfj2[2];

        /* like frame->sb_sample but joint stereo */
        int32_t sb_sample_j[16][2];

        frame->join = 0;

        for(sb = 0; sb < frame->subbands - 1; sb++)
        {
            sfj[0]  = sfj[1]  = 0;
            sfj2[0] = sfj2[1] = 2;

            for(blk = 0; blk < frame->blocks; blk++)
            {
                /* calculate joint stereo signal */
                sb_sample_j[blk][0] = (sbc->sb_sample_f[0][blk][sb] + sbc->sb_sample_f[1][blk][sb]) >> 1;
                sb_sample_j[blk][1] = (sbc->sb_sample_f[0][blk][sb] - sbc->sb_sample_f[1][blk][sb]) >> 1;

                /* calculate scale_factor_j and scalefactor_j for joint case */
                while(sfj2[0] < sbc_abs(sb_sample_j[blk][0]))
                {
                    sfj[0]++;
                    sfj2[0] *= 2;
                }

                while(sfj2[1] < sbc_abs(sb_sample_j[blk][1]))
                {
                    sfj[1]++;
                    sfj2[1] *= 2;
                }
            }

            /* decide whether to join this subband */
            if((uint32_t)(1 << frame->scale_factor[0][sb]) + (1 << frame->scale_factor[1][sb]) > (sfj2[0] + sfj2[1]) / 2)
            {
                /* use joint stereo for this subband */
                frame->join |= 1 << sb;
                frame->scale_factor[0][sb] = sfj[0];
                frame->scale_factor[1][sb] = sfj[1];

                for(blk = 0; blk < frame->blocks; blk++)
                {
                    sbc->sb_sample_f[0][blk][sb] = sb_sample_j[blk][0];
                    sbc->sb_sample_f[1][blk][sb] = sb_sample_j[blk][1];
                }
            }
        }

        data[4] = 0;	//从第四个字节开始为真正数据
        for(sb = 0; sb < frame->subbands - 1; sb++)
        {
            data[4] |= ((frame->join >> sb) & 0x01) << (frame->subbands - 1 - sb);
        }

        produced += frame->subbands;
        crc_bits += frame->subbands;
    }

    for(ch = 0; ch < sbc->num_channels; ch++)
    {
        for(sb = 0; sb < frame->subbands; sb++)
        {
            data[produced >> 3] <<= 4;
            data[produced >> 3]  |= frame->scale_factor[ch][sb] & 0x0F;

            produced += 4;
            crc_bits += 4;
        }
    }

    sbc_common_bit_allocation(frame);

    for(ch = 0; ch < sbc->num_channels; ch++)
    {
        bits   = frame->bits[ch];
        levels = frame->mem[ch];

        for(sb = 0; sb < frame->subbands; sb++)
        {
            levels[sb] = (1 << bits[sb]) - 1;
        }
    }

    for(blk = 0; blk < frame->blocks; blk++)
    {
        for(ch = 0; ch < sbc->num_channels; ch++)
        {
            bits   = frame->bits[ch];
            levels = frame->mem[ch];

            for(sb = 0; sb < frame->subbands; sb++)
            {
                if(levels[sb] > 0)
                {
                    uint16_t audio_sample;

                    audio_sample   = (uint16_t)((((sbc->sb_sample_f[ch][blk][sb] * levels[sb]) >> (frame->scale_factor[ch][sb] + 1)) + levels[sb]) >> 1);
                    audio_sample <<= 16 - bits[sb];

                    for(bit = 0; bit < bits[sb]; bit++)
                    {
                        data[produced >> 3] <<= 1;

                        if(audio_sample & 0x8000)
                        {
                            data[produced >> 3] |= 0x1;
                        }

                        audio_sample <<= 1;
                        produced++;
                    }
                }
            }
        }
    }

    data[3] = sbc_common_crc8(data + 1, crc_bits);  //第四个字节  crc_check 
		
		
		
    /* align the last byte */
    if(produced & 7)
    {
        data[produced >> 3] <<= 8 - (produced & 7);
    }

#if (1)

			//返回的数据 就是需要上传的原始数据
		for(uint8_t index=0;index<40;index++)
		{
		    out[index]=data[index];
		}

#endif
		
		
    return (produced + 7) >> 3;
}
