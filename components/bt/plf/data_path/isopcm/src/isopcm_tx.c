/**
 ****************************************************************************************
 *
 * @file isopcm.h
 *
 * @brief Declaration of the functions used for Isochronous Payload over PCM TX path
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup ISOPCM
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_ISO_PCM)

#include "isopcm_int.h"

#include "dbg_swdiag.h"
#include "ke_mem.h"
#include "ke_event.h"

#include "co_list.h"
#include "co_math.h"
#include "co_utils.h"
#include "co_bt.h"
#include "arch.h"
#include "pcm.h"

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

__STATIC uint8_t isopcm_tx_start(uint16_t conhdl, uint32_t sdu_interval, uint32_t trans_latency, uint16_t max_sdu);
__STATIC void isopcm_tx_stop(uint16_t conhdl, uint8_t reason);
__STATIC uint8_t* isopcm_tx_sdu_get (uint16_t conhdl, uint32_t* p_ref_time, uint16_t* p_sdu_len);
__STATIC void isopcm_tx_sdu_done(uint16_t conhdl, uint16_t sdu_len, uint32_t ref_time, uint8_t* p_buf, uint8_t status);


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// PCM TX interface
const struct data_path_itf isopcm_tx_itf =
{
    isopcm_tx_start,
    isopcm_tx_stop,
    isopcm_tx_sdu_get,
    isopcm_tx_sdu_done,
    NULL,
    NULL,
};

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * Handle background process to toggle to next transmission buffer
 *
 * Idea is using same PCM buffer for several streams
 * @param[in] p_djob  Pointer to Delayed job structure.
 ****************************************************************************************
 */
__STATIC void isopcm_tx_djob_handler(co_djob_t* p_djob)
{
    GLOBAL_INT_DISABLE();

    struct isopcm_tx_info* p_tx_env = isopcm_env.p_tx;
    if(p_tx_env)
    {
        DBG_SWDIAG(ISOPCM, TX, 1);
        // consider TX buffer done, prepare to use next one
        CO_VAL_INC(p_tx_env->prog_idx, p_tx_env->nb_buffer);
        p_tx_env->data_loading = false;
        DBG_SWDIAG(ISOPCM, TX, 0);
    }

    GLOBAL_INT_RESTORE();
}

__STATIC uint8_t isopcm_tx_start(uint16_t conhdl, uint32_t sdu_interval, uint32_t trans_latency, uint16_t max_sdu)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    struct isopcm_tx_info* p_tx_env = isopcm_env.p_tx;

    uint8_t nb_buffer = CO_DIVIDE_CEIL(trans_latency, sdu_interval);

    DBG_SWDIAG(ISOPCM, TX, 1);

    // Check parameters
    if((max_sdu >= 65) || ((max_sdu == 0) || (sdu_interval == 0)) || (channel >= BLE_ACTIVITY_MAX))
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else if(p_tx_env == NULL)
    {
        uint16_t env_size = sizeof(struct isopcm_tx_info) +  (nb_buffer * sizeof(uint8_t*));
        uint8_t i;
        uint8_t* p_buf;
        DBG_SWDIAG(ISOPCM, START, 1);

        // allocate environment memory for Controller to Host path
        p_tx_env = (struct isopcm_tx_info*) ke_malloc(env_size + (nb_buffer * max_sdu), KE_MEM_ENV);
        p_tx_env->nb_buffer    = nb_buffer;
        p_tx_env->prog_idx     = 0;
        p_tx_env->sdu_len      = max_sdu;
        p_tx_env->nb_channel   = 1;
        p_tx_env->data_loading = false;

        // delayed job initialization
        co_djob_init(&(p_tx_env->djob), isopcm_tx_djob_handler);

        status = CO_ERROR_NO_ERROR;

        p_buf = ((uint8_t*) p_tx_env) + env_size;

        // Allocate number of buffer required
        for(i = 0 ; i < nb_buffer ; i++)
        {
            p_tx_env->p_buf[i] = p_buf;

            p_buf += max_sdu;
        }

        // Select Codec source (BLE)
        plf_pcm_codec_src_sel(1);

        // Configure driver
        pcm_src_setup(max_sdu, sdu_interval);
        // Enable PCM
        pcm_mode_set(true);

        isopcm_env.p_tx = p_tx_env;
    }
    else
    {
        // check same parameters are used
        if((max_sdu != p_tx_env->sdu_len) && (nb_buffer != p_tx_env->nb_buffer))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
        }
        // increase number of channel that use DMA
        else
        {
            DBG_SWDIAG(ISOPCM, START, 1);
            p_tx_env->nb_channel++;
            status = CO_ERROR_NO_ERROR;
        }
    }

    DBG_SWDIAG(ISOPCM, START, 0);
    DBG_SWDIAG(ISOPCM, TX, 0);

    return status;
}

__STATIC void isopcm_tx_stop(uint16_t conhdl, uint8_t reason)
{
    struct isopcm_tx_info* p_tx_env = isopcm_env.p_tx;
    DBG_SWDIAG(ISOPCM, TX, 1);
    if(p_tx_env != NULL)
    {
        DBG_SWDIAG(ISOPCM, STOP, 1);
        p_tx_env->nb_channel --;

        if(p_tx_env->nb_channel == 0)
        {
            // Stop PCM
            pcm_mode_set(false);

            ke_free(p_tx_env);
            isopcm_env.p_tx = NULL;
        }
        DBG_SWDIAG(ISOPCM, STOP, 0);
    }
    DBG_SWDIAG(ISOPCM, TX, 0);
}

__STATIC uint8_t* isopcm_tx_sdu_get (uint16_t conhdl, uint32_t* p_ref_time, uint16_t* p_sdu_len)
{
    uint8_t* p_ptr = NULL;
    struct isopcm_tx_info* p_tx_env = isopcm_env.p_tx;
    DBG_SWDIAG(ISOPCM, TX, 1);
    if(p_tx_env != NULL)
    {
        DBG_SWDIAG(ISOPCM, GET, 1);
        // mark which new buffer can be filled
        p_ptr = p_tx_env->p_buf[p_tx_env->prog_idx];
        *p_sdu_len = p_tx_env->sdu_len;


        // consider that all transmit stream will load pointer in same interrupt context
        if(!p_tx_env->data_loading)
        {
            p_tx_env->data_loading = true;
            co_djob_isr_reg(&(p_tx_env->djob));
            // load source data
            pcm_src_ptr_set(p_ptr, p_tx_env->sdu_len);
        }
        DBG_SWDIAG(ISOPCM, GET, 0);
    }
    DBG_SWDIAG(ISOPCM, TX, 0);

    return p_ptr;
}

__STATIC void isopcm_tx_sdu_done(uint16_t conhdl, uint16_t sdu_len, uint32_t ref_time, uint8_t* p_buf, uint8_t status)
{
    DBG_SWDIAG(ISOPCM, TX, 1);
    DBG_SWDIAG(ISOPCM, DONE, 1);
    // nothing to do
    DBG_SWDIAG(ISOPCM, DONE, 0);
    DBG_SWDIAG(ISOPCM, TX, 0);
}

/*
 * FUNCTION DEFINITION
 ****************************************************************************************
 */


#endif //(BLE_ISO_PCM)

/// @} ISOPCM
