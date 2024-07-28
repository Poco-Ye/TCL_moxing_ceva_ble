/**
 ****************************************************************************************
 *
 * @file isopcm.h
 *
 * @brief Declaration of the functions used for Isochronous Payload over PCM, RX path
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

__STATIC uint8_t isopcm_rx_start(uint16_t conhdl, uint32_t sdu_interval, uint32_t trans_latency, uint16_t max_sdu);
__STATIC void isopcm_rx_stop(uint16_t conhdl, uint8_t reason);
__STATIC uint8_t* isopcm_rx_sdu_get (uint16_t conhdl, uint32_t* p_ref_time, uint16_t* p_sdu_len);
__STATIC void isopcm_rx_sdu_done(uint16_t conhdl, uint16_t sdu_len, uint32_t ref_time, uint8_t* p_buf, uint8_t status);


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// PCM RX interface
const struct data_path_itf isopcm_rx_itf =
{
    isopcm_rx_start,
    isopcm_rx_stop,
    isopcm_rx_sdu_get,
    isopcm_rx_sdu_done,
    NULL,
    NULL,
};

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

__STATIC uint8_t isopcm_rx_start(uint16_t conhdl, uint32_t sdu_interval, uint32_t trans_latency, uint16_t max_sdu)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    struct isopcm_rx_info* p_rx_env = isopcm_env.p_rx;

    DBG_SWDIAG(ISOPCM, RX, 1);

    // Check parameters
    if((max_sdu >= 65) || ((max_sdu == 0) || (sdu_interval == 0)) || (channel >= BLE_ACTIVITY_MAX))
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }

    else if(p_rx_env == NULL)
    {
        uint8_t nb_buffer = CO_DIVIDE_CEIL(trans_latency, sdu_interval);
        uint16_t env_size = sizeof(struct isopcm_rx_info) +  (nb_buffer * sizeof(uint8_t*));
        uint8_t i;
        uint8_t* p_buf;

        DBG_SWDIAG(ISOPCM, START, 1);

        // allocate environment memory for Controller to Host path
        p_rx_env = (struct isopcm_rx_info*) ke_malloc(env_size + (nb_buffer * max_sdu), KE_MEM_ENV);
        p_rx_env->nb_buffer = nb_buffer;
        p_rx_env->sdu_idx  = 0;
        p_rx_env->sdu_len   = max_sdu;
        p_rx_env->channel   = channel;

        status = CO_ERROR_NO_ERROR;

        p_buf = ((uint8_t*) p_rx_env) + env_size;

        // Allocate number of buffer required
        for(i = 0 ; i < nb_buffer ; i++)
        {
            p_rx_env->p_buf[i] = p_buf;

            p_buf += max_sdu;
        }

        // Select Codec source (BLE)
        plf_pcm_codec_src_sel(1);

        // Configure driver
        pcm_sink_setup(max_sdu, sdu_interval);
        // Enable PCM
        pcm_mode_set(true);

        isopcm_env.p_rx = p_rx_env;

        DBG_SWDIAG(ISOPCM, START, 0);
    }
    DBG_SWDIAG(ISOPCM, RX, 0);

    return status;
}

__STATIC void isopcm_rx_stop(uint16_t conhdl, uint8_t reason)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);
    struct isopcm_rx_info* p_rx_env = isopcm_env.p_rx;

    DBG_SWDIAG(ISOPCM, RX, 1);
    if((p_rx_env != NULL) && (p_rx_env->channel == channel))
    {
        DBG_SWDIAG(ISOPCM, STOP, 1);

        // Stop PCM
        pcm_mode_set(false);

        ke_free(p_rx_env);
        isopcm_env.p_rx = NULL;
        DBG_SWDIAG(ISOPCM, STOP, 0);
    }
    DBG_SWDIAG(ISOPCM, RX, 0);
}

__STATIC uint8_t* isopcm_rx_sdu_get(uint16_t conhdl, uint32_t* p_ref_time, uint16_t* p_sdu_len)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);
    uint8_t* p_ptr = NULL;
    struct isopcm_rx_info* p_rx_env = isopcm_env.p_rx;

    DBG_SWDIAG(ISOPCM, RX, 1);

    if((p_rx_env != NULL) && (p_rx_env->channel == channel))
    {
        DBG_SWDIAG(ISOPCM, GET, 1);
        p_ptr = p_rx_env->p_buf[p_rx_env->sdu_idx];
        DBG_SWDIAG(ISOPCM, GET, 0);
    }

    DBG_SWDIAG(ISOPCM, RX, 0);
    return p_ptr;
}

__STATIC void isopcm_rx_sdu_done(uint16_t conhdl, uint16_t sdu_len, uint32_t ref_time, uint8_t* p_buf, uint8_t status)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);
    struct isopcm_rx_info* p_rx_env = isopcm_env.p_rx;

    DBG_SWDIAG(ISOPCM, RX, 1);

    if((p_rx_env != NULL) && (p_rx_env->channel == channel))
    {
        // TODO [FBE] for the moment just ignore the status

        DBG_SWDIAG(ISOPCM, DONE, 1);

        // mark which new buffer can be received
        pcm_sink_ptr_set(p_rx_env->p_buf[p_rx_env->sdu_idx], p_rx_env->sdu_len);

        // Move SDU index to next buffer
        CO_VAL_INC(p_rx_env->sdu_idx, p_rx_env->nb_buffer);

        DBG_SWDIAG(ISOPCM, DONE, 0);
    }

    DBG_SWDIAG(ISOPCM, RX, 0);
}

/*
 * FUNCTION DEFINITION
 ****************************************************************************************
 */

#endif //(BLE_ISO_PCM)

/// @} ISOPCM
