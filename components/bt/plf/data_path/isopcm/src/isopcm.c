/**
 ****************************************************************************************
 *
 * @file isopcm.h
 *
 * @brief Declaration of the functions used for Isochronous Payload over PCM
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

#include <string.h>              // standard definition
#include "rwip.h"
#include "isopcm_int.h"

#include "dbg_swdiag.h"
#include "ke_mem.h"
#include "ke_event.h"

#include "co_bt_defines.h"
#include "co_list.h"
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

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct isopcm_env_tag isopcm_env;

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */


/*
 * FUNCTION DEFINITION
 ****************************************************************************************
 */

void isopcm_init(uint8_t init_type)
{
    switch (init_type)
    {
        case RWIP_INIT:
        {
            // Do nothing
        }
        break;

        case RWIP_RST:
        {
            if(isopcm_env.p_tx != NULL)
            {
                ke_free(isopcm_env.p_tx);
                isopcm_env.p_tx = NULL;
            }

            if(isopcm_env.p_rx != NULL)
            {
                ke_free(isopcm_env.p_rx);
                isopcm_env.p_rx = NULL;
            }
        }
        break;

        case RWIP_1ST_RST:
        {
            // Initialize environment variable
            memset(&isopcm_env, 0, sizeof(isopcm_env));
        }
        break;

        default:
        {
            // Do nothing
        }
        break;
    }
}

const struct data_path_itf* isopcm_itf_get(uint8_t  direction)
{
    const struct data_path_itf* res = NULL;

    // load the interface
    switch(direction)
    {
        case ISO_SEL_TX: { res = &isopcm_tx_itf; } break;
        case ISO_SEL_RX: { res = &isopcm_rx_itf; } break;
        default:         { /* Nothing to do */   } break;
    }

    return res;
}


#endif //(BLE_ISO_PCM)

/// @} ISOPCM
