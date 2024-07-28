/**
 ****************************************************************************************
 *
 * @file gapm.c
 *
 * @brief Generic Access Profile Manager Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPM Generic Access Profile Manager
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"  // Software configuration
#include "rwip.h"         // RW definitions

#include "gap.h"          // Generic access profile
#include "gapm.h"         // Generic access profile Manager
#include "gapc.h"         // Generic access profile Controller
#include "../gap_int.h" // Internal API required


#include "gapm_int.h"     // Generic access profile Manager Internal

#include "l2cap.h"        // L2CAP Module

#include "ke_mem.h"       // Kernel memory management

#include "co_math.h"      // Mathematic library
#include "co_utils.h"     // Utils


#if (HOST_PROFILES)
#include "../../inc/prf_hl_api.h"
#endif // (HOST_PROFILES)

#include <string.h>     // for memset / memcpy

#if (BLE_GAF_PRESENT)
#include "gaf_inc.h"
#endif //(BLE_GAF_PRESENT


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */



/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// environment structure
gapm_env_t gapm_env;


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize Generic Access Profile Manager Module.
 *
 * @param[in] init_type  Type of initialization (see enum #rwip_init_type)
 ****************************************************************************************
 */
extern void gapm_initialize(uint8_t init_type)
{
    // boot configuration
    switch (init_type)
    {
        case RWIP_RST:
        {
            #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
            // clean-up OOB data
            gapm_oob_data_release();
            #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
            #if (BT_HOST_PRESENT)
            if(gapm_env.p_device_identification_sdp_record != NULL)
            {
                ke_free(gapm_env.p_device_identification_sdp_record);
            }
            #endif // (BT_HOST_PRESENT)
        }
            // No break

        case RWIP_1ST_RST:
        {
            // Reset activity module
            gapm_actv_initialize(init_type);
            #if (BLE_HOST_PRESENT)
            gapm_le_actv_initialize(init_type);
            #endif // (BLE_HOST_PRESENT)

            // clear current role
            gapm_env.role = GAP_ROLE_NONE;
            // all connections reset
            gapm_env.connections = 0;
            gapm_env.configured  = false;
            gapm_env.reset       = false;

            #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
            gapm_env.past_estab_bf = 0;
            #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)

            #if (HL_LE_OBSERVER)
            gapm_env.per_sync_estab_actv_idx = GAP_INVALID_ACTV_IDX;
            #endif //(HL_LE_OBSERVER)
            gapm_env.p_name   = NULL;
            gapm_env.name_len = 0;

            #if (BT_HOST_PRESENT)
            gapm_env.p_device_identification_sdp_record = NULL;
            #endif // (BT_HOST_PRESENT)
        }
        break;

        case RWIP_INIT:
        default: { /* Do nothing */ } break;
    }

    #if (HOST_MSG_API)
    // Message initialization
    gapm_msg_initialize(init_type);
    #endif //  (HOST_MSG_API)

    // Procedure initialization
    gapm_proc_initialize(init_type);
}

#if (GAPC_PRESENT)
void gapm_con_cleanup(uint8_t conidx, uint8_t reason)
{
    // Decrement number of connections.
    gapm_env.connections--;

    // Inform GAPC Simple Timer and Defer about terminated connection
    gapc_sdt_cleanup(conidx);

    /* ******** Inform other tasks that connection has been disconnected. ******** */

    // Inform GAPC about terminated connection
    gapc_con_cleanup(conidx);

    #if (HOST_PROFILES)
    // Inform profiles about terminated connection
    prf_con_cleanup(conidx, reason);
    #endif /* (HOST_PROFILES) */

    #if (BLE_GAF_PRESENT)
    gaf_disconnect(conidx);
    #endif //(BLE_GAF_PRESENT)
}
#endif // (GAPC_PRESENT)

bool gapm_is_legacy_pairing_supp(void)
{
    return ((gapm_env.pairing_mode & GAPM_PAIRING_LEGACY) !=0);
}


bool gapm_is_sec_con_pairing_supp(void)
{
    return ((gapm_env.pairing_mode & GAPM_PAIRING_SEC_CON) !=0);
}

uint8_t gapm_is_controler_privacy_enabled(void)
{
   return ((gapm_env.priv_cfg & GAPM_PRIV_CFG_PRIV_EN_BIT) != 0);
}

const gap_sec_key_t* gapm_get_irk(void)
{
    return &(gapm_env.irk);
}

uint16_t gapm_token_id_get(void)
{
    gapm_env.token_id_cnt++;
    // ensure that token is never zero
    if(gapm_env.token_id_cnt == GAP_INVALID_TOKEN)
    {
        gapm_env.token_id_cnt++;
    }

    return gapm_env.token_id_cnt;
}

#if (HL_LE_OBSERVER)
void gapm_adv_report_flow_ctrl(bool enable)
{
    gapm_env.adv_report_flow_off = !enable;
}
#endif //(HL_LE_OBSERVER)

bool gapm_is_configured(void)
{
    return gapm_env.configured;
}

#if (!EMB_PRESENT)
bool gapm_is_doing_reset(void)
{
    return gapm_env.do_reset;
}
#endif // (!EMB_PRESENT)
/// @} GAPM
