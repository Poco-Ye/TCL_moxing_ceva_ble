/**
 ****************************************************************************************
 *
 * @file gapc.c
 *
 * @brief Generic Access Profile Controller Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPC Generic Access Profile Controller
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (GAPC_PRESENT)
#include "rwip.h"
#include "gapc_int.h"
#include "gapm.h"
#include "ke_mem.h"
#include "co_math.h"
#include "co_utils.h"
#include <string.h>
#include "gatt.h"

#if(BLE_GAPC)
#include "gapc_le_con.h"
#endif // (BLE_GAPC)

#if(BT_HOST_PRESENT)
#include "gapc_bt_con.h"
#endif // (BT_HOST_PRESENT)

#include "ke_task.h"
#include "l2cap.h"

#if (BLE_L2CAP)
#include "../../inc/l2cap_hl_api.h" // Internal API required
#endif // (BLE_L2CAP)

#if (BLE_GATT)
#include "../../inc/gatt_hl_api.h" // Internal API required
#endif // (BLE_GATT)


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
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

gapc_env_t gapc_env;

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Cleanup GAP Connection resources - internal
 *
 * @param[in] p_con  Pointer to connection object
 * @param[in] conidx connection index
 * @param[in] reset  True if cleanup reason is device reset, disconnection otherwise
 ****************************************************************************************
 */
__STATIC void gapc_con_cleanup_int(gapc_con_t* p_con, uint8_t conidx, bool reset)
{
    if(!reset)
    {
        #if (BLE_GAPC)
        if(GETB(p_con->info_bf, GAPC_LE_CON_TYPE))
        {
            #if (BLE_GATT)
            // Inform GATT about terminated connection
            gatt_cleanup(conidx);
            #endif // (BLE_GATT)
        }
        #endif // (BLE_GAPC)

        #if(BLE_L2CAP)
        // Inform L2CAP about terminated connection
        l2cap_cleanup(conidx);
        #endif // (BLE_L2CAP)

        // Abort on-going operation due to a disconnection
        gapc_proc_cleanup(conidx, GAP_ERR_DISCONNECTED);
    }
    else
    {
        // initialize procedure memory
        gapc_proc_reset(conidx);
    }

    #if (BLE_GAPC)
    if(GETB(p_con->info_bf, GAPC_LE_CON_TYPE))
    {
        gapc_le_smp_pairing_proc_cleanup((gapc_le_con_t*)p_con);
    }
    #endif // (BLE_GAPC)

    ke_free(p_con);
    gapc_env.p_con[conidx] = NULL;
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Initialize Generic Access Profile Controller Module.
 *
 * @param[in] init_type  Type of initialization (see enum #rwip_init_type)
 *
 ****************************************************************************************
 */
void gapc_initialize(uint8_t init_type)
{
    // Index
    uint8_t conidx;

    switch (init_type)
    {
        case RWIP_INIT:
        {
            #if (HOST_MSG_API)
            // Create GAP Controller task
            gapc_msg_api_handler_create();
            #endif // (HOST_MSG_API)
        } break;

        case RWIP_RST:
        {
            #if (BLE_GAPC) //  Memory size check
            ASSERT_WARN(GAPC_LE_HEAP_ENV_SIZE == (sizeof(gapc_le_con_t) + KE_HEAP_MEM_RESERVED),
                        GAPC_LE_HEAP_ENV_SIZE, sizeof(gapc_le_con_t) + KE_HEAP_MEM_RESERVED);
            ASSERT_WARN(GAPC_HEAP_ENV_SIZE >= GAPC_LE_HEAP_ENV_SIZE, GAPC_HEAP_ENV_SIZE, GAPC_LE_HEAP_ENV_SIZE);
            #endif // (BLE_GAPC)

            #if (BT_HOST_PRESENT) // Memory size check
            ASSERT_WARN(GAPC_BT_HEAP_ENV_SIZE == (sizeof(gapc_bt_con_t) + KE_HEAP_MEM_RESERVED),
                        GAPC_BT_HEAP_ENV_SIZE, sizeof(gapc_bt_con_t) + KE_HEAP_MEM_RESERVED);
            ASSERT_WARN(GAPC_HEAP_ENV_SIZE >= GAPC_BT_HEAP_ENV_SIZE, GAPC_HEAP_ENV_SIZE, GAPC_BT_HEAP_ENV_SIZE);
            #endif // (BT_HOST_PRESENT)

            // Initialize GAP controllers
            for (conidx = 0; conidx < GAPC_IDX_MAX; conidx++)
            {
                // clean connection object
                gapc_con_t* p_con = gapc_get_con_env(conidx);
                if(p_con != NULL)
                {
                    gapc_con_cleanup_int(p_con, conidx, true);
                }
            }

            // Clean-up security procedure
            if(gapc_env.p_sec_proc != NULL)
            {
                ke_free(gapc_env.p_sec_proc);
                gapc_env.p_sec_proc = NULL;
            }
        }
        // no break;

        case RWIP_1ST_RST:
        {
            // Initialize GAP controllers environment variable
            memset(&gapc_env, 0, sizeof(gapc_env));
        } break;

        default:  { /* Do nothing */ } break;
    }

    // Initialize GAPC Simple Timer and Defer
    gapc_sdt_init(init_type);
}


void gapc_set_callbacks(const gapm_callbacks_t* p_cbs)
{
    gapc_env.p_con_req_cbs   = p_cbs->p_con_req_cbs;
    gapc_env.p_sec_cbs       = p_cbs->p_sec_cbs;
    gapc_env.p_info_cbs      = p_cbs->p_info_cbs;
    #if(BLE_GAPC)
    gapc_env.p_le_config_cbs = p_cbs->p_le_config_cbs;
    #endif // (BLE_GAPC)
}

uint8_t gapc_avail_conidx_find(void)
{
    uint8_t conidx;

    // Find first available connection index
    for (conidx = 0; conidx < GAPC_IDX_MAX; conidx++)
    {
        // find first task index within free state.
        if(gapc_env.p_con[conidx] == NULL)
        {
            break;
        }
    }

    // No free slot found
    if(conidx == GAPC_IDX_MAX)
    {
        // error, return wrong connection index.
        conidx = GAP_INVALID_CONIDX;
    }

    return (conidx);
}

void gapc_con_cleanup(uint8_t conidx)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    gapc_con_cleanup_int(p_con, conidx, false);
}

uint8_t gapc_get_conidx(uint16_t conhdl)
{
    uint8_t conidx = 0;

    // Find first available connection index
    for (conidx = 0; conidx < GAPC_IDX_MAX; conidx++)
    {
        gapc_con_t* p_con = gapc_get_con_env(conidx);
        // find first task index within free state.
        if((p_con != NULL)  && (p_con->conhdl == conhdl))
        {
            break;
        }
    }

    // Nothing found
    if(conidx == GAPC_IDX_MAX)
    {
        // error, return wrong connection index.
        conidx = GAP_INVALID_CONIDX;
    }

    return conidx;
}

uint16_t gapc_get_conhdl(uint8_t conidx)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    return  ((p_con) ? p_con->conhdl: GAP_INVALID_CONHDL);
}


uint8_t gapc_get_role(uint8_t conidx)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    return ((p_con) ? GETB(p_con->info_bf, GAPC_ROLE): ROLE_MASTER); // TODO provide role none
}

bool gapc_is_estab(uint8_t conidx)
{
    return (gapc_get_con_env(conidx) != NULL);
}

bool gapc_is_sec_set(uint8_t conidx, uint8_t sec_req)
{
    gapc_bond_t* p_bond = &(gapc_env.p_con[conidx]->bond);
    bool ret = false;

    switch(sec_req)
    {
        // Link is bonded
        case GAPC_LK_BONDED:
        {
            ret = GETB(p_bond->info_bf, GAPC_BONDED);
        }
        break;
        // Link is encrypted
        case GAPC_LK_ENCRYPTED:
        {
            ret = GETB(p_bond->info_bf, GAPC_LE_ENCRYPTED);
        }
        break;

        // Link key or LTK has been exchanged during pairing
        case GAPC_LK_ENC_KEY_PRESENT:
        {
            ret = GETB(p_bond->info_bf, GAPC_ENC_KEY_PRESENT);
        }
        break;
        default: /* Nothing to do */ break;
    }

    return ret;
}

uint8_t gapc_lk_sec_lvl_get(uint8_t conidx)
{
    gapc_bond_t* p_bond = &(gapc_env.p_con[conidx]->bond);
    return (GETF(p_bond->info_bf, GAPC_SEC_LVL));
}

uint8_t gapc_get_sec_lvl_from_pairing_lvl(uint8_t pairing_lvl, bool is_able_to_encrypt)
{
    uint8_t sec_lvl;

    // clear bond field to only get pairing level achieved
    SETB(pairing_lvl, GAP_PAIRING_BOND_PRESENT, false);

    switch(pairing_lvl)
    {
        // Secure connection pairing achieved
        case GAP_PAIRING_SECURE_CON: { sec_lvl = GAP_SEC_SECURE_CON; } break;
        // Authenticated pairing achieved
        case GAP_PAIRING_AUTH:       { sec_lvl = GAP_SEC_AUTH;       } break;
        default:
        {
            // unauthenticated pairing if bond data present or link encrypted
            sec_lvl = (is_able_to_encrypt) ? GAP_SEC_UNAUTH : GAP_SEC_NOT_ENC;
        } break;
    }

    return (sec_lvl);
}


uint8_t gapc_get_pairing_lvl_from_sec_lvl(uint8_t sec_lvl, bool is_bonded)
{
    uint8_t pairing_lvl;

    switch(sec_lvl)
    {
        // Secure connection pairing achieved
        case GAP_SEC_SECURE_CON: { pairing_lvl = GAP_PAIRING_SECURE_CON; } break;
        // Authenticated pairing achieved
        case GAP_SEC_AUTH :      { pairing_lvl = GAP_PAIRING_AUTH;       } break;
        // unauthenticated pairing
        default:                 { pairing_lvl = GAP_PAIRING_UNAUTH;     } break;
    }

    // Set bond field
    SETB(pairing_lvl, GAP_PAIRING_BOND_PRESENT, is_bonded);

    return (pairing_lvl);
}

void gapc_sec_lvl_set(gapc_con_t* p_con, bool link_encrypted, uint8_t pairing_lvl, bool enc_key_present)
{
    // index should not be invalid; this is just for protection
    if(p_con != NULL)
    {
        gapc_bond_t* p_bond = &(p_con->bond);
        uint8_t sec_lvl;
        SETB(p_bond->info_bf, GAPC_BONDED, GETB(pairing_lvl, GAP_PAIRING_BOND_PRESENT));
        SETB(p_bond->info_bf, GAPC_ENC_KEY_PRESENT, enc_key_present);

        // compute security level
        sec_lvl = gapc_get_sec_lvl_from_pairing_lvl(pairing_lvl,
                                        (GETB(p_bond->info_bf, GAPC_BONDED) || enc_key_present || link_encrypted));

        SETF(p_bond->info_bf, GAPC_SEC_LVL, sec_lvl);
    }
}

uint8_t gapc_get_pairing_level(uint8_t conidx)
{
    uint8_t pairing_lvl = GAP_PAIRING_NO_BOND;
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    if(p_con)
    {
        gapc_bond_t* p_bond = &(p_con->bond);
        pairing_lvl = gapc_get_pairing_lvl_from_sec_lvl(GETF(p_bond->info_bf, GAPC_SEC_LVL), GETB(p_bond->info_bf, GAPC_BONDED));
    }

    return pairing_lvl;
}

bool gapc_is_bonded(uint8_t conidx)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    return ((p_con) ? GETB(p_con->bond.info_bf, GAPC_BONDED) : false);
}

#if(BLE_GATT)
void gapc_bond_info_send(uint8_t conidx)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    gapc_bond_t* p_bond = &(p_con->bond);

    // used when pairing is done to automatically trigger first BOND Data update to Upper layer application
    SETB(p_bond->info_bf, GAPC_BOND_DATA_UPDATED, true);

    // trigger bond info only if devices are bonded
    if(   (gapc_env.p_info_cbs->bond_data_updated != NULL)
       && (GETB(p_bond->info_bf, GAPC_BONDED) || GETB(gapc_env.cfg_flags, GAPC_DBG_BOND_INFO_TRIGGER)))
    {
        gapc_bond_data_updated_t data;

        // retrieve client information
        #if (BLE_GATT_CLI)
        data.gatt_start_hdl    = p_bond->gatt_start_hdl;
        data.gatt_end_hdl      = p_bond->gatt_start_hdl + p_bond->gatt_nb_att;
        data.svc_chg_hdl       = (p_bond->svc_chg_offset == 0)
                                      ? GATT_INVALID_HDL
                                      : (p_bond->gatt_start_hdl + p_bond->svc_chg_offset);
        data.srv_feat          = p_bond->srv_feat;
        #else // !(BLE_GATT_CLI)
        data.gatt_start_hdl    = GATT_INVALID_HDL;
        data.gatt_end_hdl      = GATT_INVALID_HDL;
        data.svc_chg_hdl       = GATT_INVALID_HDL;
        data.srv_feat          = 0;
        #endif // (BLE_GATT_CLI)

        // Update the signCounter value in the GAP
        data.local_sign_counter= p_bond->sign_counter[GAPC_INFO_SRC_LOCAL];
        data.peer_sign_counter = p_bond->sign_counter[GAPC_INFO_SRC_PEER];

        // retrieve service information
        data.cli_feat          = p_bond->cli_feat;

        gapc_svc_bond_data_get(conidx, &(data.cli_info));

        gapc_env.p_info_cbs->bond_data_updated(conidx, p_con->dummy, &data);
    }
}

void gapc_att_bearer_error_send(uint8_t conidx)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    if(gapc_env.p_info_cbs->no_more_att_bearer != NULL)
    {
       gapc_env.p_info_cbs->no_more_att_bearer(conidx, p_con->dummy);
    }
}

uint32_t gapc_bond_sign_count_get(uint8_t conidx, uint8_t src)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    return p_con->bond.sign_counter[src];
}

void gapc_bond_sign_count_set(uint8_t conidx, uint8_t src, uint32_t count)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    p_con->bond.sign_counter[src] = count;

    gapc_bond_info_send(conidx);
}

const uint8_t* gapc_bond_csrk_get(uint8_t conidx, uint8_t src)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    return p_con->bond.csrk[src].key;
}
#endif // (BLE_GATT)

uint16_t gapc_connection_cfm(uint8_t conidx, uint32_t dummy, const gapc_bond_data_t* p_data)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    gapc_con_t* p_con = gapc_get_con_env(conidx);

    // check if connection exists
    if(!p_con)
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else
    {
        p_con->dummy = dummy;
        bool bond_data_present = (p_data != NULL) && GETB(p_data->pairing_lvl, GAP_PAIRING_BOND_PRESENT);

        // check if connection contains bonding data.
        if(bond_data_present)
        {
            gapc_bond_t* p_bond = &(p_con->bond);
            // ***** fill bonded data. ***** //
            // Pairing level
            gapc_sec_lvl_set(p_con, false, p_data->pairing_lvl, p_data->enc_key_present);

            // Sign counters.
            p_bond->sign_counter[GAPC_INFO_SRC_LOCAL] = p_data->local_sign_counter;
            p_bond->sign_counter[GAPC_INFO_SRC_PEER]  = p_data->remote_sign_counter;

            // CSRKs values
            memcpy(&(p_bond->csrk[GAPC_INFO_SRC_LOCAL]), &(p_data->local_csrk), sizeof(struct gap_sec_key));
            memcpy(&(p_bond->csrk[GAPC_INFO_SRC_PEER]),  &(p_data->remote_csrk), sizeof(struct gap_sec_key));
        }
        // no bond data present, nothing to restore.
        else
        {
            // Not paired because no bonding data available.
            gapc_sec_lvl_set(p_con, false, GAP_PAIRING_NO_BOND, false);
        }

        // Enable RX Flow
        #if(BLE_L2CAP)
        l2cap_rx_ctrl(conidx, true);
        #endif // (BLE_L2CAP)

        #if (BLE_GATT)
        if(p_data)
        {
            #if (BLE_GATT_CLI)
            // Inform GAP / GATT client that connection starts
            gapc_cli_con_create(conidx, bond_data_present, p_data->gatt_start_hdl, p_data->gatt_end_hdl, p_data->svc_chg_hdl,
                                p_data->srv_feat);
            #endif // (BLE_GATT_CLI)

            // Inform GAP / GATT service that connection starts
            gapc_svc_con_create(conidx, bond_data_present, p_data->cli_info, p_data->cli_feat);
        }
        else
        {
            // Inform GAP / GATT service that connection starts
            gapc_svc_con_create(conidx, false, 0, 0);
        }
        #endif // (BLE_GATT)
    }

    return (status);
}

uint16_t gapc_pairing_numeric_compare_rsp(uint8_t conidx, bool accept)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    #if(BLE_GAPC)
    if(gapc_is_le_connection(conidx))
    {
        status = gapc_le_pairing_numeric_compare_rsp(conidx, accept);
    }
    else
    #endif // (BLE_GAPC)
    {
        #if(BT_HOST_PRESENT)
        status = gapc_bt_pairing_numeric_compare_rsp(conidx, accept);
        #endif // (BT_HOST_PRESENT)
    }

    return (status);
}
uint16_t gapc_pairing_provide_passkey(uint8_t conidx, bool accept, uint32_t passkey)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    #if(BLE_GAPC)
    if(gapc_is_le_connection(conidx))
    {
        status = gapc_le_pairing_provide_passkey(conidx, accept, passkey);
    }
    else
    #endif // (BLE_GAPC)
    {
        #if(BT_HOST_PRESENT)
        status = gapc_bt_pairing_provide_passkey(conidx, accept, passkey);
        #endif // (BT_HOST_PRESENT)
    }

    return (status);
}

/* HIDDEN INTERNAL FUNCTIONS USED TO RE-ROUTE SECURITY CALLBACK
 *************************************************************************************
 */

/// Get Application security callback interface
const gapc_security_cb_t* gapc_get_security_cb_itf(void)
{
    return gapc_env.p_sec_cbs;
}

/// Update application callback interface to re-route pairing information
void gapc_set_security_cb_itf(const gapc_security_cb_t* p_cbs)
{
    gapc_env.p_sec_cbs = p_cbs;
}

#endif // (GAPC_PRESENT)
/// @} GAPC
