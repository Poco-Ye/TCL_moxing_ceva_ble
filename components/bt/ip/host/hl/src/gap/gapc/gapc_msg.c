/**
 ****************************************************************************************
 *
 * @file gapc_task.c
 *
 * @brief Generic Access Profile Controller Task implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPC_TASK Generic Access Profile Controller Task
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (GAPC_PRESENT && HOST_MSG_API)

#include "co_math.h"
#include "co_utils.h"        // core utility functions

#include "gap.h"
#include "gapc_int.h"
#if(BLE_GAPC)
#include "gapc_le_msg.h"
#endif // (BLE_GAPC)

#include "gapm.h"
#include "gapm_msg.h"

#include "gatt.h"

#if (BT_HOST_PRESENT)
#include "gapc_bt_con.h"
#include "gapc_bt_msg.h"
#endif // (BT_HOST_PRESENT)

#if(BLE_GAPC)
#include "gapc_le_smp.h"
#endif // (BLE_GAPC)

#include "hci.h"
#include "ke_msg.h"
#include "ke_task.h"

#include <string.h>  // for memset and memcopy

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

__STATIC void gapc_msg_disconnected(uint8_t conidx, uint32_t dest_id, uint16_t reason);
__STATIC void gapc_msg_bond_data_updated(uint8_t conidx, uint32_t dest_id, const gapc_bond_data_updated_t* p_data);
__STATIC void gapc_msg_auth_payload_timeout(uint8_t conidx, uint32_t dest_id);
__STATIC void gapc_msg_no_more_att_bearer(uint8_t conidx, uint32_t dest_id);
__STATIC void gapc_msg_cli_hash_info(uint8_t conidx, uint32_t dest_id, uint16_t handle, const uint8_t* p_hash);

__STATIC void gapc_msg_info_name_get(uint8_t conidx, uint32_t dest_id, uint16_t token, uint16_t offset, uint16_t max_length);
__STATIC void gapc_msg_info_appearance_get(uint8_t conidx, uint32_t dest_id, uint16_t token);
__STATIC void gapc_msg_info_slave_pref_param_get(uint8_t conidx, uint32_t dest_id, uint16_t token);
__STATIC void gapc_msg_info_name_set(uint8_t conidx, uint32_t dest_id, uint16_t token, co_buf_t* p_buf);
__STATIC void gapc_msg_info_appearance_set(uint8_t conidx, uint32_t dest_id, uint16_t token, uint16_t appearance);



__STATIC void gapc_msg_sec_pairing_succeed(uint8_t conidx, uint32_t dest_id);
__STATIC void gapc_msg_sec_pairing_failed(uint8_t conidx, uint32_t dest_id, uint16_t reason);
__STATIC void gapc_msg_sec_numeric_compare_req(uint8_t conidx, uint32_t dest_id, uint32_t numeric_value);
__STATIC void gapc_msg_sec_info_req(uint8_t conidx, uint32_t dest_id, uint8_t exp_info);
__STATIC void gapc_msg_sec_auth_info(uint8_t conidx, uint32_t dest_id, uint8_t sec_lvl, bool encrypted);

#if (BLE_GAPC)
__STATIC void gapc_msg_sec_le_encrypt_req(uint8_t conidx, uint32_t dest_id, uint16_t ediv, const rand_nb_t* p_rand);
__STATIC void gapc_msg_sec_auth_req(uint8_t conidx, uint32_t dest_id, uint8_t auth_level);
__STATIC void gapc_msg_sec_repeated_attempt(uint8_t conidx, uint32_t dest_id);
__STATIC void gapc_msg_sec_pairing_req(uint8_t conidx, uint32_t dest_id, uint8_t auth_level);
__STATIC void gapc_msg_sec_key_pressed(uint8_t conidx, uint32_t dest_id, uint8_t notification_type);
__STATIC void gapc_msg_sec_ltk_req(uint8_t conidx, uint32_t dest_id, uint8_t key_size);
__STATIC void gapc_msg_sec_key_received(uint8_t conidx, uint32_t dest_id, const gapc_pairing_keys_t* p_keys);
#endif // (BLE_GAPC)

#if (BT_HOST_PRESENT)
__STATIC void gapc_msg_sec_bt_encrypt_req(uint8_t conidx, uint32_t dest_id);
__STATIC void gapc_msg_sec_peer_iocap(uint8_t conidx, uint32_t dest_id, uint8_t iocap, uint8_t auth_req_bf, bool oob_present);
__STATIC void gapc_msg_sec_display_passkey(uint8_t conidx, uint32_t dest_id, uint32_t passkey);
#endif // (BT_HOST_PRESENT)

#if (BLE_GAPC)
__STATIC void gapc_msg_le_connection_req(uint8_t conidx, uint32_t dest_id, uint8_t actv_idx, uint8_t role,
                                         const gap_bdaddr_t* p_peer_addr, const gap_le_con_param_t* p_con_params,
                                         uint8_t clk_accuracy);
__STATIC void gapc_msg_le_config_param_update_req(uint8_t conidx, uint32_t dest_id, const gap_le_con_param_nego_t* p_param);
__STATIC void gapc_msg_le_config_param_updated(uint8_t conidx, uint32_t dest_id, const gap_le_con_param_t* p_param);
__STATIC void gapc_msg_le_config_packet_size_updated(uint8_t conidx, uint32_t dest_id, uint16_t max_tx_octets ,
                                                     uint16_t max_tx_time, uint16_t max_rx_octets , uint16_t max_rx_time);
__STATIC void gapc_msg_le_config_phy_updated(uint8_t conidx, uint32_t dest_id, uint8_t tx_phy , uint8_t rx_phy);

#if (BLE_PWR_CTRL)
__STATIC void gapc_msg_le_power_tx_change_report(uint8_t conidx, uint32_t dest_id, bool local, const gapc_tx_power_report_t* p_report);
__STATIC void gapc_msg_le_power_path_loss_threshold_report(uint8_t conidx, uint32_t dest_id, uint8_t curr_path_loss, uint8_t zone_entered);
#endif // (BLE_PWR_CTRL)

#if (BLE_AOA || BLE_AOD)
__STATIC void gapc_msg_le_cte_iq_report_received(uint8_t conidx, uint32_t dest_id, const gapc_iq_report_info_t* p_report,
                                                 uint8_t nb_samples, const gap_iq_sample_t* p_samples);
__STATIC void gapc_msg_le_cte_request_failed_event(uint8_t conidx, uint32_t dest_id, uint16_t reason);
#endif // (BLE_AOA || BLE_AOD)
#endif // (BLE_GAPC)

#if (BT_HOST_PRESENT)
__STATIC void gapc_msg_bt_connection_req(uint8_t conidx, uint32_t dest_id, uint8_t actv_idx, bool is_initator, const gap_addr_t* p_peer_addr);
#endif // (BT_HOST_PRESENT)

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

__STATIC const gapc_connection_req_cb_t gapc_msg_connection_req_cb_itf =
{
#if (BLE_GAPC)
    .le_connection_req            = gapc_msg_le_connection_req,
#else
    .le_connection_req            = NULL,
#endif // (BLE_GAPC)

#if (BT_HOST_PRESENT)
    .bt_connection_req            = gapc_msg_bt_connection_req,
#else
    .bt_connection_req            = NULL,
#endif // (BT_HOST_PRESENT)
};

__STATIC const gapc_connection_info_cb_t gapc_msg_connection_info_cb_itf =
{
    .disconnected         = gapc_msg_disconnected,
    .bond_data_updated    = gapc_msg_bond_data_updated,
    .auth_payload_timeout = gapc_msg_auth_payload_timeout,
    .no_more_att_bearer   = gapc_msg_no_more_att_bearer,
    .cli_hash_info        = gapc_msg_cli_hash_info,
    .name_get             = gapc_msg_info_name_get,
    .appearance_get       = gapc_msg_info_appearance_get,
    .slave_pref_param_get = gapc_msg_info_slave_pref_param_get,
    .name_set             = gapc_msg_info_name_set,
    .appearance_set       = gapc_msg_info_appearance_set,
};

__STATIC const gapc_security_cb_t gapc_msg_security_cb_itf =
{
    .auth_info                = gapc_msg_sec_auth_info,
    .pairing_succeed          = gapc_msg_sec_pairing_succeed,
    .pairing_failed           = gapc_msg_sec_pairing_failed,
    .info_req                 = gapc_msg_sec_info_req,
    .numeric_compare_req      = gapc_msg_sec_numeric_compare_req,

    #if (BLE_GAPC)
    .le_encrypt_req           = gapc_msg_sec_le_encrypt_req,
    .auth_req                 = gapc_msg_sec_auth_req,
    .pairing_req              = gapc_msg_sec_pairing_req,
    .key_pressed              = gapc_msg_sec_key_pressed,
    .ltk_req                  = gapc_msg_sec_ltk_req,
    .key_received             = gapc_msg_sec_key_received,
    .repeated_attempt         = gapc_msg_sec_repeated_attempt,
    #else
    .le_encrypt_req           = NULL,
    .auth_req                 = NULL,
    .pairing_req              = NULL,
    .key_pressed              = NULL,
    .ltk_req                  = NULL,
    .key_received             = NULL,
    .repeated_attempt         = NULL,
    #endif // (BLE_GAPC)

    #if (BT_HOST_PRESENT)
    .bt_encrypt_req           = gapc_msg_sec_bt_encrypt_req,
    .peer_iocap               = gapc_msg_sec_peer_iocap,
    .display_passkey          = gapc_msg_sec_display_passkey,
    #else
    .peer_iocap               = NULL,
    .display_passkey          = NULL,
    #endif // (BT_HOST_PRESENT)
};


#if (BLE_GAPC)
__STATIC const gapc_le_config_cb_t gapc_msg_le_config_cb_itf =
{
    .param_update_req      = gapc_msg_le_config_param_update_req,
    .param_updated         = gapc_msg_le_config_param_updated,
    .packet_size_updated   = gapc_msg_le_config_packet_size_updated,
    .phy_updated           = gapc_msg_le_config_phy_updated,

};

#if (BLE_PWR_CTRL)
__STATIC const gapc_le_power_cb_t gapc_msg_le_power_cb_itf =
{
    .tx_change_report           = gapc_msg_le_power_tx_change_report,
    .path_loss_threshold_report = gapc_msg_le_power_path_loss_threshold_report,
};
#endif // (BLE_PWR_CTRL)

#if (BLE_AOA || BLE_AOD)
__STATIC const gapc_le_cte_cb_t gapc_msg_le_cte_cb_itf =
{
    .iq_report_received   = gapc_msg_le_cte_iq_report_received,
    .request_failed_event = gapc_msg_le_cte_request_failed_event,
};
#endif // (BLE_AOA || BLE_AOD)
#endif // (BLE_GAPC)

__STATIC const gapm_callbacks_t gapc_msg_callback_itf =
{
    .p_con_req_cbs   = &gapc_msg_connection_req_cb_itf,
    .p_sec_cbs       = &gapc_msg_security_cb_itf,
    .p_info_cbs      = &gapc_msg_connection_info_cb_itf,
    #if (BLE_GAPC)
    .p_le_config_cbs = &gapc_msg_le_config_cb_itf,
    #else
    .p_le_config_cbs = NULL,
    #endif // (BLE_GAPC)
};

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void gapc_msg_send_cmd_cmp_evt(uint8_t conidx, uint8_t operation, uint16_t dest_id, uint16_t status)
{
    struct gapc_cmp_evt* p_cmp_evt = KE_MSG_ALLOC(GAPC_CMP_EVT, dest_id, TASK_GAPC, gapc_cmp_evt);
    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->conidx = conidx;
        p_cmp_evt->operation = operation;
        p_cmp_evt->status = status;
        ke_msg_send(p_cmp_evt);
    }
}


// ----------------------------------- SECURITY CALLBACKS ------------------------------------------------


__STATIC void gapc_msg_sec_auth_info(uint8_t conidx, uint32_t dest_id, uint8_t sec_lvl, bool encrypted)
{
    // TODO HANDLE ENCRYPTION ONLY ....
    #if(BT_HOST_PRESENT)
    if(gapc_is_bt_connection(conidx) && !gapc_is_doing_encryption(conidx))
    {
        struct gapc_bond_ind* p_ind = KE_MSG_ALLOC(GAPC_BOND_IND, dest_id, TASK_GAPC, gapc_bond_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->info = GAPC_BT_AUTH_INFO;
            p_ind->data.bt_auth_info.sec_lvl   = sec_lvl;
            p_ind->data.bt_auth_info.encrypted = encrypted;
            ke_msg_send(p_ind);
        }
    }
    else
    #endif // (BT_HOST_PRESENT)
    {
        struct gapc_encrypt_ind* p_ind = KE_MSG_ALLOC(GAPC_ENCRYPT_IND, dest_id, TASK_GAPC, gapc_encrypt_ind);

        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->pairing_lvl = gapc_get_pairing_level(conidx);
            ke_msg_send(p_ind);
        }
    }
}

/// Callback executed to inform that an on-going pairing has succeeded
__STATIC void gapc_msg_sec_pairing_succeed(uint8_t conidx, uint32_t dest_id)
{
    #if(BLE_GAPC)
    if(gapc_is_le_connection(conidx))
    {
        struct gapc_bond_ind *p_ind = KE_MSG_ALLOC(GAPC_BOND_IND, dest_id, TASK_GAPC, gapc_bond_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->info = GAPC_PAIRING_SUCCEED;
            p_ind->data.pairing.level = gapc_get_pairing_level(conidx);
            p_ind->data.pairing.ltk_present = gapc_is_sec_set(conidx, GAPC_LK_ENC_KEY_PRESENT);
            ke_msg_send(p_ind);
        }

        if(gapc_get_role(conidx) == ROLE_MASTER)
        {
            gapc_msg_send_cmd_cmp_evt(conidx, GAPC_BOND, dest_id, GAP_ERR_NO_ERROR);
        }
    }
    else
    #endif // (BLE_GAPC)
    {
        #if(BT_HOST_PRESENT)
        struct gapc_bond_ind* p_ind = KE_MSG_ALLOC(GAPC_BOND_IND, dest_id, TASK_GAPC, gapc_bond_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->info = GAPC_BT_PAIRING_END;
            p_ind->data.bt_pairing_status = GAP_ERR_NO_ERROR;
            ke_msg_send(p_ind);
        }
        #endif // (BT_HOST_PRESENT)
    }
}

/// Callback executed to inform that an on-going pairing has failed
__STATIC void gapc_msg_sec_pairing_failed(uint8_t conidx, uint32_t dest_id, uint16_t reason)
{
    #if (BLE_GAPC)
    if(gapc_is_le_connection(conidx))
    {
        struct gapc_bond_ind *p_ind = KE_MSG_ALLOC(GAPC_BOND_IND, dest_id, TASK_GAPC, gapc_bond_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->info = GAPC_PAIRING_FAILED;
            p_ind->data.reason = reason;
            ke_msg_send(p_ind);
        }

        if(gapc_get_role(conidx) == ROLE_MASTER)
        {
            gapc_le_con_t *p_con = gapc_le_con_env_get(conidx);
            if(gapc_le_smp_is_pairing_ongoing(p_con))
            {
                gapc_msg_send_cmd_cmp_evt(conidx, GAPC_BOND, dest_id, GAP_ERR_NO_ERROR);
            }
        }
    }
    else
    #endif // (BLE_GAPC)
    {
        #if (BT_HOST_PRESENT)
        struct gapc_bond_ind* p_ind = KE_MSG_ALLOC(GAPC_BOND_IND, dest_id, TASK_GAPC, gapc_bond_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->info = GAPC_BT_PAIRING_END;
            p_ind->data.bt_pairing_status = reason;
            ke_msg_send(p_ind);
        }
        #endif // (BT_HOST_PRESENT)
    }
}

/// Callback executed when an information is required by pairing algorithm.\n
__STATIC void gapc_msg_sec_info_req(uint8_t conidx, uint32_t dest_id, uint8_t exp_info)
{
    struct gapc_bond_req_ind *p_req_ind = KE_MSG_ALLOC(GAPC_BOND_REQ_IND, dest_id, TASK_GAPC, gapc_bond_req_ind);
    if(p_req_ind != NULL)
    {
        p_req_ind->conidx = conidx;
        switch(exp_info)
        {
            #if (BT_HOST_PRESENT)
            case GAPC_INFO_BT_IOCAP:    { p_req_ind->request = GAPC_BT_IOCAP;    } break;
            case GAPC_INFO_BT_PIN_CODE: { p_req_ind->request = GAPC_BT_PIN_CODE; } break;
            case GAPC_INFO_BT_PASSKEY:  { p_req_ind->request = GAPC_BT_PASSKEY;  } break;
            #endif // (BT_HOST_PRESENT)
            case GAPC_INFO_IRK:         { p_req_ind->request = GAPC_IRK_EXCH;    } break;
            case GAPC_INFO_CSRK:        { p_req_ind->request = GAPC_CSRK_EXCH;   } break;
            case GAPC_INFO_OOB:         { p_req_ind->request = GAPC_OOB_EXCH;    } break;
            case GAPC_INFO_TK_OOB:
            {
                p_req_ind->request = GAPC_TK_EXCH;
                p_req_ind->data.tk_type = GAP_TK_OOB;
            } break;
            case GAPC_INFO_TK_DISPLAYED:
            case GAPC_INFO_PASSKEY_DISPLAYED:
            {
                p_req_ind->request = GAPC_TK_EXCH;
                p_req_ind->data.tk_type = GAP_TK_DISPLAY;
            } break;
            case GAPC_INFO_TK_ENTERED:
            case GAPC_INFO_PASSKEY_ENTERED:
            {
                p_req_ind->request = GAPC_TK_EXCH;
                p_req_ind->data.tk_type = GAP_TK_KEY_ENTRY;
            } break;
            default: { ASSERT_ERR(0); } break;
        }
        ke_msg_send(p_req_ind);
    }
}

/// Callback request some keys that must be exchange during pairing procedure\n
__STATIC void gapc_msg_sec_numeric_compare_req(uint8_t conidx, uint32_t dest_id, uint32_t numeric_value)
{
    struct gapc_bond_req_ind *p_req_ind = KE_MSG_ALLOC(GAPC_BOND_REQ_IND, dest_id, TASK_GAPC, gapc_bond_req_ind);
    if(p_req_ind != NULL)
    {
        p_req_ind->conidx = conidx;

        #if(BLE_GAPC)
        p_req_ind->request = gapc_is_le_connection(conidx) ? GAPC_NC_EXCH : GAPC_BT_USER_VALUE_CFM;
        #else
        p_req_ind->request = GAPC_BT_USER_VALUE_CFM;
        #endif // (BLE_GAPC)

        p_req_ind->data.numeric_value = numeric_value;
        ke_msg_send(p_req_ind);
    }
}

#if (BLE_GAPC)
/// Callback executed when an encryption is requested by peer device
__STATIC void gapc_msg_sec_le_encrypt_req(uint8_t conidx, uint32_t dest_id, uint16_t ediv, const rand_nb_t* p_rand)
{
    struct gapc_le_encrypt_req_ind* p_ind = KE_MSG_ALLOC(GAPC_LE_ENCRYPT_REQ_IND, dest_id, TASK_GAPC, gapc_le_encrypt_req_ind);

    if(p_ind != NULL)
    {
        p_ind->conidx  = conidx;
        p_ind->ediv    = ediv;
        p_ind->rand_nb = *p_rand;
        ke_msg_send(p_ind);
    }
}

/// Callback executed to inform that peer peripheral is asking for a pairing / encryption with a specific authentication level.\n
__STATIC void gapc_msg_sec_auth_req(uint8_t conidx, uint32_t dest_id, uint8_t auth_level)
{
    struct gapc_security_ind* p_ind = KE_MSG_ALLOC(GAPC_SECURITY_IND, dest_id, TASK_GAPC, gapc_security_ind);
    if(p_ind != NULL)
    {
        p_ind->conidx = conidx;
        p_ind->auth = auth_level;
        ke_msg_send(p_ind);
    }
}

/// Callback executed to inform that a pairing repeated attempt problem is detected.
__STATIC void gapc_msg_sec_repeated_attempt(uint8_t conidx, uint32_t dest_id)
{
    struct gapc_bond_ind* p_ind = KE_MSG_ALLOC(GAPC_BOND_IND, dest_id, TASK_GAPC, gapc_bond_ind);
    if(p_ind != NULL)
    {
        p_ind->conidx = conidx;
        p_ind->info = GAPC_REPEATED_ATTEMPT;
        ke_msg_send(p_ind);
    }
}

/// Callback executed to inform that a pairing is initiated by peer central.
__STATIC void gapc_msg_sec_pairing_req(uint8_t conidx, uint32_t dest_id, uint8_t auth_level)
{
    struct gapc_bond_req_ind *p_req_ind = KE_MSG_ALLOC(GAPC_BOND_REQ_IND, dest_id, TASK_GAPC, gapc_bond_req_ind);
    if(p_req_ind != NULL)
    {
        p_req_ind->conidx = conidx;
        p_req_ind->request = GAPC_PAIRING_REQ;
        p_req_ind->data.auth_req = auth_level;
        ke_msg_send(p_req_ind);
    }
}

/// Callback request some keys that must be exchange during pairing procedure\n
__STATIC void gapc_msg_sec_key_pressed(uint8_t conidx, uint32_t dest_id, uint8_t notification_type)
{
    struct gapc_key_press_notif_ind  *p_ind = KE_MSG_ALLOC(GAPC_KEY_PRESS_NOTIFICATION_IND, dest_id, TASK_GAPC,
                                                            gapc_key_press_notif_ind);

    if(p_ind != NULL)
    {
        p_ind->conidx = conidx;
        p_ind->notification_type = notification_type;
        ke_msg_send(p_ind);
    }
}


/// Callback executed when an information is required by pairing algorithm.\n
__STATIC void gapc_msg_sec_ltk_req(uint8_t conidx, uint32_t dest_id, uint8_t key_size)
{
    struct gapc_bond_req_ind *p_req_ind = KE_MSG_ALLOC(GAPC_BOND_REQ_IND, dest_id, TASK_GAPC, gapc_bond_req_ind);
    if(p_req_ind != NULL)
    {
        p_req_ind->conidx = conidx;
        p_req_ind->request = GAPC_LTK_EXCH;
        p_req_ind->data.key_size = key_size;
        ke_msg_send(p_req_ind);
    }
}

/// Callback used to indicate pairing keys that must be stored on a non-volatile memory.
__STATIC void gapc_msg_sec_key_received(uint8_t conidx, uint32_t dest_id, const gapc_pairing_keys_t* p_keys)
{
    struct gapc_bond_ind *p_ind;

    if((p_keys->valid_key_bf & GAP_KDIST_ENCKEY) != 0)
    {
        p_ind = KE_MSG_ALLOC(GAPC_BOND_IND, dest_id, TASK_GAPC, gapc_bond_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->info = GAPC_LTK_EXCH;
            p_ind->data.ltk = p_keys->ltk;
            ke_msg_send(p_ind);
        }
    }

    if((p_keys->valid_key_bf & GAP_KDIST_IDKEY) != 0)
    {
        p_ind = KE_MSG_ALLOC(GAPC_BOND_IND, dest_id, TASK_GAPC, gapc_bond_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->info = GAPC_IRK_EXCH;
            p_ind->data.irk = p_keys->irk;
            ke_msg_send(p_ind);
        }
    }

    if((p_keys->valid_key_bf & GAP_KDIST_SIGNKEY) != 0)
    {
        p_ind = KE_MSG_ALLOC(GAPC_BOND_IND, dest_id, TASK_GAPC, gapc_bond_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->info = GAPC_CSRK_EXCH;
            p_ind->data.csrk = p_keys->csrk;
            ke_msg_send(p_ind);
        }
    }
    #if(BT_HOST_PRESENT)
    if((p_keys->valid_key_bf & GAP_KDIST_LINKKEY) != 0)
    {
        p_ind = KE_MSG_ALLOC(GAPC_BOND_IND, dest_id, TASK_GAPC, gapc_bond_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->info = GAPC_BT_LINK_KEY;
            p_ind->data.bt_link_key.key         = p_keys->link_key;
            p_ind->data.bt_link_key.pairing_lvl = p_keys->pairing_lvl;
            ke_msg_send(p_ind);
        }
    }
    #endif // (BT_HOST_PRESENT)
}
#endif // (BLE_GAPC)

#if (BT_HOST_PRESENT)
__STATIC void gapc_msg_sec_bt_encrypt_req(uint8_t conidx, uint32_t dest_id)
{
    struct gapc_bt_encrypt_req_ind* p_ind = KE_MSG_ALLOC(GAPC_BT_ENCRYPT_REQ_IND, dest_id, TASK_GAPC, gapc_bt_encrypt_req_ind);

    if(p_ind != NULL)
    {
        p_ind->conidx  = conidx;
        ke_msg_send(p_ind);
    }
}


__STATIC void gapc_msg_sec_peer_iocap(uint8_t conidx, uint32_t dest_id, uint8_t iocap, uint8_t auth_req_bf,
                                      bool oob_present)
{
    struct gapc_bond_ind* p_ind = KE_MSG_ALLOC(GAPC_BOND_IND, dest_id, TASK_GAPC, gapc_bond_ind);
    if(p_ind != NULL)
    {
        p_ind->conidx = conidx;
        p_ind->info = GAPC_BT_IOCAP;
        p_ind->data.peer_info.iocap            = iocap;
        p_ind->data.peer_info.auth_req_bf      = auth_req_bf;
        p_ind->data.peer_info.oob_data_present = oob_present;
        ke_msg_send(p_ind);
    }
}

__STATIC void gapc_msg_sec_display_passkey(uint8_t conidx, uint32_t dest_id, uint32_t passkey)
{
    struct gapc_bond_ind* p_ind = KE_MSG_ALLOC(GAPC_BOND_IND, dest_id, TASK_GAPC, gapc_bond_ind);
    if(p_ind != NULL)
    {
        p_ind->conidx = conidx;
        p_ind->info = GAPC_BT_PASSKEY;
        p_ind->data.passkey = passkey;
        ke_msg_send(p_ind);
    }
}
#endif // (BT_HOST_PRESENT)


// ----------------------------------- CONNECTION INFO CALLBACKS ------------------------------------------------



/// Handle connection disconnection
__STATIC void gapc_msg_disconnected(uint8_t conidx, uint32_t dest_id, uint16_t reason)
{
    struct gapc_disconnect_ind * p_ind =
            KE_MSG_ALLOC(GAPC_DISCONNECT_IND, dest_id, TASK_GAPC, gapc_disconnect_ind);
    if(p_ind)
    {
        p_ind->conidx = conidx;
        p_ind->conhdl = gapc_get_conhdl(conidx);
        p_ind->reason = RW_ERR_HL_TO_HCI(reason); // still use controller reason

        ke_msg_send(p_ind);
    }
}

/// Callback executed when connection bond data is updated.
__STATIC void gapc_msg_bond_data_updated(uint8_t conidx, uint32_t dest_id, const gapc_bond_data_updated_t* p_data)
{
    struct gapc_bond_data_update_ind *p_ind = KE_MSG_ALLOC(GAPC_BOND_DATA_UPDATE_IND, dest_id,
                                                           TASK_GAPC, gapc_bond_data_update_ind);
    if(p_ind != NULL)
    {
        p_ind->conidx  = conidx;
        p_ind->data    = *p_data;
        ke_msg_send(p_ind);
    }
}

/// Callback executed when an authenticated payload timeout has been detected.
/// (no encrypted data received after a specific duration ; see LE-PING)
__STATIC void gapc_msg_auth_payload_timeout(uint8_t conidx, uint32_t dest_id)
{
    struct gapc_le_ping_to_ind* p_ind = KE_MSG_ALLOC(GAPC_LE_PING_TO_IND, dest_id, TASK_GAPC, gapc_le_ping_to_ind);
    if(p_ind != NULL)
    {
        p_ind->conidx = conidx;
        ke_msg_send(p_ind);
    }
}

/// Callback executed when all ATT bearer are closed onto a connection
__STATIC void gapc_msg_no_more_att_bearer(uint8_t conidx, uint32_t dest_id)
{
    struct gapc_no_more_att_bearer_error_ind *p_ind = KE_MSG_ALLOC(GAPC_NO_MORE_ATT_BEARER_ERROR_IND, dest_id,
                                                                   TASK_GAPC, gapc_no_more_att_bearer_error_ind);
    if(p_ind != NULL)
    {
        p_ind->conidx = conidx;
        ke_msg_send(p_ind);
    }
}

/// Handle connection disconnection
__STATIC void gapc_msg_cli_hash_info(uint8_t conidx, uint32_t dest_id, uint16_t handle, const uint8_t* p_hash)
{
    struct gapc_peer_att_info_ind* p_ind =  KE_MSG_ALLOC(GAPC_PEER_ATT_INFO_IND, dest_id, TASK_GAPC, gapc_peer_att_info_ind);
    if(p_ind != NULL)
    {
        p_ind->conidx = conidx;
        p_ind->req    = GAPC_DEV_DB_HASH;
        p_ind->handle = handle;
        memcpy(p_ind->info.hash, p_hash, 16);
        ke_msg_send(p_ind);
    }
}

/// Disconnect procedure completed handler
__STATIC void gapc_msg_disconnect_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_DISCONNECT, dest_id, status);
}

/// @brief Handles disconnection of BLE link request.
__STATIC int gapc_msg_disconnect_cmd_handler(ke_msg_id_t const msgid, struct gapc_disconnect_cmd* p_cmd,
                                ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_DISCONNECT)
    {
        // LL error used by message API
        status = gapc_disconnect(conidx, src_id, RW_ERR_HCI_TO_HL(p_cmd->reason), &gapc_msg_disconnect_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}


/// @brief Request application to provide device information
__STATIC void gapc_info_dev_info_get_req_send(uint8_t conidx, uint8_t info, uint32_t dest_id, uint16_t token, uint16_t offset, uint16_t max_length)
{
    // send value request to application.
    struct gapc_get_dev_info_req_ind * p_req_ind = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_REQ_IND, dest_id,
                                                                TASK_GAPC, gapc_get_dev_info_req_ind);

    if(p_req_ind != NULL)
    {
        p_req_ind->conidx           = conidx;
        p_req_ind->req              = info;
        p_req_ind->token            = token;
        p_req_ind->name_offset      = offset;
        p_req_ind->max_name_length  = max_length;
        ke_msg_send(p_req_ind);
    }
}

/// Callback executed when peer request device name information
__STATIC void gapc_msg_info_name_get(uint8_t conidx, uint32_t dest_id, uint16_t token, uint16_t offset,
                                     uint16_t max_length)
{
    gapc_info_dev_info_get_req_send(conidx, GAPC_DEV_NAME, dest_id, token, offset, max_length);
}

/// Callback executed when peer request appearance information
__STATIC void gapc_msg_info_appearance_get(uint8_t conidx, uint32_t dest_id, uint16_t token)
{
    gapc_info_dev_info_get_req_send(conidx, GAPC_DEV_APPEARANCE, dest_id, token, 0, 0);
}

/// Callback executed when peer request slave preferred connection parameters information
__STATIC void gapc_msg_info_slave_pref_param_get(uint8_t conidx, uint32_t dest_id, uint16_t token)
{
    gapc_info_dev_info_get_req_send(conidx, GAPC_DEV_SLV_PREF_PARAMS, dest_id, token, 0, 0);
}

/// Callback executed when peer request modification of device name information
__STATIC void gapc_msg_info_name_set(uint8_t conidx, uint32_t dest_id, uint16_t token, co_buf_t* p_buf)
{
    uint16_t length = co_buf_data_len(p_buf);
    struct gapc_set_dev_info_req_ind *p_req_ind = KE_MSG_ALLOC_DYN(GAPC_SET_DEV_INFO_REQ_IND, dest_id,
            TASK_GAPC, gapc_set_dev_info_req_ind, length);

    if(p_req_ind != NULL)
    {
        p_req_ind->conidx = conidx;
        p_req_ind->req    = GAPC_DEV_NAME;
        p_req_ind->token  = token;

        p_req_ind->info.name.value_length = length;
        memcpy(&(p_req_ind->info.name.value), co_buf_data(p_buf), length);

        ke_msg_send(p_req_ind);
    }

}

/// Callback executed when peer request modification of device appearance information
__STATIC void gapc_msg_info_appearance_set(uint8_t conidx, uint32_t dest_id, uint16_t token, uint16_t appearance)
{
    struct gapc_set_dev_info_req_ind *p_req_ind = KE_MSG_ALLOC(GAPC_SET_DEV_INFO_REQ_IND, dest_id,
                                                           TASK_GAPC, gapc_set_dev_info_req_ind);

    if(p_req_ind != NULL)
    {
        p_req_ind->conidx = conidx;
        p_req_ind->req    = GAPC_DEV_APPEARANCE;
        p_req_ind->token  = token;
        p_req_ind->info.appearance = appearance;

        ke_msg_send(p_req_ind);
    }
}

#if (BLE_GAPC)
 /// @brief Callback executed once a connection has been established. The upper layer software shall
__STATIC void gapc_msg_le_connection_req(uint8_t conidx, uint32_t dest_id, uint8_t actv_idx, uint8_t role,
                                         const gap_bdaddr_t* p_peer_addr, const gap_le_con_param_t* p_con_params,
                                         uint8_t clk_accuracy)
{
    // Send connection indication message to task that requests connection.
    struct gapc_le_connection_req_ind * p_req_ind =
            KE_MSG_ALLOC(GAPC_LE_CONNECTION_REQ_IND, dest_id, TASK_GAPC, gapc_le_connection_req_ind);

    if(p_req_ind)
    {
        p_req_ind->conidx         = conidx;
        p_req_ind->conhdl         = gapc_get_conhdl(conidx);
        p_req_ind->peer_addr_type = p_peer_addr->addr_type;
        memcpy(&(p_req_ind->peer_addr), p_peer_addr->addr, sizeof(gap_addr_t));
        p_req_ind->con_interval   = p_con_params->interval;
        p_req_ind->con_latency    = p_con_params->latency;
        p_req_ind->sup_to         = p_con_params->sup_to;
        p_req_ind->clk_accuracy   = clk_accuracy;
        p_req_ind->role           = role;
        ke_msg_send(p_req_ind);
    }
}

/// Callback executed when connection parameter update is requested
/// @see gapc_le_con_param_update_cfm shall be called to confirm new parameters.
__STATIC void gapc_msg_le_config_param_update_req(uint8_t conidx, uint32_t dest_id, const gap_le_con_param_nego_t* p_param)
{
    struct gapc_param_update_req_ind* p_req_ind = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_REQ_IND, dest_id, TASK_GAPC,
                                                               gapc_param_update_req_ind);
    if(p_req_ind)
    {
        p_req_ind->conidx   = conidx;
        p_req_ind->intv_max = p_param->interval_max;
        p_req_ind->intv_min = p_param->interval_min;
        p_req_ind->latency  = p_param->latency;
        p_req_ind->time_out = p_param->sup_to;
        ke_msg_send(p_req_ind);
    }
}

/// Callback executed when connection parameter are updated
__STATIC void gapc_msg_le_config_param_updated(uint8_t conidx, uint32_t dest_id, const gap_le_con_param_t* p_param)
{
    struct gapc_param_updated_ind *p_ind = KE_MSG_ALLOC(GAPC_PARAM_UPDATED_IND, dest_id, TASK_GAPC, gapc_param_updated_ind);

    if(p_ind)
    {
        p_ind->conidx       = conidx;
        p_ind->con_interval = p_param->interval;
        p_ind->con_latency  = p_param->latency;
        p_ind->sup_to       = p_param->sup_to;

        ke_msg_send(p_ind);
    }
}

/// Callback executed when data length over the air has been updated
__STATIC void gapc_msg_le_config_packet_size_updated(uint8_t conidx, uint32_t dest_id, uint16_t max_tx_octets ,
                                                     uint16_t max_tx_time, uint16_t max_rx_octets , uint16_t max_rx_time)
{
    struct gapc_le_pkt_size_ind* p_ind = KE_MSG_ALLOC(GAPC_LE_PKT_SIZE_IND, dest_id, TASK_GAPC, gapc_le_pkt_size_ind);

    if(p_ind != NULL)
    {
        p_ind->conidx         = conidx;
        p_ind->max_rx_octets  = max_rx_octets;
        p_ind->max_rx_time    = max_rx_time;
        p_ind->max_tx_octets  = max_tx_octets;
        p_ind->max_tx_time    = max_tx_time;

        ke_msg_send(p_ind);
    }
}

/// Callback executed when LE PHY is updated
__STATIC void gapc_msg_le_config_phy_updated(uint8_t conidx, uint32_t dest_id, uint8_t tx_phy , uint8_t rx_phy)
{
    struct gapc_le_phy_ind* p_ind = KE_MSG_ALLOC(GAPC_LE_PHY_IND, dest_id, TASK_GAPC, gapc_le_phy_ind);

    if(p_ind != NULL)
    {
        p_ind->conidx = conidx;
        p_ind->rx_phy = rx_phy;
        p_ind->tx_phy = tx_phy;

        ke_msg_send(p_ind);
    }
}


#if (BLE_PWR_CTRL)
/// Callback executed when a TX power change report is received
__STATIC void gapc_msg_le_power_tx_change_report(uint8_t conidx, uint32_t dest_id, bool local,
                                                 const gapc_tx_power_report_t* p_report)
{
    // Trigger local power change indication
    gapc_loc_tx_pwr_report_ind_t *p_ind = KE_MSG_ALLOC(local ? GAPC_LOC_TX_PWR_REPORT_IND : GAPC_PEER_TX_PWR_REPORT_IND,
                                                       dest_id, TASK_GAPC, gapc_loc_tx_pwr_report_ind);
    if(p_ind != NULL)
    {
        p_ind->conidx = conidx;
        p_ind->phy    = p_report->phy;
        p_ind->tx_pwr = p_report->tx_pwr;
        p_ind->flags  = p_report->flags;
        p_ind->delta  = p_report->delta;
        ke_msg_send(p_ind);
    }
}

/// Callback executed when a Path Loss threshold report event is received
__STATIC void gapc_msg_le_power_path_loss_threshold_report(uint8_t conidx, uint32_t dest_id, uint8_t curr_path_loss,
                                                           uint8_t zone_entered)
{
    // Trigger Path Loss Threshold Indication
    gapc_path_loss_threshold_ind_t *p_ind =
                KE_MSG_ALLOC(GAPC_PATH_LOSS_THRESHOLD_IND, dest_id, TASK_GAPC, gapc_path_loss_threshold_ind);
    if(p_ind != NULL)
    {
        p_ind->conidx         = conidx;
        p_ind->curr_path_loss = curr_path_loss;
        p_ind->zone_entered   = zone_entered;

        ke_msg_send(p_ind);
    }
}
#endif // (BLE_PWR_CTRL)

#if (BLE_AOA || BLE_AOD)
/// Callback executed when an IQ report has been received
__STATIC void gapc_msg_le_cte_iq_report_received(uint8_t conidx, uint32_t dest_id, const gapc_iq_report_info_t* p_report,
                                                 uint8_t nb_samples, const gap_iq_sample_t* p_samples)
{
    struct gapc_cte_iq_report_ind *p_ind = KE_MSG_ALLOC_DYN(GAPC_CTE_IQ_REPORT_IND, dest_id, TASK_GAPC,
                                                            gapc_cte_iq_report_ind, nb_samples * sizeof(gap_iq_sample_t));
    if(p_ind != NULL)
    {
        p_ind->conidx           = conidx;
        p_ind->rx_phy           = p_report->rx_phy;
        p_ind->data_channel_idx = p_report->channel_idx;
        p_ind->rssi             = p_report->rssi;
        p_ind->rssi_antenna_id  = p_report->rssi_antenna_id;
        p_ind->cte_type         = p_report->cte_type;
        p_ind->slot_dur         = p_report->slot_dur;
        p_ind->pkt_status       = p_report->pkt_status;
        p_ind->con_evt_cnt      = p_report->con_evt_cnt;
        p_ind->nb_samples       = nb_samples;
        memcpy(p_ind->sample, p_samples, sizeof(gap_iq_sample_t) * nb_samples);

        ke_msg_send(p_ind);
    }
}

/// Callback executed when a CTE request failed event is triggered by controller
__STATIC void gapc_msg_le_cte_request_failed_event(uint8_t conidx, uint32_t dest_id, uint16_t reason)
{
    struct gapc_cte_req_failed_ind *p_ind = KE_MSG_ALLOC(GAPC_CTE_REQ_FAILED_IND, dest_id,
            TASK_GAPC, gapc_cte_req_failed_ind);

    if(p_ind != NULL)
    {
        p_ind->conidx = conidx;
        p_ind->status = reason;
        ke_msg_send(p_ind);
    }
}
#endif // (BLE_AOA || BLE_AOD)

/// LE Connection parameter procedure completed handler
__STATIC void gapc_le_con_param_update_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_UPDATE_PARAMS, dest_id, status);
}

/// @brief Handles request to Update connection parameters.
__STATIC int gapc_param_update_cmd_handler(ke_msg_id_t const msgid, struct  gapc_param_update_cmd* p_cmd,
                                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_UPDATE_PARAMS)
    {
        gap_le_con_param_nego_with_ce_len_t param;
        memcpy(&param, &p_cmd->intv_min, sizeof(param));

        status = gapc_le_con_param_update(conidx, src_id, &param, gapc_le_con_param_update_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}

/// @brief Confirmation or not of Slave connection parameters
__STATIC int gapc_param_update_cfm_handler(ke_msg_id_t const msgid, struct gapc_param_update_cfm const* p_cfm,
                                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    gapc_le_con_param_update_cfm(p_cfm->conidx, p_cfm->accept, p_cfm->ce_len_min, p_cfm->ce_len_max);
    return (KE_MSG_CONSUMED);
}

/// LE PHY Update parameter procedure completed handler
__STATIC void gapc_le_phy_update_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_SET_PHY, dest_id, status);
}

/// @brief Handles the reception of GAPC_SET_PHY_CMD message
__STATIC int gapc_set_phy_cmd_handler(ke_msg_id_t const msgid, struct gapc_set_phy_cmd* p_cmd,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_SET_PHY)
    {
        status = gapc_le_set_phy(conidx, src_id, p_cmd->tx_phy, p_cmd->rx_phy, p_cmd->phy_opt, gapc_le_phy_update_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}


#if (HL_LE_PERIPHERAL)
/// set slave latency procedure completed handler
__STATIC void gapc_le_set_pref_slave_latency_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_SET_PREF_SLAVE_LATENCY, dest_id, status);
}

/// @brief Handles the reception of GAPC_SET_PREF_SLAVE_LATENCY_CMD message
int gapc_set_pref_slave_latency_cmd_handler(ke_msg_id_t const msgid, struct gapc_set_pref_slave_latency_cmd* p_cmd,
                                            ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_SET_PREF_SLAVE_LATENCY)
    {
        status = gapc_le_set_pref_slave_latency(conidx, src_id, p_cmd->latency, gapc_le_set_pref_slave_latency_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}

/// set slave pref evt duration procedure completed handler
__STATIC void gapc_le_set_pref_slave_evt_dur_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_SET_PREF_SLAVE_EVT_DUR, dest_id, status);
}

/// @brief Handles the reception of GAPC_SET_PREF_SLAVE_EVT_DUR_CMD message
int gapc_set_pref_slave_evt_dur_cmd_handler(ke_msg_id_t const msgid, struct gapc_set_pref_slave_evt_dur_cmd* p_cmd,
                                            ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_SET_PREF_SLAVE_EVT_DUR)
    {
        status = gapc_le_set_pref_slave_evt_dur(conidx, src_id, p_cmd->duration, p_cmd->single_tx,
                                                gapc_le_set_pref_slave_evt_dur_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}
#endif // (HL_LE_PERIPHERAL)

/// set rx max size and time procedure completed handler
__STATIC void gapc_le_set_max_rx_size_and_time_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_SET_MAX_RX_SIZE_AND_TIME, dest_id, status);
}


/// * @brief Handles the reception of GAPC_SET_MAX_RX_SIZE_AND_TIME_CMD message
__STATIC int gapc_set_max_rx_size_and_time_cmd_handler(ke_msg_id_t const msgid, struct gapc_set_max_rx_size_and_time_cmd* p_cmd,
                                                       ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_SET_MAX_RX_SIZE_AND_TIME)
    {
        status = gapc_le_set_max_rx_size_and_time(conidx, src_id, p_cmd->rx_octets, p_cmd->rx_time,
                                                  gapc_le_set_max_rx_size_and_time_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}

/// set packet size procedure completed handler
__STATIC void gapc_le_set_pkt_size_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_SET_LE_PKT_SIZE, dest_id, status);
}

/// @brief Handles the reception of GAPC_SET_LE_PKT_SIZE_CMD message
__STATIC int gapc_set_le_pkt_size_handler(ke_msg_id_t const msgid, struct gapc_set_le_pkt_size_cmd *p_cmd,
                                          ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_SET_LE_PKT_SIZE)
    {
        status = gapc_le_set_pkt_size(conidx, src_id, p_cmd->tx_octets, p_cmd->tx_time, gapc_le_set_pkt_size_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}

/// set packet size procedure completed handler
__STATIC void gapc_le_per_adv_sync_transfer_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_PER_ADV_SYNC_TRANS, dest_id, status);
}

/// @brief Handles request to retrieve connection informations.
__STATIC int gapc_per_adv_sync_trans_cmd_handler(ke_msg_id_t const msgid, struct gapc_per_adv_sync_trans_cmd *p_cmd,
                                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_PER_ADV_SYNC_TRANS)
    {
        status = gapc_le_per_adv_sync_transfer(conidx, src_id, p_cmd->actv_idx, p_cmd->service_data,
                                               gapc_le_per_adv_sync_transfer_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}


#if (BLE_PWR_CTRL)
/// Procedure completed handler
__STATIC void gapc_le_tx_pwr_report_ctrl_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_TX_PWR_REPORT_CTRL, dest_id, status);
}

/// @brief Handles control of TX power changes reports
__STATIC int gapc_tx_pwr_report_ctrl_cmd_handler(ke_msg_id_t const msgid, gapc_tx_pwr_report_ctrl_cmd_t *p_cmd,
                                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_TX_PWR_REPORT_CTRL)
    {
        status = gapc_le_tx_pwr_report_ctrl(conidx, src_id, p_cmd->local_en, p_cmd->remote_en,
                                            gapc_le_tx_pwr_report_ctrl_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}

/// Procedure completed handler
__STATIC void gapc_le_path_loss_ctrl_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_PATH_LOSS_REPORT_CTRL, dest_id, status);
}

/// @brief Handles control Path Loss Threshold
__STATIC int gapc_path_loss_ctrl_cmd_handler(ke_msg_id_t const msgid, gapc_path_loss_ctrl_cmd_t *p_cmd,
                                             ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_PATH_LOSS_REPORT_CTRL)
    {
        if(p_cmd->enable)
        {
            status = gapc_le_path_loss_enable(conidx, src_id, p_cmd->high_threshold, p_cmd->high_hysteresis,
                                              p_cmd->low_threshold, p_cmd->low_hysteresis, p_cmd->min_time,
                                              gapc_le_path_loss_ctrl_cmp_cb);
        }
        else
        {
            status = gapc_le_path_loss_disable(conidx, src_id, gapc_le_path_loss_ctrl_cmp_cb);
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_PWR_CTRL)

#if (BLE_GATT_CLI)
/// Procedure completed handler
__STATIC void gapc_client_features_enable_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_CLI_FEAT_EN, dest_id, status);
}

/// Enable robust caching onto peer device
__STATIC int gapc_cli_feat_en_cmd_handler(ke_msg_id_t const msgid, struct gapc_operation_cmd *p_cmd,
                                          ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_CLI_FEAT_EN)
    {
        status = gapc_client_features_enable(conidx, src_id, gapc_client_features_enable_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_GATT_CLI)

#if (BLE_AOA || BLE_AOD)
/// Procedure completed handler
__STATIC void gapc_le_cte_tx_configure_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_CTE_TX_CFG, dest_id, status);
}

/// @brief Handles modification of CTE transmission parameters.
__STATIC int gapc_cte_tx_cfg_cmd_handler(ke_msg_id_t const msgid, struct gapc_cte_tx_cfg_cmd *p_cmd,
                                ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_CTE_TX_CFG)
    {
        status = gapc_le_cte_tx_configure(conidx, src_id, p_cmd->cte_types, p_cmd->switching_pattern_len, p_cmd->antenna_id,
                                          gapc_le_cte_tx_configure_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}

/// Procedure completed handler
__STATIC void gapc_le_cte_rx_configure_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_CTE_RX_CFG, dest_id, status);
}

/// @brief Handles modification of CTE reception parameters.
__STATIC int gapc_cte_rx_cfg_cmd_handler(ke_msg_id_t const msgid, struct gapc_cte_rx_cfg_cmd *p_cmd,
                                ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_CTE_RX_CFG)
    {
        status = gapc_le_cte_rx_configure(conidx, src_id, p_cmd->sample_en, p_cmd->slot_dur, p_cmd->switching_pattern_len,
                                          p_cmd->antenna_id, gapc_le_cte_rx_configure_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}

/// Procedure completed handler
__STATIC void gapc_le_cte_request_control_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_CTE_REQ_CTRL, dest_id, status);
}

/// @brief Handles control of CTE request control.
__STATIC int gapc_cte_req_ctrl_cmd_handler(ke_msg_id_t const msgid, struct gapc_cte_req_ctrl_cmd *p_cmd,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_CTE_REQ_CTRL)
    {
        status = gapc_le_cte_request_control(conidx, src_id, p_cmd->enable, p_cmd->interval, p_cmd->cte_len,
                                             p_cmd->cte_type, gapc_le_cte_request_control_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}

/// Procedure completed handler
__STATIC void gapc_le_cte_response_control_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_CTE_RSP_CTRL, dest_id, status);
}

/// @brief Handles control of CTE response control.
__STATIC int gapc_cte_rsp_ctrl_cmd_handler(ke_msg_id_t const msgid, struct gapc_cte_rsp_ctrl_cmd *p_cmd,
                                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_CTE_RSP_CTRL)
    {
        status = gapc_le_cte_response_control(conidx, src_id, p_cmd->enable, gapc_le_cte_response_control_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_AOA || BLE_AOD)


#if (HL_LE_CENTRAL)
/// Procedure completed handler
__STATIC void gapc_msg_le_smp_encrypt_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_LE_ENCRYPT, dest_id, status);
}

/// @brief Handles request to Start an Encryption procedure.
__STATIC int gapc_le_smp_encrypt_cmd_handler(ke_msg_id_t const msgid, struct  gapc_le_encrypt_cmd* p_cmd,
                                          ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_LE_ENCRYPT)
    {
        status = gapc_le_encrypt(conidx, src_id, &(p_cmd->ltk), gapc_msg_le_smp_encrypt_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}


/// @brief Handles request to Start a bonding procedure.
__STATIC int gapc_le_smp_bond_cmd_handler(ke_msg_id_t const msgid, struct  gapc_bond_cmd* p_cmd,
                                          ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_BOND)
    {
        status = gapc_le_start_pairing(conidx, &(p_cmd->pairing), p_cmd->sec_req_level);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}
#endif // (HL_LE_CENTRAL)


#if (HL_LE_PERIPHERAL)
/// @brief Handles request to start security request procedure.
int gapc_le_smp_security_cmd_handler(ke_msg_id_t const msgid, struct  gapc_security_cmd *p_cmd,
                              ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_SECURITY_REQ)
    {
        status = gapc_le_send_security_request(conidx, p_cmd->auth);
    }

    gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);

    return (KE_MSG_CONSUMED);
}
#endif // (HL_LE_PERIPHERAL)

/// @brief Handles Keypress notification request
int gapc_le_smp_key_press_notification_cmd_handler(ke_msg_id_t const msgid, struct gapc_key_press_notif_cmd* p_cmd,
                                            ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_KEY_PRESS_NOTIFICATION)
    {
        status = gapc_send_key_pressed_notification(conidx, p_cmd->notification_type);
    }

    gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);

    return (KE_MSG_CONSUMED);
}

#if (HL_LE_PERIPHERAL)
/// @brief Handle reception of application confirmation for encryption procedure.
__STATIC int gapc_le_smp_encrypt_cfm_handler(ke_msg_id_t const msgid, struct gapc_le_encrypt_cfm *p_cfm,
                                          ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    gapc_le_encrypt_req_reply(p_cfm->conidx, p_cfm->found, &(p_cfm->ltk), p_cfm->key_size);

    return (KE_MSG_CONSUMED);
}
#endif // (HL_LE_PERIPHERAL)

#endif // (BLE_GAPC)


/// set packet size procedure completed handler
__STATIC void gapc_le_set_auth_payload_timeout_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status)
{
    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_SET_LE_PING_TO, dest_id, status);
}

/// @brief Handles the reception of GAPC_LE_PING_CMD message
__STATIC int gapc_set_le_ping_to_handler(ke_msg_id_t const msgid, struct gapc_set_le_ping_to_cmd* p_cmd,
                                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_SET_LE_PING_TO)
    {
        status = gapc_set_auth_payload_timeout(conidx, src_id, p_cmd->timeout, gapc_le_set_auth_payload_timeout_cmp_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}

#if (BLE_GATT_CLI)
/// Callback executed when read attribute name procedure is completed.
__STATIC void gapc_msg_read_device_name_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status, uint16_t handle,
                                      co_buf_t* p_name)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        uint16_t length = co_buf_data_len(p_name);
        struct gapc_peer_att_info_ind * p_ind = KE_MSG_ALLOC_DYN(GAPC_PEER_ATT_INFO_IND, dest_id, TASK_GAPC,
                                                                 gapc_peer_att_info_ind, length);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->handle = handle;
            p_ind->req    = GAPC_DEV_NAME;
            p_ind->info.name.value_length = length;
            memcpy(p_ind->info.name.value, co_buf_data(p_name), length);
            ke_msg_send(p_ind);
        }
    }

    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_GET_PEER_NAME, dest_id, status);
}

/// Callback executed when read attribute appearance procedure is completed.
__STATIC void gapc_msg_read_appearance_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status, uint16_t handle,
                                            uint16_t appearance)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapc_peer_att_info_ind * p_ind = KE_MSG_ALLOC(GAPC_PEER_ATT_INFO_IND, dest_id, TASK_GAPC,
                                                                 gapc_peer_att_info_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->handle = handle;
            p_ind->req    = GAPC_DEV_APPEARANCE;
            p_ind->info.appearance = appearance;
            ke_msg_send(p_ind);
        }
    }

    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_GET_PEER_APPEARANCE, dest_id, status);
}

/// Callback executed when read attribute peripheral preferred parameters procedure is completed.
__STATIC void gapc_msg_read_periph_pref_param_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status, uint16_t handle,
                                                     const gap_periph_pref_t* p_param)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapc_peer_att_info_ind * p_ind = KE_MSG_ALLOC(GAPC_PEER_ATT_INFO_IND, dest_id, TASK_GAPC,
                                                                 gapc_peer_att_info_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->handle = handle;
            p_ind->req    = GAPC_DEV_SLV_PREF_PARAMS;
            p_ind->info.slv_pref_params = *p_param;
            ke_msg_send(p_ind);
        }
    }

    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_GET_PEER_SLV_PREF_PARAMS, dest_id, status);
}

/// Callback executed when read central address resolution supported parameters procedure is completed.
__STATIC void gapc_msg_read_central_addr_resol_supp_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status, uint16_t handle,
                                                           uint8_t central_addr_resol)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapc_peer_att_info_ind * p_ind = KE_MSG_ALLOC(GAPC_PEER_ATT_INFO_IND, dest_id, TASK_GAPC,
                                                             gapc_peer_att_info_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->handle = handle;
            p_ind->req    = GAPC_DEV_CTL_ADDR_RESOL;
            p_ind->info.ctl_addr_resol = central_addr_resol;
            ke_msg_send(p_ind);
        }
    }

    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_GET_ADDR_RESOL_SUPP, dest_id, status);
}

/// Callback executed when read attribute database hash procedure is completed.
__STATIC void gapc_msg_read_database_hash_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status, uint16_t handle,
                                               const uint8_t* p_hash)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapc_peer_att_info_ind * p_ind = KE_MSG_ALLOC(GAPC_PEER_ATT_INFO_IND, dest_id, TASK_GAPC,
                                                             gapc_peer_att_info_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->handle = handle;
            p_ind->req    = GAPC_DEV_DB_HASH;
            memcpy(p_ind->info.hash, p_hash, GATT_DB_HASH_LEN);
            ke_msg_send(p_ind);
        }
    }

    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_GET_PEER_DB_HASH, dest_id, status);
}

/// Callback executed when read attribute resolvable private address only procedure is completed.
__STATIC void gapc_msg_read_rslv_priv_addr_only_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status, uint16_t handle,
                                                    uint8_t rslv_priv_addr_only)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapc_peer_att_info_ind * p_ind = KE_MSG_ALLOC(GAPC_PEER_ATT_INFO_IND, dest_id, TASK_GAPC,
                                                             gapc_peer_att_info_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->handle = handle;
            p_ind->req    = GAPC_GET_RSLV_PRIV_ADDR_ONLY;
            p_ind->info.rslv_priv_addr_only = rslv_priv_addr_only;
            ke_msg_send(p_ind);
        }
    }

    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_GET_RSLV_PRIV_ADDR_ONLY, dest_id, status);
}
#endif // (BLE_GATT_CLI)

/// Callback executed when get peer version procedure is completed.
__STATIC void gapc_msg_get_peer_version_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status,
                                               const gapc_version_t* p_version)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapc_peer_version_ind * p_ind = KE_MSG_ALLOC(GAPC_PEER_VERSION_IND, dest_id, TASK_GAPC,
                                                            gapc_peer_version_ind);

        if(p_ind != NULL)
        {
            p_ind->conidx      = conidx;
            p_ind->compid      = p_version->company_id;
            p_ind->lmp_subvers = p_version->lmp_subversion;
            p_ind->lmp_vers    = p_version->lmp_version;
            ke_msg_send(p_ind);
        }
    }

    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_GET_PEER_VERSION, dest_id, status);
}

/// Callback executed when get RSSI value procedure is completed.
__STATIC void gapc_msg_get_rssi_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status, int8_t rssi)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapc_con_rssi_ind * p_ind = KE_MSG_ALLOC(GAPC_CON_RSSI_IND, dest_id, TASK_GAPC, gapc_con_rssi_ind);

        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->rssi   = rssi;
            ke_msg_send(p_ind);
        }
    }

    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_GET_CON_RSSI, dest_id, status);
}

/// Callback executed when get authenticated payload timeout value procedure is completed.
__STATIC void gapc_msg_get_auth_payload_to_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status, uint16_t timeout)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapc_le_ping_to_val_ind *p_ind = KE_MSG_ALLOC(GAPC_LE_PING_TO_VAL_IND, dest_id, TASK_GAPC, gapc_le_ping_to_val_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx  = conidx;
            p_ind->timeout = timeout;
            ke_msg_send(p_ind);
        }
    }

    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_GET_LE_PING_TO, dest_id, status);
}

#if (BLE_GAPC)
/// Callback executed when get LE channel map for connection procedure is completed.
__STATIC void gapc_msg_get_le_chmap_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status,
                                         const le_chnl_map_t* p_ch_map)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapc_con_channel_map_ind * p_ind = KE_MSG_ALLOC(GAPC_CON_CHANNEL_MAP_IND, dest_id, TASK_GAPC, gapc_con_channel_map_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->ch_map = *p_ch_map;
            ke_msg_send(p_ind);
        }
    }

    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_GET_CON_CHANNEL_MAP, dest_id, status);
}

/// Callback executed when get LE peer supported features value procedure is completed.
__STATIC void gapc_msg_get_le_peer_features_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status,
                                                 const uint8_t* p_features)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapc_peer_features_ind * p_ind =  KE_MSG_ALLOC(GAPC_PEER_FEATURES_IND, dest_id, TASK_GAPC, gapc_peer_features_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            memcpy(p_ind->features, p_features, LE_FEATS_LEN);
            ke_msg_send(p_ind);
        }
    }

    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_GET_PEER_FEATURES, dest_id, status);
}

/// Callback executed when get LE connection used PHY value procedure is completed.
__STATIC void gapc_msg_get_le_phy_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status,
                                         uint8_t tx_phy, uint8_t rx_phy)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapc_le_phy_ind* p_ind = KE_MSG_ALLOC(GAPC_LE_PHY_IND, dest_id, TASK_GAPC, gapc_le_phy_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->rx_phy = rx_phy;
            p_ind->tx_phy = tx_phy;

            ke_msg_send(p_ind);
        }
    }

    gapc_msg_send_cmd_cmp_evt(conidx, GAPC_GET_PHY, dest_id, status);
}
#endif // (BLE_GAPC)

#if (BLE_PWR_CTRL && BLE_GAPC)
/// Callback executed when get LE connection local transmit power level information procedure is completed.
__STATIC void gapc_msg_le_get_local_tx_power_level_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status, uint8_t operation,
                                                          uint8_t phy, int8_t power_level, int8_t max_power_level)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        // Trigger Local power indication
        gapc_loc_tx_pwr_ind_t *p_ind = KE_MSG_ALLOC(GAPC_LOC_TX_PWR_IND, dest_id, TASK_GAPC, gapc_loc_tx_pwr_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx     = conidx;
            p_ind->phy        = phy;
            p_ind->tx_pwr     = power_level;
            p_ind->max_tx_pwr = max_power_level;

            ke_msg_send(p_ind);
        }
    }

    gapc_msg_send_cmd_cmp_evt(conidx, operation, dest_id, status);
}

__STATIC void gapc_msg_le_get_local_tx_power_level_1m_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status,
                                                          uint8_t phy, int8_t power_level, int8_t max_power_level)
{
    gapc_msg_le_get_local_tx_power_level_cmp_cb(conidx, dest_id, status, GAPC_GET_LOC_TX_PWR_LEVEL_1M, phy, power_level, max_power_level);
}
__STATIC void gapc_msg_le_get_local_tx_power_level_2m_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status,
                                                          uint8_t phy, int8_t power_level, int8_t max_power_level)
{
    gapc_msg_le_get_local_tx_power_level_cmp_cb(conidx, dest_id, status, GAPC_GET_LOC_TX_PWR_LEVEL_2M, phy, power_level, max_power_level);
}
__STATIC void gapc_msg_le_get_local_tx_power_level_s8_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status,
                                                          uint8_t phy, int8_t power_level, int8_t max_power_level)
{
    gapc_msg_le_get_local_tx_power_level_cmp_cb(conidx, dest_id, status, GAPC_GET_LOC_TX_PWR_LEVEL_LE_CODED_S8, phy, power_level, max_power_level);
}
__STATIC void gapc_msg_le_get_local_tx_power_level_s2_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status,
                                                          uint8_t phy, int8_t power_level, int8_t max_power_level)
{
    gapc_msg_le_get_local_tx_power_level_cmp_cb(conidx, dest_id, status, GAPC_GET_LOC_TX_PWR_LEVEL_LE_CODED_S2, phy, power_level, max_power_level);
}

/// Callback executed when LE remote transmit power level read procedure is completed.
__STATIC void gapc_msg_le_get_peer_tx_power_level_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status, uint8_t operation,
                                                         uint8_t phy, int8_t power_level, uint8_t flags)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        // Trigger Remote power indication
        gapc_peer_tx_pwr_ind_t *p_ind = KE_MSG_ALLOC(GAPC_PEER_TX_PWR_IND, dest_id, TASK_GAPC, gapc_peer_tx_pwr_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->phy    = phy;
            p_ind->tx_pwr = power_level;
            p_ind->flags  = flags;

            ke_msg_send(p_ind);
        }
    }

    gapc_msg_send_cmd_cmp_evt(conidx, operation, dest_id, status);
}

__STATIC void gapc_msg_le_get_peer_tx_power_level_1m_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status,
                                                             uint8_t phy, int8_t power_level, uint8_t flags)
{
    gapc_msg_le_get_peer_tx_power_level_cmp_cb(conidx, dest_id, status, GAPC_GET_PEER_TX_PWR_LEVEL_1M, phy, power_level, flags);
}
__STATIC void gapc_msg_le_get_peer_tx_power_level_2m_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status,
                                                             uint8_t phy, int8_t power_level, uint8_t flags)
{
    gapc_msg_le_get_peer_tx_power_level_cmp_cb(conidx, dest_id, status, GAPC_GET_PEER_TX_PWR_LEVEL_2M, phy, power_level, flags);
}
__STATIC void gapc_msg_le_get_peer_tx_power_level_s8_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status,
                                                             uint8_t phy, int8_t power_level, uint8_t flags)
{
    gapc_msg_le_get_peer_tx_power_level_cmp_cb(conidx, dest_id, status, GAPC_GET_PEER_TX_PWR_LEVEL_LE_CODED_S8, phy, power_level, flags);
}
__STATIC void gapc_msg_le_get_peer_tx_power_level_s2_cmp_cb(uint8_t conidx, uint32_t dest_id, uint16_t status,
                                                             uint8_t phy, int8_t power_level, uint8_t flags)
{
    gapc_msg_le_get_peer_tx_power_level_cmp_cb(conidx, dest_id, status, GAPC_GET_PEER_TX_PWR_LEVEL_LE_CODED_S2, phy, power_level, flags);
}

#endif // (BLE_PWR_CTRL && BLE_GAPC)

///  @brief Handles request to retrieve connection informations.
__STATIC int gapc_msg_get_info_cmd_handler(ke_msg_id_t const msgid, struct gapc_get_info_cmd *p_cmd,
                              ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;
    bool send_cmp_evt = false;

    switch(p_cmd->operation)
    {
        #if (BLE_GATT_CLI)
        case GAPC_GET_PEER_NAME:            { status = gapc_read_device_name(conidx, src_id, gapc_msg_read_device_name_cmp_cb); } break;
        case GAPC_GET_PEER_APPEARANCE:      { status = gapc_read_appearance(conidx, src_id, gapc_msg_read_appearance_cmp_cb); } break;
        case GAPC_GET_PEER_SLV_PREF_PARAMS: { status = gapc_read_periph_pref_param(conidx, src_id, gapc_msg_read_periph_pref_param_cmp_cb); } break;
        case GAPC_GET_ADDR_RESOL_SUPP:      { status = gapc_read_central_addr_resol_supp(conidx, src_id, gapc_msg_read_central_addr_resol_supp_cmp_cb); } break;
        case GAPC_GET_PEER_DB_HASH:         { status = gapc_read_database_hash(conidx, src_id, gapc_msg_read_database_hash_cmp_cb); } break;
        case GAPC_GET_RSLV_PRIV_ADDR_ONLY:  { status = gapc_read_rslv_priv_addr_only(conidx, src_id, gapc_msg_read_rslv_priv_addr_only_cmp_cb); } break;
        #else
        case GAPC_GET_PEER_NAME:
        case GAPC_GET_PEER_APPEARANCE:
        case GAPC_GET_PEER_SLV_PREF_PARAMS:
        case GAPC_GET_ADDR_RESOL_SUPP:
        case GAPC_GET_PEER_DB_HASH:
        case GAPC_GET_RSLV_PRIV_ADDR_ONLY:  { status = GAP_ERR_NOT_SUPPORTED; } break;
        #endif // (BLE_GATT_CLI)
        case GAPC_GET_PEER_VERSION:         { status = gapc_get_peer_version(conidx, src_id, gapc_msg_get_peer_version_cmp_cb); } break;
        case GAPC_GET_CON_RSSI:             { status = gapc_get_rssi(conidx, src_id, gapc_msg_get_rssi_cmp_cb); } break;
        case GAPC_GET_LE_PING_TO:           { status = gapc_get_auth_payload_to(conidx, src_id, gapc_msg_get_auth_payload_to_cmp_cb); } break;
        #if (BLE_GAPC)
        case GAPC_GET_PEER_FEATURES:        { status = gapc_get_le_peer_features(conidx, src_id, gapc_msg_get_le_peer_features_cmp_cb); } break;
        case GAPC_GET_CON_CHANNEL_MAP:      { status = gapc_get_le_chmap(conidx, src_id, gapc_msg_get_le_chmap_cmp_cb); } break;
        case GAPC_GET_PHY:                  { status = gapc_get_le_phy(conidx, src_id, gapc_msg_get_le_phy_cmp_cb); } break;
        #else
        case GAPC_GET_PEER_FEATURES:
        case GAPC_GET_CON_CHANNEL_MAP:
        case GAPC_GET_PHY:                  { status = GAP_ERR_NOT_SUPPORTED; } break;
        #endif // (BLE_GAPC)

        #if (BLE_PWR_CTRL && BLE_GAPC)
        case GAPC_GET_LOC_TX_PWR_LEVEL_1M:           { status = gapc_le_get_local_tx_power_level(conidx, src_id, GAPC_PHY_PWR_1MBPS_VALUE, gapc_msg_le_get_local_tx_power_level_1m_cmp_cb); } break;
        case GAPC_GET_LOC_TX_PWR_LEVEL_2M:           { status = gapc_le_get_local_tx_power_level(conidx, src_id, GAPC_PHY_PWR_2MBPS_VALUE, gapc_msg_le_get_local_tx_power_level_2m_cmp_cb); } break;
        case GAPC_GET_LOC_TX_PWR_LEVEL_LE_CODED_S8:  { status = gapc_le_get_local_tx_power_level(conidx, src_id, GAPC_PHY_PWR_S8_CODED_VALUE, gapc_msg_le_get_local_tx_power_level_s8_cmp_cb); } break;
        case GAPC_GET_LOC_TX_PWR_LEVEL_LE_CODED_S2:  { status = gapc_le_get_local_tx_power_level(conidx, src_id, GAPC_PHY_PWR_S2_CODED_VALUE, gapc_msg_le_get_local_tx_power_level_s2_cmp_cb); } break;
        case GAPC_GET_PEER_TX_PWR_LEVEL_1M:          { status = gapc_le_get_peer_tx_power_level(conidx, src_id, GAPC_PHY_PWR_1MBPS_VALUE, gapc_msg_le_get_peer_tx_power_level_1m_cmp_cb); } break;
        case GAPC_GET_PEER_TX_PWR_LEVEL_2M:          { status = gapc_le_get_peer_tx_power_level(conidx, src_id, GAPC_PHY_PWR_2MBPS_VALUE, gapc_msg_le_get_peer_tx_power_level_2m_cmp_cb); } break;
        case GAPC_GET_PEER_TX_PWR_LEVEL_LE_CODED_S8: { status = gapc_le_get_peer_tx_power_level(conidx, src_id, GAPC_PHY_PWR_S8_CODED_VALUE, gapc_msg_le_get_peer_tx_power_level_s8_cmp_cb); } break;
        case GAPC_GET_PEER_TX_PWR_LEVEL_LE_CODED_S2: { status = gapc_le_get_peer_tx_power_level(conidx, src_id, GAPC_PHY_PWR_S2_CODED_VALUE, gapc_msg_le_get_peer_tx_power_level_s2_cmp_cb); } break;
        #else // !(BLE_PWR_CTRL && BLE_GAPC)
        case GAPC_GET_LOC_TX_PWR_LEVEL_1M:
        case GAPC_GET_LOC_TX_PWR_LEVEL_2M:
        case GAPC_GET_LOC_TX_PWR_LEVEL_LE_CODED_S8:
        case GAPC_GET_LOC_TX_PWR_LEVEL_LE_CODED_S2:
        case GAPC_GET_PEER_TX_PWR_LEVEL_1M:
        case GAPC_GET_PEER_TX_PWR_LEVEL_2M:
        case GAPC_GET_PEER_TX_PWR_LEVEL_LE_CODED_S8:
        case GAPC_GET_PEER_TX_PWR_LEVEL_LE_CODED_S2: { status = GAP_ERR_NOT_SUPPORTED; } break;
        #endif // (BLE_PWR_CTRL && BLE_GAPC)
        case GAPC_GET_CHAN_SEL_ALGO:
        {
            #if (BLE_GAPC)
            struct gapc_chan_sel_algo_ind *p_ind = KE_MSG_ALLOC(GAPC_CHAN_SEL_ALGO_IND, src_id, TASK_GAPC,
                                                                gapc_chan_sel_algo_ind);

            if(p_ind)
            {
                p_ind->conidx        = conidx;
                p_ind->chan_sel_algo = gapc_get_le_channel_selection_algo(conidx);
                ke_msg_send(p_ind);
                send_cmp_evt = true;
                status = GAP_ERR_NO_ERROR;
            }
            else
            {
                status = GAP_ERR_INSUFF_RESOURCES;
            }
            #else
            status = GAP_ERR_NOT_SUPPORTED;
            #endif // (BLE_GAPC)
        } break;

        default: { ASSERT_ERR(0); } break;
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        send_cmp_evt = true;
    }

    if(send_cmp_evt)
    {
        gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);
    }

    return (KE_MSG_CONSUMED);
}


#if (BLE_GATT_CLI)
/// @brief Receive device information from application
int gapc_svc_gapc_get_dev_info_cfm_handler(ke_msg_id_t const msgid, struct gapc_get_dev_info_cfm const *p_cfm,
                                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    switch(p_cfm->req)
    {
        case GAPC_DEV_NAME:
        {
            gapc_info_name_get_cfm(p_cfm->conidx, p_cfm->token, p_cfm->status, p_cfm->complete_length,
                                   p_cfm->info.name.value_length, p_cfm->info.name.value);
        }
        break;
        case GAPC_DEV_APPEARANCE:
        {
            gapc_info_appearance_get_cfm(p_cfm->conidx, p_cfm->token, p_cfm->status, p_cfm->info.appearance);
        }
        break;

        #if (HL_LE_PERIPHERAL)
        case GAPC_DEV_SLV_PREF_PARAMS:
        {
            gapc_info_slave_pref_param_get_cfm(p_cfm->conidx, p_cfm->token, p_cfm->status,
                                               p_cfm->info.slv_pref_params);
        } break;
        #endif /* (HL_LE_PERIPHERAL) */
        default: /* Ignore Message */ break;
    }

    return (KE_MSG_CONSUMED);
}

/// @brief Receive device information modification confirmation from application
int gapc_svc_gapc_set_dev_info_cfm_handler(ke_msg_id_t const msgid, struct gapc_set_dev_info_cfm const *p_cfm,
                                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    switch(p_cfm->req)
    {
        case GAPC_DEV_NAME:       { gapc_info_name_set_cfm(p_cfm->conidx, p_cfm->token, p_cfm->status); } break;
        case GAPC_DEV_APPEARANCE: { gapc_info_appearance_set_cfm(p_cfm->conidx, p_cfm->token, p_cfm->status); } break;
        default: { /* ignore */ } break;
    }

    return (KE_MSG_CONSUMED);
}
#endif  // (BLE_GATT_CLI)

// TODO  FOR HELP  ------------------------------------  BT Classic ----------------------------------
#if (BT_HOST_PRESENT)

/// @brief Callback executed once a connection has been established. The upper layer software shall
__STATIC void gapc_msg_bt_connection_req(uint8_t conidx, uint32_t dest_id, uint8_t actv_idx, bool is_initator,
                                         const gap_addr_t* p_peer_addr)
{
    // Send connection indication message to task that requests connection.
    struct gapc_bt_connection_req_ind * p_req_ind =
            KE_MSG_ALLOC(GAPC_BT_CONNECTION_REQ_IND, dest_id, TASK_GAPC, gapc_bt_connection_req_ind);

    if(p_req_ind)
    {
        p_req_ind->conidx              = conidx;
        p_req_ind->conhdl              = gapc_get_conhdl(conidx);
        memcpy(p_req_ind->peer_addr.addr, p_peer_addr, sizeof(gap_addr_t));
        p_req_ind->peer_addr.addr_type = ADDR_PUBLIC;
        p_req_ind->is_initator         = is_initator;
        ke_msg_send(p_req_ind);
    }
}

#endif // (BT_HOST_PRESENT)

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of connection information retrieve from a precedent connection.
 * It mainly contains some bond data.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_APP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
__STATIC int gapc_msg_connection_cfm_handler(ke_msg_id_t const msgid, struct gapc_connection_cfm *p_cfm,
                                ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cfm->conidx;
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    if((p_con != NULL) && GETB(p_con->info_bf, GAPC_LE_CON_TYPE))
    {
        #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
        gapc_le_connection_cfm(p_cfm->conidx, src_id, &(p_cfm->bond_data));
        #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    }
    else
    {
        #if (BT_HOST_PRESENT)
        gapc_bt_connection_cfm(p_cfm->conidx, src_id, &(p_cfm->bond_data));
        #endif // (BT_HOST_PRESENT)
    }

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handle reception of application confirmation for bonding procedure.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gapc_bond_cfm_handler(ke_msg_id_t const msgid, struct gapc_bond_cfm* p_cfm,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cfm->conidx;

    switch (p_cfm->request)
    {
        #if (BLE_GAPC)
        #if (HL_LE_PERIPHERAL)
        case GAPC_PAIRING_RSP:
        {
            gapc_pairing_accept(conidx, p_cfm->accept, &(p_cfm->data.pairing_feat.pairing_info),
                                   p_cfm->data.pairing_feat.sec_req_level);
        } break;
        #endif //(HL_LE_PERIPHERAL)

        case GAPC_TK_EXCH:
        {
            gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);
            if(p_proc != NULL)
            {
                if(p_proc->app_exp_value == GAPC_LE_SMP_VALUE_TK) // check if passkey or TK is expected
                {
                    gapc_pairing_provide_tk(conidx, p_cfm->accept, &(p_cfm->data.tk));
                }
                else
                {
                    uint32_t passkey = (p_cfm->accept ? co_read32p(&(p_cfm->data.tk)) : 0);
                    gapc_pairing_provide_passkey(conidx, p_cfm->accept, passkey);
                }
            }
        } break;

        case GAPC_OOB_EXCH:
        {
            gapc_pairing_provide_oob_data(conidx, p_cfm->accept, &(p_cfm->data.oob));
        } break;

        case GAPC_NC_EXCH:
        case GAPC_BT_USER_VALUE_CFM:
        {
            gapc_pairing_numeric_compare_rsp(conidx, p_cfm->accept);
        } break;

        case GAPC_LTK_EXCH:
        {
            gapc_pairing_provide_ltk(conidx, &(p_cfm->data.ltk));
        } break;

        case GAPC_CSRK_EXCH:
        {
            gapc_pairing_provide_csrk(conidx, &(p_cfm->data.csrk));
        } break;
        case GAPC_IRK_EXCH:
        {
            gapc_pairing_provide_irk(conidx, &(p_cfm->data.irk.key));
        } break;
        #endif // (BLE_GAPC)

        #if (BT_HOST_PRESENT)
        case GAPC_BT_IOCAP:
        {
            gapc_bt_iocap_t* p_iocap = &(p_cfm->data.iocap);

            gapc_pairing_provide_iocap(conidx, p_cfm->accept, p_iocap->iocap, p_iocap->auth_req_bf,
                                  ((p_iocap->oob_data_present & GAP_BT_OOB_P192) != 0) ? &(p_iocap->oob_192) : NULL,
                                  ((p_iocap->oob_data_present & GAP_BT_OOB_P256) != 0) ? &(p_iocap->oob_256) : NULL);
        } break;

        case GAPC_BT_PIN_CODE:
        {
            gapc_pairing_provide_pin_code(conidx, p_cfm->accept, p_cfm->data.pincode.length, p_cfm->data.pincode.data);
        } break;

        case GAPC_BT_PASSKEY:
        {
            gapc_pairing_provide_passkey(conidx, p_cfm->accept, p_cfm->data.passkey);
        } break;
        #endif // (BT_HOST_PRESENT)
        default: { /* ignore */ } break;
    }

    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Default message handler
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gapc_default_msg_handler(ke_msg_id_t const msgid, void *event,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Check if it's a GAPC message and it's not sent to itself
    if (MSG_T(msgid) == TASK_ID_GAPC && (dest_id != src_id))
    {
        // prepare unknown message indication
        struct gapc_unknown_msg_ind* p_ind = KE_MSG_ALLOC(GAPC_UNKNOWN_MSG_IND,
                src_id, dest_id, gapc_unknown_msg_ind);

        p_ind->unknown_msg_id = msgid;

        // send event
        ke_msg_send(p_ind);

    }
    return (KE_MSG_CONSUMED);
}

#if (BT_HOST_PRESENT)

__STATIC int gapc_bt_set_req_sec_lvl_cmd_handler(ke_msg_id_t const msgid, struct gapc_bt_set_req_sec_lvl_cmd *p_cmd,
                                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = p_cmd->conidx;
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPC_BT_SET_REQ_SEC_LVL)
    {
        // Set the required security level for the connection
        status = gapc_bt_set_required_sec_lvl(conidx, p_cmd->sec_lvl);
    }

    gapc_msg_send_cmd_cmp_evt(conidx, p_cmd->operation, src_id, status);

    return (KE_MSG_CONSUMED);
}

__STATIC int gapc_bt_sec_encrypt_cfm_handler(ke_msg_id_t const msgid, struct gapc_bt_encrypt_cfm* p_cfm,
                                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    gapc_bt_encrypt_req_reply(p_cfm->conidx, p_cfm->found, &(p_cfm->link_key));
    return (KE_MSG_CONSUMED);
}


#endif // (BT_HOST_PRESENT)

/*
 * TASK VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// The default message handlers
KE_MSG_HANDLER_TAB(gapc)
{
    // Note: all messages must be sorted in ID ascending order
    /* Set Bonding information */
    { GAPC_CONNECTION_CFM,                         (ke_msg_func_t) gapc_msg_connection_cfm_handler            },

    /* Link management command */
    { GAPC_DISCONNECT_CMD,                         (ke_msg_func_t) gapc_msg_disconnect_cmd_handler            },

    /* Peer device info */
    { GAPC_GET_INFO_CMD,                           (ke_msg_func_t) gapc_msg_get_info_cmd_handler              },

    #if (BLE_GATT_CLI)
    /* Generic Access DB Management */
    { GAPC_GET_DEV_INFO_CFM,                       (ke_msg_func_t) gapc_svc_gapc_get_dev_info_cfm_handler     },
    { GAPC_SET_DEV_INFO_CFM,                       (ke_msg_func_t) gapc_svc_gapc_set_dev_info_cfm_handler     },
    #endif // (BLE_GATT_CLI)

    #if (BLE_GAPC)
    // Update connection parameters
    { GAPC_PARAM_UPDATE_CMD,                       (ke_msg_func_t) gapc_param_update_cmd_handler              },
    { GAPC_PARAM_UPDATE_CFM,                       (ke_msg_func_t) gapc_param_update_cfm_handler              },

    // LE Data Length Extension
    { GAPC_SET_LE_PKT_SIZE_CMD,                    (ke_msg_func_t) gapc_set_le_pkt_size_handler               },
    #endif // (BLE_GAPC)

    // BT Ping / LE Ping
    { GAPC_SET_LE_PING_TO_CMD,                     (ke_msg_func_t) gapc_set_le_ping_to_handler                },

    #if (BLE_GAPC)
    // LE Phy configuration
    { GAPC_SET_PHY_CMD,                            (ke_msg_func_t) gapc_set_phy_cmd_handler                   },

    #if (HL_LE_PERIPHERAL)
    { GAPC_SET_PREF_SLAVE_LATENCY_CMD,             (ke_msg_func_t) gapc_set_pref_slave_latency_cmd_handler    },
    { GAPC_SET_PREF_SLAVE_EVT_DUR_CMD,             (ke_msg_func_t) gapc_set_pref_slave_evt_dur_cmd_handler    },
    #endif // (HL_LE_PERIPHERAL)
    { GAPC_SET_MAX_RX_SIZE_AND_TIME_CMD,           (ke_msg_func_t) gapc_set_max_rx_size_and_time_cmd_handler  },

    #if (HL_LE_CENTRAL)
    // Bonding procedure
    { GAPC_BOND_CMD,                               (ke_msg_func_t) gapc_le_smp_bond_cmd_handler               },
    #endif // (HL_LE_CENTRAL)
    #endif // (BLE_GAPC)

    { GAPC_BOND_CFM,                               (ke_msg_func_t) gapc_bond_cfm_handler                      },

    #if (BLE_GAPC)
    // Pairing timers handler
    { GAPC_KEY_PRESS_NOTIFICATION_CMD,             (ke_msg_func_t) gapc_le_smp_key_press_notification_cmd_handler },

    #if (HL_LE_PERIPHERAL)
    // Security Request procedure
    { GAPC_SECURITY_CMD,                           (ke_msg_func_t) gapc_le_smp_security_cmd_handler              },
    #endif // (HL_LE_PERIPHERAL)

    #if (HL_LE_CENTRAL)
    // Encryption procedure
    { GAPC_LE_ENCRYPT_CMD,                         (ke_msg_func_t) gapc_le_smp_encrypt_cmd_handler               },
    #endif // (HL_LE_CENTRAL)

    #if (HL_LE_PERIPHERAL)
    { GAPC_LE_ENCRYPT_CFM,                         (ke_msg_func_t) gapc_le_smp_encrypt_cfm_handler               },
    #endif // (HL_LE_PERIPHERAL)
    #endif // (BLE_GAPC)

    #if (BT_HOST_PRESENT)
    { GAPC_BT_SET_REQ_SEC_LVL_CMD,                 (ke_msg_func_t) gapc_bt_set_req_sec_lvl_cmd_handler        },
    { GAPC_BT_ENCRYPT_CFM,                         (ke_msg_func_t) gapc_bt_sec_encrypt_cfm_handler           },
    #endif // (BT_HOST_PRESENT)

    #if (BLE_GAPC)
    // Periodic Sync Transfer
    { GAPC_PER_ADV_SYNC_TRANS_CMD,                 (ke_msg_func_t) gapc_per_adv_sync_trans_cmd_handler        },

    #if (BLE_GATT_CLI)
    { GAPC_CLI_FEAT_EN_CMD,                        (ke_msg_func_t) gapc_cli_feat_en_cmd_handler               },
    #endif // (BLE_GATT_CLI)

    // Constant Tone Extension
    #if(BLE_AOA | BLE_AOD)
    { GAPC_CTE_TX_CFG_CMD,                         (ke_msg_func_t) gapc_cte_tx_cfg_cmd_handler                },
    { GAPC_CTE_RX_CFG_CMD,                         (ke_msg_func_t) gapc_cte_rx_cfg_cmd_handler                },
    { GAPC_CTE_REQ_CTRL_CMD,                       (ke_msg_func_t) gapc_cte_req_ctrl_cmd_handler              },
    { GAPC_CTE_RSP_CTRL_CMD,                       (ke_msg_func_t) gapc_cte_rsp_ctrl_cmd_handler              },
    #endif // (BLE_AOA | BLE_AOD)

    #if (BLE_PWR_CTRL)
    { GAPC_TX_PWR_REPORT_CTRL_CMD,                 (ke_msg_func_t) gapc_tx_pwr_report_ctrl_cmd_handler        },
    { GAPC_PATH_LOSS_CTRL_CMD,                     (ke_msg_func_t) gapc_path_loss_ctrl_cmd_handler            },
    #endif // (BLE_PWR_CTRL)
    #endif // (BLE_GAPC)

    { KE_MSG_DEFAULT_HANDLER,                      (ke_msg_func_t) gapc_default_msg_handler                   },
};


/// GAPC task instance.
__STATIC ke_state_t gapc_state[GAPC_IDX_MAX];

/// GAP Manager task descriptor
__STATIC const ke_task_desc_t TASK_DESC_GAPC = {gapc_msg_handler_tab, gapc_state, GAPC_IDX_MAX, ARRAY_LEN(gapc_msg_handler_tab)};

/*
 * EXTERNAL FUNCTIONS
 ****************************************************************************************
 */

/// Force message API to handle indication received
void gapc_msg_set_default_callbacks(void)
{
    #if (BLE_GAPC)
    #if (BLE_PWR_CTRL)
    gapc_le_power_set_callbacks(&gapc_msg_le_power_cb_itf);
    #endif // (BLE_PWR_CTRL)

    #if (BLE_AOA || BLE_AOD)
    gapc_le_cte_set_callbacks(&gapc_msg_le_cte_cb_itf);
    #endif // (BLE_AOA || BLE_AOD)
    #endif // (BLE_GAPC)
}

/// Provide the default Message API
const gapm_callbacks_t* gapc_msg_get_callback_itf(void)
{
    return (&gapc_msg_callback_itf);
}

void gapc_msg_api_handler_create(void)
{
    ke_task_create(TASK_GAPC, &TASK_DESC_GAPC);
}

#endif // (GAPC_PRESENT && HOST_MSG_API)
/// @} GAPC_TASK
