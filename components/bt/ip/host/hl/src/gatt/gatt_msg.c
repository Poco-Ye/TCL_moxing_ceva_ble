/**
 ****************************************************************************************
 * @file gatt_msg.c
 *
 * @brief  GATT Message API handler
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GATT
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"    // IP configuration
#if (BLE_GATT && HOST_MSG_API)
#include "gatt.h"           // Native API
#include "gatt_int.h"       // Internals
#include "ke_task.h"        // KE Task handler
#include "ke_msg.h"         // KE Message  handler

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

// functions present in gatt_user.c
extern void gatt_user_register_cmd_handler(gatt_user_register_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_user_unregister_cmd_handler(gatt_user_unregister_cmd_t* p_cmd, uint16_t src_id);

// functions present in gatt_db.c
extern void gatt_db_svc_add_cmd_handler(gatt_db_svc_add_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_db_svc_remove_cmd_handler(gatt_db_svc_remove_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_db_svc_ctrl_cmd_handler(gatt_db_svc_ctrl_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_db_hash_get_cmd_handler(gatt_db_hash_get_cmd_t* p_cmd, uint16_t src_id);
#if (RW_DEBUG)
extern void gatt_dbg_db_svc_remove_all_cmd_handler(gatt_dbg_db_svc_remove_all_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_dbg_db_svc_list_get_cmd_handler(gatt_dbg_db_svc_list_get_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_dbg_db_svc_info_set_cmd_handler(gatt_dbg_db_svc_info_set_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_dbg_db_att_info_get_cmd_handler(gatt_dbg_db_att_info_get_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_dbg_db_att_info_set_cmd_handler(gatt_dbg_db_att_info_set_cmd_t* p_cmd, uint16_t src_id);

// functions present in gatt_bearer.c
extern void gatt_dbg_bearer_info_get_cmd_handler(gatt_dbg_bearer_info_get_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_dbg_bearer_close_cmd_handler(gatt_dbg_bearer_close_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_dbg_bearer_eatt_estab_cmd_handler(gatt_dbg_bearer_eatt_estab_cmd_t* p_cmd, uint16_t src_id);
#endif // (RW_DEBUG)

// function present in gatt_srv_event.c
extern void gatt_srv_event_reliable_send_cmd_handler(gatt_srv_event_reliable_send_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_srv_att_event_get_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t dummy, uint16_t hdl, uint16_t max_length);
extern void gatt_srv_att_event_get_cfm_handler(gatt_srv_att_event_get_cfm_t* p_cfm, uint16_t src_id);
extern void gatt_srv_event_send_cmd_handler(gatt_srv_event_send_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_srv_event_mtp_send_cmd_handler(gatt_srv_event_mtp_send_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_srv_event_mtp_cancel_cmd_handler(gatt_srv_event_mtp_cancel_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_srv_event_sent_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status);

// function present in gatt_srv_read.c
extern void gatt_srv_att_read_get_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset, uint16_t max_length);
extern void gatt_srv_att_read_get_cfm_handler(gatt_srv_att_read_get_cfm_t* p_cfm, uint16_t src_id);

// function present in gatt_srv_write.c
extern void gatt_srv_att_info_get_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl);
extern void gatt_srv_att_info_get_cfm_handler(gatt_srv_att_info_get_cfm_t* p_cfm, uint16_t src_id);
extern void gatt_srv_att_val_set_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset, co_buf_t* p_data);
extern void gatt_srv_att_val_set_cfm_handler(gatt_srv_att_val_set_cfm_t* p_cfm, uint16_t src_id);

#if (BLE_GATT_CLI)
// function present in gatt_cli_dicover.c
extern void gatt_cli_discover_svc_cmd_handler(gatt_cli_discover_svc_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_cli_discover_inc_svc_cmd_handler(gatt_cli_discover_inc_svc_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_cli_discover_char_cmd_handler(gatt_cli_discover_char_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_cli_discover_desc_cmd_handler(gatt_cli_discover_desc_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_cli_discover_cancel_cmd_handler(gatt_cli_discover_cancel_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_cli_discover_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status);
extern void gatt_cli_svc_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t hdl, uint8_t disc_info, uint8_t nb_att, const gatt_svc_att_t* p_atts);
extern void gatt_cli_svc_info_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t start_hdl, uint16_t end_hdl, uint8_t uuid_type, const uint8_t* p_uuid);
extern void gatt_cli_inc_svc_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t inc_svc_hdl, uint16_t start_hdl, uint16_t end_hdl, uint8_t uuid_type, const uint8_t* p_uuid);
extern void gatt_cli_char_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t char_hdl, uint16_t val_hdl, uint8_t prop, uint8_t uuid_type, const uint8_t* p_uuid);
extern void gatt_cli_desc_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t desc_hdl, uint8_t uuid_type, const uint8_t* p_uuid);

// function present in gatt_cli_read.c
extern void gatt_cli_read_cmd_handler(gatt_cli_read_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_cli_read_by_uuid_cmd_handler(gatt_cli_read_by_uuid_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_cli_read_multiple_cmd_handler(gatt_cli_read_multiple_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_cli_read_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status);
extern void gatt_cli_att_val_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t hdl, uint16_t offset, co_buf_t* p_data);

// function present in gatt_cli_write.c
extern void gatt_cli_write_reliable_cmd_handler(gatt_cli_write_reliable_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_cli_write_cmd_handler(gatt_cli_write_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_cli_write_exe_cmd_handler(gatt_cli_write_exe_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_cli_write_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status);
extern void gatt_cli_att_val_get_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t dummy, uint16_t hdl, uint16_t offset, uint16_t max_length);
extern void gatt_cli_att_val_get_cfm_handler(gatt_cli_att_val_get_cfm_t* p_cfm, uint16_t src_id);

// function present in gatt_cli_event.c
extern void gatt_cli_event_register_cmd_handler(gatt_cli_event_register_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_cli_event_unregister_cmd_handler(gatt_cli_event_unregister_cmd_t* p_cmd, uint16_t src_id);
extern void gatt_cli_att_val_evt_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint8_t evt_type, bool complete, uint16_t hdl, co_buf_t* p_data);
extern void gatt_cli_att_event_cfm_handler(gatt_cli_att_event_cfm_t* p_cfm, uint16_t src_id);
extern void gatt_cli_svc_changed_cb(uint8_t conidx, uint8_t user_lid, bool out_of_sync, uint16_t start_hdl, uint16_t end_hdl);

// function present in gatt_cli_mtu_exch.c
extern void gatt_cli_mtu_update_cmd_handler(gatt_cli_mtu_update_cmd_t* p_cmd, uint16_t src_id);
#endif // (BLE_GATT_CLI)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

#if (BLE_GATT_CLI)
/// Client call-back for message handlers
const gatt_cli_cb_t gatt_cli_msg_cb =
{
    .cb_discover_cmp    = gatt_cli_discover_cmp_cb,
    .cb_read_cmp        = gatt_cli_read_cmp_cb,
    .cb_write_cmp       = gatt_cli_write_cmp_cb,
    .cb_att_val_get     = gatt_cli_att_val_get_cb,
    .cb_svc             = gatt_cli_svc_cb,
    .cb_svc_info        = gatt_cli_svc_info_cb,
    .cb_inc_svc         = gatt_cli_inc_svc_cb,
    .cb_char            = gatt_cli_char_cb,
    .cb_desc            = gatt_cli_desc_cb,
    .cb_att_val         = gatt_cli_att_val_cb,
    .cb_att_val_evt     = gatt_cli_att_val_evt_cb,
    .cb_svc_changed     = gatt_cli_svc_changed_cb,
};
#endif // (BLE_GATT_CLI)

/// Server call-back for message handlers
const gatt_srv_cb_t gatt_srv_msg_cb =
{
    .cb_event_sent      = gatt_srv_event_sent_cb,
    .cb_att_read_get    = gatt_srv_att_read_get_cb,
    .cb_att_event_get   = gatt_srv_att_event_get_cb,
    .cb_att_info_get    = gatt_srv_att_info_get_cb,
    .cb_att_val_set     = gatt_srv_att_val_set_cb,
};



/*
 * INTERNAL FUNCTIONS
 ****************************************************************************************
 */

void gatt_msg_send_basic_cmp_evt(uint16_t cmd_code, uint16_t dummy, uint16_t src_id, uint8_t user_lid, uint16_t status)
{
    gatt_cmp_evt_t* p_cmp_evt = KE_MSG_ALLOC(GATT_CMP_EVT, src_id ,TASK_GATT, gatt_cmp_evt);

    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->cmd_code = cmd_code;
        p_cmp_evt->dummy    = dummy;
        p_cmp_evt->user_lid = user_lid;
        p_cmp_evt->status   = status;

        // send message to host
        ke_msg_send(p_cmp_evt);
    }
}

void gatt_msg_send_proc_cmp_evt(uint16_t cmd_code, uint16_t dummy, uint8_t conidx, uint16_t src_id, uint8_t user_lid, uint16_t status)
{
    gatt_proc_cmp_evt_t* p_cmp_evt = KE_MSG_ALLOC(GATT_CMP_EVT, src_id ,TASK_GATT, gatt_proc_cmp_evt);

    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->cmd_code = cmd_code;
        p_cmp_evt->dummy    = dummy;
        p_cmp_evt->user_lid = user_lid;
        p_cmp_evt->status   = status;
        p_cmp_evt->conidx   = conidx;

        // send message to host
        ke_msg_send(p_cmp_evt);
    }
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */


/*
 * MESSAGE API
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Default handler
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gatt_default_msg_handler(ke_msg_id_t const msgid, void *p_param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Check if it's a L2CAP message and it's not sent to itself
    if (MSG_T(msgid) == TASK_ID_GATT && (dest_id != src_id))
    {
        // prepare unknown message indication
        gatt_unknown_msg_ind_t* p_ind = KE_MSG_ALLOC(GATT_IND, src_id, dest_id, gatt_unknown_msg_ind);

        p_ind->ind_code = GATT_UNKNOWN_MSG;
        p_ind->dummy    = 0;
        p_ind->user_lid = GATT_INVALID_USER_LID;
        p_ind->msg_id   = msgid;

        // send event
        ke_msg_send(p_ind);
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle reception of a GATT command message
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_cmd     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gatt_cmd_msg_handler(ke_msg_id_t const msgid, gatt_cmd_t *p_cmd, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    switch(p_cmd->cmd_code)
    {
        case GATT_USER_REGISTER:           { gatt_user_register_cmd_handler((gatt_user_register_cmd_t*) p_cmd, src_id);                     } break;
        case GATT_USER_UNREGISTER:         { gatt_user_unregister_cmd_handler((gatt_user_unregister_cmd_t*) p_cmd, src_id);                 } break;
        case GATT_DB_SVC_ADD:              { gatt_db_svc_add_cmd_handler((gatt_db_svc_add_cmd_t*) p_cmd, src_id);                           } break;
        case GATT_DB_SVC_REMOVE:           { gatt_db_svc_remove_cmd_handler((gatt_db_svc_remove_cmd_t*) p_cmd, src_id);                     } break;
        case GATT_DB_SVC_CTRL:             { gatt_db_svc_ctrl_cmd_handler((gatt_db_svc_ctrl_cmd_t*) p_cmd, src_id);                         } break;
        case GATT_DB_HASH_GET:             { gatt_db_hash_get_cmd_handler((gatt_db_hash_get_cmd_t*) p_cmd, src_id);                         } break;
        case GATT_SRV_EVENT_RELIABLE_SEND: { gatt_srv_event_reliable_send_cmd_handler((gatt_srv_event_reliable_send_cmd_t*) p_cmd, src_id); } break;
        case GATT_SRV_EVENT_SEND:          { gatt_srv_event_send_cmd_handler((gatt_srv_event_send_cmd_t*) p_cmd, src_id);                   } break;
        case GATT_SRV_EVENT_MTP_SEND:      { gatt_srv_event_mtp_send_cmd_handler((gatt_srv_event_mtp_send_cmd_t*) p_cmd, src_id);           } break;
        case GATT_SRV_EVENT_MTP_CANCEL:    { gatt_srv_event_mtp_cancel_cmd_handler((gatt_srv_event_mtp_cancel_cmd_t*) p_cmd, src_id);       } break;
        #if (BLE_GATT_CLI)
        case GATT_CLI_DISCOVER_SVC:        { gatt_cli_discover_svc_cmd_handler((gatt_cli_discover_svc_cmd_t*) p_cmd, src_id);               } break;
        case GATT_CLI_DISCOVER_INC_SVC:    { gatt_cli_discover_inc_svc_cmd_handler((gatt_cli_discover_inc_svc_cmd_t*) p_cmd, src_id);       } break;
        case GATT_CLI_DISCOVER_CHAR:       { gatt_cli_discover_char_cmd_handler((gatt_cli_discover_char_cmd_t*) p_cmd, src_id);             } break;
        case GATT_CLI_DISCOVER_DESC:       { gatt_cli_discover_desc_cmd_handler((gatt_cli_discover_desc_cmd_t*) p_cmd, src_id);             } break;
        case GATT_CLI_DISCOVER_CANCEL:     { gatt_cli_discover_cancel_cmd_handler((gatt_cli_discover_cancel_cmd_t*) p_cmd, src_id);         } break;
        case GATT_CLI_READ:                { gatt_cli_read_cmd_handler((gatt_cli_read_cmd_t*) p_cmd, src_id);                               } break;
        case GATT_CLI_READ_BY_UUID:        { gatt_cli_read_by_uuid_cmd_handler((gatt_cli_read_by_uuid_cmd_t*) p_cmd, src_id);               } break;
        case GATT_CLI_READ_MULTIPLE:       { gatt_cli_read_multiple_cmd_handler((gatt_cli_read_multiple_cmd_t*) p_cmd, src_id);             } break;
        case GATT_CLI_WRITE_RELIABLE:      { gatt_cli_write_reliable_cmd_handler((gatt_cli_write_reliable_cmd_t*) p_cmd, src_id);           } break;
        case GATT_CLI_WRITE:               { gatt_cli_write_cmd_handler((gatt_cli_write_cmd_t*) p_cmd, src_id);                             } break;
        case GATT_CLI_WRITE_EXE:           { gatt_cli_write_exe_cmd_handler((gatt_cli_write_exe_cmd_t*) p_cmd, src_id);                     } break;
        case GATT_CLI_EVENT_REGISTER:      { gatt_cli_event_register_cmd_handler((gatt_cli_event_register_cmd_t*) p_cmd, src_id);           } break;
        case GATT_CLI_EVENT_UNREGISTER:    { gatt_cli_event_unregister_cmd_handler((gatt_cli_event_unregister_cmd_t*) p_cmd, src_id);       } break;
        case GATT_CLI_MTU_UPDATE:          { gatt_cli_mtu_update_cmd_handler((gatt_cli_mtu_update_cmd_t*) p_cmd, src_id);                   } break;
        #endif  // (BLE_GATT_CLI)
        #if (RW_DEBUG)
        case GATT_DBG_DB_SVC_REMOVE_ALL:   { gatt_dbg_db_svc_remove_all_cmd_handler((gatt_dbg_db_svc_remove_all_cmd_t*) p_cmd, src_id);     } break;
        case GATT_DBG_DB_SVC_LIST_GET:     { gatt_dbg_db_svc_list_get_cmd_handler((gatt_dbg_db_svc_list_get_cmd_t*) p_cmd, src_id);         } break;
        case GATT_DBG_DB_SVC_INFO_SET:     { gatt_dbg_db_svc_info_set_cmd_handler((gatt_dbg_db_svc_info_set_cmd_t*) p_cmd, src_id);         } break;
        case GATT_DBG_DB_ATT_INFO_GET:     { gatt_dbg_db_att_info_get_cmd_handler((gatt_dbg_db_att_info_get_cmd_t*) p_cmd, src_id);         } break;
        case GATT_DBG_DB_ATT_INFO_SET:     { gatt_dbg_db_att_info_set_cmd_handler((gatt_dbg_db_att_info_set_cmd_t*) p_cmd, src_id);         } break;
        case GATT_DBG_BEARER_INFO_GET:     { gatt_dbg_bearer_info_get_cmd_handler((gatt_dbg_bearer_info_get_cmd_t*) p_cmd, src_id);         } break;
        case GATT_DBG_BEARER_CLOSE:        { gatt_dbg_bearer_close_cmd_handler((gatt_dbg_bearer_close_cmd_t*) p_cmd, src_id);               } break;
        case GATT_DBG_BEARER_EATT_ESTAB:   { gatt_dbg_bearer_eatt_estab_cmd_handler((gatt_dbg_bearer_eatt_estab_cmd_t*) p_cmd, src_id);     } break;
        #endif // (RW_DEBUG)
        // default: Reject command
        default:                           { gatt_msg_send_basic_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, src_id, GATT_INVALID_USER_LID, GAP_ERR_NOT_SUPPORTED);      } break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle reception of a GATT confirmation message
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gatt_cfm_msg_handler(ke_msg_id_t const msgid, gatt_cfm_t *p_cfm, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    switch(p_cfm->req_ind_code)
    {
        case GATT_SRV_ATT_READ_GET: { gatt_srv_att_read_get_cfm_handler((gatt_srv_att_read_get_cfm_t*) p_cfm, src_id);   } break;
        case GATT_SRV_ATT_EVENT_GET:{ gatt_srv_att_event_get_cfm_handler((gatt_srv_att_event_get_cfm_t*) p_cfm, src_id); } break;
        case GATT_SRV_ATT_INFO_GET: { gatt_srv_att_info_get_cfm_handler((gatt_srv_att_info_get_cfm_t*) p_cfm, src_id);   } break;
        case GATT_SRV_ATT_VAL_SET:  { gatt_srv_att_val_set_cfm_handler((gatt_srv_att_val_set_cfm_t*) p_cfm, src_id);     } break;
        #if (BLE_GATT_CLI)
        case GATT_CLI_ATT_VAL_GET:  { gatt_cli_att_val_get_cfm_handler((gatt_cli_att_val_get_cfm_t*) p_cfm, src_id);     } break;
        case GATT_CLI_ATT_EVENT:    { gatt_cli_att_event_cfm_handler((gatt_cli_att_event_cfm_t*) p_cfm, src_id);         } break;
        #endif // (BLE_GATT_CLI)
    default: { /* Ignore message */ } break;
    }

    return (KE_MSG_CONSUMED);
}

#endif // (BLE_GATT && HOST_MSG_API)
/// @} GATT

