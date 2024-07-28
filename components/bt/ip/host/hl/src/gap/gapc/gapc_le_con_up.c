/**
 ****************************************************************************************
 *
 * @file gapc_le_con_up.c
 *
 * @brief Generic Access Profile Controller - BLE Connection update.
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_GAPC)

#include "gapc_le_con.h"

#include "hl_hci.h"
#include "hci.h"

#if (HOST_PROFILES)
#include "../../inc/prf_hl_api.h"
#endif // (HOST_PROFILES)
#include "../../inc/l2cap_sig.h"
#include "../../inc/gatt_hl_api.h"

#include "co_math.h"
#include <string.h>

/*
 * DEFINES
 ****************************************************************************************
 */

/// BLE Connection parameter procedure states - Initiating
enum gapc_le_con_up_proc_init_event
{
    /// Received HCI_LE_CON_UPDATE_CMD status event
    GAPC_LE_CON_UPD_PROC_HCI_STAT_EVT_RECV = HL_PROC_EVENT_FIRST,
    /// Received HCI_LE_CON_UPDATE_CMP_EVT complete event
    GAPC_LE_CON_UPD_PROC_HCI_CMP_EVT_RECV,
    /// L2CAP negotiation done
    GAPC_LE_CON_UPD_PROC_L2CAP_NEGO_DONE,
};

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// Connection update procedure object - Initiator
typedef struct gapc_le_con_up_proc_init
{
    /// procedure inheritance
    hl_proc_t        hdr;
    /// Callback to execute once command completes
    gapc_proc_cmp_cb cmp_cb;
    /// Dummy parameter provided by upper layer SW
    uint32_t         dummy;
    /// Connection parameters to negotiate
    gap_le_con_param_nego_with_ce_len_t param;
    /// Connection index
    uint8_t          conidx;
} gapc_le_con_up_proc_init_t;


/// Connection update procedure object - Responder
typedef struct gapc_le_con_up_proc_resp
{
    /// procedure inheritance
    hl_proc_t        hdr;
    /// Connection parameters to negotiate
    gap_le_con_param_nego_with_ce_len_t param;
    /// Connection index
    uint8_t          conidx;
    /// SIG Packet identifier
    uint8_t          pkt_id;
    /// Accepted or rejected parameters
    bool             accepted;
    /// Wait for application confirm - True if already provided
    bool             wait_app_cfm;
    /// True for an L2CAP negotiation, False otherwise
    bool             l2cap_nego;
} gapc_le_con_up_proc_resp_t;


#if (HL_LE_PERIPHERAL)
/// Set peripheral preferred latency procedure parameters
typedef struct gapc_le_con_up_proc_set_pref_slave_latency
{
    /// Inherited from simple GAPC procedure
    gapc_proc_simple_t hdr;
    /// Preferred latency (in number of connection events)
    uint16_t           latency;
} gapc_le_con_up_proc_set_pref_slave_latency_t;

/// Set peripheral preferred event duration
typedef struct gapc_le_con_up_proc_set_pref_slave_evt_dur
{
    /// Inherited from simple GAPC procedure
    gapc_proc_simple_t hdr;
    /// Preferred event duration (N * 0.625 ms)
    uint16_t           duration;
    /// Slave transmits a single packet per connection event (False/True)
    bool               single_tx;
} gapc_le_con_up_proc_set_pref_slave_evt_dur_t;
#endif // (HL_LE_PERIPHERAL)

/*
 * LOCAL FUNCTIONS DECLARATIONS
 ****************************************************************************************
 */
__STATIC bool gapm_le_con_up_proc_init_transition(gapc_le_con_up_proc_init_t* p_proc, uint8_t event, uint16_t status);
__STATIC bool gapm_le_con_up_proc_resp_transition(gapc_le_con_up_proc_resp_t* p_proc, uint8_t event, uint16_t status);

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
/// Connection update state machine - initiator
__STATIC const hl_proc_itf_t gapc_le_con_up_proc_init_itf =
{
    .transition  = (hl_proc_transition_cb) gapm_le_con_up_proc_init_transition,
    .cleanup     = hl_proc_cleanup,
};

/// Connection update state machine - responder
__STATIC const hl_proc_itf_t gapc_le_con_up_proc_resp_itf =
{
    .transition  = (hl_proc_transition_cb) gapm_le_con_up_proc_resp_transition,
    .cleanup     = hl_proc_cleanup,
};

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (HL_LE_PERIPHERAL)
/// empty callback function
__STATIC void gapc_le_con_disconnect_cmp_cb(uint8_t conidx, uint32_t dummy, uint16_t status) {}
#endif // (HL_LE_PERIPHERAL)


#if (HL_LE_CENTRAL)
/// @brief Sends parameter response
__STATIC uint16_t gapc_con_up_send_l2cap_param_resp(uint8_t conidx, uint8_t pkt_id, bool accept)
{
    l2cap_sig_conn_param_upd_rsp_t rsp_pdu =
    {
            .code = L2CAP_SIG_CONN_PARAM_UPD_RSP_OPCODE,
    };
    rsp_pdu.response = accept ? L2CAP_SIG_CON_PARAM_ACCEPT : L2CAP_SIG_CON_PARAM_REJECT;

    // Send connection response with error
    return l2cap_sig_pdu_send(conidx, 0, pkt_id, (l2cap_sig_pdu_t*) &rsp_pdu, 0, NULL);
}
#endif // (HL_LE_CENTRAL)

 /// @brief Checks connection parameters values
__STATIC bool gapc_le_con_is_param_update_valid(const gap_le_con_param_nego_t* p_param)
{
    bool is_valid = true;
    /* check for the range validity of the values */
    #if(BLE_ISO_MODE_0_PROFILE)
    if (   (p_param->interval_max < CON_INTERVAL_MIN_AUDIO) || (p_param->interval_max > GAP_CNX_INTERVAL_MAX)
        || (p_param->interval_min < CON_INTERVAL_MIN_AUDIO) || (p_param->interval_min > GAP_CNX_INTERVAL_MAX)
        || (p_param->sup_to < GAP_CNX_SUP_TO_MIN)           || (p_param->sup_to > GAP_CNX_SUP_TO_MAX)
        || (p_param->latency > GAP_CNX_LATENCY_MAX))
    {
        is_valid = false;
    }
    #else // !(BLE_ISO_MODE_0_PROFILE)
    if (   (p_param->interval_max < GAP_CNX_INTERVAL_MIN) || (p_param->interval_max > GAP_CNX_INTERVAL_MAX)
        || (p_param->interval_min < GAP_CNX_INTERVAL_MIN) || (p_param->interval_min > GAP_CNX_INTERVAL_MAX)
        || (p_param->sup_to < GAP_CNX_SUP_TO_MIN )        || (p_param->sup_to > GAP_CNX_SUP_TO_MAX)
        || (p_param->latency > GAP_CNX_LATENCY_MAX))
    {
        is_valid = false;
    }
    #endif // (BLE_ISO_MODE_0_PROFILE)
    return (is_valid);
}

/// Send HCI_LE_CON_UPDATE_CMD to controller
__STATIC uint16_t gapc_le_con_send_hci_le_con_update_cmd(uint8_t conidx, const gap_le_con_param_nego_with_ce_len_t* p_param,
                                                         uint16_t event, hl_hci_cmd_evt_func_t evt_cb)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    struct hci_le_con_update_cmd* p_con_up_cmd = HL_HCI_CMD_ALLOC(HCI_LE_CON_UPDATE_CMD_OPCODE, hci_le_con_update_cmd);

    if(p_con_up_cmd != NULL)
    {
        p_con_up_cmd->conhdl       = gapc_get_conhdl(conidx);
        memcpy(&(p_con_up_cmd->con_intv_min), p_param, sizeof(gap_le_con_param_nego_with_ce_len_t));
        HL_HCI_CMD_SEND_TO_CTRL(p_con_up_cmd, GAPC_PROC_TOKEN(conidx, event), evt_cb);
        status = GAP_ERR_NO_ERROR;
    }

    return(status);
}

/// Send HCI_LE_REM_CON_PARAM_REQ_REPLY_CMD or HCI_LE_REM_CON_PARAM_REQ_NEG_REPLY_CMD to controller
__STATIC uint16_t gapc_le_con_send_hci_le_con_update_resp(uint8_t conidx, bool accepted,
                                                          const gap_le_con_param_nego_with_ce_len_t* p_param)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    if(accepted)
    {
        struct hci_le_rem_con_param_req_reply_cmd* p_con_upd_reply =
                HL_HCI_CMD_ALLOC(HCI_LE_REM_CON_PARAM_REQ_REPLY_CMD_OPCODE, hci_le_rem_con_param_req_reply_cmd);
        if(p_con_upd_reply != NULL)
        {
            p_con_upd_reply->conhdl       = gapc_get_conhdl(conidx);
            memcpy(&p_con_upd_reply->interval_min, p_param, sizeof(gap_le_con_param_nego_with_ce_len_t));
            HL_HCI_CMD_SEND_TO_CTRL(p_con_upd_reply,  0, gapc_proc_ignore_hci_evt_handler);
            status = GAP_ERR_NO_ERROR;
        }
    }
    else
    {
        struct hci_le_rem_con_param_req_neg_reply_cmd* p_con_upd_rej =
                HL_HCI_CMD_ALLOC(HCI_LE_REM_CON_PARAM_REQ_NEG_REPLY_CMD_OPCODE,
                                                 hci_le_rem_con_param_req_neg_reply_cmd);
        if(p_con_upd_rej != NULL)
        {
            p_con_upd_rej->conhdl = gapc_get_conhdl(conidx);
            p_con_upd_rej->reason = CO_ERROR_UNACCEPTABLE_CONN_PARAM;
            HL_HCI_CMD_SEND_TO_CTRL(p_con_upd_rej, 0, gapc_proc_ignore_hci_evt_handler);
            status = GAP_ERR_NO_ERROR;
        }
    }

    return(status);
}


/// Handle peer initiated negotiation
__STATIC bool gapc_le_con_up_handle_peer_nego(uint8_t conidx, gapc_le_con_t* p_con, const gap_le_con_param_nego_with_ce_len_t * p_param,
                                       bool l2cap_nego, uint8_t pkt_id)
{
    bool do_handle = false;
    const gapc_le_config_cb_t* p_cbs = gapc_env.p_le_config_cbs;

    // perform a sanity check of connection parameters
    if ((p_cbs->param_update_req != NULL) && gapc_le_con_is_param_update_valid(&(p_param->hdr)))
    {
        gapc_le_con_up_proc_resp_t* p_proc;

        // Create procedure to handle peer negotiation
        if(gapc_proc_le_create(conidx, sizeof(gapc_le_con_up_proc_resp_t), &gapc_le_con_up_proc_resp_itf,
                (hl_proc_t**) &p_proc) == GAP_ERR_NO_ERROR)
        {
            p_proc->conidx       = conidx;
            p_proc->param        = *p_param;
            p_proc->wait_app_cfm = false;
            p_proc->l2cap_nego   = l2cap_nego;
            p_proc->pkt_id       = pkt_id;
            do_handle = true;
        }
    }

    return (do_handle);
}


#if (HL_LE_PERIPHERAL)
/// @brief Send the HCI_VS_SET_PREF_SLAVE_LATENCY_CMD command
__STATIC uint16_t gapc_le_con_up_proc_set_pref_slave_latency_granted(uint8_t conidx,
                                                                   gapc_le_con_up_proc_set_pref_slave_latency_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    struct hci_vs_set_pref_slave_latency_cmd *p_hci_cmd =
            HL_HCI_CMD_ALLOC(HCI_VS_SET_PREF_SLAVE_LATENCY_CMD_OPCODE, hci_vs_set_pref_slave_latency_cmd);
    if(p_hci_cmd != NULL)
    {
        p_hci_cmd->conhdl    = gapc_get_conhdl(conidx);
        p_hci_cmd->latency   = p_proc->latency;
        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_FINISHED), gapc_proc_default_hci_cmp_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }
    return (status);
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_le_con_up_proc_set_pref_slave_latency_itf =
{
        .granted  = (gapc_proc_simple_granted_cb) gapc_le_con_up_proc_set_pref_slave_latency_granted,
        .finished = gapc_proc_simple_default_finished_cb,
};

/// @brief Send the HCI_VS_SET_PREF_SLAVE_EVT_DUR_CMD command
__STATIC uint16_t gapc_le_con_up_proc_set_pref_slave_evt_dur_granted(uint8_t conidx,
                                                                  gapc_le_con_up_proc_set_pref_slave_evt_dur_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    struct hci_vs_set_pref_slave_evt_dur_cmd *p_hci_cmd =
            HL_HCI_CMD_ALLOC(HCI_VS_SET_PREF_SLAVE_EVT_DUR_CMD_OPCODE, hci_vs_set_pref_slave_evt_dur_cmd);
    if(p_hci_cmd != NULL)
    {
        p_hci_cmd->conhdl    = gapc_get_conhdl(conidx);
        p_hci_cmd->duration  = p_proc->duration;
        p_hci_cmd->single_tx = p_proc->single_tx;
        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_FINISHED), gapc_proc_default_hci_cmp_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }
    return (status);
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_le_con_up_proc_set_pref_slave_evt_dur_itf =
{
        .granted  = (gapc_proc_simple_granted_cb) gapc_le_con_up_proc_set_pref_slave_evt_dur_granted,
        .finished = gapc_proc_simple_default_finished_cb,
};
#endif // (HL_LE_PERIPHERAL)


/*
 * PROCEDURE STATE MACHINE
 ****************************************************************************************
 */

#if (HL_LE_PERIPHERAL)
/// L2CAP Signaling Nego procedure state machine
__STATIC void gapc_le_con_up_sig_proc_continue(uint8_t conidx, l2cap_sig_proc_t* p_sig_proc, uint8_t event, uint16_t status)
{
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

    if(!GETB(p_con->hdr.info_bf, GAPC_DISCONNECTING))
    {
        bool op_finished = true;

        gapc_le_con_up_proc_init_t* p_proc = (gapc_le_con_up_proc_init_t*) gapc_proc_get(conidx);
        ASSERT_ERR(p_proc != NULL);

        switch(event)
        {
            case L2CAP_SIG_PROC_START:
            {
                gap_le_con_param_nego_t* p_param = &(p_proc->param.hdr);
                l2cap_sig_conn_param_upd_req_t pdu_con_up_req =
                {
                    .code     = L2CAP_SIG_CONN_PARAM_UPD_REQ_OPCODE,
                    .intv_min = p_param->interval_min,
                    .intv_max = p_param->interval_max,
                    .latency  = p_param->latency,
                    .timeout  = p_param->sup_to,
                };

                // allocate a new packet identifier
                p_sig_proc->pkt_id = l2cap_sig_pkt_id_get(conidx);

                // send PDU
                status = l2cap_sig_pdu_send(conidx, 0, p_sig_proc->pkt_id, (l2cap_sig_pdu_t*) &pdu_con_up_req, 0, NULL);
                if(status != GAP_ERR_NO_ERROR) break;

                l2cap_sig_trans_timer_start(conidx);
                op_finished = false;
            } break;

            case L2CAP_SIG_PROC_RSP_RECEIVED:
            {
                // stop transaction timer
                l2cap_sig_trans_timer_stop(conidx);
            } break;

            case L2CAP_SIG_PROC_ERROR:
            {
                if(status == GAP_ERR_TIMEOUT)
                {
                    // force a disconnection
                    gapc_disconnect(conidx, 0, LL_ERR_UNACCEPTABLE_CONN_PARAM, gapc_le_con_disconnect_cmp_cb);
                }
            } break;

            default: { ASSERT_ERR(0); } break;
        }

        if(op_finished)
        {
            // remove l2cap procedure
            l2cap_sig_proc_pop(conidx, p_sig_proc);

            // update GAPC procedure transition
            gapc_proc_transition(conidx, GAPC_LE_CON_UPD_PROC_L2CAP_NEGO_DONE, status);
        }
    }
}
#endif // (HL_LE_PERIPHERAL)



/// Connection update state machine - Initiator
__STATIC bool gapm_le_con_up_proc_init_transition(gapc_le_con_up_proc_init_t* p_proc, uint8_t event, uint16_t status)
{
    bool is_finished = false;
    uint8_t conidx = p_proc->conidx;

    #if (HL_LE_PERIPHERAL)
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
    if((status != GAP_ERR_NO_ERROR) && (GETB(p_con->hdr.info_bf, GAPC_ROLE) == ROLE_SLAVE))
    {
        if(   (status == LL_ERR_UNKNOWN_HCI_COMMAND) ||(status == LL_ERR_COMMAND_DISALLOWED)
           || (status == LL_ERR_UNSUPPORTED) || (status == LL_ERR_UNKNOWN_LMP_PDU)
           || (status == LL_ERR_UNSUPPORTED_REMOTE_FEATURE))
        {
            // Clear parameter request feature in the environment because not supported by peer
            p_con->peer_features[0] &= ~CO_BIT(GAP_LE_FEAT_CON_PARAM_REQ_PROC);
            // change transition properties to start a L2CAP nego
            status = GAP_ERR_NO_ERROR;
            event = HL_PROC_GRANTED;
        }
    }
    #endif // (HL_LE_PERIPHERAL)


    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                #if (HL_LE_PERIPHERAL)
                // check if L2CAP Negotiation must be performed
                if(    (GETB(p_con->hdr.info_bf, GAPC_ROLE) == ROLE_SLAVE)
                   && !gapc_is_le_feat_supported(conidx, GAP_LE_FEAT_CON_PARAM_REQ_PROC))
                {
                    l2cap_sig_proc_t* p_sig_proc;
                    status = l2cap_sig_proc_create(conidx, L2CAP_SIG_PROC_REQ, L2CAP_SIG_PROC_CON_PARAM_UPDATE,
                                                   gapc_le_con_up_sig_proc_continue, sizeof(l2cap_sig_proc_t), &p_sig_proc);

                    if(status != GAP_ERR_NO_ERROR) break;
                    // push operation in execution queue
                    l2cap_sig_proc_push(conidx, p_sig_proc);
                    break;
                }
                #endif // (HL_LE_PERIPHERAL)

                // Use HCI negotiation
                status = gapc_le_con_send_hci_le_con_update_cmd(conidx, &(p_proc->param), GAPC_LE_CON_UPD_PROC_HCI_STAT_EVT_RECV,
                                                                (hl_hci_cmd_evt_func_t) gapc_proc_default_hci_stat_evt_handler);
            } break;

            case GAPC_LE_CON_UPD_PROC_L2CAP_NEGO_DONE:
            case GAPC_LE_CON_UPD_PROC_HCI_CMP_EVT_RECV:
            {
                is_finished = true;
            } break;

            case GAPC_LE_CON_UPD_PROC_HCI_STAT_EVT_RECV:
            default: { /* ignore */ } break;
        }
    }

    // error handling
    if(status != GAP_ERR_NO_ERROR)
    {
        is_finished = true;
    }

    if(is_finished)
    {
        // inform application that procedure is over
        p_proc->cmp_cb(conidx, p_proc->dummy, status);
    }

    return (is_finished);
}



/// Connection update state machine - responder
__STATIC bool gapm_le_con_up_proc_resp_transition(gapc_le_con_up_proc_resp_t* p_proc, uint8_t event, uint16_t status)
{
    bool is_finished = false;
    uint8_t conidx = p_proc->conidx;
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                const gapc_le_config_cb_t* p_cbs = gapc_env.p_le_config_cbs;
                p_proc->wait_app_cfm = true;
                SETB(p_con->hdr.info_bf, GAPC_IN_PROC, true);
                // Ask Application to accept or reject peer parameters
                p_cbs->param_update_req(conidx, p_con->hdr.dummy, &(p_proc->param.hdr));
                SETB(p_con->hdr.info_bf, GAPC_IN_PROC, false);
                // application confirmation not received
                if(p_proc->wait_app_cfm) break;
            }
            // no break

            case HL_PROC_FINISHED:
            {
                is_finished = true;

                #if (HL_LE_CENTRAL)
                if(p_proc->l2cap_nego)
                {
                    // send response to peer device
                    status = gapc_con_up_send_l2cap_param_resp(conidx, p_proc->pkt_id, p_proc->accepted);
                    if(status != GAP_ERR_NO_ERROR) break;

                    // if parameters are accepted, update them.
                    if(p_proc->accepted)
                    {
                        status = gapc_le_con_send_hci_le_con_update_cmd(conidx, &(p_proc->param), 0,
                                                                        gapc_proc_ignore_hci_evt_handler);
                    }
                }
                else
                #endif // (HL_LE_CENTRAL)
                {
                    // send HCI response
                    status = gapc_le_con_send_hci_le_con_update_resp(conidx, p_proc->accepted, &p_proc->param);
                }
            } break;

            default: { /* ignore */ } break;
        }
    }

    return (is_finished || (status != GAP_ERR_NO_ERROR));
}


/*
 * L2CAP SIGNALING MESSAGE HANDLERS FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/// @brief Callback used to handle CONNECTION PARAMETER UPDATE REQUEST L2CAP Signaling message
void l2cap_sig_conn_param_upd_req_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_conn_param_upd_req_t* p_pdu,
                                          co_buf_t* p_buf)
{
    #if (HL_LE_CENTRAL)
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
    if(GETB(p_con->hdr.info_bf, GAPC_ROLE) == ROLE_MASTER)
    {
        gap_le_con_param_nego_with_ce_len_t param;
        memcpy(&(param.hdr), &(p_pdu->intv_min), sizeof(gap_le_con_param_nego_t));
        param.ce_len_min = 1;
        param.ce_len_max = 1;

        // Clear parameter request feature in the environment because not supported by peer
        p_con->peer_features[0] &= ~CO_BIT(GAP_LE_FEAT_CON_PARAM_REQ_PROC);

        // check if peer request can be handed
        if(!gapc_le_con_up_handle_peer_nego(conidx, p_con, &param, true, pkt_id))
        {
            // by default reject
            gapc_con_up_send_l2cap_param_resp(conidx, pkt_id, false);
        }
    }
    #endif // (HL_LE_CENTRAL)
}

/**
 ****************************************************************************************
 * @brief Callback used to handle CONNECTION PARAMETER UPDATE RESPONSE L2CAP Signaling message
 *
 * @param[in] conidx         Connection Index
 * @param[in] pkt_id         Packet identifier
 * @param[in] p_pdu          L2CAP PDU information received
 * @param[in] p_buf          Buffer that contains remaining data (not extracted)
 ****************************************************************************************
 */
void l2cap_sig_conn_param_upd_resp_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_conn_param_upd_rsp_t* p_pdu,
                                           co_buf_t* p_buf)
{
    #if (HL_LE_PERIPHERAL)
    l2cap_sig_proc_t* p_sig_proc = l2cap_sig_proc_pick(conidx, 0);

    if ((p_sig_proc != NULL) && (p_sig_proc->proc_id == L2CAP_SIG_PROC_CON_PARAM_UPDATE) && (p_sig_proc->pkt_id == pkt_id))
    {
        uint16_t status = ((p_pdu->response != L2CAP_SIG_CON_PARAM_ACCEPT) ? GAP_ERR_REJECTED : GAP_ERR_NO_ERROR);

        // Continue procedure execution
        l2cap_sig_proc_continue(conidx, 0, L2CAP_SIG_PROC_RSP_RECEIVED, status);
    }
    #endif // (HL_LE_PERIPHERAL)
}


/*
 * HCI HANDLERS
 ****************************************************************************************
 */

/// @brief Connection update parameter is done. Master role.
/// Slave application should handle the connection update parameter complete event!
void gapc_hci_le_con_update_cmp_evt_handler(uint8_t evt_code, struct hci_le_con_update_cmp_evt const *p_evt)
{
    uint8_t conidx = gapc_get_conidx(p_evt->conhdl);
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

    if (p_con != NULL)
    {
        uint16_t status = RW_ERR_HCI_TO_HL(p_evt->status);

        if(status == GAP_ERR_NO_ERROR)
        {
            const gapc_le_config_cb_t* p_cbs = gapc_env.p_le_config_cbs;
            gap_le_con_param_t param;
            memcpy(&param, &p_evt->con_interval, sizeof(gap_le_con_param_t));

            if(p_cbs->param_updated != NULL)
            {
                p_cbs->param_updated(conidx, p_con->hdr.dummy, &param);
            }

            // Inform le event clients about new connection parameters
            gapc_le_event_client_provide_con_param(conidx, &param);
            // update connection information for EATT L2CAP collision mitigation
            gatt_con_info_set(conidx, p_evt->con_interval, p_evt->con_latency);
        }

        // Transition if connection update initiating procedure is on-going
        gapc_proc_transition_if_active(conidx, &gapc_le_con_up_proc_init_itf,
                                       GAPC_LE_CON_UPD_PROC_HCI_CMP_EVT_RECV, status);
    }
}

/// @brief Connection update parameter request.
void gapc_hci_le_rem_con_param_req_evt_handler(uint8_t evt_code, struct hci_le_rem_con_param_req_evt const *p_evt)
{
    uint8_t conidx = gapc_get_conidx(p_evt->conhdl);
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
    gap_le_con_param_nego_with_ce_len_t param;
    memcpy(&(param.hdr), &(p_evt->interval_min), sizeof(gap_le_con_param_nego_t));
    param.ce_len_min = 1;
    param.ce_len_max = 1;


    // check if peer request can be handed
    if(!gapc_le_con_up_handle_peer_nego(conidx, p_con, &param, false, 0))
    {
        bool auto_accept = (   (GETB(p_con->hdr.info_bf, GAPC_ROLE) == ROLE_SLAVE)
                            && gapc_le_con_is_param_update_valid(&(param.hdr)));

        // by default reject
        gapc_le_con_send_hci_le_con_update_resp(conidx, auto_accept, &param);
    }
}

/*
 * EXTERNALS FUNCTIONS
 ****************************************************************************************
 */


uint16_t gapc_le_con_param_update(uint8_t conidx, uint32_t dummy, const gap_le_con_param_nego_with_ce_len_t* p_param,
                                  gapc_proc_cmp_cb cmp_cb)
{
    uint16_t status;
    do
    {
        gapc_le_con_up_proc_init_t* p_proc;

        // perform a sanity check of connection parameters
        if ((p_param == NULL) || !gapc_le_con_is_param_update_valid(&(p_param->hdr)))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        if(cmp_cb == NULL)
        {
            status = GAP_ERR_MISSING_CALLBACK;
            break;
        }

        status = gapc_proc_le_create(conidx, sizeof(gapc_le_con_up_proc_init_t), &gapc_le_con_up_proc_init_itf,
                                     (hl_proc_t**) &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        p_proc->conidx = conidx;
        p_proc->cmp_cb = cmp_cb;
        p_proc->dummy  = dummy;
        p_proc->param  = *p_param;
    } while(0);

    return (status);
}

uint16_t gapc_le_con_param_update_cfm(uint8_t conidx, bool accept, uint16_t ce_len_min, uint16_t ce_len_max)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    do
    {
        gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
        gapc_le_con_up_proc_resp_t* p_proc;
        if(p_con == NULL) break;
        p_proc = (gapc_le_con_up_proc_resp_t*) gapc_proc_get(conidx);

        // check validity of procedure
        if((p_proc == NULL) || (p_proc->hdr.p_itf != &gapc_le_con_up_proc_resp_itf) || !p_proc->wait_app_cfm) break;
        p_proc->wait_app_cfm     = false;
        p_proc->accepted         = accept;
        p_proc->param.ce_len_min = ce_len_min;
        p_proc->param.ce_len_max = ce_len_max;

        // nothing to do if in procedure handler
        if(GETB(p_con->hdr.info_bf, GAPC_IN_PROC)) break;
        gapc_proc_transition(conidx, HL_PROC_FINISHED, GAP_ERR_NO_ERROR);
    } while(0);

    return (status);
}

#if (HL_LE_PERIPHERAL)
uint16_t gapc_le_set_pref_slave_latency(uint8_t conidx, uint32_t dummy, uint16_t latency, gapc_proc_cmp_cb cmp_cb)
{
    gapc_le_con_up_proc_set_pref_slave_latency_t* p_proc;
    uint16_t status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_le_con_up_proc_set_pref_slave_latency_t),
                                &gapc_le_con_up_proc_set_pref_slave_latency_itf, (gapc_proc_simple_t**)&p_proc);
    if(status == GAP_ERR_NO_ERROR)
    {
        p_proc->latency = latency;
    }

    return (status);
}

uint16_t gapc_le_set_pref_slave_evt_dur(uint8_t conidx, uint32_t dummy, uint16_t duration, bool single_tx,
                                     gapc_proc_cmp_cb cmp_cb)
{
    gapc_le_con_up_proc_set_pref_slave_evt_dur_t* p_proc;
    uint16_t status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_le_con_up_proc_set_pref_slave_evt_dur_t),
                                &gapc_le_con_up_proc_set_pref_slave_evt_dur_itf, (gapc_proc_simple_t**) &p_proc);
    if(status == GAP_ERR_NO_ERROR)
    {
        p_proc->duration  = duration;
        p_proc->single_tx = single_tx;
    }

    return (status);
}
#endif // (HL_LE_PERIPHERAL)

#endif // (BLE_GAPC)
/// @} GAPC
