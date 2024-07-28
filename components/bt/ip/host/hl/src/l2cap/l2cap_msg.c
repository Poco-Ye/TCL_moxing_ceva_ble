/**
 ****************************************************************************************
 * @file l2cap_msg.c
 *
 * @brief  L2CAP Message API handler
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup L2CAP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"            // IP configuration
#if (BLE_L2CAP && HOST_MSG_API)
#include "l2cap.h"                  // Native API
#include "l2cap_int.h"              // Internals
#include "../inc/l2cap_sig.h"       //  SIG Protocol and API
#include "../inc/l2cap_hl_api.h"    // For RX control function

#include "gapm.h"                   // For token id generation

#if (HOST_MSG_API)
#include "ke_msg.h"                 // KE Message handler
#include "ke_task.h"                // KE Task handler
#endif // (HOST_MSG_API)
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

__STATIC void l2cap_msg_coc_create_req_cb(uint8_t conidx, uint16_t token, uint8_t nb_chan, uint16_t spsm, uint16_t peer_rx_mtu);
__STATIC void l2cap_msg_sdu_rx_cb(uint8_t conidx, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu);
__STATIC void l2cap_msg_sdu_sent_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu);
__STATIC void l2cap_msg_coc_create_cmp_cb(uint8_t conidx, uint16_t dummy, uint16_t status, uint8_t nb_chan);
__STATIC void l2cap_msg_coc_created_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t local_rx_mtu, uint16_t peer_rx_mtu);
__STATIC void l2cap_msg_coc_reconfigure_cmp_cb(uint8_t conidx, uint16_t dummy, uint16_t status);
__STATIC void l2cap_msg_coc_mtu_changed_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t local_rx_mtu, uint16_t peer_rx_mtu);
__STATIC void l2cap_msg_coc_terminate_cmp_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t status);
__STATIC void l2cap_msg_coc_terminated_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t reason);

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Default callback set to handle LE-PSM events
__STATIC const l2cap_coc_spsm_cb_t l2cap_msg_spsm_cb =
{
        .cb_coc_connect_req      = l2cap_msg_coc_create_req_cb,
};

/// Default callback set to handle COC channel events
__STATIC const l2cap_chan_coc_cb_t l2cap_msg_coc_cb =
{
        .cb_sdu_rx              = l2cap_msg_sdu_rx_cb,
        .cb_sdu_sent            = l2cap_msg_sdu_sent_cb,
        .cb_coc_create_cmp      = l2cap_msg_coc_create_cmp_cb,
        .cb_coc_created         = l2cap_msg_coc_created_cb,
        .cb_coc_reconfigure_cmp = l2cap_msg_coc_reconfigure_cmp_cb,
        .cb_coc_mtu_changed     = l2cap_msg_coc_mtu_changed_cb,
        .cb_coc_terminate_cmp   = l2cap_msg_coc_terminate_cmp_cb,
        .cb_coc_terminated      = l2cap_msg_coc_terminated_cb,
};

#if (RW_DEBUG)
/// Default callback set to handle Fixed channel events
__STATIC const l2cap_chan_cb_t l2cap_msg_fix_chan_cb =
{
        .cb_sdu_rx              = l2cap_msg_sdu_rx_cb,
        .cb_sdu_sent            = l2cap_msg_sdu_sent_cb,
};
#endif // (RW_DEBUG)

/// Reception SDU meta-data
typedef struct l2cap_msg_rx_sdu_meta
{
    /// Unique identifier to retrieve targeted SDU
    uint16_t token;
} l2cap_msg_rx_sdu_meta_t;

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send a basic command complete event
 *
 * @param[in] p_cmd     Command parameters
 * @param[in] dest_id   Destination task
 * @param[in] status    Command execution status
 ****************************************************************************************
 */
__STATIC void l2cap_msg_send_basic_cmp_evt(l2cap_cmd_t* p_cmd, uint16_t dest_id, uint16_t status)
{
    l2cap_cmp_evt_t* p_cmp_evt;
    // Send Command complete event to upper layer task
    p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, dest_id, TASK_L2CAP, l2cap_cmp_evt);
    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->cmd_code = p_cmd->cmd_code;
        p_cmp_evt->dummy    = p_cmd->dummy;
        p_cmp_evt->status   = status;

        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Function call when peer device requests to create several Connection oriented
 *        channels
 *
 *        In response of this function, the upper layer application must call
 *        @see l2cap_coc_connect_cfm function
 *
 * @param[in] conidx      Connection Index
 * @param[in] token       Token provided by L2CAP module that must be reused in confirmation function
 * @param[in] nb_chan     Number of L2CAP channel requested to be created in parallel
 * @param[in] spsm      Simplified Protocol/Service Multiplexer
 * @param[in] peer_rx_mtu Peer device Maximum Transmit Unit reception size
 ****************************************************************************************
 */
__STATIC void l2cap_msg_coc_create_req_cb(uint8_t conidx, uint16_t token, uint8_t nb_chan, uint16_t spsm, uint16_t peer_rx_mtu)
{
    l2cap_coc_connect_req_ind_t* p_req_ind;

    // get LE-PSM structure to retrieve destination task
    l2cap_spsm_t* p_spsm = l2cap_coc_spsm_get(spsm);
    ASSERT_ERR(p_spsm != NULL);

    // Send Connection creation message
    p_req_ind = KE_MSG_ALLOC(L2CAP_REQ_IND, p_spsm->dest_task_nbr, TASK_L2CAP, l2cap_coc_connect_req_ind);
    if(p_req_ind != NULL)
    {
        p_req_ind->req_ind_code = L2CAP_COC_CONNECT;
        p_req_ind->conidx       = conidx;
        p_req_ind->token        = token;
        p_req_ind->nb_chan      = nb_chan;
        p_req_ind->spsm       = spsm;
        p_req_ind->peer_rx_mtu  = peer_rx_mtu;

        ke_msg_send(p_req_ind);
    }
}

/**
 ****************************************************************************************
 * @brief The received SDU buffer must be acquired by upper application module before
 *        function return.
 *        When SDU process is done, the corresponding SDU buffer must be release to
 *        allocate new reception credits onto a L2CAP dynamic channel.
 *
 * @param[in] conidx    Connection Index
 * @param[in] chan_lid  Connected L2CAP channel local index
 * @param[in] status    Reception status
 * @param[in] p_sdu     Buffer that contains SDU data
 ****************************************************************************************
 */
__STATIC void l2cap_msg_sdu_rx_cb(uint8_t conidx, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu)
{
    l2cap_chan_t* p_chan = l2cap_chan_get_env(conidx, chan_lid);
    ASSERT_ERR(p_chan != NULL);

    l2cap_sdu_rx_req_ind_t* p_req_ind;

    // Send Command complete event to upper layer task
    p_req_ind = KE_MSG_ALLOC_DYN(L2CAP_REQ_IND, p_chan->dest_task_nbr, TASK_L2CAP, l2cap_sdu_rx_req_ind,
                                 co_buf_data_len(p_sdu));
    if(p_req_ind != NULL)
    {
        l2cap_con_env_t* p_con = l2cap_get_con_env(conidx);
        ASSERT_ERR(p_con != NULL);

        p_req_ind->req_ind_code = L2CAP_SDU_RX;
        p_req_ind->token        = gapm_token_id_get();
        p_req_ind->conidx       = conidx;
        p_req_ind->chan_lid     = chan_lid;
        p_req_ind->status       = status;
        p_req_ind->length       = co_buf_data_len(p_sdu);

        co_buf_copy_data_to_mem(p_sdu, p_req_ind->data, p_req_ind->length);

        // put SDU in a specific message queue to release buffers only when they are processed by host
        // - valid only if flow control is enabled
        if(GETB(p_chan->config_bf, L2CAP_CHAN_CREDIT_FLOW_EN))
        {
            l2cap_msg_rx_sdu_meta_t* p_sdu_meta = (l2cap_msg_rx_sdu_meta_t*) co_buf_metadata(p_sdu);
            p_sdu_meta->token = p_req_ind->token;
            co_buf_acquire(p_sdu);
            co_list_push_back(&(p_con->msg_api_sdu_queue), &(p_sdu->hdr));
        }

        ke_msg_send(p_req_ind);
    }
}

/**
 ****************************************************************************************
 * @brief Function called when SDU has been transmitted or if an error occurs
 *
 * @param[in] conidx    Connection Index
 * @param[in] dummy     Dummy parameter provided by upper layer for command execution
 * @param[in] chan_lid  L2CAP channel local index
 * @param[in] status    Status of the procedure (see enum #hl_err)
 * @param[in] p_sdu     Pointer to SDU transmitted
 ****************************************************************************************
 */
__STATIC void l2cap_msg_sdu_sent_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu)
{
    l2cap_chan_t* p_chan = l2cap_chan_get_env(conidx, chan_lid);
    ASSERT_ERR(p_chan != NULL);

    l2cap_sdu_send_cmp_evt_t* p_cmp_evt;

    // Send Command complete event to upper layer task
    p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, p_chan->dest_task_nbr, TASK_L2CAP, l2cap_chan_cmp_evt);
    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->cmd_code = L2CAP_SDU_SEND;
        p_cmp_evt->dummy    = dummy;
        p_cmp_evt->conidx   = conidx;
        p_cmp_evt->chan_lid = chan_lid;
        p_cmp_evt->status   = status;

        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Function called when Connection Oriented Channel creation procedure is finished
 *
 * @param[in] conidx    Connection Index
 * @param[in] dummy     Dummy parameter provided by upper layer for command execution
 * @param[in] status    Status of the procedure (see enum #hl_err)
 * @param[in] nb_chan   Number of L2CAP channel created.
 ****************************************************************************************
 */
__STATIC void l2cap_msg_coc_create_cmp_cb(uint8_t conidx, uint16_t dummy, uint16_t status, uint8_t nb_chan)
{
    l2cap_coc_create_cmp_evt_t* p_cmp_evt;

    // Send Command complete event to upper layer task
    p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, l2cap_env.dest_task_nbr, TASK_L2CAP, l2cap_coc_create_cmp_evt);
    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->cmd_code = L2CAP_COC_CREATE;
        p_cmp_evt->dummy    = dummy;
        p_cmp_evt->conidx   = conidx;
        p_cmp_evt->status   = status;
        p_cmp_evt->nb_chan  = nb_chan;

        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Function called when a new Connection Oriented Channel is created
 *
 * @param[in] conidx       Connection Index
 * @param[in] dummy        Dummy parameter provided by upper layer for command execution
 * @param[in] chan_lid     Connected L2CAP channel local index
 * @param[in] local_rx_mtu Local device Maximum Transmit Unit reception size
 * @param[in] peer_rx_mtu  Peer device Maximum Transmit Unit reception size
 ****************************************************************************************
 */
__STATIC void l2cap_msg_coc_created_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid,
                                       uint16_t local_rx_mtu, uint16_t peer_rx_mtu)
{
    l2cap_coc_created_ind_t* p_ind;
    l2cap_chan_t* p_chan = l2cap_chan_get_env(conidx, chan_lid);
    ASSERT_ERR(p_chan != NULL);

    // Ensure
    p_chan->dest_task_nbr = l2cap_env.dest_task_nbr;

    // Send Indication to upper layer task
    p_ind = KE_MSG_ALLOC(L2CAP_IND, l2cap_env.dest_task_nbr, TASK_L2CAP, l2cap_coc_created_ind);
    if(p_ind != NULL)
    {
        p_ind->ind_code     = L2CAP_COC_CREATED;
        p_ind->dummy        = dummy;
        p_ind->conidx       = conidx;
        p_ind->chan_lid     = chan_lid;
        p_ind->local_rx_mtu = local_rx_mtu;
        p_ind->peer_rx_mtu  = peer_rx_mtu;

        ke_msg_send(p_ind);
    }
}

/**
 ****************************************************************************************
 * @brief Function called when Reconfigure L2CAP channel MTU is terminated
 *
 * @param[in] conidx    Connection Index
 * @param[in] dummy     Dummy parameter provided by upper layer for command execution
 * @param[in] status    Status of the procedure (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC void l2cap_msg_coc_reconfigure_cmp_cb(uint8_t conidx, uint16_t dummy, uint16_t status)
{
    l2cap_coc_reconfigure_cmp_evt_t* p_cmp_evt;
    l2cap_sig_proc_t* p_proc = l2cap_sig_proc_pick(conidx, 0);

    // Send Command complete event to upper layer task
    p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, l2cap_env.dest_task_nbr, TASK_L2CAP, l2cap_coc_reconfigure_cmp_evt);
    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->cmd_code = (p_proc->proc_id == L2CAP_SIG_PROC_COC_RECONFIGURE)
                            ? L2CAP_COC_RECONFIGURE
                            : L2CAP_DBG_COC_RECONFIGURE;
        p_cmp_evt->dummy    = dummy;
        p_cmp_evt->conidx   = conidx;
        p_cmp_evt->status   = status;

        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Function called when Local or Peer RX MTU size has been changed onto the L2CAP channel
 *
 * @param[in] conidx       Connection Index
 * @param[in] dummy        Dummy parameter provided by upper layer for command execution
 * @param[in] chan_lid     L2CAP channel local index
 * @param[in] local_rx_mtu Local device Maximum Transmit Unit reception size
 * @param[in] peer_rx_mtu  Peer device Maximum Transmit Unit reception size
 ****************************************************************************************
 */
__STATIC void l2cap_msg_coc_mtu_changed_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t local_rx_mtu,
                                           uint16_t peer_rx_mtu)
{
    l2cap_coc_mtu_changed_ind_t* p_ind;
    l2cap_chan_t* p_chan = l2cap_chan_get_env(conidx, chan_lid);
    ASSERT_ERR(p_chan != NULL);

    // Ensure
    p_chan->dest_task_nbr = l2cap_env.dest_task_nbr;

    // Send Indication to upper layer task
    p_ind = KE_MSG_ALLOC(L2CAP_IND, l2cap_env.dest_task_nbr, TASK_L2CAP, l2cap_coc_mtu_changed_ind);
    if(p_ind != NULL)
    {
        p_ind->ind_code     = L2CAP_COC_MTU_CHANGED;
        p_ind->dummy        = dummy;
        p_ind->conidx       = conidx;
        p_ind->chan_lid     = chan_lid;
        p_ind->local_rx_mtu = local_rx_mtu;
        p_ind->peer_rx_mtu  = peer_rx_mtu;

        ke_msg_send(p_ind);
    }

}

/**
 ****************************************************************************************
 * @brief Function called when Connection Oriented Channel Termination procedure is finished
 *
 * @param[in] conidx    Connection Index
 * @param[in] dummy     Dummy parameter provided by upper layer for command execution
 * @param[in] status    Status of the procedure (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC void l2cap_msg_coc_terminate_cmp_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t status)
{
    l2cap_coc_terminate_cmp_evt_t* p_cmp_evt;

    // Send Command complete event to upper layer task
    p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, l2cap_env.dest_task_nbr, TASK_L2CAP, l2cap_chan_cmp_evt);
    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->cmd_code = L2CAP_COC_TERMINATE;
        p_cmp_evt->dummy    = dummy;
        p_cmp_evt->conidx   = conidx;
        p_cmp_evt->chan_lid = chan_lid;
        p_cmp_evt->status   = status;

        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Function called when a Connection Oriented Channel is terminated
 *
 * @param[in] conidx    Connection Index
 * @param[in] dummy     Dummy parameter provided by upper layer for command execution
 * @param[in] chan_lid  L2CAP channel local index
 * @param[in] reason    Termination Reason (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC void l2cap_msg_coc_terminated_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t reason)
{
    l2cap_coc_terminated_ind_t* p_ind;

    // Send Command complete event to upper layer task
    p_ind = KE_MSG_ALLOC(L2CAP_IND, l2cap_env.dest_task_nbr, TASK_L2CAP, l2cap_coc_terminated_ind);
    if(p_ind != NULL)
    {
        p_ind->ind_code = L2CAP_COC_TERMINATED;
        p_ind->dummy    = dummy;
        p_ind->conidx   = conidx;
        p_ind->chan_lid = chan_lid;
        p_ind->reason   = reason;

        ke_msg_send(p_ind);
    }
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

#if (RW_DEBUG)
void l2cap_msg_coc_rx_credit_add(l2cap_chan_coc_t* p_chan, uint16_t credits)
{
    l2cap_dbg_chan_rx_credit_added_ind_t* p_ind;
    // Send Command complete event to upper layer task
    p_ind = KE_MSG_ALLOC(L2CAP_IND, l2cap_env.dest_task_nbr, TASK_L2CAP, l2cap_dbg_chan_rx_credit_added_ind);
    if(p_ind != NULL)
    {
        p_ind->ind_code = L2CAP_DBG_CHAN_RX_CREDIT_ADDED;
        p_ind->dummy    = 0;
        p_ind->conidx   = p_chan->conidx;
        p_ind->chan_lid = p_chan->chan_lid;
        p_ind->credits  = credits;

        ke_msg_send(p_ind);
    }
}

void l2cap_msg_coc_error_detected(l2cap_chan_coc_t* p_chan, uint16_t status)
{
    l2cap_dbg_chan_error_ind_t* p_ind;
    // Send Command complete event to upper layer task
    p_ind = KE_MSG_ALLOC(L2CAP_IND, l2cap_env.dest_task_nbr, TASK_L2CAP, l2cap_dbg_chan_error_ind);
    if(p_ind != NULL)
    {
        p_ind->ind_code = L2CAP_DBG_CHAN_ERROR;
        p_ind->dummy    = 0;
        p_ind->conidx   = p_chan->conidx;
        p_ind->chan_lid = p_chan->chan_lid;
        p_ind->reason   = status;

        ke_msg_send(p_ind);
    }
}


void l2cap_msg_coc_tx_flow_off(l2cap_chan_coc_t* p_chan)
{
    l2cap_dbg_chan_tx_flow_off_ind_t* p_ind;
    // Send Command complete event to upper layer task
    p_ind = KE_MSG_ALLOC(L2CAP_IND, l2cap_env.dest_task_nbr, TASK_L2CAP, l2cap_dbg_chan_tx_flow_off_ind);
    if(p_ind != NULL)
    {
        p_ind->ind_code = L2CAP_DBG_CHAN_TX_FLOW_OFF;
        p_ind->dummy    = 0;
        p_ind->conidx   = p_chan->conidx;
        p_ind->chan_lid = p_chan->chan_lid;

        ke_msg_send(p_ind);
    }
}
#endif // (RW_DEBUG)

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handle ADD of a new supported SPSM
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_coc_spsm_add_cmd_handler(l2cap_coc_spsm_add_cmd_t* p_cmd, uint16_t src_id)
{
    uint16_t status = l2cap_coc_spsm_add(p_cmd->spsm, p_cmd->sec_lvl_bf, &l2cap_msg_spsm_cb);

    // Assign Source task to the created SPSM
    if(status == GAP_ERR_NO_ERROR)
    {
        l2cap_spsm_t* p_spsm = l2cap_coc_spsm_get(p_cmd->spsm);
        ASSERT_ERR(p_spsm != NULL);

        p_spsm->dest_task_nbr = src_id;
    }

    l2cap_msg_send_basic_cmp_evt((l2cap_cmd_t*) p_cmd, src_id, status);
}

/**
 ****************************************************************************************
 * @brief Handle Remove of a supported SPSM
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_coc_spsm_remove_cmd_handler(l2cap_coc_spsm_remove_cmd_t* p_cmd, uint16_t src_id)
{
    uint16_t status = l2cap_coc_spsm_remove(p_cmd->spsm);

    l2cap_msg_send_basic_cmp_evt((l2cap_cmd_t*) p_cmd, src_id, status);
}

/**
 ****************************************************************************************
 * @brief Handle Creation of a new L2CAP COC channel
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_coc_create_cmd_handler(l2cap_coc_create_cmd_t* p_cmd, uint16_t src_id)
{
    // ask for L2CAP COC creation
    uint16_t status = l2cap_coc_create(p_cmd->conidx, p_cmd->dummy, p_cmd->spsm, p_cmd->nb_chan, p_cmd->local_rx_mtu,
                                       &l2cap_msg_coc_cb);

    if(status == GAP_ERR_NO_ERROR)
    {
        l2cap_sig_proc_t* p_proc;
        // update Create procedure with destination task number
        l2cap_con_env_t* p_con = l2cap_env.p_con[p_cmd->conidx];
        ASSERT_ERR(p_con != NULL);

        // Retrieve COC Creation procedure
        p_proc = (l2cap_sig_proc_t*) p_con->sig.req_proc_queue.last;
        ASSERT_ERR(p_proc != NULL);

        // Store destination task
        p_proc->dest_task_nbr = src_id;
    }
    else
    {
        l2cap_coc_create_cmp_evt_t* p_cmp_evt;
        // Send Command complete event to upper layer task
        p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, src_id, TASK_L2CAP, l2cap_coc_create_cmp_evt);
        if(p_cmp_evt != NULL)
        {
            p_cmp_evt->cmd_code = p_cmd->cmd_code;
            p_cmp_evt->dummy    = p_cmd->dummy;
            p_cmp_evt->conidx   = p_cmd->conidx;
            p_cmp_evt->status   = status;
            p_cmp_evt->nb_chan  = 0;

            ke_msg_send(p_cmp_evt);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Handle Reconfiguration of some L2CAP COC channel
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */

__STATIC void l2cap_coc_reconfigure_cmd_handler(l2cap_coc_reconfigure_cmd_t* p_cmd, uint16_t src_id)
{
    // ask for L2CAP COC reconfigure
    uint16_t status = l2cap_coc_reconfigure(p_cmd->conidx, p_cmd->dummy, p_cmd->local_rx_mtu, p_cmd->nb_chan,
                                            p_cmd->chan_lid);

    if(status != GAP_ERR_NO_ERROR)
    {
       l2cap_coc_reconfigure_cmp_evt_t* p_cmp_evt;

        // Send Command complete event to upper layer task
        p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, src_id, TASK_L2CAP, l2cap_coc_reconfigure_cmp_evt);
        if(p_cmp_evt != NULL)
        {
            p_cmp_evt->cmd_code = p_cmd->cmd_code;
            p_cmp_evt->dummy    = p_cmd->dummy;
            p_cmp_evt->conidx   = p_cmd->conidx;
            p_cmp_evt->status   = status;

            ke_msg_send(p_cmp_evt);
        }
    }
    else
    {
        l2cap_sig_proc_t* p_proc;
        // update Create procedure with destination task number
        l2cap_con_env_t* p_con = l2cap_env.p_con[p_cmd->conidx];
        ASSERT_ERR(p_con != NULL);

        // Retrieve COC Creation procedure
        p_proc = (l2cap_sig_proc_t*) p_con->sig.req_proc_queue.last;
        ASSERT_ERR(p_proc != NULL);

        // Store destination task
        p_proc->dest_task_nbr = src_id;
    }
}


/**
 ****************************************************************************************
 * @brief Handle Termination of L2CAP COC channel
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_coc_terminate_cmd_handler(l2cap_coc_terminate_cmd_t* p_cmd, uint16_t src_id)
{
    // ask for L2CAP COC creation
    uint16_t status = l2cap_coc_terminate(p_cmd->conidx, p_cmd->dummy, p_cmd->chan_lid);

    if(status != GAP_ERR_NO_ERROR)
    {
        l2cap_coc_terminate_cmp_evt_t* p_cmp_evt;

        // Send Command complete event to upper layer task
        p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, src_id, TASK_L2CAP, l2cap_chan_cmp_evt);
        if(p_cmp_evt != NULL)
        {
            p_cmp_evt->cmd_code = p_cmd->cmd_code;
            p_cmp_evt->dummy    = p_cmd->dummy;
            p_cmp_evt->conidx   = p_cmd->conidx;
            p_cmp_evt->chan_lid = p_cmd->chan_lid;
            p_cmp_evt->status   = status;

            ke_msg_send(p_cmp_evt);
        }
    }
    else
    {
        l2cap_sig_proc_t* p_proc;
        // update Create procedure with destination task number
        l2cap_con_env_t* p_con = l2cap_env.p_con[p_cmd->conidx];
        ASSERT_ERR(p_con != NULL);

        // Retrieve COC Creation procedure
        p_proc = (l2cap_sig_proc_t*) p_con->sig.req_proc_queue.last;
        ASSERT_ERR(p_proc != NULL);

        // Store destination task
        p_proc->dest_task_nbr = src_id;
    }
}

/**
 ****************************************************************************************
 * @brief Handle transmission of SDU over L2CAP channel
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_sdu_send_cmd_handler(l2cap_sdu_send_cmd_t* p_cmd, uint16_t src_id)
{
    // ask for L2CAP COC creation
    uint16_t status;
    co_buf_t* p_sdu;

    if(co_buf_alloc(&p_sdu, L2CAP_BUFFER_HEADER_LEN, p_cmd->length, L2CAP_BUFFER_TAIL_LEN) != CO_BUF_ERR_NO_ERROR)
    {
        status = GAP_ERR_INSUFF_RESOURCES;
    }
    else
    {
        // Copy data
        if(co_buf_copy_data_from_mem(p_sdu, p_cmd->data, p_cmd->length) != CO_BUF_ERR_NO_ERROR)
        {
            ASSERT_ERR(0);
            status = GAP_ERR_UNEXPECTED;
        }
        #if (RW_DEBUG)
        else if(p_cmd->dbg_bf != 0)
        {
            // request channel to send SDU
            status = l2cap_chan_debug_sdu_send(p_cmd->conidx, p_cmd->dummy, p_cmd->chan_lid, p_cmd->dbg_bf, p_sdu);
        }
        #endif // (RW_DEBUG)
        else
        {
            // request channel to send SDU
            status = l2cap_chan_sdu_send(p_cmd->conidx, p_cmd->dummy, p_cmd->chan_lid, p_sdu);
        }

        co_buf_release(p_sdu);
    }


    if(status != GAP_ERR_NO_ERROR)
    {
        l2cap_sdu_send_cmp_evt_t* p_cmp_evt;

        // Send Command complete event to upper layer task
        p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, src_id, TASK_L2CAP, l2cap_chan_cmp_evt);
        if(p_cmp_evt != NULL)
        {
            p_cmp_evt->cmd_code = p_cmd->cmd_code;
            p_cmp_evt->dummy    = p_cmd->dummy;
            p_cmp_evt->conidx   = p_cmd->conidx;
            p_cmp_evt->chan_lid = p_cmd->chan_lid;
            p_cmp_evt->status   = status;

            ke_msg_send(p_cmp_evt);
        }
    }
}

#if (RW_DEBUG)
/**
 ****************************************************************************************
 * @brief Handle Creation of a fixed L2CAP channel
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_dbg_chan_fix_register_cmd_handler(l2cap_dbg_chan_fix_register_cmd_t* p_cmd, uint16_t src_id)
{
    // Create New Fixed channel
    uint8_t chan_lid;
    uint16_t status = l2cap_chan_fix_register(p_cmd->conidx, p_cmd->cid, p_cmd->mtu, &l2cap_msg_fix_chan_cb, &chan_lid);
    l2cap_dbg_chan_fix_register_cmp_evt_t* p_cmp_evt;

    if(status == GAP_ERR_NO_ERROR)
    {
        l2cap_chan_t* p_chan = l2cap_chan_get_env(p_cmd->conidx, chan_lid);
        ASSERT_ERR(p_chan != NULL);
        p_chan->dest_task_nbr = src_id;
    }

    // Send Command complete event to upper layer task
    p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, src_id, TASK_L2CAP, l2cap_chan_cmp_evt);
    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->cmd_code = p_cmd->cmd_code;
        p_cmp_evt->dummy    = p_cmd->dummy;
        p_cmp_evt->conidx   = p_cmd->conidx;
        p_cmp_evt->chan_lid = chan_lid;
        p_cmp_evt->status   = status;

        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Handle Creation of a dynamic L2CAP channel
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_dbg_chan_dyn_register_cmd_handler(l2cap_dbg_chan_dyn_register_cmd_t* p_cmd, uint16_t src_id)
{
    // Create New Fixed channel
    uint8_t chan_lid;
    l2cap_chan_coc_t* p_chan;
    uint16_t status = l2cap_chan_dyn_reserve(p_cmd->conidx, &chan_lid, &p_chan);
    l2cap_dbg_chan_dyn_register_cmp_evt_t* p_cmp_evt;

    if(status == GAP_ERR_NO_ERROR)
    {
        ASSERT_ERR(p_chan != NULL);

        // Setup channel
        p_chan->dest_task_nbr = src_id;

        p_chan->rx_cid        = p_cmd->rx_cid;
        p_chan->tx_cid        = p_cmd->tx_cid;
        p_chan->rx_mtu        = p_cmd->rx_mtu;
        p_chan->tx_mtu        = p_cmd->tx_mtu;
        p_chan->rx_mps        = p_cmd->rx_mps;
        p_chan->tx_mps        = p_cmd->tx_mps;
        p_chan->rx_credit     = p_cmd->rx_credit;
        p_chan->rx_credit_max = p_cmd->rx_credit;
        p_chan->tx_credit     = p_cmd->tx_credit;
        p_chan->p_cb          =  &l2cap_msg_coc_cb;

        SETB(p_chan->config_bf, L2CAP_CHAN_DBG_MODE_EN, true);

        // enable it
        l2cap_chan_enable_set((l2cap_chan_t*) p_chan, true);
    }

    // Send Command complete event to upper layer task
    p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, src_id, TASK_L2CAP, l2cap_chan_cmp_evt);
    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->cmd_code = p_cmd->cmd_code;
        p_cmp_evt->dummy    = p_cmd->dummy;
        p_cmp_evt->conidx   = p_cmd->conidx;
        p_cmp_evt->chan_lid = chan_lid;
        p_cmp_evt->status   = status;

        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Handle Destruction of a L2CAP channel
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_dbg_chan_unregister_cmd_handler(l2cap_dbg_chan_unregister_cmd_t* p_cmd, uint16_t src_id)
{
    l2cap_dbg_chan_unregister_cmp_evt_t* p_cmp_evt;
    uint16_t status = l2cap_chan_unregister(p_cmd->conidx, p_cmd->chan_lid);

    // Send Command complete event to upper layer task
    p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, src_id, TASK_L2CAP, l2cap_chan_cmp_evt);
    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->cmd_code = p_cmd->cmd_code;
        p_cmp_evt->dummy    = p_cmd->dummy;
        p_cmp_evt->conidx   = p_cmd->conidx;
        p_cmp_evt->chan_lid = p_cmd->chan_lid;
        p_cmp_evt->status   = status;

        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Add some credit to a dynamic L2CAP channel
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_dbg_chan_tx_credit_add_cmd_handler(l2cap_dbg_chan_tx_credit_add_cmd_t* p_cmd, uint16_t src_id)
{
    l2cap_dbg_chan_tx_credit_add_cmp_evt_t* p_cmp_evt;
    uint16_t status = l2cap_chan_tx_credit_add(p_cmd->conidx, p_cmd->chan_lid, p_cmd->credit);

    // Send Command complete event to upper layer task
    p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, src_id, TASK_L2CAP, l2cap_chan_cmp_evt);
    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->cmd_code = p_cmd->cmd_code;
        p_cmp_evt->dummy    = p_cmd->dummy;
        p_cmp_evt->conidx   = p_cmd->conidx;
        p_cmp_evt->chan_lid = p_cmd->chan_lid;
        p_cmp_evt->status   = status;

        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Set configuration of an L2CAP channel
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_dbg_chan_config_set_cmd_handler(l2cap_dbg_chan_config_set_cmd_t* p_cmd, uint16_t src_id)
{
    l2cap_dbg_chan_config_set_cmp_evt_t* p_cmp_evt;
    uint16_t status = GAP_ERR_NO_ERROR;
    // Retrieve channel
    l2cap_chan_t* p_chan = l2cap_chan_get_env(p_cmd->conidx, p_cmd->chan_lid);

    if(p_chan == NULL)
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }

    if((status == GAP_ERR_NO_ERROR) && GETB(p_cmd->update_bf, L2CAP_CHAN_UP_EN_STATE))
    {
        status = l2cap_chan_enable_set(p_chan, GETB(p_cmd->config_bf, L2CAP_CHAN_EN));
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        if(GETB(p_chan->config_bf, L2CAP_CHAN_FIX))
        {
            if(GETB(p_cmd->update_bf, L2CAP_CHAN_UP_RX_MTU))
            {
                p_chan->mtu = p_cmd->rx_mtu;
            }
        }
        else
        {
            l2cap_chan_coc_t* p_chan_coc = (l2cap_chan_coc_t*) p_chan;

            if(GETB(p_cmd->update_bf, L2CAP_CHAN_UP_TX_PAUSED_STATE))
            {
                l2cap_chan_tx_flow_set(p_chan_coc, 0, !GETB(p_cmd->config_bf, L2CAP_CHAN_PDU_TX_PAUSED));
            }

            if(GETB(p_cmd->update_bf, L2CAP_CHAN_UP_RX_MTU))
            {
                p_chan_coc->rx_mtu = p_cmd->rx_mtu;
            }

            if(GETB(p_cmd->update_bf, L2CAP_CHAN_UP_CREDIT_FLOW_EN_STATE))
            {
                SETB(p_chan_coc->config_bf, L2CAP_CHAN_CREDIT_FLOW_EN, GETB(p_cmd->config_bf, L2CAP_CHAN_CREDIT_FLOW_EN));
            }

            if(GETB(p_cmd->update_bf, L2CAP_CHAN_UP_TX_MTU))
            {
                p_chan_coc->tx_mtu = p_cmd->tx_mtu;
            }

            if(GETB(p_cmd->update_bf, L2CAP_CHAN_UP_RX_MPS))
            {
                p_chan_coc->rx_mps = p_cmd->rx_mps;
            }

            if(GETB(p_cmd->update_bf, L2CAP_CHAN_UP_TX_MPS))
            {
                p_chan_coc->tx_mps = p_cmd->tx_mps;
            }
        }
    }

    // Send Command complete event to upper layer task
    p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, src_id, TASK_L2CAP, l2cap_chan_cmp_evt);
    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->cmd_code = p_cmd->cmd_code;
        p_cmp_evt->dummy    = p_cmd->dummy;
        p_cmp_evt->conidx   = p_cmd->conidx;
        p_cmp_evt->chan_lid = p_cmd->chan_lid;
        p_cmp_evt->status   = status;

        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Retrieve configuration of a L2CAP channel
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_dbg_chan_info_get_handler(l2cap_dbg_chan_info_get_cmd_t* p_cmd, uint16_t src_id)
{
    l2cap_dbg_chan_info_get_cmp_evt_t* p_cmp_evt;
    l2cap_chan_t* p_chan = l2cap_chan_get_env(p_cmd->conidx, p_cmd->chan_lid);
    uint16_t status = (p_chan == NULL) ? GAP_ERR_COMMAND_DISALLOWED : GAP_ERR_NO_ERROR;

    // Send Command complete event to upper layer task
    p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, src_id, TASK_L2CAP, l2cap_dbg_chan_info_get_cmp_evt);
    if(p_cmp_evt != NULL)
    {
        if(p_chan != NULL)
        {
            p_cmp_evt->chan_type = GETB(p_chan->config_bf, L2CAP_CHAN_FIX) ? L2CAP_CHAN_FIX : L2CAP_CHAN_DYNAMIC;
            p_cmp_evt->config_bf = GETF(p_chan->config_bf, L2CAP_CHAN_CONFIG_INFO);

            p_cmp_evt->rx_cid    = p_chan->rx_cid;
            p_cmp_evt->tx_cid    = p_chan->tx_cid;
            if(p_cmp_evt->chan_type == L2CAP_CHAN_DYNAMIC)
            {
                l2cap_chan_coc_t* p_chan_coc = (l2cap_chan_coc_t*) p_chan;
                p_cmp_evt->rx_mtu    = p_chan_coc->rx_mtu;
                p_cmp_evt->tx_mtu    = p_chan_coc->tx_mtu;
                p_cmp_evt->rx_mps    = p_chan_coc->rx_mps;
                p_cmp_evt->tx_mps    = p_chan_coc->tx_mps;
                p_cmp_evt->rx_credit = p_chan_coc->rx_credit;
                p_cmp_evt->tx_credit = p_chan_coc->tx_credit;
            }
            else
            {
                p_cmp_evt->rx_mtu    = p_chan->mtu;
                p_cmp_evt->tx_mtu    = p_chan->mtu;
                p_cmp_evt->rx_mps    = 0;
                p_cmp_evt->tx_mps    = 0;
                p_cmp_evt->rx_credit = 0;
                p_cmp_evt->tx_credit = 0;
            }
        }

        p_cmp_evt->cmd_code = p_cmd->cmd_code;
        p_cmp_evt->dummy    = p_cmd->dummy;
        p_cmp_evt->conidx   = p_cmd->conidx;
        p_cmp_evt->chan_lid = p_cmd->chan_lid;
        p_cmp_evt->status   = status;

        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Handle Reconfiguration of some L2CAP COC channel
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_dbg_coc_reconfigure_cmd_handler(l2cap_dbg_coc_reconfigure_cmd_t* p_cmd, uint16_t src_id)
{
    // ask for L2CAP COC reconfigure
    uint16_t status = l2cap_coc_dbg_reconfigure(p_cmd->conidx, p_cmd->dummy, p_cmd->local_rx_mtu, p_cmd->local_rx_mps,
                                                p_cmd->nb_chan, p_cmd->chan_lid);

    if(status != GAP_ERR_NO_ERROR)
    {
       l2cap_dbg_coc_reconfigure_cmp_evt_t* p_cmp_evt;

        // Send Command complete event to upper layer task
        p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, src_id, TASK_L2CAP, l2cap_coc_reconfigure_cmp_evt);
        if(p_cmp_evt != NULL)
        {
            p_cmp_evt->cmd_code = p_cmd->cmd_code;
            p_cmp_evt->dummy    = p_cmd->dummy;
            p_cmp_evt->conidx   = p_cmd->conidx;
            p_cmp_evt->status   = status;

            ke_msg_send(p_cmp_evt);
        }
    }
    else
    {
        l2cap_sig_proc_t* p_proc;
        // update Create procedure with destination task number
        l2cap_con_env_t* p_con = l2cap_env.p_con[p_cmd->conidx];
        ASSERT_ERR(p_con != NULL);

        // Retrieve COC Creation procedure
        p_proc = (l2cap_sig_proc_t*) p_con->sig.req_proc_queue.last;
        ASSERT_ERR(p_proc != NULL);

        // Store destination task
        p_proc->dest_task_nbr = src_id;
    }
}

/**
 ****************************************************************************************
 * @brief Enable or disable the reception of ACL packet for connection
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_dbg_rx_ctrl_cmd_handler(l2cap_dbg_rx_ctrl_cmd_t* p_cmd, uint16_t src_id)
{
    // Control RX flow for the connection
    uint16_t status = l2cap_rx_ctrl(p_cmd->conidx, p_cmd->enable);
    l2cap_dbg_rx_ctrl_cmp_evt_t* p_cmp_evt;

    // Send Command complete event to upper layer task
    p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, src_id, TASK_L2CAP, l2cap_dbg_rx_ctrl_cmp_evt);
    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->cmd_code = p_cmd->cmd_code;
        p_cmp_evt->dummy    = p_cmd->dummy;
        p_cmp_evt->conidx   = p_cmd->conidx;
        p_cmp_evt->status   = status;

        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Enable or disable enhanced L2CAP COC negotiation usage
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_dbg_coc_enhanced_nego_ctrl_cmd_handler(l2cap_dbg_coc_enhanced_nego_ctrl_cmd_t* p_cmd, uint16_t src_id)
{
    // Control L2CAP COC negotiation type
    uint16_t status = l2cap_coc_enhanced_nego_set(p_cmd->conidx, p_cmd->enable);
    l2cap_dbg_coc_enhanced_nego_ctrl_cmp_evt_t* p_cmp_evt;

    // Send Command complete event to upper layer task
    p_cmp_evt = KE_MSG_ALLOC(L2CAP_CMP_EVT, src_id, TASK_L2CAP, l2cap_dbg_coc_enhanced_nego_ctrl_cmp_evt);
    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->cmd_code = p_cmd->cmd_code;
        p_cmp_evt->dummy    = p_cmd->dummy;
        p_cmp_evt->conidx   = p_cmd->conidx;
        p_cmp_evt->status   = status;

        ke_msg_send(p_cmp_evt);
    }
}
#endif // (RW_DEBUG)

/**
 ****************************************************************************************
 * @brief Handle confirmation of L2CAP COC channel creation
 *
 * @param[in] p_cfm     Pointer to confirmation parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_coc_connect_cfm_handler(l2cap_coc_connect_cfm_t* p_cfm, uint16_t src_id)
{
    l2cap_env.dest_task_nbr = src_id;
    // confirm channel creation
    l2cap_coc_connect_cfm(p_cfm->conidx, p_cfm->token, p_cfm->nb_chan, p_cfm->local_rx_mtu, &l2cap_msg_coc_cb);
}

/**
 ****************************************************************************************
 * @brief Handle confirmation of SDU reception
 *
 * @param[in] p_cfm     Pointer to confirmation parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
__STATIC void l2cap_sdu_rx_req_cfm_handler(l2cap_sdu_rx_cfm_t* p_cfm, uint16_t src_id)
{
    uint8_t conidx = p_cfm->conidx;

    if((conidx < HOST_CONNECTION_MAX) && (l2cap_env.p_con[conidx] != NULL))
    {
        l2cap_con_env_t* p_con = l2cap_env.p_con[conidx];

        co_buf_t* p_sdu = (co_buf_t*)co_list_pick(&(p_con->msg_api_sdu_queue));

        while(p_sdu != NULL)
        {
            l2cap_msg_rx_sdu_meta_t* p_sdu_meta = (l2cap_msg_rx_sdu_meta_t*) co_buf_metadata(p_sdu);
            // find SDU that correspond to provided token
            if(p_sdu_meta->token == p_cfm->token)
            {
                // remove SDU from list
                co_list_extract(&(p_con->msg_api_sdu_queue), &(p_sdu->hdr));
                // release buffer
                co_buf_release(p_sdu);
                break; // nothing more to do
            }

            // not found check next buffer
            p_sdu = (co_buf_t*) p_sdu->hdr.next;
        }
    }
}

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
int l2cap_default_msg_handler(ke_msg_id_t const msgid, void *p_param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Check if it's a L2CAP message and it's not sent to itself
    if (MSG_T(msgid) == TASK_ID_L2CAP && (dest_id != src_id))
    {
        // prepare unknown message indication
        l2cap_unknown_msg_ind_t* p_ind = KE_MSG_ALLOC(L2CAP_IND, src_id, dest_id, l2cap_unknown_msg_ind);

        p_ind->ind_code = L2CAP_UNKNOWN_MSG;
        p_ind->dummy    = 0;
        p_ind->msg_id   = msgid;

        // send event
        ke_msg_send(p_ind);
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle reception of a L2CAP command message
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int l2cap_cmd_msg_handler(ke_msg_id_t const msgid, l2cap_cmd_t *p_cmd, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    switch(p_cmd->cmd_code)
    {
        case L2CAP_COC_SPSM_ADD:               { l2cap_coc_spsm_add_cmd_handler((l2cap_coc_spsm_add_cmd_t*) p_cmd, src_id);                             } break;
        case L2CAP_COC_SPSM_REMOVE:            { l2cap_coc_spsm_remove_cmd_handler((l2cap_coc_spsm_remove_cmd_t*) p_cmd, src_id);                       } break;
        case L2CAP_COC_CREATE:                 { l2cap_coc_create_cmd_handler((l2cap_coc_create_cmd_t*) p_cmd, src_id);                                 } break;
        case L2CAP_COC_RECONFIGURE:            { l2cap_coc_reconfigure_cmd_handler((l2cap_coc_reconfigure_cmd_t*) p_cmd, src_id);                       } break;
        case L2CAP_COC_TERMINATE:              { l2cap_coc_terminate_cmd_handler((l2cap_coc_terminate_cmd_t*) p_cmd, src_id);                           } break;
        case L2CAP_SDU_SEND:                   { l2cap_sdu_send_cmd_handler((l2cap_sdu_send_cmd_t*) p_cmd, src_id);                                     } break;
        #if(RW_DEBUG)
        case L2CAP_DBG_CHAN_FIX_REGISTER:      { l2cap_dbg_chan_fix_register_cmd_handler((l2cap_dbg_chan_fix_register_cmd_t*) p_cmd, src_id);           } break;
        case L2CAP_DBG_CHAN_DYN_REGISTER:      { l2cap_dbg_chan_dyn_register_cmd_handler((l2cap_dbg_chan_dyn_register_cmd_t*) p_cmd, src_id);           } break;
        case L2CAP_DBG_CHAN_UNREGISTER:        { l2cap_dbg_chan_unregister_cmd_handler((l2cap_dbg_chan_unregister_cmd_t*) p_cmd, src_id);               } break;
        case L2CAP_DBG_CHAN_TX_CREDIT_ADD:     { l2cap_dbg_chan_tx_credit_add_cmd_handler((l2cap_dbg_chan_tx_credit_add_cmd_t*) p_cmd, src_id);         } break;
        case L2CAP_DBG_CHAN_CONFIG_SET:        { l2cap_dbg_chan_config_set_cmd_handler((l2cap_dbg_chan_config_set_cmd_t*) p_cmd, src_id);               } break;
        case L2CAP_DBG_CHAN_INFO_GET:          { l2cap_dbg_chan_info_get_handler((l2cap_dbg_chan_info_get_cmd_t*) p_cmd, src_id);                       } break;
        case L2CAP_DBG_COC_RECONFIGURE:        { l2cap_dbg_coc_reconfigure_cmd_handler((l2cap_dbg_coc_reconfigure_cmd_t*) p_cmd, src_id);               } break;
        case L2CAP_DBG_COC_ENHANCED_NEGO_CTRL: { l2cap_dbg_coc_enhanced_nego_ctrl_cmd_handler((l2cap_dbg_coc_enhanced_nego_ctrl_cmd_t*) p_cmd, src_id); } break;
        case L2CAP_DBG_RX_CTRL:                { l2cap_dbg_rx_ctrl_cmd_handler((l2cap_dbg_rx_ctrl_cmd_t*) p_cmd, src_id);                               } break;
        #endif //(RW_DEBUG)
        // default: Reject command
        default:                               { l2cap_msg_send_basic_cmp_evt(p_cmd, src_id, GAP_ERR_NOT_SUPPORTED);                                    } break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle reception of a L2CAP confirmation message
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int l2cap_cfm_msg_handler(ke_msg_id_t const msgid, l2cap_cfm_t *p_cfm, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    switch(p_cfm->req_ind_code)
    {
        case L2CAP_COC_CONNECT:  { l2cap_coc_connect_cfm_handler((l2cap_coc_connect_cfm_t*) p_cfm, src_id); } break;
        case L2CAP_SDU_RX:       { l2cap_sdu_rx_req_cfm_handler((l2cap_sdu_rx_cfm_t*) p_cfm, src_id);       } break;

        default: { /* Ignore message */ } break;
    }

    return (KE_MSG_CONSUMED);
}

#endif // (BLE_L2CAP && HOST_MSG_API)
/// @} L2CAP

