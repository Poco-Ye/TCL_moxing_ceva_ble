/**
 ****************************************************************************************
 *
 * @file llc_dl_upd.c
 *
 * @brief Handles the Data Length Extension procedure.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup LLC_OP
 * @{
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_CENTRAL || BLE_PERIPHERAL)
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "compiler.h"    // __ARRAY_EMPTY and __STATIC define
#include "co_bt.h"       // BT Standard defines (HCI, LLCP, Error codes)
#include "co_utils.h"    // For device pdu preparation
#include "co_math.h"     // For co_min

#include "ke_msg.h"      // Kernel message
#include "ke_timer.h"    // Kernel timers

#include "llc_int.h"     // Internal LLC API
#include "llc_llcp.h"    // Internal LLCP API

#include "lld.h"         // Internal LLD API

#include "hci.h"         // For HCI handler
#include "llm.h"         // For LE Mask Check
#include "sch_plan.h"    // Scheduling planner


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// DLE operation indication structure definition
/*@TRACE*/
struct llc_op_dl_upd_ind
{
    /// procedure information
    llc_procedure_t proc;
    ///Preferred maximum number of payload octets that the local Controller should include
    ///in a single Link Layer Data Channel PDU.
    uint16_t    tx_octets;
    ///Preferred maximum number of microseconds that the local Controller should use to transmit
    ///a single Link Layer Data Channel PDU
    uint16_t    tx_time;
    ///Preferred maximum number of payload octets that the local Controller should receive
    ///in a single Link Layer Data Channel PDU.
    uint16_t    rx_octets;
    ///Preferred maximum number of microseconds that the local Controller should use to receive
    ///a single Link Layer Data Channel PDU
    uint16_t    rx_time;
    /// Use to know id operation requested by host
    bool req_by_host;
};



/*
 * DEFINES
 ****************************************************************************************
 */
/*@TRACE*/
enum llc_dl_update_state
{
    //Local states
    /// Local start data length update procedure
    LLC_LOC_DL_UPD_PROC_START,
    /// Wait for length response
    LLC_LOC_DL_UPD_WAIT_PEER_DL_LENGTH_RSP,
};

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
/*
 * LOCAL FUNCTIONS DECLARATIONS
 ****************************************************************************************
 */
__STATIC void llc_ll_length_req_pdu_send(uint8_t link_id, uint16_t eff_tx_time, uint16_t eff_tx_octets, uint16_t eff_rx_time, uint16_t eff_rx_octets);
__STATIC void llc_ll_length_rsp_pdu_send(uint8_t link_id, uint16_t eff_tx_time, uint16_t eff_tx_octets, uint16_t eff_rx_time, uint16_t eff_rx_octets);
__STATIC void llc_hci_dl_upd_info_send(uint8_t link_id, uint8_t status, uint16_t max_tx_time, uint16_t max_tx_octets, uint16_t max_rx_time, uint16_t max_rx_octets);
__STATIC void llc_loc_dl_upd_proc_continue(uint8_t link_id, uint8_t status);
__STATIC void llc_rem_dl_upd_proc(uint8_t link_id, uint16_t max_tx_time, uint16_t max_tx_octets, uint16_t max_rx_time, uint16_t max_rx_octets);

__STATIC void llc_dle_proc_err_cb(uint8_t link_id, uint8_t error_type, void* param);

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Sends the length request pdu.
 *
 ****************************************************************************************
 */
__STATIC void llc_ll_length_req_pdu_send(uint8_t link_id, uint16_t eff_tx_time, uint16_t eff_tx_octets, uint16_t eff_rx_time, uint16_t eff_rx_octets)
{
    //Get environment pointer
    struct ll_length_rsp pdu;

    pdu.op_code         = LL_LENGTH_REQ_OPCODE;
    pdu.max_rx_octets   = eff_rx_octets;
    pdu.max_rx_time     = eff_rx_time;
    pdu.max_tx_octets   = eff_tx_octets;
    pdu.max_tx_time     = eff_tx_time;

    llc_llcp_send(link_id, (union llcp_pdu*) &pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Sends the length response pdu.
 *
 ****************************************************************************************
 */
__STATIC void llc_ll_length_rsp_pdu_send(uint8_t link_id, uint16_t eff_tx_time, uint16_t eff_tx_octets, uint16_t eff_rx_time, uint16_t eff_rx_octets)
{
    //Get environment pointer
    struct ll_length_rsp pdu;

    pdu.op_code         = LL_LENGTH_RSP_OPCODE;
    pdu.max_rx_octets   = eff_rx_octets;
    pdu.max_rx_time     = eff_rx_time;
    pdu.max_tx_octets   = eff_tx_octets;
    pdu.max_tx_time     = eff_tx_time;

    llc_llcp_send(link_id, (union llcp_pdu*) &pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Sends the data length effective values to the host.
 *
 * @param[in] link_id        Link identifier on which the pdu will be sent
 * @param[in] status         Status of the host request
 * @param[in] max_tx_time   The max time in transmission (unit of microsecond)
 * @param[in] max_tx_octets The max size in transmission (unit of byte)
 * @param[in] max_rx_time   The max time in reception (unit of microsecond)
 * @param[in] max_rx_octets The max size in reception (unit of byte)
 ****************************************************************************************
 */
__STATIC void llc_hci_dl_upd_info_send(uint8_t link_id, uint8_t status, uint16_t max_tx_time, uint16_t max_tx_octets, uint16_t max_rx_time, uint16_t max_rx_octets)
{
    uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);
    // allocate the event message
    struct hci_le_data_len_chg_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, conhdl, HCI_LE_META_EVT_CODE, hci_le_data_len_chg_evt);
    // fill event parameters
    event->subcode          = HCI_LE_DATA_LEN_CHG_EVT_SUBCODE;
    event->conhdl           = conhdl;
    event->max_rx_octets    = max_rx_octets;
    event->max_rx_time      = max_rx_time;
    event->max_tx_octets    = max_tx_octets;
    event->max_tx_time      = max_tx_time;

    // send the message
    hci_send_2_host(event);

    // If the data length is changed
    if(status == CO_ERROR_NO_ERROR)
    {
        // Update the planner element
        struct sch_plan_elt_tag *plan_elt = llm_plan_elt_get(link_id);

        // Unregister the old connection from bandwidth allocation system
        sch_plan_rem(plan_elt);

        // Update minimal duration
        plan_elt->duration_min = US_TO_HS(max_rx_time + BLE_IFS_DUR + max_tx_time);

        // Register the connection in bandwidth allocation system
        sch_plan_set(plan_elt);
    }
}

/**
 ****************************************************************************************
 * Continue execution of local procedure
 *
 * @param[in] link_id Link identifier
 * @param[in] status  Status of the operation
 ****************************************************************************************
 */
__STATIC void llc_loc_dl_upd_proc_continue(uint8_t link_id, uint8_t status)
{
    bool finished = true;
    bool len_updated = false;
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Gets the values requested by the peer
    struct llc_op_dl_upd_ind *param = (struct llc_op_dl_upd_ind *)llc_proc_get(link_id, LLC_PROC_LOCAL);

    switch(llc_proc_state_get(&param->proc))
    {
        case LLC_LOC_DL_UPD_PROC_START:
        /*
         * @startuml
         * title : Data length update procedure start (DL_UPD_PROC_START)
         * participant LLC
         * participant LLD
         * LLC -> LLC: llc_loc_dl_upd_proc_continue(START)
         * hnote over LLC #aqua: state = WAIT_PEER_DL_LENGTH_RSP
         * activate LLC
         *     LLC --> LLD : LLCP_LENGTH_REQ
         * deactivate LLC
         * note over LLC: Start LLCP timeout
         * @enduml
         */
        {
            llc_proc_state_set(&param->proc, link_id, LLC_LOC_DL_UPD_WAIT_PEER_DL_LENGTH_RSP);

            // If the coded PHY is not in use
            if((llc_env_ptr->con_params.tx_phy != PHY_CODED_VALUE) && (llc_env_ptr->con_params.rx_phy != PHY_CODED_VALUE))
            {
                // If the peer device has not indicated support for the LE Coded PHY feature
                if(   (!GETB(llc_env_ptr->link_info, LLC_INFO_PEER_FEATURE_RECV))
                   || (!llc_le_feature_check(link_id, BLE_FEAT_CODED_PHY)) )
                {
                    param->tx_time = co_min(param->tx_time, LE_MAX_TIME);
                    param->rx_time = co_min(param->rx_time, LE_MAX_TIME);
                }
            }
            else // The coded PHY is in use
            {
                // If TX PHY is coded, transmission time must be above 2704us (LL:4.5.10)
                if(llc_env_ptr->con_params.tx_phy == PHY_CODED_VALUE)
                {
                    param->tx_time = co_max(param->tx_time, LE_MIN_TIME_CODED);
                }

                // If RX PHY is coded, reception time must be above 2704us (LL:4.5.10)
                if(llc_env_ptr->con_params.rx_phy == PHY_CODED_VALUE)
                {
                    param->rx_time = co_max(param->rx_time, LE_MIN_TIME_CODED);
                }
            }

            // Send length request to the peer
            llc_ll_length_req_pdu_send(link_id, param->tx_time, param->tx_octets, param->rx_time, param->rx_octets);

            // Start the LLCP Response TO
            llc_proc_timer_set(link_id, LLC_PROC_LOCAL, true);

            finished = false;
        }
        break;


        case LLC_LOC_DL_UPD_WAIT_PEER_DL_LENGTH_RSP:
            /*
             * @startuml
             * title : Data length update procedure length response received (DL_UPD_WAIT_PEER_DL_LENGTH_RSP)
             * participant HCI
             * participant LLC
             * participant LLD
             * LLC <- :llcp_data_length_rsp_handler
             * activate LLC
             * note over LLC: Merge local and peer parameters
             * deactivate LLC
             * LLC -> LLC: llc_loc_dl_upd_proc_continue(LENGTH_RSP)
             * activate LLC
             *     note over LLC: Stop LLCP timeout
             *     LLC -> LLD : lld_con_data_len_update
             *     alt host request or at least on value has changed
             *         LLC --> HCI : HCI_LE_DATA_LENGTH_CHANGE_EVT
             *     end
             *     hnote over LLC : LOC_PROC = Idle
             * deactivate LLC
             * @enduml
             */
        {
            // We got the response for the version indication procedure we initiated
            llc_proc_timer_set(link_id, LLC_PROC_LOCAL, false);

            if(status == CO_ERROR_NO_ERROR)
            {
                // If TX PHY is coded, transmission time must be above 2704us (LL:4.5.10)
                if(llc_env_ptr->con_params.tx_phy == PHY_CODED_VALUE)
                {
                    param->tx_time = co_max(param->tx_time, LE_MIN_TIME_CODED);
                }

                // If RX PHY is coded, reception time must be above 2704us (LL:4.5.10)
                if(llc_env_ptr->con_params.rx_phy == PHY_CODED_VALUE)
                {
                    param->rx_time = co_max(param->rx_time, LE_MIN_TIME_CODED);
                }

                if(   (llc_env_ptr->con_params.max_rx_octets != param->rx_octets)
                   || (llc_env_ptr->con_params.max_tx_octets != param->tx_octets)
                   || (llc_env_ptr->con_params.max_rx_time   != param->rx_time)
                   || (llc_env_ptr->con_params.max_tx_time   != param->tx_time))
                {
                    status = lld_con_data_len_update(link_id, param->tx_time, param->tx_octets, param->rx_time, param->rx_octets);
                    len_updated = (status == CO_ERROR_NO_ERROR) ? true : false;
                }
            }
            finished = true;
        }
        break;

        default:
        {
            ASSERT_INFO(0, link_id, llc_proc_state_get(&param->proc));
        }
        break;
    }

    if(finished)
    {
        if(len_updated)
        {
            // update connection parameters
            llc_env_ptr->con_params.max_rx_octets = param->rx_octets;
            llc_env_ptr->con_params.max_tx_octets = param->tx_octets;
            llc_env_ptr->con_params.max_rx_time   = param->rx_time;
            llc_env_ptr->con_params.max_tx_time   = param->tx_time;

            // Send the data length change event
            llc_hci_dl_upd_info_send(link_id, status, param->tx_time, param->tx_octets, param->rx_time, param->rx_octets);
        }

        if(param->req_by_host)
        {
            SETB(llc_env_ptr->flow_ctrl, LLC_HCI_SET_DATA_LEN_REQ, false);
        }

        // unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_LOCAL);
    }
}

/**
 ****************************************************************************************
 * Execute remote procedure
 *
 * @startuml
 * title : Data length update procedure length request received (DL_UPD_PROC_START)
 * participant HCI
 * participant LLC
 * participant LLD
 * LLC <- :llcp_data_length_req_handler
 * activate LLC
 * note over LLC: Merge local and peer parameters
 * deactivate LLC
 * LLC -> LLC: llc_loc_dl_upd_proc_continue(LENGTH_REQ)
 * activate LLC
 *     LLC --> LLD : LLCP_LENGTH_RSP
 *     LLC -> LLD : lld_con_data_len_update
 *     alt host request or at least on value has changed
 *         LLC --> HCI : HCI_LE_DATA_LENGTH_CHANGE_EVT
 *     end
 *     hnote over LLC : LOC_PROC = Idle
 * deactivate LLC
 * @enduml
 *
 * @param[in] link_id       Link identifier
 * @param[in] max_tx_time   The peer max time in transmission (unit of microsecond)
 * @param[in] max_tx_octets The peer max size in transmission (unit of byte)
 * @param[in] max_rx_time   The peer max time in reception (unit of microsecond)
 * @param[in] max_rx_octets The peer max size in reception (unit of byte)
 ****************************************************************************************
 */
__STATIC void llc_rem_dl_upd_proc(uint8_t link_id, uint16_t max_tx_time, uint16_t max_tx_octets, uint16_t max_rx_time, uint16_t max_rx_octets)
{
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Compute local parameters according to host preferences and peer
    uint16_t tx_octets   = co_min(llc_env_ptr->suggested_max_tx_octets, max_rx_octets);
    uint16_t tx_time     = co_min(llc_env_ptr->suggested_max_tx_time,   max_rx_time);
    // RX Buffer size are max according to Host preferences
    uint16_t rx_octets   = co_min(llc_env_ptr->suggested_max_rx_octets, max_tx_octets);
    uint16_t rx_time     = co_min(llc_env_ptr->suggested_max_rx_time,   max_tx_time);

    // If the coded PHY is not in use
    if((llc_env_ptr->con_params.tx_phy != PHY_CODED_VALUE) && (llc_env_ptr->con_params.rx_phy != PHY_CODED_VALUE))
    {
        // If the peer device has not indicated support for the LE Coded PHY feature
        if(   (!GETB(llc_env_ptr->link_info, LLC_INFO_PEER_FEATURE_RECV))
           || (!llc_le_feature_check(link_id, BLE_FEAT_CODED_PHY)) )
        {
            tx_time = co_min(tx_time, LE_MAX_TIME);
            rx_time = co_min(rx_time, LE_MAX_TIME);
        }
    }
    else // The coded PHY is in use
    {
        // If TX PHY is coded, transmission time must be above 2704us (LL:4.5.10)
        if(llc_env_ptr->con_params.tx_phy == PHY_CODED_VALUE)
        {
            tx_time = co_max(tx_time, LE_MIN_TIME_CODED);
        }

        // If RX PHY is coded, reception time must be above 2704us (LL:4.5.10)
        if(llc_env_ptr->con_params.rx_phy == PHY_CODED_VALUE)
        {
            rx_time = co_max(rx_time, LE_MIN_TIME_CODED);
        }
    }

    if(   (llc_env_ptr->con_params.max_rx_octets != rx_octets)
       || (llc_env_ptr->con_params.max_tx_octets != tx_octets)
       || (llc_env_ptr->con_params.max_rx_time   != rx_time)
       || (llc_env_ptr->con_params.max_tx_time   != tx_time))
    {
        // update connection parameters
        llc_env_ptr->con_params.max_rx_octets = rx_octets;
        llc_env_ptr->con_params.max_tx_octets = tx_octets;
        llc_env_ptr->con_params.max_rx_time   = rx_time;
        llc_env_ptr->con_params.max_tx_time   = tx_time;

        // Update the driver
        lld_con_data_len_update(link_id, tx_time, tx_octets, rx_time, rx_octets);

        // Send an event to inform that parameters have changed
        llc_hci_dl_upd_info_send(link_id, CO_ERROR_NO_ERROR, tx_time, tx_octets, rx_time, rx_octets);
    }

    //Send response to the peer
    llc_ll_length_rsp_pdu_send(link_id, tx_time, tx_octets, rx_time, rx_octets);
}

/**
 * Local procedure callback used to inform if an unexpected error is raised during procedure execution
 *
 * @param[in] link_id     Link Identifier
 * @param[in] error_type  Error type (@see enum llc_error_type)
 * @param[in] param       Parameter according to error type:
 *   - LLC_ERR_DISCONNECT:          reason
 *   - LLC_ERR_LLCP_UNKNOWN_RSP:    struct ll_unknown_rsp*
 *   - LLC_ERR_LLCP_REJECT_IND:     struct ll_reject_ind*
 *   - LLC_ERR_LLCP_REJECT_IND_EXT: struct ll_reject_ext_ind*
 */
__STATIC void llc_dle_proc_err_cb(uint8_t link_id, uint8_t error_type, void* param)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    switch (error_type)
    {
        case LLC_ERR_DISCONNECT:
        {
            status = *((uint8_t*) param);
        }break;
        case LLC_ERR_LLCP_UNKNOWN_RSP:
        {
            struct ll_unknown_rsp* rsp = (struct ll_unknown_rsp*) param;
            if(rsp->unk_type == LL_LENGTH_REQ_OPCODE)
            {
                status = CO_ERROR_UNKNOWN_LMP_PDU;
            }
        }break;
        case LLC_ERR_LLCP_REJECT_IND:
        {
            struct ll_reject_ind* reject = (struct ll_reject_ind*) param;
            status =  reject->err_code;
        }break;
        case LLC_ERR_LLCP_REJECT_IND_EXT:
        {
            struct ll_reject_ext_ind* reject_ext = (struct ll_reject_ext_ind*) param;
            if(reject_ext->rej_op_code == LL_LENGTH_REQ_OPCODE)
            {
                status =  reject_ext->err_code;
            }
        }break;
        default: /* Nothing to do, ignore */ break;
    }

    if(status != CO_ERROR_NO_ERROR)
    {
        if((status == CO_ERROR_UNKNOWN_LMP_PDU) || (status == CO_ERROR_UNSUPPORTED_REMOTE_FEATURE))
        {
                llc_le_feature_set(link_id, BLE_FEAT_DATA_PKT_LEN_EXT, false);
        }

        llc_loc_dl_upd_proc_continue(link_id, status);
    }
}


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP length request
 *
 * @param[in] link_id        Link identifier on which the pdu will be sent.
 * @param[in] pdu            LLCP PDU information received
 * @param[in] event_cnt      Event counter value when PDU has been received
 *
 * @return status code of handler:
 *    - CO_ERROR_NO_ERROR:               Nothing more to do
 *    - CO_ERROR_TERMINATED_MIC_FAILURE: Immediately disconnect the link
 *    - others:                          Send an LLCP_REJECT_IND or LLCP_REJECT_IND_EXT
 ****************************************************************************************
 */
int ROM_VT_FUNC(ll_length_req_handler)(uint8_t link_id, struct ll_length_req *pdu, uint16_t event_cnt)
{
    int status = CO_ERROR_NO_ERROR;

    // check that no remote procedure is on-going
    if (llc_proc_id_get(link_id, LLC_PROC_REMOTE) != LLC_PROC_NONE)
    {
        // unexpected message, reject it.
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }
    //If values not in minimal range
    else if(   (pdu->max_rx_octets < LE_MIN_OCTETS)     || (pdu->max_tx_octets < LE_MIN_OCTETS)
            || (pdu->max_rx_time   < LE_MIN_TIME)       || (pdu->max_tx_time   < LE_MIN_TIME)
            || (pdu->max_rx_octets > LE_MAX_OCTETS)     || (pdu->max_tx_octets > LE_MAX_OCTETS)
            || (pdu->max_rx_time   > LE_MAX_TIME_CODED) || (pdu->max_tx_time   > LE_MAX_TIME_CODED))
    {
        //Return an error to send a ll_reject_ind
        status = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
    }
    else
    {
        llc_rem_dl_upd_proc(link_id, pdu->max_tx_time, pdu->max_tx_octets, pdu->max_rx_time, pdu->max_rx_octets);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP length response
 *
 * @param[in] link_id        Link identifier on which the pdu will be sent.
 * @param[in] pdu            LLCP PDU information received
 * @param[in] event_cnt      Event counter value when PDU has been received
 *
 * @return status code of handler:
 *    - CO_ERROR_NO_ERROR:               Nothing more to do
 *    - CO_ERROR_TERMINATED_MIC_FAILURE: Immediately disconnect the link
 *    - others:                          Send an LLCP_REJECT_IND or LLCP_REJECT_IND_EXT
 ****************************************************************************************
 */
int ROM_VT_FUNC(ll_length_rsp_handler)(uint8_t link_id, struct ll_length_rsp *pdu, uint16_t event_cnt)
{
    int status = CO_ERROR_NO_ERROR;

    // check that local procedure is the correct one
    if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) != LLC_PROC_DL_UPDATE)
    {
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }
    //If values not in minimal range
    else if(   (pdu->max_rx_octets < LE_MIN_OCTETS) || (pdu->max_tx_octets < LE_MIN_OCTETS)
            || (pdu->max_rx_time   < LE_MIN_TIME)   || (pdu->max_tx_time   < LE_MIN_TIME))
    {
        //Return an error to send a ll_reject_ind
        status = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
    }
    else
    {
        // Gets the values requested by the peer
        struct llc_op_dl_upd_ind *dl_loc_req = (struct llc_op_dl_upd_ind *)llc_proc_get(link_id, LLC_PROC_LOCAL);

        // Merge Host and Peer values
        dl_loc_req->tx_octets = co_min(dl_loc_req->tx_octets,pdu->max_rx_octets);
        dl_loc_req->tx_time   = co_min(dl_loc_req->tx_time,  pdu->max_rx_time);
        dl_loc_req->rx_octets = co_min(dl_loc_req->rx_octets,pdu->max_tx_octets);
        dl_loc_req->rx_time   = co_min(dl_loc_req->rx_time,  pdu->max_tx_time);

    }
    if(status != CO_ERROR_LMP_PDU_NOT_ALLOWED)
    {
        // execute feature exchange procedure
        llc_loc_dl_upd_proc_continue(link_id, status);
    }

    return status;
}


/**
 ****************************************************************************************
 * @brief Handles the command HCI set data length command.
 *
 * PlantUML procedure description
 *
 * @startuml
 * title : Set data length procedure initiated by host
 * participant HCI
 * participant LLC
 * HCI --> LLC : HCI_LE_SET_DATA_LENGTH_CMD
 * LLC -> LLC: hci_le_set_data_length_cmd_handler()
 * activate LLC
 * alt link disconnected \nor slave \nor peer not support encryption
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(DISALLOWED)
 * else  Procedure already started but negative answer from peer
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(UNSUPPORTED_REMOTE_FEATURE)
 * else  Parameters out of range
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(INVALID_HCI_PARAMS)
 * else  Procedure already started
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(BUSY)
 * else  Procedure can be started
 *     LLC --> LLC : LLC_OP_DL_UPD_IND
 *     note right LLC #lightgreen: See __llc_op_dl_upd_ind_handler()__
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(OK)
 * end
 * deactivate LLC
 * @enduml
 *
 * @param[in] link_id    Link Identifier
 * @param[in] param      Pointer to the parameters of the message.
 * @param[in] opcode     HCI message operation code.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int ROM_VT_FUNC(hci_le_set_data_len_cmd_handler)(uint8_t link_id, struct hci_le_set_data_len_cmd const *param, uint16_t opcode)
{
    // Command status
    uint8_t status = CO_ERROR_NO_ERROR;
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // check if state is Free or in disconnected state
    if(llc_is_disconnecting(link_id))
    {
        status = CO_ERROR_COMMAND_DISALLOWED;
    }
    // ensure that host not currently processing request
    else if(GETB(llc_env_ptr->flow_ctrl, LLC_HCI_SET_DATA_LEN_REQ))
    {
        status = CO_ERROR_CONTROLLER_BUSY;
    }
    // check if peer support feature
    else if(!llc_le_feature_check(link_id, BLE_FEAT_DATA_PKT_LEN_EXT))
    {
        status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
    }
    // check parameter range
    else if(   (param->tx_octets < LE_MIN_OCTETS) || (param->tx_octets > LE_MAX_OCTETS)
            || (param->tx_time   < LE_MIN_TIME)   || (param->tx_time   > LE_MAX_TIME_CODED))
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        // start DLE negotiation only if host change Max preferred values
        if((param->tx_octets != llc_env_ptr->con_params.max_tx_octets) || (param->tx_time!= llc_env_ptr->con_params.max_tx_time))
        {
            ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
            struct llc_op_dl_upd_ind * dl_upd = KE_MSG_ALLOC(LLC_OP_DL_UPD_IND, llc_id, llc_id, llc_op_dl_upd_ind);

            llc_proc_init(&dl_upd->proc, LLC_PROC_DL_UPDATE, llc_dle_proc_err_cb);
            llc_proc_state_set(&dl_upd->proc, link_id, LLC_LOC_DL_UPD_PROC_START);
            dl_upd->tx_octets     = param->tx_octets;
            dl_upd->tx_time       = param->tx_time;
            // RX Buffer size are max according to supported buffer and rate
            dl_upd->rx_octets     = BLE_MAX_OCTETS;
            dl_upd->rx_time       = BLE_MAX_TIME;
            dl_upd->req_by_host   = true;

            ke_msg_send(dl_upd);

            SETB(llc_env_ptr->flow_ctrl, LLC_HCI_SET_DATA_LEN_REQ, true);
        }
        // store new suggested values
        llc_env_ptr->suggested_max_tx_octets = param->tx_octets;
        llc_env_ptr->suggested_max_tx_time   = param->tx_time;
    }

    // Send the command complete event
    llc_cmd_cmp_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}


extern int ROM_VT_FUNC(hci_vs_set_max_rx_size_and_time_cmd_handler)(uint8_t link_id, struct hci_vs_set_max_rx_size_and_time_cmd const *param, uint16_t opcode)
{
    // Command status
    uint8_t status = CO_ERROR_NO_ERROR;
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // check if state is Free or in disconnected state or if we expect the standard version
    if(llc_is_disconnecting(link_id))
    {
        status = CO_ERROR_COMMAND_DISALLOWED;
    }
    // ensure that host not currently processing request
    else if(GETB(llc_env_ptr->flow_ctrl, LLC_HCI_SET_DATA_LEN_REQ))
    {
        status = CO_ERROR_CONTROLLER_BUSY;
    }
    // check if peer support feature
    else if(!llc_le_feature_check(link_id, BLE_FEAT_DATA_PKT_LEN_EXT))
    {
        status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
    }
    // check parameter range
    else if(   (param->rx_octets < LE_MIN_OCTETS) || (param->rx_octets > LE_MAX_OCTETS)
            || (param->rx_time   < LE_MIN_TIME)   || (param->rx_time   > LE_MAX_TIME_CODED))
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        // start DLE negotiation only if host change Max preferred values
        if((param->rx_octets != llc_env_ptr->con_params.max_rx_octets) || (param->rx_time!= llc_env_ptr->con_params.max_rx_time))
        {
            ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
            struct llc_op_dl_upd_ind * dl_upd = KE_MSG_ALLOC(LLC_OP_DL_UPD_IND, llc_id, llc_id, llc_op_dl_upd_ind);

            llc_proc_init(&dl_upd->proc, LLC_PROC_DL_UPDATE, llc_dle_proc_err_cb);
            llc_proc_state_set(&dl_upd->proc, link_id, LLC_LOC_DL_UPD_PROC_START);
            dl_upd->tx_octets     = llc_env_ptr->con_params.max_tx_octets;
            dl_upd->tx_time       = llc_env_ptr->con_params.max_tx_time;
            dl_upd->rx_octets     = param->rx_octets;
            dl_upd->rx_time       = param->rx_time;
            dl_upd->req_by_host   = true;

            ke_msg_send(dl_upd);

            SETB(llc_env_ptr->flow_ctrl, LLC_HCI_SET_DATA_LEN_REQ, true);
        }
        // store new suggested values
        llc_env_ptr->suggested_max_rx_octets = param->rx_octets;
        llc_env_ptr->suggested_max_rx_time   = param->rx_time;
    }

    // Send the command complete event
    llc_cmd_cmp_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the Data length update procedure indication message.
 *
 * PlantUML procedure description
 *
 * @startuml
 * title : Data length procedure start
 * participant LLC
 *  --> LLC : LLC_OP_DL_UPD_IND
 * LLC -> LLC: llc_op_dl_upd_ind_handler()
 * activate LLC
 * hnote over LLC : LOC_PROC = Busy
 * LLC -> LLC: llc_loc_dl_up_proc_continue(START)
 * activate LLC
 * note right LLC #lightgreen: See llc_loc_dl_up_proc_continue()__
 * deactivate LLC
 * deactivate LLC
 * @enduml
 *
 ****************************************************************************************
 */
int ROM_VT_FUNC(llc_op_dl_upd_ind_handler)(ke_msg_id_t const msgid, struct llc_op_dl_upd_ind *param,
                                ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current message status
    int msg_status = KE_MSG_CONSUMED;
    uint8_t link_id = KE_IDX_GET(dest_id);

    // Check if state in disconnected state
    if(llc_is_disconnecting(link_id))
    {
        // disconnection on-going inform that command is aborted
        if(param->req_by_host)
        {
            //Get environment pointer
            struct llc_env_tag *llc_env_ptr = llc_env[link_id];
            llc_hci_dl_upd_info_send(link_id, llc_env_ptr->disc_reason, 0, 0, 0, 0);
        }
    }
    // check if another local procedure is on-going
    else if(llc_proc_id_get(link_id, LLC_PROC_LOCAL) != LLC_PROC_NONE)
    {
        // process this message later
        msg_status = KE_MSG_SAVED;
    }
    else
    {
        msg_status = KE_MSG_NO_FREE;

        // store local procedure
        llc_proc_reg(link_id, LLC_PROC_LOCAL, &(param->proc));

        // execute feature exchange procedure
        llc_loc_dl_upd_proc_continue(link_id, CO_ERROR_NO_ERROR);
    }
    return (msg_status);
}

/**
 ****************************************************************************************
 * Start data length update procedure
 *
 * @param[in] link_id Link identifier
 ****************************************************************************************
 */
void ROM_VT_FUNC(dl_upd_proc_start)(uint8_t link_id)
{
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // start DLE negotiation only if host change Max preferred values
    if((llc_env_ptr->suggested_max_tx_octets != llc_env_ptr->con_params.max_tx_octets) || (llc_env_ptr->suggested_max_tx_time != llc_env_ptr->con_params.max_tx_time)
    )
    {
        ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
        struct llc_op_dl_upd_ind * dl_upd = KE_MSG_ALLOC(LLC_OP_DL_UPD_IND, llc_id, llc_id, llc_op_dl_upd_ind);

        llc_proc_init(&dl_upd->proc, LLC_PROC_DL_UPDATE, llc_dle_proc_err_cb);
        llc_proc_state_set(&dl_upd->proc, link_id, LLC_LOC_DL_UPD_PROC_START);
        dl_upd->tx_octets     = llc_env_ptr->suggested_max_tx_octets;
        dl_upd->tx_time       = llc_env_ptr->suggested_max_tx_time;
        // RX Buffer size are max according to supported buffer and rate
        dl_upd->rx_octets     = BLE_MAX_OCTETS;
        dl_upd->rx_time       = BLE_MAX_TIME;
        dl_upd->req_by_host   = false;

        ke_msg_send(dl_upd);
    }
}

#endif // (BLE_CENTRAL || BLE_PERIPHERAL)
/// @} llc_dle_upd
