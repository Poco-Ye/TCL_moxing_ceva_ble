/**
 ****************************************************************************************
 * @file gatt_srv_event.c
 *
 * @brief  GATT Client Read Procedure
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
#include "rwip_config.h"       // IP configuration
#if (BLE_GATT)
#include "gatt.h"              // Native API
#include "gatt_user.h"         // User API
#include "gatt_proc.h"         // Procedure API
#include "gatt_db.h"           // ATT Database access
#include "gatt_int.h"          // Internals

#include "co_endian.h"         // endianess support
#include "co_math.h"           // Mathematics
#include <string.h>            // for memcmp and memcpy

#include "../inc/gap_hl_api.h" // to get if client is out of sync
#include "gapc.h"              // to retrieve connection handle (and check if connection exists)

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// Server event procedure state bit field
enum gatt_srv_event_state_bf
{
    /// check if procedure has been canceled
    GATT_SRV_EVENT_CANCELED_BIT             = 0x01,
    GATT_SRV_EVENT_CANCELED_POS             = 0,
    /// Buffer confirmation received
    GATT_SRV_EVENT_CFM_RECEIVED_BIT         = 0x02,
    GATT_SRV_EVENT_CFM_RECEIVED_POS         = 1,
    /// in case of MTP event, if filter enabled, do not trigger complete event for each connection but only
    /// if all events have been triggered
    GATT_SRV_EVENT_MTP_CMP_EVT_FILTERED_BIT = 0x04,
    GATT_SRV_EVENT_MTP_CMP_EVT_FILTERED_POS = 2,
};

/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Server event trigger procedure structure
typedef struct gatt_srv_event_proc
{
    /// Procedure header - required for any Attribute procedure
    gatt_proc_t             hdr;
    /// Buffer queue
    co_list_t               buf_queue;
    /// Connection index bit field (use to send event over multiple connection)
    /// if 0 event is send on one connection only
    uint32_t                conidx_bf;
    /// Packet remain length
    uint16_t                remain_len;
    /// Total event data length
    uint16_t                total_len;
    #if (HOST_MSG_API)
    /// Message command code used.
    uint16_t                msg_cmd_code;
    #endif // (HOST_MSG_API)
    /// Attribute cursor
    uint8_t                 cursor;
    /// Number of attributes
    uint8_t                 nb_att;
    /// Procedure state bit field (see enum #gatt_srv_event_state_bf)
    uint8_t                 state_bf;
    /// List of attribute information into the event
    gatt_att_t              atts[__ARRAY_EMPTY];
} gatt_srv_event_proc_t;

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
 * @brief Function called when L2CAP_ATT_HDL_VAL_CFM attribute PDU is received.
 *
 * @param[in] conidx     Connection index
 * @param[in] p_proc     Pointer to procedure under execution
 * @param[in] p_pdu      Pointer to unpacked PDU
 * @param[in] p_buf      Data of received but not yet extracted
 * @param[in] mtu        MTU size of the bearer
 *
 * @return PDU handling return status that will be provided to the procedure handler.
 ****************************************************************************************
 */
__STATIC uint16_t gatt_cli_l2cap_att_hdl_val_cfm_handler(uint8_t conidx, gatt_srv_event_proc_t* p_proc,
                                                    l2cap_att_hdl_val_cfm_t* p_pdu,
                                                    co_buf_t* p_buf, uint16_t mtu)
{
    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, (gatt_proc_t*) p_proc);

    return (GAP_ERR_NO_ERROR);
}

/// Call event sent callback
__STATIC void gatt_srv_event_call_sent_cb(uint8_t conidx, gatt_srv_event_proc_t* p_proc, uint16_t status)
{
    gatt_user_t* p_user  = gatt_user_get(p_proc->hdr.user_lid);

    #if (HOST_MSG_API)
    gatt_proc_cur_set((gatt_proc_t*) p_proc);
    #endif // (HOST_MSG_API)

    // Inform user that procedure is over
    p_user->p_cb->srv.cb_event_sent(conidx, p_proc->hdr.user_lid, p_proc->hdr.dummy, status);
}


/// Send Notification or indication to peer device
__STATIC uint16_t gatt_srv_event_ntf_ind_send(uint8_t conidx, gatt_srv_event_proc_t* p_proc, co_buf_t* p_buf)
{
    l2cap_att_pdu_t pdu;
    uint8_t proc_id = p_proc->hdr.proc_id;

    pdu.code = (proc_id == GATT_PROC_NOTIFY) ? L2CAP_ATT_HDL_VAL_NTF_OPCODE : L2CAP_ATT_HDL_VAL_IND_OPCODE;
    pdu.hdl_val_ntf.handle = p_proc->atts[0].hdl;

    // Ask for PDU transmission
    return gatt_proc_pdu_send(conidx, (gatt_proc_t*) p_proc, &pdu, p_buf, (proc_id == GATT_PROC_NOTIFY)
                              ? NULL
                              : (gatt_proc_pdu_handler_cb) gatt_cli_l2cap_att_hdl_val_cfm_handler);
}


/// Send Multiple Notification peer device
__STATIC uint16_t gatt_srv_event_mult_ntf_send(uint8_t conidx, gatt_srv_event_proc_t* p_proc, co_buf_t* p_buf)
{
    uint16_t status;

    do
    {
        l2cap_att_pdu_t pdu;
        uint16_t tail_len;

        pdu.code = L2CAP_ATT_MULT_HDL_VAL_NTF_OPCODE;
        tail_len = p_proc->total_len - co_buf_data_len(p_buf) + GATT_BUFFER_TAIL_LEN;

        // if buffer cannot contain full response, allocate a new buffer.
        if(tail_len > co_buf_tail_len(p_buf))
        {
            co_buf_t* p_out_buf;

            if(co_buf_duplicate(p_buf, &p_out_buf, GATT_BUFFER_HEADER_LEN, tail_len) != CO_BUF_ERR_NO_ERROR)
            {
                status = ATT_ERR_INSUFF_RESOURCE;
                break;
            }
            else
            {
                co_buf_release(p_buf);
                p_buf = p_out_buf;
            }
        }

        // copy remaining data onto the PDU
        while(!co_list_is_empty(&(p_proc->buf_queue)))
        {
            co_buf_t* p_in_buf = (co_buf_t*) co_list_pop_front(&(p_proc->buf_queue));
            uint16_t  data_len = co_buf_data_len(p_in_buf);

            co_buf_copy_data_to_mem(p_in_buf, co_buf_tail(p_buf), data_len);
            co_buf_tail_reserve(p_buf, data_len);

            co_buf_release(p_in_buf);
        }

        // Ask for PDU transmission
        status = gatt_proc_pdu_send(conidx, (gatt_proc_t*) p_proc, &pdu, p_buf, NULL);
    } while(0);

    return (status);
}


/// Send Notification or indication to peer device from procedure start
__STATIC uint16_t gatt_srv_event_ntf_ind_send_from_proc_start(uint8_t conidx, gatt_srv_event_proc_t* p_proc, co_buf_t* p_buf)
{
    p_proc->remain_len = gatt_proc_mtu_get(conidx, (gatt_proc_t*) p_proc)- L2CAP_ATT_HEADER_LEN - GATT_HANDLE_LEN;
    // ensure that event data does not exceed MTU length
    if(p_proc->total_len > p_proc->remain_len)
    {
        p_proc->remain_len = p_proc->total_len - p_proc->remain_len;
        co_buf_tail_release(p_buf, p_proc->remain_len);
    }
    // no need to crop buffer
    else
    {
        p_proc->remain_len = 0;
    }

    return gatt_srv_event_ntf_ind_send(conidx, p_proc, p_buf);
}

/// Terminate procedure
__STATIC void gatt_srv_event_proc_terminate(uint8_t conidx, gatt_srv_event_proc_t* p_proc, uint16_t status)
{
    // perform a buffer release
    while(!co_list_is_empty(&(p_proc->buf_queue)))
    {
        co_buf_t* p_buf = (co_buf_t*) co_list_pop_front(&(p_proc->buf_queue));
        co_buf_release(p_buf);
    }

    // Pop Procedure
    gatt_proc_pop(conidx, (gatt_proc_t*) p_proc, true);

    // Inform user that procedure is over
    gatt_srv_event_call_sent_cb(conidx, p_proc, status);
}

/// Send on event (Simple indication or notification)
__STATIC void gatt_srv_event_simple_proc_continue(uint8_t conidx, gatt_srv_event_proc_t* p_proc, uint8_t proc_state, uint16_t status)
{
    bool finished = false;

    switch(proc_state)
    {
        case GATT_PROC_START:
        {
            // Send PDU present as fist buffer in queue
            co_buf_t* p_buf = (co_buf_t*) co_list_pop_front(&(p_proc->buf_queue));
            ASSERT_ERR(p_buf != NULL);
            status = gatt_srv_event_ntf_ind_send_from_proc_start(conidx, p_proc,p_buf);
            co_buf_release(p_buf);
        } break;
        case GATT_PROC_PDU_PUSHED_TO_LL:
        {
            // procedure is over here for a notification
            if(p_proc->hdr.proc_id != GATT_PROC_NOTIFY) break;
        }
        // no break;
        case GATT_PROC_PDU_RX:
        case GATT_PROC_ERROR: // Handle error detection
        {
            finished = true;
        } break;

        default: { /*  Nothing to do */  } break;
    }

    if((finished) || (status != GAP_ERR_NO_ERROR))
    {
        // Terminate procedure
        gatt_srv_event_proc_terminate(conidx, p_proc, status);
    }
}


/// Multi point (connection) event transmission
__STATIC void gatt_srv_event_mtp_proc_continue(uint8_t conidx, gatt_srv_event_proc_t* p_proc, uint8_t proc_state, uint16_t status)
{
    bool p2p_finished = false;
    uint8_t proc_id = p_proc->hdr.proc_id;

    switch(proc_state)
    {
        case GATT_PROC_START:
        {
            // Send PDU present as fist buffer in queue
            co_buf_t* p_buf = (co_buf_t*) co_list_pick(&(p_proc->buf_queue));
            ASSERT_ERR(p_buf != NULL);
            status = gatt_srv_event_ntf_ind_send_from_proc_start(conidx, p_proc,p_buf);
        } break;
        case GATT_PROC_PDU_PUSHED_TO_LL:
        {
            // PDU sent still present in queue
            co_buf_t* p_buf = (co_buf_t*) co_list_pick(&(p_proc->buf_queue));
            co_buf_tail_reserve(p_buf, p_proc->remain_len);

            // procedure is over here for a notification
            if(proc_id != GATT_PROC_NOTIFY) break;
        }
        // no break;
        case GATT_PROC_PDU_RX:
        case GATT_PROC_ERROR: // Handle error detection
        {
            p2p_finished = true;
        } break;

        default: { /*  Nothing to do */  } break;
    }

    if(p2p_finished || (status != GAP_ERR_NO_ERROR))
    {
        bool proc_finished;

        // Pop Procedure
        gatt_proc_pop(conidx, (gatt_proc_t*) p_proc, false);

        do
        {
            p_proc->conidx_bf &= ~CO_BIT(conidx);
            proc_finished = ((p_proc->conidx_bf == 0) || GETB(p_proc->state_bf, GATT_SRV_EVENT_CANCELED));

            if(!GETB(p_proc->state_bf, GATT_SRV_EVENT_MTP_CMP_EVT_FILTERED))
            {
                // Inform user that event has been sent onto a specific connection index
                gatt_srv_event_call_sent_cb(conidx, p_proc, status);
            }

            if(proc_finished) break;

            // retrieve next connection index
            conidx = co_ctz(p_proc->conidx_bf);
            // if no more exists mark it disconnected
            if(gatt_env.p_con[conidx] == NULL)
            {
                status = GAP_ERR_DISCONNECTED;
            }
            // else push procedure to be executed onto another connection
            else
            {
                gatt_proc_push(conidx, (gatt_proc_t*) p_proc);
                break;
            }
        } while(proc_finished);

        if(proc_finished)
        {
            // perform a buffer release (only first one to prevent issue if next pointer of buffer has been altered during event transmission)
            co_buf_release((co_buf_t*) co_list_pop_front(&(p_proc->buf_queue)));
            // Mark procedure ready to be free - already pop-ed
            SETB(p_proc->hdr.info_bf, GATT_PROC_FREE, true);

            // Inform user that procedure is over
            gatt_srv_event_call_sent_cb(GAP_INVALID_CONIDX, p_proc,
                                        GETB(p_proc->state_bf, GATT_SRV_EVENT_CANCELED) ? GAP_ERR_CANCELED : GAP_ERR_NO_ERROR);
        }
    }
}

/// Function call to send reliable event
__STATIC void gatt_srv_event_reliable_proc_continue(uint8_t conidx, gatt_srv_event_proc_t* p_proc, uint8_t proc_state, uint16_t status)
{
    bool finished = false;

    switch(proc_state)
    {
        case GATT_PROC_START:
        {
            p_proc->remain_len = gatt_proc_mtu_get(conidx, (gatt_proc_t*) p_proc)- L2CAP_ATT_HEADER_LEN;
            if(p_proc->nb_att  == 1)
            {
                p_proc->remain_len -= GATT_HANDLE_LEN;
            }
        }
        // no break;
        case GATT_PROC_USER_CFM:
        {
            if(status != GAP_ERR_NO_ERROR) break;

            // check that all attribute has been parsed and there is remaining buffer data
            if((p_proc->cursor < p_proc->nb_att) && (p_proc->remain_len > 0))
            {
                gatt_user_t* p_user  = gatt_user_get(p_proc->hdr.user_lid);
                uint16_t max_len = p_proc->remain_len;

                // multiple notification, check remaining length
                if(p_proc->nb_att > 1)
                {
                    if(p_proc->remain_len <= (GATT_HANDLE_LEN + L2CAP_ATT_VALLEN_LEN))
                    {
                        max_len = 0;
                    }
                    else
                    {
                        max_len -= GATT_HANDLE_LEN + L2CAP_ATT_VALLEN_LEN;
                    }
                }

                SETB(p_proc->state_bf, GATT_SRV_EVENT_CFM_RECEIVED, false);
                // ask GATT user to provide attribute value to notify
                p_user->p_cb->srv.cb_att_event_get(conidx, p_proc->hdr.user_lid, p_proc->hdr.token,
                                                   p_proc->hdr.dummy, p_proc->atts[p_proc->cursor].hdl,
                                                   co_min(p_proc->atts[p_proc->cursor].length, max_len));
            }
            else
            {
                co_buf_t* p_buf = NULL;

                // get fist buffer from queue
                p_buf = (co_buf_t*) co_list_pop_front(&(p_proc->buf_queue));
                ASSERT_ERR(p_buf != NULL);

                if(p_proc->nb_att == 1)
                {
                    status = gatt_srv_event_ntf_ind_send(conidx, p_proc, p_buf);
                }
                else
                {
                    status = gatt_srv_event_mult_ntf_send(conidx, p_proc, p_buf);
                }
                co_buf_release(p_buf);
            }
        } break;
        case GATT_PROC_PDU_PUSHED_TO_LL:
        {
            // procedure is over here for a notification
            if(p_proc->hdr.proc_id != GATT_PROC_NOTIFY) break;
        }
        // no break;
        case GATT_PROC_PDU_RX:
        case GATT_PROC_ERROR: // Handle error detection
        {
            finished = true;
        } break;

        default: { /*  Nothing to do */  } break;
    }

    if(finished || (status != GAP_ERR_NO_ERROR))
    {
        // Terminate procedure
        gatt_srv_event_proc_terminate(conidx, p_proc, status);
    }
}


/**
 ****************************************************************************************
 * @brief Create Event send procedure
 *
 * @param[in]  conidx         Connection index
 * @param[in]  user_lid       GATT User Local identifier
 * @param[in]  dummy          Dummy parameter whose meaning is upper layer dependent and
 *                            which is returned in command complete.
 * @param[in]  evt_type       Event type to trigger (see enum #gatt_evt_type)
 * @param[in]  nb_att         Number of attribute
 * @param[in]  p_atts         Pointer to List of attribute
 * @param[in]  p_data         Available data to transmit
 * @param[in]  proc_trans_cb  Transition procedure callback
 * @param[out] pp_proc        Pointer to the allocated procedure
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC uint16_t gatt_srv_event_proc_create(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t evt_type,
                                             uint8_t nb_att, const gatt_att_t* p_atts, co_buf_t* p_data,
                                             gatt_proc_cb proc_trans_cb, gatt_srv_event_proc_t** pp_proc)
{
    DBG_FUNC_ENTER(gatt_srv_event_proc_create);
    uint16_t status      = GAP_ERR_NO_ERROR;
    gatt_user_t* p_user  = gatt_user_get(user_lid);
    uint16_t tx_length   = L2CAP_ATT_HEADER_LEN;

    // Ensure that user can handle the procedure
    if((p_user == NULL) || (p_user->p_cb->srv.cb_event_sent == NULL))
    {
        status = GAP_ERR_NOT_SUPPORTED;
    }
    else if((p_data == NULL) && (p_user->p_cb->srv.cb_att_read_get == NULL))
    {
        status = GAP_ERR_NOT_SUPPORTED;
    }
    // Check if connection exists and
    else if((conidx >= HOST_CONNECTION_MAX) || (gatt_env.p_con[conidx] == NULL))
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    // check buffer properties
    else if(   (p_data != NULL)
            && ((co_buf_head_len(p_data) < GATT_BUFFER_HEADER_LEN) || (co_buf_tail_len(p_data) < GATT_BUFFER_TAIL_LEN)))
    {
        status = GAP_ERR_INVALID_BUFFER;
    }
    // check parameter validity
    else if(   (nb_att == 0) || (p_atts == NULL) || (evt_type > GATT_INDICATE)
            || ((evt_type == GATT_INDICATE) && (nb_att > 1)))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    // check if event is authorized
    else if(gapc_svc_is_cli_out_of_sync(conidx, (evt_type == GATT_NOTIFY) ? L2CAP_ATT_HDL_VAL_NTF_OPCODE
                                                                          : L2CAP_ATT_HDL_VAL_IND_OPCODE, p_atts->hdl))
    {
        status = ATT_ERR_DB_OUT_OF_SYNC;
    }
    // check if multiple notification is supported
    else if ((nb_att > 1) && !gapc_svc_is_cli_mult_ntf_supported(conidx))
    {
        status = GAP_ERR_NOT_SUPPORTED;
    }
    else
    {
        uint8_t cursor;

        // browse all attributes to find if procedure is authorized
        for(cursor = 0 ; cursor < nb_att ; cursor++)
        {
            gatt_db_svc_t* p_svc;
            gatt_db_att_t* p_att;
            const gatt_att_t* p_att_evt = &(p_atts[cursor]);

            // retrieve attribute
            status = gatt_db_att_get(p_att_evt->hdl, &p_svc, &p_att);

            if(status != GAP_ERR_NO_ERROR) break;

            // check that user is owner of attribute
            if(p_svc->user_lid != user_lid)
            {
                status = GAP_ERR_COMMAND_DISALLOWED;
            }

            // check if permission according to attribute security level
            status = gatt_db_att_access_check(conidx, (evt_type == GATT_NOTIFY ? GATT_DB_ACCESS_NOTIFY
                                                                               : GATT_DB_ACCESS_INDICATE), p_svc, p_att);

            // compute data to transmit
            tx_length += GATT_HANDLE_LEN + p_att_evt->length + ((nb_att > 1) ? L2CAP_ATT_VALLEN_LEN : 0);

            if(status != GAP_ERR_NO_ERROR) break;
        }

        if(status == GAP_ERR_NO_ERROR)
        {
            gatt_srv_event_proc_t* p_proc;

            // Create procedure
            status = gatt_proc_create(conidx, user_lid, dummy,
                                      (evt_type == GATT_NOTIFY ? GATT_PROC_NOTIFY: GATT_PROC_INDICATE), tx_length,
                                      (sizeof(gatt_srv_event_proc_t) + (sizeof(gatt_att_t) * nb_att)),
                                      proc_trans_cb, (gatt_proc_t**) &p_proc);

            if(status == GAP_ERR_NO_ERROR)
            {
                #if (HOST_MSG_API)
                gatt_proc_cur_set((gatt_proc_t*) p_proc);
                #endif // (HOST_MSG_API)

                memcpy(p_proc->atts, p_atts, (sizeof(gatt_att_t) * nb_att));
                p_proc->nb_att    = nb_att;
                p_proc->state_bf  = GATT_SRV_EVENT_CFM_RECEIVED_BIT;
                p_proc->total_len = 0;
                p_proc->conidx_bf = 0;
                co_list_init(&(p_proc->buf_queue));

                if(p_data == NULL)
                {
                    p_proc->cursor    = 0;
                }
                else
                {
                    // mark that attribute data is available
                    p_proc->cursor    = 1;
                    p_proc->total_len = p_atts[0].length;
                    co_list_push_back(&(p_proc->buf_queue), &(p_data->hdr));
                    co_buf_acquire(p_data);
                }

                // ask procedure to be granted - push it in wait list
                gatt_proc_push(conidx, (gatt_proc_t*) p_proc);

                *pp_proc = p_proc;
            }
        }
    }
    DBG_FUNC_EXIT(gatt_srv_event_proc_create);

    return (status);
}


/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t gatt_srv_event_reliable_send(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t evt_type,
                                      uint8_t nb_att, const gatt_att_t* p_atts)
{
    uint16_t status;
    gatt_srv_event_proc_t* p_proc;

    // just create the procedure
    status = gatt_srv_event_proc_create(conidx, user_lid, dummy, evt_type, nb_att, p_atts, NULL,
                                        (gatt_proc_cb) gatt_srv_event_reliable_proc_continue, &p_proc);

    return (status);
}


uint16_t gatt_srv_att_event_get_cfm(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t status,
                                    uint16_t att_length, co_buf_t* p_data)
{
    gatt_srv_event_proc_t* p_proc = (gatt_srv_event_proc_t*) gatt_proc_pick(conidx, token);

    do
    {
        // check if procedure is found
        if((p_proc == NULL) || (user_lid != p_proc->hdr.user_lid))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }
        else if((p_proc->hdr.proc_id != GATT_PROC_NOTIFY) && (p_proc->hdr.proc_id != GATT_PROC_INDICATE))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }
        else if(GETB(p_proc->state_bf, GATT_SRV_EVENT_CFM_RECEIVED))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        if(status == GAP_ERR_NO_ERROR)
        {
            co_buf_t* p_buf;
            uint16_t data_len;

            // buffer can be NULL
            if(p_data == NULL)
            {
                status = GAP_ERR_INVALID_PARAM;
                break;
            }
            // check buffer properties
            else if((co_buf_head_len(p_data) < GATT_BUFFER_HEADER_LEN) || (co_buf_tail_len(p_data) < GATT_BUFFER_TAIL_LEN))
            {
                status = GAP_ERR_INVALID_BUFFER;
                break;
            }

            p_buf = (co_buf_t*) co_list_tail(&(p_proc->buf_queue));

            // Multiple notifications - put handle and attribute length in front of data payload
            if(p_proc->nb_att > 1)
            {
                co_buf_head_reserve(p_data, L2CAP_ATT_VALLEN_LEN);
                co_write16p(co_buf_data(p_data), co_htobs(att_length));

                co_buf_head_reserve(p_data, GATT_HANDLE_LEN);
                co_write16p(co_buf_data(p_data), co_htobs(p_proc->atts[p_proc->cursor].hdl));
            }

            data_len = co_buf_data_len(p_data);
            data_len = co_min(data_len, p_proc->remain_len);
            // check if buffer data can be copied in existing buffer
            if((p_buf != NULL) && (co_buf_tail_len(p_buf) >= data_len))
            {
                // here data is copy - it's a trade-off between memory optimization and reduce number of copy.
                // here memory usage is preferred (should be the case only for read-multiple)
                co_buf_copy_data_to_mem(p_data, co_buf_tail(p_buf), data_len);
                co_buf_tail_reserve(p_buf, data_len);
            }
            // else push buffer in queue
            else
            {
                co_list_push_back(&(p_proc->buf_queue), &(p_data->hdr));
                co_buf_tail_release(p_data, co_buf_data_len(p_data) - data_len);
                co_buf_acquire(p_data);
            }

            p_proc->total_len      += data_len;
            p_proc->remain_len     -= data_len;
            p_proc->cursor         += 1;
        }

        // continue procedure execution
        SETB(p_proc->state_bf, GATT_SRV_EVENT_CFM_RECEIVED, true);
        gatt_proc_continue(conidx, (gatt_proc_t*) p_proc, GATT_PROC_USER_CFM, status);

        status = GAP_ERR_NO_ERROR;
    } while(0);
    return (status);
}

uint16_t gatt_srv_event_send(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t evt_type, uint16_t hdl,
                             co_buf_t* p_data)
{
    uint16_t status;
    gatt_srv_event_proc_t* p_proc;
    gatt_att_t att_info =
    {
        .hdl    = hdl,
        .length = co_buf_data_len(p_data),
    };

    if(p_data == NULL)
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else
    {
        // first create the procedure
        status = gatt_srv_event_proc_create(conidx, user_lid, dummy, evt_type, 1, &att_info, p_data,
                                            (gatt_proc_cb)gatt_srv_event_simple_proc_continue, &p_proc);
    }

    return (status);
}

uint16_t gatt_srv_event_mtp_send(uint32_t conidx_bf, uint8_t user_lid, uint16_t dummy, uint8_t evt_type,
                                 uint16_t hdl, co_buf_t* p_data, bool filter)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    uint8_t conidx;

    // remove impossible connection index in bit field
    conidx_bf &= CO_BIT(HOST_CONNECTION_MAX) - 1;

    if((conidx_bf == 0) || (p_data == NULL))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else
    {
        uint32_t conidx_bf_cursor = conidx_bf;

        // check if all connections are supported
        while(conidx_bf_cursor != 0)
        {
            conidx = co_ctz(conidx_bf_cursor);
            CO_BIT_SET(&conidx_bf_cursor, conidx, false);

            if(gatt_env.p_con[conidx] == NULL)
            {
                status = GAP_ERR_COMMAND_DISALLOWED;
                break;
            }
        }
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        gatt_srv_event_proc_t* p_proc;
        gatt_att_t att_info =
        {
            .hdl    = hdl,
            .length = co_buf_data_len(p_data),
        };

        conidx = co_ctz(conidx_bf);

        // Create the procedure
        status = gatt_srv_event_proc_create(conidx, user_lid, dummy, evt_type, 1, &att_info, p_data,
                                            (gatt_proc_cb)gatt_srv_event_mtp_proc_continue, &p_proc);

        if(status == GAP_ERR_NO_ERROR)
        {
            // mark that attribute data is available
            p_proc->conidx_bf = conidx_bf;
            SETB(p_proc->state_bf, GATT_SRV_EVENT_MTP_CMP_EVT_FILTERED, filter);
        }
    }

    return (status);
}

uint16_t gatt_srv_event_mtp_cancel(uint8_t user_lid, uint16_t dummy)
{
    uint8_t conidx;
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    for(conidx = 0; conidx < HOST_CONNECTION_MAX ; conidx++)
    {
        gatt_srv_event_proc_t* p_proc = (gatt_srv_event_proc_t*) gatt_proc_find(conidx, dummy);

        if((p_proc != NULL) && (p_proc->hdr.user_lid == user_lid))
        {
            switch(p_proc->hdr.proc_id)
            {
                case GATT_PROC_NOTIFY:
                case GATT_PROC_INDICATE:
                {
                    // it must be a multi-point procedure
                    if(p_proc->conidx_bf != 0)
                    {
                        status = GAP_ERR_NO_ERROR;
                        SETB(p_proc->state_bf, GATT_SRV_EVENT_CANCELED, true);

                        // if procedure not yet started
                        if(GETF(p_proc->hdr.info_bf, GATT_PROC_STATE) == GATT_PROC_WAIT_GRANT)
                        {
                            // Request an immediate procedure abort
                            gatt_proc_continue(conidx, &(p_proc->hdr), GATT_PROC_PDU_RX, GAP_ERR_CANCELED);
                        }
                    }
                } break;
                default:                         { /* Nothing to do */  } break;
            }

            // mark that procedure is over.
            if(status == GAP_ERR_NO_ERROR) break;
        }
    }

    return (status);
}

#if (HOST_MSG_API)
/*
 * MESSAGE HANDLER FUNCTIONS
 ****************************************************************************************
 */
#include "gatt_msg_int.h"

/**
 ****************************************************************************************
 * @brief Handle transmission request of a reliable event (notification or indication)
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_srv_event_reliable_send_cmd_handler(gatt_srv_event_reliable_send_cmd_t* p_cmd, uint16_t src_id)
{
    uint16_t status = gatt_srv_event_reliable_send(p_cmd->conidx, p_cmd->user_lid, p_cmd->dummy, p_cmd->evt_type,
                                                   p_cmd->nb_att, p_cmd->atts);

    // if an error occurs immediately send back command completed
    if(status != GAP_ERR_NO_ERROR)
    {
        gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
    }
    else
    {
        gatt_srv_event_proc_t* p_proc = (gatt_srv_event_proc_t*) gatt_proc_cur_get();
        p_proc->msg_cmd_code = p_cmd->cmd_code;
    }
}

/**
 ****************************************************************************************
 * @brief This function is called when GATT server user has initiated event send procedure,
 *
 *        @see gatt_srv_att_event_get_cfm shall be called to provide attribute value
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution.
 * @param[in] hdl           Attribute handle
 * @param[in] offset        Data offset
 * @param[in] max_length    Maximum data length to return
 ****************************************************************************************
 */
void gatt_srv_att_event_get_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t dummy, uint16_t hdl,
                               uint16_t max_length)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        gatt_srv_att_event_get_req_ind_t* p_req_ind = KE_MSG_ALLOC(GATT_REQ_IND, p_user->dest_task_nbr, TASK_GATT,
                                                                   gatt_srv_att_event_get_req_ind);

        // prepare request indication to send
        if(p_req_ind != NULL)
        {
            p_req_ind->req_ind_code  = GATT_SRV_ATT_EVENT_GET;
            p_req_ind->token         = token;
            p_req_ind->user_lid      = user_lid;
            p_req_ind->conidx        = conidx;
            p_req_ind->dummy         = dummy;
            p_req_ind->hdl           = hdl;
            p_req_ind->max_length    = max_length;

            // send message to host
            ke_msg_send(p_req_ind);
        }
    }
    else
    {
        gatt_srv_att_event_get_cfm(conidx, user_lid, token, ATT_ERR_UNLIKELY_ERR, 0, NULL);
    }
}

/**
 ****************************************************************************************
 * @brief Handle confirmation of reliable event value to send to a peer device.
 *
 * @param[in] p_cfm     Pointer to confirmation parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_srv_att_event_get_cfm_handler(gatt_srv_att_event_get_cfm_t* p_cfm, uint16_t src_id)
{
    co_buf_t* p_data = NULL;

    if(p_cfm->status == GAP_ERR_NO_ERROR)
    {
        // allocate a buffer for event transmission
        if(co_buf_alloc(&p_data, GATT_BUFFER_HEADER_LEN, p_cfm->value_length, GATT_BUFFER_TAIL_LEN) != CO_BUF_ERR_NO_ERROR)
        {
            p_cfm->status = ATT_ERR_INSUFF_RESOURCE;
        }
        else
        {
            // copy data into buffer
            co_buf_copy_data_from_mem(p_data, p_cfm->value, p_cfm->value_length);
        }
    }

    // execute confirmation native function
    gatt_srv_att_event_get_cfm(p_cfm->conidx, p_cfm->user_lid, p_cfm->token, p_cfm->status, p_cfm->att_length, p_data);

    // release buffer
    if(p_data != NULL)
    {
        co_buf_release(p_data);
    }
}

/**
 ****************************************************************************************
 * @brief Handle transmission request of a non-reliable event (notification or indication)
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_srv_event_send_cmd_handler(gatt_srv_event_send_cmd_t* p_cmd, uint16_t src_id)
{
    co_buf_t* p_data;
    uint16_t status;

    // allocate a buffer for event transmission
    if(co_buf_alloc(&p_data, GATT_BUFFER_HEADER_LEN, p_cmd->value_length, GATT_BUFFER_TAIL_LEN) != CO_BUF_ERR_NO_ERROR)
    {
        status = ATT_ERR_INSUFF_RESOURCE;
    }
    else
    {
        // copy data into buffer
        co_buf_copy_data_from_mem(p_data, p_cmd->value, p_cmd->value_length);

        // request event to be sent
        status = gatt_srv_event_send(p_cmd->conidx, p_cmd->user_lid, p_cmd->dummy, p_cmd->evt_type,
                                                   p_cmd->hdl, p_data);

        co_buf_release(p_data);
    }

    // if an error occurs immediately send back command completed
    if(status != GAP_ERR_NO_ERROR)
    {
        gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
    }
    else
    {
        gatt_srv_event_proc_t* p_proc = (gatt_srv_event_proc_t*) gatt_proc_cur_get();
        p_proc->msg_cmd_code = p_cmd->cmd_code;
    }
}

/**
 ****************************************************************************************
 * @brief Handle transmission request of a multi point non-reliable event (notification or indication)
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_srv_event_mtp_send_cmd_handler(gatt_srv_event_mtp_send_cmd_t* p_cmd, uint16_t src_id)
{
    co_buf_t* p_data;
    uint16_t status;

    // allocate a buffer for event transmission
    if(co_buf_alloc(&p_data, GATT_BUFFER_HEADER_LEN, p_cmd->value_length, GATT_BUFFER_TAIL_LEN) != CO_BUF_ERR_NO_ERROR)
    {
        status = ATT_ERR_INSUFF_RESOURCE;
    }
    else
    {
        // copy data into buffer
        co_buf_copy_data_from_mem(p_data, p_cmd->value, p_cmd->value_length);

        // request event to be sent
        status = gatt_srv_event_mtp_send(p_cmd->conidx_bf, p_cmd->user_lid, p_cmd->dummy, p_cmd->evt_type,
                                         p_cmd->hdl, p_data, false);

        co_buf_release(p_data);
    }

    // if an error occurs immediately send back command completed
    if(status != GAP_ERR_NO_ERROR)
    {
        gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, GAP_INVALID_CONIDX, src_id, p_cmd->user_lid, status);
    }
    else
    {
        gatt_srv_event_proc_t* p_proc = (gatt_srv_event_proc_t*) gatt_proc_cur_get();
        p_proc->msg_cmd_code = p_cmd->cmd_code;
    }
}


/**
 ****************************************************************************************
 * @brief Handle cancelation request request of a multi point non-reliable event (notification or indication)
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_srv_event_mtp_cancel_cmd_handler(gatt_srv_event_mtp_cancel_cmd_t* p_cmd, uint16_t src_id)
{
    uint16_t status = gatt_srv_event_mtp_cancel(p_cmd->user_lid, p_cmd->dummy);
    gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, GAP_INVALID_CONIDX, src_id, p_cmd->user_lid,
                               status);
}

/**
 ****************************************************************************************
 * @brief This function is called when GATT server user has initiated event send to peer
 *        device or if an error occurs.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] status        Status of the procedure (see enum #hl_err)
 ****************************************************************************************
 */
void gatt_srv_event_sent_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        gatt_srv_event_proc_t* p_proc = (gatt_srv_event_proc_t*) gatt_proc_cur_get();

        // send back command complete event
        gatt_msg_send_proc_cmp_evt(p_proc->msg_cmd_code, dummy, conidx, p_user->dest_task_nbr, user_lid, status);
    }
}

#endif // (HOST_MSG_API)
#endif // (BLE_GATT)
/// @} GATT

