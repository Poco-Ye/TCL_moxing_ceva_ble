/**
 ****************************************************************************************
 * @file gatt_cli_event.c
 *
 * @brief  GATT Client Event Procedures
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
#if (BLE_GATT)
#include "gatt.h"           // Native API
#if (BLE_GATT_CLI)
#include "gatt.h"           // Native API
#include "gatt_int.h"       // Internals

#include <string.h>         // for memcopy
#include "co_endian.h"      // endianess support

#include "ke_mem.h"         // for allocation of registration array

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



/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Client procedure structure.
typedef struct gatt_cli_event_proc
{
    /// Procedure header - required for any Attribute procedure
    gatt_proc_handler_t     hdr;
    /// Pointer to current buffer
    co_buf_t*               p_buf;
    /// Attribute handle targeted.
    uint16_t                hdl;
    /// Maximum value size computed according to MTU when event is received
    uint16_t                max_value_size;
} gatt_cli_event_proc_t;

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
 * @brief Find a pointer in the event registration array that enter in collision with
 *        provided range or first pointer in array were a new range can be inserted.
 *
 * @param[in]  conidx       Connection index
 * @param[in]  user_lid     GATT User Local identifier
 * @param[in]  start_hdl    Attribute start handle range
 * @param[in]  end_hdl      Attribute end handle range
 * @param[in]  avail_slot   Search for an available slot

 * @return Pointer to an element that enter in collision with handle range or a position
 *         where a new range can be inserted.
 ****************************************************************************************
 */
__STATIC gatt_cli_event_reg_t* gatt_cli_event_reg_find(uint8_t conidx, uint16_t start_hdl, uint16_t end_hdl,
                                                       bool avail_slot)
{
    uint8_t cursor;
    gatt_cli_event_reg_t* p_found_reg = NULL;
    gatt_con_env_t* p_con  = gatt_env.p_con[conidx];

    // find a value that matches
    for(cursor = 0 ; cursor < p_con->reg_event_size ; cursor++)
    {
        gatt_cli_event_reg_t* p_reg = &(p_con->p_reg_events[cursor]);

        // find an available slot.
        if(p_reg->start_hdl == GATT_INVALID_HDL)
        {
            if((p_found_reg == NULL) && avail_slot)
            {
                p_found_reg = p_reg;
            }
        }
        // search if start handle or end handle can enter in collision
        else if((p_reg->start_hdl <= end_hdl) && (p_reg->end_hdl >= start_hdl))
        {
            p_found_reg = p_reg;
            break;
        }
    }

    return (p_found_reg);
}


/**
 ****************************************************************************************
 * @brief Function called when L2CAP_ATT_HDL_VAL_NTF or L2CAP_ATT_HDL_VAL_IND attribute PDU is
 *        received.
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
__STATIC uint16_t gatt_cli_l2cap_att_hdl_val_ntf_ind_handler(uint8_t conidx, gatt_cli_event_proc_t* p_proc,
                                                             l2cap_att_hdl_val_ntf_t* p_pdu,
                                                             co_buf_t* p_buf, uint16_t mtu)
{
    if(p_pdu->code != L2CAP_ATT_MULT_HDL_VAL_NTF_OPCODE)
    {
        // store received information
        p_proc->hdl            = p_pdu->handle;
        p_proc->max_value_size = mtu - L2CAP_ATT_HEADER_LEN - GATT_HANDLE_LEN;
    }
    else
    {
        p_proc->max_value_size = mtu - L2CAP_ATT_HEADER_LEN;
    }

    p_proc->p_buf          = p_buf;


    // continue reception if an indication is received, else wait for GATT user confirmation
    if(p_proc->hdr.proc_id == GATT_PROC_HANDLE_IND)
    {
        gatt_proc_bearer_rx_continue(conidx, (gatt_proc_t*) p_proc);
    }

    // acquire buffer
    co_buf_acquire(p_buf);

    return (GAP_ERR_NO_ERROR);
}


/**
 ****************************************************************************************
 * @brief function called when operation state is updated
 *
 * @param[in] conidx     Connection index
 * @param[in] p_proc     Pointer to procedure to continue
 * @param[in] proc_state Operation transition state (see enum #gatt_proc_state)
 * @param[in] status     Execution status
 ****************************************************************************************
 */
__STATIC void gatt_cli_event_proc_continue(uint8_t conidx, gatt_cli_event_proc_t* p_proc, uint8_t proc_state, uint16_t status)
{
    bool finished = false;
    bool user_cfm_received;

    do
    {
        user_cfm_received = false;

        switch(proc_state)
        {
            case GATT_PROC_USER_CFM:
            {
                // Indication received send back confirmation
                if(p_proc->hdr.proc_id == GATT_PROC_HANDLE_IND)
                {
                    l2cap_att_pdu_t pdu;
                    co_buf_t* p_buf = NULL;

                    // allocate buffer used for confirmation transmission
                    if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, GATT_BUFFER_TAIL_LEN) != CO_BUF_ERR_NO_ERROR)
                    {
                        status = GAP_ERR_INSUFF_RESOURCES;
                        break;
                    }

                    pdu.code = L2CAP_ATT_HDL_VAL_CFM_OPCODE;

                    // Ask for PDU transmission
                    status = gatt_proc_pdu_send(conidx, (gatt_proc_t*) p_proc, &pdu, p_buf, NULL);
                    // release buffer
                    co_buf_release(p_buf);
                }
            }
            // no break;

            case GATT_PROC_PDU_RX:
            {
                if(p_proc->p_buf != NULL)
                {
                    gatt_cli_event_reg_t* p_found_reg = NULL;
                    gatt_user_t* p_user = NULL;
                    uint16_t buffer_len = 0;
                    uint16_t ntf_len    = 0;
                    bool complete_data  = false;
                    co_buf_t* p_buf     = p_proc->p_buf;

                    do
                    {
                        // multiple notification
                        if(p_proc->hdr.proc_id == GATT_PROC_HANDLE_NTF_MULT)
                        {
                            // extract handle value
                            if(co_buf_data_len(p_buf) >= GATT_HANDLE_LEN)
                            {
                                p_proc->hdl = co_btohs(co_read16p(co_buf_data(p_buf)));
                                co_buf_head_release(p_buf, GATT_HANDLE_LEN);

                                // extract value length
                                if(co_buf_data_len(p_buf) >= L2CAP_ATT_VALLEN_LEN)
                                {
                                    uint16_t data_len = 0;
                                    data_len = co_btohs(co_read16p(co_buf_data(p_buf)));
                                    co_buf_head_release(p_buf, L2CAP_ATT_VALLEN_LEN);

                                    // retrieve notification length
                                    buffer_len = co_buf_data_len(p_buf);

                                    // check if complete data present
                                    if(data_len <= buffer_len)
                                    {
                                        ntf_len = data_len;
                                        complete_data = true;
                                    }
                                    else
                                    {
                                        ntf_len = buffer_len;
                                    }
                                }
                                else
                                {
                                    // remove data
                                    co_buf_head_release(p_buf, co_buf_data_len(p_buf));
                                }
                            }
                            else
                            {
                                finished = true;
                                break;
                            }
                        }
                        else
                        {
                            buffer_len = co_buf_data_len(p_buf);
                            ntf_len    = buffer_len;
                            // check if complete data present
                            complete_data = (ntf_len < p_proc->max_value_size);
                        }

                        // find user that registered for this event
                        p_found_reg = gatt_cli_event_reg_find(conidx, p_proc->hdl, p_proc->hdl, false);

                        // if no client found, check how to continue
                        if(p_found_reg == NULL)
                        {
                            if(p_proc->hdr.proc_id == GATT_PROC_HANDLE_NTF_MULT)
                            {
                                // remove data
                                co_buf_head_release(p_buf, ntf_len);
                            }
                            else
                            {
                                finished = true;
                                break;
                            }
                        }
                    } while(p_found_reg == NULL);

                    if(!finished)
                    {
                        // retrieve user
                        p_user = gatt_user_get(p_found_reg->user_lid);
                        ASSERT_ERR(p_user != NULL);

                        p_proc->hdr.user_lid = p_found_reg->user_lid;

                        if(ntf_len < buffer_len)
                        {
                            co_buf_t* p_buf_out = NULL;

                            // allocate new buffer to provide data information to upper layer
                            if(co_buf_alloc(&p_buf_out, 0, ntf_len, 0) != CO_BUF_ERR_NO_ERROR)
                            {
                                finished = true;
                                break;
                            }
                            else
                            {
                                // copy buffer data
                                co_buf_copy(p_buf, p_buf_out, ntf_len, 0);
                                co_buf_head_release(p_buf, ntf_len);
                            }

                            p_buf = p_buf_out;
                        }
                        else
                        {
                            p_proc->p_buf = NULL;
                        }

                        // inform user of received event
                        SETB(p_proc->hdr.info_bf, GATT_PROC_USER_CFM, false);
                        p_user->p_cb->cli.cb_att_val_evt(conidx, p_found_reg->user_lid, p_proc->hdr.token,
                                                         p_proc->hdr.proc_id == GATT_PROC_HANDLE_IND ? GATT_INDICATE : GATT_NOTIFY,
                                                         complete_data, p_proc->hdl, p_buf);

                        // Release the buffer
                        co_buf_release(p_buf);
                        // check if upper layer application has confirm event
                        if(GETB(p_proc->hdr.info_bf, GATT_PROC_USER_CFM))
                        {
                           user_cfm_received = true;
                           proc_state = GATT_PROC_USER_CFM;
                        }
                    }

                    break;
                }
            }
            // no break;
            case GATT_PROC_PDU_PUSHED_TO_LL:    // Confirmation properly pushed in TX LL buffers
            case GATT_PROC_ERROR:               // Handle error detection
            {
                finished = true;
            } break;

            default: { /*  Nothing to do */  } break;
        }
    } while(user_cfm_received);

    if(status != GAP_ERR_NO_ERROR)
    {
        finished = true;
    }

    if(finished)
    {
        // notification received, nothing more to do
        if((proc_state != GATT_PROC_ERROR) && (p_proc->hdr.proc_id != GATT_PROC_HANDLE_IND))
        {
            gatt_proc_bearer_rx_continue(conidx, (gatt_proc_t*) p_proc);
        }

        // check if buffer must be released
        if(p_proc->p_buf != NULL)
        {
            co_buf_release(p_proc->p_buf);
        }

        // Pop Procedure
        gatt_proc_pop(conidx, (gatt_proc_t*) p_proc, true);
    }
}

/*
 * INTERNAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Ask GATT client module to create an Event procedure handler
 *
 * @param[in]  conidx       Connection index
 * @param[in]  op_code      Attribute operation code received
 * @param[out] p_pdu_hdl_cb Pointer to PDU Handler call-back
 * @param[out] pp_proc      Pointer to the procedure created, NULL if procedure not created
 *
 * @return Procedure creation status code (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gatt_cli_event_proc_create(uint8_t conidx, uint8_t op_code, gatt_proc_pdu_handler_cb* p_pdu_hdl_cb,
                                    gatt_proc_handler_t** pp_proc)
{
    uint16_t status;
    uint8_t  proc_id = 0;

    // compute procedure identifier
    switch(op_code)
    {
        case L2CAP_ATT_HDL_VAL_NTF_OPCODE:      {proc_id = GATT_PROC_HANDLE_NTF;      } break;
        case L2CAP_ATT_HDL_VAL_IND_OPCODE:      {proc_id = GATT_PROC_HANDLE_IND;      } break;
        case L2CAP_ATT_MULT_HDL_VAL_NTF_OPCODE: {proc_id = GATT_PROC_HANDLE_NTF_MULT; } break;
        default:                                { ASSERT_ERR(0);                      } break;
    }

    // Allocate procedure
    status = gatt_proc_handler_alloc(conidx, proc_id, sizeof(gatt_cli_event_proc_t),
                                     (gatt_proc_cb) gatt_cli_event_proc_continue, pp_proc);

    *p_pdu_hdl_cb = (gatt_proc_pdu_handler_cb) gatt_cli_l2cap_att_hdl_val_ntf_ind_handler;

    return (status);
}


void gatt_cli_event_svc_chg(uint8_t conidx, bool out_of_sync,  uint16_t start_hdl, uint16_t end_hdl)
{
    uint8_t user_lid;

    // initialize new elements
    for(user_lid = 0 ; user_lid < BLE_GATT_USER_NB; user_lid ++)
    {
        gatt_user_t* p_user = gatt_user_get(user_lid);

        // Ensure that user can handle the event
        if((p_user != NULL) && (p_user->role == GATT_ROLE_CLIENT) && (p_user->p_cb->cli.cb_svc_changed != NULL))
        {
            p_user->p_cb->cli.cb_svc_changed(conidx, user_lid, out_of_sync, start_hdl, end_hdl);
        }
    }
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t gatt_cli_event_register(uint8_t conidx, uint8_t user_lid, uint16_t start_hdl, uint16_t end_hdl)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    do
    {
        gatt_con_env_t* p_con;
        gatt_user_t* p_user;
        gatt_cli_event_reg_t* p_found_reg;

        // check parameters
        if((start_hdl == GATT_INVALID_HDL) || (end_hdl < start_hdl))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Check if connection exists
        if((conidx >= HOST_CONNECTION_MAX) || (gatt_env.p_con[conidx] == NULL))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        p_con  = gatt_env.p_con[conidx];
        p_user = gatt_user_get(user_lid);

        // Ensure that user can handle the procedure
        if((p_user == NULL) || (p_user->role != GATT_ROLE_CLIENT) ||  (p_user->p_cb->cli.cb_att_val_evt == NULL))
        {
            status = GAP_ERR_NOT_SUPPORTED;
            break;
        }

        // find an available slot for event registration
        p_found_reg = gatt_cli_event_reg_find(conidx, start_hdl, end_hdl, true);

        // nothing found, it means that register array size is too small, allocate a new one.
        if(p_found_reg == NULL)
        {
            uint8_t cursor;
            ASSERT_ERR(p_con->reg_event_nb == p_con->reg_event_size);

            p_con->reg_event_size += BLE_GATT_USER_NB;

            // allocate a new array of registered event
            p_found_reg = (gatt_cli_event_reg_t*) ke_malloc_user(sizeof(gatt_cli_event_reg_t) * p_con->reg_event_size,
                                                                        KE_MEM_ENV);

            // error due to insufficient resources
            if(p_found_reg == NULL)
            {
                status = GAP_ERR_INSUFF_RESOURCES;
                break;
            }

            if(p_con->p_reg_events != NULL)
            {
                // copy old register array
                memcpy(p_found_reg, p_con->p_reg_events, sizeof(gatt_cli_event_reg_t) * p_con->reg_event_nb);
                ke_free(p_con->p_reg_events);
            }

            // Update array
            p_con->p_reg_events = p_found_reg;
            p_found_reg = &(p_con->p_reg_events[p_con->reg_event_nb]);

            // initialize new elements
            for(cursor = p_con->reg_event_nb ; cursor < p_con->reg_event_size; cursor ++)
            {
                p_con->p_reg_events[cursor].start_hdl = GATT_INVALID_HDL;
                p_con->p_reg_events[cursor].user_lid  = GATT_INVALID_USER_LID;
            }
        }

        // check if a client is already registered in the range
        if(p_found_reg->start_hdl != GATT_INVALID_HDL)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // assign client
        p_found_reg->start_hdl = start_hdl;
        p_found_reg->end_hdl   = end_hdl;
        p_found_reg->user_lid  = user_lid;

        // update number of registered element
        p_con->reg_event_nb   += 1;

    } while(0);

    return (status);
}

uint16_t gatt_cli_event_unregister(uint8_t conidx, uint8_t user_lid, uint16_t start_hdl, uint16_t end_hdl)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    do
    {
        gatt_con_env_t* p_con;
        gatt_user_t* p_user;
        gatt_cli_event_reg_t* p_found_reg;

        // check parameters
        if((start_hdl == GATT_INVALID_HDL) || (end_hdl < start_hdl))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Check if connection exists
        if((conidx >= HOST_CONNECTION_MAX) || (gatt_env.p_con[conidx] == NULL))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        p_con  = gatt_env.p_con[conidx];
        p_user = gatt_user_get(user_lid);

        // Ensure that user can handle the procedure
        if((p_user == NULL) || (p_user->role != GATT_ROLE_CLIENT))
        {
            status = GAP_ERR_NOT_SUPPORTED;
            break;
        }

        // find expected handle range
        p_found_reg = gatt_cli_event_reg_find(conidx, start_hdl, end_hdl, false);

        // Sanity check on found handle range
        if((p_found_reg == NULL) || (p_found_reg->user_lid != user_lid))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // unregister
        p_found_reg->start_hdl = GATT_INVALID_HDL;
        // update number of registered element
        p_con->reg_event_nb   -= 1;
    } while(0);

    return (status);

}


void gatt_cli_event_remove_user(uint8_t user_lid)
{
    uint8_t conidx;

    // browse all connections
    for(conidx = 0 ; conidx < HOST_CONNECTION_MAX ; conidx++)
    {
        gatt_con_env_t* p_con = gatt_env.p_con[conidx];
        if(p_con != NULL)
        {
            uint8_t cursor;
            // browse all registered user
            for(cursor = 0 ; cursor < p_con->reg_event_size ; cursor++)
            {
                gatt_cli_event_reg_t* p_reg = &(p_con->p_reg_events[cursor]);

                // Unregister user
                if(p_reg->user_lid == user_lid)
                {
                    p_reg->user_lid  = GATT_INVALID_USER_LID;
                    p_reg->start_hdl = GATT_INVALID_HDL;
                }
            }
        }
    }
}

uint16_t gatt_cli_att_event_cfm(uint8_t conidx, uint8_t user_lid, uint16_t token)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    gatt_proc_handler_t* p_proc = (gatt_proc_handler_t*) gatt_proc_pick(conidx, token);

    // check if procedure is found
    if((p_proc == NULL) || (user_lid != p_proc->user_lid))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    // check if confirm not already received
    else if (GETB(p_proc->info_bf, GATT_PROC_USER_CFM))
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else
    {
        // mark that user confirm request
        SETB(p_proc->info_bf, GATT_PROC_USER_CFM, true);
        p_proc->user_lid = GATT_INVALID_USER_LID;

        // continue procedure execution if not in continue function
        if(!GETB(p_proc->info_bf, GATT_PROC_IN_CONTINUE))
        {
            gatt_proc_continue(conidx, (gatt_proc_t*) p_proc, GATT_PROC_USER_CFM, status);
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
 * @brief Handle Event registration command from GATT client user
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_event_register_cmd_handler(gatt_cli_event_register_cmd_t* p_cmd, uint16_t src_id)
{
    // Register
    uint16_t status = gatt_cli_event_register(p_cmd->conidx, p_cmd->user_lid, p_cmd->start_hdl, p_cmd->end_hdl);
    // send command completion
    gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
}

/**
 ****************************************************************************************
 * @brief Handle Event un-registration command from GATT client user
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_event_unregister_cmd_handler(gatt_cli_event_unregister_cmd_t* p_cmd, uint16_t src_id)
{
    // Un-Register
    uint16_t status = gatt_cli_event_unregister(p_cmd->conidx, p_cmd->user_lid, p_cmd->start_hdl, p_cmd->end_hdl);
    // send command completion
    gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
}

/**
 ****************************************************************************************
 * @brief This function is called when a notification or an indication is received onto
 *        register handle range (@see gatt_cli_event_register).
 *
 *        @see gatt_cli_val_event_cfm must be called to confirm event reception.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] evt_type      Event type triggered (see enum #gatt_evt_type)
 * @param[in] complete      True if event value if complete value has been received
 *                          False if data received is equals to maximum attribute protocol value.
 *                          In such case GATT Client User should perform a read procedure.
 * @param[in] hdl           Attribute handle
 * @param[in] p_data        Pointer to buffer that contains attribute value
 ****************************************************************************************
 */
void gatt_cli_att_val_evt_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint8_t evt_type, bool complete,
                             uint16_t hdl, co_buf_t* p_data)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        uint16_t data_len = co_buf_data_len(p_data);
        gatt_cli_att_event_req_ind_t* p_req_ind = KE_MSG_ALLOC_DYN(GATT_REQ_IND, p_user->dest_task_nbr, TASK_GATT,
                                                                   gatt_cli_att_event_req_ind, data_len);

        // prepare request indication to send
        if(p_req_ind != NULL)
        {
            p_req_ind->req_ind_code = GATT_CLI_ATT_EVENT;
            p_req_ind->token        = token;
            p_req_ind->user_lid     = user_lid;
            p_req_ind->conidx       = conidx;

            p_req_ind->evt_type     = evt_type;
            p_req_ind->hdl          = hdl;
            p_req_ind->complete     = complete;
            p_req_ind->value_length = data_len;

            co_buf_copy_data_to_mem(p_data, p_req_ind->value, data_len);

            // send message to host
            ke_msg_send(p_req_ind);
        }
    }
    else if(evt_type == GATT_NOTIFY) // just ignore notification if nobody can handle it.
    {
        gatt_cli_att_event_cfm(conidx, user_lid, token);
    }
    // else - Indication will create a bearer disconnection due to response timeout
}

/**
 ****************************************************************************************
 * @brief Handle confirmation of event handling from peer device by GATT client user.
 *
 * @param[in] p_cfm     Pointer to confirmation parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_att_event_cfm_handler(gatt_cli_att_event_cfm_t* p_cfm, uint16_t src_id)
{
    gatt_cli_att_event_cfm(p_cfm->conidx, p_cfm->user_lid, p_cfm->token);
}


/**
 ****************************************************************************************
 * @brief Event triggered when a service change has been received or if an attribute
 *        transaction triggers an out of sync error.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] out_of_sync   True if an out of sync error has been received
 * @param[in] start_hdl     Service start handle
 * @param[in] end_hdl       Service end handle
 ****************************************************************************************
 */
void gatt_cli_svc_changed_cb(uint8_t conidx, uint8_t user_lid, bool out_of_sync, uint16_t start_hdl, uint16_t end_hdl)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        gatt_cli_svc_changed_ind_t* p_ind = KE_MSG_ALLOC(GATT_IND, p_user->dest_task_nbr, TASK_GATT,
                                                         gatt_cli_svc_changed_ind);

        // prepare indication to send
        if(p_ind != NULL)
        {
            p_ind->ind_code     = GATT_CLI_SVC_CHANGED;
            p_ind->dummy        = 0;
            p_ind->user_lid     = user_lid;
            p_ind->conidx       = conidx;
            p_ind->out_of_sync  = out_of_sync;
            p_ind->start_hdl    = start_hdl;
            p_ind->end_hdl      = end_hdl;

            // send message to host
            ke_msg_send(p_ind);
        }
    }
}

#endif // (HOST_MSG_API)

#else  // !(BLE_GATT_CLI)
uint16_t gatt_cli_event_register(uint8_t conidx, uint8_t user_lid, uint16_t start_hdl, uint16_t end_hdl)
{
    return (GAP_ERR_NOT_SUPPORTED);
}

uint16_t gatt_cli_event_unregister(uint8_t conidx, uint8_t user_lid, uint16_t start_hdl, uint16_t end_hdl)
{
    return (GAP_ERR_NOT_SUPPORTED);
}

uint16_t gatt_cli_att_event_cfm(uint8_t conidx, uint8_t user_lid, uint16_t token)
{
    return (GAP_ERR_NOT_SUPPORTED);
}
#endif // (BLE_GATT_CLI)
#endif // (BLE_GATT)
/// @} GATT

