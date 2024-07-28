/**
 ****************************************************************************************
 * @file gatt_srv_read.c
 *
 * @brief  GATT Server Read Procedures
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
#include "gatt_int.h"       // Internals
#include "gatt_db.h"        // GATT Database access
#include "gapc.h"           // For CSRK and Sign counter usage

#include "co_endian.h"      // for host to bt number conversion
#include "co_math.h"        // for min and max

#include <string.h>         // for memcopy / memcmp

#include "../inc/gap_hl_api.h" // to get if client is out of sync
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

/// Server read procedure structure.
typedef struct gatt_srv_read_proc
{
    /// Procedure header - required for any Attribute procedure
    gatt_proc_handler_t     hdr;
    /// Pointer to received PDU buffer (used only for multiple read)
    co_buf_t*               p_pdu_buf;
    /// Queue of buffer that represents value to return (multiple read values)
    co_list_t               buf_queue;
    /// Attribute handle targeted.
    uint16_t                hdl;
    /// offset of the attribute value
    uint16_t                offset;
    /// Response PDU length
    uint16_t                rsp_len;
    /// Maximum response size that remains
    uint16_t                rsp_remain_len;
    /// received PDU code
    uint8_t                 pdu_code;
} gatt_srv_read_proc_t;


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
 * @brief Function called when L2CAP_ATT_RD_REQ or L2CAP_ATT_RD_BLOB_REQ attribute PDU is received.
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
__STATIC uint16_t gatt_srv_read_l2cap_att_rd_req_handler(uint8_t conidx, gatt_srv_read_proc_t* p_proc,
                                                         l2cap_att_rd_blob_req_t* p_pdu, co_buf_t* p_buf, uint16_t mtu)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    // store received information
    p_proc->hdl        = p_pdu->handle;
    p_proc->offset     = (p_pdu->code == L2CAP_ATT_RD_BLOB_REQ_OPCODE) ? p_pdu->offset : 0;
    p_proc->rsp_remain_len = mtu - L2CAP_ATT_HEADER_LEN;
    p_proc->pdu_code   = p_pdu->code;

    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, (gatt_proc_t*) p_proc);

    // check if client cache is out of sync
    if(gapc_svc_is_cli_out_of_sync(conidx, p_pdu->code, p_proc->hdl))
    {
        status = ATT_ERR_DB_OUT_OF_SYNC;
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Retrieve handle present in read multiple PDU
 *
 * @param[in] p_proc     Pointer to procedure under execution
 *
 * @return Next Handle of attribute that should be read
 ****************************************************************************************
 */
__STATIC uint16_t gatt_srv_read_next_hdl_get(gatt_srv_read_proc_t* p_proc)
{
    uint16_t hdl;

    hdl = co_btohs(co_read16p(co_buf_data(p_proc->p_pdu_buf)));
    co_buf_head_release(p_proc->p_pdu_buf, GATT_HANDLE_LEN);

    // keep PDU buffer to retrieve all handles
    if(co_buf_data_len(p_proc->p_pdu_buf) == 0)
    {
        co_buf_release(p_proc->p_pdu_buf);
        p_proc->p_pdu_buf = NULL;
    }

    return (hdl);
}


/**
 ****************************************************************************************
 * @brief Function called when L2CAP_ATT_RD_REQ or L2CAP_ATT_RD_BLOB_REQ attribute PDU is received.
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
__STATIC uint16_t gatt_srv_read_l2cap_att_rd_mult_req_handler(uint8_t conidx, gatt_srv_read_proc_t* p_proc,
                                                              l2cap_att_rd_mult_req_t* p_pdu, co_buf_t* p_buf, uint16_t mtu)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    uint16_t data_len = co_buf_data_len(p_buf);

    // check we have a rounded number of handle
    if((data_len < GATT_HANDLE_LEN) || ((data_len & 0x1) != 0))
    {
        status = ATT_ERR_INVALID_PDU;
        p_proc->hdl = GATT_INVALID_HDL;
    }
    else
    {
        // store received information
        p_proc->offset     = 0;
        p_proc->rsp_remain_len = mtu - L2CAP_ATT_HEADER_LEN;
        p_proc->pdu_code   = p_pdu->code;

        p_proc->p_pdu_buf = p_buf;
        co_buf_acquire(p_buf);

        if(status == GAP_ERR_NO_ERROR)
        {
            p_proc->hdl = gatt_srv_read_next_hdl_get(p_proc);
        }
    }

    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, (gatt_proc_t*) p_proc);


    // check if client cache is out of sync
    if(gapc_svc_is_cli_out_of_sync(conidx, p_pdu->code, p_proc->hdl))
    {
        status = ATT_ERR_DB_OUT_OF_SYNC;
    }

    return (status);
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
__STATIC void gatt_srv_read_proc_continue(uint8_t conidx, gatt_srv_read_proc_t* p_proc, uint8_t proc_state,
                                          uint16_t status)
{
    bool finished = false;
    bool send_rsp = true;

    switch(proc_state)
    {
        case GATT_PROC_USER_CFM:
        {
            // check if procedure is over
            if((status != GAP_ERR_NO_ERROR) || (p_proc->p_pdu_buf == NULL))
            {
                break;
            }

            // get next handle to read
            p_proc->hdl = gatt_srv_read_next_hdl_get(p_proc);
        }
        // no break
        case GATT_PROC_PDU_RX:
        {
            // check if procedure is over
            if(status != GAP_ERR_NO_ERROR) { break; }

            while(p_proc->rsp_remain_len > 0)
            {
                gatt_db_svc_t* p_svc;
                gatt_db_att_t* p_att;

                status = gatt_db_att_get(p_proc->hdl, &p_svc, &p_att);

                if(status == GAP_ERR_NO_ERROR)
                {
                    status = gatt_db_att_access_check(conidx, GATT_DB_ACCESS_READ, p_svc, p_att);
                }

                if(status != GAP_ERR_NO_ERROR) { break; }

                // check Native supported attribute types - Data is present in database
                if(   (GETF(p_att->perm, GATT_ATT_UUID_TYPE) == GATT_UUID_16)
                   && (   (p_att->uuid == GATT_DECL_PRIMARY_SERVICE)
                       || (p_att->uuid == GATT_DECL_SECONDARY_SERVICE)
                       || (p_att->uuid == GATT_DECL_CHARACTERISTIC)
                       || (p_att->uuid == GATT_DECL_INCLUDE)
                       || (p_att->uuid == GATT_DESC_CHAR_EXT_PROPERTIES)))
                {
                    co_buf_t* p_buf;
                    uint16_t  data_length = 0;
                    uint16_t  val_length;
                    uint8_t   val[GATT_DB_ATT_MAX_NATIVE_VAL_LEN];

                    // retrieve native value
                    status = gatt_db_att_native_val_get(p_proc->hdl, p_svc, p_att, val, &val_length);
                    if(status != GAP_ERR_NO_ERROR) break;

                    // check value offset requested
                    if(p_proc->offset > val_length)
                    {
                        status = ATT_ERR_INVALID_OFFSET;
                        break;
                    }

                    // value length present in response
                    if(p_proc->pdu_code == L2CAP_ATT_RD_MULT_VAR_REQ_OPCODE)
                    {
                        data_length = L2CAP_ATT_VALLEN_LEN;
                    }

                    data_length = co_min(data_length + val_length, p_proc->rsp_remain_len);

                    p_buf = (co_buf_t*) co_list_tail(&(p_proc->buf_queue));

                    // check if buffer data can be copied in existing buffer
                    if((p_buf == NULL)  || (co_buf_tail_len(p_buf) < L2CAP_ATT_VALLEN_LEN)
                                        || (co_buf_tail_len(p_buf) < data_length))
                    {
                        // allocate a new buffer for native value
                        if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0,
                                         GATT_DB_ATT_MAX_NATIVE_VAL_LEN + L2CAP_ATT_VALLEN_LEN) != CO_BUF_ERR_NO_ERROR)
                        {
                            status = GAP_ERR_INSUFF_RESOURCES;
                            break;
                        }

                        co_list_push_back(&(p_proc->buf_queue), &(p_buf->hdr));
                    }

                    p_proc->rsp_len        += data_length;
                    p_proc->rsp_remain_len -= data_length;


                    // value length present in response
                    if(p_proc->pdu_code == L2CAP_ATT_RD_MULT_VAR_REQ_OPCODE)
                    {
                        co_write16p(co_buf_tail(p_buf), co_htobs(val_length));

                        if(data_length >= L2CAP_ATT_VALLEN_LEN)
                        {
                            co_buf_tail_reserve(p_buf, L2CAP_ATT_VALLEN_LEN);
                            data_length -= L2CAP_ATT_VALLEN_LEN;
                        }
                        else
                        {
                            co_buf_tail_reserve(p_buf, data_length);
                            data_length = 0;
                        }
                    }

                    // copy data into buffer
                    memcpy(co_buf_tail(p_buf), val, data_length);
                    co_buf_tail_reserve(p_buf, data_length);
                }
                // Ask GATT user to provide value information
                else
                {
                    // retrieve user info
                    gatt_user_t* p_user = gatt_user_get(p_svc->user_lid);
                    p_proc->hdr.user_lid = p_svc->user_lid;
                    uint16_t max_value_len = p_proc->rsp_remain_len;

                    // update maximum length for read multiple
                    if(p_proc->pdu_code == L2CAP_ATT_RD_MULT_VAR_REQ_OPCODE)
                    {
                        if(max_value_len > L2CAP_ATT_VALLEN_LEN)
                        {
                            max_value_len -= L2CAP_ATT_VALLEN_LEN;
                        }
                        else
                        {
                            max_value_len  = 0;
                        }
                    }

                    if(p_user == NULL)
                    {
                        ASSERT_ERR(0);
                        status = ATT_ERR_INSUFF_AUTHOR;
                        break;
                    }

                    if((p_proc->offset > 0) && GETB(p_att->ext_info, GATT_ATT_NO_OFFSET))
                    {
                        status = ATT_ERR_INVALID_OFFSET; // read with an offset not authorized
                        break;
                    }

                    SETB(p_proc->hdr.info_bf, GATT_PROC_USER_CFM, false);
                    p_user->p_cb->srv.cb_att_read_get(conidx, p_svc->user_lid, p_proc->hdr.token,
                                                      p_proc->hdl, p_proc->offset, max_value_len);
                    send_rsp = false;
                    break;
                }

                // check if all attribute value has been read
                if(p_proc->p_pdu_buf == NULL) break;

                // get next handle to read
                p_proc->hdl = gatt_srv_read_next_hdl_get(p_proc);
            }
        } break;

        case GATT_PROC_PDU_PUSHED_TO_LL:    // Confirmation properly pushed in TX LL buffers
        case GATT_PROC_ERROR:               // Handle error detection
        {
            finished = true;
            send_rsp = false;
        } break;

        default: { /*  Nothing to do */  } break;
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        finished = true;
    }

    if(send_rsp)
    {
        // Send read response / error responses to peer device
        l2cap_att_pdu_t pdu;
        co_buf_t* p_buf = NULL;

        if(status == GAP_ERR_NO_ERROR)
        {
            uint16_t rsp_tail_len;

            // get fist buffer from queue
            p_buf = (co_buf_t*) co_list_pop_front(&(p_proc->buf_queue));
            ASSERT_ERR(p_buf != NULL);

            rsp_tail_len = p_proc->rsp_len - co_buf_data_len(p_buf) + GATT_BUFFER_TAIL_LEN;

            // if buffer cannot contain full response, allocate a new buffer.
            if(rsp_tail_len > co_buf_tail_len(p_buf))
            {
                co_buf_t* p_out_buf;

                if(co_buf_duplicate(p_buf, &p_out_buf, GATT_BUFFER_HEADER_LEN, rsp_tail_len) != CO_BUF_ERR_NO_ERROR)
                {
                    status = ATT_ERR_INSUFF_RESOURCE;
                }
                else
                {
                    co_buf_release(p_buf);
                    p_buf = p_out_buf;
                }
            }
        }

        if(status != GAP_ERR_NO_ERROR)
        {
            // allocate buffer used for confirmation transmission
            if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
            {
                pdu.code            = L2CAP_ATT_ERR_RSP_OPCODE;
                pdu.err_rsp.op_code = p_proc->pdu_code;
                pdu.err_rsp.handle  = p_proc->hdl;
                pdu.err_rsp.reason  = status;
            }
        }
        else
        {
            // copy remaining data onto the PDU
            pdu.code = (p_proc->pdu_code + 1);

            while(!co_list_is_empty(&(p_proc->buf_queue)))
            {
                co_buf_t* p_in_buf = (co_buf_t*) co_list_pop_front(&(p_proc->buf_queue));
                uint16_t  data_len = co_buf_data_len(p_in_buf);

                co_buf_copy_data_to_mem(p_in_buf, co_buf_tail(p_buf), data_len);
                co_buf_tail_reserve(p_buf, data_len);

                co_buf_release(p_in_buf);
            }
        }

        if(p_buf != NULL)
        {
            // Ask for PDU transmission
            status = gatt_proc_pdu_send(conidx, (gatt_proc_t*) p_proc, &pdu, p_buf, NULL);
            co_buf_release(p_buf);
        }
        else
        {
            status = GAP_ERR_UNEXPECTED;
        }

        if(status != GAP_ERR_NO_ERROR)
        {
            finished = true;
        }
    }

    if(finished)
    {
        // check if PDU buffer must be released
        if(p_proc->p_pdu_buf != NULL)
        {
            co_buf_release(p_proc->p_pdu_buf);
        }

        // check if buffers must be released
        while(!co_list_is_empty(&(p_proc->buf_queue)))
        {
            co_buf_release((co_buf_t*) co_list_pop_front(&(p_proc->buf_queue)));
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
 * @brief Ask GATT Server module to create a Read procedure handler
 *
 * @param[in]  conidx       Connection index
 * @param[in]  op_code      Attribute operation code received
 * @param[out] p_pdu_hdl_cb Pointer to PDU Handler call-back
 * @param[out] pp_proc      Pointer to the procedure created, NULL if procedure not created
 *
 * @return Procedure creation status code (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gatt_srv_read_proc_create(uint8_t conidx, uint8_t op_code, gatt_proc_pdu_handler_cb* p_pdu_hdl_cb,
                                   gatt_proc_handler_t** pp_proc)
{
    uint16_t status;

     switch(op_code)
    {
        case L2CAP_ATT_RD_REQ_OPCODE:
        case L2CAP_ATT_RD_BLOB_REQ_OPCODE:
        {
            *p_pdu_hdl_cb = (gatt_proc_pdu_handler_cb) gatt_srv_read_l2cap_att_rd_req_handler;
        } break;
        case L2CAP_ATT_RD_MULT_REQ_OPCODE:
        case L2CAP_ATT_RD_MULT_VAR_REQ_OPCODE:
        {
            *p_pdu_hdl_cb = (gatt_proc_pdu_handler_cb) gatt_srv_read_l2cap_att_rd_mult_req_handler;
        } break;
        default: { ASSERT_ERR(0); } break;
    }

    // Allocate procedure
    status = gatt_proc_handler_alloc(conidx, GATT_PROC_HANDLE_READ, sizeof(gatt_srv_read_proc_t),
                                     (gatt_proc_cb) gatt_srv_read_proc_continue, pp_proc);


    if(status == GAP_ERR_NO_ERROR)
    {
        gatt_srv_read_proc_t* p_proc = (gatt_srv_read_proc_t*) *pp_proc;

        p_proc->p_pdu_buf = NULL;
        p_proc->rsp_len   = 0;
        co_list_init(&(p_proc->buf_queue));
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Upper layer provide attribute value requested by GATT Layer, if rejected,
 *        value is not used.
 *
 * @param[in]  conidx       Connection index
 * @param[in]  p_proc       Pointer to on-going procedure
 * @param[in]  status       Status of attribute value get (see enum #hl_err)
 * @param[in]  att_length   Complete Length of the attribute value
 * @param[in]  p_data       Pointer to buffer that contains attribute data (starting from offset)
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gatt_srv_read_att_val_get_cfm(uint8_t conidx, gatt_srv_read_proc_t* p_proc, uint16_t status,
                                       uint16_t att_length, co_buf_t* p_data)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        uint16_t data_len;
        co_buf_t* p_buf = (co_buf_t*) co_list_tail(&(p_proc->buf_queue));

        // put buffer length if front for a read multiple
        if(p_proc->pdu_code == L2CAP_ATT_RD_MULT_VAR_REQ_OPCODE)
        {
            co_buf_head_reserve(p_data, L2CAP_ATT_VALLEN_LEN);
            co_write16p(co_buf_data(p_data), co_htobs(att_length));
        }

        data_len = co_buf_data_len(p_data);
        data_len = co_min(data_len, p_proc->rsp_remain_len);

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

        p_proc->rsp_len        += data_len;
        p_proc->rsp_remain_len -= data_len;
    }

    // continue procedure execution
    gatt_proc_continue(conidx, (gatt_proc_t*) p_proc, GATT_PROC_USER_CFM, status);

    status = GAP_ERR_NO_ERROR;

    return (status);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

// handler of confirmation present in gatt_srv_discover.c
extern uint16_t gatt_srv_discover_att_val_get_cfm(uint8_t conidx, gatt_proc_handler_t* p_proc, uint16_t status, co_buf_t* p_data);


uint16_t gatt_srv_att_read_get_cfm(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t status,
                                   uint16_t att_length, co_buf_t* p_data)
{
    gatt_proc_handler_t* p_proc = (gatt_proc_handler_t*) gatt_proc_pick(conidx, token);

    do
    {
        // check if procedure is found
        if((p_proc == NULL) || (user_lid != p_proc->user_lid))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }
        // check if confirm not already received
        if (GETB(p_proc->info_bf, GATT_PROC_USER_CFM))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }
        if(status == GAP_ERR_NO_ERROR)
        {
            // buffer can be NULL only if status is failing
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
        }

        if(p_proc->proc_id == GATT_PROC_HANDLE_READ)
        {
            SETB(p_proc->info_bf, GATT_PROC_USER_CFM, true);
            p_proc->user_lid      = GATT_INVALID_USER_LID;
            status = gatt_srv_read_att_val_get_cfm(conidx, (gatt_srv_read_proc_t*) p_proc, status, att_length, p_data);
        }
        else if (p_proc->proc_id == GATT_PROC_HANDLE_DISCOVER)
        {
            SETB(p_proc->info_bf, GATT_PROC_USER_CFM, true);
            p_proc->user_lid      = GATT_INVALID_USER_LID;
            status = gatt_srv_discover_att_val_get_cfm(conidx, p_proc, status, p_data);
        }
        else
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
        }
    } while(0);

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
 * @brief This function is called when peer want to read local attribute database value.
 *
 *        @see gatt_srv_att_read_get_cfm shall be called to provide attribute value
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] hdl           Attribute handle
 * @param[in] offset        Data offset
 * @param[in] max_length    Maximum data length to return
 ****************************************************************************************
 */
void gatt_srv_att_read_get_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset, uint16_t max_length)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        gatt_srv_att_read_get_req_ind_t* p_req_ind = KE_MSG_ALLOC(GATT_REQ_IND, p_user->dest_task_nbr, TASK_GATT,
                                                                  gatt_srv_att_read_get_req_ind);

        // prepare request indication to send
        if(p_req_ind != NULL)
        {
            p_req_ind->req_ind_code  = GATT_SRV_ATT_READ_GET;
            p_req_ind->token         = token;
            p_req_ind->user_lid      = user_lid;
            p_req_ind->conidx        = conidx;
            p_req_ind->hdl           = hdl;
            p_req_ind->offset        = offset;
            p_req_ind->max_length    = max_length;

            // send message to host
            ke_msg_send(p_req_ind);
        }
    }
    else
    {
        gatt_srv_att_read_get_cfm(conidx, user_lid, token, ATT_ERR_UNLIKELY_ERR, 0, NULL);
    }
}

/**
 ****************************************************************************************
 * @brief Handle confirmation of read value to send to a peer device.
 *
 * @param[in] p_cfm     Pointer to confirmation parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_srv_att_read_get_cfm_handler(gatt_srv_att_read_get_cfm_t* p_cfm, uint16_t src_id)
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
    gatt_srv_att_read_get_cfm(p_cfm->conidx, p_cfm->user_lid, p_cfm->token, p_cfm->status, p_cfm->att_length, p_data);

    // release buffer
    if(p_data != NULL)
    {
        co_buf_release(p_data);
    }
}

#endif // (HOST_MSG_API)

#endif // (BLE_GATT)
/// @} GATT

