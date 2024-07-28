/**
 ****************************************************************************************
 * @file gatt_srv_read.c
 *
 * @brief  GATT Server Database Discovery Procedures
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
#include "co_math.h"        // for Min/Max usage

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

/// Server discover procedure structure.
typedef struct gatt_srv_discover_proc
{
    /// Procedure header - required for any Attribute procedure
    gatt_proc_handler_t     hdr;
    /// Pointer to the searched value
    co_buf_t*               p_val_buf;
    /// Pointer to the buffer used for response
    co_buf_t*               p_rsp_buf;
    /// Search start handle
    uint16_t                start_hdl;
    /// Seatch end handle
    uint16_t                end_hdl;
    /// Found handle
    uint16_t                hdl;
    /// Found End Group handle
    uint16_t                end_grp_hdl;
    /// Maximum size for the response
    uint16_t                max_rsp_len;
    /// ATT Operation code received
    uint8_t                 pdu_code;
    /// Searched UUID Type (@enum gatt_uuid_type)
    uint8_t                 uuid_type;
    /// Searched UUID
    uint8_t                 uuid[GATT_UUID_128_LEN];
} gatt_srv_discover_proc_t;

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
 * @brief Function called when L2CAP_ATT_FIND_INFO_REQ, L2CAP_ATT_FIND_BY_TYPE_REQ,
 *        L2CAP_ATT_RD_BY_TYPE_REQ, L2CAP_ATT_RD_BY_GRP_TYPE_REQ attribute PDU is received.
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
__STATIC uint16_t gatt_srv_discover_l2cap_att_find_req_handler(uint8_t conidx, gatt_srv_discover_proc_t* p_proc,
                                                               l2cap_att_find_info_req_t* p_pdu, co_buf_t* p_buf, uint16_t mtu)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    uint16_t data_len = co_buf_data_len(p_buf);

    // store received information
    p_proc->start_hdl = p_pdu->shdl;
    p_proc->end_hdl   = p_pdu->ehdl;
    p_proc->pdu_code  = p_pdu->code;
    p_proc->p_rsp_buf = NULL;
    p_proc->p_val_buf = NULL;
    p_proc->hdl       = p_pdu->shdl;
    p_proc->uuid_type = GATT_UUID_INVALID;

    if(( p_proc->start_hdl >  p_proc->end_hdl) || (p_proc->start_hdl == GATT_INVALID_HDL))
    {
        status = ATT_ERR_INVALID_HANDLE;
    }
    else
    {
        // check code
        switch(p_pdu->code)
        {
            case L2CAP_ATT_RD_BY_TYPE_REQ_OPCODE:
            case L2CAP_ATT_RD_BY_GRP_TYPE_REQ_OPCODE:
            {
                p_proc->max_rsp_len       = mtu - L2CAP_ATT_HEADER_LEN - L2CAP_ATT_EACHLEN_LEN;
                // extract searched UUID
                if((data_len == GATT_UUID_128_LEN) || (data_len == GATT_UUID_16_LEN))
                {
                    gatt_uuid_extract(p_proc->uuid, &(p_proc->uuid_type), co_buf_data(p_buf), data_len);
                }
                else
                {
                    status = ATT_ERR_INVALID_PDU;
                }
            } break;
            case L2CAP_ATT_FIND_INFO_REQ_OPCODE:
            {
                p_proc->max_rsp_len       = mtu - L2CAP_ATT_HEADER_LEN - L2CAP_ATT_FORMAT_LEN;
                // just verify that there is nothing more to read
                if(co_buf_data_len(p_buf) != 0)
                {
                    status = ATT_ERR_INVALID_PDU;
                }
            } break;
            case L2CAP_ATT_FIND_BY_TYPE_REQ_OPCODE:
            {
                p_proc->max_rsp_len       = mtu - L2CAP_ATT_HEADER_LEN;

                // ensure that there is at least the 16 bit UUID searched
                if((data_len >= GATT_UUID_16_LEN))
                {
                    memcpy(p_proc->uuid, co_buf_data(p_buf), GATT_UUID_16_LEN);
                    co_buf_head_release(p_buf, GATT_UUID_16_LEN);
                    p_proc->uuid_type = GATT_UUID_16;
                    p_proc->p_val_buf = p_buf;
                    co_buf_acquire(p_buf);
                }
                else
                {
                    status = ATT_ERR_INVALID_PDU;
                }
            } break;
            default: { ASSERT_ERR(0); } break;
        }
    }

    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, (gatt_proc_t*) p_proc);

    // Update client cache status
    gapc_svc_is_cli_out_of_sync(conidx, p_pdu->code, GATT_INVALID_HDL);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Retrieve Native attribute data value (handled by attribute database) and prepare
 *        value for response to send
 *
 * @param[in]  p_proc      Pointer to on-going procedure
 * @param[in]  hdl         Handle of the attribute
 * @param[in]  end_grp_hdl Value of the End Group handle
 * @param[in]  p_svc       Pointer to service
 * @param[in]  p_att       Pointer to attribute
 * @param[out] p_out       Pointer to the data buffer where value must be pushed
 * @param[out] p_length    Pointer where value length is set
 *
 * @return @ref GAP_ERR_NO_ERROR if access is permitted, otherwise the ATT error code.
 ****************************************************************************************
 */
__STATIC uint16_t gatt_srv_discover_rsp_val_prepare(gatt_srv_discover_proc_t* p_proc, uint16_t hdl, uint16_t end_grp_hdl,
                                                    gatt_db_svc_t* p_svc, gatt_db_att_t* p_att,
                                                    uint8_t* p_out, uint16_t* p_length)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    // prepare response value
    if(p_proc->pdu_code == L2CAP_ATT_FIND_INFO_REQ_OPCODE)
    {
        *p_length = GATT_HANDLE_LEN;

        co_write16p(p_out, co_htobs(hdl));
        p_out += GATT_HANDLE_LEN;

        if(GETF(p_att->perm, GATT_ATT_UUID_TYPE) == GATT_UUID_16)
        {
            co_write16p(p_out, p_att->uuid);
            *p_length += GATT_UUID_16_LEN;
        }
        else
        {
            gatt_uuid128_convert(((uint8_t*)p_svc) + p_att->uuid, GETF(p_att->perm, GATT_ATT_UUID_TYPE), p_out);
            *p_length += GATT_UUID_128_LEN;
        }
    }
    else
    {
        uint16_t val_length;
        if(p_proc->pdu_code == L2CAP_ATT_FIND_BY_TYPE_REQ_OPCODE)
        {
            *p_length = 0;
        }
        else if(p_proc->pdu_code == L2CAP_ATT_RD_BY_TYPE_REQ_OPCODE)
        {
            co_write16p(p_out, co_htobs(hdl));
            p_out += GATT_HANDLE_LEN;
            *p_length = GATT_HANDLE_LEN;
        }
        else // (p_proc->op_code ==L2CAP_ATT_RD_BY_GRP_TYPE_REQ_OPCODE)
        {
            co_write16p(p_out, co_htobs(hdl));
            p_out += GATT_HANDLE_LEN;
            co_write16p(p_out, co_htobs(end_grp_hdl));
            p_out += GATT_HANDLE_LEN;
            *p_length = GATT_HANDLE_LEN * 2;
        }

        // retrieve native value
        status = gatt_db_att_native_val_get(p_proc->hdl, p_svc, p_att, p_out, &val_length);

        if(status == GAP_ERR_NO_ERROR)
        {
            *p_length += val_length;
        }
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
__STATIC void gatt_srv_discover_proc_continue(uint8_t conidx, gatt_srv_discover_proc_t* p_proc, uint8_t proc_state,
                                           uint16_t status)
{
    bool finished = false;
    bool send_rsp = true;
    uint16_t each_len = 0;

    switch(proc_state)
    {
        case GATT_PROC_USER_CFM:
        {
            // Check if a response can be sent
            if(status != GAP_ERR_NO_ERROR)
            {
                break;
            }
            if(p_proc->p_rsp_buf != NULL)
            {
                each_len = co_buf_data_len(p_proc->p_rsp_buf);
                break;
            }
            // else continue search
        }
        // no break
        case GATT_PROC_PDU_RX:
        {
            if(status != GAP_ERR_NO_ERROR) break;

            gatt_db_svc_t* p_svc;
            gatt_db_att_t* p_att;
            uint16_t remain_len = p_proc->max_rsp_len;

            while(remain_len > each_len)
            {
                // search first valid attribute
                status = gatt_db_att_find(p_proc->start_hdl, p_proc->end_hdl,
                                          (p_proc->pdu_code != L2CAP_ATT_FIND_INFO_REQ_OPCODE),
                                          p_proc->uuid_type, p_proc->uuid,
                                          &p_proc->hdl, &p_proc->end_grp_hdl, &p_svc, &p_att);

                if(status != GAP_ERR_NO_ERROR) break;

                // check if value/information can be retrieved from local database
                if(   (p_proc->pdu_code == L2CAP_ATT_FIND_INFO_REQ_OPCODE)
                   || (   (GETF(p_att->perm, GATT_ATT_UUID_TYPE) == GATT_UUID_16)
                       && (   (p_att->uuid == GATT_DECL_PRIMARY_SERVICE) || (p_att->uuid == GATT_DECL_SECONDARY_SERVICE)
                           || (p_att->uuid == GATT_DECL_CHARACTERISTIC)  || (p_att->uuid == GATT_DECL_INCLUDE)
                           || (p_att->uuid == GATT_DESC_CHAR_EXT_PROPERTIES))))
                {
                    uint16_t  val_len;
                    uint8_t   val[GATT_DB_ATT_MAX_NATIVE_VAL_LEN + (GATT_HANDLE_LEN * 2)];

                    // prepare the response value
                    status = gatt_srv_discover_rsp_val_prepare(p_proc, p_proc->hdl, p_proc->end_grp_hdl,
                                                               p_svc, p_att, val, &val_len);

                    if(p_proc->pdu_code == L2CAP_ATT_FIND_BY_TYPE_REQ_OPCODE)
                    {
                        // check if read value is equals to value to compare
                        if(   (val_len != co_buf_data_len(p_proc->p_val_buf))
                           || (memcmp(val, co_buf_data(p_proc->p_val_buf), val_len) != 0))
                        {
                            // continue search
                            p_proc->start_hdl = p_proc->end_grp_hdl + 1;
                            continue;
                        }
                        else
                        {
                            co_write16p(val,                   co_htobs(p_proc->hdl));
                            co_write16p(val + GATT_HANDLE_LEN, co_htobs(p_proc->end_grp_hdl));
                            val_len = GATT_HANDLE_LEN * 2;
                        }
                    }

                    // ensure that value length does not exceed MTU
                    val_len =  co_min(p_proc->max_rsp_len, val_len);

                    // allocate buffer
                    if(p_proc->p_rsp_buf == NULL)
                    {
                        if(co_buf_alloc(&(p_proc->p_rsp_buf), GATT_BUFFER_HEADER_LEN, 0,
                                        co_min(p_proc->max_rsp_len, CO_BUF_SMALL_SIZE - GATT_BUFFER_HEADER_LEN))
                                != CO_BUF_ERR_NO_ERROR)
                        {
                            status = ATT_ERR_INSUFF_RESOURCE;
                            break;
                        }

                        each_len = val_len;
                        // update remain length according to buffer length
                        remain_len = co_buf_tail_len(p_proc->p_rsp_buf);
                        remain_len = co_min(remain_len, p_proc->max_rsp_len);
                    }

                    // check if data can be copied
                    if(each_len == val_len)
                    {
                        memcpy(co_buf_tail(p_proc->p_rsp_buf), val, val_len);
                        co_buf_tail_reserve(p_proc->p_rsp_buf, val_len);
                        remain_len -= val_len;

                        // update search start handle
                        if(p_proc->pdu_code == L2CAP_ATT_FIND_INFO_REQ_OPCODE)
                        {
                             p_proc->start_hdl = p_proc->hdl + 1;
                        }
                        else
                        {
                             p_proc->start_hdl = p_proc->end_grp_hdl + 1;
                        }
                    }
                    else // search is over
                    {
                        break;
                    }
                }
                // ask GATT user to provide value
                else // value is required
                {
                    gatt_user_t* p_user;

                    // Check if value can be read
                    status = gatt_db_att_access_check(conidx, GATT_DB_ACCESS_READ, p_svc, p_att);

                    if(status != GAP_ERR_NO_ERROR) break;

                    // retrieve GATT User
                    p_user = gatt_user_get(p_svc->user_lid);

                    if(p_user == NULL)
                    {
                        status = ATT_ERR_INSUFF_AUTHOR;
                    }
                    else
                    {
                        uint16_t rsp_len = p_proc->max_rsp_len - GATT_HANDLE_LEN;
                        if(p_proc->pdu_code == L2CAP_ATT_RD_BY_GRP_TYPE_REQ_OPCODE)
                        {
                            rsp_len -= p_proc->max_rsp_len;
                        }

                        // ask GATT user to provide value
                        p_proc->hdr.user_lid      = p_svc->user_lid;
                        SETB(p_proc->hdr.info_bf, GATT_PROC_USER_CFM, false);
                        p_user->p_cb->srv.cb_att_read_get(conidx, p_svc->user_lid, p_proc->hdr.token, p_proc->hdl, 0,
                                                          rsp_len);
                        send_rsp = false;
                    }
                    break;
                }
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


    if(send_rsp)
    {
        l2cap_att_pdu_t pdu;
        co_buf_t* p_buf = NULL;

        // remove value buffer
        if(p_proc->p_val_buf != NULL)
        {
            co_buf_release(p_proc->p_val_buf);
            p_proc->p_val_buf = NULL;
        }

        // check if a response  can be sent
        if(p_proc->p_rsp_buf != NULL)
        {
            // send back the response
            p_buf    = p_proc->p_rsp_buf;
            pdu.code = (p_proc->pdu_code + 1);

            switch(p_proc->pdu_code)
            {
                case L2CAP_ATT_FIND_INFO_REQ_OPCODE:
                {
                    pdu.find_info_rsp.format = ((each_len - GATT_HANDLE_LEN) == GATT_UUID_16_LEN)
                                             ? L2CAP_ATT_UUID_16_FORMAT
                                             : L2CAP_ATT_UUID_128_FORMAT;
                } break;
                case L2CAP_ATT_RD_BY_TYPE_REQ_OPCODE:     { pdu.rd_by_type_rsp.each_len     = each_len; } break;
                case L2CAP_ATT_RD_BY_GRP_TYPE_REQ_OPCODE: { pdu.rd_by_grp_type_rsp.each_len = each_len; } break;
                default: { /* Nothing to do */ } break;
            }

            p_proc->p_rsp_buf = NULL;
        }
        else
        {
            // allocate buffer used for confirmation transmission
            if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
            {
                // send error response
                pdu.code            = L2CAP_ATT_ERR_RSP_OPCODE;
                pdu.err_rsp.op_code = p_proc->pdu_code;
                pdu.err_rsp.handle  = p_proc->hdl;
                pdu.err_rsp.reason  = status;
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

    if(status != GAP_ERR_NO_ERROR)
    {
        finished = true;
    }

    if(finished)
    {
        if(p_proc->p_val_buf != NULL)
        {
            co_buf_release(p_proc->p_val_buf);
        }

        if(p_proc->p_rsp_buf != NULL)
        {
            co_buf_release(p_proc->p_rsp_buf);
            p_proc->p_rsp_buf = NULL;
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
uint16_t gatt_srv_discover_proc_create(uint8_t conidx, uint8_t op_code, gatt_proc_pdu_handler_cb* p_pdu_hdl_cb,
                                       gatt_proc_handler_t** pp_proc)
{
    uint16_t status;

    // Allocate procedure
    status = gatt_proc_handler_alloc(conidx, GATT_PROC_HANDLE_DISCOVER, sizeof(gatt_srv_discover_proc_t),
                                     (gatt_proc_cb) gatt_srv_discover_proc_continue, pp_proc);

    if(status == GAP_ERR_NO_ERROR)
    {
        gatt_srv_discover_proc_t* p_proc = (gatt_srv_discover_proc_t*)*pp_proc;
        p_proc->p_val_buf = NULL;
        p_proc->p_rsp_buf = NULL;
    }

    *p_pdu_hdl_cb = (gatt_proc_pdu_handler_cb) gatt_srv_discover_l2cap_att_find_req_handler;

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
 * @param[in]  p_data       Pointer to buffer that contains attribute data (starting from offset)
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gatt_srv_discover_att_val_get_cfm(uint8_t conidx, gatt_srv_discover_proc_t* p_proc, uint16_t status,
                                           co_buf_t* p_data)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        uint16_t data_len = co_buf_data_len(p_data);

        if(co_buf_head_len(p_data) < GATT_BUFFER_HEADER_LEN)
        {
            status = ATT_ERR_UNLIKELY_ERR;
        }
        else if(p_proc->pdu_code == L2CAP_ATT_FIND_BY_TYPE_REQ_OPCODE)
        {
            // check if read value is equals to value to compare
            if(   (data_len != co_buf_data_len(p_proc->p_val_buf))
               || (memcmp(co_buf_data(p_data), co_buf_data(p_proc->p_val_buf), data_len) != 0))
            {
                // continue search
                p_proc->start_hdl = p_proc->end_grp_hdl + 1;
            }
            else
            {
                // reuse buffer for response but don't use data
                co_buf_tail_release(p_data, data_len);
                p_proc->p_rsp_buf = p_data;
            }
        }
        else
        {
            p_proc->p_rsp_buf = p_data;
        }

        if(p_proc->p_rsp_buf != NULL)
        {
            // set end group handle data
            if(p_proc->pdu_code != L2CAP_ATT_RD_BY_TYPE_REQ_OPCODE)
            {
                co_buf_head_reserve(p_data, GATT_HANDLE_LEN);
                co_write16p(co_buf_data(p_data), co_htobs(p_proc->end_grp_hdl));
            }

            // set attribute handle
            co_buf_head_reserve(p_data, GATT_HANDLE_LEN);
            co_write16p(co_buf_data(p_data), co_htobs(p_proc->hdl));

            // ensure that response is not too big
            if(co_buf_data_len(p_data) > p_proc->max_rsp_len)
            {
                co_buf_tail_release(p_data, co_buf_data_len(p_data) - p_proc->max_rsp_len);
            }

            co_buf_acquire(p_data);
        }
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

#endif // (BLE_GATT)
/// @} GATT

