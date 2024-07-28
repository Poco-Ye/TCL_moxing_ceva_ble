/**
 ****************************************************************************************
 * @file gatt_srv_write.c
 *
 * @brief  GATT Server Write Procedures
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
#include "co_math.h"        // Min / Max usage
#include "aes.h"            // for aes_cmac usage

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

/// Bit field used to retrieve a procedure during AES-CMAC operation
enum gatt_srv_write_info_bf
{
    /// Connection Index
    GATT_SRV_WRITE_CONIDX_MASK      = 0x000000FF,
    GATT_SRV_WRITE_CONIDX_LSB       = 0,
    /// Procedure token
    GATT_SRV_WRITE_TOKEN_MASK       = 0x00FFFF00,
    GATT_SRV_WRITE_TOKEN_LSB        = 8,
};
/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Server write procedure structure.
typedef struct gatt_srv_write_proc
{
    /// Procedure header - required for any Attribute procedure
    gatt_proc_handler_t     hdr;
    /// Pointer to current buffer
    co_buf_t*               p_buf;
    /// Pointer to buffer in prepare queue list where new buffer should be inserted after
    co_buf_t*               p_prev_prep_buf;
    /// Attribute handle targeted.
    uint16_t                hdl;
    /// offset of the attribute value
    uint16_t                offset;
    /// Current Attribute value length
    uint16_t                att_val_len;
    /// Current Attribute maximum value length
    uint16_t                att_max_len;
    /// Write access type
    uint8_t                 access;
    /// Local Service User identifier
    uint8_t                 svc_user_lid;
    /// On prepare write, push data of same handle in same buffer
    bool                    append_data;
    /// Used to know if write a value with offset is authorized or not
    bool                    write_with_offset_authz;
} gatt_srv_write_proc_t;


/// Server execute write procedure structure.
typedef struct gatt_srv_write_exe_proc
{
    /// Procedure header - required for any Attribute procedure
    gatt_proc_handler_t     hdr;
    /// write execution request flag
    uint8_t                 exe_flag;
    /// Attribute handle targeted.
    uint16_t                hdl;
} gatt_srv_write_exe_proc_t;


/// Buffer meta-data value for prepare write
typedef struct gatt_srv_write_prep_meta
{
    /// Attribute handle
    uint16_t               hdl;
    /// Value offset
    uint16_t               offset;
    /// Current Attribute value length
    uint16_t               att_val_len;
    /// Current Attribute maximum value length
    uint16_t               att_max_len;
    /// Total value length
    uint16_t               total_len;
    /// User local identifier targeted
    uint8_t                svc_user_lid;
} gatt_srv_write_prep_meta_t;
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
 * @brief Function called when L2CAP_ATT_WR_REQ or L2CAP_ATT_PREP_WR_REQ attribute PDU is received.
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
__STATIC uint16_t gatt_srv_write_l2cap_att_wr_req_handler(uint8_t conidx, gatt_srv_write_proc_t* p_proc,
                                                          l2cap_att_prep_wr_req_t* p_pdu, co_buf_t* p_buf, uint16_t mtu)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    // store received information
    p_proc->hdl      = p_pdu->handle;
    p_proc->p_buf    = p_buf;
    p_proc->offset   = (p_proc->hdr.proc_id == GATT_PROC_HANDLE_PREP_WRITE) ? p_pdu->offset : 0;

    // for a write signed
    if(p_proc->access == GATT_DB_ACCESS_WRITE_SIGNED)
    {
        // check data contains signature
        if(co_buf_data_len(p_buf) < L2CAP_ATT_SIGN_LEN)
        {
            status = ATT_ERR_INVALID_PDU;
        }
        else
        {
            co_buf_tail_release(p_buf, L2CAP_ATT_SIGN_LEN);
        }
    }

    // acquire buffer
    co_buf_acquire(p_buf);

    // check if client cache is out of sync
    if(gapc_svc_is_cli_out_of_sync(conidx, p_pdu->code, p_proc->hdl))
    {
        status = ATT_ERR_DB_OUT_OF_SYNC;
    }

    if(p_proc->access == GATT_DB_ACCESS_WRITE)
    {
        // mark that reception on bearer can continue
        gatt_proc_bearer_rx_continue(conidx, (gatt_proc_t*) p_proc);
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Function called when L2CAP_ATT_EXE_WR_REQ attribute PDU is received.
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
__STATIC uint16_t gatt_srv_write_l2cap_att_exe_wr_req_handler(uint8_t conidx, gatt_srv_write_exe_proc_t* p_proc,
                                                              l2cap_att_exe_wr_req_t* p_pdu, co_buf_t* p_buf, uint16_t mtu)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    // store received information
    p_proc->exe_flag = p_pdu->flags;

    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, (gatt_proc_t*) p_proc);

    // Update client cache status
    gapc_svc_is_cli_out_of_sync(conidx, p_pdu->code, GATT_INVALID_HDL);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Call back definition of the function that can handle result of an AES based algorithm
 *
 * @param[in] aes_status   Execution status
 * @param[in] aes_res      16 bytes block result
 * @param[in] src_info     Information provided by requester
 ****************************************************************************************
 */
__STATIC void gatt_srv_write_aes_cmac_cb(uint8_t aes_status, const uint8_t* aes_res, uint32_t src_info)
{
    gatt_srv_write_proc_t* p_proc = (gatt_srv_write_proc_t*) gatt_proc_pick(GETF(src_info, GATT_SRV_WRITE_CONIDX),
                                                                            GETF(src_info, GATT_SRV_WRITE_TOKEN));

    if(p_proc != NULL)
    {
        uint16_t status = GAP_ERR_NO_ERROR;

        // copy computed MAC in buffer
        if(memcmp(co_buf_tail(p_proc->p_buf) + L2CAP_ATT_SIGN_COUNTER_LEN,
                  &(aes_res[AES_BLOCK_SIZE - L2CAP_ATT_SIGN_MAC_LEN]), L2CAP_ATT_SIGN_MAC_LEN) == 0)
        {
            uint32_t rcv_sign_counter = co_btohl(co_read32p(co_buf_tail(p_proc->p_buf)));
            // update signature value
            gapc_bond_sign_count_set(GETF(src_info, GATT_SRV_WRITE_CONIDX), GAPC_INFO_SRC_PEER, rcv_sign_counter+1);
        }
        else
        {
            status = GATT_ERR_SIGNED_WRITE;
        }

        // continue write signed operation
        gatt_proc_continue(GETF(src_info, GATT_SRV_WRITE_CONIDX), (gatt_proc_t*) p_proc, GATT_PROC_AES_RES, status);
    }
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
__STATIC void gatt_srv_write_proc_continue(uint8_t conidx, gatt_srv_write_proc_t* p_proc, uint8_t proc_state,
                                           uint16_t status)
{
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];
    bool finished = false;

    switch(proc_state)
    {
        case GATT_PROC_PDU_RX:
        {
            gatt_db_svc_t* p_svc;
            gatt_db_att_t* p_att;

            // retrieve attribute info
            if(status == GAP_ERR_NO_ERROR)
            {
                status = gatt_db_att_get(p_proc->hdl, &p_svc, &p_att);
            }

            // retrieve attribute properties
            if(status == GAP_ERR_NO_ERROR)
            {
                status = gatt_db_att_access_check(conidx, p_proc->access, p_svc, p_att);
            }

            if(status == GAP_ERR_NO_ERROR)
            {
                p_proc->svc_user_lid = p_svc->user_lid;

                if(p_proc->hdr.proc_id == GATT_PROC_HANDLE_WRITE)
                {
                    // Verify length to be updated
                    if((p_proc->offset + co_buf_data_len(p_proc->p_buf)) > GETF(p_att->ext_info, GATT_ATT_WRITE_MAX_SIZE))
                    {
                        status = ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN;
                    }
                }
                else // GATT_PROC_HANDLE_PREP_WRITE
                {
                    p_proc->att_max_len             = GETF(p_att->ext_info, GATT_ATT_WRITE_MAX_SIZE);
                    p_proc->write_with_offset_authz = !GETB(p_att->ext_info, GATT_ATT_NO_OFFSET);
                }
            }

            // check if signature must be verified
            if((status == GAP_ERR_NO_ERROR) && (p_proc->access == GATT_DB_ACCESS_WRITE_SIGNED))
            {
                uint32_t rcv_sign_counter = co_btohl(co_read32p(co_buf_tail(p_proc->p_buf)));

                // check that signature check can be executed
                if (   !gapc_is_sec_set(conidx, GAPC_LK_BONDED) || (gapc_lk_sec_lvl_get(conidx) == GAP_SEC_NOT_ENC)
                    || (rcv_sign_counter < gapc_bond_sign_count_get(conidx, GAPC_INFO_SRC_PEER)))
                {
                    status = GATT_ERR_SIGNED_WRITE;
                }
                else
                {
                    uint32_t aes_cmac_info = 0;

                    SETF(aes_cmac_info, GATT_SRV_WRITE_CONIDX, conidx);
                    SETF(aes_cmac_info, GATT_SRV_WRITE_TOKEN,  p_proc->hdr.token);

                    // ask for AES MAC computation
                    aes_cmac(gapc_bond_csrk_get(conidx, GAPC_INFO_SRC_PEER), co_buf_data(p_proc->p_buf) - (L2CAP_ATT_HEADER_LEN + GATT_HANDLE_LEN),
                             L2CAP_ATT_HEADER_LEN + GATT_HANDLE_LEN + co_buf_data_len(p_proc->p_buf) + L2CAP_ATT_SIGN_COUNTER_LEN,
                             gatt_srv_write_aes_cmac_cb, aes_cmac_info);
                    break;
                }
            }
        }
        // no break;

        case GATT_PROC_AES_RES:
        {
            if(status == GAP_ERR_NO_ERROR)
            {
                gatt_user_t* p_user = gatt_user_get(p_proc->svc_user_lid);

                if(p_user != NULL)
                {
                    // inform upper about the write request
                    if(p_proc->hdr.proc_id != GATT_PROC_HANDLE_PREP_WRITE)
                    {
                        SETB(p_proc->hdr.info_bf, GATT_PROC_USER_CFM, false);
                        p_proc->hdr.user_lid      = p_proc->svc_user_lid;
                        p_user->p_cb->srv.cb_att_val_set(conidx, p_proc->hdr.user_lid, p_proc->hdr.token, p_proc->hdl,
                                                         p_proc->offset, p_proc->p_buf);

                        co_buf_release(p_proc->p_buf);
                        p_proc->p_buf = NULL;

                        break;
                    }
                    // Ask GATT User about current size of attribute and check if modification of attribute is authorized
                    else // GATT_PROC_HANDLE_PREP_WRITE
                    {
                        uint16_t data_len = co_buf_data_len(p_proc->p_buf);
                        bool found = false;
                        gatt_srv_write_prep_meta_t* p_prep_buf_meta = NULL;
                        co_buf_t* p_prep_buf = (co_buf_t*) co_list_pick(&(p_con->prep_write_queue));

                        while(p_prep_buf != NULL)
                        {
                            p_prep_buf_meta = (gatt_srv_write_prep_meta_t*) co_buf_metadata(p_prep_buf);

                            // check if attribute value already present in prepare write list
                            if(p_prep_buf_meta->hdl == p_proc->hdl)
                            {
                                // copy attribute value length
                                p_proc->att_val_len = p_prep_buf_meta->att_val_len;
                                found = true;
                                p_proc->p_prev_prep_buf = p_prep_buf;
                            }
                            // search is over
                            else if(found)
                            {
                                break;
                            }

                            p_prep_buf = (co_buf_t*)p_prep_buf->hdr.next;
                        }

                        // if not found, get information about current attribute value size
                        if(!found)
                        {
                            if((p_user->p_cb->srv.cb_att_info_get != NULL) && p_proc->write_with_offset_authz)
                            {
                                SETB(p_proc->hdr.info_bf, GATT_PROC_USER_CFM, false);
                                p_proc->hdr.user_lid      = p_proc->svc_user_lid;
                                p_user->p_cb->srv.cb_att_info_get(conidx, p_proc->hdr.user_lid, p_proc->hdr.token, p_proc->hdl);
                                break;
                            }
                            else
                            {
                                p_proc->att_val_len = 0;
                            }
                        }
                        else
                        {
                            uint16_t prev_data_len = co_buf_data_len(p_proc->p_prev_prep_buf);
                            uint16_t prev_tail_len = co_buf_tail_len(p_proc->p_prev_prep_buf);

                            // Check if buffer can be copied in previous buffer
                            p_proc->append_data =    ((p_prep_buf_meta->offset + prev_data_len) == p_proc->offset)
                                                  && (prev_tail_len > data_len);
                        }

                        // Check if total prepare write queue does not exceed limit
                        if(p_con->prep_write_data_len + data_len > GATT_PREP_WRITE_QUEUE_MEM_LIMIT)
                        {
                            status = ATT_ERR_PREPARE_QUEUE_FULL;
                        }
                        // Check if GATT activities doesn't exceed memory limit
                        else if((!p_proc->append_data) && (gatt_env.total_mem_size > GATT_MEM_LIMIT))
                        {
                            status = ATT_ERR_INSUFF_RESOURCE;
                        }
                    }
                }
                else
                {
                    status = ATT_ERR_UNLIKELY_ERR;
                }
            }
        }
        // no break;

        case GATT_PROC_USER_CFM:
        {
            if(p_proc->access == GATT_DB_ACCESS_WRITE)
            {
                // Send write response / error responses to peer device
                l2cap_att_pdu_t pdu;
                co_buf_t* p_buf = NULL;

                if((status != GAP_ERR_NO_ERROR) || (p_proc->hdr.proc_id == GATT_PROC_HANDLE_WRITE))
                {
                    // allocate buffer used for confirmation transmission
                    if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, GATT_BUFFER_TAIL_LEN) != CO_BUF_ERR_NO_ERROR)
                    {
                        status = GAP_ERR_INSUFF_RESOURCES;
                        break;
                    }
                }
                // reuse prepare write buffer
                else
                {
                    p_buf = p_proc->p_buf;
                    co_buf_acquire(p_buf);
                }

                if(status != GAP_ERR_NO_ERROR)
                {
                    pdu.code = L2CAP_ATT_ERR_RSP_OPCODE;
                    pdu.err_rsp.op_code = (p_proc->hdr.proc_id != GATT_PROC_HANDLE_PREP_WRITE)
                                        ? L2CAP_ATT_WR_REQ_OPCODE
                                        : L2CAP_ATT_PREP_WR_REQ_OPCODE;
                    pdu.err_rsp.handle  = p_proc->hdl;
                    pdu.err_rsp.reason  = status;
                }
                else if(p_proc->hdr.proc_id == GATT_PROC_HANDLE_WRITE)
                {
                    pdu.code = L2CAP_ATT_WR_RSP_OPCODE;
                }
                else // GATT_PROC_HANDLE_PREP_WRITE
                {
                    pdu.code = L2CAP_ATT_PREP_WR_RSP_OPCODE;
                    pdu.prep_wr_rsp.handle = p_proc->hdl;
                    pdu.prep_wr_rsp.offset = p_proc->offset;
                }

                // Ask for PDU transmission
                status = gatt_proc_pdu_send(conidx, (gatt_proc_t*) p_proc, &pdu, p_buf, NULL);
                // release buffer
                co_buf_release(p_buf);
            }
            else
            {
                // mark that reception on bearer can continue
                gatt_proc_bearer_rx_continue(conidx, (gatt_proc_t*) p_proc);
                finished = true;
            }
        } break;

        case GATT_PROC_PDU_PUSHED_TO_LL:    // Confirmation properly pushed in TX LL buffers
        {
            // message is transmitted, the write element can be put in prepare write queue.
            if(   (status == GAP_ERR_NO_ERROR) && (p_proc->hdr.proc_id == GATT_PROC_HANDLE_PREP_WRITE)
               && (p_proc->p_buf != NULL))
            {
                uint16_t data_len = co_buf_data_len(p_proc->p_buf);

                // increase size of element in prepare write list
                p_con->prep_write_data_len += data_len;

                // handle not yet present in prepare write queue
                if(p_proc->p_prev_prep_buf == NULL)
                {
                    co_list_push_back(&(p_con->prep_write_queue), &(p_proc->p_buf->hdr));
                }
                // check if data can be pushed in previous buffer
                else if(p_proc->append_data)
                {
                    memcpy(co_buf_tail(p_proc->p_prev_prep_buf), co_buf_data(p_proc->p_buf), data_len);
                    co_buf_tail_reserve(p_proc->p_prev_prep_buf, data_len);
                    co_buf_release(p_proc->p_buf);
                }
                // insert new prepare write after found position in prepare write queue
                else
                {
                    co_list_insert_after(&(p_con->prep_write_queue), &(p_proc->p_prev_prep_buf->hdr),
                                         &(p_proc->p_buf->hdr));
                }

                if(!p_proc->append_data)
                {
                    gatt_srv_write_prep_meta_t* p_buf_meta;

                    // Update memory size used by GATT layer
                    gatt_env.total_mem_size += co_buf_size(p_proc->p_buf);

                    // reuse buffer for another need - give more credit to peer device
                    co_buf_reuse(p_proc->p_buf);

                    // setup metadata after buffer being used by lower layer
                    p_buf_meta = (gatt_srv_write_prep_meta_t*) co_buf_metadata(p_proc->p_buf);
                    p_buf_meta->hdl          = p_proc->hdl;
                    p_buf_meta->offset       = p_proc->offset;
                    p_buf_meta->att_val_len  = p_proc->att_val_len;
                    p_buf_meta->att_max_len  = p_proc->att_max_len;
                    p_buf_meta->svc_user_lid = p_proc->svc_user_lid;
                }

                p_proc->p_buf = NULL;
            }
        }
        // no break

        case GATT_PROC_ERROR:               // Handle error detection
        {
            finished = true;
        } break;

        default: { /*  Nothing to do */  } break;
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        finished = true;
    }

    if(finished)
    {
        // check if buffer must be released
        if(p_proc->p_buf != NULL)
        {
            co_buf_release(p_proc->p_buf);
        }

        // Pop Procedure
        gatt_proc_pop(conidx, (gatt_proc_t*) p_proc, true);
    }
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
__STATIC void gatt_srv_write_exe_proc_continue(uint8_t conidx, gatt_srv_write_exe_proc_t* p_proc, uint8_t proc_state,
                                               uint16_t status)
{
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];
    bool finished = false;
    bool send_rsp = true;
    bool clean_up = true;

    switch(proc_state)
    {
        case GATT_PROC_PDU_RX:
        {
            p_proc->hdl = GATT_INVALID_HDL;

            if(p_proc->exe_flag == L2CAP_ATT_WRITE_CANCEL)
            {
                // Nothing to do
            }
            else if (p_proc->exe_flag == L2CAP_ATT_WRITE_EXECUTE)
            {
                co_buf_t* p_buf = (co_buf_t*) co_list_pick(&(p_con->prep_write_queue));
                // Handle block environment
                gatt_srv_write_prep_meta_t* p_blk_meta = NULL;

                // First step: check each buffer and verify that there is no issues with attribute length and offset
                while(p_buf != NULL)
                {
                    gatt_srv_write_prep_meta_t* p_buf_meta = (gatt_srv_write_prep_meta_t*) co_buf_metadata(p_buf);

                    // queue is grouped by handle of same value, if a new handle is detected, consider that a new block starts
                    if(p_proc->hdl != p_buf_meta->hdl)
                    {
                        p_blk_meta   = p_buf_meta;
                        p_blk_meta->total_len = 0;
                        p_proc->hdl = p_buf_meta->hdl;
                    }
                    ASSERT_ERR(p_blk_meta != NULL);

                    // verify that offset is valid
                    if((p_buf_meta->offset > p_blk_meta->att_max_len) || (p_buf_meta->offset > p_blk_meta->att_val_len))
                    {
                        status = ATT_ERR_INVALID_OFFSET;
                        break;
                    }
                    // Verify length to be updated
                    else if((p_buf_meta->offset + co_buf_data_len(p_buf)) > p_blk_meta->att_max_len)
                    {
                        status = ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN;
                        break;
                    }
                    // if offset is less than block value length, it means that a new block starts
                    else if((p_buf_meta->offset < p_blk_meta->att_val_len) && (p_buf_meta != p_blk_meta))
                    {
                        p_blk_meta = p_buf_meta;
                        p_blk_meta->total_len = 0;
                    }

                    // update attribute value length and total block data length
                    p_blk_meta->total_len   += co_buf_data_len(p_buf);
                    p_blk_meta->att_val_len  = p_blk_meta->offset + p_blk_meta->total_len;

                    p_buf = (co_buf_t*)p_buf->hdr.next;
                }
            }
            else
            {
                status = ATT_ERR_REQUEST_NOT_SUPPORTED;
                // In this case, no need to remove write queue
                clean_up = false;
            }

        }
        // no break;
        case GATT_PROC_USER_CFM:
        {
            // Execute an Attribute set call-back when write execution is requested and there is something to write.
            if(   (p_proc->exe_flag == L2CAP_ATT_WRITE_EXECUTE) && !co_list_is_empty(&(p_con->prep_write_queue))
               && (status == GAP_ERR_NO_ERROR))
            {
                co_buf_t* p_prep_buf = (co_buf_t*) co_list_pop_front(&(p_con->prep_write_queue));
                // Handle block environment
                gatt_srv_write_prep_meta_t* p_blk_meta = NULL;
                co_buf_t* p_buf;
                p_proc->hdl = GATT_INVALID_HDL;

                // First step: check each buffer and verify that there is no issues with attribute length and offset
                while(p_prep_buf != NULL)
                {
                    gatt_srv_write_prep_meta_t* p_buf_meta = (gatt_srv_write_prep_meta_t*) co_buf_metadata(p_prep_buf);
                    gatt_env.total_mem_size -= co_buf_size(p_prep_buf);

                    // queue is grouped by handle of same value, if a new handle is detected, consider that a new block starts
                    if(p_blk_meta == NULL)
                    {
                        p_blk_meta             = p_buf_meta;
                        p_blk_meta->total_len -= co_buf_data_len(p_prep_buf);
                        p_proc->hdl           = p_buf_meta->hdl;

                        // check if current buffer can be reused
                        if(co_buf_tail_len(p_prep_buf) >= p_blk_meta->total_len)
                        {
                            p_buf = p_prep_buf;
                        }
                        // allocate a new buffer
                        else
                        {
                            if(co_buf_duplicate(p_prep_buf, &p_buf, 0, p_blk_meta->total_len) != CO_BUF_ERR_NO_ERROR)
                            {
                                status = ATT_ERR_INSUFF_RESOURCE;
                                co_buf_release(p_prep_buf);
                                break;
                            }
                            p_blk_meta = (gatt_srv_write_prep_meta_t*) co_buf_metadata(p_buf);
                            memcpy(p_blk_meta, p_buf_meta, sizeof(gatt_srv_write_prep_meta_t));
                            co_buf_release(p_prep_buf);
                        }
                    }
                    else
                    {
                        uint16_t data_len =  co_buf_data_len(p_prep_buf);
                        // copy received data
                        co_buf_copy_data_to_mem(p_prep_buf, co_buf_tail(p_buf), data_len);
                        co_buf_release(p_prep_buf);
                        p_blk_meta->total_len -= data_len;
                        co_buf_tail_reserve(p_buf, data_len);
                    }

                    // data copy is over
                    if(p_blk_meta->total_len == 0)
                    {
                        gatt_user_t* p_user = gatt_user_get(p_blk_meta->svc_user_lid);

                        if(p_user != NULL)
                        {
                            p_proc->hdr.user_lid      = p_blk_meta->svc_user_lid;
                            SETB(p_proc->hdr.info_bf, GATT_PROC_USER_CFM, false);
                            p_user->p_cb->srv.cb_att_val_set(conidx, p_blk_meta->svc_user_lid, p_proc->hdr.token,
                                                             p_blk_meta->hdl, p_blk_meta->offset, p_buf);
                            send_rsp = false;
                            clean_up = false;
                        }
                        else
                        {
                            status = ATT_ERR_APP_ERROR;
                        }

                        co_buf_release(p_buf);

                        break;
                    }

                    p_prep_buf = (co_buf_t*) co_list_pop_front(&(p_con->prep_write_queue));
                }
            }
        } break;

        case GATT_PROC_PDU_PUSHED_TO_LL:    // Confirmation properly pushed in TX LL buffers
        case GATT_PROC_ERROR:               // Handle error detection
        {
            finished = true;
            send_rsp = false;
            clean_up = false;
        } break;

        default: { /*  Nothing to do */  } break;
    }

    // Clean-up prepare write queue
    if(clean_up)
    {
        gatt_srv_write_queue_cleanup(conidx);
    }

    if(send_rsp)
    {
        // Send write response / error responses to peer device
        l2cap_att_pdu_t pdu;
        co_buf_t* p_buf = NULL;

        // allocate buffer used for confirmation transmission
        if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
        {
            if(status != GAP_ERR_NO_ERROR)
            {
                pdu.code            = L2CAP_ATT_ERR_RSP_OPCODE;
                pdu.err_rsp.op_code = L2CAP_ATT_EXE_WR_REQ_OPCODE;
                pdu.err_rsp.handle  = p_proc->hdl;
                pdu.err_rsp.reason  = status;
            }
            else
            {
                pdu.code = L2CAP_ATT_EXE_WR_RSP_OPCODE;
            }

            // Ask for PDU transmission
            status = gatt_proc_pdu_send(conidx, (gatt_proc_t*) p_proc, &pdu, p_buf, NULL);
            // release buffer
            co_buf_release(p_buf);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }

        if(status != GAP_ERR_NO_ERROR)
        {
            finished = true;
        }
    }

    if(finished)
    {
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
 * @brief Ask GATT Server module to create a Write procedure handler
 *
 * @param[in]  conidx       Connection index
 * @param[in]  op_code      Attribute operation code received
 * @param[out] p_pdu_hdl_cb Pointer to PDU Handler call-back
 * @param[out] pp_proc      Pointer to the procedure created, NULL if procedure not created
 *
 * @return Procedure creation status code (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gatt_srv_write_proc_create(uint8_t conidx, uint8_t op_code, gatt_proc_pdu_handler_cb* p_pdu_hdl_cb,
                                    gatt_proc_handler_t** pp_proc)
{
    uint16_t status;
    uint8_t  proc_id = 0;

    // compute procedure identifier
    switch(GETF(op_code, L2CAP_ATT_OPCODE_METHOD))
    {
        case L2CAP_ATT_WR_REQ_OPCODE:      { proc_id = GATT_PROC_HANDLE_WRITE;      } break;
        case L2CAP_ATT_PREP_WR_REQ_OPCODE: { proc_id = GATT_PROC_HANDLE_PREP_WRITE; } break;
        default:                           { ASSERT_ERR(0);                         } break;
    }

    // Allocate procedure
    status = gatt_proc_handler_alloc(conidx, proc_id, sizeof(gatt_srv_write_proc_t),
                                     (gatt_proc_cb) gatt_srv_write_proc_continue, pp_proc);

    *p_pdu_hdl_cb = (gatt_proc_pdu_handler_cb) gatt_srv_write_l2cap_att_wr_req_handler;

    if(status == GAP_ERR_NO_ERROR)
    {
        gatt_srv_write_proc_t* p_proc = (gatt_srv_write_proc_t*) *pp_proc;
        p_proc->append_data     = false;
        p_proc->p_prev_prep_buf = NULL;
        p_proc->att_val_len     = 0;
        p_proc->att_max_len     = 0;
        p_proc->svc_user_lid    = GATT_INVALID_USER_LID;

        // Set the access mask
        if(GETB(op_code, L2CAP_ATT_OPCODE_AUTH_SIGNATURE_FLAG))
        {
            p_proc->access = GATT_DB_ACCESS_WRITE_SIGNED;
        }
        else if (GETB(op_code, L2CAP_ATT_OPCODE_CMD_FLAG))
        {
            p_proc->access = GATT_DB_ACCESS_WRITE_COMMAND;
        }
        else
        {
            p_proc->access = GATT_DB_ACCESS_WRITE;
        }
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Ask GATT Server module to create an Execute write procedure handler
 *
 * @param[in]  conidx    Connection index
 * @param[in]  op_code   Attribute operation code received
 * @param[out] p_pdu_hdl_cb Pointer to PDU Handler call-back
 * @param[out] pp_proc   Pointer to the procedure created, NULL if procedure not created
 *
 * @return Procedure creation status code (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gatt_srv_write_exe_proc_create(uint8_t conidx, uint8_t op_code, gatt_proc_pdu_handler_cb* p_pdu_hdl_cb,
                                        gatt_proc_handler_t** pp_proc)
{
    uint16_t status;

    // Allocate procedure
    status = gatt_proc_handler_alloc(conidx, GATT_PROC_HANDLE_EXE_WRITE, sizeof(gatt_srv_write_exe_proc_t),
                                     (gatt_proc_cb) gatt_srv_write_exe_proc_continue, pp_proc);

    *p_pdu_hdl_cb = (gatt_proc_pdu_handler_cb) gatt_srv_write_l2cap_att_exe_wr_req_handler;

    return (status);
}


void gatt_srv_write_queue_cleanup(uint8_t conidx)
{
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];
    while(!co_list_is_empty(&(p_con->prep_write_queue)))
    {
        co_buf_t* p_buf = (co_buf_t*) co_list_pop_front(&(p_con->prep_write_queue));
        gatt_env.total_mem_size -= co_buf_size(p_buf);
        co_buf_release(p_buf);
    }
    p_con->prep_write_data_len = 0;
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t gatt_srv_att_val_set_cfm(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t status)
{
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
        // continue procedure execution
        SETB(p_proc->info_bf, GATT_PROC_USER_CFM, true);
        p_proc->user_lid      = GATT_INVALID_USER_LID;
        gatt_proc_continue(conidx, (gatt_proc_t*) p_proc, GATT_PROC_USER_CFM, status);

        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

uint16_t gatt_srv_att_info_get_cfm(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t status, uint16_t att_length)
{
    gatt_srv_write_proc_t* p_proc = (gatt_srv_write_proc_t*) gatt_proc_pick(conidx, token);

    // check if procedure is found
    if((p_proc == NULL) || (user_lid != p_proc->hdr.user_lid))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    // check if confirm not already received
    else if ((p_proc->hdr.proc_id != GATT_PROC_HANDLE_PREP_WRITE) && GETB(p_proc->hdr.info_bf, GATT_PROC_USER_CFM))
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else
    {
        // continue procedure execution
        SETB(p_proc->hdr.info_bf, GATT_PROC_USER_CFM, true);

        p_proc->hdr.user_lid      = GATT_INVALID_USER_LID;
        p_proc->att_val_len       = att_length;
        gatt_proc_continue(conidx, (gatt_proc_t*) p_proc, GATT_PROC_USER_CFM, status);

        status = GAP_ERR_NO_ERROR;
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
 * @brief This function is called during a write procedure to get information about a
 *        specific attribute handle.
 *
 *        @see gatt_srv_att_info_get_cfm shall be called to provide attribute information
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] hdl           Attribute handle
 ****************************************************************************************
 */
void gatt_srv_att_info_get_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        gatt_srv_att_info_get_req_ind_t* p_req_ind = KE_MSG_ALLOC(GATT_REQ_IND, p_user->dest_task_nbr, TASK_GATT,
                                                                  gatt_srv_att_info_get_req_ind);

        // prepare request indication to send
        if(p_req_ind != NULL)
        {
            p_req_ind->req_ind_code = GATT_SRV_ATT_INFO_GET;
            p_req_ind->token        = token;
            p_req_ind->user_lid     = user_lid;
            p_req_ind->conidx       = conidx;

            p_req_ind->hdl          = hdl;

            // send message to host
            ke_msg_send(p_req_ind);
        }
    }
    else
    {
        // user not found, reject write request
        gatt_srv_att_info_get_cfm(conidx, user_lid, token, ATT_ERR_UNLIKELY_ERR, 0);
    }
}

/**
 ****************************************************************************************
 * @brief Handle confirmation of attribute information get handling from peer device by GATT server user.
 *
 * @param[in] p_cfm     Pointer to confirmation parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_srv_att_info_get_cfm_handler(gatt_srv_att_info_get_cfm_t* p_cfm, uint16_t src_id)
{
    gatt_srv_att_info_get_cfm(p_cfm->conidx, p_cfm->user_lid, p_cfm->token, p_cfm->status, p_cfm->att_length);
}

/**
 ****************************************************************************************
 * @brief This function is called during a write procedure to modify attribute handle.
 *
 *        @see gatt_srv_att_val_set_cfm shall be called to accept or reject attribute
 *        update.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] hdl           Attribute handle
 * @param[in] offset        Data offset
 * @param[in] p_data        Pointer to buffer that contains data to write starting from offset
 ****************************************************************************************
 */
void gatt_srv_att_val_set_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset,
                             co_buf_t* p_data)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        uint16_t data_len = co_buf_data_len(p_data);
        gatt_srv_att_val_set_req_ind_t* p_req_ind = KE_MSG_ALLOC_DYN(GATT_REQ_IND, p_user->dest_task_nbr, TASK_GATT,
                                                                     gatt_srv_att_val_set_req_ind, data_len);

        // prepare request indication to send
        if(p_req_ind != NULL)
        {
            p_req_ind->req_ind_code = GATT_SRV_ATT_VAL_SET;
            p_req_ind->token        = token;
            p_req_ind->user_lid     = user_lid;
            p_req_ind->conidx       = conidx;

            p_req_ind->hdl          = hdl;
            p_req_ind->offset       = offset;
            p_req_ind->value_length = data_len;

            co_buf_copy_data_to_mem(p_data, p_req_ind->value, data_len);

            // send message to host
            ke_msg_send(p_req_ind);
        }
    }
    else
    {
        // user not found, reject write request
        gatt_srv_att_val_set_cfm(conidx, user_lid, token, ATT_ERR_UNLIKELY_ERR);
    }
}

/**
 ****************************************************************************************
 * @brief Handle confirmation of write handling from peer device by GATT server user.
 *
 * @param[in] p_cfm     Pointer to confirmation parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_srv_att_val_set_cfm_handler(gatt_srv_att_val_set_cfm_t* p_cfm, uint16_t src_id)
{
    gatt_srv_att_val_set_cfm(p_cfm->conidx, p_cfm->user_lid, p_cfm->token, p_cfm->status);
}

#endif // (HOST_MSG_API)

#endif // (BLE_GATT)
/// @} GATT

