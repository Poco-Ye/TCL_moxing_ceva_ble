/**
 ****************************************************************************************
 * @file gatt_cli_write.c
 *
 * @brief  GATT Client Write Procedure
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
#include "gatt_user.h"      // GATT User API
#include "gatt_proc.h"      // Procedure API
#include "gatt_int.h"       // To transform a write command to a write long
#include "../inc/gap_hl_api.h" // For CSRK and Sign counter usage

#include <string.h>         // for memcmp
#include "co_math.h"        // for co_min
#include "co_endian.h"      // for host to bt number conversion
#include "aes.h"            // for aes_cmac usage

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// Write buffer header length required
#define GATT_CLI_WRITE_HEAD_LEN    (L2CAP_ATT_HEADER_LEN + GATT_HANDLE_LEN + L2CAP_ATT_OFFSET_LEN)

/// Bit field used to retrieve a procedure during AES-CMAC operation
enum gatt_cli_write_info_bf
{
    /// Connection Index
    GATT_CLI_WRITE_CONIDX_MASK      = 0x000000FF,
    GATT_CLI_WRITE_CONIDX_LSB       = 0,
    /// Procedure token
    GATT_CLI_WRITE_TOKEN_MASK       = 0x00FFFF00,
    GATT_CLI_WRITE_TOKEN_LSB        = 8,
};
/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Write procedure information
typedef struct gatt_cli_write_proc
{
    /// Procedure header - required for any Attribute procedure
    gatt_proc_t             hdr;
    #if (HOST_MSG_API)
    /// Message command code used.
    uint16_t                msg_cmd_code;
    #endif // (HOST_MSG_API)
    /// Data value to write
    co_buf_t*               p_data;
    /// Attribute handle
    uint16_t                hdl;
    /// Data offset, valid only for GATT_WRITE
    uint16_t                offset;
    /// Data length to write
    uint16_t                length;
    /// Maximum value size in attribute PDU.
    uint16_t                max_pdu_size;
    /// Size of value in transmitted PDU
    uint16_t                pdu_val_len;
    /// Procedure execution status (used to store an erroneous state)
    uint16_t                status;
    /// Write execution mode (see enum #gatt_write_execute_mode). Valid only for GATT_WRITE.
    uint8_t                 mode;
    /// Buffer confirmation received
    bool                    cfm_recv;
} gatt_cli_write_proc_t;

/// Execute Write procedure information
typedef struct gatt_cli_write_exe_proc
{
    /// Procedure header - required for any Attribute procedure
    gatt_proc_t             hdr;
    #if (HOST_MSG_API)
    /// Message command code used.
    uint16_t                msg_cmd_code;
    #endif // (HOST_MSG_API)
    /// True: Perform pending write operations
    /// False: Cancel pending write operations
    bool                    execute;
} gatt_cli_write_exe_proc_t;

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
 * @brief Function called when L2CAP_ATT_WR_RSP attribute PDU is received.
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
__STATIC uint16_t gatt_cli_l2cap_att_wr_rsp_handler(uint8_t conidx, gatt_proc_t* p_proc, l2cap_att_wr_rsp_t* p_pdu,
                                                    co_buf_t* p_buf, uint16_t mtu)
{
    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, p_proc);

    return (GAP_ERR_NO_ERROR);
}

/**
 ****************************************************************************************
 * @brief Function called when L2CAP_ATT_RD_RSP or L2CAP_ATT_RD_BLOB_RSP attribute PDU is
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
__STATIC uint16_t gatt_cli_l2cap_att_prep_wr_rsp_handler(uint8_t conidx, gatt_cli_write_proc_t* p_proc,
                                                         l2cap_att_prep_wr_rsp_t* p_pdu,
                                                         co_buf_t* p_buf, uint16_t mtu)
{
    uint16_t status = GAP_ERR_UNEXPECTED;
    uint16_t length = co_buf_data_len(p_buf);
    // Check if prepare write response is valid
    if((p_pdu->handle == p_proc->hdl) && (p_pdu->offset == p_proc->offset) && (length == p_proc->pdu_val_len))
    {
        // check if received data is equals to transmitted data
        if(memcmp(co_buf_data(p_buf), co_buf_data(p_proc->p_data), length) == 0)
        {
            co_buf_head_release(p_proc->p_data, length);
            p_proc->offset += length;
            status = GAP_ERR_NO_ERROR;
        }
    }

    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, &(p_proc->hdr));

    return (status);
}

/**
 ****************************************************************************************
 * @brief Send Attribute Prepare write request / write request / write command or write signed PDU
 *
 * @param[in] conidx     Connection index
 * @param[in] p_proc     Pointer to procedure under execution
 *
 * @return Status of transmission execution
 ****************************************************************************************
 */
__STATIC uint16_t gatt_cli_write_pdu_send(uint8_t conidx, gatt_cli_write_proc_t* p_proc)
{
    uint16_t status     = GAP_ERR_NO_ERROR;
    co_buf_t* p_buf     = NULL;
    uint16_t data_len   = co_buf_data_len(p_proc->p_data);
    gatt_proc_pdu_handler_cb cb_rsp_handler = NULL;
    l2cap_att_pdu_t pdu;

    // check if a new buffer must be allocated
    if((p_proc->pdu_val_len < data_len) || (co_buf_head_len(p_proc->p_data) < GATT_CLI_WRITE_HEAD_LEN))
    {
        if(co_buf_alloc(&p_buf, GATT_CLI_WRITE_HEAD_LEN, p_proc->pdu_val_len, GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
        {
            // copy buffer
            co_buf_copy(p_proc->p_data, p_buf, p_proc->pdu_val_len, 0);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }
    else
    {
        // reuse transmission buffer
        p_buf = p_proc->p_data;
        co_buf_acquire(p_buf);
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        uint8_t proc_id = p_proc->hdr.proc_id;

        // Prepare PDU header - Prepare Write
        if(proc_id == GATT_PROC_WRITE_LONG)
        {
            pdu.code = L2CAP_ATT_PREP_WR_REQ_OPCODE;
            pdu.prep_wr_req.handle = p_proc->hdl;
            pdu.prep_wr_req.offset = p_proc->offset;

            cb_rsp_handler = (gatt_proc_pdu_handler_cb) gatt_cli_l2cap_att_prep_wr_rsp_handler;
        }
        // Prepare PDU header - Write Request / Command / Signed
        else
        {
            pdu.code = L2CAP_ATT_WR_REQ_OPCODE;
            pdu.wr_req.handle = p_proc->hdl;

            if(proc_id == GATT_PROC_WRITE_NO_RESP)
            {
                pdu.code |= L2CAP_ATT_OPCODE_CMD_FLAG_BIT;
            }
            else if (proc_id == GATT_PROC_WRITE_SIGNED)
            {
                pdu.code |= L2CAP_ATT_OPCODE_CMD_FLAG_BIT | L2CAP_ATT_OPCODE_AUTH_SIGNATURE_FLAG_BIT;
            }
            else
            {
                cb_rsp_handler = (gatt_proc_pdu_handler_cb) gatt_cli_l2cap_att_wr_rsp_handler;
            }
        }

        // Ask for PDU transmission
        status = gatt_proc_pdu_send(conidx, &(p_proc->hdr), &pdu, p_buf, cb_rsp_handler);

        // release buffer
        co_buf_release(p_buf);
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Send Attribute Execute write
 *
 * @param[in] conidx     Connection index
 * @param[in] p_proc     Pointer to procedure under execution
 * @param[in] execute    True: Perform pending write operations
 *                       False: Cancel pending write operations
 *
 * @return Status of transmission execution
 ****************************************************************************************
 */
__STATIC uint16_t gatt_cli_write_exe_pdu_send(uint8_t conidx, gatt_proc_t* p_proc, bool execute)
{
    uint16_t status     = GAP_ERR_NO_ERROR;
    co_buf_t* p_buf     = NULL;
    l2cap_att_pdu_t pdu;

    // Allocate buffer for transmission
    if(co_buf_alloc(&p_buf, GATT_CLI_WRITE_HEAD_LEN, 0, GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
    {
        pdu.code             = L2CAP_ATT_EXE_WR_REQ_OPCODE;
        pdu.exe_wr_req.flags = execute ? L2CAP_ATT_WRITE_EXECUTE : L2CAP_ATT_WRITE_CANCEL;

        // Ask for PDU transmission
        status = gatt_proc_pdu_send(conidx, p_proc, &pdu, p_buf,
                                    (gatt_proc_pdu_handler_cb) gatt_cli_l2cap_att_wr_rsp_handler);

        // release buffer
        co_buf_release(p_buf);
    }
    else
    {
        status = GAP_ERR_INSUFF_RESOURCES;
    }

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
__STATIC void gatt_cli_write_aes_cmac_cb(uint8_t aes_status, const uint8_t* aes_res, uint32_t src_info)
{
    uint8_t conidx = GETF(src_info, GATT_CLI_WRITE_CONIDX);
    gatt_cli_write_proc_t* p_proc =
            (gatt_cli_write_proc_t*) gatt_proc_pick(conidx, GETF(src_info, GATT_CLI_WRITE_TOKEN));

    if(p_proc != NULL)
    {
        // copy computed MAC in buffer
        memcpy(co_buf_tail(p_proc->p_data), &(aes_res[AES_BLOCK_SIZE - L2CAP_ATT_SIGN_MAC_LEN]), L2CAP_ATT_SIGN_MAC_LEN);
        co_buf_tail_reserve(p_proc->p_data, L2CAP_ATT_SIGN_MAC_LEN);
        // remove header data - generated by packer
        co_buf_head_release(p_proc->p_data, L2CAP_ATT_HEADER_LEN + GATT_HANDLE_LEN);

        // continue write signed operation
        gatt_proc_continue(conidx, &(p_proc->hdr), GATT_PROC_AES_RES, GAP_ERR_NO_ERROR);
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
__STATIC void gatt_cli_write_proc_continue(uint8_t conidx, gatt_cli_write_proc_t* p_proc, uint8_t proc_state, uint16_t status)
{
    gatt_user_t* p_user  = gatt_user_get(p_proc->hdr.user_lid);
    bool finished = false;
    uint8_t proc_id = p_proc->hdr.proc_id;

    switch(proc_state)
    {
        case GATT_PROC_START:
        {
            // retrieve supported MTU
            p_proc->max_pdu_size = gatt_proc_mtu_get(conidx, &(p_proc->hdr)) - L2CAP_ATT_HEADER_LEN - GATT_HANDLE_LEN;
            p_proc->status       = GAP_ERR_NO_ERROR;

            // Ensure that length field does not exceed  MTU - 3
            if(proc_id == GATT_PROC_WRITE_NO_RESP)
            {
                if(p_proc->p_data == NULL)
                {
                    // update length to max supported value
                    p_proc->length = co_min(p_proc->length, p_proc->max_pdu_size);
                }
                // if Write command cannot be perform use a write long procedure instead
                else if(p_proc->length > p_proc->max_pdu_size)
                {
                    // Pop Procedure
                    gatt_proc_pop(conidx, (gatt_proc_t*) p_proc, false);

                    // transform it to a write long procedure
                    p_proc->hdr.bearer_lid = GATT_INVALID_BEARER_LID;
                    p_proc->hdr.proc_id    = GATT_PROC_WRITE_LONG;

                    // wait procedure to be granted again
                    gatt_proc_push(conidx, (gatt_proc_t*) p_proc);

                    break;
                }
            }
            // Ensure that length field does not exceed  MTU - 15
            else if (proc_id == GATT_PROC_WRITE_SIGNED)
            {
                p_proc->max_pdu_size -= GATT_SIGNATURE_LEN;

                if(p_proc->p_data == NULL)
                {
                    // update length to max supported value
                    p_proc->length = co_min(p_proc->length, p_proc->max_pdu_size);
                }
                // check that write sign can be performed
                else if(p_proc->length > p_proc->max_pdu_size)
                {
                    status = L2CAP_ERR_INVALID_MTU;
                    break;
                }
            }
            else if (proc_id == GATT_PROC_WRITE_LONG)
            {
                // acquire write queue mutex
                gatt_proc_write_queue_mutex_set(conidx, true);
            }

            if(p_proc->p_data == NULL)
            {
                // Ask upper layer to provide buffer information
                p_user->p_cb->cli.cb_att_val_get(conidx, p_proc->hdr.user_lid, p_proc->hdr.token, p_proc->hdr.dummy,
                                                 p_proc->hdl, p_proc->offset, p_proc->length);
                break;
            }
        }
        // no break;

        case GATT_PROC_USER_CFM:
        {
            if(status != GAP_ERR_NO_ERROR) break;

            uint16_t data_len = co_buf_data_len(p_proc->p_data);

            // update max PDU value size if prepare write is performed
            if(proc_id == GATT_PROC_WRITE_LONG)
            {
                p_proc->max_pdu_size -= L2CAP_ATT_OFFSET_LEN;
            }

            // compute value length in pdu
            p_proc->pdu_val_len = co_min(data_len, p_proc->max_pdu_size);

            // for write command, remove data that cannot be transfered
            if(GATT_PROC_TYPE_GET(proc_id) == GATT_PROC_CMD)
            {
                co_buf_tail_release(p_proc->p_data, data_len - p_proc->pdu_val_len);
            }

            if(proc_id != GATT_PROC_WRITE_SIGNED)
            {
                status = gatt_cli_write_pdu_send(conidx, p_proc);
            }
            else
            {
                uint32_t aes_cmac_info = 0;
                uint32_t sign_count;
                // Check if signature can be insert
                ASSERT_ERR(co_buf_tail_len(p_proc->p_data) >= GATT_SIGNATURE_LEN);
                p_proc->pdu_val_len += GATT_SIGNATURE_LEN;

                sign_count = gapc_bond_sign_count_get(conidx, GAPC_INFO_SRC_LOCAL);
                // change value of sign counter
                gapc_bond_sign_count_set(conidx, GAPC_INFO_SRC_LOCAL, sign_count + 1);

                // update buffer with sign count information
                co_write32p(co_buf_tail(p_proc->p_data), co_htobl(sign_count));
                co_buf_tail_reserve(p_proc->p_data, L2CAP_ATT_SIGN_COUNTER_LEN);

                // Put ATT header data in packet
                co_buf_head_reserve(p_proc->p_data, GATT_HANDLE_LEN);
                co_write16p(co_buf_data(p_proc->p_data), co_htobs(p_proc->hdl));
                co_buf_head_reserve(p_proc->p_data, L2CAP_ATT_HEADER_LEN);
                co_buf_data(p_proc->p_data)[0] = L2CAP_ATT_WR_SIGNED_OPCODE;

                SETF(aes_cmac_info, GATT_CLI_WRITE_CONIDX, conidx);
                SETF(aes_cmac_info, GATT_CLI_WRITE_TOKEN,  p_proc->hdr.token);

                // ask for AES MAC computation
                aes_cmac(gapc_bond_csrk_get(conidx, GAPC_INFO_SRC_LOCAL), co_buf_data(p_proc->p_data),
                         co_buf_data_len(p_proc->p_data), gatt_cli_write_aes_cmac_cb, aes_cmac_info);
            }
        } break;


        case GATT_PROC_AES_RES:
        {
            status = gatt_cli_write_pdu_send(conidx, p_proc);
        } break;

        case GATT_PROC_PDU_RX:
        {
            if(status == GAP_ERR_NO_ERROR)
            {
                if(proc_id == GATT_PROC_WRITE_LONG)
                {
                    uint16_t data_len = co_buf_data_len(p_proc->p_data);
                    if(data_len > 0)
                    {
                        // compute value length in pdu
                        p_proc->pdu_val_len = co_min(data_len, p_proc->max_pdu_size);
                        // send next prepare write
                        status = gatt_cli_write_pdu_send(conidx, p_proc);
                    }
                    else if(p_proc->mode == GATT_WRITE_MODE_AUTO_EXECUTE)
                    {
                        // Send Execute Write
                        p_proc->hdr.proc_id = GATT_PROC_WRITE_EXE;
                        status = gatt_cli_write_exe_pdu_send(conidx, &(p_proc->hdr), true);
                    }
                    else
                    {
                        finished = true;
                    }
                }
                else
                {
                    finished = true;
                }
            }
            else if(proc_id == GATT_PROC_WRITE_LONG)
            {
                // Handle prepare write fail.
                p_proc->hdr.proc_id = GATT_PROC_WRITE_EXE;
                // Store failed status
                p_proc->status     = status;
                // send execute write - Cancel
                status = gatt_cli_write_exe_pdu_send(conidx, &(p_proc->hdr), false);
            }
        } break;

        case GATT_PROC_ERROR:
        {
            finished = true;
        } break;

        case GATT_PROC_PDU_PUSHED_TO_LL:
        {
            // write command and write signed procedure are over when PDU is considered as written
            if(GATT_PROC_TYPE_GET(proc_id) == GATT_PROC_CMD)
            {
                finished = true;
            }
        } break;

        default: { /*  Nothing to do */  } break;
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        finished = true;
    }

    if(finished)
    {
        uint8_t proc_state = GETF(p_proc->hdr.info_bf, GATT_PROC_STATE);
        // update status with stored status
        if(p_proc->status != GAP_ERR_NO_ERROR)
        {
            status = p_proc->status;
        }

        if(p_proc->p_data != NULL)
        {
            // release buffer
            co_buf_release(p_proc->p_data);
        }

        // Inform user that procedure is over
        #if (HOST_MSG_API)
        gatt_proc_cur_set((gatt_proc_t*) p_proc);
        #endif // (HOST_MSG_API)
        p_user->p_cb->cli.cb_write_cmp(conidx, p_proc->hdr.user_lid, p_proc->hdr.dummy, status);

        // Pop Procedure
        gatt_proc_pop(conidx, &(p_proc->hdr), true);

        // Release write queue mutex
        if(   (proc_state != GATT_PROC_WAIT_GRANT)
           && ((proc_id == GATT_PROC_WRITE_LONG) || (proc_id == GATT_PROC_WRITE_EXE)))
        {
            gatt_proc_write_queue_mutex_set(conidx, false);
        }
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
__STATIC void gatt_cli_write_exe_proc_continue(uint8_t conidx, gatt_cli_write_exe_proc_t* p_proc, uint8_t proc_state, uint16_t status)
{
    gatt_user_t* p_user  = gatt_user_get(p_proc->hdr.user_lid);
    bool finished = false;

    switch(proc_state)
    {
        case GATT_PROC_START:
        {
            // Send write execution
            gatt_cli_write_exe_pdu_send(conidx, &(p_proc->hdr), p_proc->execute);

            // Acquire write queue mutex
            gatt_proc_write_queue_mutex_set(conidx, true);
        } break;

        case GATT_PROC_PDU_RX:
        case GATT_PROC_ERROR:
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
        uint8_t proc_state = GETF(p_proc->hdr.info_bf, GATT_PROC_STATE);
        // Inform user that procedure is over
        #if (HOST_MSG_API)
        gatt_proc_cur_set((gatt_proc_t*) p_proc);
        #endif // (HOST_MSG_API)
        p_user->p_cb->cli.cb_write_cmp(conidx, p_proc->hdr.user_lid, p_proc->hdr.dummy, status);

        // Pop Procedure
        gatt_proc_pop(conidx, &(p_proc->hdr), true);

        // Release write queue mutex
        if(proc_state != GATT_PROC_WAIT_GRANT)
        {
            // release write queue mutex
            gatt_proc_write_queue_mutex_set(conidx, false);
        }
    }
}


/**
 ****************************************************************************************
 * @brief Command used to create a write procedure
 *
 * @param[in]  conidx       Connection index
 * @param[in]  user_lid     GATT User Local identifier
 * @param[in]  dummy        Dummy parameter whose meaning is upper layer dependent and
 *                          which is returned in command complete.
 * @param[in]  write_type   GATT write type (see enum #gatt_write_type)
 * @param[in]  write_mode   Write execution mode (see enum #gatt_write_mode).
 *                          Valid only for GATT_WRITE.
 * @param[in]  hdl          Attribute handle
 * @param[in]  offset       Data offset, valid only for GATT_WRITE
 * @param[in]  length       Data length to write
 * @param[in]  p_data       Pointer to the data buffer content (NULL for reliable write)
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gatt_cli_write_proc_create(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t write_type, uint8_t write_mode,
                                    uint16_t hdl, uint16_t offset, uint16_t length, co_buf_t* p_data)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    gatt_user_t* p_user  = gatt_user_get(user_lid);

    // Check parameters
    if(hdl == GATT_INVALID_HDL)
    {
        status = ATT_ERR_INVALID_HANDLE;
    }
    // check parameter range
    else if((write_mode > GATT_WRITE_MODE_QUEUE) || (write_type > GATT_WRITE_SIGNED))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    // for a write command, offset shall be equals to zero
    else if((write_type != GATT_WRITE) && (offset != 0))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    // Ensure that user can handle the procedure
    else if(   (p_user == NULL) || (p_user->p_cb->cli.cb_write_cmp == NULL)
            || ((p_data == NULL) && (p_user->p_cb->cli.cb_att_val_get == NULL)))
    {
        status = GAP_ERR_NOT_SUPPORTED;
    }
    else
    {
        gatt_cli_write_proc_t* p_proc;
        uint8_t proc_id = 0;
        uint16_t tx_length = length + L2CAP_ATT_HEADER_LEN + GATT_HANDLE_LEN;

        switch(write_type)
        {
            case GATT_WRITE:
            {
                if((offset == 0) && (write_mode == GATT_WRITE_MODE_AUTO_EXECUTE))
                {
                    proc_id = GATT_PROC_WRITE;
                }
                else
                {
                    proc_id = GATT_PROC_WRITE_LONG;
                }
            } break;
            case GATT_WRITE_SIGNED:
            {
                gatt_con_env_t* p_con = gatt_env.p_con[conidx];
                // Do not support write signed if an EATT bearer already exists or if legacy bearer not opened
                if(!GETB(p_con->state_bf, GATT_CON_LEGACY_ATT_BEARER_OPEN) || ((p_con->bearer_bf & ~CO_BIT(0)) != 0))
                {
                    status = GAP_ERR_NOT_SUPPORTED;
                }
                // Bond data must be present to perform write signed
                else if (!gapc_is_sec_set(conidx, GAPC_LK_BONDED) || (gapc_lk_sec_lvl_get(conidx) == GAP_SEC_NOT_ENC))
                {
                    status = GATT_ERR_SIGNED_WRITE;
                }

                tx_length += L2CAP_ATT_SIGN_LEN;
                proc_id = GATT_PROC_WRITE_SIGNED;

            } break;
            case GATT_WRITE_NO_RESP: { proc_id = GATT_PROC_WRITE_NO_RESP; } break;
            default:                 { ASSERT_ERR(0);                     } break;
        }

        if(status == GAP_ERR_NO_ERROR)
        {
            // Create procedure
            status = gatt_proc_create(conidx, user_lid, dummy, proc_id, tx_length, sizeof(gatt_cli_write_proc_t),
                                      (gatt_proc_cb) gatt_cli_write_proc_continue, (gatt_proc_t**) &p_proc);
        }

        if(status == GAP_ERR_NO_ERROR)
        {
            // fill procedure parameters
            p_proc->p_data            = p_data;
            p_proc->hdl               = hdl;
            p_proc->offset            = offset;
            p_proc->length            = length;
            p_proc->mode              = write_mode;
            p_proc->cfm_recv          = false;

            if(p_data != NULL)
            {
                co_buf_acquire(p_data);
            }

            #if (HOST_MSG_API)
            gatt_proc_cur_set((gatt_proc_t*) p_proc);
            #endif // (HOST_MSG_API)

            // ask procedure to be granted - push it in wait list
            gatt_proc_push(conidx, &(p_proc->hdr));
        }
    }

    return (status);
}
/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */


uint16_t gatt_cli_write_reliable(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t write_type, uint8_t write_mode,
                                 uint16_t hdl, uint16_t offset, uint16_t length)
{
    uint16_t status = gatt_cli_write_proc_create(conidx, user_lid, dummy, write_type, write_mode,
                                                 hdl,  offset,  length, NULL);

    return (status);
}

uint16_t gatt_cli_write(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t write_type,
                        uint16_t hdl, uint16_t offset, co_buf_t* p_data)
{
    uint16_t status;

    if((p_data == NULL))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    // check buffer properties
    if(   (co_buf_head_len(p_data) < GATT_BUFFER_HEADER_LEN) || (co_buf_tail_len(p_data) < GATT_BUFFER_TAIL_LEN)
       || ((write_type == GATT_WRITE_SIGNED) && (co_buf_tail_len(p_data) < (GATT_BUFFER_TAIL_LEN + L2CAP_ATT_SIGN_LEN))))
    {
        status = GAP_ERR_INVALID_BUFFER;
    }
    else
    {
        status = gatt_cli_write_proc_create(conidx, user_lid, dummy, write_type, GATT_WRITE_MODE_AUTO_EXECUTE,
                                            hdl,  offset, co_buf_data_len(p_data), p_data);
    }

    return (status);
}

uint16_t gatt_cli_att_val_get_cfm(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t status, co_buf_t* p_data)
{
    gatt_cli_write_proc_t* p_proc = (gatt_cli_write_proc_t*) gatt_proc_pick(conidx, token);

    do
    {
        // check if procedure is found
        if((p_proc == NULL) || (user_lid != p_proc->hdr.user_lid))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }
        // buffer can be NULL only if status is failing
        if((status == GAP_ERR_NO_ERROR) && (p_data == NULL))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }
        // check if value not already received
        if (p_proc->cfm_recv)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        if(status == GAP_ERR_NO_ERROR)
        {
            // buffer can be NULL
            if(p_data == NULL)
            {
                status = GAP_ERR_INVALID_PARAM;
                break;
            }
            // check buffer properties
            if(   (co_buf_head_len(p_data) < GATT_BUFFER_HEADER_LEN) || (co_buf_tail_len(p_data) < GATT_BUFFER_TAIL_LEN)
                  // For a write signed- tail buffer must be greater
               || ((p_proc->hdr.proc_id == GATT_PROC_WRITE_SIGNED) && (co_buf_tail_len(p_data) < GATT_BUFFER_SIGN_TAIL_LEN)))
            {
                status = GAP_ERR_INVALID_BUFFER;
                break;
            }
        }

        p_proc->cfm_recv = true;

        // acquire the buffer
        if((status == GAP_ERR_NO_ERROR) && (p_data != NULL))
        {
            p_proc->p_data = p_data;
            co_buf_acquire(p_data);
        }
        else { p_proc->p_data = NULL; }

        // continue procedure execution
        gatt_proc_continue(conidx, &(p_proc->hdr), GATT_PROC_USER_CFM, status);

        status = GAP_ERR_NO_ERROR;
    } while(0);

    return (status);
}

uint16_t gatt_cli_write_exe(uint8_t conidx, uint8_t user_lid, uint16_t dummy, bool execute)
{
    uint16_t status;
    gatt_user_t* p_user  = gatt_user_get(user_lid);

    // Ensure that user can handle the procedure
    if((p_user == NULL) || (p_user->p_cb->cli.cb_att_val_get == NULL) || (p_user->p_cb->cli.cb_write_cmp == NULL))
    {
        status = GAP_ERR_NOT_SUPPORTED;
    }
    else
    {
        gatt_cli_write_exe_proc_t* p_proc;

        // Create procedure
        status = gatt_proc_create(conidx, user_lid, dummy, GATT_PROC_WRITE_EXE, L2CAP_LE_MTU_MIN,
                                  sizeof(gatt_cli_write_exe_proc_t),
                                  (gatt_proc_cb) gatt_cli_write_exe_proc_continue, (gatt_proc_t**) &p_proc);

        if(status == GAP_ERR_NO_ERROR)
        {
            // fill procedure parameters
            p_proc->execute           = execute;

            #if (HOST_MSG_API)
            gatt_proc_cur_set((gatt_proc_t*) p_proc);
            #endif // (HOST_MSG_API)

            // ask procedure to be granted - push it in wait list
            gatt_proc_push(conidx, &(p_proc->hdr));
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
 * @brief Handle write reliable command initiated by GATT client user
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_write_reliable_cmd_handler(gatt_cli_write_reliable_cmd_t* p_cmd, uint16_t src_id)
{
    // start discovery
    uint16_t status = gatt_cli_write_reliable(p_cmd->conidx, p_cmd->user_lid, p_cmd->dummy,
                                              p_cmd->write_type, p_cmd->write_mode, p_cmd->hdl, p_cmd->offset,
                                              p_cmd->length);

    if(status != GAP_ERR_NO_ERROR)
    {
        // send command completion - in error
        gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
    }
    else
    {
        // store command code
        gatt_cli_write_proc_t* p_proc = (gatt_cli_write_proc_t*) gatt_proc_cur_get();
        p_proc->msg_cmd_code = p_cmd->cmd_code;
    }
}

/**
 ****************************************************************************************
 * @brief Handle write command initiated by GATT client user
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_write_cmd_handler(gatt_cli_write_cmd_t* p_cmd, uint16_t src_id)
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

        // request write to be transmitted
        status = gatt_cli_write(p_cmd->conidx, p_cmd->user_lid, p_cmd->dummy,
                                p_cmd->write_type, p_cmd->hdl, p_cmd->offset, p_data);

        co_buf_release(p_data);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        // send command completion - in error
        gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
    }
    else
    {
        // store command code
        gatt_cli_write_proc_t* p_proc = (gatt_cli_write_proc_t*) gatt_proc_cur_get();
        p_proc->msg_cmd_code = p_cmd->cmd_code;
    }
}

/**
 ****************************************************************************************
 * @brief Handle write execution command initiated by GATT client user
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_write_exe_cmd_handler(gatt_cli_write_exe_cmd_t* p_cmd, uint16_t src_id)
{
    // start discovery
    uint16_t status = gatt_cli_write_exe(p_cmd->conidx, p_cmd->user_lid, p_cmd->dummy, p_cmd->execute);

    if(status != GAP_ERR_NO_ERROR)
    {
        // send command completion - in error
        gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
    }
    else
    {
        // store command code
        gatt_cli_write_exe_proc_t* p_proc = (gatt_cli_write_exe_proc_t*) gatt_proc_cur_get();
        p_proc->msg_cmd_code = p_cmd->cmd_code;
    }
}


/**
 ****************************************************************************************
 * @brief This function is called when GATT client user write procedure is over.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] status        Status of the procedure (see enum #hl_err)
 ****************************************************************************************
 */
void gatt_cli_write_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        gatt_cli_write_proc_t* p_proc = (gatt_cli_write_proc_t*) gatt_proc_cur_get();

        // send back command complete event
        gatt_msg_send_proc_cmp_evt(p_proc->msg_cmd_code, dummy, conidx, p_user->dest_task_nbr, user_lid, status);
    }
}

/**
 ****************************************************************************************
 * @brief This function is called when GATT client user has initiated a write procedure.
 *
 *        @see gatt_cli_att_val_get_cfm shall be called to provide attribute value.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution - 0x0000 else.
 * @param[in] hdl           Attribute handle
 * @param[in] offset        Data offset
 * @param[in] max_length    Maximum data length to return
 ****************************************************************************************
 */
void gatt_cli_att_val_get_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t dummy,
                             uint16_t hdl, uint16_t offset, uint16_t max_length)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        gatt_cli_att_val_get_req_ind_t* p_req_ind = KE_MSG_ALLOC(GATT_REQ_IND, p_user->dest_task_nbr, TASK_GATT,
                                                                 gatt_cli_att_val_get_req_ind);

        // prepare request indication to send
        if(p_req_ind != NULL)
        {
            p_req_ind->req_ind_code  = GATT_CLI_ATT_VAL_GET;
            p_req_ind->token         = token;
            p_req_ind->user_lid      = user_lid;
            p_req_ind->conidx        = conidx;
            p_req_ind->dummy         = dummy;
            p_req_ind->hdl           = hdl;
            p_req_ind->offset        = offset;
            p_req_ind->max_length    = max_length;

            // send message to host
            ke_msg_send(p_req_ind);
        }
    }
    else
    {
        gatt_cli_att_val_get_cfm(conidx, user_lid, token, ATT_ERR_UNLIKELY_ERR, NULL);
    }
}

/**
 ****************************************************************************************
 * @brief Handle confirmation of reliable write value to send to a peer device.
 *
 * @param[in] p_cfm     Pointer to confirmation parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_att_val_get_cfm_handler(gatt_cli_att_val_get_cfm_t* p_cfm, uint16_t src_id)
{
    co_buf_t* p_data = NULL;

    if(p_cfm->status == GAP_ERR_NO_ERROR)
    {
        // allocate a buffer for event transmission
        if(co_buf_alloc(&p_data, GATT_BUFFER_HEADER_LEN, p_cfm->value_length, GATT_BUFFER_SIGN_TAIL_LEN) != CO_BUF_ERR_NO_ERROR)
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
    gatt_cli_att_val_get_cfm(p_cfm->conidx, p_cfm->user_lid, p_cfm->token, p_cfm->status, p_data);

    // release buffer
    if(p_data != NULL)
    {
        co_buf_release(p_data);
    }
}
#endif // (HOST_MSG_API)

#else  // !(BLE_GATT_CLI)
uint16_t gatt_cli_write(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t write_type,
                        uint16_t hdl, uint16_t offset, co_buf_t* p_data)
{
    return (GAP_ERR_NOT_SUPPORTED);
}

uint16_t gatt_cli_write_reliable(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t write_type, uint8_t write_mode,
                                 uint16_t hdl, uint16_t offset, uint16_t length)
{
    return (GAP_ERR_NOT_SUPPORTED);
}

uint16_t gatt_cli_att_val_get_cfm(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t status, co_buf_t* p_data)
{
    return (GAP_ERR_NOT_SUPPORTED);
}

uint16_t gatt_cli_write_exe(uint8_t conidx, uint8_t user_lid, uint16_t dummy, bool execute)
{
    return (GAP_ERR_NOT_SUPPORTED);
}

#endif // (BLE_GATT_CLI)
#endif // (BLE_GATT)
/// @} GATT

