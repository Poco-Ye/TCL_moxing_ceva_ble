/**
 ****************************************************************************************
 * @file gatt_cli_read.c
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
#include "rwip_config.h"    // IP configuration
#if (BLE_GATT)
#include "gatt.h"           // Native API
#if (BLE_GATT_CLI)
#include "gatt_user.h"      // User API
#include "gatt_proc.h"      // Procedure API
#include "gatt_int.h"       // Internals

#include "co_endian.h"      // endianess support
#include "co_math.h"        // Mathematics
#include <string.h>         // for memcmp and memcpy
/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

// Put buffer tail length to UUID 128 length - this means that read multiple is limited to 8 handles
#define GATT_CLI_READ_BUFFER_TAIL_LEN (GATT_BUFFER_TAIL_LEN + GATT_UUID_128_LEN)

// Read length is unknown, expect a maximum value length
#define GATT_CLI_READ_UNKNOWN_LEN     (0xFFFF)
/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Attribute read information
typedef struct gatt_cli_read_info
{
    /// Attribute handle
    uint16_t hdl;
    ///  Data offset
    uint16_t offset;
    /// Expected Data length to read (0 = read all)
    uint16_t exp_length;
    /// Received length
    uint16_t rx_length;
} gatt_cli_read_info_t;

typedef struct gatt_cli_read_proc
{
    /// Procedure header - required for any Attribute procedure
    gatt_proc_t             hdr;
    /// Buffer queue
    co_list_t               buf_queue;

    /// Search Start Handle
    uint16_t                start_hdl;
    /// Search End Handle
    uint16_t                end_hdl;
    #if (HOST_MSG_API)
    /// Message command code used.
    uint16_t                msg_cmd_code;
    #endif // (HOST_MSG_API)
    /// UUID Type (see enum #gatt_uuid_type)
    uint8_t                 uuid_type;
    /// Searched attribute UUID (LSB First)
    uint8_t                 uuid[GATT_UUID_128_LEN];
    /// Attribute handle under read
    uint8_t                 att_cursor;
    /// Number of attribute to read
    uint8_t                 nb_att;
    /// Used to know if more data must be read for current value handle
    bool                    complete;
    /// use to know if read multiple use variable length method
    bool                    variable_len;
    /// Attribute information
    gatt_cli_read_info_t    att[__ARRAY_EMPTY];
} gatt_cli_read_proc_t;

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
__STATIC uint16_t gatt_cli_read_l2cap_att_rd_rsp_handler(uint8_t conidx, gatt_cli_read_proc_t* p_proc,
                                                         l2cap_att_rd_rsp_t* p_pdu, co_buf_t* p_buf, uint16_t mtu)
{
    uint16_t data_len = co_buf_data_len(p_buf);
    uint16_t status   = GAP_ERR_NO_ERROR;

    // ensure that we don't crash due to memory overflow
    if(    ((data_len + p_proc->att[p_proc->att_cursor].rx_length) < GATT_MAX_VALUE)
        || (gatt_env.total_mem_size > GATT_MEM_LIMIT))
    {
        co_buf_t* p_prev_buf = (co_buf_t*) co_list_tail(&(p_proc->buf_queue));
        p_proc->att[p_proc->att_cursor].rx_length += data_len;
        p_proc->att[p_proc->att_cursor].offset    += data_len;

        // Check if there is more data to read
        p_proc->complete =    (co_buf_data_len(p_buf) != (mtu - L2CAP_ATT_HEADER_LEN))
                           || (p_proc->att[p_proc->att_cursor].rx_length) >= (p_proc->att[p_proc->att_cursor].exp_length);

        // try to reuse previous buffer
        if(((p_prev_buf != NULL) && co_buf_tail_len(p_prev_buf) >=  data_len))
        {
            co_buf_copy_data_to_mem(p_buf , co_buf_tail(p_prev_buf), data_len);
            co_buf_tail_reserve(p_prev_buf, data_len);
        }
        else
        {
            // put buffer in queue
            co_list_push_back(&(p_proc->buf_queue), &(p_buf->hdr));

            // Acquire buffer
            co_buf_acquire(p_buf);

            gatt_env.total_mem_size += co_buf_size(p_buf);
        }
    }
    else
    {
        status = GAP_ERR_INSUFF_RESOURCES;
    }

    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, &(p_proc->hdr));

    return (status);
}


/**
 ****************************************************************************************
 * @brief Function called when L2CAP_ATT_RD_BY_TYPE_RSP attribute PDU is received.
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
__STATIC uint16_t gatt_cli_read_l2cap_att_rd_by_type_rsp_handler(uint8_t conidx, gatt_cli_read_proc_t* p_proc,
                                                                 l2cap_att_rd_by_type_rsp_t* p_pdu,
                                                                 co_buf_t* p_buf, uint16_t mtu)
{
    uint16_t data_len = co_buf_data_len(p_buf);
    uint16_t status   = GAP_ERR_NO_ERROR;

    // Check if there is more data to read
    p_proc->complete = (   (data_len < (mtu - L2CAP_ATT_HEADER_LEN - L2CAP_ATT_EACHLEN_LEN))
                        || (p_pdu->each_len < data_len));

    // Keep only information about first found attribute handle
    if(data_len > p_pdu->each_len)
    {
        co_buf_tail_release(p_buf, data_len - p_pdu->each_len);
    }

    // check that attribute handle is present in data
    if(p_pdu->each_len >= GATT_HANDLE_LEN)
    {
        p_proc->att[p_proc->att_cursor].hdl       = co_btohs(co_read16p(co_buf_data(p_buf)));
        p_proc->att[p_proc->att_cursor].rx_length = p_pdu->each_len - GATT_HANDLE_LEN;
        p_proc->att[p_proc->att_cursor].offset    = p_proc->att[p_proc->att_cursor].rx_length;

        co_buf_head_release(p_buf, GATT_HANDLE_LEN);

        // put buffer
        co_list_push_back(&(p_proc->buf_queue), &(p_buf->hdr));

        // Acquire buffer
        co_buf_acquire(p_buf);
        gatt_env.total_mem_size += co_buf_size(p_buf);
    }
    else
    {
        status = GAP_ERR_PROTOCOL_PROBLEM;
    }

    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, &(p_proc->hdr));

    return (status);
}

/**
 ****************************************************************************************
 * @brief Function called when L2CAP_ATT_RD_MULT_RSP attribute PDU is received.
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
__STATIC uint16_t gatt_cli_l2cap_att_rd_mult_rsp_handler(uint8_t conidx, gatt_cli_read_proc_t* p_proc,
                                                         l2cap_att_rd_mult_rsp_t* p_pdu,
                                                         co_buf_t* p_buf, uint16_t mtu)
{
    uint16_t data_len = co_buf_data_len(p_buf);
    uint16_t exp_len  = 0;
    uint8_t  cursor;

    p_proc->complete = (data_len != (mtu - L2CAP_ATT_HEADER_LEN));

    // compute expected size
    if(!p_proc->variable_len)
    {
        for(cursor = p_proc->att_cursor ; cursor < p_proc->nb_att ; cursor++)
        {
            exp_len += p_proc->att[cursor].exp_length;
        }

        // Check if there is more data to read
        p_proc->complete = (p_proc->complete || (data_len >= exp_len));
    }

    // put buffer
    co_list_push_back(&(p_proc->buf_queue), &(p_buf->hdr));

    // Acquire buffer
    co_buf_acquire(p_buf);
    gatt_env.total_mem_size += co_buf_size(p_buf);

    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, &(p_proc->hdr));

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
__STATIC void gatt_cli_read_proc_continue(uint8_t conidx, gatt_cli_read_proc_t* p_proc, uint8_t proc_state, uint16_t status)
{
    gatt_user_t* p_user  = gatt_user_get(p_proc->hdr.user_lid);
    bool finished = false;
    uint8_t proc_id = p_proc->hdr.proc_id;

    switch(proc_state)
    {
        case GATT_PROC_PDU_RX:
        {
            // check if an error occurs during parsing
            if(status != GAP_ERR_NO_ERROR) break;

            // read Multiple response received
            if((proc_id == GATT_PROC_READ_MULTIPLE) && (p_proc->att[p_proc->att_cursor].offset == 0))
            {
                uint8_t   cursor;
                co_buf_t* p_buf = (co_buf_t*) co_list_pick(&(p_proc->buf_queue));
                co_buf_t* p_buf_out = NULL;

                for(cursor = p_proc->att_cursor ; cursor < p_proc->nb_att ; cursor++)
                {
                    uint16_t data_len;

                    // extract current handle
                    if(p_proc->variable_len)
                    {
                        if(co_buf_data_len(p_buf) > L2CAP_ATT_VALLEN_LEN)
                        {
                            p_proc->att[cursor].exp_length = co_btohs(co_read16p(co_buf_data(p_buf)));
                            co_buf_head_release(p_buf, L2CAP_ATT_VALLEN_LEN);
                        }
                        else
                        {
                            // no need to keep current buffer
                            co_list_pop_front(&(p_proc->buf_queue));
                            gatt_env.total_mem_size -= co_buf_size(p_buf);
                            break;
                        }
                    }

                    // extract received length
                    data_len = co_buf_data_len(p_buf);
                    data_len = co_min(data_len, p_proc->att[cursor].exp_length);

                    p_proc->att[cursor].offset    = data_len;
                    p_proc->att[cursor].rx_length = data_len;

                    // check if all expected data has been received
                    if((!p_proc->complete) && (data_len < p_proc->att[cursor].exp_length))
                    {
                        break;
                    }
                    // check if buffer can be reused to provide value to host
                    else if((cursor == p_proc->nb_att - 1) || (data_len < p_proc->att[cursor].exp_length))
                    {
                        // clean-up read list
                        co_list_pop_front(&(p_proc->buf_queue));
                        gatt_env.total_mem_size -= co_buf_size(p_buf);

                        co_buf_reuse(p_buf);
                        p_buf_out = p_buf;
                    }
                    else
                    {
                        // allocate new buffer to provide data information to upper layer
                        if(co_buf_alloc(&p_buf_out, 0, data_len, 0) != CO_BUF_ERR_NO_ERROR)
                        {
                            status = GAP_ERR_INSUFF_RESOURCES;
                            break;
                        }

                        // copy buffer data
                        co_buf_copy(p_buf, p_buf_out, data_len, 0);
                        co_buf_head_release(p_buf, data_len);
                    }

                    // check if information can be provided to user
                    if(p_proc->complete || (data_len == p_proc->att[cursor].exp_length))
                    {
                        // Inform user about received data
                        p_user->p_cb->cli.cb_att_val(conidx, p_proc->hdr.user_lid, p_proc->hdr.dummy,
                                                     p_proc->att[cursor].hdl, 0, p_buf_out);
                        p_proc->att_cursor++;

                        co_buf_release(p_buf_out);
                    }
                    // Some more data must be read
                    else
                    {
                        break;
                    }
                }

                // procedure is over, send message complete
                if(p_proc->complete)
                {
                    finished = true;
                    break;
                }
            }

            if(p_proc->complete)
            {
                // Attribute length is equals to minimum between received length and expected length
                uint16_t  data_len   = co_min(p_proc->att[p_proc->att_cursor].rx_length, p_proc->att[p_proc->att_cursor].exp_length);
                co_buf_t* p_buf      = (co_buf_t*) co_list_pop_front(&(p_proc->buf_queue));
                uint16_t  remain_len;

                // ensure that buffer length is not greater than data length
                if(data_len < co_buf_data_len(p_buf))
                {
                    co_buf_tail_release(p_buf, co_buf_data_len(p_buf) - data_len);
                    remain_len = 0;
                }
                else
                {
                    remain_len = data_len - co_buf_data_len(p_buf);
                }

                // check if another buffer must be allocated
                if(remain_len > co_buf_tail_len(p_buf))
                {
                    co_buf_t* p_buf_complete = NULL;

                    // duplicate buffer into a larger buffer
                    if(co_buf_duplicate(p_buf, &p_buf_complete, 0, remain_len) != CO_BUF_ERR_NO_ERROR)
                    {
                        // not enough data to create an output buffer
                        status = GAP_ERR_INSUFF_RESOURCES;
                        co_buf_release(p_buf);
                        break;
                    }
                    // release copied buffer
                    gatt_env.total_mem_size -= co_buf_size(p_buf);
                    co_buf_release(p_buf);
                    gatt_env.total_mem_size += co_buf_size(p_buf_complete);
                    p_buf = p_buf_complete;
                }

                // put remain received data into buffer
                while(!co_list_is_empty(&(p_proc->buf_queue)))
                {
                    co_buf_t* p_buf_in = (co_buf_t*) co_list_pop_front(&(p_proc->buf_queue));
                    uint16_t  copy_len = co_buf_data_len(p_buf_in);
                    copy_len = co_min(copy_len, remain_len);

                    // Copy data in buffer tail
                    co_buf_copy_data_to_mem(p_buf_in, co_buf_tail(p_buf), copy_len);
                    // update buffer data length
                    co_buf_tail_reserve(p_buf, copy_len);
                    // update remaining copy length
                    remain_len -= copy_len;
                    // release input buffer
                    gatt_env.total_mem_size -= co_buf_size(p_buf_in);
                    co_buf_release(p_buf_in);
                }

                // Inform user about received data
                p_user->p_cb->cli.cb_att_val(conidx, p_proc->hdr.user_lid, p_proc->hdr.dummy,
                                             p_proc->att[p_proc->att_cursor].hdl,
                                             p_proc->att[p_proc->att_cursor].offset - p_proc->att[p_proc->att_cursor].rx_length,
                                             p_buf);

                // release output buffer
                gatt_env.total_mem_size -= co_buf_size(p_buf);
                co_buf_release(p_buf);

                p_proc->att_cursor++;

                if(p_proc->att_cursor == p_proc->nb_att)
                {
                    finished = true;
                    break;
                }
            }
            else
            {
                // mark that buffer is reused to let the bearer continue reception
                co_buf_reuse((co_buf_t*) co_list_tail(&(p_proc->buf_queue)));
            }
        }
        // no break;
        case GATT_PROC_START:
        {
            gatt_proc_pdu_handler_cb cb_rsp_handler = NULL;
            l2cap_att_pdu_t pdu;
            co_buf_t* p_buf = NULL;

            // allocate buffer used for attribute transmission
            if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, GATT_CLI_READ_BUFFER_TAIL_LEN) != CO_BUF_ERR_NO_ERROR)
            {
                status = GAP_ERR_INSUFF_RESOURCES;
                break;
            }

            switch(proc_id)
            {
                case GATT_PROC_READ_MULTIPLE:
                {
                    // start of read multiple
                    if(p_proc->att[p_proc->att_cursor].offset == 0)
                    {
                        uint8_t cursor;
                        cb_rsp_handler = (gatt_proc_pdu_handler_cb) gatt_cli_l2cap_att_rd_mult_rsp_handler;

                        pdu.code = (p_proc->variable_len ? L2CAP_ATT_RD_MULT_VAR_REQ_OPCODE : L2CAP_ATT_RD_MULT_REQ_OPCODE);

                        // Copy list of handles
                        for(cursor = p_proc->att_cursor ; cursor < p_proc->nb_att ; cursor++)
                        {
                            co_write16p(co_buf_tail(p_buf), co_htobs(p_proc->att[cursor].hdl));
                            co_buf_tail_reserve(p_buf, GATT_HANDLE_LEN);
                        }

                        break;
                    } // continue read multiple
                }
                // no break;

                case GATT_PROC_READ_BY_UUID:
                {
                    if(proc_id == GATT_PROC_READ_BY_UUID)
                    {
                        if(p_proc->att[p_proc->att_cursor].offset == 0)
                        {
                            cb_rsp_handler = (gatt_proc_pdu_handler_cb) gatt_cli_read_l2cap_att_rd_by_type_rsp_handler;

                            pdu.code                = L2CAP_ATT_RD_BY_TYPE_REQ_OPCODE;
                            pdu.rd_by_type_req.shdl = p_proc->start_hdl;
                            pdu.rd_by_type_req.ehdl = p_proc->end_hdl;

                            // Copy UUID expected
                            if(p_proc->uuid_type == GATT_UUID_16)
                            {
                                co_buf_tail_reserve(p_buf, GATT_UUID_16_LEN);
                                memcpy(co_buf_data(p_buf), p_proc->uuid, GATT_UUID_16_LEN);
                            }
                            else
                            {
                                co_buf_tail_reserve(p_buf, GATT_UUID_128_LEN);
                                // convert UUID to 128-bit UUID
                                gatt_uuid128_convert(p_proc->uuid, p_proc->uuid_type, co_buf_data(p_buf));
                            }

                            break;
                        }
                        // else perform a read blob to complete read
                    }
                }
                // no break;
                case GATT_PROC_READ:
                {
                    cb_rsp_handler = (gatt_proc_pdu_handler_cb) gatt_cli_read_l2cap_att_rd_rsp_handler;

                    // read without offset
                    if(p_proc->att[p_proc->att_cursor].offset == 0)
                    {
                        pdu.code               = L2CAP_ATT_RD_REQ_OPCODE;
                        pdu.rd_req.handle      = p_proc->att[p_proc->att_cursor].hdl;
                    }
                    // read with offset
                    else
                    {
                        pdu.code               = L2CAP_ATT_RD_BLOB_REQ_OPCODE;
                        pdu.rd_blob_req.handle = p_proc->att[p_proc->att_cursor].hdl;
                        pdu.rd_blob_req.offset = p_proc->att[p_proc->att_cursor].offset;
                    }
                } break;

                default: { ASSERT_ERR(0); } break;
            }

            // Ask for PDU transmission
            status = gatt_proc_pdu_send(conidx, &(p_proc->hdr), &pdu, p_buf, cb_rsp_handler);

            // release buffer
            co_buf_release(p_buf);
        } break;

        case GATT_PROC_ERROR: // Handle error detection
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
        // perform a buffer release
        while(!co_list_is_empty(&(p_proc->buf_queue)))
        {
            co_buf_t* p_buf = (co_buf_t*) co_list_pop_front(&(p_proc->buf_queue));
            gatt_env.total_mem_size -= co_buf_size(p_buf);
            co_buf_release(p_buf);
        }

        // Inform user that procedure is over
        #if (HOST_MSG_API)
        gatt_proc_cur_set((gatt_proc_t*) p_proc);
        #endif // (HOST_MSG_API)
        p_user->p_cb->cli.cb_read_cmp(conidx, p_proc->hdr.user_lid, p_proc->hdr.dummy, status);

        // Pop Procedure
        gatt_proc_pop(conidx, &(p_proc->hdr), true);
    }
}

/**
 ****************************************************************************************
 * @brief Create the internal read procedure
 *
 * @param[in]  conidx       Connection index
 * @param[in]  user_lid     GATT User Local identifier
 * @param[in]  dummy        Dummy parameter whose meaning is upper layer dependent and
 *                          which is returned in command complete.
 * @param[in]  proc_id      Procedure Identifier (see enum #gatt_proc_id)
 * @param[in]  start_hdl    Search start handle
 * @param[in]  end_hdl      Search end handle
 * @param[in]  uuid_type    UUID Type (see enum #gatt_uuid_type)
 * @param[in]  p_uuid       Pointer to searched attribute UUID (LSB First)
 * @param[in]  offset       Data offset
 * @param[in]  nb_att       Number of attribute
 * @param[in]  p_atts       Pointer to list of attribute
 *                          If Attribute length is zero (consider length unknown - read all)
 *
 * @return Status of the function execution (see enum #hl_err)
 *         Consider status only if an error occurs; else wait for execution completion
 ****************************************************************************************
 */
__STATIC uint16_t gatt_cli_read_proc_create(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t proc_id,
                                            uint16_t start_hdl, uint16_t end_hdl, uint8_t uuid_type, const uint8_t* p_uuid,
                                            uint16_t offset, uint8_t nb_att, const gatt_att_t* p_atts)
{
    uint16_t status;
    gatt_user_t* p_user  = gatt_user_get(user_lid);

    // Ensure that user can handle the procedure
    if((p_user == NULL) || (p_user->p_cb->cli.cb_att_val == NULL) || (p_user->p_cb->cli.cb_read_cmp == NULL))
    {
        status = GAP_ERR_NOT_SUPPORTED;
    }
    else
    {
        gatt_cli_read_proc_t* p_proc;
        // Create procedure
        status = gatt_proc_create(conidx, user_lid, dummy, proc_id, L2CAP_LE_MTU_MIN,
                                  sizeof(gatt_cli_read_proc_t) + (sizeof(gatt_cli_read_info_t) * nb_att),
                                  (gatt_proc_cb) gatt_cli_read_proc_continue, (gatt_proc_t**) &p_proc);

        if(status == GAP_ERR_NO_ERROR)
        {
            uint8_t cursor;

            p_proc->variable_len = false;

            // fill procedure parameters
            for(cursor = 0 ; cursor < nb_att ; cursor++)
            {
                p_proc->att[cursor].hdl        = p_atts[cursor].hdl;
                p_proc->att[cursor].offset     = offset;
                p_proc->att[cursor].exp_length = (p_atts[cursor].length == 0)
                                               ? GATT_CLI_READ_UNKNOWN_LEN
                                               : p_atts[cursor].length ;
                p_proc->att[cursor].rx_length  = 0;

                if(p_atts[cursor].length == 0)
                {
                    p_proc->variable_len = true;
                }
            }

            p_proc->nb_att            = nb_att;
            p_proc->start_hdl         = start_hdl;
            p_proc->end_hdl           = end_hdl;
            p_proc->att_cursor        = 0;

            p_proc->uuid_type         = uuid_type;
            if(p_uuid != NULL)
            {
                memcpy(p_proc->uuid, p_uuid, GATT_UUID_128_LEN);
            }

            co_list_init(&(p_proc->buf_queue));

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

uint16_t gatt_cli_read(uint8_t conidx, uint8_t user_lid, uint16_t dummy,
                       uint16_t hdl, uint16_t offset, uint16_t length)
{
    uint16_t status;

    // Check parameters
    if(hdl == GATT_INVALID_HDL)
    {
        status = ATT_ERR_INVALID_HANDLE;
    }
    else
    {
        gatt_att_t att = { .hdl = hdl, .length = length, };

        // Create procedure
        status = gatt_cli_read_proc_create(conidx, user_lid, dummy, GATT_PROC_READ, 0, 0 , 0, NULL, offset, 1, &att);
    }

    return (status);
}

uint16_t gatt_cli_read_by_uuid(uint8_t conidx, uint8_t user_lid, uint16_t dummy,
                               uint16_t start_hdl, uint16_t end_hdl, uint8_t uuid_type, const uint8_t* p_uuid)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    // Check parameters
    if((uuid_type > GATT_UUID_128) || (start_hdl > end_hdl))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else
    {
        gatt_att_t att = { .hdl = GATT_INVALID_HDL, .length = 0, };

        // Create procedure
        status = gatt_cli_read_proc_create(conidx, user_lid, dummy, GATT_PROC_READ_BY_UUID, start_hdl, end_hdl ,
                                           uuid_type, p_uuid, 0, 1, &att);
    }
    return (status);
}

uint16_t gatt_cli_read_multiple(uint8_t conidx, uint8_t user_lid, uint16_t dummy,
                                uint8_t nb_att, const gatt_att_t* p_atts)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    // check maximum parameter range
    if((nb_att == 0) || (nb_att > GATT_RD_MULTIPLE_MAX_NB_ATT))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else
    {
        // Check attribute handles
        uint8_t cursor;
        for(cursor = 0 ; cursor < nb_att ; cursor++)
        {
            if(p_atts[cursor].hdl == GATT_INVALID_HDL)
            {
                status  = ATT_ERR_INVALID_HANDLE;
                break;
            }
        }
    }

    if(status == GAP_ERR_NO_ERROR)
    {

        // Create procedure
        status = gatt_cli_read_proc_create(conidx, user_lid, dummy, GATT_PROC_READ_MULTIPLE, 0, 0 , 0, NULL, 0,
                                           nb_att, p_atts);
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
 * @brief Handle Read command initiated by GATT client user
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_read_cmd_handler(gatt_cli_read_cmd_t* p_cmd, uint16_t src_id)
{
    // start discovery
    uint16_t status = gatt_cli_read(p_cmd->conidx, p_cmd->user_lid, p_cmd->dummy,
                                    p_cmd->hdl, p_cmd->offset, p_cmd->length);

    if(status != GAP_ERR_NO_ERROR)
    {
        // send command completion - in error
        gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
    }
    else
    {
        // store command code
        gatt_cli_read_proc_t* p_proc = (gatt_cli_read_proc_t*) gatt_proc_cur_get();
        p_proc->msg_cmd_code = p_cmd->cmd_code;
    }
}

/**
 ****************************************************************************************
 * @brief Handle Read By UUID command initiated by GATT client user
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_read_by_uuid_cmd_handler(gatt_cli_read_by_uuid_cmd_t* p_cmd, uint16_t src_id)
{
    // start discovery
    uint16_t status = gatt_cli_read_by_uuid(p_cmd->conidx, p_cmd->user_lid, p_cmd->dummy,
                                            p_cmd->start_hdl, p_cmd->end_hdl, p_cmd->uuid_type, p_cmd->uuid);

    if(status != GAP_ERR_NO_ERROR)
    {
        // send command completion - in error
        gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
    }
    else
    {
        // store command code
        gatt_cli_read_proc_t* p_proc = (gatt_cli_read_proc_t*) gatt_proc_cur_get();
        p_proc->msg_cmd_code = p_cmd->cmd_code;
    }
}

/**
 ****************************************************************************************
 * @brief Handle Read Multiple command initiated by GATT client user
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_read_multiple_cmd_handler(gatt_cli_read_multiple_cmd_t* p_cmd, uint16_t src_id)
{
    // start discovery
    uint16_t status = gatt_cli_read_multiple(p_cmd->conidx, p_cmd->user_lid, p_cmd->dummy,
                                             p_cmd->nb_att, p_cmd->atts);

    if(status != GAP_ERR_NO_ERROR)
    {
        // send command completion - in error
        gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
    }
    else
    {
        // store command code
        gatt_cli_read_proc_t* p_proc = (gatt_cli_read_proc_t*) gatt_proc_cur_get();
        p_proc->msg_cmd_code = p_cmd->cmd_code;
    }
}


/**
 ****************************************************************************************
 * @brief This function is called when GATT client user read procedure is over.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] status        Status of the procedure (see enum #hl_err)
 ****************************************************************************************
 */
void gatt_cli_read_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        gatt_cli_read_proc_t* p_proc = (gatt_cli_read_proc_t*) gatt_proc_cur_get();

        // send back command complete event
        gatt_msg_send_proc_cmp_evt(p_proc->msg_cmd_code, dummy, conidx, p_user->dest_task_nbr, user_lid, status);
    }
}


/**
 ****************************************************************************************
 * @brief This function is called during a read procedure when attribute value is retrieved
 *        form peer device.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] hdl           Attribute handle
 * @param[in] offset        Data offset
 * @param[in] p_data        Pointer to buffer that contains attribute value starting from offset
 ****************************************************************************************
 */
void gatt_cli_att_val_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t hdl, uint16_t offset,
                         co_buf_t* p_data)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        uint16_t data_len = co_buf_data_len(p_data);
        gatt_cli_att_val_ind_t* p_ind = KE_MSG_ALLOC_DYN(GATT_IND, p_user->dest_task_nbr, TASK_GATT, gatt_cli_att_val_ind,
                                                         data_len);

        // prepare indication to send
        if(p_ind != NULL)
        {
            p_ind->ind_code     = GATT_CLI_ATT_VAL;
            p_ind->dummy        = dummy;
            p_ind->user_lid     = user_lid;
            p_ind->conidx       = conidx;

            p_ind->hdl          = hdl;
            p_ind->offset       = offset;
            p_ind->value_length = data_len;

            co_buf_copy_data_to_mem(p_data, p_ind->value, data_len);

            // send message to host
            ke_msg_send(p_ind);
        }
    }
}

#endif // (HOST_MSG_API)

#else  // !(BLE_GATT_CLI)
uint16_t gatt_cli_read(uint8_t conidx, uint8_t user_lid, uint16_t dummy,
                       uint16_t hdl, uint16_t offset, uint16_t length)
{
    return (GAP_ERR_NOT_SUPPORTED);
}

uint16_t gatt_cli_read_by_uuid(uint8_t conidx, uint8_t user_lid, uint16_t dummy,
                               uint16_t start_hdl, uint16_t end_hdl, uint8_t uuid_type, const uint8_t* p_uuid)
{
    return (GAP_ERR_NOT_SUPPORTED);
}

uint16_t gatt_cli_read_multiple(uint8_t conidx, uint8_t user_lid, uint16_t dummy,
                                uint8_t nb_att, const gatt_att_t* p_atts)
{
    return (GAP_ERR_NOT_SUPPORTED);
}
#endif // (BLE_GATT_CLI)
#endif // (BLE_GATT)
/// @} GATT

