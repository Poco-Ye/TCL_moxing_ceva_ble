/**
 ****************************************************************************************
 * @file l2cap_chan_tx.c
 *
 * @brief  L2CAP channel management - Packet Transmission
 *
 * Copyright (C) RivieraWaves 2017-2021
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
#include "rwip_config.h"    // IP configuration
#if (BLE_L2CAP)
#include "l2cap_int.h"      // Internals
#include "co_math.h"        // Mathematical utilities
#include "co_endian.h"      // Endianess management

#if BLE_EMB_PRESENT
#include "ble_util_buf.h"   // ACL buffer usage
#include "reg_access.h"     // EM access API
#endif // BLE_EMB_PRESENT

#include "dbg_swdiag.h"     // SW Diags
#include "dbg_trc.h"        // SW Trace
#include "hl_hci.h"         // HL HCI Interface
#include "hci.h"            // HCI communication

#include "gapc.h"           // Retrieve connection

#include "ke_mem.h"         // Memory management
#include <string.h>         // For memory copy



/*
 * MACROS
 ****************************************************************************************
 */

// Buffer Memory Access
#if (EMB_PRESENT)
#define l2cap_chan_ll_buf_wr16p(buf_out, value)         em_wr16p((buf_out), (value))
#define l2cap_chan_ll_buf_wr(buf_out, buf_in, size)     em_wr((buf_in), (buf_out), (size))
#else // (!EMB_PRESENT)
#define l2cap_chan_ll_buf_wr16p(buf_out, value)         co_write16p(((uint8_t*) (buf_out)), (value))
#define l2cap_chan_ll_buf_wr(buf_out, buf_in, size)     memcpy(((uint8_t*) (buf_out)), (buf_in), (size))
#endif // (EMB_PRESENT)

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
typedef struct l2cap_buf_tx_meta l2cap_buf_tx_meta_t;

/// Callback used to retrieve channel Maximum transmit size
typedef uint16_t (*l2cap_chan_get_tx_mtu_size_cb)(l2cap_chan_t* p_chan);

/// Callback used to know if channel is ready to send data (push on TX queue)
typedef bool (*l2cap_chan_is_ready_to_send_cb)(l2cap_chan_t* p_chan);

/// Callback used to prepare ACL buffer
/// @return Return SDU size to copy
typedef uint16_t (*l2cap_chan_prepare_buffer_cb)(l2cap_chan_t* p_chan, co_buf_t* p_sdu, l2cap_buf_tx_meta_t* p_meta,
                                                 struct hci_acl_data* p_data_tx, uint32_t* p_buf_ptr, uint16_t max_size);

/// Callback used to get if more data expected to be transmitted on channel
typedef bool (*l2cap_chan_is_more_data_to_send_cb)(l2cap_chan_t* p_chan);

/// Callback interface used for SDU transmission
typedef struct l2cap_chan_tx_sdu_cb_itf
{
    /// Callback used for SDU send preparation
    l2cap_chan_get_tx_mtu_size_cb      cb_get_tx_mtu_size;
    /// Callback used to know if channel is ready to send data (push on TX queue)
    l2cap_chan_is_ready_to_send_cb     cb_is_ready_to_send;
    /// Callback used to prepare ACL buffer
    l2cap_chan_prepare_buffer_cb       cb_prepare_buffer;
    /// Callback used to get if more data expected to be transmitted on channel
    l2cap_chan_is_more_data_to_send_cb cb_is_more_data_to_send;
} l2cap_chan_tx_sdu_cb_itf_t;

/// Buffer transmission meta-data
typedef struct l2cap_buf_tx_meta
{
    /// Pointer to callback interface
    const l2cap_chan_tx_sdu_cb_itf_t* p_itf;
    /// Dummy parameter provided by upper layer for command execution
    uint16_t                          dummy;
    /// Transmission data cursor
    uint16_t                          data_cursor;
    /// Size remain to be used into segment (0: new segment can be started)
    uint16_t                          seg_remain_size;
    /// Token used for TX flow off
    uint16_t                          tx_flow_off_token;
} l2cap_buf_tx_meta_t;

#if (RW_DEBUG)
/// Debug Buffer transmission meta-data
typedef struct l2cap_buf_tx_debug_meta
{
    /// Buffer meta-data inherited structure
    l2cap_buf_tx_meta_t               hdr;
    /// Pointer to parent callback interface
    const l2cap_chan_tx_sdu_cb_itf_t* p_parent_itf;
    /// Debug bit field (see enum #l2cap_dbg_bf)
    uint8_t                           dbg_bf;
} l2cap_buf_tx_debug_meta_t;
#endif // (RW_DEBUG)

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/*
 * FIXED CHANNEL Callback interface
 ****************************************************************************************
 */

/// Write 16-bit value
__INLINE void l2cap_chan_write_16bit_value(uint32_t* p_buf_ptr, uint16_t value)
{
    l2cap_chan_ll_buf_wr16p(*p_buf_ptr, co_htobs(value));
    *p_buf_ptr         += 2;
}

/// Write L2CAP PDU header
__INLINE void l2cap_chan_write_header(uint32_t* p_buf_ptr, uint16_t cid, uint16_t pdu_length)
{
    // PDU Length
    l2cap_chan_write_16bit_value(p_buf_ptr, pdu_length);
    // Channel identifier
    l2cap_chan_write_16bit_value(p_buf_ptr, cid);
}

/// Retrieve Fixed channel Maximum transmit size
__STATIC uint16_t l2cap_chan_fix_get_tx_mtu_size(l2cap_chan_t* p_chan)
{
    return p_chan->mtu;
}

/// Used to know if channel is ready to send data (push on TX queue) on Fixed L2CAP channel
__STATIC bool l2cap_chan_fix_is_ready_to_send(l2cap_chan_t* p_chan)
{
    return (!GETB(p_chan->config_bf, L2CAP_CHAN_PDU_TX_PAUSED));
}

/// SDU segment send on Fixed channel
__STATIC uint16_t l2cap_chan_fix_prepare_buffer(l2cap_chan_t* p_chan, co_buf_t* p_sdu, l2cap_buf_tx_meta_t* p_meta,
                                                struct hci_acl_data* p_data_tx, uint32_t* p_buf_ptr, uint16_t max_size)
{
    // check if a new SDU is transmitted
    SETF(p_data_tx->conhdl_pb_bc_flag, HCI_ACL_HDR_PB_FLAG, PBF_1ST_HL_FRAG);
    // update segment size with size transmit maximum packet size
    p_meta->seg_remain_size = co_buf_data_len(p_sdu);

    // Write L2Cap Header
    l2cap_chan_write_header(p_buf_ptr, p_chan->tx_cid, p_meta->seg_remain_size);
    p_data_tx->length  += L2CAP_HEADER_LEN;
    max_size           -= L2CAP_HEADER_LEN;

    return co_min(p_meta->seg_remain_size, max_size);
}


/// Get if more data to send onto a fix channel
__STATIC bool l2cap_chan_fix_is_more_data_to_send(l2cap_chan_t* p_chan)
{
    return !(co_list_is_empty(&(p_chan->tx_queue)));
}

/// Fixed channel callback interface
__STATIC const l2cap_chan_tx_sdu_cb_itf_t  l2cap_chan_tx_fix_sdu_cb_itf =
{
    .cb_get_tx_mtu_size       = l2cap_chan_fix_get_tx_mtu_size,
    .cb_is_ready_to_send      = l2cap_chan_fix_is_ready_to_send,
    .cb_prepare_buffer        = l2cap_chan_fix_prepare_buffer,
    .cb_is_more_data_to_send  = l2cap_chan_fix_is_more_data_to_send,
};


/*
 * DYNAMIC CHANNEL Callback interface
 ****************************************************************************************
 */

/// Retrieve Dynamic channel Maximum transmit size
__STATIC uint16_t l2cap_chan_dyn_get_tx_mtu_size(l2cap_chan_coc_t* p_chan)
{
    return p_chan->tx_mtu;
}

/// Used to know if channel is ready to send data (push on TX queue) on Dynamic L2CAP channel
__STATIC bool l2cap_chan_dyn_is_ready_to_send(l2cap_chan_coc_t* p_chan)
{
    return (!GETB(p_chan->config_bf, L2CAP_CHAN_PDU_TX_PAUSED)
            && (   (p_chan->tx_credit > 0)
            #if(RW_DEBUG)
                || !GETB(p_chan->config_bf, L2CAP_CHAN_CREDIT_FLOW_EN)
            #endif // (RW_DEBUG)
            ));
}

/// Prepare segment to transmit for a dynamic channel
__STATIC uint16_t l2cap_chan_dyn_prepare_buffer(l2cap_chan_coc_t* p_chan, co_buf_t* p_sdu, l2cap_buf_tx_meta_t* p_meta,
                                                struct hci_acl_data* p_data_tx, uint32_t* p_buf_ptr, uint16_t max_size)
{
    uint16_t sdu_length         = co_buf_data_len(p_sdu);
    uint16_t max_segment_length = p_chan->tx_mps;
    uint16_t pdu_length         = 0;
    SETF(p_data_tx->conhdl_pb_bc_flag, HCI_ACL_HDR_PB_FLAG, PBF_1ST_HL_FRAG);


    #if (RW_DEBUG)
    // decrement number of credit if supported
    if(GETB(p_chan->config_bf, L2CAP_CHAN_CREDIT_FLOW_EN))
    #endif // (RW_DEBUG)
    {
        ASSERT_INFO(p_chan->tx_credit != 0, p_chan->conidx, p_chan->chan_lid);
        p_chan->tx_credit -= 1;
    }

    // check if a new SDU is transmitted
    if(p_meta->data_cursor == 0)
    {
        max_segment_length -= L2CAP_SDU_LEN;
        pdu_length         += L2CAP_SDU_LEN;
    }

    // update segment size with size transmit maximum packet size
    p_meta->seg_remain_size  = co_min(max_segment_length, sdu_length - p_meta->data_cursor);
    pdu_length              += p_meta->seg_remain_size;

    // Write L2Cap Header
    l2cap_chan_write_header(p_buf_ptr, p_chan->tx_cid, pdu_length);
    p_data_tx->length += L2CAP_HEADER_LEN;
    max_size          -= L2CAP_HEADER_LEN;

    // Update buffer pointer cursor for SDU field
    if(p_meta->data_cursor == 0)
    {
        // write sdu length
        l2cap_chan_write_16bit_value(p_buf_ptr, sdu_length);
        max_size          -= L2CAP_SDU_LEN;
        p_data_tx->length += L2CAP_SDU_LEN;
    }

    return (co_min(p_meta->seg_remain_size, max_size));
}

/// Get if more data to send onto a dynamic channel
__STATIC bool l2cap_chan_dyn_is_more_data_to_send(l2cap_chan_coc_t* p_chan)
{
    return (   !(co_list_is_empty(&(p_chan->tx_queue)))
            && (   (p_chan->tx_credit > 0)
            #if(RW_DEBUG)
                || !GETB(p_chan->config_bf, L2CAP_CHAN_CREDIT_FLOW_EN)
            #endif // (RW_DEBUG)
            ));
}


/// Dynamic channel callback interface
__STATIC const l2cap_chan_tx_sdu_cb_itf_t  l2cap_chan_tx_dyn_sdu_cb_itf =
{
    .cb_get_tx_mtu_size       = (l2cap_chan_get_tx_mtu_size_cb)      l2cap_chan_dyn_get_tx_mtu_size,
    .cb_is_ready_to_send      = (l2cap_chan_is_ready_to_send_cb)     l2cap_chan_dyn_is_ready_to_send,
    .cb_prepare_buffer        = (l2cap_chan_prepare_buffer_cb)       l2cap_chan_dyn_prepare_buffer,
    .cb_is_more_data_to_send  = (l2cap_chan_is_more_data_to_send_cb) l2cap_chan_dyn_is_more_data_to_send,
};


/*
 * DEBUG CHANNEL Callback interface
 ****************************************************************************************
 */
#if (RW_DEBUG)
/// Retrieve Dynamic channel Maximum transmit size
__STATIC uint16_t l2cap_chan_debug_get_tx_mtu_size(l2cap_chan_t* p_chan)
{
    return (GETB(p_chan->config_bf, L2CAP_CHAN_EN)
            ? l2cap_chan_fix_get_tx_mtu_size(p_chan)
            : l2cap_chan_dyn_get_tx_mtu_size((l2cap_chan_coc_t*)p_chan));
}


/// Used to know if channel is ready to send data (push on TX queue) on debug L2CAP channel
__STATIC bool l2cap_chan_debug_is_ready_to_send(l2cap_chan_t* p_chan)
{
    return (GETB(p_chan->config_bf, L2CAP_CHAN_EN)
            ? l2cap_chan_fix_is_ready_to_send(p_chan)
            : l2cap_chan_dyn_is_ready_to_send((l2cap_chan_coc_t*)p_chan));
}


/// Prepare segment to transmit for a dynamic channel
__STATIC uint16_t l2cap_chan_debug_prepare_buffer(l2cap_chan_t* p_chan, co_buf_t* p_sdu, l2cap_buf_tx_debug_meta_t* p_meta,
                                                  struct hci_acl_data* p_data_tx, uint32_t* p_buf_ptr, uint16_t max_size)
{
    uint16_t tx_mps =(GETB(p_chan->config_bf, L2CAP_CHAN_EN) ? p_chan->mtu : ((l2cap_chan_coc_t*)p_chan)->tx_mps);
    uint16_t data_len;


    // Segment header present in buffer payload
    if(GETB(p_meta->dbg_bf, L2CAP_DBG_SEG_HEADER_PRESENT))
    {
        uint16_t remain_size = co_buf_data_len(p_sdu) - p_meta->hdr.data_cursor;
        p_meta->hdr.seg_remain_size = co_min(remain_size, tx_mps + L2CAP_HEADER_LEN);
        data_len = p_meta->hdr.seg_remain_size;
    }
    else
    {
        // use parent prepare buffer function
        data_len = p_meta->p_parent_itf->cb_prepare_buffer(p_chan, p_sdu, &(p_meta->hdr), p_data_tx, p_buf_ptr, max_size);
    }


    // force packet type (start or continue fragment)
    SETF(p_data_tx->conhdl_pb_bc_flag, HCI_ACL_HDR_PB_FLAG,
            GETB(p_meta->dbg_bf, L2CAP_DBG_SEG_CONTINUE) ? PBF_CONT_HL_FRAG : PBF_1ST_HL_FRAG);

    return(data_len);
}


/// Get if more data to send onto a dynamic channel
__STATIC bool l2cap_chan_debug_is_more_data_to_send(l2cap_chan_t* p_chan)
{
    return (GETB(p_chan->config_bf, L2CAP_CHAN_EN)
            ? l2cap_chan_fix_is_more_data_to_send(p_chan)
            : l2cap_chan_dyn_is_more_data_to_send((l2cap_chan_coc_t*)p_chan));
}


/// Debug channel callback interface
__STATIC const l2cap_chan_tx_sdu_cb_itf_t  l2cap_chan_tx_debug_sdu_cb_itf =
{
    .cb_get_tx_mtu_size       = (l2cap_chan_get_tx_mtu_size_cb)      l2cap_chan_debug_get_tx_mtu_size,
    .cb_is_ready_to_send      = (l2cap_chan_is_ready_to_send_cb)     l2cap_chan_debug_is_ready_to_send,
    .cb_prepare_buffer        = (l2cap_chan_prepare_buffer_cb)       l2cap_chan_debug_prepare_buffer,
    .cb_is_more_data_to_send  = (l2cap_chan_is_more_data_to_send_cb) l2cap_chan_debug_is_more_data_to_send,
};
#endif // (RW_DEBUG)

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */


/// Prepare segment to transmit for a dynamic channel
__STATIC uint16_t l2cap_chan_continue_prepare_buffer(l2cap_chan_t* p_chan, co_buf_t* p_sdu, l2cap_buf_tx_meta_t* p_meta,
                                                     struct hci_acl_data* p_data_tx, uint32_t* p_buf_ptr, uint16_t max_size)
{
    SETF(p_data_tx->conhdl_pb_bc_flag, HCI_ACL_HDR_PB_FLAG, PBF_CONT_HL_FRAG);
    return co_min(p_meta->seg_remain_size, max_size);
}


__STATIC const l2cap_chan_tx_sdu_cb_itf_t* l2cap_chan_tx_get_cb_itf(uint8_t conidx, uint8_t chan_lid, l2cap_chan_t** pp_chan)
{
    const l2cap_chan_tx_sdu_cb_itf_t* p_itf = NULL;
    l2cap_chan_t* p_chan = l2cap_chan_get_env(conidx, chan_lid);

    // verify channel presence and if transmission can be performed
    if((p_chan != NULL) && GETB(p_chan->config_bf, L2CAP_CHAN_EN))
    {
        p_itf = (GETB(p_chan->config_bf, L2CAP_CHAN_FIX)) ? &l2cap_chan_tx_fix_sdu_cb_itf : &l2cap_chan_tx_dyn_sdu_cb_itf;
        *pp_chan = p_chan;
    }

    return (p_itf);
}


/// Allocate a buffer for ACL HCI Transmission over LE transport
__STATIC uint32_t l2cap_chan_tx_le_buf_alloc(uint16_t conhdl, uint16_t ll_buffer_size)
{
    uint32_t buf_ptr;
    #if BLE_EMB_PRESENT
    buf_ptr = ble_util_buf_acl_tx_alloc();
    #else // !(BLE_EMB_PRESENT)
    uint8_t* p_acl_buf;
    // allocate buffer in message heap
    p_acl_buf  = ke_malloc_system(ll_buffer_size + HCI_ACL_HDR_LEN + HCI_TRANSPORT_HDR_LEN, KE_MEM_KE_MSG);
    // move pointer to payload beginning
    p_acl_buf += HCI_ACL_HDR_LEN + HCI_TRANSPORT_HDR_LEN;
    buf_ptr = (uint32_t) p_acl_buf;
    #endif // BLE_EMB_PRESENT

    l2cap_env.ll_buf_nb_avail--;
    return (buf_ptr);
}

/// Release SDU buffer and inform upper layer application
__STATIC void l2cap_chan_tx_release_sdu(l2cap_chan_t* p_chan, co_buf_t* p_sdu, l2cap_buf_tx_meta_t* p_meta,
                                        uint16_t status)
{
    uint16_t dummy = p_meta->dummy;
    bool buf_free = (p_sdu->acq_cnt == 1);

    if(!buf_free) { co_buf_release(p_sdu); } // Pre-buffer release can be performed
    p_chan->p_cb->cb_sdu_sent(p_chan->conidx, dummy, p_chan->chan_lid, status, p_sdu);
    if(buf_free)  { co_buf_release(p_sdu); } // Post Buffer release must be performed
}

bool l2cap_chan_segment_send(uint8_t conidx, l2cap_con_env_t* p_con, l2cap_chan_t* p_chan, uint8_t nb_buffer,
                             uint16_t ll_buffer_size, uint32_t (*cb_tx_buf_alloc)(uint16_t, uint16_t)) // TODO create typedef
{
    DBG_SWDIAG(HL, L2CAP_TX, 1);
    ASSERT_ERR(!co_list_is_empty(&(p_chan->tx_queue)));

    bool is_more_data_to_send = true;
    bool is_segment_done = false;

    // retrieve SDU to transmit and corresponding environment
    co_buf_t* p_sdu = (co_buf_t*) co_list_pick(&(p_chan->tx_queue));
    l2cap_buf_tx_meta_t* p_meta = (l2cap_buf_tx_meta_t*) co_buf_metadata(p_sdu);
    uint16_t conhdl = gapc_get_conhdl(conidx);

    const l2cap_chan_tx_sdu_cb_itf_t* p_itf = p_meta->p_itf;
    // Get if start segment or continuation segment must be transmitted
    l2cap_chan_prepare_buffer_cb cb_prepare_buffer = (p_meta->seg_remain_size == 0)
                                                   ? p_itf->cb_prepare_buffer
                                                   : l2cap_chan_continue_prepare_buffer;

    // loop until we have enough buffers
    while(nb_buffer > 0)
    {
        uint32_t buf_ptr;
        uint16_t data_length;

        // Allocate the message for Controller
        struct hci_acl_data* p_data_tx = HL_HCI_DATA_ALLOC(hci_acl_data);
        ASSERT_ERR(p_data_tx != NULL);
        if(p_data_tx == NULL) { break; } // This should create a deny of service -> High risk of disconnection

        // ************ Allocate and Fill a TX buffer
        buf_ptr = cb_tx_buf_alloc(conhdl, ll_buffer_size);
        ASSERT_ERR(buf_ptr != 0); // This should not happen
        p_data_tx->buf_ptr = buf_ptr;
        p_data_tx->length  = 0;
        nb_buffer--;

        // Fill connection handle + broadcast flags (packet boundary set later in prepare buffer callback)
        p_data_tx->conhdl_pb_bc_flag = conhdl;
        SETF(p_data_tx->conhdl_pb_bc_flag, HCI_ACL_HDR_BC_FLAG, BCF_P2P);

        // Prepare buffer for segment
        data_length = cb_prepare_buffer(p_chan, p_sdu, p_meta, p_data_tx, &buf_ptr, ll_buffer_size);

        // copy SDU data
        l2cap_chan_ll_buf_wr(buf_ptr, co_buf_data(p_sdu) + p_meta->data_cursor, data_length);
        p_data_tx->length       += data_length;
        // ************ Update information for next loop
        p_meta->seg_remain_size -= data_length;
        p_meta->data_cursor     += data_length;

        //Trace the tx packet
        TRC_REQ_L2CAP_TX(GETF(p_data_tx->conhdl_pb_bc_flag, HCI_ACL_HDR_HDL), p_data_tx->length, p_data_tx->buf_ptr);

        DBG_SWDIAG(HL, L2CAP_TX_HCI, 1);
        // Send ACL Data to controller
        hl_hci_send_data_to_ctrl(p_data_tx);
        DBG_SWDIAG(HL, L2CAP_TX_HCI, 0);

        // segment transmission done
        if (p_meta->seg_remain_size == 0)
        {
            is_segment_done = true;
            if(GETB(p_chan->config_bf, L2CAP_CHAN_PDU_TX_PAUSED))
            {
                l2cap_coc_tx_flow_off((l2cap_chan_coc_t*)p_chan, p_meta->tx_flow_off_token);
                is_more_data_to_send = false;
            }
            break;
        }

        // use buffer continuation
        cb_prepare_buffer = l2cap_chan_continue_prepare_buffer;
    }

    // Transmission done, inform upper layer and release buffer
    if(p_meta->data_cursor == co_buf_data_len(p_sdu))
    {
        co_list_pop_front(&(p_chan->tx_queue));
        l2cap_chan_tx_release_sdu(p_chan, p_sdu, p_meta, GAP_ERR_NO_ERROR);
    }

    // store if a segment trasmission still on-going
    SETB(p_con->state_bf,   L2CAP_TX_SEGMENT_ONGOING,      !is_segment_done);
    SETB(p_chan->config_bf, L2CAP_CHAN_TX_SEGMENT_ONGOING, !is_segment_done);

    // Here the TX flow is paused
    if(is_segment_done && !p_itf->cb_is_more_data_to_send(p_chan))
    {
        is_more_data_to_send = false;
    }

    DBG_SWDIAG(HL, L2CAP_TX, 0);

    return (is_more_data_to_send);
}

/**
 ****************************************************************************************
 * @brief Update number of ACL LL buffer released
 *
 * @param[in] conidx        Connection Index
 * @param[in] nb_buf        Number of buffer released
 ****************************************************************************************
 */
__STATIC void l2cap_chan_ll_buf_release(uint8_t conidx, uint16_t nb_buf)
{
    l2cap_con_env_t* p_con = l2cap_get_con_env(conidx);

    if(p_con != NULL)
    {
        l2cap_env.ll_buf_nb_avail += nb_buf;
        p_con->nb_tx_pending      -= nb_buf;

        // check if transmission are pending
        if(!co_list_is_empty(&(l2cap_env.le_tx_queue)))
        {
            // mark that transmission can be started
            co_djob_reg(CO_DJOB_LOW, &(l2cap_env.le_tx_bg_job));
        }
    }
}

/// Push channel to transmission queue
__INLINE void l2cap_chan_le_tx_queue_push(l2cap_chan_t* p_chan)
{
    if(!GETB(p_chan->config_bf, L2CAP_CHAN_IN_TX_QUEUE))
    {
        SETB(p_chan->config_bf, L2CAP_CHAN_IN_TX_QUEUE, true);
        // Put channel on transmission queue
        co_list_push_back(&(l2cap_env.le_tx_queue), &(p_chan->hdr));
        // mark that transmission can be started
        co_djob_reg(CO_DJOB_LOW, &(l2cap_env.le_tx_bg_job));
    }
}


#if(BT_HOST_PRESENT)
/// Push channel to transmission queue
__INLINE void l2cap_chan_tx_queue_push(l2cap_chan_t* p_chan)
{
    l2cap_con_env_t* p_con = l2cap_env.p_con[p_chan->conidx];
    if(GETB(p_con->state_bf, L2CAP_BT_CLASSIC_CONNECTION))
    {
        l2cap_chan_bt_tx_queue_push(p_chan);
    }
    else
    {
        l2cap_chan_le_tx_queue_push(p_chan);
    }
}
#else
#define l2cap_chan_tx_queue_push l2cap_chan_le_tx_queue_push
#endif //(BT_HOST_PRESENT)

/*
 * INTERNAL FUNCTIONS
 ****************************************************************************************
 */


void l2cap_chan_tx_le_handler(co_djob_t* p_djob)
{
    DBG_FUNC_ENTER(l2cap_chan_tx_le_handler);
    DBG_SWDIAG(HL, L2CAP_TX_CHK, 1);
    co_list_t* p_tx_queue = &(l2cap_env.le_tx_queue);

    // browse all connection to search transmission to perform
    while((l2cap_env.ll_buf_nb_avail > 0) && !co_list_is_empty(p_tx_queue))
    {
        l2cap_chan_t* p_chan = (l2cap_chan_t*)co_list_pop_front(p_tx_queue);
        uint8_t conidx = p_chan->conidx;
        l2cap_con_env_t* p_con = l2cap_env.p_con[conidx];
        ASSERT_ERR(!GETB(p_con->state_bf, L2CAP_DISCONNECTING));

        // A Channel for transmission must be chosen if no segment under transmission on connection (or same channel as on-going transmission)
        if(GETB(p_con->state_bf, L2CAP_TX_SEGMENT_ONGOING) && !GETB(p_chan->config_bf, L2CAP_CHAN_TX_SEGMENT_ONGOING))
        {
            // nothing can be done, ignore current channel
            co_list_push_back(p_tx_queue, &(p_chan->hdr));
        }
        // Perform Segment transmission
        else if(l2cap_chan_segment_send(conidx, p_con, p_chan, co_min(l2cap_env.ll_buf_nb_avail, L2CAP_ACL_BUF_PER_LOOP),
                                        l2cap_env.ll_buf_size, l2cap_chan_tx_le_buf_alloc))
        {
            // if some remaining buffer must be sent put it at end of waiting TX queue
            co_list_push_back(p_tx_queue, &(p_chan->hdr));
        }
        else
        {
            SETB(p_chan->config_bf, L2CAP_CHAN_IN_TX_QUEUE, false);
        }
    }

    DBG_SWDIAG(HL, L2CAP_TX_CHK, 0);
    DBG_FUNC_EXIT(l2cap_chan_tx_le_handler);
}

void l2cap_chan_tx_release_queue(l2cap_con_env_t* p_con, l2cap_chan_t* p_chan)
{
    // release transmission queue
    while(!co_list_is_empty(&(p_chan->tx_queue)))
    {
        co_buf_t* p_sdu = (co_buf_t*) co_list_pop_front(&(p_chan->tx_queue));
        l2cap_buf_tx_meta_t* p_meta = (l2cap_buf_tx_meta_t*) co_buf_metadata(p_sdu);

        l2cap_chan_tx_release_sdu(p_chan, p_sdu, p_meta, L2CAP_ERR_CONNECTION_LOST);
    }


    // remove channel from transmission queue
    if(GETB(p_chan->config_bf, L2CAP_CHAN_IN_TX_QUEUE))
    {
        // If a segment was not finished, mark it done
        if(GETB(p_chan->config_bf, L2CAP_CHAN_TX_SEGMENT_ONGOING))
        {
            SETB(p_chan->config_bf, L2CAP_CHAN_TX_SEGMENT_ONGOING, false);
            SETB(p_con->state_bf, L2CAP_TX_SEGMENT_ONGOING, false);
        }

        co_list_extract(&(l2cap_env.le_tx_queue), &(p_chan->hdr));
        SETB(p_chan->config_bf, L2CAP_CHAN_IN_TX_QUEUE, false);
    }
}


/*
 * HCI Handlers
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles number of completed packet events.
 * When the host sends an ACL packet to controller, if the packet is
 * acknowledged, the controller sends this event to the host to indicate
 * that peer link layer has received the packet
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void l2cap_hci_nb_cmp_pkts_evt_handler(uint8_t evt_code, struct hci_nb_cmp_pkts_evt const *p_evt)
{
    uint8_t i;
    DBG_FUNC_ENTER(l2cap_hci_nb_cmp_pkts_evt_handler);
    DBG_SWDIAG(HL, L2CAP_TX_DONE, 1);
    // this will update the number of buffers available per connection handle
    for (i = 0; i < p_evt->nb_of_hdl; i++)
    {
        uint8_t conidx = gapc_get_conidx(p_evt->con[i].hdl);

        if(conidx != GAP_INVALID_CONIDX)
        {
            l2cap_chan_ll_buf_release(conidx, p_evt->con[i].nb_comp_pkt);
        }
    }
    DBG_SWDIAG(HL, L2CAP_TX_DONE, 0);
    DBG_FUNC_EXIT(l2cap_hci_nb_cmp_pkts_evt_handler);
}

/*
 * EXTERNAL Functions
 ****************************************************************************************
 */
uint16_t l2cap_chan_sdu_send(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, co_buf_t* p_sdu)
{
    DBG_FUNC_ENTER(l2cap_chan_sdu_send);
    uint16_t status = GAP_ERR_NO_ERROR;

    do
    {
        const l2cap_chan_tx_sdu_cb_itf_t* p_itf;
        l2cap_chan_t* p_chan;
        l2cap_buf_tx_meta_t* p_sdu_meta;

        // check SDU
        if(p_sdu == NULL)
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // check if environment is enough big for transmission environment
        if(co_buf_metadata_len(p_sdu) < sizeof(l2cap_buf_tx_meta_t))
        {
            status = GAP_ERR_UNEXPECTED;
            break;
        }

        // Retrieve channel and callback interface
        p_itf  = l2cap_chan_tx_get_cb_itf(conidx, chan_lid, &p_chan);
        // verify channel presence and if transmission can be performed
        if(p_itf == NULL)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Check SDU size
        if(co_buf_data_len(p_sdu) > p_itf->cb_get_tx_mtu_size(p_chan))
        {
            status = L2CAP_ERR_INVALID_MTU;
            break;
        }

        // update metadata
        p_sdu_meta = (l2cap_buf_tx_meta_t*) co_buf_metadata(p_sdu);
        co_buf_acquire(p_sdu);

        // Initialize transmission environment
        p_sdu_meta->dummy           = dummy;
        p_sdu_meta->data_cursor     = 0;
        p_sdu_meta->seg_remain_size = 0;
        p_sdu_meta->p_itf           = p_itf;

        // Push new buffer in transmission queue
        co_list_push_back(&(p_chan->tx_queue), &(p_sdu->hdr));

        // check if TX flow is on
        if(p_itf->cb_is_ready_to_send(p_chan))
        {
            // Put channel on transmission queue
            l2cap_chan_tx_queue_push(p_chan);
        }
    } while(0);

    DBG_FUNC_EXIT(l2cap_chan_sdu_send);
    return (status);
}


#if (RW_DEBUG)
uint16_t l2cap_chan_debug_sdu_send(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint8_t dbg_bf, co_buf_t* p_sdu)
{
    uint16_t status;

    // check SDU
    if(p_sdu == NULL)
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    // check if environment is enough big for transmission environment
    if(co_buf_metadata_len(p_sdu) < sizeof(l2cap_buf_tx_debug_meta_t))
    {
        status = GAP_ERR_UNEXPECTED;
    }
    else
    {
        l2cap_buf_tx_debug_meta_t* p_meta = (l2cap_buf_tx_debug_meta_t*) co_buf_metadata(p_sdu);
        status = l2cap_chan_sdu_send(conidx, dummy, chan_lid, p_sdu);

        // overwrite metadata with new information
        p_meta->p_parent_itf = p_meta->hdr.p_itf;
        p_meta->hdr.p_itf    = &l2cap_chan_tx_debug_sdu_cb_itf;
        p_meta->dbg_bf       = dbg_bf;

    }

    return (status);
}
#endif // (RW_DEBUG)


void l2cap_chan_tx_flow_set(l2cap_chan_coc_t* p_chan, uint16_t token, uint8_t on)
{
    SETB(p_chan->config_bf, L2CAP_CHAN_PDU_TX_PAUSED, !on);

    // Enable flow
    if(on)
    {
         // check that buffer are available  and there is TX credit available
        if(!co_list_is_empty(&(p_chan->tx_queue)) && (p_chan->tx_credit != 0))
        {
            // Put channel on transmission queue
            l2cap_chan_tx_queue_push((l2cap_chan_t*) p_chan);
        }
    }
    else
    {
        // No transmission onto current channel - TX flow is already OFF
        if(!GETB(p_chan->config_bf, L2CAP_CHAN_TX_SEGMENT_ONGOING))
        {
            l2cap_coc_tx_flow_off(p_chan, token); // inform now that traffic has been stopped
        }
        else
        {
            // put token into transmission buffer to know when tx flow can be stopped
            co_buf_t* p_sdu = (co_buf_t*) co_list_pick(&(p_chan->tx_queue));
            ASSERT_ERR(p_sdu != NULL);
            l2cap_buf_tx_meta_t* p_sdu_meta = (l2cap_buf_tx_meta_t*) co_buf_metadata(p_sdu);
            p_sdu_meta->tx_flow_off_token = token;
        }
    }
}

#endif // (BLE_L2CAP)
/// @} L2CAP

