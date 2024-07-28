/**
 ****************************************************************************************
 * @file l2cap_chan_tx.c
 *
 * @brief  L2CAP channel management - Packet Reception
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

#if (EMB_PRESENT)
#include "ble_util_buf.h"   // LE ACL buffer usage
#include "reg_access.h"     // EM access API
#if (BT_HOST_PRESENT)
#include "bt_util_buf.h"    // BT ACL buffer usage
#endif // (BT_HOST_PRESENT)
#endif // (EMB_PRESENT)


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
#define l2cap_chan_ll_buf_rd16p(buf_in)                   em_rd16p((buf_in))
#define l2cap_chan_ll_buf_rd(buf_out, buf_in, size)       em_rd((buf_out), (buf_in), (size))
#else // (!EMB_PRESENT)
#define l2cap_chan_ll_buf_rd16p(buf_in)                   co_read16p(((uint8_t*) (buf_in)))
#define l2cap_chan_ll_buf_rd(buf_out, buf_in, size)       memcpy(((uint8_t*) (buf_out)), (uint8_t*) (buf_in), (size))
#endif // (EMB_PRESENT)


/*
 * DEFINES
 ****************************************************************************************
 */

/// L2CAP Connection state Bit Field
enum  l2cap_buf_release_bf
{
    /// Connection index
    L2CAP_BUF_RELEASE_CONIDX_MASK   = 0x000000FF,
    L2CAP_BUF_RELEASE_CONIDX_LSB    = 0,
    /// L2CAP Channel local identifier
    L2CAP_BUF_RELEASE_CHAN_LID_MASK = 0x0000FF00,
    L2CAP_BUF_RELEASE_CHAN_LID_LSB  = 8,
    /// Number of reception credit
    L2CAP_BUF_RELEASE_CREDIT_MASK   = 0xFFFF0000,
    L2CAP_BUF_RELEASE_CREDIT_LSB    = 16,
};

/// Rx State machine state
enum l2cap_chan_rx_state
{
    /// Ignore reception of data (default state)
    L2CAP_RX_IGNORE,
    /// Receive ACL Header [*SEGMEN_LEN : CHANNEL_ID*]
    L2CAP_RX_ACL_HEADER,
    /// An error occurs during FIX Channel SDU allocation
    L2CAP_RX_FAIL_FIX_SDU_ALLOC,
    /// Receive ACL + SDU Header [SEGMEN_LEN : CHANNEL_ID : *SDU_LEN*]
    L2CAP_RX_SDU_LEN_IN_HEADER,
    /// Handle Segment DATA reception
    L2CAP_RX_SEGMENT_DATA,
    L2CAP_RX_STATE_NB,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Buffer Reception meta-data
typedef struct l2cap_buf_rx_meta
{
    /// Number of reception credit used
    uint16_t credit;
    /// Reception data cursor
    uint16_t data_cursor;
    /// Size remain to be used into segment (0: new segment can be started)
    uint16_t seg_remain_size;
    /// reception status (if an error is detected)
    uint16_t status;
} l2cap_buf_rx_meta_t;


/// Cancel Fragment reception
typedef void (*l2cap_chan_cb_cancel_fragment_reception)(l2cap_chan_t* p_chan);
/// Handle ACL header and return next transition state
typedef uint8_t (*l2cap_chan_cb_handle_header_and_get_next_reception_state)(l2cap_chan_t* p_chan, uint16_t seg_length);
/// Handle full SDU reception and give it to upper layers
typedef void (*l2cap_chan_cb_sdu_rx)(l2cap_chan_t* p_chan, l2cap_buf_rx_meta_t*p_sdu_meta, co_buf_t* p_sdu);


/// Interface used to receive fragments - Object oriented
typedef struct l2cap_chan_rx_cb_itf
{
    /// Cancel Fragment reception
    l2cap_chan_cb_cancel_fragment_reception                  cb_cancel_fragment_reception;
    /// Handle ACL header and return next transition state
    l2cap_chan_cb_handle_header_and_get_next_reception_state cb_handle_header_and_get_next_reception_state;
    /// Handle full SDU reception and give it to upper layers
    l2cap_chan_cb_sdu_rx                                     cb_sdu_rx;
} l2cap_chan_rx_cb_itf_t;


/// Handle reception state and return if SDU reception is over
typedef bool (*l2cap_chan_rx_cb_state_handler)(l2cap_con_env_t* p_con, l2cap_chan_t* p_chan, const l2cap_chan_rx_cb_itf_t* p_itf,
                                               uint32_t* p_buf_ptr, uint16_t* p_length);

/*
 * FUNCTION DEFINITION
 ****************************************************************************************
 */


__STATIC bool l2cap_chan_rx_handle_in_error(l2cap_con_env_t* p_con, l2cap_chan_t* p_chan, const l2cap_chan_rx_cb_itf_t* p_itf,
                                            uint32_t* p_buf_ptr, uint16_t* p_length);
__STATIC bool l2cap_chan_rx_handle_acl_header(l2cap_con_env_t* p_con, l2cap_chan_t* p_chan, const l2cap_chan_rx_cb_itf_t* p_itf,
                                              uint32_t* p_buf_ptr, uint16_t* p_length);
__STATIC bool l2cap_chan_rx_handle_fail_fix_sdu_alloc(l2cap_con_env_t* p_con, l2cap_chan_t* p_chan, const l2cap_chan_rx_cb_itf_t* p_itf,
                                                 uint32_t* p_buf_ptr, uint16_t* p_length);
__STATIC bool l2cap_chan_rx_handle_sdu_len_in_header(l2cap_con_env_t* p_con, l2cap_chan_coc_t* p_chan, const l2cap_chan_rx_cb_itf_t* p_itf,
                                                     uint32_t* p_buf_ptr, uint16_t* p_length);
__STATIC bool l2cap_chan_rx_handle_segment_data(l2cap_con_env_t* p_con, l2cap_chan_t* p_chan, const l2cap_chan_rx_cb_itf_t* p_itf,
                                            uint32_t* p_buf_ptr, uint16_t* p_length);

__STATIC void l2cap_chan_rx_buf_released(co_buf_t* p_buf, uint32_t release_bf);
__STATIC void l2cap_chan_dynamic_rx_buf_released(co_buf_t* p_buf, uint32_t release_bf);

__STATIC bool l2cap_rx_buf_alloc(co_buf_t** pp_sdu, uint8_t conidx, uint16_t sdu_length, uint16_t seg_length, uint16_t status);


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */


/// List of RX handlers
__STATIC l2cap_chan_rx_cb_state_handler l2cap_chan_rx_handlers[L2CAP_RX_STATE_NB] =
{
    [L2CAP_RX_IGNORE]             = l2cap_chan_rx_handle_in_error,
    [L2CAP_RX_ACL_HEADER]         = l2cap_chan_rx_handle_acl_header,
    [L2CAP_RX_FAIL_FIX_SDU_ALLOC] = l2cap_chan_rx_handle_fail_fix_sdu_alloc,
    [L2CAP_RX_SDU_LEN_IN_HEADER]  = (l2cap_chan_rx_cb_state_handler) l2cap_chan_rx_handle_sdu_len_in_header,

    [L2CAP_RX_SEGMENT_DATA]       = l2cap_chan_rx_handle_segment_data,
};



/*
 * Default Channel RX handle
 ****************************************************************************************
 */

/// Cancel Fragment reception
__STATIC void l2cap_chan_rx_default_cb_cancel_fragment_reception(l2cap_chan_t* p_chan)
{
    // Do Nothing
}

/// Handle ACL header and return next transition state
__STATIC uint8_t l2cap_chan_rx_default_cb_handle_header_and_get_next_reception_state(l2cap_chan_t* p_chan, uint16_t seg_length)
{
    return (L2CAP_RX_IGNORE);
}

/// Handle full SDU reception and give it to upper layers
__STATIC void l2cap_chan_rx_default_cb_sdu_rx(l2cap_chan_t* p_chan, l2cap_buf_rx_meta_t*p_sdu_meta, co_buf_t* p_sdu)
{
    ASSERT_ERR(0); // Not expected at all
}

/// Default RX channel reception interface
__STATIC const l2cap_chan_rx_cb_itf_t l2cap_chan_rx_default_cb_itf =
{
    .cb_cancel_fragment_reception                  = l2cap_chan_rx_default_cb_cancel_fragment_reception,
    .cb_handle_header_and_get_next_reception_state = l2cap_chan_rx_default_cb_handle_header_and_get_next_reception_state,
    .cb_sdu_rx                                     = l2cap_chan_rx_default_cb_sdu_rx,
};



/*
 * Fixed Channel RX handle
 ****************************************************************************************
 */

/// Cancel Fragment reception
__STATIC void l2cap_chan_rx_fixed_cb_cancel_fragment_reception(l2cap_chan_t* p_chan)
{
    co_buf_t* p_sdu = p_chan->p_rx_sdu;
    ASSERT_ERR(p_sdu != NULL);
    l2cap_buf_rx_meta_t* p_sdu_meta = (l2cap_buf_rx_meta_t*) co_buf_metadata(p_sdu);
    // remove tail data not yet received
    co_buf_tail_release(p_sdu, p_sdu_meta->seg_remain_size);

    p_chan->p_rx_sdu = NULL;
    // Inform reception of SDU
    p_chan->p_cb->cb_sdu_rx(p_chan->conidx, p_chan->chan_lid,
                            (p_sdu_meta->status != GAP_ERR_NO_ERROR) ? p_sdu_meta->status
                                                                     : L2CAP_ERR_INVALID_PDU, p_sdu);
    // release SDU
    co_buf_release(p_sdu);
}

/// Handle ACL header and return next transition state
__STATIC uint8_t l2cap_chan_rx_fixed_cb_handle_header_and_get_next_reception_state(l2cap_chan_t* p_chan, uint16_t seg_length)
{
    uint8_t rx_state = L2CAP_RX_SEGMENT_DATA;
    uint16_t status = GAP_ERR_NO_ERROR;

    // check if MTU exceed
    if(seg_length > p_chan->mtu)
    {
        // mark that MTU error detected
        status     = L2CAP_ERR_INVALID_MTU;
        // segment length updated to a value less that LE minimum MTU
        seg_length = co_min(seg_length, L2CAP_LE_MTU_MIN);
    }

    // Allocate buffer
    if(!l2cap_rx_buf_alloc(&(p_chan->p_rx_sdu), p_chan->conidx, seg_length, seg_length, status))
    {
        // An error occurs, do it later
        rx_state = L2CAP_RX_FAIL_FIX_SDU_ALLOC;
    }

    return (rx_state);
}


/// Handle full SDU reception and give it to upper layers
__STATIC void l2cap_chan_rx_fixed_cb_sdu_rx(l2cap_chan_t* p_chan, l2cap_buf_rx_meta_t*p_sdu_meta, co_buf_t* p_sdu)
{
    // Inform reception of SDU
    p_chan->p_cb->cb_sdu_rx(p_chan->conidx, p_chan->chan_lid, p_sdu_meta->status, p_sdu);
}

/// Fixed channel RX channel reception interface
__STATIC const l2cap_chan_rx_cb_itf_t l2cap_chan_rx_fixed_cb_itf =
{
    .cb_cancel_fragment_reception = l2cap_chan_rx_fixed_cb_cancel_fragment_reception,
    .cb_handle_header_and_get_next_reception_state = l2cap_chan_rx_fixed_cb_handle_header_and_get_next_reception_state,
    .cb_sdu_rx                                     = l2cap_chan_rx_fixed_cb_sdu_rx,
};


/*
 * Dynamic Channel RX handle
 ****************************************************************************************
 */

/// Cancel Fragment reception
__STATIC void l2cap_chan_rx_dynamic_cb_cancel_fragment_reception(l2cap_chan_coc_t* p_chan)
{
    // inform l2cap_coc that channel must be closed due to Invalid PDU received
    l2cap_coc_error_detected(p_chan, L2CAP_ERR_INVALID_PDU);
}

/// Handle ACL header and return next transition state
__STATIC uint8_t l2cap_chan_rx_dynamic_cb_handle_header_and_get_next_reception_state(l2cap_chan_coc_t* p_chan, uint16_t seg_length)
{
    uint8_t next_rx_state = L2CAP_RX_IGNORE;

    // check if MPS exceeded
    if(seg_length > p_chan->rx_mps)
    {
        // inform l2cap_coc that channel must be closed due to packet size exceed
        l2cap_coc_error_detected(p_chan, L2CAP_ERR_INVALID_MPS);
    }
    else if(GETB(p_chan->config_bf, L2CAP_CHAN_CREDIT_FLOW_EN) && (p_chan->rx_credit == 0))
    {
        // inform l2cap_coc that channel must be closed due to credit error
        l2cap_coc_error_detected(p_chan, L2CAP_ERR_INSUFF_CREDIT);
    }
    // new SDU received, wait for SDU length header
    else if(p_chan->p_rx_sdu == NULL)
    {
        if(seg_length < L2CAP_SDU_LEN)
        {
            // Stop COC due to invalid PDU received
            l2cap_coc_error_detected(p_chan, L2CAP_ERR_INVALID_PDU);
        }
        else
        {
            l2cap_con_env_t* p_con  = l2cap_env.p_con[p_chan->conidx];
            p_con->rx_temp_exp_len += L2CAP_SDU_LEN;

            // Expect reception of SDU length
            next_rx_state = L2CAP_RX_SDU_LEN_IN_HEADER;
        }
    }
    else
    {
        l2cap_buf_rx_meta_t* p_sdu_meta = (l2cap_buf_rx_meta_t*) co_buf_metadata(p_chan->p_rx_sdu);

        // check if new segment length does not overflow SDU in reception
        if(seg_length > (co_buf_data_len(p_chan->p_rx_sdu) - p_sdu_meta->data_cursor))
        {
            l2cap_coc_error_detected(p_chan, L2CAP_ERR_INVALID_PDU);
        }
        else
        {
            // update Buffer meta-data
            p_sdu_meta->seg_remain_size  = seg_length;
            // update number of reception credits
            p_sdu_meta->credit          += 1;
            p_chan->rx_credit           -= 1;

            // if number of reception credit is more that expected number
            if(p_sdu_meta->credit > CO_DIVIDE_CEIL(p_sdu_meta->data_cursor, p_chan->rx_mps))
            {
                // add one new credit for reception
                l2cap_coc_rx_credit_add(p_chan, 1);
                p_chan->rx_credit  += 1;
                p_sdu_meta->credit -= 1;
            }

            // Expect reception of SDU Data
            next_rx_state = L2CAP_RX_SEGMENT_DATA;
        }
    }

    return (next_rx_state);
}

/// Handle full SDU reception and give it to upper layers
__STATIC void l2cap_chan_rx_dynamic_cb_sdu_rx(l2cap_chan_coc_t* p_chan, l2cap_buf_rx_meta_t*p_sdu_meta, co_buf_t* p_sdu)
{
    uint32_t release_bf = 0;
    SETF(release_bf, L2CAP_BUF_RELEASE_CONIDX,   p_chan->conidx);
    SETF(release_bf, L2CAP_BUF_RELEASE_CHAN_LID, p_chan->chan_lid);
    SETF(release_bf, L2CAP_BUF_RELEASE_CREDIT,   p_sdu_meta->credit);
    co_buf_cb_free_set(p_sdu, (co_buf_free_cb)l2cap_chan_dynamic_rx_buf_released, (uint32_t*) release_bf);

    // Inform reception of SDU
    p_chan->p_cb->cb_sdu_rx(p_chan->conidx, p_chan->chan_lid, p_sdu_meta->status, p_sdu);
}

/// Dynamic channel RX channel reception interface
__STATIC const l2cap_chan_rx_cb_itf_t l2cap_chan_rx_dynamic_cb_itf =
{
    .cb_cancel_fragment_reception                  = (l2cap_chan_cb_cancel_fragment_reception) l2cap_chan_rx_dynamic_cb_cancel_fragment_reception,
    .cb_handle_header_and_get_next_reception_state = (l2cap_chan_cb_handle_header_and_get_next_reception_state) l2cap_chan_rx_dynamic_cb_handle_header_and_get_next_reception_state,
    .cb_sdu_rx                                     = (l2cap_chan_cb_sdu_rx) l2cap_chan_rx_dynamic_cb_sdu_rx,
};

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/// Handle SDU RX Buffer allocation
__STATIC bool l2cap_rx_buf_alloc(co_buf_t** pp_sdu, uint8_t conidx, uint16_t sdu_length, uint16_t seg_length, uint16_t status)
{
    bool allocation_succeed = true;
    // Flow control mechanism here - no more host buffer available
    if((l2cap_env.hl_buf_nb_avail == 0) || (co_buf_alloc(pp_sdu, 0 , sdu_length, 0) != CO_BUF_ERR_NO_ERROR))
    {
        // Pause RX traffic
        l2cap_env.rx_flow_paused_bf |= CO_BIT(conidx);
        allocation_succeed = false;
    }
    else
    {
        l2cap_buf_rx_meta_t* p_sdu_meta;
        uint32_t release_bf = 0;
        l2cap_env.hl_buf_nb_avail--;

        // Set Buffer release information
        SETF(release_bf, L2CAP_BUF_RELEASE_CONIDX,   conidx);
        SETF(release_bf, L2CAP_BUF_RELEASE_CHAN_LID, L2CAP_INVALID_CHAN_LID);
        co_buf_cb_free_set(*pp_sdu, (co_buf_free_cb)l2cap_chan_rx_buf_released, (uint32_t*) release_bf);

        // update Buffer meta-data
        p_sdu_meta = (l2cap_buf_rx_meta_t*) co_buf_metadata(*pp_sdu);
        p_sdu_meta->data_cursor     = 0;
        p_sdu_meta->seg_remain_size = seg_length;
        p_sdu_meta->status          = status;
        p_sdu_meta->credit          = 1;
    }

    return (allocation_succeed);
}


/// Prepare ACL RX information
__STATIC l2cap_acl_rx_t* l2cap_acl_rx_info_prepare(uint16_t conhdl, struct hci_acl_data const *p_evt)
{
    l2cap_acl_rx_t*  p_acl_rx;
    uint16_t info_bf = (p_evt->length & L2CAP_ACL_RX_LENGTH_MASK);

    #if (BLE_EMB_PRESENT)
    #if (BT_HOST_PRESENT) // Store type of buffers
    bool is_bt_classic_buffer = ((conhdl & BT_ACL_CONHDL_BIT) != 0);
    SETB(info_bf, L2CAP_ACL_BT_CLASSIC_BUF, is_bt_classic_buffer);
    if(is_bt_classic_buffer)
    {
        ASSERT_ERR(sizeof(struct bt_em_acl_buf_elt) >= sizeof(l2cap_acl_rx_t));
        // transform received message to a L2CAP ACL RX information
        p_acl_rx = (l2cap_acl_rx_t*) bt_util_buf_elt_rx_get(p_evt->buf_ptr);
    }
    else
    #endif // (BT_HOST_PRESENT)
    {
        ASSERT_ERR(sizeof(struct ble_em_acl_buf_elt) >= sizeof(l2cap_acl_rx_t));
        // transform received message to a L2CAP ACL RX information
        p_acl_rx = (l2cap_acl_rx_t*) ble_util_buf_elt_rx_get(p_evt->buf_ptr);
    }

    p_acl_rx->buf_ptr = (uint16_t) p_evt->buf_ptr;
    #else // (!BLE_EMB_PRESENT)
    ASSERT_ERR(sizeof(l2cap_acl_rx_t) <= HCI_HOST_BUF_HEAD_LEN);
    {
        uint32_t buf_ptr = p_evt->buf_ptr;

        // reuse event pointer
        p_acl_rx = (l2cap_acl_rx_t*)p_evt;
        p_acl_rx->buf_ptr = buf_ptr;
    }
    #endif // (BLE_EMB_PRESENT)

    SETF(info_bf, L2CAP_ACL_RX_PB_FLAG, GETF(p_evt->conhdl_pb_bc_flag, HCI_ACL_HDR_PB_FLAG));

    // keep computed information bit field
    p_acl_rx->info_bf = info_bf;

    return (p_acl_rx);
}

/// Release reception buffer
__STATIC void l2cap_chan_buf_release(l2cap_acl_rx_t* p_acl_rx)
{
    // Free RX buffer.
    #if (EMB_PRESENT)
    #if (BT_HOST_PRESENT)
    if(GETB(p_acl_rx->info_bf, L2CAP_ACL_BT_CLASSIC_BUF))
    {
        bt_util_buf_acl_rx_free(p_acl_rx->buf_ptr);
    }
    else
    #endif // (BT_HOST_PRESENT)
    {
        ble_util_buf_rx_free(p_acl_rx->buf_ptr);
    }
    #else// (!EMB_PRESENT)
    ke_free(p_acl_rx);
    #endif // (EMB_PRESENT)
}


/// @brief Callback executed when a reception buffer is released for fixed channel
__STATIC void l2cap_chan_rx_buf_released(co_buf_t* p_buf, uint32_t release_bf)
{
    // flow control mechanism
    l2cap_env.hl_buf_nb_avail++;
    if(l2cap_env.rx_flow_paused_bf != 0)
    {
        co_djob_reg(CO_DJOB_LOW, &(l2cap_env.rx_bg_job));
    }
}


/// Callback executed when a reception buffer is released for a flow control enabled channel
__STATIC void l2cap_chan_dynamic_rx_buf_released(co_buf_t* p_buf, uint32_t release_bf)
{
    // Add some new credits
    l2cap_chan_rx_credit_add(GETF(release_bf, L2CAP_BUF_RELEASE_CONIDX), GETF(release_bf, L2CAP_BUF_RELEASE_CHAN_LID),
                             GETF(release_bf, L2CAP_BUF_RELEASE_CREDIT));

    // call default release function
    l2cap_chan_rx_buf_released(p_buf, release_bf);
}


/// Retrieve callback interface of the channel
__STATIC const l2cap_chan_rx_cb_itf_t* l2cap_chan_rx_get_cb_itf(l2cap_chan_t* p_chan)
{
    const l2cap_chan_rx_cb_itf_t* p_itf = &l2cap_chan_rx_default_cb_itf;

    if((p_chan != NULL) && GETB(p_chan->config_bf, L2CAP_CHAN_EN))
    {
        if(GETB(p_chan->config_bf, L2CAP_CHAN_FIX))
        {
            p_itf = &l2cap_chan_rx_fixed_cb_itf;
        }
        else
        {
            p_itf = &l2cap_chan_rx_dynamic_cb_itf;
        }
    }

    return (p_itf);
}


/// Get Reception channel environment
__STATIC l2cap_chan_t* l2cap_chan_rx_get_env(l2cap_con_env_t* p_con, uint8_t chan_lid, const l2cap_chan_rx_cb_itf_t** pp_itf)
{
    l2cap_chan_t* p_chan = (chan_lid != L2CAP_INVALID_CHAN_LID) ? p_con->p_chan[chan_lid] : NULL;
    *pp_itf = l2cap_chan_rx_get_cb_itf(p_chan);
    return (p_chan);
}

/// Handle reception of header (4 or 6 bytes)
/// Data can be received byte per bytes so put everything in a temporary buffer
__STATIC void l2cap_chan_rx_copy_header_data(l2cap_con_env_t* p_con, uint32_t* p_buf_ptr, uint16_t* p_length)
{
    uint16_t copy_len = co_min(*p_length , p_con->rx_temp_exp_len - p_con->rx_temp_cursor);

    // handle reception of header (4 or 6 bytes)
    // data can be received byte per bytes.
    ASSERT_ERR(p_con->rx_temp_cursor <= p_con->rx_temp_exp_len);

    // Copy data
    l2cap_chan_ll_buf_rd(&(p_con->rx_temp_buf[p_con->rx_temp_cursor]), *p_buf_ptr, copy_len);

    p_con->rx_temp_cursor += copy_len;
    *p_buf_ptr            += copy_len;
    *p_length             -= copy_len;
}


/// An error occurs, Ignore data
__STATIC bool l2cap_chan_rx_handle_in_error(l2cap_con_env_t* p_con, l2cap_chan_t* p_chan, const l2cap_chan_rx_cb_itf_t* p_itf,
                                            uint32_t* p_buf_ptr, uint16_t* p_length)
{
    // nothing to do
    return (true);
}

/// Handle ACL Header: [SEGMENT_LEN : CID]
__STATIC bool l2cap_chan_rx_handle_acl_header(l2cap_con_env_t* p_con, l2cap_chan_t* p_chan, const l2cap_chan_rx_cb_itf_t* p_itf,
                                              uint32_t* p_buf_ptr, uint16_t* p_length)
{
    bool is_finished = false;
    // p_chan can be != NULL if allocation of SDU fails

    // Receive header data
    l2cap_chan_rx_copy_header_data(p_con, p_buf_ptr, p_length);

    if(p_con->rx_temp_cursor == L2CAP_HEADER_LEN)
    {
        uint16_t seg_length = co_btohs(co_read16p(&(p_con->rx_temp_buf[0])));
        uint16_t rx_cid     = co_btohs(co_read16p(&(p_con->rx_temp_buf[L2CAP_LENGTH_LEN])));
        p_con->rx_chan_lid  = l2cap_chan_find(p_con->conidx, L2CAP_CHAN_CID_LOCAL, rx_cid, &p_chan);
        p_itf = l2cap_chan_rx_get_cb_itf(p_chan);

        // Get next reception state
        p_con->rx_state = p_itf->cb_handle_header_and_get_next_reception_state(p_chan,  seg_length);
        // execute next reception state
        is_finished = l2cap_chan_rx_handlers[p_con->rx_state](p_con, p_chan, p_itf, p_buf_ptr, p_length);
    }

    return (is_finished);
}

/// An error occurs during FIX Channel SDU allocation
__STATIC bool l2cap_chan_rx_handle_fail_fix_sdu_alloc(l2cap_con_env_t* p_con, l2cap_chan_t* p_chan, const l2cap_chan_rx_cb_itf_t* p_itf,
                                                 uint32_t* p_buf_ptr, uint16_t* p_length)
{
    p_con->rx_state = L2CAP_RX_ACL_HEADER;
    return (false);
}

/// Handle ACL Header: [SEGMENT_LEN : CID : *SDU_LEN*] --> Dynamic channel only
bool l2cap_chan_rx_handle_sdu_len_in_header(l2cap_con_env_t* p_con, l2cap_chan_coc_t* p_chan, const l2cap_chan_rx_cb_itf_t* p_itf,
                                            uint32_t* p_buf_ptr, uint16_t* p_length)
{
    bool is_finished = false;
    ASSERT_ERR(p_chan != NULL);

    // Receive header data
    l2cap_chan_rx_copy_header_data(p_con, p_buf_ptr, p_length);

    if(p_con->rx_temp_cursor == L2CAP_HEADER_LEN + L2CAP_SDU_LEN)
    {
        uint16_t seg_length = co_btohs(co_read16p(&(p_con->rx_temp_buf[0]))) - L2CAP_SDU_LEN;
        uint16_t sdu_length = co_btohs(co_read16p(&(p_con->rx_temp_buf[L2CAP_HEADER_LEN])));

        // check if MTU exceeded
        if(sdu_length > p_chan->rx_mtu)
        {
            // inform l2cap_coc that channel must be closed due to SDU size exceed
            l2cap_coc_error_detected(p_chan, L2CAP_ERR_INVALID_MTU);
            is_finished = true;
        }
        else if(seg_length > sdu_length)
        {
            l2cap_coc_error_detected(p_chan, L2CAP_ERR_INVALID_PDU);
            is_finished = true;
        }
        // Allocate data
        else if(l2cap_rx_buf_alloc(&(p_chan->p_rx_sdu), p_chan->conidx, sdu_length, seg_length, GAP_ERR_NO_ERROR))
        {
            // execute handling of SDU data
            p_con->rx_state = L2CAP_RX_SEGMENT_DATA;
            // update number of reception credits
            p_chan->rx_credit         -= 1;
            // continue packet extraction
            is_finished = l2cap_chan_rx_handlers[p_con->rx_state](p_con, (l2cap_chan_t*) p_chan, p_itf, p_buf_ptr, p_length);
        } // else Wait for new buffer to be available
    }

    return (is_finished);
}

/// Handle reception of segment data
__STATIC bool l2cap_chan_rx_handle_segment_data(l2cap_con_env_t* p_con, l2cap_chan_t* p_chan, const l2cap_chan_rx_cb_itf_t* p_itf,
                                            uint32_t* p_buf_ptr, uint16_t* p_length)
{
    ASSERT_ERR(p_chan != NULL);
    bool is_finished = false;
    co_buf_t* p_sdu = p_chan->p_rx_sdu;
    ASSERT_ERR(p_sdu != NULL);

    l2cap_buf_rx_meta_t* p_sdu_meta = (l2cap_buf_rx_meta_t*) co_buf_metadata(p_sdu);
    uint8_t* p_out    = (co_buf_data(p_sdu) + p_sdu_meta->data_cursor);
    uint16_t copy_len = co_min(*p_length , p_sdu_meta->seg_remain_size); // ignore padding data

    // Copy data
    l2cap_chan_ll_buf_rd(p_out, *p_buf_ptr, copy_len);

    // Update data cursor
    p_sdu_meta->data_cursor     += copy_len;
    p_sdu_meta->seg_remain_size -= copy_len;

    if(p_sdu_meta->seg_remain_size == 0)
    {
        // check if complete SDU has been received
        if(p_sdu_meta->data_cursor == co_buf_data_len(p_sdu))
        {
            p_chan->p_rx_sdu = NULL;
            p_itf->cb_sdu_rx(p_chan, p_sdu_meta, p_sdu);
            co_buf_release(p_sdu);
        }
        is_finished = true;
    }

    return (is_finished);
}

/// Cancel on-going fragment reception
__STATIC void l2cap_chan_cancel_fragment_reception(l2cap_con_env_t* p_con)
{
    const l2cap_chan_rx_cb_itf_t* p_itf;
    uint8_t chan_lid = p_con->rx_chan_lid;
    l2cap_chan_t* p_chan = l2cap_chan_rx_get_env(p_con, chan_lid, &p_itf);

    // Cancel fragment reception
    p_itf->cb_cancel_fragment_reception(p_chan);
    // initialize new segment reception
    l2cap_chan_rx_init(p_con);
}

/**
 ****************************************************************************************
 * @brief handle reception of ACL packets for the connection in the reception queue
 *
 * @param[in] conidx        Connection Index
 ****************************************************************************************
 */
__STATIC void l2cap_chan_ll_rx_acl(uint8_t conidx)
{
    l2cap_con_env_t* p_con = l2cap_get_con_env(conidx);;
    ASSERT_ERR(p_con != NULL);

    // Handle packet if flow is enabled
    while(   ((l2cap_env.rx_flow_paused_bf & CO_BIT(conidx)) == 0)
          && !co_list_is_empty(&(p_con->rx_queue)) && GETB(p_con->state_bf, L2CAP_RX_FLOW_ENABLE))
    {
        // controller packet received
        l2cap_acl_rx_t* p_acl_rx = (l2cap_acl_rx_t*) co_list_pick(&(p_con->rx_queue));
        uint16_t length  = GETF(p_acl_rx->info_bf, L2CAP_ACL_RX_LENGTH);
        uint32_t buf_ptr = p_acl_rx->buf_ptr;

        // extract packet information
        switch (GETF(p_acl_rx->info_bf, L2CAP_ACL_RX_PB_FLAG))
        {
            case PBF_1ST_HL_FRAG:  // Start Fragment
            {
                // Cancel fragment reception
                l2cap_chan_cancel_fragment_reception(p_con);

                SETF(p_acl_rx->info_bf, L2CAP_ACL_RX_PB_FLAG, PBF_CONT_HL_FRAG);
            }
            // no break

            case PBF_CONT_HL_FRAG: // Continue Fragment
            {
                // Channel information
                const l2cap_chan_rx_cb_itf_t* p_itf;
                uint8_t chan_lid = p_con->rx_chan_lid;
                l2cap_chan_t* p_chan = l2cap_chan_rx_get_env(p_con, chan_lid, &p_itf);
                bool is_finished;
                // Check that buffer pointer is not NULL
                if(p_acl_rx->buf_ptr == 0) { break; }

                // handle packet reception
                is_finished = l2cap_chan_rx_handlers[p_con->rx_state](p_con, p_chan, p_itf, &buf_ptr, &length);
                if(is_finished)
                {
                    // initialize new segment reception
                    l2cap_chan_rx_init(p_con);
                }
            } break;

            case PBF_1ST_NF_HL_FRAG: // Non flushable packet not supported
            default: { /* Do nothing */ } break;
        }

        if((l2cap_env.rx_flow_paused_bf & CO_BIT(conidx)) == 0)
        {
            co_list_pop_front(&(p_con->rx_queue));
            l2cap_chan_buf_release(p_acl_rx);
        }
        else
        {
            // Pause traffic since no buffer can be allocated
            p_acl_rx->buf_ptr = buf_ptr;
            SETF(p_acl_rx->info_bf, L2CAP_ACL_RX_LENGTH, length);
            break;
        }
    }
}


/*
 * INTERNAL FUNCTIONS
 ****************************************************************************************
 */

void l2cap_chan_rx_init(l2cap_con_env_t* p_con)
{
    p_con->rx_temp_cursor  = 0;
    p_con->rx_temp_exp_len = L2CAP_HEADER_LEN;
    p_con->rx_chan_lid     = L2CAP_INVALID_CHAN_LID;
    p_con->rx_state        = L2CAP_RX_ACL_HEADER;
}

void l2cap_chan_rx_handler(co_djob_t* p_djob)
{
    // browse all connection to search reception to perform
    while((l2cap_env.hl_buf_nb_avail > 0) && (l2cap_env.rx_flow_paused_bf != 0))
    {
        // Retrieve connection index
        uint8_t conidx = co_ctz(l2cap_env.rx_flow_paused_bf);
        l2cap_env.rx_flow_paused_bf &= ~CO_BIT(conidx);

        l2cap_chan_ll_rx_acl(conidx);
    }
}

uint16_t l2cap_rx_ctrl(uint8_t conidx, uint8_t enable)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    l2cap_con_env_t* p_con = l2cap_get_con_env(conidx);

    if(p_con == NULL)
    {
         status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else if(GETB(p_con->state_bf, L2CAP_RX_FLOW_ENABLE) != enable)
    {
        SETB(p_con->state_bf, L2CAP_RX_FLOW_ENABLE, enable);

        // RX is now enabled, continue reception
        if(enable)
        {
            l2cap_chan_ll_rx_acl(conidx);
        }
    }

    return (status);
}

void l2cap_rx_queue_cleanup(l2cap_con_env_t* p_con)
{
    // Clean-up messages from receive queue
    while(!co_list_is_empty(&(p_con->rx_queue)))
    {
        // Retrieve message to process
        l2cap_acl_rx_t* p_acl_rx = (l2cap_acl_rx_t*)  co_list_pop_front(&(p_con->rx_queue));
        // Free RX buffer.
        l2cap_chan_buf_release(p_acl_rx);
    }
}

/*
 * HCI Handlers
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles data coming from Link Layer.
 * This handler receives the data from lower layer, and then send the data to respective
 * host block.
 *
 * @param[in] p_evt     Pointer to data parameters.
 *
 ****************************************************************************************
 */
void l2cap_hci_acl_data_handler(struct hci_acl_data const *p_evt)
{
    DBG_FUNC_ENTER(l2cap_hci_acl_data_handler);
    DBG_SWDIAG(HL, L2CAP_RX, 1);
    // Trace the RX packet received
    l2cap_acl_rx_t* p_acl_rx;
    uint16_t conhdl = GETF(p_evt->conhdl_pb_bc_flag, HCI_ACL_HDR_HDL);
    uint8_t conidx  = gapc_get_conidx(conhdl);

    TRC_REQ_L2CAP_RX(conhdl, p_evt->length, p_evt->buf_ptr);

    // Prepare ACL buffer structure
    p_acl_rx = l2cap_acl_rx_info_prepare(conhdl, p_evt);

    // check if connection exists
    if(conidx != GAP_INVALID_CONIDX)
    {
        l2cap_con_env_t* p_con = l2cap_env.p_con[conidx];

        // put message in receive queue
        co_list_push_back(&(p_con->rx_queue), &(p_acl_rx->hdr));
        // handle message reception
        l2cap_chan_ll_rx_acl(conidx);
    }
    else
    {
        l2cap_chan_buf_release(p_acl_rx);
    }

    DBG_SWDIAG(HL, L2CAP_RX, 0);
    DBG_FUNC_EXIT(l2cap_hci_acl_data_handler);
}

#endif // (BLE_L2CAP)
/// @} L2CAP

