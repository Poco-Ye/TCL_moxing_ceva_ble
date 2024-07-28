/**
 ****************************************************************************************
 *
 * @file l2cap_chan_bt.c
 *
 * @brief 2CAP channel management - BT Classic specific functions
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


/// @addtogroup L2CAP
/// @{

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // IP configuration
#if (BT_HOST_PRESENT && BLE_L2CAP)
#include "l2cap_int.h"      // Internals

#include "gap.h"            // GAP Defines

#include "co_math.h"        // Mathematical utilities
#include "co_endian.h"      // Endianess management
#include "hl_hci.h"         // HL HCI Interface

#if (EMB_PRESENT)
#include "reg_access.h"     // EM access API
#include "bt_util_buf.h"    // Buffer allocation
#else
#include "ke_mem.h"         // Memory allocation of HCI data
#endif // (EMB_PRESENT)

#include "dbg_swdiag.h"     // SW Diags


#include "bk_al_wrapper.h"     // BT Host Stack Wrapper API
#include "btstack_bluetooth.h" // BT-classic Defines
#include "btstack_hci.h"       // BT-classic HCI API


/*
 * MACROS
 ****************************************************************************************
 */
// Buffer Memory Access
#if (EMB_PRESENT)
#define l2cap_chan_ll_buf_rd16p(buf_in)                       em_rd16p((buf_in))
#else // (!EMB_PRESENT)
#define l2cap_chan_ll_buf_rd16p(buf_in)                       co_read16p(((uint8_t*) (buf_in)))
#endif // (EMB_PRESENT)

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/// Allocate a buffer for ACL HCI Transmission over BR/EDR transport
__STATIC uint32_t l2cap_chan_bt_tx_buf_alloc(uint16_t conhdl, uint16_t ll_buf_size)
{
    uint32_t buf_ptr;
    #if BLE_EMB_PRESENT
    buf_ptr = bt_util_buf_acl_tx_alloc();
    #else // !(BLE_EMB_PRESENT)
    uint8_t* p_acl_buf;
    // allocate buffer in message heap
    p_acl_buf  = ke_malloc_system(ll_buf_size + HCI_ACL_HDR_LEN + HCI_TRANSPORT_HDR_LEN, KE_MEM_KE_MSG);
    // move pointer to payload beginning
    p_acl_buf += HCI_ACL_HDR_LEN + HCI_TRANSPORT_HDR_LEN;
    buf_ptr = (uint32_t) p_acl_buf;
    #endif // BLE_EMB_PRESENT

    bk_al_acquire_acl_ll_buffer(conhdl);
    return (buf_ptr);
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/// Check if ACL data flow should be Hijacked --> Called only for a BR/EDR connection
bool l2cap_chan_rx_check_if_flow_hijacked(uint8_t conidx, const struct hci_acl_data *p_acl_data)
{
    bool is_flow_hijacked = false;
    l2cap_con_env_t* p_con = l2cap_get_con_env(conidx);

    // check if connection exists
    if(p_con != NULL)
    {
        switch(GETF(p_acl_data->conhdl_pb_bc_flag, HCI_ACL_HDR_PB_FLAG))
        {
            case PBF_1ST_HL_FRAG:
            {
                // Find if CID handled by RW Host L2CAP
                uint16_t rx_cid = co_btohs(l2cap_chan_ll_buf_rd16p(p_acl_data->buf_ptr + L2CAP_LENGTH_LEN));
                if(l2cap_chan_find(conidx, L2CAP_CHAN_CID_LOCAL, rx_cid, NULL) != L2CAP_INVALID_CHAN_LID)
                {
                    // following ACL continuous fragments will be automatically hijacked
                    is_flow_hijacked = true;
                } // else Stop ACL flow hijack

                SETB(p_con->state_bf, L2CAP_RX_ACL_FLOW_HIJACK, is_flow_hijacked);
            } break;

            // check if continuation packet handled by RW Host
            case PBF_CONT_HL_FRAG: { is_flow_hijacked = GETB(p_con->state_bf, L2CAP_RX_ACL_FLOW_HIJACK); } break;

            default: { ASSERT_INFO(0, conidx, GETF(p_acl_data->conhdl_pb_bc_flag, HCI_ACL_HDR_PB_FLAG)); } break;
        }
    }

    return (is_flow_hijacked);
}


uint16_t l2cap_chan_tx_bt_get_number_of_ll_avail_buf()
{
    return hci_number_free_acl_slots_for_connection_type(BD_ADDR_TYPE_ACL);
}

void l2cap_chan_bt_tx_handler(co_djob_t* p_djob) // check impact of merging function with l2cap_chan_tx_le_handler()
{
    DBG_FUNC_ENTER(l2cap_chan_tx_bt_handler);
    DBG_SWDIAG(HL, L2CAP_TX_CHK, 1);
    co_list_t* p_tx_queue = &(l2cap_env.bt_tx_queue);
    uint16_t ll_buf_nb_avail = l2cap_chan_tx_bt_get_number_of_ll_avail_buf();

    // browse all connection to search transmission to perform
    while((ll_buf_nb_avail > 0) && !co_list_is_empty(p_tx_queue))
    {
        l2cap_chan_t* p_chan = (l2cap_chan_t*)co_list_pop_front(p_tx_queue);
        uint8_t conidx = p_chan->conidx;
        l2cap_con_env_t* p_con = l2cap_get_con_env(conidx);

        // A Channel for transmission must be chosen if no segment under transmission on connection (or same channel as on-going transmission)
        if(GETB(p_con->state_bf, L2CAP_TX_SEGMENT_ONGOING) && !GETB(p_chan->config_bf, L2CAP_CHAN_TX_SEGMENT_ONGOING))
        {
            // nothing can be done, ignore current channel
            co_list_push_back(p_tx_queue, &(p_chan->hdr));
            continue;
        }

        // Perform Segment transmission
        if(l2cap_chan_segment_send(conidx, p_con, p_chan, co_min(ll_buf_nb_avail, L2CAP_ACL_BUF_PER_LOOP),
                                   hci_max_acl_data_packet_length(), l2cap_chan_bt_tx_buf_alloc))
        {
            // if some remaining buffer must be sent put it at end of waiting TX queue
            co_list_push_back(p_tx_queue, &(p_chan->hdr));
        }
        else
        {
            SETB(p_chan->config_bf, L2CAP_CHAN_IN_TX_QUEUE, false);
        }

        ll_buf_nb_avail = l2cap_chan_tx_bt_get_number_of_ll_avail_buf();
    }

    DBG_SWDIAG(HL, L2CAP_TX_CHK, 0);
    DBG_FUNC_EXIT(l2cap_chan_tx_bt_handler);
}


/// Check if some data can  be transmitted
void l2cap_chan_bt_tx_check_if_data_to_send(void)
{
    // If BR/EDR data to send
    if(!co_list_is_empty(&(l2cap_env.bt_tx_queue)))
    {
        // mark that transmission can be started
        co_djob_reg(CO_DJOB_LOW, &(l2cap_env.bt_tx_bg_job));
    }
}

/// Push channel to transmission queue
void l2cap_chan_bt_tx_queue_push(l2cap_chan_t* p_chan)
{
    if(!GETB(p_chan->config_bf, L2CAP_CHAN_IN_TX_QUEUE))
    {
        SETB(p_chan->config_bf, L2CAP_CHAN_IN_TX_QUEUE, true);
        // Put channel on transmission queue
        co_list_push_back(&(l2cap_env.bt_tx_queue), &(p_chan->hdr));
        // mark that transmission can be started
        co_djob_reg(CO_DJOB_LOW, &(l2cap_env.bt_tx_bg_job));
    }
}

#endif // (BT_HOST_PRESENT && BLE_L2CAP)

/// @} L2CAP
