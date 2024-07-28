/**
 ****************************************************************************************
 *
 * @file l2cap_int.h
 *
 * @brief Header file - L2CAP Internals
 *
 * Copyright (C) RivieraWaves 2009-2019
 ****************************************************************************************
 */

#ifndef L2CAP_INT_H_
#define L2CAP_INT_H_

/**
 ****************************************************************************************
 * @addtogroup L2CAP
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (BLE_L2CAP)
#include "../inc/gap_hl_api.h"
#include "l2cap.h"
#include "co_list.h"
#include "co_djob.h"

/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// Size of L2CAP Length field
#define L2CAP_LENGTH_LEN       (2)
/// Size of L2CAP CID field
#define L2CAP_CID_LEN          (2)

/// Size of L2CAP header
#define L2CAP_HEADER_LEN       (L2CAP_LENGTH_LEN + L2CAP_CID_LEN)
/// Size of SDU Length Field in first segment
#define L2CAP_SDU_LEN          (2)
/// only provide 5 buffers per transmission loops
#define L2CAP_ACL_BUF_PER_LOOP (5)


/// L2CAP Connection state Bit Field
enum  l2cap_state_bf
{
    /// Reception flow enabled
    L2CAP_RX_FLOW_ENABLE_BIT            = 0x01,
    L2CAP_RX_FLOW_ENABLE_POS            = 0,

    /// Segment transmission ongoing onto a channel
    L2CAP_TX_SEGMENT_ONGOING_BIT        = 0x02,
    L2CAP_TX_SEGMENT_ONGOING_POS        = 1,

    /// Peer device supports Enhanced L2CAP COC Creation
    L2CAP_COC_ENHANCED_SUPPORTED_BIT    = 0x04,
    L2CAP_COC_ENHANCED_SUPPORTED_POS    = 2,

    #if (BT_HOST_PRESENT)
    /// BT Classic connection
    L2CAP_BT_CLASSIC_CONNECTION_BIT     = 0X08,
    L2CAP_BT_CLASSIC_CONNECTION_POS     = 3,

    /// ACL Data Flow Hijacked
    L2CAP_RX_ACL_FLOW_HIJACK_BIT        = 0x10,
    L2CAP_RX_ACL_FLOW_HIJACK_POS        = 4,
    #endif // (BT_HOST_PRESENT)

    /// Disconnection on-going - reject incomming request
    L2CAP_DISCONNECTING_BIT             = 0x20,
    L2CAP_DISCONNECTING_POS             = 5,
};

/// LE Credit Based fields.
enum l2cc_chan_cid_type
{
    /// Local channel ID (used for reception)
    L2CAP_CHAN_CID_LOCAL,
    /// Peer channel ID (used for transmission)
    L2CAP_CHAN_CID_PEER,
};

/// L2CAP channel configuration Bit Field
enum  l2cap_chan_cfg_bf
{
    /// Channel configuration information --> Information that can be re-configured
    L2CAP_CHAN_CONFIG_INFO_MASK       = 0x07,
    L2CAP_CHAN_CONFIG_INFO_LSB        = 0,

    /// Channel enable; else reject any PDU transmission or reception -- PART of L2CAP_CHAN_CONFIG_INFO
    L2CAP_CHAN_EN_BIT                 = (1 << 0),
    L2CAP_CHAN_EN_POS                 = 0,

    /// Pause PDU transmission; else resume                           -- PART of L2CAP_CHAN_CONFIG_INFO
    L2CAP_CHAN_PDU_TX_PAUSED_BIT      = (1 << 1),
    L2CAP_CHAN_PDU_TX_PAUSED_POS      = 1,

    /// Enable credit flow control (Note: setting ignored for a fixed channel)
    L2CAP_CHAN_CREDIT_FLOW_EN_BIT     = (1 << 2),
    L2CAP_CHAN_CREDIT_FLOW_EN_POS     = 2,

    /// Fixed L2CAP channel; else dynamic  (@note cannot be reconfigured)
    L2CAP_CHAN_FIX_BIT                = (1 << 3),
    L2CAP_CHAN_FIX_POS                = 3,

    /// Used to know if buffer is within transmission queue
    L2CAP_CHAN_IN_TX_QUEUE_BIT        = (1 << 4),
    L2CAP_CHAN_IN_TX_QUEUE_POS        = 4,

    L2CAP_CHAN_TX_SEGMENT_ONGOING_BIT = (1 << 5),
    L2CAP_CHAN_TX_SEGMENT_ONGOING_POS = 5,

    #if (HOST_MSG_API)
    /// Debug Mode enabled onto a channel (valid only for Message API and Dynamic channel)
    L2CAP_CHAN_DBG_MODE_EN_BIT        = (1 << 6),
    L2CAP_CHAN_DBG_MODE_EN_POS        = 6,
    #endif  // (HOST_MSG_API)
};

/// RX Buffer information
enum l2cap_acl_rx_info_bf
{
    /// Length of received payload
    L2CAP_ACL_RX_LENGTH_LSB       = 0,
    L2CAP_ACL_RX_LENGTH_MASK      = 0x1FFF,
    #if(BT_HOST_PRESENT)
    /// True if BT Classic Buffer, False otherwise
    L2CAP_ACL_BT_CLASSIC_BUF_POS  = 13,
    L2CAP_ACL_BT_CLASSIC_BUF_BIT  = 0x2000,
    #endif // (BT_HOST_PRESENT)
    /// PB-FLAG value
    L2CAP_ACL_RX_PB_FLAG_LSB      = 14,
    L2CAP_ACL_RX_PB_FLAG_MASK     = 0xC000,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Channel environment (for a fixed channel)
typedef struct l2cap_chan
{
    /// Header used to insert channel in wait for tx queue
    co_list_hdr_t               hdr;
    /// Callback of function handled by an upper layer API for registered channel
    const l2cap_chan_cb_t*      p_cb;
    /// Buffer transmission queue
    co_list_t                   tx_queue;
    /// Reception buffer
    co_buf_t*                   p_rx_sdu;
    #if (HOST_MSG_API)
    /// Destination task number if channel registered through message API
    uint16_t                    dest_task_nbr;
    #endif // (HOST_MSG_API)
    /// Connection index of the channel
    uint8_t                     conidx;
    /// Channel Local identifier
    uint8_t                     chan_lid;
    /// Configuration bit field (see enum #l2cap_chan_cfg_bf)
    uint8_t                     config_bf;
    /// Reception L2CAP Channel Identifier
    uint16_t                    rx_cid;
    /// Transmission L2CAP Channel Identifier
    uint16_t                    tx_cid;
    //------------------------------------------ L2CAP FIX Channel specific parameters
    ///Reception / Transmission Maximum Transmit Unit Size
    uint16_t                    mtu;
} l2cap_chan_t;

/// Channel environment (for a dynamically allocated channel)
typedef struct l2cap_chan_coc
{
    //------------------------------------------ Parameters must have same offset as l2cap_chan_t parameters
    /// Header used to insert channel in wait for tx queue
    co_list_hdr_t               hdr;
    /// Callback of function handled by an upper layer API for registered channel
    const l2cap_chan_coc_cb_t*  p_cb;
    /// Buffer transmission queue
    co_list_t                   tx_queue;
    /// Reception buffer
    co_buf_t*                   p_rx_sdu;
    #if (HOST_MSG_API)
    /// Destination task number if channel registered through message API
    uint16_t                    dest_task_nbr;
    #endif // (HOST_MSG_API)
    /// Connection index of the channel
    uint8_t                     conidx;
    /// Channel Local identifier
    uint8_t                     chan_lid;
    /// Configuration bit field (see enum #l2cap_chan_cfg_bf)
    uint8_t                     config_bf;
    /// Reception L2CAP Channel Identifier
    uint16_t                    rx_cid;
    /// Transmission L2CAP Channel Identifier
    uint16_t                    tx_cid;
    //------------------------------------------ L2CAP Dynamic Channel specific parameters
    ///Reception Maximum Transmit Unit Size
    uint16_t                    rx_mtu;
    /// Transmission Maximum Transmit Unit Size
    uint16_t                    tx_mtu;

    /// Reception Maximum Packet Size.
    uint16_t                    rx_mps;
    /// Transmission Maximum Packet Size.
    uint16_t                    tx_mps;
    /// Reception credit number.
    uint16_t                    rx_credit;
    /// Maximum number of reception credits
    uint16_t                    rx_credit_max;
    /// Transmission reception credit number.
    uint16_t                    tx_credit;
} l2cap_chan_coc_t;

/// Connection Oriented channel specific environment
typedef struct l2cap_sig_env
{
    /// Request procedure queue (initiated by local device)
    co_list_t         req_proc_queue;
    /// Response procedure Queue (initiated by peer device).
    co_list_t         rsp_proc_queue;
    /// Transaction timer
    gapc_sdt_t        trans_timer;
    /// Channel Local Identifier for Signaling L2CAP
    uint8_t           chan_lid;
    /// Packet identifier counter
    uint8_t           pkt_id_counter;
    /// Signaling protocol state (see enum #l2cap_sig_state_bf)
    uint8_t           state_bf;
} l2cap_sig_env_t;

/// Connection environment
typedef struct l2cap_con_env
{
    /// Connection Oriented channel specific environment
    l2cap_sig_env_t   sig;
    #if (HOST_MSG_API)
    /// List of SDU received but waiting for Upper layer usage confirmation
    /// this is handled by buffer release in native API
    co_list_t         msg_api_sdu_queue;
    #endif // (HOST_MSG_API)
    /// List of received ACL packet not yet processed (when connection RX flow is off or paused due to lake of buffers)
    co_list_t         rx_queue;
    /// Index of current connection
    uint8_t           conidx;
    /// Channel index under reception
    uint8_t           rx_chan_lid;
    /// Channel reception state (use to know which callback to use see enum #l2cap_chan_rx_state)
    uint8_t           rx_state;
    /// Temporary Expected RX size length
    uint8_t           rx_temp_exp_len;
    /// Temporary RX data cursor
    uint8_t           rx_temp_cursor;
    /// Temporary buffer used to receive L2Cap Header + SDU Len
    uint8_t           rx_temp_buf[L2CAP_HEADER_LEN + L2CAP_SDU_LEN];
    /// Connection bit field used to know status of the connection. (see enum #l2cap_state_bf)
    uint8_t           state_bf;
    /// Number of ACL LL buffer waiting to be transmitted.
    uint8_t           nb_tx_pending;
    ///  Number of L2Cap channel (max 32)
    uint8_t           nb_channel;
    /// channel Array of channel environment pointer
    l2cap_chan_t*     p_chan[BLE_L2CAP_CHAN_PER_CON];
} l2cap_con_env_t;

/// SPSM Information
typedef struct l2cap_spsm
{
    /// List Header
    co_list_hdr_t               hdr;
    /// Callback that handle events onto a registered SPSM
    const l2cap_coc_spsm_cb_t* p_cb;
    #if (HOST_MSG_API)
    /// Destination task number if SPSM register through message API
    uint16_t                    dest_task_nbr;
    #endif // (HOST_MSG_API)
    /// Simplified Protocol/Service Multiplexer value
    uint16_t                    spsm;
    /// Security level bit field (see enum #l2cap_sec_lvl_bf)
    uint8_t                     sec_lvl_bf;
} l2cap_spsm_t;

/// L2CAP Environment structure
typedef struct l2cap_env_
{
    /// L2CAP PDU Reception background job
    co_djob_t         rx_bg_job;
    /// List of Registered SPSM
    co_list_t         reg_spsm;
    /// List of LE L2CAP channel waiting for transmission
    co_list_t         le_tx_queue;
    /// LE L2CAP PDU Transmission background job
    co_djob_t         le_tx_bg_job;
    #if (BT_HOST_PRESENT)
    /// List of BT-Classic L2CAP channel waiting for transmission
    co_list_t         bt_tx_queue;
    /// BT-Classic L2CAP PDU Transmission background job
    co_djob_t         bt_tx_bg_job;
    #endif // (BT_HOST_PRESENT)

    /// Array of connection environment pointer
    l2cap_con_env_t*  p_con[HOST_CONNECTION_MAX];
    /// Bit field of connection which are on reception flow paused
    uint32_t          rx_flow_paused_bf;

    #if (HOST_MSG_API)
    /// Destination task number of event - only valid for Message API usage
    uint16_t          dest_task_nbr;
    #endif // (HOST_MSG_API)

    /// Lower Layer buffers maximum size
    uint16_t          ll_buf_size;
    /// Total number of available Lower Layer buffers
    uint16_t          ll_buf_nb_total;
    /// Number of available Lower Layer buffers
    uint16_t          ll_buf_nb_avail;
    /// Number of buffer available for host flow control - Flow control mechanism
    uint8_t           hl_buf_nb_avail;
} l2cap_env_t;

/// Structure to manage received ACL data
typedef struct l2cap_acl_rx
{
    /// List header to put it in a queue
    co_list_hdr_t hdr;
    #if (BLE_EMB_PRESENT)
    /// Exchange memory Buffer pointer
    uint16_t      buf_ptr;
    #else // (BLE_EMB_PRESENT)
    /// Data memory pointer
    uint32_t      buf_ptr;
    #endif // (BLE_EMB_PRESENT)
    /// Buffer information bit field: PB FLAG + length of payload (see enum #l2cap_acl_rx_info_bf)
    uint16_t      info_bf;
} l2cap_acl_rx_t;

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// L2CAP Environment
extern l2cap_env_t l2cap_env;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Internal function used to retrieve connection environment
 *
 * @param[in]  conidx       Connection Index
 *
 * @return Pointer to connection environment
 ****************************************************************************************
 */
l2cap_con_env_t* l2cap_get_con_env(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Internal function used to reserve a dynamic L2CAP channel.
 *        This channel supports credit management, segmentation and reassembly mechanisms.
 *        To create a L2CAP Credit Based connection, @see l2cap_coc_create command should
 *        be used instead.
 *
 * @param[in]  conidx       Connection Index
 * @param[out] p_chan_lid   Pointer to L2CAP Channel local index
 * @param[out] pp_chan      Pointer to the allocated channel
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t l2cap_chan_dyn_reserve(uint8_t conidx, uint8_t* p_chan_lid, l2cap_chan_coc_t** pp_chan);

/**
 ****************************************************************************************
 * @brief Internal function used to increment number of dynamic L2CAP channel transmission credits.
 *
 * @param[in]  conidx       Connection Index
 * @param[in]  chan_lid     L2CAP Channel local index
 * @param[in]  credit       Number of credit to add for SDU transmission
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t l2cap_chan_tx_credit_add(uint8_t conidx, uint8_t chan_lid, uint16_t credit);


/**
 ****************************************************************************************
 * @brief Retrieve Channel information
 *
 * @param[in] conidx        Connection Index
 * @param[in] chan_lid      L2CAP Channel local index
 *
 * @return Found channel information structure; NULL if not found
 ****************************************************************************************
 */
l2cap_chan_t* l2cap_chan_get_env(uint8_t conidx, uint8_t chan_lid);


/**
 ****************************************************************************************
 * @brief Retrieve Credit Oriented channel
 *
 * @param[in] conidx        Connection Index
 * @param[in] chan_lid      L2CAP Channel local index
 *
 * @return Found channel information structure; NULL if not found
 ****************************************************************************************
 */
l2cap_chan_coc_t* l2cap_chan_coc_get(uint8_t conidx, uint8_t chan_lid);

/**
 ****************************************************************************************
 * @brief Find a L2CAP channel using TX or RX Channel Identifier
 *
 * @param[in]  conidx       Connection Index
 * @param[in]  cid_type     Channel identifier typer (see enum #l2cc_chan_cid_type)
 * @param[in]  cid          L2CAP Channel identifier
 * @param[out] pp_chan      Pointer where channel environment will be set
 *
 * @return Corresponding local channel identifier
 ****************************************************************************************
 */
uint8_t l2cap_chan_find(uint8_t conidx, uint8_t cid_type, uint16_t cid, l2cap_chan_t** pp_chan);


/**
 ****************************************************************************************
 * @brief Enable or disable usage of the channel
 *        Disabling the channel force a buffer clean-up of transmission and reception
 *
 * @param[in] p_chan        Pointer to channel environment structure
 * @param[in] enable        True to enable the channel, False to disable it
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t l2cap_chan_enable_set(l2cap_chan_t* p_chan, uint8_t enable);

/**
 ****************************************************************************************
 * @brief Control TX Flow: Accept transmission of new L2CAP Segments
 *
 * @param[in] p_chan        Pointer to channel environment structure
 * @param[in] token         Procedure token expecting to be informed when TX Flow is off.
 * @param[in] on            True: resume Segment transmission, False: Pause Segment transmission
 ****************************************************************************************
 */
void l2cap_chan_tx_flow_set(l2cap_chan_coc_t* p_chan, uint16_t token, uint8_t on);

/**
 ****************************************************************************************
 * @brief Increment number of reception credit.
 *        It ensure that number of credit added does not exceeds maximum number of credits
 *        allocated for connection
 *
 * @param[in]  conidx       Connection Index
 * @param[in]  chan_lid     L2CAP Channel local index
 * @param[in]  credit       Number of credit to add in reception
 ****************************************************************************************
 */
void l2cap_chan_rx_credit_add(uint8_t conidx, uint8_t chan_lid, uint16_t credit);

/**
 ****************************************************************************************
 * @brief Background function used to continue transmission of LE ACL data
 *
 * @param[in] p_djob  Pointer to Delayed job structure.
 ****************************************************************************************
 */
void l2cap_chan_tx_le_handler(co_djob_t* p_djob);

#if (BT_HOST_PRESENT)
/**
 ****************************************************************************************
 * @brief Background function used to continue transmission of BT-Classic ACL data
 *
 * @param[in] p_djob  Pointer to Delayed job structure.
 ****************************************************************************************
 */
void l2cap_chan_bt_tx_handler(co_djob_t* p_djob);

/// Push channel to transmission queue
void l2cap_chan_bt_tx_queue_push(l2cap_chan_t* p_chan);
#endif // (BT_HOST_PRESENT)

/**
 ****************************************************************************************
 * @brief Send a L2CAP segment, function used to start segment transmission or continue
 *        a paused segment transmission.
 *
 *        This function does not check if there is enough credit to start transmission
 *
 * @param[in] conidx          Connection index
 * @param[in] p_con           Pointer to connection environment
 * @param[in] p_chan          Pointer to channel that contains buffer to transmit
 * @param[in] nb_buffer       Number of buffer that can be sent
 * @param[in] ll_buffer_size  Size of controller buffer
 * @param[in] cb_tx_buf_alloc Function to execute in order to allocate a controller buffer
 *
 * @return True more data to send on channel, False otherwise
 ****************************************************************************************
 */
bool l2cap_chan_segment_send(uint8_t conidx, l2cap_con_env_t* p_con, l2cap_chan_t* p_chan, uint8_t nb_buffer,
                             uint16_t ll_buffer_size, uint32_t (*cb_tx_buf_alloc)(uint16_t, uint16_t));


/// Release TX queue
void l2cap_chan_tx_release_queue(l2cap_con_env_t* p_con, l2cap_chan_t* p_chan);

/**
 ****************************************************************************************
 * @brief Background function used to continue reception of ACL data
 *
 * @param[in] p_djob  Pointer to Delayed job structure.
 ****************************************************************************************
 */
void l2cap_chan_rx_handler(co_djob_t* p_djob);

/// Initialize reception information for connection
void l2cap_chan_rx_init(l2cap_con_env_t* p_con);

/**
 ****************************************************************************************
 * @brief Connection cleanup fonction of active channels
 *
 * @param[in] p_con Pointer to connection environment
 ****************************************************************************************
 */
void l2cap_chan_cleanup(l2cap_con_env_t* p_con);

/**
 ****************************************************************************************
 * @brief Handle BLE link creation
 *
 * @param[in] conidx        Connection Index
 ****************************************************************************************
 */
void l2cap_sig_create(l2cap_con_env_t* p_con);

/**
 ****************************************************************************************
 * @brief Handle BLE link disconnection
 *
 * @param[in] conidx        Connection Index
 ****************************************************************************************
 */
void l2cap_sig_cleanup(l2cap_con_env_t* p_con);

/**
 ****************************************************************************************
 * @brief Retrieve Registered SPSM information
 *
 * @param[in] spsm     Simplified Protocol/Service Multiplexer
 *
 * @return Found SPSM information structure; NULL if not found
 ****************************************************************************************
 */
l2cap_spsm_t* l2cap_coc_spsm_get(uint16_t spsm);

/**
 ****************************************************************************************
 * @brief Increment number of channel for the L2CAP COC
 *
 * @param[in] p_chan        Pointer to channel environment
 * @param[in] credits       Number of credit to increment
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t l2cap_coc_rx_credit_add(l2cap_chan_coc_t* p_chan, uint16_t credits);


/**
 ****************************************************************************************
 * @brief A L2CAP COC error has been detected, Disconnect the Link
 *
 * @param[in] p_chan        Pointer to channel environment
 * @param[in] status        Disconnection reason
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
void l2cap_coc_error_detected(l2cap_chan_coc_t* p_chan, uint16_t status);

/**
 ****************************************************************************************
 * @brief Inform that TX flow for the channel has been disabled
 *
 * @param[in] p_chan        Pointer to the COC channel
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
void l2cap_coc_tx_flow_off(l2cap_chan_coc_t* p_chan, uint16_t token);

#if (RW_DEBUG)
/**
 ****************************************************************************************
 * @brief Debug Command use to reconfigure a L2CAP connection oriented channel RX MTU and MPS
 *
 * @param[in] conidx        Connection Index
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] local_rx_mtu  New Local reception Maximum Transmit Unit Size
 * @param[in] local_rx_mps  New Local reception Maximum Packet Size
 * @param[in] nb_chan       Number of L2CAP Channel local index in provided array
 * @param[in] p_chan_lid    Pointer to an array of L2CAP Channel local index
 *
 *
 * @return Status of the function execution (see enum #hl_err)
 *         Consider status only if an error occurs; else wait for execution completion
 ****************************************************************************************
 */
uint16_t l2cap_coc_dbg_reconfigure(uint8_t conidx, uint16_t dummy, uint16_t local_rx_mtu, uint16_t local_rx_mps,
                                   uint8_t nb_chan, uint8_t* p_chan_lid);

#endif // (RW_DEBUG)
#if (HOST_MSG_API && RW_DEBUG)
/**
 ****************************************************************************************
 * @brief Inform Upper layer that number of credit for the L2CAP COC has been incremented
 *
 * @param[in] p_chan        Pointer to channel environment
 * @param[in] credits       Number of credit to increment
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
void l2cap_msg_coc_rx_credit_add(l2cap_chan_coc_t* p_chan, uint16_t credits);

/**
 ****************************************************************************************
 * @brief Inform Upper layer that L2CAP COC error has been detected
 *
 * @param[in] p_chan        Pointer to channel environment
 * @param[in] status        Disconnection reason
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
void l2cap_msg_coc_error_detected(l2cap_chan_coc_t* p_chan, uint16_t status);

/**
 ****************************************************************************************
 * @brief Inform Upper layer that L2CAP Channel TX Flow has been paused
 *
 * @param[in] p_chan        Pointer to channel environment
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
void l2cap_msg_coc_tx_flow_off(l2cap_chan_coc_t* p_chan);
#endif  // (HOST_MSG_API && RW_DEBUG)

/// Clean-up RX Queue
void l2cap_rx_queue_cleanup(l2cap_con_env_t* p_con);

#endif // (BLE_L2CAP)

/// @} L2CAP

#endif // L2CAP_INT_H_
