/**
 ****************************************************************************************
 * @file gatt_bearer.c
 *
 * @brief  GATT Bearer Manager
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
#include "rwip_config.h"         // IP configuration
#if (BLE_GATT)
#include "gatt.h"                // Native API
#include "gatt_int.h"            // Internals
#include "l2cap.h"               // L2CAP Native API
#include "gapm.h"                // to generate a token id
#include "gapc.h"                // To retrieve connection role

#include "ke_mem.h"              // Memory allocation
#include "co_math.h"             // Mathematics
#include <string.h>              // for memcmp and memcpy
#include "../inc/l2cap_hl_api.h" // For un-registration of L2CAP channel used for Legacy ATT bearer
#include "../inc/gatt_hl_api.h"  // To inform that a service change has been detected
#include "../inc/gap_hl_api.h"   // To inform that no more bearer are available

/*
 * MACROS
 ****************************************************************************************
 */

/// INFO definition of L2CAP packet
#define INFO(name, pack) \
    [name##_OPCODE]   = { pack, name##_LEN }

/// Mask of available bearers
#define GATT_BEARER_MASK    (CO_BIT(BLE_GATT_BEARER_PER_CON) - 1)

/*
 * DEFINES
 ****************************************************************************************
 */

/// Bearer Information bit field
enum gatt_bearer_info_bf
{
    /// Channel Local Identifier
    GATT_BEARER_CHAN_LID_MASK       = 0x003F,
    GATT_BEARER_CHAN_LID_LSB        = 0,
    /// Used to know if bearer is in reception function
    GATT_BEARER_IN_RX_BIT           = 0x0040,
    GATT_BEARER_IN_RX_POS           = 6,
    /// Reserved for future use
    GATT_BEARER_RFU_BIT             = 0x0080,
    GATT_BEARER_RFU_POS             = 7,
    /// Bearer is an Enhanced Attribute Bearer.
    GATT_BEARER_EATT_BIT            = 0x0100,
    GATT_BEARER_EATT_POS            = 8,
    /// Handling of new PDU is paused due to a procedure on-going --> pushed on reception queue
    GATT_BEARER_RX_BUSY_BIT         = 0x0200,
    GATT_BEARER_RX_BUSY_POS         = 9,
    /// A Server Indication procedure is assigned to the bearer.
    GATT_BEARER_SRV_IND_PROC_BIT    = 0x0400,
    GATT_BEARER_SRV_IND_PROC_POS    = 10,
    /// A Server Notification procedure is assigned to the bearer.
    GATT_BEARER_SRV_NTF_PROC_BIT    = 0x0800,
    GATT_BEARER_SRV_NTF_PROC_POS    = 11,
    /// A Client Request procedure is assigned to the bearer.
    GATT_BEARER_CLI_REQ_PROC_BIT    = 0x1000,
    GATT_BEARER_CLI_REQ_PROC_POS    = 12,
    /// A Client Command procedure is assigned to the bearer.
    GATT_BEARER_CLI_CMD_PROC_BIT    = 0x2000,
    GATT_BEARER_CLI_CMD_PROC_POS    = 13,
    /// A Server transaction is on-going on the bearer (ATT Request waiting for response)
    GATT_BEARER_SRV_TRANS_BIT       = 0x4000,
    GATT_BEARER_SRV_TRANS_POS       = 14,
    /// A Client transaction is on-going on the bearer (ATT Indication waiting for confirmation)
    GATT_BEARER_CLI_TRANS_BIT       = 0x8000,
    GATT_BEARER_CLI_TRANS_POS       = 15,
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// L2CAP PDU handler information for packing/unpacking and function handler
typedef struct gatt_bearer_pdu_info
{
    /// Pack/Unpack format string
    const char*                     pack_format;
    /// Length of L2CAP PDU
    uint16_t                        length;
} gatt_bearer_pdu_info_t;


/// Reception Buffer meta-data
typedef struct gatt_bearer_buf_rx_meta
{
    /// Background activity that handle packet reception
    gapc_sdt_t          defer;
    /// Reception status
    uint16_t            rx_status;
    /// Extracted PDU information
    union l2cap_att_pdu pdu;
} gatt_bearer_buf_rx_meta_t;


/*
 * FUNCTIONS DEFINITION
 ****************************************************************************************
 */
__STATIC void gatt_bearer_sdu_sent_cb(uint8_t conidx, uint16_t token, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu);
__STATIC void gatt_bearer_sdu_rx_cb(uint8_t conidx, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu);
__STATIC void gatt_bearer_coc_create_cmp_cb(uint8_t conidx, uint16_t dummy, uint16_t status, uint8_t nb_chan);
__STATIC void gatt_bearer_coc_created_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t local_rx_mtu, uint16_t peer_rx_mtu);
__STATIC void gatt_bearer_coc_reconfigure_cmp_cb(uint8_t conidx, uint16_t dummy, uint16_t status);
__STATIC void gatt_bearer_coc_mtu_changed_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t local_rx_mtu, uint16_t peer_rx_mtu);
__STATIC void gatt_bearer_coc_terminated_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t reason);
__STATIC void gatt_bearer_coc_terminate_cmp_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t status);
__STATIC uint16_t gatt_bearer_err_rsp_send(uint8_t conidx, uint8_t bearer_lid, uint8_t op_code, uint16_t reason);
__STATIC void gatt_bearer_coc_connect_req_cb(uint8_t conidx, uint16_t token, uint8_t nb_chan, uint16_t spsm, uint16_t peer_rx_mtu);

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Attribute packet format
__STATIC const gatt_bearer_pdu_info_t gatt_bearer_pdu[L2CAP_ATT_OPCODE_MAX] =
{
    // Reserved codes
    INFO(L2CAP_ATT_RESERVED0,                NULL),
    INFO(L2CAP_ATT_RESERVED1,                NULL),
    INFO(L2CAP_ATT_RESERVED2,                NULL),
    INFO(L2CAP_ATT_RESERVED3,                NULL),
    INFO(L2CAP_ATT_RESERVED4,                NULL),
    INFO(L2CAP_ATT_RESERVED5,                NULL),
    INFO(L2CAP_ATT_RESERVED6,                NULL),
    // Error response
    // [op_code | Handle | Err_Code]
    INFO(L2CAP_ATT_ERR_RSP,                  "BHB"),
    // Exchange MTU Request
    // [MTU]
    INFO(L2CAP_ATT_MTU_REQ,                  "H"),
    // Exchange MTU Response
    // [MTU]
    INFO(L2CAP_ATT_MTU_RSP,                  "H"),
    // Find Information Request
    // [sHdl | eHdl]
    INFO(L2CAP_ATT_FIND_INFO_REQ,            "HH"),
    // Find Information Response
    // [Format | Uuid]
    INFO(L2CAP_ATT_FIND_INFO_RSP,            "B"),
    // Find By Type Value Request
    // [sHdl | eHdl | AttType | AttVal]
    INFO(L2CAP_ATT_FIND_BY_TYPE_REQ,         "HH"),
    // Find By Type Value Response
    // [InfoList]
    INFO(L2CAP_ATT_FIND_BY_TYPE_RSP,         ""),
    // Read By Type Request
    // [sHdl | eHdl | UUID]
    INFO(L2CAP_ATT_RD_BY_TYPE_REQ,           "HH"),
    // Read By Type Response
    // [each_len | data]
    INFO(L2CAP_ATT_RD_BY_TYPE_RSP,           "B"),
    // Read Request
    // [Handle]
    INFO(L2CAP_ATT_RD_REQ,                   "H"),
    // Read Response
    // [value]
    INFO(L2CAP_ATT_RD_RSP,                   ""),
    // Read Blob Request
    // [Handle | Offset]
    INFO(L2CAP_ATT_RD_BLOB_REQ,              "HH"),
    // Read Blob Response
    // [value]
    INFO(L2CAP_ATT_RD_BLOB_RSP,              ""),
    // Read Multiple Request
    // [Handles]
    INFO(L2CAP_ATT_RD_MULT_REQ,              ""),
    // Read Multiple Response
    // [value]
    INFO(L2CAP_ATT_RD_MULT_RSP,              ""),
    // Read by Group Type Request
    // [sHdl | eHdl | UUID]
    INFO(L2CAP_ATT_RD_BY_GRP_TYPE_REQ,       "HH"),
    // Read By Group Type Response
    // [number | data]
    INFO(L2CAP_ATT_RD_BY_GRP_TYPE_RSP,       "B"),
    // Write Request
    // [Handle | Value]
    INFO(L2CAP_ATT_WR_REQ,                   "H"),
    // Write Response
    // []
    INFO(L2CAP_ATT_WR_RSP,                   ""),
    // Prepare Write Request
    // [Handle | Offset | Value]
    INFO(L2CAP_ATT_PREP_WR_REQ,              "HH"),
    // Prepare Write Response
    // [Handle | Offset | Value]
    INFO(L2CAP_ATT_PREP_WR_RSP,              "HH"),
    // Execute Write Request
    // [Flags]
    INFO(L2CAP_ATT_EXE_WR_REQ,               "B"),
    // Execute Write Response
    // []
    INFO(L2CAP_ATT_EXE_WR_RSP,               ""),
    // Handle Value Notification
    // [Handle | Value]
    INFO(L2CAP_ATT_HDL_VAL_NTF,              "H"),
    // Handle Value Indication
    // [Handle | Value]
    INFO(L2CAP_ATT_HDL_VAL_IND,              "H"),
    // Handle Value Confirmation
    // []
    INFO(L2CAP_ATT_HDL_VAL_CFM,              ""),
    // Handle Multiple Value Notification
    // [hdl | len | value] * N times
    INFO(L2CAP_ATT_MULT_HDL_VAL_NTF,         ""),
    // Handle Multiple read of variable length values request
    // [hdl] * N times
    INFO(L2CAP_ATT_RD_MULT_VAR_REQ,          ""),
    // Handle Multiple read of variable length values response
    // [len | value] * N times
    INFO(L2CAP_ATT_RD_MULT_VAR_RSP,          ""),
};

/// Attribute channel callback definition
__STATIC const l2cap_chan_cb_t gatt_bearer_fix_chan_cb =
{
        .cb_sdu_rx   = gatt_bearer_sdu_rx_cb,
        .cb_sdu_sent = gatt_bearer_sdu_sent_cb,
};

__STATIC const l2cap_chan_coc_cb_t gatt_bearer_eatt_chan_cb =
{
        .cb_sdu_rx              = gatt_bearer_sdu_rx_cb,
        .cb_sdu_sent            = gatt_bearer_sdu_sent_cb,
        .cb_coc_create_cmp      = gatt_bearer_coc_create_cmp_cb,
        .cb_coc_created         = gatt_bearer_coc_created_cb,
        .cb_coc_reconfigure_cmp = gatt_bearer_coc_reconfigure_cmp_cb,
        .cb_coc_mtu_changed     = gatt_bearer_coc_mtu_changed_cb,
        .cb_coc_terminated      = gatt_bearer_coc_terminated_cb,
        .cb_coc_terminate_cmp   = gatt_bearer_coc_terminate_cmp_cb,
};

__STATIC const l2cap_coc_spsm_cb_t gatt_bearer_coc_cb =
{
        .cb_coc_connect_req = gatt_bearer_coc_connect_req_cb,
};


/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve Bearer local identifier from L2CAP channel local identifier
 *
 * @param[in] conidx    Connection index
 * @param[in] chan_lid  L2CAP channel local identifier
 *
 * @return  Bearer local identifier
 ****************************************************************************************
 */
__STATIC uint8_t gatt_bearer_lid_get(uint8_t conidx, uint8_t chan_lid)
{
    uint8_t bearer_lid = GATT_INVALID_BEARER_LID;

    if((conidx < HOST_CONNECTION_MAX) && (gatt_env.p_con[conidx] != NULL))
    {
        gatt_con_env_t* p_con = gatt_env.p_con[conidx];
        uint16_t bearer_bf = p_con->bearer_bf;

        // search channel in all existing bearers
        while(bearer_bf != 0)
        {
            uint8_t cursor = co_ctz(bearer_bf);
            gatt_bearer_t* p_bearer = p_con->p_bearer[cursor];
            CO_BIT_SET(&(bearer_bf), cursor, false);

            ASSERT_ERR(p_bearer != NULL);

            if(GETF(p_bearer->info_bf, GATT_BEARER_CHAN_LID) == chan_lid)
            {
                bearer_lid = cursor;
                break;
            }
        }
    }

    return (bearer_lid);
}

/**
 ****************************************************************************************
 * @brief Retrieve L2CAP Channel local identifier from Bearer local identifier
 *
 * @param[in] conidx        Connection index
 * @param[in] bearer_lid    Bearer local identifier
 *
 * @return  L2CAP channel local identifier
 ****************************************************************************************
 */
__STATIC uint8_t gatt_bearer_chan_lid_get(uint8_t conidx, uint8_t bearer_lid)
{
    uint8_t chan_lid = L2CAP_INVALID_CHAN_LID;

    if((conidx < HOST_CONNECTION_MAX) && (gatt_env.p_con[conidx] != NULL) && (bearer_lid < BLE_GATT_BEARER_PER_CON))
    {
        gatt_bearer_t* p_bearer = gatt_env.p_con[conidx]->p_bearer[bearer_lid];

        if(p_bearer != NULL)
        {
            chan_lid = GETF(p_bearer->info_bf, GATT_BEARER_CHAN_LID);
        }
    }

    return (chan_lid);
}

/**
 ****************************************************************************************
 * @brief Function called when SDU has been transmitted or if an error occurs
 *
 * @param[in] conidx    Connection Index
 * @param[in] token     Token of the PDU sent, 0 if unused
 * @param[in] chan_lid  L2CAP channel local index
 * @param[in] status    Status of the operation (see enum #hl_err)
 * @param[in] p_sdu     Pointer to SDU transmitted
 ****************************************************************************************
 */
__STATIC void gatt_bearer_sdu_sent_cb(uint8_t conidx, uint16_t token, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu)
{
    gatt_proc_t* p_proc = gatt_proc_pick(conidx, token);

    if(p_proc != NULL)
    {
        if(p_sdu != NULL)
        {
            uint8_t method = GETF(co_buf_data(p_sdu)[0], L2CAP_ATT_OPCODE_METHOD);

            // Remove Header from PDU - used for prepare write procedure
            if((method < L2CAP_ATT_OPCODE_MAX) && (co_buf_data_len(p_sdu) > gatt_bearer_pdu[method].length))
            {
                co_buf_head_release(p_sdu, L2CAP_ATT_HEADER_LEN + gatt_bearer_pdu[method].length);
            }
        }

        // inform that the SDU has been properly sent
        gatt_proc_continue(conidx, p_proc, GATT_PROC_PDU_PUSHED_TO_LL, status);
    }
}

/**
 ****************************************************************************************
 * @brief Function called when Connection Oriented Channel creation operation is finished
 *
 * @param[in] conidx        Connection Index
 * @param[in] nb_chan_rsvd  Number of channel reserved
 * @param[in] status        Status of the operation (see enum #hl_err)
 * @param[in] nb_chan       Number of L2CAP channel created.
 ****************************************************************************************
 */
__STATIC void gatt_bearer_coc_create_cmp_cb(uint8_t conidx, uint16_t nb_chan_rsvd, uint16_t status, uint8_t nb_chan)
{
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];
    if(p_con != NULL)
    {
        p_con->nb_bearer_rsvd -= (nb_chan_rsvd - nb_chan);
    }

    if((status == GAP_ERR_INSUFF_RESOURCES) && (!GETB(p_con->state_bf, GATT_CON_EATT_ESTAB_RETRY)))
    {
        SETB(p_con->state_bf, GATT_CON_EATT_ESTAB_RETRY, true);

        if(gapc_get_role(conidx) == ROLE_MASTER)
        {
            // Immediately restart EATT establishment
            gatt_bearer_eatt_estab(conidx);
        }
        // start a timer to retry EATT establishment after (2 - (connSlaveLatency + 1) - connInterval)
        else
        {
            gapc_sdt_prepare(&(p_con->eatt_mitigation_timer) , conidx, GAPC_SDT_GATT_BEARER);
            gapc_sdt_timer_set(&(p_con->eatt_mitigation_timer), p_con->eatt_mitigation_time);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Function called when a new Connection Oriented Channel is created
 *
 * @param[in] conidx       Connection Index
 * @param[in] dummy        Dummy parameter provided by upper layer for command execution
 * @param[in] chan_lid     Connected L2CAP channel local index
 * @param[in] local_rx_mtu Local device Maximum Transmit Unit reception size
 * @param[in] peer_rx_mtu  Peer device Maximum Transmit Unit reception size
 ****************************************************************************************
 */
__STATIC void gatt_bearer_coc_created_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t local_rx_mtu,
                                         uint16_t peer_rx_mtu)
{
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];

    ASSERT_ERR(p_con->nb_bearer_rsvd > 0);
    if(p_con->nb_bearer_rsvd > 0)
    {
        gatt_bearer_t* p_bearer;
        // get first available bearer
        uint8_t bearer_lid = co_ctz(~p_con->bearer_bf & GATT_BEARER_MASK);
        ASSERT_ERR((~p_con->bearer_bf & GATT_BEARER_MASK) != 0);
        p_con->nb_bearer_rsvd--;

        // Allocate Default GATT Bearer.
        p_bearer = (gatt_bearer_t*) ke_malloc_user(sizeof(gatt_bearer_t), KE_MEM_ENV);

        if(p_bearer != NULL)
        {
            p_con->p_bearer[bearer_lid] = p_bearer;
            memset(p_bearer, 0, sizeof(gatt_bearer_t));

            p_bearer->mtu = co_min(local_rx_mtu, peer_rx_mtu);
            SETF(p_bearer->info_bf, GATT_BEARER_CHAN_LID, chan_lid);
            SETB(p_bearer->info_bf, GATT_BEARER_EATT,     true);
            CO_BIT_SET(&(p_con->bearer_bf), bearer_lid,   true);

            // Search if a procedure can be granted on the bearer
            gatt_proc_check_grant(conidx, bearer_lid, GATT_PROC_ALL);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Function called when Reconfigure L2CAP channel MTU is terminated
 *
 * @param[in] conidx    Connection Index
 * @param[in] dummy     Dummy parameter provided by upper layer for command execution
 * @param[in] status    Status of the operation (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC void gatt_bearer_coc_reconfigure_cmp_cb(uint8_t conidx, uint16_t dummy, uint16_t status)
{
    // Nothing to do
}

/**
 ****************************************************************************************
 * @brief Function called when Local or Peer RX MTU size has been changed onto the L2CAP channel
 *
 * @param[in] conidx       Connection Index
 * @param[in] dummy        Dummy parameter provided by upper layer for command execution
 * @param[in] chan_lid     L2CAP channel local index
 * @param[in] local_rx_mtu Local device Maximum Transmit Unit reception size
 * @param[in] peer_rx_mtu  Peer device Maximum Transmit Unit reception size
 ****************************************************************************************
 */
__STATIC void gatt_bearer_coc_mtu_changed_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t local_rx_mtu,
                                             uint16_t peer_rx_mtu)
{
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];
    // retrieve bearer
    uint8_t bearer_lid = gatt_bearer_lid_get(conidx, chan_lid);

    if(bearer_lid != GATT_INVALID_BEARER_LID)
    {
        gatt_bearer_t* p_bearer = p_con->p_bearer[bearer_lid];
        p_bearer->mtu = co_min(local_rx_mtu, peer_rx_mtu);
    }
}

/**
 ****************************************************************************************
 * @brief Function called when a Connection Oriented Channel is terminated
 *
 * @param[in] conidx    Connection Index
 * @param[in] dummy     Dummy parameter provided by upper layer for command execution
 * @param[in] chan_lid  L2CAP channel local index
 * @param[in] reason    Termination Reason (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC void gatt_bearer_coc_terminated_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t reason)
{
    uint8_t bearer_lid = gatt_bearer_lid_get(conidx, chan_lid);

    if(bearer_lid != GATT_INVALID_BEARER_LID)
    {
        // close the bearer
        gatt_bearer_close(conidx, bearer_lid, reason);
    }
}

/**
 ****************************************************************************************
 * @brief Function called when Connection Oriented Channel Termination operation is finished
 *
 * @param[in] conidx    Connection Index
 * @param[in] dummy     Dummy parameter provided by upper layer for command execution
 * @param[in] chan_lid  Connected L2CAP channel local index
 * @param[in] status    Status of the operation (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC void gatt_bearer_coc_terminate_cmp_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t status)
{
    // Nothing to do.
}

/**
 ****************************************************************************************
 * @brief Function call when peer device requests to create several Connection oriented
 *        channels
 *
 *        In response of this function, the upper layer application must call
 *        @see l2cap_coc_connect_cfm function
 *
 * @param[in] conidx      Connection Index
 * @param[in] token       Token provided by L2CAP module that must be reused in confirmation function
 * @param[in] nb_chan     Number of L2CAP channel requested to be created in parallel
 * @param[in] spsm      Simplified Protocol/Service Multiplexer
 * @param[in] peer_rx_mtu Peer device Maximum Transmit Unit reception size
 ****************************************************************************************
 */
__STATIC void gatt_bearer_coc_connect_req_cb(uint8_t conidx, uint16_t token, uint8_t nb_chan, uint16_t spsm,
                                            uint16_t peer_rx_mtu)
{
    gatt_con_env_t* p_con    = gatt_env.p_con[conidx];
    uint16_t local_rx_mtu    = gatt_user_pref_mtu_get();
    uint8_t  nb_rsvd_chan;

    // compute number of channel that can be allocated
    nb_rsvd_chan = BLE_GATT_BEARER_PER_CON - (CO_BIT_CNT(p_con->bearer_bf) + p_con->nb_bearer_rsvd);
    nb_rsvd_chan = co_min(nb_rsvd_chan, nb_chan);
    p_con->nb_bearer_rsvd += nb_rsvd_chan;

    // Accept connection
    l2cap_coc_connect_cfm(conidx, token, nb_rsvd_chan, local_rx_mtu, &gatt_bearer_eatt_chan_cb);
}

/**
 ****************************************************************************************
 * @brief Handle received message in background loop
 *
 * @param[in] p_sdu_meta  Pointer to Sdu environment
 * @param[in] conidx     Connection index
 * @param[in] bearer_lid Bearer Local Index
 ****************************************************************************************
 */
void gatt_bearer_sdu_rx_bg_handler(gatt_bearer_buf_rx_meta_t* p_sdu_meta, uint8_t conidx, uint16_t bearer_lid)
{
    DBG_FUNC_ENTER(gatt_bearer_sdu_rx_bg_handler);
    co_buf_t* p_sdu;
    gatt_bearer_t* p_bearer;
    uint16_t rx_status = p_sdu_meta->rx_status;
    union l2cap_att_pdu* p_pdu = &(p_sdu_meta->pdu);

    gatt_proc_t* p_proc = NULL;
    uint16_t proc_status = GAP_ERR_NO_ERROR;
    bool clear_rx_busy = false;

    // Retrieve bearer
    ASSERT_ERR(gatt_env.p_con[conidx] != NULL);
    p_bearer = gatt_env.p_con[conidx]->p_bearer[bearer_lid];
    ASSERT_ERR(p_bearer != NULL);

    SETB(p_bearer->info_bf, GATT_BEARER_IN_RX, true);

    // Extract SDU
    p_sdu = (co_buf_t*) co_list_pop_front(&(p_bearer->rx_queue));
    ASSERT_ERR(p_sdu != NULL);

    if(rx_status == GAP_ERR_NO_ERROR)
    {
        bool create_proc_handler = false;

        // if LSB = 1 the message shall be handled by client
        if((p_pdu->code & 0x1) != 0)
        {
            #if (BLE_GATT_CLI)
            // Check if Server event is received
            if(   (p_pdu->code == L2CAP_ATT_HDL_VAL_NTF_OPCODE)
               || (p_pdu->code == L2CAP_ATT_HDL_VAL_IND_OPCODE)
               || (p_pdu->code == L2CAP_ATT_MULT_HDL_VAL_NTF_OPCODE))
            {
                create_proc_handler = true;
            }
            // Attribute transaction response is received
            else if(GETB(p_bearer->info_bf, GATT_BEARER_CLI_TRANS))
            {
                p_proc = gatt_proc_pick(conidx, p_bearer->req_proc_token);

                if(p_proc == NULL)
                {
                    ASSERT_WARN(0, conidx, p_bearer->req_proc_token);
                    p_bearer->req_proc_token = GAP_INVALID_TOKEN;
                    SETB(p_bearer->info_bf, GATT_BEARER_CLI_TRANS, false);
                    // Search if a procedure can be granted on the bearer
                    gatt_proc_check_grant(conidx, bearer_lid, CO_BIT(GATT_PROC_REQ));
                }
            }
            #endif // (BLE_GATT_CLI)
        }
        // else it's handled by server
        else
        {
            // check if client command or request is received
            if((p_pdu->code != L2CAP_ATT_HDL_VAL_CFM_OPCODE))
            {
                if((p_pdu->code != L2CAP_ATT_WR_SIGNED_OPCODE) || !GETB(p_bearer->info_bf, GATT_BEARER_EATT))
                {
                    create_proc_handler = true;
                }
                // else write signed is ignored onto an EATT bearer
            }
            else if(GETB(p_bearer->info_bf, GATT_BEARER_SRV_TRANS))
            {
                p_proc = gatt_proc_pick(conidx, p_bearer->ind_proc_token);

                if(p_proc == NULL)
                {
                    ASSERT_WARN(0, conidx, p_bearer->ind_proc_token);
                    p_bearer->req_proc_token = GAP_INVALID_TOKEN;
                    SETB(p_bearer->info_bf, GATT_BEARER_SRV_TRANS, false);
                    // Search if a procedure can be granted on the bearer
                    gatt_proc_check_grant(conidx, bearer_lid, CO_BIT(GATT_PROC_IND));
                }
            }
        }

        // check if procedure can handle
        if (p_proc != NULL)
        {
            // expected response received
            if(p_pdu->code == (p_proc->tx_opcode + 1))
            {
                ASSERT_ERR(p_proc->cb_pdu_handler != NULL);

                // Handle PDU reception
                proc_status = p_proc->cb_pdu_handler(conidx, p_proc, p_pdu, p_sdu, p_proc->tx_mtu);
            }
            // Request rejected
            else if((p_pdu->code == L2CAP_ATT_ERR_RSP_OPCODE) && (p_pdu->err_rsp.op_code == p_proc->tx_opcode))
            {
                uint16_t reason = p_pdu->err_rsp.reason;

                // if Database out of sync error is detected, inform registered clients
                #if (BLE_GATT_CLI)
                if(reason == ATT_ERR_DB_OUT_OF_SYNC)
                {
                    gatt_cli_event_svc_chg(conidx, true, GATT_MIN_HDL, GATT_MAX_HDL);
                }
                #endif // (BLE_GATT_CLI)

                // ensure that reason is not transaction succeed
                if(reason == GAP_ERR_NO_ERROR)
                {
                    reason = ATT_ERR_UNLIKELY_ERR;
                }

                // An error occurs for response.
                proc_status = reason;
                clear_rx_busy = true;
                SETB(p_bearer->info_bf, GATT_BEARER_RX_BUSY, false);
            }
            // else ignore PDU
            else
            {
                rx_status = ATT_ERR_UNLIKELY_ERR;
            }
        }
        // Else check if a procedure handler should be created
        else if(create_proc_handler)
        {
            gatt_proc_pdu_handler_cb cb_pdu_handler;
            gatt_proc_handler_t* p_proc_handler = NULL;

            // New Client or server initiated procedure should be created to handle indication or notification
            rx_status = gatt_proc_handler_create(conidx, p_pdu->code, bearer_lid, &cb_pdu_handler, &p_proc_handler);

            if (p_proc_handler != NULL)
            {
                // Handle PDU reception
                proc_status = cb_pdu_handler(conidx, (gatt_proc_t*) p_proc_handler, p_pdu, p_sdu, p_bearer->mtu);
                p_proc = (gatt_proc_t*) p_proc_handler;
            }
        }
        // nothing can be done
        else
        {
            rx_status = ATT_ERR_INVALID_PDU;
        }
    }


    if(rx_status != GAP_ERR_NO_ERROR)
    {
        clear_rx_busy = true;
        p_proc = NULL;

        // an error occurs when unpacking PDU, check if an error response should be sent (when receiving a request PDU)
        if(   (((p_pdu->code & 0x1) == 0) || (rx_status == ATT_ERR_REQUEST_NOT_SUPPORTED))
           && !GETB(p_pdu->code, L2CAP_ATT_OPCODE_CMD_FLAG)
           && (p_pdu->code != L2CAP_ATT_HDL_VAL_CFM_OPCODE))
        {
            gatt_bearer_err_rsp_send(conidx, bearer_lid, p_pdu->code, rx_status);
        }
    }

    // release SDU
    co_buf_release(p_sdu);

    if(p_proc != NULL)
    {
        // update procedure state (on background)
        gatt_proc_continue(conidx, p_proc, GATT_PROC_PDU_RX, proc_status);
    }

    if(clear_rx_busy)
    {
        // mark that next received buffer can be handled
        SETB(p_bearer->info_bf, GATT_BEARER_RX_BUSY, false);
    }

    // Get Next SDU
    p_sdu     = (co_buf_t*)co_list_pick(&(p_bearer->rx_queue));

    // Check if another packet is present in reception queue and can be handled immediately
    if(!GETB(p_bearer->info_bf, GATT_BEARER_RX_BUSY) && (p_sdu != NULL))
    {
        SETB(p_bearer->info_bf, GATT_BEARER_RX_BUSY, true);

        // execute next SDU
        p_sdu_meta = (gatt_bearer_buf_rx_meta_t*)co_buf_metadata(p_sdu);
        gapc_sdt_defer(&(p_sdu_meta->defer), bearer_lid);
    }

    SETB(p_bearer->info_bf, GATT_BEARER_IN_RX, false);
    DBG_FUNC_EXIT(gatt_bearer_sdu_rx_bg_handler);
}


/**
 ****************************************************************************************
 * @brief The received SDU buffer must be acquired by upper application module before
 *        function return.
 *        When SDU process is done, the corresponding SDU buffer must be release to
 *        allocate new reception credits onto a L2CAP dynamic channel.
 *
 * @param[in] conidx      Connection Index
 * @param[in] chan_lid    Connected L2CAP channel local index
 * @param[in] status      Reception status
 * @param[in] p_sdu       Buffer that contains SDU data
 ****************************************************************************************
 */
__STATIC void gatt_bearer_sdu_rx_cb(uint8_t conidx, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu)
{
    DBG_FUNC_ENTER(gatt_bearer_sdu_rx_cb);
    uint8_t bearer_lid = gatt_bearer_lid_get(conidx, chan_lid);

    ASSERT_ERR(co_buf_metadata_len(p_sdu) >= sizeof(gatt_bearer_buf_rx_meta_t));

    // do nothing if a complete ATT header has not been received
    if(   (co_buf_data_len(p_sdu) >= L2CAP_ATT_HEADER_LEN) && (bearer_lid != GATT_INVALID_BEARER_LID)
       && (co_buf_metadata_len(p_sdu) >= sizeof(gatt_bearer_buf_rx_meta_t)))
    {
        gatt_bearer_buf_rx_meta_t* p_sdu_meta = (gatt_bearer_buf_rx_meta_t*)co_buf_metadata(p_sdu);
        gatt_bearer_t* p_bearer             = gatt_env.p_con[conidx]->p_bearer[bearer_lid];
        bool     immediate_rx_handle;

        // extract header information
        uint8_t  opcode = co_buf_data(p_sdu)[0];
        uint8_t  method = GETF(opcode, L2CAP_ATT_OPCODE_METHOD);

        co_buf_head_release(p_sdu, L2CAP_ATT_HEADER_LEN);

        if(status == GAP_ERR_NO_ERROR)
        {
                // command can be understood
            if((method >= L2CAP_ATT_OPCODE_MAX) || (gatt_bearer_pdu[method].pack_format == NULL))
            {
                status = ATT_ERR_REQUEST_NOT_SUPPORTED;
            }
               // check if SDU length  is acceptable
            else if(co_buf_data_len(p_sdu) < gatt_bearer_pdu[method].length)
            {
                status = ATT_ERR_INVALID_PDU;
            }
        }
        // Packet MTU exceeds maximum allowed size or another error detected, consider Invalid PDU received
        else
        {
            status = ATT_ERR_INVALID_PDU;
        }

        p_sdu_meta->pdu.code   = opcode;

        // No error detected during message reception
        if(status == GAP_ERR_NO_ERROR)
        {
            uint16_t pdu_len = sizeof(l2cap_att_pdu_t);

            // Perform PDU unpacking
            if(co_util_unpack(((uint8_t*) &(p_sdu_meta->pdu)) + 1, co_buf_data(p_sdu), &pdu_len,
                              gatt_bearer_pdu[method].length,
                              gatt_bearer_pdu[method].pack_format) == CO_UTIL_PACK_OK)
            {
                co_buf_head_release(p_sdu, gatt_bearer_pdu[method].length);
            }
            else
            {
                status = ATT_ERR_INVALID_PDU;
            }
        }

        // Get if SDU can be processed now
        immediate_rx_handle = !GETB(p_bearer->info_bf, GATT_BEARER_RX_BUSY) && co_list_is_empty(&(p_bearer->rx_queue));

        // put SDU in buffer queue
        co_list_push_back(&(p_bearer->rx_queue), &(p_sdu->hdr));

        // prepare background job
        p_sdu_meta->rx_status = status;
        gapc_sdt_prepare(&(p_sdu_meta->defer), conidx, GAPC_SDT_GATT_BEARER);

        // Acquire SDU.
        co_buf_acquire(p_sdu);

        // Handle SDU
        if(immediate_rx_handle)
        {
            SETB(p_bearer->info_bf, GATT_BEARER_RX_BUSY, true);
            gapc_sdt_defer(&(p_sdu_meta->defer), bearer_lid);
        }
    }
    DBG_FUNC_EXIT(gatt_bearer_sdu_rx_cb);
}


/**
 ****************************************************************************************
 * @brief Ask Bearer to send an Error response PDU.
 *
 * @param[in] conidx        Connection index
 * @param[in] bearer_lid    Bearer Local identifier
 * @param[in] op_code       Attribute operation code in error
 * @param[in] reason        Error reason
 *
 * @return Status code of execution request
 ****************************************************************************************
 */
__STATIC uint16_t gatt_bearer_err_rsp_send(uint8_t conidx, uint8_t bearer_lid, uint8_t op_code, uint16_t reason)
{
    uint16_t status;
    co_buf_t* p_data;

    // prepare unpacked version of attribute PDU
    l2cap_att_err_rsp_t pdu =
    {
        .code    = L2CAP_ATT_ERR_RSP_OPCODE,
        .op_code = op_code,
        .handle  = 0,
        .reason  = (reason & 0xFF),
    };

    // allocate buffer used for attribute transmission
    if(co_buf_alloc(&p_data, GATT_BUFFER_HEADER_LEN, 0, GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
    {
        // Ask for PDU transmission
        status = gatt_bearer_pdu_send(conidx, bearer_lid, 0, (l2cap_att_pdu_t*) &pdu, p_data);

        // release buffer
        co_buf_release(p_data);
    }
    else
    {
        status = GAP_ERR_INSUFF_RESOURCES;
    }

    return (status);
}

void gatt_bearer_eatt_estab_timer_handler(gapc_sdt_t* p_hdl, uint8_t conidx)
{
    gatt_bearer_eatt_estab(conidx);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

void gatt_bearer_close(uint8_t conidx, uint8_t bearer_lid, uint16_t reason)
{
    gatt_con_env_t* p_con   = gatt_env.p_con[conidx];
    gatt_bearer_t* p_bearer = p_con->p_bearer[bearer_lid];
    // remove access to the bearer
    p_con->p_bearer[bearer_lid] = NULL;
    // mark bearer closed
    CO_BIT_SET(&(p_con->bearer_bf), bearer_lid, false);

    ASSERT_ERR(p_bearer != NULL);
    if(p_bearer != NULL)
    {
        if(reason != GAP_ERR_DISCONNECTED)
        {
            gatt_proc_t* p_proc = (gatt_proc_t*) co_list_pick(&(p_con->proc_exe_queue));
            gatt_proc_t* p_proc_next;

            // check procedure in execution
            while(p_proc != NULL)
            {
                // get next procedure because procedure remove itself from procedure queue.
                p_proc_next = (gatt_proc_t*) p_proc->hdr.next;

                // If Procedure is granted on the bearer
                if(p_proc->bearer_lid == bearer_lid)
                {
                    // Mark procedure in error
                    gatt_proc_continue(conidx, p_proc, GATT_PROC_ERROR, reason);
                }

                p_proc = p_proc_next;
            }

            // Remove L2CAP channel
            if(!GETB(p_bearer->info_bf, GATT_BEARER_EATT))
            {
                l2cap_chan_unregister(conidx, GETF(p_bearer->info_bf, GATT_BEARER_CHAN_LID));
                SETB(p_con->state_bf, GATT_CON_LEGACY_ATT_BEARER_OPEN, false);
            }
            else if(reason == GATT_ERR_ATT_BEARER_CLOSE)
            {
                l2cap_coc_terminate(conidx, gapm_token_id_get(), GETF(p_bearer->info_bf, GATT_BEARER_CHAN_LID));
            }
        }

        // remove buffers in RX queue
        while(!co_list_is_empty(&(p_bearer->rx_queue)))
        {
            co_buf_t* p_sdu = (co_buf_t*) co_list_pop_front(&(p_bearer->rx_queue));
            gatt_bearer_buf_rx_meta_t* p_sdu_meta = (gatt_bearer_buf_rx_meta_t*)co_buf_metadata(p_sdu);

            // release buffer
            gapc_sdt_stop(&(p_sdu_meta->defer));
            co_buf_release(p_sdu);
        }

        // check if no other bearer available, if yes clean-up all on-going procedures, clean-up prepare write queue,
        if(p_con->bearer_bf == 0)
        {
            // clean-up server write queue
            gatt_srv_write_queue_cleanup(conidx);

            // clean-up active procedures
            gatt_proc_cleanup(conidx, reason);

            if(reason != GAP_ERR_DISCONNECTED)
            {
                // and inform application that no more bearer are available.
                gapc_att_bearer_error_send(conidx);
            }
        }

        // Remove Bearer memory
        ke_free(p_bearer);
    }
}

uint16_t gatt_bearer_pdu_send(uint8_t conidx, uint8_t bearer_lid, uint16_t token, l2cap_att_pdu_t* p_pdu,
                              co_buf_t* p_data)
{
    DBG_FUNC_ENTER(gatt_bearer_pdu_send);
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    do
    {
        uint8_t         method          = GETF(p_pdu->code, L2CAP_ATT_OPCODE_METHOD);
        co_buf_t*       p_sdu           = NULL;
        uint16_t        att_msg_len     = gatt_bearer_pdu[method].length;
        uint16_t        l2cap_msg_len   = L2CAP_BUFFER_HEADER_LEN + L2CAP_ATT_HEADER_LEN + att_msg_len;
        uint8_t         chan_lid        = gatt_bearer_chan_lid_get(conidx, bearer_lid);

        if(   (method >= L2CAP_ATT_OPCODE_MAX) || (gatt_bearer_pdu[method].pack_format == NULL)
           || (chan_lid == L2CAP_INVALID_CHAN_LID))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // if buffer head length or tail length is not sufficient
        if((co_buf_head_len(p_data) < l2cap_msg_len) || (co_buf_tail_len(p_data) < L2CAP_BUFFER_TAIL_LEN))
        {
//            // duplicate data onto another buffer - remove possibility to duplicate buffer - must be handled in procedure
//            if(co_buf_duplicate(p_data, &p_sdu, l2cap_msg_len, L2CAP_BUFFER_TAIL_LEN) != CO_BUF_ERR_NO_ERROR)
//            {
//                status = GAP_ERR_INSUFF_RESOURCES;
//                break;
//            }
//
            status = GAP_ERR_INVALID_BUFFER;
            break;
        }
        else
        {
            p_sdu = p_data;
            co_buf_acquire(p_sdu);
        }

        // Prepare attribute
        co_buf_head_reserve(p_sdu, att_msg_len);

        // Perform PDU packing
        if(co_util_pack(co_buf_data(p_sdu), ((uint8_t*) &(p_pdu->code) + 1), &(att_msg_len), sizeof(l2cap_att_pdu_t),
                        gatt_bearer_pdu[method].pack_format) != CO_UTIL_PACK_OK)
        {
            status = GAP_ERR_PROTOCOL_PROBLEM;
            co_buf_release(p_sdu);
            break;
        }

        // Push ATT OPCODE
        co_buf_head_reserve(p_sdu, 1);
        co_buf_data(p_sdu)[0] = p_pdu->code;

        // Send Signaling packet
        status = l2cap_chan_sdu_send(conidx, token, chan_lid, p_sdu);

        // release buffer
        co_buf_release(p_sdu);
    } while(0);

    DBG_FUNC_EXIT(gatt_bearer_pdu_send);

    return (status);
}

uint16_t gatt_bearer_create(gatt_con_env_t* p_con, uint8_t conidx)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    uint8_t bearer_lid = 0;
    // Allocate Default GATT Bearer.
    gatt_bearer_t* p_bearer = (gatt_bearer_t*) ke_malloc_user(sizeof(gatt_bearer_t), KE_MEM_ENV);

    if(p_bearer != NULL)
    {
        uint8_t chan_lid = L2CAP_INVALID_CHAN_LID;
        memset(p_bearer, 0, sizeof(gatt_bearer_t));

        // Create new L2CAP Fix Channel that uses ATT CID using default ATT_MTU
        status = l2cap_chan_fix_register(conidx, L2CAP_CID_ATTRIBUTE, L2CAP_LE_MTU_MIN,
                                         &gatt_bearer_fix_chan_cb, &(chan_lid));

        if(status == GAP_ERR_NO_ERROR)
        {
            // Initialize bearer parameters
            p_bearer->mtu     = L2CAP_LE_MTU_MIN;
            SETF(p_bearer->info_bf, GATT_BEARER_CHAN_LID, chan_lid);
            p_con->p_bearer[bearer_lid] = p_bearer;
            CO_BIT_SET(&(p_con->bearer_bf), bearer_lid, true);
            SETB(p_con->state_bf, GATT_CON_LEGACY_ATT_BEARER_OPEN, true);
        }
        else
        {
            ke_free(p_bearer);
            ASSERT_WARN(0, conidx, 0);
        }
    }

    return (status);
}

bool gatt_bearer_acquire(uint8_t conidx, uint8_t bearer_lid, gatt_proc_t* p_proc, uint16_t pref_mtu)
{
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];
    uint16_t bearer_bf = p_con->bearer_bf;
    uint8_t found_bearer_lid = GATT_INVALID_BEARER_LID;
    uint16_t tx_length = p_proc->token;

    gatt_bearer_t* p_bearer;
    bool granted = false;
    uint16_t mode = 0;
    uint16_t mtu_max = 0;
    uint16_t pdu_max = 0;
    uint8_t proc_type = GATT_PROC_TYPE_GET(p_proc->proc_id);

    switch(proc_type)
    {
        case GATT_PROC_IND: { mode = GATT_BEARER_SRV_IND_PROC_BIT; } break;
        case GATT_PROC_NTF: { mode = GATT_BEARER_SRV_NTF_PROC_BIT; } break;
        #if (BLE_GATT_CLI)
        case GATT_PROC_REQ: { mode = GATT_BEARER_CLI_REQ_PROC_BIT; } break;
        case GATT_PROC_CMD: { mode = GATT_BEARER_CLI_CMD_PROC_BIT; } break;
        #endif // (BLE_GATT_CLI)
        default:            { ASSERT_ERR(0); /* Not supported */   } break;
    }

    // specific bearer LID for MTU exchange
    if(p_proc->bearer_lid != GATT_INVALID_BEARER_LID)
    {
        bearer_bf = CO_BIT(p_proc->bearer_lid);
    }
    else
    {
        // check if we should search on all bearer
        if(bearer_lid != GATT_INVALID_BEARER_LID)
        {
            bearer_bf = CO_BIT(bearer_lid);
        }

        // if Legacy ATT bearer is opened and some other ATT bearer present
        if(GETB(p_con->state_bf, GATT_CON_LEGACY_ATT_BEARER_OPEN) && ((bearer_bf & ~CO_BIT(0)) != 0))
        {
            // do not use legacy ATT bearer
            bearer_bf = (bearer_bf & ~CO_BIT(0));
        }

        bearer_lid = CO_VAL_INC(p_con->last_grant_bearer_lid, BLE_GATT_BEARER_PER_CON);
    }

    // search on which bearer procedure can be granted
    while(bearer_bf != 0)
    {
        gatt_bearer_t* p_bearer;
        // Use round robin to change (if possible) bearer for each granted procedure in order to have more diversity
        uint16_t rn_robin_mask = ~(CO_BIT(bearer_lid)-1);
        bearer_lid = ((bearer_bf & rn_robin_mask) != 0)
                   ? co_ctz(bearer_bf & rn_robin_mask)
                   : co_ctz(bearer_bf);

        p_bearer = p_con->p_bearer[bearer_lid];
        CO_BIT_SET(&(bearer_bf), bearer_lid, false);

        // check if bearer can accept requested attribute transaction
        if((p_bearer != NULL) && ((p_bearer->info_bf & mode) == 0))
        {
            uint8_t  chan_lid  = GETF(p_bearer->info_bf, GATT_BEARER_CHAN_LID);
            uint16_t sdu_max   = l2cap_chan_max_sdu_tx_size_get(conidx, chan_lid);
            sdu_max   = co_min(sdu_max, p_bearer->mtu);

            // Consider first bearer as a valid bearer
            if(found_bearer_lid == GATT_INVALID_BEARER_LID)
            {
                found_bearer_lid = bearer_lid;
                mtu_max = p_bearer->mtu;
                pdu_max = sdu_max;
            }
            // check if we can transmit a greater PDU
            else if(   ((tx_length > pdu_max) && (pdu_max < sdu_max))
                    // or if MTU is important find bearer closed to preferred MTU
                    || (GATT_PROC_PREF_MTU_CHK(p_proc->proc_id) && (tx_length <= sdu_max) &&(p_bearer->mtu > mtu_max)))
            {
                found_bearer_lid = bearer_lid;
                mtu_max = p_bearer->mtu;
                pdu_max = sdu_max;
            }

            // consider that search is over
            if((tx_length <= pdu_max) && (!GATT_PROC_PREF_MTU_CHK(p_proc->proc_id) || (p_bearer->mtu >= pref_mtu)))
            {
                break;
            }
        }
    }

    // check if write procedure must be switched to a write long procedure
    if((found_bearer_lid != GATT_INVALID_BEARER_LID) && (p_proc->proc_id == GATT_PROC_WRITE) && (tx_length > pdu_max))
    {
        p_proc->proc_id = GATT_PROC_WRITE_LONG;

        // Only one write long procedure is supported at a specific instant
        if(GETB(p_con->state_bf, GATT_CON_WRITE_QUEUE_MUTEX))
        {
            found_bearer_lid = GATT_INVALID_BEARER_LID;
        }
    }

    // Bearer has been found, grant the procedure
    if(found_bearer_lid != GATT_INVALID_BEARER_LID)
    {
        p_proc->bearer_lid = found_bearer_lid;
        p_proc->token      = gapm_token_id_get();
        p_bearer = p_con->p_bearer[p_proc->bearer_lid];
        p_bearer->info_bf |= mode;

        if(proc_type == GATT_PROC_IND)
        {
            p_bearer->ind_proc_token = p_proc->token;
            SETB(p_bearer->info_bf, GATT_BEARER_SRV_TRANS, true);
        }
        #if (BLE_GATT_CLI)
        else if(proc_type == GATT_PROC_REQ)
        {
            p_bearer->req_proc_token = p_proc->token;
            SETB(p_bearer->info_bf, GATT_BEARER_CLI_TRANS, true);
        }
        #endif // (BLE_GATT_CLI)

        granted  = true;
        p_con->last_grant_bearer_lid = found_bearer_lid;
    }

    return (granted);
}

void gatt_bearer_release(uint8_t conidx, gatt_proc_t* p_proc)
{
    if(p_proc->bearer_lid != GATT_INVALID_BEARER_LID)
    {
        gatt_con_env_t* p_con = gatt_env.p_con[conidx];
        gatt_bearer_t* p_bearer;
        ASSERT_ERR(p_proc->bearer_lid < BLE_GATT_BEARER_PER_CON);

        p_bearer = p_con->p_bearer[p_proc->bearer_lid];

        if(p_bearer != NULL)
        {
            // according to procedure type, release the usage status
            switch(GATT_PROC_TYPE_GET(p_proc->proc_id))
            {
                case GATT_PROC_IND: { SETB(p_bearer->info_bf, GATT_BEARER_SRV_IND_PROC, false);
                                      p_bearer->ind_proc_token = GAP_INVALID_TOKEN;              } break;
                case GATT_PROC_NTF: { SETB(p_bearer->info_bf, GATT_BEARER_SRV_NTF_PROC, false);  } break;
                #if (BLE_GATT_CLI)
                case GATT_PROC_REQ: { SETB(p_bearer->info_bf, GATT_BEARER_CLI_REQ_PROC, false);
                                      p_bearer->req_proc_token = GAP_INVALID_TOKEN;              } break;
                case GATT_PROC_CMD: { SETB(p_bearer->info_bf, GATT_BEARER_CLI_CMD_PROC, false);  } break;
                #endif // (BLE_GATT_CLI)
                default:            { /* Nothing to do */                                        } break;
            }

            // Check if another procedure with same type can be granted.
            gatt_proc_check_grant(conidx, p_proc->bearer_lid, CO_BIT(GATT_PROC_TYPE_GET(p_proc->proc_id)));
        }
    }
}

uint16_t gatt_bearer_mtu_get(uint8_t conidx, uint8_t bearer_lid)
{
    ASSERT_ERR(gatt_env.p_con[conidx] != NULL);
    ASSERT_ERR(gatt_env.p_con[conidx]->p_bearer[bearer_lid] != NULL);

    return (gatt_env.p_con[conidx]->p_bearer[bearer_lid]->mtu);
}

bool gatt_bearer_mtu_exch_supported(uint8_t conidx, uint8_t* p_bearer_lid)
{
    bool supported = false;

    // sanity check on connection
    if((conidx < HOST_CONNECTION_MAX) && (gatt_env.p_con[conidx] != NULL))
    {
        gatt_con_env_t* p_con = gatt_env.p_con[conidx];

        // check if Legacy bearer present and MTU not already exchanged
        if(GETB(p_con->state_bf, GATT_CON_LEGACY_ATT_BEARER_OPEN))
        {
            *p_bearer_lid = 0; // Legacy ATT bearer is always 0
            supported = true;
        }
    }

    return (supported);
}

void gatt_bearer_mtu_set(uint8_t conidx, uint8_t bearer_lid, uint16_t mtu)
{
    gatt_con_env_t* p_con    = gatt_env.p_con[conidx];
    gatt_bearer_t* p_bearer;
    ASSERT_ERR(p_con != NULL);
    ASSERT_ERR(p_con->p_bearer[bearer_lid] != NULL);
    p_bearer = p_con->p_bearer[bearer_lid];
    ASSERT_ERR(mtu >= p_bearer->mtu);
    p_bearer->mtu = mtu;

    // mark MTU exchanged for a legacy ATT bearer
    if(!GETB(p_bearer->info_bf, GATT_BEARER_EATT))
    {
        // update MTU value on fixed channel
        l2cap_chan_fix_mtu_update(conidx, GETF(p_bearer->info_bf, GATT_BEARER_CHAN_LID), mtu);
    }
}

void gatt_bearer_rx_continue(uint8_t conidx, uint8_t bearer_lid)
{
    DBG_FUNC_ENTER(gatt_bearer_rx_continue);
    gatt_con_env_t* p_con    = gatt_env.p_con[conidx];
    gatt_bearer_t* p_bearer;
    co_buf_t*      p_sdu;
    ASSERT_ERR(p_con != NULL);
    ASSERT_ERR(p_con->p_bearer[bearer_lid] != NULL);

    p_bearer = p_con->p_bearer[bearer_lid];
    p_sdu = (co_buf_t*) co_list_pick(&(p_bearer->rx_queue));

    // if there is an SDU in reception queue and we are not in reception procedure
    if(!GETB(p_bearer->info_bf, GATT_BEARER_IN_RX) && (p_sdu != NULL))
    {
        gatt_bearer_buf_rx_meta_t* p_sdu_meta = (gatt_bearer_buf_rx_meta_t*)co_buf_metadata(p_sdu);
        // ask reception start of next buffer
        gapc_sdt_defer(&(p_sdu_meta->defer), bearer_lid);
    }
    else
    {
        // mark that reception no more busy
        SETB(p_bearer->info_bf, GATT_BEARER_RX_BUSY, false);
    }
    DBG_FUNC_EXIT(gatt_bearer_rx_continue);
}

uint16_t gatt_bearer_setup(void)
{
    // Register channel
    uint8_t sec_lvl = 0;
    uint16_t status;

    SETF(sec_lvl, L2CAP_COC_AUTH, GAP_SEC_UNAUTH); // Unauthenticated pairing required
    status = l2cap_coc_spsm_add(L2CAP_SPSM_ATT, sec_lvl, &gatt_bearer_coc_cb);

    return (status);
}

void gatt_bearer_eatt_estab(uint8_t conidx)
{
    uint8_t  nb_chan;
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];
    uint16_t local_rx_mtu = gatt_user_pref_mtu_get();
    uint8_t  nb_chan_exp  = gatt_user_cli_nb_get() + GETB(p_con->state_bf, GATT_CON_LEGACY_ATT_BEARER_OPEN);
    nb_chan_exp           = co_min(nb_chan_exp, BLE_GATT_BEARER_PER_CON);

    // compute number of channel that can be allocated
    nb_chan = CO_BIT_CNT(p_con->bearer_bf) + p_con->nb_bearer_rsvd;

    // check if there is new channel to create
    if(nb_chan_exp > nb_chan)
    {
        uint8_t  nb_rsvd_chan  = nb_chan_exp - nb_chan;
        p_con->nb_bearer_rsvd += nb_rsvd_chan;

        // Start creation of L2CAP channels - put number of reserved channel in parameters
        l2cap_coc_create(conidx, nb_rsvd_chan, L2CAP_SPSM_ATT, nb_rsvd_chan, local_rx_mtu,
                         &gatt_bearer_eatt_chan_cb);
    }
}

#if (RW_DEBUG && HOST_MSG_API)
/*
 * MESSAGE HANDLER FUNCTIONS
 ****************************************************************************************
 */
#include "gatt_msg_int.h"

/**
 ****************************************************************************************
 * @brief Handle Command used to retrieve internal ATT bearer information
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_dbg_bearer_info_get_cmd_handler(gatt_dbg_bearer_info_get_cmd_t* p_cmd, uint16_t src_id)
{
    gatt_dbg_bearer_info_get_cmp_evt_t* p_cmp_evt =
            KE_MSG_ALLOC_DYN(GATT_CMP_EVT, src_id, TASK_GATT, gatt_dbg_bearer_info_get_cmp_evt,
                             sizeof(gatt_dbg_bearer_info_t) * BLE_GATT_BEARER_PER_CON);

    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->cmd_code  = p_cmd->cmd_code;
        p_cmp_evt->dummy     = p_cmd->dummy;
        p_cmp_evt->user_lid  = p_cmd->user_lid;
        p_cmp_evt->conidx    = p_cmd->conidx;
        p_cmp_evt->nb_bearer = 0;

        if((p_cmd->conidx > HOST_CONNECTION_MAX) || (gatt_env.p_con[p_cmd->conidx] == NULL))
        {
            p_cmp_evt->status = GAP_ERR_COMMAND_DISALLOWED;
        }
        else
        {
            gatt_con_env_t* p_con = gatt_env.p_con[p_cmd->conidx];
            uint8_t bearer_lid;
            for(bearer_lid = 0 ; bearer_lid < BLE_GATT_BEARER_PER_CON ; bearer_lid++)
            {
                gatt_bearer_t* p_bearer = p_con->p_bearer[bearer_lid];
                if(p_bearer != NULL)
                {
                    p_cmp_evt->bearers[p_cmp_evt->nb_bearer].bearer_lid = bearer_lid;
                    p_cmp_evt->bearers[p_cmp_evt->nb_bearer].chan_lid   = GETF(p_bearer->info_bf, GATT_BEARER_CHAN_LID);
                    p_cmp_evt->bearers[p_cmp_evt->nb_bearer].eatt       = GETB(p_bearer->info_bf, GATT_BEARER_EATT);
                    p_cmp_evt->bearers[p_cmp_evt->nb_bearer].mtu        = p_bearer->mtu;
                    p_cmp_evt->nb_bearer++;
                }
            }

            p_cmp_evt->status = GAP_ERR_NO_ERROR;
        }

        // send message to host
        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Handle command used to close a specific ATT Bearer
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_dbg_bearer_close_cmd_handler(gatt_dbg_bearer_close_cmd_t* p_cmd, uint16_t src_id)
{
    uint16_t status;

    if(   (p_cmd->conidx > HOST_CONNECTION_MAX) || (gatt_env.p_con[p_cmd->conidx] == NULL)
       || (p_cmd->bearer_lid > BLE_GATT_BEARER_PER_CON))
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else
    {
        gatt_con_env_t* p_con = gatt_env.p_con[p_cmd->conidx];
        gatt_bearer_t* p_bearer = p_con->p_bearer[p_cmd->bearer_lid];

        if(p_bearer == NULL)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
        }
        else
        {
            gatt_bearer_close(p_cmd->conidx, p_cmd->bearer_lid, GATT_ERR_ATT_BEARER_CLOSE);
            status = GAP_ERR_NO_ERROR;
        }
    }

    // Directly send back result, no wait for end of procedure
    gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
}

/**
 ****************************************************************************************
 * @brief Handle command use to force establishment of ATT bearers
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_dbg_bearer_eatt_estab_cmd_handler(gatt_dbg_bearer_eatt_estab_cmd_t* p_cmd, uint16_t src_id)
{
    uint16_t status;

    if(   (p_cmd->conidx > HOST_CONNECTION_MAX) || (gatt_env.p_con[p_cmd->conidx] == NULL))
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else
    {
        // force establishment of new EATT bearers
        gatt_bearer_eatt_estab(p_cmd->conidx);
        status = GAP_ERR_NO_ERROR;
    }

    // Directly send back result, no wait for end of procedure
    gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
}

#endif // (RW_DEBUG && HOST_MSG_API)

uint16_t gatt_bearer_mtu_min_get(uint8_t conidx)
{
    uint16_t min_mtu = L2CAP_LE_MTU_MIN;

    if(conidx < HOST_CONNECTION_MAX)
    {
        gatt_con_env_t* p_con = gatt_env.p_con[conidx];
        if((p_con != NULL) && (p_con->bearer_bf != 0))
        {
            uint16_t bearer_bf = p_con->bearer_bf;
            min_mtu = 0xFFFF;

            // browse available bearer to compute minimum available MTU
            while(bearer_bf != 0)
            {
                uint8_t cursor = co_ctz(bearer_bf);
                gatt_bearer_t* p_bearer = p_con->p_bearer[cursor];
                CO_BIT_SET(&(bearer_bf), cursor, false);

                // search minimum negociated MTU
                if(p_bearer->mtu < min_mtu)
                {
                    min_mtu = p_bearer->mtu;
                }
            }
        }
    }

    return (min_mtu);
}


#endif // (BLE_GATT)
/// @} GATT

