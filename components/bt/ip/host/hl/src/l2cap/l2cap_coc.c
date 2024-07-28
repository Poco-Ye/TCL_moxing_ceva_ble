/**
 ****************************************************************************************
 * @file l2cap_coc.c
 *
 * @brief  L2CAP Connection oriented channel manager
 *
 * Copyright (C) RivieraWaves 2017-2019
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
#include "rwip_config.h"         // IP configuration
#if (BLE_L2CAP)
#include "l2cap.h"               // Native API
#include "l2cap_int.h"           // Internals
#include "../inc/l2cap_sig.h"    // Signaling
#include "../inc/l2cap_hl_api.h" // Channel un-register
#include "ke_mem.h"              // Memory allocation usage
#include "gap.h"                 // HL defines
#include "gapc.h"                // Connection specific HL defines
#include "co_math.h"             // Mathematical utilities
#include "co_endian.h"           // Endianess management

#include <string.h>              // For memset

/*
 * MACROS
 ****************************************************************************************
 */

/// Check if channel ID is within the correct range
#define L2CAP_IS_DYNAMIC_CID(cid) ((cid >= L2CAP_CID_DYN_MIN) && (cid <= L2CAP_CID_DYN_MAX))

/// Check if LE PSM is within the correct range
#define L2CAP_IS_SPSM_VALID(spsm) (spsm != L2CAP_SPSM_RESERVED)

/// Number of channel that can be create in parallel
#define L2CAP_SIMULTANEOUS_CREATE_CHAN_NB   (5)

// Generate local reception CID from L2CAP channel identifier
// this ensure that there is no duplicated CID (note: this does not check that some channel uses debug mode)
#define L2CAP_RXCID_GET(chan_lid)   (L2CAP_CID_DYN_MIN + 0x10 + (chan_lid))

/*
 * DEFINES
 ****************************************************************************************
 */
/** L2CAP LE PSM limits */
enum l2cap_spsm_limits
{
    /// Reserved
    L2CAP_SPSM_RESERVED                   = 0x0000,
    /// Fixed minimum range SIG assigned
    L2CAP_SPSM_FIXED_MIN                  = 0x0001,
    /// Fixed maximum range SIG assigned
    L2CAP_SPSM_FIXED_MAX                  = 0x007F,
    /// Dynamic minimum range SIG assigned
    L2CAP_SPSM_DYN_MIN                    = 0x0080,
    /// Dynamic maximum range SIG assigned
    L2CAP_SPSM_DYN_MAX                    = 0x00FF,
    /// Reserved minimum range SIG assigned
    L2CAP_SPSM_RSV_MIN                    = 0x0100,
    /// Reserved maximum range SIG assigned
    L2CAP_SPSM_RSV_MAX                    = 0xFFFF,
};

/// Result values for LE L2CAP Based Connection Response
enum l2cap_coc_rsp_value
{
    /// connection successful
    L2CAP_COC_ERR_SUCCESS                 = 0x0000,
    /// Connection refused - SPSM not supported
    L2CAP_COC_ERR_SPSM_NOT_SUPP           = 0x0002,
    /// Some Connection refused - Insufficient resources available
    L2CAP_COC_ERR_INS_RES_AVAIL           = 0x0004,
    /// Connection refused - insufficient authentication
    L2CAP_COC_ERR_INS_AUTH                = 0x0005,
    /// Connection refused - insufficient authorization
    L2CAP_COC_ERR_INS_AUTHOR              = 0x0006,
    /// Connection refused - insufficient encryption key size
    L2CAP_COC_ERR_INS_EKS                 = 0x0007,
    /// Connection Refused - insufficient encryption
    L2CAP_COC_ERR_INS_ENCRYPTION          = 0x0008,
    /// Some Connection Refused - invalid Source CID
    L2CAP_COC_ERR_INVALID_SRC_CID         = 0x0009,
    /// Some Connection Refused - Source CID already allocated
    L2CAP_COC_ERR_SRC_CID_ALREADY_ALLOC   = 0x000A,
    /// Connection Refused - Unacceptable parameters
    L2CAP_COC_ERR_UNACCEPTABLE_PARAM      = 0x000B,
    /// Connection Refused - Invalid connection parameters
    L2CAP_COC_ERR_INVALID_PARAM           = 0x000C,
};


/// Result values for L2CAP Credit Based reconfigure Response
enum l2cap_coc_rcfg_rsp_value
{
    /// Reconfiguration successful
    L2CAP_COC_RCFG_ERR_SUCCESS            = 0x0000,
    /// Reconfiguration failed - reduction in size of MTU not allowed
    L2CAP_COC_RCFG_ERR_MTU_RED_FORBIDEN   = 0x0001,
    /// Reconfiguration failed - reduction in size of MPS not allowed for more than one channel at a time
    L2CAP_COC_RCFG_ERR_MPS_RED_REJECTED   = 0x0002,
    /// Reconfiguration failed - one or more Destination CIDs invalid
    L2CAP_COC_RCFG_ERR_INVALID_DST_CID    = 0x0003,
    /// Reconfiguration failed - other unacceptable parameters
    L2CAP_COC_RCFG_ERR_UNACCEPTABLE_PARAM = 0x0004,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// Structure used to retrieve information about COC connection request
typedef struct l2cap_coc_proc_connect
{
    /// Procedure header - required for any signaling procedure
    l2cap_sig_proc_t            sig;
    /// Channel connection callback set
    const l2cap_chan_coc_cb_t*  p_cb;
    /// Simplified Protocol/Service Multiplexer
    uint16_t                    spsm;
    /// Maximum Transmission Unit
    uint16_t                    mtu;
    /// Maximum PDU Size
    uint16_t                    mps;
    /// Initial credits
    uint16_t                    initial_credits;
    /// Source Channel Identifiers
    uint16_t                    scids[L2CAP_SIMULTANEOUS_CREATE_CHAN_NB];
    /// Current status for channel creation
    uint16_t                    status;
    /// Allocated channel local identifier
    uint8_t                     chan_lid[L2CAP_SIMULTANEOUS_CREATE_CHAN_NB];
    /// number of channel requested
    uint8_t                     nb_chan;
    /// number of channel accepted
    uint8_t                     nb_chan_accepted;
    /// number of channel allocated
    uint8_t                     nb_chan_alloc;
    /// True: Use Enhanced Credit Based Flow Control Mode, False: use LE Credit Based Flow Control Mode
    bool                        enhanced;
    /// To check that confirmation message has been received
    bool                        cfm_received;
} l2cap_coc_proc_connect_t;



/// Structure used to retrieve information about COC Creation
typedef struct l2cap_coc_proc_create
{
    /// Procedure header - required for any signaling procedure
    l2cap_sig_proc_t            sig;
    /// Channel creation callback set
    const l2cap_chan_coc_cb_t*  p_cb;
    /// Dummy parameter that must be returned when event are triggered during procedure execution
    uint16_t                    dummy;
    /// LE PSM to register
    uint16_t                    spsm;
    /// Local reception MTU
    uint16_t                    mtu;
    /// Local reception MPS
    uint16_t                    mps;
    /// Local reception CREDIT
    uint16_t                    credit;

    /// number of channel to create
    uint8_t                     nb_chan;
    /// number of channel created
    uint8_t                     nb_chan_created;
    /// number of channel allocated
    uint8_t                     nb_chan_alloc;
    /// True: Use Enhanced Credit Based Flow Control Mode, False: use LE Credit Based Flow Control Mode
    bool                        enhanced;
    /// Allocated channel local identifier
    uint8_t                     chan_lid[L2CAP_SIMULTANEOUS_CREATE_CHAN_NB];
} l2cap_coc_proc_create_t;

/// Structure used to retrieve information about COC connection request
typedef struct l2cap_coc_proc_reconfigure_req
{
    /// Procedure header - required for any signaling procedure
    l2cap_sig_proc_t            sig;
    /// Channel callback set
    const l2cap_chan_coc_cb_t*  p_cb;
    /// Dummy parameter that must be returned when event are triggered during procedure execution
    uint16_t                    dummy;
    /// New maximum number of credit for the channels
    uint16_t                    new_rx_credit_max;
    /// New Maximum Transmission Unit
    uint16_t                    new_mtu;
    /// New Maximum PDU Size
    uint16_t                    new_mps;
    /// Cursor to channel array
    uint8_t                     cursor;
    /// Number of channel present in reconfigure request
    uint8_t                     nb_chan_req;
    /// Number of channel to reconfigure
    uint8_t                     nb_chan;
    /// Array of L2CAP channel local index to reconfigure
    uint8_t                     chan_lid[__ARRAY_EMPTY];
} l2cap_coc_proc_reconfigure_req_t;


/// Structure used to retrieve information about COC connection response
typedef struct l2cap_coc_proc_reconfigure_rsp
{
    /// Procedure header - required for any signaling procedure
    l2cap_sig_proc_t            sig;
    /// New Maximum Transmission Unit
    uint16_t                    new_mtu;
    /// New Maximum PDU Size
    uint16_t                    new_mps;
    /// Destination CID
    uint16_t                    dcids[L2CAP_CHAN_NEGO_NB];
    /// Channel where TX flow must be paused
    uint8_t                     tx_flow_off_chan_lid;
    /// Used to know if TX Flow off has been performed
    bool                        tx_flow_off_done;
    /// Array of L2CAP channel local index to reconfigure
    uint8_t                     chan_lid[L2CAP_CHAN_NEGO_NB];
    /// number of channel to reconfigure
    uint8_t                     nb_chan;
    /// Semaphore use to know if procedure is processing something
    bool                        in_proc;
} l2cap_coc_proc_reconfigure_rsp_t;

/// Structure used to retrieve information about COC Termination
typedef struct l2cap_coc_proc_terminate
{
    /// Procedure header - required for any signaling procedure
    l2cap_sig_proc_t            sig;
    /// Channel termination callback set
    const l2cap_chan_coc_cb_t*  p_cb;
    /// Dummy parameter that must be returned when event are triggered during procedure execution
    uint16_t                    dummy;
    /// Channel local identifier
    uint8_t                     chan_lid;
    /// Termination reason
    uint8_t                     reason;
    /// Source Channel identifier
    uint8_t                     scid;
    /// Destination Channel identifier
    uint8_t                     dcid;
    /// Used to know if it's an automatic termination
    bool                        error_detected;
} l2cap_coc_proc_terminate_t;


/// Structure used to retrieve information about COC Credit ADD
typedef struct l2cap_coc_proc_credit_add
{
    /// Procedure header - required for any signaling procedure
    l2cap_sig_proc_t     sig;
    /// Channel local identifier
    uint8_t              chan_lid;
    /// Source Channel identifier
    uint8_t              cid;
    /// Number of credit to add
    uint16_t             credits;
} l2cap_coc_proc_credit_add_t;


/*
 * LOCAL FUNCTION DEFINITION
 ****************************************************************************************
 */

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
 * @brief Retrieve Host error code from L2CAP COC connection error
 *
 * @param[in] l_err      L2CAP COC connection error code
 *
 * @return Host Error code
 ****************************************************************************************
 */
__STATIC uint16_t l2cap_coc_l2h_err(uint8_t l_err)
{
    uint16_t result;
    switch (l_err)
    {
        case L2CAP_COC_ERR_SUCCESS:                 { result = GAP_ERR_NO_ERROR;                } break;
        case L2CAP_COC_ERR_SPSM_NOT_SUPP:           { result = L2CAP_ERR_PSM_SPSM_NOT_SUPP;         } break;
        case L2CAP_COC_ERR_INS_RES_AVAIL:           { result = GAP_ERR_INSUFF_RESOURCES;        } break;
        case L2CAP_COC_ERR_INS_AUTH:                { result = L2CAP_ERR_INSUFF_AUTHEN;         } break;
        case L2CAP_COC_ERR_INS_AUTHOR:              { result = L2CAP_ERR_INSUFF_AUTHOR;         } break;
        case L2CAP_COC_ERR_INS_EKS:                 { result = L2CAP_ERR_INSUFF_ENC_KEY_SIZE;   } break;
        case L2CAP_COC_ERR_INS_ENCRYPTION:          { result = L2CAP_ERR_INSUFF_ENC;            } break;
        case L2CAP_COC_ERR_INVALID_SRC_CID:         { result = L2CAP_ERR_INVALID_CID;           } break;
        case L2CAP_COC_ERR_SRC_CID_ALREADY_ALLOC:   { result = L2CAP_ERR_CID_ALREADY_ALLOC;     } break;
        case L2CAP_COC_ERR_UNACCEPTABLE_PARAM:      { result = L2CAP_ERR_UNACCEPTABLE_PARAM;    } break;
        case L2CAP_COC_ERR_INVALID_PARAM:           { result = GAP_ERR_INVALID_PARAM;           } break;
        default:                                    { result = GAP_ERR_UNEXPECTED;              } break;
    }

    return (result);
}

/**
 ****************************************************************************************
 * @brief Retrieve Host error code from L2CAP COC reconfigure error
 *
 * @param[in] l_err       L2CAP COC reconfigure error code
 *
 * @return Host Error code
 ****************************************************************************************
 */
__STATIC uint16_t l2cap_coc_rcfg_l2h_err(uint8_t h_err)
{
    uint16_t result;
    switch (h_err)
    {
        case L2CAP_COC_RCFG_ERR_SUCCESS:            { result = GAP_ERR_NO_ERROR;                } break;
        case L2CAP_COC_RCFG_ERR_MTU_RED_FORBIDEN:   { result = L2CAP_ERR_INVALID_MTU;           } break;
        case L2CAP_COC_RCFG_ERR_MPS_RED_REJECTED:   { result = L2CAP_ERR_INVALID_MPS;           } break;
        case L2CAP_COC_RCFG_ERR_INVALID_DST_CID:    { result = L2CAP_ERR_INVALID_CID;           } break;
        case L2CAP_COC_RCFG_ERR_UNACCEPTABLE_PARAM: { result = L2CAP_ERR_UNACCEPTABLE_PARAM;    } break;
        default:                                    { result = GAP_ERR_UNEXPECTED;              } break;
    }

    return (result);
}

/**
 ****************************************************************************************
 * @brief Function called when L2CAP COC Creation procedure state is updated
 *
 * @param[in] conidx     Connection index
 * @param[in] p_proc     Pointer to procedure to continue
 * @param[in] proc_state procedure transition state (see enum #l2cap_sig_proc_state)
 * @param[in] status     Execution status
 ****************************************************************************************
 */
__STATIC void l2cap_coc_create_continue(uint8_t conidx, l2cap_coc_proc_create_t* p_proc, uint8_t proc_state, uint16_t status)
{
    bool proc_finished = false;
    uint8_t nb_chan_alloc_max;
    uint8_t nb_chan_alloc;

    switch(proc_state)
    {
        case L2CAP_SIG_PROC_START:
        case L2CAP_SIG_PROC_RSP_RECEIVED:
        {
            l2cap_chan_coc_t* p_chan;
            uint16_t scids[L2CAP_CHAN_NEGO_NB] = { 0, 0, 0, 0, 0 };

            if((status != GAP_ERR_NO_ERROR) || (p_proc->nb_chan_created == p_proc->nb_chan))
            {
                proc_finished = true;
                break;
            }

            nb_chan_alloc_max =  p_proc->enhanced
                              ? co_min(L2CAP_SIMULTANEOUS_CREATE_CHAN_NB, p_proc->nb_chan - p_proc->nb_chan_created)
                              : 1;

            // allocate new channels
            for(nb_chan_alloc = 0 ; nb_chan_alloc < nb_chan_alloc_max ; nb_chan_alloc++)
            {
                uint8_t       chan_lid;

                status = l2cap_chan_dyn_reserve(conidx, &chan_lid, &p_chan);

                if(status != GAP_ERR_NO_ERROR)
                {
                    break;
                }

                // set channel parameters
                p_chan->p_cb          = p_proc->p_cb;
                p_chan->rx_cid        = L2CAP_RXCID_GET(chan_lid);
                p_chan->rx_mtu        = p_proc->mtu;
                p_chan->rx_mps        = p_proc->mps;
                p_chan->rx_credit     = p_proc->credit;
                p_chan->rx_credit_max = p_proc->credit;

                // keep information about register channel
                p_proc->chan_lid[nb_chan_alloc] = chan_lid;
                scids[nb_chan_alloc] = p_chan->rx_cid;
            }

            p_proc->nb_chan_alloc = nb_chan_alloc;

            // nothing allocated. procedure is over
            if(nb_chan_alloc == 0)
            {
                proc_finished = true;
            }
            // Use LECB COC creation
            else
            {
                // allocate a new packet identifier
                p_proc->sig.pkt_id = l2cap_sig_pkt_id_get(conidx);

                if(!p_proc->enhanced)
                {
                    l2cap_sig_lecb_connect_req_t pdu_lecb_connect_req =
                    {
                        .code               = L2CAP_SIG_LECB_CONNECT_REQ_OPCODE,
                        .spsm               = p_proc->spsm,
                        .scid               = p_chan->rx_cid,
                        .mtu                = p_chan->rx_mtu,
                        .mps                = p_chan->rx_mps,
                        .initial_credits    = p_chan->rx_credit,
                    };
                    // send PDU
                    status = l2cap_sig_pdu_send(conidx, 0, p_proc->sig.pkt_id, (l2cap_sig_pdu_t*)&pdu_lecb_connect_req, 0, NULL);
                }
                // Use L2CAP COC creation
                else
                {
                    l2cap_sig_cb_connect_req_t pdu_cb_connect_req =
                    {
                        .code               = L2CAP_SIG_CB_CONNECT_REQ_OPCODE,
                        .spsm               = p_proc->spsm,
                        .mtu                = p_chan->rx_mtu,
                        .mps                = p_chan->rx_mps,
                        .initial_credits    = p_chan->rx_credit,
                    };

                    // send PDU
                    status = l2cap_sig_pdu_send(conidx, 0, p_proc->sig.pkt_id, (l2cap_sig_pdu_t*)&pdu_cb_connect_req, nb_chan_alloc, scids);
                }

                // An error occurs when sending PDU
                if(status != GAP_ERR_NO_ERROR)
                {
                    proc_finished = true;
                    // clean-up allocated channels
                    for(nb_chan_alloc = 0 ; nb_chan_alloc < p_proc->nb_chan_alloc ; nb_chan_alloc++)
                    {
                        l2cap_chan_unregister(conidx, p_proc->chan_lid[nb_chan_alloc]);
                    }
                }
                else
                {
                    l2cap_sig_trans_timer_start(conidx);
                }
            }
        }
        break;

        case L2CAP_SIG_PROC_ERROR:
        {
            // just consider procedure finished
            proc_finished = true;

            if(status != GAP_ERR_DISCONNECTED)
            {
                // clean-up allocated channels
                for(nb_chan_alloc = 0 ; nb_chan_alloc < p_proc->nb_chan_alloc ; nb_chan_alloc++)
                {
                    l2cap_chan_unregister(conidx, p_proc->chan_lid[nb_chan_alloc]);
                }
            }
        } break;

        default: { ASSERT_ERR(0); } break;
    }

    if(proc_finished)
    {
        #if (HOST_MSG_API)
        l2cap_env.dest_task_nbr = p_proc->sig.dest_task_nbr;
        #endif // (HOST_MSG_API)

        // inform that procedure is over
        p_proc->p_cb->cb_coc_create_cmp(conidx, p_proc->dummy, status, p_proc->nb_chan_created);

        // remove procedure execution
        l2cap_sig_proc_pop(conidx, &(p_proc->sig));
    }
}

/**
 ****************************************************************************************
 * @brief Function called when L2CAP COC Connect (Response) procedure state is updated
 *
 * @param[in] conidx   Connection index
 * @param[in] p_proc   Pointer to procedure to continue
 * @param[in] proc_state procedure transition state (see enum #l2cap_sig_proc_state)
 * @param[in] status   Execution status
 ****************************************************************************************
 */
__STATIC void l2cap_coc_connect_continue(uint8_t conidx, l2cap_coc_proc_connect_t* p_proc, uint8_t proc_state, uint16_t status)
{
    bool proc_finished = false;
    bool send_rsp = false;
    uint16_t dcids[L2CAP_CHAN_NEGO_NB] = {0, 0, 0, 0, 0};
    l2cap_chan_coc_t* p_chan;
    uint8_t cursor;

    switch(proc_state)
    {
        case L2CAP_SIG_PROC_START:
        {
            // check if SPSM is registered
            l2cap_spsm_t* p_spsm = l2cap_coc_spsm_get(p_proc->spsm);
            p_proc->nb_chan_accepted = 0;

            send_rsp = true;

            if(p_proc->nb_chan == 0)
            {
                p_proc->status  = L2CAP_COC_ERR_INVALID_PARAM;
                p_proc->nb_chan = 1;
                break;
            }

            if(p_spsm == NULL)
            {
                p_proc->status = L2CAP_COC_ERR_SPSM_NOT_SUPP;
                break;
            }

            // Check encryption is not enabled
            if (!gapc_is_sec_set(conidx, GAPC_LK_ENCRYPTED))
            {
                if (GETF(p_spsm->sec_lvl_bf, L2CAP_COC_AUTH) > L2CAP_SEC_NOT_ENC)
                {
                    // Check LTK availability
                    if(gapc_is_sec_set(conidx, GAPC_LK_BONDED) && gapc_is_sec_set(conidx, GAPC_LK_ENC_KEY_PRESENT))
                    {
                        p_proc->status = L2CAP_COC_ERR_INS_ENCRYPTION;
                        break;
                    }
                    else
                    {
                        p_proc->status = L2CAP_COC_ERR_INS_AUTH;
                        break;
                    }
                }
            }
            // Link encrypted, LTK available
            else if(GETF(p_spsm->sec_lvl_bf, L2CAP_COC_AUTH) > L2CAP_SEC_NOT_ENC)
            {
                // Authentication required
                if ((GETF(p_spsm->sec_lvl_bf, L2CAP_COC_AUTH) > L2CAP_SEC_UNAUTH)
                    && (GETF(p_spsm->sec_lvl_bf, L2CAP_COC_AUTH) > gapc_lk_sec_lvl_get(conidx)))
                {
                    p_proc->status = L2CAP_COC_ERR_INS_AUTH;
                    break;
                }
                // Bigger encryption size key required
                else if (   GETB(p_spsm->sec_lvl_bf, L2CAP_COC_EKS)
                         && (gapc_enc_keysize_get(conidx) < GAP_SEC_ENC_KEY_SIZE))
                {
                    p_proc->status = L2CAP_COC_ERR_INS_EKS;
                    break;
                }
            }

            // Check max MTU / MPS
            if (   ( p_proc->enhanced && ((p_proc->mtu < L2CAP_COC_MTU_MIN)|| (p_proc->mps < L2CAP_COC_MTU_MIN)))
                || (!p_proc->enhanced && ((p_proc->mtu < L2CAP_LE_MTU_MIN) || (p_proc->mps < L2CAP_LE_MTU_MIN))))
            {
                p_proc->status = L2CAP_COC_ERR_UNACCEPTABLE_PARAM;
                break;
            }

            p_proc->nb_chan_alloc    = 0;
            p_proc->cfm_received     = false;

            for(cursor = 0 ; cursor < p_proc->nb_chan ; cursor++)
            {
                uint8_t chan_lid;
                uint8_t scid = p_proc->scids[cursor];

                // check CID
                if (!L2CAP_IS_DYNAMIC_CID(scid))
                {
                    p_proc->status = L2CAP_COC_ERR_INVALID_SRC_CID;
                    p_proc->chan_lid[cursor] = L2CAP_INVALID_CHAN_LID;
                }
                // Check that the Channel Id is not used by another LECB connection
                else if((l2cap_chan_find(conidx, L2CAP_CHAN_CID_PEER, scid, NULL) != L2CAP_INVALID_CHAN_LID))
                {
                    p_proc->status = L2CAP_COC_ERR_SRC_CID_ALREADY_ALLOC;
                    p_proc->chan_lid[cursor] = L2CAP_INVALID_CHAN_LID;
                }
                else
                {
                    // register new channel
                    p_proc->status = l2cap_chan_dyn_reserve(conidx, &chan_lid, &p_chan);

                    if(p_proc->status == GAP_ERR_NO_ERROR)
                    {
                        // set channel parameters known
                        p_chan->tx_cid    = scid;
                        p_chan->tx_mtu    = p_proc->mtu;
                        p_chan->tx_mps    = p_proc->mps;
                        p_chan->tx_credit = p_proc->initial_credits;
                        p_chan->rx_cid    = L2CAP_RXCID_GET(chan_lid);
                        p_chan->rx_mps    = l2cap_env.ll_buf_size - L2CAP_HEADER_LEN;

                        // store channel creation information when confirmation is available
                        p_proc->nb_chan_alloc   += 1;
                        p_proc->chan_lid[cursor] = chan_lid;
                        p_proc->status           = L2CAP_COC_ERR_SUCCESS;
                    }
                    else
                    {
                        p_proc->status           = L2CAP_COC_ERR_INS_RES_AVAIL;
                    }
                }
            }

            // check if some channel allocated
            if(p_proc->nb_chan_alloc != 0)
            {
                send_rsp = false;

                // Inform Upper layer application about channel creation
                p_spsm->p_cb->cb_coc_connect_req(conidx, p_proc->sig.token, p_proc->nb_chan_alloc,
                                                  p_spsm->spsm, p_chan->tx_mtu);
            }
        } break;

        case L2CAP_SIG_PROC_UPPER_LAYER_RSP_RECEIVED:
        {
            uint8_t nb_chan = p_proc->nb_chan_accepted;
            send_rsp = true;

            for(cursor = 0 ; cursor < p_proc->nb_chan ; cursor++)
            {
                uint8_t chan_lid = p_proc->chan_lid[cursor];

                // ignore unknown channel
                if(chan_lid == L2CAP_INVALID_CHAN_LID)
                {
                    continue;
                }

                if(nb_chan > 0)
                {
                    p_chan = l2cap_chan_coc_get(conidx, chan_lid);
                    ASSERT_INFO(p_chan != NULL, conidx, chan_lid);

                    // update channel parameters
                    p_chan->rx_mtu        = p_proc->mtu;
                    p_chan->rx_credit     = p_proc->initial_credits;
                    p_chan->rx_credit_max = p_proc->initial_credits;
                    p_chan->p_cb          = p_proc->p_cb;

                    nb_chan--;

                    // keep destination address
                    dcids[cursor]         = p_chan->rx_cid;
                }
                // channel not used, can be unregistered
                else
                {
                    l2cap_chan_unregister(conidx, chan_lid);
                    p_proc->chan_lid[cursor] = L2CAP_INVALID_CHAN_LID;
                    if(p_proc->status == L2CAP_COC_ERR_SUCCESS)
                    {
                        p_proc->status = L2CAP_COC_ERR_INS_RES_AVAIL; // no other error code available
                    }
                }
            }
        }
        break;

        case L2CAP_SIG_PROC_PDU_PUSHED_TO_LL:
        {
            if(status != GAP_ERR_NO_ERROR) break; // could happen if a disconnection is detected

            for(cursor = 0 ; (cursor < p_proc->nb_chan_accepted) ; cursor++)
            {
                uint8_t chan_lid = p_proc->chan_lid[cursor];

                // ignore unknown channel
                ASSERT_ERR(chan_lid != L2CAP_INVALID_CHAN_LID);

                p_chan = l2cap_chan_coc_get(conidx, chan_lid);
                ASSERT_INFO(p_chan != NULL, conidx, chan_lid);

                // enable channel
                l2cap_chan_enable_set((l2cap_chan_t*)p_chan, true);

                // Inform upper layer that channel is created
                p_proc->p_cb->cb_coc_created(conidx, 0, chan_lid, p_chan->rx_mtu, p_chan->tx_mtu);
            }

            // Consider procedure finished
            proc_finished = true;
        } break;

        default: { ASSERT_ERR(0); } break;
    }

    if(send_rsp)
    {
        // Request rejected
        if(   (status != GAP_ERR_NO_ERROR)      && (status != GAP_ERR_INSUFF_RESOURCES)
           && (status != L2CAP_ERR_INVALID_CID) && (status != L2CAP_ERR_CID_ALREADY_ALLOC))
        {
            p_proc->nb_chan_accepted = 0;
            p_proc->initial_credits  = 0;
            p_proc->mps              = 0;
            p_proc->mtu              = 0;
        }

        // LE COC protocol used
        if(!p_proc->enhanced)
        {
            l2cap_sig_lecb_connect_rsp_t rsp_pdu =
            {
                    .code            = L2CAP_SIG_LECB_CONNECT_RSP_OPCODE,
                    .dcid            = dcids[0],
                    .mtu             = p_proc->mtu,
                    .mps             = p_proc->mps,
                    .initial_credits = p_proc->initial_credits,
                    .result          = p_proc->status,
            };

            // send response
            status = l2cap_sig_pdu_send(conidx, p_proc->sig.token, p_proc->sig.pkt_id, (l2cap_sig_pdu_t*) &rsp_pdu,
                                        0, NULL);
        }
        // Enhance Protocol used
        else
        {
            l2cap_sig_cb_connect_rsp_t rsp_pdu =
            {
                    .code            = L2CAP_SIG_CB_CONNECT_RSP_OPCODE,
                    .mtu             = p_proc->mtu,
                    .mps             = p_proc->mps,
                    .initial_credits = p_proc->initial_credits,
                    .result          = p_proc->status,
            };

            // send response -
            status = l2cap_sig_pdu_send(conidx, p_proc->sig.token, p_proc->sig.pkt_id, (l2cap_sig_pdu_t*) &rsp_pdu,
                                        p_proc->nb_chan, dcids);
        }

        if(status != GAP_ERR_NO_ERROR)
        {
            proc_finished = true;
        }
    }

    if(proc_finished)
    {
        // remove procedure execution
        l2cap_sig_proc_pop(conidx, &(p_proc->sig));
    }
}



/**
 ****************************************************************************************
 * @brief Function called when Termination procedure state is updated
 *
 * @param[in] conidx   Connection index
 * @param[in] p_proc   Pointer to procedure to continue
 * @param[in] proc_state procedure transition state (see enum #l2cap_sig_proc_state)
 * @param[in] status   Execution status
 ****************************************************************************************
 */
__STATIC void l2cap_coc_terminate_continue(uint8_t conidx, l2cap_coc_proc_terminate_t* p_proc, uint8_t proc_state, uint16_t status)
{
    bool proc_finished = false;

    switch(proc_state)
    {
        case L2CAP_SIG_PROC_START:
        {
            l2cap_sig_disconnect_req_t pdu_disconnect_req =
            {
                .code               = L2CAP_SIG_DISCONNECT_REQ_OPCODE,
                .scid               = p_proc->scid,
                .dcid               = p_proc->dcid,
            };

            // allocate a new packet identifier
            p_proc->sig.pkt_id = l2cap_sig_pkt_id_get(conidx);

            // send PDU
            status = l2cap_sig_pdu_send(conidx, 0, p_proc->sig.pkt_id, (l2cap_sig_pdu_t*) &pdu_disconnect_req, 0, NULL);

            // An error occurs when sending PDU
            if(status == GAP_ERR_NO_ERROR)
            {
                l2cap_sig_trans_timer_start(conidx);
            }
            else
            {
                proc_finished = true;
            }
        } break;

        case L2CAP_SIG_PROC_RSP_RECEIVED:
        case L2CAP_SIG_PROC_ERROR:
        {
            // just consider procedure finished
            proc_finished = true;
        } break;

        default: { ASSERT_ERR(0); } break;
    }

    if(proc_finished)
    {
        // anyway unregister channel
        l2cap_chan_unregister(conidx, p_proc->chan_lid);

        #if (HOST_MSG_API)
        l2cap_env.dest_task_nbr = p_proc->sig.dest_task_nbr;
        #endif // (HOST_MSG_API)

        // inform that channel is terminated
        p_proc->p_cb->cb_coc_terminated(conidx, p_proc->dummy, p_proc->chan_lid, p_proc->reason);

        // inform that procedure is over
        if(!p_proc->error_detected)
        {
            p_proc->p_cb->cb_coc_terminate_cmp(conidx, p_proc->dummy, p_proc->chan_lid, status);
        }

        // remove procedure execution
        l2cap_sig_proc_pop(conidx, &(p_proc->sig));
    }
}

/**
 ****************************************************************************************
 * @brief Function called when L2CAP COC Credit Add procedure state is updated
 *
 * @param[in] conidx   Connection index
 * @param[in] p_proc   Pointer to procedure to continue
 * @param[in] proc_state procedure transition state (see enum #l2cap_sig_proc_state)
 * @param[in] status   Execution status
 ****************************************************************************************
 */
__STATIC void l2cap_coc_credit_add_continue(uint8_t conidx, l2cap_coc_proc_credit_add_t* p_proc, uint8_t proc_state, uint16_t status)
{
    switch(proc_state)
    {
        case L2CAP_SIG_PROC_START:
        {
            uint8_t pkt_id;
            l2cap_sig_flow_control_credit_t pdu_credit_add =
            {
                .code               = L2CAP_SIG_FLOW_CONTROL_CREDIT_OPCODE,
                .cid                = p_proc->cid,
                .credits            = p_proc->credits,
            };

            // allocate a new packet identifier
            pkt_id = l2cap_sig_pkt_id_get(conidx);

            // send PDU
            l2cap_sig_pdu_send(conidx, 0, pkt_id, (l2cap_sig_pdu_t*) &pdu_credit_add, 0, NULL);
        }
        break;

        case L2CAP_SIG_PROC_ERROR: { /* Nothing to do */ } break;
        case L2CAP_SIG_PROC_RSP_RECEIVED:
        default: { ASSERT_ERR(0); } break;
    }

    // remove procedure execution
    l2cap_sig_proc_pop(conidx, &(p_proc->sig));
}


/**
 ****************************************************************************************
 * @brief Function called when L2CAP COC Reconfigure procedure (request) state is updated
 *
 * @param[in] conidx   Connection index
 * @param[in] p_proc   Pointer to procedure to continue
 * @param[in] proc_state procedure transition state (see enum #l2cap_sig_proc_state)
 * @param[in] status   Execution status
 ****************************************************************************************
 */
__STATIC void l2cap_coc_reconfigure_req_continue(uint8_t conidx, l2cap_coc_proc_reconfigure_req_t* p_proc, uint8_t proc_state, uint16_t status)
{
    bool proc_finished = false;
    l2cap_chan_coc_t* p_chan;

    switch(proc_state)
    {
        case L2CAP_SIG_PROC_RSP_RECEIVED:
        {
            uint8_t max_cursor = co_min(p_proc->cursor +  p_proc->nb_chan_req, p_proc->nb_chan);
            l2cap_sig_trans_timer_stop(conidx);

            for( ; p_proc->cursor < max_cursor ; p_proc->cursor++)
            {
                uint8_t chan_lid = p_proc->chan_lid[p_proc->cursor];
                p_chan = l2cap_chan_coc_get(conidx, chan_lid);

                // check that channel exists, and is not fixed
                if((p_chan == NULL))
                {
                    status = GAP_ERR_UNEXPECTED;
                }
                else
                {
                    int16_t rx_credit_to_add = (p_proc->new_rx_credit_max - p_chan->rx_credit_max);
                    bool trigger_evt = (p_proc->new_mtu != p_chan->rx_mtu);
                    // update channel parameters
                    p_chan->rx_mps = p_proc->new_mps;
                    p_chan->rx_mtu = p_proc->new_mtu;

                    if(trigger_evt)
                    {
                        // inform that channel MTU changed
                        #if (HOST_MSG_API)
                        l2cap_env.dest_task_nbr = p_chan->dest_task_nbr;
                        #endif // (HOST_MSG_API)
                        p_chan->p_cb->cb_coc_mtu_changed(conidx, p_proc->dummy, chan_lid, p_chan->rx_mtu, p_chan->tx_mtu);
                    }

                    // update maximum number of reception credit
                    p_chan->rx_credit_max = p_proc->new_rx_credit_max;

                    // Add number of RX credits if needed.
                    if(rx_credit_to_add > 0)
                    {
                        l2cap_chan_rx_credit_add(conidx, chan_lid, rx_credit_to_add);
                    }
                }
            }

            // check if procedure is finished
            if((status != GAP_ERR_NO_ERROR) || (p_proc->cursor == p_proc->nb_chan))
            {
                proc_finished = true;
                break;
            }
        }
        // no break

        case L2CAP_SIG_PROC_START:
        {
            uint16_t dcid[L2CAP_CHAN_NEGO_NB] = { 0, 0, 0, 0, 0 };
            uint8_t max_cursor = co_min(p_proc->cursor + L2CAP_CHAN_NEGO_NB, p_proc->nb_chan);
            uint8_t cursor;
            uint8_t dcid_idx;
            l2cap_sig_cb_reconfigure_req_t pdu =
            {
                .code               = L2CAP_SIG_CB_RECONFIGURE_REQ_OPCODE,
                .mtu                = p_proc->new_mtu,
                .mps                = p_proc->new_mps,
            };
            p_proc->nb_chan_req = 0;

            // Retrieve destination identifier
            for(cursor = p_proc->cursor, dcid_idx = 0 ; cursor < max_cursor ; cursor++ , dcid_idx++)
            {
                uint8_t chan_lid = p_proc->chan_lid[cursor];
                p_chan = l2cap_chan_coc_get(conidx, chan_lid);

                if((p_chan == NULL))
                {
                    status = GAP_ERR_UNEXPECTED;
                    break;
                }
                // ensure that only one channel is present when reducing MPS
                else if((p_proc->nb_chan_req > 0) && (p_chan->rx_mps > p_proc->new_mps))
                {
                    break;
                }
                else
                {
                    dcid[dcid_idx] = p_chan->rx_cid;
                    p_proc->nb_chan_req++;

                    // ensure that only one channel is present when reducing MPS
                    if(p_chan->rx_mps > p_proc->new_mps) break;
                }
            }

            if(status == GAP_ERR_NO_ERROR)
            {
                // allocate a new packet identifier
                p_proc->sig.pkt_id = l2cap_sig_pkt_id_get(conidx);

                // send PDU
                status = l2cap_sig_pdu_send(conidx, 0, p_proc->sig.pkt_id, (l2cap_sig_pdu_t*) &pdu,
                                            p_proc->nb_chan_req, dcid);
            }

            // An error occurs when sending PDU
            if(status == GAP_ERR_NO_ERROR)
            {
                l2cap_sig_trans_timer_start(conidx);
            }
            else
            {
                proc_finished = true;
            }
        }
        break;

        case L2CAP_SIG_PROC_ERROR:
        {
            proc_finished = true;
        } break;
        default: { ASSERT_ERR(0); } break;
    }

    if(proc_finished)
    {
        #if (HOST_MSG_API)
        l2cap_env.dest_task_nbr = p_proc->sig.dest_task_nbr;
        #endif // (HOST_MSG_API)

        // inform that procedure is over
        p_proc->p_cb->cb_coc_reconfigure_cmp(conidx, p_proc->dummy, status);

        // remove procedure execution
        l2cap_sig_proc_pop(conidx, &(p_proc->sig));
    }
}

/**
 ****************************************************************************************
 * @brief Function called when L2CAP COC Reconfigure (response) procedure state is updated
 *
 * @param[in] conidx   Connection index
 * @param[in] p_proc   Pointer to procedure to continue
 * @param[in] proc_state procedure transition state (see enum #l2cap_sig_proc_state)
 * @param[in] status   Execution status
 ****************************************************************************************
 */
__STATIC void l2cap_coc_reconfigure_rsp_continue(uint8_t conidx, l2cap_coc_proc_reconfigure_rsp_t* p_proc, uint8_t proc_state,
                                                 uint16_t status)
{
    bool proc_finished = false;
    l2cap_chan_coc_t* p_chan;
    uint8_t cursor;
    uint8_t chan_lid;
    uint16_t result = L2CAP_COC_RCFG_ERR_SUCCESS;

    switch(proc_state)
    {
        case L2CAP_SIG_PROC_START:
        {
            p_proc->tx_flow_off_done     = true;
            p_proc->tx_flow_off_chan_lid = L2CAP_INVALID_CHAN_LID;
            memset(p_proc->chan_lid, L2CAP_INVALID_CHAN_LID, L2CAP_CHAN_NEGO_NB);

            if((p_proc->new_mps < L2CAP_COC_MTU_MIN) || (p_proc->new_mtu < L2CAP_COC_MTU_MIN))
            {
                status = L2CAP_COC_RCFG_ERR_UNACCEPTABLE_PARAM;
            }
            else
            {
                // Check Parameters
                for(cursor = 0 ; cursor < p_proc->nb_chan ; cursor++)
                {
                    if(!L2CAP_IS_DYNAMIC_CID(p_proc->dcids[cursor]))
                    {
                        status = L2CAP_COC_RCFG_ERR_INVALID_DST_CID;
                        break;
                    }

                    chan_lid = l2cap_chan_find(conidx, L2CAP_CHAN_CID_PEER, p_proc->dcids[cursor], (l2cap_chan_t**) &p_chan);

                    // check that channel exists
                    if((chan_lid == L2CAP_INVALID_CHAN_LID) || GETB(p_chan->config_bf, L2CAP_CHAN_FIX))
                    {
                        status = L2CAP_COC_RCFG_ERR_MTU_RED_FORBIDEN;
                        break;
                    }
                    // check that MTU is not reduced
                    else if(p_proc->new_mtu < p_chan->tx_mtu)
                    {
                        status = L2CAP_COC_RCFG_ERR_MTU_RED_FORBIDEN;
                        break;
                    }
                    else if((p_proc->new_mps < p_chan->tx_mps) && (p_proc->nb_chan != 1))
                    {
                        status = L2CAP_COC_RCFG_ERR_MPS_RED_REJECTED;
                        break;
                    }

                    p_proc->chan_lid[cursor] = chan_lid;
                    // check if channel flow must be stopped
                    if(p_chan->tx_mps > p_proc->new_mps)
                    {
                        p_proc->tx_flow_off_chan_lid = chan_lid;
                    }
                }

                // Check if transmission flow must be stopped
                if(p_proc->tx_flow_off_chan_lid != L2CAP_INVALID_CHAN_LID)
                {
                    p_proc->tx_flow_off_done = false;
                    p_proc->in_proc          = true;
                    l2cap_chan_tx_flow_set(p_chan, p_proc->sig.token, false);
                    p_proc->in_proc          = false;
                }
            }
        }
        // no break;

        case L2CAP_SIG_PROC_TX_FLOW_OFF:
        {
            // make an xor onto flow off to be sure that everything is ready
            if(p_proc->tx_flow_off_done)
            {
                l2cap_sig_cb_reconfigure_rsp_t pdu =
                {
                    .code   = L2CAP_SIG_CB_RECONFIGURE_RSP_OPCODE,
                    .result = result,
                };

                // send PDU
                status = l2cap_sig_pdu_send(conidx, p_proc->sig.token, p_proc->sig.pkt_id, (l2cap_sig_pdu_t*) &pdu, 0, NULL);

                if(status != GAP_ERR_NO_ERROR)
                {
                    proc_finished = true;
                }
            }
        } break;

        case L2CAP_SIG_PROC_PDU_PUSHED_TO_LL:
        {
            proc_finished = true;
        } break;
        default: { ASSERT_ERR(0); } break;
    }

    if(proc_finished)
    {
        // Retrieve destination identifier
        for(cursor = 0 ; cursor < L2CAP_CHAN_NEGO_NB ; cursor++)
        {
            chan_lid = p_proc->chan_lid[cursor];
            p_chan = l2cap_chan_coc_get(conidx, chan_lid);

            if(p_chan != NULL)
            {
                bool trigger_evt = (p_proc->new_mtu != p_chan->tx_mtu);

                // update MTU and MPS
                p_chan->tx_mtu = p_proc->new_mtu;
                p_chan->tx_mps = p_proc->new_mps;

                // enable transmissions
                l2cap_chan_tx_flow_set(p_chan, 0, true);

                if(trigger_evt)
                {
                    // inform that channel MTU changed
                    #if (HOST_MSG_API)
                    l2cap_env.dest_task_nbr = p_chan->dest_task_nbr;
                    #endif // (HOST_MSG_API)
                    p_chan->p_cb->cb_coc_mtu_changed(conidx, 0, chan_lid, p_chan->rx_mtu, p_chan->tx_mtu);
                }
            }
        }

        // remove procedure execution
        l2cap_sig_proc_pop(conidx, &(p_proc->sig));
    }
}
/*
 * SIGNALING PACKET HANDLER FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Callback used to handle CONNECTION REQUEST L2CAP Signaling message
 *
 * @param[in] conidx         Connection Index
 * @param[in] pkt_id         Packet identifier
 * @param[in] p_pdu          L2CAP PDU information received
 * @param[in] p_buf          Buffer that contains remaining data (not extracted)
 ****************************************************************************************
 */
void l2cap_sig_lecb_connect_req_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_lecb_connect_req_t* p_pdu, co_buf_t* p_buf)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    l2cap_coc_proc_connect_t* p_proc;

    status = l2cap_sig_proc_create(conidx, L2CAP_SIG_PROC_RSP, L2CAP_SIG_PROC_COC_CONNECT,
                                 (l2cap_sig_proc_cb_t)l2cap_coc_connect_continue,
                                  sizeof(l2cap_coc_proc_connect_t), (l2cap_sig_proc_t**) &(p_proc));

    if(status == GAP_ERR_NO_ERROR)
    {
        // store channel creation information when confirmation is available
        p_proc->enhanced           = false;
        p_proc->sig.pkt_id         = pkt_id;

        p_proc->p_cb               = NULL;
        p_proc->spsm               = p_pdu->spsm;
        p_proc->mtu                = co_min(p_pdu->mtu, GAP_LE_MTU_MAX);
        p_proc->mps                = co_min(p_pdu->mps, GAP_LE_MPS_MAX);
        p_proc->initial_credits    = p_pdu->initial_credits;
        p_proc->scids[0]           = p_pdu->scid;
        p_proc->nb_chan            = 1;

        memset(p_proc->chan_lid, L2CAP_INVALID_CHAN_LID, L2CAP_SIMULTANEOUS_CREATE_CHAN_NB);

        l2cap_sig_proc_push(conidx, (l2cap_sig_proc_t*)p_proc);
    }
    else
    {
        l2cap_sig_lecb_connect_rsp_t rsp_pdu =
        {
            .code = L2CAP_SIG_LECB_CONNECT_RSP_OPCODE,
            .dcid = 0,
            .mtu = 0,
            .mps = 0,
            .initial_credits = 0,
            .result = L2CAP_COC_ERR_INS_RES_AVAIL,
        };

        // Send connection response with error
        l2cap_sig_pdu_send(conidx, 0, pkt_id, (l2cap_sig_pdu_t*) &rsp_pdu, 0, NULL);
    }
}

/**
 ****************************************************************************************
 * @brief Callback used to handle CONNECTION RESPONSE L2CAP Signaling message
 *
 * @param[in] conidx         Connection Index
 * @param[in] pkt_id         Packet identifier
 * @param[in] p_pdu          L2CAP PDU information received
 * @param[in] p_buf          Buffer that contains remaining data (not extracted)
 ****************************************************************************************
 */
void l2cap_sig_lecb_connect_rsp_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_lecb_connect_rsp_t* p_pdu, co_buf_t* p_buf)
{
    l2cap_coc_proc_create_t* p_proc = (l2cap_coc_proc_create_t*) l2cap_sig_proc_pick(conidx, 0);

    if ((p_proc != NULL) && (p_proc->sig.proc_id == L2CAP_SIG_PROC_COC_CREATE) && (p_proc->sig.pkt_id == pkt_id))
    {
        uint16_t status = l2cap_coc_l2h_err(p_pdu->result);

        // stop transaction timer
        l2cap_sig_trans_timer_stop(conidx);

        if (status == GAP_ERR_NO_ERROR)
        {
            // Check MTU and MPS
            if((p_pdu->mtu < L2CAP_LE_MTU_MIN) || (p_pdu->mps < L2CAP_LE_MTU_MIN))
            {
                status = GAP_ERR_INVALID_PARAM;
            }
            // check CID
            else if (!L2CAP_IS_DYNAMIC_CID(p_pdu->dcid))
            {
                status = L2CAP_ERR_INVALID_CID;
            }
            // Check that the Channel Id is not used by another LECB connection
            else if (l2cap_chan_find(conidx, L2CAP_CHAN_CID_PEER, p_pdu->dcid, NULL) != L2CAP_INVALID_CHAN_LID)
            {
                status = L2CAP_ERR_CID_ALREADY_ALLOC;
            }
        }

        if(status != GAP_ERR_NO_ERROR)
        {
            l2cap_chan_unregister(conidx, p_proc->chan_lid[0]);
            p_proc->nb_chan_alloc -= 1;
        }
        else
        {
            // Set peer channel parameters
            l2cap_chan_coc_t* p_chan = l2cap_chan_coc_get(conidx, p_proc->chan_lid[0]);

            p_chan->tx_cid    = p_pdu->dcid;
            p_chan->tx_mtu    = co_min(p_pdu->mtu, GAP_LE_MTU_MAX);
            p_chan->tx_mps    = co_min(p_pdu->mps, GAP_LE_MPS_MAX);
            p_chan->tx_credit = p_pdu->initial_credits;

            // Enable L2CAP channel
            l2cap_chan_enable_set((l2cap_chan_t*)p_chan, true);

            p_proc->nb_chan_created += 1;
            p_proc->nb_chan_alloc   -= 1;

            // Inform host of created channel'

            #if (HOST_MSG_API)
            l2cap_env.dest_task_nbr = p_proc->sig.dest_task_nbr;
            #endif // (HOST_MSG_API)
            p_proc->p_cb->cb_coc_created(conidx, p_proc->dummy, p_proc->chan_lid[0], p_chan->rx_mtu, p_chan->tx_mtu);
        }

        // Continue procedure execution
        l2cap_sig_proc_continue(conidx, 0, L2CAP_SIG_PROC_RSP_RECEIVED, status);
    }
}

/**
 ****************************************************************************************
 * @brief Callback used to handle FLOW CONTROL CREDIT L2CAP Signaling message
 *
 * @param[in] conidx         Connection Index
 * @param[in] pkt_id         Packet identifier
 * @param[in] p_pdu          L2CAP PDU information received
 * @param[in] p_buf          Buffer that contains remaining data (not extracted)
 ****************************************************************************************
 */
void l2cap_sig_flow_control_credit_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_flow_control_credit_t* p_pdu, co_buf_t* p_buf)
{
    // search channel
    l2cap_chan_coc_t* p_chan;
    uint8_t chan_lid = l2cap_chan_find(conidx, L2CAP_CHAN_CID_PEER, p_pdu->cid, (l2cap_chan_t**) &p_chan);

    // if channel is known
    if ((chan_lid != L2CAP_INVALID_CHAN_LID) && !GETB(p_chan->config_bf, L2CAP_CHAN_FIX))
    {
        // increment number of transmission credit
        uint16_t status = l2cap_chan_tx_credit_add(conidx, chan_lid, p_pdu->credits);

        if(status != GAP_ERR_NO_ERROR)
        {
            // Disconnect channel because new credit cannot be added
            l2cap_coc_error_detected(p_chan, status);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Callback used to handle DISCONNECT REQUEST L2CAP Signaling message
 *
 * @param[in] conidx         Connection Index
 * @param[in] pkt_id         Packet identifier
 * @param[in] p_pdu          L2CAP PDU information received
 * @param[in] p_buf          Buffer that contains remaining data (not extracted)
 ****************************************************************************************
 */
void l2cap_sig_disconnect_req_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_disconnect_req_t* p_pdu, co_buf_t* p_buf)
{
    // search channel
    l2cap_chan_coc_t* p_chan;
    uint8_t chan_lid = l2cap_chan_find(conidx, L2CAP_CHAN_CID_LOCAL, p_pdu->dcid, (l2cap_chan_t**)&(p_chan));

    // check if channel is known
    if((chan_lid != L2CAP_INVALID_CHAN_LID) && !GETB(p_chan->config_bf, L2CAP_CHAN_FIX))
    {
        // check that peer PDU is known
        if(p_chan->tx_cid == p_pdu->scid)
        {
            l2cap_sig_disconnect_rsp_t pdu_disconnect_rsp =
                {
                    .code               = L2CAP_SIG_DISCONNECT_RSP_OPCODE,
                    .scid               = p_pdu->scid,
                    .dcid               = p_pdu->dcid,
                };

            const l2cap_chan_coc_cb_t* p_cb = p_chan->p_cb;

            #if (HOST_MSG_API)
            l2cap_env.dest_task_nbr = p_chan->dest_task_nbr;
            #endif // (HOST_MSG_API)

            // Unregister channel
            l2cap_chan_unregister(conidx, chan_lid);

            // send PDU
            l2cap_sig_pdu_send(conidx, 0, pkt_id, (l2cap_sig_pdu_t*) &pdu_disconnect_rsp, 0, NULL);

            if(p_cb != NULL)
            {
                // inform upper layer about COC termination
                p_cb->cb_coc_terminated(conidx, 0, chan_lid, LL_ERR_REMOTE_USER_TERM_CON);
            }
        }
        // else ignore the message
    }
    else
    {
        l2cap_sig_reject_send(conidx, pkt_id, L2CAP_SIG_REJECT_INVALID_CID, p_pdu->dcid, p_pdu->scid);
    }
}

/**
 ****************************************************************************************
 * @brief Callback used to handle DISCONNECT RESPONSE L2CAP Signaling message
 *
 * @param[in] conidx         Connection Index
 * @param[in] pkt_id         Packet identifier
 * @param[in] p_pdu          L2CAP PDU information received
 * @param[in] p_buf          Buffer that contains remaining data (not extracted)
 ****************************************************************************************
 */
void l2cap_sig_disconnect_rsp_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_disconnect_rsp_t* p_pdu, co_buf_t* p_buf)
{
    l2cap_coc_proc_terminate_t* p_proc = (l2cap_coc_proc_terminate_t*) l2cap_sig_proc_pick(conidx, 0);

    if ((p_proc != NULL) && (p_proc->sig.proc_id == L2CAP_SIG_PROC_COC_TERMINATE) && (p_proc->sig.pkt_id == pkt_id))
    {
        uint16_t status = GAP_ERR_NO_ERROR;

        // perform a sanity check of parameter received
        if ((p_proc->scid == p_pdu->scid) && (p_proc->dcid == p_pdu->dcid))
        {
            // stop transaction timer
            l2cap_sig_trans_timer_stop(conidx);
            // Continue procedure execution
            l2cap_sig_proc_continue(conidx, 0, L2CAP_SIG_PROC_RSP_RECEIVED, status);
        }
        // else ignore response
    }
}

void l2cap_sig_cb_connect_req_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_cb_connect_req_t* p_pdu, co_buf_t* p_buf)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    l2cap_coc_proc_connect_t* p_proc;

    status = l2cap_sig_proc_create(conidx, L2CAP_SIG_PROC_RSP, L2CAP_SIG_PROC_COC_CONNECT,
                                 (l2cap_sig_proc_cb_t)l2cap_coc_connect_continue,
                                  sizeof(l2cap_coc_proc_connect_t), (l2cap_sig_proc_t**) &(p_proc));

    if(status == GAP_ERR_NO_ERROR)
    {
        // store channel creation information when confirmation is available
        p_proc->enhanced           = true;
        p_proc->sig.pkt_id         = pkt_id;

        p_proc->p_cb               = NULL;
        p_proc->spsm               = p_pdu->spsm;
        p_proc->mtu                = co_min(p_pdu->mtu, GAP_LE_MTU_MAX);
        p_proc->mps                = co_min(p_pdu->mps, GAP_LE_MPS_MAX);
        p_proc->initial_credits    = p_pdu->initial_credits;
        p_proc->nb_chan            = 0;

        // extract provided channel identifiers
        while((co_buf_data_len(p_buf) >= L2CAP_OPT_LEN) && (p_proc->nb_chan < L2CAP_SIMULTANEOUS_CREATE_CHAN_NB))
        {
            p_proc->scids[p_proc->nb_chan] = co_btohs(co_read16p(co_buf_data(p_buf)));
            co_buf_head_release(p_buf, L2CAP_OPT_LEN);
            p_proc->nb_chan++;
        }

        // ensure that number of channel is between 1 and 5
        if(co_buf_data_len(p_buf) != 0)
        {
            p_proc->nb_chan = 0;
        }

        memset(p_proc->chan_lid, L2CAP_INVALID_CHAN_LID, L2CAP_SIMULTANEOUS_CREATE_CHAN_NB);

        l2cap_sig_proc_push(conidx, (l2cap_sig_proc_t*)p_proc);
    }
    else
    {
        uint16_t dcid = 0;
        l2cap_sig_cb_connect_rsp_t rsp_pdu =
        {
            .code               = L2CAP_SIG_CB_CONNECT_RSP_OPCODE,
            .mtu                = 0,
            .mps                = 0,
            .initial_credits    = 0,
            .result             = L2CAP_COC_ERR_INS_RES_AVAIL,
        };

        // Send connection response with error
        l2cap_sig_pdu_send(conidx, 0, pkt_id, (l2cap_sig_pdu_t*) &rsp_pdu, 1, &(dcid));
    }
}

void l2cap_sig_cb_connect_rsp_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_cb_connect_rsp_t* p_pdu, co_buf_t* p_buf)
{
    l2cap_coc_proc_create_t* p_proc = (l2cap_coc_proc_create_t*) l2cap_sig_proc_pick(conidx, 0);

    if ((p_proc != NULL) && (p_proc->sig.proc_id == L2CAP_SIG_PROC_COC_CREATE) && (p_proc->sig.pkt_id == pkt_id))
    {
        uint16_t status = l2cap_coc_l2h_err(p_pdu->result);
        uint8_t  cursor;
        bool     reject_trans = true;
        uint8_t  nb_chan_alloc;
        uint16_t dcids[L2CAP_SIMULTANEOUS_CREATE_CHAN_NB] = {0,0,0,0,0};

        // stop transaction timer
        l2cap_sig_trans_timer_stop(conidx);

        if(   (status == GAP_ERR_NO_ERROR)      || (status == GAP_ERR_INSUFF_RESOURCES)
           || (status == L2CAP_ERR_INVALID_CID) || (status == L2CAP_ERR_CID_ALREADY_ALLOC))
        {
            // Check MTU and MPS
            if ((p_pdu->mtu >= L2CAP_COC_MTU_MIN) && (p_pdu->mps >= L2CAP_COC_MTU_MIN))
            {
                // Some channel creation accepted
                reject_trans = false;
            }
            // change error code if MTU and MPS parameters are invalid for connection success
            else if(status == GAP_ERR_NO_ERROR)
            {
                status = GAP_ERR_UNEXPECTED;
            }
        }

        // extract provided channel identifiers

        for(cursor = 0 ; (co_buf_data_len(p_buf) >= L2CAP_OPT_LEN) && (cursor < L2CAP_SIMULTANEOUS_CREATE_CHAN_NB) ;
            cursor++)
        {
            dcids[cursor] = co_btohs(co_read16p(co_buf_data(p_buf)));
            co_buf_head_release(p_buf, L2CAP_OPT_LEN);
        }


        // loop on all allocated channels
        nb_chan_alloc =  p_proc->nb_chan_alloc;
        for(cursor = 0 ; cursor < nb_chan_alloc ; cursor++)
        {
            uint8_t chan_lid = p_proc->chan_lid[cursor];
            uint8_t dcid     = dcids[cursor];

            bool reject_chan = (reject_trans || (dcid == 0));

            // check CID
            if (!L2CAP_IS_DYNAMIC_CID(dcid))
            {
                reject_chan = true;
                if(status == GAP_ERR_NO_ERROR)
                {
                    status = L2CAP_ERR_INVALID_CID;
                }
            }
            // Check that the Channel Id is not used by another LECB connection
            else if (l2cap_chan_find(conidx, L2CAP_CHAN_CID_PEER, dcid, NULL) != L2CAP_INVALID_CHAN_LID)
            {
                reject_chan = true;
                if(status == GAP_ERR_NO_ERROR)
                {
                    status = L2CAP_ERR_CID_ALREADY_ALLOC;
                }
            }

            // Channel creation rejected
            if(reject_chan)
            {
                l2cap_chan_unregister(conidx, chan_lid);
                p_proc->nb_chan_alloc   -= 1;
            }
            else
            {
                 // Set peer channel parameters
                l2cap_chan_coc_t* p_chan = l2cap_chan_coc_get(conidx, chan_lid);

                p_chan->tx_cid    = dcid;
                p_chan->tx_mtu    = co_min(p_pdu->mtu, GAP_LE_MTU_MAX);
                p_chan->tx_mps    = co_min(p_pdu->mps, GAP_LE_MPS_MAX);
                p_chan->tx_credit = p_pdu->initial_credits;

                // Enable L2CAP channel
                l2cap_chan_enable_set((l2cap_chan_t*)p_chan, true);

                p_proc->nb_chan_created += 1;
                p_proc->nb_chan_alloc   -= 1;

                // Inform host of created channel'

                #if (HOST_MSG_API)
                l2cap_env.dest_task_nbr = p_proc->sig.dest_task_nbr;
                #endif // (HOST_MSG_API)
                p_proc->p_cb->cb_coc_created(conidx, p_proc->dummy, chan_lid, p_chan->rx_mtu, p_chan->tx_mtu);
            }
        }

        // Continue procedure execution
        l2cap_sig_proc_continue(conidx, 0, L2CAP_SIG_PROC_RSP_RECEIVED, status);
    }
}

void l2cap_sig_cb_reconfigure_req_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_cb_reconfigure_req_t* p_pdu, co_buf_t* p_buf)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    l2cap_coc_proc_reconfigure_rsp_t* p_proc;

    if( (p_pdu->mps >= L2CAP_COC_MTU_MIN) && (p_pdu->mtu >= L2CAP_COC_MTU_MIN))
    {
        status = l2cap_sig_proc_create(conidx, L2CAP_SIG_PROC_RSP, L2CAP_SIG_PROC_COC_RECONFIGURE,
                                     (l2cap_sig_proc_cb_t)l2cap_coc_reconfigure_rsp_continue,
                                      sizeof(l2cap_coc_proc_reconfigure_rsp_t), (l2cap_sig_proc_t**) &(p_proc));
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        // store channel creation information when confirmation is available
        p_proc->sig.pkt_id          = pkt_id;

        p_proc->new_mtu            = co_min(p_pdu->mtu, GAP_LE_MTU_MAX);
        p_proc->new_mps            = co_min(p_pdu->mps, GAP_LE_MPS_MAX);
        p_proc->nb_chan            = 0;

        // extract provided channel identifiers
        while((co_buf_data_len(p_buf) >= L2CAP_OPT_LEN) && (p_proc->nb_chan < L2CAP_SIMULTANEOUS_CREATE_CHAN_NB))
        {
            p_proc->dcids[p_proc->nb_chan] = co_btohs(co_read16p(co_buf_data(p_buf)));
            co_buf_head_release(p_buf, L2CAP_OPT_LEN);
            p_proc->nb_chan++;
        }

        l2cap_sig_proc_push(conidx, (l2cap_sig_proc_t*)p_proc);
    }
    // else let peer device to wait for timeout
}

void l2cap_sig_cb_reconfigure_rsp_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_cb_reconfigure_rsp_t* p_pdu, co_buf_t* p_buf)
{
    l2cap_sig_proc_t* p_proc = (l2cap_sig_proc_t*) l2cap_sig_proc_pick(conidx, 0);

    if ((p_proc != NULL) && (p_proc->pkt_id == pkt_id))
    {
        uint16_t status = l2cap_coc_rcfg_l2h_err(p_pdu->result);

        // stop transaction timer
        l2cap_sig_trans_timer_stop(conidx);

        // Continue procedure execution
        l2cap_sig_proc_continue(conidx, 0, L2CAP_SIG_PROC_RSP_RECEIVED, status);
    }
}

/*
 * INTERNAL FUNCTIONS
 ****************************************************************************************
 */

l2cap_spsm_t* l2cap_coc_spsm_get(uint16_t spsm)
{
    // search if SPSM is present.
    l2cap_spsm_t* p_spsm = (l2cap_spsm_t*) co_list_pick(&(l2cap_env.reg_spsm));

    // browse register spsm list
    while(p_spsm)
    {
        // check if SPSM already registered
        if(p_spsm->spsm == spsm)
        {
            break;
        }

        // go to next element
        p_spsm = (l2cap_spsm_t*) p_spsm->hdr.next;
    }

    return p_spsm;
}

uint16_t l2cap_coc_rx_credit_add(l2cap_chan_coc_t* p_chan, uint16_t credits)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    #if (HOST_MSG_API && RW_DEBUG)
    if(GETB(p_chan->config_bf, L2CAP_CHAN_DBG_MODE_EN))
    {
        l2cap_env.dest_task_nbr = p_chan->dest_task_nbr;
        l2cap_msg_coc_rx_credit_add(p_chan, credits);
    }
    else
    #endif  // (HOST_MSG_API && RW_DEBUG)
    {
        l2cap_coc_proc_credit_add_t* p_credit_add_op;
        status = l2cap_sig_proc_create(p_chan->conidx, L2CAP_SIG_PROC_REQ, L2CAP_SIG_PROC_COC_RX_CREDIT_ADD,
                                     (l2cap_sig_proc_cb_t)l2cap_coc_credit_add_continue,
                                     sizeof(l2cap_coc_proc_credit_add_t), (l2cap_sig_proc_t**) &(p_credit_add_op));

        if(status == GAP_ERR_NO_ERROR)
        {
            // prepare procedure parameters
            p_credit_add_op->chan_lid             = p_chan->chan_lid;
            p_credit_add_op->cid                  = p_chan->rx_cid;
            p_credit_add_op->credits              = credits;

            // push procedure in execution queue
            l2cap_sig_proc_push(p_chan->conidx, &(p_credit_add_op->sig));
        }
    }

    return (status);
}

void l2cap_coc_error_detected(l2cap_chan_coc_t* p_chan, uint16_t status)
{
    uint8_t conidx = p_chan->conidx;
    if(GETB(p_chan->config_bf, L2CAP_CHAN_EN))
    {
        #if (HOST_MSG_API && RW_DEBUG)
        if(GETB(p_chan->config_bf, L2CAP_CHAN_DBG_MODE_EN))
        {
            l2cap_env.dest_task_nbr = p_chan->dest_task_nbr;
            l2cap_msg_coc_error_detected(p_chan, status);
        }
        else
        #endif  // (HOST_MSG_API && RW_DEBUG)
        {
            l2cap_coc_proc_terminate_t* p_terminate_op;

            // stop channel
            l2cap_chan_enable_set((l2cap_chan_t*) p_chan, false);

            if(l2cap_sig_proc_create(conidx, L2CAP_SIG_PROC_REQ, L2CAP_SIG_PROC_COC_TERMINATE,
                                   (l2cap_sig_proc_cb_t)l2cap_coc_terminate_continue,
                                   sizeof(l2cap_coc_proc_terminate_t),
                                   (l2cap_sig_proc_t**) &(p_terminate_op)) == GAP_ERR_NO_ERROR)
            {
                // prepare procedure parameters
                p_terminate_op->dummy               = 0;
                p_terminate_op->p_cb                = p_chan->p_cb;
                p_terminate_op->chan_lid            = p_chan->chan_lid;
                p_terminate_op->reason              = status;
                p_terminate_op->scid                = p_chan->rx_cid;
                p_terminate_op->dcid                = p_chan->tx_cid;
                #if (HOST_MSG_API && RW_DEBUG)
                p_terminate_op->sig.dest_task_nbr   = p_chan->dest_task_nbr;
                #endif  // (HOST_MSG_API && RW_DEBUG)
                p_terminate_op->error_detected      = true;
                // push procedure in execution queue
                l2cap_sig_proc_push(conidx, &(p_terminate_op->sig));
            }
        }
    }
}


void l2cap_coc_tx_flow_off(l2cap_chan_coc_t* p_chan, uint16_t token)
{
    #if (HOST_MSG_API && RW_DEBUG)
    if(GETB(p_chan->config_bf, L2CAP_CHAN_DBG_MODE_EN))
    {
        l2cap_env.dest_task_nbr = p_chan->dest_task_nbr;
        l2cap_msg_coc_tx_flow_off(p_chan);
    }
    else
    #endif  // (HOST_MSG_API && RW_DEBUG)
    {
        l2cap_sig_proc_t* p_proc = l2cap_sig_proc_pick(p_chan->conidx, token);
        if((p_proc != NULL) && (p_proc->proc_id == L2CAP_SIG_PROC_COC_RECONFIGURE))
        {
            l2cap_coc_proc_reconfigure_rsp_t* p_proc_reconfigure = (l2cap_coc_proc_reconfigure_rsp_t*) p_proc;
            p_proc_reconfigure->tx_flow_off_done = true;

            if(!p_proc_reconfigure->in_proc)
            {
                // continue procedure
                l2cap_sig_proc_continue(p_chan->conidx, token, L2CAP_SIG_PROC_TX_FLOW_OFF, GAP_ERR_NO_ERROR);
            }
        }
    }
}


/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t l2cap_coc_spsm_add(uint16_t spsm, uint8_t  sec_lvl_bf, const l2cap_coc_spsm_cb_t* p_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    // check that parameters and callback set is valid
    if((p_cb == NULL) || (p_cb->cb_coc_connect_req == NULL) || !L2CAP_IS_SPSM_VALID(spsm))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else
    {
        l2cap_spsm_t* p_spsm = l2cap_coc_spsm_get(spsm);

        // check that SPSM not already registered
        if(p_spsm == NULL)
        {
            p_spsm = (l2cap_spsm_t*) ke_malloc_user(sizeof(l2cap_spsm_t), KE_MEM_ENV);

            if(p_spsm != NULL)
            {
                p_spsm->spsm          = spsm;
                p_spsm->p_cb          = p_cb;
                p_spsm->sec_lvl_bf    = sec_lvl_bf;

                #if (HOST_MSG_API)
                p_spsm->dest_task_nbr = 0;
                #endif // (HOST_MSG_API)

                // register new SPSM
                co_list_push_back(&(l2cap_env.reg_spsm), &(p_spsm->hdr));

                status = GAP_ERR_NO_ERROR;
            }
            else
            {
                status = GAP_ERR_INSUFF_RESOURCES;
            }
        }
    }

    return (status);
}

uint16_t l2cap_coc_spsm_remove(uint16_t spsm)
{
    uint16_t status = L2CAP_ERR_PSM_SPSM_NOT_SUPP;

    // check that LE-PSM is valid
    if(!L2CAP_IS_SPSM_VALID(spsm))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else
    {
        l2cap_spsm_t* p_spsm = l2cap_coc_spsm_get(spsm);

        // check that SPSM not already registered
        if(p_spsm != NULL)
        {
            // Unregister SPSM
            co_list_extract(&(l2cap_env.reg_spsm), &(p_spsm->hdr));

            // remove structure from memory
            ke_free(p_spsm);
            status = GAP_ERR_NO_ERROR;
        }
    }

    return (status);
}

uint16_t l2cap_coc_create(uint8_t conidx, uint16_t dummy, uint16_t spsm, uint8_t nb_chan, uint16_t local_rx_mtu,
                          const l2cap_chan_coc_cb_t* p_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    l2cap_con_env_t* p_con = l2cap_get_con_env(conidx);

    // ensure that connection is available
    if(p_con != NULL)
    {
        bool enhanced_nego     = GETB(p_con->state_bf, L2CAP_COC_ENHANCED_SUPPORTED) && (local_rx_mtu >= L2CAP_COC_MTU_MIN);
        uint16_t min_mtu = enhanced_nego ? L2CAP_COC_MTU_MIN : L2CAP_LE_MTU_MIN;

        // check provided parameters - all callback must be set
        if(   (p_cb == NULL) || (p_cb->cb_sdu_rx == NULL) || (p_cb->cb_sdu_sent == NULL)
           || (p_cb->cb_coc_create_cmp == NULL) || (p_cb->cb_coc_created == NULL)
           || (p_cb->cb_coc_reconfigure_cmp == NULL) || (p_cb->cb_coc_mtu_changed == NULL)
           || (p_cb->cb_coc_terminate_cmp == NULL) || (p_cb->cb_coc_terminated == NULL)
           // validity of SPSM
            || !L2CAP_IS_SPSM_VALID(spsm)
            // check MTU
            || (local_rx_mtu < min_mtu)
            // At least one channel should be created
            || (nb_chan == 0))
        {
            status = GAP_ERR_INVALID_PARAM;
        }
        else if (local_rx_mtu > GAP_LE_MTU_MAX)
        {
            status = L2CAP_ERR_INVALID_MTU;
        }
        else
        {
            l2cap_coc_proc_create_t* p_create_op;
            status = l2cap_sig_proc_create(conidx, L2CAP_SIG_PROC_REQ, L2CAP_SIG_PROC_COC_CREATE,
                                         (l2cap_sig_proc_cb_t)l2cap_coc_create_continue,
                                         sizeof(l2cap_coc_proc_create_t), (l2cap_sig_proc_t**) &(p_create_op));

            if(status == GAP_ERR_NO_ERROR)
            {
                // prepare procedure parameters
                p_create_op->dummy                  = dummy;
                p_create_op->spsm                   = spsm;
                p_create_op->nb_chan                = nb_chan;
                p_create_op->mtu                    = local_rx_mtu;
                p_create_op->mps                    = l2cap_env.ll_buf_size - L2CAP_HEADER_LEN;
                p_create_op->p_cb                   = p_cb;
                p_create_op->nb_chan_created        = 0;
                p_create_op->nb_chan_alloc          = 0;
                p_create_op->enhanced               = enhanced_nego;
                // Ensure that there is enough credit to receive a complete SDU
                p_create_op->credit                 = CO_DIVIDE_CEIL(local_rx_mtu, p_create_op->mps) +1;

                // push procedure in execution queue
                l2cap_sig_proc_push(conidx, &(p_create_op->sig));
            }
        }
    }

    return (status);
}

uint16_t l2cap_coc_connect_cfm(uint8_t conidx, uint16_t token, uint8_t nb_chan, uint16_t local_rx_mtu,
                               const l2cap_chan_coc_cb_t* p_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    // retrieve response procedure
    l2cap_coc_proc_connect_t* p_proc = (l2cap_coc_proc_connect_t*) l2cap_sig_proc_pick(conidx, token);
    if((p_proc != NULL) && (!p_proc->cfm_received))
    {
        p_proc->cfm_received = true;

        // check provided parameters - all callback must be set
        if(   (p_cb == NULL) || (p_cb->cb_sdu_rx == NULL) || (p_cb->cb_sdu_sent == NULL)
           || (p_cb->cb_coc_created == NULL)
           || (p_cb->cb_coc_reconfigure_cmp == NULL) || (p_cb->cb_coc_mtu_changed == NULL)
           || (p_cb->cb_coc_terminate_cmp == NULL) || (p_cb->cb_coc_terminated == NULL)
            // check MTU according to protocol version)
           || (local_rx_mtu < (p_proc->enhanced ? L2CAP_COC_MTU_MIN : L2CAP_LE_MTU_MIN)))
        {
            status                   = GAP_ERR_INVALID_PARAM;
            p_proc->status           = L2CAP_COC_ERR_INS_AUTHOR;
        }
        else if (local_rx_mtu > GAP_LE_MTU_MAX)
        {
            status = L2CAP_ERR_INVALID_MTU;
            p_proc->status           = L2CAP_COC_ERR_INS_AUTHOR;
        }
        else
        {
            // L2CAP User reject due to insufficient authorizations
            if(nb_chan == L2CAP_COC_NOT_AUTORIZED)
            {
                p_proc->status           = L2CAP_COC_ERR_INS_AUTHOR;
            }
            else
            {
                p_proc->nb_chan_accepted = co_min(nb_chan, p_proc->nb_chan_alloc);
                p_proc->mtu              = local_rx_mtu;
                p_proc->mps              = l2cap_env.ll_buf_size - L2CAP_HEADER_LEN;
                p_proc->initial_credits  = CO_DIVIDE_CEIL(local_rx_mtu, p_proc->mps) +1;
            }

            p_proc->p_cb                 = p_cb;
            status                       = GAP_ERR_NO_ERROR;
        }

        l2cap_sig_proc_continue(conidx, token, L2CAP_SIG_PROC_UPPER_LAYER_RSP_RECEIVED, GAP_ERR_NO_ERROR);
    }

    return (status);
}

uint16_t l2cap_coc_reconfigure(uint8_t conidx, uint16_t dummy, uint16_t local_rx_mtu, uint8_t nb_chan,
                               uint8_t* p_chan_lid)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    l2cap_con_env_t* p_con = l2cap_get_con_env(conidx);

    // ensure that connection is available
    if(p_con != NULL)
    {
        bool enhanced_nego     = GETB(p_con->state_bf, L2CAP_COC_ENHANCED_SUPPORTED);

        // supported only if Enhanced COC creation is supported
        if(!enhanced_nego)
        {
            status = GAP_ERR_NOT_SUPPORTED;
        }
        // Check provided parameters
        else if((local_rx_mtu < L2CAP_COC_MTU_MIN) || (nb_chan == 0))
        {
            status = GAP_ERR_INVALID_PARAM;
        }
        else if (local_rx_mtu > GAP_LE_MTU_MAX)
        {
            status = L2CAP_ERR_INVALID_MTU;
        }
        else
        {
            const l2cap_chan_coc_cb_t*  p_cb = NULL;
            uint8_t cursor;
            status = GAP_ERR_NO_ERROR;
            // perform a sanity check onto channel id provided
            for(cursor = 0 ; cursor < nb_chan ; cursor++)
            {
                l2cap_chan_coc_t* p_chan = l2cap_chan_coc_get(conidx, p_chan_lid[cursor]);

                // check that channel exists, and is not fixed
                if((p_chan == NULL) || (p_chan->rx_mtu > local_rx_mtu))
                {
                    status = GAP_ERR_COMMAND_DISALLOWED;
                    break;
                }

                // return result onto first channel
                if(cursor == 0)
                {
                    p_cb = p_chan->p_cb;
                }
            }

            if(status == GAP_ERR_NO_ERROR)
            {
                l2cap_coc_proc_reconfigure_req_t* p_reconfigure_op;
                status = l2cap_sig_proc_create(conidx, L2CAP_SIG_PROC_REQ, L2CAP_SIG_PROC_COC_RECONFIGURE,
                                             (l2cap_sig_proc_cb_t)l2cap_coc_reconfigure_req_continue,
                                             sizeof(l2cap_coc_proc_reconfigure_req_t) + nb_chan,
                                             (l2cap_sig_proc_t**) &(p_reconfigure_op));

                if(status == GAP_ERR_NO_ERROR)
                {
                    p_reconfigure_op->dummy              = dummy;
                    p_reconfigure_op->new_mtu            = local_rx_mtu;
                    p_reconfigure_op->new_mps            = l2cap_env.ll_buf_size - L2CAP_HEADER_LEN;
                    p_reconfigure_op->cursor             = 0;
                    p_reconfigure_op->nb_chan            = nb_chan;
                    p_reconfigure_op->p_cb               = p_cb;
                    p_reconfigure_op->new_rx_credit_max  = CO_DIVIDE_CEIL(local_rx_mtu, p_reconfigure_op->new_mps) +1;

                    memcpy(p_reconfigure_op->chan_lid, p_chan_lid, nb_chan);

                    // push procedure in execution queue
                    l2cap_sig_proc_push(conidx, &(p_reconfigure_op->sig));
                }
            }
        }
    }

    return (status);
}


#if (RW_DEBUG)
uint16_t l2cap_coc_dbg_reconfigure(uint8_t conidx, uint16_t dummy, uint16_t local_rx_mtu, uint16_t local_rx_mps,
                                   uint8_t nb_chan, uint8_t* p_chan_lid)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    l2cap_con_env_t* p_con = l2cap_get_con_env(conidx);

    // ensure that connection is available
    if(p_con != NULL)
    {
        bool enhanced_nego     = GETB(p_con->state_bf, L2CAP_COC_ENHANCED_SUPPORTED);

        // supported only if Enhanced COC creation is supported
        if(!enhanced_nego)
        {
            status = GAP_ERR_NOT_SUPPORTED;
        }
        // Check provided parameters
        else if((local_rx_mtu < L2CAP_COC_MTU_MIN) || (local_rx_mps < L2CAP_COC_MTU_MIN)|| (nb_chan == 0))
        {
            status = GAP_ERR_INVALID_PARAM;
        }
        else if (local_rx_mtu > GAP_LE_MTU_MAX)
        {
            status = L2CAP_ERR_INVALID_MTU;
        }
        else if (local_rx_mps > GAP_LE_MPS_MAX)
        {
            status = L2CAP_ERR_INVALID_MPS;
        }
        else
        {
            const l2cap_chan_coc_cb_t*  p_cb = NULL;
            uint8_t cursor;
            status = GAP_ERR_NO_ERROR;
            // perform a sanity check onto channel id provided
            for(cursor = 0 ; cursor < nb_chan ; cursor++)
            {
                l2cap_chan_coc_t* p_chan = l2cap_chan_coc_get(conidx, p_chan_lid[cursor]);

                // check that channel exists, and is not fixed
                if((p_chan == NULL) || (p_chan->rx_mtu > local_rx_mtu))
                {
                    status = GAP_ERR_COMMAND_DISALLOWED;
                    break;
                }

                // return result onto first channel
                if(cursor == 0)
                {
                    p_cb = (const l2cap_chan_coc_cb_t*)p_chan->p_cb;
                }
            }

            if(status == GAP_ERR_NO_ERROR)
            {
                l2cap_coc_proc_reconfigure_req_t* p_reconfigure_op;
                status = l2cap_sig_proc_create(conidx, L2CAP_SIG_PROC_REQ, L2CAP_SIG_PROC_COC_RECONFIGURE_DBG,
                                             (l2cap_sig_proc_cb_t)l2cap_coc_reconfigure_req_continue,
                                             sizeof(l2cap_coc_proc_reconfigure_req_t) + nb_chan,
                                             (l2cap_sig_proc_t**) &(p_reconfigure_op));

                if(status == GAP_ERR_NO_ERROR)
                {
                    p_reconfigure_op->dummy              = dummy;
                    p_reconfigure_op->new_mtu            = local_rx_mtu;
                    p_reconfigure_op->new_mps            = local_rx_mps;
                    p_reconfigure_op->cursor             = 0;
                    p_reconfigure_op->nb_chan            = nb_chan;
                    p_reconfigure_op->p_cb               = p_cb;
                    p_reconfigure_op->new_rx_credit_max  = CO_DIVIDE_CEIL(local_rx_mtu, local_rx_mps) +1;

                    memcpy(p_reconfigure_op->chan_lid, p_chan_lid, nb_chan);

                    // push procedure in execution queue
                    l2cap_sig_proc_push(conidx, &(p_reconfigure_op->sig));
                }
            }
        }
    }

    return (status);
}
#endif  // (RW_DEBUG)

uint16_t l2cap_coc_terminate(uint8_t conidx, uint16_t dummy, uint8_t chan_lid)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    l2cap_chan_coc_t* p_chan = l2cap_chan_coc_get(conidx, chan_lid);

    if((p_chan != NULL) && GETB(p_chan->config_bf, L2CAP_CHAN_EN))
    {
        l2cap_coc_proc_terminate_t* p_terminate_op;

        // stop channel
        l2cap_chan_enable_set((l2cap_chan_t*)p_chan, false);

        status = l2cap_sig_proc_create(conidx, L2CAP_SIG_PROC_REQ, L2CAP_SIG_PROC_COC_TERMINATE,
                                     (l2cap_sig_proc_cb_t) l2cap_coc_terminate_continue,
                                     sizeof(l2cap_coc_proc_terminate_t), (l2cap_sig_proc_t**) &(p_terminate_op));

        if(status == GAP_ERR_NO_ERROR)
        {
            // prepare procedure parameters
            p_terminate_op->dummy               = dummy;
            p_terminate_op->p_cb                = p_chan->p_cb;
            p_terminate_op->chan_lid            = chan_lid;
            p_terminate_op->reason              = LL_ERR_CON_TERM_BY_LOCAL_HOST;
            p_terminate_op->scid                = p_chan->rx_cid;
            p_terminate_op->dcid                = p_chan->tx_cid;
            p_terminate_op->error_detected      = false;

            // push procedure in execution queue
            l2cap_sig_proc_push(conidx, &(p_terminate_op->sig));
        }
    }

    return (status);
}


uint16_t l2cap_coc_enhanced_nego_set(uint8_t conidx, uint8_t enable)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    l2cap_con_env_t* p_con = l2cap_get_con_env(conidx);

    if(p_con == NULL)
    {
         status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else
    {
        SETB(p_con->state_bf, L2CAP_COC_ENHANCED_SUPPORTED, enable);
    }

    return (status);
}


#endif // (BLE_L2CAP)
/// @} L2CAP

