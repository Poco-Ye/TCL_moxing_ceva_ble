/**
 ****************************************************************************************
 * @file l2cap_sig.c
 *
 * @brief  L2CAP handling of Signaling Protocol
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
#include "rwip_config.h"        // IP configuration
#if (BLE_L2CAP)
#include "l2cap.h"              // Native API
#include "l2cap_int.h"          // Internals
#include "../inc/l2cap_sig.h"   // Signaling API defines

#include "gapm.h"               // For token id generation

#include "co_utils.h"           // For packing/unpacking
#include "co_endian.h"          // Air and Host number format

#include "ke_mem.h"             // For procedures

#if (HOST_MSG_API)
#include "ke_task.h"            // For task communication
#endif // (HOST_MSG_API)

/*
 * MACROS
 ****************************************************************************************
 */

/// Handler definition of L2CAP packet
#define HANDLER(name, hdl, pack) \
    [name##_OPCODE]   = { (l2cap_sig_handler_func_t) hdl##_handler, pack, name##_LEN }


/*
 * DEFINES
 ****************************************************************************************
 */

/// SIG header length
#define L2CAP_SIG_HEADER_LEN    (4)

/// L2CAP SIG state Bit Field
enum  l2cap_sig_state_bf
{
    /// Continue procedure execution on-going
    L2CAP_SIG_PROC_CONTINUE_EXE_BIT     = 0x01,
    L2CAP_SIG_PROC_CONTINUE_EXE_POS     = 0,

    /// Execute next procedure execution
    L2CAP_SIG_PROC_EXECUTE_NEXT_BIT     = 0x02,
    L2CAP_SIG_PROC_EXECUTE_NEXT_POS     = 1,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Callback used to handle L2CAP Signaling message
 *
 * @param[in] conidx         Connection Index
 * @param[in] pkt_id         Packet identifier
 * @param[in] p_pdu          L2CAP PDU information received
 * @param[in] p_buf          Buffer that contains remaining data (not extracted)
 ****************************************************************************************
 */
typedef void (*l2cap_sig_handler_func_t)(uint8_t conidx, uint8_t pkt_id, l2cap_sig_pdu_t* p_pdu, co_buf_t* p_buf);


/// L2CAP PDU handler information for packing/unpacking and function handler
typedef struct l2cap_sig_handler_info
{
    /// Message handler
    l2cap_sig_handler_func_t    handler;
    /// Pack/Unpack format string
    const char*                 pack_format;
    /// Length of L2CAP PDU
    uint16_t                    length;
} l2cap_sig_handler_info_t;

/*
 * FUNCTION DEFINITION
 ****************************************************************************************
 */
__STATIC void l2cap_sig_default_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_pdu_t* p_pdu, co_buf_t* p_buf);
__STATIC void l2cap_sig_reject_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_reject_t* p_pdu, co_buf_t* p_buf);

// functions present in gapc_con_up.c
extern void l2cap_sig_conn_param_upd_req_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_conn_param_upd_req_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_sig_conn_param_upd_resp_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_conn_param_upd_rsp_t* p_pdu, co_buf_t* p_buf);

// functions present in l2cap_coc.c
extern void l2cap_sig_lecb_connect_req_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_lecb_connect_req_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_sig_lecb_connect_rsp_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_lecb_connect_rsp_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_sig_flow_control_credit_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_flow_control_credit_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_sig_disconnect_req_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_disconnect_req_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_sig_disconnect_rsp_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_disconnect_rsp_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_sig_cb_connect_req_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_cb_connect_req_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_sig_cb_connect_rsp_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_cb_connect_rsp_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_sig_cb_reconfigure_req_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_cb_reconfigure_req_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_sig_cb_reconfigure_rsp_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_cb_reconfigure_rsp_t* p_pdu, co_buf_t* p_buf);

__STATIC void l2cap_sig_sdu_rx_cb (uint8_t conidx, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu);
__STATIC void l2cap_sig_sdu_sent_cb (uint8_t conidx, uint16_t token, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu);

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Signaling packet format
__STATIC const l2cap_sig_handler_info_t l2cap_sig_handler[L2CAP_SIG_OPCODE_MAX] =
{
    // Reserved code
    HANDLER(L2CAP_SIG_RESERVED,                 l2cap_sig_default,               NULL),
    // Reject request
    // [ Pkt_id | Len | reason | opt1 | opt2 ]
    HANDLER(L2CAP_SIG_REJECT,                   l2cap_sig_reject,                "H"),
    // Connection request
    HANDLER(L2CAP_SIG_CONNECTION_REQ,           l2cap_sig_default,               NULL),
    // Connection response
    HANDLER(L2CAP_SIG_CONNECTION_RSP,           l2cap_sig_default,               NULL),
    // Configuration request
    HANDLER(L2CAP_SIG_CONFIGURATION_REQ,        l2cap_sig_default,               NULL),
    // Configuration response
    HANDLER(L2CAP_SIG_CONFIGURATION_RSP,        l2cap_sig_default,               NULL),
    // Disconnection request
    // [ Pkt_id | Len | Destination CID | Source CID ]
    HANDLER(L2CAP_SIG_DISCONNECT_REQ,           l2cap_sig_disconnect_req,        "HH"),
    // Disconnection response
    // [ Pkt_id | Len | Destination CID | Source CID ]
    HANDLER(L2CAP_SIG_DISCONNECT_RSP,           l2cap_sig_disconnect_rsp,        "HH"),
    // Echo request
    HANDLER(L2CAP_SIG_ECHO_REQ,                 l2cap_sig_default,               NULL),
    // Echo response
    HANDLER(L2CAP_SIG_ECHO_RSP,                 l2cap_sig_default,               NULL),
    // Information request
    HANDLER(L2CAP_SIG_INFORMATION_REQ,          l2cap_sig_default,               NULL),
    // Information response
    HANDLER(L2CAP_SIG_INFORMATION_RSP,          l2cap_sig_default,               NULL),
    // Create channel request
    HANDLER(L2CAP_SIG_CREATE_CHANNEL_REQ,       l2cap_sig_default,               NULL),
    // Create channel response
    HANDLER(L2CAP_SIG_CREATE_CHANNEL_RSP,       l2cap_sig_default,               NULL),
    // Move channel request
    HANDLER(L2CAP_SIG_MOVE_CHANNEL_REQ,         l2cap_sig_default,               NULL),
    // Move channel response
    HANDLER(L2CAP_SIG_MOVE_CHANNEL_RSP,         l2cap_sig_default,               NULL),
    // Move channel confirmation
    HANDLER(L2CAP_SIG_MOVE_CHANNEL_CFM,         l2cap_sig_default,               NULL),
    // Move channel confirmation response
    HANDLER(L2CAP_SIG_MOVE_CHANNEL_CFM_RSP,     l2cap_sig_default,               NULL),
    // Connection Parameter Update Request
    // [ Pkt_id | Len | ITV Min | ITV Max | Latency | Timeout ]
    HANDLER(L2CAP_SIG_CONN_PARAM_UPD_REQ,       l2cap_sig_conn_param_upd_req,    "HHHH"),
    // Connection Parameter Update Request
    // [ Pkt_id | Len | reason ]
    HANDLER(L2CAP_SIG_CONN_PARAM_UPD_RSP,       l2cap_sig_conn_param_upd_resp,   "H"),
    // LE Credit Based Connection request
    // [ Pkt_id | Len | SPSM | Source CID | MTU | MPS | Initial Credits ]
    HANDLER(L2CAP_SIG_LECB_CONNECT_REQ,         l2cap_sig_lecb_connect_req,      "HHHHH"),
    // LE Credit Based Connection response
    // [ Pkt_id | Len | Destination CID | MTU | MPS | Initial Credits | Result ]
    HANDLER(L2CAP_SIG_LECB_CONNECT_RSP,         l2cap_sig_lecb_connect_rsp,      "HHHHH"),
    // LE Flow Control Credit
    // [ Pkt_id | Len | Destination CID | Credits ]
    HANDLER(L2CAP_SIG_FLOW_CONTROL_CREDIT,      l2cap_sig_flow_control_credit,   "HH"),
    // L2CAP Credit Based Connection request
    // [ Pkt_id | Len | SPSM | MTU | MPS | Initial Credits | Source CID[1-5] ]
    HANDLER(L2CAP_SIG_CB_CONNECT_REQ,           l2cap_sig_cb_connect_req,        "HHHH"),
    // L2CAP Credit Based Connection response
    // [ Pkt_id | Len | MTU | MPS | Initial Credits | Result | Destination CID[1-5] ]
    HANDLER(L2CAP_SIG_CB_CONNECT_RSP,           l2cap_sig_cb_connect_rsp,        "HHHH"),
    // L2CAP Credit Based Reconfigure request
    // [ Pkt_id | Len | MTU | MPS | Destination CID[1-5] ]
    HANDLER(L2CAP_SIG_CB_RECONFIGURE_REQ,       l2cap_sig_cb_reconfigure_req,    "HH"),
    // L2CAP Credit Based Reconfigure response
    // [ Pkt_id | Len | result ]
    HANDLER(L2CAP_SIG_CB_RECONFIGURE_RSP,       l2cap_sig_cb_reconfigure_rsp,    "H"),
};

/// Signaling channel callback definition
__STATIC const l2cap_chan_cb_t l2cap_sig_chan_cb =
{
        .cb_sdu_rx   = l2cap_sig_sdu_rx_cb,
        .cb_sdu_sent = l2cap_sig_sdu_sent_cb,
};

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Callback used to handle Default L2CAP Signaling message
 *
 * @param[in] conidx         Connection Index
 * @param[in] pkt_id         Packet identifier
 * @param[in] p_pdu          L2CAP PDU information received
 * @param[in] p_buf          Buffer that contains remaining data (not extracted)
 ****************************************************************************************
 */
__STATIC void l2cap_sig_default_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_pdu_t* p_pdu, co_buf_t* p_buf)
{
    // Nothing to do
}

/**
 ****************************************************************************************
 * @brief Callback used to handle L2CAP Signaling Reject message
 *
 * @param[in] conidx         Connection Index
 * @param[in] pkt_id         Packet identifier
 * @param[in] p_pdu          L2CAP PDU information received
 * @param[in] p_buf          Buffer that contains remaining data (not extracted)
 ****************************************************************************************
 */
__STATIC void l2cap_sig_reject_handler(uint8_t conidx, uint8_t pkt_id, l2cap_sig_reject_t* p_pdu, co_buf_t* p_buf)
{
    l2cap_sig_proc_t* p_proc;

//    // Extract optional 1 parameter
//    if(co_buf_data_len(p_buf) > 2)
//    {
//        opt_1 = co_btohs(co_read16p(co_buf_data(p_buf)));
//        co_buf_head_release(p_buf, 2);
//    }
//
//    // Extract optional 2 parameter
//    if(co_buf_data_len(p_buf) > 2)
//    {
//        opt_2 = co_btohs(co_read16p(co_buf_data(p_buf)));
//        co_buf_head_release(p_buf, 2);
//    }

    // retrieve on-going procedure
    p_proc = l2cap_sig_proc_pick(conidx, 0);

    // check if on-going procedure is targetted
    if((p_proc != NULL) && (p_proc->pkt_id == pkt_id))
    {
        uint16_t status;

        // change status code
        switch(p_pdu->reason)
        {
            case L2CAP_SIG_REJECT_CMD_NOT_UNDERSTOOD:  { status = GAP_ERR_NOT_SUPPORTED; } break;
            case L2CAP_SIG_REJECT_MTU_SIG_EXCEEDED:    { status = L2CAP_ERR_INVALID_PDU; } break;
            case L2CAP_SIG_REJECT_INVALID_CID:         { status = L2CAP_ERR_INVALID_CID; } break;
            default:                                   { status = GAP_ERR_UNEXPECTED;    } break;
        }

        // inform on-going procedure that an error occurs
        l2cap_sig_proc_continue(conidx, 0, L2CAP_SIG_PROC_ERROR, status);
    }
}

/**
 ****************************************************************************************
 * @brief The received SDU buffer must be acquired by upper application module before
 *        function return.
 *        When SDU process is done, the corresponding SDU buffer must be release to
 *        allocate new reception credits onto a L2CAP dynamic channel.
 *
 * @param[in] conidx    Connection Index
 * @param[in] chan_lid  Connected L2CAP channel local index
 * @param[in] status    Reception status
 * @param[in] p_sdu     Buffer that contains SDU data
 ****************************************************************************************
 */
__STATIC void l2cap_sig_sdu_rx_cb(uint8_t conidx, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu)
{
    // do nothing if a complete SIG header has not been received
    if(co_buf_data_len(p_sdu) >= L2CAP_SIG_HEADER_LEN)
    {
        // extract header information
        uint8_t  opcode = co_buf_data(p_sdu)[0];
        uint8_t  pkt_id = co_buf_data(p_sdu)[1];
        uint16_t length = co_btohs(co_read16(&co_buf_data(p_sdu)[2]));

        co_buf_head_release(p_sdu, L2CAP_SIG_HEADER_LEN);

        if(   (status == GAP_ERR_NO_ERROR)
           // command can be understood
           && (   (opcode >= L2CAP_SIG_OPCODE_MAX) ||  (l2cap_sig_handler[opcode].pack_format == NULL)
               // check length field received is acceptable
               || (length != co_buf_data_len(p_sdu)) || (length < l2cap_sig_handler[opcode].length))
               )
        {
            status = L2CAP_ERR_INVALID_PDU;
        }

        // No error detected during message reception
        if(status == GAP_ERR_NO_ERROR)
        {
            uint16_t pdu_len = sizeof(l2cap_sig_pdu_t);
            l2cap_sig_pdu_t pdu;

            pdu.code   = opcode;

            // Perform PDU unpacking
            if(co_util_unpack(((uint8_t*) &(pdu)) + 1, co_buf_data(p_sdu), &pdu_len, l2cap_sig_handler[opcode].length,
                              l2cap_sig_handler[opcode].pack_format) == CO_UTIL_PACK_OK)
            {
                co_buf_head_release(p_sdu, l2cap_sig_handler[opcode].length);

                // call message handlr
                l2cap_sig_handler[opcode].handler(conidx, pkt_id, &pdu, p_sdu);
            }
            else
            {
                l2cap_sig_reject_send(conidx, pkt_id, L2CAP_SIG_REJECT_CMD_NOT_UNDERSTOOD, 0, 0);
            }
        }
        // Packet MTU exceeds maximum allowed size.
        else if(status == L2CAP_ERR_INVALID_MTU)
        {
            /* MTU Exceeded  */
            l2cap_sig_reject_send(conidx, pkt_id, L2CAP_SIG_REJECT_MTU_SIG_EXCEEDED, L2CAP_LE_MTU_MIN, 0);
        }
        // PDU received is invalid
        else // L2CAP_ERR_INVALID_PDU
        {
            // PDU invalid, reject command
            if (pkt_id != 0)
            {
                l2cap_sig_reject_send(conidx, pkt_id, L2CAP_SIG_REJECT_CMD_NOT_UNDERSTOOD, 0, 0);
            }
        }
    }
}

/**
 ****************************************************************************************
 * @brief Function called when SDU has been transmitted or if an error occurs
 *
 * @param[in] conidx    Connection Index
 * @param[in] token     Token of the PDU sent, 0 if unused
 * @param[in] chan_lid  L2CAP channel local index
 * @param[in] status    Status of the procedure (see enum #hl_err)
 * @param[in] p_sdu     Pointer to SDU transmitted
 ****************************************************************************************
 */
__STATIC void l2cap_sig_sdu_sent_cb(uint8_t conidx, uint16_t token, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu)
{
    // check if token is relevant
    if(token != 0)
    {
        // inform that the SDU has been properly sent
        l2cap_sig_proc_continue(conidx, token, L2CAP_SIG_PROC_PDU_PUSHED_TO_LL, status);
    }
}


/**
 ****************************************************************************************
 * @brief Callback used to handle L2CAP Transaction timer timeout
 *
 * @param[in] p_hdl  Pointer of timer handle
 * @param[in] conidx Connection index
 ****************************************************************************************
 */
void l2cap_sig_trans_timeout_handler(gapc_sdt_t* p_hdl, uint8_t conidx)
{
    // inform on-going procedure that timer expires
    l2cap_sig_proc_continue(conidx, 0, L2CAP_SIG_PROC_ERROR, GAP_ERR_TIMEOUT);
}

/**
 ****************************************************************************************
 * @brief Callback used to handle start of Signaling request procedure
 *
 * @param[in] p_hdl  Pointer of timer handle
 * @param[in] conidx Connection index
 * @param[in] dummy  Defer job dummy information
 ****************************************************************************************
 */
void l2cap_sig_req_start_handler(gapc_sdt_t* p_hdl, uint8_t conidx, uint16_t dummy)
{
    l2cap_sig_proc_continue(conidx, 0, L2CAP_SIG_PROC_START, GAP_ERR_NO_ERROR);
}

/*
 * INTERNAL FUNCTIONS
 ****************************************************************************************
 */

void l2cap_sig_trans_timer_start(uint8_t conidx)
{
    l2cap_con_env_t* p_con = l2cap_env.p_con[conidx];
    ASSERT_ERR(p_con != NULL);

    gapc_sdt_timer_set(&(p_con->sig.trans_timer), GAP_SIG_TRANS_TIMEOUT_MS);
}

void l2cap_sig_trans_timer_stop(uint8_t conidx)
{
    l2cap_con_env_t* p_con = l2cap_env.p_con[conidx];
    ASSERT_ERR(p_con != NULL);

    gapc_sdt_stop(&(p_con->sig.trans_timer));
}

uint8_t l2cap_sig_pkt_id_get(uint8_t conidx)
{
    l2cap_con_env_t* p_con = l2cap_env.p_con[conidx];
    ASSERT_ERR(p_con != NULL);

    p_con->sig.pkt_id_counter++;
    if(p_con->sig.pkt_id_counter == 0)
    {
        p_con->sig.pkt_id_counter = 1;
    }

    return p_con->sig.pkt_id_counter;
}

uint16_t l2cap_sig_pdu_send(uint8_t conidx, uint16_t token, uint8_t pkt_id, l2cap_sig_pdu_t* p_pdu,
                            uint8_t opt_len, const uint16_t* p_opt)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    l2cap_con_env_t* p_con = l2cap_env.p_con[conidx];
    ASSERT_ERR(p_con != NULL);

    // check if PDU transmission is supported
    if((p_pdu->code < L2CAP_SIG_OPCODE_MAX) && (l2cap_sig_handler[p_pdu->code].pack_format != NULL))
    {
        co_buf_t* p_sdu = NULL;
        uint16_t  sdu_len = l2cap_sig_handler[p_pdu->code].length;

        // allocated buffer - ensure that SIG header and reject optional parameters can be added
        if(co_buf_alloc(&p_sdu, L2CAP_BUFFER_HEADER_LEN + L2CAP_SIG_HEADER_LEN, sdu_len,
                        L2CAP_BUFFER_TAIL_LEN + (L2CAP_CHAN_NEGO_NB * L2CAP_OPT_LEN)) == CO_BUF_ERR_NO_ERROR)
        {
            // Perform PDU packing
            if(co_util_pack(co_buf_data(p_sdu), ((uint8_t*) &(p_pdu->code) + 1), &(sdu_len), sizeof(l2cap_sig_pdu_t),
                            l2cap_sig_handler[p_pdu->code].pack_format) == CO_UTIL_PACK_OK)
            {
                uint8_t cursor;
                for(cursor = 0 ; cursor < opt_len ; cursor++)
                {
                    // copy optional parameter
                    co_write16p(co_buf_tail(p_sdu), co_htobs(p_opt[cursor]));
                    co_buf_tail_reserve(p_sdu, L2CAP_OPT_LEN);
                }

                // Push Length field
                co_buf_head_reserve(p_sdu, L2CAP_LENGTH_LEN);
                co_write16p(co_buf_data(p_sdu), co_htobs(co_buf_data_len(p_sdu) - L2CAP_LENGTH_LEN));

                // Push Packet ID
                co_buf_head_reserve(p_sdu, 1);
                co_buf_data(p_sdu)[0] = pkt_id;

                // Push SIG OPCODE
                co_buf_head_reserve(p_sdu, 1);
                co_buf_data(p_sdu)[0] = p_pdu->code;

                // Send Signaling packet
                status = l2cap_chan_sdu_send(conidx, token, p_con->sig.chan_lid, p_sdu);
            }
            else
            {
                status = GAP_ERR_PROTOCOL_PROBLEM;
            }

            // release buffer
            co_buf_release(p_sdu);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    return (status);
}

void l2cap_sig_proc_continue(uint8_t conidx, uint16_t token, uint8_t proc_state, uint16_t status)
{
    l2cap_sig_proc_t* p_proc;
    l2cap_con_env_t* p_con = l2cap_env.p_con[conidx];
    ASSERT_ERR(p_con != NULL);

    // Continue Request procedure Execution
    if(token == 0)
    {
        while(!co_list_is_empty(&(p_con->sig.req_proc_queue)))
        {
            p_proc = (l2cap_sig_proc_t*) co_list_pick(&(p_con->sig.req_proc_queue));

            SETB(p_con->sig.state_bf, L2CAP_SIG_PROC_CONTINUE_EXE, true);
            p_proc->cb_continue(conidx, p_proc, proc_state, status);
            SETB(p_con->sig.state_bf, L2CAP_SIG_PROC_CONTINUE_EXE, false);

            if(GETB(p_con->sig.state_bf, L2CAP_SIG_PROC_EXECUTE_NEXT))
            {
                // start new procedure
                proc_state = L2CAP_SIG_PROC_START;
                status     = GAP_ERR_NO_ERROR;
                SETB(p_con->sig.state_bf, L2CAP_SIG_PROC_EXECUTE_NEXT, false);
            }
            else
            {
                break;
            }
        }
    }
    // Continue Response procedure Execution
    else
    {
        p_proc = l2cap_sig_proc_pick(conidx, token);
        if(p_proc != NULL)
        {
            p_proc->cb_continue(conidx, p_proc, proc_state, status);
        }
    }
}


uint16_t l2cap_sig_proc_create(uint8_t conidx, uint8_t proc_type, uint8_t proc_code, l2cap_sig_proc_cb_t continue_cb,
                               uint16_t proc_size, l2cap_sig_proc_t** pp_proc)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    if((continue_cb == NULL) || (proc_size < sizeof(l2cap_sig_proc_t)))
    {
        status = GAP_ERR_UNEXPECTED;
    }
    else
    {
        l2cap_sig_proc_t* p_proc = (l2cap_sig_proc_t*) ke_malloc_user(proc_size, KE_MEM_KE_MSG);

        if(p_proc == NULL)
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
        else
        {
            p_proc->proc_id     = proc_code;
            p_proc->cb_continue = continue_cb;

            #if (HOST_MSG_API)
            p_proc->dest_task_nbr = KE_TASK_INVALID;
            #endif // (HOST_MSG_API)

            if(proc_type == L2CAP_SIG_PROC_REQ)
            {
                p_proc->token = 0;
            }
            else
            {
                // generate a specific token
                p_proc->token = gapm_token_id_get();
            }

            *pp_proc = p_proc;
        }
    }

    return(status);
}


void l2cap_sig_proc_push(uint8_t conidx, l2cap_sig_proc_t* p_proc)
{
    l2cap_con_env_t* p_con = l2cap_env.p_con[conidx];
    ASSERT_ERR(p_con != NULL);

    if(p_proc->token == 0)
    {
        bool start_execution = co_list_is_empty(&(p_con->sig.req_proc_queue));

        co_list_push_back(&(p_con->sig.req_proc_queue), &(p_proc->hdr));

        if(start_execution)
        {
            // use transaction timer to defer procedure executionS
            gapc_sdt_defer(&(p_con->sig.trans_timer), 0);
        }
    }
    else
    {
        // put it at beginning of the list because its execution will start immediately
        co_list_push_front(&(p_con->sig.rsp_proc_queue), &(p_proc->hdr));
        l2cap_sig_proc_continue(conidx, p_proc->token, L2CAP_SIG_PROC_START, GAP_ERR_NO_ERROR);
    }
}

void l2cap_sig_proc_pop(uint8_t conidx, l2cap_sig_proc_t* p_proc)
{
    l2cap_con_env_t* p_con = l2cap_env.p_con[conidx];
    ASSERT_ERR(p_con != NULL);

    if(p_proc->token == 0)
    {
        // extract procedure and remove it
        co_list_pop_front(&(p_con->sig.req_proc_queue));
        ke_free(p_proc);

        // stop transaction timer
        l2cap_sig_trans_timer_stop(conidx);

        if(!GETB(p_con->state_bf, L2CAP_DISCONNECTING))
        {
            // check how to execute next procedure
            if(GETB(p_con->sig.state_bf, L2CAP_SIG_PROC_CONTINUE_EXE))
            {
                SETB(p_con->sig.state_bf, L2CAP_SIG_PROC_EXECUTE_NEXT, true);
            }
            else
            {
                l2cap_sig_proc_continue(conidx, 0, L2CAP_SIG_PROC_START, GAP_ERR_NO_ERROR);
            }
        }
    }
    else
    {
        co_list_extract(&(p_con->sig.rsp_proc_queue), &(p_proc->hdr));
        ke_free(p_proc);
    }
}

l2cap_sig_proc_t* l2cap_sig_proc_pick(uint8_t conidx, uint16_t token)
{
    l2cap_sig_proc_t* p_proc = NULL;
    l2cap_con_env_t* p_con = l2cap_env.p_con[conidx];
    ASSERT_ERR(p_con != NULL);

    if(token == 0)
    {
        // pick first element of request procedure queue
        p_proc = (l2cap_sig_proc_t*) co_list_pick(&(p_con->sig.req_proc_queue));
    }
    else
    {
        // search token into response procedure queue
        p_proc = (l2cap_sig_proc_t*) co_list_pick(&(p_con->sig.rsp_proc_queue));
        while(p_proc != NULL)
        {
            if(p_proc->token == token)
            {
                break;
            }

            p_proc = (l2cap_sig_proc_t*) p_proc->hdr.next;
        }
    }

    return (p_proc);
}


uint16_t l2cap_sig_reject_send(uint8_t conidx, uint8_t pkt_id, uint16_t reason, uint16_t opt_1, uint16_t opt_2)
{
    uint16_t opt_par[2];
    uint16_t opt_len = 0;
    l2cap_sig_reject_t pdu_reject =
    {
            .code               = L2CAP_SIG_REJECT_OPCODE,
            .reason             = reason,
    };

    // Fill optional parameters
    switch(reason)
    {
        case L2CAP_SIG_REJECT_MTU_SIG_EXCEEDED:
        {
            opt_par[0] = opt_1;
            opt_len = 1;
        } break;
        case L2CAP_SIG_REJECT_INVALID_CID:
        {
            opt_par[0] = opt_1;
            opt_par[0] = opt_2;
            opt_len = 2;
        } break;
        default: { /* Nothing to do */     } break;
    }

    // send PDU
    return (l2cap_sig_pdu_send(conidx, 0, pkt_id, (l2cap_sig_pdu_t*) &pdu_reject, opt_len, opt_par));
}

void l2cap_sig_create(l2cap_con_env_t* p_con)
{
    // initialize transaction timer
    gapc_sdt_prepare(&(p_con->sig.trans_timer), p_con->conidx, GAPC_SDT_L2CAP);

    // Register signaling channel
    l2cap_chan_fix_register(p_con->conidx, L2CAP_CID_LE_SIGNALING, L2CAP_LE_MTU_MIN, &(l2cap_sig_chan_cb), &(p_con->sig.chan_lid));

    ASSERT_ERR(p_con->sig.chan_lid != L2CAP_INVALID_CHAN_LID);
}

void l2cap_sig_cleanup(l2cap_con_env_t* p_con)
{
    // clean SIG Request queue
    while(!co_list_is_empty(&(p_con->sig.rsp_proc_queue)))
    {
        ke_free(co_list_pop_front(&(p_con->sig.rsp_proc_queue)));
    }

    // clean SIG procedure queue
    while(!co_list_is_empty(&(p_con->sig.req_proc_queue)))
    {
        l2cap_sig_proc_t* p_proc = (l2cap_sig_proc_t*) co_list_pick(&(p_con->sig.req_proc_queue));
        p_proc->cb_continue(p_con->conidx, p_proc, L2CAP_SIG_PROC_ERROR, GAP_ERR_DISCONNECTED);
        // In debug mode, ensure that procedure is properly removed
        ASSERT_ERR(!co_list_find(&(p_con->sig.req_proc_queue), (co_list_hdr_t*) p_proc));
    }

    ASSERT_ERR(p_con->sig.chan_lid != L2CAP_INVALID_CHAN_LID);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */


#endif // (BLE_L2CAP)
/// @} L2CAP

