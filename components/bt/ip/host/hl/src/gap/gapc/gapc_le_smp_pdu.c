/**
 ****************************************************************************************
 *
 * @file gapc_le_smp_pdu.c
 *
 * @brief GAPC LE SMP - PDU management
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#if (BLE_GAPC)

#include "gapc_int.h"
#include "gapc_le_smp.h"


/*
 * MACROS
 ****************************************************************************************
 */
/// Handler definition of L2CAP packet
#define HANDLER(name, hdl, pack) \
    [name##_OPCODE]   = { (gapc_le_smp_handler_func_t) hdl##_handler, pack, name##_LEN }


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Callback used to handle L2CAP SMP message
 *
 * @param[in] conidx         Connection Index
 * @param[in] p_pdu          L2CAP PDU information received
 * @param[in] p_buf          Buffer that contains remaining data (not extracted)
 *
 * @return Execution status of the message handler
 ****************************************************************************************
 */
typedef void (*gapc_le_smp_handler_func_t)(gapc_le_con_t* p_con, l2cap_smp_pdu_t* p_pdu, co_buf_t* p_buf);


/// L2CAP PDU handler information for packing/unpacking and function handler
typedef struct gapc_le_smp_handler_info
{
    /// Message handler
    gapc_le_smp_handler_func_t handler;
    /// Pack/Unpack format string
    const char*                pack_format;
    /// Length of L2CAP PDU
    uint16_t                   length;
} gapc_le_smp_handler_info_t;



/*
 * L2CAP SMP PDU MESSAGE HANDERS - DECLARATIONS
 ****************************************************************************************
 */

__STATIC void l2cap_smp_default_handler(gapc_le_con_t* p_con, l2cap_smp_pdu_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_smp_pairing_req_handler(gapc_le_con_t* p_con, l2cap_smp_pairing_req_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_smp_pairing_rsp_handler(gapc_le_con_t* p_con, l2cap_smp_pairing_rsp_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_smp_pairing_confirm_handler(gapc_le_con_t* p_con, l2cap_smp_pairing_cfm_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_smp_pairing_random_handler(gapc_le_con_t* p_con, l2cap_smp_pairing_random_t* p_pdu, co_buf_t* p_buf);
__STATIC void l2cap_smp_pairing_failed_handler(gapc_le_con_t* p_con, l2cap_smp_pairing_failed_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_smp_encryption_inf_handler(gapc_le_con_t* p_con, l2cap_smp_encryption_inf_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_smp_master_id_handler(gapc_le_con_t* p_con, l2cap_smp_master_id_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_smp_identity_inf_handler(gapc_le_con_t* p_con, l2cap_smp_identity_inf_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_smp_id_addr_inf_handler(gapc_le_con_t* p_con, l2cap_smp_id_addr_inf_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_smp_signing_inf_handler(gapc_le_con_t* p_con, l2cap_smp_signing_inf_t* p_pdu, co_buf_t* p_buf);
__STATIC void l2cap_smp_security_req_handler(gapc_le_con_t* p_con, l2cap_smp_security_req_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_smp_public_key_handler(gapc_le_con_t* p_con, l2cap_smp_public_key_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_smp_dhkey_check_handler(gapc_le_con_t* p_con, l2cap_smp_dhkey_check_t* p_pdu, co_buf_t* p_buf);
extern void l2cap_smp_keypress_ntf_handler(gapc_le_con_t* p_con, l2cap_smp_keypress_ntf_t* p_pdu, co_buf_t* p_buf);



__STATIC void gapc_le_smp_sdu_rx_cb(uint8_t conidx, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu);
__STATIC void gapc_le_smp_sdu_sent_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu);
__STATIC uint16_t gapc_le_smp_pdu_send(uint8_t conidx, l2cap_smp_pdu_t *p_pdu);
__STATIC uint16_t gapc_le_smp_pdu_send_in_transaction(gapc_le_smp_pair_proc_t* p_proc, l2cap_smp_pdu_t *p_pdu);


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Security packet format
__STATIC const gapc_le_smp_handler_info_t l2cap_smp_handler[L2CAP_SMP_OPCODE_MAX] =
{
    // Reserved code
    HANDLER(L2CAP_SMP_RESERVED,         l2cap_smp_default,           NULL),
    // Pairing Request
    // [iocap | oob | auth | max Key | IKeyX | RKeyX]
    HANDLER(L2CAP_SMP_PAIRING_REQ,      l2cap_smp_pairing_req,      "BBBBBB"),
    // Pairing Response
    // [iocap | oob | auth | max Key | IKeyX | RKeyX]
    HANDLER(L2CAP_SMP_PAIRING_RSP,      l2cap_smp_pairing_rsp,      "BBBBBB"),
    // Pairing Confirm
    // [confirm]
    HANDLER(L2CAP_SMP_PAIRING_CFM,      l2cap_smp_pairing_confirm,  "16B"),
    // Pairing Random
    // [random]
    HANDLER(L2CAP_SMP_PAIRING_RANDOM,   l2cap_smp_pairing_random,   "16B"),
    // Pairing Failed
    // [reason]
    HANDLER(L2CAP_SMP_PAIRING_FAILED,   l2cap_smp_pairing_failed,   "B"),
    // Encryption Information
    // [LTK]
    HANDLER(L2CAP_SMP_ENCRYPTION_INF,   l2cap_smp_encryption_inf,   "16B"),
    // Master Identification
    // [ediv | rand]
    HANDLER(L2CAP_SMP_MASTER_ID,        l2cap_smp_master_id,        "H8B"),
    // Identity Information
    // [IRK]
    HANDLER(L2CAP_SMP_IDENTITY_INF,     l2cap_smp_identity_inf,     "16B"),
    // Identity Address Information
    // [Addr_type | Addr]
    HANDLER(L2CAP_SMP_ID_ADDR_INF,      l2cap_smp_id_addr_inf,      "B6B"),
    // Signing Information
    // [CSRK]
    HANDLER(L2CAP_SMP_SIGNING_INF,      l2cap_smp_signing_inf,      "16B"),
    // Security Request
    // [auth]
    HANDLER(L2CAP_SMP_SECURITY_REQ,     l2cap_smp_security_req,     "B"),
    // Pairing Public Key
    // [public key X | public key Y]
    HANDLER(L2CAP_SMP_PUBLIC_KEY,       l2cap_smp_public_key,       "32B32B"),
    // DHkey Check
    // [DHkey]
    HANDLER(L2CAP_SMP_DHKEY_CHECK,      l2cap_smp_dhkey_check,      "16B"),
    // Key Press Notification
    // [Notification Type]
    HANDLER(L2CAP_SMP_KEYPRESS_NTF,     l2cap_smp_keypress_ntf,     "B"),
};

/// Signaling channel callback definition
__STATIC const l2cap_chan_cb_t gapc_le_smp_chan_cb =
{
        .cb_sdu_rx   = gapc_le_smp_sdu_rx_cb,
        .cb_sdu_sent = gapc_le_smp_sdu_sent_cb,
};

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/// @brief Callback used to handle UNKNOWN PDU
__STATIC void l2cap_smp_default_handler(gapc_le_con_t* p_con, l2cap_smp_pdu_t* p_pdu, co_buf_t* p_buf)
{
    // Nothing to do
}


/// @brief Callback used to handle PAIRING FAILED PDU
__STATIC void l2cap_smp_pairing_failed_handler(gapc_le_con_t* p_con, l2cap_smp_pairing_failed_t* p_pdu, co_buf_t* p_buf)
{
    uint8_t smp_reason = p_pdu->reason;
    uint16_t reason;
    gapc_le_smp_pair_proc_t *p_proc = gapc_le_smp_get_pairing_proc(p_con->hdr.conidx);

    if (   (smp_reason < L2CAP_SMP_ERR_PASSKEY_ENTRY_FAILED)
        || (smp_reason > L2CAP_SMP_ERR_CROSS_TRANSPORT_KEY_GENERATION_NOT_ALLOWED))
    {
        // change the reason code because outside of known error codes.
        smp_reason = L2CAP_SMP_ERR_UNSPECIFIED_REASON;
    }

    reason = GAPC_LE_SMP_ERR_TO_HL_ERR(REM, smp_reason);

    // Check if a pairing procedure is in progress
    if (p_proc != NULL)
    {
        // Inform the HL that an error has occurred.
        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PAIRING_ABORTED, reason);
    }
    else
    {
        gapc_le_smp_indicate_pairing_failed(p_con, reason);
    }
}

/// @brief Callback used to handle SECURITY REQUEST PDU
__STATIC void l2cap_smp_security_req_handler(gapc_le_con_t* p_con, l2cap_smp_security_req_t* p_pdu, co_buf_t* p_buf)
{
    #if (HL_LE_CENTRAL)
    if (!gapc_le_smp_is_pairing_ongoing(p_con) && (GETB(p_con->hdr.info_bf, GAPC_ROLE) == ROLE_MASTER))
    {
        uint8_t smp_status = L2CAP_SMP_ERR_NO_ERROR;
        if (!gapc_le_smp_is_auth_valid(p_pdu->auth))
        {
            smp_status = L2CAP_SMP_ERR_INVALID_PARAM;
        }
        // Check the repeated attempts status
        else if (gapc_le_smp_check_repeated_attempts(p_con) == GAPC_LE_SMP_NO_REP_ATTEMPTS)
        {
            if(((p_pdu->auth & GAP_AUTH_SEC_CON) == 0) && !gapm_is_legacy_pairing_supp())
            {
                smp_status = L2CAP_SMP_ERR_AUTH_REQ;

                // indicate slave ask for a pairing that cannot be achieved
                gapc_le_smp_indicate_pairing_failed(p_con, GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, L2CAP_SMP_ERR_AUTH_REQ));
            }
            // Inform that slave request pairing or encryption with a specific pairing level
            else  if(gapc_env.p_sec_cbs->auth_req != NULL)
            {
                 gapc_env.p_sec_cbs->auth_req(p_con->hdr.conidx, p_con->hdr.dummy, p_pdu->auth & GAP_AUTH_REQ_MASK);
            }
        }
        // else ignore the request

        if(smp_status != L2CAP_SMP_ERR_NO_ERROR)
        {
            gapc_le_smp_send_pairing_fail_pdu(p_con, smp_status);
        }
    }

    // else ignore the request
    #endif // (HL_LE_CENTRAL)
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
__STATIC void gapc_le_smp_sdu_rx_cb(uint8_t conidx, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu)
{
    // do nothing if a complete SIG header has not been received
    if((status == GAP_ERR_NO_ERROR) && (co_buf_data_len(p_sdu) >= L2CAP_SMP_HEADER_LEN))
    {
        // extract header information
        uint8_t  opcode = co_buf_data(p_sdu)[0];
        co_buf_head_release(p_sdu, L2CAP_SMP_HEADER_LEN);

        // command can be understood
        if((opcode < L2CAP_SMP_OPCODE_MAX) && (l2cap_smp_handler[opcode].pack_format != NULL)
            // check length received is acceptable
            && (co_buf_data_len(p_sdu) >= l2cap_smp_handler[opcode].length))
        {
            uint16_t pdu_len = sizeof(l2cap_smp_pdu_t);
            l2cap_smp_pdu_t pdu;

            pdu.code   = opcode;

            // Perform PDU unpacking
            if(co_util_unpack(((uint8_t*) &(pdu)) + 1, co_buf_data(p_sdu), &pdu_len, l2cap_smp_handler[opcode].length,
                              l2cap_smp_handler[opcode].pack_format) == CO_UTIL_PACK_OK)
            {
                co_buf_head_release(p_sdu, l2cap_smp_handler[opcode].length);

                // call message handler
                l2cap_smp_handler[opcode].handler(gapc_le_con_env_get(conidx), &pdu, p_sdu);
            }
        }
    }
}

/// @brief Function called when SDU has been transmitted or if an error occurs
__STATIC void gapc_le_smp_sdu_sent_cb(uint8_t conidx, uint16_t dummy, uint8_t chan_lid, uint16_t status, co_buf_t* p_sdu)
{
    // ignore
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

__STATIC uint16_t gapc_le_smp_pdu_send(uint8_t conidx, l2cap_smp_pdu_t *p_pdu)
{
    uint8_t status = GAP_ERR_INVALID_PARAM;
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

    ASSERT_ERR((p_pdu->code < L2CAP_SMP_OPCODE_MAX) && (l2cap_smp_handler[p_pdu->code].length != 0));

    // check if PDU transmission is supported
    if((p_pdu->code < L2CAP_SMP_OPCODE_MAX) && (l2cap_smp_handler[p_pdu->code].length != 0))
    {
        co_buf_t* p_sdu = NULL;
        uint16_t  sdu_len = l2cap_smp_handler[p_pdu->code].length;

        // allocated buffer - ensure that SIG header and reject optional parameters can be added
        if(co_buf_alloc(&p_sdu, L2CAP_BUFFER_HEADER_LEN + L2CAP_SMP_HEADER_LEN, sdu_len,
                        L2CAP_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
        {
            // Perform PDU packing
            if(co_util_pack(co_buf_data(p_sdu), ((uint8_t*) &(p_pdu->code) + 1), &(sdu_len), sizeof(l2cap_smp_pdu_t),
                            l2cap_smp_handler[p_pdu->code].pack_format) == CO_UTIL_PACK_OK)
            {
                // Push SMP OPCODE
                co_buf_head_reserve(p_sdu, 1);
                co_buf_data(p_sdu)[0] = p_pdu->code;

                // Send SMP PDU packet
                status = l2cap_chan_sdu_send(conidx, 0, p_con->smp.chan_lid, p_sdu);
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

/// Send PDU and start transaction timer
__STATIC uint16_t gapc_le_smp_pdu_send_in_transaction(gapc_le_smp_pair_proc_t* p_proc, l2cap_smp_pdu_t *p_pdu)
{
    // restart transaction timer
    gapc_le_smp_start_trans_timer(&(p_proc->hdr));

    // Send PDU
    return gapc_le_smp_pdu_send(p_proc->hdr.conidx, p_pdu);
}


/// @brief Construct and send pairing confirm PDU.
uint16_t gapc_le_smp_send_pairing_confirm_pdu(gapc_le_smp_pair_proc_t* p_proc)
{
    l2cap_smp_pairing_cfm_t pdu =
    {
            .code   = L2CAP_SMP_PAIRING_CFM_OPCODE,
    };

    // Pack the confirm value - 16 bytes (Shall be given LSB->MSB)
    memcpy(&(pdu.cfm_val[0]), p_proc->local_confirm, CFM_LEN);

    return gapc_le_smp_pdu_send_in_transaction(p_proc, (l2cap_smp_pdu_t*) &pdu);
}

/// @brief Construct and send pairing random PDU.
uint16_t gapc_le_smp_send_pair_rand_pdu(gapc_le_smp_pair_proc_t* p_proc)
{
    l2cap_smp_pairing_random_t pdu =
    {
            .code   = L2CAP_SMP_PAIRING_RANDOM_OPCODE,
    };

    // Pack the random number value - 16 bytes (Shall be given LSB->MSB)
    memcpy(&(pdu.random[0]), p_proc->local_rand, RAND_VAL_LEN);

    return gapc_le_smp_pdu_send_in_transaction(p_proc, (l2cap_smp_pdu_t*) &pdu);
}

/// @brief Construct pairing failed PDU and send it
uint16_t gapc_le_smp_send_pairing_fail_pdu(gapc_le_con_t* p_con, uint8_t reason)
{
    l2cap_smp_pairing_failed_t pdu =
    {
            .code   = L2CAP_SMP_PAIRING_FAILED_OPCODE,
            .reason = reason,
    };

    return gapc_le_smp_pdu_send(p_con->hdr.conidx, (l2cap_smp_pdu_t*) &pdu);
}

/// @brief Construct security request PDU and send it
uint16_t gapc_le_smp_send_security_req_pdu(gapc_le_con_t* p_con, uint8_t auth)
{
    l2cap_smp_security_req_t pdu =
    {
            .code   = L2CAP_SMP_SECURITY_REQ_OPCODE,
            .auth   = auth,
    };

    return gapc_le_smp_pdu_send(p_con->hdr.conidx, (l2cap_smp_pdu_t*) &pdu);
}

/// @brief Send pairing failed and start repeated attempt timer
uint16_t gapc_le_smp_send_pairing_fail_pdu_and_start_ra_timer(gapc_le_smp_pair_proc_t* p_proc, uint8_t reason)
{
    gapc_le_con_t* p_con = gapc_le_con_env_get(p_proc->hdr.conidx);
    gapc_le_smp_start_rep_attempt_timer(p_con);

    return gapc_le_smp_send_pairing_fail_pdu(p_con, reason);
}

/// @brief Construct identification information PDU.
uint16_t gapc_le_smp_send_id_info_pdu(gapc_le_smp_pair_proc_t* p_proc)
{
    l2cap_smp_identity_inf_t pdu =
    {
            .code   = L2CAP_SMP_IDENTITY_INF_OPCODE,
    };

    // Pack IRK - 16 bytes (Shall be given LSB->MSB)
    memcpy(&(pdu.irk[0]), &(p_proc->hdr.pairing_keys.irk.key), GAP_KEY_LEN);

    return gapc_le_smp_pdu_send_in_transaction(p_proc, (l2cap_smp_pdu_t*) &pdu);
}

/// @brief Construct identification address information PDU.
uint16_t gapc_le_smp_send_id_addr_info_pdu(gapc_le_smp_pair_proc_t* p_proc)
{
    gap_bdaddr_t identity;

    l2cap_smp_id_addr_inf_t pdu =
    {
        .code   = L2CAP_SMP_ID_ADDR_INF_OPCODE,
    };

    // retrieve address information
    gapm_get_identity(&identity);
    pdu.addr_type = identity.addr_type;
    memcpy(&(pdu.addr), identity.addr, GAP_BD_ADDR_LEN);

   return gapc_le_smp_pdu_send_in_transaction(p_proc, (l2cap_smp_pdu_t*) &pdu);
}

/// @brief Construct LTK PDU.
uint16_t gapc_le_smp_send_enc_info_pdu(gapc_le_smp_pair_proc_t* p_proc)
{
    l2cap_smp_encryption_inf_t pdu =
    {
            .code   = L2CAP_SMP_ENCRYPTION_INF_OPCODE,
    };

    // Pack LTK - 16 bytes (Shall be given LSB->MSB)
    memcpy(&(pdu.ltk[0]), &(p_proc->hdr.pairing_keys.ltk.key), GAP_KEY_LEN);

    return gapc_le_smp_pdu_send_in_transaction(p_proc, (l2cap_smp_pdu_t*) &pdu);
}


/// @brief Construct Master ID  PDU.
uint16_t gapc_le_smp_send_master_id_pdu(gapc_le_smp_pair_proc_t* p_proc)
{
    l2cap_smp_master_id_t pdu =
    {
            .code      = L2CAP_SMP_MASTER_ID_OPCODE,
            .ediv      = p_proc->hdr.pairing_keys.ltk.ediv,
    };

    // Pack random number value - 8 bytes (Shall be given LSB->MSB)
    memcpy(&(pdu.nb), &(p_proc->hdr.pairing_keys.ltk.randnb), GAP_RAND_NB_LEN);

    return gapc_le_smp_pdu_send_in_transaction(p_proc, (l2cap_smp_pdu_t*) &pdu);
}

/// @brief Construct CSRK PDU.
uint16_t gapc_le_smp_send_signing_info_pdu(gapc_le_smp_pair_proc_t* p_proc)
{
    l2cap_smp_signing_inf_t pdu =
    {
            .code   = L2CAP_SMP_SIGNING_INF_OPCODE,
    };

    // Pack CSRK - 16 bytes (Shall be given LSB->MSB)
    memcpy(&(pdu.csrk[0]), &(p_proc->hdr.pairing_keys.csrk), GAP_KEY_LEN);

    return gapc_le_smp_pdu_send_in_transaction(p_proc, (l2cap_smp_pdu_t*) &pdu);
}

#if(HL_LE_CENTRAL)
/// Send Pairing Request
uint16_t gapc_le_smp_send_pairing_req_pdu(gapc_le_smp_pair_proc_t* p_proc)
{
    gapc_pairing_t* p_pair_req_feat = &(p_proc->pair_req_pdu);
    l2cap_smp_pairing_req_t pdu =
    {
            .code      = L2CAP_SMP_PAIRING_REQ_OPCODE,
            .iocap     = p_pair_req_feat->iocap,
            .oob       = p_pair_req_feat->oob,
            .auth      = p_pair_req_feat->auth,
            .key_size  = p_pair_req_feat->key_size,
            .ikey_dist = p_pair_req_feat->ikey_dist,
            .rkey_dist = p_pair_req_feat->rkey_dist,
    };

    return gapc_le_smp_pdu_send_in_transaction(p_proc, (l2cap_smp_pdu_t*) &pdu);
}
#endif // (HL_LE_CENTRAL)

#if(HL_LE_PERIPHERAL)
/// Send Pairing response
uint16_t gapc_le_smp_send_pairing_rsp_pdu(gapc_le_smp_pair_proc_t* p_proc)
{
    gapc_pairing_t* p_pair_req_feat = &(p_proc->pair_rsp_pdu);
    l2cap_smp_pairing_rsp_t pdu =
    {
            .code      = L2CAP_SMP_PAIRING_RSP_OPCODE,
            .iocap     = p_pair_req_feat->iocap,
            .oob       = p_pair_req_feat->oob,
            .auth      = p_pair_req_feat->auth,
            .key_size  = p_pair_req_feat->key_size,
            .ikey_dist = p_pair_req_feat->ikey_dist,
            .rkey_dist = p_pair_req_feat->rkey_dist,
    };

    return gapc_le_smp_pdu_send_in_transaction(p_proc, (l2cap_smp_pdu_t*) &pdu);
}
#endif // (HL_LE_PERIPHERAL)

/// Send Pairing response
uint16_t gapc_le_smp_send_public_key_pdu(gapc_le_smp_pair_proc_t* p_proc)
{
    l2cap_smp_public_key_t pdu =
    {
            .code   = L2CAP_SMP_PUBLIC_KEY_OPCODE,
    };

    // Note - the Public Key is transmitted Least Significant Octet First
    memcpy(pdu.x, p_proc->local_public_key.x, GAP_P256_KEY_LEN);
    memcpy(pdu.y, p_proc->local_public_key.y, GAP_P256_KEY_LEN);

    return gapc_le_smp_pdu_send_in_transaction(p_proc, (l2cap_smp_pdu_t*) &pdu);
}


/// @brief Construct DH Key PDU.
uint16_t gapc_le_smp_send_dhkey_check_pdu(gapc_le_smp_pair_proc_t* p_proc)
{
    l2cap_smp_dhkey_check_t pdu =
    {
            .code   = L2CAP_SMP_DHKEY_CHECK_OPCODE,
    };

    // Pack DH Key - 16 bytes
    memcpy(pdu.check, p_proc->local_confirm, DHKEY_CHECK_LEN);

    return gapc_le_smp_pdu_send_in_transaction(p_proc, (l2cap_smp_pdu_t*) &pdu);
}

/// @brief Construct Key Pressed notification PDU and send it
uint16_t gapc_le_smp_send_keypress_ntf_pdu(gapc_le_smp_pair_proc_t* p_proc, uint8_t notification_type)
{
    l2cap_smp_keypress_ntf_t pdu =
    {
            .code     = L2CAP_SMP_KEYPRESS_NTF_OPCODE,
            .ntf_type = notification_type,
    };

    return gapc_le_smp_pdu_send_in_transaction(p_proc, (l2cap_smp_pdu_t*) &pdu);
}

/// @brief Start the SMP Transaction timer timer
void gapc_le_smp_start_trans_timer(gapc_sec_proc_hdr_t* p_proc)
{
    gapc_sdt_timer_set(&(p_proc->trans_timer), GAP_SMP_TRANS_TIMEOUT_MS);
}

/// @brief Stop the SMP Transaction timer timer
void gapc_le_smp_stop_trans_timer(gapc_sec_proc_hdr_t* p_proc)
{
    gapc_sdt_stop(&(p_proc->trans_timer));
}

uint16_t gapc_le_smp_create(gapc_le_con_t* p_con, uint8_t conidx)
{
    uint16_t status;

    // Initialize timer for connection
    gapc_sdt_prepare(&(p_con->smp.rep_attempt_timer), conidx, GAPC_SDT_SMP);

    // assign a channel to L2CAP transaction
    status = l2cap_chan_fix_register(conidx, L2CAP_CID_SECURITY, L2CAP_SMP_MTU, &gapc_le_smp_chan_cb, &(p_con->smp.chan_lid));
    ASSERT_WARN(p_con->smp.chan_lid != L2CAP_INVALID_CHAN_LID, conidx, status);

    return (status);
}

#endif // (BLE_GAPC)

/// @} GAPC
