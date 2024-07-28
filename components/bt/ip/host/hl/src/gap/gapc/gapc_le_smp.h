/**
 ****************************************************************************************
 *
 * @file gapc_le_smp.h
 *
 * @brief GAP LE SMP API
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


#ifndef _GAPC_LE_SMP_H_
#define _GAPC_LE_SMP_H_

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
#include "gapc_le_con.h"
#include "../gap_int.h"
#include "l2cap.h"
#include "../../inc/l2cap_smp.h" // Should Only be included there (L2CAP protocol defines)
#include "co_buf.h"
#include <string.h> // for memset/memcopy

/*
 * MACROS
 ****************************************************************************************
 */

/// Authentication Request mask
#define GAPC_LE_SMP_MASK_AUTH_REQ(req)                (req & 0x07)

/// Mask a Pairing Failed reason value with the provided mask.
#define GAPC_LE_SMP_ERR_TO_HL_ERR(mask, reason)  ((GAPC_LE_SMP_## mask ##_ERR_MASK) | reason)


/*
 * DEFINES
 ****************************************************************************************
 */

/// Mask applied to a Pairing Failed error triggered by us.
#define GAPC_LE_SMP_LOC_ERR_MASK                        (0x60)
/// Mask applied to a Pairing Failed error triggered by the peer device.
#define GAPC_LE_SMP_REM_ERR_MASK                        (0x70)


/// Pairing Request and Pairing Response PDU Length
#define GAPC_LE_SMP_CODE_PAIRING_REQ_RESP_LEN             (7)

/// Minimum Encryption key size
#define GAPC_LE_SMP_MIN_ENC_SIZE_LEN                      (7)
/// Maximum Encryption Key size
#define GAPC_LE_SMP_MAX_ENC_SIZE_LEN                     (16)

/// Pairing expected value
enum gapc_exp_value
{
    /// OOB data information
    GAPC_LE_SMP_VALUE_OOB,
    /// Internal usage Pass key
    GAPC_LE_SMP_VALUE_PASSKEY,
    /// Internal usage Temporary key
    GAPC_LE_SMP_VALUE_TK,
    /// Internal usage Pairing confirmation
    GAPC_LE_SMP_VALUE_PAIRING_CFM,
    /// Internal usage Numeric comparison
    GAPC_LE_SMP_VALUE_NUMERIC_COMPARE,
    /// Long term key
    GAPC_LE_SMP_VALUE_LTK,
    /// Identity resolving key
    GAPC_LE_SMP_VALUE_IRK,
    /// Connection signature resolving key
    GAPC_LE_SMP_VALUE_CSRK,

    GAPC_LE_SMP_VALUE_INVALID  = 0xFF,
};

/// Repeated Attempts Attack Detection status
enum gapc_le_smp_attempts_status
{
    /// No attack has been detected
    GAPC_LE_SMP_NO_REP_ATTEMPTS = 0,
    /// An attack has already been detected, drop the message
    GAPC_LE_SMP_REP_ATTEMPTS_ATTACK,
    /// Repeated Attempt detected, need to send a Pairing Failed PDU to the peer device
    GAPC_LE_SMP_REP_ATTEMPT
};

/// STK generation methods
enum gapc_le_smp_method
{
    ///Just Works Method
    GAPC_LE_SMP_METH_JW = 0x00,
    ///PassKey Entry Method
    GAPC_LE_SMP_METH_PK,
    ////OOB Method
    GAPC_LE_SMP_METH_OOB,
    ////Numeric Comparison
    GAPC_LE_SMP_METH_NC
};

/// GAPC_LE_SMP messages expected from peer device
enum gapc_le_smp_exp_message
{
    GAPC_LE_SMP_W4_NOTHING,

    // ------------------------------------
    // Pairing Features Exchange Phase
    // ------------------------------------

    /// Is waiting for the pairing response
    GAPC_LE_SMP_W4_RX_PAIRING_RSP,

    /// Wait for public key reception - Secure Connections
    GAPC_LE_SMP_W4_RX_PUBLIC_KEY,

    /// Wait for Pairing confirm from peer device
    GAPC_LE_SMP_W4_RX_PAIRING_CONFIRM,

    /// Wait for pairing random from peer device
    GAPC_LE_SMP_W4_RX_PAIRING_RAND,


    //-----------------------------
    // Secure Connections - Phase 2
    //-----------------------------

    /// Wait reception of DH-Key Check from peer device
    GAPC_LE_SMP_W4_RX_DHKEY_CHECK,

    // ------------------------------------
    // Encryption Phase
    // ------------------------------------

    /// Waiting the change encryption event
    GAPC_LE_SMP_W4_ENC,

    // ------------------------------------
    // Transport Keys Distribution Phase
    // ------------------------------------
    /// Is waiting for the peer LTK
    GAPC_LE_SMP_W4_RX_LTK,
    /// Is waiting for the peer EDIV and Rand Value
    GAPC_LE_SMP_W4_RX_MST_ID,
    /// Is waiting for the peer IRK
    GAPC_LE_SMP_W4_RX_IRK,
    /// Is waiting for the peer BD Address
    GAPC_LE_SMP_W4_RX_BD_ADDR,
    /// Is waiting for the peer CSRK
    GAPC_LE_SMP_W4_RX_CSRK,
};

/// Timer State Masks
enum gapc_le_smp_timer_state
{
    /// Repeated Attempts Timer
    GAPC_LE_SMP_TIMER_REP_ATT_POS                    = 1,
    GAPC_LE_SMP_TIMER_REP_ATT_BIT                    = 0x02,
    /// Blocked because of SMP Timeout
    GAPC_LE_SMP_TIMER_TIMEOUT_BLOCKED_POS            = 2,
    GAPC_LE_SMP_TIMER_TIMEOUT_BLOCKED_BIT            = 0x04,
};

/// Information of pairing
enum gapc_le_smp_info_bf
{
    /// Used to know if LTK has been exchanged
    GAPC_LE_SMP_LTK_EXCH_POS         = 0,
    GAPC_LE_SMP_LTK_EXCH_BIT         = 0x01,
    /// Local DH Key Computed
    GAPC_LE_SMP_LOC_DHKEY_COMP_POS   = 1,
    GAPC_LE_SMP_LOC_DHKEY_COMP_BIT   = 0x02,
    /// OOB Data Confirm value properly computed
    GAPC_LE_SMP_OOB_CFM_COMP_POS     = 2,
    GAPC_LE_SMP_OOB_CFM_COMP_BIT     = 0x04,
    /// Inform that local public key has been generated
    GAPC_LE_SMP_LOC_PUB_KEY_GEN_POS  = 3,
    GAPC_LE_SMP_LOC_PUB_KEY_GEN_BIT  = 0x08,
    /// During a pairing procedure, application can be involve (request + confirm)
    /// This bit is used to know if response is provided within request function call
    GAPC_LE_SMP_IN_PAIRING_PROC_POS  = 4,
    GAPC_LE_SMP_IN_PAIRING_PROC_BIT  = 0x10,
    /// Secure connection pairing
    GAPC_LE_SMP_SC_PAIRING_POS       = 5,
    GAPC_LE_SMP_SC_PAIRING_BIT       = 0x20,
    /// True if user accepts request, false if rejected
    GAPC_LE_SMP_USER_ACCEPT_POS      = 6,
    GAPC_LE_SMP_USER_ACCEPT_BIT      = 0x40,
};

/// SMP Procedure transition
enum gapc_le_smp_evt_proc_transition
{
    /// Application initiates pairing
    GAPC_LE_SMP_EVT_APPLI_INITIATE_PAIRING,
    /// Pairing request received
    GAPC_LE_SMP_EVT_PAIRING_REQ_RECEIVED,
    /// Pairing response received
    GAPC_LE_SMP_EVT_PAIRING_RSP_RECEIVED,
    /// Application answer pairing request
    GAPC_LE_SMP_EVT_APPLI_ANSWER_PAIRING_REQ,

    // Main Procedure transition
    /// Authentication started
    GAPC_LE_SMP_EVT_PAIRING_AUTHENTICATION_STARTED,
    /// Authentication stage 1 - Pairing procedure started
    GAPC_LE_SMP_EVT_PAIRING_STAGE_1_STARTED,
    /// Authentication stage 2 - DH-Key Check
    GAPC_LE_SMP_EVT_DH_KEY_CHECK_STARTED,
    /// Phase 3 Key distribution
    GAPC_LE_SMP_EVT_KEY_EXCHANGE_STARTED,


    // AES algo results
    /// Generate a random value
    GAPC_LE_SMP_EVT_LOCAL_RAND_GENERATED,
    /// compute local confirm value
    GAPC_LE_SMP_EVT_LOCAL_CONFIRM_COMPUTED,
    /// compute peer confirm value
    GAPC_LE_SMP_EVT_PEER_CONFIRM_COMPUTED,
    /// compute STK value
    GAPC_LE_SMP_EVT_STK_COMPUTED,

    // AES algo results for secure connection
    /// Compute Numeric value for numeric comparison - Secure connection
    GAPC_LE_SMP_EVT_NUMERIC_VALUE_COMPUTED,
    /// computation of local MAC key
    GAPC_LE_SMP_EVT_MAC_KEY_COMPUTED,
    /// Use to compute local DH-Key Check
    GAPC_LE_SMP_EVT_LOCAL_DHKEY_CHECK_COMPUTED,
    /// Use compute peer DH-Key Check - Secure connection
    GAPC_LE_SMP_EVT_PEER_DHKEY_CHECK_COMPUTED,

    // ECDH algo results
    /// Local Public key computed
    GAPC_LE_SMP_EVT_PUBLIC_KEY_COMPUTED,
    /// Local DH-KEY Computed
    GAPC_LE_SMP_EVT_DH_KEY_COMPUTED,

    // PDU RX EVENTS
    /// Peer pair confirm PDU Received
    GAPC_LE_SMP_EVT_PEER_CONFIRM_RECEIVED,
    /// Peer random PDU Received
    GAPC_LE_SMP_EVT_PEER_RANDOM_RECEIVED,
    /// Peer Public key PDU received
    GAPC_LE_SMP_EVT_PEER_PUB_KEY_RECEIVED,
    /// Peer DH-Key check PDU received
    GAPC_LE_SMP_EVT_PEER_DH_KEY_CHECK_RECEIVED,

    // HCI - EVENTS
    /// HCI LE ENC Status event received
    GAPC_LE_SMP_EVT_HCI_ENC_CMD_EVT_RECEIVED,
    /// LTK Request by controller through HCI
    GAPC_LE_SMP_EVT_HCI_LTK_REQ_RECEIVED,
    /// HCI LE Encryption performed
    GAPC_LE_SMP_EVT_HCI_ENC_CHANGED_EVT_RECEIVED,

    // Keys Exchange - received
    /// LTK has been received
    GAPC_LE_SMP_EVT_PEER_LTK_RECEIVED,
    /// Master ID for LTK has been received
    GAPC_LE_SMP_EVT_PEER_MASTER_ID_RECEIVED,
    /// IRK has been received
    GAPC_LE_SMP_EVT_PEER_IRK_RECEIVED,
    /// Address info for IRK has been received
    GAPC_LE_SMP_EVT_PEER_ADDR_INFO_RECEIVED,
    /// CSRK has been received
    GAPC_LE_SMP_EVT_PEER_CSRK_RECEIVED,
    #if (BT_HOST_PRESENT)
    /// Link Key or LTK derivation key computed
    GAPC_LE_SMP_EVT_DERIV_KEY_COMPUTED,
    #endif // (BT_HOST_PRESENT)
    // Value provided by application
    /// Legacy pairing TK provided by application
    GAPC_LE_SMP_EVT_TK_PROVIDED,

    /// OOB Data provided by application
    GAPC_LE_SMP_EVT_OOB_DATA_PROVIDED,
    /// Application accepts or reject numeric comparison
    GAPC_LE_SMP_EVT_NUMERIC_COMPARE_RSP_PROVIDED,
    /// Passkey provided by application
    GAPC_LE_SMP_EVT_PASSKEY_PROVIDED,

    /// Local LTK provided by application
    GAPC_LE_SMP_EVT_LOCAL_LTK_PROVIDED,
    /// Local IRK provided by application
    GAPC_LE_SMP_EVT_LOCAL_IRK_PROVIDED,
    /// Local CSRK provided by application
    GAPC_LE_SMP_EVT_LOCAL_CSRK_PROVIDED,

    // Pairing aborted
    /// Peer abort pairing
    GAPC_LE_SMP_EVT_PAIRING_ABORTED,
};



/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
 
typedef struct gapc_le_smp_pair_proc gapc_le_smp_pair_proc_t;

/**
 ****************************************************************************************
 * @brief Handler of a procedure transition event.
 *
 * @param[in]  p_con          Pointer to connection environment
 * @param[in]  p_proc         Pointer to LE SMP procedure
 * @param[in]  event          Procedure event transition
 * @param[out] p_is_finished  True if procedure is finished, false otherwise
 *
 * @return Transition execution status (see enum #hl_err).
 *         Procedure automatically terminated if status != GAP_ERR_NO_ERROR.
 *
 ****************************************************************************************
 */
typedef uint16_t (*gapc_le_smp_proc_transition_cb)(gapc_le_smp_pair_proc_t* p_proc, uint8_t event, bool* p_is_finished);
 

/// Pairing procedure information
typedef struct gapc_le_smp_pair_proc
{
    /// Procedure header
    gapc_sec_proc_hdr_t hdr;
    /// TK during Phase 2, LTK or IRK during Phase 3
    uint8_t             key[GAP_KEY_LEN];
    /// Pairing request command
    gapc_pairing_t      pair_req_pdu;
    /// Pairing response feature
    gapc_pairing_t      pair_rsp_pdu;
    /// Passkey value
    uint32_t            passkey;
    /// Local Random number value
    uint8_t             local_rand[RAND_VAL_LEN];
    /// Peer random number value
    uint8_t             peer_rand[RAND_VAL_LEN];
    /// Local confirm or DH-Key Check Confirm value send
    uint8_t             local_confirm[CFM_LEN];
    /// Peer confirm or DH-Key Check Confirm value to check
    uint8_t             peer_confirm[CFM_LEN];
    /// Local public key value
    public_key_t        local_public_key;
    /// Peer public key value
    public_key_t        peer_public_key;
    /// Computed MAC Key
    uint8_t             mac_key[GAP_KEY_LEN];
    /// Computed DH-Key Value
    uint8_t             dh_key[DH_KEY_LEN];
    /// Required for OOB Peer R value
    uint8_t             peer_r[GAP_KEY_LEN];
    /// Required for OOB Local R value
    uint8_t             local_r[GAP_KEY_LEN];
    /// Pairing Method
    uint8_t             pair_method;
    /// Passkey management cursor
    uint8_t             passkey_bit_cursor;
    /// state information of pairing (see enum #gapc_le_smp_info_bf)
    uint8_t             info_bf;
    /// Security requirements
    uint8_t             sec_req_level;
    /// Pairing level
    uint8_t             pairing_lvl;
    /// Key to be exchanged (transmitted or to be received)
    uint8_t             keys_dist;
    /// Application expected value  (see enum #gapc_exp_value)
    uint8_t             app_exp_value;
    /// Message expected from peer device (@see gapc_le_smp_exp_message)
    uint8_t             peer_exp_message;
    /// Negotiated key size
    uint8_t             key_size;
} gapc_le_smp_pair_proc_t;

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/// Inform application about pairing failed
void gapc_le_smp_exe_pairing_failed_cb(gapc_le_con_t* p_con, uint16_t reason);

/**
 ****************************************************************************************
 * Continue pairing procedure with provided event transition
 *
 * @param[in] p_con    Pointer to Connection environment
 * @param[in] event    Event transition value (see enum #gapc_le_smp_event_transition)
 * @param[in] status   Procedure transition status
 *
 ****************************************************************************************
 */
void gapc_le_smp_proc_transition(gapc_le_smp_pair_proc_t *p_proc, uint8_t event, uint16_t status);

/// Try to do procedure transition, nothing is performed if already in procedure.
void gapc_le_smp_try_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event);

/// Get if application involved in pairing
bool gapc_le_smp_is_app_involved(uint8_t conidx, uint8_t provided_value, gapc_le_smp_pair_proc_t** pp_proc);

/// Get if message from peer device is expected
bool gapc_le_smp_is_peer_message_expected(gapc_le_con_t* p_con, uint8_t message, gapc_le_smp_pair_proc_t** pp_proc);

/// Retrieve active pairing procedure
gapc_le_smp_pair_proc_t* gapc_le_smp_get_pairing_proc(uint8_t conidx);

/// Check if authentication flag is valid
__INLINE bool gapc_le_smp_is_auth_valid(uint8_t auth)
{
    bool is_valid = (((GAPC_LE_SMP_MASK_AUTH_REQ(auth)) == GAP_AUTH_REQ_NO_MITM_NO_BOND)
                  || ((GAPC_LE_SMP_MASK_AUTH_REQ(auth)) == GAP_AUTH_REQ_NO_MITM_BOND)
                  || ((GAPC_LE_SMP_MASK_AUTH_REQ(auth)) == GAP_AUTH_REQ_MITM_NO_BOND)
                  || ((GAPC_LE_SMP_MASK_AUTH_REQ(auth)) == GAP_AUTH_REQ_MITM_BOND));

    return (is_valid);
}

/// Check if key size is within a valid range
__INLINE bool gapc_le_smp_is_key_size_valid(uint8_t key_size)
{
    return ((key_size >= GAPC_LE_SMP_MIN_ENC_SIZE_LEN) && (key_size <= GAPC_LE_SMP_MAX_ENC_SIZE_LEN));
}

/// Check if pairing is on-going
__INLINE bool gapc_le_smp_is_pairing_ongoing(gapc_le_con_t* p_con)
{
    return ((gapc_env.p_sec_proc != NULL) && (gapc_env.p_sec_proc->conidx == p_con->hdr.conidx));
}

/// Check if application provide the response within procedure transition execution
__INLINE bool gapc_le_smp_is_app_answer_request(gapc_le_smp_pair_proc_t *p_proc)
{
    return (p_proc->app_exp_value == GAPC_LE_SMP_VALUE_INVALID);
}

/// @brief Start the timer used to detect a Repeated Attempts attack
void gapc_le_smp_start_rep_attempt_timer(gapc_le_con_t* p_con);

/// @brief Check if an attack by repeated attempts has been triggered by the peer device
uint8_t gapc_le_smp_check_repeated_attempts(gapc_le_con_t* p_con);

/*
 * STATE MACHINE TRANSITION
 ****************************************************************************************
 */

/// Change transition state machine callback handler and update transition event
uint16_t gapc_le_smp_transition_to_new_state_machine(gapc_le_smp_pair_proc_t* p_proc,
                                                     gapc_le_smp_proc_transition_cb transition_cb,
                                                     uint8_t event,
                                                     bool* p_is_finished);

#if (HL_LE_CENTRAL)
/// Peripheral start legacy pairing
uint16_t gapc_le_smp_legacy_central_proc_start(gapc_le_smp_pair_proc_t* p_proc,  bool* p_is_finished);

/// Central start secure connection pairing
uint16_t gapc_le_smp_sc_central_proc_start(gapc_le_smp_pair_proc_t* p_proc, bool* p_is_finished);

/// Central start key exchange procedure
uint16_t gapc_le_smp_central_start_key_exch_proc_transition(gapc_le_smp_pair_proc_t* p_proc, bool* p_is_finished);

#endif // (HL_LE_CENTRAL)

#if (HL_LE_PERIPHERAL)
/// Peripheral start legacy pairing
uint16_t gapc_le_smp_legacy_periph_proc_start(gapc_le_smp_pair_proc_t* p_proc,  bool* p_is_finished);

/// Peripheral start secure connection pairing
uint16_t gapc_le_smp_sc_periph_proc_start(gapc_le_smp_pair_proc_t* p_proc,  bool* p_is_finished);

/// Peripheral start key exchange procedure
uint16_t gapc_le_smp_periph_start_key_exch_proc_transition(gapc_le_smp_pair_proc_t* p_proc, bool* p_is_finished);
#endif // (HL_LE_PERIPHERAL)

/// Check if user accepts the pairing
uint16_t gapc_le_smp_check_user_accept(gapc_le_smp_pair_proc_t* p_proc, uint8_t reason);

/*
 * SEND PDU FUNCTIONS
 ****************************************************************************************
 */

/// @brief Construct pairing failed PDU and send it
uint16_t gapc_le_smp_send_pairing_fail_pdu(gapc_le_con_t* p_con, uint8_t reason);

/// @brief Send pairing failed and start repeated attempt timer
uint16_t gapc_le_smp_send_pairing_fail_pdu_and_start_ra_timer(gapc_le_smp_pair_proc_t* p_proc, uint8_t reason);

/// @brief Construct security request PDU and send it
uint16_t gapc_le_smp_send_security_req_pdu(gapc_le_con_t* p_con, uint8_t auth);

#if(HL_LE_CENTRAL)
/// Send Pairing Request
uint16_t gapc_le_smp_send_pairing_req_pdu(gapc_le_smp_pair_proc_t* p_proc);
#endif // (HL_LE_CENTRAL)

#if(HL_LE_PERIPHERAL)
/// Send Pairing response
uint16_t gapc_le_smp_send_pairing_rsp_pdu(gapc_le_smp_pair_proc_t* p_proc);
#endif // (HL_LE_PERIPHERAL)

/// @brief Construct and send pairing confirm PDU.
uint16_t gapc_le_smp_send_pairing_confirm_pdu(gapc_le_smp_pair_proc_t* p_proc);

/// @brief Construct and send pairing random PDU.
uint16_t gapc_le_smp_send_pair_rand_pdu(gapc_le_smp_pair_proc_t* p_proc);

/// Send Pairing response
uint16_t gapc_le_smp_send_public_key_pdu(gapc_le_smp_pair_proc_t* p_proc);

/// @brief Construct DH Key PDU.
uint16_t gapc_le_smp_send_dhkey_check_pdu(gapc_le_smp_pair_proc_t* p_proc);

/// @brief Construct Key Pressed notification PDU and send it
uint16_t gapc_le_smp_send_keypress_ntf_pdu(gapc_le_smp_pair_proc_t* p_proc, uint8_t notification_type);

/// @brief Construct identification information PDU.
uint16_t gapc_le_smp_send_id_info_pdu(gapc_le_smp_pair_proc_t* p_proc);

/// @brief Construct identification address information PDU.
uint16_t gapc_le_smp_send_id_addr_info_pdu(gapc_le_smp_pair_proc_t* p_proc);

/// @brief Construct LTK PDU.
uint16_t gapc_le_smp_send_enc_info_pdu(gapc_le_smp_pair_proc_t* p_proc);

/// @brief Construct Master ID  PDU.
uint16_t gapc_le_smp_send_master_id_pdu(gapc_le_smp_pair_proc_t* p_proc);

/// @brief Construct CSRK PDU.
uint16_t gapc_le_smp_send_signing_info_pdu(gapc_le_smp_pair_proc_t* p_proc);

/// @brief Start the SMP Transaction timer timer
void gapc_le_smp_start_trans_timer(gapc_sec_proc_hdr_t* p_proc);

/// @brief Stop the SMP Transaction timer timer
void gapc_le_smp_stop_trans_timer(gapc_sec_proc_hdr_t* p_proc);

/*
 * AES FUNCTIONS
 ****************************************************************************************
 */
/// Generate local random value
void gapc_le_smp_generate_rand(gapc_le_smp_pair_proc_t* p_proc);

/// Local value confirm computed
void gapc_le_smp_local_confirm_cmp_cb(uint8_t aes_status, const uint8_t* aes_res, uint32_t conidx);

/// remote value confirm computed
void gapc_le_smp_peer_confirm_cmp_cb(uint8_t aes_status, const uint8_t* aes_res, uint32_t conidx);

/*
 * INFORM APPLICATION
 ****************************************************************************************
 */
/// Ask a specific information to the application
void gapc_le_smp_ask_info_to_app(gapc_le_smp_pair_proc_t* p_proc, uint8_t exp_info,
                                 uint8_t exp_value);

/// Indicate that pairing failes
void gapc_le_smp_indicate_pairing_failed(gapc_le_con_t* p_con, uint16_t reason);


/*
 * HCI FUNCTIONS
 ****************************************************************************************
 */

#if (HL_LE_CENTRAL)
/// @brief Send a request to the controller to start the encryption procedure.
uint16_t gapc_le_smp_send_start_enc_cmd(gapc_le_smp_pair_proc_t* p_proc);
#endif // (HL_LE_CENTRAL)

#if (HL_LE_PERIPHERAL)
/// @brief Send the LTK provided by the HL to the controller.
uint16_t gapc_le_smp_send_ltk_req_rsp(gapc_le_smp_pair_proc_t* p_proc);
#endif // (HL_LE_PERIPHERAL)

#endif // (BLE_GAPC)
/// @} GAPC

#endif /* _GAPC_LE_SMP_H_ */
