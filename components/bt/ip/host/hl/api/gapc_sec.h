/**
 ****************************************************************************************
 *
 * @file gapc_sec.h
 *
 * @brief Generic Access Profile Controller - Security API.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */


#ifndef _GAPC_SEC_H_
#define _GAPC_SEC_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gapc.h"

/**
 ****************************************************************************************
 * @addtogroup GAPC_SEC_API Connection Security (Bond and Encrypt)
 * @ingroup GAPC_API
 * @brief Set of function and callback to handle connection security.
 *
 * Information about handling security is provided in \ref gap_link_sec_section documentation section
 *
 * Example of LE Security interface callback function handling:
 * \snippet{lineno} app_test_le_periph.c APP_LE_SEC_CB
 *
 * \if(btdm)
 * Example of BT-Classic Security interface callback function handling:
 * \snippet{lineno} app_connection.c APP_BT_SEC_CB
 * \endif
 *
 * @{
 * @}
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// @addtogroup GAPC_ENUM_API
/// @{

/// Keypress Notification types
enum gapc_key_ntf_type
{
    /// Passkey entry started
    GAPC_KEY_NTF_ENTRY_STARTED = 0x00,
    /// Passkey digit entered
    GAPC_KEY_NTF_DIGIT_ENTERED,
    /// Passkey digit erased
    GAPC_KEY_NTF_DIGIT_ERASED,
    /// Passkey cleared
    GAPC_KEY_NTF_CLEARED,
    /// Passkey entry completed
    GAPC_KEY_NTF_ENTRY_COMPLETED
};

/// Pairing info
enum gapc_expected_info
{
    /// Identity resolving key
    GAPC_INFO_IRK,
    /// Connection signature resolving key
    GAPC_INFO_CSRK,
    /// Temporary key - OOB data information - Legacy Pairing
    GAPC_INFO_TK_OOB,
    /// Temporary key - Pin code displayed - Legacy Pairing
    GAPC_INFO_TK_DISPLAYED,
    /// Temporary key - Pin code entered - Legacy Pairing
    GAPC_INFO_TK_ENTERED,
    /// OOB data information - Secure Connection
    GAPC_INFO_OOB,
    /// Pass key Displayed- Secure Connection
    GAPC_INFO_PASSKEY_DISPLAYED,
    /// Pass key Entered - Secure Connection
    GAPC_INFO_PASSKEY_ENTERED,
    /// Bluetooth classic IO capabilities
    GAPC_INFO_BT_IOCAP,
    /// Bluetooth classic PIN code value
    GAPC_INFO_BT_PIN_CODE,
    /// Bluetooth classic Passkey value
    GAPC_INFO_BT_PASSKEY,
    GAPC_INFO_MAX,
};

/// @} GAPC_ENUM_API

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// @addtogroup GAPC_STRUCT_API
/// @{

/// Pairing parameters
/*@TRACE*/
typedef struct gapc_pairing
{
    /// IO capabilities (see enum #gap_io_cap)
    uint8_t iocap;
    /// OOB information (see enum #gap_le_oob_flag)
    uint8_t oob;
    /// Authentication (see enum #gap_auth)
    /// Note in BT 4.1 the Auth Field is extended to include 'Key Notification' and
    /// and 'Secure Connections'.
    uint8_t auth;
    /// Encryption key size (7 to 16)
    uint8_t key_size;
    ///Initiator key distribution (see enum #gap_kdist)
    uint8_t ikey_dist;
    ///Responder key distribution (see enum #gap_kdist)
    uint8_t rkey_dist;
} gapc_pairing_t;

/// Long Term Key information
/*@TRACE*/
typedef struct gapc_ltk
{
    /// Long Term Key
    gap_sec_key_t key;
    /// Encryption Diversifier
    uint16_t      ediv;
    /// Random Number
    rand_nb_t     randnb;
    /// Encryption key size (7 to 16)
    uint8_t       key_size;
} gapc_ltk_t;


/// Identity Resolving Key information
/*@TRACE*/
typedef struct gapc_irk
{
    /// Identity Resolving Key
    gap_sec_key_t key;
    /// Device BD Identity Address
    gap_bdaddr_t  addr;
} gapc_irk_t;

/// Pairing Key information that can be stored in non-volatile memory
/*@TRACE*/
typedef struct gapc_pairing_keys
{
    /// Bit field that describe which key is valid (see enum #gap_kdist)
    uint8_t         valid_key_bf;
    /// Pairing security level (see enum #gap_pairing_lvl)
    uint8_t         pairing_lvl;
    /// Identity resolving key information - for address resolution
    gapc_irk_t      irk;
    /// Long term key - for encryption on Low Energy connection
    gapc_ltk_t      ltk;
    /// Connection Signature Resolving Key - for attribute packet signature
    gap_sec_key_t   csrk;
    /// Link key - for encryption on BR/EDR connection
    gap_sec_key_t   link_key;
} gapc_pairing_keys_t;

/// @} GAPC_SEC_API

/*
 * CALLBACK DEFINITIONS
 ****************************************************************************************
 */

/// @addtogroup GAPC_SEC_API
/// @{

/// Callback structure required to handle security events
typedef struct gapc_security_cb
{
    /**
     ****************************************************************************************
     * Callback executed when an LE encryption is requested by peer device
     * Request shall be accept using #gapc_le_encrypt_req_reply
     *
     * @note Mandatory callback for peripheral if pairing supported
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] ediv          Encryption diversifier
     * @param[in] p_rand        Pointer to random number value
     ****************************************************************************************
     */
    void (*le_encrypt_req)(uint8_t conidx, uint32_t dummy, uint16_t ediv, const rand_nb_t* p_rand);

    /**
     ****************************************************************************************
     * Callback executed when link key is requested for BR/EDR encryption.
     * Request shall be accept using #gapc_bt_encrypt_req_reply
     *
     * @note Mandatory callback if BT Classic role is supported
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     ****************************************************************************************
     */
    void (*bt_encrypt_req)(uint8_t conidx, uint32_t dummy);

    /**
     ****************************************************************************************
     * @brief Callback executed when link authentication information available
     *
     * @note Optional callback
     *
     * @param[in] conidx    Connection index
     * @param[in] dummy     Dummy parameter provided by upper layer application
     * @param[in] sec_lvl   Link security level (see enum #gap_sec_lvl)
     * @param[in] encrypted True if link is encrypted, false otherwise
     ****************************************************************************************
     */
    void (*auth_info)(uint8_t conidx, uint32_t dummy, uint8_t sec_lvl, bool encrypted);

    /**
     ****************************************************************************************
     * Callback executed to inform that an on-going pairing has succeeded
     *
     * @note Mandatory callback
     *
     * @param[in] conidx         Connection index
     * @param[in] dummy          Dummy parameter provided by upper layer application
     *
     ****************************************************************************************
     */
    void (*pairing_succeed)(uint8_t conidx, uint32_t dummy);

    /**
     ****************************************************************************************
     * Callback executed to inform that an on-going pairing has failed
     *
     * @note Mandatory callback
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] reason        Failing reason
     ****************************************************************************************
     */
    void (*pairing_failed)(uint8_t conidx, uint32_t dummy, uint16_t reason);

    /**
     ****************************************************************************************
     * Callback executed when an information is required by pairing algorithm.\n
     *
     * Request shall be be answered with:
     * - #gapc_pairing_provide_irk  if exp_info = #GAPC_INFO_IRK
     * - #gapc_pairing_provide_csrk if exp_info = #GAPC_INFO_CSRK
     * - #gapc_pairing_provide_tk if exp_info = #GAPC_INFO_TK_OOB or #GAPC_INFO_TK_DISPLAYED or #GAPC_INFO_TK_ENTERED
     * - #gapc_pairing_provide_oob_data if exp_info = #GAPC_INFO_OOB
     * - #gapc_pairing_provide_passkey if exp_info = #GAPC_INFO_PASSKEY_DISPLAYED or #GAPC_INFO_PASSKEY_ENTERED or #GAPC_INFO_BT_PASSKEY
     * - #gapc_pairing_provide_iocap if exp_info = #GAPC_INFO_BT_IOCAP
     * - #gapc_pairing_provide_pin_code if exp_info = #GAPC_INFO_BT_PIN_CODE
     *
     * @note Mandatory callback
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] exp_info      Expected information (see enum #gapc_expected_info)
     ****************************************************************************************
     */
    void (*info_req)(uint8_t conidx, uint32_t dummy, uint8_t exp_info);

    /**
     ****************************************************************************************
     * Callback executed to inform that peer LE peripheral is asking for a pairing / encryption
     * with a specific authentication level.\n
     *
     * Starting pairing / encryption is optional on central side.
     *
     * @note Optional callback
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] auth_level    Authentication level (@ref gap_auth)
     ****************************************************************************************
     */
    void (*auth_req)(uint8_t conidx, uint32_t dummy, uint8_t auth_level);

    /**
     ****************************************************************************************
     * Callback executed to inform that a LE pairing is initiated by peer central.
     * Request shall be accept using #gapc_pairing_accept
     *
     * @note Mandatory callback for peripheral if pairing supported
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] auth_level    Authentication level (@ref gap_auth)
     ****************************************************************************************
     */
    void (*pairing_req)(uint8_t conidx, uint32_t dummy, uint8_t auth_level);

    /**
     ****************************************************************************************
     * @brief Callback executed when peer BT-classic IO capabilities are received.
     *
     * @note Optional callback.
     *
     * @param[in] conidx       Connection index
     * @param[in] dummy        Dummy parameter provided by upper layer application
     * @param[in] iocap        Local device IO capabilities (@ref gap_io_cap)
     * @param[in] auth_req_bf  Authentication requirement bit field (see enum #gap_bt_auth_req_bf)
     * @param[in] oob_present  True if OOB data is present, False otherwise
     ****************************************************************************************
     */
    void (*peer_iocap)(uint8_t conidx, uint32_t dummy, uint8_t iocap, uint8_t auth_req_bf, bool oob_present);

    /**
     ****************************************************************************************
     * Callback request some keys that must be exchange during pairing procedure\n
     * Request shall be be answered with: #gapc_pairing_numeric_compare_rsp
     *
     * @note Mandatory callback for LE secure connection or BT Classic pairing
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] numeric_value Numeric value to display
     ****************************************************************************************
     */
    void (*numeric_compare_req)(uint8_t conidx, uint32_t dummy, uint32_t numeric_value);

    /**
     ****************************************************************************************
     * @brief Callback executed during BT-classic pairing to display the numeric value
     *
     * @note Mandatory for BT Classic pairing
     *
     * @param[in] conidx    Connection index
     * @param[in] dummy     Dummy parameter provided by upper layer application
     * @param[in] passkey   Passkey numeric value to display
     *
     ****************************************************************************************
     */
    void (*display_passkey)(uint8_t conidx, uint32_t dummy, uint32_t passkey);

    /**
     ****************************************************************************************
     * Callback executed when peer key pressed is received
     *
     * @note Optional callback.
     *
     * @param[in] conidx            Connection index
     * @param[in] dummy             Dummy parameter provided by upper layer application
     * @param[in] notification_type Key pressed information (see enum #gapc_key_ntf_type)
     ****************************************************************************************
     */
    void (*key_pressed)(uint8_t conidx, uint32_t dummy, uint8_t notification_type);

    /**
     ****************************************************************************************
     * Callback executed when an information is required by pairing algorithm.\n
     *
     * Request shall be be answered with #gapc_pairing_provide_ltk
     *
     * @note Required callback if legacy pairing supported.
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] key_size      Size of the LTK to provide (range [7-16])
     ****************************************************************************************
     */
    void (*ltk_req)(uint8_t conidx, uint32_t dummy, uint8_t key_size);

    /**
     ****************************************************************************************
     * Callback used to indicate pairing keys that must be stored on a non-volatile memory.
     *
     * @note Mandatory callback
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] p_keys        Pointer to structure that contains keys received / computed
     ****************************************************************************************
     */
    void (*key_received)(uint8_t conidx, uint32_t dummy, const gapc_pairing_keys_t* p_keys);

    /**
     ****************************************************************************************
     * Callback executed to inform that a pairing repeated attempt problem is detected.
     * Peer device is supposed to wait before initiating a new pairing
     *
     * @note Optional callback
     *
     * @param[in] conidx         Connection index
     * @param[in] dummy          Dummy parameter provided by upper layer application
     *
     ****************************************************************************************
     */
    void (*repeated_attempt)(uint8_t conidx, uint32_t dummy);
} gapc_security_cb_t;
/// @} GAPC_SEC_API


/*
 * MACROS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/// @addtogroup GAPC_SEC_API Connection Security (Bond and Encrypt)
/// @{
// ---------------------------- SECURITY API ----------------------------------------------

/**
 ****************************************************************************************
 * @brief On LE connection, encrypt an exiting BLE connection
 *
 * @note Can be initiated only by Central of the connection.
 * @note Once link is encrypted, #gapc_security_cb_t.auth_info is called
 *
 * @param[in] conidx     Connection index
 * @param[in] dummy      Dummy parameter provided by upper layer application
 * @param[in] p_ltk_info Pointer to LTK information
 * @param[in] cmp_cb     Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_encrypt(uint8_t conidx, uint32_t dummy, const gapc_ltk_t* p_ltk_info, gapc_proc_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * @brief On LE connection, this function shall be used to accept or reject encryption request from peer device
 *
 * @note Once link is encrypted, #gapc_security_cb_t.auth_info is called
 *
 * @param[in] conidx     Connection index
 * @param[in] accept     True to accept, False to reject
 * @param[in] p_ltk      Pointer to LTK value
 * @param[in] key_size   Size of LTK key (range [7:16])
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_le_encrypt_req_reply(uint8_t conidx, bool accept, const gap_sec_key_t* p_ltk, uint8_t key_size);

/**
 ****************************************************************************************
 * @brief On BT-Classic connection, this function shall be used to accept or reject encryption request.
 *
 * @note Once link is encrypted, #gapc_security_cb_t.auth_info is called
 *
 * @param[in] conidx     Connection index
 * @param[in] accept     True to accept, False to reject
 * @param[in] p_link_key Pointer to link key value
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_bt_encrypt_req_reply(uint8_t conidx, bool accept, const gap_sec_key_t* p_link_key);

/**
 ****************************************************************************************
 * @brief On LE connection, peripheral side, ask central to secure BLE connection with a certain authentication level
 *
 * @param[in] conidx   Connection Index
 * @param[in] auth     Authentication (see enum #gap_auth)
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_le_send_security_request(uint8_t conidx, uint8_t auth);

/**
 ****************************************************************************************
 * @brief On LE connection, this function is used to initiate a pairing
 *
 * @note Can be initiated only by Central of the connection.
 *
 * @param[in] conidx         Connection index
 * @param[in] p_pairing_info Pointer to pairing information
 * @param[in] sec_req_level  Device security requirements (minimum security level). (see enum #gap_sec_req)
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_start_pairing(uint8_t conidx, const gapc_pairing_t* p_pairing_info, uint8_t sec_req_level);

/**
 ****************************************************************************************
 * @brief On LE connection, this function shall be used to accept or reject pairing request from peer device
 *
 * @param[in] conidx         Connection index
 * @param[in] accept         True to accept, false to reject.
 * @param[in] p_pairing_info Pointer to local pairing requirement info
 * @param[in] sec_req_level  Device security requirements (minimum security level). (see enum #gap_sec_req)
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_pairing_accept(uint8_t conidx, bool accept, const gapc_pairing_t* p_pairing_info, uint8_t sec_req_level);

/**
 ****************************************************************************************
 * @brief On LE connection, this function shall be used to provide requested entered or displayed pin code
 *        or OOB Data for legacy pairing
 *
 * @param[in] conidx     Connection index
 * @param[in] accept     True to accept, false to reject.
 * @param[in] p_tk       Pointer to Temporary key.
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_pairing_provide_tk(uint8_t conidx, bool accept, const gap_sec_key_t* p_tk);

/**
 ****************************************************************************************
 * @brief On LE connection, this function shall be used to provide requested OOB data - Secure connection
 *
 * @param[in] conidx     Connection index
 * @param[in] accept     True to accept, false to reject.
 * @param[in] p_data     Pointer to OOB data
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_pairing_provide_oob_data(uint8_t conidx, bool accept, const gap_oob_t* p_data);

/**
 ****************************************************************************************
 * @brief On LE connection, this function shall be used to provide requested entered or displayed pass key
 *        - Secure connection
 *
 * @param[in] conidx     Connection index
 * @param[in] accept     True to accept, false to reject.
 * @param[in] passkey    Pass key value (range [0, 999999])
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_pairing_provide_passkey(uint8_t conidx, bool accept, uint32_t passkey);

/**
 ****************************************************************************************
 * @brief On LE connection, this function shall be used to provide requested LTK - legacy pairing only
 *
 * @param[in] conidx     Connection index
 * @param[in] p_ltk      Pointer to LTK information
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_pairing_provide_ltk(uint8_t conidx, const gapc_ltk_t* p_ltk);

/**
 ****************************************************************************************
 * @brief On LE connection, this function shall be used to provide requested IRK - legacy pairing only
 *
 * @param[in] conidx     Connection index
 * @param[in] p_irk      Pointer to IRK information
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_pairing_provide_irk(uint8_t conidx, const gap_sec_key_t* p_irk);

/**
 ****************************************************************************************
 * @brief On LE connection, this function shall be used to provide requested CSRK.
 *
 * @param[in] conidx     Connection index
 * @param[in] p_csrk     Pointer to CSRK information
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_pairing_provide_csrk(uint8_t conidx, const gap_sec_key_t* p_csrk);


/**
 ****************************************************************************************
 * @brief On LE connection, during a passkey entry pairing, this function informs peer device about user actions.
 *
 * @param[in] conidx            Connection index
 * @param[in] notification_type Key notification type (see enum #gapc_key_ntf_type)
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_send_key_pressed_notification(uint8_t conidx, uint8_t notification_type);

/**
 ****************************************************************************************
 * @brief This function shall be used to accept or reject the requested numeric comparison
 *
 * @param[in] conidx     Connection index
 * @param[in] accept     True to accept, false to reject.
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_pairing_numeric_compare_rsp(uint8_t conidx, bool accept);

/**
 ****************************************************************************************
 * @brief  BT Classic link, send back IO capabilities to continue pairing
 *
 * @param[in] conidx        Connection index
 * @param[in] accept        True to continue pairing, false to reject it
 * @param[in] iocap         Local device IO capabilities (@ref gap_io_cap)
 * @param[in] auth_req_bf   Authentication requirement bit field (see enum #gap_bt_auth_req_bf)
 * @param[in] p_oob_192     Pointer to received P-192 OOB data (NULL nothing received)
 * @param[in] p_oob_256     Pointer to received P-256 OOB data (NULL nothing received)
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_pairing_provide_iocap(uint8_t conidx, bool accept, uint8_t iocap, uint8_t auth_req_bf,
                                    const gap_oob_t* p_oob_192, const gap_oob_t* p_oob_256);

/**
 ****************************************************************************************
 * @brief BT Classic link, provide PIN code requested during pairing phase.
 *
 * Pin code automatically rejected if no pin code provided or if PIN code length is out of valid range
 *
 * @param[in] conidx        Connection index
 * @param[in] accept        True to accept, false to reject PIN code entry.
 * @param[in] pin_code_len  PIN code length (range [1, 16])
 * @param[in] p_pin_code    Pointer to PIN code value
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_pairing_provide_pin_code(uint8_t conidx, bool accept, uint8_t pin_code_len, const uint8_t* p_pin_code);


/**
 ****************************************************************************************
 * @brief For a BT Classic link, authenticate link with a required security level to achieve.
 *
 * According to link key manager content, it will either start link encryption
 * or initiate a new pairing.
 *
 * Pairing will be initiated if there is no link key with sufficient security level to authenticate
 * connection with peer device on link key manager.
 *
 * If link is authenticated with at least required level, no action is performed.
 *
 * @note A BT Classic profile could specify a specific security level that will automatically
 *       initiate authentication.
 *
 * @param[in] conidx    Connection index
 * @param[in] sec_lvl   Required security level (see enum #gap_sec_lvl)
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_bt_set_required_sec_lvl(uint8_t conidx, uint8_t sec_lvl);
/// @} GAPC_SEC_API

#endif /* _GAPC_SEC_H_ */
