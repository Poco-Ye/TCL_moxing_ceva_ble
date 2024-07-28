/**
 ****************************************************************************************
 *
 * @file gap_hl_api.h
 *
 * @brief Header file - GAP Internal HL API
 *
 * Copyright (C) RivieraWaves 2009-2019
 ****************************************************************************************
 */

#ifndef GAP_HL_API_H_
#define GAP_HL_API_H_

/**
 ****************************************************************************************
 * @addtogroup GAP
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include <stdint.h>
#include "co_list.h"

/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */
/// Maximum timer duration in milliseconds (30s)
#define GAPC_SDT_DURATION_MAX   (30000)

/// Defer job timer and defer job clients
enum gapc_sdt_client
{
    #if (BLE_GAPC)
    /// SMP module
    GAPC_SDT_SMP,
    #endif // (BLE_GAPC)
    #if (BLE_L2CAP)
    /// L2CAP module
    GAPC_SDT_L2CAP,
    #endif // (BLE_L2CAP)
    #if (BLE_GATT)
    /// GATT Procedure module
    GAPC_SDT_GATT_PROC,
    /// GATT Bearer module
    GAPC_SDT_GATT_BEARER,
    #endif // (BLE_GATT)
    GAPC_SDT_MAX,
};

/// Link security status. This status represents the authentication/authorization/bonding levels of the connection
enum gapc_lk_sec_req
{
    /// Link is bonded
    GAPC_LK_BONDED,
    /// Link is Encrypted
    GAPC_LK_ENCRYPTED,
    /// Link Key or LTK Exchanged during pairing
    GAPC_LK_ENC_KEY_PRESENT,
};

/// Information source.
enum gapc_info_src
{
    /// Local info.
    GAPC_INFO_SRC_LOCAL,
    /// Peer info.
    GAPC_INFO_SRC_PEER,
    /// Maximum info source.
    GAPC_INFO_SRC_MAX
};
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

#if (GAPC_PRESENT)
/// Timer and defer job structure
typedef struct gapc_sdt
{
    /// Timer and defer job are put onto a queue
    co_list_hdr_t hdr;
    /// Information bit field (see enum #gapc_sdt_info_bf)
    uint32_t      info_bf;
} gapc_sdt_t;

/**
 ****************************************************************************************
 * @brief Callback definition of timer handler
 *
 * @param[in] p_hdl  Pointer of timer handle
 * @param[in] conidx Connection index
 ****************************************************************************************
 */
typedef void (*gapc_sdt_timer_handler_t)(gapc_sdt_t* p_hdl, uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Callback definition of defer job handler
 *
 * @param[in] p_hdl  Pointer of timer handle
 * @param[in] conidx Connection index
 * @param[in] dummy  Defer job dummy information
 ****************************************************************************************
 */
typedef void (*gapc_sdt_defer_handler_t)(gapc_sdt_t* p_hdl, uint8_t conidx, uint16_t dummy);
#endif // (GAPC_PRESENT)


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Get if Host has been properly setup
 *
 * @return True if setup succeed, False otherwise
 ****************************************************************************************
 */
bool gapm_is_configured(void);

/**
 * Get bit field of supported roles
 *
 * @return bit field which contains supported roles (see #role)
 */
uint8_t gapm_get_suppported_roles(void);


#if (!EMB_PRESENT)
/**
 ****************************************************************************************
 * @brief Get if Host is doing a reset
 *
 * @return True if reset on-going, False otherwise
 ****************************************************************************************
 */
bool gapm_is_doing_reset(void);
#endif // (!EMB_PRESENT)

#if (BLE_GATT)
/**
 ****************************************************************************************
 * @brief The Attribute database has been updated, Inform GATT service clients
 *
 * @param[in] start_hdl Service changed start handle
 * @param[in] end_hdl   Service changed end handle
 ****************************************************************************************
 */
void gapc_svc_db_updated(uint16_t start_hdl, uint16_t end_hdl);

/**
 ****************************************************************************************
 * @brief Check if Client database cache is considered out of sync.
 *
 * @param[in] conidx    Connection index
 * @param[in] code      Attribute operation code
 * @param[in] hdl       Attribute handle value impacted
 ****************************************************************************************
 */
bool gapc_svc_is_cli_out_of_sync(uint8_t conidx, uint8_t code, uint16_t hdl);

/**
 ****************************************************************************************
 *  Check if peer client supports Multiple notifications
 *
 *  @param[in] conidx    Connection index
 *
 * @return True if peer client supports multiple notification
 ****************************************************************************************
 */
bool gapc_svc_is_cli_mult_ntf_supported(uint8_t conidx);


/**
 ****************************************************************************************
 * @brief Inform application that no more ATT bearer are available onto the connection.
 *
 * @param[in] conidx    Connection index
 ****************************************************************************************
 */
void gapc_att_bearer_error_send(uint8_t conidx);
#endif// (BLE_GATT)


#if (GAPC_PRESENT)


/**
 ****************************************************************************************
 * @brief Check if current link support security requirements.
 *
 * @param[in] conidx  Connection index
 * @param[in] sec_req Link security requirement to test
 *
 * @return True if link requirement is supported, False else.
 ****************************************************************************************
 */
bool gapc_is_sec_set(uint8_t conidx, uint8_t sec_req);

/**
 ****************************************************************************************
 * @brief Retrieve Link Security level
 *
 * @param[in] conidx  Connection index
 *
 * @return Link Security level.
 ****************************************************************************************
 */
uint8_t gapc_lk_sec_lvl_get(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Retrieve the encryption key size of the connection
 *
 * @param[in] conidx Connection index
 *
 * @return encryption key size (size is 7 - 16 byte range)
 *
 ****************************************************************************************
 */
uint8_t gapc_enc_keysize_get(uint8_t conidx);


/**
 ****************************************************************************************
 * @brief Retrieve the current signature counter value
 *
 * @param[in] conidx Connection index
 * @param[in] src    Signature information source (see enum #gapc_info_src)
 *
 * @return Signature counter value
 ****************************************************************************************
 */
uint32_t gapc_bond_sign_count_get(uint8_t conidx, uint8_t src);

/**
 ****************************************************************************************
 * @brief Update the current signature counter value
 *
 * @param[in] conidx Connection index
 * @param[in] src    Signature information source (see enum #gapc_info_src)
 * @param[in] count  New Signature counter value
 ****************************************************************************************
 */
void gapc_bond_sign_count_set(uint8_t conidx, uint8_t src, uint32_t count);

/**
 ****************************************************************************************
 * @brief Retrieve the CSRK for signature generation / resolution
 *
 * @param[in] conidx Connection index
 * @param[in] src    Signature information source (see enum #gapc_info_src)
 *
 * @return CSRK value
 ****************************************************************************************
 */
const uint8_t* gapc_bond_csrk_get(uint8_t conidx, uint8_t src);

/**
 ****************************************************************************************
 * @brief Prepare Defer and Timer handler structure.
 *
 * @param[in] p_hdl     Pointer to Defer and Timer handler
 * @param[in] conidx  Connection index to return in defer function
 * @param[in] client_id Client indentifer of timer and defer feature. (see enum #gapc_sdt_client)
 *
 * @return Status of the function execution
 ****************************************************************************************
 */
uint16_t gapc_sdt_prepare(gapc_sdt_t* p_hdl, uint8_t conidx, uint8_t client_id);

/**
 ****************************************************************************************
 * @brief Request defer of a procedure to be executed in background
 *
 * @param[in] p_hdl   Pointer to Defer and Timer handler
 * @param[in] dummy   Dummy parameter to return in defer function
 *
 * @return Status of the function execution
 ****************************************************************************************
 */
uint16_t gapc_sdt_defer(gapc_sdt_t* p_hdl, uint16_t dummy);

/**
 ****************************************************************************************
 * @brief Request start or re-start a timer.
 *        The timer duration must not exceed GAPC_SDT_DURATION_MAX
 *
 * @param[in] p_hdl    Pointer to Defer and Timer handler
 * @param[in] duration Timer duration
 *
 * @return Status of the function execution
 ****************************************************************************************
 */
uint16_t gapc_sdt_timer_set(gapc_sdt_t* p_hdl, uint16_t duration);

/**
 ****************************************************************************************
 * @brief Stop a programmed timer / or cancel job execution defer.
 *
 * @param[in] p_hdl    Pointer to Defer and Timer handler
 *
 * @return Status of the function execution
 ****************************************************************************************
 */
uint16_t gapc_sdt_stop(gapc_sdt_t* p_hdl);
#endif // (GAPC_PRESENT)

/// @} GAPM

#endif // GAP_HL_API_H_
