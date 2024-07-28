/**
 ****************************************************************************************
 *
 * @file gapc_le_con.h
 *
 * @brief Generic Access Profile Controller Internal Header - BT Classic Connection
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */
#ifndef _GAPC_BT_CON_H_
#define _GAPC_BT_CON_H_

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
#if (BT_HOST_PRESENT)
#include "gapc_int.h"

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DECLARATIONS
 ****************************************************************************************
 */


/// GAP controller connection variable structure.
typedef struct gapc_bt_con
{
    /// base connection information
    gapc_con_t          hdr;
    /// peer BD Address used for the link that should be kept
    gap_addr_t          peer_addr;
    /// Keep some data useful for pairing
    /// TODO temporary solution to keep BT-Classic pairing data (OOB / pin code). will be change later to remove
    /// capability to perform simultaneous pairing on several transport layers (BR/EDR or LE)
    uint8_t             bt_pairing_data[64];
    /// pairing state
    uint8_t             sec_state;
    /// Store IOCAP to automatically accept numeric value if Display Yes/No not present
    uint8_t             iocap;
    /// True if a pairing is bonding devices, false otherwise -- TODO use information bit field
    bool                do_bonding;
    /// True if a pairing procedure is initiated -- TODO use information bit field
    bool                do_pairing;
    /// True is an encryption is on-going -- TODO use information bit field
    bool                do_encrypt;
    /// True if local or peer device try to perform dedicated bond (no encryption)
    bool                do_dedicated_bond;
    /// Used to know if link key received
    bool                is_link_key_received;
    /// True to indicate that authentication information must be triggered to application -- TODO use information bit field
    bool                indicate_auth_info;
} gapc_bt_con_t;


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve BT Classic Connection environment
 *
 * @param[in] conidx    Connection index
 *
 * @return Pointer to connection structure if BT classic connection
 ****************************************************************************************
 */
__INLINE gapc_bt_con_t* gapc_bt_get_con_env(uint8_t conidx)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    // Only return BT classic environment
    return ((p_con != NULL) && !GETB(p_con->info_bf, GAPC_LE_CON_TYPE) ? (gapc_bt_con_t*) p_con : NULL);
}

/**
 ****************************************************************************************
 * @brief Retrieve BT connection pointer from connection handle
 *
 * @param[in]  conhdl   Connection handle
 * @param[out] p_conidx Connection index
 *
 * @return Pointer to connection object, NULL if not found
 ****************************************************************************************
 */
__INLINE gapc_bt_con_t* gapc_bt_get_con_env_from_conhdl(uint16_t conhdl, uint8_t* p_conidx)
{
    *p_conidx = gapc_get_conidx(conhdl);
    return gapc_bt_get_con_env(*p_conidx);
}


/// Provide passkey for a BT Classic pairing
uint16_t gapc_bt_pairing_provide_passkey(uint8_t conidx, bool accept, uint32_t passkey);
/// Provide number comparison for a BT Classic pairing
uint16_t gapc_bt_pairing_numeric_compare_rsp(uint8_t conidx, bool accept);

/// Get if an encryption is on-going
bool gapc_is_doing_encryption(uint8_t conidx);

#endif // (BT_HOST_PRESENT)
/// @} GAPC

#endif /* _GAPC_BT_CON_H_ */
