/**
 ****************************************************************************************
 *
 * @file gap_int.h
 *
 * @brief Generic Access Profile Internal Header - Shared by GAP manager and controller only
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */


#ifndef _GAP_INT_H_
#define _GAP_INT_H_

/**
 ****************************************************************************************
 * @addtogroup GAP_INT Generic Access Profile Internal
 * @ingroup GAP
 * @brief defines for internal GAP usage
 *
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include "gap.h"
#include "gapm.h"
#include "gapc.h"
#include "co_bt.h"

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * INTERNAL API TYPES
 ****************************************************************************************
 */


/*
 * MACROS
 ****************************************************************************************
 */



/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

#if (BLE_GAPC)
/// OOB data to keep for a following pairing that uses OOB data
typedef struct gapm_oob_data
{
    /// Value of the public key used to compute confirmation data
    public_key_t pub_key;
    //// OOB Data generated
    gap_oob_t data;
} gapm_oob_data_t;
#endif // (BLE_GAPC)

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve device name
 *
 * @param[out] p_name_len   Pointer to name length
 *
 * @return Pointer to device, NULL if not set by application
 ****************************************************************************************
 */
const uint8_t* gapm_get_name(uint8_t* p_name_len);

#if (GAPC_PRESENT)


/**
 ****************************************************************************************
 * @brief Set callbacks required for communication with application
 *
 * @param[in] p_cbs   Pointer to callbacks
 ****************************************************************************************
 */
void gapc_set_callbacks(const gapm_callbacks_t* p_cbs);


/**
 ****************************************************************************************
 * @brief A connection has been disconnected, uninitialized Controller task.
 *
 * unregister connection, and destroy environment variable allocated for current connection.
 *
 * @param[in] conidx  Connection index
 ****************************************************************************************
 */
void gapc_con_cleanup(uint8_t conidx);

#if (BT_HOST_PRESENT)
/**
 ****************************************************************************************
 * @brief A BT Classic connection has been created, initialize HL connection manager
 *
 * This function find first available task index available for new connection.
 * It triggers also connection event to task that has requested the connection.
 *
 * @param[in]  actv_idx          Activity index used to create connection
 * @param[in]  dummy             Dummy parameter provided by upper layer software
 * @param[in]  is_initiator      True if connection initiator, connection responder otherwise
 * @param[in]  conhdl            Connection handle
 * @param[in]  p_peer_addr       Pointer to peer device address
 * @param[in]  enc_en            True if link is encrypted at establishment, False otherwise
 * @param[out] p_conidx          Pointer to allocated connection index
 *
 * @return Status of LE connection creation (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapc_bt_con_create(uint8_t actv_idx, uint32_t dummy, bool is_initiator, uint16_t conhdl,
                            const gap_addr_t* p_peer_addr, bool enc_en, uint8_t* p_conidx);
#endif // (BT_HOST_PRESENT)

#if (BLE_GAPC)
/**
 ****************************************************************************************
 * @brief A LE connection has been created, initialize HL connection manager
 *
 * This function find first available task index available for new connection.
 * It triggers also connection event to task that has requested the connection.
 *
 * @param[in]  actv_idx          Activity index used to create connection
 * @param[in]  dummy             Dummy parameter from Upper layer SW
 * @param[in]  is_name_discovery True if a name discovery procedure handle connection establishment, false otherwise
 * @param[in]  p_local_addr      Pointer to local BD Address
 * @param[in]  p_evt             HCI event that contains connection information
 * @param[out] p_conidx          Pointer to allocated connection index
 *
 * @return Status of LE connection creation (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapc_le_con_create(uint8_t actv_idx, uint32_t dummy, bool is_name_discovery,
                            const gap_bdaddr_t* p_local_addr, const struct hci_le_enh_con_cmp_evt* p_evt,
                            uint8_t* p_conidx);

/**
 ****************************************************************************************
 * @brief Set periodic sync activity index related to current connection
 *
 * @param[in] conidx    Connection index
 * @param[in] actv_idx  Periodic sync activity index
 *
 ****************************************************************************************
 */
void gapc_past_actv_idx_set(uint8_t conidx, uint8_t actv_idx);

/**
 ****************************************************************************************
 * @brief Get periodic sync activity index related to current connection
 *
 * @param[in] conidx    Connection index
 *
 * @return Periodic sync activity index
 ****************************************************************************************
 */
uint8_t gapc_past_actv_idx_get(uint8_t conidx);
#endif // (BLE_GAPC)

/**
 ****************************************************************************************
 * @brief Clean-up timers and deferred functions related to a specific connection.
 *
 * @param[in] conidx  Connection index
 ****************************************************************************************
 */
void gapc_sdt_cleanup(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief A link has been disconnected, clean-up host stack for this connection.
 *
 * @param[in] conidx     Connection Index
 * @param[in] reason     Reason of the disconnection
 *
 ****************************************************************************************
 */
void gapm_con_cleanup(uint8_t conidx, uint8_t reason);

#if (BLE_GAPC)
/**
 ****************************************************************************************
 * Retrieve OOB data generated if application requests it.
 *
 * @return OOB data generated + Corresponding Local Public Key, NULL if no OOB data available
 ****************************************************************************************
 */
const gapm_oob_data_t* gapm_oob_data_get(void);

/**
 ****************************************************************************************
 * Release if present OOB data generated for OOB pairing
 ****************************************************************************************
 */
void gapm_oob_data_release(void);

/**
 ****************************************************************************************
 * @brief Setup internal GAP/GATT service
 *
 * @param[in] att_cfg               Attribute database configuration (see enum #gapm_att_cfg_flag)
 * @param[in] gapc_svc_start_hdl     GAP service start handle
 * @param[in] gatt_svc_start_hdl    GATT service start handle
 * @param[in] central_res_en        True to enable central resolution characteristic
 *
 * @return Execution status (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapc_svc_setup(uint16_t att_cfg, uint16_t gapc_svc_start_hdl, uint16_t gatt_svc_start_hdl, bool central_res_en);

#if (BLE_GATT_CLI)
/**
 ****************************************************************************************
 * @brief Setup internal GAP/GATT client
 *
 * @param[in] att_cfg Attribute database configuration (see enum #gapm_att_cfg_flag)
 *
 * @return Execution status (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapc_cli_setup(uint16_t att_cfg);
#endif // (BLE_GATT_CLI)
#endif // (BLE_GAPC)
#endif // (GAPC_PRESENT)


/**
 ****************************************************************************************
 * @brief Retrieve if controller privacy is enabled
 ****************************************************************************************
 */
uint8_t gapm_is_controler_privacy_enabled(void);

/**
 ****************************************************************************************
 * @brief Retrieve device identity key.
 *
 * @return Device Identity Key
 ****************************************************************************************
 */
const gap_sec_key_t* gapm_get_irk(void);


/**
 ****************************************************************************************
 * @brief Get information about a non connected activity
 *
 * @param[in] actv_idx          Activity Identifier
 * @param[in] p_act_type        Activity Type (enum gapm_actv_type)
 * @param[in] p_act_sub_type    Activity Sub Type, depends on activity
 *
 * @return GAP_ERR_NO_ERROR if activity exists else GAP_ERR_COMMAND_DISALLOWED
 ****************************************************************************************
 */
uint16_t gapm_actv_info_get(uint8_t actv_idx, uint8_t* p_act_type, uint8_t* p_act_sub_type);

/**
 ****************************************************************************************
 * @brief Retrieve if Legacy pairing is supported on local device
 *
 * @return True if legacy pairing is supported
 ****************************************************************************************
 */
bool gapm_is_legacy_pairing_supp(void);


/**
 ****************************************************************************************
  * @brief Retrieve if Secure Connection pairing is supported on local device
 *
 * @return True if Secure Connection pairing is supported
 ****************************************************************************************
 */
bool gapm_is_sec_con_pairing_supp(void);


/// @} GAP_INT

#endif /* _GAP_INT_H_ */
