/**
 ****************************************************************************************
 *
 * @file gapc.h
 *
 * @brief Generic Access Profile Controller - Native API .
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */


#ifndef _GAPC_H_
#define _GAPC_H_

/**
 ****************************************************************************************
 * @addtogroup GAPC_API Generic Access Profile Controller (GAPC)
 * @ingroup GAP_API
 * @brief  Generic Access Profile Controller.
 *
 * The GAP Controller module is responsible for providing an API to the application in
 * to perform GAP action related to a BLE connection (pairing, update parameters,
 * disconnect ...).
 *
 * @{
 * @}
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPC_ENUM_API Enumerations
 * @ingroup GAPC_API
 * @{
 * @}
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPC_STRUCT_API Structures and types
 * @ingroup GAPC_API
 * @{
 * @}
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPC_PROC_API Procedures Handlers
 * @ingroup GAPC_API
 * @{
 * @}
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPC_CON_REQ_API Connection establishment handling
 * @ingroup GAPC_API
 * @brief Function and callback to use in order to handle an \glos{LE} or \glos{BT} connection establishment
 * @{
 * @}
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPC_CON_UP_API Connection Update
 * @ingroup GAPC_API
 * @brief Function and callback to use to negotiate and apply new connection parameters
 * @{
 * @}
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gap.h"
#include "co_buf.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * DEFINES
 ****************************************************************************************
 */

/// @addtogroup GAPC_ENUM_API
/// @{

/// Option for PHY configuration
enum gapc_phy_option
{
    /// No preference for rate used when transmitting on the LE Coded PHY
    GAPC_PHY_OPT_LE_CODED_ALL_RATES     = 0,
    /// 500kbps rate preferred when transmitting on the LE Coded PHY
    GAPC_PHY_OPT_LE_CODED_500K_RATE     = 1,
    /// 125kbps  when transmitting on the LE Coded PHY
    GAPC_PHY_OPT_LE_CODED_125K_RATE     = 2,
};

/// Client bond information
enum gapc_cli_info
{
    /// Service changed indication enabled
    GAPC_CLI_SVC_CHANGED_IND_EN_BIT = (1 << 0),
    GAPC_CLI_SVC_CHANGED_IND_EN_POS = 0,
    /// Database updated since last connection
    GAPC_CLI_DB_UPDATED_BIT         = (1 << 1),
    GAPC_CLI_DB_UPDATED_POS         = 1,
};

/// Client supported features
enum gapc_cli_feat
{
    /// Robust Cache feature enabled
    GAPC_CLI_ROBUST_CACHE_EN_BIT    = (1 << 0),
    GAPC_CLI_ROBUST_CACHE_EN_POS    = 0,
    /// The client supports Enhanced ATT bearer
    GAPC_CLI_EATT_SUPPORTED_BIT     = (1 << 1),
    GAPC_CLI_EATT_SUPPORTED_POS     = 1,
    /// The client supports Multiple Handle Value Notifications
    GAPC_CLI_MULT_NTF_SUPPORTED_BIT = (1 << 2),
    GAPC_CLI_MULT_NTF_SUPPORTED_POS = 2,
};

/// Server supported features
enum gapc_srv_feat
{
    /// Enhanced ATT bearer supported
    GAPC_SRV_EATT_SUPPORTED_BIT     = (1 << 0),
    GAPC_SRV_EATT_SUPPORTED_POS     = 0,
};


/// Power Control Bit Field parameters.
enum gapc_pwr_ctrl_flags
{
    /// bit[0] - Sender is at the minimum supported power level
    GAPC_PWR_CTRL_MIN_BIT    = 0x01,
    GAPC_PWR_CTRL_MIN_POS    = 0,

    /// bit[1] - Sender is at the maximum supported power level
    GAPC_PWR_CTRL_MAX_BIT    = 0x02,
    GAPC_PWR_CTRL_MAX_POS    = 1,
};

/// Specify which PHY the Controller is specifying transmit power.
enum gapc_phy_pwr_value
{
    /// 1 Mbits PHY
    GAPC_PHY_PWR_1MBPS_VALUE    = 1,
    /// 2 Mbits PHY
    GAPC_PHY_PWR_2MBPS_VALUE    = 2,
    /// LE Coded PHY with S=8 data coding
    GAPC_PHY_PWR_S8_CODED_VALUE = 3,
    /// LE Coded PHY with S=2 data coding
    GAPC_PHY_PWR_S2_CODED_VALUE = 4,
};

/// Path Loss zones.
enum gapc_path_loss_zone
{
    /// Entered Low zone
    GAPC_PATH_LOSS_LOW           = 0,
    /// Entered Middle zone
    GAPC_PATH_LOSS_MID           = 1,
    /// Entered High zone
    GAPC_PATH_LOSS_HIGH          = 2,
};


/// @} GAPC_ENUM_API

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// @addtogroup GAPC_STRUCT_API
/// @{

/// Bond data
/*@TRACE*/
typedef struct gapc_bond_data_
{
    /// Local CSRK value
    gap_sec_key_t      local_csrk;
    /// Local signature counter value
    uint32_t           local_sign_counter;
    /// Remote CSRK value
    gap_sec_key_t      remote_csrk;
    /// Remote signature counter value
    uint32_t           remote_sign_counter;
    /// Pairing level (see enum #gap_pairing_lvl)
    uint8_t            pairing_lvl;
    /// Client bond data information (see enum #gapc_cli_info)
    uint8_t            cli_info;
    /// LTK or link key exchanged during pairing.
    bool               enc_key_present;
    /// Client supported features    (see enum #gapc_cli_feat)
    uint8_t            cli_feat;
    /// Peer GATT Service Start handle
    uint16_t           gatt_start_hdl;
    /// Peer GATT Service End Handle
    uint16_t           gatt_end_hdl;
    /// Peer Service Change value handle
    uint16_t           svc_chg_hdl;
    /// Server supported features    (see enum #gapc_srv_feat)
    uint8_t            srv_feat;
} gapc_bond_data_t;


/// Updated bond data information
/*@TRACE*/
typedef struct gapc_bond_data_updated
{
    /// Local SignCounter value
    uint32_t local_sign_counter;
    /// Peer SignCounter value
    uint32_t peer_sign_counter;
    /// Peer GATT Service Start handle
    uint16_t gatt_start_hdl;
    /// Peer GATT Service End Handle
    uint16_t gatt_end_hdl;
    /// Peer Service Change value handle
    uint16_t svc_chg_hdl;
    /// Client bond data information (see enum #gapc_cli_info)
    uint8_t  cli_info;
    /// Client supported features    (see enum #gapc_cli_feat)
    uint8_t  cli_feat;
    /// Server supported features    (see enum #gapc_srv_feat)
    uint8_t  srv_feat;
} gapc_bond_data_updated_t;

/// TX Power Report information
typedef struct gapc_tx_power_report
{
    /// PHY (see enum #gapc_phy_pwr_value)
    uint8_t phy;
    /// Transmit Power level (dBm)
    int8_t  tx_pwr;
    /// Transmit Power level flags (see enum #gapc_pwr_ctrl_flags)
    uint8_t flags;
    /// Delta (dB)
    int8_t  delta;
} gapc_tx_power_report_t;

/// information about IQ report
typedef struct gapc_iq_report_info
{
    /// Connection event counter
    uint16_t con_evt_cnt;
    /// Rx PHY  (see enum #gap_phy_val)
    uint8_t  rx_phy;
    /// Data channel index
    uint8_t  channel_idx;
    /// RSSI (in 0.1 dBm)
    int16_t  rssi;
    /// RSSI antenna ID
    uint8_t  rssi_antenna_id;
    /// CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us) (see enum #gap_cte_type)
    uint8_t  cte_type;
    /// Slot durations (1: 1us | 2: 2us)
    uint8_t  slot_dur;
    /// Packet status
    uint8_t  pkt_status;
} gapc_iq_report_info_t;

/// Peer version information
typedef struct gapc_version
{
    /// Manufacturer company identifier
    uint16_t company_id;
    /// LMP subversion
    uint16_t lmp_subversion;
    /// LMP version
    uint8_t  lmp_version;
} gapc_version_t;

/// @} GAPC_STRUCT_API


/*
 * CALLBACK DEFINITIONS
 ****************************************************************************************
 */

/// @addtogroup GAPC_PROC_API
/// @{

/**
 ****************************************************************************************
 * Callback executed when a procedure is completed.
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] status    Status of procedure execution (see enum #hl_err)
 *
 ****************************************************************************************
 */
typedef void (*gapc_proc_cmp_cb)(uint8_t conidx, uint32_t dummy, uint16_t status);

/// @} GAPC_PROC_API


/// @addtogroup GAPC_CON_REQ_API
/// @{

/// Callback structure required to handle BT-Classic or LE connection request events
typedef struct gapc_connection_req_cb
{
    /**
     ****************************************************************************************
     * @brief Callback executed once a connection has been established. The upper layer software shall
     *        execute #gapc_le_connection_cfm to enable ACL data reception and restore bond data.
     *
     * @note Mandatory for a connectable activity, optionnal otherwise
     *
     * @param[in] conidx        allocated connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] actv_idx      Activity Local identifier
     * @param[in] role          Connection role (see enum #gap_role)
     * @param[in] p_con_params  Pointer to connection parameters
     * @param[in] clk_accuracy  Master clock accuracy in PPM
     ****************************************************************************************
     */
    void (*le_connection_req)(uint8_t conidx, uint32_t dummy, uint8_t actv_idx, uint8_t role,
                              const gap_bdaddr_t* p_peer_addr, const gap_le_con_param_t* p_con_params,
                              uint8_t clk_accuracy);

    /**
     ****************************************************************************************
     * \if btdm
     * @brief Callback executed once a connection has been established. The upper layer software shall
     *        execute #gapc_bt_connection_cfm to enable ACL data reception and restore bond data.
     *
     * @note Mandatory callback if BT-Classic role is supported
     * \else
     * @brief Unused, only called if BT-Classic role is supported
     * \endif
     *
     * @param[in] conidx        allocated connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] actv_idx      Activity Local identifier
     * @param[in] is_initiator  True if connection initiator, False if connection responder
     * @param[in] p_peer_addr   Pointer to peer device address
     ****************************************************************************************
     */
    void (*bt_connection_req)(uint8_t conidx, uint32_t dummy, uint8_t actv_idx, bool is_initator,
                           const gap_addr_t* p_peer_addr);
} gapc_connection_req_cb_t;

/// @} GAPC_CON_REQ_API


/**
 ****************************************************************************************
 * @addtogroup GAPC_CON_INFO_API Connection information
 * @ingroup GAPC_API
 * @brief Function and callback to use to get information about active connection or answer peer device requests.
 * @{
 ****************************************************************************************
 */
/// Callback structure required to handle general connection events
typedef struct gapc_connection_info_cb
{
    /**
     ****************************************************************************************
     * Callback executed when link is disconnected
     *
     * @note Mandatory callback.
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] reason        Disconnection reason (see enum #hl_err)
     ****************************************************************************************
     */
    void (*disconnected)(uint8_t conidx, uint32_t dummy, uint16_t reason);

    /**
     ****************************************************************************************
     * Callback executed when connection bond data is updated.
     *
     * @note Optional callback.
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] p_data        Pointer to updated bond data
     ****************************************************************************************
     */
    void (*bond_data_updated)(uint8_t conidx, uint32_t dummy, const gapc_bond_data_updated_t* p_data);

    /**
     ****************************************************************************************
     * Callback executed when an authenticated payload timeout has been detected.
     * (no encrypted data received after a specific duration ; see LE-PING)
     *
     * @note Optional callback.
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     ****************************************************************************************
     */
    void (*auth_payload_timeout)(uint8_t conidx, uint32_t dummy);

    /**
     ****************************************************************************************
     * Callback executed when all ATT bearer are closed onto a connection
     *
     * @note Optional callback.
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     ****************************************************************************************
     */
    void (*no_more_att_bearer)(uint8_t conidx, uint32_t dummy);

    /**
     ****************************************************************************************
     * Callback executed when peer database hash value is read
     *
     * @note Optional callback.
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] handle        Handle value of HASH attribute
     * @param[in] p_hash        Pointer to the peer HASH attribute value
     ****************************************************************************************
     */
    void (*cli_hash_info)(uint8_t conidx, uint32_t dummy, uint16_t handle, const uint8_t* p_hash);

    /**
     ****************************************************************************************
     * Callback executed when peer request device name information
     * Upper layer SW shall call #gapc_info_name_get_cfm to provide expected information
     *
     * @note Mandatory callback if attribute present.
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] token         Token value that must be returned in confirmation
     * @param[in] offset        Device Name data offset
     * @param[in] max_length    Maximum name length to return (starting from offset)
     ****************************************************************************************
     */
    void (*name_get)(uint8_t conidx, uint32_t dummy, uint16_t token, uint16_t offset, uint16_t max_length);

    /**
     ****************************************************************************************
     * Callback executed when peer request appearance information
     * Upper layer SW shall call #gapc_info_appearance_get_cfm to provide expected information
     *
     * @note Mandatory callback if attribute present
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] token         Token value that must be returned in confirmation
     ****************************************************************************************
     */
    void (*appearance_get)(uint8_t conidx, uint32_t dummy, uint16_t token);

    /**
     ****************************************************************************************
     * Callback executed when peer request slave preferred connection parameters information
     * Upper layer SW shall call #gapc_info_slave_pref_param_get_cfm to provide expected information
     *
     * @note Mandatory callback if attribute present
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] token         Token value that must be returned in confirmation
     ****************************************************************************************
     */
    void (*slave_pref_param_get)(uint8_t conidx, uint32_t dummy, uint16_t token);

    /**
     ****************************************************************************************
     * Callback executed when peer request modification of device name information
     * Upper layer SW shall call #gapc_info_name_set_cfm to accept or reject request
     *
     * @note Optional callback - Automatically rejected if not set
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] token         Token value that must be returned in confirmation
     * @param[in] p_buf         Pointer to buffer that contains new device name
     ****************************************************************************************
     */
    void (*name_set)(uint8_t conidx, uint32_t dummy, uint16_t token, co_buf_t* p_buf);

    /**
     ****************************************************************************************
     * Callback executed when peer request modification of device appearance information
     * Upper layer SW shall call #gapc_info_appearance_set_cfm to accept or reject request
     *
     * @note Optional callback - Automatically rejected if not set
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] token         Token value that must be returned in confirmation
     * @param[in] appearance    New appearance value
     ****************************************************************************************
     */
    void (*appearance_set)(uint8_t conidx, uint32_t dummy, uint16_t token, uint16_t appearance);
} gapc_connection_info_cb_t;
/// @} GAPC_CON_INFO_API





/*
 * MACROS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/// @addtogroup GAPC_CON_UP_API Connection Update
/// @{
/**
 ****************************************************************************************
 * @brief Ask for BT classic or LE link disconnection.
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] reason    Disconnection error (@ref hl_err)
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_disconnect(uint8_t conidx, uint32_t dummy, uint16_t reason, gapc_proc_cmp_cb cmp_cb);
/// @} GAPC_CON_UP_API

/// @addtogroup GAPC_CON_INFO_API
/// @{
/**
 ****************************************************************************************
 * @brief Retrieve connection index from connection handle.
 *
 * @param[in] conhdl Connection handle
 *
 * @return Return found connection index, GAP_INVALID_CONIDX if not found.
 ****************************************************************************************
 */
uint8_t gapc_get_conidx(uint16_t conhdl);

/**
 ****************************************************************************************
 * @brief Retrieve connection handle from connection index.
 *
 * @param[in] conidx Connection index
 *
 * @return Return found connection handle, GAP_INVALID_CONHDL if not found.
 ****************************************************************************************
 */
uint16_t gapc_get_conhdl(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Check if a connection for provided connection index is established
 *
 * @param[in] conidx Connection index
 *
 * @return Return true if connection is established; false otherwise.
 ****************************************************************************************
 */
bool gapc_is_estab(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Retrieve connection role from connection index.
 *
 * @param[in] conidx Connection index
 *
 * @return Return found connection role (see enum #gap_role)
 ****************************************************************************************
 */
uint8_t gapc_get_role(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Retrieve BD address used by peer device on current link.
 *
 * @param[in] conidx Connection index
 *
 * @return Return found connection address
 ****************************************************************************************
 */
const gap_bdaddr_t* gapc_le_get_peer_bdaddr(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Retrieve BD address used by local device on current link.
 *
 * @param[in] conidx Connection index
 *
 * @return Return found connection address
 ****************************************************************************************
 */
const gap_bdaddr_t* gapc_le_get_local_bdaddr(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Get if peer device and local device are bonded
 *
 * @param[in] conidx    Connection index
 *
 * @return Return true if a bond exists with peer device, false otherwise
 ****************************************************************************************
 */
bool gapc_is_bonded(uint8_t conidx);

/**
 ****************************************************************************************
 * Callback executed when read attribute name procedure is completed.
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] status    Status of procedure execution (see enum #hl_err)
 * @param[in] handle    Attribute handle
 * @param[in] p_name    Pointer to buffer that contains peer device name
 *
 ****************************************************************************************
 */
typedef void (*gapc_read_device_name_cmp_cb)(uint8_t conidx, uint32_t dummy, uint16_t status, uint16_t handle,
                                             co_buf_t* p_name);

/**
 ****************************************************************************************
 * @brief Read peer device name characteristic present in attribute database
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_read_device_name_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_read_device_name(uint8_t conidx, uint32_t dummy, gapc_read_device_name_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * Callback executed when read attribute appearance procedure is completed.
 *
 * @param[in] conidx      Connection index
 * @param[in] dummy       Dummy parameter provided by upper layer application
 * @param[in] status      Status of procedure execution (see enum #hl_err)
 * @param[in] handle      Attribute handle
 * @param[in] appearance  Peer device appearance characteristic value
 *
 ****************************************************************************************
 */
typedef void (*gapc_read_appearance_cmp_cb)(uint8_t conidx, uint32_t dummy, uint16_t status, uint16_t handle,
                                            uint16_t appearance);

/**
 ****************************************************************************************
 * @brief Read peer device appearance characteristic present in attribute database
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_read_appearance_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_read_appearance(uint8_t conidx, uint32_t dummy, gapc_read_appearance_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * Callback executed when read attribute peripheral preferred parameters procedure is completed.
 *
 * @param[in] conidx      Connection index
 * @param[in] dummy       Dummy parameter provided by upper layer application
 * @param[in] status      Status of procedure execution (see enum #hl_err)
 * @param[in] handle      Attribute handle
 * @param[in] p_param     Pointer to read peer device slave preferred parameters characteristic value
 *
 ****************************************************************************************
 */
typedef void (*gapc_read_periph_pref_param_cmp_cb)(uint8_t conidx, uint32_t dummy, uint16_t status, uint16_t handle,
                                                  const gap_periph_pref_t* p_param);

/**
 ****************************************************************************************
 * @brief Read peer device peripheral preferred parameters characteristic present in attribute database
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_read_periph_pref_param_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_read_periph_pref_param(uint8_t conidx, uint32_t dummy, gapc_read_periph_pref_param_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * Callback executed when read central address resolution supported parameters procedure is completed.
 *
 * @param[in] conidx               Connection index
 * @param[in] dummy                Dummy parameter provided by upper layer application
 * @param[in] status               Status of procedure execution (see enum #hl_err)
 * @param[in] handle               Attribute handle
 * @param[in] central_addr_resol   Read peer central address resolution characteristic value
 *
 ****************************************************************************************
 */
typedef void (*gapc_read_central_addr_resol_supp_cmp_cb)(uint8_t conidx, uint32_t dummy, uint16_t status, uint16_t handle,
                                                         uint8_t central_addr_resol);

/**
 ****************************************************************************************
 * @brief Read peer device central address resolution supported characteristic present in attribute database
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_read_central_addr_resol_supp_cmp_cb
 *         callback execution
 ****************************************************************************************
 */
uint16_t gapc_read_central_addr_resol_supp(uint8_t conidx, uint32_t dummy, gapc_read_central_addr_resol_supp_cmp_cb cmp_cb);


/**
 ****************************************************************************************
 * Callback executed when read attribute database hash procedure is completed.
 *
 * @param[in] conidx      Connection index
 * @param[in] dummy       Dummy parameter provided by upper layer application
 * @param[in] status      Status of procedure execution (see enum #hl_err)
 * @param[in] handle      Attribute handle
 * @param[in] p_hash      Pointer to read peer device database hash characteristic value (128-bit value)
 *
 ****************************************************************************************
 */
typedef void (*gapc_read_database_hash_cmp_cb)(uint8_t conidx, uint32_t dummy, uint16_t status, uint16_t handle,
                                               const uint8_t* p_hash);

/**
 ****************************************************************************************
 * @brief Read peer device attribute database hash characteristic present in attribute database
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_read_database_hash_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_read_database_hash(uint8_t conidx, uint32_t dummy, gapc_read_database_hash_cmp_cb cmp_cb);


/**
 ****************************************************************************************
 * Callback executed when read attribute resolvable private address only procedure is completed.
 *
 * @param[in] conidx              Connection index
 * @param[in] dummy               Dummy parameter provided by upper layer application
 * @param[in] status              Status of procedure execution (see enum #hl_err)
 * @param[in] handle              Attribute handle
 * @param[in] rslv_priv_addr_only Peer device resolvable private address only characteristic value
 *
 ****************************************************************************************
 */
typedef void (*gapc_read_rslv_priv_addr_only_cmp_cb)(uint8_t conidx, uint32_t dummy, uint16_t status, uint16_t handle,
                                                    uint8_t rslv_priv_addr_only);

/**
 ****************************************************************************************
 * @brief Read peer device attribute resolvable private address only characteristic present in attribute database
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_read_rslv_priv_addr_only_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_read_rslv_priv_addr_only(uint8_t conidx, uint32_t dummy, gapc_read_rslv_priv_addr_only_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * Callback executed when get peer version procedure is completed.
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] status    Status of procedure execution (see enum #hl_err)
 * @param[in] p_version Pointer to peer version information
 *
 ****************************************************************************************
 */
typedef void (*gapc_get_peer_version_cmp_cb)(uint8_t conidx, uint32_t dummy, uint16_t status,
                                             const gapc_version_t* p_version);

/**
 ****************************************************************************************
 * @brief Read peer version information
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_get_peer_version_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_get_peer_version(uint8_t conidx, uint32_t dummy, gapc_get_peer_version_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * Callback executed when get RSSI value procedure is completed.
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] status    Status of procedure execution (see enum #hl_err)
 * @param[in] rssi      Latest measured RSSI value (in dBm)
 *
 ****************************************************************************************
 */
typedef void (*gapc_get_rssi_cmp_cb)(uint8_t conidx, uint32_t dummy, uint16_t status, int8_t rssi);

/**
 ****************************************************************************************
 * @brief Get latest measured RSSI value onto connection
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_get_rssi_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_get_rssi(uint8_t conidx, uint32_t dummy, gapc_get_rssi_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * Callback executed when get authenticated payload timeout value procedure is completed.
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] status    Status of procedure execution (see enum #hl_err)
 * @param[in] timeout   Authenticated payload timeout (10ms unit)
 *
 ****************************************************************************************
 */
typedef void (*gapc_get_auth_payload_to_cmp_cb)(uint8_t conidx, uint32_t dummy, uint16_t status, uint16_t timeout);

/**
 ****************************************************************************************
 * @brief Get configured authentication timeout value
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_get_auth_payload_to_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_get_auth_payload_to(uint8_t conidx, uint32_t dummy, gapc_get_auth_payload_to_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * @brief Provide name to peer device after in response of request received.
 *
 * @param[in] conidx          Connection index
 * @param[in] token           Token value provided in request indication
 * @param[in] status          Status code used to accept (GAP_ERR_NO_ERROR) or reject request (see enum #hl_err).
 * @param[in] complete_length Complete device name length
 * @param[in] length          Length of the value to transmit (should be less or equals requested maximum length)
 * @param[in] p_name          Pointer to array that contains device name starting from requested offset
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_info_name_get_cfm(uint8_t conidx, uint16_t token, uint16_t status, uint16_t complete_length,
                                uint8_t length, const uint8_t* p_name);

/**
 ****************************************************************************************
 * @brief Provide appearance to peer device after in response of request received.
 *
 * @param[in] conidx          Connection index
 * @param[in] token           Token value provided in request indication
 * @param[in] status          Status code used to accept (GAP_ERR_NO_ERROR) or reject request (see enum #hl_err).
 * @param[in] appearance      Appearance icon value
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_info_appearance_get_cfm(uint8_t conidx, uint16_t token, uint16_t status, uint16_t appearance);

/**
 ****************************************************************************************
 * @brief Provide slave preferred connection parameters to peer device in response of request received.
 *
 * @param[in] conidx          Connection index
 * @param[in] token           Token value provided in request indication
 * @param[in] status          Status code used to accept (GAP_ERR_NO_ERROR) or reject request (see enum #hl_err).
 * @param[in] pref            Slave preferred connection parameters
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_info_slave_pref_param_get_cfm(uint8_t conidx, uint16_t token, uint16_t status, gap_periph_pref_t pref);

/**
 ****************************************************************************************
 * @brief Inform if name modification in response of request received is accepted or rejected.
 *
 * @param[in] conidx          Connection index
 * @param[in] token           Token value provided in request indication
 * @param[in] status          Status code used to accept (GAP_ERR_NO_ERROR) or reject request (see enum #hl_err).
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_info_name_set_cfm(uint8_t conidx, uint16_t token, uint16_t status);

/**
 ****************************************************************************************
 * @brief Inform if appearance modification in response of request received is accepted or rejected.
 *
 * @param[in] conidx          Connection index
 * @param[in] token           Token value provided in request indication
 * @param[in] status          Status code used to accept (GAP_ERR_NO_ERROR) or reject request (see enum #hl_err).
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_info_appearance_set_cfm(uint8_t conidx, uint16_t token, uint16_t status);

/**
 ****************************************************************************************
 * @brief Enable usage of supported client features
 *
 * @param[in] conidx          Connection index
 * @param[in] dummy           Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] cmp_cb          Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_client_features_enable(uint8_t conidx, uint32_t dummy, gapc_proc_cmp_cb cmp_cb);
/// @} GAPC_CON_INFO_API

/// @addtogroup GAPC_CON_UP_API
/// @{

/**
 ****************************************************************************************
 * @brief Function used to set authenticated payload timeout (BT-Ping / LE Ping).
 *        Feature used to ensure that encrypted packet are exchanged before timer expiration.
 *
 * @param[in] conidx        Connection index
 * @param[in] dummy         Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] timeout       Authenticated payload timeout (N*10ms)
 * @param[in] cmp_cb        Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_set_auth_payload_timeout(uint8_t conidx, uint32_t dummy, uint16_t timeout, gapc_proc_cmp_cb cmp_cb);

/// @} GAPC_CON_UP_API

#endif /* _GAPC_H_ */
