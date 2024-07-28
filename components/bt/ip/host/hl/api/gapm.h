/**
 ****************************************************************************************
 *
 * @file gapm.h
 *
 * @brief Generic Access Profile Manager - Native API .
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */


#ifndef _GAPM_H_
#define _GAPM_H_

/**
 ****************************************************************************************
 * @addtogroup GAPM_API Generic Access Profile Manager (GAPM)
 * @ingroup GAP_API
 * @brief Generic Access Profile Manager.
 *
 * The GAP Manager module is responsible for providing an API to the application in order
 * to manage all non connected stuff such as configuring device to go in desired mode
 * (discoverable, connectable, etc.) and perform required actions (scanning, connection,
 * etc.). GAP Manager is also responsible of managing GAP Controller state according to
 * corresponding BLE connection states.
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_ENUM_API Enumerations
 * @ingroup GAPM_API
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_STRUCT_API Structures and types
 * @ingroup GAPM_API
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_CONFIG_API Configuration
 * @ingroup GAPM_API
 * @brief Set of functions used to configure the device
 *
 * Prior to starting any activities or load profiles, device configuration must be setup using
 * #gapm_set_dev_config.
 *
 * Once configuration is done, it's recommended to load profiles before starting any activities.
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_PROC_API Procedures
 *
 * @ingroup GAPM_API
 *
 * @brief Procedure interface and helpful functions
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_ACTV_API Non-connected activities
 *
 * @ingroup GAPM_API
 *
 * @brief A GAPM activity is a non-connected activity.
 *
 * All activities can be stopped and deleted using #gapm_actv_stop and #gapm_actv_delete
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_ACTV_ITF_API Default interface
 * @ingroup GAPM_ACTV_API
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "gap.h"

#include <stdbool.h>
#include "co_buf.h"

/*
 * DEFINES
 ****************************************************************************************
 */



/// @addtogroup GAPM_ENUM_API
/// @{

/// Automatic packet type selection
#define GAPM_PAGE_AUTO_PKT_TYPE_SEL             (0xFFFF)

/// Privacy configuration
enum gapm_priv_cfg
{
    /// Indicate if identity address is a public (0) or static private random (1) address
    GAPM_PRIV_CFG_PRIV_ADDR_BIT = (1 << 0),
    GAPM_PRIV_CFG_PRIV_ADDR_POS = 0,
    /// Reserved
    GAPM_PRIV_CFG_RSVD_BIT      = (1 << 1),
    GAPM_PRIV_CFG_RSVD_BIT_POS  = 1,
    /// Indicate if controller privacy is enabled
    GAPM_PRIV_CFG_PRIV_EN_BIT   = (1 << 2),
    GAPM_PRIV_CFG_PRIV_EN_POS   = 2,
};

/// Activity procedure type
enum gapm_actv_proc_id
{
    /// Create a new activity
    GAPM_ACTV_CREATE,
    /// Start activity
    GAPM_ACTV_START,
    /// Stop activity
    GAPM_ACTV_STOP,
    /// Delete activity
    GAPM_ACTV_DELETE,
    /// Set advertising data
    GAPM_ACTV_SET_ADV_DATA,
    /// Set scan response Data
    GAPM_ACTV_SET_SCAN_RSP_DATA,
    ///  Set periodic advertising data
    GAPM_ACTV_SET_PERIOD_ADV_DATA,
    /// Control CTE transmission in periodic ADV activity.
    GAPM_ACTV_PERIOD_ADV_CTE_TX_CTRL,
    /// Control periodic advertising report reception
    GAPM_ACTV_PERIOD_REPORT_CTRL,
    /// Control IQ Report (manage sampling configuration)
    GAPM_ACTV_PERIOD_IQ_REPORT_CTRL,
};

/// Device Attribute write permission requirement
enum gapm_write_att_perm
{
    /// Disable write access
    GAPM_WRITE_DISABLE     = 0,
    /// Enable write access - no authentication required
    GAPM_WRITE_NO_AUTH     = 1,
    /// Write access requires unauthenticated link
    GAPM_WRITE_UNAUTH      = 2,
    /// Write access requires authenticated link
    GAPM_WRITE_AUTH        = 3,
    /// Write access requires secure connected link
    GAPM_WRITE_SEC_CON     = 4
};

/// Attribute database configuration
/// @verbatim
///   15   14   13   12   11      10      9    8    7    6    5    4    3    2    1    0
/// +----+----+----+----+----+---------+----+----+----+----+----+----+----+----+----+----+
/// |           RFU     |RPAO|BOND_INFO|EATT| FE |MTU |PCP |   APP_PERM   |   NAME_PERM  |
/// +----+----+----+----+----+---------+----+----+----+----+----+----+----+----+----+----+
/// @endverbatim
/// - Bit [0-2]  : Device Name write permission requirements for peer device (#gapm_write_att_perm)
/// - Bit [3-5]  : Device Appearance write permission requirements for peer device (#gapm_write_att_perm)
/// - Bit [6]    : Slave Preferred Connection Parameters present
/// - Bit [7]    : Disable automatic MTU exchange at connection establishment (on legacy ATT bearer)
/// - Bit [8]    : Disable automatic client feature enable setup at connection establishment
/// - Bit [9]    : Disable automatic establishment of Enhanced ATT bearers
/// - Bit [10]   : Enable presence of Resolvable private address only.
/// - Bit [11-14]: Reserved
/// - Bit [15]   : Trigger bond information to application even if devices are not bonded
enum gapm_att_cfg_flag
{
    /// Device Name write permission requirements for peer device (#gapm_write_att_perm)
    GAPM_ATT_NAME_PERM_MASK                  = 0x0007,
    GAPM_ATT_NAME_PERM_LSB                   = 0,
    /// Device Appearance write permission requirements for peer device (#gapm_write_att_perm)
    GAPM_ATT_APPEARENCE_PERM_MASK            = 0x0038,
    GAPM_ATT_APPEARENCE_PERM_LSB             = 3,
    /// Slave Preferred Connection Parameters present in GAP attribute database.
    GAPM_ATT_SLV_PREF_CON_PAR_EN_MASK        = 0x0040,
    GAPM_ATT_SLV_PREF_CON_PAR_EN_LSB         = 6,
    /// Disable automatic MTU exchange at connection establishment (on legacy ATT bearer)
    GAPM_ATT_CLI_DIS_AUTO_MTU_EXCH_MASK      = 0x0080,
    GAPM_ATT_CLI_DIS_AUTO_MTU_EXCH_LSB       = 7,
    /// Disable automatic client feature enable setup at connection establishment
    GAPM_ATT_CLI_DIS_AUTO_FEAT_EN_MASK       = 0x0100,
    GAPM_ATT_CLI_DIS_AUTO_FEAT_EN_LSB        = 8,
    /// Disable automatic establishment of Enhanced ATT bearers
    GAPM_ATT_CLI_DIS_AUTO_EATT_MASK          = 0x0200,
    GAPM_ATT_CLI_DIS_AUTO_EATT_LSB           = 9,
    /// Enable presence of Resolvable private address only.
    /// This means that after a bond, device must only use resolvable private address
    GAPM_ATT_RSLV_PRIV_ADDR_ONLY_MASK        = 0x0400,
    GAPM_ATT_RSLV_PRIV_ADDR_ONLY_LSB         = 10,
    /// Trigger bond information to application even if devices are not bonded
    GAPM_DBG_BOND_INFO_TRIGGER_BIT           = 0x8000,
    GAPM_DBG_BOND_INFO_TRIGGER_POS           = 15,
};

/// Pairing mode authorized on the device
/// @verbatim
///    7    6    5    4    3    2    1    0
/// +----+----+----+----+----+----+----+----+
/// |               RFU           | SCP| LP |
/// +----+----+----+----+----+----+----+----+
/// @endverbatim
enum gapm_pairing_mode
{
    /// No pairing authorized
    GAPM_PAIRING_DISABLE  = 0,
    /// Legacy pairing Authorized
    GAPM_PAIRING_LEGACY   = (1 << 0),
    /// Secure Connection pairing Authorized
    GAPM_PAIRING_SEC_CON  = (1 << 1),
};

/// Type of activities that can be created
/*@TRACE*/
enum gapm_actv_type
{
    // LE - Activities
    /// Advertising activity
    GAPM_ACTV_TYPE_ADV = 0,
    /// Scanning activity
    GAPM_ACTV_TYPE_SCAN,
    /// Initiating activity
    GAPM_ACTV_TYPE_INIT,
    /// Periodic synchronization activity
    GAPM_ACTV_TYPE_PER_SYNC,

    // BT Classic - Activities
    /// Inquiry activity
    GAPM_ACTV_TYPE_INQUIRY,
    /// Inquiry scan activity
    GAPM_ACTV_TYPE_INQUIRY_SCAN,
    /// Page activity
    GAPM_ACTV_TYPE_PAGE,
    /// Page scan activity
    GAPM_ACTV_TYPE_PAGE_SCAN,

    // LE -Test Mode
    /// TX Test Mode
    GAPM_ACTV_TYPE_TX_TEST,
    /// RX Test Mode
    GAPM_ACTV_TYPE_RX_TEST,
};


/// Link Policy Bit Field
enum gapm_link_policy_bf
{
    /// Role Switch enabled
    GAPM_ROLE_SWITCH_ENABLE_BIT  = (1 << 0),
    /// Hold Mode enabled
    GAPM_HOLD_MODE_ENABLE_BIT    = (1 << 1),
    /// Sniff mode enabled
    GAPM_SNIFF_MODE_ENABLE_BIT   = (1 << 2),
};


/// @} GAPM_ENUM_API

/*
 * TYPE DEFINITION
 ****************************************************************************************
 */

typedef struct gapc_connection_req_cb gapc_connection_req_cb_t;
typedef struct gapc_security_cb gapc_security_cb_t;
typedef struct gapc_connection_info_cb gapc_connection_info_cb_t;
typedef struct gapc_le_config_cb gapc_le_config_cb_t;

/// @addtogroup GAPM_STRUCT_API
/// @{

/// Set device configuration command
/*@TRACE*/
typedef struct gapm_config
{
    /// Device Role: Central, Peripheral, Observer, Broadcaster or All roles (see #gap_role enumeration)
    uint8_t         role;

    // -------------- Security Config ------------------------------------
    /// Pairing mode authorized (see enum #gapm_pairing_mode)
    uint8_t         pairing_mode;

    // -------------- Privacy Config -------------------------------------
    /// Duration before regenerate device address when privacy is enabled. - in seconds
    uint16_t        renew_dur;
    /// Provided own static private random address
    gap_addr_t      addr;
    /// Device IRK used for resolvable random BD address generation (LSB first)
    gap_sec_key_t   irk;
    /// Privacy configuration bit field (see enum #gapm_priv_cfg for bit signification)
    uint8_t         privacy_cfg;

    // -------------- ATT Database Config --------------------------------
    /// GAP service start handle
    uint16_t        gap_start_hdl;
    /// GATT service start handle
    uint16_t        gatt_start_hdl;
    /// Attribute database configuration (see enum #gapm_att_cfg_flag)
    uint16_t        att_cfg;

    // -------------- LE Data Length Extension ---------------------------
    ///Suggested value for the Controller's maximum transmitted number of payload octets to be used
    uint16_t        sugg_max_tx_octets;
    ///Suggested value for the Controller's maximum packet transmission time to be used
    uint16_t        sugg_max_tx_time;

    // ------------------ LE PHY Management  -----------------------------
    /// Preferred LE PHY for data transmission (see enum #gap_phy)
    uint8_t         tx_pref_phy;
    /// Preferred LE PHY for data reception (see enum #gap_phy)
    uint8_t         rx_pref_phy;

    // ------------------ Radio Configuration ----------------------------
    /// RF TX Path Compensation value (from -128dB to 128dB, unit is 0.1dB)
    uint16_t        tx_path_comp;
    /// RF RX Path Compensation value (from -128dB to 128dB, unit is 0.1dB)
    uint16_t        rx_path_comp;

    // ------------------ BT classic configuration ----------------------
    /// Bluetooth Class of device
    uint32_t        class_of_device;
    /// Default link policy (see enum #gapm_link_policy_bf)
    uint16_t        dflt_link_policy;
    /// True if Secure simple pairing enabled, False otherwise
    bool            ssp_enable;
} gapm_config_t;

/// List of callbacks that will handle GAP events
typedef struct gapm_callbacks
{
    /// Connection request event callback functions provided by upper layer software
    /// Mandatory if #GAP_ROLE_CENTRAL or #GAP_ROLE_PERIPHERAL or  #GAP_ROLE_BT_CLASSIC is supported
    const gapc_connection_req_cb_t*  p_con_req_cbs;
    /// Security event callback functions provided by upper layer software
    /// Mandatory if #GAP_ROLE_CENTRAL or #GAP_ROLE_PERIPHERAL or  #GAP_ROLE_BT_CLASSIC is supported
    const gapc_security_cb_t*        p_sec_cbs;
    /// Connection event callback functions provided by upper layer software
    /// Mandatory if #GAP_ROLE_CENTRAL or #GAP_ROLE_PERIPHERAL or  #GAP_ROLE_BT_CLASSIC is supported
    const gapc_connection_info_cb_t* p_info_cbs;
    /// LE Connection configuration event callback functions provided by upper layer software
    /// Mandatory if #GAP_ROLE_CENTRAL or #GAP_ROLE_PERIPHERAL
    const gapc_le_config_cb_t*       p_le_config_cbs;
} gapm_callbacks_t;

/// Device SW/HW version information
typedef struct gapm_version
{
    /// HCI version
    uint8_t  hci_ver;
    /// LMP version
    uint8_t  lmp_ver;
    /// Host version
    uint8_t  host_ver;
    /// HCI revision
    uint16_t hci_subver;
    /// LMP subversion
    uint16_t lmp_subver;
    /// Host revision
    uint16_t host_subver;
    /// Manufacturer name
    uint16_t manuf_name;
} gapm_version_t;

/// @} GAPM_STRUCT_API
/*
 * INTERFACES
 ****************************************************************************************
 */

/*
 * RESULT CALLBACK FUNCTIONS
 ****************************************************************************************
 */

/// @addtogroup GAPM_PROC_API
/// @{

/**
 ****************************************************************************************
 * Callback executed when a procedure is completed.
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] status    Status of procedure execution (see enum #hl_err)
 *
 ****************************************************************************************
 */
typedef void (*gapm_proc_cmp_cb)(uint32_t dummy, uint16_t status);

/// @} GAPM_PROC_API

/**
 ****************************************************************************************
 * @addtogroup GAPM_INFO_API Device Information
 * @ingroup GAPM_API
 * @brief Set of functions used to retrieve device information
 * @{
 ****************************************************************************************
 */

/**
 ***************************************************************************************
 * @brief Function executed when procedure execution is over.
 *
 * @param[in] dummy         Dummy parameter provided by upper layer application
 * @param[in] status        Procedure execution status  (see enum #hl_err)
 * @param[in] p_version     Pointer to local device version
 *                          (NULL if status != GAP_ERR_NO_ERROR)
 ***************************************************************************************
 */
typedef void (*gapm_version_cb)(uint32_t dummy, uint16_t status, const gapm_version_t* p_version);

/// @} GAPM_INFO_API

/// @addtogroup GAPM_ACTV_ITF_API
/// @{

/// Callback structure required to create an activity
typedef struct gapm_actv_cb
{
    /**
     ****************************************************************************************
     * Callback executed when a procedure is completed.
     *
     * @note Mandatory callback. Shall be set to a valid callback
     *
     * @param[in] dummy     Dummy parameter provided by upper layer application
     * @param[in] proc_id   Procedure identifier (see enum #gapm_actv_proc_id)
     * @param[in] actv_idx  Activity local index
     * @param[in] status    Status of procedure execution (see enum #hl_err)
     *
     ****************************************************************************************
     */
    void (*proc_cmp)(uint32_t dummy, uint8_t proc_id, uint8_t actv_idx, uint16_t status);


    /**
     ****************************************************************************************
     * Callback executed when a procedure is completed.
     *
     * @note Mandatory callback. Shall be set to a valid callback
     *
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] actv_idx      Activity local index
     * @param[in] reason        Activity stop reason (see enum #hl_err)
     ****************************************************************************************
     */
    void (*stopped)(uint32_t dummy, uint8_t actv_idx, uint16_t reason);
} gapm_actv_cb_t;

/// @} GAPM_ACTV_ITF_API

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/// @addtogroup GAPM_PROC_API
/// @{

/**
 ****************************************************************************************
 * @brief Generate a new token for any HL procedures
 *
 * @return New Generated token
 ***************************************************************************************
 */
uint16_t gapm_token_id_get(void);

/// @} GAPM_PROC_API


/// @addtogroup GAPM_CONFIG_API
/// @{

/**
 ***************************************************************************************
 * @brief Optional and is present only for testing purpose.
 *
 * It's used to stop all device activity, but new activity cannot be started before
 * setting device configuration (#gapm_set_dev_config)
 *
 * This will initialize the RW-BLE Host stack - rearrange to default settings the ATT, GAP, GATT, L2CAP and SMP blocks.
 * Furthermore, this will cause the host to send a reset command down to the link layer part.
 *
 * Platform reset: Use platform mechanism to reset hardware.
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_proc_cmp_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_reset(uint32_t dummy, gapm_proc_cmp_cb cmp_cb);

/**
 ***************************************************************************************
 * @brief Setup initial device configuration
 *
 * Set the device configuration such as:
 *     - Device role
 *     - Manage device address type: Public, Private static or Generated for Privacy
 *     - Internal IRK used to generate resolvable random address
 *     - Set Internal GAP / GATT service start
 *     - Set specific write permissions on the appearance and name attributes in internal GAP database.
 *     - Manage presence of some attribute.
 *     - Configure Data Length Extension features
 *
 * The set device configuration first resets the device to close all active link and configured profiles.
 * This command must be sent before adding profiles and start air activities.
 *
 * Application shall wait #gapm_proc_cmp_cb callback execution before starting any activities
 *
 * example:
 * \snippet{lineno} app.c APP_SET_DEV_CFG
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] p_cfg     Pointer to device configuration.
 * @param[in] p_cbs     Pointer to callbacks that handles events
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_proc_cmp_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_set_dev_config(uint32_t dummy, const gapm_config_t* p_cfg, const gapm_callbacks_t* p_cbs, gapm_proc_cmp_cb cmp_cb);

/**
 ***************************************************************************************
 * @brief Set device name
 *
 * Device name pointer life cycle must be handled by application, it shall be valid until a name modification or a reset
 * is performed.
 *
 * Device name is set for both BT Classic and LE.
 *
 * Application should wait #gapm_proc_cmp_cb callback execution before starting a new procedure
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] name_len  Length of the name array
 * @param[in] p_name    Pointer to device name pointer in UTF-8 format.
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_proc_cmp_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_set_name(uint32_t dummy, uint8_t name_len, const uint8_t* p_name, gapm_proc_cmp_cb cmp_cb);
/// @} GAPM_CONFIG_API


/// @addtogroup GAPM_INFO_API
/// @{

/**
 ***************************************************************************************
 * @brief Get device version information. Version is returned in res_cb function
 *
 * @note In a configuration where lower layer are present, result is provided before function returns
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] res_cb    Function called when version is available
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_version_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_get_version(uint32_t dummy, gapm_version_cb res_cb);

/**
 ***************************************************************************************
 * @brief Get Local device Identity address
 *
 * @param[out] p_addr   Pointer to the address structure to fill with local address information
 *
 * @return Execution status (see enum #hl_err).
 ***************************************************************************************
 */
uint16_t gapm_get_identity(gap_bdaddr_t* p_addr);

/// @} GAPM_INFO_API

/// @addtogroup GAPM_ACTV_ITF_API
/// @{

/**
 ****************************************************************************************
 * @brief Stop an activity
 *
 * @param[in] actv_idx          Activity local index
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_actv_cb_t.proc_cmp callback execution
 ****************************************************************************************
 */
uint16_t gapm_actv_stop(uint8_t actv_idx);

/**
 ****************************************************************************************
 * @brief Stop all activities
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapm_actv_stop_all(uint32_t dummy, gapm_proc_cmp_cb cmp_cb);


/**
 ****************************************************************************************
 * @brief Delete an activity
 *
 * @param[in] actv_idx          Activity local index
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_actv_cb_t.proc_cmp callback execution
 ****************************************************************************************
 */
uint16_t gapm_actv_delete(uint8_t actv_idx);

/**
 ****************************************************************************************
 * @brief Delete all activities
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapm_actv_delete_all(uint32_t dummy, gapm_proc_cmp_cb cmp_cb);

/// @} GAPM_ACTV_ITF_API

#endif /* _GAPM_H_ */
