/**
 ****************************************************************************************
 *
 * @file gapc_msg.h
 *
 * @brief Generic Access Profile Controller  Message API.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */
#ifndef _GAPC_MSG_H_
#define _GAPC_MSG_H_

/**
 ****************************************************************************************
 * @addtogroup GAPC_MSG_API Message API
 * @ingroup GAPC_API
 * @brief Message API for GAP Client module
 * @details It handles messages from lower and higher layers related to an ongoing connection.
 * @{
 * @}
 ****************************************************************************************
 */


/// @addtogroup GAPC_MSG_ID_API Message Identifiers
/// @ingroup GAPC_MSG_API
/// @{
/// @}

/// @addtogroup GAPC_MSG_OPID_API Operation Identifiers
/// @ingroup GAPC_MSG_API
/// @{
/// @}

/// @addtogroup GAPC_MSG_ENUM_API Enumerations
/// @ingroup GAPC_MSG_API
/// @{
/// @}

/// @addtogroup GAPC_MSG_STRUCT_API Message Structures
/// @ingroup GAPC_MSG_API
/// @{
/// @}

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_task.h" // Task definitions
#include "gapc.h"
#include "gapc_sec.h"

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// @addtogroup GAPC_MSG_ID_API
/// @{

/// GAP Controller Task messages
/*@TRACE*/
enum gapc_msg_id
{
    /* Default event */
    /// Command Complete event
    GAPC_CMP_EVT                                        = MSG_ID(GAPC, 0x00),
    /// Indication to the task that sends the unknown message
    GAPC_UNKNOWN_MSG_IND                                = MSG_ID(GAPC, 0x01),

    /* Connection state information */
    /// Indicate that a LE connection has been established
    GAPC_LE_CONNECTION_REQ_IND                          = MSG_ID(GAPC, 0x02),
    /// Indicate that a BT Classic connection has been established
    GAPC_BT_CONNECTION_REQ_IND                          = MSG_ID(GAPC, 0x07),
    /// Set specific link data configuration.
    GAPC_CONNECTION_CFM                                 = MSG_ID(GAPC, 0x03),
    /// Indicate that there is no more ATT bearer available
    GAPC_NO_MORE_ATT_BEARER_ERROR_IND                   = MSG_ID(GAPC, 0x04),

    /* Link management command */
    /// Request disconnection of current link command.
    GAPC_DISCONNECT_CMD                                 = MSG_ID(GAPC, 0x05),
    /// Indicate that a link has been disconnected
    GAPC_DISCONNECT_IND                                 = MSG_ID(GAPC, 0x06),


    /* Connection info */
    /// Retrieve information command
    GAPC_GET_INFO_CMD                                   = MSG_ID(GAPC, 0x10),
    /// Peer device attribute DB info such as Device Name, Appearance or Slave Preferred Parameters
    GAPC_PEER_ATT_INFO_IND                              = MSG_ID(GAPC, 0x11),
    /// Indication of peer version info
    GAPC_PEER_VERSION_IND                               = MSG_ID(GAPC, 0x12),
    /// Indication of peer features info
    GAPC_PEER_FEATURES_IND                              = MSG_ID(GAPC, 0x13),
    /// Indication of ongoing connection RSSI
    GAPC_CON_RSSI_IND                                   = MSG_ID(GAPC, 0x14),
    /// Indication of ongoing connection Channel Map
    GAPC_CON_CHANNEL_MAP_IND                            = MSG_ID(GAPC, 0x15),
    /// Peer device request local device info such as name, appearance or slave preferred parameters
    GAPC_GET_DEV_INFO_REQ_IND                           = MSG_ID(GAPC, 0x16),
    /// Send requested info to peer device
    GAPC_GET_DEV_INFO_CFM                               = MSG_ID(GAPC, 0x17),
    /// Peer device request to modify local device info such as name or appearance
    GAPC_SET_DEV_INFO_REQ_IND                           = MSG_ID(GAPC, 0x18),
    /// Local device accept or reject device info modification
    GAPC_SET_DEV_INFO_CFM                               = MSG_ID(GAPC, 0x19),
    /// Indication of currently used channel selection algorithm
    /// see struct #gapc_chan_sel_algo_ind
    GAPC_CHAN_SEL_ALGO_IND                              = MSG_ID(GAPC, 0x1A),

    /* Connection parameters update */
    /// Perform update of connection parameters command
    GAPC_PARAM_UPDATE_CMD                               = MSG_ID(GAPC, 0x40),
    /// Request of updating connection parameters indication
    GAPC_PARAM_UPDATE_REQ_IND                           = MSG_ID(GAPC, 0x41),
    /// Master confirm or not that parameters proposed by slave are accepted or not
    GAPC_PARAM_UPDATE_CFM                               = MSG_ID(GAPC, 0x42),
    /// Connection parameters updated indication
    GAPC_PARAM_UPDATED_IND                              = MSG_ID(GAPC, 0x43),
    /// LE Set Data Length Command
    GAPC_SET_LE_PKT_SIZE_CMD                            = MSG_ID(GAPC, 0x44),
    /// LE Set Data Length Indication
    GAPC_LE_PKT_SIZE_IND                                = MSG_ID(GAPC, 0x45),
    /// Update LE Ping timeout value
    GAPC_SET_LE_PING_TO_CMD                             = MSG_ID(GAPC, 0x46),
    /// LE Ping timeout indication
    GAPC_LE_PING_TO_VAL_IND                             = MSG_ID(GAPC, 0x47),
    /// LE Ping timeout expires indication
    GAPC_LE_PING_TO_IND                                 = MSG_ID(GAPC, 0x48),
    /// Set the PHY configuration for current active link
    GAPC_SET_PHY_CMD                                    = MSG_ID(GAPC, 0x49),
    /// Active link PHY configuration. Triggered when configuration is read or during an update.
    GAPC_LE_PHY_IND                                     = MSG_ID(GAPC, 0x4A),
    /// Set the preferred slave latency (for slave only, with RW controller)
    GAPC_SET_PREF_SLAVE_LATENCY_CMD                     = MSG_ID(GAPC, 0x4B),
    /// Set the preferred slave event duration (for slave only, with RW controller)
    GAPC_SET_PREF_SLAVE_EVT_DUR_CMD                     = MSG_ID(GAPC, 0x4C),
    /// Set the maximum reception and size and time using DLE negotiation
    GAPC_SET_MAX_RX_SIZE_AND_TIME_CMD                   = MSG_ID(GAPC, 0x4D),

    /* Bonding procedure */
    /// Start Bonding command procedure
    GAPC_BOND_CMD                                       = MSG_ID(GAPC, 0x50),
    /// Bonding requested by peer device indication message.
    GAPC_BOND_REQ_IND                                   = MSG_ID(GAPC, 0x51),
    /// Confirm requested bond information.
    GAPC_BOND_CFM                                       = MSG_ID(GAPC, 0x52),
    /// Bonding information indication message
    GAPC_BOND_IND                                       = MSG_ID(GAPC, 0x53),
    /// Request to inform the remote device when keys have been entered or erased
    GAPC_KEY_PRESS_NOTIFICATION_CMD                     = MSG_ID(GAPC, 0x54),
    /// Indication that a KeyPress has been performed on the peer device.
    GAPC_KEY_PRESS_NOTIFICATION_IND                     = MSG_ID(GAPC, 0x55),
    /* Security request procedure */
    /// Start Security Request command procedure
    GAPC_SECURITY_CMD                                   = MSG_ID(GAPC, 0x56),
    /// Security requested by peer device indication message
    GAPC_SECURITY_IND                                   = MSG_ID(GAPC, 0x57),

    /* Encryption procedure */
    /// Start LE Encryption command procedure
    /// see #gapc_le_encrypt
    GAPC_LE_ENCRYPT_CMD                                 = MSG_ID(GAPC, 0x58),
    /// LE Encryption requested by peer device indication message.
    /// see #gapc_security_cb_t.le_encrypt_req
    GAPC_LE_ENCRYPT_REQ_IND                             = MSG_ID(GAPC, 0x59),
    /// Confirm requested LE Encryption information.
    /// see #gapc_le_encrypt_req_reply
    GAPC_LE_ENCRYPT_CFM                                 = MSG_ID(GAPC, 0x5A),

    /// BT Classic Encryption information requested indication message.
    /// see #gapc_security_cb_t.bt_encrypt_req
    GAPC_BT_ENCRYPT_REQ_IND                             = MSG_ID(GAPC, 0x5E),
    /// Confirm requested Encryption information.
    /// see #gapc_bt_encrypt_req_reply
    GAPC_BT_ENCRYPT_CFM                                 = MSG_ID(GAPC, 0x5F),

    /// Message triggered when LE or BT Classic link becomes encrypted
    /// see #gapc_security_cb_t.auth_info
    GAPC_ENCRYPT_IND                                    = MSG_ID(GAPC, 0x5B),

    /* Bond Data information  */
    /// Indicate update of bond data information
    GAPC_BOND_DATA_UPDATE_IND                           = MSG_ID(GAPC, 0x5C),
    /* BT Classic Security  */
    /// Set BT classic connection required security level
    GAPC_BT_SET_REQ_SEC_LVL_CMD                         = MSG_ID(GAPC, 0x5D),

    /* Periodic Sync Transfer */
    /// Transfer periodic advertising sync information to peer device
    GAPC_PER_ADV_SYNC_TRANS_CMD                         = MSG_ID(GAPC, 0x60),
    /* Client Features */
    /// Enable usage of supported client features
    GAPC_CLI_FEAT_EN_CMD                                = MSG_ID(GAPC, 0x61),

    /* Constant Tone Extension */
    /// Constant Tone Extension Transmission configuration command
    GAPC_CTE_TX_CFG_CMD                                 = MSG_ID(GAPC, 0x70),
    /// Constant Tone Extension Reception configuration command
    GAPC_CTE_RX_CFG_CMD                                 = MSG_ID(GAPC, 0x71),
    /// Constant Tone Extension request control command (enable / disable)
    GAPC_CTE_REQ_CTRL_CMD                               = MSG_ID(GAPC, 0x72),
    /// Constant Tone Extension Response control command (enable / disable)
    GAPC_CTE_RSP_CTRL_CMD                               = MSG_ID(GAPC, 0x73),
    /// Indicate reception of a IQ Report event over a ble connection
    GAPC_CTE_IQ_REPORT_IND                              = MSG_ID(GAPC, 0x74),
    /// Indicate that an IQ Request has been rejected or CTE data not present in LMP response
    GAPC_CTE_REQ_FAILED_IND                             = MSG_ID(GAPC, 0x75),

    /* LE Power Control */
    /// Local TX power indication
    GAPC_LOC_TX_PWR_IND                                 = MSG_ID(GAPC, 0x80),
    /// Remote TX power indication
    GAPC_PEER_TX_PWR_IND                                = MSG_ID(GAPC, 0x81),
    /// Control TX Power Reports command
    GAPC_TX_PWR_REPORT_CTRL_CMD                         = MSG_ID(GAPC, 0x82),
    /// Local TX power change report indication
    GAPC_LOC_TX_PWR_REPORT_IND                          = MSG_ID(GAPC, 0x83),
    /// Remote TX power change report indication
    GAPC_PEER_TX_PWR_REPORT_IND                         = MSG_ID(GAPC, 0x84),
    /// Control Path loss configuration
    GAPC_PATH_LOSS_CTRL_CMD                             = MSG_ID(GAPC, 0x85),
    /// Path Loss Threshold Event Indication
    GAPC_PATH_LOSS_THRESHOLD_IND                        = MSG_ID(GAPC, 0x86),
};
/// @} GAPC_MSG_ID_API

/// @addtogroup GAPC_MSG_OPID_API
/// @{

/// request operation type - application interface
/*@TRACE*/
enum gapc_operation
{
    /*                 Operation Flags                  */
    /* No Operation (if nothing has been requested)     */
    /* ************************************************ */
    /// No operation
    GAPC_NO_OP                                    = 0x00,

    /* Connection management */
    /// Disconnect link
    GAPC_DISCONNECT                               = 0x01,

    /* Connection information */
    /// Retrieve name of peer device.
    GAPC_GET_PEER_NAME                            = 0x10,
    /// Retrieve peer device version info.
    GAPC_GET_PEER_VERSION                         = 0x11,
    /// Retrieve peer device features.
    GAPC_GET_PEER_FEATURES                        = 0x12,
    /// Get Peer device appearance
    GAPC_GET_PEER_APPEARANCE                      = 0x13,
    /// Get Peer device Slaved Preferred Parameters
    GAPC_GET_PEER_SLV_PREF_PARAMS                 = 0x14,
    /// Retrieve connection RSSI.
    GAPC_GET_CON_RSSI                             = 0x15,
    /// Retrieve Connection Channel MAP.
    GAPC_GET_CON_CHANNEL_MAP                      = 0x16,
    /// Retrieve Channel Selection Algorithm
    GAPC_GET_CHAN_SEL_ALGO                        = 0x17,
    /// Get if Central Address resolution supported
    GAPC_GET_ADDR_RESOL_SUPP                      = 0x18,
    /// Retrieve Peer database Hash value
    GAPC_GET_PEER_DB_HASH                         = 0x19,
    /// get timer timeout value
    GAPC_GET_LE_PING_TO                           = 0x1A,
    /// Retrieve PHY configuration of active link
    GAPC_GET_PHY                                  = 0x1B,
    /// Read the local current and maximum transmit power levels for 1M PHY
    GAPC_GET_LOC_TX_PWR_LEVEL_1M                  = 0x1C,
    /// Read the local current and maximum transmit power levels for 2M PHY
    GAPC_GET_LOC_TX_PWR_LEVEL_2M                  = 0x1D,
    /// Read the local current and maximum transmit power levels for LE CODED PHY with S=8 data coding
    GAPC_GET_LOC_TX_PWR_LEVEL_LE_CODED_S8         = 0x1E,
    /// Read the local current and maximum transmit power levels for LE CODED PHY with S=2 data coding
    GAPC_GET_LOC_TX_PWR_LEVEL_LE_CODED_S2         = 0x1F,

    /// Read the transmit power level used by the remote Controller for 1M PHY
    GAPC_GET_PEER_TX_PWR_LEVEL_1M                 = 0x20,
    /// Read the transmit power level used by the remote Controller for 2M PHY
    GAPC_GET_PEER_TX_PWR_LEVEL_2M                 = 0x21,
    /// Read the transmit power level used by the remote Controller for LE CODED PHY with S=8 data coding
    GAPC_GET_PEER_TX_PWR_LEVEL_LE_CODED_S8        = 0x22,
    /// Read the transmit power level used by the remote Controller for LE CODED PHY with S=2 data coding
    GAPC_GET_PEER_TX_PWR_LEVEL_LE_CODED_S2        = 0x23,
    /// Resolvable Private Address Only declaration
    GAPC_GET_RSLV_PRIV_ADDR_ONLY                  = 0x24,

    /* Connection parameters update */
    /// Perform update of connection parameters.
    GAPC_UPDATE_PARAMS                            = 0x40,
    /// set timer timeout value
    GAPC_SET_LE_PING_TO                           = 0x41,
    /// LE Set Data Length
    GAPC_SET_LE_PKT_SIZE                          = 0x42,
    /// Set the PHY configuration for current active link
    GAPC_SET_PHY                                  = 0x43,
    /// Set the preferred slave latency (for slave only, with RW controller)
    GAPC_SET_PREF_SLAVE_LATENCY                   = 0x44,
    /// Set the preferred slave event duration (for slave only, with RW controller)
    GAPC_SET_PREF_SLAVE_EVT_DUR                   = 0x45,
    /// Set maximum RX size and time using DLE negotiation
    GAPC_SET_MAX_RX_SIZE_AND_TIME                 = 0x46,

    /* Security procedures */
    /// Start bonding procedure.
    GAPC_BOND                                     = 0x50,
    /// Start encryption procedure.
    GAPC_LE_ENCRYPT                                  = 0x51,
    /// Start security request procedure
    GAPC_SECURITY_REQ                             = 0x52,
    /// Request to inform the remote device when keys have been entered or erased
    GAPC_KEY_PRESS_NOTIFICATION                   = 0x53,
    /// Set BT classic connection required security level operation
    GAPC_BT_SET_REQ_SEC_LVL                       = 0x54,

    /* Periodic Sync Transfer */
    /// Transfer periodic advertising sync information to peer device
    GAPC_PER_ADV_SYNC_TRANS                       = 0x60,

    /* Client Features */
    /// Enable usage of supported client features
    GAPC_CLI_FEAT_EN                              = 0x61,

    /* Constant Tone Extension */
    /// Constant Tone Extension Transmission configuration
    GAPC_CTE_TX_CFG                               = 0x70,
    /// Constant Tone Extension Reception configuration
    GAPC_CTE_RX_CFG                               = 0x71,
    /// Constant Tone Extension request control (enable / disable)
    GAPC_CTE_REQ_CTRL                             = 0x72,
    /// Constant Tone Extension Response control (enable / disable)
    GAPC_CTE_RSP_CTRL                             = 0x73,

    /* LE Power Control */
    /// Enable or disable the reporting to the local Host of transmit power level
    /// changes in the local and remote Controllers for the ACL connection
    GAPC_TX_PWR_REPORT_CTRL                       = 0x80,
    /// Command is used to enable/disable path loss reporting for the connection
    GAPC_PATH_LOSS_REPORT_CTRL                    = 0x81,
};
/// @} GAPC_MSG_OPID_API

/// @addtogroup GAPC_MSG_ENUM_API
/// @{

/// Bond event type.
/*@TRACE*/
enum gapc_bond
{
    /// Bond Pairing request
    GAPC_PAIRING_REQ,
    /// Respond to Pairing request
    GAPC_PAIRING_RSP,

    /// Pairing Finished information
    GAPC_PAIRING_SUCCEED,
    /// Pairing Failed information
    GAPC_PAIRING_FAILED,

    /// Used to retrieve pairing Temporary Key
    GAPC_TK_EXCH,
    /// Used for Identity Resolving Key exchange
    GAPC_IRK_EXCH,
    /// Used for Connection Signature Resolving Key exchange
    GAPC_CSRK_EXCH,
    /// Used for Long Term Key exchange
    GAPC_LTK_EXCH,

    /// Bond Pairing request issue, Repeated attempt
    GAPC_REPEATED_ATTEMPT,

    /// Out of Band - exchange of confirm and rand.
    GAPC_OOB_EXCH,

    /// Numeric Comparison - Exchange of Numeric Value -
    GAPC_NC_EXCH,

    /// BT Classic IO Capabilities
    GAPC_BT_IOCAP,
    /// BT Classic PIN Code
    GAPC_BT_PIN_CODE,
    /// BT Classic user value Confirm
    GAPC_BT_USER_VALUE_CFM,
    /// BT Classic passkey value
    GAPC_BT_PASSKEY,
    /// BT Classic pairing end status
    GAPC_BT_PAIRING_END,
    /// BT Classic link authentication information
    GAPC_BT_AUTH_INFO,
    /// BT Classic link key generated
    GAPC_BT_LINK_KEY,

};

/// List of device info that should be provided by application
/*@TRACE*/
enum gapc_dev_info
{
    /// Device Name
    GAPC_DEV_NAME,
    /// Device Appearance Icon
    GAPC_DEV_APPEARANCE,
    /// Device Slave preferred parameters
    GAPC_DEV_SLV_PREF_PARAMS,
    /// Device Central address resolution
    GAPC_DEV_CTL_ADDR_RESOL,
    /// Device database hash value
    GAPC_DEV_DB_HASH,
    /// Resolvable Private address only after bond
    GAPC_DEV_RSLV_PRIV_ADDR_ONLY,
    /// maximum device info parameter
    GAPC_DEV_INFO_MAX,
};

/// @} GAPC_MSG_ENUM_API

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// @addtogroup GAPC_MSG_STRUCT_API
/// @{

/// Operation command structure in order to keep requested operation.
struct gapc_operation_cmd
{
    /// Connection index
    uint8_t conidx;
    /// GAP request type
    uint8_t operation;
};


/// Command complete event data structure
/*@TRACE*/
struct gapc_cmp_evt
{
    /// Connection index
    uint8_t conidx;
    /// GAP request type
    uint8_t operation;
    /// Status of the request
    uint16_t status;
};

/// Indicate that an unknown message has been received
/*@TRACE*/
struct gapc_unknown_msg_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Unknown message id
    uint16_t unknown_msg_id;
};


/// Set specific link data configuration.
/*@TRACE*/
struct gapc_connection_cfm
{
    /// Connection index
    uint8_t          conidx;
    /// Bond data
    gapc_bond_data_t bond_data;
};

/// Parameters of #GAPC_NO_MORE_ATT_BEARER_ERROR_IND message
/*@TRACE*/
struct gapc_no_more_att_bearer_error_ind
{
    /// Connection index
    uint8_t          conidx;
};

/// Request disconnection of current link command.
/*@TRACE*/
struct gapc_disconnect_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// GAP request type:
    /// - #GAPC_DISCONNECT: Disconnect link.
    uint8_t  operation;
    /// Reason of disconnection
    uint16_t reason;
};


/// Indicate that a link has been disconnected
/*@TRACE*/
struct gapc_disconnect_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Connection handle
    uint16_t conhdl;
    /// Reason of disconnection
    uint16_t reason;
};


/// Retrieve information command
/*@TRACE*/
struct gapc_get_info_cmd
{
    /// Connection index
    uint8_t conidx;
    /// GAP request type:
    /// - #GAPC_GET_PEER_NAME: Retrieve name of peer device.
    /// - #GAPC_GET_PEER_VERSION: Retrieve peer device version info.
    /// - #GAPC_GET_PEER_FEATURES: Retrieve peer device features.
    /// - #GAPC_GET_PEER_APPEARANCE: Get Peer device appearance
    /// - #GAPC_GET_PEER_SLV_PREF_PARAMS: Get Peer device Slaved Preferred Parameters
    /// - #GAPC_GET_CON_RSSI: Retrieve connection RSSI.
    /// - #GAPC_GET_CON_CHANNEL_MAP: Retrieve Connection Channel MAP.
    /// - #GAPC_GET_CHAN_SEL_ALGO: Retrieve Channel Selection Algorithm
    /// - #GAPC_GET_ADDR_RESOL_SUPP: Address Resolution Supported
    /// - #GAPC_GET_PEER_DB_HASH: Retrieve Peer database Hash value
    /// - #GAPC_GET_LE_PING_TO: Retrieve LE Ping Timeout Value
    /// - #GAPC_GET_PHY: Retrieve PHY configuration of active link
    /// - #GAPC_GET_LOC_TX_PWR_LEVEL_1M: Read the current and maximum transmit power levels for 1M PHY
    /// - #GAPC_GET_LOC_TX_PWR_LEVEL_2M: Read the current and maximum transmit power levels for 2M PHY
    /// - #GAPC_GET_LOC_TX_PWR_LEVEL_LE_CODED_S8: Read the current and maximum transmit power levels for LE CODED PHY with S=8 data coding
    /// - #GAPC_GET_LOC_TX_PWR_LEVEL_LE_CODED_S2: Read the current and maximum transmit power levels for LE CODED PHY with S=2 data coding
    /// - #GAPC_GET_PEER_TX_PWR_LEVEL_1M: Read the transmit power level used by the remote Controller for 1M PHY
    /// - #GAPC_GET_PEER_TX_PWR_LEVEL_2M: Read the transmit power level used by the remote Controller for 2M PHY
    /// - #GAPC_GET_PEER_TX_PWR_LEVEL_LE_CODED_S8: Read the transmit power level used by the remote Controller for LE CODED PHY with S=8 data coding
    /// - #GAPC_GET_PEER_TX_PWR_LEVEL_LE_CODED_S2: Read the transmit power level used by the remote Controller for LE CODED PHY with S=2 data coding
    /// - #GAPC_GET_RSLV_PRIV_ADDR_ONLY: Resolvable Private Address Only declaration
    uint8_t operation;
};

/// device information data
/*@TRACE
 @trc_ref gapc_dev_info*/
union gapc_dev_info_val
{
    /// Device name
    //@trc_union parent.req == GAPC_DEV_NAME
    struct gap_dev_name name;
    /// Appearance Icon
    //@trc_union parent.req == GAPC_DEV_APPEARANCE
    uint16_t            appearance;
    /// Slave preferred parameters
    //@trc_union parent.req == GAPC_DEV_SLV_PREF_PARAMS
    gap_periph_pref_t   slv_pref_params;
    /// Central address resolution
    //@trc_union parent.req == GAPC_DEV_CTL_ADDR_RESOL
    uint8_t             ctl_addr_resol;
    /// Database Hash value
    //@trc_union parent.req == GAPC_DEV_DB_HASH
    uint8_t             hash[16];
    /// Resolvable Private address only
    //@trc_union parent.req == GAPC_DEV_RSLV_PRIV_ADDR_ONLY
    uint8_t             rslv_priv_addr_only;
};

/// Peer device attribute DB info such as Device Name, Appearance or Slave Preferred Parameters
/*@TRACE*/
struct gapc_peer_att_info_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Requested information
    /// - #GAPC_DEV_NAME: Device Name
    /// - #GAPC_DEV_APPEARANCE: Device Appearance Icon
    /// - #GAPC_DEV_SLV_PREF_PARAMS: Device Slave preferred parameters
    /// - #GAPC_DEV_CTL_ADDR_RESOL: Address resolution supported
    /// - #GAPC_DEV_DB_HASH: Device Database Hash value
    /// - #GAPC_GET_RSLV_PRIV_ADDR_ONLY: Resolvable Private Address Only declaration
    uint8_t  req;
    /// Attribute handle
    uint16_t handle;

    /// device information data
    union gapc_dev_info_val info;
};

/// Indication of peer version info
/*@TRACE*/
struct gapc_peer_version_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Manufacturer name
    uint16_t compid;
    /// LMP subversion
    uint16_t lmp_subvers;
    /// LMP version
    uint8_t  lmp_vers;
};

/// Indication of ongoing connection RSSI
/*@TRACE*/
struct gapc_con_rssi_ind
{
    /// Connection index
    uint8_t conidx;
    /// RSSI value
    int8_t  rssi;
};


/// Sign counter value changed due to packet signing or signature verification.
struct gapc_sign_counter_updated_ind
{
    /// Connection index
    uint8_t  conidx;
    /// New Local signature counter value
    uint32_t lsign_counter;
    /// New Remote signature counter value
    uint32_t rsign_counter;
};

/// Indication of LE Ping
/*@TRACE*/
struct gapc_le_ping_to_val_ind
{
    /// Connection index
    uint8_t  conidx;
    ///Authenticated payload timeout
    uint16_t timeout;
};

/// Peer device request local device info such as name, appearance or slave preferred parameters
/*@TRACE*/
struct gapc_get_dev_info_req_ind
{
    /// Connection index
    uint8_t                 conidx;
    /// Requested information
    /// - #GAPC_DEV_NAME: Device Name
    /// - #GAPC_DEV_APPEARANCE: Device Appearance Icon
    /// - #GAPC_DEV_SLV_PREF_PARAMS: Device Slave preferred parameters
    uint8_t                 req;
    /// Token value that must be returned in confirmation
    uint16_t                token;
    /// Device Name data offset
    uint16_t                name_offset;
    /// Maximum name length (starting from offset)
    uint16_t                max_name_length;
};

/// Send requested info to peer device
/*@TRACE*/
struct gapc_get_dev_info_cfm
{
    /// Connection index
    uint8_t                 conidx;
    /// Requested information
    /// - #GAPC_DEV_NAME: Device Name
    /// - #GAPC_DEV_APPEARANCE: Device Appearance Icon
    /// - #GAPC_DEV_SLV_PREF_PARAMS: Device Slave preferred parameters
    uint8_t                 req;
    /// Status code used to know if requested has been accepted or not
    uint16_t                status;
    /// Token value provided in request indication
    uint16_t                token;
    /// Complete value length including offset
    uint16_t                complete_length;
    /// Peer device information data
    union gapc_dev_info_val info;
};

/// device information data
/*@TRACE
 @trc_ref gapc_dev_info*/
union gapc_set_dev_info
{
    /// Device name
    //@trc_union parent.req == GAPC_DEV_NAME
    struct gap_dev_name     name;
    /// Appearance Icon
    //@trc_union parent.req == GAPC_DEV_APPEARANCE
    uint16_t                appearance;
};

/// Peer device request to modify local device info such as name or appearance
/*@TRACE*/
struct gapc_set_dev_info_req_ind
{
    /// Connection index
    uint8_t                 conidx;
    /// Requested information
    /// - #GAPC_DEV_NAME: Device Name
    /// - #GAPC_DEV_APPEARANCE: Device Appearance Icon
    uint8_t                 req;
    /// Token value that must be returned in confirmation
    uint16_t                token;
    /// device information data
    union gapc_set_dev_info info;
};

/// Local device accept or reject device info modification
/*@TRACE*/
struct gapc_set_dev_info_cfm
{
    /// Connection index
    uint8_t                 conidx;
    /// Requested information
    /// - #GAPC_DEV_NAME: Device Name
    /// - #GAPC_DEV_APPEARANCE: Device Appearance Icon
    uint8_t                 req;
    /// Status code used to know if requested has been accepted or not
    uint16_t                status;
    /// Token value provided in request indication
    uint16_t                token;
};

/// Bond procedure requested information data
/*@TRACE
 @trc_ref gapc_bond*/
union gapc_bond_req_data
{
    /// Authentication level (#gap_auth) (if request = #GAPC_PAIRING_REQ)
    //@trc_union parent.request == GAPC_PAIRING_REQ
    uint8_t auth_req;
    /// LTK Key Size (if request = #GAPC_LTK_EXCH)
    //@trc_union parent.request == GAPC_LTK_EXCH
    uint8_t key_size;
    /// Device IO used to get TK: (if request = #GAPC_TK_EXCH)
    ///  - #GAP_TK_OOB:       TK get from out of band method
    ///  - #GAP_TK_DISPLAY:   TK generated and shall be displayed by local device
    ///  - #GAP_TK_KEY_ENTRY: TK shall be entered by user using device keyboard
    //@trc_union parent.request == GAPC_TK_EXCH
    uint8_t tk_type;

    /// Numeric value
    //@trc_union parent.request == GAPC_BT_USER_VALUE_CFM
    //@trc_union parent.request == GAPC_NC_EXCH
    uint32_t numeric_value;
};

/// Bonding requested by peer device indication message.
/*@TRACE*/
struct gapc_bond_req_ind
{
    /// Connection index
    uint8_t conidx;
    /// Bond request type (see #gapc_bond)
    uint8_t request;

    /// Bond procedure requested information data
    union gapc_bond_req_data data;
};


/// BT Classic IO Capabilities
/*@TRACE*/
typedef struct gapc_bt_iocap
{
    /// Local device IO capabilities (#gap_io_cap)
    uint8_t   iocap;
    /// Authentication requirement bit field (see enum #gap_bt_auth_req_bf)
    uint8_t   auth_req_bf;
    /// OOB data presence (see enum #gap_bt_oob_flag)
    uint8_t   oob_data_present;
    /// P-192 OOB data if present
    gap_oob_t oob_192;
    /// P-256 OOB data if present
    gap_oob_t oob_256;
} gapc_bt_iocap_t;


/// BT classic PIN Code value
/*@TRACE*/
typedef struct gapc_bt_pincode
{
    /// Pin code length (range [1-16])
    uint8_t length;
    /// Pin Code data (16 bytes max)
    uint8_t data[GAP_KEY_LEN];
} gapc_bt_pincode_t;

/// Pairing features
/*@TRACE*/
typedef struct gapc_pairing_feat
{
    /// Pairing information
    gapc_pairing_t pairing_info;
    /// Device security requirements (minimum security level). (see enum #gap_sec_req)
    uint8_t        sec_req_level;
} gapc_pairing_feat_t;


/// Confirmation message bond data union
/*@TRACE
 @trc_ref gapc_bond
*/
union gapc_bond_cfm_data
{
    /// Pairing Features (request = #GAPC_PAIRING_RSP)
    //@trc_union parent.request == GAPC_PAIRING_RSP
    gapc_pairing_feat_t pairing_feat;
    /// LTK (request = #GAPC_LTK_EXCH)
    //@trc_union parent.request == GAPC_LTK_EXCH
    gapc_ltk_t ltk;
    /// CSRK (request = #GAPC_CSRK_EXCH)
    //@trc_union parent.request == GAPC_CSRK_EXCH
    gap_sec_key_t csrk;
    /// TK (request = #GAPC_TK_EXCH)
    //@trc_union parent.request == GAPC_TK_EXCH
    gap_sec_key_t tk;
    /// IRK (request = #GAPC_IRK_EXCH)
    //@trc_union parent.request == GAPC_IRK_EXCH
    gapc_irk_t irk;
    /// OOB Confirm and Random from the peer (request = #GAPC_OOB_EXCH)
    //@trc_union parent.request == GAPC_OOB_EXCH
    gap_oob_t oob;

    /// BT Classic IO Capabilities (request = #GAPC_BT_IOCAP)
    //@trc_union parent.request == GAPC_BT_IOCAP
    gapc_bt_iocap_t   iocap;
    /// BT Classic PIN Code (request = #GAPC_BT_PIN_CODE)
    //@trc_union parent.request == GAPC_BT_PIN_CODE
    gapc_bt_pincode_t pincode;
    /// BT Classic Passkey value (request = #GAPC_BT_PASSKEY)
    //@trc_union parent.request == GAPC_BT_PASSKEY
    uint32_t          passkey;
};

/// Confirm requested bond information.
/*@TRACE*/
struct gapc_bond_cfm
{
    /// Connection index
    uint8_t conidx;
    /// Bond request type (see #gapc_bond)
    uint8_t request;
    /// Request accepted
    uint8_t accept;

    /// Bond procedure information data
    union gapc_bond_cfm_data data;
};

/**
 *  Pairing information
 */
/*@TRACE*/
typedef struct gapc_pairing_info
{
    /// Pairing level information (#gap_pairing_lvl)
    uint8_t level;
    /// LTK exchanged during pairing.
    bool    ltk_present;
} gapc_pairing_info_t;


/// BT Classing Link authentication information
/*@TRACE*/
typedef struct gapc_bt_auth_info
{
    /// Link Security level (see enum #gap_sec_lvl)
    uint8_t sec_lvl;
    /// True if link encrypted, False otherwise
    bool    encrypted;
} gapc_bt_auth_info_t;

/// BT Classing Link key generated
/*@TRACE*/
typedef struct gap_bt_link_key
{
    /// Link key value
    gap_sec_key_t key;
    /// Link key pairing level (see enum #gap_pairing_lvl)
    uint8_t       pairing_lvl;
} gap_bt_link_key_t;

/// BT Classic exchanged IO Capabilities
/*@TRACE*/
typedef struct gapc_bt_iocap_info
{
    /// Local device IO capabilities (#gap_io_cap)
    uint8_t   iocap;
    /// Authentication requirement bit field (see enum #gap_bt_auth_req_bf)
    uint8_t   auth_req_bf;
    /// OOB data presence: True if present, false otherwise
    bool   oob_data_present;
} gapc_bt_iocap_info_t;

/// Bond procedure information data
/*@TRACE
 @trc_ref gapc_bond*/
union gapc_bond_data
{
    /// Pairing information
    /// (if info = #GAPC_PAIRING_SUCCEED)
    //@trc_union parent.info == GAPC_PAIRING_SUCCEED
    gapc_pairing_info_t  pairing;
    /// Pairing failed reason  (if info = #GAPC_PAIRING_FAILED)
    //@trc_union parent.info == GAPC_PAIRING_FAILED
    uint16_t             reason;
    /// Long Term Key information (if info = #GAPC_LTK_EXCH)
    //@trc_union parent.info == GAPC_LTK_EXCH
    gapc_ltk_t           ltk;
    /// Identity Resolving Key information (if info = #GAPC_IRK_EXCH)
    //@trc_union parent.info == GAPC_IRK_EXCH
    gapc_irk_t           irk;
    /// Connection Signature Resolving Key information (if info = #GAPC_CSRK_EXCH)
    //@trc_union parent.info == GAPC_CSRK_EXCH
    gap_sec_key_t        csrk;
    /// BT Classic Peer IO CAP information (if info = #GAPC_BT_IOCAP)
    //@trc_union parent.info == GAPC_BT_IOCAP
    gapc_bt_iocap_info_t peer_info;
    /// BT Classic Passkey numeric value to display (if info = #GAPC_BT_PASSKEY)
    //@trc_union parent.info == GAPC_BT_PASSKEY
    uint32_t             passkey;
    /// BT Classic pairing end status (if info = #GAPC_BT_PAIRING_END)
    /// (value is type enum #hl_err)
    //@trc_union parent.info == GAPC_BT_PAIRING_END
    uint16_t             bt_pairing_status;
    /// BT Classic link authentication information (if info = #GAPC_BT_AUTH_INFO)
    //@trc_union parent.info == GAPC_BT_AUTH_INFO
    gapc_bt_auth_info_t  bt_auth_info;
    /// BT Classic link key generated by pairing (if info = #GAPC_BT_LINK_KEY)
    //@trc_union parent.info == GAPC_BT_LINK_KEY
    gap_bt_link_key_t    bt_link_key;
};

/// Bonding information indication message
/*@TRACE*/
struct gapc_bond_ind
{
    /// Connection index
    uint8_t conidx;
    /// Bond information type (see #gapc_bond)
    uint8_t info;

    /// Bond procedure information data
    union gapc_bond_data data;
};

/// Parameters of the #GAPC_BOND_DATA_UPDATE_IND message
/*@TRACE*/
struct gapc_bond_data_update_ind
{
    /// Connection index
    uint8_t                  conidx;
    /// Updated bond data
    gapc_bond_data_updated_t data;
};

/// Encryption information indication message
/*@TRACE*/
struct gapc_encrypt_ind
{
    /// Connection index
    uint8_t conidx;
    /// Pairing level (#gap_pairing_lvl)
    uint8_t pairing_lvl;
};

/// Parameters of the #GAPC_SET_LE_PING_TO_CMD message
/*@TRACE*/
struct gapc_set_le_ping_to_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// GAP request type:
    /// - #GAPC_SET_LE_PING_TO : Set the LE Ping timeout value
    uint8_t  operation;
    /// Authenticated payload timeout
    uint16_t timeout;
};

/// Parameters of the #GAPC_LE_PING_TO_IND message
/*@TRACE*/
struct gapc_le_ping_to_ind
{
    /// Connection index
    uint8_t  conidx;
};

/// @} GAPC_MSG_STRUCT_API

#endif /* _GAPC_MSG_H_ */
