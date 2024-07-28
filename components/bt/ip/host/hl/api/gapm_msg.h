/**
 ****************************************************************************************
 *
 * @file gapm_msg.h
 *
 * @brief Generic Access Profile Manager Message API.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


#ifndef _GAPM_MSG_H_
#define _GAPM_MSG_H_

/**
 ****************************************************************************************
 * @addtogroup GAPM_MSG_API Message API
 * @ingroup GAPM_API
 * @brief Message API for GAP Manager module
 * @details It handles messages from lower and higher layers not related to an ongoing connection.
 * @{
 * @}
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_task.h" // Task definitions
#include "gap.h"
#include "gapm.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */
/// @addtogroup GAPM_MSG_ID_API Message Identifiers
/// @ingroup GAPM_MSG_API
/// @{

/// GAP Manager Message Interface
/*@TRACE*/
enum gapm_msg_ids
{
    /* Default event */
    /// Command Complete event
    GAPM_CMP_EVT                                     = MSG_ID(GAPM, 0x00),
    /// Indicate that a message has been received on an unknown task
    GAPM_UNKNOWN_TASK_IND                            = MSG_ID(GAPM, 0x01),
    /// Indication to the task that sends the unknown message
    /// see struct #gapm_unknown_msg_ind
    GAPM_UNKNOWN_MSG_IND                             = MSG_ID(GAPM, 0x02),

    /* Device Configuration */
    /// Reset link layer and the host command
    GAPM_RESET_CMD                                   = MSG_ID(GAPM, 0x03),
    /// Set device configuration command
    GAPM_SET_DEV_CONFIG_CMD                          = MSG_ID(GAPM, 0x04),
    /// Set device le channel map
    GAPM_SET_LE_CHANNEL_MAP_CMD                      = MSG_ID(GAPM, 0x05),
    /// Modify current IRK
    GAPM_SET_IRK_CMD                                 = MSG_ID(GAPM, 0x06),
    /// Set Device Name
    GAPM_SET_NAME_CMD                                = MSG_ID(GAPM, 0x07),
    /// Insert into SDP a device identification record command.
    GAPM_SET_SDP_DEVICE_IDENTIFICATION_RECORD_CMD    = MSG_ID(GAPM, 0x08),

    /* Local device information */
    /// Get local device info command
    GAPM_GET_DEV_INFO_CMD                            = MSG_ID(GAPM, 0x10),
    /// Local device version indication event
    GAPM_DEV_VERSION_IND                             = MSG_ID(GAPM, 0x11),
    /// Local device BD Address indication event
    GAPM_DEV_BDADDR_IND                              = MSG_ID(GAPM, 0x12),
    /// Advertising channel Tx power level
    GAPM_DEV_ADV_TX_POWER_IND                        = MSG_ID(GAPM, 0x13),
    /// Debug Indication containing information about memory usage.
    GAPM_DBG_MEM_INFO_IND                            = MSG_ID(GAPM, 0x14),
    /// Indication containing controller antenna information
    GAPM_ANTENNA_INF_IND                             = MSG_ID(GAPM, 0x15),
    /// Suggested Default Data Length indication
    GAPM_SUGG_DFLT_DATA_LEN_IND                      = MSG_ID(GAPM, 0x16),
    /// Maximum Data Length indication
    GAPM_MAX_LE_DATA_LEN_IND                         = MSG_ID(GAPM, 0x17),
    /// Indicate maximum advertising data length supported by controller
    /// see struct #gapm_max_adv_data_len_ind
    GAPM_MAX_ADV_DATA_LEN_IND                        = MSG_ID(GAPM, 0x18),
    /// Indicate number of available advertising sets
    /// see struct #gapm_nb_adv_sets_ind
    GAPM_NB_ADV_SETS_IND                             = MSG_ID(GAPM, 0x19),
    /// Indicate the transmit powers supported by the controller
    /// see struct #gapm_dev_tx_pwr_ind
    GAPM_DEV_TX_PWR_IND                              = MSG_ID(GAPM, 0x1A),
    /// Indicate the RF path compensation values
    /// see struct #gapm_dev_rf_path_comp_ind
    GAPM_DEV_RF_PATH_COMP_IND                        = MSG_ID(GAPM, 0x1B),
    /// Debug Indication containing statistics of the system.
    GAPM_DBG_STATS_IND                               = MSG_ID(GAPM, 0x1C),

    /* Security / Encryption Toolbox */
    /// (AES command) Resolve address command
    /// see #gapm_resolve_address
    GAPM_RESOLV_ADDR_CMD                             = MSG_ID(GAPM, 0x30),
    /// Indicate that resolvable random address has been solved
    GAPM_ADDR_SOLVED_IND                             = MSG_ID(GAPM, 0x31),
    /// (AES command) Generate a random address.
    /// see #gapm_generate_random_address
    /// @note Running in parallel different AES command from different task is dangerous,
    ///       result could be provided to last task which request AES function.
    GAPM_GEN_RAND_ADDR_CMD                           = MSG_ID(GAPM, 0x32),
    /// (AES command) Use the AES-128 block in the controller
    /// see #gapm_aes_cipher
    GAPM_USE_ENC_BLOCK_CMD                           = MSG_ID(GAPM, 0x33),
    ///  AES-128 block result indication
    GAPM_USE_ENC_BLOCK_IND                           = MSG_ID(GAPM, 0x34),
    /// (AES command) Generate a 8-byte random number
    /// see gapm_generate_random_number
    GAPM_GEN_RAND_NB_CMD                             = MSG_ID(GAPM, 0x35),
    /// Random Number Indication
    GAPM_GEN_RAND_NB_IND                             = MSG_ID(GAPM, 0x36),
    /// Request to provide DH Key
    GAPM_GEN_DH_KEY_CMD                              = MSG_ID(GAPM, 0x39),
    /// Indicates the DH Key computation is complete and available
    GAPM_GEN_DH_KEY_IND                              = MSG_ID(GAPM, 0x3A),
    /// Retrieve Public Key
    GAPM_GET_PUB_KEY_CMD                             = MSG_ID(GAPM, 0x3B),
    /// Indicates the Public Key Pair value
    GAPM_PUB_KEY_IND                                 = MSG_ID(GAPM, 0x3C),
    /// Generate some OOB Data before a secure connection paring
    GAPM_GEN_OOB_DATA_CMD                            = MSG_ID(GAPM, 0x3D),
    /// Generated LE OOB Data for a following secure connection paring
    GAPM_LE_OOB_DATA_IND                             = MSG_ID(GAPM, 0x3E),
    /// Generated BT Classic OOB Data for a following secure connection paring
    GAPM_BT_OOB_DATA_IND                             = MSG_ID(GAPM, 0x3F),

    /* List Management Operations */
    /// Get local or peer address
    /// see struct #gapm_get_ral_addr_cmd
    GAPM_GET_RAL_ADDR_CMD                            = MSG_ID(GAPM, 0x50),
    /// Resolving address list address indication
    GAPM_RAL_ADDR_IND                                = MSG_ID(GAPM, 0x51),
    /// Set content of either white list or resolving list or periodic advertiser list
    /// see struct #gapm_list_set_wl_cmd
    /// see struct #gapm_list_set_ral_cmd
    /// see struct #gapm_list_set_pal_cmd
    GAPM_LIST_SET_CMD                                = MSG_ID(GAPM, 0x52),
    /// Indicate size of list indicated in GAPM_GET_DEV_CONFIG_CMD message
    /// see struct #gapm_list_size_ind
    GAPM_LIST_SIZE_IND                               = MSG_ID(GAPM, 0x53),

    /* Air Operations */
    /// Create an advertising, a scanning, an initiating or a periodic synchronization activity
    /// see struct #gapm_activity_create_cmd
    /// see struct #gapm_activity_create_adv_cmd
    GAPM_ACTIVITY_CREATE_CMD                         = MSG_ID(GAPM, 0x60),
    /// Start a previously created activity
    /// see struct #gapm_activity_start_cmd
    GAPM_ACTIVITY_START_CMD                          = MSG_ID(GAPM, 0x61),
    /// Stop either a given activity or all existing activities
    /// see struct #gapm_activity_stop_cmd
    GAPM_ACTIVITY_STOP_CMD                           = MSG_ID(GAPM, 0x62),
    /// Delete either a given activity or all existing activities
    /// see struct #gapm_activity_delete_cmd
    GAPM_ACTIVITY_DELETE_CMD                         = MSG_ID(GAPM, 0x63),
    /// Indicate that an activity has well been created
    /// see struct #gapm_activity_created_ind
    GAPM_ACTIVITY_CREATED_IND                        = MSG_ID(GAPM, 0x64),
    /// Indicate that an activity has been stopped and can be restarted
    /// see struct #gapm_activity_stopped_ind
    GAPM_ACTIVITY_STOPPED_IND                        = MSG_ID(GAPM, 0x65),
    /// Set either advertising data or scan response data or periodic advertising data
    /// see struct #gapm_set_adv_data_cmd
    GAPM_SET_ADV_DATA_CMD                            = MSG_ID(GAPM, 0x66),
    /// Indicate reception of an advertising report (periodic or not), a scan response report
    /// see struct #gapm_ext_adv_report_ind
    GAPM_EXT_ADV_REPORT_IND                          = MSG_ID(GAPM, 0x67),
    /// Indicate reception of a scan request
    /// see struct #gapm_scan_request_ind
    GAPM_SCAN_REQUEST_IND                            = MSG_ID(GAPM, 0x68),
    /// Indicate that synchronization has been successfully established with a periodic advertiser
    /// see struct #gapm_sync_established_ind
    GAPM_SYNC_ESTABLISHED_IND                        = MSG_ID(GAPM, 0x69),
    /// Control reception or not of Periodic Advertising Report in a Periodic Advertising Sync activity
    /// see struct #gapm_per_adv_report_ctrl_cmd
    GAPM_PER_ADV_REPORT_CTRL_CMD                     = MSG_ID(GAPM, 0x6A),
    /// Control capturing IQ samples from the Constant Tone Extension of periodic advertising packets
    GAPM_PER_SYNC_IQ_SAMPLING_CTRL_CMD               = MSG_ID(GAPM, 0x6B),
    /// Indicate reception of a IQ Report event over a periodic advertising sync activity
    GAPM_PER_ADV_IQ_REPORT_IND                       = MSG_ID(GAPM, 0x6C),
    /// Control CTE transmission in a periodic advertising activity
    GAPM_PER_ADV_CTE_TX_CTL_CMD                      = MSG_ID(GAPM, 0x6D),
    /// Name of peer device retrieve from device name query procedure
    GAPM_PEER_NAME_IND                               = MSG_ID(GAPM, 0x6E),
    /// Indicate reception of periodic advertising report that contains BIGInfo data
    /// see struct #gapm_big_info_adv_report_ind
    GAPM_BIG_INFO_ADV_REPORT_IND                     = MSG_ID(GAPM, 0x6F),
    /// Reception of an Inquiry report
    GAPM_INQUIRY_REPORT_IND                          = MSG_ID(GAPM, 0x70),

    /* LE Test Mode */
    /// Control of the test mode command
    GAPM_LE_TEST_MODE_CTRL_CMD                       = MSG_ID(GAPM, 0x90),
    /// Indicate end of test mode
    GAPM_LE_TEST_END_IND                             = MSG_ID(GAPM, 0x91),
    /// Indicate reception of a IQ report in LE test mode
    GAPM_LE_TEST_IQ_REPORT_IND                       = MSG_ID(GAPM, 0x92),

    /* BT Test Mode */
    /// Allows the local BR/EDR controller to enter test mode via LMP test commands.
    /// see #gapm_bt_write_loopback_mode
    GAPM_BT_WRITE_LOOPBACK_MODE_CMD                  = MSG_ID(GAPM, 0x93),
    /// Allows the local BR/EDR controller to enter test mode via LMP test commands.
    /// see #gapm_bt_write_simple_pairing_debug_mode
    GAPM_BT_ENABLE_DEVICE_UNDER_TEST_MODE_CMD        = MSG_ID(GAPM, 0x94),
    /// Configures the BR/EDR controller to use a predefined Diffie Hellman private key for simple pairing
    /// see #gapm_bt_write_simple_pairing_debug_mode
    GAPM_BT_WRITE_SIMPLE_PAIRING_DEBUG_MODE_CMD      = MSG_ID(GAPM, 0x95),
    /// Configures the BR/EDR controller to enable and disable the two test modes
    /// see #gapm_bt_write_secure_connections_test_mode
    GAPM_BT_WRITE_SECURE_CONNECTIONS_TEST_MODE_CMD   = MSG_ID(GAPM, 0x96),
    /// Event triggered when executed when reads controller's loopback mode procedure is completed
    /// see #GAPM_BT_READ_LOOPBACK_MODE
    GAPM_BT_LOOPBACK_IND                             = MSG_ID(GAPM, 0x97),

    /* Channel Assessment Mode */
    /// Set Channel Assessment mode
    GAPM_SET_CH_ASSESS_CMD                           = MSG_ID(GAPM, 0x98),

    /* Profile Management */
    /// Create new task for specific profile
    GAPM_PROFILE_TASK_ADD_CMD                        = MSG_ID(GAPM, 0xA0),
    /// Inform that profile task has been added.
    GAPM_PROFILE_ADDED_IND                           = MSG_ID(GAPM, 0xA1),

    /* ************************************************ */
    /* ---------------- DEBUG COMMANDS ---------------- */
    /* ************************************************ */
    /// Configure the Debug Platform I&Q Sampling generator
    GAPM_DBG_IQGEN_CFG_CMD                           = MSG_ID(GAPM, 0xE0),
    /// Undocumented security test command - Debug purpose only
    GAPM_DBG_SECURITY_TEST_CMD                       = MSG_ID(GAPM, 0xE1),
    /// Undocumented security test result - Debug purpose only
    GAPM_DBG_SECURITY_TEST_IND                       = MSG_ID(GAPM, 0xE2),

    /* ************************************************ */
    /* -------------- Internal usage only ------------- */
    /* ************************************************ */
    /// Message received to unknown task received
    GAPM_UNKNOWN_TASK_MSG                            = MSG_ID(GAPM, 0xF0),
};
/// @} GAPM_MSG_ID_API

/// @addtogroup GAPM_MSG_OPID_API Operation Identifiers
/// @ingroup GAPM_MSG_API
/// @{

/// GAP Manager operation type - application interface
/*@TRACE*/
enum gapm_operation
{
    /* No Operation (if nothing has been requested)     */
    /* ************************************************ */
    /// No operation.
    GAPM_NO_OP                                     = 0x00,

    /* Configuration operations                         */
    /* ************************************************ */
    /// Reset BLE subsystem: LL and HL.
    GAPM_RESET                                     = 0x01,
    /// Perform a platform reset - Debug only
    GAPM_PLF_RESET                                 = 0x02,
    /// Set device configuration
    GAPM_SET_DEV_CONFIG                            = 0x03,
    /// Set LE device channel map
    GAPM_SET_LE_CHANNEL_MAP                        = 0x04,
    /// Set IRK
    GAPM_SET_IRK                                   = 0x05,
    /// Set Device Name
    GAPM_SET_NAME                                  = 0x06,
    /// Insert into SDP a device identification record
    /// \if btdm see #gapm_set_sdp_device_identification_record \endif
    GAPM_SET_SDP_DEVICE_IDENTIFICATION_RECORD      = 0x07,

    /* Retrieve device information                      */
    /* ************************************************ */
    /// Get Local device version
    GAPM_GET_DEV_VERSION                           = 0x10,
    /// Get Local device BD Address
    GAPM_GET_DEV_BDADDR                            = 0x11,
    /// Get device advertising power level
    GAPM_GET_DEV_ADV_TX_POWER                      = 0x12,
    /// Get White List Size.
    GAPM_GET_WLIST_SIZE                            = 0x13,
    /// Retrieve Antenna information
    GAPM_GET_ANTENNA_INFO                          = 0x14,
    /// Get memory usage - Debug only
    GAPM_DBG_GET_MEM_INFO                          = 0x15,
    /// Get Suggested Default LE Data Length
    GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN            = 0x16,
    /// Get Maximum LE Data Length
    GAPM_GET_MAX_LE_DATA_LEN                       = 0x17,
    /// Get number of available advertising sets
    GAPM_GET_NB_ADV_SETS                           = 0x18,
    /// Get maximum advertising data length supported by the controller
    GAPM_GET_MAX_LE_ADV_DATA_LEN                   = 0x19,
    /// Get minimum and maximum transmit powers supported by the controller
    GAPM_GET_DEV_TX_PWR                            = 0x1A,
    /// Get the RF Path Compensation values used in the TX Power Level and RSSI calculation
    GAPM_GET_DEV_RF_PATH_COMP                      = 0x1B,
    /// Get statistics - Debug only
    GAPM_DBG_GET_STATS_INFO                        = 0x1C,

    /* Security / Encryption Toolbox                    */
    /* ************************************************ */
    /// Resolve device address
    GAPM_RESOLV_ADDR                               = 0x30,
    /// Generate a random address
    GAPM_GEN_RAND_ADDR                             = 0x31,
    /// Use the controller's AES-128 block
    GAPM_USE_ENC_BLOCK                             = 0x32,
    /// Generate a 8-byte random number
    GAPM_GEN_RAND_NB                               = 0x33,
    /// Generate DH_Key
    GAPM_GEN_DH_KEY                                = 0x34,
    /// Retrieve Public Key
    GAPM_GET_PUB_KEY                               = 0x35,
    /// Generate LE OOB Data
    GAPM_GEN_LE_OOB_DATA                           = 0x36,
    /// Generate BT Classic OOB Data
    GAPM_GEN_BT_OOB_DATA                           = 0x37,

    /* List Management for air operations               */
    /* ************************************************ */
    /// Get resolving address list size
    GAPM_GET_RAL_SIZE                              = 0x50,
    /// Get resolving local address
    GAPM_GET_RAL_LOC_ADDR                          = 0x51,
    /// Get resolving peer address
    GAPM_GET_RAL_PEER_ADDR                         = 0x52,
    /// Set content of white list
    GAPM_SET_WL                                    = 0x53,
    /// Set content of resolving list
    GAPM_SET_RAL                                   = 0x54,
    /// Set content of periodic advertiser list
    GAPM_SET_PAL                                   = 0x55,
    /// Get periodic advertiser list size
    GAPM_GET_PAL_SIZE                              = 0x56,

    /* Air Operations                                   */
    /* ************************************************ */
    /// Create advertising activity
    GAPM_CREATE_ADV_ACTIVITY                       = 0x60,
    /// Create scanning activity
    GAPM_CREATE_SCAN_ACTIVITY                      = 0x61,
    /// Create initiating activity
    GAPM_CREATE_INIT_ACTIVITY                      = 0x62,
    /// Create periodic synchronization activity
    GAPM_CREATE_PERIOD_SYNC_ACTIVITY               = 0x63,
    /// Start an activity
    GAPM_START_ACTIVITY                            = 0x64,
    /// Stop an activity
    GAPM_STOP_ACTIVITY                             = 0x65,
    /// Stop all activities
    GAPM_STOP_ALL_ACTIVITIES                       = 0x66,
    /// Delete an activity
    GAPM_DELETE_ACTIVITY                           = 0x67,
    /// Delete all activities
    GAPM_DELETE_ALL_ACTIVITIES                     = 0x68,
    /// Set advertising data
    GAPM_SET_ADV_DATA                              = 0x69,
    /// Set scan response data
    GAPM_SET_SCAN_RSP_DATA                         = 0x6A,
    /// Set periodic advertising data
    GAPM_SET_PERIOD_ADV_DATA                       = 0x6B,
    /// Enable/Disable reception of periodic advertising report
    GAPM_PER_ADV_REPORT_CTRL                       = 0x6C,
    /// Enable / Disable IQ sampling
    GAPM_PER_SYNC_IQ_SAMPLING_CTRL                 = 0x6D,
    /// Enable / Disable CTE transmission
    GAPM_PER_ADV_CTE_TX_CTL                        = 0x6E,
    /// Create inquiry activity
    GAPM_CREATE_INQUIRY_ACTIVITY                   = 0x70,
    /// Create inquiry scan activity
    GAPM_CREATE_INQUIRY_SCAN_ACTIVITY              = 0x71,
    /// Create page activity
    GAPM_CREATE_PAGE_ACTIVITY                      = 0x72,
    /// Create page scan activity
    GAPM_CREATE_PAGE_SCAN_ACTIVITY                 = 0x73,

    /* LE Direct Test Mode                              */
    /* ************************************************ */
    /// Stop the test mode
    GAPM_LE_TEST_STOP                              = 0x90,
    /// Start RX Test Mode
    GAPM_LE_TEST_RX_START                          = 0x91,
    /// Start TX Test Mode
    GAPM_LE_TEST_TX_START                          = 0x92,

    /* BT Test Mode                                     */
    /* ************************************************ */
    /// Allows the local BR/EDR controller to enter test mode via LMP test commands.
    GAPM_BT_WRITE_LOOPBACK_MODE                    = 0x93,
    /// Configures the BR/EDR controller to use a predefined Diffie Hellman private key
    GAPM_BT_ENABLE_DEVICE_UNDER_TEST_MODE          = 0x94,
    /// Configures the BR/EDR controller to use a predefined Diffie Hellman private key for simple pairing
    GAPM_BT_WRITE_SIMPLE_PAIRING_DEBUG_MODE        = 0x95,
    /// Configures the BR/EDR controller to enable and disable the two test modes
    GAPM_BT_WRITE_SECURE_CONNECTIONS_TEST_MODE     = 0x96,
    /// Read BT loopback mode using #GAPM_GET_DEV_INFO_CMD
    /// see #gapm_bt_read_loopback_mode
    GAPM_BT_READ_LOOPBACK_MODE                     = 0x97,

    /* Channel Assessment Mode                          */
    /* ************************************************ */
    /// Start Channel Assessment Mode
    GAPM_CH_ASSESS_START                           = 0x98,
    /// Stop Channel Assessment Mode
    GAPM_CH_ASSESS_STOP                            = 0x99,

    /* Profile Management                               */
    /* ************************************************ */
    /// Create new task for specific profile
    GAPM_PROFILE_TASK_ADD                          = 0xA0,

    /* Debug Commands                                   */
    /* ************************************************ */
    /// Configure the Debug Platform I&Q Sampling generator
    GAPM_DBG_IQGEN_CFG                             = 0xE0,
    /// Undocumented security test command - Debug purpose only
    GAPM_DBG_SECURITY_TEST                         = 0xE1,
};
/// @} GAPM_MSG_OPID_API


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// @addtogroup GAPM_MSG_STRUCT_API Message Structures
/// @ingroup GAPM_MSG_API
/// @{

/// Operation command structure in order to keep requested operation.
struct gapm_operation_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    uint8_t operation;
};

/// Command complete event data structure
/*@TRACE*/
struct gapm_cmp_evt
{
    /// Requested operation type (see enum #gapm_operation)
    uint8_t  operation;
    /// Status of the request
    uint16_t status;
    /// Activity index (valid only for air operation, else discard)
    uint8_t  actv_idx;
};

///  Reset link layer and the host command
/*@TRACE*/
struct gapm_reset_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    /// - #GAPM_RESET: Reset BLE subsystem: LL and HL.
    uint8_t operation;
};

/// Set device configuration command
/*@TRACE*/
struct gapm_set_dev_config_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_SET_DEV_CONFIG: Set device configuration
    uint8_t         operation;
    /// Device configuation
    gapm_config_t   cfg;
};


/// Parameters of #GAPM_SET_NAME_CMD
/*@TRACE*/
struct gapm_set_name_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_SET_NAME: Set device name
    uint8_t operation;
    /// Size of the device name
    uint8_t name_len;
    /// Device Name
    uint8_t name[__ARRAY_EMPTY];
};


/// Parameters of #GAPM_GET_DEV_INFO_CMD
/*@TRACE*/
struct gapm_get_dev_info_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_GET_DEV_VERSION: Get Local device version
    ///  - #GAPM_GET_DEV_BDADDR: Get Local device BD Address
    ///  - #GAPM_GET_DEV_ADV_TX_POWER: Get device advertising power level
    ///  - #GAPM_DBG_GET_MEM_INFO: Get memory usage (debug only)
    ///  - #GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN: Get Suggested Default LE Data Length
    ///  - #GAPM_GET_MAX_LE_DATA_LEN: Get Maximum LE Data Length
    ///  - #GAPM_GET_NB_ADV_SETS: Read number of advertising sets currently supported by the controller
    ///  - #GAPM_GET_MAX_LE_ADV_DATA_LEN: Get maximum data length for advertising data
    ///  - #GAPM_GET_ANTENNA_INFO: Retrieve Antenna information
    ///  - #GAPM_BT_READ_LOOPBACK_MODE: Read BT loopback mode
    uint8_t operation;
};

/// Local device version indication event
/*@TRACE*/
struct gapm_dev_version_ind
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
};

/// Local device BD Address indication event
/*@TRACE*/
struct gapm_dev_bdaddr_ind
{
    /// Local device address information
    gap_bdaddr_t addr;
    /// Activity index
    uint8_t actv_idx;
};


/// Parameters of the @ref GAPM_GEN_OOB_DATA_CMD message
/*@TRACE*/
struct gapm_gen_oob_data_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    /// - #GAPM_GEN_LE_OOB_DATA: LE OOB data
    /// - #GAPM_GEN_BT_OOB_DATA: BT-Classic OOB data
    uint8_t operation;
};



/// Create new task for specific profile
/*@TRACE*/
struct gapm_profile_task_add_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_PROFILE_TASK_ADD: Add new profile task
    uint8_t  operation;
    /// Service Security level - Only for a GATT service (see enum #gatt_svc_info_bf)
    uint8_t  sec_lvl;
    /// GATT user priority
    uint8_t  user_prio;
    /// Profile Application identifier (use profile identifier)
    uint16_t prf_api_id;
    /// Application task number
    uint16_t app_task;
    /// Service start handle
    /// Only applies for services - Ignored by collectors
    /// 0: dynamically allocated in Attribute database
    uint16_t start_hdl;
    /// 32 bits value that contains value to initialize profile (database parameters, etc...)
    uint32_t param[__ARRAY_EMPTY];
};

/// Inform that profile task has been added.
/*@TRACE*/
struct gapm_profile_added_ind
{
    /// Profile task identifier
    uint16_t prf_task_id;
    /// Profile task number allocated
    uint16_t prf_task_nb;
    /// Service start handle
    /// Only applies for services - Ignored by collectors
    uint16_t start_hdl;
};

/// Indicate that a message has been received on an unknown task
/*@TRACE*/
struct gapm_unknown_task_ind
{
    /// Message identifier
    uint16_t msg_id;
    /// Task identifier
    uint16_t task_id;
};

/// Create an advertising, a scanning, an initiating, a periodic synchonization activity command (common)
/*@TRACE*/
struct gapm_activity_create_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_CREATE_ADV_ACTIVITY: Create advertising activity
    ///  - #GAPM_CREATE_SCAN_ACTIVITY: Create scanning activity
    ///  - #GAPM_CREATE_INIT_ACTIVITY: Create initiating activity
    ///  - #GAPM_CREATE_PERIOD_SYNC_ACTIVITY: Create periodic synchronization activity
	///  - #GAPM_CREATE_INQUIRY_ACTIVITY: Create inquiry activity
	///  - #GAPM_CREATE_INQUIRY_SCAN_ACTIVITY: Create inquiry scan activity
	///  - #GAPM_CREATE_PAGE_ACTIVITY: Create page activity
	///  - #GAPM_CREATE_PAGE_SCAN_ACTIVITY: Create page scan activity
    uint8_t operation;
    /// Own address type (see enum #gapm_own_addr)
    uint8_t own_addr_type;
};


/// Start a given activity command
/*@TRACE*/
struct gapm_activity_start_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_START_ACTIVITY: Start a given activity
    uint8_t operation;
    /// Activity identifier
    uint8_t actv_idx;
    /// Activity parameters
    uint16_t u_param[__ARRAY_EMPTY]; // 16-bit aligned parameters
};

/// Stop one or all activity(ies) command
/*@TRACE*/
struct gapm_activity_stop_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_STOP_ACTIVITY: Stop a given activity
    ///  - #GAPM_STOP_ALL_ACTIVITIES: Stop all existing activities
    uint8_t operation;
    /// Activity identifier - used only if operation is GAPM_STOP_ACTIVITY
    uint8_t actv_idx;
};

/// Delete one or all activity(ies) command
/*@TRACE*/
struct gapm_activity_delete_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_DELETE_ACTIVITY: Delete a given activity
    ///  - #GAPM_DELETE_ALL_ACTIVITIES: Delete all existing activities
    uint8_t operation;
    /// Activity identifier - used only if operation is GAPM_DELETE_ACTIVITY
    uint8_t actv_idx;
};

/// Indicate creation of an activity
/*@TRACE
 @trc_exec activity_map[$actv_idx] = $actv_type
 activity_map = {}*/
struct gapm_activity_created_ind
{
    /// Activity identifier
    uint8_t actv_idx;
    /// Activity type (see enum #gapm_actv_type)
    uint8_t actv_type;
    /// Selected TX power for advertising activity
    int8_t  tx_pwr;
};

/// Indicate that an activity has been stopped
/*@TRACE*/
struct gapm_activity_stopped_ind
{
    /// Activity identifier
    uint8_t actv_idx;
    /// Activity type (see enum #gapm_actv_type)
    uint8_t actv_type;
    /// Activity stop reason (see enum #hl_err)
    uint16_t reason;
    /// In case of periodic advertising, indicate if periodic advertising has been stopped
    uint8_t per_adv_stop;
};



/// Indicate that an unknown message has been received
/*@TRACE*/
struct gapm_unknown_msg_ind
{
    /// Unknown message id
    uint16_t unknown_msg_id;
};


/*
 * MACROS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/// @} GAPM_MSG_STRUCT_API

#endif /* _GAPM_MSG_H_ */
