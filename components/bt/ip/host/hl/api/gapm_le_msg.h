/**
 ****************************************************************************************
 *
 * @file gapm_le_msg.h
 *
 * @brief Generic Access Profile Manager Message API. Low Energy
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


#ifndef _GAPM_LE_MSG_H_
#define _GAPM_LE_MSG_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gapm_msg.h"
#include "gapm_le.h"
#include "gapm_le_adv.h"
#include "gapm_le_scan.h"
#include "gapm_le_init.h"
#include "gapm_le_per_sync.h"
#include "gapm_le_test.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// @addtogroup GAPM_MSG_STRUCT_API Message Structures
/// @ingroup GAPM_MSG_API
/// @{

/// Set new IRK
/*@TRACE*/
struct gapm_set_irk_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_SET_IRK: Set device Identity Resolving key
    uint8_t operation;
    /// Device IRK used for resolvable random BD address generation (LSB first)
    gap_sec_key_t irk;
};


/// Set device channel map
/*@TRACE*/
struct gapm_set_le_channel_map_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_SET_LE_CHANNEL_MAP: Set device channel map.
    uint8_t operation;
    /// Channel map
    le_chnl_map_t chmap;
};


/// Advertising channel Tx power level indication event
/*@TRACE*/
struct gapm_dev_adv_tx_power_ind
{
    /// Advertising channel Tx power level
    int8_t     power_lvl;
};


/// Indication containing controller antenna information
/*@TRACE*/
struct gapm_antenna_inf_ind
{
    /// Supported switching sampling rates bit field (see enum #gapm_switch_sampling_rate)
    uint8_t     supp_switching_sampl_rates;
    /// Number of antennae
    uint8_t     antennae_num;
    /// Max length of switching pattern (number of antenna IDs in the pattern)
    uint8_t     max_switching_pattern_len;
    /// Max CTE length
    uint8_t     max_cte_len;
};

/// Resolving Address indication event
/*@TRACE*/
struct gapm_ral_addr_ind
{
    /// Requested operation type (see enum #gapm_operation)
    uint8_t operation;
    /// Resolving List address
    gap_bdaddr_t addr;
};

/// Resolve Address command
/*@TRACE*/
struct gapm_resolv_addr_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_RESOLV_ADDR: Resolve device address
    uint8_t operation;
    /// Number of provided IRK (sahlle be > 0)
    uint8_t nb_key;
    /// Resolvable random address to solve
    gap_addr_t addr;
    /// Array of IRK used for address resolution (MSB -> LSB)
    gap_sec_key_t irk[__ARRAY_EMPTY];
};

/// Indicate that resolvable random address has been solved
/*@TRACE*/
struct gapm_addr_solved_ind
{
    /// Resolvable random address solved
    gap_addr_t addr;
    /// IRK that correctly solved the random address
    gap_sec_key_t irk;
};

/// Name of peer device indication
/*@TRACE*/
struct gapm_peer_name_ind
{
    /// peer device bd address
    gap_bdaddr_t addr;
    /// peer device name length
    uint8_t      name_len;
    /// peer device name
    uint8_t      name[__ARRAY_EMPTY];
};

/// Generate a random address.
/*@TRACE*/
struct gapm_gen_rand_addr_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_GEN_RAND_ADDR: Generate a random address
    uint8_t  operation;
    /// Random address type #gap_rnd_addr_type
    ///  - #GAP_BD_ADDR_STATIC: Static random address
    ///  - #GAP_BD_ADDR_NON_RSLV: Private non resolvable address
    ///  - #GAP_BD_ADDR_RSLV: Private resolvable address
    uint8_t rnd_type;
};

/// Parameters of the @ref GAPM_USE_ENC_BLOCK_CMD message
/*@TRACE*/
struct gapm_use_enc_block_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    /// (shall be #GAPM_USE_ENC_BLOCK)
    uint8_t operation;
    /// True to cipher; False to de-cipher
    bool    cipher;
    /// Operand 1
    uint8_t operand_1[GAP_KEY_LEN];
    /// Operand 2
    uint8_t operand_2[GAP_KEY_LEN];
};

/// Parameters of the @ref GAPM_USE_ENC_BLOCK_IND message
/*@TRACE*/
struct gapm_use_enc_block_ind
{
    /// Result (16 bytes)
    uint8_t result[GAP_KEY_LEN];
};

/// Parameters of the @ref GAPM_GEN_DH_KEY_CMD message
/*@TRACE*/
struct gapm_gen_dh_key_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    /// (shall be #GAPM_GEN_DH_KEY)
    uint8_t      operation;
    /// peer public key
    public_key_t pub_key;
};

/// Parameters of the @ref GAPM_GEN_DH_KEY_IND message
/*@TRACE*/
struct gapm_gen_dh_key_ind
{
    /// Result (32 bytes)
    uint8_t result[GAP_P256_KEY_LEN];
};

/// Parameters of the @ref GAPM_GET_PUB_KEY_CMD message
/*@TRACE*/
struct gapm_get_pub_key_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    /// (shall be #GAPM_GET_PUB_KEY)
    uint8_t operation;
};

/// Parameters of the @ref GAPM_PUB_KEY_IND message
/*@TRACE*/
struct gapm_pub_key_ind
{
    /// X coordinate
    uint8_t pub_key_x[GAP_P256_KEY_LEN];
    /// Y coordinate
    uint8_t pub_key_y[GAP_P256_KEY_LEN];
};


/// Parameters of the @ref GAPM_LE_OOB_DATA_IND message
/*@TRACE*/
struct gapm_le_oob_data_ind
{
    /// Generated OOB data
    gap_oob_t oob;
};


/// Parameters of the @ref GAPM_GEN_RAND_NB_CMD message
/*@TRACE*/
struct gapm_gen_rand_nb_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    /// (shall be #GAPM_GEN_RAND_NB)
    uint8_t operation;
};

/// Parameters of the @ref GAPM_GEN_RAND_NB_IND message
/*@TRACE*/
struct gapm_gen_rand_nb_ind
{
    /// Generation Random Number (16 bytes)
    uint8_t randnb[GAP_AES_LEN];
};

/// Indicates suggested default data length
/*@TRACE*/
struct gapm_sugg_dflt_data_len_ind
{
    ///Host's suggested value for the Controller's maximum transmitted number of payload octets
    uint16_t suggted_max_tx_octets;
    ///Host's suggested value for the Controller's maximum packet transmission time
    uint16_t suggted_max_tx_time;
};

/// Indicates maximum data length
/*@TRACE*/
struct gapm_max_le_data_len_ind
{
    ///Maximum number of payload octets that the local Controller supports for transmission
    uint16_t suppted_max_tx_octets;
    ///Maximum time, in microseconds, that the local Controller supports for transmission
    uint16_t suppted_max_tx_time;
    ///Maximum number of payload octets that the local Controller supports for reception
    uint16_t suppted_max_rx_octets;
    ///Maximum time, in microseconds, that the local Controller supports for reception
    uint16_t suppted_max_rx_time;
};

/// Control LE Test Mode command
struct gapm_le_test_mode_ctrl_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_LE_TEST_STOP: Stop Test mode
    ///  - #GAPM_LE_TEST_RX_START: Start RX Test Mode
    ///  - #GAPM_LE_TEST_TX_START: Start TX Test Mode
    uint8_t operation;
    /// Tx or Rx Channel (Range 0x00 to 0x27)
    uint8_t channel;
    /// Length in bytes of payload data in each packet (only valid for TX mode, range 0x00-0xFF)
    uint8_t tx_data_length;
    /// Packet Payload type (only valid for TX mode see enum #gap_pkt_pld_type)
    uint8_t tx_pkt_payload;
    /// Test PHY rate (see enum #gap_phy_val)
    uint8_t phy;
    /// Modulation Index (only valid for RX mode see enum #gap_modulation_idx)
    uint8_t modulation_idx;
    /// CTE length (in 8us unit) (Expected for RX Mode)
    uint8_t cte_len;
    /// CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us) (Expected for TX Mode)
    uint8_t cte_type;
    /// Slot durations (only valid for RX mode)
    uint8_t slot_dur;
    /// Transmit power level in dBm (0x7E: minimum | 0x7F: maximum | range: -127 to +20)
    int8_t  tx_pwr_lvl;
    /// Length of switching pattern (number of antenna IDs in the pattern)
    uint8_t switching_pattern_len;
    /// Antenna IDs
    uint8_t antenna_id[__ARRAY_EMPTY];
};

/// Parameters of #GAPM_LE_TEST_END_IND
struct gapm_le_test_end_ind
{
    /// Number of received packets
    uint16_t nb_packet_received;
};

/// Indicate reception of a IQ Report event over a direct test mode.
/*@TRACE*/
struct gapm_le_test_iq_report_ind
{
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
    /// Periodic Adv Event Counter
    uint16_t pa_evt_cnt;
    /// Number of samples
    uint8_t  nb_samples;
    /// I/Q sample
    gap_iq_sample_t sample[__ARRAY_EMPTY];
};

/// Create an advertising activity command
/*@TRACE*/
struct gapm_activity_create_adv_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_CREATE_ADV_ACTIVITY: Create advertising activity
    uint8_t                 operation;
    /// Own address type (see enum #gapm_own_addr)
    uint8_t                 own_addr_type;
    /// Advertising type (see enum #gapm_adv_type)
    uint8_t                 type;
    /// Advertising parameters (optional, shall be present only if operation is GAPM_CREATE_ADV_ACTIVITY)
    gapm_adv_create_param_t adv_param;
    /// Configuration for secondary advertising (valid only if advertising type is
    /// GAPM_ADV_TYPE_EXTENDED or GAPM_ADV_TYPE_PERIODIC)
    gapm_adv_second_cfg_t   second_cfg;
    /// Configuration for periodic advertising (valid only if advertising type is
    /// GAPM_ADV_TYPE_PERIODIC)
    gapm_adv_period_cfg_t   period_cfg;
    /// Configuration for  constant tone extension (valid only if advertising type is
    /// GAPM_ADV_TYPE_PERIODIC)
    gapm_adv_cte_cfg_t      cte_cfg;
    /// Length of switching pattern (number of antenna IDs in the pattern)
    uint8_t                 switching_pattern_len;
    /// Antenna IDs
    uint8_t                 antenna_id[__ARRAY_EMPTY];
};


/// Set advertising, scan response or periodic advertising data command
/*@TRACE*/
struct gapm_set_adv_data_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_SET_ADV_DATA: Set advertising data
    ///  - #GAPM_SET_SCAN_RSP_DATA: Set scan response data
    ///  - #GAPM_SET_PERIOD_ADV_DATA: Set periodic advertising data
    uint8_t operation;
    /// Activity identifier
    uint8_t actv_idx;
    /// Data length
    uint16_t length;
    /// Data
    uint8_t data[__ARRAY_EMPTY];
};

/// Indicate reception of scan request
/*@TRACE*/
struct gapm_scan_request_ind
{
    /// Activity identifier
    uint8_t actv_idx;
    /// Transmitter device address
    gap_bdaddr_t trans_addr;
};

/// Indicate reception of advertising, scan response or periodic advertising data
/*@TRACE*/
struct gapm_ext_adv_report_ind
{
    /// Activity identifier
    uint8_t actv_idx;
    /// Bit field providing information about the received report (see enum #gapm_adv_report_info_bf)
    uint8_t info;
    /// Transmitter device address
    gap_bdaddr_t trans_addr;
    /// Target address (in case of a directed advertising report)
    gap_bdaddr_t target_addr;
    /// TX power (in dBm)
    int8_t tx_pwr;
    /// RSSI (between -127 and +20 dBm)
    int8_t rssi;
    /// Primary PHY on which advertising report has been received
    uint8_t phy_prim;
    /// Secondary PHY on which advertising report has been received
    uint8_t phy_second;
    /// Advertising SID
    /// Valid only for periodic advertising report
    uint8_t adv_sid;
    /// Periodic advertising interval (in unit of 1.25ms, min is 7.5ms)
    /// Valid only for periodic advertising report
    uint16_t period_adv_intv;
    /// Report length
    uint16_t length;
    /// Report
    uint8_t data[__ARRAY_EMPTY];
};

/// Indicate reception of periodic advertising report that contains BIGInfo data
/*@TRACE*/
struct gapm_big_info_adv_report_ind
{
    /// Activity identifier
    uint8_t         actv_idx;
    /// Big Info report
    gap_big_info_t  report;
};


/// Indicate that synchronization has been established with a periodic advertiser
/*@TRACE*/
struct gapm_sync_established_ind
{
    /// Activity identifier
    uint8_t           actv_idx;
    /// PHY on which synchronization has been established (#gap_phy_val)
    uint8_t           phy;
    /// Periodic advertising interval (in unit of 1.25ms, min is 7.5ms)
    uint16_t          intv;
    /// Advertising SID
    uint8_t           adv_sid;
    /// Advertiser clock accuracy (see enum #gapm_clk_acc)
    uint8_t           clk_acc;
    /// Advertiser address
    gap_bdaddr_t      addr;
    /// Only valid for a Periodic Advertising Sync Transfer, else ignore
    uint16_t          serv_data;
};

/// Read local or peer address
/*@TRACE*/
struct gapm_get_ral_addr_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_GET_RAL_LOC_ADDR: Get resolving local address
    ///  - #GAPM_GET_RAL_PEER_ADDR: Get resolving peer address
    uint8_t operation;
    /// Peer device identity
    gap_bdaddr_t peer_identity;
};

/// Set content of either white list or resolving list or periodic advertiser list command (common part)
struct gapm_list_set_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_SET_WL: Set white list content
    ///  - #GAPM_SET_RAL: Set resolving list content
    ///  - #GAPM_SET_PAL: Set periodic advertiser list content
    uint8_t operation;
    /// Number of entries to be added in the list. 0 means that list content has to be cleared
    uint8_t size;
};

/// Set content of white list
/*@TRACE*/
struct gapm_list_set_wl_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_SET_WL: Set white list content
    uint8_t operation;
    /// Number of entries to be added in the list. 0 means that list content has to be cleared
    uint8_t size;
    /// List of entries to be added in the list
    gap_bdaddr_t wl_info[__ARRAY_EMPTY];
};

/// Set content of resolving list command
/*@TRACE*/
struct gapm_list_set_ral_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_SET_RAL: Set resolving list content
    uint8_t operation;
    /// Number of entries to be added in the list. 0 means that list content has to be cleared
    uint8_t size;
    /// List of entries to be added in the list
    gap_ral_dev_info_t ral_info[__ARRAY_EMPTY];
};

/// Set content of periodic advertiser list command
/*@TRACE*/
struct gapm_list_set_pal_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_SET_PAL: Set periodic advertiser list content
    uint8_t              operation;
    /// Number of entries to be added in the list. 0 means that list content has to be cleared
    uint8_t              size;
    /// List of entries to be added in the list
    gap_per_adv_bdaddr_t pal_info[__ARRAY_EMPTY];
};


/// List Set union
/*@TRACE
 @trc_ref gapm_operation
 gapm_list_set_cmd = gapm_u_list_set
*/
union gapm_u_list_set
{
    /// Requested operation type (see enum #gapm_operation)
    uint8_t operation;

    /// Set white list command structure
    //@trc_union operation == GAPM_SET_WL
    struct gapm_list_set_wl_cmd list_set_wl_cmd;
    /// Set resolving list command structure
    //@trc_union operation == GAPM_SET_RAL
    struct gapm_list_set_ral_cmd list_set_ral_cmd;
    /// Set periodic advertising list command structure
    //@trc_union operation == GAPM_SET_PAL
    struct gapm_list_set_pal_cmd list_set_pal_cmd;
};

/// List Size indication event
/*@TRACE*/
struct gapm_list_size_ind
{
    /// Requested operation type (see enum #gapm_operation)
    ///     - #GAPM_SET_WL
    ///     - #GAPM_SET_RAL
    ///     - #GAPM_SET_PAL
    uint8_t operation;
    /// List size
    uint8_t size;
};

/// Maximum advertising data length indication event
/*@TRACE*/
struct gapm_max_adv_data_len_ind
{
    /// Maximum advertising data length supported by controller
    uint16_t length;
};

/// Number of available advertising sets indication event
/*@TRACE*/
struct gapm_nb_adv_sets_ind
{
    /// Number of available advertising sets
    uint8_t nb_adv_sets;
};

/// Indicate the transmit powers supported by the controller
/*@TRACE*/
struct gapm_dev_tx_pwr_ind
{
    /// Minimum TX power
    int8_t min_tx_pwr;
    /// Maximum TX power
    int8_t max_tx_pwr;
};

/// Indicate the RF path compensation values
/*@TRACE*/
struct gapm_dev_rf_path_comp_ind
{
    /// RF TX path compensation
    int16_t tx_path_comp;
    /// RF RX path compensation
    int16_t rx_path_comp;
};

/// Control reception or not of Periodic Advertising Report in a Periodic Advertising Sync activity
/*@TRACE*/
struct gapm_per_adv_report_ctrl_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_PER_ADV_REPORT_CTRL: Enable / Disable reception of periodic advertising report
    uint8_t  operation;
    /// Activity identifier
    uint8_t  actv_idx;
    /// bit field that contains list of reports that are enabled or not (see enum #gapm_report_en_bf)
    uint8_t  report_en_bf;
};

/// Control capturing IQ samples from the Constant Tone Extension of periodic advertising packets
/*@TRACE*/
struct gapm_per_sync_iq_sampling_ctrl_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_PER_SYNC_IQ_SAMPLING_CTRL: Enable / Disable IQ sampling
    uint8_t  operation;
    /// Activity identifier
    uint8_t  actv_idx;
    /// True to enable IQ sampling, false to disable
    uint8_t  enable;
    /// Slot durations (1: 1us | 2: 2us)
    uint8_t  slot_dur;
    /// Max sampled CTEs
    uint8_t  max_sampl_cte;
    /// Length of switching pattern
    uint8_t  switching_pattern_len;
    /// Antenna IDs
    uint8_t  antenna_id[__ARRAY_EMPTY];
};

/// Indicate reception of a IQ Report event over a periodic advertising sync activity
/*@TRACE*/
struct gapm_per_adv_iq_report_ind
{
    /// Activity identifier
    uint8_t  actv_idx;
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
    /// Periodic Adv Event Counter
    uint16_t pa_evt_cnt;
    /// Number of samples
    uint8_t  nb_samples;
    /// I/Q sample
    gap_iq_sample_t sample[__ARRAY_EMPTY];
};

/// Control CTE transmission in a periodic advertising activity
/*@TRACE*/
struct gapm_per_adv_cte_tx_ctl_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_PER_ADV_CTE_TX_CTL: Control CTE transmission in a periodic advertising activity
    uint8_t  operation;
    /// Activity identifier
    uint8_t  actv_idx;
    /// True to enable CTE transmission, false to disable
    uint8_t  enable;
};


/// Configure the Debug Platform I&Q Sampling generator
/*@TRACE*/
struct gapm_dbg_iqgen_cfg_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_DBG_IQGEN_CFG: Configure the Debug Platform I&Q Sampling generator
    uint8_t                 operation;
    /// Antenna switch/sample control
    /// bit[0]: 0: up-sweep; 1: up-down sweep (internal switching mode)
    /// bit[1]: 0: 1us intervals; 1: 2us intervals (internal switching mode)
    /// bit[2]: 0: internal switching mode; 1: baseband switching mode
    uint8_t                 mode;
    /// Number of antenna patterns
    uint8_t                 nb_antenna;
    /// I/Q sample control
    gapm_dbg_iq_ctrl_t      iq_ctrl[__ARRAY_EMPTY];
};

/// Start Channel Assessment activity
/*@TRACE*/
struct gapm_ch_assess_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_CH_ASSESS_START: Start Channel Assessment Mode
    uint8_t operation;
    /// Scan window duration in us
    uint32_t scan_win_duration;
    /// Minimum Channel Scanning event in us
    uint32_t scan_duration_min;
    /// Maximum Channel Scanning event in us
    uint32_t scan_duration_max;
    /// Channel Scanning interval in Time = N*1.25ms
    uint16_t intv;
};


/// Activity parameters
typedef union gapm_le_start_param
{
    /// Additional advertising parameters (for advertising activity)
    //@trc_union @activity_map[$parent.actv_idx] == GAPM_ACTV_TYPE_ADV
    gapm_adv_param_t                adv_add_param;
    /// Scan parameters (for scanning activity)
    //@trc_union @activity_map[$parent.actv_idx] == GAPM_ACTV_TYPE_SCAN
    gapm_scan_param_t               scan_param;
    /// Initiating parameters (for initiating activity)
    //@trc_union @activity_map[$parent.actv_idx] == GAPM_ACTV_TYPE_INIT
    gapm_init_param_t               init_param;
    /// Periodic synchronization parameters (for periodic synchronization activity)
    //@trc_union @activity_map[$parent.actv_idx] == GAPM_ACTV_TYPE_PER_SYNC
    gapm_per_sync_param_t           per_sync_param;
} gapm_le_start_param_u;

/// Start a given activity command
/*@TRACE*/
struct gapm_le_activity_start_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_START_ACTIVITY: Start a given activity
    uint8_t operation;
    /// Activity identifier
    uint8_t actv_idx;
    /// Activity parameters
    gapm_le_start_param_u u_param;
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

#endif /* _GAPM_LE_MSG_H_ */
