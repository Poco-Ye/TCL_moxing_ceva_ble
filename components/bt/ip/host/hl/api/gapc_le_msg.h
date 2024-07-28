/**
 ****************************************************************************************
 *
 * @file gapc_le_msg.h
 *
 * @brief Generic Access Profile Controller  Message API. - Low Energy
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */
#ifndef _GAPC_LE_MSG_H_
#define _GAPC_LE_MSG_H_


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gapc_msg.h"
#include "gapc_le.h"

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// @addtogroup GAPC_MSG_STRUCT_API Message Structures
/// @ingroup GAPC_MSG_API
/// @{

/// Indicate that a le connection has been established
/*@TRACE*/
struct gapc_le_connection_req_ind
{
    /// Connection index
    uint8_t    conidx;
    /// Connection handle
    uint16_t   conhdl;
    /// Connection interval
    uint16_t   con_interval;
    /// Connection latency
    uint16_t   con_latency;
    /// Link supervision timeout
    uint16_t   sup_to;
    /// Clock accuracy
    uint8_t    clk_accuracy;
    /// Peer address type
    uint8_t    peer_addr_type;
    /// Peer BT address
    gap_addr_t peer_addr;
    /// Role of device in connection (0 = Central / 1 = Peripheral)
    uint8_t    role;
};

/// Indication of peer features info
/*@TRACE*/
struct gapc_peer_features_ind
{
    /// Connection index
    uint8_t conidx;
    /// 8-byte array for LE features
    uint8_t features[GAP_LE_FEATS_LEN];
};


/// Indication of ongoing connection Channel Map
/*@TRACE*/
struct gapc_con_channel_map_ind
{
    /// Connection index
    uint8_t       conidx;
    /// channel map value
    le_chnl_map_t ch_map;
};

/// Connection Parameter used to update connection parameters
struct gapc_conn_param
{
    /// Connection interval minimum
    uint16_t intv_min;
    /// Connection interval maximum
    uint16_t intv_max;
    /// Latency
    uint16_t latency;
    /// Supervision timeout
    uint16_t time_out;
};

/// Perform update of connection parameters command
/*@TRACE*/
struct gapc_param_update_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// GAP request type:
    /// - #GAPC_UPDATE_PARAMS: Perform update of connection parameters.
    uint8_t  operation;
    /// Connection interval minimum
    uint16_t intv_min;
    /// Connection interval maximum
    uint16_t intv_max;
    /// Latency
    uint16_t latency;
    /// Supervision timeout
    uint16_t time_out;
    /// Minimum Connection Event Duration
    uint16_t ce_len_min;
    /// Maximum Connection Event Duration
    uint16_t ce_len_max;
};

/// Request of updating connection parameters indication
/*@TRACE*/
struct gapc_param_update_req_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Connection interval minimum
    uint16_t intv_min;
    /// Connection interval maximum
    uint16_t intv_max;
    /// Latency
    uint16_t latency;
    /// Supervision timeout
    uint16_t time_out;
};

/// Connection parameters updated indication
/*@TRACE*/
struct gapc_param_updated_ind
{
    /// Connection index
    uint8_t  conidx;
    ///Connection interval value
    uint16_t con_interval;
    ///Connection latency value
    uint16_t con_latency;
    ///Supervision timeout
    uint16_t sup_to;
};

/// Master confirm or not that parameters proposed by slave are accepted or not
/*@TRACE*/
struct gapc_param_update_cfm
{
    /// Connection index
    uint8_t  conidx;
    /// True to accept peer connection parameters, False otherwise
    bool     accept;
    /// Minimum Connection Event Duration
    uint16_t ce_len_min;
    /// Maximum Connection Event Duration
    uint16_t ce_len_max;
};

/// Parameters of the #GAPC_SET_PREF_SLAVE_LATENCY_CMD message
/*@TRACE*/
struct gapc_set_pref_slave_latency_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// GAP request type:
    /// - #GAPC_SET_PREF_SLAVE_LATENCY_CMD : Set preferred slave latency
    uint8_t  operation;
    /// Preferred latency that the controller should use on a connection (in number of connection events)
    uint16_t latency;
};

/// Parameters of the #GAPC_SET_PREF_SLAVE_EVT_DUR_CMD message
/*@TRACE*/
struct gapc_set_pref_slave_evt_dur_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// GAP request type:
    /// - #GAPC_SET_PREF_SLAVE_EVT_DUR_CMD : Set preferred slave event duration
    uint8_t  operation;
    /// Preferred event duration that the controller should use on a connection (N * 0.625 ms)
    uint16_t duration;
    /// Slave transmits a single packet per connection event (False/True)
    bool     single_tx;
};

/// Parameters of the #GAPC_SET_MAX_RX_SIZE_AND_TIME_CMD message
/*@TRACE*/
struct gapc_set_max_rx_size_and_time_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// GAP request type:
    /// - #GAPC_SET_MAX_RX_SIZE_AND_TIME: Set maximum RX size and time using DLE negotiation
    uint8_t  operation;
    /// Maximum RX size (in Bytes)
    uint16_t rx_octets;
    /// Maximum RX time (in us)
    uint16_t rx_time;
};


/// Parameters of the #GAPC_PER_ADV_SYNC_TRANS_CMD message
/*@TRACE*/
struct gapc_per_adv_sync_trans_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// GAP request type:
    /// - #GAPC_PER_ADV_SYNC_TRANS : Periodic Advertising Sync Transfer
    uint8_t  operation;
    /// Periodic Advertising or Periodic Sync activity index
    uint8_t  actv_idx;
    /// A value provided by application
    uint16_t service_data;
};

/// Parameters of the #GAPC_CTE_TX_CFG_CMD message
/*@TRACE*/
struct gapc_cte_tx_cfg_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// GAP request type:
    /// - #GAPC_CTE_TX_CFG: Constant Tone Extension Transmission configuration
    uint8_t  operation;
    /// CTE types (bit0: AOA | bit1: AOD-1us | bit2: AOD-2us) (see enum #gap_cte_type_bf)
    uint8_t  cte_types;
    /// Length of switching pattern (number of antenna IDs in the pattern)
    uint8_t  switching_pattern_len;
    /// Antenna IDs
    uint8_t  antenna_id[__ARRAY_EMPTY];
};

/// Parameters of the #GAPC_CTE_RX_CFG_CMD message
/*@TRACE*/
struct gapc_cte_rx_cfg_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// GAP request type:
    /// - #GAPC_CTE_RX_CFG: Constant Tone Extension Reception configuration
    uint8_t  operation;
    /// Sampling enable
    bool     sample_en;
    /// Slot durations (1: 1us | 2: 2us)
    uint8_t  slot_dur;
    /// Length of switching pattern (number of antenna IDs in the pattern)
    uint8_t  switching_pattern_len;
    /// Antenna IDs
    uint8_t  antenna_id[__ARRAY_EMPTY];
};

/// Parameters of the #GAPC_CTE_REQ_CTRL_CMD message
/*@TRACE*/
struct gapc_cte_req_ctrl_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// GAP request type:
    /// - #GAPC_CTE_REQ_CTRL: Constant Tone Extension request control (enable / disable)
    uint8_t  operation;
    /// True to enable TX or RX Constant Tone Extension, False to disable
    bool     enable;
    /// CTE request interval (in number of connection events)
    uint16_t interval;
    /// Requested CTE length (in 8us unit)
    uint8_t  cte_len;
    /// Requested CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us)  (see enum #gap_cte_type)
    uint8_t  cte_type;
};

/// Parameters of the #GAPC_CTE_RSP_CTRL_CMD message
/*@TRACE*/
struct gapc_cte_rsp_ctrl_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// GAP request type:
    /// - #GAPC_CTE_RSP_CTRL: Constant Tone Extension response control (enable / disable)
    uint8_t  operation;
    /// True to enable TX or RX Constant Tone Extension, False to disable
    bool     enable;
};

/// Indicate reception of a IQ Report event over a BLE connection
/*@TRACE*/
struct gapc_cte_iq_report_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Rx PHY  (see enum #gap_phy_val)
    uint8_t  rx_phy;
    /// Data channel index
    uint8_t  data_channel_idx;
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
    /// Connection event counter
    uint16_t con_evt_cnt;
    /// Number of samples
    uint8_t  nb_samples;
    /// I/Q sample
    gap_iq_sample_t sample[__ARRAY_EMPTY];
};

/// Parameters of #GAPC_CTE_REQ_FAILED_IND message
struct gapc_cte_req_failed_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Reason status code of the CTE request failed (see enum #hl_err)
    uint16_t status;
};


/// Parameters of the #GAPC_SET_LE_PKT_SIZE_CMD message
/*@TRACE*/
struct gapc_set_le_pkt_size_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// GAP request type:
    /// - #GAPC_SET_LE_PKT_SIZE : Set the LE Data length value
    uint8_t  operation;
    ///Preferred maximum number of payload octets that the local Controller should include
    ///in a single Link Layer Data Channel PDU.
    uint16_t tx_octets;
    ///Preferred maximum number of microseconds that the local Controller should use to transmit
    ///a single Link Layer Data Channel PDU
    uint16_t tx_time;
};

/// Parameters of the #GAPC_LE_PKT_SIZE_IND message
/*@TRACE*/
struct gapc_le_pkt_size_ind
{
    /// Connection index
    uint8_t  conidx;
    ///The maximum number of payload octets in TX
    uint16_t max_tx_octets;
    ///The maximum time that the local Controller will take to TX
    uint16_t max_tx_time;
    ///The maximum number of payload octets in RX
    uint16_t max_rx_octets;
    ///The maximum time that the local Controller will take to RX
    uint16_t max_rx_time;
};


/// Set the PHY configuration for current active link
/*@TRACE*/
struct gapc_set_phy_cmd
{
    /// Connection index
    uint8_t conidx;
    /// GAP request type:
    /// - #GAPC_SET_PHY : Set the PHY configuration for current active link
    uint8_t operation;
    /// Supported LE PHY for data transmission (see enum #gap_phy)
    uint8_t tx_phy;
    /// Supported LE PHY for data reception (see enum #gap_phy)
    uint8_t rx_phy;
    /// PHY options (see enum #gapc_phy_option)
    uint8_t phy_opt;
};

/// Active link PHY configuration. Triggered when configuration is read or during an update.
/*@TRACE*/
struct gapc_le_phy_ind
{
    /// Connection index
    uint8_t conidx;
    /// LE PHY for data transmission (see enum #gap_phy)
    uint8_t tx_phy;
    /// LE PHY for data reception (see enum #gap_phy)
    uint8_t rx_phy;
};

/// Parameters of the #GAPC_CHAN_SEL_ALGO_IND
/*@TRACE*/
struct gapc_chan_sel_algo_ind
{
    /// Connection index
    uint8_t conidx;
    /// Used channel selection algorithm
    uint8_t chan_sel_algo;
};

/// Local TX power indication
/*@TRACE*/
typedef struct gapc_loc_tx_pwr_ind
{
    /// Connection index
    uint8_t conidx;
    /// PHY (see enum #gapc_phy_pwr_value)
    uint8_t phy;
    /// Current transmit power level (dBm)
    int8_t  tx_pwr;
    /// Max transmit power level (dBm)
    int8_t  max_tx_pwr;
} gapc_loc_tx_pwr_ind_t;

/// Remote TX power indication
/*@TRACE*/
typedef struct gapc_peer_tx_pwr_ind
{
    /// Connection index
    uint8_t conidx;
    /// PHY (see enum #gapc_phy_pwr_value)
    uint8_t phy;
    /// Transmit Power level (dBm)
    int8_t  tx_pwr;
    /// Transmit Power level flags (see enum #gapc_pwr_ctrl_flags)
    uint8_t flags;
} gapc_peer_tx_pwr_ind_t;

/// Control TX Power Reports command
/*@TRACE*/
typedef struct gapc_tx_pwr_report_ctrl_cmd
{
    /// Connection index
    uint8_t conidx;
    /// GAP request type:
    /// - #GAPC_TX_PWR_REPORT_CTRL: Enable or disable the reporting to the local Host of transmit power level
    ///                             changes in the local and remote Controllers for the ACL connection
    uint8_t operation;
    /// 1 To Enable local power changes reporting, 0 to disable.
    uint8_t local_en;
    /// 1 To Enable remote power changes reporting, 0 to disable.
    uint8_t remote_en;

} gapc_tx_pwr_report_ctrl_cmd_t;

/// Local TX power change report indication
/*@TRACE*/
typedef struct gapc_loc_tx_pwr_report_ind
{
    /// Connection index
    uint8_t conidx;
    /// PHY (see enum #gapc_phy_pwr_value)
    uint8_t phy;
    /// Transmit Power level (dBm)
    int8_t  tx_pwr;
    /// Transmit Power level flags (see enum #gapc_pwr_ctrl_flags)
    uint8_t flags;
    /// Delta (dB)
    int8_t  delta;
} gapc_loc_tx_pwr_report_ind_t;

/// Remote TX power change report indication
/*@TRACE*/
typedef struct gapc_peer_tx_pwr_report_ind
{
    /// Connection index
    uint8_t conidx;
    /// PHY (see enum #gapc_phy_pwr_value)
    uint8_t phy;
    /// Transmit Power level (dBm)
    int8_t  tx_pwr;
    /// Transmit Power level flags (see enum #gapc_pwr_ctrl_flags)
    uint8_t flags;
    /// Delta (dB)
    int8_t  delta;
} gapc_peer_tx_pwr_report_ind_t;


/// Control Path loss configuration
/*@TRACE*/
typedef struct gapc_path_loss_ctrl_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// GAP request type:
    /// - #GAPC_PATH_LOSS_REPORT_CTRL: Command is used to enable/disable path loss reporting for the connection
    uint8_t  operation;
    /// 1 To Enable reporting, 0 to disable.
    uint8_t  enable;
    /// High threshold (dB)
    uint8_t  high_threshold;
    /// High hysteresis (dB)
    uint8_t  high_hysteresis;
    /// Low threshold (dB)
    uint8_t  low_threshold;
    /// Low hysteresis (dB)
    uint8_t  low_hysteresis;
    /// Min time spent (conn events)
    uint16_t min_time;

} gapc_path_loss_ctrl_cmd_t;

/// Path Loss Threshold Indication
/*@TRACE*/
typedef struct gapc_path_loss_threshold_ind
{
    /// Connection index
    uint8_t conidx;
    /// Current path loss (dB)
    uint8_t curr_path_loss;
    /// Zone entered (see enum #gapc_path_loss_zone)
    uint8_t zone_entered;
} gapc_path_loss_threshold_ind_t;


/// Start Encryption command procedure
/*@TRACE*/
struct gapc_le_encrypt_cmd
{
    /// Connection index
    uint8_t conidx;
    /// GAP request type:
    /// - #GAPC_LE_ENCRYPT:  Start encryption procedure.
    uint8_t operation;
    /// Long Term Key information
    gapc_ltk_t ltk;
};

/// Encryption requested by peer device indication message.
/*@TRACE*/
struct gapc_le_encrypt_req_ind
{
    /// Connection index
    uint8_t conidx;
    /// Encryption Diversifier
    uint16_t ediv;
    /// Random Number
    rand_nb_t rand_nb;
};

/// Confirm requested Encryption information.
/*@TRACE*/
struct gapc_le_encrypt_cfm
{
    /// Connection index
    uint8_t conidx;
    /// Indicate if a LTK has been found for the peer device
    uint8_t found;
    /// Long Term Key
    gap_sec_key_t ltk;
    /// LTK Key Size
    uint8_t key_size;
};

/// Start Security Request command procedure
/*@TRACE*/
struct gapc_security_cmd
{
    /// Connection index
    uint8_t conidx;
    /// GAP request type:
    /// - #GAPC_SECURITY_REQ: Start security request procedure
    uint8_t operation;
    /// Authentication level (#gap_auth)
    uint8_t auth;
};
/// Security requested by peer device indication message
/*@TRACE*/
struct gapc_security_ind
{
    /// Connection index
    uint8_t conidx;
    /// Authentication level (#gap_auth)
    uint8_t auth;
};


/// Parameters of the #GAPC_KEY_PRESS_NOTIFICATION_CMD message
/*@TRACE*/
struct gapc_key_press_notif_cmd
{
    /// Connection index
    uint8_t conidx;
    /// GAP request type:
    /// - #GAPC_KEY_PRESS_NOTIFICATION_CMD : Inform the remote device when keys have been entered or erased
    uint8_t operation;
    /// notification type
    uint8_t notification_type;
};

/// Parameters of the #GAPC_KEY_PRESS_NOTIFICATION_IND message
/*@TRACE*/
struct gapc_key_press_notif_ind
{
    /// Connection index
    uint8_t conidx;
    /// notification type
    uint8_t notification_type;
};

/// Start Bonding command procedure
/*@TRACE*/
struct gapc_bond_cmd
{
    /// Connection index
    uint8_t        conidx;
    /// GAP request type:
    /// - #GAPC_BOND:  Start bonding procedure.
    uint8_t        operation;
    /// Pairing information
    gapc_pairing_t pairing;
    /// Device security requirements (minimum security level). (see enum #gap_sec_req)
    uint8_t        sec_req_level;
};
/// @} GAPC_MSG_STRUCT_API

#endif /* _GAPC_LE_MSG_H_ */
