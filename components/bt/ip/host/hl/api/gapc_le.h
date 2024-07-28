/**
 ****************************************************************************************
 *
 * @file gapc_le.h
 *
 * @brief Generic Access Profile Controller - Low Energy API.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */


#ifndef _GAPC_LE_H_
#define _GAPC_LE_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gapc.h"


/**
 ****************************************************************************************
 * @addtogroup GAPC_LE_POWER_API LE Power Control
 * @ingroup GAPC_API
 *
 * @brief Control and be informed about radio power for on-going LE connection.
 *
 * @{
 * @}
 ****************************************************************************************
 */



/**
 ****************************************************************************************
 * @addtogroup GAPC_CTE_API Constant Tone Extension
 * @ingroup GAPC_API
 *
 * @brief Manage Constant Tone Extension for on-going LE connection
 *
 * @{
 * @}
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


/*
 * CALLBACK DEFINITIONS
 ****************************************************************************************
 */

/// @addtogroup GAPC_CON_UP_API
/// @{

/// Callback structure used to be notified about connection configuration events
typedef struct gapc_le_config_cb
{
    /**
     ****************************************************************************************
     * Callback executed when connection parameter update is requested
     * #gapc_le_con_param_update_cfm shall be called to confirm new parameters.
     *
     * @note Optional callback - Parameters automatically accepted if callback not provided on peripheral side.
     *                           Automatically rejected on central side.
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] p_param       Pointer to negotiated LE connection parameters
     ****************************************************************************************
     */
    void (*param_update_req)(uint8_t conidx, uint32_t dummy, const gap_le_con_param_nego_t* p_param);

    /**
     ****************************************************************************************
     * Callback executed when connection parameter are updated
     *
     * @note Optional callback.
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] p_param       Pointer to new connection parameters
     ****************************************************************************************
     */
    void (*param_updated)(uint8_t conidx, uint32_t dummy, const gap_le_con_param_t* p_param);

    /**
     ****************************************************************************************
     * Callback executed when data length over the air has been updated
     *
     * @note Optional callback.
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] max_tx_octets The maximum number of payload octets in TX
     * @param[in] max_tx_time   The maximum time that the local Controller will take to TX (in microseconds)
     * @param[in] max_tx_octets The maximum number of payload octets in RX
     * @param[in] max_tx_time   The maximum time that the local Controller will take to RX (in microseconds)
     ****************************************************************************************
     */
    void (*packet_size_updated)(uint8_t conidx, uint32_t dummy, uint16_t max_tx_octets , uint16_t max_tx_time,
                                uint16_t max_rx_octets , uint16_t max_rx_time);

    /**
     ****************************************************************************************
     * Callback executed when LE PHY is updated
     *
     * @note Optional callback.
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] tx_phy        LE PHY for data transmission (see enum #gap_phy)
     * @param[in] rx_phy        LE PHY for data reception (see enum #gap_phy)
     ****************************************************************************************
     */
    void (*phy_updated)(uint8_t conidx, uint32_t dummy, uint8_t tx_phy , uint8_t rx_phy);
} gapc_le_config_cb_t;

/// Callback structure used to be notified about LE Events (from a profile or an application module)
typedef struct gapc_le_event_cb
{
    /**
     ****************************************************************************************
     * Callback executed to provide initial LE connection parameter or updated one
     *
     * @note Optional callback.
     *
     * @param[in] conidx        Connection index
     * @param[in] p_param       Pointer to connection parameters
     ****************************************************************************************
     */
    void (*con_param)(uint8_t conidx, const gap_le_con_param_t* p_param);
} gapc_le_event_cb_t;


/// Structure that must be followed by a client of LE events
typedef struct gapc_le_event_client_t
{
    /// List header element
    co_list_hdr_t             hdr;
    /// Pointer to the callback structure, SHALL NOT BE NULL
    const gapc_le_event_cb_t* p_cbs;
} gapc_le_event_client_t;

/// @} GAPC_CON_UP_API


/// @addtogroup GAPC_LE_POWER_API
/// @{

/// Callback structure used to be notified about LE Power events
typedef struct gapc_le_power_cb
{
    /**
     ****************************************************************************************
     * Callback executed when a TX power change report is received
     *
     * @note  Mandatory callback
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] local         True if local TX power change report, remote TX power change report otherwise
     * @param[in] p_report      Pointer to TX Power report information
     ****************************************************************************************
     */
    void (*tx_change_report)(uint8_t conidx, uint32_t dummy, bool local, const gapc_tx_power_report_t* p_report);

    /**
     ****************************************************************************************
     * Callback executed when a Path Loss threshold report event is received
     *
     * @note Mandatory callback
     *
     * @param[in] conidx         Connection index
     * @param[in] dummy          Dummy parameter provided by upper layer application
     * @param[in] curr_path_loss Current path loss (dB)
     * @param[in] zone_entered   Zone entered (see enum #gapc_path_loss_zone)
     ****************************************************************************************
     */
    void (*path_loss_threshold_report)(uint8_t conidx, uint32_t dummy, uint8_t curr_path_loss, uint8_t zone_entered);
} gapc_le_power_cb_t;
/// @} GAPC_LE_POWER_API


/// @addtogroup GAPC_CTE_API
/// @{

/// Callback structure used to be notified about constant tone extension events
typedef struct gapc_le_cte_cb
{
    /**
     ****************************************************************************************
     * Callback executed when an IQ report has been received
     *
     * @note Mandatory callback
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] p_report      Pointer to IQ report information
     * @param[in] nb_samples    Number of samples
     * @param[in] p_samples     Pointer to samples data
     ****************************************************************************************
     */
    void (*iq_report_received)(uint8_t conidx, uint32_t dummy, const gapc_iq_report_info_t* p_report, uint8_t nb_samples,
                               const gap_iq_sample_t* p_samples);


    /**
     ****************************************************************************************
     * Callback executed when a CTE request failed event is triggered by controller
     *
     * @note Mandatory callback
     *
     * @param[in] conidx        Connection index
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] reason        Failed reason (see enum #hl_err)
     ****************************************************************************************
     */
    void (*request_failed_event)(uint8_t conidx, uint32_t dummy, uint16_t reason);
} gapc_le_cte_cb_t;
/// @} GAPC_CTE_API
///
/*
 * MACROS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/// @addtogroup GAPC_CON_REQ_API
/// @{

/**
 ****************************************************************************************
 * @brief Upper layer SW confirmation of Low energy link creation with bond data if available.
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] p_data    Pointer to bond data if present, NULL otherwise
 *
 * @return Return function execution status (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapc_le_connection_cfm(uint8_t conidx, uint32_t dummy, const gapc_bond_data_t* p_data);

/// @} GAPC_CON_REQ_API


/// @addtogroup GAPC_CON_INFO_API
/// @{

/**
 ****************************************************************************************
 * @brief Get if connection is an LE connection.
 *
 * @param[in] conidx Connection index
 *
 * @return Return true if connection is an LE connection; false otherwise.
 ****************************************************************************************
 */
bool gapc_is_le_connection(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Get if peer device supports a specific Low Energy feature.
 *
 * @param[in] conidx    Connection index
 * @param[in] feature   Feature bit (see enum #gap_le_feature)
 *
 * @return Return true if remote features has been read and if supported, false otherwise
 ****************************************************************************************
 */
bool gapc_is_le_feat_supported(uint8_t conidx, uint8_t feature);

/**
 ****************************************************************************************
 * @brief Get LE channel selection algorithm used for a given connection identified
 *        by its connection index.
 *
 * @param[in] conidx    Connection index
 *
 * @return Channel selection algorithm used (0 if algo #1, 1 if algo #2, 0xFF if invalid connection)
 ****************************************************************************************
 */
uint8_t gapc_get_le_channel_selection_algo(uint8_t conidx);

/**
 ****************************************************************************************
 * Callback executed when get LE channel map for connection procedure is completed.
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] status    Status of procedure execution (see enum #hl_err)
 * @param[in] p_ch_map  Pointer to the LE channel map value
 *
 ****************************************************************************************
 */
typedef void (*gapc_get_le_chmap_cmp_cb)(uint8_t conidx, uint32_t dummy, uint16_t status,
                                         const le_chnl_map_t* p_ch_map);

/**
 ****************************************************************************************
 * @brief Get connection LE channel map used.
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_get_auth_payload_to_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_get_le_chmap(uint8_t conidx, uint32_t dummy, gapc_get_le_chmap_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * Callback executed when get LE peer supported features value procedure is completed.
 *
 * @param[in] conidx      Connection index
 * @param[in] dummy       Dummy parameter provided by upper layer application
 * @param[in] status      Status of procedure execution (see enum #hl_err)
 * @param[in] p_features  Pointer to peer supported feature array
 *
 ****************************************************************************************
 */
typedef void (*gapc_get_le_peer_features_cmp_cb)(uint8_t conidx, uint32_t dummy, uint16_t status,
                                                 const uint8_t* p_features);

/**
 ****************************************************************************************
 * @brief Get connection LE peer supported features
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_get_auth_payload_to_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_get_le_peer_features(uint8_t conidx, uint32_t dummy, gapc_get_le_peer_features_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * Callback executed when get LE connection used PHY value procedure is completed.
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] status    Status of procedure execution (see enum #hl_err)
 * @param[in] tx_phy    LE PHY for data transmission (see enum #gap_phy)
 * @param[in] rx_phy    LE PHY for data reception (see enum #gap_phy)
 *
 ****************************************************************************************
 */
typedef void (*gapc_get_le_phy_cmp_cb)(uint8_t conidx, uint32_t dummy, uint16_t status,
                                       uint8_t tx_phy, uint8_t rx_phy);

/**
 ****************************************************************************************
 * @brief Get LE connection used PHY value
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_get_auth_payload_to_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_get_le_phy(uint8_t conidx, uint32_t dummy, gapc_get_le_phy_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * Callback executed when get LE connection local transmit power level information procedure is completed.
 *
 * @param[in] conidx          Connection index
 * @param[in] dummy           Dummy parameter provided by upper layer application
 * @param[in] status          Status of procedure execution (see enum #hl_err)
 * @param[in] phy             Transmission PHY (see enum #gapc_phy_pwr_value)
 * @param[in] power_level     Current transmit power level (in dBm)
 * @param[in] max_power_level Max transmit power level (in dBm)
 *
 ****************************************************************************************
 */
typedef void (*gapc_le_get_local_tx_power_level_cmp_cb)(uint8_t conidx, uint32_t dummy, uint16_t status,
                                                         uint8_t phy, int8_t power_level, int8_t max_power_level);

/**
 ****************************************************************************************
 * @brief Get LE connection local transmit power level information for a specific PHY.
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] phy       Specific transmission PHY (see enum #gapc_phy_pwr_value)
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_get_auth_payload_to_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_get_local_tx_power_level(uint8_t conidx, uint32_t dummy, uint8_t phy,
                                          gapc_le_get_local_tx_power_level_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * Callback executed when LE remote transmit power level read procedure is completed.
 *
 * @param[in] conidx          Connection index
 * @param[in] dummy           Dummy parameter provided by upper layer application
 * @param[in] status          Status of procedure execution (see enum #hl_err)
 * @param[in] phy             Transmission PHY (see enum #gapc_phy_pwr_value)
 * @param[in] power_level     Current transmit power level (in dBm)
 * @param[in] flags           Transmit Power level flags (see enum #gapc_pwr_ctrl_flags)
 *
 ****************************************************************************************
 */
typedef void (*gapc_le_get_peer_tx_power_level_cmp_cb)(uint8_t conidx, uint32_t dummy, uint16_t status,
                                                         uint8_t phy, int8_t power_level, uint8_t flags);

 /**
 ****************************************************************************************
 * @brief Get LE connection remote transmit power level information for a specific PHY.
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] phy       Specific transmission PHY (see enum #gapc_phy_pwr_value)
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_get_auth_payload_to_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_get_peer_tx_power_level(uint8_t conidx, uint32_t dummy, uint8_t phy,
                                         gapc_le_get_peer_tx_power_level_cmp_cb cmp_cb);

/// @} GAPC_CON_INFO_API

/// @addtogroup GAPC_CON_UP_API Connection Update
/// @{
/**
 ****************************************************************************************
 * @brief Function used to negotiate new BLE connection parameters
 *
 * @param[in] conidx        Connection index
 * @param[in] dummy         Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] p_param       Pointer to new LE connection parameters to negotiate.
 * @param[in] cmp_cb        Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_con_param_update(uint8_t conidx, uint32_t dummy, const gap_le_con_param_nego_with_ce_len_t* p_param,
                                  gapc_proc_cmp_cb cmp_cb);
/**
 ****************************************************************************************
 * @brief Function used to accept or reject LE connection parameter update
 *
 * @param[in] conidx      Connection index
 * @param[in] accept      True to accept peer connection parameters, False otherwise
 * @param[in] ce_len_min  Minimum connection Event Duration (0.625 ms unit)
 * @param[in] ce_len_max  Maximum connection Event Duration (0.625 ms unit)
 *
 * @return Return function execution status (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapc_le_con_param_update_cfm(uint8_t conidx, bool accept, uint16_t ce_len_min, uint16_t ce_len_max);

/**
 ****************************************************************************************
 * @brief Function used to negotiate new BLE phy for connection
 *
 * @param[in] conidx        Connection index
 * @param[in] dummy         Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] tx_phy        Supported LE PHY for data transmission (see enum #gap_phy)
 * @param[in] rx_phy        Supported LE PHY for data reception (see enum #gap_phy)
 * @param[in] phy_opt       PHY options (see enum #gapc_phy_option)
 * @param[in] cmp_cb        Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_set_phy(uint8_t conidx, uint32_t dummy, uint8_t tx_phy, uint8_t rx_phy, uint8_t phy_opt,
                         gapc_proc_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * @brief Function used to set the preferred BLE connection peripheral latency (dynamically without negotiation)
 * @note Can be initiated only by a peripheral
 *
 * @param[in] conidx        Connection index
 * @param[in] dummy         Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] latency       Preferred latency (in number of connection events)
 * @param[in] cmp_cb        Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_set_pref_slave_latency(uint8_t conidx, uint32_t dummy, uint16_t latency, gapc_proc_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * @brief Function used to set the preferred slave event duration (dynamically without negotiation)
 * @note Can be initiated only by a peripheral
 *
 * @param[in] conidx        Connection index
 * @param[in] dummy         Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] duration      Preferred event duration (N * 0.625 ms)
 * @param[in] single_tx      Slave transmits a single packet per connection event (False/True)
 * @param[in] cmp_cb        Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_set_pref_slave_evt_dur(uint8_t conidx, uint32_t dummy, uint16_t duration, bool single_tx,
                                        gapc_proc_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * @brief Function used to set the maximum reception size and time
 *
 * @param[in] conidx        Connection index
 * @param[in] dummy         Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] rx_octets     Maximum RX size (in Bytes)
 * @param[in] rx_time       Maximum RX time (in microseconds)
 * @param[in] cmp_cb        Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_set_max_rx_size_and_time(uint8_t conidx, uint32_t dummy, uint16_t rx_octets, uint16_t rx_time,
                                       gapc_proc_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * @brief Function used to set the maximum transmit size and time
 *
 * @param[in] conidx        Connection index
 * @param[in] dummy         Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] tx_octets     Preferred maximum number of payload octets that the local Controller should include
 *                          in a single Link Layer Data Channel PDU.
 * @param[in] tx_time       Preferred maximum number of microseconds that the local Controller should use to transmit
 *                          a single Link Layer Data Channel PDU
 * @param[in] cmp_cb        Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_set_pkt_size(uint8_t conidx, uint32_t dummy, uint16_t tx_octets, uint16_t tx_time,
                              gapc_proc_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * @brief Register a client waiting for LE events.
 *
 * Corresponding callback will be executed when an LE event is triggered
 *
 * @param[in] p_client      Pointer to the LE event client structure
 *
 * @return Return function execution status (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapc_le_event_client_register(gapc_le_event_client_t* p_client);

/**
 ****************************************************************************************
 * @brief Un-Register a client waiting for LE events.
 *
 * @param[in] p_client      Pointer to the LE event client structure
 *
 * @return Return function execution status (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapc_le_event_client_unregister(gapc_le_event_client_t* p_client);

/// @} GAPC_CON_UP_API

/// @addtogroup GAPC_PAST_API Periodic ADV Sync Transfer
/// @ingroup GAPC_API
/// @{
/**
 ****************************************************************************************
 * @brief Transfer periodic advertising sync information to peer device.
 *        Either a periodic advertising or a periodic sync activity.
 *
 * @param[in] conidx        Connection index
 * @param[in] dummy         Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] actv_idx      Periodic Advertising or Periodic Sync activity index
 * @param[in] service_data  A value provided by application given to peer device
 * @param[in] cmp_cb        Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_per_adv_sync_transfer(uint8_t conidx, uint32_t dummy, uint8_t actv_idx, uint16_t service_data,
                                       gapc_proc_cmp_cb cmp_cb);
/// @} GAPC_PAST_API

/// @addtogroup GAPC_LE_POWER_API LE Power Control
/// @{

/**
 ****************************************************************************************
 * @brief Set Callback that will handle path loss and tx power change reports
 *
 * @param[in] p_cbs         Pointer to the callback set
 *
 * @return Return function execution status (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapc_le_power_set_callbacks(const gapc_le_power_cb_t* p_cbs);

/**
 ****************************************************************************************
 * @brief Control reception of TX Local and/or remote power report
 *        #gapc_le_power_cb_t.tx_change_report must be valid
 *
 * @param[in] conidx        Connection index
 * @param[in] dummy         Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] local_en      True to enable remote power changes reporting, false to disable.
 * @param[in] remote_en     True to enable remote power changes reporting, false to disable.
 * @param[in] cmp_cb        Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_tx_pwr_report_ctrl(uint8_t conidx, uint32_t dummy, bool local_en, bool remote_en, gapc_proc_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * @brief Enable path loss detection
 *        #gapc_le_power_cb_t.path_loss_threshold_report must be valid
 *
 * @param[in] conidx          Connection index
 * @param[in] dummy           Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] high_threshold  High threshold (dB)
 * @param[in] high_hysteresis High hysteresis (dB)
 * @param[in] low_threshold   Low threshold (dB)
 * @param[in] low_hysteresis  Low hysteresis (dB)
 * @param[in] min_time        Min time spent (Number of connection events)
 * @param[in] cmp_cb          Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_path_loss_enable(uint8_t conidx, uint32_t dummy, uint8_t high_threshold, uint8_t high_hysteresis,
                                  uint8_t low_threshold, uint8_t low_hysteresis, uint16_t min_time,
                                  gapc_proc_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * @brief Disable path loss detection
 *
 * @param[in] conidx          Connection index
 * @param[in] dummy           Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] cmp_cb          Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_path_loss_disable(uint8_t conidx, uint32_t dummy, gapc_proc_cmp_cb cmp_cb);

/// @} GAPC_LE_POWER_API

/// @addtogroup GAPC_CTE_API Constant Tone Extension
/// @{
/**
 ****************************************************************************************
 * @brief Set Callbacks used to handle IQ reports
 *
 * @param[in] p_cbs         Pointer to the callback set
 *
 * @return Return function execution status (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapc_le_cte_set_callbacks(const gapc_le_cte_cb_t* p_cbs);

/**
 ****************************************************************************************
 * @brief Configure constant tone extension transmission parameters.
 *
 * @param[in] conidx                Connection index
 * @param[in] dummy                 Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] cte_types             CTE types (bit0: AOA | bit1: AOD-1us | bit2: AOD-2us) (see enum #gap_cte_type_bf)
 * @param[in] switching_pattern_len Length of switching pattern (number of antenna IDs in the pattern)
 * @param[in] p_antenna_id          Pointer to antenna IDs
 * @param[in] cmp_cb                Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_cte_tx_configure(uint8_t conidx, uint32_t dummy, uint8_t cte_types, uint8_t switching_pattern_len,
                                  const uint8_t* p_antenna_id, gapc_proc_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * @brief Configure constant tone extension reception parameters.
 *
 * @param[in] conidx                Connection index
 * @param[in] dummy                 Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] sample_enable         True to enable sampling, false otherwise
 * @param[in] slot_dur              Slot durations (1: 1us | 2: 2us)
 * @param[in] switching_pattern_len Length of switching pattern (number of antenna IDs in the pattern)
 * @param[in] p_antenna_id          Pointer to antenna IDs
 * @param[in] cmp_cb                Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_cte_rx_configure(uint8_t conidx, uint32_t dummy, bool sample_enable, uint8_t slot_dur,
                                  uint8_t switching_pattern_len, const uint8_t* p_antenna_id, gapc_proc_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * @brief Control transmission of constant tone extension requests initiated by controller.
 *
 * @param[in] conidx      Connection index
 * @param[in] dummy       Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] enable      True to enable CTE requests, false otherwise
 * @param[in] interval    CTE request interval (in number of connection events)
 * @param[in] cte_length  Requested CTE length (in 8us unit)
 * @param[in] cte_type    Requested CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us)
 * @param[in] cmp_cb      Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_cte_request_control(uint8_t conidx, uint32_t dummy, bool enable, uint16_t interval,
                                     uint8_t cte_length, uint8_t cte_type, gapc_proc_cmp_cb cmp_cb);

/**
 ****************************************************************************************
 * @brief Control if controller answers constant tone extension requests.
 *
 * @param[in] conidx      Connection index
 * @param[in] dummy       Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] enable      True to enable CTE responses, false otherwise
 * @param[in] cmp_cb      Function called when procedure is over.
 *
 * @return Return function execution status (see enum #hl_err)
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapc_proc_cmp_cb callback execution
 ****************************************************************************************
 */
uint16_t gapc_le_cte_response_control(uint8_t conidx, uint32_t dummy, bool enable, gapc_proc_cmp_cb cmp_cb);

/// @} GAPC_CTE_API

#endif /* _GAPC_LE_H_ */
