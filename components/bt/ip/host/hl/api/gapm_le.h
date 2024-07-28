/**
 ****************************************************************************************
 *
 * @file gapm_le.h
 *
 * @brief Generic Access Profile Manager - Low Energy Activities
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


#ifndef GAPM_LE_H_
#define GAPM_LE_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gapm.h"
#include "co_buf.h"


/**
 ****************************************************************************************
 * @addtogroup GAPM_LE_API Low Energy
 *
 * @ingroup GAPM_API
 *
 * Set of functions and interfaces required to create and manage Low Energy activities.
 *
 * @{
 * @}
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_LE_LIST_API List management
 * @ingroup GAPM_LE_API
 * @brief White list, Resolving Address list, Periodic Advertising List
 *
 * @{
 * @}
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_LE_SEC_API Security toolbox
 * @ingroup GAPM_LE_API
 * @brief OOB data, Random number generation and use ECDH P-256 toolbox
 *
 * @{
 * @}
 ****************************************************************************************
 */

/// @addtogroup GAPM_ENUM_API
/// @{

/*
 * DEFINES
 ****************************************************************************************
 */
/// Own BD address source of the device
enum gapm_own_addr
{
   /// Public or Private Static Address according to device address configuration
   GAPM_STATIC_ADDR,
   /// Generated resolvable private random address
   GAPM_GEN_RSLV_ADDR,
   /// Generated non-resolvable private random address
   GAPM_GEN_NON_RSLV_ADDR,
};

/// PHY Type
enum gapm_phy_type
{
    /// LE 1M
    GAPM_PHY_TYPE_LE_1M = 1,
    /// LE 2M
    GAPM_PHY_TYPE_LE_2M,
    /// LE Coded
    GAPM_PHY_TYPE_LE_CODED,
};

/// Advertising report type
enum gapm_adv_report_type
{
    /// Extended advertising report
    GAPM_REPORT_TYPE_ADV_EXT = 0,
    /// Legacy advertising report
    GAPM_REPORT_TYPE_ADV_LEG,
    /// Extended scan response report
    GAPM_REPORT_TYPE_SCAN_RSP_EXT,
    /// Legacy scan response report
    GAPM_REPORT_TYPE_SCAN_RSP_LEG,
    /// Periodic advertising report
    GAPM_REPORT_TYPE_PER_ADV,
};

/// Advertising report information
enum gapm_adv_report_info_bf
{
    /// Report Type
    GAPM_REPORT_INFO_REPORT_TYPE_MASK    = 0x07,
    /// Report is complete
    GAPM_REPORT_INFO_COMPLETE_BIT        = (1 << 3),
    /// Connectable advertising
    GAPM_REPORT_INFO_CONN_ADV_BIT        = (1 << 4),
    /// Scannable advertising
    GAPM_REPORT_INFO_SCAN_ADV_BIT        = (1 << 5),
    /// Directed advertising
    GAPM_REPORT_INFO_DIR_ADV_BIT         = (1 << 6),
};

/// @} GAPM_ENUM_API

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// @addtogroup GAPM_STRUCT_API
/// @{

/// Information about received ADV report
typedef struct gapm_adv_report_info
{
    /// Bit field providing information about the received report (see enum #gapm_adv_report_info_bf)
    uint8_t      info;
    /// Transmitter device address
    gap_bdaddr_t trans_addr;
    /// Target address (in case of a directed advertising report)
    gap_bdaddr_t target_addr;
    /// TX power (in dBm)
    int8_t       tx_pwr;
    /// RSSI (between -127 and +20 dBm)
    int8_t       rssi;
    /// Primary PHY on which advertising report has been received
    uint8_t      phy_prim;
    /// Secondary PHY on which advertising report has been received
    uint8_t      phy_second;
    /// Advertising SID
    /// Valid only for periodic advertising report
    uint8_t      adv_sid;
    /// Periodic advertising interval (in unit of 1.25ms, min is 7.5ms)
    /// Valid only for periodic advertising report
    uint16_t     period_adv_intv;
} gapm_adv_report_info_t;

/// Scan Window operation parameters
/*@TRACE*/
typedef struct gapm_scan_wd_op_param
{
    /// Scan interval (N * 0.625 ms)
    uint16_t scan_intv;
    /// Scan window (N * 0.625 ms)
    uint16_t scan_wd;
} gapm_scan_wd_op_param_t;


/// information about IQ report
typedef struct gapm_iq_report_info
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
} gapm_iq_report_info_t;

/// Controller antenna information
typedef struct gapm_antenna_inf
{
    /// Supported switching sampling rates bit field (see enum #gapm_switch_sampling_rate)
    uint8_t     supp_switching_sampl_rates;
    /// Number of antennae
    uint8_t     antennae_num;
    /// Max length of switching pattern (number of antenna IDs in the pattern)
    uint8_t     max_switching_pattern_len;
    /// Max CTE length
    uint8_t     max_cte_len;
} gapm_antenna_inf_t;

/// Suggested default data length
typedef struct gapm_sugg_dflt_data_len
{
    ///Host's suggested value for the Controller's maximum transmitted number of payload octets
    uint16_t suggted_max_tx_octets;
    ///Host's suggested value for the Controller's maximum packet transmission time
    uint16_t suggted_max_tx_time;
} gapm_sugg_dflt_data_len_t;

/// Maximum LE data length
typedef struct gapm_max_le_data_len
{
    ///Maximum number of payload octets that the local Controller supports for transmission
    uint16_t suppted_max_tx_octets;
    ///Maximum time, in microseconds, that the local Controller supports for transmission
    uint16_t suppted_max_tx_time;
    ///Maximum number of payload octets that the local Controller supports for reception
    uint16_t suppted_max_rx_octets;
    ///Maximum time, in microseconds, that the local Controller supports for reception
    uint16_t suppted_max_rx_time;
} gapm_max_le_data_len_t;


/// Transmit powers range supported by the controller
typedef struct gapm_tx_pwr_rng
{
    /// Minimum TX power
    int8_t min_tx_pwr;
    /// Maximum TX power
    int8_t max_tx_pwr;
} gapm_tx_pwr_rng_t;

/// TX/RX RF path compensation values
typedef struct gapm_rf_path_comp
{
    /// RF TX path compensation
    int16_t tx;
    /// RF RX path compensation
    int16_t rx;
} gapm_rf_path_comp_t;

/// @} GAPM_STRUCT_API

/*
 * INTERFACES
 ****************************************************************************************
 */

/// @addtogroup GAPM_ACTV_ITF_API
/// @{

///  Callback structure required to create a \glos{LE}  activity
typedef struct gapm_le_actv_cb
{
    /// Inherits Activity callback interface
    gapm_actv_cb_t actv;

    /**
     ****************************************************************************************
     * Callback executed when Random (resolvable or non-resolvable) address has been updated.
     *
     * @note Optional callback. Set it to NULL to ignore event reception
     *
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] actv_idx      Activity local index
     * @param[in] p_addr        Pointer to the updated Private Address (resolvable or non-resolvable)
     ****************************************************************************************
     */
    void (*addr_updated)(uint32_t dummy, uint8_t actv_idx, const gap_addr_t* p_addr);
} gapm_le_actv_cb_t;
/// @} GAPM_ACTV_ITF_API

/// @addtogroup GAPM_INFO_API
/// @{

/**
 ***************************************************************************************
 * @brief Function executed when procedure execution is over.
 *
 * @param[in] dummy         Dummy parameter provided by upper layer application
 * @param[in] status        Procedure execution status  (see enum #hl_err)
 * @param[in] p_info        Pointer to controller antenna informations
 *                          (NULL if status != GAP_ERR_NO_ERROR)
 ***************************************************************************************
 */
typedef void (*gapm_antenna_inf_cb)(uint32_t dummy, uint16_t status, const gapm_antenna_inf_t *p_info);

/**
 ***************************************************************************************
 * @brief Function executed when procedure execution is over.
 *
 * @param[in] dummy         Dummy parameter provided by upper layer application
 * @param[in] status        Procedure execution status  (see enum #hl_err)
 * @param[in] p_info        Pointer to controller default data length
 *                          (NULL if status != GAP_ERR_NO_ERROR)
 ***************************************************************************************
 */
typedef void (*gapm_sugg_dflt_data_len_cb)(uint32_t dummy, uint16_t status, const gapm_sugg_dflt_data_len_t *p_info);

/**
 ***************************************************************************************
 * @brief Function executed when procedure execution is over.
 *
 * @param[in] dummy         Dummy parameter provided by upper layer application
 * @param[in] status        Procedure execution status  (see enum #hl_err)
 * @param[in] p_info        Pointer to controller maximum LE data length information
 *                          (NULL if status != GAP_ERR_NO_ERROR)
 ***************************************************************************************
 */
typedef void (*gapm_max_le_data_len_cb)(uint32_t dummy, uint16_t status, const gapm_max_le_data_len_t *p_info);

/**
 ***************************************************************************************
 * @brief Function executed when procedure execution is over.
 *
 * @param[in] dummy         Dummy parameter provided by upper layer application
 * @param[in] status        Procedure execution status  (see enum #hl_err)
 * @param[in] p_rng         Pointer to TX Power Range value
 *                          (NULL if status != GAP_ERR_NO_ERROR)
 ***************************************************************************************
 */
typedef void (*gapm_tx_pwr_rng_cb)(uint32_t dummy, uint16_t status, const gapm_tx_pwr_rng_t* p_rng);

/**
 ***************************************************************************************
 * @brief Function executed when procedure execution is over.
 *
 * @param[in] dummy           Dummy parameter provided by upper layer application
 * @param[in] status          Procedure execution status  (see enum #hl_err)
 * @param[in] p_rf_path_comp  Pointer to RF Path compensation information
 *                            (NULL if status != GAP_ERR_NO_ERROR)
 ***************************************************************************************
 */
typedef void (*gapm_rf_path_comp_cb)(uint32_t dummy, uint16_t status, const gapm_rf_path_comp_t* p_rf_path_comp);

/// @} GAPM_INFO_API


/// @addtogroup GAPM_LE_LIST_API
/// @{


/**
 ***************************************************************************************
 * @brief Function executed when procedure execution is over.
 *
 * @param[in] dummy         Dummy parameter provided by upper layer application
 * @param[in] status        Procedure execution status  (see enum #hl_err)
 * @param[in] size          Size of the list
 ***************************************************************************************
 */
typedef void (*gapm_list_size_cb)(uint32_t dummy, uint16_t status, uint8_t size);

/// @} GAPM_LE_LIST_API


/// @addtogroup GAPM_LE_SEC_API
/// @{

/**
 ***************************************************************************************
 * @brief Function executed when procedure execution is over.
 *
 * @param[in] dummy           Dummy parameter provided by upper layer application
 * @param[in] status          Procedure execution status  (see enum #hl_err)
 * @param[in] p_pub_key       Pointer to ECDH Public key
 *                            (NULL if status != GAP_ERR_NO_ERROR)
 ***************************************************************************************
 */
typedef void (*gapm_public_key_cb)(uint32_t dummy, uint16_t status, const public_key_t* p_pub_key);

/**
 ***************************************************************************************
 * @brief Function executed when procedure execution is over.
 *
 * @param[in] dummy           Dummy parameter provided by upper layer application
 * @param[in] status          Procedure execution status  (see enum #hl_err)
 * @param[in] p_dh_key        Pointer to computed DH-Key
 *                            (NULL if status != GAP_ERR_NO_ERROR)
 ***************************************************************************************
 */
typedef void (*gapm_dh_key_cb)(uint32_t dummy, uint16_t status, const gap_dh_key_t* p_dh_key);


/**
 ***************************************************************************************
 * @brief Function executed when procedure get local or peer RPA execution is over.
 *
 * @param[in] dummy           Dummy parameter provided by upper layer application
 * @param[in] status          Procedure execution status  (see enum #hl_err)
 * @param[in] p_addr          Pointer to generated random address
 *                            (NULL if status != GAP_ERR_NO_ERROR)
 ***************************************************************************************
 */
typedef void (*gapm_get_rpa_res_cb)(uint32_t dummy, uint16_t status, const gap_addr_t* p_addr);

/**
 ***************************************************************************************
 * @brief Function executed when procedure execution is over.
 *
 * @param[in] dummy           Dummy parameter provided by upper layer application
 * @param[in] status          Procedure execution status  (see enum #hl_err)
 * @param[in] p_data          Pointer to generated LE OOB data
 *                            (NULL if status != GAP_ERR_NO_ERROR)
 ***************************************************************************************
 */
typedef void (*gapm_le_oob_cb)(uint32_t dummy, uint16_t status, const gap_oob_t* p_data);


/**
 ***************************************************************************************
 * @brief Function executed when procedure execution is over.
 *
 * @param[in] status          Procedure execution status  (see enum #hl_err)
 * @param[in] p_addr          Pointer to generated random address
 *                            (NULL if status != GAP_ERR_NO_ERROR)
 ***************************************************************************************
 */
typedef void (*gapm_random_address_cb)(uint16_t status, const gap_addr_t* p_addr);

/**
 ***************************************************************************************
 * @brief Function executed when AES procedure execution is over.
 *
 * @param[in] status          Procedure execution status  (see enum #hl_err)
 * @param[in] p_cipher        Pointer to ciphered data
 *                            (NULL if status != GAP_ERR_NO_ERROR)
 ***************************************************************************************
 */
typedef void (*gapm_aes_cipher_res_cb)(uint16_t status, const gap_aes_cipher_t* p_cipher);

/**
 ***************************************************************************************
 * @brief Function executed when random number procedure is over.
 *
 * @param[in] status          Procedure execution status  (see enum #hl_err)
 * @param[in] p_rand          Pointer to structure that contains 128-bit random number
 *                            (NULL if status != GAP_ERR_NO_ERROR)
 ***************************************************************************************
 */
typedef void (*gapm_random_number_res_cb)(uint16_t status, const gap_aes_rand_nb_t* p_rand);

/**
 ***************************************************************************************
 * @brief Function executed when address resolution is over.
 *
 * @param[in] status          Procedure execution status  (see enum #hl_err)
 * @param[in] p_addr          Pointer to resolvable private address
 * @param[in] p_irk           Pointer to IRK that correspond to RPA resolution
 *                            (NULL if status != GAP_ERR_NO_ERROR)
 ***************************************************************************************
 */
typedef void (*gapm_resolve_address_res_cb)(uint16_t status, const gap_addr_t* p_addr, const gap_sec_key_t* p_irk);

/// @} GAPM_LE_SEC_API

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/// @addtogroup GAPM_CONFIG_API
/// @{

/**
 ***************************************************************************************
 * @brief Set device channel map
 *
 * Application should wait #gapm_proc_cmp_cb callback execution before starting a new procedure
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] p_chmap   Pointer to new channel map.
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_proc_cmp_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_set_le_chmap(uint32_t dummy, const le_chnl_map_t* p_chmap, gapm_proc_cmp_cb cmp_cb);

/**
 ***************************************************************************************
 * @brief Start Channel Assessment activity
 *
 * Application should wait #gapm_proc_cmp_cb callback execution before starting a new procedure
 *
 * @param[in] dummy                 Dummy parameter provided by upper layer application
 * @param[in] scan_win_duration     Scan window duration in us
 * @param[in] scan_duration_min     Minimum Channel Scanning event in us
 * @param[in] scan_duration_max     Maximum Channel Scanning event in us
 * @param[in] intv                  Channel Scanning interval in Time = N*1.25ms
 * @param[in] cmp_cb                Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_proc_cmp_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_start_channel_assessment(uint32_t dummy, uint32_t scan_win_duration, const uint32_t scan_duration_min,
                                       const uint32_t scan_duration_max, const uint16_t intv, gapm_proc_cmp_cb cmp_cb);

/**
 ***************************************************************************************
 * @brief Stop Channel Assessment activity
 *
 * Application should wait #gapm_proc_cmp_cb callback execution before starting a new procedure
 *
 * @param[in] dummy                 Dummy parameter provided by upper layer application
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_proc_cmp_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_stop_channel_assessment(uint32_t dummy, gapm_proc_cmp_cb cmp_cb);

/**
 ***************************************************************************************
 * @brief Set device IRK used for resolvable random BD address generation
 *
 * @param[in] p_irk     Pointer to device irk (LSB first).
 *
 * @return Execution status (see enum #hl_err).
 ***************************************************************************************
 */
uint16_t gapm_set_irk(const gap_sec_key_t* p_irk);


/// @} GAPM_CONFIG_API

/// @addtogroup GAPM_INFO_API
/// @{

/**
 ***************************************************************************************
 * @brief Get controller antenna information. Information returned in res_cb function
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_antenna_inf_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_get_antenna_inf(uint32_t dummy, gapm_antenna_inf_cb res_cb);

/**
 ***************************************************************************************
 * @brief Get information about suggested default data length. Information returned in res_cb function
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_sugg_dflt_data_len_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_get_sugg_dflt_data_len(uint32_t dummy, gapm_sugg_dflt_data_len_cb res_cb);

/**
 ***************************************************************************************
 * @brief Get information about maximum LE data length. Information returned in res_cb function
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_max_le_data_len_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_get_max_le_data_len(uint32_t dummy, gapm_max_le_data_len_cb res_cb);

/**
 ***************************************************************************************
 * @brief Get TX Power range value. Information returned in res_cb function
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_tx_pwr_rng_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_get_tx_pwr_rng(uint32_t dummy, gapm_tx_pwr_rng_cb res_cb);


/**
 ***************************************************************************************
 * @brief Get RF Path compensation values. Information returned in res_cb function
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_rf_path_comp_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_get_rf_path_comp(uint32_t dummy, gapm_rf_path_comp_cb res_cb);

/// @} GAPM_INFO_API

/// @addtogroup GAPM_LE_LIST_API
/// @{

/**
 ***************************************************************************************
 * @brief Fill white list entries
 *
 * White list is cleared before inserting new entries
 * Application should wait #gapm_proc_cmp_cb callback execution before starting a new procedure
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] size      Size of the array size
 * @param[in] p_array   Pointer white list entries. Information must stay valid and available after function execution
 *                      and until procedure completes (global variable).
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_proc_cmp_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_fill_white_list(uint32_t dummy, uint8_t size, const gap_bdaddr_t* p_array, gapm_proc_cmp_cb cmp_cb);

/**
 ***************************************************************************************
 * @brief Fill resolving address list entries
 *
 * Resolving list is cleared before inserting new entries
 * Application should wait #gapm_proc_cmp_cb callback execution before starting a new procedure
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] size      Size of the array size
 * @param[in] p_array   Pointer white list entries. Information must stay valid and available after function execution
 *                      and until procedure completes (global variable).
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_proc_cmp_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_fill_resolving_address_list(uint32_t dummy, uint8_t size, const gap_ral_dev_info_t* p_array,
                                         gapm_proc_cmp_cb cmp_cb);

/**
 ***************************************************************************************
 * @brief Fill periodic advertising list entries
 *
 * Resolving list is cleared before inserting new entries
 * Application should wait #gapm_proc_cmp_cb callback execution before starting a new procedure
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] size      Size of the array size
 * @param[in] p_array   Pointer white list entries. Information must stay valid and available after function execution
 *                      and until procedure completes (global variable).
 * @param[in] cmp_cb    Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_proc_cmp_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_fill_periodic_adv_list(uint32_t dummy, uint8_t size, const gap_per_adv_bdaddr_t* p_array,
                                     gapm_proc_cmp_cb cmp_cb);

/**
 ***************************************************************************************
 * @brief Retrieve local resolvable private address generated by controller for a specific peer identity
 *
 * Application should wait #gapm_get_rpa_res_cb callback execution before starting a new procedure
 *
 * @param[in] dummy             Dummy parameter provided by upper layer application.
 * @param[in] p_peer_identity   Pointer to peer identity address.
 * @param[in] res_cb            Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_get_rpa_res_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_get_ral_local_rpa(uint32_t dummy, const gap_bdaddr_t* p_peer_identity, gapm_get_rpa_res_cb res_cb);


/**
 ***************************************************************************************
 * @brief Retrieve peer resolvable private address detected by controller for a specific peer identity
 *
 * Application should wait #gapm_get_rpa_res_cb callback execution before starting a new procedure
 *
 * @param[in] dummy             Dummy parameter provided by upper layer application.
 * @param[in] p_peer_identity   Pointer to peer identity address.
 * @param[in] res_cb            Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_get_rpa_res_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_get_ral_peer_rpa(uint32_t dummy, const gap_bdaddr_t* p_peer_identity, gapm_get_rpa_res_cb res_cb);


/**
 ***************************************************************************************
 * @brief Get white list size. Information returned in res_cb function
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_list_size_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_get_wlist_size(uint32_t dummy, gapm_list_size_cb res_cb);

/**
 ***************************************************************************************
 * @brief Get periodic advertising list size. Information returned in res_cb function
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_list_size_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_get_pal_size(uint32_t dummy, gapm_list_size_cb res_cb);

/**
 ***************************************************************************************
 * @brief Get resolving address list size. Information returned in res_cb function
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_list_size_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_get_ral_size(uint32_t dummy, gapm_list_size_cb res_cb);

/// @} GAPM_LE_LIST_API


/// @addtogroup GAPM_LE_SEC_API
/// @{

/**
 ***************************************************************************************
 * @brief Generate a random number.
 *
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_random_address_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_generate_random_number(gapm_random_number_res_cb res_cb);

/**
 ***************************************************************************************
 * @brief Cipher 128-bit data using AES
 *
 * @param[in] p_key     Pointer to 128-bit key used for ciphering
 * @param[in] p_data    Pointer to 128-bit data to cipher
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_random_number_res_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_aes_cipher(const uint8_t* p_key, const uint8_t* p_data, gapm_aes_cipher_res_cb res_cb);


/**
 ***************************************************************************************
 * @brief De-Cipher 128-bit data using AES - Shall be supported by HW
 *
 * @param[in] p_key     Pointer to 128-bit key used for deciphering
 * @param[in] p_data    Pointer to 128-bit data to decipher
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_random_number_res_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_aes_decipher(const uint8_t* p_key, const uint8_t* p_data, gapm_aes_cipher_res_cb res_cb);

/**
 ***************************************************************************************
 * @brief Generate a random address.
 *
 * @param[in] rnd_type  Random address type see enum #gap_rnd_addr_type
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_random_address_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_generate_random_address(uint8_t rnd_type, gapm_random_address_cb res_cb);

/**
 ***************************************************************************************
 * @brief Resolve a resolvable private address
 *
 * @param[in] p_addr    Pointer to random private address
 * @param[in] nb_irk    Number of IRKs
 * @param[in] p_irk     Pointer to array that contain IRK(s)
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_resolve_address_res_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_resolve_address(const gap_addr_t* p_addr, uint8_t nb_irk, const gap_sec_key_t* p_irk,
                              gapm_resolve_address_res_cb res_cb);


/**
 ***************************************************************************************
 * @brief Get ECDH public key value (New key pair generated each time this function is called)
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_public_key_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_get_public_key(uint32_t dummy, gapm_public_key_cb res_cb);

/**
 ***************************************************************************************
 * @brief Comnpute DH-Key using own ECDH private key and a given public key
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] p_pub_key Public key from a peer device
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_dh_key_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_compute_dh_key(uint32_t dummy, const public_key_t* p_pub_key, gapm_dh_key_cb res_cb);

/**
 ***************************************************************************************
 * @brief Generate LE OOB data using ECDH (New ECDH key pair generated each time this function is called)
 *        OOB data shall be used onto following pairing onto a LE connection
 *
 *        OOB data must be conveyed to peer device through an out of band method.
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_le_oob_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_generate_le_oob_data(uint32_t dummy, gapm_le_oob_cb res_cb);

/// @} GAPM_LE_SEC_API

#endif /* GAPM_LE_H_ */
