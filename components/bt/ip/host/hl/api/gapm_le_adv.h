/**
 ****************************************************************************************
 *
 * @file gapm_le_adv.h
 *
 * @brief Generic Access Profile Manager - Low Energy Advertising Activities
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


#ifndef GAPM_LE_ADV_H_
#define GAPM_LE_ADV_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gapm_le.h"

/**
 ****************************************************************************************
 * @addtogroup GAPM_LE_ADV_ACTV_API LE Advertising and Periodic Advertising
 *
 * @ingroup GAPM_ACTV_API
 *
 * @brief Create and control Advertising or Periodic Advertising activities.
 *
 * Application can control several advertising activities in parallel.
 *
 * The application must follow the #gapm_adv_actv_cb_t callback interface to handle activities events:
 * \snippet{lineno} app_connectable_adv.c APP_CONNECTABLE_ADV_ACTV_CB
 *
 * Application can then create an advertising activity using:
 *  - #gapm_adv_legacy_create: Legacy Advertising
 *  - #gapm_adv_ext_create: Extended Advertising
 *  - #gapm_adv_periodic_create: Periodic Advertising
 *  - #gapm_adv_periodic_with_cte_create: Periodic Advertising with Constant Tone Extension
 *
 * example:
 * \snippet{lineno} app_connectable_adv.c APP_CONNECTABLE_ADV_ACTV_CREATE_AND_START
 *
 *
 * Once activity is created, #gapm_adv_actv_cb_t.created is called, then application can set
 * advertising data and start the activity.
 *
 * An application example is available in \ref app_test_le_periph.c
 *
 * @note At least #GAP_ROLE_PERIPHERAL role is required for a connectable advertising and
 *       #GAP_ROLE_BROADCASTER for a non-connectable advertising.
 *
 * @{
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// Type of advertising that can be created
enum gapm_adv_type
{
    /// Legacy advertising
    GAPM_ADV_TYPE_LEGACY = 0,
    /// Extended advertising
    GAPM_ADV_TYPE_EXTENDED,
    /// Periodic advertising
    GAPM_ADV_TYPE_PERIODIC,
};


/// Advertising properties bit field bit positions
enum gapm_adv_prop_bf
{
    /// Indicate that advertising is connectable, reception of CONNECT_REQ or AUX_CONNECT_REQ
    /// PDUs is accepted. Not applicable for periodic advertising.
    GAPM_ADV_PROP_CONNECTABLE_POS     = 0,
    GAPM_ADV_PROP_CONNECTABLE_BIT     = 0x0001,

    /// Indicate that advertising is scannable, reception of SCAN_REQ or AUX_SCAN_REQ PDUs is
    /// accepted
    GAPM_ADV_PROP_SCANNABLE_POS       = 1,
    GAPM_ADV_PROP_SCANNABLE_BIT       = 0x0002,

    /// Indicate that advertising targets a specific device. Only apply in following cases:
    ///   - Legacy advertising: if connectable
    ///   - Extended advertising: connectable or (non connectable and non discoverable)
    GAPM_ADV_PROP_DIRECTED_POS        = 2,
    GAPM_ADV_PROP_DIRECTED_BIT        = 0x0004,

    /// Indicate that High Duty Cycle has to be used for advertising on primary channel
    /// Apply only if created advertising is not an extended advertising
    GAPM_ADV_PROP_HDC_POS             = 3,
    GAPM_ADV_PROP_HDC_BIT             = 0x0008,

    /// Bit 4 is reserved
    GAPM_ADV_PROP_RESERVED_4_POS      = 4,
    GAPM_ADV_PROP_RESERVED_4_BIT      = 0x0010,

    /// Enable anonymous mode. Device address won't appear in send PDUs
    /// Valid only if created advertising is an extended advertising
    GAPM_ADV_PROP_ANONYMOUS_POS       = 5,
    GAPM_ADV_PROP_ANONYMOUS_BIT       = 0x0020,

    /// Include TX Power in the extended header of the advertising PDU.
    /// Valid only if created advertising is not a legacy advertising
    GAPM_ADV_PROP_TX_PWR_POS          = 6,
    GAPM_ADV_PROP_TX_PWR_BIT          = 0x0040,

    /// Include TX Power in the periodic advertising PDU.
    /// Valid only if created advertising is a periodic advertising
    GAPM_ADV_PROP_PER_TX_PWR_POS      = 7,
    GAPM_ADV_PROP_PER_TX_PWR_BIT      = 0x0080,

    /// Indicate if application must be informed about received scan requests PDUs
    GAPM_ADV_PROP_SCAN_REQ_NTF_EN_POS = 8,
    GAPM_ADV_PROP_SCAN_REQ_NTF_EN_BIT = 0x0100,
};

/// Advertising discovery mode
enum gapm_adv_disc_mode
{
    /// Mode in non-discoverable
    GAPM_ADV_MODE_NON_DISC = 0,
    /// Mode in general discoverable
    GAPM_ADV_MODE_GEN_DISC,
    /// Mode in limited discoverable
    GAPM_ADV_MODE_LIM_DISC,
    /// Broadcast mode without presence of AD_TYPE_FLAG in advertising data
    GAPM_ADV_MODE_BEACON,
    GAPM_ADV_MODE_MAX,
};

// -------------------------------------------------------------------------------------
// Masks for advertising properties
// -------------------------------------------------------------------------------------

/// Advertising properties configurations for legacy advertising
enum gapm_leg_adv_prop
{
    /// Non connectable non scannable advertising
    GAPM_ADV_PROP_NON_CONN_NON_SCAN_MASK  = 0x0000,
    /// Broadcast non scannable advertising - Discovery mode must be Non Discoverable
    GAPM_ADV_PROP_BROADCAST_NON_SCAN_MASK = GAPM_ADV_PROP_NON_CONN_NON_SCAN_MASK,
    /// Non connectable scannable advertising
    GAPM_ADV_PROP_NON_CONN_SCAN_MASK      = GAPM_ADV_PROP_SCANNABLE_BIT,
    /// Broadcast non scannable advertising - Discovery mode must be Non Discoverable
    GAPM_ADV_PROP_BROADCAST_SCAN_MASK     = GAPM_ADV_PROP_NON_CONN_SCAN_MASK,
    /// Undirected connectable advertising
    GAPM_ADV_PROP_UNDIR_CONN_MASK         = GAPM_ADV_PROP_CONNECTABLE_BIT | GAPM_ADV_PROP_SCANNABLE_BIT,
    /// Directed connectable advertising
    GAPM_ADV_PROP_DIR_CONN_MASK           = GAPM_ADV_PROP_DIRECTED_BIT | GAPM_ADV_PROP_CONNECTABLE_BIT,
    /// Directed connectable with Low Duty Cycle
    GAPM_ADV_PROP_DIR_CONN_LDC_MASK       = GAPM_ADV_PROP_DIR_CONN_MASK,
    /// Directed connectable with High Duty Cycle
    GAPM_ADV_PROP_DIR_CONN_HDC_MASK       = GAPM_ADV_PROP_DIR_CONN_MASK | GAPM_ADV_PROP_HDC_BIT,
};

/// Advertising properties configurations for extended advertising
enum gapm_ext_adv_prop
{
    /// Non connectable non scannable extended advertising
    GAPM_EXT_ADV_PROP_NON_CONN_NON_SCAN_MASK = 0x0000,
    /// Non connectable scannable extended advertising
    GAPM_EXT_ADV_PROP_NON_CONN_SCAN_MASK     = GAPM_ADV_PROP_SCANNABLE_BIT,
    /// Non connectable scannable directed extended advertising
    GAPM_EXT_ADV_PROP_NON_CONN_SCAN_DIR_MASK = GAPM_ADV_PROP_SCANNABLE_BIT | GAPM_ADV_PROP_DIRECTED_BIT,
    /// Non connectable anonymous directed extended advertising
    GAPM_EXT_ADV_PROP_ANONYM_DIR_MASK        = GAPM_ADV_PROP_ANONYMOUS_BIT | GAPM_ADV_PROP_DIRECTED_BIT,
    /// Undirected connectable extended advertising
    GAPM_EXT_ADV_PROP_UNDIR_CONN_MASK        = GAPM_ADV_PROP_CONNECTABLE_BIT,
    /// Directed connectable extended advertising
    GAPM_EXT_ADV_PROP_DIR_CONN_MASK          = GAPM_ADV_PROP_CONNECTABLE_BIT | GAPM_ADV_PROP_DIRECTED_BIT,
};

/// Advertising properties configurations for periodic advertising
enum gapm_per_adv_prop
{
    /// Undirected periodic advertising
    GAPM_PER_ADV_PROP_UNDIR_MASK = 0x0000,
    /// Directed periodic advertising
    GAPM_PER_ADV_PROP_DIR_MASK   = GAPM_ADV_PROP_DIRECTED_BIT,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Configuration for advertising on primary channel
/*@TRACE*/
typedef struct gapm_adv_prim_cfg
{
    /// Minimum advertising interval (in unit of 625us). Must be greater than 20ms
    uint32_t adv_intv_min;
    /// Maximum advertising interval (in unit of 625us). Must be greater than 20ms
    uint32_t adv_intv_max;
    /// Bit field indicating the channel mapping
    uint8_t chnl_map;
    /// Indicate on which PHY primary advertising has to be performed (see enum #gapm_phy_type)
    /// Note that LE 2M PHY is not allowed and that legacy advertising only support LE 1M PHY
    uint8_t phy;
} gapm_adv_prim_cfg_t;

/// Configuration for advertising on secondary channel
/*@TRACE*/
typedef struct gapm_adv_second_cfg
{
    /// Maximum number of advertising events the controller can skip before sending the
    /// AUX_ADV_IND packets. 0 means that AUX_ADV_IND PDUs shall be sent prior each
    /// advertising events
    uint8_t max_skip;
    /// Indicate on which PHY secondary advertising has to be performed (see enum #gapm_phy_type)
    uint8_t phy;
    /// Advertising SID
    uint8_t adv_sid;
} gapm_adv_second_cfg_t;

/// Configuration for periodic advertising
/*@TRACE*/
typedef struct gapm_adv_period_cfg
{
    /// Minimum periodic advertising interval (in unit of 1.25ms). Must be greater than 20ms
    uint16_t    interval_min;
    /// Maximum periodic advertising interval (in unit of 1.25ms). Must be greater than 20ms
    uint16_t    interval_max;
} gapm_adv_period_cfg_t;

/// Configuration for constant tone extension
/*@TRACE*/
typedef struct gapm_adv_cte_cfg
{
    /// CTE count (number of CTEs to transmit in each periodic advertising interval, range 0x01 to 0x10)
    /// 0 to disable CTE transmission
    uint8_t     count;
    /// CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us) (see enum #gap_cte_type)
    uint8_t     type;
    /// CTE length (in 8us unit)
    uint8_t     length;
} gapm_adv_cte_cfg_t;

/// Advertising parameters for advertising creation
/*@TRACE*/
typedef struct gapm_adv_create_param
{
    /// Bit field value provided advertising properties
    /// (see enum #gapm_leg_adv_prop, see enum #gapm_ext_adv_prop and see enum #gapm_per_adv_prop for bit signification)
    uint16_t             prop;
    /// Discovery mode (see enum #gapm_adv_disc_mode)
    uint8_t              disc_mode;
    /// Maximum power level at which the advertising packets have to be transmitted
    /// (between -127 and 126 dBm)
    int8_t               max_tx_pwr;
    /// Advertising filtering policy (see enum #gap_adv_filter_policy)
    uint8_t              filter_pol;
    /// Peer address configuration (only used in case of directed advertising)
    gap_bdaddr_t         peer_addr;
    /// Configuration for primary advertising
    gapm_adv_prim_cfg_t  prim_cfg;
} gapm_adv_create_param_t;

/// Additional advertising parameters
/*@TRACE*/
typedef struct gapm_adv_param
{
    /// Advertising duration (in unit of 10ms). 0 means that advertising continues
    /// until the host disable it
    uint16_t duration;
    /// Maximum number of extended advertising events the controller shall attempt to send prior to
    /// terminating the extending advertising
    /// Valid only if extended advertising
    uint8_t max_adv_evt;
} gapm_adv_param_t;

/*
 * INTERFACES
 ****************************************************************************************
 */

/// Callback structure required to create an advertising activity
typedef struct gapm_adv_actv_cb
{
    /// Inherits Activity callback interface
    gapm_le_actv_cb_t hdr;

    /**
     ****************************************************************************************
     * Callback executed when advertising activity is created
     *
     * @note Mandatory callback. Shall be set to a valid callback
     *
     * @param[in] dummy      Dummy parameter provided by upper layer application
     * @param[in] actv_idx   Activity local index
     * @param[in] tx_pwr     Selected TX power for advertising activity
     *
     ****************************************************************************************
     */
    void (*created)(uint32_t dummy, uint8_t actv_idx, int8_t tx_pwr);


    /**
     ****************************************************************************************
     * Callback executed when receiving a scan request (if enabled in advertising properties)
     *
     * @note Optional callback. Set it to NULL to ignore event reception
     *
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] actv_idx      Activity local index
     * @param[in] actv_type     Activity type (see enum #gapm_actv_type)
     * @param[in] p_addr        Pointer to transmitter device identity address
     ****************************************************************************************
     */
    void (*scan_req_received)(uint32_t dummy, uint8_t actv_idx, const gap_bdaddr_t* p_addr);


    /**
     ****************************************************************************************
     * Callback executed for periodic ADV to indicate that non periodic advertising is stopped.
     *
     * @note Optional callback. Mandatory for a periodic ADV
     *
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] actv_idx      Activity local index
     * @param[in] reason        Activity stop reason (see enum #hl_err)
     ****************************************************************************************
     */
    void (*ext_adv_stopped)(uint32_t dummy, uint8_t actv_idx, uint16_t reason);
} gapm_adv_actv_cb_t;


/**
 ****************************************************************************************
 * @brief Callback function allowing to inform a module that address is about to be renew for an advertising
 * activity
 ****************************************************************************************
 */
typedef void (*gapm_adv_cb_addr_renew)(void);

/**
 ***************************************************************************************
 * @brief Function executed when procedure execution is over.
 *
 * @param[in] dummy         Dummy parameter provided by upper layer application
 * @param[in] status        Procedure execution status  (see enum #hl_err)
 * @param[in] nb_adv_set    Number of advertising set supported by controller
 ***************************************************************************************
 */
typedef void (*gapm_nb_adv_set_cb)(uint32_t dummy, uint16_t status, uint8_t nb_adv_set);

/**
 ***************************************************************************************
 * @brief Function executed when procedure execution is over.
 *
 * @param[in] dummy         Dummy parameter provided by upper layer application
 * @param[in] status        Procedure execution status  (see enum #hl_err)
 * @param[in] max_adv_len   Maximum advertising data length
 ***************************************************************************************
 */
typedef void (*gapm_max_adv_len_cb)(uint32_t dummy, uint16_t status, uint16_t max_adv_len);

/**
 ***************************************************************************************
 * @brief Function executed when procedure execution is over.
 *
 * @param[in] dummy         Dummy parameter provided by upper layer application
 * @param[in] status        Procedure execution status  (see enum #hl_err)
 * @param[in] power_lvl     Advertising channel Tx power level
 ***************************************************************************************
 */
typedef void (*gapm_adv_tx_power_cb)(uint32_t dummy, uint16_t status, int8_t power_lvl);

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Create an Legacy advertising activity.
 *
 * @param[in] dummy             Dummy parameter provided by upper layer application
 * @param[in] own_addr_type     Own address type (see enum #gapm_own_addr)
 * @param[in] p_param           Pointer to advertising parameters
 * @param[in] p_cbs             Activity Callback interface
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_actv_cb_t.proc_cmp callback execution
 ****************************************************************************************
 */
uint16_t gapm_adv_legacy_create(uint32_t dummy, uint8_t own_addr_type, const gapm_adv_create_param_t* p_param,
                                const gapm_adv_actv_cb_t* p_cbs);

/**
 ****************************************************************************************
 * @brief Create an extended advertising activity.
 *
 * @param[in] dummy             Dummy parameter provided by upper layer application
 * @param[in] own_addr_type     Own address type (see enum #gapm_own_addr)
 * @param[in] p_param           Pointer to advertising parameters
 * @param[in] p_second_cfg      Pointer to configuration for secondary advertising
 * @param[in] p_cbs             Activity Callback interface
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_actv_cb_t.proc_cmp callback execution
 ****************************************************************************************
 */
uint16_t gapm_adv_ext_create(uint32_t dummy, uint8_t own_addr_type, const gapm_adv_create_param_t* p_param,
                             const gapm_adv_second_cfg_t* p_second_cfg,  const gapm_adv_actv_cb_t* p_cbs);

/**
 ****************************************************************************************
 * @brief Create a Periodic advertising activity.
 *
 * @param[in] dummy             Dummy parameter provided by upper layer application
 * @param[in] own_addr_type     Own address type (see enum #gapm_own_addr)
 * @param[in] p_param           Pointer to advertising parameters
 * @param[in] p_second_cfg      Pointer to configuration for secondary advertising
 * @param[in] p_period_cfg      Pointer to configuration for periodic advertising
 * @param[in] p_cbs             Activity Callback interface
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_actv_cb_t.proc_cmp callback execution
 ****************************************************************************************
 */
uint16_t gapm_adv_periodic_create(uint32_t dummy, uint8_t own_addr_type, const gapm_adv_create_param_t* p_param,
                                  const gapm_adv_second_cfg_t* p_second_cfg, const gapm_adv_period_cfg_t* p_period_cfg,
                                  const gapm_adv_actv_cb_t* p_cbs);

/**
 ****************************************************************************************
 * @brief Create a Periodic advertising activity with constant tone extension.
 *
 * @param[in] dummy                 Dummy parameter provided by upper layer application
 * @param[in] own_addr_type         Own address type (see enum #gapm_own_addr)
 * @param[in] p_param               Pointer to advertising parameters
 * @param[in] p_second_cfg          Pointer to configuration for secondary advertising
 * @param[in] p_period_cfg          Pointer to configuration for periodic advertising
 * @param[in] p_cte_cfg             Pointer to CTE Configuration
 * @param[in] switching_pattern_len Length of switching pattern (number of antenna IDs in the pattern)
 * @param[in] p_antenna_id          Pointer to array of antenna IDs
 * @param[in] p_cbs                 Activity Callback interface
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_actv_cb_t.proc_cmp callback execution
 ****************************************************************************************
 */
uint16_t gapm_adv_periodic_with_cte_create(uint32_t dummy, uint8_t own_addr_type, const gapm_adv_create_param_t* p_param,
                       const gapm_adv_second_cfg_t* p_second_cfg, const gapm_adv_period_cfg_t* p_period_cfg,
                       const gapm_adv_cte_cfg_t* p_cte_cfg, uint8_t switching_pattern_len, const uint8_t* p_antenna_id,
                       const gapm_adv_actv_cb_t* p_cbs);


/**
 ****************************************************************************************
 * @brief Start advertising activity.
 *
 * @param[in] actv_idx          Activity local index
 * @param[in] p_param           Additional Advertising start parameters
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_actv_cb_t.proc_cmp callback execution
 ****************************************************************************************
 */
uint16_t gapm_adv_start(uint8_t actv_idx, const gapm_adv_param_t* p_param);

/**
 ****************************************************************************************
 * @brief Set advertising data
 *
 * @param[in] actv_idx          Activity local index
 * @param[in] p_data            Pointer to buffer that contains Advertising data
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_actv_cb_t.proc_cmp callback execution
 ****************************************************************************************
 */
uint16_t gapm_adv_set_data(uint8_t actv_idx, co_buf_t* p_data);

/**
 ****************************************************************************************
 * @brief Set scan response data
 *
 * @param[in] actv_idx          Activity local index
 * @param[in] p_data            Pointer to buffer that contains Advertising data
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_actv_cb_t.proc_cmp callback execution
 ****************************************************************************************
 */
uint16_t gapm_adv_set_scan_rsp(uint8_t actv_idx, co_buf_t* p_data);

/**
 ****************************************************************************************
 * @brief Set periodic advertising data
 *
 * @param[in] actv_idx  Activity local index
 * @param[in] p_data    Pointer to buffer that contains Advertising data
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_actv_cb_t.proc_cmp callback execution
 ****************************************************************************************
 */
uint16_t gapm_adv_set_period_data(uint8_t actv_idx, co_buf_t* p_data);

/**
 ***************************************************************************************
 * @brief Control transmission of constant tone extension with periodic advertising data
 *
 * @param[in] actv_idx  Activity local index
 * @param[in] enable    True to enable CTE transmission, False otherwise
 *
 * @return Execution status (see enum #hl_err).
 ***************************************************************************************
 */
uint16_t gapm_adv_periodic_cte_tx_ctl(uint8_t actv_idx, bool enable);

/**
 ***************************************************************************************
 * @brief Control flow of advertising reports. If disabled, reports are dropped without
 *        informing application.
 *
 * @param[in] enable   True to enable report, false to drop them
 ***************************************************************************************
 */
void gapm_adv_report_flow_ctrl(bool enable);

/**
 ***************************************************************************************
 * @brief Get advertising handle
 *
 * @param[in] actv_idx   Activity index
 *
 * @return Required advertising handle (see enum #hl_err).
 ***************************************************************************************
 */
uint8_t gapm_adv_hdl_get(uint8_t actv_idx);

/**
 ***************************************************************************************
 * @brief Get address used for an advertising activity
 *
 * @param[in] actv_idx   Activity index
 *
 * @return Pointer to required address
 ***************************************************************************************
 */
gap_addr_t* gapm_adv_get_addr(uint8_t actv_idx);

/**
 ***************************************************************************************
 * @brief Set callback function allowing to inform a block that BD Address is about to be renewed for an
 * advertising activity\n
 *  /// !!!! FOR INTERNAL USE ONLY !!!!
 *
 * @param[in] actv_idx          Activity index
 * @param[in] cb_addr_renew     Callback function
 *
 * @return Execution status (see enum #hl_err).
 ***************************************************************************************
 */
uint16_t gapm_adv_set_cb_addr_renew(uint8_t actv_idx, gapm_adv_cb_addr_renew cb_addr_renew);

/**
 ***************************************************************************************
 * @brief Get device advertising power level. TX power returned in res_cb function
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] res_cb    Function called when Advertising TX power procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_adv_tx_power_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_get_adv_tx_power(uint32_t dummy, gapm_adv_tx_power_cb res_cb);

/**
 ***************************************************************************************
 * @brief Get number of advertising set. Information returned in res_cb function
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_nb_adv_set_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_get_nb_adv_set(uint32_t dummy, gapm_nb_adv_set_cb res_cb);

/**
 ***************************************************************************************
 * @brief Get maximum advertising data length. Information returned in res_cb function
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_max_adv_len_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_get_max_adv_len(uint32_t dummy, gapm_max_adv_len_cb res_cb);
/// @} GAPM_LE_ADV_ACTV_API

#endif /* GAPM_LE_ADV_H_ */
