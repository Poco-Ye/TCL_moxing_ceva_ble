/**
 ****************************************************************************************
 *
 * @file gapm_le_init.h
 *
 * @brief Generic Access Profile Manager - Low Energy Initiating Activities
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


#ifndef GAPM_LE_INIT_H_
#define GAPM_LE_INIT_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gapm_le.h"

/**
 ****************************************************************************************
 * @addtogroup GAPM_LE_INIT_ACTV_API LE Initiating
 *
 * @ingroup GAPM_ACTV_API
 *
 * @brief Create and control LE Initiating activity to discover device name or establish an LE connection
 *
 * Even if application can create several initiating activities, only one can be active (started) at a time.
 *
 * The application must have a callback structure to handle activities events
 * \snippet{lineno} app_test_le_central.c APP_INIT_ACTV_CB
 *
 * Application can create an initiating activity using #gapm_init_create.
 *
 * example:
 * \snippet{lineno} app_test_le_central.c APP_CREATE_INIT
 *
 * Once activity is created, application can immediately start initiating.
 *
 * An application example is available in \ref app_test_le_central.c
 *
 * @note At least #GAP_ROLE_CENTRAL role is required
 * @{
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// Initiating Types
enum gapm_init_type
{
    /// Direct connection establishment, establish a connection with an indicated device
    GAPM_INIT_TYPE_DIRECT_CONN_EST = 0,
    /// Automatic connection establishment, establish a connection with all devices whose address is
    /// present in the white list
    GAPM_INIT_TYPE_AUTO_CONN_EST,
    /// Name discovery, Establish a connection with an indicated device in order to read content of its
    /// Device Name characteristic. Connection is closed once this operation is stopped.
    GAPM_INIT_TYPE_NAME_DISC,
};

/// Initiating Properties
enum gapm_init_prop
{
    /// Scan connectable advertisements on the LE 1M PHY. Connection parameters for the LE 1M PHY are provided
    GAPM_INIT_PROP_1M_BIT       = (1 << 0),
    /// Connection parameters for the LE 2M PHY are provided
    GAPM_INIT_PROP_2M_BIT       = (1 << 1),
    /// Scan connectable advertisements on the LE Coded PHY. Connection parameters for the LE Coded PHY are provided
    GAPM_INIT_PROP_CODED_BIT    = (1 << 2),
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Connection parameters
/*@TRACE*/
typedef struct gapm_conn_param
{
    /// Minimum value for the connection interval (in unit of 1.25ms). Shall be less than or equal to
    /// conn_intv_max value. Allowed range is 7.5ms to 4s.
    uint16_t conn_intv_min;
    /// Maximum value for the connection interval (in unit of 1.25ms). Shall be greater than or equal to
    /// conn_intv_min value. Allowed range is 7.5ms to 4s.
    uint16_t conn_intv_max;
    /// Slave latency. Number of events that can be missed by a connected slave device
    uint16_t conn_latency;
    /// Link supervision timeout (in unit of 10ms). Allowed range is 100ms to 32s
    uint16_t supervision_to;
    /// Recommended minimum duration of connection events (in unit of 625us)
    uint16_t ce_len_min;
    /// Recommended maximum duration of connection events (in unit of 625us)
    uint16_t ce_len_max;
} gapm_conn_param_t;

/// Initiating parameters
/*@TRACE*/
typedef struct gapm_init_param
{
    /// Initiating type (see enum #gapm_init_type)
    uint8_t                 type;
    /// Properties for the initiating procedure (see enum #gapm_init_prop for bit signification)
    uint8_t                 prop;
    /// Timeout for automatic connection establishment (in unit of 10ms). Cancel the procedure if not all
    /// indicated devices have been connected when the timeout occurs. 0 means there is no timeout
    uint16_t                conn_to;
    /// Scan window opening parameters for LE 1M PHY
    gapm_scan_wd_op_param_t scan_param_1m;
    /// Scan window opening parameters for LE Coded PHY
    gapm_scan_wd_op_param_t scan_param_coded;
    /// Connection parameters for LE 1M PHY
    gapm_conn_param_t       conn_param_1m;
    /// Connection parameters for LE 2M PHY
    gapm_conn_param_t       conn_param_2m;
    /// Connection parameters for LE Coded PHY
    gapm_conn_param_t       conn_param_coded;
    /// Address of peer device in case white list is not used for connection
    gap_bdaddr_t            peer_addr;
} gapm_init_param_t;

/*
 * INTERFACES
 ****************************************************************************************
 */

/// Callback structure required to create an Initiating activity
typedef struct gapm_init_actv_cb
{
    /// Inherits Activity callback interface
    gapm_le_actv_cb_t hdr;

    /**
     ****************************************************************************************
     * @brief Callback executed when Peer device name has been read on peer device
     *
     * @note Optional callback. Shall be set to a valid callback only for Name discovery procedure
     *
     * @param[in] dummy         Dummy parameter provided by upper layer application
     * @param[in] actv_idx      Activity Local identifier
     * @param[in] p_addr        Pointer to peer identity address information
     * @param[in] name_len      Length of peer device name
     * @param[in] p_name        Pointer to peer device name data
     ****************************************************************************************
     */
    void (*peer_name)(uint32_t dummy, uint8_t actv_idx, const gap_bdaddr_t* p_addr, uint16_t name_len,
                      const uint8_t* p_name);
} gapm_init_actv_cb_t;


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Create initiating connection activity
 *
 * @param[in]  dummy            Dummy parameter provided by upper layer application
 * @param[in]  own_addr_type    Own address type (see enum #gapm_own_addr)
 * @param[in]  p_cbs            Activity Callback interface
 * @param[out] p_actv_idx       Pointer used to return allocated activity index
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapm_init_create(uint32_t dummy, uint8_t own_addr_type, const gapm_init_actv_cb_t* p_cbs, uint8_t* p_actv_idx);

/**
 ****************************************************************************************
 * @brief Start connection creation
 *
 * @param[in] actv_idx          Activity local index
 * @param[in] p_param           Initiating parameters
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_actv_cb_t.proc_cmp callback execution
 ****************************************************************************************
 */
uint16_t gapm_init_start(uint8_t actv_idx, const gapm_init_param_t* p_param);

/// @} GAPM_LE_INIT_ACTV_API

#endif /* GAPM_LE_INIT_H_ */
