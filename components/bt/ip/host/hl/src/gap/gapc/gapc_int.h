/**
 ****************************************************************************************
 *
 * @file gapc_int.h
 *
 * @brief Generic Access Profile Controller Internal Header.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */
#ifndef _GAPC_INT_H_
#define _GAPC_INT_H_

/**
 ****************************************************************************************
 * @addtogroup GAPC_INT Generic Access Profile Controller Internals
 * @ingroup GAPC
 * @brief Handles ALL Internal GAPC API
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (GAPC_PRESENT)
#include "gapc.h"
#include "gapc_proc.h"
#include "../../inc/gap_hl_api.h"
#include "../../inc/hl_proc.h"
#include "co_bt.h"
#include "co_buf.h"
#include "co_time.h"
#include "co_djob.h"
#include "gapc_sec.h"    // GAPC Security API
#if(BLE_GAPC)
#include "gapc_le.h"     // GAPC LE API
#endif // (BLE_GAPC)


/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// number of GAP Controller Process
#define GAPC_IDX_MAX                                 HOST_CONNECTION_MAX


/**
 * Repeated Attempts Timer Configuration
 */
/// Repeated Attempts Timer default value
#define GAPC_LE_SMP_REP_ATTEMPTS_TIMER_DEF_VAL_MS         (2000)      //2s
/// Repeated Attempts Timer max value )
#define GAPC_LE_SMP_REP_ATTEMPTS_TIMER_MAX_VAL_MS         (30000)     //30s
/// Repeated Attempts Timer multiplier
#define GAPC_LE_SMP_REP_ATTEMPTS_TIMER_MULT               (2)

/// GAP Supported client feature mask
#define GAPC_CLI_FEAT_SUPPORTED_MASK                   (0x07)
/// GAP Supported server feature mask
#define GAPC_SRV_FEAT_SUPPORTED_MASK                   (0x01)

/// size of bit field array according to number of connection
#define GAPC_CON_ARRAY_BF_SIZE  ((HOST_CONNECTION_MAX + 7) / 8)

/// Connection information bit field
enum gapc_con_info_bf
{
    /// Connection type
    GAPC_CON_TYPE_POS          = 0,
    GAPC_CON_TYPE_BIT          = 0x01,
    /// Local connection role
    GAPC_ROLE_POS              = 1,
    GAPC_ROLE_BIT              = 0x02,
    /// A disconnection has been initiated
    GAPC_DISCONNECTING_POS     = 2,
    GAPC_DISCONNECTING_BIT     = 0x04,
    /// During a procedure, application can be involve (request + confirm)
    /// This bit is used to know if response is provided within request function call
    GAPC_IN_PROC_POS           = 3,
    GAPC_IN_PROC_BIT           = 0x08,
    /// Connection type; 1: LE Connection, 0: BT Classic Connection
    GAPC_LE_CON_TYPE_POS       = 4,
    GAPC_LE_CON_TYPE_BIT       = 0x10,
    /// Waiting for encryption procedure to be executed - pairing excluded if encryption on-going
    GAPC_WAIT_ENCRYPTION_POS   = 5,
    GAPC_WAIT_ENCRYPTION_BIT   = 0X20,
    /// Connection used for a name discovery procedure.
    /// Application not involved by connection establishment and disconnection.
    GAPC_IS_NAME_DISCOVERY_POS = 6,
    GAPC_IS_NAME_DISCOVERY_BIT = 0x40,
};


/// Bond  data information bit field
enum gapc_bond_info_bf
{
    /// Link Security Level
    GAPC_SEC_LVL_LSB           = 0,
    GAPC_SEC_LVL_MASK          = 0x0003,
    /// Link Bonded or not
    GAPC_BONDED_POS            = 2,
    GAPC_BONDED_BIT            = 0x0004,
    /// Encrypted connection or not
    GAPC_LE_ENCRYPTED_POS         = 3,
    GAPC_LE_ENCRYPTED_BIT         = 0x0008,
    /// Ltk or link key present and exchanged during pairing
    GAPC_ENC_KEY_PRESENT_POS   = 4,
    GAPC_ENC_KEY_PRESENT_BIT   = 0x0010,
    /// Used to know if bond data has been updated at client or server level
    GAPC_BOND_DATA_UPDATED_POS = 5,
    GAPC_BOND_DATA_UPDATED_BIT = 0x0020,
};

/// GAP Client and Server configuration bit field
///
///     7    6  5     4        3       2     1      0
/// +-------+--+--+------+----------+-----+------+-----+
/// | D_B_I | RFU | EATT | CLI_FEAT | MTU | RPAO | PCP |
/// +-------+--+--+------+----------+-----+------+-----+
///
/// - Bit [0]  : Preferred Connection parameters present in GAP DB
/// - Bit [1]  : Presence of Resolvable private address only in GAP DB
/// - Bit [2]  : Automatically start MTU exchange at connection establishment
/// - Bit [3]  : Enable automatic robust cache enable at connection establishment
/// - Bit [4]  : Automatically start establishment of Enhanced ATT bearers
/// - Bit [7]  : Trigger bond information to application even if devices are not bonded
enum gapc_cfg_bf
{
    /// Preferred Connection parameters present in GAP DB
    GAPC_SVC_PREF_CON_PAR_PRES_BIT          = 0x01,
    GAPC_SVC_PREF_CON_PAR_PRES_POS          = 0,
    /// Presence of Resolvable private address only in GAP DB
    GAPC_SVC_RSLV_PRIV_ADDR_ONLY_PRES_BIT   = 0x02,
    GAPC_SVC_RSLV_PRIV_ADDR_ONLY_PRES_POS   = 1,
    /// Automatically start MTU exchange at connection establishment
    GAPC_CLI_AUTO_MTU_EXCH_EN_BIT           = 0x04,
    GAPC_CLI_AUTO_MTU_EXCH_EN_POS           = 2,
    /// Enable automatic client features enabling at connection establishment
    GAPC_CLI_AUTO_FEAT_EN_BIT               = 0x08,
    GAPC_CLI_AUTO_FEAT_EN_POS               = 3,
    /// Automatically start establishment of Enhanced ATT bearers
    GAPC_CLI_AUTO_EATT_BIT                  = 0x10,
    GAPC_CLI_AUTO_EATT_POS                  = 4,
    /// Trigger bond information to application even if devices are not bonded
    GAPC_DBG_BOND_INFO_TRIGGER_BIT          = 0x80,
    GAPC_DBG_BOND_INFO_TRIGGER_POS          = 7,
};

/*
 * TYPE DECLARATIONS
 ****************************************************************************************
 */
typedef struct gapc_con gapc_con_t;


typedef struct gapc_bond_
{
    /// CSRK values (Local and remote)
    gap_sec_key_t         csrk[GAPC_INFO_SRC_MAX];

    /// signature counter values (Local and remote)
    uint32_t              sign_counter[GAPC_INFO_SRC_MAX];

    #if (BLE_GATT_CLI)
    /// GATT Service Start handle
    uint16_t              gatt_start_hdl;
    /// Number of attributes present in GATT service
    uint8_t               gatt_nb_att;
    /// Service Change value handle offset
    uint8_t               svc_chg_offset;
    /// Server supported feature bit field (see enum #gapc_srv_feat)
    uint8_t               srv_feat;
    #endif // (BLE_GATT_CLI)

    /// Client supported feature bit field (see enum #gapc_cli_feat)
    uint8_t               cli_feat;

    /// Bond data information fields (see enum #gapc_bond_info_bf)
    uint16_t              info_bf;
} gapc_bond_t;


/// GAP connection variable structure.
typedef struct gapc_con
{
    /// Procedure queue
    hl_proc_queue_t  proc_queue;
    /// Bond data for the connections
    gapc_bond_t      bond;
    /// Upper layer software dummy parameter
    uint32_t         dummy;
    /// connection handle
    uint16_t         conhdl;
    /// Allocated connection index
    uint8_t          conidx;
    /// Connection information fields (see enum #gapc_con_info_bf)
    uint8_t          info_bf;
} gapc_con_t;

/// GAPM Service environment
typedef struct gapc_svc
{
    /// Pointer to the buffer used for service changed indication
    co_buf_t* p_svc_chg_ind_buf;
    /// Token identifier of Service change indication
    uint16_t  svc_chg_ind_token;
    /// GAP service start handle
    uint16_t  gap_start_hdl;
    /// GATT service start handle
    uint16_t  gatt_start_hdl;
    /// Bit field that contains service change event registration bit field (1 bit per connection)
    uint8_t   svc_chg_ccc_bf[GAPC_CON_ARRAY_BF_SIZE];
    /// Bit field that contains client aware status (1 bit per connection)
    uint8_t   cli_chg_aware_bf[GAPC_CON_ARRAY_BF_SIZE];
    /// Bit field that contains client attribute request authorization(1 bit per connection)
    uint8_t   cli_att_req_allowed_bf[GAPC_CON_ARRAY_BF_SIZE];
    /// GATT user local identifier for service
    uint8_t   user_lid;
} gapc_svc_t;


typedef struct gapc_sec_proc_hdr gapc_sec_proc_hdr_t;

/**
 ****************************************************************************************
 * @brief Handler of a procedure transition event.
 *
 * @param[in]  p_con          Pointer to connection environment
 * @param[in]  p_proc         Pointer to Security procedure
 * @param[in]  event          Procedure event transition
 * @param[out] p_is_finished  True if procedure is finished, false otherwise
 *
 * @return Transition execution status (see enum #hl_err).
 *         Procedure automatically terminated if status != GAP_ERR_NO_ERROR.
 *
 ****************************************************************************************
 */
typedef uint16_t (*gapc_sec_proc_transition_cb)(gapc_sec_proc_hdr_t* p_proc, uint8_t event, bool* p_is_finished);

/// Security procedure header
typedef struct gapc_sec_proc_hdr
{
    /// Procedure transition callback
    gapc_sec_proc_transition_cb transition_cb;
    /// transaction timer
    gapc_sdt_t                  trans_timer;
    /// Upper layer software dummy parameter
    uint32_t                    dummy;
    /// Pairing keys used to exchange with peer device
    gapc_pairing_keys_t         pairing_keys;
    /// Procedure connection index
    uint8_t                     conidx;
} gapc_sec_proc_hdr_t;

/// GAP controller environment variable structure.
typedef struct gapc_env_
{
    /// Pointer to connection environment
    gapc_con_t*                      p_con[HOST_CONNECTION_MAX];

    /// Pairing Information - This structure is allocated at the beginning of a pairing
    /// or procedure. It is freed when a disconnection occurs or at the end of
    /// the pairing procedure. If not enough memory can be found, the procedure will fail
    ///  with an "Unspecified Reason" error
    gapc_sec_proc_hdr_t*             p_sec_proc;

    /// Connection request event callback functions provided by upper layer software
    const gapc_connection_req_cb_t*  p_con_req_cbs;
    /// Connection event callback functions provided by upper layer software
    const gapc_connection_info_cb_t* p_info_cbs;
    /// Security event callback functions provided by upper layer software
    /// Mandatory if @see GAP_ROLE_CENTRAL or @see GAP_ROLE_PERIPHERAL or  @see GAP_ROLE_BT_CLASSIC is supported
    const gapc_security_cb_t*        p_sec_cbs;

    #if(BLE_GAPC)
    /// LE Connection configuration event callback functions provided by upper layer software
    const gapc_le_config_cb_t*       p_le_config_cbs;
    #if (BLE_PWR_CTRL)
    /// LE Power event callbacks
    const gapc_le_power_cb_t*        p_le_power_cbs;
    #endif // (BLE_PWR_CTRL)
    /// List of client to notify when a LE event is triggered (see #gapc_le_event_cb_t)
    co_list_t                        le_event_clients;
    #if (BLE_AOA || BLE_AOD)
    /// Constant tone extension callbacks
    const  gapc_le_cte_cb_t*         p_le_cte_cbs;
    #endif // (BLE_AOA || BLE_AOD)
    #endif // (BLE_GAPC)
    #if  (BT_HOST_PRESENT)
    #endif // (BT_HOST_PRESENT)
    /// GAP / GATT service environment
    gapc_svc_t                       svc;
    /// Main SDT delayed job. - TODO remove SDT
    co_djob_t                        sdt_job;
    /// Defer Job queue
    co_list_t                        sdt_job_queue;
    /// Main SDT timer
    co_time_timer_t                  sdt_timer;
    /// Timer queue
    co_list_t                        sdt_timer_queue;
    /// Last timer duration (used to speed up timer insertion)
    uint16_t                         sdt_max_push_duration;
    /// Timer and defer bit field
    uint8_t                          sdt_state_bf;
    #if (BLE_GATT_CLI)
    /// GAP / GATT client environment
    uint8_t                          cli_user_lid;
    #endif // (BLE_GATT_CLI)
    /// Client and server configuration - (see enum #gapc_cfg_bf)
    uint8_t                          cfg_flags;
} gapc_env_t;

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */
extern gapc_env_t gapc_env;


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve connection pointer from connection index
 *
 * @param[in] conidx Connection index
 *
 * @return Pointer to connection object, NULL if not found
 ****************************************************************************************
 */
__INLINE gapc_con_t* gapc_get_con_env(uint8_t conidx)
{
    return ((conidx < HOST_CONNECTION_MAX) ? gapc_env.p_con[conidx] : NULL);
}


/**
 ****************************************************************************************
 * @brief Upper layer SW confirmation of link creation with bond data if available.
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] p_data    Pointer to bond data if present, NULL otherwise
 *
 * @return Return function execution status (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapc_connection_cfm(uint8_t conidx, uint32_t dummy, const gapc_bond_data_t* p_data);

/**
 ****************************************************************************************
 * @brief Find a Connection index for BLE Connection Creation
 *
 * @return Connection index Available, GAP_INVALID_CONIDX if nothing found
 ****************************************************************************************
 */
uint8_t gapc_avail_conidx_find(void);


/**
 ****************************************************************************************
 * @brief Update link security level
 *
 * @param[in] p_con             Pointer to connection environment
 * @param[in] link_encrypted    Use to know if link is encrypted
 * @param[in] pairing_lvl       Link pairing level
 * @param[in] enc_key_present   Link paired and Link key or LTK has generated
 *
 ****************************************************************************************
 */
void gapc_sec_lvl_set(gapc_con_t* p_con, bool link_encrypted, uint8_t pairing_lvl, bool enc_key_present);

/**
 ****************************************************************************************
 * @brief Retrieve link security level from pairing level
 *
 * @param[in] pairing_lvl        Link pairing level
 * @param[in] is_able_to_encrypt True if able to encrypt link, false otherwise
 *
 * @return Link security level
 ****************************************************************************************
 */
uint8_t gapc_get_sec_lvl_from_pairing_lvl(uint8_t pairing_lvl, bool is_able_to_encrypt);

/**
 ****************************************************************************************
 * @brief Retrieve pairing level from link security level
 *
 * @param[in] sec_lvl        Link security level
 * @param[in] is_bonded      True if link is bonded, False otherwise
 *
 * @return Link pairing level
 ****************************************************************************************
 */
uint8_t gapc_get_pairing_lvl_from_sec_lvl(uint8_t sec_lvl, bool is_bonded);

/**
 ****************************************************************************************
 * @brief Retrieve link pairing level
 *
 * @param[in] conidx Connection index
 * @return Link pairing level
 ****************************************************************************************
 */
uint8_t gapc_get_pairing_level(uint8_t conidx);

#if (BLE_GATT)
/**
 ****************************************************************************************
 * @brief Inform Application about update of bond information
 *
 * @param[in] conidx        Connection Index
 ****************************************************************************************
 */
void gapc_bond_info_send(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief inform GAP/GATT service that a connection has been established
 *
 * @param[in] conidx             Connection index
 * @param[in] bond_data_present  True if bond data present, False else
 * @param[in] cli_info           Client bond data information (see enum #gapc_cli_info)
 * @param[in] cli_feat           Client supported features    (see enum #gapc_cli_feat)
 ****************************************************************************************
 */
void gapc_svc_con_create(uint8_t conidx, bool bond_data_present, uint8_t cli_info, uint8_t cli_feat);

#if (BLE_GATT_CLI)

/**
 ****************************************************************************************
 * @brief Inform GAP/GATT client that a connection has been established
 *
 * @param[in] conidx             Connection index
 * @param[in] bond_data_present  True if bond data present, False else
 * @param[in] gatt_start_hdl     GATT service start handle
 * @param[in] gatt_end_hdl       GATT Service end handle
 * @param[in] svc_chg_hdl        GATT Service changed value handle
 * @param[in] srv_feat           Server supported features (see enum #gapc_srv_feat)
 ****************************************************************************************
 */
void gapc_cli_con_create(uint8_t conidx, bool bond_data_present, uint16_t gatt_start_hdl, uint16_t gatt_end_hdl,
                        uint16_t svc_chg_hdl, uint8_t srv_feat);

/**
 ****************************************************************************************
 * @brief Function called when link becomes encrypted.
 *
 * @param[in] conidx    Connection index
 ****************************************************************************************
 */
void gapc_cli_link_encrypted(uint8_t conidx);
#endif // (BLE_GATT_CLI)
#endif // (BLE_GATT)


/**
 ****************************************************************************************
 * @brief Retrieve Bond Data information
 *
 * @param[in]  conidx     Connection index
 * @param[out] p_cli_info Pointer to client information (see enum #gapc_cli_info)
 ****************************************************************************************
 */
void gapc_svc_bond_data_get(uint8_t conidx, uint8_t* p_cli_info);

/**
 ****************************************************************************************
 * @brief Retrieve connection address information on current link.
 *
 * @param[in] conidx Connection index
 * @param[in] src    Connection information source
 *
 * @return Return found connection address
 ****************************************************************************************
 */
const gap_bdaddr_t* gapc_le_get_bdaddr(uint8_t conidx, uint8_t src);


/**
 ****************************************************************************************
 * @brief Initialize GAPC Simple Timer and Defer module
 *
 * @param[in] init_type  Type of initialization (see enum #rwip_init_type)
 *
 ****************************************************************************************
 */
void gapc_sdt_init(uint8_t init_type);

#if (HOST_MSG_API)
/// Create GAP Controller task
void gapc_msg_api_handler_create(void);
#endif // (HOST_MSG_API)

#endif // (GAPC_PRESENT)

/// @} GAPC_INT

#endif /* _GAPC_INT_H_ */
