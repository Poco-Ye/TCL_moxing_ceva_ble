/**
 ****************************************************************************************
 *
 * @file app.h
 *
 * @brief Application entry point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

#ifndef APP_H_
#define APP_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief Application entry point.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration
#include "rwapp_config.h"
#if (APP_PRESENT)

#include <stdint.h>          // Standard Integer Definition
#include "arch.h"            // Platform Definitions
#include <co_bt.h>           // Common BT Definitions

#include "gapc_msg.h"
#include "gapc_le_msg.h"   //add by liujin
#include "co_time.h"       


#if (NVDS_SUPPORT)
#include "nvds.h"
#endif // (NVDS_SUPPORT)

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximal length of the Device Name value
#define APP_DEVICE_NAME_MAX_LEN      (18)
#define APP_MESH_DEMO_TYPE_LEN        (1)

#define MAX_BONDED_DEV_NUM               (1) //(3)
#define MAX_BONDED_DEV_INDEX             (MAX_BONDED_DEV_NUM - 1)
#define INVALID_BONDED_INDEX             (-1)
#define APP_GAP_KEY_LEN                  (0x10)
#define APP_GAP_RAND_NB_LEN              (0x08)
#define APP_BD_ADDR_LEN                  (6)



/*
 * MACROS
 ****************************************************************************************
 */

#define APP_HANDLERS(subtask)    {&subtask##_msg_handler_list[0], ARRAY_LEN(subtask##_msg_handler_list)}

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Long Term Key information
typedef struct app_gap_ltk
{
    /// Long Term Key
    uint8_t ltk[APP_GAP_KEY_LEN];
    /// Encryption Diversifier
    uint16_t ediv;
    /// Random Number
    uint8_t randnb[APP_GAP_RAND_NB_LEN];
}app_ms_gap_ltk_t;

/// Short Term Key information
typedef struct app_ms_gap_irk
{
    /// Short Term Key
    uint8_t irk[APP_GAP_KEY_LEN];
    /// irk addr
    uint8_t irk_addr[APP_BD_ADDR_LEN];
}app_ms_gap_irk_t;

typedef struct
{
    uint8_t              peer_addr[APP_BD_ADDR_LEN];
    app_ms_gap_ltk_t ltk;
    uint8_t              ltk_in[APP_GAP_KEY_LEN];
    app_ms_gap_irk_t irk;
    uint8_t              periph_bond;
}bonded_dev_info_t;

typedef struct bonded_dev_info_list
{
    uint8_t           total_dev;
    uint8_t           current_dev_index;
    bonded_dev_info_t bonded_device_info[MAX_BONDED_DEV_NUM];
}bonded_dev_info_list_t;



#if (NVDS_SUPPORT)
/// List of Application NVDS TAG identifiers
enum app_nvds_tag
{
    /// Device Name
    NVDS_TAG_DEVICE_NAME                = 0x02,     //see  PARAM_ID,same with PARAM_ID_DEVICE_NAME
    NVDS_LEN_DEVICE_NAME                = 248,

    /// BD Address
    NVDS_TAG_BD_ADDRESS                 = 0x01,      //see  PARAM_ID,same with PARAM_ID_BD_ADDRESS
    NVDS_LEN_BD_ADDRESS                 = 6,

    /// Local device Identity resolving key
    NVDS_TAG_LOC_IRK                    = PARAM_ID_APP_SPECIFIC_FIRST+1,
    NVDS_LEN_LOC_IRK                    = KEY_LEN,


    #if (BLE_APP_PRF)
    /// BLE Application Advertising data
    NVDS_TAG_APP_BLE_ADV_DATA           = PARAM_ID_APP_SPECIFIC_FIRST+2,
    NVDS_LEN_APP_BLE_ADV_DATA           = 32,

    /// BLE Application Scan response data
    NVDS_TAG_APP_BLE_SCAN_RESP_DATA     = PARAM_ID_APP_SPECIFIC_FIRST+3,
    NVDS_LEN_APP_BLE_SCAN_RESP_DATA     = 32,


    /// Peripheral Bonded
    NVDS_TAG_PERIPH_BONDED              = PARAM_ID_APP_SPECIFIC_FIRST+4,
    NVDS_LEN_PERIPH_BONDED              = 1,


    /// Peer Device BD Address
    NVDS_TAG_PEER_BD_ADDRESS            = PARAM_ID_APP_SPECIFIC_FIRST+5,
    NVDS_LEN_PEER_BD_ADDRESS            = 7,


    /// EDIV (2bytes), RAND NB (8bytes),  LTK (16 bytes), Key Size (1 byte)
    NVDS_TAG_LTK                        = PARAM_ID_APP_SPECIFIC_FIRST+6,
    NVDS_LEN_LTK                        = 28,

    /// PAIRING
    NVDS_TAG_PAIRING                    = PARAM_ID_APP_SPECIFIC_FIRST+7,
    NVDS_LEN_PAIRING                    = 54,

    /// Peer device Resolving identity key (+identity address)
    NVDS_TAG_PEER_IRK                   = PARAM_ID_APP_SPECIFIC_FIRST+8,
    NVDS_LEN_PEER_IRK                   = sizeof(gapc_irk_t),

	 NVDS_TAG_BONDED_DEV_INFO           = PARAM_ID_APP_SPECIFIC_FIRST+9,
	 NVDS_LEN_BONDED_DEV_INFO           = sizeof(bonded_dev_info_list_t),
	 
    /*add new nvds tag below              */



	 NVDS_TAG_END                        = PARAM_ID_APP_SPECIFIC_LAST,
    
    //TODO
#endif //(BLE_APP_PRF)
};
#endif // (NVDS_SUPPORT)


/// Advertising state machine
enum app_adv_state
{
    /// Advertising activity does not exists
    APP_ADV_STATE_IDLE = 0,
    #if BLE_APP_PRF
    /// Creating advertising activity
    APP_ADV_STATE_CREATING,
    /// Setting advertising data
    APP_ADV_STATE_SETTING_ADV_DATA,
    /// Setting scan response data
    APP_ADV_STATE_SETTING_SCAN_RSP_DATA,

    /// Advertising activity created
    APP_ADV_STATE_CREATED,
    /// Starting advertising activity
    APP_ADV_STATE_STARTING,
    /// Advertising activity started
    APP_ADV_STATE_STARTED,
    /// Stopping advertising activity
    APP_ADV_STATE_STOPPING,
    #endif //(BLE_APP_PRF)
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure containing information about the handlers for an application subtask
struct app_subtask_handlers
{
    /// Pointer to the message handler table
    const struct ke_msg_handler *p_msg_handler_tab;
    /// Number of messages handled
    uint16_t msg_cnt;
};

/// Application environment structure
struct app_env_tag
{
    /// Connection handle
    uint16_t conhdl;
    /// Connection Index
    uint8_t  conidx;

    /// Advertising activity index
    uint8_t adv_actv_idx;
    /// Current advertising state (@see enum app_adv_state)
    uint8_t adv_state;
    /// Next expected operation completed event
    uint8_t adv_op;

    /// Last initialized profile
    uint8_t next_svc;

    /// Bonding status
    bool bonded;

    /// Device Name length
    uint8_t dev_name_len;
    /// Device Name
    uint8_t dev_name[APP_DEVICE_NAME_MAX_LEN];

    /// Local device IRK
    uint8_t loc_irk[KEY_LEN];

    /// Secure Connections on current link
    bool sec_con_enabled;

    /// Counter used to generate IRK
    uint8_t rand_cnt;

    /// Demonstration type length
    uint8_t demo_type_len;
    /// Demonstration type
    uint8_t demo_type;
	 //add by liujin for app update conn param
	 co_time_timer_t app_timer;
	 co_time_timer_t test_timer; //for app test
	 co_time_timer_t batt_timer;
	 co_time_timer_t IR_repeat_timer;
	 co_time_timer_t app_pair_timer;
	 co_time_timer_t app_recon_timer;
	 co_time_timer_t app_adv_timer;
};

/*
 * GLOBAL VARIABLE DECLARATION
 ****************************************************************************************
 */

/// Application environment
extern struct app_env_tag app_env;

extern uint8_t peer_dbaddr[APP_BD_ADDR_LEN];
extern bonded_dev_info_list_t bonded_dev_info;

extern uint8_t last_adv_type;
extern uint8_t direct_adv_idx;
extern uint8_t normal_adv_idx;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize the BLE demo application.
 ****************************************************************************************
 */
void app_init(void);



int8_t app_check_device_isbonded(const uint16_t in_ediv,const uint8_t *in_nb);

/**
 ****************************************************************************************
 * @brief Retrieve device name
 *
 * @param[out] Pointer at which device name will be returned
 *
 * @return Name length
 ****************************************************************************************
 */
uint8_t app_get_dev_name(uint8_t* p_name);

#if (BLE_APP_PRF)
/**
 ****************************************************************************************
 * @brief
 ****************************************************************************************
 */
void app_adv_fsm_next(void);

/**
 ****************************************************************************************
 * @brief Send to request to update the connection parameters
 ****************************************************************************************
 */
void app_update_param(struct gapc_conn_param *conn_param);

/**
 ****************************************************************************************
 * @brief Send a disconnection request
 ****************************************************************************************
 */
void app_disconnect(void);

/**
 ****************************************************************************************
 * @brief Start/stop advertising
 *
 * @param[in] start     True if advertising has to be started, else false
 ****************************************************************************************
 */

void app_update_adv_state(bool start);

/**
 ****************************************************************************************
 * @brief delete advertising
 *
 * @param[in] none
 ****************************************************************************************
 */

void app_delete_advertising(void);
/**
 ****************************************************************************************
 * @brief Return if the device is currently bonded
 ****************************************************************************************
 */
bool app_sec_get_bond_status(void);

void app_stop_advertising(uint8_t adv_idx);

void app_stop_pair_timer(void);

void app_start_pair(void);


void app_ir_send_code_and_repert(uint8_t code, uint8_t times);

void app_reconnect(void);

void app_start_reconnet_handler(void* p_env);


/// @} APP
///
#if (NVDS_SUPPORT)	

bool app_nv_set_bonded_device_info(bonded_dev_info_list_t *dev_info);

bool app_nv_get_bonded_device_info(bonded_dev_info_list_t *dev_info);

#endif
#endif //(BLE_APP_PRF)
#endif //(APP_PRESENT)

#endif // APP_H_
