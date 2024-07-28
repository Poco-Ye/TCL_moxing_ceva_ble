/**
 ****************************************************************************************
 *
 * @file gapm_adv.c
 *
 * @brief Generic Access Profile Manager - Advertising manager module.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_ADV Generic Access Profile Manager - Advertising manager module.
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "gapm_le_adv.h"
#if (HL_LE_BROADCASTER)
#include "gapm_int.h"

#include <string.h>

#include "gap.h"
#include "../gap_int.h"
#include "gapc.h"
#include "ke_mem.h"
#include "hl_hci.h"
#include "hci.h"
#include "co_utils.h"
#include "co_math.h"
#include "aes.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Mask allowing to map prop field of GAPM_ACTIVITY_CREATE_CMD message structure on
/// adv_evt_properties field of LE Set Extended Advertising Parameter Command HCI
/// message structure.
#define GAPM_ADV_EVT_PROP_MASK                  (0x007F)

/// Bit field for Periodic Advertising Properties field (LE Set Periodic Advertising Parameters command)
#define GAPM_ADV_PERIOD_PROP_TX_POWER_BIT       (6)

/// Maximum advertising data fragment length that can be provided to the controller through HCI
#define GAPM_ADV_FRGMT_ADV_DATA_LENGTH          (251)
/// Maximum advertising data fragment length that can be provided to the controller through HCI
#define GAPM_ADV_FRGMT_SCAN_RSP_DATA_LENGTH     (251)
/// Maximum advertising data fragment length that can be provided to the controller through HCI
#define GAPM_ADV_FRGMT_PER_ADV_DATA_LENGTH      (252)

/// Length of AD Type flags
#define GAPM_ADV_AD_TYPE_FLAGS_LENGTH           (3)
/// Length of AD Type flags information
#define GAPM_ADV_AD_TYPE_FLAGS_INFO_LENGTH      (2)

#define GAPM_ADV_TOKEN_CREATE(actv_idx, event)   (((actv_idx & 0xFF) << 0) | ((event & 0xFF) << 8))
#define GAPM_ADV_ACTV_IDX_GET(token)  ((uint8_t)((token >> 0) & 0xFF))
#define GAPM_ADV_EVENT_GET(token)     ((uint8_t)((token >> 8) & 0xFF))

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Available value for scan_req_notif_en field in LE Set Extended Advertising Parameters HCI command
enum gapm_adv_scan_req_notif_en
{
    /// Scan request notifications disabled
    GAPM_ADV_SCAN_REQ_NOTIF_DIS = 0,
    /// Scan request notification enabled
    GAPM_ADV_SCAN_REQ_NOTIF_EN,
};

/// Bit field bit positions for advertising activity info parameter
enum gapm_adv_info_bf
{
    /// Indicate that scan response data can be set (meaning advertising is a scannable advertising)
    GAPM_ADV_INFO_ACPT_SCAN_RSP_DATA_POS = 0,
    GAPM_ADV_INFO_ACPT_SCAN_RSP_DATA_BIT = CO_BIT(GAPM_ADV_INFO_ACPT_SCAN_RSP_DATA_POS),

    /// Indicate that advertising data can be set
    ///     Use of advertising data is excluded in following cases:
    ///         - Legacy directed advertising
    ///         - Scannable extended advertising (undirected or directed)
    GAPM_ADV_INFO_ACPT_ADV_DATA_POS      = 1,
    GAPM_ADV_INFO_ACPT_ADV_DATA_BIT      = CO_BIT(GAPM_ADV_INFO_ACPT_ADV_DATA_POS),

    /// Indicate that advertising data has been set at least one time for the activity
    GAPM_ADV_INFO_ADV_DATA_SET_POS       = 2,
    GAPM_ADV_INFO_ADV_DATA_SET_BIT       = CO_BIT(GAPM_ADV_INFO_ADV_DATA_SET_POS),

    /// Indicate that scan response data has been set (mandatory only for extended advertising)
    GAPM_ADV_INFO_SCAN_RSP_DATA_SET_POS  = 3,
    GAPM_ADV_INFO_SCAN_RSP_DATA_SET_BIT  = CO_BIT(GAPM_ADV_INFO_SCAN_RSP_DATA_SET_POS),

    /// Indicate that periodic advertising data has been set (mandatory only for periodic advertising)
    GAPM_ADV_INFO_PER_ADV_DATA_SET_POS   = 4,
    GAPM_ADV_INFO_PER_ADV_DATA_SET_BIT   = CO_BIT(GAPM_ADV_INFO_PER_ADV_DATA_SET_POS),

    /// Indicate that notification of Scan Requests has been required by application
    GAPM_ADV_INFO_SCAN_REQ_NTF_POS       = 5,
    GAPM_ADV_INFO_SCAN_REQ_NTF_BIT       = CO_BIT(GAPM_ADV_INFO_SCAN_REQ_NTF_POS),

    /// Indicate that advertising is running
    GAPM_ADV_INFO_ADV_EN_POS             = 6,
    GAPM_ADV_INFO_ADV_EN_BIT             = CO_BIT(GAPM_ADV_INFO_ADV_EN_POS),

    /// Indicate that periodic advertising is running
    GAPM_ADV_INFO_PER_ADV_EN_POS         = 7,
    GAPM_ADV_INFO_PER_ADV_EN_BIT         = CO_BIT(GAPM_ADV_INFO_PER_ADV_EN_POS),
};

/// Bit field bit positions for advertising activity additional info parameter
enum gapm_adv_add_info_bf
{
    /// Indicate that advertising is a connectable advertising
    GAPM_ADV_ADD_INFO_CONNECTABLE_POS = 0,
    GAPM_ADV_ADD_INFO_CONNECTABLE_BIT = CO_BIT(GAPM_ADV_ADD_INFO_CONNECTABLE_POS),
};


/// Advertising activity create procedure states
enum gapm_adv_create_proc_event
{
    /// Received HCI_LE_SET_EXT_ADV_PARAM_CMD complete event
    GAPM_ADV_CREATE_HCI_SET_ADV_PARAM_CMP = HL_PROC_EVENT_FIRST,
    /// Received HCI_LE_SET_PER_ADV_PARAM_CMD complete event
    GAPM_ADV_CREATE_HCI_SET_PER_ADV_PARAM_CMP,

    #if (BLE_AOD | BLE_AOA)
    /// Received HCI_LE_SET_CONLESS_CTE_TX_PARAM_CMD complete event
    GAPM_ADV_CREATE_HCI_SET_CONLESS_CTE_TX_PARAM_CMP,
    #endif // (BLE_AOD | BLE_AOA)
    /// Result of Address generation procedure received
    GAPM_ADV_CREATE_GEN_RAND_ADDR_CMP,
    /// Received HCI_LE_RMV_ADV_SET_CMD complete event
    GAPM_ADV_CREATE_HCI_REMOVE_SET_CMP,
};


/// Advertising activity start procedure states
enum gapm_adv_start_proc_event
{
    /// Received HCI_LE_SET_ADV_SET_RAND_ADDR_CMD complete event
    GAPM_ADV_START_HCI_SET_ADDR_CMP = HL_PROC_EVENT_FIRST,
    /// Received HCI_LE_SET_EXT_ADV_EN_CMD complete event
    GAPM_ADV_START_HCI_ADV_EN_CMP,
    /// Received HCI_LE_SET_PER_ADV_EN_CMD complete event
    GAPM_ADV_START_HCI_PER_ADV_EN_CMP,
    /// Received HCI_LE_SET_EXT_ADV_EN_CMD complete event (disable adv)
    GAPM_ADV_START_HCI_ADV_DIS_CMP,
};

/// Advertising activity stop procedure states
enum gapm_adv_stop_proc_event
{
    /// Received HCI_LE_SET_EXT_ADV_EN_CMD complete event (disable adv)
    GAPM_ADV_STOP_HCI_ADV_DIS_CMP = HL_PROC_EVENT_FIRST,
    /// Received HCI_LE_SET_PER_ADV_EN_CMD complete event (disable per_adv)
    GAPM_ADV_STOP_HCI_PER_ADV_DIS_CMP,
};

/// Advertising activity delete procedure states
enum gapm_adv_delete_proc_event
{
    /// Received HCI_LE_RMV_ADV_SET_CMD complete event
    GAPM_ADV_DELETE_HCI_ADV_RMV_CMP = HL_PROC_EVENT_FIRST,
};

/// Advertising activity address renew procedure states
enum gapm_adv_addr_renew_proc_event
{
    /// Result of Address generation procedure received
    GAPM_ADV_RENEW_GEN_RAND_ADDR_CMP = HL_PROC_EVENT_FIRST,
    /// Received HCI_LE_SET_ADV_SET_RAND_ADDR_CMD complete event
    GAPM_ADV_RENEW_HCI_SET_ADDR_CMP,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// GAP Manager activity structure for advertising activity
typedef struct gapm_adv_actv
{
    /// Activity inherited parameters
    gapm_actv_t hdr;
    /// Advertising mode (see enum #gapm_adv_disc_mode)
    uint8_t     mode;
    /// Additional information (see enum #gapm_adv_add_info_bf)
    uint8_t     add_info;
    /// Information bit field, meaning depends on activity type
    uint8_t     info_bf;
    /// Own address type
    uint8_t     own_addr_type;
    /// BD Address used by the activity (can be different if controller privacy is used and
    /// application chose to use a resolvable private address)
    gap_addr_t  addr;
    /// Callback function allowing to inform GAF block that BD Address is about to be renewed for an advertising
    /// activity\n
    /// !!!! FOR INTERNAL USE ONLY !!!!
    gapm_adv_cb_addr_renew cb_addr_renew;
} gapm_adv_actv_t;

/// Legacy Advertising activity create procedure structure
typedef struct gapm_adv_proc_create
{
    /// Inherited from default procedure object
    gapm_actv_proc_t        hdr;
    /// Selected TX power
    int8_t                  tx_pwr;
    /// Error status kept to be reused after an ADV set removal
    uint16_t                kept_status;
    /// Advertising parameters (optional, shall be present only if operation is GAPM_CREATE_ADV_ACTIVITY)
    gapm_adv_create_param_t param;
} gapm_adv_proc_create_t;


/// Extended Advertising activity create procedure structure
typedef struct gapm_adv_ext_proc_create
{
    /// Inherited from Legacy advertising
    gapm_adv_proc_create_t  hdr;
    /// Configuration for secondary advertising
    gapm_adv_second_cfg_t   second_cfg;
} gapm_adv_ext_proc_create_t;


/// Periodic Advertising activity create procedure structure
typedef struct gapm_adv_periodic_proc_create
{
    /// Inherited from Legacy advertising
    gapm_adv_ext_proc_create_t  hdr;
    /// Configuration for periodic advertising
    gapm_adv_period_cfg_t       period_cfg;
    /// Constant tone extention parameters
    gapm_adv_cte_cfg_t          cte_cfg;
    /// Length of switching pattern (number of antenna IDs in the pattern)
    uint8_t                     switching_pattern_len;
    /// Antenna IDs
    uint8_t                     antenna_id[__ARRAY_EMPTY];
} gapm_adv_periodic_proc_create_t;

/// Advertising activity start procedure structure
typedef struct gapm_adv_proc_start
{
    /// Inherited from default procedure object
    gapm_actv_proc_t hdr;
    /// Advertising start parameters
    gapm_adv_param_t param;
    /// Error status kept to be reused after an ADV set removal
    uint16_t         kept_status;
} gapm_adv_proc_start_t;

/// function prototype to send advertising data to controller
typedef uint16_t (*gapm_adv_hci_send_adv_data_func)(gapm_adv_actv_t* p_actv, co_buf_t* p_data, bool first_frag);

/// Advertising activity Set Data procedure structure
typedef struct gapm_adv_proc_data_set
{
    /// Inherited from default procedure object
    gapm_actv_proc_t                hdr;
    /// Pointer to buffer that contains advertising data
    co_buf_t*                       p_data;
    /// Function to call in order to send HCI adv data command
    gapm_adv_hci_send_adv_data_func send_data_func;
    /// Bit to set once the procedure succeed
    uint8_t                         data_set_bit;
} gapm_adv_proc_data_set_t;

#if(BLE_AOA || BLE_AOD)
/// Advertising activity Set Data procedure structure
typedef struct gapm_adv_proc_cte_tx_ctrl
{
    /// Inherited from default procedure object
    gapm_actv_proc_t  hdr;
    /// True to enable CTE transmission
    bool              enable;
} gapm_adv_proc_cte_tx_ctrl_t;
#endif // (BLE_AOA || BLE_AOD)

/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */
__STATIC uint16_t gapm_adv_proc_start_transition(gapm_adv_actv_t* p_actv, gapm_adv_proc_start_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_adv_stop_transition(gapm_adv_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_adv_delete_transition(gapm_adv_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_adv_create_transition(gapm_adv_actv_t *p_actv, gapm_adv_proc_create_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_adv_data_set_transition(gapm_adv_actv_t *p_actv, gapm_adv_proc_data_set_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_adv_addr_renew_transition(gapm_adv_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Activity object interface
__STATIC const gapm_actv_itf_t gapm_adv_actv_itf =
{
    .cb_clean                  = (gapm_actv_clean_cb)           gapm_actv_clean, // default clean
    .cb_start_transition       = (gapm_actv_proc_transition_cb) gapm_adv_proc_start_transition,
    .cb_stop_transition        = (gapm_actv_proc_transition_cb) gapm_adv_stop_transition,
    .cb_delete_transition      = (gapm_actv_proc_transition_cb) gapm_adv_delete_transition,
    .cb_addr_renew_transition  = (gapm_actv_proc_transition_cb) gapm_adv_addr_renew_transition,
};

/*
 * HCI HANDLERS DEFINITIONS
 ****************************************************************************************
 */

/// Default HCI Command complete handler used to continue procedure execution
__STATIC void gapm_adv_hci_cmd_cmp_handler(uint16_t opcode, uint16_t event, struct hci_basic_cmd_cmp_evt const *p_evt)
{
    gapm_proc_transition(GAPM_PROC_AIR, (uint8_t) event, RW_ERR_HCI_TO_HL(p_evt->status));
}

/// Handle Reception of HCI_LE_SET_EXT_ADV_PARAM_CMD complete event
__STATIC void gapm_hci_le_set_ext_adv_param_cmd_cmp_handler(uint16_t opcode, uint16_t event,
                                                            struct hci_le_set_ext_adv_param_cmd_cmp_evt const *p_evt)
{
    gapm_adv_proc_create_t* p_proc = (gapm_adv_proc_create_t*) gapm_proc_get(GAPM_PROC_AIR);
    p_proc->tx_pwr = p_evt->sel_tx_pwr;
    gapm_adv_hci_cmd_cmp_handler(opcode, event, (struct hci_basic_cmd_cmp_evt const *)p_evt);
}

#if (HL_LE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Handles enhanced connection complete event from the lower layer.
 * This handler is responsible for initiating the creation of L2CAP channel.
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void gapm_adv_hci_le_enh_con_cmp_evt_handler(uint8_t evt_code, struct hci_le_enh_con_cmp_evt const *p_evt)
{
    if(p_evt->status == CO_ERROR_NO_ERROR)
    {
        co_buf_t* p_buf = NULL;

        // use a buffer to keep connection information for advertising activity
        if(co_buf_alloc(&p_buf, 0, 0, 0) != CO_BUF_ERR_NO_ERROR)
        {
            // Automatically send a disconnect
            gapm_le_actv_send_hci_disconnect(p_evt->conhdl, CO_ERROR_MEMORY_CAPA_EXCEED);
        }
        else
        {
            struct hci_le_enh_con_cmp_evt* p_meta;
            p_meta = (struct hci_le_enh_con_cmp_evt*) co_buf_metadata(p_buf);
            *p_meta = *p_evt;
            gapm_env.p_adv_connection_info = p_buf;
        }
    }
}
#endif // (HL_LE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Advertising Set Terminated Event.
 * This event indicates that the controller has terminated advertising in the advertising
 * set specified by the advertising handle parameter.
 * The event is generated in following cases:
 *    - Connection has been created (in that case connection handle of new connection is
 *    provided)
 *    - Advertising duration elapsed (status parameter code set to Advertising Timeout (0x3C)
 *    - Max extended advertising events value has been reached (status parameter set to
 *    Limit Reached (0x43)
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void gapm_hci_le_adv_set_term_evt_handler(uint8_t evt_code, struct hci_le_adv_set_term_evt const *p_evt)
{
    // Advertising activity structure
    gapm_adv_actv_t *p_actv;
    // Reason
    uint8_t reason = GAP_ERR_NO_ERROR;

    do
    {
        p_actv = (gapm_adv_actv_t *) gapm_actv_get(p_evt->adv_hdl);

        // Check if event can be expected
        if (!p_actv
                || (p_actv->hdr.type != GAPM_ACTV_TYPE_ADV)
                || (p_actv->hdr.state != GAPM_ACTV_STARTED))
        {
            // Drop the event
            break;
        }

        // Keep in mind that advertising is over
        SETB(p_actv->info_bf, GAPM_ADV_INFO_ADV_EN, false);

        switch (p_evt->status)
        {
            #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
            case CO_ERROR_NO_ERROR:
            {
                if (GETB(p_actv->add_info, GAPM_ADV_ADD_INFO_CONNECTABLE))
                {
                    uint8_t conidx;
                    uint16_t status;
                    struct hci_le_enh_con_cmp_evt* p_con_evt;
                    ASSERT_ERR(gapm_env.p_adv_connection_info != NULL);
                    p_con_evt = (struct hci_le_enh_con_cmp_evt*) co_buf_metadata(gapm_env.p_adv_connection_info);
                    status = gapm_le_actv_con_create(&(p_actv->hdr), false, p_con_evt, p_actv->own_addr_type, &p_actv->addr, &conidx);

                    if(status != GAP_ERR_NO_ERROR)
                    {
                        gapm_le_actv_send_hci_disconnect(p_evt->conhdl, CO_ERROR_MEMORY_CAPA_EXCEED);
                    }

                    // release connection info
                    co_buf_release(gapm_env.p_adv_connection_info);
                    gapm_env.p_adv_connection_info = NULL;
                }
            } break;
            #endif //(HL_LE_CENTRAL || HL_LE_PERIPHERAL)

            case CO_ERROR_ADV_TO:
            case CO_ERROR_LIMIT_REACHED:   { reason = GAP_ERR_TIMEOUT; } break;
            default:                       {/* Nothing to do */        } break;
        }

        if (GETB(p_actv->add_info, GAPM_ADV_ADD_INFO_CONNECTABLE))
        {
            // Sanity check
            ASSERT_INFO(gapm_env.nb_connect_actvs != 0, gapm_env.nb_connect_actvs, 0);

            // Decrease number of started activities that can lead to a connection
            gapm_env.nb_connect_actvs--;
        }

        // Special case: in case of periodic advertising, extended advertising can be stopped.
        if (p_actv->hdr.subtype == GAPM_ADV_TYPE_PERIODIC)
        {
            gapm_adv_actv_cb_t* p_cbs = (gapm_adv_actv_cb_t*) p_actv->hdr.p_cbs;
            p_cbs->ext_adv_stopped(p_actv->hdr.dummy, p_actv->hdr.idx, reason);
        }
        else
        {
            // Inform the application that activity has been stopped
            gapm_actv_stopped((gapm_actv_t *)p_actv, reason);
        }
    } while (0);
}

/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Scan Request Received event.
 * This event indicates that a SCAN_REQ PDU or an AUX_SCAN_REQ PDU has been received.
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void gapm_hci_le_scan_req_rcvd_evt_handler(uint8_t evt_code, struct hci_le_scan_req_rcvd_evt const *p_evt)
{
    do
    {
        gapm_adv_actv_cb_t* p_cbs;
        gapm_adv_actv_t *p_actv = (gapm_adv_actv_t *) gapm_actv_get(p_evt->adv_hdl);

        // Check if event can be expected
        if (!p_actv
                || (p_actv->hdr.type != GAPM_ACTV_TYPE_ADV)
                || (p_actv->hdr.state != GAPM_ACTV_STARTED)
                || !GETB(p_actv->info_bf, GAPM_ADV_INFO_SCAN_REQ_NTF))
        {
            // Drop the event
            break;
        }

        // Inform upper layer about scan request reception
        p_cbs = (gapm_adv_actv_cb_t*) p_actv->hdr.p_cbs;
        if(p_cbs->scan_req_received != NULL)
        {
            gap_bdaddr_t identity;

            identity.addr_type = p_evt->scan_addr_type;
            memcpy(&identity.addr[0], &p_evt->scan_addr.addr[0], BD_ADDR_LEN);

            p_cbs->scan_req_received(p_actv->hdr.dummy, p_actv->hdr.idx, &identity);
        }
    } while (0);
}

/**
 ****************************************************************************************
 * @brief Send a LE Set Advertising Set Random Address command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_adv_handler function.
 *
 * @param[in] p_actv_adv    Pointer to the activity structure
 * @param[in] event         Procedure transition event
 ****************************************************************************************
 */
__STATIC uint16_t gapm_adv_send_hci_le_set_adv_set_rand_addr_cmd(gapm_adv_actv_t *p_actv, uint8_t event)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    // Allocate HCI command message
    struct hci_le_set_adv_set_rand_addr_cmd *p_cmd =
            HL_HCI_CMD_ALLOC(HCI_LE_SET_ADV_SET_RAND_ADDR_CMD_OPCODE, hci_le_set_adv_set_rand_addr_cmd);

    if(p_cmd != NULL)
    {
        status = GAP_ERR_NO_ERROR;
        // Use local advertising identifier as advertising handle
        p_cmd->adv_hdl = p_actv->hdr.idx;
        // Copy the address
        memcpy(&p_cmd->rand_addr.addr[0], &(p_actv->addr), BD_ADDR_LEN);

        // Send the command
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, event, gapm_adv_hci_cmd_cmp_handler);
    }

    return (status);
}

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send a LE Remove Advertising Set command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_adv_handler function.
 *
 * @param[in] actv_idx    Activity Index
 * @param[in] event       Event triggered when command complete is received
 *
 * @return Status to know if command has been conveyed to controller (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC uint16_t gapm_adv_send_hci_le_rmv_adv_set_cmd(uint8_t actv_idx, uint16_t event)
{
    uint16_t status;
    struct hci_le_rem_adv_set_cmd *p_cmd = HL_HCI_CMD_ALLOC(HCI_LE_RMV_ADV_SET_CMD_OPCODE, hci_le_rem_adv_set_cmd);

    if(p_cmd == NULL)
    {
        status = GAP_ERR_INSUFF_RESOURCES;
    }
    else
    {
        // Use local advertising identifier as advertising handle
        p_cmd->adv_hdl = actv_idx;

        // Send the command
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, event, gapm_adv_hci_cmd_cmp_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}



/**
 ****************************************************************************************
 * @brief Send a LE Set Extended Advertising Enable command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_adv_handler function.
 *
 * @param[in] p_actv_adv    Pointer to the activity structure
 * @param[in] p_param       GAPM_ACTIVITY_START_CMD message parameters if advertising has
 * to be enabled, NULL pointer if has to be disabled
 * @param[in] event         Event type receive that induce procedure state transition
 ****************************************************************************************
 */
__STATIC uint16_t gapm_adv_send_hci_le_set_ext_adv_en_cmd(gapm_adv_actv_t *p_actv, const gapm_adv_param_t* p_param,
                                                          uint8_t event)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    // Allocate HCI command message
    struct hci_le_set_ext_adv_en_cmd *p_cmd =
            HL_HCI_CMD_ALLOC(HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE, hci_le_set_ext_adv_en_cmd);

    if(p_cmd != NULL)
    {
        status = GAP_ERR_NO_ERROR;

        // Enable or disable
        p_cmd->enable = (p_param) ? true : false;
        // Only one set is considered
        p_cmd->nb_sets = 1;
        // Use local advertising identifier as advertising handle
        p_cmd->sets[0].adv_hdl = p_actv->hdr.idx;

        if (p_param)
        {
            if (p_actv->mode == GAPM_ADV_MODE_LIM_DISC)
            {
                // Force duration for limited discovery mode
                p_cmd->sets[0].duration = co_min(p_param->duration, GAP_TMR_LIM_ADV_TIMEOUT);

                // 0 means no timeout, force a timeout value
                if(p_cmd->sets[0].duration == 0)
                {
                    p_cmd->sets[0].duration = GAP_TMR_LIM_ADV_TIMEOUT;
                }
            }
            else
            {
                p_cmd->sets[0].duration = p_param->duration;
            }
            p_cmd->sets[0].max_ext_adv_evt = p_param->max_adv_evt;
        }

        // Send the command
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, event, gapm_adv_hci_cmd_cmp_handler);
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Send a LE Set Periodic Advertising Enable command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_adv_handler function.
 *
 * @param[in] actv_idx      Activity local identifier
 * @param[in] enable        Indicate if periodic advertising must be enabled or disabled
 * @param[in] event         Event type receive that induce procedure state transition
 ****************************************************************************************
 */
__STATIC uint16_t gapm_adv_send_hci_le_set_per_adv_en_cmd(uint8_t actv_idx, bool enable, uint8_t event)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    // Allocate HCI command message
    struct hci_le_set_per_adv_en_cmd *p_cmd =
            HL_HCI_CMD_ALLOC(HCI_LE_SET_PER_ADV_EN_CMD_OPCODE, hci_le_set_per_adv_en_cmd);

    if(p_cmd == NULL)
    {
        status = GAP_ERR_INSUFF_RESOURCES;
    }
    else
    {
        // Enable or disable
        p_cmd->enable = (uint8_t)enable;
        // Use local advertising identifier as advertising handle
        p_cmd->adv_hdl = actv_idx;

        // Send the command
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, event, gapm_adv_hci_cmd_cmp_handler);
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Get Advertising data fragmentation operation
 *
 * @param[in]  max_frag_size    Maximum size of Advertising fragment
 * @param[in]  remain_data_len  Remaining ADV data length
 * @param[in]  first_frag       True if first fragment, False otherwise
 * @param[out] p_wr_len         Length of the fragment to create
 *
 * @return Fragmentation operation (see enum #adv_data_op)
 ****************************************************************************************
 */
uint8_t gapm_adv_data_set_operation_get(uint16_t max_frag_size, uint16_t remain_data_len, bool first_frag,
                                        uint16_t* p_wr_len)
{
    uint8_t set_operation;

    // Check if data must be fragmented (AD type flags have to be added)
    if (remain_data_len <= max_frag_size)
    {
        set_operation = first_frag ? ADV_DATA_OP_COMPLETE : ADV_DATA_OP_LAST_FRAG;
        *p_wr_len = remain_data_len;
    }
    else
    {
        set_operation = first_frag ? ADV_DATA_OP_FIRST_FRAG : ADV_DATA_OP_INTERMEDIATE_FRAG;
        *p_wr_len = max_frag_size;
    }

    return (set_operation);
}


/**
 ****************************************************************************************
 * @brief Generate AD Type additional data on advertising or scan response data
 *
 * @param[in]  p_actv           Pointer to advertising activity
 * @param[in]  p_data           Pointer to advertising data to fill
 *
 ****************************************************************************************
 */
void gapm_adv_set_ad_type_data(gapm_adv_actv_t *p_actv, uint8_t* p_data)
{
    // Length of ad type flags
    p_data[0] = GAPM_ADV_AD_TYPE_FLAGS_INFO_LENGTH;
    p_data[1] = GAP_AD_TYPE_FLAGS;
    p_data[2] = GAP_BR_EDR_NOT_SUPPORTED_BIT;

    // Set mode in ad_type
    switch (p_actv->mode)
    {
        // General discoverable mode
        case GAPM_ADV_MODE_GEN_DISC:
        {
            p_data[2] |= GAP_LE_GEN_DISCOVERABLE_FLG_BIT;
        }
        break;

        // Limited discoverable mode
        case GAPM_ADV_MODE_LIM_DISC:
        {
            p_data[2] |= GAP_LE_LIM_DISCOVERABLE_FLG_BIT;
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
    }
}


/**
 ****************************************************************************************
 * @brief Send a HCI LE Set Extended Advertising Data command to the controller.
 *
 * @param[in] p_data     Pointer to the advertising data buffer.
 * @param[in] fist_frag  True if first fragment transmission, false otherwise
 *
 * @return Status of the command transmission (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC uint16_t gapm_adv_send_hci_le_set_ext_adv_data_cmd(gapm_adv_actv_t *p_actv, co_buf_t* p_data, bool first_frag)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    uint16_t wr_length = co_buf_data_len(p_data);
    uint8_t set_operation;
    bool add_data_present = false;
    struct hci_le_set_ext_adv_data_cmd *p_cmd;

    // check if additional data
    if(first_frag && (p_actv->mode != GAPM_ADV_MODE_BEACON))
    {
        add_data_present = true;
        wr_length   += GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
    }

    // Get fragmentation rules
    set_operation = gapm_adv_data_set_operation_get(GAPM_ADV_FRGMT_ADV_DATA_LENGTH, wr_length, first_frag, &wr_length);

    p_cmd = HL_HCI_CMD_ALLOC_DYN(HCI_LE_SET_EXT_ADV_DATA_CMD_OPCODE, hci_le_set_ext_adv_data_cmd, wr_length);


    if(p_cmd)
    {
        uint8_t* p_adv_data = &(p_cmd->data[0]);
        status = GAP_ERR_NO_ERROR;
        // Use local advertising identifier as advertising handle
        p_cmd->adv_hdl = p_actv->hdr.idx;
        // Set operation, fragment preferences and length
        p_cmd->operation = set_operation;
        p_cmd->frag_pref = ADV_DATA_MAY_FRAG;
        p_cmd->data_len  = wr_length;

        // Set the AD type flags
        if (add_data_present)
        {
            gapm_adv_set_ad_type_data(p_actv, p_adv_data);
            p_adv_data += GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
            wr_length  -= GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
        }

        // Copy the data
        memcpy(p_adv_data, co_buf_data(p_data), wr_length);
        co_buf_head_release(p_data, wr_length);

        // Send the command
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, HL_PROC_EVENT_FIRST, gapm_adv_hci_cmd_cmp_handler);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Send a HCI LE Set External Scan Response Data command to the controller.
 * This function is in charge of splitting the scan response data provided by the
 * application in several fragment.
 *
 * @param[in] p_actv     Pointer to the advertising activity structure.
 * @param[in] p_data     Pointer to the advertising data buffer.
 * @param[in] first_frag True if first fragment transmission, false otherwise
 *
 * @return Status of the command transmission (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC uint16_t gapm_adv_send_hci_le_set_ext_scan_rsp_data_cmd(gapm_adv_actv_t *p_actv, co_buf_t* p_data, bool first_frag)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    uint16_t wr_length = co_buf_data_len(p_data);
    uint8_t set_operation;
    bool add_data_present = false;
    struct hci_le_set_ext_scan_rsp_data_cmd *p_cmd;

    // check if additional data
    if(first_frag && !GETB(p_actv->info_bf, GAPM_ADV_INFO_ACPT_ADV_DATA))
    {
        add_data_present = true;
        wr_length   += GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
    }

    // Get fragmentation rules
    set_operation = gapm_adv_data_set_operation_get(GAPM_ADV_FRGMT_SCAN_RSP_DATA_LENGTH, wr_length, first_frag, &wr_length);

    p_cmd = HL_HCI_CMD_ALLOC_DYN(HCI_LE_SET_EXT_SCAN_RSP_DATA_CMD_OPCODE, hci_le_set_ext_scan_rsp_data_cmd, wr_length);

    if(p_cmd)
    {
        uint8_t* p_adv_data = &(p_cmd->data[0]);
        status = GAP_ERR_NO_ERROR;
        // Use local advertising identifier as advertising handle
        p_cmd->adv_hdl = p_actv->hdr.idx;
        // Set operation, fragment preferences and length
        p_cmd->operation = set_operation;
        p_cmd->frag_pref = ADV_DATA_MAY_FRAG;
        p_cmd->data_len  = wr_length;

        // Set the AD type flags
        if (add_data_present)
        {
            gapm_adv_set_ad_type_data(p_actv, p_adv_data);
            p_adv_data += GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
            wr_length  -= GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
        }

        // Copy the data
        memcpy(p_adv_data, co_buf_data(p_data), wr_length);
        co_buf_head_release(p_data, wr_length);

        // Send the command
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, HL_PROC_EVENT_FIRST, gapm_adv_hci_cmd_cmp_handler);
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Send a HCI LE Set Periodic Advertising Data to the controller.
 *
 * @param[in] p_actv     Pointer to the activity structure for which the periodic advertising data has to be set.
 * @param[in] p_data     Pointer to the advertising data buffer.
 * @param[in] first_frag  True if first fragment transmission, false otherwise
 *
 * @return Status of the command transmission (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC uint16_t gapm_adv_send_hci_le_set_per_adv_data_cmd(gapm_adv_actv_t *p_actv, co_buf_t* p_data, bool first_frag)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    uint16_t wr_length = co_buf_data_len(p_data);
    uint8_t set_operation;
    struct hci_le_set_per_adv_data_cmd *p_cmd;

    // Get fragmentation rules
    set_operation = gapm_adv_data_set_operation_get(GAPM_ADV_FRGMT_PER_ADV_DATA_LENGTH, wr_length, first_frag, &wr_length);


    p_cmd = HL_HCI_CMD_ALLOC_DYN(HCI_LE_SET_PER_ADV_DATA_CMD_OPCODE, hci_le_set_per_adv_data_cmd, wr_length);

    if(p_cmd != NULL)
    {
        status = GAP_ERR_NO_ERROR;

        // Use local advertising identifier as advertising handle
        p_cmd->adv_hdl = p_actv->hdr.idx;
        // Set operation and written length
        p_cmd->operation = set_operation;
        p_cmd->data_len = wr_length;

        // Copy the data
        memcpy(&p_cmd->data[0], co_buf_data(p_data), wr_length);
        co_buf_head_release(p_data, wr_length);

        // Send the command
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, HL_PROC_EVENT_FIRST, gapm_adv_hci_cmd_cmp_handler);
    }

    return(status);
}

/**
 ****************************************************************************************
 * @brief Check provided advertising parameters in order to know if advertising
 * activity can be created:
 *      - Check if roles supported by the device allow advertising
 *      - Check advertising properties
 *      - Check privacy configuration
 * For several parameters such as advertising interval, channel map, ... host relies
 * on checks performed by the controller.
 *
 * @param[in] type              Advertising type (see enum #gapm_adv_type)
 * @param[in] own_addr_type     Own address type (see enum #gapm_own_addr)
 * @param[in] p_param           Pointer to advertising parameters
 *
 * @return GAP_ERR_NO_ERROR if advertising activity can be created, else GAP_ERR_INVALID_PARAM
 ****************************************************************************************
 */
__STATIC uint16_t gapm_adv_check_param(uint8_t type, uint8_t own_addr_type, const gapm_adv_create_param_t* p_param)
{
    // Error code, invalid parameter by default
    uint16_t status = GAP_ERR_INVALID_PARAM;

    do
    {
        // Current role must at least be broadcaster
        uint8_t needed_role = GAP_ROLE_BROADCASTER;

        if(p_param == NULL) break;

        if (own_addr_type > GAPM_GEN_NON_RSLV_ADDR) break;

        if (GETB(p_param->prop, GAPM_ADV_PROP_CONNECTABLE))
        {
            #if (HL_LE_PERIPHERAL)
            if (gapm_env.connections == HOST_CONNECTION_MAX)
            {
                // No more connection can be established
                status = GAP_ERR_COMMAND_DISALLOWED;
                break;
            }

            needed_role = GAP_ROLE_PERIPHERAL;
            #else
            // Cannot create a connectable advertising if peripheral role is not supported
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
            #endif //(HL_LE_PERIPHERAL)
        }

        // Check discovery mode
        if (p_param->disc_mode >= GAPM_ADV_MODE_MAX)
        {
            break;
        }

        // General and limited discoverable modes cannot be used if peripheral role is not supported
        if ((p_param->disc_mode != GAPM_ADV_MODE_NON_DISC)
            && (p_param->disc_mode != GAPM_ADV_MODE_BEACON))
        {
            needed_role = GAP_ROLE_PERIPHERAL;
        }

        // Check if this operation is supported by current role.
        if(!GAPM_IS_ROLE_SUPPORTED(needed_role))
        {
            // role not supported
            status = GAP_ERR_NOT_SUPPORTED;
            break;
        }

        // Check advertising type
        if (type > GAPM_ADV_TYPE_PERIODIC)
        {
            break;
        }

        // Checks for directed advertising
        if (GETB(p_param->prop, GAPM_ADV_PROP_DIRECTED))
        {
            // Device must be non discoverable and non scannable if directed advertising is used
            if ((p_param->disc_mode != GAPM_ADV_MODE_NON_DISC)
                    || (GETB(p_param->prop, GAPM_ADV_PROP_SCANNABLE)))
            {
                break;
            }
        }

        // Checks for legacy advertising
        if (type == GAPM_ADV_TYPE_LEGACY)
        {
            // Anonymous and Extended TX Power bits must be set to 0
            if (GETB(p_param->prop, GAPM_ADV_PROP_ANONYMOUS) ||
                GETB(p_param->prop, GAPM_ADV_PROP_TX_PWR))
            {
                break;
            }

            if (GETB(p_param->prop, GAPM_ADV_PROP_CONNECTABLE))
            {
                // Undirected connectable advertising must be scannable
                if (!GETB(p_param->prop, GAPM_ADV_PROP_DIRECTED))
                {
                    if (!GETB(p_param->prop, GAPM_ADV_PROP_SCANNABLE))
                    {
                        break;
                    }
                }
                // Directed connectable advertising must not be scannable
                else
                {
                    if (GETB(p_param->prop, GAPM_ADV_PROP_SCANNABLE))
                    {
                        break;
                    }
                }
            }

            // Direct advertising must be connectable
            if (GETB(p_param->prop, GAPM_ADV_PROP_DIRECTED) &&
                !GETB(p_param->prop, GAPM_ADV_PROP_CONNECTABLE))
            {
                break;
            }
        }
        else
        {
            // High duty cycle directed advertising cannot be used
            if (GETB(p_param->prop, GAPM_ADV_PROP_HDC))
            {
                break;
            }

            // Extended advertising
            if (type == GAPM_ADV_TYPE_EXTENDED)
            {
                // The advertisement shall not be both connectable and scannable
                if (GETB(p_param->prop, GAPM_ADV_PROP_CONNECTABLE)
                        && GETB(p_param->prop, GAPM_ADV_PROP_SCANNABLE))
                {
                    break;
                }

                // Anonymous mode is only available for non connectable non scannable non discoverable advertising
                if (GETB(p_param->prop, GAPM_ADV_PROP_ANONYMOUS))
                {
                    if (   (p_param->disc_mode == GAPM_ADV_MODE_GEN_DISC)
                        || (p_param->disc_mode == GAPM_ADV_MODE_LIM_DISC)
                        || GETB(p_param->prop, GAPM_ADV_PROP_CONNECTABLE)
                        || GETB(p_param->prop, GAPM_ADV_PROP_SCANNABLE))
                    {
                        break;
                    }
                }
            }
            // Periodic advertising
            else
            {
                // Connectable, anonymous, scannable bit must be set to 0
                if (GETB(p_param->prop, GAPM_ADV_PROP_CONNECTABLE) ||
                    GETB(p_param->prop, GAPM_ADV_PROP_SCANNABLE) ||
                    GETB(p_param->prop, GAPM_ADV_PROP_ANONYMOUS))
                {
                    break;
                }
            }
        }

        // Check filter policy
        if (type < GAPM_ADV_TYPE_PERIODIC)
        {
            if (p_param->filter_pol > ADV_ALLOW_SCAN_WLST_CON_WLST)
            {
                break;
            }

            // While a device is in limited/general discoverable mode the Host configures the Controller as follows:
            //  - The Host shall set the advertising filter policy to "process scan and
            //    connection requests from all devices".
            if (   (p_param->disc_mode == GAPM_ADV_MODE_GEN_DISC)
                || (p_param->disc_mode == GAPM_ADV_MODE_LIM_DISC))
            {
                if (p_param->filter_pol != ADV_ALLOW_SCAN_ANY_CON_ANY)
                {
                    break;
                }
            }
        }

        // Check privacy if enabled
        if (GETB(gapm_env.priv_cfg, GAPM_PRIV_CFG_PRIV_EN))
        {
            status = GAP_ERR_PRIVACY_CFG_PB;

            // If advertising is connectable, non-resolvable private address cannot be used
            if ((GETB(p_param->prop, GAPM_ADV_PROP_CONNECTABLE))
                    && (own_addr_type == GAPM_GEN_NON_RSLV_ADDR))
            {
                break;
            }
        }

        status = GAP_ERR_NO_ERROR;
    } while (0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Check if advertising has be properly prepared and can be started.
 * Following rules apply:
 *     - If advertising data must be provided, it has to be set at least once by the application
 * using the GAPM_SET_ADV_DATA_CMD even if its length is 0.
 *     - If scan response data must be provided, it has to be set at least once
 * by the application using the GAPM_SET_ADV_DATA_CMD except if advertising is a legacy
 * advertising.
 *     - If advertising is a periodic advertising, periodic advertising data has to
 * be provided at least once.
 *
 * @param[in] p_actv    Pointer to the advertising activity structure
 *
 * @return GAP_ERR_NO_ERROR if advertising can be started, else GAP_ERR_COMMAND_DISALLOWED
 ****************************************************************************************
 */
__STATIC uint8_t gapm_adv_check(gapm_adv_actv_t *p_actv)
{
    // Error code, command disallowed by default
    uint8_t status = GAP_ERR_COMMAND_DISALLOWED;

    do
    {
        // If advertising data can be set, advertising data has to be set at least once
        if (GETB(p_actv->info_bf, GAPM_ADV_INFO_ACPT_ADV_DATA)
                && !GETB(p_actv->info_bf, GAPM_ADV_INFO_ADV_DATA_SET))
        {
            break;
        }

        // If scan response data can be set, it has to be set at least once, except
        // if advertising is a legacy advertising
        if (GETB(p_actv->info_bf, GAPM_ADV_INFO_ACPT_SCAN_RSP_DATA)
                && (p_actv->hdr.subtype != GAPM_ADV_TYPE_LEGACY)
                && !GETB(p_actv->info_bf, GAPM_ADV_INFO_SCAN_RSP_DATA_SET))
        {
            break;
        }

        // If periodic advertising, periodic advertising data has to be set
        if ((p_actv->hdr.subtype == GAPM_ADV_TYPE_PERIODIC)
                && !GETB(p_actv->info_bf, GAPM_ADV_INFO_PER_ADV_DATA_SET))
        {
            break;
        }

        status = GAP_ERR_NO_ERROR;
    } while (0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Verify if advertising data type must be unique
 *
 * @param[in] adv_type  Type of advertising data
 *
 * @return True if unique, False else
 ****************************************************************************************
 */
__STATIC bool gapm_adv_is_advtype_unique(uint8_t type)
{
    // Advertising type check which shall be unique
    switch (type)
    {
        case GAP_AD_TYPE_MORE_16_BIT_UUID:
        case GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID:
        case GAP_AD_TYPE_MORE_32_BIT_UUID:
        case GAP_AD_TYPE_COMPLETE_LIST_32_BIT_UUID:
        case GAP_AD_TYPE_MORE_128_BIT_UUID:
        case GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID:
        case GAP_AD_TYPE_SHORTENED_NAME:
        case GAP_AD_TYPE_COMPLETE_NAME:
        case GAP_AD_TYPE_APPEARANCE:
        case GAP_AD_TYPE_ADV_INTV:
        case GAP_AD_TYPE_PUB_TGT_ADDR:
        case GAP_AD_TYPE_RAND_TGT_ADDR:
        case GAP_AD_TYPE_LE_BT_ADDR:
        case GAP_AD_TYPE_LE_ROLE:
        case GAP_AD_TYPE_FLAGS:
        {
            return true;
        }

        default:
        {
            return false;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Check content of advertising or scan response data provided by application.
 *
 * @param[in] p_data        Pointer to the buffer that contains advertising or scan response data
 *
 * @return true if provided advertising data is well formatted, else false
 ****************************************************************************************
 */
__STATIC bool gapm_adv_check_data_sanity(co_buf_t* p_data)
{
    // Returned status
    bool status = true;
    // Cursor
    uint8_t *p_cursor = co_buf_data(p_data);
    // End of data
    uint8_t *p_end_cursor = p_cursor + co_buf_data_len(p_data);
    // Check for duplicate information in advertising or scan response data.
    uint8_t dup_filter[GAP_AD_TYPE_BITFIELD_BYTES];

    // Clear presence status of unique advertising type
    memset(&dup_filter[0], 0, GAP_AD_TYPE_BITFIELD_BYTES);

    // AD type flags must not be set by application
    GAP_AD_TYPE_SET_BIT(dup_filter, GAP_AD_TYPE_FLAGS);

    // enusure that AD-Type can be read
    while ((p_cursor + 1) < p_end_cursor)
    {
        // Extract AD type
        uint8_t ad_type = *(p_cursor + 1);

        // Check if it's AD Type which shall be unique
        if (gapm_adv_is_advtype_unique(ad_type))
        {
            if (!GAP_AD_TYPE_CHECK_BIT(dup_filter, ad_type))
            {
                // Mark the advertising type as found
                GAP_AD_TYPE_SET_BIT(dup_filter, ad_type);
            }
            else
            {
                // Advertising type has been found twice
                break;
            }
        }

        /* Go to next advertising info */
        p_cursor += (*p_cursor + 1);
    }

    // Check if total advertising length is valid with advertising data info
    if (p_cursor != p_end_cursor)
    {
        status = false;
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Check if advertising data parameters are valid
 *
 * @param[in] p_actv_adv    Pointer to the advertising activity structure
 * @param[in] proc_id       Procedure identifier (see enum #gapm_actv_proc_id)
 * @param[in] p_data        Pointer to the advertising data buffer
 *
 * @return GAP_ERR_NO_ERROR if data and data parameters are valid, GAP_ERR_INVALID_PARAM
 * if a data parameter is not valid, GAP_ERR_ADV_DATA_INVALID if data is not well formatted
 ****************************************************************************************
 */
__STATIC uint8_t gapm_adv_check_data_param(gapm_adv_actv_t *p_actv, uint8_t proc_id, co_buf_t* p_data)
{
    // Returned status
    uint8_t status = GAP_ERR_INVALID_PARAM;
    // Length to be provided
    uint16_t length = co_buf_data_len(p_data);

    do
    {
        if(p_data == NULL) break;

        // No presence of AD type flags for beacons
        if(p_actv->mode != GAPM_ADV_MODE_BEACON)
        {
            // AD type flags are added to any advertising data or to scan response data
            // if advertising data cannot be set, we have to take length of this field in account
            if (   (proc_id == GAPM_ACTV_SET_ADV_DATA)
                || ((proc_id == GAPM_ACTV_SET_SCAN_RSP_DATA) && !GETB(p_actv->info_bf, GAPM_ADV_INFO_ACPT_ADV_DATA)))
            {
                length += GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
            }
        }

        // Check if data length is not too long for controller
        if (length > gapm_env.max_adv_data_len)
        {
            break;
        }

        // If advertising is a legacy advertising, length of advertising data cannot be greater
        // than 31 bytes
        if (p_actv->hdr.subtype == GAPM_ADV_TYPE_LEGACY)
        {
            if (length > GAP_ADV_DATA_LEN)
            {
                break;
            }
        }

        // Perform a advertising/scan response data sanity check
        if ((proc_id == GAPM_ACTV_SET_ADV_DATA) || (proc_id == GAPM_ACTV_SET_SCAN_RSP_DATA))
        {
            if (!gapm_adv_check_data_sanity(p_data))
            {
                status = GAP_ERR_ADV_DATA_INVALID;
                break;
            }
        }

        status = GAP_ERR_NO_ERROR;
    } while (0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Common function used to set Advertising, periodic or scan response data in controller
 *
 * @param[in] actv_idx          Activity local index
 * @param[in] proc_id           Procedure identifer (see enum #gapm_actv_proc_id)
 * @param[in] p_data            Pointer to buffer that contains Advertising data
 * @param[in] send_data_func    Function to call in order to send HCI command to controller
 * @param[in] data_set_bit      Bit to once data has been set at controller level
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for @see gapm_actv_cb_t.proc_cmp callback execution
 ****************************************************************************************
 */
__STATIC uint16_t gapm_adv_data_set(uint8_t actv_idx, uint8_t proc_id, co_buf_t* p_data,
                                    gapm_adv_hci_send_adv_data_func send_data_func, uint8_t data_set_bit)
{
    uint16_t status;

    do
    {
        gapm_adv_actv_t* p_actv = (gapm_adv_actv_t*) gapm_actv_get(actv_idx);
        gapm_adv_proc_data_set_t *p_proc;

        status = GAP_ERR_COMMAND_DISALLOWED;

        if((p_actv == NULL) || (p_actv->hdr.type != GAPM_ACTV_TYPE_ADV))
        {
            break;
        }

        // Data can be set only if activity state is CREATED or STARTED
        if (!((p_actv->hdr.state == GAPM_ACTV_CREATED) || (p_actv->hdr.state == GAPM_ACTV_STARTED)))
        {
            break;
        }

        if (proc_id == GAPM_ACTV_SET_ADV_DATA)
        {
            // Check if advertising data can be set
            if (!GETB(p_actv->info_bf, GAPM_ADV_INFO_ACPT_ADV_DATA))
            {
                break;
            }
        }
        else if (proc_id == GAPM_ACTV_SET_SCAN_RSP_DATA)
        {
            // Check if scan response data can be set
            if (!GETB(p_actv->info_bf, GAPM_ADV_INFO_ACPT_SCAN_RSP_DATA))
            {
                break;
            }
        }
        else if (proc_id == GAPM_ACTV_SET_PERIOD_ADV_DATA)
        {
            // Periodic advertising data can only be set for a periodic advertising
            if (p_actv->hdr.subtype != GAPM_ADV_TYPE_PERIODIC)
            {
                break;
            }
        }

        // Check advertising data
        status = gapm_adv_check_data_param(p_actv, proc_id, p_data);
        if (status != GAP_ERR_NO_ERROR) break;


        // create data set procedure
        status = gapm_actv_proc_create(&(p_actv->hdr), proc_id, sizeof(gapm_adv_proc_data_set_t), true,
                                       (gapm_actv_proc_transition_cb) gapm_adv_data_set_transition,
                                       (gapm_actv_proc_t**)&p_proc);
        if(status != GAP_ERR_NO_ERROR) break;
        co_buf_acquire(p_data);

        p_proc->p_data          = p_data;
        p_proc->send_data_func  = send_data_func;
        p_proc->data_set_bit    = data_set_bit;

    } while (0);

    return (status);
}


/**
 ****************************************************************************************
 * @brief Create an advertising activity.
 *
 * @param[in]  dummy            Dummy parameter provided by upper layer application
 * @param[in]  type             Advertising type (see enum #gapm_adv_type)
 * @param[in]  own_addr_type    Own address type (see enum #gapm_own_addr)
 * @param[in]  p_param          Pointer to advertising parameters
 * @param[in]  proc_size        Size of procedure structure
 * @param[in]  p_cbs            Activity Callback interface
 * @param[out] pp_proc          Pointer to the created procedure pointer to return
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for @see gapm_actv_cb_t.proc_cmp callback execution
 ****************************************************************************************
 */
__STATIC uint16_t gapm_adv_create(uint32_t dummy, uint8_t type, uint8_t own_addr_type, const gapm_adv_create_param_t* p_param,
                                  uint16_t proc_size, const gapm_adv_actv_cb_t* p_cbs, gapm_adv_proc_create_t ** pp_proc)
{
    uint16_t status;

    // Allocated advertising activity
    gapm_adv_actv_t *p_actv;
    gapm_adv_proc_create_t *p_proc;

    do
    {
        ASSERT_WARN(GAPM_HEAP_ENV_SIZE >= (sizeof(gapm_adv_actv_t) + KE_HEAP_MEM_RESERVED),
                    GAPM_HEAP_ENV_SIZE, sizeof(gapm_adv_actv_t));

        // Check if a new advertising set can be used in controller
        if (gapm_env.nb_adv_actv == gapm_env.max_adv_set)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Check callback
        if((p_cbs == NULL) || (p_cbs->created == NULL))
        {
            status = GAP_ERR_MISSING_CALLBACK;
            break;
        }

        if((type == GAPM_ADV_TYPE_PERIODIC) && (p_cbs->ext_adv_stopped == NULL))
        {
            status = GAP_ERR_MISSING_CALLBACK;
            break;
        }

        // Check provided advertising parameters
        status = gapm_adv_check_param(type, own_addr_type, p_param);
        if (status != GAP_ERR_NO_ERROR)
        {
            break;
        }

        // Allocate an activity structure
        status = gapm_actv_create(GAPM_ACTV_TYPE_ADV, type, dummy, sizeof(gapm_adv_actv_t),
                                  &gapm_adv_actv_itf, (gapm_actv_cb_t*) p_cbs, (gapm_actv_t**) &p_actv);
        if(status != GAP_ERR_NO_ERROR) break;


        ASSERT_ERR(proc_size >= sizeof(gapm_adv_proc_create_t));
        status = gapm_actv_proc_create(&(p_actv->hdr), GAPM_ACTV_CREATE, proc_size, false,
                                       (gapm_actv_proc_transition_cb) gapm_adv_create_transition,
                                       (gapm_actv_proc_t**)&p_proc);
        if(status != GAP_ERR_NO_ERROR)
        {
            gapm_actv_clean(&(p_actv->hdr), false);
            break;
        }

        // Fill the activity structure
        p_actv->hdr.type          = GAPM_ACTV_TYPE_ADV;
        p_actv->mode              = p_param->disc_mode;
        p_actv->own_addr_type     = own_addr_type;
        p_actv->info_bf           = 0;
        memcpy(&p_actv->addr, &gapm_env.addr, GAP_BD_ADDR_LEN);
        SETB(p_actv->add_info, GAPM_ADV_ADD_INFO_CONNECTABLE, GETB(p_param->prop, GAPM_ADV_PROP_CONNECTABLE));
        p_actv->cb_addr_renew = NULL;

        // Keep in mind if advertising data is expected
        //     -> Not expected if legacy directed advertising or scannable extended advertising
        if (!((p_actv->hdr.subtype == GAPM_ADV_TYPE_LEGACY) && (p_param->prop & ADV_DIRECT))
         && !((p_actv->hdr.subtype != GAPM_ADV_TYPE_LEGACY) && (p_param->prop & ADV_SCAN)))
        {
            SETB(p_actv->info_bf, GAPM_ADV_INFO_ACPT_ADV_DATA, true);
        }

        // Keep in mind if advertising is scan response data is expected
        if (p_param->prop & ADV_SCAN)
        {
            SETB(p_actv->info_bf, GAPM_ADV_INFO_ACPT_SCAN_RSP_DATA, true);
        }

        // Grant memory as not initialized padding bytes are copied
        DBG_MEM_INIT(p_param, sizeof(gapm_adv_create_param_t));
        p_proc->param = *p_param;
        p_proc->kept_status = GAP_ERR_NO_ERROR;

        // return procedure pointer
        *pp_proc = p_proc;
    } while (0);
    return (status);
}

/// @brief Function executed when AES execution is over.
__STATIC void gapm_adv_rand_addr_cb(uint8_t aes_status, const gap_addr_t* p_addr, uint32_t token)
{
    uint16_t status = RW_ERR_HCI_TO_HL(aes_status);

    if(status == GAP_ERR_NO_ERROR)
    {
        uint8_t actv_idx = GAPM_ADV_ACTV_IDX_GET(token);
        gapm_adv_actv_t* p_actv = (gapm_adv_actv_t*) gapm_actv_get(actv_idx);
        memcpy(&(p_actv->addr), p_addr, sizeof(gap_addr_t));
    }

    gapm_proc_transition(GAPM_PROC_AIR, GAPM_ADV_EVENT_GET(token), status);
}

/*
 * PROCEDURE TRANSITION - State machine
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Function called when an event is trigger that creates a transition in procedure state machine
 *
 * @param[in] p_proc     Pointer to procedure object
 * @param[in] event      Event type receive that induce procedure state transition
 * @param[in] status     Status linked to event transition (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC uint16_t gapm_adv_create_transition(gapm_adv_actv_t *p_actv, gapm_adv_proc_create_t* p_proc, uint8_t event,
                                             uint16_t status, bool* p_finished)
{
    uint8_t actv_idx = p_actv->hdr.idx;
    *p_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                // Allocate HCI command message
                struct hci_le_set_ext_adv_param_cmd *p_cmd =
                        HL_HCI_CMD_ALLOC(HCI_LE_SET_EXT_ADV_PARAM_CMD_OPCODE, hci_le_set_ext_adv_param_cmd);

                if(p_cmd == NULL)
                {
                    status = GAP_ERR_INSUFF_RESOURCES;
                    break;
                }

                // Use local advertising identifier as advertising handle
                p_cmd->adv_hdl = actv_idx;

                // Advertising event properties
                //    Following bits of field in HCI command can be directly mapped on those in received command
                //       - Connectable advertising bit (0)
                //       - Scannable advertising bit (1)
                //       - Directed advertising bit (2)
                //       - High Duty Cycle directed connectable advertising bit (3)
                //       - Legacy advertising bit (4)
                //       - Anonymous advertising (5)
                //       - Include TX power in extended headed (6)
                p_cmd->adv_evt_properties = (p_proc->param.prop & GAPM_ADV_EVT_PROP_MASK);

                // Set configuration for advertising on primary channel
                //    Primary advertising minimum interval
                co_write24p(p_cmd->prim_adv_intv_min, p_proc->param.prim_cfg.adv_intv_min);
                //    Primary advertising maximum interval
                co_write24p(p_cmd->prim_adv_intv_max, p_proc->param.prim_cfg.adv_intv_max);
                //    Primary advertising channel map
                p_cmd->prim_adv_chnl_map = p_proc->param.prim_cfg.chnl_map;
                //    Primary advertising PHY
                p_cmd->prim_adv_phy = p_proc->param.prim_cfg.phy;

                if (p_actv->hdr.subtype >= GAPM_ADV_TYPE_EXTENDED)
                {
                    gapm_adv_ext_proc_create_t* p_ext_proc = (gapm_adv_ext_proc_create_t*) p_proc;
                    // Set configuration for advertising on secondary channel
                    //    Secondary advertising max skip
                    p_cmd->sec_adv_max_skip = p_ext_proc->second_cfg.max_skip;
                    //    Secondary advertising PHY
                    p_cmd->sec_adv_phy = p_ext_proc->second_cfg.phy;
                    //    Advertising SID
                    p_cmd->adv_sid = p_ext_proc->second_cfg.adv_sid;
                }
                else
                {
                    p_cmd->adv_sid          = 0;
                    p_cmd->sec_adv_max_skip = 0;
                    p_cmd->sec_adv_phy      = 0;
                }

                //  Own address type
                p_cmd->own_addr_type = gapm_le_actv_get_hci_own_addr_type(p_actv->own_addr_type);

                //  Peer address if directed advertising is used or if resolvable private address generated by controller has to be used
                if (GETB(p_proc->param.prop, GAPM_ADV_PROP_DIRECTED) || GETB(gapm_env.priv_cfg, GAPM_PRIV_CFG_PRIV_EN))
                {
                    p_cmd->peer_addr_type = p_proc->param.peer_addr.addr_type;
                    memcpy(&p_cmd->peer_addr.addr[0], &p_proc->param.peer_addr.addr[0], BD_ADDR_LEN);
                }
                else
                {
                    p_cmd->peer_addr_type = 0;
                    memset(&p_cmd->peer_addr.addr[0], 0, BD_ADDR_LEN);
                }

                // Advertising filter policy
                p_cmd->adv_filt_policy = p_proc->param.filter_pol;
                // Advertising TX power
                p_cmd->adv_tx_pwr = p_proc->param.max_tx_pwr;
                // Enable scan request notification
                if (GETB(p_proc->param.prop, GAPM_ADV_PROP_SCAN_REQ_NTF_EN))
                {
                    p_cmd->scan_req_notif_en = GAPM_ADV_SCAN_REQ_NOTIF_EN;

                    // Keep in mind that notification for received scan requests is enabled
                    SETB(p_actv->info_bf, GAPM_ADV_INFO_SCAN_REQ_NTF, true);
                }
                else
                {
                    p_cmd->scan_req_notif_en = GAPM_ADV_SCAN_REQ_NOTIF_DIS;
                }

                // Send the command
                HL_HCI_CMD_SEND_TO_CTRL(p_cmd, GAPM_ADV_CREATE_HCI_SET_ADV_PARAM_CMP, gapm_hci_le_set_ext_adv_param_cmd_cmp_handler);
            } break;
            case GAPM_ADV_CREATE_HCI_SET_ADV_PARAM_CMP:
            {
                if (p_actv->hdr.subtype == GAPM_ADV_TYPE_PERIODIC)
                {
                    gapm_adv_periodic_proc_create_t* p_period_proc = (gapm_adv_periodic_proc_create_t*) p_proc;

                    // Allocate HCI command message
                    struct hci_le_set_per_adv_param_cmd *p_cmd =
                            HL_HCI_CMD_ALLOC(HCI_LE_SET_PER_ADV_PARAM_CMD_OPCODE, hci_le_set_per_adv_param_cmd);

                    if(p_cmd == NULL)
                    {
                        status = GAP_ERR_INSUFF_RESOURCES;
                        break;
                    }

                    // Use local advertising identifier as advertising handle
                    p_cmd->adv_hdl      = actv_idx;
                    // Advertising interval
                    p_cmd->adv_intv_min = p_period_proc->period_cfg.interval_min;
                    p_cmd->adv_intv_max = p_period_proc->period_cfg.interval_max;
                    // Check if advertising PDU shall contain TX Power
                    if (GETB(p_proc->param.prop, GAPM_ADV_PROP_PER_TX_PWR))
                    {
                        p_cmd->adv_prop = GAPM_ADV_PERIOD_PROP_TX_POWER_BIT;
                    }
                    else
                    {
                        p_cmd->adv_prop = 0;
                    }

                    // Send the command
                    HL_HCI_CMD_SEND_TO_CTRL(p_cmd, GAPM_ADV_CREATE_HCI_SET_PER_ADV_PARAM_CMP, gapm_adv_hci_cmd_cmp_handler);
                    break;
                }
            }
            // no break
            case GAPM_ADV_CREATE_HCI_SET_PER_ADV_PARAM_CMP:
            #if (BLE_AOD | BLE_AOA)
            {
                if (p_actv->hdr.subtype == GAPM_ADV_TYPE_PERIODIC)
                {
                    gapm_adv_periodic_proc_create_t* p_period_proc = (gapm_adv_periodic_proc_create_t*) p_proc;
                    gapm_adv_cte_cfg_t* p_cte_cfg = &(p_period_proc->cte_cfg);
                    // check if constant tone extension must be configured
                    if(p_cte_cfg->count > 0)
                    {
                        // Send a LE set connection-less CTE transmit parameters command to the controller
                        struct hci_le_set_conless_cte_tx_param_cmd *p_cmd =
                                HL_HCI_CMD_ALLOC(HCI_LE_SET_CONLESS_CTE_TX_PARAM_CMD_OPCODE, hci_le_set_conless_cte_tx_param_cmd);

                        if(p_cmd == NULL)
                        {
                            status = GAP_ERR_INSUFF_RESOURCES;
                            break;
                        }

                        // Use local advertising identifier as advertising handle
                        p_cmd->adv_hdl                = actv_idx;
                        // Fill CTE parameters
                        p_cmd->cte_len                = p_cte_cfg->length;
                        p_cmd->cte_type               = p_cte_cfg->type;
                        p_cmd->cte_count              = p_cte_cfg->count;
                        p_cmd->switching_pattern_len  = p_period_proc->switching_pattern_len;
                        memcpy(p_cmd->antenna_id, p_period_proc->antenna_id, p_period_proc->switching_pattern_len);

                        // Send the command
                        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, GAPM_ADV_CREATE_HCI_SET_CONLESS_CTE_TX_PARAM_CMP, gapm_adv_hci_cmd_cmp_handler);
                        break;
                    }
                }
            }
            // no break
            case GAPM_ADV_CREATE_HCI_SET_CONLESS_CTE_TX_PARAM_CMP:
            #endif // (BLE_AOD | BLE_AOA)
            {
                // Generate Random address
                if (p_actv->own_addr_type != GAPM_STATIC_ADDR)
                {
                    // Send an address generation command
                     aes_gen_rand_addr(gapm_get_irk()->key,
                                       (p_actv->own_addr_type == GAPM_GEN_RSLV_ADDR) ? BD_ADDR_RSLV : BD_ADDR_NON_RSLV,
                                       (aes_func_result_cb)gapm_adv_rand_addr_cb,
                                       GAPM_ADV_TOKEN_CREATE(actv_idx, GAPM_ADV_CREATE_GEN_RAND_ADDR_CMP));
                    break;
                }
            }
            // no break
            case GAPM_ADV_CREATE_GEN_RAND_ADDR_CMP:
            case GAPM_ADV_CREATE_HCI_REMOVE_SET_CMP:
            default:
            {
                *p_finished = true;
            } break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        p_proc->kept_status = status;
        if(   (event <= GAPM_ADV_CREATE_HCI_SET_ADV_PARAM_CMP)
           || (event == GAPM_ADV_CREATE_HCI_REMOVE_SET_CMP))
        {
            *p_finished = true;
        }
        else
        {
            // Send a LE Remove Advertising Set Command
            if(gapm_adv_send_hci_le_rmv_adv_set_cmd(actv_idx, GAPM_ADV_CREATE_HCI_REMOVE_SET_CMP) != GAP_ERR_NO_ERROR)
            {
                *p_finished = true;
            }
        }
    }

    if(*p_finished)
    {
        uint32_t dummy = p_actv->hdr.dummy;
        const gapm_adv_actv_cb_t* p_cbs = (const gapm_adv_actv_cb_t*) p_actv->hdr.p_cbs;
        status = p_proc->kept_status;

        if (status == GAP_ERR_NO_ERROR)
        {
            gapm_actv_created((gapm_actv_t *)p_actv);
            gapm_env.nb_adv_actv++;

            // inform that activity has been created
            p_cbs->created(dummy, actv_idx, p_proc->tx_pwr);

            if(p_actv->own_addr_type != GAPM_STATIC_ADDR)
            {
                // Inform the host about the BD address used for the activity
                gapm_le_actv_send_new_bdaddr(&(p_actv->hdr), &(p_actv->addr));

                // Ask for renew timer start
                gapm_le_actv_addr_renew_timer_start();
            }
        }
        else
        {
            // Free the allocated activity structure
            gapm_actv_clean(&(p_actv->hdr), false);
            actv_idx = GAP_INVALID_ACTV_IDX;
        }

        // Send procedure complete - Done manualy since activity can be deleted
        ((const gapm_actv_cb_t*) p_cbs)->proc_cmp(dummy, GAPM_ACTV_CREATE, actv_idx, status);
    }

    return (status);
}





/// Activity start transition
__STATIC uint16_t gapm_adv_proc_start_transition(gapm_adv_actv_t* p_actv, gapm_adv_proc_start_t* p_proc, uint8_t event,
                                                 uint16_t status, bool* p_finished)
{
    *p_finished = false;
    uint8_t actv_idx = p_actv->hdr.idx;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                // Set Random Address
                if ((p_actv->own_addr_type != GAPM_STATIC_ADDR) || GETB(gapm_env.priv_cfg, GAPM_PRIV_CFG_PRIV_ADDR))
                {
                    status = gapm_adv_send_hci_le_set_adv_set_rand_addr_cmd(p_actv, GAPM_ADV_START_HCI_SET_ADDR_CMP);
                    break;
                }
            }
            // no break
            case GAPM_ADV_START_HCI_SET_ADDR_CMP:
            {
                status = gapm_adv_send_hci_le_set_ext_adv_en_cmd(p_actv, &(p_proc->param), GAPM_ADV_START_HCI_ADV_EN_CMP);
            } break;
            case GAPM_ADV_START_HCI_ADV_EN_CMP:
            {
                // Keep in mind that advertising is running
                SETB(p_actv->info_bf, GAPM_ADV_INFO_ADV_EN, true);

                if (   (p_actv->hdr.subtype == GAPM_ADV_TYPE_PERIODIC)
                    && !GETB(p_actv->info_bf, GAPM_ADV_INFO_PER_ADV_EN))
                {
                    // Send a LE Set Periodic Advertising Enable command to the controller
                    status = gapm_adv_send_hci_le_set_per_adv_en_cmd(actv_idx, true, GAPM_ADV_START_HCI_PER_ADV_EN_CMP);
                }
                else
                {
                    *p_finished = true;
                }
            } break;
            case GAPM_ADV_START_HCI_PER_ADV_EN_CMP:
            {
                // Keep in mind that periodic advertising is running
                SETB(p_actv->info_bf, GAPM_ADV_INFO_PER_ADV_EN, true);
                *p_finished = true;
            } break;
            default:
            {
                *p_finished = true;
            } break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        p_proc->kept_status = status;
        if(   (event <= GAPM_ADV_START_HCI_ADV_EN_CMP)
           || (event == GAPM_ADV_START_HCI_ADV_DIS_CMP))
        {
            *p_finished = true;
        }
        else
        {
            // Stop advertising immediately
            if(gapm_adv_send_hci_le_set_ext_adv_en_cmd(p_actv, NULL, GAPM_ADV_START_HCI_ADV_DIS_CMP) != GAP_ERR_NO_ERROR)
            {
                *p_finished = true;
            }
        }
    }

    if(*p_finished)
    {
        status = p_proc->kept_status;

        if(status != GAP_ERR_NO_ERROR)
        {
            if (GETB(p_actv->add_info, GAPM_ADV_ADD_INFO_CONNECTABLE))
            {
                // Sanity check
                ASSERT_INFO(gapm_env.nb_connect_actvs != 0, gapm_env.nb_connect_actvs, 0);

                // Decrease number of started activities that can lead to a connection
                gapm_env.nb_connect_actvs--;
            }

            // Keep in mind that advertising is running
            SETB(p_actv->info_bf, GAPM_ADV_INFO_ADV_EN, false);
        }
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Function called when an event is trigger that creates a transition in procedure state machine
 *
 * @param[in] p_proc     Pointer to procedure object
 * @param[in] event      Event type receive that induce procedure state transition
 * @param[in] status     Status linked to event transition (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC uint16_t gapm_adv_data_set_transition(gapm_adv_actv_t *p_actv, gapm_adv_proc_data_set_t* p_proc, uint8_t event,
                                               uint16_t status, bool* p_finished)
{
    *p_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        if(co_buf_data_len(p_proc->p_data) == 0)
        {
            *p_finished = true;
        }
        else
        {
            // continue transmission of adv fragment
            status = p_proc->send_data_func(p_actv, p_proc->p_data, event == HL_PROC_GRANTED);
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        *p_finished = true;
    }

    if(*p_finished)
    {
        // release buffer
        co_buf_release(p_proc->p_data);

        if(status == GAP_ERR_NO_ERROR)
        {
            // Keep in mind that advertising data has been set
            p_actv->info_bf |= p_proc->data_set_bit;
        }
    }

    return (status);
}


__STATIC uint16_t gapm_adv_stop_transition(gapm_adv_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                           uint16_t status, bool* p_finished)
{
    uint8_t actv_idx = p_actv->hdr.idx;
    *p_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                if (GETB(p_actv->info_bf, GAPM_ADV_INFO_ADV_EN))
                {
                    // Send a LE Set Extended Advertising Enable (Disable) command to the controller
                    status = gapm_adv_send_hci_le_set_ext_adv_en_cmd(p_actv, NULL, GAPM_ADV_STOP_HCI_ADV_DIS_CMP);
                    break;
                }
            }
            // no break
            case GAPM_ADV_STOP_HCI_ADV_DIS_CMP:
            {
                SETB(p_actv->info_bf, GAPM_ADV_INFO_ADV_EN, false);

                if (GETB(p_actv->info_bf, GAPM_ADV_INFO_PER_ADV_EN))
                {
                    // Send a LE Set Periodic Advertising Enable (Disable) command to the controller
                    status = gapm_adv_send_hci_le_set_per_adv_en_cmd(actv_idx, false, GAPM_ADV_STOP_HCI_PER_ADV_DIS_CMP);
                    break;
                }
            }
            // no break
            case GAPM_ADV_STOP_HCI_PER_ADV_DIS_CMP:
            {
                SETB(p_actv->info_bf, GAPM_ADV_INFO_PER_ADV_EN, false);
            }
            // no break
            default:
            {
                *p_finished = true;

                if (GETB(p_actv->add_info, GAPM_ADV_ADD_INFO_CONNECTABLE))
                {
                    ASSERT_INFO(gapm_env.nb_connect_actvs != 0, gapm_env.nb_connect_actvs, 0);
                    // Decrease number of started activities that can lead to a connection
                    gapm_env.nb_connect_actvs--;
                }
            } break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        *p_finished = true;
    }

    return (status);
}

__STATIC uint16_t gapm_adv_delete_transition(gapm_adv_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                             uint16_t status, bool* p_finished)
{
    uint8_t actv_idx = p_actv->hdr.idx;
    *p_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                // Send a LE Set Extended Advertising Enable (Disable) command to the controller
                status = gapm_adv_send_hci_le_rmv_adv_set_cmd(actv_idx, GAPM_ADV_DELETE_HCI_ADV_RMV_CMP);
            } break;
            case GAPM_ADV_DELETE_HCI_ADV_RMV_CMP:
            default:
            {
                *p_finished = true;
                gapm_env.nb_adv_actv--;
            } break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        *p_finished = true;
    }

    return (status);
}

/// Advertising address renew procedure state machine
__STATIC uint16_t gapm_adv_addr_renew_transition(gapm_adv_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                                 uint16_t status, bool* p_finished)
{
    uint8_t actv_idx = p_actv->hdr.idx;
    *p_finished = false;

    if(p_actv->own_addr_type == GAPM_STATIC_ADDR)
    {
        *p_finished = true;
    }
    else if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                // !!!!! It is assumed here that provided kind of address in valid (always the case for advertising
                //       activities, checked with gapm_actv_is_addr_type_valid function for initiating and scanning
                //       activities)
                // !!!!! It is assumed here that the function is called as part of execution of an air operation
                //       so that it does not enter in conflict with address renewal procedure !!!!!
                // !!!!! It is assumed that the activity is in CREATING state !!!!!

                // Check kind of activity
                //     - If activity is an advertising activity it can have its own random address so a
                // new address can be generated if it is not required to use the random static address
                //     - Initiating activity and scanning activity must share the same random address.
                // If a random address has already been generated for one of these activities, we can reuse
                // the same.

                #if (BLE_GAF_PRESENT)
                if (p_actv->cb_addr_renew != NULL)
                {
                    p_actv->cb_addr_renew();

                    // Do not generate a new address, advertising activity will be stopped
                    *p_finished = true;
                    break;
                }
                #endif //(BLE_GAF_PRESENT)

                // Send an address generation command
                aes_gen_rand_addr(gapm_get_irk()->key,
                                  (p_actv->own_addr_type == GAPM_GEN_RSLV_ADDR) ? BD_ADDR_RSLV : BD_ADDR_NON_RSLV,
                                  (aes_func_result_cb) gapm_adv_rand_addr_cb,
                                  GAPM_ADV_TOKEN_CREATE(actv_idx, GAPM_ADV_RENEW_GEN_RAND_ADDR_CMP));
            }
            break;
            case GAPM_ADV_RENEW_GEN_RAND_ADDR_CMP:
            {
                // Set Random Address
                status = gapm_adv_send_hci_le_set_adv_set_rand_addr_cmd(p_actv, GAPM_ADV_RENEW_HCI_SET_ADDR_CMP);
            } break;
            case GAPM_ADV_RENEW_HCI_SET_ADDR_CMP:
            default:
            {
                // Inform upper layer software about new random address
                gapm_le_actv_send_new_bdaddr(&(p_actv->hdr), &(p_actv->addr));
                *p_finished = true;
            } break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        *p_finished = true;
    }

    return (status);
}

#if(BLE_AOA || BLE_AOD)
__STATIC uint16_t gapm_adv_cte_tx_ctrl_transition(gapm_adv_actv_t *p_actv, gapm_adv_proc_cte_tx_ctrl_t* p_proc,
                                                  uint8_t event, uint16_t status, bool* p_finished)
{
    uint8_t actv_idx = p_actv->hdr.idx;
    *p_finished = true;

    if(event == HL_PROC_GRANTED)
    {
        struct hci_le_set_conless_cte_tx_en_cmd* p_cmd =
                HL_HCI_CMD_ALLOC(HCI_LE_SET_CONLESS_CTE_TX_EN_CMD_OPCODE, hci_le_set_conless_cte_tx_en_cmd);

        if(p_cmd)
        {
            p_cmd->adv_hdl = actv_idx;
            p_cmd->cte_en  = p_proc->enable;
            HL_HCI_CMD_SEND_TO_CTRL(p_cmd, HL_PROC_FINISHED, gapm_adv_hci_cmd_cmp_handler);
            *p_finished = false;
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    return (status);
}
#endif // (BLE_AOA || BLE_AOD)

/*
 * EXTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint8_t gapm_adv_hdl_get(uint8_t actv_idx)
{
    uint8_t adv_hdl = GAP_INVALID_CONIDX;
    gapm_actv_t* p_actv = gapm_actv_get(actv_idx);
    // Check that activity is well an advertising activity
    if ((p_actv != NULL) && (p_actv->type == GAPM_ACTV_TYPE_ADV))
    {
        // Use local advertising identifier as advertising handle
        adv_hdl = p_actv->idx;
    }

    return (adv_hdl);
}

gap_addr_t* gapm_adv_get_addr(uint8_t actv_idx)
{
    // Pointer to required address
    gap_addr_t* p_addr = NULL;
    // Pointer to Activity structure
    gapm_adv_actv_t* p_actv = (gapm_adv_actv_t*)gapm_actv_get(actv_idx);

    // Check that activity is well an advertising activity
    if ((p_actv != NULL) && (p_actv->hdr.type == GAPM_ACTV_TYPE_ADV))
    {
        p_addr = &p_actv->addr;
    }

    return (p_addr);
}
#endif //(HL_LE_BROADCASTER)


uint16_t gapm_adv_legacy_create(uint32_t dummy, uint8_t own_addr_type, const gapm_adv_create_param_t* p_param,
                                const gapm_adv_actv_cb_t* p_cbs)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_BROADCASTER)
    // Allocated advertising activity
    gapm_adv_proc_create_t *p_proc;

    do
    {
        // create activity
        status = gapm_adv_create(dummy, GAPM_ADV_TYPE_LEGACY, own_addr_type, p_param, sizeof(gapm_adv_proc_create_t),
                                 p_cbs, &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        // Make sure legacy mode properly configured in properties
        p_proc->param.prop |= ADV_LEGACY;
    } while (0);
    #endif // (HL_LE_BROADCASTER)
    return (status);
}

uint16_t gapm_adv_ext_create(uint32_t dummy, uint8_t own_addr_type, const gapm_adv_create_param_t* p_param,
                             const gapm_adv_second_cfg_t* p_second_cfg,  const gapm_adv_actv_cb_t* p_cbs)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_BROADCASTER)
    // Allocated advertising activity
    gapm_adv_ext_proc_create_t *p_proc;

    do
    {
        if(p_second_cfg == NULL)
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // create activity
        status = gapm_adv_create(dummy, GAPM_ADV_TYPE_EXTENDED, own_addr_type, p_param, sizeof(gapm_adv_ext_proc_create_t),
                                 p_cbs, (gapm_adv_proc_create_t**)&p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        // Make sure legacy mode properly configured in properties
        p_proc->hdr.param.prop &= ~ADV_LEGACY;
        p_proc->second_cfg  = *p_second_cfg;

    } while (0);
    #endif // (HL_LE_BROADCASTER)

    return (status);
}

uint16_t gapm_adv_periodic_create(uint32_t dummy, uint8_t own_addr_type, const gapm_adv_create_param_t* p_param,
                                  const gapm_adv_second_cfg_t* p_second_cfg, const gapm_adv_period_cfg_t* p_period_cfg,
                                  const gapm_adv_actv_cb_t* p_cbs)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_BROADCASTER)
    // Allocated advertising activity
    gapm_adv_periodic_proc_create_t *p_proc;

    do
    {
        if((p_second_cfg == NULL) || (p_period_cfg == NULL))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // create activity
        status = gapm_adv_create(dummy, GAPM_ADV_TYPE_PERIODIC, own_addr_type, p_param, sizeof(gapm_adv_periodic_proc_create_t),
                                 p_cbs, (gapm_adv_proc_create_t**)&p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        // Make sure legacy mode properly configured in properties
        p_proc->hdr.hdr.param.prop &= ~ADV_LEGACY;
        p_proc->hdr.second_cfg      = *p_second_cfg;
        p_proc->period_cfg          = *p_period_cfg;
        p_proc->cte_cfg.count       = 0; // NO CTE

    } while (0);
    #endif // (HL_LE_BROADCASTER)

    return (status);
}

uint16_t gapm_adv_periodic_with_cte_create(uint32_t dummy, uint8_t own_addr_type, const gapm_adv_create_param_t* p_param,
                       const gapm_adv_second_cfg_t* p_second_cfg, const gapm_adv_period_cfg_t* p_period_cfg,
                       const gapm_adv_cte_cfg_t* p_cte_cfg, uint8_t switching_pattern_len, const uint8_t* p_antenna_id,
                       const gapm_adv_actv_cb_t* p_cbs)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_BROADCASTER)
    gapm_adv_periodic_proc_create_t *p_proc;

    do
    {
        uint16_t size = sizeof(gapm_adv_periodic_proc_create_t) + switching_pattern_len;
        if((p_second_cfg == NULL) || (p_period_cfg == NULL) || (p_cte_cfg == NULL) || (p_antenna_id == NULL))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // To ensure that hci packer doesn't crash
        if((p_cte_cfg->count > 0) && (switching_pattern_len > MAX_SWITCHING_PATTERN_LEN))
        {
            status = LL_ERR_INVALID_HCI_PARAM;
            break;
        }

        // create activity
        status = gapm_adv_create(dummy, GAPM_ADV_TYPE_PERIODIC, own_addr_type, p_param, size,
                                 p_cbs, (gapm_adv_proc_create_t**)&p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        // Make sure legacy mode properly configured in properties
        p_proc->hdr.hdr.param.prop   &= ~ADV_LEGACY;
        p_proc->hdr.second_cfg        = *p_second_cfg;
        p_proc->period_cfg            = *p_period_cfg;
        p_proc->cte_cfg               = *p_cte_cfg;
        p_proc->switching_pattern_len = switching_pattern_len;
        memcpy(p_proc->antenna_id,      p_antenna_id, switching_pattern_len);
    } while (0);
    #endif // (HL_LE_BROADCASTER)

    return (status);
}


uint16_t gapm_adv_start(uint8_t actv_idx, const gapm_adv_param_t* p_param)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_BROADCASTER)
    do
    {
        gapm_adv_actv_t* p_actv = (gapm_adv_actv_t*) gapm_actv_get(actv_idx);
        gapm_adv_proc_start_t *p_proc;

        if((p_actv == NULL) || (p_actv->hdr.type != GAPM_ACTV_TYPE_ADV))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        if(p_param == NULL)
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Check if advertising data has been set
        status = gapm_adv_check(p_actv);
        if (status != GAP_ERR_NO_ERROR) break;

        // If advertising is a connectable advertising check if a new connection can be established
        if (GETB(p_actv->add_info, GAPM_ADV_ADD_INFO_CONNECTABLE))
        {
            if ((gapm_env.connections + gapm_env.nb_connect_actvs) >= HOST_CONNECTION_MAX)
            {
                status = GAP_ERR_COMMAND_DISALLOWED;
                break;
            }
        }

        // Specific case when periodic adv is started and not normal adv
        if (p_actv->hdr.state == GAPM_ACTV_STARTED)
        {
            // Verify that advertising is not running
            if (GETB(p_actv->info_bf, GAPM_ADV_INFO_ADV_EN))
            {
                status = GAP_ERR_COMMAND_DISALLOWED;
                break;
            }
        }

        // start activity
        status = gapm_actv_start(&(p_actv->hdr), sizeof(gapm_adv_proc_start_t), (gapm_actv_proc_t**)&p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        // Grant memory as not initialized padding bytes are copied
        DBG_MEM_INIT(p_param, sizeof(gapm_adv_create_param_t));
        p_proc->kept_status = GAP_ERR_NO_ERROR;
        p_proc->param = *p_param;

        if (GETB(p_actv->add_info, GAPM_ADV_ADD_INFO_CONNECTABLE))
        {
            gapm_env.nb_connect_actvs++;
        }

    } while (0);
    #endif // (HL_LE_BROADCASTER)

    return (status);
}

uint16_t gapm_adv_set_data(uint8_t actv_idx, co_buf_t* p_data)
{
    uint16_t status =  GAP_ERR_NOT_SUPPORTED;
    #if (HL_LE_BROADCASTER)
    status = gapm_adv_data_set(actv_idx, GAPM_ACTV_SET_ADV_DATA, p_data, gapm_adv_send_hci_le_set_ext_adv_data_cmd,
                               GAPM_ADV_INFO_ADV_DATA_SET_BIT);
    #endif // (HL_LE_BROADCASTER)
    return (status);
}

uint16_t gapm_adv_set_scan_rsp(uint8_t actv_idx, co_buf_t* p_data)
{
    uint16_t status =  GAP_ERR_NOT_SUPPORTED;
    #if (HL_LE_BROADCASTER)
    status = gapm_adv_data_set(actv_idx, GAPM_ACTV_SET_SCAN_RSP_DATA, p_data, gapm_adv_send_hci_le_set_ext_scan_rsp_data_cmd,
                               GAPM_ADV_INFO_SCAN_RSP_DATA_SET_BIT);
    #endif // (HL_LE_BROADCASTER)
    return (status);
}

uint16_t gapm_adv_set_period_data(uint8_t actv_idx, co_buf_t* p_data)
{
    uint16_t status =  GAP_ERR_NOT_SUPPORTED;
    #if (HL_LE_BROADCASTER)
    status = gapm_adv_data_set(actv_idx, GAPM_ACTV_SET_PERIOD_ADV_DATA, p_data, gapm_adv_send_hci_le_set_per_adv_data_cmd,
                               GAPM_ADV_INFO_PER_ADV_DATA_SET_BIT);
    #endif // (HL_LE_BROADCASTER)
    return (status);
}


uint16_t gapm_adv_periodic_cte_tx_ctl(uint8_t actv_idx, bool enable)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_BROADCASTER && (BLE_AOA || BLE_AOD))
    do
    {
        gapm_adv_actv_t* p_actv = (gapm_adv_actv_t*) gapm_actv_get(actv_idx);
        gapm_adv_proc_cte_tx_ctrl_t *p_proc;

        if(   (p_actv == NULL) || (p_actv->hdr.type != GAPM_ACTV_TYPE_ADV)
           || (p_actv->hdr.subtype != GAPM_ADV_TYPE_PERIODIC))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // create data set procedure
        status = gapm_actv_proc_create(&(p_actv->hdr), GAPM_ACTV_PERIOD_ADV_CTE_TX_CTRL,
                                       sizeof(gapm_adv_proc_cte_tx_ctrl_t), true,
                                       (gapm_actv_proc_transition_cb) gapm_adv_cte_tx_ctrl_transition,
                                       (gapm_actv_proc_t**)&p_proc);
        if(status != GAP_ERR_NO_ERROR) break;
        p_proc->enable = enable;

    } while (0);
    #endif // (HL_LE_BROADCASTER && (BLE_AOA || BLE_AOD))

    return (status);
}

uint16_t gapm_adv_set_cb_addr_renew(uint8_t actv_idx, gapm_adv_cb_addr_renew cb_addr_renew)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (BLE_GAF_PRESENT)
    do
    {
        gapm_adv_actv_t* p_actv = (gapm_adv_actv_t*)gapm_actv_get(actv_idx);

        if ((p_actv == NULL)
                || (p_actv->hdr.type != GAPM_ACTV_TYPE_ADV)
                || (p_actv->cb_addr_renew != NULL))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        p_actv->cb_addr_renew = cb_addr_renew;
        status = GAP_ERR_NO_ERROR;
    } while (0);
    #endif //(BLE_GAF_PRESENT)

    return (status);
}

/// @} GAPM_ADV
