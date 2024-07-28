/**
 ****************************************************************************************
 *
 * @file gapm_init.c
 *
 * @brief Generic Access Profile Manager - Initiating manager module.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_INIT Generic Access Profile Manager - Initiating manager module.
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "gapm_le_init.h"
#if (HL_LE_CENTRAL)
#include "gapm_int.h"
#include "gapc_le.h"

#include <string.h>

#include "gap.h"
#include "../gap_int.h"
#include "ke_mem.h"
#include "hl_hci.h"
#include "hci.h"
#include "co_math.h"
#include "co_utils.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Initiating activity start procedure states
enum gapm_init_start_proc_event
{
    /// Received HCI_LE_SET_RAND_ADDR_CMD complete event
    GAPM_INIT_PROC_START_HCI_SET_RAND_ADDR_CMP = HL_PROC_EVENT_FIRST,
    /// Received HCI_LE_EXT_CREATE_CON_CMD complete event
    GAPM_INIT_PROC_START_HCI_CREATE_CON_CMP,
};

/*
 * TYPES DEFINITIONS
 ****************************************************************************************
 */
/// GAP Manager activity structure for initiating activity
typedef struct gapm_init_act
{
    /// Activity inherited parameters
    gapm_actv_t       hdr;
    /// Timer use to stop automatic connection after it elapse
    co_time_timer_t   auto_con_to_timer;
    /// Initiating parameters
    gapm_init_param_t param;
    /// Status to be triggered when stopping activity
    uint16_t          stop_status;
    /// Number of connection to be established for automatic connection
    ///    -> Number of devices in the white list when GAPM_ACTIVITY_START_CMD is received
    uint8_t           nb_con;
    /// Connection index used for name discovery
    uint8_t           conidx;
    /// Information bit field, meaning depends on activity type
    uint8_t           info_bf;
} gapm_init_actv_t;

/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

__STATIC uint16_t gapm_init_proc_start_transition(gapm_init_actv_t* p_actv, gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_init_stop_transition(gapm_init_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_init_delete_transition(gapm_init_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_init_addr_renew_transition(gapm_init_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);

__STATIC void gapm_init_hci_cmd_cmp_handler(uint16_t opcode, uint16_t event, struct hci_basic_cmd_cmp_evt const *p_evt);

__STATIC uint16_t gapm_init_send_hci_le_ext_create_con_cmd(gapm_init_actv_t *p_actv, const gapm_init_param_t *p_param,
                                                           uint8_t event, hl_hci_cmd_evt_func_t cmp_evt_cb);

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Activity object interface
__STATIC const gapm_actv_itf_t gapm_init_actv_itf =
{
    .cb_clean                  = (gapm_actv_clean_cb) gapm_actv_clean, // Use default destructor
    .cb_start_transition       = (gapm_actv_proc_transition_cb) gapm_init_proc_start_transition,
    .cb_stop_transition        = (gapm_actv_proc_transition_cb) gapm_init_stop_transition,
    .cb_delete_transition      = (gapm_actv_proc_transition_cb) gapm_init_delete_transition,
    .cb_addr_renew_transition  = (gapm_actv_proc_transition_cb) gapm_init_addr_renew_transition,
};

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Check initiating parameters provided by the application.
 *
 * @param[in] p_param      Pointer to the initiating parameters
 *
 * @return GAP_ERR_NO_ERROR if parameters are valid, else GAP_ERR_INVALID_PARAM
 ****************************************************************************************
 */
__STATIC uint16_t gapm_init_check_param(const gapm_init_param_t *p_param)
{
    // Error code, invalid parameter by default
    uint16_t status = GAP_ERR_INVALID_PARAM;

    do
    {
        // Check initiating type
        if (p_param->type > GAPM_INIT_TYPE_NAME_DISC) break;

        // Check peer address type
        if (p_param->peer_addr.addr_type > ADDR_RAND) break;

        status = GAP_ERR_NO_ERROR;
    } while (0);

    return (status);
}

/// Initiating is stopped, clean-up
__STATIC void gapm_init_stopped(gapm_init_actv_t *p_actv)
{
    gapm_env.init_actv_idx = GAPM_ACTV_INVALID_IDX;
    // Decrease number of started activities that can lead to a connection
    ASSERT_INFO(gapm_env.nb_connect_actvs != 0, gapm_env.nb_connect_actvs, 0);
    gapm_env.nb_connect_actvs--;

    // Clear timeout timer for automatic connection
    co_time_timer_stop(&(p_actv->auto_con_to_timer));
    p_actv->conidx = GAP_INVALID_CONIDX;
}

/// Handle disconnection completed
void gapm_init_disconnect_cmp_cb(uint8_t conidx, uint32_t actv_idx, uint16_t status)
{
    // Update transition of the procedure
    gapm_proc_transition(GAPM_PROC_AIR, HL_PROC_FINISHED, status);
}

#if (BLE_GATT_CLI)
/// Handle result of device name read
__STATIC void gapm_init_read_device_name_cmp_cb(uint8_t conidx, uint32_t actv_idx, uint16_t status, uint16_t handle,
                                                co_buf_t* p_name)
{
    gapm_init_actv_t *p_actv = (gapm_init_actv_t *)gapm_actv_get(actv_idx);
    if((p_actv != NULL) && (p_actv->conidx == conidx))
    {
        if(status == GAP_ERR_NO_ERROR)
        {
            // inform upper layer software about received name
            gapm_init_actv_cb_t* p_cbs = (gapm_init_actv_cb_t*) p_actv->hdr.p_cbs;
            p_cbs->peer_name(p_actv->hdr.dummy, p_actv->hdr.idx, gapc_le_get_peer_bdaddr(p_actv->conidx),
                             co_buf_data_len(p_name), co_buf_data(p_name));
        }

        // Keep operation status
        if (p_actv->stop_status == GAP_ERR_NO_ERROR)
        {
            p_actv->stop_status = status;
        }

        // Stop the activity
        gapm_actv_stop_quiet(p_actv->hdr.idx);
    }
}
#endif // (BLE_GATT_CLI)

/*
 * HCI HANDLERS DEFINITIONS
 ****************************************************************************************
 */

/// Default HCI Command complete handler used to continue procedure execution
__STATIC void gapm_init_hci_cmd_cmp_handler(uint16_t opcode, uint16_t event, struct hci_basic_cmd_cmp_evt const *p_evt)
{
    gapm_proc_transition(GAPM_PROC_AIR, (uint8_t) event, RW_ERR_HCI_TO_HL(p_evt->status));
}

/// HCI Command complete handler used for auto connection continuation
__STATIC void gapm_init_auto_con_hci_cmd_cmp_handler(uint16_t opcode, uint16_t event, struct hci_basic_cmd_cmp_evt const *p_evt)
{
    gapm_init_actv_t *p_actv = (gapm_init_actv_t *)gapm_actv_get(gapm_env.init_actv_idx);
    if(p_actv)
    {
        uint16_t status = RW_ERR_HCI_TO_HL(p_evt->status);
        if(status != GAP_ERR_NO_ERROR)
        {
            // an error occurs, stop activity
            p_actv->stop_status = status;
            gapm_actv_stop_quiet(p_actv->hdr.idx);
        }
    }
}


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
void gapm_init_hci_le_enh_con_cmp_evt_handler(uint8_t evt_code, struct hci_le_enh_con_cmp_evt const *p_evt)
{
    bool auto_disconnect = (p_evt->status == CO_ERROR_NO_ERROR);

    // Retrieve initiating activity currently in use
    gapm_init_actv_t *p_actv = (gapm_init_actv_t *)gapm_actv_get(gapm_env.init_actv_idx);
    if (p_actv)
    {
        bool stop_activity = true;
        uint8_t conidx = GAP_INVALID_CONIDX;
        uint16_t status = RW_ERR_HCI_TO_HL(p_evt->status);

        if (status == GAP_ERR_NO_ERROR)
        {
            status  = gapm_le_actv_con_create(&(p_actv->hdr), (p_actv->hdr.subtype == GAPM_INIT_TYPE_NAME_DISC),  p_evt,
                                              gapm_env.scan_init_own_addr_type, &(gapm_env.scan_init_rand_addr), &conidx);
        }

        if(status == GAP_ERR_NO_ERROR)
        {
            auto_disconnect = false;

            // Next action depends on initiating activity subtype
            switch (p_actv->hdr.subtype)
            {
                #if (BLE_GATT_CLI)
                case GAPM_INIT_TYPE_NAME_DISC:
                {
                    // activate connection - no bond data
                    gapc_le_connection_cfm(conidx, p_actv->hdr.dummy, NULL);

                    status = gapc_read_device_name(conidx, p_actv->hdr.idx, gapm_init_read_device_name_cmp_cb);
                    if(status != GAP_ERR_NO_ERROR) break;

                    stop_activity = false;
                    p_actv->conidx = conidx;
                }
                break;
                #endif // (BLE_GATT_CLI)

                case GAPM_INIT_TYPE_AUTO_CONN_EST:
                {
                    // Sanity check
                    ASSERT_ERR(p_actv->nb_con);

                    // Decrease the number of connection to be established
                    p_actv->nb_con--;

                    // Check if there is more connections to be established - Activity can be stopped
                    if(p_actv->nb_con == 0) break;

                    // Check if a new connection can be established
                    if ((gapm_env.connections + gapm_env.nb_connect_actvs) >= HOST_CONNECTION_MAX)
                    {
                        status = GAP_ERR_INSUFF_RESOURCES;
                        break;
                    }
                    // Send a LE Extended Create Connection command to the controller
                    status = gapm_init_send_hci_le_ext_create_con_cmd(p_actv, &(p_actv->param), HL_PROC_FINISHED,
                                                       (hl_hci_cmd_evt_func_t) gapm_init_auto_con_hci_cmd_cmp_handler);
                    if(status != GAP_ERR_NO_ERROR) break;
                    stop_activity = false;
                }
                break;

                // Activity can be stopped
                default: {  /* Nothing to do */ } break;
            }
        }

        if (stop_activity)
        {
            // Stop the activity
            gapm_init_stopped(p_actv);
            gapm_actv_stopped(&(p_actv->hdr), status);
        }
    }


    // Automatically send a disconnect
    if(auto_disconnect)
    {
        gapm_le_actv_send_hci_disconnect(p_evt->conhdl, CO_ERROR_MEMORY_CAPA_EXCEED);
    }
}


/**
 ****************************************************************************************
 * @brief Send a LE Extended Create Connection command over HCI.
 *
 * @param[in] p_actv       Pointer to the scan activity structure
 * @param[in] p_param      Pointer to the initiating parameters provided by the application
 * @param[in] event        Event type receive that induce procedure state transition
 * @param[in] cmp_cb       Callback to command complete
 ****************************************************************************************
 */
__STATIC uint16_t gapm_init_send_hci_le_ext_create_con_cmd(gapm_init_actv_t *p_actv, const gapm_init_param_t *p_param,
                                                           uint8_t event, hl_hci_cmd_evt_func_t cmp_evt_cb)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    // Allocate HCI command message
    struct hci_le_ext_create_con_cmd *p_cmd =
            HL_HCI_CMD_ALLOC(HCI_LE_EXT_CREATE_CON_CMD_OPCODE, hci_le_ext_create_con_cmd);

    if(p_cmd != NULL)
    {
        uint8_t phy_idx = 0;
        status = GAP_ERR_NO_ERROR;
        // Fill the command parameters
        p_cmd->own_addr_type = gapm_le_actv_get_hci_own_addr_type(gapm_env.scan_init_own_addr_type);

        if (p_actv->hdr.subtype != GAPM_INIT_TYPE_AUTO_CONN_EST)
        {
            // Do not use white list
            p_cmd->init_filter_policy = INIT_FILT_IGNORE_WLST;

            // Peer address
            p_cmd->peer_addr_type = p_param->peer_addr.addr_type;
            memcpy(&p_cmd->peer_addr.addr[0], &p_param->peer_addr.addr[0], GAP_BD_ADDR_LEN);
        }
        else
        {
            // Use white list
            p_cmd->init_filter_policy = INIT_FILT_USE_WLST;
            p_cmd->peer_addr_type = 0;
            memset(&p_cmd->peer_addr.addr[0], 0, GAP_BD_ADDR_LEN);
        }

        p_cmd->init_phys = p_param->prop & (GAPM_INIT_PROP_1M_BIT | GAPM_INIT_PROP_2M_BIT |
                                                 GAPM_INIT_PROP_CODED_BIT);

        if (p_param->prop & GAPM_INIT_PROP_1M_BIT)
        {
            p_cmd->phy[phy_idx].scan_interval = p_param->scan_param_1m.scan_intv;
            p_cmd->phy[phy_idx].scan_window   = p_param->scan_param_1m.scan_wd;
            p_cmd->phy[phy_idx].con_intv_min  = p_param->conn_param_1m.conn_intv_min;
            p_cmd->phy[phy_idx].con_intv_max  = p_param->conn_param_1m.conn_intv_max;
            p_cmd->phy[phy_idx].ce_len_min    = p_param->conn_param_1m.ce_len_min;
            p_cmd->phy[phy_idx].ce_len_max    = p_param->conn_param_1m.ce_len_max;
            p_cmd->phy[phy_idx].con_latency   = p_param->conn_param_1m.conn_latency;
            p_cmd->phy[phy_idx].superv_to     = p_param->conn_param_1m.supervision_to;

            // Increase counter
            phy_idx++;
        }

        if (p_param->prop & GAPM_INIT_PROP_2M_BIT)
        {
            p_cmd->phy[phy_idx].scan_interval = 0;
            p_cmd->phy[phy_idx].scan_window   = 0;
            p_cmd->phy[phy_idx].con_intv_min  = p_param->conn_param_2m.conn_intv_min;
            p_cmd->phy[phy_idx].con_intv_max  = p_param->conn_param_2m.conn_intv_max;
            p_cmd->phy[phy_idx].ce_len_min    = p_param->conn_param_2m.ce_len_min;
            p_cmd->phy[phy_idx].ce_len_max    = p_param->conn_param_2m.ce_len_max;
            p_cmd->phy[phy_idx].con_latency   = p_param->conn_param_2m.conn_latency;
            p_cmd->phy[phy_idx].superv_to     = p_param->conn_param_2m.supervision_to;

            // Increase counter
            phy_idx++;
        }

        if (p_param->prop & GAPM_INIT_PROP_CODED_BIT)
        {
            p_cmd->phy[phy_idx].scan_interval = p_param->scan_param_coded.scan_intv;
            p_cmd->phy[phy_idx].scan_window   = p_param->scan_param_coded.scan_wd;
            p_cmd->phy[phy_idx].con_intv_min  = p_param->conn_param_coded.conn_intv_min;
            p_cmd->phy[phy_idx].con_intv_max  = p_param->conn_param_coded.conn_intv_max;
            p_cmd->phy[phy_idx].ce_len_min    = p_param->conn_param_coded.ce_len_min;
            p_cmd->phy[phy_idx].ce_len_max    = p_param->conn_param_coded.ce_len_max;
            p_cmd->phy[phy_idx].con_latency   = p_param->conn_param_coded.conn_latency;
            p_cmd->phy[phy_idx].superv_to     = p_param->conn_param_coded.supervision_to;
        }

        // Send the command
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, event, cmp_evt_cb);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Send a LE Create Connection Cancel command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_init_handler function.
 *
 *
 * @param[in] p_actv       Pointer to the activity structure
 * @param[in] event        Event type receive that induce procedure state transition
 ****************************************************************************************
 */
__STATIC uint16_t gapm_init_send_hci_le_create_con_cancel_cmd(gapm_init_actv_t *p_actv, uint8_t event)
{
    return HL_HCI_BASIC_CMD_SEND(HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE, event, gapm_init_hci_cmd_cmp_handler);
}

/*
 * PROCEDURE TRANSITION - State machine
 ****************************************************************************************
 */

/// Initiating start procedure state machine
__STATIC uint16_t gapm_init_proc_start_transition(gapm_init_actv_t* p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                                  uint16_t status, bool* p_finished)
{
    *p_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                // Check if private address must be configured
                if ((gapm_env.scan_init_own_addr_type != GAPM_STATIC_ADDR) || GETB(gapm_env.priv_cfg, GAPM_PRIV_CFG_PRIV_ADDR))
                {
                    // set random address only if scan isn't active
                    if(gapm_env.scan_actv_idx == GAP_INVALID_ACTV_IDX)
                    {
                        status = gapm_le_actv_addr_send_hci_le_set_rand_addr_cmd(GAPM_INIT_PROC_START_HCI_SET_RAND_ADDR_CMP,
                                                                         (hl_hci_cmd_evt_func_t)gapm_init_hci_cmd_cmp_handler);
                        break;
                    }
                }
            }
            // no break;
            case GAPM_INIT_PROC_START_HCI_SET_RAND_ADDR_CMP:
            {
                status = gapm_init_send_hci_le_ext_create_con_cmd(p_actv, &(p_actv->param),
                                                                  GAPM_INIT_PROC_START_HCI_CREATE_CON_CMP,
                                                                  (hl_hci_cmd_evt_func_t)gapm_init_hci_cmd_cmp_handler);
            } break;
            case GAPM_INIT_PROC_START_HCI_CREATE_CON_CMP:
            default:
            {
                *p_finished = true;
            } break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        // Init has not been started properly
        gapm_init_stopped(p_actv);
        *p_finished = true;
    }

    return (status);
}


/// Initiating stop procedure state machine
__STATIC uint16_t gapm_init_stop_transition(gapm_init_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                            uint16_t status, bool* p_finished)
{
    *p_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                if (p_actv->conidx == GAP_INVALID_CONIDX)
                {
                    // Send a LE Set Extended Scan Enable command (Disable) to the controller
                    status = gapm_init_send_hci_le_create_con_cancel_cmd(p_actv, HL_PROC_FINISHED);
                }
                else
                {
                    // disconnect
                    status = gapc_disconnect(p_actv->conidx, p_actv->hdr.idx, LL_ERR_REMOTE_USER_TERM_CON,
                                             gapm_init_disconnect_cmp_cb);
                }
            }
            break;
            default:
            {
                *p_finished = true;
                gapm_init_stopped(p_actv);
                p_proc->event_status = p_actv->stop_status;
            } break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        *p_finished = true;
    }

    return (status);
}

/// Initiating delete procedure state machine
__STATIC uint16_t gapm_init_delete_transition(gapm_init_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                              uint16_t status, bool* p_finished)
{
    // Nothing special to be done
    *p_finished = true;
    return (status);
}


/// Initiating activity procedure address renew transition state machine
__STATIC uint16_t gapm_init_addr_renew_transition(gapm_init_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                                  uint16_t status, bool* p_finished)
{
    *p_finished = true;

    // just inform upper layer software about new address
    if(gapm_env.scan_init_own_addr_type != GAPM_STATIC_ADDR)
    {
        gapm_le_actv_send_new_bdaddr(&(p_actv->hdr), &(gapm_env.scan_init_rand_addr));
    }

    return (status);
}
/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/// @brief Handle automatic connection establishment timeout
void gapm_auto_conn_to_handler(void* p_env)
{
    gapm_init_actv_t *p_actv = (gapm_init_actv_t *)gapm_actv_get(gapm_env.init_actv_idx);

    // check that message is for expected  activity, else ignore
    if(p_actv)
    {
        p_actv->stop_status = GAP_ERR_TIMEOUT;
        gapm_actv_stop_quiet(p_actv->hdr.idx);
    }
}

/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
#endif //(HL_LE_CENTRAL)

uint16_t gapm_init_create(uint32_t dummy, uint8_t own_addr_type, const gapm_init_actv_cb_t* p_cbs, uint8_t* p_actv_idx)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_CENTRAL)
    do
    {
        // Allocated activity structure
        gapm_init_actv_t *p_actv;

        ASSERT_INFO(GAPM_HEAP_ENV_SIZE >= (sizeof(gapm_init_actv_t) + KE_HEAP_MEM_RESERVED),
                    GAPM_HEAP_ENV_SIZE, sizeof(gapm_init_actv_t));

        // Central role must be supported by the device
        if (!GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_CENTRAL))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Check that provided address type is valid
        if (!gapm_le_actv_addr_is_type_valid(own_addr_type))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // If controller privacy is enabled, non resolvable private address can not be used.
        if (GETB(gapm_env.priv_cfg, GAPM_PRIV_CFG_PRIV_EN) && (own_addr_type == GAPM_GEN_NON_RSLV_ADDR))
        {
            status = GAP_ERR_PRIVACY_CFG_PB;
            break;
        }

        // Allocate an activity structure
        status = gapm_actv_create(GAPM_ACTV_TYPE_INIT, GAPM_INIT_TYPE_DIRECT_CONN_EST, dummy, sizeof(gapm_init_actv_t),
                                  &gapm_init_actv_itf, (const gapm_actv_cb_t*) p_cbs, (gapm_actv_t**) &p_actv);
        if(status != GAP_ERR_NO_ERROR) break;
        p_actv->info_bf = 0;
        gapm_actv_created(&(p_actv->hdr));

        *p_actv_idx = p_actv->hdr.idx;

        co_time_timer_init(&(p_actv->auto_con_to_timer), gapm_auto_conn_to_handler, NULL);

        gapm_le_actv_addr_scan_init_type_set(own_addr_type);
    } while (0);
    #endif //(HL_LE_CENTRAL)

    return (status);
}


uint16_t gapm_init_start(uint8_t actv_idx, const gapm_init_param_t* p_param) // TODO [NATIVE API] create several functions
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_CENTRAL)
    do
    {
        gapm_init_actv_t* p_actv = (gapm_init_actv_t*) gapm_actv_get(actv_idx);
        gapm_actv_proc_t* p_proc;

        if((p_actv == NULL) || (p_actv->hdr.type != GAPM_ACTV_TYPE_INIT))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        if(p_param->type == GAPM_INIT_TYPE_NAME_DISC)
        {
            #if (BLE_GATT_CLI)
            gapm_init_actv_cb_t* p_cbs = (gapm_init_actv_cb_t*) p_actv->hdr.p_cbs;
            if(p_cbs->peer_name == NULL)
            {
                status = GAP_ERR_MISSING_CALLBACK;
                break; // Not supported
            }
            #else
            status = GAP_ERR_NOT_SUPPORTED;
            break;
            #endif // (BLE_GATT_CLI)
        }

        // Cannot have two initiating procedures in parallel
        if (gapm_env.init_actv_idx != GAPM_ACTV_INVALID_IDX)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Check if a new connection can be established
        if ((gapm_env.connections + gapm_env.nb_connect_actvs) >= HOST_CONNECTION_MAX)
        {
            status = GAP_ERR_INSUFF_RESOURCES;
            break;
        }

        // Check provided scan parameters
        status = gapm_init_check_param(p_param);
        if(status != GAP_ERR_NO_ERROR) break;

        // create start procedure
        status = gapm_actv_start(&(p_actv->hdr), sizeof(gapm_actv_proc_t), &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        // Configure the activity
        p_actv->hdr.subtype = p_param->type;

        if (p_param->type == GAPM_INIT_TYPE_AUTO_CONN_EST)
        {
            // Keep number of connection to be established in mind
            p_actv->nb_con = gapm_env.nb_dev_wl;

            // Start timeout timer for automatic connection
            if (p_param->conn_to > 0)
            {
                co_time_timer_set(&(p_actv->auto_con_to_timer), 10*p_param->conn_to);
            }
        }
        else
        {
            p_actv->nb_con = 1;
        }

        p_actv->param          = *p_param;
        p_actv->stop_status    = GAP_ERR_NO_ERROR;
        p_actv->conidx         = GAP_INVALID_CONIDX;
        gapm_env.init_actv_idx = actv_idx;
        gapm_env.nb_connect_actvs++;
    } while (0);
    #endif // (HL_LE_CENTRAL)

    return (status);
}

/// @} GAPM_INIT
