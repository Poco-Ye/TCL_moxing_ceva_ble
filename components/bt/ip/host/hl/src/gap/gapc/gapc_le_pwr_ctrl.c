/**
 ****************************************************************************************
 *
 * @file gapc_le_pwr_ctrl.c
 *
 * @brief GAPC Generic Access Profile Connection - LE Power Control
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPC Generic Access Profile Connection - LE Power Control
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_GAPC)
#if (BLE_PWR_CTRL)

#include "arch.h"

#include "gap.h"
#include "gapm.h"
#include "gapc_le_con.h"

#include "hl_hci.h"
#include "hci.h"

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

/// Control TX Power reports procedure object
typedef struct gapc_le_tx_pwr_report_ctrl_proc
{
    /// procedure inheritance
    gapc_proc_simple_t hdr;
    /// True to enable local power changes reporting, false to disable.
    bool               local_en;
    /// True to enable remote power changes reporting, false to disable.
    bool               remote_en;
} gapc_le_tx_pwr_report_ctrl_proc_t;

/// Path Loss Threshold procedure object
typedef struct gapc_le_path_loss_ctrl_proc
{
    /// procedure inheritance
    hl_proc_t        hdr;
    /// Callback to execute once command completes
    gapc_proc_cmp_cb cmp_cb;
    /// Dummy parameter provided by upper layer SW
    uint32_t         dummy;
    /// Connection index
    uint8_t          conidx;
    /// 1 To Enable reporting, 0 to disable.
    uint8_t          enable;
    /// High threshold (dB)
    uint8_t          high_threshold;
    /// High hysteresis (dB)
    uint8_t          high_hysteresis;
    /// Low threshold (dB)
    uint8_t          low_threshold;
    /// Low hysteresis (dB)
    uint8_t          low_hysteresis;
    /// Min time spent (conn events)
    uint16_t         min_time;
} gapc_le_path_loss_ctrl_proc_t;


/*
 * LOCAL FUNCTIONS DECLARATIONS
 ****************************************************************************************
 */
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/*
 * PROCEDURE STATE MACHINE
 ****************************************************************************************
 */
///Procedure state machine granted function
__STATIC uint16_t gapc_le_tx_pwr_report_ctrl_proc_granted(uint8_t conidx, gapc_le_tx_pwr_report_ctrl_proc_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    struct hci_le_set_tx_power_rep_en_cmd *p_hci_cmd =
                HL_HCI_CMD_ALLOC(HCI_LE_SET_TX_POWER_REP_EN_CMD_OPCODE, hci_le_set_tx_power_rep_en_cmd);

    if(p_hci_cmd != NULL)
    {
        p_hci_cmd->conhdl    = gapc_get_conhdl(conidx);
        p_hci_cmd->local_en  = p_proc->local_en;
        p_hci_cmd->remote_en = p_proc->remote_en;


        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_FINISHED),
                                gapc_proc_default_hci_cmp_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_le_tx_pwr_report_ctrl_proc_itf =
{
        .granted  = (gapc_proc_simple_granted_cb) gapc_le_tx_pwr_report_ctrl_proc_granted,
        .finished = gapc_proc_simple_default_finished_cb,
};


/// Path Loss Threshold procedure
__STATIC bool gapc_le_path_loss_ctrl_proc_transistion(gapc_le_path_loss_ctrl_proc_t* p_proc, uint8_t event, uint16_t status)
{
    bool is_finished = false;
    uint8_t conidx = p_proc->conidx;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                if(p_proc->enable)
                {
                    struct hci_le_set_path_loss_rep_param_cmd *p_hci_cmd =
                            HL_HCI_CMD_ALLOC(HCI_LE_SET_PATH_LOSS_REP_PARAM_CMD_OPCODE, hci_le_set_path_loss_rep_param_cmd);

                    if(p_hci_cmd == NULL)
                    {
                        status = GAP_ERR_INSUFF_RESOURCES;
                        break;
                    }

                    p_hci_cmd->conhdl       = gapc_get_conhdl(conidx);
                    p_hci_cmd->hi_thr       = p_proc->high_threshold;
                    p_hci_cmd->hi_hyst      = p_proc->high_hysteresis;
                    p_hci_cmd->lo_thr       = p_proc->low_threshold;
                    p_hci_cmd->lo_hyst      = p_proc->low_hysteresis;
                    p_hci_cmd->min_time     = p_proc->min_time;

                    HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_CONTINUE),
                                            gapc_proc_default_hci_cmp_evt_handler);
                    break;
                }
            }
            // no break
            case HL_PROC_CONTINUE:
            {
                struct hci_le_set_path_loss_rep_en_cmd* p_hci_cmd =
                        HL_HCI_CMD_ALLOC(HCI_LE_SET_PATH_LOSS_REP_EN_CMD_OPCODE, hci_le_set_path_loss_rep_en_cmd);

                if(p_hci_cmd == NULL)
                {
                    status = GAP_ERR_INSUFF_RESOURCES;
                    break;
                }

                p_hci_cmd->conhdl = gapc_get_conhdl(conidx);
                p_hci_cmd->en     = p_proc->enable;
                HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_FINISHED),
                                        gapc_proc_default_hci_cmp_evt_handler);
            } break;
            default:
            {
                is_finished = true;
            } break;
        }
    }

    // error handling
    if(status != GAP_ERR_NO_ERROR)
    {
        is_finished = true;
    }

    if(is_finished)
    {
        // inform application that procedure is over
        p_proc->cmp_cb(conidx, p_proc->dummy, status);
    }

    return (is_finished);
}

/// Path Loss Threshold procedure state machine structure
__STATIC const hl_proc_itf_t gapc_le_path_loss_ctrl_proc_itf =
{
    .transition  = (hl_proc_transition_cb) gapc_le_path_loss_ctrl_proc_transistion,
    .cleanup     = hl_proc_cleanup,
};


// ------------------ GET LOCAL TX POWER LEVEL -------------------


/// Get local TX power level procedure is granted
__STATIC uint16_t gapc_get_local_tx_power_level_proc_granted(uint8_t conidx, gapc_proc_info_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    struct hci_le_enh_rd_tx_pwr_lvl_cmd* p_hci_cmd =
            HL_HCI_CMD_ALLOC(HCI_LE_ENH_RD_TX_PWR_LVL_CMD_OPCODE, hci_le_enh_rd_tx_pwr_lvl_cmd);

    if(p_hci_cmd != NULL)
    {
        p_hci_cmd->conhdl = gapc_get_conhdl(conidx);
        p_hci_cmd->phy = p_proc->hci_cmd_info; // optional info

        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, HL_PROC_FINISHED, gapc_proc_info_default_hci_cmp_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

/// Get local TX power level procedure is finished
__STATIC void gapc_get_local_tx_power_level_proc_finished(uint8_t conidx, gapc_proc_info_t* p_proc, uint16_t status)
{
    gapc_le_get_local_tx_power_level_cmp_cb cmp_cb = (gapc_le_get_local_tx_power_level_cmp_cb) p_proc->hdr.cmp_cb;
    uint8_t phy             = 0;
    int8_t  power_level     = 0;
    int8_t  max_power_level = 0;

    if(status == GAP_ERR_NO_ERROR)
    {
        // procedure finished is called within hci complete event handler
        const struct hci_le_enh_rd_tx_pwr_lvl_cmd_cmp_evt* p_evt =
                (const struct hci_le_enh_rd_tx_pwr_lvl_cmd_cmp_evt*) p_proc->p_res_info;
        phy             = p_evt->phy;
        power_level     = p_evt->curr_tx_pwr_lvl;
        max_power_level = p_evt->max_tx_pwr_lvl;
    }

    cmp_cb(conidx, p_proc->hdr.dummy, status, phy, power_level, max_power_level);
}


/// Get local TX power level procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_get_local_tx_power_level_proc_itf =
{
    .granted  = (gapc_proc_simple_granted_cb)  gapc_get_local_tx_power_level_proc_granted,
    .finished = (gapc_proc_simple_finished_cb) gapc_get_local_tx_power_level_proc_finished,
};



// ------------------ GET PEER TX POWER LEVEL -------------------


/// Get peer TX power level procedure is granted
__STATIC uint16_t gapc_get_peer_tx_power_level_proc_granted(uint8_t conidx, gapc_proc_info_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    struct hci_le_rd_remote_tx_pwr_lvl_cmd* p_hci_cmd =
            HL_HCI_CMD_ALLOC(HCI_LE_RD_REMOTE_TX_PWR_LVL_CMD_OPCODE, hci_le_rd_remote_tx_pwr_lvl_cmd);

    if(p_hci_cmd != NULL)
    {
        p_hci_cmd->conhdl = gapc_get_conhdl(conidx);
        p_hci_cmd->phy = p_proc->hci_cmd_info; // optional info

        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, HL_PROC_CONTINUE, gapc_proc_default_hci_stat_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

/// Get peer TX power level procedure is finished
__STATIC void gapc_get_peer_tx_power_level_proc_finished(uint8_t conidx, gapc_proc_info_t* p_proc, uint16_t status)
{
    gapc_le_get_peer_tx_power_level_cmp_cb cmp_cb = (gapc_le_get_peer_tx_power_level_cmp_cb) p_proc->hdr.cmp_cb;
    uint8_t phy         = 0;
    int8_t  power_level = 0;
    uint8_t flags       = 0;

    if(status == GAP_ERR_NO_ERROR)
    {
        // procedure finished is called within hci complete event handler
        const struct hci_le_tx_power_rep_evt* p_evt = (const struct hci_le_tx_power_rep_evt*) p_proc->p_res_info;
        phy         = p_evt->phy;
        power_level = p_evt->tx_pwr;
        flags       = p_evt->flags;
    }

    cmp_cb(conidx, p_proc->hdr.dummy, status, phy, power_level, flags);
}


/// Get peer TX power level procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_get_peer_tx_power_level_proc_itf =
{
    .granted  = (gapc_proc_simple_granted_cb)  gapc_get_peer_tx_power_level_proc_granted,
    .finished = (gapc_proc_simple_finished_cb) gapc_get_peer_tx_power_level_proc_finished,
};

/*
 * HCI FUNCTION HANDLERS
 ****************************************************************************************
 */

/// @brief Handles reception of a TX power change event
void gapc_hci_le_tx_power_rep_evt_handler(uint8_t evt_code, struct hci_le_tx_power_rep_evt const *p_evt)
{
    // Current connection index
    uint8_t conidx = gapc_get_conidx(p_evt->conhdl);

    if(conidx != GAP_INVALID_CONIDX)
    {
        switch(p_evt->reason)
        {
            case BLE_PWR_HCI_REQ:
            {
                gapc_proc_info_default_hci_le_cmp_evt_handler(HL_PROC_FINISHED,
                                                              (const struct hci_basic_conhdl_le_cmd_cmp_evt*) p_evt);
            } break;

            case BLE_PWR_LOC_TX_CHG:
            case BLE_PWR_REM_TX_CHG:
            {
                uint8_t conidx = gapc_get_conidx(p_evt->conhdl);
                gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
                if(p_con != NULL)
                {
                    const gapc_le_power_cb_t* p_cbs = gapc_env.p_le_power_cbs;
                    p_cbs->tx_change_report(conidx, p_con->hdr.dummy, (p_evt->reason == BLE_PWR_LOC_TX_CHG),
                                            (const gapc_tx_power_report_t*) &(p_evt->phy));
                }
            } break;
            default: { /* Nothing to do */ } break;
        }
    }
}

/// @brief Handles reception of a path loss change event
void gapc_hci_le_path_loss_threshold_evt_handler(uint8_t evt_code, struct hci_le_path_loss_threshold_evt const *p_evt)
{
    uint8_t conidx = gapc_get_conidx(p_evt->conhdl);
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
    if(p_con != NULL)
    {
        const gapc_le_power_cb_t* p_cbs = gapc_env.p_le_power_cbs;
        p_cbs->path_loss_threshold_report(conidx, p_con->hdr.dummy, p_evt->curr_path_loss, p_evt->zone_entered);
    }
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapc_le_tx_pwr_report_ctrl(uint8_t conidx, uint32_t dummy, bool local_en, bool remote_en, gapc_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

    if(p_con != NULL)
    {
        if(gapc_env.p_le_power_cbs == NULL)
        {
            status = GAP_ERR_MISSING_CALLBACK;
        }
        else
        {
            gapc_le_tx_pwr_report_ctrl_proc_t* p_proc;

            status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_le_tx_pwr_report_ctrl_proc_t),
                                             &gapc_le_tx_pwr_report_ctrl_proc_itf, (gapc_proc_simple_t**) &p_proc);
            if(status == GAP_ERR_NO_ERROR)
            {
                p_proc->local_en  = local_en;
                p_proc->remote_en = remote_en;
            }
        }
    }

    return (status);
}

uint16_t gapc_le_path_loss_enable(uint8_t conidx, uint32_t dummy, uint8_t high_threshold, uint8_t high_hysteresis,
                                  uint8_t low_threshold, uint8_t low_hysteresis, uint16_t min_time,
                                  gapc_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    do
    {
        gapc_le_path_loss_ctrl_proc_t* p_proc;
        gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
        if(p_con == NULL) break;

        if(gapc_env.p_le_power_cbs == NULL)
        {
            status = GAP_ERR_MISSING_CALLBACK;
            break;
        }

        status = gapc_proc_le_create(conidx, sizeof(gapc_le_path_loss_ctrl_proc_t), &gapc_le_path_loss_ctrl_proc_itf,
                                     (hl_proc_t**) &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        p_proc->conidx          = conidx;
        p_proc->cmp_cb          = cmp_cb;
        p_proc->dummy           = dummy;
        p_proc->enable          = true;
        p_proc->high_threshold  = high_threshold;
        p_proc->high_hysteresis = high_hysteresis;
        p_proc->low_threshold   = low_threshold;
        p_proc->low_hysteresis  = low_hysteresis;
        p_proc->min_time        = min_time;
    } while(0);

    return (status);
}

uint16_t gapc_le_path_loss_disable(uint8_t conidx, uint32_t dummy, gapc_proc_cmp_cb cmp_cb)
{
    uint16_t status;
    do
    {
        gapc_le_path_loss_ctrl_proc_t* p_proc;

        if(cmp_cb == NULL)
        {
            status = GAP_ERR_MISSING_CALLBACK;
            break;
        }

        status = gapc_proc_le_create(conidx, sizeof(gapc_le_path_loss_ctrl_proc_t), &gapc_le_path_loss_ctrl_proc_itf,
                                     (hl_proc_t**) &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        p_proc->conidx = conidx;
        p_proc->cmp_cb = cmp_cb;
        p_proc->dummy  = dummy;
        p_proc->enable = false;
    } while(0);

    return (status);
}

uint16_t gapc_le_power_set_callbacks(const gapc_le_power_cb_t* p_cbs)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    if((p_cbs != NULL) && (p_cbs->path_loss_threshold_report != NULL) && (p_cbs->tx_change_report != NULL))
    {
        gapc_env.p_le_power_cbs = p_cbs;
    }
    else
    {
        status = GAP_ERR_INVALID_PARAM;
    }

    return (status);
}

uint16_t gapc_le_get_local_tx_power_level(uint8_t conidx, uint32_t dummy, uint8_t phy,
                                          gapc_le_get_local_tx_power_level_cmp_cb cmp_cb)
{
    return (gapc_proc_info_create(conidx, dummy, phy,
                                  &gapc_get_local_tx_power_level_proc_itf, (gapc_proc_cmp_cb) cmp_cb));
}

uint16_t gapc_le_get_peer_tx_power_level(uint8_t conidx, uint32_t dummy, uint8_t phy,
                                         gapc_le_get_peer_tx_power_level_cmp_cb cmp_cb)
{
    return (gapc_proc_info_create(conidx, dummy, phy,
                                  &gapc_get_peer_tx_power_level_proc_itf, (gapc_proc_cmp_cb) cmp_cb));
}
#endif // (BLE_PWR_CTRL)
#endif // (BLE_GAPC)
/// @} GAPC
