/**
 ****************************************************************************************
 *
 * @file gapc_le_cte.c
 *
 * @brief Generic Access Profile - Constant Tone Extension implementation.
 *
 * Copyright (C) RivieraWaves 2009-2018
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPC_CTE Generic Access Profile - Constant Tone Extension
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_GAPC)
#if (BLE_AOA | BLE_AOD)

#include "gapc_le_con.h"
#include "hl_hci.h"
#include "hci.h"

#include "co_math.h"
#include <string.h>

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// Modification of CTE transmission parameters procedure object
typedef struct gapc_le_cte_tx_cfg_proc
{
    /// procedure inheritance
    gapc_proc_simple_t hdr;
    /// CTE types (bit0: AOA | bit1: AOD-1us | bit2: AOD-2us) (see enum #gap_cte_type_bf)
    uint8_t            cte_types;
    /// Length of switching pattern (number of antenna IDs in the pattern)
    uint8_t            switching_pattern_len;
    /// Antenna IDs
    uint8_t            antenna_id[__ARRAY_EMPTY];
} gapc_le_cte_tx_cfg_proc_t;

/// Modification of CTE reception parameters procedure object
typedef struct gapc_le_cte_rx_cfg_proc
{
    /// procedure inheritance
    gapc_proc_simple_t hdr;
    /// Sampling enable
    bool               sample_enable;
    /// Slot durations (1: 1us | 2: 2us)
    uint8_t            slot_dur;
    /// Length of switching pattern (number of antenna IDs in the pattern)
    uint8_t            switching_pattern_len;
    /// Antenna IDs
    uint8_t            antenna_id[__ARRAY_EMPTY];
} gapc_le_cte_rx_cfg_proc_t;

/// Control of CTE request procedure object
typedef struct gapc_le_cte_req_ctrl_proc
{
    /// procedure inheritance
    gapc_proc_simple_t hdr;
    /// Enable
    uint8_t            enable;
    /// CTE request interval (in number of connection events)
    uint16_t           interval;
    /// Requested CTE length (in 8us unit)
    uint8_t            cte_length;
    /// Requested CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us)
    uint8_t            cte_type;
} gapc_le_cte_req_ctrl_proc_t;

/// Control of CTE response procedure object
typedef struct gapc_le_cte_rsp_ctrl_proc
{
    /// procedure inheritance
    gapc_proc_simple_t hdr;
    /// Enable
    uint8_t            enable;
} gapc_le_cte_rsp_ctrl_proc_t;


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * MACROS
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
__STATIC uint16_t gapc_le_cte_tx_cfg_proc_granted(uint8_t conidx, gapc_le_cte_tx_cfg_proc_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    struct hci_le_set_con_cte_tx_param_cmd *p_hci_cmd =
                HL_HCI_CMD_ALLOC(HCI_LE_SET_CON_CTE_TX_PARAM_CMD_OPCODE, hci_le_set_con_cte_tx_param_cmd);

    if(p_hci_cmd != NULL)
    {
        p_hci_cmd->conhdl                 = gapc_get_conhdl(conidx);
        p_hci_cmd->cte_types              = p_proc->cte_types;
        p_hci_cmd->switching_pattern_len  = p_proc->switching_pattern_len;

        memcpy(p_hci_cmd->antenna_id, p_proc->antenna_id, co_min(MAX_SWITCHING_PATTERN_LEN, p_proc->switching_pattern_len));

        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_FINISHED),
                                gapc_proc_default_hci_cmp_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_le_cte_tx_cfg_proc_itf =
{
        .granted  = (gapc_proc_simple_granted_cb) gapc_le_cte_tx_cfg_proc_granted,
        .finished = gapc_proc_simple_default_finished_cb,
};


///Procedure state machine granted function
__STATIC uint16_t gapc_le_cte_rx_cfg_proc_granted(uint8_t conidx, gapc_le_cte_rx_cfg_proc_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    struct hci_le_set_con_cte_rx_param_cmd *p_hci_cmd =
                HL_HCI_CMD_ALLOC(HCI_LE_SET_CON_CTE_RX_PARAM_CMD_OPCODE, hci_le_set_con_cte_rx_param_cmd);

    if(p_hci_cmd != NULL)
    {
        p_hci_cmd->conhdl                  = gapc_get_conhdl(conidx);
        p_hci_cmd->sampl_en                = p_proc->sample_enable;
        p_hci_cmd->slot_dur                = p_proc->slot_dur;
        p_hci_cmd->switching_pattern_len   = p_proc->switching_pattern_len;
        memcpy(p_hci_cmd->antenna_id, p_proc->antenna_id, co_min(MAX_SWITCHING_PATTERN_LEN, p_proc->switching_pattern_len));

        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_FINISHED),
                                gapc_proc_default_hci_cmp_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_le_cte_rx_cfg_proc_itf =
{
        .granted  = (gapc_proc_simple_granted_cb) gapc_le_cte_rx_cfg_proc_granted,
        .finished = gapc_proc_simple_default_finished_cb,
};

///Procedure state machine granted function
__STATIC uint16_t gapc_le_cte_req_ctrl_proc_granted(uint8_t conidx, gapc_le_cte_req_ctrl_proc_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    struct hci_le_con_cte_req_en_cmd *p_hci_cmd =
                HL_HCI_CMD_ALLOC(HCI_LE_CON_CTE_REQ_EN_CMD_OPCODE, hci_le_con_cte_req_en_cmd);

    if(p_hci_cmd != NULL)
    {
        p_hci_cmd->conhdl       = gapc_get_conhdl(conidx);
        p_hci_cmd->en           = p_proc->enable;
        p_hci_cmd->cte_req_intv = p_proc->interval;
        p_hci_cmd->req_cte_len  = p_proc->cte_length;
        p_hci_cmd->req_cte_type = p_proc->cte_type;

        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_FINISHED),
                                gapc_proc_default_hci_cmp_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_le_cte_req_ctrl_proc_itf =
{
        .granted  = (gapc_proc_simple_granted_cb) gapc_le_cte_req_ctrl_proc_granted,
        .finished = gapc_proc_simple_default_finished_cb,
};

///Procedure state machine granted function
__STATIC uint16_t gapc_le_cte_rsp_ctrl_proc_granted(uint8_t conidx, gapc_le_cte_rsp_ctrl_proc_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    struct hci_le_con_cte_rsp_en_cmd *p_hci_cmd =
                HL_HCI_CMD_ALLOC(HCI_LE_CON_CTE_RSP_EN_CMD_OPCODE, hci_le_con_cte_rsp_en_cmd);
    if(p_hci_cmd != NULL)
    {
        p_hci_cmd->conhdl       = gapc_get_conhdl(conidx);
        p_hci_cmd->en           = p_proc->enable;

        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_FINISHED),
                                gapc_proc_default_hci_cmp_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}


/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_le_cte_rsp_ctrl_proc_itf =
{
        .granted  = (gapc_proc_simple_granted_cb) gapc_le_cte_rsp_ctrl_proc_granted,
        .finished = gapc_proc_simple_default_finished_cb,
};

/*
 * HCI HANDLERS FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
/// @brief Handles reception of an IQ report event over a BLE connection
void gapc_hci_le_con_iq_report_evt_handler(uint8_t evt_code, struct hci_le_con_iq_report_evt const *p_evt)
{
    if(gapc_env.p_le_cte_cbs != NULL)
    {
        uint8_t conidx = gapc_get_conidx(p_evt->conhdl);
        gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

        if (p_con != NULL)
        {
            gapc_iq_report_info_t report;
            report.rx_phy           = p_evt->rx_phy;
            report.channel_idx      = p_evt->data_channel_idx;
            report.rssi             = p_evt->rssi;
            report.rssi_antenna_id  = p_evt->rssi_antenna_id;
            report.cte_type         = p_evt->cte_type;
            report.slot_dur         = p_evt->slot_dur;
            report.pkt_status       = p_evt->pkt_status;
            report.con_evt_cnt      = p_evt->con_evt_cnt;

            gapc_env.p_le_cte_cbs->iq_report_received(conidx, p_con->hdr.dummy, &report, p_evt->sample_cnt,
                                                   (const gap_iq_sample_t*) p_evt->iq_sample);
        }
    }
}

/// @brief Handles reception of an Request Failed event over a BLE connection
void gapc_hci_le_cte_req_failed_evt_handler(uint8_t evt_code, struct hci_le_cte_req_failed_evt const *p_evt)
{
    if(gapc_env.p_le_cte_cbs != NULL)
    {
        uint8_t conidx = gapc_get_conidx(p_evt->conhdl);
        gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

        if (p_con != NULL)
        {
            gapc_env.p_le_cte_cbs->request_failed_event(conidx, p_con->hdr.dummy, RW_ERR_HCI_TO_HL(p_evt->status));
        }
    }
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapc_le_cte_set_callbacks(const gapc_le_cte_cb_t* p_cbs)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    if((p_cbs != NULL) && (p_cbs->iq_report_received != NULL) && (p_cbs->request_failed_event != NULL))
    {
        gapc_env.p_le_cte_cbs = p_cbs;
    }
    else
    {
        status = GAP_ERR_INVALID_PARAM;
    }

    return (status);
}


uint16_t gapc_le_cte_tx_configure(uint8_t conidx, uint32_t dummy, uint8_t cte_types, uint8_t switching_pattern_len,
                                  const uint8_t* p_antenna_id, gapc_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    if((p_antenna_id == NULL) && (switching_pattern_len != 0))
    {
         status = GAP_ERR_INVALID_PARAM;
    }
    else if(gapc_env.p_le_cte_cbs == NULL)
    {
        status = GAP_ERR_MISSING_CALLBACK;
    }
    else
    {
        gapc_le_cte_tx_cfg_proc_t* p_proc;
        uint8_t len = co_min(MAX_SWITCHING_PATTERN_LEN, switching_pattern_len);
        status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_le_cte_tx_cfg_proc_t) + len,
                                         &gapc_le_cte_tx_cfg_proc_itf, (gapc_proc_simple_t**) &p_proc);
        if(status == GAP_ERR_NO_ERROR)
        {
            p_proc->cte_types             = cte_types;
            p_proc->switching_pattern_len = switching_pattern_len;
            memcpy(p_proc->antenna_id, p_antenna_id, len);
        }
    }

    return (status);
}

uint16_t gapc_le_cte_rx_configure(uint8_t conidx, uint32_t dummy, bool sample_enable, uint8_t slot_dur,
                                  uint8_t switching_pattern_len, const uint8_t* p_antenna_id, gapc_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    if((p_antenna_id == NULL) && (switching_pattern_len != 0))
    {
         status = GAP_ERR_INVALID_PARAM;
    }
    else if(gapc_env.p_le_cte_cbs == NULL)
    {
        status = GAP_ERR_MISSING_CALLBACK;
    }
    else
    {
        gapc_le_cte_rx_cfg_proc_t* p_proc;
        uint8_t len = co_min(MAX_SWITCHING_PATTERN_LEN, switching_pattern_len);

        status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_le_cte_rx_cfg_proc_t) + len,
                                         &gapc_le_cte_rx_cfg_proc_itf, (gapc_proc_simple_t**) &p_proc);
        if(status == GAP_ERR_NO_ERROR)
        {
            p_proc->sample_enable         = sample_enable;
            p_proc->slot_dur              = slot_dur;
            p_proc->switching_pattern_len = switching_pattern_len;
            memcpy(p_proc->antenna_id, p_antenna_id, len);
        }
    }

    return (status);
}

uint16_t gapc_le_cte_request_control(uint8_t conidx, uint32_t dummy, bool enable, uint16_t interval,
                                     uint8_t cte_length, uint8_t cte_type, gapc_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_le_cte_req_ctrl_proc_t* p_proc;

    if(gapc_env.p_le_cte_cbs == NULL)
    {
        status = GAP_ERR_MISSING_CALLBACK;
    }
    else
    {
        status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_le_cte_req_ctrl_proc_t),
                                         &gapc_le_cte_req_ctrl_proc_itf, (gapc_proc_simple_t**) &p_proc);
        if(status == GAP_ERR_NO_ERROR)
        {
            p_proc->enable       = enable;
            p_proc->interval     = interval;
            p_proc->cte_length   = cte_length;
            p_proc->cte_type     = cte_type;
        }
    }

    return (status);
}

uint16_t gapc_le_cte_response_control(uint8_t conidx, uint32_t dummy, bool enable, gapc_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_le_cte_rsp_ctrl_proc_t* p_proc;

    if(gapc_env.p_le_cte_cbs == NULL)
    {
        status = GAP_ERR_MISSING_CALLBACK;
    }
    else
    {
        status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_le_cte_rsp_ctrl_proc_t),
                                         &gapc_le_cte_rsp_ctrl_proc_itf, (gapc_proc_simple_t**) &p_proc);
        if(status == GAP_ERR_NO_ERROR)
        {
            p_proc->enable           = enable;
        }
    }

    return (status);
}

#endif // (BLE_AOA | BLE_AOD)
#endif // (BLE_GAPC)
/// @} GAPC_CTE
