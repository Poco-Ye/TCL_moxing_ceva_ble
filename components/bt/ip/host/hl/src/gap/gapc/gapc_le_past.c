/**
 ****************************************************************************************
 *
 * @file gapc_past.c
 *
 * @brief GGeneric Access Profile - Periodic Advertising Sync Transfer implementation.
 *
 * Copyright (C) RivieraWaves 2009-2018
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPC_PAST Generic Access Profile - Periodic Advertising Sync Transfer
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_GAPC)
#include "gapc_le_con.h"
#include "gapm_le_per_sync.h"
#include "gapm_le_adv.h"
#include "../gap_int.h"

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

/// PAST procedure object
typedef struct gapc_le_past_proc
{
    /// procedure inheritance
    gapc_proc_simple_t hdr;
    /// Periodic Advertising or Periodic Sync activity handle
    uint16_t           actv_handle;
    /// A value provided by application
    uint16_t           service_data;
    /// True if a periodic sync activity
    bool               is_per_sync_actv;
} gapc_le_past_proc_t;


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
/// LE Ping procedure state machine granted function
__STATIC uint16_t gapc_le_past_proc_granted(uint8_t conidx, gapc_le_past_proc_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    // Periodic advertising activity
    #if (HL_LE_BROADCASTER)
    if(!p_proc->is_per_sync_actv)
    {
        struct hci_le_per_adv_set_info_transf_cmd* p_hci_cmd =
                HL_HCI_CMD_ALLOC(HCI_LE_PER_ADV_SET_INFO_TRANSF_CMD_OPCODE, hci_le_per_adv_set_info_transf_cmd);

        if(p_hci_cmd != NULL)
        {
            p_hci_cmd->conhdl    = gapc_get_conhdl(conidx);
            p_hci_cmd->adv_hdl   = p_proc->actv_handle;
            p_hci_cmd->serv_data = p_proc->service_data;
            HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_FINISHED), gapc_proc_default_hci_cmp_evt_handler);
            status = GAP_ERR_NO_ERROR;
        }
    }
    else
    #endif // (HL_LE_BROADCASTER)
    {
        struct hci_le_per_adv_sync_transf_cmd* p_hci_cmd =
                HL_HCI_CMD_ALLOC(HCI_LE_PER_ADV_SYNC_TRANSF_CMD_OPCODE, hci_le_per_adv_sync_transf_cmd);

        if(p_hci_cmd != NULL)
        {
        p_hci_cmd->conhdl    = gapc_get_conhdl(conidx);
        p_hci_cmd->sync_hdl  = p_proc->actv_handle;
        p_hci_cmd->serv_data = p_proc->service_data;
        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_FINISHED), gapc_proc_default_hci_cmp_evt_handler);
            status = GAP_ERR_NO_ERROR;
        }
    }


    return (status);
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_le_past_proc_itf =
{
        .granted  = (gapc_proc_simple_granted_cb) gapc_le_past_proc_granted,
        .finished = gapc_proc_simple_default_finished_cb,
};

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapc_le_per_adv_sync_transfer(uint8_t conidx, uint32_t dummy, uint8_t actv_idx, uint16_t service_data,
                                       gapc_proc_cmp_cb cmp_cb)
{
    uint8_t  act_type;
    uint8_t  act_sub_type;
    bool     is_per_sync_actv = false;
    uint16_t actv_handle = GAP_INVALID_CONHDL;

    // retrieve information about non-connected activity
    uint16_t  status = gapm_actv_info_get(actv_idx, &act_type, &act_sub_type);

    if(status == GAP_ERR_NO_ERROR)
    {
        // Periodic sync activity
        if(act_type == GAPM_ACTV_TYPE_PER_SYNC)
        {
            actv_handle = gapm_per_sync_hdl_get(actv_idx);
            is_per_sync_actv = true;
        }
        // Periodic advertising activity
        #if (HL_LE_BROADCASTER)
        else if((act_type == GAPM_ACTV_TYPE_ADV) && (act_sub_type == GAPM_ADV_TYPE_PERIODIC))
        {
            actv_handle      = gapm_adv_hdl_get(actv_idx);
        }
        #endif // (HL_LE_BROADCASTER)
        else
        {
            status = GAP_ERR_NOT_SUPPORTED;
        }
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        gapc_le_past_proc_t* p_proc;

        status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_le_past_proc_t), &gapc_le_past_proc_itf,
                                         (gapc_proc_simple_t**) &p_proc);
        if(status == GAP_ERR_NO_ERROR)
        {
            p_proc->actv_handle      = actv_handle;
            p_proc->service_data     = service_data;
            p_proc->is_per_sync_actv = is_per_sync_actv;
        }
    }

    return (status);
}

uint8_t gapc_past_actv_idx_get(uint8_t conidx)
{
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
    return ((p_con != NULL) ?  p_con->past_actv_idx : GAP_INVALID_ACTV_IDX);
}

void gapc_past_actv_idx_set(uint8_t conidx, uint8_t actv_idx)
{
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
    if(p_con != NULL)
    {
        p_con->past_actv_idx = actv_idx;
    }
}

#endif // (BLE_GAPC)
/// @} GAPC_PAST
