/**
 ****************************************************************************************
 *
 * @file llm_ch_scan.c
 *
 * @brief LLM Channel Scanning source file
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLMCHSCAN
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration
#if (BLE_OBSERVER || BLE_BROADCASTER)
#include <string.h>

#include "ke_task.h"        // kernel task definitions

#include "llm.h"            // link layer manager definitions
#include "lld.h"            // link layer driver definitions

#include "llm_int.h"        // link layer manager internal definitions

#if HCI_PRESENT
#include "hci.h"            // host controller interface
#endif //HCI_PRESENT


/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */
KE_MSG_HANDLER_NO_STATIC(lld_ch_scan_end_ind, struct lld_ch_scan_end_ind)
{
    // Send the complete event
    struct hci_vs_le_ch_scan_end_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_VS_LE_CH_SCAN_END_CMD_OPCODE, hci_vs_le_ch_scan_end_cmd_cmp_evt);
    event->status = param->status;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}


/*
 * HCI COMMAND HANDLERS
 ****************************************************************************************
 */

int ROM_VT_FUNC(hci_vs_le_ch_scan_cmd_handler)(struct hci_vs_le_ch_scan_cmd const *param, uint16_t opcode)
{
    // Status returned in the command complete event
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    uint8_t act_id;

    do
    {
        struct lld_ch_scan_params ch_scan_params;

        // Ensure Channel Scanning is not already active
        for (act_id = 0; act_id < BLE_ACTIVITY_MAX; act_id++)
        {
            if (llm_env.act_info[act_id].state == LLM_CH_SCAN_EN)
                break;
        }
        if(act_id != BLE_ACTIVITY_MAX)
            break;

        status = llm_activity_free_get(&act_id);

        // If not possible to start a new channel scanning activity, reject the command
        if(status != CO_ERROR_NO_ERROR)
            break;

        // Assign the allocated activity identifier to initiating
        llm_env.act_info[act_id].state = LLM_CH_SCAN_EN;

        ch_scan_params.scan_win_duration = param->scan_win_duration;
        ch_scan_params.scan_duration_min = param->scan_duration_min;
        ch_scan_params.scan_duration_max = param->scan_duration_max;
        ch_scan_params.intv = param->intv;

        status = lld_ch_scan_start(act_id, &ch_scan_params);
    } while(0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int ROM_VT_FUNC(hci_vs_le_ch_scan_end_cmd_handler)(void const *param, uint16_t opcode)
{
    uint8_t status = lld_ch_scan_stop();
    int act_id;

    // If not present, try to allocate an activity identifier
    for (act_id = 0; act_id <= BLE_ACTIVITY_MAX; act_id++)
    {
        if (llm_env.act_info[act_id].state == LLM_CH_SCAN_EN)
            break;
    }
    if (act_id != BLE_ACTIVITY_MAX)
        llm_env.act_info[act_id].state = LLM_FREE;

    if (status != CO_ERROR_NO_ERROR)
    {
        // Send the complete event
        struct hci_vs_le_ch_scan_end_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_VS_LE_CH_SCAN_END_CMD_OPCODE, hci_vs_le_ch_scan_end_cmd_cmp_evt);
        event->status = status;
        hci_send_2_host(event);
    }

    return (KE_MSG_CONSUMED);
}

#endif //(BLE_OBSERVER || BLE_BROADCASTER)
/// @} LLMCHSCAN
