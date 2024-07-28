/**
 ****************************************************************************************
 *
 * @file gapm_le_test.c
 *
 * @brief Generic Access Profile Manager - LE Test mode control
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if(BLE_HOST_PRESENT && HOST_TEST_MODE)
#include "gapm_int.h"
#include "gapm_le_test.h"
#include "hl_hci.h"
#include "hci.h"
#include "co_math.h"
#include "string.h"


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


/// RX test activity start procedure structure
typedef struct gapm_le_test_rx_proc_start
{
    /// Inherited from default procedure object
    gapm_actv_proc_t      hdr;
    /// RX Test parameters
    gapm_le_test_rx_param_t  param;
    /// CTE parameters
    gapm_le_test_cte_param_t cte_param;
    /// Antenna ID array
    uint8_t               antenna_id[__ARRAY_EMPTY];
} gapm_le_test_rx_proc_start_t;


/// TX test activity start procedure structure
typedef struct gapm_le_test_tx_proc_start
{
    /// Inherited from default procedure object
    gapm_actv_proc_t  hdr;
    /// TX Test parameters
    gapm_le_test_tx_param_t  param;
    /// CTE parameters
    gapm_le_test_cte_param_t cte_param;
    /// Antenna ID array
    uint8_t               antenna_id[__ARRAY_EMPTY];
} gapm_le_test_tx_proc_start_t;


/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */
__STATIC uint16_t gapm_le_test_rx_start_transition(gapm_actv_t* p_actv, gapm_le_test_rx_proc_start_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_le_test_tx_start_transition(gapm_actv_t* p_actv, gapm_le_test_tx_proc_start_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_le_test_stop_transition(gapm_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_le_test_delete_transition(gapm_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Activity object interface
__STATIC const gapm_actv_itf_t gapm_le_test_rx_actv_itf =
{
    .cb_clean                  = (gapm_actv_clean_cb)           gapm_actv_clean, // Use default destructor
    .cb_start_transition       = (gapm_actv_proc_transition_cb) gapm_le_test_rx_start_transition,
    .cb_stop_transition        = (gapm_actv_proc_transition_cb) gapm_le_test_stop_transition,
    .cb_delete_transition      = (gapm_actv_proc_transition_cb) gapm_le_test_delete_transition,
    .cb_addr_renew_transition  = (gapm_actv_proc_transition_cb) NULL,
};

/// Activity object interface
__STATIC const gapm_actv_itf_t gapm_le_test_tx_actv_itf =
{
    .cb_clean                  = (gapm_actv_clean_cb)           gapm_actv_clean, // Use default destructor
    .cb_start_transition       = (gapm_actv_proc_transition_cb) gapm_le_test_tx_start_transition,
    .cb_stop_transition        = (gapm_actv_proc_transition_cb) gapm_le_test_stop_transition,
    .cb_delete_transition      = (gapm_actv_proc_transition_cb) gapm_le_test_delete_transition,
    .cb_addr_renew_transition  = (gapm_actv_proc_transition_cb) NULL,
};
/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/*
 * HCI EVENT HANDLERS DEFINITIONS
 ****************************************************************************************
 */

/// Default HCI Command complete handler used to continue procedure execution
__STATIC void gapm_le_test_hci_cmd_cmp_handler(uint16_t opcode, uint16_t event, struct hci_basic_cmd_cmp_evt const *p_evt)
{
    gapm_proc_transition(GAPM_PROC_AIR, (uint8_t) event, RW_ERR_HCI_TO_HL(p_evt->status));
}

/// Default HCI Command complete for RX test end
__STATIC void gapm_le_test_rx_end_hci_cmd_cmp_handler(uint16_t opcode, uint16_t event, struct hci_test_end_cmd_cmp_evt const *p_evt)
{
     uint16_t status = RW_ERR_HCI_TO_HL(p_evt->status);
    gapm_actv_t* p_actv =  gapm_actv_get(gapm_env.test_actv_idx);
    if(p_actv != NULL)
    {
        const gapm_le_test_rx_actv_cb_t* p_cbs = (const gapm_le_test_rx_actv_cb_t*) p_actv->p_cbs;
        if ((status == GAP_ERR_NO_ERROR) && (p_cbs->nb_packet_received != NULL))
        {
            p_cbs->nb_packet_received(p_actv->dummy, p_actv->idx, p_evt->nb_packet_received);
        }
    }

    gapm_proc_transition(GAPM_PROC_AIR, (uint8_t) event, status);
}


#if (BLE_AOD | BLE_AOA)
/**
 ****************************************************************************************
 * @brief Handle reception of IQ report.
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void gapm_le_test_hci_le_conless_iq_report_evt_handler(uint8_t evt_code, struct hci_le_conless_iq_report_evt const *p_evt)
{
    gapm_actv_t* p_actv = gapm_actv_get(gapm_env.test_actv_idx);
    if((p_actv != NULL) && (p_actv->type == GAPM_ACTV_TYPE_RX_TEST))
    {
        const gapm_le_test_rx_actv_cb_t* p_cbs = (const gapm_le_test_rx_actv_cb_t*) p_actv->p_cbs;

        // send report only if callback registered
        if(p_cbs->iq_report_received != NULL)
        {
            gapm_iq_report_info_t info;
            info.channel_idx          = p_evt->channel_idx;
            info.rssi                 = p_evt->rssi;
            info.rssi_antenna_id      = p_evt->rssi_antenna_id;
            info.cte_type             = p_evt->cte_type;
            info.slot_dur             = p_evt->slot_dur;
            info.pkt_status           = p_evt->pkt_status;
            info.pa_evt_cnt           = p_evt->pa_evt_cnt;

            p_cbs->iq_report_received(p_actv->dummy, p_actv->idx, &info, p_evt->sample_cnt,
                                      (const gap_iq_sample_t*)p_evt->iq_sample);
        }
    }
}
#endif // (BLE_AOD | BLE_AOA)


/*
 * PROCEDURE TRANSITION - State machine
 ****************************************************************************************
 */

/// Activity start transition state machine
__STATIC uint16_t gapm_le_test_rx_start_transition(gapm_actv_t* p_actv, gapm_le_test_rx_proc_start_t* p_proc, uint8_t event,
                                                uint16_t status, bool* p_finished)
{
    *p_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                if(p_proc->cte_param.switching_pattern_len == 0)
                {
                    // allocate Start RX test mode command
                    struct hci_le_rx_test_v2_cmd* p_rx_test_cmd
                                    = HL_HCI_CMD_ALLOC(HCI_LE_RX_TEST_V2_CMD_OPCODE, hci_le_rx_test_v2_cmd);

                    if(p_rx_test_cmd == NULL)
                    {
                        status = GAP_ERR_INSUFF_RESOURCES;
                        break;
                    }

                    p_rx_test_cmd->rx_channel             = p_proc->param.channel;
                    p_rx_test_cmd->phy                    = p_proc->param.phy;
                    p_rx_test_cmd->mod_idx                = p_proc->param.modulation_idx;
                    HL_HCI_CMD_SEND_TO_CTRL(p_rx_test_cmd, HL_PROC_FINISHED, gapm_le_test_hci_cmd_cmp_handler);
                }
                else
                {
                    uint8_t cursor;
                    // allocate Start RX test mode command
                    struct hci_le_rx_test_v3_cmd* p_rx_test_cmd
                                    = HL_HCI_CMD_ALLOC(HCI_LE_RX_TEST_V3_CMD_OPCODE, hci_le_rx_test_v3_cmd);

                    if(p_rx_test_cmd == NULL)
                    {
                        status = GAP_ERR_INSUFF_RESOURCES;
                        break;
                    }

                    p_rx_test_cmd->rx_channel             = p_proc->param.channel;
                    p_rx_test_cmd->phy                    = p_proc->param.phy;
                    p_rx_test_cmd->mod_idx                = p_proc->param.modulation_idx;
                    p_rx_test_cmd->slot_dur               = p_proc->param.slot_dur;
                    p_rx_test_cmd->exp_cte_len            = p_proc->cte_param.cte_len;
                    p_rx_test_cmd->exp_cte_type           = p_proc->cte_param.cte_type;
                    p_rx_test_cmd->switching_pattern_len  = p_proc->cte_param.switching_pattern_len;
                    for(cursor = 0 ; cursor < co_min(p_rx_test_cmd->switching_pattern_len, MAX_SWITCHING_PATTERN_LEN) ; cursor ++)
                    {
                        p_rx_test_cmd->antenna_id[cursor] = p_proc->antenna_id[cursor];
                    }
                    HL_HCI_CMD_SEND_TO_CTRL(p_rx_test_cmd, HL_PROC_FINISHED, gapm_le_test_hci_cmd_cmp_handler);
                }
            } break;
            default:
            {
                *p_finished = true;
            } break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        // test has not been started properly
        gapm_env.test_actv_idx = GAPM_ACTV_INVALID_IDX;
        *p_finished = true;
    }

    return (status);
}


/// Activity start transition state machine
__STATIC uint16_t gapm_le_test_tx_start_transition(gapm_actv_t* p_actv, gapm_le_test_tx_proc_start_t* p_proc, uint8_t event,
                                                uint16_t status, bool* p_finished)
{
    *p_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                if(p_proc->cte_param.switching_pattern_len == 0)
                {
                    // allocate Start TX test mode command
                    struct hci_le_tx_test_v2_cmd* p_tx_test_cmd
                                    = HL_HCI_CMD_ALLOC(HCI_LE_TX_TEST_V2_CMD_OPCODE, hci_le_tx_test_v2_cmd);

                    if(p_tx_test_cmd == NULL)
                    {
                        status = GAP_ERR_INSUFF_RESOURCES;
                        break;
                    }

                    p_tx_test_cmd->tx_channel             = p_proc->param.channel;
                    p_tx_test_cmd->test_data_len          = p_proc->param.tx_data_length;
                    p_tx_test_cmd->pkt_payl               = p_proc->param.tx_pkt_payload;
                    p_tx_test_cmd->phy                    = p_proc->param.phy;

                    HL_HCI_CMD_SEND_TO_CTRL(p_tx_test_cmd, HL_PROC_FINISHED, gapm_le_test_hci_cmd_cmp_handler);
                }
                else
                {
                    uint8_t cursor;
                    // allocate Start TX test mode command
                    struct hci_le_tx_test_v4_cmd* p_tx_test_cmd
                                    = HL_HCI_CMD_ALLOC(HCI_LE_TX_TEST_V4_CMD_OPCODE, hci_le_tx_test_v4_cmd);

                    if(p_tx_test_cmd == NULL)
                    {
                        status = GAP_ERR_INSUFF_RESOURCES;
                        break;
                    }

                    p_tx_test_cmd->tx_channel             = p_proc->param.channel;
                    p_tx_test_cmd->test_data_len          = p_proc->param.tx_data_length;
                    p_tx_test_cmd->pkt_payl               = p_proc->param.tx_pkt_payload;
                    p_tx_test_cmd->phy                    = p_proc->param.phy;
                    p_tx_test_cmd->tx_pwr_lvl             = p_proc->param.tx_pwr_lvl;
                    p_tx_test_cmd->cte_len                = p_proc->cte_param.cte_len;
                    p_tx_test_cmd->cte_type               = p_proc->cte_param.cte_type;
                    p_tx_test_cmd->switching_pattern_len  = p_proc->cte_param.switching_pattern_len;
                    for(cursor = 0 ; cursor < co_min(p_tx_test_cmd->switching_pattern_len, MAX_SWITCHING_PATTERN_LEN) ; cursor ++)
                    {
                        p_tx_test_cmd->antenna_id[cursor] = p_proc->antenna_id[cursor];
                    }

                    HL_HCI_CMD_SEND_TO_CTRL(p_tx_test_cmd, HL_PROC_FINISHED, gapm_le_test_hci_cmd_cmp_handler);
                }
            } break;
            default:
            {
                *p_finished = true;
            } break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        // test has not been started properly
        gapm_env.test_actv_idx = GAPM_ACTV_INVALID_IDX;
        *p_finished = true;
    }

    return (status);
}

/// Activity stop transition state machine
__STATIC uint16_t gapm_le_test_stop_transition(gapm_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                               uint16_t status, bool* p_finished)
{
    *p_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                hl_hci_cmd_evt_func_t cb_cmp_evt = (p_actv->type == GAPM_ACTV_TYPE_RX_TEST)
                                                 ? (hl_hci_cmd_evt_func_t) gapm_le_test_rx_end_hci_cmd_cmp_handler // specific for RX test
                                                 : (hl_hci_cmd_evt_func_t) gapm_le_test_hci_cmd_cmp_handler;

                // Disable the RX test mode
                status = HL_HCI_BASIC_CMD_SEND(HCI_LE_TEST_END_CMD_OPCODE, HL_PROC_FINISHED, cb_cmp_evt);
            }
            break;
            default:
            {
                *p_finished = true;
                gapm_env.test_actv_idx = GAPM_ACTV_INVALID_IDX;
            } break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        *p_finished = true;
    }

    return (status);
}

/// Activity delete transition state machine
__STATIC uint16_t gapm_le_test_delete_transition(gapm_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                               uint16_t status, bool* p_finished)
{
    // Nothing special to be done
    *p_finished = true;
    return (status);
}


/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapm_le_test_rx_create(uint32_t dummy, const gapm_le_test_rx_actv_cb_t* p_cbs, uint8_t* p_actv_idx)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    do
    {
        // Allocated activity structure
        gapm_actv_t *p_actv;

        ASSERT_INFO(GAPM_HEAP_ENV_SIZE >= (sizeof(gapm_actv_t) + KE_HEAP_MEM_RESERVED),
                    GAPM_HEAP_ENV_SIZE, sizeof(gapm_actv_t));

        // Check if supported roles allow to start page scanning
        if (!GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_BROADCASTER))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Allocate an activity structure
        status = gapm_actv_create(GAPM_ACTV_TYPE_RX_TEST, 0, dummy, sizeof(gapm_actv_t),
                                  &gapm_le_test_rx_actv_itf,  (gapm_actv_cb_t*) p_cbs, (gapm_actv_t**) &p_actv);
        if(status != GAP_ERR_NO_ERROR) break;

        gapm_actv_created(p_actv);
        *p_actv_idx = p_actv->idx;
    } while (0);

    return (status);
}

uint16_t gapm_le_test_rx_start(uint8_t actv_idx, const gapm_le_test_rx_param_t* p_param)
{
    gapm_le_test_cte_param_t cte_param;
    memset(&cte_param, 0, sizeof(gapm_le_test_cte_param_t));

    return (gapm_le_test_rx_start_with_cte(actv_idx, p_param, &cte_param, NULL));
}

uint16_t gapm_le_test_rx_start_with_cte(uint8_t actv_idx, const gapm_le_test_rx_param_t* p_param,
                                     const gapm_le_test_cte_param_t* p_cte_param, const uint8_t* p_antenna_id)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    do
    {
        gapm_actv_t* p_actv = gapm_actv_get(actv_idx);
        gapm_le_test_rx_proc_start_t *p_proc;

        // activity type sanity check
        if((p_actv == NULL) || (p_actv->type != GAPM_ACTV_TYPE_RX_TEST))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Cannot have two test activities started in parallel
        if (gapm_env.test_actv_idx != GAPM_ACTV_INVALID_IDX)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Parameter sanity check
        if ((p_param == NULL) || (p_cte_param == NULL))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // create start procedure
        status = gapm_actv_start(p_actv, sizeof(gapm_le_test_rx_proc_start_t) + p_cte_param->switching_pattern_len,
                                 (gapm_actv_proc_t**)&p_proc);
        if(status != GAP_ERR_NO_ERROR) break;
        p_proc->param      = *p_param;
        p_proc->cte_param  = *p_cte_param;
        memcpy(p_proc->antenna_id, p_antenna_id, p_cte_param->switching_pattern_len);

        gapm_env.test_actv_idx = actv_idx;
    } while (0);

    return (status);
}

uint16_t gapm_le_test_tx_create(uint32_t dummy, const gapm_actv_cb_t* p_cbs, uint8_t* p_actv_idx)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    do
    {
        // Allocated activity structure
        gapm_actv_t *p_actv;

        ASSERT_INFO(GAPM_HEAP_ENV_SIZE >= (sizeof(gapm_actv_t) + KE_HEAP_MEM_RESERVED),
                    GAPM_HEAP_ENV_SIZE, sizeof(gapm_actv_t));

        // Check if supported roles allow to start page scanning
        if (!GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_BROADCASTER))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Allocate an activity structure
        status = gapm_actv_create(GAPM_ACTV_TYPE_TX_TEST, 0, dummy, sizeof(gapm_actv_t),
                                  &gapm_le_test_tx_actv_itf, p_cbs, (gapm_actv_t**) &p_actv);
        if(status != GAP_ERR_NO_ERROR) break;

        gapm_actv_created(p_actv);
        *p_actv_idx = p_actv->idx;
    } while (0);

    return (status);
}

uint16_t gapm_le_test_tx_start(uint8_t actv_idx, const gapm_le_test_tx_param_t* p_param)
{
    gapm_le_test_cte_param_t cte_param;
    memset(&cte_param, 0, sizeof(gapm_le_test_cte_param_t));

    return (gapm_le_test_tx_start_with_cte(actv_idx, p_param, &cte_param, NULL));
}

uint16_t gapm_le_test_tx_start_with_cte(uint8_t actv_idx, const gapm_le_test_tx_param_t* p_param,
                                     const gapm_le_test_cte_param_t* p_cte_param, const uint8_t* p_antenna_id)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    do
    {
        gapm_actv_t* p_actv = gapm_actv_get(actv_idx);
        gapm_le_test_tx_proc_start_t *p_proc;

        // activity type sanity check
        if((p_actv == NULL) || (p_actv->type != GAPM_ACTV_TYPE_TX_TEST))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Cannot have two test activities started in parallel
        if (gapm_env.test_actv_idx != GAPM_ACTV_INVALID_IDX)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Parameter sanity check
        if ((p_param == NULL) || (p_cte_param == NULL))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // create start procedure
        status = gapm_actv_start(p_actv, sizeof(gapm_le_test_tx_proc_start_t) + p_cte_param->switching_pattern_len,
                                 (gapm_actv_proc_t**)&p_proc);
        if(status != GAP_ERR_NO_ERROR) break;
        p_proc->param      = *p_param;
        p_proc->cte_param  = *p_cte_param;
        memcpy(p_proc->antenna_id, p_antenna_id, p_cte_param->switching_pattern_len);

        gapm_env.test_actv_idx = actv_idx;
    } while (0);

    return (status);
}
#endif // (BLE_HOST_PRESENT && HOST_TEST_MODE)
/// @} GAPM_CFG
