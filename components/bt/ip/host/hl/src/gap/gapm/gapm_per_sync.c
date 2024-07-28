/**_
 ****************************************************************************************
 *
 * @file gapm_per_sync.c
 *
 * @brief Generic Access Profile Manager - Periodic synchronization manager module.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_PER_SYNC Generic Access Profile Manager - Periodic synchronization manager module.
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "gapm_le_per_sync.h"

#if (HL_LE_OBSERVER)
#include "gapm_int.h"

#include <string.h>

#include "gap.h"
#include "gapc.h"

#include "ke_mem.h"
#include "hl_hci.h"
#include "hci.h"
#include "co_utils.h"
#include "co_math.h"
#include "co_buf.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// First invalid sync handle value
#define GAPM_PER_SYNC_INVALID_SYNC_HANDLE   (0x0EFF + 1)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Bit field bit positions for periodic synchronization activity info parameter
enum gapm_per_sync_info_bf
{
    /// Synchronization has been established for this activity
    GAPM_PER_SYNC_INFO_SYNC_POS               = 0,
    GAPM_PER_SYNC_INFO_SYNC_BIT               = CO_BIT(GAPM_PER_SYNC_INFO_SYNC_POS),

    /// Indicate that application accepts uploading of truncated reports
    GAPM_PER_SYNC_INFO_ACCEPT_TRUNCATED_POS   = 1,
    GAPM_PER_SYNC_INFO_ACCEPT_TRUNCATED_BIT   = CO_BIT(GAPM_PER_SYNC_INFO_ACCEPT_TRUNCATED_POS),

    /// Enable reception of ADV reports
    GAPM_PER_SYNC_ADV_REPORT_EN_POS           = 2,
    GAPM_PER_SYNC_ADV_REPORT_EN_BIT           = CO_BIT(GAPM_PER_SYNC_ADV_REPORT_EN_POS),

    /// Enable reception of BIG info reports
    GAPM_PER_SYNC_BIG_INFO_REPORT_EN_POS      = 3,
    GAPM_PER_SYNC_BIG_INFO_REPORT_EN_BIT      = CO_BIT(GAPM_PER_SYNC_BIG_INFO_REPORT_EN_POS),
};

/*
 * TYPES DEFINITIONS
 ****************************************************************************************
 */

typedef void (*gapm_per_sync_hci_cmd_cmp_cb)(uint16_t opcode, uint16_t event, struct hci_basic_cmd_cmp_evt const *p_evt);


/// GAP Manager activity structure for periodic synchronization activity
typedef struct gapm_per_sync_actv
{
    /// Common activity parameters
    gapm_actv_t     hdr;
    /// Pointer to currently received periodic ADV report
    co_buf_t*       p_report;
    /// Synchronization Handle
    uint16_t        sync_hdl;
    /// Information bit field, meaning depends on activity type
    uint8_t         info_bf;
    #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    /// Connection index where PAST sync info_bf is expected
    uint8_t         conidx;
    #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
} gapm_per_sync_actv_t;


/// Periodic sync activity start procedure structure
typedef struct gapm_per_sync_proc_start
{
    /// Inherited from default procedure object
    gapm_actv_proc_t      hdr;
    /// Scan start parameters
    gapm_per_sync_param_t param;
} gapm_per_sync_proc_start_t;



/// Periodic sync activity iq report control procedure structure
typedef struct gapm_per_sync_proc_iq_report_ctrl
{
    /// Inherited from default procedure object
    gapm_actv_proc_t hdr;
    /// True to enable IQ sampling, false to disable
    uint8_t          enable;
    /// Slot durations (1: 1us | 2: 2us)
    uint8_t          slot_dur;
    /// Max sampled CTEs
    uint8_t          max_sampl_cte;
    /// Length of switching pattern
    uint8_t          switching_pattern_len;
    /// Antenna IDs
    uint8_t          antenna_id[__ARRAY_EMPTY];
} gapm_per_sync_proc_iq_report_ctrl_t;

/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */
__STATIC uint16_t gapm_per_sync_start_transition(gapm_per_sync_actv_t *p_actv, gapm_per_sync_proc_start_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_per_sync_stop_transition(gapm_per_sync_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_per_sync_delete_transition(gapm_per_sync_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC void gapm_per_sync_clean(gapm_per_sync_actv_t* p_actv, bool reset);

__STATIC void gapm_per_sync_hci_cmd_cmp_handler(uint16_t opcode, uint16_t event, struct hci_basic_cmd_cmp_evt const *p_evt);

#if (HOST_TEST_MODE)
#if (BLE_AOD | BLE_AOA)
// file present in gapm_le_test.c
extern void gapm_le_test_hci_le_conless_iq_report_evt_handler(uint8_t evt_code, struct hci_le_conless_iq_report_evt const *p_evt);
#endif // (BLE_AOD | BLE_AOA)
#endif // (HOST_TEST_MODE)

#if(HL_LE_CENTRAL || HL_LE_PERIPHERAL)
__STATIC uint16_t gapm_per_adv_sync_trans_param_send(gapm_per_sync_actv_t *p_actv, uint8_t mode,
                                                     uint16_t skip, uint16_t sync_to, uint8_t cte_type, uint8_t event,
                                                     gapm_per_sync_hci_cmd_cmp_cb cb_cmp);
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Activity object interface
__STATIC const gapm_actv_itf_t gapm_per_sync_actv_itf =
{
    .cb_clean                  = (gapm_actv_clean_cb)           gapm_per_sync_clean,
    .cb_start_transition       = (gapm_actv_proc_transition_cb) gapm_per_sync_start_transition,
    .cb_stop_transition        = (gapm_actv_proc_transition_cb) gapm_per_sync_stop_transition,
    .cb_delete_transition      = (gapm_actv_proc_transition_cb) gapm_per_sync_delete_transition,
    .cb_addr_renew_transition  = (gapm_actv_proc_transition_cb) NULL,
};

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Clean content of a scan activity, more specifically the stored fragments
 * of advertising report and the filtering list.
 *
 * @param[in] p_actv    Pointer to the scanning activity structure.
 * @param[in] reset     True if a reset is on-going, False otherwise
 ****************************************************************************************
 */
__STATIC void gapm_per_sync_clean(gapm_per_sync_actv_t *p_actv, bool reset)
{
    if((!reset) && (p_actv->p_report != NULL))
    {
        co_buf_release(p_actv->p_report);
    }

    // call inherited destructor
    gapm_actv_clean(&(p_actv->hdr), reset);
}


/**
 ****************************************************************************************
 * @brief Look for a periodic synchronization activity.
 *
 * @param[in] sync_handle   Synchronization handle if looking for an activity on which
 *                          synchronization has been established,
 ****************************************************************************************
 */
__STATIC gapm_per_sync_actv_t *gapm_per_sync_get_activity(uint16_t sync_handle)
{
    // Found activity
    gapm_per_sync_actv_t *p_actv = NULL;
    uint8_t actv_idx;

    // Loop over the activities
    for (actv_idx = 0; actv_idx < GAPM_ACTV_NB; actv_idx++)
    {
        // Get activity
        gapm_per_sync_actv_t *p_actv_loop = (gapm_per_sync_actv_t *)gapm_actv_get(actv_idx);

        // Check that activity exists, is started and is a periodic synchronization activity
        if (   (p_actv_loop != NULL)
            && (p_actv_loop->hdr.type == GAPM_ACTV_TYPE_PER_SYNC)
            && (p_actv_loop->hdr.state == GAPM_ACTV_STARTED))
        {
            // Look for a synchronized activity with the same sync handle
            if (GETB(p_actv_loop->info_bf, GAPM_PER_SYNC_INFO_SYNC) && (p_actv_loop->sync_hdl == sync_handle))
            {
                // If we are here, activity is the one we are looking for
                p_actv = p_actv_loop;
                break;
            }
        }
    }

    return (p_actv);
}

/**
 ****************************************************************************************
 * @brief Send a GAPM_EXT_ADV_REPORT_IND message containing a received periodic advertising
 * report to the application.
 *
 * @param[in] p_actv   Pointer to the periodic synchronization activity
 * @param[in] p_event           Pointer to the last received advertising report fragment
 * @param[in] complete          Indicate if periodic advertising report is complete
 ****************************************************************************************
 */
__STATIC void gapm_per_sync_send_adv_report(gapm_per_sync_actv_t *p_actv,
                                            struct hci_le_per_adv_report_evt const *p_event, bool complete)
{
    co_buf_t* p_report = p_actv->p_report;

    const gapm_per_sync_actv_cb_t* p_cbs = (const gapm_per_sync_actv_cb_t*) p_actv->hdr.p_cbs;
    gapm_adv_report_info_t* p_meta = (gapm_adv_report_info_t*) co_buf_metadata(p_report);

    p_meta->rssi     = p_event->rssi;
    p_meta->tx_pwr   = p_event->tx_power;
    p_meta->info     = GAPM_REPORT_TYPE_PER_ADV;

    p_meta->phy_prim        = 0;
    p_meta->phy_second      = 0;
    p_meta->adv_sid         = 0;
    p_meta->period_adv_intv = 0;
    memset(&(p_meta->trans_addr), 0, sizeof(gap_bdaddr_t));
    memset(&(p_meta->target_addr), 0, sizeof(gap_bdaddr_t));

    if (complete)
    {
        p_meta->info |= GAPM_REPORT_INFO_COMPLETE_BIT;
    }

    // Push report to upper layer software
    p_cbs->report_received(p_actv->hdr.dummy, p_actv->hdr.idx, p_meta, p_report);

    // release report
    co_buf_release(p_report);
    p_actv->p_report = NULL;
}


/**
 ****************************************************************************************
 * @brief Inform upper layer software about establishment of periodic sync.
 *
 * @param[in] p_actv            Pointer to the periodic synchronization activity
 * @param[in] p_info            Pointer to periodic sync information
 ****************************************************************************************
 */
__STATIC void gapm_per_sync_estab_send(gapm_per_sync_actv_t* p_actv, const gapm_per_sync_info_t *p_info)
{
    gapm_per_sync_actv_cb_t* p_cbs = (gapm_per_sync_actv_cb_t*) p_actv->hdr.p_cbs;
    p_cbs->established(p_actv->hdr.dummy, p_actv->hdr.idx, p_info);
}

/*
 * HCI HANDLERS DEFINITIONS
 ****************************************************************************************
 */

/// Default HCI Command complete handler used to continue procedure execution
__STATIC void gapm_per_sync_hci_cmd_cmp_handler(uint16_t opcode, uint16_t event, struct hci_basic_cmd_cmp_evt const *p_evt)
{
    gapm_proc_transition(GAPM_PROC_AIR, (uint8_t) event, RW_ERR_HCI_TO_HL(p_evt->status));
}

#if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
/// Command complete handler used to ignore event
__STATIC void gapm_per_sync_hci_cmd_cmp_ignore_handler(uint16_t opcode, uint16_t event, struct hci_basic_cmd_cmp_evt const *p_evt)
{
    // nothing to do
}
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)


/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Periodic Advertising Sync Established event.
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void gapm_hci_le_per_adv_sync_est_evt_handler(uint8_t evt_code, struct hci_le_per_adv_sync_est_evt const *p_evt)
{
    // Get periodic synchronization activity for which synchronization establishment was in progress
    gapm_per_sync_actv_t* p_actv = (gapm_per_sync_actv_t*) gapm_actv_get(gapm_env.per_sync_estab_actv_idx);

    if (p_actv != NULL)
    {
        // Next step depends on the provided status
        if (p_evt->status == CO_ERROR_NO_ERROR)
        {
            gapm_per_sync_info_t info;
            // Keep in mind the provided sync handle
            p_actv->sync_hdl = p_evt->sync_handle;

            // Keep in mind that synchronization has been established
            SETB(p_actv->info_bf, GAPM_PER_SYNC_INFO_SYNC, true);

            info.phy            = p_evt->phy;
            info.interval       = p_evt->interval;
            info.adv_sid        = p_evt->adv_sid;
            info.clk_acc        = p_evt->adv_ca;
            info.addr.addr_type = p_evt->adv_addr_type;
            memcpy(&info.addr.addr[0], &p_evt->adv_addr.addr[0], GAP_BD_ADDR_LEN);
            info.serv_data      = 0;

            gapm_per_sync_estab_send(p_actv, &info);
        }
        else
        {
            // Consider that the activity can be stopped
            gapm_actv_stopped(&(p_actv->hdr), RW_ERR_HCI_TO_HL(p_evt->status));
        }
    }

    // release periodic sync establishment
    gapm_env.per_sync_estab_actv_idx = GAP_INVALID_ACTV_IDX;
}

#if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Periodic Advertising Sync Transfer received event.
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void gapm_hci_le_per_adv_sync_transf_rec_evt_handler(uint8_t evt_code, struct hci_le_per_adv_sync_transf_rec_evt const *p_evt)
{
    // Get periodic synchronization activity for which synchronization establishment was in progress,
    // only one exist
    uint8_t conidx = gapc_get_conidx(p_evt->conhdl);
    uint8_t actv_idx = gapc_past_actv_idx_get(conidx);
    gapm_per_sync_actv_t *p_actv = (gapm_per_sync_actv_t*) gapm_actv_get(actv_idx);

    if (p_actv)
    {
        // Next step depends on the provided status
        if (p_evt->status == CO_ERROR_NO_ERROR)
        {
            gapm_per_sync_info_t info;
            // Keep in mind the provided sync handle
            p_actv->sync_hdl = p_evt->sync_hdl;

            // Keep in mind that synchronization has been established
            SETB(p_actv->info_bf, GAPM_PER_SYNC_INFO_SYNC, true);

            info.phy            = p_evt->phy;
            info.interval       = p_evt->interval;
            info.adv_sid        = p_evt->adv_sid;
            info.clk_acc        = p_evt->adv_ca;
            info.addr.addr_type = p_evt->adv_addr_type;
            memcpy(&info.addr.addr[0], &p_evt->adv_addr.addr[0], GAP_BD_ADDR_LEN);
            info.serv_data      = p_evt->serv_data;

            gapm_per_sync_estab_send(p_actv, &info);
        }
        else
        {
            // Consider that the activity can be stopped
            gapm_actv_stopped(&(p_actv->hdr), RW_ERR_HCI_TO_HL(p_evt->status));
        }

        // Send command to stop receive PAST from a specific connection
        gapm_per_adv_sync_trans_param_send(p_actv, NO_SYNC, 0, SYNC_TIMEOUT_MIN, 0, HL_PROC_FINISHED,
                                           gapm_per_sync_hci_cmd_cmp_ignore_handler);
        gapm_env.past_estab_bf &= ~CO_BIT(conidx);
    }
}
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)


/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Periodic Advertising Report event.
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void gapm_hci_le_per_adv_report_evt_handler(uint8_t evt_code, struct hci_le_per_adv_report_evt const *p_evt)
{
    // Look for period synchronization activity with provided sync handle
    gapm_per_sync_actv_t *p_actv = gapm_per_sync_get_activity(p_evt->sync_handle);

    if (p_actv)
    {
        bool truncated = (p_evt->status == ADV_EVT_DATA_STATUS_TRUNCATED);

        // if advertising flow off or truncated buffer not supported by application, remove already received fragments
        if(   gapm_env.adv_report_flow_off || !GETB(p_actv->info_bf, GAPM_PER_SYNC_ADV_REPORT_EN)
           || (truncated && !GETB(p_actv->info_bf, GAPM_PER_SYNC_INFO_ACCEPT_TRUNCATED)))
        {
            if(p_actv->p_report != NULL)
            {
                co_buf_release(p_actv->p_report);
                p_actv->p_report = NULL;
            }
        }
        else if(gapm_scan_append(&(p_actv->p_report), p_evt->data_len, p_evt->data))
        {
            if((p_evt->status == PER_ADV_EVT_DATA_STATUS_COMPLETE) || truncated)
            {
                gapm_per_sync_send_adv_report(p_actv, p_evt, !truncated);
            }
        }
    }
}


/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE BIG Info Advertising Report event.
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void gapm_hci_le_big_info_adv_report_evt_handler(uint8_t evt_code, struct hci_le_big_info_adv_report_evt const *p_evt)
{
    // Look for period synchronization activity with provided sync handle
    gapm_per_sync_actv_t *p_actv = gapm_per_sync_get_activity(p_evt->sync_hdl);

    // check if report can be conveyed to an upper layer
    if (p_actv)
    {
        gapm_per_sync_actv_cb_t* p_cbs = (gapm_per_sync_actv_cb_t*) p_actv->hdr.p_cbs;

        // send report only if callback registered
        if((p_cbs->big_info_report_received != NULL) && GETB(p_actv->info_bf, GAPM_PER_SYNC_BIG_INFO_REPORT_EN))
        {
            gap_big_info_t report;
            report.sdu_interval = co_read24p(p_evt->sdu_interval);
            report.iso_interval = p_evt->iso_interval;
            report.max_pdu      = p_evt->max_pdu;
            report.max_sdu      = p_evt->max_sdu;
            report.num_bis      = p_evt->num_bis;
            report.nse          = p_evt->nse;
            report.bn           = p_evt->bn;
            report.pto          = p_evt->pto;
            report.irc          = p_evt->irc;
            report.phy          = p_evt->phy;
            report.framing      = p_evt->framing;
            report.encrypted    = p_evt->encryption;

            p_cbs->big_info_report_received(p_actv->hdr.dummy, p_actv->hdr.idx, &report);
        }
    }
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
void gapm_per_sync_hci_le_conless_iq_report_evt_handler(uint8_t evt_code, struct hci_le_conless_iq_report_evt const *p_evt)
{
    #if (HOST_TEST_MODE)
    if(p_evt->sync_hdl == RX_TEST_SYNC_HDL)
    {
        // forward event to Test Mode
        gapm_le_test_hci_le_conless_iq_report_evt_handler(evt_code, p_evt);
    }
    else
    #endif // (HOST_TEST_MODE)
    {
        // Look for period synchronization activity with provided sync handle
        gapm_per_sync_actv_t *p_actv = gapm_per_sync_get_activity(p_evt->sync_hdl);

        if (p_actv)
        {
            gapm_per_sync_actv_cb_t* p_cbs = (gapm_per_sync_actv_cb_t*) p_actv->hdr.p_cbs;

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

                p_cbs->iq_report_received(p_actv->hdr.dummy, p_actv->hdr.idx, &info, p_evt->sample_cnt,
                                          (const gap_iq_sample_t*)p_evt->iq_sample);
            }
        }
    }
}
#endif // (BLE_AOD | BLE_AOA)

/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Periodic Advertising Sync Lost event.
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void gapm_hci_le_per_adv_sync_lost_evt_handler(uint8_t evt_code, struct hci_le_per_adv_sync_lost_evt const *p_evt)
{
    // Get periodic synchronization activity with the provided sync handle
    gapm_per_sync_actv_t *p_actv = gapm_per_sync_get_activity(p_evt->sync_handle);

    if (p_actv)
    {
        // Keep in mind that the synchronization has been lost
        SETB(p_actv->info_bf, GAPM_PER_SYNC_INFO_SYNC, false);

        // Stop the activity and inform the application
        gapm_actv_stopped((gapm_actv_t *)p_actv, GAP_ERR_DISCONNECTED);
    }
}


/**
 ****************************************************************************************
 * @brief Send a LE Periodic Advertising Create Sync command over HCI.
 *
 * @param[in] p_actv    Pointer to the activity structure
 * @param[in] p_param   Pointer to the synchronization parameters
 * @param[in] event     Event transition to trigger when command complete event is received
 ****************************************************************************************
 */
__STATIC uint16_t gapm_per_sync_send_hci_le_per_adv_create_sync_cmd(gapm_per_sync_actv_t *p_actv,
                                                                    gapm_per_sync_param_t *p_param,
                                                                    uint8_t event)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    // Allocate HCI command message
    struct hci_le_per_adv_create_sync_cmd *p_cmd =
            HL_HCI_CMD_ALLOC(HCI_LE_PER_ADV_CREATE_SYNC_CMD_OPCODE, hci_le_per_adv_create_sync_cmd);

    if(p_cmd)
    {
        status = GAP_ERR_NO_ERROR;
        p_cmd->options = 0;

        // Fill the command
        if (p_actv->hdr.subtype == GAPM_PER_SYNC_TYPE_GENERAL)
        {
            SETB(p_cmd->options, PER_SYNC_FILT_USE_PAL, false);

            // Fill advertiser identity
            memcpy(&p_cmd->adv_addr.addr[0], &p_param->adv_addr.addr[0], GAP_BD_ADDR_LEN);
            p_cmd->adv_addr_type = p_param->adv_addr.addr_type;
            p_cmd->adv_sid       = p_param->adv_addr.adv_sid;
        }
        else
        {
            p_cmd->adv_sid       = 0;
            p_cmd->adv_addr_type = 0;
            memset(&p_cmd->adv_addr.addr[0], 0, GAP_BD_ADDR_LEN);
            SETB(p_cmd->options, PER_SYNC_FILT_USE_PAL, true);
        }

        // Check if periodic advertising reports must be disabled
        if(!GETB(p_param->report_en_bf, GAPM_REPORT_ADV_EN))
        {
            SETB(p_cmd->options, PER_SYNC_REP_INIT_DIS, true);
        }

        p_cmd->skip          = p_param->skip;
        p_cmd->sync_to       = p_param->sync_to;
        p_cmd->sync_cte_type = p_param->cte_type;

        // Send the command
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, event, gapm_per_sync_hci_cmd_cmp_handler);
    }

    return (status);
}


#if (HL_LE_PERIPHERAL || HL_LE_CENTRAL)
/**
 ****************************************************************************************
 * @brief Set PAST sync parameters to controller
 *
 * @param[in] p_actv    On-going activity
 * @param[in] conhdl    targeted connection handle
 * @param[in] mode      Mode (see enum #per_adv_sync_info_rec_mode)
 * @param[in] skip      The number of periodic advertising packets that can be skipped after a successful receive
 * @param[in] sync_to   Sync timeout (Time=N*10ms)
 * @param[in] cte_type  CTE type (see enum #sync_cte_type)
 * @param[in] event     Event transition to trigger when command complete event is received
 * @param[in] cb_cmp    Function called once HCI complete event is received
 ****************************************************************************************
 */
__STATIC uint16_t gapm_per_adv_sync_trans_param_send(gapm_per_sync_actv_t *p_actv, uint8_t mode,
                                                     uint16_t skip, uint16_t sync_to, uint8_t cte_type,
                                                     uint8_t event, gapm_per_sync_hci_cmd_cmp_cb cb_cmp)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    uint16_t conhdl = gapc_get_conhdl(p_actv->conidx);

    if(conhdl == GAP_INVALID_CONHDL)
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else
    {
        struct hci_le_set_per_adv_sync_transf_param_cmd *p_cmd;
        p_cmd = HL_HCI_CMD_ALLOC(HCI_LE_SET_PER_ADV_SYNC_TRANSF_PARAM_CMD_OPCODE,
                                 hci_le_set_per_adv_sync_transf_param_cmd);

        if(p_cmd != NULL)
        {
            status = GAP_ERR_NO_ERROR;
            p_cmd->conhdl   = conhdl;
            p_cmd->mode     = mode;
            p_cmd->skip     = skip;
            p_cmd->sync_to  = sync_to;
            p_cmd->cte_type = cte_type;
            HL_HCI_CMD_SEND_TO_CTRL(p_cmd, event, cb_cmp);

            // store activity index in connection environment
            gapc_past_actv_idx_set(p_actv->conidx, p_actv->hdr.idx);
        }
    }

    return (status);
}
#endif // (HL_LE_PERIPHERAL || HL_LE_CENTRAL)

/**
 ****************************************************************************************
 * @brief Send a LE Periodic Advertising Create Sync Cancel command over HCI.
 *
 * @param[in] p_actv    Pointer to the activity structure
 * @param[in] event     Event transition to trigger when command complete event is received
 ****************************************************************************************
 */
__STATIC uint16_t gapm_per_sync_send_hci_le_per_adv_create_sync_cancel_cmd(gapm_per_sync_actv_t *p_actv, uint8_t event)
{
    return HL_HCI_BASIC_CMD_SEND(HCI_LE_PER_ADV_CREATE_SYNC_CANCEL_CMD_OPCODE, event, gapm_per_sync_hci_cmd_cmp_handler);
}

/**
 ****************************************************************************************
 * @brief Send a LE Periodic Advertising Terminate Sync command over HCI.
 *
 * @param[in] p_actv    Pointer to the activity structure
 ****************************************************************************************
 */
__STATIC uint16_t gapm_per_sync_send_hci_le_per_adv_term_sync_cmd(gapm_per_sync_actv_t *p_actv, uint8_t event)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    struct hci_le_per_adv_term_sync_cmd *p_cmd =
            HL_HCI_CMD_ALLOC(HCI_LE_PER_ADV_TERM_SYNC_CMD_OPCODE, hci_le_per_adv_term_sync_cmd);
    if(p_cmd != NULL)
    {
        status = GAP_ERR_NO_ERROR;
        p_cmd->sync_handle = p_actv->sync_hdl;
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, event, gapm_per_sync_hci_cmd_cmp_handler);
    }
    return (status);
}

/*
 * PROCEDURE TRANSITION - State machine
 ****************************************************************************************
 */


/// Activity start state machine
__STATIC uint16_t gapm_per_sync_start_transition(gapm_per_sync_actv_t *p_actv, gapm_per_sync_proc_start_t* p_proc, uint8_t event,
                                                  uint16_t status, bool* p_finished)
{
    gapm_per_sync_param_t* p_param = &(p_proc->param);
    *p_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        if(event == HL_PROC_GRANTED)
        {
            #if (HL_LE_PERIPHERAL || HL_LE_CENTRAL)
            if(p_actv->hdr.subtype == GAPM_PER_SYNC_TYPE_PAST)
            {
                // send the HCI Command to create sync using Periodic ADV Sync transfer method
                status = gapm_per_adv_sync_trans_param_send(p_actv,
                                           (GETB(p_param->report_en_bf, GAPM_REPORT_ADV_EN) ? SYNC_REP_EN : SYNC_REP_DIS),
                                           p_param->skip, p_param->sync_to, p_param->cte_type,
                                           HL_PROC_FINISHED, gapm_per_sync_hci_cmd_cmp_handler);
            }
            else
            #endif //(HL_LE_PERIPHERAL || HL_LE_CENTRAL)
            {
                // Send a LE Periodic Advertising Create Sync command to the controller
                status = gapm_per_sync_send_hci_le_per_adv_create_sync_cmd(p_actv, p_param, HL_PROC_FINISHED);
            }
        }
        else
        {
            *p_finished = true;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        // release periodic sync establishment information
        #if (HL_LE_PERIPHERAL || HL_LE_CENTRAL)
        if(p_param->type == GAPM_PER_SYNC_TYPE_PAST)
        {
            gapm_env.past_estab_bf &= ~CO_BIT(p_actv->conidx);
        }
        else
        #endif //(HL_LE_PERIPHERAL || HL_LE_CENTRAL)
        {
            gapm_env.per_sync_estab_actv_idx = GAP_INVALID_ACTV_IDX;
        }
        *p_finished = true;
    }

    return (status);
}

/// State machine of report control
__STATIC uint16_t gapm_per_sync_report_ctrl_transition(gapm_per_sync_actv_t* p_actv, gapm_actv_proc_t* p_proc,
                                                       uint8_t event, uint16_t status, bool* p_finished)
{
    *p_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                struct hci_le_set_per_adv_rec_en_cmd* p_cmd =
                        HL_HCI_CMD_ALLOC(HCI_LE_SET_PER_ADV_REC_EN_CMD_OPCODE, hci_le_set_per_adv_rec_en_cmd);

                if(p_cmd == NULL)
                {
                    status = GAP_ERR_INSUFF_RESOURCES;
                    break;
                }

                p_cmd->sync_hdl = p_actv->sync_hdl;
                p_cmd->en       =    GETB(p_actv->info_bf, GAPM_PER_SYNC_BIG_INFO_REPORT_EN)
                                  || GETB(p_actv->info_bf, GAPM_PER_SYNC_ADV_REPORT_EN);
                HL_HCI_CMD_SEND_TO_CTRL(p_cmd, HL_PROC_FINISHED, gapm_per_sync_hci_cmd_cmp_handler);

            } break;
            default:
            {
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


#if (HL_LE_OBSERVER && (BLE_AOD | BLE_AOA))
/// State machine of IQ report control
__STATIC uint16_t gapm_per_sync_iq_report_ctrl_transition(gapm_per_sync_actv_t* p_actv, gapm_per_sync_proc_iq_report_ctrl_t* p_proc,
                                                          uint8_t event, uint16_t status, bool* p_finished)
{
    *p_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                struct hci_le_set_conless_iq_sampl_en_cmd* p_cmd =
                        HL_HCI_CMD_ALLOC(HCI_LE_SET_CONLESS_IQ_SAMPL_EN_CMD_OPCODE, hci_le_set_conless_iq_sampl_en_cmd);

                if(p_cmd == NULL)
                {
                    status = GAP_ERR_INSUFF_RESOURCES;
                    break;
                }

                p_cmd->sync_hdl               = p_actv->sync_hdl;
                p_cmd->sampl_en               = p_proc->enable;
                p_cmd->slot_dur               = p_proc->slot_dur;
                p_cmd->max_sampl_cte          = p_proc->max_sampl_cte;
                p_cmd->switching_pattern_len  = p_proc->switching_pattern_len;
                memcpy(p_cmd->antenna_id, p_proc->antenna_id, p_proc->switching_pattern_len);
                HL_HCI_CMD_SEND_TO_CTRL(p_cmd, HL_PROC_FINISHED, gapm_per_sync_hci_cmd_cmp_handler);

            } break;
            default:
            {
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
#endif // (HL_LE_OBSERVER && (BLE_AOD | BLE_AOA))



__STATIC uint16_t gapm_per_sync_stop_transition(gapm_per_sync_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                                uint16_t status, bool* p_finished)
{
    *p_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                // Command to be sent to the controller depends on current synchronization state
                if (GETB(p_actv->info_bf, GAPM_PER_SYNC_INFO_SYNC))
                {
                    // Keep in mind that the synchronization is over
                    SETB(p_actv->info_bf, GAPM_PER_SYNC_INFO_SYNC, false);
                    // Send a LE Periodic Advertising Terminate Sync command to the controller
                    status = gapm_per_sync_send_hci_le_per_adv_term_sync_cmd(p_actv, HL_PROC_FINISHED);
                }
                #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
                else if (p_actv->hdr.subtype == GAPM_PER_SYNC_TYPE_PAST)
                {
                    // Send command to stop receive PAST from a specific connection
                    status = gapm_per_adv_sync_trans_param_send(p_actv, NO_SYNC, 0, SYNC_TIMEOUT_MIN, 0, HL_PROC_FINISHED,
                                                                gapm_per_sync_hci_cmd_cmp_handler);
                    if(status != GAP_ERR_NO_ERROR) break;
                    gapm_env.past_estab_bf &= ~CO_BIT(p_actv->conidx);
                }
                #endif //(HL_LE_CENTRAL || HL_LE_PERIPHERAL)
                else
                {
                    // Send a LE Periodic Advertising Create Sync Cancel command to the controller
                    status = gapm_per_sync_send_hci_le_per_adv_create_sync_cancel_cmd(p_actv, HL_PROC_FINISHED);
                    if(status != GAP_ERR_NO_ERROR) break;
                    gapm_env.per_sync_estab_actv_idx = GAP_INVALID_ACTV_IDX;
                }
            }
            break;
            default:
            {
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

__STATIC uint16_t gapm_per_sync_delete_transition(gapm_per_sync_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                                  uint16_t status, bool* p_finished)
{
    // Nothing special to be done
    *p_finished = true;
    return (status);
}
#endif //(HL_LE_OBSERVER)


/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapm_per_sync_create(uint32_t dummy, const gapm_per_sync_actv_cb_t* p_cbs, uint8_t* p_actv_idx)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_OBSERVER)
    do
    {
        // Allocated activity structure
        gapm_per_sync_actv_t *p_actv;

        ASSERT_INFO(GAPM_HEAP_ENV_SIZE >= (sizeof(gapm_per_sync_actv_t) + KE_HEAP_MEM_RESERVED),
                    GAPM_HEAP_ENV_SIZE, sizeof(gapm_per_sync_actv_t));

        // Check if supported roles allow to start scanning
        if (!GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_OBSERVER))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Callback sanity check
        if((p_cbs == NULL) || (p_cbs->report_received == NULL) || (p_cbs->established == NULL))
        {
            status = GAP_ERR_MISSING_CALLBACK;
        }

        // Allocate an activity structure
        status = gapm_actv_create(GAPM_ACTV_TYPE_PER_SYNC, GAPM_PER_SYNC_TYPE_GENERAL, dummy,
                                  sizeof(gapm_per_sync_actv_t), &gapm_per_sync_actv_itf, &(p_cbs->actv),
                                  (gapm_actv_t**) &p_actv);
        if(status != GAP_ERR_NO_ERROR) break;
        p_actv->info_bf  = 0;

        gapm_actv_created(&(p_actv->hdr));

        *p_actv_idx = p_actv->hdr.idx;
    } while (0);
    #endif //(HL_LE_OBSERVER)

    return (status);
}


uint16_t gapm_per_sync_start(uint8_t actv_idx, const gapm_per_sync_param_t* p_param)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_OBSERVER)
    do
    {
        gapm_per_sync_actv_t* p_actv = (gapm_per_sync_actv_t*) gapm_actv_get(actv_idx);
        gapm_per_sync_proc_start_t *p_proc;

        if((p_actv == NULL) || (p_actv->hdr.type != GAPM_ACTV_TYPE_PER_SYNC))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        status = GAP_ERR_NO_ERROR;
        // Check periodic synchronization type
        switch(p_param->type)
        {
            case GAPM_PER_SYNC_TYPE_GENERAL:
            case GAPM_PER_SYNC_TYPE_SELECTIVE:
            {
                // Only one synchronization establishment can be started
                if (gapm_env.per_sync_estab_actv_idx != GAP_INVALID_ACTV_IDX)
                {
                    // A periodic synchronization activity in synchronizing state has been found
                    status = GAP_ERR_COMMAND_DISALLOWED;
                }
            } break;

            case GAPM_PER_SYNC_TYPE_PAST:
            {
                #if (HL_LE_PERIPHERAL || HL_LE_CENTRAL)
                uint16_t conhdl = gapc_get_conhdl(p_param->conidx);

                // connection does not exists or PAST sync busy
                if((conhdl == GAP_INVALID_CONHDL) || ((gapm_env.past_estab_bf & CO_BIT(p_param->conidx)) != 0))
                {
                    status = GAP_ERR_COMMAND_DISALLOWED;
                }

                #else // !(HL_LE_PERIPHERAL || HL_LE_CENTRAL)
                status = GAP_ERR_NOT_SUPPORTED;
                #endif //(HL_LE_PERIPHERAL || HL_LE_CENTRAL)
            } break;

            default: { status = GAP_ERR_INVALID_PARAM; } break;
        }
        if(status != GAP_ERR_NO_ERROR) break;

        // create start procedure
        status = gapm_actv_start(&(p_actv->hdr), sizeof(gapm_per_sync_proc_start_t), (gapm_actv_proc_t**)&p_proc);
        if(status != GAP_ERR_NO_ERROR) break;
        p_proc->param    = *p_param;

        // Configure the activity
        p_actv->hdr.subtype = p_param->type;

        #if (HL_LE_PERIPHERAL || HL_LE_CENTRAL)
        if(p_param->type == GAPM_PER_SYNC_TYPE_PAST)
        {
            p_actv->conidx          = p_param->conidx;
            gapm_env.past_estab_bf |= CO_BIT(p_param->conidx);
        }
        else
        #endif //(HL_LE_PERIPHERAL || HL_LE_CENTRAL)
        {
            gapm_env.per_sync_estab_actv_idx = p_actv->hdr.idx;
        }

        // update BIG Info report trigger
        SETB(p_actv->info_bf, GAPM_PER_SYNC_ADV_REPORT_EN,      GETB(p_param->report_en_bf, GAPM_REPORT_ADV_EN));
        SETB(p_actv->info_bf, GAPM_PER_SYNC_BIG_INFO_REPORT_EN, GETB(p_param->report_en_bf, GAPM_REPORT_BIGINFO_EN));
    } while (0);
    #endif // (HL_LE_OBSERVER)

    return (status);
}

uint16_t gapm_per_sync_report_ctrl(uint8_t actv_idx, uint8_t report_en_bf)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_OBSERVER)
    do
    {
        gapm_per_sync_actv_t* p_actv = (gapm_per_sync_actv_t*) gapm_actv_get(actv_idx);
        gapm_actv_proc_t* p_proc;

        // Check if activity exists, is a periodic advertising and is in started and synchronized state
        if(   (p_actv == NULL) || (p_actv->hdr.type != GAPM_ACTV_TYPE_PER_SYNC)
           || !GETB(p_actv->info_bf, GAPM_PER_SYNC_INFO_SYNC))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // update report trigger
        SETB(p_actv->info_bf, GAPM_PER_SYNC_ADV_REPORT_EN,      GETB(report_en_bf, GAPM_REPORT_ADV_EN));
        SETB(p_actv->info_bf, GAPM_PER_SYNC_BIG_INFO_REPORT_EN, GETB(report_en_bf, GAPM_REPORT_BIGINFO_EN));

        // create start procedure
        status = gapm_actv_proc_create(&(p_actv->hdr), GAPM_ACTV_PERIOD_REPORT_CTRL, sizeof(gapm_actv_proc_t), true,
                                       (gapm_actv_proc_transition_cb) gapm_per_sync_report_ctrl_transition, &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        p_proc->actv_idx  = actv_idx;

        // clear fragment since reports no more expected
        if((!GETB(report_en_bf, GAPM_REPORT_ADV_EN)) && (p_actv->p_report != NULL))
        {
            co_buf_release(p_actv->p_report);
            p_actv->p_report = NULL;
        }
    } while (0);
    #endif // (HL_LE_OBSERVER)

    return (status);
}

uint16_t gapm_per_sync_iq_report_ctrl(uint8_t actv_idx, uint8_t enable, uint8_t slot_dur, uint8_t max_sampl_cte,
                                     uint8_t switching_pattern_len, const uint8_t* p_antenna_id)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_OBSERVER && (BLE_AOD | BLE_AOA))
    do
    {
        gapm_per_sync_actv_t* p_actv = (gapm_per_sync_actv_t*) gapm_actv_get(actv_idx);
        gapm_per_sync_proc_iq_report_ctrl_t *p_proc;
        uint16_t size = sizeof(gapm_per_sync_proc_iq_report_ctrl_t) + switching_pattern_len;

        // Check if activity exists, is a periodic advertising and is in started and synchronized state
        if(   (p_actv == NULL) || (p_actv->hdr.type != GAPM_ACTV_TYPE_PER_SYNC)
           || !GETB(p_actv->info_bf, GAPM_PER_SYNC_INFO_SYNC))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // parameter check
        if(p_antenna_id == NULL)
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Ensure that switching pattern length is valid
        if(switching_pattern_len >= MAX_SWITCHING_PATTERN_LEN)
        {
            status = LL_ERR_INVALID_HCI_PARAM;
            break;
        }
        // create start procedure
        status = gapm_actv_proc_create(&(p_actv->hdr), GAPM_ACTV_PERIOD_IQ_REPORT_CTRL, size, true,
                                       (gapm_actv_proc_transition_cb) gapm_per_sync_iq_report_ctrl_transition,
                                       (gapm_actv_proc_t**)&p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        p_proc->enable                = enable;
        p_proc->slot_dur              = slot_dur;
        p_proc->max_sampl_cte         = max_sampl_cte;
        p_proc->switching_pattern_len = switching_pattern_len;
        memcpy(p_proc->antenna_id, p_antenna_id, switching_pattern_len);
    } while (0);
    #endif // (HL_LE_OBSERVER && (BLE_AOD | BLE_AOA))

    return (status);
}

uint16_t gapm_per_sync_hdl_get(uint8_t actv_idx)
{
    uint16_t per_sync_hdl = GAP_INVALID_CONHDL;

    #if (HL_LE_OBSERVER)
    gapm_per_sync_actv_t *p_actv = (gapm_per_sync_actv_t *)gapm_actv_get(actv_idx);

    // Check that activity is well an advertising activity
    if ((p_actv != NULL) && (p_actv->hdr.type == GAPM_ACTV_TYPE_PER_SYNC))
    {
        per_sync_hdl = p_actv->sync_hdl;
    }
    #endif // (HL_LE_OBSERVER)

    return (per_sync_hdl);
}

/// @} GAPM_PER_SYNC
