/**
 ****************************************************************************************
 *
 * @file gapm_scan.c
 *
 * @brief Generic Access Profile Manager - Scanning manager module.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_SCAN Generic Access Profile Manager - Scanning manager module.
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "gapm_le_scan.h"
#if (HL_LE_OBSERVER)
#include "gapm_int.h"


#include <string.h>
#include "gap.h"
#include "ke_mem.h"
#include "hl_hci.h"
#include "hci.h"
#include "co_utils.h"
#include "co_math.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Bit field bit positions for scanning activity info parameter
enum gapm_scan_info_bf
{
    /// Indicate that application accepts uploading of truncated reports
    GAPM_SCAN_INFO_ACCEPT_TRUNCATED_POS = 0,
    GAPM_SCAN_INFO_ACCEPT_TRUNCATED_BIT = CO_BIT(GAPM_SCAN_INFO_ACCEPT_TRUNCATED_POS),
};

/// Activity start procedure states
enum gapm_scan_start_proc_event
{
    /// Received HCI_LE_SET_RAND_ADDR_CMD complete event
    GAPM_SCAN_PROC_START_HCI_SET_RAND_ADDR_CMP = HL_PROC_EVENT_FIRST,
    /// Received HCI_LE_SET_EXT_SCAN_PARAM_CMD complete event
    GAPM_SCAN_PROC_START_HCI_SET_PARAM_CMP,
    /// Received HCI_LE_SET_EXT_SCAN_EN_CMD complete event
    GAPM_SCAN_PROC_START_HCI_ENABLE_CMP,
};

/*
 * TYPES DEFINITIONS
 ****************************************************************************************
 */

/// GAP Manager activity structure for scanning activity
typedef struct gapm_scan_actv
{
    /// Activity inherited parameters
    gapm_actv_t        hdr;
    /// Information bit field, meaning depends on activity type
    uint8_t            info_bf;
    /// Lists containing fragments for GAPM_REPORT_NB_MAX reports that can be received in parallel
    co_buf_t*          p_reports[GAPM_REPORT_NB_MAX];
    /// Scan filtering Array
    gap_bdaddr_t      *p_scan_filter;
} gapm_scan_actv_t;

/// LE Scan activity start procedure structure
typedef struct gapm_scan_proc_start
{
    /// Inherited from default procedure object
    gapm_actv_proc_t  hdr;
    /// Scan start parameters
    gapm_scan_param_t param;
} gapm_scan_proc_start_t;

/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */
__STATIC uint16_t gapm_scan_start_transition(gapm_scan_actv_t* p_actv, gapm_scan_proc_start_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_scan_stop_transition(gapm_scan_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_scan_delete_transition(gapm_scan_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC uint16_t gapm_scan_addr_renew_transition(gapm_scan_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status, bool* p_finished);
__STATIC void gapm_scan_clean(gapm_scan_actv_t* p_actv, bool reset);

__STATIC void gapm_scan_hci_cmd_cmp_handler(uint16_t opcode, uint16_t event, struct hci_basic_cmd_cmp_evt const *p_evt);

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Activity object interface
__STATIC const gapm_actv_itf_t gapm_scan_actv_itf =
{
    .cb_clean                  = (gapm_actv_clean_cb)           gapm_scan_clean,
    .cb_start_transition       = (gapm_actv_proc_transition_cb) gapm_scan_start_transition,
    .cb_stop_transition        = (gapm_actv_proc_transition_cb) gapm_scan_stop_transition,
    .cb_delete_transition      = (gapm_actv_proc_transition_cb) gapm_scan_delete_transition,
    .cb_addr_renew_transition  = (gapm_actv_proc_transition_cb) gapm_scan_addr_renew_transition,
};


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Check scanning parameters provided by the application.
 *
 * @param[in] p_scan_param      Pointer to the scanning parameters
 *
 * @return GAP_ERR_NO_ERROR if parameters are valid, else GAP_ERR_INVALID_PARAM
 ****************************************************************************************
 */
__STATIC uint16_t gapm_scan_check_param(const gapm_scan_param_t *p_scan_param)
{
    // Error code, invalid parameter by default
    uint16_t status = GAP_ERR_INVALID_PARAM;

    do
    {
        // Check scan type
        if (p_scan_param->type > GAPM_SCAN_TYPE_SEL_CONN_DISC)
        {
            break;
        }

        // Check filter policy for duplicated packets
        if (p_scan_param->dup_filt_pol > GAPM_DUP_FILT_EN_PERIOD)
        {
            break;
        }

        status = GAP_ERR_NO_ERROR;
    } while (0);

    return (status);
}


/**
 ****************************************************************************************
 * @brief Send a LE Set Extended Scan Parameters command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_scan_handler function.
 *
 * @param[in] p_actv        Pointer to the activity structure
 * @param[in] p_param       Pointer to the scan parameters
 * @param[in] event         Event type receive that induce procedure state transition
 ****************************************************************************************
 */
__STATIC uint16_t gapm_scan_send_hci_le_set_ext_scan_param_cmd(gapm_scan_actv_t *p_actv, const gapm_scan_param_t *p_param,
                                                               uint8_t event)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    // Allocate HCI command message
    struct hci_le_set_ext_scan_param_cmd *p_cmd =
            HL_HCI_CMD_ALLOC(HCI_LE_SET_EXT_SCAN_PARAM_CMD_OPCODE, hci_le_set_ext_scan_param_cmd);

    if(p_cmd != NULL)
    {
        uint8_t phy_index = 0;
        status = GAP_ERR_NO_ERROR;
        // Fill the command parameters
        p_cmd->own_addr_type = gapm_le_actv_get_hci_own_addr_type(gapm_env.scan_init_own_addr_type);
        p_cmd->scan_filt_policy = 0;

        // Check if white list must be used
        if ((p_actv->hdr.subtype == GAPM_SCAN_TYPE_SEL_OBSERVER)
                || (p_actv->hdr.subtype == GAPM_SCAN_TYPE_SEL_CONN_DISC))
        {
            p_cmd->scan_filt_policy |= SCAN_ALLOW_ADV_WLST;
        }

        if (gapm_env.scan_init_own_addr_type == GAPM_GEN_RSLV_ADDR)
        {
            // Check if received directed advertising packets with a non resolved target address must be
            // sent to the host
            if (p_param->prop & GAPM_SCAN_PROP_ACCEPT_RPA_BIT)
            {
                p_cmd->scan_filt_policy |= SCAN_ALLOW_ADV_ALL_AND_INIT_RPA;
            }
        }

        p_cmd->scan_phys = 0;

        // Check if scan has to be done on LE 1M PHY
        if (p_param->prop & GAPM_SCAN_PROP_PHY_1M_BIT)
        {
            p_cmd->scan_phys |= PHY_1MBPS_BIT;

            p_cmd->phy[phy_index].scan_intv   = p_param->scan_param_1m.scan_intv;
            p_cmd->phy[phy_index].scan_window = p_param->scan_param_1m.scan_wd;
            p_cmd->phy[phy_index].scan_type   = (p_param->prop & GAPM_SCAN_PROP_ACTIVE_1M_BIT) ? SCAN_ACTIVE : SCAN_PASSIVE;

            // Increase index
            phy_index++;
        }

        // Check if scan has to be done on LE 1M Coded
        if (p_param->prop & GAPM_SCAN_PROP_PHY_CODED_BIT)
        {
            p_cmd->scan_phys |= PHY_CODED_BIT;

            p_cmd->phy[phy_index].scan_intv   = p_param->scan_param_coded.scan_intv;
            p_cmd->phy[phy_index].scan_window = p_param->scan_param_coded.scan_wd;
            p_cmd->phy[phy_index].scan_type   = (p_param->prop & GAPM_SCAN_PROP_ACTIVE_CODED_BIT) ? SCAN_ACTIVE : SCAN_PASSIVE;
        }

        // Send the command
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, event, gapm_scan_hci_cmd_cmp_handler);
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Send a LE Set Extended Scan Enable command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_scan_handler function.
 *
 * @param[in] p_actv    Pointer to the activity structure
 * @param[in] p_param        Pointer to the scan parameters
 * @param[in] enable         True to enable scan, False to disable
 * @param[in] event          Event type receive that induce procedure state transition
 ****************************************************************************************
 */
__STATIC uint16_t gapm_scan_send_hci_le_set_ext_scan_en_cmd(gapm_scan_actv_t *p_actv, const gapm_scan_param_t *p_param,
                                                            bool enable, uint8_t event)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    // Allocate HCI command message
    struct hci_le_set_ext_scan_en_cmd *p_cmd =
            HL_HCI_CMD_ALLOC(HCI_LE_SET_EXT_SCAN_EN_CMD_OPCODE, hci_le_set_ext_scan_en_cmd);

    if(p_cmd != NULL)
    {
        status = GAP_ERR_NO_ERROR;
        // Fill command parameters
        p_cmd->scan_en = (uint8_t)enable;

        if(enable)
        {
            p_cmd->filter_duplic = p_param->dup_filt_pol;

            if (p_actv->hdr.subtype == GAPM_SCAN_TYPE_LIM_DISC)
            {
                if(p_param->duration > 0)
                {
                    p_cmd->duration = co_max(p_param->duration, (p_param->prop & GAPM_SCAN_PROP_PHY_1M_BIT)
                                                                ? GAP_TMR_LIM_DISC_SCAN_1M
                                                                : GAP_TMR_LIM_DISC_SCAN_CODED);
                }
                else // keep it infinitely active
                {
                    p_cmd->duration = 0;
                }

                p_cmd->period = 0;
            }
            else if (p_actv->hdr.subtype == GAPM_SCAN_TYPE_GEN_DISC)
            {
                if(p_param->duration > 0)
                {
                    p_cmd->duration = co_max(p_param->duration, (p_param->prop & GAPM_SCAN_PROP_PHY_1M_BIT)
                                                                ? GAP_TMR_GEN_DISC_SCAN_1M
                                                                : GAP_TMR_GEN_DISC_SCAN_CODED);
                }
                else // keep it infinitely active
                {
                    p_cmd->duration = 0;
                }

                p_cmd->period   = 0;
            }
            else
            {
                p_cmd->duration = p_param->duration;
                p_cmd->period   = p_param->period;
            }
        }

        // Send the command
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, event, gapm_scan_hci_cmd_cmp_handler);
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Clean content of a scan activity, more specifically the stored fragments
 * of advertising report and the filtering list.
 *
 * @param[in] p_actv    Pointer to the scanning activity structure.
 * @param[in] reset     True if a reset is on-going, False otherwise
 ****************************************************************************************
 */
__STATIC void gapm_scan_clean(gapm_scan_actv_t *p_actv, bool reset)
{

    if (p_actv->p_scan_filter)
    {
        ke_free(p_actv->p_scan_filter);
    }

    if(!reset)
    {
        // remove reports
        uint8_t report_idx;
        for (report_idx = 0; report_idx < GAPM_REPORT_NB_MAX; report_idx++)
        {
            if(p_actv->p_reports[report_idx] != NULL)
            {
                co_buf_release(p_actv->p_reports[report_idx]);
            }
        }
    }

    // call inherited destructor
    gapm_actv_clean(&(p_actv->hdr), reset);
}


/**
 ****************************************************************************************
 * @brief Look for AD Type flag value in a received advertising report.
 *
 * @param[in] p_data    Pointer to the received advertising report
 * @param[in] length    Length of the received advertising report
 *
 * @return 0 if AD Type flag has not been found else AD Type flag value
 ****************************************************************************************
 */
__STATIC uint8_t gapm_scan_get_ad_type_flag(co_buf_t* p_buf)
{
    uint8_t* p_data = co_buf_data(p_buf);
    uint8_t* p_data_end = co_buf_tail(p_buf);

    uint8_t ad_flag = 0;

    // Parse advertising data and look for ad_type
    while (p_data < p_data_end)
    {
        // Check if it's AD Type flag data
        if (*(p_data + 1) == GAP_AD_TYPE_FLAGS)
        {
            // Keep flag value
            ad_flag = *(p_data + 2);
            break;
        }

        // Go to next advertising info
        p_data += (*p_data + 1);
    }

    return (ad_flag);
}

/**
 ****************************************************************************************
 * @brief Add device to list of filtered devices
 *
 * @param[in] p_addr     Device address
 * @param[in] addr_type  Device address type
 ****************************************************************************************
 */
__STATIC void gapm_scan_add_to_filter(gapm_scan_actv_t *p_scan_actv,
                                      gap_addr_t *p_addr, uint8_t addr_type)
{
    gap_addr_t default_addr = {{0,0,0,0,0,0}};
    uint8_t cursor = 0;

    // Allocate scan filtered device list if needed.
    if (p_scan_actv->p_scan_filter == NULL)
    {
        p_scan_actv->p_scan_filter =
                (struct gap_bdaddr*)ke_malloc_user(sizeof(struct gap_bdaddr) * GAPM_SCAN_FILTER_SIZE, KE_MEM_KE_MSG);
        if (p_scan_actv->p_scan_filter != NULL)
        {
            memset(p_scan_actv->p_scan_filter, 0, sizeof(struct gap_bdaddr) * GAPM_SCAN_FILTER_SIZE);
        }
    }

    if (p_scan_actv->p_scan_filter != NULL)
    {
        // Find first available space in array
        while (   (cursor < GAPM_SCAN_FILTER_SIZE)
               && (memcmp(&(p_scan_actv->p_scan_filter[cursor].addr), &default_addr, sizeof(gap_addr_t)) != 0))
        {
            cursor++;
        }

        // Copy provided device address in array
        if (cursor < GAPM_SCAN_FILTER_SIZE)
        {
            memcpy(&(p_scan_actv->p_scan_filter[cursor].addr), p_addr, GAP_BD_ADDR_LEN);
            p_scan_actv->p_scan_filter[cursor].addr_type = addr_type;
        }
    }
    // else don't put device into filter.
}

/**
 ****************************************************************************************
 * @brief Check if device is filtered or not when a scan response data is received
 *
 * If device is not filtered (present in filter list), in that case function returns false
 * and device is removed from filtered devices.
 *
 * @param[in] p_actv     Pointer to activity structure
 * @param[in] p_addr     Device address
 * @param[in] addr_type  Device address type
 *
 * @return true if device filtered, else false.
 ****************************************************************************************
 */
__STATIC bool gapm_scan_accept_scan_rsp(gapm_scan_actv_t *p_actv,
                                        gap_addr_t *p_addr, uint8_t addr_type)
{
    bool ret = false;
    uint8_t cursor = 0;

    // Check that filter array exists
    if (p_actv->p_scan_filter != NULL)
    {
        // Find provided address in filter
        while (((memcmp(&(p_actv->p_scan_filter[cursor].addr), p_addr, sizeof(gap_addr_t)) != 0)
                    || (p_actv->p_scan_filter[cursor].addr_type != addr_type))
                        && (cursor <  GAPM_SCAN_FILTER_SIZE))
        {
            cursor++;
        }

        // Copy provided device address in array
        if (cursor < GAPM_SCAN_FILTER_SIZE)
        {
            ret = true;

            // Remove device from filter.
            memset(&(p_actv->p_scan_filter[cursor].addr), 0, GAP_BD_ADDR_LEN);
        }
    }
    return ret;
}

/**
 ****************************************************************************************
 * @brief Check if received advertising and scan response report must be filtered or
 * not.
 *
 * @param[in] p_actv            Pointer to activity structure
 * @param[in] p_buf             Buffer that contains advertising report
 * @param[in] p_last_adv_report Last received fragment information
 ****************************************************************************************
 */
__STATIC bool gapm_scan_filter_packet(gapm_scan_actv_t *p_actv, co_buf_t* p_buf,
                                      struct ext_adv_report *p_last_adv_report)
{
    // Return status, indicate if packet has to be filtered
    bool status = false;

    do
    {
        // Received AD type flag
        uint8_t ad_type;

        if (p_actv->hdr.subtype >= GAPM_SCAN_TYPE_OBSERVER)
        {
            break;
        }

        // Check if reception of advertising data triggered filtering of scan response data
        if (p_last_adv_report->evt_type & SCAN_RSP_EVT_MSK)
        {
            if ((p_last_adv_report->evt_type & LGCY_ADV_EVT_MSK) ||
                (!(p_last_adv_report->evt_type & LGCY_ADV_EVT_MSK) && !(p_last_adv_report->evt_type & SCAN_ADV_EVT_MSK)))
            {
                // Check if reception of scan response is filtered
                status = !gapm_scan_accept_scan_rsp(p_actv,
                                                    (gap_addr_t *)(&p_last_adv_report->adv_addr),
                                                    p_last_adv_report->adv_addr_type);
                break;
            }
        }

        // Retrieve AD Type flag
        ad_type = gapm_scan_get_ad_type_flag(p_buf);

        // Check for limited discovery that we have received limited discoverable data
        if(  (GETB(ad_type, GAP_LE_LIM_DISCOVERABLE_FLG) && (p_actv->hdr.subtype == GAPM_SCAN_TYPE_LIM_DISC))
        // or For general discovery that one of limited or general discoverable, flag is present in ad_type flag
           || (   ((GETB(ad_type, GAP_LE_GEN_DISCOVERABLE_FLG)) || (GETB(ad_type, GAP_LE_LIM_DISCOVERABLE_FLG)))
               && (p_actv->hdr.subtype == GAPM_SCAN_TYPE_GEN_DISC)))
        {
            if (!(p_last_adv_report->evt_type & SCAN_RSP_EVT_MSK))
            {
                // Add device to filtering list, scan response will be accepted
                gapm_scan_add_to_filter(p_actv,
                                        (gap_addr_t*)&(p_last_adv_report->adv_addr),
                                        p_last_adv_report->adv_addr_type);
            }
        }
        else
        {
            status = true;
        }
    } while (0);

    return (status);
}


/**
 ****************************************************************************************
 * @brief Send a GAPM_EXT_ADV_REPORT_IND message to the application. Used either once
 * all fragments of an advertising or a scan response report have been received
 * or once latest fragment of a truncated report has been received (if forwarding
 * or truncated reports has been enabled by the application when scan activity has been
 * started).
 *
 * @param[in] p_actv       Pointer to the scan activity structure
 * @param[in] report_idx   Report index where identity address can be found
 * @param[in] p_adv_report Pointer to the last received fragment
 * @param[in] complete     Indicate if report is complete (true) or truncated.
 ****************************************************************************************
 */
__STATIC void gapm_scan_send_adv_report(gapm_scan_actv_t *p_actv, uint8_t report_idx,
                                        struct ext_adv_report *p_adv_report, bool complete)
{
    co_buf_t* p_report = p_actv->p_reports[report_idx];

    // Check if packet must be filtered
    if (!gapm_scan_filter_packet(p_actv, p_report, p_adv_report))
    {
        const gapm_scan_actv_cb_t* p_cbs = (const gapm_scan_actv_cb_t*) p_actv->hdr.p_cbs;
        gapm_adv_report_info_t* p_meta = (gapm_adv_report_info_t*) co_buf_metadata(p_report);

        // Fill the message
        if (p_adv_report->evt_type & SCAN_RSP_EVT_MSK)
        {
            p_meta->info = (p_adv_report->evt_type & LGCY_ADV_EVT_MSK)
                        ? GAPM_REPORT_TYPE_SCAN_RSP_LEG : GAPM_REPORT_TYPE_SCAN_RSP_EXT;
        }
        else
        {
            p_meta->info = (p_adv_report->evt_type & LGCY_ADV_EVT_MSK)
                        ? GAPM_REPORT_TYPE_ADV_LEG : GAPM_REPORT_TYPE_ADV_EXT;
        }

        if (p_adv_report->evt_type & CON_ADV_EVT_MSK)
        {
            p_meta->info |= GAPM_REPORT_INFO_CONN_ADV_BIT;
        }

        if (p_adv_report->evt_type & SCAN_ADV_EVT_MSK)
        {
            p_meta->info |= GAPM_REPORT_INFO_SCAN_ADV_BIT;
        }

        if (p_adv_report->evt_type & DIR_ADV_EVT_MSK)
        {
            p_meta->info |= GAPM_REPORT_INFO_DIR_ADV_BIT;
        }

        if (complete)
        {
            p_meta->info |= GAPM_REPORT_INFO_COMPLETE_BIT;
        }

        p_meta->rssi                 = p_adv_report->rssi;
        p_meta->tx_pwr               = p_adv_report->tx_power;
        p_meta->phy_prim             = p_adv_report->phy;
        p_meta->phy_second           = p_adv_report->phy2;
        p_meta->adv_sid              = p_adv_report->adv_sid;
        p_meta->period_adv_intv      = p_adv_report->interval;

        // Transmitter address - already filled

        // Target address
        if (p_adv_report->evt_type & DIR_ADV_EVT_MSK)
        {
            p_meta->target_addr.addr_type = (p_adv_report->dir_addr_type & ADDR_MASK);
            memcpy(&p_meta->target_addr.addr[0], &p_adv_report->dir_addr.addr[0], GAP_BD_ADDR_LEN);
        }
        else
        {
            memset(&(p_meta->target_addr), 0, sizeof(gap_bdaddr_t));
        }

        // Push report to upper layer software
        p_cbs->report_received(p_actv->hdr.dummy, p_actv->hdr.idx, p_meta, p_report);
    }

    // release report
    co_buf_release(p_report);
    p_actv->p_reports[report_idx] = NULL;
}



/**
 ****************************************************************************************
 * @brief Store received fragment of advertising or scan response report.
 *
 * @param[in] p_actv        Pointer to the scan activity for which fragment has been received.
 * @param[in] report_idx    Index where report has to be inserted. Can be GAPM_INVALID_IDX if fragment is the first one.
 * @param[in] p_adv_report  Pointer to the received fragment.
 *
 * @return True if fragment has been properly stored, False otherwise
 ****************************************************************************************
 */
__STATIC bool gapm_scan_store_fragment(gapm_scan_actv_t *p_actv, uint8_t report_idx,
                                       struct ext_adv_report *p_adv_report)
{
    bool succeed = false;

    if (report_idx != GAP_INVALID_IDX)
    {
        succeed = gapm_scan_append(&(p_actv->p_reports[report_idx]), p_adv_report->data_len, p_adv_report->data);

        if (succeed)
        {
            gapm_adv_report_info_t* p_meta = (gapm_adv_report_info_t*) co_buf_metadata(p_actv->p_reports[report_idx]);
            // store advertiser identity
            p_meta->trans_addr.addr_type = p_adv_report->adv_addr_type;
            if(p_adv_report->adv_addr_type != ADDR_NONE)
            {
                p_meta->trans_addr.addr_type &= ADDR_MASK;
            }
            memcpy(&(p_meta->trans_addr.addr), &p_adv_report->adv_addr.addr[0], GAP_BD_ADDR_LEN);

            succeed = true;
        }
    }

    return (succeed);
}

/**
 ****************************************************************************************
 * @brief Find a slot where advertising report can be stored. If a report already received for specific address,
 *        can be reused for new report
 *
 * @param[in]  p_actv        Pointer to the scan activity structure
 * @param[in]  p_adv_report  Pointer to the received advertising report fragment. Contains the advertiser's address.
 * @param[out] pp_buf        Pointer to returned buffer
 *
 * @return Index containing previously received fragment of the report
 ****************************************************************************************
 */
__STATIC uint8_t gapm_scan_find_report_slot(gapm_scan_actv_t *p_actv, struct ext_adv_report *p_adv_report, co_buf_t** pp_buf)
{
    uint8_t report_idx = GAPM_REPORT_NB_MAX;
    uint8_t idx;

    for (idx = 0; idx < GAPM_REPORT_NB_MAX; idx++)
    {
        gapm_adv_report_info_t* p_meta;
        // Get report structure
        co_buf_t* p_report = p_actv->p_reports[idx];
        if(p_report == NULL)
        {
            report_idx = idx; // keep available slot
            continue;
        }

        p_meta = (gapm_adv_report_info_t*) co_buf_metadata(p_report);

        // Compare advertiser address
        if (   (p_meta->trans_addr.addr_type == p_adv_report->adv_addr_type)
            && !memcmp(p_meta->trans_addr.addr, &p_adv_report->adv_addr.addr[0], GAP_BD_ADDR_LEN))
        {
            *pp_buf = p_report;
            report_idx = idx; // keep available slot
            break;
        }
    }

    return (report_idx);
}

/*
 * HCI HANDLERS DEFINITIONS
 ****************************************************************************************
 */

/// Default HCI Command complete handler used to continue procedure execution
__STATIC void gapm_scan_hci_cmd_cmp_handler(uint16_t opcode, uint16_t event, struct hci_basic_cmd_cmp_evt const *p_evt)
{
    gapm_proc_transition(GAPM_PROC_AIR, (uint8_t) event, RW_ERR_HCI_TO_HL(p_evt->status));
}



/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Extended Advertising Report Event
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void gapm_hci_le_ext_adv_report_evt_handler(uint8_t evt_code, struct hci_le_ext_adv_report_evt *p_evt)
{
    // Get index of started scanning activity
    uint8_t actv_idx = gapm_env.scan_actv_idx;
    // Get activity structure
    gapm_scan_actv_t *p_actv = (gapm_scan_actv_t *) gapm_actv_get(actv_idx);

    if (p_actv != NULL)
    {
        for (uint8_t loop = 0; loop < p_evt->nb_reports; loop++)
        {
            // Get report information
            struct ext_adv_report *p_adv_report = &p_evt->adv_rep[loop];
            uint8_t report_idx;
            // Data status
            uint8_t data_status;
            bool truncated;
            co_buf_t* p_report = NULL;

            // If only connectable discovery, only accept connectable advertising report
            if (p_actv->hdr.subtype >= GAPM_SCAN_TYPE_CONN_DISC)
            {
                if (!(p_adv_report->evt_type & CON_ADV_EVT_MSK))
                {
                    continue;
                }
            }

            // Check if fragments are currently stored for the advertiser address
            report_idx = gapm_scan_find_report_slot(p_actv, p_adv_report, &p_report);


            // Extract data status
            data_status = (uint8_t) GETF(p_adv_report->evt_type, ADV_EVT_DATA_STATUS);
            truncated   = (data_status == ADV_EVT_DATA_STATUS_TRUNCATED);


            // if advertising flow off or truncated buffer not supported by application, remove already received fragments
            if(   gapm_env.adv_report_flow_off
               || (truncated && !GETB(p_actv->info_bf, GAPM_SCAN_INFO_ACCEPT_TRUNCATED)))
            {
                if(p_report != NULL)
                {
                    co_buf_release(p_report);
                    p_actv->p_reports[report_idx] = NULL;
                }
            }
            // store received fragment
            else if(gapm_scan_store_fragment(p_actv, report_idx, p_adv_report))
            {
                // If no more fragment will come because report is complete or truncated, upload the report
                if ((data_status == ADV_EVT_DATA_STATUS_COMPLETE) || truncated)
                {
                    // Send the report to the application
                    gapm_scan_send_adv_report(p_actv, report_idx, p_adv_report, !truncated);
                }
            }
            // else drop the fragment
        }
    }
}

/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Scan Timeout Event
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void gapm_hci_le_scan_timeout_evt_handler(uint8_t evt_code, struct hci_le_scan_timeout_evt const *p_evt)
{
    // Get index of started scanning activity
    uint8_t actv_idx = gapm_env.scan_actv_idx;

    if (actv_idx != GAPM_ACTV_INVALID_IDX)
    {
        gapm_env.scan_actv_idx = GAPM_ACTV_INVALID_IDX;
        gapm_actv_stopped(gapm_actv_get(actv_idx), GAP_ERR_TIMEOUT);
    }
}

/*
 * PROCEDURE TRANSITION - State machine
 ****************************************************************************************
 */

/// Scan activity start transition state machine
__STATIC uint16_t gapm_scan_start_transition(gapm_scan_actv_t* p_actv, gapm_scan_proc_start_t* p_proc, uint8_t event,
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
                    #if(HL_LE_CENTRAL)
                    // set random address only if init isn't active
                    if(gapm_env.init_actv_idx == GAP_INVALID_ACTV_IDX)
                    #endif // (HL_LE_CENTRAL)
                    {
                        status = gapm_le_actv_addr_send_hci_le_set_rand_addr_cmd(GAPM_SCAN_PROC_START_HCI_SET_RAND_ADDR_CMP,
                                                                         (hl_hci_cmd_evt_func_t)gapm_scan_hci_cmd_cmp_handler);
                        break;
                    }
                }
            }
            // no break;
            case GAPM_SCAN_PROC_START_HCI_SET_RAND_ADDR_CMP:
            {
                status = gapm_scan_send_hci_le_set_ext_scan_param_cmd(p_actv, &(p_proc->param), GAPM_SCAN_PROC_START_HCI_SET_PARAM_CMP);
            } break;
            case GAPM_SCAN_PROC_START_HCI_SET_PARAM_CMP:
            {
                status = gapm_scan_send_hci_le_set_ext_scan_en_cmd(p_actv, &(p_proc->param), true,
                                                                   GAPM_SCAN_PROC_START_HCI_ENABLE_CMP);
            } break;
            case GAPM_SCAN_PROC_START_HCI_ENABLE_CMP:
            default:
            {
                *p_finished = true;
            } break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        // Scan has not been started properly
        gapm_env.scan_actv_idx = GAPM_ACTV_INVALID_IDX;
        *p_finished = true;
    }

    return (status);
}

/// Scan activity stop transition state machine
__STATIC uint16_t gapm_scan_stop_transition(gapm_scan_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                            uint16_t status, bool* p_finished)
{
    *p_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                // Send a LE Set Extended Scan Enable command (Disable) to the controller
                status = gapm_scan_send_hci_le_set_ext_scan_en_cmd(p_actv, NULL, false, HL_PROC_FINISHED);
            }
            break;
            default:
            {
                *p_finished = true;
                gapm_env.scan_actv_idx = GAPM_ACTV_INVALID_IDX;
            } break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        *p_finished = true;
    }

    return (status);
}

/// Scan activity delete transition state machine
__STATIC uint16_t gapm_scan_delete_transition(gapm_scan_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                              uint16_t status, bool* p_finished)
{
    // Nothing special to be done
    *p_finished = true;
    return (status);
}

/// Scan activity address renew transition state machine
__STATIC uint16_t gapm_scan_addr_renew_transition(gapm_scan_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
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
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

bool gapm_scan_append(co_buf_t** pp_buf, uint16_t length, const uint8_t* p_data)
{
    bool succeed = false;

    if(*pp_buf == NULL)
    {
        co_buf_alloc(pp_buf, 0, 0, length);
    }
    else if(co_buf_tail_len(*pp_buf) < length)
    {
        co_buf_t* p_old_buf = *pp_buf;
        co_buf_t* p_new_buf = NULL;

        // allocate a bigger buffer to concatenate data
        co_buf_duplicate(p_old_buf, &p_new_buf, 0, length);

        // release old buffer
        co_buf_release(p_old_buf);
        *pp_buf = p_new_buf;
    }


    if (*pp_buf)
    {
        // append fragment
        memcpy(co_buf_tail(*pp_buf), p_data, length);
        co_buf_tail_reserve(*pp_buf, length);

        succeed = true;
    }
    return (succeed);
}
#endif //(HL_LE_OBSERVER)


uint16_t gapm_scan_create(uint32_t dummy, uint8_t own_addr_type, const gapm_scan_actv_cb_t* p_cbs, uint8_t* p_actv_idx)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_OBSERVER)
    do
    {
        // Allocated activity structure
        gapm_scan_actv_t *p_actv;

        ASSERT_INFO(GAPM_HEAP_ENV_SIZE >= (sizeof(gapm_scan_actv_t) + KE_HEAP_MEM_RESERVED),
                    GAPM_HEAP_ENV_SIZE, sizeof(gapm_scan_actv_t));

        // Check if supported roles allow to start scanning
        if (!GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_OBSERVER) && !GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_CENTRAL))
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

        // callback sanity check
        if((p_cbs == NULL) || (p_cbs->report_received == NULL))
        {
            status = GAP_ERR_MISSING_CALLBACK;
            break;
        }

        // Allocate an activity structure
        status = gapm_actv_create(GAPM_ACTV_TYPE_SCAN, GAPM_SCAN_TYPE_GEN_DISC, dummy, sizeof(gapm_scan_actv_t),
                                  &gapm_scan_actv_itf, (const gapm_actv_cb_t*) p_cbs, (gapm_actv_t**) &p_actv);
        if(status != GAP_ERR_NO_ERROR) break;

        p_actv->info_bf = 0;
        gapm_actv_created(&(p_actv->hdr));

        *p_actv_idx = p_actv->hdr.idx;
        gapm_le_actv_addr_scan_init_type_set(own_addr_type);
    } while (0);
    #endif //(HL_LE_OBSERVER)

    return (status);
}


uint16_t gapm_scan_start(uint8_t actv_idx, const gapm_scan_param_t* p_param)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_OBSERVER)
    do
    {
        gapm_scan_actv_t* p_actv = (gapm_scan_actv_t*) gapm_actv_get(actv_idx);
        gapm_scan_proc_start_t *p_proc;

        if((p_actv == NULL) || (p_actv->hdr.type != GAPM_ACTV_TYPE_SCAN))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Cannot have two scanning procedures in parallel
        if (gapm_env.scan_actv_idx != GAPM_ACTV_INVALID_IDX)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Check provided scan parameters
        status = gapm_scan_check_param(p_param);
        if(status != GAP_ERR_NO_ERROR) break;

        // create start procedure
        status = gapm_actv_start(&(p_actv->hdr), sizeof(gapm_scan_proc_start_t), (gapm_actv_proc_t**)&p_proc);
        if(status != GAP_ERR_NO_ERROR) break;
        p_proc->param          = *p_param;
        gapm_env.scan_actv_idx = actv_idx;

        // Configure the activity
        p_actv->hdr.subtype = p_param->type;

        // mark if truncated data can be received
        SETB(p_actv->info_bf, GAPM_SCAN_INFO_ACCEPT_TRUNCATED, !(p_param->prop & GAPM_SCAN_PROP_FILT_TRUNC_BIT));
    } while (0);
    #endif // (HL_LE_OBSERVER)

    return (status);
}

/// @} GAPM_SCAN
