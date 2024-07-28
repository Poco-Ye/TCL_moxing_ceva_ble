/**
 ****************************************************************************************
 *
 * @file hl_hci.c
 *
 * @brief Handlers of HCI Data and Events
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup HOST
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"    // IP Configuration
#include "rwip.h"           // Initialization types
#include "arch.h"           // ASSERT
#include "hci.h"            // HCI Defines
#include "gap.h"            // Default GAP defines
#include "hl_hci.h"         // HCI Host local identifiers
#include "../inc/gap_hl_api.h"   // Get Host configuration state

#include "gapc.h"           // Retrieve connection index from handle

#if (BLE_GAF_PRESENT)
#include "gaf_cfg.h"        // GAF configuration if present
#endif // (BLE_GAF_PRESENT)

#if (BT_HOST_PRESENT)
#include "bk_al.h"          // BK Adaptation layer
#include <string.h>         // memcpy
#endif // (BT_HOST_PRESENT)

#include "co_djob.h"       // Job defer execution
#include "ke_msg.h"        // Controller HCI messages are using kernel message structure
#include "ke_mem.h"        // Allocation of data
#include "co_list.h"       // List manipulation
#include "co_utils.h"      // Read / write on array

#if (EMB_PRESENT && RW_DEBUG)
#include "em_map.h"        // For HCI Dump
#endif // (EMB_PRESENT && RW_DEBUG)

/*
 * DEFINES
 ****************************************************************************************
 */

#if(!BT_EMB_PRESENT)
#define BT_ACL_CONHDL_BIT       (0x80)
#elif (BT_ACL_CONHDL_BIT != 0x80)
#warning "Value of BT Connection handle bit is invalid"
#endif // (!BT_EMB_PRESENT)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Format of a HCI event handler function
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
typedef void (*hl_hci_evt_handler_func_t)(uint8_t evt_code, void const *p_evt);

/// List of handlers
typedef struct hl_hci_evt_hdl_info
{
    /// Number of HCI event handlers
    uint8_t                          nb_handlers;
    /// Pointer to the Event handler array
    const hl_hci_evt_handler_func_t* p_handlers;
} hl_hci_evt_handler_info_t;

/// RW-Host command structure
typedef struct hl_hci_cmd_hdr
{
    /// Used to put element in the queue
    co_list_hdr_t          hdr;
    /// Command Complete or Command status function to call when controller message is received
    hl_hci_cmd_evt_func_t  cmd_evt_cb;
    /// Command Operation Code
    uint16_t               opcode;
    /// Optional parameter used to retrieve command context
    uint16_t               handle;
} hl_hci_cmd_hdr_t;


#if(BT_HOST_PRESENT)
/// BT Host command structure
typedef struct hl_hci_bt_cmd
{
    /// Used to put element in the queue
    co_list_hdr_t   hdr;
    /// Function to call when command is sent
    hl_hci_done_cb  cb_done;
    /// Pointer to the HCI message packed
    uint8_t*        p_buf;
    /// Parameter length
    uint16_t        length;
    /// Parameter length
    uint16_t        opcode;
} hl_hci_bt_cmd_t;
#endif // (BT_HOST_PRESENT)

/// Environement structure
typedef struct hl_hci_env_
{
    #if (BT_HOST_PRESENT)
    /// BT Host command structure
    hl_hci_bt_cmd_t        bt_cmd;
    #endif // (BT_HOST_PRESENT)

    #if (EMB_PRESENT)
    /// Defer handling of event messages
    co_djob_t              evt_djob;
    /// HCI message queue
    co_list_t              evt_queue;
    #endif // (EMB_PRESENT)

    /// Defer command transmission and command status/complete handler execution
    co_djob_t              cmd_djob;
    /// Command queue
    co_list_t              cmd_queue;
    /// Command Complete or Command status function to call when controller message is received
    hl_hci_cmd_evt_func_t  cmd_evt_cb;
    /// Command Handle of latest transmitted command
    uint16_t               cmd_handle;
    /// Command opcode of latest transmitted command
    uint16_t               cmd_opcode;
    /// Command transport layer destination
    uint8_t                cmd_tl_dest;
    /// Wait for a command complete or status event
    bool                   wait_cmd_evt;
    #if (EMB_PRESENT)
    /// Command complete event has been received, it has been put on top of command queue
    bool                   handle_cmd_evt;
    #endif // (EMB_PRESENT)
} hl_hci_env_t;

typedef struct hl_hci_evt_msg
{
    /// list header
    co_list_hdr_t             hdr;
    /// Event handler callback
    hl_hci_evt_handler_func_t cb_handler;
    /// Event code
    uint8_t                   evt_code;
} hl_hci_evt_msg_t;

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
/// Environment
hl_hci_env_t hl_hci_env;

/*
 * LOCAL FUNCTIONS DECLARATIONS
 ****************************************************************************************
 */
#if (BLE_HOST_PRESENT)
#if(HL_LE_CENTRAL || HL_LE_PERIPHERAL)
__STATIC void hl_hci_disc_cmp_evt_handler(uint8_t evt_code, struct hci_disc_cmp_evt const *p_evt);
__STATIC void hl_hci_le_enh_con_cmp_evt_handler(uint8_t evt_code, struct hci_le_enh_con_cmp_evt const *p_evt);
#endif //(HL_LE_CENTRAL || HL_LE_PERIPHERAL)

/*
 * EXTERNAL FUNCTIONS
 ****************************************************************************************
 */

// Handler in gapm_adv.c
#if (HL_LE_BROADCASTER)
extern void gapm_hci_le_adv_set_term_evt_handler(uint8_t evt_code, struct hci_le_adv_set_term_evt const *p_evt);
extern void gapm_hci_le_scan_req_rcvd_evt_handler(uint8_t evt_code, struct hci_le_scan_req_rcvd_evt const *p_evt);
#endif // (HL_LE_BROADCASTER)
#if(HL_LE_PERIPHERAL)
extern void gapm_adv_hci_le_enh_con_cmp_evt_handler(uint8_t evt_code, struct hci_le_enh_con_cmp_evt const *p_evt);
#endif // (HL_LE_PERIPHERAL)

// Handler in gapm_scan.c
extern void gapm_hci_le_ext_adv_report_evt_handler(uint8_t evt_code, struct hci_le_ext_adv_report_evt const *p_evt);
extern void gapm_hci_le_scan_timeout_evt_handler(uint8_t evt_code, struct hci_le_scan_timeout_evt const *p_evt);

// Handler in gapm_init.c
#if(HL_LE_CENTRAL)
extern void gapm_init_hci_le_enh_con_cmp_evt_handler(uint8_t evt_code, struct hci_le_enh_con_cmp_evt const *p_evt);
#endif // (HL_LE_CENTRAL)

// Handler in gapm_period_sync.c
#if (HL_LE_OBSERVER)
extern void gapm_hci_le_per_adv_sync_est_evt_handler(uint8_t evt_code, struct hci_le_per_adv_sync_est_evt const *p_evt);
extern void gapm_hci_le_per_adv_report_evt_handler(uint8_t evt_code, struct hci_le_per_adv_report_evt const *p_evt);
extern void gapm_hci_le_big_info_adv_report_evt_handler(uint8_t evt_code, struct hci_le_big_info_adv_report_evt const *p_evt);
extern void gapm_hci_le_per_adv_sync_lost_evt_handler(uint8_t evt_code, struct hci_le_per_adv_sync_lost_evt const *p_evt);
#if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
extern void gapm_hci_le_per_adv_sync_transf_rec_evt_handler(uint8_t evt_code, struct hci_le_per_adv_sync_transf_rec_evt const *p_evt);
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
#if (BLE_AOD | BLE_AOA)
extern void gapm_per_sync_hci_le_conless_iq_report_evt_handler(uint8_t evt_code, struct hci_le_conless_iq_report_evt const *p_evt);
#endif // (BLE_AOD | BLE_AOA)
#endif //(HL_LE_OBSERVER)

// Handler in gapm_smp.c
extern void gapm_hci_le_rd_local_p256_public_key_cmp_evt_handler(uint8_t evt_code, struct hci_rd_local_p256_public_key_cmp_evt const *p_evt);
extern void gapm_hci_le_gen_dhkey_cmp_evt_handler(uint8_t evt_code, struct hci_le_gen_dhkey_cmp_evt const *p_evt);

#if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
// function handlers present in gapc_le_con.c module
extern void gapc_hci_le_ch_sel_algo_evt_handler(uint8_t evt_code, struct hci_le_ch_sel_algo_evt const *p_evt);
extern void gapc_hci_le_rd_rem_feats_cmp_evt_handler(uint8_t evt_code, struct hci_le_rd_rem_feats_cmd_cmp_evt const *p_evt);

// function handlers present in gapc_disconnect.c module
extern void gapc_le_con_hci_con_disc_cmp_evt_handler(uint8_t evt_code, uint8_t conidx, struct hci_disc_cmp_evt const *p_evt);

// function handlers present in gapc_con_up.c module
extern void gapc_hci_le_con_update_cmp_evt_handler(uint8_t evt_code, struct hci_le_con_update_cmp_evt const *p_evt);
extern void gapc_hci_le_rem_con_param_req_evt_handler(uint8_t evt_code, struct hci_le_rem_con_param_req_evt const *p_evt);

// function handlers present in gapc_le_smp_encrypt.c module
#if (HL_LE_PERIPHERAL)
extern void gapc_hci_le_ltk_request_evt_handler(uint8_t evt_code, struct hci_le_ltk_request_evt const *p_evt);
#endif //(HL_LE_PERIPHERAL)
extern void gapc_hci_enc_chg_evt_handler(uint8_t evt_code, struct hci_enc_change_evt const *p_evt);
extern void gapc_hci_enc_key_refr_evt_handler(uint8_t evt_code, struct hci_enc_key_ref_cmp_evt const *p_evt);

// function handlers present in gapc_le_cte.c module
extern void gapc_hci_le_con_iq_report_evt_handler(uint8_t evt_code, struct hci_le_con_iq_report_evt const *p_evt);
extern void gapc_hci_le_cte_req_failed_evt_handler(uint8_t evt_code, struct hci_le_cte_req_failed_evt const *p_evt);

// function handlers present in gapc_info.c module.
extern void gapc_hci_rd_rem_ver_info_cmp_evt_handler(uint8_t evt_code, struct hci_rd_rem_ver_info_cmp_evt const *p_evt);

// function handlers present in gapc_le_phy_up.c module.
extern void gapc_hci_le_phy_upd_cmp_evt_handler(uint8_t evt_code, struct hci_le_phy_upd_cmp_evt const *p_evt);

// function handlers present in gapc_le_dle.c module.
extern void gapc_hci_le_data_len_chg_evt_handler(uint8_t evt_code, struct hci_le_data_len_chg_evt const *p_evt);

// function handlers present in gapc_le_ping.c module.
extern void gapc_hci_auth_payl_to_exp_evt_handler(uint8_t evt_code, struct hci_auth_payl_to_exp_evt const *p_evt);

#if (BLE_PWR_CTRL)
// function handlers present in gapc_le_pwr_ctrl.c module.
extern void gapc_hci_le_tx_power_rep_evt_handler(uint8_t evt_code, struct hci_le_tx_power_rep_evt const *p_evt);
extern void gapc_hci_le_path_loss_threshold_evt_handler(uint8_t evt_code, struct hci_le_path_loss_threshold_evt const *p_evt);
#endif // (BLE_PWR_CTRL)

// function handlers present in l2cap.c module.
void l2cap_hci_nb_cmp_pkts_evt_handler(uint8_t evt_code, struct hci_nb_cmp_pkts_evt const *p_evt);
void l2cap_hci_acl_data_handler(struct hci_acl_data const *p_evt);
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)

#if (BLE_GAF_PRESENT)
#if (GAF_IAP)
#if (GAF_BROADCAST_SUPP)
// Handlers present in iap_bg.c
#if (GAF_BROADCAST_MST_SUPP)
extern void iap_bg_hci_create_cmp_evt_handler(uint8_t evt_code, struct hci_le_create_big_cmp_evt const* p_evt);
extern void iap_bg_hci_terminate_cmp_evt_handler(uint8_t evt_code, struct hci_le_terminate_big_cmp_evt const* p_evt);
#endif //(GAF_BROADCAST_SLV_SUPP)
#if (GAF_BROADCAST_SLV_SUPP)
extern void iap_bg_hci_sync_lost_evt_handler(uint8_t evt_code, struct hci_le_big_sync_lost_evt const* p_evt);
extern void iap_bg_hci_sync_estab_evt_handler(uint8_t evt_code, struct hci_le_big_sync_est_evt const* p_evt);
#endif //(GAF_BROADCAST_SLV_SUPP)
#endif //(GAF_BROADCAST_SUPP)
// Handlers present in iap_ug.c
#if (GAF_UNICAST_SUPP)
extern void iap_ug_hci_established_evt_handler(uint8_t evt_code, struct hci_le_cis_established_evt const* p_evt);
extern void iap_ug_hci_request_evt_handler(uint8_t evt_code, struct hci_le_cis_request_evt const* p_evt);
extern void iap_ug_hci_disc_cmp_evt_handler(uint8_t evt_code, uint16_t conhdl, struct hci_disc_cmp_evt const* p_evt);
#endif //(GAF_UNICAST_SUPP)
#endif // (GAF_IAP)
#endif // (BLE_GAF_PRESENT)
#endif // (BLE_HOST_PRESENT)

#if (BT_HOST_PRESENT)
extern void gapm_inquiry_hci_inq_cmp_evt(uint8_t opcode, const struct hci_inq_cmp_evt* p_evt);
extern void gapm_inquiry_hci_inq_res_evt(uint8_t opcode, const struct hci_inq_res_evt* p_evt);
extern void gapm_inquiry_hci_inq_res_with_rssi_evt(uint8_t opcode, const struct hci_inq_res_with_rssi_evt* p_evt);
extern void gapm_inquiry_hci_ext_inq_res_evt(uint8_t opcode, const struct hci_ext_inq_res_evt* p_evt);
#if (BLE_L2CAP)
extern bool l2cap_chan_rx_check_if_flow_hijacked(uint8_t conidx, const struct hci_acl_data *p_acl_data);
#endif // (BLE_L2CAP)
#endif // (BT_HOST_PRESENT)

/*
 * HCI HANDLERS DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_HOST_PRESENT)
/// The message handlers for HCI LE events
__STATIC const hl_hci_evt_handler_func_t hl_hci_le_event_handler_tab[] =
{
    #if (HL_LE_OBSERVER)
    /* Scan procedure */
    [HL_HCI_LE_EXT_ADV_REPORT_EVT]                 = (hl_hci_evt_handler_func_t) gapm_hci_le_ext_adv_report_evt_handler,
    [HL_HCI_LE_SCAN_TIMEOUT_EVT]                   = (hl_hci_evt_handler_func_t) gapm_hci_le_scan_timeout_evt_handler,

    /* Periodic synchronization procedure */
    [HL_HCI_LE_PER_ADV_SYNC_EST_EVT]               = (hl_hci_evt_handler_func_t) gapm_hci_le_per_adv_sync_est_evt_handler,
    [HL_HCI_LE_PER_ADV_REPORT_EVT]                 = (hl_hci_evt_handler_func_t) gapm_hci_le_per_adv_report_evt_handler,
    [HL_HCI_LE_PER_ADV_SYNC_LOST_EVT]              = (hl_hci_evt_handler_func_t) gapm_hci_le_per_adv_sync_lost_evt_handler,
    [HL_HCI_LE_BIG_INFO_ADV_REPORT_EVT]            = (hl_hci_evt_handler_func_t) gapm_hci_le_big_info_adv_report_evt_handler,
    #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    [HL_HCI_LE_PER_ADV_SYNC_TRANSF_REC_EVT]        = (hl_hci_evt_handler_func_t) gapm_hci_le_per_adv_sync_transf_rec_evt_handler,
    #else // !(HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    [HL_HCI_LE_PER_ADV_SYNC_TRANSF_REC_EVT]        = (hl_hci_evt_handler_func_t) NULL,
    #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    #if (BLE_AOD | BLE_AOA)
    [HL_HCI_LE_CONLESS_IQ_REPORT_EVT]              = (hl_hci_evt_handler_func_t) gapm_per_sync_hci_le_conless_iq_report_evt_handler,
    #endif // (BLE_AOD | BLE_AOA)
    #else  // !(HL_LE_OBSERVER)
    /* Scan procedure */
    [HL_HCI_LE_EXT_ADV_REPORT_EVT]                 = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_SCAN_TIMEOUT_EVT]                   = (hl_hci_evt_handler_func_t) NULL,

    /* Periodic synchronization procedure */
    [HL_HCI_LE_PER_ADV_SYNC_EST_EVT]               = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_PER_ADV_REPORT_EVT]                 = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_PER_ADV_SYNC_LOST_EVT]              = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_BIG_INFO_ADV_REPORT_EVT]            = (hl_hci_evt_handler_func_t) NULL,
    #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    [HL_HCI_LE_PER_ADV_SYNC_TRANSF_REC_EVT]        = (hl_hci_evt_handler_func_t) NULL,
    #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    #if (BLE_AOD | BLE_AOA)
    [HL_HCI_LE_CONLESS_IQ_REPORT_EVT]              = (hl_hci_evt_handler_func_t) NULL,
    #endif // (BLE_AOD | BLE_AOA)
    #endif //(HL_LE_OBSERVER)

    #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    [HL_HCI_LE_ENH_CON_CMP_EVT]                    = (hl_hci_evt_handler_func_t) hl_hci_le_enh_con_cmp_evt_handler,
    #else // !(HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    [HL_HCI_LE_ENH_CON_CMP_EVT]                    = (hl_hci_evt_handler_func_t) NULL,
    #endif //(HL_LE_CENTRAL || HL_LE_PERIPHERAL)

    [HL_HCI_LE_RD_LOC_P256_PUB_KEY_CMP_EVT]        = (hl_hci_evt_handler_func_t) gapm_hci_le_rd_local_p256_public_key_cmp_evt_handler,
    [HL_HCI_LE_GEN_DHKEY_CMP_EVT]                  = (hl_hci_evt_handler_func_t) gapm_hci_le_gen_dhkey_cmp_evt_handler,

    /* Advertising procedure */
    #if (HL_LE_BROADCASTER)
    [HL_HCI_LE_ADV_SET_TERMINATED_EVT]             = (hl_hci_evt_handler_func_t) gapm_hci_le_adv_set_term_evt_handler,
    [HL_HCI_LE_SCAN_REQ_RCVD_EVT]                  = (hl_hci_evt_handler_func_t) gapm_hci_le_scan_req_rcvd_evt_handler,
    #else // !(HL_LE_BROADCASTER)
    [HL_HCI_LE_ADV_SET_TERMINATED_EVT]             = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_SCAN_REQ_RCVD_EVT]                  = (hl_hci_evt_handler_func_t) NULL,
    #endif //(HL_LE_BROADCASTER)

    #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    [HL_HCI_LE_RD_REM_FEATS_CMP_EVT]               = (hl_hci_evt_handler_func_t) gapc_hci_le_rd_rem_feats_cmp_evt_handler,
    [HL_HCI_LE_CON_UPDATE_CMP_EVT]                 = (hl_hci_evt_handler_func_t) gapc_hci_le_con_update_cmp_evt_handler,
    [HL_HCI_LE_REM_CON_PARAM_REQ_EVT]              = (hl_hci_evt_handler_func_t) gapc_hci_le_rem_con_param_req_evt_handler,
    [HL_HCI_LE_DATA_LEN_CHG_EVT]                   = (hl_hci_evt_handler_func_t) gapc_hci_le_data_len_chg_evt_handler,
    [HL_HCI_LE_PHY_UPD_CMP_EVT]                    = (hl_hci_evt_handler_func_t) gapc_hci_le_phy_upd_cmp_evt_handler,
    [HL_HCI_LE_CH_SEL_ALGO_EVT]                    = (hl_hci_evt_handler_func_t) gapc_hci_le_ch_sel_algo_evt_handler,
    #if(BLE_AOA | BLE_AOD)
    [HL_HCI_LE_CON_IQ_REPORT_EVT]                  = (hl_hci_evt_handler_func_t) gapc_hci_le_con_iq_report_evt_handler,
    [HL_HCI_LE_CTE_REQ_FAILED_EVT]                 = (hl_hci_evt_handler_func_t) gapc_hci_le_cte_req_failed_evt_handler,
    #endif // (BLE_AOA | BLE_AOD)
    #if (BLE_PWR_CTRL)
    [HL_HCI_LE_TX_POWER_REPORTING_EVT]             = (hl_hci_evt_handler_func_t) gapc_hci_le_tx_power_rep_evt_handler,
    [HL_HCI_LE_PATH_LOSS_THRESHOLD_EVT]            = (hl_hci_evt_handler_func_t) gapc_hci_le_path_loss_threshold_evt_handler,
    #endif // (BLE_PWR_CTRL)
    #else // !(HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    [HL_HCI_LE_RD_REM_FEATS_CMP_EVT]               = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_CON_UPDATE_CMP_EVT]                 = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_REM_CON_PARAM_REQ_EVT]              = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_DATA_LEN_CHG_EVT]                   = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_PHY_UPD_CMP_EVT]                    = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_CH_SEL_ALGO_EVT]                    = (hl_hci_evt_handler_func_t) NULL,
    #if(BLE_AOA | BLE_AOD)
    [HL_HCI_LE_CON_IQ_REPORT_EVT]                  = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_CTE_REQ_FAILED_EVT]                 = (hl_hci_evt_handler_func_t) NULL,
    #endif // (BLE_AOA | BLE_AOD)
    #if (BLE_PWR_CTRL)
    [HL_HCI_LE_TX_POWER_REPORTING_EVT]             = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_PATH_LOSS_THRESHOLD_EVT]            = (hl_hci_evt_handler_func_t) NULL,
    #endif // (BLE_PWR_CTRL)
    #endif //  (HL_LE_CENTRAL || HL_LE_PERIPHERAL)


    #if (HL_LE_PERIPHERAL)
    [HL_HCI_LE_LTK_REQUEST_EVT]                    = (hl_hci_evt_handler_func_t) gapc_hci_le_ltk_request_evt_handler,
    #else // !(HL_LE_PERIPHERAL)
    [HL_HCI_LE_LTK_REQUEST_EVT]                    = (hl_hci_evt_handler_func_t) NULL,
    #endif //(HL_LE_PERIPHERAL)

    #if(BLE_CIS || BLE_BIS)
    #if (!BLE_GAF_PRESENT)
    [HL_HCI_LE_CIS_ESTABLISHED_EVT]                = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_CIS_REQUEST_EVT]                    = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_CREATE_BIG_CMP_EVT]                 = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_TERMINATE_BIG_CMP_EVT]              = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_BIG_SYNC_ESTABLISHED_EVT]           = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_BIG_SYNC_LOST_EVT]                  = (hl_hci_evt_handler_func_t) NULL,
    #else  // (BLE_GAF_PRESENT)
    #if (GAF_IAP && GAF_UNICAST_SUPP)
    [HL_HCI_LE_CIS_ESTABLISHED_EVT]                = (hl_hci_evt_handler_func_t) iap_ug_hci_established_evt_handler,
    [HL_HCI_LE_CIS_REQUEST_EVT]                    = (hl_hci_evt_handler_func_t) iap_ug_hci_request_evt_handler,
    #else  // !(GAF_IAP && GAF_UNICAST_SUPP)
    [HL_HCI_LE_CIS_ESTABLISHED_EVT]                = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_CIS_REQUEST_EVT]                    = (hl_hci_evt_handler_func_t) NULL,
    #endif // (GAF_IAP && GAF_UNICAST_SUPP)
    #if (GAF_IAP && GAF_BROADCAST_SUPP && GAF_BROADCAST_MST_SUPP)
    [HL_HCI_LE_CREATE_BIG_CMP_EVT]                 = (hl_hci_evt_handler_func_t) iap_bg_hci_create_cmp_evt_handler,
    [HL_HCI_LE_TERMINATE_BIG_CMP_EVT]              = (hl_hci_evt_handler_func_t) iap_bg_hci_terminate_cmp_evt_handler,
    #else // !(GAF_IAP && GAF_BROADCAST_SUPP && GAF_BROADCAST_MST_SUPP)
    [HL_HCI_LE_CREATE_BIG_CMP_EVT]                 = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_TERMINATE_BIG_CMP_EVT]              = (hl_hci_evt_handler_func_t) NULL,
    #endif // (GAF_IAP && GAF_BROADCAST_SUPP && GAF_BROADCAST_MST_SUPP)
    #if (GAF_IAP && GAF_BROADCAST_SUPP && GAF_BROADCAST_SLV_SUPP)
    [HL_HCI_LE_BIG_SYNC_ESTABLISHED_EVT]           = (hl_hci_evt_handler_func_t) iap_bg_hci_sync_estab_evt_handler,
    [HL_HCI_LE_BIG_SYNC_LOST_EVT]                  = (hl_hci_evt_handler_func_t) iap_bg_hci_sync_lost_evt_handler,
    #else // !(GAF_IAP && GAF_BROADCAST_SUPP && GAF_BROADCAST_SLV_SUPP)
    [HL_HCI_LE_BIG_SYNC_ESTABLISHED_EVT]           = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_LE_BIG_SYNC_LOST_EVT]                  = (hl_hci_evt_handler_func_t) NULL,
    #endif // (GAF_IAP && GAF_BROADCAST_SUPP && GAF_BROADCAST_SLV_SUPP)
    #endif // (!BLE_GAF_PRESENT)
    #endif // (BLE_CIS || BLE_BIS)
};
#endif // (BLE_HOST_PRESENT)


/// The message handlers for HCI events
__STATIC const hl_hci_evt_handler_func_t hl_hci_event_handler_tab[] =
{
    #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    [HL_HCI_DISC_CMP_EVT]                          = (hl_hci_evt_handler_func_t) hl_hci_disc_cmp_evt_handler,
    [HL_HCI_RD_REM_VER_INFO_CMP_EVT]               = (hl_hci_evt_handler_func_t) gapc_hci_rd_rem_ver_info_cmp_evt_handler,
    [HL_HCI_AUTH_PAYL_TO_EXP_EVT]                  = (hl_hci_evt_handler_func_t) gapc_hci_auth_payl_to_exp_evt_handler,
    [HL_HCI_ENC_CHG_EVT]                           = (hl_hci_evt_handler_func_t) gapc_hci_enc_chg_evt_handler,
    [HL_HCI_ENC_KEY_REFRESH_CMP_EVT]               = (hl_hci_evt_handler_func_t) gapc_hci_enc_key_refr_evt_handler,
    [HL_HCI_NB_CMP_PKTS_EVT]                       = (hl_hci_evt_handler_func_t) l2cap_hci_nb_cmp_pkts_evt_handler,
    #else // !(HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    [HL_HCI_DISC_CMP_EVT]                          = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_RD_REM_VER_INFO_CMP_EVT]               = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_AUTH_PAYL_TO_EXP_EVT]                  = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_ENC_CHG_EVT]                           = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_ENC_KEY_REFRESH_CMP_EVT]               = (hl_hci_evt_handler_func_t) NULL,
    [HL_HCI_NB_CMP_PKTS_EVT]                       = (hl_hci_evt_handler_func_t) NULL,
    #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    #if (BT_HOST_PRESENT)
    [HL_HCI_INQ_CMP_EVT]                           = (hl_hci_evt_handler_func_t) gapm_inquiry_hci_inq_cmp_evt,
    [HL_HCI_INQ_RES_EVT]                           = (hl_hci_evt_handler_func_t) gapm_inquiry_hci_inq_res_evt,
    [HL_HCI_INQ_RES_WITH_RSSI_EVT]                 = (hl_hci_evt_handler_func_t) gapm_inquiry_hci_inq_res_with_rssi_evt,
    [HL_HCI_EXT_INQ_RES_EVT]                       = (hl_hci_evt_handler_func_t) gapm_inquiry_hci_ext_inq_res_evt,
    #endif // (BT_HOST_PRESENT)
};


/// The message handlers for HCI VS events
__STATIC const hl_hci_evt_handler_func_t hl_hci_vs_event_handler_tab[] =
{
#ifdef WIN32
	0
#endif
};


/// Information about handle tables
__STATIC const hl_hci_evt_handler_info_t hl_hci_handler_info_tab[] =
{
    [HL_HCI_EVT]          = { HL_HCI_EVT_HANDLER_NB,          hl_hci_event_handler_tab          },
    #if (BLE_HOST_PRESENT)
    [HL_HCI_LE_EVT]       = { HL_HCI_LE_EVT_HANDLER_NB,       hl_hci_le_event_handler_tab       },
    #endif // (BLE_HOST_PRESENT)
    [HL_HCI_DBG_EVT]      = { HL_HCI_VS_EVT_HANDLER_NB,       hl_hci_vs_event_handler_tab       },
};

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Handles Disconnect command completed event
 *
 * @param[in] evt_code  HCI event code or subcode
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void hl_hci_disc_cmp_evt_handler(uint8_t evt_code, struct hci_disc_cmp_evt const *p_evt)
{
    uint8_t conidx = gapc_get_conidx(p_evt->conhdl);

    if(conidx != GAP_INVALID_CONIDX)
    {
        gapc_le_con_hci_con_disc_cmp_evt_handler(evt_code, conidx, p_evt);
    }
    #if (BLE_GAF_PRESENT)
    #if (GAF_IAP && GAF_UNICAST_SUPP)
    // message handled by IAP
    else
    {
        iap_ug_hci_disc_cmp_evt_handler(evt_code, p_evt->conhdl, p_evt);
    }
    #endif // (GAF_IAP && GAF_UNICAST_SUPP)
    #endif // (BLE_GAF_PRESENT)
}

__STATIC void hl_hci_le_enh_con_cmp_evt_handler(uint8_t evt_code, struct hci_le_enh_con_cmp_evt const *p_evt)
{
    #if(HL_LE_CENTRAL)
    if(p_evt->role == ROLE_MASTER)
    {
        gapm_init_hci_le_enh_con_cmp_evt_handler(evt_code, p_evt);
    }
    else
    #endif // (HL_LE_CENTRAL)
    {
        #if (HL_LE_PERIPHERAL)
        gapm_adv_hci_le_enh_con_cmp_evt_handler(evt_code, p_evt);
        #endif // (HL_LE_PERIPHERAL)
    }

}
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)


/**
 ****************************************************************************************
 * @brief Handles defer of HCI command transmission and command complete or status event handler execution
 *
 * @param[in] p_djob    Pointer to the defer job structure
 *
 ****************************************************************************************
 */
__STATIC void hl_hci_cmd_defer(co_djob_t* p_djob)
{
    hl_hci_cmd_hdr_t* p_cmd_hdr = (hl_hci_cmd_hdr_t*) co_list_pop_front(&(hl_hci_env.cmd_queue));

    #if (EMB_PRESENT)
    // if a command complete or status event has been received
    if(hl_hci_env.handle_cmd_evt)
    {
        const void* p_cmd_evt = ke_msg2param((ke_msg_t*) p_cmd_hdr);
        // execute command status or command complete event handler
        hl_hci_env.cmd_evt_cb(hl_hci_env.cmd_opcode, hl_hci_env.cmd_handle, p_cmd_evt);
        hl_hci_env.wait_cmd_evt = false;
        hl_hci_env.handle_cmd_evt = false;
        ke_free(p_cmd_hdr);

        // check if there is a command to transmit
        p_cmd_hdr = (hl_hci_cmd_hdr_t*) co_list_pop_front(&(hl_hci_env.cmd_queue));
    }
    #endif // (EMB_PRESENT)

    #if(BT_HOST_PRESENT)
    if((hl_hci_bt_cmd_t*) p_cmd_hdr == &(hl_hci_env.bt_cmd))
    {
        // Inform that command transmission is done
        hl_hci_env.bt_cmd.hdr.next = NULL;
        hl_hci_env.cmd_tl_dest     = HOST_TL_PK;
        hl_hci_env.wait_cmd_evt    = true;
        hl_hci_env.cmd_opcode      = hl_hci_env.bt_cmd.opcode;

        hci_send_pk_2_controller(HCI_CMD_MSG_TYPE, hl_hci_env.bt_cmd.length, hl_hci_env.bt_cmd.p_buf,
                                 (hci_done_cb) hl_hci_env.bt_cmd.cb_done);
    }
    else
    #endif // (BT_HOST_PRESENT)
    if (p_cmd_hdr != NULL)
    {
        ke_msg_t* p_hci_msg = (ke_msg_t*) p_cmd_hdr;
        ASSERT_ERR(sizeof(hl_hci_cmd_hdr_t) <= sizeof(ke_msg_t));

        hl_hci_env.cmd_tl_dest  = HOST_TL_UPK;
        hl_hci_env.wait_cmd_evt = true;
        hl_hci_env.cmd_handle   = p_cmd_hdr->handle;
        hl_hci_env.cmd_opcode   = p_cmd_hdr->opcode;
        hl_hci_env.cmd_evt_cb   = p_cmd_hdr->cmd_evt_cb;

        p_hci_msg->id        = HCI_COMMAND;
        p_hci_msg->src_id    = hl_hci_env.cmd_opcode;
        p_hci_msg->dest_id   = 0;      // unused
        p_hci_msg->param_len = 0xFFFF; // force to maximum length - not supposed to be used

        hci_send_2_controller(ke_msg2param(p_hci_msg));
    }
}

#if (EMB_PRESENT)
/**
 ****************************************************************************************
 * @brief Handles defer of HCI event to send to host
 *
 * @param[in] p_djob  Pointer to the defer job structure
 *
 ****************************************************************************************
 */
__STATIC void hl_hci_evt_send_to_host_defer(co_djob_t* p_djob)
{
    struct ke_msg* p_msg = (struct ke_msg*) co_list_pop_front(&(hl_hci_env.evt_queue));

    if (p_msg != NULL)
    {
        hl_hci_evt_msg_t* p_evt_info = ( hl_hci_evt_msg_t*) p_msg;

        p_evt_info->cb_handler(p_evt_info->evt_code, ke_msg2param(p_msg));
        ke_msg_free(p_msg);
    }

    // check if another event can be send
    if(!co_list_is_empty(&(hl_hci_env.evt_queue)))
    {
        co_djob_reg(CO_DJOB_HIGH, &(hl_hci_env.evt_djob));
    }
}
#endif // (EMB_PRESENT)

/**
 ****************************************************************************************
 * @brief Retrieve command header from command parameters
 *
 * @param[in] p_cmd Pointer to HCI command parameters
 *
 * @return HCI Command header (metadata)
 ****************************************************************************************
 */
hl_hci_cmd_hdr_t* hl_hci_cmd2hdr(void* p_cmd)
{
    return (hl_hci_cmd_hdr_t*) (((uint8_t*)p_cmd) - sizeof (hl_hci_cmd_hdr_t));
}



/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint8_t hl_hci_get_host_tl_dest_from_conhdl(uint16_t conhdl)
{
    uint8_t host_tl_dest;

    #if(!EMB_PRESENT)
    // during reset procedure ignore unexpected event received
    if(gapm_is_doing_reset())
    {
        host_tl_dest = HOST_NONE;
    }
    else
    #endif // (!EMB_PRESENT)
    #if (BT_HOST_PRESENT)
    if((conhdl & BT_ACL_CONHDL_BIT) != 0) // Much more faster than  (gapc_is_bt_connection(gapc_get_conidx(conhdl)))
    {
        host_tl_dest = HOST_TL_PK;
    }
    else
    #endif // (BT_HOST_PRESENT)
    {
        host_tl_dest = HOST_TL_UPK;
    }

    return (host_tl_dest);
}

uint8_t hl_hci_get_acl_data_host_tl_dest(const struct hci_acl_data *p_acl_data)
{
    uint16_t conhdl = GETF(p_acl_data->conhdl_pb_bc_flag, HCI_ACL_HDR_HDL);
    uint8_t host_tl_dest = hl_hci_get_host_tl_dest_from_conhdl(conhdl);
    #if (BT_HOST_PRESENT & BLE_L2CAP)
    if(   (host_tl_dest == HOST_TL_PK) // BR/EDR connection
       // Check if L2cap data can be Hijacked
       && l2cap_chan_rx_check_if_flow_hijacked(gapc_get_conidx(conhdl), p_acl_data))
    {
        host_tl_dest = HOST_TL_UPK; // route data to unpacked transport layer (RW Host)
    }
    #endif // (BT_HOST_PRESENT & BLE_L2CAP)

    return (host_tl_dest);
}

uint8_t hl_hci_cmd_evt_get_host_tl_dest(uint16_t opcode)
{
    uint8_t tl_dest = hl_hci_env.cmd_tl_dest;

    #if(EMB_PRESENT)
    ASSERT_ERR(hl_hci_env.cmd_opcode == opcode);
    #else
    // during reset procedure ignore unexpected event received
    if((gapm_is_doing_reset()) && (opcode != HCI_RESET_CMD_OPCODE))
    {
        tl_dest = HOST_NONE;
    }
    else
    #endif // (!EMB_PRESENT)
    #if (BT_HOST_PRESENT)
    // for BT Host, next command can be sent once event destination is computed
    if(hl_hci_env.cmd_tl_dest == HOST_TL_PK)
    {
        ASSERT_INFO(hl_hci_env.wait_cmd_evt, hl_hci_env.cmd_opcode, opcode);
        hl_hci_env.wait_cmd_evt = false;
        // check if another command can be send
        if(!co_list_is_empty(&(hl_hci_env.cmd_queue)))
        {
            co_djob_reg(CO_DJOB_HIGH, &(hl_hci_env.cmd_djob));
        }
    }
    else
    #endif // (BT_HOST_PRESENT)
    {
        ASSERT_INFO(hl_hci_env.wait_cmd_evt, hl_hci_env.cmd_opcode, opcode);
        // nothing to do
    }

    return (tl_dest);
}


void hl_hci_send_evt_to_host(uint8_t evt_type, uint8_t evt_code, uint8_t host_lid, const void* p_evt)
{
    hl_hci_evt_handler_func_t evt_handler = NULL;

    #if (EMB_PRESENT && RW_DEBUG)
    DUMP_UPK_HCI(evt_type, 1, evt_code, p_evt, ke_param2msg(p_evt)->param_len);
    #endif // (EMB_PRESENT && RW_DEBUG)

    #if(!EMB_PRESENT)
    // during reset procedure ignore unexpected event received
    if(gapm_is_doing_reset())
    {
        // Ignore
    }
    else
    #endif // (!EMB_PRESENT)
    // retrieve function handler
    if((evt_type >= HL_HCI_EVT) && (evt_type <= HL_HCI_DBG_EVT) && (host_lid != HL_HCI_INVALID_LID))
    {
        const hl_hci_evt_handler_info_t* p_info = &(hl_hci_handler_info_tab[evt_type]);

        if(host_lid < p_info->nb_handlers)
        {
            evt_handler = p_info->p_handlers[host_lid];
        }
    }

    ASSERT_WARN(evt_handler != NULL, evt_code, (host_lid << 8) | evt_type);

    // Ask for execution of event handler
    if(evt_handler != NULL)
    {
        #if (EMB_PRESENT)
        hl_hci_evt_msg_t* p_evt_info = ( hl_hci_evt_msg_t*) ke_param2msg(p_evt);
        p_evt_info->cb_handler = evt_handler;
        p_evt_info->evt_code   = evt_code;

        co_list_push_back(&(hl_hci_env.evt_queue), &(p_evt_info->hdr));
        co_djob_reg(CO_DJOB_HIGH, &(hl_hci_env.evt_djob));
        #else // ! (EMB_PRESENT)
        evt_handler(evt_code, p_evt);
        #endif // (EMB_PRESENT)
    }
    #if (EMB_PRESENT)
    else
    {
        ke_msg_free(ke_param2msg(p_evt));
    }
    #endif // (EMB_PRESENT)
}

void hl_hci_send_cmd_evt_to_host(uint8_t evt_type, uint16_t opcode, const void* p_evt)
{
    #if (EMB_PRESENT && RW_DEBUG)
    DUMP_UPK_HCI(evt_type, 1, opcode, p_evt, ke_param2msg(p_evt)->param_len);
    #endif // (EMB_PRESENT && RW_DEBUG)

    // Command complete or status received
    ASSERT_ERR(hl_hci_env.cmd_opcode == opcode);
    ASSERT_ERR(hl_hci_env.wait_cmd_evt);

    #if (EMB_PRESENT)
    ASSERT_ERR(!hl_hci_env.handle_cmd_evt);
    hl_hci_env.handle_cmd_evt = true;
    co_list_push_front(&(hl_hci_env.cmd_queue), &(ke_param2msg(p_evt)->hdr));

    // Handle command complete or status event in defer function execution and check if another command can be sent
    co_djob_reg(CO_DJOB_HIGH, &(hl_hci_env.cmd_djob));
    #else // !(EMB_PRESENT)
    // Immediately execute command complete or status event handler
    hl_hci_env.wait_cmd_evt = false;
    // check if another command can be send
    if(!co_list_is_empty(&(hl_hci_env.cmd_queue)))
    {
        co_djob_reg(CO_DJOB_HIGH, &(hl_hci_env.cmd_djob));
    }
    hl_hci_env.cmd_evt_cb(opcode, hl_hci_env.cmd_handle, p_evt);
    #endif // (EMB_PRESENT)
}

void hl_hci_send_data_to_host(uint8_t evt_type, const void* p_evt)
{
    #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    const struct hci_acl_data *p_data_rx =  (const struct hci_acl_data*) p_evt;

    #if (EMB_PRESENT && RW_DEBUG)
    {
        uint8_t hdr[HCI_ACL_HDR_LEN];
        co_write16p(&(hdr[0]), p_data_rx->conhdl_pb_bc_flag);
        co_write16p(&(hdr[2]), p_data_rx->length);

        DUMP_HCI_2(HCI_ACL_MSG_TYPE, 1, hdr, HCI_ACL_HDR_LEN, p_data_rx->buf_ptr + EM_BASE_ADDR, p_data_rx->length);
    }
    #endif // (EMB_PRESENT && RW_DEBUG)

    l2cap_hci_acl_data_handler(p_data_rx);
    #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)

    #if(EMB_PRESENT)
    ke_msg_free(ke_param2msg(p_evt));
    #endif // (EMB_PRESENT)
}

void hl_hci_send_cmd_to_ctrl(void* p_cmd, uint16_t handle, hl_hci_cmd_evt_func_t cmd_evt_cb)
{
    #if (EMB_PRESENT && RW_DEBUG)
    DUMP_UPK_HCI(HL_HCI_CMD, 0, ke_param2msg(p_cmd)->src_id, p_cmd, 255); // length parameter ignored
    #endif // (EMB_PRESENT && RW_DEBUG)

    hl_hci_cmd_hdr_t* p_cmd_hdr = hl_hci_cmd2hdr(p_cmd);
    p_cmd_hdr->handle = handle; // store handle
    p_cmd_hdr->cmd_evt_cb = cmd_evt_cb; // function to call once command complete or status is received
    co_list_push_back(&(hl_hci_env.cmd_queue), &(p_cmd_hdr->hdr));

    if(!hl_hci_env.wait_cmd_evt)
    {
        co_djob_reg(CO_DJOB_HIGH, &(hl_hci_env.cmd_djob));
    }
}

void hl_hci_send_data_to_ctrl(struct hci_acl_data* p_data)
{
    #if (EMB_PRESENT && RW_DEBUG)
    {
        uint8_t hdr[HCI_ACL_HDR_LEN];
        co_write16p(&(hdr[0]), p_data->conhdl_pb_bc_flag);
        co_write16p(&(hdr[2]), p_data->length);

        DUMP_HCI_2(HCI_ACL_MSG_TYPE, 0, hdr, HCI_ACL_HDR_LEN, p_data->buf_ptr + EM_BASE_ADDR, p_data->length);
    }
    #endif // (EMB_PRESENT && RW_DEBUG)

    hci_send_2_controller(p_data);
}

uint16_t hl_hci_basic_cmd_send_to_ctrl(uint16_t opcode, uint16_t handle, hl_hci_cmd_evt_func_t cmp_evt_cb)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    uint8_t *p_no_param = hl_hci_cmd_alloc(opcode, 0);
    if(p_no_param != NULL)
    {
        status = GAP_ERR_NO_ERROR;
        hl_hci_send_cmd_to_ctrl(p_no_param, handle, cmp_evt_cb);
    }
    return (status);
}


uint16_t hl_hci_basic_cmd_send_with_conhdl_to_ctrl(uint16_t opcode, uint16_t conhdl, uint16_t handle,
                                                   hl_hci_cmd_evt_func_t cmp_evt_cb)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    struct hci_basic_conhdl_cmd* p_cmd = HL_HCI_CMD_ALLOC(opcode, hci_basic_conhdl_cmd);

    if(p_cmd != NULL)
    {
        status = GAP_ERR_NO_ERROR;
        p_cmd->conhdl = conhdl;
        hl_hci_send_cmd_to_ctrl(p_cmd, handle, cmp_evt_cb);
    }
    return (status);
}

#if (BT_HOST_PRESENT)
void hl_hci_pk_send_to_host(uint8_t type, uint8_t *p_buf, uint16_t len, hl_hci_done_cb cb_done)
{
    #if(EMB_PRESENT)
    DUMP_HCI(type, 1, p_buf, len);
    #endif // (EMB_PRESENT)

    #if(!EMB_PRESENT)
    // during reset procedure ignore unexpected event received
    if(gapm_is_doing_reset())
    {
        cb_done();
    }
    else
    #endif // (!EMB_PRESENT)
    {
        // Handle HCI Packet receptions
        bk_al_core_hci_msg_publish(type, p_buf, len, (bk_al_hci_done_cb)cb_done);
    }
}

void hl_hci_pk_send_to_ctrl(uint8_t type, uint8_t *p_buf, uint16_t len, hl_hci_done_cb cb_done)
{
    #if(EMB_PRESENT)
    DUMP_HCI(type, 0, p_buf, len);
    #endif // (EMB_PRESENT)
    // send packet
    switch(type)
    {
        case HCI_CMD_MSG_TYPE :
        {
            uint16_t opcode = co_read16p(&p_buf[0]);
            // Only one command expected from BT Host
            ASSERT_ERR(hl_hci_env.bt_cmd.hdr.next == NULL);

            // If host is in configuring state
            if(!gapm_is_configured())
            {
                // Some HCI message are handled by RW host, so it shall not be sent to controller by BT Host
                if(   (opcode == HCI_RESET_CMD_OPCODE) || (opcode == HCI_SET_EVT_MASK_CMD_OPCODE)
                   || (opcode == HCI_WR_LOCAL_NAME_CMD_OPCODE) || (opcode == HCI_WR_EXT_INQ_RSP_CMD_OPCODE)
                   || (opcode == HCI_WR_INQ_MODE_CMD_OPCODE) || (opcode == HCI_WR_SCAN_EN_CMD_OPCODE))
                {
                    // reuse command info buffer
                    uint8_t* p_evt = (uint8_t*) &(hl_hci_env.bt_cmd.p_buf);
                    // pack event code
                    *p_evt++ = HCI_CMD_CMP_EVT_CODE;
                    // pack event parameter length
                    *p_evt++ = HCI_CCEVT_HDR_PARLEN + 1;
                    // pack the number of h2c packets
                    *p_evt++ = HCI_NB_CMD_PKTS;
                    // pack opcode
                    co_write16p(p_evt, opcode);
                    p_evt+=2;
                    // pack status
                    *p_evt++ = CO_ERROR_NO_ERROR;


                    // provide tx done callback to rxdone so that when event is processed, it releases command message
                    hl_hci_pk_send_to_host(HCI_EVT_MSG_TYPE, (uint8_t*) &(hl_hci_env.bt_cmd.p_buf),
                                           HCI_EVT_HDR_LEN + HCI_CCEVT_HDR_PARLEN + 1, cb_done);

                    break;
                }
            }

            hl_hci_env.bt_cmd.length  = len;
            hl_hci_env.bt_cmd.p_buf   = p_buf;
            hl_hci_env.bt_cmd.opcode  = opcode;
            hl_hci_env.bt_cmd.cb_done = cb_done;

            co_list_push_back(&(hl_hci_env.cmd_queue), &(hl_hci_env.bt_cmd.hdr));

            if(!hl_hci_env.wait_cmd_evt)
            {
                co_djob_reg(CO_DJOB_HIGH, &(hl_hci_env.cmd_djob));
            }
        } break;

        case HCI_ACL_MSG_TYPE:
        #if (VOICE_OVER_HCI)
        case HCI_SYNC_MSG_TYPE:
        #endif // (VOICE_OVER_HCI)
        {
            // Immediately send packet to controller
            hci_send_pk_2_controller(type, len, p_buf, (hci_done_cb) cb_done);
        } break;

        // ignore the packet
        default : { cb_done(); }  break;
    }
}
#endif // (BT_HOST_PRESENT)

/**
 ****************************************************************************************
 * @brief Initialize Generic Access Profile Manager Module.
 *
 * @param[in] init_type  Type of initialization (see enum #rwip_init_type)
 ****************************************************************************************
 */
void hl_hci_initialize(uint8_t init_type)
{
    // boot configuration
    switch (init_type)
    {
        case RWIP_INIT:
        {
            co_list_init(&(hl_hci_env.cmd_queue));
            co_djob_init(&(hl_hci_env.cmd_djob), hl_hci_cmd_defer);

            #if (EMB_PRESENT)
            co_list_init(&(hl_hci_env.evt_queue));
            co_djob_init(&(hl_hci_env.evt_djob), hl_hci_evt_send_to_host_defer);
            #endif  // (EMB_PRESENT)

            #if (BT_HOST_PRESENT)
            hl_hci_env.bt_cmd.hdr.next = NULL;
            #endif // (BT_HOST_PRESENT)
        }
        break;

        case RWIP_RST:
        case RWIP_1ST_RST:
        {
            while(!co_list_is_empty(&(hl_hci_env.cmd_queue)))
            {
                struct ke_msg* p_msg = (struct ke_msg*) co_list_pop_front(&(hl_hci_env.cmd_queue));
                #if (BT_HOST_PRESENT)
                if((hl_hci_bt_cmd_t*) p_msg == &(hl_hci_env.bt_cmd))
                {
                    hl_hci_env.bt_cmd.hdr.next = NULL;
                }
                else
                #endif // (BT_HOST_PRESENT)
                {
                    ke_msg_free(p_msg);
                }
            }
            #if (EMB_PRESENT)
            while(!co_list_is_empty(&(hl_hci_env.evt_queue)))
            {
                struct ke_msg* p_msg = (struct ke_msg*) co_list_pop_front(&(hl_hci_env.cmd_queue));
                ke_msg_free(p_msg);
            }

            hl_hci_env.handle_cmd_evt = false;
            #endif // (EMB_PRESENT)

            hl_hci_env.wait_cmd_evt = false;
        } break;
        default: { /* Do nothing */ } break;
    }
}

uint8_t* hl_hci_cmd_alloc(uint16_t opcode, uint16_t size)
{
    hl_hci_cmd_hdr_t* p_cmd_hdr = (hl_hci_cmd_hdr_t*) ke_malloc_user(size + sizeof(hl_hci_cmd_hdr_t), KE_MEM_KE_MSG);
    uint8_t* p_cmd = NULL;
    if(p_cmd_hdr != NULL)
    {
        p_cmd_hdr->opcode = opcode;
        p_cmd = (uint8_t*) (p_cmd_hdr + 1);
    }

    return p_cmd;
}

struct hci_acl_data* hl_hci_data_alloc()
{
    struct hci_acl_data* p_data = KE_MSG_ALLOC(HCI_ACL_DATA, 0, 0, hci_acl_data);
    return p_data;
}

/// @} HOST
