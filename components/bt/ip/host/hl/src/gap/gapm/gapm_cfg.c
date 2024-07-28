/**
 ****************************************************************************************
 *
 * @file gapm_cfg.c
 *
 * @brief Generic Access Profile Manager - Device configuration
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
#include "rwip_config.h" // IP Configuration
#include "gapm_int.h"    // GAP Manager Internals
#if (BLE_HOST_PRESENT)
#include "gapm_le.h"      // GAP Manager LE API
#include "gapm_le_test.h" // GAP Manager LE test API
#endif //(BLE_HOST_PRESENT)
#if (BT_HOST_PRESENT)
#include "bk_al.h"       // BT Host initialization
#include "gapm_bt.h"     // GAP Manager BT API
#endif // (BT_HOST_PRESENT)
#include "hci.h"         // HCI interface definition
#include "co_bt.h"       // Bluetooth defines
#include "hl_hci.h"      // Host HCI interface
#include "../../inc/l2cap_hl_api.h" // L2CAP internal API
#include "../gap_int.h"  // GAP internals
#include "co_math.h"     // For parameter range check
#include "co_utils.h"    // Default BT address
#include "rwip.h"        // To reset the device
#include "gapc_sec.h"    // GAPC Security API
#if(BLE_GAPC)
#include "gapc_le.h"     // GAPC LE API
#endif // (BLE_GAPC)


#if (BT_HOST_PRESENT)
#include "bk_al.h"       // BT Host initialization
#endif // (BT_HOST_PRESENT)

#include <string.h>      // for memcopy

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * DEFINES
 ****************************************************************************************
 */


#if (!BT_HOST_PRESENT)
#define GAP_EVT_MASK                {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9F, 0x00, 0x20}
#else
#define GAP_EVT_MASK                {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F}
#endif // (!BT_HOST_PRESENT)

/// Low energy mask
#define GAP_LE_EVT_MASK             {0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00}


/// state machine of reset operation
enum gapm_cfg_proc_event
{
    GAPM_CFG_INIT = HL_PROC_EVENT_FIRST,
    #if (!EMB_PRESENT)
    /// HCI_RESET_CMD command execution completed
    GAPM_CFG_RESET_HCI,
    /// HCI_SET_EVT_MASK_CMD command execution completed
    GAPM_CFG_RESET_SET_EVT_MASK,
    #if (BLE_HOST_PRESENT)
    /// HCI_LE_SET_EVT_MASK_CMD command execution completed
    GAPM_CFG_RESET_LE_SET_EVT_MASK,
    #endif // (BLE_HOST_PRESENT)
    /// HCI_RD_BD_ADDR_CMD command execution completed
    GAPM_CFG_RESET_RD_BD_ADDR,
    #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    /// HCI_LE_RD_BUF_SIZE_CMD command execution completed
    GAPM_CFG_RESET_LE_RD_BUF_SIZE,
    /// HCI_RD_BUF_SIZE_CMD command execution completed
    GAPM_CFG_RESET_RD_BUF_SIZE,
    #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    #if (HL_LE_BROADCASTER)
    /// HCI_RD_MAX_ADV_DATA_LEN command execution completed
    GAPM_CFG_RESET_RD_MAX_ADV_DATA_LEN,
    /// HCI_RD_NB_ADV_SETS command execution completed
    GAPM_CFG_RESET_RD_NB_ADV_SETS,
    #endif //(HL_LE_BROADCASTER)
    #endif // (EMB_PRESENT)

    #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    /// HCI_LE_WR_SUGGTED_DFT_DATA_LEN_CMD command execution completed
    GAPM_CFG_SETUP_WR_LE_DFT_DATA_LEN_CMD,
    #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    #if (!BLE_EMB_PRESENT && BLE_ISO_PRESENT)
    /// HCI_LE_SET_HOST_FEATURE_CMD command execution completed
    GAPM_CFG_SETUP_LE_SET_HOST_FEATURE_CMD,
    #endif // (!BLE_EMB_PRESENT && BLE_ISO_PRESENT)

    #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    /// HCI_LE_SET_DFT_PHY_CMD command execution completed
    GAPM_CFG_SETUP_SET_LE_DFT_PHY_CMD,
    #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    #if (BLE_HOST_PRESENT)
    /// HCI_LE_WR_RF_PATH_COMP_CMD command execution completed
    GAPM_CFG_SETUP_WR_RF_PATH_COMP_CMD,
    /// HCI_LE_SET_RSLV_PRIV_ADDR_TO_CMD command execution completed
    GAPM_CFG_SETUP_SET_RENEW_TO,
    /// HCI_LE_SET_ADDR_RESOL_EN_CMD command execution completed
    GAPM_CFG_SETUP_EN_CTRL_PRIV,
    #endif // (BLE_HOST_PRESENT)
    #if (BT_HOST_PRESENT)
    /// Initialize of the BT host completed properly
    GAPM_CFG_SETUP_BT_HOST,
    #endif // (BT_HOST_PRESENT)
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure of device setup procedure
typedef struct gapm_cfg_proc_setup
{
    /// Procedure inheritance
    gapm_proc_set_cfg_t hdr;
    /// True if procedure requires reset to be performed
    bool                do_reset;
    /// True if complete configuration must be performed
    bool                do_configure;
    /// Device configuration
    gapm_config_t       cfg;
} gapm_cfg_proc_setup_t;

#if (HL_LE_CENTRAL || HL_LE_BROADCASTER)
/// Structure of Set channel map procedure
typedef struct gapm_cfg_proc_set_chmap
{
    /// Procedure inheritance
    gapm_proc_send_hci_cmd_t hdr;
    /// Channel map
    le_chnl_map_t            chmap;
} gapm_cfg_proc_set_chmap_t;
#endif // (HL_LE_CENTRAL || HL_LE_BROADCASTER)

#if (HL_LE_OBSERVER || HL_LE_BROADCASTER)
/// Structure of Start Channel Assessment procedure
typedef struct gapm_cfg_proc_start_ch_assess
{
    /// Procedure inheritance
    gapm_proc_send_hci_cmd_t hdr;
    /// Scan window duration in us
    uint32_t scan_win_duration;
    /// Minimum Channel Scanning event in us
    uint32_t scan_duration_min;
    /// Maximum Channel Scanning event in us
    uint32_t scan_duration_max;
    /// Channel Scanning interval in Time = N*1.25ms
    uint16_t intv;
} gapm_cfg_proc_start_ch_assess_t;

/// Structure of Stop Channel Assessment procedure
typedef struct gapm_cfg_proc_stop_ch_assess
{
    /// Procedure inheritance
    gapm_proc_send_hci_cmd_t hdr;

} gapm_cfg_proc_stop_ch_assess_t;
#endif // (HL_LE_OBSERVER || HL_LE_BROADCASTER)

/// Structure of Set Device Name procedure
typedef struct gapm_cfg_proc_set_name
{
    /// Procedure inheritance
    gapm_proc_set_cfg_t hdr;
    /// Pointer to the device name
    const uint8_t*      p_name;
    /// Length of device name
    uint8_t             name_len;
} gapm_cfg_proc_set_name_t;

#if (BLE_HOST_IQ_GEN)
/// Structure of Configure IQ Generator procedure
typedef struct gapm_cfg_proc_dbg_iqgen
{
    /// Procedure inheritance
    gapm_proc_send_hci_cmd_t hdr;
    /// Antenna switch/sample control
    uint8_t                  mode;
    /// Number of antenna patterns
    uint8_t                  nb_antenna;
    /// I/Q sample control
    gapm_dbg_iq_ctrl_t       iq_ctrl[__ARRAY_EMPTY];
} gapm_cfg_proc_dbg_iqgen_t;
#endif // (BLE_HOST_IQ_GEN)

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

__STATIC bool gapm_cfg_proc_setup_config_transition(gapm_cfg_proc_setup_t* p_proc, uint8_t event, uint16_t status);
__STATIC bool gapm_cfg_proc_setup_reset_transition(gapm_cfg_proc_setup_t* p_proc, uint8_t event, uint16_t status);
__STATIC bool gapm_cfg_proc_set_name_transition(gapm_cfg_proc_set_name_t* p_proc, uint8_t event, uint16_t status);

#if(BT_HOST_PRESENT)
__STATIC void gapm_bk_al_reset_done_cb(uint16_t status);
#endif // (BT_HOST_PRESENT)


/// Configuration procedure interface
__STATIC const hl_proc_itf_t gapm_cfg_proc_setup_config_itf =
{
    .transition  = (hl_proc_transition_cb) gapm_cfg_proc_setup_config_transition,
    .cleanup     = hl_proc_cleanup,
};

/// Reset procedure interface
__STATIC const hl_proc_itf_t gapm_cfg_proc_setup_reset_itf =
{
    .transition  = (hl_proc_transition_cb) gapm_cfg_proc_setup_reset_transition,
    .cleanup     = hl_proc_cleanup,
};

/// Set Device name procedure interface
__STATIC const hl_proc_itf_t gapm_cfg_proc_set_name_itf =
{
    .transition  = (hl_proc_transition_cb) gapm_cfg_proc_set_name_transition,
    .cleanup     = hl_proc_cleanup,
};

/*
 * HCI HANDLERS
 ****************************************************************************************
 */

#if ((!BLE_EMB_PRESENT) && (HL_LE_CENTRAL || HL_LE_PERIPHERAL))
/**
 ****************************************************************************************
 * @brief Handles LE read buffer size command complete event.
 * Used to read the maximum size of the data portion of HCI LE ACL Data Packets sent
 * from the Host to the Controller.
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_cfg_hci_le_rd_buf_size_cmd_cmp_evt_handler(uint16_t opcode, uint16_t event,
                                                        struct hci_le_rd_buf_size_cmd_cmp_evt const *p_evt)
{
    if(p_evt->status == CO_ERROR_NO_ERROR)
    {
        /* update the buffer size */
        l2cap_chan_ll_buf_info_set(p_evt->hc_data_pk_len, p_evt->hc_tot_nb_data_pkts);
    }

    gapm_proc_transition(GAPM_PROC_CFG, event, RW_ERR_HCI_TO_HL(p_evt->status));
}

/// Handles read buffer size command complete event.
__STATIC void gapm_cfg_hci_rd_buf_size_cmd_cmp_evt_handler(uint16_t opcode, uint16_t event,
                                                            struct hci_rd_buf_size_cmd_cmp_evt const *p_evt)
{
    if(p_evt->status == CO_ERROR_NO_ERROR)
    {
        // sanity check
        ASSERT_ERR(p_evt->hc_tot_nb_data_pkts != 0);
        /* update the buffer size */
        l2cap_chan_ll_buf_info_set(p_evt->hc_data_pk_len, p_evt->hc_tot_nb_data_pkts);
    }

    gapm_proc_transition(GAPM_PROC_CFG, event, RW_ERR_HCI_TO_HL(p_evt->status));
}
#endif // ((!BLE_EMB_PRESENT) && (HL_LE_CENTRAL || HL_LE_PERIPHERAL))


#if (!EMB_PRESENT)
/// Handles the read Bluetooth device address complete event.
__STATIC void gapm_cfg_hci_rd_bd_addr_cmd_cmp_evt_handler(uint16_t opcode, uint16_t event,
                                                            struct hci_rd_bd_addr_cmd_cmp_evt const *p_evt)
{
    /* Store Local BD address */
    memcpy(&(gapm_env.addr), &(p_evt->local_addr), BD_ADDR_LEN);

    gapm_proc_transition(GAPM_PROC_CFG, event, RW_ERR_HCI_TO_HL(p_evt->status));
}
#endif // (!EMB_PRESENT)



#if (!BLE_EMB_PRESENT && HL_LE_BROADCASTER)
/**
 ****************************************************************************************
 * @brief Handle reception of HCI Read Maximum Advertising Data Length command complete
 * event.
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_hci_le_read_max_adv_data_len_cmd_cmp_evt_handler(uint16_t opcode, uint16_t event,
                                                           struct hci_le_rd_max_adv_data_len_cmd_cmp_evt const *p_evt)
{
    if (p_evt->status == CO_ERROR_NO_ERROR)
    {
        // Keep the read length in mind, should not changed
        gapm_env.max_adv_data_len = p_evt->max_adv_data_len;
    }
    // Continue reset procedure
    gapm_proc_transition(GAPM_PROC_CFG, event, RW_ERR_HCI_TO_HL(p_evt->status));
}

/**
 ****************************************************************************************
 * @brief Handle reception of HCI Read Number of Supported Advertising Sets command complete
 * event. The read value may change over time.
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_cfg_hci_le_rd_nb_supp_adv_sets_cmd_cmp_evt_handler(uint16_t opcode, uint16_t event,
                                                         struct hci_le_rd_nb_supp_adv_sets_cmd_cmp_evt const *p_evt)
{
    if (p_evt->status == CO_ERROR_NO_ERROR)
    {
        // Keep the value in mind
        gapm_env.max_adv_set = p_evt->nb_supp_adv_sets;
    }

    // Continue reset procedure
    gapm_proc_transition(GAPM_PROC_CFG, event, RW_ERR_HCI_TO_HL(p_evt->status));
}
#endif //(!BLE_EMB_PRESENT && HL_LE_BROADCASTER)

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/// check if procedure is finished, and if yes handle it.
bool gapm_cfg_proc_handle_if_finished(gapm_cfg_proc_setup_t* p_proc, uint16_t status, bool is_finished)
{
    if(status != GAP_ERR_NO_ERROR)
    {
        is_finished = true;
    }

    if(is_finished)
    {
        gapm_proc_execute_cmp_cb(&(p_proc->hdr), status);
    }

    return is_finished;
}

/**
 ****************************************************************************************
 * @brief Function called when an event is trigger that creates a transition in procedure state machine
 *        Device reset
 *
 * @param[in] p_proc     Pointer to procedure object
 * @param[in] event      Event type receive that induce procedure state transition (see enum #gapm_cfg_proc_evt)
 * @param[in] status     Status linked to event transition (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC bool gapm_cfg_proc_setup_reset_transition(gapm_cfg_proc_setup_t* p_proc, uint8_t event, uint16_t status)
{
    bool is_finished = false;
    bool do_configure = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            #if !(EMB_PRESENT)
            {
                // nothing to do, wait for completion of HCI Reset function
            }break;
            case GAPM_CFG_RESET_HCI:
            {
                /* default device mask */
                uint8_t default_dev_mask[EVT_MASK_LEN] = GAP_EVT_MASK;
                gapm_env.do_reset = false; // Mark that HCI reset has been done

                /* query set event mask */
                struct hci_set_evt_mask_cmd *p_event_mask =
                        HL_HCI_CMD_ALLOC(HCI_SET_EVT_MASK_CMD_OPCODE, hci_set_evt_mask_cmd);
                if(p_event_mask == NULL)
                {
                    status = GAP_ERR_INSUFF_RESOURCES;
                    break;
                }
                /* fill up the event masking */
                memcpy(&p_event_mask->event_mask.mask[0], default_dev_mask, EVT_MASK_LEN);

                /* send the event mask */
                HL_HCI_CMD_SEND_TO_CTRL(p_event_mask, GAPM_CFG_RESET_SET_EVT_MASK, gapm_default_cfg_hci_cmd_cmp_evt_handler);
            }
            break;
            case GAPM_CFG_RESET_SET_EVT_MASK:
            #if (BLE_HOST_PRESENT)
            {
                /* default device mask */
                uint8_t default_dev_mask[EVT_MASK_LEN] = GAP_LE_EVT_MASK;

                /* send query LE event mask */
                struct hci_le_set_evt_mask_cmd *p_event_mask =
                        HL_HCI_CMD_ALLOC(HCI_LE_SET_EVT_MASK_CMD_OPCODE, hci_le_set_evt_mask_cmd);

                if(p_event_mask == NULL)
                {
                    status = GAP_ERR_INSUFF_RESOURCES;
                    break;
                }
                /* fill up the event masking */
                memcpy(&p_event_mask->le_mask.mask[0], default_dev_mask, EVT_MASK_LEN);

                /* send the event mask */
                HL_HCI_CMD_SEND_TO_CTRL(p_event_mask, GAPM_CFG_RESET_LE_SET_EVT_MASK, gapm_default_cfg_hci_cmd_cmp_evt_handler);
            } break;

            case GAPM_CFG_RESET_LE_SET_EVT_MASK:
            #endif // (BLE_HOST_PRESENT)
            {
                /* Get local device address. */
                status = HL_HCI_BASIC_CMD_SEND(HCI_RD_BD_ADDR_CMD_OPCODE, GAPM_CFG_RESET_RD_BD_ADDR,
                                               gapm_cfg_hci_rd_bd_addr_cmd_cmp_evt_handler);
            } break;

            case GAPM_CFG_RESET_RD_BD_ADDR:
            #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
            {
                /* send read buffer size */
                status = HL_HCI_BASIC_CMD_SEND(HCI_LE_RD_BUF_SIZE_CMD_OPCODE, GAPM_CFG_RESET_LE_RD_BUF_SIZE,
                                               gapm_cfg_hci_le_rd_buf_size_cmd_cmp_evt_handler);

            } break;
            case GAPM_CFG_RESET_LE_RD_BUF_SIZE:
            {
                // check if number of ble packet is available
                if(l2cap_chan_ll_buf_nb_get() == 0)
                {
                    // number of buffer are shared with BT ACL packets

                    // Send read buffer size legacy command to link layer
                    status = HL_HCI_BASIC_CMD_SEND(HCI_RD_BUF_SIZE_CMD_OPCODE, GAPM_CFG_RESET_RD_BUF_SIZE,
                                                   gapm_cfg_hci_rd_buf_size_cmd_cmp_evt_handler);
                    break;
                }
            }
            // no break
            case GAPM_CFG_RESET_RD_BUF_SIZE:
            #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
            #if (HL_LE_BROADCASTER)
            {
                // Send read maximum advertising data length command
                status = HL_HCI_BASIC_CMD_SEND(HCI_LE_RD_MAX_ADV_DATA_LEN_CMD_OPCODE, GAPM_CFG_RESET_RD_MAX_ADV_DATA_LEN,
                                               gapm_hci_le_read_max_adv_data_len_cmd_cmp_evt_handler);
            } break;
            case GAPM_CFG_RESET_RD_MAX_ADV_DATA_LEN:
            {
                // Send read number of supported advertising sets command to the controller
                status = HL_HCI_BASIC_CMD_SEND(HCI_LE_RD_NB_SUPP_ADV_SETS_CMD_OPCODE, GAPM_CFG_RESET_RD_NB_ADV_SETS,
                                               gapm_cfg_hci_le_rd_nb_supp_adv_sets_cmd_cmp_evt_handler);
            } break;
            case GAPM_CFG_RESET_RD_NB_ADV_SETS:
            #endif //(HL_LE_BROADCASTER)
            #else // (EMB_PRESENT)
            {
                /* BD address length */
                uint8_t bd_addr_len = BD_ADDR_LEN;
                // try to read BD address from NVDS
                if (rwip_param.get(PARAM_ID_BD_ADDRESS, &bd_addr_len, (uint8_t *)&gapm_env.addr) != PARAM_OK)
                {
                    /* Store Local BD address */
                    memcpy(&(gapm_env.addr), &co_default_bdaddr, sizeof(co_default_bdaddr));
                }

                #if (BLE_L2CAP)
                // Set default value for link layer buffer size
                l2cap_chan_ll_buf_info_set(BLE_MAX_OCTETS, BLE_ACL_BUF_NB_TX);
                #endif //(BLE_L2CAP)

                #if (HL_LE_BROADCASTER)
                // Set default value for maximum advertising data length
                gapm_env.max_adv_data_len = BLE_CFG_MAX_ADV_DATA_LEN;

                // Set default value for number of advertising sets
                gapm_env.max_adv_set = HOST_ACTIVITY_MAX;
                #endif //(HL_LE_BROADCASTER)
            }
            #endif // !(BLE_EMB_PRESENT)
            // no break
            default:
            {
                gapm_env.reset = true;
                do_configure   = p_proc->do_configure;
                is_finished    = true;
            } break;
        }
    }

    if(do_configure)
    {
        // change procedure interface to configuration one
        p_proc->hdr.hdr.p_itf = &gapm_cfg_proc_setup_config_itf;
        // execute configuration procedure grant
        is_finished =  gapm_cfg_proc_setup_config_transition(p_proc, HL_PROC_GRANTED, status);
    }
    else
    {
        is_finished = gapm_cfg_proc_handle_if_finished(p_proc, status, is_finished);
    }

    return is_finished;
}

/**
 ****************************************************************************************
 * @brief Function called when an event is trigger that creates a transition in procedure state machine
 *        Device configuration
 *
 * @param[in] p_proc     Pointer to procedure object
 * @param[in] event      Event type receive that induce procedure state transition (see enum #gapm_cfg_proc_evt)
 * @param[in] status     Status linked to event transition (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC bool gapm_cfg_proc_setup_config_transition(gapm_cfg_proc_setup_t* p_proc, uint8_t event, uint16_t status)
{
    bool is_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        gapm_config_t* p_cfg = &(p_proc->cfg);

        switch(event)
        {
            case HL_PROC_GRANTED:
            #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
            {
                #if (BLE_EMB_PRESENT)
                // Check if default value updated
                if((p_cfg->tx_pref_phy != 0) || (p_cfg->rx_pref_phy != 0))
                #endif // (BLE_EMB_PRESENT)
                {
                    // Write Default LE PHY
                    struct hci_le_set_dft_phy_cmd *p_dft_phy =
                            HL_HCI_CMD_ALLOC(HCI_LE_SET_DFT_PHY_CMD_OPCODE, hci_le_set_dft_phy_cmd);
                    if(p_dft_phy == NULL)
                    {
                        status = GAP_ERR_INSUFF_RESOURCES;
                        break;
                    }

                    // Fill data
                    p_dft_phy->all_phys  = (p_cfg->tx_pref_phy == GAP_PHY_ANY) ? ALL_PHYS_TX_NO_PREF : 0;
                    p_dft_phy->all_phys |= (p_cfg->rx_pref_phy == GAP_PHY_ANY) ? ALL_PHYS_RX_NO_PREF : 0;

                    p_dft_phy->rx_phys  = p_cfg->rx_pref_phy;
                    p_dft_phy->tx_phys  = p_cfg->tx_pref_phy;

                    /* send the message */
                    HL_HCI_CMD_SEND_TO_CTRL(p_dft_phy, GAPM_CFG_SETUP_SET_LE_DFT_PHY_CMD,
                                            gapm_default_cfg_hci_cmd_cmp_evt_handler);

                    // here break out of "switch(current_state)"
                    break;
                }
                // else we continue checking the next step without break
            }
            // no break
            case GAPM_CFG_SETUP_SET_LE_DFT_PHY_CMD:
            #endif //(HL_LE_CENTRAL || HL_LE_PERIPHERAL)
            #if (BLE_HOST_PRESENT)
            {
                #if (BLE_EMB_PRESENT)
                // Check if default value updated
                if((p_cfg->tx_path_comp != 0) || (p_cfg->rx_path_comp != 0))
                #endif // (BLE_EMB_PRESENT)
                {
                    // Write RF Path Compensation values
                    struct hci_le_wr_rf_path_comp_cmd *p_cmd =
                            HL_HCI_CMD_ALLOC(HCI_LE_WR_RF_PATH_COMP_CMD_OPCODE, hci_le_wr_rf_path_comp_cmd);
                    if(p_cmd == NULL)
                    {
                        status = GAP_ERR_INSUFF_RESOURCES;
                        break;
                    }
                    // Fill data
                    p_cmd->tx_path_comp = p_cfg->tx_path_comp;
                    p_cmd->rx_path_comp = p_cfg->rx_path_comp;

                    /* send the message */
                    HL_HCI_CMD_SEND_TO_CTRL(p_cmd, GAPM_CFG_SETUP_WR_RF_PATH_COMP_CMD,
                                            gapm_default_cfg_hci_cmd_cmp_evt_handler);

                    // here break out of "switch(current_state)"
                    break;
                }
                // else we continue checking the next step without break
            }
            // no break

            case GAPM_CFG_SETUP_WR_RF_PATH_COMP_CMD:
            #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
            {
                #if (BLE_EMB_PRESENT)
                // Check if default value updated
                if((p_cfg->sugg_max_tx_octets != BLE_MIN_OCTETS) || (p_cfg->sugg_max_tx_time != BLE_MIN_TIME))
                #endif // (BLE_EMB_PRESENT)
                {
                    // Write Suggested Default LE Data Length
                    struct hci_le_wr_suggted_dft_data_len_cmd *p_set_sugg_data =
                            HL_HCI_CMD_ALLOC(HCI_LE_WR_SUGGTED_DFT_DATA_LEN_CMD_OPCODE,
                                             hci_le_wr_suggted_dft_data_len_cmd);
                    if(p_set_sugg_data == NULL)
                    {
                        status = GAP_ERR_INSUFF_RESOURCES;
                        break;
                    }
                    // Fill data
                    p_set_sugg_data->suggted_max_tx_octets = p_cfg->sugg_max_tx_octets;
                    p_set_sugg_data->suggted_max_tx_time = p_cfg->sugg_max_tx_time;

                    /* send the message */
                    HL_HCI_CMD_SEND_TO_CTRL(p_set_sugg_data, GAPM_CFG_SETUP_WR_LE_DFT_DATA_LEN_CMD,
                                            gapm_default_cfg_hci_cmd_cmp_evt_handler);

                    // here break out of "switch(current_state)"
                    break;
                }
                // else we continue checking the next step without break
            }
            // no break
            case GAPM_CFG_SETUP_WR_LE_DFT_DATA_LEN_CMD:
            #endif //(HL_LE_CENTRAL || HL_LE_PERIPHERAL)
            #if (!BLE_EMB_PRESENT && BLE_ISO_PRESENT)
            {
                // Write Suggested Default LE Data Length
                struct hci_le_set_host_feature_cmd *p_set_host_feature =
                        HL_HCI_CMD_ALLOC(HCI_LE_SET_HOST_FEATURE_CMD_OPCODE, hci_le_set_host_feature_cmd);

                if(p_set_host_feature == NULL)
                {
                    status = GAP_ERR_INSUFF_RESOURCES;
                    break;
                }
                // Fill data
                p_set_host_feature->bit_number = BLE_FEAT_ISO_CHANNELS_HOST_SUPPORT;
                p_set_host_feature->bit_value  = 1;

                /* send the message */
                HL_HCI_CMD_SEND_TO_CTRL(p_set_host_feature, GAPM_CFG_SETUP_LE_SET_HOST_FEATURE_CMD,
                                        gapm_default_cfg_hci_cmd_cmp_evt_handler);
            } break;
            case GAPM_CFG_SETUP_LE_SET_HOST_FEATURE_CMD:
            #endif // (!BLE_EMB_PRESENT && BLE_ISO_PRESENT)
            {
                // Set Address configuration.
                gapm_env.priv_cfg = p_cfg->privacy_cfg;
                gapm_env.renew_dur = co_max(p_cfg->renew_dur, GAP_TMR_PRIV_ADDR_MIN);
                gapm_env.renew_dur = co_min(gapm_env.renew_dur, GAP_TMR_PRIV_ADDR_MAX);

                // Check if controller privacy is enabled
                if (p_cfg->privacy_cfg & GAPM_PRIV_CFG_PRIV_EN_BIT)
                {
                    // Set timeout for address generation
                    struct hci_le_set_rslv_priv_addr_to_cmd *p_set_to =
                            HL_HCI_CMD_ALLOC(HCI_LE_SET_RSLV_PRIV_ADDR_TO_CMD_OPCODE, hci_le_set_rslv_priv_addr_to_cmd);

                    if(p_set_to == NULL)
                    {
                        status = GAP_ERR_INSUFF_RESOURCES;
                        break;
                    }
                    // Fill data
                    p_set_to->rpa_timeout = p_cfg->renew_dur;

                    /* send the message */
                    HL_HCI_CMD_SEND_TO_CTRL(p_set_to, GAPM_CFG_SETUP_SET_RENEW_TO, gapm_default_cfg_hci_cmd_cmp_evt_handler);
                    break;
                }
                // else we continue checking the next step without break
            }
            // no break

            case GAPM_CFG_SETUP_SET_RENEW_TO:
            {
                // Check if controller privacy is enabled
                if (p_cfg->privacy_cfg & GAPM_PRIV_CFG_PRIV_EN_BIT)
                {
                    // Enable Address Resolution in controller
                    struct hci_le_set_addr_resol_en_cmd *p_en_addr_resol =
                            HL_HCI_CMD_ALLOC(HCI_LE_SET_ADDR_RESOL_EN_CMD_OPCODE, hci_le_set_addr_resol_en_cmd);

                    if(p_en_addr_resol == NULL)
                    {
                        status = GAP_ERR_INSUFF_RESOURCES;
                        break;
                    }
                    // Fill data
                    p_en_addr_resol->enable = 1;

                    /* send the message */
                    HL_HCI_CMD_SEND_TO_CTRL(p_en_addr_resol, GAPM_CFG_SETUP_EN_CTRL_PRIV, gapm_default_cfg_hci_cmd_cmp_evt_handler);
                    break;
                }
                // else we continue checking the next step without break
            }
            // no break

            case GAPM_CFG_SETUP_EN_CTRL_PRIV:
            #endif // (BLE_HOST_PRESENT)
            #if(BT_HOST_PRESENT)
            {
                // Reset BT Host
                status =  bk_al_configure(p_cfg->class_of_device, p_cfg->dflt_link_policy,
                                          p_cfg->ssp_enable, gapm_bk_al_reset_done_cb);

                if(status == GAP_ERR_NO_ERROR)
                {
                    break;
                }
            }
            // no break
            case GAPM_CFG_SETUP_BT_HOST:
            #endif // (BT_HOST_PRESENT)
            {
                is_finished = true;
                gapm_env.configured = true;
            } break;

            default:
            {
                ASSERT_INFO(0, event, status);
                // Abort of reset procedure
                status = GAP_ERR_PROTOCOL_PROBLEM;
                is_finished = true;
            } break;
        }
    }

    return gapm_cfg_proc_handle_if_finished(p_proc, status, is_finished);
}

/**
 ****************************************************************************************
 * @brief Function called when an event is trigger that creates a transition in procedure state machine
 *
 * @param[in] p_proc     Pointer to procedure object
 * @param[in] event      Event type receive that induce procedure state transition (see enum #gapm_cfg_proc_evt)
 * @param[in] status     Status linked to
 ****************************************************************************************
 */
__STATIC bool gapm_cfg_proc_set_name_transition(gapm_cfg_proc_set_name_t* p_proc, uint8_t event, uint16_t status)
{
    bool finished = false;

    switch(event)
    {
        case HL_PROC_GRANTED:
        #if (BT_HOST_PRESENT)
        {
            if(GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_BT_CLASSIC))
            {
                struct hci_wr_local_name_cmd *p_cmd =
                        HL_HCI_CMD_ALLOC(HCI_WR_LOCAL_NAME_CMD_OPCODE, hci_wr_local_name_cmd);

                if(p_cmd != NULL)
                {
                    memcpy(&(p_cmd->name.name[0]), p_proc->p_name, p_proc->name_len);
                    // force trailing char to zero
                    memset(&(p_cmd->name.name[p_proc->name_len]), 0, BD_NAME_SIZE - p_proc->name_len);
                    HL_HCI_CMD_SEND_TO_CTRL(p_cmd, HL_PROC_FINISHED, gapm_default_cfg_hci_cmd_cmp_evt_handler);
                    break;
                }
                else
                {
                    status = GAP_ERR_INSUFF_RESOURCES;
                }
            }
        }
        #endif // (BT_HOST_PRESENT)
        // no break
        default:
        {
            finished = true;
            if(status == GAP_ERR_NO_ERROR)
            {
                gapm_env.p_name   = p_proc->p_name;
                gapm_env.name_len = p_proc->name_len;
            }
        } break;
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        finished = true;
    }

    if(finished)
    {
        // inform upper layer about procedure execution
        gapm_proc_execute_cmp_cb(&(p_proc->hdr), status);
    }

    return (finished);
}

#if (HL_LE_CENTRAL || HL_LE_BROADCASTER)
/// @brief Prepare HCI_LE_SET_HOST_CH_CLASS_CMD command
__STATIC void gapm_cfg_prepare_set_chmap_hci_cmd(gapm_cfg_proc_set_chmap_t* p_proc, struct hci_le_set_host_ch_class_cmd *p_cmd)
{
    // update channel map value
    memcpy(&p_cmd->chmap.map[0], &p_proc->chmap.map[0], LE_CHNL_MAP_LEN);
}

/// Send HCI_LE_SET_HOST_CH_CLASS_CMD command information
__STATIC const gapm_proc_hci_cmd_info_t gapm_cfg_send_set_chmap_hci_cmd_info =
{
    .prep_cb     = (gapm_proc_prepare_hci_cmd_cb) gapm_cfg_prepare_set_chmap_hci_cmd,
    .cmd_op_code = HCI_LE_SET_HOST_CH_CLASS_CMD_OPCODE,
    .cmd_length  = sizeof(struct hci_le_set_host_ch_class_cmd),
};

#endif // (HL_LE_CENTRAL || HL_LE_BROADCASTER)

#if (HL_LE_OBSERVER || HL_LE_BROADCASTER)
/// @brief Prepare HCI_VS_LE_CH_SCAN_CMD_OPCODE command
__STATIC void gapm_cfg_prepare_create_ch_scan_hci_cmd(gapm_cfg_proc_start_ch_assess_t* p_proc, struct hci_vs_le_ch_scan_cmd *p_cmd)
{
    // update channel map value
    p_cmd->scan_win_duration =  p_proc->scan_win_duration;
    p_cmd->scan_duration_max =  p_proc->scan_duration_max;
    p_cmd->scan_duration_min =  p_proc->scan_duration_min;
    p_cmd->intv =  p_proc->intv;
}

/// Send HCI_VS_LE_CH_SCAN_CMD_OPCODE command information
__STATIC const gapm_proc_hci_cmd_info_t gapm_cfg_send_ch_scan_hci_cmd_info =
{
    .prep_cb     = (gapm_proc_prepare_hci_cmd_cb) gapm_cfg_prepare_create_ch_scan_hci_cmd,
    .cmd_op_code = HCI_VS_LE_CH_SCAN_CMD_OPCODE,
    .cmd_length  = sizeof(struct hci_vs_le_ch_scan_cmd),
};

/// Send HCI_VS_LE_CH_SCAN_END_CMD_OPCODE command information
__STATIC const gapm_proc_hci_cmd_info_t gapm_cfg_send_ch_scan_end_hci_cmd_info =
{
    .prep_cb     = (gapm_proc_prepare_hci_cmd_cb) gapm_proc_empty,
    .cmd_op_code = HCI_VS_LE_CH_SCAN_END_CMD_OPCODE,
    .cmd_length  = 0,
};
#endif // (HL_LE_OBSERVER || HL_LE_BROADCASTER)


#if (BLE_HOST_IQ_GEN)
/// @brief Prepare HCI_DBG_IQGEN_CFG_CMD command
__STATIC void gapm_cfg_prepare_dbg_iqgen_cfg_hci_cmd(gapm_cfg_proc_dbg_iqgen_t* p_proc, struct hci_dbg_iqgen_cfg_cmd* p_cmd)
{
    uint8_t cursor;

    p_cmd->nb_antenna            = p_proc->nb_antenna;
    p_cmd->mode                  = p_proc->mode;

    for(cursor = 0 ; cursor < p_proc->nb_antenna ; cursor++)
    {
        p_cmd->iq_ctrl[cursor].i = p_proc->iq_ctrl[cursor].i;
        p_cmd->iq_ctrl[cursor].q = p_proc->iq_ctrl[cursor].q;
    }
}

/// Send HCI_DBG_IQGEN_CFG_CMD command information
__STATIC const gapm_proc_hci_cmd_info_t gapm_cfg_send_dbg_iqgen_cfg_hci_cmd_info =
{
    .prep_cb     = (gapm_proc_prepare_hci_cmd_cb) gapm_cfg_prepare_dbg_iqgen_cfg_hci_cmd,
    .cmd_op_code = HCI_DBG_IQGEN_CFG_CMD_OPCODE,
    .cmd_length  = sizeof(struct hci_dbg_iqgen_cfg_cmd),
};


#endif // (BLE_HOST_IQ_GEN)

#if(BT_HOST_PRESENT)
// -------------------------------------------------------------------------------------
// ------------------                 BT HOST HANDLER            -----------------------
// -------------------------------------------------------------------------------------


/**
 ****************************************************************************************
 * @brief Handles BT Host reset completion
 *
 * @param[in] status    status of procedure exection (see enum #hl_err)
 *
 ****************************************************************************************
 */
__STATIC void gapm_bk_al_reset_done_cb(uint16_t status)
{
    gapm_proc_transition(GAPM_PROC_CFG, GAPM_CFG_SETUP_BT_HOST, status);
}
#endif // (BT_HOST_PRESENT)

/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


uint16_t gapm_reset(uint32_t dummy, gapm_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    do
    {
        gapm_cfg_proc_setup_t* p_proc = NULL;
        if(cmp_cb == NULL)
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // perform an IP reset.
        rwip_reset();

        #if !(EMB_PRESENT)
        // send a reset message to lower layers
        status = HL_HCI_BASIC_CMD_SEND(HCI_RESET_CMD_OPCODE, GAPM_CFG_RESET_HCI, gapm_default_cfg_hci_cmd_cmp_evt_handler);
        ASSERT_ERR(status == GAP_ERR_NO_ERROR);
        if(status != GAP_ERR_NO_ERROR) break;
        gapm_env.do_reset = true; // Mark that HCI reset is on-going
        #endif // !(EMB_PRESENT)

        // Create new procedure
        status = gapm_proc_set_cfg_create(sizeof(gapm_cfg_proc_setup_t) - sizeof(gapm_config_t),
                                          &gapm_cfg_proc_setup_reset_itf, dummy, cmp_cb, (gapm_proc_set_cfg_t**) &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        p_proc->do_reset     = true;
        p_proc->do_configure = false;
    } while (0);

    return (status);
}

uint16_t gapm_set_dev_config(uint32_t dummy, const gapm_config_t* p_cfg, const gapm_callbacks_t* p_cbs, gapm_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    do
    {
        bool do_reset = (!gapm_env.reset || gapm_env.configured); // already configured or not yet reset, force reset
        uint8_t authorized_role = GAP_ROLE_NONE;
        uint8_t targeted_role;
        gapm_cfg_proc_setup_t* p_proc = NULL;

        if((p_cfg == NULL) || (cmp_cb == NULL))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // remove unsupported bit in role bit field
        targeted_role = p_cfg->role & GAP_ROLE_BTDM;

        // create a variable used to check if role is supported or not
        #if (HL_LE_BROADCASTER)
        authorized_role |= GAP_ROLE_BROADCASTER;
        #endif // (HL_LE_BROADCASTER)

        #if (HL_LE_OBSERVER)
        authorized_role |= GAP_ROLE_OBSERVER;
        #endif // (HL_LE_OBSERVER)

        #if (HL_LE_CENTRAL)
        authorized_role |= GAP_ROLE_CENTRAL;
        #endif // (HL_LE_CENTRAL)

        #if (HL_LE_PERIPHERAL)
        authorized_role |= GAP_ROLE_PERIPHERAL;
        #endif // (HL_LE_PERIPHERAL)

        #if (BT_HOST_PRESENT)
        authorized_role |= GAP_ROLE_BT_CLASSIC;
        #endif // (BT_HOST_PRESENT)

        // Check that configured role is in authorized role
        if((targeted_role == GAP_ROLE_NONE) || ((targeted_role | authorized_role) != authorized_role))
        {
            /* Role which is configured is not supported by the Host stack */
            status = GAP_ERR_NOT_SUPPORTED;
            break;
        }

        #if (GAPC_PRESENT)
        // Callback configuration check
        if(   ((targeted_role & GAP_ROLE_PERIPHERAL) != 0) || ((targeted_role & GAP_ROLE_CENTRAL) != 0)
           || ((targeted_role & GAP_ROLE_BT_CLASSIC) != 0))
        {
            const gapc_connection_info_cb_t* p_info_cbs    = p_cbs->p_info_cbs;
            const gapc_connection_req_cb_t*  p_con_req_cbs = p_cbs->p_con_req_cbs;
            const gapc_security_cb_t*        p_sec_cbs     = p_cbs->p_sec_cbs;

            if(   (p_cbs == NULL) || (p_info_cbs == NULL) || (p_con_req_cbs == NULL)  || (p_sec_cbs == NULL)
               || (p_info_cbs->disconnected == NULL) || (p_sec_cbs->key_received == NULL))
            {
                status = GAP_ERR_MISSING_CALLBACK;
                break;
            }

            if((p_cfg->pairing_mode != 0) || ((targeted_role & GAP_ROLE_BT_CLASSIC) != 0))
            {
                if(   (p_sec_cbs->info_req == NULL) || (p_sec_cbs->pairing_succeed == NULL)
                   || (p_sec_cbs->pairing_failed == NULL))
                {
                    status = GAP_ERR_MISSING_CALLBACK;
                    break;
                }
            }

            #if (BLE_GAPC)
            if(((targeted_role & GAP_ROLE_PERIPHERAL) != 0) || ((targeted_role & GAP_ROLE_CENTRAL) != 0))
            {
                const gapc_le_config_cb_t* p_le_config_cbs = p_cbs->p_le_config_cbs;

                // Here until GATT not supported on BR/EDR
                if(   (p_info_cbs->name_get == NULL) || (p_info_cbs->appearance_get == NULL)
                   || (GETF(p_cfg->att_cfg, GAPM_ATT_SLV_PREF_CON_PAR_EN) && (p_info_cbs->slave_pref_param_get == NULL))
                   || ((GETF(p_cfg->att_cfg, GAPM_ATT_NAME_PERM) != GAPM_WRITE_DISABLE) &&  (p_info_cbs->name_set == NULL))
                   || ((GETF(p_cfg->att_cfg, GAPM_ATT_APPEARENCE_PERM) != GAPM_WRITE_DISABLE) && (p_info_cbs->appearance_set == NULL)))
                {
                    status = GAP_ERR_MISSING_CALLBACK;
                    break;
                }

                // check callback for LE pairing
                if(p_cfg->pairing_mode != 0)
                {
                    if(   ((targeted_role & GAP_ROLE_PERIPHERAL) != 0)
                       && ((p_sec_cbs->le_encrypt_req == NULL) || (p_sec_cbs->pairing_req == NULL)))
                    {
                        status = GAP_ERR_MISSING_CALLBACK;
                        break;
                    }

                    if(   ((p_cfg->pairing_mode & GAPM_PAIRING_LEGACY) != 0)
                       && ((p_sec_cbs->ltk_req == NULL)))
                    {
                        status = GAP_ERR_MISSING_CALLBACK;
                        break;
                    }

                    if(   ((p_cfg->pairing_mode & GAPM_PAIRING_SEC_CON) != 0)
                       && ((p_sec_cbs->numeric_compare_req == NULL)))
                    {
                        status = GAP_ERR_MISSING_CALLBACK;
                        break;
                    }
                }

                if((p_le_config_cbs == NULL) || (p_con_req_cbs->le_connection_req == NULL))
                {
                    status = GAP_ERR_MISSING_CALLBACK;
                    break;
                }
            }
            #endif // (BLE_GAPC)

            #if (BT_HOST_PRESENT)
            if((targeted_role & GAP_ROLE_BT_CLASSIC) != 0)
            {
                if(   (p_con_req_cbs->bt_connection_req == NULL) || (p_sec_cbs->bt_encrypt_req == NULL)
                   || (p_sec_cbs->numeric_compare_req == NULL) || (p_sec_cbs->display_passkey == NULL))
                {
                    status = GAP_ERR_MISSING_CALLBACK;
                    break;
                }
            }
            #endif // (BT_HOST_PRESENT)
        }
        #endif // (GAPC_PRESENT)

        if(do_reset)
        {
            // perform an IP reset.
            rwip_reset();
            #if !(EMB_PRESENT)
            // send a reset message to lower layers
            status = HL_HCI_BASIC_CMD_SEND(HCI_RESET_CMD_OPCODE, GAPM_CFG_RESET_HCI, gapm_default_cfg_hci_cmd_cmp_evt_handler);
            ASSERT_ERR(status == GAP_ERR_NO_ERROR);
            if(status != GAP_ERR_NO_ERROR) break;
            gapm_env.do_reset = true; // Mark that HCI reset is on-going
            #endif // !(EMB_PRESENT)
        }

        #if (GAPC_PRESENT)
        gapc_set_callbacks(p_cbs);
        #endif // (GAPC_PRESENT)

        // clear role and config
        gapm_env.priv_cfg = 0;
        gapm_env.configured = false;

        #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
        // database available only for central and peripheral roles.
        if(  ((targeted_role & GAP_ROLE_PERIPHERAL) == GAP_ROLE_PERIPHERAL)
           ||((targeted_role & GAP_ROLE_CENTRAL) == GAP_ROLE_CENTRAL))
        {
            bool central_res_en = false;

            // check if central resolution is enabled
            if ((targeted_role & GAP_ROLE_CENTRAL) && ((p_cfg->privacy_cfg & GAPM_PRIV_CFG_PRIV_EN_BIT) != 0))
            {
                central_res_en = true;
            }

            // Create GAP and GATT service
            status = gapc_svc_setup(p_cfg->att_cfg, p_cfg->gap_start_hdl, p_cfg->gatt_start_hdl, central_res_en);
            if(status != GAP_ERR_NO_ERROR) break;

            #if(BLE_GATT_CLI)
            // Initiate GAP and GATT client
            status = gapc_cli_setup(p_cfg->att_cfg);
            if(status != GAP_ERR_NO_ERROR) break;
            #endif // (BLE_GATT_CLI)
        }
        #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)

        // set role
        gapm_env.role = targeted_role;

        #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
        // security and MTU configuration only needed if device is peripheral or central
        if((gapm_env.role & ~(GAP_ROLE_BT_CLASSIC | GAP_ROLE_OBSERVER | GAP_ROLE_BROADCASTER)) != 0)
        {
            // Store pairing mode
            gapm_env.pairing_mode = p_cfg->pairing_mode;
        }
        else
        #endif //(HL_LE_CENTRAL || HL_LE_PERIPHERAL)
        {
            gapm_env.pairing_mode = 0;
        }

        // Set device Identity key (IRK)
        memcpy(&(gapm_env.irk), &(p_cfg->irk), sizeof(struct gap_sec_key));

        if (p_cfg->privacy_cfg & GAPM_PRIV_CFG_PRIV_ADDR_BIT)
        {
            // Copy static address received from application in local memory
            memcpy(&(gapm_env.addr), &(p_cfg->addr), GAP_BD_ADDR_LEN);
        }

        // Create new procedure
        status = gapm_proc_set_cfg_create(sizeof(gapm_cfg_proc_setup_t),
                                          (do_reset ? &gapm_cfg_proc_setup_reset_itf : &gapm_cfg_proc_setup_config_itf),
                                          dummy, cmp_cb, (gapm_proc_set_cfg_t**) &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        // Initialize configuration structure
        p_proc->do_reset     = do_reset;
        p_proc->do_configure = true;
        p_proc->cfg          = *p_cfg;
        p_proc->cfg.role     = targeted_role;
    } while (0);

    return status;
}

uint16_t gapm_set_name(uint32_t dummy, uint8_t name_len, const uint8_t* p_name, gapm_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    do
    {
        gapm_cfg_proc_set_name_t* p_proc = NULL;

        if(!gapm_env.configured) break;

        if((p_name == NULL) || (name_len == 0) || (name_len > BD_NAME_SIZE))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Create new procedure
        status = gapm_proc_set_cfg_create(sizeof(gapm_cfg_proc_set_name_t), &gapm_cfg_proc_set_name_itf, dummy, cmp_cb,
                                          (gapm_proc_set_cfg_t**) &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        // Initialize procedure parameters
        p_proc->name_len = name_len;
        p_proc->p_name   = p_name;
    } while (0);

    return status;
}

const uint8_t* gapm_get_name(uint8_t* p_name_len)
{
    *p_name_len = gapm_env.name_len;
    return (gapm_env.p_name);
}

uint16_t gapm_set_le_chmap(uint32_t dummy, const le_chnl_map_t* p_chmap, gapm_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_CENTRAL || HL_LE_BROADCASTER)
    do
    {
        gapm_cfg_proc_set_chmap_t* p_proc = NULL;

        if(!gapm_env.configured)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // this is only for central or broadcaster role
        if (!GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_CENTRAL) && !GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_BROADCASTER)) break;

        if(p_chmap == NULL)
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Create new procedure
        status = gapm_proc_send_hci_cmd_create(sizeof(gapm_cfg_proc_set_chmap_t),
                                               &gapm_cfg_send_set_chmap_hci_cmd_info,
                                               dummy, cmp_cb, (gapm_proc_send_hci_cmd_t**) &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        // Initialize procedure parameters
        p_proc->chmap  = *p_chmap;
    } while (0);
    #endif //(HL_LE_CENTRAL || HL_LE_BROADCASTER)

    return status;
}

uint16_t gapm_start_channel_assessment(uint32_t dummy, uint32_t scan_win_duration, const uint32_t scan_duration_min,
                                       const uint32_t scan_duration_max, const uint16_t intv, gapm_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_OBSERVER || HL_LE_BROADCASTER)
    do
    {
        gapm_cfg_proc_start_ch_assess_t* p_proc = NULL;

        if(!gapm_env.configured)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Create new procedure
        status = gapm_proc_send_hci_cmd_create(sizeof(gapm_cfg_proc_start_ch_assess_t), &gapm_cfg_send_ch_scan_hci_cmd_info,
                                               dummy, cmp_cb, (gapm_proc_send_hci_cmd_t**) &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        // Initialize procedure parameters
        p_proc->scan_win_duration = scan_win_duration;
        p_proc->scan_duration_max = scan_duration_max;
        p_proc->scan_duration_min = scan_duration_min;
        p_proc->intv  = intv;
    } while (0);
    #endif //(HL_LE_OBSERVER || HL_LE_BROADCASTER)

    return status;
}

uint16_t gapm_stop_channel_assessment(uint32_t dummy, gapm_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HL_LE_OBSERVER || HL_LE_BROADCASTER)
    do
    {
        gapm_cfg_proc_stop_ch_assess_t* p_proc = NULL;

        if(!gapm_env.configured)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Create new procedure
        status = gapm_proc_send_hci_cmd_create(sizeof(gapm_cfg_proc_stop_ch_assess_t), &gapm_cfg_send_ch_scan_end_hci_cmd_info,
                                               dummy, cmp_cb, (gapm_proc_send_hci_cmd_t**) &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

    } while (0);
    #endif //(HL_LE_OBSERVER || HL_LE_BROADCASTER)

    return status;
}

uint16_t gapm_set_irk(const gap_sec_key_t* p_irk)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    if(!gapm_env.configured)
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else if(p_irk == NULL)
    {
         status = GAP_ERR_INVALID_PARAM;
    }
    else
    {
        gapm_env.irk = *p_irk;
    }

    return (status);
}

#if(BLE_HOST_PRESENT)
uint16_t gapm_dbg_iqgen_cfg(uint32_t dummy, uint8_t mode, uint8_t nb_antenna, const gapm_dbg_iq_ctrl_t* p_iq_ctrl,
                            gapm_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;
    #if (BLE_HOST_IQ_GEN)
    do
    {
        uint8_t iq_ctrl_size = sizeof(gapm_dbg_iq_ctrl_t) * nb_antenna;
        gapm_cfg_proc_dbg_iqgen_t* p_proc = NULL;

        if(!gapm_env.configured) break;

        if((p_iq_ctrl == NULL) || (nb_antenna > DBG_IQGEN_MAX_ANTENNA))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Create new procedure
        status = gapm_proc_send_hci_cmd_create((sizeof(gapm_cfg_proc_dbg_iqgen_t) + iq_ctrl_size),
                                               &gapm_cfg_send_dbg_iqgen_cfg_hci_cmd_info,
                                               dummy, cmp_cb, (gapm_proc_send_hci_cmd_t**) &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        // Initialize procedure parameters
        p_proc->mode          = mode;
        p_proc->nb_antenna    = nb_antenna;
        memcpy(p_proc->iq_ctrl, p_iq_ctrl, iq_ctrl_size);
    } while (0);
    #endif // (BLE_HOST_IQ_GEN)
    return (status);
}
#endif // (BLE_HOST_PRESENT)

uint8_t gapm_get_suppported_roles(void)
{
    return gapm_env.role;
}

/// @} GAPM
