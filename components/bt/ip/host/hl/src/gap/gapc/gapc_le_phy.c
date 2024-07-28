/**
 ****************************************************************************************
 *
 * @file gapc_le_phy.c
 *
 * @brief GAPC Generic Access Profile Connection - PHY Update / Get information
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPC Generic Access Profile Connection - PHY Update
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

/// Connection update procedure object
typedef struct gapc_le_phy_up_proc
{
    /// procedure inheritance
    gapc_proc_simple_t hdr;
    /// Supported LE PHY for data transmission (see enum #gap_phy)
    uint8_t            tx_phy;
    /// Supported LE PHY for data reception (see enum #gap_phy)
    uint8_t            rx_phy;
    /// PHY options (see enum #gapc_phy_option)
    uint8_t            phy_opt;
} gapc_le_phy_up_proc_t;

/*
 * LOCAL FUNCTIONS DECLARATION
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
/// PHY update procedure state machine granted function
__STATIC uint16_t gapc_le_phy_up_proc_granted(uint8_t conidx, gapc_le_phy_up_proc_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    // send HCI command to set PHY
    struct hci_le_set_phy_cmd *p_hci_cmd = HL_HCI_CMD_ALLOC(HCI_LE_SET_PHY_CMD_OPCODE, hci_le_set_phy_cmd);
    if(p_hci_cmd != NULL)
    {
        p_hci_cmd->conhdl    = gapc_get_conhdl(conidx);
        p_hci_cmd->all_phys  = (p_proc->tx_phy == GAP_PHY_ANY) ? ALL_PHYS_TX_NO_PREF : 0;
        p_hci_cmd->all_phys |= (p_proc->rx_phy == GAP_PHY_ANY) ? ALL_PHYS_RX_NO_PREF : 0;
        p_hci_cmd->rx_phys   = p_proc->rx_phy;
        p_hci_cmd->tx_phys   = p_proc->tx_phy;
        p_hci_cmd->phy_opt   = (uint16_t)p_proc->phy_opt;

        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_CONTINUE),
                gapc_proc_default_hci_stat_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_le_phy_up_proc_itf =
{
        .granted  = (gapc_proc_simple_granted_cb) gapc_le_phy_up_proc_granted,
        .finished = gapc_proc_simple_default_finished_cb,
};


/*
 * HCI HANDLERS FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/// @brief LE PHY updated complete event
void gapc_hci_le_phy_upd_cmp_evt_handler(uint8_t evt_code, struct hci_le_phy_upd_cmp_evt const *p_evt)
{
    // Current connection index
    uint8_t conidx = gapc_get_conidx(p_evt->conhdl);
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

    if(p_con != NULL)
    {
        uint16_t status = RW_ERR_HCI_TO_HL(p_evt->status);

        if(status == GAP_ERR_NO_ERROR)
        {
            const gapc_le_config_cb_t* p_cbs = gapc_env.p_le_config_cbs;
            if(p_cbs->phy_updated != NULL)
            {
                p_cbs->phy_updated(conidx, p_con->hdr.dummy, p_evt->tx_phy, p_evt->rx_phy);
            }
        }

        // Handle procedure transition if procedure is active
        gapc_proc_simple_transition_if_active(conidx, &gapc_le_phy_up_proc_itf, HL_PROC_FINISHED, status);
    }
}


// ------------------ GET PHY -------------------

/// Get PHY info procedure is granted
__STATIC uint16_t gapc_get_le_phy_proc_granted(uint8_t conidx, gapc_proc_info_t* p_proc)
{
    uint16_t status = HL_HCI_BASIC_CMD_SEND_WITH_CONHDL(HCI_LE_RD_PHY_CMD_OPCODE, gapc_get_conhdl(conidx),
                                                        HL_PROC_FINISHED, gapc_proc_info_default_hci_cmp_evt_handler)
    return (status);
}

/// Get PHY info procedure is finished
__STATIC void gapc_get_le_phy_proc_finished(uint8_t conidx, gapc_proc_info_t* p_proc, uint16_t status)
{
    gapc_get_le_phy_cmp_cb cmp_cb = (gapc_get_le_phy_cmp_cb) p_proc->hdr.cmp_cb;
    uint16_t tx_phy = 0;
    uint16_t rx_phy = 0;
    if(status == GAP_ERR_NO_ERROR)
    {
        // procedure finished is called within hci complete event handler
        const struct hci_le_rd_phy_cmd_cmp_evt* p_evt = (const struct hci_le_rd_phy_cmd_cmp_evt*) p_proc->p_res_info;
        tx_phy = p_evt->tx_phy;
        rx_phy = p_evt->rx_phy;
    }

    cmp_cb(conidx, p_proc->hdr.dummy, status, tx_phy, rx_phy);
}


/// Get PHY info procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_get_le_phy_proc_itf =
{
    .granted  = (gapc_proc_simple_granted_cb)  gapc_get_le_phy_proc_granted,
    .finished = (gapc_proc_simple_finished_cb) gapc_get_le_phy_proc_finished,
};


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapc_le_set_phy(uint8_t conidx, uint32_t dummy, uint8_t tx_phy, uint8_t rx_phy, uint8_t phy_opt,
                         gapc_proc_cmp_cb cmp_cb)
{
    uint16_t status;
    gapc_le_phy_up_proc_t* p_proc;

    status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_le_phy_up_proc_t),
                                     &gapc_le_phy_up_proc_itf, (gapc_proc_simple_t**) &p_proc);
    if(status == GAP_ERR_NO_ERROR)
    {
        p_proc->tx_phy  = tx_phy;
        p_proc->rx_phy  = rx_phy;
        p_proc->phy_opt = phy_opt;
    }

    return (status);
}


uint16_t gapc_get_le_phy(uint8_t conidx, uint32_t dummy, gapc_get_le_phy_cmp_cb cmp_cb)
{
    return (gapc_proc_info_create(conidx, dummy, 0,  &gapc_get_le_phy_proc_itf, (gapc_proc_cmp_cb) cmp_cb));
}


#endif // (BLE_GAPC)
/// @} GAPC
