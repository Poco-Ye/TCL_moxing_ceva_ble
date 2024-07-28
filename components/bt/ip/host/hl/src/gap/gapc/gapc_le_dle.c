/**
 ****************************************************************************************
 *
 * @file gapc_dle.c
 *
 * @brief GAPC Generic Access Profile Connection - Data Length Extension
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPC Generic Access Profile Connection - Data Length Extension
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

/// Data Length procedure object
typedef struct gapc_le_dle_proc
{
    /// procedure inheritance
    gapc_proc_simple_t hdr;
    ///Preferred maximum number of payload octets that the local Controller should include
    ///in a single Link Layer Data Channel PDU.
    uint16_t           tx_octets;
    ///Preferred maximum number of microseconds that the local Controller should use to transmit
    ///a single Link Layer Data Channel PDU
    uint16_t           tx_time;
} gapc_le_dle_proc_t;

/// Set preferred rx size and time
typedef struct gapc_le_dle_proc_set_max_rx_size_and_time
{
    /// Inherited from simple GAPC procedure
    gapc_proc_simple_t hdr;
    /// Maximum RX size (in Bytes)
    uint16_t           rx_octets;
    /// Maximum RX time (in us)
    uint16_t           rx_time;
} gapc_le_dle_proc_set_max_rx_size_and_time_t;

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
/// Procedure state machine granted function
__STATIC uint16_t gapc_le_dle_proc_granted(uint8_t conidx, gapc_le_dle_proc_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    // send HCI command
    struct hci_le_set_data_len_cmd *p_hci_cmd = HL_HCI_CMD_ALLOC(HCI_LE_SET_DATA_LEN_CMD_OPCODE, hci_le_set_data_len_cmd);

    if(p_hci_cmd != NULL)
    {
        p_hci_cmd->conhdl    = gapc_get_conhdl(conidx);
        p_hci_cmd->tx_octets = p_proc->tx_octets;
        p_hci_cmd->tx_time   = p_proc->tx_time;

        /* send the message */
        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_FINISHED),
                                gapc_proc_default_hci_cmp_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_le_dle_proc_itf =
{
        .granted  = (gapc_proc_simple_granted_cb) gapc_le_dle_proc_granted,
        .finished = gapc_proc_simple_default_finished_cb,
};


/// @brief Send the HCI_VS_SET_PREF_SLAVE_LATENCY_CMD command
__STATIC uint16_t gapc_le_dle_proc_set_max_rx_size_and_time_granted(uint8_t conidx,
                                                               gapc_le_dle_proc_set_max_rx_size_and_time_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    struct hci_vs_set_max_rx_size_and_time_cmd *p_hci_cmd =
            HL_HCI_CMD_ALLOC(HCI_VS_SET_MAX_RX_SIZE_AND_TIME_CMD_OPCODE, hci_vs_set_max_rx_size_and_time_cmd);
    if(p_hci_cmd != NULL)
    {
        p_hci_cmd->conhdl    = gapc_get_conhdl(conidx);
        p_hci_cmd->rx_octets = p_proc->rx_octets;
        p_hci_cmd->rx_time   = p_proc->rx_time;
        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_FINISHED), gapc_proc_default_hci_cmp_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }
    return (status);
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_le_dle_proc_set_max_rx_size_and_time_itf =
{
        .granted  = (gapc_proc_simple_granted_cb) gapc_le_dle_proc_set_max_rx_size_and_time_granted,
        .finished = gapc_proc_simple_default_finished_cb,
};



/*
 * HCI HANDLERS
 ****************************************************************************************
 */

/// @brief Set Data Length extension event.
void gapc_hci_le_data_len_chg_evt_handler(uint8_t evt_code, struct hci_le_data_len_chg_evt const *p_evt)
{
    uint8_t conidx = gapc_get_conidx(p_evt->conhdl);
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

    if (p_con != NULL)
    {
        const gapc_le_config_cb_t* p_cbs = gapc_env.p_le_config_cbs;
        if(p_cbs->packet_size_updated != NULL)
        {
            p_cbs->packet_size_updated(conidx, p_con->hdr.dummy, p_evt->max_tx_octets, p_evt->max_tx_time,
                                       p_evt->max_rx_octets, p_evt->max_rx_time);
        }
    }
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapc_le_set_pkt_size(uint8_t conidx, uint32_t dummy, uint16_t tx_octets, uint16_t tx_time,
                              gapc_proc_cmp_cb cmp_cb)
{
    uint16_t status;
    gapc_le_dle_proc_t* p_proc;

    status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_le_dle_proc_t), &gapc_le_dle_proc_itf,
                                     (gapc_proc_simple_t**) &p_proc);
    if(status == GAP_ERR_NO_ERROR)
    {
        p_proc->tx_octets  = tx_octets;
        p_proc->tx_time    = tx_time;
    }

    return (status);
}


uint16_t gapc_le_set_max_rx_size_and_time(uint8_t conidx, uint32_t dummy, uint16_t rx_octets, uint16_t rx_time,
                                       gapc_proc_cmp_cb cmp_cb)
{
    gapc_le_dle_proc_set_max_rx_size_and_time_t* p_proc;
    uint16_t status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_le_dle_proc_set_max_rx_size_and_time_t),
                                &gapc_le_dle_proc_set_max_rx_size_and_time_itf, (gapc_proc_simple_t**) &p_proc);
    if(status == GAP_ERR_NO_ERROR)
    {
        p_proc->rx_octets = rx_octets;
        p_proc->rx_time   = rx_time;
    }

    return (status);
}


#endif // (BLE_GAPC)
/// @} GAPC
