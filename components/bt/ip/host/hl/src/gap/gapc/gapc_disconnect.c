/**
 ****************************************************************************************
 *
 * @file gapc_disconnect.c
 *
 * @brief Generic Access Profile Controller - Disconnect procedure handling
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#if (GAPC_PRESENT)
#include "gapc_int.h"
#include "hl_hci.h"
#include "../gap_int.h"

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

/// Disconnection procedure
typedef struct gapc_disconnect_proc
{
    /// Inherits Simple procedure
    gapc_proc_simple_t hdr;
    /// True if procedure is aborted, False otherwise
    bool               is_aborted;
} gapc_disconnect_proc_t;

/// Disconnected callback
typedef void (*gapc_disconnected_cb)(uint8_t conidx, uint32_t dummy, uint16_t reason);

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/// Retrieve disconnect procedure (at end of procedure queue)
__STATIC gapc_disconnect_proc_t* gapc_disconnect_proc_get(gapc_con_t* p_con)
{
    co_list_t* p_proc_queue = &(p_con->proc_queue.queue);
    // disconnection procedure is tail of link update procedure queue
    co_list_hdr_t* p_tail_proc = co_list_tail(p_proc_queue);
    return ((gapc_disconnect_proc_t*) p_tail_proc);
}

/*
 * PROCEDURE STATE MACHINE INTERFACE
 ****************************************************************************************
 */

/// Disconnect procedure granted callback
__STATIC uint16_t gapm_disconnect_granted(uint8_t conidx, gapc_disconnect_proc_t* p_proc)
{
    return (p_proc->is_aborted ? GAP_ERR_CANCELED : GAP_ERR_NO_ERROR);
}

/// Disconnect procedure finished callback
__STATIC void gapm_disconnect_finished(uint8_t conidx, gapc_disconnect_proc_t* p_proc, uint16_t status)
{
    if(!p_proc->is_aborted)
    {
        p_proc->hdr.cmp_cb(conidx, p_proc->hdr.dummy, status);
    }

    p_proc->is_aborted = true; // ensure that callback will not be executed twice
}

/// Disconnect Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapm_disconnect_proc_itf =
{
    .granted =  (gapc_proc_simple_granted_cb) gapm_disconnect_granted,
    .finished =  (gapc_proc_simple_finished_cb) gapm_disconnect_finished,
};

/*
 * HCI HANDLERS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
///  Handles common status event for connection purpose.
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conidx    Connection index (since connection handle already retrieved
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapc_disconnect_hci_disconnect_cmd_stat_event_handler(uint16_t opcode, uint16_t conidx,
                                                                struct hci_cmd_stat_event const *p_evt)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    uint16_t status = RW_ERR_HCI_TO_HL(p_evt->status);
    // send error event only if it's a requested disconnect
    if ((status != GAP_ERR_NO_ERROR) && (p_con != NULL) && (GETB(p_con->info_bf, GAPC_DISCONNECTING)))
    {
        SETB(p_con->info_bf, GAPC_DISCONNECTING, false); // no more on disconnected state

        if(!gapc_proc_simple_transition_if_active(conidx, &gapm_disconnect_proc_itf, HL_PROC_FINISHED, status))
        {
            gapm_disconnect_finished(conidx, gapc_disconnect_proc_get(p_con), status);
        }
    }
}

/// Handle HCI disconnection complete event
__STATIC void gapc_disconnect_hci_con_disc_cmp_evt_handler(uint8_t conidx, uint16_t status, uint16_t reason)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    if(p_con) // sanity check
    {
        if(!GETB(p_con->info_bf, GAPC_IS_NAME_DISCOVERY))
        {
            // inform connection owner about disconnection
            gapc_env.p_info_cbs->disconnected(conidx, p_con->dummy, reason);
        }

        // Disconnection initiated locally
        if (GETB(p_con->info_bf, GAPC_DISCONNECTING))
        {
            gapm_disconnect_finished(conidx, gapc_disconnect_proc_get(p_con), status);
        }

        // cleanup Allocated connection resources
        gapm_con_cleanup(conidx, reason);
    }
}

#if(BT_HOST_PRESENT)
/// Handles Disconnect command completed event received from BK bridge
void gapc_bt_con_hci_con_disc_cmp_evt_handler(uint8_t evt_code, uint8_t status, uint16_t conhdl, uint8_t reason)
{
    gapc_disconnect_hci_con_disc_cmp_evt_handler(gapc_get_conidx(conhdl), RW_ERR_HCI_TO_HL(status), RW_ERR_HCI_TO_HL(reason));
}
#endif // (BT_HOST_PRESENT)

#if (BLE_GAPC)
/// @brief Handles Disconnect command completed event
void gapc_le_con_hci_con_disc_cmp_evt_handler(uint8_t evt_code, uint8_t conidx, struct hci_disc_cmp_evt const *p_evt)
{
    gapc_disconnect_hci_con_disc_cmp_evt_handler(conidx, RW_ERR_HCI_TO_HL(p_evt->status), RW_ERR_HCI_TO_HL(p_evt->reason));
}
#endif // (BLE_GAPC)

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


uint16_t gapc_disconnect(uint8_t conidx, uint32_t dummy, uint16_t reason, gapc_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_con_t* p_con = gapc_get_con_env(conidx);

    if(p_con && !GETB(p_con->info_bf, GAPC_DISCONNECTING))
    {
        struct hci_disconnect_cmd *p_cmd;

        p_cmd = HL_HCI_CMD_ALLOC(HCI_DISCONNECT_CMD_OPCODE, hci_disconnect_cmd);
        if(p_cmd)
        {
            gapc_disconnect_proc_t* p_proc;
            p_cmd->conhdl = gapc_get_conhdl(conidx);
            p_cmd->reason = RW_ERR_HL_TO_HCI(reason);
            // send disconnect command immediately (consider it high prio) and ignore if procedure creation succeed
            HL_HCI_CMD_SEND_TO_CTRL(p_cmd, conidx, gapc_disconnect_hci_disconnect_cmd_stat_event_handler);

            // create disconnection procedure to keep information about requester
            status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_disconnect_proc_t),
                                             &gapm_disconnect_proc_itf, (gapc_proc_simple_t**) &p_proc);
            if(status == GAP_ERR_NO_ERROR)
            {
                SETB(p_con->info_bf, GAPC_DISCONNECTING, true);
                p_proc->is_aborted = false;
            }
        }
    }

    return (status);
}

#endif // (GAPC_PRESENT)
/// @} GAPC
