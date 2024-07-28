/**
 ****************************************************************************************
 *
 * @file gapc_ping.c
 *
 * @brief GAPC Generic Access Profile Connection - BT Ping / LE Ping
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

/// LE Ping timeout procedure object
typedef struct gapc_ping_proc
{
    /// procedure inheritance
    gapc_proc_simple_t hdr;
    /// Authenticated payload timeout (N*10ms)
    uint16_t           timeout;
} gapc_ping_proc_t;


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
__STATIC uint16_t gapc_ping_proc_granted(uint8_t conidx, gapc_ping_proc_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    struct hci_wr_auth_payl_to_cmd *p_hci_cmd = HL_HCI_CMD_ALLOC(HCI_WR_AUTH_PAYL_TO_CMD_OPCODE, hci_wr_auth_payl_to_cmd);

    if(p_hci_cmd != NULL)
    {
        p_hci_cmd->conhdl       = gapc_get_conhdl(conidx);
        p_hci_cmd->auth_payl_to = p_proc->timeout;

        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_FINISHED),
                                gapc_proc_default_hci_cmp_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_ping_proc_itf =
{
        .granted  = (gapc_proc_simple_granted_cb) gapc_ping_proc_granted,
        .finished = gapc_proc_simple_default_finished_cb,
};


// ------------------ GET AUTH PAYLOAD TO -------------------


/// Get RSSI  procedure is granted
__STATIC uint16_t gapc_get_auth_payload_to_proc_granted(uint8_t conidx, gapc_proc_info_t* p_proc)
{
    uint16_t status = HL_HCI_BASIC_CMD_SEND_WITH_CONHDL(HCI_RD_AUTH_PAYL_TO_CMD_OPCODE, gapc_get_conhdl(conidx),
                                                        HL_PROC_FINISHED, gapc_proc_info_default_hci_cmp_evt_handler)
    return (status);
}

/// Get RSSI procedure is finished
__STATIC void gapc_get_auth_payload_to_proc_finished(uint8_t conidx, gapc_proc_info_t* p_proc, uint16_t status)
{
    gapc_get_auth_payload_to_cmp_cb cmp_cb = (gapc_get_auth_payload_to_cmp_cb) p_proc->hdr.cmp_cb;
    uint16_t timeout = 0;

    if(status == GAP_ERR_NO_ERROR)
    {
        // procedure finished is called within hci complete event handler
        const struct hci_rd_auth_payl_to_cmd_cmp_evt* p_evt = (const struct hci_rd_auth_payl_to_cmd_cmp_evt*) p_proc->p_res_info;
        timeout = p_evt->auth_payl_to;
    }

    cmp_cb(conidx, p_proc->hdr.dummy, status, timeout);
}


/// Get RSSI procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_get_auth_payload_to_proc_itf =
{
    .granted  = (gapc_proc_simple_granted_cb)  gapc_get_auth_payload_to_proc_granted,
    .finished = (gapc_proc_simple_finished_cb) gapc_get_auth_payload_to_proc_finished,
};


/*
 * HCI HANDLERS FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
/// @brief LE ping authenticated payload timeout expires
void gapc_hci_auth_payl_to_exp_evt_handler(uint8_t evt_code, struct hci_auth_payl_to_exp_evt const *p_evt)
{
    uint8_t conidx = gapc_get_conidx(p_evt->conhdl);
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    if(p_con != NULL)
    {
        const gapc_connection_info_cb_t* p_cbs = gapc_env.p_info_cbs;
        if(p_cbs->auth_payload_timeout != NULL)
        {
            p_cbs->auth_payload_timeout(conidx, p_con->dummy);
        }
    }
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapc_set_auth_payload_timeout(uint8_t conidx, uint32_t dummy, uint16_t timeout, gapc_proc_cmp_cb cmp_cb)
{
    uint16_t status;
    gapc_ping_proc_t* p_proc;

    status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_ping_proc_t), &gapc_ping_proc_itf,
                                     (gapc_proc_simple_t**) &p_proc);
    if(status == GAP_ERR_NO_ERROR)
    {
        p_proc->timeout  = timeout;
    }

    return (status);
}

uint16_t gapc_get_auth_payload_to(uint8_t conidx, uint32_t dummy, gapc_get_auth_payload_to_cmp_cb cmp_cb)
{
    return (gapc_proc_info_create(conidx, dummy, 0, &gapc_get_auth_payload_to_proc_itf, (gapc_proc_cmp_cb) cmp_cb));
}

#endif // (GAPC_PRESENT)
/// @} GAPC
