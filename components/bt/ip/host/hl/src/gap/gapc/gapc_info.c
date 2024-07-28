/**
 ****************************************************************************************
 *
 * @file gapc_info.c
 *
 * @brief GAPC Generic Access Profile Connection - Connection information
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPC Generic Access Profile Connection - Connection information
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

#include "co_endian.h"
#include "co_utils.h"
#include "co_math.h"
#include <string.h>

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

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/*
 * HCI HANDLERS FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/// @brief Handles Read version command completed event
void gapc_hci_rd_rem_ver_info_cmp_evt_handler(uint8_t evt_code, struct hci_rd_rem_ver_info_cmp_evt const *p_evt)
{
    gapc_proc_info_default_hci_cmp_evt_handler(evt_code, HL_PROC_FINISHED, (const struct hci_basic_conhdl_cmd_cmp_evt*) p_evt);
}

/*
 * PROCEDURE STATE MACHINES
 ****************************************************************************************
 */

// ------------------ PEER VERSION -------------------
/// Peer version procedure is granted
__STATIC uint16_t gapc_get_peer_version_proc_granted(uint8_t conidx, gapc_proc_info_t* p_proc)
{
    uint16_t status = HL_HCI_BASIC_CMD_SEND_WITH_CONHDL(HCI_RD_REM_VER_INFO_CMD_OPCODE, gapc_get_conhdl(conidx),
                                                        HL_PROC_CONTINUE, gapc_proc_default_hci_stat_evt_handler)
    return (status);
}

/// Peer version procedure is finished
__STATIC void gapc_get_peer_version_proc_finished(uint8_t conidx, gapc_proc_info_t* p_proc, uint16_t status)
{
    gapc_get_peer_version_cmp_cb cmp_cb = (gapc_get_peer_version_cmp_cb) p_proc->hdr.cmp_cb;
    gapc_version_t* p_version = NULL;
    gapc_version_t version;

    if(status == GAP_ERR_NO_ERROR)
    {
        // procedure finished is called within hci complete event handler
        const struct hci_rd_rem_ver_info_cmp_evt* p_evt = (const struct hci_rd_rem_ver_info_cmp_evt*) p_proc->p_res_info;
        version.company_id     = p_evt->compid;
        version.lmp_version    = p_evt->vers;
        version.lmp_subversion = p_evt->subvers;
        p_version = &version;
    }

    cmp_cb(conidx, p_proc->hdr.dummy, status, p_version);
}


/// Peer version simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_get_peer_version_proc_itf =
{
    .granted  = (gapc_proc_simple_granted_cb)  gapc_get_peer_version_proc_granted,
    .finished = (gapc_proc_simple_finished_cb) gapc_get_peer_version_proc_finished,
};


// ------------------ GET RSSI -------------------


/// Get RSSI  procedure is granted
__STATIC uint16_t gapc_get_rssi_proc_granted(uint8_t conidx, gapc_proc_info_t* p_proc)
{
    uint16_t status = HL_HCI_BASIC_CMD_SEND_WITH_CONHDL(HCI_RD_RSSI_CMD_OPCODE, gapc_get_conhdl(conidx),
                                                        HL_PROC_FINISHED, gapc_proc_info_default_hci_cmp_evt_handler)
    return (status);
}

/// Get RSSI procedure is finished
__STATIC void gapc_get_rssi_proc_finished(uint8_t conidx, gapc_proc_info_t* p_proc, uint16_t status)
{
    gapc_get_rssi_cmp_cb cmp_cb = (gapc_get_rssi_cmp_cb) p_proc->hdr.cmp_cb;
    int8_t rssi = 0;

    if(status == GAP_ERR_NO_ERROR)
    {
        // procedure finished is called within hci complete event handler
        const struct hci_rd_rssi_cmd_cmp_evt* p_evt = (const struct hci_rd_rssi_cmd_cmp_evt*) p_proc->p_res_info;
        rssi     = p_evt->rssi;
    }

    cmp_cb(conidx, p_proc->hdr.dummy, status, rssi);
}


/// Get RSSI procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_get_rssi_proc_itf =
{
    .granted  = (gapc_proc_simple_granted_cb)  gapc_get_rssi_proc_granted,
    .finished = (gapc_proc_simple_finished_cb) gapc_get_rssi_proc_finished,
};

/*
 * EXTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapc_get_peer_version(uint8_t conidx, uint32_t dummy, gapc_get_peer_version_cmp_cb cmp_cb)
{
    return (gapc_proc_info_create(conidx, dummy, 0, &gapc_get_peer_version_proc_itf, (gapc_proc_cmp_cb) cmp_cb));
}

uint16_t gapc_get_rssi(uint8_t conidx, uint32_t dummy, gapc_get_rssi_cmp_cb cmp_cb)
{
    return (gapc_proc_info_create(conidx, dummy, 0, &gapc_get_rssi_proc_itf, (gapc_proc_cmp_cb) cmp_cb));
}

#endif // (GAPC_PRESENT)
/// @} GAPC
