/**
 ****************************************************************************************
 *
 * @file gapc_proc.c
 *
 * @brief GAPC Generic Access Profile Connection - Procedure manager
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPC Generic Access Profile Connection - Procedure manager
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (GAPC_PRESENT)

#include "arch.h"
#include "gapc_int.h"

#include "gap.h"
#include "gapm.h"
#include "gapc.h"
#if (BLE_GAPC)
#include "gapc_le_con.h"
#endif // (BLE_GAPC)
#include "hci.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DECLARATIONS
 ****************************************************************************************
 */

__STATIC bool gapc_proc_simple_state_transition(gapc_proc_simple_t* p_proc, uint8_t event, uint16_t status);

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/// Simple procedure interface
__STATIC const hl_proc_itf_t gapc_proc_simple_itf =
{
    .transition  = (hl_proc_transition_cb)  gapc_proc_simple_state_transition,
    .cleanup     = hl_proc_cleanup,
};

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/// Get if current procedure is active
__STATIC bool gapc_proc_simple_is_active(uint8_t conidx, const gapc_proc_simple_itf_t* p_itf)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    gapc_proc_simple_t* p_proc = (gapc_proc_simple_t*) gapc_proc_get(conidx);
    return (   (p_proc != NULL) && (p_proc->hdr.p_itf == &gapc_proc_simple_itf) && (p_proc->p_itf == p_itf)
            && !hl_proc_is_waiting_grant(&(p_con->proc_queue)));
}


/*
 * PROCEDURE STATE MACHINE
 ****************************************************************************************
 */

/// Simple procedure state machine transition function
__STATIC bool gapc_proc_simple_state_transition(gapc_proc_simple_t* p_proc, uint8_t event, uint16_t status)
{
    bool is_finished = true;

    if((status == GAP_ERR_NO_ERROR) && (event == HL_PROC_GRANTED))
    {
        // execute granted function --> once
        status = p_proc->p_itf->granted(p_proc->conidx, p_proc);
    }

    is_finished = ((status != GAP_ERR_NO_ERROR) || (event == HL_PROC_FINISHED));

    if(is_finished)
    {
        // inform that procedure is over
        p_proc->p_itf->finished(p_proc->conidx, p_proc, status);
    }

    return (is_finished);
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapc_proc_create(gapc_con_t* p_con, uint16_t proc_size, const hl_proc_itf_t *p_itf,
                          hl_proc_t** pp_proc)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    // Do not accept command creation if disconnection on-going
    if(!GETB(p_con->info_bf, GAPC_DISCONNECTING))
    {
        status = hl_proc_create(&(p_con->proc_queue), proc_size, p_itf, pp_proc);
    }
    return (status);
}

bool gapc_proc_is_active(uint8_t conidx, const hl_proc_itf_t *p_itf)
{
    hl_proc_t* p_proc = gapc_proc_get(conidx);
    return (p_proc != NULL) && (p_proc->p_itf == p_itf);
}

void gapc_proc_transition(uint8_t conidx, uint8_t event, uint16_t status)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    hl_proc_transition(&(p_con->proc_queue), event, status);
}

bool gapc_proc_transition_if_active(uint8_t conidx, const hl_proc_itf_t *p_itf, uint8_t event, uint16_t status)
{
    bool is_active = gapc_proc_is_active(conidx, p_itf);
    if(is_active)
    {
        gapc_proc_transition(conidx, event, status);
    }
    return (is_active);
}

hl_proc_t* gapc_proc_get(uint8_t conidx)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    return ((p_con != NULL) ? hl_proc_get(&(p_con->proc_queue)) : NULL);
}

void gapc_proc_reset(uint8_t conidx)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    hl_proc_queue_initialize(&(p_con->proc_queue));
}

void gapc_proc_cleanup(uint8_t conidx, uint16_t reason)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    hl_proc_queue_abort(&(p_con->proc_queue), reason);
}

void gapc_proc_default_hci_stat_evt_handler(uint16_t opcode, uint16_t token, struct hci_cmd_stat_event const *p_evt)
{
    uint16_t status = RW_ERR_HCI_TO_HL(p_evt->status);
    gapc_proc_transition(GAPC_PROC_TOKEN_GET_CONIDX(token), GAPC_PROC_TOKEN_GET_EVENT(token), status);

}

void gapc_proc_default_hci_cmp_evt_handler(uint16_t opcode, uint16_t token,  struct hci_basic_conhdl_cmd_cmp_evt const *p_evt)
{
    uint16_t status = RW_ERR_HCI_TO_HL(p_evt->status);
    gapc_proc_transition(GAPC_PROC_TOKEN_GET_CONIDX(token), GAPC_PROC_TOKEN_GET_EVENT(token), status);
}

void gapc_proc_ignore_hci_evt_handler(uint16_t opcode, uint16_t event, const void *p_evt)
{
    // do nothing
}

uint16_t gapc_proc_simple_create(uint8_t conidx, uint32_t dummy, gapc_proc_cmp_cb cmp_cb, uint16_t proc_size,
                                 const gapc_proc_simple_itf_t* p_itf, gapc_proc_simple_t** pp_proc)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_con_t* p_con = gapc_get_con_env(conidx);

    if(cmp_cb == NULL)
    {
        status = GAP_ERR_MISSING_CALLBACK;
    }
    else if(p_con != NULL)
    {
        status = gapc_proc_create(p_con, proc_size, &gapc_proc_simple_itf, (hl_proc_t**) pp_proc);
        if(status == GAP_ERR_NO_ERROR)
        {
            (*pp_proc)->conidx = conidx;
            (*pp_proc)->cmp_cb = cmp_cb;
            (*pp_proc)->dummy  = dummy;
            (*pp_proc)->p_itf  = p_itf;
        }
    }

    return (status);
}

bool gapc_proc_simple_transition_if_active(uint8_t conidx, const gapc_proc_simple_itf_t* p_itf, uint8_t event,
                                           uint16_t status)
{
    bool is_active = gapc_proc_simple_is_active(conidx, p_itf);
    if(is_active)
    {
        gapc_proc_transition(conidx, event, status);
    }
    return (is_active);
}

void gapc_proc_simple_default_finished_cb(uint8_t conidx, gapc_proc_simple_t* p_proc, uint16_t status)
{
    p_proc->cmp_cb(conidx, p_proc->dummy, status);
}


#if(BLE_GAPC)
uint16_t gapc_proc_le_create(uint8_t conidx, uint16_t proc_size, const hl_proc_itf_t *p_itf, hl_proc_t** pp_proc)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

    if(p_con != NULL)
    {
        status = gapc_proc_create(&(p_con->hdr), proc_size, p_itf, pp_proc);
    }

    return (status);
}
#endif // (BLE_GAPC)

/// Default Information command complete event handler
void gapc_proc_info_default_hci_cmp_evt_handler(uint16_t opcode, uint16_t event,
                                           const struct hci_basic_conhdl_cmd_cmp_evt *p_evt)
{
    uint8_t conidx = gapc_get_conidx(p_evt->conhdl);
    gapc_proc_info_t* p_proc = (gapc_proc_info_t*) gapc_proc_get(conidx);
    if(p_proc != NULL)
    {
        p_proc->p_res_info = p_evt;
        gapc_proc_transition(conidx, event,  RW_ERR_HCI_TO_HL(p_evt->status));
    }
}

void gapc_proc_info_default_hci_le_cmp_evt_handler(uint16_t event, const struct hci_basic_conhdl_le_cmd_cmp_evt *p_evt)
{
    uint8_t conidx = gapc_get_conidx(p_evt->conhdl);
    gapc_proc_info_t* p_proc = (gapc_proc_info_t*) gapc_proc_get(conidx);
    if(p_proc != NULL)
    {
        p_proc->p_res_info = p_evt;
        gapc_proc_transition(conidx, event,  RW_ERR_HCI_TO_HL(p_evt->status));
    }
}

/// Create a get information procedure
uint16_t gapc_proc_info_create(uint8_t conidx, uint32_t dummy, uint32_t hci_cmd_info,
                               const gapc_proc_simple_itf_t* p_itf, gapc_proc_cmp_cb cmp_cb)
{
    gapc_proc_info_t* p_proc;
    uint16_t status;

    status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_proc_info_t), p_itf,
                                     (gapc_proc_simple_t**) &p_proc);
    if(status == GAP_ERR_NO_ERROR)
    {
        p_proc->hci_cmd_info = hci_cmd_info;
        p_proc->p_res_info   = NULL;
    }

    return (status);
}
#endif // (GAPC_PRESENT)
/// @} GAPC
