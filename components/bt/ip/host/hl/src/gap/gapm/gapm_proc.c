/**
 ****************************************************************************************
 *
 * @file gapm_proc.c
 *
 * @brief GAP Manager procedure API
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

#include "rwip_config.h"    // IP Configuration
#include "rwip.h"           // Initialization enum
#include "gapm_int.h"       // GAPM Internal API
#include "ke_mem.h"         // For procedure allocation/free
#include "co_math.h"        // For bit field manipulation


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
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

__STATIC bool gapm_proc_info_transition(gapm_proc_info_t* p_proc, uint8_t event, uint16_t status);
__STATIC bool gapm_proc_send_hci_cmd_transition(gapm_proc_send_hci_cmd_t* p_proc, uint8_t event, uint16_t status);

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
/// Get controller information procedure interface
__STATIC const hl_proc_itf_t gapm_info_proc_itf =
{
    .transition  = (hl_proc_transition_cb) gapm_proc_info_transition,
    .cleanup     = hl_proc_cleanup,
};

/// Send hci command procedure transition interface
__STATIC const hl_proc_itf_t gapm_proc_send_hci_write_cmd_itf =
{
    .transition  = (hl_proc_transition_cb) gapm_proc_send_hci_cmd_transition,
    .cleanup     = hl_proc_cleanup,
};

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Function called when an event is trigger that creates a transition in procedure state machine
 *
 * @param[in] p_proc     Pointer to procedure object
 * @param[in] event      Event type receive that induce procedure state transition
 * @param[in] status     Status linked to event transition (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC bool gapm_proc_info_transition(gapm_proc_info_t* p_proc, uint8_t event, uint16_t status)
{
    ASSERT_ERR((status == GAP_ERR_NO_ERROR) && (event == HL_PROC_GRANTED));

    if((status == GAP_ERR_NO_ERROR) && (event == HL_PROC_GRANTED))
    {
        HL_HCI_BASIC_CMD_SEND(p_proc->hci_cmd_opcode, 0, p_proc->hci_cmd_evt_cb);
    }

    return false;
}


/// Write loopback_mode procedure transition
__STATIC bool gapm_proc_send_hci_cmd_transition(gapm_proc_send_hci_cmd_t* p_proc, uint8_t event, uint16_t status)
{
    bool is_finished = true;

    if(event == HL_PROC_GRANTED)
    {
        const gapm_proc_hci_cmd_info_t* p_info = p_proc->p_info;

        void* p_cmd = hl_hci_cmd_alloc(p_info->cmd_op_code, p_info->cmd_length);
        if(p_cmd != NULL)
        {
            p_info->prep_cb(p_proc, p_cmd);
            HL_HCI_CMD_SEND_TO_CTRL(p_cmd, HL_PROC_FINISHED, gapm_default_cfg_hci_cmd_cmp_evt_handler);
            is_finished = false;
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    if(is_finished)
    {
        // inform upper layer about procedure execution
        gapm_proc_execute_cmp_cb(&(p_proc->hdr), status);
    }

    return (is_finished);
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void gapm_default_cfg_hci_cmd_cmp_evt_handler(uint16_t opcode, uint16_t event, struct hci_basic_cmd_cmp_evt const *p_evt)
{
    gapm_proc_transition(GAPM_PROC_CFG, event, RW_ERR_HCI_TO_HL(p_evt->status));
}

uint16_t gapm_proc_create(uint8_t proc_type, uint16_t proc_size, const hl_proc_itf_t *p_itf, hl_proc_t** pp_proc)
{
    uint16_t status = hl_proc_create(&gapm_env.proc_queue[proc_type], proc_size, p_itf, pp_proc);
    return (status);
}

void gapm_proc_transition(uint8_t proc_type, uint8_t event, uint16_t status)
{
    hl_proc_transition(&gapm_env.proc_queue[proc_type], event, status);
}

hl_proc_t* gapm_proc_get(uint8_t proc_type)
{
    return (hl_proc_get(&gapm_env.proc_queue[proc_type]));
}

void gapm_proc_stop(uint8_t proc_type)
{
    hl_proc_stop(&gapm_env.proc_queue[proc_type]);
}

uint16_t gapm_proc_info_start(uint32_t dummy, uint16_t hci_cmd_opcode, hl_hci_cmd_evt_func_t cmd_evt_cb,
                                       gapm_proc_info_res_cb res_cb)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    do
    {
        gapm_proc_info_t* p_proc = NULL;
        if(res_cb == NULL)
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Create new procedure
        status = gapm_proc_create(GAPM_PROC_CFG, sizeof(gapm_proc_info_t),
                                  &gapm_info_proc_itf, (hl_proc_t**) &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        p_proc->hci_cmd_opcode = hci_cmd_opcode;
        p_proc->res_cb         = res_cb;
        p_proc->hci_cmd_evt_cb = cmd_evt_cb;
        p_proc->dummy          = dummy;
    } while (0);

    return (status);
}

uint16_t gapm_proc_info_stop(uint32_t* p_dummy, gapm_proc_info_res_cb* p_res_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapm_proc_info_t* p_proc = (gapm_proc_info_t*) gapm_proc_get(GAPM_PROC_CFG);
    ASSERT_ERR(p_proc != NULL);
    if(p_proc != NULL)
    {
        *p_res_cb = p_proc->res_cb;
        *p_dummy  = p_proc->dummy;
        // clean-up procedure
        gapm_proc_stop(GAPM_PROC_CFG);
    }

    return (status);
}

uint16_t gapm_proc_set_cfg_create(uint16_t proc_size, const hl_proc_itf_t *p_itf, uint32_t dummy,
                                  gapm_proc_cmp_cb cmp_cb, gapm_proc_set_cfg_t** pp_proc)
{
    uint16_t status;

    do
    {
        if(cmp_cb == NULL)
        {
            status = GAP_ERR_MISSING_CALLBACK;
            break;
        }

        // Create new procedure
        status = gapm_proc_create(GAPM_PROC_CFG, proc_size, p_itf, (hl_proc_t**) pp_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        (*pp_proc)->cmp_cb         = cmp_cb;
        (*pp_proc)->dummy          = dummy;
    } while (0);

    return (status);
}

uint16_t gapm_proc_send_hci_cmd_create(uint16_t proc_size, const gapm_proc_hci_cmd_info_t* p_info, uint32_t dummy,
                                       gapm_proc_cmp_cb cmp_cb, gapm_proc_send_hci_cmd_t** pp_proc)
{
    uint16_t status = gapm_proc_set_cfg_create(proc_size, &gapm_proc_send_hci_write_cmd_itf, dummy, cmp_cb,
                                               (gapm_proc_set_cfg_t**) pp_proc);
    if(status == GAP_ERR_NO_ERROR)
    {
        (*pp_proc)->p_info = p_info;
    }

    return status;
}


void gapm_proc_execute_cmp_cb(gapm_proc_set_cfg_t* p_proc, uint16_t status)
{
     gapm_proc_cmp_cb cmp_cb = p_proc->cmp_cb;
     // inform upper layer about procedure execution
     cmp_cb(p_proc->dummy, status);
}


void gapm_proc_initialize(uint8_t init_type)
{
    switch (init_type)
    {
        case RWIP_INIT: { /* Do Nothing */ } break;
        default:
        {
            // clean procedure in queue
            uint8_t proc_type;
            for (proc_type = 0; proc_type < GAPM_PROC_MAX; proc_type++)
            {
                hl_proc_queue_initialize(&gapm_env.proc_queue[proc_type]);
            }
        } break;
    }
}


void gapm_proc_empty(void* p_proc_dummy, void* p_cmd_dummy)
{
}

/// @} GAPM
