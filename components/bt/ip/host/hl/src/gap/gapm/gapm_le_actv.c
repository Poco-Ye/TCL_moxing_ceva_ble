/**
 ****************************************************************************************
 *
 * @file gapm_le_actv.c
 *
 * @brief Generic Access Profile Manager - LE Activities
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#if (BLE_HOST_PRESENT)

#include <string.h>
#include "gap.h"
#include "gapm_int.h"
#include "gapm_le.h"
#include "ke_mem.h"
#include "hl_hci.h"
#include "hci.h"
#include "aes.h"
#include "../../inc/prf_hl_api.h"


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Address renewal information bit field
enum gapm_le_actv_addr_info_bf
{
    #if (HL_LE_OBSERVER)
    /// Scan and initiating random address must be generated
    GAPM_ADDR_RENEW_SCAN_INIT_POS     = 0,
    GAPM_ADDR_RENEW_SCAN_INIT_BIT     = 0x01,
    #endif // (HL_LE_OBSERVER)

    /// Renew all activity address
    GAPM_ADDR_RENEW_ALL_POS           = 1,
    GAPM_ADDR_RENEW_ALL_BIT           = 0x02,

    /// Renew timer started
    GAPM_ADDR_RENEW_TIMER_STARTED_POS = 2,
    GAPM_ADDR_RENEW_TIMER_STARTED_BIT = 0x04,

    /// Renew procedure started
    GAPM_ADDR_RENEW_PROC_STARTED_POS  = 3,
    GAPM_ADDR_RENEW_PROC_STARTED_BIT  = 0x08,
};

#if(HL_LE_OBSERVER)
/// SCAN / INIT activity address renew
enum gapm_le_actv_addr_renew_proc_event
{
    /// Result of Address generation procedure received
    GAPM_ADDR_RENEW_GEN_RAND_ADDR_CMP = HL_PROC_EVENT_FIRST,
    /// Received HCI_LE_SET_ADV_SET_RAND_ADDR_CMD complete event
    GAPM_ADDR_RENEW_HCI_SET_ADDR_CMP,
};
#endif // (HL_LE_OBSERVER)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure of Address generation procedure
typedef struct gapm_le_actv_addr_proc_renew
{
    /// Procedure inheritance
    hl_proc_t   hdr;
    /// Activity index cursor
    uint8_t     actv_idx;
} gapm_le_actv_addr_proc_renew_t;

/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */
__STATIC bool gapm_le_actv_addr_proc_renew_transition(gapm_le_actv_addr_proc_renew_t* p_proc, uint8_t event, uint16_t status);

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Address update procedure interface
__STATIC const hl_proc_itf_t gapm_le_actv_addr_proc_renew_itf =
{
    .transition  = (hl_proc_transition_cb) gapm_le_actv_addr_proc_renew_transition,
    .cleanup     = hl_proc_cleanup,
};

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/// Start address renew procedure
__STATIC void gapm_le_actv_addr_do_renew(void)
{
    if(!GETB(gapm_env.addr_info_bf, GAPM_ADDR_RENEW_PROC_STARTED))
    {
        gapm_le_actv_addr_proc_renew_t* p_proc;
        if(gapm_proc_create(GAPM_PROC_AIR, sizeof(gapm_le_actv_addr_proc_renew_t), &gapm_le_actv_addr_proc_renew_itf,
                            (hl_proc_t**)&p_proc) == GAP_ERR_NO_ERROR)
        {
            SETB(gapm_env.addr_info_bf, GAPM_ADDR_RENEW_PROC_STARTED, true);
            p_proc->actv_idx = 0;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Function to called once timer expires
 *
 * @param[in] p_env   Pointer to environment that will be used as callback parameter.
 ****************************************************************************************
 */
__STATIC void gapm_le_actv_addr_renew_timer_expire(void* p_env)
{
    SETB(gapm_env.addr_info_bf, GAPM_ADDR_RENEW_TIMER_STARTED, false);

    // only do something if address renewed
    if(gapm_env.actv_bf != 0)
    {
        #if(HL_LE_OBSERVER)
        SETB(gapm_env.addr_info_bf, GAPM_ADDR_RENEW_SCAN_INIT, true);
        #endif // (HL_LE_OBSERVER)
        SETB(gapm_env.addr_info_bf, GAPM_ADDR_RENEW_ALL, true);
        gapm_le_actv_addr_renew_timer_start();
        gapm_le_actv_addr_do_renew();
    }
}

#if (HL_LE_OBSERVER)
__STATIC void gapm_le_actv_addr_rand_addr_cmp_handler(uint8_t aes_status, const gap_addr_t* p_addr, uint32_t event)
{
    uint16_t status = RW_ERR_HCI_TO_HL(aes_status);

    if(status == GAP_ERR_NO_ERROR)
    {
        gapm_env.scan_init_rand_addr = *p_addr;
    }

    gapm_proc_transition(GAPM_PROC_AIR, (uint8_t) event, status);
}
#endif // (HL_LE_OBSERVER)

/*
 * HCI HANDLERS DEFINITIONS
 ****************************************************************************************
 */

#if (HL_LE_OBSERVER)
/// Default HCI Command complete handler used to continue procedure execution
__STATIC void gapm_le_actv_addr_hci_cmd_cmp_handler(uint16_t opcode, uint16_t event, struct hci_basic_cmd_cmp_evt const *p_evt)
{
    gapm_proc_transition(GAPM_PROC_AIR, (uint8_t) event, RW_ERR_HCI_TO_HL(p_evt->status));
}
#endif // (HL_LE_OBSERVER)

#if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
/// HCI Command complete handler used to ignore completion of command execution
__STATIC void gapm_le_actv_hci_cmd_cmp_ignore_handler(uint16_t opcode, uint16_t event, struct hci_basic_cmd_cmp_evt const *p_evt)
{
    // Do nothing
}
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)


/*
 * PROCEDURE TRANSITION - State machine
 ****************************************************************************************
 */


#if (HL_LE_OBSERVER)
/// SCAN / INIT address renew procedure state machine
__STATIC uint16_t gapm_le_actv_addr_scan_init_renew_transition(uint8_t event, uint16_t status, bool* p_finished)
{
    *p_finished = false;
    if(gapm_env.scan_init_own_addr_type == GAPM_STATIC_ADDR)
    {
        *p_finished = true;
    }
    else if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                uint8_t addr_type = (gapm_env.scan_init_own_addr_type == GAPM_GEN_RSLV_ADDR)
                                  ? BD_ADDR_RSLV : BD_ADDR_NON_RSLV;
                // Send an address generation command
                aes_gen_rand_addr(gapm_get_irk()->key, addr_type,
                                  (aes_func_result_cb)gapm_le_actv_addr_rand_addr_cmp_handler,
                                  GAPM_ADDR_RENEW_GEN_RAND_ADDR_CMP);
            }
            break;
            case GAPM_ADDR_RENEW_GEN_RAND_ADDR_CMP:
            {
                // set random address only if scan or init activity is started
                if(   (gapm_env.scan_actv_idx != GAP_INVALID_ACTV_IDX)
                   #if(HL_LE_CENTRAL)
                   || (gapm_env.scan_actv_idx != GAP_INVALID_ACTV_IDX)
                   #endif // (HL_LE_CENTRAL)
                   )
                {
                    status = gapm_le_actv_addr_send_hci_le_set_rand_addr_cmd(GAPM_ADDR_RENEW_HCI_SET_ADDR_CMP,
                                                                (hl_hci_cmd_evt_func_t) gapm_le_actv_addr_hci_cmd_cmp_handler);
                    break;
                }
            }
            // no break
            case GAPM_ADDR_RENEW_HCI_SET_ADDR_CMP:
            default: { *p_finished = true; }break;
        }
    }
    if(status != GAP_ERR_NO_ERROR)
    {
        *p_finished = true;
    }

    return (status);
}
#endif // (HL_LE_OBSERVER)

/// handle transition of address renewal procedure state machine
__STATIC bool gapm_le_actv_addr_proc_renew_transition(gapm_le_actv_addr_proc_renew_t* p_proc, uint8_t event, uint16_t status)
{
    bool finished = false;
    do
    {
        gapm_actv_t* p_actv;

        #if(HL_LE_OBSERVER)
        if(GETB(gapm_env.addr_info_bf, GAPM_ADDR_RENEW_SCAN_INIT))
        {
            // renew scan/init address
            gapm_le_actv_addr_scan_init_renew_transition(event, status, &finished);
            if(!finished) break;

            SETB(gapm_env.addr_info_bf, GAPM_ADDR_RENEW_SCAN_INIT, false);
            event = HL_PROC_GRANTED;
            status = GAP_ERR_NO_ERROR;
        }
        #endif // (HL_LE_OBSERVER))

        // Get Pointer to current activity
        p_actv = gapm_actv_get(p_proc->actv_idx);
        if(p_actv && (p_actv->p_itf->cb_addr_renew_transition != NULL))
        {
            // specific case for init and scan activities
            if(GETB(gapm_env.addr_info_bf, GAPM_ADDR_RENEW_ALL) || (p_actv->type != GAPM_ACTV_TYPE_ADV))
            {
                p_actv->p_itf->cb_addr_renew_transition(p_actv, NULL, event, status, &finished);
            }
            if(!finished) break;
            event = HL_PROC_GRANTED;
            status = GAP_ERR_NO_ERROR;
        }

        // check next activity
        p_proc->actv_idx++;
    } while (p_proc->actv_idx < GAPM_ACTV_NB);

    // check if procedure finished
    if(p_proc->actv_idx == GAPM_ACTV_NB)
    {
        SETB(gapm_env.addr_info_bf, GAPM_ADDR_RENEW_PROC_STARTED, false);
    }

    return (p_proc->actv_idx == GAPM_ACTV_NB);
}

/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (HL_LE_OBSERVER)
/**
 ****************************************************************************************
 * @brief Send HCI LE Set Random Address command to the controller.
 *
 * @param[in] event         Event transition to trigger once command completes
 * @param[in] cmd_evt_func  Pointer to HCI command complete or status event handler
 *
 * @return status of function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapm_le_actv_addr_send_hci_le_set_rand_addr_cmd(uint8_t event, hl_hci_cmd_evt_func_t cmd_evt_func)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    struct hci_le_set_rand_addr_cmd *p_cmd =
            HL_HCI_CMD_ALLOC(HCI_LE_SET_RAND_ADDR_CMD_OPCODE, hci_le_set_rand_addr_cmd);

    if(p_cmd != NULL)
    {
        const gap_addr_t* p_addr = (gapm_env.scan_init_own_addr_type == GAPM_STATIC_ADDR)
                                 ? &(gapm_env.addr) : &(gapm_env.scan_init_rand_addr);
        memcpy(&p_cmd->rand_addr.addr[0], p_addr, BD_ADDR_LEN);

        // Send the command
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, event, cmd_evt_func);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}
#endif //(HL_LE_OBSERVER)



void gapm_le_actv_initialize(uint8_t init_type)
{
    co_time_timer_init(&(gapm_env.addr_renew_timer), gapm_le_actv_addr_renew_timer_expire, NULL);
    gapm_env.addr_info_bf = 0;
    #if(HL_LE_OBSERVER)
    gapm_env.scan_init_own_addr_type = GAPM_STATIC_ADDR;
    #endif // (HL_LE_OBSERVER)
}

void gapm_le_actv_addr_renew_timer_start(void)
{
    if(!GETB(gapm_env.addr_info_bf, GAPM_ADDR_RENEW_TIMER_STARTED))
    {
        SETB(gapm_env.addr_info_bf, GAPM_ADDR_RENEW_TIMER_STARTED, true);
        co_time_timer_set(&(gapm_env.addr_renew_timer), gapm_env.renew_dur * 1000);
    }
}

#if (HL_LE_OBSERVER)
bool gapm_le_actv_addr_is_type_valid(uint8_t addr_type)
{
    // Indicate if address type is valid
    bool addr_type_valid = false;

    if(addr_type <= GAPM_GEN_NON_RSLV_ADDR)
    {
        uint8_t actv_idx;
        // Loop over the activities
        for (actv_idx = 0; actv_idx < GAPM_ACTV_NB; actv_idx++)
        {
            // Get activity structure
            gapm_actv_t *p_actv = gapm_actv_get(actv_idx);
            // Check if initiating or scan activity exists
            if (p_actv && ((p_actv->type == GAPM_ACTV_TYPE_INIT) || (p_actv->type == GAPM_ACTV_TYPE_SCAN)))
            {
                break;
            }
        }

        // No init or scan or equals to other activities own address type
        addr_type_valid = ((actv_idx == GAPM_ACTV_NB) || (addr_type == gapm_env.scan_init_own_addr_type));
    }

    return (addr_type_valid);
}


#if(HOST_MSG_API)
bool gapm_le_actv_addr_is_scan_init_addr_generated(void)
{
    return (!GETB(gapm_env.addr_info_bf, GAPM_ADDR_RENEW_SCAN_INIT));
}
#endif // (HOST_MSG_API)

void gapm_le_actv_addr_scan_init_type_set(uint8_t own_addr_type)
{
    bool addr_type_change = (own_addr_type != gapm_env.scan_init_own_addr_type);

    gapm_env.scan_init_own_addr_type = own_addr_type;

    if(own_addr_type != GAPM_STATIC_ADDR)
    {
        // Start renewal timer if not done
        gapm_le_actv_addr_renew_timer_start();

        // address type change, ask for immediate address generation
        if(addr_type_change)
        {
            SETB(gapm_env.addr_info_bf, GAPM_ADDR_RENEW_SCAN_INIT, true);
            gapm_le_actv_addr_do_renew();
        }
    }
}
#endif // (HL_LE_OBSERVER)

uint8_t gapm_le_actv_get_hci_own_addr_type(uint8_t own_addr_type)
{
    //  Own address type
    uint8_t hci_own_addr_type = ADDR_RAND;

    if (own_addr_type == GAPM_STATIC_ADDR)
    {
         if (!GETB(gapm_env.priv_cfg, GAPM_PRIV_CFG_PRIV_ADDR))
         {
             hci_own_addr_type = ADDR_PUBLIC;
         }
    }

    // Check if controller privacy is enabled
    if (GETB(gapm_env.priv_cfg, GAPM_PRIV_CFG_PRIV_EN))
    {
        hci_own_addr_type |= ADDR_RPA_MASK;
    }

    return (hci_own_addr_type);
}

void gapm_le_actv_send_new_bdaddr(gapm_actv_t *p_actv, const gap_addr_t* p_addr)
{
    const gapm_le_actv_cb_t* p_cbs = (const gapm_le_actv_cb_t*) p_actv->p_cbs;

    if(p_cbs->addr_updated != NULL)
    {
        p_cbs->addr_updated(p_actv->dummy, p_actv->idx, p_addr);
    }
}

#if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)

uint16_t gapm_le_actv_con_create(gapm_actv_t *p_actv, bool is_name_discovery, const struct hci_le_enh_con_cmp_evt* p_evt,
                                 uint8_t own_addr_type, const gap_addr_t* p_rpa, uint8_t* p_conidx)
{
    uint8_t conidx;
    uint16_t status;
    gap_bdaddr_t local_addr;

    // retrieve initiating host address
    if(own_addr_type != GAPM_STATIC_ADDR)
    {
        memcpy(local_addr.addr, p_rpa, BD_ADDR_LEN);
        local_addr.addr_type = ADDR_RAND;
    }
    else
    {
        memcpy(local_addr.addr, gapm_env.addr.addr, BD_ADDR_LEN);
        local_addr.addr_type = (GETB(gapm_env.priv_cfg, GAPM_PRIV_CFG_PRIV_ADDR) ? ADDR_RAND : ADDR_PUBLIC);;
    }

    // Connection creation succeeds, inform all layers
    status = gapc_le_con_create(p_actv->idx, p_actv->dummy,  is_name_discovery, &local_addr, p_evt, &conidx);
    if(status == GAP_ERR_NO_ERROR)
    {
        // Increment number of connections.
        gapm_env.connections++;

        *p_conidx = conidx;
    }

    return (status);
}


void gapm_le_actv_send_hci_disconnect(uint16_t conhdl, uint8_t reason)
{
    struct hci_disconnect_cmd *p_cmd = HL_HCI_CMD_ALLOC(HCI_DISCONNECT_CMD_OPCODE, hci_disconnect_cmd);
    if(p_cmd != NULL)
    {
        p_cmd->conhdl = conhdl;
        p_cmd->reason = reason;
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, p_cmd->conhdl, gapm_le_actv_hci_cmd_cmp_ignore_handler);
    }
}
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
#endif // (BLE_HOST_PRESENT)
/// @} GAPM
