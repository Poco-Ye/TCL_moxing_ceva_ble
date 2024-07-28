/**
 ****************************************************************************************
 *
 * @file gapm_list.c
 *
 * @brief Generic Access Profile Manager list manager module.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPM_LIST Generic Access Profile Manager list manager
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
#include "gapm_int.h"
#include "gapm_le.h"

#include "hl.h"
#include "rwip.h"            // Common API to retrieve device parameters

#include <string.h>

#include "co_error.h"
#include "co_bt.h"
#include "co_math.h"
#include "co_version.h"
#include "co_utils.h"        // core utility functions


#include "ke_mem.h"

#include "hl_hci.h"
#include "hci.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Event transition for address fill state machine
enum gapm_list_fill_proc_event
{
    /// Inform that list has been cleared
    GAPM_LIST_FILL_PROC_LIST_CLEARED = HL_PROC_EVENT_FIRST,
    /// New entry in the list added
    GAPM_LIST_FILL_PROC_ENTRY_ADDED,
    /// For resolving list only, prepare entry before setting privacy mode
    GAPM_LIST_FILL_PROC_ENTRY_PREPARED,
};

/*
 * MACROS
 ****************************************************************************************
 */



/*
 * TYPE DEFINITION
 ****************************************************************************************
 */

/// Entry within a list
typedef union gapm_list_entry
{
    /// White list
    gap_bdaddr_t wl;
    /// Resolving list
    gap_ral_dev_info_t ral;
    /// Periodic Adv List
    gap_ral_dev_info_t pal;
} gapm_list_entry_t;


/// Clear list using HCI
typedef uint16_t (*gapm_list_clear_cb)(void);

/**
 ****************************************************************************************
 * @brief  Function prototype used to add a list entry using HCI command
 *
 * @param[in] index        Pointer to entry index
 * @param[in] p_array      Pointer to list array
 * @param[in] event_trans  Procedure Event transition returned in command complete event
 *
 * @return status of function execution (@see  hl_err)
 ****************************************************************************************
 */
typedef uint16_t (*gapm_list_entry_hci_add_cmd_cb)(uint8_t index, const gapm_list_entry_t* p_array, uint8_t event_trans);

/// Interface of callback used to manage a list
typedef struct gapm_list_fill_itf
{
    /// Clear List
    gapm_list_clear_cb              clear_list;
    /// Add an entry
    gapm_list_entry_hci_add_cmd_cb  add_entry;
} gapm_list_fill_itf_t;


/// Structure of Filling list procedure
typedef struct gapm_list_fill_proc
{
    /// Procedure inheritance
    gapm_proc_set_cfg_t             hdr;
    /// Array of list entries
    const gapm_list_entry_t*        p_array;
    /// Function used to manage a list
    const gapm_list_fill_itf_t*     p_cbs;
    /// Size of list
    uint8_t                         size;
    /// List cursor
    uint8_t                         cursor;
} gapm_list_fill_proc_t;


/// Structure of get resolving list information
typedef struct gapm_list_get_ral_info_proc
{
    /// Procedure inheritance
    hl_proc_t            hdr;
    /// Dummy parameter provided by upper layer SW
    uint32_t             dummy;
    /// Callback to execute at end of procedure execution
    gapm_get_rpa_res_cb  res_cb;
    /// HCI Command operation code to use
    uint16_t             hci_opcode;
    /// Peer identity address
    gap_bdaddr_t         peer_identity;
} gapm_list_get_ral_info_proc_t;

/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */
__STATIC bool gapm_list_fill_proc_transition(gapm_list_fill_proc_t* p_proc, uint8_t event, uint16_t status);

__STATIC bool gapm_list_get_ral_info_proc_transition(gapm_list_get_ral_info_proc_t* p_proc, uint8_t event, uint16_t status);

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// List Fill procedure interface
__STATIC const hl_proc_itf_t gapm_list_fill_proc_itf =
{
    .transition  = (hl_proc_transition_cb) gapm_list_fill_proc_transition,
    .cleanup     = hl_proc_cleanup,
};

/// Get resolving list information
__STATIC const hl_proc_itf_t gapm_list_get_ral_info_proc_itf =
{
    .transition  = (hl_proc_transition_cb) gapm_list_get_ral_info_proc_transition,
    .cleanup     = hl_proc_cleanup,
};

/*
 * HCI EVENT HANDLERS DEFINITIONS
 ****************************************************************************************
 */

/// @brief Read resolving list address complete event handler.
__STATIC void gapm_hci_le_rd_ral_addr_cmd_cmp_evt_handler(uint16_t opcode, uint16_t event,
                                                          struct hci_le_rd_peer_rslv_addr_cmd_cmp_evt const *p_evt)
{
    gapm_list_get_ral_info_proc_t* p_proc = (gapm_list_get_ral_info_proc_t*) gapm_proc_get(GAPM_PROC_CFG);
    ASSERT_ERR(p_proc != NULL);
    if(p_proc != NULL)
    {
        gap_addr_t addr;
        uint32_t dummy = p_proc->dummy;
        gapm_get_rpa_res_cb res_cb =  p_proc->res_cb;

        gapm_proc_stop(GAPM_PROC_CFG);

        memcpy(&addr, &p_evt->peer_rslv_addr, GAP_BD_ADDR_LEN);

        // send back result
        res_cb(dummy, RW_ERR_HCI_TO_HL(p_evt->status), &addr);
    }
}


/// @brief Send a LE Clear White List command over HCI. Command complete event
__STATIC uint16_t gapm_list_send_hci_le_clear_wlst_cmd(void)
{
    uint16_t status = HL_HCI_BASIC_CMD_SEND(HCI_LE_CLEAR_WLST_CMD_OPCODE, GAPM_LIST_FILL_PROC_LIST_CLEARED,
                                  gapm_default_cfg_hci_cmd_cmp_evt_handler);
    gapm_env.nb_dev_wl = 0;
    return (status);
}

/// @brief Send a LE Add Device To White List command over HCI. Command complete event
__STATIC uint16_t gapm_list_send_hci_le_add_dev_to_wlst_cmd(uint8_t index, const gap_bdaddr_t* p_array, uint8_t event_trans)
{
    const gap_bdaddr_t* p_entry = &p_array[index];
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    struct hci_le_add_dev_to_wlst_cmd *p_cmd =
            HL_HCI_CMD_ALLOC(HCI_LE_ADD_DEV_TO_WLST_CMD_OPCODE, hci_le_add_dev_to_wlst_cmd);
    if(p_cmd != NULL)
    {
        p_cmd->dev_addr_type = p_entry->addr_type;
        memcpy(&p_cmd->dev_addr.addr[0], &p_entry->addr[0], BD_ADDR_LEN);

        gapm_env.nb_dev_wl++;
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, GAPM_LIST_FILL_PROC_ENTRY_ADDED, gapm_default_cfg_hci_cmd_cmp_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

/// Interface of callback used to manage white list
__STATIC const gapm_list_fill_itf_t gapm_list_fill_wl_itf =
{
    .clear_list = gapm_list_send_hci_le_clear_wlst_cmd,
    .add_entry  = (gapm_list_entry_hci_add_cmd_cb) gapm_list_send_hci_le_add_dev_to_wlst_cmd,
};


/// Send a LE Clear Resolving List command over HCI. Command complete event
__STATIC uint16_t gapm_list_send_hci_le_clear_rslv_list_cmd(void)
{
    uint16_t status = HL_HCI_BASIC_CMD_SEND(HCI_LE_CLEAR_RSLV_LIST_CMD_OPCODE, GAPM_LIST_FILL_PROC_LIST_CLEARED,
                                            gapm_default_cfg_hci_cmd_cmp_evt_handler);
    return (status);
}

/// Send a LE Add Device To Resolving List command over HCI. Command complete event
__STATIC uint16_t gapm_list_send_hci_le_add_dev_to_rslv_list_cmd(uint8_t index, const gap_ral_dev_info_t* p_array, uint8_t event_trans)
{
    const gap_ral_dev_info_t* p_entry = &p_array[index];
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    // Put device in list
    if(event_trans != GAPM_LIST_FILL_PROC_ENTRY_PREPARED)
    {
        struct hci_le_add_dev_to_rslv_list_cmd *p_cmd =
                HL_HCI_CMD_ALLOC(HCI_LE_ADD_DEV_TO_RSLV_LIST_CMD_OPCODE, hci_le_add_dev_to_rslv_list_cmd);

        if(p_cmd != NULL)
        {
            p_cmd->peer_id_addr_type = p_entry->addr.addr_type;
            memcpy(&p_cmd->peer_id_addr.addr[0], &p_entry->addr.addr[0], BD_ADDR_LEN);
            memcpy(&p_cmd->peer_irk.key[0],      &p_entry->peer_irk[0],  GAP_KEY_LEN);
            memcpy(&p_cmd->local_irk.key[0],     &p_entry->local_irk[0], GAP_KEY_LEN);
            HL_HCI_CMD_SEND_TO_CTRL(p_cmd, GAPM_LIST_FILL_PROC_ENTRY_PREPARED, gapm_default_cfg_hci_cmd_cmp_evt_handler);
            status = GAP_ERR_NO_ERROR;
        }
    }
    // Set privacy mode
    else
    {

        struct hci_le_set_priv_mode_cmd *p_cmd =
                HL_HCI_CMD_ALLOC(HCI_LE_SET_PRIV_MODE_CMD_OPCODE, hci_le_set_priv_mode_cmd);

        if(p_cmd != NULL)
        {
            p_cmd->peer_addr_type = p_entry->addr.addr_type;
            memcpy(&p_cmd->peer_addr.addr[0], &p_entry->addr.addr[0], BD_ADDR_LEN);
            p_cmd->priv_mode      = p_entry->priv_mode;

            HL_HCI_CMD_SEND_TO_CTRL(p_cmd, GAPM_LIST_FILL_PROC_ENTRY_ADDED, gapm_default_cfg_hci_cmd_cmp_evt_handler);
            status = GAP_ERR_NO_ERROR;
        }
    }

    return (status);
}

/// Interface of callback used to manage resolving list
__STATIC const gapm_list_fill_itf_t gapm_list_fill_ral_itf =
{
    .clear_list = gapm_list_send_hci_le_clear_rslv_list_cmd,
    .add_entry  = (gapm_list_entry_hci_add_cmd_cb) gapm_list_send_hci_le_add_dev_to_rslv_list_cmd,
};

/// @brief Send a LE Clear Periodic Advertiser List command over HCI. Command complete event
__STATIC uint16_t gapm_list_send_hci_le_clear_per_adv_list_cmd(void)
{
    uint16_t status = HL_HCI_BASIC_CMD_SEND(HCI_LE_CLEAR_PER_ADV_LIST_CMD_OPCODE, GAPM_LIST_FILL_PROC_LIST_CLEARED,
                                            gapm_default_cfg_hci_cmd_cmp_evt_handler);
    return (status);
}

/// @brief Send a LE Add Device To Periodic Advertiser List command over HCI.
__STATIC uint16_t gapm_list_send_hci_le_add_dev_to_per_adv_list_cmd(uint8_t index, const gap_per_adv_bdaddr_t* p_array,
                                                                    uint8_t event_trans)
{
    const gap_per_adv_bdaddr_t* p_entry = &p_array[index];
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    struct hci_le_add_dev_to_per_adv_list_cmd *p_cmd =
            HL_HCI_CMD_ALLOC(HCI_LE_ADD_DEV_TO_PER_ADV_LIST_CMD_OPCODE, hci_le_add_dev_to_per_adv_list_cmd);

    if(p_cmd)
    {
        p_cmd->adv_addr_type = p_entry->addr_type;
        p_cmd->adv_sid       = p_entry->adv_sid;
        memcpy(&p_cmd->adv_addr.addr[0], &p_entry[0], BD_ADDR_LEN);
        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, GAPM_LIST_FILL_PROC_ENTRY_ADDED, gapm_default_cfg_hci_cmd_cmp_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

/// Interface of callback used to manage Periodic ADV list
__STATIC const gapm_list_fill_itf_t gapm_list_fill_pal_itf =
{
    .clear_list = gapm_list_send_hci_le_clear_per_adv_list_cmd,
    .add_entry  = (gapm_list_entry_hci_add_cmd_cb) gapm_list_send_hci_le_add_dev_to_per_adv_list_cmd,
};

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

///  * @brief Function called when an event is trigger that creates a transition in procedure state machine
__STATIC bool gapm_list_fill_proc_transition(gapm_list_fill_proc_t* p_proc, uint8_t event, uint16_t status)
{

    bool finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                // Start by clearing list
                status = p_proc->p_cbs->clear_list();
            }break;
            case GAPM_LIST_FILL_PROC_ENTRY_ADDED:
            {
                // increment cursor since new entry added
                p_proc->cursor++;
            }
            // no break;
            default:
            {
                if(p_proc->cursor == p_proc->size)
                {
                    finished = true;
                    break;
                }

                // Add Array entry in list
                status = p_proc->p_cbs->add_entry(p_proc->cursor, p_proc->p_array, event);
            }break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        finished = true;
    }

    if(finished)
    {
        gapm_proc_execute_cmp_cb(&(p_proc->hdr), status);
    }

    return (finished);
}

///  * @brief Function called when an event is trigger that creates a transition in procedure state machine
__STATIC bool gapm_list_get_ral_info_proc_transition(gapm_list_get_ral_info_proc_t* p_proc, uint8_t event, uint16_t status)
{
    bool finished = false;

    switch(event)
    {
        case HL_PROC_GRANTED:
        {
            struct hci_le_rd_loc_rslv_addr_cmd *p_cmd = HL_HCI_CMD_ALLOC(p_proc->hci_opcode, hci_le_rd_loc_rslv_addr_cmd);
            if(p_cmd == NULL)
            {
                status = GAP_ERR_INSUFF_RESOURCES;
                break;
            }

            // Fill up the parameters */
            p_cmd->peer_id_addr_type = p_proc->peer_identity.addr_type;
            memcpy(&p_cmd->peer_id_addr.addr, p_proc->peer_identity.addr, BD_ADDR_LEN);

            // Send the message
            HL_HCI_CMD_SEND_TO_CTRL(p_cmd, HL_PROC_FINISHED, gapm_hci_le_rd_ral_addr_cmd_cmp_evt_handler);
        }break;

        // Note: the procedure is aborted as soon as an HCI result is received to reduce code complexity
        default: { ASSERT_ERR(0); } break;
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_get_rpa_res_cb res_cb = (gapm_get_rpa_res_cb)p_proc->res_cb;
        finished = true;
        // inform upper layer about procedure execution
        res_cb(p_proc->dummy, status, NULL);
    }

    return (finished);
}

uint16_t gapm_list_fill(uint32_t dummy, const gapm_list_fill_itf_t* p_cbs,
                        uint8_t size, const gapm_list_entry_t* p_array, gapm_proc_cmp_cb cmp_cb)
{

    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    do
    {
        gapm_list_fill_proc_t* p_proc = NULL;

        if((size != 0) && (p_array == NULL))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Create new procedure
        status = gapm_proc_set_cfg_create(sizeof(gapm_list_fill_proc_t), &gapm_list_fill_proc_itf, dummy, cmp_cb,
                                  (gapm_proc_set_cfg_t**) &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        // Initialize procedure parameters
        p_proc->p_array = p_array;
        p_proc->p_cbs   = p_cbs;
        p_proc->size    = size;
        p_proc->cursor  = 0;
    } while (0);

    return (status);
}

/// Retrieve Resolving address list information
__STATIC uint16_t gapm_list_get_ral_info(uint32_t dummy, uint16_t hci_opcode, const gap_bdaddr_t* p_peer_identity,
                                         gapm_get_rpa_res_cb res_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    do
    {
        gapm_list_get_ral_info_proc_t* p_proc = NULL;

        if((res_cb == NULL) || (p_peer_identity == NULL))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Create new procedure
        status = gapm_proc_create(GAPM_PROC_CFG, sizeof(gapm_list_get_ral_info_proc_t), &gapm_list_get_ral_info_proc_itf,
                                  (hl_proc_t**) &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        // Initialize procedure parameters
        p_proc->res_cb        = res_cb;
        p_proc->hci_opcode    = hci_opcode;
        p_proc->dummy         = dummy;
        p_proc->peer_identity = *p_peer_identity;
    } while (0);

    return (status);
}


/*
 * EXTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapm_fill_white_list(uint32_t dummy, uint8_t size, const gap_bdaddr_t* p_array, gapm_proc_cmp_cb cmp_cb)
{
    return (gapm_list_fill(dummy, &gapm_list_fill_wl_itf, size, (const gapm_list_entry_t*) p_array, cmp_cb));
}

uint16_t gapm_fill_resolving_address_list(uint32_t dummy, uint8_t size, const gap_ral_dev_info_t* p_array,
                                         gapm_proc_cmp_cb cmp_cb)
{
    return (gapm_list_fill(dummy, &gapm_list_fill_ral_itf, size, (const gapm_list_entry_t*) p_array, cmp_cb));
}

uint16_t gapm_fill_periodic_adv_list(uint32_t dummy, uint8_t size, const gap_per_adv_bdaddr_t* p_array,
                                        gapm_proc_cmp_cb cmp_cb)
{
    return (gapm_list_fill(dummy, &gapm_list_fill_pal_itf, size, (const gapm_list_entry_t*) p_array, cmp_cb));
}

uint16_t gapm_get_ral_local_rpa(uint32_t dummy, const gap_bdaddr_t* p_peer_identity, gapm_get_rpa_res_cb res_cb)
{
    return (gapm_list_get_ral_info(dummy, HCI_LE_RD_LOC_RSLV_ADDR_CMD_OPCODE, p_peer_identity, res_cb));
}

uint16_t gapm_get_ral_peer_rpa(uint32_t dummy, const gap_bdaddr_t* p_peer_identity, gapm_get_rpa_res_cb res_cb)
{
    return (gapm_list_get_ral_info(dummy, HCI_LE_RD_PEER_RSLV_ADDR_CMD_OPCODE, p_peer_identity, res_cb));
}

#endif // (BLE_HOST_PRESENT)

/// @} GAPM_LIST
