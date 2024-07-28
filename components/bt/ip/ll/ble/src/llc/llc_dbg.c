/**
 ****************************************************************************************
 *
 * @file llc_dbg.c
 *
 * @brief Handles the link-specific debug commands.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup LLC_DBG
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (RW_DEBUG)
#include <stdint.h>
#include <stdbool.h>

#include "co_utils.h"    // For bit field manipulation

#include "ke_msg.h"      // Kernel message

#include "llc_int.h"    // Internal LLC API
#include "llc_llcp.h"   // Internal LLCP API

#include "hci.h"        // For HCI handler

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
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */



/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


#if (BLE_PERIPHERAL || BLE_CENTRAL)
/// @brief Handle the reception of the vendor specific command llcp discard.
int hci_dbg_llcp_discard_cmd_handler(uint8_t link_id, struct hci_dbg_llcp_discard_cmd const *param, uint16_t opcode)
{
    // structure type for the complete command event
    struct hci_basic_cmd_cmp_evt *event;
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);

    // Set the LLCP discard flag in the LLC environment
    SETB(llc_env_ptr->link_info, LLC_INFO_DBG_LLCP_DISCARD, param->enable);

    // allocate the status event message
    event =  KE_MSG_ALLOC(HCI_CMD_CMP_EVENT , conhdl, HCI_DBG_LLCP_DISCARD_CMD_OPCODE, hci_basic_cmd_cmp_evt);
    event->status = CO_ERROR_NO_ERROR;

    // sends the message
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}
#endif //(BLE_PERIPHERAL || BLE_CENTRAL)



#endif // (RW_DEBUG)

/// @} LLC_DBG
