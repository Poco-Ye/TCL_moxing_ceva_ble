/**
****************************************************************************************
*
* @file rwble.c
*
* @brief RWBLE core interrupt handler
*
* Copyright (C) RivieraWaves 2009-2016
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"      // stack configuration

#include <string.h>           // for mem* functions
#include "co_version.h"
#include "co_math.h"
#include "rwble.h"            // BLE API definition
#include "rwip.h"             // stack main module

#include "ble_util_buf.h"     // BLE EM buffer management
#include "lld.h"              // link layer driver definition
#include "llc.h"              // link layer controller definition
#include "llm.h"              // link layer manager definition

#if (BLE_ISO_PRESENT)
#include "lli.h"              // Link Layer ISO definition
#endif // (BLE_ISO_PRESENT)

#include "ke_event.h"         // kernel event definition

#include "sch_arb.h"          // Scheduling Arbiter
#include "sch_prog.h"         // Scheduling Programmer

#include "dbg.h"              // debug definitions

#include "reg_blecore.h"      // BLE Core registers


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void ROM_VT_FUNC(rwble_init)(uint8_t init_type)
{
    // Initialize buffer management system
    ble_util_buf_init(init_type);

    // LLD block does nothing at IP INIT state
    if (init_type > RWIP_INIT)
    {
        // Initialize the Link Layer Driver
        lld_init(init_type);
    }

    #if(BLE_CENTRAL || BLE_PERIPHERAL)
    // Initialize the Link Layer Controller
    llc_init(init_type);
    #endif // (BLE_CENTRAL || BLE_PERIPHERAL)

    // Initialize the Link Layer Manager
    llm_init(init_type);

    #if (BLE_ISO_PRESENT)
    // Initialize the Link Layer ISO
    lli_init(init_type);
    #endif // (BLE_ISO_PRESENT)
}

#if BT_DUAL_MODE
bool rwble_activity_ongoing_check(void)
{
    // check that a BLE activity is ongoing (advertising, scan, initiating, connection)
    return llm_activity_ongoing_check();
}
#endif //BT_DUAL_MODE

__BLEIRQ void ROM_VT_FUNC(rwble_isr)(void)
{
    DBG_SWDIAG(ISR, BLE, 1);

    // Check BLE interrupt status and call the appropriate handlers
    uint32_t irq_stat      = ble_intstat0_get();

    // Error interrupt should be checked first
    if (irq_stat & BLE_ERRORINTSTAT_BIT)
    {
        // Clear the interrupt
        ble_intack0_errorintack_clearf(1);
        ASSERT_INFO(0, ble_errortypestat_get(), (ble_errortypestat_get()>>16));
    }

    #if (BLE_BIS | BLE_CIS)
    // Hopping scheme computation
    if (irq_stat & BLE_HOPINTACK_BIT)
    {
        // Clear the interrupt
        ble_intack0_hopintack_clearf(1);

        // Handle end of hopping HW accelerator
        lld_iso_hop_isr();
    }
    #endif // (BLE_BIS | BLE_CIS)


    DBG_SWDIAG(ISR, BLE, 0);
}

///@} RWBTINT
