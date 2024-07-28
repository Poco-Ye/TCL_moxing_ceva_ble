/**
****************************************************************************************
*
* @file lld_iso.c
*
* @brief LLD Isochronous Channel common driver source code
*
* Copyright (C) RivieraWaves 2009-2017
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLDISO
 * @ingroup LLD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"               // stack configuration

#if (BLE_CIS || BLE_BIS)

#include <string.h>
#include "rwip.h"
#include "lld.h"                       // link driver API
#include "lld_int.h"                   // link layer driver internal
#include "lld_int_iso.h"               // LLD Internal API for ISO

#include "sch_alarm.h"                 // Scheduling Alarm
#include "sch_arb.h"                   // Scheduling Arbiter
#include "sch_prog.h"                  // Scheduling Programmer
#include "sch_slice.h"                 // Scheduling Slicer

#include "reg_blecore.h"               // BLE core registers

#include "ke_mem.h"                    // memory allocation

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * ENUMERATIONS
 *****************************************************************************************
 */

/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LLD ISO Environment
struct lld_iso_env_tag lld_iso_env;

/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Request HW to start Hopping scheme computation
 *
 * @param[in] hop_info Hopping input info
 ****************************************************************************************
 */
__STATIC void lld_iso_start_hop_scheme_compute(struct lld_iso_hop_inf* hop_info)
{
    uint16_t chan_identifier;
    // Prevent going to deep sleep during encryption
    rwip_prevent_sleep_set(RW_HOP_CALC_ONGOING);


    // channelIdentifier = (Access Address[31-16]) XOR (Access Address[15-0])
    chan_identifier = (hop_info->acc_code & 0xFFFF) ^ ((hop_info->acc_code>>16) & 0xFFFF);

    // configure HW to start Hopping scheme computation accelerator
    ble_freqselptr_setf(hop_info->em_ptr >> 2);
    ble_freqsel_cs2_seed_pack(chan_identifier, hop_info->evt_cnt);
    ble_freqsel_llchmap0_setf(  (hop_info->chmap[0] <<  0) | (hop_info->chmap[1] <<  8) | (hop_info->chmap[2] << 16)
                              | (hop_info->chmap[3] << 24));
    ble_freqsel_llchmap1_setf(hop_info->chmap[4]);

    // enable ISR
    ble_intcntl0_hopintmsk_setf(1);

    // exectute request
    ble_freqselcntl_pack(hop_info->nse, 1, 1);
}

/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void lld_iso_init(uint8_t init_type)
{
    #if (BLE_CIS)
    // Initialize CIS driver
    lld_ci_init(init_type);
    #endif //(BLE_CIS)

    #if (BLE_BIS)
    // Initialize BIS driver
    lld_bi_init(init_type);
    #endif //(BLE_BIS)

    switch (init_type)
    {
        case RWIP_INIT:
        {
            // Do nothing
        }
        break;

        case RWIP_RST:
        {
            // Do nothing
        }
        // No break

        case RWIP_1ST_RST:
        {
            co_list_init(&lld_iso_env.iso_hop_list);
        }
        break;

        default:
        {
            // Do nothing
        }
        break;
    }
}

void lld_iso_hop_compute(struct lld_iso_hop_inf *p_hop_inf)
{
    bool ready = co_list_is_empty(&lld_iso_env.iso_hop_list);

    ASSERT_ERR(!p_hop_inf->busy);

    // put element at end of the list
    p_hop_inf->busy = true;
    co_list_push_back(&lld_iso_env.iso_hop_list, &(p_hop_inf->hdr));

    // if possible start hopping scheme computation
    if(ready)
    {
        lld_iso_start_hop_scheme_compute(p_hop_inf);
    }
}

void lld_iso_hop_cancel(struct lld_iso_hop_inf *p_hop_inf)
{
    // Check if the hopping is ongoing
    lld_iso_env.iso_hop_cancel = (&p_hop_inf->hdr == co_list_pick(&lld_iso_env.iso_hop_list));

    // Extract from the hopping computation list
    co_list_extract(&lld_iso_env.iso_hop_list, &(p_hop_inf->hdr));

    // Indicate as idle
    p_hop_inf->busy = false;
}

void lld_iso_hop_isr(void)
{
    if(!lld_iso_env.iso_hop_cancel)
    {
        // Get hopping info
        struct lld_iso_hop_inf* p_hop_inf = (struct lld_iso_hop_inf*)co_list_pop_front(&lld_iso_env.iso_hop_list);

        // Prevent going to deep sleep during encryption
        rwip_prevent_sleep_clear(RW_HOP_CALC_ONGOING);
        // Disable ISR
        ble_intcntl0_hopintmsk_setf(0);
        // mark hopping computation done
        p_hop_inf->busy = false;

        // check if their is still something to perform
        if(!co_list_is_empty(&lld_iso_env.iso_hop_list))
        {
            // execute next hopping request
            p_hop_inf = (struct lld_iso_hop_inf*)co_list_pick(&lld_iso_env.iso_hop_list);
            lld_iso_start_hop_scheme_compute(p_hop_inf);
        }
    }
    else
    {
        lld_iso_env.iso_hop_cancel = false;
    }
}
#endif //(BLE_CIS || BLE_BIS)

///@} LLDISO
