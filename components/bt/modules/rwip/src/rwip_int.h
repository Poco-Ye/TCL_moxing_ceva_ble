/**
****************************************************************************************
*
* @file rwip_int.h
*
* @brief RW IP internal SW main module
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/
#ifndef _RWIP_INT_H_
#define _RWIP_INT_H_

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @brief Entry points of the RW IP stacks/modules
 *
 * This module contains the primitives that allow an application accessing and running the
 * RW IP protocol stacks / modules.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"          // stack configuration

#include <stdint.h>               // standard integer definitions
#include <stdbool.h>              // standard boolean definitions


/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/// RWIP Environment structure
struct rwip_env_tag
{
    /// Prevent sleep bit field
    uint32_t          prevent_sleep;

    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /// Arbiter target timer  (integer part, in half slots)
    rwip_time_t       timer_arb_target;
    /// Alarm target timer (integer part, in half slots)
    rwip_time_t       timer_alarm_target;
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /// Common target timer (in half slots)
    rwip_time_t       timer_co_target;
    /// Last Sampled time (used for time conversion)
    rwip_time_t       last_samp_time;

    #if (BLE_EMB_PRESENT)
    /// BLE channel assessment data
    struct rwip_ch_assess_data_ble ch_assess_ble;
    #endif //BLE_EMB_PRESENT
    #if (BT_EMB_PRESENT)
    /// BT channel assessment data
    struct rwip_ch_assess_data_bt ch_assess_bt;
    #endif //BT_EMB_PRESENT

    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /// Maximum sleep duration (in LP cycles, depends on Low power clock frequency)
    uint32_t          sleep_dur_max;
    /// Contains sleep duration accumulated timing error (32kHz: 1/2 half us | 32.768kHz: 1/256 half-us)
    uint32_t          sleep_acc_error;
    /// Power_up delay (in LP clock cycle unit, depends on Low power clock frequency)
    uint32_t          lp_cycle_wakeup_delay;
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /// Maximum value of the sleep clock drift (in ppm)
    uint16_t          sleep_clock_drift;
    /// Maximum value of the sleep clock accuracy (@see enum SCA)
    uint8_t           sleep_clock_accuracy;
    /// Maximum value of the active clock drift (in ppm)
    uint8_t           active_clock_drift;
    /// External wake-up support
    bool              ext_wakeup_enable;
    #if (!BLE_ISO_HW_PRESENT)
    /// BTS sampling clock half microseconds residual (0 or 1)
    uint8_t           samp_hus_residual;
    #endif // (!BLE_ISO_HW_PRESENT)

    /// Channel assessment scheme enabled/disabled
    bool ch_ass_en;
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
};


/*
 * GLOBAL DEFINITIONS
 ****************************************************************************************
 */

/// RW SW environment
extern struct rwip_env_tag rwip_env;


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 * Initialization of the RW IP Common core driver
 *
 * @param[in] init_type  Type of initialization (@see enum rwip_init_type)
 */
void rwip_driver_init(uint8_t init_type);


///@} ROOT

#endif // _RWIP_INT_H_
