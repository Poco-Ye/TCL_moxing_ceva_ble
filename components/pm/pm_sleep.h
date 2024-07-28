/**
 * Copyright © 2022 by MooreSilicon. All rights reserved
 * @file  pm_sleep.h
 * @brief power management sleep module
 * @author haimin.zhang
 * @date 2022-04-12
 * @version 1.0
 * @Revision
 */

#ifndef _PM_SLEEP_H_
#define _PM_SLEEP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "pm_callback.h"
#include "pm_lock.h"
#include "pm_frequency.h"
#include "pm_battery.h"

//#define PM_DEBUG_BLE_WKUP

/**
 * @brief  power management system wakeup type
 */
typedef enum
{
    PM_PWR_MODE_POWERDOWN = 0,
    PM_PWR_MODE_DEEPSLEEP,
    PM_PWR_MODE_SLEEP,
    PM_PWR_MODE_ACTIVE,
    PM_PWR_MODE_IDLE,
}PM_PWR_MODE_Type;

/**
 * @brief  power management system wakeup type
 */
typedef enum
{
    PM_WKUP_DEFAULT = 0,
    PM_WKUP_GPIO,
    PM_WKUP_UART2,
    PM_WKUP_RTC,
    PM_WKUP_BLE,
}PM_WKUP_Type;

// Power management status structure
typedef struct pm_status
{
    // The current battery level/voltage
    pm_battery_t battery;
    // Power on pre mode
    uint32_t pre_mode;
    // Wakeup resource
    uint32_t wakeup_resource;
    // BLE sleep time
    uint32_t ble_sleep_time;
    // BLE enter sleep timetick
    uint32_t ble_enter_sleep_timetick;
    // Wake lock status
    pm_lock_t lock[PM_LOCK_MAX];
    // Frequency configeration
    pm_freq_config_t freq;
    // Enter sleep callback quene
    pm_sleep_cb_t *enter_sleep_callback_q;
    // Exit sleep callback quene
    pm_sleep_cb_t *exit_sleep_callback_q;
    // Enter deep sleep callback quene
    pm_sleep_cb_t *enter_dsleep_callback_q;
    // Exit deep sleep callback quene
    pm_sleep_cb_t *exit_dsleep_callback_q;
} pm_status_t;

extern pm_status_t g_pm_status;

void pm_sys_init(void);
void pm_sys_enter_sleep(void);
void pm_sys_exit_sleep(void);
void pm_sys_enter_dsleep(void);
void pm_sys_exit_dsleep(void);
PM_PWR_MODE_Type pm_sys_get_pre_mode(void);
PM_WKUP_Type pm_sys_get_wakeup_resource(void);

#endif // _PM_SLEEP_H_

