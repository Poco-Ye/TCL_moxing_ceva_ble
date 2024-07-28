/**
 * Copyright © 2022 by MooreSilicon. All rights reserved
 * @file  pm_battery.h
 * @brief power management battery module
 * @author haimin.zhang
 * @date 2022-04-12
 * @version 1.0
 * @Revision
 */

#ifndef _PM_BATTERY_H_
#define _PM_BATTERY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief  power management get battery type
 */
typedef enum
{
    PM_BATT_LEVEL = 0,
    PM_BATT_VOLTAGE,
}PM_Battery_Type;

// Power management battery structure
typedef struct pm_battery
{
    // Battery level
    uint32_t level;
    // Battery voltage
    uint32_t voltage;
} pm_battery_t;

int32_t pm_get_batterylevel(PM_Battery_Type type);
void pm_low_battery(uint32_t type);

#endif // _PM_BATTERY_H_

