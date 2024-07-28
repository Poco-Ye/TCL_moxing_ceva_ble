/**
 * Copyright © 2022 by MooreSilicon. All rights reserved
 * @file  pm_battery.c
 * @brief power management battery module
 * @author haimin.zhang
 * @date 2022-04-12
 * @version 1.0
 * @Revision
 */

#include "pm_sleep.h"
#include "log.h"

int32_t pm_get_batterylevel(PM_Battery_Type type)
{
    g_pm_status.battery.voltage = 285;

    if (g_pm_status.battery.voltage > 300)
        g_pm_status.battery.level = 100;
    else if (g_pm_status.battery.voltage < 200)
        g_pm_status.battery.level = 0;
    else
        g_pm_status.battery.level = g_pm_status.battery.voltage - 200;

    switch (type)
    {
        case PM_BATT_LEVEL:
            return g_pm_status.battery.level;

        case PM_BATT_VOLTAGE:
            return g_pm_status.battery.voltage;

        default:
            break;
    }

    return 0;
}

void pm_low_battery(uint32_t type)
{
    // TO DO
    return;
}



