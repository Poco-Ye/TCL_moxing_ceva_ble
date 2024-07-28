/**
 * Copyright © 2022 by MooreSilicon. All rights reserved
 * @file  pm_lock.c
 * @brief power management lock module
 * @author haimin.zhang
 * @date 2022-04-12
 * @version 1.0
 * @Revision
 */

#include "pm_sleep.h"
#include "log.h"

int32_t pm_lock_create(void)
{
    g_pm_status.lock[PM_LOCK_NONE].name = "NONE";
    g_pm_status.lock[PM_LOCK_NONE].count = 0;
    g_pm_status.lock[PM_LOCK_NONE].timetick = 0;
    g_pm_status.lock[PM_LOCK_CPU].name = "CPU";
    g_pm_status.lock[PM_LOCK_CPU].count = 0;
    g_pm_status.lock[PM_LOCK_CPU].timetick = 0;

    return 0;
}

int32_t pm_lock_acquire(PM_LOCK_Type locktype)
{
    switch (locktype)
    {
        case PM_LOCK_CPU:
            // Save the time of 1st acquire the lock, for check lock timeout
            if (!g_pm_status.lock[locktype].count)
                g_pm_status.lock[locktype].timetick = 0;

            g_pm_status.lock[locktype].count++;
            break;

        default:
            break;
    }

    return 0;
}

int32_t pm_lock_release(PM_LOCK_Type locktype)
{
    switch (locktype)
    {
        case PM_LOCK_CPU:
            g_pm_status.lock[locktype].count--;
            if (g_pm_status.lock[locktype].count < 0)
            {
                MS_LOGI(MS_DRIVER, "pm_lock_release ERROR!\r\n");
                g_pm_status.lock[locktype].count = 0;
            }

            // Clean the acquire time
            if (!g_pm_status.lock[locktype].count)
                g_pm_status.lock[locktype].timetick = 0;
            break;

        default:
            break;
    }

    return 0;
}

int32_t pm_lock_delete(void)
{
    return 0;
}

