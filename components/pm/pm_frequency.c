/**
 * Copyright © 2022 by MooreSilicon. All rights reserved
 * @file  pm_frequency.c
 * @brief power management frequency module
 * @author haimin.zhang
 * @date 2022-04-12
 * @version 1.0
 * @Revision
 */

#include "pm_sleep.h"
#include "log.h"

int32_t pm_set_frequency(PM_FREQ_Type type, uint32_t freq)
{
    switch (type)
    {
        case PM_CPU_FREQ:
            g_pm_status.freq.cpu_freq = freq;
            break;

        case PM_AHB_FREQ:
            g_pm_status.freq.ahb_freq = freq;
            break;

        case PM_APB_FREQ:
            g_pm_status.freq.apb_freq = freq;
            break;

        default:
            break;
    }

    return 0;
}

uint32_t pm_get_frequency(PM_FREQ_Type type)
{
    switch (type)
    {
        case PM_CPU_FREQ:
            return g_pm_status.freq.cpu_freq;

        case PM_AHB_FREQ:
            return g_pm_status.freq.ahb_freq;

        case PM_APB_FREQ:
            return g_pm_status.freq.apb_freq;

        default:
            break;
    }

    return 0;
}

