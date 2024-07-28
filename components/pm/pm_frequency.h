/**
 * Copyright © 2022 by MooreSilicon. All rights reserved
 * @file  pm_frequency.h
 * @brief power management frequency module
 * @author haimin.zhang
 * @date 2022-04-12
 * @version 1.0
 * @Revision
 */

#ifndef _PM_FREQUENCY_H_
#define _PM_FREQUENCY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief  power management frequency type
 */
typedef enum
{
    PM_CPU_FREQ = 0,
    PM_AHB_FREQ,
    PM_APB_FREQ,
}PM_FREQ_Type;

// Power management frequency configeration structure
typedef struct pm_freq_config
{
    // CPU frequency configeration
    uint32_t cpu_freq;
    // AHB frequency configeration
    uint32_t ahb_freq;
    // APB frequency configeration
    uint32_t apb_freq;
} pm_freq_config_t;

int32_t pm_set_frequency(PM_FREQ_Type type, uint32_t freq);
uint32_t pm_get_frequency(PM_FREQ_Type type);

#endif // _PM_FREQUENCY_H_

