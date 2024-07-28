/**
 * Copyright © 2022 by MooreSilicon. All rights reserved
 * @file  pm_lock.h
 * @brief power management lock module
 * @author haimin.zhang
 * @date 2022-04-12
 * @version 1.0
 * @Revision
 */

#ifndef _PM_LOCK_H_
#define _PM_LOCK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief  power management lock type
 */
typedef enum
{
    PM_LOCK_NONE = 0,
    PM_LOCK_CPU,
    PM_LOCK_MAX,
}PM_LOCK_Type;

// Power management lock structure
typedef struct pm_lock
{
    // Wake lock name
    char* name;
    // Wake lock acquire count
    uint32_t count;
    // Save the wack lock acquire time
    uint32_t timetick;
} pm_lock_t;

int32_t pm_lock_create(void);
int32_t pm_lock_acquire(PM_LOCK_Type locktype);
int32_t pm_lock_release(PM_LOCK_Type locktype);
int32_t pm_lock_delete(void);

#endif // _PM_LOCK_H_

