/**
 * Copyright © 2022 by MooreSilicon. All rights reserved
 * @file  pm_callback.c
 * @brief power management sleep callback module
 * @author haimin.zhang
 * @date 2022-04-12
 * @version 1.0
 * @Revision
 */

#include <string.h>
#include "pm_sleep.h"
#include "log.h"

int32_t pm_register_enter_sleep_callback(app_sleep_cb func)
{
    return 0;
}

int32_t pm_deregister_enter_sleep_callback(app_sleep_cb func)
{
    return 0;
}

int32_t pm_register_exit_sleep_callback(app_sleep_cb func)
{
    return 0;
}
int32_t pm_deregister_exit_sleep_callback(app_sleep_cb func)
{
    return 0;
}
int32_t pm_register_enter_dsleep_callback(app_sleep_cb func)
{
    return 0;
}
int32_t pm_deregister_enter_dsleep_callback(app_sleep_cb func)
{
    return 0;
}
int32_t pm_register_exit_dsleep_callback(app_sleep_cb func)
{
    return 0;
}
int32_t pm_deregister_exit_dsleep_callback(app_sleep_cb func)
{
    return 0;
}

