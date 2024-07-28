/**
 * Copyright © 2022 by MooreSilicon. All rights reserved
 * @file  pm_callback.h
 * @brief power management sleep callback module
 * @author haimin.zhang
 * @date 2022-04-12
 * @version 1.0
 * @Revision
 */

#ifndef _PM_CALLBACK_H_
#define _PM_CALLBACK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
/*
void ms_pwr_register_user_sleep_callback(int32_t (*sleep_cb)())
{
    PWR_Config_Type *p_pwr_cfg = (PWR_Config_Type *)&g_pwr_cfg;
    p_pwr_cfg->pwr_before_sleep_callback = sleep_cb;
}

void ms_pwr_register_user_wakeup_callback(int32_t (*wakeup_cb)())
{
    PWR_Config_Type *p_pwr_cfg = (PWR_Config_Type *)&g_pwr_cfg;
    p_pwr_cfg->pwr_after_wakeup_callback = wakeup_cb;
}
*/

typedef void (*app_sleep_cb) (void);

// Sleep callback structure
typedef struct pm_sleep_cb
{
    // Pointer to next callback in sleep callback list
    struct pm_sleep_cb_t * p_next;
    // Callback to execute
    app_sleep_cb cb;
} pm_sleep_cb_t;


int32_t pm_register_enter_sleep_callback(app_sleep_cb func);
int32_t pm_deregister_enter_sleep_callback(app_sleep_cb func);
int32_t pm_register_exit_sleep_callback(app_sleep_cb func);
int32_t pm_deregister_exit_sleep_callback(app_sleep_cb func);

int32_t pm_register_enter_dsleep_callback(app_sleep_cb func);
int32_t pm_deregister_enter_dsleep_callback(app_sleep_cb func);
int32_t pm_register_exit_dsleep_callback(app_sleep_cb func);
int32_t pm_deregister_exit_dsleep_callback(app_sleep_cb func);

#endif // _PM_CALLBACK_H_

