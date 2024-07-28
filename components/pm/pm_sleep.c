/**
 * Copyright © 2022 by MooreSilicon. All rights reserved
 * @file  pm_sleep.c
 * @brief power management sleep module
 * @author haimin.zhang
 * @date 2022-04-12
 * @version 1.0
 * @Revision
 */

#include "pm_sleep.h"
#include "pwr.h"
#include "flash.h"
#include "gpio.h"
#include "log.h"
#include "pm_imp.h"

extern void ble_sleep_enter(void);
extern void ble_deepsleep_clear(void);
extern uint32_t ble_deepsleep_statusget(void);

pm_status_t g_pm_status;

static int32_t pm_sys_cfg_before_sleep(void)
{
    // check the callback function pointer list, to be added

    // enable wakeup sleep source
    // INTERRUPT_ENABLE_IRQ(GPIO_IRQn);
    // INTERRUPT_ENABLE_IRQ(UART2_IRQn);
    // INTERRUPT_ENABLE_IRQ(RTC_IRQn);
    MS_LOGI(MS_POWERMANAGE, "pm_sys_cfg_before_sleep\r\n");
    return 0;
}

static int32_t pm_sys_cfg_after_wkup(void)
{
    // disable wakeup sleep source
    //INTERRUPT_DISABLE_IRQ(GPIO_IRQn);
    //INTERRUPT_DISABLE_IRQ(UART2_IRQn);
    //INTERRUPT_DISABLE_IRQ(RTC_IRQn);

    // check the callback function pointer list, to be added
    MS_LOGI(MS_POWERMANAGE,"pm_sys_cfg_after_wkup over!!!");
    return 0;
}

void pm_sys_init(void)
{
#ifdef PM_DEBUG_BLE_DEEP_SLEEP
    g_sleep_data.is_allowed_to_enter_sleep = 0;
#endif

    pwr_init();
    pwr_register_user_sleep_cb(pm_sys_cfg_before_sleep);
    pwr_register_flash_sleep_cb(flash_enter_deepsleep_mode);
    pwr_register_gpio_sleep_cb(gpio_enter_deep_sleep_callback);
    pwr_register_user_wakeup_cb(pm_sys_cfg_after_wkup);

    g_pm_status.battery.level = 85;
    g_pm_status.battery.voltage = 285;
    g_pm_status.pre_mode = pwr_get_pre_mode();
    g_pm_status.wakeup_resource = pwr_getwakeup_resource();
    g_pm_status.ble_sleep_time = 32000 * 10;
    g_pm_status.ble_enter_sleep_timetick = 0;
    g_pm_status.freq.cpu_freq = 0;
    g_pm_status.freq.ahb_freq = 0;
    g_pm_status.freq.apb_freq = 0;
    g_pm_status.enter_sleep_callback_q = NULL;
    g_pm_status.exit_sleep_callback_q = NULL;
    g_pm_status.enter_dsleep_callback_q = NULL;
    g_pm_status.exit_dsleep_callback_q = NULL;
    pm_lock_create();

    MS_LOGI(MS_POWERMANAGE, "pm_sys_init - g_pm_status.pre_mode %d\r\n", g_pm_status.pre_mode);
    MS_LOGI(MS_POWERMANAGE, "pm_sys_init - g_pm_status.wakeup_resource %d\r\n", g_pm_status.wakeup_resource);

    return;
}

void pm_sys_enter_sleep(void)
{
    //MS_LOGI(MS_POWERMANAGE, "pm_sys_enter_sleep\r\n");

    return;
}

void pm_sys_exit_sleep(void)
{
    //MS_LOGI(MS_POWERMANAGE, "pm_sys_exit_sleep\r\n");

    return;
}

void pm_sys_enter_dsleep(void)
{
    MS_LOGI(MS_POWERMANAGE, "pm_sys_enter_dsleep\r\n");

    // calculate how long time can sleep / store ble varaibles / ble sleep
    // configure the ble sleep time
#ifdef BT_SUPPORT
    ble_sleep_enter();
    MS_LOGI(MS_POWERMANAGE, "RWIP deep status: 0x%x\r\n", ble_deepsleep_statusget());
    //MS_LOGI(MS_POWERMANAGE, "wakup resource regiser %x\r\n", SYS_CTRL->WKUP_CSR);
#endif
    pwr_enter_sleep();

    return;
}

void pm_sys_exit_dsleep(void)
{
    MS_LOGI(MS_POWERMANAGE, "pm_sys_exit_dsleep\r\n");

    pwr_exit();
#ifdef BT_SUPPORT
    // ble ip recover to active register
    ble_deepsleep_clear();
    MS_LOGI(MS_POWERMANAGE, "RWIP deep status: 0x%x\r\n", ble_deepsleep_statusget());

    // ble connect status recover
#endif
    return;
}

PM_PWR_MODE_Type pm_sys_get_pre_mode(void)
{
    g_pm_status.pre_mode = pwr_get_pre_mode();
    return g_pm_status.pre_mode;
}

PM_WKUP_Type pm_sys_get_wakeup_resource(void)
{
    g_pm_status.wakeup_resource = pwr_getwakeup_resource();
    return g_pm_status.wakeup_resource;
}

