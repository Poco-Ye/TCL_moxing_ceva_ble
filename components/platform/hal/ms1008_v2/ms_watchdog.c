/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_watchdog_hal.c
 * @brief c source  file of watchdog  module.
 * @author haijun.mai
 * @date   2022-01-04
 * @version 1.0
 * @Revision
 */

#include <ms1008.h>
#include "ms_watchdog_hal.h"
#include "ms_watchdog.h"
#include "ms_watchdog_regs.h"
#include "ms_clock_hal.h"
#include "ms_sys_ctrl_regs.h"
#include "ms_interrupt.h"
#include <stddef.h>
#include "ms_sys_wild_hal.h"
/**
 * @brief enable watchdog interrupt
 * @param  WatchdogHandle_Type *watchdog: 
 * @retval None
 */
void ms_watchdog_enable_cpu_interrupt(WatchdogHandle_Type *watchdog) {
	INTERRUPT_ENABLE_IRQ(watchdog->irq);
}

/**
 * @brief disable watchdog interrupt
 * @param  WatchdogHandle_Type *watchdog: 
 * @retval None
 */
void ms_watchdog_disable_cpu_interrupt(WatchdogHandle_Type *watchdog) {
	INTERRUPT_DISABLE_IRQ(watchdog->irq);
}

/**
 * @brief watchdog module init
 * @param  WatchdogHandle_Type *watchdog: 
 * @retval None
 */
void ms_watchdog_init(WatchdogHandle_Type *watchdog) {
	if (NULL == watchdog) {
		return;
	}
	/*initial uart gpio pinmux, clock and interrupt setting*/
	if (watchdog->p_callback && watchdog->p_callback->init_callback) {
		watchdog->p_callback->init_callback(watchdog);
	}

	MS_SYS_HAL_ENABLE_WDT(); 

	//set watchdog time out value
	ms_wdg_set_timeout_period_hal(watchdog->instance, watchdog->init.period);

	ms_wdg_set_reset_pulse_length_hal(watchdog->instance,
			watchdog->init.reset_pulse_length);

	ms_wdg_set_rsp_mode_hal(watchdog->instance, watchdog->init.response_mode);

	ms_wdg_enable_hal(watchdog->instance);

	ms_wdg_restart_hal(watchdog->instance);
}

/**
 * @brief get watchdog count value
 * @param  WatchdogHandle_Type *watchdog: 
 * @retval None
 */
int32_t ms_watchdog_get_cnt_value(WatchdogHandle_Type *watchdog) {
	return (ms_wdg_get_cnt_val_hal(watchdog->instance));
}

/**
 * @brief watchdog  feed dog
 * @param  WatchdogHandle_Type *watchdog: 
 * @retval None
 */
void ms_watchdog_feed(WatchdogHandle_Type *watchdog) {
	ms_wdg_restart_hal(watchdog->instance);
}

/**
 * @brief watchdog  clear interrupt status
 * @param  WatchdogHandle_Type *watchdog: 
 * @retval None
 */
void ms_watchdog_interrupt_clear(WatchdogHandle_Type *watchdog) {
	ms_wdg_clear_interrupt_status_hal(watchdog->instance);
}

/**
 * @brief watchdog  handler function
 * @param  WatchdogHandle_Type *watchdog: 
 * @retval None
 */
void ms_watchdog_irq_handler(WatchdogHandle_Type *watchdog) {

	if (watchdog->p_callback == NULL) {
		watchdog->error_code |= WATCHDOG_ERROR_INVALID_CALLBACK;
		return;
	}
	watchdog->p_callback->watchdog_reach_callback(watchdog);
}

int32_t ms_watchdog_deinit(WatchdogHandle_Type *watchdog) {

	/* DeInit the low level hardware */
	if (watchdog->p_callback && watchdog->p_callback->deinit_callback) {
		watchdog->p_callback->deinit_callback(watchdog);
	}
	return 0;
}

/**
 * @brief watchdog  disable
 * @param  WatchdogHandle_Type *watchdog: 
 * @retval None
 */
void ms_watchdog_close(WatchdogHandle_Type *watchdog) {
	ms_wdg_close_hal(watchdog->instance);
}