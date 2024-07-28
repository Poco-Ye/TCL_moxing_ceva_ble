/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_watchdog_hal.h
 * @brief Header file of watchdog  module.
 * @author haijun.mai
 * @date   2022-01-05
 * @version 1.0
 * @Revision
 */

#ifndef MS_WATCHDOG_H_
#define MS_WATCHDOG_H_

#include <ms1008.h>
#include "ms_watchdog_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WATCHDOG_ERROR_INVALID_CALLBACK  0x00000001U   /*!< Invalid Callback error  */

typedef struct {
	uint32_t period; /*us*/
	uint32_t reset_pulse_length;
	uint8_t response_mode;
} WatchdogInit_Type;

struct __WatchdogHandle_Type;

/**
 * @brief  watchdog callback handle Structure definition
 */
typedef struct {
	void (*error_callback)(struct __WatchdogHandle_Type *watchdog);
	void (*init_callback)(struct __WatchdogHandle_Type *watchdog);
	void (*deinit_callback)(struct __WatchdogHandle_Type *watchdog);
	void (*watchdog_reach_callback)(struct __WatchdogHandle_Type *watchdog);
} WatchdogCallback_Type;

typedef struct __WatchdogHandle_Type {
	Watchdog_Type *instance;
	WatchdogInit_Type init;/*!< Timer communication parameters      */
	uint32_t error_code; /*!< TIMER Error code*/
	IRQn_Type irq;
	WatchdogCallback_Type *p_callback;
} WatchdogHandle_Type;

extern void ms_watchdog_enable_cpu_interrupt(WatchdogHandle_Type *watchdog);
extern void ms_watchdog_disable_cpu_interrupt(WatchdogHandle_Type *watchdog);
extern void ms_watchdog_init(WatchdogHandle_Type *watchdog);
extern void ms_watchdog_feed(WatchdogHandle_Type *watchdog);
extern void ms_watchdog_interrupt_clear(WatchdogHandle_Type *watchdog);
extern int32_t ms_watchdog_deinit(WatchdogHandle_Type *watchdog);
extern void ms_watchdog_irq_handler(WatchdogHandle_Type *watchdog);
extern void ms_watchdog_close(WatchdogHandle_Type *watchdog);
#ifdef __cplusplus
}
#endif

#endif /* MS_WATCHDOG_H_ */

