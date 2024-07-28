/*
 * watchdog.c
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#include <ms_clock_hal.h>
#include "ms_watchdog.h"
#include <stddef.h>




WatchdogHandle_Type watchdog_handle;


void watchdog_reach_calllback(WatchdogHandle_Type *watchdog)
{
     ms_watchdog_interrupt_clear(watchdog);
}



void watchdog_init_calllback(WatchdogHandle_Type *watchdog)
{
    /*config the watchdog clock*/
    MS_CLOCK_HAL_CLK_ENABLE_WDT();
    ms_clock_hal_set_wdt_div(0);
	
   // ms_clock_hal_peripheral_clk_div_toggle(TIMER0_DIV_TOG);

    /*enable the interrupt*/
    ms_watchdog_enable_cpu_interrupt(watchdog);
}

void watchdog_deinit_calllback(WatchdogHandle_Type *watchdog)
{
    /*disable the interrupt*/
    ms_watchdog_disable_cpu_interrupt(watchdog);

    /*config the watchdog clock*/
    MS_CLOCK_HAL_CLK_DISABLE_WDT();
}

WatchdogCallback_Type  watchdog_callback =
{
       .init_callback = watchdog_init_calllback,
	.deinit_callback = watchdog_deinit_calllback,
	.watchdog_reach_callback = watchdog_reach_calllback,
};

int32_t watchdog_init(void)
{
    watchdog_handle.instance = WATCHDOG;
    watchdog_handle.init.period = USER7_OR_8M;
    watchdog_handle.init.reset_pulse_length = USER6_OR_4M;
    watchdog_handle.init.response_mode = RESPONSE_MODE_GENERATE_A_SYSTEM_RESET;
    //watchdog_handle.init.response_mode = RESPONSE_MODE_GENERATE_INTERRUPT_BEFORE_SYSTEM_RESET;
  
    watchdog_handle.p_callback = &watchdog_callback;
    watchdog_handle.irq = WDG_IRQn;
	
    ms_watchdog_init(&watchdog_handle);

     return STATUS_SUCCESS;
}

int32_t watchdog_feed(void)
{
	ms_watchdog_feed(&watchdog_handle);
	return STATUS_SUCCESS;
}





int32_t watchdog_deinit(void)
{
	ms_watchdog_deinit(&watchdog_handle);
       return STATUS_SUCCESS;
}

void WDG_IRQHandler(void)
{
    ms_watchdog_irq_handler(&watchdog_handle);
}


int32_t watchdog_close(void)
{
	ms_watchdog_close(&watchdog_handle);
	return STATUS_SUCCESS;
}


