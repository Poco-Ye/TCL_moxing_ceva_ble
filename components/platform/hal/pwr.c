/*
 * uart.c
 *
 *  Created on: 2021年12月24日
 *      Author: bingrui.chen
 */

#include <string.h>
#include <ms_clock_hal.h>
#include <ms_pinmux_hal.h>
#include "ms_pwr.h"
#include "log.h"


PWRHandle_Type pwr_handle;


int32_t pwr_init()
{
    pwr_handle.sysctl_instance = SYS_CTRL;	
    pwr_handle.pwr_config.pwr_mode = PWR_MODE_DEEPSLEEP;
    pwr_handle.pwr_config.ds_wkup_sel = PWR_WAKEUP_SEL_GPIO_PORT | PWR_WAKEUP_SEL_BLE_TIMER;
    pwr_handle.pwr_config.pull_type_p00_p15 = PAD0_15_PULLTYPE(PAD1, PULL_UP) |
            PAD0_15_PULLTYPE(PAD2, PULL_UP) | PAD0_15_PULLTYPE(PAD3, PULL_UP) |
            PAD0_15_PULLTYPE(PAD4, PULL_UP) | PAD0_15_PULLTYPE(PAD5, PULL_UP) | 
            PAD0_15_PULLTYPE(PAD6, PULL_UP);
    pwr_handle.pwr_config.pull_type_p16_p26 = PAD16_26_PULLTYPE(PAD17, PULL_DOWN) |
            PAD16_26_PULLTYPE(PAD18, PULL_DOWN) | PAD16_26_PULLTYPE(PAD19, PULL_DOWN) |
            PAD16_26_PULLTYPE(PAD20, PULL_DOWN) | PAD16_26_PULLTYPE(PAD21, PULL_DOWN);

    return ms_pwr_init(&pwr_handle);
}

int32_t pwr_exit()
{
    ms_pwr_exit(&pwr_handle);
}


int32_t pwr_config_wkup_sel(PWR_WAKEUP_SEL_Type pwr_wkup_sel)
{
    return ms_pwr_config_wkup_sel(pwr_wkup_sel);
}

int32_t pwr_config_sleep_mode(PWR_MODE_Type pwr_mode)
{
    return ms_pwr_config_sleep_mode(pwr_mode);
}

int32_t pwr_config_pad_pulltype(uint32_t pin, PadPull_Type pull_type)
{
    return STATUS_SUCCESS;
}


int32_t pwr_enter_sleep()
{
    return ms_pwr_enter(&pwr_handle);
}

int32_t pwr_after_wakeup()
{
   return STATUS_SUCCESS;
}

int32_t pwr_get_pre_mode()
{
    return ms_pwr_get_pre_mode(&pwr_handle);
}


uint32_t pwr_getwakeup_resource()
{ 
    uint32_t value;
	
    value = ms_pwr_hal_get_wakup_source(pwr_handle.sysctl_instance);
	return value;
}

#if 0
void WAKEUP_IRQHandler(void)
{
    ms_pwr_wakeup_irq_handler(&pwr_handle);
}
#endif

