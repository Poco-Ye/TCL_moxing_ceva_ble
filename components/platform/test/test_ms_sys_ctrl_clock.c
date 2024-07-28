/*
 * ms_sys_ctrl.c
 *
 *  Created on: 2021年12月16日
 *      Author: bingrui.chen
 */

#include "ms_clock_hal.h"
#include "unity.h"

void setUp(void)
{

}

void tearDown(void)
{

}


void test_ms_ms_clock_hal_clock_config_osc(void)
{

}

#if 0
void main(void)
{

    MS_CLOCK_HAL_CLK_ENABLE_TIMERx(1);
    if(MS_CLOCK_HAL_CLK_IS_ENABLE_UART0())
    {
        return;
    }

    if(MS_CLOCK_HAL_CLK_IS_ENABLE_TIMERx(2))
    {
        return;
    }

    MS_CLOCK_HAL_CLK_ENABLE_I2S0();
    MS_CLOCK_HAL_CLK_ENABLE_IR_SAMPLE();
}
#endif
