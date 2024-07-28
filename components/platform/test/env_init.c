/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  env_init.c
  * @brief 
  * @author bingrui.chen
  * @date 2022-1-11
  * @version 1.0
  * @Revision: 
  */
#include <ms_clock_hal.h>
#include <ms1008.h>
#include "ms_pinmux_hal.h"
#include "ms_uart.h"
#include "unity.h"
#include "uart.h"

#include <string.h>

void system_clock_init(void)
{
    SysOscInit_Type osc_init;
    osc_init.hf_osc_src = HF_OSC_CLK_SEL_EXT_OSC24M;
    osc_init.sys_hf_clk_src = HF_CLK_SEL_BYPASS_PLL;
    osc_init.sys_lf_clk_src = LF_CLK_SEL_RC_OSC32K;
    osc_init.ext_osc_div = 1;
    osc_init.pll.mul = 1;
    osc_init.pll.pdiv = 0;
    ms_clock_hal_sys_config_osc(&osc_init);

    SysClockInit_Type clock_init;
    clock_init.sys_clk_src = SYS_CLK_SEL_HF_CLK;
    clock_init.ahb_div = 0;
    clock_init.apb_div = 0;
    clock_init.peri_clk_div = 0;
    ms_clock_hal_config_clock(&clock_init);
}
