/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_clock_hal.c
 * @brief Header file of Uart  module.
 * @author bingrui.chen
 * @date   2021-12-16
 * @version 1.0
 * @Revision:
 */

#include "ms_clock_hal.h"
#include "sys_tick.h"

//#define CLK_16M (16000000UL)
#define CLK_12M (12000000UL)
#define CLK_24M (24000000UL)
#define CLK_32K (32000UL)

#define CLK_DEFAULT CLK_24M

static uint32_t system_core_clock = CLK_DEFAULT;
static uint32_t sys_hf_clk = CLK_DEFAULT;
static uint32_t sys_lf_clk = CLK_32K;

int32_t ms_clock_hal_sys_config_osc(SysOscInit_Type *p_osc_init)
{
    if (!p_osc_init)
    {
        return STATUS_ERROR;
    }

    sys_hf_clk = CLK_DEFAULT;

    /*config the hf_osc_clk*/
    ms_clock_hal_hf_osc_sel(p_osc_init->hf_osc_src);
    if (p_osc_init->hf_osc_src == HF_OSC_CLK_SEL_EXT_OSC24M)
    {
        sys_hf_clk = CLK_24M;
    }

    if (p_osc_init->sys_hf_clk_src == HF_CLK_SEL_PLL_CLK)
    {
        ms_clock_hal_pll_enable();
        ms_clock_hal_pll_config(p_osc_init->pll.pdiv, p_osc_init->pll.mul);
        while(!ms_clock_hal_pll_is_locked());
        /*todo: need to calculate the sys_hf_clk by the  mul and pdiv in PLL*/
//        sys_hf_clk = (sys_hf_clk * p_osc_init->pll.mul) >> p_osc_init->pll.pdiv;
    }
    else
    {
        ms_clock_hal_pll_disable();
    }

    /*config the sys_hf_clk*/
    ms_clock_hal_hf_clk_sel(p_osc_init->sys_hf_clk_src);

    /* config the sys_lf_clk*/
    if (p_osc_init->sys_lf_clk_src == LF_CLK_SEL_EXT_OSC24M)
    {
        ms_clock_hal_lf_clk_sel(LF_CLK_SEL_EXT_OSC24M);
        ms_clock_hal_set_ext_high_osc_div(p_osc_init->ext_osc_div);
        /*todo: wait to lock and toggle*/
        ms_clock_hal_set_ext_high_osc_div_toggle();

        sys_lf_clk = CLK_24M >> p_osc_init->ext_osc_div;
    }
    else
    {
        ms_clock_hal_lf_clk_sel(LF_CLK_SEL_RC_OSC32K);
        sys_lf_clk = CLK_32K;
    }

    return STATUS_SUCCESS;
}

int32_t ms_clock_hal_config_clock(SysClockInit_Type *p_sys_clock_init)
{
    if (!p_sys_clock_init)
    {
        return STATUS_ERROR;
    }

    /*select the sys_clk source*/
    ms_clock_hal_sys_clk_sel(p_sys_clock_init->sys_clk_src);
    if (p_sys_clock_init->sys_clk_src == SYS_CLK_SEL_HF_CLK)
    {
        MS_CLOCK_HAL_CLK_ENABLE_HF_CLK();
        system_core_clock = sys_hf_clk;
    }
    else
    {
        MS_CLOCK_HAL_CLK_ENABLE_LF_CLK();
        system_core_clock = sys_lf_clk;
    }

    /*set the ahb division*/
    ms_clock_hal_set_ahb_div(p_sys_clock_init->ahb_div);

    /*set the apb division*/
    ms_clock_hal_set_apb_div(p_sys_clock_init->apb_div);

    /*set the peri_clk division*/
    ms_clock_hal_set_periph_pre_div(p_sys_clock_init->peri_clk_div);

    system_core_clock = system_core_clock >> p_sys_clock_init->ahb_div;

    ms_sys_timer_config_systick(system_core_clock / MS_TICK_PER_SECOND);

    return STATUS_SUCCESS;
}

uint32_t ms_clock_hal_get_sys_clk_freq(void)
{
    uint32_t ahb_div = ms_clock_hal_get_ahb_div();
    return (system_core_clock << ahb_div);
}

uint32_t ms_clock_hal_get_peri_clk_freq(void)
{
    uint32_t sys_clk = ms_clock_hal_get_sys_clk_freq();
    return (sys_clk >> ms_clock_hal_get_periph_pre_div());
}

uint32_t ms_clock_hal_get_hclk_freq(void)
{
    return system_core_clock;
}

uint32_t ms_clock_hal_get_lf_clk_freq(void)
{
    return sys_lf_clk;
}

