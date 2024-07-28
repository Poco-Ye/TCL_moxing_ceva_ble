/**
 * Copyright ? 2022 by MooreSilicon.All rights reserved
 * @file   ms_pwm_ll.c
 * @brief c source  file of timer  module.
 * @author qun.teng
 * @date   2022-4-25
 * @version 1.0
 * @Revision
 */

#include <ms_pwm_ll.h>
#include "ms_clock_hal.h"

/**
 * @brief Set the flash time value for PWMOUT0
 * @param  PWM_Type *base
 * @param  uint32_t tim
 */
int32_t ms_pwm_set_flash_time0_ll(PWM_Type *base, uint32_t tim, PwmTimCnt_Type type)
{
    if(PWM_TIM_VALUE_SELF == type)
    {
        ms_pwm_set_tim0_ll(base, type);
        ms_pwm_set_tim0_timhigh_ll(base, ((tim >> 16) & 0xFF));
        ms_pwm_set_tim0_timlow_ll(base, (tim & 0xFFFF));
    }
    else
    {
        ms_pwm_set_tim0_ll(base, type);
    }
}

/**
 * @brief Set the flash time value for PWMOUT1
 * @param  PWM_Type *base
 * @param  uint32_t tim
 */
int32_t ms_pwm_set_flash_time1_ll(PWM_Type *base, uint32_t tim, PwmTimCnt_Type type)
{
    if(PWM_TIM_VALUE_SELF == type)
    {
        ms_pwm_set_tim1_ll(base, type);
        ms_pwm_set_tim1_timhigh_ll(base, ((tim >> 16) & 0xFF));
        ms_pwm_set_tim1_timlow_ll(base, (tim & 0xFFFF));
    }
    else
    {
        ms_pwm_set_tim1_ll(base, type);
    }
}

/**
 * @brief Set the flash time value for PWMOUT2
 * @param  PWM_Type *base
 * @param  uint32_t tim
 */
int32_t ms_pwm_set_flash_time2_ll(PWM_Type *base, uint32_t tim, PwmTimCnt_Type type)
{
    if(PWM_TIM_VALUE_SELF == type)
    {
        ms_pwm_set_tim2_ll(base, type);
        ms_pwm_set_tim2_timhigh_ll(base, ((tim >> 16) & 0xFF));
        ms_pwm_set_tim2_timlow_ll(base, (tim & 0xFFFF));
    }
    else
    {
        ms_pwm_set_tim2_ll(base, type);
    }
}