/**
  * Copyright 漏 2022 by MooreSilicon. All rights reserved
  * @file  ms_pwm.c
  * @brief pwm driver
  * @author qun.teng
  * @date 2022-4-25
  * @version 1.0
  * @Revision:
  */
#include <ms1008.h>
#include "ms_pwm_hal.h"
#include "ms_pwm.h"
#include "ms_pwm_regs.h"
#include "ms_clock_hal.h"
#include "ms_sys_ctrl_regs.h"
#include "ms_interrupt.h"
#include <stddef.h>

/**
 * @brief Enable pwm cpu interrupt.
 * @param  PWmHandle_Type *pwm : Pointer to pwm module.
 * @retval none
 */
void ms_pwm_enable_cpu_interrupt(PWMHandle_Type *pwm)
{
    INTERRUPT_ENABLE_IRQ(pwm->irq);
}

/**
 * @brief Disable pwm cpu interrupt.
 * @param  PWmHandle_Type *pwm : Pointer to pwm module.
 * @retval none
 */
void ms_pwm_disable_cpu_interrupt(PWMHandle_Type *pwm)
{
    INTERRUPT_DISABLE_IRQ(pwm->irq);
}

/**
 * @brief Enable period interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_enable_period_int(PWMHandle_Type *pwm)
{
    switch(pwm->pwmNum)
    {
        case PWM_NUM_0:
        ms_pwm_cfg_per_int0_hal(pwm->instance, PWM_ENABLE);
        break;

        case PWM_NUM_1:
        ms_pwm_cfg_per_int1_hal(pwm->instance, PWM_ENABLE);
        break;

        case PWM_NUM_2:
        ms_pwm_cfg_per_int2_hal(pwm->instance, PWM_ENABLE);
        break;

        default:
        break;
    }
}


/**
 * @brief Disable period interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_disable_period_int(PWMHandle_Type *pwm)
{
    switch(pwm->pwmNum)
    {
        case PWM_NUM_0:
        ms_pwm_cfg_per_int0_hal(pwm->instance, PWM_DISABLE);
        break;

        case PWM_NUM_1:
        ms_pwm_cfg_per_int1_hal(pwm->instance, PWM_DISABLE);
        break;

        case PWM_NUM_2:
        ms_pwm_cfg_per_int2_hal(pwm->instance, PWM_DISABLE);
        break;

        default:
        break;
    }
}

/**
 * @brief Enable flash time interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_enable_flash_int(PWMHandle_Type *pwm)
{
    switch(pwm->pwmNum)
    {
        case PWM_NUM_0:
        ms_pwm_cfg_fla_int0_hal(pwm->instance, PWM_ENABLE);
        break;

        case PWM_NUM_1:
        ms_pwm_cfg_fla_int1_hal(pwm->instance, PWM_ENABLE);
        break;

        case PWM_NUM_2:
        ms_pwm_cfg_fla_int2_hal(pwm->instance, PWM_ENABLE);
        break;

        default:
        break;
    }
}

/**
 * @brief Disable flash time interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_disable_flash_int(PWMHandle_Type *pwm)
{
    switch(pwm->pwmNum)
    {
    case PWM_NUM_0:
        ms_pwm_cfg_fla_int0_hal(pwm->instance, PWM_DISABLE);
        break;

    case PWM_NUM_1:
        ms_pwm_cfg_fla_int1_hal(pwm->instance, PWM_DISABLE);
        break;

    case PWM_NUM_2:
        ms_pwm_cfg_fla_int2_hal(pwm->instance, PWM_DISABLE);
        break;

    default:
        break;
    }
}

/**
 * @brief Get pwms interrupt status
 * @param none
 * @retval Return value can be one of the following values:
 *         PWM_INTSR_PERIOD_INT_PWM0
 *         PWM_INTSR_PERIOD_INT_PWM1
 *         PWM_INTSR_PERIOD_INT_PWM2
 *         PWM_INTSR_FLASH_TIM_INT_PWM0
 *         PWM_INTSR_FLASH_TIM_INT_PWM1
 *         PWM_INTSR_FLASH_TIM_INT_PWM2
 */
uint8_t ms_pwm_get_int_status()
{
    return ms_pwm_get_int_status_hal();
}

/**
 * @brief Clear pwms interrupt status
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_clear_int_status(PWMHandle_Type *pwm)
{
    uint8_t intStatus = 0;

    intStatus = ms_pwm_get_int_status_hal();
    if(intStatus & PWM_INTSR_PERIOD_INT_PWM0)
    {
        ms_pwm_clear_int_status_hal(pwm->instance, PWM_INTSR_PERIOD_INT_PWM0_MASK);
    }
    if(intStatus & PWM_INTSR_PERIOD_INT_PWM1)
    {
        ms_pwm_clear_int_status_hal(pwm->instance, PWM_INTSR_PERIOD_INT_PWM1_MASK);
    }
    if(intStatus & PWM_INTSR_PERIOD_INT_PWM2)
    {
        ms_pwm_clear_int_status_hal(pwm->instance, PWM_INTSR_PERIOD_INT_PWM2_MASK);
    }
    if(intStatus & PWM_INTSR_FLASH_TIM_INT_PWM0)
    {
        ms_pwm_clear_int_status_hal(pwm->instance, PWM_INTSR_FLASH_TIM_INT_PWM0_MASK);
    }
    if(intStatus & PWM_INTSR_FLASH_TIM_INT_PWM1)
    {
        ms_pwm_clear_int_status_hal(pwm->instance, PWM_INTSR_FLASH_TIM_INT_PWM1_MASK);
    }
    if(intStatus & PWM_INTSR_FLASH_TIM_INT_PWM2)
    {
        ms_pwm_clear_int_status_hal(pwm->instance, PWM_INTSR_FLASH_TIM_INT_PWM2_MASK);
    }
}

/**
 * @brief Get the value of PWMn current counter
 * @param  PWMHandle_Type *pwm
 * @retval PWMn current counter
 */
uint16_t ms_pwm_get_current_cnt(PWMHandle_Type *pwm)
{
    switch(pwm->pwmNum)
    {
    case PWM_NUM_0:
        return ms_pwm_get_cnt0_hal(pwm->instance);
        break;

    case PWM_NUM_1:
        return ms_pwm_get_cnt1_hal(pwm->instance);
        break;

    case PWM_NUM_2:
        return ms_pwm_get_cnt2_hal(pwm->instance);
        break;

    default:
        break;
    }
}

/**
 * @brief Restart PWMn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_restart(PWMHandle_Type *pwm)
{
    switch(pwm->pwmNum)
    {
    case PWM_NUM_0:
        ms_pwm_cfg_restart0_hal(pwm->instance, PWM_ENABLE);
        break;

    case PWM_NUM_1:
        ms_pwm_cfg_restart1_hal(pwm->instance, PWM_ENABLE);
        break;

    case PWM_NUM_2:
        ms_pwm_cfg_restart2_hal(pwm->instance, PWM_ENABLE);
        break;

    default:
        break;
    }
}

/**
 * @brief Enable PWMn output
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_enable_output(PWMHandle_Type *pwm)
{
    switch(pwm->pwmNum)
    {
    case PWM_NUM_0:
        ms_pwm_cfg_pwmout0_hal(pwm->instance, PWM_ENABLE);
        break;

    case PWM_NUM_1:
        ms_pwm_cfg_pwmout1_hal(pwm->instance, PWM_ENABLE);
        break;

    case PWM_NUM_2:
        ms_pwm_cfg_pwmout2_hal(pwm->instance, PWM_ENABLE);
        break;

    default:
        break;
    }
}

/**
 * @brief Disable PWMn output
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_disable_output(PWMHandle_Type *pwm)
{
    switch(pwm->pwmNum)
    {
    case PWM_NUM_0:
        ms_pwm_cfg_pwmout0_hal(pwm->instance, PWM_DISABLE);
        break;

    case PWM_NUM_1:
        ms_pwm_cfg_pwmout1_hal(pwm->instance, PWM_DISABLE);
        break;

    case PWM_NUM_2:
        ms_pwm_cfg_pwmout2_hal(pwm->instance, PWM_DISABLE);
        break;

    default:
        break;
    }
}

/**
 * @brief Set the type of PWMn period interrupt assert timing is 0
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_set_perint_assert_0(PWMHandle_Type *pwm)
{
    switch(pwm->pwmNum)
    {
    case PWM_NUM_0:
        ms_pwm_cfg_intas0_hal(pwm->instance, PWM_PERINT_ASSERT_0);
        break;

    case PWM_NUM_1:
        ms_pwm_cfg_intas1_hal(pwm->instance, PWM_PERINT_ASSERT_0);
        break;

    case PWM_NUM_2:
        ms_pwm_cfg_intas2_hal(pwm->instance, PWM_PERINT_ASSERT_0);
        break;

    default:
        break;
    }
}

/**
 * @brief Set the type of PWMn period interrupt assert timing is DUTn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_set_perint_assert_dut(PWMHandle_Type *pwm)
{
    switch(pwm->pwmNum)
    {
    case PWM_NUM_0:
        ms_pwm_cfg_intas0_hal(pwm->instance, PWM_PERINT_ASSERT_DUT);
        break;

    case PWM_NUM_1:
        ms_pwm_cfg_intas1_hal(pwm->instance, PWM_PERINT_ASSERT_DUT);
        break;

    case PWM_NUM_2:
        ms_pwm_cfg_intas2_hal(pwm->instance, PWM_PERINT_ASSERT_DUT);
        break;

    default:
        break;
    }
}

/**
 * @brief Register the pwm interrupt handler function
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_irq_handler(PWMHandle_Type *pwm, uint8_t intStatus)
{
    ms_pwm_clear_int_status(pwm);
    if (pwm->p_callback == NULL)
    {
        pwm->error_code |= PWM_ERROR_INVALID_CALLBACK;
        return;
    }
    if((intStatus & PWM_INTSR_PERIOD_INT_PWM0) || (intStatus & PWM_INTSR_PERIOD_INT_PWM1) || (intStatus & PWM_INTSR_PERIOD_INT_PWM2))
    {
        pwm->p_callback->pwm_period_callback(pwm);
    }
    if((intStatus & PWM_INTSR_FLASH_TIM_INT_PWM0) || (intStatus & PWM_INTSR_FLASH_TIM_INT_PWM1) || (intStatus & PWM_INTSR_FLASH_TIM_INT_PWM2))
    {
        pwm->p_callback->pwm_flash_callback(pwm);
    }
}

/**
 * @brief Initial pwm module.
 * @param  PWMHandle_Type *pwm : Pointer to pwm module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_pwm_init(PWMHandle_Type *pwm)
{
    if(NULL == pwm)
    {
        return STATUS_ERROR;
    }
    /*initial PWM gpio pinmux, clock and interrupt setting*/
    if (pwm->p_callback && pwm->p_callback->init_callback)
    {
        pwm->p_callback->init_callback(pwm);
    }

    switch(pwm->pwmNum)
    {
    case PWM_NUM_0:
        ms_pwm_set_period_cyc0_hal(pwm->instance, pwm->init.cycle);
        ms_pwm_set_dut0_hal(pwm->instance, pwm->init.duty);
        ms_pwm_set_flash_time0_hal(pwm->instance, pwm->init.timVal, pwm->init.cntType);
        ms_pwm_enable_pwm0_hal(pwm->instance);
        break;

    case PWM_NUM_1:
        ms_pwm_set_period_cyc1_hal(pwm->instance, pwm->init.cycle);
        ms_pwm_set_dut1_hal(pwm->instance, pwm->init.duty);
        ms_pwm_set_flash_time1_hal(pwm->instance, pwm->init.timVal, pwm->init.cntType);
        ms_pwm_enable_pwm1_hal(pwm->instance);
        break;

    case PWM_NUM_2:
        ms_pwm_set_period_cyc2_hal(pwm->instance, pwm->init.cycle);
        ms_pwm_set_dut2_hal(pwm->instance, pwm->init.duty);
        ms_pwm_set_flash_time2_hal(pwm->instance, pwm->init.timVal, pwm->init.cntType);
        ms_pwm_enable_pwm2_hal(pwm->instance);
        break;

    default:
        break;
    }
    return STATUS_SUCCESS;
}

/**
 * De-initialises an pwm interface
 * @param[in] PWMHandle_Type *pwm : Pointer to pwm module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_pwm_deinit(PWMHandle_Type *pwm)
{
    if(NULL == pwm)
    {
        return STATUS_ERROR;
    }

    /*Disable the peripheral */
    switch(pwm->pwmNum)
    {
    case PWM_NUM_0:
        ms_pwm_disable_pwm0_hal(pwm->instance);
        break;

    case PWM_NUM_1:
        ms_pwm_disable_pwm1_hal(pwm->instance);
        break;

    case PWM_NUM_2:
        ms_pwm_disable_pwm2_hal(pwm->instance);
        break;

    default:
        break;
    }
    /* DeInit the low level hardware */
    if (pwm->p_callback && pwm->p_callback->deinit_callback)
    {
        pwm->p_callback->deinit_callback(pwm);
    }

    return STATUS_SUCCESS;
}