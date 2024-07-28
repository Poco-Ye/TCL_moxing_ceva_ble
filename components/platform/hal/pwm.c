/*
 * pwm.c
 *
 *  Created on: 2022-4-26
 *      Author: qun.teng
 */

#include <ms_clock_hal.h>
#include <ms_pinmux_hal.h>
#include "ms_pwm.h"
#include "pwm.h"
#include <stddef.h>
#include "log.h"

#define PWM0_PERIOD_INT              0x1
#define PWM1_PERIOD_INT              0x2
#define PWM2_PERIOD_INT              0x4
#define PWM0_FLASH_TIME_INT          0x8
#define PWM1_FLASH_TIME_INT          0x10
#define PWM2_FLASH_TIME_INT          0x20

PwmCallback_Type pwm0_callback =
{
    .init_callback = pwm_init_callback,
    .deinit_callback = pwm_deinit_callback,
    .pwm_period_callback = pwm0_period_callback,
    .pwm_flash_callback = pwm0_flash_callback,
};

PwmCallback_Type pwm1_callback =
{
    .init_callback = pwm_init_callback,
    .deinit_callback = pwm_deinit_callback,
    .pwm_period_callback = pwm1_period_callback,
    .pwm_flash_callback = pwm1_flash_callback,
};

PwmCallback_Type pwm2_callback =
{
    .init_callback = pwm_init_callback,
    .deinit_callback = pwm_deinit_callback,
    .pwm_period_callback = pwm2_period_callback,
    .pwm_flash_callback = pwm2_flash_callback,
};

PWMHandle_Type pwm_handle[PWM_NUM_MAX] =
{
    {
        .instance = PWM,
        .init.cntType = PWM_TIM_VALUE_2,
        .init.cycle = 100,
        .init.duty = 50,
        .init.timVal = 0,
        .pwmNum = PWM_NUM_0,
        .p_callback = &pwm0_callback,
        .irq = PWM_IRQn,
    },
    {
        .instance = PWM,
        .init.cntType = PWM_TIM_VALUE_SELF,
        .init.cycle = 77,
        .init.duty = 25,
        .init.timVal = 0xFF00,
        .pwmNum = PWM_NUM_1,
        .p_callback = &pwm1_callback,
        .irq = PWM_IRQn,
    },
    {
        .instance = PWM,
        .init.cntType = PWM_TIM_VALUE_1,
        .init.cycle = 77,
        .init.duty = 25,
        .init.timVal = 0x1ffff,
        .pwmNum = PWM_NUM_2,
        .p_callback = &pwm2_callback,
        .irq = PWM_IRQn,
    },
};

/**
 * @brief Enable period interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_enable_period_int(PWMHandle_Type *pwm)
{
    /*enable pwm period interrupt*/
    ms_pwm_enable_period_int(pwm);
    return STATUS_SUCCESS;
}

/**
 * @brief Enable flash time interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_enable_flash_int(PWMHandle_Type *pwm)
{
    /*enable pwm flash time interrupt*/
    ms_pwm_enable_flash_int(pwm);
    return STATUS_SUCCESS;
}

/**
 * @brief Disable period interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_disable_period_int(PWMHandle_Type *pwm)
{
    /*disable pwm period interrupt*/
    ms_pwm_disable_period_int(pwm);
    return STATUS_SUCCESS;
}

/**
 * @brief Disable flash time interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_disable_flash_int(PWMHandle_Type *pwm)
{
    /*disable pwm flash time interrupt*/
    ms_pwm_disable_flash_int(pwm);
    return STATUS_SUCCESS;
}

/**
 * @brief Inital callback function setting for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm_init_callback(PWMHandle_Type *pwm)
{
    /*config the pwm clock*/
    MS_CLOCK_HAL_CLK_ENABLE_PWM();

    ms_clock_hal_set_pwm_div(5);

    ms_clock_hal_peripheral_clk_div_toggle(PWM_DIV_TOG);

    /*config the pwm0 pin mux*/
    ms_pinmux_hal_set_pinmux(PAD3,PIN03_PWM0);
    /*config the pwm0 pin mux*/
    ms_pinmux_hal_set_pinmux(PAD4,PIN04_PWM1);
    /*config the pwm0 pin mux*/
    ms_pinmux_hal_set_pinmux(PAD5,PIN05_PWM2);

    /*disable pwm interrupt*/
    ms_pwm_disable_flash_int(pwm);
    ms_pwm_disable_period_int(pwm);

    /*enable the cpu interrupt*/
    ms_pwm_enable_cpu_interrupt(pwm);

    MS_LOGI(MS_DRIVER, "pwm init finish\r\n");
}

/**
 * @brief De-init callback function setting for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm_deinit_callback(PWMHandle_Type *pwm)
{
    MS_LOGI(MS_DRIVER, "pwm deinit\r\n");
    /*disable cpu interrupt*/
    ms_pwm_disable_cpu_interrupt(pwm);

    /*disable the pwm clock*/
    MS_CLOCK_HAL_CLK_DISABLE_PWM();
}

/**
 * @brief PWM period interrupt callback function setting for PWM0
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm0_period_callback(PWMHandle_Type *pwm)
{
    MS_LOGI(MS_DRIVER, "\r\n pwm0 period int!\n");
}

/**
 * @brief PWM period interrupt callback function setting for PWM1
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm1_period_callback(PWMHandle_Type *pwm)
{
    MS_LOGI(MS_DRIVER, "\r\n pwm1 period int!\n");
}

/**
 * @brief PWM period interrupt callback function setting for PWM2
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm2_period_callback(PWMHandle_Type *pwm)
{
    MS_LOGI(MS_DRIVER, "\r\n pwm2 period int!\n");
}

/**
 * @brief PWM flash time interrupt callback function setting for PWM0
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm0_flash_callback(PWMHandle_Type *pwm)
{
    MS_LOGI(MS_DRIVER, "\r\n pwm0 flash time int!\n");
}

/**
 * @brief PWM flash time interrupt callback function setting for PWM1
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm1_flash_callback(PWMHandle_Type *pwm)
{
    MS_LOGI(MS_DRIVER, "\r\n pwm1 flash time int!\n");
}

/**
 * @brief PWM flash time interrupt callback function setting for PWM2
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm2_flash_callback(PWMHandle_Type *pwm)
{
    MS_LOGI(MS_DRIVER, "\r\n pwm2 flash time int!\n");
}

/**
 * @brief Get the value of PWMn current counter
 * @param  PWMHandle_Type *pwm
 * @retval current counter
 */
uint16_t pwm_get_current_cnt(PWMHandle_Type *pwm)
{
    return ms_pwm_get_current_cnt(pwm);
}

/**
 * @brief Restart PWMn
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_restart(PWMHandle_Type *pwm)
{
    ms_pwm_restart(pwm);
    return STATUS_SUCCESS;
}

/**
 * @brief Enable PWMn output
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_enable_output(PWMHandle_Type *pwm)
{
    ms_pwm_enable_output(pwm);
    return STATUS_SUCCESS;
}

/**
 * @brief Disable PWMn output
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_disable_output(PWMHandle_Type *pwm)
{
    ms_pwm_disable_output(pwm);
    return STATUS_SUCCESS;
}

/**
 * @brief Set the type of PWMn period interrupt assert timing is 0
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_set_perint_assert_0(PWMHandle_Type *pwm)
{
    ms_pwm_set_perint_assert_0(pwm);
    return STATUS_SUCCESS;
}

/**
 * @brief Set the type of PWMn period interrupt assert timing is DUTn
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_set_perint_assert_dut(PWMHandle_Type *pwm)
{
    ms_pwm_set_perint_assert_dut(pwm);
    return STATUS_SUCCESS;
}

/**
 * @brief Initial PWM0
 * @param none
 * @retval STATUS_SUCCESS
 */
int32_t pwm0_init(void)
{
    ms_pwm_init(&pwm_handle[0]);

    return STATUS_SUCCESS;
}

/**
 * @brief Initial PWM1
 * @param none
 * @retval STATUS_SUCCESS
 */
int32_t pwm1_init(void)
{
    ms_pwm_init(&pwm_handle[1]);

    return STATUS_SUCCESS;
}

/**
 * @brief Initial PWM2
 * @param none
 * @retval STATUS_SUCCESS
 */
int32_t pwm2_init(void)
{
    ms_pwm_init(&pwm_handle[2]);

    return STATUS_SUCCESS;
}

/**
 * @brief De-init PWM0
 * @param none
 * @retval STATUS_SUCCESS
 */
int32_t pwm0_deinit(void)
{
    ms_pwm_deinit(&pwm_handle[0]);
    return STATUS_SUCCESS;
}

/**
 * @brief De-init PWM1
 * @param none
 * @retval STATUS_SUCCESS
 */
int32_t pwm1_deinit(void)
{
    ms_pwm_deinit(&pwm_handle[1]);
    return STATUS_SUCCESS;
}

/**
 * @brief De-init PWM2
 * @param none
 * @retval STATUS_SUCCESS
 */
int32_t pwm2_deinit(void)
{
    ms_pwm_deinit(&pwm_handle[2]);
    return STATUS_SUCCESS;
}

/**
 * @brief PWM interrupt handler function
 * @param none
 * @retval STATUS_SUCCESS
 */
void PWM_IRQHandler(void)
{
    uint8_t intStatus = 0 ;

    intStatus = ms_pwm_get_int_status();

    if((intStatus & PWM0_PERIOD_INT) || (intStatus & PWM0_FLASH_TIME_INT))
    {
        ms_pwm_irq_handler(&pwm_handle[0], intStatus);
    }
    if((intStatus & PWM1_PERIOD_INT) || (intStatus & PWM1_FLASH_TIME_INT))
    {
        ms_pwm_irq_handler(&pwm_handle[1], intStatus);
    }
    if((intStatus & PWM2_PERIOD_INT) || (intStatus & PWM2_FLASH_TIME_INT))
    {
        ms_pwm_irq_handler(&pwm_handle[2], intStatus);
    }
}
