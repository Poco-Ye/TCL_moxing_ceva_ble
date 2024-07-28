/**
 * Copyright ? 2022 by MooreSilicon.All rights reserved
 * @file   ms_pwm_ll.c
 * @brief c source  file of timer  module.
 * @author qun.teng
 * @date   2022-4-25
 * @version 1.0
 * @Revision
 */
#ifndef MS_PWM_HAL_H_
#define MS_PWM_HAL_H_

#include "ms_pwm_ll.h"

/**
 * @brief Set the value of period for PWMOUT0
 * @param  PWM_Type *base
 * @param  uint16_t period
 * @retval none
 */
#define ms_pwm_set_period_cyc0_hal(base, period) ms_pwm_set_period_cyc0_ll(base, period)

/**
 * @brief Set the value of period for PWMOUT1
 * @param  PWM_Type *base
 * @param  uint16_t period
 * @retval none
 */
#define ms_pwm_set_period_cyc1_hal(base, period) ms_pwm_set_period_cyc1_ll(base, period)

/**
 * @brief Set the value of period for PWMOUT2
 * @param  PWM_Type *base
 * @param  uint16_t period
 * @retval none
 */
#define ms_pwm_set_period_cyc2_hal(base, period) ms_pwm_set_period_cyc2_ll(base, period)

/**
 * @brief Set the Duty cycle of period for PWMOUT0
 * @param  PWM_Type *base
 * @param  uint16_t dutyCycle
 * @retval none
 */
#define ms_pwm_set_dut0_hal(base, dutyCycle) ms_pwm_set_dut0_ll(base, dutyCycle)

/**
 * @brief Set the Duty cycle of period for PWMOUT1
 * @param  PWM_Type *base
 * @param  uint16_t dutyCycle
 * @retval none
 */
#define ms_pwm_set_dut1_hal(base, dutyCycle) ms_pwm_set_dut1_ll(base, dutyCycle)

/**
 * @brief Set the Duty cycle of period for PWMOUT2
 * @param  PWM_Type *base
 * @param  uint16_t dutyCycle
 * @retval none
 */
#define ms_pwm_set_dut2_hal(base, dutyCycle) ms_pwm_set_dut2_ll(base, dutyCycle)

/**
 * @brief Get the Counter of PWMOUT0
 * @param  PWM_Type *base
 * @retval Counter value
 */
#define ms_pwm_get_cnt0_hal(base) ms_pwm_get_cnt0_ll(base)

/**
 * @brief Get the Counter of PWMOUT1
 * @param  PWM_Type *base
 * @retval Counter value
 */
#define ms_pwm_get_cnt1_hal(base) ms_pwm_get_cnt1_ll(base)

/**
 * @brief Get the Counter of PWMOUT2
 * @param  PWM_Type *base
 * @retval Counter value
 */
#define ms_pwm_get_cnt2_hal(base) ms_pwm_get_cnt2_ll(base)

/**
 * @brief Set the flash time value for PWMOUT0
 * @param  PWM_Type *base
 * @param  uint32_t tim
 */
#define ms_pwm_set_flash_time0_hal(base, tim, type) ms_pwm_set_flash_time0_ll(base, tim, type)

/**
 * @brief Set the flash time value for PWMOUT1
 * @param  PWM_Type *base
 * @param  uint32_t tim
 */
#define ms_pwm_set_flash_time1_hal(base, tim, type) ms_pwm_set_flash_time1_ll(base, tim, type)

/**
 * @brief Set the flash time value for PWMOUT2
 * @param  PWM_Type *base
 * @param  uint32_t tim
 */
#define ms_pwm_set_flash_time2_hal(base, tim, type) ms_pwm_set_flash_time2_ll(base, tim, type)


/**
 * @brief Restart the PWMOUT0 or not
 * @param  PWM_Type *base
 * @param  uint32_t enable
 * @retval none
 */
#define ms_pwm_cfg_restart0_hal(base, enable) ms_pwm_cfg_restart0_ll(base, enable)

/**
 * @brief Restart the PWMOUT1 or not
 * @param  PWM_Type *base
 * @param  uint32_t enable
 * @retval none
 */
#define ms_pwm_cfg_restart1_hal(base, enable) ms_pwm_cfg_restart1_ll(base, enable)

/**
 * @brief Restart the PWMOUT2 or not
 * @param  PWM_Type *base
 * @param  uint32_t enable
 * @retval none
 */
#define ms_pwm_cfg_restart2_hal(base, enable) ms_pwm_cfg_restart2_ll(base, enable)

/**
 * @brief Enable for output driver PWMOUT0 or disable
 * @param  PWM_Type *base
 * @param  uint32_t enable
 * @retval none
 */
#define ms_pwm_cfg_pwmout0_hal(base, enable) ms_pwm_cfg_pwmout0_ll(base, enable)

/**
 * @brief Enable for output driver PWMOUT1 or disable
 * @param  PWM_Type *base
 * @param  uint32_t enable
 * @retval none
 */
#define ms_pwm_cfg_pwmout1_hal(base, enable) ms_pwm_cfg_pwmout1_ll(base, enable)

/**
 * @brief Enable for output driver PWMOUT2 or disable
 * @param  PWM_Type *base
 * @param  uint32_t enable
 * @retval none
 */
#define ms_pwm_cfg_pwmout2_hal(base, enable) ms_pwm_cfg_pwmout2_ll(base, enable)

/**
 * @brief Configure the interrupt assert timing for PER_INT0(Period Interrupt)
 * @param  PWM_Type *base
 * @param  uint8_t type
 * @retval none
 */
#define ms_pwm_cfg_intas0_hal(base, type) ms_pwm_cfg_intas0_ll(base, type)

/**
 * @brief Configure the interrupt assert timing for PER_INT1(Period Interrupt)
 * @param  PWM_Type *base
 * @param  uint8_t type
 * @retval none
 */
#define ms_pwm_cfg_intas1_hal(base, type) ms_pwm_cfg_intas1_ll(base, type)

/**
 * @brief Configure the interrupt assert timing for PER_INT2(Period Interrupt)
 * @param  PWM_Type *base
 * @param  uint8_t type
 * @retval none
 */
#define ms_pwm_cfg_intas2_hal(base, type) ms_pwm_cfg_intas2_ll(base, type)

/**
 * @brief Enable the period interrupt for PWM0 or disable
 * @param  PWM_Type *base
 * @param  uint8_t enable
 * @retval none
 */
#define ms_pwm_cfg_per_int0_hal(base, enable) ms_pwm_cfg_per_int0_ll(base, enable)

/**
 * @brief Enable the period interrupt for PWM1 or disable
 * @param  PWM_Type *base
 * @param  uint8_t enable
 * @retval none
 */
#define ms_pwm_cfg_per_int1_hal(base, enable) ms_pwm_cfg_per_int1_ll(base, enable)

/**
 * @brief Enable the period interrupt for PWM2 or disable
 * @param  PWM_Type *base
 * @param  uint8_t enable
 * @retval none
 */
#define ms_pwm_cfg_per_int2_hal(base, enable) ms_pwm_cfg_per_int2_ll(base, enable)

/**
 * @brief Enable the flash time interrupt for PWM0 or disable
 * @param  PWM_Type *base
 * @param  uint8_t enable
 * @retval none
 */
#define ms_pwm_cfg_fla_int0_hal(base, enable) ms_pwm_cfg_fla_int0_ll(base, enable)

/**
 * @brief Enable the flash time interrupt for PWM1 or disable
 * @param  PWM_Type *base
 * @param  uint8_t enable
 * @retval none
 */
#define ms_pwm_cfg_fla_int1_hal(base, enable) ms_pwm_cfg_fla_int1_ll(base, enable)

/**
 * @brief Enable the flash time interrupt for PWM2 or disable
 * @param  PWM_Type *base
 * @param  uint8_t enable
 * @retval none
 */
#define ms_pwm_cfg_fla_int2_hal(base, enable) ms_pwm_cfg_fla_int2_ll(base, enable)

/**
 * @brief Get the interrupt status
 * @param none
 * @retval Interrupt status
 */
#define ms_pwm_get_int_status_hal() ms_pwm_get_int_status_ll()

/**
 * @brief Clear the interrupt status
 * @param  PWM_Type *base
 * @param  uint32_t clearValue
 * @retval none
 */
#define ms_pwm_clear_int_status_hal(base, clearValue) ms_pwm_clear_int_status_ll(base, clearValue)

/**
 * @brief PWM enable and Clock gate enable for PWM0
 * @param  PWM_Type *base
 * @retval Counter value
 */
#define ms_pwm_enable_pwm0_hal(base) ms_pwm_enable_pwm0_ll(base)

/**
 * @brief PWM enable and Clock gate enable for PWM1
 * @param  PWM_Type *base
 * @retval Counter value
 */
#define ms_pwm_enable_pwm1_hal(base) ms_pwm_enable_pwm1_ll(base)

/**
 * @brief PWM enable and Clock gate enable for PWM2
 * @param  PWM_Type *base
 * @retval Counter value
 */
#define ms_pwm_enable_pwm2_hal(base) ms_pwm_enable_pwm2_ll(base)

/**
 * @brief PWM disable and Clock gate disable for PWM0
 * @param  PWM_Type *base
 * @retval Counter value
 */
#define ms_pwm_disable_pwm0_hal(base) ms_pwm_disable_pwm0_ll(base)

/**
 * @brief PWM disable and Clock gate disable for PWM1
 * @param  PWM_Type *base
 * @retval Counter value
 */
#define ms_pwm_disable_pwm1_hal(base) ms_pwm_disable_pwm1_ll(base)

/**
 * @brief PWM disable and Clock gate disable for PWM2
 * @param  PWM_Type *base
 * @retval Counter value
 */
#define ms_pwm_disable_pwm2_hal(base) ms_pwm_disable_pwm2_ll(base)

#endif/* MS_TIMER_HAL_H_ */