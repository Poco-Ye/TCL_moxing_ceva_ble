/**
 * Copyright 漏 2022 by MooreSilicon.All rights reserved
 * @file  ms_pwm_ll.h
 * @brief Header file of PWM  module.
 * @author qun.teng
 * @date   2022-4-24
 * @version 1.0
 * @Revision:
 */
#ifndef MS_LL_PWM_H_
#define MS_LL_PWM_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>
#include "ms_pwm_regs.h"

/**
 * @brief  PWM flash time value type
 */
typedef enum
{
    PWM_TIM_VALUE_0 = 0,                                                             /*!< PWM flash time counter is 0 */
    PWM_TIM_VALUE_1,                                                                 /*!< PWM flash time counter is 1637 */
    PWM_TIM_VALUE_2,                                                                 /*!< PWM flash time counter is 3275 */
    PWM_TIM_VALUE_3,                                                                 /*!< PWM flash time counter is 6552 */
    PWM_TIM_VALUE_4,                                                                 /*!< PWM flash time counter is 16383 */
    PWM_TIM_VALUE_5,                                                                 /*!< PWM flash time counter is 32767 */
    PWM_TIM_VALUE_6,                                                                 /*!< PWM flash time counter is 65535 */
    PWM_TIM_VALUE_7,                                                                 /*!< PWM flash time counter is 163839 */
    PWM_TIM_VALUE_8,                                                                 /*!< PWM flash time counter is 327679 */
    PWM_TIM_VALUE_9,                                                                 /*!< PWM flash time counter is 491519 */
    PWM_TIM_VALUE_10,                                                                /*!< PWM flash time counter is 983039 */
    PWM_TIM_VALUE_11,                                                                /*!< PWM flash time counter is 1966079 */
    PWM_TIM_VALUE_12,                                                                /*!< PWM flash time counter is 3932159 */
    PWM_TIM_VALUE_13,                                                                /*!< PWM flash time counter is infinity */
    PWM_TIM_VALUE_REREVED,                                                           /*!< PWM flash time counter is infinity */
    PWM_TIM_VALUE_SELF,                                                              /*!< PWM flash time counter is user self-defined */
}PwmTimCnt_Type;

/**
 * @brief Set the flash time value for PWMOUT0
 * @param  PWM_Type *base
 * @param  uint32_t tim
 */
int32_t ms_pwm_set_flash_time0_ll(PWM_Type *base, uint32_t tim, PwmTimCnt_Type type);

/**
 * @brief Set the flash time value for PWMOUT1
 * @param  PWM_Type *base
 * @param  uint32_t tim
 */
int32_t ms_pwm_set_flash_time1_ll(PWM_Type *base, uint32_t tim, PwmTimCnt_Type type);

/**
 * @brief Set the flash time value for PWMOUT2
 * @param  PWM_Type *base
 * @param  uint32_t tim
 */
int32_t ms_pwm_set_flash_time2_ll(PWM_Type *base, uint32_t tim, PwmTimCnt_Type type);

/**
 * @brief Set the value of period for PWMOUT0
 * @param  PWM_Type *base
 * @param  uint16_t period
 * @retval none
 */
static inline void ms_pwm_set_period_cyc0_ll(PWM_Type *base, uint16_t period)
{
    MODIFY_REG(base->CYC0, PWM_CYC0_PWMOUT0,((period<<PWM_CYC0_PWMOUT0_POS)&PWM_CYC0_PWMOUT0));
}

/**
 * @brief Set the value of period for PWMOUT1
 * @param  PWM_Type *base
 * @param  uint16_t period
 * @retval none
 */
static inline void ms_pwm_set_period_cyc1_ll(PWM_Type *base, uint16_t period)
{
    MODIFY_REG(base->CYC1, PWM_CYC1_PWMOUT1,((period<<PWM_CYC1_PWMOUT1_POS)&PWM_CYC1_PWMOUT1));
}

/**
 * @brief Set the value of period for PWMOUT2
 * @param  PWM_Type *base
 * @param  uint16_t period
 * @retval none
 */
static inline void ms_pwm_set_period_cyc2_ll(PWM_Type *base, uint16_t period)
{
    MODIFY_REG(base->CYC2, PWM_CYC2_PWMOUT2,((period<<PWM_CYC2_PWMOUT2_POS)&PWM_CYC2_PWMOUT2));
}

/**
 * @brief Set the Duty cycle of period for PWMOUT0
 * @param  PWM_Type *base
 * @param  uint16_t dutyCycle
 * @retval none
 */
static inline void ms_pwm_set_dut0_ll(PWM_Type *base, uint16_t dutyCycle)
{
    MODIFY_REG(base->DUT0, PWM_DUT0_PWMOUT0,((dutyCycle<<PWM_DUT0_PWMOUT0_POS)&PWM_DUT0_PWMOUT0));
}

/**
 * @brief Set the Duty cycle of period for PWMOUT1
 * @param  PWM_Type *base
 * @param  uint16_t dutyCycle
 * @retval none
 */
static inline void ms_pwm_set_dut1_ll(PWM_Type *base, uint16_t dutyCycle)
{
    MODIFY_REG(base->DUT1, PWM_DUT1_PWMOUT1,((dutyCycle<<PWM_DUT1_PWMOUT1_POS)&PWM_DUT1_PWMOUT1));
}

/**
 * @brief Set the Duty cycle of period for PWMOUT2
 * @param  PWM_Type *base
 * @param  uint16_t dutyCycle
 * @retval none
 */
static inline void ms_pwm_set_dut2_ll(PWM_Type *base, uint16_t dutyCycle)
{
    MODIFY_REG(base->DUT2, PWM_DUT2_PWMOUT2,((dutyCycle<<PWM_DUT2_PWMOUT2_POS)&PWM_DUT2_PWMOUT2));
}

/**
 * @brief Get the Counter of PWMOUT0
 * @param  PWM_Type *base
 * @retval Counter value
 */
static inline uint16_t ms_pwm_get_cnt0_ll(PWM_Type *base)
{
    return (uint16_t)READ_REG(base->CNT0);
}

/**
 * @brief Get the Counter of PWMOUT1
 * @param  PWM_Type *base
 * @retval Counter value
 */
static inline uint16_t ms_pwm_get_cnt1_ll(PWM_Type *base)
{
    return (uint16_t)READ_REG(base->CNT1);
}

/**
 * @brief Get the Counter of PWMOUT2
 * @param  PWM_Type *base
 * @retval Counter value
 */
static inline uint16_t ms_pwm_get_cnt2_ll(PWM_Type *base)
{
    return (uint16_t)READ_REG(base->CNT2);
}

/**
 * @brief Set the value of flash time for PWMOUT0
 * @param  PWM_Type *base
 * @param  uint8_t tim
 * @retval Counter value
 */
static inline void ms_pwm_set_tim0_ll(PWM_Type *base, uint8_t tim)
{
    MODIFY_REG(base->TIM0, PWM_TIM0_PWMOUT0,((tim<<PWM_TIM0_PWMOUT0_POS)&PWM_TIM0_PWMOUT0));
}

/**
 * @brief Set the value of flash time for PWMOUT1
 * @param  PWM_Type *base
 * @param  uint8_t tim
 * @retval Counter value
 */
static inline void ms_pwm_set_tim1_ll(PWM_Type *base, uint8_t tim)
{
    MODIFY_REG(base->TIM1, PWM_TIM1_PWMOUT1,((tim<<PWM_TIM1_PWMOUT1_POS)&PWM_TIM1_PWMOUT1));
}

/**
 * @brief Set the value of flash time for PWMOUT2
 * @param  PWM_Type *base
 * @param  uint8_t tim
 * @retval Counter value
 */
static inline void ms_pwm_set_tim2_ll(PWM_Type *base, uint8_t tim)
{
    MODIFY_REG(base->TIM2, PWM_TIM2_PWMOUT2,((tim<<PWM_TIM2_PWMOUT2_POS)&PWM_TIM2_PWMOUT2));
}

/**
 * @brief Set the High part of self-define flash time value for PWMOUT0
 * @param  PWM_Type *base
 * @param  uint8_t timHigh
 * @retval Counter value
 */
static inline void ms_pwm_set_tim0_timhigh_ll(PWM_Type *base, uint8_t timHigh)
{
    MODIFY_REG(base->TIM0, PWM_TIM0_TIM0_VAL_H,((timHigh<<PWM_TIM0_TIM0_VAL_H_POS)&PWM_TIM0_TIM0_VAL_H));
}

/**
 * @brief Set the High part of self-define flash time value for PWMOUT1
 * @param  PWM_Type *base
 * @param  uint8_t timHigh
 * @retval Counter value
 */
static inline void ms_pwm_set_tim1_timhigh_ll(PWM_Type *base, uint8_t timHigh)
{
    MODIFY_REG(base->TIM1, PWM_TIM1_TIM1_VAL_H,((timHigh<<PWM_TIM1_TIM1_VAL_H_POS)&PWM_TIM1_TIM1_VAL_H));
}

/**
 * @brief Set the High part of self-define flash time value for PWMOUT2
 * @param  PWM_Type *base
 * @param  uint8_t timHigh
 * @retval Counter value
 */
static inline void ms_pwm_set_tim2_timhigh_ll(PWM_Type *base, uint8_t timHigh)
{
    MODIFY_REG(base->TIM2, PWM_TIM2_TIM2_VAL_H,((timHigh<<PWM_TIM2_TIM2_VAL_H_POS)&PWM_TIM2_TIM2_VAL_H));
}

/**
 * @brief Restart the PWMOUT0 or not
 * @param  PWM_Type *base
 * @param  uint32_t enable
 * @retval none
 */
static inline void ms_pwm_cfg_restart0_ll(PWM_Type *base, uint32_t enable)
{
    MODIFY_REG(base->PWMOUTEN, PWM_PWMOUTEN_RESTART_PWM0,((enable<<PWM_PWMOUTEN_RESTART_PWM0_POS)&PWM_PWMOUTEN_RESTART_PWM0));
}

/**
 * @brief Restart the PWMOUT1 or not
 * @param  PWM_Type *base
 * @param  uint32_t enable
 * @retval none
 */
static inline void ms_pwm_cfg_restart1_ll(PWM_Type *base, uint32_t enable)
{
    MODIFY_REG(base->PWMOUTEN, PWM_PWMOUTEN_RESTART_PWM1,((enable<<PWM_PWMOUTEN_RESTART_PWM1_POS)&PWM_PWMOUTEN_RESTART_PWM1));
}

/**
 * @brief Restart the PWMOUT2 or not
 * @param  PWM_Type *base
 * @param  uint32_t enable
 * @retval none
 */
static inline void ms_pwm_cfg_restart2_ll(PWM_Type *base, uint32_t enable)
{
    MODIFY_REG(base->PWMOUTEN, PWM_PWMOUTEN_RESTART_PWM2,((enable<<PWM_PWMOUTEN_RESTART_PWM2_POS)&PWM_PWMOUTEN_RESTART_PWM2));
}

/**
 * @brief Enable for output driver PWMOUT0 or disable
 * @param  PWM_Type *base
 * @param  uint32_t enable
 * @retval none
 */
static inline void ms_pwm_cfg_pwmout0_ll(PWM_Type *base, uint32_t enable)
{
    MODIFY_REG(base->PWMOUTEN, PWM_PWMOUTEN_OUTEN_PWM0,((enable<<PWM_PWMOUTEN_OUTEN_PWM0_POS)&PWM_PWMOUTEN_OUTEN_PWM0));
}

/**
 * @brief Enable for output driver PWMOUT1 or disable
 * @param  PWM_Type *base
 * @param  uint32_t enable
 * @retval none
 */
static inline void ms_pwm_cfg_pwmout1_ll(PWM_Type *base, uint32_t enable)
{
    MODIFY_REG(base->PWMOUTEN, PWM_PWMOUTEN_OUTEN_PWM1,((enable<<PWM_PWMOUTEN_OUTEN_PWM1_POS)&PWM_PWMOUTEN_OUTEN_PWM1));
}

/**
 * @brief Enable for output driver PWMOUT2 or disable
 * @param  PWM_Type *base
 * @param  uint32_t enable
 * @retval none
 */
static inline void ms_pwm_cfg_pwmout2_ll(PWM_Type *base, uint32_t enable)
{
    MODIFY_REG(base->PWMOUTEN, PWM_PWMOUTEN_OUTEN_PWM2,((enable<<PWM_PWMOUTEN_OUTEN_PWM2_POS)&PWM_PWMOUTEN_OUTEN_PWM2));
}

/**
 * @brief Configure the interrupt assert timing for PER_INT0(Period Interrupt)
 * @param  PWM_Type *base
 * @param  uint8_t type
 * @retval none
 */
static inline void ms_pwm_cfg_intas0_ll(PWM_Type *base, uint8_t type)
{
    MODIFY_REG(base->INTAS, PWM_INTAS_PWM0,((type<<PWM_INTAS_PWM0_POS)&PWM_INTAS_PWM0));
}

/**
 * @brief Configure the interrupt assert timing for PER_INT1(Period Interrupt)
 * @param  PWM_Type *base
 * @param  uint8_t type
 * @retval none
 */
static inline void ms_pwm_cfg_intas1_ll(PWM_Type *base, uint8_t type)
{
    MODIFY_REG(base->INTAS, PWM_INTAS_PWM1,((type<<PWM_INTAS_PWM1_POS)&PWM_INTAS_PWM1));
}

/**
 * @brief Configure the interrupt assert timing for PER_INT2(Period Interrupt)
 * @param  PWM_Type *base
 * @param  uint8_t type
 * @retval none
 */
static inline void ms_pwm_cfg_intas2_ll(PWM_Type *base, uint8_t type)
{
    MODIFY_REG(base->INTAS, PWM_INTAS_PWM2,((type<<PWM_INTAS_PWM2_POS)&PWM_INTAS_PWM2));
}

/**
 * @brief Enable the period interrupt for PWM0 or disable
 * @param  PWM_Type *base
 * @param  uint8_t enable
 * @retval none
 */
static inline void ms_pwm_cfg_per_int0_ll(PWM_Type *base, uint8_t enable)
{
    MODIFY_REG(base->INTEN, PWM_INTEN_PERIOD_INT_PWM0,((enable<<PWM_INTEN_PERIOD_INT_PWM0_POS)&PWM_INTEN_PERIOD_INT_PWM0));
}

/**
 * @brief Enable the period interrupt for PWM1 or disable
 * @param  PWM_Type *base
 * @param  uint8_t enable
 * @retval none
 */
static inline void ms_pwm_cfg_per_int1_ll(PWM_Type *base, uint8_t enable)
{
    MODIFY_REG(base->INTEN, PWM_INTEN_PERIOD_INT_PWM1,((enable<<PWM_INTEN_PERIOD_INT_PWM1_POS)&PWM_INTEN_PERIOD_INT_PWM1));
}

/**
 * @brief Enable the period interrupt for PWM2 or disable
 * @param  PWM_Type *base
 * @param  uint8_t enable
 * @retval none
 */
static inline void ms_pwm_cfg_per_int2_ll(PWM_Type *base, uint8_t enable)
{
    MODIFY_REG(base->INTEN, PWM_INTEN_PERIOD_INT_PWM2,((enable<<PWM_INTEN_PERIOD_INT_PWM2_POS)&PWM_INTEN_PERIOD_INT_PWM2));
}

/**
 * @brief Enable the flash time interrupt for PWM0 or disable
 * @param  PWM_Type *base
 * @param  uint8_t enable
 * @retval none
 */
static inline void ms_pwm_cfg_fla_int0_ll(PWM_Type *base, uint8_t enable)
{
    MODIFY_REG(base->INTEN, PWM_INTEN_FLASH_TIM_INT_PWM0,((enable<<PWM_INTEN_FLASH_TIM_INT_PWM0_POS)&PWM_INTEN_FLASH_TIM_INT_PWM0));
}

/**
 * @brief Enable the flash time interrupt for PWM1 or disable
 * @param  PWM_Type *base
 * @param  uint8_t enable
 * @retval none
 */
static inline void ms_pwm_cfg_fla_int1_ll(PWM_Type *base, uint8_t enable)
{
    MODIFY_REG(base->INTEN, PWM_INTEN_FLASH_TIM_INT_PWM1,((enable<<PWM_INTEN_FLASH_TIM_INT_PWM1_POS)&PWM_INTEN_FLASH_TIM_INT_PWM1));
}

/**
 * @brief Enable the flash time interrupt for PWM2 or disable
 * @param  PWM_Type *base
 * @param  uint8_t enable
 * @retval none
 */
static inline void ms_pwm_cfg_fla_int2_ll(PWM_Type *base, uint8_t enable)
{
    MODIFY_REG(base->INTEN, PWM_INTEN_FLASH_TIM_INT_PWM2,((enable<<PWM_INTEN_FLASH_TIM_INT_PWM2_POS)&PWM_INTEN_FLASH_TIM_INT_PWM2));
}

/**
 * @brief Get the interrupt status
 * @param none
 * @retval Interrupt status
 */
static inline uint8_t ms_pwm_get_int_status_ll()
{
    return (uint8_t)READ_REG(PWM->INTSR);
}

/**
 * @brief Clear the interrupt status
 * @param  PWM_Type *base
 * @param  uint32_t clearValue
 * @retval none
 */
static inline void ms_pwm_clear_int_status_ll(PWM_Type *base, uint32_t clearValue)
{
    WRITE_REG(base->INTSR, clearValue);
}

/**
 * @brief PWM enable and Clock gate enable for PWM0
 * @param  PWM_Type *base
 * @retval Counter value
 */
static inline void ms_pwm_enable_pwm0_ll(PWM_Type *base)
{
    SET_BIT(base->PWMEN, PWM_PWMEN_PWM0_MASK);
}

/**
 * @brief PWM enable and Clock gate enable for PWM1
 * @param  PWM_Type *base
 * @retval Counter value
 */
static inline void ms_pwm_enable_pwm1_ll(PWM_Type *base)
{
    SET_BIT(base->PWMEN, PWM_PWMEN_PWM1_MASK);
}

/**
 * @brief PWM enable and Clock gate enable for PWM2
 * @param  PWM_Type *base
 * @retval Counter value
 */
static inline void ms_pwm_enable_pwm2_ll(PWM_Type *base)
{
    SET_BIT(base->PWMEN, PWM_PWMEN_PWM2_MASK);
}

/**
 * @brief PWM disable and Clock gate disable for PWM0
 * @param  PWM_Type *base
 * @retval Counter value
 */
static inline void ms_pwm_disable_pwm0_ll(PWM_Type *base)
{
    CLEAR_BIT(base->PWMEN, PWM_PWMEN_PWM0_MASK);
}

/**
 * @brief PWM disable and Clock gate disable for PWM1
 * @param  PWM_Type *base
 * @retval Counter value
 */
static inline void ms_pwm_disable_pwm1_ll(PWM_Type *base)
{
    CLEAR_BIT(base->PWMEN, PWM_PWMEN_PWM1_MASK);
}

/**
 * @brief PWM disable and Clock gate disable for PWM2
 * @param  PWM_Type *base
 * @retval Counter value
 */
static inline void ms_pwm_disable_pwm2_ll(PWM_Type *base)
{
    CLEAR_BIT(base->PWMEN, PWM_PWMEN_PWM2_MASK);
}

/**
 * @brief Set the low part of self-define flash time value for PWMOUT0
 * @param  PWM_Type *base
 * @param  uint16_t timLow
 * @retval Counter value
 */
static inline void ms_pwm_set_tim0_timlow_ll(PWM_Type *base, uint16_t timLow)
{
    MODIFY_REG(base->TIM0VAL, PWM_TIM0VAL_TIM0_VAL_L,((timLow<<PWM_TIM0VAL_TIM0_VAL_L_POS)&PWM_TIM0VAL_TIM0_VAL_L));
}

/**
 * @brief Set the low part of self-define flash time value for PWMOUT1
 * @param  PWM_Type *base
 * @param  uint16_t timLow
 * @retval Counter value
 */
static inline void ms_pwm_set_tim1_timlow_ll(PWM_Type *base, uint16_t timLow)
{
    MODIFY_REG(base->TIM1VAL, PWM_TIM1VAL_TIM1_VAL_L,((timLow<<PWM_TIM1VAL_TIM1_VAL_L_POS)&PWM_TIM1VAL_TIM1_VAL_L));
}

/**
 * @brief Set the low part of self-define flash time value for PWMOUT2
 * @param  PWM_Type *base
 * @param  uint16_t timLow
 * @retval Counter value
 */
static inline void ms_pwm_set_tim2_timlow_ll(PWM_Type *base, uint16_t timLow)
{
    MODIFY_REG(base->TIM2VAL, PWM_TIM2VAL_TIM2_VAL_L,((timLow<<PWM_TIM2VAL_TIM2_VAL_L_POS)&PWM_TIM2VAL_TIM2_VAL_L));
}

#endif /* MS_PWM_H_ */
