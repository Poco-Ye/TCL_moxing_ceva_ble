/*
 * pwm.h
 *
 *  Created on: 2022-4-26
 *      Author: qun.teng
 */

#ifndef PWM_H_
#define PWM_H_

#include "ms1008.h"
#include "ms_pwm.h"

/**
 * @brief Enable period interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_enable_period_int(PWMHandle_Type *pwm);

/**
 * @brief Enable flash time interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_enable_flash_int(PWMHandle_Type *pwm);

/**
 * @brief Disable period interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_disable_period_int(PWMHandle_Type *pwm);

/**
 * @brief Disable flash time interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_disable_flash_int(PWMHandle_Type *pwm);

/**
 * @brief Inital callback function setting for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm_init_callback(PWMHandle_Type *pwm);

/**
 * @brief De-init callback function setting for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm_deinit_callback(PWMHandle_Type *pwm);

/**
 * @brief PWM period interrupt callback function setting for PWM0
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm0_period_callback(PWMHandle_Type *pwm);

/**
 * @brief PWM period interrupt callback function setting for PWM1
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm1_period_callback(PWMHandle_Type *pwm);

/**
 * @brief PWM period interrupt callback function setting for PWM2
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm2_period_callback(PWMHandle_Type *pwm);

/**
 * @brief PWM flash time interrupt callback function setting for PWM0
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm0_flash_callback(PWMHandle_Type *pwm);

/**
 * @brief PWM flash time interrupt callback function setting for PWM1
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm1_flash_callback(PWMHandle_Type *pwm);

/**
 * @brief PWM flash time interrupt callback function setting for PWM2
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void pwm2_flash_callback(PWMHandle_Type *pwm);

/**
 * @brief Get the value of PWMn current counter
 * @param  PWMHandle_Type *pwm
 * @retval current counter
 */
uint16_t pwm_get_current_cnt(PWMHandle_Type *pwm);

/**
 * @brief Restart PWMn
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_restart(PWMHandle_Type *pwm);

/**
 * @brief Enable PWMn output
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_enable_output(PWMHandle_Type *pwm);

/**
 * @brief Disable PWMn output
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_disable_output(PWMHandle_Type *pwm);

/**
 * @brief Set the type of PWMn period interrupt assert timing is 0
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_set_perint_assert_0(PWMHandle_Type *pwm);

/**
 * @brief Set the type of PWMn period interrupt assert timing is DUTn
 * @param  PWMHandle_Type *pwm
 * @retval STATUS_SUCCESS
 */
int32_t pwm_set_perint_assert_dut(PWMHandle_Type *pwm);

/**
 * @brief Initial PWM0
 * @param none
 * @retval STATUS_SUCCESS
 */
int32_t pwm0_init(void);

/**
 * @brief Initial PWM1
 * @param none
 * @retval STATUS_SUCCESS
 */
int32_t pwm1_init(void);

/**
 * @brief Initial PWM2
 * @param none
 * @retval STATUS_SUCCESS
 */
int32_t pwm2_init(void);

/**
 * @brief De-init PWM0
 * @param none
 * @retval STATUS_SUCCESS
 */
int32_t pwm0_deinit(void);

/**
 * @brief De-init PWM1
 * @param none
 * @retval STATUS_SUCCESS
 */
int32_t pwm1_deinit(void);

/**
 * @brief De-init PWM2
 * @param none
 * @retval STATUS_SUCCESS
 */
int32_t pwm2_deinit(void);

/**
 * @brief PWM interrupt handler function
 * @param none
 * @retval STATUS_SUCCESS
 */
void PWM_IRQHandler(void);

#endif /* PWM_H_ */