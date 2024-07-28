/**
 * Copyright ? 2022 by MooreSilicon.All rights reserved
 * @file   ms_pwm_hal.h
 * @brief Header file of pwm module.
 * @author qun.teng
 * @date   2022-4-25
 * @version 1.0
 * @Revision
 */


#ifndef MS_PWM_H_
#define MS_PWM_H_

#include "ms1008.h"
#include "ms_pwm_ll.h"
#include "ms_pwm_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PWM_ERROR_INVALID_CALLBACK  0x00000001U                                      /*!< Invalid Callback error  */

/**
 * @brief  PWM flash time value type
 */
typedef enum
{
    PWM_NUM_0 = 0,
    PWM_NUM_1,
    PWM_NUM_2,
    PWM_NUM_MAX
}PwmNum_Type;

/**
 * @brief  PWM interrupt enable or disable settings
 */
typedef enum
{
    PWM_DISABLE = 0,                                                                 /*!< PWM interrupt setting is disable */
    PWM_ENABLE,                                                                      /*!< PWM interrupt setting is enable */
}PwmEn_Type;

/**
 * @brief  Type for PWM period interrupt assert timing 
 */
typedef enum
{
    PWM_PERINT_ASSERT_0 = 0,                                                         /*!< PWM interrupt setting is disable */
    PWM_PERINT_ASSERT_DUT,                                                           /*!< PWM interrupt setting is enable */
}PwmPerIntAss_Type;

typedef struct
{
    uint16_t cycle;                                                                  /*!< PWM period setting */
    uint16_t duty;                                                                   /*!< PWM duty cycle settings */
    PwmTimCnt_Type cntType;                                                          /*!< PWM flash time value type setting */
    uint32_t timVal;                                                                 /*!< PWM self-defined flash time value setting */
} PwmInit_Type;

struct __PWMHandle_Type;
/**
  * @brief  Timer callback handle Structure definition
  */
typedef struct
{
    void (* error_callback)           (struct __PWMHandle_Type *pwm);
    void (* init_callback)            (struct __PWMHandle_Type *pwm);
    void (* deinit_callback)          (struct __PWMHandle_Type *pwm);
    void (* pwm_period_callback)      (struct __PWMHandle_Type *pwm);
    void (* pwm_flash_callback)       (struct __PWMHandle_Type *pwm);
}PwmCallback_Type;


typedef struct __PWMHandle_Type
{
    PWM_Type *instance;
    PwmInit_Type init;                                                               /*!< PWM communication parameters      */
    PwmNum_Type pwmNum;                                                              /*!< PWM number */
    uint32_t error_code;                                                             /*!< PWM Error code*/      
    IRQn_Type irq;
    PwmCallback_Type *p_callback;
}PWMHandle_Type;

/**
 * @brief Enable pwm cpu interrupt.
 * @param  PWmHandle_Type *pwm : Pointer to pwm module.
 * @retval none
 */
void ms_pwm_enable_cpu_interrupt(PWMHandle_Type *pwm);

/**
 * @brief Disable pwm cpu interrupt.
 * @param  PWmHandle_Type *pwm : Pointer to pwm module.
 * @retval none
 */
void ms_pwm_disable_cpu_interrupt(PWMHandle_Type *pwm);

/**
 * @brief Enable period interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_enable_period_int(PWMHandle_Type *pwm);

/**
 * @brief Disable period interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_disable_period_int(PWMHandle_Type *pwm);

/**
 * @brief Enable flash time interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_enable_flash_int(PWMHandle_Type *pwm);

/**
 * @brief Disable flash time interrupt for PWMn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_disable_flash_int(PWMHandle_Type *pwm);

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
uint8_t ms_pwm_get_int_status();

/**
 * @brief Clear pwms interrupt status
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_clear_int_status(PWMHandle_Type *pwm);

/**
 * @brief Get the value of PWMn current counter
 * @param  PWMHandle_Type *pwm
 * @retval PWMn current counter
 */
uint16_t ms_pwm_get_current_cnt(PWMHandle_Type *pwm);

/**
 * @brief Restart PWMn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_restart(PWMHandle_Type *pwm);

/**
 * @brief Enable PWMn output
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_enable_output(PWMHandle_Type *pwm);

/**
 * @brief Disable PWMn output
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_disable_output(PWMHandle_Type *pwm);

/**
 * @brief Set the type of PWMn period interrupt assert timing is 0
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_set_perint_assert_0(PWMHandle_Type *pwm);

/**
 * @brief Set the type of PWMn period interrupt assert timing is DUTn
 * @param  PWMHandle_Type *pwm
 * @retval none
 */
void ms_pwm_set_perint_assert_dut(PWMHandle_Type *pwm);

/**
 * @brief Register the pwm interrupt handler function
 * @param  PWMHandle_Type *pwm
 * @param  uint8_t intStatus
 * @retval none
 */
void ms_pwm_irq_handler(PWMHandle_Type *pwm, uint8_t intStatus);

/**
 * @brief Initial pwm module.
 * @param  PWMHandle_Type *pwm : Pointer to pwm module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_pwm_init(PWMHandle_Type *pwm);

/**
 * De-initialises an pwm interface
 * @param[in] PWMHandle_Type *pwm : Pointer to pwm module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_pwm_deinit(PWMHandle_Type *pwm);

#ifdef __cplusplus
}
#endif

#endif /* MS_PWM_H_ */