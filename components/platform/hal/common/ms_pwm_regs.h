/**
 * Copyright 漏 2022 by MooreSilicon.All rights reserved
 * @file  ms_pwm_regs.h
 * @brief
 * @author qun.teng
 * @date 2022-4-24
 * @version 1.0
 * @Revision
 */
#ifndef MS_PWM_REGS_H_
#define MS_PWM_REGS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup PWM_Type PWM Registers definition
 * @{
 */
typedef struct {
    volatile uint32_t CYC0;                            /* Period set register for PWMOUT0 */
    volatile uint32_t DUT0;                            /* Duty cycle set register for PWMOUT0 */
    volatile uint32_t CNT0;                            /* Counter for PWMOUT0 */
    volatile uint32_t TIM0;                            /* Flash Time for PWMOUT0 */
    volatile uint32_t CYC1;                            /* Period set register for PWMOUT1 */
    volatile uint32_t DUT1;                            /* Duty cycle set register for PWMOUT1 */
    volatile uint32_t CNT1;                            /* Counter for PWMOUT1 */
    volatile uint32_t TIM1;                            /* Flash Time for PWMOUT1 */
    volatile uint32_t CYC2;                            /* Period set register for PWMOUT2 */
    volatile uint32_t DUT2;                            /* Duty cycle set register for PWMOUT2 */
    volatile uint32_t CNT2;                            /* Counter for PWMOUT1 */
    volatile uint32_t TIM2;                            /* Flash Time for PWMOUT1 */
    volatile uint32_t PWMOUTEN;                        /* Output enable/restart/disable register for PWMOUTn (n=0~2) */
    volatile uint32_t INTAS;                           /* Interrupt assert timing register */
    volatile uint32_t INTEN;                           /* Interrupt enable register */
    volatile uint32_t INTSR;                           /* Interrupt status register */
    volatile uint32_t PWMEN;                           /* PWM module enable/disable, clock gating bits */
    volatile uint32_t TIM0VAL;                         /* Low Part for self-defined Counter Tim0 */
    volatile uint32_t TIM1VAL;                         /* Low Part for self-defined Counter Tim1 */
    volatile uint32_t TIM2VAL;                         /* Low Part for self-defined Counter Tim2 */
}PWM_Type;

/* Period set register for PWMOUT0 Bit Define */
#define PWM_CYC0_PWMOUT0_POS                                        (0UL)
#define PWM_CYC0_PWMOUT0_MASK                                       (0xFFFFUL << PWM_CYC0_PWMOUT0_POS)
#define PWM_CYC0_PWMOUT0                                            PWM_CYC0_PWMOUT0_MASK

/* Duty cycle set register for PWMOUT0 Bit Define */
#define PWM_DUT0_PWMOUT0_POS                                        (0UL)
#define PWM_DUT0_PWMOUT0_MASK                                       (0xFFFFUL << PWM_DUT0_PWMOUT0_POS)
#define PWM_DUT0_PWMOUT0                                            PWM_DUT0_PWMOUT0_MASK

/* Counter for PWMOUT0 Bit Define */
#define PWM_CNT0_PWMOUT0_POS                                        (0UL)
#define PWM_CNT0_PWMOUT0_MASK                                       (0xFFFFUL << PWM_CNT0_PWMOUT0_POS)
#define PWM_CNT0_PWMOUT0                                            PWM_CNT0_PWMOUT0_MASK

/* Flash Time for PWMOUT0 Bit Define */
#define PWM_TIM0_PWMOUT0_POS                                        (0UL)
#define PWM_TIM0_PWMOUT0_MASK                                       (0xFUL << PWM_TIM0_PWMOUT0_POS)
#define PWM_TIM0_PWMOUT0                                            PWM_TIM0_PWMOUT0_MASK
#define PWM_TIM0_TIM0_VAL_H_POS                                     (8UL)
#define PWM_TIM0_TIM0_VAL_H_MASK                                    (0xFFUL << PWM_TIM0_TIM0_VAL_H_POS)
#define PWM_TIM0_TIM0_VAL_H                                         PWM_TIM0_TIM0_VAL_H_MASK

/* Period set register for PWMOUT1 Bit Define */
#define PWM_CYC1_PWMOUT1_POS                                        (0UL)
#define PWM_CYC1_PWMOUT1_MASK                                       (0xFFFFUL << PWM_CYC1_PWMOUT1_POS)
#define PWM_CYC1_PWMOUT1                                            PWM_CYC1_PWMOUT1_MASK

/* Duty cycle set register for PWMOUT1 Bit Define */
#define PWM_DUT1_PWMOUT1_POS                                        (0UL)
#define PWM_DUT1_PWMOUT1_MASK                                       (0xFFFFUL << PWM_DUT1_PWMOUT1_POS)
#define PWM_DUT1_PWMOUT1                                            PWM_DUT1_PWMOUT1_MASK

/* Counter for PWMOUT1 Bit Define */
#define PWM_CNT1_PWMOUT1_POS                                        (0UL)
#define PWM_CNT1_PWMOUT1_MASK                                       (0xFFFFUL << PWM_CNT1_PWMOUT1_POS)
#define PWM_CNT1_PWMOUT1                                            PWM_CNT1_PWMOUT1_MASK

/* Flash Time for PWMOUT1 Bit Define */
#define PWM_TIM1_PWMOUT1_POS                                        (0UL)
#define PWM_TIM1_PWMOUT1_MASK                                       (0xFUL << PWM_TIM1_PWMOUT1_POS)
#define PWM_TIM1_PWMOUT1                                            PWM_TIM1_PWMOUT1_MASK
#define PWM_TIM1_TIM1_VAL_H_POS                                     (8UL)
#define PWM_TIM1_TIM1_VAL_H_MASK                                    (0xFFUL << PWM_TIM1_TIM1_VAL_H_POS)
#define PWM_TIM1_TIM1_VAL_H                                         PWM_TIM1_TIM1_VAL_H_MASK

/* Period set register for PWMOUT2 Bit Define */
#define PWM_CYC2_PWMOUT2_POS                                        (0UL)
#define PWM_CYC2_PWMOUT2_MASK                                       (0xFFFFUL << PWM_CYC2_PWMOUT2_POS)
#define PWM_CYC2_PWMOUT2                                            PWM_CYC2_PWMOUT2_MASK

/* Duty cycle set register for PWMOUT2 Bit Define */
#define PWM_DUT2_PWMOUT2_POS                                        (0UL)
#define PWM_DUT2_PWMOUT2_MASK                                       (0xFFFFUL << PWM_DUT2_PWMOUT2_POS)
#define PWM_DUT2_PWMOUT2                                            PWM_DUT2_PWMOUT2_MASK

/* Counter for PWMOUT2 Bit Define */
#define PWM_CNT2_PWMOUT2_POS                                        (0UL)
#define PWM_CNT2_PWMOUT2_MASK                                       (0xFFFFUL << PWM_CNT2_PWMOUT2_POS)
#define PWM_CNT2_PWMOUT2                                            PWM_CNT2_PWMOUT2_MASK

/* Flash Time for PWMOUT2 Bit Define */
#define PWM_TIM2_PWMOUT2_POS                                        (0UL)
#define PWM_TIM2_PWMOUT2_MASK                                       (0xFUL << PWM_TIM2_PWMOUT2_POS)
#define PWM_TIM2_PWMOUT2                                            PWM_TIM2_PWMOUT2_MASK
#define PWM_TIM2_TIM2_VAL_H_POS                                     (8UL)
#define PWM_TIM2_TIM2_VAL_H_MASK                                    (0xFFUL << PWM_TIM2_TIM2_VAL_H_POS)
#define PWM_TIM2_TIM2_VAL_H                                         PWM_TIM2_TIM2_VAL_H_MASK

/* Output enable/restart/disable register Bit Define */
#define PWM_PWMOUTEN_OUTEN_PWM0_POS                                 (0UL)
#define PWM_PWMOUTEN_OUTEN_PWM0_MASK                                (0x1UL << PWM_PWMOUTEN_OUTEN_PWM0_POS)
#define PWM_PWMOUTEN_OUTEN_PWM0                                     PWM_PWMOUTEN_OUTEN_PWM0_MASK
#define PWM_PWMOUTEN_OUTEN_PWM1_POS                                 (1UL)
#define PWM_PWMOUTEN_OUTEN_PWM1_MASK                                (0x1UL << PWM_PWMOUTEN_OUTEN_PWM1_POS)
#define PWM_PWMOUTEN_OUTEN_PWM1                                     PWM_PWMOUTEN_OUTEN_PWM1_MASK
#define PWM_PWMOUTEN_OUTEN_PWM2_POS                                 (2UL)
#define PWM_PWMOUTEN_OUTEN_PWM2_MASK                                (0x1UL << PWM_PWMOUTEN_OUTEN_PWM2_POS)
#define PWM_PWMOUTEN_OUTEN_PWM2                                     PWM_PWMOUTEN_OUTEN_PWM2_MASK
#define PWM_PWMOUTEN_RESTART_PWM0_POS                               (3UL)
#define PWM_PWMOUTEN_RESTART_PWM0_MASK                              (0x1UL << PWM_PWMOUTEN_RESTART_PWM0_POS)
#define PWM_PWMOUTEN_RESTART_PWM0                                   PWM_PWMOUTEN_RESTART_PWM0_MASK
#define PWM_PWMOUTEN_RESTART_PWM1_POS                               (4UL)
#define PWM_PWMOUTEN_RESTART_PWM1_MASK                              (0x1UL << PWM_PWMOUTEN_RESTART_PWM1_POS)
#define PWM_PWMOUTEN_RESTART_PWM1                                   PWM_PWMOUTEN_RESTART_PWM1_MASK
#define PWM_PWMOUTEN_RESTART_PWM2_POS                               (5UL)
#define PWM_PWMOUTEN_RESTART_PWM2_MASK                              (0x1UL << PWM_PWMOUTEN_RESTART_PWM2_POS)
#define PWM_PWMOUTEN_RESTART_PWM2                                   PWM_PWMOUTEN_RESTART_PWM2_MASK

/* Interrupt assert timing register Bit Define */
#define PWM_INTAS_PWM0_POS                                          (0UL)
#define PWM_INTAS_PWM0_MASK                                         (0x1UL << PWM_INTAS_PWM0_POS)
#define PWM_INTAS_PWM0                                              PWM_INTAS_PWM0_MASK
#define PWM_INTAS_PWM1_POS                                          (1UL)
#define PWM_INTAS_PWM1_MASK                                         (0x1UL << PWM_INTAS_PWM1_POS)
#define PWM_INTAS_PWM1                                              PWM_INTAS_PWM1_MASK
#define PWM_INTAS_PWM2_POS                                          (2UL)
#define PWM_INTAS_PWM2_MASK                                         (0x1UL << PWM_INTAS_PWM2_POS)
#define PWM_INTAS_PWM2                                              PWM_INTAS_PWM2_MASK

/* Interrupt enable register Bit Define */
#define PWM_INTEN_PERIOD_INT_PWM0_POS                               (0UL)
#define PWM_INTEN_PERIOD_INT_PWM0_MASK                              (0x1UL << PWM_INTEN_PERIOD_INT_PWM0_POS)
#define PWM_INTEN_PERIOD_INT_PWM0                                   PWM_INTEN_PERIOD_INT_PWM0_MASK
#define PWM_INTEN_PERIOD_INT_PWM1_POS                               (1UL)
#define PWM_INTEN_PERIOD_INT_PWM1_MASK                              (0x1UL << PWM_INTEN_PERIOD_INT_PWM1_POS)
#define PWM_INTEN_PERIOD_INT_PWM1                                   PWM_INTEN_PERIOD_INT_PWM1_MASK
#define PWM_INTEN_PERIOD_INT_PWM2_POS                               (2UL)
#define PWM_INTEN_PERIOD_INT_PWM2_MASK                              (0x1UL << PWM_INTEN_PERIOD_INT_PWM2_POS)
#define PWM_INTEN_PERIOD_INT_PWM2                                   PWM_INTEN_PERIOD_INT_PWM2_MASK
#define PWM_INTEN_FLASH_TIM_INT_PWM0_POS                            (3UL)
#define PWM_INTEN_FLASH_TIM_INT_PWM0_MASK                           (0x1UL << PWM_INTEN_FLASH_TIM_INT_PWM0_POS)
#define PWM_INTEN_FLASH_TIM_INT_PWM0                                PWM_INTEN_FLASH_TIM_INT_PWM0_MASK
#define PWM_INTEN_FLASH_TIM_INT_PWM1_POS                            (4UL)
#define PWM_INTEN_FLASH_TIM_INT_PWM1_MASK                           (0x1UL << PWM_INTEN_FLASH_TIM_INT_PWM1_POS)
#define PWM_INTEN_FLASH_TIM_INT_PWM1                                PWM_INTEN_FLASH_TIM_INT_PWM1_MASK
#define PWM_INTEN_FLASH_TIM_INT_PWM2_POS                            (5UL)
#define PWM_INTEN_FLASH_TIM_INT_PWM2_MASK                           (0x1UL << PWM_INTEN_FLASH_TIM_INT_PWM2_POS)
#define PWM_INTEN_FLASH_TIM_INT_PWM2                                PWM_INTEN_FLASH_TIM_INT_PWM2_MASK

/* Interrupt status register Bit Define */
#define PWM_INTSR_PERIOD_INT_PWM0_POS                               (0UL)
#define PWM_INTSR_PERIOD_INT_PWM0_MASK                              (0x1UL << PWM_INTSR_PERIOD_INT_PWM0_POS)
#define PWM_INTSR_PERIOD_INT_PWM0                                   PWM_INTSR_PERIOD_INT_PWM0_MASK
#define PWM_INTSR_PERIOD_INT_PWM1_POS                               (1UL)
#define PWM_INTSR_PERIOD_INT_PWM1_MASK                              (0x1UL << PWM_INTSR_PERIOD_INT_PWM1_POS)
#define PWM_INTSR_PERIOD_INT_PWM1                                   PWM_INTSR_PERIOD_INT_PWM1_MASK
#define PWM_INTSR_PERIOD_INT_PWM2_POS                               (2UL)
#define PWM_INTSR_PERIOD_INT_PWM2_MASK                              (0x1UL << PWM_INTSR_PERIOD_INT_PWM2_POS)
#define PWM_INTSR_PERIOD_INT_PWM2                                   PWM_INTSR_PERIOD_INT_PWM2_MASK
#define PWM_INTSR_FLASH_TIM_INT_PWM0_POS                            (3UL)
#define PWM_INTSR_FLASH_TIM_INT_PWM0_MASK                           (0x1UL << PWM_INTSR_FLASH_TIM_INT_PWM0_POS)
#define PWM_INTSR_FLASH_TIM_INT_PWM0                                PWM_INTSR_FLASH_TIM_INT_PWM0_MASK
#define PWM_INTSR_FLASH_TIM_INT_PWM1_POS                            (4UL)
#define PWM_INTSR_FLASH_TIM_INT_PWM1_MASK                           (0x1UL << PWM_INTSR_FLASH_TIM_INT_PWM1_POS)
#define PWM_INTSR_FLASH_TIM_INT_PWM1                                PWM_INTSR_FLASH_TIM_INT_PWM1_MASK
#define PWM_INTSR_FLASH_TIM_INT_PWM2_POS                            (5UL)
#define PWM_INTSR_FLASH_TIM_INT_PWM2_MASK                           (0x1UL << PWM_INTSR_FLASH_TIM_INT_PWM2_POS)
#define PWM_INTSR_FLASH_TIM_INT_PWM2                                PWM_INTSR_FLASH_TIM_INT_PWM2_MASK

/* PWM module enable Bit Define */
#define PWM_PWMEN_PWM0_POS                                          (0UL)
#define PWM_PWMEN_PWM0_MASK                                         (0x1UL << PWM_PWMEN_PWM0_POS)
#define PWM_PWMEN_PWM0                                              PWM_PWMEN_PWM0_MASK
#define PWM_PWMEN_PWM1_POS                                          (1UL)
#define PWM_PWMEN_PWM1_MASK                                         (0x1UL << PWM_PWMEN_PWM1_POS)
#define PWM_PWMEN_PWM1                                              PWM_PWMEN_PWM1_MASK
#define PWM_PWMEN_PWM2_POS                                          (2UL)
#define PWM_PWMEN_PWM2_MASK                                         (0x1UL << PWM_PWMEN_PWM2_POS)
#define PWM_PWMEN_PWM2                                              PWM_PWMEN_PWM2_MASK

/* Low Part for self-defined Counter Tim0 */
#define PWM_TIM0VAL_TIM0_VAL_L_POS                                  (0UL)
#define PWM_TIM0VAL_TIM0_VAL_L_MASK                                 (0xFFFFUL << PWM_TIM0VAL_TIM0_VAL_L_POS)
#define PWM_TIM0VAL_TIM0_VAL_L                                      PWM_TIM0VAL_TIM0_VAL_L_MASK

/* Low Part for self-defined Counter Tim1 */
#define PWM_TIM1VAL_TIM1_VAL_L_POS                                  (0UL)
#define PWM_TIM1VAL_TIM1_VAL_L_MASK                                 (0xFFFFUL << PWM_TIM1VAL_TIM1_VAL_L_POS)
#define PWM_TIM1VAL_TIM1_VAL_L                                      PWM_TIM1VAL_TIM1_VAL_L_MASK

/* Low Part for self-defined Counter Tim2 */
#define PWM_TIM2VAL_TIM2_VAL_L_POS                                  (0UL)
#define PWM_TIM2VAL_TIM2_VAL_L_MASK                                 (0xFFFFUL << PWM_TIM2VAL_TIM2_VAL_L_POS)
#define PWM_TIM2VAL_TIM2_VAL_L                                      PWM_TIM2VAL_TIM2_VAL_L_MASK

#ifdef __cplusplus
}
#endif

#endif /* MS_PWM_REGS_H_ */
