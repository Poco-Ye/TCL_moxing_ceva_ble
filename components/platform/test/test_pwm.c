/*
 * test_pwm.c
 *
 *  Created on: 2022-4-27
 *      Author:qun.teng
 */

#include "uart.h"
#include <string.h>
#include "ms_uart.h"
#include "ms_clock_hal.h"
#include "ms_pinmux_hal.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pwm.h"
#include "ms_pwm.h"

extern PWMHandle_Type pwm_handle[PWM_NUM_MAX];

#define  PWM1_PWM2_FLASH_TIME_INT     0x30

TEST_CASE("pwm","test_pwm0 configure", "[Driver/pwm_0]")
{
    MS_LOGI(MS_DRIVER, "\r\npwm 0 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm0_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_enable_period_int(&pwm_handle[0]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_enable_flash_int(&pwm_handle[0]));
    TEST_ASSERT_EQUAL_INT32(0, pwm_get_current_cnt(&pwm_handle[0]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_set_perint_assert_0(&pwm_handle[0]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_enable_output(&pwm_handle[0]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_restart(&pwm_handle[0]));

    MS_LOGI(MS_DRIVER, "CYC0 = %x\r\n",PWM->CYC0);
    MS_LOGI(MS_DRIVER, "DUT0 = %x\r\n",PWM->DUT0);
    MS_LOGI(MS_DRIVER, "CNT0 = %x\r\n",PWM->CNT0);
    MS_LOGI(MS_DRIVER, "TIM0 = %x\r\n",PWM->TIM0);
    MS_LOGI(MS_DRIVER, "PWMEN = %x\r\n",PWM->PWMEN);
    MS_LOGI(MS_DRIVER, "PWMOUTEN = %x\r\n",PWM->PWMOUTEN);
    MS_LOGI(MS_DRIVER, "INTAS = %x\r\n",PWM->INTAS);
    MS_LOGI(MS_DRIVER, "INTEN = %x\r\n",PWM->INTEN);
    MS_LOGI(MS_DRIVER, "INTSR = %x\r\n",PWM->INTSR);
    MS_LOGI(MS_DRIVER, "TIM0VAL = %x\r\n",PWM->TIM0VAL);

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_disable_period_int(&pwm_handle[0]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_disable_flash_int(&pwm_handle[0]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_set_perint_assert_dut(&pwm_handle[0]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_disable_output(&pwm_handle[0]));
    MS_LOGI(MS_DRIVER, "PWMOUTEN = %x\r\n",PWM->PWMOUTEN);
    MS_LOGI(MS_DRIVER, "INTAS = %x\r\n",PWM->INTAS);
    MS_LOGI(MS_DRIVER, "INTEN = %x\r\n",PWM->INTEN);

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm0_deinit());
    MS_LOGI(MS_DRIVER, "Deinit CYC0 = %x\r\n",PWM->CYC0);
    MS_LOGI(MS_DRIVER, "Deinit DUT0 = %x\r\n",PWM->DUT0);
    MS_LOGI(MS_DRIVER, "Deinit CNT0 = %x\r\n",PWM->CNT0);
    MS_LOGI(MS_DRIVER, "Deinit TIM0 = %x\r\n",PWM->TIM0);
    MS_LOGI(MS_DRIVER, "Deinit PWMEN = %x\r\n",PWM->PWMEN);
    MS_LOGI(MS_DRIVER, "Deinit PWMOUTEN = %x\r\n",PWM->PWMOUTEN);
    MS_LOGI(MS_DRIVER, "Deinit INTAS = %x\r\n",PWM->INTAS);
    MS_LOGI(MS_DRIVER, "Deinit INTEN = %x\r\n",PWM->INTEN);
    MS_LOGI(MS_DRIVER, "Deinit INTSR = %x\r\n",PWM->INTSR);
    MS_LOGI(MS_DRIVER, "Deinit TIM0VAL = %x\r\n",PWM->TIM0VAL);
}

TEST_CASE("pwm","test_pwm1 configure", "[Driver/pwm_1]")
{
    MS_LOGI(MS_DRIVER, "\r\npwm 1 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm1_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_enable_period_int(&pwm_handle[1]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_enable_flash_int(&pwm_handle[1]));
    TEST_ASSERT_EQUAL_INT32(0, pwm_get_current_cnt(&pwm_handle[1]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_set_perint_assert_0(&pwm_handle[1]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_enable_output(&pwm_handle[1]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_restart(&pwm_handle[1]));

    MS_LOGI(MS_DRIVER, "CYC1 = %x\r\n",PWM->CYC1);
    MS_LOGI(MS_DRIVER, "DUT1 = %x\r\n",PWM->DUT1);
    MS_LOGI(MS_DRIVER, "CNT1 = %x\r\n",PWM->CNT1);
    MS_LOGI(MS_DRIVER, "TIM1 = %x\r\n",PWM->TIM1);
    MS_LOGI(MS_DRIVER, "PWMEN = %x\r\n",PWM->PWMEN);
    MS_LOGI(MS_DRIVER, "PWMOUTEN = %x\r\n",PWM->PWMOUTEN);
    MS_LOGI(MS_DRIVER, "INTAS = %x\r\n",PWM->INTAS);
    MS_LOGI(MS_DRIVER, "INTEN = %x\r\n",PWM->INTEN);
    MS_LOGI(MS_DRIVER, "INTSR = %x\r\n",PWM->INTSR);
    MS_LOGI(MS_DRIVER, "TIM0VAL = %x\r\n",PWM->TIM0VAL);

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_disable_period_int(&pwm_handle[1]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_disable_flash_int(&pwm_handle[1]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_set_perint_assert_dut(&pwm_handle[1]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_disable_output(&pwm_handle[1]));
    MS_LOGI(MS_DRIVER, "PWMOUTEN = %x\r\n",PWM->PWMOUTEN);
    MS_LOGI(MS_DRIVER, "INTAS = %x\r\n",PWM->INTAS);
    MS_LOGI(MS_DRIVER, "INTEN = %x\r\n",PWM->INTEN);

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm1_deinit());
    MS_LOGI(MS_DRIVER, "Deinit CYC1 = %x\r\n",PWM->CYC1);
    MS_LOGI(MS_DRIVER, "Deinit DUT1 = %x\r\n",PWM->DUT1);
    MS_LOGI(MS_DRIVER, "Deinit CNT1 = %x\r\n",PWM->CNT1);
    MS_LOGI(MS_DRIVER, "Deinit TIM1 = %x\r\n",PWM->TIM1);
    MS_LOGI(MS_DRIVER, "Deinit PWMEN = %x\r\n",PWM->PWMEN);
    MS_LOGI(MS_DRIVER, "Deinit PWMOUTEN = %x\r\n",PWM->PWMOUTEN);
    MS_LOGI(MS_DRIVER, "Deinit INTAS = %x\r\n",PWM->INTAS);
    MS_LOGI(MS_DRIVER, "Deinit INTEN = %x\r\n",PWM->INTEN);
    MS_LOGI(MS_DRIVER, "Deinit INTSR = %x\r\n",PWM->INTSR);
    MS_LOGI(MS_DRIVER, "Deinit TIM0VAL = %x\r\n",PWM->TIM0VAL);
}

TEST_CASE("pwm","test_pwm2 configure", "[Driver/pwm_2]")
{
    MS_LOGI(MS_DRIVER, "\r\npwm 2 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm2_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_enable_period_int(&pwm_handle[2]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_enable_flash_int(&pwm_handle[2]));
    TEST_ASSERT_EQUAL_INT32(0, pwm_get_current_cnt(&pwm_handle[2]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_set_perint_assert_0(&pwm_handle[2]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_enable_output(&pwm_handle[2]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_restart(&pwm_handle[2]));

    MS_LOGI(MS_DRIVER, "CYC2 = %x\r\n",PWM->CYC2);
    MS_LOGI(MS_DRIVER, "DUT2 = %x\r\n",PWM->DUT2);
    MS_LOGI(MS_DRIVER, "CNT2 = %x\r\n",PWM->CNT2);
    MS_LOGI(MS_DRIVER, "TIM2 = %x\r\n",PWM->TIM2);
    MS_LOGI(MS_DRIVER, "PWMEN = %x\r\n",PWM->PWMEN);
    MS_LOGI(MS_DRIVER, "PWMOUTEN = %x\r\n",PWM->PWMOUTEN);
    MS_LOGI(MS_DRIVER, "INTAS = %x\r\n",PWM->INTAS);
    MS_LOGI(MS_DRIVER, "INTEN = %x\r\n",PWM->INTEN);
    MS_LOGI(MS_DRIVER, "INTSR = %x\r\n",PWM->INTSR);
    MS_LOGI(MS_DRIVER, "TIM0VAL = %x\r\n",PWM->TIM0VAL);

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_disable_period_int(&pwm_handle[2]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_disable_flash_int(&pwm_handle[2]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_set_perint_assert_dut(&pwm_handle[2]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_disable_output(&pwm_handle[2]));
    MS_LOGI(MS_DRIVER, "PWMOUTEN = %x\r\n",PWM->PWMOUTEN);
    MS_LOGI(MS_DRIVER, "INTAS = %x\r\n",PWM->INTAS);
    MS_LOGI(MS_DRIVER, "INTEN = %x\r\n",PWM->INTEN);

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm2_deinit());
    MS_LOGI(MS_DRIVER, "Deinit CYC2 = %x\r\n",PWM->CYC2);
    MS_LOGI(MS_DRIVER, "Deinit DUT2 = %x\r\n",PWM->DUT2);
    MS_LOGI(MS_DRIVER, "Deinit CNT2 = %x\r\n",PWM->CNT2);
    MS_LOGI(MS_DRIVER, "Deinit TIM2 = %x\r\n",PWM->TIM2);
    MS_LOGI(MS_DRIVER, "Deinit PWMEN = %x\r\n",PWM->PWMEN);
    MS_LOGI(MS_DRIVER, "Deinit PWMOUTEN = %x\r\n",PWM->PWMOUTEN);
    MS_LOGI(MS_DRIVER, "Deinit INTAS = %x\r\n",PWM->INTAS);
    MS_LOGI(MS_DRIVER, "Deinit INTEN = %x\r\n",PWM->INTEN);
    MS_LOGI(MS_DRIVER, "Deinit INTSR = %x\r\n",PWM->INTSR);
    MS_LOGI(MS_DRIVER, "Deinit TIM0VAL = %x\r\n",PWM->TIM0VAL);
}

TEST_CASE("pwm","test_pwm0~2 working togerther", "[Driver/pwm_0_1_2]")
{
    pwm_handle[0].init.cntType = PWM_TIM_VALUE_13;
    pwm_handle[1].init.cntType = PWM_TIM_VALUE_5;
    pwm_handle[1].init.cycle = 400;
    pwm_handle[1].init.duty = 300;
    pwm_handle[2].init.cntType = PWM_TIM_VALUE_4;
    pwm_handle[2].init.cycle = 40;
    pwm_handle[2].init.duty = 10;

    MS_LOGI(MS_DRIVER, "\r\npwm 0~2 start output!\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm0_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm1_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm2_init());

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_enable_output(&pwm_handle[0]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_enable_output(&pwm_handle[1]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_enable_output(&pwm_handle[2]));

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_enable_flash_int(&pwm_handle[1]));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_enable_flash_int(&pwm_handle[2]));

    /* Set for pwm1/pwm2 flash time interrupt flag */
    ms_pwm_disable_cpu_interrupt(&pwm_handle[1]);
    MS_LOGI(MS_DRIVER, "CYC0 = %x\r\n",PWM->CYC0);
    MS_LOGI(MS_DRIVER, "DUT0 = %x\r\n",PWM->DUT0);
    MS_LOGI(MS_DRIVER, "CNT0 = %x\r\n",PWM->CNT0);
    MS_LOGI(MS_DRIVER, "TIM0 = %x\r\n",PWM->TIM0);
    MS_LOGI(MS_DRIVER, "CYC1 = %x\r\n",PWM->CYC1);
    MS_LOGI(MS_DRIVER, "DUT1 = %x\r\n",PWM->DUT1);
    MS_LOGI(MS_DRIVER, "CNT1 = %x\r\n",PWM->CNT1);
    MS_LOGI(MS_DRIVER, "TIM1 = %x\r\n",PWM->TIM1);
    MS_LOGI(MS_DRIVER, "CYC2 = %x\r\n",PWM->CYC2);
    MS_LOGI(MS_DRIVER, "DUT2 = %x\r\n",PWM->DUT2);
    MS_LOGI(MS_DRIVER, "CNT2 = %x\r\n",PWM->CNT2);
    MS_LOGI(MS_DRIVER, "TIM2 = %x\r\n",PWM->TIM2);
    MS_LOGI(MS_DRIVER, "PWMEN = %x\r\n",PWM->PWMEN);
    MS_LOGI(MS_DRIVER, "PWMOUTEN = %x\r\n",PWM->PWMOUTEN);
    MS_LOGI(MS_DRIVER, "INTAS = %x\r\n",PWM->INTAS);
    MS_LOGI(MS_DRIVER, "INTEN = %x\r\n",PWM->INTEN);
    MS_LOGI(MS_DRIVER, "INTSR = %x\r\n",PWM->INTSR);
    MS_LOGI(MS_DRIVER, "TIM0VAL = %x\r\n",PWM->TIM0VAL);
    MS_LOGI(MS_DRIVER, "TIM1VAL = %x\r\n",PWM->TIM1VAL);
    MS_LOGI(MS_DRIVER, "TIM2VAL = %x\r\n",PWM->TIM2VAL);

    while(1)
    {

        while(((ms_pwm_get_int_status() & (PWM1_PWM2_FLASH_TIME_INT)) != PWM1_PWM2_FLASH_TIME_INT));
        ms_pwm_clear_int_status(&pwm_handle[1]);
        ms_pwm_clear_int_status(&pwm_handle[2]);
        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_restart(&pwm_handle[1]));
        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwm_restart(&pwm_handle[2]));
    }
}

