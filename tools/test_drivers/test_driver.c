
/*
 * test_main.c
 *
 *  Created on: 2021年12月21日
 *      Author: bingrui.chen
 */

#include <ms1008.h>
#include <string.h>
#include "ms_uart.h"
#include "unity.h"
#include "uart.h"
#include "pwr.h"
#include "unity_test_runner.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"


#include "ms_keyscan.h"


#define TESTTASK_STACK_DEPTH  (2*1024)

extern void system_clock_init(void);
extern void ms_pinmux_hal_config_default();
extern void unity_test_init();
extern void test_keypad2(void);


static TaskHandle_t testmaintaskHandler;
static void starttesttask(void* pvParameters);

static void starttesttask(void* pvParameters)
{
    unity_test_init();
    test_keypad2();
    log_level_set(MS_DRIVER, MS_LOG_DEBUG);
    log_level_set(MS_FREERTOS, MS_LOG_DEBUG);
    MS_LOGI(MS_DRIVER, "Unit test menu start\r\n");
    unity_run_menu();
}


int main(void)
{

    system_clock_init ();
    ms_pinmux_hal_config_default();
    ms_dmac_mgmt_inti();

    uart0_int();
    MS_LOGI(MS_DRIVER, "uart0_int\r\n");

    xTaskCreate((TaskFunction_t)starttesttask, (const char*)"starttesttask",
                (uint16_t)TESTTASK_STACK_DEPTH, (void*)NULL, (UBaseType_t)configMAX_PRIORITIES - 1,
                (TaskHandle_t*)&testmaintaskHandler); 
	
    vTaskStartScheduler();

    return 0;
}












