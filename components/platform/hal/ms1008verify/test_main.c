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

#include "ms_clock_hal.h"
#include "ms_pinmux_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "ms_keyscan.h"


//#include "ms_keyscan.h"


#ifdef KEYPAD_TO2

void app_keypad_signal_key_process()
{
    MS_LOGI(MS_DRIVER,"KEYPAD_TO2 key pressed\r\n");
}

#else
void app_keypad_callback()
{
    MS_LOGI(MS_DRIVER,"key pressed\r\n"); 
}
#endif 

#ifdef KEYPAD_TO2
static kpad_config_type cfg;
static keyscan_process_func kpad_app;
#endif

void test_keypad2(void)
{
#ifdef KEYPAD_TO2
    kpad_app.func_release = 0;
    kpad_app.func_signal_press = app_keypad_signal_key_process;
    kpad_app.func_muil2_press = 0;
    kpad_app.func_mutil3_press = 0;

    ms_keyscan_default_config(&cfg);
    keyscan_module_init(&cfg, &kpad_app);
#else
    ms_keyscan_init(app_keypad_callback);
#endif
}






#define TESTTASK_STACK_DEPTH  (2*1024)

extern void unity_test_init();



static TaskHandle_t testmaintaskHandler;
static void starttesttask(void* pvParameters);

static void starttesttask(void* pvParameters)
{
   // MS_LOGI(MS_DRIVER, "starttesttask\r\n");

    unity_test_init();
  //  test_keypad2();
    log_level_set(MS_DRIVER, MS_LOG_DEBUG);
    log_level_set(MS_FREERTOS, MS_LOG_DEBUG);
    MS_LOGI(MS_DRIVER, "Unit test menu start\r\n");
    unity_run_menu();
}



void system_clock_init(void)
{
	SysOscInit_Type osc_init;
    osc_init.hf_osc_src = HF_OSC_CLK_SEL_EXT_OSC24M;
    osc_init.sys_hf_clk_src = HF_CLK_SEL_BYPASS_PLL;
    osc_init.sys_lf_clk_src = LF_CLK_SEL_RC_OSC32K;
    osc_init.ext_osc_div = 1;
    osc_init.pll.mul = 1;
    osc_init.pll.pdiv = 0;
    ms_clock_hal_sys_config_osc(&osc_init);

    SysClockInit_Type clock_init;
    clock_init.sys_clk_src = SYS_CLK_SEL_HF_CLK;
    clock_init.ahb_div = 0;
    clock_init.apb_div = 0;
    clock_init.peri_clk_div = 0;
    ms_clock_hal_config_clock(&clock_init);
}


extern void ms_pinmux_hal_config_default();

int main(void)
{

    system_clock_init ();
    ms_pinmux_hal_config_default();
    ms_dmac_mgmt_inti();

    uart0_int();
    MS_LOGI(MS_DRIVER, "uart0_int\r\n");

    xTaskCreate((TaskFunction_t)starttesttask, (const char*)"starttesttask",
                (uint16_t)TESTTASK_STACK_DEPTH, (void*)NULL, (UBaseType_t)1,
                (TaskHandle_t*)&testmaintaskHandler); 
	
    vTaskStartScheduler();
    return 0;
}



