
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

#include "ms_clock_hal.h"
#include "ms_keyscan.h"
#include "ms_audio_adc.h"
#include "ir1.h"
#include "app_keypad.h"
#include "flash.h"
#include "pm_sleep.h"
static uint8_t  key_table1[5][6] = {
    IR_MUTE,       IR_POWER,      IR_EXIT,        IR_SETTING, IR_HOME,   IR_ARROW_UP, //K1 ~ K6
    IR_ARROW_DOWN, IR_ARROW_LEFT, IR_ARROW_RIGHT, IR_OK,      IR_VOL_UP, IR_VOL_DOWN, //K7 ~ K12
    IR_0,          IR_1,          IR_2,           IR_3,       IR_4,      IR_5, //K13 ~ K18
    IR_6,          IR_7,          IR_8,           IR_9,       IR_NEXT,   IR_NEXT, //K19 ~ K24
    IR_NEXT,       IR_NEXT,       IR_NEXT,        IR_NEXT,    IR_NEXT,   IR_MIC  //K25 ~ K30
};


static key_mutil2_type key_table2[2] = {
    {IR_PAIR,  {1, 3}, {2, 4}},              //just test
    {0x22,  {1, 2}, {1, 1}}              //just test
};


static key_mutil3_type key_table3[1] = {
    {0x33,  {1, 5}, {1, 4}, {1, 3}}  //just test
};


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
extern void unity_test_init();
extern uint8_t app_get_connection_state(void);
#define TESTTASK_STACK_DEPTH 2*1024  // test main stack 2K * 4


void ir_process(uint8_t connection_state,int8_t data,int8_t event)
{

    #ifdef USE_IR_RCA
    int8_t  userCodes;
    userCodes  = TCL_RCA_USER_CODE;
#endif





   if(connection_state == 0)
   {
       if( KPAD_EVENT_KEY_PRESS == event)	
       {
    #ifdef USE_IR_RCA
    		    ir_rca_send_frame(userCodes, data);
    #endif
    
    #ifdef USE_IR_NEC
    
    		    ir_nec_send_frame(IR_DEV_ADDR,data);
    #endif

    
       }
   else if(KPAD_EVENT_KEY_RELEASE == event)
       {
    
    #ifdef USE_IR_RCA
    		    ir_rca_send_frame_repeat_stop();
    #endif
    
    #ifdef USE_IR_NEC
    
    		   ir_nec_send_frame_repeat_stop();
    #endif
       
       
    
       }else if(KPAD_EVENT_KEY_LONG_PRESS == event)
       {
    
    #ifdef USE_IR_RCA
    		    ir_rca_send_frame_repeat(userCodes, data);
    #endif
    
    #ifdef USE_IR_NEC
    
    		    ir_nec_send_frame_repeat(IR_DEV_ADDR,data);
    #endif
       
    
       }


   }
	
  

}



void app_keypad_report_key(uint32_t key_value, uint32_t type);


void app_keypad_just_key_release(key_signal_type key)
{
    MS_LOGI(MS_DRIVER, "\r\njust release app: key map = %#x,  row= %#x, col= %#x,event=%d\r\n", 
        key.map_val,  key.key.row, key.key.col, key.event);
    app_keypad_report_key(key.map_val, key.event);
    ir_process(app_get_connection_state(), key.map_val,key.event);
}

void app_keypad_signal_key_process(key_signal_type key)
{
#ifndef BT_SUPPORT
    MS_LOGI(MS_DRIVER, "\r\nsignal press app: key map = %#x,  row= %#x, col= %#x,event=%d\r\n", 
        key.map_val,  key.key.row, key.key.col, key.event);
#endif
    app_keypad_report_key(key.map_val, key.event);

    ir_process(app_get_connection_state(), key.map_val,key.event);
    if((key.map_val == 0xD5) && (app_get_connection_state()))
    {
        ms_audio_adc_start_dma_xfer();
    }
}

void app_keypad_mutil2_key_process(key_mutil2_type km2)
{
      MS_LOGI(MS_DRIVER, "mutil2 press app:key map = %#x,  row1= %#x, col1= %#x, row2= %#x, col2= %#x\r\n", 
		km2.map_val, km2.key1.row, km2.key1.col, km2.key2.row, km2.key2.col);
      app_keypad_report_key(km2.map_val, 1);
         //ir_process(app_get_connection_state(), km2.map_val,key.event);
}

void app_keypad_mutil3_key_process(key_mutil3_type km3)
{


    MS_LOGI(MS_DRIVER, "mutil3 press app:key map =%#x,  row1= %#x, col1= %#x, row2= %#x, col2= %#x, row3= %#x, col3= %#x\r\n", 
		km3.map_val, km3.key1.row, km3.key1.col, km3.key2.row, km3.key2.col, km3.key3.row, km3.key3.col);
      //ir_process(app_get_connection_state(), km2.map_val,key.event);
		 
	
}



static TaskHandle_t testmaintaskHandler;
static void starttesttask(void* pvParameters);
extern int blemain();
extern void sys_init(void);
extern void user_clk_cfg(void);
extern void main_init(PWR_MODE_Type startmode);

FLASH_Handle_Type msflash_handle;

static void starttesttask(void* pvParameters)
{
    blemain();
}

int main(void)
{
    PM_PWR_MODE_Type startmode;
    PM_WKUP_Type wakeuptype;

    system_clock_init ();
    ms_pinmux_hal_config_default();
    ms_dmac_mgmt_inti();

    uart0_int();
    MS_LOGI(MS_DRIVER, "uart0_int\r\n");

    sys_init();
    MS_LOGI(MS_DRIVER, "sys_init\r\n");

    msflash_handle.flash_type = FLASH_QSPI_TYPE;
    flash_init(&msflash_handle);   // used in the sleep mode.
    pm_sys_init();

    startmode = pm_sys_get_pre_mode();
    MS_LOGI(MS_POWERMANAGE, "startmode:  %d\r\n", startmode);

    MS_LOGI(MS_POWERMANAGE, "wakup resource regiser %x\r\n", SYS_CTRL->WKUP_CSR);
    wakeuptype = pm_sys_get_wakeup_resource();
    MS_LOGI(MS_POWERMANAGE, "wakup resource %d\r\n",wakeuptype);

#ifdef KEYPAD_TO2
    kpad_config_type cfg;
    keyscan_process_func kpad_app;

    kpad_app.func_release = app_keypad_just_key_release;
    kpad_app.func_signal_press = app_keypad_signal_key_process;
    kpad_app.func_muil2_press = app_keypad_mutil2_key_process;
    kpad_app.func_mutil3_press = app_keypad_mutil3_key_process;

    ms_keypad_signal_map_register((void **)key_table1, 5, 6);
    ms_keypad_mutil2_map_register(key_table2,  2);
    ms_keypad_mutil3_map_register(key_table3,  1);

    ms_keyscan_default_config(&cfg);

    keyscan_module_init(&cfg, &kpad_app);

#else
    ms_keyscan_init(app_keypad_report_key);
#endif

#ifdef IR1_TO2
#ifdef USE_IR_RCA
    ir_rca_init();
#endif
#ifdef USE_IR_NEC
    ir_nec_init();
#endif
#endif

#ifdef CFG_ROM_VT
    my_patch();
#endif // CFG_ROM_VT

    if(PM_PWR_MODE_POWERDOWN == startmode)
    {
        //user clk config
        user_clk_cfg();
        //user peri init
        //user_peri_init();
        main_init(startmode);
        MS_LOGI(MS_POWERMANAGE,"app init!!!!\r\n");
    }
    else
    {
        switch (wakeuptype)
        {
            case PM_WKUP_GPIO:
                MS_LOGI(MS_POWERMANAGE, "wakeup by GPIO ++++++++++\r\n");
                break;

            case PM_WKUP_UART2:
                MS_LOGI(MS_POWERMANAGE, "wakeup by UART2 ++++++++++\r\n");
                break;

            case PM_WKUP_RTC:
                MS_LOGI(MS_POWERMANAGE, "wakeup by RTC ++++++++++\r\n");
                break;

            case PM_WKUP_BLE:
                MS_LOGI(MS_POWERMANAGE, "wakeup by BLE Timer ++++++++++\r\n");
#ifdef PM_DEBUG_BLE_WKUP
                user_clk_cfg();
                pm_sys_exit_dsleep();
                main_init(startmode);
                pm_sys_enter_dsleep();
                goto LOOP_END;
#endif
                break;

            default:
                MS_LOGI(MS_POWERMANAGE, "wakeup unknow type !!!!!!!!!!!!!!!!!!\r\n");
                break;
        }

        user_clk_cfg();
        pm_sys_exit_dsleep();
        main_init(startmode);
        MS_LOGI(MS_POWERMANAGE,"app wakeup!!!!\r\n");
    }

    MS_LOGI(MS_FREERTOS, "Start BLE task ++++++++++\r\n");
    xTaskCreate((TaskFunction_t)starttesttask, (const char*)"starttesttask",
                (uint16_t)TESTTASK_STACK_DEPTH, (void*)NULL, (UBaseType_t)configMAX_PRIORITIES - 1,
                (TaskHandle_t*)&testmaintaskHandler);

    MS_LOGI(MS_FREERTOS, "vTaskStartScheduler ++++++++++\r\n");
    vTaskStartScheduler();

LOOP_END:
    while(1);
    return 0;
}












