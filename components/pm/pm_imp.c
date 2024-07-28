/**
 * Copyright © 2022 by MooreSilicon.All rights reserved
 * @file  pm_imp.c
 * @brief power management implementation
 * @author pengyu.xue
 * @date 2022-3-16
 * @version 1.0
 * @Revision
 */


#include <stdio.h>
#include <stddef.h>

#include "pm_imp.h"                    
#include "ms_pinmux_hal.h"
#include "ms_pwr.h"
#include "pwr.h"
#include "ms_interrupt.h"
#include "flash.h"
#include "log.h"
#include "gpio.h"
#include "timer.h"
#include "ms_timer.h"
#include "ms_clock_hal.h"

#ifdef PM_DEBUG_BLE_DEEP_SLEEP
ms_sleep_global_data g_sleep_data;

#define DEEP_SLEEP_TIMER 320000 * 60 // 60s
extern uint8_t app_get_connection_state(void);
extern void app_stop_advertising(void);
extern void app_disconnect(void);

extern TimerHandle_Type timer_handle[];
uint32_t dsleep_timer_init = 0;


void dsleep_timer_reach_calllback(TimerHandle_Type *tim)
{
    MS_LOGI(MS_DRIVER, "dsleep_timer_reach\r\n");
	
    g_sleep_data.is_allowed_to_enter_sleep = 1;

    ms_timer_clear_timer_interrupt_status(tim);

    ms_timer_stop(tim);
 
    if(0 == app_get_connection_state())
    {
        //stop adv
        MS_LOGI(MS_DRIVER, "dsleep_timer_reach stop_advertising\r\n");        
        app_stop_advertising();
    }
    else //conn stat
    {
         // disconn if in conn state
        MS_LOGI(MS_DRIVER, "dsleep_timer_reach disconnect\r\n");            
        app_disconnect();
    }
}

void dsleep_timer_init_calllback(TimerHandle_Type *tim)
{
    /*config the timer clock*/
    MS_CLOCK_HAL_CLK_ENABLE_TIMERx(0);
	
    ms_clock_hal_set_timer0_div(0);
	
    ms_clock_hal_peripheral_clk_div_toggle(TIMER0_DIV_TOG);

    /*disable timer interrupt*/
    ms_timer_int_disable( tim);
	
    /*enable the cpu interrupt*/
    ms_timer_enable_cpu_interrupt(tim);
    MS_LOGI(MS_DRIVER, "dsleep_timer_init callback\r\n");     
}


TimerCallback_Type  dsleep_timer_callback =
{
    .init_callback = dsleep_timer_init_calllback,
	.deinit_callback = 0,
	.timer_reach_callback = dsleep_timer_reach_calllback,
};

void dsleep_timer_start(void)
{
    
    timer_handle[0].instance = TIMER0;
    timer_handle[0].instance_conmon = TIMER;
	  
    timer_handle[0].init.period = DEEP_SLEEP_TIMER;
    timer_handle[0].init.reload_mode = TIMER_USER_DEFINED;
  
    timer_handle[0].p_callback = &dsleep_timer_callback;
    timer_handle[0].irq = TIMER_IRQn;
    ms_timer_init(&timer_handle[0]);
    ms_timer_int_enable(&timer_handle[0]);
    ms_timer_start(&timer_handle[0]);
    dsleep_timer_init = 1;
    MS_LOGI(MS_DRIVER, "dsleep_timer_start \r\n");    
    return;
}

void dsleep_timer_restart(void)
{
    if(dsleep_timer_init){
        g_sleep_data.is_allowed_to_enter_sleep = 0;   // disable deep sleep
        ms_timer_stop(&timer_handle[0]);
        ms_timer_reload(&timer_handle[0]);
        ms_timer_start(&timer_handle[0]);	
        MS_LOGI(MS_DRIVER, "dsleep_timer restart \r\n");
    }
}


//void dsleep_timer_stop(void)
//{
//   	ms_timer_stop(&timer_handle[0]);
//}


uint32_t ms_sys_check_if_enter_sleep()
{
    return g_sleep_data.is_allowed_to_enter_sleep;
}


//PWRHandle_Type pmhandle;

int32_t app_registerbefore_sleepaction(void)
{
    // store function pointer list
}

int32_t app_deregisterbefore_sleepaction(void)
{
    // delete function pointer list
}


int32_t app_registerafter_sleepaction(void)
{
   // store function pointer list
}

int32_t app_deregisterafter_sleepaction(void)
{
   // delete function pointer list
}



/*  freertos related  power management,  TBD

    1. idle task hook            call from vApplicationIdleHook
    2. freertos tick hook        call from vApplicationTickHook
    
*/



/*  wakelock related functions TBD
    create a lock type: AHB clock lock, CPU clock lock, sleep prevent,  etc///
uint32_t pm_wakelock_new()
{

}

uint32_t pm_wakelock_delete()
{

}


uint32_t pm_wakelock_apply()
{
       g_sleep_data.is_allowed_to_enter_sleep = 0;
}

uint32_t pm_wakelock_release()
{
    
}

uint32_t pm_wakelock_status_get()
{
    
}

*/





/*
    CPU frequency setting released functinos

uint32_t pm_set_cpu_working_frequency()
{

}    
*/



/* battery level *

uint32_t pm_get_battery_llevel()
{

}   

*/


/**
  * @brief  CPU enter ds callback function(Save CPU register values when system enter DS)
  * @param  None
  * @retval None
  */
void ms_cpu_store_register(void)
{
#if 0
    //NVIC store
    uint32_t i;

    cpu_store_reg[0] = NVIC->ISER[0];
    cpu_store_reg[1] = NVIC->ISPR[0];
    cpu_store_reg[2] = NVIC->IABR[0];

    //The priority of system on interrupt is 0 at first DS ENTER CB, so store and restore is skipped
    for (i = 3; i < 32; ++i)  //skip System_IRQn, WDG_IRQn, BTMAC_IRQn which are handled in rom
    {
        cpu_store_reg[i] = NVIC->IP[i];
        //ATINY_LOG(LOG_INFO,"ms_cpu_sleep_enter cpu_store_reg[%d]=%x",i,cpu_store_reg[i]);
    }

    cpu_store_reg[32] = SCB->VTOR;
    ATINY_LOG(LOG_INFO,"ms_cpu_sleep_enter cpu_store_reg[%d]=%x",i,cpu_store_reg[32]);
		
    for (i = 0; i < 39; ++i)  //skip System_IRQn, WDG_IRQn, BTMAC_IRQn which are handled in rom
    {
        vector_store_reg[i] = REG_RD(cpu_store_reg[32] + (i<<2)); //REG_WR((VTOR_NEW_ADDR + (i<<2)), REG_RD((i<<2)+VTOR_OLD_ADDR));
        //ATINY_LOG(LOG_INFO,"ms_cpu_sleep_enter cpu_store_reg[%d]=%x",i,cpu_store_reg[i]);
    }
		
    /* Save Vendor register */
    //PeriIntStoreReg = PERIPHINT->EN;
#endif
    return;
}

/**
  * @brief  CPU exit ds callback function(Resume CPU register values when system exit DS)
  * @param  None
  * @retval None
  */
void ms_cpu_restore_register(void)
{
#if 0  // restore CPU registers
    //NVIC restore
    uint32_t i;

    //Don't restore NVIC pending register, but report warning
    //NVIC->ISPR[0] = cpu_store_reg[1];
    if (cpu_store_reg[1])
    {
        //DS_PRINT_WARN1("miss interrupt: pending register: 0x%x", cpu_store_reg[1]);
    }
    //NVIC->IABR[0] = cpu_store_reg[2];

    //The priority of system on interrupt is 0 at first DS ENTER CB, so store and restore is skipped
    for (i = 3; i < 32; ++i) //skip System_IRQn, WDG_IRQn, BTMAC_IRQn which are handled in rom
    {
        //NVIC->IP[i] = cpu_store_reg[i];
    }


    for (i = 0; i < 39; ++i)  //skip System_IRQn, WDG_IRQn, BTMAC_IRQn which are handled in rom
    {
        REG_WR((cpu_store_reg[32] + (i<<2)), vector_store_reg[i]);
        //ATINY_LOG(LOG_INFO,"ms_cpu_sleep_exit addr=0x%x,value[%d]=%x",
            //(cpu_store_reg[32] + (i<<2)), i,vector_store_reg[i]);
    }

    //NVIC->ISER[0] = cpu_store_reg[0];	
    SCB->VTOR = cpu_store_reg[32];
    //NVIC->ISER[0] = cpu_store_reg[0];	
    //PERIPHINT->EN = PeriIntStoreReg;
		
	//ATINY_LOG(LOG_INFO,"ms_cpu_sleep_exit over!!!");
#endif	
    return;
}
#endif

