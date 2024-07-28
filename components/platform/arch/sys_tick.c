/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  ms_sys_timer.c
 * @brief
 * @author bingrui.chen
 * @date 2022年1月17日
 * @version 1.0
 * @Revision:
 */
#include <ms_clock_hal.h>
#include "ms_interrupt.h"
#include "sys_tick.h"
#include "log.h"

#ifdef FREERTOS_SUPPORT
#include "FreeRTOS.h"
#include "task.h"
#endif

uint32_t globalcycleL;
uint32_t globalcycleH;
uint32_t globalinstructionL;
uint32_t globalinstructionH;

uint32_t ms_sys_get_cycles()
{

	uint32_t tempL;
	uint32_t tempH;
	uint32_t value;
	

	tempL = __RV_CSR_READ(CSR_MCYCLE);
	tempH = __RV_CSR_READ(CSR_MCYCLEH); 
    value =  tempL - globalcycleL;

    globalcycleL = 	tempL;
	//MS_LOGI(MS_FREERTOS,"cycles total: 0x %x %x\r\n", tempH,tempL);	
	MS_LOGI(MS_FREERTOS,"time used %dms\r\n", globalcycleL  / 48000);	// cycle*1000/48000000  ms
	return value;
}

uint32_t ms_sys_get_instruction ()
{
    uint32_t tempL;
    uint32_t tempH;	
    uint32_t value;
		
    tempL = __RV_CSR_READ(CSR_MINSTRET);
	tempH = __RV_CSR_READ(CSR_MINSTRETH); 
    value =  tempL - globalinstructionL;
	
    globalinstructionL = tempL;
    MS_LOGI(MS_FREERTOS,"instructions total:  0x %x %x\r\n", tempH,tempL);	
	return value;
}


static volatile uint32_t sys_timer_systick = 0;


__weak uint32_t ms_sys_timer_get_systick(void)
{
    return sys_timer_systick;
}

int32_t ms_sys_timer_config_systick(uint64_t ticks)
{


/*    MS_CLOCK_HAL_CLK_ENABLE_CPU_TIMER();
    //set mtime value
    SysTimer_SetLoadValue(0);
    //set mtimectl value
    SysTimer_SetCompareValue(ticks - 1);
    SysTimer_Start();
    INTERRUPT_ENABLE_IRQ(SysTimer_IRQn); */

    globalcycleL = 0;
    globalcycleH = 0;
    globalinstructionL = 0;
    globalinstructionH = 0;
	__enable_all_counter();  // for debug, should be closed in release version

    return STATUS_SUCCESS;
}

__weak void ms_sys_timer_delay(__IO uint32_t delay)
{
    uint32_t tickstart = 0;
    tickstart = ms_sys_timer_get_systick();
    while ((ms_sys_timer_get_systick() - tickstart) < delay)
    {
    }
}

extern void xPortSysTickHandler(void);

void SysTick_Handler(void)
{
/*    uint32_t tasknumber = 0;
	
	tasknumber = uxTaskGetNumberOfTasks();

    sys_timer_systick++; */

/* if FreeRTOS is running. FreeRTOS will take over system tick.
 * in task schedule function, FreeRTOS will enable the sys tick   */
/*	if(tasknumber != 0)
	{
        xPortSysTickHandler();
    }
    else
    {
        SysTimer_SetLoadValue(0);
    } */


    sys_timer_systick++;

#ifdef FREERTOS_SUPPORT
		xPortSysTickHandler();
#endif


}

void delay(unsigned int cycles)
{
    while(cycles > 0)
    {
        cycles--;
        __asm("nop");
    }
}





