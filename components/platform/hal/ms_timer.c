/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_timer_.c
 * @brief c source  file of timer  module.
 * @author haijun.mai
 * @date   2021-12-30
 * @version 1.0
 * @Revision
 */
#include <ms1008.h>
#include "ms_timer_hal.h"
#include "ms_timer.h"
#include "ms_timer_regs.h"
#include "ms_clock_hal.h"
#include "ms_sys_ctrl_regs.h"
#include "ms_interrupt.h"
#include <stddef.h>


void ms_timer_enable_cpu_interrupt(TimerHandle_Type *tim)
{
    INTERRUPT_ENABLE_IRQ(tim->irq);
}


void ms_timer_disable_cpu_interrupt(TimerHandle_Type *tim)
{
    INTERRUPT_DISABLE_IRQ(tim->irq);
}


/**
 * @brief enable timer n  interrupt 
 * @param  TimerConfig_Type *timer     
 * @retval none
 */
void ms_timer_int_enable(TimerHandle_Type *tim)
{
    ms_timer_int_enable_hal( tim->instance);
}


/**
 * @brief disable timer n  interrupt 
 * @param  TimerConfig_Type *timer     
 * @retval none
 */
void ms_timer_int_disable(TimerHandle_Type *tim)
{
    ms_timer_int_disable_hal( tim->instance);
}



/**
 * @brief get timers  interrupt status
 * @param  TimerHandle_Type *tim 
 * @param  uint32_t timern_mask
 * @param   This parameter can be one of the following values:
 * @param TIMER0_TIS_STATUS
 * @param TIMER1_TIS_STATUS
 * @param TIMER2_TIS_STATUS
 * @param TIMER3_TIS_STATUS
 * @param TIMER4_TIS_STATUS
 * @param TIMER5_TIS_STATUS
 * @param TIMER6_TIS_STATUS
 * @param TIMER7_TIS_STATUS
 * @retval none
 */
uint32_t ms_timer_get_timers_interrupt_status()
{
	return ms_timer_get_timers_interrupt_status_hal();
}



/**
 * @brief get timers  raw status
 * @param  TimerHandle_Type *tim 
 * @param  uint32_t timern_mask
 * @param   This parameter can be one of the following values:
 * @param TIMER0_TIS_STATUS
 * @param TIMER1_TIS_STATUS
 * @param TIMER2_TIS_STATUS
 * @param TIMER3_TIS_STATUS
 * @param TIMER4_TIS_STATUS
 * @param TIMER5_TIS_STATUS
 * @param TIMER6_TIS_STATUS
 * @param TIMER7_TIS_STATUS
 * @retval none
 */
uint32_t ms_timer_get_timers_raw_status(TimerHandle_Type *tim,uint32_t timern_mask)
{
	 return ms_timer_get_timers_raw_status_hal(tim->instance_conmon,timern_mask);
}

/**
 * @brief clear timers  interrupt status
 * @param  TimerHandle_Type *tim    
 * @retval none
 */
void ms_timer_clear_timers_interrupt_status(TimerHandle_Type *tim)
{
     ms_timer_clear_timers_interrupt_status_hal(tim->instance_conmon);
}


/**
 * @brief clear timer n  interrupt status
 * @param  TimerHandle_Type *tim    
 * @retval none
 */
void ms_timer_clear_timer_interrupt_status(TimerHandle_Type *tim)
{
     ms_timer_clear_timer_interrupt_status_hal(tim->instance);
}


/**
 * @brief get timer current value
 * @param  TimerHandle_Type *tim    
 * @retval timer current value
 */
uint32_t ms_timer_get_curvalue(TimerHandle_Type *tim)
{
    if(NULL == tim)
    {
        return 0;
    }
    return ms_timer_get_curvalue_hal(tim->instance);
}








void ms_timer_irq_handler(TimerHandle_Type *tim)
{
	// timer_inter_clear(tim->instance); //clear irq
    if (tim->p_callback == NULL)
    {
        tim->error_code |= TIMER_ERROR_INVALID_CALLBACK;
        return;
    }
    tim->p_callback->timer_reach_callback(tim);
}

/**
 * set timer mode
 * @param  TimerHandle_Type *tim 
 * @return  0 or -1
 */
int32_t ms_timer_set_mode(TimerHandle_Type *tim)
{
    if(NULL == tim)
    {
        return -1;
    }
    ms_timer_set_mode_hal(tim->instance,tim->init.reload_mode);
    return 0;
}


/**
 * init a hardware timer
 *
 * @param[in]  tmr         timer struct
 */
int32_t ms_timer_init(TimerHandle_Type *tim)
{

    if(NULL == tim)
    {
        return -1;
    }
    /*initial uart gpio pinmux, clock and interrupt setting*/
    if (tim->p_callback && tim->p_callback->init_callback)
    {
        tim->p_callback->init_callback(tim);
    }
	
    ms_timer_stop_hal(tim->instance);
    ms_timer_reload_hal(tim->instance,tim->init.period);
    ms_timer_set_mode_hal(tim->instance, tim->init.reload_mode);
    return 0;
}


/**
 * start a hardware timer
 * @param  TimerHandle_Type *tim  
 * @return  none
 */
int32_t ms_timer_start(TimerHandle_Type *tim)
{
    if(NULL == tim)
    {
        return -1;
    }
    ms_timer_start_hal(tim->instance);
    return 0;
}

/**
 * @brief get timer current value
 * @param  TimerHandle_Type *tim     
 * @retval timer current value (unit us)
 */
uint32_t  ms_timer_get(TimerHandle_Type *tim)
{
    uint32_t reg_value = 0;
    if(NULL == tim)
    {
        return -1;
    }

    reg_value = ms_timer_get_curvalue_hal(tim->instance);
    return (reg_value/(ms_clock_hal_get_sys_clk_freq() / 1000000)); //time for us
}



/**
 * reload hardware timer value
 * @param  TimerHandle_Type *tim  
 * @param int32_t reload_value
 * @return none
 */
int32_t ms_timer_reload(TimerHandle_Type *tim)
{
    if(NULL == tim)
    {
        return -1;
    }

    ms_timer_reload_hal(tim->instance,tim->init.period);
    return 0;
}

/**
 * stop a hardware timer
 * @param  TimerHandle_Type *tim 
 * @return node
 */
void ms_timer_stop(TimerHandle_Type *tim)
{
 
    if(NULL == tim)
    {
        return;
    }

    ms_timer_stop_hal(tim->instance);
}



/**
 * De-initialises an TIMER interface, Turns off an TIMER hardware interface
 * @param[in]  tmr         timer struct
 *
 */
int32_t ms_timer_deinit(TimerHandle_Type *tim)
{

  
    ms_timer_stop_hal(tim->instance);
    /* DeInit the low level hardware */
    if (tim->p_callback && tim->p_callback->deinit_callback)
    {
        tim->p_callback->deinit_callback(tim);
    }
    return 0;
}

