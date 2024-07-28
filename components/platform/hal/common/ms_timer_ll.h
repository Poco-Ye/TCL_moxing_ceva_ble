/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_timer_ll.h
 * @brief Header file of timer  module.
 * @author haijun.mai
 * @date   2021-12-30
 * @version 1.0
 * @Revision
 */

#ifndef MS_TIMER_LL_H_
#define MS_TIMER_LL_H_

#include <ms1008.h>
#include "ms_timer_regs.h"
#include <stdint.h>
#include <stdbool.h>


/**
 * @brief enable timer n  interrupt 
 * @param  TimerConfig_Type *timer     
 * @retval none
 */
static inline void ms_timer_int_enable_ll(TimerConfig_Type * timer)
{
       CLEAR_BIT(timer->TCR,TIMER_TCR_TIMER_INTERRUPT_MASK);
}

/**
 * @brief disable  timer interrupt 
 * @param  TimerConfig_Type *timer     
 * @retval none
 */
static inline void ms_timer_int_disable_ll(TimerConfig_Type * timer)
{
       SET_BIT(timer->TCR,TIMER_TCR_TIMER_INTERRUPT_MASK);
}

/**
 * @brief get  timer interrupt mask 
 * @param  TimerConfig_Type *timer     
 * @retval none
 */
static inline uint32_t ms_timer_get_int_mask_ll()
{
      return ((READ_REG(TIMER0->TCR)|READ_REG(TIMER1->TCR)|READ_REG(TIMER2->TCR)|READ_REG(TIMER3->TCR)|READ_REG(TIMER4->TCR)|READ_REG(TIMER5->TCR)|READ_REG(TIMER6->TCR)|READ_REG(TIMER7->TCR))&TIMER_TCR_TIMER_INTERRUPT_MASK);
}


/**
 * @brief clear timer interrupt status
 * @none
 * @retval interrupt status
 */
static inline uint8_t ms_timer_get_timers_interrupt_status_ll()
{
    return (READ_REG(TIMER->TIS)&0xff);
}

/**
 * @brief clear timers interrupt status
 * @param  TimerConfig_Type *timer   
 * @param   This parameter can be one of the following values:
 * @param TIMER0_TRIS_INT_STATUS
 * @param TIMER1_TRIS_INT_STATUS
 * @param TIMER2_TRIS_INT_STATUS
 * @param TIMER3_TRIS_INT_STATUS
 * @param TIMER4_TRIS_INT_STATUS
 * @param TIMER5_TRIS_INT_STATUS
 * @param TIMER6_TRIS_INT_STATUS
 * @param TIMER7_TRIS_INT_STATUS
 * @retval none
 */
static inline uint32_t ms_timer_get_timers_raw_status_ll(TimerCommon_Type * timer,uint32_t timern_mask)
{
    return (READ_REG(timer->TRIS)&timern_mask);
}

/**
 * @brief clear timers interrupt status
 * @param  TimerCommon_Type *timer     
 * @retval none
 */
static inline void ms_timer_clear_timers_interrupt_status_ll(TimerCommon_Type * timer)
{
    int reg_value  = 0;
	
     reg_value =  READ_REG(timer->TEOI);
}



/**
 * @brief clear timer interrupt status
 * @param  TimerConfig_Type *timer     
 * @retval none
 */
static inline void ms_timer_clear_timer_interrupt_status_ll(TimerConfig_Type * timer)
{
     int reg_value  = 0;
    reg_value = READ_REG(timer->EOI);
}


/**
 * @brief clear timer interrupt status
 * @param  TimerConfig_Type *timer     
 * @retval none
 */
static inline uint32_t ms_timer_get_timer_interrupt_status_ll(TimerConfig_Type * timer)
{
    return READ_REG(timer->IS);
}




/**
 * @brief get timer current value
 * @param  TimerConfig_Type *timer     
 * @retval timer current value
 */
static inline uint32_t ms_timer_get_curvalue_ll(TimerConfig_Type *timer)
{
    return READ_REG(timer->TCV);
}


/**
 * start a hardware timer
 * @param  TimerConfig_Type *timer  
 * @return  none
 */
static inline void  ms_timer_start_ll(TimerConfig_Type *timer)
{
    SET_BIT(timer->TCR,TIMER_TCR_TIMER_ENABLE);
}

/**
 * set timer mode
 * @param  TimerConfig_Type *timer  
 * @param  uint32_t timer_mode
 * @param   This parameter can be one of the following values:
 * @TIMER_USER_DEFINED
 * @TIMER_FREE_RUNNING
 * @return  none
 */
static inline void  ms_timer_set_mode_ll(TimerConfig_Type *timer,uint32_t timer_mode)
{
    MODIFY_REG(timer->TCR,TIMER_TCR_TIMER_MODE,((timer_mode<<TIMER_TCR_TIMER_MODE_POS)&TIMER_TCR_TIMER_MODE));
}



/**
 * stop a hardware timer
 * @param  TimerConfig_Type *timer  
 * @return node
 */
static inline void  ms_timer_stop_ll(TimerConfig_Type *timer)
{
    CLEAR_BIT(timer->TCR ,TIMER_TCR_TIMER_ENABLE);
}



/**
 * reload hardware timer value
 * @param  TimerConfig_Type *timer  
 * @param  int32_t reload_value
 * @return none
 */
static inline void ms_timer_reload_ll(TimerConfig_Type *timer,int32_t reload_value)
{
    WRITE_REG(timer->TLC,(reload_value&TIMER_TLC_LOADCOUNT));
}




#endif /* MS_TIMER_LL_H_ */
