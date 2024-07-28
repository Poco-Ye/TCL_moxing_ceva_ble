/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_timer_ll.c
 * @brief c source  file of timer  module.
 * @author haijun.mai
 * @date   2021-12-30
 * @version 1.0
 * @Revision
 */
#ifndef MS_TIMER_HAL_H_
#define MS_TIMER_HAL_H_

#include "ms_timer_ll.h"
#define TIMER_USER_DEFINED  1  /* timer reload automatic */
#define TIMER_FREE_RUNNING  0  /* timer reload manual */

/**
 * @brief enable timer n  interrupt 
 * @param  TimerConfig_Type *timer     
 * @retval none
 */
#define ms_timer_int_enable_hal( timer) ms_timer_int_enable_ll( timer)

/**
 * @brief enable timer n  interrupt 
 * @param  TimerConfig_Type *timer     
 * @retval none
 */
#define ms_timer_int_disable_hal( timer) ms_timer_int_disable_ll( timer)

/**
 * @brief get  timer interrupt mask 
 * @param  TimerConfig_Type *timer     
 * @retval none
 */
#define  ms_timer_get_int_mask_hal()   ms_timer_get_int_mask_ll()

/**
 * @brief clear timer interrupt status
 * @param  TimerCommon_Type *timer  
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
#define ms_timer_get_timers_interrupt_status_hal()  ms_timer_get_timers_interrupt_status_ll()



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
#define ms_timer_get_timers_raw_status_hal(timer, timern_mask)  ms_timer_get_timers_raw_status_ll(timer, timern_mask)


/**
 * @brief clear timers interrupt status
 * @param  TimerCommon_Type *timer     
 * @retval none
 */
#define ms_timer_clear_timers_interrupt_status_hal(timer)  ms_timer_clear_timers_interrupt_status_ll(timer)



/**
 * @brief clear timer interrupt status
 * @param  TimerConfig_Type *timer     
 * @retval none
 */
#define ms_timer_clear_timer_interrupt_status_hal(timer) ms_timer_clear_timer_interrupt_status_ll(timer)



/**
 * @brief clear timer interrupt status
 * @param  TimerConfig_Type *timer     
 * @retval none
 */
#define  ms_timer_get_timer_interrupt_status_hal( timer)   ms_timer_get_timer_interrupt_status_ll(timer)


/**
 * @brief get timer current value
 * @param  TimerConfig_Type *timer     
 * @retval timer current value
 */
#define  ms_timer_get_curvalue_hal(timer)       ms_timer_get_curvalue_ll(timer)





/**
 * start a hardware timer
 * @param  TimerConfig_Type *timer  
 * @return  none
 */
#define ms_timer_start_hal(timer)     ms_timer_start_ll(timer)


/**
 * set timer mode
 * @param  TimerConfig_Type *timer  
 * @param  uint32_t timer_mode
 * @param   This parameter can be one of the following values:
 * @TIMER_USER_DEFINED
 * @TIMER_FREE_RUNNING
 * @return  none
 */
#define ms_timer_set_mode_hal(timer, timer_mode)       ms_timer_set_mode_ll(timer, timer_mode)



/**
 * stop a hardware timer
 * @param  TimerConfig_Type *timer  
 * @return node
 */
#define  ms_timer_stop_hal(timer)      ms_timer_stop_ll(timer)


/**
 * reload hardware timer value
 * @param  TimerConfig_Type *timer  
 * @param  int32_t reload_value
 * @return none
 */
 #define   ms_timer_reload_hal(timer,reload_value)      ms_timer_reload_ll(timer,reload_value)



#endif/* MS_TIMER_HAL_H_ */
