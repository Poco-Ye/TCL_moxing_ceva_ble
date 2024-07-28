/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_timer_hal.h
 * @brief Header file of timer  module.
 * @author haijun.mai
 * @date   2021-12-30
 * @version 1.0
 * @Revision
 */


#ifndef MS_TIMER_H_
#define MS_TIMER_H_

#include "ms1008.h"
#include "ms_timer_ll.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif



#define MS_TIMER_NUM 8




#define FREE_RUNNING_MODE 0
#define PERIODIC_MODE (1 << 1)
#define ONE_SHOT_MODE 0


#define TIMER_ERROR_INVALID_CALLBACK  0x00000001U   /*!< Invalid Callback error  */


typedef struct
{
    uint32_t      period;   /*us*/
    uint8_t        reload_mode;
} TimerInit_Type;

struct __TimerHandle_Type;
/**
  * @brief  Timer callback handle Structure definition
  */
typedef struct
{
    void (* error_callback)                (struct __TimerHandle_Type *timer);
    void (* init_callback)                   (struct __TimerHandle_Type *timer);
    void (* deinit_callback)               (struct __TimerHandle_Type *timer);
    void (* timer_reach_callback)    (struct __TimerHandle_Type *timer);
}TimerCallback_Type;


typedef struct __TimerHandle_Type
{
    TimerConfig_Type           *instance;
    TimerCommon_Type       *instance_conmon;
    TimerExtend_Type          *instance_extend; 
    TimerInit_Type                init;/*!< Timer communication parameters      */
    uint32_t                           error_code;         /*!< TIMER Error code*/      
    IRQn_Type                      irq;
    TimerCallback_Type         *p_callback;
}TimerHandle_Type;

extern void ms_timer_enable_cpu_interrupt(TimerHandle_Type *tim);
extern void ms_timer_disable_cpu_interrupt(TimerHandle_Type *tim);
extern int32_t ms_timer_init(TimerHandle_Type *tim);
extern int32_t ms_timer_start(TimerHandle_Type *tim);
extern uint32_t  ms_timer_get(TimerHandle_Type *tim);
extern int32_t ms_timer_reload(TimerHandle_Type *tim);
extern void ms_timer_stop(TimerHandle_Type *tim);
extern int32_t ms_timer_deinit(TimerHandle_Type *tim);
extern int32_t ms_timer_set_mode(TimerHandle_Type *tim);
extern uint32_t ms_timer_get_curvalue(TimerHandle_Type *tim);
extern void ms_timer_irq_handler(TimerHandle_Type *tim);
extern void ms_timer_clear_timers_interrupt_status(TimerHandle_Type *tim);
extern void ms_timer_clear_timer_interrupt_status(TimerHandle_Type *tim);
extern uint32_t ms_timer_get_timers_interrupt_status();
extern uint32_t ms_timer_get_timers_raw_status(TimerHandle_Type *tim,uint32_t timern_mask);
extern void ms_timer_int_enable(TimerHandle_Type *tim);
extern void ms_timer_int_disable(TimerHandle_Type *tim);

#ifdef __cplusplus
}
#endif

#endif /* MS_TIMER_H_ */
