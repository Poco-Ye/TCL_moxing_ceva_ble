/*
 * timer.h
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "ms1008.h"

extern int32_t timer0_init(void);
extern int32_t timer0_start(void);
extern int32_t timer0_stop(void);
extern int32_t timer0_set_mode(int32_t reload_mode);
extern int32_t timer0_relaod_value(int32_t reload_value);
extern uint32_t timer0_get_curvalue(void);
extern int32_t timer0_get_timers_raw_status();
extern int32_t timer0_interrupt_enable(void);
extern int32_t timer0_interrupt_disable(void);
extern int32_t timer0_deinit(void);


extern int32_t timer1_init(void);
extern int32_t timer1_start(void);
extern int32_t timer1_stop(void);
extern int32_t timer1_set_mode(int32_t reload_mode);
extern int32_t timer1_relaod_value(int32_t reload_value);
extern uint32_t timer1_get_curvalue(void);
extern int32_t timer1_get_timers_raw_status();
extern int32_t timer1_interrupt_enable(void);
extern int32_t timer1_interrupt_disable(void);
extern int32_t timer1_deinit(void);


extern int32_t timer2_init(void);
extern int32_t timer2_start(void);
extern int32_t timer2_stop(void);
extern int32_t timer2_set_mode(int32_t reload_mode);
extern int32_t timer2_relaod_value(int32_t reload_value);
extern uint32_t timer2_get_curvalue(void);
extern int32_t timer2_get_timers_raw_status();
extern int32_t timer2_interrupt_enable(void);
extern int32_t timer2_interrupt_disable(void);
extern int32_t timer2_deinit(void);


extern int32_t timer3_init(void);
extern int32_t timer3_start(void);
extern int32_t timer3_stop(void);
extern int32_t timer3_set_mode(int32_t reload_mode);
extern int32_t timer3_relaod_value(int32_t reload_value);
extern uint32_t timer3_get_curvalue(void);
extern int32_t timer3_get_timers_raw_status();
extern int32_t timer3_interrupt_enable(void);
extern int32_t timer3_interrupt_disable(void);
extern int32_t timer3_deinit(void);


extern int32_t timer4_init(void);
extern int32_t timer4_start(void);
extern int32_t timer4_stop(void);
extern int32_t timer4_set_mode(int32_t reload_mode);
extern int32_t timer4_relaod_value(int32_t reload_value);
extern uint32_t timer4_get_curvalue(void);
extern int32_t timer4_get_timers_raw_status();
extern int32_t timer4_interrupt_enable(void);
extern int32_t timer4_interrupt_disable(void);
extern int32_t timer4_deinit(void);


extern int32_t timer5_init(void);
extern int32_t timer5_start(void);
extern int32_t timer5_stop(void);
extern int32_t timer5_set_mode(int32_t reload_mode);
extern int32_t timer5_relaod_value(int32_t reload_value);
extern uint32_t timer5_get_curvalue(void);
extern int32_t timer5_get_timers_raw_status();
extern int32_t timer5_interrupt_enable(void);
extern int32_t timer5_interrupt_disable(void);
extern int32_t timer5_deinit(void);


extern int32_t timer6_init(void);
extern int32_t timer6_start(void);
extern int32_t timer6_stop(void);
extern int32_t timer6_set_mode(int32_t reload_mode);
extern int32_t timer6_relaod_value(int32_t reload_value);
extern uint32_t timer6_get_curvalue(void);
extern int32_t timer6_get_timers_raw_status();
extern int32_t timer6_interrupt_enable(void);
extern int32_t timer6_interrupt_disable(void);
extern int32_t timer6_deinit(void);


extern int32_t timer7_init(void);
extern int32_t timer7_start(void);
extern int32_t timer7_stop(void);
extern int32_t timer7_set_mode(int32_t reload_mode);
extern int32_t timer7_relaod_value(int32_t reload_value);
extern uint32_t timer7_get_curvalue(void);
extern int32_t timer7_get_timers_raw_status();
extern int32_t timer7_interrupt_enable(void);
extern int32_t timer7_interrupt_disable(void);
extern int32_t timer7_deinit(void);


#endif /* TIMER_H_ */
