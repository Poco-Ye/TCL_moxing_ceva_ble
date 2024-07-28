/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_timer_regs.h
 * @brief Header file of timer  module.
 * @author haijun.mai
 * @date   2021-12-30
 * @version 1.0
 * @Revision
 */


#ifndef MS_TIMER_REGS_H_
#define MS_TIMER_REGS_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    volatile uint32_t TLC;//Timer N Load Count Register
    volatile uint32_t TCV;// Current value of Timer N
    volatile uint32_t TCR;//Timer N Control Register
    volatile uint32_t EOI;//Timer N End-of-Interrupt Register
    volatile uint32_t IS;//Timer N Interrupt Status Register
} TimerConfig_Type;


typedef struct
{
       volatile uint32_t TIS;//Timers Interrupt Status Register
	volatile uint32_t TEOI;//Timers End-of-Interrupt Register
	volatile uint32_t TRIS;//Timers Raw Interrupt Status Register
	volatile uint32_t TCV; //Timers Component Version
} TimerCommon_Type;

typedef struct
{
	volatile uint32_t TLC2;//Timer N Load Count2 Register
	volatile uint32_t TNPL;//Timer_N Protection level
} TimerExtend_Type;


	



/*Timer N Load Count Register Bit Define*/
#define TIMER_TLC_LOADCOUNT_POS               					                       (0UL)
#define TIMER_TLC_LOADCOUNT_MASK      							                (0xFFFFFFFFUL << TIMER_TLC_LOADCOUNT_POS)
#define TIMER_TLC_LOADCOUNT	                                                                        (TIMER_TLC_LOADCOUNT_MASK)

 
/*Current value of Timer N Bit Define*/
#define TIMER_TCV_CURRENTVALUE_POS               			                              (0UL)
#define TIMER_TCV_CURRENTVALUE_MASK      					                       (0xFFFFFFFFUL << TIMER_TCV_CURRENTVALUE_POS)
#define TIMER_TCV_CURRENTVALUE	                                                                  (TIMER_TCV_CURRENTVALUE_MASK)

/*Timer N Control Register Bit Define*/
#define TIMER_TCR_TIMER_ENABLE_POS               						         (0UL)
#define TIMER_TCR_TIMER_ENABLE_MASK      					                       (0x1UL << TIMER_TCR_TIMER_ENABLE_POS)
#define TIMER_TCR_TIMER_ENABLE      					                                      (TIMER_TCR_TIMER_ENABLE_MASK)

#define TIMER_TCR_TIMER_MODE_POS               							          (1UL)
#define TIMER_TCR_TIMER_MODE_MASK      					                               (0x1UL << TIMER_TCR_TIMER_MODE_POS)
#define TIMER_TCR_TIMER_MODE      					                                      (TIMER_TCR_TIMER_MODE_MASK)

#define TIMER_TCR_TIMER_INTERRUPT_MASK_POS               				          (2UL)
#define TIMER_TCR_TIMER_INTERRUPT_MASK_MASK      					          (0x1UL << TIMER_TCR_TIMER_INTERRUPT_MASK_POS)
#define TIMER_TCR_TIMER_INTERRUPT_MASK      					                 (TIMER_TCR_TIMER_INTERRUPT_MASK_MASK)

#define TIMER_TCR_TIMER_PWM_POS               							          (3UL)
#define TIMER_TCR_TIMER_PWM_MASK      					                               (0x1UL << TIMER_TCR_TIMER_PWM_POS)
#define TIMER_TCR_TIMER_PWM      					                                      (TIMER_TCR_TIMER_PWM_MASK)

#define TIMER_TCR_TIMER_0N100PWM_EN_POS               					   (4UL)
#define TIMER_TCR_TIMER_0N100PWM_EN_MASK      					          (0x1UL << TIMER_TCR_TIMER_0N100PWM_EN_POS)
#define TIMER_TCR_TIMER_0N100PWM_EN      					                        (TIMER_TCR_TIMER_0N100PWM_EN_MASK)

/*Timer N End-of-Interrupt Register Bit Define*/
#define TIMER_EOI_TIMERNEOI_POS               							          (0UL)
#define TIMER_EOI_TIMERNEOI_MASK      					                               (0x1UL << TIMER_EOI_TIMERNEOI_POS)
#define TIMER_EOI_TIMERNEOI      					                                             (TIMER_EOI_TIMERNEOI_MASK)


/*Timer N Interrupt Status Register Bit Define*/
#define TIMER_IS_STATUS_POS               							                 (0UL)
#define TIMER_IS_STATUS_MASK      					                                      (0x1UL << TIMER_IS_STATUS_POS)
#define TIMER_IS_STATUS      					                                                    (TIMER_IS_STATUS_MASK)

/*Timers Interrupt Status Register Bit Define*/
#define TIMER0_TIS_STATUS_POS               							               (0UL)
#define TIMER0_TIS_STATUS_MASK      					                                    (0x1UL << TIMER0_TIS_STATUS_POS)
#define TIMER0_TIS_STATUS      					                                           (TIMER0_TIS_STATUS_MASK)

#define TIMER1_TIS_STATUS_POS               							               (1UL)
#define TIMER1_TIS_STATUS_MASK      					                                    (0x1UL << TIMER1_TIS_STATUS_POS)
#define TIMER1_TIS_STATUS      					                                            (TIMER1_TIS_STATUS_MASK)

#define TIMER2_TIS_STATUS_POS               							               (2UL)
#define TIMER2_TIS_STATUS_MASK      					                                    (0x1UL << TIMER2_TIS_STATUS_POS)
#define TIMER2_TIS_STATUS      					                                           (TIMER2_TIS_STATUS_MASK)

#define TIMER3_TIS_STATUS_POS               							               (3UL)
#define TIMER3_TIS_STATUS_MASK      					                                    (0x1UL << TIMER3_TIS_STATUS_POS)
#define TIMER3_TIS_STATUS      					                                           (TIMER3_TIS_STATUS_MASK)

#define TIMER4_TIS_STATUS_POS               							               (4UL)
#define TIMER4_TIS_STATUS_MASK      					                                    (0x1UL << TIMER4_TIS_STATUS_POS)
#define TIMER4_TIS_STATUS      					                                           (TIMER4_TIS_STATUS_MASK)

#define TIMER5_TIS_STATUS_POS               							               (5UL)
#define TIMER5_TIS_STATUS_MASK      					                                    (0x1UL << TIMER5_TIS_STATUS_POS)
#define TIMER5_TIS_STATUS      					                                           (TIMER5_TIS_STATUS_MASK)

#define TIMER6_TIS_STATUS_POS               							               (6UL)
#define TIMER6_TIS_STATUS_MASK      					                                    (0x1UL << TIMER6_TIS_STATUS_POS)
#define TIMER6_TIS_STATUS      					                                           (TIMER6_TIS_STATUS_MASK)

#define TIMER7_TIS_STATUS_POS               							               (7UL)
#define TIMER7_TIS_STATUS_MASK      					                                    (0x1UL << TIMER7_TIS_STATUS_POS)
#define TIMER7_TIS_STATUS      					                                           (TIMER7_TIS_STATUS_MASK)


/*Timers End-of-Interrupt Register Bit Define*/
#define TIMER0_TEOI_POS               							                      (0UL)
#define TIMER0_TEOI_MASK      					                                           (0x1UL << TIMER0_TEOI_POS)
#define TIMER0_TEOI_STATUS      					                                           (TIMER0_TEOI_MASK)

#define TIMER1_TEOI_POS               							                      (1UL)
#define TIMER1_TEOI_MASK      					                                           (0x1UL << TIMER1_TEOI_POS)
#define TIMER1_TEOI_STATUS      					                                           (TIMER1_TEOI_MASK)

#define TIMER2_TEOI_POS               							                     (2UL)
#define TIMER2_TEOI_MASK      					                                          (0x1UL << TIMER2_TEOI_POS)
#define TIMER2_TEOI_STATUS      					                                          (TIMER2_TEOI_MASK)

#define TIMER3_TEOI_POS               							                     (3UL)
#define TIMER3_TEOI_MASK      					                                          (0x1UL << TIMER3_TEOI_POS)
#define TIMER3_TEOI_STATUS      					                                          (TIMER3_TEOI_MASK)

#define TIMER4_TEOI_POS               							                     (4UL)
#define TIMER4_TEOI_MASK      					                                          (0x1UL << TIMER4_TEOI_POS)
#define TIMER4_TEOI_STATUS      					                                          (TIMER4_TEOI_MASK)

#define TIMER5_TEOI_POS               							                     (5UL)
#define TIMER5_TEOI_MASK      					                                          (0x1UL << TIMER5_TEOI_POS)
#define TIMER5_TEOI_STATUS      					                                          (TIMER5_TEOI_MASK)

#define TIMER6_TEOI_POS               							                     (6UL)
#define TIMER6_TEOI_MASK      					                                          (0x1UL << TIMER6_TEOI_POS)
#define TIMER6_TEOI_STATUS      					                                          (TIMER6_TEOI_MASK)

#define TIMER7_TEOI_POS               							                     (7UL)
#define TIMER7_TEOI_MASK      					                                          (0x1UL << TIMER7_TEOI_POS)
#define TIMER7_TEOI_STATUS      					                                          (TIMER7_TEOI_MASK)

/*Timers Raw Interrupt Status Register Bit Define*/
#define TIMER0_TRIS_INT_STATUS_POS               							 (0UL)
#define TIMER0_TRIS_INT_STATUS_MASK      					                      (0x1UL << TIMER0_TRIS_INT_STATUS_POS)
#define TIMER0_TRIS_INT_STATUS      					                                    (TIMER0_TRIS_INT_STATUS_MASK)

#define TIMER1_TRIS_INT_STATUS_POS               							 (1UL)
#define TIMER1_TRIS_INT_STATUS_MASK      					                      (0x1UL << TIMER1_TRIS_INT_STATUS_POS)
#define TIMER1_TRIS_INT_STATUS      					                                    (TIMER1_TRIS_INT_STATUS_MASK)

#define TIMER2_TRIS_INT_STATUS_POS               							 (2UL)
#define TIMER2_TRIS_INT_STATUS_MASK      					                      (0x1UL << TIMER2_TRIS_INT_STATUS_POS)
#define TIMER2_TRIS_INT_STATUS      					                                    (TIMER2_TRIS_INT_STATUS_MASK)

#define TIMER3_TRIS_INT_STATUS_POS               							 (3UL)
#define TIMER3_TRIS_INT_STATUS_MASK      					                      (0x1UL << TIMER3_TRIS_INT_STATUS_POS)
#define TIMER3_TRIS_INT_STATUS      					                                    (TIMER3_TRIS_INT_STATUS_MASK)

#define TIMER4_TRIS_INT_STATUS_POS               							 (4UL)
#define TIMER4_TRIS_INT_STATUS_MASK      					                      (0x1UL << TIMER4_TRIS_INT_STATUS_POS)
#define TIMER4_TRIS_INT_STATUS      					                                    (TIMER4_TRIS_INT_STATUS_MASK)

#define TIMER5_TRIS_INT_STATUS_POS               							 (5UL)
#define TIMER5_TRIS_INT_STATUS_MASK      					                      (0x1UL << TIMER5_TRIS_INT_STATUS_POS)
#define TIMER5_TRIS_INT_STATUS      					                                    (TIMER5_TRIS_INT_STATUS_MASK)

#define TIMER6_TRIS_INT_STATUS_POS               							 (6UL)
#define TIMER6_TRIS_INT_STATUS_MASK      					                      (0x1UL << TIMER6_TRIS_INT_STATUS_POS)
#define TIMER6_TRIS_INT_STATUS      					                                    (TIMER6_TRIS_INT_STATUS_MASK)

#define TIMER7_TRIS_INT_STATUS_POS               							 (7UL)
#define TIMER7_TRIS_INT_STATUS_MASK      					                      (0x1UL << TIMER7_TRIS_INT_STATUS_POS)
#define TIMER7_TRIS_INT_STATUS      					                                    (TIMER7_TRIS_INT_STATUS_MASK)



/*Timers Component Version Bit Define*/
#define TIMER_TCV_TIMERSCOMPVERSION_POS               						(0UL)
#define TIMER_TCV_TIMERSCOMPVERSION_MASK      					              (0xFFFFFFFFUL << TIMER_TCV_TIMERSCOMPVERSION_POS)
#define TIMER_TCV_TIMERSCOMPVERSION_STATUS      					       (TIMER_TCV_TIMERSCOMPVERSION_MASK)

/*Timer N Load Count2 Register Bit Define*/
#define TIMER_TLC2_TIMERLOADCOUNT2_POS               						(0UL)
#define TIMER_TLC2_TIMERLOADCOUNT2_MASK      							(0xFFFFFFFFUL << TIMER_TLC2_TIMERLOADCOUNT2_POS)
#define TIMER_TLC2_TIMERLOADCOUNT2	                                                       (TIMER_TLC2_TIMERLOADCOUNT2_MASK)



/*Timer_N Protection level Bit Define*/
#define TIMER_TNPL_TIMERNPROTLEVELFIELD_POS               					(0UL)
#define TIMER_TNPL_TIMERNPROTLEVELFIELD_MASK      						(0x7UL << TIMER_TNPL_TIMERNPROTLEVELFIELD_POS)
#define TIMER_TNPL_TIMERNPROTLEVELFIELD	                                                 (TIMER_TNPL_TIMERNPROTLEVELFIELD_MASK)


#ifdef __cplusplus
}
#endif

#endif /* MS_TIMER_H_ */

