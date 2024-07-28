/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  ms_sys_timer.h
  * @brief 
  * @author bingrui.chen
  * @date 2022年1月17日
  * @version 1.0
  * @Revision: 
  */
#ifndef MS_SYS_TIMER_H_
#define MS_SYS_TIMER_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>

#define MS_TICK_PER_SECOND  1000


/**
  * @brief  Initializes the System Timer and its interrupt, and starts the System Tick Timer.
  *         Counter is in free running mode to generate periodic interrupts.
  * @param[in]  ticks: Specifies the ticks Number of ticks between two interrupts.
  * @retval status:  - 0  Function succeeded.
  *                  - 1  Function failed.
  */
extern int32_t ms_sys_timer_config_systick(uint64_t ticks);


/**
  * @brief Provides a tick value in millisecond.
  * @note   This function is declared as __weak  to be overwritten  in case of other
  *       implementations in user file.
  * @retval tick value
  */
extern uint32_t ms_sys_timer_get_systick(void);

/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * @note ThiS function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @param[in]  delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
extern  void ms_sys_timer_delay(__IO uint32_t delay);


/**
  * @brief This function check cycle numbers since last call this function.
  * @note ThiS function is only for performance debug. 
  *  __enable_all_counter  should be removed (disabl) in the final release to save power
  * cycle numbers since this function is called last time
  * @retval None
  */
extern uint32_t ms_sys_get_cycles();



/**
  * @brief This function check instruction numbers executed since last call this function.
  * @note ThiS function is only for performance debug. 
  *  __enable_all_counter  should be removed (disabl) in the final release to save power
  * instruction numbers since this function is called last time
  * @retval None
  */
extern  uint32_t ms_sys_get_instruction ();



extern void delay(unsigned int cycles);




#endif /* MS_SYS_TIMER_H_ */
