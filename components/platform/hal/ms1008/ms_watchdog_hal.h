/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_watchdog_hal.h
 * @brief Header file of watchdog  module.
 * @author haijun.mai
 * @date   2022-01-04
 * @version 1.0
 * @Revision
 */

#ifndef MS_WATCHDOG_HAL_H_
#define MS_WATCHDOG_HAL_H_

#include "ms_watchdog_ll.h"

/**
 * @brief enable watchdog 
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @retval None
 */
#define  ms_wdg_enable_hal(watchdog)        ms_wdg_enable_ll(watchdog)


/**
 * @brief enable watchdog 
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @retval None
 */
#define  ms_wdg_close_hal(watchdog)        ms_wdg_close_ll(watchdog)

/**
 * @brief enable watchdog 
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @int32_t Response_mode
 * @RESPONSE_MODE_GENERATE_A_SYSTEM_RESET                                             (0)
 * @RESPONSE_MODE_GENERATE_INTERRUPT_BEFORE_SYSTEM_RES
 * @retval None
 */
#define  ms_wdg_set_rsp_mode_hal(watchdog, Response_mode)        ms_wdg_set_rsp_mode_ll(watchdog, Response_mode)

/**
 * @brief set after n clock cycles reset the chip when reset single reach
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @int32_t reset_pulse_length
 * @param   This parameter can be one of the following values:
 * @PCLK_CYCLES_2                           
 * @PCLK_CYCLES_4                         
 * @PCLK_CYCLES_8                            
 * @PCLK_CYCLES_16                          
 * @PCLK_CYCLES_32                          
 * @PCLK_CYCLES_64                          
 * @PCLK_CYCLES_128                       
 * @PCLK_CYCLES_256                        
 * @retval None
 */
#define  ms_wdg_set_reset_pulse_length_hal(watchdog,reset_pulse_length)        ms_wdg_set_reset_pulse_length_ll(watchdog,reset_pulse_length)

/**
 * @brief set timeout period
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @int32_t timeout_period
 * @param   This parameter can be one of the following values:
 * @USER0_OR_64K               
 * @USER1_OR_128K             
 * @USER2_OR_256K           
 * @USER3_OR_512K                
 * @USER4_OR_1M               
 * @USER5_OR_2M               
 * @USER6_OR_4M               
 * @USER7_OR_8M                
 * @USER8_OR_16M           
 * @USER9_OR_32M             
 * @USER10_OR_64M            
 * @USER11_OR_128M         
 * @USER12_OR_256M        
 * @USER13_OR_512M         
 * @USER14_OR_1G           
 * @USER15_OR_2G  
 * @retval None
 */
#define   ms_wdg_set_timeout_period_hal(watchdog,timeout_period)        ms_wdg_set_timeout_period_ll(watchdog,timeout_period)

/**
 * @brief get watchdog count value
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @retval None
 */
#define  ms_wdg_get_cnt_val_hal(watchdog)         ms_wdg_get_cnt_val_ll(watchdog)     

/**
 * @brief reset watchdog count  value 
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @retval None
 */
#define  ms_wdg_restart_hal(watchdog)        ms_wdg_restart_ll(watchdog)

/**
 * @brief get watchdog interrupt status 
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @retval interrupt status
 */
#define  ms_wdg_get_interrupt_status_hal(watchdog)        ms_wdg_get_interrupt_status_ll(watchdog)

/**
 * @brief clear watchdog interrupt status 
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @retval None
 */
#define  ms_wdg_clear_interrupt_status_hal(watchdog)        ms_wdg_clear_interrupt_status_ll(watchdog)

#endif/* MS_WATCHDOG_HAL_H_ */

