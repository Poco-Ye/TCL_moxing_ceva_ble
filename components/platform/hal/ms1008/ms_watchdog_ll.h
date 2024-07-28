/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_watchdog_ll.h
 * @brief Header file of watchdog  module.
 * @author haijun.mai
 * @date   2022-01-04
 * @version 1.0
 * @Revision
 */

#ifndef MS_WATCHDOG_LL_H_
#define MS_WATCHDOG_LL_H_

#include <ms1008.h>
#include "ms_watchdog_regs.h"
#include <stdint.h>
#include <stdbool.h>

#define WDT_RESTART_CMD          0x76

/*define Response mode*/
#define              RESPONSE_MODE_GENERATE_A_SYSTEM_RESET                                             (0)
#define              RESPONSE_MODE_GENERATE_INTERRUPT_BEFORE_SYSTEM_RESET               (1)

/*define Timeout period*/
#define           USER0_OR_64K               (0x0)
#define      	 USER1_OR_128K             (0x1)
#define  	        USER2_OR_256K             (0x2)
#define  	        USER3_OR_512K             (0x3)       
#define  	        USER4_OR_1M                (0x4)
#define  	        USER5_OR_2M                (0x5)
#define  	        USER6_OR_4M                (0x6)
#define     	 USER7_OR_8M                (0x7)
#define  	        USER8_OR_16M              (0x8)
#define     	 USER9_OR_32M              (0x9)
#define  	        USER10_OR_64M            (0xa)
#define    	 USER11_OR_128M          (0xb)
#define   	 USER12_OR_256M          (0xc)
#define  	        USER13_OR_512M          (0xd) 
#define  	        USER14_OR_1G              (0xe)
#define           USER15_OR_2G              (0xf)

/*define Reset pulse length*/
#define             PCLK_CYCLES_2                            (0x0)
#define             PCLK_CYCLES_4                            (0x1)
#define             PCLK_CYCLES_8                            (0x2)
#define             PCLK_CYCLES_16                          (0x3)
#define             PCLK_CYCLES_32                          (0x4)
#define             PCLK_CYCLES_64                          (0x5)
#define             PCLK_CYCLES_128                        (0x6)
#define             PCLK_CYCLES_256                        (0x7)

/**
 * @brief enable watchdog 
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @retval None
 */
static inline void ms_wdg_enable_ll(Watchdog_Type *watchdog) {
	SET_BIT(watchdog->CR, WATCHDOG_CR_WDT_EN);
}

/**
 * @brief disable watchdog 
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @retval None
 */
static inline void ms_wdg_close_ll(Watchdog_Type *watchdog) {
	
}

/**
 * @brief enable watchdog 
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @int32_t Response_mode
 * @RESPONSE_MODE_GENERATE_A_SYSTEM_RESET                                             (0)
 * @RESPONSE_MODE_GENERATE_INTERRUPT_BEFORE_SYSTEM_RES
 * @retval None
 */
static inline void ms_wdg_set_rsp_mode_ll(Watchdog_Type *watchdog,
		int32_t Response_mode) {
	CLEAR_BIT(watchdog->CR, WATCHDOG_CR_RMOD);
	SET_BIT(watchdog->CR, Response_mode&WATCHDOG_CR_RMOD);
}

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
static inline void ms_wdg_set_reset_pulse_length_ll(Watchdog_Type *watchdog,
		int32_t reset_pulse_length) {
	CLEAR_BIT(watchdog->CR, WATCHDOG_CR_RPL);
	SET_BIT(watchdog->CR, reset_pulse_length&WATCHDOG_CR_RPL);
}

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
static inline void ms_wdg_set_timeout_period_ll(Watchdog_Type *watchdog,
		int32_t timeout_period) {
	CLEAR_BIT(watchdog->TORR, WATCHDOG_TORR_TOP);
	SET_BIT(watchdog->TORR, timeout_period&WATCHDOG_TORR_TOP);
}

/**
 * @brief get watchdog count value
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @retval None
 */
static inline int32_t ms_wdg_get_cnt_val_ll(Watchdog_Type *watchdog) {
	return (READ_REG(watchdog->CCVR));
}

/**
 * @brief reset watchdog count  value 
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @retval None
 */
static inline void ms_wdg_restart_ll(Watchdog_Type *watchdog) {
	WRITE_REG(watchdog->CRR, WDT_RESTART_CMD);
}

/**
 * @brief get watchdog interrupt status 
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @retval interrupt status
 */
static inline int32_t ms_wdg_get_interrupt_status_ll(Watchdog_Type *watchdog) {
	return (READ_REG(watchdog->STAT));
}

/**
 * @brief clear watchdog interrupt status 
 * @param  Watchdog_Type *watchdog: WATCHDOG regs
 * @retval None
 */
static inline int32_t ms_wdg_clear_interrupt_status_ll(
		Watchdog_Type *watchdog) {
	return (READ_REG(watchdog->EOI));
}

#endif
