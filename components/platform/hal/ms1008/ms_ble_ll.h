/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_blel_ll.h
 * @brief Header file of BLE control
 * @author pengyu.xue
 * @date   2022-03-04
 * @version 1.0
 * @Revision
 */

#ifndef MS_BLE_CONTROL_LL_H_
#define MS_BLE_CONTROL_LL_H_

#include <ms1008.h>
#include "ms_sys_ctrl_regs.h"
#include <stdint.h>
#include <stdbool.h>


/**
 * @brief        RF spi select  0: SPI0 control IO external device  / 1: SPI0 control RF SPI
 *                    this function is to set SPI0 control RF SPI
 * @param  none
 * @retval None
 */
static inline void ms_ctrl_set_ble_spi_rf_selec_ll()
{
    SET_BIT(SYS_CTRL->BLE_CTRL,SYS_CTRL_BLE_CTRL_RF_SEL);
}


/**
 * @brief select ble controller clock
 * @param  clock frequency
 * @retval none
 */
static inline void ms_ctrl_ble_clk_selec_ll(uint32_t freq)
{
    MODIFY_REG(SYS_CTRL->BLE_CTRL, SYS_CTRL_BLE_CTRL_CLOCK, 
		(freq <<SYS_CTRL_BLE_CTRL_CLOCK_POS) & SYS_CTRL_BLE_CTRL_CLOCK_POS_MASK);              
}



/**
 * @brief       ble mdm clock by pass setting
 *                
 * @param  none
 * @retval None
 */
static inline void ms_ctrl_set_blemdm_clock_bypass_ll(bool bypass)
{
    
}



/**
 * @brief RF control mode, enable hw control
 * @param  none
 * @retval None
 */
static inline void ms_ctrl_set_ble_rf_hw_valid()
{
    SET_BIT(SYS_CTRL->LP_KEEP,SYS_CTRL_RF_MODE_HW_VLD);
}

/**
 * @brief RF control mode, enable hw control
 * @param  none
 * @retval None
 */
static inline void ms_ctrl_set_ble_rf_hw_invalid()
{
    CLEAR_BIT(SYS_CTRL->LP_KEEP,SYS_CTRL_RF_MODE_HW_VLD);
}


/**
 * @brief SW RF control mode, set SW control parameter
 * @param  SW control parameter
 * b00001:deep sleep(power down)
 * b00010:sleep,disable XC24M & RF
 * b00100:standby, enable XC24M, disable RF
 * b01000:TX
 * b10000:RX
 * @retval None
 */
static inline void ms_ctrl_set_ble_rf_swcontrol_parameter(uint32_t parameter)
{
    MODIFY_REG(SYS_CTRL->LP_KEEP, SYS_CTRL_RF_MODE_SW, 
		 (parameter <<SYS_CTRL_RF_MODE_SW_POS) & SYS_CTRL_RF_MODE_SW_MASK);

}


#endif //MS_BLE_CONTROL_LL_H_
