/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_blel_hal.h
 * @brief Header file of BLE control
 * @author pengyu.xue
 * @date   2022-03-04
 * @version 1.0
 * @Revision
 */

#ifndef MS_BLE_CONTROL_HAL_H_
#define MS_BLE_CONTROL_HAL_H_

#include "ms_ble_ll.h"
#include <stdint.h>
#include <stdbool.h>



/**
 * @brief  Enable spi0 control bel rf
 * @param  None
 * @retval None
 */
#define ms_ctrl_set_ble_spi_rf_selec_hal()   ms_ctrl_set_ble_spi_rf_selec_ll()



/**
 * @brief  set ble controller Clock
 * @param  clock
 * @retval None
 */
#define ms_ctrl_ble_clk_selec_hal(feq)   ms_ctrl_ble_clk_selec_ll(feq)


/**
 * @brief  set mdm ble clock by pass
 * @param  clock
 * @retval None
 */
#define ms_ctrl_set_blemdm_clock_bypass_hal(bypass) ms_ctrl_set_blemdm_clock_bypass_ll( bypass)



/**
 * @brief  set rf control hw valid
 * @param  
 * @retval None
 */
#define ms_ctrl_ble_rf_hw_valid_hal()  ms_ctrl_set_ble_rf_hw_valid()


/**
 * @brief set rf control hw invalid
 * @param  None
 * @retval None
 */
#define ms_ctrl_ble_rf_hw_invalid_hal() ms_ctrl_set_ble_rf_hw_invalid()


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
#define ms_ctrl_ble_rf_swcontrol_parameter_hal(parameter) ms_ctrl_set_ble_rf_swcontrol_parameter(parameter)


#endif //MS_BLE_CONTROL_HAL_H_
