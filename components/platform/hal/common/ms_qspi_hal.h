/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  ms_qpsi_hal.h
  * @brief 
  * @author che.jiang
  * @date 2022年1月4日
  * @version 1.0
  * @Revision: 
  */
#ifndef BLE_SOC_XXX_MS_QSPI_HAL_H_
#define BLE_SOC_XXX_MS_QSPI_HAL_H_

#include "ms_qspi_ll.h"




/**
 * @brief  QSPI control command mode
 * @retval None
 */
#define  ms_qspi_hal_flash_control_command(hqspi, command)      ms_qspi_ll_flash_control_command(hqspi,command)
#define  ms_qspi_hal_flash_control_clear_command(hqspi)   ms_qspi_ll_flash_control_clear_command(hqspi)
#define  ms_qspi_hal_config_master_baud_div(hqspi, div_type)    ms_qspi_ll_config_master_baud_div(hqspi, div_type)



#endif /* BLE_SOC_XXX_MS_QSPI_HAL_H_ */
