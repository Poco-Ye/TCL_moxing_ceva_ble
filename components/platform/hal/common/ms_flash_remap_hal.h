/**
 * Copyright  2021 by MooreSilicon.All rights reserved
 * @file   ms_flash_remap_hal.h
 * @brief Header file of flash remap  module.
 * @author che.jiang
 * @date   2022-03-10
 * @version 1.0
 * @Revision
 */


#ifndef MS_FLASH_REMAP_HAL_H_
#define MS_FLASH_REMAP_HAL_H_

#include "ms_flash_remap_ll.h"


/**
 * @brief flash remap set source addr
 * @retval None
 */
#define ms_flash_remap_set_src_addr_hal(addr)        ms_flash_remap_set_src_addr_ll(addr)


/**
 * @brief flash remap set destination addr
 * @retval None
 */
#define ms_flash_remap_set_dst_addr_hal(addr)        ms_flash_remap_set_dst_addr_ll(addr)

/**
 * @brief flash remap set remap offset range 
 * @retval None
 */
#define ms_flash_remap_set_region_oft_hal(addr)     ms_flash_remap_set_region_oft_ll(addr)

/**
 * @brief flash remap set remap enable
 * @retval None
 */
#define ms_flash_remap_ctrl_enable_hal()             ms_flash_remap_ctrl_enable_ll()

/**
 * @brief flash remap set remap disable
 * @retval None
 */
#define ms_flash_remap_ctrl_disable_hal()            ms_flash_remap_ctrl_disable_ll()

#endif/* MS_FLASH_REMAP_HAL_H_ */

