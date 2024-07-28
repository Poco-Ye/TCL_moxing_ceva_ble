/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  ms_flash_remap.h
 * @brief
 * @author che.jiang
 * @date 2022年1月14日
 * @version 1.0
 * @Revision:
 */
#ifndef MS_FLASH_REMAP_H_
#define MS_FLASH_REMAP_H_

#include "ms1008.h"
#include "ms_flash_remap_hal.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  Set Flash Remap Address
 * @param[in]  src_addr:  Flash Remap source address.
 * @param[in]  dst_addr:  Flash Remap destination address.
 * @param[in]  offset:  Flash Remap region offset.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_flash_remap_set_addr(uint32_t src_addr, uint32_t dst_addr, uint32_t offset);

/**
 * @brief  Set Flash Remap Enable
 * @param[in]  none
 * @retval none
 */
void ms_flash_remap_enable();

/**
 * @brief  Set Flash Remap Disable
 * @param[in]  none
 * @retval none
 */
void ms_flash_remap_disable();



#endif /* MS_FLASH_REMAP_H_ */
