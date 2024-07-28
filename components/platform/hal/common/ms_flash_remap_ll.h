/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_flash_remap_ll.h
 * @brief Header file of flash remap  module.
 * @author che.jiang
 * @date   2022-01-05
 * @version 1.0
 * @Revision
 */

#ifndef MS_FLASH_REMAP_LL_H_
#define MS_FLASH_REMAP_LL_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>


/**
 * @brief  Set the OTA_SRC_ADDR
 * @param  uint32_t addr: remap source address
 * @retval none
 */
static inline void ms_flash_remap_set_src_addr_ll(uint32_t addr)
{
    WRITE_REG(SYS_CTRL->OTA_SRC_ADDR, addr);
}

/**
 * @brief  Set the OTA_DST_ADDR
 * @param  uint32_t addr: remap source address
 * @retval none
 */
static inline void ms_flash_remap_set_dst_addr_ll(uint32_t addr)
{
    WRITE_REG(SYS_CTRL->OTA_DST_ADDR, addr);
}

/**
 * @brief  Set the OTA_OFT_ADDR
 * @param  uint32_t addr: remap source address
 * @retval none
 */
static inline void ms_flash_remap_set_region_oft_ll(uint32_t addr)
{
    WRITE_REG(SYS_CTRL->OTA_OFT_ADDR, addr);
}

/**
 * @brief  Set the REMAP_CTRL enable
 * @param  none
 * @retval none
 */
static inline void ms_flash_remap_ctrl_enable_ll()
{
    SET_BIT(SYS_CTRL->REMAP_CTRL, SYS_CTRL_REMAP_CTRL_OTA_EN);
}

/**
 * @brief  Set the REMAP_CTRL disable
 * @param  none
 * @retval none
 */
static inline void ms_flash_remap_ctrl_disable_ll()
{
    CLEAR_BIT(SYS_CTRL->REMAP_CTRL, SYS_CTRL_REMAP_CTRL_OTA_EN);
}

#endif

