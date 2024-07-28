/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  ms_flash_remap.c
 * @brief
 * @author bingrui.chen
 * @date 2022-1-14
 * @version 1.0
 * @Revision:
 */

#include <string.h>
#include "ms_flash_remap.h"
#include "log.h"


int32_t ms_flash_remap_set_addr(uint32_t src_addr, uint32_t dst_addr, uint32_t offset)
{
  
    if((offset == 0) || (dst_addr <= src_addr))
    {
        MS_LOGE(MS_DRIVER, "flash remap error");
		return STATUS_ERROR;
    }
	
    ms_flash_remap_set_src_addr_hal(src_addr);

    ms_flash_remap_set_region_oft_hal(offset);

    ms_flash_remap_set_dst_addr_hal(dst_addr - src_addr);

    return STATUS_SUCCESS;
}


void ms_flash_remap_enable()
{
    ms_flash_remap_ctrl_enable_hal();
}

void ms_flash_remap_disable()
{
    ms_flash_remap_ctrl_disable_hal();
}
