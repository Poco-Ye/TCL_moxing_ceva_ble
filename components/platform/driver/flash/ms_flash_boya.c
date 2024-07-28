/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  ms_flash_boya.c
  * @brief boya qspiflash driver common api
  * @author che.jiang
  * @date 2022年1月14日
  * @version 1.0
  * @Revision:
  */

#include "ms_flash_boya.h"
#include <stddef.h>

/* Driver for boya flash chip */

RAM_FUNCTION int32_t ms_qspiflash_boya_probe(uint32_t flash_id)
{
    if((flash_id&0xFF)  != FLASH_BOYA_MFR_ID)
    {
        return STATUS_ERROR;
    }
#if 0
    uint32_t product_id = flash_id & FLASH_BOYA_ID_MASK;
    if (product_id != FLASH_BOYA_PRODUCT_ID)
    {
        return STATUS_ERROR;
    }
#endif
    return STATUS_SUCCESS;
}




const FLASH_Chip_Type flash_chip_boya =
{
    .name = "boya",
    .page_size = FLASH_BOYA_PAGE_SIZE,
    .sector_size = FLASH_BOYA_SECTOR_SIZE,
    .block_size = FLASH_BOYA_BLOCK_64K_SIZE,

    .probe = ms_qspiflash_boya_probe,
    .reset = ms_qspiflash_comm_reset,
    .detect_size = ms_qspiflash_comm_detect_size,
    .erase_chip = ms_qspiflash_comm_erase_chip,
    .erase_sector = ms_qspiflash_comm_erase_sector,
    .erase_block = ms_qspiflash_comm_erase_block,

    .read = ms_qspiflash_comm_read,
    .write = ms_qspiflash_comm_write,
    .stig_read = ms_qspiflash_comm_stig_read,
    .stig_write = ms_qspiflash_comm_stig_write,
    .program_page = ms_qspiflash_comm_page_program,
    .set_io_mode = ms_qspiflash_comm_set_io_mode,
    .get_io_mode = ms_qspiflash_comm_get_io_mode,

};
