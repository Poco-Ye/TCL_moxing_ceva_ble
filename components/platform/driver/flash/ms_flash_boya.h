/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  ms_flash_boya.h
  * @brief boya qspiflash driver common api
  * @author che.jiang
  * @date 2022年1月14日
  * @version 1.0
  * @Revision:
  */

#ifndef MS_FLASH_BOYA_H_
#define MS_FLASH_BOYA_H_


#include "ms_qspi_ll.h"
#include "ms_interrupt.h"
#include "ms_flash_comm.h"
#include "ms_flash.h"

#include <stdint.h>
#include <stdbool.h>

/** @defgroup Flash GD desc
  * @{
  */
#define FLASH_BOYA_ID_MASK                  0x0000FF00U
#define FLASH_BOYA_SIZE_MASK                0x000000FFU
#define FLASH_BOYA_MFR_ID                   0x00000068U
#define FLASH_BOYA_PRODUCT_ID               0x00004000U

/** @defgroup Flash size
  * @{
  */
#define FLASH_BOYA_PAGE_SIZE                0x00000100U
#define FLASH_BOYA_SECTOR_SIZE              0x00001000U
#define FLASH_BOYA_BLOCK_32K_SIZE           0x00008000U
#define FLASH_BOYA_BLOCK_64K_SIZE           0x00010000U


extern const FLASH_Chip_Type flash_chip_boya;

#endif /* MS_FLASH_BOYA_H_ */
