/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  ms_flash_gd.h
  * @brief boya qspiflash driver common api
  * @author che.jiang
  * @date 2022年1月14日
  * @version 1.0
  * @Revision:
  */

#ifndef MS_FLASH_GD_H_
#define MS_FLASH_GD_H_


#include <ms_qspi_ll.h>
#include "ms_interrupt.h"
#include "ms_flash_comm.h"
#include "ms_flash.h"


#include <stdint.h>
#include <stdbool.h>

/** @defgroup Flash GD desc
  * @{
  */
#define FLASH_GD_ID_MASK                  0x0000FF00U
#define FLASH_GD_SIZE_MASK                0x000000FFU
#define FLASH_GD_MFR_ID                   0x000000C8U
#define FLASH_GD25Q_PRODUCT_ID            0x00004000U
#define FLASH_GD25LQ_PRODUCT_ID           0x00006000U

/** @defgroup Flash size
  * @{
  */
#define FLASH_GD_PAGE_SIZE                0x00000100U
#define FLASH_GD_SECTOR_SIZE              0x00001000U
#define FLASH_GD_BLOCK_32K_SIZE           0x00008000U
#define FLASH_GD_BLOCK_64K_SIZE           0x00010000U


extern const FLASH_Chip_Type flash_chip_gd;

#endif /* MS_FLASH_GD_H_ */
