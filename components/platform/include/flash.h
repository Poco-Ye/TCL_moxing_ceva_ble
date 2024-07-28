/*
 * flash.h
 *
 *  Created on: 2021年12月21日
 *      Author: che.jiang
 */

#ifndef FLASH_H_
#define FLASH_H_

#include "ms1008.h"
#include <stddef.h>
#include <stdbool.h>

/** @defgroup Flash Type
  * @{
  */
#define FLASH_QSPI_TYPE                   0x00U   /*!< QspiFlash*/
#define FLASH_OFFCHIP_TYPE                0x01U   /*!< Offchip type Flash            */

/**
  * @brief  FLASH handle Structure definition
  */
typedef struct __FLASH_Handle_Type
{
    uint8_t                     flash_type;               /*!< 0-QSPI Flash,1-Off-chip Flash       */
} FLASH_Handle_Type;


extern int32_t flash_init(FLASH_Handle_Type *hflash);
extern int32_t flash_deinit(FLASH_Handle_Type *hflash);
extern int32_t flash_erase(FLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz);
extern int32_t flash_program(FLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf);
extern int32_t flash_read(FLASH_Handle_Type *hflash,uint32_t adr, uint32_t sz, uint8_t *buf);
extern int32_t flash_stig_read(FLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf);
extern int32_t flash_stig_write(FLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf);
extern uint32_t flash_read_id(FLASH_Handle_Type *hflash);
extern int32_t flash_write_protected(FLASH_Handle_Type *hflash, bool wp);
extern int32_t flash_enter_deepsleep_mode( uint32_t addr, uint32_t *size);

#endif /* FLASH_H_ */
