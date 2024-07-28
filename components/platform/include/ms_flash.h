/*
 * ms_flash.h
 *
 *  Created on: 2021年12月21日
 *      Author: che.jiang
 */

#ifndef MS_QSPIFLASH_H_
#define MS_QSPIFLASH_H_

#include <ms_flash_comm.h>
#include <ms_qspi_hal.h>

#include <stdint.h>
#include <stdbool.h>


/**
  * @brief  QSPIFLASH handle Structure definition
  */
typedef struct __QSPIFLASH_Handle_Type
{
    QSPI_Type                     *instance;               /*!< QSPI registers base address        */
    FLASH_Chip_Type               *chip_sel;               /*!< QSPI  select Chip                  */
    uint8_t                        io_mode;                /*!< Flash IO mode                      */
    uint32_t                       chip_id;                /*!< Flash Chip Id                      */
    uint32_t                       chip_size;              /*!< Flash Chip Size                    */
    void (*init_callback)(struct __QSPIFLASH_Handle_Type *hqspiflash);/*!< QspiFlash Initial callback handle  */
    void (*deinit_callback)(struct __QSPIFLASH_Handle_Type *hqspiflash);/*!< QspiFlash De-initial callback handle  */
} QSPIFLASH_Handle_Type;


/**
 * @brief This function initial flash module  .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_init (QSPIFLASH_Handle_Type *hflash);


/**
 * @brief This function deinitial flash module  .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_deinit (QSPIFLASH_Handle_Type *hflash);


/**
 * @brief This function erase flash Region .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @param  uint32_t adr  Flash Erase Address.
 * @param  uint32_t sz   Flash Erase size.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_region_erase(QSPIFLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz);


/**
 * @brief This function erase flash total Chip .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @param  uint32_t adr  Flash Erase Address.
 * @param  uint32_t sz   Flash Erase size.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_chip_erase(QSPIFLASH_Handle_Type *hflash);

/**
 * @brief This function program flash region .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @param  uint32_t adr  Flash program Address.
 * @param  uint32_t sz   Flash program size.
 * @param  uint8_t_t *buf Flash write data buffer.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_program(QSPIFLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf);

/**
 * @brief This function write data to flash  .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @param  uint32_t adr  Flash write Address.
 * @param  uint32_t sz   Flash write size.
 * @param  uint8_t_t *buf Flash write data buffer.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_write(QSPIFLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf);

/**
 * @brief This function read data from flash  .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @param  uint32_t adr  Flash read Address.
 * @param  uint32_t sz   Flash read size.
 * @param  uint8_t_t *buf Flash write data buffer.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_read(QSPIFLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf);

/**
 * @brief This function write data to flash  (stig mode).
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @param  uint32_t adr  Flash write Address.
 * @param  uint32_t sz   Flash write size.
 * @param  uint8_t_t *buf Flash write data buffer.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_stig_write(QSPIFLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf);


/**
 * @brief This function read data from flash  (stig mode).
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @param  uint32_t adr  Flash read Address.
 * @param  uint32_t sz   Flash read size.
 * @param  uint8_t_t *buf Flash read data buffer.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_stig_read(QSPIFLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf);


/**
 * @brief This function read flash id from chip  .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
uint32_t ms_qspiflash_read_id(QSPIFLASH_Handle_Type *hflash);

/**
 * @brief This function write protected .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @param  bool wp : write protected.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_write_protected(QSPIFLASH_Handle_Type *hflash, bool wp);

/**
 * @brief This function read flash mode  .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
uint32_t ms_qspiflash_read_mode(QSPIFLASH_Handle_Type *hflash);


/**
 * @brief This function run flash enter deep powr down mode  .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 *         uint32_t   addr  : Flash Retention Address.
 *         uint32_t   size  : Flash Retention Size.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_enter_deep_power_down(QSPIFLASH_Handle_Type *hflash, uint32_t addr, uint32_t *size);


#endif /* MS_QSPIFLASH_H_ */
