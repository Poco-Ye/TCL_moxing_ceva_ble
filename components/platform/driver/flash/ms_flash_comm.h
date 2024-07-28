/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  ms_flash_comm.h
  * @brief qspiflash driver common api
  * @author che.jiang
  * @date 2022年1月14日
  * @version 1.0
  * @Revision:
  */

#ifndef MS_FLASH_COMM_H_
#define MS_FLASH_COMM_H_


#include <stdint.h>
#include <stdbool.h>
#include <ms_qspi_ll.h>
#include "ms_interrupt.h"

/** @defgroup Flash Read Mode Type
  * @{
  */
#define FLASH_IO_MODE_NORMALRD            0x00000000U   /*!< Data read using single I/O, limit speed*/
#define FLASH_IO_MODE_FASTRD              0x00000001U   /*!< Data read using single I/O             */
#define FLASH_IO_MODE_DOUT                0x00000002U   /*!< Data read using dual I/O               */
#define FLASH_IO_MODE_DIO                 0x00000003U   /*!< Address&Data transfer using dual I/O   */
#define FLASH_IO_MODE_QOUT                0x00000004U   /*!< Data read using quad I/O               */
#define FLASH_IO_MODE_QIO                 0x00000005U   /*!< Address&Data transfer using quad I/O   */

/** @defgroup Flash Control command type
  * @{
  */
#define FLASH_CMD_WRITE_ENABLE            0x00000006U
#define FLASH_CMD_WRITE_DISABLE           0x00000004U
#define FLASH_CMD_READ_STATUS             0x00000005U
#define FLASH_CMD_READ_STATUS_2           0x00000035U
#define FLASH_CMD_WRITE_STATUS            0x00000001U
#define FLASH_CMD_VOLT_SR_WRITE_ENABLE    0x00000050U
#define FLASH_CMD_ACTIVE_STATUS_INT       0x00000025U
#define FLASH_CMD_READ_DATA               0x00000003U
#define FLASH_CMD_FAST_READ               0x0000000BU
#define FLASH_CMD_DUAL_OUTPUT_FAST_READ   0x0000003BU
#define FLASH_CMD_QUAD_OUTPUT_FAST_READ   0x0000006BU
#define FLASH_CMD_DUAL_IO_FAST_READ       0x000000BBU
#define FLASH_CMD_QUAD_IO_FAST_READ       0x000000EBU
#define FLASH_CMD_SET_BURST_WITH_WRAP     0x00000077U
#define FLASH_CMD_PAGE_PROGRAM            0x00000002U
#define FLASH_CMD_DUAL_PAGE_PROGRAM       0x000000A2U
#define FLASH_CMD_QUAD_PAGE_PROGRAM       0x00000032U
#define FLASH_CMD_PAGE_ERASE              0x00000081U
#define FLASH_CMD_PAGE_ERASE1             0x000000DBU
#define FLASH_CMD_SECTOR_ERASE            0x00000020U
#define FLASH_CMD_BLOCK_ERASE32K          0x00000052U
#define FLASH_CMD_BLOCK_ERASE64K          0x000000D8U
#define FLASH_CMD_CHIP_ERASE              0x00000060U
#define FLASH_CMD_CHIP_ERASE1             0x00000075U
#define FLASH_CMD_READ_DEVICE_ID          0x00000090U
#define FLASH_CMD_READ_ID                 0x0000009FU
#define FLASH_CMD_READ_UNIQUE_ID          0x0000004BU
#define FLASH_CMD_ERASE_SECURITY_REG      0x00000044U
#define FLASH_CMD_PROGRAM_SECURITY_REG    0x00000042U
#define FLASH_CMD_READ_SECURITY_REG       0x00000048U
#define FLASH_CMD_ENABLE_RESET            0x00000066U
#define FLASH_CMD_RESET                   0x00000099U
#define FLASH_CMD_PROGRAM_SUSPEND         0x00000075U
#define FLASH_CMD_ERASE_SUSPEND           0x00000075U
#define FLASH_CMD_PROGRAM_RESUME          0x0000007AU
#define FLASH_CMD_ERASE_RESUME            0x0000007AU
#define FLASH_CMD_DEEP_POWER_DOWN         0x000000B9U
#define FLASH_CMD_REL_FROM_DPD            0x000000ABU
#define FLASH_CMD_REL_FROM_DPD_READ_DEVID 0x000000ABU
#define FLASH_CMD_RD_SERIAL_DISCIDER_PARA 0x0000005AU
#define FLASH_CMD_MFTR_DEVICE_ID_DUAL_IO  0x00000092U
#define FLASH_CMD_MFTR_DEVICE_ID_QUAD_IO  0x00000094U


/** @defgroup Flash Status Register define
  * @{
  */
#define FLASH_STATUS_REG_WIP              (0x00000001U)
#define FLASH_STATUS_REG_WEL              (0x00000002U)
#define FLASH_STATUS_REG_BP0              (0x00000004U)
#define FLASH_STATUS_REG_BP1              (0x00000008U)
#define FLASH_STATUS_REG_BP2              (0x00000010U)
#define FLASH_STATUS_REG_BP3              (0x00000020U)
#define FLASH_STATUS_REG_BP4              (0x00000040U)
#define FLASH_STATUS_REG_QE               (0x00000200U)
#define FLASH_STATUS_REG_CMP              (0x00004000U)



/**
  * @brief  Flash Chip Type Structure definition
  */
typedef struct __FLASH_Chip_Type
{
    const int8_t *name;

    const uint32_t page_size;
    const uint32_t sector_size;
    const uint32_t block_size;

    int32_t (*probe)(uint32_t flash_id);

    int32_t (*reset)(QSPI_Type *hqspi);


    /* Detect SPI flash size
     *
     * Interrogate the chip to detect its size.
     */
    int32_t (*detect_size)(QSPI_Type *hqspi, uint32_t *size);

    /* Erase the entire chip

       Caller has verified the chip is not write protected.
     */
    int32_t (*erase_chip)(QSPI_Type *hqspi);

    /* Erase a sector of the chip. Sector size is specified in the 'sector_size' field.

       sector_address is an offset in bytes.

       Caller has verified that this sector should be non-write-protected.
     */
    int32_t (*erase_sector)(QSPI_Type *hqspi, uint32_t sector_adr);

    /* Erase a multi-sector block of the chip. Block size is specified in the 'block_erase_size' field.
       sector_address is an offset in bytes.

       Caller has verified that this block should be non-write-protected.
     */
    int32_t (*erase_block)(QSPI_Type *hqspi, uint32_t block_adr);


    /* Read data from the chip.
     *
     * Before calling this function, the caller will have called chip->drv->set_read_mode(chip) in order to configure the chip's read mode correctly.
     */
    int32_t (*read)(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf);

    /* Write any amount of data to the chip.
     */
    int32_t (*write)(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf);


    /* Stig Read data from the chip.
     *
     * Before calling this function, the caller will have called chip->drv->set_read_mode(chip) in order to configure the chip's read mode correctly.
     */
    int32_t (*stig_read)(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf);

    /* Stig Write any amount of data to the chip.
     */
    int32_t (*stig_write)(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf);
    

    /* Use the page program command to write data to the chip.
     *
     * This function is expected to be called by chip->drv->write (if the
     * chip->drv->write implementation doesn't call it then it can be left as NULL.)
     *
     * - The length argument supplied to this function is at most 'page_size' bytes.
     *
     * - The region between 'address' and 'address + length' will not cross a page_size aligned boundary (the write
     *   implementation is expected to split such a write into two before calling page_program.)
     */
    int32_t (*program_page)(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf);

    /* Configure both the SPI host and the chip for the read mode specified in chip->read_mode.
     *
     * This function is called by the higher-level API before the 'read' function is called.
     *
     * Can return ESP_ERR_FLASH_UNSUPPORTED_HOST or ESP_ERR_FLASH_UNSUPPORTED_CHIP if the specified mode is unsupported.
     */
    int32_t (*set_io_mode)(QSPI_Type *hqspi, uint32_t in_io_mode);

    /*
     * Get whether the Quad Enable (QE) is set. (*out_io_mode)=SPI_FLASH_QOUT if
     * enabled, otherwise disabled
     */
    int32_t (*get_io_mode)(QSPI_Type *hqspi, uint32_t* out_io_mode);

    /*
     * Read the chip ID. Called when chip driver is set, but we want to know the exact chip id (to
     * get the size, etc.).
     */
    int32_t (*read_id)(QSPI_Type *hqspi, uint32_t* out_chip_id);

    /*
     * Write Protected..
     */
    int32_t (*write_protected)(QSPI_Type *hqspi, bool wp);
}FLASH_Chip_Type;


/**
 * @brief qspiflash read id  common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
uint32_t ms_qspiflash_comm_read_id(QSPI_Type *hqspi);

/**
 * @brief qspiflash erase sector common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : erase address .
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_comm_erase_sector(QSPI_Type *hqspi, uint32_t adr);

/**
 * @brief qspiflash erase block common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : erase address .
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_comm_erase_block(QSPI_Type *hqspi, uint32_t adr);

/**
 * @brief qspiflash erase chip common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_comm_erase_chip(QSPI_Type *hqspi);

/**
 * @brief qspiflash program page  common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : program address .
 * @param  uint32_t sz      : program size .
 * @param  uint8_t* buf     : program data .
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_comm_page_program(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf);

/**
 * @brief qspiflash detect chip size  common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t* size   : chip size.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_comm_detect_size(QSPI_Type *hqspi, uint32_t *size);

/**
 * @brief qspiflash get io mode   common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t* io_mode: flash io mode.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_comm_get_io_mode(QSPI_Type *hqspi,  uint32_t *io_mode);

/**
 * @brief qspiflash set io mode   common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t io_mode : flash io mode.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_comm_set_io_mode(QSPI_Type *hqspi,  uint32_t io_mode);

/**
 * @brief qspiflash reset  common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_comm_reset(QSPI_Type *hqspi);

/**
 * @brief qspiflash write data  common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : write address .
 * @param  uint32_t sz      : write size .
 * @param  uint8_t* buf     : write data buf.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_comm_write(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf);

/**
 * @brief qspiflash read data  common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : read address .
 * @param  uint32_t sz      : read size .
 * @param  uint8_t* buf     : read data buf.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_comm_read(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf);


/**
 * @brief qspiflash write data  common api (stig mode).
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : write address .
 * @param  uint32_t sz      : write size .
 * @param  uint8_t* buf     : write data buf.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_comm_stig_write(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf);


/**
 * @brief qspiflash read data  common api (stig mode).
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : read address .
 * @param  uint32_t sz      : read size .
 * @param  uint8_t* buf     : read data buf.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_comm_stig_read(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf);


/**
 * @brief qspiflash write protecetd  common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  bool_t wp     : write protected  .
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_comm_write_protected(QSPI_Type *hqspi, bool wp);

/**
 * @brief qspiflash enter deep power down mode .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 *         uint32_t   addr  : Flash Retention Address.
 *         uint32_t   size  : Flash Retention Size.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_qspiflash_comm_enter_deep_power_down(QSPI_Type *hqspi, uint32_t addr, uint32_t *size);




#endif /* MS_FLASH_COMM_H_ */
