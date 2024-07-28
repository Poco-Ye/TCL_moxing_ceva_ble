/* -----------------------------------------------------------------------------
 * Copyright (c) 2014 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * $Date:        14. Jan 2014
 * $Revision:    V1.00
 *
 * Project:      FlashOS Headerfile for Flash drivers
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.00
 *    Initial release
 */
#ifndef _MS_FLASH_ALG_H_
#define _MS_FLASH_ALG_H_

#ifdef __cplusplus
    extern "C" {
#endif
#include "ms_flash_alg_reg.h"
			
#define FLASH_BLOCK_NUM             16
#define FLASH_BLOCK_SECTOR_SIZE     16
#define FLASH_SECTOR_PAGE_SIZE      16
#define FLASH_PAGE_BYTE_SIZE        256

#define FLASH_SECTOR_NUM            ( FLASH_BLOCK_NUM  * FLASH_BLOCK_SECTOR_SIZE )
#define FLASH_PAGE_NUM              ( FLASH_SECTOR_NUM * FLASH_SECTOR_PAGE_SIZE  )

#define FLASH_BLOCK_PAGE_SIZE       ( FLASH_BLOCK_SECTOR_SIZE * FLASH_SECTOR_PAGE_SIZE )
#define FLASH_BLOCK_BYTE_SIZE       ( FLASH_BLOCK_SECTOR_SIZE * FLASH_SECTOR_BYTE_SIZE )
#define FLASH_SECTOR_BYTE_SIZE      ( FLASH_SECTOR_PAGE_SIZE  * FLASH_PAGE_BYTE_SIZE   )

#define FLASH_DRIVER_SEG __attribute__((section(".func")))			
			
typedef enum QSPI_RW_TYPE {
    QSPI_READ,
    QSPI_WRITE
} QSPI_RW_TYPE;

typedef enum QSPI_MODE_TYPE {
    QSPI_QPI_MODE,
    QSPI_DPI_MODE,
    QSPI_SPI_MODE
} QSPI_MODE_TYPE;

// GD flash command
typedef enum FLASH_CMD_TYPE {
    FLASH_RESET_ENABLE      = 0x66,
    FLASH_RESET             = 0x99,
    FLASH_WRITE_ENABLE      = 0x06,
    FLASH_WRITE_DISABLE     = 0x04,
    FLASH_READ_STATUS       = 0x05,     // GD: read_status_1
    FLASH_READ_STATUS_2     = 0x35,     // GD: read_status_2
    FLASH_WRITE_STATUS      = 0x01,
    FLASH_ENABLE_QPI        = 0x38,

    FLASH_READ_DATA         = 0x03,
    FLASH_FAST_DATA         = 0x0B,
    FLASH_DUAL_FAST_DATA    = 0x3B,
    FLASH_QUAD_FAST_DATA    = 0x6B,
    FLASH_DUAL_IO_READ      = 0xBB,     // address dual, data dual
    FLASH_QUAD_IO_READ      = 0xEB,     // address quad, data quad
    FLASH_PAGE_PROGRAM      = 0x02,
    FLASH_QUAD_PAGE_PROGRAM = 0x32,
    FLASH_SECTOR_ERASE      = 0x20,
    FLASH_BLOCK_ERASE       = 0x52,
    FLASH_CHIP_ERASE        = 0x60,

    FLASH_DEEP_POWERDOWN = 0xB9,
    FLASH_REL_FROM_DEEP_POWERDOWN = 0xAB,
} FLASH_CMD_TYPE;

extern int  ms_flash_alg_init (void);
extern int  ms_flash_alg_erase(unsigned int cmd, unsigned int address);
extern int  ms_flash_alg_programpage(unsigned int address, unsigned int sz, unsigned char *buf);
extern int  ms_flash_alg_read_id(void);
extern int  ms_flash_alg_modify_clk(void);
extern FLASH_DRIVER_SEG void qspi_stig_single_command(FLASH_CMD_TYPE cmd);
extern FLASH_DRIVER_SEG void qspi_stig_write_status(unsigned int status, unsigned int size);
extern FLASH_DRIVER_SEG void qspi_stig_wait_wip(void);
extern FLASH_DRIVER_SEG void qspi_stig_erase(FLASH_CMD_TYPE cmd, unsigned int addr) ;
extern FLASH_DRIVER_SEG UINT32   qspi_stig_read_spi_flash_Identification(void);
extern FLASH_DRIVER_SEG int ms_flash_alg_program(unsigned int address, unsigned int sz, unsigned char *buf);
extern FLASH_DRIVER_SEG int ms_flash_stig_write(unsigned int address, unsigned int size, unsigned char *buf);
extern FLASH_DRIVER_SEG int ms_flash_stig_read(unsigned int address, unsigned int size, unsigned char *buf);
#ifdef __cplusplus
   }
#endif

#endif //_MS_FLASH_ALG_H_