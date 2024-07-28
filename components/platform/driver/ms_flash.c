/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  ms_flash.c
  * @brief qspiflash device driver
  * @author che.jiang
  * @date 2022年1月14日
  * @version 1.0
  * @Revision:
  */
#include "ms_flash.h"
#include "ms_flash_comm.h"
#include "ms_flash_gd.h"
#include "ms_flash_boya.h"


#define ALIGN_DOWN(x, align) ((unsigned long)(x) & ~((unsigned long)align - 1))

static const FLASH_Chip_Type *default_flash_chips[] =
{
    &flash_chip_gd,
    &flash_chip_boya,
    NULL,
};

/**
 * @brief This function detect chip  .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_detect_chip(QSPIFLASH_Handle_Type *hflash)
{
    if (NULL == hflash)
    {
        return STATUS_ERROR;
    }
    const FLASH_Chip_Type **chip_sel = default_flash_chips;

    hflash->chip_sel = NULL;
    hflash->chip_id = ms_qspiflash_comm_read_id(hflash->instance);

    while (*chip_sel != NULL)
    {
        hflash->chip_sel = (FLASH_Chip_Type *)(*chip_sel);
        if (hflash->chip_sel->probe(hflash->chip_id) == STATUS_SUCCESS)
        {

            return STATUS_SUCCESS;
        }

        chip_sel++;
    }

    hflash->chip_sel = NULL;

    return STATUS_ERROR;
}

/**
 * @brief This function initial flash module  .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_init(QSPIFLASH_Handle_Type *hflash)
{
    if (NULL == hflash)
    {
        return STATUS_ERROR;
    }

    /**initial the clock*/
    if(hflash->init_callback)
    {
        hflash->init_callback(hflash);
    }

    /*detect flash id and probe device to driver*/
    if(STATUS_SUCCESS != ms_qspiflash_detect_chip(hflash))
    {
        return STATUS_ERROR;
    }

    return STATUS_SUCCESS;
}

/**
 * @brief This function deinitial flash module  .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_deinit(QSPIFLASH_Handle_Type *hflash)
{
    if (NULL == hflash)
    {
        return STATUS_ERROR;
    }

    /**initial the clock*/
    if(hflash->deinit_callback)
    {
        hflash->deinit_callback(hflash);
    }

    hflash->chip_sel = NULL;

    return STATUS_SUCCESS;
}

/**
 * @brief This function erase flash Region .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @param  uint32_t adr  Flash Erase Address.
 * @param  uint32_t sz   Flash Erase size.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_region_erase(QSPIFLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz)
{
    if(NULL == hflash || NULL == hflash->chip_sel)
    {
        return STATUS_ERROR;
    }
    FLASH_Chip_Type *chip_sel = (FLASH_Chip_Type *)hflash->chip_sel;
    if(0 == chip_sel->page_size)
    {
        return STATUS_ERROR;
    }


    uint32_t addr, sector_index;

    uint32_t start_addr = ALIGN_DOWN(adr, chip_sel->sector_size);
    uint32_t end_addr = ALIGN_DOWN((adr + sz + (chip_sel->sector_size -1)), chip_sel->sector_size);

    for (addr = start_addr; addr < end_addr; addr += chip_sel->sector_size)
    {
        sector_index = (adr - FLASH_BASE_ADDR)/(chip_sel->sector_size);
        if (STATUS_SUCCESS != chip_sel->erase_sector(hflash->instance, sector_index))
        {
            return STATUS_ERROR;
        }
    }

    return STATUS_SUCCESS;
}


/**
 * @brief This function program flash region .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @param  uint32_t adr  Flash Erase Address.
 * @param  uint32_t sz   Flash Erase size.
 * @param  uint8_t_t *buf Flash write data buffer.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_program(QSPIFLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    if(NULL == hflash || NULL == hflash->chip_sel)
    {
        return STATUS_ERROR;
    }

    FLASH_Chip_Type *chip_sel = (FLASH_Chip_Type *)hflash->chip_sel;
    if(0 == chip_sel->page_size)
    {
        return STATUS_ERROR;
    }

    uint32_t prg_size;
    int32_t left_buf_len = sz;
    uint32_t start_addr = adr;
    uint32_t end_addr = start_addr + sz;

    while(left_buf_len > 0)
    {
        if((end_addr / chip_sel->page_size) > (start_addr / chip_sel->page_size))
        {
            prg_size = chip_sel->page_size - (start_addr % chip_sel->page_size);
        }
        else
        {
            prg_size = left_buf_len;
        }

        chip_sel->program_page(hflash->instance, start_addr, prg_size, buf);
        buf += prg_size;
        start_addr += prg_size;
        left_buf_len -= prg_size;
    }
    return STATUS_SUCCESS;
}


/**
 * @brief This function read data from flash  .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @param  uint32_t adr  Flash Erase Address.
 * @param  uint32_t sz   Flash Erase size.
 * @param  uint8_t_t *buf Flash write data buffer.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_read(QSPIFLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    if(NULL == hflash || NULL == hflash->chip_sel)
    {
        return STATUS_ERROR;
    }
    FLASH_Chip_Type *chip_sel = (FLASH_Chip_Type *)hflash->chip_sel;
    if(0 == chip_sel->page_size)
    {
        return STATUS_ERROR;
    }

    uint32_t prg_size;
    int32_t left_buf_len = sz;
    uint32_t start_addr = adr;
    uint32_t end_addr = start_addr + sz;

    while(left_buf_len > 0)
    {
        if((end_addr / chip_sel->page_size) > (start_addr / chip_sel->page_size))
        {
            prg_size = chip_sel->page_size - (start_addr % chip_sel->page_size);
        }
        else
        {
            prg_size = left_buf_len;
        }

        chip_sel->read(hflash->instance, start_addr, prg_size, buf);
        buf += prg_size;
        start_addr += prg_size;
        left_buf_len -= prg_size;
    }

    return STATUS_SUCCESS;
}

/**
 * @brief This function write data to flash  .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @param  uint32_t adr  Flash Erase Address.
 * @param  uint32_t sz   Flash Erase size.
 * @param  uint8_t_t *buf Flash write data buffer.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_write(QSPIFLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    if(NULL == hflash)
    {
        return STATUS_ERROR;
    }

    FLASH_Chip_Type *chip_sel = (FLASH_Chip_Type *)hflash->chip_sel;
    if(chip_sel)
    {
        chip_sel->write(hflash->instance, adr, sz, buf);
    }
    
    return STATUS_SUCCESS;
}

/**
 * @brief This function read data from flash(stig mode)  .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @param  uint32_t adr  Flash Erase Address.
 * @param  uint32_t sz   Flash Erase size.
 * @param  uint8_t_t *buf Flash read data buffer.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_stig_read(QSPIFLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    if(NULL == hflash)
    {
        return STATUS_ERROR;
    }

    FLASH_Chip_Type *chip_sel = (FLASH_Chip_Type *)hflash->chip_sel;
    if(chip_sel)
    {
        if(0 == chip_sel->page_size)
        {
            return STATUS_ERROR;
        }
  
        chip_sel->stig_read(hflash->instance, adr, sz, buf);
    }
    
    return STATUS_SUCCESS;
}

/**
 * @brief This function write data to flash (stig mode) .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @param  uint32_t adr  Flash Erase Address.
 * @param  uint32_t sz   Flash Erase size.
 * @param  uint8_t_t *buf Flash write data buffer.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_stig_write(QSPIFLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    if(NULL == hflash)
    {
        return STATUS_ERROR;
    }

    FLASH_Chip_Type *chip_sel = (FLASH_Chip_Type *)hflash->chip_sel;
    if(chip_sel)
    {
        if(0 == chip_sel->page_size)
        {
            return STATUS_ERROR;
        }

        chip_sel->stig_write(hflash->instance, adr, sz, buf);
    }
    
    return STATUS_SUCCESS;
}



/**
 * @brief This function read flash id from chip  .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION uint32_t ms_qspiflash_read_id(QSPIFLASH_Handle_Type *hflash)
{
    if(NULL == hflash)
    {
        return STATUS_ERROR;
    }

    return ms_qspiflash_comm_read_id(hflash->instance);
}


/**
 * @brief This function write protected .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 * @param  bool wp : write protected.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_write_protected(QSPIFLASH_Handle_Type *hflash, bool wp)
{
    if(NULL == hflash || NULL == hflash->chip_sel)
    {
        return STATUS_ERROR;
    }

    FLASH_Chip_Type *chip_sel = (FLASH_Chip_Type *)hflash->chip_sel;

    return chip_sel->write_protected(hflash->instance, wp);
}


/**
 * @brief This function run flash enter deep powr down mode  .
 * @param  QSPIFLASH_Handle_Type *hflash : Pointer to  QspiFlash module.
 *         uint32_t   addr  : Flash Retention Address.
 *         uint32_t   size  : Flash Retention Size.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_enter_deep_power_down(QSPIFLASH_Handle_Type *hflash, uint32_t addr, uint32_t *size)
{
    if(NULL == hflash)
    {
        return STATUS_ERROR;
    }

    return  ms_qspiflash_comm_enter_deep_power_down(hflash->instance, addr, size);
}
