/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  flash.c
  * @brief flash driver common api
  * @author che.jiang
  * @date 2022年1月14日
  * @version 1.0
  * @Revision:
  */

#include "ms_flash.h"
#include "flash.h"


QSPIFLASH_Handle_Type qspiflash_handle;
//EXTFLASH_Handle_Type ext_flash_handle;



RAM_FUNCTION void qspiflash_init_calllback(struct __QSPIFLASH_Handle_Type *hqspiflash)
{
    /*ms_qspi_hal_config_master_baud_div(hqspiflash->instance, QSPI_CLK_DIV32);*/
}

RAM_FUNCTION void qspiflash_deinit_calllback(struct __QSPIFLASH_Handle_Type *hqspiflash)
{

}

/**
 * @brief  flash  init
 * @retval None
 */
RAM_FUNCTION int32_t flash_init(FLASH_Handle_Type *hflash)
{
    if(NULL == hflash)
    {
        return STATUS_ERROR;
    }
    if(FLASH_QSPI_TYPE == hflash->flash_type)
    {
        qspiflash_handle.instance = QSPI;
        qspiflash_handle.io_mode = FLASH_IO_MODE_QOUT;
        qspiflash_handle.chip_sel = NULL;
        qspiflash_handle.init_callback = qspiflash_init_calllback;
        qspiflash_handle.deinit_callback = qspiflash_deinit_calllback;

        if(STATUS_SUCCESS != ms_qspiflash_init(&qspiflash_handle))
        {
            return STATUS_ERROR;
        }
    }


    return STATUS_SUCCESS;
}

/**
 * @brief  flash deinit
 * @retval None
 */
RAM_FUNCTION int32_t flash_deinit(FLASH_Handle_Type *hflash)
{
    if(NULL == hflash)
    {
        return STATUS_ERROR;
    }

    if(FLASH_QSPI_TYPE == hflash->flash_type)
    {
        if(STATUS_SUCCESS != ms_qspiflash_deinit(&qspiflash_handle))
        {
            return STATUS_ERROR;
        }
    }
}

/**
 * @brief  flash read id
 * @retval None
 */
RAM_FUNCTION uint32_t flash_read_id(FLASH_Handle_Type *hflash)
{
    if(FLASH_QSPI_TYPE == hflash->flash_type)
    {
        return  ms_qspiflash_read_id(&qspiflash_handle);
    }
    else
    {
        return STATUS_ERROR;
    }
}

/**
 * @brief  flash erase
 * @retval None
 */
RAM_FUNCTION int32_t flash_erase(FLASH_Handle_Type *hflash,uint32_t adr, uint32_t sz)
{
    if(FLASH_QSPI_TYPE == hflash->flash_type)
    {
        return  ms_qspiflash_region_erase(&qspiflash_handle, adr, sz);
    }
    else
    {
        return STATUS_ERROR;
    }
}




/**
 * @brief  flash program
 * @retval None
 */
RAM_FUNCTION int32_t flash_program(FLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    if(FLASH_QSPI_TYPE == hflash->flash_type)
    {
        return ms_qspiflash_program(&qspiflash_handle, adr, sz, buf);
    }
    else
    {
        return STATUS_ERROR;
    }
}


/**
 * @brief  flash read
 * @retval None
 */
RAM_FUNCTION int32_t flash_read(FLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    if(FLASH_QSPI_TYPE == hflash->flash_type)
    {
        return ms_qspiflash_read(&qspiflash_handle, adr, sz, buf);
    }
    else
    {
        return STATUS_ERROR;
    }
}

/**
 * @brief  flash stig read
 * @retval None
 */
RAM_FUNCTION int32_t flash_stig_read(FLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    if(FLASH_QSPI_TYPE == hflash->flash_type)
    {
        return ms_qspiflash_stig_read(&qspiflash_handle, adr, sz, buf);
    }
    else
    {
        return STATUS_ERROR;
    }
}

/**
 * @brief  flash stig write
 * @retval None
 */
RAM_FUNCTION int32_t flash_stig_write(FLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    if(FLASH_QSPI_TYPE == hflash->flash_type)
    {
        return ms_qspiflash_stig_write(&qspiflash_handle, adr, sz, buf);
    }
    else
    {
        return STATUS_ERROR;
    }
}



/**
 * @brief  flash write protected
 * @retval None
 */
RAM_FUNCTION int32_t flash_write_protected(FLASH_Handle_Type *hflash, bool wp)
{
    if(FLASH_QSPI_TYPE == hflash->flash_type)
    {
        return ms_qspiflash_write_protected(&qspiflash_handle, wp);
    }
    else
    {
        return STATUS_ERROR;
    }
}

/**
 * @brief  flash enter deep sleep mode
 * @retval None
 */
RAM_FUNCTION int32_t flash_enter_deepsleep_mode(uint32_t addr, uint32_t *size)
{
  //  if(FLASH_QSPI_TYPE == hflash->flash_type)
  //  {
    int32_t status;

    status =  ms_qspiflash_enter_deep_power_down(&qspiflash_handle, addr, size);
    return status;
 //   }
 //   else
 //   {
 //       return STATUS_ERROR;
 //   }
}


RAM_FUNCTION int32_t flash_write(FLASH_Handle_Type *hflash, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    if(FLASH_QSPI_TYPE == hflash->flash_type)
    {
        return ms_qspiflash_read(&qspiflash_handle, adr, sz, buf);
    }
    else
    {
        return STATUS_ERROR;
    }
}
