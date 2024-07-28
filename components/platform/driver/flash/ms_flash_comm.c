/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  ms_flash_comm.c
  * @brief qspiflash driver common api
  * @author che.jiang
  * @date 2022年1月14日
  * @version 1.0
  * @Revision:
  */

#include "ms_flash_comm.h"
#include "ms_qspi_hal.h"
#include <stddef.h>
#include "log.h"



/**
  * @} Flash Chip Retention Command definition
  */
typedef struct
{
    uint32_t   reg_addr;
    uint32_t   reg_val;
    uint32_t   wait_cnt;
    uint32_t   check_sum;
} ms_sleep_flash_cmd;

/**
  * @} Flash Chip Retention Structure definition
  */
typedef struct
{
    uint8_t     header[4];      //"MSFR"
    uint32_t    magic;
    uint16_t    cmd_number;     //flash cmd number
    uint8_t     is_valid;       //flash cmd valid flag,0-unvalid,1-valid
    uint8_t     un_used;        //no used
    ms_sleep_flash_cmd *p_cmd;
} ms_sleep_flash_data;

RAM_FUNCTION void ms_qspiflash_comm_polling_wip(QSPI_Type *hqspi)
{
    uint32_t reg_value = 0;
    reg_value = (1 << QSPI_FCCR_EXC_CMD_POS) |
                (FLASH_CMD_READ_STATUS << QSPI_FCCR_CMD_OPCODE_POS) |
                (1 << QSPI_FCCR_R_DATA_EN_POS) |
                (0 << QSPI_FCCR_R_BYTE_NUM_POS);

    do
    {
        wait_nop(20);
        WRITE_REG(hqspi->FCCR, reg_value);
        while (QSPI_STATUS_IS_STIG_EXE(hqspi));
    } while (READ_REG(hqspi->FCRDR_L) & FLASH_STATUS_REG_WIP);
}



/**
 * @brief qspiflash erase sector common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : erase address .
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_comm_erase_sector(QSPI_Type *hqspi, uint32_t adr)
{
    uint32_t reg_value= 0;

    // send enable write
    ms_qspi_hal_flash_control_command(hqspi, FLASH_CMD_WRITE_ENABLE);


    //should be insure sector addr and size align to sector size
    WRITE_REG(hqspi->FCAR,  adr);

    // Erase the sector size
    reg_value = (1 << QSPI_FCCR_EXC_CMD_POS) |
                (FLASH_CMD_SECTOR_ERASE << QSPI_FCCR_CMD_OPCODE_POS) |
                (QSPI_ADR_NUMER_BYTE2 << QSPI_FCCR_ADR_BYTE_NUM_POS) |
                (1 << QSPI_FCCR_ADR_EN_POS);

    WRITE_REG(hqspi->FCCR, reg_value);
    while (QSPI_STATUS_IS_STIG_EXE(hqspi));

    ms_qspiflash_comm_polling_wip(hqspi);

    return STATUS_SUCCESS;
}

/**
 * @brief qspiflash erase block common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : erase address .
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_comm_erase_block(QSPI_Type *hqspi, uint32_t adr)
{
    return STATUS_SUCCESS;
}



/**
 * @brief qspiflash erase chip common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_comm_erase_chip(QSPI_Type *hqspi)
{
    return STATUS_SUCCESS;
}



/**
 * @brief qspiflash program page  common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : program address .
 * @param  uint32_t sz      : program size .
 * @param  uint8_t* buf     : program data .
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_comm_page_program(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    int32_t remain_sz = (int32_t)sz;

    ms_qspi_set_ind_trig_adr(hqspi, adr, QSPI_IND_RANGE_WIDTH_64);

    ms_qspi_cfg_ind_wr(hqspi, adr - FLASH_BASE_ADDR, sz);

    while(0 < remain_sz)
    {
        // Program the next word.
        WRITE_REG(*(volatile uint32_t *)adr, *(uint32_t *)buf);

        // Increment to the next word.
        buf += 4;
        remain_sz -= 4;
    }

    //if need check flash status is idle
    ms_qspiflash_comm_polling_wip(hqspi);

    return STATUS_SUCCESS;
}


/**
 * @brief qspiflash read data  common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : read address .
 * @param  uint32_t sz      : read size .
 * @param  uint8_t* buf     : read data buf.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_comm_read(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    int32_t remain_sz = (int32_t)sz;
    uint32_t *read_word = (uint32_t*)buf;

    ms_qspi_set_ind_trig_adr(hqspi, adr, QSPI_IND_RANGE_WIDTH_64);

    ms_qspi_cfg_ind_rd(hqspi, adr - FLASH_BASE_ADDR, sz);

    while(0 < remain_sz)
    {
        // Program the next word.
        *read_word = READ_REG(*(volatile uint32_t *)adr);

        // Increment to the next word.
        read_word++;
        remain_sz -= 4;
    }

    return STATUS_SUCCESS;
}

/**
 * @brief qspiflash read data  common api (stig mode).
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : read address .
 * @param  uint32_t sz      : read size .
 * @param  uint8_t* buf     : read data buf.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_comm_stig_read(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    uint32_t reg_value= 0;
    uint8_t *read_data = buf;
    uint32_t adr_offset = adr - FLASH_BASE_ADDR;
    uint32_t read_addr = adr_offset;

    for(uint32_t i=0; i < sz; i+=1)
    {
        read_addr = adr_offset + i;
        
        // send enable write
        ms_qspi_hal_flash_control_command(hqspi, FLASH_CMD_WRITE_ENABLE);
        
        
        //write addr
        WRITE_REG(hqspi->FCAR,  read_addr);

        reg_value = (1 << QSPI_FCCR_EXC_CMD_POS) |
                    (FLASH_CMD_FAST_READ << QSPI_FCCR_CMD_OPCODE_POS) |
                    (QSPI_ADR_NUMER_BYTE3 << QSPI_FCCR_ADR_BYTE_NUM_POS) |
                    (1 << QSPI_FCCR_ADR_EN_POS) |
                    (0 << QSPI_FCCR_R_BYTE_NUM_POS) |
                    (8 << QSPI_FCCR_DUMMY_NUM_POS) |
                    (1 << QSPI_FCCR_R_DATA_EN_POS);
        
        WRITE_REG(hqspi->FCCR, reg_value);
        while (QSPI_STATUS_IS_STIG_EXE(hqspi));     
        
        //read data
        *read_data = (uint8_t)READ_REG(hqspi->FCRDR_L);

        //clear read cmd
        ms_qspi_hal_flash_control_clear_command(hqspi);
        read_data += 1;

    }


 
    return STATUS_SUCCESS;

}

/**
 * @brief qspiflash read data  common api (stig mode).
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : read address .
 * @param  uint32_t sz      : read size .
 * @param  uint8_t* buf     : read data buf.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_comm_stig_read_word(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    uint32_t reg_value= 0;
    uint8_t *read_data = buf;
    uint32_t adr_offset = adr - FLASH_BASE_ADDR;
    uint32_t read_addr = adr_offset;


    for(uint32_t i=0; i < sz; i+=8)
    {
        read_addr = adr_offset + i;
        
        // send enable write
        ms_qspi_hal_flash_control_command(hqspi, FLASH_CMD_WRITE_ENABLE);
        
        
        //write addr
        WRITE_REG(hqspi->FCAR,  read_addr);

        reg_value = (1 << QSPI_FCCR_EXC_CMD_POS) |
                    (FLASH_CMD_FAST_READ << QSPI_FCCR_CMD_OPCODE_POS) |
                    (QSPI_ADR_NUMER_BYTE3 << QSPI_FCCR_ADR_BYTE_NUM_POS) |
                    (1 << QSPI_FCCR_ADR_EN_POS) |
                    (7 << QSPI_FCCR_R_BYTE_NUM_POS) |
                    (8 << QSPI_FCCR_DUMMY_NUM_POS) |
                    (1 << QSPI_FCCR_R_DATA_EN_POS);
        
        WRITE_REG(hqspi->FCCR, reg_value);
        while (QSPI_STATUS_IS_STIG_EXE(hqspi));     
        
        //read data
        *(uint32_t *)read_data = READ_REG(hqspi->FCRDR_L);
        *(uint32_t *)(read_data+4) = READ_REG(hqspi->FCRDR_H);

        //clear read cmd
        ms_qspi_hal_flash_control_clear_command(hqspi);
        read_data += 8;

    }


 
    return STATUS_SUCCESS;

}



/**
 * @brief qspiflash write data  common api (stig mode).
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : write address .
 * @param  uint32_t sz      : write size .
 * @param  uint8_t* buf     : write data buf.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */

RAM_FUNCTION int32_t ms_qspiflash_comm_stig_write(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    uint32_t reg_value= 0;
    uint8_t *write_data = buf;
    uint32_t adr_offset = adr - FLASH_BASE_ADDR;
    uint32_t write_addr = adr_offset;


    for(uint32_t i=0; i < sz; i+=1)
    {
        write_addr = adr_offset + i;
   
        // send enable write
        ms_qspi_hal_flash_control_command(hqspi, FLASH_CMD_WRITE_ENABLE);
 
        //write addr
        WRITE_REG(hqspi->FCAR,  write_addr);
        
        //write data
        WRITE_REG(hqspi->FCWDR_L,  *write_data);
        
        // program 1 byte
        reg_value = (1 << QSPI_FCCR_EXC_CMD_POS) |
                    (FLASH_CMD_PAGE_PROGRAM << QSPI_FCCR_CMD_OPCODE_POS) |
                    (QSPI_ADR_NUMER_BYTE3 << QSPI_FCCR_ADR_BYTE_NUM_POS) |
                    (1 << QSPI_FCCR_ADR_EN_POS) |
                    (0 << QSPI_FCCR_W_BYTE_NUM_POS) |
                    (1 << QSPI_FCCR_W_DATA_EN_POS);
      
        WRITE_REG(hqspi->FCCR, reg_value);
        while (QSPI_STATUS_IS_STIG_EXE(hqspi));
        
        ms_qspiflash_comm_polling_wip(hqspi);
        ms_qspi_hal_flash_control_clear_command(hqspi);

        write_data += 1;

    }

    return STATUS_SUCCESS;

}



/**
 * @brief qspiflash write data  common api (stig mode).
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : write address .
 * @param  uint32_t sz      : write size .
 * @param  uint8_t* buf     : write data buf.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */

RAM_FUNCTION int32_t ms_qspiflash_comm_stig_write_word(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    uint32_t reg_value= 0;
    uint8_t *write_data = buf;
    uint32_t adr_offset = adr - FLASH_BASE_ADDR;
    uint32_t write_addr = adr_offset;

    for(uint32_t i=0; i < sz; i+=8)
    {
        write_addr = adr_offset + i;
   
        // send enable write
        ms_qspi_hal_flash_control_command(hqspi, FLASH_CMD_WRITE_ENABLE);
 
        //write addr
        WRITE_REG(hqspi->FCAR,  write_addr);
        
        //write data
        WRITE_REG(hqspi->FCWDR_L,  *(uint32_t *)write_data);
        WRITE_REG(hqspi->FCWDR_H,  *(uint32_t *)(write_data+4));
        
        // program 2 word
        reg_value = (1 << QSPI_FCCR_EXC_CMD_POS) |
                    (FLASH_CMD_PAGE_PROGRAM << QSPI_FCCR_CMD_OPCODE_POS) |
                    (QSPI_ADR_NUMER_BYTE3 << QSPI_FCCR_ADR_BYTE_NUM_POS) |
                    (1 << QSPI_FCCR_ADR_EN_POS) |
                    (7 << QSPI_FCCR_W_BYTE_NUM_POS) |
                    (1 << QSPI_FCCR_W_DATA_EN_POS);
      
        WRITE_REG(hqspi->FCCR, reg_value);
        while (QSPI_STATUS_IS_STIG_EXE(hqspi));
        
        ms_qspiflash_comm_polling_wip(hqspi);
        ms_qspi_hal_flash_control_clear_command(hqspi);

        write_data += 8;

    }

    return STATUS_SUCCESS;

}


/**
 * @brief qspiflash write data  common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t adr     : write address .
 * @param  uint32_t sz      : write size .
 * @param  uint8_t* buf     : write data buf.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_comm_write(QSPI_Type *hqspi, uint32_t adr, uint32_t sz, uint8_t *buf)
{
    return STATUS_SUCCESS;
}


/**
 * @brief qspiflash reset  common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_comm_reset(QSPI_Type *hqspi)
{

    return STATUS_SUCCESS;
}

/**
 * @brief qspiflash read id  common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION uint32_t ms_qspiflash_comm_read_id(QSPI_Type *hqspi)
{
    uint32_t reg_value= 0;

    reg_value = (1 << QSPI_FCCR_EXC_CMD_POS) |
                (FLASH_CMD_READ_ID << QSPI_FCCR_CMD_OPCODE_POS) |
                (1 << QSPI_FCCR_R_DATA_EN_POS) |
                (2 << QSPI_FCCR_R_BYTE_NUM_POS);

    WRITE_REG(hqspi->FCCR, reg_value);
    while (QSPI_STATUS_IS_STIG_EXE(hqspi));

    return READ_REG(hqspi->FCRDR_L);
}


/**
 * @brief qspiflash detect chip size  common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t* size   : chip size.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_comm_detect_size(QSPI_Type *hqspi, uint32_t *size)
{
    uint32_t id = ms_qspiflash_comm_read_id(hqspi);
    *size = 0;

    if (((id & 0xFF0000) == 0x0) || ((id & 0xFF0000) == 0xFF0000))
    {
        return STATUS_ERROR;
    }

    *size = 1 << ((id>>16) & 0xFF);
    return STATUS_SUCCESS;
}


/**
 * @brief qspiflash get io mode   common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t* io_mode: flash io mode.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_comm_get_io_mode(QSPI_Type *hqspi,  uint32_t *io_mode)
{
    uint32_t chip_io_mode = ms_qspi_get_io_mode(hqspi);
    uint32_t data_trans_mode = (chip_io_mode&QSPI_DRIR_DATA_TRAN_TYPE_MASK)>>QSPI_DRIR_DATA_TRAN_TYPE_POS;
    uint32_t adr_trans_mode = (chip_io_mode&QSPI_DRIR_ADR_TRAN_TYPE_MASK)>>QSPI_DRIR_DATA_TRAN_TYPE_POS;

    if(data_trans_mode == QSPI_QIO_SPI_MODE && adr_trans_mode == QSPI_QIO_SPI_MODE)
    {
        *io_mode = FLASH_IO_MODE_QIO;
    }
    else if(data_trans_mode == QSPI_QIO_SPI_MODE)
    {
        *io_mode = FLASH_IO_MODE_QOUT;
    }
    else
    {
        *io_mode = FLASH_IO_MODE_NORMALRD;
    }

    return STATUS_SUCCESS;
}


/**
 * @brief qspiflash set io mode   common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  uint32_t io_mode : flash io mode.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_comm_set_io_mode(QSPI_Type *hqspi,  uint32_t io_mode)
{
    return STATUS_SUCCESS;
}

/**
 * @brief qspiflash write protecetd  common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @param  bool_t wp     : write protected  .
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_comm_write_protected(QSPI_Type *hqspi, bool wp)
{
    volatile uint32_t reg_value = 0;
    volatile uint32_t reg_value_l = 0;
    volatile uint32_t reg_value_h = 0;

    //read status s0 ~s7
    reg_value = (1 << QSPI_FCCR_EXC_CMD_POS) |
                (FLASH_CMD_READ_STATUS << QSPI_FCCR_CMD_OPCODE_POS) |
                (1 << QSPI_FCCR_R_DATA_EN_POS) |
                (0 << QSPI_FCCR_R_BYTE_NUM_POS);

    WRITE_REG(hqspi->FCCR, reg_value);
    while (QSPI_STATUS_IS_STIG_EXE(hqspi));
    reg_value_l = READ_REG(hqspi->FCRDR_L)&0xFF;


    // read status s8 ~s15
    reg_value = (1 << QSPI_FCCR_EXC_CMD_POS) |
                (FLASH_CMD_READ_STATUS_2 << QSPI_FCCR_CMD_OPCODE_POS) |
                (1 << QSPI_FCCR_R_DATA_EN_POS) |
                (0 << QSPI_FCCR_R_BYTE_NUM_POS);

    WRITE_REG(hqspi->FCCR, reg_value);
    while (QSPI_STATUS_IS_STIG_EXE(hqspi));
    reg_value_h = (READ_REG(hqspi->FCRDR_L)&0xFF)<<8;


    if(wp)
    {
        //CMP = 1, Protected area: 000000H ~ 077FFFH(Lower 480KB)
        reg_value_l &= ~(FLASH_STATUS_REG_BP1 | FLASH_STATUS_REG_BP3);
        reg_value_l |= (FLASH_STATUS_REG_BP2 | FLASH_STATUS_REG_BP4);
        reg_value_h |= (FLASH_STATUS_REG_CMP);
        WRITE_REG(hqspi->FCWDR_L,  (reg_value_l) | (reg_value_h));
    }
    else
    {
        //CMP = 1, Protected area: None
        reg_value_l &= ~(FLASH_STATUS_REG_BP4);
        reg_value_l |= (FLASH_STATUS_REG_BP2);
        reg_value_h |= (FLASH_STATUS_REG_CMP);
        WRITE_REG(hqspi->FCWDR_L,  (reg_value_l) | (reg_value_h));
    }


    ms_qspi_hal_flash_control_command(hqspi, FLASH_CMD_WRITE_ENABLE);

    //write status
    reg_value = (1 << QSPI_FCCR_EXC_CMD_POS) |
                (FLASH_CMD_WRITE_STATUS << QSPI_FCCR_CMD_OPCODE_POS) |
                (1 << QSPI_FCCR_W_DATA_EN_POS) |
                (1 << QSPI_FCCR_W_BYTE_NUM_POS);

    WRITE_REG(hqspi->FCCR, reg_value);
    while (QSPI_STATUS_IS_STIG_EXE(hqspi));

    ms_qspiflash_comm_polling_wip(hqspi);


    return STATUS_SUCCESS;
}


/**
 * @brief qspiflash read id  common api .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_comm_set_retention_data(QSPI_Type *hqspi, uint32_t addr, uint32_t *size)
{
    if(NULL == size)
    {
        return STATUS_ERROR;
    }
    uint32_t io_mode;
    ms_sleep_flash_cmd *p_sleep_flash_cmd = NULL;
    ms_sleep_flash_data *p_sleep_flash_data = (ms_sleep_flash_data *)addr;
    
	/*Get Flash wakeup command*/
    p_sleep_flash_data->header[0] = 'M';
    p_sleep_flash_data->header[1] = 'S';
    p_sleep_flash_data->header[2] = 'F';
    p_sleep_flash_data->header[3] = 'R';
    p_sleep_flash_data->magic = 0x55555555;
    p_sleep_flash_data->cmd_number = 0;
    p_sleep_flash_data->is_valid = 1;
    p_sleep_flash_data->p_cmd = (ms_sleep_flash_cmd *)(addr + sizeof(ms_sleep_flash_data));


    /*save flash wakeup cmd*/
    p_sleep_flash_cmd = p_sleep_flash_data->p_cmd;
    p_sleep_flash_cmd->reg_addr = (uint32_t)(&hqspi->FCCR); //0x40012090
    p_sleep_flash_cmd->reg_val  = (1 << QSPI_FCCR_EXC_CMD_POS) | (FLASH_CMD_REL_FROM_DPD << QSPI_FCCR_CMD_OPCODE_POS);
    p_sleep_flash_cmd->wait_cnt = 1000;
    p_sleep_flash_cmd->check_sum = p_sleep_flash_cmd->reg_addr +
          p_sleep_flash_cmd->reg_val + p_sleep_flash_cmd->wait_cnt;
    p_sleep_flash_data->cmd_number++;

    ms_qspiflash_comm_get_io_mode(hqspi, &io_mode);
    if(FLASH_IO_MODE_QOUT == io_mode)
    {
        /*save qspi mode bit*/
        p_sleep_flash_cmd += 1;
        p_sleep_flash_cmd->reg_addr = (uint32_t)(&hqspi->MBCR);//0x40012028
        p_sleep_flash_cmd->reg_val = 0;
        p_sleep_flash_cmd->wait_cnt = 10;
        p_sleep_flash_cmd->check_sum = p_sleep_flash_cmd->reg_addr +
              p_sleep_flash_cmd->reg_val + p_sleep_flash_cmd->wait_cnt;
        p_sleep_flash_data->cmd_number++;

        /*save qspi mode rdinstr cfg*/
        p_sleep_flash_cmd += 1;
        p_sleep_flash_cmd->reg_addr = (uint32_t)(&hqspi->DRIR); //0x40012004
        p_sleep_flash_cmd->reg_val  = (FLASH_CMD_QUAD_IO_FAST_READ << QSPI_DRIR_R_OPCODE_POS) |
                                      (0 << QSPI_DRIR_INSTR_TYPE_POS) |
                                      (2 << QSPI_DRIR_ADR_TRAN_TYPE_POS) |
                                      (2 << QSPI_DRIR_DATA_TRAN_TYPE_POS) |
                                      (1 << QSPI_DRIR_MODE_BIT_EN_POS) |
                                      (4 << QSPI_DRIR_DUMMY_CLOCK_NUM_POS);
        p_sleep_flash_cmd->wait_cnt = 10;
        p_sleep_flash_cmd->check_sum = p_sleep_flash_cmd->reg_addr +
              p_sleep_flash_cmd->reg_val + p_sleep_flash_cmd->wait_cnt;
        p_sleep_flash_data->cmd_number++;

        /*save qspi mode wrinstr cfg*/
        p_sleep_flash_cmd += 1;
        p_sleep_flash_cmd->reg_addr = (uint32_t)(&hqspi->DWIR);//0x40012008
        p_sleep_flash_cmd->reg_val =  (FLASH_CMD_QUAD_PAGE_PROGRAM << QSPI_DWIR_W_OPCODE_POS) |
                                      (0 << QSPI_DRIR_ADR_TRAN_TYPE_POS) |
                                      (2 << QSPI_DRIR_DATA_TRAN_TYPE_POS);
        p_sleep_flash_cmd->wait_cnt = 10;
        p_sleep_flash_cmd->check_sum = p_sleep_flash_cmd->reg_addr +
              p_sleep_flash_cmd->reg_val + p_sleep_flash_cmd->wait_cnt;
        p_sleep_flash_data->cmd_number++;
    }
    *size = sizeof(ms_sleep_flash_data) + p_sleep_flash_data->cmd_number*sizeof(ms_sleep_flash_cmd);

    return STATUS_SUCCESS;
}


/**
 * @brief qspiflash enter deep power down mode .
 * @param  QSPI_Type *hqspi : QSPI Controller.
 *         uint32_t   addr  : Flash Retention Address.
 *         uint32_t   size  : Flash Retention Size.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
RAM_FUNCTION int32_t ms_qspiflash_comm_enter_deep_power_down(QSPI_Type *hqspi, uint32_t addr, uint32_t *size)
{
    int32_t status = STATUS_SUCCESS;

    status = ms_qspiflash_comm_set_retention_data(hqspi, addr, size);
    if(STATUS_SUCCESS != status)
    {
        return STATUS_ERROR;
    }

    ms_qspi_hal_flash_control_command(hqspi, FLASH_CMD_DEEP_POWER_DOWN);

    return STATUS_SUCCESS;
}
