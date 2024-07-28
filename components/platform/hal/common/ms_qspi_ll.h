/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_qspi_ll.h
 * @brief Header file of qspi  module.
 * @author che.jiang
 * @date   2021-12-21
 * @version 1.0
 * @Revision:
 */

#ifndef MS_LL_QSPI_H_
#define MS_LL_QSPI_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>


/** @defgroup QSPI RW Type
  * @{
  */
#define QSPI_READ_TYPE                 			0x00000000U
#define QSPI_WRITE_TYPE                 		0x00000001U

/** @defgroup QSPI Mode Type
  * @{
  */
#define QSPI_QIO_SPI_MODE                  		0x00000002U   /*!< Ins,Adr,Data send on  DQ0~DQ3  */
#define QSPI_DIO_SPI_MODE                  		0x00000001U   /*!< Ins,Adr,Data send on  DQ0&DQ1  */
#define QSPI_STANDARD_SPI_MODE                  0x00000000U   /*!< Ins,Adr,Data send on  DQ0  */


/** @defgroup QSPI Sample Edge Selection Type
  * @{
  */
#define QSPI_RD_FALLING_SAMPLE                  0x00000000U
#define QSPI_RD_RISING_SAMPLE                  	0x00000001U

/** @defgroup QSPI Master mode baud rate divisor(2 to 32)
  * @{
  */
#define QSPI_CLK_DIV2                  			0x00000000U
#define QSPI_CLK_DIV4                  	        0x00000001U
#define QSPI_CLK_DIV6                  	        0x00000002U
#define QSPI_CLK_DIV8                  	        0x00000003U
#define QSPI_CLK_DIV10                  	    0x00000004U
#define QSPI_CLK_DIV12                  	    0x00000005U
#define QSPI_CLK_DIV14                  	    0x00000006U
#define QSPI_CLK_DIV16                  	    0x00000007U
#define QSPI_CLK_DIV18                  	    0x00000008U
#define QSPI_CLK_DIV20                  	    0x00000009U
#define QSPI_CLK_DIV22                  	    0x0000000AU
#define QSPI_CLK_DIV24                  	    0x0000000BU
#define QSPI_CLK_DIV26                  	    0x0000000CU
#define QSPI_CLK_DIV28                  	    0x0000000DU
#define QSPI_CLK_DIV30                  	    0x0000000EU
#define QSPI_CLK_DIV32                  	    0x0000000FU

/** @defgroup QSPI Dma number bytes 2*
  * @{
  */
#define QSPI_DMA_BYTE1                  		0x00000000U
#define QSPI_DMA_BYTE2                  	    0x00000001U
#define QSPI_DMA_BYTE4                  	    0x00000002U
#define QSPI_DMA_BYTE8                  	    0x00000003U
#define QSPI_DMA_BYTE16                  	    0x00000004U
#define QSPI_DMA_BYTE32                  	    0x00000005U
#define QSPI_DMA_BYTE64                  	    0x00000006U
#define QSPI_DMA_BYTE128                  	    0x00000007U
#define QSPI_DMA_BYTE256                  	    0x00000008U
#define QSPI_DMA_BYTE512                  	    0x00000009U
#define QSPI_DMA_BYTE1K                  	    0x0000000AU
#define QSPI_DMA_BYTE2K                  	    0x0000000BU
#define QSPI_DMA_BYTE4K                  	    0x0000000CU
#define QSPI_DMA_BYTE8K                  	    0x0000000DU
#define QSPI_DMA_BYTE16K                  	    0x0000000EU
#define QSPI_DMA_BYTE32K                  	    0x0000000FU


/** @defgroup QSPI Indirect Range Width
  * @{
  */
#define QSPI_IND_RANGE_WIDTH_16                 0x00000004U
#define QSPI_IND_RANGE_WIDTH_64                 0x00000008U


/** @defgroup QSPI Address number bytes 2*
  * @{
  */
#define QSPI_ADR_NUMER_BYTE1                    0x00000000U
#define QSPI_ADR_NUMER_BYTE2                    0x00000001U
#define QSPI_ADR_NUMER_BYTE3                    0x00000002U
#define QSPI_ADR_NUMER_BYTE4                    0x00000003U

/** @defgroup QSPI indirect read and write reset value
  * @{
  */
#define QSPI_DEV_RDINSTR_RESET_VALUE            0x00000003U
#define QSPI_DEV_WRINSTR_RESET_VALUE            0x00000002U




/** @defgroup Qspi command exe Staus
  *
  * @{
  */
#define QSPI_STATUS_IS_STIG_EXE(instance)      (READ_BIT(instance->FCCR,QSPI_FCCR_STIG_EXC_P) == QSPI_FCCR_STIG_EXC_P)
/**
  * @}
  */

/**
 * @brief  QSPI Flash Command Control
 * @retval None
 */
__STATIC_FORCEINLINE void ms_qspi_ll_flash_control_command(QSPI_Type *hqspi, uint32_t command)
{
    uint32_t reg_value = 0;
    reg_value = (1 << QSPI_FCCR_EXC_CMD_POS) | (command << QSPI_FCCR_CMD_OPCODE_POS);
    WRITE_REG(hqspi->FCCR,reg_value);
    while(QSPI_STATUS_IS_STIG_EXE(hqspi));
}

/**
 * @brief  QSPI Data Transfer type for standard IO mode
 * @retval None
 */
__STATIC_FORCEINLINE void ms_qspi_ll_flash_control_clear_command(QSPI_Type *hqspi)
{
    CLEAR_BIT(hqspi->FCCR, QSPI_FCCR_CMD_OPCODE_MASK);
}

/**
 * @brief  QSPI Flash config sample delay
 * @retval None
 */
__STATIC_FORCEINLINE  void ms_qspi_sample_delay(QSPI_Type *hqspi, uint32_t delay)
{
    uint32_t reg_value = READ_REG(hqspi->RDCR);
    reg_value &= ~QSPI_RDCR_DELAY_NUMBER;
    reg_value |= (delay << QSPI_RDCR_DELAY_NUMBER_POS);
    WRITE_REG(hqspi->RDCR, reg_value);
}

/**
 * @brief  QSPI Flash master baud divisor
 * @retval None
 */
__STATIC_FORCEINLINE void ms_qspi_ll_config_master_baud_div(QSPI_Type *hqspi, uint32_t div_type)
{
    MODIFY_REG(hqspi->CFG, QSPI_CFG_BAUD_RATE_CFG, (div_type<<QSPI_CFG_BAUD_RATE_CFG_POS));
}

/**
 * @brief  QSPI config  indirect mode address
 * @retval None
 */
__STATIC_FORCEINLINE void ms_qspi_set_ind_trig_adr(QSPI_Type *hqspi, uint32_t trig_adr, uint32_t trig_range)
{
    WRITE_REG(hqspi->IAATR, trig_adr);
    WRITE_REG(hqspi->ITARR, trig_range);      // 2^tri_range (byte)
}

/**
 * @brief  QSPI config write indirect mode
 * @retval None
 */
__STATIC_FORCEINLINE  void ms_qspi_cfg_ind_wr(QSPI_Type *hqspi, uint32_t flash_addr, uint32_t wr_byte)
{
    WRITE_REG(hqspi->IWTSAR, flash_addr);
    WRITE_REG(hqspi->IWTNBR, wr_byte);
    WRITE_REG(hqspi->IWTCR, QSPI_IWTCR_IND_WRITE_START);
}

/**
 * @brief  QSPI config read indirect mode
 * @retval None
 */
__STATIC_FORCEINLINE void ms_qspi_cfg_ind_rd(QSPI_Type *hqspi, uint32_t flash_addr, uint32_t rd_byte)
{
    WRITE_REG(hqspi->IRTSAR, flash_addr);
    WRITE_REG(hqspi->IRTNBR, rd_byte);
    WRITE_REG(hqspi->IRTCR, QSPI_IRTCR_IND_READ_START);
}



/**
 * @brief  QSPI Data Transfer type for standard IO mode
 * @retval None
 */
__STATIC_FORCEINLINE uint32_t ms_qspi_get_io_mode(QSPI_Type *hqspi)
{
    return READ_REG(hqspi->DRIR);
}



#endif /* MS_LL_QSPI_H_ */
