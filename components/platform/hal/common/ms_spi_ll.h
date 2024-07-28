/**
 * Copyright ? 2022 by MooreSilicon.All rights reserved
 * @file   ms_spi_ll.h
 * @brief Header file of spi master  module.
 * @author haijun.mai
 * @date   2022-01-10
 * @version 1.0
 * @Revision
 */

#ifndef MS_SPI_LL_H_
#define MS_SPI_LL_H_

#include "ms1008.h"
#include "ms_spi_regs.h"
#include <stdint.h>
#include <stdbool.h>


#define FRAME_04BITS                                 (0x3)
#define FRAME_05BITS                                 (0x4)
#define FRAME_06BITS                                 (0x5)
#define FRAME_07BITS                                 (0x6)
#define FRAME_08BITS                                 (0x7)
#define FRAME_09BITS                                 (0x8)
#define FRAME_10BITS                                 (0x9)
#define FRAME_11BITS                                 (0xa)
#define FRAME_12BITS                                 (0xb)
#define FRAME_13BITS                                 (0xc)
#define FRAME_14BITS                                 (0xd)
#define FRAME_15BITS                                 (0xe)
#define FRAME_16BITS                                 (0xf)
#define FRAME_17BITS                                 (0x10)
#define FRAME_18BITS                                 (0x11)
#define FRAME_19BITS                                 (0x12)
#define FRAME_20BITS                                 (0x13)
#define FRAME_21BITS                                 (0x14)
#define FRAME_22BITS                                 (0x15)
#define FRAME_23BITS                                 (0x16)
#define FRAME_24BITS                                 (0x17)
#define FRAME_25BITS                                 (0x18)
#define FRAME_26BITS                                 (0x19)
#define FRAME_27BITS                                 (0x1a)
#define FRAME_28BITS                                 (0x1b)
#define FRAME_29BITS                                 (0x1c)
#define FRAME_30BITS                                 (0x1d)
#define FRAME_31BITS                                 (0x1e)
#define FRAME_32BITS                                 (0x1f)

#define MOTOROLA_SPI                                  (0x0)/*Motorolla SPI Frame Format*/
#define TEXAS_SSP                                         (0x1)/*Texas Instruments SSP FrameFormat*/
#define NS_MICROWIRE                                  (0x2)/*National Microwire FrameFormat*/

#define SCPH_MIDDLE                                      (0)/*Serial clock toggles in middle offirst data bit*/
#define SCPH_START                                        (1)/* Serial clock toggles at start of firstdata bit*/

#define STD_SPI_FRF                                      (0x0)/*Standard SPI Frame Format*/
#define DUAL_SPI_FRF                                    (0x1)/*Dual SPI Frame Format*/
#define QUAD_SPI_FRF                                   (0x2)/*Quad SPI Frame Format*/
#define OCTAL_SPI_FRF                                  (0x3)/*Octal SPI Frame Format*/

#define MS_SPI_MASTER                                 (0x1)
#define MS_SPI_SLAVE                                   (0x0)

#define MS_SPI_SLAVE_SLECT_GOGGLE_DISABLE            (0)
#define MS_SPI_SLAVE_SLECT_GOGGLE_ENABLE              (1)

#define TRANSMIT_AND_RECEIVE                                         (0)
#define TRANSMIT_ONLY                                                       (1)
#define RECEIVE_ONLY                                                          (2)

#define SCPOL_SCLK_LOW                  (0)
#define SCPOL_SCLK_HIGH                 (1)

#define SCPH_SCPH_MIDDLE              (0)
#define SCPH_SCPH_START                (1)



/**
 * @brief set spi as master or slave 
 * @param  Spi_Type *spi: 
 * @param  uint8_t role_mode
 * @param   This parameter can be one of the following values:
 * @MS_SPI_MASTER
 * @MS_SPI_SLAVE
 * @retval none
 */
 static inline void ms_spi_set_role_mode_ll(Spi_Type *spi,uint8_t role_mode)
{ 
       CLEAR_BIT(spi->CTRLR0, SPI_CTRLR0_SLVOE); 
	 SET_BIT(spi->CTRLR0, ((role_mode<<SPI_CTRLR0_SLVOE_POS)&SPI_CTRLR0_SLVOE_MASK));
	  
}




/**
 * @brief spi  set spi data frame size
 * @param  Spi_Type *spi: 
 * @uint8_t frame_size
 * @param   This parameter can be one of the following values:
 *@FRAME_04BITS 
 *@FRAME_05BITS 
 *@FRAME_06BITS 
 *@FRAME_07BITS 
 *@FRAME_08BITS 
 *@FRAME_09BITS 
 *@FRAME_10BITS 
 *@FRAME_11BITS 
 *@FRAME_12BITS 
 *@FRAME_13BITS 
 *@FRAME_14BITS 
 *@FRAME_15BITS 
 *@FRAME_16BITS 
 *@FRAME_17BITS 
 *@FRAME_18BITS 
 *@FRAME_19BITS 
 *@FRAME_20BITS 
 *@FRAME_21BITS 
 *@FRAME_22BITS 
 *@FRAME_23BITS 
 *@FRAME_24BITS 
 *@FRAME_25BITS 
 *@FRAME_26BITS 
 *@FRAME_27BITS 
 *@FRAME_28BITS 
 *@FRAME_29BITS 
 *@FRAME_30BITS 
 *@FRAME_31BITS 
 *@FRAME_32BITS 
 * @retval none
 */
 static inline void ms_spi_set_data_farme_size_ll(Spi_Type *spi,uint8_t frame_size)
{
	uint32_t value  = 0;

	value = READ_REG(spi->CTRLR0);
	
       CLEAR_BIT(value, SPI_CTRLR0_DFS32);
	   
       SET_BIT(value, ((frame_size<<SPI_CTRLR0_DFS32_POS)&SPI_CTRLR0_DFS32_MASK));

	 WRITE_REG(spi->CTRLR0, value);  
}


/**
 * @brief set spi protocol frame format,MOTOROLA,TEXAS,or MICROWIRE
 * @param  Spi_Type *spi: 
 * @uint8_t frame_format
 * @param   This parameter can be one of the following values:
 * @ MOTOROLA_SPI
 * @ TEXAS_SSP
 * @ NS_MICROWIRE
 * @retval none
 */
 static inline void ms_spi_set_protocol_frame_format_ll(Spi_Type *spi,uint8_t frame_format)
{
     uint32_t value  = 0;

	value = READ_REG(spi->CTRLR0);
	
       CLEAR_BIT(value, SPI_CTRLR0_FRF);
	   
       SET_BIT(value, ((frame_format<<SPI_CTRLR0_FRF_POS)&SPI_CTRLR0_FRF_MASK));

	 WRITE_REG(spi->CTRLR0, value);  
}


/**
 * @brief spi  enable
 * @param  Spi_Type *spi: 
 * @retval none
 */static inline void ms_spi_enable_ll(Spi_Type *spi)
{
     SET_BIT(spi->SSIENR, SPI_SSIENR_SSIEN);
}

/**
 * @brief spi  enable
 * @param  Spi_Type *spi: 
 * @retval none
 */static inline void ms_spi_disable_ll(Spi_Type *spi)
{
     CLEAR_BIT(spi->SSIENR, SPI_SSIENR_SSIEN);
}

/**
 * @brief set spi serial clock  phase
 * @param  Spi_Type *spi: 
 * @param  uint8_t scph: 
 * @param   This parameter can be one of the following values:
 * @ SCPOL_SCLK_LOW
 * @ SCPOL_SCLK_HIGH
 * @retval none
 */static inline void ms_spi_set_serial_clock_phase_ll(Spi_Type *spi,uint8_t scph)
{
     uint32_t value  = 0;

	value = READ_REG(spi->CTRLR0);
	
       CLEAR_BIT(value, SPI_CTRLR0_SCPH);
	   
       SET_BIT(value, ((scph<<SPI_CTRLR0_SCPH_POS)&SPI_CTRLR0_SCPH_MASK));

	 WRITE_REG(spi->CTRLR0, value);  
}



/**
 * @brief set spi serial clock  polarity
 * @param  Spi_Type *spi: 
 * @param  uint8_t scph: 
 * @param   This parameter can be one of the following values:
 * @ SCPH_SCPH_MIDDLE
 * @ SCPH_SCPH_START
 * @retval none
 */static inline void ms_spi_set_serial_clock_polarity_ll(Spi_Type *spi,uint8_t scpol)
{
     uint32_t value  = 0;

	value = READ_REG(spi->CTRLR0);
	
       CLEAR_BIT(value, SPI_CTRLR0_SCPOL);
	   
       SET_BIT(value, ((scpol<<SPI_CTRLR0_SCPOL_POS)&SPI_CTRLR0_SCPOL_MASK));

	 WRITE_REG(spi->CTRLR0, value);  
}


/**
 * @brief set spi serial transfer_mode
 * @param  Spi_Type *spi: 
 * @param  uint8_t transfer_mode: 
 * @param   This parameter can be one of the following values:
 * @ TRANSMIT_AND_RECEIVE
 * @ TRANSMIT_ONLY
 * @ RECEIVE_ONLY
 * @retval none
 */
 static inline void ms_spi_set_transfer_mode_ll(Spi_Type *spi,uint8_t transfer_mode)
{
     uint32_t value  = 0;

	value = READ_REG(spi->CTRLR0);
	
       CLEAR_BIT(value, SPI_CTRLR0_TMOD);
	   
       SET_BIT(value, ((transfer_mode<<SPI_CTRLR0_TMOD_POS)&SPI_CTRLR0_TMOD_MASK));

	 WRITE_REG(spi->CTRLR0, value);  
}





/**
 * @brief set  spi frame format
 * @param  Spi_Type *spi: 
 * @uint8_t spi_frame_format
 * @param   This parameter can be one of the following values:
 * @ STD_SPI_FRF
 * @ DUAL_SPI_FRF
 * @ QUAD_SPI_FRF
 * @  OCTAL_SPI_FRF
 * @retval none
 */static inline void ms_spi_set_frame_format_ll(Spi_Type *spi,uint8_t spi_frame_format)
{
     uint32_t value  = 0;

	value = READ_REG(spi->CTRLR0);
	
       CLEAR_BIT(value, SPI_CTRLR0_SPIFRF);
	   
       SET_BIT(value, ((spi_frame_format<<SPI_CTRLR0_SPIFRF_POS)&SPI_CTRLR0_SPIFRF_MASK));

	 WRITE_REG(spi->CTRLR0, value);  
}



/**
 * @brief spi slave select toggle enable
 * @param  Spi_Type *spi: 
 * @param   uint8_t sste
 * @param   This parameter can be one of the following values:
 * @MS_SPI_SLAVE_SLECT_GOGGLE_DISABLE
 * @MS_SPI_SLAVE_SLECT_GOGGLE_ENABLE     
 * @retval none
 */
 static inline void ms_spi_slave_select_toggle_enable_ll(Spi_Type *spi,uint8_t sste)
{ 
	CLEAR_BIT(spi->CTRLR0, SPI_CTRLR0_SSTE);
       SET_BIT(spi->CTRLR0,  ((sste<<SPI_CTRLR0_SSTE_POS)&SPI_CTRLR0_SSTE_MASK)); 
}




/**
 * @brief set  spi frames number
 * @param  Spi_Type *spi: 
 * @retval none
 */static inline void ms_spi_set_data_frames_number_ll(Spi_Type *spi,uint8_t data_frames_number)
{
     uint32_t value  = 0;

	value = READ_REG(spi->CTRLR1);
	
       CLEAR_BIT(value, SPI_CTRLR1_NDF);
	   
       SET_BIT(value, ((data_frames_number<<SPI_CTRLR1_NDF_POS)&SPI_CTRLR1_NDF_MASK));

	 WRITE_REG(spi->CTRLR1, value);  
}



/**
 * @brief spi slave select toggle enable
 * @param  Spi_Type *spi: 
 * @retval none
 */static inline void ms_spi_slave_enable_ll(Spi_Type *spi)
{ 
       SET_BIT(spi->SER, SPI_SER_SER); 
}

/**
 * @brief spi slave select toggle disable
 * @param  Spi_Type *spi: 
 * @retval none
 */static inline void ms_spi_slave_disable_ll(Spi_Type *spi)
{ 
       CLEAR_BIT(spi->SER, SPI_SER_SER); 
}


/**
 * @brief set  spi frames number
 * @param  Spi_Type *spi: 
 * @retval none
 */static inline void ms_spi_set_sck_div_ll(Spi_Type *spi,uint8_t sck_div)
{
     uint32_t value  = 0;

	value = READ_REG(spi->BAUDR);
	
       CLEAR_BIT(value, SPI_BAUDR_SCKDV);
	   
       SET_BIT(value, ((sck_div<<SPI_BAUDR_SCKDV_POS)&SPI_BAUDR_SCKDV_MASK));

	 WRITE_REG(spi->BAUDR, value);  
}



/**
 * @brief set  spi transmit fifo threshold level
 * @param  Spi_Type *spi: 
 * @param  uint8_t tx_threshold_level: 
 * @retval none
 */static inline void ms_spi_set_transmit_fifo_threshold_level_ll(Spi_Type *spi,uint8_t tx_threshold_level)
{
     uint32_t value  = 0;

	value = READ_REG(spi->TXFTLR);
	
       CLEAR_BIT(value, SPI_TXFTLR_TFT);
	   
       SET_BIT(value, ((tx_threshold_level<<SPI_TXFTLR_TFT_POS)&SPI_TXFTLR_TFT_MASK));

	 WRITE_REG(spi->TXFTLR, value);  
}

/**
 * @brief set  spi transmit fifo threshold level
 * @param  Spi_Type *spi: 
 * @param  uint8_t tx_threshold_level: 
 * @retval none
 */static inline void ms_spi_set_receive_fifo_threshold_level_ll(Spi_Type *spi,uint8_t rx_threshold_level)
{
     uint32_t value  = 0;

	value = READ_REG(spi->RXFTLR);
	
       CLEAR_BIT(value, SPI_RXFTLR_RFT);
	   
       SET_BIT(value, ((rx_threshold_level<<SPI_RXFTLR_RFT_POS)&SPI_RXFTLR_RFT_MASK));

	 WRITE_REG(spi->RXFTLR, value);  
}

/**
 * @brief get  spi transmit fifo threshold level
 * @param  Spi_Type *spi: 
 * @param  
 * @retval none
 */static inline uint8_t ms_spi_get_transmit_fifo_threshold_level_ll(Spi_Type *spi )
{
      return (READ_REG(spi->TXFLR) & SPI_TXFLR_TXTFL);
}

/**
 * @brief get  spi receive fifo threshold level
 * @param  Spi_Type *spi: 
 * @param 
 * @retval none
 */static inline uint8_t ms_spi_get_receive_fifo_threshold_level_ll(Spi_Type *spi )
{
      return (READ_REG(spi->RXFLR) & SPI_RXFLR_RXTFL);
}


/**
 * @brief get  spi status
 * @param  Spi_Type *spi: 
 * @param  uint8_t status_mask
 * This parameter can be a combination of the following values:
 * SPI_SR_BUSY
 * SPI_SR_TFNF
 * SPI_SR_TFE
 * SPI_SR_RFNE
 * SPI_SR_RFF
 * SPI_SR_TXE
 * SPI_SR_DCOL
 * @retval  0 or 1
 */static inline uint8_t ms_spi_get_status_ll(Spi_Type *spi,uint8_t status_mask )
{
      return (READ_REG(spi->SR) & status_mask);
}

/**
 * @brief set spi interrupt mask
 * @param  Spi_Type *spi: 
 * @param  uint8_t interrupt_mask: 
 * This parameter can be a combination of the following values:
 * SPI_IMR_TXEIM
 * SPI_IMR_TXOIM
 * SPI_IMR_RXUIM
 * SPI_IMR_RXOIM
 * SPI_IMR_RXFIM
 * SPI_IMR_MSTIM
 * @retval none
 */static inline void ms_spi_set_interrupt_mask_ll(Spi_Type *spi,uint8_t interrupt_mask)
{
	  CLEAR_BIT(spi->IMR, interrupt_mask);  
}

/**
 * @brief clear spi interrupt mask
 * @param  Spi_Type *spi: 
 * @param  uint8_t interrupt_mask: 
 * This parameter can be a combination of the following values:
 * SPI_IMR_TXEIM
 * SPI_IMR_TXOIM
 * SPI_IMR_RXUIM
 * SPI_IMR_RXOIM
 * SPI_IMR_RXFIM
 * SPI_IMR_MSTIM
 * @retval none
 */static inline void ms_spi_clear_interrupt_mask_ll(Spi_Type *spi,uint8_t interrupt_mask)
{
	  SET_BIT(spi->IMR, interrupt_mask);  
}

/**
 * @brief get  spi interrupt status
 * @param  Spi_Type *spi: 
 * @retval can be a combination of the following values:
 * SPI_ISR_TXEIS
 * SPI_ISR_TXOIS
 * SPI_ISR_RXUIS
 * SPI_ISR_RXOIS
 * SPI_ISR_RXFIS
 * SPI_ISR_MSTIS
 */
 static inline uint8_t ms_spi_get_interrupt_status_ll(Spi_Type *spi )
{
      return (READ_REG(spi->ISR));
}


/**
 * @brief get  spi raw interrupt status
 * @param  Spi_Type *spi: 
 * @param  uint8_t status_mask
 * This parameter can be a combination of the following values:
 * SPI_RISR_TXEIS
 * SPI_RISR_TXOIS
 * SPI_RISR_RXUIS
 * SPI_RISR_RXOIS
 * SPI_RISR_RXFIS
 * SPI_RISR_MSTIS
 * @retval  0 or 1
 */
 static inline uint8_t ms_spi_get_raw_interrupt_status_ll(Spi_Type *spi,uint8_t raw_interrput_status_mask )
{
      return (READ_REG(spi->RISR) & raw_interrput_status_mask);
}

/**
 * @brief clear spi transmit fifo overflow interrupt status
 * @param  Spi_Type *spi: 
 * @retval  0 or 1
 */
static inline uint8_t ms_spi_clear_transmit_fifo_overflow_interrupt_status_ll(Spi_Type *spi)
{
      return READ_REG(spi->TXOICR);
}

/**
 * @brief clear spi receive fifo overflow interrupt status
 * @param  Spi_Type *spi: 
 * @retval  0 or 1
 */
static inline uint8_t ms_spi_clear_receive_fifo_overflow_interrupt_status_ll(Spi_Type *spi)
{
      return READ_REG(spi->RXOICR);
}

/**
 * @brief clear spi receive fifo underflow interrupt status
 * @param  Spi_Type *spi: 
 * @retval  0 or 1
 */
static inline uint8_t ms_spi_clear_receive_fifo_underflow_interrupt_status_ll(Spi_Type *spi)
{
      return READ_REG(spi->RXUICR);
}

/**
 * @brief clear spi multi master interrupt status
 * @param  Spi_Type *spi: 
 * @retval  0 or 1
 */
static inline uint8_t ms_spi_clear_multi_master_interrupt_status_ll(Spi_Type *spi)
{
      return READ_REG(spi->MSTICR);
}

/**
 * @brief clear spi  interrupt status
 * @param  Spi_Type *spi: 
 * @retval  0 or 1
 */
static inline uint8_t ms_spi_clear_interrupt_status_ll(Spi_Type *spi)
{
      return READ_REG(spi->ICR);
}


/**
 * @brief spi read data from fifo
 * @param  Spi_Type *spi: 
 * @retval  0 or 1
 */
static inline uint32_t ms_spi_read_data_ll(Spi_Type *spi)
{
      return READ_REG(spi->DRX[0]);
}


/**
 * @brief spi write data to fifo
 * @param  Spi_Type *spi: 
 * @retval  none
 */
static inline  void ms_spi_write_data_ll(Spi_Type *spi,uint32_t write_data)
{
       WRITE_REG(spi->DRX[0],write_data);
}



/**
 * @brief set  spi frames number
 * @param  Spi_Type *spi: 
 * @uint8_t sample_delay_value: can be 0~255
 * @retval none
 */static inline void ms_spi_set_sample_delay_value_ll(Spi_Type *spi,uint8_t sample_delay_value)
{
     uint32_t value  = 0;

	value = READ_REG(spi->RXSAMPLEDLY);
	
       CLEAR_BIT(value, SPI_RXSAMPLEDLY_RSD);
	   
       SET_BIT(value, ((sample_delay_value<<SPI_RXSAMPLEDLY_RSD_POS)&SPI_RXSAMPLEDLY_RSD_MASK));

	 WRITE_REG(spi->RXSAMPLEDLY, value);  
}

#endif
