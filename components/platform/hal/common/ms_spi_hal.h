/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_spi_hal.h
 * @brief head   file of spi  module.
 * @author haijun.mai
 * @date   2022-01-12
 * @version 1.0
 * @Revision
 */
#ifndef MS_SPI_HAL_H_
#define MS_SPI_HAL_H_

#include "ms_spi_ll.h"



/**
 * @brief set spi as master or slave 
 * @param  Spi_Type *spi: 
 * @param  uint8_t role_mode
 * @param   This parameter can be one of the following values:
 * @MS_SPI_MASTER
 * @MS_SPI_SLAVE
 * @retval none
 */
#define  ms_spi_set_role_mode_hal(spi, role_mode)    ms_spi_set_role_mode_ll(spi, role_mode)


/**
 * @brief   set spi data frame size
 * @param  Spi_Type *spi: 
 * @retval none
 */
 #define  ms_spi_set_data_farme_size_hal(spi, frame_size)   ms_spi_set_data_farme_size_ll(spi, frame_size)


 
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
#define ms_spi_set_protocol_frame_format_hal(spi, frame_format)    ms_spi_set_protocol_frame_format_ll(spi, frame_format)


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
 */
 #define  ms_spi_set_frame_format_hal(spi, frame_format)   ms_spi_set_frame_format_ll(spi, frame_format)


/**
 * @brief spi  enable
 * @param  Spi_Type *spi: 
 * @retval none
 */
 #define  ms_spi_enable_hal(spi)  ms_spi_enable_ll(spi)

/**
 * @brief spi  enable
 * @param  Spi_Type *spi: 
 * @retval none
 */
#define ms_spi_disable_hal(spi)  ms_spi_disable_ll(spi) 

/**
 * @brief set spi serial clock  phase
 * @param  Spi_Type *spi: 
 * @param  uint8_t scph: 
 * @retval none
 */
#define  ms_spi_set_serial_clock_phase_hal(spi, scph)   ms_spi_set_serial_clock_phase_ll(spi, scph)


/**
 * @brief set spi serial clock  polarity
 * @param  Spi_Type *spi: 
 * @param  uint8_t scph: 
 * @retval none
 */
#define  ms_spi_set_serial_clock_polarity_hal(spi, scpol)  ms_spi_set_serial_clock_polarity_ll(spi, scpol)



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
#define ms_spi_set_transfer_mode_hal(spi, transfer_mode)   ms_spi_set_transfer_mode_ll(spi, transfer_mode)




/**
 * @brief spi slave select toggle enable
 * @param  Spi_Type *spi: 
 * @param   uint8_t sste
 * @param   This parameter can be one of the following values:
 * @MS_SPI_SLAVE_SLECT_GOGGLE_DISABLE
 * @MS_SPI_SLAVE_SLECT_GOGGLE_ENABLE     
 * @retval none
 */
 #define ms_spi_slave_select_toggle_enable_hal(spi, sste)     ms_spi_slave_select_toggle_enable_ll(spi, sste)


/**
 * @brief set  spi data frames number
 * @param  Spi_Type *spi: 
 * @retval none
 */
 #define  ms_spi_set_data_frames_number_hal(spi, data_frames_number)    ms_spi_set_data_frames_number_ll(spi, data_frames_number)


 /**
 * @brief spi slave select toggle enable
 * @param  Spi_Type *spi: 
 * @retval none
 */
 #define  ms_spi_slave_enable_hal(spi)     ms_spi_slave_enable_ll(spi)

 /**
 * @brief spi slave select toggle disable
 * @param  Spi_Type *spi: 
 * @retval none
 */
  #define    ms_spi_slave_disable_hal(spi)      ms_spi_slave_disable_ll(spi)


  /**
 * @brief set  spi frames number
 * @param  Spi_Type *spi: 
 * @retval none
 */
#define  ms_spi_set_sck_div_hal(spi, sck_div)    ms_spi_set_sck_div_ll(spi, sck_div)


/**
 * @brief set  spi transmit fifo threshold level
 * @param  Spi_Type *spi: 
 * @param  uint8_t tx_threshold_level: 
 * @retval none
 */
#define  ms_spi_set_transmit_fifo_threshold_level_hal(spi, tx_threshold_level)    ms_spi_set_transmit_fifo_threshold_level_ll(spi, tx_threshold_level)


/**
 * @brief set  spi transmit fifo threshold level
 * @param  Spi_Type *spi: 
 * @param  uint8_t tx_threshold_level: 
 * @retval none
 */
 #define ms_spi_set_receive_fifo_threshold_level_hal(spi, rx_threshold_level)    ms_spi_set_receive_fifo_threshold_level_ll(spi, rx_threshold_level)


 /**
 * @brief get  spi transmit fifo threshold level
 * @param  Spi_Type *spi: 
 * @param  
 * @retval none
 */
 #define  ms_spi_get_transmit_fifo_threshold_level_hal(spi )     ms_spi_get_transmit_fifo_threshold_level_ll(spi )

 /**
 * @brief get  spi receive fifo threshold level
 * @param  Spi_Type *spi: 
 * @param 
 * @retval none
 */
 #define  ms_spi_get_receive_fifo_threshold_level_hal(spi )   ms_spi_get_receive_fifo_threshold_level_ll(spi )


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
 */
#define  ms_spi_get_status_hal(spi, status_mask )    ms_spi_get_status_ll(spi, status_mask )

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
 */
 #define ms_spi_set_interrupt_mask_hal(spi, interrupt_mask)    ms_spi_set_interrupt_mask_ll(spi, interrupt_mask)

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
 */
 #define ms_spi_clear_interrupt_mask_hal(spi, interrupt_mask)    ms_spi_clear_interrupt_mask_ll(spi, interrupt_mask)

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
 #define  ms_spi_get_interrupt_status_hal(spi)    ms_spi_get_interrupt_status_ll(spi)



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
 #define ms_spi_get_raw_interrupt_status_hal(spi, raw_interrput_status_mask )    ms_spi_get_raw_interrupt_status_ll(spi, raw_interrput_status_mask )
 
 /**
 * @brief clear spi transmit fifo overflow interrupt status
 * @param  Spi_Type *spi: 
 * @retval  0 or 1
 */
#define ms_spi_clear_transmit_fifo_overflow_interrupt_status_hal(spi)   ms_spi_clear_transmit_fifo_overflow_interrupt_status_ll(spi)


/**
 * @brief clear spi receive fifo overflow interrupt status
 * @param  Spi_Type *spi: 
 * @retval  0 or 1
 */
#define ms_spi_clear_receive_fifo_overflow_interrupt_status_hal(spi)     ms_spi_clear_receive_fifo_overflow_interrupt_status_ll(spi)


/**
 * @brief clear spi receive fifo underflow interrupt status
 * @param  Spi_Type *spi: 
 * @retval  0 or 1
 */
#define ms_spi_clear_receive_fifo_underflow_interrupt_status_hal(spi)      ms_spi_clear_receive_fifo_underflow_interrupt_status_ll(spi)

/**
 * @brief clear spi multi master interrupt status
 * @param  Spi_Type *spi: 
 * @retval  0 or 1
 */
#define  ms_spi_clear_multi_master_interrupt_status_hal(spi)   ms_spi_clear_multi_master_interrupt_status_ll(spi)


/**
 * @brief clear spi  interrupt status
 * @param  Spi_Type *spi: 
 * @retval  0 or 1
 */
#define ms_spi_clear_interrupt_status_hal(spi)    ms_spi_clear_interrupt_status_ll(spi)

/**
 * @brief spi read data from fifo
 * @param  Spi_Type *spi: 
 * @retval  0 or 1
 */
#define ms_spi_read_data_hal(spi)    ms_spi_read_data_ll(spi)

/**
 * @brief spi write data to fifo
 * @param  Spi_Type *spi: 
 * @retval  none
 */
#define ms_spi_write_data_hal(spi, write_data)     ms_spi_write_data_ll(spi, write_data)

/**
 * @brief set  spi frames number
 * @param  Spi_Type *spi: 
 * @retval none
 */
 #define ms_spi_set_sample_delay_value_hal(spi, sample_delay_value)  ms_spi_set_sample_delay_value_ll(spi, sample_delay_value)


#endif /* MS_SPI_MASTER_HAL_H_ */

 
