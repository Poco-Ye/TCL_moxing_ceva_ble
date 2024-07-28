/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file  ms_spi.c
 * @brief
 * @author haijun.mai
 * @date 2022-02-22
 * @version 1.0
 * @Revision
 */

#include "ms_spi_hal.h"
#include "ms_spi_slave.h"
#include "ms_spi_regs.h"
#include <ms_clock_hal.h>
#include "ms_sys_ctrl_regs.h"
#include "ms1008.h"
#include "ms_interrupt.h"
#include <stddef.h>
#include "log.h"


#define TX_FIFO_LEN   8
#define RX_FIFO_LEN   8


/**
 * @brief  Enable Cpu Interrupt
 * @param[in] SpiSlaveHandle_Type *spi : Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @retval None
 */
void ms_spi_slave_enable_cpu_interrupt(SpiSlaveHandle_Type *spi) {
       CHECK_PTR_NULL(spi);
	INTERRUPT_ENABLE_IRQ(spi->irq);
}

/**
 * @brief  Disable Cpu Interrupt
 * @param[in] SpiSlaveHandle_Type *spi : Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @retval None
 */
void ms_spi_slave_disable_cpu_interrupt(SpiSlaveHandle_Type *spi) { 
       CHECK_PTR_NULL(spi);
	INTERRUPT_DISABLE_IRQ(spi->irq);
}




/**
 * @brief  Initial SPI Controller
 * @param[in]  SpiSlaveHandle_Type *spi:  Pointer to a SpiSlaveHandle_Type  structure that contains
 * @ the configuration information for the specified SPI module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_spi_slave_init(SpiSlaveHandle_Type *spi)
{
    CHECK_PTR_NULL_RET(spi, STATUS_ERROR);
    /*initial uart spi pinmux, clock and interrupt setting*/
    if (spi->p_callback && spi->p_callback->init_callback)
    {
        spi->p_callback->init_callback(spi);
    }

    /*disable spi controller*/
    ms_spi_disable_hal(spi->instance);

    /*select  protocol frame format*/
    ms_spi_set_protocol_frame_format_hal(spi->instance, spi->init.protocol_frame_format);
	
      /*set slave toggle enable */
	ms_spi_slave_select_toggle_enable_hal(spi->instance, spi->init.sste);

    /*set data farme size */
    ms_spi_set_data_farme_size_hal(spi->instance,spi->init.frame_size);

	/*set data farme forma */
   ms_spi_set_frame_format_hal(spi->instance, spi->init.frame_forma);


    /*set spi clock phase*/
    ms_spi_set_serial_clock_phase_hal(spi->instance,spi->init.cpha);

     /*set spi clock pllarity*/
    ms_spi_set_serial_clock_polarity_hal(spi->instance, spi->init.cpol);

    /*set spi transfer mode*/
    ms_spi_set_transfer_mode_hal(spi->instance, spi->init.mode_transfer);

   /*set data frame number*/
  // ms_spi_set_data_frames_number_hal(spi->instance, spi->init.data_frames_number);

  /*set spi div*/
//   ms_spi_set_sck_div_hal(spi->instance, spi->init.sck_div);

  /*set spi transmit fifo threseold levle*/
  ms_spi_set_transmit_fifo_threshold_level_hal(spi->instance,spi->init.transmit_fifo_threshold);

  /*set spi receive fifo threshold level*/
  ms_spi_set_receive_fifo_threshold_level_hal(spi->instance, spi->init.rx_threshold_level);  

 /*set spi sample delay*/
  ms_spi_set_sample_delay_value_hal(spi->instance,spi->init.sample_delay);

 /*slave output enable */
  ms_spi_set_role_mode_hal(spi->instance, spi->init.role);

  ms_spi_set_interrupt_mask_hal(spi->instance,  (SPI_IMR_TXEIM|SPI_IMR_TXOIM|SPI_IMR_RXUIM|SPI_IMR_RXOIM|SPI_IMR_RXFIM|SPI_IMR_MSTIM)) ;


  //enable slave
  // ms_spi_slave_enable_hal(spi->instance);


   /*enable spi controller */
    ms_spi_enable_hal(spi->instance);
    return STATUS_SUCCESS;
}


/**
 * @brief  Deinitial SPI Controller
 * @param[in]  SpiSlaveHandle_Type *spi:  Pointer to a SpiSlaveHandle_Type  structure that contains
 * @ the configuration information for the specified SPI module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_spi_slave_deinit(SpiSlaveHandle_Type *spi)
{
     CHECK_PTR_NULL_RET(spi, STATUS_ERROR);
       /* DeInit the low level hardware */
    if (spi->p_callback && spi->p_callback->deinit_callback)
    {
        spi->p_callback->deinit_callback(spi);
    }
     return STATUS_SUCCESS;
}





/**
 * @brief  Transmit an amount of data in Polling Mode
 * @param[in]  SpiSlaveHandle_Type *spi:  Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @param[in] const uint8_t *data: Pointer to Transmit data buffer.
 * @param[in] uint16_t size: The length of Transmit data
 * @param[in] uint32_t timeout: The timerout value
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_spi_slave_send(SpiSlaveHandle_Type *spi, const uint8_t *data, uint16_t size, uint32_t timeout)
{
  
    int reg_value = 0;

    CHECK_PTR_NULL_RET(spi, STATUS_ERROR);
	
   /*disable spi controller*/
   ms_spi_disable_hal(spi->instance);

   /*set spi send data  only*/
   ms_spi_set_transfer_mode_hal(spi->instance, TRANSMIT_ONLY);   
   /*enable spi controller */
   ms_spi_enable_hal(spi->instance);

    while(size--)
    {
        //wait till tx fifo is not full
        do
        {
            reg_value =  ms_spi_get_status_hal(spi->instance, SPI_SR_TFNF );
        }while(SPI_SR_TFNF !=reg_value);
       MS_LOGI(MS_DRIVER, "\r\nwr data  = %x\n",*data);
	ms_spi_write_data_hal(spi->instance, *data++);  
    }

    //wait tx fifo empty	
    do
    {
        reg_value =  ms_spi_get_status_hal(spi->instance, SPI_SR_TFE );
    }while(SPI_SR_TFE != reg_value);

	

  //wait send finish
   do
   {
           reg_value =  ms_spi_get_status_hal(spi->instance, SPI_SR_BUSY );
    }while(SPI_SR_BUSY == reg_value);

	
return STATUS_SUCCESS;

   
}


/**
 * @brief  Receive Spi Data by Polling Mode
 * @param[in] SpiSlaveHandle_Type *spi: Pointer to a SpiHandle_Type structure that contains
 *            the configuration information for the specified SPI module.
 * @param[in] uint8_t *rx_data: Pointer to Received Buffer
 * @param[in] uint16_t size: The buffer length
 * @param[in] uint32_t timeout: The timer value
 * @retval The Received Data Length
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_spi_slave_receive(SpiSlaveHandle_Type *spi, uint8_t *rx_data, uint16_t size, uint32_t timeout)
{
    int reg_value = 0;

    CHECK_PTR_NULL_RET(spi, STATUS_ERROR);
	
    /*disable spi controller*/
    ms_spi_disable_hal(spi->instance);

    /*set spi receive data  only*/
   ms_spi_set_transfer_mode_hal(spi->instance, RECEIVE_ONLY);   

   /*set reveive data count */
   //ms_spi_set_data_frames_number_hal(spi->instance,  size - 1);    

    /*enable spi controller */
    ms_spi_enable_hal(spi->instance);

	//start to receive data
    ms_spi_write_data_hal(spi->instance, 0);  
    while(size--)
    {	
        //wait till rx fifo is not empty,
        do{
            reg_value =  ms_spi_get_status_hal(spi->instance, SPI_SR_RFNE );
        }while(SPI_SR_RFNE != reg_value);

        *rx_data++ = ms_spi_read_data_hal(spi->instance) ;
    }
    return STATUS_SUCCESS;

}



/**
 * @brief  Transmit an amount of data in interrupt mode
 * @param[in]  SpiSlaveHandle_Type *spi:  Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @param[in] const uint8_t *data: Pointer to Transmit data buffer.
 * @param[in] uint16_t size: The length of Transmit data
 * @param[in] uint32_t timeout: The timerout value
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_spi_slave_int_send(SpiSlaveHandle_Type *spi, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    int32_t reg_value = 0;


	
     CHECK_PTR_NULL_RET(spi, STATUS_ERROR);


     if(spi->interrupt.status == SPI_SLAVE_STATUS_BUSY)
     {
          return -1;
      }
   
    spi->interrupt.tx_total_len = size;
    spi->interrupt.tx_ptr = (uint8_t *)data;
    spi->interrupt.tx_len = 0; 
   /*disable spi controller*/
   ms_spi_disable_hal(spi->instance);

   /*set spi send data  only*/
   ms_spi_set_transfer_mode_hal(spi->instance, TRANSMIT_ONLY);   

  /*set transmit fifo threshold level*/
   ms_spi_set_transmit_fifo_threshold_level_hal(spi->instance,0);


    if( spi->interrupt.tx_total_len>0)
    {
   
	      //wait till tx fifo is not full
        do
        {
            reg_value =  ms_spi_get_status_hal(spi->instance, SPI_SR_TFNF );
        }while(SPI_SR_TFNF !=reg_value);


            /*set  busy  flag*/
          spi->interrupt.status = SPI_SLAVE_STATUS_BUSY;

         /*unmask Transmit FIFO Empty Interrupt*/
         ms_spi_clear_interrupt_mask_ll(spi->instance, SPI_IMR_TXEIM);
		  
          /*enable spi controller */
          ms_spi_enable_hal(spi->instance);

	
    }

    /*wait data send finish*/
    while(spi->interrupt.status == SPI_SLAVE_STATUS_BUSY);

	
return STATUS_SUCCESS;

   
}


/**
 * @brief  Receive Spi Data in interrupt mode 
 * @param[in] SpiSlaveHandle_Type *spi: Pointer to a SpiHandle_Type structure that contains
 *            the configuration information for the specified SPI module.
 * @param[in] uint8_t *rx_data: Pointer to Received Buffer
 * @param[in] uint16_t size: The buffer length
 * @param[in] uint32_t timeout: The timer value
 * @retval The Received Data Length
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_spi_slave_int_receive(SpiSlaveHandle_Type *spi, uint8_t *rx_data, uint16_t size, uint32_t timeout)
{
    int reg_value = 0;

    CHECK_PTR_NULL_RET(spi, STATUS_ERROR);

      if(spi->interrupt.status == SPI_SLAVE_STATUS_BUSY)
     {
          return -1;
      }

    spi->interrupt.rx_total_len = size;
    spi->interrupt.rx_ptr = (uint8_t *)rx_data;
    spi->interrupt.rx_len = 0; 
	
    /*disable spi controller*/
    ms_spi_disable_hal(spi->instance);

    /*set spi receive data  only*/
   ms_spi_set_transfer_mode_hal(spi->instance, RECEIVE_ONLY);   

   /*set reveive data count */
   //ms_spi_set_data_frames_number_hal(spi->instance,  0);    

  /*set transmit fifo threshold level*/
   ms_spi_set_receive_fifo_threshold_level_hal(spi->instance,0);

    /*enable spi controller */
    ms_spi_enable_hal(spi->instance);

	//start to receive data
    ms_spi_write_data_hal(spi->instance, 0);  

    /*set  busy  flag*/
    spi->interrupt.status = SPI_SLAVE_STATUS_BUSY;
    
    /*unmask receive FIFO full Interrupt*/
    ms_spi_clear_interrupt_mask_ll(spi->instance, SPI_IMR_RXFIM);

   /*wait data receive finish*/
    while(spi->interrupt.status == SPI_SLAVE_STATUS_BUSY);
	
    return STATUS_SUCCESS;

}



void spi_slave_int_fifo_empty(SpiSlaveHandle_Type *spi,uint8_t status)
{
   int32_t remain_len = 0;
   int32_t wr_cnt = 0;
   
  if(status&SPI_ISR_TXEIS)
  {

	// MS_LOGI(MS_DRIVER, "\r\tx_total_len  = %x  tx_len = %x\n",spi->interrupt.tx_total_len,spi->interrupt.tx_len);
      remain_len = spi->interrupt.tx_total_len - spi->interrupt.tx_len;
	  
       if(remain_len>0)
        {

	      //MS_LOGI(MS_DRIVER, "\r\remain_len  = %x\n",remain_len);
            if( remain_len < TX_FIFO_LEN)
            {
                 remain_len =  remain_len;
            }
             else
            {
                  remain_len = TX_FIFO_LEN;
             }


          //   MS_LOGI(MS_DRIVER, "\r\remain_len  = %x\n",remain_len);
            for(wr_cnt=0;wr_cnt<remain_len;wr_cnt++)
            {
                  ms_spi_write_data_hal(spi->instance, *(spi->interrupt.tx_ptr)++);  
    	     }

	     spi->interrupt.tx_len += remain_len;

	 
           
	
    }
     else
     {
            //  MS_LOGI(MS_DRIVER, "\r\ndisable spi controlle\n");

	       if( spi->interrupt.tx_len ==  spi->interrupt.tx_total_len)
	     {
                       /*mask Transmit FIFO Empty Interrupt*/
                   ms_spi_set_interrupt_mask_ll(spi->instance, SPI_IMR_TXEIM);
            }
             /*not busy*/
             spi->interrupt.status = SPI_SLAVE_STATUS_IDLE;
	
	      /*disable spi controller */
            ms_spi_disable_hal(spi->instance);

     }
	 


  }


}






void spi_slave_int_fifo_full(SpiSlaveHandle_Type *spi,uint8_t status)
{
   int32_t remain_len = 0;
   int32_t wr_cnt = 0;
   int32_t reg_value = 0;
   
  if(status&SPI_ISR_RXFIS)
  {
        /*read data from rx fifo */
        while(ms_spi_get_status_hal(spi->instance, SPI_SR_RFNE ))
        {
           //  MS_LOGI(MS_DRIVER, "\r\nrx_len  = %x\n",spi->interrupt.rx_len);
            *(spi->interrupt.rx_ptr)++ = ms_spi_read_data_hal(spi->instance) ;
            spi->interrupt.rx_len ++;
        }

  
      remain_len = spi->interrupt.rx_total_len - spi->interrupt.rx_len;
       //MS_LOGI(MS_DRIVER, "\r\nremain_len  = %x       rx_total_len = %x   rx_len = %x\n",remain_len,spi->interrupt.rx_total_len,spi->interrupt.rx_len,spi->interrupt.rx_len);
       if(remain_len>0)
        {
    
            if( remain_len < RX_FIFO_LEN)
            {
                 remain_len =  remain_len;
            }
             else
            {
                  remain_len = RX_FIFO_LEN;
             }
         
           /*disable spi controller*/
           ms_spi_disable_hal(spi->instance);
       
           /*set reveive data count */
         //  ms_spi_set_data_frames_number_hal(spi->instance,  remain_len - 1);    
         
           /*set transmit fifo threshold level*/
           ms_spi_set_receive_fifo_threshold_level_hal(spi->instance,remain_len-1);
         
           /*enable spi controller */
           ms_spi_enable_hal(spi->instance);
        
          //start to receive data
          //ms_spi_write_data_hal(spi->instance, 0);  
	
    }
     else
     {


             if( spi->interrupt.rx_len ==  spi->interrupt.rx_total_len)
	     {
                   /*mask receive FIFO full Interrupt*/
                   ms_spi_set_interrupt_mask_ll(spi->instance, SPI_ISR_RXFIS);
            }
	       // MS_LOGI(MS_DRIVER, "\r\nclose spi controller!\n");
             /*not busy*/
             spi->interrupt.status = SPI_SLAVE_STATUS_IDLE;
	
	      /*disable spi controller */
            ms_spi_disable_hal(spi->instance);

     }
	 


  }

}




/**
 * @brief  This function handles SPI interrupt request.
 * @param[in] SpiSlaveHandle_Type *spi: Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @retval None
 */
void ms_spi_slave_irq_handler(SpiSlaveHandle_Type *spi) {
    int32_t reg_value = 0;
    int32_t send_len = 0;
    int32_t wr_cnt = 0;

       CHECK_PTR_NULL(spi);
	
     //MS_LOGI(MS_DRIVER, "\r\nms_spi_master_irq_handler\n");

       uint8_t status  = ms_spi_get_interrupt_status_hal(spi->instance);

       //MS_LOGI(MS_DRIVER, "\r\nstatus = %x\n",status);
     switch (status & 0x3F)
    {
    case SPI_ISR_TXEIS:
        /*Transmit FIFO Empty Interrupt Status*/
	 spi_slave_int_fifo_empty(spi,SPI_ISR_TXEIS);
	
        break;


    case SPI_ISR_TXOIS:
     
        break;

    case SPI_ISR_RXUIS:
    
        break; 
    
    case SPI_ISR_RXOIS:
    
        break; 
    
    
    case SPI_ISR_RXFIS:
       /*Transmit FIFO Full Interrupt Status*/
	spi_slave_int_fifo_full(spi,SPI_ISR_RXFIS);
    
        break; 
    
    case SPI_ISR_MSTIS:
    
        break; 
    default:
        break;
    }

    
  if (spi->p_callback == NULL) {
		spi->error_code |= SPI_SLAVE_ERROR_INVALID_CALLBACK;
		return;
	}
	spi->p_callback->spi_reach_callback(spi);
	
}





