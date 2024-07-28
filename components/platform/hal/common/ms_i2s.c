/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_watchdog_hal.c
 * @brief c source  file of watchdog  module.
 * @author haijun.mai
 * @date   2022-03-08
 * @version 1.0
 * @Revision
 */

#include <ms1008.h>
#include "ms_i2s_hal.h"
#include "ms_i2s.h"
#include "ms_i2s_regs.h"
#include "ms_clock_hal.h"
#include "ms_sys_ctrl_regs.h"
#include "ms_interrupt.h"
#include <stddef.h>
#include "log.h"


#define I2S_TX_FIFO_LEN   16
#define I2S_RX_FIFO_LEN   16



/**
 * @brief enable i2s interrupt
 * @param  I2sHandle_Type *i2s:
 * @retval None
 */
void ms_i2s_enable_cpu_interrupt(I2sHandle_Type *i2s) {
	INTERRUPT_ENABLE_IRQ(i2s->irq);
}

/**
 * @brief disable watchdog interrupt
 * @param  I2sHandle_Type *i2s: 
 * @retval None
 */
void ms_i2s_disable_cpu_interrupt(I2sHandle_Type *i2s) {
	INTERRUPT_DISABLE_IRQ(i2s->irq);
}



/**
 * @brief  Initial I2s Controller
 * @param[in]  2sHandle_Type *i2s:  Pointer to a I2sHandle_Type  structure that contains
 * @ the configuration information for the specified SPI module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_i2s_master_init(I2sHandle_Type *i2s)
{
    CHECK_PTR_NULL_RET(i2s, STATUS_ERROR);
    /*initial uart spi pinmux, clock and interrupt setting*/
    if (i2s->p_callback && i2s->p_callback->init_callback)
    {
        i2s->p_callback->init_callback(i2s);
    }

    /*enable i2s controller*/
    ms_i2s_enable_hal(i2s->instance);


  

   /*config i2s rx valid data len*/
    ms_i2s_cfg_rx_resolution_wlen_hal(i2s->instance,i2s->init.rx_data_valid_len);

      /*config i2s tx valid data len*/
    ms_i2s_cfg_tx_resolution_wlen_hal(i2s->instance,i2s->init.tx_data_valid_len);
  

   /*configi2s rx fifo threshold*/
   ms_i2s_cfg_rx_fifo_trigger_level_hal(i2s->instance,i2s->init.rx_fifo_trigger_level);  

   /*configi2s tx fifo threshold*/
   ms_i2s_cfg_tx_fifo_trigger_level_hal(i2s->instance,i2s->init.tx_fifo_trigger_level);


     /*reset i2s rx fifo*/
   ms_i2s_rx_fifo_reset_hal(i2s->instance); 

    /*reset i2s tx fifo*/
    ms_i2s_tx_fifo_reset_hal(i2s->instance); 


   if(i2s->init.i2s_tx_block_en == I2S_TRUE)
   {
        ms_i2s_tx_block_enable_hal(i2s->instance);
   }
   else
   {
        ms_i2s_tx_block_disable_hal(i2s->instance);
   }


   
   if(i2s->init.i2s_rx_block_en == I2S_TRUE)
   {
        ms_i2s_rx_block_enable_hal(i2s->instance);
   }
   else
   {
        ms_i2s_rx_block_disable_hal(i2s->instance);
   }
  

      if(i2s->init.i2s_tx_channel_en == I2S_TRUE)
   {
        ms_i2s_tx_channel_enable_hal(i2s->instance);
   }
   else
   {
        ms_i2s_tx_channel_disable_hal(i2s->instance);
   }


   
      if(i2s->init.i2s_rx_channel_en== I2S_TRUE)
   {
        ms_i2s_rx_channel_enable_hal(i2s->instance);
   }
   else
   {
        ms_i2s_rx_channel_disable_hal(i2s->instance);
   }



    
      if(i2s->init.i2s_tx_dma_en == I2S_TRUE)
   {
        ms_i2s_dma_tx_channel_enable_hal(i2s->dma_instance);
   }
   else
   {
        ms_i2s_dma_tx_channel_disable_hal(i2s->dma_instance);
   }


      if(i2s->init.i2s_rx_dma_en == I2S_TRUE)
   {
        ms_i2s_dma_rx_channel_enable_hal(i2s->dma_instance);
   }
   else
   {
        ms_i2s_dma_rx_channel_disable_hal(i2s->dma_instance);
   }


   if(i2s->init.role == I2S_MASTER == I2S_TRUE)
   {
	  /*config i2s wss*/
	    ms_i2s_cfg_wss_hal(i2s->instance,i2s->init.wss);

	   /*config i2s sclkg*/
	   ms_i2s_cfg_sclkg_hal(i2s->instance,i2s->init.sclkg);

	    ms_i2s_clock_enable_hal(i2s->instance);
   }



    return STATUS_SUCCESS;
}


/**
 * @brief  Deinitial I2S Controller
 * @param[in]  2sHandle_Type *i2s:  Pointer to a I2sHandle_Type  structure that contains
 * @ the configuration information for the specified SPI module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_i2s_master_deinit(I2sHandle_Type *i2s)
{
     CHECK_PTR_NULL_RET(i2s, STATUS_ERROR);
       /* DeInit the low level hardware */
    if (i2s->p_callback && i2s->p_callback->deinit_callback)
    {
        i2s->p_callback->deinit_callback(i2s);
    }
     return STATUS_SUCCESS;
}




void ms_i2s_master_send_data(I2sHandle_Type *i2s, uint32_t* left_chan_data, uint32_t* right_chan_data, uint32_t len)
{
    while(1)
    {
        while( !(I2S_ISRX_TXFE&ms_i2s_get_interrupt_status_hal(i2s->instance)) ); // wait till tx fifo emptys
        if( len <= I2S_FIFO_DEPTH )
        {
            for( int i = 0; i < len; i++)
            {
			ms_i2s_fill_left_channel_data_hal(i2s->instance,*left_chan_data++);
			ms_i2s_fill_right_channel_data_hal(i2s->instance,*right_chan_data++);
            }
            return;
        }
        else
        {
            for( int i = 0; i < I2S_FIFO_DEPTH; i++)
            {
			ms_i2s_fill_left_channel_data_hal(i2s->instance,*left_chan_data++);
			ms_i2s_fill_right_channel_data_hal(i2s->instance,*right_chan_data++);
            }
            len -= I2S_FIFO_DEPTH;
        }
    }
}



uint32_t ms_i2s_master_receive_data(I2sHandle_Type *i2s,  uint32_t *left_chan_data, uint32_t *right_chan_data, uint32_t len)
{
      for(uint32_t  i=0;i<len;i++)
      {
             while( !(I2S_ISRX_RXDA&ms_i2s_get_interrupt_status_hal(i2s->instance)) ); // wait till tx fifo emptys
            left_chan_data[i]  = ms_i2s_get_left_channel_data_hal(i2s->instance);
            right_chan_data[i] =  ms_i2s_get_right_channel_data_hal(i2s->instance);
    }

}


/**
 * @brief  Transmit an amount of data in interrupt mode
 * @param[in]  I2sHandle_Type *i2s:  Pointer to a I2sHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @param[in] uint32_t* left_chan_data: Pointer to Transmit left channel data buffer.
 * @param[in] uint32_t* right_chan_data: Pointer to Transmit right channel data buffer.
 * @param[in] uint32_t len: The length of Transmit data
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_i2s_master_int_send_data(I2sHandle_Type *i2s, uint32_t* left_chan_data, uint32_t* right_chan_data, uint32_t len)
{
    int32_t reg_value = 0;


	
     CHECK_PTR_NULL_RET(i2s, STATUS_ERROR);


     if(i2s->interrupt.status == I2S_MASTER_STATUS_BUSY)
     {
          return -1;
      }
   
    i2s->interrupt.tx_total_len = len;
    i2s->interrupt.tx_l_ptr = left_chan_data;
    i2s->interrupt.tx_r_ptr = right_chan_data;
    i2s->interrupt.tx_len = 0; 



  /*set transmit fifo threshold level*/
   ms_i2s_cfg_tx_fifo_trigger_level_hal(i2s->instance,0);


    if( i2s->interrupt.tx_total_len>0)
    {
   
	      //wait till tx fifo is not full
        do
        {
            reg_value =  ms_i2s_get_interrupt_status_hal(i2s->instance);
        }while( I2S_ISRX_TXFE  != (reg_value&I2S_ISRX_TXFE));


            /*set  busy  flag*/
          i2s->interrupt.status = I2S_MASTER_STATUS_BUSY;

         /*unmask Transmit FIFO Empty Interrupt*/
         ms_i2s_interrupt_unmask_hal(i2s->instance, I2S_IMRX_TXFEM);
		  

            MS_LOGI(MS_DRIVER, "\r\n i2s->interrupt.status  = %x\n",i2s->interrupt.status );
          /*wait data send finish*/
         while(i2s->interrupt.status == I2S_MASTER_STATUS_BUSY);
	
    }



	
return STATUS_SUCCESS;

   
}


/**
 * @brief  Receive Spi Data in interrupt mode 
 * @param[in] I2sHandle_Type *i2si: Pointer to a I2sHandle_Type structure that contains
 *            the configuration information for the specified I2S module.
 * @param[in] uint32_t* left_chan_data: Pointer to Received left channel Buffer
 * @param[in] uint32_t* right_chan_data: Pointer to Received right channel Buffer
 * @param[in] uint32_t len: The buffer length
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_i2s_master_int_receive_data(I2sHandle_Type *i2s,  uint32_t* left_chan_data, uint32_t* right_chan_data, uint32_t len)
{
    int reg_value = 0;

    CHECK_PTR_NULL_RET(i2s, STATUS_ERROR);

      if(i2s->interrupt.status == I2S_MASTER_STATUS_BUSY)
     {
          return -1;
      }

    i2s->interrupt.rx_total_len = len;
    i2s->interrupt.rx_l_ptr = left_chan_data;
    i2s->interrupt.rx_r_ptr = right_chan_data;
    i2s->interrupt.rx_len = 0; 

  /*set receive  fifo threshold level*/
   ms_i2s_cfg_rx_fifo_trigger_level_ll(i2s->instance,0);


    /*set  busy  flag*/
    i2s->interrupt.status = I2S_MASTER_STATUS_BUSY;
    
    /*unmask receive FIFO full Interrupt*/
    ms_i2s_interrupt_unmask_hal(i2s->instance, I2S_IMRX_RXFOM);
    //MS_LOGI(MS_DRIVER, "\r\n status = %x\n",i2s->interrupt.status);
   /*wait data receive finish*/
    while(i2s->interrupt.status == I2S_MASTER_STATUS_BUSY)
    {
         MS_LOGI(MS_DRIVER, "\r\n status = %x\n",i2s->interrupt.status);;
    }
    //MS_LOGI(MS_DRIVER, "\r\n xxx2!\n");
    return STATUS_SUCCESS;

}



static void ms_i2s_master_dma_tx_xfer_tfr_callback(DmacHandle_Type *hdma)
{
    CHECK_PTR_NULL(hdma);

   
}

 static void ms_i2s_master_dma_tx_xfer_error_callback(DmacHandle_Type *hdma)
{
    CHECK_PTR_NULL(hdma);

  
}

static void ms_i2s_master_dma_rx_xfer_tfr_callback(DmacHandle_Type *hdma)
{
    CHECK_PTR_NULL(hdma);

}
static void ms_i2s_master_dma_rx_xfer_error_callback(DmacHandle_Type *hdma)
{
    CHECK_PTR_NULL(hdma);

}



/**
 * @brief  Transmit an amount of data in dma mode
 * @param  i2s  Pointer to a I2sHandle_Type structure that contains
 *                the configuration information for the specified I2S module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_i2s_master_dma_send_data(I2sHandle_Type *i2s, uint32_t *p_data, uint16_t size)
{
    CHECK_PTR_NULL_RET(i2s, STATUS_ERROR);
    CHECK_PTR_NULL_RET(i2s->hdmatx, STATUS_ERROR);
    CHECK_PTR_NULL_RET(p_data, STATUS_ERROR);

 
    /*set transmit fifo threshold level*/
    ms_i2s_cfg_tx_fifo_trigger_level_hal(i2s->instance,0);
 

    /*Wait for dmac channel to be idle*/
    while (ms_dmac_hal_is_enable_channel(i2s->hdmatx->dma_instance, i2s->hdmatx->channel_index));

    i2s->hdmatx->xfer_tfr_callback = ms_i2s_master_dma_tx_xfer_tfr_callback;
    i2s->hdmatx->xfer_error_callback = ms_i2s_master_dma_tx_xfer_error_callback;
    uint32_t *tmp = (uint32_t*) &p_data;
    ms_dmac_start_chx_xfer_it(i2s->hdmatx, *(uint32_t*) tmp, (uint32_t) &i2s->dma_instance->TXDMA, size);

    return STATUS_SUCCESS;
}

int32_t ms_i2s_master_dma_receive_data(I2sHandle_Type *i2s, uint32_t *buffer_ptr, uint32_t size)
{
    CHECK_PTR_NULL_RET(i2s, STATUS_ERROR);
    CHECK_PTR_NULL_RET(i2s->hdmarx, STATUS_ERROR);
    CHECK_PTR_NULL_RET(buffer_ptr, STATUS_ERROR);

 
    /*set transmit fifo threshold level*/
 //  ms_i2s_cfg_rx_fifo_trigger_level_hal(i2s->instance,4);
    
    /*Wait for dmac channel to be idle*/
    while (ms_dmac_hal_is_enable_channel(i2s->hdmarx->dma_instance, i2s->hdmarx->channel_index));

    i2s->hdmarx->xfer_tfr_callback = ms_i2s_master_dma_rx_xfer_tfr_callback;
    i2s->hdmarx->xfer_error_callback = ms_i2s_master_dma_rx_xfer_error_callback;
    uint32_t *tmp = (uint32_t*) &buffer_ptr;
    ms_dmac_start_chx_xfer_it(i2s->hdmarx, (uint32_t) &i2s->dma_instance->RXDMA, *(uint32_t*) tmp, size);

    return STATUS_SUCCESS;
}


void i2s_master_int_fifo_empty(I2sHandle_Type *i2s,uint8_t status)
{
   int32_t remain_len = 0;
   int32_t wr_cnt = 0;
   
  if(status&I2S_ISRX_TXFE)
  {

	// MS_LOGI(MS_DRIVER, "\r\tx_total_len  = %x  tx_len = %x\n",i2s->interrupt.tx_total_len,i2s->interrupt.tx_len);
      remain_len = i2s->interrupt.tx_total_len - i2s->interrupt.tx_len;
	  
       if(remain_len>0)
        {

	      //MS_LOGI(MS_DRIVER, "\r\remain_len  = %x\n",remain_len);
            if( remain_len < I2S_TX_FIFO_LEN)
            {
                 remain_len =  remain_len;
            }
             else
            {
                  remain_len = I2S_TX_FIFO_LEN;
             }
            // MS_LOGI(MS_DRIVER, "\r\remain_len  = %x\n",remain_len);
            for(wr_cnt=0;wr_cnt<remain_len;wr_cnt++)
            {
                
		    ms_i2s_fill_left_channel_data_hal(i2s->instance,*(i2s->interrupt.tx_l_ptr)++);
		    ms_i2s_fill_right_channel_data_hal(i2s->instance,*(i2s->interrupt.tx_r_ptr)++);
    	     }

	     i2s->interrupt.tx_len += remain_len;


    }
     else
     {
            // MS_LOGI(MS_DRIVER, "\r\ndisable spi controlle\n");

	       if( i2s->interrupt.tx_len ==  i2s->interrupt.tx_total_len)
	     {
                       /*mask Transmit FIFO Empty Interrupt*/
                   ms_i2s_interrupt_mask_hal(i2s->instance, I2S_ISRX_TXFE);
            }
             /*not busy*/
             i2s->interrupt.status = I2S_MASTER_STATUS_IDLE;
	
	  
     }
	 


  }


}



void i2s_master_int_fifo_full(I2sHandle_Type *i2s,uint8_t status)
{
   int32_t remain_len = 0;
   int32_t rd_cnt = 0;
   int32_t reg_value = 0;
   
   
  if(status&I2S_ISRX_RXFO)
  {

         remain_len = i2s->interrupt.rx_total_len - i2s->interrupt.rx_len;

	 if(remain_len >I2S_RX_FIFO_LEN )	
	 {
		remain_len = I2S_RX_FIFO_LEN;
	 }
	 else 
	 {
                /*mask receive FIFO full Interrupt*/
                   ms_i2s_interrupt_mask_hal(i2s->instance, I2S_ISRX_RXFO);
         
	       // MS_LOGI(MS_DRIVER, "\r\n close i2s controller!\n");
             /*not busy*/
             i2s->interrupt.status = I2S_MASTER_STATUS_IDLE;
	 }

	    //MS_LOGI(MS_DRIVER, "\r\n re  len  = %x\n",remain_len);
	   /*set receive  fifo threshold level*/
          ms_i2s_cfg_rx_fifo_trigger_level_ll(i2s->instance,remain_len);

	 for(rd_cnt=0; rd_cnt< remain_len; rd_cnt++)
	 {
		   
	        /*read data from rx fifo */
	        while(!(I2S_ISRX_RXDA&ms_i2s_get_interrupt_status_hal(i2s->instance)));
	        
	            *(i2s->interrupt.rx_l_ptr)++  = ms_i2s_get_left_channel_data_hal(i2s->instance);
	            *(i2s->interrupt.rx_r_ptr)++ =  ms_i2s_get_right_channel_data_hal(i2s->instance);
			
	             //MS_LOGI(MS_DRIVER, "\r\n rxx_len  = %x\n",i2s->interrupt.rx_len);
	        
	            i2s->interrupt.rx_len ++;
		     
		
	       
	 }
	 
         
  
   
      // MS_LOGI(MS_DRIVER, "\r\nremain_len  = %x       rx_total_len = %x   rx_len = %x\n",remain_len,i2s->interrupt.rx_total_len,i2s->interrupt.rx_len,i2s->interrupt.rx_len);
   

  }

}



/**
 * @brief  This function handles SPI interrupt request.
 * @param[in] SpiMasterHandle_Type *spi: Pointer to a SpiHandle_Type structure that contains
 * @the configuration information for the specified SPI module.
 * @retval None
 */
void ms_i2s_master_irq_handler(I2sHandle_Type *i2s) {

       CHECK_PTR_NULL(i2s);
	
    

       uint8_t status  = ms_i2s_get_interrupt_status_hal(i2s->instance);
	uint8_t mask  = ms_i2s_get_interrupt_mask_hal(i2s->instance);

       //MS_LOGI(MS_DRIVER, "\r\nstatus = %x\n",status);
	//MS_LOGI(MS_DRIVER, "\r\nmask = %x   umask = %x    xxx = %x\n",mask,(~mask),(status & 0x3F &(~mask)));
     switch (status & 0x3F &(~mask))
    {
    case I2S_ISRX_RXDA:
    
        break;


    case I2S_ISRX_RXFO:
        i2s_master_int_fifo_full(i2s,I2S_ISRX_RXFO);
        break;

    case I2S_ISRX_TXFE:
         i2s_master_int_fifo_empty(i2s,I2S_ISRX_TXFE);
        break; 
    
    case I2S_ISRX_TXFO:
    
        break; 
    
    default:
        break;
    }

    
  if (i2s->p_callback == NULL) {
		i2s->error_code |= I2S_ERROR_INVALID_CALLBACK;
		return;
	}
	i2s->p_callback->i2s_reach_callback(i2s);
	
}


