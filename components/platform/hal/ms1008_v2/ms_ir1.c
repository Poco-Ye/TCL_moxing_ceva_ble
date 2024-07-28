/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_irl.c
 * @brief c source  file of ir  module.
 * @author haijun.mai
 * @date   2022-03-29
 * @version 1.0
 * @Revision
 */

#include <ms1008.h>
#include "ms_ir1_hal.h"
#include "ms_ir1.h"
#include "ms_ir1_regs.h"
#include "ms_clock_hal.h"
#include "ms_interrupt.h"
#include <stddef.h>
#include "ms_sys_wild_hal.h"
#include "log.h"


Ir1Handle_Type *g_ir_handdle_ptr = NULL;
#define IR1_TX_FIFO_LEN   16
#if 0
void ms_ir1_init_dma_tx(DmacHandle_Type **phdmatx)
{
    /*select dma port for ir1 */
    //*(volatile unsigned int *)(0x40110000+0x108)  &= ~(0x3<<2);	
    //*(volatile unsigned int *)(0x40110000+0x108)  |=   (0x1<<3);	
   MS_SYS_HAL_ENABLE_IR1_DMA_TREQ(); 	

    //    *phdmatx = NULL;
    DmacHandle_Type *hdmatx;
    hdmatx = ms_dmac_mgmt_alloc();
    *phdmatx = hdmatx;

    hdmatx->init.dst_tr_width = DMAC_XFER_WIDTH_32BITS;
    hdmatx->init.dst_addr_mode = DMAC_ADDRESS_MODE_NOC;
    hdmatx->init.dst_peri_type = DMAC_I2S1_TX;
    hdmatx->init.dst_msize = DMAC_MSIZE_1;
    hdmatx->init.src_peri_type = DMAC_MEM;
    hdmatx->init.src_tr_width = DMAC_XFER_WIDTH_32BITS;
    hdmatx->init.src_addr_mode = DMAC_ADDRESS_MODE_INC;
    hdmatx->init.src_msize = DMAC_MSIZE_1;
    ms_dmac_init(hdmatx);
}
#else 


void ms_ir1_init_dma_tx(DmacHandle_Type **phdmatx)
{
    /*select dma port for ir1 */
   // *(volatile unsigned int *)(0x40110000+0x108)  &= ~(0x3<<2);	
  //  *(volatile unsigned int *)(0x40110000+0x108)  |=   (0x1<<2);	
   MS_SYS_HAL_ENABLE_IR1_DMA_RREQ();
    //    *phdmatx = NULL;
    DmacHandle_Type *hdmatx;
    hdmatx = ms_dmac_mgmt_alloc();
    *phdmatx = hdmatx;

    hdmatx->init.dst_tr_width = DMAC_XFER_WIDTH_32BITS;
    hdmatx->init.dst_addr_mode = DMAC_ADDRESS_MODE_NOC;
    hdmatx->init.dst_peri_type = DMAC_I2S1_RX;
    hdmatx->init.dst_msize = DMAC_MSIZE_1;
    hdmatx->init.src_peri_type = DMAC_MEM;
    hdmatx->init.src_tr_width = DMAC_XFER_WIDTH_32BITS;
    hdmatx->init.src_addr_mode = DMAC_ADDRESS_MODE_INC;
    hdmatx->init.src_msize = DMAC_MSIZE_1;
    ms_dmac_init(hdmatx);
}



#endif



/**
 * @brief enable ir interrupt
 * @param  Ir_Type *ir: 
 * @retval None
 */
int32_t ms_ir1_enable_cpu_interrupt(Ir1Handle_Type *ir) 
{
	INTERRUPT_ENABLE_IRQ(ir->irq);
	 return STATUS_SUCCESS;
}

/**
 * @brief disable ir interrupt
 * @param  Ir_Type *ir: 
 * @retval None
 */
int32_t ms_ir1_disable_cpu_interrupt(Ir1Handle_Type *ir) 
{
	INTERRUPT_DISABLE_IRQ(ir->irq);
	 return STATUS_SUCCESS;
}


/**
 * @brief ir module init
 * @param  IrHandle_Type *ir: 
 * @retval None
 */
int32_t ms_ir1_init(Ir1Handle_Type *ir)
{
	if (NULL == ir) {
		STATUS_ERROR;
	}
      MS_SYS_HAL_ENABLE_IR1();
	
	/*initial uart gpio pinmux, clock and interrupt setting*/
	if (ir->p_callback && ir->p_callback->init_callback) {
		ir->p_callback->init_callback(ir);
	}

    
    /*select tx proto mode*/
    ms_ir1_config_proto_mode_hal(ir->instance, ir->init.proto_mode);
    

    /*config tx carry high cycle cnt*/
    ms_ir1_tx_carry_high_cycle_config_hal(ir->instance,ir->init.carry_high_cycle);
    
    /*config tx carry high cycle cnt*/
    ms_ir1_tx_carry_low_cycle_config_hal(ir->instance,ir->init.carry_low_cycle);

    ms_ir1_config_carry_compensation_mode_hal(ir->instance,ir->init.carry_compensation_mode);	

    if(ir->init.proto_mode == IR1_PROCOTOL_MODE_USER_DEFINE)
    {

	if(ir->init.arbitrary_data_mode)
	{
            /*fifo data level control by hard ware*/
            ms_ir1_fifo_data_level_enable_hal(ir->instance);
	}
	else
	{
            /*fifo data level control by solt ware*/
	    ms_ir1_fifo_data_level_disable_hal(ir->instance) ;
	}
    

            	
        /*config dma interface*/
        if(ir->init.dma_enable == IR1_TRUE)
        {
            ms_ir1_tx_dma_enable_hal(ir->instance); 
        }
        else
        {
        
            ms_ir1_tx_dma_disable_hal(ir->instance);
        }

    }
    else
    {
     
	   
        /*config start bit high level carry  cycle cnt*/
        ms_ir1_config_proto_start_bit_high_cycle_hal(ir->instance,ir->init.start_carry_cnt);	 
    
        /*config start bit low level carry  cycle cnt*/
        ms_ir1_config_proto_start_bit_low_cycle_hal(ir->instance,ir->init.start_idle_cnt);
    
        
         /*config data0 low level  carry  cycle cnt*/
        ms_ir1_config_proto_data0_bit_low_cycle_hal( ir->instance, ir->init.data0_idle_cnt);
    
         /*config data1 low level  carry  cycle cnt*/
        ms_ir1_config_proto_data1_bit_low_cycle_hal(ir->instance, ir->init.data1_idle_cnt);
    
         /*config end low level  carry  cycle cnt*/
        ms_ir1_config_proto_end_bit_low_cycle_hal(ir->instance,ir->init.end_idle_cnt);
        
        
        /*config data0,data1,end  high level  carry  cycle cnt*/
        ms_ir1_config_proto_data_and_end_bit_high_cycle_hal( ir->instance, ir->init.end_data0_data1_carry_cnt); 
        
        /*config repeat low level  carry  cycle cnt*/
        ms_ir1_config_proto_repeat_end_bit_low_cycle_hal(  ir->instance, ir->init.repeat_carry_cnt);
    }
	


      return STATUS_SUCCESS;

}








int32_t ms_ir1_send_data(Ir1Handle_Type *ir,uint32_t tx_data, uint32_t size) 
{
     int reg_value = 0;
   /*flash fifo*/
    ms_ir1_flush_fifo_hal(ir->instance);  

		
    /*set tx data len*/
     ms_ir1_set_tx_length_hal(ir->instance,size);
	
     ms_ir1_tx_data_fill_hal( ir->instance, tx_data);


    /*enable tx */
    ms_ir1_tx_enable_hal(ir->instance);


   
    do {
        reg_value =(IR1_STATUS_TX_DONE_STA&ms_ir1_get_status_hal(ir->instance));
		//MS_LOGI(MS_DRIVER, "r= %x\n",reg_value );
    } while(!reg_value);

   #if 1
    do {
        reg_value =(IR1_STATUS_IR_BUSY_STA&ms_ir1_get_status_hal(ir->instance));
		//MS_LOGI(MS_DRIVER, "b= %x\n",reg_value );
    } while(reg_value);
#endif
   /*wait tx data send finish*/
 //  while(IR1_STATUS_TX_DONE_STA != (IR1_STATUS_TX_DONE_STA&ms_ir1_get_status_hal(ir->instance)));

   ms_ir1_clear_status_hal( ir->instance, IR1_STATUS_TX_DONE_STA);
 
   
    /*disable tx*/
    ms_ir1_tx_disable_hal(ir->instance);

     return STATUS_SUCCESS;
}





int32_t ms_ir1_arbitrary_send_data(Ir1Handle_Type *ir,uint32_t *tx_data, uint32_t size) 
{
    //int32_t level_pol_sel = 0;
    int32_t  reg_value = 0;
 
 
    /*flash fifo*/
    ms_ir1_flush_fifo_hal(ir->instance);   

 
    /*set tx data len*/
     ms_ir1_set_tx_length_hal(ir->instance,size-1);
	


    /*enable tx */
    ms_ir1_tx_enable_hal(ir->instance);
   


   for(int32_t i = 0;i<size;i++)	
   {

	  do {
                  reg_value =(IR1_STATUS_TX_FIFO_EMPTY_STA &ms_ir1_get_status_hal(ir->instance));
		  // MS_LOGI(MS_DRIVER, "r= %x\n",reg_value );
        } while(reg_value != IR1_STATUS_TX_FIFO_EMPTY_STA);
         ms_ir1_tx_level_cycle_config_hal( ir->instance, tx_data[i]);


   }


   /*wait send data finish*/
    do {
        reg_value =(IR1_STATUS_TX_DONE_STA&ms_ir1_get_status_hal(ir->instance));
		//MS_LOGI(MS_DRIVER, "r= %x\n",reg_value );
    } while(!reg_value);

   #if 0
    do {
        reg_value =(IR1_STATUS_IR_BUSY_STA&ms_ir1_get_status_hal(ir->instance));
		//MS_LOGI(MS_DRIVER, "b= %x\n",reg_value );
    } while(reg_value);
#endif
   
    /*enable tx */
    ms_ir1_tx_disable_hal(ir->instance);
	 
     return STATUS_SUCCESS;
}



int32_t ms_ir1_arbitrary_interrupt_send_data(Ir1Handle_Type *ir,uint32_t *tx_data, uint32_t size) 
{

   
     if(ir->interrupt.status == IR1_STATUS_BUSY)
     {
          return STATUS_ERROR;
      }

    ir->interrupt.tx_total_len = size;
    ir->interrupt.tx_ptr = tx_data;
    ir->interrupt.tx_len = 0; 
    ir->interrupt.status = IR1_STATUS_BUSY;
    /*flash fifo*/
    ms_ir1_flush_fifo_hal(ir->instance);   

    /*set tx data len*/
     ms_ir1_set_tx_length_hal(ir->instance,size-1);

    /*config water mask*/	
    ms_ir1_set_tx_watermask_hal(ir->instance, 0);
   
   /*enable interrupt*/
    ms_ir1_tx_int_enable_hal(ir->instance);  

    /*enable tx */
    ms_ir1_tx_enable_hal(ir->instance);

   /*enable ir1 interrupt*/
    ms_ir1_interrupt_unmask_hal(ir->instance,(IR1_INT_MASK_TX_FIFO_EMPTY|IR1_INT_MASK_TX_DONE));


    /*wait data send finish*/
     while(IR1_STATUS_BUSY == ir->interrupt.status );
  

     return STATUS_SUCCESS;
}


int32_t ms_ir1_arbitrary_dma_send_data(Ir1Handle_Type *ir,uint32_t *tx_data, uint32_t size) 
{
   
    CHECK_PTR_NULL_RET(ir, STATUS_ERROR);
    CHECK_PTR_NULL_RET(ir->hdmatx, STATUS_ERROR);
    CHECK_PTR_NULL_RET(tx_data, STATUS_ERROR);

   
    /*set transmit fifo threshold level*/
   ms_ir1_set_tx_watermask_hal(ir->instance, 0);
   
    /*flash fifo*/
    ms_ir1_flush_fifo_hal(ir->instance);   


    /*set tx data len*/
     ms_ir1_set_tx_length_hal(ir->instance,size-1);
	

    /*enable tx */
    ms_ir1_tx_enable_hal(ir->instance);


    /*Wait for dmac channel to be idle*/
    while (ms_dmac_hal_is_enable_channel(ir->hdmatx->dma_instance, ir->hdmatx->channel_index));

    ir->hdmatx->xfer_tfr_callback = NULL;
    ir->hdmatx->xfer_error_callback = NULL;
    uint32_t *tmp = (uint32_t*) &tx_data;
    ms_dmac_start_chx_xfer_it(ir->hdmatx, *(uint32_t*) tmp, (uint32_t) &ir->instance->IR1_TX_LEVEL_CFG, size*4);

    return STATUS_SUCCESS;	


}



int32_t ms_ir1_send_data_repeat(Ir1Handle_Type *ir,uint32_t tx_data,uint32_t size) 
{

    /*flash fifo*/
    ms_ir1_flush_fifo_hal(ir->instance);   
    
    /*set tx data len*/
    ms_ir1_set_tx_length_hal(ir->instance,size);
    
    ms_ir1_tx_data_fill_hal( ir->instance, tx_data);

    /*repeat send data enable */
    ms_ir1_proto_repeat_enable_hal(ir->instance); 
    
    /*enable tx */
    ms_ir1_tx_enable_hal(ir->instance);
    
   
    return STATUS_SUCCESS;
}



int32_t ms_ir1_send_data_repeat_stop(Ir1Handle_Type *ir) 
{

    /*disable tx*/
    ms_ir1_tx_disable_hal(ir->instance);
    
    return STATUS_SUCCESS;
}


/**
 * @brief  Deinitial Ir Controller
 * @param[in]  IrHandle_Type *ir:  Pointer to a IrHandle_Type  structure that contains
 * @ the configuration information for the specified Ir module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_ir1_deinit(Ir1Handle_Type *ir)
{
     CHECK_PTR_NULL_RET(ir, STATUS_ERROR);
       /* DeInit the low level hardware */
    if (ir->p_callback && ir->p_callback->deinit_callback)
    {
        ir->p_callback->deinit_callback(ir);
    }
     return STATUS_SUCCESS;
}



/**
 * @brief unmask ir interrupt
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t interrupt_source
 * @retval None
 */
int32_t  ms_ir1_interrupt_unmask(Ir1Handle_Type* ir,uint32_t interrupt_source)    
{
    ms_ir1_interrupt_unmask_hal(ir->instance, interrupt_source);
    return STATUS_SUCCESS;
}




/**
 * @brief mask ir interrupt
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t interrupt_source
 * @retval None
 */
int32_t  ms_ir1_interrupt_mask(Ir1Handle_Type* ir,uint32_t interrupt_source)    
{
    ms_ir1_interrupt_mask_hal(ir->instance, interrupt_source);
    return STATUS_SUCCESS;
}



/**
 * @brief register ir handler
 * @param  IrHandle_Type *ir   
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t  ms_ir1_register_handler(Ir1Handle_Type *ir)
{
    g_ir_handdle_ptr = ir;
    return STATUS_SUCCESS;
}

/**
 * @brief unregister ir handler
 * @param  none 
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t  ms_ir1_unregister_handler(void)    
{
    g_ir_handdle_ptr = NULL;
    return STATUS_SUCCESS;
}




void ir1_int_fifo_empty(Ir1Handle_Type *ir,uint8_t status)
{
   int32_t remain_len = 0;
   int32_t wr_cnt = 0;
   
  if(status&IR1_INT_STATUS_TX_FIFO_EMPTY_INT)
  {

	// MS_LOGI(MS_DRIVER, "\r\n 1 tx_total_len  = %x  tx_len = %x\n",ir->interrupt.tx_total_len,ir->interrupt.tx_len);
      remain_len = ir->interrupt.tx_total_len - ir->interrupt.tx_len;
	  
       if(remain_len>0)
        {

	      //MS_LOGI(MS_DRIVER, "\r\remain_len  = %x\n",remain_len);
            if( remain_len < IR1_TX_FIFO_LEN)
            {
                 remain_len =  remain_len;
            }
             else
            {
                  remain_len = IR1_TX_FIFO_LEN;
             }


            // MS_LOGI(MS_DRIVER, "\r\remain_len  = %x\n",remain_len);
            for(wr_cnt=0;wr_cnt<remain_len;wr_cnt++)
            {
 
		    ms_ir1_tx_level_cycle_config_hal(ir->instance,*(ir->interrupt.tx_ptr)++);
    	     }

	     ir->interrupt.tx_len += remain_len;

	 
           //	 MS_LOGI(MS_DRIVER, "\r\n 2 tx_total_len  = %x  tx_len = %x\n",ir->interrupt.tx_total_len,ir->interrupt.tx_len);
	
    }
     else
     {
          //  MS_LOGI(MS_DRIVER, "\r\ndisable ir controlle\n");
          //    MS_LOGI(MS_DRIVER, "\r\n 3 tx_total_len  = %x  tx_len = %x\n",ir->interrupt.tx_total_len,ir->interrupt.tx_len);
	       if( ir->interrupt.tx_len ==  ir->interrupt.tx_total_len)
	     {
                       /*mask Transmit FIFO Empty Interrupt*/
                   ms_ir1_interrupt_mask_hal(ir->instance, IR1_INT_STATUS_TX_FIFO_EMPTY_INT);

		
            }
       

     }
	 


  }


}




void ms_ir1_irq_handler(Ir1Handle_Type *ir) 
{

    CHECK_PTR_NULL(ir);

    	
   
    
    uint8_t status  = ms_ir1_get_interrupt_status_hal(ir->instance);
  
  //  MS_LOGI(MS_DRIVER, "\r\n ir addr = %x\n",&ir);
    //MS_LOGI(MS_DRIVER, "\r\nstatus = %x\n",status);
 
     if ((status & IR1_INT_STATUS_TX_DONE_INT) == IR1_INT_STATUS_TX_DONE_INT)
    {

            // MS_LOGI(MS_DRIVER, "\r\n x6\n");
	       /*disable spi controller */
	      ms_ir1_tx_disable_hal(ir->instance);
	      // ms_ir1_clear_status_hal(ir->instance,status&IR1_INT_STATUS_TX_DONE_INT);
	      /*mask Transmit FIFO Empty Interrupt*/
             ms_ir1_interrupt_mask_hal(ir->instance, IR1_INT_STATUS_TX_DONE_INT);
	     /*mask Transmit FIFO Empty Interrupt*/
              ms_ir1_interrupt_mask_hal(ir->instance, IR1_INT_STATUS_TX_FIFO_EMPTY_INT);
	       /*not busy*/
	      ir->interrupt.status = IR1_STATUS_IDLE;
	    //  MS_LOGI(MS_DRIVER, "\r\nir->interrupt.status(%x) = %x\n",&ir->interrupt.status ,ir->interrupt.status);
     	}
      else if ((status & IR1_INT_STATUS_TX_FIFO_EMPTY_INT) == IR1_INT_STATUS_TX_FIFO_EMPTY_INT)
    	{
	       ms_ir1_clear_status_hal(ir->instance,status&IR1_INT_STATUS_TX_FIFO_EMPTY_INT);
              //MS_LOGI(MS_DRIVER, "\r\n x5\n");
              ir1_int_fifo_empty(ir,IR1_INT_STATUS_TX_FIFO_EMPTY_INT);
    	}
    
 



    
    if (ir->p_callback == NULL) 
    {
        ir->error_code |= IR1_ERROR_INVALID_CALLBACK;
        return;
    }
	
    ir->p_callback->ir_reach_callback(ir);
   
}



void IR1_IRQHandler(void)
{
    if(g_ir_handdle_ptr != NULL)
    {
	 ms_ir1_irq_handler(g_ir_handdle_ptr);
    }
}


















