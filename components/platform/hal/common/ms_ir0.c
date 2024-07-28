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
#include "ms_ir0_hal.h"
#include "ms_ir0.h"
#include "ms_ir0_regs.h"
#include "ms_clock_hal.h"
#include "ms_interrupt.h"
#include <stddef.h>
#include "log.h"

#ifdef IR1_TO2
#include "ms_sys_wild_hal.h"
#endif

Ir0Handle_Type *g_ir0_handdle_ptr = NULL;

/**
 * @brief enable ir interrupt
 * @param  Ir_Type *ir: 
 * @retval None
 */
int32_t ms_ir0_enable_cpu_interrupt(Ir0Handle_Type *ir) 
{
       CHECK_PTR_NULL_RET(ir,STATUS_ERROR);
	INTERRUPT_ENABLE_IRQ(ir->irq);
	 return STATUS_SUCCESS;
}

/**
 * @brief disable ir interrupt
 * @param  Ir_Type *ir: 
 * @retval None
 */
int32_t ms_ir0_disable_cpu_interrupt(Ir0Handle_Type *ir) 
{
      CHECK_PTR_NULL_RET(ir,STATUS_ERROR);
	INTERRUPT_DISABLE_IRQ(ir->irq);
	 return STATUS_SUCCESS;
}


/**
 * @brief ir module init
 * @param  Ir0Handle_Type *ir: 
 * @retval None
 */
int32_t ms_ir0_init(Ir0Handle_Type *ir)
{
	CHECK_PTR_NULL_RET(ir,STATUS_ERROR);
#ifdef IR1_TO2
      MS_SYS_HAL_ENABLE_IR0();
#endif
	/*initial uart gpio pinmux, clock and interrupt setting*/
	if (ir->p_callback && ir->p_callback->init_callback) {
		ir->p_callback->init_callback(ir);
	}

    
    /*select tx proto mode*/
    ms_ir0_config_proto_mode_hal(ir->instance, ir->init.proto_mode);

    /*config tx carry high cycle cnt*/
    ms_ir0_tx_carry_high_cycle_config_hal(ir->instance,ir->init.carry_high_cycle);
 
    /*config tx carry high cycle cnt*/
    ms_ir0_tx_carry_low_cycle_config_hal(ir->instance,ir->init.carry_low_cycle);

    /*config preamble carry cnt*/
    ms_ir0_tx_config_preamble_carry_cnt_hal(ir->instance, ir->init.preamble_carry_cnt);

    /*config preamble idle cnt*/
    ms_ir0_tx_config_preamble_idle_cnt_hal(ir->instance, ir->init.preamble_idle_cnt);
  
    /*preamble enable */
    ms_ir0_tx_preamble_enable_hal(ir->instance);
    
    /*config data0 carry cnt*/
    ms_ir0_tx_config_data0_carry_cnt_hal(ir->instance,ir->init.data0_carry_cnt);

    /*config data0 idle cnt*/
    ms_ir0_tx_config_data0_idle_cnt_hal(ir->instance,ir->init.data0_idle_cnt);

    /*config data1 carry cnt*/
    ms_ir0_tx_config_data1_carry_cnt_hal(ir->instance,ir->init.data1_carry_cnt);

    /*config data1 idle cnt*/
    ms_ir0_tx_config_data1_idle_cnt_hal(ir->instance,ir->init.data1_idle_cnt);
 
    /*data enable */
    ms_ir0_tx_data_enable_hal(ir->instance);
    
    /*config end carry cnt*/
    ms_ir0_tx_config_end_carry_cnt_hal(ir->instance,ir->init.end_carry_cnt);

    /*config end idle cnt*/
    ms_ir0_tx_config_end_idle_cnt_hal(ir->instance,ir->init.end_idle_cnt);
     
    /*end enable */
    ms_ir0_tx_end_enable_hal(ir->instance);
    
    /*config repeat cycle cnt*/
    ms_ir0_tx_config_repeat_cycle_hal(ir->instance,ir->init.repeat_carry_cnt);

  
   /*interrupt disable*/
  // ms_ir0_tx_interrupt_disable_hal(ir->instance);

   /*tx_abort enable*/
    ms_ir0_tx_abort_enable_hal(ir->instance);
    //ms_ir0_tx_repeat_enable_hal(ir->instance); 

#if 0
     MS_LOGI(MS_DRIVER, "\r\n IR0_CTL(%x) = %x \n",&ir->instance->IR0_CTL,ir->instance->IR0_CTL);
     MS_LOGI(MS_DRIVER, "\r\n IR0_TX_CTL(%x) = %x \n",&ir->instance->IR0_TX_CTL,ir->instance->IR0_TX_CTL);
     MS_LOGI(MS_DRIVER, "\r\n IR0_TX_CARRY_CFG = %x \n",ir->instance->IR0_TX_CARRY_CFG);
     MS_LOGI(MS_DRIVER, "\r\n IR0_TX_PREAMBLE_CFG = %x \n",ir->instance->IR0_TX_PREAMBLE_CFG);
     MS_LOGI(MS_DRIVER, "\r\n IR0_TX_DATA0_CFG = %x \n",ir->instance->IR0_TX_DATA0_CFG);
     MS_LOGI(MS_DRIVER, "\r\n IR0_TX_DATA1_CFG = %x \n",ir->instance->IR0_TX_DATA1_CFG);
     MS_LOGI(MS_DRIVER, "\r\n IR0_TX_END_CFG = %x \n",ir->instance->IR0_TX_END_CFG);
     MS_LOGI(MS_DRIVER, "\r\n IR0_TX_REPEAT_CFG = %x \n",ir->instance->IR0_TX_REPEAT_CFG);
     MS_LOGI(MS_DRIVER, "\r\n IR0_TX_DATA = %x \n",ir->instance->IR0_TX_DATA);
     MS_LOGI(MS_DRIVER, "\r\n IR0_TX_STATUS = %x \n",ir->instance->IR0_TX_STATUS);
#endif 
    return STATUS_SUCCESS;

}







/**
 * @brief   Ir Controller send data
 * @param[in]  Ir0Handle_Type *ir:  Pointer to a IrHandle_Type  structure that contains
 * @param[in]  uint32_t tx_data :the data that will be send by ir controller
 * @param[in]  uint32_t tx_data :the data's size(bit) that will be send by ir controller
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_ir0_send_data(Ir0Handle_Type *ir,uint32_t tx_data, uint32_t size) 
{
     CHECK_PTR_NULL_RET(ir,STATUS_ERROR);

 
     int reg_value = 0;
  
		
    /*set tx data len*/
     ms_ir0_set_tx_num_bit_hal(ir->instance,size);
 
     ms_ir0_tx_data_fill_hal( ir->instance, tx_data);


    /*enable tx */
    ms_ir0_tx_enable_hal(ir->instance);


   #if 1
    do {
        reg_value =(IR0_TX_STATUS_TX_BUSY&ms_ir0_get_status_hal(ir->instance));
	//	MS_LOGI(MS_DRIVER, "b= %x\n",reg_value );
    } while(reg_value);
#endif
#if 1
   
    do {
        reg_value =(IR0_TX_STATUS_TX_DONE&ms_ir0_get_status_hal(ir->instance));
		  //MS_LOGI(MS_DRIVER, "\r\n reg_value = %x \n",reg_value);
		//MS_LOGI(MS_DRIVER, "r= %x\n",reg_value );
    } while(!reg_value);
#endif

   ms_ir0_clear_tx_done_status_hal( ir->instance);
    
    /*disable tx*/
    ms_ir0_tx_disable_hal(ir->instance);
	
     return STATUS_SUCCESS;
}




/**
 * @brief   Ir Controller repeat send data
 * @param[in]  Ir0Handle_Type *ir:  Pointer to a IrHandle_Type  structure that contains
 * @param[in]  uint32_t tx_data :the data that will be repeat send by ir controller
 * @param[in]  uint32_t tx_data :the data's size(bit) that will be repeat  send by ir controller
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_ir0_send_data_repeat(Ir0Handle_Type *ir,uint32_t tx_data,uint32_t size) 
{
     CHECK_PTR_NULL_RET(ir,STATUS_ERROR);


    
    /*set tx data len*/
     ms_ir0_set_tx_num_bit_hal(ir->instance,size);
	
     ms_ir0_tx_data_fill_hal( ir->instance, tx_data);

     /*repeat enable*/
    ms_ir0_tx_repeat_enable_hal(ir->instance);

     /*enable tx */
    ms_ir0_tx_enable_hal(ir->instance);
    
   
    return STATUS_SUCCESS;
}


/**
 * @brief ir module stop repeat send data
 * @param  Ir0Handle_Type *ir: 
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_ir0_send_data_repeat_stop(Ir0Handle_Type *ir) 
{
    CHECK_PTR_NULL_RET(ir,STATUS_ERROR);

    /*disable tx*/
    ms_ir0_tx_disable_hal(ir->instance);

    /*repeat enable*/
    ms_ir0_tx_repeat_disable_hal(ir->instance);
    
    return STATUS_SUCCESS;
}


/**
 * @brief   Ir interrupt send data
 * @param[in]  Ir0Handle_Type *ir:  Pointer to a IrHandle_Type  structure that contains
 * @param[in]  uint32_t tx_data :the data that will be send by ir controller
 * @param[in]  uint32_t tx_data :the data's size(bit) that will be send by ir controller
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_ir0_int_send_data(Ir0Handle_Type *ir,uint32_t tx_data, uint32_t size) 
{
     CHECK_PTR_NULL_RET(ir,STATUS_ERROR);


     int reg_value = 0;
     
		
    /*set tx data len*/
     ms_ir0_set_tx_num_bit_hal(ir->instance,size);
	
		
     ms_ir0_tx_data_fill_hal( ir->instance, tx_data);

		

    /*enable tx */
    ms_ir0_tx_enable_hal(ir->instance);

		
    ir->interrupt.status = IR0_STATUS_BUSY;

   /*unmask ir interrupt*/
   ms_ir0_tx_interrupt_enable_hal(ir->instance);

		
    while( ir->interrupt.status == IR0_STATUS_BUSY);
  
		
    /*disable tx*/
    ms_ir0_tx_disable_hal(ir->instance);

		
   /*mask ir interrupt*/
  ms_ir0_tx_interrupt_disable_hal(ir->instance);
   
		
     return STATUS_SUCCESS;
}




/**
 * @brief  Deinitial Ir Controller
 * @param[in]  IrHandle_Type *ir:  Pointer to a IrHandle_Type  structure that contains
 * @ the configuration information for the specified Ir module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_ir0_deinit(Ir0Handle_Type *ir)
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
int32_t  ms_ir0_interrupt_unmask(Ir0Handle_Type* ir)    
{
    CHECK_PTR_NULL_RET(ir,STATUS_ERROR);
    ms_ir0_tx_interrupt_enable_hal(ir->instance);
    return STATUS_SUCCESS;
}




/**
 * @brief mask ir interrupt
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t interrupt_source
 * @retval None
 */
int32_t  ms_ir0_interrupt_mask(Ir0Handle_Type* ir)    
{
     CHECK_PTR_NULL_RET(ir,STATUS_ERROR);
     ms_ir0_tx_interrupt_disable_hal(ir->instance);
    return STATUS_SUCCESS;
}



/**
 * @brief register ir handler
 * @param  IrHandle_Type *ir   
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t  ms_ir0_register_handler(Ir0Handle_Type *ir)
{
    CHECK_PTR_NULL_RET(ir,STATUS_ERROR);
    g_ir0_handdle_ptr = ir;
    return STATUS_SUCCESS;
}

/**
 * @brief unregister ir handler
 * @param  none 
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t  ms_ir0_unregister_handler(void)    
{
    g_ir0_handdle_ptr = NULL;
    return STATUS_SUCCESS;
}




void ms_ir0_irq_handler(Ir0Handle_Type *ir) 
{

    CHECK_PTR_NULL(ir);


    uint8_t status  = ms_ir0_get_status_hal(ir->instance);
  

   // MS_LOGI(MS_DRIVER, "\r\nstatus = %x\n",status);
 
     if ((status & IR0_TX_STATUS_TX_DONE) == IR0_TX_STATUS_TX_DONE)
    {
              ms_ir0_clear_tx_done_status_hal( ir->instance);
                // MS_LOGI(MS_DRIVER, "\r\n clear int\n");
	 
	       /*not busy*/
	      ir->interrupt.status = IR0_STATUS_IDLE;
	  
     	}
    

    if (ir->p_callback == NULL) 
    {
        ir->error_code |= IR0_ERROR_INVALID_CALLBACK;
        return;
    }
	
    ir->p_callback->ir_reach_callback(ir);
   
}



void IR_IRQHandler(void)
{
    if(g_ir0_handdle_ptr != NULL)
    {
	 ms_ir0_irq_handler(g_ir0_handdle_ptr);
    }
}




