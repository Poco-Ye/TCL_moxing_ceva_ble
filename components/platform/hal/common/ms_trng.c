/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file  ms_trng_hal.c
 * @brief
 * @author haijun.mai
 * @date 2022-01-06
 * @version 1.0
 * @Revision
 */

#include <ms1008.h>
#include "ms_trng_hal.h"
#include "ms_trng.h"
#include "ms_trng_regs.h"
#include "ms_clock_hal.h"
#include "ms_sys_ctrl_regs.h"
#include "ms_interrupt.h"
#include <stddef.h>

/**
 * @brief  Enable Trng Interrupt 
 * @param  TrngHandle_Type *trng: TRNG Instance
 * @retval none
 */
void ms_trng_enable_cpu_interrupt(TrngHandle_Type *trng)
{
    INTERRUPT_ENABLE_IRQ(trng->irq);
}

/**
 * @brief  Disable Trng Interrupt 
 * @param  TrngHandle_Type *trng: TRNG Instance
 * @retval none
 */
void ms_trng_disable_cpu_interrupt(TrngHandle_Type *trng)
{
    INTERRUPT_DISABLE_IRQ(trng->irq);
}


/**
 * @brief  mask trng interrupt
 * @param  Trng_Type *trng: TRNG regs
 * @param  uint32_t interrupt_mask: 
 * @retval none
 */
void ms_trng_interrupt_mask(TrngHandle_Type *trng,uint32_t interrupt_mask)
{
     ms_trng_interrupt_mask_hal(trng->instance,interrupt_mask);
}


/**
 * @brief  unmask trng interrupt
 * @param  Trng_Type *trng: TRNG regs
 * @param  uint32_t interrupt_mask: 
 * @retval none
 */
void ms_trng_interrupt_unmask(TrngHandle_Type *trng,uint32_t interrupt_mask)    
{
	 ms_trng_interrupt_unmask_hal(trng->instance,interrupt_mask);
}



/**
 * @brief Trng Init 
 * @param  TrngHandle_Type *trng: TRNG Instance
 * @retval SUCCESS (0) or ERROR(-1)
 */
uint32_t ms_trng_init(TrngHandle_Type *trng)
{
      if(NULL == trng)
    {
        return -1;
    }
    /*initial uart gpio pinmux, clock and interrupt setting*/
    if (trng->p_callback && trng->p_callback->init_callback)
    {
        trng->p_callback->init_callback(trng);
    }
	

    /*set trng sample rate*/
    ms_trng_set_sample_rate_hal(trng->instance,trng->init.sample_rate);

    /*select trng src*/	
    ms_trng_rnd_src_sel_hal(trng->instance,trng->init.rnd_src_sel);

     /*set trng debug control*/	
    ms_trng_set_debug_control_hal(trng->instance,trng->init.debug_control_sel);

    return 0;
	 
}



/**
 * @brief Trng Get Random data 
 * @param  TrngHandle_Type *trng: TRNG Instance
 * @retval data0-data6
 */
uint32_t ms_trng_get_data(TrngHandle_Type *trng)
{
    uint32_t status  = 0;
    uint32_t re_val  = 0;
	
    /*enable trng*/
    ms_trng_source_enablel_hal(trng->instance);

    /*wait data valid*/
    do
    {
        status = ms_trng_get_data_valid_status_hal(trng->instance);
    }
    while((status&TRNG_VALID_EHR_VALID) != TRNG_VALID_EHR_VALID);


        /*get  data status*/
    do
    {
        status = ms_trng_get_interrupt_status_hal(trng->instance,TRNG_ISR_EHR_VALID);
    }
    while(!status);
	
    /*get data*/
    re_val =   ms_trng_get_data_hal(trng->instance,trng->init.data_x);

   //clear interrupt status
   ms_trng_clear_interrupt_status_hal(trng->instance,TRNG_ICR_EHR_VALID);

     /*disable trng*/
    ms_trng_source_disablel_hal(trng->instance);	
    return re_val;
}


/**
 * @brief Trng Deinit 
 * @param  TrngHandle_Type *trng: TRNG Instance
 * @retval none
 */
void ms_trng_deinit(TrngHandle_Type *trng)
{
       /* DeInit the low level hardware */
    if (trng->p_callback && trng->p_callback->deinit_callback)
    {
        trng->p_callback->deinit_callback(trng);
    }
}


/**
 * @brief  This function handles trng interrupt request.
 * @param  param  TrngHandle_Type *trng: TRNG Instance
 * @retval None
 */
void ms_trng_irq_handler(TrngHandle_Type *trng)
{
	// timer_inter_clear(tim->instance); //clear irq
    if (trng->p_callback == NULL)
    {
        trng->error_code |= TRNG_ERROR_INVALID_CALLBACK;
        return;
    }
    trng->p_callback->trng_reach_callback(trng);
}

void ms_trng_clear_interrupt_status(TrngHandle_Type *trng,uint32_t interrupt_clear)
{
	ms_trng_clear_interrupt_status_hal(trng->instance, interrupt_clear);
}



