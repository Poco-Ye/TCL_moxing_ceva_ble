/*
 * trng.c
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#include <ms_clock_hal.h>
#include "ms_trng.h"
#include <stddef.h>

TrngHandle_Type trng_handle;


void trng_reach_calllback(TrngHandle_Type *trng)
{
   ms_trng_clear_interrupt_status(trng,TRNG_ICR_EHR_VALID);
}



void trng_init_calllback(TrngHandle_Type *trng)
{
    /*config the trng clock*/
   MS_CLOCK_HAL_CLK_ENABLE_TRNG();
   

    /*enable the interrupt*/
    //ms_trng_enable_cpu_interrupt(trng);
    ms_trng_interrupt_unmask(trng,TRNG_IMR_EGR_VALID_INT);
}

void trng_deinit_calllback(TrngHandle_Type *trng)
{
    /*disable the interrupt*/
    ms_trng_interrupt_mask(trng,TRNG_IMR_EGR_VALID_INT);
    ms_trng_disable_cpu_interrupt(trng);

    /*config the trng clock*/
    MS_CLOCK_HAL_CLK_DISABLE_TRNG();
}

TrngCallback_Type  trng_handle_callback =
{
       .init_callback = trng_init_calllback,
	.deinit_callback = trng_deinit_calllback,
	.trng_reach_callback = trng_reach_calllback,
};

uint32_t  trng_init(void)
{
    trng_handle.instance = TRNG;
    trng_handle.init.sample_rate = 0x20;
    trng_handle.init.debug_control_sel = RND_SRC_SEL2;
    //trng_handle.init.rnd_src_sel = (TRNG_TDC_VNC_BYPASS|TRNG_TDC_TRNG_CRNGT_BYPASS|TRNG_TDC_AUTO_CORRELATE_BYPASS);
	//   trng_handle.init.rnd_src_sel = 0xffffffff;
	
    trng_handle.p_callback = &trng_handle_callback;
    trng_handle.irq = TRNG_IRQn;
	
    ms_trng_init(&trng_handle);

    return STATUS_SUCCESS;
}


uint32_t trng_get_data(void)
{
    trng_handle.init.data_x = TRNG_DATA0_SEL ;
    return  ms_trng_get_data(&trng_handle);
}


uint32_t trng_deinit(void)
{
	ms_trng_deinit(&trng_handle);
	 return STATUS_SUCCESS;
}

void TRNG_IRQHandler(void)
{
    ms_trng_irq_handler(&trng_handle);
}
