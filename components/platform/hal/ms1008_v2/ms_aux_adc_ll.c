
/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_aux_adc_ll.c
 * @brief c source  file of aux_adc  module.
 * @author haijun.mai
 * @date   2021-01-24
 * @version 1.0
 * @Revision
 */
#include <ms_trng_ll.h>
#include <ms_clock_hal.h>

#include "ms_aux_adc_ll.h"




/**
 * @brief  aux adc set channel queue
 * @param AuxAdc_Type *aux_adc: aux_adc regs
 * @param uint8_t idx   
 * @param This parameter can be one of the following values:
 * @param  MS_AUX_ADC_Q0
 * @param  MS_AUX_ADC_Q1
 * @param  MS_AUX_ADC_Q2
 * @param  MS_AUX_ADC_Q3
 * @param  MS_AUX_ADC_Q4
 * @param  MS_AUX_ADC_Q5
 * @param  MS_AUX_ADC_Q6
 * @param  MS_AUX_ADC_Q7
 * @param uint8_t channel : channel is 0~7   
 * @retval None
 */
 void  ms_aux_adc_set_channel_queue_ll(AuxAdc_Type *aux_adc,uint8_t idx,uint8_t channel)
{
    int reg_value = 0;
	
    reg_value = READ_REG(aux_adc->CHNQUEUE);

     switch (idx)
    {
        case MS_AUX_ADC_Q0: 
	{
	    CLEAR_BIT(reg_value,AUX_ADC_CHNQUEUE_CH0_QUEUE);
	    SET_BIT(reg_value,((channel<<AUX_ADC_CHNQUEUE_CH0_QUEUE_POS)&AUX_ADC_CHNQUEUE_CH0_QUEUE_MASK));
           break;
        }
        case MS_AUX_ADC_Q1:
	{
           CLEAR_BIT(reg_value,AUX_ADC_CHNQUEUE_CH1_QUEUE);
	    SET_BIT(reg_value,((channel<<AUX_ADC_CHNQUEUE_CH1_QUEUE_POS)&AUX_ADC_CHNQUEUE_CH1_QUEUE_MASK));
           break;
        }
        case MS_AUX_ADC_Q2:
	{
           CLEAR_BIT(reg_value,AUX_ADC_CHNQUEUE_CH2_QUEUE);
	    SET_BIT(reg_value,((channel<<AUX_ADC_CHNQUEUE_CH2_QUEUE_POS )&AUX_ADC_CHNQUEUE_CH2_QUEUE_MASK));
           break;
        }
        case MS_AUX_ADC_Q3: 
	{
            CLEAR_BIT(reg_value,AUX_ADC_CHNQUEUE_CH3_QUEUE);
	     SET_BIT(reg_value,((channel<<AUX_ADC_CHNQUEUE_CH3_QUEUE_POS )&AUX_ADC_CHNQUEUE_CH3_QUEUE_MASK));
            break;
        }
        case MS_AUX_ADC_Q4: 
	{
           CLEAR_BIT(reg_value,AUX_ADC_CHNQUEUE_CH4_QUEUE);
	    SET_BIT(reg_value,((channel<<AUX_ADC_CHNQUEUE_CH4_QUEUE_POS )&AUX_ADC_CHNQUEUE_CH4_QUEUE_MASK));
           break;
        }
        case MS_AUX_ADC_Q5: 
	{
           CLEAR_BIT(reg_value,AUX_ADC_CHNQUEUE_CH5_QUEUE);
	    SET_BIT(reg_value,((channel<<AUX_ADC_CHNQUEUE_CH5_QUEUE_POS )&AUX_ADC_CHNQUEUE_CH5_QUEUE_MASK));
           break;
        }
        case MS_AUX_ADC_Q6: 
	{
           CLEAR_BIT(reg_value,AUX_ADC_CHNQUEUE_CH6_QUEUE);
	    SET_BIT(reg_value,((channel<<AUX_ADC_CHNQUEUE_CH6_QUEUE_POS )&AUX_ADC_CHNQUEUE_CH6_QUEUE_MASK));
           break;
        }
        case MS_AUX_ADC_Q7: 
	{
           CLEAR_BIT(reg_value,AUX_ADC_CHNQUEUE_CH7_QUEUE);
	    SET_BIT(reg_value,((channel<<AUX_ADC_CHNQUEUE_CH7_QUEUE_POS )&AUX_ADC_CHNQUEUE_CH7_QUEUE_MASK));
           break;
        }
        default:
   
           ;
   }

    WRITE_REG(aux_adc->CHNQUEUE, reg_value);
	
 
}


