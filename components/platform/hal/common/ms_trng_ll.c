/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_trngl_ll.c
 * @brief c source  file of TRNG  module.
 * @author haijun.mai
 * @date   2021-01-06
 * @version 1.0
 * @Revision
 */
#include <ms_trng_ll.h>
#include "ms_clock_hal.h"


/**
 * @brief  Trng get random data
 * @param  Trng_Type *trng: TRNG regs
 * @param  uint8_t data_x: select the random data ,data_x is 0-6 
 * @retval the random data
 */
uint32_t ms_trng_get_data_ll(Trng_Type *trng,uint8_t data_x)
{
    uint32_t value = 0;

    if(TRNG_DATA0_SEL == data_x)
    {
        value = READ_REG(trng->DATA0);
    }
    else if(TRNG_DATA1_SEL == data_x)
   {
         value = READ_REG(trng->DATA1);
    }
    else if(TRNG_DATA2_SEL == data_x)
   {
         value = READ_REG(trng->DATA2);
    }
    else if(TRNG_DATA3_SEL == data_x)
   {
         value = READ_REG(trng->DATA3);
    } 
    else if(TRNG_DATA4_SEL == data_x)
   {
         value = READ_REG(trng->DATA4);
    }
    else if(TRNG_DATA5_SEL == data_x)
   {
         value = READ_REG(trng->DATA5);
    }
    else 
   {
          value = READ_REG(trng->DATA0);
    }
    return value;   	
}





