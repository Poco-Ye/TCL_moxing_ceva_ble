/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_gpio_hal.c
 * @brief c source  file of gpio  module.
 * @author haijun.mai
 * @date   2022-01-05
 * @version 1.0
 * @Revision
 */

#include <ms1008.h>
#include "ms_gpio_hal.h"
#include "ms_gpio.h"
#include "ms_gpio_regs.h"
#include "ms_clock_hal.h"
#include "ms_sys_ctrl_regs.h"
#include "ms_interrupt.h"
#include <stddef.h>




void ms_gpio_enable_cpu_interrupt(GpioHandle_Type *gpio)
{
	 INTERRUPT_ENABLE_IRQ(gpio->irq);
}



void ms_gpio_disable_cpu_interrupt(GpioHandle_Type *gpio)
{
	INTERRUPT_DISABLE_IRQ(gpio->irq);
}





void ms_gpio_init(GpioHandle_Type *gpio)
{
   if(NULL == gpio)
    {
        return ;
    }
    /*initial uart gpio pinmux, clock and interrupt setting*/
    if (gpio->p_callback && gpio->p_callback->init_callback)
    {
        gpio->p_callback->init_callback(gpio);
    }
	
    if(GPIO_MODE_IN_PUT == gpio->init.mode)
    {
    	ms_gpio_set_dir_in_put_hal(gpio->instance,gpio->init.pin);
    }
    else
    {
    	ms_gpio_set_dir_out_put_hal(gpio->instance,gpio->init.pin);
    }

}



void ms_gpio_set_out_put_high(GpioHandle_Type *gpio)
{
	 ms_gpio_out_put_high_hal(gpio->instance,gpio->init.pin);
}


void ms_gpio_set_out_put_low(GpioHandle_Type *gpio)
{
	 ms_gpio_out_put_low_hal(gpio->instance,gpio->init.pin);
}

uint8_t  ms_gpio_get_input_value(GpioHandle_Type *gpio)
{
	return ms_gpio_get_input_value_hal(gpio->instance,gpio->init.pin);
}

void ms_gpio_output_toggle(GpioHandle_Type *gpio)
{
	ms_gpio_output_toggle_hal(gpio->instance,gpio->init.pin);
}

void ms_gpio_interrupt_en(GpioHandle_Type *gpio)
{
	 ms_gpio_interrupt_en_hal(gpio->instance, gpio->init.pin);
}

void ms_gpio_interrupt_disen(GpioHandle_Type *gpio)
{
	 ms_gpio_interrupt_disen_hal(gpio->instance, gpio->init.pin);
}

void ms_gpio_interrupt_mask(GpioHandle_Type *gpio)
{
	 ms_gpio_interrupt_mask_hal(gpio->instance, gpio->init.pin);
}

void ms_gpio_interrupt_unmask(GpioHandle_Type *gpio)
{
	 ms_gpio_interrupt_unmask_hal(gpio->instance, gpio->init.pin);
}
void ms_gpio_set_interrupt_level(GpioHandle_Type *gpio)
{
	 ms_gpio_set_interrupt_level_hal(gpio->instance, gpio->init.pin,gpio->init.interrupt_level);
}


void ms_gpio_set_interrupt_polarity(GpioHandle_Type *gpio)
{
	 ms_gpio_set_interrupt_polarity_hal(gpio->instance, gpio->init.pin,gpio->init.interrupt_polarity);
}


void ms_gpio_interrupt_clear_status(GpioHandle_Type *gpio)
{
	 ms_gpio_interrupt_clear_status_hal(gpio->instance, gpio->init.pin);
}

uint8_t  ms_gpio_get_interrupt_raw_status(GpioHandle_Type *gpio)
{
	return ms_gpio_get_interrupt_raw_status_hal(gpio->instance,gpio->init.pin);
}

uint8_t  ms_gpio_get_interrupt_status()
{
	return ms_gpio_get_interrupt_status_hal();
}



void ms_gpio_irq_handler(GpioHandle_Type *gpio)
{
	
    if (gpio->p_callback == NULL)
    {
        gpio->error_code |= GPIO_ERROR_INVALID_CALLBACK;
        return;
    }
    gpio->p_callback->gpio_reach_callback(gpio);
}

int32_t ms_gpio_deinit(GpioHandle_Type *gpio)
{

  
   
    /* DeInit the low level hardware */
    if (gpio->p_callback && gpio->p_callback->deinit_callback)
    {
        gpio->p_callback->deinit_callback(gpio);
    }
    return 0;
}



