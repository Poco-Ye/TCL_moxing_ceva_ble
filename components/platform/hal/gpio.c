/*
 * gpio.c
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#include <ms_clock_hal.h>
#include "ms_gpio.h"
#include <stddef.h>
#include "ms_pinmux_hal.h"
#include "log.h"
#include "ms_sw_Keypad.h"
#define GPIO_MAX_CNT        26
GpioHandle_Type gpio_handle[GPIO_MAX_CNT] = {0};

void gpio_reach_calllback(GpioHandle_Type *gpio)
{
   uint8_t status = 0;

    status = ms_gpio_get_interrupt_status(gpio);
    ms_gpio_interrupt_clear_status(gpio);
    ms_sw_keypad_gpio_irq_handler(NULL);
}

void gpio_init_calllback(GpioHandle_Type *gpio)
{

    /*config the gpio clock*/
    MS_CLOCK_HAL_CLK_ENABLE_GPIO_PCLK();

    ms_pinmux_hal_set_pinmux(gpio->init.pin,0);
	
    // sys_ctrl_peripheral_clk_div_toggle(TIMER0_DIV_TOG);

    ms_gpio_interrupt_disen(gpio);

    ms_gpio_interrupt_clear_status(gpio);
	
    /*enable the interrupt*/
    //ms_gpio_enable_cpu_interrupt(gpio);

    ms_gpio_set_interrupt_level(gpio);
   
    ms_gpio_set_interrupt_polarity(gpio);

    ms_gpio_interrupt_en(gpio);

    ms_gpio_interrupt_unmask(gpio);
}  

void gpio_deinit_calllback(GpioHandle_Type *gpio)
{
    /*disable the interrupt*/
    ms_gpio_disable_cpu_interrupt(gpio);

    /*config the gpio clock*/
    MS_CLOCK_HAL_CLK_DISABLE_GPIO_PCLK();
}

GpioCallback_Type gpio_callback =
{
        .init_callback = gpio_init_calllback,
        .deinit_callback = gpio_deinit_calllback,
        .gpio_reach_callback = gpio_reach_calllback
};

int32_t gpio_input_init(int32_t pin)
{

    if(pin>GPIO_MAX_CNT)
    {
         return STATUS_ERROR;
    }
    gpio_handle[pin].instance = GPIO;
    gpio_handle[pin].init.mode = GPIO_MODE_IN_PUT;
    gpio_handle[pin].init.pin = pin;
    gpio_handle[pin].p_callback = &gpio_callback;
    gpio_handle[pin].irq = Reserved0_IRQn;
    gpio_handle[pin].init.interrupt_level = GPIO_INTERRUPT_EDGE_SENSITIVE;
    gpio_handle[pin].init.interrupt_polarity = GPIO_INTERRUPT_ACTIVE_LOW_POLARITY;
	

    ms_gpio_init(&gpio_handle[pin]);
    return STATUS_SUCCESS;
}


int32_t gpio_out_init(int32_t pin)
{

    if(pin>GPIO_MAX_CNT)
    {
         return STATUS_ERROR;
    }
    gpio_handle[pin].instance = GPIO;
    gpio_handle[pin].init.mode = GPIO_MODE_OUT_PUT;
    gpio_handle[pin].init.pin = pin;
    gpio_handle[pin].p_callback = &gpio_callback;
    gpio_handle[pin].irq = Reserved0_IRQn;
    gpio_handle[pin].init.interrupt_level = GPIO_INTERRUPT_EDGE_SENSITIVE;
    gpio_handle[pin].init.interrupt_polarity = GPIO_INTERRUPT_ACTIVE_LOW_POLARITY;
	

    ms_gpio_init(&gpio_handle[pin]);
    return STATUS_SUCCESS;
}

int32_t gpio_enter_deep_sleep_callback(uint8_t pin, uint8_t dir, uint8_t level)
{
	
    if(GPIO26_INDEX < pin)
    {
        return STATUS_ERROR;
    }
    gpio_handle[pin].instance = GPIO;
    gpio_handle[pin].init.mode = dir;
    gpio_handle[pin].init.pin = pin;
    gpio_handle[pin].irq = GPIO_IRQn;
    gpio_handle[pin].p_callback = &gpio_callback;
    gpio_handle[pin].init.interrupt_level = GPIO_INTERRUPT_EDGE_SENSITIVE;
    gpio_handle[pin].init.interrupt_polarity = GPIO_INTERRUPT_ACTIVE_LOW_POLARITY;
  //  gpio_handle[pin].init.interrupt_level = GPIO_INTERRUPT_EDGE_SENSITIVE;
  //  gpio_handle[pin].init.interrupt_polarity = GPIO_INTERRUPT_ACTIVE_HIGH_POLARITY; 
    ms_gpio_init(&gpio_handle[pin]);

    if(GPIO_MODE_OUT_PUT == gpio_handle[pin].init.mode)
    {
        if(level)
        {
            ms_gpio_set_out_put_high(&gpio_handle[pin]);
        }
        else
        {
            ms_gpio_set_out_put_low(&gpio_handle[pin]);
        }
    }
    else
    {
        ms_gpio_disable_cpu_interrupt(&gpio_handle[pin]);
    }
	
    return STATUS_SUCCESS;
}

int32_t gpio_falling_edge_interrput_init(int32_t pin)
{

    if(pin>8)
    {
         return STATUS_ERROR;
    }
    gpio_handle[pin].instance = GPIO;
    gpio_handle[pin].init.mode = GPIO_MODE_IN_PUT;
    gpio_handle[pin].init.pin = pin;
    gpio_handle[pin].p_callback = &gpio_callback;
    gpio_handle[pin].irq = GPIO_IRQn;
    gpio_handle[pin].init.interrupt_level = GPIO_INTERRUPT_EDGE_SENSITIVE;
    gpio_handle[pin].init.interrupt_polarity = GPIO_INTERRUPT_ACTIVE_LOW_POLARITY;
	

    ms_gpio_init(&gpio_handle[pin]);
    return STATUS_SUCCESS;
}

int32_t gpio_rising_edge_interrput_init(int32_t pin)
{

    if(pin>8)
    {
         return STATUS_ERROR;
    }
    gpio_handle[pin].instance = GPIO;
    gpio_handle[pin].init.mode = GPIO_MODE_IN_PUT;
    gpio_handle[pin].init.pin = pin;
    gpio_handle[pin].p_callback = &gpio_callback;
    gpio_handle[pin].irq = GPIO_IRQn;
    gpio_handle[pin].init.interrupt_level = GPIO_INTERRUPT_EDGE_SENSITIVE;
    gpio_handle[pin].init.interrupt_polarity = GPIO_INTERRUPT_ACTIVE_HIGH_POLARITY;
	

    ms_gpio_init(&gpio_handle[pin]);
    return STATUS_SUCCESS;
}


int32_t gpio_low_level_interrput_init(int32_t pin)
{

    if(pin>8)
    {
         return STATUS_ERROR;
    }
    gpio_handle[pin].instance = GPIO;
    gpio_handle[pin].init.mode = GPIO_MODE_IN_PUT;
    gpio_handle[pin].init.pin = pin;
    gpio_handle[pin].p_callback = &gpio_callback;
    gpio_handle[pin].irq = GPIO_IRQn;
    gpio_handle[pin].init.interrupt_level = GPIO_INTERRUPT_LEVEL_SENSITIVE;
    gpio_handle[pin].init.interrupt_polarity = GPIO_INTERRUPT_ACTIVE_LOW_POLARITY;
	

    ms_gpio_init(&gpio_handle[pin]);
    return STATUS_SUCCESS;
}


int32_t gpio_high_level_interrput_init(int32_t pin)
{

    if(pin>8)
    {
         return STATUS_ERROR;
    }
    gpio_handle[pin].instance = GPIO;
    gpio_handle[pin].init.mode = GPIO_MODE_IN_PUT;
    gpio_handle[pin].init.pin = pin;
    gpio_handle[pin].p_callback = &gpio_callback;
    gpio_handle[pin].irq = GPIO_IRQn;
    gpio_handle[pin].init.interrupt_level = GPIO_INTERRUPT_LEVEL_SENSITIVE;
    gpio_handle[pin].init.interrupt_polarity = GPIO_INTERRUPT_ACTIVE_HIGH_POLARITY;
	

    ms_gpio_init(&gpio_handle[pin]);
    return STATUS_SUCCESS;
}


 int32_t  gpio_out_put_high_level(int32_t pin)
{
       if(pin>GPIO_MAX_CNT)
       {
         return STATUS_ERROR;
        }
	ms_gpio_set_out_put_high(&gpio_handle[pin]);
	return STATUS_SUCCESS;
}

int32_t  gpio_out_put_low_level(int32_t pin)
{
       if(pin>GPIO_MAX_CNT)
       {
         return STATUS_ERROR;
        }
	ms_gpio_set_out_put_low(&gpio_handle[pin]);
       return STATUS_SUCCESS;
}


uint8_t gpio_get_value(int32_t pin)
{
    if(pin>GPIO_MAX_CNT)
    {
        return STATUS_ERROR;
    }
    return ms_gpio_get_input_value(&gpio_handle[pin]);
}

int32_t  gpio_output_toggle(int32_t pin)
{
    if(pin>GPIO_MAX_CNT)
    {
        return STATUS_ERROR;
    }
    ms_gpio_output_toggle(&gpio_handle[pin]);
    return STATUS_SUCCESS;
}


int32_t gpio_deinit(int32_t pin)
{
    if(pin>GPIO_MAX_CNT)
    {
         return STATUS_ERROR;
    }
    ms_gpio_deinit(&gpio_handle[pin]);
    return STATUS_SUCCESS;
}

void GPIO_IRQHandler(void)
{
    uint8_t interrupt_status = 0;
    uint8_t i = 0;
	
    interrupt_status = ms_gpio_get_interrupt_status();
   for(i=0;i<MS_GPIO_PORT_LEN;i++)
   {
           if((interrupt_status>>i)&0x1)
           {
                  ms_gpio_irq_handler(&gpio_handle[i]);
	           /*clear interrupt status*/
		    ms_gpio_interrupt_clear_status(&gpio_handle[i]);

		    //MS_LOGI( MS_DRIVER, "pin=%d\r\n",i);
		    break;
	    }
   }

}

