/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_gpio_ll.c
 * @brief c source  file of gpio  module.
 * @author haijun.mai
 * @date   2022-01-05
 * @version 1.0
 * @Revision
 */

#include "ms_gpio_ll.h"

void ms_gpio_set_dir_out_put_ll(Gpio_Type *gpio, uint8_t pin)
{
    SET_BIT(gpio->PORTX[pin /MS_GPIO_PORT_LEN].PXDDR,(0x1UL<< (pin % MS_GPIO_PORT_LEN)));
}

void ms_gpio_set_dir_in_put_ll(Gpio_Type *gpio, uint8_t pin)
{
    CLEAR_BIT(gpio->PORTX[pin /MS_GPIO_PORT_LEN].PXDDR,(0x1UL<< (pin % MS_GPIO_PORT_LEN)));
}

void ms_gpio_out_put_high_ll(Gpio_Type *gpio, uint8_t pin)
{
    SET_BIT(gpio->PORTX[pin /MS_GPIO_PORT_LEN].PXDR,(0x1UL<< (pin % MS_GPIO_PORT_LEN)));
}

void ms_gpio_out_put_low_ll(Gpio_Type *gpio, uint8_t pin)
{
    CLEAR_BIT(gpio->PORTX[pin /MS_GPIO_PORT_LEN].PXDR, (0x1UL<<(pin % MS_GPIO_PORT_LEN)));
}

uint8_t ms_gpio_get_input_value_ll(Gpio_Type *gpio, uint8_t pin)
{

    return (READ_REG(gpio->EXTPX[pin /MS_GPIO_PORT_LEN])&(0x1UL<<(pin % MS_GPIO_PORT_LEN)))>>(pin % MS_GPIO_PORT_LEN);

}

void ms_gpio_output_toggle_ll(Gpio_Type *gpio, uint8_t pin)
{
    uint8_t value = 0;

    value = READ_REG(gpio->EXTPX[pin /MS_GPIO_PORT_LEN]);
    if (value & (pin %MS_GPIO_PORT_LEN))
    {
        CLEAR_BIT(gpio->PORTX[pin /MS_GPIO_PORT_LEN].PXDR,(0x1UL<< (pin %MS_GPIO_PORT_LEN)));
    }
    else
    {
        SET_BIT(gpio->PORTX[pin /MS_GPIO_PORT_LEN].PXDR,(0x1UL<< (pin %MS_GPIO_PORT_LEN)));
    }

}

