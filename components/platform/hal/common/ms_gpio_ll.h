/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_gpio_ll.h
 * @brief Header file of gpio  module.
 * @author haijun.mai
 * @date   2022-01-05
 * @version 1.0
 * @Revision
 */

#ifndef MS_GPIO_LL_H_
#define MS_GPIO_LL_H_

#include <ms1008.h>
#include "ms_gpio_regs.h"
#include <stdint.h>
#include <stdbool.h>

#define MS_GPIO_PORT_LEN                           8

static inline void ms_gpio_interrupt_en_ll(Gpio_Type *gpio, uint8_t pin)
{
    SET_BIT(gpio->INTEN, (0x1ul<<(pin % MS_GPIO_PORT_LEN)));
}
static inline void ms_gpio_interrupt_disen_ll(Gpio_Type *gpio, uint8_t pin)
{
    CLEAR_BIT(gpio->INTEN, (0x1ul<<(pin % MS_GPIO_PORT_LEN)));
}


static inline void ms_gpio_interrupt_mask_ll(Gpio_Type *gpio, uint8_t pin)
{
    SET_BIT(gpio->INTMASK, (0x1ul<<(pin % MS_GPIO_PORT_LEN)));
}

static inline void ms_gpio_interrupt_unmask_ll(Gpio_Type *gpio, uint8_t pin)
{
    CLEAR_BIT(gpio->INTMASK, (0x1ul<<(pin % MS_GPIO_PORT_LEN)));
}

static inline void ms_gpio_set_interrupt_level_ll(Gpio_Type *gpio, uint8_t pin,uint8_t level)
{
    CLEAR_BIT(gpio->INTTYPELEVEL, (0x1ul<<(pin % MS_GPIO_PORT_LEN)));
    SET_BIT(gpio->INTTYPELEVEL, (level<<(pin % MS_GPIO_PORT_LEN)));
}

static inline void ms_gpio_set_interrupt_polarity_ll(Gpio_Type *gpio, uint8_t pin,uint8_t polarity)
{
    CLEAR_BIT(gpio->INTPOLARITY, (0x1ul<<(pin % MS_GPIO_PORT_LEN)));
    SET_BIT(gpio->INTPOLARITY, (polarity<<(pin % MS_GPIO_PORT_LEN)));
}

static inline void ms_gpio_interrupt_clear_status_ll(Gpio_Type *gpio, uint8_t pin)
{

    SET_BIT(gpio->PAEOI, (0x1ul<<(pin % MS_GPIO_PORT_LEN)));

}

static inline uint8_t ms_gpio_get_interrupt_raw_status_ll(Gpio_Type *gpio, uint8_t pin)
{
    return READ_REG(gpio->RAWINTSTATUS);
}

static inline uint8_t ms_gpio_get_interrupt_status_ll()
{
    return READ_REG(GPIO->INTSTATUS);
}

extern void ms_gpio_set_dir_out_put_ll(Gpio_Type *gpio, uint8_t pin);
extern void ms_gpio_set_dir_in_put_ll(Gpio_Type *gpio, uint8_t pin);
extern void ms_gpio_out_put_high_ll(Gpio_Type *gpio, uint8_t pin);
extern void ms_gpio_out_put_low_ll(Gpio_Type *gpio, uint8_t pin);
extern uint8_t ms_gpio_get_input_value_ll(Gpio_Type *gpio, uint8_t pin);
extern void ms_gpio_output_toggle_ll(Gpio_Type *gpio, uint8_t pin);

#endif

