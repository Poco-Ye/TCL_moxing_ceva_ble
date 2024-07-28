/*
 * gpio.h
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "ms1008.h"

extern int32_t gpio_input_init(int32_t pin);
extern int32_t gpio_out_init(int32_t pin);
extern int32_t gpio_falling_edge_interrput_init(int32_t pin);
extern int32_t gpio_rising_edge_interrput_init(int32_t pin);
extern int32_t gpio_low_level_interrput_init(int32_t pin);
extern int32_t gpio_high_level_interrput_init(int32_t pin);
extern int32_t gpio_out_put_high_level(int32_t pin);
extern int32_t gpio_out_put_low_level(int32_t pin);
extern uint8_t gpio_get_value(int32_t pin);
extern int32_t gpio_deinit(int32_t pin);

extern int32_t gpio_enter_deep_sleep_callback(uint8_t pin, uint8_t dir, uint8_t level);

#endif /* GPIO_H_ */

