/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_gpio_hal.h
 * @brief Header file of gpio  module.
 * @author haijun.mai
 * @date   2022-01-07
 * @version 1.0
 * @Revision
 */


#ifndef MS_GPIO_HAL_H_
#define MS_GPIO_HAL_H_

#include "ms_gpio_hal.h"


/**
 * @brief gpio interrupt enable
 * @retval None
 */
#define ms_gpio_interrupt_en_hal(gpio, pin)        ms_gpio_interrupt_en_ll(gpio,pin)


/**
 * @brief gpio interrupt enable
 * @retval None
 */
#define ms_gpio_interrupt_disen_hal(gpio, pin)        ms_gpio_interrupt_disen_ll(gpio,pin)


/**
 * @brief gpio interrupt mask
 * @retval None
 */
#define ms_gpio_interrupt_mask_hal(gpio, pin)        ms_gpio_interrupt_mask_ll(gpio, pin)        

/**
 * @brief gpio interrupt mask
 * @retval None
 */
#define ms_gpio_interrupt_unmask_hal(gpio, pin)        ms_gpio_interrupt_unmask_ll(gpio, pin)        



/**
 * @brief gpio set interrupt level
 * @retval None
 */
#define ms_gpio_set_interrupt_level_hal(gpio,pin,level)        ms_gpio_set_interrupt_level_ll(gpio,pin,level)        


/**
 * @brief gpio set interrupt polarity
 * @retval None
 */
 #define  ms_gpio_set_interrupt_polarity_hal( gpio, pin,polarity)        ms_gpio_set_interrupt_polarity_ll( gpio, pin,polarity)

/**
 * @brief gpio clear interrupt status
 * @retval None
 */
 #define    ms_gpio_interrupt_clear_status_hal(gpio,  pin)        ms_gpio_interrupt_clear_status_ll(gpio, pin)     

/**
 * @brief gpio get interrupt raw status
 * @retval None
 */
 #define  ms_gpio_get_interrupt_raw_status_hal(gpio, pin)         ms_gpio_get_interrupt_raw_status_ll(gpio, pin)

/**
 * @brief gpio get interrupt  status
 * @retval None
 */
#define ms_gpio_get_interrupt_status_hal()        ms_gpio_get_interrupt_status_ll()

/**
 * @brief gpio set dir as output
 * @retval None
 */
#define ms_gpio_set_dir_out_put_hal(gpio, pin)        ms_gpio_set_dir_out_put_ll(gpio, pin)

/**
 * @brief gpio set dir as input
 * @retval None
 */
#define ms_gpio_set_dir_in_put_hal(gpio, pin)        ms_gpio_set_dir_in_put_ll(gpio, pin)


/**
 * @brief gpio out put high level
 * @retval None
 */ 
#define ms_gpio_out_put_high_hal(gpio, pin)         ms_gpio_out_put_high_ll(gpio, pin)

/**
 * @brief gpio out put low level
 * @retval None
 */ 
#define ms_gpio_out_put_low_hal(gpio,pin)         ms_gpio_out_put_low_ll(gpio, pin)

/**
 * @brief gpio get input value
 * @retval None
 */ 
#define ms_gpio_get_input_value_hal(gpio, pin)        ms_gpio_get_input_value_ll(gpio, pin)        


/**
 * @brief gpio output toggle
 * @retval None
 */ 
#define ms_gpio_output_toggle_hal(gpio, pin)        ms_gpio_output_toggle_ll(gpio, pin)    


#endif/* MS_GPIO_HAL_H_ */

