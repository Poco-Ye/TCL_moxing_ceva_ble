/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_gpio_hal.h
 * @brief Header file of gpio  module.
 * @author haijun.mai
 * @date   2022-01-05
 * @version 1.0
 * @Revision
 */

#ifndef MS_GPIO_H_
#define MS_GPIO_H_

#include "ms1008.h"
#include "ms_gpio_ll.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO0_INDEX 0
#define GPIO1_INDEX 1
#define GPIO2_INDEX 2
#define GPIO3_INDEX 3
#define GPIO4_INDEX 4
#define GPIO5_INDEX 5
#define GPIO6_INDEX 6
#define GPIO7_INDEX 7
#define GPIO8_INDEX 8
#define GPIO9_INDEX 9
#define GPIO10_INDEX 10
#define GPIO11_INDEX 11
#define GPIO12_INDEX 12
#define GPIO13_INDEX 13
#define GPIO14_INDEX 14
#define GPIO15_INDEX 15
#define GPIO16_INDEX 16
#define GPIO17_INDEX 17
#define GPIO18_INDEX 18
#define GPIO19_INDEX 19
#define GPIO20_INDEX 20
#define GPIO21_INDEX 21
#define GPIO22_INDEX 22
#define GPIO23_INDEX 23
#define GPIO24_INDEX 24
#define GPIO25_INDEX 25
#define GPIO26_INDEX 26





/*define gpio mode*/
#define GPIO_MODE_OUT_PUT     (1)
#define GPIO_MODE_IN_PUT        (0)

/*define gpio interrupt level*/
#define GPIO_INTERRUPT_LEVEL_SENSITIVE          (0)
#define GPIO_INTERRUPT_EDGE_SENSITIVE           (1)


/*define gpio interrupt polarity*/
#define GPIO_INTERRUPT_ACTIVE_LOW_POLARITY            (0)
#define GPIO_INTERRUPT_ACTIVE_HIGH_POLARITY           (1)


#define GPIO_ERROR_INVALID_CALLBACK  0x00000001U   /*!< Invalid Callback error  */


typedef struct
{
    uint8_t      pin;   
    uint8_t      mode;
    uint8_t      interrupt_level;  /*Whenever a 0 is config to this , it configuresthe interrupt type to be level-sensitive; otherwise, it is edgesensitive*/
    uint8_t      interrupt_polarity; /* Whenever a 0 is config to  this , it configures the interrupt type to falling-edge oractive-low sensitive; otherwise, it is rising-edge or active-highsensitive.*/
} GpioInit_Type;

struct __GpioHandle_Type;
/**
  * @brief  Timer callback handle Structure definition
  */
typedef struct
{
    void (* error_callback)                (struct __GpioHandle_Type *gpio);
    void (* init_callback)                   (struct __GpioHandle_Type *gpio);
    void (* deinit_callback)               (struct __GpioHandle_Type *gpio);
    void (* gpio_reach_callback)    (struct __GpioHandle_Type *gpio);
}GpioCallback_Type;


typedef struct __GpioHandle_Type
{
    Gpio_Type           *instance;
    GpioInit_Type              init;/*!< Timer communication parameters      */
    uint32_t                       error_code;         /*!< TIMER Error code*/      
    IRQn_Type                    irq;
    GpioCallback_Type         *p_callback;
}GpioHandle_Type;

extern void ms_gpio_enable_cpu_interrupt(GpioHandle_Type *gpio);
extern void ms_gpio_disable_cpu_interrupt(GpioHandle_Type *gpio);
extern void ms_gpio_init(GpioHandle_Type *gpio);
extern void ms_gpio_set_out_put_high(GpioHandle_Type *gpio);
extern void ms_gpio_set_out_put_low(GpioHandle_Type *gpio);
extern uint8_t  ms_gpio_get_input_value(GpioHandle_Type *gpio);
extern void ms_gpio_interrupt_en(GpioHandle_Type *gpio);
extern void ms_gpio_interrupt_disen(GpioHandle_Type *gpio);
extern void ms_gpio_interrupt_mask(GpioHandle_Type *gpio);
void ms_gpio_interrupt_unmask(GpioHandle_Type *gpio);
extern void ms_gpio_set_interrupt_level(GpioHandle_Type *gpio);
extern void ms_gpio_set_interrupt_polarity(GpioHandle_Type *gpio);
extern void ms_gpio_interrupt_clear_status(GpioHandle_Type *gpio);
extern uint8_t  ms_gpio_get_interrupt_raw_status(GpioHandle_Type *gpio);
extern uint8_t  ms_gpio_get_interrupt_status();
extern void ms_gpio_irq_handler(GpioHandle_Type *gpio);
extern int32_t ms_gpio_deinit(GpioHandle_Type *gpio);
extern void ms_gpio_output_toggle(GpioHandle_Type *gpio);
#ifdef __cplusplus
}
#endif

#endif /* MS_GPIO_HAL_H_ */

