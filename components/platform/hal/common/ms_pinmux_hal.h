/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_pinmux_hal.h
 * @brief
 * @author bingrui.chen
 * @date 2021-12-24
 * @version 1.0
 * @Revision
 */

#ifndef __MS_PINMUX_HAL_H
#define __MS_PINMUX_HAL_H

#include <ms1008.h>
#include "ms_pinmux_ll.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup MS_HAL_Driver
 * @{
 */

/** @defgroup PINMUX_HAL PINMUX
 * @{
 */
typedef void (*PinmuxHanle_Type)(uint32_t mode);

/**
 * @brief  configure pin pull down mode
 * @param[in]  PadNum_Type pin:  @ref PadNum_Type
 * @retval None
 */
#define ms_pinmux_hal_set_pin_pull_down(pin) ms_pinmux_ll_set_pin_pull_down((pin))

/**
 * @brief  configure pin pull up mode
 * @param[in]  PadNum_Type pin:  @ref PadNum_Type
 * @retval None
 */
#define ms_pinmux_hal_set_pin_pull_up(pin) ms_pinmux_ll_set_pin_pull_up((pin))

/**
 * @brief  configure pin pull none mode
 * @param[in]  PadNum_Type pin:  @ref PadNum_Type
 * @retval None
 */
#define ms_pinmux_hal_set_pin_pull_none(pin) ms_pinmux_ll_set_pin_pull_none((pin))

/**
 * @brief  configure pin jtag keep enable
 * @retval None
 */
#define ms_pinmux_hal_set_pin_jtag_keep_enable() ms_pinmux_ll_set_pin_jtag_keep_enable()

/**
 * @brief  configure pin jtag keep disable
 * @retval None
 */
#define ms_pinmux_hal_set_pin_jtag_keep_diable() ms_pinmux_ll_set_pin_jtag_keep_diable()

/**
 * @brief  configure default pins mux
 * @retval None
 */
PinmuxHanle_Type ms_pinmux_hal_get_pinmux_handle(uint32_t pin);
/**
 * @brief  configure default pins mux
 * @retval None
 */
extern int32_t ms_pinmux_hal_set_pinmux(uint32_t pin,uint32_t mode);

/**
 * @brief  configure default pins mux
 * @retval None
 */
extern void ms_pinmux_hal_config_default();

/**
 * @brief  configure default pins mux
 * @retval None
 */
extern int32_t ms_pinmux_hal_config_pull_type(uint32_t pin, uint32_t pull_type);


/**
 * @brief  store pins mux setting
 * @param[out] uint32_t *store_buf: pointer to pin mux stored buffer
 * @retval None
 */
extern void ms_pinmux_hal_sleep_store(uint32_t *store_buf);

/**
 * @brief  store pins mux setting
 * @param[in] uint32_t *store_buf: pointer to pin mux stored buffer
 * @retval None
 */
extern void ms_pinmux_hal_sleep_restore(uint32_t *store_buf);

#ifdef __cplusplus
}
#endif

/** @}*/

/** @}*/

#endif /*__MS_PINMUX_H */

