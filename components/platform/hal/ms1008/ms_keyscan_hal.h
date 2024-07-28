/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  ms_keyscan_hal.h
 * @brief
 * @author bingrui.chen
 * @date 2022-2-8
 * @version 1.0
 * @Revision:
 */
#ifndef MS_KEYSCAN_HAL_H_
#define MS_KEYSCAN_HAL_H_

#include "ms_clock_hal.h"
#include "ms_keyscan_ll.h"
/** @addtogroup MS_HAL_Driver
 * @{
 */

/** @defgroup KEYSCAN_HAL KEYSCAN
 * @{
 */

/**
 * @brief  Enable Keyscan Clock
 * @param  AudioAdc_Type *audio_adc: AudioAdc Instance
 * @retval None
 */
#define ms_keyscan_hal_enable_clk()   MS_CLOCK_HAL_CLK_ENABLE_KSCAN()

/**
 * @brief  Disable Keyscan Clock
 * @retval None
 */
#define ms_keyscan_hal_disable_clk()   MS_CLOCK_HAL_CLK_DISABLE_KSCAN()

/**
 * @brief  Enable Keyscan CPU interrupt
 * @retval None
 */
#define ms_keyscan_hal_enable_cpu_int() ms_keyscan_ll_enable_cpu_int()

/**
 * @brief  Disable Keyscan CPU interrupt
 * @retval None
 */
#define ms_keyscan_hal_disable_cpu_int() ms_keyscan_ll_disable_cpu_int()

/**
 * @brief  Enable Keyscan Module
 * @retval None
 */
#define ms_keyscan_hal_enable_module() ms_keyscan_ll_enable_module()

/**
 * @brief  Disable Keyscan Module
 * @retval None
 */
#define ms_keyscan_hal_disable_module() ms_keyscan_ll_disable_module()

/**
 * @brief  flush Keyscan data
 * @retval None
 */
#define ms_keyscan_hal_flush() ms_keyscan_ll_flush()

/**
 * @brief  Set Keyscan push event to fifo option
 * @param[in] option:
 *			  KEYSCAN_PUSH_EVENT_LONG_REPEAT_RELEASE
 *            KEYSCAN_PUSH_EVENT_ALL
 * @retval None
 */
#define ms_keyscan_hal_set_push_event_option(option) ms_keyscan_ll_set_push_event_option((option))

/**
 * @brief  Set Keyscan push event to fifo option
 * @param[in] freq:
 *			  KEYSCAN_SCAN_FREQ_1_CYCLE
 *            KEYSCAN_SCAN_FREQ_2_CYCLE
 *			  KEYSCAN_SCAN_FREQ_4_CYCLE
 *			  KEYSCAN_SCAN_FREQ_8_CYCLE
 * @retval None
 */
#define ms_keyscan_hal_set_scan_freq(freq) ms_keyscan_ll_set_scan_freq((freq))

/**
 * @brief  Set Keyscan repeat detect enable
 * @param[in] enable:
 *			  KEYSCAN_REPEAT_EN_ONE_TIME
 *            KEYSCAN_REPEAT_EN_MULTI_TIME
 * @retval None
 */
#define ms_keyscan_hal_set_rep_en(enable) ms_keyscan_ll_set_rep_en((enable))

/**
 * @brief  Enable Keyscan long press event
 * @retval None
 */
#define ms_keyscan_hal_enable_long_press() ms_keyscan_ll_enable_long_press()

/**
 * @brief  Disable Keyscan long press event
 * @retval None
 */
#define ms_keyscan_hal_disable_long_press() ms_keyscan_ll_disable_long_press()

/**
 * @brief  Enable Keyscan multi-key detect 
 * @retval None
 */
#define ms_keyscan_hal_enable_multi_key() ms_keyscan_ll_enable_multi_key()

/**
 * @brief  Disable Keyscan multi-key detect 
 * @retval None
 */
#define ms_keyscan_hal_disable_multi_key() ms_keyscan_ll_disable_multi_key()

/**
 * @brief  Enable Keyscan debounce
 * @retval None
 */
#define ms_keyscan_hal_enable_debounce() ms_keyscan_ll_enable_debounce()

/**
 * @brief  Disable Keyscan debounce
 * @retval None
 */
#define ms_keyscan_hal_disable_debounce() ms_keyscan_ll_disable_debounce()


/**
 * @brief  Enable pushkd, save  press event to FIFO
 * @retval None
 */
#define ms_keyscan_hal_enable_pushkd() ms_keyscan_ll_enable_pushkd()


/**
 * @brief  Disable pushkd, not save  press event to FIFO
 * @retval None
 */
#define ms_keyscan_hal_disable_pushkd() ms_keyscan_ll_disable_pushkd()





/**
 * @brief  Set Keyscan Interrupt Mask
 * @param[in] int_mask:
 *			  KEYSCAN_INT_MASK_KEY_PRESS
 *			  KEYSCAN_INT_MASK_KEY_RELEASE
 *            KEYSCAN_INT_MASK_KEY_REPEAT
 *            KEYSCAN_INT_MASK_KEY_LONG_PRESS		
 *            KEYSCAN_INT_MASK_KEY_HOLD_TIMEOUT		
 *            KEYSCAN_INT_MASK_KEY_FIFO_FULL	
 * @retval None
 */
#define ms_keyscan_hal_set_interrupt_mask(int_mask) ms_keyscan_ll_set_interrupt_mask((int_mask))

/**
 * @brief  Set Keyscan Interrupt Mask
 * @param[in] int_mask:
 *			  KEYSCAN_INT_MASK_KEY_PRESS
 *			  KEYSCAN_INT_MASK_KEY_RELEASE
 *            KEYSCAN_INT_MASK_KEY_REPEAT
 *            KEYSCAN_INT_MASK_KEY_LONG_PRESS		
 *            KEYSCAN_INT_MASK_KEY_HOLD_TIMEOUT		
 *            KEYSCAN_INT_MASK_KEY_FIFO_FULL	
 * @retval None
 */
#define ms_keyscan_hal_set_interrupt_unmask(int_unmask) ms_keyscan_ll_set_interrupt_unmask((int_unmask))

/**
 * @brief  Enable Keyscan column lat when in debounce
 * @retval None
 */
#define ms_keyscan_hal_enable_column_lat() ms_keyscan_ll_enable_column_lat()

/**
 * @brief  Disable Keyscan column lat when in debounce
 * @retval None
 */
#define ms_keyscan_hal_disable_column_lat() ms_keyscan_ll_disable_column_lat()

/**
 * @brief  clear state hold when debounce disable
 * @retval None
 */
#define ms_keyscan_hal_clear_hold() ms_keyscan_ll_clear_hold()

/**
 * @brief  set long press timer
 * @retval None
 */
#define ms_keyscan_hal_set_long_press_time(long_press_time) ms_keyscan_ll_set_long_press_time((long_press_time))

/**
 * @brief  set repeat press timer
 * @retval None
 */
#define ms_keyscan_hal_set_repeat_press_time(repeat_press_time) ms_keyscan_ll_set_repeat_press_time((repeat_press_time))

/**
 * @brief  get keydown event
 * @retval true or false
 */
#define ms_keyscan_hal_event_is_keydown() ms_keyscan_ll_event_is_keydown()

/**
 * @brief  get long press timeout event
 * @retval true or false
 */
#define ms_keyscan_hal_event_is_long_press_timeout() ms_keyscan_ll_event_is_long_press_timeout()

/**
 * @brief  get fifo full event
 * @retval true or false
 */
#define ms_keyscan_hal_event_is_fifo_full() ms_keyscan_ll_event_is_fifo_full()

/**
 * @brief  get fifo overflow event
 * @retval true or false
 */
#define ms_keyscan_hal_event_is_fifo_overflow() ms_keyscan_ll_event_is_fifo_overflow()

/**
 * @brief  get fifo not empty event
 * @retval true or false
 */
#define ms_keyscan_hal_event_is_fifo_ne() ms_keyscan_ll_event_is_fifo_ne()

/**
 * @brief  get key hold timeout event
 * @retval true or false
 */
#define ms_keyscan_hal_event_is_hold_timeout() ms_keyscan_ll_event_is_hold_timeout()

/**
 * @brief  get key column code
 * @retval key column code
 */
#define ms_keyscan_hal_get_key_col() ms_keyscan_ll_get_key_col()

/**
 * @brief  get key row code
 * @retval key row code
 */
#define ms_keyscan_hal_get_key_row() ms_keyscan_ll_get_key_row()

/**
 * @brief  get key column input
 * @retval key column input
 */
#define ms_keyscan_hal_get_key_col_in() ms_keyscan_ll_get_key_col_in()

/**
 * @brief  get key fifo rdata
 * @retval key fifo rdata
 */
#define ms_keyscan_hal_get_fifo_rdata() ms_keyscan_ll_get_fifo_rdata()


/**
 * @brief  get key fifo rdata coloum
 * @retval key fifo rdata
 */
#define ms_keyscan_hal_get_fifo_rdata_col()  ms_keyscan_ll_get_fifo_rdata_col()

/**
 * @brief  get key fifo rdata row
 * @retval key fifo rdata
 */
#define ms_keyscan_hal_get_fifo_rdata_row() ms_keyscan_ll_get_fifo_rdata_row()


/**
 * @brief  get key fifo rdata key press
 * @retval key fifo rdata
 */
#define ms_keyscan_hal_get_fifo_rdata_keypress() ms_keyscan_ll_get_fifo_rdata_keypress()

/**
 * @brief  get key fifo rdata long key
 * @retval key fifo rdata
 */
#define ms_keyscan_hal_get_fifo_rdata_longkey() ms_keyscan_ll_get_fifo_rdata_longkey()


/**
 * @brief  get key interrupt source
 * @retval key interrupt source
 */
#define ms_keyscan_hal_get_int_src() ms_keyscan_ll_get_int_src()

/**
 * @brief  verify key press interrupt source
 * @retval true or false
 */
#define ms_keyscan_hal_int_is_key_press(int_src) ms_keyscan_ll_int_is_key_press(int_src)

/**
 * @brief  verify key release interrupt source
 * @retval true or false
 */
#define ms_keyscan_hal_int_is_key_release(int_src) ms_keyscan_ll_int_is_key_release(int_src)

/**
 * @brief  verify key repeat interrupt source
 * @retval true or false
 */
#define ms_keyscan_hal_int_is_key_repeat(int_src) ms_keyscan_ll_int_is_key_repeat(int_src)

/**
 * @brief  verify key long press interrupt source
 * @retval true or false
 */
#define ms_keyscan_hal_int_is_key_long_press(int_src) ms_keyscan_ll_int_is_key_long_press(int_src)

/**
 * @brief  verify key hold timeout interrupt source
 * @retval true or false
 */
#define ms_keyscan_hal_int_is_key_hold_timeout(int_src) ms_keyscan_ll_int_is_key_hold_timeout(int_src)

/**
 * @brief  verify fifo full interrupt source
 * @retval true or false
 */
#define ms_keyscan_hal_int_is_fifo_full(int_src) ms_keyscan_ll_int_is_fifo_full(int_src)

/**
 * @brief  set fifo control register
 * @retval none
 */
#define ms_keyscan_hal_set_fifo_ctrl(option) ms_keyscan_ll_set_fifo_ctrl(option)

/**
 * @brief  set fifo debounce_length
 * @retval none
 */
#define ms_keyscan_hal_set_debounce_length(length) ms_keyscan_ll_set_debounce_length(length)

/**
 * @brief  set fifo control register
 * @retval none
 */
#define ms_keyscan_hal_set_debonce_timeout(timeout) ms_keyscan_ll_set_debonce_timeout(timeout)

/**
 * @}
 */

/**
 * @}
 */

#endif /* MS_KEYSCAN_HAL_H_ */
