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
 * @brief  enable long press
 * @retval None
 */
#define ms_keyscan_hal_enable_long_press() ms_keyscan_ll_enable_long_press()


/**
 * @brief  disable long press
 * @retval None
 */
#define ms_keyscan_hal_disable_long_press() ms_keyscan_ll_disable_long_press()


/**
 * @brief  enable long press repeat
 * @retval None
 */
#define ms_keyscan_hal_enable_long_press_repeat_int()  ms_keyscan_ll_enable_long_repeat_press()


/**
 * @brief  disable long press repeat
 * @retval None
 */
#define ms_keyscan_hal_disable_long_press_repeat_int()  ms_keyscan_ll_disable_long_repeat_press()


/**
 * @brief  enable press fifo
 * @retval None
 */
#define ms_keyscan_hal_enable_press_fifo()  ms_keyscan_ll_enable_press_fifo()


/**
 * @brief  disable press fifo
 * @retval None
 */
#define ms_keyscan_hal_disable_press_fifo() ms_keyscan_ll_disable_press_fifo()


/**
 * @brief  enable release fifo
 * @retval None
 */
#define ms_keyscan_hal_enable_release_fifo()  ms_keyscan_ll_enable_release_fifo()


/**
 * @brief  disable release fifo
 * @retval None
 */
#define ms_keyscan_hal_disable_release_fifo() ms_keyscan_ll_disable_release_fifo()

/**
 * @brief  enable long press fifo
 * @retval None
 */
#define ms_keyscan_hal_enable_long_press_fifo()  ms_keyscan_ll_enable_long_press_fifo()


/**
 * @brief  disable long press fifo
 * @retval None
 */
#define ms_keyscan_hal_disable_long_press_fifo() ms_keyscan_ll_disable_long_press_fifo()


/**
 * @brief  enable long  repeat press fifo
 * @retval None
 */
#define ms_keyscan_hal_enable_long_repeat_press_fifo()  ms_keyscan_ll_enable_repeat_long_press_fifo()


/**
 * @brief  disable long repeat press fifo
 * @retval None
 */
#define ms_keyscan_hal_disable_long_repeat_press_fifo() ms_keyscan_ll_disable_repeat_long_press_fifo()

/**
 * @brief  enable press timeout fifo
 * @retval None
 */
#define ms_keyscan_hal_enable_press_timeout_fifo()  ms_keyscan_ll_enable_press_fifo_timeout()


/**
 * @brief  disable press timeout fifo
 * @retval None
 */
#define ms_keyscan_hal_disable_press_timeout_fifo() ms_keyscan_ll_disable_press_fifo_timeout()

/**
 * @brief  enable press debounce
 * @retval None
 */
#define ms_keyscan_hal_enable_press_debounce()  ms_keyscan_ll_enable_debounce()


/**
 * @brief  disable press debounce
 * @retval None
 */
#define ms_keyscan_hal_disable_press_debounce() ms_keyscan_ll_disable_debounce()

/**
 * @brief  enable press timeout
 * @retval None
 */
#define ms_keyscan_hal_enable_press_timeout()  ms_keyscan_ll_enable_press_timeout()


/**
 * @brief  disable press timeout
 * @retval None
 */
#define ms_keyscan_hal_disable_press_timeout() ms_keyscan_ll_disable_press_timeout()

/**
 * @brief  enable kpad ip int
 * @retval None
 */
#define ms_keyscan_hal_enable_kpad_int()  ms_keyscan_ll_enable_kpad_int()


/**
 * @brief  disable kpad ip int
 * @retval None
 */
#define ms_keyscan_hal_disable_kpad_int() ms_keyscan_ll_disable_kpad_int()

/**
 * @brief  config scan period
 * @retval None
 */
#define ms_keyscan_hal_config_scan_period(val)  ms_keyscan_ll_config_scan_period(val)

/**
 * @brief  flush Keyscan data
 * @retval None
 */
#define ms_keyscan_hal_fifo_flush() ms_keyscan_ll_fifo_flush()

/**
 * @brief  config key press debounce value
 * @retval None
 */
#define ms_keyscan_hal_config_press_debounce(val) ms_keyscan_ll_config_press_debounce_value(val)

/**
 * @brief  config key release debounce value
 * @retval None
 */
#define ms_keyscan_hal_config_release_debounce(val) ms_keyscan_ll_config_release_debounce_value(val)

/**
 * @brief  config key  debounce timeout value
 * @retval None
 */
#define ms_keyscan_hal_config_debounce_timeout(val) ms_keyscan_ll_config_debounce_timeout_value(val)

/**
 * @brief  config key press timeout value
 * @retval None
 */
#define ms_keyscan_hal_config_press_timeout(val) ms_keyscan_ll_config_press_timeout_value(val)

/**
 * @brief  get fifo keyvalue rdata
 * @retval None
 */
#define ms_keyscan_hal_get_fifo_rdata()  ms_keyscan_ll_get_fifo_rdata()

/**
 * @brief  get fifo keyvalue rdata col
 * @retval None
 */
#define ms_keyscan_hal_get_fifo_rdata_col()  ms_keyscan_ll_get_fifo_rdata_col()

/**
 * @brief  get fifo keyvalue rdata row
 * @retval None
 */
#define ms_keyscan_hal_get_fifo_rdata_row()  ms_keyscan_ll_get_fifo_rdata_row()

/**
 * @brief  get fifo rdata event
 * @retval None
 */
#define ms_keyscan_hal_get_fifo_rdata_event()  ms_keyscan_ll_get_fifo_rdata_event()

/**
 * @brief  config fifo water mark
 * @retval None
 */
#define ms_keyscan_hal_config_fifo_watermark(val)  ms_keyscan_ll_config_fifo_watermark(val)

/**
 * @brief  enable press interrupt
 * @retval None
 */
#define ms_keyscan_hal_enable_press_int()  ms_keyscan_ll_enable_int_press()

/**
 * @brief  disable press interrupt
 * @retval None
 */
#define ms_keyscan_hal_disable_press_int()  ms_keyscan_ll_disable_int_press()

/**
 * @brief  enable release interrupt
 * @retval None
 */
#define ms_keyscan_hal_enable_release_int()  ms_keyscan_ll_enable_int_release()

/**
 * @brief  disable release interrupt
 * @retval None
 */
#define ms_keyscan_hal_disable_release_int()  ms_keyscan_ll_disable_int_release()

/**
 * @brief  enable long press interrupt
 * @retval None
 */
#define ms_keyscan_hal_enable_long_press_int()  ms_keyscan_ll_enable_int_long_press()

/**
 * @brief  disable long press interrupt
 * @retval None
 */
#define ms_keyscan_hal_disable_long_press_int()  ms_keyscan_ll_disable_int_long_press()

/**
 * @brief  enable  press timeout interrupt
 * @retval None
 */
#define ms_keyscan_hal_enable_press_timeout_int()  ms_keyscan_ll_enable_int_press_timeout()

/**
 * @brief  disable  press timeout interrupt
 * @retval None
 */
#define ms_keyscan_hal_disable_press_timeout_int()  ms_keyscan_ll_disable_int_press_timeout()

/**
 * @brief  enable  fifo water mark interrupt
 * @retval None
 */
#define ms_keyscan_hal_enable_fifo_watermark_int()  ms_keyscan_ll_enable_int_fifomark()

/**
 * @brief  disable  fifo water mark interrupt
 * @retval None
 */
#define ms_keyscan_hal_disable_fifo_watermark_int()  ms_keyscan_ll_disable_int_fifomark()


/**
 * @brief  enable  fifo empty
 * @retval None
 */
#define ms_keyscan_hal_enable_fifo_empty_int()  ms_keyscan_ll_enable_int_fifoempty()

/**
 * @brief  disable  fifo empty
 * @retval None
 */
#define ms_keyscan_hal_disable_fifo_empty_int()  ms_keyscan_ll_disable_int_fifoempty()

/**
 * @brief  enable  fifo full
 * @retval None
 */
#define ms_keyscan_hal_enable_fifo_full_int()  ms_keyscan_ll_enable_int_fifofull()

/**
 * @brief  disable  fifo full
 * @retval None
 */
#define ms_keyscan_hal_disable_fifo_full_int()  ms_keyscan_ll_disable_int_fifofull()

/**
 * @brief  get press int status
 * @retval None
 */
#define ms_keyscan_hal_get_press_int_status()  ms_keyscan_ll_int_press_status()

/**
 * @brief  get release int status
 * @retval None
 */
#define ms_keyscan_hal_get_release_int_status()  ms_keyscan_ll_int_release_status()

/**
 * @brief  get long press int status
 * @retval None
 */
#define ms_keyscan_hal_get_long_press_int_status()  ms_keyscan_ll_int_long_press_status()

/**
 * @brief  get press timeout int status
 * @retval None
 */
#define ms_keyscan_hal_get_press_timeout_int_status()  ms_keyscan_ll_int_press_timeout_status()

/**
 * @brief  get fifo mark int status
 * @retval None
 */
#define ms_keyscan_hal_get_fifo_mark_int_status()  ms_keyscan_ll_int_fifomark_status()

/**
 * @brief  get fifo empty int status
 * @retval None
 */
#define ms_keyscan_hal_get_fifo_empty_int_status()  ms_keyscan_ll_int_fifoempty_status()

/**
 * @brief  get fifo full int status
 * @retval None
 */
#define ms_keyscan_hal_get_fifo_full_int_status()  ms_keyscan_ll_int_fifofull_status()

/**
 * @brief  press int status clear
 * @retval None
 */
#define ms_keyscan_hal_clear_press_int_status()  ms_keyscan_ll_int_press_clear()

/**
 * @brief  release int status clear
 * @retval None
 */
#define ms_keyscan_hal_clear_release_int_status()  ms_keyscan_ll_int_release_clear()

/**
 * @brief  long press int status clear
 * @retval None
 */
#define ms_keyscan_hal_clear_long_press_int_status()  ms_keyscan_ll_int_long_press_clear()

/**
 * @brief  press timeout int status clear
 * @retval None
 */
#define ms_keyscan_hal_clear_press_timeout_int_status()  ms_keyscan_ll_int_press_timeout_clear()

/**
 * @brief  get fifo empty int status
 * @retval None
#define ms_keyscan_hal_get_fifo_empty_int_status()  ms_keyscan_ll_int_fifoempty_pnd_get()

/**
 * @brief  scan kpad state
 * @retval None
 */
#define ms_keyscan_hal_scan_kpad_state()  ms_keyscan_ll_scan_kpad_state()

/**
 * @brief  key value to key num(0~29)
 * @retval None
 */
#define ms_keyscan_hal_keyvalue_to_kpadnum(row, col)  ms_keyscan_ll_keyvalue_to_kpadnum(row, col)


/**
 * @brief  key long press value
 * @retval None
 */
#define ms_keyscan_hal_config_long_press_value(val)  ms_keyscan_ll_config_long_press_value(val)

/**
 * @brief kpad sel
 * @retval None
 */
#define ms_keyscan_hal_config_sel()  ms_keyscan_ll_config_sel()



#define ms_keyscan_hal_soft_rst()  ms_keyscan_ll_soft_rst()

/**
 * @}
 */

/**
 * @}
 */

#endif /* MS_KEYSCAN_HAL_H_ */
