/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  ms_audio_adc_hal.h
 * @brief
 * @author bingrui.chen
 * @date 2022-1-13
 * @version 1.0
 * @Revision:
 */
#ifndef MS_AUDIO_ADC_HAL_H_
#define MS_AUDIO_ADC_HAL_H_

#include "ms_audio_adc_ll.h"
#include "ms_clock_hal.h"

/** @addtogroup MS_HAL_Driver
 * @{
 */

/** @defgroup AUDIO_ADC_HAL AUDIO_ADC
 * @{
 */

/**
 * @brief  Enable Audio ADC Power
 * @retval None
 */
#define ms_audio_adc_hal_enable_power()   ms_audio_adc_ll_enable_power()


/**
 * @brief  Disable Audio ADC Power
 * @retval None
 */
#define ms_audio_adc_hal_disable_power()   ms_audio_adc_ll_disable_power()


/**
 * @brief  Disable Audio ADC Power
 * @retval None
 */
#define ms_audio_adc_hal_rc_osc16m_enable()   


/**
 * @brief  Disable Audio ADC Power
 * @retval None
 */
#define ms_audio_adc_hal_pll_enable()   ms_audio_adc_ll_enable_pll()

/**
 * @brief  Disable Audio ADC Power
 * @retval None
 */
#define ms_audio_adc_hal_ext_osc16m_enable()   ms_audio_adc_ll_disable_pll()



/**
 * @brief  Enable Audio ADC Clock
 * @param  AudioAdc_Type *audio_adc: AudioAdc Instance
 * @retval None
 */
#define ms_audio_adc_hal_enable_clk()   MS_CLOCK_HAL_CLK_ENABLE_AUD_ADC()

/**
 * @brief  Disable Audio ADC Clock
 * @retval None
 */
#define ms_audio_adc_hal_disable_clk()   MS_CLOCK_HAL_CLK_DISABLE_AUD_ADC()

/**
 * @brief  Enable Audio ADC Dma
 * @param  AudioAdc_Type *audio_adc: AudioAdc Instance
 * @retval None
 */
#define ms_audio_adc_hal_enable_dma()   ms_audio_adc_ll_enable_dma()

/**
 * @brief  Disable Audio ADC Dma
 * @retval None
 */
#define ms_audio_adc_hal_disable_dma()   ms_audio_adc_ll_disable_dma()

/**
 * @brief  Enable Audio ADC CPU interrupt
 * @retval None
 */
#define ms_audio_adc_hal_enable_cpu_int()   ms_audio_adc_ll_enable_cpu_int()

/**
 * @brief  Disable Audio ADC CPU interrupt
 * @retval None
 */
#define ms_audio_adc_hal_disable_cpu_int()   ms_audio_adc_ll_disable_cpu_int()

/**
 * @brief  Enable Audio ADC interrupt
 * @retval None
 */
#define ms_audio_adc_hal_enable_int()   ms_audio_adc_ll_enable_int()

/**
 * @brief  Disable Audio ADC interrupt
 * @retval None
 */
#define ms_audio_adc_hal_disable_int()   ms_audio_adc_ll_disable_int()

/**
 * @brief  Flush Audio ADC FIFO
 * @retval None
 */
#define ms_audio_adc_hal_active_flush_fifo()   ms_audio_adc_ll_active_flush_fifo()

/**
 * @brief  Inactive Flush Audio ADC FIFO
 * @retval None
 */
#define ms_audio_adc_hal_inactive_flush_fifo()   ms_audio_adc_ll_inactive_flush_fifo()


/**
 * @brief  Mask Interrupt
 * @param  uint32_t mask: AudioAdc Instance
 *         @arg @ref AUDIO_ADC_INT_MASK_FULL
 *         @arg @ref AUDIO_ADC_INT_MASK_EMPTY
 *         @arg @ref AUDIO_ADC_INT_MASK_WATER
 *         @arg @ref AUDIO_ADC_INT_MASK_OVERFLOW
 *         @arg @ref AUDIO_ADC_INT_MASK_UNDERFLOW
 * @retval None
 */
#define ms_audio_adc_hal_mask_int(mask)   ms_audio_adc_ll_mask_int((mask))

/**
 * @brief  Unmask Interrupt
 * @param  uint32_t mask: AudioAdc Instance
 *         @arg @ref AUDIO_ADC_INT_MASK_FULL
 *         @arg @ref AUDIO_ADC_INT_MASK_EMPTY
 *         @arg @ref AUDIO_ADC_INT_MASK_WATER
 *         @arg @ref AUDIO_ADC_INT_MASK_OVERFLOW
 *         @arg @ref AUDIO_ADC_INT_MASK_UNDERFLOW
 * @retval None
 */
#define ms_audio_adc_hal_unmask_int(unmask)   ms_audio_adc_ll_unmask_int((unmask))

/**
 * @brief  Get FIFO Full State
 * @retval true or not
 */
#define ms_audio_adc_hal_fifo_is_full()   ms_audio_adc_ll_fifo_is_full()

/**
 * @brief  Get FIFO Empty State
 * @retval true or not
 */
#define ms_audio_adc_hal_fifo_is_empty()   ms_audio_adc_ll_fifo_is_empty()

/**
 * @brief  Get FIFO Water State
 * @retval true or not
 */
#define ms_audio_adc_hal_fifo_is_water()   ms_audio_adc_ll_fifo_is_water()

/**
 * @brief  Get FIFO overflow State
 * @retval true or not
 */
#define ms_audio_adc_hal_fifo_is_overflow()   ms_audio_adc_ll_fifo_is_overflow()

/**
 * @brief  Get FIFO underflow State
 * @retval true or not
 */
#define ms_audio_adc_hal_fifo_is_underflow()   ms_audio_adc_ll_fifo_is_underflow()

/**
 * @brief  Get Data From FIFO
 * @retval none
 */
#define ms_audio_adc_hal_get_data()   ms_audio_adc_ll_get_data()

/**
 * @brief  Set  FIFO Water Level
 * @retval none
 */
#define ms_audio_adc_hal_set_water_level(water_level)   ms_audio_adc_ll_set_water_level((water_level))

/**
 * @brief  Get FIFO Water Level
 * @retval the Setting FIFO Water Level
 */
#define ms_audio_adc_hal_get_water_level()   ms_audio_adc_ll_get_water_level()

#define ms_audio_adc_hal_get_instance()  ms_audio_adc_ll_get_instance()

/**
 * @}
 */

/**
 * @}
 */

#endif /* MS_AUDIO_ADC_HAL_H_ */
