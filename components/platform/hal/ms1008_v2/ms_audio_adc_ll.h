/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  ms_audio_adc_ll.h
 * @brief
 * @author bingrui.chen
 * @date 2022-1-13
 * @version 1.0
 * @Revision:
 */
#ifndef MS_audio_adc_LL_H_
#define MS_audio_adc_LL_H_

#include <ms1008.h>
#include "ms_interrupt.h"
#include <stdint.h>
#include <stdbool.h>

/** @addtogroup MS_LL_Driver
 * @{
 */

/** @defgroup AUDIO_ADC_LL AUDIO_ADC
 * @{
 */

/** @defgroup AUDIO_ADC_INT_MASK        Audio ADC Interrupt Mask definition
 * @{
 */
#define AUDIO_MASK_NONE         0x00000000UL
#define AUDIO_MASK_FULL         0x00000001UL
#define AUDIO_MASK_EMPTY        0x00000002UL
#define AUDIO_MASK_WATER        0x00000004UL
#define AUDIO_MASK_OVERFLOW     0x00000010UL
#define AUDIO_MASK_UNDERFLOW    0x00000008UL
/**
 * @}
 */

/** @defgroup AUDIO_ADC_FIFO_STA        Audio ADC FIFO State definition
 * @{
 */
#define AUDIO_ADC_FIFO_STA_FULL         0x00000001UL
#define AUDIO_ADC_FIFO_STA_EMPTY        0x00000002UL
#define AUDIO_ADC_FIFO_STA_WATER        0x00000004UL
#define AUDIO_ADC_FIFO_STA_UNDERFLOW    0x00000008UL
#define AUDIO_ADC_FIFO_STA_OVERFLOW     0x00000010UL



/**
 * @}
 */

/**
 * @brief  Enable Audio ADC Power
 * @retval None
 */
static inline void ms_audio_adc_ll_enable_power(void)
{
    SET_BIT(SYS_CTRL->UNIT_EN, SYS_CTRL_UNIT_EN_AUD_ADC_PU);
}

/**
 * @brief  Disable Audio ADC Power
 * @retval None
 */
static inline void ms_audio_adc_ll_disable_power(void)
{
    CLEAR_BIT(SYS_CTRL->UNIT_EN, SYS_CTRL_UNIT_EN_AUD_ADC_PU);
}

/**
 * @brief  Enable Audio ADC Power
 * @retval None
 */
static inline void ms_audio_adc_ll_enable_pll(void)
{
    SET_BIT(SYS_CTRL->UNIT_EN, SYS_CTRL_UNIT_EN_PLL_EN_MASK);
}

/**
 * @brief  Disable Audio ADC Power
 * @retval None
 */
static inline void ms_audio_adc_ll_disable_pll(void)
{
    CLEAR_BIT(SYS_CTRL->UNIT_EN, SYS_CTRL_UNIT_EN_PLL_EN_MASK);
}




/**
 * @brief  Enable Audio ADC Dma
 * @retval None
 */
static inline void ms_audio_adc_ll_enable_dma(void)
{
    SET_BIT(AUDIO_ADC->CFG, AUDIO_ADC_CFG_DMA_EN);
}

/**
 * @brief  Disable Audio ADC Dma
 * @retval None
 */
static inline void ms_audio_adc_ll_disable_dma(void)
{
    CLEAR_BIT(AUDIO_ADC->CFG, AUDIO_ADC_CFG_DMA_EN);
}

/**
 * @brief  Enable Audio ADC CPU interrupt
 * @retval None
 */
static inline void ms_audio_adc_ll_enable_cpu_int(void)
{
    INTERRUPT_ENABLE_IRQ(AUDADC_IRQn);
}

/**
 * @brief  Disable Audio ADC CPU interrupt
 * @retval None
 */
static inline void ms_audio_adc_ll_disable_cpu_int(void)
{
    INTERRUPT_DISABLE_IRQ(AUDADC_IRQn);
}

/**
 * @brief  Enable Audio ADC interrupt
 * @retval None
 */
static inline void ms_audio_adc_ll_enable_int(void)
{
    SET_BIT(AUDIO_ADC->CFG, AUDIO_ADC_CFG_INT_EN);
}

/**
 * @brief  Disable Audio ADC interrupt
 * @retval None
 */
static inline void ms_audio_adc_ll_disable_int(void)
{
    CLEAR_BIT(AUDIO_ADC->CFG, AUDIO_ADC_CFG_INT_EN);
}

/**
 * @brief  Active Flush Audio ADC FIFO
 * @retval None
 */
static inline void ms_audio_adc_ll_active_flush_fifo(void)
{
    SET_BIT(AUDIO_ADC->CFG, AUDIO_ADC_CFG_FIFO_FLUSH);
}

/**
 * @brief  Inactive Flush Audio ADC FIFO
 * @retval None
 */
static inline void ms_audio_adc_ll_inactive_flush_fifo(void)
{
    CLEAR_BIT(AUDIO_ADC->CFG, AUDIO_ADC_CFG_FIFO_FLUSH);
}

/**
 * @brief  Mask Interrupt
 *         @arg @ref AUDIO_ADC_INT_MASK_FULL
 *         @arg @ref AUDIO_ADC_INT_MASK_EMPTY
 *         @arg @ref AUDIO_ADC_INT_MASK_WATER
 *         @arg @ref AUDIO_ADC_INT_MASK_OVERFLOW
 *         @arg @ref AUDIO_ADC_INT_MASK_UNDERFLOW
 * @retval None
 */
static inline void ms_audio_adc_ll_mask_int(uint32_t mask)
{
    SET_BIT(AUDIO_ADC->CFG, (mask << AUDIO_ADC_CFG_INT_MASK_POS));
}

/**
 * @brief  Unmask Interrupt
 *         uint32_t mask: AudioAdc Instance
 *         @arg @ref AUDIO_ADC_INT_MASK_FULL
 *         @arg @ref AUDIO_ADC_INT_MASK_EMPTY
 *         @arg @ref AUDIO_ADC_INT_MASK_WATER
 *         @arg @ref AUDIO_ADC_INT_MASK_OVERFLOW
 *         @arg @ref AUDIO_ADC_INT_MASK_UNDERFLOW
 * @retval None
 */
static inline void ms_audio_adc_ll_unmask_int(uint32_t unmask)
{
    CLEAR_BIT(AUDIO_ADC->CFG, (unmask << AUDIO_ADC_CFG_INT_MASK_POS));
}

/**
 * @brief  Get FIFO Full State
 * @retval true or not
 */
static inline bool ms_audio_adc_ll_fifo_is_full(void)
{
    return ((READ_REG(AUDIO_ADC->CFG) & (AUDIO_ADC_FIFO_STA_FULL << AUDIO_ADC_CFG_FIFO_STA_POS)) != 0);
}

/**
 * @brief  Get FIFO Empty State
 * @retval true or not
 */
static inline bool ms_audio_adc_ll_fifo_is_empty(void)
{
    return ((READ_REG(AUDIO_ADC->CFG) & (AUDIO_ADC_FIFO_STA_EMPTY << AUDIO_ADC_CFG_FIFO_STA_POS)) != 0);
}

/**
 * @brief  Get FIFO Water State
 * @retval true or not
 */
static inline bool ms_audio_adc_ll_fifo_is_water(void)
{
    return ((READ_REG(AUDIO_ADC->CFG) & (AUDIO_ADC_FIFO_STA_WATER << AUDIO_ADC_CFG_FIFO_STA_POS)) != 0);
}

/**
 * @brief  Get FIFO overflow State
 * @retval true or not
 */
static inline bool ms_audio_adc_ll_fifo_is_overflow(void)
{
    return ((READ_REG(AUDIO_ADC->CFG) & (AUDIO_ADC_FIFO_STA_OVERFLOW << AUDIO_ADC_CFG_FIFO_STA_POS)) != 0);
}

/**
 * @brief  Get FIFO underflow State
 * @retval true or not
 */
static inline bool ms_audio_adc_ll_fifo_is_underflow(void)
{
    return ((READ_REG(AUDIO_ADC->CFG) & (AUDIO_ADC_FIFO_STA_UNDERFLOW << AUDIO_ADC_CFG_FIFO_STA_POS)) != 0);
}

/**
 * @brief  Get Data From FIFO
 * @retval Data from FIFO
 */
static inline uint16_t ms_audio_adc_ll_get_data(void)
{
    return (uint16_t) (READ_REG(AUDIO_ADC->RDATA) & AUDIO_ADC_FIFO_RDATA_DATA);
}

/**
 * @brief  Set FIFO Water Level
 * @param  water_level: the FIFO Water Level
 *                      The value maybe 0 ~ 15 bytes
 * @retval none
 */
static inline void ms_audio_adc_ll_set_water_level(uint32_t water_level)
{
    MODIFY_REG(AUDIO_ADC->CFG, AUDIO_ADC_CFG_FIFO_WATER_LEVEL_MASK,
            (water_level << AUDIO_ADC_CFG_FIFO_WATER_LEVEL_POS) & AUDIO_ADC_CFG_FIFO_WATER_LEVEL_MASK);
}

/**
 * @brief  Get FIFO Water Level
 * @retval the Setting FIFO Water Level
 */
static inline uint32_t ms_audio_adc_ll_get_water_level(void)
{
    return ((READ_REG(AUDIO_ADC->CFG) & AUDIO_ADC_CFG_FIFO_WATER_LEVEL_MASK) >> AUDIO_ADC_CFG_FIFO_WATER_LEVEL_POS);
}


/**
 * @brief  Get instance
 * @retval intance
 */
static inline AudioAdc_Type* ms_audio_adc_ll_get_instance(void)
{
    return AUDIO_ADC;
}



/**
 * @}
 */

/**
 * @}
 */
#endif /* BLE_SOC_XXX_MS_audio_adc_LL_H_ */
