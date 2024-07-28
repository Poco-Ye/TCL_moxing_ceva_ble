/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  ms_audio_adc_regs.h
 * @brief
 * @author bingrui.chen
 * @date 2022年1月13日
 * @version 1.0
 * @Revision:
 */
#ifndef BLE_SOC_XXX_MS_AUDIO_ADC_REGS_H_
#define BLE_SOC_XXX_MS_AUDIO_ADC_REGS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup MS_REGISTER
 * @{
 */

/** @defgroup AudioAdc_Type Audio ADC Registers definition
 * @{
 */
typedef struct
{
    volatile uint32_t CFG;      /*!< Audio ADC Configure Registers  */
    volatile uint32_t RDATA;    /*!< Audio ADC Read Data Registers  */
} AudioAdc_Type;

/** @brief Audio ADC Control Bit Definition*/
#define AUDIO_ADC_CFG_DMA_EN_POS                                (0UL)
#define AUDIO_ADC_CFG_DMA_EN_MASK                               ((1UL) << AUDIO_ADC_CFG_DMA_EN_POS)
#define AUDIO_ADC_CFG_DMA_EN                                    AUDIO_ADC_CFG_DMA_EN_MASK

#define AUDIO_ADC_CFG_INT_EN_POS                                (1UL)
#define AUDIO_ADC_CFG_INT_EN_MASK                               ((1UL) << AUDIO_ADC_CFG_INT_EN_POS)
#define AUDIO_ADC_CFG_INT_EN                                    AUDIO_ADC_CFG_INT_EN_MASK

#define AUDIO_ADC_CFG_INT_MASK_POS                              (2UL)
#define AUDIO_ADC_CFG_INT_MASK_MASK                             ((0x1FUL) << AUDIO_ADC_CFG_INT_MASK_POS)
#define AUDIO_ADC_CFG_INT_MASK                                  AUDIO_ADC_CFG_INT_MASK_MASK

#define AUDIO_ADC_CFG_FIFO_FLUSH_POS                            (7UL)
#define AUDIO_ADC_CFG_FIFO_FLUSH_MASK                           ((1UL) << AUDIO_ADC_CFG_FIFO_FLUSH_POS)
#define AUDIO_ADC_CFG_FIFO_FLUSH                                AUDIO_ADC_CFG_FIFO_FLUSH_MASK

#define AUDIO_ADC_CFG_FIFO_WATER_LEVEL_POS                      (8UL)
#define AUDIO_ADC_CFG_FIFO_WATER_LEVEL_MASK                     ((0xFUL) << AUDIO_ADC_CFG_FIFO_WATER_LEVEL_POS)
#define AUDIO_ADC_CFG_FIFO_WATER_LEVEL                          AUDIO_ADC_CFG_FIFO_WATER_LEVEL_MASK

#define AUDIO_ADC_CFG_FIFO_STA_POS                              (12UL)
#define AUDIO_ADC_CFG_FIFO_STA_MASK                             ((0x1FUL) << AUDIO_ADC_CFG_FIFO_STA_POS)
#define AUDIO_ADC_CFG_FIFO_STA                                  AUDIO_ADC_CFG_FIFO_STA_MASK

/** @brief Audio ADC FIFO Read Data Bit Definition*/
#define AUDIO_ADC_FIFO_RDATA_DATA_POS                           (0UL)
#define AUDIO_ADC_FIFO_RDATA_DATA_MASK                          (0xFFFFUL)
#define AUDIO_ADC_FIFO_RDATA_DATA                               AUDIO_ADC_FIFO_RDATA_DATA_MASK




/**
 * @}
 */

#ifdef __cplusplus
}
#endif
#endif /* BLE_SOC_XXX_MS_AUDIO_ADC_REGS_H_ */
