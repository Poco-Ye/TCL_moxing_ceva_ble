/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_trngl_hal.h
 * @brief Header file of TRNG  module.
 * @author haijun.mai
 * @date   2021-01-07
 * @version 1.0
 * @Revision
 */

#ifndef MS_TRNG_HAL_H_
#define MS_TRNG_HAL_H_

#include "ms_trng_ll.h"


/**
 * @brief This SAMPLE_CNT register stores the number of rng_clk cycles between two consecutive ROSC samples.
 * @param  Trng_Type *trng: TRNG regs
 * @param  uint32_t sample_rate:
 * @retval None
 */
#define ms_trng_set_sample_rate_hal(trng, sample_rate)        ms_trng_set_sample_rate_ll(trng, sample_rate)


/**
 * @brief clear trng interrupt status
 * @param  Trng_Type *trng: TRNG regs
 * @param  uint32_t interrupt_clear:
 * @param   This parameter can be one of the following values:
 * @ TRNG_ICR_EHR_VALID
 * @ TRNG_ICR_AUTOCORR_ERR
 * @ TRNG_ICR_CRNGT_ERR
 * @ TRNG_ICR_VN_ERR
 * @retval none
 */
#define ms_trng_clear_interrupt_status_hal(trng, interrupt_clear)        ms_trng_clear_interrupt_status_ll(trng, interrupt_clear)


/**
 * @brief Defines the length of the oscillator ring(=the number of inverters )out of four possible selections.
 * @param  Trng_Type *trng: TRNG regs
 * @param  uint32_t rnd_src_sel:0-3
 * @param   This parameter can be one of the following values:0-3
 * @retval none
 */
#define  ms_trng_rnd_src_sel_hal(trng,rnd_src_sel)        ms_trng_rnd_src_sel_ll(trng,rnd_src_sel)


/**
 * @brief controls the debug behavior of the TRNG
 * @param  Trng_Type *trng: TRNG regs
 * @param  uint32_t debug_control_sel:[1]	VNC_BYPASS [2]	TRNG_CRNGT_BYPASS [3]	AUTO_CORRELATE_BYPASS
 * @param   This parameter can be one of the following values:
 * @ TRNG_TDC_VNC_BYPASS
 * @ TRNG_TDC_TRNG_CRNGT_BYPASS
 * @ TRNG_TDC_AUTO_CORRELATE_BYPASS
 * @retval none
 */
#define ms_trng_set_debug_control_hal(trng, debug_control_sel)        ms_trng_set_debug_control_ll(trng, debug_control_sel)


/**
 * @brief trng source enable
 * @param  Trng_Type *trng: TRNG regs
 * @retval none
 */
#define  ms_trng_source_enablel_hal(trng)        ms_trng_source_enablel_ll(trng)


/**
 * @brief trng source disable
 * @param  Trng_Type *trng: TRNG regs
 * @retval none
 */
#define ms_trng_source_disablel_hal(trng)       ms_trng_source_disablel_ll(trng)


/**
 * @brief trng get random data valid status
 * @param  Trng_Type *trng: TRNG regs
 * @retval random data valid status
 */
#define ms_trng_get_data_valid_status_hal(trng)        ms_trng_get_data_valid_status_ll(trng)


/**
 * @brief  get trng interrupt status
 * @param  Trng_Type *trng: TRNG regs
 * @ uint8_t interrupt_mask: interrupt mask
 * @retval 0 or 1
 */
#define ms_trng_get_interrupt_status_hal(trng,interrupt_mask)         ms_trng_get_interrupt_status_ll(trng, interrupt_mask)

/**
 * @brief  mask trng interrupt
 * @param  Trng_Type *trng: TRNG regs
 * @param  uint32_t interrupt_mask: 
 * @retval none
 */
#define ms_trng_interrupt_mask_hal(trng,interrupt_mask)        ms_trng_interrupt_mask_ll(trng, interrupt_mask)

/**
 * @brief  unmask trng interrupt
 * @param  Trng_Type *trng: TRNG regs
 * @param  uint32_t interrupt_mask: 
 * @retval none
 */
#define ms_trng_interrupt_unmask_hal(trng,interrupt_mask)        ms_trng_interrupt_unmask_ll(trng, interrupt_mask)

/**
 * @brief  Trng get random data
 * @param  Trng_Type *trng: TRNG regs
 * @param  uint8_t data_x: select the random data ,data_x is 0-6 
 * @param   This parameter can be one of the following values:
 * @TRNG_DATA0_SEL
 * @TRNG_DATA1_SEL
 * @TRNG_DATA2_SEL
 * @TRNG_DATA3_SEL
 * @TRNG_DATA4_SEL
 * @TRNG_DATA5_SEL
 * @TRNG_DATA6_SEL
 * @retval the random data
 */
#define ms_trng_get_data_hal(trng,data_x)        ms_trng_get_data_ll(trng,data_x)

#endif/* MS_TRNG_HAL_H_ */

