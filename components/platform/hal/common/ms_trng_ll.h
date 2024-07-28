/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_trngl_ll.h
 * @brief Header file of TRNG  module.
 * @author haijun.mai
 * @date   2021-01-06
 * @version 1.0
 * @Revision
 */

#ifndef MS_TRNG_LL_H_
#define MS_TRNG_LL_H_

#include <ms1008.h>
#include "ms_trng_regs.h"
#include <stdint.h>
#include <stdbool.h>


/*define random data read fron which data reg*/
#define TRNG_DATA0_SEL        (0)
#define TRNG_DATA1_SEL        (1)
#define TRNG_DATA2_SEL        (2)
#define TRNG_DATA3_SEL        (3)
#define TRNG_DATA4_SEL        (4)
#define TRNG_DATA5_SEL        (5)
#define TRNG_DATA6_SEL        (6)

/**
 * @brief This SAMPLE_CNT register stores the number of rng_clk cycles between two consecutive ROSC samples.
 * @param  Trng_Type *trng: TRNG regs
 * @param  uint32_t sample_rate:
 * @retval None
 */
static inline void ms_trng_set_sample_rate_ll(Trng_Type *trng,uint32_t sample_rate)
{
	WRITE_REG(trng->SC1,sample_rate);
}


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
static inline void ms_trng_clear_interrupt_status_ll(Trng_Type *trng,uint32_t interrupt_clear)
{
	SET_BIT(trng->ICR,interrupt_clear);
}

/**
 * @brief Defines the length of the oscillator ring(=the number of inverters )out of four possible selections.
 * @param  Trng_Type *trng: TRNG regs
 * @param  uint32_t rnd_src_sel:0-3
 * @param   This parameter can be one of the following values:0-3
 * @retval none
 */
static inline void ms_trng_rnd_src_sel_ll(Trng_Type *trng,uint32_t rnd_src_sel)
{
	WRITE_REG(trng->CONFIG,(rnd_src_sel&TRNG_CONFIG_RND_SRC_SEL));
}

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
static inline void ms_trng_set_debug_control_ll(Trng_Type *trng,uint32_t debug_control_sel)
{
	//WRITE_REG(trng->TDC,debug_control_sel);
	WRITE_REG(trng->TDC,0xffffffff);
}


/**
 * @brief trng source enable
 * @param  Trng_Type *trng: TRNG regs
 * @retval none
 */static inline void ms_trng_source_enablel_ll(Trng_Type *trng)
{
     SET_BIT(trng->RSEN, TRNG_RSEN_RND_SRC_EN);
}

/**
 * @brief trng source disable
 * @param  Trng_Type *trng: TRNG regs
 * @retval none
 */
static inline void ms_trng_source_disablel_ll(Trng_Type *trng)
{
	CLEAR_BIT(trng->RSEN, TRNG_RSEN_RND_SRC_EN);
}


/**
 * @brief trng get random data valid status
 * @param  Trng_Type *trng: TRNG regs
 * @retval random data valid status
 */
static inline uint32_t ms_trng_get_data_valid_status_ll(Trng_Type *trng)
{
    return (READ_REG(trng->VALID));
}


/**
 * @brief  get trng interrupt status
 * @param  Trng_Type *trng: TRNG regs
 * @ uint8_t interrupt_mask: interrupt mask
 * @retval 0 or 1
 */
static inline uint8_t ms_trng_get_interrupt_status_ll(Trng_Type *trng,uint8_t interrupt_mask)
{
    return (READ_REG(trng->ISR)&interrupt_mask);
}


/**
 * @brief  mask trng interrupt
 * @param  Trng_Type *trng: TRNG regs
 * @param  uint32_t interrupt_mask: 
 * @retval none
 */
static inline void ms_trng_interrupt_mask_ll(Trng_Type *trng,uint32_t interrupt_mask)
{
	SET_BIT(trng->IMR,interrupt_mask);
}

/**
 * @brief  unmask trng interrupt
 * @param  Trng_Type *trng: TRNG regs
 * @param  uint32_t interrupt_mask: 
 * @retval none
 */
static inline void ms_trng_interrupt_unmask_ll(Trng_Type *trng,uint32_t interrupt_mask)
{
	CLEAR_BIT(trng->IMR,interrupt_mask);
}


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
extern uint32_t ms_trng_get_data_ll(Trng_Type *trng,uint8_t data_x);







#endif /* MS_TRNG_LL_H_ */
