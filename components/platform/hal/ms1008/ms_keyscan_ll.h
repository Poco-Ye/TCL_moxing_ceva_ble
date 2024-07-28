/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  ms_keyscan_ll.h
 * @brief
 * @author bingrui.chen
 * @date 2022-2-8
 * @version 1.0
 * @Revision:
 */
#ifndef MS_KEYSCAN_LL_H_
#define MS_KEYSCAN_LL_H_

#include "ms_interrupt.h"
#include "ms1008.h"
#include <stdint.h>
#include <stdbool.h>

/** @addtogroup MS_LL_Driver
 * @{
 */

/** @defgroup KEYSCAN_LL KEYSCAN
 * @{
 */

/** @defgroup KEYSCAN_PUSH_EVENT_OPTION        Keyscan push event option definition
 * @{
 */
#define KEYSCAN_PUSH_EVENT_LONG_REPEAT_RELEASE              0x00000000UL
#define KEYSCAN_PUSH_EVENT_ALL        				        0x00000001UL
/**
 * @}
 */

/** @defgroup KEYSCAN_SCAN_FREQ        Keyscan scan frequency definition
 * @{
 */
#define KEYSCAN_SCAN_FREQ_1_CYCLE         					0x00000000UL
#define KEYSCAN_SCAN_FREQ_2_CYCLE        				   	0x00000001UL
#define KEYSCAN_SCAN_FREQ_4_CYCLE         					0x00000002UL
#define KEYSCAN_SCAN_FREQ_8_CYCLE        				   	0x00000003UL
/**
 * @}
 */
/** @defgroup KEYSCAN_REPEAT_MULTI_TIME_TDETECT_ENABLE     Keyscan repeat multi-time detect enable definition
 * @{
 */
#define KEYSCAN_REPEAT_EN_ONE_TIME         					0x00000000UL
#define KEYSCAN_REPEAT_EN_MULTI_TIME        				0x00000001UL

/**
 * @}
 */

/** @defgroup KEYSCAN_INT_MASK     Keyscan Interrupt mask definition
 * @{
 */
#define KEYSCAN_INT_MASK_KEY_PRESS         					KEYSCAN_CTRL_KP_INT_EN
#define KEYSCAN_INT_MASK_KEY_RELEASE        				KEYSCAN_CTRL_KR_INT_EN
#define KEYSCAN_INT_MASK_KEY_REPEAT         				KEYSCAN_CTRL_RH_INT_EN
#define KEYSCAN_INT_MASK_KEY_LONG_PRESS        				KEYSCAN_CTRL_LH_INT_EN
#define KEYSCAN_INT_MASK_KEY_HOLD_TIMEOUT         			KEYSCAN_CTRL_KHTO_INT_EN
#define KEYSCAN_INT_MASK_KEY_FIFO_FULL        				KEYSCAN_CTRL_FIFO_FULL_INT_EN
/**
 * @}
 */

/**
 * @brief  Enable Keyscan CPU interrupt
 * @retval None
 */
static inline void ms_keyscan_ll_enable_cpu_int(void)
{
    INTERRUPT_ENABLE_IRQ(KEYSCAN_IRQn_to1);
}

/**
 * @brief  Disable Keyscan CPU interrupt
 * @retval None
 */
static inline void ms_keyscan_ll_disable_cpu_int(void)
{
    INTERRUPT_DISABLE_IRQ(KEYSCAN_IRQn_to1);
}

/**
 * @brief  Enable Keyscan Module
 * @retval None
 */
static inline void ms_keyscan_ll_enable_module(void)
{
    SET_BIT(KEYSCAN->ENABLE, KEYSCAN_ENABLE_CTRL_MODULE_EN);
}

/**
 * @brief  Disable Keyscan Module
 * @retval None
 */
static inline void ms_keyscan_ll_disable_module(void)
{
    CLEAR_BIT(KEYSCAN->ENABLE, KEYSCAN_ENABLE_CTRL_MODULE_EN);
}

/**
 * @brief  flush Keyscan data
 * @retval None
 */
static inline void ms_keyscan_ll_flush(void)
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_FLUSH);
}

/**
 * @brief  Set Keyscan push event to fifo option
 * @param[in] option:
 *			  KEYSCAN_PUSH_EVENT_LONG_REPEAT_RELEASE
 *            KEYSCAN_PUSH_EVENT_ALL
 * @retval None
 */
static inline void ms_keyscan_ll_set_push_event_option(uint32_t option)
{
    MODIFY_REG(KEYSCAN->CTRL, KEYSCAN_CTRL_PUSHKD_MASK, (option << KEYSCAN_CTRL_PUSHKD_POS) & KEYSCAN_CTRL_PUSHKD_MASK);
}

/**
 * @brief  Set Keyscan push event to fifo option
 * @param[in] freq:
 *			  KEYSCAN_SCAN_FREQ_1_CYCLE
 *            KEYSCAN_SCAN_FREQ_2_CYCLE
 *			  KEYSCAN_SCAN_FREQ_4_CYCLE
 *			  KEYSCAN_SCAN_FREQ_8_CYCLE
 * @retval None
 */
static inline void ms_keyscan_ll_set_scan_freq(uint32_t freq)
{
    MODIFY_REG(KEYSCAN->CTRL, KEYSCAN_CTRL_SCAN_FREQ_MASK,
            (freq << KEYSCAN_CTRL_SCAN_FREQ_POS) & KEYSCAN_CTRL_SCAN_FREQ_MASK);
}

/**
 * @brief  Set Keyscan repeat detect enable
 * @param[in] enable:
 *			  KEYSCAN_REPEAT_EN_ONE_TIME
 *            KEYSCAN_REPEAT_EN_MULTI_TIME
 * @retval None
 */
static inline void ms_keyscan_ll_set_rep_en(uint32_t enable)
{
    MODIFY_REG(KEYSCAN->CTRL, KEYSCAN_CTRL_REPEN_MASK, (enable << KEYSCAN_CTRL_REPEN_POS) & KEYSCAN_CTRL_REPEN_MASK);
}

/**
 * @brief  Enable Keyscan long press event
 * @retval None
 */
static inline void ms_keyscan_ll_enable_long_press(void)
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_LNPEN_MASK);
}

/**
 * @brief  Disable Keyscan long press event
 * @retval None
 */
static inline void ms_keyscan_ll_disable_long_press(void)
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_LNPEN_MASK);
}

/**
 * @brief  Enable Keyscan multi-key detect 
 * @retval None
 */
static inline void ms_keyscan_ll_enable_multi_key()
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_MKEN);
}

/**
 * @brief  Disable Keyscan multi-key detect 
 * @retval None
 */
static inline void ms_keyscan_ll_disable_multi_key()
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_MKEN);
}


/**
 * @brief  Enable pushkd, save  press event to FIFO
 * @retval None
 */
static inline void ms_keyscan_ll_enable_pushkd()
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_PUSHKD);
}

/**
 * @brief  Disable pushkd, not save  press event to FIFO
 * @retval None
 */
static inline void ms_keyscan_ll_disable_pushkd()
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_PUSHKD);
}



/**
 * @brief  Enable Keyscan debounce
 * @retval None
 */
static inline void ms_keyscan_ll_enable_debounce()
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_DBEN);
}

/**
 * @brief  Disable Keyscan debounce
 * @retval None
 */
static inline void ms_keyscan_ll_disable_debounce()
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_DBEN);
}

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
static inline void ms_keyscan_ll_set_interrupt_mask(uint32_t int_mask)
{
    CLEAR_BIT(KEYSCAN->CTRL, int_mask);
}

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
static inline void ms_keyscan_ll_set_interrupt_unmask(uint32_t int_unmask)
{
    SET_BIT(KEYSCAN->CTRL, int_unmask);
}

/**
 * @brief  Enable Keyscan column lat when in debounce
 * @retval None
 */
static inline void ms_keyscan_ll_enable_column_lat()
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_COL_LAT_EN);
}

/**
 * @brief  Disable Keyscan column lat when in debounce
 * @retval None
 */
static inline void ms_keyscan_ll_disable_column_lat()
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_COL_LAT_EN);
}

/**
 * @brief  clear state hold when debounce disable
 * @retval None
 */
static inline void ms_keyscan_ll_clear_hold()
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_HOLD_CLR);
}

/**
 * @brief  set long press timer
 * @retval None
 */
static inline void ms_keyscan_ll_set_long_press_time(uint32_t long_press_time)
{
    MODIFY_REG(KEYSCAN->LNP_REP_TMR, KEYSCAN_LNP_REP_TMR_LNPTIMER_MASK,
            (long_press_time << KEYSCAN_LNP_REP_TMR_LNPTIMER_POS) & KEYSCAN_LNP_REP_TMR_LNPTIMER_MASK);
}

/**
 * @brief  set repeat press timer
 * @retval None
 */
static inline void ms_keyscan_ll_set_repeat_press_time(uint32_t repeat_press_time)
{
    MODIFY_REG(KEYSCAN->LNP_REP_TMR, KEYSCAN_LNP_REP_TMR_REPTIMER_MASK,
            (repeat_press_time << KEYSCAN_LNP_REP_TMR_REPTIMER_POS) & KEYSCAN_LNP_REP_TMR_REPTIMER_MASK);
}

/**
 * @brief  get keydown event
 * @retval true or false
 */
static inline bool ms_keyscan_ll_event_is_keydown(void)
{
    return (READ_BIT(KEYSCAN->STATE,KEYSCAN_STATE_KEYDOWM) == KEYSCAN_STATE_KEYDOWM);
}

/**
 * @brief  get long press timeout event
 * @retval true or false
 */
static inline bool ms_keyscan_ll_event_is_long_press_timeout(void)
{
    return (READ_BIT(KEYSCAN->STATE,KEYSCAN_STATE_LNPTO) == KEYSCAN_STATE_LNPTO);
}

/**
 * @brief  get fifo full event
 * @retval true or false
 */
static inline bool ms_keyscan_ll_event_is_fifo_full(void)
{
    return (READ_BIT(KEYSCAN->STATE,KEYSCAN_STATE_FIFO_FULL) == KEYSCAN_STATE_FIFO_FULL);
}

/**
 * @brief  get fifo overflow event
 * @retval true or false
 */
static inline bool ms_keyscan_ll_event_is_fifo_overflow(void)
{
    return (READ_BIT(KEYSCAN->STATE,KEYSCAN_STATE_FIFO_OF) == KEYSCAN_STATE_FIFO_OF);
}

/**
 * @brief  get fifo not empty event
 * @retval true or false
 */
static inline bool ms_keyscan_ll_event_is_fifo_ne(void)
{
    return (READ_BIT(KEYSCAN->STATE,KEYSCAN_STATE_FIFO_NE) == KEYSCAN_STATE_FIFO_NE);
}

/**
 * @brief  get key hold timeout event
 * @retval true or false
 */
static inline bool ms_keyscan_ll_event_is_hold_timeout(void)
{
    return (READ_BIT(KEYSCAN->STATE,KEYSCAN_STATE_KEY_KHTOUT) == KEYSCAN_STATE_KEY_KHTOUT);
}

/**
 * @brief  get key column code
 * @retval key column code
 */
static inline uint32_t ms_keyscan_ll_get_key_col(void)
{
    return ((READ_REG(KEYSCAN->STATE) & KEYSCAN_STATE_KEY_COL_MASK) >> KEYSCAN_STATE_KEY_COL_POS);
}

/**
 * @brief  get key row code
 * @retval key row code
 */
static inline uint32_t ms_keyscan_ll_get_key_row(void)
{
    return ((READ_REG(KEYSCAN->STATE) & KEYSCAN_STATE_KEY_ROW_MASK) >> KEYSCAN_STATE_KEY_ROW_POS);
}

/**
 * @brief  get key column input
 * @retval key column input
 */
static inline uint32_t ms_keyscan_ll_get_key_col_in(void)
{
    return ((READ_REG(KEYSCAN->STATE) & KEYSCAN_STATE_KEY_KPIN_MASK) >> KEYSCAN_STATE_KEY_KPIN_POS);
}

/**
 * @brief  get key fifo rdata
 * @retval key fifo rdata
 */
static inline uint32_t ms_keyscan_ll_get_fifo_rdata(void)
{
    return (READ_REG(KEYSCAN->FIFO_RDATA) & KEYSCAN_FIFO_RDATA);
}


/**
 * @brief  get key fifo rdata coloum
 * @retval key fifo rdata
 */
static inline uint32_t ms_keyscan_ll_get_fifo_rdata_col(void)
{
    return ((READ_REG(KEYSCAN->FIFO_RDATA) & KEYSCAN_FIFO_RDATA_COL_MASK) >> KEYSCAN_FIFO_RDATA_COL_POS);
}


/**
 * @brief  get key fifo rdata rom
 * @retval key fifo rdata
 */
static inline uint32_t ms_keyscan_ll_get_fifo_rdata_row(void)
{
    return ((READ_REG(KEYSCAN->FIFO_RDATA) & KEYSCAN_FIFO_RDATA_ROW_MASK) >> KEYSCAN_FIFO_RDATA_ROW_POS);
}


/**
 * @brief  get key fifo rdata key press
 * @retval key fifo rdata
 */
static inline uint32_t ms_keyscan_ll_get_fifo_rdata_keypress(void)
{
    return ((READ_REG(KEYSCAN->FIFO_RDATA) & KEYSCAN_FIFO_RDATA_PRESS_MASK) >> KEYSCAN_FIFO_RDATA_PRESS_POS);
}

/**
 * @brief  get key fifo rdata long key
 * @retval key fifo rdata
 */
static inline uint32_t ms_keyscan_ll_get_fifo_rdata_longkey(void)
{
    return ((READ_REG(KEYSCAN->FIFO_RDATA) & KEYSCAN_FIFO_RDATA_LP_MASK) >> KEYSCAN_FIFO_RDATA_LP_POS);

}


/**
 * @brief  get key interrupt source
 * @retval key interrupt source
 */
static inline uint32_t ms_keyscan_ll_get_int_src(void)
{
    return READ_REG(KEYSCAN->INT_SRC);
}

/**
 * @brief  verify key press interrupt source
 * @retval true or false
 */
static inline bool ms_keyscan_ll_int_is_key_press(uint32_t int_src)
{
    return ((int_src & KEYSCAN_INT_SRC_KEY_PRESS) == KEYSCAN_INT_SRC_KEY_PRESS);
}

/**
 * @brief  verify key release interrupt source
 * @retval true or false
 */
static inline bool ms_keyscan_ll_int_is_key_release(uint32_t int_src)
{
    return ((int_src & KEYSCAN_INT_SRC_KEY_RELEASE) == KEYSCAN_INT_SRC_KEY_RELEASE);
}

/**
 * @brief  verify key repeat interrupt source
 * @retval true or false
 */
static inline bool ms_keyscan_ll_int_is_key_repeat(uint32_t int_src)
{
    return ((int_src & KEYSCAN_INT_SRC_REP_HIT) == KEYSCAN_INT_SRC_REP_HIT);
}

/**
 * @brief  verify key long press interrupt source
 * @retval true or false
 */
static inline bool ms_keyscan_ll_int_is_key_long_press(uint32_t int_src)
{
    return ((int_src & KEYSCAN_INT_SRC_LNP_HIT) == KEYSCAN_INT_SRC_LNP_HIT);
}

/**
 * @brief  verify key hold timeout interrupt source
 * @retval true or false
 */
static inline bool ms_keyscan_ll_int_is_key_hold_timeout(uint32_t int_src)
{
    return ((int_src & KEYSCAN_INT_SRC_KHTOUT) == KEYSCAN_INT_SRC_KHTOUT);
}

/**
 * @brief  verify fifo full interrupt source
 * @retval true or false
 */
static inline bool ms_keyscan_ll_int_is_fifo_full(uint32_t int_src)
{
    return ((int_src & KEYSCAN_INT_SRC_FIFO_FULL) == KEYSCAN_INT_SRC_FIFO_FULL);
}

/**
 * @brief  set fifo control register
 * @retval none
 */
static inline uint32_t ms_keyscan_ll_set_fifo_ctrl(uint32_t option)
{
    return WRITE_REG(KEYSCAN->FIFO_CTRL, option);
}

/**
 * @brief  set fifo debounce_length
 * @retval none
 */
static inline void ms_keyscan_ll_set_debounce_length(uint32_t length)
{
    MODIFY_REG(KEYSCAN->ENABLE, KEYSCAN_ENABLE_CTRL_DBCNT_MASK,
            (length << KEYSCAN_ENABLE_CTRL_DBCNT_POS) & KEYSCAN_ENABLE_CTRL_DBCNT_MASK);
}

/**
 * @brief  set fifo control register
 * @retval none
 */
static inline void ms_keyscan_ll_set_debonce_timeout(uint32_t timeout)
{
    MODIFY_REG(KEYSCAN->ENABLE, KEYSCAN_ENABLE_CTRL_DBPRD_MASK,
            (timeout << KEYSCAN_ENABLE_CTRL_DBPRD_POS) & KEYSCAN_ENABLE_CTRL_DBPRD_MASK);
}

/**
 * @}
 */

/**
 * @}
 */

#endif /* MS_KEYSCAN_LL_H_ */
