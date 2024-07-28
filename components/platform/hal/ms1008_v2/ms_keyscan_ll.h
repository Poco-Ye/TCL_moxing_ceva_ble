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

    typedef unsigned int        UINT32;
    typedef unsigned short      UINT16;
    typedef unsigned char       UINT8;

    #define inl(addr)           (*((volatile UINT32 *)(addr)))
    #define inw(addr)           (*((volatile UINT16 *)(addr)))
    #define inb(addr)           (*((volatile UINT8  *)(addr)))

    #define outl(addr, val)     (*((volatile UINT32 *)(addr)) = (val))
    #define outw(addr, val)     (*((volatile UINT16 *)(addr)) = (val))
    #define outb(addr, val)     (*((volatile UINT8  *)(addr)) = (val))

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

/**
 * @}
 */

/**
 * @brief  Enable Keyscan CPU interrupt
 * @retval None
 */
static inline void ms_keyscan_ll_enable_cpu_int(void)
{
    INTERRUPT_ENABLE_IRQ(KEYSCAN_IRQn);
}

/**
 * @brief  Disable Keyscan CPU interrupt
 * @retval None
 */
static inline void ms_keyscan_ll_disable_cpu_int(void)
{
    INTERRUPT_DISABLE_IRQ(KEYSCAN_IRQn);
}

/**
 * @brief  Enable Keyscan Module [0:0]
 * @retval None
 */
static inline void ms_keyscan_ll_enable_module(void)
{
    //SET_BIT(KEYSCAN->CTRL, KEYSCAN_ENABLE_CTRL_MODULE_EN);
    uint32_t reg = inl(&(KEYSCAN->CTRL));
    outl(&(KEYSCAN->CTRL), reg | KEYSCAN_ENABLE_CTRL_MODULE_EN);
}

/**
 * @brief  Disable Keyscan Module [0:0]
 * @retval None
 */
static inline void ms_keyscan_ll_disable_module(void)
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_ENABLE_CTRL_MODULE_EN);
}

/**
 * @brief  Enable Keyscan long press event [1:1]
 * @retval None
 */
static inline void ms_keyscan_ll_enable_long_press(void)
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_LNPEN);
}

/**
 * @brief  Disable Keyscan long press event [1:1]
 * @retval None
 */
static inline void ms_keyscan_ll_disable_long_press(void)
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_LNPEN);
}

/**
 * @brief  Enable Keyscan long press repeat event [2:2]
 * @retval None
 */
static inline void ms_keyscan_ll_enable_long_repeat_press(void)
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_LONG_REPEN);
}

/**
 * @brief  Disable Keyscan long press repeat event [2:2]
 * @retval None
 */
static inline void ms_keyscan_ll_disable_long_repeat_press(void)
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_LONG_REPEN);
}


/**
 * @brief  key detect write fifo enable[3:3]
 * @retval None
 */
static inline void ms_keyscan_ll_enable_press_fifo(void)
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_PRESS_FIFOEN);
}

/**
 * @brief  key detect write fifo disable [3:3]
 * @retval None
 */
static inline void ms_keyscan_ll_disable_press_fifo(void)
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_PRESS_FIFOEN);
}


/**
 * @brief  key rlease write fifo enable[4:4]
 * @retval None
 */
static inline void ms_keyscan_ll_enable_release_fifo(void)
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_RELEASE_FIFOEN);
}

/**
 * @brief  key rlease write fifo disable [4:4]
 * @retval None
 */
static inline void ms_keyscan_ll_disable_release_fifo(void)
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_RELEASE_FIFOEN);
}


/**
 * @brief  key long press write fifo enable[5:5]
 * @retval None
 */
static inline void ms_keyscan_ll_enable_long_press_fifo(void)
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_LPRESS_FIFOEN);
}

/**
 * @brief  key long press write fifo disable [5:5]
 * @retval None
 */
static inline void ms_keyscan_ll_disable_long_press_fifo(void)
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_LPRESS_FIFOEN);
}

/**
 * @brief long press repeat write  fifo enalbe[6:6]
 * @retval None
 */
static inline void ms_keyscan_ll_enable_repeat_long_press_fifo(void)
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_LRPRESS_FIFOEN);
}

/**
 * @brief  long press repeat write  fifo disable [6:6]
 * @retval None
 */
static inline void ms_keyscan_ll_disable_repeat_long_press_fifo(void)
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_LRPRESS_FIFOEN);
}

/**
 * @brief  press timeout write  fifo enalbe[7:7]
 * @retval None
 */
static inline void ms_keyscan_ll_enable_press_fifo_timeout(void)
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_TIMEOUT_FIFOEN);
}

/**
 * @brief   press timeout write  fifo disable [7:7]
 * @retval None
 */
static inline void ms_keyscan_ll_disable_press_fifo_timeout(void)
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_TIMEOUT_FIFOEN);
}


/**
 * @brief  press debounce  enalbe[8:8]
 * @retval None
 */
static inline void ms_keyscan_ll_enable_debounce(void)
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_DEBOUNCEEN);
}

/**
 * @brief   press debounce disable [8:8]
 * @retval None
 */
static inline void ms_keyscan_ll_disable_debounce(void)
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_DEBOUNCEEN);
}

/**
 * @brief  press timeout enalbe[9:9]
 * @retval None
 */
static inline void ms_keyscan_ll_enable_press_timeout(void)
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_TIMEOUTEN);
}

/**
 * @brief   press timeout disable [9:9]
 * @retval None
 */
static inline void ms_keyscan_ll_disable_press_timeout(void)
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_TIMEOUTEN);
}

/**
 * @brief  kpad int enalbe[10:10]
 * @retval None
 */
static inline void ms_keyscan_ll_enable_kpad_int(void)
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_KPADINTEN);
}

/**
 * @brief   kpad int disable [10:10]
 * @retval None
 */
static inline void ms_keyscan_ll_disable_kpad_int(void)
{
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_KPADINTEN);
}

/**
 * @brief  scan interval period [17:11]
 * @retval None
 */
static inline void ms_keyscan_ll_config_scan_period(uint32_t val)
{
    uint32_t reg = READ_REG(KEYSCAN->CTRL);
    WRITE_REG(KEYSCAN->CTRL, KEYSCAN_CTRL_SCAN_PERIOD(val) | reg);
}


/**
 * @brief  fifo flush [18:18]
 * @retval None
 */
static inline void ms_keyscan_ll_fifo_flush(void)
{
    SET_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_FIFO_CLEAR);
    CLEAR_BIT(KEYSCAN->CTRL, KEYSCAN_CTRL_FIFO_CLEAR);
}


/**
 * @brief  config key down debounce value [31:0]
 * @retval None
 */
static inline void ms_keyscan_ll_config_press_debounce_value(uint32_t val)
{
    WRITE_REG(KEYSCAN->DOWN_DB, KEYSCAN_PRESS_DB_VALUE(val));
}


/**
 * @brief  config key release debounce value [31:0]
 * @retval None
 */
static inline void ms_keyscan_ll_config_release_debounce_value(uint32_t val)
{
    WRITE_REG(KEYSCAN->RELEASE_DB, KEYSCAN_RELEASE_DB_VALUE(val));
}

/**
 * @brief  config key  debounce  timeout value [31:0]
 * @retval None
 */
static inline void ms_keyscan_ll_config_debounce_timeout_value(uint32_t val)
{
    WRITE_REG(KEYSCAN->DB_TIMEOUT , KEYSCAN_TIMEOUT_DB_VALUE(val));
}

/**
 * @brief  config key  press  timeout value [31:0]
 * @retval None
 */
static inline void ms_keyscan_ll_config_press_timeout_value(uint32_t val)
{
    WRITE_REG(KEYSCAN->PRESS_TIMEOUT , KEYSCAN_TIMEOUT_PRESS_VALUE(val));
}



/**
 * @brief  get key fifo rdata
 * @retval key fifo rdata
 */
static inline uint32_t ms_keyscan_ll_get_fifo_rdata(void)
{
    return KEYSCAN_KPAD_RDATA(KEYSCAN->FIFO_RD);
}



/**
 * @brief  get key fifo rdata coloum
 * @retval key col
 */
static inline uint32_t ms_keyscan_ll_get_fifo_rdata_col(void)
{
    return KEYSCAN_KPAD_COL(KEYSCAN->FIFO_RD);
}


/**
 * @brief  get key fifo rdata row
 * @retval key row
 */
static inline uint32_t ms_keyscan_ll_get_fifo_rdata_row(void)
{
    return KEYSCAN_KPAD_ROW(KEYSCAN->FIFO_RD);
}

/**
 * @brief  get key fifo rdata event
 * @retval key event
 */
static inline uint32_t ms_keyscan_ll_get_fifo_rdata_event(void)
{
    return KEYSCAN_KEY_EVEN(KEYSCAN->FIFO_RD);
}


/**
 * @brief  config fifo watermark
 * @retval none
 */
static inline void ms_keyscan_ll_config_fifo_watermark(uint32_t val)
{
    WRITE_REG(KEYSCAN->FIFO_WATERMARK_MARK, KEYSCAN_AFULL_WATER_MARK(val));
}


/**
 * @brief  key press interrupt enable, clear bit is enable!!![0:0]
 * @retval none
 */
static inline void ms_keyscan_ll_enable_int_press(void)
{
    CLEAR_BIT(KEYSCAN->KPAD_INT_MASK, KEYSCAN_INT_KEYPRESSEN);
}

/**
 * @brief  key press interrupt disable, set bit is enable!!![0:0]
 * @retval none
 */
static inline void ms_keyscan_ll_disable_int_press(void)
{
    SET_BIT(KEYSCAN->KPAD_INT_MASK, KEYSCAN_INT_KEYPRESSEN);
}

/**
 * @brief  key release interrupt enable, clear bit is enable!!![1:1]
 * @retval none
 */
static inline void ms_keyscan_ll_enable_int_release(void)
{
    CLEAR_BIT(KEYSCAN->KPAD_INT_MASK, KEYSCAN_INT_KEYRELEASEEN);
}

/**
 * @brief  key release interrupt disable, set bit is enable!!![1:1]
 * @retval none
 */
static inline void ms_keyscan_ll_disable_int_release(void)
{
    SET_BIT(KEYSCAN->KPAD_INT_MASK, KEYSCAN_INT_KEYRELEASEEN);
}

/**
 * @brief  key long press interrupt enable, clear bit is enable!!![2:2]
 * @retval none
 */
static inline void ms_keyscan_ll_enable_int_long_press(void)
{
    CLEAR_BIT(KEYSCAN->KPAD_INT_MASK, KEYSCAN_INT_LONG_KEYPRESSEN);
}

/**
 * @brief  key long press  interrupt disable, set bit is enable!!![2:2]
 * @retval none
 */
static inline void ms_keyscan_ll_disable_int_long_press(void)
{
    SET_BIT(KEYSCAN->KPAD_INT_MASK, KEYSCAN_INT_LONG_KEYPRESSEN);
}


/**
 * @brief  key press timout interrupt enable, clear bit is enable!!![3:3]
 * @retval none
 */
static inline void ms_keyscan_ll_enable_int_press_timeout(void)
{
    CLEAR_BIT(KEYSCAN->KPAD_INT_MASK, KEYSCAN_INT_PRESS_TIMEOUTEN);
}

/**
 * @brief  key press timout interrupt disable, set bit is enable!!![3:3]
 * @retval none
 */
static inline void ms_keyscan_ll_disable_int_press_timeout(void)
{
    SET_BIT(KEYSCAN->KPAD_INT_MASK, KEYSCAN_INT_PRESS_TIMEOUTEN);
}


/**
 * @brief  key fifo mark interrupt enable, clear bit is enable!!![4:4]
 * @retval none
 */
static inline void ms_keyscan_ll_enable_int_fifomark(void)
{
    CLEAR_BIT(KEYSCAN->KPAD_INT_MASK, KEYSCAN_INT_WATERMARKEN);
}

/**
 * @brief  key fifo mark interrupt disable, set bit is enable!!![4:4]
 * @retval none
 */
static inline void ms_keyscan_ll_disable_int_fifomark(void)
{
    SET_BIT(KEYSCAN->KPAD_INT_MASK, KEYSCAN_INT_WATERMARKEN);
}


/**
 * @brief  key fifo empty interrupt enable, clear bit is enable!!![5:5]
 * @retval none
 */
static inline void ms_keyscan_ll_enable_int_fifoempty(void)
{
    CLEAR_BIT(KEYSCAN->KPAD_INT_MASK, KEYSCAN_INT_FIFO_EMPTY);
}

/**
 * @brief  key fifo empty interrupt disable, set bit is enable!!![5:5]
 * @retval none
 */
static inline void ms_keyscan_ll_disable_int_fifoempty(void)
{
    SET_BIT(KEYSCAN->KPAD_INT_MASK, KEYSCAN_INT_FIFO_EMPTY);
}


/**
 * @brief  key fifo full interrupt enable, clear bit is enable!!![6:6]
 * @retval none
 */
static inline void ms_keyscan_ll_enable_int_fifofull(void)
{
    CLEAR_BIT(KEYSCAN->KPAD_INT_MASK, KEYSCAN_INT_FIFO_FULL);
}

/**
 * @brief  key fifo full interrupt disable, set bit is enable!!![6:6]
 * @retval none
 */
static inline void ms_keyscan_ll_disable_int_fifofull(void)
{
    SET_BIT(KEYSCAN->KPAD_INT_MASK, KEYSCAN_INT_FIFO_FULL);
}


/**
 * @brief  key press int status[0:0]
 * @retval  status
 */
static inline bool ms_keyscan_ll_int_press_status(void)
{
    return READ_BIT(KEYSCAN->KPAD_INT_SRC, KEYSCAN_INT_FLAG_KEYPRESS_MASK);
}


/**
 * @brief  key release int status [1:1]
 * @retval status
 */
static inline bool ms_keyscan_ll_int_release_status(void)
{
    return READ_BIT(KEYSCAN->KPAD_INT_SRC, KEYSCAN_INT_FLAG_KEYRELEASE_MASK);
}

/**
 * @brief  key long press int status [2:2]
 * @retval status
 */
static inline bool ms_keyscan_ll_int_long_press_status(void)
{
    return READ_BIT(KEYSCAN->KPAD_INT_SRC, KEYSCAN_INT_FLAG_KEYLONGPRESS_MASK);
}

/**
 * @brief  key timeout press int status [3:3]
 * @retval status
 */
static inline bool ms_keyscan_ll_int_press_timeout_status(void)
{
    return READ_BIT(KEYSCAN->KPAD_INT_SRC, KEYSCAN_INT_FLAG_PRESSTIMEOUT_MASK);
}

/**
 * @brief  key fifo mark int status [4:4]
 * @retval status
 */
static inline bool ms_keyscan_ll_int_fifomark_status(void)
{
    return READ_BIT(KEYSCAN->KPAD_INT_SRC, KEYSCAN_INT_FLAG_FIFOMARK_MASK);
}


/**
 * @brief  key fifo empty int status [5:5]
 * @retval status
 */
static inline bool ms_keyscan_ll_int_fifoempty_status(void)
{
    return READ_BIT(KEYSCAN->KPAD_INT_SRC, KEYSCAN_INT_FLAG_FIFOEMPTY_MASK); // test
}

/**
 * @brief  key fifo full int status [6:6]
 * @retval status
 */
static inline bool ms_keyscan_ll_int_fifofull_status(void)
{
    return READ_BIT(KEYSCAN->KPAD_INT_SRC, KEYSCAN_INT_FLAG_FIFOFULL_MASK);
}


/**
 * @brief  key int status clear press[0:0]
 * @retval none
 */
static inline void ms_keyscan_ll_int_press_clear(void)
{
     SET_BIT(KEYSCAN->KPAD_INT_PND, KEYSCAN_INT_KEYPRESS_PND);
}

/**
 * @brief  key int status clear release[1:1]
 * @retval none
 */
static inline void ms_keyscan_ll_int_release_clear(void)
{
     SET_BIT(KEYSCAN->KPAD_INT_PND, KEYSCAN_INT_KEYRELEASE_PND);
}

/**
 * @brief  key int status clear long press[2:2]
 * @retval none
 */
static inline void ms_keyscan_ll_int_long_press_clear(void)
{
     SET_BIT(KEYSCAN->KPAD_INT_PND, KEYSCAN_INT_KEYLONGPRESS_PND);
}


/**
 * @brief  key int status clear timeout[3:3]
 * @retval none
 */
static inline void ms_keyscan_ll_int_press_timeout_clear(void)
{
     SET_BIT(KEYSCAN->KPAD_INT_PND, KEYSCAN_INT_KEYPRESSTIMEOUT_PND);
}

/**
 * @brief  get keypad all key state [29:0]
 * @retval key state
 */
static inline uint32_t ms_keyscan_ll_scan_kpad_state(void)
{
    return  (READ_REG(KEYSCAN->KPAD_KEY_STAT) & KEYSCAN_STA_KEYPRESS_MASK);
}

/**
 * @brief  get key fifo rdata event [29:0]
 * @retval key num
 */
static inline uint32_t ms_keyscan_ll_keyvalue_to_kpadnum(uint32_t row, uint32_t  col)
{
    uint32_t reg = ms_keyscan_ll_scan_kpad_state();
    return KEYSCAN_STA_KEYPRESS(reg, row , col);
}


/**
 * @brief  get key fifo rdata event [31:0]
 * @retval key event
 */
static inline void ms_keyscan_ll_config_long_press_value(uint32_t val)
{
    WRITE_REG(KEYSCAN->KPAD_LP_VALUE, val);
}


/**
 * @brief  kpad peri sel
 * @retval key event
 */
static inline void ms_keyscan_ll_config_sel(void)
{
    SET_BIT(SYS_CTRL->PERI_SEL, KEYSCAN_PERI_SEL_MASK);
}


static inline void ms_keyscan_ll_soft_rst(void)
{
    SET_BIT(SYS_CTRL->SOFT_RST, KEYSCAN_SOFT_RST_MASK);
    delay(10);
    CLEAR_BIT(SYS_CTRL->SOFT_RST, KEYSCAN_SOFT_RST_MASK);
}

/**
 * @}
 */

/**
 * @}
 */

#endif /* MS_KEYSCAN_LL_H_ */
