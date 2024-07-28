/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  ms_keyscan_regs.h
 * @brief
 * @author bingrui.chen
 * @date 2022-2-8
 * @version 1.0
 * @Revision:
 */
#ifndef MS_KEYSCAN_REGS_H_
#define MS_KEYSCAN_REGS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup MS_REGISTER
 * @{
 */

/** @defgroup Keyscan_Type Keyscan Registers definition
 * @{
 */
typedef struct
{
    volatile uint32_t CTRL;         /*!< Keyscan Configure Registers  */
    volatile uint32_t DOWN_DB; 
    volatile uint32_t RELEASE_DB;
    volatile uint32_t DB_TIMEOUT;
    volatile uint32_t PRESS_TIMEOUT;
    volatile uint32_t FIFO_RD;
    volatile uint32_t FIFO_WATERMARK_MARK;
    volatile uint32_t KPAD_INT_MASK;
    volatile uint32_t KPAD_INT_SRC;
    volatile uint32_t KPAD_INT_PND;
    volatile uint32_t KPAD_KEY_STAT;
    volatile uint32_t KPAD_LP_VALUE;  /*!< long press value */
} Keyscan_Type;



/**
 **  REGISTER: kp_ctrl_csr, offset 0x00
 **/
/** keypad module enable, high active*/
#define KEYSCAN_ENABLE_CTRL_MODULE_EN_POS                     (0UL)
#define KEYSCAN_ENABLE_CTRL_MODULE_EN_MASK                  ((1UL) << KEYSCAN_ENABLE_CTRL_MODULE_EN_POS)
#define KEYSCAN_ENABLE_CTRL_MODULE_EN                             KEYSCAN_ENABLE_CTRL_MODULE_EN_MASK


/** long press enbale */
#define KEYSCAN_CTRL_LNPEN_POS                            	         (1UL)
#define KEYSCAN_CTRL_LNPEN_MASK                           	         ((1UL) << KEYSCAN_CTRL_LNPEN_POS)
#define KEYSCAN_CTRL_LNPEN                                	                KEYSCAN_CTRL_LNPEN_MASK


/** repeat detect enable */
#define KEYSCAN_CTRL_LONG_REPEN_POS                                (2UL)
#define KEYSCAN_CTRL_LONG_REPEN_MASK                      	 ((1UL) << KEYSCAN_CTRL_LONG_REPEN_POS)
#define KEYSCAN_CTRL_LONG_REPEN                                        KEYSCAN_CTRL_LONG_REPEN_MASK


/** key detect write fifo enable */
#define KEYSCAN_CTRL_PRESS_FIFOEN_POS                              (3UL)
#define KEYSCAN_CTRL_PRESS_FIFOEN_MASK                           ((1UL) << KEYSCAN_CTRL_PRESS_FIFOEN_POS)
#define KEYSCAN_CTRL_PRESS_FIFOEN                                	  KEYSCAN_CTRL_PRESS_FIFOEN_MASK


/** key release write fifo enable */
#define KEYSCAN_CTRL_RELEASE_FIFOEN_POS                          (4UL)
#define KEYSCAN_CTRL_RELEASE_FIFOEN_MASK                       ((1UL) << KEYSCAN_CTRL_RELEASE_FIFOEN_POS)
#define KEYSCAN_CTRL_RELEASE_FIFOEN                                  KEYSCAN_CTRL_RELEASE_FIFOEN_MASK


/** long key press write fifo enable */
#define KEYSCAN_CTRL_LPRESS_FIFOEN_POS                            (5UL)
#define KEYSCAN_CTRL_LPRESS_FIFOEN_MASK                         ((1UL) << KEYSCAN_CTRL_LPRESS_FIFOEN_POS)
#define KEYSCAN_CTRL_LPRESS_FIFOEN                                    KEYSCAN_CTRL_LPRESS_FIFOEN_MASK


/** long key press repeat write fifo enable */
#define KEYSCAN_CTRL_LRPRESS_FIFOEN_POS                          (6UL)
#define KEYSCAN_CTRL_LRPRESS_FIFOEN_MASK                       ((1UL) << KEYSCAN_CTRL_LRPRESS_FIFOEN_POS)
#define KEYSCAN_CTRL_LRPRESS_FIFOEN                                  KEYSCAN_CTRL_LRPRESS_FIFOEN_MASK


/** key press timeout  write fifo enable */
#define KEYSCAN_CTRL_TIMEOUT_FIFOEN_POS                          (7UL)
#define KEYSCAN_CTRL_TIMEOUT_FIFOEN_MASK                       ((1UL) << KEYSCAN_CTRL_TIMEOUT_FIFOEN_POS)
#define KEYSCAN_CTRL_TIMEOUT_FIFOEN                                  KEYSCAN_CTRL_TIMEOUT_FIFOEN_MASK


/** debounce enalbe */
#define KEYSCAN_CTRL_DEBOUNCEEN_POS                                 (8UL)
#define KEYSCAN_CTRL_DEBOUNCEEN_MASK                              ((1UL) << KEYSCAN_CTRL_DEBOUNCEEN_POS)
#define KEYSCAN_CTRL_DEBOUNCEEN                                         KEYSCAN_CTRL_DEBOUNCEEN_MASK


/** key down timeout enable */
#define KEYSCAN_CTRL_TIMEOUTEN_POS                                   (9UL)
#define KEYSCAN_CTRL_TIMEOUTEN_MASK                                 ((1UL) << KEYSCAN_CTRL_TIMEOUTEN_POS)
#define KEYSCAN_CTRL_TIMEOUTEN                                            KEYSCAN_CTRL_TIMEOUTEN_MASK


/** keypad interrupt enable */
#define KEYSCAN_CTRL_KPADINTEN_POS                                   (10UL)
#define KEYSCAN_CTRL_KPADINTEN_MASK                                ((1UL) << KEYSCAN_CTRL_KPADINTEN_POS)
#define KEYSCAN_CTRL_KPADINTEN                                           KEYSCAN_CTRL_KPADINTEN_MASK


/** scan interval period */
#define KEYSCAN_CTRL_SCAN_PERIOD_POS                               (11UL)
#define KEYSCAN_CTRL_SCAN_PERIOD_MASK                             (0x7FUL)
#define KEYSCAN_CTRL_SCAN_PERIOD(x)                                   (((x) & KEYSCAN_CTRL_SCAN_PERIOD_MASK) << KEYSCAN_CTRL_SCAN_PERIOD_POS)


/** keypad clear fifo */
#define KEYSCAN_CTRL_FIFO_CLEAR_POS                                   (18UL)
#define KEYSCAN_CTRL_FIFO_CLEAR_MASK                                ((1UL) << KEYSCAN_CTRL_FIFO_CLEAR_POS)
#define KEYSCAN_CTRL_FIFO_CLEAR                                           KEYSCAN_CTRL_FIFO_CLEAR_MASK


/**
 **  REGISTER: kp_kd_db_val, 0ffset:0x04  
 **/
 
/** key down debounce value */
#define KEYSCAN_PRESS_DB_VALUE_POS                                   (0UL)
#define KEYSCAN_PRESS_DB_VALUE_MASK                                (0xFFFFFFFFUL)
#define KEYSCAN_PRESS_DB_VALUE(x)                                      (((x) & KEYSCAN_PRESS_DB_VALUE_MASK) << KEYSCAN_PRESS_DB_VALUE_POS)


/**
 **  REGISTER: kp_kr_db_val, 0ffset:0x08  
 **/
 
/** key release debounce value */
#define KEYSCAN_RELEASE_DB_VALUE_POS                               (0UL)
#define KEYSCAN_RELEASE_DB_VALUE_MASK                            (0xFFFFFFFFUL)
#define KEYSCAN_RELEASE_DB_VALUE(x)                                  (((x) & KEYSCAN_RELEASE_DB_VALUE_MASK) << KEYSCAN_RELEASE_DB_VALUE_POS)


/**
 **  REGISTER: kp_db_timeout_val, 0ffset:0x0C  
 **/
 
/** key release debounce value */
#define KEYSCAN_TIMEOUT_DB_VALUE_POS                               (0UL)
#define KEYSCAN_TIMEOUT_DB_VALUE_MASK                            (0xFFFFFFFFUL)
#define KEYSCAN_TIMEOUT_DB_VALUE(x)                                  (((x) & KEYSCAN_TIMEOUT_DB_VALUE_MASK) << KEYSCAN_TIMEOUT_DB_VALUE_POS)


/**
 **  REGISTER: kp_kd_timeout_val, 0ffset:0x10  
 **/
 
/** key release down value */
#define KEYSCAN_TIMEOUT_PRESS_VALUE_POS                         (0UL)
#define KEYSCAN_TIMEOUT_PRESS_VALUE_MASK                      (0xFFFFFFFFUL)
#define KEYSCAN_TIMEOUT_PRESS_VALUE(x)                            (((x) & KEYSCAN_TIMEOUT_PRESS_VALUE_MASK) << KEYSCAN_TIMEOUT_PRESS_VALUE_POS)


/**
 **  REGISTER: kp_fifo_rdata, 0ffset:0x14  
 **/
 
/** keypad fifo read data */
#define KEYSCAN_KPAD_RDATA_POS                                        (0UL)
#define KEYSCAN_KPAD_RDATA_MASK                                     (0x1FFUL)
#define KEYSCAN_KPAD_RDATA(x)                                           ((x)  & KEYSCAN_KPAD_RDATA_MASK)


/**row_val */
#define KEYSCAN_KPAD_ROW_POS                                           (0UL)
#define KEYSCAN_KPAD_ROW_MASK                                        (0x7UL)
#define KEYSCAN_KPAD_ROW(x)                                              (((x) >> KEYSCAN_KPAD_ROW_POS) & KEYSCAN_KPAD_ROW_MASK)


/**col_val */
#define KEYSCAN_KPAD_COL_POS                                            (3UL)
#define KEYSCAN_KPAD_COL_MASK                                         (0x7UL)
#define KEYSCAN_KPAD_COL(x)                                               (((x) >> KEYSCAN_KPAD_COL_POS) & KEYSCAN_KPAD_COL_MASK)


/**key_event */
#define KEYSCAN_KEY_EVENT_POS                                            (6UL)
#define KEYSCAN_KEY_EVENT_MASK                                         (0x7UL)
#define KEYSCAN_KEY_EVEN(x)                                                 (((x) >> KEYSCAN_KEY_EVENT_POS) & KEYSCAN_KEY_EVENT_MASK)



/**
 **  REGISTER: kp_fifo_afull_water_mark, 0ffset:0x18  
 **/
 
/** fifo almost full water mark */
#define KEYSCAN_AFULL_WATER_MARK_POS                             (0UL)
#define KEYSCAN_AFULL_WATER_MARK_MASK                          (0x7UL)
#define KEYSCAN_AFULL_WATER_MARK(x)                                (((x) >> KEYSCAN_AFULL_WATER_MARK_POS) & KEYSCAN_AFULL_WATER_MARK_MASK)


/**
 **  REGISTER: kp_int_mask, 0ffset:0x1C  
 **  keypad interrupt mask
 **/

/** key press int enable */
#define KEYSCAN_INT_KEYPRESSEN_POS                                  (0UL)
#define KEYSCAN_INT_KEYPRESSEN_MASK                               ((1UL) << KEYSCAN_INT_KEYPRESSEN_POS)
#define KEYSCAN_INT_KEYPRESSEN                                          (KEYSCAN_INT_KEYPRESSEN_MASK)


/** key release int enable */
#define KEYSCAN_INT_KEYRELEASEEN_POS                               (1UL)
#define KEYSCAN_INT_KEYRELEASEEN_MASK                            ((1UL) << KEYSCAN_INT_KEYRELEASEEN_POS)
#define KEYSCAN_INT_KEYRELEASEEN                                       (KEYSCAN_INT_KEYRELEASEEN_MASK)


/** long press interrupt mask */
#define KEYSCAN_INT_LONG_KEYPRESSEN_POS                        (2UL)
#define KEYSCAN_INT_LONG_KEYPRESSEN_MASK                     ((1UL) << KEYSCAN_INT_LONG_KEYPRESSEN_POS)
#define KEYSCAN_INT_LONG_KEYPRESSEN                                (KEYSCAN_INT_LONG_KEYPRESSEN_MASK)


/** key press timeout interrupt */
#define KEYSCAN_INT_PRESS_TIMEOUTEN_POS                        (3UL)
#define KEYSCAN_INT_PRESS_TIMEOUTEN_MASK                     ((1UL) << KEYSCAN_INT_PRESS_TIMEOUTEN_POS)
#define KEYSCAN_INT_PRESS_TIMEOUTEN                                (KEYSCAN_INT_PRESS_TIMEOUTEN_MASK)


/** key fifo water mark interrupt */
#define KEYSCAN_INT_WATERMARKEN_POS                              (4UL)
#define KEYSCAN_INT_WATERMARKEN_MASK                           ((1UL) << KEYSCAN_INT_WATERMARKEN_POS)
#define KEYSCAN_INT_WATERMARKEN                                      (KEYSCAN_INT_WATERMARKEN_MASK)


/** key fifo empty interrupt */
#define KEYSCAN_INT_FIFO_EMPTY_POS                                 (5UL)
#define KEYSCAN_INT_FIFO_EMPTY_MASK                              ((1UL) << KEYSCAN_INT_FIFO_EMPTY_POS)
#define KEYSCAN_INT_FIFO_EMPTY                                         (KEYSCAN_INT_FIFO_EMPTY_MASK)


/** key fifo full interrupt */
#define KEYSCAN_INT_FIFO_FULL_POS                                   (6UL)
#define KEYSCAN_INT_FIFO_FULL_MASK                                ((1UL) << KEYSCAN_INT_FIFO_FULL_POS)
#define KEYSCAN_INT_FIFO_FULL                                           (KEYSCAN_INT_FIFO_FULL_MASK)


/**
 **  REGISTER: kp_int_src, 0ffset:0x20  
 **  keypad interrupt source
 **/

/** key detect interrupt src */
#define KEYSCAN_INT_FLAG_KEYPRESS_POS                           (0UL)
#define KEYSCAN_INT_FLAG_KEYPRESS_MASK                        ((1UL) << KEYSCAN_INT_FLAG_KEYPRESS_POS)


/** key release interrupt src */
#define KEYSCAN_INT_FLAG_KEYRELEASE_POS                       (1UL)
#define KEYSCAN_INT_FLAG_KEYRELEASE_MASK                    ((1UL) << KEYSCAN_INT_FLAG_KEYRELEASE_POS)


/** key long press interrupt src */
#define KEYSCAN_INT_FLAG_KEYLONGPRESS_POS                  (2UL)
#define KEYSCAN_INT_FLAG_KEYLONGPRESS_MASK               ((1UL) << KEYSCAN_INT_FLAG_KEYLONGPRESS_POS)


/** key  press timeout  interrupt src */
#define KEYSCAN_INT_FLAG_PRESSTIMEOUT_POS                  (3UL)
#define KEYSCAN_INT_FLAG_PRESSTIMEOUT_MASK               ((1UL) << KEYSCAN_INT_FLAG_PRESSTIMEOUT_POS)

/** key  press fifo mark  interrupt src */
#define KEYSCAN_INT_FLAG_FIFOMARK_POS                         (4UL)
#define KEYSCAN_INT_FLAG_FIFOMARK_MASK                       ((1UL) << KEYSCAN_INT_FLAG_FIFOMARK_POS)

/** key  press fifo empty  interrupt src */
#define KEYSCAN_INT_FLAG_FIFOEMPTY_POS                       (5UL)
#define KEYSCAN_INT_FLAG_FIFOEMPTY_MASK                     ((1UL) << KEYSCAN_INT_FLAG_FIFOEMPTY_POS)


/** key  press fifo full  interrupt src */
#define KEYSCAN_INT_FLAG_FIFOFULL_POS                         (6UL)
#define KEYSCAN_INT_FLAG_FIFOFULL_MASK                       ((1UL) << KEYSCAN_INT_FLAG_FIFOFULL_POS)


/**
 **  REGISTER: kp_int_pnd, 0ffset:0x24  
 **  keypad int clear
 **/

/** key detect interrupt clear 1:clear */
#define KEYSCAN_INT_KEYPRESS_PND_POS                           (0UL)
#define KEYSCAN_INT_KEYPRESS_PND_MASK                        ((1UL) << KEYSCAN_INT_KEYPRESS_PND_POS)
#define KEYSCAN_INT_KEYPRESS_PND                                   KEYSCAN_INT_KEYPRESS_PND_MASK

/** key release  interrupt clear 1:clear */
#define KEYSCAN_INT_KEYRELEASE_PND_POS                       (1UL)
#define KEYSCAN_INT_KEYRELEASE_PND_MASK                    ((1UL) << KEYSCAN_INT_KEYRELEASE_PND_POS)
#define KEYSCAN_INT_KEYRELEASE_PND                               KEYSCAN_INT_KEYRELEASE_PND_MASK

/** key long press interrupt clear 1:clear */
#define KEYSCAN_INT_KEYLONGPRESS_PND_POS                   (2UL)
#define KEYSCAN_INT_KEYLONGPRESS_PND_MASK                ((1UL) << KEYSCAN_INT_KEYLONGPRESS_PND_POS)
#define KEYSCAN_INT_KEYLONGPRESS_PND                           KEYSCAN_INT_KEYLONGPRESS_PND_MASK

/** key press timeout  interrupt clear 1:clear */
#define KEYSCAN_INT_KEYPRESSTIMEOUT_PND_POS             (3UL)
#define KEYSCAN_INT_KEYPRESSTIMEOUT_PND_MASK            ((1UL) << KEYSCAN_INT_KEYPRESSTIMEOUT_PND_POS)
#define KEYSCAN_INT_KEYPRESSTIMEOUT_PND                      KEYSCAN_INT_KEYPRESSTIMEOUT_PND_MASK


/** key press timeout  interrupt clear 1:clear */
#define KEYSCAN_INT_FIFOEMPTY_PND_POS                         (5UL)
#define KEYSCAN_INT_FIFOEMPTY_PND_MASK                      ((1UL) << KEYSCAN_INT_FIFOEMPTY_PND_POS)
#define KEYSCAN_INT_FIFOEMPTY_PND                                 KEYSCAN_INT_FIFOEMPTY_PND_MASK


/**
 **  REGISTER: kp_key_sta, 0ffset:0x28  
 **  keypad int clear
 **/
#define ROW_MAX              (5)
#define COL_MAX               (6)
/** key state:[29:0]  */
#define KEYSCAN_STA_KEYPRESS_POS                                  (0UL)
#define KEYSCAN_STA_KEYPRESS_MASK                               ((0x3FFFFFFFUL) << KEYSCAN_STA_KEYPRESS_POS)
#define KEYSCAN_STA_KEYPRESS(sta, row , col)                   (((sta) & KEYSCAN_STA_KEYPRESS_MASK) & ((row) + (col) * COL_MAX))


/**
 **  REGISTER: kp_lp_val, 0ffset:0x2c  
 **  keypad int clear
 **/
/** long press value */
#define KEYSCAN_VAL_LONGPRESS_POS                                (0UL)
#define KEYSCAN_VAL_LONGPRESS_MASK                              ((0xFFFFFFFFUL) << KEYSCAN_VAL_LONGPRESS_POS)
#define KEYSCAN_VAL_LONGPRESS(x)                                    ((x) & KEYSCAN_VAL_LONGPRESS_MASK)


/**
 **  REGISTER: SOFT_RST 0ffset:0x90  
 **  
 **/
 #define KEYSCAN_SOFT_RST_POS                                         (13UL)
 #define KEYSCAN_SOFT_RST_MASK                                      (1<<KEYSCAN_SOFT_RST_POS)


/**
 **  REGISTER: PERI SEL 
 **  sel keypad vertion v1 or v2
 **/
 #define KEYSCAN_PERI_SEL_POS                                          (1UL)
 #define KEYSCAN_PERI_SEL_MASK                                       (1<<KEYSCAN_PERI_SEL_POS)


/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* MS_KEYSCAN_REGS_H_ */
