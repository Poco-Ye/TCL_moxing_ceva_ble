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
    volatile uint32_t LNP_REP_TMR;  /*!< Keyscan long press timer and repeat press timer Configure Registers  */
    volatile uint32_t STATE;        /*!< Keyscan State Registers  */
    volatile uint32_t FIFO_RDATA;   /*!< Keyscan FIFO RDATA Registers  */
    volatile uint32_t INT_SRC;      /*!< Keyscan Interrupt Source Registers  */
    volatile uint32_t FIFO_CTRL;    /*!< Keyscan FIFO Control Registers  */
    volatile uint32_t ENABLE;       /*!< Keyscan Module Enable and Debounce Configure Registers  */
} Keyscan_Type;

/** @brief Keyscan Control Register Bit Definition*/
/** Reset debounce timer,row scan code and debuouce shift reg*/
#define KEYSCAN_CTRL_FLUSH_POS                              (0UL)
#define KEYSCAN_CTRL_FLUSH_MASK                             ((1UL) << KEYSCAN_CTRL_FLUSH_POS)
#define KEYSCAN_CTRL_FLUSH                                  KEYSCAN_CTRL_FLUSH_MASK

/** push key event to FIFO, 
 0:FIFO just save long press & repeat press & key release event
 1:FIFO also save key press events
 */
#define KEYSCAN_CTRL_PUSHKD_POS                             (1UL)
#define KEYSCAN_CTRL_PUSHKD_MASK                            ((1UL) << KEYSCAN_CTRL_PUSHKD_POS)
#define KEYSCAN_CTRL_PUSHKD                                 KEYSCAN_CTRL_PUSHKD_MASK

/** row scan interval  */
#define KEYSCAN_CTRL_SCAN_FREQ_POS                          (2UL)
#define KEYSCAN_CTRL_SCAN_FREQ_MASK                         ((3UL) << KEYSCAN_CTRL_SCAN_FREQ_POS)
#define KEYSCAN_CTRL_SCAN_FREQ                              KEYSCAN_CTRL_SCAN_FREQ_MASK

/** repeat detect enable */
#define KEYSCAN_CTRL_REPEN_POS                            	(5UL)
#define KEYSCAN_CTRL_REPEN_MASK                           	((1UL) << KEYSCAN_CTRL_REPEN_POS)
#define KEYSCAN_CTRL_REPEN                                	KEYSCAN_CTRL_REPEN_MASK

/** long press enbale */
#define KEYSCAN_CTRL_LNPEN_POS                            	(6UL)
#define KEYSCAN_CTRL_LNPEN_MASK                           	((1UL) << KEYSCAN_CTRL_LNPEN_POS)
#define KEYSCAN_CTRL_LNPEN                                	KEYSCAN_CTRL_LNPEN_MASK

/** multi-key detect enable */
#define KEYSCAN_CTRL_MKEN_POS                            	(7UL)
#define KEYSCAN_CTRL_MKEN_MASK                           	((1UL) << KEYSCAN_CTRL_MKEN_POS)
#define KEYSCAN_CTRL_MKEN                                	KEYSCAN_CTRL_MKEN_MASK

/** debounce enable*/
#define KEYSCAN_CTRL_DBEN_POS                            	(11UL)
#define KEYSCAN_CTRL_DBEN_MASK                           	((1UL) << KEYSCAN_CTRL_DBEN_POS)
#define KEYSCAN_CTRL_DBEN                                	KEYSCAN_CTRL_DBEN_MASK

/** key press intrrupt enable, high active*/
#define KEYSCAN_CTRL_KP_INT_EN_POS                          (16UL)
#define KEYSCAN_CTRL_KP_INT_EN_MASK                         ((1UL) << KEYSCAN_CTRL_KP_INT_EN_POS)
#define KEYSCAN_CTRL_KP_INT_EN                              KEYSCAN_CTRL_KP_INT_EN_MASK

/** key release interrupt enable, high active*/
#define KEYSCAN_CTRL_KR_INT_EN_POS                          (17UL)
#define KEYSCAN_CTRL_KR_INT_EN_MASK                         ((1UL) << KEYSCAN_CTRL_KR_INT_EN_POS)
#define KEYSCAN_CTRL_KR_INT_EN                              KEYSCAN_CTRL_KR_INT_EN_MASK

/** repeat hit interrupt enable, high active*/
#define KEYSCAN_CTRL_RH_INT_EN_POS                          (18UL)
#define KEYSCAN_CTRL_RH_INT_EN_MASK                         ((1UL) << KEYSCAN_CTRL_RH_INT_EN_POS)
#define KEYSCAN_CTRL_RH_INT_EN                              KEYSCAN_CTRL_RH_INT_EN_MASK

/** long hit interrupt enable, high active*/
#define KEYSCAN_CTRL_LH_INT_EN_POS                          (19UL)
#define KEYSCAN_CTRL_LH_INT_EN_MASK                         ((1UL) << KEYSCAN_CTRL_LH_INT_EN_POS)
#define KEYSCAN_CTRL_LH_INT_EN                              KEYSCAN_CTRL_LH_INT_EN_MASK

/** key held time out interrupt enable, high active*/
#define KEYSCAN_CTRL_KHTO_INT_EN_POS                        (20UL)
#define KEYSCAN_CTRL_KHTO_INT_EN_MASK                       ((1UL) << KEYSCAN_CTRL_KHTO_INT_EN_POS)
#define KEYSCAN_CTRL_KHTO_INT_EN                            KEYSCAN_CTRL_KHTO_INT_EN_MASK

/** fifo full intrrupt enable, high active*/
#define KEYSCAN_CTRL_FIFO_FULL_INT_EN_POS                   (21UL)
#define KEYSCAN_CTRL_FIFO_FULL_INT_EN_MASK                  ((1UL) << KEYSCAN_CTRL_FIFO_FULL_INT_EN_POS)
#define KEYSCAN_CTRL_FIFO_FULL_INT_EN                       KEYSCAN_CTRL_FIFO_FULL_INT_EN_MASK

/** column lat when in debounce, high active*/
#define KEYSCAN_CTRL_COL_LAT_EN_POS                         (22UL)
#define KEYSCAN_CTRL_COL_LAT_EN_MASK                        ((1UL) << KEYSCAN_CTRL_COL_LAT_EN_POS)
#define KEYSCAN_CTRL_COL_LAT_EN                             KEYSCAN_CTRL_COL_LAT_EN_MASK

/** clear state hold when debounce disable*/
#define KEYSCAN_CTRL_HOLD_CLR_POS                           (23UL)
#define KEYSCAN_CTRL_HOLD_CLR_MASK                          ((1UL) << KEYSCAN_CTRL_HOLD_CLR_POS)
#define KEYSCAN_CTRL_HOLD_CLR                               KEYSCAN_CTRL_HOLD_CLR_MASK

/** keyheld timer hit*/
#define KEYSCAN_CTRL_KHTIMER_POS                            (24UL)
#define KEYSCAN_CTRL_KHTIMER_MASK                           ((1UL) << KEYSCAN_CTRL_KHTIMER_POS)
#define KEYSCAN_CTRL_KHTIMER                                KEYSCAN_CTRL_KHTIMER_MASK

/** @brief Keyscan long press timer and repeat press timer Configure Register Bit Definition*/
/** long press timer*/
#define KEYSCAN_LNP_REP_TMR_LNPTIMER_POS                    (0UL)
#define KEYSCAN_LNP_REP_TMR_LNPTIMER_MASK                   ((0xFFFFUL) << KEYSCAN_LNP_REP_TMR_LNPTIMER_POS)
#define KEYSCAN_LNP_REP_TMR_LNPTIMER                        KEYSCAN_LNP_REP_TMR_LNPTIMER_MASK
/** repeat press timer*/
#define KEYSCAN_LNP_REP_TMR_REPTIMER_POS                    (16UL)
#define KEYSCAN_LNP_REP_TMR_REPTIMER_MASK                   ((0xFFFFUL) << KEYSCAN_LNP_REP_TMR_REPTIMER_POS)
#define KEYSCAN_LNP_REP_TMR_REPTIMER                        KEYSCAN_LNP_REP_TMR_REPTIMER_MASK

/** @brief Keyscan State Register Bit Definition*/
/** key down event*/
#define KEYSCAN_STATE_KEYDOWM_POS                            (2UL)
#define KEYSCAN_STATE_KEYDOWM_MASK                           ((1UL) << KEYSCAN_STATE_KEYDOWM_POS)
#define KEYSCAN_STATE_KEYDOWM                                KEYSCAN_STATE_KEYDOWM_MASK

/** long press time out*/
#define KEYSCAN_STATE_LNPTO_POS                              (4UL)
#define KEYSCAN_STATE_LNPTO_MASK                             ((1UL) << KEYSCAN_STATE_LNPTO_POS)
#define KEYSCAN_STATE_LNPTO                                  KEYSCAN_STATE_LNPTO_MASK

/** fifo full*/
#define KEYSCAN_STATE_FIFO_FULL_POS                          (5UL)
#define KEYSCAN_STATE_FIFO_FULL_MASK                         ((1UL) << KEYSCAN_STATE_FIFO_FULL_POS)
#define KEYSCAN_STATE_FIFO_FULL                              KEYSCAN_STATE_FIFO_FULL_MASK

/** fifo write overflow*/
#define KEYSCAN_STATE_FIFO_OF_POS                            (6UL)
#define KEYSCAN_STATE_FIFO_OF_MASK                           ((1UL) << KEYSCAN_STATE_FIFO_OF_POS)
#define KEYSCAN_STATE_FIFO_OF                                KEYSCAN_STATE_FIFO_OF_MASK

/** fifo not empty*/
#define KEYSCAN_STATE_FIFO_NE_POS                            (7UL)
#define KEYSCAN_STATE_FIFO_NE_MASK                           ((1UL) << KEYSCAN_STATE_FIFO_NE_POS)
#define KEYSCAN_STATE_FIFO_NE                                KEYSCAN_STATE_FIFO_NE_MASK

/** key column code*/
#define KEYSCAN_STATE_KEY_COL_POS                            (8UL)
#define KEYSCAN_STATE_KEY_COL_MASK                           ((0XFUL) << KEYSCAN_STATE_KEY_COL_POS)
#define KEYSCAN_STATE_KEY_COL                                KEYSCAN_STATE_KEY_COL_MASK

/** key row code*/
#define KEYSCAN_STATE_KEY_ROW_POS                            (12UL)
#define KEYSCAN_STATE_KEY_ROW_MASK                           ((0XFUL) << KEYSCAN_STATE_KEY_ROW_POS)
#define KEYSCAN_STATE_KEY_ROW                                KEYSCAN_STATE_KEY_ROW_MASK

/** key held time out*/
#define KEYSCAN_STATE_KEY_KHTOUT_POS                         (16UL)
#define KEYSCAN_STATE_KEY_KHTOUT_MASK                        ((1UL) << KEYSCAN_STATE_KEY_KHTOUT_POS)
#define KEYSCAN_STATE_KEY_KHTOUT                             KEYSCAN_STATE_KEY_KHTOUT_MASK

/** key column in*/
#define KEYSCAN_STATE_KEY_KPIN_POS                           (17UL)
#define KEYSCAN_STATE_KEY_KPIN_MASK                          ((0x7FFUL) << KEYSCAN_STATE_KEY_KPIN_POS)
#define KEYSCAN_STATE_KEY_KPIN                               KEYSCAN_STATE_KEY_KPIN_MASK

/** @brief Keyscan FIFO RDATA Register Bit Definition*/
#define KEYSCAN_FIFO_RDATA_POS                           	(0UL)
#define KEYSCAN_FIFO_RDATA_MASK                          	((0x3FFUL) << KEYSCAN_FIFO_RDATA_POS)
#define KEYSCAN_FIFO_RDATA                               	KEYSCAN_FIFO_RDATA_MASK

#define KEYSCAN_FIFO_RDATA_COL_POS                           (0UL)
#define KEYSCAN_FIFO_RDATA_COL_MASK                          ((0x3UL) << KEYSCAN_FIFO_RDATA_COL_POS)
#define KEYSCAN_FIFO_RDATA_COL                               KEYSCAN_FIFO_RDATA_COL_MASK

#define KEYSCAN_FIFO_RDATA_ROW_POS                           (4UL)
#define KEYSCAN_FIFO_RDATA_ROW_MASK                          ((0x3UL) << KEYSCAN_FIFO_RDATA_ROW_POS)
#define KEYSCAN_FIFO_RDATA_ROW                               KEYSCAN_FIFO_RDATA_ROW_MASK

#define KEYSCAN_FIFO_RDATA_PRESS_POS                         (8UL)
#define KEYSCAN_FIFO_RDATA_PRESS_MASK                        ((1UL) << KEYSCAN_FIFO_RDATA_PRESS_POS)
#define KEYSCAN_FIFO_RDATA_PRESS                             KEYSCAN_FIFO_RDATA_PRESS_MASK

#define KEYSCAN_FIFO_RDATA_LP_POS                            (9UL)
#define KEYSCAN_FIFO_RDATA_LP_MASK                           ((1UL) << KEYSCAN_FIFO_RDATA_LP_POS)
#define KEYSCAN_FIFO_RDATA_LP                                KEYSCAN_FIFO_RDATA_LP_MASK


/** @brief Keyscan Interrupt Source Register Bit Definition*/
/** key press event, cleared when read*/
#define KEYSCAN_INT_SRC_KEY_PRESS_POS                        (0UL)
#define KEYSCAN_INT_SRC_KEY_PRESS_MASK                       ((1UL) << KEYSCAN_INT_SRC_KEY_PRESS_POS)
#define KEYSCAN_INT_SRC_KEY_PRESS                            KEYSCAN_INT_SRC_KEY_PRESS_MASK

/** key release event, cleared when read*/
#define KEYSCAN_INT_SRC_KEY_RELEASE_POS                      (1UL)
#define KEYSCAN_INT_SRC_KEY_RELEASE_MASK                     ((1UL) << KEYSCAN_INT_SRC_KEY_RELEASE_POS)
#define KEYSCAN_INT_SRC_KEY_RELEASE                          KEYSCAN_INT_SRC_KEY_RELEASE_MASK

/** repeat key event, cleared when read*/
#define KEYSCAN_INT_SRC_REP_HIT_POS                          (2UL)
#define KEYSCAN_INT_SRC_REP_HIT_MASK                         ((1UL) << KEYSCAN_INT_SRC_REP_HIT_POS)
#define KEYSCAN_INT_SRC_REP_HIT                              KEYSCAN_INT_SRC_REP_HIT_MASK

/** long press event, cleared when read*/
#define KEYSCAN_INT_SRC_LNP_HIT_POS                          (3UL)
#define KEYSCAN_INT_SRC_LNP_HIT_MASK                         ((1UL) << KEYSCAN_INT_SRC_LNP_HIT_POS)
#define KEYSCAN_INT_SRC_LNP_HIT                              KEYSCAN_INT_SRC_LNP_HIT_MASK

/** key held time out event, cleared when read*/
#define KEYSCAN_INT_SRC_KHTOUT_POS                           (4UL)
#define KEYSCAN_INT_SRC_KHTOUT_MASK                          ((1UL) << KEYSCAN_INT_SRC_KHTOUT_POS)
#define KEYSCAN_INT_SRC_KHTOUT                               KEYSCAN_INT_SRC_KHTOUT_MASK

/** fifo full, cleared when read*/
#define KEYSCAN_INT_SRC_FIFO_FULL_POS                        (5UL)
#define KEYSCAN_INT_SRC_FIFO_FULL_MASK                       ((1UL) << KEYSCAN_INT_SRC_FIFO_FULL_POS)
#define KEYSCAN_INT_SRC_FIFO_FULL                            KEYSCAN_INT_SRC_FIFO_FULL_MASK

/** @brief Keyscan FIFO Control Register Bit Definition*/
/** key buffer code invalidate, 
 0: Write key code into fifo when key release
 1: Do not write key code into fifo when key release*/
#define KEYSCAN_FIFO_CTRL_KBUFIVD_POS                         (0UL)
#define KEYSCAN_FIFO_CTRL_KBUFIVD_MASK                        ((1UL) << KEYSCAN_FIFO_CTRL_KBUFIVD_POS)
#define KEYSCAN_FIFO_CTRL_KBUFIVD                             KEYSCAN_FIFO_CTRL_KBUFIVD_MASK

/** @brief Keyscan Module Enable and Debounce Configure Registers Bit Definition*/
/** keypad module enable, high active*/
#define KEYSCAN_ENABLE_CTRL_MODULE_EN_POS                     (0UL)
#define KEYSCAN_ENABLE_CTRL_MODULE_EN_MASK                    ((1UL) << KEYSCAN_ENABLE_CTRL_MODULE_EN_POS)
#define KEYSCAN_ENABLE_CTRL_MODULE_EN                         KEYSCAN_ENABLE_CTRL_MODULE_EN_MASK

/** debounce length:dbcnt * Tclk*/
#define KEYSCAN_ENABLE_CTRL_DBCNT_POS                         (1UL)
#define KEYSCAN_ENABLE_CTRL_DBCNT_MASK                        ((0xFFFFUL) << KEYSCAN_ENABLE_CTRL_DBCNT_POS)
#define KEYSCAN_ENABLE_CTRL_DBCNT                             KEYSCAN_ENABLE_CTRL_DBCNT_MASK

/** debonce time out : dbprd * 2 + 1*/
#define KEYSCAN_ENABLE_CTRL_DBPRD_POS                         (17UL)
#define KEYSCAN_ENABLE_CTRL_DBPRD_MASK                        ((0xFFFFUL) << KEYSCAN_ENABLE_CTRL_DBPRD_POS)
#define KEYSCAN_ENABLE_CTRL_DBPRD                             KEYSCAN_ENABLE_CTRL_DBPRD_MASK

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* MS_KEYSCAN_REGS_H_ */
