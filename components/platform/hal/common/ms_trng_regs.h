/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file  ms_trng_regs.h
 * @brief
 * @author haijun.mai
 * @date 2022-01-06
 * @version 1.0
 * @Revision
 */
#ifndef MS_TRNG_REGS_H_
#define MS_TRNG_REGS_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    volatile uint32_t IMR;/*Interrupt Mask Register.*/
    volatile uint32_t ISR;/*Interrupt Status Return Resister*/
    volatile uint32_t ICR;/*Interrupts Clear Register*/
    volatile uint32_t CONFIG;/*handles the TRNG configuration*/
    volatile uint32_t VALID;/*This register indicates that thr TRNG data is valid 0x10*/
    volatile uint32_t DATA0;/*random number */
    volatile uint32_t DATA1;/*random number */
    volatile uint32_t DATA2;/*random number */
    volatile uint32_t DATA3;/*random number 0x20*/
    volatile uint32_t DATA4;/*random number */
    volatile uint32_t DATA5;/*random number */
    volatile uint32_t RSEN;/*enable for the random source*/
    volatile uint32_t SC1;/*SAMPLE_CNT1 0x30*/
    volatile uint32_t AS;/*AUTOCORR_STATISTIC register */
    volatile uint32_t TDC;/*TRNG_DEBUG_CONTROL register */
    volatile uint32_t RESERVED1;
    volatile uint32_t TSR;/*TRNG_SW_RESET 0x40*/
    volatile uint32_t RESERVED2[28];
    volatile uint32_t RDEI;/*RNG_DEBUG_EN_INPUT register 0x1b4*/
    volatile uint32_t RBUSY;/*RNG_BUSY*/
    volatile uint32_t RBC;/*RNG_BITS_COUNTER*/
    volatile uint32_t RV;/*RNG_VERSION register  0x1c0*/
    volatile uint32_t 	RESERVED3[7];
    volatile uint32_t RBCBTR0;/*RNG_BIST_CBTR0 register*/
    volatile uint32_t RBCBTR1;/*RNG_BIST_CBTR1 register*/
    volatile uint32_t RBCBTR2;/*RNG_BIST_CBTR2 register*/
	 	     
} Trng_Type;




	



/*Interrupt Mask Register Bit Define*/
#define TRNG_IMR_EGR_VALID_INT_POS               					                       (0UL)
#define TRNG_IMR_EGR_VALID_INT_MASK      							                (0x1UL << TRNG_IMR_EGR_VALID_INT_POS)
#define TRNG_IMR_EGR_VALID_INT	                                                                        (TRNG_IMR_EGR_VALID_INT_MASK)

#define TRNG_IMR_AUTOCORR_ERR_INT_POS               					                (1UL)
#define TRNG_IMR_AUTOCORR_ERR_INT_MASK      							         (0x1UL << TRNG_IMR_AUTOCORR_ERR_INT_POS)
#define TRNG_IMR_AUTOCORR_ERR_INT	                                                                 (TRNG_IMR_AUTOCORR_ERR_INT_MASK)

#define TRNG_IMR_CRNGT_ERR_INT_POS               					                       (2UL)
#define TRNG_IMR_CRNGT_ERR_INT_MASK      							                (0x1UL << TRNG_IMR_CRNGT_ERR_INT_POS)
#define TRNG_IMR_CRNGT_ERR_INT	                                                                        (TRNG_IMR_CRNGT_ERR_INT_MASK)

#define TRNG_IMR_VN_ERR_INT_POS               					                              (3UL)
#define TRNG_IMR_VN_ERR_INT_MASK      							                       (0x1UL << TRNG_IMR_VN_ERR_INT_POS)
#define TRNG_IMR_VN_ERR_INT	                                                                               (TRNG_IMR_VN_ERR_INT_MASK)


/*Interrupt Status Return Resister Bit Define*/
#define TRNG_ISR_EHR_VALID_POS               					                              (0UL)
#define TRNG_ISR_EHR_VALID_MASK      							                       (0x1UL << TRNG_ISR_EHR_VALID_POS)
#define TRNG_ISR_EHR_VALID	                                                                                      (TRNG_ISR_EHR_VALID_MASK)

#define TRNG_ISR_AUTOCORR_ERR_POS               					                       (1UL)
#define TRNG_ISR_AUTOCORR_ERR_MASK      							                (0x1UL << TRNG_ISR_AUTOCORR_ERR_POS)
#define TRNG_ISR_AUTOCORR_ERR	                                                                               (TRNG_ISR_AUTOCORR_ERR_MASK)

#define TRNG_ISR_CRNGT_ERR_POS               					                              (2UL)
#define TRNG_ISR_CRNGT_ERR_MASK      							                       (0x1UL << TRNG_ISR_CRNGT_ERR_POS)
#define TRNG_ISR_CRNGT_ERR	                                                                                      (TRNG_ISR_CRNGT_ERR_MASK)

#define TRNG_ISR_VN_ERR_POS               					                                     (3UL)
#define TRNG_ISR_VN_ERR_MASK      							                              (0x1UL << TRNG_ISR_VN_ERR_POS)
#define TRNG_ISR_VN_ERR	                                                                                      (TRNG_ISR_VN_ERR_MASK)

/*Interrupts Clear Register Bit Define*/
#define TRNG_ICR_EHR_VALID_POS               					                               (0UL)
#define TRNG_ICR_EHR_VALID_MASK      							                        (0x1UL << TRNG_ICR_EHR_VALID_POS)
#define TRNG_ICR_EHR_VALID	                                                                                       (TRNG_ICR_EHR_VALID_MASK)

#define TRNG_ICR_AUTOCORR_ERR_POS               					                        (1UL)
#define TRNG_ICR_AUTOCORR_ERR_MASK      							                 (0x1UL << TRNG_ICR_AUTOCORR_ERR_POS)
#define TRNG_ICR_AUTOCORR_ERR	                                                                                (TRNG_ICR_AUTOCORR_ERR_MASK)

#define TRNG_ICR_CRNGT_ERR_POS               					                               (2UL)
#define TRNG_ICR_CRNGT_ERR_MASK      							                        (0x1UL << TRNG_ICR_CRNGT_ERR_POS)
#define TRNG_ICR_CRNGT_ERR	                                                                                       (TRNG_ICR_CRNGT_ERR_MASK)

#define TRNG_ICR_VN_ERR_POS               					                                      (3UL)
#define TRNG_ICR_VN_ERR_MASK      							                               (0x1UL << TRNG_ICR_VN_ERR_POS)
#define TRNG_ICR_VN_ERR	                                                                                       (TRNG_ICR_VN_ERR_MASK)

/*handles the TRNG configuration Bit Define*/
#define TRNG_CONFIG_RND_SRC_SEL_POS               					                        (0UL)
#define TRNG_CONFIG_RND_SRC_SEL_MASK      							                 (0x3UL << TRNG_CONFIG_RND_SRC_SEL_POS)
#define TRNG_CONFIG_RND_SRC_SEL                                                                             (TRNG_CONFIG_RND_SRC_SEL_MASK)

/*This register indicates that thr TRNG data is valid Bit Define*/
#define TRNG_VALID_EHR_VALID_POS               					                              (0UL)
#define TRNG_VALID_EHR_VALID_MASK      							                       (0x1UL << TRNG_VALID_EHR_VALID_POS)
#define TRNG_VALID_EHR_VALID                                                                                   (TRNG_VALID_EHR_VALID_MASK)

/*random number Bit Define*/
#define TRNG_DATA0_EHR_DATA_POS               					                              (0UL)
#define TRNG_DATA0_EHR_DATA_MASK      							                       (0xFFFFFFFFUL << TRNG_DATA0_EHR_DATA_POS)
#define TRNG_DATA0_EHR_DATA                                                                                   (TRNG_DATA0_EHR_DATA_MASK)

/*random number Bit Define*/
#define TRNG_DATA1_EHR_DATA_POS               					                              (0UL)
#define TRNG_DATA1_EHR_DATA_MASK      							                       (0xFFFFFFFFUL << TRNG_DATA1_EHR_DATA_POS)
#define TRNG_DATA1_EHR_DATA                                                                                   (TRNG_DATA1_EHR_DATA_MASK)

/*random number Bit Define*/
#define TRNG_DATA2_EHR_DATA_POS               					                              (0UL)
#define TRNG_DATA2_EHR_DATA_MASK      							                       (0xFFFFFFFFUL << TRNG_DATA2_EHR_DATA_POS)
#define TRNG_DATA2_EHR_DATA                                                                                   (TRNG_DATA2_EHR_DATA_MASK)

/*random number Bit Define*/
#define TRNG_DATA3_EHR_DATA_POS               					                              (0UL)
#define TRNG_DATA3_EHR_DATA_MASK      							                       (0xFFFFFFFFUL << TRNG_DATA3_EHR_DATA_POS)
#define TRNG_DATA3_EHR_DATA                                                                                   (TRNG_DATA3_EHR_DATA_MASK)

/*random number Bit Define*/
#define TRNG_DATA4_EHR_DATA_POS               					                              (0UL)
#define TRNG_DATA4_EHR_DATA_MASK      							                       (0xFFFFFFFFUL << TRNG_DATA4_EHR_DATA_POS)
#define TRNG_DATA4_EHR_DATA                                                                                   (TRNG_DATA4_EHR_DATA_MASK)

/*random number Bit Define*/
#define TRNG_DATA5_EHR_DATA_POS               					                              (0UL)
#define TRNG_DATA5_EHR_DATA_MASK      							                       (0xFFFFFFFFUL << TRNG_DATA5_EHR_DATA_POS)
#define TRNG_DATA5_EHR_DATA                                                                                   (TRNG_DATA5_EHR_DATA_MASK)

/*enable for the random source Bit Define*/
#define TRNG_RSEN_RND_SRC_EN_POS               					                              (0UL)
#define TRNG_RSEN_RND_SRC_EN_MASK      							                (0x1UL << TRNG_RSEN_RND_SRC_EN_POS)
#define TRNG_RSEN_RND_SRC_EN                                                                                  (TRNG_RSEN_RND_SRC_EN_MASK)

/*SAMPLE_CNT register Bit Define*/
#define TRNG_SC1_SAMPLE_CNTR1_POS               					                        (0UL)
#define TRNG_SC1_SAMPLE_CNTR1_MASK      							                 (0xFFFFFFFFUL << TRNG_SC1_SAMPLE_CNTR1_POS)
#define TRNG_SC1_SAMPLE_CNTR1                                                                                 (TRNG_SC1_SAMPLE_CNTR1_MASK)

/*AUTOCORR_STATISTIC register  Bit Define*/
#define TRNG_AS_AUTOCORR_TRYS_POS               					                        (0UL)
#define TRNG_AS_AUTOCORR_TRYS_MASK      							                 (0x3FFFUL << TRNG_AS_AUTOCORR_TRYS_POS)
#define TRNG_AS_AUTOCORR_TRYS                                                                                 (TRNG_AS_AUTOCORR_TRYS_MASK)


/*TRNG_DEBUG_CONTROL register  Bit Define*/
#define TRNG_TDC_VNC_BYPASS_POS               					                              (1UL)
#define TRNG_TDC_VNC_BYPASS_MASK      							                       (0x1UL << TRNG_TDC_VNC_BYPASS_POS)
#define TRNG_TDC_VNC_BYPASS                                                                                    (TRNG_TDC_VNC_BYPASS_MASK)

#define TRNG_TDC_TRNG_CRNGT_BYPASS_POS               					                (2UL)
#define TRNG_TDC_TRNG_CRNGT_BYPASS_MASK      							         (0x1UL << TRNG_TDC_TRNG_CRNGT_BYPASS_POS)
#define TRNG_TDC_TRNG_CRNGT_BYPASS                                                                     (TRNG_TDC_TRNG_CRNGT_BYPASS_MASK)

#define TRNG_TDC_AUTO_CORRELATE_BYPASS_POS               					          (3UL)
#define TRNG_TDC_AUTO_CORRELATE_BYPASS_MASK      							   (0x1UL << TRNG_TDC_AUTO_CORRELATE_BYPASS_POS)
#define TRNG_TDC_AUTO_CORRELATE_BYPASS                                                              (TRNG_TDC_AUTO_CORRELATE_BYPASS_MASK)

/*TRNG_SW_RESET register  Bit Define*/
#define TRNG_TSR_RNG_SW_RESET_POS               					                        (0UL)
#define TRNG_TSR_RNG_SW_RESET_MASK      							                 (0x1UL << TRNG_TSR_RNG_SW_RESET_POS)
#define TRNG_TSR_RNG_SW_RESET                                                                                (TRNG_TSR_RNG_SW_RESET_MASK)

/*RNG_DEBUG_EN_INPUT register  Bit Define*/
#define TRNG_RDEI_RNG_DEBUG_EN_POS               					                        (0UL)
#define TRNG_RDEI_RNG_DEBUG_EN_MASK      							                 (0x1UL << TRNG_RDEI_RNG_DEBUG_EN_POS)
#define TRNG_RDEI_RNG_DEBUG_EN                                                                              (TRNG_RDEI_RNG_DEBUG_EN_MASK)

/*RNG_BUSY register  Bit Define*/
#define TRNG_RBUSY_RNG_BUSY_POS               					                               (0UL)
#define TRNG_RBUSY_RNG_BUSY_MASK      							                        (0x1UL << TRNG_RBUSY_RNG_BUSY_POS)
#define TRNG_RBUSY_RNG_BUSY                                                                                     (TRNG_RBUSY_RNG_BUSY_MASK)

/*RNG_BITS_COUNTER register  Bit Define*/
#define TRNG_RBC_RST_BITS_COUNTER_POS               					                  (0UL)
#define TRNG_RBC_RST_BITS_COUNTER_MASK      							           (0x1UL << TRNG_RBC_RST_BITS_COUNTER_POS)
#define TRNG_RBC_RST_BITS_COUNTER                                                                           (TRNG_RBC_RST_BITS_COUNTER_MASK)


/*RNG_VERSION register  Bit Define*/
#define TRNG_RV_EHR_WIDTH_192_POS               					                         (0UL)
#define TRNG_RV_EHR_WIDTH_192_MASK      							                  (0x1UL << TRNG_RV_EHR_WIDTH_192_POS)
#define TRNG_RV_EHR_WIDTH_192                                                                                 (TRNG_RV_EHR_WIDTH_192_MASK)

#define TRNG_RV_CRNGT_EXISTS_POS               					                               (1UL)
#define TRNG_RV_CRNGT_EXISTS_MASK      							                        (0x1UL << TRNG_RV_CRNGT_EXISTS_POS)
#define TRNG_RV_CRNGT_EXISTS                                                                                    (TRNG_RV_CRNGT_EXISTS_MASK)

#define TRNG_RV_AUTOCORR_EXISTS_POS               					                        (2UL)
#define TRNG_RV_AUTOCORR_EXISTS_MASK      							                 (0x1UL << TRNG_RV_AUTOCORR_EXISTS_POS)
#define TRNG_RV_AUTOCORR_EXISTS                                                                             (TRNG_RV_AUTOCORR_EXISTS_MASK)

#define TRNG_RV_TRNG_TESTS_BYPASS_EN_POS               					           (3UL)
#define TRNG_RV_TRNG_TESTS_BYPASS_EN_MASK      							    (0x1UL << TRNG_RV_TRNG_TESTS_BYPASS_EN_POS)
#define TRNG_RV_TRNG_TESTS_BYPASS_EN                                                                    (TRNG_RV_TRNG_TESTS_BYPASS_EN_MASK)

#define TRNG_RV_PRNG_EXISTS_POS               					                                (4UL)
#define TRNG_RV_PRNG_EXISTS_MASK      							                         (0x1UL << TRNG_RV_PRNG_EXISTS_POS)
#define TRNG_RV_PRNG_EXISTS                                                                                       (TRNG_RV_PRNG_EXISTS_MASK)

#define TRNG_RV_KAT_EXISTS_POS               					                                (5UL)
#define TRNG_RV_KAT_EXISTS_MASK      							                         (0x1UL << TRNG_RV_KAT_EXISTS_POS)
#define TRNG_RV_KAT_EXISTS                                                                                         (TRNG_RV_KAT_EXISTS_MASK)

#define TRNG_RV_RESEEDING_EXISTS_POS               					                         (6UL)
#define TRNG_RV_RESEEDING_EXISTS_MASK      							           (0x1UL << TRNG_RV_RESEEDING_EXISTS_POS)
#define TRNG_RV_RESEEDING_EXISTS                                                                             (TRNG_RV_RESEEDING_EXISTS_MASK)

#define TRNG_RV_RNG_USE_5_SBOXES_POS               					                  (7UL)
#define TRNG_RV_RNG_USE_5_SBOXES_MASK      							           (0x1UL << TRNG_RV_RNG_USE_5_SBOXES_POS)
#define TRNG_RV_RNG_USE_5_SBOXES                                                                            (TRNG_RV_RNG_USE_5_SBOXES_MASK)

/*RNG_BIST_CBTR0 register  Bit Define*/
#define TRNG_RBCBTR0_ROSC_CNTR_VAL_POS               					                   (0UL)
#define TRNG_RBCBTR0_ROSC_CNTR_VAL_MASK      							            (0x3FFFFFUL << TRNG_RBCBTR0_ROSC_CNTR_VAL_POS)
#define TRNG_RBCBTR0_ROSC_CNTR_VAL                                                                         (TRNG_RBCBTR0_ROSC_CNTR_VAL_MASK)

/*RNG_BIST_CBTR1 register  Bit Define*/
#define TRNG_RBCBTR1_ROSC_CNTR_VAL_POS               					                   (0UL)
#define TRNG_RBCBTR1_ROSC_CNTR_VAL_MASK      							            (0x3FFFFFUL << TRNG_RBCBTR1_ROSC_CNTR_VAL_POS)
#define TRNG_RBCBTR1_ROSC_CNTR_VAL                                                                         (TRNG_RBCBTR1_ROSC_CNTR_VAL_MASK)

/*RNG_BIST_CBTR2 register  Bit Define*/
#define TRNG_RBCBTR2_ROSC_CNTR_VAL_POS               					                   (0UL)
#define TRNG_RBCBTR2_ROSC_CNTR_VAL_MASK      							            (0x3FFFFFUL << TRNG_RBCBTR2_ROSC_CNTR_VAL_POS)
#define TRNG_RBCBTR2_ROSC_CNTR_VAL                                                                         (TRNG_RBCBTR2_ROSC_CNTR_VAL_MASK)


#ifdef __cplusplus
}
#endif

#endif /* MS_TRNG_H_ */

