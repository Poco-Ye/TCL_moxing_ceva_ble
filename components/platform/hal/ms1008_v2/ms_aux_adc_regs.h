
#if 1
/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_aux_adc_regs.h
 * @brief Header file of timer  module.
 * @author haijun.mai
 * @date   2021-01-24
 * @version 1.0
 * @Revision
 */


#ifndef MS_AUX_ADC_REGS_H_
#define MS_AUX_ADC_REGS_H_

#include "ms1008.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    volatile uint32_t CTR;/**/
    volatile uint32_t SMPCNT;/**/
    volatile uint32_t SMPCNTR;/**/
    volatile uint32_t WATERMARK;/**/
    volatile uint32_t CHNQUEUE;/**/
    volatile uint32_t IRQMASK;/**/
    volatile uint32_t IRQSTATUS;/**/
    volatile uint32_t FIFOSTATUS;/**/
    volatile uint32_t FIFODATA;/**/
    volatile uint32_t DLYCFG;/**/
} AuxAdc_Type;

/*Aux Adc  control Register Bit Define*/
#define AUX_ADC_CTR_ADC_SMP_EN_POS               					                       (0UL)
#define AUX_ADC_CTR_ADC_SMP_EN_MASK      							                (0x1UL << AUX_ADC_CTR_ADC_SMP_EN_POS)
#define AUX_ADC_CTR_ADC_SMP_EN	                                                                        (AUX_ADC_CTR_ADC_SMP_EN_MASK)


#define AUX_ADC_CTR_DMA_CTL_POS               					                              (1UL)
#define AUX_ADC_CTR_DMA_CTL_MASK      							                       (0x1UL << AUX_ADC_CTR_DMA_CTL_POS)
#define AUX_ADC_CTR_DMA_CTL	                                                                               (AUX_ADC_CTR_DMA_CTL_MASK)


#define AUX_ADC_CTR_FIFO_FLUSH_POS               					                       (2UL)
#define AUX_ADC_CTR_FIFO_FLUSH_MASK      							                (0x1UL << AUX_ADC_CTR_FIFO_FLUSH_POS)
#define AUX_ADC_CTR_FIFO_FLUSH	                                                                        (AUX_ADC_CTR_FIFO_FLUSH_MASK)

#define AUX_ADC_CTR_SDIF_POS               					                                     (3UL)
#define AUX_ADC_CTR_SDIF_MASK      							                              (0x1UL << AUX_ADC_CTR_SDIF_POS)
#define AUX_ADC_CTR_SDIF	                                                                                      (AUX_ADC_CTR_SDIF_MASK)

#define AUX_ADC_CTR_GCMP_POS               					                                     (4UL)
#define AUX_ADC_CTR_GCMP_MASK      							                              (0x1UL << AUX_ADC_CTR_GCMP_POS)
#define AUX_ADC_CTR_GCMP	                                                                                      (AUX_ADC_CTR_GCMP_MASK)

#define AUX_ADC_CTR_DISH_POS               					                                     (5UL)
#define AUX_ADC_CTR_DISH_MASK      							                              (0x1UL << AUX_ADC_CTR_DISH_POS)
#define AUX_ADC_CTR_DISH	                                                                                      (AUX_ADC_CTR_DISH_MASK)

#define AUX_ADC_CTR_CAL_OFFSET_POS               					                       (6UL)
#define AUX_ADC_CTR_CAL_OFFSET_MASK      							                (0x3FUL << AUX_ADC_CTR_CAL_OFFSET_POS)
#define AUX_ADC_CTR_CAL_OFFSET	                                                                        (AUX_ADC_CTR_CAL_OFFSET_MASK)

/*Aux Adc  Samplel Register Bit Define*/
#define AUX_ADC_SMPCNT_SMP_CNT_POS               					                       (0UL)
#define AUX_ADC_SMPCNT_SMP_CNT_MASK      							                (0xFFFFUL << AUX_ADC_SMPCNT_SMP_CNT_POS)
#define AUX_ADC_SMPCNT_SMP_CNT	                                                                        (AUX_ADC_SMPCNT_SMP_CNT_MASK)

/*Aux Adc  already Sample  Register Bit Define*/
#define AUX_ADC_SMPCNT_SMP_CNT_R_POS               					                (0UL)
#define AUX_ADC_SMPCNT_SMP_CNT_R_MASK      							         (0xFFFFUL << AUX_ADC_SMPCNT_SMP_CNT_R_POS)
#define AUX_ADC_SMPCNT_SMP_CNT_R	                                                                        (AUX_ADC_SMPCNT_SMP_CNT_R_MASK)

/*Aux Adc  Watermask Register Bit Define*/
#define AUX_ADC_WATERMARK_POS               					                              (0UL)
#define AUX_ADC_WATERMARK_MASK      							                       (0x7UL << AUX_ADC_WATERMARK_POS)
#define AUX_ADC_WATERMARK	                                                                               (AUX_ADC_WATERMARK_MASK)

/*Aux Adc  Chnqueue Register Bit Define*/
#define AUX_ADC_CHNQUEUE_CHN_QUEUE_LEN_POS               					  (0UL)
#define AUX_ADC_CHNQUEUE_CHN_QUEUE_LEN_MASK      						  (0x7UL << AUX_ADC_CHNQUEUE_CHN_QUEUE_LEN_POS)
#define AUX_ADC_CHNQUEUE_CHN_QUEUE_LEN	                                                          (AUX_ADC_CHNQUEUE_CHN_QUEUE_LEN_MASK)

#define AUX_ADC_CHNQUEUE_CH0_QUEUE_POS               					                (3UL)
#define AUX_ADC_CHNQUEUE_CH0_QUEUE_MASK      						         (0x7UL << AUX_ADC_CHNQUEUE_CH0_QUEUE_POS)
#define AUX_ADC_CHNQUEUE_CH0_QUEUE	                                                                 (AUX_ADC_CHNQUEUE_CH0_QUEUE_MASK)

#define AUX_ADC_CHNQUEUE_CH1_QUEUE_POS               					                (6UL)
#define AUX_ADC_CHNQUEUE_CH1_QUEUE_MASK      						         (0x7UL << AUX_ADC_CHNQUEUE_CH1_QUEUE_POS)
#define AUX_ADC_CHNQUEUE_CH1_QUEUE	                                                                 (AUX_ADC_CHNQUEUE_CH1_QUEUE_MASK)

#define AUX_ADC_CHNQUEUE_CH2_QUEUE_POS               					                (9UL)
#define AUX_ADC_CHNQUEUE_CH2_QUEUE_MASK      						         (0x7UL << AUX_ADC_CHNQUEUE_CH2_QUEUE_POS)
#define AUX_ADC_CHNQUEUE_CH2_QUEUE	                                                                 (AUX_ADC_CHNQUEUE_CH2_QUEUE_MASK)

#define AUX_ADC_CHNQUEUE_CH3_QUEUE_POS               					                (12UL)
#define AUX_ADC_CHNQUEUE_CH3_QUEUE_MASK      						         (0x7UL << AUX_ADC_CHNQUEUE_CH3_QUEUE_POS)
#define AUX_ADC_CHNQUEUE_CH3_QUEUE	                                                                 (AUX_ADC_CHNQUEUE_CH3_QUEUE_MASK)

#define AUX_ADC_CHNQUEUE_CH4_QUEUE_POS               					                (15UL)
#define AUX_ADC_CHNQUEUE_CH4_QUEUE_MASK      						         (0x7UL << AUX_ADC_CHNQUEUE_CH4_QUEUE_POS)
#define AUX_ADC_CHNQUEUE_CH4_QUEUE	                                                                 (AUX_ADC_CHNQUEUE_CH4_QUEUE_MASK)

#define AUX_ADC_CHNQUEUE_CH5_QUEUE_POS               					                (18UL)
#define AUX_ADC_CHNQUEUE_CH5_QUEUE_MASK      						         (0x7UL << AUX_ADC_CHNQUEUE_CH5_QUEUE_POS)
#define AUX_ADC_CHNQUEUE_CH5_QUEUE	                                                                 (AUX_ADC_CHNQUEUE_CH5_QUEUE_MASK)

#define AUX_ADC_CHNQUEUE_CH6_QUEUE_POS               					                (21UL)
#define AUX_ADC_CHNQUEUE_CH6_QUEUE_MASK      						         (0x7UL << AUX_ADC_CHNQUEUE_CH6_QUEUE_POS)
#define AUX_ADC_CHNQUEUE_CH6_QUEUE	                                                                 (AUX_ADC_CHNQUEUE_CH6_QUEUE_MASK)

#define AUX_ADC_CHNQUEUE_CH7_QUEUE_POS               					                (24UL)
#define AUX_ADC_CHNQUEUE_CH7_QUEUE_MASK      						         (0x7UL << AUX_ADC_CHNQUEUE_CH7_QUEUE_POS)
#define AUX_ADC_CHNQUEUE_CH7_QUEUE	                                                                 (AUX_ADC_CHNQUEUE_CH7_QUEUE_MASK)



/*Aux Adc Irq Mask Register Bit Define*/
#define AUX_ADC_IRQMASK_WATERMASK_POS               					               (0UL)
#define AUX_ADC_IRQMASK_WATERMASK_MASK      							        (0x1UL << AUX_ADC_IRQMASK_WATERMASK_POS)
#define AUX_ADC_IRQMASK_WATERMASK	                                                                (AUX_ADC_IRQMASK_WATERMASK_MASK)

#define AUX_ADC_IRQMASK_ALMOSTFULL_POS               					               (1UL)
#define AUX_ADC_IRQMASK_ALMOSTFULL_MASK      							        (0x1UL << AUX_ADC_IRQMASK_ALMOSTFULL_POS)
#define AUX_ADC_IRQMASK_ALMOSTFULL	                                                                (AUX_ADC_IRQMASK_ALMOSTFULL_MASK)

#define AUX_ADC_IRQMASK_FULL_POS               					                             (2UL)
#define AUX_ADC_IRQMASK_FULL_MASK      							               (0x1UL << AUX_ADC_IRQMASK_FULL_POS)
#define AUX_ADC_IRQMASK_FULL	                                                                              (AUX_ADC_IRQMASK_FULL_MASK)

#define AUX_ADC_IRQMASK_ALMOSTEMPTY_POS               					        (3UL)
#define AUX_ADC_IRQMASK_ALMOSTEMPTY_MASK      							 (0x1UL << AUX_ADC_IRQMASK_ALMOSTEMPTY_POS)
#define AUX_ADC_IRQMASK_ALMOSTEMPTY	                                                                (AUX_ADC_IRQMASK_ALMOSTEMPTY_MASK)

#define AUX_ADC_IRQMASK_EMPTY_POS               					                      (4UL)
#define AUX_ADC_IRQMASK_EMPTY_MASK      							               (0x1UL << AUX_ADC_IRQMASK_EMPTY_POS)
#define AUX_ADC_IRQMASK_EMPTY	                                                                       (AUX_ADC_IRQMASK_EMPTY_MASK)

#define AUX_ADC_IRQMASK_FIFO_ERR_POS               					               (5UL)
#define AUX_ADC_IRQMASK_FIFO_ERR_MASK      							        (0x1UL << AUX_ADC_IRQMASK_FIFO_ERR_POS)
#define AUX_ADC_IRQMASK_FIFO_ERR	                                                                       (AUX_ADC_IRQMASK_FIFO_ERR_MASK)

#define AUX_ADC_IRQMASK_SMP_DONE_POS               					               (6UL)
#define AUX_ADC_IRQMASK_SMP_DONE_MASK      							        (0x1UL << AUX_ADC_IRQMASK_SMP_DONE_POS)
#define AUX_ADC_IRQMASK_SMP_DONE	                                                                       (AUX_ADC_IRQMASK_SMP_DONE_MASK)

#define AUX_ADC_IRQMASK_ALL_POS               					                             (0UL)
#define AUX_ADC_IRQMASK_ALL_MASK      							                      (0x7fUL << AUX_ADC_IRQMASK_ALL_POS)
#define AUX_ADC_IRQMASK_ALL	                                                                              (AUX_ADC_IRQMASK_ALL_MASK)



/*Aux Adc Irq Satus Register Bit Define*/
#define AUX_ADC_IRQSTATUS_WATERMASK_POS               					        (0UL)
#define AUX_ADC_IRQSTATUS_WATERMASK_MASK      							 (0x1UL << AUX_ADC_IRQSTATUS_WATERMASK_POS)
#define AUX_ADC_IRQSTATUS_WATERMASK	                                                                (AUX_ADC_IRQSTATUS_WATERMASK_MASK)

#define AUX_ADC_IRQSTATUS_ALMOSTFULL_POS               					        (1UL)
#define AUX_ADC_IRQSTATUS_ALMOSTFULL_MASK      							 (0x1UL << AUX_ADC_IRQSTATUS_ALMOSTFULL_POS)
#define AUX_ADC_IRQSTATUS_ALMOSTFULL	                                                                (AUX_ADC_IRQSTATUS_ALMOSTFULL_MASK)

#define AUX_ADC_IRQSTATUS_FULL_POS               					                      (2UL)
#define AUX_ADC_IRQSTATUS_FULL_MASK      							               (0x1UL << AUX_ADC_IRQSTATUS_FULL_POS)
#define AUX_ADC_IRQSTATUS_FULL	                                                                       (AUX_ADC_IRQSTATUS_FULL_MASK)

#define AUX_ADC_IRQSTATUS_ALMOSTEMPTY_POS               					        (3UL)
#define AUX_ADC_IRQSTATUS_ALMOSTEMPTY_MASK      							 (0x1UL << AUX_ADC_IRQSTATUS_ALMOSTEMPTY_POS)
#define AUX_ADC_IRQSTATUS_ALMOSTEMPTY	                                                         (AUX_ADC_IRQSTATUS_ALMOSTEMPTY_MASK)

#define AUX_ADC_IRQSTATUS_EMPTY_POS               					                      (4UL)
#define AUX_ADC_IRQSTATUS_EMPTY_MASK      							               (0x1UL << AUX_ADC_IRQSTATUS_EMPTY_POS)
#define AUX_ADC_IRQSTATUS_EMPTY	                                                                       (AUX_ADC_IRQSTATUS_EMPTY_MASK)

#define AUX_ADC_IRQSTATUS_FIFO_ERR_POS               					               (5UL)
#define AUX_ADC_IRQSTATUS_FIFO_ERR_MASK      							        (0x1UL << AUX_ADC_IRQSTATUS_FIFO_ERR_POS)
#define AUX_ADC_IRQSTATUS_FIFO_ERR	                                                                (AUX_ADC_IRQSTATUS_FIFO_ERR_MASK)

#define AUX_ADC_IRQSTATUS_SMP_DONE_POS               					               (6UL)
#define AUX_ADC_IRQSTATUS_SMP_DONE_MASK      							        (0x1UL << AUX_ADC_IRQSTATUS_SMP_DONE_POS)
#define AUX_ADC_IRQSTATUS_SMP_DONE	                                                                (AUX_ADC_IRQSTATUS_SMP_DONE_MASK)

/*Aux Adc  Fifostatus Register Bit Define*/
#define AUX_ADC_FIFOSTATUS_WATERMASK_POS               					        (0UL)
#define AUX_ADC_FIFOSTATUS_WATERMASK_MASK      							 (0x1UL << AUX_ADC_FIFOSTATUS_WATERMASK_POS)
#define AUX_ADC_FIFOSTATUS_WATERMASK	                                                         (AUX_ADC_FIFOSTATUS_WATERMASK_MASK)

#define AUX_ADC_FIFOSTATUS_ALMOSTFULL_POS               					        (1UL)
#define AUX_ADC_FIFOSTATUS_ALMOSTFULL_MASK      							 (0x1UL << AUX_ADC_FIFOSTATUS_ALMOSTFULL_POS)
#define AUX_ADC_FIFOSTATUS_ALMOSTFULL	                                                         (AUX_ADC_FIFOSTATUS_ALMOSTFULL_MASK)

#define AUX_ADC_FIFOSTATUS_FULL_POS               					                      (2UL)
#define AUX_ADC_FIFOSTATUS_FULL_MASK      							               (0x1UL << AUX_ADC_FIFOSTATUS_FULL_POS)
#define AUX_ADC_FIFOSTATUS_FULL	                                                                       (AUX_ADC_FIFOSTATUS_FULL_MASK)

#define AUX_ADC_FIFOSTATUS_ALMOSTEMPTY_POS               					        (3UL)
#define AUX_ADC_FIFOSTATUS_ALMOSTEMPTY_MASK      						 (0x1UL << AUX_ADC_FIFOSTATUS_ALMOSTEMPTY_POS)
#define AUX_ADC_FIFOSTATUS_ALMOSTEMPTY	                                                         (AUX_ADC_FIFOSTATUS_ALMOSTEMPTY_MASK)

#define AUX_ADC_FIFOSTATUS_EMPTY_POS               					               (4UL)
#define AUX_ADC_FIFOSTATUS_EMPTY_MASK      							        (0x1UL << AUX_ADC_FIFOSTATUS_EMPTY_POS)
#define AUX_ADC_FIFOSTATUS_EMPTY	                                                                       (AUX_ADC_FIFOSTATUS_EMPTY_MASK)

#define AUX_ADC_FIFOSTATUS_FIFO_ERR_POS               					               (5UL)
#define AUX_ADC_FIFOSTATUS_FIFO_ERR_MASK      							        (0x1UL << AUX_ADC_FIFOSTATUS_FIFO_ERR_POS)
#define AUX_ADC_FIFOSTATUS_FIFO_ERR	                                                                (AUX_ADC_FIFOSTATUS_FIFO_ERR_MASK)


/*Aux Adc  Fifodata Register Bit Define*/
#define AUX_ADC_FIFODATA_POS               					                                    (0UL)
#define AUX_ADC_FIFODATA_MASK      							                             (0xFFFUL << AUX_ADC_FIFODATA_POS)
#define AUX_ADC_FIFODATA	                                                                                     (AUX_ADC_FIFODATA_MASK)

#define AUX_ADC_FIFODATA_CHN_POS               					                      (12UL)
#define AUX_ADC_FIFODATA_CHN_MASK      							               (0x7UL << AUX_ADC_FIFODATA_CHN_POS)
#define AUX_ADC_FIFODATA_CHN	                                                                              (AUX_ADC_FIFODATA_CHN_MASK)


/*Aux Adc  Fifodata Register Bit Define*/
#define AUX_ADC_DLYCFG_ADCEN_DLY_POS               					               (0UL)
#define AUX_ADC_DLYCFG_ADCEN_DLY_MASK      							        (0x1FUL << AUX_ADC_DLYCFG_ADCEN_DLY_POS)
#define AUX_ADC_DLYCFG_ADCEN_DLY	                                                                       (AUX_ADC_DLYCFG_ADCEN_DLY_MASK)

#define AUX_ADC_DLYCFG_CHN_SW_DLY_POS               					               (5UL)
#define AUX_ADC_DLYCFG_CHN_SW_DLY_MASK      							        (0xFUL << AUX_ADC_DLYCFG_CHN_SW_DLY_POS)
#define AUX_ADC_DLYCFG_CHN_SW_DLY	                                                                (AUX_ADC_DLYCFG_CHN_SW_DLY_MASK)




#ifdef __cplusplus
}
#endif

#endif /* MS_AUX_ADC_H_ */

#endif

