/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_i2s_regs.h
 * @brief Header file of watchdog  module.
 * @author haijun.mai
 * @date   2022-03-07
 * @version 1.0
 * @Revision
 */
#ifndef MS_I2S_REGS_H_
#define MS_I2S_REGS_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	volatile uint32_t IER; /*This register acts as a global enable/disable for DW_apb_i2sr   0x00*/
	volatile uint32_t IRER; /*This register acts as an enable/disable for the DW_apb_i2s Receiver block  0x04*/
	volatile uint32_t ITER; /*This register acts as an enable/disable for the DW_apb_i2s Transmitter block 0x08*/
	volatile uint32_t CER; /*This register acts as an enable/disable for the DW_apb_i2s Clock Generation block  0x0c*/
	volatile uint32_t CCR; /*This register configures the ws_out and sclk_gate signals when DW_apb_i2s is a master  0x10*/
	volatile uint32_t RXFFR; /*This register specifies the Receiver Block FIFO Reset Register  0x14*/
	volatile uint32_t TXFFR;/*This register specifies the Transmitter Block FIFO Reset Register 0x18*/
	volatile uint32_t RSVD0;/*0X1C*/
	union {
	volatile uint32_t LRBR0;/*This specifies the Left Receive Buffer Register 0x20*/
	volatile uint32_t LTHR0;/*This specifies the Left Transmit Holding Register 0x20*/
	} LRTR0;
	
	union {
	volatile uint32_t RRBR0;/*This specifies the Right Receive Buffer Register  0x24*/
	volatile uint32_t RTHR0;/*This specifies the Right Transmit Holding Register  0x24*/
	} RRTR0;
	
	volatile uint32_t RER0;/*This specifies the Receive Enable Register 0x28*/
	volatile uint32_t TER0;/*This specifies the Transmit Enable Register 0x2c*/
	volatile uint32_t RCR0;/*This specifies the Receive Configuration Register 0x30*/
	volatile uint32_t TCR0;/*This specifies the Transmit Configuration Register 0x34*/
	volatile uint32_t ISR0;/* This specifies the Interrupt Status Register 0x38*/
	volatile uint32_t IMR0;/*This specifies the Interrupt Mask Register 0x3c*/
	volatile uint32_t ROR0;/*This specifies the Receive Overrun Register 0x40*/
	volatile uint32_t TOR0;/*This specifies the Transmit Overrun Register 0x44*/
	volatile uint32_t RFCR0;/*This specifies the Receive FIFO Configuration Register 0x48*/
	volatile uint32_t TFCR0;/*This specifies the Transmit FIFO Configuration Register 0x4c*/
	volatile uint32_t RFF0;/*This specifies the Receive FIFO Flush Register 0x50*/
	volatile uint32_t TFF0;/*This specifies the Transmit FIFO Flush Register 0x54*/
} I2s_Type;

typedef struct {
	volatile uint32_t RXDMA;/*Receiver Block DMA Register 0x1c0*/
	volatile uint32_t RRXDMA;/*Reset Receiver Block DMA Register 0x1c4*/
	volatile uint32_t TXDMA;/*Transmitter Block DMA Register 0x1c8*/
	volatile uint32_t RTXDMA;/*Reset Transmitter Block DMA Register 0x1cc*/
	volatile uint32_t RESV[8];
	volatile uint32_t I2S_COMP_PARAM_2; /*0x1f0*/
	volatile uint32_t I2S_COMP_PARAM_1;/*0x1f4*/
	volatile uint32_t I2S_COMP_VERSION;/*0x1f8*/
	volatile uint32_t I2S_COMP_TYPE;/*0x1fc*/
	volatile uint32_t DMACR;/*DMA Control Register 0x200*/
	//volatile uint32_t RXDMA_CH[4];/*0x204 - 0x210*/
	//volatile uint32_t TXDMA_CH[4];/*0x214-0x220*/	
} I2sDma_Type;


/*i2s Enable Register Bit Define*/
#define I2S_IER_IEN_POS               		               			(0UL)
#define I2S_IER_IEN_MASK      								(0x1UL << I2S_IER_IEN_POS)
#define I2S_IER_IEN										       (I2S_IER_IEN_MASK)


/*I2S Receiver Block Enable Register Bit Define*/
#define I2S_IRER_RXEN_POS               		               			(0UL)
#define I2S_IRER_RXEN_MASK      								(0x1UL << I2S_IRER_RXEN_POS)
#define I2S_IRER_RXEN										(I2S_IRER_RXEN_MASK)

/*I2S Transmitter Block Enable Register Bit Define*/
#define I2S_ITER_TXEN_POS               		               			(0UL)
#define I2S_ITER_TXEN_MASK      								(0x1UL << I2S_ITER_TXEN_POS)
#define I2S_ITER_TXEN										(I2S_ITER_TXEN_MASK)


/*Clock Enable  Register Bit Define*/
#define I2S_CER_CLKEN_POS               		               			(0UL)
#define I2S_CER_CLKEN_MASK      								(0x1UL << I2S_CER_CLKEN_POS)
#define I2S_CER_CLKEN										(I2S_CER_CLKEN_MASK)


/*Clock Configuration Register Bit Define*/
#define I2S_CCR_SCLKG_POS               		               			(0UL)
#define I2S_CCR_SCLKG_MASK      								(0x7UL << I2S_CCR_SCLKG_POS)
#define I2S_CCR_SCLKG										(I2S_CCR_SCLKG_MASK)

#define I2S_CCR_WSS_POS               		               			(3UL)
#define I2S_CCR_WSS_MASK      								(0x3UL << I2S_CCR_WSS_POS)
#define I2S_CCR_WSS										       (I2S_CCR_WSS_MASK)


/*Receiver Block FIFO Reset Register Bit Define*/
#define I2S_RXFFR_RXFFR_POS               		               		(0UL)
#define I2S_RXFFR_RXFFR_MASK      							(0x1UL << I2S_RXFFR_RXFFR_POS)
#define I2S_RXFFR_RXFFR										(I2S_RXFFR_RXFFR_MASK)

/*Receiver Block FIFO Reset Register Bit Define*/
#define I2S_TXFFR_TXFFR_POS               		               		(0UL)
#define I2S_TXFFR_TXFFR_MASK      							(0x1UL << I2S_TXFFR_TXFFR_POS)
#define I2S_TXFFR_TXFFR										(I2S_TXFFR_TXFFR_MASK)


/*Left Receive Buffer Register x Bit Define*/
#define I2S_LRBRX_LRBRX_POS               		               	       (0UL)
#define I2S_LRBRX_LRBRX_MASK      							(0xFFFFFFFFUL << I2S_LRBRX_LRBRX_POS)
#define I2S_LRBRX_LRBRX										(I2S_LRBRX_LRBRX_MASK)

/*Left Transmit Holding Register x Bit Define*/
#define I2S_LTHRX_LTHRX_POS               		               		(0UL)
#define I2S_LTHRX_LTHRX_MASK      							(0xFFFFFFFFUL << I2S_LTHRX_LTHRX_POS)
#define I2S_LTHRX_LTHRX										(I2S_LTHRX_LTHRX_MASK)

/*Right Transmit Holding Register x Bit Define*/
#define I2S_RRBRX_RRBRX_POS               		               		(0UL)
#define I2S_RRBRX_RRBRX_MASK      							(0xFFFFFFFFUL << I2S_RRBRX_RRBRX_POS)
#define I2S_RRBRX_RRBRX										(I2S_RRBRX_RRBRX_MASK)

/*This specifies the Right Transmit Holding  Register x Bit Define*/
#define I2S_RTHRX_RTHRX_POS               		               		(0UL)
#define I2S_RTHRX_RTHRX_MASK      							(0xFFFFFFFFUL << I2S_RTHRX_RTHRX_POS)
#define I2S_RTHRX_RTHRX									(I2S_RTHRX_RTHRX_MASK)

/*Receive Enable Register x Bit Define*/
#define I2S_RERX_RXCHENX_POS               		               		(0UL)
#define I2S_RERX_RXCHENX_MASK      							(0x1UL << I2S_RERX_RXCHENX_POS)
#define I2S_RERX_RXCHENX									(I2S_RERX_RXCHENX_MASK)

/*Transmit Enable Register x Bit Define*/
#define I2S_TERX_TXCHENX_POS               		               		(0UL)
#define I2S_TERX_TXCHENX_MASK      							(0x1UL << I2S_TERX_TXCHENX_POS)
#define I2S_TERX_TXCHENX									(I2S_TERX_TXCHENX_MASK)

/*Transmit Enable Register x Bit Define*/
#define I2S_RCRX_WLEN_POS               		               		       (0UL)
#define I2S_RCRX_WLEN_MASK      							       (0x7UL << I2S_RCRX_WLEN_POS)
#define I2S_RCRX_WLEN									       (I2S_RCRX_WLEN_MASK)

/*Transmit Configuration Register x Bit Define*/
#define I2S_TCRX_WLEN_POS               		               		       (0UL)
#define I2S_TCRX_WLEN_MASK      							       (0x7UL << I2S_TCRX_WLEN_POS)
#define I2S_TCRX_WLEN									       (I2S_TCRX_WLEN_MASK)

/*Interrupt status Register x Bit Define*/
#define I2S_ISRX_RXDA_POS               		               		       (0UL)
#define I2S_ISRX_RXDA_MASK      							       (0x1UL << I2S_ISRX_RXDA_POS)
#define I2S_ISRX_RXDA									       (I2S_ISRX_RXDA_MASK)

#define I2S_ISRX_RXFO_POS               		               		       (1UL)
#define I2S_ISRX_RXFO_MASK      							       (0x1UL << I2S_ISRX_RXFO_POS)
#define I2S_ISRX_RXFO									       (I2S_ISRX_RXFO_MASK)

#define I2S_ISRX_TXFE_POS               		               		       (4UL)
#define I2S_ISRX_TXFE_MASK      							       (0x1UL << I2S_ISRX_TXFE_POS)
#define I2S_ISRX_TXFE									       (I2S_ISRX_TXFE_MASK)

#define I2S_ISRX_TXFO_POS               		               		       (5UL)
#define I2S_ISRX_TXFO_MASK      							       (0x1UL << I2S_ISRX_TXFO_POS)
#define I2S_ISRX_TXFO									       (I2S_ISRX_TXFO_MASK)


/*Interrupt Mask Register x Bit Define*/
#define I2S_IMRX_RXDAM_POS               		               		(0UL)
#define I2S_IMRX_RXDAM_MASK      							(0x1UL << I2S_IMRX_RXDAM_POS)
#define I2S_IMRX_RXDAM									       (I2S_IMRX_RXDAM_MASK)

#define I2S_IMRX_RXFOM_POS               		               		(1UL)
#define I2S_IMRX_RXFOM_MASK      							(0x1UL << I2S_IMRX_RXFOM_POS)
#define I2S_IMRX_RXFOM									       (I2S_IMRX_RXFOM_MASK)

#define I2S_IMRX_TXFEM_POS               		               		(4UL)
#define I2S_IMRX_TXFEM_MASK      							(0x1UL << I2S_IMRX_TXFEM_POS)
#define I2S_IMRX_TXFEM									       (I2S_IMRX_TXFEM_MASK)

#define I2S_IMRX_TXFOM_POS               		               		(5UL)
#define I2S_IMRX_TXFOM_MASK      							(0x1UL << I2S_IMRX_TXFOM_POS)
#define I2S_IMRX_TXFOM									       (I2S_IMRX_TXFOM_MASK)


/*Receive Overrun Register x Bit Define*/
#define I2S_RORX_RXCHO_POS               		               		(0UL)
#define I2S_RORX_RXCHO_MASK      							(0x1UL << I2S_RORX_RXCHO_POS)
#define I2S_RORX_RXCHO									       (I2S_RORX_RXCHO_MASK)

/*Transmit Overrun Register x Bit Define*/
#define I2S_TORX_TXCHO_POS               		               		(0UL)
#define I2S_TORX_TXCHO_MASK      							(0x1UL << I2S_TORX_TXCHO_POS)
#define I2S_TORX_TXCHO									       (I2S_TORX_TXCHO_MASK)

/*Receive FIFO Configuration Register x Bit Define*/
#define I2S_RFCRX_RXCHDT_POS               		               		(0UL)
#define I2S_RFCRX_RXCHDT_MASK      							(0xFUL << I2S_RFCRX_RXCHDT_POS)
#define I2S_RFCRX_RXCHDT									(I2S_RFCRX_RXCHDT_MASK)

/*Transmit FIFO Configuration Register x Bit Define*/
#define I2S_TFCRX_TXCHET_POS               		               		(0UL)
#define I2S_TFCRX_TXCHET_MASK      							(0xFUL << I2S_TFCRX_TXCHET_POS)
#define I2S_TFCRX_TXCHET									(I2S_TFCRX_TXCHET_MASK)

/*Receive FIFO Flush Register x Bit Define*/
#define I2S_RFFX_RXCHFR_POS               		               		(0UL)
#define I2S_RFFX_RXCHFR_MASK      							(0x1UL << I2S_RFFX_RXCHFR_POS)
#define I2S_RFFX_RXCHFR									       (I2S_RFFX_RXCHFR_MASK)

/*Transmit FIFO Flush Register x Bit Define*/
#define I2S_TFFX_TXCHFR_POS               		               		(0UL)
#define I2S_TFFX_TXCHFR_MASK      							(0x1UL << I2S_TFFX_TXCHFR_POS)
#define I2S_TFFX_TXCHFR									       (I2S_TFFX_TXCHFR_MASK)


/*Receiver Block DMA Register Bit Define*/
#define I2S_RXDMA_RXDMA_POS               		               		(0UL)
#define I2S_RXDMA_RXDMA_MASK      							(0xFFFFFFFFUL << I2S_RXDMA_RXDMA_POS)
#define I2S_RXDMA_RXDMA									(I2S_RXDMA_RXDMA_MASK)

/*Reset Receiver Block DMA Register Bit Define*/
#define I2S_RRXDMA_RRXDMA_POS               		               	(0UL)
#define I2S_RRXDMA_RRXDMA_MASK      						(0x1UL << I2S_RRXDMA_RRXDMA_POS)
#define I2S_RRXDMA_RRXDMA									(I2S_RRXDMA_RRXDMA_MASK)


/*Transmitter Block DMA Register Bit Define*/
#define I2S_TXDMA_TXDMA_POS               		               	      (0UL)
#define I2S_TXDMA_TXDMA_MASK      						      (0xFFFFFFFFUL << I2S_TXDMA_TXDMA_POS)
#define I2S_TXDMA_TXDMA								      (I2S_TXDMA_TXDMA_MASK)

/*Reset Transmitter Block DMA Register bit Define*/
#define I2S_RTXDMA_RTXDMA_POS               		               	(0UL)
#define I2S_RTXDMA_RTXDMA_MASK      						(0x1UL << I2S_RTXDMA_RTXDMA_POS)
#define I2S_RTXDMA_RTXDMA									(I2S_RTXDMA_RTXDMA_MASK)



/*DMA Control  Register bit Define*/
#define I2S_DMACR_DMAEN_RXCH_0_POS               		              (0UL)
#define I2S_DMACR_DMAEN_RXCH_0_MASK      					(0x1UL << I2S_DMACR_DMAEN_RXCH_0_POS)
#define I2S_DMACR_DMAEN_RXCH_0							(I2S_DMACR_DMAEN_RXCH_0_POS)

#define I2S_DMACR_DMAEN_RXCH_1_POS               		              (1UL)
#define I2S_DMACR_DMAEN_RXCH_1_MASK      					(0x1UL << I2S_DMACR_DMAEN_RXCH_1_POS)
#define I2S_DMACR_DMAEN_RXCH_1							(I2S_DMACR_DMAEN_RXCH_1_POS)

#define I2S_DMACR_DMAEN_RXCH_2_POS               		              (2UL)
#define I2S_DMACR_DMAEN_RXCH_2_MASK      					(0x1UL << I2S_DMACR_DMAEN_RXCH_2_POS)
#define I2S_DMACR_DMAEN_RXCH_2							(I2S_DMACR_DMAEN_RXCH_2_POS)

#define I2S_DMACR_DMAEN_RXCH_3_POS               		              (3UL)
#define I2S_DMACR_DMAEN_RXCH_3_MASK      					(0x1UL << I2S_DMACR_DMAEN_RXCH_3_POS)
#define I2S_DMACR_DMAEN_RXCH_3							(I2S_DMACR_DMAEN_RXCH_3_POS)


#define I2S_DMACR_DMAEN_TXCH_0_POS               		              (8UL)
#define I2S_DMACR_DMAEN_TXCH_0_MASK       					(0x1UL << I2S_DMACR_DMAEN_TXCH_0_POS)
#define I2S_DMACR_DMAEN_TXCH_0							(I2S_DMACR_DMAEN_TXCH_0_MASK )

#define I2S_DMACR_DMAEN_TXCH_1_POS               		              (9UL)
#define I2S_DMACR_DMAEN_TXCH_1_MASK       					(0x1UL << I2S_DMACR_DMAEN_TXCH_1_POS)
#define I2S_DMACR_DMAEN_TXCH_1							(I2S_DMACR_DMAEN_TXCH_1_MASK )

#define I2S_DMACR_DMAEN_TXCH_2_POS               		              (10UL)
#define I2S_DMACR_DMAEN_TXCH_2_MASK       					(0x1UL << I2S_DMACR_DMAEN_TXCH_2_POS)
#define I2S_DMACR_DMAEN_TXCH_2							(I2S_DMACR_DMAEN_TXCH_2_MASK )

#define I2S_DMACR_DMAEN_TXCH_3_POS               		              (11UL)
#define I2S_DMACR_DMAEN_TXCH_3_MASK       					(0x1UL << I2S_DMACR_DMAEN_TXCH_3_POS)
#define I2S_DMACR_DMAEN_TXCH_3							(I2S_DMACR_DMAEN_TXCH_3_MASK )


#define I2S_DMACR_DMAEN_RXBLOCK_POS               		              (16UL)
#define I2S_DMACR_DMAEN_RXBLOCK_MASK       					(0x1UL << I2S_DMACR_DMAEN_RXBLOCK_POS)
#define I2S_DMACR_DMAEN_RXBLOCK							(I2S_DMACR_DMAEN_RXBLOCK_MASK )


#define I2S_DMACR_DMAEN_TXBLOCK_POS               		              (17UL)
#define I2S_DMACR_DMAEN_TXBLOCK_MASK       					(0x1UL << I2S_DMACR_DMAEN_TXBLOCK_POS)
#define I2S_DMACR_DMAEN_TXBLOCK							(I2S_DMACR_DMAEN_TXBLOCK_MASK )



#ifdef __cplusplus
}
#endif

#endif
