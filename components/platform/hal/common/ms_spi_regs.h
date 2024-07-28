/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_spi_regs.h
 * @brief Header file of timer  module.
 * @author haijun.mai
 * @date   2021-12-30
 * @version 1.0
 * @Revision
 */

#ifndef MS_SPI_REGS_H_
#define MS_SPI_REGS_H_

#include "ms1008.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    volatile uint32_t CTRLR0;/*Control Register 0*/
    volatile uint32_t CTRLR1;/*Control Register 1*/
    volatile uint32_t SSIENR;/*SSI Enable Register*/
    volatile uint32_t MWCR;/*Microwire Control Register*/
    volatile uint32_t SER;/* Slave Enable Register*/
    volatile uint32_t BAUDR;/*Baud Rate Select*/
    volatile uint32_t TXFTLR;/*Transmit FIFO Threshold Level*/
    volatile uint32_t RXFTLR;/*Receive FIFO Threshold Level*/
    volatile uint32_t TXFLR;/*Transmit FIFO Level Register*/
    volatile uint32_t RXFLR;/*Receive FIFO Level Register*/
    volatile uint32_t SR;/*Status Register*/
    volatile uint32_t IMR;/*Interrupt Mask Register*/
    volatile uint32_t ISR;/*Interrupt Status Register*/
    volatile uint32_t RISR;/*Raw Interrupt Status Register*/
    volatile uint32_t TXOICR;/*Transmit FIFO Overflow Interrupt Clear Registers*/
    volatile uint32_t RXOICR;/*Receive FIFO Overflow Interrupt Clear Register*/
    volatile uint32_t RXUICR;/*Receive FIFO Underflow Interrupt Clear Register*/
    volatile uint32_t MSTICR;/*Multi-Master Interrupt Clear Register*/
    volatile uint32_t ICR;/*Interrupt Clear Register*/
    volatile uint32_t DMACR;/*DMA Control Register*/
    volatile uint32_t DMATDLR;/*DMATDLR*/
    volatile uint32_t DMARDLR;/*DMARDLR*/
    volatile uint32_t IDR;/*Identification Register*/
    volatile uint32_t SSIVERSIONID;/* coreKit version ID Register*/
    volatile uint32_t DRX[35];/*Data Register X*/
    volatile uint32_t RXSAMPLEDLY;/*RX Sample Delay Register*/
    volatile uint32_t SPICTRLR0;/*SPI Control Register*/
    volatile uint32_t TXDDRIVEEDGE;/*Transmit Drive Edge Register*/
    volatile uint32_t RSVD;/*RSVD - Reserved address location*/
} Spi_Type;

/*Control Register 0 Bit Define*/
#define SPI_CTRLR0_DFS_POS               					                       (0UL)
#define SPI_CTRLR0_DFS_MASK      							         (0xFUL << SPI_CTRLR0_DFS_POS)
#define SPI_CTRLR0_DFS                                                                           (SPI_CTRLR0_DFS_MASK)

#define SPI_CTRLR0_FRF_POS               					                       (4UL)
#define SPI_CTRLR0_FRF_MASK      							         (0x3UL << SPI_CTRLR0_FRF_POS)
#define SPI_CTRLR0_FRF                                                                           (SPI_CTRLR0_FRF_MASK)

#define SPI_CTRLR0_SCPH_POS               					                 (6UL)
#define SPI_CTRLR0_SCPH_MASK      							          (0x1UL << SPI_CTRLR0_SCPH_POS)
#define SPI_CTRLR0_SCPH                                                                          (SPI_CTRLR0_SCPH_MASK)

#define SPI_CTRLR0_SCPOL_POS               					                 (7UL)
#define SPI_CTRLR0_SCPOL_MASK      							          (0x1UL << SPI_CTRLR0_SCPOL_POS)
#define SPI_CTRLR0_SCPOL                                                                        (SPI_CTRLR0_SCPOL_MASK)

#define SPI_CTRLR0_TMOD_POS               					                 (8UL)
#define SPI_CTRLR0_TMOD_MASK      							          (0x3UL << SPI_CTRLR0_TMOD_POS)
#define SPI_CTRLR0_TMOD                                                                           (SPI_CTRLR0_TMOD_MASK)

#define SPI_CTRLR0_SLVOE_POS               					                 (10UL)
#define SPI_CTRLR0_SLVOE_MASK      							          (0x1UL << SPI_CTRLR0_SLVOE_POS)
#define SPI_CTRLR0_SLVOE                                                                          (SPI_CTRLR0_SLVOE_MASK)

#define SPI_CTRLR0_SRL_POS               					                        (11UL)
#define SPI_CTRLR0_SRL_MASK      							                 (0x1UL << SPI_CTRLR0_SRL_POS)
#define SPI_CTRLR0_SRL                                                                              (SPI_CTRLR0_SRL_MASK)

#define SPI_CTRLR0_CFS_POS               					                        (12UL)
#define SPI_CTRLR0_CFS_MASK      							                 (0xFUL << SPI_CTRLR0_CFS_POS)
#define SPI_CTRLR0_CFS                                                                              (SPI_CTRLR0_CFS_MASK)

#define SPI_CTRLR0_DFS32_POS               					                 (16UL)
#define SPI_CTRLR0_DFS32_MASK      							          (0x1FUL << SPI_CTRLR0_DFS32_POS)
#define SPI_CTRLR0_DFS32                                                                         (SPI_CTRLR0_DFS32_MASK)

#define SPI_CTRLR0_SPIFRF_POS               					                 (21UL)
#define SPI_CTRLR0_SPIFRF_MASK      							          (0x3UL << SPI_CTRLR0_SPIFRF_POS)
#define SPI_CTRLR0_SPIFRF                                                                         (SPI_CTRLR0_SPIFRF_MASK)

#define SPI_CTRLR0_SSTE_POS               					                        (24UL)
#define SPI_CTRLR0_SSTE_MASK      							          (0x1UL << SPI_CTRLR0_SSTE_POS)
#define SPI_CTRLR0_SSTE                                                                            (SPI_CTRLR0_SSTE_MASK)

/*Control Register 1 Bit Define*/
#define SPI_CTRLR1_NDF_POS               					                 (0UL)
#define SPI_CTRLR1_NDF_MASK      							          (0xFFFFUL << SPI_CTRLR1_NDF_POS)
#define SPI_CTRLR1_NDF                                                                           (SPI_CTRLR1_NDF_MASK)

/*SSI Enable Register Bit Define*/
#define SPI_SSIENR_SSIEN_POS               					                 (0UL)
#define SPI_SSIENR_SSIEN_MASK      							          (0x1UL << SPI_SSIENR_SSIEN_POS)
#define SPI_SSIENR_SSIEN                                                                        (SPI_SSIENR_SSIEN_MASK)

/* Microwire Control Register Bit Define*/
#define SPI_MWCR_MWMOD_POS               					                 (0UL)
#define SPI_MWCR_MWMOD_MASK      							          (0x1UL << SPI_MWCR_MWMOD_POS)
#define SPI_MWCR_MWMOD                                                                        (SPI_MWCR_MWMOD_MASK)

#define SPI_MWCR_MDD_POS               					                        (1UL)
#define SPI_MWCR_MDD_MASK      							                 (0x1UL << SPI_MWCR_MDD_POS)
#define SPI_MWCR_MDD                                                                             (SPI_MWCR_MDD_MASK)

#define SPI_MWCR_MHS_POS               					                        (2UL)
#define SPI_MWCR_MHS_MASK      							                 (0x1UL << SPI_MWCR_MHS_POS)
#define SPI_MWCR_MHS                                                                             (SPI_MWCR_MHS_MASK)

/*Slave Enable Register Bit Define*/
#define SPI_SER_SER_POS               					                        (0UL)
#define SPI_SER_SER_MASK      							                 (0x1UL << SPI_SER_SER_POS)
#define SPI_SER_SER                                                                                  (SPI_SER_SER_MASK)

/*BAUDR Register Bit Define*/
#define SPI_BAUDR_SCKDV_POS               					                 (0UL)
#define SPI_BAUDR_SCKDV_MASK      							          (0xFFFFUL << SPI_BAUDR_SCKDV_POS)
#define SPI_BAUDR_SCKDV                                                                         (SPI_BAUDR_SCKDV_MASK)

/*Transmit FIFO Threshold Level Bit Define*/
#define SPI_TXFTLR_TFT_POS               					                       (0UL)
#define SPI_TXFTLR_TFT_MASK      							         (0x1FUL << SPI_TXFTLR_TFT_POS)
#define SPI_TXFTLR_TFT                                                                           (SPI_BAUDR_SCKDV_MASK)

/*Transmit FIFO Threshold Level Bit Define*/
#define SPI_RXFTLR_RFT_POS               					                       (0UL)
#define SPI_RXFTLR_RFT_MASK      							         (0x1FUL << SPI_RXFTLR_RFT_POS)
#define SPI_RXFTLR_RFT                                                                           (SPI_RXFTLR_RFT_MASK)

/*Transmit FIFO Level Register Bit Define*/
#define SPI_TXFLR_TXTFL_POS               					                (0UL)
#define SPI_TXFLR_TXTFL_MASK      							         (0x1FUL << SPI_TXFLR_TXTFL_POS)
#define SPI_TXFLR_TXTFL                                                                         (SPI_TXFLR_TXTFL_MASK)

/*Receive FIFO Level Register Register Bit Define*/
#define SPI_RXFLR_RXTFL_POS               					                (0UL)
#define SPI_RXFLR_RXTFL_MASK      							         (0x1FUL << SPI_RXFLR_RXTFL_POS)
#define SPI_RXFLR_RXTFL                                                                         (SPI_RXFLR_RXTFL_MASK)

/*Status  Register Register Bit Define*/
#define SPI_SR_BUSY_POS               					                       (0UL)
#define SPI_SR_BUSY_MASK      							                (0x1UL << SPI_SR_BUSY_POS)
#define SPI_SR_BUSY                                                                                (SPI_SR_BUSY_MASK)

#define SPI_SR_TFNF_POS               					                       (1UL)
#define SPI_SR_TFNF_MASK      							                (0x1UL << SPI_SR_TFNF_POS)
#define SPI_SR_TFNF                                                                                (SPI_SR_TFNF_MASK)

#define SPI_SR_TFE_POS               					                             (2UL)
#define SPI_SR_TFE_MASK      							                      (0x1UL << SPI_SR_TFE_POS)
#define SPI_SR_TFE                                                                                  (SPI_SR_TFE_MASK)

#define SPI_SR_RFNE_POS               					                       (3UL)
#define SPI_SR_RFNE_MASK      							                (0x1UL << SPI_SR_RFNE_POS)
#define SPI_SR_RFNE                                                                                (SPI_SR_RFNE_MASK)

#define SPI_SR_RFF_POS               					                             (4UL)
#define SPI_SR_RFF_MASK      							                      (0x1UL << SPI_SR_RFF_POS)
#define SPI_SR_RFF                                                                                 (SPI_SR_RFF_MASK)

#define SPI_SR_TXE_POS               					                             (5UL)
#define SPI_SR_TXE_MASK      							                      (0x1UL << SPI_SR_TXE_POS)
#define SPI_SR_TXE                                                                                 (SPI_SR_TXE_MASK)

#define SPI_SR_DCOL_POS               					                      (6UL)
#define SPI_SR_DCOL_MASK      							               (0x1UL << SPI_SR_DCOL_POS)
#define SPI_SR_DCOL                                                                                (SPI_SR_DCOL_MASK)

/*Interrupt Mask Register Bit Define*/
#define SPI_IMR_TXEIM_POS               					                       (0UL)
#define SPI_IMR_TXEIM_MASK      							                (0x1UL << SPI_IMR_TXEIM_POS)
#define SPI_IMR_TXEIM                                                                             (SPI_IMR_TXEIM_MASK)

#define SPI_IMR_TXOIM_POS               					                       (1UL)
#define SPI_IMR_TXOIM_MASK      							                (0x1UL << SPI_IMR_TXOIM_POS)
#define SPI_IMR_TXOIM                                                                            (SPI_IMR_TXOIM_MASK)

#define SPI_IMR_RXUIM_POS               					                       (2UL)
#define SPI_IMR_RXUIM_MASK      							                (0x1UL << SPI_IMR_RXUIM_POS)
#define SPI_IMR_RXUIM                                                                            (SPI_IMR_RXUIM_MASK)

#define SPI_IMR_RXOIM_POS               					                       (3UL)
#define SPI_IMR_RXOIM_MASK      							                (0x1UL << SPI_IMR_RXOIM_POS)
#define SPI_IMR_RXOIM                                                                            (SPI_IMR_RXOIM_MASK)

#define SPI_IMR_RXFIM_POS               					                       (4UL)
#define SPI_IMR_RXFIM_MASK      							                (0x1UL << SPI_IMR_RXFIM_POS)
#define SPI_IMR_RXFIM                                                                            (SPI_IMR_RXFIM_MASK)

#define SPI_IMR_MSTIM_POS               					                       (5UL)
#define SPI_IMR_MSTIM_MASK      							                (0x1UL << SPI_IMR_MSTIM_POS)
#define SPI_IMR_MSTIM                                                                            (SPI_IMR_MSTIM_MASK)

/*Interrupt Status  Register Bit Define*/
#define SPI_ISR_TXEIS_POS               					                       (0UL)
#define SPI_ISR_TXEIS_MASK      							                (0x1UL << SPI_ISR_TXEIS_POS)
#define SPI_ISR_TXEIS                                                                             (SPI_ISR_TXEIS_MASK)

#define SPI_ISR_TXOIS_POS               					                       (1UL)
#define SPI_ISR_TXOIS_MASK      							                (0x1UL << SPI_ISR_TXOIS_POS)
#define SPI_ISR_TXOIS                                                                             (SPI_ISR_TXOIS_MASK)

#define SPI_ISR_RXUIS_POS               					                       (2UL)
#define SPI_ISR_RXUIS_MASK      							                (0x1UL << SPI_ISR_RXUIS_POS)
#define SPI_ISR_RXUIS                                                                             (SPI_ISR_RXUIS_MASK)

#define SPI_ISR_RXOIS_POS               					                       (3UL)
#define SPI_ISR_RXOIS_MASK      							                (0x1UL << SPI_ISR_RXOIS_POS)
#define SPI_ISR_RXOIS                                                                             (SPI_ISR_RXOIS_MASK)

#define SPI_ISR_RXFIS_POS               					                       (4UL)
#define SPI_ISR_RXFIS_MASK      							                (0x1UL << SPI_ISR_RXFIS_POS)
#define SPI_ISR_RXFIS                                                                             (SPI_ISR_RXFIS_MASK)

#define SPI_ISR_MSTIS_POS               					                       (5UL)
#define SPI_ISR_MSTIS_MASK      							                (0x1UL << SPI_ISR_MSTIS_POS)
#define SPI_ISR_MSTIS                                                                             (SPI_ISR_MSTIS_MASK)

/*Raw Interrupt Status Register  Register Bit Define*/
#define SPI_RISR_TXEIR_POS               					                       (0UL)
#define SPI_RISR_TXEIR_MASK      							                (0x1UL << SPI_RISR_TXEIR_POS)
#define SPI_RISR_TXEIR                                                                            (SPI_RISR_TXEIR_MASK)

#define SPI_RISR_TXOIR_POS               					                       (1UL)
#define SPI_RISR_TXOIR_MASK      							                (0x1UL << SPI_RISR_TXOIR_POS)
#define SPI_RISR_TXOIR                                                                             (SPI_RISR_TXOIR_MASK)

#define SPI_RISR_RXUIR_POS               					                       (2UL)
#define SPI_RISR_RXUIR_MASK      							                (0x1UL << SPI_RISR_RXUIR_POS)
#define SPI_RISR_RXUIR                                                                            (SPI_RISR_RXUIR_MASK)

#define SPI_RISR_RXOIR_POS               					                       (3UL)
#define SPI_RISR_RXOIR_MASK      							                (0x1UL << SPI_RISR_RXOIR_POS)
#define SPI_RISR_RXOIR                                                                             (SPI_RISR_RXOIR_MASK)

#define SPI_RISR_RXFIR_POS               					                       (4UL)
#define SPI_RISR_RXFIR_MASK      							                (0x1UL << SPI_RISR_RXFIR_POS)
#define SPI_RISR_RXFIR                                                                             (SPI_RISR_RXFIR_MASK)

#define SPI_RISR_MSTIR_POS               					                       (5UL)
#define SPI_RISR_MSTIR_MASK      							                (0x1UL << SPI_RISR_MSTIR_POS)
#define SPI_RISR_MSTIR                                                                           (SPI_RISR_MSTIR_MASK)

/*Transmit FIFO Overflow Interrupt Clear   Register Bit Define*/
#define SPI_TXOICR_TXOICR_POS               					                (0UL)
#define SPI_TXOICR_TXOICR_MASK      							         (0x1UL << SPI_TXOICR_TXOICR_POS)
#define SPI_TXOICR_TXOICR                                                                     (SPI_TXOICR_TXOICR_MASK)

/*  Receive FIFO Overflow Interrupt Clear rRegister Bit Define*/
#define SPI_RXOICR_RXOICR_POS               					                (0UL)
#define SPI_RXOICR_RXOICR_MASK      							         (0x1UL << SPI_RXOICR_RXOICR_POS)
#define SPI_RXOICR_RXOICR                                                                     (SPI_RXOICR_RXOICR_MASK)

/*Receive FIFO Underflow Interrupt Clear Register Register Bit Define*/
#define SPI_RXUICR_RXUICR_POS               					                (0UL)
#define SPI_RXUICR_RXUICR_MASK      							         (0x1UL << SPI_RXUICR_RXUICR_POS)
#define SPI_RXUICR_RXUICR                                                                     (SPI_RXUICR_RXUICR_MASK)

/*Multi-Master Interrupt Clear  Register Bit Define*/
#define SPI_MSTICR_MSTICR_POS               					                (0UL)
#define SPI_MSTICR_MSTICR_MASK      							         (0x1UL << SPI_MSTICR_MSTICR_POS)
#define SPI_MSTICR_MSTICR                                                                    (SPI_MSTICR_MSTICR_MASK)

/*Interrupt Clear  Register Bit Define*/
#define SPI_ICR_ICR_POS               					                             (0UL)
#define SPI_ICR_ICR_MASK      							               (0x1UL << SPI_ICR_ICR_POS)
#define SPI_ICR_ICR                                                                                 (SPI_ICR_ICR_MASK)

/*DMA Control  Register Bit Define*/
#define SPI_DMACR_TDMAE_POS               					               (0UL)
#define SPI_DMACR_TDMAE_MASK      							        (0x1UL << SPI_DMACR_TDMAE_POS)
#define SPI_DMACR_TDMAE                                                                      (SPI_DMACR_TDMAE_MASK)

#define SPI_DMACR_RDMAE_POS               					               (1UL)
#define SPI_DMACR_RDMAE_MASK      							        (0x1UL << SPI_DMACR_RDMAE_POS)
#define SPI_DMACR_RDMAE                                                                      (SPI_DMACR_RDMAE_MASK)

/*DMA Transmit Data Level Register Bit Define*/
#define SPI_DMATDLR_DMATDL_POS               					         (0UL)
#define SPI_DMATDLR_DMATDL_MASK      							  (0x1FUL << SPI_DMATDLR_DMATDL_POS)
#define SPI_DMATDLR_DMATDL                                                                 (SPI_DMATDLR_DMATDL_MASK)

/*DMA Receive Data Level Register Bit Define*/
#define SPI_DMARDLR_DMARDL_POS               					         (0UL)
#define SPI_DMARDLR_DMARDL_MASK      							  (0x1FUL << SPI_DMARDLR_DMARDL_POS)
#define SPI_DMARDLR_DMARDL                                                                 (SPI_DMARDLR_DMARDL_MASK)

/*Identification Register Bit Define*/
#define SPI_IDR_IDCODE_POS               			              		         (0UL)
#define SPI_IDR_IDCODE_MASK      							         (0xFFFFFFFFUL << SPI_IDR_IDCODE_POS)
#define SPI_IDR_IDCODE                                                                           (SPI_IDR_IDCODE_MASK)

/*coreKit version ID Register Bit Define*/
#define SPI_SSIVERSIONID_SSICOMPVERSION_POS               			   (0UL)
#define SPI_SSIVERSIONID_SSICOMPVERSION_MASK      				   (0xFFFFFFFFUL << SPI_SSIVERSIONID_SSICOMPVERSION_POS)
#define SPI_SSIVERSIONID_SSICOMPVERSION                                          (SPI_SSIVERSIONID_SSICOMPVERSION_MASK)

/*Data Register x Register Bit Define*/
#define SPI_DRX_DR_POS               			              		                (0UL)
#define SPI_DRX_DR_MASK      							                       (0xFFFFUL << SPI_DRX_DR_POS)
#define SPI_DRX_DR                                                                                  (SPI_DRX_DR_MASK)

/*RX Sample Delay Register Bit Define*/
#define SPI_RXSAMPLEDLY_RSD_POS               			              	   (0UL)
#define SPI_RXSAMPLEDLY_RSD_MASK      							   (0xFFUL << SPI_RXSAMPLEDLY_RSD_POS)
#define SPI_RXSAMPLEDLY_RSD                                                                 (SPI_RXSAMPLEDLY_RSD_MASK)

/*SPI Control Register Bit Define*/
#define SPI_SPICTRLR0_TRANS_TYPE_POS               			                 (0UL)
#define SPI_SPICTRLR0_TRANS_TYPE_MASK      						   (0x3UL << SPI_SPICTRLR0_TRANS_TYPE_POS)
#define SPI_SPICTRLR0_TRANS_TYPE                                                        (SPI_SPICTRLR0_TRANS_TYPE_MASK)

#define SPI_SPICTRLR0_ADDR_L_POS                  			                 (2UL)
#define SPI_SPICTRLR0_ADDR_L_MASK      					        	   (0xFUL << SPI_SPICTRLR0_ADDR_L_POS)
#define SPI_SPICTRLR0_ADDR_L                                                                (SPI_SPICTRLR0_ADDR_L_MASK)

#define SPI_SPICTRLR0_INST_L_POS                  			                        (8UL)
#define SPI_SPICTRLR0_INST_L_MASK      					        	   (0x3UL << SPI_SPICTRLR0_INST_L_POS)
#define SPI_SPICTRLR0_INST_L                                                                 (SPI_SPICTRLR0_INST_L_MASK)

#define SPI_SPICTRLR0_WAIT_CYCLES_POS                  			           (11UL)
#define SPI_SPICTRLR0_WAIT_CYCLES_MASK      					    (0x1FUL << SPI_SPICTRLR0_WAIT_CYCLES_POS)
#define SPI_SPICTRLR0_WAIT_CYCLES                                                       (SPI_SPICTRLR0_WAIT_CYCLES_MASK)

#define SPI_SPICTRLR0_SPI_DDR_EN_POS                  			           (16UL)
#define SPI_SPICTRLR0_SPI_DDR_EN_MASK      					           (0x1UL << SPI_SPICTRLR0_SPI_DDR_EN_POS)
#define SPI_SPICTRLR0_SPI_DDR_EN                                                         (SPI_SPICTRLR0_SPI_DDR_EN_MASK)

#define SPI_SPICTRLR0_INST_DDR_EN_POS                  			           (17UL)
#define SPI_SPICTRLR0_INST_DDR_EN_MASK      					    (0x1UL << SPI_SPICTRLR0_INST_DDR_EN_POS)
#define SPI_SPICTRLR0_INST_DDR_EN                                                       (SPI_SPICTRLR0_INST_DDR_EN_MASK)

#define SPI_SPICTRLR0_SPI_RXDS_EN_POS                  			           (18UL)
#define SPI_SPICTRLR0_SPI_RXDS_EN_MASK      					    (0x1UL << SPI_SPICTRLR0_SPI_RXDS_EN_POS)
#define SPI_SPICTRLR0_SPI_RXDS_EN                                                       (SPI_SPICTRLR0_SPI_RXDS_EN_MASK)

/*Transmit Drive Edge Register Bit Define*/
#define SPI_TXDDRIVEEDGE_TDE_POS                  			                 (0UL)
#define SPI_TXDDRIVEEDGE_TDE_MASK      						          (0xFFUL << SPI_TXDDRIVEEDGE_TDE_POS)
#define SPI_TXDDRIVEEDGE_TDE                                                               (SPI_TXDDRIVEEDGE_TDE_MASK)

/*RSVD - Reserved address location Bit Define*/
#define SPI_RSVD_RSVD_POS                  			                             (0UL)
#define SPI_RSVD_RSVD_MASK      						                     (0xFFUL << SPI_RSVD_RSVD_POS)
#define SPI_RSVD_RSVD                                                                          (SPI_RSVD_RSVD_MASK)

#ifdef __cplusplus
}
#endif

#endif /* MS_TIMER_H_ */

