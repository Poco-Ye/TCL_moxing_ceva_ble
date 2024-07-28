/*
 * ms_qspi_reg.h
 *
 *  Created on: 2021年12月21日
 *      Author: che.jiang
 */

#ifndef MS_QSPI_REGS_H_
#define MS_QSPI_REGS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    volatile uint32_t CFG;
    volatile uint32_t DRIR;
    volatile uint32_t DWIR;
    volatile uint32_t DDR;
    volatile uint32_t RDCR;
    volatile uint32_t DSCR;
    volatile uint32_t SPCR;
    volatile uint32_t IAATR;
    volatile uint32_t DPCR;
    volatile uint32_t RAR;
    volatile uint32_t MBCR;
    volatile uint32_t SFLR;
    volatile uint32_t TTR;
    volatile uint32_t RTR;
    volatile uint32_t WCCR;
    volatile uint32_t PER;
    volatile uint32_t ISR;
    volatile uint32_t IMR;
    volatile uint32_t UNUSED[2];
    volatile uint32_t LWPR;
    volatile uint32_t UWPR;
    volatile uint32_t WPR;
    volatile uint32_t UNUSED1;
    volatile uint32_t IRTCR;
    volatile uint32_t IRTWR;
    volatile uint32_t IRTSAR;
    volatile uint32_t IRTNBR;
    volatile uint32_t IWTCR;
    volatile uint32_t IWTWR;
    volatile uint32_t IWTSAR;
    volatile uint32_t IWTNBR;
    volatile uint32_t ITARR;
    volatile uint32_t FCCMR;
    volatile uint32_t UNUSED2[2];
    volatile uint32_t FCCR;
    volatile uint32_t FCAR;
    volatile uint32_t UNUSED3[2];
    volatile uint32_t FCRDR_L;
    volatile uint32_t FCRDR_H;
    volatile uint32_t FCWDR_L;
    volatile uint32_t FCWDR_H;
    volatile uint32_t PFSR;
    volatile uint32_t UNUSED4[18];
    volatile uint32_t MIR;
} QSPI_Type;

/*Basic configuration fileds of controller*/
#define QSPI_CFG_QSPI_EN_POS               						(0UL)
#define QSPI_CFG_QSPI_EN_MASK      								(0x1UL << QSPI_CFG_QSPI_EN_POS)
#define QSPI_CFG_QSPI_EN    									QSPI_CFG_QSPI_EN_MASK
#define QSPI_CFG_QSPI_CPOL_POS               					(1UL)
#define QSPI_CFG_QSPI_CPOL_MASK      							(0x1UL << QSPI_CFG_QSPI_CPOL_POS)
#define QSPI_CFG_QSPI_CPOL   									QSPI_CFG_QSPI_CPOL_MASK
#define QSPI_CFG_QSPI_CPHA_POS               					(2UL)
#define QSPI_CFG_QSPI_CPHA_MASK      							(0x1UL << QSPI_CFG_QSPI_CPHA_POS)
#define QSPI_CFG_QSPI_CPHA   									QSPI_CFG_QSPI_CPHA_MASK
#define QSPI_CFG_DIRAC_EN_POS               					(7UL)
#define QSPI_CFG_DIRAC_EN_MASK      							(0x1UL << QSPI_CFG_DIRAC_EN_POS)
#define QSPI_CFG_DIRAC_EN   									QSPI_CFG_DIRAC_EN_MASK
#define QSPI_CFG_LEGACY_EN_POS               					(8UL)
#define QSPI_CFG_LEGACY_EN_MASK      							(0x1UL << QSPI_CFG_LEGACY_EN_POS)
#define QSPI_CFG_LEGACY_EN_EN   								QSPI_CFG_LEGACY_EN_MASK
#define QSPI_CFG_PER_SEL_DEC_POS               					(9UL)
#define QSPI_CFG_PER_SEL_DEC_MASK      							(0x1UL << QSPI_CFG_PER_SEL_DEC_POS)
#define QSPI_CFG_PER_SEL_DEC   								    QSPI_CFG_PER_SEL_DEC_MASK
#define QSPI_CFG_PER_SEL_POS               			    		(10UL)
#define QSPI_CFG_PER_SEL_MASK      						    	(0xFUL << QSPI_CFG_PER_SEL_POS)
#define QSPI_CFG_PER_SEL   								        QSPI_CFG_PER_SEL_MASK
#define QSPI_CFG_WP_EN_POS               			    		(14UL)
#define QSPI_CFG_WP_EN_MASK      						    	(0x1UL << QSPI_CFG_WP_EN_POS)
#define QSPI_CFG_WP_EN   								        QSPI_CFG_WP_EN_MASK
#define QSPI_CFG_DMA_EN_POS               			            (15UL)
#define QSPI_CFG_DMA_EN_MASK      					            (0x1UL << QSPI_CFG_DMA_EN_POS)
#define QSPI_CFG_DMA_EN 								        QSPI_CFG_DMA_EN_MASK
#define QSPI_CFG_AHB_ADR_REMAP_EN_POS               			(16UL)
#define QSPI_CFG_AHB_ADR_REMAP_EN_MASK      					(0x1UL << QSPI_CFG_AHB_ADR_REMAP_EN_POS)
#define QSPI_CFG_AHB_ADR_REMAP_EN 								QSPI_CFG_AHB_ADR_REMAP_EN_MASK
#define QSPI_CFG_XIP_NEXT_POS               			        (17UL)
#define QSPI_CFG_XIP_NEXT_MASK      					        (0x1UL << QSPI_CFG_XIP_NEXT_POS)
#define QSPI_CFG_XIP_NEXT 								        QSPI_CFG_XIP_NEXT_MASK
#define QSPI_CFG_XIP_NOW_POS               			            (18UL)
#define QSPI_CFG_XIP_NOW_MASK      					            (0x1UL << QSPI_CFG_XIP_NOW_POS)
#define QSPI_CFG_XIP_NOW 								        QSPI_CFG_XIP_NOW_MASK
#define QSPI_CFG_BAUD_RATE_CFG_POS        			            (19UL)
#define QSPI_CFG_BAUD_RATE_CFG_MASK					            (0xFUL << QSPI_CFG_BAUD_RATE_CFG_POS)
#define QSPI_CFG_BAUD_RATE_CFG 								    QSPI_CFG_BAUD_RATE_CFG_MASK
#define QSPI_CFG_AHB_DEC_EN_POS        			                (23UL)
#define QSPI_CFG_AHB_DEC_EN_MASK					            (0x1UL << QSPI_CFG_AHB_DEC_EN_POS)
#define QSPI_CFG_AHB_DEC_EN 								    QSPI_CFG_AHB_DEC_EN_MASK
#define QSPI_CFG_DTR_EN_POS        			                    (24UL)
#define QSPI_CFG_DTR_EN_MASK					                (0x1UL << QSPI_CFG_DTR_EN_POS)
#define QSPI_CFG_DTR_EN 								        QSPI_CFG_DTR_EN_MASK
#define QSPI_CFG_RE_TIME_ST_POS        			                (31UL)
#define QSPI_CFG_RE_TIME_ST_MASK					            (0x1UL << QSPI_CFG_RE_TIME_ST_POS)
#define QSPI_CFG_RE_TIME_ST 								    QSPI_CFG_RE_TIME_ST_MASK

/*Device Read Instruction Register*/
#define QSPI_DRIR_R_OPCODE_POS               					(0UL)
#define QSPI_DRIR_R_OPCODE_MASK      							(0xFFUL << QSPI_DRIR_R_OPCODE_POS)
#define QSPI_DRIR_R_OPCODE    									QSPI_DRIR_R_OPCODE_MASK
#define QSPI_DRIR_INSTR_TYPE_POS               			        (8UL)
#define QSPI_DRIR_INSTR_TYPE_MASK      					        (0x3UL << QSPI_DRIR_INSTR_TYPE_POS)
#define QSPI_DRIR_INSTR_TYPE   							        QSPI_DRIR_INSTR_TYPE_MASK
#define QSPI_DRIR_DDR_EN_POS               					    (10UL)
#define QSPI_DRIR_DDR_EN_MASK      							    (0x1UL << QSPI_DRIR_DDR_EN_POS)
#define QSPI_DRIR_DDR_EN   									    QSPI_DRIR_DDR_EN_MASK
#define QSPI_DRIR_ADR_TRAN_TYPE_POS               			    (12UL)
#define QSPI_DRIR_ADR_TRAN_TYPE_MASK      						(0x3UL << QSPI_DRIR_ADR_TRAN_TYPE_POS)
#define QSPI_DRIR_ADR_TRAN_TYPE 								QSPI_DRIR_ADR_TRAN_TYPE_MASK
#define QSPI_DRIR_DATA_TRAN_TYPE_POS               			    (16UL)
#define QSPI_DRIR_DATA_TRAN_TYPE_MASK      						(0x3UL << QSPI_DRIR_DATA_TRAN_TYPE_POS)
#define QSPI_DRIR_DATA_TRAN_TYPE 								QSPI_DRIR_DATA_TRAN_TYPE_MASK
#define QSPI_DRIR_MODE_BIT_EN_POS               			    (20UL)
#define QSPI_DRIR_MODE_BIT_EN_MASK      					    (0x1UL << QSPI_DRIR_MODE_BIT_EN_POS)
#define QSPI_DRIR_MODE_BIT_EN 								    QSPI_DRIR_MODE_BIT_EN_MASK
#define QSPI_DRIR_DUMMY_CLOCK_NUM_POS               			(24UL)
#define QSPI_DRIR_DUMMY_CLOCK_NUM_MASK      					(0x1FUL << QSPI_DRIR_DUMMY_CLOCK_NUM_POS)
#define QSPI_DRIR_DUMMY_CLOCK_NUM								QSPI_DRIR_DUMMY_CLOCK_NUM_MASK

/*Device Write Instruction Register*/
#define QSPI_DWIR_W_OPCODE_POS               					(0UL)
#define QSPI_DWIR_W_OPCODE_MASK      							(0xFFUL << QSPI_DWIR_W_OPCODE_POS)
#define QSPI_DWIR_W_OPCODE    									QSPI_DWIR_W_OPCODE_MASK
#define QSPI_DWIR_WEL_DISABLE_POS               				(8UL)
#define QSPI_DWIR_WEL_DISABLE_MASK      						(0x1UL << QSPI_DWIR_WEL_DISABLE_POS)
#define QSPI_DWIR_WEL_DISABLE   								QSPI_DWIR_WEL_DISABLE_MASK
#define QSPI_DWIR_ADR_TRAN_TYPE_POS               			    (12UL)
#define QSPI_DWIR_ADR_TRAN_TYPE_MASK      						(0x3UL << QSPI_DWIR_ADR_TRAN_TYPE_POS)
#define QSPI_DWIR_ADR_TRAN_TYPE 								QSPI_DWIR_ADR_TRAN_TYPE_MASK
#define QSPI_DWIR_DATA_TRAN_TYPE_POS               			    (16UL)
#define QSPI_DWIR_DATA_TRAN_TYPE_MASK      						(0x3UL << QSPI_DWIR_DATA_TRAN_TYPE_POS)
#define QSPI_DWIR_DATA_TRAN_TYPE 								QSPI_DWIR_DATA_TRAN_TYPE_MASK
#define QSPI_DWIR_DUMMY_CLOCK_NUM_POS               			(24UL)
#define QSPI_DWIR_DUMMY_CLOCK_NUM_MASK      					(0x1FUL << QSPI_DWIR_DUMMY_CLOCK_NUM_POS)
#define QSPI_DWIR_DUMMY_CLOCK_NUM								QSPI_DWIR_DUMMY_CLOCK_NUM_MASK

/*Device Delay Register*/
#define QSPI_DDR_CSSOT_POS               					    (0UL)
#define QSPI_DDR_CSSOT_MASK      							    (0xFFUL << QSPI_DDR_CSSOT_POS)
#define QSPI_DDR_CSSOT    									    QSPI_DDR_CSSOT_MASK
#define QSPI_DDR_CSEOT_POS               					    (8UL)
#define QSPI_DDR_CSEOT_MASK      							    (0xFFUL << QSPI_DDR_CSEOT_POS)
#define QSPI_DDR_CSEOT    									    QSPI_DDR_CSEOT_MASK
#define QSPI_DDR_CSDADS_POS               					    (16UL)
#define QSPI_DDR_CSDADS_MASK      							    (0xFFUL << QSPI_DDR_CSDASD_POS)
#define QSPI_DDR_CSDADS    									    QSPI_DDR_CSDADS_MASK
#define QSPI_DDR_CSDA_POS               					    (24UL)
#define QSPI_DDR_CSDA_MASK      							    (0xFFUL << QSPI_DDR_CSDA_POS)
#define QSPI_DDR_CSDA    									    QSPI_DDR_CSDA_MASK

/*Read Data Capture Register*/
#define QSPI_RDCR_ADAPT_LOOPBACK_POS               				(0UL)
#define QSPI_RDCR_ADAPT_LOOPBACK_MASK      						(0x1UL << QSPI_RDCR_ADAPT_LOOPBACK_POS)
#define QSPI_RDCR_ADAPT_LOOPBACK   							    QSPI_RDCR_ADAPT_LOOPBACK_MASK
#define QSPI_RDCR_DELAY_NUMBER_POS               				(1UL)
#define QSPI_RDCR_DELAY_NUMBER_MASK      						(0xFUL << QSPI_RDCR_DELAY_NUMBER_POS)
#define QSPI_RDCR_DELAY_NUMBER   							    QSPI_RDCR_DELAY_NUMBER_MASK
#define QSPI_RDCR_SAMP_EDGE_POS               				    (5UL)
#define QSPI_RDCR_SAMP_EDGE_MASK      						    (0x1UL << QSPI_RDCR_SAMP_EDGE_POS)
#define QSPI_RDCR_SAMP_EDGE   							        QSPI_RDCR_SAMP_EDGE_MASK
#define QSPI_RDCR_DELAY_TRANS_POS               				(16UL)
#define QSPI_RDCR_DELAY_TRANS_MASK      						(0xFUL << QSPI_RDCR_DELAY_TRANS_POS)
#define QSPI_RDCR_DELAY_TRANS  							        QSPI_RDCR_DELAY_TRANS_MASK

/*Device Size Configuration Register*/
#define QSPI_DSCR_ADR_B_NUM_POS               				    (0UL)
#define QSPI_DSCR_ADR_B_NUM_MASK      						    (0xFUL << QSPI_DSCR_ADR_B_NUM_POS)
#define QSPI_DSCR_ADR_B_NUM   							        QSPI_RDCR_ADR_B_NUM_MASK
#define QSPI_DSCR_PAGE_NUM_POS               				    (4UL)
#define QSPI_DSCR_PAGE_NUM_MASK      						    (0xFFFUL << QSPI_DSCR_PAGE_NUM_POS)
#define QSPI_DSCR_PAGE_NUM   							        QSPI_DSCR_PAGE_NUM_MASK
#define QSPI_DSCR_BLOCK_NUM_POS               				    (16UL)
#define QSPI_DSCR_BLOCK_NUM_MASK      						    (0x1FUL << QSPI_DSCR_BLOCK_NUM_POS)
#define QSPI_DSCR_BLOCK_NUM   							        QSPI_DSCR_BLOCK_NUM_MASK
#define QSPI_DSCR_CS0_SIZE_POS               				    (21UL)
#define QSPI_DSCR_CS0_SIZE_MASK      						    (0x3UL << QSPI_DSCR_CS0_SIZE_POS)
#define QSPI_DSCR_CS0_SIZE   							        QSPI_DSCR_CS0_SIZE_MASK
#define QSPI_DSCR_CS1_SIZE_POS               				    (23UL)
#define QSPI_DSCR_CS1_SIZE_MASK      						    (0x3UL << QSPI_DSCR_CS1_SIZE_POS)
#define QSPI_DSCR_CS1_SIZE   							        QSPI_DSCR_CS1_SIZE_MASK
#define QSPI_DSCR_CS2_SIZE_POS               				    (25UL)
#define QSPI_DSCR_CS2_SIZE_MASK      						    (0x3UL << QSPI_DSCR_CS2_SIZE_POS)
#define QSPI_DSCR_CS2_SIZE   							        QSPI_DSCR_CS2_SIZE_MASK
#define QSPI_DSCR_CS3_SIZE_POS               				    (27UL)
#define QSPI_DSCR_CS3_SIZE_MASK      						    (0x3UL << QSPI_DSCR_CS3_SIZE_POS)
#define QSPI_DSCR_CS3_SIZE   							        QSPI_DSCR_CS3_SIZE_MASK

/*SRAM Partion Configuration Register*/
#define QSPI_SPCR_RD_PARTITION_POS               				(0UL)
#define QSPI_SPCR_RD_PARTITION_MASK      						(0xFFUL << QSPI_SPCR_RD_PARTITION_POS)
#define QSPI_SPCR_RD_PARTITION  							    QSPI_SPCR_RD_PARTITION_MASK

/*Indirect AHB Address Trigger Register*/
#define QSPI_IAATR_IND_ADR_TRIG_POS               				(0UL)
#define QSPI_IAATR_IND_ADR_TRIG_MASK      						(0xFFFFFFFFUL)
#define QSPI_IAATR_IND_ADR_TRIG 							    QSPI_IAATR_IND_ADR_TRIG_MASK

/*DMA Peripheral Configuration Register*/
#define QSPI_DPCR_SING_TYPE_REQ_POS               				(0UL)
#define QSPI_DPCR_SING_TYPE_REQ_MASK      						(0xFUL << QSPI_DPCR_SING_TYPE_REQ_POS)
#define QSPI_DPCR_SING_TYPE_REQ							        QSPI_DPCR_SING_TYPE_REQ_MASK
#define QSPI_DPCR_BURST_TYPE_REQ_POS               				(8UL)
#define QSPI_DPCR_BURST_TYPE_REQ_MASK      						(0xFUL << QSPI_DPCR_BURST_TYPE_REQ_POS)
#define QSPI_DPCR_BURST_TYPE_REQ							    QSPI_DPCR_BURST_TYPE_REQ_MASK

/*Remap Address Register*/
#define QSPI_RAR_REMAP_ADR_POS               					(0UL)
#define QSPI_RAR_REMAP_ADR_MASK      							(0xFFFFFFFFUL)
#define QSPI_RAR_REMAP_ADR						            	QSPI_RAR_REMAP_ADR_MASK

/*Mode Bit Configrution Register*/
#define QSPI_MBCR_MODE_BIT_POS               					(0UL)
#define QSPI_MBCR_MODE_BIT_MASK      							(0xFFUL << QSPI_MBCR_MODE_BIT_POS)
#define QSPI_MBCR_MODE_BIT						            	QSPI_MBCR_MODE_BIT_MASK

/*SRAM Fill Level Register*/
#define QSPI_SFLR_SRAM_RFILL_LEVEL_POS               			(0UL)
#define QSPI_SFLR_SRAM_RFILL_LEVEL_MASK      					(0xFFFFUL << QSPI_SFLR_SRAM_RFILL_LEVEL_POS)
#define QSPI_SFLR_SRAM_RFILL_LEVEL						        QSPI_SFLR_SRAM_RFILL_LEVEL_MASK
#define QSPI_SFLR_SRAM_WFILL_LEVEL_POS               			(16UL)
#define QSPI_SFLR_SRAM_WFILL_LEVEL_MASK      					(0xFFFFUL << QSPI_SFLR_SRAM_WFILL_LEVEL_POS)
#define QSPI_SFLR_SRAM_WFILL_LEVEL						        QSPI_SFLR_SRAM_WFILL_LEVEL_MASK

/*TX Threshold Register*/
#define QSPI_TTR_TX_THRESH_POS               		         	(0UL)
#define QSPI_TTR_TX_THRESH_MASK      				        	(0x1FUL << QSPI_TTR_TX_THRESH_POS)
#define QSPI_TTR_TX_THRESH						       			QSPI_TTR_TX_THRESH_MASK

/*RX Threshold Register*/
#define QSPI_RTR_RX_THRESH_POS               		         	(0UL)
#define QSPI_RTR_RX_THRESH_MASK      				        	(0x1FUL << QSPI_RTR_RX_THRESH_POS)
#define QSPI_RTR_RX_THRESH						       			QSPI_RTR_RX_THRESH_MASK

/*Write Completion Control Register*/
#define QSPI_WCCR_OPCODE_POS               		         		(0UL)
#define QSPI_WCCR_OPCODE_MASK      				        		(0xFFUL << QSPI_WCCR_OPCODE_POS)
#define QSPI_WCCR_OPCODE						       			QSPI_WCCR_OPCODE_MASK
#define QSPI_WCCR_POLLING_BIT_INDEX_POS               		    (8UL)
#define QSPI_WCCR_POLLING_BIT_INDEX_MASK      				    (0x7UL << QSPI_WCCR_POLLING_BIT_INDEX_POS)
#define QSPI_WCCR_POLLING_BIT_INDEX						       	QSPI_WCCR_POLLING_BIT_INDEX_MASK
#define QSPI_WCCR_POLLING_POLAR_POS               		   		(13UL)
#define QSPI_WCCR_POLLING_POLAR_MASK      				    	(0x1UL << QSPI_WCCR_POLLING_POLAR_POS)
#define QSPI_WCCR_POLLING_POLAR						       		QSPI_WCCR_POLLING_POLAR_MASK
#define QSPI_WCCR_POLLING_DISABLE_POS               		   	(14UL)
#define QSPI_WCCR_POLLING_DISABLE_MASK      				    (0x1UL << QSPI_WCCR_POLLING_DISABLE_POS)
#define QSPI_WCCR_POLLING_DISABLE						       	QSPI_WCCR_POLLING_DISABLE_MASK
#define QSPI_WCCR_POLLING_EN_END_POS               		   		(15UL)
#define QSPI_WCCR_POLLING_EN_END_MASK      				    	(0x1UL << QSPI_WCCR_POLLING_EN_END_POS)
#define QSPI_WCCR_POLLING_EN_END						       	QSPI_WCCR_POLLING_EN_END_MASK
#define QSPI_WCCR_POLLING_CNT_POS               		   		(16UL)
#define QSPI_WCCR_POLLING_CNT_MASK      				    	(0xFFUL << QSPI_WCCR_POLLING_CNT_POS)
#define QSPI_WCCR_POLLING_CNT						       		QSPI_WCCR_POLLING_CNT_MASK
#define QSPI_WCCR_POLLING_REP_DELAY_POS               		   	(23UL)
#define QSPI_WCCR_POLLING_REP_DELAY_MASK      				    (0xFFUL << QSPI_WCCR_POLLING_REP_DELAY_POS)
#define QSPI_WCCR_POLLING_REP_DELAY						       	QSPI_WCCR_POLLING_REP_DELAY_MASK

/*Polling Expiration Register*/
#define QSPI_PER_MAX_NUM_POLLS_POS               		        (0UL)
#define QSPI_PER_MAX_NUM_POLLS_MASK      				        (0xFFFFFFFFUL)
#define QSPI_PER_MAX_NUM_POLLS						       		QSPI_PER_MAX_NUM_POLLS_MASK

/*Interrupt Status Register*/
#define QSPI_ISR_MODE_FAIL_POS               		        	(0UL)
#define QSPI_ISR_MODE_FAIL_MASK      				        	(0x1UL << QSPI_ISR_MODE_FAIL_POS)
#define QSPI_ISR_MODE_FAIL						       			QSPI_ISR_MODE_FAIL_MASK
#define QSPI_ISR_UNDERFLOW_DETECTED_POS               		    (1UL)
#define QSPI_ISR_UNDERFLOW_DETECTED_MASK      				    (0x1UL << QSPI_ISR_UNDERFLOW_DETECTED_POS)
#define QSPI_ISR_UNDERFLOW_DETECTED						       	QSPI_ISR_UNDERFLOW_DETECTED_MASK
#define QSPI_ISR_LAST_IND_COMPLETED_POS               		    (2UL)
#define QSPI_ISR_LAST_IND_COMPLETED_MASK      				    (0x1UL << QSPI_ISR_LAST_IND_COMPLETED_POS)
#define QSPI_ISR_LAST_IND_COMPLETED						       	QSPI_ISR_LAST_IND_COMPLETED_MASK
#define QSPI_ISR_IND_NOACCEPT_POS               		    	(3UL)
#define QSPI_ISR_IND_NOACCEPT_MASK      				    	(0x1UL << QSPI_ISR_IND_NOACCEPT_POS)
#define QSPI_ISR_IND_NOACCEPT						       		QSPI_ISR_IND_NOACCEPT_MASK
#define QSPI_ISR_WP_ATTEMPT_POS               		    		(4UL)
#define QSPI_ISR_WP_ATTEMPT_MASK      				    		(0x1UL << QSPI_ISR_WP_ATTEMPT_POS)
#define QSPI_ISR_WP_ATTEMPT						       			QSPI_ISR_WP_ATTEMPT_MASK
#define QSPI_ISR_ILLEGAL_AHB_ACCESS_POS               		    (5UL)
#define QSPI_ISR_ILLEGAL_AHB_ACCESS_MASK      				    (0x1UL << QSPI_ISR_ILLEGAL_AHB_ACCESS_POS)
#define QSPI_ISR_ILLEGAL_AHB_ACCESS					       		QSPI_ISR_ILLEGAL_AHB_ACCESS_MASK
#define QSPI_ISR_IND_WATERMARK_POS               		    	(6UL)
#define QSPI_ISR_IND_WATERMARK_MASK      				    	(0x1UL << QSPI_ISR_IND_WATERMARK_POS)
#define QSPI_ISR_IND_WATERMARK				       				QSPI_ISR_IND_WATERMARK_MASK
#define QSPI_ISR_RECVIVE_OVERFLOW_POS               		    (7UL)
#define QSPI_ISR_RECVIVE_OVERFLOW_MASK      				    (0x1UL << QSPI_ISR_RECVIVE_OVERFLOW_POS)
#define QSPI_ISR_RECVIVE_OVERFLOW				       			QSPI_ISR_RECVIVE_OVERFLOW_MASK
#define QSPI_ISR_TX_FIFO_NOT_FULL_POS               		    (8UL)
#define QSPI_ISR_TX_FIFO_NOT_FULL_MASK      				    (0x1UL << QSPI_ISR_TX_FIFO_NOT_FULL_POS)
#define QSPI_ISR_TX_FIFO_NOT_FULL				       			QSPI_ISR_TX_FIFO_NOT_FULL_MASK
#define QSPI_ISR_TX_FIFO_FULL_POS               		    	(9UL)
#define QSPI_ISR_TX_FIFO_FULL_MASK      				    	(0x1UL << QSPI_ISR_TX_FIFO_FULL_POS)
#define QSPI_ISR_TX_FIFO_FULL				       				QSPI_ISR_TX_FIFO_FULL_MASK
#define QSPI_ISR_RX_FIFO_NOT_FULL_POS               		    (10UL)
#define QSPI_ISR_RX_FIFO_NOT_FULL_MASK      				    (0x1UL << QSPI_ISR_RX_FIFO_NOT_FULL_POS)
#define QSPI_ISR_RX_FIFO_NOT_FULL				       			QSPI_ISR_RX_FIFO_NOT_FULL_MASK
#define QSPI_ISR_RX_FIFO_FULL_POS               		    	(11UL)
#define QSPI_ISR_RX_FIFO_FULL_MASK      				    	(0x1UL << QSPI_ISR_RX_FIFO_FULL_POS)
#define QSPI_ISR_RX_FIFO_FULL				       				QSPI_ISR_RX_FIFO_FULL_MASK
#define QSPI_ISR_IND_READ_POS               		    		(12UL)
#define QSPI_ISR_IND_READ_MASK      				    		(0x1UL << QSPI_ISR_IND_READ_POS)
#define QSPI_ISR_IND_READ				       					QSPI_ISR_IND_READ_MASK
#define QSPI_ISR_MAX_POLLS_FULL_POS               		    	(13UL)
#define QSPI_ISR_MAX_POLLS_FULL_MASK      				    	(0x1UL << QSPI_ISR_MAX_POLLS_FULL_POS)
#define QSPI_ISR_MAX_POLLS_FULL				       				QSPI_ISR_MAX_POLLS_FULL_MASK
#define QSPI_ISR_STIG_REQUEST_INT_POS               		    (14UL)
#define QSPI_ISR_STIG_REQUEST_INT_MASK      				    (0x1UL << QSPI_ISR_STIG_REQUEST_INT_POS)
#define QSPI_ISR_STIG_REQUEST_INT			       				QSPI_ISR_STIG_REQUEST_INT_MASK

/*Interrupt Mask Register*/
#define QSPI_IMR_INT_MASK_POS               		        	(0UL)
#define QSPI_IMR_INT_MASK_MASK      				        	(0x7FFFUL << QSPI_IMR_INT_MASK_POS)
#define QSPI_IMR_INT_MASK						       			QSPI_IMR_INT_MASK_MASK

/*Lower Write Protection Register*/
#define QSPI_LWPR_WP_L_POS               		        		(0UL)
#define QSPI_LWPR_WP_L_MASK      				        		(0xFFFFFFFFUL)
#define QSPI_LWPR_WP_L						       				QSPI_LWPR_WP_L_MASK

/*Upper Write Protection Register*/
#define QSPI_UWPR_WP_H_POS               		        		(0UL)
#define QSPI_UWPR_WP_H_MASK      				        		(0xFFFFFFFFUL)
#define QSPI_UWPR_WP_H						       				QSPI_UWPR_WP_H_MASK

/*Write Protection Register*/
#define QSPI_WPR_WP_INV_POS               		        		(0UL)
#define QSPI_WPR_WP_INV_MASK      				        		(0x1UL << QSPI_IMR_INT_MASK_POS)
#define QSPI_WPR_WP_INV						       				QSPI_WPR_WP_INV_MASK
#define QSPI_WPR_WP_EN_POS               		        		(1UL)
#define QSPI_WPR_WP_EN_MASK      				        		(0x1UL << QSPI_WPR_WP_EN_POS)
#define QSPI_WPR_WP_EN						       				QSPI_WPR_WP_EN_MASK

/*Indirect Read Transfer Control Register*/
#define QSPI_IRTCR_IND_READ_START_POS               		    (0UL)
#define QSPI_IRTCR_IND_READ_START_MASK      				    (0x1UL << QSPI_IRTCR_IND_READ_START_POS)
#define QSPI_IRTCR_IND_READ_START						       	QSPI_IRTCR_IND_READ_START_MASK
#define QSPI_IRTCR_IND_READ_CANCEL_POS               		    (1UL)
#define QSPI_IRTCR_IND_READ_CANCEL_MASK      				    (0x1UL << QSPI_IRTCR_IND_READ_CANCEL_POS)
#define QSPI_IRTCR_IND_READ_CANCEL						       	QSPI_IRTCR_IND_READ_CANCEL_MASK
#define QSPI_IRTCR_IND_READ_STATUS_POS               		    (2UL)
#define QSPI_IRTCR_IND_READ_STATUS_MASK      				    (0x1UL << QSPI_IRTCR_IND_READ_STATUS_POS)
#define QSPI_IRTCR_IND_READ_STATUS						       	QSPI_IRTCR_IND_READ_STATUS_MASK
#define QSPI_IRTCR_SRAM_FULL_POS               		    		(3UL)
#define QSPI_IRTCR_SRAM_FULL_MASK      				    		(0x1UL << QSPI_IRTCR_SRAM_FULL_POS)
#define QSPI_IRTCR_SRAM_FULL						       		QSPI_IRTCR_SRAM_FULL_MASK
#define QSPI_IRTCR_IND_READ_TWO_POS               		    	(4UL)
#define QSPI_IRTCR_IND_READ_TWO_MASK      				    	(0x1UL << QSPI_IRTCR_IND_READ_TWO_POS)
#define QSPI_IRTCR_IND_READ_TWO						       		QSPI_IRTCR_IND_READ_TWO_MASK
#define QSPI_IRTCR_IND_COMPLETION_STATUS_POS               		(5UL)
#define QSPI_IRTCR_IND_COMPLETION_STATUS_MASK      				(0x1UL << QSPI_IRTCR_IND_COMPLETION_STATUS_POS)
#define QSPI_IRTCR_IND_COMPLETION_STATUS						QSPI_IRTCR_IND_COMPLETION_STATUS_MASK
#define QSPI_IRTCR_IND_COMPLETION_NUM_POS               		(6UL)
#define QSPI_IRTCR_IND_COMPLETION_NUM_MASK      				(0x3UL << QSPI_IRTCR_IND_COMPLETION_NUM_POS)
#define QSPI_IRTCR_IND_COMPLETION_NUM							QSPI_IRTCR_IND_COMPLETION_NUM_MASK

/*Indirect Read Transfer Watermark Register*/
#define QSPI_IRTWR_IND_WATERMARK_POS               		    	(0UL)
#define QSPI_IRTWR_IND_WATERMARK_MASK      				    	(0xFFFFFFFFUL)
#define QSPI_IRTWR_IND_WATERMARK						       	QSPI_IRTWR_IND_WATERMARK_MASK

/*Indirect Read Transfer Start Address Register*/
#define QSPI_IRTSAR_IND_RD_ADR_POS               		    	(0UL)
#define QSPI_IRTSAR_IND_RD_ADR_MASK      				    	(0xFFFFFFFFUL)
#define QSPI_IRTSAR_IND_RD_ADR						       		QSPI_IRTSAR_IND_RD_ADR_MASK

/*Indirect Read Transfer Number Bytes Register*/
#define QSPI_IRTNBR_IND_RD_BYTE_POS               		    	(0UL)
#define QSPI_IRTNBR_IND_RD_BYTE_MASK      				    	(0xFFFFFFFFUL)
#define QSPI_IRTNBR_IND_RD_BYTE						       		QSPI_IRTNBR_IND_RD_BYTE_MASK

/*Indirect Write Transfer Control Register*/
#define QSPI_IWTCR_IND_WRITE_START_POS               		    (0UL)
#define QSPI_IWTCR_IND_WRITE_START_MASK      				    (0x1UL << QSPI_IWTCR_IND_WRITE_START_POS)
#define QSPI_IWTCR_IND_WRITE_START						       	QSPI_IWTCR_IND_WRITE_START_MASK
#define QSPI_IWTCR_IND_WRITE_CANCEL_POS               		    (1UL)
#define QSPI_IWTCR_IND_WRITE_CANCEL_MASK      				    (0x1UL << QSPI_IWTCR_IND_WRITE_CANCEL_POS)
#define QSPI_IWTCR_IND_WRITE_CANCEL						       	QSPI_IWTCR_IND_WRITE_CANCEL_MASK
#define QSPI_IWTCR_IND_WRITE_STATUS_POS               		    (2UL)
#define QSPI_IWTCR_IND_WRITE_STATUS_MASK      				    (0x1UL << QSPI_IWTCR_IND_WRITE_STATUS_POS)
#define QSPI_IWTCR_IND_WRITE_STATUS						       	QSPI_IWTCR_IND_WRITE_STATUS_MASK
#define QSPI_IWTCR_IND_WRITE_TWO_POS               		    	(4UL)
#define QSPI_IWTCR_IND_WRITE_TWO_MASK      				    	(0x1UL << QSPI_IWTCR_IND_WRITE_TWO_POS)
#define QSPI_IWTCR_IND_WRITE_TWO						       	QSPI_IWTCR_IND_WRITE_TWO_MASK
#define QSPI_IWTCR_IND_COMPLETION_STATUS_POS               		(5UL)
#define QSPI_IWTCR_IND_COMPLETION_STATUS_MASK      				(0x1UL << QSPI_IWTCR_IND_COMPLETION_STATUS_POS)
#define QSPI_IWTCR_IND_COMPLETION_STATUS						QSPI_IWTCR_IND_COMPLETION_STATUS_MASK
#define QSPI_IWTCR_IND_COMPLETION_NUM_POS               		(6UL)
#define QSPI_IWTCR_IND_COMPLETION_NUM_MASK      				(0x3UL << QSPI_IWTCR_IND_COMPLETION_NUM_POS)
#define QSPI_IWTCR_IND_COMPLETION_NUM							QSPI_IWTCR_IND_COMPLETION_NUM_MASK

/*Indirect Write Transfer Watermark Register*/
#define QSPI_IWTWR_IND_WATERMARK_POS               		    	(0UL)
#define QSPI_IWTWR_IND_WATERMARK_MASK      				    	(0xFFFFFFFFUL)
#define QSPI_IWTWR_IND_WATERMARK						       	QSPI_IWTWR_IND_WATERMARK_MASK

/*Indirect Write Transfer Start Address Register*/
#define QSPI_IWTSAR_IND_WR_ADR_POS               		    	(0UL)
#define QSPI_IWTSAR_IND_WR_ADR_MASK      				    	(0xFFFFFFFFUL)
#define QSPI_IWTSAR_IND_WR_ADR						       		QSPI_IWTSAR_IND_WR_ADR_MASK

/*Indirect Write Transfer Number Bytes Register*/
#define QSPI_IWTNBR_IND_WR_BYTE_POS               		    	(0UL)
#define QSPI_IWTNBR_IND_WR_BYTE_MASK      				    	(0xFFFFFFFFUL)
#define QSPI_IWTNBR_IND_WR_BYTE						       		QSPI_IWTNBR_IND_WR_BYTE_MASK

/*Indirect Trigger Address Range Register*/
#define QSPI_ITARR_IND_RANGE_BYTE_POS               		    (0UL)
#define QSPI_ITARR_IND_RANGE_BYTE_MASK      				    (0xFFFFFFFFUL)
#define QSPI_ITARR_IND_RANGE_BYTE						       	QSPI_ITARR_IND_RANGE_BYTE_MASK

/*Flash Command Control Memory Register*/
#define QSPI_FCCMR_MEM_REQUEST_T_POS               		    	(0UL)
#define QSPI_FCCMR_MEM_REQUEST_T_MASK      				    	(0x1UL << QSPI_FCCMR_MEM_REQUEST_T_POS)
#define QSPI_FCCMR_MEM_REQUEST_T						       	QSPI_FCCMR_MEM_REQUEST_T_MASK
#define QSPI_FCCMR_MEM_REQUEST_P_POS               		    	(1UL)
#define QSPI_FCCMR_MEM_REQUEST_P_MASK      				    	(0x1UL << QSPI_FCCMR_MEM_REQUEST_P_POS)
#define QSPI_FCCMR_MEM_REQUEST_P						       	QSPI_FCCMR_MEM_REQUEST_P_MASK
#define QSPI_FCCMR_MEM_RDATA_POS               		    		(8UL)
#define QSPI_FCCMR_MEM_RDATA_MASK      				    		(0xFFUL << QSPI_FCCMR_MEM_RDATA_POS)
#define QSPI_FCCMR_MEM_RDATA						       		QSPI_FCCMR_MEM_RDATA_MASK
#define QSPI_FCCMR_MEM_RBYTE_POS               		    		(16UL)
#define QSPI_FCCMR_MEM_RBYTE_MASK      				    		(0x7UL << QSPI_FCCMR_MEM_RBYTE_POS)
#define QSPI_FCCMR_MEM_RBYTE						       		QSPI_FCCMR_MEM_RBYTE_MASK
#define QSPI_FCCMR_MEM_ADR_POS               		    		(20UL)
#define QSPI_FCCMR_MEM_ADR_MASK      				    		(0x1FFUL << QSPI_FCCMR_MEM_ADR_POS)
#define QSPI_FCCMR_MEM_ADR						       			QSPI_FCCMR_MEM_ADR_MASK

/*Flash Command Control Register*/
#define QSPI_FCCR_EXC_CMD_POS               		    		(0UL)
#define QSPI_FCCR_EXC_CMD_MASK      				    		(0x1UL << QSPI_FCCR_EXC_CMD_MASK)
#define QSPI_FCCR_EXC_CMD						       			QSPI_FCCR_EXC_CMD_MASK
#define QSPI_FCCR_STIG_EXC_P_POS               		    		(1UL)
#define QSPI_FCCR_STIG_EXC_P_MASK      				    		(0x1UL << QSPI_FCCR_STIG_EXC_P_POS)
#define QSPI_FCCR_STIG_EXC_P						       		QSPI_FCCR_STIG_EXC_P_MASK
#define QSPI_FCCR_STIG_MEM_EN_POS               		    	(2UL)
#define QSPI_FCCR_STIG_MEM_EN_MASK      				    	(0x1UL << QSPI_FCCR_STIG_MEM_EN_POS)
#define QSPI_FCCR_STIG_MEM_EN					       			QSPI_FCCR_STIG_MEM_EN_MASK
#define QSPI_FCCR_DUMMY_NUM_POS               		    		(7UL)
#define QSPI_FCCR_DUMMY_NUM_MASK      				    		(0x1FUL << QSPI_FCCR_DUMMY_NUM_POS)
#define QSPI_FCCR_DUMMY_NUM					       				QSPI_FCCR_DUMMY_NUM_MASK
#define QSPI_FCCR_W_BYTE_NUM_POS               		    		(12UL)
#define QSPI_FCCR_W_BYTE_NUM_MASK      				    		(0x7UL << QSPI_FCCR_W_BYTE_NUM_POS)
#define QSPI_FCCR_W_BYTE_NUM					       			QSPI_FCCR_W_BYTE_NUM_MASK
#define QSPI_FCCR_W_DATA_EN_POS               		    		(15UL)
#define QSPI_FCCR_W_DATA_EN_MASK      				    		(0x1UL << QSPI_FCCR_W_DATA_EN_POS)
#define QSPI_FCCR_W_DATA_EN					       				QSPI_FCCR_W_DATA_EN_MASK
#define QSPI_FCCR_ADR_BYTE_NUM_POS               		    	(16UL)
#define QSPI_FCCR_ADR_BYTE_NUM_MASK      				    	(0x3UL << QSPI_FCCR_ADR_BYTE_NUM_POS)
#define QSPI_FCCR_ADR_BYTE_NUM					       			QSPI_FCCR_ADR_BYTE_NUM_MASK
#define QSPI_FCCR_MODE_BIT_EN_POS               		    	(18UL)
#define QSPI_FCCR_MODE_BIT_EN_MASK      				    	(0x1UL << QSPI_FCCR_MODE_BIT_EN_POS)
#define QSPI_FCCR_MODE_BIT_EN					       			QSPI_FCCR_MODE_BIT_EN_MASK
#define QSPI_FCCR_ADR_EN_POS               		    			(19UL)
#define QSPI_FCCR_ADR_EN_MASK      				    			(0x1UL << QSPI_FCCR_ADR_EN_POS)
#define QSPI_FCCR_ADR_EN					       				QSPI_FCCR_ADR_EN_MASK
#define QSPI_FCCR_R_BYTE_NUM_POS               		    		(20UL)
#define QSPI_FCCR_R_BYTE_NUM_MASK      				    		(0x7UL << QSPI_FCCR_R_BYTE_NUM_POS)
#define QSPI_FCCR_R_BYTE_NUM					       			QSPI_FCCR_R_BYTE_NUM_MASK
#define QSPI_FCCR_R_DATA_EN_POS               		    		(23UL)
#define QSPI_FCCR_R_DATA_EN_MASK      				    		(0x1UL << QSPI_FCCR_R_DATA_EN_POS)
#define QSPI_FCCR_R_DATA_EN					       				QSPI_FCCR_R_DATA_EN_MASK
#define QSPI_FCCR_CMD_OPCODE_POS               		    		(24UL)
#define QSPI_FCCR_CMD_OPCODE_MASK      				    		(0xFFUL << QSPI_FCCR_CMD_OPCODE_POS)
#define QSPI_FCCR_CMD_OPCODE					       			QSPI_FCCR_CMD_OPCODE_MASK

/*Flash Command Address Register*/
#define QSPI_FCAR_CMD_ADR_POS               		    		(0UL)
#define QSPI_FCAR_CMD_ADR_MASK      				    		(0xFFFFFFFFUL)
#define QSPI_FCAR_CMD_ADR						       			QSPI_FCCR_EXC_CMD_MASK

/*Flash Command Read Data Register(Lower)*/
#define QSPI_FCRDR_CMD_ADR_L_POS               		    		(0UL)
#define QSPI_FCRDR_CMD_ADR_L_MASK      				    		(0xFFFFFFFFUL)
#define QSPI_FCRDR_CMD_ADR_L						       		QSPI_FCRDR_CMD_ADR_L_MASK

/*Flash Command Read Data  Register(Upper)*/
#define QSPI_FCRDR_CMD_ADR_H_POS               		    		(0UL)
#define QSPI_FCRDR_CMD_ADR_H_MASK      				    		(0xFFFFFFFFUL)
#define QSPI_FCRDR_CMD_ADR_H						       		QSPI_FCRDR_CMD_ADR_H_MASK


/*Flash Command Write Data Register(Lower)*/
#define QSPI_FCWDR_CMD_ADR_L_POS               		    		(0UL)
#define QSPI_FCWDR_CMD_ADR_L_MASK      				    		(0xFFFFFFFFUL)
#define QSPI_FCWDR_CMD_ADR_L						       		QSPI_FCWDR_CMD_ADR_L_MASK

/*Flash Command Write Data Register(Upper)*/
#define QSPI_FCWDR_CMD_ADR_H_POS               		    		(0UL)
#define QSPI_FCWDR_CMD_ADR_H_MASK      				    		(0xFFFFFFFFUL)
#define QSPI_FCWDR_CMD_ADR_H						       		QSPI_FCWDR_CMD_ADR_H_MASK

/*Polling Flash Status Register*/
#define QSPI_QFSR_FLASH_STATUS_POS               		    	(0UL)
#define QSPI_QFSR_FLASH_STATUS_MASK      				    	(0xFFUL << QSPI_FCCR_CMD_OPCODE_POS)
#define QSPI_QFSR_FLASH_STATUS						       		QSPI_QFSR_FLASH_STATUS_MASK
#define QSPI_QFSR_POLL_STATUS_POS               		    	(8UL)
#define QSPI_QFSR_POLL_STATUS_MASK      				    	(0x1UL << QSPI_QFSR_POLL_STATUS_POS)
#define QSPI_QFSR_POLL_STATUS						       		QSPI_QFSR_POLL_STATUS_MASK
#define QSPI_QFSR_DUMMY_NUM_POS               		    		(16UL)
#define QSPI_QFSR_DUMMY_NUM_MASK      				    		(0xFUL << QSPI_QFSR_DUMMY_NUM_POS)
#define QSPI_QFSR_DUMMY_NUM						       			QSPI_QFSR_DUMMY_NUM_MASK

/*Module ID Register*/
#define QSPI_MIR_CFG_ID_POS               		    			(0UL)
#define QSPI_MIR_CFG_ID_MASK      				    			(0x3UL << QSPI_MIR_CFG_ID_POS)
#define QSPI_MIR_CFG_ID						       				QSPI_MIR_CFG_ID_MASK
#define QSPI_MIR_REVISION_ID_POS               		    		(8UL)
#define QSPI_MIR_REVISION_ID_MASK      				    		(0xFFFFUL << QSPI_MIR_REVISION_ID_POS)
#define QSPI_MIR_REVISION_ID						       		QSPI_MIR_REVISION_ID_MASK
#define QSPI_MIR_FIX_NUM_ID_POS               		    		(24UL)
#define QSPI_MIR_FIX_NUM_ID_MASK      				    		(0xFFUL << QSPI_MIR_FIX_NUM_ID_POS)
#define QSPI_MIR_FIX_NUM_ID						       			QSPI_MIR_FIX_NUM_ID_MASK

#ifdef __cplusplus
}
#endif

#endif /* MS_QSPI_REGS_H_ */

