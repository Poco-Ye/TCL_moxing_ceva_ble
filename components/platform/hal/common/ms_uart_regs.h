/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_uart_regs.h
 * @brief
 * @author bingrui.chen
 * @date 2021-12-21
 * @version 1.0
 * @Revision
 */
#ifndef MS_UART_REGS_H_
#define MS_UART_REGS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup MS_REGISTER
 * @{
 */

/** @defgroup Uart_Type UART Registers definition
 * @{
 */
typedef struct {
	union {
		volatile uint32_t RBR;
		volatile uint32_t DLL;
		volatile uint32_t THR;
	} OFFSET_0;

	union {
		volatile uint32_t DLH;
		volatile uint32_t IER;
	} OFFSET_4;

	union {
		volatile uint32_t FCR;
		volatile uint32_t IIR;
	} OFFSET_8;

	volatile uint32_t LCR;
	volatile uint32_t MCR;
	volatile uint32_t LSR;
	volatile uint32_t MSR;
	volatile uint32_t SCR;
	volatile uint32_t LPDLL;
	volatile uint32_t LPDLH;
	volatile uint32_t RESERVED1;
	volatile uint32_t RESERVED2;
	union {
		volatile uint32_t SRBR[16];
		volatile uint32_t STHR[16];
	} OFFSET_48;
	volatile uint32_t FAR;
	volatile uint32_t TFR;
	volatile uint32_t RFW;
	volatile uint32_t USR;
	volatile uint32_t TFL;
	volatile uint32_t RFL;
	volatile uint32_t SRR;
	volatile uint32_t SRTS;
	volatile uint32_t SBCR;
	volatile uint32_t SDMAM;
	volatile uint32_t SFE;
	volatile uint32_t SRT;
	volatile uint32_t STET;
	volatile uint32_t HTX;
	volatile uint32_t DMASA;
	volatile uint32_t TCR;
	volatile uint32_t DE_EN;
	volatile uint32_t RE_EN;
	volatile uint32_t DET;
	volatile uint32_t TAT;
	volatile uint32_t DLF;
	volatile uint32_t RAR;
	volatile uint32_t TAR;
	volatile uint32_t LCR_EXT;
	volatile uint32_t UART_PROT_LEVEL;
	volatile uint32_t REG_TIMEOUT_RST;
} Uart_Type;

/**
 * @}
 */

/*Receive Buffer Register Bit Define*/
#define UART_RBR_RBR_POS               							(0UL)
#define UART_RBR_RBR_MASK      									(0x1FFUL << UART_RBR_RBR_POS)
#define UART_RBR_RBR											UART_RBR_RBR_MASK

/*Divisor Latch (Low)*/
#define UART_DLL_DLL_POS               							(0UL)
#define UART_DLL_DLL_MASK      									(0xFFUL << UART_DLL_DLL_POS)
#define UART_DLL_DLL											UART_DLL_DLL_MASK

/*Transmit Holding Register Bit Define*/
#define UART_THR_THR_POS               							(0UL)
#define UART_THR_THR_MASK      									(0x1FFUL << UART_THR_THR_POS)
#define UART_THR_THR											UART_THR_THR_MASK

/*Divisor Latch High*/
#define UART_DLH_DLH_POS               							(0UL)
#define UART_DLH_DLH_MASK      									(0xFFUL << UART_DLH_DLH_POS)
#define UART_DLH_DLH											UART_DLH_DLH_MASK

/*Interrupt Enable Register Bit Define*/
#define UART_IER_ERBFI_POS               						(0UL)
#define UART_IER_ERBFI_MASK      								(0x1UL << UART_IER_ERBFI_POS)
#define UART_IER_ERBFI											UART_IER_ERBFI_MASK
#define UART_IER_ETBEI_POS               						(1UL)
#define UART_IER_ETBEI_MASK      								(0x1UL << UART_IER_ETBEI_POS)
#define UART_IER_ETBEI											UART_IER_ETBEI_MASK
#define UART_IER_ELSI_POS               						(2UL)
#define UART_IER_ELSI_MASK      								(0x1UL << UART_IER_ELSI_POS)
#define UART_IER_ELSI											UART_IER_ELSI_MASK
#define UART_IER_EDSSI_POS               						(3UL)
#define UART_IER_EDSSI_MASK      								(0x1UL << UART_IER_EDSSI_POS)
#define UART_IER_EDSSI											UART_IER_EDSSI_MASK
#define UART_IER_ELCOLR_POS               						(4UL)
#define UART_IER_ELCOLR_MASK      								(0x1UL << UART_IER_ELCOLR_POS)
#define UART_IER_ELCOLR											UART_IER_ELCOLR_MASK
#define UART_IER_PTIME_POS               						(7UL)
#define UART_IER_PTIME_MASK      								(0x1UL << UART_IER_PTIME_POS)
#define UART_IER_PTIME											UART_IER_PTIME_MASK

/*FIFO Control Register Bit Define*/
#define UART_FCR_FIFOE_POS               						(0UL)
#define UART_FCR_FIFOE_MASK      								(0x1UL << UART_FCR_FIFOE_POS)
#define UART_FCR_FIFOE											UART_FCR_FIFOE_MASK
#define UART_FCR_RFIFOR_POS               						(1UL)
#define UART_FCR_RFIFOR_MASK      								(0x1UL << UART_FCR_RFIFOR_POS)
#define UART_FCR_RFIFOR											UART_FCR_RFIFOR_MASK
#define UART_FCR_XFIFOR_POS               						(2UL)
#define UART_FCR_XFIFOR_MASK      								(0x1UL << UART_FCR_XFIFOR_POS)
#define UART_FCR_XFIFOR											UART_FCR_XFIFOR_MASK
#define UART_FCR_DMAM_POS               						(3UL)
#define UART_FCR_DMAM_MASK      								(0x1UL << UART_FCR_DMAM_POS)
#define UART_FCR_DMAM											UART_FCR_DMAM_MASK
#define UART_FCR_TET_POS               							(4UL)
#define UART_FCR_TET_MASK      									(0x3UL << UART_FCR_TET_POS)
#define UART_FCR_TET											UART_FCR_TET_MASK
#define UART_FCR_RT_POS               							(6UL)
#define UART_FCR_RT_MASK      									(0x3UL << UART_FCR_RT_POS)
#define UART_FCR_RT												UART_FCR_RT_MASK

/*Interrupt Identification Register Bit Define*/
#define UART_IIR_IID_POS                             			(0UL)
#define UART_IIR_IID_MASK                            			(0xFUL << UART_IIR_IID_POS)
#define UART_IIR_IID                                 			UART_IIR_IID_MASK
#define UART_IIR_FIFOSE_POS                          			(6UL)
#define UART_IIR_FIFOSE_MASK                         			(0x3UL << UART_IIR_FIFOSE_POS)
#define UART_IIR_FIFOSE                              			UART_IIR_FIFOSE_MASK

/*Line Control Register Bit Define*/
#define UART_LCR_DLS_POS                            			(0UL)
#define UART_LCR_DLS_MASK                           			(0x3UL << UART_LCR_DLS_POS)
#define UART_LCR_DLS                                			UART_LCR_DLS_MASK
#define UART_LCR_STOP_POS                           			(2UL)
#define UART_LCR_STOP_MASK                          			(0x1UL << UART_LCR_STOP_POS)
#define UART_LCR_STOP                               			UART_LCR_STOP_MASK
#define UART_LCR_PEN_POS                            			(3UL)
#define UART_LCR_PEN_MASK                           			(0x1UL << UART_LCR_PEN_POS)
#define UART_LCR_PEN                                			UART_LCR_PEN_MASK
#define UART_LCR_EPS_POS                            			(4UL)
#define UART_LCR_EPS_MASK                           			(0x1UL << UART_LCR_EPS_POS)
#define UART_LCR_EPS                                			UART_LCR_EPS_MASK
#define UART_LCR_SP_POS                             			(5UL)
#define UART_LCR_SP_MASK                            			(0x1UL << UART_LCR_SP_POS)
#define UART_LCR_SP                                 			UART_LCR_SP_MASK
#define UART_LCR_BC_POS                             			(6UL)
#define UART_LCR_BC_MASK                            			(0x1UL << UART_LCR_BC_POS)
#define UART_LCR_BC                                 			UART_LCR_BC_MASK
#define UART_LCR_DLAB_POS                           			(7UL)
#define UART_LCR_DLAB_MASK                          			(0x1UL << UART_LCR_DLAB_POS)
#define UART_LCR_DLAB                               			UART_LCR_DLAB_MASK

/*Modem Control Register Bit Define*/
#define UART_MCR_DTR_POS                            			(0UL)
#define UART_MCR_DTR_MASK                           			(0x1UL << UART_MCR_DTR_POS)
#define UART_MCR_DTR                                			UART_MCR_DTR_MASK
#define UART_MCR_RTS_POS                            			(1UL)
#define UART_MCR_RTS_MASK                           			(0x1UL << UART_MCR_RTS_POS)
#define UART_MCR_RTS                                			UART_MCR_RTS_MASK
#define UART_MCR_OUT1_POS                           			(2UL)
#define UART_MCR_OUT1_MASK                          			(0x1UL << UART_MCR_OUT1_POS)
#define UART_MCR_OUT1                               			UART_MCR_OUT1_MASK
#define UART_MCR_OUT2_POS                           			(3UL)
#define UART_MCR_OUT2_MASK                          			(0x1UL << UART_MCR_OUT2_POS)
#define UART_MCR_OUT2                               			UART_MCR_OUT2_MASK
#define UART_MCR_LoopBack_POS                       			(4UL)
#define UART_MCR_LoopBack_MASK                      			(0x1UL << UART_MCR_LoopBack_POS)
#define UART_MCR_LoopBack                           			UART_MCR_LoopBack_MASK
#define UART_MCR_AFCE_POS                           			(5UL)
#define UART_MCR_AFCE_MASK                          			(0x1UL << UART_MCR_AFCE_POS)
#define UART_MCR_AFCE                               			UART_MCR_AFCE_MASK
#define UART_MCR_SIRE_POS                           			(6UL)
#define UART_MCR_SIRE_MASK                          			(0x1UL << UART_MCR_SIRE_POS)
#define UART_MCR_SIRE                               			UART_MCR_SIRE_MASK

/*Line Status Register Bit Define*/
#define UART_LSR_DR_POS                           				(0UL)
#define UART_LSR_DR_MASK                          				(0x1UL << UART_LSR_DR_POS)
#define UART_LSR_DR                               				UART_LSR_DR_MASK
#define UART_LSR_OE_POS                           				(1UL)
#define UART_LSR_OE_MASK                          				(0x1UL << UART_LSR_OE_POS)
#define UART_LSR_OE                               				UART_LSR_OE_MASK
#define UART_LSR_PE_POS                           				(2UL)
#define UART_LSR_PE_MASK                          				(0x1UL << UART_LSR_PE_POS)
#define UART_LSR_PE                               				UART_LSR_PE_MASK
#define UART_LSR_FE_POS                           				(3UL)
#define UART_LSR_FE_MASK                          				(0x1UL << UART_LSR_FE_POS)
#define UART_LSR_FE                               				UART_LSR_FE_MASK
#define UART_LSR_BI_POS                           				(4UL)
#define UART_LSR_BI_MASK                          				(0x1UL << UART_LSR_BI_POS)
#define UART_LSR_BI                               				UART_LSR_BI_MASK
#define UART_LSR_THRE_POS                           			(5UL)
#define UART_LSR_THRE_MASK                          			(0x1UL << UART_LSR_THRE_POS)
#define UART_LSR_THRE                               			UART_LSR_THRE_MASK
#define UART_LSR_TEMT_POS                           			(6UL)
#define UART_LSR_TEMT_MASK                          			(0x1UL << UART_LSR_TEMT_POS)
#define UART_LSR_TEMT                               			UART_LSR_TEMT_MASK
#define UART_LSR_RFE_POS                           				(7UL)
#define UART_LSR_RFE_MASK                          				(0x1UL << UART_LSR_RFE_POS)
#define UART_LSR_RFE                               				UART_LSR_RFE_MASK
#define UART_LSR_ADDR_RCVD_POS                           		(7UL)
#define UART_LSR_ADDR_RCVD_MASK                          		(0x1UL << UART_LSR_ADDR_RCVD_POS)
#define UART_LSR_ADDR_RCVD                               		UART_LSR_ADDR_RCVD_MASK

/*Modem Status Register Bit Define*/
#define UART_MLR_DCTS_POS                           			(0UL)
#define UART_MLR_DCTS_MASK                          			(0x1UL << UART_MLR_DCTS_POS)
#define UART_MLR_DCTS                               			UART_MLR_DCTS_MASK
#define UART_MLR_DDSR_POS                           			(1UL)
#define UART_MLR_DDSR_MASK                          			(0x1UL << UART_MLR_DDSR_POS)
#define UART_MLR_DDSR                               			UART_MLR_DDSR_MASK
#define UART_MLR_TERI_POS                           			(2UL)
#define UART_MLR_TERI_MASK                          			(0x1UL << UART_MLR_TERI_POS)
#define UART_MLR_TERI                               			UART_MLR_TERI_MASK
#define UART_MLR_DDCD_POS                           			(3UL)
#define UART_MLR_DDCD_MASK                          			(0x1UL << UART_MLR_DDCD_POS)
#define UART_MLR_DDCD                               			UART_MLR_DDCD_MASK
#define UART_MLR_CTS_POS                           				(4UL)
#define UART_MLR_CTS_MASK                          				(0x1UL << UART_MLR_CTS_POS)
#define UART_MLR_CTS                               				UART_MLR_CTS_MASK
#define UART_MLR_DSR_POS                           				(5UL)
#define UART_MLR_DSR_MASK                          				(0x1UL << UART_MLR_DSR_POS)
#define UART_MLR_DSR                               				UART_MLR_DSR_MASK
#define UART_MLR_RI_POS                           				(6UL)
#define UART_MLR_RI_MASK                          				(0x1UL << UART_MLR_RI_POS)
#define UART_MLR_RI                               				UART_MLR_RI_MASK
#define UART_MLR_DCD_POS                           				(7UL)
#define UART_MLR_DCD_MASK                          				(0x1UL << UART_MLR_DCD_POS)
#define UART_MLR_DCD                               				UART_MLR_DCD_MASK

/*Scratchpad Register Bit Define*/
#define UART_SCR_SCR_POS                           				(0UL)
#define UART_SCR_SCR_MASK                          				(0xFFUL << UART_SCR_SCR_POS)
#define UART_SCR_SCR                               				UART_SCR_SCR_MASK

/*Low Power Divisor Latch Low*/
#define UART_LPDLL_LPDLL_POS                           			(0UL)
#define UART_LPDLL_LPDLL_MASK                          			(0xFFUL << UART_LPDLL_LPDLL_POS)
#define UART_LPDLL_LPDLL                               			UART_LPDLL_LPDLL_MASK

/*Low Power Divisor Latch High*/
#define UART_LPDLH_LPDLH_POS                           			(0UL)
#define UART_LPDLH_LPDLH_MASK                          			(0xFFUL << UART_LPDLH_LPDLH_POS)
#define UART_LPDLH_LPDLH                               			UART_LPDLH_LPDLH_MASK

/*Shadow Receive Buffer Register Bit Define*/
#define UART_SRBRn_SRBRn_POS                           			(0UL)
#define UART_SRBRn_SRBRn_MASK                          			(0x1FFUL << UART_SRBRn_SRBRn_POS)
#define UART_SRBRn_SRBRn                               			UART_SRBRn_SRBRn_MASK

/*Shadow Transmit Holding Register Bit Define*/
#define UART_STHRn_STHRn_POS                           			(0UL)
#define UART_STHRn_STHRn_MASK                          			(0x1FFUL << UART_STHRn_STHRn_POS)
#define UART_STHRn_STHRn                               			UART_STHRn_STHRn_MASK

/*FIFO Access Register Bit Define*/
#define UART_FAR_FAR_POS                                		(0UL)
#define UART_FAR_FAR_MASK                               		(0x1UL << UART_FAR_FAR_POS)
#define UART_FAR_FAR                                    		UART_FAR_FAR_MASK

/*Transmit FIFO Read Bit Define*/
#define UART_TFR_TFR_POS                                		(0UL)
#define UART_TFR_TFR_MASK                               		(0xFFUL << UART_TFR_TFR_POS)
#define UART_TFR_TFR                                    		UART_TFR_TFR_MASK

/*Receive FIFO Write Bit Define*/
#define UART_RFW_RFWD_POS                                		(0UL)
#define UART_RFW_RFWD_MASK                               		(0xFFUL << UART_RFW_RFWD_POS)
#define UART_RFW_RFWD                                    		UART_RFW_RFWD_MASK
#define UART_RFW_RFPE_POS                                		(8UL)
#define UART_RFW_RFPE_MASK                               		(0x1UL << UART_RFW_RFPE_POS)
#define UART_RFW_RFPE                                    		UART_RFW_RFPE_MASK
#define UART_RFW_RFFE_POS                                		(9UL)
#define UART_RFW_RFFE_MASK                               		(0x1UL << UART_RFW_RFFE_POS)
#define UART_RFW_RFFE                                    		UART_RFW_RFFE_MASK

/* UART Status Register Bit Define*/
#define UART_USR_BUSY_POS                               		(0UL)
#define UART_USR_BUSY_MASK                              		(0x1UL << UART_USR_BUSY_POS)
#define UART_USR_BUSY                                   		UART_USR_BUSY_MASK
#define UART_USR_TFNF_POS                               		(1UL)
#define UART_USR_TFNF_MASK                              		(0x1UL << UART_USR_TFNF_POS)
#define UART_USR_TFNF                                   		UART_USR_TFNF_MASK
#define UART_USR_TFE_POS                                		(2UL)
#define UART_USR_TFE_MASK                               		(0x1UL << UART_USR_TFE_POS)
#define UART_USR_TFE                                    		UART_USR_TFE_MASK
#define UART_USR_RFNE_POS                               		(3UL)
#define UART_USR_RFNE_MASK                              		(0x1UL << UART_USR_RFNE_POS)
#define UART_USR_RFNE                                   		UART_USR_RFNE_MASK
#define UART_USR_RFF_POS                                		(4UL)
#define UART_USR_RFF_MASK                               		(0x1UL << UART_USR_RFF_POS)
#define UART_USR_RFF                                    		UART_USR_RFF_MASK

/* Software Reset Register Bit Define*/
#define UART_SRR_UR_POS                                			(0UL)
#define UART_SRR_UR_MASK                               			(0x1UL << UART_SRR_UR_POS)
#define UART_SRR_UR                                    			UART_SRR_UR_MASK
#define UART_SRR_RFR_POS                                		(1UL)
#define UART_SRR_RFR_MASK                               		(0x1UL << UART_SRR_RFR_POS)
#define UART_SRR_RFR                                    		UART_SRR_RFR_MASK
#define UART_SRR_XFR_POS                                		(2UL)
#define UART_SRR_XFR_MASK                               		(0x1UL << UART_SRR_XFR_POS)
#define UART_SRR_XFR                                    		UART_SRR_XFR_MASK

/* Shadow Request to Send Bit Define*/
#define UART_SRTS_SRTS_POS                                		(0UL)
#define UART_SRTS_SRTS_MASK                               		(0x1UL << UART_SRTS_SRTS_POS)
#define UART_SRTS_SRTS                                    		UART_SRTS_SRTS_MASK

/* Shadow Break Control Register Bit Define*/
#define UART_SBCR_SBCR_POS                                		(0UL)
#define UART_SBCR_SBCR_MASK                               		(0x1UL << UART_SBCR_SBCR_POS)
#define UART_SBCR_SBCR                                    		UART_SBCR_SBCR_MASK

/* Shadow DMA Mode Register Bit Define*/
#define UART_SDMAM_SDMAM_POS                                	(0UL)
#define UART_SDMAM_SDMAM_MASK                               	(0x1UL << UART_SDMAM_SDMAM_POS)
#define UART_SDMAM_SDMAM                                    	UART_SDMAM_SDMAM_MASK

/*  Shadow FIFO Enable Register Bit Define*/
#define UART_SFE_SFE_POS                                		(0UL)
#define UART_SFE_SFE_MASK                               		(0x1UL << UART_SFE_SFE_POS)
#define UART_SFE_SFE                                    		UART_SFE_SFE_MASK

/*  Shadow RCVR Trigger Register Bit Define*/
#define UART_SRT_SRT_POS                                		(0UL)
#define UART_SRT_SRT_MASK                               		(0x3UL << UART_SRT_SRT_POS)
#define UART_SRT_SRT                                    		UART_SRT_SRT_MASK

/*  Shadow TX Empty Trigger Register Bit Define*/
#define UART_STET_STET_POS                                		(0UL)
#define UART_STET_STET_MASK                               		(0x3UL << UART_STET_STET_POS)
#define UART_STET_STET                                    		UART_STET_STET_MASK

/*  Halt TX Bit Define*/
#define UART_HTX_HTX_POS                                		(0UL)
#define UART_HTX_HTX_MASK                               		(0x1UL << UART_HTX_HTX_POS)
#define UART_HTX_HTX                                    		UART_HTX_HTX_MASK

/*  DMA Software Acknowledge Register Bit Define*/
#define UART_DMASA_DMASA_POS                                	(0UL)
#define UART_DMASA_DMASA_MASK                               	(0x1UL << UART_DMASA_DMASA_POS)
#define UART_DMASA_DMASA                                    	UART_DMASA_DMASA_MASK

/*Transceiver Control Register Bit Define*/
#define UART_TCR_RS485_EN_POS                                	(0UL)
#define UART_TCR_RS485_EN_MASK                               	(0x1UL << UART_TCR_RS485_EN_POS)
#define UART_TCR_RS485_EN                                    	UART_TCR_RS485_EN_MASK
#define UART_TCR_RE_POL_POS                                		(1UL)
#define UART_TCR_RE_POL_MASK                               		(0x1UL << UART_TCR_RE_POL_POS)
#define UART_TCR_RE_POL                                    		UART_TCR_RE_POL_MASK
#define UART_TCR_DE_POL_POS                                		(2UL)
#define UART_TCR_DE_POL_MASK                               		(0x1UL << UART_TCR_DE_POL_POS)
#define UART_TCR_DE_POL                                    		UART_TCR_DE_POL_MASK
#define UART_TCR_XFER_MODE_POS                                	(3UL)
#define UART_TCR_XFER_MODE_MASK                               	(0x3UL << UART_TCR_XFER_MODE_POS)
#define UART_TCR_XFER_MODE                                    	UART_TCR_XFER_MODE_MASK

/*Driver Output Enable Register Bit Define*/
#define UART_DE_EN_DE_Enable_POS                                (0UL)
#define UART_DE_EN_DE_Enable_MASK                               (0x1UL << UART_DE_EN_DE_Enable_POS)
#define UART_DE_EN_DE_Enable                                    UART_DE_EN_DE_Enable_MASK

/*Receiver Output Enable Register Bit Define*/
#define UART_RE_EN_RE_Enable_POS                                (0UL)
#define UART_RE_EN_RE_Enable_MASK                               (0x1UL << UART_RE_EN_RE_Enable_POS)
#define UART_RE_EN_RE_Enable                                    UART_RE_EN_RE_Enable_MASK

/*Driver Output Enable Timing Register Bit Define*/
#define UART_DET_DE_Assertion_Time_POS                          (0UL)
#define UART_DET_DE_Assertion_Time_MASK                         (0xFFUL << UART_DET_DE_Assertion_Time_POS)
#define UART_DET_DE_Assertion_Time                              UART_DET_DE_Assertion_Time_MASK
#define UART_DET_DE_Deassertion_Time_POS                        (16UL)
#define UART_DET_DE_Deassertion_Time_MASK                       (0xFFUL << UART_DET_DE_Deassertion_Time_POS)
#define UART_DET_DE_Deassertion_Time                            UART_DET_DE_Deassertion_Time_MASK

/*/*Driver Output Enable Timing Register Bit Define*/
#define UART_TAT_DE_to_RE_POS                                	(0UL)
#define UART_TAT_DE_to_RE_MASK                               	(0xFFFFUL << UART_TAT_DE_to_RE_POS)
#define UART_TAT_DE_to_RE                                    	UART_TAT_DE_to_RE_MASK
#define UART_TAT_RE_to_DE_POS                                	(16UL)
#define UART_TAT_RE_to_DE_MASK                               	(0xFFUL << UART_TAT_RE_to_DE_POS)
#define UART_TAT_RE_to_DE                                    	UART_TAT_RE_to_DE_MASK

/*Receive Address Register Bit Define*/
#define UART_RAR_RAR_POS                                		(0UL)
#define UART_RAR_RAR_MASK                               		(0xFFUL << UART_RAR_RAR_POS)
#define UART_RAR_RAR                                    		UART_RAR_RAR_MASK

/*Transmit Address Register Bit Define*/
#define UART_TAR_TAR_POS                                		(0UL)
#define UART_TAR_TAR_MASK                               		(0xFFUL << UART_TAR_TAR_POS)
#define UART_TAR_TAR                                    		UART_TAR_TAR_MASK

/*Line Extended Control Register Bit Define*/
#define UART_LCR_EXT_DLS_E_POS                                	(0UL)
#define UART_LCR_EXT_DLS_E_MASK                               	(0x1UL << UART_LCR_EXT_DLS_E_POS)
#define UART_LCR_EXT_DLS_E                                    	UART_LCR_EXT_DLS_E_MASK
#define UART_LCR_EXT_ADDR_MATCH_POS                           	(1UL)
#define UART_LCR_EXT_ADDR_MATCH_MASK                          	(0x1UL << UART_LCR_EXT_ADDR_MATCH_POS)
#define UART_LCR_EXT_ADDR_MATCH                               	UART_LCR_EXT_ADDR_MATCH_MASK
#define UART_LCR_EXT_SEND_ADDR_POS                            	(2UL)
#define UART_LCR_EXT_SEND_ADDR_MASK                           	(0x1UL << UART_LCR_EXT_SEND_ADDR_POS)
#define UART_LCR_EXT_SEND_ADDR                                	UART_LCR_EXT_SEND_ADDR_MASK
#define UART_LCR_EXT_TRANSMIT_MODE_POS                        	(3UL)
#define UART_LCR_EXT_TRANSMIT_MODE_MASK                       	(0x1UL << UART_LCR_EXT_TRANSMIT_MODE_POS)
#define UART_LCR_EXT_TRANSMIT_MODE                            	UART_LCR_EXT_TRANSMIT_MODE_MASK

/*UART Protection level Bit Define*/
#define UART_UART_PROT_LEVEL_UART_PROT_LEVEL_POS              	(0UL)
#define UART_UART_PROT_LEVEL_UART_PROT_LEVEL_MASK             	(0x7UL << UART_UART_PROT_LEVEL_UART_PROT_LEVEL_POS)
#define UART_UART_PROT_LEVEL_UART_PROT_LEVEL                  	UART_UART_PROT_LEVEL_UART_PROT_LEVEL_MASK

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* MS_UART_REGS_H_ */
