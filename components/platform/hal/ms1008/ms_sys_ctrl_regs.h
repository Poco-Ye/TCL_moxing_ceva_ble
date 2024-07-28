/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_ms_clock_hal_regs.h
 * @brief
 * @author bingrui.chen
 * @date 2021-12-16
 * @version 1.0
 * @Revision
 */
#ifndef MS_SYS_CTRL_REGS_H_
#define MS_SYS_CTRL_REGS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup MS_REGISTER
  * @{
  */


/** @defgroup SYS_CTRL System Control Register definition
  * @{
  */

typedef struct
{
    volatile uint32_t CLK_SEL;                    /*! 0x00           */
    volatile uint32_t CLK_EN0;                    /*! 0x04           */
    volatile uint32_t CLK_EN1;                    /*! 0x08           */
    volatile uint32_t CLK_DIV0;                   /*! 0x0C           */
    volatile uint32_t CLK_DIV1;                   /*! 0x10           */
    volatile uint32_t CLK_DIV2;                   /*! 0x14           */
    volatile uint32_t CLK_DIV3;                   /*! 0x18           */
    volatile uint32_t GPIO_MODE0;                 /*! 0x1C           */
    volatile uint32_t GPIO_MODE1;                 /*! 0x20           */
    volatile uint32_t GPIO_MODE2;                 /*! 0x24           */
    volatile uint32_t GPIO_MODE3;                 /*! 0x28           */
    volatile uint32_t OTA_SRC_ADDR;               /*! 0x2C           */
    volatile uint32_t OTA_DST_ADDR;               /*! 0x30           */
    volatile uint32_t OTA_OFT_ADDR;               /*! 0x34           */
    volatile uint32_t REMAP_CTRL;                 /*! 0x38           */
    volatile uint32_t CALI_CTL;                   /*! 0x3C  reserved */
    volatile uint32_t CALI_CNT_LEN;               /*! 0x40  reserved */
    volatile uint32_t CALI_CNT_RES;               /*! 0x44  reserved */
    volatile uint32_t CALI_STATUS;                /*! 0x48  reserved */
    volatile uint32_t PLL_CTRL;                   /*! 0x4C           */
    volatile uint32_t PRE_PWR_MODE;               /*! 0x50           */
    volatile uint32_t PWR_EN;                     /*! 0x54           */
    volatile uint32_t BOOT_SEL;                   /*! 0x58           */
    volatile uint32_t EFUSE_CTRL;                 /*! 0x5C           */
    volatile uint32_t EFUSE_ADDR;                 /*! 0x60           */
    volatile uint32_t EFUSE_AEN_LEN;              /*! 0x64           */
    volatile uint32_t EFUSE_PWR_CTRL;             /*! 0x68           */
    volatile uint32_t EFUSE_PWR_LEN;              /*! 0x6C           */
    volatile uint32_t EFUSE_KEY;                  /*! 0x70           */
    volatile uint32_t EFUSE_RDATA;                /*! 0x74           */
    volatile uint32_t EFUSE_LOAD_EN;              /*! 0x78           */
    volatile uint32_t DIV_TOG;                    /*! 0x7C           */
    volatile uint32_t RAM_CTRL;                   /*! 0x80           */
    volatile uint32_t PAD_PE;                     /*! 0x84           */
    volatile uint32_t OSC_CFG;                    /*! 0x88  reserved */
    volatile uint32_t SOFT_FLAG;                  /*! 0x8C           */
    volatile uint32_t SOFT_RST;                   /*! 0x90           */
    volatile uint32_t LP_KEEP;                    /*! 0x94           */
    volatile uint32_t CLK_DIV4;                   /*! 0x98           */
    volatile uint32_t CLK_DIV5;                   /*! 0x9C           */
    volatile uint32_t DFX_SEL;                    /*! 0xA0           */
    volatile uint32_t WKUP_CSR;                   /*! 0xA4           */
    volatile uint32_t PUP_DLY_SR;                 /*! 0xA8           */
    volatile uint32_t PAD_PS;                     /*! 0xAC           */
    volatile uint32_t UNIT_EN;                    /*! 0xB0           */
    volatile uint32_t CPU_RESET_VECTOR;           /*! 0xB4           */
    volatile uint32_t BLE_CTRL;                   /*! 0xB8           */
    volatile uint32_t CLK_DIV6;                   /*! 0xBC           */
    volatile uint32_t CPU_DBG;                    /*! 0xC0           */
    volatile uint32_t PLL_CTRL1;                  /*! 0xC4  reserved */
    volatile uint32_t OSC16M_CFG1;                /*! 0xC8  reserved */
    volatile uint32_t OSC16M_CFG2;                /*! 0xCC  reserved */
    volatile uint32_t PAD_IS;                     /*! 0xD0           */
    volatile uint32_t PAD_DR0;                    /*! 0xD4           */
    volatile uint32_t PAD_DR1;                    /*! 0xD8           */
    volatile uint32_t PAD_HV_CTL;                 /*! 0xDC  reserved */
    volatile uint32_t EXT_OSC24M_CFG;             /*! 0xE0           */
    volatile uint32_t PWR_TRIM;                   /*! 0xE4  reserved */
    volatile uint32_t AUD_ADC_CFG0;               /*! 0xE8  reserved */
    volatile uint32_t AUD_ADC_CFG1;               /*! 0xEC  reserved */
    volatile uint32_t AUD_ADC_CFG2;               /*! 0xF0           */
    volatile uint32_t AUD_ADC_RDATA;              /*! 0xF4           */
    volatile uint32_t RC_CLK_STA;                 /*! 0xF8           */
    volatile uint32_t RF_TEST_CFG;                /*! 0xFC           */
} SysCtrlRegs_Type;


/**
 *@brief  SYS_CTRL_CLK_SEL System Clock Select Register Bits Define
 */

#define SYS_CTRL_CLK_SEL_HF_OSC_CLK_POS         	(0UL)
#define SYS_CTRL_CLK_SEL_HF_OSC_CLK_MASK    		(1UL << SYS_CTRL_CLK_SEL_HF_OSC_CLK_POS)
#define SYS_CTRL_CLK_SEL_HF_OSC_CLK					SYS_CTRL_CLK_SEL_HF_OSC_CLK_MASK
#define SYS_CTRL_CLK_SEL_LF_CLK_POS         	    (1UL)
#define SYS_CTRL_CLK_SEL_LF_CLK_MASK    		    (1UL << SYS_CTRL_CLK_SEL_LF_CLK_POS)
#define SYS_CTRL_CLK_SEL_LF_CLK					    SYS_CTRL_CLK_SEL_LF_CLK_MASK
#define SYS_CTRL_CLK_SEL_SYS_CLK_POS              	(2UL) 
#define SYS_CTRL_CLK_SEL_SYS_CLK_MASK    			(1UL << SYS_CTRL_CLK_SEL_SYS_CLK_POS)
#define SYS_CTRL_CLK_SEL_SYS_CLK					SYS_CTRL_CLK_SEL_SYS_CLK_MASK
#define SYS_CTRL_CLK_SEL_GPIO_FCLK_POS          	(3UL) 
#define SYS_CTRL_CLK_SEL_GPIO_FCLK_MASK    			(1UL << SYS_CTRL_CLK_SEL_GPIO_FCLK_POS)
#define SYS_CTRL_CLK_SEL_GPIO_FCLK					SYS_CTRL_CLK_SEL_GPIO_FCLK_MASK
#define SYS_CTRL_CLK_SEL_RTC_FPCLK_POS          	(4UL) 
#define SYS_CTRL_CLK_SEL_RTC_FPCLK_MASK    			(1UL << SYS_CTRL_CLK_SEL_GPIO_FCLK_POS)
#define SYS_CTRL_CLK_SEL_RTC_FPCLK					SYS_CTRL_CLK_SEL_GPIO_FCLK_MASK
#define SYS_CTRL_CLK_SEL_UART2_PCLK_POS         	(5UL) 
#define SYS_CTRL_CLK_SEL_UART2_PCLK_MASK    		(1UL << SYS_CTRL_CLK_SEL_UART2_PCLK_POS)
#define SYS_CTRL_CLK_SEL_UART2_PCLK					SYS_CTRL_CLK_SEL_UART2_PCLK_MASK
#define SYS_CTRL_CLK_SEL_HF_CLK_POS            		(6UL) 
#define SYS_CTRL_CLK_SEL_HF_CLK_MASK    			(1UL << SYS_CTRL_CLK_SEL_HF_CLK_POS)
#define SYS_CTRL_CLK_SEL_HF_CLK						SYS_CTRL_CLK_SEL_HF_CLK_MASK


/**
 *@brief  SYS_CTRL_CLK_EN0 BIT DEFINE
 */
#define SYS_CTRL_CLK_EN0_OFFSET                		0x04
#define SYS_CTRL_CLK_EN0_I2S0_POS    			    (0UL)
#define SYS_CTRL_CLK_EN0_I2S0_MASK    			    (1UL << SYS_CTRL_CLK_EN0_I2S0_POS)
#define SYS_CTRL_CLK_EN0_I2S0						SYS_CTRL_CLK_EN0_I2S0_MASK
#define SYS_CTRL_CLK_EN0_I2S1_POS                   (1UL)
#define SYS_CTRL_CLK_EN0_I2S1_MASK    			    (1UL << SYS_CTRL_CLK_EN0_I2S1_POS)
#define SYS_CTRL_CLK_EN0_I2S1						SYS_CTRL_CLK_EN0_I2S1_MASK
//#define SYS_CTRL_CLK_EN0_IR_SAMPLE_POS			    (2UL)
//#define SYS_CTRL_CLK_EN0_IR_SAMPLE_MASK    			(1UL << SYS_CTRL_CLK_EN0_IR_SAMPLE_POS)
//#define SYS_CTRL_CLK_EN0_IR_SAMPLE					SYS_CTRL_CLK_EN0_IR_SAMPLE_MASK
#define SYS_CTRL_CLK_EN0_UART2_POS                  (3UL)
#define SYS_CTRL_CLK_EN0_UART2_MASK    				(1UL << SYS_CTRL_CLK_EN0_UART2_POS)
#define SYS_CTRL_CLK_EN0_UART2						SYS_CTRL_CLK_EN0_UART2_MASK
#define SYS_CTRL_CLK_EN0_GPIO_RC_POS                (4UL)
#define SYS_CTRL_CLK_EN0_GPIO_RC_MASK    			(1UL << SYS_CTRL_CLK_EN0_GPIO_RC_POS)
#define SYS_CTRL_CLK_EN0_GPIO_RC					SYS_CTRL_CLK_EN0_GPIO_RC_MASK
#define SYS_CTRL_CLK_EN0_RTC_POS                    (5UL)
#define SYS_CTRL_CLK_EN0_RTC_MASK    				(1UL << SYS_CTRL_CLK_EN0_RTC_POS)
#define SYS_CTRL_CLK_EN0_RTC						SYS_CTRL_CLK_EN0_RTC_MASK
#define SYS_CTRL_CLK_EN0_BLE_32K_POS                (6UL)
#define SYS_CTRL_CLK_EN0_BLE_32K_MASK    			(1UL << SYS_CTRL_CLK_EN0_BLE_32K_POS)
#define SYS_CTRL_CLK_EN0_BLE_32K					SYS_CTRL_CLK_EN0_BLE_32K_MASK
//#define SYS_CTRL_CLK_EN0_CPU_DBG_POS                (7UL)
//#define SYS_CTRL_CLK_EN0_CPU_DBG_MASK    			(1UL << SYS_CTRL_CLK_EN0_CPU_DBG_POS)
//#define SYS_CTRL_CLK_EN0_CPU_DBG					SYS_CTRL_CLK_EN0_CPU_DBG_MASK
#define SYS_CTRL_CLK_EN0_HF_CLK_POS                 (8UL)
#define SYS_CTRL_CLK_EN0_HF_CLK_MASK    		    (1UL << SYS_CTRL_CLK_EN0_HF_CLK_POS)
#define SYS_CTRL_CLK_EN0_HF_CLK				        SYS_CTRL_CLK_EN0_HF_CLK_MASK
#define SYS_CTRL_CLK_EN0_LF_CLK_POS                 (9UL)
#define SYS_CTRL_CLK_EN0_LF_CLK_MASK    		    (1UL << SYS_CTRL_CLK_EN0_LF_CLK_POS)
#define SYS_CTRL_CLK_EN0_LF_CLK				        SYS_CTRL_CLK_EN0_LF_CLK_MASK
#define SYS_CTRL_CLK_EN0_DMA_POS                    (10UL)
#define SYS_CTRL_CLK_EN0_DMA_MASK    				(1UL << SYS_CTRL_CLK_EN0_DMA_POS)
#define SYS_CTRL_CLK_EN0_DMA						SYS_CTRL_CLK_EN0_DMA_MASK
#define SYS_CTRL_CLK_EN0_I2C0_POS                   (11UL)
#define SYS_CTRL_CLK_EN0_I2C0_MASK    				(1UL << SYS_CTRL_CLK_EN0_I2C0_POS)
#define SYS_CTRL_CLK_EN0_I2C0						SYS_CTRL_CLK_EN0_I2C0_MASK
#define SYS_CTRL_CLK_EN0_I2C1_POS                   (12UL)
#define SYS_CTRL_CLK_EN0_I2C1_MASK    				(1UL << SYS_CTRL_CLK_EN0_I2C1_POS)
#define SYS_CTRL_CLK_EN0_I2C1						SYS_CTRL_CLK_EN0_I2C1_MASK
#define SYS_CTRL_CLK_EN0_UART0_POS                  (13UL)
#define SYS_CTRL_CLK_EN0_UART0_MASK    				(1UL << SYS_CTRL_CLK_EN0_UART0_POS)
#define SYS_CTRL_CLK_EN0_UART0						SYS_CTRL_CLK_EN0_UART0_MASK
#define SYS_CTRL_CLK_EN0_UART1_POS                  (14UL)
#define SYS_CTRL_CLK_EN0_UART1_MASK    				(1UL << SYS_CTRL_CLK_EN0_UART1_POS)
#define SYS_CTRL_CLK_EN0_UART1						SYS_CTRL_CLK_EN0_UART1_MASK
#define SYS_CTRL_CLK_EN0_SPI0_POS                   (15UL)
#define SYS_CTRL_CLK_EN0_SPI0_MASK    				(1UL << SYS_CTRL_CLK_EN0_SPI0_POS)
#define SYS_CTRL_CLK_EN0_SPI0	                    SYS_CTRL_CLK_EN0_SPI0_MASK
#define SYS_CTRL_CLK_EN0_SPI1_POS                   (16UL)
#define SYS_CTRL_CLK_EN0_SPI1_MASK    				(1UL << SYS_CTRL_CLK_EN0_SPI1_POS)
#define SYS_CTRL_CLK_EN0_SPI1						SYS_CTRL_CLK_EN0_SPI1_MASK
#define SYS_CTRL_CLK_EN0_TIMER0_POS                 (17UL)
#define SYS_CTRL_CLK_EN0_TIMER0_MASK    			(1UL << SYS_CTRL_CLK_EN0_TIMER0_POS)
#define SYS_CTRL_CLK_EN0_TIMER0						SYS_CTRL_CLK_EN0_TIMER0_MASK
#define SYS_CTRL_CLK_EN0_TIMER1_POS                 (18UL)
#define SYS_CTRL_CLK_EN0_TIMER1_MASK    			(1UL << SYS_CTRL_CLK_EN0_TIMER1_POS)
#define SYS_CTRL_CLK_EN0_TIMER1						SYS_CTRL_CLK_EN0_TIMER1_MASK
#define SYS_CTRL_CLK_EN0_TIMER2_POS                 (19UL)
#define SYS_CTRL_CLK_EN0_TIMER2_MASK    			(1UL << SYS_CTRL_CLK_EN0_TIMER2_POS)
#define SYS_CTRL_CLK_EN0_TIMER2						SYS_CTRL_CLK_EN0_TIMER2_MASK
#define SYS_CTRL_CLK_EN0_TIMER3_POS                 (20UL)
#define SYS_CTRL_CLK_EN0_TIMER3_MASK    			(1UL << SYS_CTRL_CLK_EN0_TIMER3_POS)
#define SYS_CTRL_CLK_EN0_TIMER3						SYS_CTRL_CLK_EN0_TIMER3_MASK
#define SYS_CTRL_CLK_EN0_TIMER4_POS                 (21UL)
#define SYS_CTRL_CLK_EN0_TIMER4_MASK    			(1UL << SYS_CTRL_CLK_EN0_TIMER4_POS)
#define SYS_CTRL_CLK_EN0_TIMER4						SYS_CTRL_CLK_EN0_TIMER4_MASK
#define SYS_CTRL_CLK_EN0_TIMER5_POS                 (22UL)
#define SYS_CTRL_CLK_EN0_TIMER5_MASK    			(1UL << SYS_CTRL_CLK_EN0_TIMER5_POS)
#define SYS_CTRL_CLK_EN0_TIMER5						SYS_CTRL_CLK_EN0_TIMER5_MASK
#define SYS_CTRL_CLK_EN0_TIMER6_POS                 (23UL)
#define SYS_CTRL_CLK_EN0_TIMER6_MASK    			(1UL << SYS_CTRL_CLK_EN0_TIMER6_POS)
#define SYS_CTRL_CLK_EN0_TIMER6						SYS_CTRL_CLK_EN0_TIMER6_MASK
#define SYS_CTRL_CLK_EN0_TIMER7_POS                 (24UL)
#define SYS_CTRL_CLK_EN0_TIMER7_MASK    			(1UL << SYS_CTRL_CLK_EN0_TIMER7_POS)
#define SYS_CTRL_CLK_EN0_TIMER7						SYS_CTRL_CLK_EN0_TIMER7_MASK
#define SYS_CTRL_CLK_EN0_KSCAN_POS                  (25UL)
#define SYS_CTRL_CLK_EN0_KSCAN_MASK    				(1UL << SYS_CTRL_CLK_EN0_KSCAN_POS)
#define SYS_CTRL_CLK_EN0_KSCAN						SYS_CTRL_CLK_EN0_KSCAN_MASK
#define SYS_CTRL_CLK_EN0_PWM_POS                    (26UL)
#define SYS_CTRL_CLK_EN0_PWM_MASK    				(1UL << SYS_CTRL_CLK_EN0_PWM_POS)
#define SYS_CTRL_CLK_EN0_PWM						SYS_CTRL_CLK_EN0_PWM_MASK
#define SYS_CTRL_CLK_EN0_IR_POS                     (27UL)
#define SYS_CTRL_CLK_EN0_IR_MASK    				(1UL << SYS_CTRL_CLK_EN0_IR_POS)
#define SYS_CTRL_CLK_EN0_IR							SYS_CTRL_CLK_EN0_IR_MASK
//#define SYS_CTRL_CLK_EN0_CALIB_POS                  (28UL)
//#define SYS_CTRL_CLK_EN0_CALIB_MASK    				(1UL << SYS_CTRL_CLK_EN0_CALIB_POS)
//#define SYS_CTRL_CLK_EN0_CALIB						SYS_CTRL_CLK_EN0_CALIB_MASK
#define SYS_CTRL_CLK_EN0_QSPI_REF_POS               (29UL)
#define SYS_CTRL_CLK_EN0_QSPI_REF_MASK    			(1UL << SYS_CTRL_CLK_EN0_QSPI_REF_POS)
#define SYS_CTRL_CLK_EN0_QSPI_REF					SYS_CTRL_CLK_EN0_QSPI_REF_MASK
#define SYS_CTRL_CLK_EN0_MDM_POS                    (30UL)
#define SYS_CTRL_CLK_EN0_MDM_MASK                   (1UL << SYS_CTRL_CLK_EN0_MDM_POS)
#define SYS_CTRL_CLK_EN0_MDM                        SYS_CTRL_CLK_EN0_MDM_MASK
#define SYS_CTRL_CLK_EN0_BLE_POS                    (31UL)
#define SYS_CTRL_CLK_EN0_BLE_MASK                   (1UL << SYS_CTRL_CLK_EN0_BLE_POS)
#define SYS_CTRL_CLK_EN0_BLE                        SYS_CTRL_CLK_EN0_BLE_MASK

/**
 *@brief  SYS_CTRL_CLK_EN1 BIT DEFINE
 */
#define SYS_CTRL_CLK_EN1_OFFSET_POS               	0x08
#define SYS_CTRL_CLK_EN1_I2C0_POS           		(0UL)
#define SYS_CTRL_CLK_EN1_I2C0_MASK    				(1UL << SYS_CTRL_CLK_EN1_I2C0_POS)
#define SYS_CTRL_CLK_EN1_I2C0						SYS_CTRL_CLK_EN1_I2C0_MASK
#define SYS_CTRL_CLK_EN1_I2C1_POS           		(1UL)
#define SYS_CTRL_CLK_EN1_I2C1_MASK    				(1UL << SYS_CTRL_CLK_EN1_I2C1_POS)
#define SYS_CTRL_CLK_EN1_I2C1						SYS_CTRL_CLK_EN1_I2C1_MASK
#define SYS_CTRL_CLK_EN1_UART0_POS          		(2UL)
#define SYS_CTRL_CLK_EN1_UART0_MASK    				(1UL << SYS_CTRL_CLK_EN1_UART0_POS)
#define SYS_CTRL_CLK_EN1_UART0						SYS_CTRL_CLK_EN1_UART0_MASK
#define SYS_CTRL_CLK_EN1_UART1_POS          		(3UL)
#define SYS_CTRL_CLK_EN1_UART1_MASK    				(1UL << SYS_CTRL_CLK_EN1_UART1_POS)
#define SYS_CTRL_CLK_EN1_UART1						SYS_CTRL_CLK_EN1_UART1_MASK
#define SYS_CTRL_CLK_EN1_SPI0_POS          			(4UL)
#define SYS_CTRL_CLK_EN1_SPI0_MASK    				(1UL << SYS_CTRL_CLK_EN1_SPI0_POS)
#define SYS_CTRL_CLK_EN1_SPI0						SYS_CTRL_CLK_EN1_SPI0_MASK
#define SYS_CTRL_CLK_EN1_SPI1_POS            		(5UL)
#define SYS_CTRL_CLK_EN1_SPI1_MASK    				(1UL << SYS_CTRL_CLK_EN1_SPI1_POS)
#define SYS_CTRL_CLK_EN1_SPI1						SYS_CTRL_CLK_EN1_SPI1_MASK
#define SYS_CTRL_CLK_EN1_TIMER_POS           		(6UL)
#define SYS_CTRL_CLK_EN1_TIMER_MASK    				(1UL << SYS_CTRL_CLK_EN1_TIMER_POS)
#define SYS_CTRL_CLK_EN1_TIMER						SYS_CTRL_CLK_EN1_TIMER_MASK
#define SYS_CTRL_CLK_EN1_KSCAN_POS          		(7UL)
#define SYS_CTRL_CLK_EN1_KSCAN_MASK    				(1UL << SYS_CTRL_CLK_EN1_KSCAN_POS)
#define SYS_CTRL_CLK_EN1_KSCAN						SYS_CTRL_CLK_EN1_KSCAN_MASK
#define SYS_CTRL_CLK_EN1_PWM_POS            		(8UL)
#define SYS_CTRL_CLK_EN1_PWM_MASK    				(1UL << SYS_CTRL_CLK_EN1_PWM_POS)
#define SYS_CTRL_CLK_EN1_PWM						SYS_CTRL_CLK_EN1_PWM_MASK
#define SYS_CTRL_CLK_EN1_CPU_TIMER_POS             	(9UL)
#define SYS_CTRL_CLK_EN1_CPU_TIMER_MASK    			(1UL << SYS_CTRL_CLK_EN1_CPU_TIMER_POS)
#define SYS_CTRL_CLK_EN1_CPU_TIMER					SYS_CTRL_CLK_EN1_CPU_TIMER_MASK
#define SYS_CTRL_CLK_EN1_IR_POS             		(10UL)
#define SYS_CTRL_CLK_EN1_IR_MASK    				(1UL << SYS_CTRL_CLK_EN1_IR_POS)
#define SYS_CTRL_CLK_EN1_IR							SYS_CTRL_CLK_EN1_IR_MASK
#define SYS_CTRL_CLK_EN1_I2S0_POS           		(11UL)
#define SYS_CTRL_CLK_EN1_I2S0_MASK    				(1UL << SYS_CTRL_CLK_EN1_I2S0_POS)
#define SYS_CTRL_CLK_EN1_I2S0						SYS_CTRL_CLK_EN1_I2S0_MASK
#define SYS_CTRL_CLK_EN1_I2S1_POS           		(12UL)
#define SYS_CTRL_CLK_EN1_I2S1_MASK    				(1UL << SYS_CTRL_CLK_EN1_I2S1_POS)
#define SYS_CTRL_CLK_EN1_I2S1						SYS_CTRL_CLK_EN1_I2S1_MASK
#define SYS_CTRL_CLK_EN1_GPIO_POS           		(13UL)
#define SYS_CTRL_CLK_EN1_GPIO_MASK    				(1UL << SYS_CTRL_CLK_EN1_GPIO_POS)
#define SYS_CTRL_CLK_EN1_GPIO						SYS_CTRL_CLK_EN1_GPIO_MASK
#define SYS_CTRL_CLK_EN1_UART2_POS          		(14UL)
#define SYS_CTRL_CLK_EN1_UART2_MASK    				(1UL << SYS_CTRL_CLK_EN1_UART2_POS)
#define SYS_CTRL_CLK_EN1_UART2						SYS_CTRL_CLK_EN1_UART2_MASK
#define SYS_CTRL_CLK_EN1_SYS_POS 	        		(15UL)
#define SYS_CTRL_CLK_EN1_SYS_MASK    				(1UL << SYS_CTRL_CLK_EN1_SYS_POS)
#define SYS_CTRL_CLK_EN1_SYS						SYS_CTRL_CLK_EN1_SYS_MASK
#define SYS_CTRL_CLK_EN1_RTC_POS            		(16UL)
#define SYS_CTRL_CLK_EN1_RTC_MASK    				(1UL << SYS_CTRL_CLK_EN1_RTC_POS)
#define SYS_CTRL_CLK_EN1_RTC						SYS_CTRL_CLK_EN1_RTC_MASK
#define SYS_CTRL_CLK_EN1_TRNG_POS           		(17UL)
#define SYS_CTRL_CLK_EN1_TRNG_MASK    				(1UL << SYS_CTRL_CLK_EN1_TRNG_POS)
#define SYS_CTRL_CLK_EN1_TRNG						SYS_CTRL_CLK_EN1_TRNG_MASK
#define SYS_CTRL_CLK_EN1_WDT_POS            		(18UL)
#define SYS_CTRL_CLK_EN1_WDT_MASK    				(1UL << SYS_CTRL_CLK_EN1_WDT_POS)
#define SYS_CTRL_CLK_EN1_WDT						SYS_CTRL_CLK_EN1_WDT_MASK
#define SYS_CTRL_CLK_EN1_AUD_POS            		(19UL)
#define SYS_CTRL_CLK_EN1_AUD_MASK    				(1UL << SYS_CTRL_CLK_EN1_AUD_POS)
#define SYS_CTRL_CLK_EN1_AUD						SYS_CTRL_CLK_EN1_AUD_MASK
#define SYS_CTRL_CLK_EN1_AUX_POS           			(20UL)
#define SYS_CTRL_CLK_EN1_AUX_MASK    				(1UL << SYS_CTRL_CLK_EN1_AUX_POS)
#define SYS_CTRL_CLK_EN1_AUX						SYS_CTRL_CLK_EN1_AUX_MASK

/**
 *@brief  SYS_CTRL_CLK_DIV0 BIT DEFINE
 */
#define SYS_CTRL_CLK_DIV0_OFFSET              		0x0c
#define SYS_CTRL_CLK_DIV0_I2S0_MDIV_POS      		(0UL)
#define SYS_CTRL_CLK_DIV0_I2S0_MDIV_MASK      		(0xFUL << SYS_CTRL_CLK_DIV0_I2S0_MDIV_POS)
#define SYS_CTRL_CLK_DIV0_I2S0_MDIV					SYS_CTRL_CLK_DIV0_I2S0_MDIV_MASK
#define SYS_CTRL_CLK_DIV0_I2S1_MDIV_POS      		(4UL)
#define SYS_CTRL_CLK_DIV0_I2S1_MDIV_MASK      		(0xFUL << SYS_CTRL_CLK_DIV0_I2S1_MDIV_POS)
#define SYS_CTRL_CLK_DIV0_I2S1_MDIV					SYS_CTRL_CLK_DIV0_I2S1_MDIV_MASK
#define SYS_CTRL_CLK_DIV0_I2S0_DIV_POS       		(8UL)
#define SYS_CTRL_CLK_DIV0_I2S0_DIV_MASK      		(0x1FFUL << SYS_CTRL_CLK_DIV0_I2S0_DIV_POS) 
#define SYS_CTRL_CLK_DIV0_I2S0_DIV					SYS_CTRL_CLK_DIV0_I2S0_DIV_MASK
#define SYS_CTRL_CLK_DIV0_I2S1_DIV_POS       		(17UL) 
#define SYS_CTRL_CLK_DIV0_I2S1_DIV_MASK      		(0x1FFUL << SYS_CTRL_CLK_DIV0_I2S1_DIV_POS) 
#define SYS_CTRL_CLK_DIV0_I2S1_DIV					SYS_CTRL_CLK_DIV0_I2S1_DIV_MASK

/**
 *@brief  SYS_CTRL_CLK_DIV1 BIT DEFINE
 */
#define SYS_CTRL_CLK_DIV1_OFFSET_POS              	0x10
#define SYS_CTRL_CLK_DIV1_I2C0_POS          		(0UL)
#define SYS_CTRL_CLK_DIV1_I2C0_MASK      			(0xFUL << SYS_CTRL_CLK_DIV1_I2C0_POS)
#define SYS_CTRL_CLK_DIV1_I2C0						SYS_CTRL_CLK_DIV1_I2C0_MASK
#define SYS_CTRL_CLK_DIV1_I2C1_POS          		(4UL)
#define SYS_CTRL_CLK_DIV1_I2C1_MASK      			(0xFUL << SYS_CTRL_CLK_DIV1_I2C1_POS)
#define SYS_CTRL_CLK_DIV1_I2C1						SYS_CTRL_CLK_DIV1_I2C1_MASK
#define SYS_CTRL_CLK_DIV1_UART0_POS         		(8UL)
#define SYS_CTRL_CLK_DIV1_UART0_MASK      			(0xFUL << SYS_CTRL_CLK_DIV1_UART0_POS)
#define SYS_CTRL_CLK_DIV1_UART0						SYS_CTRL_CLK_DIV1_UART0_MASK
#define SYS_CTRL_CLK_DIV1_UART1_POS         		(12UL)
#define SYS_CTRL_CLK_DIV1_UART1_MASK      			(0xFUL << SYS_CTRL_CLK_DIV1_UART1_POS)
#define SYS_CTRL_CLK_DIV1_UART1						SYS_CTRL_CLK_DIV1_UART1_MASK
#define SYS_CTRL_CLK_DIV1_SPI0_POS          		(16UL)
#define SYS_CTRL_CLK_DIV1_SPI0_MASK      			(0xFUL << SYS_CTRL_CLK_DIV1_SPI0_POS)
#define SYS_CTRL_CLK_DIV1_SPI0						SYS_CTRL_CLK_DIV1_SPI0_MASK
#define SYS_CTRL_CLK_DIV1_SPI1_POS          		(20UL)
#define SYS_CTRL_CLK_DIV1_SPI1_MASK      			(0xFUL << SYS_CTRL_CLK_DIV1_SPI1_POS)
#define SYS_CTRL_CLK_DIV1_SPI1						SYS_CTRL_CLK_DIV1_SPI1_MASK
#define SYS_CTRL_CLK_DIV1_TIMER0_POS        		(24UL)
#define SYS_CTRL_CLK_DIV1_TIMER0_MASK      			(0xFUL << SYS_CTRL_CLK_DIV1_TIMER0_POS)
#define SYS_CTRL_CLK_DIV1_TIMER0					SYS_CTRL_CLK_DIV1_TIMER0_MASK
#define SYS_CTRL_CLK_DIV1_TIMER1_POS        		(28UL)
#define SYS_CTRL_CLK_DIV1_TIMER1_MASK      			(0xFUL << SYS_CTRL_CLK_DIV1_TIMER1_POS)
#define SYS_CTRL_CLK_DIV1_TIMER1					SYS_CTRL_CLK_DIV1_TIMER1_MASK

/**
 *@brief  SYS_CTRL_CLK_DIV2 BIT DEFINE
 */
#define SYS_CTRL_CLK_DIV2_OFFSET              		0x14
#define SYS_CTRL_CLK_DIV2_TIMER2_POS        		(0UL)
#define SYS_CTRL_CLK_DIV2_TIMER2_MASK      			(0xFUL << SYS_CTRL_CLK_DIV2_TIMER2_POS)
#define SYS_CTRL_CLK_DIV2_TIMER2					SYS_CTRL_CLK_DIV2_TIMER2_MASK
#define SYS_CTRL_CLK_DIV2_TIMER3_POS        		(4UL)
#define SYS_CTRL_CLK_DIV2_TIMER3_MASK      			(0xFUL << SYS_CTRL_CLK_DIV2_TIMER3_POS)
#define SYS_CTRL_CLK_DIV2_TIMER3					SYS_CTRL_CLK_DIV2_TIMER3_MASK
#define SYS_CTRL_CLK_DIV2_TIMER4_POS        		(8UL)
#define SYS_CTRL_CLK_DIV2_TIMER4_MASK      			(0xFUL << SYS_CTRL_CLK_DIV2_TIMER4_POS)
#define SYS_CTRL_CLK_DIV2_TIMER4					SYS_CTRL_CLK_DIV2_TIMER4_MASK
#define SYS_CTRL_CLK_DIV2_TIMER5_POS        		(12UL)
#define SYS_CTRL_CLK_DIV2_TIMER5_MASK      			(0xFUL << SYS_CTRL_CLK_DIV2_TIMER5_POS)
#define SYS_CTRL_CLK_DIV2_TIMER5					SYS_CTRL_CLK_DIV2_TIMER5_MASK
#define SYS_CTRL_CLK_DIV2_TIMER6_POS        		(16UL)
#define SYS_CTRL_CLK_DIV2_TIMER6_MASK      			(0xFUL << SYS_CTRL_CLK_DIV2_TIMER6_POS)
#define SYS_CTRL_CLK_DIV2_TIMER6					SYS_CTRL_CLK_DIV2_TIMER6_MASK
#define SYS_CTRL_CLK_DIV2_TIMER7_POS        		(20UL)
#define SYS_CTRL_CLK_DIV2_TIMER7_MASK      			(0xFUL << SYS_CTRL_CLK_DIV2_TIMER7_POS)
#define SYS_CTRL_CLK_DIV2_TIMER7					SYS_CTRL_CLK_DIV2_TIMER7_MASK
//#define SYS_CTRL_CLK_DIV2_KSCAN_POS         		    (24UL)
//#define SYS_CTRL_CLK_DIV2_KSCAN_MASK      			(0xFUL << SYS_CTRL_CLK_DIV2_KSCAN_POS)
//#define SYS_CTRL_CLK_DIV2_KSCAN						SYS_CTRL_CLK_DIV2_KSCAN_MASK
#define SYS_CTRL_CLK_DIV2_PWM_POS          			(28UL)
#define SYS_CTRL_CLK_DIV2_PWM_MASK      			(0xFUL << SYS_CTRL_CLK_DIV2_PWM_POS)
#define SYS_CTRL_CLK_DIV2_PWM						SYS_CTRL_CLK_DIV2_PWM_MASK

/**
 *@brief  SYS_CTRL_CLK_DIV3 BIT DEFINE
 */
#define SYS_CTRL_CLK_DIV3_OFFSET              		0x18
#define SYS_CTRL_CLK_DIV3_CPU_TMR_POS            	(0UL)
#define SYS_CTRL_CLK_DIV3_CPU_TMR_MASK      		(0xFUL << SYS_CTRL_CLK_DIV3_CPU_TMR_POS)
#define SYS_CTRL_CLK_DIV3_CPU_TMR					SYS_CTRL_CLK_DIV3_CPU_TMR_MASK
#define SYS_CTRL_CLK_DIV3_IR_POS                  	(4UL)
#define SYS_CTRL_CLK_DIV3_IR_MASK      				(0xFUL << SYS_CTRL_CLK_DIV3_IR_POS)
#define SYS_CTRL_CLK_DIV3_IR						SYS_CTRL_CLK_DIV3_IR_MASK
#define SYS_CTRL_CLK_DIV3_AHB_POS                	(8UL)
#define SYS_CTRL_CLK_DIV3_AHB_MASK      			(0xFUL << SYS_CTRL_CLK_DIV3_AHB_POS)
#define SYS_CTRL_CLK_DIV3_AHB						SYS_CTRL_CLK_DIV3_AHB_MASK
#define SYS_CTRL_CLK_DIV3_APB_POS                 	(12UL)
#define SYS_CTRL_CLK_DIV3_APB_MASK      			(0xFUL << SYS_CTRL_CLK_DIV3_APB_POS)
#define SYS_CTRL_CLK_DIV3_APB						SYS_CTRL_CLK_DIV3_APB_MASK
#define SYS_CTRL_CLK_DIV3_WDT_POS                 	(16UL)
#define SYS_CTRL_CLK_DIV3_WDT_MASK      			(0xFUL << SYS_CTRL_CLK_DIV3_WDT_POS)
#define SYS_CTRL_CLK_DIV3_WDT						SYS_CTRL_CLK_DIV3_WDT_MASK
//#define SYS_CTRL_CLK_DIV3_CALI_POS                    (20UL)
//#define SYS_CTRL_CLK_DIV3_CALI_MASK      			    (0xFUL << SYS_CTRL_CLK_DIV3_CALI_POS)
//#define SYS_CTRL_CLK_DIV3_CALI					    SYS_CTRL_CLK_DIV3_CALI_MASK
//#define SYS_CTRL_CLK_DIV3_AUD_ADC_POS                 (24UL)
//#define SYS_CTRL_CLK_DIV3_AUD_ADC_MASK      			(0xFUL << SYS_CTRL_CLK_DIV3_AUD_ADC_POS)
//#define SYS_CTRL_CLK_DIV3_AUD_ADC						SYS_CTRL_CLK_DIV3_AUD_ADC_MASK
#define SYS_CTRL_CLK_DIV3_AUX_ADC_POS                (28UL)
#define SYS_CTRL_CLK_DIV3_AUX_ADC_MASK               (0xFUL << SYS_CTRL_CLK_DIV3_AUX_ADC_POS)
#define SYS_CTRL_CLK_DIV3_AUX_ADC                    SYS_CTRL_CLK_DIV3_AUX_ADC_MASK

/**
 *@brief  SYS_CTRL_GPIO_MODE0 BIT DEFINE
 */
#define SYS_CTRL_GPIO_MODE0_OFFSET             		0x1c
#define SYS_CTRL_GPIO_MODE0_IO0_POS                	(0UL)
#define SYS_CTRL_GPIO_MODE0_IO0_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE0_IO0_POS)
#define SYS_CTRL_GPIO_MODE0_IO0						SYS_CTRL_GPIO_MODE0_IO0_MASK
#define SYS_CTRL_GPIO_MODE0_IO1_POS                	(4UL)
#define SYS_CTRL_GPIO_MODE0_IO1_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE0_IO1_POS)
#define SYS_CTRL_GPIO_MODE0_IO1						SYS_CTRL_GPIO_MODE0_IO1_MASK
#define SYS_CTRL_GPIO_MODE0_IO2_POS                	(8UL)
#define SYS_CTRL_GPIO_MODE0_IO2_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE0_IO2_POS)
#define SYS_CTRL_GPIO_MODE0_IO2						SYS_CTRL_GPIO_MODE0_IO2_MASK
#define SYS_CTRL_GPIO_MODE0_IO3_POS                	(12UL)
#define SYS_CTRL_GPIO_MODE0_IO3_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE0_IO3_POS)
#define SYS_CTRL_GPIO_MODE0_IO3						SYS_CTRL_GPIO_MODE0_IO3_MASK
#define SYS_CTRL_GPIO_MODE0_IO4_POS                	(16UL)
#define SYS_CTRL_GPIO_MODE0_IO4_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE0_IO4_POS)
#define SYS_CTRL_GPIO_MODE0_IO4						SYS_CTRL_GPIO_MODE0_IO4_MASK
#define SYS_CTRL_GPIO_MODE0_IO5_POS                	(20UL)
#define SYS_CTRL_GPIO_MODE0_IO5_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE0_IO5_POS)
#define SYS_CTRL_GPIO_MODE0_IO5						SYS_CTRL_GPIO_MODE0_IO5_MASK
#define SYS_CTRL_GPIO_MODE0_IO6_POS                	(24UL)
#define SYS_CTRL_GPIO_MODE0_IO6_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE0_IO6_POS)
#define SYS_CTRL_GPIO_MODE0_IO6						SYS_CTRL_GPIO_MODE0_IO6_MASK
#define SYS_CTRL_GPIO_MODE0_IO7_POS                	(28UL)
#define SYS_CTRL_GPIO_MODE0_IO7_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE0_IO7_POS)
#define SYS_CTRL_GPIO_MODE0_IO7						SYS_CTRL_GPIO_MODE0_IO7_MASK

/**
 *@brief  SYS_CTRL_GPIO_MODE1 BIT DEFINE
 */
#define SYS_CTRL_GPIO_MODE1_OFFSET            		0x20
#define SYS_CTRL_GPIO_MODE1_IO8_POS            		(0UL)
#define SYS_CTRL_GPIO_MODE1_IO8_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE1_IO8_POS)
#define SYS_CTRL_GPIO_MODE1_IO8						SYS_CTRL_GPIO_MODE1_IO8_MASK
#define SYS_CTRL_GPIO_MODE1_IO9_POS            		(4UL)
#define SYS_CTRL_GPIO_MODE1_IO9_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE1_IO9_POS)
#define SYS_CTRL_GPIO_MODE1_IO9						SYS_CTRL_GPIO_MODE1_IO9_MASK
#define SYS_CTRL_GPIO_MODE1_IO10_POS           		(8UL)
#define SYS_CTRL_GPIO_MODE1_IO10_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE1_IO10_POS)
#define SYS_CTRL_GPIO_MODE1_IO10					SYS_CTRL_GPIO_MODE1_IO10_MASK
#define SYS_CTRL_GPIO_MODE1_IO11_POS          		 (12UL)
#define SYS_CTRL_GPIO_MODE1_IO11_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE1_IO11_POS)
#define SYS_CTRL_GPIO_MODE1_IO11					SYS_CTRL_GPIO_MODE1_IO11_MASK
#define SYS_CTRL_GPIO_MODE1_IO12_POS           		(16UL)
#define SYS_CTRL_GPIO_MODE1_IO12_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE1_IO12_POS)
#define SYS_CTRL_GPIO_MODE1_IO12					SYS_CTRL_GPIO_MODE1_IO12_MASK
#define SYS_CTRL_GPIO_MODE1_IO13_POS           		(20UL)
#define SYS_CTRL_GPIO_MODE1_IO13_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE1_IO13_POS)
#define SYS_CTRL_GPIO_MODE1_IO13					SYS_CTRL_GPIO_MODE1_IO13_MASK
#define SYS_CTRL_GPIO_MODE1_IO14_POS           		(24UL)
#define SYS_CTRL_GPIO_MODE1_IO14_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE1_IO14_POS)
#define SYS_CTRL_GPIO_MODE1_IO14					SYS_CTRL_GPIO_MODE1_IO14_MASK
#define SYS_CTRL_GPIO_MODE1_IO15_POS           		(28UL)
#define SYS_CTRL_GPIO_MODE1_IO15_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE1_IO15_POS)
#define SYS_CTRL_GPIO_MODE1_IO15					SYS_CTRL_GPIO_MODE1_IO15_MASK

/**
 *@brief  SYS_CTRL_GPIO_MODE2 BIT DEFINE
 */
#define SYS_CTRL_GPIO_MODE2_OFFSET            		0x24

#define SYS_CTRL_GPIO_MODE2_IO16_POS           		(0UL)
#define SYS_CTRL_GPIO_MODE2_IO16_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE2_IO16_POS)
#define SYS_CTRL_GPIO_MODE2_IO16					SYS_CTRL_GPIO_MODE2_IO16_MASK
#define SYS_CTRL_GPIO_MODE2_IO17_POS           		(4UL)
#define SYS_CTRL_GPIO_MODE2_IO17_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE2_IO17_POS)
#define SYS_CTRL_GPIO_MODE2_IO17					SYS_CTRL_GPIO_MODE2_IO17_MASK
#define SYS_CTRL_GPIO_MODE2_IO18_POS           		(8UL)
#define SYS_CTRL_GPIO_MODE2_IO18_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE2_IO18_POS)
#define SYS_CTRL_GPIO_MODE2_IO18					SYS_CTRL_GPIO_MODE2_IO18_MASK
#define SYS_CTRL_GPIO_MODE2_IO19_POS           		(12UL)
#define SYS_CTRL_GPIO_MODE2_IO19_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE2_IO19_POS)
#define SYS_CTRL_GPIO_MODE2_IO19					SYS_CTRL_GPIO_MODE2_IO19_MASK
#define SYS_CTRL_GPIO_MODE2_IO20_POS           		(16UL)
#define SYS_CTRL_GPIO_MODE2_IO20_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE2_IO20_POS)
#define SYS_CTRL_GPIO_MODE2_IO20					SYS_CTRL_GPIO_MODE2_IO20_MASK
#define SYS_CTRL_GPIO_MODE2_IO21_POS           		(20UL)
#define SYS_CTRL_GPIO_MODE2_IO21_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE2_IO21_POS)
#define SYS_CTRL_GPIO_MODE2_IO21					SYS_CTRL_GPIO_MODE2_IO21_MASK
#define SYS_CTRL_GPIO_MODE2_IO22_POS           		(24UL)
#define SYS_CTRL_GPIO_MODE2_IO22_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE2_IO22_POS)
#define SYS_CTRL_GPIO_MODE2_IO22					SYS_CTRL_GPIO_MODE2_IO22_MASK
#define SYS_CTRL_GPIO_MODE2_IO23_POS           		(28UL)
#define SYS_CTRL_GPIO_MODE2_IO23_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE2_IO23_POS)
#define SYS_CTRL_GPIO_MODE2_IO23					SYS_CTRL_GPIO_MODE2_IO23_MASK 

/**
 *@brief  SYS_CTRL_GPIO_MODE3 BIT DEFINE
 */
#define SYS_CTRL_GPIO_MODE3_OFFSET            		0x28
#define SYS_CTRL_GPIO_MODE3_IO24_POS           		(0UL)
#define SYS_CTRL_GPIO_MODE3_IO24_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE3_IO24_POS)
#define SYS_CTRL_GPIO_MODE3_IO24					SYS_CTRL_GPIO_MODE3_IO24_MASK
#define SYS_CTRL_GPIO_MODE3_IO25_POS           		(4UL)
#define SYS_CTRL_GPIO_MODE3_IO25_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE3_IO25_POS)
#define SYS_CTRL_GPIO_MODE3_IO25					SYS_CTRL_GPIO_MODE3_IO25_MASK
#define SYS_CTRL_GPIO_MODE3_IO26_POS           		(8UL)
#define SYS_CTRL_GPIO_MODE3_IO26_MASK      			(0xFUL << SYS_CTRL_GPIO_MODE3_IO26_POS)
#define SYS_CTRL_GPIO_MODE3_IO26					SYS_CTRL_GPIO_MODE3_IO26_MASK

/**
 *@brief  SYS_CTRL_OTA_SRC_ADDR
 */
#define SYS_CTRL_OTA_SRC_ADDR_OFFSET          0x2c
/**
 *@brief  SYS_CTRL_OTA_DST_ADDR
 */
#define SYS_CTRL_OTA_DST_ADDR_OFFSET          0x30
/**
 *@brief  SYS_CTRL_OTA_OFT_ADDR
 */
#define SYS_CTRL_OTA_OFT_ADDR_OFFSET          0x34

/**
 *@brief  SYS_CTRL_REMAP_CTRL_REG BIT DEFINE
 */
#define SYS_CTRL_REMAP_CTRL_OFFSET            	0x38

#define SYS_CTRL_REMAP_CTRL_OTA_EN_POS           	(0UL)
#define SYS_CTRL_REMAP_CTRL_OTA_EN_MASK      		(0x1UL << SYS_CTRL_REMAP_CTRL_OTA_EN_POS)
#define SYS_CTRL_REMAP_CTRL_OTA_EN					SYS_CTRL_REMAP_CTRL_OTA_EN_MASK
#define SYS_CTRL_REMAP_CTRL_PERIPH_EN_POS           (1UL)
#define SYS_CTRL_REMAP_CTRL_PERIPH_EN_MASK      	(0x1UL << SYS_CTRL_REMAP_CTRL_PERIPH_EN_POS)
#define SYS_CTRL_REMAP_CTRL_PERIPH_EN				SYS_CTRL_REMAP_CTRL_PERIPH_EN_MASK

/**
 *@brief  SYS_CTRL_CALI_CTL BIT DEFINE
 */
//#define SYS_CTRL_CALI_CTL_OFFSET            	0x3c
//#define SYS_CTRL_CALI_KST_POS           		(0UL)
//#define SYS_CTRL_CALI_KST_MASK      			(0x1UL << SYS_CTRL_CALI_KST_POS)
//#define SYS_CTRL_CALI_KST						SYS_CTRL_CALI_KST_MASK
//#define SYS_CTRL_CALI_INT_EN_POS           		(2UL)
//#define SYS_CTRL_CALI_INT_EN_MASK      			(0x1UL << SYS_CTRL_CALI_INT_EN_POS)
//#define SYS_CTRL_CALI_INT_EN					SYS_CTRL_CALI_INT_EN_MASK
/**
 *@brief  SYS_CTRL_CALI_CNT_LEN
 */
#define SYS_CTRL_CALI_CNT_LEN_OFFSET            0x40
/**
 *@brief  SYS_CTRL_CALI_CNT_RES
 */
#define SYS_CTRL_CALI_CNT_RES_OFFSET            0x44
/**
 *@brief  SYS_CTRL_CALI_STATUS BIT DEFINE
 */
//#define SYS_CTRL_CALI_STATUS_OFFSET           	0x48
//#define SYS_CTRL_CALI_STATUS_DONE_POS           (0UL)
//#define SYS_CTRL_CALI_STATUS_DONE_MASK      	(0x1UL << SYS_CTRL_CALI_STATUS_DONE_POS)
//#define SYS_CTRL_CALI_STATUS_DONE				SYS_CTRL_CALI_STATUS_DONE_MASK
//#define SYS_CTRL_CALI_STATUS_BUSY_POS           (1UL)
//#define SYS_CTRL_CALI_STATUS_BUSY_MASK      	(0x1UL << SYS_CTRL_CALI_STATUS_BUSY_POS)
//#define SYS_CTRL_CALI_STATUS_BUSY				SYS_CTRL_CALI_STATUS_BUSY_MASK
//#define SYS_CTRL_CALI_STATUS_OVF_POS           	(2UL)
//#define SYS_CTRL_CALI_STATUS_OVF_MASK      		(0x1UL << SYS_CTRL_CALI_STATUS_OVF_POS)
//#define SYS_CTRL_CALI_STATUS_OVF				SYS_CTRL_CALI_STATUS_OVF_MASK
/**
 *@brief  SYS_CTRL_PLL_CTRL_REG BIT DEFINE
 */
#define SYS_CTRL_PLL_CTRL_OFFSET            	0x4c
#define SYS_CTRL_PLL_CTRL_LOCK_POS           	(1UL)
#define SYS_CTRL_PLL_CTRL_LOCK_MASK      		(0x1UL << SYS_CTRL_PLL_CTRL_LOCK_POS)
#define SYS_CTRL_PLL_CTRL_LOCK					SYS_CTRL_PLL_CTRL_LOCK_MASK
#define SYS_CTRL_PLL_CTRL_PDIV_POS           	(2UL)
#define SYS_CTRL_PLL_CTRL_PDIV_MASK      		(0x1FUL << SYS_CTRL_PLL_CTRL_PDIV_POS)
#define SYS_CTRL_PLL_CTRL_PDIV					SYS_CTRL_PLL_CTRL_PDIV_MASK
#define SYS_CTRL_PLL_CTRL_FDIV_POS           	(7UL)
#define SYS_CTRL_PLL_CTRL_FDIV_MASK      		(0x3FFUL << SYS_CTRL_PLL_CTRL_FDIV_POS)
#define SYS_CTRL_PLL_CTRL_FDIV					SYS_CTRL_PLL_CTRL_FDIV_MASK


/**
 *@brief  SYS_CTRL_PWR_MODE_CTRL_REG
 */
#define SYS_CTRL_PRE_PWR_MODE_OFFSET          	0x50
//#define SYS_CTRL_PRE_PWR_MODE0_POS 				(0UL)
//#define SYS_CTRL_PRE_PWR_MODE0_MASK      		(0x1UL << SYS_CTRL_PRE_PWR_MODE0_POS)
//#define SYS_CTRL_PRE_PWR_MODE0					SYS_CTRL_PRE_PWR_MODE0_MASK
//#define SYS_CTRL_PRE_PWR_MODE1_POS 				(1UL)
//#define SYS_CTRL_PRE_PWR_MODE1_MASK      		(0x1UL << SYS_CTRL_PRE_PWR_MODE1_POS)
//#define SYS_CTRL_PRE_PWR_MODE1					SYS_CTRL_PRE_PWR_MODE1_MASK
//#define SYS_CTRL_PRE_PWR_MODE2_POS 				(2UL)
//#define SYS_CTRL_PRE_PWR_MODE2_MASK      		(0x1UL << SYS_CTRL_PRE_PWR_MODE2_POS)
//#define SYS_CTRL_PRE_PWR_MODE2					SYS_CTRL_PRE_PWR_MODE2_MASK
//#define SYS_CTRL_PRE_PWR_MODE3_POS 				(3UL)
//#define SYS_CTRL_PRE_PWR_MODE3_MASK      		(0x1UL << SYS_CTRL_PRE_PWR_MODE3_POS)
//#define SYS_CTRL_PRE_PWR_MODE3					SYS_CTRL_PRE_PWR_MODE3_MASK
/**
 *@brief  SYS_CTRL_PWR_EN_OFFSET
 */
#define SYS_CTRL_PWR_EN_OFFSET                	0x54
#define SYS_CTRL_PWR_EN_PD_POS					(0UL)	
#define SYS_CTRL_PWR_EN_PD_MASK      			(0x1UL << SYS_CTRL_PWR_EN_PD_POS)
#define SYS_CTRL_PWR_EN_PD						SYS_CTRL_PWR_EN_PD_MASK
#define SYS_CTRL_PWR_EN_DS_POS					(1UL)	
#define SYS_CTRL_PWR_EN_DS_MASK      			(0x1UL << SYS_CTRL_PWR_EN_DS_POS)
#define SYS_CTRL_PWR_EN_DS						SYS_CTRL_PWR_EN_DS_MASK
#define SYS_CTRL_PWR_EN_SLP_POS					(2UL)	
#define SYS_CTRL_PWR_EN_SLP_MASK      			(0x1UL << SYS_CTRL_PWR_EN_SLP_POS)
#define SYS_CTRL_PWR_EN_SLP						SYS_CTRL_PWR_EN_SLP_MASK

/**
 *@brief  SYS_CTRL_BOOT_SEL_REG
 */
#define SYS_CTRL_BOOT_SEL_OFFSET              	0x58
#define SYS_CTRL_BOOT_SEL0_POS					(0UL)	
#define SYS_CTRL_BOOT_SEL0_MASK      			(0x1UL << SYS_CTRL_BOOT_SEL0_POS)
#define SYS_CTRL_BOOT_SEL0						SYS_CTRL_BOOT_SEL0_MASK
#define SYS_CTRL_BOOT_SEL1_POS					(0UL)	
#define SYS_CTRL_BOOT_SEL1_MASK      			(0x1UL << SYS_CTRL_BOOT_SEL1_POS)
#define SYS_CTRL_BOOT_SEL1						SYS_CTRL_BOOT_SEL1_MASK

/**
 *@brief  EFUSE Control Bits Definition
 */
#define SYS_CTRL_EFUSE_CTRL_OFFSET            	    0x5C
#define SYS_CTRL_EFUSE_CTRL_RD_EN_POS               (0UL)
#define SYS_CTRL_EFUSE_CTRL_RD_EN_MASK              (0x1UL << SYS_CTRL_EFUSE_CTRL_RD_EN_POS)
#define SYS_CTRL_EFUSE_CTRL_RD_EN                   SYS_CTRL_EFUSE_CTRL_RD_EN_MASK
#define SYS_CTRL_EFUSE_CTRL_PRG_EN_POS              (1UL)
#define SYS_CTRL_EFUSE_CTRL_PRG_EN_MASK             (0x1UL << SYS_CTRL_EFUSE_CTRL_PRG_EN_POS)
#define SYS_CTRL_EFUSE_CTRL_PRG_EN                  SYS_CTRL_EFUSE_CTRL_PRG_EN_MASK
#define SYS_CTRL_EFUSE_CTRL_ADDR_EN_POS             (2UL)
#define SYS_CTRL_EFUSE_CTRL_ADDR_EN_MASK            (0x1UL << SYS_CTRL_EFUSE_CTRL_ADDR_EN_POS)
#define SYS_CTRL_EFUSE_CTRL_ADDR_EN                 SYS_CTRL_EFUSE_CTRL_ADDR_EN_MASK
#define SYS_CTRL_EFUSE_CTRL_DONE_POS                (3UL)
#define SYS_CTRL_EFUSE_CTRL_DONE_MASK               (0x1UL << SYS_CTRL_EFUSE_CTRL_DONE_POS)
#define SYS_CTRL_EFUSE_CTRL_DONE                    SYS_CTRL_EFUSE_CTRL_DONE_MASK
#define SYS_CTRL_EFUSE_CTRL_PRG_LOCK_POS            (4UL)
#define SYS_CTRL_EFUSE_CTRL_PRG_LOCK_MASK           (0x1UL << SYS_CTRL_EFUSE_CTRL_PRG_LOCK_POS)
#define SYS_CTRL_EFUSE_CTRL_PRG_LOCK                SYS_CTRL_EFUSE_CTRL_PRG_LOCK_MASK

/**
 *@brief  EFUSE Address Bits Definition
 */
#define SYS_CTRL_EFUSE_ADDR_OFFSET            	0x60 

/**
 *@brief  EFUSE Address Assert Length Bits Definition
 */
#define SYS_CTRL_EFUSE_AEN_LEN_OFFSET         	0x64 
#define SYS_CTRL_EFUSE_AEN_LEN_HIGH_POS         (0UL)
#define SYS_CTRL_EFUSE_AEN_LEN_HIGH_MASK        (0xFFFFUL << SYS_CTRL_EFUSE_AEN_LEN_HIGH_POS)
#define SYS_CTRL_EFUSE_AEN_LEN_HIGH             SYS_CTRL_EFUSE_AEN_LEN_HIGH_MASK
#define SYS_CTRL_EFUSE_AEN_LEN_LOW_POS          (16UL)
#define SYS_CTRL_EFUSE_AEN_LEN_LOW_MASK         (0xFFFFUL << SYS_CTRL_EFUSE_AEN_LEN_LOW_POS)
#define SYS_CTRL_EFUSE_AEN_LEN_LOW              SYS_CTRL_EFUSE_AEN_LEN_LOW_MASK

/**
 *@brief  EFUSE Power Control Bits Definition
 */
#define SYS_CTRL_EFUSE_PWR_CTRL_OFFSET        	    0x68
#define SYS_CTRL_EFUSE_PWR_CTRL_DVDD_EN_POS         (0UL)
#define SYS_CTRL_EFUSE_PWR_CTRL_DVDD_EN_MASK        (0x1UL << SYS_CTRL_EFUSE_PWR_CTRL_DVDD_EN_POS)
#define SYS_CTRL_EFUSE_PWR_CTRL_DVDD_EN             SYS_CTRL_EFUSE_PWR_CTRL_DVDD_EN_MASK
#define SYS_CTRL_EFUSE_PWR_CTRL_AVDD_EN_POS         (1UL)
#define SYS_CTRL_EFUSE_PWR_CTRL_AVDD_EN_MASK        (0x1UL << SYS_CTRL_EFUSE_PWR_CTRL_AVDD_EN_POS)
#define SYS_CTRL_EFUSE_PWR_CTRL_AVDD_EN             SYS_CTRL_EFUSE_PWR_CTRL_AVDD_EN_MASK

/**
 *@brief  EFUSE DVDD/AVDD Enable max time Bits Definition
 */
#define SYS_CTRL_EFUSE_PWR_LEN_OFFSET         	0x6C

/**
 *@brief  EFUSE Access Key Bits Definition
 */
#define SYS_CTRL_EFUSE_KEY_OFFSET             	0x70 

/**
 *@brief  EFUSE Read Data Bits Definition
 */
#define SYS_CTRL_EFUSE_RDATA_OFFSET           	0x74 

/**
 *@brief  EFUSE Auto Load enable, from sysctrl power Ctrl Bits Definition
 */
#define SYS_CTRL_EFUSE_LOAD_EN_OFFSET         	0x78 
#define SYS_CTRL_EFUSE_LOAD_EN_POS              (0UL)
#define SYS_CTRL_EFUSE_LOAD_EN_MASK             (0x1UL << SYS_CTRL_EFUSE_LOAD_EN_POS)
#define SYS_CTRL_EFUSE_LOAD_EN                  SYS_CTRL_EFUSE_LOAD_EN_MASK
/**
 *@brief  SYS_CTRL_CLK_DIV_TOG
 */
#define SYS_CTRL_DIV_TOG_OFFSET               	0x7c
#define SYS_CTRL_I2S0_MDIV_TOG_POS   			(0UL)
#define SYS_CTRL_I2S0_MDIV_TOG_MASK      		(0x1UL << SYS_CTRL_I2S0_MDIV_TOG_POS)
#define SYS_CTRL_I2S0_MDIV_TOG					SYS_CTRL_I2S0_MDIV_TOG_MASK
#define SYS_CTRL_I2S1_MDIV_TOG_POS              (1UL)
#define SYS_CTRL_I2S1_MDIV_TOG_MASK      		(0x1UL << SYS_CTRL_I2S1_MDIV_TOG_POS)
#define SYS_CTRL_I2S1_MDIV_TOG					SYS_CTRL_I2S1_MDIV_TOG_MASK
#define SYS_CTRL_I2S0_DIV_TOG_POS               (2UL)
#define SYS_CTRL_I2S0_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_I2S0_DIV_TOG_POS)
#define SYS_CTRL_I2S0_DIV_TOG					SYS_CTRL_I2S0_DIV_TOG_MASK
#define SYS_CTRL_I2S1_DIV_TOG_POS               (3UL)
#define SYS_CTRL_I2S1_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_I2S1_DIV_TOG_POS)
#define SYS_CTRL_I2S1_DIV_TOG					SYS_CTRL_I2S1_DIV_TOG_MASK
#define SYS_CTRL_I2C0_DIV_TOG_POS               (4UL)
#define SYS_CTRL_I2C0_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_I2C0_DIV_TOG_POS)
#define SYS_CTRL_I2C0_DIV_TOG					SYS_CTRL_I2C0_DIV_TOG_MASK
#define SYS_CTRL_I2C1_DIV_TOG_POS               (5UL)
#define SYS_CTRL_I2C1_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_I2C1_DIV_TOG_POS)
#define SYS_CTRL_I2C1_DIV_TOG					SYS_CTRL_I2C1_DIV_TOG_MASK
#define SYS_CTRL_UART0_DIV_TOG_POS              (6UL)
#define SYS_CTRL_UART0_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_UART0_DIV_TOG_POS)
#define SYS_CTRL_UART0_DIV_TOG					SYS_CTRL_UART0_DIV_TOG_MASK
#define SYS_CTRL_UART1_DIV_TOG_POS              (7UL)
#define SYS_CTRL_UART1_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_UART1_DIV_TOG_POS)
#define SYS_CTRL_UART1_DIV_TOG					SYS_CTRL_UART1_DIV_TOG_MASK
#define SYS_CTRL_SPI0_DIV_TOG_POS               (8UL)
#define SYS_CTRL_SPI0_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_SPI0_DIV_TOG_POS)
#define SYS_CTRL_SPI0_DIV_TOG					SYS_CTRL_SPI0_DIV_TOG_MASK
#define SYS_CTRL_SPI1_DIV_TOG_POS               (9UL)
#define SYS_CTRL_SPI1_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_SPI1_DIV_TOG_POS)
#define SYS_CTRL_SPI1_DIV_TOG					SYS_CTRL_SPI1_DIV_TOG_MASK
#define SYS_CTRL_TIMER0_DIV_TOG_POS             (10UL)
#define SYS_CTRL_TIMER0_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_TIMER0_DIV_TOG_POS)
#define SYS_CTRL_TIMER0_DIV_TOG					SYS_CTRL_TIMER0_DIV_TOG_MASK
#define SYS_CTRL_TIMER1_DIV_TOG_POS             (11UL)
#define SYS_CTRL_TIMER1_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_TIMER1_DIV_TOG_POS)
#define SYS_CTRL_TIMER1_DIV_TOG					SYS_CTRL_TIMER1_DIV_TOG_MASK
#define SYS_CTRL_TIMER2_DIV_TOG_POS             (12UL)
#define SYS_CTRL_TIMER2_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_TIMER2_DIV_TOG_POS)
#define SYS_CTRL_TIMER2_DIV_TOG					SYS_CTRL_TIMER2_DIV_TOG_MASK
#define SYS_CTRL_TIMER3_DIV_TOG_POS             (13UL)
#define SYS_CTRL_TIMER3_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_TIMER3_DIV_TOG_POS)
#define SYS_CTRL_TIMER3_DIV_TOG					SYS_CTRL_TIMER3_DIV_TOG_MASK
#define SYS_CTRL_TIMER4_DIV_TOG_POS             (14UL)
#define SYS_CTRL_TIMER4_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_TIMER4_DIV_TOG_POS)
#define SYS_CTRL_TIMER4_DIV_TOG					SYS_CTRL_TIMER4_DIV_TOG_MASK
#define SYS_CTRL_TIMER5_DIV_TOG_POS             (15UL)
#define SYS_CTRL_TIMER5_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_TIMER5_DIV_TOG_POS)
#define SYS_CTRL_TIMER5_DIV_TOG					SYS_CTRL_TIMER5_DIV_TOG_MASK
#define SYS_CTRL_TIMER6_DIV_TOG_POS             (16UL)
#define SYS_CTRL_TIMER6_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_TIMER6_DIV_TOG_POS)
#define SYS_CTRL_TIMER6_DIV_TOG					SYS_CTRL_TIMER6_DIV_TOG_MASK
#define SYS_CTRL_TIMER7_DIV_TOG_POS             (17UL)
#define SYS_CTRL_TIMER7_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_TIMER7_DIV_TOG_POS)
#define SYS_CTRL_TIMER7_DIV_TOG					SYS_CTRL_TIMER7_DIV_TOG_MASK
#define SYS_CTRL_KSCAN_DIV_TOG_POS              (18UL)
#define SYS_CTRL_KSCAN_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_KSCAN_DIV_TOG_POS)
#define SYS_CTRL_KSCAN_DIV_TOG					SYS_CTRL_KSCAN_DIV_TOG_MASK
#define SYS_CTRL_PWM_DIV_TOG_POS                (19UL)
#define SYS_CTRL_PWM_DIV_TOG_MASK      			(0x1UL << SYS_CTRL_PWM_DIV_TOG_POS)
#define SYS_CTRL_PWM_DIV_TOG					SYS_CTRL_PWM_DIV_TOG_MASK
#define SYS_CTRL_BASEBAND_DIV_TOG_POS           (20UL)
#define SYS_CTRL_BASEBAND_DIV_TOG_MASK      	(0x1UL << SYS_CTRL_BASEBAND_DIV_TOG_POS)
#define SYS_CTRL_BASEBAND_DIV_TOG				SYS_CTRL_BASEBAND_DIV_TOG_MASK
#define SYS_CTRL_IR_DIV_TOG_POS                 (21UL)
#define SYS_CTRL_IR_DIV_TOG_MASK      			(0x1UL << SYS_CTRL_IR_DIV_TOG_POS)
#define SYS_CTRL_IR_DIV_TOG						SYS_CTRL_IR_DIV_TOG_MASK
#define SYS_CTRL_CALI_DIV_TOG_POS               (22UL)
#define SYS_CTRL_CALI_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_CALI_DIV_TOG_POS)
#define SYS_CTRL_CALI_DIV_TOG					SYS_CTRL_CALI_DIV_TOG_MASK
#define SYS_CTRL_QSPI_DIV_TOG_POS               (23UL)
#define SYS_CTRL_QSPI_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_QSPI_DIV_TOG_POS)
#define SYS_CTRL_QSPI_DIV_TOG					SYS_CTRL_QSPI_DIV_TOG_MASK
#define SYS_CTRL_GPIO_DIV_TOG_POS               (24UL)
#define SYS_CTRL_GPIO_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_GPIO_DIV_TOG_POS)
#define SYS_CTRL_GPIO_DIV_TOG					SYS_CTRL_GPIO_DIV_TOG_MASK

#define SYS_CTRL_PERIPRE_DIV_TOG_POS            (25UL)
#define SYS_CTRL_PERIPRE_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_PERIPRE_DIV_TOG_POS)
#define SYS_CTRL_PERIPRE_DIV_TOG			    SYS_CTRL_PERIPRE_DIV_TOG_MASK

#define SYS_CTRL_MDM0_DIV_TOG_POS               (26UL)
#define SYS_CTRL_MDM0_DIV_TOG_MASK      		(0x1UL << SYS_CTRL_MDM0_DIV_TOG_POS)
#define SYS_CTRL_MDM0_DIV_TOG					 SYS_CTRL_MDM0_DIV_TOG_MASK


/**
 *@brief  Power Down Mode Enable Control Bits Definition
 */
#define SYS_CTRL_RAM_CTRL_OFFSET              				0x80
#define SYS_CTRL_RAM_CTRL_SRAM0_SD_EN_POS          		    (0UL)
#define SYS_CTRL_RAM_CTRL_SRAM0_SD_EN_MASK      		    (0x1UL << SYS_CTRL_RAM_CTRL_SRAM0_SD_EN_POS)
#define SYS_CTRL_RAM_CTRL_SRAM0_SD_EN					    SYS_CTRL_RAM_CTRL_SRAM0_SD_EN_MASK
#define SYS_CTRL_RAM_CTRL_SRAM1_SD_EN_POS                   (1UL)
#define SYS_CTRL_RAM_CTRL_SRAM1_SD_EN_MASK                  (0x1UL << SYS_CTRL_RAM_CTRL_SRAM1_SD_EN_POS)
#define SYS_CTRL_RAM_CTRL_SRAM1_SD_EN                       SYS_CTRL_RAM_CTRL_SRAM1_SD_EN_MASK
#define SYS_CTRL_RAM_CTRL_SRAM2_SD_EN_POS                   (2UL)
#define SYS_CTRL_RAM_CTRL_SRAM2_SD_EN_MASK                  (0x1UL << SYS_CTRL_RAM_CTRL_SRAM2_SD_EN_POS)
#define SYS_CTRL_RAM_CTRL_SRAM2_SD_EN                       SYS_CTRL_RAM_CTRL_SRAM2_SD_EN_MASK
#define SYS_CTRL_RAM_CTRL_RETRAM_SD_EN_POS           		(3UL)
#define SYS_CTRL_RAM_CTRL_RETRAM_SD_EN_MASK      			(0x1UL << SYS_CTRL_RAM_CTRL_RETRAM_SD_EN_POS)
#define SYS_CTRL_RAM_CTRL_RETRAM_SD_EN					    SYS_CTRL_RAM_CTRL_RETRAM_SD_EN_MASK
#define SYS_CTRL_RAM_CTRL_EM_NRM_SD_EN_POS                  (4UL)
#define SYS_CTRL_RAM_CTRL_EM_NRM_SD_EN_MASK                 (0x1UL << SYS_CTRL_RAM_CTRL_EM_NRM_SD_EN_POS)
#define SYS_CTRL_RAM_CTRL_EM_NRM_SD_EN                      SYS_CTRL_RAM_CTRL_EM_NRM_SD_EN_MASK
#define SYS_CTRL_RAM_CTRL_EM_RET_SD_EN_POS                  (5UL)
#define SYS_CTRL_RAM_CTRL_EM_RET_SD_EN_MASK                 (0x1UL << SYS_CTRL_RAM_CTRL_EM_RET_SD_EN_POS)
#define SYS_CTRL_RAM_CTRL_EM_RET_SD_EN                      SYS_CTRL_RAM_CTRL_EM_RET_SD_EN_MASK
#define SYS_CTRL_RAM_CTRL_SRAM0_DS_EN_POS                   (16UL)
#define SYS_CTRL_RAM_CTRL_SRAM0_DS_EN_MASK                  (0x1UL << SYS_CTRL_RAM_CTRL_SRAM0_DS_EN_POS)
#define SYS_CTRL_RAM_CTRL_SRAM0_DS_EN                       SYS_CTRL_RAM_CTRL_SRAM0_DS_EN_MASK
#define SYS_CTRL_RAM_CTRL_SRAM1_DS_EN_POS                   (17UL)
#define SYS_CTRL_RAM_CTRL_SRAM1_DS_EN_MASK                  (0x1UL << SYS_CTRL_RAM_CTRL_SRAM1_DS_EN_POS)
#define SYS_CTRL_RAM_CTRL_SRAM1_DS_EN                       SYS_CTRL_RAM_CTRL_SRAM1_DS_EN_MASK
#define SYS_CTRL_RAM_CTRL_SRAM2_DS_EN_POS                   (18UL)
#define SYS_CTRL_RAM_CTRL_SRAM2_DS_EN_MASK                  (0x1UL << SYS_CTRL_RAM_CTRL_SRAM2_DS_EN_POS)
#define SYS_CTRL_RAM_CTRL_SRAM2_DS_EN                       SYS_CTRL_RAM_CTRL_SRAM2_DS_EN_MASK
#define SYS_CTRL_RAM_CTRL_RETRAM_DS_EN_POS                  (19UL)
#define SYS_CTRL_RAM_CTRL_RETRAM_DS_EN_MASK                 (0x1UL << SYS_CTRL_RAM_CTRL_RETRAM_DS_EN_POS)
#define SYS_CTRL_RAM_CTRL_RETRAM_DS_EN                      SYS_CTRL_RAM_CTRL_RETRAM_DS_EN_MASK
#define SYS_CTRL_RAM_CTRL_EM_NRM_DS_EN_POS                  (20UL)
#define SYS_CTRL_RAM_CTRL_EM_NRM_DS_EN_MASK                 (0x1UL << SYS_CTRL_RAM_CTRL_EM_NRM_DS_EN_POS)
#define SYS_CTRL_RAM_CTRL_EM_NRM_DS_EN                      SYS_CTRL_RAM_CTRL_EM_NRM_DS_EN_MASK
#define SYS_CTRL_RAM_CTRL_EM_RET_DS_EN_POS                  (21UL)
#define SYS_CTRL_RAM_CTRL_EM_RET_DS_EN_MASK                 (0x1UL << SYS_CTRL_RAM_CTRL_EM_RET_DS_EN_POS)
#define SYS_CTRL_RAM_CTRL_EM_RET_DS_EN                      SYS_CTRL_RAM_CTRL_EM_RET_DS_EN_MASK

/**
 *@brief  SYS_CTRL_PAD_PLL_ENABLE_REG
 */
#define SYS_CTRL_PAD_EN_OFFSET               				0x84

#define SYS_CTRL_PAD_JTAG_KEEP_ENABLE_POS                   (27UL)
#define SYS_CTRL_PAD_JTAG_KEEP_ENABLE_MASK                  (0x1UL << SYS_CTRL_PAD_JTAG_KEEP_ENABLE_POS)
#define SYS_CTRL_PAD_JTAG_KEEP_ENABLE                       SYS_CTRL_PAD_JTAG_KEEP_ENABLE_MASK
/**
 *@brief  SYS_CTRL_OSC_CFG_REG
 */
//#define SYS_CTRL_OSC_CFG_OFFSET               				0x88
//#define SYS_CTRL_OSC_CFG_OSC16M_TRIM_EN_POS          		(0UL)
//#define SYS_CTRL_OSC_CFG_OSC16M_TRIM_EN_MASK      			(0x1UL << SYS_CTRL_OSC_CFG_OSC16M_TRIM_EN_POS)
//#define SYS_CTRL_OSC_CFG_OSC16M_TRIM_EN						SYS_CTRL_OSC_CFG_OSC16M_TRIM_EN_MASK
//#define SYS_CTRL_OSC_CFG_OSC16M_IOP_SEL_POS          		(1UL)
//#define SYS_CTRL_OSC_CFG_OSC16M_IOP_SEL_MASK      			(0x1UL << SYS_CTRL_OSC_CFG_OSC16M_IOP_SEL_POS)
//#define SYS_CTRL_OSC_CFG_OSC16M_IOP_SEL						SYS_CTRL_OSC_CFG_OSC16M_IOP_SEL_MASK
//#define SYS_CTRL_OSC_CFG_OSC16M_FREQ_SEL_POS         		(2UL)
//#define SYS_CTRL_OSC_CFG_OSC16M_FREQ_SEL_MASK      			(0x7UL << SYS_CTRL_OSC_CFG_OSC16M_FREQ_SEL_POS)
//#define SYS_CTRL_OSC_CFG_OSC16M_FREQ_SEL					SYS_CTRL_OSC_CFG_OSC16M_FREQ_SEL_MASK
//#define SYS_CTRL_OSC_CFG_OSC16M_FREQ_CTUNE_POS       		(5UL)
//#define SYS_CTRL_OSC_CFG_OSC16M_FREQ_CTUNE_MASK      		(0x3FUL << SYS_CTRL_OSC_CFG_OSC16M_FREQ_CTUNE_POS)
//#define SYS_CTRL_OSC_CFG_OSC16M_FREQ_CTUNE					SYS_CTRL_OSC_CFG_OSC16M_FREQ_CTUNE_MASK
//#define SYS_CTRL_OSC_CFG_OSC16M_FREQ_FTUNE_POS       		(11UL)
//#define SYS_CTRL_OSC_CFG_OSC16M_FREQ_FTUNE_MASK      		(0x3FUL << SYS_CTRL_OSC_CFG_OSC16M_FREQ_FTUNE_POS)
//#define SYS_CTRL_OSC_CFG_OSC16M_FREQ_FTUNE					SYS_CTRL_OSC_CFG_OSC16M_FREQ_FTUNE_MASK
//#define SYS_CTRL_OSC_CFG_OSC16M_REG_STBDLY_POS       		(17UL)
//#define SYS_CTRL_OSC_CFG_OSC16M_REG_STBDLY_MASK      		(0x1UL << SYS_CTRL_OSC_CFG_OSC16M_REG_STBDLY_POS)
//#define SYS_CTRL_OSC_CFG_OSC16M_REG_STBDLY					SYS_CTRL_OSC_CFG_OSC16M_REG_STBDLY_MASK
//#define SYS_CTRL_OSC_CFG_OSC16M_TRIM_DONE_POS        		(18UL)
//#define SYS_CTRL_OSC_CFG_OSC16M_TRIM_DONE_MASK      		(0x1UL << SYS_CTRL_OSC_CFG_OSC16M_TRIM_DONE_POS)
//#define SYS_CTRL_OSC_CFG_OSC16M_TRIM_DONE					SYS_CTRL_OSC_CFG_OSC16M_TRIM_DONE_MASK
//#define SYS_CTRL_OSC_CFG_OSC16M_CLK_STABLE_POS      		(19UL)
//#define SYS_CTRL_OSC_CFG_OSC16M_CLK_STABLE_MASK      		(0x1UL << SYS_CTRL_OSC_CFG_OSC16M_CLK_STABLE_POS)
//#define SYS_CTRL_OSC_CFG_OSC16M_CLK_STABLE					SYS_CTRL_OSC_CFG_OSC16M_CLK_STABLE_MASK

// RESERVED REG
#define SYS_CTRL_SOFT_FLAG_OFFSET              				0x8c
#define SYS_CTRL_SOFT_FLAG_BLE_WKUP_POS                     (0UL)
#define SYS_CTRL_SOFT_FLAG_BLE_WKUP_MASK                    (0x1UL << SYS_CTRL_SOFT_FLAG_BLE_WKUP_POS)
#define SYS_CTRL_SOFT_FLAG_BLE_WKUP                         SYS_CTRL_SOFT_FLAG_BLE_WKUP_MASK
/**
 *@brief   SOFT RST REG
 */
#define SYS_CTRL_SOFT_RST_OFFSET            				0x90
#define SYS_CTRL_SOFT_RST_POS               				(0UL)
#define SYS_CTRL_SOFT_RST_MASK      						(0x1UL << SYS_CTRL_SOFT_RST_POS)
#define SYS_CTRL_SOFT_RST									SYS_CTRL_SOFT_RST_MASK
#define SYS_CTRL_SOFT_RST_PWM_POS           				(1UL)
#define SYS_CTRL_SOFT_RST_PWM_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_PWM_POS)
#define SYS_CTRL_SOFT_RST_PWM								SYS_CTRL_SOFT_RST_PWM_MASK
#define SYS_CTRL_SOFT_RST_BASEBAND_POS      				(2UL)
#define SYS_CTRL_SOFT_RST_BASEBAND_MASK     				(0x1UL << SYS_CTRL_SOFT_RST_BASEBAND_POS)
#define SYS_CTRL_SOFT_RST_BASEBAND							SYS_CTRL_SOFT_RST_BASEBAND_MASK
#define SYS_CTRL_SOFT_RST_QSPI_POS          				(3UL)
#define SYS_CTRL_SOFT_RST_QSPI_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_QSPI_POS)
#define SYS_CTRL_SOFT_RST_QSPI								SYS_CTRL_SOFT_RST_QSPI_MASK
#define SYS_CTRL_SOFT_RST_TIMER0_POS        				(4UL)
#define SYS_CTRL_SOFT_RST_TIMER0_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_TIMER0_POS)
#define SYS_CTRL_SOFT_RST_TIMER0							SYS_CTRL_SOFT_RST_TIMER0_MASK
#define SYS_CTRL_SOFT_RST_TIMER1_POS        				(5UL)
#define SYS_CTRL_SOFT_RST_TIMER1_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_TIMER1_POS)
#define SYS_CTRL_SOFT_RST_TIMER1							SYS_CTRL_SOFT_RST_TIMER1_MASK
#define SYS_CTRL_SOFT_RST_TIMER2_POS        				(6UL)
#define SYS_CTRL_SOFT_RST_TIMER2_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_TIMER2_POS)
#define SYS_CTRL_SOFT_RST_TIMER2							SYS_CTRL_SOFT_RST_TIMER2_MASK
#define SYS_CTRL_SOFT_RST_TIMER3_POS        				(7UL)
#define SYS_CTRL_SOFT_RST_TIMER3_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_TIMER3_POS)
#define SYS_CTRL_SOFT_RST_TIMER3							SYS_CTRL_SOFT_RST_TIMER3_MASK
#define SYS_CTRL_SOFT_RST_TIMER4_POS        				(8UL)
#define SYS_CTRL_SOFT_RST_TIMER4_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_TIMER4_POS)
#define SYS_CTRL_SOFT_RST_TIMER4							SYS_CTRL_SOFT_RST_TIMER4_MASK
#define SYS_CTRL_SOFT_RST_TIMER5_POS        				(9UL)
#define SYS_CTRL_SOFT_RST_TIMER5_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_TIMER5_POS)
#define SYS_CTRL_SOFT_RST_TIMER5							SYS_CTRL_SOFT_RST_TIMER5_MASK
#define SYS_CTRL_SOFT_RST_TIMER6_POS        				(10UL)
#define SYS_CTRL_SOFT_RST_TIMER6_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_TIMER6_POS)
#define SYS_CTRL_SOFT_RST_TIMER6							SYS_CTRL_SOFT_RST_TIMER6_MASK
#define SYS_CTRL_SOFT_RST_TIMER7_POS        				(11UL)
#define SYS_CTRL_SOFT_RST_TIMER7_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_TIMER7_POS)
#define SYS_CTRL_SOFT_RST_TIMER7							SYS_CTRL_SOFT_RST_TIMER7_MASK
#define SYS_CTRL_SOFT_RST_IR_POS            				(12UL)
#define SYS_CTRL_SOFT_RST_IR_MASK      						(0x1UL << SYS_CTRL_SOFT_RST_IR_POS)
#define SYS_CTRL_SOFT_RST_IR								SYS_CTRL_SOFT_RST_IR_MASK
#define SYS_CTRL_SOFT_RST_KSCAN_POS         				(13UL)
#define SYS_CTRL_SOFT_RST_KSCAN_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_KSCAN_POS)
#define SYS_CTRL_SOFT_RST_KSCAN								SYS_CTRL_SOFT_RST_KSCAN_MASK
#define SYS_CTRL_SOFT_RST_RTC_POS           				(14UL)
#define SYS_CTRL_SOFT_RST_RTC_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_RTC_POS)
#define SYS_CTRL_SOFT_RST_RTC								SYS_CTRL_SOFT_RST_RTC_MASK
#define SYS_CTRL_SOFT_RST_UART0_POS         				(15UL)
#define SYS_CTRL_SOFT_RST_UART0_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_UART0_POS)
#define SYS_CTRL_SOFT_RST_UART0								SYS_CTRL_SOFT_RST_UART0_MASK
#define SYS_CTRL_SOFT_RST_UART1_POS         				(16UL)
#define SYS_CTRL_SOFT_RST_UART1_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_UART1_POS)
#define SYS_CTRL_SOFT_RST_UART1								SYS_CTRL_SOFT_RST_UART1_MASK
#define SYS_CTRL_SOFT_RST_UART2_POS         				(17UL)
#define SYS_CTRL_SOFT_RST_UART2_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_UART2_POS)
#define SYS_CTRL_SOFT_RST_UART2								SYS_CTRL_SOFT_RST_UART2_MASK
#define SYS_CTRL_SOFT_RST_GPIO_POS          				(18UL)
#define SYS_CTRL_SOFT_RST_GPIO_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_GPIO_POS)
#define SYS_CTRL_SOFT_RST_GPIO								SYS_CTRL_SOFT_RST_GPIO_MASK
#define SYS_CTRL_SOFT_RST_SPI0_POS          				(19UL)
#define SYS_CTRL_SOFT_RST_SPI0_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_SPI0_POS)
#define SYS_CTRL_SOFT_RST_SPI0								SYS_CTRL_SOFT_RST_SPI0_MASK
#define SYS_CTRL_SOFT_RST_SPI1_POS          				(20UL)
#define SYS_CTRL_SOFT_RST_SPI1_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_SPI1_POS)
#define SYS_CTRL_SOFT_RST_SPI1								SYS_CTRL_SOFT_RST_SPI1_MASK
#define SYS_CTRL_SOFT_RST_I2C0_POS          				(21UL)
#define SYS_CTRL_SOFT_RST_I2C0_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_I2C0_POS)
#define SYS_CTRL_SOFT_RST_I2C0								SYS_CTRL_SOFT_RST_I2C0_MASK
#define SYS_CTRL_SOFT_RST_I2C1_POS          				(22UL)
#define SYS_CTRL_SOFT_RST_I2C1_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_I2C1_POS)
#define SYS_CTRL_SOFT_RST_I2C1								SYS_CTRL_SOFT_RST_I2C1_MASK
#define SYS_CTRL_SOFT_RST_I2S0_POS          				(23UL)
#define SYS_CTRL_SOFT_RST_I2S0_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_I2S0_POS)
#define SYS_CTRL_SOFT_RST_I2S0								SYS_CTRL_SOFT_RST_I2S0_MASK
#define SYS_CTRL_SOFT_RST_I2S1_POS          				(24UL)
#define SYS_CTRL_SOFT_RST_I2S1_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_I2S1_POS)
#define SYS_CTRL_SOFT_RST_I2S1								SYS_CTRL_SOFT_RST_I2S1_MASK
#define SYS_CTRL_SOFT_RST_WDT_POS   		 				(25UL)
#define SYS_CTRL_SOFT_RST_WDT_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_WDT_POS)
#define SYS_CTRL_SOFT_RST_WDT								SYS_CTRL_SOFT_RST_WDT_MASK
#define SYS_CTRL_SOFT_RST_TRNG_POS       	 				(26UL)
#define SYS_CTRL_SOFT_RST_TRNG_MASK      					(0x1UL << SYS_CTRL_SOFT_RST_TRNG_POS)
#define SYS_CTRL_SOFT_RST_TRNG								SYS_CTRL_SOFT_RST_TRNG_MASK
#define SYS_CTRL_SOFT_RST_MDM_POS                           (27UL)
#define SYS_CTRL_SOFT_RST_MDM_MASK                          (0x1UL << SYS_CTRL_SOFT_RST_MDM_POS)
#define SYS_CTRL_SOFT_RST_MDM                               SYS_CTRL_SOFT_RST_MDM_MASK
#define SYS_CTRL_SOFT_RST_AUD_ADC_POS                       (28UL)
#define SYS_CTRL_SOFT_RST_AUD_ADC_MASK                      (0x1UL << SYS_CTRL_SOFT_RST_AUD_ADC_POS)
#define SYS_CTRL_SOFT_RST_AUD_ADC                           SYS_CTRL_SOFT_RST_AUD_ADC_MASK
#define SYS_CTRL_SOFT_RST_AUX_ADC_POS                       (29UL)
#define SYS_CTRL_SOFT_RST_AUX_ADC_MASK                      (0x1UL << SYS_CTRL_SOFT_RST_AUX_ADC_POS)
#define SYS_CTRL_SOFT_RST_AUX_ADC                           SYS_CTRL_SOFT_RST_AUX_ADC_MASK

/**	
 *@brief   LOW PWR CONTROL REG
 */
#define SYS_CTRL_LP_KEEP_OFFSET               				0x94
#define SYS_CTRL_LP_CLK_KEEP_POS              				(0UL)
#define SYS_CTRL_LP_CLK_KEEP_MASK      						(0x1UL << SYS_CTRL_LP_CLK_KEEP_POS)
#define SYS_CTRL_LP_CLK_KEEP								SYS_CTRL_LP_CLK_KEEP_MASK

#define SYS_CTRL_LP_RST_KEEP_POS              				(1UL)
#define SYS_CTRL_LP_RST_KEEP_MASK      						(0x1UL << SYS_CTRL_LP_RST_KEEP_POS)
#define SYS_CTRL_LP_RST_KEEP								SYS_CTRL_LP_RST_KEEP_MASK

#define SYS_CTRL_LP_PWR_KEEP_POS              				(2UL)
#define SYS_CTRL_LP_PWR_KEEP_MASK      						(0x1UL << SYS_CTRL_LP_PWR_KEEP_POS)
#define SYS_CTRL_LP_PWR_KEEP								SYS_CTRL_LP_PWR_KEEP_MASK

#define SYS_CTRL_LP_RST_MASK_POS              				(3UL)
#define SYS_CTRL_LP_RST_MASK_MASK      						(0x1UL << SYS_CTRL_LP_RST_MASK_POS)
#define SYS_CTRL_LP_RST_MASK								SYS_CTRL_LP_RST_MASK_MASK

#define SYS_CTRL_PUP_CLK_BYPASS_POS                         (4UL)
#define SYS_CTRL_PUP_CLK_BYPASS_MASK                        (0x1UL << SYS_CTRL_PUP_CLK_BYPASS_POS)
#define SYS_CTRL_PUP_CLK_BYPASS                             SYS_CTRL_PUP_CLK_BYPASS_MASK

#define SYS_CTRL_PMU_RC_EN_POS                              (5UL)
#define SYS_CTRL_PMU_RC_EN_MASK                             (0x1UL << SYS_CTRL_PMU_RC_EN_POS)
#define SYS_CTRL_PMU_RC_EN                                  SYS_CTRL_PMU_RC_EN_MASK

#define SYS_CTRL_BLE_WKUP_DIS_POS                           (6UL)
#define SYS_CTRL_BLE_WKUP_DIS_MASK                          (0x1UL << SYS_CTRL_BLE_WKUP_DIS_POS)
#define SYS_CTRL_BLE_WKUP_DIS                               SYS_CTRL_BLE_WKUP_DIS_MASK

#define SYS_CTRL_MAIN_LDO_EN_HW_VLD_POS                     (7UL)
#define SYS_CTRL_MAIN_LDO_EN_HW_VLD_MASK                    (0x1UL << SYS_CTRL_MAIN_LDO_EN_HW_VLD_POS)
#define SYS_CTRL_MAIN_LDO_EN_HW_VLD                         SYS_CTRL_MAIN_LDO_EN_HW_VLD_MASK

#define SYS_CTRL_RC16M_EN_HW_VLD_POS                        (8UL)
#define SYS_CTRL_RC16M_EN_HW_VLD_MASK                       (0x1UL << SYS_CTRL_RC16M_EN_HW_VLD_POS)
#define SYS_CTRL_RC16M_EN_HW_VLD                            SYS_CTRL_RC16M_EN_HW_VLD_MASK

#define SYS_CTRL_PLL_EN_HW_VLD_POS                          (9UL)
#define SYS_CTRL_PLL_EN_HW_VLD_MASK                         (0x1UL << SYS_CTRL_PLL_EN_HW_VLD_POS)
#define SYS_CTRL_PLL_EN_HW_VLD                              SYS_CTRL_PLL_EN_HW_VLD_MASK

#define SYS_CTRL_RF_MODE_HW_VLD_POS                         (10UL)
#define SYS_CTRL_RF_MODE_HW_VLD_MASK                        (0x1UL << SYS_CTRL_RF_MODE_HW_VLD_POS)
#define SYS_CTRL_RF_MODE_HW_VLD                             SYS_CTRL_RF_MODE_HW_VLD_MASK

#define SYS_CTRL_MAIN_LDO_EN_SW_POS                         (11UL)
#define SYS_CTRL_MAIN_LDO_EN_SW_MASK                        (0x1UL << SYS_CTRL_MAIN_LDO_EN_SW_POS)
#define SYS_CTRL_MAIN_LDO_EN_SW                             SYS_CTRL_MAIN_LDO_EN_SW_MASK

#define SYS_CTRL_RC16M_EN_SW_POS                            (12UL)
#define SYS_CTRL_RC16M_EN_SW_MASK                           (0x1UL << SYS_CTRL_RC16M_EN_SW_POS)
#define SYS_CTRL_RC16M_EN_SW                                SYS_CTRL_RC16M_EN_SW_MASK

#define SYS_CTRL_PLL_EN_SW_POS                              (13UL)
#define SYS_CTRL_PLL_EN_SW_MASK                             (0x1UL << SYS_CTRL_PLL_EN_SW_POS)
#define SYS_CTRL_PLL_EN_SW                                  SYS_CTRL_PLL_EN_SW_MASK

#define SYS_CTRL_RF_MODE_SW_POS                             (14UL)
#define SYS_CTRL_RF_MODE_SW_MASK                            (0x1FUL << SYS_CTRL_RF_MODE_SW_POS)
#define SYS_CTRL_RF_MODE_SW                                 SYS_CTRL_RF_MODE_SW_MASK

#define SYS_CTRL_MEM_SD_EN_HW_MASK_POS                      (19UL)
#define SYS_CTRL_MEM_SD_EN_HW_MASK_MASK                     (0x1UL << SYS_CTRL_MEM_SD_EN_HW_MASK_POS)
#define SYS_CTRL_MEM_SD_EN_HW_MASK                          SYS_CTRL_MEM_SD_EN_HW_MASK_MASK

#define SYS_CTRL_MEM_DS_EN_HW_MASK_POS                      (20UL)
#define SYS_CTRL_MEM_DS_EN_HW_MASK_MASK                     (0x1UL << SYS_CTRL_MEM_DS_EN_HW_MASK_POS)
#define SYS_CTRL_MEM_DS_EN_HW_MASK                          SYS_CTRL_MEM_DS_EN_HW_MASK_MASK

/**	
 *@brief    SYS_CTRL_CLK_DIV4	
 */
#define SYS_CTRL_CLK_DIV4_OFFSET              				0x98

/**
 *@brief    SYS_CTRL_CLK_DIV5
 */
#define SYS_CTRL_CLK_DIV5_OFFSET              				0x9c

// DFX
#define SYS_CTRL_DFX_SEL_OFFSET                             0xa0
#define SYS_CTRL_DFX_SEL0_POS                               (0UL)
#define SYS_CTRL_DFX_SEL0_MASK                              (0xFUL << SYS_CTRL_DFX_SEL0_POS)
#define SYS_CTRL_DFX_SEL0                                   SYS_CTRL_DFX_SEL0_MASK
#define SYS_CTRL_DFX_SEL1_POS                               (4UL)
#define SYS_CTRL_DFX_SEL1_MASK                              (0xFUL << SYS_CTRL_DFX_SEL1_POS)
#define SYS_CTRL_DFX_SEL1                                   SYS_CTRL_DFX_SEL1_MASK
#define SYS_CTRL_DFX_SEL2_POS                               (8UL)
#define SYS_CTRL_DFX_SEL2_MASK                              (0xFUL << SYS_CTRL_DFX_SEL2_POS)
#define SYS_CTRL_DFX_SEL2                                   SYS_CTRL_DFX_SEL2_MASK
#define SYS_CTRL_DFX_SEL3_POS                               (12UL)
#define SYS_CTRL_DFX_SEL3_MASK                              (0xFUL << SYS_CTRL_DFX_SEL3_POS)
#define SYS_CTRL_DFX_SEL3                                   SYS_CTRL_DFX_SEL3_MASK
#define SYS_CTRL_DFX_SEL4_POS                               (16UL)
#define SYS_CTRL_DFX_SEL4_MASK                              (0xFUL << SYS_CTRL_DFX_SEL4_POS)
#define SYS_CTRL_DFX_SEL4                                   SYS_CTRL_DFX_SEL4_MASK
#define SYS_CTRL_DFX_SEL5_POS                               (20UL)
#define SYS_CTRL_DFX_SEL5_MASK                              (0xFUL << SYS_CTRL_DFX_SEL5_POS)
#define SYS_CTRL_DFX_SEL5                                   SYS_CTRL_DFX_SEL5_MASK
#define SYS_CTRL_DFX_SEL6_POS                               (24UL)
#define SYS_CTRL_DFX_SEL6_MASK                              (0xFUL << SYS_CTRL_DFX_SEL6_POS)
#define SYS_CTRL_DFX_SEL6                                   SYS_CTRL_DFX_SEL6_MASK
#define SYS_CTRL_DFX_SEL7_POS                               (28UL)
#define SYS_CTRL_DFX_SEL7_MASK                              (0xFUL << SYS_CTRL_DFX_SEL7_POS)
#define SYS_CTRL_DFX_SEL7                                   SYS_CTRL_DFX_SEL7_MASK

// WKUP CSR
#define SYS_CTRL_WKUP_CSR_OFFSET                            0xa4
#define SYS_CTRL_WKUP_CSR_INT_EN_POS                        (0UL)
#define SYS_CTRL_WKUP_CSR_INT_EN_MASK                       (0x1UL << SYS_CTRL_WKUP_CSR_INT_EN_POS)
#define SYS_CTRL_WKUP_CSR_INT_EN                            SYS_CTRL_WKUP_CSR_INT_EN_MASK
#define SYS_CTRL_WKUP_CSR_GPIO_WAKUP_MASK_POS               (1UL)
#define SYS_CTRL_WKUP_CSR_GPIO_WAKUP_MASK_MASK              (0x1UL << SYS_CTRL_WKUP_CSR_GPIO_WAKUP_MASK_POS)
#define SYS_CTRL_WKUP_CSR_GPIO_WAKUP_MASK                   SYS_CTRL_WKUP_CSR_GPIO_WAKUP_MASK_MASK
#define SYS_CTRL_WKUP_CSR_UART2_WAKUP_MASK_POS              (2UL)
#define SYS_CTRL_WKUP_CSR_UART2_WAKUP_MASK_MASK             (0x1UL << SYS_CTRL_WKUP_CSR_UART2_WAKUP_MASK_POS)
#define SYS_CTRL_WKUP_CSR_UART2_WAKUP_MASK                  SYS_CTRL_WKUP_CSR_UART2_WAKUP_MASK_MASK
#define SYS_CTRL_WKUP_CSR_RTC_WAKUP_MASK_POS                (3UL)
#define SYS_CTRL_WKUP_CSR_RTC_WAKUP_MASK_MASK               (0x1UL << SYS_CTRL_WKUP_CSR_RTC_WAKUP_MASK_POS)
#define SYS_CTRL_WKUP_CSR_RTC_WAKUP_MASK                    SYS_CTRL_WKUP_CSR_RTC_WAKUP_MASK_MASK
#define SYS_CTRL_WKUP_CSR_GPIO_WAKUP_INT_POS                (4UL)
#define SYS_CTRL_WKUP_CSR_GPIO_WAKUP_INT_MASK               (0x1UL << SYS_CTRL_WKUP_CSR_GPIO_WAKUP_INT_POS)
#define SYS_CTRL_WKUP_CSR_GPIO_WAKUP_INT                    SYS_CTRL_WKUP_CSR_GPIO_WAKUP_INT_MASK
#define SYS_CTRL_WKUP_CSR_UART2_WAKUP_INT_POS               (5UL)
#define SYS_CTRL_WKUP_CSR_UART2_WAKUP_INT_MASK              (0x1UL << SYS_CTRL_WKUP_CSR_UART2_WAKUP_INT_POS)
#define SYS_CTRL_WKUP_CSR_UART2_WAKUP_INT                   SYS_CTRL_WKUP_CSR_UART2_WAKUP_INT_MASK
#define SYS_CTRL_WKUP_CSR_RTC_WAKUP_INT_POS                 (6UL)
#define SYS_CTRL_WKUP_CSR_RTC_WAKUP_INT_MASK                (0x1UL << SYS_CTRL_WKUP_CSR_RTC_WAKUP_INT_POS)
#define SYS_CTRL_WKUP_CSR_RTC_WAKUP_INT                     SYS_CTRL_WKUP_CSR_RTC_WAKUP_INT_MASK
#define SYS_CTRL_WKUP_CSR_BLE_WAKUP_MASK_POS                (7UL)
#define SYS_CTRL_WKUP_CSR_BLE_WAKUP_MASK_MASK               (0x1UL << SYS_CTRL_WKUP_CSR_BLE_WAKUP_MASK_POS)
#define SYS_CTRL_WKUP_CSR_BLE_WAKUP_MASK                    SYS_CTRL_WKUP_CSR_BLE_WAKUP_MASK_MASK
#define SYS_CTRL_WKUP_CSR_BLE_WAKUP_INT_POS                 (8UL)
#define SYS_CTRL_WKUP_CSR_BLE_WAKUP_INT_MASK                (0x1UL << SYS_CTRL_WKUP_CSR_BLE_WAKUP_INT_POS)
#define SYS_CTRL_WKUP_CSR_BLE_WAKUP_INT                     SYS_CTRL_WKUP_CSR_BLE_WAKUP_INT_MASK




// PAD CTRL REG
#define SYS_CTRL_PAD_PD_OFFSET                              0xac

/**	
 *@brief    ADC CTRL REG	
 */
#define SYS_CTRL_UNIT_EN_OFFSET               				0xb0
#define SYS_CTRL_UNIT_EN_AUD_ADC_PU_POS                 	(0UL)
#define SYS_CTRL_UNIT_EN_AUD_ADC_PU_MASK      				(0x1UL << SYS_CTRL_UNIT_EN_AUD_ADC_PU_POS)
#define SYS_CTRL_UNIT_EN_AUD_ADC_PU							SYS_CTRL_UNIT_EN_AUD_ADC_PU_MASK
#define SYS_CTRL_UNIT_EN_AUX_ADC_PU_POS                 	(1UL)
#define SYS_CTRL_UNIT_EN_AUX_ADC_PU_MASK      				(0x1UL << SYS_CTRL_UNIT_EN_AUX_ADC_PU_POS)
#define SYS_CTRL_UNIT_EN_AUX_ADC_PU							SYS_CTRL_UNIT_EN_AUX_ADC_PU_MASK
//#define SYS_CTRL_UNIT_EN_RC_OSC16M_EN_POS               	(2UL)
//#define SYS_CTRL_UNIT_EN_RC_OSC16M_EN_MASK      			(0x1UL << SYS_CTRL_UNIT_EN_RC_OSC16M_EN_POS)
//#define SYS_CTRL_UNIT_EN_RC_OSC16M_EN						SYS_CTRL_UNIT_EN_RC_OSC16M_EN_MASK
//#define SYS_CTRL_UNIT_EN_PLL_EN_POS                     	(3UL)
//#define SYS_CTRL_UNIT_EN_PLL_EN_MASK      					(0x1UL << SYS_CTRL_UNIT_EN_PLL_EN_POS)
//#define SYS_CTRL_UNIT_EN_PLL_EN								SYS_CTRL_UNIT_EN_PLL_EN_MASK
#define SYS_CTRL_UNIT_EN_EX_OSC16M_EN_POS                  	(4UL)
#define SYS_CTRL_UNIT_EN_EX_OSC16M_EN_MASK      			(0x1UL << SYS_CTRL_UNIT_EN_EX_OSC16M_EN_POS)
#define SYS_CTRL_UNIT_EN_EX_OSC16M_EN						SYS_CTRL_UNIT_EN_EX_OSC16M_EN_MASK
#define SYS_CTRL_UNIT_EN_EX_OSC32K_EN_POS                  	(5UL)
#define SYS_CTRL_UNIT_EN_EX_OSC32K_EN_MASK      			(0x1UL << SYS_CTRL_UNIT_EN_EX_OSC32K_EN_POS)
#define SYS_CTRL_UNIT_EN_EX_OSC32K_EN						SYS_CTRL_UNIT_EN_EX_OSC32K_EN_MASK

/**
 *@brief    CPU RESET VECTOR REGISTER
 */
#define SYS_CTRL_CPU_RESET_VECTOR_OFFSET      				0xb4

/**
 *@brief    BLE Control REGISTER
 */
#define SYS_CTRL_BLE_CTRL_OFFSET              				 0xb8
#define SYS_CTRL_BLE_CTRL_CLOCK_POS                         (0UL) 
#define SYS_CTRL_BLE_CTRL_CLOCK_POS_MASK                    (0x3FUL) << SYS_CTRL_BLE_CTRL_CLOCK_POS
#define SYS_CTRL_BLE_CTRL_CLOCK                              SYS_CTRL_BLE_CTRL_CLOCK_POS_MASK
#define SYS_CTRL_BLE_CTRL_RF_SEL_POS                         (6UL)
#define SYS_CTRL_BLE_CTRL_RF_SEL_POS_MASK                    (0x1UL << SYS_CTRL_BLE_CTRL_RF_SEL_POS)
#define SYS_CTRL_BLE_CTRL_RF_SEL                             SYS_CTRL_BLE_CTRL_RF_SEL_POS_MASK




/**
 *@brief    Clock div6 REGISTER
 */
#define SYS_CTRL_CLK_DIV6_OFFSET          				    0xbc
#define SYS_CTRL_CLK_DIV6_PERI_PRE_POS                      (0UL)
#define SYS_CTRL_CLK_DIV6_PERI_PRE_MASK                     (0xFUL << SYS_CTRL_CLK_DIV6_PERI_PRE_POS)
#define SYS_CTRL_CLK_DIV6_PERI_PRE                          SYS_CTRL_CLK_DIV6_PERI_PRE_MASK
#define SYS_CTRL_CLK_DIV6_MDM0_POS                          (4UL)
#define SYS_CTRL_CLK_DIV6_MDM0_MASK                         (0xFUL << SYS_CTRL_CLK_DIV6_MDM0_POS)
#define SYS_CTRL_CLK_DIV6_MDM0                              SYS_CTRL_CLK_DIV6_MDM0_MASK
#define SYS_CTRL_CLK_DIV6_BLE_POS                          (8UL)
#define SYS_CTRL_CLK_DIV6_BLE_MASK                         (0x1FUL << SYS_CTRL_CLK_DIV6_BLE_POS)
#define SYS_CTRL_CLK_DIV6_BLE                              SYS_CTRL_CLK_DIV6_BLE_MASK

/**
 *@brief CPU DEGUG REGISTER
 */
#define SYS_CTRL_CPU_DBG_OFFSET               				0xc0

/**
 *@brief PLL CONTROL REGISTER
 */
#define SYS_CTRL_PLL_CTRL1_OFFSET             				0xc4


/**
 *@brief PAD IS REGISTER
 */
#define SYS_CTRL_PAD_IS_OFFSET                				0xd0

/**
 *@brief PAD IS REGISTER
 */
#define SYS_CTRL_PAD_DR0_OFFSET               				0xd4

/**
 *@brief PAD IS REGISTER
 */
#define SYS_CTRL_PAD_DR1_OFFSET               				0xd8


/**
 *@brief PAD IS REGISTER
 */
#define SYS_CTRL_PAD_HV_CTL_OFFSET            				0xdc


/**	
 *@brief   OSC16M_CFG1	
 */
#define SYS_CTRL_OSC16M_CFG1_OFFSET                         0xc8
#define SYS_CTRL_OSC16M_CFG1_OSC16M_REF_CNT_POS  			(0UL)
#define SYS_CTRL_OSC16M_CFG1_OSC16M_REF_CNT_MASK      		(0xFFFFUL << SYS_CTRL_OSC16M_CFG1_OSC16M_REF_CNT_POS)
#define SYS_CTRL_OSC16M_CFG1_OSC16M_REF_CNT					SYS_CTRL_OSC16M_CFG1_OSC16M_REF_CNT_MASK

/**	
 *@brief   OSC16M_CFG2	
 */
#define SYS_CTRL_OSC16M_CFG2_OFFSET                         0xcc
#define SYS_CTRL_OSC16M_CFG2_FREQ_CTUNE_POS  				(0UL)
#define SYS_CTRL_OSC16M_CFG2_FREQ_CTUNE_MASK      			(0x3FUL << SYS_CTRL_OSC16M_CFG2_FREQ_CTUNE_POS)
#define SYS_CTRL_OSC16M_CFG2_FREQ_CTUNE						SYS_CTRL_OSC16M_CFG2_FREQ_CTUNE_MASK
#define SYS_CTRL_OSC16M_CFG2_FREQ_FTUNE_POS  				(6UL)
#define SYS_CTRL_OSC16M_CFG2_FREQ_FTUNE_MASK      			(0x3FUL << SYS_CTRL_OSC16M_CFG2_FREQ_FTUNE_POS)
#define SYS_CTRL_OSC16M_CFG2_FREQ_FTUNE						SYS_CTRL_OSC16M_CFG2_FREQ_FTUNE_MASK

/**	
 *@brief  SYS_CTRL_EXT_OSC24M_CFG Ext OSC24M divided to 32k
 */
#define SYS_CTRL_EXT_OSC24M_CFG_OFFSET                      0xe0
#define SYS_CTRL_EXT_OSC24M_CFG_DIV_POS  					(0UL)
#define SYS_CTRL_EXT_OSC24M_CFG_DIV_MASK      				(0x3FFUL << SYS_CTRL_EXT_OSC24M_CFG_DIV_POS)
#define SYS_CTRL_EXT_OSC24M_CFG_DIV							SYS_CTRL_EXT_OSC24M_CFG_DIV_MASK
#define SYS_CTRL_EXT_OSC24M_CFG_DIV_TOG_POS  				(10UL)
#define SYS_CTRL_EXT_OSC24M_CFG_DIV_TOG_MASK      			(0x1UL << SYS_CTRL_EXT_OSC24M_CFG_DIV_TOG_POS)
#define SYS_CTRL_EXT_OSC24M_CFG_DIV_TOG						SYS_CTRL_EXT_OSC24M_CFG_DIV_TOG_MASK

#ifdef __cplusplus
}
#endif
/** @}*/

/** @}*/
#endif /* REGS_DEFINE_H_ */
