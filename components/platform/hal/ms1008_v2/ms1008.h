/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms1008.h
 * @brief
 * @author bingrui.chen
 * @date 2021-12-17
 * @version 1.0
 * @Revision
 */
#ifndef MS_BLE_SOC_XXX_H_
#define MS_BLE_SOC_XXX_H_

#include <stdint.h>

#include "NUCLEI_N.h"
#include "ms_driver_def.h"
#include "module_base_address.h"
#include "ms_sys_ctrl_regs.h"
#include "ms_uart_regs.h"
#include "ms_dmac_regs.h"
#include "ms_qspi_reg.h"
#include "ms_rtc_reg.h"
#include "ms_audio_adc_regs.h"
#include "ms_keyscan_regs.h"
#include "ms_section.h"

#ifdef __cplusplus
extern "C" {
#endif

static __inline uint8_t _CLZ(uint32_t data) {
	if (data == 0U) {
		return 32U;
	}

	uint32_t count = 0U;
	uint32_t mask = 0x80000000U;

	while ((data & mask) == 0U) {
		count += 1U;
		mask = mask >> 1U;
	}
	return count;
}

static __inline uint32_t _RBIT(uint32_t v) {
	uint8_t sc = 31U;
	uint32_t r = v;
	for (v >>= 1U; v; v >>= 1U) {
		r <<= 1U;
		r |= v & 1U;
		sc--;
	}
	return (r << sc);
}

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (_CLZ(_RBIT(VAL)))

#define REG_BIT_MASK(bits,offset)			 	(bits<<offset)

#define TRNG_OFFSET                                 0x100

#define DMAC_OFFSET                                 0x2c0
#define DMAC_CHANNLE0_OFFSET                        0x000
#define DMAC_CHANNLE1_OFFSET                        0x058
#define DMAC_CHANNLE2_OFFSET                        0x0b0
#define DMAC_CHANNLE3_OFFSET                        0x108
#define DMAC_CHANNLE4_OFFSET                        0x160
#define DMAC_CHANNLE5_OFFSET                        0x1b8

#define TIMER0_CONFIG_OFFSET                        0x00
#define TIMER1_CONFIG_OFFSET                        0x14
#define TIMER2_CONFIG_OFFSET                        0x28
#define TIMER3_CONFIG_OFFSET                        0x3c
#define TIMER4_CONFIG_OFFSET                        0x50
#define TIMER5_CONFIG_OFFSET                        0x64
#define TIMER6_CONFIG_OFFSET                        0x78
#define TIMER7_CONFIG_OFFSET                        0x8c
#define TIMER_COMMON_OFFSET                         0xa0

#define AUDIO_ADC_OFFSET                            0xF0

#define I2S_DMA_OFFSET                                 0x1c0
#define SYS_CTRL              					    ((SysCtrlRegs_Type *)SYS_CTRL_BASE_ADDR)

#define UART0                                       ((Uart_Type *)UART0_BASE_ADDR)
#define UART1                                       ((Uart_Type *)UART1_BASE_ADDR)
#define UART2                                       ((Uart_Type *)UART2_BASE_ADDR)

#define QSPI                                        ((QSPI_Type *)QSPI_BASE_ADDR)

#define TRNG                                        ((Trng_Type *)(TRNG_BASE_ADDR+TRNG_OFFSET))

#define WATCHDOG                                    ((Watchdog_Type *)WDT_BASE_ADDR)

#define TIMER0                                      ((TimerConfig_Type *)(TIMER_BASE_ADDR+TIMER0_CONFIG_OFFSET))
#define TIMER1                                      ((TimerConfig_Type *)(TIMER_BASE_ADDR+TIMER1_CONFIG_OFFSET))
#define TIMER2                                      ((TimerConfig_Type *)(TIMER_BASE_ADDR+TIMER2_CONFIG_OFFSET))
#define TIMER3                                      ((TimerConfig_Type *)(TIMER_BASE_ADDR+TIMER3_CONFIG_OFFSET))
#define TIMER4                                      ((TimerConfig_Type *)(TIMER_BASE_ADDR+TIMER4_CONFIG_OFFSET))
#define TIMER5                                      ((TimerConfig_Type *)(TIMER_BASE_ADDR+TIMER5_CONFIG_OFFSET))
#define TIMER6                                      ((TimerConfig_Type *)(TIMER_BASE_ADDR+TIMER6_CONFIG_OFFSET))
#define TIMER7                                      ((TimerConfig_Type *)(TIMER_BASE_ADDR+TIMER7_CONFIG_OFFSET))
#define TIMER                                       ((TimerCommon_Type *)(TIMER_BASE_ADDR+TIMER_COMMON_OFFSET))

#define PWM                                       ((PWM_Type *)(PWM_BASE_ADDR))

#define DMAC                                        ((Dmac_Type *)(DMAC_BASE_ADDR+DMAC_OFFSET))
#define DMAC_CHANNLE0                               ((DmacChannel_Type *)(DMAC_BASE_ADDR+DMAC_CHANNLE0_OFFSET))
#define DMAC_CHANNLE1                               ((DmacChannel_Type *)(DMAC_BASE_ADDR+DMAC_CHANNLE1_OFFSET))
#define DMAC_CHANNLE2                               ((DmacChannel_Type *)(DMAC_BASE_ADDR+DMAC_CHANNLE2_OFFSET))
#define DMAC_CHANNLE3                               ((DmacChannel_Type *)(DMAC_BASE_ADDR+DMAC_CHANNLE3_OFFSET))
#define DMAC_CHANNLE4                               ((DmacChannel_Type *)(DMAC_BASE_ADDR+DMAC_CHANNLE4_OFFSET))
#define DMAC_CHANNLE5                               ((DmacChannel_Type *)(DMAC_BASE_ADDR+DMAC_CHANNLE5_OFFSET))

#define AUDIO_ADC                                   ((AudioAdc_Type *)(AUD_WRAP_BASE_ADDR))
#define KEYSCAN                                     ((Keyscan_Type *)KEY_SCAN_BASE_ADDR)
#define GPIO                                        ((Gpio_Type *)GPIO_BASE_ADDR)
#define AUX_ADC                                     ((AuxAdc_Type *)(AUX_ADC_BASE_ADDR))
#define SPI0                                            ((Spi_Type *)SPI0_BASE_ADDR)
#define SPI1                                            ((Spi_Type *)SPI1_BASE_ADDR)
#define I2S0                                            ((I2s_Type *)I2S0_BASE_ADDR)
#define I2S0_DMA                                   ((I2sDma_Type *)(I2S0_BASE_ADDR+I2S_DMA_OFFSET))
#define I2S1                                            ((I2s_Type *)I2S1_BASE_ADDR)
#define I2S1_DMA                                   ((I2sDma_Type *)(I2S1_BASE_ADDR+I2S_DMA_OFFSET))
#define RTC                                        ((RTC_Type *)(RTC_BASE_ADDR))
#define IR0                                               ((Ir0_Type *)(IR_BASE_ADDR))
#define IR1                                               ((Ir1_Type *)(IR1_BASE_ADDR))
#ifdef __cplusplus
}
#endif

#endif /* MS_BLE_SOC_XXX_H_ */
