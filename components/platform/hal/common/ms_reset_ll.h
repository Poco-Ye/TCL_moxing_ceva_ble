/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_reset_ll.h
 * @brief
 * @author bingrui.chen
 * @date 2021-12-20
 * @version 1.0
 * @Revision
 */
#ifndef MS_SYS_CTRL_RESET_H_
#define MS_SYS_CTRL_RESET_H_

#include "ms1008.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MS_CLOCK_LL_SOFT_RST()    			do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_PWM()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_PWM);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_PWM);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_PWM);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_BASEBAND()    	do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_BASEBAND);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_BASEBAND);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_BASEBAND);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_QSPI()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_QSPI);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_QSPI);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_QSPI);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_IR()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_IR);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_IR);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_IR);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_KSCAN()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_KSCAN);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_KSCAN);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_KSCAN);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_RTC()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_RTC);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_RTC);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_RTC);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_UART0()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_UART0);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_UART0);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_UART0);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_UART1()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_UART1);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_UART1);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_UART1);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_UART2()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_UART2);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_UART2);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_UART2);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_GPIO()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_GPIO);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_GPIO);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_GPIO);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_SPI0()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_SPI0);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_SPI0);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_SPI0);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_SPI1()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_SPI1);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_SPI1);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_SPI1);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_I2C0()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_I2C0);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_I2C0);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_I2C0);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_I2C1()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_I2C1);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_I2C1);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_I2C1);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_I2S0()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_I2S0);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_I2S0);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_I2S0);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_I2S1()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_I2S1);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_I2S1);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_I2S1);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_WDT()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_WDT);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_WDT);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_WDT);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_TRNG()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_TRNG);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_TRNG);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_TRNG);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_AUD_ADC()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_AUD_ADC);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_AUD_ADC);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_AUD_ADC);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#define MS_CLOCK_LL_SOFT_RST_AUX_ADC()    		do { \
		                                        SET_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_AUX_ADC);\
		                                        /* Delay after active the soft reset */\
		                                        volatile tmpreg = READ_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_AUX_ADC);\
		                                        CLEAR_BIT(SYS_CTRL_PTR->soft_rst, SYS_CTRL_SOFT_RST_AUX_ADC);\
		                                        UNUSED(tmpreg); \
		                                      } while(0U)

#ifdef __cplusplus
}
#endif

#endif /* MS_SYS_CTRL_RESET_H_ */
