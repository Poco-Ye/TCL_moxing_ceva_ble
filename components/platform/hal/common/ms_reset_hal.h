/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  ms_reset_hal.h
  * @brief 
  * @author bingrui.chen
  * @date 2022年1月18日
  * @version 1.0
  * @Revision: 
  */
#ifndef MS_RESET_HAL_H_
#define MS_RESET_HAL_H_

#include "ms_reset_ll.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MS_CLOCK_HAL_SOFT_RST()    				MS_CLOCK_LL_SOFT_RST()

#define MS_CLOCK_HAL_SOFT_RST_PWM()    		 	MS_CLOCK_LL_SOFT_RST_PWM()

#define MS_CLOCK_HAL_SOFT_RST_BASEBAND()    	MS_CLOCK_LL_SOFT_RST_BASEBAND() 

#define MS_CLOCK_HAL_SOFT_RST_QSPI()    		MS_CLOCK_LL_SOFT_RST_QSPI() 

#define MS_CLOCK_HAL_SOFT_RST_IR()    			MS_CLOCK_LL_SOFT_RST_IR() 

#define MS_CLOCK_HAL_SOFT_RST_KSCAN()    		MS_CLOCK_LL_SOFT_RST_KSCAN()

#define MS_CLOCK_HAL_SOFT_RST_RTC()    			MS_CLOCK_LL_SOFT_RST_RTC()

#define MS_CLOCK_HAL_SOFT_RST_UART0()    		MS_CLOCK_LL_SOFT_RST_UART0()

#define MS_CLOCK_HAL_SOFT_RST_UART1()    		MS_CLOCK_LL_SOFT_RST_UART1()

#define MS_CLOCK_HAL_SOFT_RST_UART2()    		MS_CLOCK_LL_SOFT_RST_UART2() 

#define MS_CLOCK_HAL_SOFT_RST_GPIO()    		MS_CLOCK_LL_SOFT_RST_GPIO()

#define MS_CLOCK_HAL_SOFT_RST_SPI0()    		MS_CLOCK_LL_SOFT_RST_SPI0()

#define MS_CLOCK_HAL_SOFT_RST_SPI1()    		MS_CLOCK_LL_SOFT_RST_SPI1()

#define MS_CLOCK_HAL_SOFT_RST_I2C0()    		MS_CLOCK_LL_SOFT_RST_I2C0()

#define MS_CLOCK_HAL_SOFT_RST_I2C1()    		MS_CLOCK_LL_SOFT_RST_I2C1()

#define MS_CLOCK_HAL_SOFT_RST_I2S0()    		MS_CLOCK_LL_SOFT_RST_I2S0()

#define MS_CLOCK_HAL_SOFT_RST_I2S1()    		MS_CLOCK_LL_SOFT_RST_I2S1()

#define MS_CLOCK_HAL_SOFT_RST_WDT()    			MS_CLOCK_LL_SOFT_RST_WDT()

#define MS_CLOCK_HAL_SOFT_RST_TRNG()    		MS_CLOCK_LL_SOFT_RST_TRNG()

#define MS_CLOCK_HAL_SOFT_RST_AUD_ADC()    		MS_CLOCK_LL_SOFT_RST_AUD_ADC()

#define MS_CLOCK_HAL_SOFT_RST_AUX_ADC()    		MS_CLOCK_LL_SOFT_RST_AUX_ADC()

#ifdef __cplusplus
}
#endif

#endif /* MS_RESET_HAL_H_ */
