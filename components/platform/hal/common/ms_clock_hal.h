/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_clock_hal.h
 * @brief Header file of clock  module.
 * @author bingrui.chen
 * @date   2021-12-16
 * @version 1.0
 * @Revision:
 */
#ifndef MS_CLOCK_HAL_H_
#define MS_CLOCK_HAL_H_

#include <ms1008.h>
#include "ms_clock_ll.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup MS_HAL_Driver
 * @{
 */

/** @defgroup CLOCK_HAL CLOCK
 * @{
 */

/** @defgroup SysClockInit_Type  System Clock Initial Structure definition
 * @{
 */
typedef struct
{
    uint32_t sys_clk_src;
    uint16_t ahb_div;
    uint16_t peri_clk_div;
    uint16_t apb_div;
} SysClockInit_Type;
/**
 * @}
 */

/** @defgroup Pll_Type  PLL Structure definition
 * @{
 */
typedef struct
{
    uint32_t pdiv;
    uint32_t mul;
} Pll_Type;
/**
 * @}
 */

/** @defgroup SysOscInit_Type  System OSC Initial Structure definition
 * @{
 */
typedef struct
{
    uint32_t hf_osc_src;
    uint32_t sys_hf_clk_src;
    uint32_t sys_lf_clk_src;
    uint32_t ext_osc_div;
    Pll_Type pll;
} SysOscInit_Type;
/**
 * @}
 */

/** @defgroup SYS_CLOCK_ENABLE_GROUP  Enable Module Clock definition
 * @{
 */

/*Enable I2S0 Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_I2S0()              __MS_CLOCK_LL_CLK_ENABLE_I2S0()

/*Enable I2S1 Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_I2S1()              __MS_CLOCK_LL_CLK_ENABLE_I2S1()

/*Enable IR Sample Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_IR_SAMPLE()         __MS_CLOCK_LL_CLK_ENABLE_IR_SAMPLE()

/*Enable UART2 communication clk*/
#define MS_CLOCK_HAL_CLK_ENABLE_UART2_COMM_CLK()    __MS_CLOCK_LL_CLK_ENABLE_UART2()

/*Enable uart2 work clk*/
#define MS_CLOCK_HAL_CLK_ENABLE_UART2_PCLK()        __MS_CLOCK_LL_CLK_ENABLE_UART2_PCLK()

/*Enable gpio rtc uart2 low power work clk*/
#define MS_CLOCK_HAL_CLK_ENABLE_GPIO_RC()           __MS_CLOCK_LL_CLK_ENABLE_GPIO_RC()
/*Enable rtc count clk*/
#define MS_CLOCK_HAL_CLK_ENABLE_RTC()               __MS_CLOCK_LL_CLK_ENABLE_RTC()

/*Enable rtc work clk*/
#define MS_CLOCK_HAL_CLK_ENABLE_RTC_PCLK()          __MS_CLOCK_LL_CLK_ENABLE_RTC_PCLK()

/*Enable BLE BaseBand Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_BB_32K()            __MS_CLOCK_LL_CLK_ENABLE_BB_32K()

/*Enable High Speed Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_HF_CLK()            __MS_CLOCK_LL_CLK_ENABLE_HF_CLK()

/*Enable Low Speed Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_LF_CLK()            __MS_CLOCK_LL_CLK_ENABLE_LF_CLK()

/*Enable DMA Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_DMA()               __MS_CLOCK_LL_CLK_ENABLE_DMA()

/*Enable I2C0 Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_I2C0()              __MS_CLOCK_LL_CLK_ENABLE_I2C0()
/*Enable I2C1 Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_I2C1()              __MS_CLOCK_LL_CLK_ENABLE_I2C1()
/*Enable UART0 Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_UART0()             __MS_CLOCK_LL_CLK_ENABLE_UART0()
/*Enable UART1 Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_UART1()             __MS_CLOCK_LL_CLK_ENABLE_UART1()
/*Enable SPI0 Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_SPI0()              __MS_CLOCK_LL_CLK_ENABLE_SPI0()
/*Enable SPI1 Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_SPI1()              __MS_CLOCK_LL_CLK_ENABLE_SPI1()
/*Enable Timerx Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_TIMERx(x)           __MS_CLOCK_LL_CLK_ENABLE_TIMERx(x)
/*Enable Key Scan Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_KSCAN()             __MS_CLOCK_LL_CLK_ENABLE_KSCAN()
/*Enable PWM Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_PWM()               __MS_CLOCK_LL_CLK_ENABLE_PWM()
/*Enable IR Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_IR()                __MS_CLOCK_LL_CLK_ENABLE_IR()

/*Enable QSPI Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_QSPI_REF()          __MS_CLOCK_LL_CLK_ENABLE_QSPI_REF()

/*Enable BLE Modem Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_MDM()               __MS_CLOCK_LL_CLK_ENABLE_MDM()
/*Enable BLE  Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_BLE()               __MS_CLOCK_LL_CLK_ENABLE_BLE()

/*Enable GPIO Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_GPIO_PCLK()         __MS_CLOCK_LL_CLK_ENABLE_GPIO_PCLK()

/*Enable CPU Timer Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_CPU_TIMER()         __MS_CLOCK_LL_CLK_ENABLE_CPU_TIMER()

/*Enable System Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_SYS()               __MS_CLOCK_LL_CLK_ENABLE_SYS()
/*Enable TRNG Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_TRNG()              __MS_CLOCK_LL_CLK_ENABLE_TRNG()
/*Enable WDT Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_WDT()               __MS_CLOCK_LL_CLK_ENABLE_WDT()
/*Enable Audio ADC Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_AUD_ADC()           __MS_CLOCK_LL_CLK_ENABLE_AUD_ADC()
/*Enable Aux ADC Clock*/
#define MS_CLOCK_HAL_CLK_ENABLE_AUX_ADC()           __MS_CLOCK_LL_CLK_ENABLE_AUX_ADC()
/**
 * @}
 */

/** @defgroup SYS_CLOCK_DISABLE_GROUP  Disable Module Clock definition
 * @{
 */

#define MS_CLOCK_HAL_CLK_DISABLE_I2S0()         __MS_CLOCK_LL_CLK_DISABLE_I2S0()

#define MS_CLOCK_HAL_CLK_DISABLE_I2S1()         __MS_CLOCK_LL_CLK_DISABLE_I2S1()

#define MS_CLOCK_HAL_CLK_DISABLE_IR_SAMPLE()    __MS_CLOCK_LL_CLK_DISABLE_IR_SAMPLE()

#define MS_CLOCK_HAL_CLK_DISABLE_UART2()        __MS_CLOCK_LL_CLK_DISABLE_UART2()

#define MS_CLOCK_HAL_CLK_DISABLE_UART2_PCLK()   __MS_CLOCK_LL_CLK_DISABLE_UART2_PCLK()

#define MS_CLOCK_HAL_CLK_DISABLE_GPIO_RC()      __MS_CLOCK_LL_CLK_DISABLE_GPIO_RC()

#define MS_CLOCK_HAL_CLK_DISABLE_RTC()          __MS_CLOCK_LL_CLK_DISABLE_RTC()

#define MS_CLOCK_HAL_CLK_DISABLE_RTC_PCLK()     __MS_CLOCK_LL_CLK_DISABLE_RTC_PCLK()

#define MS_CLOCK_HAL_CLK_DISABLE_BB_32K()       __MS_CLOCK_LL_CLK_DISABLE_BB_32K()

#define MS_CLOCK_HAL_CLK_DISABLE_HF_CLK()       __MS_CLOCK_LL_CLK_DISABLE_HF_CLK()

#define MS_CLOCK_HAL_CLK_DISABLE_LF_CLK()       __MS_CLOCK_LL_CLK_DISABLE_LF_CLK()

#define MS_CLOCK_HAL_CLK_DISABLE_DMA()          __MS_CLOCK_LL_CLK_DISABLE_DMA()

#define MS_CLOCK_HAL_CLK_DISABLE_I2C0()         __MS_CLOCK_LL_CLK_DISABLE_I2C0()

#define MS_CLOCK_HAL_CLK_DISABLE_I2C1()         __MS_CLOCK_LL_CLK_DISABLE_I2C1()

#define MS_CLOCK_HAL_CLK_DISABLE_UART0()        __MS_CLOCK_LL_CLK_DISABLE_UART0()

#define MS_CLOCK_HAL_CLK_DISABLE_UART1()        __MS_CLOCK_LL_CLK_DISABLE_UART1()

#define MS_CLOCK_HAL_CLK_DISABLE_SPI0()         __MS_CLOCK_LL_CLK_DISABLE_SPI0()

#define MS_CLOCK_HAL_CLK_DISABLE_SPI1()         __MS_CLOCK_LL_CLK_DISABLE_SPI1()

#define MS_CLOCK_HAL_CLK_DISABLE_TIMERx(x)      __MS_CLOCK_LL_CLK_DISABLE_TIMERx(x)

#define MS_CLOCK_HAL_CLK_DISABLE_KSCAN()        __MS_CLOCK_LL_CLK_DISABLE_KSCAN()

#define MS_CLOCK_HAL_CLK_DISABLE_PWM()          __MS_CLOCK_LL_CLK_DISABLE_PWM()

#define MS_CLOCK_HAL_CLK_DISABLE_IR()           __MS_CLOCK_LL_CLK_DISABLE_IR()

#define MS_CLOCK_HAL_CLK_DISABLE_QSPI_REF()     __MS_CLOCK_LL_CLK_DISABLE_QSPI_REF()

#define MS_CLOCK_HAL_CLK_DISABLE_MDM()          __MS_CLOCK_LL_CLK_DISABLE_MDM()

#define MS_CLOCK_HAL_CLK_DISABLE_BLE()          __MS_CLOCK_LL_CLK_DISABLE_BLE()

#define MS_CLOCK_HAL_CLK_DISABLE_GPIO_PCLK()    __MS_CLOCK_LL_CLK_DISABLE_GPIO_PCLK()

#define MS_CLOCK_HAL_CLK_DISABLE_CPU_TIMER()    __MS_CLOCK_LL_CLK_DISABLE_CPU_TIMER()

#define MS_CLOCK_HAL_CLK_DISABLE_SYSC()         __MS_CLOCK_LL_CLK_DISABLE_SYSC()

#define MS_CLOCK_HAL_CLK_DISABLE_TRNG()         __MS_CLOCK_LL_CLK_DISABLE_TRNG()

#define MS_CLOCK_HAL_CLK_DISABLE_WDT()          __MS_CLOCK_LL_CLK_DISABLE_WDT()

#define MS_CLOCK_HAL_CLK_DISABLE_AUD_ADC()      __MS_CLOCK_LL_CLK_DISABLE_AUD_ADC()

#define MS_CLOCK_HAL_CLK_DISABLE_AUX_ADC()      __MS_CLOCK_LL_CLK_DISABLE_AUX_ADC()
/**
 * @}
 */

/** @defgroup GET_SYS_CLOCK_SETTING_GROUP  Get Module Clock Setting definition
 * @{
 */

#define MS_CLOCK_HAL_CLK_IS_ENABLE_I2S0()         __MS_CLOCK_LL_CLK_IS_ENABLE_I2S0()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_I2S1()         __MS_CLOCK_LL_CLK_IS_ENABLE_I2S1()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_IR_SAMPLE()    __MS_CLOCK_LL_CLK_IS_ENABLE_IR_SAMPLE()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_UART2()        __MS_CLOCK_LL_CLK_IS_ENABLE_UART2()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_UART2_PCLK()   __MS_CLOCK_LL_CLK_IS_ENABLE_UART2_PCLK()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_GPIO_RC()      __MS_CLOCK_LL_CLK_IS_ENABLE_GPIO_RC()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_RTC()          __MS_CLOCK_LL_CLK_IS_ENABLE_RTC()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_RTC_PCLK()     __MS_CLOCK_LL_CLK_IS_ENABLE_RTC_PCLK()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_BB_32K()       __MS_CLOCK_LL_CLK_IS_ENABLE_BB_32K()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_HF_CLK()       __MS_CLOCK_LL_CLK_IS_ENABLE_HF_CLK()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_LF_CLK()       __MS_CLOCK_LL_CLK_IS_ENABLE_LF_CLK()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_DMA()          __MS_CLOCK_LL_CLK_IS_ENABLE_DMA()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_I2C0()         __MS_CLOCK_LL_CLK_IS_ENABLE_I2C0()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_I2C1()         __MS_CLOCK_LL_CLK_IS_ENABLE_I2C1()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_UART0()        __MS_CLOCK_LL_CLK_IS_ENABLE_UART0()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_UART1()        __MS_CLOCK_LL_CLK_IS_ENABLE_UART1()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_SPI0()         __MS_CLOCK_LL_CLK_IS_ENABLE_SPI0()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_SPI1()         __MS_CLOCK_LL_CLK_IS_ENABLE_SPI1()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_TIMERx(x)      __MS_CLOCK_LL_CLK_IS_ENABLE_TIMERx(x)

#define MS_CLOCK_HAL_CLK_IS_ENABLE_KSCAN()        __MS_CLOCK_LL_CLK_IS_ENABLE_KSCAN()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_PWM()          __MS_CLOCK_LL_CLK_IS_ENABLE_PWM()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_IR()           __MS_CLOCK_LL_CLK_IS_ENABLE_IR()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_QSPI_REF()     __MS_CLOCK_LL_CLK_IS_ENABLE_QSPI_REF()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_MDM()          __MS_CLOCK_LL_CLK_IS_ENABLE_MDM()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_BLE()          __MS_CLOCK_LL_CLK_IS_ENABLE_BLE()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_GPIO_PCLK()    __MS_CLOCK_LL_CLK_IS_ENABLE_GPIO_PCLK()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_CPU_TIMER()    __MS_CLOCK_LL_CLK_IS_ENABLE_CPU_TIMER()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_SYSC()         __MS_CLOCK_LL_CLK_IS_ENABLE_SYSC()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_TRNG()         __MS_CLOCK_LL_CLK_IS_ENABLE_TRNG()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_WDT()          __MS_CLOCK_LL_CLK_IS_ENABLE_WDT()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_AUD_ADC()      __MS_CLOCK_LL_CLK_IS_ENABLE_AUD_ADC()

#define MS_CLOCK_HAL_CLK_IS_ENABLE_AUX_ADC()      __MS_CLOCK_LL_CLK_IS_ENABLE_AUX_ADC()

/**
 * @}
 */

/**
 * @brief  select the hf osc clk source
 * @param  uint32_t hf_osc_soucre: This parameter can be one of the following values:
 *         HF_OSC_CLK_SEL_RC_OSC16M		    : select rc osc16m
 *         HF_OSC_CLK_SEL_EXT_OSC24M	    : select external high osc
 * @retval None
 */
#define ms_clock_hal_hf_osc_sel(hf_osc_soucre) ms_clock_ll_hf_osc_sel((hf_osc_soucre))

/**
 * @brief  get the hf_osc source
 * @retval The current hf_osc source
 */
#define ms_clock_ll_get_hf_osc_src() ms_clock_ll_get_hf_osc_src()

/**
 * @brief  select the ahb high clk source
 * @param  uint32_t ahb_high_clk_soucre: This parameter can be one of the following values:
 *         HF_CLK_SEL_PLL_CLK    : select the clk output of  pll
 *         HF_CLK_SEL_BYPASS_PLL : select the clk bypass pll
 * @retval None
 */
#define ms_clock_hal_hf_clk_sel(hf_clk_soucre) ms_clock_ll_hf_clk_sel((hf_clk_soucre))

/**
 * @brief  get the hf_clk source
 * @retval The current hf_clk source
 */
#define ms_clock_hal_get_hf_clk_src() ms_clock_ll_get_hf_clk_src()

/**
 * @brief  select the main clk source
 * @param  uint32_t main_clk_soucre: This parameter can be one of the following values:
 *         SYS_CLK_SEL_LF_CLK	: select the sys_lf_clk
 *         SYS_CLK_SEL_HF_CLK	: select the sys_hf_clk
 * @retval None
 */
#define ms_clock_hal_sys_clk_sel(sys_clk_soucre) ms_clock_ll_sys_clk_sel((sys_clk_soucre))

/**
 * @brief  get the sys_clk source
 * @retval The current sys_clk source
 */
#define ms_clock_hal_get_sys_clk_src() ms_clock_ll_get_sys_clk_src()

/**
 * @brief  select the low clk source
 * @param  uint32_t low_clk_soucre: This parameter can be one of the following values:
 *         LF_CLK_SEL_RC_OSC32K		    : select rc osc32k
 *         LF_CLK_SEL_EXT_OSC24M	    : select the division clk of  the external high osc
 * @retval None
 */
#define ms_clock_hal_lf_clk_sel(lf_clk_source) ms_clock_ll_lf_clk_sel((lf_clk_source))

/**
 * @brief  get the lf_clk source
 * @retval The current lf_clk source
 */
#define ms_clock_ll_get_lf_clk_src() ms_clock_ll_get_lf_clk_src()

/**
 * @brief  select the gpio_fclk
 * @param  uint32_t gpio_fclk_source:This parameter can be one of the following values:
 *           @arg @ref  GPIO_CLK_SEL_PCLK          Select the pclk as the GPIO Clock
 *           @arg @ref  GPIO_CLK_SEL_RC_CLK        Select the rc clock as the GPIO Clock
 * @retval None
 */
#define ms_clock_hal_gpio_fclk_sel(gpio_fclk_source) ms_clock_ll_gpio_fclk_sel((gpio_fclk_source))

/**
 * @brief  get the gpio fclk source
 * @retval The current gpio fclk source
 */
#define ms_clock_hal_get_gpio_fclk_src() ms_clock_ll_get_gpio_fclk_src()

/**
 * @brief  select the uart2_fclk
 * @param  uint32_t uart2_fclk_source: This parameter can be one of the following values:
 *           @arg @ref  UART2_CLK_SEL_PCLK          Select the pclk as the UART2 Clock
 *           @arg @ref  UART2_CLK_SEL_RC_CLK        Select the rc clock as the UART2 Clock
 * @retval None
 */
#define ms_clock_hal_uart2_fclk_sel(uart2_fclk_source) ms_clock_ll_uart2_fclk_sel((uart2_fclk_source))

/**
 * @brief  get the uart2 fclk source
 * @retval The current uart2 fclk source
 */
#define ms_clock_hal_get_uart2_fclk_src() ms_clock_ll_get_uart2_fclk_src()

/**
 * @brief  configure the params of pll
 * @param  uint32_t rtc_fclk_source:This parameter can be one of the following values:
 *           @arg @ref  RTC_CLK_SEL_PCLK          Select the pclk as the RTC Clock
 *           @arg @ref  RTC_CLK_SEL_RC_CLK        Select the rc clock as the RTC Clock
 * @retval None
 */
#define ms_clock_hal_rtc_fclk_sel(rtc_fclk_source) ms_clock_ll_rtc_fclk_sel((rtc_fclk_source))

/**
 * @brief  get the rtc fclk source
 * @retval The current rtc fclk source
 */
#define ms_clock_hal_get_rtc_fclk_src() ms_clock_ll_get_rtc_fclk_src()

/**
 * @brief  Enable the pll unit
 * @retval None
 */
#define ms_clock_hal_pll_enable() ms_clock_ll_pll_enable()

/**
 * @brief  Disable the pll unit
 * @retval None
 */
#define ms_clock_hal_pll_disable() ms_clock_ll_pll_disable()

/**
 * @brief  Configure the params of pll
 * @param  uint32_t pre_div: 1~31, the valid value: 1,2,4,8,16
 *         uint32_t mul_pll: 10~511, the value less than 9 is forbidden
 * @retval None
 */
#define ms_clock_hal_pll_config(pre_div,mul_pll) ms_clock_ll_pll_config((pre_div),(mul_pll))

/**
 * @brief  verify the pll lock status
 * @retval true  : lock
 *         false : unlock
 */
#define ms_clock_hal_pll_is_locked() ms_clock_ll_pll_is_locked()

/**
 * @brief  set the ahb division
 * @param  uint32_t ahb_div: 0~15, the valid value: 0,1,2,4,8
 * @retval None
 */
#define ms_clock_hal_set_ahb_div(ahb_div) ms_clock_ll_set_ahb_div((ahb_div))

/**
 * @brief  get the apb division
 * @retval The current ahb clock division
 */
#define ms_clock_hal_get_ahb_div() ms_clock_ll_get_ahb_div()

/**
 * @brief  set the apb division
 * @param  uint32_t apb_div: 0~15, the valid value: 0,1,2,4,8
 * @retval None
 */
#define ms_clock_hal_set_apb_div(apb_div) ms_clock_ll_set_apb_div((apb_div))

/**
 * @brief  get the apb division
 * @retval The current apb division
 */
#define ms_clock_hal_get_apb_div() ms_clock_ll_get_apb_div()

/**
 * @brief  set the peripheral pre division
 * @param  uint32_t pre_div: 0~15, the valid value: 0,1,2,4,8
 * @retval None
 */
#define ms_clock_hal_set_periph_pre_div(pre_div) ms_clock_ll_set_periph_pre_div((pre_div))

/**
 * @brief  get the peripheral pre division
 * @retval The current peripheral pre division
 */
#define ms_clock_hal_get_periph_pre_div() ms_clock_ll_get_periph_pre_div()

/**
 * @brief  set the ext high osc division
 * @param  uint32_t osc_div: 0~15, the valid value: 0,1,2,4,8
 * @retval None
 */
#define ms_clock_hal_set_ext_high_osc_div(osc_div) ms_clock_ll_set_ext_high_osc_div((osc_div))

/**
 * @brief  set the ext high osc division
 * @retval The current ext high osc division
 */
#define ms_clock_hal_get_ext_high_osc_div() ms_clock_ll_get_ext_high_osc_div()


/**
 * @brief  trigger after setting the ext osc division
 * @retval None
 */
#define ms_clock_hal_set_ext_high_osc_div_toggle() ms_clock_ll_set_ext_high_osc_div_toggle()


/**
 * @brief  set i2s0 mdiv
 * @param  uint32_t mdiv
 * @retval None
 */
#define ms_clock_hal_set_i2s0_mdiv(mdiv) ms_clock_ll_set_i2s0_mdiv((mdiv))


/**
 * @brief  get i2s0 mdiv
 * @retval The setting i2s0 mdiv
 */
#define ms_clock_hal_get_i2s0_mdiv() ms_clock_ll_get_i2s0_mdiv()


/**
 * @brief  set i2s1 mdiv
 * @param  uint32_t mdiv
 * @retval None
 */
#define ms_clock_hal_set_i2s1_mdiv(mdiv) ms_clock_ll_set_i2s1_mdiv((mdiv))


/**
 * @brief  get i2s1 mdiv
 * @retval The setting i2s1 mdiv
 */
#define ms_clock_hal_get_i2s1_mdiv() ms_clock_ll_get_i2s1_mdiv()



/**
 * @brief  set modem 0 div
 * @retval none
 */
#define ms_clock_hal_set_mdm0_div(div) ms_clock_ll_set_mdm0_div(div)

/**
 * @brief  set modem 1 div
 * @retval none
 */

#define ms_clock_hal_set_ble_div(div) ms_clock_ll_set_ble_div(div)


/**
 * @brief  set i2s0 div
 * @param  uint32_t div
 * @retval None
 */
#define ms_clock_hal_set_i2s0_div(div) ms_clock_ll_set_i2s0_div((div))


/**
 * @brief  get i2s0 div
 * @retval The setting i2s0 div
 */
#define ms_clock_hal_get_i2s0_div() ms_clock_ll_get_i2s0_div()


/**
 * @brief  set i2s1 div
 * @param  uint32_t div
 * @retval None
 */
#define ms_clock_hal_set_i2s1_div(div) ms_clock_ll_set_i2s1_div((div))



/**
 * @brief  get i2s1 div
 * @retval The setting i2s0 div
 */
#define ms_clock_hal_get_i2s1_div() ms_clock_ll_get_i2s1_div()


/**
 * @brief  set UART0 div
 * @param  uint32_t div
 * @retval None
 */
#define ms_clock_hal_set_uart0_div(div) ms_clock_ll_set_uart0_div((div))


/**
 * @brief  get UART0 div
 * @retval The setting UART0 div
 */
#define ms_clock_hal_get_uart0_div() ms_clock_ll_get_uart0_div()


/**
 * @brief  set UART1 div
 * @param  uint32_t div
 * @retval None
 */
#define ms_clock_hal_set_uart1_div(div) ms_clock_ll_set_uart1_div((div))


/**
 * @brief  get UART1 div
 * @retval The setting UART1 div
 */
#define ms_clock_hal_get_uart1_div() ms_clock_ll_get_uart1_div()



/**
 * @brief  set spi0 div
 * @retval none
 */
#define  ms_clock_hal_set_spi0_div( div) ms_clock_ll_set_spi0_div(( div))

/**
 * @brief  set spi1 div
 * @retval none
 */
#define  ms_clock_hal_set_spi1_div( div) ms_clock_ll_set_spi1_div(( div))


/**
 * @brief  set time1 div
 * @retval none
 */
#define ms_clock_hal_set_timer0_div(div) ms_clock_ll_set_timer0_div((div))


/**
 * @brief  set time1 div
 * @retval none
 */
#define ms_clock_hal_set_timer1_div(div) ms_clock_ll_set_timer1_div((div))

/**
 * @brief  set time1 div
 * @retval none
 */
#define ms_clock_hal_set_timer2_div(div) ms_clock_ll_set_timer2_div((div))
/**
 * @brief  set time1 div
 * @retval none
 */
#define ms_clock_hal_set_timer3_div(div) ms_clock_ll_set_timer3_div((div))

/**
 * @brief  set time1 div
 * @retval none
 */
#define ms_clock_hal_set_timer4_div(div) ms_clock_ll_set_timer4_div((div))


/**
 * @brief  set time1 div
 * @retval none
 */
#define ms_clock_hal_set_timer5_div(div) ms_clock_ll_set_timer5_div((div))

/**
 * @brief  set time1 div
 * @retval none
 */
#define ms_clock_hal_set_timer6_div(div) ms_clock_ll_set_timer6_div((div))

/**
 * @brief  set time1 div
 * @retval none
 */
#define ms_clock_hal_set_timer7_div(div) ms_clock_ll_set_timer7_div((div))



/**
 * @brief  set pwm div
 * @retval none
 */
#define ms_clock_hal_set_pwm_div(div) ms_clock_ll_set_pwm_div((div))


/**
 * @brief  set cpu timer div
 * @retval none
 */
#define ms_clock_hal_set_cpu_tmr_div(div) ms_clock_ll_set_cpu_tmr_div((div))

/**
 * @brief  set ir div
 * @retval none
 */
#define ms_clock_hal_set_ir_div(div) ms_clock_ll_set_ir_div((div))


/**
 * @brief  set time1 div
 * @retval none
 */
#define ms_clock_hal_set_wdt_div(div) ms_clock_ll_set_wdt_div((div))

/**
 * @brief  set aux adc div
 * @retval none
 */
#define ms_clock_hal_set_aux_adc_div(div) ms_clock_ll_set_aux_adc_div((div))


/**
 * @brief  set keyscan div
 * @retval none
 */
#define ms_clock_hal_set_keyscan_div(div) ms_clock_ll_set_kscan_div((div))
/**
 * @brief  set aux adc pwr on
 * @retval none
 */
#define ms_aux_adc_pwr_on()  ms_clock_ll_aux_adc_pwr_on()


/**
 * @brief  toggle the peripheral clk div
 * @param[in]  PeripheralClkToggle_Type clk:  peripheral clock type
 *            @ref PeripheralClkToggle_Type
 * @retval None
 */
#define ms_clock_hal_peripheral_clk_div_toggle(clk) ms_clock_ll_peripheral_clk_div_toggle((clk))



#define ms_clock_hal_BLE_clk_div_toggle()  ms_clock_ll_BLE_clk_div_toggle()

#define ms_clock_hal_MDM0_clk_div_toggle() ms_clock_ll_MDM0_clk_div_toggle()

#define ms_clock_hal_peripre_clk_div_toggle() ms_clock_ll_peripre_clk_div_toggle()



/**
 * @brief  Initializes the Oscillators according to the specified parameters in the
 *         sys_osc_init_t.
 * @param  p_osc_init pointer to an sys_osc_init_t structure that
 *         contains the configuration information for the Oscillators.
 * @retval 0 :success, -1 : error
 */
extern int32_t ms_clock_hal_sys_config_osc(SysOscInit_Type *p_osc_init);

/**
 * @brief  Initializes the CPU, AHB and APB buses clocks according to the specified
 *         parameters in the sys_clock_init_t.
 * @param  p_sys_clock_init pointer to an sys_clock_init_t structure that
 *         contains the configuration information for the peripheral.
 * @retval 0 :success, -1 : error
 */
extern int32_t ms_clock_hal_config_clock(SysClockInit_Type *p_sys_clock_init);

/**
 * @brief  get the sys_clk frequency.
 * @retval The sys_clk frequency
 */
extern uint32_t ms_clock_hal_get_sys_clk_freq(void);

/**
 * @brief  get the peri_clk frequency.
 * @retval The peri_clk frequency
 */
extern uint32_t ms_clock_hal_get_peri_clk_freq(void);

/**
 * @brief  get the peri_clk frequency.
 * @retval The peri_clk frequency
 */
extern uint32_t ms_clock_hal_get_hclk_freq(void);

/**
 * @brief  get the peri_clk frequency.
 * @retval The peri_clk frequency
 */
extern uint32_t ms_clock_hal_get_lf_clk_freq(void);

#ifdef __cplusplus
}
#endif

/** @}*/

/** @}*/

#endif /* MS_SYS_CTRL_CLOCK_H_ */
