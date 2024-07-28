/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  ms_clock_ll.h
 * @brief
 * @author bingrui.chen
 * @date 2022-1-18
 * @version 1.0
 * @Revision:
 */
#ifndef MS_CLOCK_LL_H_
#define MS_CLOCK_LL_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup MS_LL_Driver
 * @{
 */

/** @defgroup CLOCK_LL CLOCK
 * @{
 */
/** @defgroup PeripheralClkToggle_Type Peripheral Clock Toggle Type definition
 * @{
 */
typedef enum
{
    I2S0_MDIV_TOG,
    I2S1_MDIV_TOG,
    I2S0_DIV_TOG,
    I2S1_DIV_TOG,
    I2C0_DIV_TOG,
    I2C1_DIV_TOG,
    UART0_DIV_TOG,
    UART1_DIV_TOG,
    SPI0_DIV_TOG,
    SPI1_DIV_TOG,
    TIMER0_DIV_TOG,
    TIMER1_DIV_TOG,
    TIMER2_DIV_TOG,
    TIMER3_DIV_TOG,
    TIMER4_DIV_TOG,
    TIMER5_DIV_TOG,
    TIMER6_DIV_TOG,
    TIMER7_DIV_TOG,
    KSCAN_DIV_TOG,
    PWM_DIV_TOG,
    BLE_DIV_TOG,
    IR_DIV_TOG,
    CALI_DIV_TOG,
    QSPI_DIV_TOG,
    GPIO_DIV_TOG,
    PERI_PRE_DIV_TOG,
    MDM0_DIV_TOG
} PeripheralClkToggle_Type;

/**
 * @}
 */
/** @defgroup HF_OSC_CLK_SEL High OSC Clock Select definition
 * @{
 */
#define HF_OSC_CLK_SEL_RC_OSC16M                    (0x0000 << SYS_CTRL_CLK_SEL_HF_OSC_CLK_POS)
#define HF_OSC_CLK_SEL_EXT_OSC24M                   (0x0001 << SYS_CTRL_CLK_SEL_HF_OSC_CLK_POS)

/**
 * @}
 */

/** @defgroup LF_CLK_SEL Low Speed Clock Select definition
 * @{
 */
#define LF_CLK_SEL_RC_OSC32K                        (0x0000<<SYS_CTRL_CLK_SEL_LF_CLK_POS)
#define LF_CLK_SEL_EXT_OSC24M                       (0x0001<<SYS_CTRL_CLK_SEL_LF_CLK_POS)

/**
 * @}
 */

/** @defgroup SYS_CLK_SEL System Clock Select definition
 * @{
 */
#define SYS_CLK_SEL_LF_CLK                          (0x0000 << SYS_CTRL_CLK_SEL_SYS_CLK_POS)
#define SYS_CLK_SEL_HF_CLK                          (0x0001 << SYS_CTRL_CLK_SEL_SYS_CLK_POS)

/**
 * @}
 */

/** @defgroup HF_CLK_SEL_PLL  High Speed Clock PLL Path Select definition
 * @{
 */
#define HF_CLK_SEL_PLL_CLK                          (0x0000 << SYS_CTRL_CLK_SEL_HF_CLK_POS)
#define HF_CLK_SEL_BYPASS_PLL                       (0x0001 << SYS_CTRL_CLK_SEL_HF_CLK_POS)
/**
 * @}
 */

/** @defgroup GPIO_CLK_SEL  GPIO Clock Select definition
 * @{
 */
#define GPIO_CLK_SEL_PCLK                           (0x0000 << SYS_CTRL_CLK_SEL_GPIO_FCLK_POS)
#define GPIO_CLK_SEL_RC_CLK                         (0x0001 << SYS_CTRL_CLK_SEL_GPIO_FCLK_POS)
/**
 * @}
 */

/** @defgroup RTC_CLK_SEL  RTC Clock Select definition
 * @{
 */
#define RTC_CLK_SEL_PCLK                            (0x0000 << SYS_CTRL_CLK_SEL_RTC_FPCLK_POS)
#define RTC_CLK_SEL_RC_CLK                          (0x0001 << SYS_CTRL_CLK_SEL_RTC_FPCLK_POS)
/**
 * @}
 */

/** @defgroup UART2_CLK_SEL  UART2 Clock Select definition
 * @{
 */
#define UART2_CLK_SEL_PCLK                          (0x0000 << SYS_CTRL_CLK_SEL_UART2_PCLK_POS)
#define UART2_CLK_SEL_RC_CLK                        (0x0001 << SYS_CTRL_CLK_SEL_UART2_PCLK_POS)
/**
 * @}
 */

/** @defgroup SYS_CLOCK_ENABLE_GROUP  Enable Module Clock definition
 * @{
 */

/*Enable I2S0 Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_I2S0()         do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2S0);\
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_I2S0);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2S0);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable I2S1 Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_I2S1()         do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2S1);\
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_I2S1);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2S1);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable IR Sample Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_IR_SAMPLE()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_IR_SAMPLE);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_IR_SAMPLE);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable UART2 communication clk*/
#define __MS_CLOCK_LL_CLK_ENABLE_UART2()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_UART2);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_UART2);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/*Enable uart2 work clk*/
#define __MS_CLOCK_LL_CLK_ENABLE_UART2_PCLK()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_UART2);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_UART2);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable gpio rtc uart2 low power work clk*/
#define __MS_CLOCK_LL_CLK_ENABLE_GPIO_RC()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_GPIO_RC);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_GPIO_RC);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable rtc count clk*/
#define __MS_CLOCK_LL_CLK_ENABLE_RTC()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_RTC);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_RTC);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable rtc work clk*/
#define __MS_CLOCK_LL_CLK_ENABLE_RTC_PCLK()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_RTC);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_RTC);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable BLE BaseBand Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_BLE_32K()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_BLE_32K);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_BLE_32K);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable High Speed Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_HF_CLK()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_HF_CLK);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_HF_CLK);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable Low Speed Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_LF_CLK()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_LF_CLK);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_LF_CLK);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable DMA Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_DMA()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_DMA);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_DMA);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable I2C0 Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_I2C0()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2C0);\
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_I2C0);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2C0);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/*Enable I2C1 Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_I2C1()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2C1);\
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_I2C1);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2C1);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/*Enable UART0 Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_UART0()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_UART0);\
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_UART0);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_UART0);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/*Enable UART1 Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_UART1()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_UART1);\
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_UART1);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_UART1);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/*Enable SPI0 Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_SPI0()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_SPI0);\
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_SPI0);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_SPI0);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/*Enable SPI1 Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_SPI1()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_SPI1);\
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_SPI1);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_SPI1);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/*Enable Timerx Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_TIMERx(x)     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_TIMER##x);\
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_TIMER);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_TIMER##x);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/*Enable Key Scan Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_KSCAN()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_KSCAN);\
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_KSCAN);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_KSCAN);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable PWM Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_PWM()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_PWM);\
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_PWM);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_PWM);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/*Enable IR Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_IR()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_IR);\
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_IR);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_IR);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable QSPI Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_QSPI_REF()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_QSPI_REF);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_QSPI_REF);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable BLE Modem Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_MDM()         do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_MDM);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_MDM);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/*Enable BLE  Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_BLE()         do { \
                                                SET_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_BLE);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_BLE);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable GPIO Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_GPIO_PCLK()     do { \
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_GPIO);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_GPIO);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable CPU Timer Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_CPU_TIMER()         do { \
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_CPU_TIMER);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_CPU_TIMER);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable System Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_SYS()         do { \
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_SYS);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_SYS);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/*Enable TRNG Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_TRNG()         do { \
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_TRNG);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_TRNG);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/*Enable WDT Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_WDT()         do { \
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_WDT);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_WDT);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/*Enable Audio ADC Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_AUD_ADC()    do { \
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_AUD);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_AUD);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/*Enable Aux ADC Clock*/
#define __MS_CLOCK_LL_CLK_ENABLE_AUX_ADC()         do { \
                                                SET_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_AUX);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_AUX);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/**
 * @}
 */

/** @defgroup SYS_CLOCK_DISABLE_GROUP  Disable Module Clock definition
 * @{
 */

#define __MS_CLOCK_LL_CLK_DISABLE_I2S0()         do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2S0);\
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_I2S0);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2S0);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_I2S1()         do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2S1);\
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_I2S1);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2S1);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_IR_SAMPLE()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_IR_SAMPLE);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_IR_SAMPLE);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_UART2()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_UART2);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_UART2);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_UART2_PCLK()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_UART2);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_UART2);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_GPIO_RC()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_GPIO_RC);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_GPIO_RC);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_RTC()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_RTC);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_RTC);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_RTC_PCLK()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_RTC);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_RTC);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_BLE_32K()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_BLE_32K);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_BLE_32K);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_HF_CLK()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_HF_CLK);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_HF_CLK);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_LF_CLK()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_LF_CLK);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_LF_CLK);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_DMA()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_DMA);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_DMA);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_I2C0()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2C0);\
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_I2C0);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2C0);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_I2C1()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2C1);\
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_I2C1);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_I2C1);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_UART0()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_UART0);\
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_UART0);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_UART0);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_UART1()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_UART1);\
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_UART1);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_UART1);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_SPI0()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_SPI0);\
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_SPI0);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_SPI0);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_SPI1()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_SPI1);\
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_SPI1);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_SPI1);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_TIMERx(x)     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_TIMER##x);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_TIMER##x);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_KSCAN()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_KSCAN);\
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_KSCAN);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_KSCAN);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_PWM()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_PWM);\
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_PWM);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_PWM);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_IR()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_IR);\
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_IR);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_IR);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_QSPI_REF()     do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_QSPI_REF);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_QSPI_REF);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_MDM()         do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_MDM);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_MDM);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_BLE()         do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_BLE);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN0, SYS_CTRL_CLK_EN0_BLE);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_GPIO_PCLK()         do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_GPIO);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_GPIO);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_CPU_TIMER()         do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_CPU_TIMER);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_CPU_TIMER);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_SYSC()         do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_SYS);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_SYS);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_TRNG()         do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_TRNG);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_TRNG);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_WDT()         do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_WDT);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_WDT);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_AUD_ADC()         do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_AUD);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_AUD);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

#define __MS_CLOCK_LL_CLK_DISABLE_AUX_ADC()         do { \
                                                CLEAR_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_AUX);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CLK_EN1, SYS_CTRL_CLK_EN1_AUX);\
                                                UNUSED(tmpreg); \
                                              } while(0U)
/**
 * @}
 */

/** @defgroup GET_SYS_CLOCK_SETTING_GROUP  Get Module Clock Setting definition
 * @{
 */

#define __MS_CLOCK_LL_CLK_IS_ENABLE_I2S0()         (((SYS_CTRL->CLK_EN0&SYS_CTRL_CLK_EN0_I2S0)== SYS_CTRL_CLK_EN0_I2S0) && \
                                                    ((SYS_CTRL->CLK_EN1&SYS_CTRL_CLK_EN1_I2S0)==SYS_CTRL_CLK_EN1_I2S0))

#define __MS_CLOCK_LL_CLK_IS_ENABLE_I2S1()         (((SYS_CTRL->CLK_EN0&SYS_CTRL_CLK_EN0_I2S1)== SYS_CTRL_CLK_EN0_I2S1) && \
                                                    ((SYS_CTRL->CLK_EN1&SYS_CTRL_CLK_EN1_I2S1)==SYS_CTRL_CLK_EN1_I2S1))

#define __MS_CLOCK_LL_CLK_IS_ENABLE_IR_SAMPLE()    ((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_IR_SAMPLE) == SYS_CTRL_CLK_EN0_IR_SAMPLE)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_UART2()        ((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_UART2) == SYS_CTRL_CLK_EN0_UART2)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_UART2_PCLK()   ((SYS_CTRL->CLK_EN1 & SYS_CTRL_CLK_EN1_UART2) == SYS_CTRL_CLK_EN1_UART2)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_GPIO_RC()      ((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_GPIO_RC) == SYS_CTRL_CLK_EN0_GPIO_RC)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_RTC()          ((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_RTC) == SYS_CTRL_CLK_EN0_RTC)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_RTC_PCLK()     ((SYS_CTRL->CLK_EN1 & SYS_CTRL_CLK_EN1_RTC_PCLK) == SYS_CTRL_CLK_EN1_RTC_PCLK)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_BLE_32K()       ((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_BLE_32K) == SYS_CTRL_CLK_EN0_BLE_32K)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_HF_CLK()       ((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_HF_CLK) == SYS_CTRL_CLK_EN0_BB_32K)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_LF_CLK()       ((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_LF_CLK)==SYS_CTRL_CLK_EN0_LF_CLK)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_DMA()          ((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_DMA)== SYS_CTRL_CLK_EN0_DMA)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_I2C0()         (((SYS_CTRL->CLK_EN0&SYS_CTRL_CLK_EN0_I2C0) == SYS_CTRL_CLK_EN0_I2C0) && \
                                                    ((SYS_CTRL->CLK_EN1&SYS_CTRL_CLK_EN1_I2C0)== SYS_CTRL_CLK_EN1_I2C0))

#define __MS_CLOCK_LL_CLK_IS_ENABLE_I2C1()         (((SYS_CTRL->CLK_EN0&SYS_CTRL_CLK_EN0_I2C1)== SYS_CTRL_CLK_EN0_I2C1) && \
                                                    ((SYS_CTRL->CLK_EN1&SYS_CTRL_CLK_EN1_I2C1)==SYS_CTRL_CLK_EN1_I2C1))

#define __MS_CLOCK_LL_CLK_IS_ENABLE_UART0()        (((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_UART0)== SYS_CTRL_CLK_EN0_UART0) && \
                                                    ((SYS_CTRL->CLK_EN1 & SYS_CTRL_CLK_EN1_UART0) == SYS_CTRL_CLK_EN1_UART0))

#define __MS_CLOCK_LL_CLK_IS_ENABLE_UART1()        (((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_UART1) == SYS_CTRL_CLK_EN0_UART1)&& \
                                                    ((SYS_CTRL->CLK_EN1&SYS_CTRL_CLK_EN1_UART1) == SYS_CTRL_CLK_EN1_UART1))

#define __MS_CLOCK_LL_CLK_IS_ENABLE_SPI0()         (((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_SPI0) == SYS_CTRL_CLK_EN0_SPI0)&&\
                                                    ((SYS_CTRL->CLK_EN1&SYS_CTRL_CLK_EN1_SPI0)==SYS_CTRL_CLK_EN1_SPI0))

#define __MS_CLOCK_LL_CLK_IS_ENABLE_SPI1()         (((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_SPI1) == SYS_CTRL_CLK_EN0_SPI1) &&\
                                                    ((SYS_CTRL->CLK_EN1 & SYS_CTRL_CLK_EN1_SPI1) == SYS_CTRL_CLK_EN1_SPI1))

#define __MS_CLOCK_LL_CLK_IS_ENABLE_TIMERx(x)      ((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_TIMER##x) == SYS_CTRL_CLK_EN0_TIMER##x)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_KSCAN()        (((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_KSCAN) == SYS_CTRL_CLK_EN0_KSCAN)&&\
                                                    ((SYS_CTRL->CLK_EN1 & SYS_CTRL_CLK_EN1_KSCAN)==SYS_CTRL_CLK_EN1_KSCAN))

#define __MS_CLOCK_LL_CLK_IS_ENABLE_PWM()          (((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_PWM) == SYS_CTRL_CLK_EN0_PWM)&&\
                                                    ((SYS_CTRL->CLK_EN1 & SYS_CTRL_CLK_EN1_PWM)==SYS_CTRL_CLK_EN1_PWM))

#define __MS_CLOCK_LL_CLK_IS_ENABLE_IR()           (((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_IR) == SYS_CTRL_CLK_EN0_IR)&&\
                                                    ((SYS_CTRL->CLK_EN1 & SYS_CTRL_CLK_EN1_IR)==SYS_CTRL_CLK_EN1_IR))

#define __MS_CLOCK_LL_CLK_IS_ENABLE_QSPI_REF()     ((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_QSPI_REF)==SYS_CTRL_CLK_EN0_QSPI_REF)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_MDM()          ((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_MDM)==SYS_CTRL_CLK_EN0_MDM)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_BLE()          ((SYS_CTRL->CLK_EN0 & SYS_CTRL_CLK_EN0_BLE)==SYS_CTRL_CLK_EN0_BLE)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_GPIO_PCLK()    ((SYS_CTRL->CLK_EN1 & SYS_CTRL_CLK_EN1_GPIO)==SYS_CTRL_CLK_EN1_GPIO)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_CPU_TIMER()    ((SYS_CTRL->CLK_EN1 & SYS_CTRL_CLK_EN1_CPU_TIMER)==SYS_CTRL_CLK_EN1_CPU_TIMER)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_SYSC()         ((SYS_CTRL->CLK_EN1 & SYS_CTRL_CLK_EN1_SYS)==SYS_CTRL_CLK_EN1_SYS)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_TRNG()         ((SYS_CTRL->CLK_EN1 & SYS_CTRL_CLK_EN1_TRNG)==SYS_CTRL_CLK_EN1_TRNG)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_WDT()          ((SYS_CTRL->CLK_EN1 & SYS_CTRL_CLK_EN1_WDT)==SYS_CTRL_CLK_EN1_WDT)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_AUD_ADC()      ((SYS_CTRL->CLK_EN1 & SYS_CTRL_CLK_EN1_AUD)==SYS_CTRL_CLK_EN1_AUD)

#define __MS_CLOCK_LL_CLK_IS_ENABLE_AUX_ADC()      ((SYS_CTRL->CLK_EN1 & SYS_CTRL_CLK_EN1_AUX)==SYS_CTRL_CLK_EN1_AUX)

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
static inline void ms_clock_ll_hf_osc_sel(uint32_t hf_osc_soucre)
{
    MODIFY_REG(SYS_CTRL->CLK_SEL, SYS_CTRL_CLK_SEL_HF_OSC_CLK, hf_osc_soucre);
}

/**
 * @brief  get the hf_osc source
 * @retval The current hf_osc source
 */
static inline uint32_t ms_clock_ll_get_hf_osc_src(void)
{
    return READ_BIT(SYS_CTRL->CLK_SEL, SYS_CTRL_CLK_SEL_HF_OSC_CLK);
}

/**
 * @brief  select the ahb high clk source
 * @param  uint32_t ahb_high_clk_soucre: This parameter can be one of the following values:
 *         HF_CLK_SEL_PLL_CLK    : select the clk output of  pll
 *         HF_CLK_SEL_BYPASS_PLL : select the clk bypass pll
 * @retval None
 */
static inline void ms_clock_ll_hf_clk_sel(uint32_t hf_clk_soucre)
{
    MODIFY_REG(SYS_CTRL->CLK_SEL, SYS_CTRL_CLK_SEL_HF_CLK, hf_clk_soucre);
}

/**
 * @brief  get the hf_clk source
 * @retval The current hf_clk source
 */
static inline uint32_t ms_clock_ll_get_hf_clk_src(void)
{
    return READ_BIT(SYS_CTRL->CLK_SEL, SYS_CTRL_CLK_SEL_HF_CLK);
}

/**
 * @brief  select the main clk source
 * @param  uint32_t main_clk_soucre: This parameter can be one of the following values:
 *         SYS_CLK_SEL_LF_CLK	: select the sys_lf_clk
 *         SYS_CLK_SEL_HF_CLK	: select the sys_hf_clk
 * @retval None
 */
static inline void ms_clock_ll_sys_clk_sel(uint32_t sys_clk_soucre)
{
    MODIFY_REG(SYS_CTRL->CLK_SEL, SYS_CTRL_CLK_SEL_SYS_CLK, sys_clk_soucre);
}

/**
 * @brief  get the sys_clk source
 * @retval The current sys_clk source
 */
static inline uint32_t ms_clock_ll_get_sys_clk_src(void)
{
    return READ_BIT(SYS_CTRL->CLK_SEL, SYS_CTRL_CLK_SEL_SYS_CLK);
}

/**
 * @brief  select the low clk source
 * @param  uint32_t low_clk_soucre: This parameter can be one of the following values:
 *         LF_CLK_SEL_RC_OSC32K		    : select rc osc32k
 *         LF_CLK_SEL_EXT_OSC24M	    : select the division clk of  the external high osc
 * @retval None
 */
__STATIC_FORCEINLINE void ms_clock_ll_lf_clk_sel(uint32_t lf_clk_source)
{
    MODIFY_REG(SYS_CTRL->CLK_SEL, SYS_CTRL_CLK_SEL_LF_CLK, lf_clk_source);
}

/**
 * @brief  get the lf_clk source
 * @retval The current lf_clk source
 */
static inline uint32_t ms_clock_ll_get_lf_clk_src(void)
{
    return READ_BIT(SYS_CTRL->CLK_SEL, SYS_CTRL_CLK_SEL_LF_CLK);
}

/**
 * @brief  select the gpio_fclk
 * @param  uint32_t gpio_fclk_source:This parameter can be one of the following values:
 *           @arg @ref  GPIO_CLK_SEL_PCLK          Select the pclk as the GPIO Clock
 *           @arg @ref  GPIO_CLK_SEL_RC_CLK        Select the rc clock as the GPIO Clock
 * @retval None
 */
static inline void ms_clock_ll_gpio_fclk_sel(uint32_t gpio_fclk_source)
{
    MODIFY_REG(SYS_CTRL->CLK_SEL, SYS_CTRL_CLK_SEL_GPIO_FCLK, gpio_fclk_source);
}

/**
 * @brief  get the gpio fclk source
 * @retval The current gpio fclk source
 */
static inline uint32_t ms_clock_ll_get_gpio_fclk_src(void)
{
    return READ_BIT(SYS_CTRL->CLK_SEL, SYS_CTRL_CLK_SEL_GPIO_FCLK);
}

/**
 * @brief  select the uart2_fclk
 * @param  uint32_t uart2_fclk_source: This parameter can be one of the following values:
 *           @arg @ref  UART2_CLK_SEL_PCLK          Select the pclk as the UART2 Clock
 *           @arg @ref  UART2_CLK_SEL_RC_CLK        Select the rc clock as the UART2 Clock
 * @retval None
 */
static inline void ms_clock_ll_uart2_fclk_sel(uint32_t uart2_fclk_source)
{
    MODIFY_REG(SYS_CTRL->CLK_SEL, SYS_CTRL_CLK_SEL_UART2_PCLK, uart2_fclk_source);
}

/**
 * @brief  get the uart2 fclk source
 * @retval The current uart2 fclk source
 */
static inline uint32_t ms_clock_ll_get_uart2_fclk_src(void)
{
    return READ_BIT(SYS_CTRL->CLK_SEL, SYS_CTRL_CLK_SEL_UART2_PCLK);
}

/**
 * @brief  configure the params of pll
 * @param  uint32_t rtc_fclk_source:This parameter can be one of the following values:
 *           @arg @ref  RTC_CLK_SEL_PCLK          Select the pclk as the RTC Clock
 *           @arg @ref  RTC_CLK_SEL_RC_CLK        Select the rc clock as the RTC Clock
 * @retval None
 */
static inline void ms_clock_ll_rtc_fclk_sel(uint32_t rtc_fclk_source)
{
    MODIFY_REG(SYS_CTRL->CLK_SEL, SYS_CTRL_CLK_SEL_RTC_FPCLK, rtc_fclk_source);
}

/**
 * @brief  get the rtc fclk source
 * @retval The current rtc fclk source
 */
static inline uint32_t ms_clock_ll_get_rtc_fclk_src(void)
{
    return READ_BIT(SYS_CTRL->CLK_SEL, SYS_CTRL_CLK_SEL_RTC_FPCLK);
}

/**
 * @brief  Enable the pll unit
 * @retval None
 */
static inline void ms_clock_ll_pll_enable(void)
{
    SET_BIT(SYS_CTRL->LP_KEEP, SYS_CTRL_PLL_EN_SW);
}

/**
 * @brief  Disable the pll unit
 * @retval None
 */
static inline void ms_clock_ll_pll_disable(void)
{
    CLEAR_BIT(SYS_CTRL->LP_KEEP, SYS_CTRL_PLL_EN_SW);
}

/**
 * @brief  Configure the params of pll
 * @param  uint32_t pre_div: 1~31, the valid value: 1,2,4,8,16
 *         uint32_t mul_pll: 10~511, the value less than 9 is forbidden
 * @retval None
 */
static inline void ms_clock_ll_pll_config(uint32_t pre_div, uint32_t mul_pll)
{
    MODIFY_REG(SYS_CTRL->PLL_CTRL, SYS_CTRL_PLL_CTRL_PDIV,
            ((pre_div << SYS_CTRL_PLL_CTRL_PDIV_POS)&SYS_CTRL_PLL_CTRL_PDIV_MASK));
    MODIFY_REG(SYS_CTRL->PLL_CTRL, SYS_CTRL_PLL_CTRL_FDIV,
            ((mul_pll << SYS_CTRL_PLL_CTRL_FDIV_POS)&SYS_CTRL_PLL_CTRL_FDIV_MASK));
}

/**
 * @brief  verify the pll lock status
 * @retval true  : lock
 *         false : unlock
 */
static inline bool ms_clock_ll_pll_is_locked(void)
{
    return (READ_BIT(SYS_CTRL->PLL_CTRL,SYS_CTRL_PLL_CTRL_LOCK) == SYS_CTRL_PLL_CTRL_LOCK);
}

/**
 * @brief  set the ahb division
 * @param  uint32_t ahb_div: 0~15, the valid value: 0,1,2,4,8
 * @retval None
 */
static inline void ms_clock_ll_set_ahb_div(uint32_t ahb_div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV3, SYS_CTRL_CLK_DIV3_AHB,
            ((ahb_div << SYS_CTRL_CLK_DIV3_AHB_POS) & SYS_CTRL_CLK_DIV3_AHB_MASK));
}

/**
 * @brief  get the apb division
 * @retval The current ahb clock division
 */
static inline uint32_t ms_clock_ll_get_ahb_div(void)
{
    return (READ_REG(SYS_CTRL->CLK_DIV3) & SYS_CTRL_CLK_DIV3_APB_MASK) >> SYS_CTRL_CLK_DIV3_APB_POS;
}

/**
 * @brief  set the apb division
 * @param  uint32_t apb_div: 0~15, the valid value: 0,1,2,4,8
 * @retval None
 */
static inline void ms_clock_ll_set_apb_div(uint32_t apb_div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV3, SYS_CTRL_CLK_DIV3_APB,
            ((apb_div << SYS_CTRL_CLK_DIV3_APB_POS) & SYS_CTRL_CLK_DIV3_APB_MASK));
}

/**
 * @brief  get the apb division
 * @retval The current apb division
 */
static inline uint32_t ms_clock_ll_get_apb_div(void)
{
    return (READ_REG(SYS_CTRL->CLK_DIV3) & SYS_CTRL_CLK_DIV3_APB_MASK) >> SYS_CTRL_CLK_DIV3_APB_POS;
}

/**
 * @brief  set the peripheral pre division
 * @param  uint32_t pre_div: 0~15, the valid value: 0,1,2,4,8
 * @retval None
 */
static inline void ms_clock_ll_set_periph_pre_div(uint32_t pre_div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV6, SYS_CTRL_CLK_DIV6_PERI_PRE,
            ((pre_div << SYS_CTRL_CLK_DIV6_PERI_PRE_POS) & SYS_CTRL_CLK_DIV6_PERI_PRE_MASK));
}

/**
 * @brief  get the peripheral pre division
 * @retval The current peripheral pre division
 */
static inline uint32_t ms_clock_ll_get_periph_pre_div(void)
{
    return (READ_REG(SYS_CTRL->CLK_DIV6) & SYS_CTRL_CLK_DIV6_PERI_PRE_MASK) >> SYS_CTRL_CLK_DIV6_PERI_PRE_POS;
}

/**
 * @brief  set the ext high osc division
 * @param  uint32_t osc_div: 0~15, the valid value: 0,1,2,4,8
 * @retval None
 */
static inline void ms_clock_ll_set_ext_high_osc_div(uint32_t osc_div)
{
    MODIFY_REG(SYS_CTRL->EXT_OSC24M_CFG, SYS_CTRL_EXT_OSC24M_CFG_DIV,
            ((osc_div << SYS_CTRL_EXT_OSC24M_CFG_DIV_POS) & SYS_CTRL_EXT_OSC24M_CFG_DIV_MASK));
}

/**
 * @brief  set the ext high osc division
 * @retval The current ext high osc division
 */
static inline uint32_t ms_clock_ll_get_ext_high_osc_div(void)
{
    return (READ_REG(SYS_CTRL->EXT_OSC24M_CFG) & SYS_CTRL_EXT_OSC24M_CFG_DIV_MASK) >> SYS_CTRL_EXT_OSC24M_CFG_DIV_POS;
}

/**
 * @brief  trigger after setting the ext osc division
 * @retval None
 */
static inline void ms_clock_ll_set_ext_high_osc_div_toggle(void)
{
    SET_BIT(SYS_CTRL->EXT_OSC24M_CFG, SYS_CTRL_EXT_OSC24M_CFG_DIV_TOG);
}




/**
 * @brief  set i2s0 mdiv
 * @param  uint32_t mdiv
 * @retval None
 */
static inline void ms_clock_ll_set_i2s0_mdiv(uint32_t mdiv)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV0, SYS_CTRL_CLK_DIV0_I2S0_MDIV,
            (mdiv <<SYS_CTRL_CLK_DIV0_I2S0_MDIV_POS) & SYS_CTRL_CLK_DIV0_I2S0_MDIV_MASK);
}

/**
 * @brief  get i2s0 mdiv
 * @retval The setting i2s0 mdiv
 */
static inline uint32_t ms_clock_ll_get_i2s0_mdiv(void)
{
    return (READ_REG(SYS_CTRL->CLK_DIV0) & SYS_CTRL_CLK_DIV0_I2S0_MDIV_MASK) >> SYS_CTRL_CLK_DIV0_I2S0_MDIV_POS;
}

/**
 * @brief  set i2s1 mdiv
 * @param  uint32_t mdiv
 * @retval None
 */
static inline void ms_clock_ll_set_i2s1_mdiv(uint32_t mdiv)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV0, SYS_CTRL_CLK_DIV0_I2S1_MDIV,
            (mdiv <<SYS_CTRL_CLK_DIV0_I2S0_MDIV_POS) & SYS_CTRL_CLK_DIV0_I2S1_MDIV_MASK);
}

/**
 * @brief  get i2s1 mdiv
 * @retval The setting i2s1 mdiv
 */
static inline uint32_t ms_clock_ll_get_i2s1_mdiv(void)
{
    return (READ_REG(SYS_CTRL->CLK_DIV0) & SYS_CTRL_CLK_DIV0_I2S1_MDIV_MASK) >> SYS_CTRL_CLK_DIV0_I2S1_MDIV_POS;
}

/**
 * @brief  set i2s0 div
 * @param  uint32_t div
 * @retval None
 */
static inline void ms_clock_ll_set_i2s0_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV0, SYS_CTRL_CLK_DIV0_I2S0_DIV,
            (div <<SYS_CTRL_CLK_DIV0_I2S0_DIV_POS) & SYS_CTRL_CLK_DIV0_I2S0_DIV_MASK);
}

/**
 * @brief  get i2s0 div
 * @retval The setting i2s0 div
 */
static inline uint32_t ms_clock_ll_get_i2s0_div(void)
{
    return (READ_REG(SYS_CTRL->CLK_DIV0) & SYS_CTRL_CLK_DIV0_I2S0_DIV_MASK) >> SYS_CTRL_CLK_DIV0_I2S0_DIV_POS;
}

/**
 * @brief  set i2s1 div
 * @param  uint32_t div
 * @retval None
 */
static inline void ms_clock_ll_set_i2s1_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV0, SYS_CTRL_CLK_DIV0_I2S1_DIV,
            (div <<SYS_CTRL_CLK_DIV0_I2S1_DIV_POS) & SYS_CTRL_CLK_DIV0_I2S1_DIV_MASK);
}

/**
 * @brief  get i2s1 div
 * @retval The setting i2s0 div
 */
static inline uint32_t ms_clock_ll_get_i2s1_div(void)
{
    return (READ_REG(SYS_CTRL->CLK_DIV0) & SYS_CTRL_CLK_DIV0_I2S1_DIV_MASK) >> SYS_CTRL_CLK_DIV0_I2S1_DIV_POS;
}

/**
 * @brief  set UART0 div
 * @param  uint32_t div
 * @retval None
 */
static inline void ms_clock_ll_set_uart0_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV1, SYS_CTRL_CLK_DIV1_UART0,
            (div <<SYS_CTRL_CLK_DIV1_UART0_POS) & SYS_CTRL_CLK_DIV1_UART0_MASK);
}

/**
 * @brief  get UART0 div
 * @retval The setting UART0 div
 */
static inline uint32_t ms_clock_ll_get_uart0_div(void)
{
    return (READ_REG(SYS_CTRL->CLK_DIV1) & SYS_CTRL_CLK_DIV1_UART0_MASK) >> SYS_CTRL_CLK_DIV1_UART0_POS;
}



/**
 * @brief  set UART1 div
 * @param  uint32_t div
 * @retval None
 */
static inline void ms_clock_ll_set_uart1_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV1, SYS_CTRL_CLK_DIV1_UART0,
            (div <<SYS_CTRL_CLK_DIV1_UART1_POS) & SYS_CTRL_CLK_DIV1_UART1_MASK);
}

/**
 * @brief  get UART1 div
 * @retval The setting UART1 div
 */
static inline uint32_t ms_clock_ll_get_uart1_div(void)
{
    return (READ_REG(SYS_CTRL->CLK_DIV1) & SYS_CTRL_CLK_DIV1_UART1_MASK) >> SYS_CTRL_CLK_DIV1_UART1_POS;
}


/**
 * @brief  set spi0 div
 * @retval none
 */
static inline void ms_clock_ll_set_spi0_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV1, SYS_CTRL_CLK_DIV1_SPI0,
            (div <<SYS_CTRL_CLK_DIV1_SPI0_POS) & SYS_CTRL_CLK_DIV1_SPI0_MASK);
}

/**
 * @brief  set spi1 div
 * @retval none
 */
static inline void ms_clock_ll_set_spi1_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV1, SYS_CTRL_CLK_DIV1_SPI1,
            (div <<SYS_CTRL_CLK_DIV1_SPI1_POS) & SYS_CTRL_CLK_DIV1_SPI1_MASK);
}

/**
 * @brief  set timer0 div
 * @retval none
 */
static inline void ms_clock_ll_set_timer0_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV1, SYS_CTRL_CLK_DIV1_TIMER0,
            (div <<SYS_CTRL_CLK_DIV1_TIMER0_POS) & SYS_CTRL_CLK_DIV1_TIMER0_MASK);
}

/**
 * @brief  set time1 div
 * @retval none
 */
static inline void ms_clock_ll_set_timer1_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV1, SYS_CTRL_CLK_DIV1_TIMER1,
            (div <<SYS_CTRL_CLK_DIV1_TIMER1_POS) & SYS_CTRL_CLK_DIV1_TIMER1_MASK);
}

/**
 * @brief  set time2 div
 * @retval none
 */
static inline void ms_clock_ll_set_timer2_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV2, SYS_CTRL_CLK_DIV2_TIMER2,
            (div <<SYS_CTRL_CLK_DIV2_TIMER2_POS) & SYS_CTRL_CLK_DIV2_TIMER2_MASK);
}


/**
 * @brief  set time3 div
 * @retval none
 */
static inline void ms_clock_ll_set_timer3_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV2, SYS_CTRL_CLK_DIV2_TIMER3,
            (div <<SYS_CTRL_CLK_DIV2_TIMER3_POS) & SYS_CTRL_CLK_DIV2_TIMER3_MASK);
}

/**
 * @brief  set time4 div
 * @retval none
 */
static inline void ms_clock_ll_set_timer4_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV2, SYS_CTRL_CLK_DIV2_TIMER4,
            (div <<SYS_CTRL_CLK_DIV2_TIMER4_POS) & SYS_CTRL_CLK_DIV2_TIMER4_MASK);
}
/**
 * @brief  set time5 div
 * @retval none
 */
static inline void ms_clock_ll_set_timer5_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV2, SYS_CTRL_CLK_DIV2_TIMER5,
            (div <<SYS_CTRL_CLK_DIV2_TIMER5_POS) & SYS_CTRL_CLK_DIV2_TIMER5_MASK);
}

/**
 * @brief  set time6 div
 * @retval none
 */
static inline void ms_clock_ll_set_timer6_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV2, SYS_CTRL_CLK_DIV2_TIMER6,
            (div <<SYS_CTRL_CLK_DIV2_TIMER6_POS) & SYS_CTRL_CLK_DIV2_TIMER6_MASK);
}


/**
 * @brief  set time7 div
 * @retval none
 */
static inline void ms_clock_ll_set_timer7_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV2, SYS_CTRL_CLK_DIV2_TIMER7,
            (div <<SYS_CTRL_CLK_DIV2_TIMER7_POS) & SYS_CTRL_CLK_DIV2_TIMER7_MASK);
}

/**
 * @brief  set pwm div
 * @retval none
 */
static inline void ms_clock_ll_set_pwm_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV2, SYS_CTRL_CLK_DIV2_PWM,
            (div <<SYS_CTRL_CLK_DIV2_PWM_POS) & SYS_CTRL_CLK_DIV2_PWM_MASK);
}

/**
 * @brief  set cpu timer div
 * @retval none
 */
static inline void ms_clock_ll_set_cpu_tmr_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV3, SYS_CTRL_CLK_DIV3_CPU_TMR,
            (div <<SYS_CTRL_CLK_DIV3_CPU_TMR_POS) & SYS_CTRL_CLK_DIV3_CPU_TMR_MASK);
}

/**
 * @brief  set ir div
 * @retval none
 */
static inline void ms_clock_ll_set_ir_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV3, SYS_CTRL_CLK_DIV3_IR,
            (div <<SYS_CTRL_CLK_DIV3_IR_POS) & SYS_CTRL_CLK_DIV3_IR_MASK);
}

/**
 * @brief  set watchdog div
 * @retval none
 */
static inline void ms_clock_ll_set_wdt_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV3, SYS_CTRL_CLK_DIV3_WDT,
            (div <<SYS_CTRL_CLK_DIV3_WDT_POS) & SYS_CTRL_CLK_DIV3_WDT_MASK);
}

/**
 * @brief  set aux adc div
 * @retval none
 */
static inline void ms_clock_ll_set_aux_adc_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV3, SYS_CTRL_CLK_DIV3_AUX_ADC,
            (div <<SYS_CTRL_CLK_DIV3_AUX_ADC_POS) & SYS_CTRL_CLK_DIV3_AUX_ADC_MASK);
}


/**
 * @brief  set modem 0 div
 * @retval none
 */
static inline void ms_clock_ll_set_mdm0_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV6, SYS_CTRL_CLK_DIV6_MDM0,
            (div <<SYS_CTRL_CLK_DIV6_MDM0_POS) & SYS_CTRL_CLK_DIV6_MDM0_MASK);
}

/**
 * @brief  set modem 0 div
 * @retval none
 */
static inline void ms_clock_ll_set_ble_div(uint32_t div)
{
    MODIFY_REG(SYS_CTRL->CLK_DIV6, SYS_CTRL_CLK_DIV6_BLE,
            (div <<SYS_CTRL_CLK_DIV6_BLE_POS) & SYS_CTRL_CLK_DIV6_BLE_MASK);
}

/**
 * @brief  set kscan div
 * @retval none
 */
static inline void ms_clock_ll_set_kscan_div(uint32_t div)
{
    WRITE_REG(SYS_CTRL->CLK_DIV4, div);
}

/**
 * @brief  set kscan div
 * @retval none
 */
static inline void ms_clock_ll_set_gpio_div(uint32_t div)
{
    WRITE_REG(SYS_CTRL->CLK_DIV5, div&0xFFFF);
}

/**
 * @brief  toggle the peripheral clk div
 * @param[in]  PeripheralClkToggle_Type clk:  peripheral clock type
 *            @ref PeripheralClkToggle_Type
 * @retval None
 */
static inline void ms_clock_ll_peripheral_clk_div_toggle(PeripheralClkToggle_Type clk)
{
    SET_BIT(SYS_CTRL->DIV_TOG, 1UL << clk);
}

/**
 * @brief  Enable the aux_adc unit
 * @retval None
 */
static inline void ms_clock_ll_aux_adc_pwr_on(void)
{
    SET_BIT(SYS_CTRL->UNIT_EN, SYS_CTRL_UNIT_EN_AUX_ADC_PU);
}


static inline void ms_clock_ll_BLE_clk_div_toggle()
{
    SET_BIT(SYS_CTRL->DIV_TOG, SYS_CTRL_BASEBAND_DIV_TOG);
}

static inline void ms_clock_ll_peripre_clk_div_toggle()
{
    SET_BIT(SYS_CTRL->DIV_TOG, SYS_CTRL_PERIPRE_DIV_TOG);
}


static inline void ms_clock_ll_MDM0_clk_div_toggle()
{
    SET_BIT(SYS_CTRL->DIV_TOG, SYS_CTRL_MDM0_DIV_TOG);
}



/** @}*/

/** @}*/
#ifdef __cplusplus
}
#endif
#endif /* BLE_SOC_XXX_MS_CLOCK_LL_H_ */
