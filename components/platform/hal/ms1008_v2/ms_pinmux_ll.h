
/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_pinmux_ll.h
 * @brief
 * @author bingrui.chen
  * @date 2022年1月18日
 * @version 1.0
  * @Revision: 
 */

#ifndef ms_pinmux_LL_H_
#define ms_pinmux_LL_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup MS_LL_Driver
 * @{
 */


/** @defgroup PINMUX_LL PINMUX
 * @{
 */

/** @defgroup PadNum_Type PIN Padding Type
 * @{
 */
typedef enum
{
    PAD0,
    PAD1,
    PAD2,
    PAD3,
    PAD4,
    PAD5,
    PAD6,
    PAD7,
    PAD8,
    PAD9,
    PAD10,
    PAD11,
    PAD12,
    PAD13,
    PAD14,
    PAD15,
    PAD16,
    PAD17,
    PAD18,
    PAD19,
    PAD20,
    PAD21,
    PAD22,
    PAD23,
    PAD24,
    PAD25,
    PAD26,
    PADMAX
}PadNum_Type;
/**
 * @}
 */

/*Pad pull type, every pad need 2 bit type(reference PadPull_Type)*/
#define PAD0_15_PULLTYPE(pad, type)      ((type) << (2*pad))
#define PAD16_26_PULLTYPE(pad, type)     ((type) << (2*(pad-PAD16)))


/** @defgroup Pin00Mode_Type PIN00 Padding Mode Type
 * @{
 */
typedef enum {
	PIN00_BOOT_SEL0,
	PIN00_GPIO_0,
	PIN00_PWM0,
	PIN00_SPI1_CLK,
	PIN00_IR,
	PIN00_I2C1_SCL,
	PIN00_KEY_ROW0,
	PIN00_KEY_COL0,
	PIN00_I2S0_SCLK,
	PIN00_DFX_RESERVE
} Pin00Mode_Type;
/**
 * @}
 */


/** @defgroup Pin01Mode_Type PIN01 Padding Mode Type
 * @{
 */
typedef enum {
	PIN01_BOOT_SEL1,
	PIN01_GPIO_1,
	PIN01_PWM1,
	PIN01_SPI1_CS,
	PIN01_IR,
	PIN01_I2C1_SDA,
	PIN01_KEY_ROW1,
	PIN01_KEY_COL1,
	PIN01_I2S0_LRCK,
	PIN01_DFX_RESERVE
} Pin01Mode_Type;
/**
 * @}
 */

/** @defgroup Pin02Mode_Type PIN02 Padding Mode Type
 * @{
 */
typedef enum {
	PIN02_GPIO_2,
	PIN02_UART0_TXD,
	PIN02_PWM2,
	PIN02_SPI0_CLK,
	PIN02_IR,
	PIN02_I2C0_SCL,
	PIN02_KEY_ROW2,
	PIN02_KEY_COL2,
	PIN02_I2S0_DI,
	PIN02_DFX_RESERVE
} Pin02Mode_Type;
/**
 * @}
 */

/** @defgroup Pin03Mode_Type PIN03 Padding Mode Type
 * @{
 */
typedef enum {
	PIN03_GPIO_3,
	PIN03_UART0_RXD,
	PIN03_PWM0,
	PIN03_SPI0_CS,
	PIN03_IR,
	PIN03_I2C0_SDA,
	PIN03_KEY_ROW3,
	PIN03_KEY_COL3,
	PIN03_I2S0_MCLK,
	PIN03_DFX_RESERVE
} Pin03Mode_Type;
/**
 * @}
 */

/** @defgroup Pin04Mode_Type PIN04 Padding Mode Type
 * @{
 */
typedef enum {
	PIN04_GPIO_4,
	PIN04_UART0_CTS,
	PIN04_PWM1,
	PIN04_SPI0_TXD,
	PIN04_IR,
	PIN04_I2C1_SCL,
	PIN04_KEY_ROW4,
	PIN04_KEY_COL4,
	PIN04_I2S0_DO,
	PIN04_DFX_RESERVE
} Pin04Mode_Type;
/**
 * @}
 */

/** @defgroup Pin05Mode_Type PIN05 Padding Mode Type
 * @{
 */
typedef enum {
	PIN05_GPIO_5,
	PIN05_UART0_RTS,
	PIN05_PWM2,
	PIN05_SPI0_RXD,
	PIN05_IR,
	PIN05_I2C1_SDA,
	PIN05_KEY_ROW5,
	PIN05_KEY_COL5,
	PIN05_I2S1_SCLK,
	PIN05_DFX_RESERVE
} Pin05Mode_Type;
/**
 * @}
 */

/** @defgroup Pin06Mode_Type PIN06 Padding Mode Type
 * @{
 */
typedef enum {
	PIN06_GPIO_6,
	PIN06_UART1_TXD,
	PIN06_PWM0,
	PIN06_SPI1_CLK,
	PIN06_IR,
	PIN06_I2C0_SCL,
	PIN06_KEY_ROW6,
	PIN06_KEY_COL6,
	PIN06_I2S1_LRCK,
	PIN06_DFX_RESERVE
} Pin06Mode_Type;
/**
 * @}
 */

/** @defgroup Pin07Mode_Type PIN07 Padding Mode Type
 * @{
 */
typedef enum {
	PIN07_GPIO_7,
	PIN07_UART1_RXD,
	PIN07_PWM1,
	PIN07_SPI1_CS,
	PIN07_IR,
	PIN07_I2C0_SDA,
	PIN07_KEY_ROW7,
	PIN07_KEY_COL7,
	PIN07_I2S1_DI,
	PIN07_DFX_RESERVE
} Pin07Mode_Type;
/**
 * @}
 */

/** @defgroup Pin08Mode_Type PIN08 Padding Mode Type
 * @{
 */
typedef enum {
	PIN08_GPIO_8,
	PIN08_UART1_CTS,
	PIN08_PWM2,
	PIN08_SPI0_TXD,
	PIN08_IR,
	PIN08_I2C1_SCL,
	PIN08_KEY_ROW0,
	PIN08_KEY_COL0,
	PIN08_I2S0_MCLK,
	PIN08_DFX_RESERVE
} Pin08Mode_Type;
/**
 * @}
 */

/** @defgroup Pin09Mode_Type PIN09 Padding Mode Type
 * @{
 */
typedef enum {
	PIN09_GPIO_9,
	PIN09_UART1_RTS,
	PIN09_PWM0,
	PIN09_SPI0_RXD,
	PIN09_IR,
	PIN09_I2C1_SDA,
	PIN09_KEY_ROW1,
	PIN09_KEY_COL1,
	PIN09_I2S0_DO,
	PIN09_DFX_RESERVE
} Pin09Mode_Type;
/**
 * @}
 */

/** @defgroup Pin10Mode_Type PIN10 Padding Mode Type
 * @{
 */
typedef enum {
    PIN10_UART2_TXD,
    PIN10_GPIO_10,
    PIN10_PWM1,
    PIN10_SPI0_CLK,
    PIN10_AUX_ADC0,
    PIN10_I2C0_SCL,
    PIN10_KEY_ROW2,
    PIN10_KEY_COL2,
    PIN10_I2S0_SCLK,
    PIN10_DFX_D0,
} Pin10Mode_Type;
/**
 * @}
 */

/** @defgroup Pin11Mode_Type PIN11 Padding Mode Type
 * @{
 */
typedef enum {
    PIN11_UART2_RXD,
    PIN11_GPIO_11,
    PIN11_PWM2,
    PIN11_SPI0_CS,
    PIN11_AUX_ADC1,
    PIN11_I2C0_SDA,
    PIN11_KEY_ROW3,
    PIN11_KEY_COL3,
    PIN11_I2S0_LRCK,
    PIN11_DFX_D1
} Pin11Mode_Type;
/**
 * @}
 */

/** @defgroup Pin12Mode_Type PIN12 Padding Mode Type
 * @{
 */
typedef enum {
	PIN12_GPIO_12,
	PIN12_UART2_CTS,
	PIN12_PWM0,
	PIN12_SPI0_TXD,
	PIN12_AUX_ADC2,
	PIN12_I2C1_SCL,
	PIN12_KEY_ROW4,
	PIN12_KEY_COL4,
	PIN12_I2S0_DI,
	PIN12_DFX_D2
} Pin12Mode_Type;

/**
 * @}
 */


/** @defgroup Pin13Mode_Type PIN13 Padding Mode Type
 * @{
 */
typedef enum {
	PIN13_GPIO_13,
	PIN13_UART2_TXD,
	PIN13_IR,
	PIN13_SPI0_RXD,
	PIN13_AUX_ADC3,
	PIN13_I2C1_SDA,
	PIN13_KEY_ROW5,
	PIN13_KEY_COL5,
	PIN13_I2S0_MCLK,
	PIN13_DFX_D3
} Pin13Mode_Type;
/**
 * @}
 */


/** @defgroup Pin14Mode_Type PIN14 Padding Mode Type
 * @{
 */
typedef enum {
	PIN14_GPIO_14,
	PIN14_UART0_TXD,
	PIN14_PWM2,
	PIN14_SPI1_CLK,
	PIN14_AUX_ADC4,
	PIN14_IR,
	PIN14_KEY_ROW6,
	PIN14_KEY_COL6,
	PIN14_I2S0_DO,
	PIN14_DFX_D4
} Pin14Mode_Type;
/**
 * @}
 */


/** @defgroup Pin15Mode_Type PIN15 Padding Mode Type
 * @{
 */
typedef enum {
	PIN15_GPIO_15,
	PIN15_UART0_RXD,
	PIN15_PWM0,
	PIN15_SPI1_CS,
	PIN15_AUX_ADC5,
	PIN15_IR,
	PIN15_KEY_ROW7,
	PIN15_KEY_COL7,
	PIN15_I2S1_SCLK,
	PIN15_DFX_D5
} Pin15Mode_Type;
/**
 * @}
 */

/** @defgroup Pin16Mode_Type PIN16 Padding Mode Type
 * @{
 */
typedef enum {
	PIN16_GPIO_16,
	PIN16_UART0_CTS,
	PIN16_PWM1,
	PIN16_SPI1_TXD,
	PIN16_AUX_ADC6,
	PIN16_IR,
	PIN16_KEY_ROW0,
	PIN16_KEY_COL0,
	PIN16_I2S1_LRCK,
	PIN16_DFX_RESERVE
} Pin16Mode_Type;
/**
 * @}
 */

/** @defgroup Pin17Mode_Type PIN17 Padding Mode Type
 * @{
 */
typedef enum {
	PIN17_GPIO_17,
	PIN17_UART0_RTS,
	PIN17_PWM2,
	PIN17_SPI1_RXD,
	PIN17_AUX_ADC7,
	PIN17_IR,
	PIN17_KEY_ROW1,
	PIN17_KEY_COL1,
	PIN17_I2S1_DI,
	PIN17_DFX_D6,
} Pin17Mode_Type;
/**
 * @}
 */

/** @defgroup Pin18Mode_Type PIN18 Padding Mode Type
 * @{
 */
typedef enum {
	PIN18_GPIO_18,
	PIN18_UART1_TXD,
	PIN18_PWM0,
	PIN18_SPI0_CLK,
	PIN18_AUX_ADC0,
	PIN18_IR,
	PIN18_KEY_ROW2,
	PIN18_KEY_COL2,
	PIN18_I2S1_MCLK,
	PIN18_DFX_RESERVE
} Pin18Mode_Type;
/**
 * @}
 */

/** @defgroup Pin19Mode_Type PIN19 Padding Mode Type
 * @{
 */
typedef enum {
	PIN19_GPIO_19,
	PIN19_UART1_RXD,
	PIN19_PWM1,
	PIN19_SPI0_CS,
	PIN19_AUX_ADC1,
	PIN19_IR,
	PIN19_KEY_ROW3,
	PIN19_KEY_COL3,
	PIN19_I2S1_DO,
	PIN19_DFX_RESERVE
} Pin19Mode_Type;
/**
 * @}
 */

/** @defgroup Pin20Mode_Type PIN20 Padding Mode Type
 * @{
 */
typedef enum {
	PIN20_GPIO_20,
	PIN20_UART1_CTS,
	PIN20_PWM2,
	PIN20_SPI0_TXD,
	PIN20_AUX_ADC2,
	PIN20_IR,
	PIN20_KEY_ROW4,
	PIN20_KEY_COL4,
	PIN20_I2C0_SCL,
	PIN20_DFX_RESERVE
} Pin20Mode_Type;
/**
 * @}
 */

/** @defgroup Pin21Mode_Type PIN21 Padding Mode Type
 * @{
 */
typedef enum {
	PIN21_GPIO_21,
	PIN21_UART1_RTS,
	PIN21_PWM0,
	PIN21_SPI0_RXD,
	PIN21_AUX_ADC3,
	PIN21_I2C0_SCL,
	PIN21_KEY_ROW5,
	PIN21_KEY_COL5,
	PIN21_I2C0_SDA,
	PIN21_DFX_RESERVE
} Pin21Mode_Type;
/**
 * @}
 */


/** @defgroup Pin22Mode_Type PIN22 Padding Mode Type
 * @{
 */
typedef enum {
	PIN22_GPIO_22,
	PIN22_UART1_TXD,
	PIN22_PWM1,
	PIN22_IR,
	PIN22_AUX_ADC4,
	PIN22_I2C0_SDA,
	PIN22_KEY_ROW6,
	PIN22_KEY_COL6,
	PIN22_I2S0_SCLK,
	PIN22_DFX_RESERVE
} Pin22Mode_Type;
/**
 * @}
 */

/** @defgroup Pin23Mode_Type PIN23 Padding Mode Type
 * @{
 */
typedef enum {
	PIN23_GPIO_23,
	PIN23_UART2_RXD,
	PIN23_PWM2,
	PIN23_SWC,
	PIN23_AUX_ADC5,
	PIN23_I2C1_SCL,
	PIN23_KEY_ROW7,
	PIN23_KEY_COL7,
	PIN23_I2S0_LRCK,
	PIN23_DFX_RESERVE
} Pin23Mode_Type;
/**
 * @}
 */

/** @defgroup Pin24Mode_Type PIN24 Padding Mode Type
 * @{
 */
typedef enum {
	PIN24_GPIO_24,
	PIN24_UART0_RTS,
	PIN24_PWM0,
	PIN24_SWD,
	PIN24_AUX_ADC6,
	PIN24_I2C1_SDA,
	PIN24_KEY_ROW0,
	PIN24_KEY_COL0,
	PIN24_I2S0_DI,
	PIN24_DFX_RESERVE
} Pin24Mode_Type;
/**
 * @}
 */

/** @defgroup Pin25Mode_Type PIN25 Padding Mode Type
 * @{
 */
typedef enum {
	PIN25_SWC,
	PIN25_UART2_RTS,
	PIN25_PWM1,
	PIN25_SPI1_TXD,
	PIN25_AUX_ADC7,
	PIN25_GPIO_25,
	PIN25_KEY_ROW1,
	PIN25_KEY_COL1,
	PIN25_I2S0_MCLK,
	PIN25_DFX_RESERVE
} Pin25Mode_Type;
/**
 * @}
 */

/** @defgroup Pin26Mode_Type PIN26 Padding Mode Type
 * @{
 */
typedef enum {
	PIN26_SWD,
	PIN26_IR,
	PIN26_PWM2,
	PIN26_SPI1_RXD,
	PIN26_AUX_ADC0,
	PIN26_GPIO_26,
	PIN26_KEY_ROW2,
	PIN26_KEY_COL2,
	PIN26_I2S0_DO,
	PIN26_DFX_D7
} Pin26Mode_Type;
/**
 * @}
 */

/** @defgroup PadPull_Type Padding PULL Type
 * @{
 */
typedef enum
{
    PULL_DEFAULT = 0x00,
    PULL_UP,
    PULL_DOWN,
    PULL_NONE
}PadPull_Type;
/**
 * @}
 */

/** @defgroup PadDS_Type Padding DS Type
 * @{
 */
typedef enum
{
    DS1DS0_00 = 0x00,
    DS1DS0_01,
    DS1DS0_10,
    DS1DS0_11
}PadDS_Type;
/**
 * @}
 */

/**
 * @brief  configure the pin00  function
 * @param[in]  Pin00Mode_Type mode: the function of pin00.
 *                              @ref Pin00Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin00_usage(Pin00Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE0, SYS_CTRL_GPIO_MODE0_IO0,
            (mode << SYS_CTRL_GPIO_MODE0_IO0_POS) & SYS_CTRL_GPIO_MODE0_IO0_MASK);
}

/**
 * @brief  configure the pin01  function
 * @param[in]  Pin00Mode_Type mode: the function of pin01.
 *                              @ref Pin01Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin01_usage(Pin01Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE0, SYS_CTRL_GPIO_MODE0_IO1,
            (mode << SYS_CTRL_GPIO_MODE0_IO1_POS) & SYS_CTRL_GPIO_MODE0_IO1_MASK);
}

/**
 * @brief  configure the pin02  function
 * @param[in]  Pin02Mode_Type mode: the function of pin02.
 *                              @ref Pin02Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin02_usage(Pin02Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE0, SYS_CTRL_GPIO_MODE0_IO2,
            (mode << SYS_CTRL_GPIO_MODE0_IO2_POS) & SYS_CTRL_GPIO_MODE0_IO2_MASK);
}

/**
 * @brief  configure the pin03  function
 * @param[in]  Pin03Mode_Type mode: the function of pin03.
 *                              @ref Pin03Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin03_usage(Pin03Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE0, SYS_CTRL_GPIO_MODE0_IO3,
            (mode << SYS_CTRL_GPIO_MODE0_IO3_POS) & SYS_CTRL_GPIO_MODE0_IO3_MASK);
}


/**
 * @brief  configure the pin04  function
 * @param[in]  Pin04Mode_Type mode: the function of pin04.
 *                              @ref Pin04Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin04_usage(Pin04Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE0, SYS_CTRL_GPIO_MODE0_IO4,
            (mode << SYS_CTRL_GPIO_MODE0_IO4_POS) & SYS_CTRL_GPIO_MODE0_IO4_MASK);
}


/**
 * @brief  configure the pin05  function
 * @param[in]  Pin05Mode_Type mode: the function of pin05.
 *                              @ref Pin05Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin05_usage(Pin05Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE0, SYS_CTRL_GPIO_MODE0_IO5,
            (mode << SYS_CTRL_GPIO_MODE0_IO5_POS) & SYS_CTRL_GPIO_MODE0_IO5_MASK);
}


/**
 * @brief  configure the pin06  function
 * @param[in]  Pin06Mode_Type mode: the function of pin06.
 *                              @ref Pin06Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin06_usage(Pin06Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE0, SYS_CTRL_GPIO_MODE0_IO6,
            (mode << SYS_CTRL_GPIO_MODE0_IO6_POS) & SYS_CTRL_GPIO_MODE0_IO6_MASK);
}


/**
 * @brief  configure the pin07  function
 * @param[in]  Pin07Mode_Type mode: the function of pin07.
 *                              @ref Pin07Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin07_usage(Pin07Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE0, SYS_CTRL_GPIO_MODE0_IO7,
            (mode << SYS_CTRL_GPIO_MODE0_IO7_POS) & SYS_CTRL_GPIO_MODE0_IO7_MASK);
}

/**
 * @brief  configure the pin08  function
 * @param[in]  Pin08Mode_Type mode: the function of pin08.
 *                              @ref Pin08Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin08_usage(Pin08Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE1, SYS_CTRL_GPIO_MODE1_IO8,
            (mode << SYS_CTRL_GPIO_MODE1_IO8_POS) & SYS_CTRL_GPIO_MODE1_IO8_MASK);
}

/**
 * @brief  configure the pin09  function
 * @param[in]  Pin09Mode_Type mode: the function of pin09.
 *                              @ref Pin08Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin09_usage(Pin09Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE1, SYS_CTRL_GPIO_MODE1_IO9,
            (mode << SYS_CTRL_GPIO_MODE1_IO9_POS) & SYS_CTRL_GPIO_MODE1_IO9_MASK);
}

/**
 * @brief  configure the pin10  function
 * @param[in]  Pin10Mode_Type mode: the function of pin10.
 *                              @ref Pin10Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin10_usage(Pin10Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE1, SYS_CTRL_GPIO_MODE1_IO10,
            (mode << SYS_CTRL_GPIO_MODE1_IO10_POS) & SYS_CTRL_GPIO_MODE1_IO10_MASK);
}

/**
 * @brief  configure the pin11  function
 * @param[in]  Pin11Mode_Type mode: the function of pin11.
 *                              @ref Pin11Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin11_usage(Pin11Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE1, SYS_CTRL_GPIO_MODE1_IO11,
            (mode << SYS_CTRL_GPIO_MODE1_IO11_POS) & SYS_CTRL_GPIO_MODE1_IO11_MASK);
}

/**
 * @brief  configure the pin12  function
 * @param[in]  Pin12Mode_Type mode: the function of pin12.
 *                              @ref Pin12Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin12_usage(Pin12Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE1, SYS_CTRL_GPIO_MODE1_IO12,
            (mode << SYS_CTRL_GPIO_MODE1_IO12_POS) & SYS_CTRL_GPIO_MODE1_IO12_MASK);
}


/**
 * @brief  configure the pin13  function
 * @param[in]  Pin13Mode_Type mode: the function of pin13.
 *                              @ref Pin13Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin13_usage(Pin13Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE1, SYS_CTRL_GPIO_MODE1_IO13,
            (mode << SYS_CTRL_GPIO_MODE1_IO13_POS) & SYS_CTRL_GPIO_MODE1_IO13_MASK);
}


/**
 * @brief  configure the pin14  function
 * @param[in]  Pin14Mode_Type mode: the function of pin14.
 *                              @ref Pin14Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin14_usage(Pin14Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE1, SYS_CTRL_GPIO_MODE1_IO14,
            (mode << SYS_CTRL_GPIO_MODE1_IO14_POS) & SYS_CTRL_GPIO_MODE1_IO14_MASK);
}

/**
 * @brief  configure the pin15  function
 * @param[in]  Pin15Mode_Type mode: the function of pin15.
 *                              @ref Pin15Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin15_usage(Pin15Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE1, SYS_CTRL_GPIO_MODE1_IO15,
            (mode << SYS_CTRL_GPIO_MODE1_IO15_POS) & SYS_CTRL_GPIO_MODE1_IO15_MASK);
}

/**
 * @brief  configure the pin16  function
 * @param[in]  Pin16Mode_Type mode: the function of pin16.
 *                              @ref Pin16Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin16_usage(Pin16Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE2, SYS_CTRL_GPIO_MODE2_IO16,
            (mode << SYS_CTRL_GPIO_MODE2_IO16_POS) & SYS_CTRL_GPIO_MODE2_IO16_MASK);
}

/**
 * @brief  configure the pin17  function
 * @param[in]  Pin17Mode_Type mode: the function of pin17.
 *                              @ref Pin17Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin17_usage(Pin17Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE2, SYS_CTRL_GPIO_MODE2_IO17,
            (mode << SYS_CTRL_GPIO_MODE2_IO17_POS) & SYS_CTRL_GPIO_MODE2_IO17_MASK);
}


/**
 * @brief  configure the pin18  function
 * @param[in]  Pin18Mode_Type mode: the function of pin18.
 *                              @ref Pin18Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin18_usage(Pin18Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE2, SYS_CTRL_GPIO_MODE2_IO18,
            (mode << SYS_CTRL_GPIO_MODE2_IO18_POS) & SYS_CTRL_GPIO_MODE2_IO18_MASK);
}

/**
 * @brief  configure the pin19  function
 * @param[in]  Pin19Mode_Type mode: the function of pin19.
 *                              @ref Pin19Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin19_usage(Pin19Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE2, SYS_CTRL_GPIO_MODE2_IO19,
            (mode << SYS_CTRL_GPIO_MODE2_IO19_POS) & SYS_CTRL_GPIO_MODE2_IO19_MASK);
}

/**
 * @brief  configure the pin20  function
 * @param[in]  Pin20Mode_Type mode: the function of pin20.
 *                              @ref Pin20Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin20_usage(Pin20Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE2, SYS_CTRL_GPIO_MODE2_IO20,
            (mode << SYS_CTRL_GPIO_MODE2_IO20_POS) & SYS_CTRL_GPIO_MODE2_IO20_MASK);
}

/**
 * @brief  configure the pin21  function
 * @param[in]  Pin21Mode_Type mode: the function of pin21.
 *                              @ref Pin21Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin21_usage(Pin21Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE2, SYS_CTRL_GPIO_MODE2_IO21,
            (mode << SYS_CTRL_GPIO_MODE2_IO21_POS) & SYS_CTRL_GPIO_MODE2_IO21_MASK);
}

/**
 * @brief  configure the pin22  function
 * @param[in]  Pin22Mode_Type mode: the function of pin22.
 *                              @ref Pin22Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin22_usage(Pin22Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE2, SYS_CTRL_GPIO_MODE2_IO22,
            (mode << SYS_CTRL_GPIO_MODE2_IO22_POS) & SYS_CTRL_GPIO_MODE2_IO22_MASK);
}

/**
 * @brief  configure the pin23  function
 * @param[in]  Pin23Mode_Type mode: the function of pin23.
 *                              @ref Pin23Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin23_usage(Pin23Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE2, SYS_CTRL_GPIO_MODE2_IO23,
            (mode << SYS_CTRL_GPIO_MODE2_IO23_POS) & SYS_CTRL_GPIO_MODE2_IO23_MASK);
}

/**
 * @brief  configure the pin24  function
 * @param[in]  Pin24Mode_Type mode: the function of pin24.
 *                              @ref Pin24Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin24_usage(Pin24Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE3, SYS_CTRL_GPIO_MODE3_IO24,
            (mode << SYS_CTRL_GPIO_MODE3_IO24_POS) & SYS_CTRL_GPIO_MODE3_IO24_MASK);
}

/**
 * @brief  configure the pin25  function
 * @param[in]  Pin25Mode_Type mode: the function of pin25.
 *                              @ref Pin25Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin25_usage(Pin25Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE3, SYS_CTRL_GPIO_MODE3_IO25,
            (mode << SYS_CTRL_GPIO_MODE3_IO25_POS) & SYS_CTRL_GPIO_MODE3_IO25_MASK);
}

/**
 * @brief  configure the pin26  function
 * @param[in]  Pin26Mode_Type mode: the function of pin26.
 *                              @ref Pin26Mode_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin26_usage(Pin26Mode_Type mode)
{
    MODIFY_REG(SYS_CTRL->GPIO_MODE3, SYS_CTRL_GPIO_MODE3_IO26,
            (mode << SYS_CTRL_GPIO_MODE3_IO26_POS) & SYS_CTRL_GPIO_MODE3_IO26_MASK);
}

/**
 * @brief  configure pin pull down mode
 * @param[in]  PadNum_Type pin:  @ref PadNum_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin_pull_down(PadNum_Type pin)
{
    SET_BIT(SYS_CTRL->PAD_PE, 1<<pin);
    CLEAR_BIT(SYS_CTRL->PAD_PS, 1<<pin);
}

/**
 * @brief  configure pin pull up mode
 * @param[in]  PadNum_Type pin:  @ref PadNum_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin_pull_up(PadNum_Type pin)
{
    SET_BIT(SYS_CTRL->PAD_PE, 1<<pin);
    SET_BIT(SYS_CTRL->PAD_PS, 1<<pin);
}

/**
 * @brief  configure pin pull none mode
 * @param[in]  PadNum_Type pin:  @ref PadNum_Type
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin_pull_none(PadNum_Type pin)
{
    CLEAR_BIT(SYS_CTRL->PAD_PE, 1<<pin);
    CLEAR_BIT(SYS_CTRL->PAD_PS, 1<<pin);
}

/**
 * @brief  configure pin jtag keep enable
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin_jtag_keep_enable()
{
    ms_pinmux_ll_set_pin24_usage(PIN25_SWC);
    ms_pinmux_ll_set_pin24_usage(PIN26_SWD);

    MODIFY_REG(SYS_CTRL->PAD_PE, (0x03<<27),(0x03<<27));
}

/**
 * @brief  configure pin jtag keep disable
 * @retval None
 */
static inline void ms_pinmux_ll_set_pin_jtag_keep_diable()
{
    MODIFY_REG(SYS_CTRL->PAD_PE, (0x03<<27),(0x00<<27));
}


#ifdef __cplusplus
}
#endif


/** @}*/

/** @}*/




#endif /* ms_pinmux_ll_LL_H_ */
