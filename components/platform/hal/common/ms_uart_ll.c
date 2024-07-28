/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_ll_uart.c
 * @brief
 * @author bingrui.chen
 * @date 2021-12-23
 * @version 1.0
 * @Revision
 */
#include <ms_clock_hal.h>
#include "ms_uart_ll.h"

int32_t ms_uart_ll_calc_baudrate(uint32_t pclk, uint32_t baudrate, uint32_t *p_int_div, uint32_t *p_fac_div)
{
    CHECK_PTR_NULL_RET(p_int_div, STATUS_ERROR);
    CHECK_PTR_NULL_RET(p_fac_div, STATUS_ERROR);
    /**
     * DIVISOR = pclk /(16 × Baud Rate)
     * DLF = mod(pclk , (16 × Baud Rate)) / Baud Rate + 0.5
     */
    uint32_t div = baudrate * 16;
    uint32_t int_div = pclk / div;
    uint32_t remainder = pclk % div;
    uint32_t fac_div = remainder / baudrate;
    uint32_t remainder_fac = remainder % baudrate;
    fac_div = (remainder_fac > (baudrate >> 1)) ? (fac_div + 1) : fac_div;

    *p_int_div = int_div;
    *p_fac_div = fac_div;

    return STATUS_SUCCESS;
}

/**
 * @brief  Set Uart Baudrate
 * @retval None
 */
int32_t ms_uart_ll_set_baudrate(Uart_Type *huart, uint32_t baudrate)
{
//    CHECK_PTR_NULL_RET(huart, STATUS_ERROR);

    uint32_t peri_clk = ms_clock_hal_get_peri_clk_freq();
    uint32_t pclk = peri_clk;
    if (huart == UART0)
    {
        pclk >>= ms_clock_hal_get_uart0_div();
    }
    else if (huart == UART1)
    {
        pclk >>= ms_clock_hal_get_uart1_div();
    }
    else if (huart == UART2)
    {
        pclk = ms_clock_hal_get_lf_clk_freq();
    }

    uint32_t int_div = 0;
    uint32_t fac_div = 0;

    if(ms_uart_ll_calc_baudrate(pclk, baudrate, &int_div, &fac_div) == STATUS_ERROR)
    {
        return STATUS_ERROR;
    }

    if (int_div == 0)
    {
        return STATUS_ERROR;
    }

    SET_BIT(huart->LCR, UART_LCR_DLAB);
    WRITE_REG(huart->OFFSET_0.DLL, int_div & 0xFF);
    WRITE_REG(huart->OFFSET_4.DLH, (int_div >> 8) & 0xFF);
    WRITE_REG(huart->DLF, fac_div & 0x1F);
    CLEAR_BIT(huart->LCR, UART_LCR_DLAB);

    return STATUS_SUCCESS;
}

