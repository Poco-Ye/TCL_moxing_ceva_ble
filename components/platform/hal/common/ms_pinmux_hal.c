/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_pinmux_hal.c
 * @brief
 * @author bingrui.chen
 * @date 2021-12-24
 * @version 1.0
 * @Revision
 */
#include "ms_pinmux_hal.h"
#include <stddef.h>



const static PinmuxHanle_Type pin_mux_handle_list[]=
{
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin00_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin01_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin02_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin03_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin04_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin05_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin06_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin07_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin08_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin09_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin10_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin11_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin12_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin13_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin14_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin15_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin16_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin17_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin18_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin19_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin20_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin21_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin22_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin23_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin24_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin25_usage,
     (PinmuxHanle_Type)ms_pinmux_ll_set_pin26_usage
};

PinmuxHanle_Type ms_pinmux_hal_get_pinmux_handle(uint32_t pin)
{
    if(pin >= (sizeof(pin_mux_handle_list)/sizeof(pin_mux_handle_list[0])))
    {
        return NULL;
    }

    return pin_mux_handle_list[pin];
}

int32_t ms_pinmux_hal_set_pinmux(uint32_t pin,uint32_t mode)
{
    PinmuxHanle_Type pinmux_handle = ms_pinmux_hal_get_pinmux_handle(pin);
    if(pinmux_handle == NULL)
    {
        return STATUS_ERROR;
    }

    pinmux_handle(mode);

    return STATUS_SUCCESS;
}

int32_t ms_pinmux_hal_config_pull_type(uint32_t pin, uint32_t pull_type)
{
    switch (pull_type)
    {
    case PULL_UP:
        ms_pinmux_hal_set_pin_pull_up(pin);
        break;
    case PULL_DOWN:
        ms_pinmux_hal_set_pin_pull_down(pin);
        break;
    case PULL_NONE:
        ms_pinmux_hal_set_pin_pull_none(pin);
        break;
    default:
        break;
    }

    return STATUS_SUCCESS;
}

void ms_pinmux_hal_config_default()
{
    WRITE_REG(SYS_CTRL->GPIO_MODE0, 0);
    WRITE_REG(SYS_CTRL->GPIO_MODE1, 0);
    WRITE_REG(SYS_CTRL->GPIO_MODE2, 0);
    WRITE_REG(SYS_CTRL->GPIO_MODE3, 0);

    ms_pinmux_hal_set_pinmux(PAD14,PIN14_UART0_TXD);
    ms_pinmux_hal_set_pinmux(PAD15,PIN15_UART0_RXD);
}

void ms_pinmux_hal_sleep_store(uint32_t *store_buf)
{
    if (NULL == store_buf)
    {
        return;
    }

    store_buf[0] = READ_REG(SYS_CTRL->GPIO_MODE0);
    store_buf[1] = READ_REG(SYS_CTRL->GPIO_MODE1);
    store_buf[2] = READ_REG(SYS_CTRL->GPIO_MODE2);
    store_buf[3] = READ_REG(SYS_CTRL->GPIO_MODE3);
}

void ms_pinmux_hal_sleep_restore(uint32_t *store_buf)
{
    if (NULL == store_buf)
    {
        return;
    }
    WRITE_REG(SYS_CTRL->GPIO_MODE0, store_buf[0]);
    WRITE_REG(SYS_CTRL->GPIO_MODE1, store_buf[1]);
    WRITE_REG(SYS_CTRL->GPIO_MODE2, store_buf[2]);
    WRITE_REG(SYS_CTRL->GPIO_MODE3, store_buf[3]);
}

