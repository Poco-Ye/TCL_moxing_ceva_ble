/*
 * fputc.c
 *
 *  Created on: 2021年12月24日
 *      Author: bingrui.chen
 */

#include <stdio.h>
#include <uart.h>

int putchar(int ch)
{
    char tmp = ch;
    uart0_write((char*) &tmp, 1);
    return ch;
}
