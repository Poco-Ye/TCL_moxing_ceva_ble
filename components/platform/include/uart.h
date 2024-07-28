/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  uart.h
 * @brief
 * @author bingrui.chen
 * @date 2021年12月24日
 * @version 1.0
 * @Revision
 */
#ifndef UART_H_
#define UART_H_

#include "ms1008.h"

extern void uart0_int(void);
extern int32_t uart0_write_dma(char *ptr, int32_t len);
extern int32_t uart0_write(char *ptr, int32_t len);
extern int32_t uart0_read_noblock(char *ptr, int32_t len);
extern int32_t uart0_read(char *ptr, int32_t len);
extern int32_t uart0_read_dma(char *ptr, int32_t len);
extern int32_t uart0_set_baudrate(uint32_t baudrate);
extern int32_t uart0_start_rx_dma(char *ptr, int32_t len);

#endif /* UART_H_ */
