/*
 * uart.h
 *
 *  Created on: May 23, 2025
 *      Author: ARITRA
 */

#ifndef UART_H
#define UART_H

#include "stdint.h"

void uart_init(void);
void uart_send_async(char c);
void uart_send_async_string(const char *str);
int uart_get_char(char *c);
void uart_send_hex(uint32_t value);
uint8_t uart_read_byte(char *c);
void uart_send_hex8(uint8_t value);
#define USART1_IRQ_NUMBER 37

#endif
