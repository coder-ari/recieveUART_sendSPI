/*
 * uart.c
 *
 *  Created on: May 26, 2025
 *      Author: ARITRA
 */

#include "stm32f401.h"
#include "uart.h"

#define UART_BUFFER_SIZE 128

// TX Ring Buffer
static volatile char uart_tx_buffer[UART_BUFFER_SIZE];
static volatile uint16_t uart_tx_head = 0;
static volatile uint16_t uart_tx_tail = 0;

// Last received byte (simple read model)
volatile char uart_last_byte = 0;
volatile uint8_t uart_data_available = 0;

void uart_init(void) {
    // Enable GPIOA clock
    RCC_AHB1ENR |= (1 << 0);

    // PA9 (TX), PA10 (RX) to AF7 (USART1)
    GPIOA_MODER &= ~((0x3 << (9 * 2)) | (0x3 << (10 * 2)));
    GPIOA_MODER |=  ((0x2 << (9 * 2)) | (0x2 << (10 * 2)));

    GPIOA_AFRH &= ~((0xF << ((9 - 8) * 4)) | (0xF << ((10 - 8) * 4)));
    GPIOA_AFRH |=  ((0x7 << ((9 - 8) * 4)) | (0x7 << ((10 - 8) * 4)));

    // Enable USART1 clock
    RCC_APB2ENR |= (1 << 4);

    // Baud rate = 57600, assuming 16 MHz PCLK2
    USART1_BRR = (17 << 4) | 6;

    // Enable USART: UE, TE, RE, RXNEIE
    USART1_CR1 = (1 << 13) | (1 << 3) | (1 << 2) | (1 << 5);

    // Enable NVIC for USART1
    NVIC_ISER1 |= (1 << (USART1_IRQ_NUMBER - 32));
}

void uart_send_async(char c) {
    uint16_t next_head = (uart_tx_head + 1) % UART_BUFFER_SIZE;

    if (next_head == uart_tx_tail) {
        // TX buffer full, drop or handle error
        return;
    }

    uart_tx_buffer[uart_tx_head] = c;
    uart_tx_head = next_head;

    // Enable TXE interrupt
    USART1_CR1 |= (1 << 7);
}

void uart_send_hex(uint32_t value) {
    uart_send_async((value >> 24) & 0xFF);
    uart_send_async((value >> 16) & 0xFF);
    uart_send_async((value >> 8) & 0xFF);
    uart_send_async(value & 0xFF);
}

void uart_send_hex8(uint8_t value) {
    const char hex_chars[] = "0123456789ABCDEF";
    uart_send_async(hex_chars[(value >> 4) & 0x0F]);  // High nibble
    uart_send_async(hex_chars[value & 0x0F]);         // Low nibble
}

uint8_t uart_read_byte(char *c) {
    if (uart_data_available) {
        *c = uart_last_byte;
        uart_data_available = 0;
        return 1;
    }
    return 0;
}
void uart_send_async_string(const char *str) {
    while (*str) {
        uart_send_async(*str++);
    }
}
void USART1_IRQHandler(void) {
    // RXNE
    if (USART1_SR & (1 << 5)) {
        uart_last_byte = USART1_DR;
        uart_data_available = 1;
    }

    // TXE
    if ((USART1_SR & (1 << 7)) && (USART1_CR1 & (1 << 7))) {
        if (uart_tx_tail != uart_tx_head) {
            USART1_DR = uart_tx_buffer[uart_tx_tail];
            uart_tx_tail = (uart_tx_tail + 1) % UART_BUFFER_SIZE;
        } else {
            // Buffer empty, disable TXE interrupt
            USART1_CR1 &= ~(1 << 7);
        }
    }
}
