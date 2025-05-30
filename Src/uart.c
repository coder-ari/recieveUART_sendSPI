/*
 * uart.c
 * Author: ARITRA
 */

#include "stm32f401.h"
#include "uart.h"
#include "can.h"

#define UART_BUFFER_SIZE 128
#define FIFO_SIZE        128
#define START_BYTE       0x1B
#define CAN_FRAME_SIZE   8

// UART TX ring buffer
static volatile char uart_tx_buffer[UART_BUFFER_SIZE];
static volatile uint16_t uart_tx_head = 0;
static volatile uint16_t uart_tx_tail = 0;

// FIFO for CAN frames (holds up to 16×8 = 128 bytes)
static volatile uint8_t fifo[FIFO_SIZE];
static volatile uint16_t fifo_head = 0;
static volatile uint16_t fifo_tail = 0;

// Shift buffer for frame detection
#define FRAME_SEARCH_BUFFER 8
static uint8_t shift_buf[FRAME_SEARCH_BUFFER] = {0};

// FIFO helpers
static uint16_t fifo_available(void) {
    return (fifo_head - fifo_tail + FIFO_SIZE) % FIFO_SIZE;
}

static uint16_t fifo_free_space(void) {
    return FIFO_SIZE - fifo_available() - 1;
}

static void fifo_push(uint8_t byte) {
    fifo[fifo_head] = byte;
    fifo_head = (fifo_head + 1) % FIFO_SIZE;
}

static void fifo_push_frame(uint8_t *frame) {
    for (int i = 0; i < CAN_FRAME_SIZE; i++) {
        fifo_push(frame[i]);
    }
}

static uint8_t fifo_pop(void) {
    uint8_t byte = fifo[fifo_tail];
    fifo_tail = (fifo_tail + 1) % FIFO_SIZE;
    return byte;
}

void uart_init(void) {
    RCC_AHB1ENR |= (1 << 0);   // GPIOA clock
    RCC_APB2ENR |= (1 << 4);   // USART1 clock

    // PA9, PA10 to AF7 (USART1)
    GPIOA_MODER &= ~((3 << (9 * 2)) | (3 << (10 * 2)));
    GPIOA_MODER |= ((2 << (9 * 2)) | (2 << (10 * 2)));
    GPIOA_AFRH &= ~((0xF << ((9 - 8) * 4)) | (0xF << ((10 - 8) * 4)));
    GPIOA_AFRH |= ((7 << ((9 - 8) * 4)) | (7 << ((10 - 8) * 4)));

    USART1_BRR = (17 << 4) | 6;  // 57600 baud @ 16MHz
    USART1_CR1 = (1 << 13) | (1 << 3) | (1 << 2) | (1 << 5);  // UE, TE, RE, RXNEIE

    NVIC_ISER1 |= (1 << (USART1_IRQ_NUMBER - 32));
}

void uart_send_async(char c) {
    uint16_t next = (uart_tx_head + 1) % UART_BUFFER_SIZE;
    if (next == uart_tx_tail) return; // Buffer full

    uart_tx_buffer[uart_tx_head] = c;
    uart_tx_head = next;

    USART1_CR1 |= (1 << 7); // Enable TXE interrupt
}

void uart_send_async_string(const char *s) {
    while (*s) uart_send_async(*s++);
}

void uart_try_send_from_fifo(void) {
    while (fifo_available() >= CAN_FRAME_SIZE) {
        uint8_t frame[CAN_FRAME_SIZE];
        for (int i = 0; i < CAN_FRAME_SIZE; i++) frame[i] = fifo_pop();

        uint8_t retries = 100;
        while (!can_send_bytes(frame, CAN_FRAME_SIZE) && retries--);

        if (retries == 0)
            uart_send_async_string("CAN Fail\r\n");
    }
}

void USART1_IRQHandler(void) {
    if (USART1_SR & (1 << 5)) { // RXNE
        uint8_t byte = USART1_DR;

        // Shift buffer left
        for (int i = 0; i < FRAME_SEARCH_BUFFER - 1; i++) {
            shift_buf[i] = shift_buf[i + 1];
        }
        shift_buf[FRAME_SEARCH_BUFFER - 1] = byte;

        // Check if we have a candidate frame
        if (shift_buf[0] == START_BYTE) {
            uint8_t sum = 0;
            for (int i = 0; i < CAN_FRAME_SIZE - 1; i++) {
                sum += shift_buf[i];
            }

            uint8_t checksum = (uint8_t)(sum & 0xFF);

            if (checksum == shift_buf[CAN_FRAME_SIZE - 1]) {
                if (fifo_free_space() >= CAN_FRAME_SIZE) {
                    fifo_push_frame(shift_buf);
                } else {
                    uart_send_async_string("FIFO Full\r\n");
                }

                // Clear shift_buf after valid frame to avoid repeat detection
                for (int i = 0; i < FRAME_SEARCH_BUFFER; i++) shift_buf[i] = 0;
            }
        }
    }

    if ((USART1_SR & (1 << 7)) && (USART1_CR1 & (1 << 7))) { // TXE
        if (uart_tx_tail != uart_tx_head) {
            USART1_DR = uart_tx_buffer[uart_tx_tail];
            uart_tx_tail = (uart_tx_tail + 1) % UART_BUFFER_SIZE;
        } else {
            USART1_CR1 &= ~(1 << 7);  // Disable TXE interrupt
        }
    }
}
