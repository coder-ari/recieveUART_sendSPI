/*
 * main.c
 * Created on: May 29, 2025
 * Author: ARITRA
 */

#include "stm32f401.h"
#include "uart.h"
#include "can.h"
#include "spi.h"

int main(void) {
	spi1_init();
    uart_init();
    can_init();

    uint8_t rx_index = 0;
    uint8_t rx_buffer[4];  // Buffer to accumulate 4 bytes
    char byte;

    uart_send_async_string(" System Initialized \r\n");

    while (1) {
        if (uart_read_byte(&byte)) {
            rx_buffer[rx_index++] = byte;

            if (rx_index == 4) {
                // Convert to uint32_t
                uint32_t value = (rx_buffer[0] << 24) |
                                 (rx_buffer[1] << 16) |
                                 (rx_buffer[2] << 8) |
                                 (rx_buffer[3]);

                if (can_send_uint32(value)) {
                    uart_send_async_string("Sent: ");
                    uart_send_hex(value);
                    uart_send_async('\n');
                } else {
                    uart_send_async_string("CAN Send Failed\r\n");
                }

                rx_index = 0;  // Reset buffer
            }
        }
    }
}
