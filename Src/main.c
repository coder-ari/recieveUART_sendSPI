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

    uart_send_async_string("System Initialized\r\n");

    while (1) {

    	uart_try_send_from_fifo();

    }
}
