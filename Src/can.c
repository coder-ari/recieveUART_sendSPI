/*
 * can.c
 *
 *  Created on: May 26, 2025
 *      Author: ARITRA
 */

#include "stm32f401.h"
#include "spi.h"
#include "can.h"

void delay_ms(volatile uint32_t ms) {
    // Approximate cycles per ms for 16 MHz = 16,000 cycles/ms
    // Each iteration of the inner loop takes roughly 4 CPU cycles (NOP + loop overhead)
    // So number of iterations for 1 ms = 16000 / 4 = 4000

    for (volatile uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 4000; j++) {
            __asm__("nop");
        }
    }
}

void cs_low(void) {
    GPIOA_BSRR = (1 << (CS_PIN + 16));
}

void cs_high(void) {
    GPIOA_BSRR = (1 << CS_PIN);
}

void mcp2515_write_register(uint8_t reg, uint8_t val) {
    cs_low();
    spi1_transfer(MCP2515_WRITE);
    spi1_transfer(reg);
    spi1_transfer(val);
    cs_high();
}

uint8_t mcp2515_read_register(uint8_t reg) {
    uint8_t val;
    cs_low();
    spi1_transfer(MCP2515_READ);
    spi1_transfer(reg);
    val = spi1_transfer(0xFF);
    cs_high();
    return val;
}

static void mcp2515_reset(void) {
    cs_low();
    spi1_transfer(MCP2515_RESET);
    cs_high();
    delay_ms(10);
}

void can_init(void) {
    // Enable GPIOA clock
    RCC_AHB1ENR |= (1 << 0);

    // Configure CS_PIN as output
    GPIOA_MODER &= ~(0x3 << (CS_PIN * 2));
    GPIOA_MODER |=  (0x1 << (CS_PIN * 2));  // Output mode
    cs_high();

    // Reset MCP2515 chip
    mcp2515_reset();

    // Enter configuration mode
    mcp2515_write_register(MCP2515_CANCTRL, 0x80);
    delay_ms(10);

    // Set bit timing for 500 kbps (adjust for your clock)
    mcp2515_write_register(MCP2515_CNF1, 0x00);
    mcp2515_write_register(MCP2515_CNF2, 0x90);
    mcp2515_write_register(MCP2515_CNF3, 0x02);

    // Accept all messages
    mcp2515_write_register(MCP2515_RXB0CTRL, 0x00);
    mcp2515_write_register(MCP2515_RXB1CTRL, 0x00);

    // Switch to normal mode
    mcp2515_write_register(MCP2515_CANCTRL, 0x00);
    //mcp2515_write_register(MCP2515_CANCTRL, 0x40); // 0x40 = Loopback mode
    delay_ms(10);
}

bool can_send_uint32(uint32_t value) {
    // Check if TX buffer is free (TXREQ should be 0)
    uint8_t txb0ctrl = mcp2515_read_register(MCP2515_TXB0CTRL);
    if (txb0ctrl & 0x08) { // TXREQ bit
        return false; // Transmission buffer is busy
    }

    cs_low();
    spi1_transfer(MCP2515_WRITE);
    spi1_transfer(MCP2515_TXB0SIDH);
    spi1_transfer(0x00); // SIDH
    spi1_transfer(0x00); // SIDL
    spi1_transfer(0x00); // EID8
    spi1_transfer(0x00); // EID0
    spi1_transfer(0x04); // DLC = 4
    spi1_transfer((value >> 24) & 0xFF);
    spi1_transfer((value >> 16) & 0xFF);
    spi1_transfer((value >> 8) & 0xFF);
    spi1_transfer(value & 0xFF);
    cs_high();

    cs_low();
    spi1_transfer(MCP2515_RTS_TXB0);
    cs_high();

    return true;
}

bool can_receive(uint8_t *id, uint8_t *data, uint8_t *len) {
    uint8_t canintf = mcp2515_read_register(MCP2515_CANINTF);
    if (!(canintf & MCP2515_RX0IF)) {
        return false; // No message received
    }

    cs_low();
    spi1_transfer(MCP2515_READ_RXB0SIDH);

    uint8_t sidh = spi1_transfer(0xFF);
    uint8_t sidl = spi1_transfer(0xFF);
    spi1_transfer(0xFF); // EID8
    spi1_transfer(0xFF); // EID0

    *len = spi1_transfer(0xFF) & 0x0F;

    for (int i = 0; i < *len; i++) {
        data[i] = spi1_transfer(0xFF);
    }

    cs_high();

    *id = (sidh << 3) | (sidl >> 5);

    mcp2515_write_register(MCP2515_CANINTF, canintf & ~MCP2515_RX0IF);

    return true;
}
