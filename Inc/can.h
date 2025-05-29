/*
 * can.h
 *
 *  Created on: May 26, 2025
 *      Author: ARITRA
 */

#ifndef CAN_H
#define CAN_H

#include <stdint.h>
#include <stdbool.h>




// MCP2515 SPI commands
#define MCP2515_RESET      0xC0
#define MCP2515_READ       0x03
#define MCP2515_WRITE      0x02
#define MCP2515_RTS_TXB0   0x81

// MCP2515 register addresses
#define MCP2515_CANSTAT    0x0E
#define MCP2515_CANCTRL    0x0F
#define MCP2515_CNF1       0x2A
#define MCP2515_CNF2       0x29
#define MCP2515_CNF3       0x28
#define MCP2515_RXB0CTRL   0x60
#define MCP2515_RXB1CTRL   0x70
#define MCP2515_TXB0SIDH   0x31
#define MCP2515_CANINTF          0x2C
#define MCP2515_RX0IF            (1 << 0)
#define MCP2515_READ_RXB0SIDH    0x90
#define MCP2515_TXB0CTRL   0x30

// Chip Select pin number for your board (e.g., PA4)
#define CS_PIN 4

void can_init(void);
void cs_low(void);
void cs_high(void);
void delay_ms(volatile uint32_t ms);
uint8_t mcp2515_read_register(uint8_t reg);
void mcp2515_write_register(uint8_t reg, uint8_t val);
bool can_send_uint32(uint32_t value);
bool can_receive(uint8_t *id, uint8_t *data, uint8_t *len);

#endif
