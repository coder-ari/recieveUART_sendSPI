/*
 * spi.c
 *
 *  Created on: May 26, 2025
 *      Author: ARITRA
 */

#include "stm32f401.h"
#include "spi.h"

void spi1_init(void) {
    RCC_AHB1ENR |= (1 << 0);     // GPIOA
    RCC_APB2ENR |= (1 << 12);    // SPI1

    // PA5 = SCK, PA6 = MISO, PA7 = MOSI
    // Clear PA5, PA6, PA7 (each pin is 2 bits in MODER)
    GPIOA_MODER &= ~((3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2)));
    // Set to Alternate Function mode (10)
    GPIOA_MODER |=  ((2 << (5 * 2)) | (2 << (6 * 2)) | (2 << (7 * 2)));


    GPIOA_AFRL &= ~((0xF << (4 * 5)) | (0xF << (4 * 6)) | (0xF << (4 * 7)));
    GPIOA_AFRL |=  ((5 << (4 * 5)) | (5 << (4 * 6)) | (5 << (4 * 7)));  // AF5

    SPI1_CR1 = 0;                     // Reset
    SPI1_CR1 |= (1 << 2);             // Master
    SPI1_CR1 &= ~((1 << 1) | (1 << 0));  // ✅ Mode 0
    SPI1_CR1 |= (3 << 3);             // Baud rate = fPCLK/16
    SPI1_CR1 |= (1 << 9) | (1 << 8);  // SSM=1, SSI=1 (manual NSS)
    SPI1_CR1 |= (1 << 6);             // SPI enable
}

uint8_t spi1_transfer(uint8_t data) {
    while (!(SPI1_SR & (1 << 1)));  // Wait until TXE
    SPI1_DR = data;
    while (!(SPI1_SR & (1 << 0)));  // Wait until RXNE
    uint8_t received = SPI1_DR;
    while (SPI1_SR & (1 << 7));     // Wait until BSY cleared
    return received;
}

