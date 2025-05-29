/*
 * spi.h
 *
 *  Created on: May 26, 2025
 *      Author: ARITRA
 */

#ifndef SPI_H
#define SPI_H

#include <stdint.h>

void spi1_init(void);
uint8_t spi1_transfer(uint8_t data);

#endif
