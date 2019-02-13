/**
 * @file config.hpp
 * @author Peter Härlen (peter@haerlen.net)
 * @brief 
 * @date 2019-02-11
 * 
 * @copyright Copyright Peter Härlen
 * 
 */

#pragma once

/// the SPI module to use
#define SPI_MODULE USART1

/// SPI location defines (USART1 is used)
#define SPI_PORT gpioPortC
#define SPI_MOSI_PIN 6
#define SPI_MISO_PIN 7
#define SPI_SCLK_PIN 8
#define SPI_CS_PIN 9

/// reset port and pin
#define RESET_PORT gpioPortA
#define RESET_PIN 2

/// Interrupt port and pin
#define INTERRUPT_PORT gpioPortA
#define INTERRUPT_PIN 3
#define DW_INTERRUPT_NUM 0

/// spi low baudrate 4MHz
#define SPI_LOW_BAUDRATE 2000000

/// spi high baudrate 16MHz
#define SPI_HIGH_BAUDRATE 16000000