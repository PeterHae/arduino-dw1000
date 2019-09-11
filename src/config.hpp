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

// the standard expansion hearder locations are used here. Can be changed though to meet you pin configuration
#ifdef _SILICON_LABS_32B_SERIES_0
    /// the SPI module to use
    #define SPI_MODULE USART0

    /// SPI location defines (USART1 is used)
    #define SPI_PORT gpioPortE
    #define SPI_MOSI_PIN 10
    #define SPI_MISO_PIN 11
    #define SPI_SCLK_PIN 12

#else
    /// the SPI module to use
    #define SPI_MODULE USART1

    /// SPI location defines (USART1 is used)
    #define SPI_PORT gpioPortB
    #define SPI_MOSI_PIN 13
    #define SPI_MISO_PIN 12
    #define SPI_SCLK_PIN 11
    #define SPI_CS_PIN 9

    //#define USART_PIN_LOCATION 11
	#define SPI_MISO_LOCATION 6
	#define SPI_MOSI_LOCATION 8
	#define SPI_CLK_LOCATION 4
#endif

/// reset port and pin
#define RESET_PORT gpioPortA
#define RESET_PIN 2

/// Interrupt port and pin
#define INTERRUPT_PORT gpioPortA
#define INTERRUPT_PIN 3
#define DW_INTERRUPT_NUM 0

/// spi low baudrate 2MHz
#define SPI_LOW_BAUDRATE 2000000ULL

/// spi high baudrate 16MHz
#define SPI_HIGH_BAUDRATE 16000000ULL
