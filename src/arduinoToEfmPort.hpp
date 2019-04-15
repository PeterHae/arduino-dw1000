/**
 * @file arduinoToEfmPort.h
 * @author Peter Härlen (peter@haerlen.net)
 * @brief definitions for all needed functions to replace arduino stuff
 * @date 2019-02-04
 *
 * @copyright Copyright (c) 2019 Peter Härlen
 *
 */

#pragma once

#include "cstring"
#include "em_device.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
// the port functions need 2 timers on for the sysclock and one for the delay set up the timer functionality
/**
 * @brief see if the wallclock has been defined globally in the project
 * This is needed for the millis configuration
 * You should do so, as seen here:
 *      https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__RTCDRV.html
 *
 */
#include "rtcdrv_config.h"
#ifndef EMDRV_RTCDRV_WALLCLOCK_CONFIG
#define EMDRV_RTCDRV_WALLCLOCK_CONFIG
#endif
#include "rtcdriver.h"
#include "em_usart.h"

#include "config.hpp"
#include "Serial.hpp"


/// digital high
#define HIGH true

/// digital low
#define LOW false

/// the maximal number of Interrupts that are supported to attach
#define MAX_ATTACHED_INTERRUPTS 16

/// Arduino specific typedefs
typedef uint8_t byte;
typedef bool boolean;
typedef char* String;

/// other typedefs
typedef void (*voidFunctionPtr)(void);

/**
 * @brief EFM32 pin consits of port and pinnumber
 *
 */
struct Pin{
    /// port location
    GPIO_Port_TypeDef  port;

    /// number of pin
    unsigned int pinNumber;

    /**
     * @brief Construct a new Pin object
     *
     * @param po port to use
     * @param pi pinnumber to use
     */
    Pin(GPIO_Port_TypeDef po, unsigned int pi){
        port = po;
        pinNumber = pi;
    }
    Pin(){
    	port = (GPIO_Port_TypeDef)0xff;
    	pinNumber = 0xff;
    }
    bool operator==(const Pin& b){
    	if(b.port == this->port && b.pinNumber == this->pinNumber){
    		return true;
    	}
    	return false;
    }
};

/// all available pinmodes
enum PinMode{INPUT, OUTPUT, INPUT_PULL_UP, INPUT_PULL_DOWN};

/// interrupt trigger
enum InterruptTrigger{FALLING, RISING, CHANGE};

enum Order:uint8_t{MSBFIRST, LSBFIRST};

enum Mode:uint8_t{SPI_MODE0, SPI_MODE1};

class SPISettings{
private:
public:
    uint8_t order;
    uint8_t mode;
    uint32_t speed;
    SPISettings(uint32_t speed, uint8_t order, uint8_t mode);
};

/**
 * @brief Class dor the arduino like SPI functionality
 * 
 */
class Spi
{
private:
    /// the USART module used for spi
    USART_TypeDef* usartUsed;

    /// the clock pin
    Pin clk;

    /// the miso pin
    Pin miso;

    /// the mosi pin 
    Pin mosi;

    /// the current configured spi speed
    uint32_t speed;

    /**
     * @brief initializes the USART module for spi usage with the pins set
     * 
     */
    void initSpi();
public:
    /**
     * @brief Construct a new Spi object
     * 
     * @param usartTo Use USART module to use
     * @param clk clock pin
     * @param miso MISO pin
     * @param mosi MOSI pin
     */
    Spi(USART_TypeDef* usartToUse, Pin clk, Pin miso, Pin mosi);
    
    /**
     * @brief Construct a new Spi object, with no initialisation etc.
     * 
     */
    Spi();

    /**
     * @brief only sets the speed right now. 
     * 
     * @param spiSettings 
     */
    void beginTransaction(const SPISettings& spiSettings);

    /**
     * @brief Does an spi transfer.
     * 
     * @param val value to write
     * @return uint8_t value read from the spi bus
     */
    uint8_t transfer(uint8_t val);

    /// arduino dummy
    void endTransaction();

    /// arduino dummy
    void begin();
    
    /// arduino dummy
    void end();
};

/**
 * @brief initializes all necessary modules etc.
 * MUST BE CALLED BEFORE USAGE OF ANY FUNCTION IN THIS MODULE!
 *
 */
void arduinoToEfmPortInit();

/**
 * @brief Returns time since board start in milliseconds
 *
 * @return uint32_t time in milliseconds
 */
uint32_t millis();

/**
 * @brief pauses the program for the specified time in millisecond
 *
 * @param ms time in ms
 */
void delay(uint32_t ms);

/**
 * @brief pauses the program for the specified time in microseconds
 *
 * @param us time in microseconds
 */
void delayMicroseconds(uint32_t us);

/**
 * @brief initializes a pin for digital operations
 *
 * @param pin Pin to initialize
 * @param mode Mode to inititalize the Pin with
 */
void pinMode(Pin pin, PinMode mode);

/**
 * @brief Sets a digital value to the specified pin
 *
 * @param pin Pin to set
 * @param val Value to set
 */
void digitalWrite(Pin pin, bool val);

/**
 * @brief Returns the digital value of a Pin
 *
 * @return true Pin is low
 * @return false Pin is high
 */
bool digitalRead(Pin);

/**
 * @brief registers a function to a interrupt
 * Once a interrupt on A1 is attached ataching a interrupt to any PORTx1
 * would overwrite A1
 *
 * @param pin Pin where the interrupt is triggered
 * @param intFunc fuction to call then
 * @param mode edge direction
 */
void attachInterrupt(Pin pin, void (*intFunc)(void), int mode);

/**
 * @brief EFM32 even pinnumber interrupt function
 *
 */
void GPIO_EVEN_IRQHandler(void);

/**
 * @brief EFM32 odd pinnumber interrupt function
 *
 */
void GPIO_ODD_IRQHandler(void);

/**
 * @brief Sets (writes a 1 to) a bit of a numeric variable.
 *
 * @param var variable to manipulate
 * @param pos position to set the bit
 */
void bitSet(uint8_t& var, uint8_t pos);

/**
 * @brief Cleares (writes a 0 to) a bit of a numeric variable.
 *
 * @param var variable to manipulate
 * @param pos position to clear the bit
 */
void bitClear(uint8_t& var, uint8_t pos);

/**
 * @brief Reads a bit of a number.
 *
 * @param var variable to read from
 * @param pos position to read
 * @return true
 * @return false
 */
bool bitRead(uint8_t var, uint8_t pos);

/**
 * @brief  The random function generates pseudo-random numbers.
 *
 *
 * @param min
 * @param max
 * @return
 */
uint32_t random(uint32_t min, uint32_t max);

/**
 * @brief initializes the pseudo number generator
 *
 * @param seed value to seed
 */
void randomSeed(unsigned int seed);


/// Spi Class instance to use by the lib
extern Spi SPI;

