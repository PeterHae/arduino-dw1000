/**
 * @file arduinoToEfmPort.cpp
 * @author Peter Härlen (peter@haerlen.net)
 * @brief implementation for all needed functions to replace arduino stuff
 * @date 2019-02-04
 *
 * @copyright Copyright (c) 2019 Peter Härlen
 *
 */
#include "arduinoToEfmPort.hpp"
#include <stdlib.h>
//#include "thunderboard/util.h"
#include "udelay.h"

static voidFunctionPtr interruptFunctions[MAX_ATTACHED_INTERRUPTS];

Spi SPI = Spi(SPI_MODULE, Pin(SPI_PORT, SPI_SCLK_PIN), Pin(SPI_PORT, SPI_MISO_PIN), Pin(SPI_PORT, SPI_MOSI_PIN));

void efmPortInit(){
    UDELAY_Calibrate();

    // setup the wallclock timer for the millis function
    RTCDRV_Init();

    // enable the gpio Interrupts
    NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
    NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);

    //SPI = Spi(SPI_MODULE, Pin(SPI_PORT, SPI_SCLK_PIN), Pin(SPI_PORT, SPI_MISO_PIN), Pin(SPI_PORT, SPI_MOSI_PIN));

    // seed the number generator
    // TODO srand((unsigned int)RTCDRV_GetWallClockTicks32());
}

uint32_t millis(){
    uint64_t wallclockticks = RTCDRV_GetWallClockTicks64();
    // TODO what about overflows?? use 64 bit?? that never overflows
    return RTCDRV_TicksToMsec(wallclockticks);
}

void delay(uint32_t ms){
    RTCDRV_Delay(ms);
}

void delayMicroseconds(uint32_t us){
    UDELAY_Delay(us);
}

void pinMode(Pin pin, PinMode mode){
    switch (mode)
    {
        case OUTPUT:
            GPIO_PinModeSet(pin.port, pin.pinNumber, gpioModePushPull, false);
            break;
        case INPUT_PULL_DOWN:
            GPIO_PinModeSet(pin.port, pin.pinNumber, gpioModeInputPull, false);
            break;
        case INPUT_PULL_UP:
            GPIO_PinModeSet(pin.port, pin.pinNumber, gpioModeInputPull, true);
            break;
        default:
            // INPUT as default init
            GPIO_PinModeSet(pin.port, pin.pinNumber, gpioModeInput, false);
            break;
    }
}

void digitalWrite(Pin pin, bool val){
    if(val == true){
        // set pin
        GPIO_PinOutSet(pin.port, pin.pinNumber);
    }else if(val == false){
        // reset pin
        GPIO_PinOutClear(pin.port, pin.pinNumber);
    }
}

bool digitalRead(Pin pin){
    return (bool)GPIO_PinInGet(pin.port, pin.pinNumber);
}

void attachInterrupt(Pin pin, void (*intFunc)(void), int mode){
    bool fallingFlag, risingFlag;
    switch (mode)
    {
        case FALLING:
            fallingFlag = true;
            risingFlag = false;
            break;
        case CHANGE:
            fallingFlag = true;
            risingFlag = true;
            break;
        default: // RISING
            fallingFlag = false;
            risingFlag = true;
            break;
    }
    interruptFunctions[pin.pinNumber] = intFunc;
    GPIO_IntConfig(pin.port, pin.pinNumber, risingFlag, fallingFlag, true);
}


void GPIO_EVEN_IRQHandler(void){
    uint32_t iFlags = GPIO_IntGetEnabled();
    for(int i = 0; i < MAX_ATTACHED_INTERRUPTS; i += 2) {
        if((iFlags >> i) & 1)  {
            // interrupt at location i has been triggered exectue function
            GPIO_IntClear(1 << i);
            interruptFunctions[i]();
        }
    }
}

void GPIO_ODD_IRQHandler(void){
    uint32_t iFlags = GPIO_IntGetEnabled();

    for(int i = 1; i < MAX_ATTACHED_INTERRUPTS; i += 2) {
        if((iFlags >> i) & 1)  {
            // interrupt at location i has been triggered exectue function, clean flag
            GPIO_IntClear(1 << i);
            interruptFunctions[i]();
        }
    }
}


void Spi::endTransaction(){
    // TODO implement?
    // not necessary for correct library function
    ;
}

uint8_t Spi::transfer(uint8_t val){
    return USART_SpiTransfer(usartUsed, val);
}

void Spi::beginTransaction(const SPISettings& spiSettings){
    // only speeed funtion implemendet yet...
    USART_BaudrateSyncSet(usartUsed, 0, spiSettings.speed);
}

Spi::Spi(USART_TypeDef* usartToUse, Pin clk, Pin miso, Pin mosi){
    this->clk = clk;
    this->mosi = mosi;
    this->miso = miso;
    this->usartUsed = usartToUse;
    initSpi();
}

void Spi::begin(){;}

void Spi::end(){;}


SPISettings::SPISettings(uint32_t speed, uint8_t order, uint8_t mode){
    this->speed = speed;
    this->order = order;
    this->mode = mode;
}

void Spi::initSpi(){
    USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

    // Enable module clocks
	CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(cmuClock_HFPER, true);

    if(usartUsed == USART0) {
	    CMU_ClockEnable(cmuClock_USART0, true);
    } else if(usartUsed == USART1) {
	    CMU_ClockEnable(cmuClock_USART1, true);
    }

	// spi pin io set
	GPIO_PinModeSet(miso.port, miso.pinNumber, gpioModeInput, false);
	GPIO_PinModeSet(mosi.port, mosi.pinNumber, gpioModePushPull, false);
	GPIO_PinModeSet(clk.port, clk.pinNumber, gpioModePushPull, false);

    USART_Reset(usartUsed);

	init.baudrate = SPI_LOW_BAUDRATE;
	init.databits = usartDatabits8;
	init.msbf = true;
	init.master = true;
	init.clockMode = usartClockMode0;

	USART_InitSync(usartUsed, &init);

    // TODO check if that works?
    // Module USART1 is configured to location 1
    usartUsed->ROUTEPEN = (usartUsed->ROUTEPEN & ~ _USART_ROUTELOC0_MASK) | USART_ROUTELOC0_TXLOC_LOC1;

    // Enable signals TX, RX, CLK
    usartUsed->ROUTEPEN |= USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_CLKPEN;

    usartUsed->CTRL |= USART_CTRL_CLKPOL;
    usartUsed->CTRL |= USART_CTRL_CLKPHA;
}

void bitSet(uint8_t& var, uint8_t pos){
    var |= (1 << pos);
}


void bitClear(uint8_t& var, uint8_t pos){
    var &= ~((uint8_t)(1 << pos));
}

bool bitRead(uint8_t var, uint8_t pos){
    return var & (1 << pos);
}

uint32_t random(uint32_t min, uint32_t max){
	if(min == max){
		return -1;
	}
    int32_t range = max - min;
    return (rand() % range + 1) + min;
}

void randomSeed(unsigned int seed){
    srand(seed);
}
