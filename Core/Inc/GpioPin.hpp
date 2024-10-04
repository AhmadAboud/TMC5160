/*
 * GpioPin.hpp
 *
 *  Created on: 07.09.2023
 *      Author: ahmad.alaboud
 */

#ifndef GPIOPIN_HPP_
#define GPIOPIN_HPP_
#include "stm32h7xx_hal.h"

class GpioPin
{
public:
    GpioPin(uint16_t pin, GPIO_TypeDef* port);   // Constructor
    bool Read();
    void Set();
    void Clear();
    void Toggle();

private:
    uint16_t mGpioPin;
    GPIO_TypeDef* mGpioPort;
};


#endif /* INC_GPIOPIN_HPP_ */
