/*
 * Pinout.hpp
 *
 *  Created on: 07.09.2023
 *      Author: ahmad.alaboud
 */

#ifndef PINOUT_HPP_
#define PINOUT_HPP_
#include "GpioPin.hpp"
#include "TimerGpio.hpp"
#include "SPIGpio.hpp"
#include "main.h"
#include "HX711_ADC.hpp"
#include "inputsCat9555.hpp"



class Pinout
{
public:
    Pinout();

    // Define all wanted GPIO pins for specific project below
    GpioPin* DirSM40;

    GpioPin* csSM40;

    GpioPin* V24Vor;

    GpioPin* Out7;

    GpioPin* Out10;

    GpioPin* V24ON;

    TimerGpio* Timer17;

    SPIGpio* spi1;

    TimerGpio* Timer24;

    TimerGpio* Timer16;

    GpioPin* sckPin;

    GpioPin* doutPin;

    inputsCat9555* i2c1;




};




#endif /* INC_PINOUT_HPP_ */
