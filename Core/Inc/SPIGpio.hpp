/*
 * SPIGpio.hpp
 *
 *  Created on: 18.09.2023
 *      Author: ahmad.alaboud
 */

#ifndef INC_SPIGPIO_HPP_
#define INC_SPIGPIO_HPP_

#include "stm32h7xx_hal.h"

class SPIGpio
{
public:
    SPIGpio (SPI_HandleTypeDef spi);
    void Transmit();
    void Transmit_Motor2();

private:

    SPI_HandleTypeDef mSPI;
};



#endif /* INC_SPIGPIO_HPP_ */
