/*
 * AD7794.h
 *
 *  Created on: 09.10.2023
 *      Author: ahmad.alaboud
 */


#ifndef INC_AD7794_H_
#define INC_AD7794_H_
#include "main.h"

#include "stm32h7xx_hal.h"



uint8_t SPI2_Read_OneByte(uint8_t reg);
void AD7794_Init(void);
int32_t Read_AD7794_Data(void);
void toBin(int32_t xValue);

#endif /* INC_AD7794_H_ */
