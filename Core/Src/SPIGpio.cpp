/*
 * SPIGpio.cpp
 *
 *  Created on: 18.09.2023
 *      Author: ahmad.alaboud
 */

#include "SPIGpio.hpp"
#include "Pinout.hpp"


SPIGpio :: SPIGpio (SPI_HandleTypeDef spi)
{
	mSPI = spi;
}


void SPIGpio :: Transmit()
{
	uint8_t pDataSend1[5]={0xEC, 0, 0x1, 0, 0xC3};
	uint8_t pDataRecv1[5]={0, 0, 0, 0, 0};
	HAL_GPIO_WritePin(csSM40_GPIO_Port, csSM40_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&mSPI, pDataSend1, pDataRecv1, 5, 1000);
	HAL_GPIO_WritePin(csSM40_GPIO_Port, csSM40_Pin,GPIO_PIN_SET);
}

void SPIGpio :: Transmit_Motor2()
{
	uint8_t pDataSend1[5]={0xEC, 0, 0x1, 0, 0xC3};
	uint8_t pDataRecv1[5]={0, 0, 0, 0, 0};
	HAL_GPIO_WritePin(csSM40_GPIO_Port, csSM40_2_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&mSPI, pDataSend1, pDataRecv1, 5, 1000);
	HAL_GPIO_WritePin(csSM40_GPIO_Port, csSM40_2_Pin,GPIO_PIN_SET);
}
