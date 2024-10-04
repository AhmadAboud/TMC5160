/*
 * AD7794.c
 *
 *  Created on: 09.10.2023
 *      Author: ahmad.alaboud
 */

#include "AD7794.h"
#include "main.h"
#include "stm32h7xx_hal.h"
/*
uint8_t SPI2_Read_OneByte(uint8_t reg)
{
		uint8_t i;
		HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
		HAL_SPI_Receive(&hspi2, reg, 1, 100);
	        // Register und Data zusammen ?
			HAL_SPI_TransmitReceive(&hspi2, 0xFF , i, 1, 100);


		HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
		return i;

}

void SPI2_Read_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
	uint8_t i;
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi2, reg, 1, 100);
	for(i = 0; i < len; i++)
	{
		// Register und Data zusammen ?
		HAL_SPI_TransmitReceive(&hspi2, 0xFF , pbuf[i], 1, 100);
	}

	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);

}


void SPI2_Write_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
	uint8_t status , i;

	// SPI_CS_Enable();
	 HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);

	 HAL_SPI_Transmit(&hspi2, reg, 1, 100);

	 for (int i=0 ; i< len; i++)
		{
		 HAL_SPI_Transmit(&hspi2, *pbuf++, 1, 100);
		}

	 HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
}

void AD7794_Reset(void)
{
	uint8_t xtemp[3] = {0xFF,0xFF,0xFF};
	SPI2_Write_buf(0xFF, xtemp, 3);
}

uint8_t xDevice_ID;
void AD7794_Init(void)
{

	uint8_t ModeRegisterMsg[2] = {0x00,0x07};

	uint8_t ConfigRegisterMsg[2] = {0x06,0x91};

	uint8_t IoRegisterMsg[1] = {0x00};

	AD7794_Reset();

	HAL_Delay(1);

	xDevice_ID = SPI2_Read_OneByte(0x60);

	if((xDevice_ID & 0x0F)==0x0B)
	{
		HAL_Delay(1);
		SPI2_Write_buf(0x08, ModeRegisterMsg, 2);   // Mode register
		HAL_Delay(1);
		SPI2_Write_buf(0x08, ConfigRegisterMsg, 2);   // Config register
		HAL_Delay(1);
		SPI2_Write_buf(0x28, IoRegisterMsg, 1);   // IO register
		HAL_Delay(1);
	}
}


int32_t Read_AD7794_Data(void)
{
	uint8_t xStatus = 1;
	uint8_t xRDY = 0x80;
	uint8_t xtemp[3];
	int32_t adValue =0;

/*	while(1)
	{
		xStatus = SPI2_Read_OneByte(0x40);
		xRDY = xStatus60x80;
		HAL_Delay(1);
	}*/
/*
SPI2_Read_buf(0x58, xtemp,3);
adValue = ((int32_t)xtemp[0]<<16) | (((int32_t)xtemp[1]) <<8)|(((int32_t)xtemp[2]));

return adValue;

}*/
