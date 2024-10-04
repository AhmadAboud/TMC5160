/*
 * inputsCat9555.cpp
 *
 *  Created on: Sep 26, 2023
 *      Author: ahmad.alaboud
 */

#include "inputsCat9555.hpp"
#include "Pinout.hpp"
#include "stm32h7xx_hal.h"

inputsCat9555 :: inputsCat9555(I2C_HandleTypeDef i2c)
{
	mI2C = i2c;
}


uint16_t inputsCat9555 :: mem_read_reg(uint8_t register_pointer)

{    HAL_StatusTypeDef status = HAL_OK;
     uint8_t return_value = 0;
    while (HAL_I2C_IsDeviceReady(&mI2C , (uint16_t)(0x40), 3, 100) != HAL_OK) {}

    HAL_I2C_Mem_Read(&mI2C ,(uint8_t)(0x40),(uint8_t)register_pointer, I2C_MEMADD_SIZE_8BIT,  &return_value, 2,   100);
    /* Check the communication status */

    if(status != HAL_OK) {
    }

    return return_value;

}
