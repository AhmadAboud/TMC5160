/*
 * DriveControllTMC.h
 *
 *  Created on: 05.09.2023
 *      Author: ahmad.alaboud
 */

#ifndef DRIVECONTROLLTMC_HPP_
#define DRIVECONTROLLTMC_HPP_
#include "stm32h7xx_hal.h"
#include "Pinout.hpp"
#include "main.h"


class DriveControllTMC
{
// Hier alles deklarieren Methoden und Variablen und in Drive CPP Code schreieben!

public:
	DriveControllTMC();
	~DriveControllTMC();

	Pinout* mPinout;


private:
	TIM_HandleTypeDef htim17;
	I2C_HandleTypeDef hi2c1;
	SPI_HandleTypeDef hspi1;
	SPI_HandleTypeDef hspi2;

};


#endif /* INC_DRIVECONTROLLTMC_HPP_ */
