/*
 * DriveControllTMC.c
 *
 *  Created on: 05.09.2023
 *      Author: ahmad.alaboud
 */

#include <DriveControllTMC.hpp>
#include "main.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include "Pinout.hpp"
#include "GpioPin.hpp"
#include "inputsCat9555.hpp"
#include "EventLoop.hpp"





DriveControllTMC::DriveControllTMC()
{
	mPinout = new Pinout();

	// Test
	/*while(1)
	{
    mPinout->Out10->Toggle();
    HAL_Delay(1000);
    mPinout->Out7->Toggle();
    HAL_Delay(1000);
	}*/

	mPinout->V24Vor->Set();
	mPinout->csSM40->Set();
		HAL_Delay(200);
	mPinout->V24ON->Set();
		HAL_Delay(200);

	mPinout->DirSM40->Set();
	mPinout->Out10->Set();
	mPinout->Out10->Clear();
	mPinout->spi1->Transmit();




	/*int t =3;
	while(t)
	{
		t--;*/
   for(int i=1; i<330;i+=60){
			mPinout->Timer17->Start_Ch1(i);
			mPinout->Timer24->Start_Ch2(i);
			mPinout->spi1->Transmit();
			HAL_Delay(80);
	   }
	//}
	/*HAL_Delay(1000);
	mPinout->Timer24->Stop();*/


	/*HX711_ADC LoadCell(HX_DOUT2_Pin, GPIOE , HX_SCK2_Pin, GPIOD);
	LoadCell.begin();
    unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
    bool _tare = true; //set this to false if you don't want tare to be performed in the next step
    LoadCell.start(stabilizingtime, _tare);
    LoadCell.getTareTimeoutFlag();
    LoadCell.update();*/




}
