/*
 * EventLoop.cpp
 *
 *  Created on: 05.09.2023
 *      Author: ahmad.alaboud
 */

// Main Cpp event loop to run application
#include "EventLoop.hpp"
#include "DriveControllTMC.hpp"
#include "main.h"

DriveControllTMC* startCppKlasse;

	void EventCreateCpp()
	{
		startCppKlasse = new DriveControllTMC();

        EventLoopC();
   	}

void EventLoopCpp()
{
	/*startCppKlasse->mPinout->V24ON->Toggle();
	HAL_Delay(1000);
	startCppKlasse->mPinout->TestOut10->Toggle();*/

}

// Define all C function calls from main.c below
extern "C"
{

	void EventCreateC()
		{
			EventCreateCpp();
		}

	void EventLoopC()
		{
			EventLoopCpp();
		}



}

