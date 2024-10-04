/*
 * Pinout.cpp
 *
 *  Created on: 07.09.2023
 *      Author: ahmad.alaboud
 */
#include "Pinout.hpp"



Pinout::Pinout()
{
    // Definiere alle GPIO pins /  von main.h

    Out10 = new GpioPin(OUT10_Pin, GPIOA);

    Out7 = new GpioPin(OUT10_Pin,GPIOE);

    V24ON = new GpioPin(V24ON_Pin, V24ON_GPIO_Port);

    DirSM40 = new GpioPin(DirSM40_Pin, GPIOC);

    V24Vor = new GpioPin(V24Vor_Pin, V24Vor_GPIO_Port);

    csSM40 = new GpioPin(csSM40_Pin, csSM40_GPIO_Port);

    extern TIM_HandleTypeDef htim17;
    Timer17 = new TimerGpio (htim17);

    extern SPI_HandleTypeDef hspi1;
    spi1 = new SPIGpio (hspi1);

    extern I2C_HandleTypeDef hi2c1;
    i2c1 = new inputsCat9555 (hi2c1);

    extern TIM_HandleTypeDef htim24;
    Timer24 = new TimerGpio (htim24);

    extern TIM_HandleTypeDef htim16;
     Timer16 = new TimerGpio (htim16);

   /*  sckPin = new GpioPin ( HX_SCK2_Pin, GPIOD);
     doutPin = new GpioPin ( HX_DOUT2_Pin, HX_DOUT2_GPIO_Port );*/

}


   // TestV24ON(V24ON_Pin, V24ON_GPIO_Port){}

