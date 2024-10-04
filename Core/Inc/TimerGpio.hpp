/*
 * TimerGpio.hpp
 *
 *  Created on: 18.09.2023
 *      Author: ahmad.alaboud
 */

#ifndef INC_TIMERGPIO_HPP_
#define INC_TIMERGPIO_HPP_
#include "stm32h7xx_hal.h"

class TimerGpio
{
public:
    TimerGpio(TIM_HandleTypeDef timer);
    void Start_Ch1(int i);
    void Start_Ch2(int i);
    void Stop();
    double calcFrequency(double m_dCycle, double m_dRampTimeMS,double m_dSollSpeedDriveHZ );


private:
    uint16_t TimerPin;
    double m_dSollSpeedDriveHZ= 700;
    double m_dCycle=1;
    double m_dRampTimeMS =100;
    double m_dCurrentlySpeedDriveHz=0;
    int m_bStart = 0, m_bReverseRun = 0;
    TIM_HandleTypeDef mTimer;
};






#endif /* INC_TIMERGPIO_HPP_ */
