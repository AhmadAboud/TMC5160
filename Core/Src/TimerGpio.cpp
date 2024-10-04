/*
 * TimerGpio.cpp
 *
 *  Created on: 18.09.2023
 *      Author: ahmad.alaboud
 */
#include "TimerGpio.hpp"
#include "Pinout.hpp"

extern TIM_HandleTypeDef htim24;

  TimerGpio::TimerGpio(TIM_HandleTypeDef timer)
  {
	  mTimer = timer;
  }

  double TimerGpio :: calcFrequency(double m_dCycle, double m_dRampTimeMS,double m_dSollSpeedDriveHZ )
  {
		double PI =3.14159999999478589672;
		//double m_dCurrentlySpeedDriveHz;
		m_dCurrentlySpeedDriveHz = m_dSollSpeedDriveHZ*pow(sin(m_dCycle*(1/m_dRampTimeMS)-PI/2),10);
  }

  void TimerGpio :: Start_Ch1(int i)
  {


		m_dCurrentlySpeedDriveHz = 10+round(calcFrequency(i, m_dRampTimeMS,m_dSollSpeedDriveHZ));
		__HAL_TIM_SET_AUTORELOAD (&mTimer, 60 + m_dCurrentlySpeedDriveHz); // 31khz
		__HAL_TIM_SET_PRESCALER (&mTimer,5);
	    __HAL_TIM_SET_COMPARE (&mTimer,TIM_CHANNEL_1,50);
		HAL_TIM_PWM_Start(&mTimer, TIM_CHANNEL_1);
  }

  void TimerGpio :: Start_Ch2(int i)
  {
		/*m_dCurrentlySpeedDriveHz = 10+round(calcFrequency(i, m_dRampTimeMS,m_dSollSpeedDriveHZ));
		__HAL_TIM_SET_AUTORELOAD (&mTimer, 60 + m_dCurrentlySpeedDriveHz); // 31khz
		__HAL_TIM_SET_PRESCALER (&mTimer,65537);
	    __HAL_TIM_SET_COMPARE (&mTimer,TIM_CHANNEL_2,50);
	    HAL_TIMEx_PWMN_Start(&mTimer, TIM_CHANNEL_2);*/
	    TIM24->CCR2 = 65535/2;
   		HAL_TIM_PWM_Start(&htim24, TIM_CHANNEL_2);

  }

  void TimerGpio :: Stop()
  {
	  HAL_TIMEx_PWMN_Stop(&mTimer, TIM_CHANNEL_1);

  }
