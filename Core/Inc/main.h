/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_ADC_Pin GPIO_PIN_9
#define CS_ADC_GPIO_Port GPIOB
#define Weigh_Cell_On_Pin GPIO_PIN_5
#define Weigh_Cell_On_GPIO_Port GPIOD
#define OUT10_Pin GPIO_PIN_8
#define OUT10_GPIO_Port GPIOA
#define csSM40_Pin GPIO_PIN_10
#define csSM40_GPIO_Port GPIOG
#define V24ON_Pin GPIO_PIN_0
#define V24ON_GPIO_Port GPIOD
#define V24Vor_Pin GPIO_PIN_10
#define V24Vor_GPIO_Port GPIOA
#define U_Weigh_Cell_Flag_Pin GPIO_PIN_6
#define U_Weigh_Cell_Flag_GPIO_Port GPIOD
#define DirSM40_2_Pin GPIO_PIN_7
#define DirSM40_2_GPIO_Port GPIOC
#define DirSM40_Pin GPIO_PIN_6
#define DirSM40_GPIO_Port GPIOC
#define HX_SCK3_Pin GPIO_PIN_14
#define HX_SCK3_GPIO_Port GPIOD
#define HX_SCK2_Pin GPIO_PIN_13
#define HX_SCK2_GPIO_Port GPIOD
#define HX_SCK4_Pin GPIO_PIN_15
#define HX_SCK4_GPIO_Port GPIOD
#define HX_SCK1_Pin GPIO_PIN_12
#define HX_SCK1_GPIO_Port GPIOD
#define HX_DOUT4_Pin GPIO_PIN_14
#define HX_DOUT4_GPIO_Port GPIOE
#define OUT7_Pin GPIO_PIN_8
#define OUT7_GPIO_Port GPIOE
#define HX_DOUT3_Pin GPIO_PIN_13
#define HX_DOUT3_GPIO_Port GPIOE
#define csSM40_2_Pin GPIO_PIN_15
#define csSM40_2_GPIO_Port GPIOE
#define HX_DOUT1_Pin GPIO_PIN_9
#define HX_DOUT1_GPIO_Port GPIOE
#define HX_DOUT2_Pin GPIO_PIN_11
#define HX_DOUT2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
