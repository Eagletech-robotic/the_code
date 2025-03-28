/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define BLUE_BUTTON_Pin GPIO_PIN_13
#define BLUE_BUTTON_GPIO_Port GPIOC
#define BTLP1_RX_Pin GPIO_PIN_0
#define BTLP1_RX_GPIO_Port GPIOC
#define BTLP1_TX_Pin GPIO_PIN_1
#define BTLP1_TX_GPIO_Port GPIOC
#define JACK_Pin GPIO_PIN_3
#define JACK_GPIO_Port GPIOC
#define encoder1_1_Pin GPIO_PIN_0
#define encoder1_1_GPIO_Port GPIOA
#define encoder1_2_Pin GPIO_PIN_1
#define encoder1_2_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_2
#define PWM2_GPIO_Port GPIOA
#define sens2_Pin GPIO_PIN_4
#define sens2_GPIO_Port GPIOA
#define encoder2_1_Pin GPIO_PIN_6
#define encoder2_1_GPIO_Port GPIOA
#define encoder2_2_Pin GPIO_PIN_7
#define encoder2_2_GPIO_Port GPIOA
#define INS3_TX_Pin GPIO_PIN_4
#define INS3_TX_GPIO_Port GPIOC
#define INS3_RX_Pin GPIO_PIN_5
#define INS3_RX_GPIO_Port GPIOC
#define sens1_Pin GPIO_PIN_2
#define sens1_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_15
#define PWM1_GPIO_Port GPIOA
#define PWM16_Pin GPIO_PIN_8
#define PWM16_GPIO_Port GPIOB
#define PWM17_Pin GPIO_PIN_9
#define PWM17_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
