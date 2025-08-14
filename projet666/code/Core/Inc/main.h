/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h5xx_hal.h"

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
#define PWM1_Pin GPIO_PIN_0
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_1
#define PWM2_GPIO_Port GPIOA
#define SERVO1_Pin GPIO_PIN_2
#define SERVO1_GPIO_Port GPIOA
#define SERVO2_Pin GPIO_PIN_3
#define SERVO2_GPIO_Port GPIOA
#define VBAT_VSENSE_Pin GPIO_PIN_5
#define VBAT_VSENSE_GPIO_Port GPIOA
#define ENCODER1_1_Pin GPIO_PIN_6
#define ENCODER1_1_GPIO_Port GPIOA
#define ENCODER1_2_Pin GPIO_PIN_7
#define ENCODER1_2_GPIO_Port GPIOA
#define TOF1_SCL_Pin GPIO_PIN_10
#define TOF1_SCL_GPIO_Port GPIOB
#define TOF1_SDA_Pin GPIO_PIN_12
#define TOF1_SDA_GPIO_Port GPIOB
#define RGB_SCK_Pin GPIO_PIN_13
#define RGB_SCK_GPIO_Port GPIOB
#define SERVO4_Pin GPIO_PIN_14
#define SERVO4_GPIO_Port GPIOB
#define RGB_MOSI_Pin GPIO_PIN_15
#define RGB_MOSI_GPIO_Port GPIOB
#define ENCODER2_2_Pin GPIO_PIN_6
#define ENCODER2_2_GPIO_Port GPIOC
#define ENCODER2_2C7_Pin GPIO_PIN_7
#define ENCODER2_2C7_GPIO_Port GPIOC
#define TOF2_SDA_Pin GPIO_PIN_9
#define TOF2_SDA_GPIO_Port GPIOC
#define TOF2_SCL_Pin GPIO_PIN_8
#define TOF2_SCL_GPIO_Port GPIOA
#define SERVO3_Pin GPIO_PIN_9
#define SERVO3_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
