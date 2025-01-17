/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l1xx_hal.h"

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
void send_pulse();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BOARD_BTN_Pin GPIO_PIN_13
#define BOARD_BTN_GPIO_Port GPIOC
#define BOARD_BTN_EXTI_IRQn EXTI15_10_IRQn
#define SERVO_Pin GPIO_PIN_0
#define SERVO_GPIO_Port GPIOA
#define TRIGGER_Pin GPIO_PIN_4
#define TRIGGER_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOA
#define READ_SENSOR_Pin GPIO_PIN_6
#define READ_SENSOR_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

#define MODO_A 0
#define MODO_M 1
#define MODO_S 2

#define NONE_EDGE 0
#define RISING_EDGE 1
#define FALLING_EDGE 2

#define D_M90 1500
#define D_0   ((D_P90 + D_M90) / 2)
#define D_P90 6500

#define INC ((D_P90 - D_M90) / NUM_SAMPLES)
#define NUM_SAMPLES 5

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
