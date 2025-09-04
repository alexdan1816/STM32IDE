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
#include "stm32f1xx_hal.h"

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
#define BUZZER_Pin GPIO_PIN_14
#define BUZZER_GPIO_Port GPIOC
#define PWMB_Pin GPIO_PIN_0
#define PWMB_GPIO_Port GPIOA
#define PWMA_Pin GPIO_PIN_1
#define PWMA_GPIO_Port GPIOA
#define LEFT_IR_RECE_Pin GPIO_PIN_2
#define LEFT_IR_RECE_GPIO_Port GPIOA
#define FLEFT_IR_RECE_Pin GPIO_PIN_3
#define FLEFT_IR_RECE_GPIO_Port GPIOA
#define FRIGHT_IR_RECE_Pin GPIO_PIN_4
#define FRIGHT_IR_RECE_GPIO_Port GPIOA
#define RIGHT_IR_RECE_Pin GPIO_PIN_5
#define RIGHT_IR_RECE_GPIO_Port GPIOA
#define RIGHT_C1_Pin GPIO_PIN_6
#define RIGHT_C1_GPIO_Port GPIOA
#define RIGHT_C2_Pin GPIO_PIN_7
#define RIGHT_C2_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_10
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_11
#define BIN2_GPIO_Port GPIOB
#define LED_RIGHT_Pin GPIO_PIN_14
#define LED_RIGHT_GPIO_Port GPIOB
#define LED_FORWARD_Pin GPIO_PIN_15
#define LED_FORWARD_GPIO_Port GPIOB
#define LED_BACK_Pin GPIO_PIN_8
#define LED_BACK_GPIO_Port GPIOA
#define LED_LEFT_Pin GPIO_PIN_9
#define LED_LEFT_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_10
#define AIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_11
#define AIN1_GPIO_Port GPIOA
#define RIGHT_IR_EMIT_Pin GPIO_PIN_15
#define RIGHT_IR_EMIT_GPIO_Port GPIOA
#define FRIGHT_IR_EMIT_Pin GPIO_PIN_3
#define FRIGHT_IR_EMIT_GPIO_Port GPIOB
#define FLEFT_IR_EMIT_Pin GPIO_PIN_4
#define FLEFT_IR_EMIT_GPIO_Port GPIOB
#define LEFT_IR_EMIT_Pin GPIO_PIN_5
#define LEFT_IR_EMIT_GPIO_Port GPIOB
#define LEFT_C1_Pin GPIO_PIN_6
#define LEFT_C1_GPIO_Port GPIOB
#define LEFT_C2_Pin GPIO_PIN_7
#define LEFT_C2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
