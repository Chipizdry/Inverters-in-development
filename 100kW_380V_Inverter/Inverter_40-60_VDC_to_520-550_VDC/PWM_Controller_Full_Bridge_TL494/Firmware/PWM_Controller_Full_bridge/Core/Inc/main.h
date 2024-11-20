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
#include "stm32g0xx_hal.h"

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
#define DIRECT_Pin GPIO_PIN_14
#define DIRECT_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_15
#define LED_1_GPIO_Port GPIOC
#define INT_UI_Pin GPIO_PIN_0
#define INT_UI_GPIO_Port GPIOA
#define INT_EXT_Pin GPIO_PIN_1
#define INT_EXT_GPIO_Port GPIOA
#define OUT_1_Pin GPIO_PIN_2
#define OUT_1_GPIO_Port GPIOA
#define OUT_2_Pin GPIO_PIN_3
#define OUT_2_GPIO_Port GPIOA
#define OUT_3_Pin GPIO_PIN_4
#define OUT_3_GPIO_Port GPIOA
#define ERR_1_Pin GPIO_PIN_5
#define ERR_1_GPIO_Port GPIOA
#define ERR_2_Pin GPIO_PIN_7
#define ERR_2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
# define LED_1_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET)
# define LED_1_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET)

# define TX_2   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET)
# define RX_2 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET)


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
