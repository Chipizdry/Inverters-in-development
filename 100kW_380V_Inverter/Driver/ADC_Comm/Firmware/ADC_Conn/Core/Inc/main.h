/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
# define LED_1_ON   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
# define LED_1_OFF  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)

# define LED_2_ON   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
# define LED_2_OFF  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)

# define LED_3_ON   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET)
# define LED_3_OFF  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET)

# define LED_4_ON   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
# define LED_4_OFF  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)

# define LED_5_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
# define LED_5_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)

# define RELAY_1_ON   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
# define RELAY_1_OFF  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)

# define RELAY_2_ON   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET)
# define RELAY_2_OFF  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET)

# define DELAY_1_ON   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)
# define DELAY_1_OFF  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET)

# define START_1_ON   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
# define START_1_OFF  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)

# define TX_2   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
# define RX_2   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_1_Pin GPIO_PIN_0
#define LED_1_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_1
#define LED_2_GPIO_Port GPIOA
#define LED_3_Pin GPIO_PIN_2
#define LED_3_GPIO_Port GPIOA
#define RELAY_1_Pin GPIO_PIN_3
#define RELAY_1_GPIO_Port GPIOA
#define RELAY_2_Pin GPIO_PIN_4
#define RELAY_2_GPIO_Port GPIOA
#define LED_4_Pin GPIO_PIN_6
#define LED_4_GPIO_Port GPIOA
#define V_SENS_DATA_Pin GPIO_PIN_7
#define V_SENS_DATA_GPIO_Port GPIOA
#define LED_5_Pin GPIO_PIN_0
#define LED_5_GPIO_Port GPIOB
#define I_SENS_DATA_Pin GPIO_PIN_12
#define I_SENS_DATA_GPIO_Port GPIOB
#define DELAY_Pin GPIO_PIN_11
#define DELAY_GPIO_Port GPIOA
#define START_Pin GPIO_PIN_12
#define START_GPIO_Port GPIOA
#define BTN_Pin GPIO_PIN_3
#define BTN_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
