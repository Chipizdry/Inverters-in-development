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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
# define LED_1_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET)
# define LED_1_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET)

# define LED_2_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET)
# define LED_2_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET)

# define LED_3_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET)
# define LED_3_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET)

# define LED_4_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)
# define LED_4_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)

# define LED_5_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)
# define LED_5_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET)

# define LED_6_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET)
# define LED_6_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET)

# define LED_7_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET)
# define LED_7_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET)

# define LED_8_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET)
# define LED_8_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET)


# define DRIVE_1_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)
# define DRIVE_1_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)

# define DRIVE_2_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)
# define DRIVE_2_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)

# define DRIVE_3_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET)
# define DRIVE_3_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET)


# define TX_2   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
# define RX_2   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
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
#define Port_1_Pin GPIO_PIN_0
#define Port_1_GPIO_Port GPIOC
#define Port_2_Pin GPIO_PIN_1
#define Port_2_GPIO_Port GPIOC
#define Port_3_Pin GPIO_PIN_2
#define Port_3_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_3
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_4
#define LED_2_GPIO_Port GPIOC
#define HALL_A_Pin GPIO_PIN_0
#define HALL_A_GPIO_Port GPIOB
#define HALL_A_EXTI_IRQn EXTI0_IRQn
#define HALL_B_Pin GPIO_PIN_1
#define HALL_B_GPIO_Port GPIOB
#define HALL_B_EXTI_IRQn EXTI1_IRQn
#define HALL_C_Pin GPIO_PIN_2
#define HALL_C_GPIO_Port GPIOB
#define HALL_C_EXTI_IRQn EXTI2_IRQn
#define CS_2_Pin GPIO_PIN_11
#define CS_2_GPIO_Port GPIOB
#define CS_3_Pin GPIO_PIN_12
#define CS_3_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_6
#define LED_3_GPIO_Port GPIOC
#define LED_4_Pin GPIO_PIN_7
#define LED_4_GPIO_Port GPIOC
#define LED_5_Pin GPIO_PIN_8
#define LED_5_GPIO_Port GPIOC
#define LED_6_Pin GPIO_PIN_9
#define LED_6_GPIO_Port GPIOC
#define LED_7_Pin GPIO_PIN_10
#define LED_7_GPIO_Port GPIOC
#define LED_8_Pin GPIO_PIN_11
#define LED_8_GPIO_Port GPIOC
#define DIRECT_Pin GPIO_PIN_8
#define DIRECT_GPIO_Port GPIOB
#define CS_1_Pin GPIO_PIN_9
#define CS_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
