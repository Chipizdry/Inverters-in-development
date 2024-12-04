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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
# define LED_1_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
# define LED_1_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)

# define LED_2_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
# define LED_2_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)

# define LED_3_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
# define LED_3_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)

# define LED_4_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
# define LED_4_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)

# define LED_5_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET)
# define LED_5_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET)

# define LED_6_ON   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
# define LED_6_OFF  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)

# define LED_7_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET)
# define LED_7_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET)

# define DRV_1_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)
# define DRV_1_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET)

# define DRV_2_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
# define DRV_2_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)

# define DRV_3_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET)
# define DRV_3_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET)

# define DRV_4_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET)
# define DRV_4_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET)




# define TX_2   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET)
# define RX_2 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)
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
#define PORT_2_EN_Pin GPIO_PIN_13
#define PORT_2_EN_GPIO_Port GPIOC
#define PORT_3_EN_Pin GPIO_PIN_14
#define PORT_3_EN_GPIO_Port GPIOC
#define PORT_4_EN_Pin GPIO_PIN_15
#define PORT_4_EN_GPIO_Port GPIOC
#define DISP_INT_Pin GPIO_PIN_3
#define DISP_INT_GPIO_Port GPIOC
#define LED_6_Pin GPIO_PIN_4
#define LED_6_GPIO_Port GPIOA
#define HAL_A_Pin GPIO_PIN_5
#define HAL_A_GPIO_Port GPIOA
#define HAL_B_Pin GPIO_PIN_6
#define HAL_B_GPIO_Port GPIOA
#define HAL_C_Pin GPIO_PIN_7
#define HAL_C_GPIO_Port GPIOA
#define UI_INT_Pin GPIO_PIN_4
#define UI_INT_GPIO_Port GPIOC
#define LED_7_Pin GPIO_PIN_5
#define LED_7_GPIO_Port GPIOC
#define CS_1_Pin GPIO_PIN_0
#define CS_1_GPIO_Port GPIOB
#define CS_2_Pin GPIO_PIN_1
#define CS_2_GPIO_Port GPIOB
#define CS_3_Pin GPIO_PIN_2
#define CS_3_GPIO_Port GPIOB
#define INT_2_TIM_Pin GPIO_PIN_10
#define INT_2_TIM_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_11
#define LED_1_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_12
#define LED_2_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_13
#define LED_3_GPIO_Port GPIOB
#define LED_4_Pin GPIO_PIN_14
#define LED_4_GPIO_Port GPIOB
#define DISP_DC_RS_Pin GPIO_PIN_7
#define DISP_DC_RS_GPIO_Port GPIOC
#define DISP_LED_Pin GPIO_PIN_8
#define DISP_LED_GPIO_Port GPIOC
#define DISP_RST_Pin GPIO_PIN_9
#define DISP_RST_GPIO_Port GPIOC
#define INT_1_TIM_Pin GPIO_PIN_15
#define INT_1_TIM_GPIO_Port GPIOA
#define LED_5_Pin GPIO_PIN_11
#define LED_5_GPIO_Port GPIOC
#define PORT_1_EN_Pin GPIO_PIN_12
#define PORT_1_EN_GPIO_Port GPIOC
#define DIRECT_Pin GPIO_PIN_2
#define DIRECT_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
