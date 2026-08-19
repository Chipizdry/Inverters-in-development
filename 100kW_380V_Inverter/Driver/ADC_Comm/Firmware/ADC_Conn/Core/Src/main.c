/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"
#include "modbusDevice.h"
#include "modbusSlave.h"
#include "Registers_handler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RX_BUFFER_SIZE 64
#define USART_TIMEOUT 900 // Таймаут в миллисекундах
#define DFSDM_BUFFER_SIZE 128

#define FS_CODE_24       13395000.0f   // 2^23 для 24-битного знакового  13395000 ..8388608.0
#define FS_CODE_A_24     8388608.0f
#define AMC1336_RANGE    1.0f         // ±1 В для AMC1336-Q1
#define AMC1306_RANGE    0.25f        // ±50 мВ для AMC1306 (если используется для тока)
#define DIVIDER_RATIO    862.0f       // (1806+2.1)/2.1
#define SHUNT_RESISTANCE 0.004f       // Сопротивление шунта в Ом (если ток через шунт)

#define REG_VOLTAGE    1
#define REG_CURRENT    2
#define REG_POWER      3

#define REG_DFSDM_VOLT_L   4
#define REG_DFSDM_VOLT_H   5
#define REG_DFSDM_CURR_L   6
#define REG_DFSDM_CURR_H   7

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;
DMA_HandleTypeDef hdma_dfsdm1_flt0;
DMA_HandleTypeDef hdma_dfsdm1_flt1;

TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint32_t lastActivityTime = 0;
 bool usartBusy = false;
 uint8_t rxFrame[128]={0,};
 uint8_t txFrame[128]={0,};
 uint8_t SLAVE_ID=2;
 uint16_t data_reg[32]={0,};
 uint16_t rcv_data_reg[32]={0,};

 volatile int32_t dfsdm_value0 = 0;
 volatile int32_t dfsdm_value1 = 0;
 volatile int32_t dfsdm_raw0 = 0;
 volatile int32_t dfsdm_raw1 = 0;

 int32_t dfsdm_buffer0[DFSDM_BUFFER_SIZE];
 int32_t dfsdm_buffer1[DFSDM_BUFFER_SIZE];

 volatile float voltage_V = 0.0f;
 volatile float current_A = 0.0f;
 volatile float power_W   = 0.0f;

 float voltage_offset = 0.04f;
 float current_offset = 0.483f;

 float voltage_gain = 1.0f;
 float current_gain = 1.0f;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM15_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void Check_USART1_Timeout(void);
void Reset_USART1(void);
int _write(int file, char *ptr, int len);
static int32_t DFSDM_Average(const int32_t *buffer, uint32_t size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void DFSDM_ToRegisters(int32_t value, uint16_t *reg_low,uint16_t *reg_high)
{
    uint32_t u = (uint32_t)value;
    *reg_low  = (uint16_t)(u & 0xFFFF);
    *reg_high = (uint16_t)((u >> 16) & 0xFFFF);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DFSDM1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */



  // 1. Включить тактирование отладки

  // 2. Разрешить трассировку (TRCENA)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  // Настроить пин PB3 как SWO (AF0)
  GPIO_InitTypeDef gpioInit = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpioInit.Pin = GPIO_PIN_3;
  gpioInit.Mode = GPIO_MODE_AF_PP;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
  gpioInit.Alternate = 0; // AF0 для SWO
  HAL_GPIO_Init(GPIOB, &gpioInit);

  // Настроить предделитель SWO
  ITM->TCR = (ITM->TCR & ~ITM_TCR_TSPrescale_Msk) | (15 << 8);
  ITM->TCR |= ITM_TCR_ITMENA_Msk;
  ITM->TER |= 1;

  // 5. Тест
  ITM_SendChar('S');
  ITM_SendChar('W');
  ITM_SendChar('O');
  ITM_SendChar('\r');
  ITM_SendChar('\n');





  HAL_DFSDM_FilterRegularStart_DMA(
      &hdfsdm1_filter0,
      dfsdm_buffer0,
      DFSDM_BUFFER_SIZE
  );

  HAL_DFSDM_FilterRegularStart_DMA(
      &hdfsdm1_filter1,
      dfsdm_buffer1,
      DFSDM_BUFFER_SIZE
  );


  RELAY_1_ON;
  DELAY_1_ON;

  HAL_Delay(500);
  RELAY_2_ON;
  START_1_ON;
  HAL_Delay(500);
  LED_1_ON;
  LED_2_ON;
  LED_3_ON;
  LED_4_ON;
  LED_5_ON;
  TX_2;
  HAL_Delay(500);

  LED_1_OFF;
  LED_2_OFF;
  LED_3_OFF;
  LED_4_OFF;
  LED_5_OFF;
  RX_2;

  RELAY_1_OFF;
  DELAY_1_OFF;
  HAL_Delay(500);
  RELAY_2_OFF;
  START_1_OFF;
  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 256;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_filter1.Instance = DFSDM1_Filter1;
  hdfsdm1_filter1.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter1.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter1.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter1.Init.FilterParam.Oversampling = 256;
  hdfsdm1_filter1.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter1) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
  hdfsdm1_channel0.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel0.Init.OutputClock.Divider = 5;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 1;
  hdfsdm1_channel0.Init.Offset = 0;
  hdfsdm1_channel0.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 5;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_0, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter1, DFSDM_CHANNEL_1, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_1_Pin|LED_2_Pin|LED_3_Pin|RELAY_1_Pin
                          |RELAY_2_Pin|LED_4_Pin|GPIO_PIN_8|DELAY_Pin
                          |START_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_1_Pin LED_2_Pin LED_3_Pin LED_4_Pin
                           PA8 DELAY_Pin START_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin|LED_3_Pin|LED_4_Pin
                          |GPIO_PIN_8|DELAY_Pin|START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_1_Pin RELAY_2_Pin */
  GPIO_InitStruct.Pin = RELAY_1_Pin|RELAY_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_5_Pin */
  GPIO_InitStruct.Pin = LED_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Reset_USART1(void) {
    // Включить индикатор (если требуется)
    LED_4_ON;

    // Остановить передачу и прием по DMA
    if (HAL_UART_DMAStop(&huart1) != HAL_OK) {
        // Обработка ошибки
    }

    // Прерывание активных DMA транзакций
    if (hdma_usart1_rx.Instance != NULL) {
        HAL_DMA_Abort(&hdma_usart1_rx);
    }
    if (hdma_usart1_tx.Instance != NULL) {
        HAL_DMA_Abort(&hdma_usart1_tx);
    }

    // Отключить все прерывания UART
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE | UART_IT_TC | UART_IT_RXNE);

    // Сбросить флаги ошибок UART
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);

    // Сбросить периферийный модуль USART1
    __HAL_RCC_USART1_FORCE_RESET();
    HAL_Delay(1); // Задержка для завершения сброса
    __HAL_RCC_USART1_RELEASE_RESET();

    // Деинициализация UART
    if (HAL_UART_DeInit(&huart1) != HAL_OK) {
        // Обработка ошибки
    }

    // Повторная инициализация UART
    MX_USART1_UART_Init();

    // Повторная настройка DMA
    MX_DMA_Init();

    // Настройка UART для приема данных с использованием DMA
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxFrame, RX_BUFFER_SIZE) != HAL_OK) {
        // Обработка ошибки
    }
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    // Сброс параметров и таймеров
    lastActivityTime = HAL_GetTick();
    LED_4_OFF;

}



void Check_USART1_Timeout(void)
 {
     if (HAL_GetTick() - lastActivityTime >= USART_TIMEOUT)
     {
         Reset_USART1();
         RX_2;
     }
 }

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)

{
	    RX_2;
	    lastActivityTime = HAL_GetTick();
	    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxFrame,RX_BUFFER_SIZE);
	    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
       LED_4_OFF;
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1)
	{

		lastActivityTime = HAL_GetTick();
	    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
	    HAL_DMA_Abort(&hdma_usart1_rx);
	    Registers_handler(rxFrame, data_reg, rcv_data_reg,Size);

	}
}

void HAL_DFSDM_FilterRegConvCpltCallback(
    DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    if (hdfsdm_filter == &hdfsdm1_filter0) {
        dfsdm_value0 = DFSDM_Average(dfsdm_buffer0, DFSDM_BUFFER_SIZE);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter1) {
        dfsdm_value1 = DFSDM_Average(dfsdm_buffer1, DFSDM_BUFFER_SIZE);
    }
}


static int32_t DFSDM_Average(const int32_t *buffer, uint32_t size)
{
    int64_t sum = 0;

    for (uint32_t i = 0; i < size; ++i) {
        // signed 24-bit data расположены в [31:8]
        sum += buffer[i] >> 8;
      //  sum += buffer[i] ;
    }

    return (int32_t)(sum / (int64_t)size);
}




int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar((uint32_t)ptr[i]);
    }

    return len;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	 for(;;)
	    {
	        // 1. Захватываем текущие значения (атомарно, без прерываний)
	        int32_t adc_voltage = dfsdm_value0;
	        int32_t adc_current = dfsdm_value1;

	        // 2. Преобразование напряжения
	        float volt_adc = ((float)adc_voltage / FS_CODE_24) * AMC1336_RANGE;
	        voltage_V = (volt_adc* voltage_gain * DIVIDER_RATIO) - voltage_offset;

	        // 3. Преобразование тока
	        float curr_adc = ((float)adc_current / FS_CODE_24) * AMC1306_RANGE;
	        current_A = (curr_adc * current_gain / SHUNT_RESISTANCE )- current_offset;

	        // 4. Мощность
	        power_W = voltage_V * current_A;

	        // 5. Заполняем регистры (все из одних и тех же данных)
	       // data_reg[REG_VOLTAGE] = 0x1234;
	       // data_reg[REG_CURRENT] = 0x5678;
	       // data_reg[REG_POWER]   = 0x9ABC;
	        data_reg[REG_VOLTAGE] = (uint16_t)(voltage_V * 10.0f);
	        data_reg[REG_CURRENT] = (uint16_t)(current_A * 100.0f);
	        data_reg[REG_POWER]   = (uint16_t)(power_W);

	        DFSDM_ToRegisters(adc_voltage, &data_reg[REG_DFSDM_VOLT_L], &data_reg[REG_DFSDM_VOLT_H]);
	        DFSDM_ToRegisters(adc_current, &data_reg[REG_DFSDM_CURR_L], &data_reg[REG_DFSDM_CURR_H]);

	        printf("RAW0=%ld VALUE0=%ld RAW1=%ld VALUE1=%ld\r\n",dfsdm_raw0,dfsdm_value0,dfsdm_raw1, dfsdm_value1);

	        printf("Volts=%.3f Amps=%.3f\r\n", voltage_V, current_A);
	        Check_USART1_Timeout();
	        osDelay(500);
	    }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  LED_4_ON;
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
