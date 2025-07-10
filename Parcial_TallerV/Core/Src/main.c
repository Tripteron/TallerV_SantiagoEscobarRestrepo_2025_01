/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "maquina_estados_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 128  // Tamaño de cada buffer (ajustar según necesidades)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t timer2FLAG = 0;
volatile uint8_t display7segmentFLAG = 0;
volatile uint8_t encoderSWextiFLAG = 0;

// Buffers Ping-Pong para X e Y (intercalados)
volatile uint16_t adc_buffer_ping[2 * BUFFER_SIZE];  // [X0, Y0, X1, Y1, ...]
volatile uint16_t adc_buffer_pong[2 * BUFFER_SIZE];  // [X0, Y0, X1, Y1, ...]

// Flags de estado
volatile uint8_t ping_active = 1;  // 1=Ping activo, 0=Pong activo
volatile uint8_t half_transfer_flag = 0;
volatile uint8_t full_transfer_flag = 0;

uint16_t x_values[BUFFER_SIZE];
uint16_t y_values[BUFFER_SIZE];
uint16_t offset = 0;

// Control de buffers
volatile enum {
    BUFFER_PING_FIRST_HALF,
    BUFFER_PING_SECOND_HALF,
    BUFFER_PONG_FIRST_HALF,
    BUFFER_PONG_SECOND_HALF
} current_buffer_section = BUFFER_PING_FIRST_HALF;

volatile uint8_t data_ready = 0;

//7 segmentos
uint8_t contadorDigito_XX = 0;
uint8_t contadorDigito_YY = 0;
volatile uint16_t display_value_XX = 0;  // Valor a mostrar en el display XX (0-100)
volatile uint16_t display_value_YY = 0;  // Valor a mostrar en el display YY (0-100)

uint8_t miles = 0;
uint8_t centenas = 0;
uint8_t decenas = 0;
uint8_t	unidades = 0;

fsm_states_t stateMachine = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
e_PosibleStates state_machine_action(uint8_t event);
void InitProgram(void);
void mostrarUnidades(void);
void mostrarDecenas(void);
void mostrarCentenas(void);
void mostrarMiles(void);
void update7SegmentDisplay(void);
void segmentoON(uint8_t number);
void divideNumber(uint16_t contador);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  InitProgram();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer_ping, 2 * BUFFER_SIZE);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if(half_transfer_flag || full_transfer_flag) {
//	          // Determinar qué buffer procesar
//	          volatile uint16_t* current_buffer;
//
//	          if(half_transfer_flag) {
//	              current_buffer = ping_active ? adc_buffer_ping : adc_buffer_pong;
//	          } else { // full_transfer_flag
//	              current_buffer = ping_active ? adc_buffer_pong : adc_buffer_ping;
//	          }
//
//	          // Procesar datos (ejemplo: separar ejes)
//
//
//	          for(int i = 0; i < BUFFER_SIZE; i++) {
//	              x_values[i] = current_buffer[2 * i];      // Posiciones pares: X
//	              y_values[i] = current_buffer[2 * i + 1];  // Posiciones impares: Y
//	          }
//
//	          // Resetear flags
//	          half_transfer_flag = 0;
//	          full_transfer_flag = 0;
//
//	          // Cambiar buffer activo si es transferencia completa
//	          if(full_transfer_flag) {
//	              ping_active = !ping_active;
//	          }
//
//	          // AQUÍ TU PROCESAMIENTO DE DATOS (ejes X e Y separados)
//	          // Puedes implementar filtrado, calibración, etc.
//	      }
	  state_machine_action(0);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 32000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 50000;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 25000;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 502-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1200;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 25;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(pinH1Led2Board_GPIO_Port, pinH1Led2Board_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, pinDigit2_Pin|pinDigit1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(pinSegmentA_GPIO_Port, pinSegmentA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, pinSegmentE_Pin|pinSegmentD_Pin|pinSegmentF_Pin|pinSegmentG_Pin
                          |pinSegmentC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, pinDigit3_Pin|pinDigit4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(pinSegmentB_GPIO_Port, pinSegmentB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : pinH1Led2Board_Pin */
  GPIO_InitStruct.Pin = pinH1Led2Board_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(pinH1Led2Board_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : pinDigit2_Pin pinDigit1_Pin pinSegmentE_Pin pinSegmentD_Pin
                           pinSegmentF_Pin pinSegmentG_Pin pinSegmentC_Pin */
  GPIO_InitStruct.Pin = pinDigit2_Pin|pinDigit1_Pin|pinSegmentE_Pin|pinSegmentD_Pin
                          |pinSegmentF_Pin|pinSegmentG_Pin|pinSegmentC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : pinEncoderDT_Pin */
  GPIO_InitStruct.Pin = pinEncoderDT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(pinEncoderDT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : pinSegmentA_Pin */
  GPIO_InitStruct.Pin = pinSegmentA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(pinSegmentA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : pinEncoderSW_Pin */
  GPIO_InitStruct.Pin = pinEncoderSW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(pinEncoderSW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : pinDigit3_Pin pinDigit4_Pin */
  GPIO_InitStruct.Pin = pinDigit3_Pin|pinDigit4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : pinSegmentB_Pin */
  GPIO_InitStruct.Pin = pinSegmentB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(pinSegmentB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : pinEncoderCLK_Pin */
  GPIO_InitStruct.Pin = pinEncoderCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(pinEncoderCLK_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Funcion para inicar la FSM en el estado base. El main.c con drivers propios no lo tiene
void InitProgram(void)
{
	stateMachine.state=IDLE;
	current_buffer_section = BUFFER_PING_FIRST_HALF;
}

void divideNumber(uint16_t contador)
{
	if(contador ==4096)
	{
		contador = 0;
	}
	miles = contador/1000;
	centenas = contador/100 %10;
	decenas = contador/10 %10;
	unidades = contador%10;
}
void update7SegmentDisplay(void)
{
	if(contadorDigito_XX == 4)
	{
		contadorDigito_XX = 0;
	}
	if(display_value_XX>=100){
		display_value_XX = 99;
	}
	if(display_value_YY>=100){
		display_value_YY = 99;
	}
	divideNumber(display_value_YY*100 + display_value_XX);
	switch(contadorDigito_XX)
	{
		case 0:
			mostrarUnidades();
		break;
		case 1:
			mostrarDecenas();
		break;
		case 2:
			mostrarCentenas();
		break;
		case 3:
			mostrarMiles();
		break;
	}
	contadorDigito_XX++;
}
//void update7SegmentDisplay_YY(void)
//{
//	if(contadorDigito_YY == 2)
//	{
//		contadorDigito_YY = 0;
//	}
//	if(display_value_YY>=100){
//		display_value_YY = 99;
//	}
//	divideNumber(display_value_YY);
//	switch(contadorDigito_YY)
//	{
//		case 0:
//			mostrarCentenas();
//		break;
//		case 1:
//			mostrarMiles();
//		break;
//	}
//	contadorDigito_YY++;
//}
void segmentoON(uint8_t number)
{
	switch(number)
	{

		case 0:
			HAL_GPIO_WritePin(pinSegmentA_GPIO_Port, pinSegmentA_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentA_GPIO_Port, pinSegmentA_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentB_GPIO_Port, pinSegmentB_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentC_GPIO_Port, pinSegmentC_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentD_GPIO_Port, pinSegmentD_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentE_GPIO_Port, pinSegmentE_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentF_GPIO_Port, pinSegmentF_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentG_GPIO_Port, pinSegmentG_Pin, RESET);
		break;

		case 1:
			HAL_GPIO_WritePin(pinSegmentA_GPIO_Port, pinSegmentA_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentB_GPIO_Port, pinSegmentB_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentC_GPIO_Port, pinSegmentC_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentD_GPIO_Port, pinSegmentD_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentE_GPIO_Port, pinSegmentE_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentF_GPIO_Port, pinSegmentF_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentG_GPIO_Port, pinSegmentG_Pin, RESET);
		break;

		case 2:
			HAL_GPIO_WritePin(pinSegmentA_GPIO_Port, pinSegmentA_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentB_GPIO_Port, pinSegmentB_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentC_GPIO_Port, pinSegmentC_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentD_GPIO_Port, pinSegmentD_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentE_GPIO_Port, pinSegmentE_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentF_GPIO_Port, pinSegmentF_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentG_GPIO_Port, pinSegmentG_Pin, SET);
		break;

		case 3:
			HAL_GPIO_WritePin(pinSegmentA_GPIO_Port, pinSegmentA_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentB_GPIO_Port, pinSegmentB_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentC_GPIO_Port, pinSegmentC_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentD_GPIO_Port, pinSegmentD_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentE_GPIO_Port, pinSegmentE_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentF_GPIO_Port, pinSegmentF_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentG_GPIO_Port, pinSegmentG_Pin, SET);
		break;

		case 4:
			HAL_GPIO_WritePin(pinSegmentA_GPIO_Port, pinSegmentA_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentB_GPIO_Port, pinSegmentB_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentC_GPIO_Port, pinSegmentC_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentD_GPIO_Port, pinSegmentD_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentE_GPIO_Port, pinSegmentE_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentF_GPIO_Port, pinSegmentF_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentG_GPIO_Port, pinSegmentG_Pin, SET);
		break;

		case 5:
			HAL_GPIO_WritePin(pinSegmentA_GPIO_Port, pinSegmentA_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentB_GPIO_Port, pinSegmentB_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentC_GPIO_Port, pinSegmentC_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentD_GPIO_Port, pinSegmentD_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentE_GPIO_Port, pinSegmentE_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentF_GPIO_Port, pinSegmentF_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentG_GPIO_Port, pinSegmentG_Pin, SET);
		break;

		case 6:
			HAL_GPIO_WritePin(pinSegmentA_GPIO_Port, pinSegmentA_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentB_GPIO_Port, pinSegmentB_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentC_GPIO_Port, pinSegmentC_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentD_GPIO_Port, pinSegmentD_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentE_GPIO_Port, pinSegmentE_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentF_GPIO_Port, pinSegmentF_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentG_GPIO_Port, pinSegmentG_Pin, SET);
		break;

		case 7:
			HAL_GPIO_WritePin(pinSegmentA_GPIO_Port, pinSegmentA_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentB_GPIO_Port, pinSegmentB_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentC_GPIO_Port, pinSegmentC_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentD_GPIO_Port, pinSegmentD_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentE_GPIO_Port, pinSegmentE_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentF_GPIO_Port, pinSegmentF_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentG_GPIO_Port, pinSegmentG_Pin, RESET);
		break;

		case 8:
			HAL_GPIO_WritePin(pinSegmentA_GPIO_Port, pinSegmentA_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentB_GPIO_Port, pinSegmentB_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentC_GPIO_Port, pinSegmentC_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentD_GPIO_Port, pinSegmentD_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentE_GPIO_Port, pinSegmentE_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentF_GPIO_Port, pinSegmentF_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentG_GPIO_Port, pinSegmentG_Pin, SET);
		break;

		case 9:
			HAL_GPIO_WritePin(pinSegmentA_GPIO_Port, pinSegmentA_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentB_GPIO_Port, pinSegmentB_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentC_GPIO_Port, pinSegmentC_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentD_GPIO_Port, pinSegmentD_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentE_GPIO_Port, pinSegmentE_Pin, RESET);
			HAL_GPIO_WritePin(pinSegmentF_GPIO_Port, pinSegmentF_Pin, SET);
			HAL_GPIO_WritePin(pinSegmentG_GPIO_Port, pinSegmentG_Pin, SET);
		break;

		default:
		{
			break;
		}
	}
}
void mostrarUnidades(void)
{
	HAL_GPIO_WritePin(pinDigit1_GPIO_Port,pinDigit1_Pin, SET);
	HAL_GPIO_WritePin(pinDigit1_GPIO_Port,pinDigit1_Pin,SET);
	HAL_GPIO_WritePin(pinDigit2_GPIO_Port,pinDigit2_Pin,SET);
	HAL_GPIO_WritePin(pinDigit3_GPIO_Port,pinDigit3_Pin,SET);
	segmentoON(unidades);
	HAL_GPIO_WritePin(pinDigit4_GPIO_Port,pinDigit4_Pin,RESET);
}

void mostrarDecenas(void)
{
	HAL_GPIO_WritePin(pinDigit1_GPIO_Port,pinDigit1_Pin,SET);
	HAL_GPIO_WritePin(pinDigit2_GPIO_Port,pinDigit2_Pin,SET);
	HAL_GPIO_WritePin(pinDigit4_GPIO_Port,pinDigit4_Pin,SET);
	segmentoON(decenas);
	HAL_GPIO_WritePin(pinDigit3_GPIO_Port,pinDigit3_Pin,RESET);
}

void mostrarCentenas(void)
{
	HAL_GPIO_WritePin(pinDigit1_GPIO_Port,pinDigit1_Pin,SET);
	HAL_GPIO_WritePin(pinDigit4_GPIO_Port,pinDigit4_Pin,SET);
	HAL_GPIO_WritePin(pinDigit3_GPIO_Port,pinDigit3_Pin,SET);
	segmentoON(centenas);
	HAL_GPIO_WritePin(pinDigit2_GPIO_Port,pinDigit2_Pin,RESET);
}

void mostrarMiles(void)
{
	HAL_GPIO_WritePin(pinDigit4_GPIO_Port,pinDigit4_Pin,SET);
	HAL_GPIO_WritePin(pinDigit2_GPIO_Port,pinDigit2_Pin,SET);
	HAL_GPIO_WritePin(pinDigit3_GPIO_Port,pinDigit3_Pin,SET);
	segmentoON(miles);
	HAL_GPIO_WritePin(pinDigit1_GPIO_Port,pinDigit1_Pin,RESET);
}
// maquina de estados
e_PosibleStates state_machine_action(uint8_t event)
{
	switch (stateMachine.state){
	case IDLE:
	{

//	    if(uart_rx_flag)
//	    {
//	    	stateMachine.state=PROCESANDO_COMANDO;
//	    	uart_rx_flag = 0;
//	    }
	   if(data_ready) {
			volatile uint16_t* current_half;


			// Determinar qué sección procesar
			switch(current_buffer_section) {
				case BUFFER_PING_FIRST_HALF:
					current_half = adc_buffer_ping;
					break;
				case BUFFER_PING_SECOND_HALF:
					current_half = adc_buffer_ping + BUFFER_SIZE;
					break;
				case BUFFER_PONG_FIRST_HALF:
					current_half = adc_buffer_pong;
					break;
				case BUFFER_PONG_SECOND_HALF:
					current_half = adc_buffer_pong + BUFFER_SIZE;
					break;
			}

			// Separar ejes (2 muestras por posición)
			for(int i = 0; i < BUFFER_SIZE/2; i++) {
				x_values[i] = (current_half[2 * i]* 100 + 2047) / 4095; ; // Redondeo al entero más cercano
				y_values[i] = (current_half[2 * i + 1]* 100 + 2047) / 4095; ; // Redondeo al entero más cercano
			}
			//Promedio para eje X
			uint32_t sumaTotal_ejeX = 0;
			for(int i = 0; i < BUFFER_SIZE/2; i++) {
				sumaTotal_ejeX += x_values[i];
			}
			display_value_XX = 2*sumaTotal_ejeX / BUFFER_SIZE;  // Promedio entero

			//Promedio para eje Y
			uint32_t sumaTotal_ejeY = 0;
			for(int i = 0; i < BUFFER_SIZE/2; i++) {
				sumaTotal_ejeY += y_values[i];
			}
			display_value_YY = 2*sumaTotal_ejeY / BUFFER_SIZE;  // Promedio entero

			data_ready = 0;

			// AQUÍ PROCESAR LOS DATOS (x_values y y_values)
		}

		if(display7segmentFLAG)
		{
			stateMachine.state = DISPLAY;
			display7segmentFLAG = 0;
		}

	}
	return stateMachine.state;

	case DISPLAY:
	{

		update7SegmentDisplay();
//		update7SegmentDisplay_YY();
		stateMachine.state=IDLE;
	}
	return stateMachine.state;

	default:
		{
		stateMachine.state = IDLE;
		return stateMachine.state;
		}

	}
}

// %%%%%%%%% CALLBACK %%%%%%%%%%%%
// Callback de media transferencia DMA
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
//    half_transfer_flag = 1;
//    full_transfer_flag = 0;
//}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    // Se llenó la primera mitad del buffer actual
    switch(current_buffer_section) {
        case BUFFER_PING_FIRST_HALF:
            current_buffer_section = BUFFER_PING_SECOND_HALF;
            break;
        case BUFFER_PONG_FIRST_HALF:
            current_buffer_section = BUFFER_PONG_SECOND_HALF;
            break;
        default:
            // Manejo de error
            break;
    }
    data_ready = 1;
}

//// Callback de transferencia completa DMA
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//    full_transfer_flag = 1;
//    half_transfer_flag = 0;
//}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    // Se llenó la segunda mitad del buffer actual + cambio de buffer
    switch(current_buffer_section) {
        case BUFFER_PING_SECOND_HALF:
            // Cambiar a pong y actualizar DMA
            HAL_ADC_Stop_DMA(&hadc1);
            HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer_pong, 2 * BUFFER_SIZE);
            current_buffer_section = BUFFER_PONG_FIRST_HALF;
            break;

        case BUFFER_PONG_SECOND_HALF:
            // Cambiar a ping y actualizar DMA
            HAL_ADC_Stop_DMA(&hadc1);
            HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer_ping, 2 * BUFFER_SIZE);
            current_buffer_section = BUFFER_PING_FIRST_HALF;
            break;

        default:
            // Manejo de error
            break;
    }
    data_ready = 1;
}

// %%%%%%% TIMER - EXTI %%%%%%%%%%%
//Timers
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		timer2FLAG = 1;
	}
	else if(htim->Instance == TIM4)
	{
		display7segmentFLAG=1;
	}
}

//EXTI
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	if(GPIO_Pin == pinEncoderCLK_Pin)
//	{
//		valorCLK = HAL_GPIO_ReadPin(pinEncoderCLK_GPIO_Port, pinEncoderCLK_Pin);
//		valorDT = HAL_GPIO_ReadPin(pinEncoderDT_GPIO_Port, pinEncoderDT_Pin);
//		encoderCLKextiFLAG = 1;
//	}
	if(GPIO_Pin == pinEncoderSW_Pin)
	{
		encoderSWextiFLAG = 1;
	}
}

/* USER CODE END 4 */

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
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
