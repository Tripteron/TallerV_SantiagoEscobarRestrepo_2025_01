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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t timer2FLAG = 0;
volatile uint8_t display7segmentFLAG = 0;
volatile uint8_t encoderCLKextiFLAG = 0;
volatile uint8_t encoderSWextiFLAG = 0;
volatile uint16_t adc_buffer[2] = {0}; // [0]=X, [1]=Y
volatile uint8_t joystick_x = 0, joystick_y = 0;

uint8_t valorCLK=0;
uint8_t valorDT =0;

uint8_t contadorDigito = 0;
uint16_t contador = 0;
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
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  stateMachine.state=IDLE;
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
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
  sConfigOC.Pulse = 5000;
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000;
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
}

// maquina de estados
e_PosibleStates state_machine_action(uint8_t event)
{
	switch (stateMachine.state){
	case IDLE:
	{
		if(display7segmentFLAG)
		{
			update7SegmentDisplay();
			display7segmentFLAG = 0;
		}

		if(timer2FLAG)
		{
			HAL_GPIO_TogglePin(pinH1Led2Board_GPIO_Port, pinH1Led2Board_Pin);
			timer2FLAG = 0;
		}
		if(encoderCLKextiFLAG)
		{
			stateMachine.state = ROTACION;
			state_machine_action(0);
			encoderCLKextiFLAG = 0;
		}
		if(encoderSWextiFLAG)
		{
			stateMachine.state = BOTON_SW;
			state_machine_action(0);
			encoderSWextiFLAG = 0;
		}
	}
	return stateMachine.state;

	case ROTACION:
	{
		stateMachine.state=IDLE;
		if(valorCLK != valorDT)
		{
			if(contador == 4095)
			{
				contador = 0;
			}
			else
			{
				contador++;

			}
		}
		else
		{
			if(contador == 0)
				{
					contador = 4095;
				}
			else
				{
				contador--;
				}
		}
	}
	return stateMachine.state;

	case BOTON_SW:
	{
		stateMachine.state=IDLE;
		contador = 0;
	}
	return stateMachine.state;

	default:
		{
		stateMachine.state = IDLE;
		return stateMachine.state;
		}

	}
}

// %%%%%%%%% FUNCIONES PRIVADAS USER %%%%%%%%%%%%

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
		65535
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

void update7SegmentDisplay(void)
{
	divideNumber(contador);
	if(contadorDigito == 4)
	{
		contadorDigito = 0;
	}
	switch(contadorDigito)
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
	contadorDigito++;
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
	if(GPIO_Pin == pinEncoderCLK_Pin)
	{
		valorCLK = HAL_GPIO_ReadPin(pinEncoderCLK_GPIO_Port, pinEncoderCLK_Pin);
		valorDT = HAL_GPIO_ReadPin(pinEncoderDT_GPIO_Port, pinEncoderDT_Pin);
		encoderCLKextiFLAG = 1;
	}
	else if(GPIO_Pin == pinEncoderSW_Pin)
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
