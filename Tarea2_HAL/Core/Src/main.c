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
#include <stdint.h>
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t timer2FLAG = 0;
volatile uint8_t display7segmentFLAG = 0;
volatile uint8_t encoderCLKextiFLAG = 0;
volatile uint8_t encoderSWextiFLAG = 0;

uint8_t valorCLK=0;
uint8_t valorDT =0;

uint8_t contadorDigito = 0;
uint16_t contador = 0;
uint8_t miles = 0;
uint8_t centenas = 0;
uint8_t decenas = 0;
uint8_t	unidades = 0;

fsm_states_t stateMachine = {0};

GPIO_Handler_t pinH1Led2Board = {0};
GPIO_Handler_t pinEncoderCLK = {0};
GPIO_Handler_t pinEncoderDT = {0};
GPIO_Handler_t pinEncoderSW = {0};
GPIO_Handler_t pinSegmentA = {0};
GPIO_Handler_t pinSegmentB = {0};
GPIO_Handler_t pinSegmentC = {0};
GPIO_Handler_t pinSegmentD = {0};
GPIO_Handler_t pinSegmentE = {0};
GPIO_Handler_t pinSegmentF = {0};
GPIO_Handler_t pinSegmentG = {0};
GPIO_Handler_t pinDigit1 = {0};
GPIO_Handler_t pinDigit2 = {0};
GPIO_Handler_t pinDigit3 = {0};
GPIO_Handler_t pinDigit4 = {0};

Timer_Handler_t display7SegmentTime = {0};
Timer_Handler_t blinkLedPinH1 = {0};

EXTI_Config_t pinExtiEncoderCLK = {0};
EXTI_Config_t pinExtiEncoderSW = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
e_PosibleStates state_machine_action(uint8_t event);
void gpioConfig(void);
void extiConfig(void);
void timerConfig(void);
void segmentoON(uint8_t number);
void divideNumber(uint16_t contador);
void mostrarUnidades(void);
void mostrarDecenas(void);
void mostrarCentenas(void);
void mostrarMiles(void);
void update7SegmentDisplay(void);

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(display7segmentFLAG)
	  		{
	  			update7SegmentDisplay();
	  			display7segmentFLAG = 0;
	  		}

	  		if(timer2FLAG)
	  		{
	  			gpio_TogglePin(&pinH1Led2Board);
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
  htim3.Init.Prescaler = 16000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  HAL_HAL_GPIO_WritePin(pinH1Led2Board_GPIO_Port, pinH1Led2Board_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_HAL_GPIO_WritePin(GPIOC, pinDigit2_Pin|pinDigit1_Pin|pinSegmentE_Pin|pinSegmentD_Pin
                          |pinSegmentF_Pin|pinSegmentG_Pin|pinSegmentC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_HAL_GPIO_WritePin(pinSegmentA_GPIO_Port, pinSegmentA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_HAL_GPIO_WritePin(GPIOA, pinDigit3_Pin|pinDigit4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_HAL_GPIO_WritePin(pinSegmentB_GPIO_Port, pinSegmentB_Pin, GPIO_PIN_RESET);

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
// maquina de estados
e_PosibleStates state_machine_action(uint8_t event)
{
	switch (stateMachine.state){
	case IDLE:
	{

	}
	return stateMachine.state;

	case ROTACION:
	{

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
	HAL_GPIO_WritePin(&pinDigit1, SET);
	HAL_GPIO_WritePin(&pinDigit2, SET);
	HAL_GPIO_WritePin(&pinDigit3, SET);
	segmentoON(unidades);
	HAL_GPIO_WritePin(&pinDigit4, RESET);
}

void mostrarDecenas(void)
{
	HAL_GPIO_WritePin(&pinDigit1, SET);
	HAL_GPIO_WritePin(&pinDigit2, SET);
	HAL_GPIO_WritePin(&pinDigit4, SET);
	segmentoON(decenas);
	HAL_GPIO_WritePin(&pinDigit3, RESET);
}

void mostrarCentenas(void)
{
	HAL_GPIO_WritePin(&pinDigit1, SET);
	HAL_GPIO_WritePin(&pinDigit4, SET);
	HAL_GPIO_WritePin(&pinDigit3, SET);
	segmentoON(centenas);
	HAL_GPIO_WritePin(&pinDigit2, RESET);
}

void mostrarMiles(void)
{
	HAL_GPIO_WritePin(&pinDigit4, SET);
	HAL_GPIO_WritePin(&pinDigit2, SET);
	HAL_GPIO_WritePin(&pinDigit3, SET);
	segmentoON(miles);
	HAL_GPIO_WritePin(&pinDigit1, RESET);
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
	else if(htim->Instance == TIM3)
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
