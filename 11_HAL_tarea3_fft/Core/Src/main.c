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
// FFT
#include "arm_math.h"
#include <math.h>  // Para funciones sqrtf, powf, etc.

#include "arm_const_structs.h"
#include "maquina_estados_hal.h"
#include <stdint.h>
#include <stdlib.h>  // Para atoi()
#include <string.h>  // Para strncmp()
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Configuración de muestreo
typedef enum {
    FS_44100 = 0,
    FS_48000,
    FS_96000,
    FS_128000
} sample_rate_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUF_SIZE 64
#define FFT_SIZE_MAX 2048
#define ADC_BUF_SIZE 1024
#define ADC_TX_BLOCK_SIZE 64   // Para comandoPrintADC
#define FFT_TX_BLOCK_SIZE 32   // Para comandoPrintFFT
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */


volatile uint8_t flag_adc_ready = 0;

arm_rfft_fast_instance_f32 fftInstance;
float32_t fft_input[FFT_SIZE_MAX];   // Entrada para la FFT (real)
float32_t fft_output[FFT_SIZE_MAX];  // Salida de la FFT (compleja: real, imag, ...)
float32_t fft_mag[FFT_SIZE_MAX / 2]; // Magnitudes (solo mitad significativa)

volatile uint8_t fft_ready = 0;      // Bandera FFT calculada


volatile sample_rate_t current_sample_rate = FS_48000;  // Valor por defecto
volatile uint16_t fft_size = 1024;                     // Tamaño FFT por defecto

static uint16_t adc_buffer[2048];  // Máximo tamaño necesario
volatile uint8_t uart_rx_flag = 0;
uint8_t uart_rx_buffer[UART_RX_BUF_SIZE];
uint8_t uart_rx_index = 0;

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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void set_sample_rate(sample_rate_t new_rate);
void set_fft_size(uint16_t new_size);
void comandoHelp(void);
void comandoPrintFFTPeak(void);
void compute_fft(void);
void clean_command(char* cmd);
void comandoPrintADC(void);
void comandoFrecuenciaMuestreo(char* command);
void comandoRGB(char* command);
void comandoLED(char* command);
void configure_sample_timer(sample_rate_t sample_rate);
void ProcessUARTCommand(char* command);
e_PosibleStates state_machine_action(uint8_t event);
void segmentoON(uint8_t number);
void divideNumber(uint16_t contador);
void mostrarUnidades(void);
void mostrarDecenas(void);
void mostrarCentenas(void);
void mostrarMiles(void);
void update7SegmentDisplay(void);
void InitProgram(void);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  InitProgram();

  arm_rfft_fast_init_f32(&fftInstance, fft_size);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_UART_Receive_IT(&huart2, &uart_rx_buffer[0], 1); // Iniciar recepción UART
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, fft_size);
  configure_sample_timer(current_sample_rate);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  htim3.Init.Prescaler = 25;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1200;
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
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
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
  HAL_GPIO_WritePin(GPIOC, green_rgb_Pin|pinSegmentE_Pin|pinSegmentD_Pin|pinSegmentF_Pin
                          |pinSegmentG_Pin|pinSegmentC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, pinDigit2_Pin|pinDigit1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, pinSegmentA_Pin|red_rgb_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(blue_rgb_GPIO_Port, blue_rgb_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : green_rgb_Pin pinDigit2_Pin pinDigit1_Pin pinSegmentE_Pin
                           pinSegmentD_Pin pinSegmentF_Pin pinSegmentG_Pin pinSegmentC_Pin */
  GPIO_InitStruct.Pin = green_rgb_Pin|pinDigit2_Pin|pinDigit1_Pin|pinSegmentE_Pin
                          |pinSegmentD_Pin|pinSegmentF_Pin|pinSegmentG_Pin|pinSegmentC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : pinEncoderDT_Pin */
  GPIO_InitStruct.Pin = pinEncoderDT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(pinEncoderDT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : pinSegmentA_Pin red_rgb_Pin */
  GPIO_InitStruct.Pin = pinSegmentA_Pin|red_rgb_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : pinEncoderSW_Pin */
  GPIO_InitStruct.Pin = pinEncoderSW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(pinEncoderSW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : blue_rgb_Pin pinDigit3_Pin pinDigit4_Pin */
  GPIO_InitStruct.Pin = blue_rgb_Pin|pinDigit3_Pin|pinDigit4_Pin;
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
		if(flag_adc_ready)
		{
			compute_fft();
			flag_adc_ready = 0;
		}
	    if(uart_rx_flag)
	    {
	    	stateMachine.state=PROCESANDO_COMANDO;
	    	uart_rx_flag = 0;
	    }
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

	case PROCESANDO_COMANDO:
	{
	  stateMachine.state=IDLE;
	  uart_rx_buffer[uart_rx_index] = '\0'; // Terminar cadena
	  ProcessUARTCommand((char*)uart_rx_buffer);
	  uart_rx_index = 0;
	  HAL_UART_Receive_IT(&huart2, &uart_rx_buffer[uart_rx_index], 1);
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


void configure_sample_timer(sample_rate_t sample_rate) {
    // Detener el timer y ADC antes de cambiar
    HAL_TIM_Base_Stop(&htim3);
    HAL_ADC_Stop_DMA(&hadc1);

    // Configurar TIM3 según la frecuencia seleccionada
    switch(sample_rate) {
        case FS_44100:
            __HAL_TIM_SET_PRESCALER(&htim3, 0);
            __HAL_TIM_SET_AUTORELOAD(&htim3, 363);  // 16MHz / 44100 ≈ 363
            break;
        case FS_48000:
            __HAL_TIM_SET_PRESCALER(&htim3, 0);
            __HAL_TIM_SET_AUTORELOAD(&htim3, 333);  // 16MHz / 48000 ≈ 333
            break;
        case FS_96000:
            __HAL_TIM_SET_PRESCALER(&htim3, 0);
            __HAL_TIM_SET_AUTORELOAD(&htim3, 167);  // 16MHz / 96000 ≈ 167
            break;
        case FS_128000:
            __HAL_TIM_SET_PRESCALER(&htim3, 0);
            __HAL_TIM_SET_AUTORELOAD(&htim3, 125);  // 16MHz / 128000 = 125
            break;

            // Reinicializar FFT con nuevo tamaño
            arm_rfft_fast_init_f32(&fftInstance, fft_size);
            fft_ready = 0;
    }

    // Reiniciar contador y periféricos
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, fft_size);
}

void comandoLED(char* command)
{
    int freq = atoi(command + 5);  // Convertir valor después del '='

    if (freq >= 1 && freq <= 100) {  // Rango válido 1-100 Hz
      // Calcular nuevo período (TIM2 clock = 16MHz / 16000 = 1KHz)
      uint32_t new_period = (1000 / freq) - 1;

      // Actualizar configuración del TIMER
      HAL_TIM_Base_Stop_IT(&htim2);
      __HAL_TIM_SET_AUTORELOAD(&htim2, new_period);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      HAL_TIM_Base_Start_IT(&htim2);

      // Confirmación por UART
      char response[50];
      int len = snprintf(response, sizeof(response), "OK:FREQ=%dHz\r\n", freq);
      HAL_UART_Transmit(&huart2, (uint8_t*)response, len, 100);
    }
    else {
      const char* error_msg = "ERROR:Frecuencia invalida (1-100Hz)\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, strlen(error_msg), 100);
    }

}

void comandoRGB(char* command)
{

    int r, g, b;
    if (sscanf(command + 4, "%d,%d,%d", &r, &g, &b) == 3) {
      // Validar valores
      r = (r != 0) ? 1 : 0;
      g = (g != 0) ? 1 : 0;
      b = (b != 0) ? 1 : 0;

      // Controlar LEDs
      HAL_GPIO_WritePin(red_rgb_GPIO_Port, red_rgb_Pin, r ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(green_rgb_GPIO_Port, green_rgb_Pin, g ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(blue_rgb_GPIO_Port, blue_rgb_Pin, b ? GPIO_PIN_SET : GPIO_PIN_RESET);

      // Confirmación
      char response[32];
      int len = snprintf(response, sizeof(response), "OK:RGB=%d,%d,%d\r\n", r, g, b);
      HAL_UART_Transmit(&huart2, (uint8_t*)response, len, 100);
    } else {
      const char* error_msg = "ERROR:Formato RGB invalido. Usar RGB=r,g,b\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, strlen(error_msg), 100);
    }

}

//void comandoFrecuenciaMuestreo(char* command)
//{
//
//    char* params = command + 12;
//    char* rate_str = strtok(params, ",");
//    char* size_str = strtok(NULL, ",");
//
//    // Procesar frecuencia
//    sample_rate_t new_rate = current_sample_rate;
//    if (rate_str) {
//        if (strcmp(rate_str, "44100") == 0) new_rate = FS_44100;
//        else if (strcmp(rate_str, "48000") == 0) new_rate = FS_48000;
//        else if (strcmp(rate_str, "96000") == 0) new_rate = FS_96000;
//        else if (strcmp(rate_str, "128000") == 0) new_rate = FS_128000;
//    }
//
//    // Procesar tamaño FFT
//    uint16_t new_size = fft_size;
//    if (size_str) {
//        int size_val = atoi(size_str);
//        if (size_val == 1024 || size_val == 2048) {
//            new_size = size_val;
//        }
//    }
//
//    // Aplicar cambios si son diferentes
//    if (new_rate != current_sample_rate || new_size != fft_size) {
//        current_sample_rate = new_rate;
//        fft_size = new_size;
//        configure_sample_timer(current_sample_rate);
//    }
//
//    // Confirmar configuración
//    const char* rate_names[] = {"44.1KHz", "48KHz", "96KHz", "128KHz"};
//    char response[64];
//    snprintf(response, sizeof(response), "OK:SAMPLE_RATE=%s,FFT_SIZE=%d\r\n",
//            rate_names[current_sample_rate], fft_size);
//    HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
//}
void comandoPrintADC(void) {
    HAL_ADC_Stop_DMA(&hadc1);

    char header[64];
    int len = snprintf(header, sizeof(header), "ADC_RAW:%d samples\r\n", fft_size);
    HAL_UART_Transmit(&huart2, (uint8_t*)header, len, 100);

    char tx_buf[ADC_TX_BLOCK_SIZE * 7]; // Usar define global

    int samples_sent = 0;
    while(samples_sent < fft_size) {
        int block_size = (fft_size - samples_sent) > ADC_TX_BLOCK_SIZE ?
                         ADC_TX_BLOCK_SIZE : (fft_size - samples_sent);
        int buf_pos = 0;

        // Formatear bloque de muestras
        for(int i = 0; i < block_size; i++) {
            int sample = adc_buffer[samples_sent + i];
            buf_pos += snprintf(tx_buf + buf_pos, sizeof(tx_buf) - buf_pos, "%d\n", sample);
        }

        // Enviar bloque
        HAL_UART_Transmit(&huart2, (uint8_t*)tx_buf, buf_pos, HAL_MAX_DELAY);
        samples_sent += block_size;

        // Pequeña pausa para permitir procesamiento de otras tareas
        HAL_Delay(1);
    }

    // Reiniciar adquisición
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, fft_size);

    // Confirmar finalización
    const char* end_msg = "END_ADC_RAW\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)end_msg, strlen(end_msg), 100);
}


void comandoPrintFFT(void) {
    if (!fft_ready) {
        const char* error_msg = "ERROR:FFT no calculada\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, strlen(error_msg), 100);
        return;
    }
    char header[64];
    int len = snprintf(header, sizeof(header), "FFT_SPECTRUM:%d bins\r\n", fft_size / 2);
    HAL_UART_Transmit(&huart2, (uint8_t*)header, len, 100);

    char tx_buf[FFT_TX_BLOCK_SIZE * 16]; // Usar define global

    int bins_sent = 0;
    int total_bins = fft_size / 2;

    while (bins_sent < total_bins) {
        int block_size = (total_bins - bins_sent) > FFT_TX_BLOCK_SIZE ?
                         FFT_TX_BLOCK_SIZE : (total_bins - bins_sent);
        // ... resto del código ...
        int buf_pos = 0;

        // Formatear bloque de bins
        for (int i = 0; i < block_size; i++) {
            buf_pos += snprintf(tx_buf + buf_pos, sizeof(tx_buf) - buf_pos,
                               "%.2f\n", fft_mag[bins_sent + i]);
        }

        // Enviar bloque
        HAL_UART_Transmit(&huart2, (uint8_t*)tx_buf, buf_pos, HAL_MAX_DELAY);
        bins_sent += block_size;

        // Pequeña pausa
        HAL_Delay(1);
    }

    // Confirmar finalización
    const char* end_msg = "END_FFT_SPECTRUM\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)end_msg, strlen(end_msg), 100);
}


void comandoPrintFFTPeak(void) {
    if (!fft_ready) {
        const char* error_msg = "ERROR:FFT no calculada\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, strlen(error_msg), 100);
        return;
    }

    // Calcular parámetros importantes
    int total_bins = fft_size / 2;

    // 1. Encontrar pico fundamental (excluyendo DC)
    int peak_bin = 1; // Comenzar desde bin 1 (el 0 es DC)
    float peak_mag = fft_mag[1];

    for (int i = 2; i < total_bins; i++) {
        if (fft_mag[i] > peak_mag) {
            peak_mag = fft_mag[i];
            peak_bin = i;
        }
    }

    // 2. Calcular frecuencia del pico
    float sample_rate_value;
    switch(current_sample_rate) {
        case FS_44100: sample_rate_value = 44100.0f; break;
        case FS_48000: sample_rate_value = 48000.0f; break;
        case FS_96000: sample_rate_value = 96000.0f; break;
        case FS_128000: sample_rate_value = 128000.0f; break;
        default: sample_rate_value = 48000.0f;
    }

    float bin_width = sample_rate_value / fft_size;
    float peak_freq = peak_bin * bin_width;

    // 3. Calcular potencia total
    float total_power = 0.0f;
    for (int i = 1; i < total_bins; i++) {
        total_power += fft_mag[i] * fft_mag[i];
    }

    // 4. Componente DC
    float dc_offset = fft_mag[0] / fft_size;

    // 5. Calcular THD aproximado (suma de armónicos / fundamental)
    float harmonic_power = 0.0f;
    int fundamental = peak_bin;

    // Considerar hasta 5 armónicos
    for (int h = 2; h <= 5; h++) {
        int harmonic_bin = fundamental * h;
        if (harmonic_bin < total_bins) {
            harmonic_power += fft_mag[harmonic_bin] * fft_mag[harmonic_bin];
        }
    }

    float thd = (harmonic_power > 0) ? sqrtf(harmonic_power) / peak_mag : 0.0f;

    // Preparar y enviar resultados
    char response[256];
    int len = snprintf(response, sizeof(response),
        "FFT_PEAK_RESULTS:\r\n"
        "Fundamental Frequency: %.2f Hz\r\n"
        "Peak Amplitude: %.2f\r\n"
        "Total Power: %.2f\r\n"
        "DC Offset: %.2f\r\n"
        "Approx THD: %.2f%%\r\n"
        "END_FFT_PEAK\r\n",
        peak_freq,
        peak_mag,
        total_power,
        dc_offset,
        thd * 100.0f);

    HAL_UART_Transmit(&huart2, (uint8_t*)response, len, 100);
}
// Función para cambiar tamaño FFT
void set_fft_size(uint16_t new_size) {
    // Solo si es diferente
    if (new_size == fft_size) return;

    // Deshabilitar interrupciones durante el cambio
    __disable_irq();

    // Detener adquisición
    HAL_ADC_Stop_DMA(&hadc1);

    // Actualizar tamaño
    fft_size = new_size;

    // Reinicializar FFT
    arm_rfft_fast_init_f32(&fftInstance, fft_size);

    // Reiniciar ADC con nuevo tamaño
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, fft_size);

    // Resetear estado de FFT
    fft_ready = 0;

    // Habilitar interrupciones
    __enable_irq();
}

// Función para cambiar frecuencia de muestreo
void set_sample_rate(sample_rate_t new_rate) {
    // Solo si es diferente
    if (new_rate == current_sample_rate) return;

    // Deshabilitar interrupciones durante el cambio
    __disable_irq();

    // Detener periféricos
    HAL_TIM_Base_Stop(&htim3);
    HAL_ADC_Stop_DMA(&hadc1);

    // Configurar timer según nueva frecuencia
    switch(new_rate) {
        case FS_44100:
            __HAL_TIM_SET_PRESCALER(&htim3, 0);
            __HAL_TIM_SET_AUTORELOAD(&htim3, 363);
            break;
        case FS_48000:
            __HAL_TIM_SET_PRESCALER(&htim3, 0);
            __HAL_TIM_SET_AUTORELOAD(&htim3, 333);
            break;
        case FS_96000:
            __HAL_TIM_SET_PRESCALER(&htim3, 0);
            __HAL_TIM_SET_AUTORELOAD(&htim3, 167);
            break;
        case FS_128000:
            __HAL_TIM_SET_PRESCALER(&htim3, 0);
            __HAL_TIM_SET_AUTORELOAD(&htim3, 125);
            break;
    }

    // Actualizar variable global
    current_sample_rate = new_rate;

    // Reiniciar periféricos
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, fft_size);

    // Resetear estado de FFT
    fft_ready = 0;

    // Habilitar interrupciones
    __enable_irq();
}

void ProcessUARTCommand(char* command)
{
    clean_command(command);  // Limpiar caracteres de nueva línea

	  // Comando para cambiar frecuencia del LED
	  if (strncmp(command, "FREQ=", 5) == 0) {
		  comandoLED(command);
	  }
	  else if (strncmp(command, "RGB=", 4) == 0) {
		  comandoRGB(command);
	  }
	  // Comando para cambiar SOLO tamaño FFT
	      else if (strncmp(command, "SET_FFT_SIZE=", 13) == 0) {
	          char* size_str = command + 13;
	          int size_val = atoi(size_str);

	          if (size_val == 1024 || size_val == 2048) {
	              set_fft_size((uint16_t)size_val);
	          }
	          else {
	              const char* error_msg = "ERROR:Tamaño FFT invalido (1024,2048)\r\n";
	              HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, strlen(error_msg), 100);
	              return;
	          }

	          // Confirmación
	          char response[40];
	          snprintf(response, sizeof(response), "OK:FFT_SIZE=%d\r\n", fft_size);
	          HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
	      }
	  //Comando cambio de muestreo
//	  else if (strncmp(command, "SAMPLE_RATE=", 12) == 0) {
//		  comandoFrecuenciaMuestreo(command);
//	    }
	  //FIN COMANDO CAMBIAR MUESTREO
	  // Comando para cambiar SOLO frecuencia de muestreo
	     else if (strncmp(command, "SET_SAMPLE_RATE=", 16) == 0) {
	         char* rate_str = command + 16;

	         if (strcmp(rate_str, "44100") == 0) {
	             set_sample_rate(FS_44100);
	         }
	         else if (strcmp(rate_str, "48000") == 0) {
	             set_sample_rate(FS_48000);
	         }
	         else if (strcmp(rate_str, "96000") == 0) {
	             set_sample_rate(FS_96000);
	         }
	         else if (strcmp(rate_str, "128000") == 0) {
	             set_sample_rate(FS_128000);
	         }
	         else {
	             const char* error_msg = "ERROR:Frecuencia invalida (44100,48000,96000,128000)\r\n";
	             HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, strlen(error_msg), 100);
	             return;
	         }

	         // Confirmación
	         const char* rate_names[] = {"44.1KHz", "48KHz", "96KHz", "128KHz"};
	         char response[50];
	         snprintf(response, sizeof(response), "OK:SAMPLE_RATE=%s\r\n", rate_names[current_sample_rate]);
	         HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
	     }
	  else if (strcmp(command, "PRINT_ADC") == 0) {
	        comandoPrintADC();
	    }
	  else if (strcmp(command, "PRINT_FFT") == 0) {
	        comandoPrintFFT();
	    }
	   else if (strcmp(command, "FFT_PEAK") == 0) {
	        comandoPrintFFTPeak();
	    }
	    else if (strcmp(command, "HELP") == 0) {
	        comandoHelp();
	    }


}


void compute_fft(void) {
    // Convertir datos ADC a float
    for (int i = 0; i < fft_size; i++) {
        fft_input[i] = (float32_t)adc_buffer[i];
    }

    // Calcular FFT
    arm_rfft_fast_f32(&fftInstance, fft_input, fft_output, 0);

    // Calcular magnitudes (solo primera mitad)
    arm_cmplx_mag_f32(fft_output, fft_mag, fft_size / 2);

    fft_ready = 1;
}


void comandoHelp(void) {
    const char *help_message =
        "Comandos disponibles:\r\n"
        "FREQ=<frecuencia> - Cambia frecuencia LED (1-100 Hz)\r\n"
        "RGB=<r,g,b> - Enciende/Apaga LEDs RGB (ej: RGB=1,0,1)\r\n"
		"SET_SAMPLE_RATE=<frecuencia> - Configura frecuencia de muestreo (44100,48000,96000,128000)\r\n"
		"SET_FFT_SIZE=<tamaño> - Configura tamaño FFT (1024,2048)\r\n"        "PRINT_ADC - Imprime valores ADC crudos\r\n"
        "PRINT_FFT - Imprime espectro FFT completo\r\n"
        "FFT_PEAK - Imprime parámetros clave de FFT (frecuencia, potencia, etc.)\r\n"
        "HELP - Muestra esta ayuda\r\n"
        "END_HELP\r\n";  // Marcador de fin

    HAL_UART_Transmit(&huart2, (uint8_t*)help_message, strlen(help_message), 100);
}
void clean_command(char* cmd) {
    char* p = cmd;
    while (*p) {
        if (*p == '\r' || *p == '\n') {
            *p = '\0';
            break;
        }
        p++;
    }
}

// %%%%%%% CALLBACKS %%%%%%%%%%%
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
    	flag_adc_ready = 1;
    }
}
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    if (uart_rx_buffer[uart_rx_index] == '\n' || uart_rx_index >= UART_RX_BUF_SIZE - 1 ||uart_rx_buffer[uart_rx_index] == '\r') {
      uart_rx_flag = 1;
    } else {
      uart_rx_index++;
      HAL_UART_Receive_IT(&huart2, &uart_rx_buffer[uart_rx_index], 1);
    }
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
