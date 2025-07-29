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
#include <math.h>  // Necesario para funciones trigonométricas
#include "maquina_estados_hal.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    I2C_HandleTypeDef* i2c_handle;
    uint8_t             i2c_addr;
    uint8_t             backlight;
} DisplayDevice_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUF_SIZE 64
#define BUFFER_SIZE 128  // Tamaño de cada buffer (ajustar según necesidades)

// Parámetros geométricos ajustados
//#define ARM_LENGTH    9.5f   // Longitud del brazo del servo en cm
#define TRIANGLE_SIDE 17.0f   // Lado del triángulo equilátero en cm
#define ARM_LENGTH    9.5f   // Longitud del brazo del servo en cm
#define SIDE_AC       17.0f  // Lado AC en cm
#define SIDE_BC       17.0f  // Lado BC en cm
#define BASE_AB       19.0f  // Base AB en cm
// --- Direcciones I2C ---
#define LCD_I2C_ADDR            (0x23 << 1) // Cambiar a la dirección de tu LCD
//=============================================================================
#pragma region /// Driver para Pantalla LCD (integrado) ///
//=============================================================================
//#define SIDE_BASE     19.0f   // Base diferente (BC)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint16_t x_values[BUFFER_SIZE];
uint16_t y_values[BUFFER_SIZE];
uint16_t offset = 0;

uint8_t contadorDigito_XX = 0;
uint8_t contadorDigito_YY = 0;
volatile uint16_t display_value_XX = 0;  // Valor a mostrar en el display XX (0-100)
volatile uint16_t display_value_YY = 0;  // Valor a mostrar en el display YY (0-100)

// Flags de estado
volatile uint8_t data_ready = 0;
volatile uint8_t ping_active = 1;  // 1=Ping activo, 0=Pong activo
volatile uint8_t half_transfer_flag = 0;
volatile uint8_t full_transfer_flag = 0;
// Control de buffers
volatile enum {
    BUFFER_PING_FIRST_HALF,
    BUFFER_PING_SECOND_HALF,
    BUFFER_PONG_FIRST_HALF,
    BUFFER_PONG_SECOND_HALF
} current_buffer_section = BUFFER_PING_FIRST_HALF;

// Buffers Ping-Pong para X e Y (intercalados)
volatile uint16_t adc_buffer_ping[2 * BUFFER_SIZE];  // [X0, Y0, X1, Y1, ...]
volatile uint16_t adc_buffer_pong[2 * BUFFER_SIZE];  // [X0, Y0, X1, Y1, ...]

volatile uint8_t uart_tx_busy = 0; // Nueva bandera para estado TX
uint8_t command_buffer[UART_RX_BUF_SIZE];  // Nuevo búfer para copia segura
uint8_t main_rx_buffer[UART_RX_BUF_SIZE]; // Búfer para construir el comando
uint8_t dma_rx_byte;                      // Búfer de 1 byte para que el DMA escriba
volatile uint8_t uart_rx_flag = 0;
volatile uint8_t uart_rx_index = 0;

fsm_states_t stateMachine = {0};
DisplayDevice_t main_display;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void actualizar_display_estado(e_PosibleStates estado_actual);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) ;
void ProcessUARTCommand(char* command);
void comandoHelp(void);
void clean_command(char* cmd);
// --- Prototipos para el driver del LCD ---
void display_init(DisplayDevice_t* display, I2C_HandleTypeDef* i2c, uint8_t addr);
void display_clear(DisplayDevice_t* display);
void display_set_cursor(DisplayDevice_t* display, uint8_t row, uint8_t col);
void display_write_string(DisplayDevice_t* display, const char* str);
void refresh_display_data(void);

e_PosibleStates state_machine_action(uint8_t event);
void cartesianToServoAngles(float x, float y, uint16_t angles[3]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Funciones de bajo nivel para el LCD
static void display_send_nibble(DisplayDevice_t* display, uint8_t nibble, uint8_t is_data) {
    uint8_t i2c_data = (nibble << 4) | (is_data ? 0x01 : 0x00) | display->backlight;
    HAL_I2C_Master_Transmit(display->i2c_handle, display->i2c_addr, &i2c_data, 1, 10);
    i2c_data |= 0x04; // EN = 1
    HAL_I2C_Master_Transmit(display->i2c_handle, display->i2c_addr, &i2c_data, 1, 10);
    i2c_data &= ~0x04; // EN = 0
    HAL_I2C_Master_Transmit(display->i2c_handle, display->i2c_addr, &i2c_data, 1, 10);
}

static void display_send_byte(DisplayDevice_t* display, uint8_t byte, uint8_t is_data) {
    display_send_nibble(display, byte >> 4, is_data);
    display_send_nibble(display, byte & 0x0F, is_data);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer_ping, 2 * BUFFER_SIZE);

  HAL_UART_Receive_DMA(&huart2, &dma_rx_byte, 1);

  HAL_TIM_Base_Start(&htim2);				// muestreo ADC
  HAL_TIM_Base_Start_IT(&htim4);
  // Iniciar los 3 canales PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // PC6
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // PC7
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  // PC8


//  // Posición inicial (centro)
//  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
//  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
//  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500);
//  HAL_Delay(2000);

  // CORRECCIÓN: Inicializar la pantalla en I2C1
  display_init(&main_display, &hi2c1, LCD_I2C_ADDR);
  // Mensaje de bienvenida
  display_set_cursor(&main_display, 0, 2);
  display_write_string(&main_display, "Proyecto Final");
  display_set_cursor(&main_display, 1, 3);
  display_write_string(&main_display, "robot 3RRR");
  HAL_Delay(2000);
  display_clear(&main_display);

  // Variable para guardar el estado anterior y detectar cambios
   e_PosibleStates estado_previo = -1; // Valor inicial que no coincide con ningún estado

   // Actualizar la pantalla con el estado inicial
   actualizar_display_estado(stateMachine.state);
   estado_previo = stateMachine.state;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  const float circle_radius = 3.0f;  // Radio del círculo en cm
//  const uint16_t points = 100;       // Puntos para aproximar el círculo
//  const uint16_t delay = 20;         // Tiempo entre puntos (ms)

  while (1)
  {
	  state_machine_action(0);
	     // Comprobar si el estado ha cambiado
	      if (stateMachine.state != estado_previo) {
	          // Si cambió, actualizar la pantalla
	          actualizar_display_estado(stateMachine.state);
	          // Guardar el nuevo estado como el previo para la siguiente iteración
	          estado_previo = stateMachine.state;
	      }

	  // dibujar círculo completo
//	          for(uint16_t i = 0; i < points; i++) {
//	              // calcular posición en el círculo (coordenadas cartesianas)
//	              float angle = 2 * M_PI * i / points;
//	              float x = circle_radius * cosf(angle);
//	              float y = circle_radius * sinf(angle);
//
//	              // calcular ángulos de los servos
//	              uint16_t servo_angles[3];
//	              cartesianToServoAngles(x, y, servo_angles);
//
//	              // enviar comandos a los servos
//	              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_angles[0]);
//	              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo_angles[1]);
//	              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo_angles[2]);
//
//	              HAL_Delay(delay);
//	          }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 20000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 200-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 40000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  htim2.Init.Prescaler = 25-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim4.Init.Prescaler = 50000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(pinH1Led2Board_GPIO_Port, pinH1Led2Board_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : pinH1Led2Board_Pin */
  GPIO_InitStruct.Pin = pinH1Led2Board_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(pinH1Led2Board_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI1_joystickSW_Pin */
  GPIO_InitStruct.Pin = EXTI1_joystickSW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI1_joystickSW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Funciones públicas para el LCD
void display_set_cursor(DisplayDevice_t* display, uint8_t row, uint8_t col) {
    const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    display_send_byte(display, 0x80 | (row_offsets[row] + col), 0);
}

void display_write_string(DisplayDevice_t* display, const char* str) {
    while (*str) {
        display_send_byte(display, *str++, 1);
    }
}

void display_clear(DisplayDevice_t* display) {
    display_send_byte(display, 0x01, 0);
    HAL_Delay(2);
}

void display_init(DisplayDevice_t* display, I2C_HandleTypeDef* i2c, uint8_t addr) {
    display->i2c_handle = i2c;
    display->i2c_addr = addr;
    display->backlight = 0x08; // Backlight ON

    HAL_Delay(50);
    display_send_nibble(display, 0x03, 0); HAL_Delay(5);
    display_send_nibble(display, 0x03, 0); HAL_Delay(1);
    display_send_nibble(display, 0x03, 0); HAL_Delay(1);
    display_send_nibble(display, 0x02, 0); // Modo 4 bits

    display_send_byte(display, 0x28, 0); // 2 líneas, 5x8
    display_send_byte(display, 0x0C, 0); // Display ON, Cursor OFF
    display_send_byte(display, 0x06, 0); // Incremento de cursor
    display_clear(display);
}




/**
  * @brief Envía el mensaje de ayuda o informa si el UART está ocupado.
  */
void comandoHelp(void) {
    // Verificamos si el transmisor del UART está libre
    if (!uart_tx_busy)
    {
        const char *help_message =
            "\r\n--- MENU DE AYUDA ---\r\n"
            "Comandos disponibles:\r\n"
            "HELP		- Muestra esta ayuda\r\n"
            "CIRCULO	- Inicia el dibujo del circulo\r\n"
        	"CENTRAR	- Lleva el lacipero al centro de la base\r\n"
			"CORAZON    - Dibuja un corazon\r\n"
			"LINEA_AB   - Dibuja una linea paralela al eje AB\r\n"
			"LINEA_BC   - Dibuja una linea paralela al eje BC\r\n"
			"LINEA_CA   - Dibuja una linea paralela al eje CA\r\n"
            "---------------------\r\n\r\n";

        uart_tx_busy = 1;
        HAL_UART_Transmit_IT(&huart2, (uint8_t*)help_message, strlen(help_message));
    }
    else
    {
        // Si el UART está ocupado, no hacemos nada o enviamos un mensaje de error corto.
        // Ojo: esta transmisión corta también podría fallar si el UART está muy congestionado,
        // pero es improbable en este caso. La opción más segura es no hacer nada.
        // const char *busy_msg = "UART ocupado, intente de nuevo.\r\n";
        // HAL_UART_Transmit_DMA(&huart2, (uint8_t*)busy_msg, strlen(busy_msg));
    }
}


void ProcessUARTCommand(char* command)
{
    clean_command(command);  // Limpiar caracteres de nueva línea

	  // Comando para cambiar frecuencia del LED
	  if (strcmp(command, "HELP") == 0) {
	        comandoHelp();
	}
	else if (strcmp(command, "CIRCULO") == 0) {
		// Cambiar el estado de la máquina para que dibuje el círculo
		stateMachine.state = CIRCULO;
	}
	else if (strcmp(command, "IDLE") == 0) {
		// Cambiar el estado de la máquina para que dibuje el círculo
		stateMachine.state = IDLE;
	}
    else if (strcmp(command, "LINEA_AB") == 0) {
        stateMachine.state = LINEA_PARALELA_AB; // Cambiar al estado LINEA
    }
    else if (strcmp(command, "LINEA_BC") == 0) {
        stateMachine.state = LINEA_PARALELA_BC;
    }
    else if (strcmp(command, "LINEA_CA") == 0) {
        stateMachine.state = LINEA_PARALELA_CA;
    }
    else if (strcmp(command, "CENTRAR") == 0) {
        stateMachine.state = CENTRO;
    }
    else if (strcmp(command, "CORAZON") == 0) {
        stateMachine.state = CORAZON;
    }
    else {
        if (!uart_tx_busy) {
            const char *error_msg =
			"Comando no reconocido. Enviar HELP para información\r\n"
			"¡IMPORTANTE! todo comando debe llevar ENTER al final\r\n";

            uart_tx_busy = 1;
            HAL_UART_Transmit_IT(&huart2, (uint8_t*)error_msg, strlen(error_msg));
        }
    }

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
/**
  * @brief Actualiza la pantalla LCD para mostrar el estado actual de la máquina.
  * @param estado_actual: El estado a mostrar en la pantalla.
  */
void actualizar_display_estado(e_PosibleStates estado_actual) {
    char lcd_buffer[20]; // Búfer para formatear el texto

    // Limpiar el búfer
    memset(lcd_buffer, 0, sizeof(lcd_buffer));

    // Seleccionar el texto basado en el estado actual
    switch (estado_actual) {
        case IDLE:
            sprintf(lcd_buffer, "ESTADO: IDLE");
            break;
        case CIRCULO:
            sprintf(lcd_buffer, "ESTADO: CIRCULO");
            break;
        case LINEA_PARALELA_AB:
            sprintf(lcd_buffer, "ESTADO: LINEA AB");
            break;
        case LINEA_PARALELA_BC:
            sprintf(lcd_buffer, "ESTADO: LINEA BC");
            break;
        case LINEA_PARALELA_CA:
            sprintf(lcd_buffer, "ESTADO: LINEA CA");
            break;
        case CENTRO:
            sprintf(lcd_buffer, "ESTADO: CENTRADO");
            break;
        case CORAZON:
            sprintf(lcd_buffer, "ESTADO: CORAZON");
            break;
        default:
            sprintf(lcd_buffer, "ESTADO: ???");
            break;
    }

    // Escribir en la pantalla LCD
    display_clear(&main_display);
    display_set_cursor(&main_display, 0, 0); // Escribir en la primera fila
    display_write_string(&main_display, lcd_buffer);
}

// maquina de estados
e_PosibleStates state_machine_action(uint8_t event)
{
	switch (stateMachine.state){
	case IDLE:
	{
	    if(uart_rx_flag) {
	        // 1. Deshabilitar interrupciones brevemente
	        __disable_irq();

	        // 2. Copiar comando a búfer seguro
	        strcpy((char*)command_buffer, (char*)main_rx_buffer);
	        uart_rx_index = 0;            // Resetear índice
	        uart_rx_flag = 0;             // Bajar bandera

	        // Limpiar el búfer de recepción para el siguiente comando
	        memset(main_rx_buffer, 0, UART_RX_BUF_SIZE); // <--- AÑADE ESTA LÍNEA

	        // 3. Habilitar interrupciones
	        __enable_irq();

	        // 4. Procesar comando desde búfer seguro
	        ProcessUARTCommand((char*)command_buffer);
	    }
	    if(data_ready)
	    	   {
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




	}		return stateMachine.state;
	case CIRCULO:
	{
	    // Revisa si ha llegado un nuevo comando
	    if (uart_rx_flag) {
	        __disable_irq();
	        strcpy((char*)command_buffer, (char*)main_rx_buffer);
	        uart_rx_index = 0;
	        uart_rx_flag = 0;
	        memset(main_rx_buffer, 0, UART_RX_BUF_SIZE);
	        __enable_irq();

	        // Procesa el nuevo comando para decidir si cambia de estado
	        ProcessUARTCommand((char*)command_buffer);

	    } else { // Si no hay nuevo comando, sigue dibujando el círculo
	        const float circle_radius = 3.0f;
	        const uint16_t points = 100;
	        const uint16_t delay = 20;

	        for (uint16_t i = 0; i < points; i++) {
	            // Si durante el dibujo llega un comando, interrumpe el círculo y procesa
	            if (uart_rx_flag) break;

	            float angle = 2 * M_PI * i / points;
	            float x = circle_radius * cosf(angle);
	            float y = circle_radius * sinf(angle);

	            uint16_t servo_angles[3];
	            cartesianToServoAngles(x, y, servo_angles);

	            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_angles[0]);
	            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo_angles[1]);
	            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo_angles[2]);

	            HAL_Delay(delay);
	        }
	    }
	}return stateMachine.state;
//	case CIRCULO:
//	{
//		  const float circle_radius = 3.0f;  // Radio del círculo en cm
//		  const uint16_t points = 100;       // Puntos para aproximar el círculo
//		  const uint16_t delay = 20;         // Tiempo entre puntos (ms)
//	// dibujar círculo completo
//		  for(uint16_t i = 0; i < points; i++) {
//			  // calcular posición en el círculo (coordenadas cartesianas)
//			  float angle = 2 * M_PI * i / points;
//			  float x = circle_radius * cosf(angle);
//			  float y = circle_radius * sinf(angle);
//
//			  // calcular ángulos de los servos
//			  uint16_t servo_angles[3];
//			  cartesianToServoAngles(x, y, servo_angles);
//
//			  // enviar comandos a los servos
//			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_angles[0]);
//			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo_angles[1]);
//			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo_angles[2]);
//
//			  HAL_Delay(delay);
//		  }
//		stateMachine.state=IDLE;
//	}return stateMachine.state;
	case LINEA_PARALELA_AB:
	{
	    // Revisa si ha llegado un nuevo comando
	    if (uart_rx_flag) {
	        __disable_irq();
	        strcpy((char*)command_buffer, (char*)main_rx_buffer);
	        uart_rx_index = 0;
	        uart_rx_flag = 0;
	        memset(main_rx_buffer, 0, UART_RX_BUF_SIZE);
	        __enable_irq();

	        // Procesa el nuevo comando para decidir si cambia de estado
	        ProcessUARTCommand((char*)command_buffer);

	    }
	    else {
            // Parámetros de la línea
            const float y_constante = 2.0f; // Distancia de la línea al centro (en cm)
            const float x_inicio = -4.0f;   // Punto X inicial
            const float x_fin = 4.0f;       // Punto X final
            const uint16_t points = 100;
            const uint16_t delay = 20;

            // Bucle para dibujar la línea
            for (uint16_t i = 0; i < points; i++) {
                // Interpolar para obtener la posición X actual
                float x = x_inicio + (x_fin - x_inicio) * i / (points - 1);
                float y = y_constante; // La coordenada Y es siempre la misma

                uint16_t servo_angles[3];
                cartesianToServoAngles(x, y, servo_angles);

                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_angles[0]);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo_angles[1]);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo_angles[2]);
                HAL_Delay(delay);
            }

	    }

	}
	return stateMachine.state;

	case LINEA_PARALELA_BC:
	{
	    // Revisa si ha llegado un nuevo comando
	    if (uart_rx_flag) {
	        __disable_irq();
	        strcpy((char*)command_buffer, (char*)main_rx_buffer);
	        uart_rx_index = 0;
	        uart_rx_flag = 0;
	        memset(main_rx_buffer, 0, UART_RX_BUF_SIZE);
	        __enable_irq();

	        // Procesa el nuevo comando para decidir si cambia de estado
	        ProcessUARTCommand((char*)command_buffer);

	    }
	    else{
		// Parámetros de la línea base (horizontal)
			const float y_offset = 2.0f;
			const float x_inicio = -4.0f, x_fin = 4.0f;
			const uint16_t points = 100, delay = 20;

			// Ángulo de rotación para alinear con BC (-60 grados)
			const float angulo_rotacion = -60.0f * M_PI / 180.0f;

			for (uint16_t i = 0; i < points; i++) {
				// 1. Generar punto en la línea horizontal base
				float x_base = x_inicio + (x_fin - x_inicio) * i / (points - 1);
				float y_base = y_offset;

				// 2. Rotar el punto
				float x_final = x_base * cosf(angulo_rotacion) - y_base * sinf(angulo_rotacion);
				float y_final = x_base * sinf(angulo_rotacion) + y_base * cosf(angulo_rotacion);

				// 3. Mover los servos
				uint16_t servo_angles[3];
				cartesianToServoAngles(x_final, y_final, servo_angles);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_angles[0]);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo_angles[1]);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo_angles[2]);
				HAL_Delay(delay);
			}

	    }

	}return stateMachine.state;

	case LINEA_PARALELA_CA:
	{
	    // Revisa si ha llegado un nuevo comando
	    if (uart_rx_flag) {
	        __disable_irq();
	        strcpy((char*)command_buffer, (char*)main_rx_buffer);
	        uart_rx_index = 0;
	        uart_rx_flag = 0;
	        memset(main_rx_buffer, 0, UART_RX_BUF_SIZE);
	        __enable_irq();

	        // Procesa el nuevo comando para decidir si cambia de estado
	        ProcessUARTCommand((char*)command_buffer);

	    }
	    else{
            // Parámetros de la línea base (horizontal)
            const float y_offset = 2.0f;
            const float x_inicio = -4.0f, x_fin = 4.0f;
            const uint16_t points = 100, delay = 20;

            // Ángulo de rotación para alinear con CA (+60 grados)
            const float angulo_rotacion = 60.0f * M_PI / 180.0f;

            for (uint16_t i = 0; i < points; i++) {
                // 1. Generar punto en la línea horizontal base
                float x_base = x_inicio + (x_fin - x_inicio) * i / (points - 1);
                float y_base = y_offset;

                // 2. Rotar el punto
                float x_final = x_base * cosf(angulo_rotacion) - y_base * sinf(angulo_rotacion);
                float y_final = x_base * sinf(angulo_rotacion) + y_base * cosf(angulo_rotacion);

                // 3. Mover los servos
                uint16_t servo_angles[3];
                cartesianToServoAngles(x_final, y_final, servo_angles);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_angles[0]);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo_angles[1]);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo_angles[2]);
                HAL_Delay(delay);
            }
	    }

		}return stateMachine.state;

	case CENTRO:
	{
		stateMachine.state = IDLE;

		   uint16_t servo_angles[3];

			// 1. Calcular ángulos para la posición (0,0)
			cartesianToServoAngles(0.0f, 0.0f, servo_angles);

			// 2. Mover los servos a esa posición
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_angles[0]);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo_angles[1]);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo_angles[2]);



	}return stateMachine.state;
	case CORAZON:
	{
	    // Revisa si ha llegado un nuevo comando
	    if (uart_rx_flag) {
	        __disable_irq();
	        strcpy((char*)command_buffer, (char*)main_rx_buffer);
	        uart_rx_index = 0;
	        uart_rx_flag = 0;
	        memset(main_rx_buffer, 0, UART_RX_BUF_SIZE);
	        __enable_irq();

	        // Procesa el nuevo comando para decidir si cambia de estado
	        ProcessUARTCommand((char*)command_buffer);

	    }
	    else{
	    	 const uint16_t points = 150; // Más puntos para una curva más suave
			const uint16_t delay = 20;
			const float escala = 0.25f; // Factor para ajustar el tamaño del corazón

			for (uint16_t i = 0; i < points; i++) {
				// El parámetro 't' va de 0 a 2*PI
				float t = 2 * M_PI * i / points;

				// Ecuaciones paramétricas para un corazón
				float x = escala * (16 * powf(sinf(t), 3));
				float y = escala * (13 * cosf(t) - 5 * cosf(2*t) - 2 * cosf(3*t) - cosf(4*t));

				// Desplazamiento para centrar el corazón verticalmente
				float y_final = y + 1.5f;

				uint16_t servo_angles[3];
				cartesianToServoAngles(x, y_final, servo_angles);

				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_angles[0]);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo_angles[1]);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo_angles[2]);
				HAL_Delay(delay);
				}
	    }

	}return stateMachine.state;


	default:
		{
		stateMachine.state = IDLE;
		}return stateMachine.state;


	}
    // Retornar el estado actual al final de la función
	return stateMachine.state;

}

// Función para convertir coordenadas (x,y) a ángulos de servo
void cartesianToServoAngles(float x, float y, uint16_t angles[3]) {
    // Calcular la altura del triángulo equilátero
    const float h = TRIANGLE_SIDE * sqrtf(3) / 2;

    // Posiciones de los servos (coordenadas cartesianas)
    const float servo_pos[3][2] = {
        { 0, 2*h/3 },          // Servo 1 (vértice superior)
        { -TRIANGLE_SIDE/2, -h/3 }, // Servo 2 (vértice inferior izquierdo)
        { TRIANGLE_SIDE/2, -h/3 }   // Servo 3 (vértice inferior derecho)
    };

    for(int i = 0; i < 3; i++) {
        // Calcular vector desde el servo al punto
        float dx = x - servo_pos[i][0];
        float dy = y - servo_pos[i][1];

        // Calcular ángulo (en radianes)
        float angle = atan2f(dy, dx);

        // Convertir a pulsos PWM (500-2500 μs = 0°-180°)
        angles[i] = 500 + (uint16_t)((angle + M_PI) * (2000.0f / (2 * M_PI)));
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{
		HAL_GPIO_TogglePin(pinH1Led2Board_GPIO_Port, pinH1Led2Board_Pin);	}

}
/**
  * @brief  Callback que se ejecuta cuando la recepción UART por DMA está completa.
  * @param  huart: puntero a la estructura UART.
  * @retval None
  */
// Modificar la callback de UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART2) {
        if (dma_rx_byte == '\r' || dma_rx_byte == '\n') {
            if (uart_rx_index > 0) {
                main_rx_buffer[uart_rx_index] = '\0'; // Terminar string
                uart_rx_flag = 1;                     // Levantar bandera
                uart_rx_index = 0;
            }
        }
        else if (uart_rx_index < UART_RX_BUF_SIZE - 1) {
            main_rx_buffer[uart_rx_index++] = dma_rx_byte;
        }
        HAL_UART_Receive_DMA(huart, &dma_rx_byte, 1); // Reactivar DMA
    }
}
// Callback de transmisión UART completada
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        uart_tx_busy = 0; // Liberar el UART
    }
}

//adc
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
