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
#include "main.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Estructura que representa el dispositivo de pantalla LCD
typedef struct {
    I2C_HandleTypeDef* i2c_handle;
    uint8_t             i2c_addr;
    uint8_t             backlight;
} DisplayDevice_t;
// Estructura que representa el sensor de vibración
typedef struct {
    I2C_HandleTypeDef* i2c_handle;
    uint8_t             i2c_addr;
    int16_t             accel_x, accel_y, accel_z;
} VibrationSensor_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_SAMPLES             1024
#define SAMPLING_FREQUENCY_HZ   1000.0f // Debe coincidir con la configuración del sensor

// --- Direcciones I2C ---
#define MPU6050_I2C_ADDR        (0x68 << 1)
#define LCD_I2C_ADDR            (0x23 << 1) // Cambiar a la dirección de tu LCD

//=============================================================================
#pragma region /// Driver para Pantalla LCD (integrado) ///
//=============================================================================
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
// --- Instancias de nuestros dispositivos ---
DisplayDevice_t main_display;
VibrationSensor_t main_sensor;

// --- Buffers y variables para FFT ---
arm_rfft_fast_instance_f32 fft_instance;
float32_t fft_input_buffer[FFT_SAMPLES];
float32_t fft_output_buffer[FFT_SAMPLES];

int16_t data_capture_ping[FFT_SAMPLES];
int16_t data_capture_pong[FFT_SAMPLES];

volatile int16_t* current_capture_buffer = data_capture_ping;
volatile int16_t* processing_buffer = NULL;
volatile uint16_t capture_index = 0;
volatile bool fft_data_is_ready = false;

// --- Variables de la aplicación ---
float dominant_frequency = 0.0f;
volatile bool new_sensor_data_available = false;
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
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
// --- Prototipos para el driver del LCD ---
void display_init(DisplayDevice_t* display, I2C_HandleTypeDef* i2c, uint8_t addr);
void display_clear(DisplayDevice_t* display);
void display_set_cursor(DisplayDevice_t* display, uint8_t row, uint8_t col);
void display_write_string(DisplayDevice_t* display, const char* str);

// --- Prototipos para el driver del Sensor ---
void sensor_init(VibrationSensor_t* sensor, I2C_HandleTypeDef* i2c, uint8_t addr);
void sensor_acquire_data(VibrationSensor_t* sensor);

// --- Prototipos para las funciones de la aplicación ---
float analyze_vibration_frequency(volatile int16_t* raw_data);
void refresh_display_data(void);
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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  // CORRECCIÓN: Inicializar el sensor en I2C1
  sensor_init(&main_sensor, &hi2c1, MPU6050_I2C_ADDR);

  // CORRECCIÓN: Inicializar la pantalla en I2C2
  display_init(&main_display, &hi2c2, LCD_I2C_ADDR);

      // Inicializar instancia de la librería FFT
      arm_rfft_fast_init_f32(&fft_instance, FFT_SAMPLES);

      // Mensaje de bienvenida
      display_set_cursor(&main_display, 0, 2);
      display_write_string(&main_display, "Analizador de");
      display_set_cursor(&main_display, 1, 3);
      display_write_string(&main_display, "Vibraciones");
      HAL_Delay(2000);
      display_clear(&main_display);

      uint32_t last_screen_update = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // --- Tarea 1: Capturar datos del sensor (manejado por interrupción) ---
	          if (new_sensor_data_available) {
	              new_sensor_data_available = false; // Bajar la bandera

	              sensor_acquire_data(&main_sensor);

	              if (capture_index < FFT_SAMPLES) {
	                  current_capture_buffer[capture_index++] = main_sensor.accel_z;
	              }

	              if (capture_index >= FFT_SAMPLES) {
	                  processing_buffer = current_capture_buffer; // Marcar buffer como listo
	                  fft_data_is_ready = true;
	                  capture_index = 0;

	                  // Intercambiar al otro buffer (Ping-Pong)
	                  current_capture_buffer = (current_capture_buffer == data_capture_ping) ? data_capture_pong : data_capture_ping;
	              }
	          }

	          // --- Tarea 2: Procesar FFT cuando un buffer esté lleno ---
	          if (fft_data_is_ready) {
	              fft_data_is_ready = false; // Bajar la bandera
	              dominant_frequency = analyze_vibration_frequency(processing_buffer);
	          }

	          // --- Tarea 3: Actualizar la pantalla periódicamente sin bloquear ---
	          if (HAL_GetTick() - last_screen_update > 150) { // Actualizar cada 150 ms
	              last_screen_update = HAL_GetTick();
	              refresh_display_data();
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim2.Init.Prescaler = 42000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-2;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, red_rgb_Pin|pinSegmentA_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : mpu_EXTI_Pin */
  GPIO_InitStruct.Pin = mpu_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(mpu_EXTI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : green_rgb_Pin pinDigit2_Pin pinDigit1_Pin pinSegmentE_Pin
                           pinSegmentD_Pin pinSegmentF_Pin pinSegmentG_Pin pinSegmentC_Pin */
  GPIO_InitStruct.Pin = green_rgb_Pin|pinDigit2_Pin|pinDigit1_Pin|pinSegmentE_Pin
                          |pinSegmentD_Pin|pinSegmentF_Pin|pinSegmentG_Pin|pinSegmentC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : red_rgb_Pin pinSegmentA_Pin */
  GPIO_InitStruct.Pin = red_rgb_Pin|pinSegmentA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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

#pragma endregion

//=============================================================================
#pragma region /// Driver para Sensor MPU6050 (integrado) ///
//=============================================================================

// Estructura que representa el sensor de vibración

// Funciones de bajo nivel
static void sensor_write_register(VibrationSensor_t* sensor, uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    HAL_I2C_Master_Transmit(sensor->i2c_handle, sensor->i2c_addr, data, 2, 100);
}

static void sensor_read_registers(VibrationSensor_t* sensor, uint8_t reg, uint8_t* buffer, uint16_t len) {
    HAL_I2C_Master_Transmit(sensor->i2c_handle, sensor->i2c_addr, &reg, 1, 100);
    HAL_I2C_Master_Receive(sensor->i2c_handle, sensor->i2c_addr, buffer, len, 100);
}

// Funciones públicas para el sensor
void sensor_acquire_data(VibrationSensor_t* sensor) {
    uint8_t buffer[6];
    sensor_read_registers(sensor, 0x3B, buffer, 6);
    sensor->accel_x = (int16_t)(buffer[0] << 8 | buffer[1]);
    sensor->accel_y = (int16_t)(buffer[2] << 8 | buffer[3]);
    sensor->accel_z = (int16_t)(buffer[4] << 8 | buffer[5]);
}

void sensor_init(VibrationSensor_t* sensor, I2C_HandleTypeDef* i2c, uint8_t addr) {
    sensor->i2c_handle = i2c;
    sensor->i2c_addr = addr;

    HAL_Delay(100);
    sensor_write_register(sensor, 0x6B, 0x00); // Salir del modo sleep
    HAL_Delay(100);

    // Configurar Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    // Con DLPF desactivado, Gyro Rate = 8kHz. Para 1kHz: 8000 / (1 + 7) = 1000
    sensor_write_register(sensor, 0x1A, 0x00); // DLPF_CFG = 0 (Gyro Rate 8kHz)
    sensor_write_register(sensor, 0x19, 7);    // SMPLRT_DIV = 7 -> 1kHz sample rate
    sensor_write_register(sensor, 0x1C, 0x00); // Rango Acelerómetro: ±2g
    sensor_write_register(sensor, 0x38, 0x01); // Habilitar interrupción 'Data Ready'
}

#pragma endregion

float analyze_vibration_frequency(volatile int16_t* raw_data) {
    // Paso 1: Copiar datos y remover componente DC
    float32_t dc_component = 0.0f;
    for (int i = 0; i < FFT_SAMPLES; i++) {
        dc_component += (float32_t)raw_data[i];
    }
    dc_component /= FFT_SAMPLES;

    for (int i = 0; i < FFT_SAMPLES; i++) {
        // Aplicar ventana de Hann mientras se remueve el DC
        float32_t hann_window = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (FFT_SAMPLES - 1)));
        fft_input_buffer[i] = ((float32_t)raw_data[i] - dc_component) * hann_window;
    }

    // Paso 2: Ejecutar FFT
    arm_rfft_fast_f32(&fft_instance, fft_input_buffer, fft_output_buffer, 0);

    // Paso 3: Calcular magnitud del espectro
    arm_cmplx_mag_f32(fft_output_buffer, fft_input_buffer, FFT_SAMPLES / 2);

    // Paso 4: Encontrar el pico de magnitud (ignorando el bin 0 de DC)
    float32_t max_magnitude;
    uint32_t max_index;
    arm_max_f32(&fft_input_buffer[1], (FFT_SAMPLES / 2) - 1, &max_magnitude, &max_index);
    max_index += 1; // Ajustar índice porque la búsqueda comenzó en 1

    // Paso 5: Calcular frecuencia con interpolación para mayor precisión
    float calculated_freq;
    if (max_index > 0 && max_index < (FFT_SAMPLES / 2) - 1) {
        float y0 = fft_input_buffer[max_index - 1];
        float y1 = fft_input_buffer[max_index];
        float y2 = fft_input_buffer[max_index + 1];

        // Evitar división por cero si los 3 puntos son iguales
        if (fabsf(y0 - 2.0f * y1 + y2) < 1e-6) {
             calculated_freq = (float)max_index * SAMPLING_FREQUENCY_HZ / FFT_SAMPLES;
        } else {
            float delta = 0.5f * (y0 - y2) / (y0 - 2.0f * y1 + y2);
            calculated_freq = (max_index + delta) * SAMPLING_FREQUENCY_HZ / FFT_SAMPLES;
        }
    } else {
        calculated_freq = (float)max_index * SAMPLING_FREQUENCY_HZ / FFT_SAMPLES;
    }

    return calculated_freq;
}

/**
  * @brief  Formatea y muestra todos los datos en la pantalla LCD.
  */
void refresh_display_data(void) {
    char line_buffer[21]; // Buffer para una línea completa (20 chars + null)
    char text_buffer[21];

    // Convertir valores RAW a m/s^2
    const float G_TO_MS2 = 9.80665f;
    const float SENSITIVITY = 16384.0f;
    float accel_x_ms2 = main_sensor.accel_x / SENSITIVITY * G_TO_MS2;
    float accel_y_ms2 = main_sensor.accel_y / SENSITIVITY * G_TO_MS2;
    float accel_z_ms2 = main_sensor.accel_z / SENSITIVITY * G_TO_MS2;

    // --- Fila 0: Eje X ---
    snprintf(text_buffer, sizeof(text_buffer), "X:%+6.2fm/s^2", accel_x_ms2);
    snprintf(line_buffer, sizeof(line_buffer), "%-20s", text_buffer); // Rellenar con espacios
    display_set_cursor(&main_display, 0, 0);
    display_write_string(&main_display, line_buffer);

    // --- Fila 1: Eje Y ---
    snprintf(text_buffer, sizeof(text_buffer), "Y:%+6.2fm/s^2", accel_y_ms2);
    snprintf(line_buffer, sizeof(line_buffer), "%-20s", text_buffer);
    display_set_cursor(&main_display, 1, 0);
    display_write_string(&main_display, line_buffer);

    // --- Fila 2: Eje Z ---
    snprintf(text_buffer, sizeof(text_buffer), "Z:%+6.2fm/s^2", accel_z_ms2);
    snprintf(line_buffer, sizeof(line_buffer), "%-20s", text_buffer);
    display_set_cursor(&main_display, 2, 0);
    display_write_string(&main_display, line_buffer);

    // --- Fila 3: Frecuencia Dominante ---
    snprintf(text_buffer, sizeof(text_buffer), "Freq: %.1f Hz", dominant_frequency);
    snprintf(line_buffer, sizeof(line_buffer), "%-20s", text_buffer);
    display_set_cursor(&main_display, 3, 0);
    display_write_string(&main_display, line_buffer);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		HAL_GPIO_TogglePin(pinH1Led2Board_GPIO_Port, pinH1Led2Board_Pin);
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Asegurarse de que la interrupción vino del pin del MPU6050 (PA4)
    if (GPIO_Pin == mpu_EXTI_Pin) { // MPU_INT_Pin debe estar definido como GPIO_PIN_4
        new_sensor_data_available = true; // Activar el estado/bandera para el main loop
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
