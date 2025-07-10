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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define pinH1Led2Board_Pin GPIO_PIN_1
#define pinH1Led2Board_GPIO_Port GPIOH
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define pinDigit2_Pin GPIO_PIN_5
#define pinDigit2_GPIO_Port GPIOC
#define pinEncoderDT_Pin GPIO_PIN_10
#define pinEncoderDT_GPIO_Port GPIOB
#define pinSegmentA_Pin GPIO_PIN_12
#define pinSegmentA_GPIO_Port GPIOB
#define green_rgb_PWM_Pin GPIO_PIN_13
#define green_rgb_PWM_GPIO_Port GPIOB
#define red_rgb_PWM_Pin GPIO_PIN_14
#define red_rgb_PWM_GPIO_Port GPIOB
#define blue_rgb_PWM_Pin GPIO_PIN_15
#define blue_rgb_PWM_GPIO_Port GPIOB
#define pinDigit1_Pin GPIO_PIN_6
#define pinDigit1_GPIO_Port GPIOC
#define pinSegmentE_Pin GPIO_PIN_8
#define pinSegmentE_GPIO_Port GPIOC
#define pinSegmentD_Pin GPIO_PIN_9
#define pinSegmentD_GPIO_Port GPIOC
#define pinEncoderSW_Pin GPIO_PIN_8
#define pinEncoderSW_GPIO_Port GPIOA
#define pinEncoderSW_EXTI_IRQn EXTI9_5_IRQn
#define pinDigit3_Pin GPIO_PIN_11
#define pinDigit3_GPIO_Port GPIOA
#define pinDigit4_Pin GPIO_PIN_12
#define pinDigit4_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define pinSegmentF_Pin GPIO_PIN_10
#define pinSegmentF_GPIO_Port GPIOC
#define pinSegmentG_Pin GPIO_PIN_11
#define pinSegmentG_GPIO_Port GPIOC
#define pinSegmentC_Pin GPIO_PIN_12
#define pinSegmentC_GPIO_Port GPIOC
#define pinSegmentB_Pin GPIO_PIN_2
#define pinSegmentB_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define pinEncoderCLK_Pin GPIO_PIN_5
#define pinEncoderCLK_GPIO_Port GPIOB
#define pinEncoderCLK_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
