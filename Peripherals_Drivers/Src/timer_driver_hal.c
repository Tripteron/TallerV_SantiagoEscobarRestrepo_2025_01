/*
 * timer_driver_hal.c
 *
 *  Created on: Jun 4, 2025
 *      Author: santiago
 */
#include "stm32f4xx.h"
#include "timer_driver_hal.h"
#include "stm32_assert.h"

/* Variable que guarda la referencia del periférico que se está utilizando */

TIM_TypeDef *ptrTimerUsed;

/* === Headers for private functions === */
static void timer_enable_clock_peripheral(Timer_Handler_t *pTimerHandler);
static void timer_set_prescaler(Timer_Handler_t *pTimerHandler);
static void timer_set_period(Timer_Handler_t *pTimerHandler);
static void timer_set_mode(Timer_Handler_t *pTimerHandler);
static void timer_config_interrupt(Timer_Handler_t *pTimerHandler);
