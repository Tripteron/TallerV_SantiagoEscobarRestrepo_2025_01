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
/* Función en la que cargamos la configuración del Timer *
 * Recordar que siempre se debe comenzar con activar la señal de reloj * del periférico que se está utilizando.
 * * Además, en este caso, debemos ser cuidadosos al momento de utilizar las interrupciones.
 * * Los Timer están conectados directamente al elemento NVIC del Cortex-Mx * Debemos configurar y/o utilizar:
 * * - TIMx_CR1 (control Register 1) * - TIMx_SMCR (slave mode control register) -> mantener en 0 para modo Timer Básico
 * * - TIMx_DIER (DMA and Interrupt enable register)
 * * - TIMx_SR (Status register)
 *  * - TIMx_CNT (Counter)
 *  * - TIMx_PSC (Pre-scaler)
 * * - TIMx_ARR (Auto-reload register) * *
 *  Como vamos a trabajar con interrupciones, antes de configurar una nueva, debemos desactivar
 *  * el sistema global de interrupciones, activar la IRQ específica y luego volver a encender * el sistema. */

void timer_Config(Timer_Handler_t *pTimerHandler) { // Guardamos una referencia al periférico que estamos utilizando... ptrTimerUsed = pTimerHandler->pTIMx;
