/*
 * maquina_estados_hal.h
 *
 *  Created on: Jun 4, 2025
 *      Author: santiago
 */

#ifndef MAQUINA_ESTADOS_HAL_H_
#define MAQUINA_ESTADOS_HAL_H_

#include <stdint.h>
#include "stm32f4xx.h"
typedef enum
{
	IDLE,
	ROTACION,
	BOTON_SW,
} e_PosibleStates;

typedef struct
{
	uint32_t accessCounter;
	e_PosibleStates;

} fsm_states_t;

extern void configMagic(void);


#endif /* MAQUINA_ESTADOS_HAL_H_ */
