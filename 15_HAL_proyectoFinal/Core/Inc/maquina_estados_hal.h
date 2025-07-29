/*
 * maquina_estados_hal.h
 *
 *  Created on: Jul 10, 2025
 *      Author: santiago
 */

#ifndef INC_MAQUINA_ESTADOS_HAL_H_
#define INC_MAQUINA_ESTADOS_HAL_H_
typedef enum
{
	IDLE,
	CIRCULO,
	LINEA_PARALELA_AB,
	PROCESANDO_COMANDO
	//DISPLAY,
} e_PosibleStates;

typedef struct
{
	e_PosibleStates state;

} fsm_states_t;


#endif /* INC_MAQUINA_ESTADOS_HAL_H_ */
