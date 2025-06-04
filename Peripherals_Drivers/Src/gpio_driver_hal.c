/*
 * gpio_driver_hal.c
 *
 *  Created on: Jun 4, 2025
 *      Author: santiago
 */


#include "stm32_assert.h"
#include "stm32f4xx.h"
#include"gpio_driver_hal.h"


// HEADERS FOR PRIVATE FUNCTIONS


static void gpio_enable_clock_peripheral(GPIO_Handler_t *pGPIOHandler);
static void gpio_config_mode(GPIO_Handler_t *pGPIOHandler);
static void gpio_config_output_type(GPIO_Handler_t *pGPIOHandler);
static void gpio_config_output_speed(GPIO_Handler_t *pGPIOHandler);
static void gpio_config_pullup_pulldown(GPIO_Handler_t *pGPIOHandler);
static void gpio_config_alternate_function(GPIO_Handler_t *pGPIOHandler);


void gpio_Config (GPIO_Handler_t *pGPIOHandler){


	//Verificamos que el PIN seleccionado es correcto
	assert_param(IS_GPIO_PIN(pGPIOHandler->pinConfig.GPIO_PinNumber));

	// 1) Activar el periférico
	gpio_enable_clock_peripheral(pGPIOHandler);


	//despues de activado, podemos comenzar a confiurar.


	// 2)configurando el registro GPIOx_MODER
	gpio_config_mode(pGPIOHandler);

	// 3) configurando el registro GPIOx_OTYPER
	gpio_config_output_type(pGPIOHandler);

	// 4) configurando ahora la velocidad
	gpio_config_output_speed(pGPIOHandler);

	// 5) configurando si se desea pull up pull down o flotante
	gpio_config_pullup_pulldown(pGPIOHandler);

	// 6) Configuracion de funciones alternativas
	gpio_config_alternate_function(pGPIOHandler);

}

//Enable clock signal for specife gPIOx port


void gpio_enable_clock_peripheral(GPIO_Handler_t *pGPIOHandler){

	//Verificamos que el puerto configurado si es permitido

	assert_param(IS_GPIO_ALL_INSTANCE(pGPIOHandler->pGPIOx));




		// Verificamos para GPIOA
		if(pGPIOHandler->pGPIOx == GPIOA){

		//Escribimos 1 (SET) en la posicion correspondiente al GPIOA
		RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN_Pos);

	}

	// Verificamos para GPIOB

	else if(pGPIOHandler->pGPIOx == GPIOB){

	// Escribimos 1 (SET) en la posicion correspondiente al GPIOB

	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOBEN_Pos);

	}

	// Verificamos para GPIOC

	else if(pGPIOHandler->pGPIOx == GPIOC){

	// Escribimos 1 (SET) en la posicion correspondiente al GPIOC

	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOCEN_Pos);

	}

	// Verificamos para GPIOD

	else if(pGPIOHandler->pGPIOx == GPIOD){

	// Escribimos 1 (SET) en la posicion correspondiente al GPIOD

	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIODEN_Pos);

	}

	// Verificamos para GPIOE

	else if(pGPIOHandler->pGPIOx == GPIOE){

	// Escribimos 1 (SET) en la posicion correspondiente al GPIOE

	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOEEN_Pos);

	}

	// Verificamos para GPIOH

	else if(pGPIOHandler->pGPIOx == GPIOH){

	// Escribimos 1 (SET) en la posicion correspondiente al GPIOH

	//cambio cuando hacia el update basic
	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOHEN_Pos);
	}

}



/*CONFIGURES THE MODE IN WHICH THE PIN WILL WORK

* - Input

* - Output

* - Analog

* - Alternate Function

* */

void gpio_config_mode(GPIO_Handler_t *pGPIOHandler){

	uint32_t auxConfig = 0;

	// uint32_t limpiadorMODER = 0;


	//Verificamos si el modo que se ha seleccionado es permitido

	assert_param(IS_GPIO_MODE(pGPIOHandler->pinConfig.GPIO_PinMode));


	//Aca estamos leyendo la config moviendo "pinnumber" veces hacia la izquierda ese valor (shift left)

	//y to_do eso lo cargamos en la variable auxConfig

	auxConfig = (pGPIOHandler->pinConfig.GPIO_PinMode << 2 * pGPIOHandler->pinConfig.GPIO_PinNumber);


	//antes de cargar el nuevo valor, limpiamos los bits especificos de ese registro (debemos escribir 0b00)

	//Para lo cual aplicamos una mascara y una operacion bitwise AND

	//No está limpiando los registros de MODER13, MODER14 y MODER15.

	//No es un problema, como GPIOA empieza en 0xA8000000, entonces esos espacios deben estar asi.

	//Los demas empiezan en 0x00000000


	pGPIOHandler->pGPIOx->MODER &= ~(0b11 << 2 * pGPIOHandler->pinConfig.GPIO_PinNumber);


	//CArgamos a auxConfig en el registro MODER

	pGPIOHandler->pGPIOx->MODER |= auxConfig;

}

/* CONFIGURES WHICH TYPE OF OUTPUT THE PINX WILL USE:

* -Push-Pull

* -openDrain

* */

void gpio_config_output_type(GPIO_Handler_t *pGPIOHandler){


	uint32_t auxConfig = 0;


	// Verificamos que el tipo de salida corresponda a los que se pueden utilizar

	assert_param(IS_GPIO_OUTPUT_TYPE(pGPIOHandler->pinConfig.GPIO_PinOutputType));


	//De nuevo, leemos y movemos el valor de un numero "pinNumber" de veces

	auxConfig = (pGPIOHandler->pinConfig.GPIO_PinOutputType << pGPIOHandler->pinConfig.GPIO_PinNumber);


	//Limpiamos antes de cargar

	pGPIOHandler->pGPIOx->OTYPER &= ~(SET << pGPIOHandler->pinConfig.GPIO_PinNumber);


	//Garcamos el resultado sobre el registro adecuado

	pGPIOHandler->pGPIOx->OTYPER |= auxConfig;

}


/* SELECTS BETWEEN FOUR DIFERRENTS POSSIBLE SPEEDS FOR OUTPUT PINX

* - LOW

* - MEDIUM

* - FAST

* - HIGHSPEED

* */

void gpio_config_output_speed(GPIO_Handler_t *pGPIOHandler){


	uint32_t auxConfig = 0;


	// Verificamos que el tipo de salida corresponda a los que se pueden utilizar

	assert_param(IS_GPIO_OSPEED(pGPIOHandler->pinConfig.GPIO_PinOutputSpeed));


	//De nuevo, leemos y movemos el valor de un numero "pinNumber" de veces

	auxConfig = (pGPIOHandler->pinConfig.GPIO_PinOutputSpeed << 2 * pGPIOHandler->pinConfig.GPIO_PinNumber);


	//Limpiando la posicion antes de cargar la nueva configuracion

	pGPIOHandler->pGPIOx->OSPEEDR &= ~(0b11 << 2 * pGPIOHandler->pinConfig.GPIO_PinNumber);


	//Cargamos el resultado sobre el registro adecuado

	pGPIOHandler->pGPIOx->OSPEEDR |= auxConfig;

}



/* TURNS ON/OFF THE PULL-UP AND PULL-DOWN RESISTER FOR EACH PINX IN SELECTED GPIO PORT */

void gpio_config_pullup_pulldown(GPIO_Handler_t *pGPIOHandler){


	uint32_t auxConfig = 0;


	// Verificamos que el tipo de salida corresponda a los que se pueden utilizar

	assert_param(IS_GPIO_PUPDR(pGPIOHandler->pinConfig.GPIO_PinPupdControl));


	//Aca estamos leyendo la config moviendo "pinnumber" veces hacia la izquierda ese valor (shift left)

	//y to_do eso lo cargamos en la variable auxConfig

	auxConfig = (pGPIOHandler->pinConfig.GPIO_PinPuPdControl << 2 * pGPIOHandler->pinConfig.GPIO_PinNumber);


	//Limpiando la posicion antes de cargar la nueva configuracion

	pGPIOHandler->pGPIOx->PUPDR &= ~(0b11 << 2 * pGPIOHandler->pinConfig.GPIO_PinNumber);


	//Cargamos el resultado sobre el registro adecuado

	pGPIOHandler->pGPIOx->PUPDR |= auxConfig;


}



/* ALLOWS TO CONFIGURE OTHER FUNCTION (MORE SPECIALIZED) OB THE SELECTED PINX*/

void gpio_config_alternate_function(GPIO_Handler_t *pGPIOHandler){


	uint32_t auxPosition = 0;


	//esta es diferente


	//seleccionamos primero si se debe utilizar el registro bajo (AFRL) o el alto (AFRH)

	if(pGPIOHandler->pinConfig.GPIO_PinNumber < 8){

	//Estamos en el registo AFRL, que controla los pines del PIN 0 al PIN 7

	auxPosition = 4 * pGPIOHandler->pinConfig.GPIO_PinNumber;


	//Limpiamos la posicion del registro que deseamos escribir a continuacion

	pGPIOHandler->pGPIOx->AFR[0] &= ~(0b1111 << auxPosition);


	//Escribimos el valor configurado en la posicion seleccionada

	pGPIOHandler->pGPIOx->AFR[0] |= (pGPIOHandler->pinConfig.GPIO_PinAltFunMode << auxPosition);

	}

	else{

	//Estamos en el registro AFRH, que controla los pines PIN_8 al PIN_15

	auxPosition = 4 * (pGPIOHandler->pinConfig.GPIO_PinNumber -8);


	//Limpiamos la posicion del registro que deseamos escribir a continuacion

	pGPIOHandler->pGPIOx->AFR[1] &= ~(0b1111 << auxPosition);


	//Escribimos el valor configurado en la posicion seleccionada

	pGPIOHandler->pGPIOx->AFR[1] |= (pGPIOHandler->pinConfig.GPIO_PinAltFunMode << auxPosition);


	}

}


/* FUNCION UTILIZADA PARA CAMBIAR EL ESTADO DE UN PIN ESTREGADO EN EL HANDLER, ASIGNADO EL VALOR ENTREGADO

* EN LA VARIABLE newState

* y el parámetro newState define si el pin debe ponerse en alto (SET) o en bajo (RESET).

* */


void gpio_WritePin(GPIO_Handler_t *pPinHandler, uint8_t newState){


	//Verificamos si la acciones que se quiere hacer esta permitida

	assert_param(IS_GPIO_PIN_ACTION(newState));


	//Limpiamos la posicion que deseamos

	// pPinHandler->pGPIOx->ODR &= ~(SET << pPinHandler->pinConfig.GPIO_PinNumber);

	/*Conclusión

	El uso de |= es erróneo porque BSRR no requiere combinar el valor antiguo con el nuevo.

	El diseño de BSRR permite operaciones atómicas y seguras con una sola escritura.

	Este ajuste hace que la función sea más eficiente y correcta.


	* */


	if(newState== SET){

	//Trabajndo con la parte baja del registro ( la parte de BSx)

	pPinHandler->pGPIOx->BSRR = (SET << pPinHandler->pinConfig.GPIO_PinNumber);

	}

	else{

	//Trabajando con la parte alta del registro ( la parte de BRx)

	pPinHandler->pGPIOx->BSRR = (SET << (pPinHandler->pinConfig.GPIO_PinNumber + 16));

	}

}


/*FUNCION PARA LEER EL ESTADO DEL PIN ESPECIFICO

*

* Ejemplo

Si el pin 3 del puerto GPIOA está en alto (1):


GPIOA->IDR: 0b00001000.

Desplazamos IDR >> 3: 0b00000001.

& 0x1: devuelve 1.

*/

uint32_t gpio_ReadPin(GPIO_Handler_t *pPinHandler){

	//Creamos una variable auxiliar la cual luego retornaremos

	uint32_t pinValue = 0;

	pinValue = (pPinHandler->pGPIOx->IDR >> pPinHandler->pinConfig.GPIO_PinNumber) & 0x01;;


	return pinValue;

}





void gpio_TogglePin(GPIO_Handler_t *pPinHandler){

	// Alternar el estado del pin (ON <-> OFF)


	//Asi estaba antes:

	// pPinHandler->pGPIOx->ODR ^= (1 << pPinHandler->pinConfig.GPIO_PinNumber);


	/* De esta manera se tiene un 1 un la posición del número del pin. */

	uint32_t pinMask = (1 << pPinHandler->pinConfig.GPIO_PinNumber);


	/*Si ODR está en 1 para un correspondiente pin, es porque hay voltaje de salida (Led prendido por ejemplo).

	* Recordar que en el registro BSRR de 32 bit, del BIT 0 al 15 son para BSy (B SET), y del BIT 16 al 31 son

	* para el BRy (B RESET)

	* */

	if (pPinHandler->pGPIOx->ODR & pinMask) {


	pPinHandler->pGPIOx->BSRR = pinMask << 16; // Reset bit


	} else {


	pPinHandler->pGPIOx->BSRR = pinMask; // Set bit

	}

}







