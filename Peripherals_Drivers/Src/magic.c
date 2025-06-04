/*
 * magic.c
 *
 *  Created on: Oct 31, 2024
 *      Author: namontoy
 */
#include <stdint.h>
#include <stdio.h>
#include <stm32f4xx.h>


void configMagic(void)
{
	setvbuf(stdin, NULL, _IONBF, 0);
	SCB->CPACR |= (0xF << 20);
	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);
	GPIOA->MODER &= ~GPIO_MODER_MODER2;
	GPIOA->MODER |= (0x2 << GPIO_MODER_MODER2_Pos);
	GPIOA->MODER &= ~GPIO_MODER_MODER3;
	GPIOA->MODER |= (0x2 << GPIO_MODER_MODER3_Pos);
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT2;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR2;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2;
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR3;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD2;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD3;
	GPIOA->AFR[0] &= ~(0xF << 8);
	GPIOA->AFR[0] |= (0x7 << 8);
	GPIOA->AFR[0] &= ~(0xF << 12);
	GPIOA->AFR[0] |= (0x7 << 12);
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	USART2->CR1 &= ~USART_CR1_PCE;
	USART2->CR1 &= ~USART_CR1_M;
	USART2->CR2 &= ~USART_CR2_STOP;
	USART2->BRR = 0x008B;
	USART2->CR1 &= ~USART_CR1_RE;
	USART2->CR1 |= USART_CR1_RE;
	USART2->CR1 &= ~USART_CR1_TE;
	USART2->CR1 |= USART_CR1_TE;
	USART2->CR1 |= USART_CR1_UE;

	for(int i = 0; i < 30000; i++){
		__NOP();
	}

	char clear[5] = { 0 };
	clear[0] = 0x1B;
	clear[1] = 0x5B;
	clear[2] = 0x32;
	clear[3] = 0x4A;
	printf(&clear[0]);
}


int __io_putchar(int ch)
{
	while (!(USART2->SR & USART_SR_TXE)) {
		__NOP();
	}
	USART2->DR = ch;
	return ch;
}

int __io_getchar(void){
	uint8_t ch = 0;
	USART2->SR &= ~USART_SR_ORE_Msk;
	while (!(USART2->SR & USART_SR_RXNE)) {
		__NOP();
	}
	ch = USART2->DR;
	return ch;
}
