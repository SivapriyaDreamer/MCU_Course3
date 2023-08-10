/*
 * main.c
 *
 *  Created on: 06-Jun-2023
 *      Author: SivapriyaK
 */

#include "stm32f407xx.h"
int main(void)
{

return 0;
}

void EXTI0_IRQHandler(void)
{
	//handle the interrupt
	GPIO_IRQHandling(0);
}
