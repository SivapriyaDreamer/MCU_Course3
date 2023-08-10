/*
 * 002LedButton.c
 *
 *  Created on: 03-Jun-2023
 *      Author: SivapriyaK
 */


#include "stm32f407xx.h"
#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW
void delay(void)
{
	for(uint32_t i=0;i<500000/2;i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioBtn;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdcontrol = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);


	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdcontrol = GPIO_NO_PUPD;


		GPIO_PeriClockControl(GPIOC, ENABLE);
		GPIO_Init(&GpioBtn);
	while(1)
	{
      delay();
	 if(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_8) == BTN_PRESSED)
	 {
		 delay();
		 GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_8);

	 }
	}
	return 0;

}
