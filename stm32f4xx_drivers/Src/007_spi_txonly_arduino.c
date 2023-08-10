/*
 * 006Spi_tx_testing.c
 *
 *  Created on: 08-Jun-2023
 *      Author: SivapriyaK
 */

#include "stm32f407xx.h"
#include<string.h>

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdcontrol = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // generates clk of 2 MHz
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;  // Hw slave management enabled

    SPI_Init(&SPI2Handle);

}

void delay(void)
{
	for(uint32_t i=0;i<500000/2;i++);
}

void GPIO_Btn_Init(void)
{
	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdcontrol = GPIO_NO_PUPD;



	GPIO_Init(&GpioBtn);
}


int main(void)
{
	char user_data[]= "Hello world";
	GPIO_Btn_Init();

	SPI2_GPIOInits();

	SPI2_Inits();

	while(1)
	{
	/*
	 * Making SSOE 1 does NSS output enable
	 * The NSS pin is automatically managed by the hardware
	 * i.e. when SPE=1, NSS will be pulled to low
	 * and NSS pin will be high when SPE = 0
	 */
	SPI_SSOEConfig(SPI2,ENABLE);
	//enable the SPE SPI peripheral enable
	while( !GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

	delay();

	SPI_PeripheralControl(SPI2,ENABLE);

	// first send length information
	uint8_t dataLen = strlen(user_data);
	SPI_SendData(SPI2, &dataLen, 1);


	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));
	// lets confirm SPI is not busy



	while(SPI_GetFlagStatus(SPI2,SPI_SR_BSY));

	SPI_PeripheralControl(SPI2,DISABLE);
	}
	return 0;
}
