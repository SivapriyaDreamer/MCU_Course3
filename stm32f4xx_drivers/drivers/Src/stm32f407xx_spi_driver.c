/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 08-Jun-2023
 *      Author: SivapriyaK
 */

#include "stm32f4xx_spi_driver.h"




/*
 * GPIO Init and Deinit
 */


/* ****************************************************
 * @ func - SPI_PeriClockControl
 *
 * @ brief - This function Initializes the clock for the SPI peripheral
 *
 * @ param [in] -
 * @ param [in] -
 * @ param [in] -
 *
 * @ return -
 *
 * @ note -  none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
		}else
		{
			if(pSPIx == SPI1)
					{
						SPI1_PCLK_DI();
					}else if(pSPIx == SPI2)
					{
						SPI2_PCLK_DI();
					}else if(pSPIx == SPI3)
					{
						SPI3_PCLK_DI();
					}
		}
}



/* ****************************************************
 * @ func - SPI_Init
 *
 * @ brief - This function Initializes the GPIO port
 *
 * @ param [in] -
 * @ param [in] -
 * @ param [in] -
 *
 * @ return -
 *
 * @ note -  none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

	uint32_t tempreg = 0;
	// configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//configurepSPIHandle the bus configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD )
	{
		// bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RX-only bit should be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//configure the spi serial clock speed ( baud rate )
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;


	//6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7. configure the SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;

}




void SPI_DeInit(SPI_RegDef_t *pSPIx)  //resetting the peripheral reset register
{
	if(pSPIx == SPI1)
				{
					SPI1_REG_RESET();
				}else if(pSPIx == SPI2)
				{
					SPI2_REG_RESET();
				}else if(pSPIx == SPI3)
				{
					SPI3_REG_RESET();
				}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}




/* ****************************************************
 * @ func - GPIO_Init
 *
 * @ brief - This function Initializes the GPIO port
 *
 * @ param [in] -
 * @ param [in] -
 * @ param [in] -
 *
 * @ return -
 *
 * @ note -  none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//while(!(pSPIx->SR & (1 << 1))); // till txe is set we will hang here

		// 2. check the DFF bit in CR1
		if( (pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit data
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit data
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}


}




void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~( 1 << SPI_CR2_SSOE);
	}
}



void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//wait until RXNE is set
			while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);

			//while(!(pSPIx->SR & (1 << 1))); // till txe is set we will hang here

			// 2. check the DFF bit in CR1
			if( (pSPIx->CR1 & (1 << SPI_CR1_DFF)))
			{
				//16 bit data
				//load the data from the Rx buffer
				 *((uint16_t*)pRxBuffer) = pSPIx->DR ;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}else
			{
				//8 bit data
				*(pRxBuffer)= pSPIx->DR;
				Len--;
				pRxBuffer++;
			}

		}

}

/*
 * GPIO Init and Deinit
 */


/* ****************************************************
 * @ func - GPIO_Init
 *
 * @ brief - This function Initializes the GPIO port
 *
 * @ param [in] -
 * @ param [in] -
 * @ param [in] -
 *
 * @ return -
 *
 * @ note -  none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
