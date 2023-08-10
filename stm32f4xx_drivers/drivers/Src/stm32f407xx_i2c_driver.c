/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 24-Jun-2023
 *      Author: SivapriyaK
 */


#include "stm32f407xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;  // making space for the r/nw bit
	//clear the 0th bit of SlaveAddr, the lsb bit 0 will correspond to write
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr;
}

// helper function which is private to the C file
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);

}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyread = pI2Cx->SR1;
	dummyread = pI2Cx->SR2;
	(void)dummyread; // to avoid throwing unused variable error


}



uint32_t RCC_GetPLLOutputClk(void)
{
 return 0;

}


void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
			{
				if(pI2Cx == I2C1)
				{
					I2C1_PCLK_EN();
				}else if(pI2Cx == I2C2)
				{
					I2C2_PCLK_EN();
				}else if(pI2Cx == I2C3)
				{
					I2C3_PCLK_EN();
				}
			}else
			{
				if(pI2Cx == I2C1)
						{
					I2C1_PCLK_EN();
						}else if(pI2Cx == I2C2)
						{
							I2C2_PCLK_EN();
						}else if(pI2Cx == I2C3)
						{
							I2C3_PCLK_EN();
						}
			}
}

/*
 * Getting the frequency of PCLK
 */

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;
	// finding the value of the bit 3:2 in the CCR register
	// to find which clock source i sbeing used

	uint8_t clksrc,temp, ahbp, apb1p;
	clksrc = (RCC->CFGR >> 2) & 0x3;

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if(clksrc ==2)
	{
		SystemClk = RCC_GetPLLOutputClk();
	}

	// to find out the AHB1 PreScaler
	temp = (RCC->CFGR >> 4) & 0xF;

	if( temp < 8 )
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}

    // to find out the APB1 prescaler
	temp = (RCC->CFGR >> 10) & 0x7;

	if( temp < 4 )
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = ((SystemClk/ ahbp) / apb1p );


	return pclk1;
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;
	// enable the clk for the I2C peripheral

	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);
	// Ack ocntrol bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// initialising the freq field
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000U; // dividing to get only the number in MHz
	pI2CHandle->pI2Cx->CR2 = tempreg & 0x3F ;

	// program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14 );
	pI2CHandle->pI2Cx->OAR1 = tempreg & 0x7F;

	// CCR Calculations
	uint16_t ccr_value = 0;
	tempreg =0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//standard mode
		ccr_value = RCC_GetPCLK1Value() / (2* pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= ( ccr_value & 0xFFF);

	}else
	{
		//fast mode
		tempreg |= ( 1 << 15 );
		tempreg |= ( pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (3* pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}else
		{
			ccr_value = RCC_GetPCLK1Value() / (25* pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= ( ccr_value & 0xFFF);

	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Calculation
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//SM

		tempreg = (RCC_GetPCLK1Value()/1000000U) + 1; //1000ns is 1 us which is 1 MHz

	}
	else{
		//FM
		tempreg = ((RCC_GetPCLK1Value()*300)/1000000000U)+1; // for FM 300 ns

	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);




}



void I2C_DeInit(I2C_RegDef_t *pI2Cx)  //resetting the peripheral reset register
{



}
/*
 * Data Send and Receive
 */


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// 1. Generate the START Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm hat the START generation is completed by checking the SB flag in the SR1
	//Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)));

	//3. Send the address of the slave with r/nw bit set to w(0) ( 8 bits )
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);


	//4.Confirm that the address phase is completed by checking the ADDR flag in the SR1 register
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)));

	//5. clear the ADDR flag according to the software sequence
	//Note: until ADDR will be cleared SCL will be stretched
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. send the data until Len becomes 0
	while(Len!=0)
	{
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;


	}


	//7. when Len becomes 0 wait for the TXE=1, BTF=1 before generating the STOP condition
	// Note: TXE=1, BTF=1 means that both SR and DR is empty and next transmission should begin
	//when BTF=1, SCL will be stretched ( pulled to LOW )
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)));
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF)));


	//8. Generate STOP condition and master need not wait for the completion of stop condition
	//Note: generating STOP automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

/*
 * IRQ COnfiguration and ISR Handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{


}
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{


}

/*
 * Other peripheral APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		}else
		{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
		}


}


/*
 * Application Callback
 *
 */

void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t AppEv );

