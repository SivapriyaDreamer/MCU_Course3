/*
 * stm32f407xx.h
 *
 *  Created on: Jun 1, 2023
 *      Author: SivapriyaK
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>
#define __vo volatile

/**************************************Processor Specific Details********************************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx Register Addresses
 */
#define NVIC_ISER0          ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1          ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2          ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3          ((__vo uint32_t*)0xE000E10C)
#define NVIC_ISER4          ((__vo uint32_t*)0xE000E110)
#define NVIC_ISER5          ((__vo uint32_t*)0xE000E114)
#define NVIC_ISER6          ((__vo uint32_t*)0xE000E118)
#define NVIC_ISER7          ((__vo uint32_t*)0xE000E11C)


/*
 * ARM Cortex Mx Processor NVIC ICERx Register Addresses
 */
#define NVIC_ICER0          ((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1          ((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2          ((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3          ((__vo uint32_t*)0XE000E18C)


/*
 * NVIC Priority Register Address Configuration
 */
#define NVIC_PR_BASEADDR      ((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 4

#define FLASH_BASEADDR      0x08000000U
#define SRAM1_BASEADDR      0x20000000U
#define SRAM2_BASEADDR      0x2001C000U
#define SRAM                SRAM1_BASEADDR

/* Defining the peripheral base address */
#define PERIPH_BASE         0x40000000U
#define APB1PERIPH_BASE 	PERIPH_BASE
#define APB2PERIPH_BASE 	0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U

/* Base address of all the peripherals hanging on AHB1 */


#define GPIOA_BASEADDR      (AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASEADDR      (AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR      (AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR      (AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR      (AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASEADDR      (AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASEADDR      (AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASEADDR      (AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASEADDR      (AHB1PERIPH_BASE + 0x2000U)
#define GPIOJ_BASEADDR      (AHB1PERIPH_BASE + 0x2400U)
#define GPIOK_BASEADDR      (AHB1PERIPH_BASE + 0x2800U)
#define RCC_BASEADDR        (AHB1PERIPH_BASE + 0x3800U)


/* Base address of peripherals hanging on APB1 bus*/

#define  I2C1_BASEADDR      (APB1PERIPH_BASE + 0x5400U)
#define  I2C2_BASEADDR      (APB1PERIPH_BASE + 0x5800U)
#define  I2C3_BASEADDR      (APB1PERIPH_BASE + 0x5C00U)
#define  USART3_BASEADDR      (APB1PERIPH_BASE + 0x4800U)
#define  USART2_BASEADDR      (APB1PERIPH_BASE + 0x4400U)
#define  UART4_BASEADDR      (APB1PERIPH_BASE + 0x4C00U)
#define  UART5_BASEADDR      (APB1PERIPH_BASE + 0x5000U)
#define  SPI2_BASEADDR      (APB1PERIPH_BASE + 0x3800U)
#define  SPI3_BASEADDR      (APB1PERIPH_BASE + 0x3C00U)

/* Base address of peripherals hanging on APB2 bus*/

#define SPI1_BASEADDR         (APB2PERIPH_BASE + 0x3000U)
#define USART1_BASEADDR       (APB2PERIPH_BASE + 0x1000U)
#define USART6_BASEADDR       (APB2PERIPH_BASE + 0x1400U)
#define EXTI_BASEADDR         (APB2PERIPH_BASE + 0x3C00U)
#define SYSCFG_BASEADDR       (APB2PERIPH_BASE + 0x3800U)




#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET
#define FLAG_RESET		RESET
#define FLAG_SET 		SET
/* Peripheral Register Definition Address  */

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];


}GPIO_RegDef_t;


/* Peripheral Register Definition Address for EXTI */
typedef struct
{
	__vo uint32_t IMR;                                /*|< Address offset : 0x00 >|*/
	__vo uint32_t EMR;                               /*|< Address offset : 0x04 >|*/
	__vo uint32_t RTSR;                               /*|< Address offset : 0x08 >|*/
	__vo uint32_t FTSR;                               /*|< Address offset : 0x0C >|*/
	__vo uint32_t SWIER;                               /*|< Address offset : 0x10 >|*/
	__vo uint32_t PR;                               /*|< Address offset : 0x14 >|*/

}EXTI_RegDef_t;

/* Peripheral Register Definition Address for SYSCFG */
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;

}SYSCFG_RegDef_t;


/*
 * Peripheral Register Definition for SPI Base address
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;

}I2C_RegDef_t;


/* Peripheral Register Definition Address for RCC Registers */
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;


}RCC_RegDef_t;


/* Typecasting the peripherals to the strcuture pointer*/

#define GPIOA  ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  ((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define EXTI   ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define RCC   ((RCC_RegDef_t*)RCC_BASEADDR)
#define SYSCFG  ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
#define SPI1 ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1 ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t*)I2C3_BASEADDR)


/* Defining structure definitions for the RCC */


/* Clock enable macros for the GPIO peripherals */

#define GPIOA_PCLK_EN()    (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()    (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()    (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()    (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()    (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()    (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()    (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()    (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()    (RCC->AHB1ENR |= (1 << 8))

/* Enabling Clock Macros for the I2C peripherals */

#define I2C1_PCLK_EN()     (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()     (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()     (RCC->APB1ENR |= (1 << 23))

/* Enabling Clock Macros for the SPI peripherals*/

#define SPI1_PCLK_EN()     (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()     (RCC->APB1ENR |= ( 1 << 14))
#define SPI3_PCLK_EN()     (RCC->APB1ENR |= ( 1 << 15))

/* Enabling Clock Macros for the usart peripherals */

#define USART2_PCLK_EN()   (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()   (RCC->APB1ENR |= (1 << 18))
#define USART1_PCLK_EN()   (RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN()   (RCC->APB2ENR |= (1 << 5))


/* Enabling the clock macros for the syscfg peripheral
 *
 *
 */

#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |= (1 << 14))


/*
 * Macros to disable the peripherals
 */
#define GPIOA_REG_RESET() do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0);}while(0)
#define GPIOB_REG_RESET() do{ RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1);}while(0)
#define GPIOC_REG_RESET() do{ RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2);}while(0)
#define GPIOD_REG_RESET() do{ RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3);}while(0)
#define GPIOE_REG_RESET() do{ RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4);}while(0)
#define GPIOF_REG_RESET() do{ RCC->AHB1RSTR |= (1 << 5); RCC->AHB1RSTR &= ~(1 << 5);}while(0)
#define GPIOG_REG_RESET() do{ RCC->AHB1RSTR |= (1 << 6); RCC->AHB1RSTR &= ~(1 << 6);}while(0)
#define GPIOH_REG_RESET() do{ RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7);}while(0)
#define GPIOI_REG_RESET() do{ RCC->AHB1RSTR |= (1 << 8); RCC->AHB1RSTR &= ~(1 << 8);}while(0)
#define SPI1_REG_RESET()  do{RCC->APB2RSTR |= (1 << 12); RCC->APB2RSTR &= ~(1 << 12);}while(0)
#define SPI2_REG_RESET()  do{RCC->APB1RSTR |= (1 << 14); RCC->APB1RSTR &= ~(1 << 14);}while(0)
#define SPI3_REG_RESET()  do{RCC->APB1RSTR |= (1 << 15); RCC->APB1RSTR &= ~(1 << 15);}while(0)
/*
 * Returns port code for the given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)     ((x == GPIOA) ? 0 :\
									 (x == GPIOB) ? 1 :\
									 (x == GPIOC) ? 2 :\
								     (x == GPIOD) ? 3 :\
								     (x == GPIOE) ? 4 :\
								     (x == GPIOF) ? 5 :\
								     (x == GPIOG) ? 6 :\
								     (x == GPIOH) ? 7 :\
								     (x == GPIOI) ? 8 :0)

/*
 * Interrupt Request numbers for stm32f407xx.h MCU
 * NOTE: update these macros with valid values according to the MCU
 */
#define IRQ_NO_EXTI0       6
#define IRQ_NO_EXTI1       7
#define IRQ_NO_EXTI2       8
#define IRQ_NO_EXTI3       9
#define IRQ_NO_EXTI4       10
#define IRQ_NO_EXTI9_5     23
#define IRQ_NO_EXTI15_10   40

/*
 * Macros for all the possible priority levels
 * NVIC priority configuration
 */
#define NVIC_IRQ_PRI0      0
#define NVIC_IRQ_PRI1      0
#define NVIC_IRQ_PRI2      0
#define NVIC_IRQ_PRI3      0
#define NVIC_IRQ_PRI4      0
#define NVIC_IRQ_PRI5      0
#define NVIC_IRQ_PRI6      0
#define NVIC_IRQ_PRI7      0
#define NVIC_IRQ_PRI8      0
#define NVIC_IRQ_PRI9      0
#define NVIC_IRQ_PRI10     0
#define NVIC_IRQ_PRI11     0
#define NVIC_IRQ_PRI12     0
#define NVIC_IRQ_PRI13     0
#define NVIC_IRQ_PRI14     0
#define NVIC_IRQ_PRI15     0


/*
 * Disabling the Clock Macros
 */

/* Clock Disable macros for the GPIO peripherals */

#define GPIOA_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 8))

/* Disabling Clock Macros for the I2C peripherals */

#define I2C1_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 23))

/* Disabling Clock Macros for the SPI peripherals*/

#define SPI1_PCLK_DI()     (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()     (RCC->APB1ENR &= ~( 1 << 14))
#define SPI3_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 15))

/* Disabling Clock Macros for the usart peripherals */

#define USART2_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 18))
#define USART1_PCLK_DI()   (RCC->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI()   (RCC->APB2ENR &= ~(1 << 5))


/* Disabling the clock macros for the syscfg peripheral
 *
 *
 */

#define SYSCFG_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 14))


/*
 * Bit Position definitions for the SPI_CR1 register
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * bit position declaration for SPI_CR2 register
 *
 */

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE 		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 * bit positions for the SPI_SR register
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/*
 * BIT position definitions of I2C peripherals Registers
 *
 *
 */

/* CR1 REGISTER */
#define I2C_CR1_PE          0
#define I2C_CR1_SMBUS       1
#define I2C_CR1_SMBTYPE     3
#define I2C_CR1_ENARP       4
#define I2C_CR1_ENPEC       5
#define I2C_CR1_ENGC        6
#define I2C_CR1_NOSTRETCH   7
#define I2C_CR1_START       8
#define I2C_CR1_STOP        9
#define I2C_CR1_ACK         10
#define I2C_CR1_POS         11
#define I2C_CR1_PEC         12
#define I2C_CR1_ALERT       13
#define I2C_CR1_SWRST       15

/* CR2 REGISTER */

#define I2C_CR2_FREQ        0
#define I2C_CR2_ITERREN     8
#define I2C_CR2_ITEVTEN     9
#define I2C_CR2_ITBUFEN     10
#define I2C_CR2_DMAEN       11
#define I2C_CR2_LAST        12

/* SR1 REGISTER */

#define I2C_SR1_SB          0
#define I2C_SR1_ADDR        1
#define I2C_SR1_BTF         2
#define I2C_SR1_ADD10       3
#define I2C_SR1_STOPF       4
#define I2C_SR1_RXNE        6
#define I2C_SR1_TXE         7
#define I2C_SR1_BERR        8
#define I2C_SR1_ARLO        9
#define I2C_SR1_AF          10
#define I2C_SR1_OVR         11
#define I2C_SR1_PECERR      12
#define I2C_SR1_TIMEOUT     14
#define I2C_SR1_SMBALERT    15


/* SR2 REGISTER */

#define I2C_SR2_MSL         0
#define I2C_SR2_BUSY        1
#define I2C_SR2_TRA         2
#define I2C_SR2_GENCALL     4
#define I2C_SR2_SMBDEFAULT  5
#define I2C_SR2_SMBHOST     6
#define I2C_SR2_DUALF       7
#define I2C_SR2_PEC         8


/* CCR REGISTER */

#define I2C_CCR_CCR                0
#define I2C_CCR_DUTY               14
#define I2C_CCR_FS                 15

/* OAR REGISTER */







#include "stm32f407xx_gpio_driver.h"
#include "stm32f4xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"


#endif /* INC_STM32F407XX_H_ */
