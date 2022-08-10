/*
 * stm32f303xx.h
 *
 *  Created on: Mar 17, 2022
 *      Author: jscoran
 */

#ifndef INC_STM32F303XX_H_
#define INC_STM32F303XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo 	volatile	// Short hand notation for volatile
#define __weak	__attribute__((weak))

/**************************Processor Specific Details***************************/
/*
 * ARM Cortex Mx Processor NVIC Interrupt Set Enable Registerx Register Addresses
 */
#define NVIC_ISER0		  ( (__vo uint32_t*)0xE000E100)
#define NVIC_ISER1		  ( (__vo uint32_t*)0xE000E104)
#define NVIC_ISER2		  ( (__vo uint32_t*)0xE000E108)
#define NVIC_ISER3		  ( (__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC Interrupt Clear Enable Registerx Register Addresses
 */
#define NVIC_ICER0		  ( (__vo uint32_t*)0xE000E180)
#define NVIC_ICER1		  ( (__vo uint32_t*)0xE000E184)
#define NVIC_ICER2		  ( (__vo uint32_t*)0xE000E188)
#define NVIC_ICER3		  ( (__vo uint32_t*)0xE000E18C)



/*
 * ARM Cortex Mx Processor NVIC Priority Base Address
 */
#define NVIC_PR_BASE_ADDR ( (__vo uint32_t*)0xE000E400)


/*
 * ARM Cortex Mx Processor Priority Bits Implemented
 */
#define PRIORITY_BITS_IMPLEMENTED 	4


/*
 * Base Addresses of Flash and SRAM memories
 * Addresses can't be "signed", so we can use UL
 */

#define FLASH_BASEADDR		0x08000000UL	// Flash aka Code Memory Address
#define SRAM1_BASEADDR		0x20000000UL	// 40KB of SRAM (40 * 1024 = 40,960 Bytes)
#define	SRAM1 				SRAM1_BASEADDR
#define CCM_SRAM_BASEADDR	0x10000000UL
#define CCM_SRAM			CCM_SRAM_BASEADDR
#define ROM					0x1FFFD800UL	// ROM == System Memory

#define PERIPH_BASEADDR		0x40000000UL
#define APB1_BASEADDR		PERIPH_BASEADDR
#define APB2_BASEADDR		0x40010000UL
#define AHB1_BASEADDR		0x40020000UL
#define AHB2_BASEADDR		0x48000000UL
#define AHB3_BASEADDR		0x50000000UL

// AHB1 Base Addresses
#define RCC_BASEADDR		(AHB1_BASEADDR + 0x1000)

// AHB2 Base Addresses
#define GPIOA_BASEADDR		(AHB2_BASEADDR + 0x0000)
#define GPIOB_BASEADDR		(AHB2_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB2_BASEADDR + 0x0800)
#define GPIOD_BASEADDR		(AHB2_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		(AHB2_BASEADDR + 0x1000)
#define GPIOF_BASEADDR		(AHB2_BASEADDR + 0x1400)

// APB1 Base Addresses
#define I2C1_BASEADDR		(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR		(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR		(APB1_BASEADDR + 0x7800)

#define SPI2_BASEADDR		(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1_BASEADDR + 0x3C00)

#define UART4_BASEADDR		(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR		(APB1_BASEADDR + 0x5000)

#define USART2_BASEADDR		(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR		(APB1_BASEADDR + 0x4800)


// APB2 Base Addresses
#define EXTI_BASEADDR		(APB2_BASEADDR + 0x0400)
#define SPI1_BASEADDR		(APB2_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR		(APB2_BASEADDR + 0x0000)
#define USART1_BASEADDR		(APB2_BASEADDR + 0x3800)

/*
 * Peripheral Register Definition Structure for GPIO
 */

// Generic Structure for STM32F303xx Device GPIO Peripherals
//

// To Do - Document the member elements
typedef struct
{
	__vo uint32_t MODER;			/* GPIO Port Mode Register, Address Offset 0x00 */
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];			/* AFR[0]: GPIO Alternate Function Low Register, AFR[1]: GPIO Alternate Function High Register */
	__vo uint32_t BRR;
}GPIO_RegDef_t;


#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)


typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t APB1RSTR;
	__vo uint32_t AHBENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t APB1ENR;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t AHBRSTR;
	__vo uint32_t CFGR2;
	__vo uint32_t CFGR3;

}RCC_RegDef_t;
/*
 * EXTI Register Definition
 */
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
	__vo uint32_t IMR2;
	__vo uint32_t EMR2;
	__vo uint32_t RTSR2;
	__vo uint32_t FTSR2;
	__vo uint32_t SWIER2;
	__vo uint32_t PR2;
}EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t CFGR;
	__vo uint32_t RCR;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CFGR2;
	__vo uint32_t CFGR3;
	__vo uint32_t CFGR4;
}SYSCFG_RegDef_t;

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI 				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * Clock Enable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_EN()		(RCC->AHBENR |=  (1 << 17) )
#define GPIOB_PCLK_EN()		(RCC->AHBENR |=  (1 << 18) )
#define GPIOC_PCLK_EN()		(RCC->AHBENR |=  (1 << 19) )
#define GPIOD_PCLK_EN()		(RCC->AHBENR |=  (1 << 20) )
#define GPIOE_PCLK_EN()		(RCC->AHBENR |=  (1 << 21) )
#define GPIOF_PCLK_EN()		(RCC->AHBENR |=  (1 << 22) )

/*
 * Clock Disable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_DIS()	(RCC->AHBENR &=  ~(1 << 17) )
#define GPIOB_PCLK_DIS()	(RCC->AHBENR &=  ~(1 << 18) )
#define GPIOC_PCLK_DIS()	(RCC->AHBENR &=  ~(1 << 19) )
#define GPIOD_PCLK_DIS()	(RCC->AHBENR &=  ~(1 << 20) )
#define GPIOE_PCLK_DIS()	(RCC->AHBENR &=  ~(1 << 21) )
#define GPIOF_PCLK_DIS()	(RCC->AHBENR &=  ~(1 << 22) )

/*
 * System Clock Enable / Disable Macro
 */
#define SYS_PCLK_EN()		(RCC->APB2ENR |= (1 << 0))
#define SYS_PCLK_DIS()		(RCC->APB2ENR &= ~(1 << 0))

typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t TIMINGR;
	__vo uint32_t TIMEOUTR;
	__vo uint32_t ISR;
	__vo uint32_t ICR;
	__vo uint32_t PECR;
	__vo uint32_t RXDR;
	__vo uint32_t TXDR;
}I2C_RegDef_t;


#define I2C1				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*)I2C3_BASEADDR)

/*
 * Clock Enable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 30))

/*
 * Clock Disable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_DIS()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DIS()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DIS()		(RCC->APB1ENR &= ~(1 << 30))

typedef struct {
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

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)


/*
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))


/*
 * Clock Disable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_DIS()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DIS()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DIS()		(RCC->APB1ENR &= ~(1 << 15))

typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t BRR;
	__vo uint32_t GTPR;
	__vo uint32_t RTOR;
	__vo uint32_t RQR;
	__vo uint32_t ISR;
	__vo uint32_t ICR;
	__vo uint32_t RDR;
	__vo uint32_t TDR;
}USART_RegDef_t;

#define USART1				((USART_RegDef_t*)USART1_BASEADDR)
#define USART2				((USART_RegDef_t*)USART2_BASEADDR)
#define USART3				((USART_RegDef_t*)USART3_BASEADDR)
#define UART4				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5				((USART_RegDef_t*)UART5_BASEADDR)




/*
 * Clock Enable Macros for USARTx Peripherals
 */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))

/*
 * Clock Disable Macros for USARTx Peripherals
 */
#define USART1_PCLK_DIS()	(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 20))



/*
 * Clock Reset Macros for GPIOx Peripherals
 */
#define GPIOA_REG_RESET()		do{ ( RCC->AHBRSTR |=  (1 << 17) ); ( RCC->AHBRSTR &= ~(1 << 17) ); }while(0)
#define GPIOB_REG_RESET()		do{ ( RCC->AHBRSTR |=  (1 << 18) ); ( RCC->AHBRSTR &= ~(1 << 18) ); }while(0)
#define GPIOC_REG_RESET()		do{ ( RCC->AHBRSTR |=  (1 << 19) ); ( RCC->AHBRSTR &= ~(1 << 19) ); }while(0)
#define GPIOD_REG_RESET()		do{ ( RCC->AHBRSTR |=  (1 << 20) ); ( RCC->AHBRSTR &= ~(1 << 20) ); }while(0)
#define GPIOE_REG_RESET()		do{ ( RCC->AHBRSTR |=  (1 << 21) ); ( RCC->AHBRSTR &= ~(1 << 21) ); }while(0)
#define GPIOF_REG_RESET()		do{ ( RCC->AHBRSTR |=  (1 << 22) ); ( RCC->AHBRSTR &= ~(1 << 22) ); }while(0)

#define GPIO_BASEADDR_CODE(x)		(	(x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
										(x == GPIOE)?4:\
										(x == GPIOF)?5:0)


/*
 * Clock Reset Macros for SPIx Peripherals
 */
#define SPI1_REG_RESET()		do{ ( RCC->APB2RSTR |=  (1 << 12) ); ( RCC->APB2RSTR &= ~(1 << 12) ); }while(0)
#define SPI2_REG_RESET()		do{ ( RCC->APB1RSTR |=  (1 << 14) ); ( RCC->APB1RSTR &= ~(1 << 14) ); }while(0)
#define SPI3_REG_RESET()		do{ ( RCC->APB1RSTR |=  (1 << 15) ); ( RCC->APB1RSTR &= ~(1 << 15) ); }while(0)


/*
 * Clock Reset Macros for I2Cx Peripherals
 */
#define I2C1_REG_RESET()		do{ ( RCC->APB1RSTR |=  (1 << 21) ); ( RCC->APB1RSTR &= ~(1 << 21) ); }while(0)
#define I2C2_REG_RESET()		do{ ( RCC->APB1RSTR |=  (1 << 22) ); ( RCC->APB1RSTR &= ~(1 << 22) ); }while(0)
#define I2C3_REG_RESET()		do{ ( RCC->APB1RSTR |=  (1 << 30) ); ( RCC->APB1RSTR &= ~(1 << 30) ); }while(0)


/*
 * Clock Reset Macros for USART/UARTx Peripherals
 */
#define USART1_REG_RESET()		do{ ( RCC->APB2RSTR |=  (1 << 14) ); ( RCC->APB1RSTR &= ~(1 << 14) ); }while(0)
#define USART2_REG_RESET()		do{ ( RCC->APB1RSTR |=  (1 << 17) ); ( RCC->APB1RSTR &= ~(1 << 17) ); }while(0)
#define USART3_REG_RESET()		do{ ( RCC->APB1RSTR |=  (1 << 18) ); ( RCC->APB1RSTR &= ~(1 << 18) ); }while(0)
#define UART4_REG_RESET()		do{ ( RCC->APB1RSTR |=  (1 << 19) ); ( RCC->APB1RSTR &= ~(1 << 19) ); }while(0)
#define UART5_REG_RESET()		do{ ( RCC->APB1RSTR |=  (1 << 20) ); ( RCC->APB1RSTR &= ~(1 << 20) ); }while(0)


// IRQ_NO_ Macros
// This provides the position of the interrupt
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2_TS		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI10_15	40

// IRQ No SPI Macros
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51


// Generic Macros
#define ENABLE 				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define	FLAG_SET			SET


// EXTI Macros
// This provides the position of the interrupt
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2_TS		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI10_15	40

/*
 * Priority Macros
 */
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15

/*
 * SPI_CR1 Bit Field Definitions. To be used in place of the bit positions within
 * the register.
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RX_ONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRC_NEXT	12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * SPI_CR2 Bit Field Definitions. To be used in place of the bit positions within
 * the register.
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_NSSP		3
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7
#define SPI_CR2_DS			8
#define SPI_CR2_FRXTH		12
#define SPI_CR2_LDMARX		13
#define SPI_CR2_LDMATX		14


/*
 * SPI_CR2 Bit Field Definitions. To be used in place of the bit positions within
 * the register.
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
#define SPI_SR_FRLVL		9
#define SPI_SR_FTLVL		11


/*
 * I2Cx_CR1 Bit Field Definitions
 */
#define I2C_CR1_PE					0
#define I2C_CR1_TXIE				1
#define I2C_CR1_RXIE				2
#define I2C_CR1_ADDRIE				3
#define I2C_CR1_NACKIE				4
#define I2C_CR1_STOPIE				5
#define I2C_CR1_TCIE				6
#define I2C_CR1_ERRIE				7
#define I2C_CR1_DNF					8
#define I2C_CR1_ANFOFF				12
#define I2C_CR1_TXDMAEN				14
#define I2C_CR1_RXDMAEN				15
#define I2C_CR1_SBC					16
#define I2C_CR1_NOSTRETCH			17
#define I2C_CR1_WUPEN				18
#define I2C_CR1_GCEN				19
#define I2C_CR1_SMBHEN				20
#define I2C_CR1_SMBDEN				21
#define I2C_CR1_ALERTEN				22
#define I2C_CR1_PECEN				23

/*
 * I2Cx_CR2 Bit Field Definitions
 */
#define I2C_CR2_SADD_0BIT			0
#define I2C_CR2_SADD_17BIT			1
#define I2C_CR2_RD_WRN				10
#define I2C_CR2_ADD10				11
#define I2C_CR2_HEAD10R				12
#define I2C_CR2_START				13
#define I2C_CR2_STOP				14
#define I2C_CR2_NACK				15
#define I2C_CR2_NBYTES				16
#define I2C_CR2_RELOAD				24
#define I2C_CR2_AUTOEND				25
#define I2C_CR2_PECBYTE				26

/*
 * I2Cx_SR Bit Field Definitions
 */
#define I2C_SR_TXE					0
#define I2C_SR_TXIS					1
#define I2C_SR_RXNE					2
#define I2C_SR_ADDR					3
#define I2C_SR_NACKF				4
#define I2C_SR_STOPF				5
#define I2C_SR_TC					6
#define I2C_SR_TCR					7
#define I2C_SR_BERR					8
#define I2C_SR_ARLO					9
#define I2C_SR_OVR					10
#define I2C_SR_PECERR				11
#define I2C_SR_TIMEOUT				12
#define I2C_SR_ALERT				13
#define I2C_SR_BUSY					15
#define I2C_SR_DIR					16
#define I2C_SR_ADDCODE				17

/*
 * I2Cx_ICR Bit Field Definitions
 */
#define I2C_ICR_ADDRCF				3
#define I2C_ICR_NACKCF				4
#define I2C_ICR_STOPCF				5
#define I2C_ICR_BERRCF				8
#define I2C_ICR_ARLOCF				9
#define I2C_ICR_OVRCF				10
#define I2C_ICR_PECCF				11
#define I2C_ICR_TIMOUTCF			12
#define I2C_ICR_ALERTCF				13

/*
 * I2Cx_OAR1 Bit Field Definitions
 */
#define I2C_OAR1_OA_0BIT			0
#define I2C_OAR1_OA_17BIT			1
#define I2C_OAR1_OA_89BIT			2
#define I2C_OAR1_OA1MODE			10
#define I2C_OAR1_OA1EN				15

/*
 * I2Cx_RXDR Bit Field Definitions
 */
#define I2C_RXDR_RXDATA				0

/*
 * I2Cx_TXDR Bit Field Definitions
 */
#define I2C_RXDR_TXDATA				0

/*
 * USART_CR1 Bit Field Definitions. To be used in place of the bit positions within
 * the register.
 */
#define USART_CR1_UE		0
#define USART_CR1_UESM		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE	    10
#define USART_CR1_WAKE		11
#define USART_CR1_M0		12
#define USART_CR1_MME		13
#define USART_CR1_CMIE		14
#define USART_CR1_OVER8		15
#define USART_CR1_DEDT		16
#define USART_CR1_DEAT		21
#define USART_CR1_RTOIE		26
#define USART_CR1_EOBIE		27
#define USART_CR1_M1		28

/*
 * USART_CR2 Bit Field Definitions. To be used in place of the bit positions within
 * the register.
 */
#define USART_CR2_ADDM7			4
#define USART_CR2_LBDL			5
#define USART_CR2_LBDIE			6
#define USART_CR2_LBCL			8
#define USART_CR2_CPHA	    	9
#define USART_CR2_CPOL  		10
#define USART_CR2_CLKEN			11
#define USART_CR2_STOP			12
#define USART_CR2_LINEN			14
#define USART_CR2_SWAP			15
#define USART_CR2_RXINV	    	16
#define USART_CR2_TXINV			17
#define USART_CR2_DATAINV		18
#define USART_CR2_MSBFIRST		19
#define USART_CR2_ABREN			20
#define USART_CR2_ABRMOD		21
#define USART_CR2_RTOEN			23
#define USART_CR2_ADD_FIRST		24
#define USART_CR2_ADD_SECOND	28

/*
 * USART_BRR Bit Field Definitions
 */
#define USART_BRR_UNDER8		0
#define USART_BRR_OVER8			4

/*
 * USART_ISR Bit Field Definitions. To be used in place of the bit positions within
 * the register.
 */
#define USART_ISR_PE			0
#define USART_ISR_FE			1
#define USART_ISR_NF			2
#define USART_ISR_ORE			3
#define USART_ISR_IDLE			4
#define USART_ISR_RXNE			5
#define USART_ISR_TC			6
#define USART_ISR_TXE			7
#define USART_ISR_LBDF			8
#define USART_ISR_CTSIF			9
#define USART_ISR_CTS			10
#define USART_ISR_RTOF			11
#define USART_ISR_EOBF			12
#define USART_ISR_ABRE			14
#define USART_ISR_ABRF			15
#define USART_ISR_BUSY			16
#define USART_ISR_CMF			17
#define USART_ISR_SBKF			18
#define USART_ISR_RWU			19
#define USART_ISR_WUF			20
#define USART_ISR_TEACK			21
#define USART_ISR_REACK			22

/*
 * USART_ICR Bit Field Definitions. To be used in place of the bit positions within
 * the register.
 */
#define USART_ICR_PECF				0
#define USART_ICR_FECF				1
#define USART_ICR_NCF				2
#define USART_ICR_ORECF				3
#define USART_ICR_IDLECF			4
#define USART_ICR_TCCF				5
#define USART_ICR_LBDCF				8
#define USART_ICR_CTSCF				9
#define USART_ICR_RTOCF				11
#define USART_ICR_EOBCF				12
#define USART_ICR_CMCF				17
#define USART_ICR_WUCF				20




#include "stm32f303xx_gpio_driver.h"
#include "stm32f303xx_spi_driver.h"
#include "stm32f303xx_i2c_driver.h"

#endif /* INC_STM32F303XX_H_ */
