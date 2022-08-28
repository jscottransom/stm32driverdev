/*
 * stm32f303xx_usart_driver.c
 *
 *  Created on: Jul 5, 2022
 *      Author: jscoran
 */

/**
 * @file stm32f303xx_usart_driver.c
 * @author Jared Scott-Ransom
 * @date 9 Sep 2012
 * @brief File containing example of doxygen usage for quick reference.
 *
 * Here typically goes a more extensive explanation of what the header
 * defines. Doxygens tags are words preceeded by either a backslash @\
 * or by an at symbol @@.
 * @see http://www.stack.nl/~dimitri/doxygen/docblocks.html
 * @see http://www.stack.nl/~dimitri/doxygen/commands.html
 */

#include "stm32f303xx_usart_driver.h"

/******************************************************************************
 * @fn				USART_PeripheralClockControl
 *
 * @brief			 			Enable or Disable the peripheral clock for the given USART
 *
 * @param[in] pGPIOx			Base Address of the USART Peripheral (pointer), where x defines the peripheral
 * @param[in] enable_disable	Enable or Disable the Clock
 *
 * @return						None
 *
 * @Note
 */

void USART_PeripheralClockControl(SPI_RegDef_t *pUSARTx, uint8_t enable_disable)
{
	if(enable_disable == ENABLE)
		{
			if(pUSARTx == USART1)
			{
				USART1_PCLK_EN();
			}else if(pUSARTx == USART2)
			{
				USART2_PCLK_EN();
			}else if(pUSARTx == USART3)
			{
				USART3_PCLK_EN();
			}else if(pUSARTx == UART4)
			{
				UART4_PCLK_EN();
			}else if(pUSARTx == UART5)
			{
				USART5_PCLK_EN();
			}
		}
		else
		{
			if(pUSARTx == USART1)
			{
				USART1_PCLK_DIS();
			}else if(pUSARTx == USART2)
			{
				USART2_PCLK_DIS();
			}else if(pUSARTx == USART3)
			{
				USART3_PCLK_DIS();
			}else if(pUSARTx == UART4)
			{
				UART4_PCLK_DIS();
			}else if(pUSARTx == UART5)
			{
				USART5_PCLK_DIS();
			}
		}
}


/**
 * @brief Enable or disable the peripheral.
 *
 * A simple utility to allow or disallow features of the given USART peripheral.
 * @param pUSARTx USART peripheral to enable or disable.
 * @param enable_or_disable	Boolean flag to either enable or disable the peripheral.
 * @return None
 */

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t enable_or_disable)
{
	if(enable_or_disable == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}

}


/******************************************************************************
 * @fn				USART_Init
 *
 * @brief
 *
 * @param[in] pUSARTx
 *
 * @return						None
 *
 * @Note
 */
void USART_Init(USART_Handle_t *pUSARTHandle) {
	{

	//Temporary variable
	uint32_t tempreg=0;

	// Disable the peripheral. This is necessary so most settings can be configured.
	tempreg &= ~(1 << USART_CR1_UE);

	/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	USART_PeripheralClockControl(pUSARTHandle->pUSARTx, enable);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= ( 1 << USART_CR1_RE );
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}

	//M0 Word Length should be cleared for 8 & 9 Bit Word Lengths
	tempreg &= ~(1 << USART_CR1_M0);
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M1 ;

	//Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Enable parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable ODD parity
		tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= ( 1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= ( ( 1 << USART_CR3_CTSE) | ( 1 << USART_CR3_RTSE) );
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/******************************** Configuration of BRR(Baudrate register)******************************************/

		//Implement the code to configure the baud rate
		//We will cover this in the lecture. No action required here

	}


}



/******************************************************************************
 * @fn							USART_Reset
 *
 * @brief			 			Reset the USART Peripheral to it's initial settings
 *
 * @param[in] pSPIx				Base Address of the USART Peripheral
 *
 * @return						None
 *
 * @Note
 */
void USART_Reset(USART_RegDef_t *pUSARTx) {

	// Reset SPIx Peripheral
	if(pUSARTx == USART1) {
		USART1_REG_RESET();
	}else if(pUSARTx == USART2) {
		USART2_REG_RESET();
	}else if(pUSARTx == USART3) {
		USART3_REG_RESET();
	}else if(pUSARTx == UART4) {
		UART4_REG_RESET();
	}else if(pUSARTx == UART5) {
		UART5_REG_RESET();
	}

}



/******************************************************************************
 * @fn							USART_GetFlagStatus
 *
 * @brief			 				Determine if a given flag is set
 *
 * @param[in] pUSARTx				Base address of USART peripheral
 * @param[in] StatusFlagName		Name of status flag
 *
 * @return							None
 *
 * @Note
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{

	if(pUSARTx->ISR & StatusFlagName){

		return FLAG_SET;
	}

	return FLAG_RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{

	pUSARTx->ICR |= ( 1 << StatusFlagName);
}


/*
 * IRQ & ISR Handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_or_disable)
{
	if(enable_or_disable == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// Program ISER0 Register
			*NVIC_ISER0 |= ( 1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// Program ISER1 Register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program ISER2 Register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			// Program ICER0 Register
			*NVIC_ICER0 |= ( 1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// Program ICER1 Register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program ICER2 Register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}

	}

}


void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// Identify IPR Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;


	// Lower 4 bits of the register are not implemented, so we need to shift by implemented bits (architecture specific)
	uint8_t shift_amount = (8 * iprx_section) + (8 - PRIORITY_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount );
}

