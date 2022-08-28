/*
 * stm32f303xx_spi_driver.c
 *
 *  Created on: May 4, 2022
 *      Author: jscoran
 */


#include "stm32f303xx_spi_driver.h"

// User should not call these functions (these are helper functions, that's why they are static)
static void spi_txe_interrupt_handle();
static void spi_rxne_interrupt_handle();
static void spi_ovr_err_interrupt_handle();

/*
 * Peripheral Clock Control
 */

/******************************************************************************
 * @fn				SPI_PeripheralClockControl
 *
 * @brief			 			Enable or Disable the peripheral clock for the given SPI
 *
 * @param[in] pGPIOx			Base Address of the SPI Peripheral (pointer), where x defines the peripheral
 * @param[in] enable_disable	Enable or Disable the Clock
 *
 * @return						None
 *
 * @Note
 */

void SPI_PeripheralClockControl(SPI_RegDef_t *pSPIx, uint8_t enable_disable)
{
	if(enable_disable == ENABLE)
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
		}
		else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DIS();
			}else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DIS();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLK_DIS();
			}
		}


}

/******************************************************************************
 * @fn							SPI_Init
 *
 * @brief			 			Enable or Disable the peripheral clock for the given SPI
 *
 * @param[in] pSPIHandle		Configuration Settings for SPI including
 *
 * @return						None
 *
 * @Note
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Configure SPI_CR1 Register

	uint32_t tempreg = 0;

	// Enable the clock for the peripheral
	SPI_PeripheralClockControl(pSPIHandle->pSPIx, ENABLE);

	// Configure Device Mode (3rd Bit of the register)

	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){

		// BIDI Mode Cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){

		// BIDI mode Set
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX){

		// BIDI Mode Cleared
		// RX Only must be set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RX_ONLY);

	}

	// Configure SPI_BusConfig
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// Configure SPI_DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// Configure CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// Configure CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//Configure SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	// The values of various bits are configured at once in a temporary register, then saved to the location of the SPI register.
	pSPIHandle->pSPIx->CR1 = tempreg;


}

/******************************************************************************
 * @fn							SPI_Reset
 *
 * @brief			 			Reset the SPI Peripheral to it's initial settings
 *
 * @param[in] pSPIx				Base Address of the SPI Peripheral
 *
 * @return						None
 *
 * @Note
 */
void SPI_Reset(SPI_RegDef_t *pSPIx) {

	// Reset SPIx Peripheral
	if(pSPIx == SPI1) {
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2) {
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3) {
		SPI3_REG_RESET();
	}

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {


	if(pSPIx->SR & FlagName){

		return FLAG_SET;
	}


	return FLAG_RESET;




}


/******************************************************************************
 * @fn							SPI_SendData
 *
 * @brief			 			Send Data in the Tx Buffer (blocking call, polling type)
 *
 * @param[in] pSPIx				Base Address of the SPI Peripheral
 * @param[in] pTxBuffer			Buffer Data to Send
 * @param[in] len				Length of Buffer Data
 *
 * @return						None
 *
 * @Note
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {

	// Check if the length of the TxBuffer is 0
	while(len > 0) {

		// Wait until the the TXE Buffer is Empty (ie. FLAG_SET). This will block
		// until it's true when run.
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);


		// Right Shift Register by position of the TXE bit, then create a mask and compare using bitwise and

		if(( pSPIx->CR1 & (1 << SPI_CR1_DFF) ))
		{

			// Load DR with 2 Bytes of Data
			// TxBuffer is a pointer, we need to type cast to uint16_t, since this is a 16 bit format
			//  Then dereference to load data
			pSPIx->DR = *((uint16_t*)pTxBuffer);

			// Increment the address of the buffer by 2, since we sent out two bytes of data
			(uint16_t*)pTxBuffer++;

			// Decrement the length by 2
			len-=2;
		}else
		{

			// Load DR with 1 Byte of Data
			pSPIx->DR = *(pTxBuffer);

			// Increment the address of the buffer by 1
			pTxBuffer++;

			// Decrement the length by 1
			len--;

		}

	}

}

/******************************************************************************
 * @fn							SPI_PeripheralControl
 *
 * @brief			 				Enable or Disable the peripheral for the given SPI
 *
 * @param[in] pSPIx					Base address of SPI peripheral
 * @param[in] enable_or_disable		Enable or Disable the peripheral
 *
 * @return							None
 *
 * @Note
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t enable_or_disable)
{
	if(enable_or_disable == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}

/******************************************************************************
 * @fn							SPI_SSIConfig
 *
 * @brief			 				Enable or Disable the SSI pin for the given SPI
 *
 * @param[in] pSPIx					Base address of SPI peripheral
 * @param[in] enable_or_disable		Enable or Disable the SSI Pin
 *
 * @return							None
 *
 * @Note
 */

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enable_or_disable)

{
	if(enable_or_disable == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}

}


/******************************************************************************
 * @fn							SPI_ReceiveData
 *
 * @brief			 			Receive Data in the Rx Buffer (blocking call, polling type)
 *
 * @param[in] pSPIx				Base Address of the SPI Peripheral
 * @param[in] pRxBuffer			Buffer Data to Read
 *
 * @return						None
 *
 * @Note
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) {

	// Check if the length of the TxBuffer is 0
	while(len > 0) {

		// Wait until the the RXE Buffer is Non-Empty (ie. FLAG_RESET). This will block
		// until it's true when run.
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		if(( pSPIx->CR1 & (1 << SPI_CR1_DFF) ))
		{
			// Load DR with 2 Bytes of Data
			*((uint16_t*)pRxBuffer) = pSPIx->DR;

			// Increment the address of the buffer by 2, since we sent out two bytes of data
			(uint16_t*)pRxBuffer++;

			// Decrement the length by 1
			len-=2;
		}else
		{

			// Load DR with 1 Byte of Data
			*(pRxBuffer) = pSPIx->DR;

			// Increment the address of the buffer by 1
			pRxBuffer++;

			// Decrement the length by 1
			len--;

		}

	}

}


/*
 * IRQ & ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_or_disable)
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
			// Program ISER0 Register
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


void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// Identify IPR Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;


	// Lower 4 bits of the register are not implemented, so we need to shift by implemented bits (architecture specific)
	uint8_t shift_amount = (8 * iprx_section) + (8 - PRIORITY_BITS_IMPLEMENTED);


	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount );
}


uint8_t SPI_SendDataInt(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	// Data is not written to the register in this version of Send data, that is actually handled in the Interrupt Handler
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{

		// Save the Tx Buffer Address and Length Info in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;
		// Mark the SPI State as busy in transmission, to prevent other code from using the same SPI peripheral.
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		// Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// Data Transmission handled by ISR
	}

	return state;

}

uint8_t SPI_ReceiveDataInt(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	// Data is not written to the register in this version of Send data, that is actually handled in the Interrupt Handler
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{

		// Save the Tx Buffer Address and Length Info in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;
		// Mark the SPI State as busy in transmission, to prevent other code from using the same SPI peripheral.
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		// Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// Data Reception handled by ISR
	}

	return state;
}





void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	// Check if the TXE Flag is Set
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		// Handle the TXE scenario
		spi_txe_interrupt_handle(pSPIHandle);

	}


	// Check if the RXNE Flag is Set
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		// Handle the RXNE scenario
		spi_rxne_interrupt_handle(pSPIHandle);

	}

	// Check if the OVR Flag is Set
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		// Handle the RXNE scenario
		spi_ovr_err_interrupt_handle();

	}


}

// Helper Functions
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {
if(( pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ))
	{

		// Load DR with 2 Bytes of Data
		// TxBuffer is a pointer, we need to type cast to uint16_t, since this is a 16 bit format
		//  Then dereference to load data
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);

		// Increment the address of the buffer by 2 bytes (which is an increment of one for type uint16_t), since we sent out two bytes of data
		(uint16_t*)pSPIHandle->pTxBuffer++;

		// Decrement the length by 2
		pSPIHandle->TxLen-=2;
	}else
	{

		// Load DR with 1 Byte of Data
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);

		// Increment the address of the buffer by 1
		pSPIHandle->pTxBuffer++;

		// Decrement the length by 1
		pSPIHandle->TxLen--;

	}

	if(!pSPIHandle->TxLen) {
		// TxLen is 0

		//Deactivate the TXEIE bit.
		// Prevents interrupts from TXE flag or setting of TXE Flag
		SPI_CloseTransmission(pSPIHandle);

		// Let the app know that Tx is REady
		SPI_AppEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}

}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(( pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ))
		{

			// Load DR with 2 Bytes of Data
			// RxBuffer is a pointer, we need to type cast to uint16_t, since this is a 16 bit format
			//  Then dereference to load data
		    *((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;

			// Increment the address of the buffer by 2 bytes (which is an increment of one for type uint16_t), since we sent out two bytes of data
			(uint16_t*)pSPIHandle->pRxBuffer--;

			// Decrement the length by 2
			pSPIHandle->RxLen-= 2;
		}else
		{

			// Load DR with 1 Byte of Data
			 *(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;

			// Decrement the address of the buffer by 1
			pSPIHandle->pRxBuffer--;

			// Decrement the length by 1
			pSPIHandle->RxLen--;

		}

		if(!pSPIHandle->RxLen) {
			// RxLen is 0

			//Deactivate the TXEIE bit.
			// Prevents interrupts from TXE flag or setting of TXE Flag
			SPI_CloseReception(pSPIHandle);

			// Let the app know that Rx is Ready
			SPI_AppEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
		}

}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	// Clear the OVR Flag (read DR then SR)

	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	// Inform Application
	SPI_AppEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;

	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}

__weak void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t app_event)
{
	// This is a weak implementation. Application may override
}





