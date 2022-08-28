/*
 * stm32f303xx_i2c_driver.c
 *
 *  Created on: May 18, 2022
 *      Author: jscoran
 */
#include "stm32f303xx_i2c_driver.h"


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
static void I2C_ExecuteAddressPhase(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr);
static void I2C_Transfer(I2C_Handle_t *pI2CHandle, uint8_t Address, uint8_t Size, uint32_t auto_or_reload, uint32_t start_stop);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {

	pI2Cx->CR2 |= (1 << I2C_CR2_START);

}
// @ StartStopModes
static void I2C_Transfer(I2C_Handle_t *pI2CHandle, uint8_t Address, uint8_t Size, uint32_t auto_or_reload, uint32_t start_stop)
{
	// Clear out the CR2 Register
	uint32_t tempreg = 0;
	uint32_t mode_mask;

	tempreg &= ~(0x3FF << I2C_CR2_SADD_0BIT | 0xFF << I2C_CR2_NBYTES | 1 << I2C_CR2_AUTOEND | 1 << I2C_CR2_RD_WRN | 1 << I2C_CR2_RELOAD | 1 << I2C_CR2_START | 1 << I2C_CR2_STOP) ;

	if(auto_or_reload == AUTOMODE) {

		mode_mask = 1 << I2C_CR2_AUTOEND;

	}else {
		mode_mask = 1 << I2C_CR2_RELOAD;

	}

	tempreg |= (((Address << 1) & 0x7F) << I2C_CR2_SADD_17BIT) | mode_mask | start_stop;
	pI2CHandle->pI2Cx->CR2 = tempreg;

}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {

	if(pI2Cx->SR & FlagName){

		return FLAG_SET;
	}

	return FLAG_RESET;

}



/*
 * Peripheral Clock Control
 */

/******************************************************************************
 * @fn				I2C_PeripheralClockControl
 *
 * @brief			 			Enable or Disable the peripheral clock for the given I2C
 *
 * @param[in] pGPIOx			Base Address of the I2C Peripheral (pointer), where x defines the peripheral
 * @param[in] enable_disable	Enable or Disable the Clock
 *
 * @return						None
 *
 * @Note
 */


void I2C_PeripheralClockControl(I2C_RegDef_t *pI2Cx, uint8_t enable_disable)
{
	if(enable_disable == ENABLE)
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
		else
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_DIS();
			}else if(pI2Cx == I2C2)
			{
				I2C2_PCLK_DIS();
			}else if(pI2Cx == I2C3)
			{
				I2C3_PCLK_DIS();
			}
		}


}

/******************************************************************************
 * @fn							I2C_Reset
 *
 * @brief			 			Reset the I2C Peripheral to it's initial settings
 *
 * @param[in] pSPIx				Base Address of the I2C Peripheral
 *
 * @return						None
 *
 * @Note
 */
void I2C_Reset(I2C_RegDef_t *pI2Cx) {

	// Reset SPIx Peripheral
	if(pI2Cx == I2C1) {
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2) {
		I2C2_REG_RESET();
	}else if(pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}

}


void I2C_Init(I2C_Handle_t *pI2CHandle) {

	uint32_t tempreg = 0;

	// Enable the Peripheral Clock for the specific I2C
	I2C_PeripheralClockControl(pI2CHandle->pI2Cx, ENABLE);

	//1. Disable the Peripheral. Most settings cannot be set while peripheral is active
	tempreg &= ~(1 << I2C_CR1_PE);

	//2. Configure ANOFF & DNF
	tempreg &= ~(1 << I2C_CR1_ANOFF);
	tempreg |= (pI2CHandle->I2CConfig.I2C_AnalogOff << I2C_CR1_ANFOFF);

	// Clear the value in bits 8 - 11
	tempreg &= ~(0xF << I2C_CR1_DNF);
	tempreg |= (pI2CHandle->I2CConfig.I2C_DigitalFilter << I2C_CR1_DNF);

	// Set the NoStretch Configuration
	tempreg &= ~(1 << I2C_CR1_NOSTRETCH);
	tempreg |= (pI2CHandle->I2CConfig.I2C_NoStretch << I2C_CR1_NOSTRETCH);

	// Apply the settings to the CR1 Register
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// Configure settings for CR2 Register in a temporary register & apply to CR2 Register
	// Enable the AUTOEND by default (will generate STOP),
	// and enable NACK (should be disabled only during Slave process) -> Pulled from HAL

	tempreg = 0;
	tempreg |= (1 << I2C_CR2_AUTOEND);
	tempreg |= (1 << I2C_CR2_NACK);
	pI2CHandle->pI2Cx->CR2 = tempreg;

	// Program the device own address & apply the settings to the OAR1 Register
	// Assumes this is for 7-Bit Addressing. 10 Bit Addressing not configured

	// Clear the Own Address Enable Bit, settings cannot be applied when this bit is 1.
	pI2CHandle->pI2Cx->OAR1 &= ~(1 << I2C_OAR1_OA1EN);

	tempreg = 0;
	tempreg |= pI2CHandle->I2CConfig.I2C_OwnAddress << I2C_OAR1_OA_17BIT;
	pI2CHandle->pI2Cx->OAR1 = tempreg;


	//4. Set the timing based on the target clock speed
	pI2CHandle->pI2Cx->TIMINGR = pI2CHandle->I2CConfig.I2C_Timing;

	// Enable the peripheral
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_PE);

}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint32_t bytes, uint8_t SlaveAddr) {

	// Generate the Start Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// Send the Slave Address by clear the address in the field
	// Write the Address to the SADD_1-7 Bits
	// We need to shift the slave address by one
	slave = SlaveAddr << 1;
	pI2CHandle->pI2Cx->CR2 &= ~(0x7F << I2C_CR2_SADD_17BIT);
	pI2CHandle->pI2Cx->CR2 |= ((slave & 0x7F) << I2C_CR2_SADD_17BIT);

	// Select a Write Condition
	pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR2_RD_WRN);

	// Place the Number of Bytes to be Transmitted


	uint32_t len;
	if(bytes < MAX_BYTES) {

		pI2CHandle->pI2Cx->CR2 |= (bytes << I2C_CR2_NBYTES);
		len = bytes;
	}
	else {

		// 0xFF == 255
		pI2CHandle->pI2Cx->CR2 |= (0xFF << I2C_CR2_NBYTES);

		// Set RELOAD Mode
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_RELOAD);
		len = MAX_BYTES;
	}


	// Send data until len 0
	while(bytes > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXIS) ); // Wait until TXIS is set (loop doesn't run and breaks to next line)
		pI2CHandle->pI2Cx->TXDR = *pTxBuffer;
		pTxBuffer++;
		len--;
		bytes--;

		if ((bytes != 0) && (len == 0))
		  {
			// Wait until TCR flag is set
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TCR) );

			if (bytes > MAX_BYTES)
			{
			  len = MAX_BYTES;

			  I2C_Transfer(pI2CHandle, slave, len, AUTOMODE, I2C_NO_START_STOP);
			}
			else
			{
			  len = bytes;
			  I2C_Transfer(pI2CHandle, slave, len, RELOADMODE, I2C_NO_START_STOP);
			}
		  }
	}

	// Wait until STOPF flag is set.
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF) );

	// Clear the Stop Flag
	pI2CHandle->pI2Cx |= (1 << I2C_CR1_ANFOFF);

}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint32_t bytes, uint8_t SlaveAddr) {

	uint8_t slave;
	// Configure the Slave Address
	slave = SlaveAddr << 1;
	pI2CHandle->pI2Cx->CR2 &= ~(0x7F << I2C_CR2_SADD_17BIT);
	pI2CHandle->pI2Cx->CR2 |= ((slave & 0x7F) << I2C_CR2_SADD_17BIT);

	// Select a Read Condition
	pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR2_RD_WRN);

	// Generate the Start Condition
	I2C_GenerateStartCondition();

	// Configure the number of bytes to be transmitted
	// Place the Number of Bytes to be Transmitted
	if(bytes < MAX_BYTES) {

		pI2CHandle->pI2Cx->CR2 |= (bytes << I2C_CR2_NBYTES);
		len = bytes;
	}
	else {

		// 0xFF == 255
		pI2CHandle->pI2Cx->CR2 |= (0xFF << I2C_CR2_NBYTES);

		// Set RELOAD Mode
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_RELOAD);
		len = MAX_BYTES;
	}

	// Read data until len 0
	while(bytes > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx I2C_FLAG_RXNE) ); // Wait until RXNE is set (loop doesn't run and breaks to next line)
		*pRxBuffer = pI2CHandle->pI2Cx->RXDR;
		pRxBuffer++;
		len--;
		bytes--;

		if ((bytes != 0) && (len == 0))
		  {
			// Wait until TCR flag is set
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TCR) );

			if (bytes > MAX_BYTES)
			{
			  len = MAX_BYTES;

			  I2C_Transfer(pI2CHandle, slave, len, AUTOMODE, I2C_NO_START_STOP);
			}
			else
			{
			  len = bytes;
			  I2C_Transfer(pI2CHandle, slave, len, RELOADMODE, I2C_NO_START_STOP);
			}
		  }
	}

	// Wait until STOPF flag is set.
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF) );

	// Clear the Stop Flag
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_SR_STOPF);

}

void I2C_SlaveReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr)
{
	// Wait until ADDR is set
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	// Slave mode requires a disabled NACK
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_NACK);

	// Clear the ADDR Flag
	pI2CHandle->pI2Cx->CR2 |= (1 << I2C_SR_ADDR);

	// Wait until DIR is set
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_DIR) );

	while (len > 0)
    {
      /* Wait until TXIS flag is set */
      while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXIS) );

      /* Write data to TXDR */
      pI2CHandle->pI2Cx->TXDR = pTxBuffer;

      /* Increment Buffer pointer */
      pTxBuffer++;
      len--;
    }

    // Wait until STOP flag is set //
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF) );

    // Clear STOP flag 
	pI2CHandle->pI2Cx->ICR &= ~(1 << I2C_ICR_STOPCF);

	 // Wait until Busy flag is reset //
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BUSY) );

    // Disable Address Acknowledge
    pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_NACK);

   
}

void I2C_SlaveSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr)
{
	// Wait until ADDR is set
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	// Slave mode requires a disabled NACK
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_NACK);

	// Clear the ADDR Flag
	pI2CHandle->pI2Cx->CR2 |= (1 << I2C_SR_ADDR);

	// Wait until DIR is set
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_DIR) );

	while (len > 0)
    {
      /* Wait until TXIS flag is set */
      while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXIS) );

      /* Write data to TXDR */
      pI2CHandle->pI2Cx->TXDR = pTxBuffer;

      /* Increment Buffer pointer */
      pTxBuffer++;
      len--;
    }

    /* Wait until STOP flag is set */
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF) );
   
    // Clear STOP flag 
	pI2CHandle->pI2Cx->CR2 |= (1 << I2C_SR_STOPF);

    // Disable Address Acknowledge
    pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_NACK);

   
}




