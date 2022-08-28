/*
 * stm32f303xx_gpio.c
 *
 *  Created on: Mar 21, 2022
 *      Author: jscoran
 */

#include "stm32f303xx_gpio_driver.h"

/*
 * Peripheral Clock Control
 */

/******************************************************************************
 * @fn				GPIO_PeripheralClockControl
 *
 * @brief			 			Enable or Disable the peripheral clock for the given GPIO port
 *
 * @param[in] pGPIOx			Base Address of the GPIO Peripheral (pointer), where x defines the Port
 * @param[in] enable_disable	Enable or Disable the Clock
 *
 * @return						None
 *
 * @Note
 */

void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enable_disable)
{
	if(enable_disable == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DIS();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DIS();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DIS();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DIS();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DIS();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DIS();
		}
	}
}

/******************************************************************************
 * @fn				GPIO_Init
 *
 * @brief					Initialize the GPIO
 *
 * @param[in] pGPIOHandle	Configuration Settings for GPIO including Mode, Pin, Speed, Output Type, Alt Function
 *
 * @return					None
 *
 * @Note
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp_value = 0;

	// Enable the clock
	GPIO_PeripheralClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// Configure the Mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Handles the non-interrupt scenarios

		// PinMode and PinNumber are provided by the user. The PinMode will be shifted into the correct register
		// by the PinNumber value.
		temp_value = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		// Once the correct register has been identified, dereference the Handle, to get access to the Base Address
		// for the GPIO port. Dereference once again to get the Mode Register and place the value

		// When changing the contents of a register, use bitwise OR to only change the specified bits
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear
		pGPIOHandle->pGPIOx->MODER |= temp_value; // Set

	}else
	{
		// Interrupt Mode


		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_FALLING)
		{
			// Configure Falling Trigger Mode (FTSR) -> Falling Trigger Status Register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// Clear the corresponding RTSR bit (safety)
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_RISING)
		{
			// Configure the Rising Trigger Mode (RTSR) -> Rising Trigger Status Register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// Clear the corresponding FTSR bit (safety)
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_FALL_RISE_TRIG){
			// Configure Falling and Rising Trigger Mode -> FTSR & RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// Configure the GPIO Port Selection in SYSCFG_EXTICR
		// (1) Enable the Clock

		uint8_t exti_cr = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t exti_pos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_CODE(pGPIOHandle->pGPIOx);
		SYS_PCLK_EN();
		SYSCFG->EXTICR[exti_cr] = (portcode << (exti_pos * 4));

		// Enable the EXTI Interrupt Delivery
		// The line corresponds to the pin number.
		EXTI->IMR |= (1 <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


	}

	// Configure the speed.
	temp_value = 0;
	temp_value = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp_value;

	// Configure the Pull-Up / Pull-Down
	temp_value = 0;
	temp_value = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPd << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp_value;

	// Configure the Output Type
	temp_value = 0;
	temp_value = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp_value;

	// Configure Alternate Functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// Need Modulo Operator since an array defines the high and low registers
		// There are 8 registers each in the High and Low, so to determine which set, we'll take the remainder
		uint8_t reg_val, shift_val;

		reg_val = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		shift_val = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8; // Provides the remainder and indicates the value for the shift

		pGPIOHandle->pGPIOx->AFR[reg_val] &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear 4 Bits
		pGPIOHandle->pGPIOx->AFR[reg_val] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * shift_val) );
	}



}
/******************************************************************************
 * @fn				GPIO_Reset
 *
 * @brief				Reset the GPIO to its initial settings
 *
 * @param[in] pGPIOx    GPIO Port Base Address
 *
 * @return				None
 *
 * @Note
 */
void GPIO_Reset(GPIO_RegDef_t *pGPIOx)
{

	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}


}

/******************************************************************************
 * @fn				GPIO_ReadPin
 *
 * @brief			 	Read the value of the specified pin
 *
 * @param[in] pGPIOx 	GPIO Port Base Address
 * @param[in] PinNumber GPIO Pin
 *
 * @return			 	0 or 1
 *
 * @Note
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t pin_value;
	pin_value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001 );

	return pin_value;
}
/******************************************************************************
 * @fn				GPIO_ReadPort
 *
 * @brief			 	Read the value of the specified port
 *
 * @param[in] pGPIOx 	GPIO Port Base Address
 *
 * @return			 	0x0000 - 0xFFFF
 *
 * @Note
 */
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t port_value;
	port_value = (uint16_t)(pGPIOx->IDR);

	return port_value;
}

/******************************************************************************
 * @fn				GPIO_WritePin
 *
 * @brief			 	Write the value of the specified pin
 *
 * @param[in] pGPIOx 	GPIO Port Base Address
 * @param[in] PinNumber	GPIO Pin
 * @param[in] value		Write Value for Pin (0 or 1)
 *
 * @return			 	None
 *
 * @Note
 */
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET) {
		// Write 1 to the output data register of the corresponding Port & Pin
		pGPIOx->ODR |= ( 1 << PinNumber );
	}else
	{
		pGPIOx->ODR &= ~( 1 << PinNumber) ;
	}
}

/******************************************************************************
 * @fn				GPIO_WritePort
 *
 * @brief			 	Write the value to the specified port
 *
 * @param[in] pGPIOx 	GPIO Port Base Address
 * @param[in] value		Write Value for Pin (0 or 1)
 *
 * @return			 	None
 *
 * @Note
 */
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
	pGPIOx->ODR = value;
}

/******************************************************************************
 * @fn				GPIO_TogglePin
 *
 * @brief			 	Invert the state of the pin
 *
 * @param[in] pGPIOx 	GPIO Port Base Address
 * @param[in] PinNumber	GPIO Pin
 *
 * @return			 	None
 *
 * @Note
 */

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ & ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_or_disable)
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
			// Program ISER0 Register
			*NVIC_ICER0 |= ( 1 << IRQNumber);
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program ICER2 Register
		}

	}

}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// Identify IPR Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;


	// Lower 4 bits of the register are not implemented, so we need to shift by implemented bits (architecture specific)
	uint8_t shift_amount = (8 * iprx_section) + (8 - PRIORITY_BITS_IMPLEMENTED);


	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount );
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the EXTI Pending Register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber)) {
		// Clear the bit
		EXTI->PR |= (1 << PinNumber);
	}

}
