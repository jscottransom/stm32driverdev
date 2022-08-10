/*
 * 001LedToggle.c
 *
 *  Created on: Mar 23, 2022
 *      Author: jscoran
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f303xx.h"


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i++);
}



int main(void)
{
	GPIO_Handle_t GpioButton, GpioLED;
	// "Clear" the values in the fields
	memset(&GpioButton, 0, sizeof(GpioButton));
	memset(&GpioLED, 0, sizeof(GpioLED));


	GpioLED.pGPIOx = GPIOE;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15 ;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLED.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;

	GPIO_PeripheralClockControl(GPIOE, ENABLE);

	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0 ;
	// Since we're using this as an interrupt, MODE should be interrupt
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_RISING;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;

	GPIO_PeripheralClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLED);
	GPIO_Init(&GpioButton);

	//IRQ Configuration
	// This tells the interrupt to send the signal over the EXTI 0 Line (since that's where PA0 is)
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	while(1);
}
	// Implement the ISR. Must collect from the startup file

void EXTI0_IRQHandler(void)
{
	// Button debouncing, ~ 200ms
	delay();

	// "Handle" the interrupt, Clear the pending bit
	GPIO_IRQHandling(GPIO_PIN_0);

	// Toggle the GPIO
	GPIO_TogglePin(GPIOE, 15);
}






