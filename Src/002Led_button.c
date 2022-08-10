/*
 * 001LedToggle.c
 *
 *  Created on: Mar 23, 2022
 *      Author: jscoran
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32f303xx.h"

#define HIGH 1
#define BTN_PRESSED HIGH

void delay(void)
{
	for(uint32_t i = 0 ; i < 250000 ; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLED;
	GPIO_Handle_t GpioBtn;

	GpioLED.pGPIOx = GPIOE;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15 ;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLED.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;

	GPIO_PeripheralClockControl(GPIOE, ENABLE);

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0 ;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;

	GPIO_PeripheralClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioBtn);
	GPIO_Init(&GpioLED);
	while(1)
	{
		if(GPIO_ReadPin(GPIOA, 0) == BTN_PRESSED)
		{
			// Button Debouncing
			delay();
			GPIO_TogglePin(GPIOE, 15);

		}
	}



	}
