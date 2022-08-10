/*
 * 001LedToggle.c
 *
 *  Created on: Mar 23, 2022
 *      Author: jscoran
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32f303xx.h"


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i++);
}



int main(void)
{
	GPIO_Handle_t GpioLED;
	GPIO_Handle_t GpioLED2;
	GPIO_Handle_t GpioSensor;

	GpioLED.pGPIOx = GPIOE;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15 ;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLED.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;


	GpioLED2.pGPIOx = GPIOE;
	GpioLED2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14 ;
	GpioLED2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLED2.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;
	GpioLED2.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;

	GPIO_PeripheralClockControl(GPIOE, ENABLE);

	GpioSensor.pGPIOx = GPIOD;
	GpioSensor.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3 ;
	GpioSensor.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioSensor.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioSensor.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;

	GPIO_PeripheralClockControl(GPIOD, ENABLE);


	GPIO_Init(&GpioLED);
	GPIO_Init(&GpioLED2);
	GPIO_Init(&GpioSensor);
	while(1)
	{


			if(GPIO_ReadPin(GPIOD, 8) == 1) {
				delay();
				GPIO_TogglePin(GPIOE, 15);

			} else
			{
				delay();
				GPIO_TogglePin(GPIOE, 14);
			}



	}



}


