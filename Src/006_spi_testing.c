/*
 * 006_spi_testing.c
 *
 *  Created on: May 10, 2022
 *      Author: jscoran
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f303xx.h"

// PB15 - SPI2_MOSI
// PB14 - MISO
// PB13 - SCK
// PB12 - NSS
// Alternate Function Mode 5


void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// SPI2_SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	// SPI2_MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);

	// SPI2_MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIPins);

	// SPI2_NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	// Maximum speed is the "minimum value / default of the prescaler"
	// Generates a clock of 8Mhz
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);

}


int main(void) {

	// User Buffer
	char user_data[] = "hello world";

	// Initialize specific GPIO Pins for specific SPI purposes
	SPI2_GPIOInits();

	// Initialize SPI Configurations
	SPI2_Inits();

	// Avoid MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	// Configurations must take place while the peripheral is disabled.
	// Now, we will enable the peripheral.
	SPI_PeripheralControl(SPI2, ENABLE);

	// Send Data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);
}
