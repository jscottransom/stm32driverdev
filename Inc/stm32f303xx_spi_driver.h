/*
 * stm32f303xx_spi_driver.h
 *
 *  Created on: May 4, 2022
 *      Author: jscoran
 */

#ifndef INC_STM32F303XX_SPI_DRIVER_H_
#define INC_STM32F303XX_SPI_DRIVER_H_


#include "stm32f303xx.h"


/*
 * Config structure for SPI Peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


typedef struct
{
	/* Pointer to hold the base address of the GPIO Peripheral for the specified Pin
	the user wants to control
	*/
	SPI_RegDef_t *pSPIx;		// Base address of the SPIx Peripherals

	SPI_Config_t SPIConfig;		// Structure which holds SPI Configuration settings

	uint8_t 	 *pTxBuffer;	// Tx Buffer Address
	uint8_t		 *pRxBuffer;	// Rx Buffer Address
	uint32_t	 TxLen;			// Tx Length
	uint32_t	 RxLen; 		// Rx Length
	uint8_t		 TxState;		// State of Transmission
	uint8_t 	 RxState;		// State of Receiving

}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD			1
#define SPI_BUS_CONFIG_HD			2
#define SPI_BUS_CONFIG_SIMPLEX_TX	3
#define SPI_BUS_CONFIG_SIMPLEX_RX	4

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW	0
#define SPI_CPOL_HIGH	1


/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW	0
#define SPI_CPHA_HIGH	1


/*
 * @SPI_SSM
 */
#define SPI_SSM_DIS			0
#define SPI_SSM_EN			1

/*
 * SPI Related Status Flags Definitions (for reading the value of the given bit)
 */
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1 << SPI_SR_BSY)

/*
 * SPI Application States
 */
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

/*
 * Possible SPI App events
 */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4

/*
 * Clock Control
 */
void SPI_PeripheralClockControl(SPI_RegDef_t *pSPIx, uint8_t enable_disable);


/*
 * Port Initialization and Reset
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_Reset(SPI_RegDef_t *pSPIx);



/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendDataInt(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataInt(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ & ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_or_disable);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQ_Priority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other Peripheral API
 */

/*
 * SPI Peripheral Enable
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t enable_or_disable);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enable_or_disable);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t enable_or_disable);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

/*
 * Transmission
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application Callback
 */
void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t app_event);


#endif /* INC_STM32F303XX_SPI_DRIVER_H_ */
