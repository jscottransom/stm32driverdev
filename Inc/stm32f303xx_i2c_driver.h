/*
 * stm32f303xx_i2c_driver.h
 *
 *  Created on: May 18, 2022
 *      Author: jscoran
 */

#ifndef INC_STM32F303XX_I2C_DRIVER_H_
#define INC_STM32F303XX_I2C_DRIVER_H_

#include "stm32f303xx.h"

/*
 * Config structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t 	I2C_Timing;
	uint8_t		I2C_OwnAddress;
	uint8_t		I2C_ACKControl;
	uint8_t		I2C_AnalogOff;
	uint8_t		I2C_DigitalFilter;
	uint8_t		I2C_NoStretch;


}I2C_Config_t;

/*
 * Handle Structure for I2Cx Peripheral
 */
typedef struct
{
	I2C_RegDef_t	*pI2Cx;
	I2C_Config_t 	I2CConfig;
}I2C_Handle_t;

/*
 * @I2C_TIMING
 */

#define I2C_TIMING_4_SM_LOW		 0x004091F3
#define I2C_TIMING_4_SM_HIGH	 0x00400D10
#define I2C_TIMING_4_FM			 0x00100002
#define I2C_TIMING_4_FM_PLUS	 0x00000001
#define I2C_TIMING_8_SM_LOW		 0x1042C3C7
#define I2C_TIMING_8_SM_HIGH 	 0x10420F13
#define I2C_TIMING_8_FM	 		 0x00310309
#define I2C_TIMING_8_FM_PLUS 	 0x00100306
#define I2C_TIMING_16_SM_LOW	 0x3042C3C7
#define I2C_TIMING_16_SM_HIGH	 0x30420F13
#define I2C_TIMING_16_FM	 	 0x10320309
#define I2C_TIMING_16_FM_PLUS	 0x00200204
#define I2C_TIMING_48_SM_LOW	 0xB042C3C7
#define I2C_TIMING_48_SM_HIGH	 0xB0420F13
#define I2C_TIMING_48_FM	 	 0x50330309
#define I2C_TIMING_48_FM_PLUS	 0x50100103
#define I2C_TIMING_54_SM_LOW	 0xD0417BFF
#define I2C_TIMING_54_SM_HIGH	 0x40D32A31
#define I2C_TIMING_54_FM		 0x10A60D20
#define I2C_TIMING_54_FM_PLUS	 0x00900916

/*
 * @I2c_AckControl
 */

#define I2C_ACK_ENABLE			0
#define I2C_ACK_DISABLE			1

/*
 * I2C Related Status Flags Definitions (for reading the value of the given bit)
 */
#define I2C_FLAG_TXE		(1 << I2C_SR_TXE)
#define I2C_FLAG_TXIS		(1 << I2C_SR_TXIS)
#define I2C_FLAG_RXNE		(1 << I2C_SR_RXNE)
#define I2C_FLAG_ADDR 		(1 << I2C_SR_ADDR)
#define I2C_FLAG_BERR		(1 << I2C_SR_BERR)
#define I2C_FLAG_TCR		(1 << I2C_SR_TCR)
#define I2C_FLAG_NACKF		(1 << I2C_SR_NACKF)
#define I2C_FLAG_ARLO		(1 << I2C_SR_ARLO)
#define I2C_FLAG_ARLO		(1 << I2C_SR_OVR)
#define I2C_FLAG_TIMEOUT	(1 << I2C_SR_TIMEOUT)
#define I2C_FLAG_STOPF		(1 << I2C_SR_STOPF)
#define I2C_FLAG_DIR		(1 << I2C_SR_DIR)
#define I2C_FLAG_BUSY		(1 << I2C_SR_BUSY)


/*
 * Start and Stop Modes
 */
#define I2C_NO_START_STOP			(0x00000000)
#define I2C_GENERATE_START_READ		( (1 << I2C_CR2_START) | (1 << I2C_CR2_RD_WRN) )
#define I2C_GENERATE_START_WRITE	( (1 << I2C_CR2_START) )
#define I2C_GENERATE_STOP			( (1 << I2C_CR2_STOP ) )

/*
* Auto or Reload Mode
*/
#define AUTOMODE		   (1 << I2C_CR2_AUTOEND)
#define RELOADMODE		   (1 << I2C_CR2_RELOAD)



/*
 * Max Transmission Size
 *
 */
#define MAX_BYTES			255


/*
 * Clock Control
 */
void I2C_PeripheralClockControl(I2C_RegDef_t *pI2Cx, uint8_t enable_disable);


/*
 * Port Initialization and Reset
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_Reset(I2C_RegDef_t *pI2Cx);


/*
 * IRQ & ISR Handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_or_disable);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQ_Priority);

/*
 * Other Peripheral API
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint32_t bytes, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint32_t bytes, uint8_t SlaveAddr);

void I2C_SlaveSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_SlaveReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr)


/*
 * SPI Peripheral Enable
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t enable_or_disable);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/*
 * Application Callback
 */
void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t app_event);



#endif /* INC_STM32F303XX_I2C_DRIVER_H_ */
