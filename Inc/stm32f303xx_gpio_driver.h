/*
 * stm32f303xx_gpio_driver.h
 *
 *  Created on: Mar 21, 2022
 *      Author: jscoran
 */

#ifndef INC_STM32F303XX_GPIO_DRIVER_H_
#define INC_STM32F303XX_GPIO_DRIVER_H_

#include "stm32f303xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;			/* POSSIBLE VALUES: @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;			/* POSSIBLE VALUES: @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;			/* POSSIBLE VALUES: @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPd;			/* POSSIBLE VALUES: @GPIO_PU_PD */
	uint8_t GPIO_PinOType;			/* POSSIBLE VALUES: @GPIO_PIN_OUTPUT_TYPES */
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


typedef struct
{
	/* Pointer to hold the base address of the GPIO Peripheral for the specified Pin
	the user wants to control
	*/
	GPIO_RegDef_t *pGPIOx;

	GPIO_PinConfig_t GPIO_PinConfig;		// Structure which holds Pin Configuration settings

}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO Pin Numbers
 */

#define GPIO_PIN_0 0
#define GPIO_PIN_1 1
#define GPIO_PIN_2 2
#define GPIO_PIN_3 3
#define GPIO_PIN_4 4
#define GPIO_PIN_5 5
#define GPIO_PIN_6 6
#define GPIO_PIN_7 7
#define GPIO_PIN_8 8
#define GPIO_PIN_9 9
#define GPIO_PIN_10 10
#define GPIO_PIN_11 11
#define GPIO_PIN_12 12
#define GPIO_PIN_13 13
#define GPIO_PIN_14 14
#define GPIO_PIN_15 15

/*
 * @GPIO_PIN_MODES
 * Possible GPIO Modes
 */
#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_ALTFN 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IN_FALLING 4
#define GPIO_MODE_IN_RISING 5
#define GPIO_MODE_IN_FALL_RISE_TRIG 6

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * Possible GPIO Output Types
 */
#define GPIO_OTYPE_PP 0
#define GPIO_OTYPE_OD 1

/*
 * @GPIO_PIN_SPEED
 * Possible GPIO SPEED
 */
#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_HIGH 2

/*
 * @GPIO_PU_PD
 * Possible GPIO Pull-Up / Pull-Down Configuration
 */
#define GPIO_NO_PUPD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2

/********************************************************************
 *					APIs supported by this driver 					*
 ********************************************************************/


/*
 * Clock Control
 */
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enable_disable);

/*
 * Port Initialization and Reset
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Reset(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read-Write
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ & ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_or_disable);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQ_Priority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F303XX_GPIO_DRIVER_H_ */
