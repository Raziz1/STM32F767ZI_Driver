/*
 * STM32F767ZI_i2c_driver.h
 *
 *  Created on: Sep 14, 2023
 *      Author: rahim
 */

#ifndef INC_STM32F767ZI_I2C_DRIVER_H_
#define INC_STM32F767ZI_I2C_DRIVER_H_
#include "STM32F767ZI.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	//uint8_t I2C_ACKControl; This bit does not exist on STM32F767 registers
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx; /*!< This holds the base address of I2Cx(X:0,1,2) peripherals>*/
	I2C_Config_t I2C_Config;
    uint8_t         Mode;           /*!< To store current mode: master or slave  > */
    uint32_t        RxSize;         /*!< To store Rx size  > */
    uint8_t         Sr;			    /*!< To store repeated start value  > */
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM10K         0
#define I2C_SCL_SPEED_SM100K        1
#define I2C_SCL_SPEED_FM            2
#define I2C_SCL_SPEED_FMPLUS        3

/*
 * @I2C_ACKControl
 * This bit doesn't exist on STM32F767 registers
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * I2C modes
 */
#define I2C_MODE_SLAVE              0
#define I2C_MODE_MASTER             1

/**********************************************************************************
 * 							APIs supported by this driver
 * 		For more information about the API's check the function definitions
 *********************************************************************************/
/*
 * Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and receive
 */


/*
 * Interrupt based data send and receive
 */

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus (I2C_RegDef_t *pI2Cx, uint32_t FlagName);
/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F767ZI_I2C_DRIVER_H_ */
