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
	I2C_RegDef_t    *pI2Cx;       	/*!< This holds the base address of I2Cx peripheral >*/
    I2C_Config_t    I2C_Config;     /*!< This holds I2Cx configuration settings >*/
    uint8_t 		*pTxBuffer;     /*!< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	    /*!< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		    /*!< To store Tx len > */
	uint32_t 		RxLen;		    /*!< To store Rx len > */
	uint8_t 		TxRxState;	    /*!< To store Communication state > */
	uint8_t 		DevAddr;	    /*!< To store slave/device address > */
    uint8_t         Mode;           /*!< To store current mode: master or slave  > */
    uint32_t        RxSize;         /*!< To store Rx size  > */
    uint8_t         Sr;			    /*!< To store repeated start value  > */
}I2C_Handle_t;

/*
 * I2C application states
 */
#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2

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

/*
 * I2C status flags definitions
 */
#define I2C_FLAG_TXE        (1 << I2C_ISR_TXE)
#define I2C_FLAG_TXIS       (1 << I2C_ISR_TXIS)
#define I2C_FLAG_RXNE       (1 << I2C_ISR_RXNE)
#define I2C_FLAG_ADDR 		(1 << I2C_ISR_ADDR)
#define I2C_FLAG_NACKF      (1 << I2C_ISR_NACKF)
#define I2C_FLAG_STOPF 		(1 << I2C_ISR_STOPF)
#define I2C_FLAG_TC  		(1 << I2C_ISR_TC)
#define I2C_FLAG_TCR  		(1 << I2C_ISR_TCR)
#define I2C_FLAG_BERR 		(1 << I2C_ISR_BERR)
#define I2C_FLAG_ARLO 		(1 << I2C_ISR_ARLO)
#define I2C_FLAG_OVR  		(1 << I2C_ISR_OVR)
#define I2C_FLAG_TIMEOUT 	(1 << I2C_ISR_TIMEOUT)
#define I2C_FLAG_BUSY       (1 << I2C_ISR_BUSY)
#define I2C_FLAG_DIR       	(1 << I2C_ISR_DIR)

/**
 * Repeated start Enable/Disable
 */
#define I2C_DISABLE_SR           0
#define I2C_ENABLE_SR            1

/*
 * I2C application events macros
 */
#define I2C_EV_TX_CMPLT         0
#define I2C_EV_RX_CMPLT         1
#define I2C_EV_STOP             2
#define I2C_EV_NACK             3
#define I2C_EV_DATA_REQ         4
#define I2C_EV_DATA_RCV         5

/*
 * brief I2C error flags macros
 */
#define I2C_ERROR_BERR      0
#define I2C_ERROR_ARLO      1
#define I2C_ERROR_OVR       2
#define I2C_ERROR_TIMEOUT   3

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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

/*
 * Interrupt based data send and receive
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
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
