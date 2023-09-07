/*
 * STM32F767ZI_spi_driver.h
 *
 *  Created on: Sep 7, 2023
 *      Author: rahim
 */

#ifndef INC_STM32F767ZI_SPI_DRIVER_H_
#define INC_STM32F767ZI_SPI_DRIVER_H_
#include "STM32F767ZI.h"

/*
 * Configuration structure for SPIx peripheral
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

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx; /*!< This holds the base address of SPIx(X:0,1,2) peripherals>*/
	SPI_Config_t SPIConfig;
}SPI_Handle_t;

/**********************************************************************************
 * 							APIs supported by this driver
 * 		For more information about the API's check the function definitions
 *********************************************************************************/
/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */


#endif /* INC_STM32F767ZI_SPI_DRIVER_H_ */
