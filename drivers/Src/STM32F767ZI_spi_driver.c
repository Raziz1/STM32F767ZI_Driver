/*
 * STM32F767ZI_spi_driver.c
 *
 *  Created on: Sep 7, 2023
 *      Author: rahim
 */

#include "STM32F767ZI_spi_driver.h"

/*
 * Peripheral Clock Setup
 */

/****************************************************************
 * @fn					- GPIO_PeriClockControl
 *
 * @brief				- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]			- Base address of the GPIO peripheral
 * @param[in]			- ENABLE or DISABLE macros
 * @param[in]
 *
 * @return				- none
 *
 * @Note 				- none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
		else if (pSPIx == SPI5)
		{
			SPI5_PCLK_EN();
		}
		else if (pSPIx == SPI6)
		{
			SPI6_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
		else if (pSPIx == SPI5)
		{
			SPI5_PCLK_DI();
		}
		else if (pSPIx == SPI6)
		{
			SPI6_PCLK_DI();
		}
	}
}

/****************************************************************
 * @fn					- SPI_Init
 *
 * @brief				-
 *
 * @param[in]			- Base address of the SPI peripheral
 * @param[in]
 * @param[in]
 *
 * @return				- none
 *
 * @Note 				- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//First lets configure the SPI_CR1 register
	uint32_t tempreg = 0;

	//Enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//2. Configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1 << 15);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI mode should be set
		tempreg |= (1 << 15);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1 << 15);
		//TXONLY but must be set
		tempreg |= (1 << 10);

	}

	//3. Configure the SPI serial clock speed (Baud Rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

	//4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	//5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

	//6. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/****************************************************************
 * @fn					- SPI_SendData
 *
 * @brief				-
 *
 * @param[in]			- Base address of the SPI peripheral
 * @param[in]			- Pointer to TX buffer
 * @param[in]			- Length of TX buffer
 *
 * @return				- none
 *
 * @Note 				- Blocking API because it waits until all bytes are transmitted
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit in CR1
		if ((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//1. Load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len -= 2;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len --;
			pTxBuffer++;
		}
	}
}

/****************************************************************
 * @fn					- SPI_PeripheralControl
 *
 * @brief				-
 *
 * @param[in]			- Base address of the SPI peripheral
 * @param[in]			- Enable or Disable
 *
 * @return				- none
 *
 * @Note 				- Must configure SPI parameters before enabling
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
