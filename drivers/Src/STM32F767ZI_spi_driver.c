/*
 * STM32F767ZI_spi_driver.c
 *
 *  Created on: Sep 7, 2023
 *      Author: rahim
 */

#include "STM32F767ZI_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_over_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*
 * Peripheral Clock Setup
 */

/****************************************************************
 * @fn					- SPI_PeriClockControl
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
 * @fn					- SPI_DeInit
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
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
	else if (pSPIx == SPI5)
	{
		SPI5_REG_RESET();
	}
	else if (pSPIx == SPI6)
	{
		SPI6_REG_RESET();
	}
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
			(uint16_t*)pTxBuffer++; // Move to the address of the next data bit to send
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
 * @fn					- SPI_ReceiveData
 *
 * @brief				-
 *
 * @param[in]			- Base address of the SPI peripheral
 * @param[in]			- Pointer to RX buffer
 * @param[in]			- Length of RX buffer
 *
 * @return				- none
 *
 * @Note 				-
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. Wait until RNXE is full
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit in CR1
		if ((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//1. Load the data from DR to RxBuffer
			*((uint16_t*)pRxBuffer) = pSPIx->DR ;
			Len -= 2;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			//8 bit DFF
			*pRxBuffer = pSPIx->DR;
			Len --;
			pRxBuffer++;
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

/****************************************************************
 * @fn					- SPI_SendDataIT
 *
 * @brief				-
 *
 * @param[in]			- Base address of the SPI peripheral
 * @param[in]			- Pointer to TX buffer
 * @param[in]			- Length of TX buffer
 *
 * @return				- none
 *
 * @Note 				- Interrupt based send data. This is a non-blocking API
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2. Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

/****************************************************************
 * @fn					- SPI_ReceiveDataIT
 *
 * @brief				-
 *
 * @param[in]			- Base address of the SPI peripheral
 * @param[in]			- Pointer to RX buffer
 * @param[in]			- Length of RX buffer
 *
 * @return				- none
 *
 * @Note 				- Interrupt based receive data. This is a non-blocking API
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX)
	{
		//1. Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2. Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//3. Enable the RXEIE control bit to get interrupt whenever RXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

/****************************************************************
 * @fn					- SPI_IRQPriorityConfig
 *
 * @brief				-
 *
 * @param[in]			- IRQNumber
 * @param[in]			- Enable or Disable
 *
 * @return				- none
 *
 * @Note 				- SPI IRQ configure priority function
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			//Program register ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) //32 to 63
		{
			//Program register ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}
		else if(IRQNumber >= 64 && IRQNumber < 96) //64 to 95
		{
			//Program register ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));

		}
		else if(IRQNumber >= 96 && IRQNumber < 128) //96 to 127
		{
			//Program register ISER3 register
			*NVIC_ISER3 |= (1 << (IRQNumber % 128));
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			//Program register ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) //32 to 63
		{
			//Program register ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		}
		else if(IRQNumber >= 64 && IRQNumber < 96) //64 to 95
		{
			//Program register ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));

		}
		else if(IRQNumber >= 96 && IRQNumber < 128) //96 to 127
		{
			//Program register ICER3 register
			*NVIC_ICER3 |= (1 << (IRQNumber % 128));
		}
	}
}


/****************************************************************
 * @fn					- SPI_IRQPriorityConfig
 *
 * @brief				-
 *
 * @param[in]			- IRQNumber
 * @param[in]			- Enable or Disable
 *
 * @return				- none
 *
 * @Note 				- SPI IRQ configure priority function
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber % 4;

	// !! 16 programmable priority levels (4 bits of interrupt priority are used) Cf. RM 10.1
	// only 4 MSBits are implemented!

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/****************************************************************
 * @fn					- SPI_IRQHandling
 *
 * @brief				-
 *
 * @param[in]			- Pointer to the SPI handle
 *
 * @return				- none
 *
 * @Note 				- SPI IRQ handling function
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	//First lets check for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//Check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		//handle RXE
		spi_rxe_interrupt_handle(pSPIHandle);
	}

	//Check for overrun error
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		//handle overrun error
		spi_over_err_interrupt_handle(pSPIHandle);
	}
}


/****************************************************************
 * @fn					- SPI_SSIConfig
 *
 * @brief				-
 *
 * @param[in]			- Base address of the SPI peripheral
 * @param[in]			- Enable or Disable
 *
 * @return				- none
 *
 * @Note 				- This bit has an effect only when the SSM bit is set. The value of this bit is forced onto the NSS pin and the I/O value of the NSS pin is ignored.
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/****************************************************************
 * @fn					- SPI_SSOEConfig
 *
 * @brief				-
 *
 * @param[in]			- Base address of the SPI peripheral
 * @param[in]			- Enable or Disable
 *
 * @return				- none
 *
 * @Note 				-
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

// Additional helper functions implementation
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Check the DFF bit in CR1
	if ((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		//16 bit DFF
		//1. Load the data into the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2;
		(uint16_t*)pSPIHandle->pTxBuffer++; // Move to the address of the next data bit to send
	}
	else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen --;
		pSPIHandle->pTxBuffer++;
	}

	if (pSPIHandle->TxLen == 0)
	{
		//TxLen is zero, so close the SPI transmission and inform the application that TX is over
		//Prevents interrupts from setting up TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Check the DFF bit in CR1
	if ((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		//16 bit DFF
		//1. Load the data into the DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		(uint16_t*)pSPIHandle->pRxBuffer++; // Move to the address of the next data bit to send
	}
	else
	{
		//8 bit DFF
		*pSPIHandle->pRxBuffer = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen --;
		pSPIHandle->pRxBuffer++;
	}

	if (pSPIHandle->RxLen == 0)
	{
		//TxLen is zero, so close the SPI transmission and inform the application that TX is over
		//Prevents interrupts from setting up TXE flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_over_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFLag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//This is a weak implementation. The application may override this function
}
